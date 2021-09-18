#include "rigcontrol.h"

boolean RigControl::bt_connected;
BluetoothSerial RigControl::SerialBT;
char RigControl::btname[64];

uint8_t PLAsyncOper::OnInit(ACM *pacm)
{
    uint8_t rcode;

    // Set DTR = 1
    rcode = pacm->SetControlLineState(1);

    if (rcode)
    {
        ErrorMessage<uint8_t>(PSTR("SetControlLineState"), rcode);
        return rcode;
    }

    LINE_CODING lc;
    lc.dwDTERate = 4800;
    lc.bCharFormat = 2;
    lc.bParityType = 0;
    lc.bDataBits = 8;

    rcode = pacm->SetLineCoding(&lc);

    if (rcode)
    {
        ErrorMessage<uint8_t>(PSTR("SetLineCoding"), rcode);
    }

    return rcode;
};

void RigControl::setConfig(boolean serverp, DynamicJsonDocument &config)
{
    String prefix;
    String ch;
    char conf[128];

    strlcpy(btname, config["keyername"], sizeof(btname));

    if (serverp)
        prefix = "sv-";
    else
        prefix = "cl-";

    for (int i = 0; i < num_of_channel; i++)
    {
        ch = prefix + "ch" + String(i + 1) + "-";

        strlcpy(conf, config[ch + "media"], sizeof(conf));
        if (strcmp(conf, "BT") == 0)
        {
            media[i] = IF_BLUETOOTH;
        }
        else if (strcmp(conf, "USB") == 0)
        {
            if (serverp)
                media[i] = IF_USBH;
            else
                media[i] = IF_USB;
        }
        else
            media[i] = IF_USB;

        strlcpy(conf, config[ch + "proto"], sizeof(conf));
        if (strcmp(conf, "CAT") == 0)
            proto[i] = PROTO_CAT;
        else if (strcmp(conf, "CIV") == 0)
            proto[i] = PROTO_CIV;
        else
            proto[i] = NONE;

        baudrate[i] = config[ch + "baudrate"];
    }
    serial_setup();
}

void RigControl::serial_setup(void)
{
    int sbaud = 115200;
    boolean enable_serial = false, enable_bt = false,
            enable_hostadpt = false;

    for (int i = 0; i < num_of_channel; i++)
    {
        switch (media[i])
        {
        case IF_USB:
            sbaud = baudrate[i];
            enable_serial = true;
            break;
        case IF_USBH:
            sbaud = baudrate[i];
            enable_hostadpt = true;
            break;
        case IF_BLUETOOTH:
            enable_bt = true;
            break;
        }
    }

    if (enable_bt)
    {
        if (!bt_started)
        {
            bt_connected = false;
            bt_started = true;
            SerialBT.register_callback(handleBT);
            SerialBT.begin(btname, false);
        }
    }
    else if (bt_connected)
    {
        SerialBT.disconnect();
        SerialBT.end();
        bt_started = false;
    }

    if (enable_serial)
    {
        Serial.flush();
        Serial.end();
        Serial.begin(sbaud);
    }

    if (enable_hostadpt)
    {
        if (!hostadpt_started && Usb.Init() == -1)
        {
            hostadpt_started = false;
        }
        else
            hostadpt_started = true;

        if (hostadpt_started)
        {
            LINE_CODING lc;
            lc.dwDTERate = sbaud;
            lc.bCharFormat = 2; /* two stop bit */
            lc.bParityType = 0; /* parity none */
            lc.bDataBits = 8;
            Pl.SetLineCoding(&lc);
        }
    }
}

boolean RigControl::available(uint8_t channel)
{
    uint8_t c, delim;

    if (channel > num_of_channel - 1)
        return false;

    bready[channel] = false;

    switch (proto[channel])
    {
    case PROTO_CAT:
        delim = 0x3b; // 0x3b = ';'
        break;
    case PROTO_CIV:
        delim = 0xfd;
        break;
    default:
        delim = 0x0;
        break;
    }

    switch (media[channel])
    {
    case IF_USB:
        if (Serial.available())
        {
            c = Serial.read();
            buffer[channel][bptr[channel]++] = c;
            if (c == delim || bptr[channel] >= PKTBUFSIZ)
            {
                bsize[channel] = bptr[channel];
                bready[channel] = true;
                bptr[channel] = 0;
                return true;
            }
        }
        return false;
        break;

    case IF_USBH:
        if (hostadpt_started && Pl.isReady())
        {
            uint16_t rcvd, rcode;
            uint8_t tmpbuff[PKTBUFSIZ];

            rcvd = sizeof(buffer[channel]);
            rcode = Pl.RcvData(&rcvd, tmpbuff);
            if (rcode && rcode != hrNAK)
                return false;
            if (rcvd > 1)
            {
                memcpy(buffer[channel], tmpbuff, rcvd);
                bsize[channel] = rcvd;
                bready[channel] = true;
                return true;
            }
            else if (rcvd == 1)
            {
                c = tmpbuff[0];
                buffer[channel][bptr[channel]++] = c;
                if (c == delim || bptr[channel] >= PKTBUFSIZ)
                {
                    bsize[channel] = bptr[channel];
                    bready[channel] = true;
                    bptr[channel] = 0;
                    return true;
                }
            }
        }
        return false;
        break;

    case IF_BLUETOOTH:
        if (bt_connected && SerialBT.available())
        {
            c = SerialBT.read();
            buffer[channel][bptr[channel]++] = c;
            if (c == 0xfd || c == 0x3b || bptr[channel] >= PKTBUFSIZ)
            {
                bsize[channel] = bptr[channel];
                bready[channel] = true;
                bptr[channel] = 0;
                return true;
            }
        }
        return false;
        break;
    default:
        return false;
    }
}

uint16_t RigControl::fromRig(uint8_t channel, SerialData &data)
{
    if (!bready[channel])
    {
        data.size = 0;
        return 0;
    }
    memcpy(data.buffer, buffer[channel], bsize[channel]);

    memcpy(lastmsg, buffer[channel], bsize[channel]);
    lastch = channel;
    lastsize = bsize[channel];
    hasnewmsg = true;

    data.channel = channel;
    data.size = bsize[channel];
    bready[channel] = false;
    bsize[channel] = 0;

    return data.size;
}

uint16_t RigControl::toRig(SerialData &data)
{
    if (data.channel > num_of_channel - 1)
        return 0;

    switch (media[data.channel])
    {
    case IF_USB:
        Serial.write(data.buffer, data.size);
        break;
    case IF_USBH:
        if (hostadpt_started)
            Pl.SndData(data.size, data.buffer);
        break;
    case IF_BLUETOOTH:
        if (bt_connected)
            SerialBT.write(data.buffer, data.size);
        break;
    default:
        return 0;
    }

    memcpy(lastmsg, data.buffer, data.size);
    lastch = data.channel;
    lastsize = data.size;
    hasnewmsg = true;

    return data.size;
}

boolean RigControl::receiving(void)
{
    if (hasnewmsg)
    {
        hasnewmsg = false;
        return true;
    }
    return false;
}

String RigControl::decodedMsg(void)
{
    String msg;
    uint8_t c;

    switch (media[lastch])
    {
    case IF_USB:
        msg = "USB";
        break;
    case IF_USBH:
        msg = "USBH";
        break;
    case IF_BLUETOOTH:
        msg = "BT";
        break;
    }

    c = lastmsg[0];
    if (c == 0xfe)
    {
        msg += "(CI-V):";
        msg += " TO:" + String(lastmsg[2], HEX);
        msg += " CMD:" + String(lastmsg[4], HEX);
        for (int i = 6; i < lastsize; i++)
            msg += " " + String(lastmsg[i], HEX);
    }
    else if (c >= 'A' || c <= 'Z')
    {
        msg += "(CAT):";
        for (int i = 0; i < lastsize; i++)
            msg += String((char)lastmsg[i]);
    }

    return msg;
}

boolean RigControl::catRead(int ch, const char *command, const char *arg, char *res)
{
    SerialData d;
    unsigned long now;

    sprintf((char *)d.buffer, "%s%s;", command, arg);
    d.size = strlen((char *)d.buffer);
    d.channel = ch;
    toRig(d);

    now = millis();
    while ((millis() - now) < 5000)
    {
        if (available(ch))
        {
            fromRig(ch, d);
            if (d.size > 2)
            {
                memcpy(res, &d.buffer[2], d.size - 3);
                res[d.size - 3] = '\0';
                return true;
            }
            else
            {
                memcpy(res, d.buffer, d.size);
                res[d.size] = '\0';
                return false;
            }
        }
    }
    strcpy(res, "Timeout");
    return false;
}

boolean RigControl::catSet(int ch, const char *command, const char *arg)
{
    SerialData d;
    unsigned long now;

    sprintf((char *)d.buffer, "%s%s;", command, arg);
    d.size = strlen((char *)d.buffer);
    d.channel = ch;
    toRig(d);

    now = millis();
    while (millis() - now < 100)
    {
        if (available(ch))
        {
            fromRig(ch, d);
            if (strncmp("?;", (char *)d.buffer, 2) == 0)
                return false;
            return true;
        }
    }
    return true;
}

boolean RigControl::civRead(int ch, uint16_t command, char *res)
{
    SerialData d;
    unsigned long now;

    d.buffer[0] = 0xfe;
    d.buffer[1] = 0xfe;
    d.buffer[2] = civ_addr_to;
    d.buffer[3] = civ_addr_from;
    d.buffer[4] = (command & 0xff00) >> 8;
    d.buffer[5] = (uint8_t)(command & 0x00ff);
    d.buffer[6] = 0xfd;

    d.size = 7;
    d.channel = ch;
    toRig(d);

    now = millis();
    while ((millis() - now) < 5000)
    {
        if (available(ch))
        {
            fromRig(ch, d);
            if (d.buffer[d.size - 1] == 0xfd)
            {
                memcpy(res, &d.buffer[6], d.size - 6);
                res[d.size - 1] = '\0';
                return true;
            }
            else
            {
                memcpy(res, d.buffer, d.size);
                res[d.size] = '\0';
                return false;
            }
        }
    }
    strcpy(res, "Timeout");
    return false;
}

boolean RigControl::civSet(int ch, uint16_t command, uint8_t arg)
{
    SerialData d;
    unsigned long now;

    d.buffer[0] = 0xfe;
    d.buffer[1] = 0xfe;
    d.buffer[2] = civ_addr_to;
    d.buffer[3] = civ_addr_from;
    d.buffer[4] = (command & 0xff00) >> 8;
    d.buffer[5] = (uint8_t)(command & 0x00ff);
    d.buffer[6] = arg;
    d.buffer[7] = 0xfd;

    d.size = 8;
    d.channel = ch;
    toRig(d);

    now = millis();
    while (millis() - now < 100)
    {
        if (available(ch))
        {
            fromRig(ch, d);
            return true;
        }
    }
    return true;
}

boolean RigControl::startATU(int channel, SerialData &res)
{
    boolean result = true, tuneerror;
    char pwr[16], mode[16], swr[16];
    uint8_t state;
    unsigned long now;

    switch (proto[channel])
    {
    case PROTO_CAT:
        /* Save current power & mode */
        result = result && catRead(channel, "PC", "", pwr);
        result = result && catRead(channel, "MD", "0", mode);
        /* Set power to 10w and change RTTY mode */
        result = result && catSet(channel, "PC", "010");
        result = result && catSet(channel, "MD", "06");
        break;

    case PROTO_CIV:
        /* Save current power & mode */
        //result = result && civRead(channel, 0x140a, 0x1a);
        result = result && civRead(channel, 0x0400, mode);
        /* Set power to 10w and change RTTY mode */
        //result = result && civRead(channel, 0x140a, 0x1a);
        result = result && civSet(channel, 0x0200, 0x04);
        break;

    default:
        sprintf((char *)res.buffer, "NO ATU connected.");
        res.size = strlen((char *)res.buffer) + 1;
        res.channel = channel;
        return result;
        return false;
    }

    if (!result)
    {
        /* Something wrong. Stop tuning. */
        sprintf((char *)res.buffer, "Parameter read error");
        res.size = strlen((char *)res.buffer) + 1;
        res.channel = channel;
        return result;
    }

    /* Start AH-4 */
    switch (proto[channel])
    {
    case PROTO_CAT:
        result = result && catSet(channel, "TX", "1");
        break;
    case PROTO_CIV:
        result = result && civSet(channel, 0x1c00, 0x01);
    }

    delay(500);
    digitalWrite(ah4_start, HIGH);
    delay(1500);
    digitalWrite(ah4_start, LOW);
    delay(500);

    now = millis();
    do
    {
        delay(200);
        state = digitalRead(ah4_key);
    } while (state != HIGH && (millis() - now) < 10000);

    /*  Unable to tuning within 10 sec */
    if ((millis() - now) > 10000)
        tuneerror = true;
    else
        tuneerror = false;

    /* Stop transmission and restore params. */
    switch (proto[channel])
    {
    case PROTO_CAT:
        result = result && catRead(channel, "RM", "6", swr);
        result = result && catSet(channel, "TX", "0");
        result = result && catSet(channel, "MD", mode);
        result = result && catSet(channel, "PC", pwr);
        break;

    case PROTO_CIV:
        result = result && civRead(channel, 0x1512, swr);
        result = result && civSet(channel, 0x1c00, 0x00);
        result = result && civSet(channel, 0x0600, (uint8_t)mode[0]);
        //result = result && civSet(channel, 0x140a, pwr);
        break;
    }

    if (tuneerror || state == LOW)
    {
        sprintf((char *)res.buffer, "ERR K=%d T/O=%d", state, tuneerror);
        res.size = strlen((char *)res.buffer) + 1;
        res.channel = channel;
    }
    else
    {

        swr[4] = '\0';
        sprintf((char *)res.buffer, "DONE SWR=%s", &swr[1]);
        res.size = strlen((char *)res.buffer) + 1;
        res.channel = channel;
    }

    return tuneerror && result;
}