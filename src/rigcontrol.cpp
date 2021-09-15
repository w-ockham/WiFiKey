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
    lc.bCharFormat = 0;
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
        if (strcmp(conf, "%notchecked%") == 0)
        {
            media[i] = IF_BLUETOOTH;
        }
        else if (strcmp(conf, "%checked%") == 0)
        {
            if (serverp)
                media[i] = IF_USBH;
            else
                media[i] = IF_USB;
        }
        else
            media[i] = IF_USB;

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

        if (hostadpt_started) {
            LINE_CODING lc;
            lc.dwDTERate = sbaud;
            lc.bCharFormat = 0;
            lc.bParityType = 0;
            lc.bDataBits = 8;
            Pl.SetLineCoding(&lc);
        }
    }
}

boolean RigControl::available(uint8_t channel)
{
    uint8_t c;

    if (channel > num_of_channel - 1)
        return false;

    bready[channel] = false;

    switch (media[channel])
    {
    case IF_USB:
        if (Serial.available())
        {
            c = Serial.read();
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

    case IF_USBH:
        if (Pl.isReady())
        {
            uint16_t rcvd, rcode;
            rcvd = sizeof(buffer[channel]);
            rcode = Pl.RcvData(&rcvd, buffer[channel]);
            if (rcode && rcode != hrNAK)
                return false;
            if (rcvd)
            {
                bsize[channel] = rcvd;
                bready[channel] = true;
                return true;
            }
        }
        return false;
        break;

    case IF_BLUETOOTH:
        if (SerialBT.available())
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
        Pl.SndData(data.size, data.buffer);
        break;
    case IF_BLUETOOTH:
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
    if (hasnewmsg) {
        hasnewmsg = false;
        return true;
    }
    return false;
}

String RigControl::decodedMsg(void)
{
    String msg;
    uint8_t c;

    switch (media[lastch]) {
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
    } else if ( c >= 'A' || c <= 'Z') {
        msg += "(CAT):";
        for (int i = 0; i < lastsize; i++)
            msg += String((char)lastmsg[i]);
    }

    return msg;

}