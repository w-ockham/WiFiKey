/*
* Wifikey rigcontrol class
*/
#ifndef RIGCON_H
#define RIGCON_H

#include <usbhub.h>
#include <cdcacm.h>
#include <cdcprolific.h>
#include <SPI.h>
#include <BluetoothSerial.h>
#include "wifikey.h"
#include "config_settings.h"

class PLAsyncOper : public CDCAsyncOper
{
public:
    uint8_t OnInit(ACM *pacm);
};

class RigControl
{
    enum
    {
        IF_USB,      /* Onboard USB */
        IF_USBH,     /* HostAdapter */
        IF_BLUETOOTH /* Onboard Bluetooth */
    };
    static char btname[64];
    int num_of_channel;

    int ah4_channel;
    uint8_t ah4_start;
    uint8_t ah4_key;

    int media[RIG_CTRL_CHANNEL];
    int baudrate[RIG_CTRL_CHANNEL];
    int bsize[RIG_CTRL_CHANNEL];
    int bptr[RIG_CTRL_CHANNEL];
    boolean bready[RIG_CTRL_CHANNEL];
    uint8_t buffer[RIG_CTRL_CHANNEL][PKTBUFSIZ];
    boolean mready[RIG_CTRL_CHANNEL];

    uint8_t lastmsg[PKTBUFSIZ];
    int lastch, lastsize;
    boolean hasnewmsg;

    USB Usb;
    PLAsyncOper AsyncOper;
    PL2303 Pl;

    static BluetoothSerial SerialBT;
    static boolean bt_connected;
    boolean hostadpt_started, bt_started;
    void serial_setup(void);

public:
    RigControl(int numchan) : Pl(&Usb, &AsyncOper)
    {
        num_of_channel = numchan;

        bt_started = false;
        bt_connected = false;
        hostadpt_started = false;
        hasnewmsg = false;

        for (int i = 0; i < num_of_channel; i++)
        {
            bsize[i] = 0;
            bptr[i] = 0;
            bready[i] = false;
            mready[i] = false;
        }
    };

    void configAH4(int ch, uint8_t start, uint8_t key)
    {
        ah4_channel =ch;
        ah4_start = start;
        ah4_key = key;

        pinMode(ah4_start, OUTPUT);
        digitalWrite(ah4_start, LOW);
        pinMode(ah4_key, INPUT_PULLUP);
    }

    inline int numofChannel(void)
    {
        return num_of_channel;
    };

    static void handleBT(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
    {
        if (event == ESP_SPP_SRV_OPEN_EVT)
        {
            Serial.println("Bluetooth is connected.");
            bt_connected = true;
        }
        else if (event == ESP_SPP_CLOSE_EVT)
        {
            Serial.println("Bluetooth is disconnected.");
            bt_connected = false;
            SerialBT.begin(btname);
        }
    }

    void setConfig(boolean serverp, DynamicJsonDocument &config);
    boolean available(uint8_t channel);
    uint16_t fromRig(uint8_t channel, SerialData &data);
    uint16_t toRig(SerialData &data);
    boolean receiving(void);
    String decodedMsg(void);
    boolean startAH4(SerialData &);

    boolean catRead(int, const char *, const char *, char *);
    boolean catSet(int , const char *, const char *);

    inline void loop(void)
    {
        if (hostadpt_started)
            Usb.Task();
    }
};
#endif