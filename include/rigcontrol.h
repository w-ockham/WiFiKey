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
    static char btname[64];
    int num_of_channel;

    int current_band[RIG_CTRL_CHANNEL];
    typedef struct band_data {
        uint32_t freq;
        uint8_t mode;
        uint8_t channel;
    } band_data_t;
    band_data_t banddata[16];

    int ah4_channel;
    uint8_t ah4_start;
    uint8_t ah4_key;
    uint8_t civ_addr_from, civ_addr_to;

    int media[RIG_CTRL_CHANNEL];
    int proto[RIG_CTRL_CHANNEL];
    int baudrate[RIG_CTRL_CHANNEL];
    int bsize[RIG_CTRL_CHANNEL];
    int bptr[RIG_CTRL_CHANNEL];
    boolean bready[RIG_CTRL_CHANNEL];
    uint8_t buffer[RIG_CTRL_CHANNEL][PKTBUFSIZ];
    boolean mready[RIG_CTRL_CHANNEL];
    uint32_t rigfreq[RIG_CTRL_CHANNEL];

    int atutype[RIG_CTRL_CHANNEL];
    uint8_t civaddress[RIG_CTRL_CHANNEL];
    uint8_t pwrreduce[RIG_CTRL_CHANNEL];

    uint8_t lastmsg[PKTBUFSIZ];
    int lastch, lastsize;
    uint32_t lastfreq;
    boolean hasnewmsg;

    USB Usb;
    PLAsyncOper AsyncOper;
    PL2303 Pl;

    static BluetoothSerial SerialBT;
    static boolean bt_connected;
    boolean hostadpt_started, bt_started;
    void serial_setup(void);

public:
    enum
    {
        IF_USB,      /* Onboard USB */
        IF_USBH,     /* HostAdapter */
        IF_BLUETOOTH /* Onboard Bluetooth */
    };

    enum
    {
        PROTO_CAT,   /*  CAT */
        PROTO_CIV,   /* CI-V */
        PROTO_OTHER, /* No protocol */
    };

    enum
    {
        ATU_AH4,   /*  AH4 */
        ATU_OTHER, /*  OTHER */
        ATU_NONE   /*  NONE */
    };

    enum
    {
        ENC_UP,
        ENC_DOWN
    };
    
    enum 
    {
        B_80m, B_40m, B_30m, B_20m, B_17m, B_15m, B_13m, B_10m, B_6m,
        B_GEN, B_MW, B_2m, B_70cm, B_END 
    };

    enum
    {
        M_CW, M_USB, M_LSB, M_FM, M_AM, M_END
    };

    int tune_fine_step[5] = {10, 10, 10, 100, 100};
    int tune_coarse_step[5] = { 5000, 5000, 5000, 20000, 10000};

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
            
            switch(i)
            {
                case 0:
                    current_band[i] = B_20m;
                    break;
                case 1:
                    current_band[i] = B_70cm;
                    break;    
            }
        }

        civ_addr_from = 0x30;
        civ_addr_to = 0xa4;

        for(int i = B_80m; i < B_END; i++) {
            switch(i) {
                case B_80m:
                    banddata[i].freq = 3520000;
                    banddata[i].mode = M_CW;
                    banddata[i].channel = 0;
                    break;
                case B_40m:
                    banddata[i].freq = 7010000;
                    banddata[i].mode = M_CW;
                    banddata[i].channel = 0;
                    break;
                case B_30m:
                    banddata[i].freq = 10110000;
                    banddata[i].mode = M_CW;
                    banddata[i].channel = 0;
                    break;
                case B_20m:
                    banddata[i].freq = 14060000;
                    banddata[i].mode = M_CW;
                    banddata[i].channel = 0;
                    break;
                case B_17m:
                    banddata[i].freq = 1806800;
                    banddata[i].mode = M_CW;
                    banddata[i].channel = 0;
                    break;
                case B_15m:
                    banddata[i].freq = 21060000;
                    banddata[i].mode = M_CW;
                    banddata[i].channel = 0;
                    break;
                case B_13m:
                    banddata[i].freq = 24890000;
                    banddata[i].mode = M_CW;
                    banddata[i].channel = 0;
                    break;
                case B_10m:
                    banddata[i].freq = 28060000;
                    banddata[i].mode = M_CW;
                    banddata[i].channel = 0;
                    break;
                case B_6m:
                    banddata[i].freq = 50200000;
                    banddata[i].mode = M_CW;
                    banddata[i].channel = 0;
                    break;
                case B_2m:
                    banddata[i].freq = 144090000;
                    banddata[i].mode = M_CW;
                    banddata[i].channel = 1;
                    break;
                case B_70cm:
                    banddata[i].freq = 433000000;
                    banddata[i].mode = M_FM;
                    banddata[i].channel = 1;
                    break;
                case B_GEN:
                    banddata[i].freq = 9380000;
                    banddata[i].mode = M_AM;
                    banddata[i].channel = 0;
                    break;
                case B_MW:
                    banddata[i].freq = 810000;
                    banddata[i].mode = M_AM;
                    banddata[i].channel = 0;
                    break;
                default:
                    break;
            }
        }
    };

    void configAH4(uint8_t start, uint8_t key)
    {
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

    inline int redirect(int channel)
    {
        return banddata[current_band[channel]].channel;
    }

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
    uint16_t fromRig(uint8_t channel, SerialData &data, boolean storemsg = true);
    uint16_t toRig(SerialData &data, boolean storemsg = true);
    boolean receiving(void);
    String decodedMsg(void);
    boolean startATU(int, SerialData &);
    boolean catRead(int, const char *, const char *, char *);
    boolean catSet(int, const char *, const char *);
    boolean civRead(int, uint16_t, char *);
    boolean civSet(int, uint16_t, int, const char*);
    void civReadFreq(int, uint8_t []);
    void civSetFreq(int, int);
    void civSetFreq(int, uint32_t);
    int readSWR(int, int, float &);
    boolean encoderMain(uint8_t channel,int direction, int step);
    boolean encoderSub(uint8_t channel,int direction, int step);
    boolean encoderMode(uint8_t channel,int direction, int step);
    boolean encoderBand(uint8_t channel,int direction, int step);
    
    inline void loop(void)
    {
        if (hostadpt_started)
            Usb.Task();
    }
};
#endif