/*
* Prolific USB serial converter
*/
#include <usbhub.h>
#include <cdcacm.h>
#include <cdcprolific.h>
#include <SPI.h>

class PLAsyncOper : public CDCAsyncOper
{
public:
    PLAsyncOper(int baudrate)
    {
        serialbaudrate = baudrate;
    }
    uint8_t OnInit(ACM *pacm);

private:
    int serialbaudrate;
};

uint8_t PLAsyncOper::OnInit(ACM *pacm)
{
    uint8_t rcode;

    // Set DTR = 1
    rcode = pacm->SetControlLineState(1);

    if (rcode) {
        ErrorMessage<uint8_t>(PSTR("SetControlLineState"), rcode);
        return rcode;
    }

    LINE_CODING lc;
    lc.dwDTERate  = serialbaudrate;
    lc.bCharFormat  = 0;
    lc.bParityType  = 0;
    lc.bDataBits  = 8;

    rcode = pacm->SetLineCoding(&lc);

    if (rcode) {
        ErrorMessage<uint8_t>(PSTR("SetLineCoding"), rcode);
    }

    return rcode;
}

USB     Usb;
PLAsyncOper  AsyncOper(9600);
PL2303       Pl(&Usb, &AsyncOper);
