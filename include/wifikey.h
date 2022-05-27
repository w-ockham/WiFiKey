#ifndef WIFIKEY
#define WIFIKEY

#define DEBUG_LEVEL 0
#define DROPPKT 0

/* Session Parameters */
#define WIFITIMEOUT 8000    /* WiFi AP connection time out */
#define KEEPALIVE 15000     /* Send keep-alive to server every 15sec */
#define AUTHTIMEOUT 5000    /* Authentication timeout (5sec) */
#define IDLETIMEOUT 1800000 /* Idle timeout (30min)  */

/* Queue params. */
#define SYMBOLWAIT 1           /* Wait one symbol */
#define QLATENCY 2             /* Process queued symbol if ((mark and space duriotn) * qlatency (msec) elapsed. */
#define MAX_MARK_DURATION 5000 /* Maximum MARK duration 5sec */
#define PKTBUFSIZ 128

/* GPIO setting */
#define MAX_KEY_CHANNEL 4

#ifdef M5ATOM
#define KEY_CHANNEL 1
const uint8_t gpio_button = 39;  /* Toggle channel */
const uint8_t gpio_key = 19;     /* from Keyer */
const uint8_t gpio_led = 27;     /* to LED */
const uint8_t gpio_out[] = {23}; /* to Photocoupler */
#else
#ifndef TUNE_ENCODER 
#define KEY_CHANNEL 2
const uint8_t gpio_button = 39;      /* Toggle channel */
const uint8_t gpio_key = 25;         /* from Keyer */
const uint8_t gpio_led = 26;         /* to LED */
const uint8_t gpio_out[] = {27, 32}; /* to Photocoupler */
const uint8_t gpio_atu_start = 33;   /* AH-4 START */
const uint8_t gpio_atu_key = 16;     /* AH-4 KEY */
#else
#define KEY_CHANNEL 1
const uint8_t gpio_key = 4;         /* from Keyer */
const uint8_t gpio_led = 16;         /* to LED */
const uint8_t gpio_out[] = {17}; /* to Photocoupler */
const uint8_t gpio_atu_start = 18;   /* AH-4 START */
const uint8_t gpio_atu_key = 19;    /* AH-4 KEY */

const uint8_t gpio_MAIN_a = 35;   /* Main encoder phase A inverted*/
const uint8_t gpio_MAIN_b = 34;   /* Main encoder phase B*/
const uint8_t gpio_SUB_a = 32;    /* Sub encoder phase A*/
const uint8_t gpio_SUB_b = 33;    /* Sub encoder phase B*/
const uint8_t gpio_MODE_a = 25;   /* Mode encodor phase A */
const uint8_t gpio_MODE_b = 26;   /* Mode encoder phase B */
const uint8_t gpio_BAND_a = 14;   /* Band encoder phase A inerted*/
const uint8_t gpio_BAND_b = 27;   /* Band encoder phase B */
const uint8_t gpio_button = 12;   /* ATU Start (Band encoder)*/
#endif
#endif

/* Rig Control Channel */
#define RIG_CTRL_CHANNEL 2

enum PktType
{
  PKT_SYN,
  PKT_RST,
  PKT_AUTH,
  PKT_KEEPALIVE,
  PKT_ACK,
  PKT_NACK,
  PKT_CODE,
  PKT_CODE_RESENT,
  PKT_CONF,
  PKT_SERIAL,
  PKT_MAIN_UP,
  PKT_MAIN_DOWN,
  PKT_SUB_UP,
  PKT_SUB_DOWN,
  PKT_BAND_UP,
  PKT_BAND_DOWN,
  PKT_MODE_UP,
  PKT_MODE_DOWN,
  PKT_MODE_TOGGLE,
  PKT_START_ATU,
  PKT_ATU_OK,
  PKT_ATU_FAIL
};

enum EdgeType
{
  RISE_EDGE = 1,
  FALL_EDGE
};

struct DotDash
{
  EdgeType data;
  unsigned long seq;
  unsigned long t;
  unsigned long d;
};

struct SerialData
{
  uint8_t channel;
  uint16_t size;
  uint8_t buffer[PKTBUFSIZ];
};

struct EncoderData
{
  uint8_t step;
};

struct KeyerPkt
{
  PktType type;
  uint8_t channel;
  union
  {
    uint8_t hash[16];
    DotDash data;
    SerialData sdata;
    EncoderData edata;
  };
};

/* Server state */
enum ServerState
{
  KEYER_WAIT,
  KEYER_AUTH,
  KEYER_ACTIVE
};

/* Key state */
enum KeyState
{
  NONE,
  MARK,
  SPACE
};

/* Keyer error code */
enum KeyError
{
  FAIL_NONE,
  FAIL_AUTHFAIL,
  FAIL_AUTHTIMEOUT,
  FAIL_RESETPEER,
  FAIL_DURATIONTOOLONG
};
#endif