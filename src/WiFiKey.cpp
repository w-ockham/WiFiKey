#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebServer.h>
#include <BluetoothSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <MD5Builder.h>
#include <FastLED.h>

#ifdef ARDUINO_M5Stack_ATOM
#define M5ATOM
#endif

#define DEBUG_LEVEL 0
#define DROPPKT 0

#include "config_settings.h"
#include "ringqueue.h"

/* Session Parameters */
#define WIFITIMEOUT 8000    /* WiFi AP connection time out */
#define KEEPALIVE 15000     /* Send keep-alive to server every 15sec */
#define AUTHTIMEOUT 5000    /* Authentication timeout (5sec) */
#define IDLETIMEOUT 1800000 /* Idle timeout (30min)  */

/* Packet Type & Queue params. */
#define SYMBOLWAIT 2           /* Wait two symbol */
#define QLATENCY 2             /* Process queued symbol if ((mark and space duriotn) * qlatency (msec) elapsed. */
#define MAX_MARK_DURATION 5000 /* Maximum MARK duration 5sec */

/* GPIO setting */
#ifdef M5ATOM
const uint8_t gpio_key = 19; /* from Keyer */
const uint8_t gpio_led = 27; /* to LED */
const uint8_t gpio_out = 23; /* to Photocoupler */
#else
const uint8_t gpio_key = 25; /* from Keyer */
const uint8_t gpio_led = 26; /* to LED */
const uint8_t gpio_out = 27; /* to Photocoupler */
#endif

/* WiFi & IP Address Configrations */
/* Keyer ID and password */
char keyer_name[64];
char keyer_passwd[64];
char server_name[64];

/* for WiFi Station  */
#define MAXSSID 3
char ssid[MAXSSID][64];
char passwd[MAXSSID][64];

/* for WiFi Access point */
char ap_ssid[64];
char ap_passwd[64];

/* Server global address & port */
char keyer_global[64];
int keyer_global_port;

/* Server/client local address & port */
char keyer_local[64];
int keyer_local_port;

IPAddress keying_server;
int keying_server_port;
IPAddress authsender;
int authport;

unsigned int symbolwait = SYMBOLWAIT;
unsigned int queuelatency = QLATENCY;
boolean server_mode;
boolean ap_mode;
boolean udp_send_edge;
boolean connected;
boolean forbidden_reset_outside = false;

DynamicJsonDocument config(1024);
KeyConfig configfile = KeyConfig(config);
boolean config_update;

WiFiUDP wudp;
WiFiMulti wifiMulti;
WebServer httpServer(80);
BluetoothSerial SerialIBT;
boolean enable_bt, bt_connected, bt_receiving;
int serial_baudrate;

int pkt_error, pkt_delay;
unsigned long last_received = 0, last_received_pkt = 0;
unsigned int long_mark, short_mark;
unsigned int space_duration;

unsigned long prev_seq;
unsigned long seq_number;
unsigned long authtimer, keepalivetimer, idletimer;

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
  PKT_SERIAL
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

#define PKTBUFSIZ 128
struct KeyerPkt
{
  PktType type;
  int16_t size;
  union
  {
    DotDash data;
    uint8_t hash[16];
    uint8_t buffer[PKTBUFSIZ];
  };
};

/* Server state */
enum ServerState
{
  KEYER_WAIT,
  KEYER_AUTH,
  KEYER_ACTIVE
};
int serverstate;

/* Key state */
enum KeyState
{
  NONE,
  MARK,
  SPACE
};
int keystate;

/* Keyer error code */
enum KeyError
{
  FAIL_NONE,
  FAIL_AUTHFAIL,
  FAIL_AUTHTIMEOUT,
  FAIL_RESETPEER,
  FAIL_DURATIONTOOLONG
};
int keyer_errno;

/* Display LED */
const CRGB blackcolor = 0x00000;
const CRGB redcolor = CRGB::OrangeRed;
const CRGB greencolor = CRGB::YellowGreen;
const CRGB bluecolor = CRGB::RoyalBlue;
CRGB _ledbuffer[1];

void init_led()
{
#ifdef M5ATOM
  FastLED.addLeds<SK6812, gpio_led, GRB>(_ledbuffer, 1);
  FastLED.setBrightness(1);
#else
  pinMode(gpio_led, OUTPUT);
#endif
}

void set_led(CRGB color)
{
#ifdef M5ATOM
  _ledbuffer[0] = color;
  FastLED.show();
#else
  if (blackcolor != color)
    digitalWrite(gpio_led, HIGH);
  else
    digitalWrite(gpio_led, LOW);
#endif
}

RingQueue<DotDash> queue = RingQueue<DotDash>();
unsigned long lastqueued;

/* ISR Stuff */
#define STABLEPERIOD 10 /* Discard Interrupts 10msec */
volatile int numofint = 0;
volatile int key_state;
volatile unsigned long lastmillis;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

EdgeType key_edge;
boolean key_changed;

/* prototype */
void send_code(IPAddress, int, DotDash &);
void settimeout(unsigned long &t);

void IRAM_ATTR keyInput()
{
  portENTER_CRITICAL_ISR(&mux);
  numofint += 1;
  key_state = digitalRead(gpio_key);
  lastmillis = xTaskGetTickCount();
  portEXIT_CRITICAL_ISR(&mux);
}

void processKeyInput()
{
  int saved_key, current_key;
  int counter;
  unsigned long now, saved_lastmillis;
  static unsigned long lastfalledge = 0;
  DotDash d;

  portENTER_CRITICAL(&mux);
  saved_key = key_state;
  counter = numofint;
  saved_lastmillis = lastmillis;
  portEXIT_CRITICAL(&mux);

  current_key = digitalRead(gpio_key);
  now = millis();
  /* wait for stable period & read key status */
  if ((counter != 0) && (current_key == saved_key) &&
      (now - saved_lastmillis > STABLEPERIOD))
  {
    if (current_key == HIGH)
    {
      key_edge = RISE_EDGE;
    }
    else
    {
      key_edge = FALL_EDGE;
      lastfalledge = now;
    }

    d.data = key_edge;
    d.t = now;
    d.d = now - lastfalledge;
    d.seq = seq_number;

    /* update statics */
    if (d.d > long_mark)
      long_mark = d.d;
    if ((d.d > 0) && (d.d < short_mark))
      short_mark = d.d;

    if (serverstate == KEYER_ACTIVE)
    {
      if (!udp_send_edge && d.d > MAX_MARK_DURATION)
      {
        keyer_errno = FAIL_DURATIONTOOLONG;
      }
      else
      {
        keyer_errno = FAIL_NONE;
        if (udp_send_edge || key_edge == RISE_EDGE)
        {
          send_code(keying_server, keying_server_port, d);
          seq_number++;
          settimeout(keepalivetimer);
        }
      }
    }
    /* clear interrupt counter and notify*/
    portENTER_CRITICAL(&mux);
    numofint = 0;
    portEXIT_CRITICAL(&mux);
    key_changed = true;
  }
}

void debug_print(const char *msg, DotDash &d)
{
#if DEBUG_LEVEL > 1
  Serial.printf("%s:[seq:%d] prev=%d, keystate=%d, edge=%d, t=%d, d=%d space= %d, shortest=%d longest=%d\n",
                msg, d.seq, prev_seq, keystate, d.data, d.t, d.d, space_duration, short_mark, long_mark);
#endif
}
void debug_sem(String mesg, KeyerPkt p)
{
#if DEBUG_LEVEL > 0
  String type, data, state;

  switch (p.type)
  {
  case PKT_SYN:
    type = "SYN";
    break;
  case PKT_RST:
    type = "RST";
    break;
  case PKT_ACK:
    type = "ACK";
    break;
  case PKT_NACK:
    type = "NACK";
    break;
  case PKT_AUTH:
    type = "AUTH";
    data = "";
    break;
  case PKT_CODE:
    type = "CODE";
    data = "time=" + String(p.data.t) + " duration=" + String(p.data.d) + " seq=" + String(p.data.seq);
    break;
  case PKT_CODE_RESENT:
    type = "RESENT";
    data = "duration=" + String(p.data.d) + " seq=" + String(p.data.seq);
    break;
  case PKT_CONF:
    type = "CONF";
    data = "mode=" + String(p.data.data) + " latency=" + String(p.data.d) + " symbol=" + String(p.data.t);
    break;
  case PKT_SERIAL:
    type = "SERIAL";
    data = "size=" + String(p.size) + " data=";
    for (int i = 0; i < p.size; i++)
      data += String(p.buffer[i], HEX);
    break;
  case PKT_KEEPALIVE:
    type = "KEEPALIVE";
    break;
  }
  switch (serverstate)
  {
  case KEYER_WAIT:
    state = "WAIT";
    break;
  case KEYER_AUTH:
    state = "AUTH";
    break;
  case KEYER_ACTIVE:
    state = "ACTIVE";
    break;
  }
  Serial.println("[" + mesg + "]" + state + " " + type + " " + data);
#endif
}

void settimeout(unsigned long &t)
{
  t = millis();
}

boolean timeout(unsigned long t, unsigned long timeout)
{
  unsigned long now = millis();
  return (now - t) > timeout;
}

KeyerPkt _resend_buffer[16];

void send_udp(IPAddress recipient, int port, KeyerPkt &k)
{
  wudp.beginPacket(recipient, port);
  wudp.write((uint8_t *)&k, sizeof(KeyerPkt));
  wudp.endPacket();
}

void clear_udp_buffer()
{
  for (int i = 0; i < 16; i++)
    _resend_buffer[i].data.seq = 0;
}

void send_code(IPAddress recipient, int port, DotDash &d)
{
  KeyerPkt k;
  k.type = PKT_CODE;
  k.data = d;
  _resend_buffer[d.seq % 16] = k;
  debug_sem("send code", k);
  send_udp(recipient, port, k);
}

void resend_code(IPAddress recipient, int port, int seq)
{
  if (_resend_buffer[seq % 16].data.seq == seq)
  {
    _resend_buffer[seq % 16].type = PKT_CODE_RESENT;
    debug_sem("resend code", _resend_buffer[seq % 16]);
    send_udp(recipient, port, _resend_buffer[seq % 16]);
  }
}

void send_nack(IPAddress recipient, int port, int seq)
{
  KeyerPkt k;
  k.type = PKT_NACK;
  k.data.seq = seq;
  send_udp(recipient, port, k);
}

void send_config(IPAddress recipient, int port)
{
  KeyerPkt k;

  k.type = PKT_CONF;
  if (udp_send_edge)
    k.data.data = FALL_EDGE;
  else
    k.data.data = RISE_EDGE;

  k.data.d = queuelatency;
  k.data.t = symbolwait;
  send_udp(recipient, port, k);
}

void recv_config(DotDash &d)
{
  if (d.data == RISE_EDGE)
    udp_send_edge = false;
  else
    udp_send_edge = true;

  queuelatency = d.d;
  symbolwait = d.t;
}

/* Toglle Key output */
void mark()
{
  #ifndef M5ATOM
  /* High priority Wi-Fi tasks may cause serial LED data crashes.*/
  set_led(redcolor);
  #endif
  digitalWrite(gpio_out, HIGH);
}

void space()
{
  #ifndef M5ATOM
  set_led(blackcolor);
  #endif
  digitalWrite(gpio_out, LOW);
}

/* Parse packets & processing state machine */
void handle_code(PktType, DotDash &);

void process_incoming_packet(void)
{
  static uint8_t keyseed[16];
  static KeyerPkt pkt_pushed;
  static boolean has_pushed = false;
  KeyerPkt k;
  unsigned long now;
  IPAddress ip;
  int psize;

  psize = wudp.parsePacket();

  if (psize > 0 || (has_pushed && serverstate == KEYER_ACTIVE))
  {
    now = millis();
    if (has_pushed && serverstate == KEYER_ACTIVE)
    {
      k = pkt_pushed;
      has_pushed = false;
    }
    else
    {
      wudp.read((uint8_t *)&k, sizeof(k));
      wudp.flush();
    }

    debug_sem("recv", k);

    switch (serverstate)
    {
    case KEYER_WAIT:
      if (server_mode)
      {
        switch (k.type)
        {
        /* Send challenge to client */
        case PKT_SYN:
          MD5Builder md5;
          md5.begin();
          md5.add(keyer_name);
          md5.add(String(esp_random()));
          md5.calculate();
          md5.getBytes(keyseed);
          memcpy(k.hash, keyseed, 16);
          k.type = PKT_AUTH;
          send_udp(wudp.remoteIP(), wudp.remotePort(), k);
          settimeout(authtimer);
          serverstate = KEYER_AUTH;
          debug_sem("send", k);
          break;
        case PKT_CODE:
          pkt_pushed = k;
          has_pushed = true;
        case PKT_KEEPALIVE:
          /* Force re-authenticate client */
          k.type = PKT_RST;
          send_udp(wudp.remoteIP(), wudp.remotePort(), k);
          break;
        default:
          break;
        }
      }
      else
      {
        switch (k.type)
        {
        /* Receive challenge from server */
        case PKT_SYN:
          uint8_t buff[16];
          MD5Builder md5;
          md5.begin();
          md5.add(server_name);
          md5.add(k.hash, 16);
          md5.add(keyer_passwd);
          md5.calculate();
          md5.getBytes(buff);
          memcpy(k.hash, buff, 16);
          k.type = PKT_AUTH;
          send_udp(keying_server, keying_server_port, k);
          settimeout(authtimer);
          serverstate = KEYER_AUTH;
          break;

        default:
          break;
        }
      }
      break;

    case KEYER_AUTH:
      if (server_mode)
      {
        switch (k.type)
        {
          /* Receive response from client */
        case PKT_AUTH:
          uint8_t buff[16];
          MD5Builder md5;
          md5.begin();
          md5.add(keyer_name);
          md5.add(keyseed, 16);
          md5.add(keyer_passwd);
          md5.calculate();
          md5.getBytes(buff);

          if (memcmp(buff, k.hash, 16) == 0)
          {
            authsender = wudp.remoteIP();
            authport = wudp.remotePort();
            k.type = PKT_ACK;
            if (udp_send_edge)
              k.data.data = FALL_EDGE;
            else
              k.data.data = RISE_EDGE;
            send_udp(authsender, authport, k);
            settimeout(idletimer);
            pkt_error = 0;
            keyer_errno = FAIL_NONE;
            serverstate = KEYER_ACTIVE;
            debug_sem("send", k);
          }
          else
          {
            k.type = PKT_NACK;
            send_udp(wudp.remoteIP(), wudp.remotePort(), k);
            keyer_errno = FAIL_AUTHFAIL;
            serverstate = KEYER_WAIT;
            debug_sem("send", k);
          }
          break;
        default:
          break;
        }
      }
      else
      {
        switch (k.type)
        {
          /* Receive challenge from server */
        case PKT_AUTH:
          uint8_t buff[16];
          MD5Builder md5;
          keyer_errno = FAIL_NONE;
          md5.begin();
          md5.add(server_name);
          md5.add(k.hash, 16);
          md5.add(keyer_passwd);
          md5.calculate();
          md5.getBytes(buff);
          memcpy(k.hash, buff, 16);
          k.type = PKT_AUTH;
          send_udp(keying_server, keying_server_port, k);
          debug_sem("send", k);
          break;

          /* Success server authentication */
        case PKT_ACK:
          if (!((k.data.data == FALL_EDGE && udp_send_edge) ||
                (k.data.data == RISE_EDGE && !udp_send_edge)))
            send_config(keying_server, keying_server_port);

          pkt_error = 0;
          keyer_errno = FAIL_NONE;
          serverstate = KEYER_ACTIVE;
          settimeout(keepalivetimer);
          break;

          /* Server authentication failed */
        case PKT_NACK:
          keyer_errno = FAIL_AUTHFAIL;
          serverstate = KEYER_WAIT;
          break;
        default:
          break;
        }
      }
      break;
    case KEYER_ACTIVE:
      switch (k.type)
      {
      case PKT_CODE:
      case PKT_CODE_RESENT:
        /* Handle incoming keying packets */
        if (server_mode && authsender == wudp.remoteIP() && authport == wudp.remotePort())
        {
          settimeout(idletimer);
          pkt_delay = (now - last_received) - (k.data.t - last_received_pkt);
          last_received = now;
          last_received_pkt = k.data.t;
          handle_code(k.type, k.data);
        }
        break;

      /* Resend keying packets from client */
      case PKT_NACK:
        if (!server_mode)
        {
          pkt_error++;
          resend_code(keying_server, keying_server_port, k.data.seq);
        }
        break;

      /* Configration change */
      case PKT_CONF:
        if (server_mode)
          recv_config(k.data);
        break;

      /* Receive serial data from Server */
      case PKT_SERIAL:
        if (server_mode)
        {
          if (bt_connected)
          {
            settimeout(idletimer);
            SerialIBT.write(k.buffer, k.size);
          }
        }
        else
        {
          if (enable_bt)
          {
            bt_receiving = true;
            Serial.write(k.buffer, k.size);
          }
        }
        break;

      /* keep sanity */
      case PKT_KEEPALIVE:
        if (server_mode)
          space();
        break;

      /* Connection closed by peer */
      case PKT_RST:
        if (server_mode)
        {
          if (forbidden_reset_outside)
          {
            if (authsender == wudp.remoteIP() && authport == wudp.remotePort())
            {
              keyer_errno = FAIL_RESETPEER;
              serverstate = KEYER_WAIT;
            }
          }
          else
          {
            keyer_errno = FAIL_RESETPEER;
            serverstate = KEYER_WAIT;
          }
        }
        else
        {
          keyer_errno = FAIL_RESETPEER;
          serverstate = KEYER_WAIT;
        }
        break;

      default:
        break;
      }
      break;

    default:
      break;
    }
  }
}

/* Process Blutooth Serial */
void process_serial()
{
  static uint8_t sbuff[PKTBUFSIZ];
  static int sbptr = 0;

  uint8_t c;
  KeyerPkt k;

  if (server_mode)
  {
    if (bt_connected && SerialIBT.available())
    {
      c = SerialIBT.read();
      sbuff[sbptr++] = c;
      if (c == 0xfd || sbptr >= PKTBUFSIZ)
      {
        k.type = PKT_SERIAL;
        k.size = sbptr;
        memcpy(k.buffer, sbuff, sbptr);
        if (serverstate == KEYER_ACTIVE)
        {
          settimeout(idletimer);
          debug_sem("send", k);
          send_udp(authsender, authport, k);
        }
        sbptr = 0;
      }
    }
  }
  else
  {
    if (Serial.available())
    {
      c = Serial.read();
      sbuff[sbptr++] = c;
      if (c == 0xfd || sbptr >= PKTBUFSIZ)
      {
        k.type = PKT_SERIAL;
        k.size = sbptr;
        memcpy(k.buffer, sbuff, sbptr);
        if (serverstate == KEYER_ACTIVE)
        {
          debug_sem("send", k);
          send_udp(keying_server, keying_server_port, k);
        }
        sbptr = 0;
      }
    }
  }
}

/* Periodical Stuff */
void process_periodical()
{
  KeyerPkt k;

  /*  Process Serial Input */
  process_serial();

  /* Periodical process*/
  switch (serverstate)
  {
  case KEYER_AUTH:
    if (timeout(authtimer, AUTHTIMEOUT))
    {
      Serial.println("Authentication timed out");
      keyer_errno = FAIL_AUTHTIMEOUT;
      serverstate = KEYER_WAIT;
    }
    break;

  case KEYER_ACTIVE:
    if (server_mode)
    {
      if (timeout(idletimer, IDLETIMEOUT))
      {
        k.type = PKT_RST;
        send_udp(authsender, authport, k);
        serverstate = KEYER_WAIT;
      }
    }
    else
    {
      if (timeout(keepalivetimer, KEEPALIVE))
      {
        k.type = PKT_KEEPALIVE;
        send_udp(keying_server, keying_server_port, k);
        settimeout(keepalivetimer);
      }
    }
  default:
    break;
  }
}

unsigned long marktimeout = 0;

void toggleKeyEdge(DotDash &d)
{
  if (d.data == RISE_EDGE)
  {
    space();
  }
  else
  {
    mark();
    settimeout(marktimeout);
  }
}

void toggleKeyTime()
{
  DotDash d;
  unsigned long now = millis();
  static unsigned long duration, startperiod;
  static unsigned long prev_t, prev_d;

  if (keystate == NONE)
  {
    if (queue.length() > symbolwait || (now - lastqueued) > (long_mark + space_duration) * queuelatency)
    {
      if (queue.isEmpty())
        return;

      queue.dequeue(d);
      if (d.data == RISE_EDGE)
      {
        long_mark = 0;
        short_mark = MAX_MARK_DURATION;
        keystate = MARK;
        debug_print("MARK", d);
        prev_t = d.t;
        duration = d.d;
        startperiod = now;
        mark();
        return;
      }
    }
  }

  if (keystate == MARK)
  {
    if ((now - startperiod) > duration)
    {
      space();
      if (queue.isEmpty())
      {
        keystate = NONE;
        debug_print("NONE", d);
        return;
      }
      queue.dequeue(d);
      if (d.data == RISE_EDGE)
      {
        keystate = SPACE;
        debug_print("SPACE", d);
        duration = d.t - d.d - prev_t;
        if (duration > MAX_MARK_DURATION) /* may not happen */
          duration = 0;
        space_duration = duration;
        prev_t = d.t;
        prev_d = d.d;
        startperiod = now;
        return;
      }
    }
  }

  if (keystate == SPACE)
  {
    if ((now - startperiod) > duration)
    {
      keystate = MARK;
      debug_print("MARK2", d);
      duration = prev_d;
      startperiod = now;
      mark();
      return;
    }
  }
}

void handle_code(PktType t, DotDash &d)
{
  /* Handle incoming Keyer Packet */
  if (udp_send_edge)
  {
    /* Toggle Keyer Output */
    toggleKeyEdge(d);
  }
  else
  {
    /* Enqeue packets */
    switch (t)
    {
    case PKT_CODE:
#if DROPPKT
      /* Drop 10% of incoming packet for test. */
      if (random(100) > 90)
        return;
#endif
      if (d.seq == (prev_seq + 1))
      {
        debug_print("PKT_OK", d);
      }
      else if (prev_seq != 0)
      {
        pkt_error++;
        debug_print("PKT_ERR", d);
        send_nack(authsender, authport, prev_seq + 1);
      }
      prev_seq = d.seq;
      if (d.d > MAX_MARK_DURATION)
      { /* mark duration too long drop packet */
        keyer_errno = FAIL_DURATIONTOOLONG;
        pkt_error++;
        return;
      }
      if (d.d > long_mark)
        long_mark = d.d;
      if ((d.d > 0) && (d.d < short_mark))
        short_mark = d.d;

      queue.enqueue(d.seq, d);
      lastqueued = millis();
      break;

    case PKT_CODE_RESENT:
      queue.pri_enqueue(d.seq, d);
      debug_print("PKT_RESENT", d);
      lastqueued = millis();
      break;
    default:
      break;
    }
  }
}

void handleWiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.print("WiFi Connected IP Address:");
    Serial.println(WiFi.localIP());
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    connected = false;
    Serial.println("WiFi Lost Connection...");
    break;
  default:
    break;
  }
}

void handleRoot(void)
{
  String response, keyer, mode;
  String updated = "";

  keyer = keyer_name;
  keyer += ".local (";
  keyer += WiFi.localIP().toString() + ")";
  if (server_mode)
    mode = "Server";
  else
    mode = "Client";

  response =
      "<!DOCTYPE html>"
      "<html>"
      "<head>"
      " <meta charset=\"utf-8\">"
      "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1, shrink-to-fit=no\">"
      " <title> WiFiKey</title>"
      "</head>"
      "<body bgcolor=\"#455a64\" text=\"#ffffff\">"
      "<h3>WiFiKey</h3>"
      "<hr>"
      "<table>"
      "<tr><td>Name:&nbsp</td><td>" +
      keyer +
      "</td></tr>"
      "<tr><td>Mode:&nbsp</td><td>" +
      mode +
      "</td></tr>"
      "</table><hr>"
      "<iframe src=\"./stats.html\" width=\"320\" height=\"240\"></iframe>"
      "<hr>"
      "<button onclick=\"location.href='./settings.html'\">Settings</button>"
      "</body></html>";
  httpServer.send(200, "text/html", response);
}

String keyer_errmsg(void)
{
  const char *msg;
  switch (keyer_errno)
  {
  case FAIL_NONE:
    msg = "";
    break;
  case FAIL_AUTHFAIL:
    msg = "Authentication failure.";
    break;
  case FAIL_AUTHTIMEOUT:
    msg = "Authentication timed out.";
    break;
  case FAIL_RESETPEER:
    msg = "Connection timed out.";
    break;
  case FAIL_DURATIONTOOLONG:
    msg = "Too long mark duration.";
    break;
  default:
    msg = "Unknown Error.";
    break;
  }
  return String(msg);
}

void handleStats(void)
{
  String response, host;
  float estimated_wpm = 0, estimated_ratio = 0;

  if (short_mark > 0)
  {
    if (short_mark < MAX_MARK_DURATION)
      estimated_wpm = 1200 / short_mark;
    estimated_ratio = long_mark / short_mark;
  }
  short_mark = MAX_MARK_DURATION;

  if (serverstate == KEYER_ACTIVE)
  {
    if (server_mode)
    {
      int remain = (IDLETIMEOUT - (millis() - idletimer)) / 60000;
      host = "Client: " + authsender.toString() + ":" + String(authport) + "<br>";
      host += "Timeout: " + String(remain + 1) + " min<br>";
    }
    else
    {
      host = "Server: " + keying_server.toString() + ":" + String(keying_server_port) + "<br>";
      long_mark = 0;
    }
  }
  else
    host = "";

  if (keyer_errno != FAIL_NONE)
    host += "Error: " + keyer_errmsg() + "<br>";

  response =
      "<!DOCTYPE html>"
      "<html>"
      "<head>"
      " <meta charset=\"utf-8\">"
      " <meta http-equiv=\"Refresh\" content=\"3\">"
      "</head>"
      "<body bgcolor=\"#000000\" text=\"#ffffff\"><p>" +
      host +
      "Estimated Speed: " +
      String(estimated_wpm, 0) +
      " WPM<br>"
      "Estimated Dash Dot Ratio: " +
      String(estimated_ratio, 1) +
      "<br>"
      "Packet Error: " +
      String(pkt_error) +
      "<br>";

  if (server_mode)
  {
    response +=
        "Packet Delay: " +
        String(pkt_delay) +
        "<br>"
        "Max queue length: " +
        String(queue.maxLength()) +
        "<br>"
        "Max long mark duration: " +
        String(long_mark) +
        "ms<br>"
        "Space duration: " +
        String(space_duration) +
        "ms<br></p>"
        "</body></html>";
  }
  else
  {
    if (enable_bt)
    {
      response += "CI-V: ";
      if (bt_receiving)
        response += "Receiving";
      else
        response += "Not receiving";
      response += "<br>";
      bt_receiving = false;
    }
  }
  pkt_delay = 0;

  httpServer.send(200, "text/html", response);
}

void handleSettings()
{
  if (httpServer.hasArg("get-properties"))
  {
    String response;

    serializeJson(config, response);
    httpServer.send(200, "applicaton/json", response);
  }

  else if (httpServer.hasArg("init"))
  {
    String response;

    if (configfile.readFile("/config.json"))
      configfile.savePrefs();
    serializeJson(config, response);
    httpServer.send(200, "applicaton/json", response);
  }

  else if (httpServer.hasArg("reset"))
  {
    delay(2000);
    ESP.restart();
  }

  else if (SPIFFS.exists("/settings.html"))
  {
    File file = SPIFFS.open("/settings.html", "r");

    httpServer.streamFile(file, "text/html");
    file.close();
  }

  else
    Serial.println("File not found");
}

void handleSettingsPost(void)
{
  String response;

  response = httpServer.arg("plain");
  DeserializationError error = deserializeJson(config, response);
  if (error)
  {
    Serial.println("Bad Response:" + response);
  }
  else
  {
    configfile.savePrefs();
    config_update = true;
  }
  response = "";
  serializeJson(config, response);
  httpServer.send(200, "application/json", response);
}

void handleNotFound()
{
  httpServer.send(404, "text/plain", "404 Not Found.");
}

boolean hostByString(const char *host, IPAddress &ip)
{
  IPAddress myip;

  if (WiFiGenericClass::hostByName(host, myip))
  {
    ip = myip;
    return true;
  }
  else if (myip.fromString(host))
  {
    ip = myip;
    return true;
  }
  else
    return false;
}

void handleBT(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
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
    SerialIBT.begin(keyer_name);
  }
}

void bt_setup()
{

  bt_receiving = false;

  if (server_mode)
  {
    if (enable_bt)
    {
      bt_connected = false;
      Serial.println("Start Bluetooh service.");
      SerialIBT.register_callback(handleBT);
      SerialIBT.begin(keyer_name);
    }
    else if (bt_connected)
    {
      SerialIBT.disconnect();
      SerialIBT.end();
    }
  }
  /* Reset serial baudrate */
  Serial.flush();
  Serial.end();
  Serial.begin(serial_baudrate);
}

void wifi_setup()
{
  IPAddress localip;

  connected = false;
  set_led(greencolor);

  WiFi.disconnect(true, true);
  WiFi.onEvent(handleWiFiEvent);

  if (server_mode)
  { /* Server side */
    if (ap_mode)
    {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(ap_ssid, ap_passwd);
      delay(100);

      if (!hostByString(keyer_local, localip))
      {
        Serial.println("Config error: Access Point IP");
        while (1)
          delay(1000);
      }
      WiFi.softAPConfig(localip, localip, IPAddress(255, 255, 255, 0)); /* Assume Class C */
      Serial.print("Access point address: ");
      Serial.println(WiFi.softAPIP());
    }
    else
    {
      for (int i = 0; i < MAXSSID; i++)
        wifiMulti.addAP(ssid[i], passwd[i]);

      while (wifiMulti.run(WIFITIMEOUT) != WL_CONNECTED)
      {
        WiFi.disconnect(true, true);
        delay(500);
        Serial.println("Connecting to AP...");
      }
    }

    MDNS.begin(keyer_name);
    wudp.begin(keyer_local_port);
    pkt_error = 0;
    Serial.println("Keyer server ready");
  }
  else
  { /* Client Side */
    if (ap_mode)
    {
      WiFi.begin(ap_ssid, ap_passwd);
      while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
        Serial.println("Connecting to standalone AP...");
      }

      MDNS.begin(keyer_name);
      localip = MDNS.queryHost(server_name);
      if (localip != IPAddress(0, 0, 0, 0))
      {
        Serial.println("[mDNS] Found a local keying server at " + localip.toString());
        keying_server = localip;
        keying_server_port = keyer_local_port;
      }
      else
      {
        if (hostByString(keyer_local, localip))
        {
          Serial.println("No local server");
          keying_server = localip;
          keying_server_port = keyer_global_port;
        }
      }
    }
    else
    {
      for (int i = 0; i < MAXSSID; i++)
        wifiMulti.addAP(ssid[i], passwd[i]);

      while (wifiMulti.run(WIFITIMEOUT) != WL_CONNECTED)
      {
        WiFi.disconnect(true, true);
        delay(500);
        Serial.println("Connecting to AP...");
      }

      MDNS.begin(keyer_name);
      localip = MDNS.queryHost(server_name);
      if (localip != IPAddress(0, 0, 0, 0))
      {
        Serial.println("[mDNS] Found a local keying server at " + localip.toString());
        keying_server = localip;
        keying_server_port = keyer_local_port;
      }
      else
      {
        if (hostByString(keyer_global, localip))
        {
          Serial.println("No local keying server.");
          keying_server = localip;
          keying_server_port = keyer_global_port;
        }
      }
    }

    /* client side */
    wudp.begin(keyer_local_port);
    Serial.print("Keyer client is ready to connet: ");
    Serial.println(keying_server.toString() + ":" + String(keying_server_port));
  }

  httpServer.on("/", handleRoot);
  httpServer.on("/stats.html", handleStats);
  httpServer.on("/settings.html", HTTP_GET, handleSettings);
  httpServer.on("/settings.html", HTTP_POST, handleSettingsPost);
  httpServer.onNotFound(handleNotFound);

  httpServer.begin();

  connected = true;
  set_led(blackcolor);
}

#define _strcpy(x, y) strlcpy(x, y, sizeof(x))

void load_config()
{
  if (!configfile.loadPrefs())
  {
    Serial.println("NVM Configration Error.");
    while (1)
      delay(1000);
  }

  _strcpy(keyer_name, config["keyername"]);
  _strcpy(keyer_passwd, config["keyerpasswd"]);

  if (strcmp(config["servermode"], "%checked%") == 0)
    server_mode = true;
  else
    server_mode = false;

  if (strcmp(config["enablebt"], "%checked%") == 0)
    enable_bt = true;
  else
    enable_bt = false;

  serial_baudrate = config["baudrate"];

  _strcpy(server_name, config["servername"]);

  if (strcmp(config["wifistn"], "%checked%") == 0)
    ap_mode = false;
  else
    ap_mode = true;

  _strcpy(ssid[0], config["SSID1"]);
  _strcpy(passwd[0], config["passwd1"]);
  _strcpy(ssid[1], config["SSID2"]);
  _strcpy(passwd[1], config["passwd2"]);
  _strcpy(ssid[2], config["SSID3"]);
  _strcpy(passwd[2], config["passwd3"]);
  _strcpy(ap_ssid, config["APSSID"]);
  _strcpy(ap_passwd, config["passwdap"]);

  if (strcmp(config["pkttypetime"], "%checked%") == 0)
    udp_send_edge = false;
  else
    udp_send_edge = true;

  _strcpy(keyer_local, config["localaddr"]);
  keyer_local_port = config["localport"];

  _strcpy(keyer_global, config["globaladdr"]);
  keyer_global_port = config["globalport"];

  queuelatency = config["latency"];
  symbolwait = config["symbol"];
}

void config_sanitycheck()
{
  char nvm_version[64], file_version[64];

  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    while (1)
      delay(1000);
  }
  if (!configfile.readFile("/config.json"))
  {
    Serial.println("No Configration files");
    while (1)
      delay(1000);
  }
  if (!config.containsKey("version"))
  {
    Serial.println("Illegal configration format");
    while (1)
      delay(1000);
  }
  _strcpy(file_version, config["version"]);
#if DEBUG_LEVEL > 1
  Serial.print("File version:");
  Serial.println(file_version);
#endif
  nvm_version[0] = '\0';
  if (configfile.loadPrefs())
  {
    if (config.containsKey("version"))
      _strcpy(nvm_version, config["version"]);
  }
#if DEBUG_LEVEL > 1
  Serial.print("NVM version:");
  Serial.println(nvm_version);
#endif
  if (strcmp(nvm_version, file_version) != 0)
  {
    Serial.println("Obsolete NVM configration. Read new configration from config.json");
    configfile.readFile("/config.json");
    configfile.savePrefs();
  }
  load_config();
}

void setup()
{
  serial_baudrate = 115200;

  Serial.begin(serial_baudrate);
  pinMode(gpio_out, OUTPUT);
  pinMode(gpio_key, INPUT_PULLUP);

  digitalWrite(gpio_out, LOW);
  init_led();

  connected = false;

  config_sanitycheck();
  wifi_setup();

  bt_connected = false;
  bt_setup();

  if (!server_mode)
    attachInterrupt(gpio_key, keyInput, CHANGE);

  clear_udp_buffer();
  keystate = NONE;
  serverstate = KEYER_WAIT;
  prev_seq = 0;
  seq_number = 0;
  lastqueued = 0;
  lastmillis = 0;
  key_changed = false;
  keyer_errno = 0;
  config_update = false;
}

void start_auth()
{
  KeyerPkt k;

  if (server_mode)
  {
    serverstate = KEYER_WAIT;
  }
  else
  {
    k.type = PKT_RST;
    send_udp(keying_server, keying_server_port, k);

    k.type = PKT_SYN;
    send_udp(keying_server, keying_server_port, k);

    settimeout(authtimer);
    serverstate = KEYER_AUTH;
  }
}

void update_config(void)
{
  IPAddress localip;

  boolean reauth = false;
  config_update = false;

  if (strcmp(config["pkttypetime"], "%checked%") == 0)
    udp_send_edge = false;
  else
    udp_send_edge = true;

  queuelatency = config["latency"];
  symbolwait = config["symbol"];

  if (!server_mode)
    send_config(keying_server, keying_server_port);
  pkt_error = 0;

  if (strcmp(config["enablebt"], "%checked%") == 0)
    enable_bt = true;
  else
    enable_bt = false;
  serial_baudrate = config["baudrate"];
  bt_setup();

  if (strcmp(keyer_passwd, config["keyerpasswd"]) != 0)
  {
    reauth = true;
    _strcpy(keyer_passwd, config["keyerpasswd"]);
  }

  if (!server_mode &&
      (strcmp(server_name, config["servername"]) != 0))
  {
    reauth = true;
    _strcpy(server_name, config["servername"]);
    localip = MDNS.queryHost(server_name);
    if (localip != IPAddress(0, 0, 0, 0))
    {
      keying_server = localip;
      keying_server_port = keyer_local_port;
    }
    else
    {
      if (hostByString(keyer_global, localip))
      {
        keying_server = localip;
        keying_server_port = keyer_global_port;
      }
    }
  }

  if (reauth)
    start_auth();
}

void loop()
{
  if (!connected)
  {
    if (server_mode)
      space();
    wifi_setup();
    serverstate = KEYER_WAIT;
  }
  else
  {
    process_incoming_packet();
    process_periodical();
    httpServer.handleClient();

    if (config_update)
      update_config();

    if (server_mode)
    { /* Process Keyer Server Loop */
      switch (serverstate)
      {
      case KEYER_WAIT:
        space();
        break;

      case KEYER_ACTIVE:
        if (udp_send_edge)
        {
          if (timeout(marktimeout, MAX_MARK_DURATION))
            space();
        }
        else
        {
          toggleKeyTime();
        }
        break;
      }
    }
    else
    {
      /* Process Keyer Client Loop */
      processKeyInput();

      switch (serverstate)
      {
      case KEYER_WAIT:
        if (key_changed ||
            (enable_bt && Serial.available()))
          start_auth();
        break;
      }
      /* Toggle LED */
      if (key_changed)
      {
        key_changed = false;
        if (key_edge == RISE_EDGE)
          set_led(blackcolor);
        else
          set_led(redcolor);
      }
    }
  }
}