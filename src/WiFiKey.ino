#include <WiFi.h>
#include <WiFiUDP.h>
#include <WebServer.h>

#define DEBUG 1

/* GPIO setting */
const uint8_t gpio_key = 14;   /* from Keyer */
const uint8_t gpio_apcfg = 25; /* WiFi mode H = via AP, L = Standalone AP */
const uint8_t gpio_pkcfg = 12; /* Packet Type H = Duration and Time, L = Edge */
const uint8_t gpio_led = 26;   /* to LED */
const uint8_t gpio_out = 27;   /* to Photocoupler */

/* Packet Type & Queue params. */
#define SYMBOLWAIT 4     /* Wait four symbol */
#define QLATENCY 50       /* Process queued symbol if ((mark and space duriotn) + 50) msec elapsed. */
#define MAXDURATION 10000 /* Maximum MARK duration */

/* WiFi and IP Address configrations */
#include "config_wifi.h"

/* Global variables */
unsigned int symbolwait = SYMBOLWAIT;
unsigned int queuelatency = QLATENCY;
boolean server_mode;
boolean ap_mode;
boolean udp_send_edge;
boolean connected;
WiFiUDP wudp;
WebServer httpServer(httpport);
#define MAX_MARK_DURAION 1000
unsigned int pkt_error;
unsigned int queue_error;
unsigned int long_mark, short_mark;
unsigned int space_duration;
unsigned long prev_seq;

struct DotDash
{
  uint8_t edge;
  unsigned long seq;
  unsigned long t;
  unsigned long d;
};

static int _head, _tail;

#define QUEUESIZE 256
DotDash _queue[QUEUESIZE];

void initQueue()
{
  _head = 0;
  _tail = 0;
}

boolean isEmpty()
{
  return (_head == _tail);
}

boolean isFull()
{
  return (_head == ((_tail + 1) % QUEUESIZE));
}

int queLength()
{
  int c = _tail - _head;
  if (c < 0)
    c += QUEUESIZE;
  return c;
}

void enqueue(DotDash &d)
{
  if (isFull())
  {
#ifdef DEBUG
    Serial.println("Queue full");
#endif
    queue_error++;
    return;
  }
  _queue[_tail].edge = d.edge;
  _queue[_tail].seq = d.seq;
  _queue[_tail].t = d.t;
  _queue[_tail].d = d.d;
  _tail = (_tail + 1) % QUEUESIZE;
}

void dequeue(DotDash &d)
{
  if (isEmpty())
  {
    return;
  }
  d.edge = _queue[_head].edge;
  d.seq = _queue[_head].seq;
  d.t = _queue[_head].t;
  d.d = _queue[_head].d;
  _head = (_head + 1) % QUEUESIZE;
}

/* ISR Stuff */
#define STABLEPERIOD 5 /* Discard Interrupts 5msec */
volatile unsigned long seq_number;
volatile unsigned long lastmillis, stableperiod;
volatile boolean changed;
volatile DotDash dotdash;
void IRAM_ATTR keyIn(void);

/* Key state */
enum KeyState
{
  NONE,
  MARK,
  SPACE
};
int keystate;
unsigned long lastpoll;

void handleWiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_STA_DISCONNECTED:
    connected = false;
    Serial.println("WiFi Lost Connection...");
    break;
  }
}

void handleRoot(void)
{
  String response;
  String updated = "";
  unsigned int param;
  float estimated_wpm = 0, estimated_ratio = 0;

  if (httpServer.hasArg("queuelatency"))
  {
    param = httpServer.arg("queuelatency").toInt();
    if ((param > 50) && (param < 1000) && (queuelatency != param))
    {
      queuelatency = param;
      pkt_error = 0;
      queue_error = 0;
      updated = "<p> Parameter updated.</p>";
    }
  }

  if (httpServer.hasArg("symbolwait"))
  {
    param = httpServer.arg("symbolwait").toInt();
    if ((param > 0) && (param < 50) && (symbolwait != param))
    {
      symbolwait = param;
      pkt_error = 0;
      queue_error = 0;
      updated = "<p> Parameter updated.</p>";
    }
  }

  if (short_mark < MAX_MARK_DURAION && short_mark != 0)
  {
    estimated_wpm = 1200 / short_mark;
    estimated_ratio = long_mark / short_mark;
  }

  short_mark = MAX_MARK_DURAION;
  long_mark = 0;
  prev_seq = 0;

  response =
      "<!DOCTYPE html>"
      "<html>"
      "<head>"
      " <meta charset=\"utf-8\">"
      " <title> WiFiKey Parameter Configraton</title>"
      "</head>"
      "<body>"
      "<p>WiFiKey Parameters </p><hr>"
      "<p>Server Address:" +
      WiFi.localIP().toString() +
      "</p>"
      "<p>Estimated WPM:" +
      String(estimated_wpm, 1) +
      "wpm<br>"
      "Estimated Dash Dot Ratio:" +
      String(estimated_ratio, 1) +
      "<br>"
      "Packet Error:" +
      String(pkt_error) +
      "<br>"
      "Queue Error:" +
      String(queue_error) +
      "<br></p>"
      "<form action=\"\" method=\"get\">"
      "<p> QueueLatency:"
      " <input type=\"textarea\" size=\"5\" name=\"queuelatency\" value=\"" +
      String(queuelatency) +
      "\">"
      "<br>"
      "SymbolWait:"
      " <input type=\"textarea\" size=\"5\" name=\"symbolwait\" value=\"" +
      String(symbolwait) +
      "\">"
      "<br>"
      "<input type=\"submit\" name=\"submit\" value=\"Set\"></p>"
      "</form><hr>" +
      updated + "</body></html>";

  httpServer.send(200, "text/html", response);
}

void handleNotFound(void)
{
  httpServer.send(404, "text/plain", "404 Not Found.");
}

void wifi_setup()
{
  digitalWrite(gpio_led, HIGH);
  WiFi.disconnect(true);
  WiFi.onEvent(handleWiFiEvent);
  if (server_mode)
  {
    if (ap_mode)
    {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(ssid[0], passwd[0]);
      delay(100);
      WiFi.softAPConfig(gateway, gateway, subnet);
      Serial.print("Server address: ");
      Serial.println(WiFi.softAPIP());
    }
    else
    {
      WiFi.config(server, server, subnet);
      WiFi.begin(ssid[1], passwd[1]);
      while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
        Serial.println("Connecting to AP...");
      }
      Serial.println("Connected.");
      Serial.print("Server ddress: ");
      Serial.println(WiFi.localIP());
    }
    wudp.begin(udpport);

    pkt_error = 0;
    queue_error = 0;
    long_mark = 0;
    short_mark = MAX_MARK_DURAION;

    httpServer.on("/", handleRoot);
    httpServer.onNotFound(handleNotFound);
    httpServer.begin();

    Serial.println("Server Ready");
  }
  else
  {
    if (ap_mode)
    {
      WiFi.mode(WIFI_STA);
      WiFi.config(client, gateway, subnet);
      WiFi.begin(ssid[0], passwd[0]);
      while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
        Serial.println("Connecting to Server...");
      }
      Serial.println("Connected.");
    }
    else
    {
      WiFi.begin(ssid[1], passwd[1]);
      while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
        Serial.println("Connecting to AP...");
      }
      Serial.println("Connected.");
    }
    wudp.begin(udpport);
    Serial.println("Client Ready");
  }
  connected = true;
  digitalWrite(gpio_led, LOW);
}

void setup()
{
  Serial.begin(115200);

  pinMode(gpio_out, OUTPUT);
  pinMode(gpio_led, OUTPUT);
  pinMode(gpio_key, INPUT_PULLUP);
  pinMode(gpio_apcfg, INPUT_PULLUP);
  pinMode(gpio_pkcfg, INPUT_PULLUP);

  digitalWrite(gpio_out, LOW);
  digitalWrite(gpio_led, LOW);

  connected = false;
  if (digitalRead(gpio_key) == HIGH)
  {
    server_mode = false;
    attachInterrupt(gpio_key, keyIn, CHANGE);
  }
  else
  {
    server_mode = true;
  }

  if (digitalRead(gpio_apcfg) == HIGH)
    ap_mode = false;
  else
    ap_mode = true;

  if (digitalRead(gpio_pkcfg) == HIGH)
    udp_send_edge = false;
  else
    udp_send_edge = true;

  wifi_setup();
  initQueue();

  keystate = NONE;
  lastpoll = 0;
  prev_seq = 0;
  seq_number = 0;
  lastmillis = 0;
  stableperiod = 0;
  changed = false;
}

enum EdgeType
{
  RISE_EDGE = 1,
  FALL_EDGE
};

void IRAM_ATTR keyIn()
{
  unsigned long now = millis();

  if ((now - stableperiod) < STABLEPERIOD)
    return;
  stableperiod = now;

  if (digitalRead(gpio_key) == HIGH)
  {
    dotdash.edge = RISE_EDGE;
    seq_number++;
  }
  else
  {
    dotdash.edge = FALL_EDGE;
    lastmillis = now;
  }

  dotdash.seq = seq_number;
  dotdash.t = now;
  dotdash.d = now - lastmillis;
  changed = true;
}

void debug_print(const char *msg, DotDash &d)
{
#ifdef DEBUG
  Serial.printf("%s:[seq:%d] prev=%d, keystate=%d, edge=%d, t=%d, d=%d space= %d, shortest=%d longest=%d\n",
                msg, d.seq, prev_seq, keystate, d.edge, d.t, d.d, space_duration, short_mark, long_mark);
#endif
}

void send_udp()
{
  if (ap_mode)
    wudp.beginPacket(gateway, udpport);
  else
    wudp.beginPacket(server, udpport);
  wudp.write((uint8_t *)&dotdash, sizeof(dotdash));
  wudp.endPacket();
}

void receive_udp()
{
  DotDash d;
  int psize = wudp.parsePacket();
  if (psize > 0)
  {
    wudp.read((uint8_t *)&d, sizeof(d));
    wudp.flush();

    if (d.seq == (prev_seq + 1))
    {
      debug_print("UDP", d);
    }
    else if (prev_seq != 0)
    {
      pkt_error++;
      debug_print("UDPERR", d);
    }
    prev_seq = d.seq;

    if (server_mode && !udp_send_edge)
    {
      if (d.d > long_mark)
        long_mark = d.d;

      if ((d.d > 0) && (d.d < short_mark))
        short_mark = d.d;
    }

    enqueue(d);
  }
}

/* Toglle Key output */
void mark()
{
  digitalWrite(gpio_led, HIGH);
  digitalWrite(gpio_out, HIGH);
}

void space()
{
  digitalWrite(gpio_led, LOW);
  digitalWrite(gpio_out, LOW);
}

unsigned long lastmark = 0;
void toggleKeyEdge()
{
  DotDash d;
  unsigned long now = millis();

  if ((now - lastmark) > MAXDURATION)
    space();

  if (isEmpty())
    return;

  dequeue(d);
  if (d.edge == RISE_EDGE)
  {
    space();
  }
  else
  {
    mark();
    lastmark = now;
  }

  return;
}

void toggleKeyTime()
{
  unsigned long now = millis();
  static unsigned long duration, startperiod;
  static unsigned long prev_t, prev_d;
  static DotDash d;

  if (keystate == NONE)
  {
    if (queLength() > symbolwait || (now - lastpoll) > (long_mark + space_duration + queuelatency))
    {
      lastpoll = now;
      if (isEmpty())
        return;
      dequeue(d);
      if (d.edge == RISE_EDGE)
      {
        keystate = MARK;
        debug_print("MARK", d);
        prev_t = d.t;
        duration = d.d;
        if (duration > MAXDURATION)
        {
          keystate = NONE;
          debug_print("ERR1", d);
          return;
        }
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
      lastpoll = now;
      if (isEmpty())
      {
        keystate = NONE;
        debug_print("NONE", d);
        return;
      }
      dequeue(d);
      if (d.edge == RISE_EDGE)
      {
        keystate = SPACE;
        debug_print("SPACE", d);
        duration = d.t - d.d - prev_t;
        space_duration = duration;
        if (duration > MAXDURATION)
        {
          keystate = NONE;
          debug_print("ERR2", d);
          return;
        }
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

void loop()
{
  if (!connected)
  {
    wifi_setup();
  }
  else if (server_mode)
  {
    receive_udp();

    if (udp_send_edge)
      toggleKeyEdge();
    else
      toggleKeyTime();
  }
  else if (changed)
  {
    changed = false;
    if (dotdash.edge == RISE_EDGE)
    {
      digitalWrite(gpio_led, LOW);
      send_udp();
    }
    else
    {
      digitalWrite(gpio_led, HIGH);
      if (udp_send_edge)
      {
        send_udp();
      }
    }
  }
  httpServer.handleClient();
}
