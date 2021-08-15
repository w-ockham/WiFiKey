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
#define SYMBOLWAIT 4      /* Wait four symbol */
#define QLATENCY 1        /* Process queued symbol if ((mark and space duriotn) * qlatency (msec) elapsed. */
#define MAXDURATION 10000 /* Maximum MARK duration */

/* WiFi and IP Address configrations */
#include "config_wifi.h"

/* Simple Ring Buffer */
#include "ringqueue.h"

/* Global variables */
IPAddress receiver;
unsigned int symbolwait = SYMBOLWAIT;
unsigned int queuelatency = QLATENCY;
boolean server_mode;
boolean ap_mode;
boolean udp_send_edge;
boolean connected;
WiFiUDP wudp;
WebServer httpServer(httpport);
#define MAX_MARK_DURAION 1000
int pkt_error, pkt_delay;
unsigned int long_mark, short_mark;
unsigned int space_duration;
unsigned long prev_seq;
unsigned long seq_number;

enum DataType
{
  PKT_CODE,
  PKT_SERIAL
};

enum EdgeType
{
  RISE_EDGE = 1,
  FALL_EDGE
};

struct DotDash
{
  DataType type;
  uint8_t data;
  unsigned long seq;
  unsigned long t;
  unsigned long d;
};

RingQueue<DotDash> queue = RingQueue<DotDash>();
unsigned long lastqueued;

/* ISR Stuff */
#define STABLEPERIOD 5 /* Discard Interrupts 5msec */
volatile int key_edge;
volatile boolean key_changed;
volatile unsigned long key_time, key_duration;
volatile unsigned long stableperiod, lastmillis;
void IRAM_ATTR keyIn(void);

/* Key state */
enum KeyState
{
  NONE,
  MARK,
  SPACE
};
int keystate;

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
  }
}

void handleRoot(void)
{
  String response;
  String updated = "";
  unsigned int param;

  if (httpServer.hasArg("queuelatency"))
  {
    param = httpServer.arg("queuelatency").toInt();
    if ((param > 0) && (param <= 10) && (queuelatency != param))
    {
      queuelatency = param;
      pkt_error = 0;
      prev_seq = 0;
      updated = "<p> Parameter updated.</p>";
    }
  }

  if (httpServer.hasArg("symbolwait"))
  {
    param = httpServer.arg("symbolwait").toInt();
    if ((param > 0) && (param <= 20) && (symbolwait != param))
    {
      symbolwait = param;
      pkt_error = 0;
      prev_seq = 0;
      updated = "<p> Parameter updated.</p>";
    }
  }

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
      "<iframe src=\"./stats.html\" width=\"320\" height=\"200\"></iframe>"
      "<form action=\"\" method=\"get\">"
      "<p> Queue Latency:"
      " <input type=\"textarea\" size=\"5\" name=\"queuelatency\" value=\"" +
      String(queuelatency) +
      "\">"
      "<br>"
      "Symbol Wait:"
      " <input type=\"textarea\" size=\"5\" name=\"symbolwait\" value=\"" +
      String(symbolwait) +
      "\">"
      "<br>"
      "<input type=\"submit\" name=\"submit\" value=\"Set\"></p>"
      "</form><hr>" +
      updated + "</body></html>";

  httpServer.send(200, "text/html", response);
}

void handleStats(void)
{
  String response;
  float estimated_wpm = 0, estimated_ratio = 0;

  if (short_mark > 0)
  {
    if (short_mark < MAX_MARK_DURAION)
      estimated_wpm = 1200 / short_mark;
    estimated_ratio = long_mark / short_mark;
  }
  short_mark = MAX_MARK_DURAION;

  response =
      "<!DOCTYPE html>"
      "<html>"
      "<head>"
      " <meta charset=\"utf-8\">"
      " <meta http-equiv=\"Refresh\" content=\"3\">"
      "</head>"
      "<body>"
      "<p>Estimated Speed: " +
      String(estimated_wpm, 0) +
      " WPM<br>"
      "Estimated Dash Dot Ratio: " +
      String(estimated_ratio, 1) +
      "<br>"
      "Packet Error: " +
      String(pkt_error) +
      "<br>"
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
  pkt_delay = 0;
  httpServer.send(200, "text/html", response);
}

void handleNotFound(void)
{
  httpServer.send(404, "text/plain", "404 Not Found.");
}

void wifi_setup()
{
  connected = false;
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
      WiFi.softAPConfig(ap_server, ap_server, subnet);
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
    short_mark = MAX_MARK_DURAION;

    httpServer.on("/", handleRoot);
    httpServer.on("/stats.html", handleStats);
    httpServer.onNotFound(handleNotFound);
    httpServer.begin();

    Serial.println("Server Ready");
  }
  else
  {
    if (ap_mode)
    {
      WiFi.mode(WIFI_STA);
      WiFi.config(client, ap_server, subnet);
      receiver = ap_server;

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
      receiver = server;

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
  keystate = NONE;
  prev_seq = 0;
  seq_number = 0;
  lastqueued = 0;
  lastmillis = 0;
  stableperiod = 0;
  key_changed = false;
}

void IRAM_ATTR keyIn()
{
  unsigned long now = millis();

  if ((now - stableperiod) < STABLEPERIOD)
    return;
  stableperiod = now;

  if (digitalRead(gpio_key) == HIGH)
  {
    key_edge = RISE_EDGE;
  }
  else
  {
    key_edge = FALL_EDGE;
    lastmillis = now;
  }
  key_time = now;
  key_duration = now - lastmillis;
  key_changed = true;
}

void debug_print(const char *msg, DotDash &d)
{
#ifdef DEBUG
  Serial.printf("%s:[seq:%d] prev=%d, keystate=%d, edge=%d, t=%d, d=%d space= %d, shortest=%d longest=%d\n",
                msg, d.seq, prev_seq, keystate, d.data, d.t, d.d, space_duration, short_mark, long_mark);
#endif
}

void send_udp(DotDash &d)
{
  wudp.beginPacket(receiver, udpport);
  wudp.write((uint8_t *)&d, sizeof(d));
  wudp.endPacket();
}

unsigned long last_received = 0, last_received_pkt = 0;

int receive_udp(DotDash &d)
{
  unsigned long now;

  int psize = wudp.parsePacket();
  if (psize > 0)
  {
    now = millis();
    wudp.read((uint8_t *)&d, sizeof(d));
    wudp.flush();
    pkt_delay = (now - last_received) - (d.t - last_received_pkt);
    last_received = now;
    last_received_pkt = d.t;
  }
  return psize;
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
void toggleKeyEdge(int psize, DotDash &d)
{
  unsigned long now = millis();

  if ((now - lastmark) > MAXDURATION)
    space();

  if (psize > 0)
  {
    if (d.data == RISE_EDGE)
    {
      space();
    }
    else
    {
      mark();
      lastmark = now;
    }
  }
}

void toggleKeyTime(int psize, DotDash &d)
{
  unsigned long now = millis();
  static unsigned long duration, startperiod;
  static unsigned long prev_t, prev_d;

  if (psize > 0)
  {
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

    if (d.d > long_mark)
      long_mark = d.d;
    if ((d.d > 0) && (d.d < short_mark))
      short_mark = d.d;

    queue.enqueue(d);
    lastqueued = now;
  }

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
        short_mark = MAX_MARK_DURAION;
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
      lastmark = now;
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
  DotDash d;

  if (!connected)
  {
    wifi_setup();
  }
  else if (server_mode)
  {
    int psize = receive_udp(d);

    if (udp_send_edge)
      toggleKeyEdge(psize, d);
    else {
      toggleKeyTime(psize, d);
      httpServer.handleClient();
    } 
  }
  else if (key_changed)
  {
    key_changed = false;

    if (udp_send_edge || (key_edge == RISE_EDGE))
    {
      d.type = PKT_CODE;
      d.data = key_edge;
      d.t = key_time;
      d.d = key_duration;
      d.seq = seq_number++;
      send_udp(d);
    }

    if (key_edge == RISE_EDGE)
      digitalWrite(gpio_led, LOW);
    else
      digitalWrite(gpio_led, HIGH);
  }
}
