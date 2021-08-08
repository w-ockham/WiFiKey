#include <WiFi.h>
#include <WiFiUDP.h>

#define DEBUG 1

/* GPIO setting */
const uint8_t gpio_key = 14;   /* from Keyer */
const uint8_t gpio_apcfg = 25; /* WiFi mode H = via AP, L = Standalone AP */
const uint8_t gpio_pkcfg = 12; /* Packet Type H = Duration and Time, L = Edge */
const uint8_t gpio_led = 26;   /* to LED */
const uint8_t gpio_out = 27;   /* to Photocoupler */

/* SSID & Password */
const char *ssid[] = {
    "ESP32-WiFiKey", /* Standalone */
    "YOURSSID"       /* WiFi AP SSID */
};

const char *passwd[] = {
    "wifikey32", /* Standalone */
    "YOURPASSWD" /* WiFi AP Password */
};

/* IP Addresses */
const IPAddress server(192, 168, 1, 192);
const IPAddress myip(192, 168, 4, 2);
const IPAddress gateway(192, 168, 4, 1);
const IPAddress subnet(255, 255, 255, 0);
const int udpport = 9000;

/* Packet Type & Queue params. */
#define SYMBOLWAIT 3      /* Wait three symbol */
#define QLATENCY 150      /* Process symbol 150 msec elapsed. */
#define MAXDURATION 10000 /* Maximum MARK duration */

/* Global variables */
boolean server_mode;
boolean ap_mode;
boolean udp_send_edge;
static WiFiUDP wudp;

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
#ifdef DEBUG
    Serial.println("Queue empty");
#endif
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
unsigned long seq_number;
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

void wifi_setup()
{
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
    Serial.println("Server Ready");
  }
  else
  {
    if (ap_mode)
    {
      WiFi.mode(WIFI_STA);
      WiFi.config(myip, gateway, subnet);
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
  Serial.printf("%s:[seq:%d] keystate=%d, edge=%d, t=%d, d=%d\n",
                msg, d.seq, keystate, d.edge, d.t, d.d);
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

static unsigned long prev_seq = 0;

void receive_udp()
{
  DotDash d;
  int psize = wudp.parsePacket();
  if (psize > 0)
  {
    wudp.read((uint8_t *)&d, sizeof(d));
    wudp.flush();

    if (d.seq == (prev_seq + 1))
      debug_print("UDP", d);
    else
      debug_print("UDPERR", d);

    prev_seq = d.seq;
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
    if (queLength() > SYMBOLWAIT || (now - lastpoll) > QLATENCY)
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
  if (server_mode)
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
        send_udp();
    }
  }
}
