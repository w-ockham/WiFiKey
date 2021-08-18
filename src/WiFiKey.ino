#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebServer.h>
#include <MD5Builder.h>

#define DEBUG_LEVEL 1
#define DROPPKT 0

/* GPIO setting */
const uint8_t gpio_key = 14;   /* from Keyer */
const uint8_t gpio_apcfg = 25; /* WiFi mode H = via AP, L = Standalone AP */
const uint8_t gpio_pkcfg = 12; /* Packet Type H = Duration and Time, L = Edge */
const uint8_t gpio_led = 26;   /* to LED */
const uint8_t gpio_out = 27;   /* to Photocoupler */

/* Packet Type & Queue params. */
#define SYMBOLWAIT 4           /* Wait four symbol */
#define QLATENCY 2             /* Process queued symbol if ((mark and space duriotn) * qlatency (msec) elapsed. */
#define MAX_MARK_DURATION 5000 /* Maximum MARK duration */

/* WiFi and IP Address configrations */
#include "config_wifi.h"

/* Session Parameters */
#define KEEPALIVE 15000     /* Send keep-alive to server every 15sec */
#define AUTHTIMEOUT 5000    /* Authentication timeout (5sec) */
#define IDLETIMEOUT 1800000 /* Idle timeout (30min)  */

/* Simple Ring Buffer */
#include "ringqueue.h"

/* Global variables */
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

WiFiUDP wudp;
WiFiMulti wifiMulti;
WebServer httpServer(local_httpport);

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
  PKT_SERIAL,
  PKT_SERIAL_RESENT
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

struct KeyerPkt
{
  PktType type;
  union
  {
    DotDash data;
    uint8_t hash[16];
    uint8_t buff[16];
    long seed;
  };
};

RingQueue<DotDash> queue = RingQueue<DotDash>();
RingQueue<DotDash> send_queue = RingQueue<DotDash>();
unsigned long lastqueued;

/* ISR Stuff */
#define STABLEPERIOD 5 /* Discard Interrupts 5msec */
volatile EdgeType key_edge;
volatile boolean key_changed;
volatile unsigned long key_time, key_duration;
volatile unsigned long stableperiod, lastmillis;
void IRAM_ATTR keyIn(void);

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
      "<iframe src=\"./stats.html\" width=\"320\" height=\"220\"></iframe>"
      "<hr>"
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
  String response, keyer;
  float estimated_wpm = 0, estimated_ratio = 0;

  if (short_mark > 0)
  {
    if (short_mark < MAX_MARK_DURATION)
      estimated_wpm = 1200 / short_mark;
    estimated_ratio = long_mark / short_mark;
  }
  short_mark = MAX_MARK_DURATION;

  if (serverstate == KEYER_ACTIVE)
    keyer = "Client: " + authsender.toString() + ":" + String(authport) + "<br>";
  else
    keyer = "";

  response =
      "<!DOCTYPE html>"
      "<html>"
      "<head>"
      " <meta charset=\"utf-8\">"
      " <meta http-equiv=\"Refresh\" content=\"3\">"
      "</head>"
      "<body><p>" +
      keyer +
      "Estimated Speed: " +
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
    data = String(p.seed);
    break;
  case PKT_CODE:
    type = "CODE";
    data = "durion=" + String(p.data.d) + " seq=" + String(p.data.seq);
    break;
  case PKT_CODE_RESENT:
    type = "RESENT";
    data = "durion=" + String(p.data.d) + " seq=" + String(p.data.seq);
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
  send_udp(recipient, port, k);
}

void resend_code(IPAddress recipient, int port, int seq)
{
  if (_resend_buffer[seq % 16].data.seq == seq)
  {
    _resend_buffer[seq % 16].type = PKT_CODE_RESENT;
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

void handle_code(PktType, DotDash &);
void handle_code_loop(void);

void process_incoming_packet(void)
{
  static long keyseed;
  KeyerPkt k;
  unsigned long now;
  IPAddress ip;
  int port, psize;

  psize = wudp.parsePacket();

  if (psize > 0)
  {
    now = millis();
    wudp.read((uint8_t *)&k, sizeof(k));
    wudp.flush();

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
          keyseed = esp_random();
          k.seed = keyseed;
          k.type = PKT_AUTH;
          send_udp(wudp.remoteIP(), wudp.remotePort(), k);
          settimeout(authtimer);
          serverstate = KEYER_AUTH;
          debug_sem("send", k);
          break;
        case PKT_CODE:
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
          MD5Builder md5;
          md5.begin();
          md5.add(keyer_code);
          md5.add(String(k.seed));
          md5.add(keyer_passwd);
          md5.calculate();
          md5.getBytes(k.hash);
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
          md5.add(keyer_code);
          md5.add(String(keyseed));
          md5.add(keyer_passwd);
          md5.calculate();
          md5.getBytes(buff);

          if (memcmp(buff, k.hash, 16) == 0)
          {
            authsender = wudp.remoteIP();
            authport = wudp.remotePort();
            k.type = PKT_ACK;
            send_udp(authsender, authport, k);
            settimeout(idletimer);
            serverstate = KEYER_ACTIVE;
            debug_sem("send", k);
          }
          else
          {
            k.type = PKT_NACK;
            send_udp(wudp.remoteIP(), wudp.remotePort(), k);
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
          MD5Builder md5;
          md5.begin();
          md5.add(keyer_code);
          md5.add(String(k.seed));
          md5.add(keyer_passwd);
          md5.calculate();
          md5.getBytes(k.hash);
          k.type = PKT_AUTH;
          send_udp(keying_server, keying_server_port, k);
          break;

          /* Success server authentication */
        case PKT_ACK:
          serverstate = KEYER_ACTIVE;
          settimeout(keepalivetimer);
          break;

          /* Server authentication failed */
        case PKT_NACK:
          serverstate = KEYER_WAIT;
          break;
        }
      }
      if (timeout(authtimer, AUTHTIMEOUT))
        serverstate = KEYER_WAIT;
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
          resend_code(keying_server, keying_server_port, k.data.seq);
        break;

      /* Connection closed by peer */
      case PKT_RST:
        if (forbidden_reset_outside && server_mode)
        {
          if (authsender == wudp.remoteIP() && authport == wudp.remotePort())
            serverstate = KEYER_WAIT;
        }
        else
        {
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

  /* Periodical process*/
  switch (serverstate)
  {
  case KEYER_AUTH:
    if (timeout(authtimer, AUTHTIMEOUT))
    {
      Serial.println("Authentication timed out");
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
        if (duration > MAX_MARK_DURATION)
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
        if (duration > MAX_MARK_DURATION)
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
      /* Drop 10% of incoming packet */
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
        send_nack(authsender, authport, prev_seq + 1);
      }
      prev_seq = d.seq;

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
    }
  }
}

void wifi_setup()
{
  IPAddress localID;

  connected = false;
  digitalWrite(gpio_led, HIGH);
  WiFi.disconnect(true);
  WiFi.onEvent(handleWiFiEvent);

  localID = local_server;
  for (int i = 0; i < 4; i++)
    localID[i] = localID[i] & subnet[i];

  if (server_mode)
  {
    if (ap_mode)
    {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(ap_ssid, ap_passwd);
      delay(100);
      WiFi.softAPConfig(ap_server, ap_server, subnet);
      Serial.print("Access point address: ");
      Serial.println(WiFi.softAPIP());
    }
    else
    {
      WiFi.config(local_server, local_gateway, subnet);

      for (int i = 0; server_ssid[i] != NULL; i++)
        wifiMulti.addAP(server_ssid[i], server_passwd[i]);

      while (wifiMulti.run(5000) != WL_CONNECTED)
      {
        delay(500);
        Serial.println("Connecting to AP...");
      }

      Serial.println("Connected.");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    }

    wudp.begin(local_udpport);

    pkt_error = 0;
    short_mark = MAX_MARK_DURATION;

    httpServer.on("/", handleRoot);
    httpServer.on("/stats.html", handleStats);
    httpServer.onNotFound(handleNotFound);
    httpServer.begin();

    Serial.println("Keyer server ready");
  }
  else
  {
    if (ap_mode)
    {
      WiFi.mode(WIFI_STA);
      WiFi.config(ap_client, ap_server, subnet);
      keying_server = ap_server;
      keying_server_port = local_udpport;
      WiFi.begin(ap_ssid, ap_passwd);
      while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
        Serial.println("Connecting to standalone AP...");
      }
      Serial.println("Connected.");
    }
    else
    {
      for (int i = 0; client_ssid[i] != NULL; i++)
        wifiMulti.addAP(client_ssid[i], client_passwd[i]);

      while (wifiMulti.run(5000) != WL_CONNECTED)
      {
        delay(500);
        Serial.println("Connecting to AP...");
      }

      Serial.println("Connected.");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());

      if (localID == WiFi.networkID())
      {
        Serial.println("Found a keying server in the same network.");
        keying_server = local_server;
        keying_server_port = local_udpport;
      }
      else
      {
        keying_server = global_server;
        keying_server_port = global_udpport;
      }
    }

    wudp.begin(local_udpport);
    Serial.println("Keyer client ready");
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

  clear_udp_buffer();
  keystate = NONE;
  serverstate = KEYER_WAIT;
  prev_seq = 0;
  seq_number = 0;
  lastqueued = 0;
  lastmillis = 0;
  stableperiod = 0;
  key_changed = false;
}

void start_auth()
{
  KeyerPkt k;

  Serial.println("Start authentication.");

  k.type = PKT_RST;
  send_udp(keying_server, keying_server_port, k);

  k.type = PKT_SYN;
  send_udp(keying_server, keying_server_port, k);

  settimeout(authtimer);
  serverstate = KEYER_AUTH;
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
    httpServer.handleClient();

    if (server_mode)
    { /* Process Keyer Server Loop */
      switch (serverstate)
      {
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
      switch (serverstate)
      {
      case KEYER_WAIT:
        if (key_changed)
          start_auth();
        break;

      case KEYER_ACTIVE:
        if (!send_queue.isEmpty())
        {
          DotDash d;
          send_queue.dequeue(d);
          send_code(keying_server, keying_server_port, d);
          settimeout(keepalivetimer);
        };
        break;
      }

      /* Enqueue keyinputs */
      if (key_changed)
      {
        key_changed = false;
        if (udp_send_edge || (key_edge == RISE_EDGE))
        {
          DotDash d;
          d.data = key_edge;
          d.t = key_time;
          d.d = key_duration;
          d.seq = seq_number++;
          send_queue.enqueue(d);
        }

        if (key_edge == RISE_EDGE)
          digitalWrite(gpio_led, LOW);
        else
          digitalWrite(gpio_led, HIGH);
      }
    }
  }
}