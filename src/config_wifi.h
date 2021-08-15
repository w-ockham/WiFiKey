/* WiFi & IP Address Configrations */
/* SSID & Password */
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
IPAddress server(192, 168, 1, 192);
IPAddress ap_server(192, 168, 4, 1);
IPAddress client(192, 168, 4, 2);
IPAddress subnet(255, 255, 255, 0);
int httpport = 80;
int udpport = 56000;
