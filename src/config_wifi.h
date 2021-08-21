/* WiFi & IP Address Configrations */
/* Keyer ID and password */
const char *keyer_name = "wifikey";
const char *keyer_passwd = "Passwd";
const char *server_name = "wifikey";

/* WiFi SSIDs and passwords */
/* for Standalone Access Point */
const char *ap_ssid = "ESP32-WiFiKey";
const char *ap_passwd = "wifikey32";

/* for WiFi Station  */
const char *ssid[] = {
    "SSID1",
    "SSID2",
    NULL};
const char *passwd[] = {
    "PASSWD1",
    "PASSWD2",
    NULL};

/* IP Addresses */
/* Standalone */
IPAddress ap_server(192, 168, 4, 1);
IPAddress ap_client(192, 168, 4, 2);
IPAddress ap_subnet(255, 255, 255, 0);

/* Server global address */
const char *keyer_global = "192.168.1.192";
const int keyer_global_port = 56000;
/* Server local port */
const int keyer_local_port = 56000;
