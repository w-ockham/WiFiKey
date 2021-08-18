/* WiFi & IP Address Configrations */
/* Keyer ID and password */
const char *keyer_code = "123-456-789";
const char *keyer_passwd = "Passwd";

/* WiFi SSIDs and passwords */
/* Standalone Access Point */
const char *ap_ssid = "ESP32-WiFiKey";
const char *ap_passwd = "wifikey32";

/* Server */
const char *server_ssid[] = {
    "SSID", 
    NULL};
const char *server_passwd[] = {
    "PASSWD",
    NULL};

/* Client */
const char *client_ssid[] = {
    "SSID1", 
    "SSID2",
    NULL};
const char *client_passwd[] = {
    "PASSWD1",
    "PASSWD2",
    NULL};

/* IP Addresses */
/* Standalone */
IPAddress ap_server(192, 168, 4, 1);
IPAddress ap_client(192, 168, 4, 2);

/* Server global address */
IPAddress global_server(192, 168, 1, 192);
int global_udpport = 56000;

/* Server local address */
IPAddress local_server(192, 168, 1, 192);
IPAddress local_gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
int local_httpport = 80;
int local_udpport = 56000;
