/* WiFi & IP Address Configrations */
/* Keyer ID and password */
const char *keyer_name = "wifikey";
const char *keyer_passwd = "Passwd";
const char *server_name = "wifikey";

/* WiFi SSIDs and passwords */
/* Standalone Access Point */
const char *ap_ssid = "ESP32-WiFiKey";
const char *ap_passwd = "wifikey32";

/* for WiFi Station  */
const char *ssid[] = {
    "Pixel5",
    "Buffalo-85C0",
    NULL};
const char *passwd[] = {
    "wifikey32",
    "rvrwyfgjsj7vs",
    NULL};

/* IP Addresses */
/* Standalone */
IPAddress ap_server(192, 168, 4, 1);
IPAddress ap_client(192, 168, 4, 2);
IPAddress ap_subnet(255, 255, 255, 0);

/* Server address:port */
const char *keyer_global = "minecraft.penpen.tk";
const int keyer_global_port = 50704;
const int keyer_local_port = 56000;
