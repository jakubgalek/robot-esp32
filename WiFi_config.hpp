#include <WiFi.h>

#define WIFI_SSID ""
#define WIFI_PASSWORD ""

#define AP_SSID ""
#define AP_PASSWORD ""


const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

const char* ssid_ap = AP_SSID;
const char* password_ap = AP_PASSWORD;

void startWiFi() {

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Configuration AP
  //WiFi.mode(WIFI_AP);
  //WiFi.softAP(ssid_ap, password_ap);

}


String signalStrength;
static String lastSignalStrength = "";

void checkWiFiSignal() {

  int rssi = WiFi.RSSI();

  if (rssi >= -50) {
    signalStrength = "Swietne";
  } else if (rssi >= -60) {
    signalStrength = "Dobre";
  } else if (rssi >= -70) {
    signalStrength = "Srednie";
  } else {
    signalStrength = "Slabe";
  }
}