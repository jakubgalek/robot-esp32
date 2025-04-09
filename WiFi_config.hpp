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
  Serial.print("🔄Connecting to ");
  Serial.println(ssid);

  // Configuration ROUTER
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait for Wi-Fi connection
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
  }

  Serial.print("\n🌐IP address: ");
  Serial.println(WiFi.localIP());
}


String signalStrength;
static String lastSignalStrength = "";

void checkWiFiSignal() {

  int rssi = WiFi.RSSI();

  if (rssi >= -50) {
    signalStrength = "4/4";
  } else if (rssi >= -60) {
    signalStrength = "3/4";
  } else if (rssi >= -70) {
    signalStrength = "2/4";
  } else {
    signalStrength = "1/4";
  }
}
