#include <WiFi.h>

#define WIFI_SSID ""
#define WIFI_PASSWORD ""

#define AP_SSID ""
#define AP_PASSWORD ""

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

const char* ssid_ap = AP_SSID;
const char* password_ap = AP_PASSWORD;

const unsigned long WIFI_CONNECT_TIMEOUT_MS = 20000;

// Adres do wyświetlenia na TFT: w trybie AP adres punktu dostępowego, w STA adres z routera.
IPAddress getDisplayIP() {
  return (WiFi.getMode() == WIFI_AP) ? WiFi.softAPIP() : WiFi.localIP();
}

void startWiFi() {
  Serial.print("🔄Connecting to ");
  Serial.println(ssid);

  // Configuration ROUTER
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);   // sam wróci do sieci po zerwaniu połączenia
  WiFi.begin(ssid, password);

  // Wait for Wi-Fi connection (z limitem czasu - wcześniej pętla bez końca i bez delay)
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\n🌐IP address: ");
    Serial.println(WiFi.localIP());
    return;
  }

  Serial.println("\n❌WiFi connection timeout");

  // Tryb awaryjny: własny punkt dostępowy (tylko jeśli AP_SSID i hasło >= 8 znaków są ustawione;
  // nie uruchamiamy otwartej sieci sterującej robotem).
  if (strlen(ssid_ap) > 0 && strlen(password_ap) >= 8) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid_ap, password_ap);
    Serial.print("📡AP mode, IP address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("AP fallback not configured - robot keeps trying to reconnect in background");
  }
}


String signalStrength = "0/4";

void checkWiFiSignal() {
  // RSSI wystarczy odświeżać raz na sekundę
  static unsigned long lastCheck = 0;
  unsigned long now = millis();
  if (now - lastCheck < 1000) return;
  lastCheck = now;

  if (WiFi.status() != WL_CONNECTED) {
    signalStrength = "0/4";
    return;
  }

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
