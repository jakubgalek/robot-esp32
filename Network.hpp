#pragma once
// ---------------------------------------------------------------------------
// Wi-Fi z ekranu robota: sieć wpisana na ekranie jest zapamiętywana w NVS i ma pierwszeństwo przed domyślną
// z WiFi_config.hpp. Łączenie i skanowanie sieci są nieblokujące (stan sprawdza net_poll() w loop()).
// Wymaga z WiFi_config.hpp: ssid, password, ssid_ap, password_ap, WIFI_CONNECT_TIMEOUT_MS.
// ---------------------------------------------------------------------------
#include <WiFi.h>
#include <Preferences.h>
#include <string.h>
#include <time.h>
#include "Settings.hpp"
#include "Time.hpp"

enum NetState { NET_IDLE, NET_CONNECTING, NET_OK, NET_FAIL };
volatile NetState netState = NET_IDLE;

static char netActiveSsid[33] = "";    // sieć, z którą robot jest (lub próbuje być) połączony
static char netActivePass[65] = "";
static char netTrySsid[33] = "", netTryPass[65] = "";
static unsigned long netTryStart = 0;

// ---- wczytanie zapisanej sieci (NVS), a gdy jej nie ma - domyślnej z WiFi_config.hpp
static void netLoad(char* s, char* p) {
  s[0] = 0; p[0] = 0;
  Preferences pr;
  if (pr.begin("net", true)) {
    String a = pr.getString("ssid", ""), b = pr.getString("pass", "");
    pr.end();
    if (a.length() > 0 && a.length() < 33 && b.length() < 65) { strcpy(s, a.c_str()); strcpy(p, b.c_str()); return; }
  }
  strncpy(s, ssid, 32); s[32] = 0;
  strncpy(p, password, 64); p[64] = 0;
}

bool net_hasSaved() {
  Preferences pr;
  bool has = false;
  if (pr.begin("net", true)) { has = pr.getString("ssid", "").length() > 0; pr.end(); }
  return has;
}

// ---- tryb pracy: 0 = sieć (domyślnie; gdy się nie połączy przez dłuższy czas, włącza się awaryjnie AP), 1 = tylko własny punkt dostępowy (AP)
#define NET_FALLBACK_MS 120000UL           // tyle bez połączenia i robot dodatkowo uruchamia AP
static uint8_t netMode = 0;
static bool netApOn = false;
static char netApS[33] = "", netApP[65] = "";
static unsigned long netDownSince = 0, netUpSince = 0;

// Dane AP: z WiFi_config.hpp (AP_SSID / AP_PASSWORD >= 8 znaków), a gdy ich tam nie ma - nazwa "Robot-XXXX" i hasło losowe,
// zapamiętane w NVS (widać je na ekranie Wi-Fi). Sieć otwarta nie jest nigdy uruchamiana.
static void netApInit() {
  if (netApS[0]) return;
  if (strlen(ssid_ap) > 0 && strlen(password_ap) >= 8) {
    strncpy(netApS, ssid_ap, 32); netApS[32] = 0; strncpy(netApP, password_ap, 64); netApP[64] = 0;
    return;
  }
  uint8_t m[6]; WiFi.macAddress(m);
  snprintf(netApS, sizeof(netApS), "Robot-%02X%02X", m[4], m[5]);
  Preferences pr;
  String p;
  if (pr.begin("net", true)) { p = pr.getString("appass", ""); pr.end(); }
  if (p.length() < 8) {
    char g[12]; snprintf(g, sizeof(g), "%08u", (unsigned)(esp_random() % 100000000UL));
    p = String(g);
    if (pr.begin("net", false)) { pr.putString("appass", p); pr.end(); }
  }
  strncpy(netApP, p.c_str(), 64); netApP[64] = 0;
}

static void netStartAp(bool keepSta) {
  netApInit();
  WiFi.mode(keepSta ? WIFI_AP_STA : WIFI_AP);
  WiFi.softAP(netApS, netApP);
  netApOn = true;
  Serial.print("AP: "); Serial.print(netApS); Serial.print(" IP: "); Serial.println(WiFi.softAPIP());
}

static void netStopAp() {
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  netApOn = false;
}

uint8_t net_mode() { return netMode; }
const char* net_apSsid() { netApInit(); return netApS; }
const char* net_apPass() { netApInit(); return netApP; }
bool net_apOn() { return netApOn; }

// Adres do pokazania: z routera, a gdy go nie ma - adres własnego AP
IPAddress net_ip() {
  if (WiFi.status() == WL_CONNECTED) return WiFi.localIP();
  if (netApOn) return WiFi.softAPIP();
  return IPAddress(0, 0, 0, 0);
}

// Start systemu: blokuje najwyżej WIFI_CONNECT_TIMEOUT_MS (na ekranie widać "Laczenie...")
void net_begin() {
  netLoad(netActiveSsid, netActivePass);
  {
    Preferences pr;
    if (pr.begin("net", true)) { netMode = pr.getUChar("mode", 0) ? 1 : 0; pr.end(); }
  }
  WiFi.setAutoReconnect(true);   // sam wróci do sieci po zerwaniu połączenia
  if (netMode == 1) { netStartAp(false); return; }
  WiFi.mode(WIFI_STA);
  if (!netActiveSsid[0]) { netStartAp(true); return; }
  WiFi.begin(netActiveSsid, netActivePass);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < WIFI_CONNECT_TIMEOUT_MS) delay(250);
  if (WiFi.status() == WL_CONNECTED) { Serial.print("IP: "); Serial.println(WiFi.localIP()); return; }
  Serial.println("WiFi: timeout");
  netStartAp(true);              // AP awaryjnie, a robot dalej próbuje wrócić do sieci
}

// Zmiana trybu z ekranu (zapamiętywana)
void net_setMode(uint8_t m) {
  m = m ? 1 : 0;
  Preferences pr;
  if (pr.begin("net", false)) { pr.putUChar("mode", m); pr.end(); }
  if (m == netMode && (m == 0 || netApOn)) return;
  netMode = m;
  if (m == 1) {
    WiFi.disconnect(false);
    netStartAp(false);
  } else {
    if (netApOn) netStopAp();
    netDownSince = 0;
    if (netActiveSsid[0]) WiFi.begin(netActiveSsid, netActivePass[0] ? netActivePass : NULL); else netStartAp(true);
  }
}

// Nazwa sieci do wyświetlenia ("" = brak): sieć z routera albo nazwa własnego AP
String net_ssid() {
  if (WiFi.status() == WL_CONNECTED) return WiFi.SSID();
  return netApOn ? String(netApS) : String("");
}

// Próba połączenia z nową siecią (nieblokująca). Dane zapisujemy dopiero po udanym połączeniu.
void net_connect(const char* s, const char* p) {
  strncpy(netTrySsid, s, 32); netTrySsid[32] = 0;
  strncpy(netTryPass, p, 64); netTryPass[64] = 0;
  if (netApOn) netStopAp();
  if (netMode != 0) { netMode = 0; Preferences pr; if (pr.begin("net", false)) { pr.putUChar("mode", 0); pr.end(); } }
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false);
  WiFi.begin(netTrySsid, netTryPass[0] ? netTryPass : NULL);
  netTryStart = millis();
  netState = NET_CONNECTING;
}

// ---- czas z internetu (NTP): jednorazowo po włączeniu przełącznika, potem raz na dobę
static bool ntpWaiting = false, ntpEverOk = false;
static int8_t ntpResult = 0;                       // 0 brak, 1 ok, -1 błąd
static unsigned long ntpStartMs = 0, ntpLastOkMs = 0, ntpTryMs = 0;

void ntp_request() {
  ntpTryMs = millis();
  if (WiFi.status() != WL_CONNECTED) { ntpResult = -1; return; }
  configTzTime(NTP_TZ, "pool.ntp.org", "time.google.com");
  ntpWaiting = true; ntpStartMs = millis(); ntpResult = 0;
}

const char* ntp_status() {
  if (ntpWaiting) return "Laczenie...";
  if (ntpResult == 1) return "Zsynchronizowano";
  if (ntpResult == -1) return "Blad - brak sieci?";
  return "Strefa: Warszawa";
}

static void ntp_poll() {
  unsigned long now = millis();
  if (cfgNtp && !ntpWaiting && WiFi.status() == WL_CONNECTED &&
      ((!ntpEverOk && (ntpTryMs == 0 || now - ntpTryMs > 60000UL)) || (ntpEverOk && now - ntpLastOkMs > 86400000UL))) ntp_request();
  if (!ntpWaiting) return;
  time_t t = time(NULL);
  struct tm lt;
  localtime_r(&t, &lt);
  if (lt.tm_year + 1900 >= 2024) {
    if (Time_set(lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec)) { ntpResult = 1; ntpEverOk = true; ntpLastOkMs = now; }
    else ntpResult = -1;
    ntpWaiting = false;
  } else if (now - ntpStartMs > 10000UL) { ntpWaiting = false; ntpResult = -1; }
}

// Wołać w loop()
void net_poll() {
  ntp_poll();
  unsigned long now = millis();
  if (netState != NET_CONNECTING && netMode == 0) {
    bool up = WiFi.status() == WL_CONNECTED;
    if (!up && !netApOn) {                                   // dłuższy brak sieci -> AP awaryjnie
      if (!netDownSince) netDownSince = now;
      else if (now - netDownSince > NET_FALLBACK_MS) netStartAp(true);
    } else if (up) netDownSince = 0;
    if (netApOn && up) {                                      // sieć wróciła -> wyłącz AP po 30 s
      if (!netUpSince) netUpSince = now;
      else if (now - netUpSince > 30000UL) { netStopAp(); netUpSince = 0; }
    } else netUpSince = 0;
  }
  if (netState != NET_CONNECTING) return;
  unsigned long dt = millis() - netTryStart;
  int st = WiFi.status();
  if (st == WL_CONNECTED) {
    Preferences pr;
    if (pr.begin("net", false)) { pr.putString("ssid", netTrySsid); pr.putString("pass", netTryPass); pr.end(); }
    strcpy(netActiveSsid, netTrySsid); strcpy(netActivePass, netTryPass);
    netState = NET_OK;
  } else if (dt > 15000 || (dt > 3000 && (st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL))) {
    // nie udało się: wróć do poprzedniej sieci
    if (netActiveSsid[0]) WiFi.begin(netActiveSsid, netActivePass[0] ? netActivePass : NULL); else netStartAp(true);
    netState = NET_FAIL;
  }
}

// Zapomnij zapisaną sieć (po restarcie wróci domyślna z WiFi_config.hpp, jeśli jest)
void net_forget() {
  Preferences pr;
  if (pr.begin("net", false)) { pr.remove("ssid"); pr.remove("pass"); pr.end(); }
  netActiveSsid[0] = 0; netActivePass[0] = 0;
  WiFi.disconnect(false);
  netState = NET_IDLE;
}

// ---- skanowanie sieci (asynchroniczne)
struct NetEntry { char ssid[33]; int8_t rssi; bool secure; };
#define NET_LIST_MAX 12
static NetEntry netList[NET_LIST_MAX];
static int netListN = 0;
static bool netScanning = false;

void net_scanStart() {
  WiFi.scanDelete();
  WiFi.scanNetworks(true, false);
  netScanning = true;
  netListN = 0;
}

// true gdy skan się skończył (lista w netList / netListN)
bool net_scanPoll() {
  if (!netScanning) return false;
  int n = WiFi.scanComplete();
  if (n == -1) return false;                 // trwa
  netScanning = false;
  if (n < 0) { netListN = 0; return true; }  // błąd
  netListN = 0;
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    if (s.length() == 0 || s.length() > 32) continue;
    int dup = -1;
    for (int j = 0; j < netListN; j++) if (strcmp(netList[j].ssid, s.c_str()) == 0) { dup = j; break; }
    int8_t r = (int8_t)WiFi.RSSI(i);
    bool sec = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
    if (dup >= 0) { if (r > netList[dup].rssi) netList[dup].rssi = r; continue; }
    int pos;
    if (netListN < NET_LIST_MAX) pos = netListN++;
    else {                                     // lista pełna: zastąp najsłabszą, jeśli ta jest silniejsza
      pos = 0;
      for (int j = 1; j < netListN; j++) if (netList[j].rssi < netList[pos].rssi) pos = j;
      if (r <= netList[pos].rssi) continue;
    }
    strcpy(netList[pos].ssid, s.c_str()); netList[pos].rssi = r; netList[pos].secure = sec;
  }
  for (int a = 0; a < netListN; a++)           // sortowanie: najsilniejsze na górze
    for (int b = a + 1; b < netListN; b++)
      if (netList[b].rssi > netList[a].rssi) { NetEntry t = netList[a]; netList[a] = netList[b]; netList[b] = t; }
  WiFi.scanDelete();
  return true;
}
