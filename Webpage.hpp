#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <SPIFFS.h>

#define INDEX "/index.htm"

// Jeśli przez tyle ms nie przyjdzie "keepalive" od trzymanego przycisku, silniki są zatrzymywane
// (zerwane WiFi, zamknięta karta, puszczony przycisk którego zdarzenie nie dotarło).
#define MOTOR_DEADMAN_MS 700

WebServer server(80);

const char* hostname = "robot";


boolean MOTOR_state[4] = {0};


// Prośba o zakończenie zadania auto-jazdy: autoDriveStopRequested (Explore.hpp). Zadanie kończy się
// samo (nie przez vTaskDelete z zewnątrz - to mogło zostawić zablokowany mutex czujników albo
// przerwaną transakcję I2C).

unsigned long lastMotorCmdMs = 0;
bool mdnsStarted = false;


void checkMotorsAndReloadBuzzer(){
  if (busy_motors != true) {
    Buzzer_reload();
  } else {
    Buzzer_silence();   // podczas jazdy cisza, ale ton nie może zostać włączony "na stałe"
  }
}

void startAutoDrive() {
  lastAutoMode = refineMode ? 4 : wallFollowMode ? 2 : exploreHomeMode ? 1 : 0;
  autoDriveStopRequested = false;
  maneuverAbort = false;
  AUTO_DRIVE_state = 1;
  if (AutomaticDriveTaskHandle == NULL) {
    xTaskCreatePinnedToCore(
      refineMode ? refine_Task : (wallFollowMode ? wallFollow_Task : explore_Task),   // jazda przy ścianie albo eksploracja mapy (Explore.hpp)
      "ExploreTask",
      7000,                       // stos
      NULL,
      1,                          // priorytet
      &AutomaticDriveTaskHandle,
      0                           // rdzeń
    );
  }
}

void requestAutoDriveStop() {
  AUTO_DRIVE_state = 0;
  autoDriveStopRequested = true;
  maneuverAbort = true;     // przerwij trwający obrót/jazdę
  scanAbort = true;         // i skan
}

// Sterowanie ręczne "jedź dopóki trzymam": stan przycisków pochodzi z MOTOR_state[],
// a przeglądarka wysyła keepalive co ~200 ms, dopóki przycisk jest wciśnięty.
void applyManualDrive() {
  if (MOTOR_state[0]) {
    if (!frontBlocked()) {   // progi w Config.hpp; brak świeżych danych = blokada
      ledcWrite(pwmChannelSpeed, motorSpeed);
      forward();
    } else {
      stop_driving();
    }
  } else if (MOTOR_state[1]) {
    // ten sam warunek co w checkDriveConditions()
    if (!rearBlocked()) {
      ledcWrite(pwmChannelSpeed, motorSpeed);
      backward();
    } else {
      stop_driving();
    }
  } else if (MOTOR_state[2]) {
    ledcWrite(pwmChannelSpeed, motorSpeed);
    turn_left();     // ciągły obrót dopóki trzymany (wcześniej blokujące turn(...,90,...) w obsłudze HTTP)
  } else if (MOTOR_state[3]) {
    ledcWrite(pwmChannelSpeed, motorSpeed);
    turn_right();
  } else {
    stop_driving();
  }
}

void SetMOTORs() {
    static const char* const names[4] = {"MOTOR1", "MOTOR2", "MOTOR3", "MOTOR4"};
    bool anyMotorArg = false;
    bool anyPressed = false;

    for (int i = 0; i < 4; i++) {
        if (server.hasArg(names[i])) {
            anyMotorArg = true;
            MOTOR_state[i] = server.arg(names[i]).toInt() ? 1 : 0;
            if (MOTOR_state[i]) anyPressed = true;
        }
    }

    if (anyMotorArg) {
        if (anyPressed) lastMotorCmdMs = millis();
        // Przy aktywnej auto-jazdzie ręczne komendy są ignorowane (wyłącz najpierw auto-jazdę).
        if (!AUTO_DRIVE_state) {
            applyManualDrive();
        }
    }

    // Speed Slider on website
    if (server.hasArg("Slider")) {
        int v = server.arg("Slider").toInt();
        motorSpeed = constrain(v, 0, (int)maxSpeed);   // Get speed value from the slider
        ledcWrite(pwmChannelSpeed, motorSpeed);        // Set the motor speed
    }

    // Automatic drive button on website
    if (server.hasArg("AUTO_DRIVE")) {
        if (server.arg("AUTO_DRIVE").toInt() == 1) {
            for (int i = 0; i < 4; i++) MOTOR_state[i] = 0;
            stop_driving();
            exploreHomeMode = false;
            wallFollowMode = false; refineMode = false;
            startAutoDrive();
        } else {
            requestAutoDriveStop();
        }
    }
}

// Sterowanie z panelu: krótka odpowiedź, bez pomiarów (telemetria idzie przez /api/state)
void ajaxInputs() {
  SetMOTORs();
  server.sendHeader("Connection", "close");
  server.sendHeader("Cache-Control", "no-store, must-revalidate");
  server.send(200, "text/plain", "ok");
}

// ---------------------------------------------------------------------------
// API panelu
// ---------------------------------------------------------------------------
static void sendJson(const char* body) {
  server.sendHeader("Connection", "close");
  server.sendHeader("Cache-Control", "no-store, must-revalidate");
  server.send(200, "application/json", body);
}

// Statystyki mapy liczymy tylko po zmianie mapy
static void mapStatsCached(uint32_t& occ, uint32_t& fr) {
  static uint32_t cachedVer = 0xFFFFFFFF, cOcc = 0, cFree = 0;
  MapLock lock;
  if (cachedVer != gridMap.version) {
    gridMap.stats(cOcc, cFree);
    cachedVer = gridMap.version;
  }
  occ = cOcc; fr = cFree;
}

int btKeyboardState();   // robot-esp32.ino: 1 = klawiatura Bluetooth podłączona, 0 = nie, -1 = nieznany

void apiState() {
  Pose2 p = odometry_get();
  uint32_t occ = 0, fr = 0;
  mapStatsCached(occ, fr);
  // punkty trasy eksploracji (cm)
  char wp[200];
  {
    float wx[PLAN_MAX_WP], wy[PLAN_MAX_WP];
    int n = Explore_copyWaypoints(wx, wy, PLAN_MAX_WP);
    int o = 0;
    wp[0] = 0;
    for (int i = 0; i < n && o < (int)sizeof(wp) - 20; i++) o += snprintf(wp + o, sizeof(wp) - o, "%s%d,%d", i ? "," : "", (int)lroundf(wx[i]), (int)lroundf(wy[i]));
  }
    // czujniki przeszkód: odległości [cm], wiek odczytu [ms], blokady (do podglądu w panelu)
  char sn[200];
  {
    int v[6] = {measurement2, measurement3, measurement4, measurement5, measurement6, measurement7};
    int o = snprintf(sn, sizeof(sn), "\"v\":[%d,%d,%d,%d,%d,%d],\"age\":[", v[0], v[1], v[2], v[3], v[4], v[5]);
    for (int i = 0; i < 6; i++) {
      long age = sensorAgeMs(i + 2);
      if (age > 9999) age = 9999;
      o += snprintf(sn + o, sizeof(sn) - o, "%s%ld", i ? "," : "", age);
    }
    snprintf(sn + o, sizeof(sn) - o, "],\"fb\":%d,\"rb\":%d,\"ff\":%d,\"bf\":%d",
             frontBlocked() ? 1 : 0, rearBlocked() ? 1 : 0, frontSensorsFresh() ? 1 : 0, backSensorFresh() ? 1 : 0);
  }
  char buf[2600];
  snprintf(buf, sizeof(buf),
    "{\"x\":%.1f,\"y\":%.1f,\"h\":%.1f,\"path\":%.0f,"
    "\"scan\":{\"on\":%d,\"pct\":%d,\"n\":%u,\"ov\":%u,\"match\":%d,\"dx\":%.1f,\"dy\":%.1f,\"dh\":%.1f,\"wd\":%.1f},"
    "\"map\":{\"ver\":%u,\"cells\":%d,\"cell\":%d,\"occ\":%u,\"free\":%u,\"dirty\":%d,\"loaded\":%d,\"err\":%d},"
    "\"auto\":%d,\"dir\":\"%s\",\"speed\":%d,"
    "\"vm\":%.2f,\"ve\":%.2f,\"temp\":%.1f,\"alt\":%d,\"wifi\":\"%s\","
    "\"robot\":{\"len\":%.1f,\"wid\":%.1f,\"sx\":%.1f,\"sy\":%.1f},"
    "\"ex\":{\"home\":%d,\"st\":%d,\"tx\":%.0f,\"ty\":%.0f,\"fr\":%d,\"wp\":[%s]},"
    "\"heap\":%u,\"minheap\":%u,\"gsign\":%d,"
    "\"sn\":{%s},\"gy\":{\"sign\":%d,\"bias\":%.2f,\"scale\":%.3f,\"dps\":%.1f,\"ready\":%d,\"cy\":%lu,\"cymax\":%lu},\"sh\":%.1f,\"bt\":%d,\"log\":%u,\"wf\":{\"on\":%d,\"k\":%d,\"d\":%d},\"am\":%d}",
    p.x, p.y, p.h, (double)odoPathCm,
    (int)scanning, (int)scanProgress, (unsigned)scanCount, (unsigned)scanOverlayVersion,
    (int)lastMatchResult, (double)lastMatchDx, (double)lastMatchDy, (double)lastMatchDh, (double)lastWallDelta,
    (unsigned)gridMap.version, MAP_CELLS, MAP_CELL_CM, (unsigned)occ, (unsigned)fr,
    (int)mapDirty, (int)mapLoadedFromFlash, (int)mapAllocFailed,
    AUTO_DRIVE_state ? 1 : 0, (const char*)direction, (int)motorSpeed,
    readVoltageMotors(), readVoltageESP32(), getFormattedTemperature(), getFormattedAltitude(), signalStrength.c_str(),
    (double)ROBOT_LENGTH_CM, (double)ROBOT_WIDTH_CM, (double)SERVO_SENSOR_OFFSET_FWD_CM, (double)SERVO_SENSOR_OFFSET_LEFT_CM,
    (int)exploreHomeMode, (int)exploreState, (double)exploreTx, (double)exploreTy, (int)exploreFrontiers, wp,
    (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(), (int)gyroSign,
    sn, (int)gyroSign, (double)gyroBiasDps, (double)gyroScale, (double)lastGyroDps, odoGyroReady ? 1 : 0, (unsigned long)cycleAvgMs, (unsigned long)cycleMaxMs, (double)gridMap.startHeading, btKeyboardState(), (unsigned)logSeqNo, (int)wallFollowMode, (int)wallFollowKind, (int)wallFollowDist, (int)lastAutoMode);
  sendJson(buf);
}

// Mapa: pary bajtów (liczba powtórzeń, klasa komórki: 0 nieznana, 1 wolna, 2 przeszkoda),
// komórki wierszami od y = najmniejszego (dół mapy) do największego.
void apiMap() {
  MapLock lock;
  size_t len = gridMap.encodeRle([](const uint8_t*, size_t) {});
  server.sendHeader("Connection", "close");
  server.sendHeader("Cache-Control", "no-store, must-revalidate");
  server.setContentLength(len);
  server.send(200, "application/octet-stream", "");
  gridMap.encodeRle([](const uint8_t* p, size_t l) { server.sendContent((const char*)p, l); });
}

// ---- Zapisane mapy (karta SD): lista, podgląd, zmiana nazwy, wczytanie, usunięcie
extern volatile bool sdOk;
int sd_listMaps(char names[][32], int maxN);
int sd_renameMap(const char* oldName, const char* newBase);
bool sd_deleteMap(const char* name);
bool sd_loadMapFile(const char* name);
bool sd_saveMapNamed();
bool sd_recheck();
PackedMap* sd_readMapCells(const char* name);

bool robotBusyNow();
static bool webMapNameOk(const String& f) {              // tylko nazwy z biblioteki: litery, cyfry, _ - . i koniec .bin
  const char* p = f.c_str();
  size_t L = strlen(p);
  if (L < 5 || L > 31 || strcmp(p + L - 4, ".bin") != 0) return false;
  for (size_t i = 0; i < L; i++) { char c = p[i]; if (!(isalnum((unsigned char)c) || c == '_' || c == '-' || c == '.')) return false; }
  return strstr(p, "..") == NULL;
}

static void apiMaps() {
  static char names[24][32];
  int n = sd_listMaps(names, 24);
  String j("{\"sd\":"); j += (sdOk ? "1" : "0"); j += ",\"maps\":[";
  bool first = true;
  for (int i = 0; i < n; i++) {
    if (!webMapNameOk(names[i])) continue;
    if (!first) j += ",";
    j += "\""; j += names[i]; j += "\""; first = false;
  }
  j += "]}";
  sendJson(j.c_str());
}

// Podgląd zapisanej mapy: ten sam format RLE co /api/map
static void apiMapView() {
  String f = server.arg("f");
  PackedMap* c = webMapNameOk(f) ? sd_readMapCells(f.c_str()) : NULL;
  if (!c) { server.send(404, "text/plain", "brak mapy"); return; }
  const int total = MAP_CELLS * MAP_CELLS;
  auto cls = [&](int i) -> uint8_t { return packedCls(c, i); };
  size_t len = 0;
  for (int i = 0; i < total;) { int e = i + 1; while (e < total && e - i < 255 && cls(e) == cls(i)) e++; len += 2; i = e; }
  server.sendHeader("Connection", "close");
  server.setContentLength(len);
  server.send(200, "application/octet-stream", "");
  uint8_t buf[128]; size_t bl = 0;
  for (int i = 0; i < total;) {
    int e = i + 1; while (e < total && e - i < 255 && cls(e) == cls(i)) e++;
    buf[bl++] = (uint8_t)(e - i); buf[bl++] = cls(i); i = e;
    if (bl >= sizeof(buf)) { server.sendContent((const char*)buf, bl); bl = 0; }
  }
  if (bl) server.sendContent((const char*)buf, bl);
  packedFree(c);
}

static void apiMapAct() {                                 // /api/maps/act?a=rename|load|delete|save&f=..&n=..
  String a = server.arg("a"), f = server.arg("f");
  const char* err = NULL;
  if (!sdOk && !sd_recheck()) err = "brak karty SD";
  else if (a == "save") { if (!sd_saveMapNamed()) err = "zapis sie nie udal"; }
  else if (!webMapNameOk(f)) err = "zla nazwa";
  else if (a == "rename") { int r = sd_renameMap(f.c_str(), server.arg("n").c_str()); if (r < 0) err = "taka nazwa juz istnieje"; else if (r == 0) err = "nie udalo sie zmienic nazwy"; }
  else if (a == "delete") { if (!sd_deleteMap(f.c_str())) err = "nie udalo sie usunac"; }
  else if (a == "load") { if (robotBusyNow()) err = "zatrzymaj robota"; else if (!sd_loadMapFile(f.c_str())) err = "nie udalo sie wczytac"; }
  else err = "nieznane polecenie";
  char buf[96];
  if (err) snprintf(buf, sizeof(buf), "{\"ok\":0,\"err\":\"%s\"}", err); else strcpy(buf, "{\"ok\":1}");
  sendJson(buf);
}

// Ostatni skan: punkty trafień w cm (układ mapy)
void apiScan() {
  static int16_t pts[SCAN_MAX_BEAMS][2];
  Pose2 sp;
  int n = Scan_copyOverlay(pts, SCAN_MAX_BEAMS, sp);
  String res;
  res.reserve(40 + n * 12);
  char b[48];
  snprintf(b, sizeof(b), "{\"pose\":[%.1f,%.1f,%.1f],\"pts\":[", sp.x, sp.y, sp.h);
  res += b;
  for (int i = 0; i < n; i++) {
    snprintf(b, sizeof(b), "%s%d,%d", i ? "," : "", pts[i][0], pts[i][1]);
    res += b;
  }
  res += "]}";
  sendJson(res.c_str());
}

void apiLog() {
  String j = Log_json();
  sendJson(j.c_str());
}

// Nowa mapa = nowy układ współrzędnych: obecne miejsce i kierunek robota to (0,0), "do przodu".
bool mapNewMap() {
  MapLock lock;
  gridMap.clear();
  Pose2 o = {0.0f, 0.0f, ROBOT_START_HEADING};
  odometry_set(o);
  scanOverlayCount = 0;
  scanOverlayVersion = scanOverlayVersion + 1;
  mapDirty = true;
  return true;
}

bool robotBusyNow() { return scanning || scanRequested || AUTO_DRIVE_state || busy_motors; }

// Polecenia robota (panel WWW i ekran): zwraca NULL gdy przyjęte, w przeciwnym razie tekst błędu.
// scan | locate | auto | wall | home | refine | stop | clear | save
const char* robotCommand(const char* c) {
  bool robotBusy = AUTO_DRIVE_state || busy_motors;
  if (!strcmp(c, "scan") || !strcmp(c, "locate")) {
    if (mapAllocFailed) return "brak pamieci na mape";
    if (scanning || scanRequested) return "skan juz trwa";
    if (robotBusy) return "robot jedzie - najpierw Stop";
    scanLocateRequested = (c[0] == 'l'); scanRequested = true;
  } else if (!strcmp(c, "stop")) {
    requestAutoDriveStop();
    scanAbort = true; scanRequested = false;
  } else if (!strcmp(c, "abort")) {
    scanAbort = true; scanRequested = false;
  } else if (!strcmp(c, "clear")) {
    if (scanning || robotBusy) return "zatrzymaj robota i poczekaj na koniec skanu";
    mapNewMap();
  } else if (!strcmp(c, "auto")) {
    if (scanning || scanRequested || robotBusy) return "robot zajety - najpierw Stop";
    if (mapAllocFailed) return "brak pamieci na mape";
    for (int i = 0; i < 4; i++) MOTOR_state[i] = 0;
    stop_driving();
    exploreHomeMode = false; wallFollowMode = false; refineMode = false; startAutoDrive();
  } else if (!strcmp(c, "home")) {
    if (scanning || robotBusy) return "robot zajety - najpierw Stop";
    if (mapAllocFailed) return "brak pamieci na mape";
    exploreHomeMode = true; wallFollowMode = false; refineMode = false; startAutoDrive();
  } else if (!strcmp(c, "refine")) {
    if (scanning || robotBusy) return "robot zajety - najpierw Stop";
    if (mapAllocFailed) return "brak pamieci na mape";
    uint32_t o, f; gridMap.stats(o, f);
    if (o < 40 || f < 100) return "najpierw zmapuj pokoj";
    exploreHomeMode = false; wallFollowMode = false; refineMode = true; startAutoDrive();
  } else if (!strcmp(c, "wall")) {
    if (scanning || robotBusy) return "robot zajety - najpierw Stop";
    exploreHomeMode = false; refineMode = false; wallFollowMode = true; startAutoDrive();
  } else if (!strcmp(c, "save")) {
    if (!MapStore_save()) return "zapis nie powiodl sie";
  } else return "nieznane polecenie";
  return NULL;
}

// Polecenia mapy: /api/cmd?c=scan|locate|abort|clear|save|home|refine|wall|stop|auto|pose
void apiCmd() {
  String c = server.arg("c");
  const char* err = NULL;
  if (c == "pose") {
    // Ręcznie wskazane miejsce robota (kliknięcie na mapie): szukamy dokładnej pozycji i kierunku w jego pobliżu
    bool robotBusy = AUTO_DRIVE_state || busy_motors;
    if (mapAllocFailed) err = "brak pamieci na mape";
    else if (scanning || scanRequested) err = "skan juz trwa";
    else if (robotBusy) err = "robot jedzie - zatrzymaj go przed skanem";
    else if (!server.hasArg("x") || !server.hasArg("y")) err = "brak wspolrzednych";
    else {
      scanHintX = server.arg("x").toFloat(); scanHintY = server.arg("y").toFloat();
      scanHintValid = true; scanLocateRequested = true; scanRequested = true;
    }
  } else err = robotCommand(c.c_str());

  char buf[96];
  if (err) snprintf(buf, sizeof(buf), "{\"ok\":0,\"err\":\"%s\"}", err);
  else snprintf(buf, sizeof(buf), "{\"ok\":1}");
  sendJson(buf);
}

String getContentType(String filename) {
    if (server.hasArg("download")) return "application/octet-stream";
    else if (filename.endsWith(".htm")) return "text/html";
    else if (filename.endsWith(".html")) return "text/html";
    else if (filename.endsWith(".css")) return "text/css";
    else if (filename.endsWith(".js")) return "application/javascript";
    else if (filename.endsWith(".png")) return "image/png";
    else if (filename.endsWith(".gif")) return "image/gif";
    else if (filename.endsWith(".jpg")) return "image/jpeg";
    else if (filename.endsWith(".ico")) return "image/x-icon";
    else if (filename.endsWith(".xml")) return "text/xml";
    else if (filename.endsWith(".pdf")) return "application/x-pdf";
    else if (filename.endsWith(".zip")) return "application/x-zip";
    else if (filename.endsWith(".gz")) return "application/x-gzip";
    else if (filename.endsWith(".svg")) return "image/svg+xml";
    else if (filename.endsWith(".woff2")) return "font/woff2";
    else if (filename.endsWith(".woff")) return "font/woff";
    return "text/plain";
}

bool handleFileRead(String path) {
    // Serial.println("handleFileRead: " + path);
    if (path.endsWith("/")) path += "index.htm"; // If the path is a directory, add "index.htm"

    String contentType = getContentType(path); // Get the content type
    if (SPIFFS.exists(path)) { // Check if the file exists
        File file = SPIFFS.open(path, "r");
        if (!file) return false;
        server.sendHeader("Connection", "close");       // bez keep-alive: jednowatkowy serwer nie czeka na kolejne zapytanie tego samego klienta
        if (path.endsWith(".htm") || path.endsWith(".html")) {
            server.sendHeader("Cache-Control", "no-store, must-revalidate");
        } else {
            server.sendHeader("Cache-Control", "public, max-age=600");     // css/js/ikona z pamieci przegladarki (szybsze ladowanie)
        }
        server.streamFile(file, contentType); // Send file to client
        file.close();
        return true; // File found and handled
    }
    return false; // File not found
}

void startMdns() {
  if (MDNS.begin(hostname)) {
    MDNS.addService("http", "tcp", 80); // Advertise a service (HTTP)
    mdnsStarted = true;
  } else {
    Serial.println("❌ Error setting up mDNS");
  }
}

// Wołane z loop(): deadman dla sterowania z WWW, samonaprawa auto-jazdy, ponowna próba mDNS.
void Web_maintain() {
  unsigned long now = millis();

  // Deadman: brak keepalive od trzymanego przycisku => stop
  if (!AUTO_DRIVE_state &&
      (MOTOR_state[0] || MOTOR_state[1] || MOTOR_state[2] || MOTOR_state[3]) &&
      (now - lastMotorCmdMs) > MOTOR_DEADMAN_MS) {
    for (int i = 0; i < 4; i++) MOTOR_state[i] = 0;
    stop_driving();
  }

  // Auto-jazda włączona, a zadania nie ma (np. wyścig przy zatrzymywaniu) - uruchom ponownie,
  // chyba że eksploracja właśnie skończyła (explore_Task zdejmuje flagę sam)
  if (AUTO_DRIVE_state && AutomaticDriveTaskHandle == NULL && !autoDriveStopRequested) {
    startAutoDrive();
  }

  // mDNS wymaga działającej sieci - ponawiaj, dopóki się nie uda
  static unsigned long lastMdnsTry = 0;
  if (!mdnsStarted && WiFi.status() == WL_CONNECTED && (now - lastMdnsTry) > 5000) {
    lastMdnsTry = now;
    startMdns();
  }
}

void Web_init() {

  // mDNS initialization
  if (WiFi.status() == WL_CONNECTED) startMdns();   // inaczej ponowi Web_maintain()

  // Web server initialization
    server.on("/ajax_inputs", ajaxInputs);
    server.on("/api/state", apiState);
    server.on("/api/map", apiMap);
    server.on("/api/scan", apiScan);
    server.on("/api/cmd", apiCmd);
    server.on("/api/log", apiLog);
    server.on("/api/maps", apiMaps);
    server.on("/api/maps/view", apiMapView);
    server.on("/api/maps/act", apiMapAct);
    server.onNotFound([]() {
        if (!handleFileRead(server.uri())) {
            String m = "Nie znaleziono: " + server.uri() + "\n\n";
            if (!SPIFFS.exists("/index.htm")) {
                m += "Brak plikow strony w pamieci robota (SPIFFS).\nWgraj folder data/ : Narzedzia > ESP32 Sketch Data Upload (SPIFFS, nie LittleFS),\n"
                     "po wybraniu tego samego schematu partycji co przy wgrywaniu programu, przy zamknietym monitorze portu.\n"
                     "Zajete: ";
                char nb[40]; snprintf(nb, sizeof(nb), "%u z %u B", (unsigned)SPIFFS.usedBytes(), (unsigned)SPIFFS.totalBytes()); m += nb;
            }
            server.send(404, "text/plain", m);
        }
    });

    server.begin();

    // SPIFFS jest montowany w Map_init() (razem z wczytaniem zapisanej mapy)
}
