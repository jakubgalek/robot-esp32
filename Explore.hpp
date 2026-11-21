#pragma once
// ---------------------------------------------------------------------------
// Jazda automatyczna = eksploracja mapy (logika w Planner.hpp, testowana w symulacji).
// Zadanie: skan -> plan do najbliższej niezbadanej granicy -> obrót -> odcinek do 40 cm
// -> (skan co gScanEveryCm i po dotarciu do celu) ... aż nie ma już granic.
// ---------------------------------------------------------------------------
#include "Scan.hpp"
#include "Planner.hpp"
#include "WallFollow.hpp"
#include "Coverage.hpp"

TaskHandle_t AutomaticDriveTaskHandle = NULL;       // (deklarowane tu, bo używa go cały kod)

volatile int exploreState = -1;                      // -1 nie uruchomiono, EX_RUNNING..EX_LIMIT jak w Planner.hpp
volatile float exploreTx = 0, exploreTy = 0;
volatile int exploreWpN = 0;
volatile int exploreFrontiers = 0;
static float exploreWx[PLAN_MAX_WP], exploreWy[PLAN_MAX_WP];

volatile int wallFollowKind = 0;                     // ostatni stan sterownika (do wyświetlenia)
volatile int wallFollowDist = -1;
volatile bool exploreHomeMode = false;               // true = zadanie wraca na start zamiast mapować
volatile bool autoDriveStopRequested = false;        // ustawiane przez WWW/BLE
boolean AUTO_DRIVE_state = 0;                        // 1 = eksploracja włączona (WWW)

// "Robot" dla Explorera: ruchy przez prawdziwe silniki, odometria i skan
struct RealRobot {
  Pose2 pose() { return odometry_get(); }
  const OccGrid& map() { return gridMap; }
  bool aborted() { return autoDriveStopRequested || maneuverAbort; }

  bool locate() { return Scan_run(true); }
  bool scan() {
    for (int tries = 0; tries < 3 && !aborted(); tries++) {
      if (Scan_run()) {
        Pose2 p = odometry_get();
        MapLock lock(500);
        if (lock.ok()) { gridMap.clearFootprint(p.x, p.y, p.h, ROBOT_LENGTH_CM, ROBOT_WIDTH_CM); gridMap.version++; mapDirty = true; }
        return true;
      }
      vTaskDelay(pdMS_TO_TICKS(300));
    }
    return false;
  }
  bool turn(float deg) {
    if (deg > 0) return ::turn(turn_left, deg, motorSpeed);
    return ::turn(turn_right, -deg, motorSpeed);
  }
  bool drive(float cm) {
    bool ok = ::drive(forward, cm, motorSpeed);
    if (!ok && !aborted()) {
      logEvent("stop: %s", driveStopInfo[0] ? driveStopInfo : "przerwano / limit czasu jazdy");
      fixedSensorsMapOnce(true);   // zapisz na mapie przeszkodę, która nas zatrzymała
    }
    return ok;
  }
  void note(const char* text) { logEvent("%s", text); }
};


// "Robot" dla jazdy przy lewej ścianie: czujniki boczne 2 i 7, przód 3/4/5; bez skanów i bez zapisu na mapę
struct WallRobot {
  // Odczyt boczny uśredniamy z kilku świeżych pomiarów (robot stoi), bo pojedynczy odczyt VL53L0X potrafi
  // skakać o 1-2 cm, a różnica dwóch czujników dawałaby fałszywe "krzywe ustawienie".
  int side(int idx) {
    if (sensorAgeMs(idx) > SENSOR_MAX_AGE_MS) return -2;
    int sum = 0, n = 0;
    unsigned long lastSeen = 0;
    for (int tries = 0; tries < 12 && n < 3; tries++) {
      unsigned long t = sensorLastOkMs[idx];
      if (t != lastSeen) {
        lastSeen = t;
        int v = (idx == 2) ? measurement2 : measurement7;
        if (v < 1 || v > WALLF_SEEN_CM) return -1;     // brak ściany w zasięgu
        sum += v; n++;
      }
      vTaskDelay(pdMS_TO_TICKS(15));
    }
    return n ? (sum + n / 2) / n : -2;
  }
  int front() {
    if (!frontSensorsFresh()) return -2;
    // Do decyzji o rogu liczy się przód-środek i przód-prawy. Przód-lewy patrzy skosem na lewą ścianę (ta, której się trzymamy),
    // więc uwzględniamy go dopiero, gdy coś jest naprawdę blisko - inaczej każda korekta w lewo wyglądałaby jak róg.
    int m = measurement4;
    if (measurement5 < m) m = measurement5;
    if (measurement3 <= 15 && measurement3 < m) m = measurement3;
    return m > 250 ? 250 : m;
  }
  bool aborted() { return autoDriveStopRequested || maneuverAbort; }
  bool turn(float deg) {
    if (deg > 0) return ::turn(turn_left, deg, motorSpeed);
    return ::turn(turn_right, -deg, motorSpeed);
  }
  bool drive(float cm) { return ::drive(forward, cm, motorSpeed); }
  void note(const char* text) { logEvent("%s", text); }
};

void wallFollow_Task(void* pvParameters) {
  static WallFollower wf;
  WallRobot robot;
  wf.reset();
  exploreState = EX_RUNNING;
  logEvent("jazda przy lewej scianie: start");
  int st = WF_RUNNING;
  while (!autoDriveStopRequested && (st = wf.step(robot)) == WF_RUNNING) {
    wallFollowKind = wf.lastKind; wallFollowDist = (int)wf.lastDist;
    vTaskDelay(pdMS_TO_TICKS(wf.stale ? 50 : 10));
  }
  stop_driving();
  exploreState = (st == WF_STUCK) ? EX_STUCK : EX_ABORT;
  logEvent("jazda przy lewej scianie: koniec (%s), krokow %d", st == WF_STUCK ? "zablokowany" : "zatrzymano", wf.steps);
  wallFollowMode = false;
  AUTO_DRIVE_state = 0;
  AutomaticDriveTaskHandle = NULL;
  vTaskDelete(NULL);
}


// Zadanie "drugi przebieg": dodatkowe skany z siatki punktów po całym zmapowanym pokoju
void refine_Task(void* pvParameters) {
  static RefinePass refine;
  RealRobot robot;
  exploreState = EX_RUNNING;
  refine.begin(gridMap, odometry_get());
  logEvent("drugi przebieg: %d punktow skanu", refine.n);
  int st = EX_RUNNING;
  while (!autoDriveStopRequested && (st = refine.step(robot)) == EX_RUNNING) {
    int i = refine.idx < refine.n ? refine.idx : refine.n - 1;
    if (i >= 0) { exploreTx = refine.px[i]; exploreTy = refine.py[i]; }
    exploreFrontiers = 0;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  if (autoDriveStopRequested && st == EX_RUNNING) st = EX_ABORT;
  stop_driving();
  exploreWpN = 0;
  refine.ex.planner.release();
  exploreState = st;
  if (st != EX_ABORT) MapStore_save();
  logEvent("drugi przebieg: koniec, status %d", st);
  refineMode = false;
  AUTO_DRIVE_state = 0;
  AutomaticDriveTaskHandle = NULL;
  vTaskDelete(NULL);
}

void explore_Task(void* pvParameters) {
  static Explorer ex;                 // static: nie obciąża stosu zadania
  RealRobot robot;
  if (exploreHomeMode) ex.beginHome(0.0f, 0.0f, gridMap.startHeading); else ex.begin();
  exploreState = EX_RUNNING;
  logEvent(exploreHomeMode ? "powrot na start: start, wolny RAM" : "automat: start, wolny RAM %u KB, najwiekszy blok %u KB", (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMaxAllocHeap() / 1024));
  int st = EX_RUNNING;
  while (!autoDriveStopRequested && (st = ex.step(robot)) == EX_RUNNING) {
    exploreTx = ex.tx; exploreTy = ex.ty;
    exploreFrontiers = ex.last.frontiers;
    int n = ex.last.n;
    for (int i = 0; i < n; i++) { exploreWx[i] = ex.last.wx[i]; exploreWy[i] = ex.last.wy[i]; }
    exploreWpN = n;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  if (autoDriveStopRequested && st == EX_RUNNING) st = EX_ABORT;
#if EXPLORE_SECOND_PASS
  if (st == EX_DONE && !exploreHomeMode) {
    static RefinePass refine;
    refine.begin(gridMap, odometry_get());
    logEvent("drugi przebieg: %d punktow skanu", refine.n);
    int rs;
    while (!autoDriveStopRequested && (rs = refine.step(robot)) == EX_RUNNING) vTaskDelay(pdMS_TO_TICKS(20));
    refine.ex.planner.release();
  }
#endif
  stop_driving();
  exploreWpN = 0;
  ex.planner.release();
  exploreState = st;
  if (st != EX_ABORT && !exploreHomeMode) MapStore_save();
  logEvent("automat: koniec, status %d, krokow %d, RAM %u KB, blok %u KB", st, ex.steps, (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMaxAllocHeap() / 1024));
  AUTO_DRIVE_state = 0;
  AutomaticDriveTaskHandle = NULL;
  vTaskDelete(NULL);
}

int Explore_copyWaypoints(float* wx, float* wy, int maxN) {
  int n = exploreWpN;
  if (n > maxN) n = maxN;
  for (int i = 0; i < n; i++) { wx[i] = exploreWx[i]; wy[i] = exploreWy[i]; }
  return n;
}
