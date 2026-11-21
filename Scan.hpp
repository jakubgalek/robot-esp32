#pragma once
// ---------------------------------------------------------------------------
// Skanowanie 360 stopni NA POSTOJU.
// Serwo obraca oba czujniki (0 = przód, 1 = tył) od 0 do 180 stopni co SCAN_STEP_DEG,
// razem tworzą pełny okrąg. Skan jest zbierany w całości, potem (opcjonalnie) dopasowywany
// do istniejącej mapy w celu korekty dryfu odometrii, i dopiero wtedy wpisywany do mapy.
// Jeśli robot ruszy w trakcie skanu, skan jest odrzucany w całości (mapa nie jest psuta).
// ---------------------------------------------------------------------------
#include <ESP32Servo.h>
#include "Adafruit_VL53L0X.h"
#include "Config.hpp"
#include "I2C.hpp"
#include "MapGrid.hpp"
#include "Odometry.hpp"
#include "MapStore.hpp"

extern Adafruit_VL53L0X lox0, lox1;   // Distance_sensors.hpp

static const int SERVO_PIN = 25;
int pwmFrequencyServo = 50;
Servo servoMotor;

#define SCAN_MAX_BEAMS (2 * (180 / SCAN_STEP_DEG + 1))

static Beam scanBeams[SCAN_MAX_BEAMS];
volatile bool scanning = false;
volatile bool scanRequested = false;
volatile bool refineMode = false;   // drugi przebieg skanów (Explore.hpp)
volatile int lastAutoMode = 0;            // 0 mapowanie, 1 powrót na start, 2 przy ścianie, 4 drugi przebieg
volatile bool wallFollowMode = false;     // true = jazda wzdłuż lewej ściany (bez mapowania, Explore.hpp)
volatile bool scanLocateRequested = false;
volatile bool scanHintValid = false;      // użytkownik wskazał na mapie, gdzie stoi robot
volatile float scanHintX = 0, scanHintY = 0;
volatile bool scanAbort = false;
volatile int scanProgress = 0;           // 0..100
volatile uint32_t scanCount = 0;         // ile skanów wpisano do mapy

// Podgląd ostatniego skanu dla UI (punkty trafień w cm)
volatile uint32_t scanOverlayVersion = 0;
static int16_t scanOverlay[SCAN_MAX_BEAMS][2];
static volatile int scanOverlayCount = 0;
static Pose2 scanOverlayPose = {0, 0, 0};

// Wynik ostatniego dopasowania do mapy (dla UI / diagnostyki)
volatile float lastMatchDx = 0, lastMatchDy = 0, lastMatchDh = 0;
volatile float lastWallDelta = 0, lastWallConf = 0;   // ostatnia korekta kierunku względem ścian
volatile int lastMatchResult = 0;   // 0 brak, 1 zastosowano korektę, 2 odrzucono, 3 pominięto (pusta mapa / za mało punktów)

// ---------------------------------------------------------------------------
// Czujniki stałe jako źródło przeszkód (tylko trafienia, bez zerowania wolnej przestrzeni)
// ---------------------------------------------------------------------------
struct FixedSensorGeom { uint8_t idx; float fwd, left, angDeg; bool cont; };   // cont: stale dopisuj do mapy podczas jazdy
static const FixedSensorGeom FIXED_SENSORS[] = {
  {4,  WHEEL_AXLE_FROM_FRONT_CM - 1.0f,  0.0f,    0.0f, true },   // przód - środek (1 cm od przodu): na bieżąco podczas jazdy
  {3,  WHEEL_AXLE_FROM_FRONT_CM - 1.5f,  4.0f,   45.0f, false},   // przód - lewy, skos 45 st.: tylko po zatrzymaniu przez przeszkodę (położenie ZGADYWANE)
  {5,  WHEEL_AXLE_FROM_FRONT_CM - 1.5f, -4.0f,  -45.0f, false},   // przód - prawy, skos 45 st.: jw.
};

// Wołane z zadania czujników (rate-limit w środku) i po zatrzymaniu przez przeszkodę (force).
// Nie blokuje: gdy mapa jest zajęta (skan), po prostu pomija.
extern boolean AUTO_DRIVE_state;                 // Explore.hpp
void fixedSensorsMapOnce(bool force) {
#if FIXED_SENSOR_MAPPING
  if (!AUTO_DRIVE_state) return;   // czujniki stale pisza na mape tylko podczas mapowania pokoju (automat); poza tym mape zmienia tylko skan serwem
  if (wallFollowMode) return;   // jazda przy ścianie: bez zapisu na mapę
  static unsigned long lastRun = 0, lastBump = 0;
  unsigned long now = millis();
  if (!force && (now - lastRun) < 200) return;
  lastRun = now;
  if (!gridMap.cells || scanning || !gridMap.scanned) return;   // przed pierwszym skanem pojedyncze odczyty z przodu nie trafiają na mapę
  MotionState m = currentMotion;
  if (!force && (m == MOTION_LEFT || m == MOTION_RIGHT)) return;   // kąt zmienia się szybciej niż pomiar
  Pose2 p = odometry_get();
  float c = cosf(p.h * MG_DEG2RAD), s = sinf(p.h * MG_DEG2RAD);
  MapLock lock(3);
  if (!lock.ok()) return;
  bool any = false;
  for (size_t i = 0; i < sizeof(FIXED_SENSORS) / sizeof(FIXED_SENSORS[0]); i++) {
    const FixedSensorGeom& g = FIXED_SENSORS[i];
    if (!force && !g.cont) continue;
    if (sensorAgeMs(g.idx) > 150) continue;
    int cm = (g.idx == 2) ? measurement2 : (g.idx == 3) ? measurement3 : (g.idx == 4) ? measurement4 :
             (g.idx == 5) ? measurement5 : (g.idx == 6) ? measurement6 : measurement7;
    if (cm < FIXED_MAP_MIN_CM || cm > FIXED_MAP_MAX_CM) continue;
    float sx = p.x + c * g.fwd - s * g.left, sy = p.y + s * g.fwd + c * g.left;
    // po zatrzymaniu przez przeszkodę wpisujemy ją mocno (3 trafienia), żeby planer ją uznał
    for (int k = 0; k < (force ? 3 : 1); k++) gridMap.integrateBeam(sx, sy, p.h + g.angDeg, (float)cm, true);
    any = true;
  }
  if (any) {
    mapDirty = true;
    if (force || (now - lastBump) > 500) { gridMap.version++; lastBump = now; }
  }
#endif
}
void fixedSensorsMap() { fixedSensorsMapOnce(false); }

// Zamienia pomiar czujnika serwa na promień. Zwraca false, gdy pomiaru nie da się użyć.
static bool classifyBeam(const VL53L0X_RangingMeasurementData_t& m, float relDeg, uint8_t sensor, Beam& b) {
  uint8_t st = m.RangeStatus;
  float cm = m.RangeMilliMeter / 10.0f;
  b.relDeg = relDeg;
  b.sensor = sensor;
  if (st <= 2 && cm >= MAP_MIN_RANGE_CM && cm <= MAP_MAX_RANGE_CM) {
    b.kind = 1; b.rangeCm = cm + SERVO_SENSOR_RADIUS_CM;   // czujnik siedzi 1,9 cm od osi bębna
    return true;
  }
  if (st == 4 || (st <= 2 && cm > MAP_MAX_RANGE_CM)) {   // brak echa albo za daleko
    b.kind = 0; b.rangeCm = MAP_NOECHO_FREE_CM;
    return true;
  }
  return false;   // 3 = za blisko, 5 = błąd sprzętowy: bez danych
}

static int scanStat[8];   // diagnostyka skanu: 0..5 = RangeStatus czujnika, 6 = brak dostępu do I2C, 7 = błąd I2C

static bool readServoSensor(Adafruit_VL53L0X& lox, float relDeg, uint8_t sensor, Beam& b) {
  VL53L0X_RangingMeasurementData_t m;
  {
    I2CLock lock(100);
    if (!lock.ok()) { scanStat[6]++; return false; }
    lox.rangingTest(&m, false);
    if (lox.Status != VL53L0X_ERROR_NONE) { scanStat[7]++; return false; }
  }
  if (m.RangeStatus < 6) scanStat[m.RangeStatus]++;
  return classifyBeam(m, relDeg, sensor, b);
}

// Czeka, aż robot się uspokoi (np. po zatrzymaniu przez auto-jazdę)
static bool waitStationary(unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while (!odometry_stationary()) {
    if (millis() - t0 > timeoutMs) return false;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  return true;
}

// Wykonuje skan. locate=true: szersze okno dopasowania (po restarcie / po przeniesieniu robota).
// Zwraca true, gdy skan trafił do mapy.
static void scanYieldHook() { vTaskDelay(1); }

bool Scan_run(bool locate = false) {
  if (scanning || !gridMap.cells) return false;
  if (!waitStationary(1500)) return false;

  scanning = true;
  scanAbort = false;
  scanProgress = 0;
  logEvent("skan: start (%.0f, %.0f, kat %.0f)", odometry_get().x, odometry_get().y, odometry_get().h);

  Pose2 pose = odometry_get();
  int nb = 0;
  bool aborted = false;
  const int steps = 180 / SCAN_STEP_DEG + 1;

  memset(scanStat, 0, sizeof(scanStat));
  servoMotor.write(0);
  vTaskDelay(pdMS_TO_TICKS(350));   // serwo wraca na kąt 0

  for (int i = 0; i < steps && !aborted; i++) {
    int a = i * SCAN_STEP_DEG;
    servoMotor.write(a);
    vTaskDelay(pdMS_TO_TICKS(SCAN_SETTLE_MS));

    if (scanAbort || !odometry_stationary()) { aborted = true; break; }

    float rel0 = SERVO_DIRECTION * (float)a + SERVO_ZERO_OFFSET_DEG;
    Beam b;
    if (readServoSensor(lox0, rel0, 0, b)) scanBeams[nb++] = b;
    if (readServoSensor(lox1, rel0 + 180.0f, 1, b)) scanBeams[nb++] = b;
    scanProgress = (i + 1) * 100 / steps;
  }

  servoMotor.write(0);

  // Pozycja w trakcie skanu nie mogła się zmienić (inaczej odrzucamy)
  Pose2 now = odometry_get();
  if (fabsf(now.x - pose.x) > 1.0f || fabsf(now.y - pose.y) > 1.0f) aborted = true;

  bool stored = false;
  if (!aborted && nb >= 20) {
    int hits = 0;
    for (int i = 0; i < nb; i++) if (scanBeams[i].kind == 1) hits++;

    MapLock lock;
    bool globalApplied = false;
    Pose2 startPose = pose;
    if (locate) {
      // "Odnajdź się": jeśli skan pasuje do mapy w obecnie przyjętym miejscu, robot się nie ruszył - wystarczy
      // lokalna korekta. W przeciwnym razie szukamy pozycji w całej mapie (robot mógł zostać przeniesiony).
      bool hint = scanHintValid;
      float hx = scanHintX, hy = scanHintY;
      scanHintValid = false;
      float here = hint ? 0.0f : gridMap.scoreAt(pose, scanBeams, nb);
      if (here < 0.7f * hits) {
        if (hint) logEvent("lokalizacja: szukam kierunku i dokladnej pozycji wokol wskazanego miejsca (%.0f, %.0f)...", (double)hx, (double)hy);
        else logEvent("lokalizacja: skan nie pasuje do przyjetej pozycji - szukam w calej mapie...");
        gridMap.yieldHook = scanYieldHook;
        OccGrid::GlobalInfo gi;
        Pose2 hp = {hx, hy, pose.h};
        bool ok = hint ? gridMap.globalLocalize(hp, scanBeams, nb, gi, 30.0f) : gridMap.globalLocalize(pose, scanBeams, nb, gi);
        gridMap.yieldHook = NULL;
        if (ok) {
          logEvent("lokalizacja: ZNALEZIONO (%.0f, %.0f, kat %.0f) - bylo (%.0f, %.0f, kat %.0f); ocena %.0f/%d, druga %.0f, sprawdzono %d pozycji",
                   (double)gi.pose.x, (double)gi.pose.y, (double)gi.pose.h, (double)pose.x, (double)pose.y, (double)pose.h,
                   (double)gi.score, gi.hits, (double)gi.second, gi.cands);
          pose = gi.pose; globalApplied = true;
        } else {
          logEvent("lokalizacja: NIE ZNALEZIONO - %s (najlepsza ocena %.0f/%d, druga %.0f)", gi.why, (double)gi.score, gi.hits, (double)gi.second);
        }
      }
    }
    OccGrid::LocalizeInfo li;
    Pose2 finalPose = gridMap.localize(pose, scanBeams, nb, locate, li);
    if (globalApplied) { li.wallMeasured = false; odoTurnSinceScan = 0.0f; }
    (void)startPose;
    lastMatchResult = li.result;
    lastMatchDx = li.dx; lastMatchDy = li.dy; lastMatchDh = li.dh;
    lastWallDelta = li.wallApplied ? li.wallDelta : 0.0f; lastWallConf = li.wallConf;
    if (li.wallRefSet) logEvent("skan: wyrownano uklad mapy do scian (obrot %.1f st., pewnosc %.2f)", li.wallDelta, li.wallConf);
    if (li.wallApplied && !li.wallRefSet) logEvent("skan: korekta kierunku wzgledem scian %.1f st. (pewnosc %.2f)", li.wallDelta, li.wallConf);
    if (li.wallMeasured) {   // ściany pokazały, ile kąta nazbieraliśmy od ostatniego skanu -> uczymy skalę żyroskopu
      float ns = mgScaleUpdate(gyroScale, li.wallDelta, odoTurnSinceScan);
      if (fabsf(ns - gyroScale) > 0.0005f) {
        logEvent("zyroskop: skala %.3f -> %.3f (blad kata %.1f st. po obrotach %.0f st.)", (double)gyroScale, (double)ns, (double)li.wallDelta, (double)odoTurnSinceScan);
        gyroScale = ns;
        odoGyroScaleChanged = true;
      }
    }
    odoTurnSinceScan = 0.0f;
    if (li.wallApplied || li.result == 1 || globalApplied) odometry_set(finalPose);   // robot stoi, więc nadpisanie pozycji jest bezpieczne
    gridMap.integrateScan(finalPose, scanBeams, nb);
    gridMap.scanned = true;
    mapDirty = true;
    scanCount = scanCount + 1;

    // Podgląd: punkty trafień w układzie mapy
    float sx, sy;
    OccGrid::sensorOrigin(finalPose, SERVO_SENSOR_OFFSET_FWD_CM, SERVO_SENSOR_OFFSET_LEFT_CM, sx, sy);
    int cnt = 0;
    for (int i = 0; i < nb; i++) {
      if (scanBeams[i].kind != 1) continue;
      float ang = (finalPose.h + scanBeams[i].relDeg) * MG_DEG2RAD;
      scanOverlay[cnt][0] = (int16_t)lroundf(sx + cosf(ang) * scanBeams[i].rangeCm);
      scanOverlay[cnt][1] = (int16_t)lroundf(sy + sinf(ang) * scanBeams[i].rangeCm);
      cnt++;
    }
    scanOverlayCount = cnt;
    scanOverlayPose = finalPose;
    scanOverlayVersion = scanOverlayVersion + 1;
    stored = true;
  }

  scanProgress = stored ? 100 : 0;
  scanning = false;
  if (stored) {
    int h = 0;
    for (int i = 0; i < nb; i++) if (scanBeams[i].kind == 1) h++;
    logEvent("skan: ok, %d pomiarow (%d trafien), dopasowanie %d (%.1f, %.1f cm, %.1f st.)", nb, h,
             (int)lastMatchResult, (double)lastMatchDx, (double)lastMatchDy, (double)lastMatchDh);
    logEvent("skan: statusy czujnika 0:%d 1:%d 2:%d 3:%d 4:%d 5:%d, brak I2C:%d, blad I2C:%d", scanStat[0], scanStat[1], scanStat[2], scanStat[3], scanStat[4], scanStat[5], scanStat[6], scanStat[7]);
  } else {
    logEvent("skan: odrzucony (%s, %d pomiarow; statusy 0:%d 1:%d 2:%d 3:%d 4:%d 5:%d, brak I2C:%d, blad I2C:%d)", aborted ? "robot sie ruszyl albo przerwano" : "za malo pomiarow", nb, scanStat[0], scanStat[1], scanStat[2], scanStat[3], scanStat[4], scanStat[5], scanStat[6], scanStat[7]);
  }
  return stored;
}

// Kopia podglądu skanu dla serwera WWW (pod blokadą mapy)
int Scan_copyOverlay(int16_t (*out)[2], int maxN, Pose2& pose) {
  MapLock lock;
  int n = scanOverlayCount;
  if (n > maxN) n = maxN;
  for (int i = 0; i < n; i++) { out[i][0] = scanOverlay[i][0]; out[i][1] = scanOverlay[i][1]; }
  pose = scanOverlayPose;
  return n;
}

void Scan_servoInit() {
  ESP32PWM::allocateTimer(0);
  servoMotor.setPeriodHertz(pwmFrequencyServo);
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(0);
}

// Zadanie realizujące skany na żądanie (przycisk "Skanuj" / "Lokalizuj")
void scan_Task(void* pvParameters) {
  for (;;) {
    if (scanRequested) {
      bool loc = scanLocateRequested;
      scanRequested = false;
      scanLocateRequested = false;
      Scan_run(loc);
    }
    MapStore_saveIfDue();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
