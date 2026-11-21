#pragma once
// ---------------------------------------------------------------------------
// Odometria: pozycja robota w cm i kąt w stopniach.
//  - droga: impulsy enkodera x cm/impuls, kierunek (przód/tył) z aktualnego stanu silników
//  - kąt podczas obrotu: całkowanie prędkości kątowej z żyroskopu, znak z komendy silników
//    (nie trzeba znać orientacji osi Z czujnika)
//  - kąt podczas jazdy: robot zbacza, więc żyroskop liczymy też wtedy. Znak osi Z robot uczy
//    się sam z obrotów (gyroSign); do czasu nauczenia kąt podczas jazdy się nie zmienia.
//  - na postoju mierzymy resztkowy bias żyroskopu i odejmujemy go (mniej dryfu)
//  - po zatrzymaniu liczymy jeszcze ODO_SETTLE_MS "dobiegu", żeby nie gubić ruchu z rozpędu
// Wołane z zadania czujników (jedyne miejsce, które czyta żyroskop).
// ---------------------------------------------------------------------------
#include <Preferences.h>
#include "Log.hpp"
#include "Config.hpp"
#include "Motor_functions.hpp"
#include "MapGrid.hpp"

bool frontBlocked();   // Distance_sensors.hpp
bool rearBlocked();
bool frontSensorsFresh();
bool backSensorFresh();
extern volatile int measurement2, measurement3, measurement4, measurement5, measurement6, measurement7;

// Prośba o przerwanie manewrów (jazda/obrót/skan) - ustawiana przez WWW, BLE, auto-jazdę
volatile bool maneuverAbort = false;

static portMUX_TYPE poseMux = portMUX_INITIALIZER_UNLOCKED;
static Pose2 odoPose = {0.0f, 0.0f, ROBOT_START_HEADING};
volatile double odoPathCm = 0.0;                 // łączna przejechana droga (zawsze rośnie)
volatile bool odoGyroReady = false;              // ustawiane po inicjalizacji MPU
const float encoderCmPerPulse = ENCODER_CM_PER_PULSE_DEFAULT;   // droga na impuls enkodera (Config.hpp)
volatile float lastGyroDps = 0.0f;
volatile unsigned long cycleMaxMs = 0, cycleAvgMs = 0;   // czas jednego obiegu zadania czujników (diagnostyka)
volatile float gyroBiasDps = 0.0f;               // resztkowe zero żyroskopu, śledzone na postoju
volatile float gyroScale = GYRO_SCALE;           // skala żyroskopu; uczy się ze skanów (korekta względem ścian)
volatile float odoTurnSinceScan = 0.0f;          // suma obrotów (lewo +, prawo -) od ostatniego skanu, według żyroskopu
volatile bool odoGyroScaleChanged = false;
volatile int gyroSign = 0;                       // +1/-1: znak osi Z względem naszego kąta (uczy się z obrotów), 0 = nieznany
static int gyroSignVotes = 0;
static volatile unsigned long odoLastMoveMs = 0;
volatile bool odoGyroSignChanged = false;

const unsigned long DRIVE_TIMEOUT_MS = 12000;
const unsigned long TURN_TIMEOUT_MS  = 8000;
const unsigned long DRIVE_BLOCK_DEBOUNCE_MS = 120;
char driveStopInfo[96] = "";   // dlaczego ostatnia jazda została zatrzymana (do Dziennika)
float turnLeadDeg[2] = {0.0f, 0.0f};   // nauczony dobieg obrotu [stopnie]: [0] w prawo, [1] w lewo - silniki jeszcze kręcą po odcięciu zasilania

Pose2 odometry_get() {
  Pose2 p;
  portENTER_CRITICAL(&poseMux);
  p = odoPose;
  portEXIT_CRITICAL(&poseMux);
  return p;
}

void odometry_set(const Pose2& p) {
  portENTER_CRITICAL(&poseMux);
  odoPose = p;
  odoPose.h = mgWrap360(odoPose.h);
  portEXIT_CRITICAL(&poseMux);
}

// Czy robot na pewno stoi (silniki wyłączone i dobieg się skończył) - warunek skanu
bool odometry_stationary() {
  MotionState m = currentMotion;
  if (m != MOTION_STOP && m != MOTION_UNKNOWN) return false;
  return (millis() - odoLastMoveMs) > ODO_SETTLE_MS;
}

void odometry_update() {
  static unsigned long lastUs = 0;
  static long lastIndents = 0;
  static MotionState lastMove = MOTION_STOP;

  unsigned long nowUs = micros();
  float dt = (lastUs == 0) ? 0.0f : (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt > 1.0f) dt = 1.0f;   // prawdziwy upływ czasu (wcześniej 0.1 s zaniżało kąt, gdy pętla zwolniła)

  unsigned long now = millis();
  MotionState m = currentMotion;
  MotionState eff = m;
  if (m == MOTION_STOP || m == MOTION_UNKNOWN) {
    eff = ((now - odoLastMoveMs) < ODO_SETTLE_MS) ? lastMove : MOTION_STOP;
  } else {
    odoLastMoveMs = now;
    lastMove = m;
  }

  long n = indents;
  long d = n - lastIndents;
  lastIndents = n;

  if (d > 0 && (eff == MOTION_FORWARD || eff == MOTION_BACKWARD)) {
    float cm = d * encoderCmPerPulse;
    float sign = (eff == MOTION_FORWARD) ? 1.0f : -1.0f;
    portENTER_CRITICAL(&poseMux);
    float hr = odoPose.h * MG_DEG2RAD;
    odoPose.x += sign * cm * cosf(hr);
    odoPose.y += sign * cm * sinf(hr);
    portEXIT_CRITICAL(&poseMux);
    odoPathCm = odoPathCm + cm;
  }

  if (odoGyroReady && dt > 0.0f) {
    bool turning = (eff == MOTION_LEFT || eff == MOTION_RIGHT);
    bool driving = (eff == MOTION_FORWARD || eff == MOTION_BACKWARD);
    bool idle = (eff == MOTION_STOP || eff == MOTION_UNKNOWN);
    static unsigned long lastIdleRead = 0;
    static float turnSigned = 0.0f;          // suma gz*dt w trwającym obrocie
    static bool prevTurning = false, prevLeft = false;

    // Koniec obrotu: nauka znaku osi Z (obrót w lewo ma zwiększać kąt)
    if (prevTurning && !turning) {
      if (fabsf(turnSigned) > 20.0f) {
        int observed = (turnSigned > 0 ? 1 : -1) * (prevLeft ? 1 : -1);
        gyroSignVotes += observed;
        if (gyroSignVotes > 4) gyroSignVotes = 4;
        if (gyroSignVotes < -4) gyroSignVotes = -4;
        int ns = gyroSignVotes >= 2 ? 1 : (gyroSignVotes <= -2 ? -1 : 0);
        if (ns != gyroSign) {
          gyroSign = ns;
          odoGyroSignChanged = true;   // zapis do pamięci robi zadanie główne (Preferences nie w tej pętli)
          logEvent("zyroskop: znak osi Z = %d (obrot %.0f st.)", ns, turnSigned);
        }
      }
      turnSigned = 0.0f;
    }
    prevTurning = turning;
    if (turning) prevLeft = (eff == MOTION_LEFT);

    bool want = turning || driving || (idle && (now - lastIdleRead) >= 50);
    if (want) {
      float gz = 0.0f;
      bool ok = false;
      {
        I2CLock lock(10);
        if (lock.ok()) {
          mpu.update();
          gz = mpu.getGyroZ();   // [stopnie/s]
          ok = true;
        }
      }
      if (ok) {
        lastGyroDps = gz;
        if (idle) {
          lastIdleRead = now;
          if (fabsf(gz - gyroBiasDps) < 2.0f) {                 // nie uczymy się z ruchu (np. przesuwanie ręką)
            float b = gyroBiasDps * 0.97f + gz * 0.03f;
            if (b > 3.0f) b = 3.0f;
            if (b < -3.0f) b = -3.0f;
            gyroBiasDps = b;
          }
        } else {
          float g = gz - gyroBiasDps;
          if (turning) {
            turnSigned += g * dt;
            float mag = fabsf(g);
            if (mag >= GYRO_DEADBAND_DPS) {
              float dh = mag * dt * gyroScale;
              odoTurnSinceScan = odoTurnSinceScan + (eff == MOTION_LEFT ? dh : -dh);
              portENTER_CRITICAL(&poseMux);
              odoPose.h = mgWrap360(odoPose.h + (eff == MOTION_LEFT ? dh : -dh));
              portEXIT_CRITICAL(&poseMux);
            }
          } else if (driving && gyroSign != 0 && fabsf(g) >= GYRO_DEADBAND_STRAIGHT_DPS) {
            portENTER_CRITICAL(&poseMux);
            odoPose.h = mgWrap360(odoPose.h + gyroSign * g * dt * gyroScale);
            portEXIT_CRITICAL(&poseMux);
          }
        }
      }
    }
  }
}

// ---- Zapamiętany znak osi Z żyroskopu i licznik przebiegu ----
static double odoTotalBaseCm = 0.0;     // przebieg z poprzednich uruchomień
static double odoTotalSavedCm = 0.0;    // ile z odoPathCm jest już zapisane
static uint32_t odoRunBaseS = 0;        // czas pracy z poprzednich uruchomień [s]
static uint32_t odoRunSavedS = 0;       // czas pracy (uptime), przy którym ostatnio zapisano

void odometry_loadGyroSign() {
  Preferences prefs;
  if (prefs.begin("odo", true)) {
    int v = prefs.getInt("gsign", 0);
    float sc = prefs.getFloat("gscale", 0.0f);
    odoTotalBaseCm = prefs.getFloat("total", 0.0f);
    odoRunBaseS = prefs.getUInt("runs", 0);
    prefs.end();
    if (v == 1 || v == -1) { gyroSign = v; gyroSignVotes = 3 * v; }
    if (sc >= 0.9f && sc <= 1.1f) gyroScale = sc;
  }
}

// Przebieg całkowity robota [cm] (z poprzednich uruchomień + ten)
double odometry_totalCm() { return odoTotalBaseCm + odoPathCm; }
// Łączny czas pracy [s]: z poprzednich uruchomień + ten
uint32_t odometry_runSeconds() { return odoRunBaseS + millis() / 1000UL; }

// Wołane z zadania głównego: zapisuje nowo nauczony znak osi Z oraz (co ok. 5 m jazdy) przebieg
void odometry_persistGyroSign() {
  bool trip = (odoPathCm - odoTotalSavedCm) >= 500.0;
  bool runDue = (millis() / 1000UL) - odoRunSavedS >= 600;      // czas pracy zapisujemy co 10 min (oszczędza flash)
  if (!odoGyroSignChanged && !odoGyroScaleChanged && !trip && !runDue) return;
  odoGyroSignChanged = false;
  odoGyroScaleChanged = false;
  Preferences prefs;
  if (prefs.begin("odo", false)) {
    prefs.putInt("gsign", gyroSign);
    prefs.putFloat("gscale", gyroScale);
    if (trip) { prefs.putFloat("total", (float)odometry_totalCm()); odoTotalSavedCm = odoPathCm; }
    if (runDue) { prefs.putUInt("runs", odometry_runSeconds()); odoRunSavedS = millis() / 1000UL; }
    prefs.end();
  }
}
// ---------------------------------------------------------------------------
// Manewry blokujące (używane przez auto-jazdę i kalibrację). Zwracają true, gdy
// cel został osiągnięty; false przy przeszkodzie, przerwaniu albo przekroczeniu czasu.
// ---------------------------------------------------------------------------
bool drive(DirectionFunction directionFunc, double targetCm, int speed) {
  if (speed < 0 || speed > maxSpeed) return false;
  ledcWrite(pwmChannelSpeed, speed);
  directionFunc();

  double start = odoPathCm;
  unsigned long t0 = millis();
  unsigned long blockedSince = 0;
  bool ok = true;
  driveStopInfo[0] = 0;
  while ((odoPathCm - start) < targetCm) {
    if (maneuverAbort || (millis() - t0) > DRIVE_TIMEOUT_MS) { ok = false; break; }
    bool blocked = (directionFunc == forward && frontBlocked()) || (directionFunc == backward && rearBlocked());
    if (blocked) {
      // pojedynczy zły odczyt (szum, chwilowy brak danych) nie zatrzymuje jazdy - dopiero 120 ms z rzędu
      if (!blockedSince) blockedSince = millis();
      if (millis() - blockedSince >= DRIVE_BLOCK_DEBOUNCE_MS) {
        ok = false;
        snprintf(driveStopInfo, sizeof(driveStopInfo), "%s L=%d C=%d P=%d T=%d, dane %s", directionFunc == forward ? "przod" : "tyl",
                 (int)measurement3, (int)measurement4, (int)measurement5, (int)measurement6,
                 (directionFunc == forward ? frontSensorsFresh() : backSensorFresh()) ? "swieze" : "PRZESTARZALE");
        break;
      }
    } else blockedSince = 0;
    vTaskDelay(2);
  }
  stop_driving();
  vTaskDelay(pdMS_TO_TICKS(ODO_SETTLE_MS));   // pozwól odometrii doliczyć dobieg
  return ok;
}

static bool turnOnce(TurnFunction turnDirection, double targetDeg, int speed, double lead, double* achievedDeg) {
  bool left = (turnDirection == turn_left);
  float h0 = odometry_get().h;
  float last = h0;
  double progress = 0.0;
  unsigned long t0 = millis();
  bool ok = true;

  ledcWrite(pwmChannelSpeed, speed);
  turnDirection();

  while (progress < targetDeg - lead) {
    if (maneuverAbort || (millis() - t0) > TURN_TIMEOUT_MS) {
      ok = false;
      logEvent("obrot: przerwany po %lu ms, zmierzono %.0f z %.0f st., cykl czujnikow sr. %lu max %lu ms", millis() - t0, progress, targetDeg, cycleAvgMs, cycleMaxMs);
      break;
    }
    vTaskDelay(2);
    float h = odometry_get().h;
    float dh = h - last;
    if (dh > 180.0f) dh -= 360.0f;
    if (dh < -180.0f) dh += 360.0f;
    last = h;
    progress += left ? dh : -dh;
  }
  stop_driving();
  vTaskDelay(pdMS_TO_TICKS(ODO_SETTLE_MS));   // dobieg jest doliczany przez odometrię
  float dh = odometry_get().h - h0;
  if (dh > 180.0f) dh -= 360.0f;
  if (dh < -180.0f) dh += 360.0f;
  *achievedDeg = left ? dh : -dh;             // ile faktycznie obróciliśmy się wg żyroskopu (z dobiegiem)
  return ok;
}

// Obrót o targetDeg stopni. Uczy się dobiegu (zatrzymuje wcześniej o tyle, ile robot kręci się po odcięciu
// zasilania) i na końcu dokręca resztę, jeśli błąd jest większy niż kilka stopni.
bool turn(TurnFunction turnDirection, double targetDeg, int speed) {
  if (!odoGyroReady) return false;
  bool left = (turnDirection == turn_left);
  int li = left ? 1 : 0;
  double lead = turnLeadDeg[li];
  if (lead > targetDeg * 0.5) lead = targetDeg * 0.5;
  double got = 0.0;
  bool ok = turnOnce(turnDirection, targetDeg, speed, lead, &got);
  if (ok && targetDeg >= 15.0) {
    // dobieg = o ile przekręciliśmy ponad to, co żyroskop pokazał w chwili odcięcia zasilania
    float learned = (float)(got - (targetDeg - lead));
    if (learned < 0) learned = 0;
    if (learned > 40) learned = 40;
    turnLeadDeg[li] = turnLeadDeg[li] * 0.6f + learned * 0.4f;
  }
  double err = targetDeg - got;
  logEvent("obrot %s: cel %.0f, wykonano %.0f st. (dobieg %.1f)", left ? "w lewo" : "w prawo", targetDeg, got, (double)turnLeadDeg[li]);
  if (ok && !maneuverAbort && fabs(err) > 4.0 && fabs(err) < 60.0) {   // dokrętka: krótki obrót w stronę błędu
    double extra = 0.0;
    bool wantLeft = (err > 0) == left;
    turnOnce(wantLeft ? turn_left : turn_right, fabs(err), speed, 0.0, &extra);
  }
  return ok;
}

// Zapis licznikow od razu (np. tuż przed restartem)
void odometry_saveCounters() {
  Preferences prefs;
  if (prefs.begin("odo", false)) {
    prefs.putFloat("total", (float)odometry_totalCm());
    prefs.putUInt("runs", odometry_runSeconds());
    prefs.end();
  }
}
