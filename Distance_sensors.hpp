#include "Adafruit_VL53L0X.h"
#include <PCF8574.h>
#include "Odometry.hpp"
#include "Log.hpp"

void fixedSensorsMap();   // Scan.hpp: przeszkody z czujników stałych trafiają też na mapę

// czujnik 0 - Servo (przód), czujnik 1 - Servo (tył): tylko skan mapy, patrz Scan.hpp
// measurement2 - Left Front 
// measurement3 - Front Left   
// measurement4 - Front Center 
// measurement5 - Front Right
// measurement6 - Back 
// measurement7 - Left Back

// Addresses for the VL53L0X sensors
#define LOX0_ADDRESS 0x10
#define LOX1_ADDRESS 0x11
#define LOX2_ADDRESS 0x12
#define LOX3_ADDRESS 0x13
#define LOX4_ADDRESS 0x14
#define LOX5_ADDRESS 0x15
#define LOX6_ADDRESS 0x16
#define LOX7_ADDRESS 0x17

// Pins to control the sensor shutdown
#define SHT_LOX0 0
#define SHT_LOX1 1
#define SHT_LOX2 2
#define SHT_LOX3 3
#define SHT_LOX4 4
#define SHT_LOX5 5
#define SHT_LOX6 6
#define SHT_LOX7 7

// I/O expander address
#define EXPANDER_ADDR 0x24

// Tylny czujnik: zwykłe odczyty od 3 cm. Mniej = za blisko dla czujnika (siedzi 1,5 cm w głębi robota), wtedy
// VL53L0X daje brak echa albo śmieci - patrz niżej (zatrzask "blisko").
#define BACK_SENSOR_MIN_VALID 3
#define BACK_SENSOR_STOP_DISTANCE 10
#define NEAR_LATCH_CM 15          // odczyt <= tyle = przeszkoda tuż-tuż; jeśli zaraz potem czujnik "traci" echo, to znaczy że jest jeszcze bliżej
#define NEAR_LATCH_FRONT_MS 2000
#define NEAR_LATCH_REAR_MS 4000
#define SENSOR_STUCK_MS 1000      // tyle bez nowego odczytu = restart pomiarów czujnika

Adafruit_VL53L0X lox0 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox5 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox6 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox7 = Adafruit_VL53L0X();


PCF8574 expander(EXPANDER_ADDR);

// Użyj volatile dla zmiennych współdzielonych między wątkami
volatile int measurement2, measurement3, measurement4, measurement5, measurement6, measurement7;
volatile int rearCloseCount = 0;   // ile kolejnych odczytów tylnego czujnika mieści się w zakresie 3-10 cm
volatile uint16_t sensorRestarts = 0;   // ile razy trzeba było wskrzesić zawieszony czujnik (diagnostyka)

extern TaskHandle_t AutomaticDriveTaskHandle;      // Explore.hpp


// ---------------------------------------------------------------------------
// Czujniki przeszkód (2..7) pracują w trybie CIĄGŁYM: każdy mierzy sam w tle
// (co 40 ms), a zadanie czujników tylko odbiera gotowe wyniki. Dzięki temu:
//  - loop() i auto-jazda nie czekają na pomiary (wcześniej 6 x ok. 33 ms w każdej iteracji),
//  - magistrala I2C jest zajęta na krótko (blokada I2C trzymana tylko na czas odczytu),
//  - wiemy, KIEDY dany pomiar był aktualny - robot nie jedzie na starych danych.
// Czujniki serwa (0, 1) działają w trybie pojedynczym (rangingTest) i służą tylko do skanu mapy (Scan.hpp).
// ---------------------------------------------------------------------------
#define OBSTACLE_PERIOD_MS 40   // okres pomiarów w trybie ciągłym (musi być >= budżet czasowy ok. 33 ms)

bool obstacleSensorActive[8] = {false};
volatile unsigned long sensorLastOkMs[8] = {0};   // kiedy ostatnio odebrano poprawny pomiar

// Wiek ostatniego odczytu [ms]; 99999 = jeszcze nie było. Liczone ze znakiem: inne zadanie mogło zapisać odczyt
// chwilę PO naszym millis() - bez tego różnica "zawijała się" do ogromnej liczby i świeży odczyt wyglądał na stary.
long sensorAgeMs(int idx) {
  unsigned long last = sensorLastOkMs[idx];
  if (last == 0) return 99999;
  long a = (long)(millis() - last);
  return a < 0 ? 0 : a;
}

// Czy dane z czujników przodu (3, 4, 5) są świeże
bool frontSensorsFresh() {
  return sensorAgeMs(3) <= SENSOR_MAX_AGE_MS && sensorAgeMs(4) <= SENSOR_MAX_AGE_MS && sensorAgeMs(5) <= SENSOR_MAX_AGE_MS;
}

// Czy dane z czujnika tylnego (6) są świeże
bool backSensorFresh() {
  return sensorAgeMs(6) <= SENSOR_MAX_AGE_REAR_MS;
}

// Przód zablokowany: przeszkoda za blisko LUB brak świeżych danych (fail-safe)
bool frontBlocked() {
  return !frontSensorsFresh() ||
         measurement3 <= STOP_FRONT_SIDE_CM ||
         measurement4 <= STOP_FRONT_CENTER_CM ||
         measurement5 <= STOP_FRONT_SIDE_CM;
}

// Tył zablokowany: przeszkoda w zakresie 3-10 cm (bliżej niż 3 cm = zatrzask "blisko", patrz wyżej)
// LUB brak świeżych danych (fail-safe)
// Przeszkodę uznajemy dopiero po 2 kolejnych odczytach w zakresie (pojedynczy fałszywy odczyt
// nie zatrzymuje już jazdy do tyłu).
bool rearBlocked() {
  return !backSensorFresh() || rearCloseCount >= 2;
}

// Odbiera wynik jednego czujnika pracującego w trybie ciągłym (jeśli jest gotowy).
// Zatrzask "blisko" (czujniki bezpieczeństwa 3-6): gdy ostatni dobry odczyt był <= 15 cm, a potem czujnik
// przestaje widzieć (brak echa / 0-3 cm - przedmiot przyłożony tuż przed czujnikiem), raportujemy 3 cm, a nie "daleko".
static unsigned long nearSeenMs[8] = {0};
static bool nearLatched[8] = {false};

static void serviceObstacleSensor(int idx, Adafruit_VL53L0X &lox, volatile int &target) {
  if (!obstacleSensorActive[idx]) return;

  I2CLock lock(30);
  if (!lock.ok()) return;

  unsigned long now = millis();
  if (sensorAgeMs(idx) > SENSOR_STUCK_MS && sensorLastOkMs[idx] != 0) {   // zawieszony pomiar: restart trybu ciągłego
    lox.stopRangeContinuous();
    obstacleSensorActive[idx] = lox.startRangeContinuous(OBSTACLE_PERIOD_MS);
    sensorLastOkMs[idx] = now;                       // nie restartuj w kółko
    sensorRestarts++;
    logEvent("czujnik %d: brak odczytu, restart", idx);
    return;
  }

  bool ready = lox.isRangeComplete();
  if (lox.Status != VL53L0X_ERROR_NONE) return;   // błąd I2C - nie aktualizujemy (dane zestarzeją się)
  if (!ready) return;

  uint16_t mm = lox.readRangeResult();            // 0xFFFF = brak echa (status 4)
  if (lox.Status != VL53L0X_ERROR_NONE) return;

  int cm = (mm == 0xFFFF) ? 1000 : (int)(mm / 10);
  bool safety = idx >= 3 && idx <= 6;
  bool lost = (mm == 0xFFFF) || (idx == 6 && cm < BACK_SENSOR_MIN_VALID);
  if (safety) {
    if (!lost) {
      nearLatched[idx] = cm <= NEAR_LATCH_CM;
      if (nearLatched[idx]) nearSeenMs[idx] = now;
    } else {
      unsigned long hold = (idx == 6) ? NEAR_LATCH_REAR_MS : NEAR_LATCH_FRONT_MS;
      cm = (nearLatched[idx] && now - nearSeenMs[idx] < hold) ? BACK_SENSOR_MIN_VALID : 1000;
    }
  } else if (lost) cm = 1000;
  if (idx == 6) {
    if (cm >= BACK_SENSOR_MIN_VALID && cm <= BACK_SENSOR_STOP_DISTANCE) { if (rearCloseCount < 10) rearCloseCount = rearCloseCount + 1; }
    else rearCloseCount = 0;
  }

  target = cm;
  sensorLastOkMs[idx] = now;
}

// Zadanie czujników: odbiera pomiary przeszkód i liczy odometrię (co ok. 10 ms) oraz odświeża wolne
// czujniki (napięcia, BMP180) raz na sekundę. TFT, WWW i buzzer czytają tylko wyniki z pamięci.
void sensors_Task(void *pvParameters) {
  unsigned long lastSlow = 0, lastObs = 0, lastStat = 0, sumMs = 0, cnt = 0, maxMs = 0, prev = millis();
  while (1) {
    odometry_update();   // droga z enkodera i kąt z żyroskopu (patrz Odometry.hpp)
    unsigned long nowMs = millis();
    if (nowMs - lastObs >= 20) {   // czujniki mierzą co 40 ms - odpytywanie co 20 ms wystarcza i oszczędza magistralę
      lastObs = nowMs;
      serviceObstacleSensor(2, lox2, measurement2);
      serviceObstacleSensor(3, lox3, measurement3);
      serviceObstacleSensor(4, lox4, measurement4);
      serviceObstacleSensor(5, lox5, measurement5);
      serviceObstacleSensor(6, lox6, measurement6);
      serviceObstacleSensor(7, lox7, measurement7);
    }
    fixedSensorsMap();

    unsigned long now = millis();
    if (now - lastSlow >= 1000) {
      lastSlow = now;
      Voltages_update();
      BMP_update();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    unsigned long t = millis();
    unsigned long d = t - prev; prev = t;
    sumMs += d; cnt++; if (d > maxMs) maxMs = d;
    if (t - lastStat >= 1000) {
      lastStat = t;
      cycleAvgMs = cnt ? sumMs / cnt : 0; cycleMaxMs = maxMs;
      sumMs = 0; cnt = 0; maxMs = 0;
    }
  }
}

// Zatrzymuje jazdę, gdy przeszkoda jest za blisko albo dane są nieaktualne.
// Wołane z loop() - nie czeka już na czujniki, tylko sprawdza gotowe wyniki.
void checkDriveConditions() {
  if (AutomaticDriveTaskHandle != NULL) return;   // auto-jazda sprawdza przeszkody sama

  if (busy_forward && frontBlocked()) {
    stop_driving();
  }
  if (busy_backward && rearBlocked()) {
    stop_driving();
  }
}

void setID() {
  // ⚠️ WAŻNE: Upewnij się, że expander jest zainicjalizowany PRZED wywołaniem tej funkcji!
  
  // Konfiguracja pinów jako wyjścia
  expander.pinMode(SHT_LOX0, OUTPUT);
  expander.pinMode(SHT_LOX1, OUTPUT);
  expander.pinMode(SHT_LOX2, OUTPUT);
  expander.pinMode(SHT_LOX3, OUTPUT);
  expander.pinMode(SHT_LOX4, OUTPUT);
  expander.pinMode(SHT_LOX5, OUTPUT);
  expander.pinMode(SHT_LOX6, OUTPUT);
  expander.pinMode(SHT_LOX7, OUTPUT);

  // Reset wszystkich czujników
  expander.digitalWrite(SHT_LOX0, LOW);
  expander.digitalWrite(SHT_LOX1, LOW);
  expander.digitalWrite(SHT_LOX2, LOW);
  expander.digitalWrite(SHT_LOX3, LOW);
  expander.digitalWrite(SHT_LOX4, LOW);
  expander.digitalWrite(SHT_LOX5, LOW);
  expander.digitalWrite(SHT_LOX6, LOW);
  expander.digitalWrite(SHT_LOX7, LOW);
  delay(50);  // Zwiększony czas resetu

  // Włącz wszystkie czujniki
  expander.digitalWrite(SHT_LOX0, HIGH);
  expander.digitalWrite(SHT_LOX1, HIGH);
  expander.digitalWrite(SHT_LOX2, HIGH);
  expander.digitalWrite(SHT_LOX3, HIGH);
  expander.digitalWrite(SHT_LOX4, HIGH);
  expander.digitalWrite(SHT_LOX5, HIGH);
  expander.digitalWrite(SHT_LOX6, HIGH);
  expander.digitalWrite(SHT_LOX7, HIGH);
  delay(50);

  // Inicjalizacja czujnika 0
  expander.digitalWrite(SHT_LOX0, HIGH);
  expander.digitalWrite(SHT_LOX1, LOW);
  expander.digitalWrite(SHT_LOX2, LOW);
  expander.digitalWrite(SHT_LOX3, LOW);
  expander.digitalWrite(SHT_LOX4, LOW);
  expander.digitalWrite(SHT_LOX5, LOW);
  expander.digitalWrite(SHT_LOX6, LOW);
  expander.digitalWrite(SHT_LOX7, LOW);
  delay(50);

  if (!lox0.begin(LOX0_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX0"));
    while (1);
  }
  delay(50);

  // Inicjalizacja czujnika 1
  expander.digitalWrite(SHT_LOX1, HIGH);
  delay(50);

  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX1"));
    while (1);
  }
  delay(50);

  // Inicjalizacja czujnika 2
  expander.digitalWrite(SHT_LOX2, HIGH);
  delay(50);

  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX2"));
    while (1);
  }
  delay(50);

  // Inicjalizacja czujnika 3
  expander.digitalWrite(SHT_LOX3, HIGH);
  delay(50);

  if (!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX3"));
    while (1);
  }
  delay(50);

  // Inicjalizacja czujnika 4
  expander.digitalWrite(SHT_LOX4, HIGH);
  delay(50);

  if (!lox4.begin(LOX4_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX4"));
    while (1);
  }
  delay(50);

  // Inicjalizacja czujnika 5
  expander.digitalWrite(SHT_LOX5, HIGH);
  delay(50);

  if (!lox5.begin(LOX5_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX5"));
    while (1);
  }
  delay(50);

  // Inicjalizacja czujnika 6
  expander.digitalWrite(SHT_LOX6, HIGH);
  delay(50);

  if (!lox6.begin(LOX6_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX6"));
    while (1);
  }
  delay(50);

  // Inicjalizacja czujnika 7
  expander.digitalWrite(SHT_LOX7, HIGH);
  delay(50);

  if (!lox7.begin(LOX7_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX7"));
    while (1);
  }
  delay(50);
}

// Przełącza czujnik przeszkód w tryb ciągły; przy błędzie czujnik zostaje nieaktywny,
// jego dane nigdy nie będą świeże, więc robot (fail-safe) nie pojedzie w tym kierunku.
static void startObstacleSensor(int idx, Adafruit_VL53L0X &lox) {
  I2CLock lock(200);
  obstacleSensorActive[idx] = lox.startRangeContinuous(OBSTACLE_PERIOD_MS);
  if (obstacleSensorActive[idx]) {
  } else {
    Serial.printf("LOX%d: blad trybu ciaglego\n", idx);
  }
}

void Distance_sensors_init() {

  // 1. NAJPIERW inicjalizuj expander
  expander.begin();
  delay(100);

  // 2. TERAZ inicjalizuj czujniki (nadają im adresy)
  setID();

  // 3. Czujniki przeszkód -> tryb ciągły
  startObstacleSensor(2, lox2);
  startObstacleSensor(3, lox3);
  startObstacleSensor(4, lox4);
  startObstacleSensor(5, lox5);
  startObstacleSensor(6, lox6);
  startObstacleSensor(7, lox7);

}
