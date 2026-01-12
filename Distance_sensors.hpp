#include "Adafruit_VL53L0X.h"
#include <PCF8574.h>

// measurement0 - Servo (Front)
// measurement1 - Servo (Back)
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

// ⭐ PRÓG DLA SENSORA TYLNEGO - wartości 0-3 to wolna przestrzeń (błąd odczytu)
#define BACK_SENSOR_MIN_VALID 4
#define BACK_SENSOR_STOP_DISTANCE 10

Adafruit_VL53L0X lox0 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox5 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox6 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox7 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure0;
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;
VL53L0X_RangingMeasurementData_t measure5;
VL53L0X_RangingMeasurementData_t measure6;
VL53L0X_RangingMeasurementData_t measure7;

PCF8574 expander(EXPANDER_ADDR);

// Użyj volatile dla zmiennych współdzielonych między wątkami
volatile int measurement0, measurement1, measurement2, measurement3, measurement4, measurement5, measurement6, measurement7;

TaskHandle_t AutomaticDriveTaskHandle = NULL;
TaskHandle_t CollectDistancesTaskHandle = NULL;

// Mutex do ochrony odczytów czujników
SemaphoreHandle_t sensorMutex = NULL;

// ⭐ FORWARD DECLARATIONS - deklaracje funkcji przed użyciem
void read_obstacle_sensors();
void read_servo_sensors();

// ⭐ TERAZ można używać tych funkcji
void checkDriveConditions() {
  if (AutomaticDriveTaskHandle == NULL) {
    read_obstacle_sensors();

    if (busy_forward == true) {
      if (measurement3 <= 4 || measurement4 <= 15 || measurement5 <= 4) {
        stop_driving();
      }
    }

    if (busy_backward == true) {
      // ⭐ POPRAWKA: Zatrzymaj tylko gdy measurement6 jest w zakresie 4-7 cm
      // Wartości 0-3 to błędne odczyty oznaczające wolną przestrzeń
      if (measurement6 >= BACK_SENSOR_MIN_VALID && measurement6 <= BACK_SENSOR_STOP_DISTANCE) {
        stop_driving();
      }
    }
  }
}

void read_obstacle_sensors() {
  // Zabezpieczenie mutexem - opcjonalne, jeśli tylko jeden wątek wywołuje tę funkcję
  if (sensorMutex != NULL && xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    lox2.rangingTest(&measure2, false);
    lox3.rangingTest(&measure3, false);
    lox4.rangingTest(&measure4, false);
    lox5.rangingTest(&measure5, false);
    lox6.rangingTest(&measure6, false);
    lox7.rangingTest(&measure7, false);

    measurement2 = (measure2.RangeStatus != 4) ? measure2.RangeMilliMeter / 10 : 1000;
    measurement3 = (measure3.RangeStatus != 4) ? measure3.RangeMilliMeter / 10 : 1000;
    measurement4 = (measure4.RangeStatus != 4) ? measure4.RangeMilliMeter / 10 : 1000;
    measurement5 = (measure5.RangeStatus != 4) ? measure5.RangeMilliMeter / 10 : 1000;
    
    // ⭐ POPRAWKA: Sensor tylny - wartości 0-3 cm zamień na 1000 (wolna przestrzeń)
    int rawMeasurement6 = (measure6.RangeStatus != 4) ? measure6.RangeMilliMeter / 10 : 1000;
    if (rawMeasurement6 >= 0 && rawMeasurement6 <= 3) {
      measurement6 = 1000;  // Traktuj jako wolną przestrzeń
    } else {
      measurement6 = rawMeasurement6;
    }
    
    measurement7 = (measure7.RangeStatus != 4) ? measure7.RangeMilliMeter / 10 : 1000;

    xSemaphoreGive(sensorMutex);
  }
}

void read_servo_sensors() {
  // Zabezpieczenie mutexem dla czujników servo
  if (sensorMutex != NULL && xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    lox0.rangingTest(&measure0, false);
    lox1.rangingTest(&measure1, false);
   
    measurement0 = (measure0.RangeStatus != 4) ? measure0.RangeMilliMeter / 10 : 1000;
    measurement1 = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter / 10 : 1000;

    xSemaphoreGive(sensorMutex);
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
  Serial.println(F("✅LOX0 initialized"));
  delay(50);

  // Inicjalizacja czujnika 1
  expander.digitalWrite(SHT_LOX1, HIGH);
  delay(50);

  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX1"));
    while (1);
  }
  Serial.println(F("✅LOX1 initialized"));
  delay(50);

  // Inicjalizacja czujnika 2
  expander.digitalWrite(SHT_LOX2, HIGH);
  delay(50);

  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX2"));
    while (1);
  }
  Serial.println(F("✅LOX2 initialized"));
  delay(50);

  // Inicjalizacja czujnika 3
  expander.digitalWrite(SHT_LOX3, HIGH);
  delay(50);

  if (!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX3"));
    while (1);
  }
  Serial.println(F("✅LOX3 initialized"));
  delay(50);

  // Inicjalizacja czujnika 4
  expander.digitalWrite(SHT_LOX4, HIGH);
  delay(50);

  if (!lox4.begin(LOX4_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX4"));
    while (1);
  }
  Serial.println(F("✅LOX4 initialized"));
  delay(50);

  // Inicjalizacja czujnika 5
  expander.digitalWrite(SHT_LOX5, HIGH);
  delay(50);

  if (!lox5.begin(LOX5_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX5"));
    while (1);
  }
  Serial.println(F("✅LOX5 initialized"));
  delay(50);

  // Inicjalizacja czujnika 6
  expander.digitalWrite(SHT_LOX6, HIGH);
  delay(50);

  if (!lox6.begin(LOX6_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX6"));
    while (1);
  }
  Serial.println(F("✅LOX6 initialized"));
  delay(50);

  // Inicjalizacja czujnika 7
  expander.digitalWrite(SHT_LOX7, HIGH);
  delay(50);

  if (!lox7.begin(LOX7_ADDRESS)) {
    Serial.println(F("❌Failed to boot LOX7"));
    while (1);
  }
  Serial.println(F("✅LOX7 initialized"));
  delay(50);
}

void Distance_sensors_init() {
  Serial.println("🔄Starting VL53L0X sensors...");
  
  // 1. NAJPIERW inicjalizuj expander
  expander.begin();
  delay(100);
  Serial.println("✅PCF8574 expander initialized");
  
  // 2. Utwórz mutex do ochrony czujników
  sensorMutex = xSemaphoreCreateMutex();
  if (sensorMutex == NULL) {
    Serial.println("❌Failed to create sensor mutex!");
    while(1);
  }
  Serial.println("✅Sensor mutex created");
  
  // 3. TERAZ inicjalizuj czujniki
  setID();
  
  Serial.println("✅All VL53L0X sensors initialized");
}