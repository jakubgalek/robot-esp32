#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include <PCF8574.h>

// Addresses for the VL53L0X sensors
#define LOX1_ADDRESS 0x11
#define LOX2_ADDRESS 0x12
#define LOX3_ADDRESS 0x13
#define LOX4_ADDRESS 0x14
#define LOX5_ADDRESS 0x15
#define LOX6_ADDRESS 0x16
#define LOX7_ADDRESS 0x17

int sensor1, sensor2, sensor3, sensor4, sensor5, sensor6, sensor7;

// Pins to control the sensor shutdown
#define SHT_LOX1 0
#define SHT_LOX2 1
#define SHT_LOX3 2
#define SHT_LOX4 3
#define SHT_LOX5 4
#define SHT_LOX6 5
#define SHT_LOX7 6

// I/O expander address
#define EXPANDER_ADDR 0x24

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox5 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox6 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox7 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;
VL53L0X_RangingMeasurementData_t measure5;
VL53L0X_RangingMeasurementData_t measure6;
VL53L0X_RangingMeasurementData_t measure7;

PCF8574 expander(EXPANDER_ADDR);

void setID() {
  expander.pinMode(SHT_LOX1, OUTPUT);
  expander.pinMode(SHT_LOX2, OUTPUT);
  expander.pinMode(SHT_LOX3, OUTPUT);
  expander.pinMode(SHT_LOX4, OUTPUT);
  expander.pinMode(SHT_LOX5, OUTPUT);
  expander.pinMode(SHT_LOX6, OUTPUT);
  expander.pinMode(SHT_LOX7, OUTPUT);

  expander.digitalWrite(SHT_LOX1, LOW);
  expander.digitalWrite(SHT_LOX2, LOW);
  expander.digitalWrite(SHT_LOX3, LOW);
  expander.digitalWrite(SHT_LOX4, LOW);
  expander.digitalWrite(SHT_LOX5, LOW);
  expander.digitalWrite(SHT_LOX6, LOW);
  expander.digitalWrite(SHT_LOX7, LOW);
  delay(10);

  expander.digitalWrite(SHT_LOX1, HIGH);
  expander.digitalWrite(SHT_LOX2, HIGH);
  expander.digitalWrite(SHT_LOX3, HIGH);
  expander.digitalWrite(SHT_LOX4, HIGH);
  expander.digitalWrite(SHT_LOX5, HIGH);
  expander.digitalWrite(SHT_LOX6, HIGH);
  expander.digitalWrite(SHT_LOX7, HIGH);
  delay(10);

  expander.digitalWrite(SHT_LOX1, HIGH);
  expander.digitalWrite(SHT_LOX2, LOW);
  expander.digitalWrite(SHT_LOX3, LOW);
  expander.digitalWrite(SHT_LOX4, LOW);
  expander.digitalWrite(SHT_LOX5, LOW);
  expander.digitalWrite(SHT_LOX6, LOW);
  expander.digitalWrite(SHT_LOX7, LOW);

  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1);
  }
  delay(10);

  expander.digitalWrite(SHT_LOX2, HIGH);
  expander.digitalWrite(SHT_LOX3, LOW);
  expander.digitalWrite(SHT_LOX4, LOW);
  expander.digitalWrite(SHT_LOX5, LOW);
  expander.digitalWrite(SHT_LOX6, LOW);
  expander.digitalWrite(SHT_LOX7, LOW);
  delay(10);

  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1);
  }
  delay(10);

  expander.digitalWrite(SHT_LOX3, HIGH);
  expander.digitalWrite(SHT_LOX4, LOW);
  expander.digitalWrite(SHT_LOX5, LOW);
  expander.digitalWrite(SHT_LOX6, LOW);
  expander.digitalWrite(SHT_LOX7, LOW);
  delay(10);

  if (!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while (1);
  }
  delay(10);

  expander.digitalWrite(SHT_LOX4, HIGH);
  expander.digitalWrite(SHT_LOX5, LOW);
  expander.digitalWrite(SHT_LOX6, LOW);
  expander.digitalWrite(SHT_LOX7, LOW);
  delay(10);

  if (!lox4.begin(LOX4_ADDRESS)) {
    Serial.println(F("Failed to boot fourth VL53L0X"));
    while (1);
  }
  delay(10);

  expander.digitalWrite(SHT_LOX5, HIGH);
  expander.digitalWrite(SHT_LOX6, LOW);
  expander.digitalWrite(SHT_LOX7, LOW);
  delay(10);

  if (!lox5.begin(LOX5_ADDRESS)) {
    Serial.println(F("Failed to boot fifth VL53L0X"));
    while (1);
  }
  delay(10);

  expander.digitalWrite(SHT_LOX6, HIGH);
  expander.digitalWrite(SHT_LOX7, LOW);
  delay(10);

  if (!lox6.begin(LOX6_ADDRESS)) {
    Serial.println(F("Failed to boot sixth VL53L0X"));
    while (1);
  }
  delay(10);

  expander.digitalWrite(SHT_LOX7, HIGH);
  delay(10);

  if (!lox7.begin(LOX7_ADDRESS)) {
    Serial.println(F("Failed to boot seventh VL53L0X"));
    while (1);
  }
}

void read_seven_sensors() {
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);
  lox4.rangingTest(&measure4, false);
  lox5.rangingTest(&measure5, false);
  lox6.rangingTest(&measure6, false);
  lox7.rangingTest(&measure7, false);

  if (measure1.RangeStatus != 4) {
    Serial.print("1: ");
    Serial.print(measure1.RangeMilliMeter);
    Serial.println("mm");
  } else {
    Serial.println("1: Out of range or measurement invalid");
  }

  if (measure2.RangeStatus != 4) {
    Serial.print("2: ");
    Serial.print(measure2.RangeMilliMeter);
    Serial.println("mm");
  } else {
    Serial.println("2: Out of range or measurement invalid");
  }

  if (measure3.RangeStatus != 4) {
    Serial.print("3: ");
    Serial.print(measure3.RangeMilliMeter);
    Serial.println("mm");
  } else {
    Serial.println("3: Out of range or measurement invalid");
  }

  if (measure4.RangeStatus != 4) {
    Serial.print("4: ");
    Serial.print(measure4.RangeMilliMeter);
    Serial.println("mm");
  } else {
    Serial.println("4: Out of range or measurement invalid");
  }

  if (measure5.RangeStatus != 4) {
    Serial.print("5: ");
    Serial.print(measure5.RangeMilliMeter);
    Serial.println("mm");
  } else {
    Serial.println("5: Out of range or measurement invalid");
  }

  if (measure6.RangeStatus != 4) {
    Serial.print("6: ");
    Serial.print(measure6.RangeMilliMeter);
    Serial.println("mm");
  } else {
    Serial.println("6: Out of range or measurement invalid");
  }

  if (measure7.RangeStatus != 4) {
    Serial.print("7: ");
    Serial.print(measure7.RangeMilliMeter);
    Serial.println("mm");
  } else {
    Serial.println("7: Out of range or measurement invalid");
  }
}


void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(1);
  }

  Serial.println("Shutdown pins inited...");

  Wire.begin();

  Serial.println("Starting...");

  setID();
}

void loop() {
  read_seven_sensors();
  delay(1000);
}