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

int measurement0, measurement1, measurement2, measurement3, measurement4, measurement5, measurement6, measurement7;

TaskHandle_t AutomaticDriveTaskHandle  = NULL;
TaskHandle_t CollectDistancesTaskHandle  = NULL;


void checkDriveConditions() {

  void read_obstacle_sensors();

  if (AutomaticDriveTaskHandle==NULL)
  {
    read_obstacle_sensors();

    if (busy_forward == true)
    {
      if (measurement3 <=4 || measurement4 <=35 || measurement5 <=4)
      {
        stop_driving();
      }
    }

    //if (busy_backward == true)
    //{
      //if (measurement6 <= 7)
     // {
      //  stop_driving();
      //}
    //}
  }
}

void read_obstacle_sensors() {
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
  measurement6 = (measure6.RangeStatus != 4) ? measure6.RangeMilliMeter / 10 : 1000;
  measurement7 = (measure7.RangeStatus != 4) ? measure7.RangeMilliMeter / 10 : 1000;
}

void read_servo_sensors() {
  lox0.rangingTest(&measure0, false);
  lox1.rangingTest(&measure1, false);
 
  measurement0 = (measure0.RangeStatus != 4) ? measure0.RangeMilliMeter / 10 : 1000;
  measurement1 = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter / 10 : 1000;
}

void setID() {
  expander.pinMode(SHT_LOX0, OUTPUT);
  expander.pinMode(SHT_LOX1, OUTPUT);
  expander.pinMode(SHT_LOX2, OUTPUT);
  expander.pinMode(SHT_LOX3, OUTPUT);
  expander.pinMode(SHT_LOX4, OUTPUT);
  expander.pinMode(SHT_LOX5, OUTPUT);
  expander.pinMode(SHT_LOX6, OUTPUT);
  expander.pinMode(SHT_LOX7, OUTPUT);

  expander.digitalWrite(SHT_LOX0, LOW);
  expander.digitalWrite(SHT_LOX1, LOW);
  expander.digitalWrite(SHT_LOX2, LOW);
  expander.digitalWrite(SHT_LOX3, LOW);
  expander.digitalWrite(SHT_LOX4, LOW);
  expander.digitalWrite(SHT_LOX5, LOW);
  expander.digitalWrite(SHT_LOX6, LOW);
  expander.digitalWrite(SHT_LOX7, LOW);
  delay(10);

  expander.digitalWrite(SHT_LOX0, HIGH);
  expander.digitalWrite(SHT_LOX1, HIGH);
  expander.digitalWrite(SHT_LOX2, HIGH);
  expander.digitalWrite(SHT_LOX3, HIGH);
  expander.digitalWrite(SHT_LOX4, HIGH);
  expander.digitalWrite(SHT_LOX5, HIGH);
  expander.digitalWrite(SHT_LOX6, HIGH);
  expander.digitalWrite(SHT_LOX7, HIGH);

  delay(10);

  expander.digitalWrite(SHT_LOX0, HIGH);
  expander.digitalWrite(SHT_LOX1, LOW);
  expander.digitalWrite(SHT_LOX2, LOW);
  expander.digitalWrite(SHT_LOX3, LOW);
  expander.digitalWrite(SHT_LOX4, LOW);
  expander.digitalWrite(SHT_LOX5, LOW);
  expander.digitalWrite(SHT_LOX6, LOW);
  expander.digitalWrite(SHT_LOX7, LOW);

  if (!lox0.begin(LOX0_ADDRESS)) {
    Serial.println(F("❌Failed to boot 0 VL53L0X"));
    while (1);
  }
  delay(10);

  expander.digitalWrite(SHT_LOX1, HIGH);
  expander.digitalWrite(SHT_LOX2, LOW);
  expander.digitalWrite(SHT_LOX3, LOW);
  expander.digitalWrite(SHT_LOX4, LOW);
  expander.digitalWrite(SHT_LOX5, LOW);
  expander.digitalWrite(SHT_LOX6, LOW);
    expander.digitalWrite(SHT_LOX7, LOW);
  delay(10);

  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("❌Failed to boot 1 VL53L0X"));
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
    Serial.println(F("❌Failed to boot 2 VL53L0X"));
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
    Serial.println(F("❌Failed to boot 3 VL53L0X"));
    while (1);
  }
  delay(10);

  expander.digitalWrite(SHT_LOX4, HIGH);
  expander.digitalWrite(SHT_LOX5, LOW);
  expander.digitalWrite(SHT_LOX6, LOW);
    expander.digitalWrite(SHT_LOX7, LOW);
  delay(10);

  if (!lox4.begin(LOX4_ADDRESS)) {
    Serial.println(F("❌Failed to boot 4 VL53L0X"));
    while (1);
  }
  delay(10);

  expander.digitalWrite(SHT_LOX5, HIGH);
  expander.digitalWrite(SHT_LOX6, LOW);
    expander.digitalWrite(SHT_LOX7, LOW);
  delay(10);

  if (!lox5.begin(LOX5_ADDRESS)) {
    Serial.println(F("❌Failed to boot 5 VL53L0X"));
    while (1);
  }
  delay(10);

  expander.digitalWrite(SHT_LOX6, HIGH);
      expander.digitalWrite(SHT_LOX7, LOW);
  delay(10);

  if (!lox6.begin(LOX6_ADDRESS)) {
    Serial.println(F("❌Failed to boot 6 VL53L0X"));
    while (1);
  }
  delay(10);

  expander.digitalWrite(SHT_LOX7, HIGH);

  if (!lox7.begin(LOX7_ADDRESS)) {
    Serial.println(F("❌Failed to boot 7 VL53L0X"));
    while (1);
  }
  delay(10);
}

void Distance_sensors_init() {
  Serial.println("🔄Starting VL53L0X...");
  setID();
  Serial.println("✅Shutdown pins VL53L0X inited");
}