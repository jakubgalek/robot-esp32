#include <Arduino.h>
#include "WiFi_config.hpp"
#define numAngles 180
volatile int radarData[numAngles][2];
#include <Wire.h>
#include "Voltages.hpp"
#include "Time.hpp"
#include "Buzzer.hpp"
#include "Motor_functions.hpp"
#include "OTA.hpp"
#include "BMP180.hpp"
#include "Distance_sensors.hpp"
#include "TFT.hpp"
#include "Map.hpp"
#include "Webpage.hpp"


static const int SERVO_PIN = 25;

const int pwmFrequency = 1000; // (1 kHz)
const int pwmResolution = 8; // (8 bit)


SemaphoreHandle_t xMutex;


void setup() 
{
  Serial.begin(115200);

  Wire.begin();

  Time_init();

  Buzzer_setup();

  pcf8574_init();
  
  BMP_init();
  
  INA3221_measure_init();
  
  Distance_sensors_init();

  startWiFi();

  startOTA();

  TFTsetup();

  refreshTFT();

  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(SPEED_PIN, pwmChannel);

  servoMotor.attach(SERVO_PIN);
  servoMotor.write(0);

  Web_init();
  
  xMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    collect_distances_servo_Task,       // Function to execute in the task
    "CollectDistancesServoTask",        // Task name
    7000,                               // Stack size (adjust as needed)
    NULL,                               // Parameters supplied to the thread
    1,                                  // Task priority
    &CollectDistancesTaskHandle,        // Task handle
    1                                   // Core number (0 or 1)
  );                

}

void loop()
{
  ArduinoOTA.handle();
 
  Time_refresh();

  checkWiFiSignal();

  server.handleClient();
  refreshTFT();

  checkForwardDriveConditions();
  checkMotorsAndReloadBuzzer();
  
// Free thread memory test

/*UBaseType_t stackWaterMark = uxTaskGetStackHighWaterMark(CollectDistancesTaskHandle);
UBaseType_t stackWaterMark2 = uxTaskGetStackHighWaterMark(AutomaticDriveTaskHandle);

Serial.printf("Stack CollectDistancesTaskHandle: %u bytes\n", stackWaterMark);
Serial.printf("Stack AutomaticDriveTaskHandle: %u bytes\n", stackWaterMark2);
*/

}


