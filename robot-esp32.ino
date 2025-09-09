#include <Arduino.h>
#include "WiFi_config.hpp"
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
#include <Ticker.h>

SemaphoreHandle_t xMutex;

Ticker mpuTicker;

void updateMPU() {
  mpu.update();
}

float currentAngleZ = 0;          // Do tymczasowej integracji
unsigned long lastAngleUpdate = 0;

void setup() 
{
  Serial.begin(115200);

  Wire.begin();

  pinMode(ENCODER_PIN, INPUT); // Ustawienie pinu enkodera jako wejście
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderInterrupt, RISING); // Przerwanie na rosnącej krawędzi

  Time_init();

  Buzzer_setup();

  pcf8574_init();
  
  BMP_init();
  
  INA3221_measure_init();
  
  Distance_sensors_init();

  startWiFi();

  //startOTA();

  TFTsetup();

  refreshTFT();

  ledcSetup(pwmChannelSpeed, pwmFrequencySpeed, pwmResolution);
  ledcAttachPin(SPEED_PIN, pwmChannelSpeed);

  ledcWrite(pwmChannelSpeed, motorSpeed);

  ledcSetup(pwmChannelServo, pwmFrequencyServo, pwmResolution); 
  ledcAttachPin(SERVO_PIN, pwmChannelServo);

  servoMotor.attach(SERVO_PIN);
  servoMotor.write(0);

  
  // Inicjalizacja MPU6050
  mpu.begin();
  mpu.calcGyroOffsets();
 
  mpuTicker.attach_ms(5, updateMPU);  // Co 5 ms

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
  //ArduinoOTA.handle();
  Time_refresh();
  checkWiFiSignal();
  server.handleClient();
  refreshTFT();
  checkDriveConditions();
  checkMotorsAndReloadBuzzer();

  // Tutaj tylko obliczenia kąta:
  //static unsigned long last = micros();
  //unsigned long now = micros();
  //float delta = (now - last) / 1000000.0;

  //if (delta > 0.005) {
    //float gyroZ = mpu.getGyroZ();
    //robotAngle -= gyroZ * delta;

    //robotAngle = fmod(robotAngle, 360.0);
    //if (robotAngle < 0) robotAngle -= 360.0;

    //last = now;
  //}
// Free thread memory test
/*
UBaseType_t stackWaterMark = uxTaskGetStackHighWaterMark(CollectDistancesTaskHandle);
UBaseType_t stackWaterMark2 = uxTaskGetStackHighWaterMark(AutomaticDriveTaskHandle);

Serial.print("FreeHeap: "); Serial.println(ESP.getFreeHeap());
Serial.printf("Stack CollectDistancesTaskHandle: %u bytes\n", stackWaterMark);
Serial.printf("Stack AutomaticDriveTaskHandle: %u bytes\n", stackWaterMark2);
*/



}