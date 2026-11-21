#include <Adafruit_BMP085.h>
#include <stdio.h>
#include "I2C.hpp"

Adafruit_BMP085 bmp;

bool bmpOk = false;   // false, gdy czujnika nie znaleziono (wtedy nie czytamy śmieci)

// Ostatnie pomiary - odświeża BMP_update() z zadania czujników (raz na sekundę)
volatile float cachedTemperature = 0.0f;
volatile int cachedAltitude = 0;
volatile int cachedPressure = 0;

/* ------------------------------------------------- */
float roundToOneDecimal(float value) {
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "%.1f", value);
    return atof(buffer);
}
/* ------------------------------------------------- */
void BMP_update() {
  if (!bmpOk) return;
  I2CLock lock(100);
  if (!lock.ok()) return;
  float temperature = bmp.readTemperature() - 1;
  cachedTemperature = roundToOneDecimal(temperature);
  cachedAltitude = (int)bmp.readAltitude();
  float pressure = (bmp.readPressure() + 4) / 100.0;
  cachedPressure = (int)(pressure + 0.5);
}
/* ------------------------------------------------- */
void BMP_init() {
  bmpOk = bmp.begin();
  if (!bmpOk) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    return;
  }
  BMP_update();
}
/* ------------------------------------------------- */
float getFormattedTemperature() {
  return cachedTemperature;
}
/* ------------------------------------------------- */
int getFormattedAltitude() {
  return cachedAltitude;
}
/* ------------------------------------------------- */
