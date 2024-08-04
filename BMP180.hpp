#include <Adafruit_BMP085.h>
#include <stdio.h>

Adafruit_BMP085 bmp;


/* ------------------------------------------------- */
void BMP_init() {
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  }
}
/* ------------------------------------------------- */
float roundToOneDecimal(float value) {
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "%.1f", value);
    return atof(buffer);
}
/* ------------------------------------------------- */
float getFormattedTemperature() {
  float temperature = bmp.readTemperature() - 1;
  return roundToOneDecimal(temperature);
}
/* ------------------------------------------------- */
int getFormattedAltitude() {
  int altitude = bmp.readAltitude();
  return altitude;
}
/* ------------------------------------------------- */
int getFormattedPressure() {
  float pressure = bmp.readPressure() / 100.0; 
  return (int)(pressure + 0.5);
}
/* ------------------------------------------------- */