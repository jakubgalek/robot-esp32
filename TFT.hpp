#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_Sensor.h>

#define TFT_DC 4
#define TFT_CS 5

#define TFT_MOSI 23
#define TFT_CLK 18
#define TFT_RST 2
#define TFT_MISO 19

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

void TFTsetup() {
  tft.begin();
  tft.setRotation(2);
  tft.fillScreen(ILI9341_BLACK);

  // Draw static text
  tft.setCursor(70, 90);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("ESP32:");

  tft.setCursor(70, 170);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Silniki:");

  tft.setCursor(70, 245);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("WiFi:");

  // Draw rectangles
  tft.drawFastHLine(0, 73, 240, ILI9341_WHITE);
  tft.drawFastHLine(0, 150, 240, ILI9341_WHITE);
  tft.drawFastHLine(0, 230, 240, ILI9341_WHITE);
}

void updateVoltageValues(float voltageMotors, float voltageESP) {
  static float lastVoltageMotors = -1;
  static float lastVoltageESP = -1;

  if (voltageESP != lastVoltageESP) {
    tft.setTextSize(3);
    tft.setTextColor(ILI9341_BLUE);
    tft.setCursor(69, 120);
    tft.fillRect(69, 120, 100, 22, ILI9341_BLACK); // Clear the previous value
    tft.print(voltageESP);
    tft.print("V");
    lastVoltageESP = voltageESP;
  }

  if (voltageMotors != lastVoltageMotors) {
    tft.setTextSize(3);
    tft.setTextColor(ILI9341_BLUE);
    tft.setCursor(69, 200);
    tft.fillRect(69, 200, 100, 22, ILI9341_BLACK); // Clear the previous value
    tft.print(voltageMotors);
    tft.print("V");
    lastVoltageMotors = voltageMotors;
  }
}



void updateWifiSignal() {

  if (signalStrength != lastSignalStrength) {
    tft.setTextSize(3);
    tft.setTextColor(ILI9341_YELLOW);
    tft.setCursor(70, 270);
    tft.fillRect(70, 270, 180, 25, ILI9341_BLACK); // Clear the previous value
    tft.print(signalStrength);
    lastSignalStrength = signalStrength;
  }
}

void updateAdditionalInfo() {
  static int lastHour = -1;
  static int lastMinute = -1;
  static float lastTemperature = -1.0;
  static int lastAltitude = -1;
 
 
  float temperature = getFormattedTemperature();
  int altitude = getFormattedAltitude();

  static IPAddress lastIP;

  int actualhour = dt.hour;
  int actualminute = dt.minute;

  IPAddress currentIP = WiFi.localIP();

  if (actualhour != lastHour || actualminute != lastMinute) {
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_YELLOW);
    tft.fillRect(tft.width() - 70, 1, 70, 20, ILI9341_BLACK);
    tft.setCursor(tft.width() - 70, 1);

    // Format the time with leading zeros
    char timeStr[6];
    sprintf(timeStr, "%02d:%02d", actualhour, actualminute);
    tft.print(timeStr);

    lastHour = actualhour;
    lastMinute = actualminute;
  }

  if (temperature != lastTemperature) {
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_YELLOW);
    tft.fillRect(20, 40, 70, 20, ILI9341_BLACK);
    tft.setCursor(20, 40);
    char tempStr[6];
    dtostrf(temperature, 4, 1, tempStr);
    tft.print(tempStr);
    tft.print(" C");

    lastTemperature = temperature;
  }

    if (altitude != lastAltitude) {
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_YELLOW);
    tft.fillRect(tft.width() - 90, 40, 70, 20, ILI9341_BLACK);
    tft.setCursor(tft.width() - 90, 40);
    tft.print(altitude);
    tft.print(" m");

    lastAltitude = altitude;
  }

  if (currentIP != lastIP) {
    tft.setTextSize(1);
    tft.fillRect(1, 1, 70, 20, ILI9341_BLACK);
    tft.setCursor(1, 1);
    tft.setTextColor(ILI9341_WHITE);
    tft.print("IP:");
    tft.setTextColor(ILI9341_YELLOW);
    tft.println(currentIP);
    lastIP = currentIP;
  }
}

void refreshTFT() {
  static unsigned long lastUpdate = 0;
  const unsigned long updateInterval = 1000; // Update every 1000 ms
  unsigned long currentTime = millis();

  if (currentTime - lastUpdate >= updateInterval) {
    float voltageMotors = readVoltageMotors();
    float voltageESP = readVoltageESP32();

    updateVoltageValues(voltageMotors, voltageESP);
    updateWifiSignal();
    updateAdditionalInfo();

    lastUpdate = currentTime;
  }
}
