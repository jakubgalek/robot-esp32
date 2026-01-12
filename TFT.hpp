#include <TFT_eSPI.h>
#include <SPI.h>
#include <Bonezegei_Utility.h>
#include <Bonezegei_XPT2046v2.h>

#define TOUCH_CS 26
#define TOUCH_IRQ 27

TFT_eSPI tft = TFT_eSPI();

Bonezegei_XPT2046v2 touch(TOUCH_CS);
Bonezegei_Input* input = &touch;

// Vista color scheme
#define VISTA_BLUE 0x3D1F
#define VISTA_LIGHT_BLUE 0x7E3F
#define VISTA_DARK_BLUE 0x1A2F
#define VISTA_GLASS 0x4A49
#define VISTA_WHITE 0xFFFF
#define VISTA_GREY 0xA534
#define VISTA_DARK_GREY 0x5ACB
#define VISTA_GREEN 0x07E0
#define VISTA_RED 0xF800
#define VISTA_ORANGE 0xFD20
#define VISTA_YELLOW 0xFFE0

// Screen states
enum ScreenState {
  SCREEN_MAIN,
  SCREEN_SETTINGS
};

ScreenState currentScreen = SCREEN_MAIN;

// Force update flag for returning from settings
bool forceDisplayUpdate = false;

// Touch calibration values (adjust these based on your display)
#define TOUCH_MIN_X 300
#define TOUCH_MAX_X 3800
#define TOUCH_MIN_Y 400
#define TOUCH_MAX_Y 3800

void drawGlassPanel(int x, int y, int w, int h, uint16_t color) {
  tft.fillRect(x, y, w, h, color);
  tft.drawFastHLine(x, y, w, VISTA_WHITE);
  tft.drawFastHLine(x, y+1, w, VISTA_LIGHT_BLUE);
  tft.drawFastHLine(x, y+h-1, w, VISTA_DARK_BLUE);
  tft.drawFastHLine(x, y+h-2, w, VISTA_DARK_GREY);
  tft.drawFastVLine(x, y, h, VISTA_LIGHT_BLUE);
  tft.drawFastVLine(x+w-1, y, h, VISTA_DARK_GREY);
}

void drawVistaButton(int x, int y, int w, int h) {
  tft.fillRoundRect(x, y, w, h, 4, VISTA_GLASS);
  tft.drawRoundRect(x, y, w, h, 4, VISTA_LIGHT_BLUE);
  tft.drawRoundRect(x+1, y+1, w-2, h-2, 3, VISTA_WHITE);
}

void drawSettingsIcon(int x, int y, uint16_t color) {
  int centerX = x + 13;
  int centerY = y + 11;
  
  for (int i = 0; i < 8; i++) {
    float angle = i * 45 * PI / 180;
    int x1 = centerX + cos(angle) * 8;
    int y1 = centerY + sin(angle) * 8;
    int x2 = centerX + cos(angle) * 11;
    int y2 = centerY + sin(angle) * 11;
    tft.drawLine(x1, y1, x2, y2, color);
    tft.drawLine(x1+1, y1, x2+1, y2, color);
  }
  
  tft.drawCircle(centerX, centerY, 7, color);
  tft.drawCircle(centerX, centerY, 8, color);
  tft.fillCircle(centerX, centerY, 4, VISTA_BLUE);
  tft.drawCircle(centerX, centerY, 4, color);
}

void drawBackIcon(int x, int y, uint16_t color) {
  // Draw back arrow
  int centerY = y + 13;
  
  // Arrow shaft
  tft.drawFastHLine(x + 5, centerY, 20, color);
  tft.drawFastHLine(x + 5, centerY + 1, 20, color);
  
  // Arrow head
  for (int i = 0; i < 6; i++) {
    tft.drawFastVLine(x + 5 + i, centerY - i, i * 2 + 2, color);
  }
}

void drawBatteryIcon(int x, int y, float voltage, bool isLow) {
  uint16_t batteryColor = isLow ? VISTA_RED : VISTA_GREEN;
  
  tft.drawRect(x, y + 2, 20, 10, batteryColor);
  tft.fillRect(x + 20, y + 4, 2, 6, batteryColor);
  
  int fillWidth = 16;
  if (!isLow) {
    if (voltage >= 7.0) fillWidth = 16;
    else if (voltage >= 6.0) fillWidth = 12;
    else if (voltage >= 5.5) fillWidth = 8;
    else fillWidth = 4;
  } else {
    fillWidth = 4;
  }
  
  tft.fillRect(x + 2, y + 4, fillWidth, 6, batteryColor);
  
  if (isLow) {
    tft.fillRect(x + 9, y + 5, 2, 3, VISTA_WHITE);
    tft.fillRect(x + 9, y + 9, 2, 1, VISTA_WHITE);
  }
}

void drawMainScreen() {
  tft.fillScreen(VISTA_BLUE);
  
  // Vista gradient background
  for (int i = 0; i < 320; i++) {
    uint16_t color = 0x0800 + (i / 20);
    tft.drawFastHLine(0, i, 240, color);
  }
  
  // Title bar
  drawGlassPanel(0, 0, 240, 30, VISTA_BLUE);
  
  // Settings button
  drawVistaButton(196, 2, 38, 26);
  drawSettingsIcon(200, 3, VISTA_WHITE);
  
  // ESP32 Voltage Panel
  drawGlassPanel(10, 45, 220, 70, VISTA_GLASS);
  tft.setTextSize(1);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(20, 52);
  tft.print("BATERIE LOGIKA");
  
  // Motors Voltage Panel
  drawGlassPanel(10, 125, 220, 70, VISTA_GLASS);
  tft.setCursor(20, 132);
  tft.print("BATERIE SILNIKI");
  
  // WiFi Signal Panel
  drawGlassPanel(10, 205, 220, 70, VISTA_GLASS);
  tft.setCursor(20, 212);
  tft.print("SYGNAL WiFi");
  
  // Bottom status bar
  drawGlassPanel(0, 285, 240, 35, VISTA_DARK_BLUE);
}

void drawSettingsScreen() {
  tft.fillScreen(VISTA_BLUE);
  
  // Vista gradient background
  for (int i = 0; i < 320; i++) {
    uint16_t color = 0x0800 + (i / 20);
    tft.drawFastHLine(0, i, 240, color);
  }
  
  // Title bar
  drawGlassPanel(0, 0, 240, 30, VISTA_BLUE);
  tft.setTextSize(2);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(50, 7);
  tft.print("USTAWIENIA");
  
  // Back button
  drawVistaButton(5, 2, 38, 26);
  drawBackIcon(9, 3, VISTA_WHITE);
  
  // Settings panels
  drawGlassPanel(10, 45, 220, 60, VISTA_GLASS);
  tft.setTextSize(1);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(20, 52);
  tft.print("Jasnosc ekranu");
  
  // Brightness bar
  tft.fillRect(20, 75, 200, 20, VISTA_DARK_GREY);
  tft.fillRect(20, 75, 150, 20, VISTA_LIGHT_BLUE);
  
  drawGlassPanel(10, 115, 220, 60, VISTA_GLASS);
  tft.setCursor(20, 122);
  tft.print("Auto-wylaczenie");
  tft.setTextSize(2);
  tft.setTextColor(VISTA_LIGHT_BLUE);
  tft.setCursor(70, 145);
  tft.print("5 min");
  
  drawGlassPanel(10, 185, 220, 60, VISTA_GLASS);
  tft.setTextSize(1);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(20, 192);
  tft.print("WiFi Power Mode");
  tft.setTextSize(2);
  tft.setTextColor(VISTA_GREEN);
  tft.setCursor(60, 215);
  tft.print("NORMAL");
  
  // Info panel
  drawGlassPanel(10, 255, 220, 60, VISTA_GLASS);
  tft.setTextSize(1);
  tft.setTextColor(VISTA_GREY);
  tft.setCursor(20, 265);
  tft.print("Firmware: v1.0.3");
  tft.setCursor(20, 280);
  tft.print("Uptime: ");
  tft.print(millis() / 1000);
  tft.print("s");
  tft.setCursor(20, 295);
  tft.print("Free Heap: ");
  tft.print(ESP.getFreeHeap() / 1024);
  tft.print("KB");
}

void TFTsetup() {
  // Inicjalizacja TFT_eSPI
  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);
  
  // Upewnij siÄ™ Å¼e dotyk nie blokuje SPI
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);

  // Start dotyku
  input->begin();
  input->setRotation(1);

  drawMainScreen();
}

// Map touch coordinates to screen coordinates
bool getTouchCoordinates(int &x, int &y) {
    if (input->getPress()) {
        x = input->point.x;
        y = input->point.y;
        return true;
    }
    return false;
}

// Forward declarations for reset functions
void resetVoltageCache();
void resetWifiCache();
void resetAdditionalInfoCache();

void handleTouch() {
  static bool lastTouchState = false;
  static unsigned long lastTouchTime = 0;
  const unsigned long debounceDelay = 200;
  
  int touchX, touchY;
  bool touched = getTouchCoordinates(touchX, touchY);
  
  if (touched && !lastTouchState && (millis() - lastTouchTime > debounceDelay)) {
    lastTouchTime = millis();
    
    if (currentScreen == SCREEN_MAIN) {
      // Settings button area - VERY LARGE (entire right corner)
      // Visual button: (196, 2, 38, 26)
      // Touch area: entire right side of title bar + more
      if (touchX >= 250 && touchX <= 360 && touchY >= 180 && touchY <= 260) {
        currentScreen = SCREEN_SETTINGS;
        drawSettingsScreen();
      }
    } else if (currentScreen == SCREEN_SETTINGS) {
      // Back button area - VERY LARGE (entire left corner)
      // Visual button: (5, 2, 38, 26)
      // Touch area: entire left side of title bar + more
      if (touchX >= 260 && touchX <= 340 && touchY >= 0 && touchY <= 60) {
        currentScreen = SCREEN_MAIN;
        drawMainScreen();
        // Force immediate update of all values
        forceDisplayUpdate = true;
      }
    }
  }
  
  lastTouchState = touched;
}

void updateVoltageValues(float voltageMotors, float voltageESP) {
  if (currentScreen != SCREEN_MAIN) return;
  
  static float lastVoltageMotors = -1;
  static float lastVoltageESP = -1;

  if (voltageESP != lastVoltageESP || forceDisplayUpdate) {
    tft.fillRect(20, 70, 200, 35, VISTA_GLASS);
    
    tft.setTextSize(4);
    bool isLowESP = voltageESP < 5.0;
    tft.setTextColor(isLowESP ? VISTA_RED : VISTA_GREEN);
    tft.setCursor(30, 75);
    tft.print(voltageESP, 2);
    
    tft.setTextSize(3);
    tft.setTextColor(VISTA_WHITE);
    tft.print(" V");
    
    drawBatteryIcon(185, 80, voltageESP, isLowESP);
    
    lastVoltageESP = voltageESP;
  }

  if (voltageMotors != lastVoltageMotors || forceDisplayUpdate) {
    tft.fillRect(20, 150, 200, 35, VISTA_GLASS);
    
    tft.setTextSize(4);
    bool isLowMotors = voltageMotors < 5.0;
    tft.setTextColor(isLowMotors ? VISTA_RED : VISTA_GREEN);
    tft.setCursor(30, 155);
    tft.print(voltageMotors, 2);
    
    tft.setTextSize(3);
    tft.setTextColor(VISTA_WHITE);
    tft.print(" V");
    
    drawBatteryIcon(185, 160, voltageMotors, isLowMotors);
    
    lastVoltageMotors = voltageMotors;
  }
}

void resetVoltageCache() {
  // Not needed anymore - using forceDisplayUpdate flag
}

void updateWifiSignal() {
  if (currentScreen != SCREEN_MAIN) return;
  
  static int lastSignalStrength = -999;
  static IPAddress lastIP = IPAddress(0, 0, 0, 0);
  
  int signalStrength = WiFi.RSSI();
  IPAddress currentIP = WiFi.localIP();

  // Update IP address in title bar if changed
  if (currentIP != lastIP || forceDisplayUpdate) {
    // Clear title bar IP area
    tft.fillRect(10, 5, 180, 20, VISTA_BLUE);
    
    tft.setTextSize(2);
    tft.setTextColor(VISTA_WHITE);
    tft.setCursor(10, 7);
    
    if (currentIP[0] == 0) {
      // Not connected yet
      tft.print("Connecting...");
    } else {
      tft.print("IP:");
      tft.setTextColor(0x07FF);
      tft.print(currentIP);
    }
    
    lastIP = currentIP;
  }

  // Update signal strength
  if (signalStrength != lastSignalStrength || forceDisplayUpdate) {
    tft.fillRect(20, 230, 200, 35, VISTA_GLASS);
    
    tft.setTextSize(3);
    tft.setTextColor(VISTA_LIGHT_BLUE);
    tft.setCursor(30, 235);
    tft.print(signalStrength);
    tft.setTextSize(2);
    tft.print(" dBm");
    
    int bars = map(signalStrength, -90, -30, 1, 5);
    bars = constrain(bars, 0, 5);
    
    for (int i = 0; i < 5; i++) {
      int barHeight = 10 + (i * 4);
      uint16_t barColor = (i < bars) ? VISTA_GREEN : VISTA_DARK_GREY;
      tft.fillRect(180 + (i * 10), 265 - barHeight, 6, barHeight, barColor);
    }
    
    lastSignalStrength = signalStrength;
  }
}

void resetWifiCache() {
  // Not needed anymore - using forceDisplayUpdate flag
}

void updateAdditionalInfo() {
  if (currentScreen != SCREEN_MAIN) return;
  
  static int lastHour = -1;
  static int lastMinute = -1;
  static float lastTemperature = -999.0;
  static int lastAltitude = -999;

  int actualhour = dt.hour;
  int actualminute = dt.minute;
  float temperature = getFormattedTemperature();
  int altitude = getFormattedAltitude();

  if (actualhour != lastHour || actualminute != lastMinute || forceDisplayUpdate) {
    tft.fillRect(165, 292, 70, 25, VISTA_DARK_BLUE);
    tft.setTextSize(2);
    tft.setTextColor(VISTA_WHITE);
    tft.setCursor(170, 296);
    
    char timeStr[6];
    sprintf(timeStr, "%02d:%02d", actualhour, actualminute);
    tft.print(timeStr);
    
    lastHour = actualhour;
    lastMinute = actualminute;
  }

  if (temperature != lastTemperature || forceDisplayUpdate) {
    tft.fillRect(5, 292, 70, 25, VISTA_DARK_BLUE);
    tft.setTextSize(2);
    tft.setTextColor(VISTA_YELLOW);
    tft.setCursor(10, 296);
    tft.print(temperature, 1);
    tft.print(" C");
    
    lastTemperature = temperature;
  }

  if (altitude != lastAltitude || forceDisplayUpdate) {
    tft.fillRect(85, 292, 75, 25, VISTA_DARK_BLUE);
    tft.setTextSize(2);
    tft.setTextColor(VISTA_LIGHT_BLUE);
    tft.setCursor(90, 296);
    tft.print(altitude);
    tft.print("m");
    
    lastAltitude = altitude;
  }
}

void resetAdditionalInfoCache() {
  // Not needed anymore - using forceDisplayUpdate flag
}

void debugTouch() {
  if (input->getPress()) {
    Serial.print("RAW X = ");
    Serial.print(input->point.x);
    Serial.print("   RAW Y = ");
    Serial.println(input->point.y);
  }
}

void refreshTFT() {
  static unsigned long lastUpdate = 0;
  const unsigned long updateInterval = 1000;
  unsigned long currentTime = millis();

  // Always handle touch
  handleTouch();
  debugTouch();

  if (currentTime - lastUpdate >= updateInterval) {
    if (currentScreen == SCREEN_MAIN) {
      float voltageMotors = readVoltageMotors();
      float voltageESP = readVoltageESP32();

      updateVoltageValues(voltageMotors, voltageESP);
      updateWifiSignal();
      updateAdditionalInfo();
      
      // Reset force update flag after first refresh
      if (forceDisplayUpdate) {
        forceDisplayUpdate = false;
      }
    }

    lastUpdate = currentTime;
  }
}