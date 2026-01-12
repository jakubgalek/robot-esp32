#include <TFT_eSPI.h>
#include <SPI.h>
#include <Bonezegei_Utility.h>
#include <Bonezegei_XPT2046v2.h>
#include <ESP32Servo.h>
#include <WiFi.h>

#define TOUCH_CS 26
#define TOUCH_IRQ 27

TFT_eSPI tft = TFT_eSPI();

Bonezegei_XPT2046v2 touch(TOUCH_CS);
Bonezegei_Input* input = &touch;

// External declarations from other files
extern Servo servoMotor;
extern const char* ssid;
extern const char* password;
extern const char* ssid_ap;
extern const char* password_ap;
extern volatile long indents;
extern const double distancePerIndent;
extern double robotX;
extern double robotY;
extern double robotAngle;
extern volatile int motorSpeed;
extern int pwmChannelSpeed;
extern String direction;
extern int measurement0, measurement1, measurement2, measurement3, measurement4, measurement5, measurement6, measurement7;

// Forward declarations for functions from other files
void forward();
void backward();
void turn_left();
void turn_right();
void stop_driving();
void read_obstacle_sensors();
float readVoltageMotors();
float readVoltageESP32();
float getFormattedTemperature();
int getFormattedAltitude();
int getFormattedPressure();
bool getTouchCoordinates(int &x, int &y);

// External time variable
extern RTCDateTime dt;

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
#define VISTA_CYAN 0x07FF

// Metro/Windows 8 colors for tiles
#define METRO_BLUE 0x1E9F
#define METRO_GREEN 0x5D20
#define METRO_ORANGE 0xFB80
#define METRO_PURPLE 0x8811
#define METRO_RED 0xF800

// Screen states
enum ScreenState {
  SCREEN_MAIN,
  SCREEN_MENU,
  SCREEN_SETTINGS,
  SCREEN_SENSORS,
  SCREEN_DIAGNOSTICS
};

ScreenState currentScreen = SCREEN_MAIN;

// Settings variables
int screenBrightness = 128; // 0-255
bool wifiAPMode = false; // false = Router mode, true = AP mode

// Diagnostic test states
bool testMotor1 = false;
bool testMotor2 = false;
bool testMotor3 = false;
bool testMotor4 = false;
int servoTestAngle = 90;

// Force update flag
bool forceDisplayUpdate = false;

// ==================== DRAWING UTILITIES ====================

void drawGlassPanel(int x, int y, int w, int h, uint16_t color) {
  tft.fillRect(x, y, w, h, color);
  tft.drawFastHLine(x, y, w, VISTA_WHITE);
  tft.drawFastHLine(x, y+1, w, VISTA_LIGHT_BLUE);
  tft.drawFastHLine(x, y+h-1, w, VISTA_DARK_BLUE);
  tft.drawFastHLine(x, y+h-2, w, VISTA_DARK_GREY);
  tft.drawFastVLine(x, y, h, VISTA_LIGHT_BLUE);
  tft.drawFastVLine(x+w-1, y, h, VISTA_DARK_GREY);
}

void drawVistaButton(int x, int y, int w, int h, bool pressed = false) {
  uint16_t bgColor = pressed ? VISTA_DARK_BLUE : VISTA_GLASS;
  tft.fillRoundRect(x, y, w, h, 4, bgColor);
  tft.drawRoundRect(x, y, w, h, 4, VISTA_LIGHT_BLUE);
  if (!pressed) {
    tft.drawRoundRect(x+1, y+1, w-2, h-2, 3, VISTA_WHITE);
  }
}

void drawMenuIcon(int x, int y, uint16_t color) {
  // Windows 8 style menu icon (3x3 grid)
  int size = 5;
  int spacing = 8;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      tft.fillRect(x + col * spacing, y + row * spacing, size, size, color);
    }
  }
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
  int centerY = y + 13;
  tft.drawFastHLine(x + 5, centerY, 20, color);
  tft.drawFastHLine(x + 5, centerY + 1, 20, color);
  for (int i = 0; i < 6; i++) {
    tft.drawFastVLine(x + 5 + i, centerY - i, i * 2 + 2, color);
  }
}

void drawSensorIcon(int x, int y, uint16_t color) {
  tft.drawCircle(x + 13, y + 13, 10, color);
  tft.drawCircle(x + 13, y + 13, 7, color);
  tft.drawCircle(x + 13, y + 13, 4, color);
  tft.drawLine(x + 13, y + 13, x + 20, y + 8, color);
  tft.drawLine(x + 13, y + 13, x + 21, y + 9, color);
}

void drawDiagnosticIcon(int x, int y, uint16_t color) {
  tft.fillRect(x + 8, y + 5, 4, 15, color);
  tft.fillCircle(x + 10, y + 7, 4, color);
  tft.fillCircle(x + 10, y + 18, 3, color);
  tft.drawRect(x + 15, y + 8, 8, 4, color);
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

void drawMetroTile(int x, int y, int w, int h, uint16_t color, const char* label) {
  // Draw tile background
  tft.fillRect(x, y, w, h, color);
  
  // Draw subtle border
  tft.drawRect(x, y, w, h, VISTA_WHITE);
  
  // Draw label
  tft.setTextSize(2);
  tft.setTextColor(VISTA_WHITE);
  
  // Center text
  int textWidth = strlen(label) * 12; // Approximate width for size 2
  int textX = x + (w - textWidth) / 2;
  int textY = y + h - 20;
  
  tft.setCursor(textX, textY);
  tft.print(label);
}

// ==================== MAIN SCREEN (Original) ====================

void drawMainScreen() {
  tft.fillScreen(VISTA_BLUE);
  
  // Vista gradient background
  for (int i = 0; i < 320; i++) {
    uint16_t color = 0x0800 + (i / 20);
    tft.drawFastHLine(0, i, 240, color);
  }
  
  // Title bar
  drawGlassPanel(0, 0, 240, 30, VISTA_BLUE);
  
  // Menu button (left side)
  drawVistaButton(5, 2, 38, 26);
  drawMenuIcon(12, 7, VISTA_WHITE);
  
  // ESP32 Voltage Panel
  drawGlassPanel(10, 45, 220, 70, VISTA_GLASS);
  tft.setTextSize(1);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(20, 52);
  tft.print("NAPIECIE ESP32");
  
  // Motors Voltage Panel
  drawGlassPanel(10, 125, 220, 70, VISTA_GLASS);
  tft.setCursor(20, 132);
  tft.print("NAPIECIE SILNIKOW");
  
  // WiFi Signal Panel
  drawGlassPanel(10, 205, 220, 70, VISTA_GLASS);
  tft.setCursor(20, 212);
  tft.print("SYGNAL WiFi");
  
  // Bottom status bar
  drawGlassPanel(0, 285, 240, 35, VISTA_DARK_BLUE);
}

// ==================== MENU SCREEN (Windows 8 Style) ====================

void drawMenuScreen() {
  tft.fillScreen(TFT_BLACK);
  
  // Title bar
  drawGlassPanel(0, 0, 240, 30, VISTA_DARK_BLUE);
  tft.setTextSize(2);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(80, 7);
  tft.print("MENU");
  
  // Back button
  drawVistaButton(5, 2, 38, 26);
  drawBackIcon(9, 3, VISTA_WHITE);
  
  // Metro tiles in 2x2 grid
  // Settings tile (top left)
  drawMetroTile(10, 45, 105, 95, METRO_BLUE, "Ustawienia");
  drawSettingsIcon(45, 60, VISTA_WHITE);
  
  // Sensors tile (top right)
  drawMetroTile(125, 45, 105, 95, METRO_GREEN, "Czujniki");
  drawSensorIcon(160, 60, VISTA_WHITE);
  
  // Diagnostics tile (bottom left)
  drawMetroTile(10, 150, 105, 95, METRO_ORANGE, "Diagnostyka");
  drawDiagnosticIcon(45, 165, VISTA_WHITE);
  
  // Info tile (bottom right)
  drawMetroTile(125, 150, 105, 95, METRO_PURPLE, "Info");
  tft.setTextSize(3);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(162, 175);
  tft.print("i");
  
  // Bottom info
  tft.setTextSize(1);
  tft.setTextColor(VISTA_GREY);
  tft.setCursor(10, 260);
  tft.print("FW: v1.0.3 | Heap: ");
  tft.print(ESP.getFreeHeap() / 1024);
  tft.print("KB");
  tft.setCursor(10, 275);
  tft.print("Pos: X:");
  tft.print((int)robotX);
  tft.print(" Y:");
  tft.print((int)robotY);
  tft.print(" A:");
  tft.print((int)robotAngle);
  tft.print("°");
}

// ==================== SETTINGS SCREEN ====================

void drawSettingsScreen() {
  tft.fillScreen(VISTA_BLUE);
  
  for (int i = 0; i < 320; i++) {
    uint16_t color = 0x0800 + (i / 20);
    tft.drawFastHLine(0, i, 240, color);
  }
  
  drawGlassPanel(0, 0, 240, 30, VISTA_BLUE);
  tft.setTextSize(2);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(50, 7);
  tft.print("USTAWIENIA");
  
  // Back button
  drawVistaButton(5, 2, 38, 26);
  drawBackIcon(9, 3, VISTA_WHITE);
  
  // Brightness panel
  drawGlassPanel(10, 40, 220, 65, VISTA_GLASS);
  tft.setTextSize(1);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(20, 47);
  tft.print("Jasnosc ekranu: ");
  tft.print((screenBrightness * 100) / 255);
  tft.print("%");
  
  // Brightness slider background
  tft.fillRect(20, 65, 200, 15, VISTA_DARK_GREY);
  // Brightness slider fill
  int fillWidth = (screenBrightness * 200) / 255;
  tft.fillRect(20, 65, fillWidth, 15, VISTA_LIGHT_BLUE);
  
  // Brightness buttons
  drawVistaButton(20, 85, 30, 15);
  tft.setTextSize(1);
  tft.setCursor(28, 88);
  tft.print("-");
  
  drawVistaButton(190, 85, 30, 15);
  tft.setCursor(198, 88);
  tft.print("+");
  
  // WiFi Mode panel
  drawGlassPanel(10, 115, 220, 80, VISTA_GLASS);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(20, 122);
  tft.print("Tryb WiFi: ");
  tft.setTextColor(wifiAPMode ? VISTA_ORANGE : VISTA_GREEN);
  tft.print(wifiAPMode ? "AP" : "ROUTER");
  
  tft.setTextSize(1);
  tft.setTextColor(VISTA_GREY);
  tft.setCursor(20, 140);
  if (wifiAPMode) {
    tft.print("SSID: ");
    tft.print(ssid_ap);
    tft.setCursor(20, 153);
    tft.print("IP: ");
    tft.print(WiFi.softAPIP());
  } else {
    tft.print("Polaczony z: ");
    tft.print(ssid);
    tft.setCursor(20, 153);
    tft.print("IP: ");
    tft.print(WiFi.localIP());
  }
  
  // Toggle button
  drawVistaButton(20, 168, 90, 22, false);
  tft.setTextSize(1);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(25, 175);
  tft.print("PRZELACZ");
  
  // WiFi Power Mode panel
  drawGlassPanel(10, 205, 220, 45, VISTA_GLASS);
  tft.setTextSize(1);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(20, 212);
  tft.print("WiFi Power Mode");
  tft.setTextSize(2);
  tft.setTextColor(VISTA_GREEN);
  tft.setCursor(60, 230);
  tft.print("NORMAL");
  
  // Info at bottom
  tft.setTextSize(1);
  tft.setTextColor(VISTA_GREY);
  tft.setCursor(10, 265);
  tft.print("Uptime: ");
  tft.print(millis() / 1000);
  tft.print("s | Heap: ");
  tft.print(ESP.getFreeHeap() / 1024);
  tft.print("KB");
}

// ==================== SENSORS SCREEN ====================

// Cache dla sensor screen - żeby nie migotało
static int lastSensorUpdate = 0;
static int cachedMeasurements[8] = {0};
static float cachedTemp = 0;
static int cachedAlt = 0;
static int cachedPress = 0;

void drawRobotWithSensors(int centerX, int centerY) {
  // Robot body
  tft.fillRoundRect(centerX - 20, centerY - 15, 40, 30, 5, VISTA_GREY);
  
  // Wheels
  tft.fillRect(centerX - 25, centerY - 20, 8, 10, VISTA_DARK_GREY);
  tft.fillRect(centerX + 17, centerY - 20, 8, 10, VISTA_DARK_GREY);
  tft.fillRect(centerX - 25, centerY + 10, 8, 10, VISTA_DARK_GREY);
  tft.fillRect(centerX + 17, centerY + 10, 8, 10, VISTA_DARK_GREY);
  
  // Front indicator
  tft.fillRect(centerX + 15, centerY - 3, 8, 6, VISTA_YELLOW);
  
  // Sensor lines using cached values
  int dist4 = cachedMeasurements[4];
  uint16_t color4 = (dist4 < 35) ? VISTA_RED : VISTA_GREEN;
  tft.drawLine(centerX, centerY, centerX + 35, centerY, color4);
  tft.fillCircle(centerX + 35, centerY, 3, color4);
  
  int dist3 = cachedMeasurements[3];
  uint16_t color3 = (dist3 < 4) ? VISTA_RED : VISTA_GREEN;
  tft.drawLine(centerX, centerY, centerX + 25, centerY - 25, color3);
  tft.fillCircle(centerX + 25, centerY - 25, 3, color3);
  
  int dist5 = cachedMeasurements[5];
  uint16_t color5 = (dist5 < 4) ? VISTA_RED : VISTA_GREEN;
  tft.drawLine(centerX, centerY, centerX + 25, centerY + 25, color5);
  tft.fillCircle(centerX + 25, centerY + 25, 3, color5);
  
  int dist6 = cachedMeasurements[6];
  uint16_t color6 = (dist6 < 7) ? VISTA_RED : VISTA_GREEN;
  tft.drawLine(centerX, centerY, centerX - 30, centerY, color6);
  tft.fillCircle(centerX - 30, centerY, 3, color6);
  
  int dist2 = cachedMeasurements[2];
  uint16_t color2 = (dist2 < 10) ? VISTA_RED : VISTA_GREEN;
  tft.drawLine(centerX, centerY, centerX - 20, centerY - 25, color2);
  tft.fillCircle(centerX - 20, centerY - 25, 3, color2);
  
  int dist7 = cachedMeasurements[7];
  uint16_t color7 = (dist7 < 10) ? VISTA_RED : VISTA_GREEN;
  tft.drawLine(centerX, centerY, centerX - 20, centerY + 25, color7);
  tft.fillCircle(centerX - 20, centerY + 25, 3, color7);
}

void updateSensorReadings() {
  read_obstacle_sensors();
  cachedMeasurements[2] = measurement2;
  cachedMeasurements[3] = measurement3;
  cachedMeasurements[4] = measurement4;
  cachedMeasurements[5] = measurement5;
  cachedMeasurements[6] = measurement6;
  cachedMeasurements[7] = measurement7;
  cachedTemp = getFormattedTemperature();
  cachedAlt = getFormattedAltitude();
  cachedPress = getFormattedPressure();
}

void drawSensorsScreen() {
  // Only full redraw on first load
  static bool firstDraw = true;
  
  if (firstDraw || forceDisplayUpdate) {
    tft.fillScreen(VISTA_BLUE);
    
    for (int i = 0; i < 320; i++) {
      uint16_t color = 0x0800 + (i / 20);
      tft.drawFastHLine(0, i, 240, color);
    }
    
    drawGlassPanel(0, 0, 240, 30, VISTA_BLUE);
    tft.setTextSize(2);
    tft.setTextColor(VISTA_WHITE);
    tft.setCursor(60, 7);
    tft.print("CZUJNIKI");
    
    // Back button
    drawVistaButton(5, 2, 38, 26);
    drawBackIcon(9, 3, VISTA_WHITE);
    
    // Robot visualization panel
    drawGlassPanel(10, 40, 220, 120, VISTA_GLASS);
    
    // Sensor readings panel
    drawGlassPanel(10, 170, 220, 110, VISTA_GLASS);
    
    firstDraw = false;
  }
  
  // Update robot visualization
  tft.fillRect(15, 45, 210, 110, VISTA_GLASS);
  drawRobotWithSensors(120, 100);
  
  // Update sensor readings
  tft.fillRect(15, 175, 210, 100, VISTA_GLASS);
  tft.setTextSize(1);
  tft.setTextColor(VISTA_WHITE);
  
  int yPos = 177;
  tft.setCursor(15, yPos);
  tft.print("FC:");
  tft.setTextColor(cachedMeasurements[4] < 35 ? VISTA_RED : VISTA_GREEN);
  tft.print(cachedMeasurements[4]);
  tft.setTextColor(VISTA_WHITE);
  tft.print("cm FL:");
  tft.setTextColor(cachedMeasurements[3] < 4 ? VISTA_RED : VISTA_GREEN);
  tft.print(cachedMeasurements[3]);
  tft.setTextColor(VISTA_WHITE);
  tft.print("cm FR:");
  tft.setTextColor(cachedMeasurements[5] < 4 ? VISTA_RED : VISTA_GREEN);
  tft.print(cachedMeasurements[5]);
  tft.setTextColor(VISTA_WHITE);
  tft.print("cm");
  
  yPos += 12;
  tft.setCursor(15, yPos);
  tft.print("BACK:");
  tft.setTextColor(cachedMeasurements[6] < 7 ? VISTA_RED : VISTA_GREEN);
  tft.print(cachedMeasurements[6]);
  tft.setTextColor(VISTA_WHITE);
  tft.print("cm LF:");
  tft.setTextColor(cachedMeasurements[2] < 10 ? VISTA_RED : VISTA_GREEN);
  tft.print(cachedMeasurements[2]);
  tft.setTextColor(VISTA_WHITE);
  tft.print("cm LB:");
  tft.setTextColor(cachedMeasurements[7] < 10 ? VISTA_RED : VISTA_GREEN);
  tft.print(cachedMeasurements[7]);
  tft.setTextColor(VISTA_WHITE);
  tft.print("cm");
  
  yPos += 18;
  tft.setCursor(15, yPos);
  tft.print("Temp:");
  tft.setTextColor(VISTA_YELLOW);
  tft.print(cachedTemp, 1);
  tft.setTextColor(VISTA_WHITE);
  tft.print("C Alt:");
  tft.setTextColor(VISTA_CYAN);
  tft.print(cachedAlt);
  tft.setTextColor(VISTA_WHITE);
  tft.print("m P:");
  tft.setTextColor(VISTA_LIGHT_BLUE);
  tft.print(cachedPress);
  tft.setTextColor(VISTA_WHITE);
  tft.print("hPa");
  
  yPos += 12;
  tft.setCursor(15, yPos);
  tft.print("Kierunek: ");
  tft.setTextColor(VISTA_ORANGE);
  tft.print(direction);
  
  yPos += 12;
  tft.setCursor(15, yPos);
  tft.setTextColor(VISTA_WHITE);
  tft.print("Kat:");
  tft.setTextColor(VISTA_LIGHT_BLUE);
  tft.print(robotAngle, 1);
  tft.setTextColor(VISTA_WHITE);
  tft.print("° Speed:");
  tft.setTextColor(VISTA_GREEN);
  tft.print(motorSpeed);
}

// ==================== DIAGNOSTICS SCREEN ====================

void drawDiagnosticsScreen() {
  tft.fillScreen(VISTA_BLUE);
  
  for (int i = 0; i < 320; i++) {
    uint16_t color = 0x0800 + (i / 20);
    tft.drawFastHLine(0, i, 240, color);
  }
  
  drawGlassPanel(0, 0, 240, 30, VISTA_BLUE);
  tft.setTextSize(2);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(30, 7);
  tft.print("DIAGNOSTYKA");
  
  // Back button
  drawVistaButton(5, 2, 38, 26);
  drawBackIcon(9, 3, VISTA_WHITE);
  
  // Motors test section
  drawGlassPanel(10, 40, 220, 100, VISTA_GLASS);
  tft.setTextSize(1);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(15, 47);
  tft.print("TEST SILNIKOW");
  
  // Motor buttons - larger touch areas
  drawVistaButton(15, 60, 50, 30, testMotor1);
  tft.setTextSize(1);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(22, 70);
  tft.print("PRZOD");
  
  drawVistaButton(15, 100, 50, 30, testMotor2);
  tft.setCursor(25, 110);
  tft.print("TYL");
  
  drawVistaButton(75, 60, 50, 30, testMotor3);
  tft.setCursor(82, 70);
  tft.print("LEWO");
  
  drawVistaButton(75, 100, 50, 30, testMotor4);
  tft.setCursor(80, 110);
  tft.print("PRAWO");
  
  // Motor status
  tft.setTextSize(1);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(135, 65);
  tft.print("Status:");
  tft.setCursor(135, 77);
  if (testMotor1 || testMotor2 || testMotor3 || testMotor4) {
    tft.setTextColor(VISTA_GREEN);
    tft.print("AKTYWNY");
  } else {
    tft.setTextColor(VISTA_GREY);
    tft.print("STOP");
  }
  
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(135, 95);
  tft.print("V:");
  tft.setTextColor(VISTA_CYAN);
  tft.print(motorSpeed);
  
  // Servo test section
  drawGlassPanel(10, 150, 220, 75, VISTA_GLASS);
  tft.setTextSize(1);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(15, 157);
  tft.print("TEST SERWA - Kat: ");
  tft.setTextColor(VISTA_YELLOW);
  tft.print(servoTestAngle);
  tft.setTextColor(VISTA_WHITE);
  tft.print("°");
  
  // Servo angle buttons - larger
  drawVistaButton(15, 175, 40, 35);
  tft.setTextSize(3);
  tft.setCursor(27, 185);
  tft.print("-");
  
  drawVistaButton(65, 175, 35, 35);
  tft.setTextSize(2);
  tft.setCursor(73, 185);
  tft.print("0");
  
  drawVistaButton(110, 175, 35, 35);
  tft.setCursor(113, 185);
  tft.print("90");
  
  drawVistaButton(155, 175, 40, 35);
  tft.setCursor(157, 185);
  tft.print("180");
  
  drawVistaButton(205, 175, 25, 35);
  tft.setTextSize(3);
  tft.setCursor(210, 185);
  tft.print("+");
  
  // Encoder info
  drawGlassPanel(10, 235, 220, 45, VISTA_GLASS);
  tft.setTextSize(1);
  tft.setTextColor(VISTA_WHITE);
  tft.setCursor(15, 242);
  tft.print("Enkoder: ");
  tft.setTextColor(VISTA_LIGHT_BLUE);
  tft.print(indents);
  tft.setTextColor(VISTA_WHITE);
  tft.print(" imp");
  
  tft.setCursor(15, 257);
  tft.print("Dystans: ");
  tft.setTextColor(VISTA_GREEN);
  tft.print(indents * distancePerIndent, 1);
  tft.setTextColor(VISTA_WHITE);
  tft.print(" cm");
}

// ==================== TOUCH HANDLING ====================

void handleMainScreenTouch(int touchX, int touchY) {
  // Menu button area - VERY LARGE (entire left corner)
  if (touchX >= 260 && touchX <= 360 && touchY >= 180 && touchY <= 260) {
    currentScreen = SCREEN_MENU;
    drawMenuScreen();
    forceDisplayUpdate = true;
  }
}

void handleMenuScreenTouch(int touchX, int touchY) {
  // Back button - entire left top corner
  if (touchX >= 260 && touchX <= 360 && touchY >= 180 && touchY <= 260) {
    currentScreen = SCREEN_MAIN;
    drawMainScreen();
    forceDisplayUpdate = true;
    return;
  }
  
  // Settings tile (top left) - LARGE
  if (touchX >= 180 && touchX <= 360 && touchY >= 105 && touchY <= 230) {
    currentScreen = SCREEN_SETTINGS;
    drawSettingsScreen();
    forceDisplayUpdate = true;
  }
  
  // Sensors tile (top right) - LARGE
  if (touchX >= 0 && touchX <= 170 && touchY >= 105 && touchY <= 230) {
    currentScreen = SCREEN_SENSORS;
    updateSensorReadings(); // Load initial data
    drawSensorsScreen();
    forceDisplayUpdate = true;
  }
  
  // Diagnostics tile (bottom left) - LARGE
  if (touchX >= 180 && touchX <= 360 && touchY >= 0 && touchY <= 100) {
    currentScreen = SCREEN_DIAGNOSTICS;
    drawDiagnosticsScreen();
    forceDisplayUpdate = true;
  }
  
  // Info tile (bottom right) - LARGE
  if (touchX >= 0 && touchX <= 170 && touchY >= 0 && touchY <= 100) {
    // Could open info screen in future
    // For now just flash the tile
  }
}

void handleSettingsScreenTouch(int touchX, int touchY) {
  // Back button - entire left top corner
  if (touchX >= 260 && touchX <= 360 && touchY >= 180 && touchY <= 260) {
    currentScreen = SCREEN_MENU;
    drawMenuScreen();
    forceDisplayUpdate = true;
    return;
  }
  
  // Brightness decrease button - LARGE
  if (touchX >= 280 && touchX <= 360 && touchY >= 200 && touchY <= 250) {
    screenBrightness = max(0, screenBrightness - 25);
    ledcWrite(5, screenBrightness);
    drawSettingsScreen();
  }
  
  // Brightness increase button - LARGE
  if (touchX >= 50 && touchX <= 130 && touchY >= 200 && touchY <= 250) {
    screenBrightness = min(255, screenBrightness + 25);
    ledcWrite(5, screenBrightness);
    drawSettingsScreen();
  }
  
  // WiFi mode toggle button - LARGE
  if (touchX >= 280 && touchX <= 360 && touchY >= 85 && touchY <= 145) {
    wifiAPMode = !wifiAPMode;
    
    // Restart WiFi with new mode
    WiFi.disconnect();
    delay(100);
    
    if (wifiAPMode) {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(ssid_ap, password_ap);
      Serial.println("Switched to AP Mode");
      Serial.print("AP IP: ");
      Serial.println(WiFi.softAPIP());
    } else {
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      Serial.println("Switching to Router Mode...");
      
      int timeout = 0;
      while (WiFi.status() != WL_CONNECTED && timeout < 20) {
        delay(500);
        Serial.print(".");
        timeout++;
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to Router");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
      } else {
        Serial.println("\nFailed to connect to Router");
        wifiAPMode = true;
        WiFi.mode(WIFI_AP);
        WiFi.softAP(ssid_ap, password_ap);
      }
    }
    
    drawSettingsScreen();
  }
}

void handleSensorsScreenTouch(int touchX, int touchY) {
  // Back button - entire left top corner
  if (touchX >= 260 && touchX <= 360 && touchY >= 180 && touchY <= 260) {
    currentScreen = SCREEN_MENU;
    drawMenuScreen();
    forceDisplayUpdate = true;
  }
}

void handleDiagnosticsScreenTouch(int touchX, int touchY) {
  // Back button - entire left top corner
  if (touchX >= 260 && touchX <= 360 && touchY >= 180 && touchY <= 260) {
    currentScreen = SCREEN_MENU;
    drawMenuScreen();
    forceDisplayUpdate = true;
    return;
  }
  
  // Motor test buttons - LARGE AREAS
  // Forward button
  if (touchX >= 275 && touchX <= 360 && touchY >= 210 && touchY <= 260) {
    testMotor1 = !testMotor1;
    if (testMotor1) {
      testMotor2 = testMotor3 = testMotor4 = false;
      forward();
      ledcWrite(pwmChannelSpeed, motorSpeed);
    } else {
      stop_driving();
    }
    drawDiagnosticsScreen();
  }
  
  // Backward button
  if (touchX >= 275 && touchX <= 360 && touchY >= 150 && touchY <= 200) {
    testMotor2 = !testMotor2;
    if (testMotor2) {
      testMotor1 = testMotor3 = testMotor4 = false;
      backward();
      ledcWrite(pwmChannelSpeed, motorSpeed);
    } else {
      stop_driving();
    }
    drawDiagnosticsScreen();
  }
  
  // Left button
  if (touchX >= 200 && touchX <= 265 && touchY >= 210 && touchY <= 260) {
    testMotor3 = !testMotor3;
    if (testMotor3) {
      testMotor1 = testMotor2 = testMotor4 = false;
      turn_left();
      ledcWrite(pwmChannelSpeed, motorSpeed);
    } else {
      stop_driving();
    }
    drawDiagnosticsScreen();
  }
  
  // Right button
  if (touchX >= 200 && touchX <= 265 && touchY >= 150 && touchY <= 200) {
    testMotor4 = !testMotor4;
    if (testMotor4) {
      testMotor1 = testMotor2 = testMotor3 = false;
      turn_right();
      ledcWrite(pwmChannelSpeed, motorSpeed);
    } else {
      stop_driving();
    }
    drawDiagnosticsScreen();
  }
  
  // Servo test buttons - LARGE AREAS
  // Servo - button
  if (touchX >= 275 && touchX <= 360 && touchY >= 80 && touchY <= 135) {
    servoTestAngle = max(0, servoTestAngle - 15);
    servoMotor.write(servoTestAngle);
    drawDiagnosticsScreen();
  }
  
  // Servo + button
  if (touchX >= 30 && touchX <= 90 && touchY >= 80 && touchY <= 135) {
    servoTestAngle = min(180, servoTestAngle + 15);
    servoMotor.write(servoTestAngle);
    drawDiagnosticsScreen();
  }
  
  // Servo 0° button
  if (touchX >= 225 && touchX <= 280 && touchY >= 80 && touchY <= 135) {
    servoTestAngle = 0;
    servoMotor.write(servoTestAngle);
    drawDiagnosticsScreen();
  }
  
  // Servo 90° button
  if (touchX >= 165 && touchX <= 220 && touchY >= 80 && touchY <= 135) {
    servoTestAngle = 90;
    servoMotor.write(servoTestAngle);
    drawDiagnosticsScreen();
  }
  
  // Servo 180° button
  if (touchX >= 105 && touchX <= 160 && touchY >= 80 && touchY <= 135) {
    servoTestAngle = 180;
    servoMotor.write(servoTestAngle);
    drawDiagnosticsScreen();
  }
}

void handleTouch() {
  static bool lastTouchState = false;
  static unsigned long lastTouchTime = 0;
  const unsigned long debounceDelay = 300;
  
  int touchX, touchY;
  bool touched = getTouchCoordinates(touchX, touchY);
  
  if (touched && !lastTouchState && (millis() - lastTouchTime > debounceDelay)) {
    lastTouchTime = millis();
    
    switch (currentScreen) {
      case SCREEN_MAIN:
        handleMainScreenTouch(touchX, touchY);
        break;
      case SCREEN_MENU:
        handleMenuScreenTouch(touchX, touchY);
        break;
      case SCREEN_SETTINGS:
        handleSettingsScreenTouch(touchX, touchY);
        break;
      case SCREEN_SENSORS:
        handleSensorsScreenTouch(touchX, touchY);
        break;
      case SCREEN_DIAGNOSTICS:
        handleDiagnosticsScreenTouch(touchX, touchY);
        break;
    }
  }
  
  lastTouchState = touched;
}

// ==================== UPDATE FUNCTIONS ====================

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
    tft.setTextColor(isLowMotors ? VISTA_RED : VISTA_ORANGE);
    tft.setCursor(30, 155);
    tft.print(voltageMotors, 2);
    
    tft.setTextSize(3);
    tft.setTextColor(VISTA_WHITE);
    tft.print(" V");
    
    drawBatteryIcon(185, 160, voltageMotors, isLowMotors);
    
    lastVoltageMotors = voltageMotors;
  }
}

void updateWifiSignal() {
  if (currentScreen != SCREEN_MAIN) return;
  
  static int lastSignalStrength = -999;
  static IPAddress lastIP = IPAddress(0, 0, 0, 0);
  
  int signalStrength = WiFi.RSSI();
  IPAddress currentIP = wifiAPMode ? WiFi.softAPIP() : WiFi.localIP();

  // Update IP address in title bar if changed
  if (currentIP != lastIP || forceDisplayUpdate) {
    tft.fillRect(10, 5, 180, 20, VISTA_BLUE);
    
    tft.setTextSize(2);
    tft.setTextColor(VISTA_WHITE);
    tft.setCursor(10, 7);
    
    if (currentIP[0] == 0 && !wifiAPMode) {
      tft.print("Connecting...");
    } else {
      tft.print("IP:");
      tft.setTextColor(VISTA_CYAN);
      tft.print(currentIP);
    }
    
    lastIP = currentIP;
  }

  // Update signal strength
  if (signalStrength != lastSignalStrength || forceDisplayUpdate) {
    tft.fillRect(20, 230, 200, 35, VISTA_GLASS);
    
    if (wifiAPMode) {
      tft.setTextSize(2);
      tft.setTextColor(VISTA_ORANGE);
      tft.setCursor(30, 235);
      tft.print("AP MODE");
    } else {
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
    }
    
    lastSignalStrength = signalStrength;
  }
}

void updateMainScreenInfo() {
  if (currentScreen != SCREEN_MAIN) return;
  
  static int lastHour = -1;
  static int lastMinute = -1;
  static float lastTemperature = -999.0;

  int actualhour = dt.hour;
  int actualminute = dt.minute;
  float temperature = getFormattedTemperature();

  // Time at top right
  if (actualhour != lastHour || actualminute != lastMinute || forceDisplayUpdate) {
    tft.fillRect(180, 5, 55, 20, VISTA_BLUE);
    tft.setTextSize(2);
    tft.setTextColor(VISTA_WHITE);
    tft.setCursor(185, 7);
    
    char timeStr[6];
    sprintf(timeStr, "%02d:%02d", actualhour, actualminute);
    tft.print(timeStr);
    
    lastHour = actualhour;
    lastMinute = actualminute;
  }

  // Temperature display in bottom bar
  if (temperature != lastTemperature || forceDisplayUpdate) {
    tft.fillRect(5, 292, 70, 25, VISTA_DARK_BLUE);
    tft.setTextSize(2);
    tft.setTextColor(VISTA_YELLOW);
    tft.setCursor(10, 296);
    tft.print(temperature, 1);
    tft.setTextColor(VISTA_WHITE);
    tft.print("C");
    
    lastTemperature = temperature;
  }
  
  // Altitude in bottom bar
  static int lastAltitude = -999;
  int altitude = getFormattedAltitude();
  if (altitude != lastAltitude || forceDisplayUpdate) {
    tft.fillRect(85, 292, 75, 25, VISTA_DARK_BLUE);
    tft.setTextSize(2);
    tft.setTextColor(VISTA_LIGHT_BLUE);
    tft.setCursor(90, 296);
    tft.print(altitude);
    tft.print("m");
    
    lastAltitude = altitude;
  }
  
  // Direction in bottom bar
  static String lastDirection = "";
  if (direction != lastDirection || forceDisplayUpdate) {
    tft.fillRect(165, 292, 70, 25, VISTA_DARK_BLUE);
    tft.setTextSize(1);
    tft.setTextColor(VISTA_ORANGE);
    tft.setCursor(170, 298);
    tft.print(direction);
    
    lastDirection = direction;
  }
}

// ==================== SETUP & MAIN FUNCTIONS ====================

void debugTouch() {
  if (input->getPress()) {
    Serial.print("RAW X = ");
    Serial.print(input->point.x);
    Serial.print("   RAW Y = ");
    Serial.println(input->point.y);
  }
}

bool getTouchCoordinates(int &x, int &y) {
    if (input->getPress()) {
        x = input->point.x;
        y = input->point.y;
        return true;
    }
    return false;
}

void TFTsetup() {
  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);
  
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);

  input->begin();
  input->setRotation(1);
  
  // Setup backlight PWM (optional - adjust pin as needed)
  // ledcSetup(5, 5000, 8);
  // ledcAttachPin(YOUR_BACKLIGHT_PIN, 5);
  // ledcWrite(5, screenBrightness);

  drawMainScreen();
}

void refreshTFT() {
  static unsigned long lastUpdate = 0;
  const unsigned long updateInterval = 1000;
  unsigned long currentTime = millis();

  // Always handle touch
  handleTouch();
  
  // Uncomment for touch debugging
  // debugTouch();

  if (currentTime - lastUpdate >= updateInterval) {
    if (currentScreen == SCREEN_MAIN) {
      float voltageMotors = readVoltageMotors();
      float voltageESP = readVoltageESP32();

      updateVoltageValues(voltageMotors, voltageESP);
      updateWifiSignal();
      updateMainScreenInfo();
      
      if (forceDisplayUpdate) {
        forceDisplayUpdate = false;
      }
    } else if (currentScreen == SCREEN_SENSORS) {
      // Update sensor readings every second
      updateSensorReadings();
      // Only redraw the data areas, not the whole screen
      tft.fillRect(15, 45, 210, 110, VISTA_GLASS);
      drawRobotWithSensors(120, 100);
      
      tft.fillRect(15, 175, 210, 100, VISTA_GLASS);
      tft.setTextSize(1);
      tft.setTextColor(VISTA_WHITE);
      
      int yPos = 177;
      tft.setCursor(15, yPos);
      tft.print("FC:");
      tft.setTextColor(cachedMeasurements[4] < 35 ? VISTA_RED : VISTA_GREEN);
      tft.print(cachedMeasurements[4]);
      tft.setTextColor(VISTA_WHITE);
      tft.print("cm FL:");
      tft.setTextColor(cachedMeasurements[3] < 4 ? VISTA_RED : VISTA_GREEN);
      tft.print(cachedMeasurements[3]);
      tft.setTextColor(VISTA_WHITE);
      tft.print("cm FR:");
      tft.setTextColor(cachedMeasurements[5] < 4 ? VISTA_RED : VISTA_GREEN);
      tft.print(cachedMeasurements[5]);
      tft.setTextColor(VISTA_WHITE);
      tft.print("cm");
      
      yPos += 12;
      tft.setCursor(15, yPos);
      tft.print("BACK:");
      tft.setTextColor(cachedMeasurements[6] < 7 ? VISTA_RED : VISTA_GREEN);
      tft.print(cachedMeasurements[6]);
      tft.setTextColor(VISTA_WHITE);
      tft.print("cm LF:");
      tft.setTextColor(cachedMeasurements[2] < 10 ? VISTA_RED : VISTA_GREEN);
      tft.print(cachedMeasurements[2]);
      tft.setTextColor(VISTA_WHITE);
      tft.print("cm LB:");
      tft.setTextColor(cachedMeasurements[7] < 10 ? VISTA_RED : VISTA_GREEN);
      tft.print(cachedMeasurements[7]);
      tft.setTextColor(VISTA_WHITE);
      tft.print("cm");
      
      yPos += 18;
      tft.setCursor(15, yPos);
      tft.print("Temp:");
      tft.setTextColor(VISTA_YELLOW);
      tft.print(cachedTemp, 1);
      tft.setTextColor(VISTA_WHITE);
      tft.print("C Alt:");
      tft.setTextColor(VISTA_CYAN);
      tft.print(cachedAlt);
      tft.setTextColor(VISTA_WHITE);
      tft.print("m P:");
      tft.setTextColor(VISTA_LIGHT_BLUE);
      tft.print(cachedPress);
      tft.setTextColor(VISTA_WHITE);
      tft.print("hPa");
      
      yPos += 12;
      tft.setCursor(15, yPos);
      tft.print("Kierunek: ");
      tft.setTextColor(VISTA_ORANGE);
      tft.print(direction);
      tft.print("     "); // Clear old text
      
      yPos += 12;
      tft.setCursor(15, yPos);
      tft.setTextColor(VISTA_WHITE);
      tft.print("Kat:");
      tft.setTextColor(VISTA_LIGHT_BLUE);
      tft.print(robotAngle, 1);
      tft.setTextColor(VISTA_WHITE);
      tft.print("° Speed:");
      tft.setTextColor(VISTA_GREEN);
      tft.print(motorSpeed);
      tft.print("   "); // Clear old text
    }

    lastUpdate = currentTime;
  }
}
