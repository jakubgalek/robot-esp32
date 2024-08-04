#include <SPI.h>
#include <SD.h>

// Pinout ESP32 HSPI interface
const int8_t SDCARD_SS_PIN = 15; // CS pin for SD card

void setup() {
  Serial.begin(115200);

  // Initialize HSPI interface
  SPI.begin(14, 32, 13, SDCARD_SS_PIN); // HSPI: CLK=14, MISO=2, MOSI=12

  // Initialize SD card
  if (!SD.begin(SDCARD_SS_PIN)) {
    Serial.println("Card Mount Failed");
    return;
  }
  
  Serial.println("Card mounted!");

  listFiles("/");
}

void loop() {
  // Your code here
}

void listFiles(const char *dirname) {
  Serial.println("Listing files in directory: " + String(dirname));
  
  File root = SD.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }
  
  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
    } else {
      Serial.print("  FILE: ");
    }
    Serial.println(file.name());
    file = root.openNextFile();
  }

  root.close();
}
