#include <Arduino.h>
#include "WiFi_config.hpp"
#include <Wire.h>
#include "Voltages.hpp" 
#include "Time.hpp"
#include "Buzzer.hpp"
#include "Motor_functions.hpp"
#include "BMP180.hpp"
#include "Distance_sensors.hpp"
#include "TFT.hpp"
#include "Map.hpp"
#include "Webpage.hpp"
#include <Ticker.h>
#include <ble_keyboard_mouse_client.h>

SemaphoreHandle_t xMutex;
Ticker mpuTicker;

// BLE HID Client
BLEHIDClient hid;

// Zmienne dla kontroli Bluetooth
volatile bool bt_forward_pressed = false;
volatile bool bt_backward_pressed = false;
volatile bool bt_left_pressed = false;
volatile bool bt_right_pressed = false;

void updateMPU() {
  mpu.update();
}

float currentAngleZ = 0;
unsigned long lastAngleUpdate = 0;

// Funkcje callback dla Bluetooth
void on_key_pressed(bool is_modifier, uint8_t k) {
  if (!is_modifier) {
    Serial.printf("Key 0x%02x pressed!\n", k);
    
    switch(k) {
      case 0x52: // Strzałka w górę - Forward
        bt_forward_pressed = true;
        Serial.println("Forward activated");
        break;
      
      case 0x51: // Strzałka w dół - Backward
        bt_backward_pressed = true;
        Serial.println("Backward activated");
        break;
      
      case 0x50: // Strzałka w lewo - Left
        bt_left_pressed = true;
        Serial.println("Left activated");
        break;
      
      case 0x4F: // Strzałka w prawo - Right
        bt_right_pressed = true;
        Serial.println("Right activated");
        break;
      
      case 0x29: // ESC - Zwiększ prędkość
        if (motorSpeed < maxSpeed - 10) {
          motorSpeed += 10;
          ledcWrite(pwmChannelSpeed, motorSpeed);
          Serial.print("Speed increased to: ");
          Serial.println(motorSpeed);
        }
        break;
      
      case 0x35: // ~ - Zmniejsz prędkość
        if (motorSpeed > 140) {
          motorSpeed -= 10;
          ledcWrite(pwmChannelSpeed, motorSpeed);
          Serial.print("Speed decreased to: ");
          Serial.println(motorSpeed);
        }
        break;
      
      case 0x2C: // Spacja - Stop
        stop_driving();
        bt_forward_pressed = false;
        bt_backward_pressed = false;
        bt_left_pressed = false;
        bt_right_pressed = false;
        Serial.println("Emergency stop!");
        break;
      
      default:
        break;
    }
  }
}

void on_key_released(bool is_modifier, uint8_t k) {
  if (!is_modifier) {
    Serial.printf("Key 0x%02x released!\n", k);
    
    switch(k) {
      case 0x52: // Forward released
        bt_forward_pressed = false;
        if (!bt_backward_pressed && !bt_left_pressed && !bt_right_pressed) {
          stop_driving();
          Serial.println("Forward released - stopping");
        }
        break;
      
      case 0x51: // Backward released
        bt_backward_pressed = false;
        if (!bt_forward_pressed && !bt_left_pressed && !bt_right_pressed) {
          stop_driving();
          Serial.println("Backward released - stopping");
        }
        break;
      
      case 0x50: // Left released
        bt_left_pressed = false;
        if (!bt_forward_pressed && !bt_backward_pressed && !bt_right_pressed) {
          stop_driving();
          Serial.println("Left released - stopping");
        }
        break;
      
      case 0x4F: // Right released
        bt_right_pressed = false;
        if (!bt_forward_pressed && !bt_backward_pressed && !bt_left_pressed) {
          stop_driving();
          Serial.println("Right released - stopping");
        }
        break;
      
      default:
        break;
    }
  }
}

void handle_bluetooth_control() {
  // Obsługa sterowania tylko gdy automatyczna jazda jest wyłączona
  if (AutomaticDriveTaskHandle == NULL) {
    if (bt_forward_pressed) {
      // Sprawdź czujniki przed jazdą do przodu
      if (!(measurement3 <= 4 || measurement4 <= 35 || measurement5 <= 4)) {
        forward();
        ledcWrite(pwmChannelSpeed, motorSpeed);
      } else {
        stop_driving();
      }
    } else if (bt_backward_pressed) {
      // Sprawdź czujnik przed jazdą do tyłu
      if (!(measurement6 <= 7)) {
        backward();
        ledcWrite(pwmChannelSpeed, motorSpeed);
      } else {
        stop_driving();
      }
    } else if (bt_left_pressed) {
      turn_left();
      ledcWrite(pwmChannelSpeed, motorSpeed);
    } else if (bt_right_pressed) {
      turn_right();
      ledcWrite(pwmChannelSpeed, motorSpeed);
    }
  }
}

void setup() 
{
  Serial.begin(115200);

  Wire.begin();

  pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderInterrupt, RISING);

  Time_init();

  Buzzer_setup();

  pcf8574_init();
  
  BMP_init();
  
  INA3221_measure_init();
  
  Distance_sensors_init();

  startWiFi();

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
 
  mpuTicker.attach_ms(5, updateMPU);

  Web_init();
  
  xMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    collect_distances_servo_Task,
    "CollectDistancesServoTask",
    7000,
    NULL,
    1,
    &CollectDistancesTaskHandle,
    1
  );

  // Inicjalizacja Bluetooth HID
  Serial.println("🔄Starting Bluetooth HID Client...");
  hid.begin("Robot ESP32", true, false); // nazwa, keyboard enabled, mouse disabled
  
  BLEKeyboard& keyboard = hid.get_keyboard();
  keyboard.on_key_pressed(on_key_pressed);
  keyboard.on_key_released(on_key_released);
  
  Serial.println("✅Bluetooth HID Client initialized");
  Serial.println("Waiting for BLE Keyboard connection...");
  Serial.println("Controls:");
  Serial.println("  Arrow Up    - Forward");
  Serial.println("  Arrow Down  - Backward");
  Serial.println("  Arrow Left  - Turn Left");
  Serial.println("  Arrow Right - Turn Right");
  Serial.println("  ESC         - Increase Speed (+10)");
  Serial.println("  ~           - Decrease Speed (-10)");
  Serial.println("  Space       - Emergency Stop");
}

void loop()
{
  // Obsługa BLE HID
  hid.loop();
  
  Time_refresh();
  checkWiFiSignal();
  server.handleClient();
  refreshTFT();
  checkDriveConditions();
  checkMotorsAndReloadBuzzer();
  // Obsługa sterowania Bluetooth
  handle_bluetooth_control();
}