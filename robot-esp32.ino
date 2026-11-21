#include <Arduino.h>
#include "WiFi_config.hpp"
#include <Wire.h>
#include "I2C.hpp"
#include "Config.hpp"
#include "Voltages.hpp" 
#include "Time.hpp"
#include "Buzzer.hpp"
#include "Motor_functions.hpp"
#include "Odometry.hpp"
#include "BMP180.hpp"
#include "Distance_sensors.hpp"
#include "Explore.hpp"
#include "Webpage.hpp"
#include "SDCard.hpp"
#include "Settings.hpp"
#include "Network.hpp"
#include "TFT.hpp"
#include <ble_keyboard_mouse_client.h>

// BLE HID Client
BLEHIDClient hid;

// Stan połączenia klawiatury dla strony (kropka w górnym pasku). Biblioteka nie udostępnia publicznie
// BLEHIDDevice::is_connected() (jest prywatna), więc sięgamy do niej oficjalnym "obejściem" C++
// (jawna instancjacja szablonu daje dostęp do wskaźnika na prywatną metodę). Stan jest prawdziwy:
// zmienia się przy podłączeniu i odłączeniu klawiatury, bez wciskania klawiszy.
template <class Tag, typename Tag::type M> struct BtRob { friend typename Tag::type btGet(Tag) { return M; } };
struct BtTagConn { typedef bool (BLEHIDDevice::*type)(); friend type btGet(BtTagConn); };
template struct BtRob<BtTagConn, &BLEHIDDevice::is_connected>;
volatile bool btReady = false;   // ustawiane po hid.begin()

// Zmienne dla kontroli Bluetooth
volatile bool bt_forward_pressed = false;
volatile bool bt_backward_pressed = false;
volatile bool bt_left_pressed = false;
volatile bool bt_right_pressed = false;

// Funkcje callback dla Bluetooth
void on_key_pressed(bool is_modifier, uint8_t k) {
  if (!is_modifier) {
    
    switch(k) {
      case 0x52: // Strzałka w górę - Forward
        bt_forward_pressed = true;
        break;
      
      case 0x51: // Strzałka w dół - Backward
        bt_backward_pressed = true;
        break;
      
      case 0x50: // Strzałka w lewo - Left
        bt_left_pressed = true;
        break;
      
      case 0x4F: // Strzałka w prawo - Right
        bt_right_pressed = true;
        break;
      
      case 0x29: // ESC - Zwiększ prędkość
        if (motorSpeed < maxSpeed - 10) {
          motorSpeed += 10;
          ledcWrite(pwmChannelSpeed, motorSpeed);
        }
        break;
      
      case 0x35: // ~ - Zmniejsz prędkość
        if (motorSpeed > 140) {
          motorSpeed -= 10;
          ledcWrite(pwmChannelSpeed, motorSpeed);
        }
        break;
      
      case 0x2C: // Spacja - Stop
        requestAutoDriveStop();
        stop_driving();
        bt_forward_pressed = false;
        bt_backward_pressed = false;
        bt_left_pressed = false;
        bt_right_pressed = false;
        break;
      
      default:
        break;
    }
  }
}

void on_key_released(bool is_modifier, uint8_t k) {
  if (!is_modifier) {
    
    switch(k) {
      case 0x52: // Forward released
        bt_forward_pressed = false;
        if (!bt_backward_pressed && !bt_left_pressed && !bt_right_pressed) {
          stop_driving();
        }
        break;
      
      case 0x51: // Backward released
        bt_backward_pressed = false;
        if (!bt_forward_pressed && !bt_left_pressed && !bt_right_pressed) {
          stop_driving();
        }
        break;
      
      case 0x50: // Left released
        bt_left_pressed = false;
        if (!bt_forward_pressed && !bt_backward_pressed && !bt_right_pressed) {
          stop_driving();
        }
        break;
      
      case 0x4F: // Right released
        bt_right_pressed = false;
        if (!bt_forward_pressed && !bt_backward_pressed && !bt_left_pressed) {
          stop_driving();
        }
        break;
      
      default:
        break;
    }
  }
}

int btKeyboardState() {
  if (!btReady) return -1;
  BLEKeyboard& kb = hid.get_keyboard();
  return (kb.*btGet(BtTagConn()))() ? 1 : 0;
}

void handle_bluetooth_control() {
  // Obsługa sterowania tylko gdy automatyczna jazda jest wyłączona
  if (AutomaticDriveTaskHandle == NULL) {
    if (bt_forward_pressed) {
      // Sprawdź czujniki przed jazdą do przodu
      if (!frontBlocked()) {
        forward();
        ledcWrite(pwmChannelSpeed, motorSpeed);
      } else {
        stop_driving();
      }
    } else if (bt_backward_pressed) {
      // Sprawdź czujnik przed jazdą do tyłu
      if (!rearBlocked()) {
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
  sd_preinit();                  // CS karty SD w stan wysoki
  Settings_load();

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);   // 400 kHz: magistrala była zbyt wolna dla 8 czujników + żyroskopu
  Wire.setTimeOut(20);           // zawieszony odczyt nie blokuje zadań dłużej niż 20 ms
  I2C_init();   // wspólna blokada magistrali I2C - przed uruchomieniem jakichkolwiek zadań

  pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderInterrupt, RISING);

  Time_init();

  Buzzer_setup();

  pcf8574_init();
  
  BMP_init();
  
  INA3221_measure_init();
  
  Distance_sensors_init();

  // Zadanie czujników: odbiera pomiary przeszkód i odświeża napięcia / BMP180 (priorytet wyższy niż loop)
  xTaskCreatePinnedToCore(sensors_Task, "SensorsTask", 6144, NULL, 2, NULL, 1);

  // Ekran najpierw: podczas łączenia z WiFi widać "Connecting..." zamiast czarnego ekranu
  TFTsetup();

  TFTbootScreen("Laczenie z Wi-Fi");
  net_begin();
  sd_init();
  logEvent("program: poprawka 24, karta %s, RAM %u KB, blok %u KB", sdOk ? "jest" : "brak", (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMaxAllocHeap() / 1024));
  gotoScreen(SC_HOME);

  refreshTFT();

  ledcSetup(pwmChannelSpeed, pwmFrequencySpeed, pwmResolution);
  ledcAttachPin(SPEED_PIN, pwmChannelSpeed);

  ledcWrite(pwmChannelSpeed, motorSpeed);

  // Serwo skanera (ESP32Servo)
  Scan_servoInit();

  // Inicjalizacja MPU6050 (robot musi stać nieruchomo - kalibracja zera żyroskopu)
  byte mpuStatus;
  {
    I2CLock lock(8000);   // kalibracja trwa kilka sekund - zadanie czujników czeka na magistralę
    mpuStatus = mpu.begin();
    if (mpuStatus != 0) {
      Serial.print("❌MPU6050 init failed, status: ");
      Serial.println(mpuStatus);
    }
    mpu.calcGyroOffsets();
  }
  odoGyroReady = (mpuStatus == 0);   // odczyt żyroskopu robi odometria (zadanie czujników)

  // Mapa: wczytaj zapisaną (razem z ostatnią pozycją) albo zacznij od pustej
  odometry_loadGyroSign();
  Map_init();
  if (!mapLoadedFromFlash && sdOk) sd_restoreMap();

  Web_init();

  // Skany na żądanie (przycisk w panelu) + zapis mapy do flash
  xTaskCreatePinnedToCore(scan_Task, "ScanTask", 6144, NULL, 1, NULL, 0);

  // Inicjalizacja Bluetooth HID
  hid.begin("Robot ESP32", true, false); // nazwa, keyboard enabled, mouse disabled
  
  BLEKeyboard& keyboard = hid.get_keyboard();
  keyboard.on_key_pressed(on_key_pressed);
  keyboard.on_key_released(on_key_released);
  
  btReady = true;
}

void loop()
{
  // Obsługa BLE HID
  hid.loop();
  
  // RTC wystarczy odczytywać raz na sekundę
  static unsigned long lastTimeRefresh = 0;
  if (millis() - lastTimeRefresh >= 1000) {
    lastTimeRefresh = millis();
    Time_refresh();
  }
  odometry_persistGyroSign();
  checkWiFiSignal();
  server.handleClient();
  server.handleClient();          // przegladarka otwiera kilka polaczen naraz - obsluz wiecej niz jedno na obrot petli
  Web_maintain();
  refreshTFT();
  sd_loop();
  checkDriveConditions();
  checkMotorsAndReloadBuzzer();
  // Obsługa sterowania Bluetooth
  handle_bluetooth_control();
}