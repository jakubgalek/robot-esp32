#define BUZZER_PIN 33

// Strefy alarmowe napięć [V]: alarm gdy napięcie mieści się w przedziale [min, max]
// (logika bez zmian względem poprzedniej wersji).
const float minVoltageMotors = 5.0;
const float maxVoltageMotors = 7.0;

const float minVoltageESP32 = 4.55;
const float maxVoltageESP32 = 6.0;

const unsigned long buzzerInterval = 1500; // Okres sygnału [ms]
const unsigned long buzzerOnTime   = 450;  // Czas trwania dźwięku w okresie [ms]
const unsigned long voltageCheckInterval = 500; // Jak często odczytywać napięcia do alarmu [ms]
bool buzzerActive = false;

int pwmChannelBuzzer = 4;

bool checkVoltagesAlarm();
void buzzerControl();

/* ------------------------------------------------- */
// Włącza/wyłącza dźwięk tylko przy zmianie stanu (bez ciągłego wołania noTone/tone).
// Używamy wyłącznie API LEDC (kanał pwmChannelBuzzer), bez tone()/noTone(),
// żeby nie kolidowało z kanałami serwa i silników.
static void buzzerSet(bool on) {
  static int state = -1;
  if (state == (int)on) return;
  state = on ? 1 : 0;
  if (on) {
    ledcWriteTone(pwmChannelBuzzer, 1000);  // 1000 Hz
  } else {
    ledcWrite(pwmChannelBuzzer, 0);
  }
}
/* ------------------------------------------------- */
void Buzzer_setup() {
  ledcSetup(pwmChannelBuzzer, 1000, 8);                  // Settings: PWM channel, frequency 1000 Hz, resolution 8 bits
  ledcAttachPin(BUZZER_PIN, pwmChannelBuzzer);           // Assign the BUZZER_PIN pin to the selected PWM channel
  ledcWrite(pwmChannelBuzzer, 0);
}
/* ------------------------------------------------- */
// Wyciszenie (np. gdy jadą silniki) - gwarantuje, że ton nie zostanie włączony "na stałe".
void Buzzer_silence() {
  buzzerSet(false);
}
/* ------------------------------------------------- */
void Buzzer_reload() {
  if (checkVoltagesAlarm()) {
    buzzerControl();
  } else {
    buzzerSet(false);
  }
}
/* ------------------------------------------------- */

bool checkVoltagesAlarm() {
  // Napięcia czytamy z INA3221 (I2C) tylko co voltageCheckInterval,
  // a nie w każdej iteracji loop().
  static unsigned long lastCheck = 0;
  static bool cachedAlarm = false;
  static bool firstRun = true;

  unsigned long now = millis();
  if (!firstRun && (now - lastCheck) < voltageCheckInterval) return cachedAlarm;
  firstRun = false;
  lastCheck = now;

  float voltageESP32 = readVoltageESP32();
  float voltageMotors = readVoltageMotors();

  cachedAlarm = (voltageESP32 >= minVoltageESP32 && voltageESP32 <= maxVoltageESP32) ||
                (voltageMotors >= minVoltageMotors && voltageMotors <= maxVoltageMotors);
  buzzerActive = cachedAlarm;
  return cachedAlarm;
}
/* ------------------------------------------------- */
// Sygnał: buzzerOnTime ms dźwięku, potem cisza do końca okresu buzzerInterval.
// (Poprzednio dźwięk trwał ok. 1050 ms z 1500 ms, bo żadna gałąź nie wyłączała go po 450 ms.)
void buzzerControl() {
  unsigned long phase = millis() % buzzerInterval;
  buzzerSet(phase < buzzerOnTime);
}
/* ------------------------------------------------- */
