#define BUZZER_PIN 33

const float minVoltageMotors = 5.0;
const float maxVoltageMotors = 5.8;

const float minVoltageESP32 = 4.55;
const float maxVoltageESP32 = 4.9;

unsigned long buzzerPreviousMillis = 0;
const unsigned long buzzerInterval = 1500; // Buzzer sound interval in milliseconds
bool buzzerActive = false;

bool checkVoltagesAlarm();
void buzzerControl();

/* ------------------------------------------------- */
void Buzzer_setup() {
  ledcSetup(4, 1000, 8); // Settings: PWM channel, frequency 1000 Hz, resolution 8 bits
  ledcAttachPin(BUZZER_PIN, 4); // Assign the BUZZER_PIN pin to the selected PWM channel

  Serial.println("Buzzer setup completed.");
}
/* ------------------------------------------------- */
void Buzzer_reload() {
  bool alarmActive = checkVoltagesAlarm();
  if (alarmActive) {
    //Serial.println("Alarm condition detected. Activating buzzer.");
    buzzerControl();
  } else {
    //Serial.println("No alarm condition detected. Buzzer deactivated.");
    noTone(BUZZER_PIN);
  }
}
/* ------------------------------------------------- */

bool checkVoltagesAlarm() {
  float voltageESP32 = readVoltageESP32();
  float voltageMotors = readVoltageMotors();

  // Print Voltages
  //Serial.print("Voltage ESP32: ");
  //Serial.println(voltageESP32);
  //Serial.print("Voltage Motors: ");
  //Serial.println(voltageMotors);

  // Check if the ESP32 or Motors voltage is out of range
  if ((voltageESP32 >= minVoltageESP32 && voltageESP32 <= maxVoltageESP32) ||
      (voltageMotors >= minVoltageMotors && voltageMotors <= maxVoltageMotors)) {
    buzzerActive = true;
   // Serial.println("Alarm condition met. Setting buzzerActive to true.");
    return true;
  } else {
    buzzerActive = false;
   // Serial.println("No alarm condition met. Setting buzzerActive to false.");
    return false;
  }
}
/* ------------------------------------------------- */
void buzzerControl() {
  static unsigned long buzzerStartTime = 0;
  unsigned long currentMillis = millis();

  if (buzzerStartTime == 0 || (currentMillis - buzzerStartTime >= buzzerInterval)) {
    buzzerStartTime = currentMillis;
    //Serial.println("Buzzer start/reset interval.");
  }

  if (currentMillis - buzzerStartTime <= 450) { // Last for 0.45 seconds
    tone(BUZZER_PIN, 1000); // Emit a tone with a frequency of 1000 Hz
  //  Serial.println("Buzzer sounding...");
  } else if (currentMillis - buzzerStartTime > buzzerInterval - 450) {  // Short pause before the buzzer sounds again
    noTone(BUZZER_PIN);
  //  Serial.println("Buzzer off before next cycle.");
  }
}
/* ------------------------------------------------- */