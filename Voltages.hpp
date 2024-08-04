#include <INA3221.h>

INA3221 ina_0(INA3221_ADDR40_GND);

/* ------------------------------------------------- */
void INA3221_measure_init() {
    ina_0.begin(&Wire);
    ina_0.reset();
    ina_0.setShuntRes(10, 10, 10);
}
/* ------------------------------------------------- */
float readVoltageMotors() {
    return ina_0.getVoltage(INA3221_CH2);
}
float readVoltageESP32() {
    return ina_0.getVoltage(INA3221_CH1);
}
/* ------------------------------------------------- */
void read_voltages_console() {
    Serial.print("Voltage ESP32: ");
    Serial.println(ina_0.getVoltage(INA3221_CH1));
   // Serial.print("Current: ");
   // Serial.println(ina_1.getCurrent(INA3221_CH2) * 1000);
    Serial.print("Voltage Motors: ");
    Serial.println(ina_0.getVoltage(INA3221_CH2));
   // Serial.print("Current: ");
   // Serial.println(ina_1.getCurrent(INA3221_CH3) * 1000);
}
/* ------------------------------------------------- */