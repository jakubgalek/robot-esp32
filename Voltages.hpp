#include <INA3221.h>
#include "I2C.hpp"

INA3221 ina_0(INA3221_ADDR40_GND);

// Ostatnio zmierzone napięcia. Odczyt z INA3221 (I2C) robi tylko Voltages_update()
// z zadania czujników (raz na sekundę); TFT, WWW i buzzer czytają te wartości z pamięci.
volatile float cachedVoltageMotors = 0.0f;
volatile float cachedVoltageESP32 = 0.0f;

/* ------------------------------------------------- */
void Voltages_update() {
    I2CLock lock(50);
    if (!lock.ok()) return;
    cachedVoltageESP32 = ina_0.getVoltage(INA3221_CH1);
    cachedVoltageMotors = ina_0.getVoltage(INA3221_CH2);
}
/* ------------------------------------------------- */
void INA3221_measure_init() {
    ina_0.begin(&Wire);
    ina_0.reset();
    ina_0.setShuntRes(10, 10, 10);
    Voltages_update();   // pierwsze wartości od razu, żeby TFT/buzzer nie widziały zera
}
/* ------------------------------------------------- */
float readVoltageMotors() {
    return cachedVoltageMotors;
}
float readVoltageESP32() {
    return cachedVoltageESP32;
}
/* ------------------------------------------------- */
