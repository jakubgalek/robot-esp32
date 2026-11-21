#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Wspólna blokada magistrali I2C.
//
// Do I2C (Wire) sięgają równocześnie: loop(), zadanie czujników, zadanie radaru,
// zadanie auto-jazdy i callbacki BLE. Wire nie jest bezpieczny dla transakcji
// "zapis rejestru + odczyt", więc każde wywołanie biblioteki, która używa Wire,
// opakowujemy w:
//
//     I2CLock lock(50);          // czekaj maks. 50 ms
//     if (!lock.ok()) return;    // nie udało się - pomiń (albo użyj starej wartości)
//     ... wywołanie biblioteki ...
//
// Blokada jest rekurencyjna (to samo zadanie może ją wziąć ponownie) i zwalnia się
// sama przy wyjściu z zakresu. Przed I2C_init() (czyli w setup()) nic nie blokuje.
// ---------------------------------------------------------------------------
static SemaphoreHandle_t i2cMutex = NULL;

static void I2C_init() {
  if (i2cMutex == NULL) i2cMutex = xSemaphoreCreateRecursiveMutex();
}

class I2CLock {
 public:
  explicit I2CLock(uint32_t timeoutMs = 100) : _ok(true), _held(false) {
    if (i2cMutex != NULL) {
      _ok = (xSemaphoreTakeRecursive(i2cMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
      _held = _ok;
    }
  }
  ~I2CLock() {
    if (_held) xSemaphoreGiveRecursive(i2cMutex);
  }
  bool ok() const { return _ok; }

 private:
  I2CLock(const I2CLock&);             // bez kopiowania
  I2CLock& operator=(const I2CLock&);
  bool _ok;
  bool _held;
};
