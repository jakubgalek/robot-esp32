#pragma once
#include <DS3231.h>
#include "I2C.hpp"

DS3231 rtcClock; // Variable name changed from the default "clock" to "rtcClock" because there is a library conflict with the esp32 built-in clock
RTCDateTime dt;

/* ------------------------------------------------- */
void Time_init() {
  // Initialize DS3231
#if !RTC_ENABLED
  return;
#endif
  rtcClock.begin();
  //rtcClock.setDateTime(__DATE__, __TIME__);
}
/* ------------------------------------------------- */
void Time_refresh() {
#if !RTC_ENABLED
  return;
#endif
  I2CLock lock(50);
  if (!lock.ok()) return;   // zostaje poprzednia wartość dt
  dt = rtcClock.getDateTime();
}
/* ------------------------------------------------- */

// Ustawienie zegara RTC (z ekranu albo z internetu). Zwraca false, gdy RTC wyłączony lub magistrala zajęta.
bool Time_set(uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mi, uint8_t s) {
#if !RTC_ENABLED
  return false;
#endif
  I2CLock lock(200);
  if (!lock.ok()) return false;
  rtcClock.setDateTime(y, mo, d, h, mi, s);
  dt = rtcClock.getDateTime();
  return true;
}
