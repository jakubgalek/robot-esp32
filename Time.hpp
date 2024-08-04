#include <DS3231.h>

DS3231 rtcClock; // Variable name changed from the default "clock" to "rtcClock" because there is a library conflict with the esp32 built-in clock
RTCDateTime dt;

/* ------------------------------------------------- */
void Time_init() {
  // Initialize DS3231
  Serial.println("Initialize DS3231");
  rtcClock.begin();
  //rtcClock.setDateTime(__DATE__, __TIME__);
}
/* ------------------------------------------------- */
void Time_refresh() {
  dt = rtcClock.getDateTime();
}
/* ------------------------------------------------- */