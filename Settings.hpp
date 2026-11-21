#pragma once
// ---------------------------------------------------------------------------
// Ustawienia zmieniane z ekranu robota - zapisywane w pamięci nieulotnej (NVS, przestrzeń "cfg").
// ---------------------------------------------------------------------------
#include <Preferences.h>
#include "Config.hpp"

bool cfgSdLog = true;       // dziennik na kartę SD
bool cfgSdBackup = true;    // automatyczna kopia mapy na kartę SD
bool cfgNtp = false;        // czas z internetu (NTP), gdy jest Wi-Fi
uint8_t cfgTheme = 0;       // motyw ekranu: 0 domyslny, 1 jasny

void Settings_load() {
  Preferences p;
  if (!p.begin("cfg", true)) return;
  uint16_t s = p.getUShort("scan", AUTO_SCAN_EVERY_CM);
  if (s >= 30 && s <= 120) gScanEveryCm = s;
  cfgSdLog = p.getBool("sdlog", true);
  cfgSdBackup = p.getBool("sdbak", true);
  cfgNtp = p.getBool("ntp", false);
  cfgTheme = p.getUChar("thm", 255);
  if (cfgTheme == 255) cfgTheme = p.getUChar("theme", 0) == 2 ? 1 : 0;      // stary zapis: 2 = jasny
  cfgTheme %= 2;
  p.end();
}

void Settings_save() {
  Preferences p;
  if (!p.begin("cfg", false)) return;
  p.putUShort("scan", (uint16_t)gScanEveryCm);
  p.putBool("sdlog", cfgSdLog);
  p.putBool("sdbak", cfgSdBackup);
  p.putBool("ntp", cfgNtp);
  p.putUChar("thm", cfgTheme);
  p.end();
}
