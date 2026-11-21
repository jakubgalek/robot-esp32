#pragma once
// ---------------------------------------------------------------------------
// Zapis i odczyt mapy w SPIFFS (/map.bin): nagłówek + surowa siatka (log-szanse).
// Razem z mapą zapisujemy ostatnią pozycję robota, żeby po restarcie wiedział, gdzie jest.
// UWAGA: "ESP32 Sketch Data Upload" kasuje cały SPIFFS, razem z mapą.
// ---------------------------------------------------------------------------
#include <FS.h>
#include <SPIFFS.h>
#include "MapGrid.hpp"
#include "Odometry.hpp"

#define MAP_FILE      "/map.bin"
#define MAP_FILE_TMP  "/map.tmp"

struct MapFileHeader {
  char magic[4];
  uint16_t version;
  uint16_t cells;
  uint16_t cellCm;
  uint16_t reserved;
  float x, y, h;
  uint32_t checksum;
};

OccGrid gridMap;
static SemaphoreHandle_t mapMutex = NULL;

class MapLock {
 public:
  explicit MapLock(uint32_t waitMs = 500) : _held(false) {
    if (mapMutex) _held = (xSemaphoreTake(mapMutex, pdMS_TO_TICKS(waitMs)) == pdTRUE);
  }
  ~MapLock() { if (_held) xSemaphoreGive(mapMutex); }
  bool ok() const { return _held; }
 private:
  MapLock(const MapLock&);
  MapLock& operator=(const MapLock&);
  bool _held;
};

volatile bool mapDirty = false;          // zmiany jeszcze nie zapisane
volatile bool mapLoadedFromFlash = false;
volatile bool mapAllocFailed = false;
static unsigned long mapLastSaveMs = 0;

static uint32_t mapChecksum(const int8_t* c, size_t n) {
  uint32_t a = 1, b = 0;
  for (size_t i = 0; i < n; i++) {
    a = (a + (uint8_t)c[i]) % 65521;
    b = (b + a) % 65521;
  }
  return (b << 16) | a;
}

// Zapis siatki do otwartego pliku (SPIFFS albo karta SD). Wołający trzyma MapLock.
bool MapStore_writeFile(File& f) {
  const size_t n = (size_t)MAP_CELLS * MAP_CELLS;
  MapFileHeader h;
  memcpy(h.magic, "RMAP", 4);
  h.version = 3;
  h.cells = MAP_CELLS;
  h.cellCm = MAP_CELL_CM;
  h.reserved = gridMap.refValid ? (uint16_t)lroundf(mgWrap360(gridMap.startHeading) * 100.0f) % 36000 : 0xFFFF;   // kierunek startu w układzie wyrównanym do ścian (0.01 st.)
  Pose2 p = odometry_get();
  h.x = p.x; h.y = p.y; h.h = p.h;
  h.checksum = mapChecksum(gridMap.cells, n);
  bool ok = f.write((const uint8_t*)&h, sizeof(h)) == sizeof(h);
  for (size_t off = 0; ok && off < n; off += 1024) {
    size_t len = (n - off) < 1024 ? (n - off) : 1024;
    ok = f.write((const uint8_t*)gridMap.cells + off, len) == len;
  }
  return ok;
}

// Odczyt z otwartego pliku do siatki (sprawdza nagłówek i sumę kontrolną). Wołający trzyma MapLock.
bool MapStore_readFile(File& f) {
  const size_t n = (size_t)MAP_CELLS * MAP_CELLS;
  MapFileHeader h;
  bool ok = f.read((uint8_t*)&h, sizeof(h)) == sizeof(h) &&
            memcmp(h.magic, "RMAP", 4) == 0 && (h.version == 1 || h.version == 2 || h.version == 3) &&
            h.cells == MAP_CELLS && h.cellCm == MAP_CELL_CM &&
            (size_t)f.size() == sizeof(h) + n;
  if (ok) {
    for (size_t off = 0; ok && off < n; off += 1024) {
      size_t len = (n - off) < 1024 ? (n - off) : 1024;
      ok = f.read((uint8_t*)gridMap.cells + off, len) == len;
    }
  }
  if (ok && mapChecksum(gridMap.cells, n) != h.checksum) ok = false;
  if (!ok) { gridMap.clear(); return false; }
  Pose2 p = {h.x, h.y, h.h};
  odometry_set(p);
  gridMap.scanned = true;
  gridMap.refValid = (h.version == 3 && h.reserved < 36000);
  gridMap.startHeading = gridMap.refValid ? h.reserved / 100.0f : (float)ROBOT_START_HEADING;
  gridMap.version++;
  mapDirty = false;
  return true;
}

void (*mapSavedHook)() = NULL;   // wołane po udanym zapisie do flash (karta SD robi wtedy kopię)

bool MapStore_save() {
  MapLock lock;
  if (!gridMap.cells) return false;
  File f = SPIFFS.open(MAP_FILE_TMP, "w");
  if (!f) return false;
  bool ok = MapStore_writeFile(f);
  f.close();
  if (!ok) { SPIFFS.remove(MAP_FILE_TMP); return false; }
  SPIFFS.remove(MAP_FILE);
  if (!SPIFFS.rename(MAP_FILE_TMP, MAP_FILE)) return false;
  mapDirty = false;
  mapLastSaveMs = millis();
  if (mapSavedHook) mapSavedHook();
  return true;
}

// Zapis, jeśli są niezapisane zmiany i minął minimalny odstęp (chroni pamięć flash)
void MapStore_saveIfDue() {
  if (mapDirty && (millis() - mapLastSaveMs) > MAP_SAVE_MIN_INTERVAL_MS) MapStore_save();
}

bool MapStore_load() {
  MapLock lock;
  if (!gridMap.cells || !SPIFFS.exists(MAP_FILE)) return false;
  File f = SPIFFS.open(MAP_FILE, "r");
  if (!f) return false;
  bool ok = MapStore_readFile(f);
  f.close();
  return ok;
}

void Map_init() {
  mapMutex = xSemaphoreCreateMutex();
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS: nie można zamontować - mapa nie będzie zapisywana");
  }
  if (!gridMap.begin()) {
    mapAllocFailed = true;
    Serial.println("❌ Za mało pamięci na mapę - zmniejsz MAP_CELLS w Config.hpp");
    return;
  }
  gridMap.clear();
  if (MapStore_load()) {
    mapLoadedFromFlash = true;
  } else {
  }
}
