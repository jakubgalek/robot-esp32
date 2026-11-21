#pragma once
// ---------------------------------------------------------------------------
// Opcjonalna karta SD. Bez karty (albo przy SD_ENABLED 0) wszystko działa jak wcześniej.
// Karta pracuje na osobnej magistrali HSPI. Operacje na niej robi wątek loop(); inne zadania tylko
// zgłaszają prośby (flagi) lub dopisują wpisy do małego bufora.
// ---------------------------------------------------------------------------
#include "Config.hpp"
#include "Settings.hpp"

volatile bool sdOk = false;          // karta włożona i działa
volatile bool sdBackupPending = false;
uint32_t sdSizeMB = 0;

#if SD_ENABLED
#include <SD.h>
#include <SPI.h>
#include "MapStore.hpp"
#include "Time.hpp"
#include "Log.hpp"

#define SD_DIR       "/robot"
#define SD_MAPS_DIR  "/robot/maps"
#define SD_LOG_FILE  "/robot/log.txt"
#define SD_LOG_OLD   "/robot/log_old.txt"
#define SD_MAP_LAST  "/robot/map_last.bin"
#define SD_LOG_MAX   400000UL

// Bufor wpisów dziennika czekających na zapis (zapis co kilka sekund, nie przy każdym wpisie)
#define SD_Q_LINES 12
static char sdQ[SD_Q_LINES][LOG_LEN];
static volatile uint8_t sdQHead = 0, sdQCount = 0;
static portMUX_TYPE sdQMux = portMUX_INITIALIZER_UNLOCKED;

static void sdLogSink(const char* line) {
  if (!sdOk || !cfgSdLog) return;
  portENTER_CRITICAL(&sdQMux);
  strncpy(sdQ[sdQHead], line, LOG_LEN - 1); sdQ[sdQHead][LOG_LEN - 1] = 0;
  sdQHead = (sdQHead + 1) % SD_Q_LINES;
  if (sdQCount < SD_Q_LINES) sdQCount++;      // pełny bufor: najstarszy wpis ginie
  portEXIT_CRITICAL(&sdQMux);
}

static SPIClass sdSpi(HSPI);                    // karta ma własną magistralę - ekran (VSPI) działa bez zakłóceń

static void sdFail(const char* why) {
  sdOk = false;
  Serial.print("SD: "); Serial.println(why);
}

// Pin CS w stan wysoki jak najwcześniej (setup), żeby karta nie reagowała na ruch na wspólnej magistrali
void sd_preinit() { pinMode(SD_CS_PIN, OUTPUT); digitalWrite(SD_CS_PIN, HIGH); }

static bool sdSpiStarted = false;

static void sdLog(const char* what) { logEvent("karta SD: %s", what); }

// Karta jest montowana TYLKO na czas operacji (SdSession) i zaraz odmontowywana: poza operacja nie zajmuje RAM-u (FatFs, bufory),
// nie ma stanu, ktory moze sie zepsuc po wyjeciu karty, i nie dotyka magistrali. sdOk = wynik ostatniej proby wykrycia.
static int sdDepth = 0;
static bool sdMount() {
  if (sdDepth > 0) { sdDepth++; return true; }
  if (!sdSpiStarted) { sdSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, -1); sdSpiStarted = true; }   // CS obsluguje biblioteka SD
  sd_preinit();
  if (!SD.begin(SD_CS_PIN, sdSpi, SD_FREQ_HZ) || SD.cardType() == CARD_NONE) { SD.end(); sdOk = false; return false; }
  sdSizeMB = (uint32_t)(SD.cardSize() / (1024ULL * 1024ULL));
  SD.mkdir(SD_DIR); SD.mkdir(SD_MAPS_DIR);
  sdOk = true; sdDepth = 1;
  return true;
}
static void sdUnmount() { if (sdDepth > 0 && --sdDepth == 0) SD.end(); }
struct SdSession {
  bool ok;
  SdSession() : ok(sdMount()) {}
  ~SdSession() { if (ok) sdUnmount(); }
};

bool sd_init() { SdSession ss; return ss.ok; }                 // wykrycie karty przy starcie (potem odmontowanie)

static void sdStamp(char* out, size_t n, bool forName) {
  if (dt.year >= 2024) {
    if (forName) snprintf(out, n, "%04u%02u%02u_%02u%02u%02u", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
    else snprintf(out, n, "%04u-%02u-%02u %02u:%02u:%02u  ", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
  } else {
    if (forName) snprintf(out, n, "t%lu", millis() / 1000);   // brak daty z RTC: czas od startu
    else out[0] = 0;
  }
}

// Zapis oczekujących wpisów dziennika
static void sdFlushLog() {
  char local[SD_Q_LINES][LOG_LEN];
  uint8_t cnt;
  portENTER_CRITICAL(&sdQMux);
  cnt = sdQCount;
  for (uint8_t i = 0; i < cnt; i++) memcpy(local[i], sdQ[(sdQHead + SD_Q_LINES - cnt + i) % SD_Q_LINES], LOG_LEN);
  sdQCount = 0;
  portEXIT_CRITICAL(&sdQMux);
  if (!cnt) return;
  File f = SD.open(SD_LOG_FILE, FILE_APPEND);
  if (!f) { sdFail("nie mozna otworzyc dziennika"); return; }
  char ts[28];
  sdStamp(ts, sizeof(ts), false);
  bool ok = true;
  for (uint8_t i = 0; ok && i < cnt; i++) {
    ok = f.print(ts) > 0 || ts[0] == 0;
    ok = ok && f.println(local[i]) > 0;
  }
  size_t sz = f.size();
  f.close();
  if (!ok) { sdFail("blad zapisu dziennika"); return; }
  if (sz > SD_LOG_MAX) { SD.remove(SD_LOG_OLD); SD.rename(SD_LOG_FILE, SD_LOG_OLD); }   // rotacja: zostaje poprzedni plik
}

// Zapis aktualnej mapy pod daną ścieżką (format jak w flash)
static bool sdWriteMap(const char* path) {
  MapLock lock(2000);
  if (!lock.ok() || !gridMap.cells) return false;
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  bool ok = MapStore_writeFile(f);
  f.close();
  return ok;
}

// Podgląd mapy jako obraz PGM (do otwarcia na komputerze): ściany czarne, wolne białe, nieznane szare
static bool sdWritePgm(const char* path) {
  MapLock lock(2000);
  if (!lock.ok() || !gridMap.cells) return false;
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  char hdr[32];
  int hl = snprintf(hdr, sizeof(hdr), "P5\n%d %d\n255\n", MAP_CELLS, MAP_CELLS);
  bool ok = f.write((const uint8_t*)hdr, hl) == (size_t)hl;
  uint8_t row[MAP_CELLS];
  for (int cy = MAP_CELLS - 1; ok && cy >= 0; cy--) {          // y do góry
    for (int cx = 0; cx < MAP_CELLS; cx++) {
      int8_t v = gridMap.at(cx, cy);
      row[cx] = v >= L_OCC_T ? 0 : (v <= L_FREE_T ? 255 : 170);
    }
    ok = f.write(row, MAP_CELLS) == (size_t)MAP_CELLS;
  }
  f.close();
  return ok;
}

// Zapis mapy do biblioteki: /robot/maps/mapa_<data>.bin + .pgm
bool sd_saveMapNamed() {
  SdSession ss; if (!ss.ok) return false;
  char st[24], p1[64], p2[64];
  sdStamp(st, sizeof(st), true);
  snprintf(p1, sizeof(p1), SD_MAPS_DIR "/mapa_%s.bin", st);
  snprintf(p2, sizeof(p2), SD_MAPS_DIR "/mapa_%s.pgm", st);
  bool ok = sdWriteMap(p1);
  if (ok) sdWritePgm(p2);
  if (ok) logEvent("karta SD: zapisano mape %s", p1 + 12);
  else logEvent("karta SD: nie udalo sie zapisac mapy");
  return ok;
}

// Kopia zapasowa (zapis do pliku tymczasowego, potem podmiana - przerwany zapis nie niszczy starej kopii)
static bool sdBackupMap() {
  const char* tmp = "/robot/map_last.tmp";
  bool ok = sdWriteMap(tmp);
  if (ok) { SD.remove(SD_MAP_LAST); ok = SD.rename(tmp, SD_MAP_LAST); }
  else SD.remove(tmp);
  return ok;
}

static bool sdReadMap(const char* path) {
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  MapLock lock(2000);
  bool ok = lock.ok() && gridMap.cells && MapStore_readFile(f);
  f.close();
  if (ok) { mapDirty = true; mapLastSaveMs = 0; }   // zapisze się też do flash
  return ok;
}

// Przy starcie, gdy we flash nie ma mapy (np. po "Sketch Data Upload"): odtwórz ostatnią kopię z karty
bool sd_restoreMap() {
  SdSession ss; if (!ss.ok || !SD.exists(SD_MAP_LAST)) return false;
  bool ok = sdReadMap(SD_MAP_LAST);
  if (ok) { MapStore_save(); logEvent("karta SD: przywrocono ostatnia kopie mapy"); }
  return ok;
}

// Lista zapisanych map (najnowsze pierwsze); names = tylko nazwy plików .bin
int sd_listMaps(char names[][32], int maxN) {
  int n = 0;
  SdSession ss; if (!ss.ok) return 0;
  File dir = SD.open(SD_MAPS_DIR);
  if (!dir) return 0;
  for (File e = dir.openNextFile(); e; e = dir.openNextFile()) {
    const char* nm = e.name();
    const char* sl = strrchr(nm, '/');
    if (sl) nm = sl + 1;
    size_t L = strlen(nm);
    if (!e.isDirectory() && L > 4 && L < 32 && strcmp(nm + L - 4, ".bin") == 0 && n < maxN) { strcpy(names[n], nm); n++; }
    e.close();
  }
  dir.close();
  for (int a = 0; a < n; a++)                                   // malejąco po nazwie (data w nazwie)
    for (int b = a + 1; b < n; b++)
      if (strcmp(names[b], names[a]) > 0) { char t[32]; strcpy(t, names[a]); strcpy(names[a], names[b]); strcpy(names[b], t); }
  return n;
}

bool sd_loadMapFile(const char* name) {
  SdSession ss; if (!ss.ok) return false;
  char p[64];
  snprintf(p, sizeof(p), SD_MAPS_DIR "/%s", name);
  bool ok = sdReadMap(p);
  logEvent(ok ? "karta SD: wczytano mape %s" : "karta SD: nie udalo sie wczytac mapy %s", name);
  return ok;
}

// Nazwa z klawiatury -> bezpieczna nazwa pliku (litery, cyfry, _ i -)
static bool sdCleanName(char* out, size_t n, const char* in) {
  size_t k = 0;
  for (; *in && k + 1 < n && k < 24; in++) {
    char c = *in;
    if (c == ' ') c = '_';
    if (isalnum((unsigned char)c) || c == '_' || c == '-') out[k++] = c;
  }
  out[k] = 0;
  return k > 0;
}

// Zmiana nazwy zapisanej mapy (plik .bin i podglad .pgm). 1 = ok, 0 = blad, -1 = nazwa zajeta
int sd_renameMap(const char* oldName, const char* newBase) {
  SdSession ss; if (!ss.ok) return 0;
  char nb[28], p1[64], p2[64], q1[64], q2[64];
  if (!sdCleanName(nb, sizeof(nb), newBase)) return 0;
  size_t L = strlen(oldName);
  if (L < 5) return 0;
  snprintf(p1, sizeof(p1), SD_MAPS_DIR "/%s", oldName);
  snprintf(p2, sizeof(p2), SD_MAPS_DIR "/%.*s.pgm", (int)(L - 4), oldName);
  snprintf(q1, sizeof(q1), SD_MAPS_DIR "/%s.bin", nb);
  snprintf(q2, sizeof(q2), SD_MAPS_DIR "/%s.pgm", nb);
  if (!strcmp(p1, q1)) return 1;
  if (SD.exists(q1)) return -1;
  if (!SD.rename(p1, q1)) return 0;
  if (SD.exists(p2)) SD.rename(p2, q2);
  logEvent("karta SD: mapa %s -> %s", oldName, nb);
  return 1;
}

bool sd_deleteMap(const char* name) {
  SdSession ss; if (!ss.ok) return false;
  size_t L = strlen(name);
  if (L < 5) return false;
  char p1[64], p2[64];
  snprintf(p1, sizeof(p1), SD_MAPS_DIR "/%s", name);
  snprintf(p2, sizeof(p2), SD_MAPS_DIR "/%.*s.pgm", (int)(L - 4), name);
  bool ok = SD.remove(p1);
  SD.remove(p2);
  logEvent(ok ? "karta SD: usunieto mape %s" : "karta SD: nie usunieto mapy %s", name);
  return ok;
}

// Wczytuje zapisana mape do malego bufora (2 bity na komorke = 10 KB zamiast 40 KB, latwiej o pamiec) -
// podglad bez ruszania biezacej mapy. Odczyt komorki: mapPackedAt() z MapGrid.hpp. Zwalnia: free().
// Zwraca NULL przy braku pamieci albo blednym pliku.
PackedMap* sd_readMapCells(const char* name) {
  SdSession ss; if (!ss.ok) return NULL;
  char p[64];
  snprintf(p, sizeof(p), SD_MAPS_DIR "/%s", name);
  File f = SD.open(p, FILE_READ);
  if (!f) return NULL;
  const size_t n = (size_t)MAP_CELLS * MAP_CELLS;
  MapFileHeader h;
  PackedMap* buf = NULL;
  bool ok = f.read((uint8_t*)&h, sizeof(h)) == sizeof(h) && memcmp(h.magic, "RMAP", 4) == 0 &&
            h.cells == MAP_CELLS && h.cellCm == MAP_CELL_CM && (size_t)f.size() == sizeof(h) + n;
  if (!ok) logEvent("karta SD: podglad %s - zly naglowek/rozmiar pliku", name);
  else { buf = packedAlloc(); ok = buf != NULL; if (!ok) logEvent("karta SD: podglad - brak RAM (wolne %u B)", (unsigned)ESP.getFreeHeap()); }
  uint8_t chunk[512];
  uint32_t a = 1, b = 0;                                        // suma kontrolna liczona w locie (jak mapChecksum)
  for (size_t off = 0; ok && off < n; off += sizeof(chunk)) {
    size_t len = (n - off) < sizeof(chunk) ? (n - off) : sizeof(chunk);
    ok = f.read(chunk, len) == len;
    for (size_t i = 0; ok && i < len; i++) {
      a = (a + chunk[i]) % 65521; b = (b + a) % 65521;
      int8_t v = (int8_t)chunk[i];
      uint8_t c = v >= L_OCC_T ? 2 : (v <= L_FREE_T ? 1 : 0);
      size_t k = off + i;
      buf->row[k / MAP_CELLS][(k % MAP_CELLS) >> 2] |= c << (((k % MAP_CELLS) & 3) * 2);
    }
  }
  f.close();
  if (ok && ((b << 16) | a) != h.checksum) { ok = false; logEvent("karta SD: podglad %s - zla suma kontrolna", name); }
  if (!ok) { packedFree(buf); return NULL; }
  return buf;
}

// Wolane w loop(): nic nie zapisujemy w tle (zapisy na karte blokowaly serwer WWW). Dziennik i mape zapisuje sie recznie.
void sd_loop() {}

// Zapisuje biezacy dziennik (ostatnie wpisy tej sesji) do pliku na karcie
bool sd_saveLog() {
  SdSession ss; if (!ss.ok) return false;
  File f = SD.open(SD_LOG_FILE, FILE_WRITE);
  if (!f) return false;
  Log_dump(f);
  f.close();
  return true;
}

// Pomocnicze dla ekranu: ponowne wykrycie karty (np. po jej wlozeniu)
bool sd_recheck() { SdSession ss; return ss.ok; }               // zawsze swieze zamontowanie - dziala po wyjeciu i wlozeniu karty

// Usuwa wszystkie pliki i foldery z karty (szybkie "formatowanie"; nie naprawia i nie zmienia systemu plików FAT)
static bool sdWipeDir(const char* path, int depth) {
  for (int guard = 0; guard < 4000; guard++) {
    File d = SD.open(path);
    if (!d) return false;
    File e = d.openNextFile();
    if (!e) { d.close(); return true; }
    const char* n = e.name();
    const char* sl = strrchr(n, '/');
    if (sl) n = sl + 1;
    char full[96];
    snprintf(full, sizeof(full), "%s/%s", strcmp(path, "/") ? path : "", n);
    bool isDir = e.isDirectory();
    e.close(); d.close();
    if (isDir) { if (depth >= 6 || !sdWipeDir(full, depth + 1) || !SD.rmdir(full)) return false; }
    else if (!SD.remove(full)) return false;
  }
  return false;
}

bool sd_format() {
  SdSession ss; if (!ss.ok) return false;
  bool ok = sdWipeDir("/", 0);
  SD.mkdir(SD_DIR); SD.mkdir(SD_MAPS_DIR);
  logEvent(ok ? "karta SD: sformatowano (usunieto wszystkie pliki)" : "karta SD: formatowanie niepelne");
  return ok;
}

#else   // SD_ENABLED 0: puste zaślepki
void sd_preinit() {}
bool sd_init() { return false; }
void sd_loop() {}
bool sd_recheck() { return false; }
bool sd_saveLog() { return false; }
bool sd_format() { return false; }
bool sd_saveMapNamed() { return false; }
bool sd_restoreMap() { return false; }
int sd_listMaps(char[][32], int) { return 0; }
bool sd_loadMapFile(const char*) { return false; }
int sd_renameMap(const char*, const char*) { return 0; }
bool sd_deleteMap(const char*) { return false; }
PackedMap* sd_readMapCells(const char*) { return NULL; }
#endif
