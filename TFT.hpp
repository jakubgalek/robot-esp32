#pragma once
// ---------------------------------------------------------------------------
// Ekran dotykowy 240x320 (pionowo): ekran startowy + Ustawienia w stylu smartfona (przewijane listy):
// Wi-Fi z klawiatura ekranowa, czujniki na zywo, podglad mapy, data i czas (RTC + internet), karta SD, dotyk, informacje.
// Kolorystyka "Vista". Napisy bez polskich znakow (wbudowana czcionka ich nie ma).
// Dolaczany jako OSTATNI plik - korzysta z Webpage/Explore/Network/SDCard.
// ---------------------------------------------------------------------------
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Preferences.h>
#include <esp_ota_ops.h>

#ifndef TOUCH_CS
#error "Zdefiniuj TOUCH_CS w User_Setup.h (u Ciebie: #define TOUCH_CS 26)"
#endif

int btKeyboardState();          // z robot-esp32.ino: 1 polaczona, 0 rozlaczona, -1 BLE jeszcze nie dziala

TFT_eSPI tft = TFT_eSPI();

#define RGB565(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
// ---- motywy kolorow: 0 domyslny (niebieski "szklany"), 1 ciemny, 2 jasny
struct Theme {
  uint16_t hdr, accent, dkblue, card, grey, dgrey, edge, panel, text, sub, green, red, cyan, wall, mapFree, mapUnk;
  bool grad, mono;
};
static const Theme THEMES[2] = {
  {0x3D1F, 0x7E3F, 0x1A2F, 0x4A49, 0xA534, 0x5ACB, 0x6B6D, 0x0841, 0xFFFF, 0xFFE0, 0x07E0, 0xF800, 0x07FF, 0xFFFF, RGB565(70, 105, 150), RGB565(8, 12, 20), true, false},
  {0x3D1F, 0x1B39, 0x1A31, 0xFFFF, 0x7BCF, 0xC618, 0xBDF7, 0xEF7D, 0x1082, 0x9320, 0x0540, 0xC000, 0x0410, 0x10E7, RGB565(200, 225, 250), RGB565(245, 245, 245), false, false},
};
static const Theme* th = &THEMES[0];
static const char* const THEME_NAMES[2] = {"Ciemny", "Jasny"};
#define VISTA_BLUE (th->hdr)
#define VISTA_LIGHT_BLUE (th->accent)
#define VISTA_DARK_BLUE (th->dkblue)
#define VISTA_GLASS (th->card)
#define VISTA_WHITE 0xFFFF                  // zawsze biale: napisy na naglowku i niebieskich przyciskach
#define VISTA_TEXT (th->text)               // glowny kolor napisow na kartach i tle ekranu
#define VISTA_WALL (th->wall)
#define VISTA_GREY (th->grey)
#define VISTA_DARK_GREY (th->dgrey)
#define VISTA_GREEN (th->green)
#define VISTA_RED (th->red)
#define VISTA_YELLOW (th->sub)              // zolty: male podpisy i stany (w jasnym motywie ciemny bursztyn)
#define C_CYAN (th->cyan)
#define C_EDGE (th->edge)
#define C_PANEL (th->panel)                 // tlo ekranow z listami
// czarno-bialy: kazdy kolor zamieniany na odcien szarosci wg jasnosci
static inline uint16_t mono(uint16_t c, int pct = 100) {
  if (!th->mono) return c;
  int r = (c >> 11) << 3, g = ((c >> 5) & 63) << 2, b = (c & 31) << 3;
  int l = (r * 30 + g * 59 + b * 11) / 100 * pct / 100;
  return (uint16_t)(((l >> 3) << 11) | ((l >> 2) << 5) | (l >> 3));
}
#define BAR_YELLOW mono(0xFFE0)                 // dolna belka ekranu glownego jest ciemna w kazdym motywie
#define BAR_CYAN mono(0x07FF)

#define SCR_W 240
#define SCR_H 320
#define HDR_H 44

enum Screen : uint8_t { SC_HOME, SC_SETTINGS, SC_WIFI, SC_KEY, SC_CONNECT, SC_MAP, SC_SD, SC_SDMAPS, SC_INFO, SC_CONFIRM, SC_SENS, SC_MAPVIEW, SC_TIME, SC_CTRL, SC_SDACT, SC_THEME };
enum ConfirmKind : uint8_t { CF_NEWMAP, CF_LOADMAP, CF_FORGET, CF_RESTART, CF_DELMAP, CF_SDFORMAT };
enum RowId : uint8_t { R_NONE, R_WIFI, R_BT, R_SENS, R_SCAN, R_MAPVIEW, R_MAPMGR, R_TIME, R_SD, R_TOUCH, R_INFO, R_RESTART,
                       R_THEME, R_NTP, R_NTPNOW, R_SDLOG, R_SDBAK, R_SDFMT, R_SDLOAD, R_MSAVE, R_MNEW, R_MSD, R_CTRL, R_C_SCAN, R_C_AUTO, R_C_WALL, R_C_LOC, R_C_HOME, R_C_REF, R_LIST = 100 };

static Screen scr = SC_HOME, scrBack = SC_HOME;
static ConfirmKind confirmKind = CF_NEWMAP;
static bool forceDisplayUpdate = false;
static unsigned long tftLastUpdate = 0;
static char toastMsg[40] = "";

static uint16_t touchCal[5] = {275, 3620, 264, 3532, 1};
static bool touchCalStored = false;

bool robotBusyNow();            // Webpage.hpp
bool mapNewMap();               // Webpage.hpp
void ntp_request();             // Network.hpp
const char* ntp_status();

// ====================================================================== narzedzia rysowania
static void T(const char* s, int x, int y, uint8_t size, uint16_t fg, uint8_t datum = TL_DATUM) {
  tft.setTextDatum(datum); tft.setTextColor(fg);
#if 0   // ladniejsze fonty 2/4 wylaczone na prosbe - wraca stara czcionka
  // ladniejsza, proporcjonalna czcionka (Helvetica-like) zamiast powiekszanego 6x8: rozmiar 2 -> font 2 (16 px), 3 -> font 4 (26 px).
  // Napisy rozmiaru 1 zostaja malym fontem 6x8 (mieszcza sie w ciasnych miejscach). Wymaga LOAD_FONT2/4 w User_Setup.h.
  if (size == 2) { tft.setTextSize(1); tft.drawString(s, x, y, 2); return; }
  if (size == 3) { tft.setTextSize(1); tft.drawString(s, x, y, 4); return; }
#endif
  tft.setTextSize(size);
  tft.drawString(s, x, y, 1);
}

static void fit(char* dst, size_t n, const char* src, size_t maxChars) {
  size_t L = strlen(src);
  if (maxChars >= n) maxChars = n - 1;
  if (L <= maxChars) { memcpy(dst, src, L + 1); return; }
  if (maxChars < 3) { dst[0] = 0; return; }
  memcpy(dst, src, maxChars - 2); dst[maxChars - 2] = '.'; dst[maxChars - 1] = '.'; dst[maxChars] = 0;
}

// przewijany napis (czcionka stalej szerokosci): dlugie nazwy przesuwaja sie powoli, z przerwa na poczatku i na koncu
static void marquee(const char* s, int x, int y, int maxChars, uint8_t size, uint16_t fg, uint16_t bg) {
  int L = strlen(s), off = 0;
  if (L > maxChars) {
    int extra = L - maxChars, step = (millis() / 500) % (extra + 6);
    off = step < 3 ? 0 : (step - 3 > extra ? extra : step - 3);
  }
  char b[34]; int n = L - off < maxChars ? L - off : maxChars; if (n > 33) n = 33;
  memcpy(b, s + off, n); b[n] = 0;
  tft.fillRect(x, y - 4 * size, maxChars * 6 * size, 8 * size, bg);
  T(b, x, y, size, fg, ML_DATUM);
}

static inline bool inRect(int px, int py, int x, int y, int w, int h) { return px >= x && px < x + w && py >= y && py < y + h; }
static inline int sigBars(int rssi) { return rssi >= -55 ? 4 : rssi >= -65 ? 3 : rssi >= -75 ? 2 : 1; }

static void drawBackground() {
  if (!th->grad) { tft.fillScreen(th->panel); return; }
  for (int band = 0; band < 16; band++) tft.fillRect(0, band * 20, SCR_W, 20, 0x0800 + band);
}

static void drawCard(int x, int y, int w, int h) {
  tft.fillRoundRect(x, y, w, h, 8, VISTA_GLASS);
  tft.drawRoundRect(x, y, w, h, 8, C_EDGE);
}

// 0 glowny (niebieski), 1 ostrzegawczy (czerwony), 2 neutralny, 3 nieaktywny
static void drawButton(int x, int y, int w, int h, const char* label, uint8_t style = 0, uint8_t size = 2) {
  uint16_t bg = style == 0 ? VISTA_BLUE : style == 1 ? mono(0xA000) : style == 3 ? VISTA_DARK_GREY : VISTA_GLASS;
  uint16_t fg = style == 3 ? VISTA_GREY : style == 2 ? VISTA_TEXT : VISTA_WHITE;
  tft.fillRoundRect(x, y, w, h, 6, bg);
  tft.drawRoundRect(x, y, w, h, 6, style == 3 ? VISTA_GREY : VISTA_LIGHT_BLUE);
  T(label, x + w / 2, y + h / 2 + 1, size, fg, MC_DATUM);
}

static void drawBackIcon(int x, int y, uint16_t color) {          // x, y = lewy gorny rog przycisku (74 x 36)
  int cy = y + 18, x0 = x + 16;
  tft.fillRect(x0 + 2, cy - 2, 36, 5, color);                       // trzon
  for (int i = 0; i < 13; i++) tft.drawFastVLine(x0 + i, cy - i, i * 2 + 1, color);   // grot
}

static void drawHeader(const char* title, bool back) {
  tft.fillRect(0, 0, SCR_W, HDR_H, VISTA_BLUE);
  tft.drawFastHLine(0, 0, SCR_W, VISTA_WHITE);
  tft.drawFastHLine(0, HDR_H - 1, SCR_W, VISTA_LIGHT_BLUE);
  if (back) {
    uint16_t bb = th->card == 0xFFFF ? VISTA_DARK_BLUE : VISTA_GLASS;      // w jasnym motywie karta jest biala, a strzalka tez
    tft.fillRoundRect(4, 4, 74, 36, 7, bb);
    tft.drawRoundRect(4, 4, 74, 36, 7, th->card == 0xFFFF ? VISTA_WHITE : VISTA_LIGHT_BLUE);
    drawBackIcon(4, 4, VISTA_WHITE);
  }
  T(title, 160, HDR_H / 2, 2, VISTA_WHITE, MC_DATUM);                     // podwojony z przesunieciem o 1 px = gruby napis
  T(title, 161, HDR_H / 2, 2, VISTA_WHITE, MC_DATUM);
}

// zebatka ze srodkiem w (cx, cy): pelny korpus z zebami i otworem w kolorze tla kafelka
static void drawGear(int cx, int cy, uint16_t color, uint16_t hole) {
  for (int i = 0; i < 8; i++) {
    float a = i * 45 * PI / 180, ca = cosf(a), sa = sinf(a);
    tft.fillCircle(cx + (int)lroundf(ca * 10), cy + (int)lroundf(sa * 10), 2, color);
    tft.drawLine(cx + (int)lroundf(ca * 6), cy + (int)lroundf(sa * 6), cx + (int)lroundf(ca * 10), cy + (int)lroundf(sa * 10), color);
  }
  tft.fillCircle(cx, cy, 8, color);
  tft.fillCircle(cx, cy, 3, hole);
}

static void drawBatteryIcon(int x, int y, float voltage, bool isLow) {
  uint16_t c = isLow ? VISTA_RED : VISTA_GREEN;
  tft.drawRect(x, y + 2, 20, 10, c);
  tft.fillRect(x + 20, y + 4, 2, 6, c);
  int w = isLow ? 4 : (voltage >= 7.0 ? 16 : voltage >= 6.0 ? 12 : voltage >= 5.5 ? 8 : 4);
  tft.fillRect(x + 2, y + 4, w, 6, c);
}

static void drawBarsN(int x, int y, int bars) {                    // 4 slupki zasiegu, y = dol
  for (int i = 0; i < 4; i++) {
    int h = 4 + i * 4;
    tft.fillRect(x + i * 6, y - h, 4, h, i < bars ? VISTA_GREEN : VISTA_DARK_GREY);
  }
}
static void drawBars(int x, int y, int rssi, bool connected) { drawBarsN(x, y, connected ? sigBars(rssi) : 0); }

static void drawLock(int x, int y, uint16_t c) {
  tft.drawRoundRect(x + 2, y, 6, 6, 2, c);
  tft.fillRect(x, y + 5, 10, 7, c);
}

static void drawBtGlyph(int cx, int cy, int s, uint16_t c) {
  tft.drawLine(cx, cy - s, cx, cy + s, c);
  tft.drawLine(cx, cy - s, cx + s * 2 / 3, cy - s / 3, c);
  tft.drawLine(cx + s * 2 / 3, cy - s / 3, cx - s * 2 / 3, cy + s * 2 / 3, c);
  tft.drawLine(cx, cy + s, cx + s * 2 / 3, cy + s / 3, c);
  tft.drawLine(cx + s * 2 / 3, cy + s / 3, cx - s * 2 / 3, cy - s * 2 / 3, c);
}

static void drawSdGlyph(int x, int y, uint16_t c) {                  // karta 12x16
  tft.drawRect(x, y + 3, 12, 13, c);
  tft.fillTriangle(x, y + 3, x + 4, y + 3, x, y, c);
  tft.fillRect(x + 3, y + 5, 2, 4, c); tft.fillRect(x + 6, y + 5, 2, 4, c); tft.fillRect(x + 9, y + 5, 2, 4, c);
}

enum IconKind : int8_t { I_NONE = -1, I_WIFI, I_BT, I_SCAN, I_MAP, I_SD, I_TOUCH, I_INFO, I_SAVE, I_TRASH, I_LOAD, I_SENS, I_CLOCK, I_POWER, I_VIEW, I_HOME };

// Kolorowy kwadrat z symbolem 28x28 - jak ikony w ustawieniach telefonu
static void drawIcon(int8_t kind, int x, int y) {
  static const uint16_t col[] = {0x3D1F, 0x1C9F, 0x0600, 0xFC00, 0x7BEF, 0xA01F, 0x5AEB, 0x2D6B, 0xC000, 0x2D6B, 0xF81F - 0x3800, 0xFB00, 0xC000, 0x0410, 0x2D6B};
  tft.fillRoundRect(x, y, 28, 28, 7, mono(col[kind], 75));
  int cx = x + 14, cy = y + 14;
  uint16_t w = VISTA_WHITE;
  switch (kind) {
    case I_WIFI: tft.fillRect(cx - 8, cy + 3, 4, 5, w); tft.fillRect(cx - 2, cy - 1, 4, 9, w); tft.fillRect(cx + 4, cy - 6, 4, 14, w); break;
    case I_BT: drawBtGlyph(cx, cy, 9, w); break;
    case I_SCAN: tft.drawCircle(cx, cy, 9, w); tft.drawCircle(cx, cy, 4, w); tft.drawLine(cx, cy, cx + 8, cy - 6, w); break;
    case I_MAP: tft.drawRect(cx - 9, cy - 9, 18, 18, w); tft.drawFastVLine(cx, cy - 9, 18, w); tft.drawFastHLine(cx - 9, cy, 18, w); tft.fillRect(cx + 2, cy + 2, 5, 5, w); break;
    case I_SD: drawSdGlyph(cx - 6, cy - 8, w); break;
    case I_TOUCH: tft.drawCircle(cx, cy, 9, w); tft.fillCircle(cx, cy, 3, w); tft.drawFastHLine(cx - 12, cy, 5, w); tft.drawFastHLine(cx + 8, cy, 5, w); break;
    case I_INFO: T("i", cx, cy + 1, 3, w, MC_DATUM); break;
    case I_SAVE: tft.drawRect(cx - 8, cy - 8, 16, 16, w); tft.fillRect(cx - 4, cy - 8, 8, 6, w); tft.drawRect(cx - 4, cy + 1, 8, 7, w); break;
    case I_TRASH: tft.drawRect(cx - 6, cy - 4, 12, 12, w); tft.drawFastHLine(cx - 8, cy - 5, 16, w); tft.drawFastVLine(cx - 2, cy - 1, 7, w); tft.drawFastVLine(cx + 2, cy - 1, 7, w); break;
    case I_LOAD: tft.drawRect(cx - 8, cy - 2, 16, 10, w); tft.drawFastVLine(cx, cy - 9, 9, w); tft.fillTriangle(cx - 4, cy - 3, cx + 4, cy - 3, cx, cy + 2, w); break;
    case I_SENS: tft.drawRect(cx - 10, cy - 5, 7, 10, w); tft.drawLine(cx - 2, cy, cx + 9, cy - 7, w); tft.drawLine(cx - 2, cy, cx + 10, cy, w); tft.drawLine(cx - 2, cy, cx + 9, cy + 7, w); break;   // czujnik z wiazka
    case I_CLOCK: tft.drawCircle(cx, cy, 9, w); tft.drawLine(cx, cy, cx, cy - 6, w); tft.drawLine(cx, cy, cx + 4, cy + 2, w); break;
    case I_POWER: tft.drawCircle(cx, cy + 1, 8, w); tft.fillRect(cx - 1, cy - 10, 3, 9, w); break;
    case I_HOME: tft.fillTriangle(cx - 10, cy, cx + 10, cy, cx, cy - 9, w); tft.drawRect(cx - 7, cy, 14, 9, w); tft.fillRect(cx - 2, cy + 3, 4, 6, w); break;
    case I_VIEW: tft.drawCircle(cx - 2, cy - 2, 6, w); tft.drawLine(cx + 2, cy + 2, cx + 8, cy + 8, w); tft.drawLine(cx + 3, cy + 2, cx + 9, cy + 8, w); break;
  }
}

static void drawChevron(int x, int y, uint16_t c) {
  tft.drawLine(x, y - 6, x + 5, y, c); tft.drawLine(x + 5, y, x, y + 6, c);
  tft.drawLine(x + 1, y - 6, x + 6, y, c); tft.drawLine(x + 6, y, x + 1, y + 6, c);
}

static void drawSwitch(int x, int y, bool on) {                      // 38x20
  tft.fillRoundRect(x, y, 38, 20, 10, on ? VISTA_GREEN : VISTA_DARK_GREY);
  tft.fillCircle(on ? x + 28 : x + 10, y + 10, 8, VISTA_WHITE);
}

// ====================================================================== przewijana lista wierszy
// kind: 0 zwykly, 1 strzalka, 2 przelacznik, 3 naglowek sekcji, 5 siec Wi-Fi (slupki + klodka)
struct Row { const char* title; char sub[30]; int8_t icon; uint8_t kind; bool on; uint16_t tcol; uint8_t id; uint8_t h; int8_t bars; bool lock; };
#define ROWS_MAX 18
static Row rows[ROWS_MAX];
static int rowOff[ROWS_MAX + 1];
static int rowsN = 0;
static int listTop = HDR_H, listBot = SCR_H, scrollY = 0, scrollMax = 0;
static const char* emptyMsg = NULL;
static int RW = SCR_W;                      // szerokosc wierszy (z przyciskami przewijania jest wezsza)
#define SCROLL_STEP 88                       // o ile przesuwa jedno dotkniecie strzalki

static void rowsReset(int top) { rowsN = 0; listTop = top; listBot = SCR_H; emptyMsg = NULL; rowOff[0] = 0; }

static Row& rowAdd(uint8_t id, const char* title, const char* sub, int8_t icon, uint8_t kind, bool on = false, uint16_t tcol = VISTA_TEXT) {
  Row& r = rows[rowsN];
  r.title = title; strncpy(r.sub, sub ? sub : "", sizeof(r.sub) - 1); r.sub[sizeof(r.sub) - 1] = 0;
  r.icon = icon; r.kind = kind; r.on = on; r.tcol = tcol; r.id = id; r.bars = 0; r.lock = false;
  r.h = kind == 3 ? 24 : kind == 5 ? 40 : 42;
  rowOff[rowsN + 1] = rowOff[rowsN] + r.h;
  rowsN++;
  return r;
}

static void rowsFinish() {
  int viewH = listBot - listTop;
  int need = rowOff[rowsN] > viewH ? rowOff[rowsN] - viewH : 0;
  scrollMax = need;
  for (int i = 0; i <= rowsN && need > 0; i++) if (rowOff[i] >= need) { scrollMax = rowOff[i]; break; }   // przewijanie tylko o cale wiersze
  RW = scrollMax > 0 ? 210 : SCR_W;
  if (scrollY > scrollMax) scrollY = scrollMax;
  if (scrollY < 0) scrollY = 0;
  int sn = 0; for (int i = 0; i <= rowsN; i++) if (rowOff[i] <= scrollY) sn = rowOff[i];
  scrollY = sn;
}

static void drawRow(int i, int y) {
  Row& r = rows[i];
  if (r.kind == 3) {
    tft.fillRect(0, y, RW, r.h, C_PANEL);
    T(r.title, 12, y + r.h - 8, 1, VISTA_LIGHT_BLUE, ML_DATUM);
    return;
  }
  tft.fillRect(0, y, RW, r.h, VISTA_GLASS);
  int tx = 12;
  if (r.kind == 5) {
    drawBarsN(12, y + 28, r.bars);
    marquee(r.title, 46, y + r.h / 2, (RW - 76) / 12, 2, VISTA_TEXT, VISTA_GLASS);
    if (r.lock) drawLock(RW - 26, y + 14, VISTA_GREY);
    tft.drawFastHLine(46, y + r.h - 1, RW - 46, VISTA_DARK_GREY);
    return;
  }
  if (r.kind == 6) {                                          // wybor jednej opcji: pusta kropka, a przy wybranej wypelniona
    T(r.title, 14, y + r.h / 2, 2, VISTA_TEXT, ML_DATUM);
    tft.drawCircle(RW - 26, y + r.h / 2, 11, VISTA_TEXT); tft.drawCircle(RW - 26, y + r.h / 2, 10, VISTA_TEXT);
    if (r.on) tft.fillCircle(RW - 26, y + r.h / 2, 6, VISTA_TEXT);
    tft.drawFastHLine(12, y + r.h - 1, RW - 12, VISTA_DARK_GREY);
    return;
  }
  if (r.icon != I_NONE) { drawIcon(r.icon, 10, y + (r.h - 28) / 2); tx = 48; }
  if (r.sub[0]) {
    T(r.title, tx, y + 5, 2, mono(r.tcol));
    T(r.sub, tx, y + 27, 1, VISTA_YELLOW);
  } else T(r.title, tx, y + r.h / 2, 2, mono(r.tcol), ML_DATUM);
  if (r.kind == 1) drawChevron(RW - 18, y + r.h / 2, VISTA_GREY);
  else if (r.kind == 2) drawSwitch(RW - 44, y + r.h / 2 - 10, r.on);
  tft.drawFastHLine(tx, y + r.h - 1, RW - tx, VISTA_DARK_GREY);
}

static void drawChrome();       // gora ekranu (naglowek, karty) - zalezy od ekranu

static void drawList() {
  int base = listTop - scrollY;
  for (int i = 0; i < rowsN; i++) {
    int y = base + rowOff[i];
    if (y + rows[i].h <= listTop || y >= listBot) continue;
    drawRow(i, y);
  }
  int endY = base + rowOff[rowsN];
  if (endY < listBot) {
    int y0 = endY > listTop ? endY : listTop;
    tft.fillRect(0, y0, RW, listBot - y0, C_PANEL);
  }
  if (rowsN == 0 && emptyMsg) T(emptyMsg, SCR_W / 2, listTop + 60, 2, VISTA_YELLOW, MC_DATUM);
  if (scrollMax > 0) {                                    // przyciski przewijania (zamiast przeciagania palcem)
    int mid = (listTop + listBot) / 2;
    bool canUp = scrollY > 0, canDn = scrollY < scrollMax;
    tft.fillRect(RW, listTop, SCR_W - RW, listBot - listTop, C_PANEL);
    tft.fillRoundRect(RW + 2, listTop + 4, SCR_W - RW - 4, mid - listTop - 8, 6, canUp ? VISTA_BLUE : VISTA_DARK_GREY);
    tft.fillRoundRect(RW + 2, mid + 4, SCR_W - RW - 4, listBot - mid - 8, 6, canDn ? VISTA_BLUE : VISTA_DARK_GREY);
    int cx = RW + (SCR_W - RW) / 2, uy = (listTop + mid) / 2, dy = (mid + listBot) / 2;
    uint16_t cu = canUp ? VISTA_WHITE : VISTA_GREY, cd = canDn ? VISTA_WHITE : VISTA_GREY;
    tft.fillTriangle(cx - 8, uy + 5, cx + 8, uy + 5, cx, uy - 6, cu);
    tft.fillTriangle(cx - 8, dy - 5, cx + 8, dy - 5, cx, dy + 6, cd);
  }
  drawChrome();
}

static void scrollTo(int v) {
  if (v < 0) v = 0;
  if (v > scrollMax) v = scrollMax;
  int sn = 0, bd = 9999;                                  // zawsze na granicy wiersza, zeby zaden nie wchodzil pod naglowek
  for (int i = 0; i <= rowsN; i++) { int d = abs(rowOff[i] - v); if (d < bd) { bd = d; sn = rowOff[i]; } }
  v = sn;
  if (v == scrollY) return;
  scrollY = v;
  drawList();
}

static int rowHit(int px, int py) {
  if (py < listTop || py >= listBot || px >= RW) return -1;
  int rel = py - listTop + scrollY;
  for (int i = 0; i < rowsN; i++) if (rel >= rowOff[i] && rel < rowOff[i + 1]) return rows[i].kind == 3 ? -1 : i;
  return -1;
}

// Dotkniecie przycisku przewijania (prawy pasek listy). Zwraca true, gdy obsluzone.
static bool listScrollTap(int tx, int ty) {
  if (scrollMax <= 0 || tx < RW || ty < listTop || ty >= listBot) return false;
  scrollTo(ty < (listTop + listBot) / 2 ? scrollY - SCROLL_STEP : scrollY + SCROLL_STEP);
  return true;
}

// ====================================================================== dane pomocnicze
static void fmtSize(char* out, size_t n, uint32_t mb) {
  if (mb >= 1024) snprintf(out, n, "%.1f GB", mb / 1024.0f); else snprintf(out, n, "%u MB", (unsigned)mb);
}

static float mapAreaM2() {
  uint32_t o, f; gridMap.stats(o, f);
  return f * (MAP_CELL_CM / 100.0f) * (MAP_CELL_CM / 100.0f);
}

static void showToast(const char* m) { strncpy(toastMsg, m, sizeof(toastMsg) - 1); toastMsg[sizeof(toastMsg) - 1] = 0; }

static void drawToastLine(int y) {
  tft.fillRect(0, y, SCR_W, 20, C_PANEL);
  if (toastMsg[0]) T(toastMsg, SCR_W / 2, y + 10, 1, VISTA_YELLOW, MC_DATUM);
}

static bool clockValid() { return dt.year >= 2024; }

// ====================================================================== mapa (miniatura i pelny podglad)
static float mapPxPerCm = 0;
static const PackedMap* mapSrc = NULL;     // NULL = biezaca mapa; inaczej bufor podgladanej mapy z karty (2 bity/komorka)
static PackedMap* mapPrevBuf = NULL;
static const uint16_t MAP_ZOOMS[8] = {25, 40, 60, 100, 200, 300, 400, 600};   // % (100 = cala zbadana czesc)
#define MAP_ZOOM_DEF 3
static int mapZoomI = MAP_ZOOM_DEF, mapPanX = 0, mapPanY = 0, mapViewCols = 40, mapViewRows = 40;
static void mapPreviewFree() { if (mapPrevBuf) { packedFree(mapPrevBuf); mapPrevBuf = NULL; } mapSrc = NULL; }       // skala ostatnio narysowanej mapy (do paska skali)
#define MAP_UNKNOWN (th->mapUnk)
#define MAP_FREE (th->mapFree)

// Rysuje mape dopasowana do prostokata. Zwraca false, gdy mapa zajeta/niedostepna (wtedy nic nie rysuje).
static bool mapRender(int ax, int ay, int aw, int ah, bool big) {
  if (!mapSrc && !gridMap.cells) return false;
  auto at = [&](size_t i) -> int8_t { return mapSrc ? mapPackedAt(mapSrc, i) : gridMap.cells[i]; };
  int bx0 = MAP_CELLS, bx1 = -1, by0 = MAP_CELLS, by1 = -1;
  for (int cy = 0; cy < MAP_CELLS; cy++) {
    MapLock lk(mapSrc ? 0 : 10);
    if (!mapSrc && !lk.ok()) return false;
    for (int cx = 0; cx < MAP_CELLS; cx++) {
      int8_t v = at((size_t)cy * MAP_CELLS + cx);
      if (v >= L_OCC_T || v <= L_FREE_T) { if (cx < bx0) bx0 = cx; if (cx > bx1) bx1 = cx; if (cy < by0) by0 = cy; if (cy > by1) by1 = cy; }
    }
  }
  Pose2 rp = odometry_get();
  int rcx = gridMap.toCell(rp.x), rcy = gridMap.toCell(rp.y), ocx = gridMap.toCell(0), ocy = gridMap.toCell(0);
  int pts[2][2] = {{rcx, rcy}, {ocx, ocy}};
  for (int k = 0; k < (mapSrc ? 0 : 2); k++) {
    if (pts[k][0] < bx0) bx0 = pts[k][0]; if (pts[k][0] > bx1) bx1 = pts[k][0];
    if (pts[k][1] < by0) by0 = pts[k][1]; if (pts[k][1] > by1) by1 = pts[k][1];
  }
  bx0 -= 3; by0 -= 3; bx1 += 3; by1 += 3;
  if (bx0 < 0) bx0 = 0; if (by0 < 0) by0 = 0; if (bx1 >= MAP_CELLS) bx1 = MAP_CELLS - 1; if (by1 >= MAP_CELLS) by1 = MAP_CELLS - 1;
  int bw = bx1 - bx0 + 1, bh = by1 - by0 + 1;
  if (bw < 30) { bx0 -= (30 - bw) / 2; bw = 30; if (bx0 < 0) bx0 = 0; if (bx0 + bw > MAP_CELLS) bx0 = MAP_CELLS - bw; }
  if (bh < 30) { by0 -= (30 - bh) / 2; bh = 30; if (by0 < 0) by0 = 0; if (by0 + bh > MAP_CELLS) by0 = MAP_CELLS - bh; }
  by1 = by0 + bh - 1;
  int px = 1, cols = bw, rws = bh;                       // powiekszenie calkowite, a gdy mapa nie miesci sie - zmniejszenie (probkowanie)
  int zp = big ? MAP_ZOOMS[mapZoomI] : 100;
  if (big && zp < 100) {                                 // oddalenie: okno wieksze niz zbadana czesc (do calej siatki)
    int nbw = bw * 100 / zp, nbh = bh * 100 / zp;
    if (nbw > MAP_CELLS) nbw = MAP_CELLS; if (nbh > MAP_CELLS) nbh = MAP_CELLS;
    int ccx = (bx0 + bx1) / 2 + mapPanX, ccy = (by0 + by1) / 2 + mapPanY;
    if (ccx < nbw / 2) ccx = nbw / 2; if (ccx > MAP_CELLS - nbw / 2) ccx = MAP_CELLS - nbw / 2;
    if (ccy < nbh / 2) ccy = nbh / 2; if (ccy > MAP_CELLS - nbh / 2) ccy = MAP_CELLS - nbh / 2;
    mapPanX = ccx - (bx0 + bx1) / 2; mapPanY = ccy - (by0 + by1) / 2;
    bx0 = ccx - nbw / 2; by0 = ccy - nbh / 2; bw = nbw; bh = nbh; bx1 = bx0 + bw - 1; by1 = by0 + bh - 1;
    mapViewCols = bw; mapViewRows = bh;
  }
  if (big && zp >= 100) {                                 // przyblizenie: okno o srodku przesunietym o (mapPanX, mapPanY)
    float fitS = (float)aw / bw < (float)ah / bh ? (float)aw / bw : (float)ah / bh;
    px = (int)(fitS * zp / 100.0f); if (px < 2) px = 2;
    int vc = aw / px, vr = ah / px; if (vc > MAP_CELLS) vc = MAP_CELLS; if (vr > MAP_CELLS) vr = MAP_CELLS;
    int ccx = (bx0 + bx1) / 2 + mapPanX, ccy = (by0 + by1) / 2 + mapPanY;
    if (ccx < vc / 2) ccx = vc / 2; if (ccx > MAP_CELLS - vc / 2) ccx = MAP_CELLS - vc / 2;
    if (ccy < vr / 2) ccy = vr / 2; if (ccy > MAP_CELLS - vr / 2) ccy = MAP_CELLS - vr / 2;
    mapPanX = ccx - (bx0 + bx1) / 2; mapPanY = ccy - (by0 + by1) / 2;
    bx0 = ccx - vc / 2; if (bx0 < 0) bx0 = 0; if (bx0 + vc > MAP_CELLS) bx0 = MAP_CELLS - vc;
    by0 = ccy - vr / 2; if (by0 < 0) by0 = 0; if (by0 + vr > MAP_CELLS) by0 = MAP_CELLS - vr;
    bw = vc; bh = vr; by1 = by0 + bh - 1; cols = bw; rws = bh;
    mapViewCols = vc; mapViewRows = vr;
  }
  else if (bw <= aw && bh <= ah) { px = aw / bw < ah / bh ? aw / bw : ah / bh; }
  else { float sc = (float)aw / bw < (float)ah / bh ? (float)aw / bw : (float)ah / bh; cols = (int)(bw * sc); rws = (int)(bh * sc); if (cols < 1) cols = 1; if (rws < 1) rws = 1; }
  int dw = cols * px, dh = rws * px;
  int ox = ax + (aw - dw) / 2, oy = ay + (ah - dh) / 2;
  mapPxPerCm = (float)(cols * px) / (bw * MAP_CELL_CM);
  if (oy > ay) { tft.fillRect(ax, ay, aw, oy - ay, MAP_UNKNOWN); }
  if (oy + dh < ay + ah) tft.fillRect(ax, oy + dh, aw, ay + ah - (oy + dh), MAP_UNKNOWN);
  if (ox > ax) tft.fillRect(ax, oy, ox - ax, dh, MAP_UNKNOWN);
  if (ox + dw < ax + aw) tft.fillRect(ox + dw, oy, ax + aw - (ox + dw), dh, MAP_UNKNOWN);

  static uint8_t cls[MAP_CELLS];
  for (int r = 0; r < rws; r++) {
    int cy = by1 - r * bh / rws;
    {
      MapLock lk(mapSrc ? 0 : 10);
      if (!mapSrc && !lk.ok()) return false;
      for (int c = 0; c < cols; c++) { int8_t v = at((size_t)cy * MAP_CELLS + bx0 + c * bw / cols); cls[c] = v >= L_OCC_T ? 2 : (v <= L_FREE_T ? 1 : 0); }
    }
    int c = 0;
    while (c < cols) {
      int e = c + 1;
      while (e < cols && cls[e] == cls[c]) e++;
      tft.fillRect(ox + c * px, oy + r * px, (e - c) * px, px, cls[c] == 2 ? VISTA_WALL : cls[c] == 1 ? MAP_FREE : MAP_UNKNOWN);
      c = e;
    }
  }
  // punkt startu i robot (tylko dla biezacej mapy i tylko gdy w widoku)
  if (!mapSrc) {
    if (ocx >= bx0 && ocx <= bx1 && ocy >= by0 && ocy <= by1) {
      int stx = ox + (ocx - bx0) * cols / bw * px + px / 2, sty = oy + (by1 - ocy) * rws / bh * px + px / 2;
      tft.drawCircle(stx, sty, big ? 4 : 2, C_CYAN);
    }
    if (rcx >= bx0 && rcx <= bx1 && rcy >= by0 && rcy <= by1) {
      int rx = ox + (rcx - bx0) * cols / bw * px + px / 2, ry = oy + (by1 - rcy) * rws / bh * px + px / 2;
      float hr = rp.h * PI / 180.0f;
      int rr = big ? 4 : 2, hl = big ? 11 : 6;
      tft.fillCircle(rx, ry, rr, VISTA_GREEN);
      tft.drawLine(rx, ry, rx + (int)(cosf(hr) * hl), ry - (int)(sinf(hr) * hl), VISTA_YELLOW);
    }
  }
  return true;
}

static int mapSig() {                                  // zmienia sie, gdy mapa albo robot sie zmienily
  Pose2 p = odometry_get();
  return (int)(gridMap.version * 31u) ^ ((int)(p.x / 4) * 7919) ^ ((int)(p.y / 4) * 104729) ^ ((int)(p.h / 8) * 13);
}

// ====================================================================== EKRAN STARTOWY
#define FIELD_SLOTS 16
static char fieldCache[FIELD_SLOTS][24];
static int thumbSig = 0;
static bool thumbDirty = true;

static void fieldClear() { for (int i = 0; i < FIELD_SLOTS; i++) fieldCache[i][0] = 0; thumbDirty = true; }

// slot, prostokat pola, tlo, kolor, rozmiar, datum (TL/ML/MC/MR) - przerysowuje tylko po zmianie napisu
static void field(int slot, int x, int y, int w, int h, uint16_t bg, uint16_t fg, uint8_t size, uint8_t datum, const char* s) {
  if (strncmp(fieldCache[slot], s, 23) == 0) return;
  strncpy(fieldCache[slot], s, 23); fieldCache[slot][23] = 0;
  tft.fillRect(x, y, w, h, bg);
  int tx = (datum == TL_DATUM || datum == ML_DATUM) ? x : datum == MR_DATUM ? x + w : x + w / 2;
  int ty = datum == TL_DATUM ? y : y + h / 2;
  T(s, tx, ty, size, fg, datum);
}

static void homeDraw() {
  drawBackground();
  tft.fillRect(0, 0, SCR_W, 44, VISTA_BLUE);                                // gorna belka: godzina po lewej, ikony i ustawienia po prawej
  tft.drawFastHLine(0, 0, SCR_W, VISTA_WHITE);
  tft.drawFastHLine(0, 43, SCR_W, VISTA_LIGHT_BLUE);
  tft.fillRoundRect(195, 6, 41, 33, 6, 0x1082);                             // srodek kafelka: (215, 22)
  tft.drawRoundRect(195, 6, 41, 33, 6, VISTA_LIGHT_BLUE);
  drawGear(215, 22, VISTA_WHITE, 0x1082);

  drawCard(8, 52, 224, 72);                                                 // zasilanie
  T("LOGIKA", 64, 68, 2, VISTA_LIGHT_BLUE, MC_DATUM);
  T("SILNIKI", 176, 68, 2, VISTA_LIGHT_BLUE, MC_DATUM);
  tft.drawFastVLine(120, 58, 60, VISTA_DARK_GREY);

  tft.fillRoundRect(45, 132, 150, 104, 10, VISTA_GLASS);                    // kafelek mapy
  tft.drawRoundRect(45, 132, 150, 104, 10, VISTA_LIGHT_BLUE);
  T("MAPA", 120, 143, 2, VISTA_TEXT, MC_DATUM);
  drawButton(30, 244, 180, 30, "STEROWANIE", 0, 2);

  tft.fillRect(0, 282, SCR_W, 38, VISTA_DARK_BLUE);                         // dolna belka: adres IP + temperatura (dotkniecie = Wi-Fi)
  tft.drawFastHLine(0, 282, SCR_W, VISTA_LIGHT_BLUE);
  fieldClear();
}

static unsigned long homeDateUntil = 0;
static void drawHomeDate() {                                  // godzina (bez sekund) i data w gornej belce
  char b[24];
  tft.fillRect(0, 1, 190, 42, VISTA_BLUE);
  if (clockValid()) {
    snprintf(b, sizeof(b), "%02d:%02d", dt.hour, dt.minute); T(b, 95, 13, 2, VISTA_WHITE, MC_DATUM);
    snprintf(b, sizeof(b), "%02d.%02d.%04d", dt.day, dt.month, dt.year); T(b, 95, 31, 2, VISTA_WHITE, MC_DATUM);
  } else T("Czas nie ustawiony", 95, 22, 1, VISTA_WHITE, MC_DATUM);
}
static void homeUpdate(bool force) {
  if (force) { fieldClear(); homeDateUntil = 0; }
  if (homeDateUntil && (long)(millis() - homeDateUntil) > 0) { homeDateUntil = 0; tft.fillRect(0, 1, 190, 42, VISTA_BLUE); fieldClear(); }
  else if (homeDateUntil) drawHomeDate();
  char b[32];
  // gorna belka: godzina (lewo); zasieg Wi-Fi, Bluetooth, karta SD (przy ustawieniach)
  if (clockValid()) snprintf(b, sizeof(b), "%02d:%02d", dt.hour, dt.minute); else strcpy(b, "--:--");
  if (!homeDateUntil) field(0, 6, 4, 92, 36, VISTA_BLUE, VISTA_WHITE, 3, ML_DATUM, b);
  bool wifi = WiFi.status() == WL_CONNECTED;
  int rssi = wifi ? WiFi.RSSI() : -100;
  int bt = btKeyboardState();
  snprintf(b, sizeof(b), "%d%d%d", wifi ? sigBars(rssi) : 0, bt, (int)sdOk);
  if (!homeDateUntil && strcmp(fieldCache[5], b) != 0) {
    strcpy(fieldCache[5], b);
    bool light = (th == &THEMES[1]);
    uint16_t okCol  = light ? 0x07E0 : VISTA_GREEN;     // jaskrawa zielen w jasnym motywie
    uint16_t offCol = light ? 0xE71C : VISTA_GREY;      // wyrazniejszy jasny szary na niebieskim tle
    uint16_t errCol = light ? 0xF800 : VISTA_RED;       // jaskrawa czerwien w jasnym motywie
    uint16_t dimBar = light ? 0x1993 : VISTA_DARK_GREY; // nieaktywne slupki sygnalu

    tft.fillRect(100, 1, 92, 42, VISTA_BLUE);                              // kolejnosc: SD, Bluetooth, Wi-Fi (przy zebatce)
    drawSdGlyph(126, 14, sdOk ? okCol : offCol);
    drawBtGlyph(152, 22, 9, bt == 1 ? okCol : bt == 0 ? errCol : offCol);
    int bars = wifi ? sigBars(rssi) : 0;
    for (int i = 0; i < 4; i++) {
      int h = 4 + i * 4;
      tft.fillRect(168 + i * 6, 30 - h, 4, h, i < bars ? okCol : dimBar);
    }
  }

  // zasilanie
  float vE = readVoltageESP32(), vM = readVoltageMotors();
  snprintf(b, sizeof(b), "%.2fV", vE);
  if (strcmp(fieldCache[2], b) != 0) {
    strcpy(fieldCache[2], b); bool low = vE < 5.0;
    tft.fillRect(12, 80, 104, 42, VISTA_GLASS);
    T(b, 64, 93, 3, low ? VISTA_RED : VISTA_GREEN, MC_DATUM);
    drawBatteryIcon(53, 107, vE, low);
  }
  snprintf(b, sizeof(b), "%.2fV", vM);
  if (strcmp(fieldCache[3], b) != 0) {
    strcpy(fieldCache[3], b); bool low = vM < 5.0;
    tft.fillRect(124, 80, 104, 42, VISTA_GLASS);
    T(b, 176, 93, 3, low ? VISTA_RED : VISTA_GREEN, MC_DATUM);
    drawBatteryIcon(165, 107, vM, low);
  }

  // kafelek mapy: miniatura (tylko po zmianie) i powierzchnia
  int sig = mapSig();
  if (thumbDirty || sig != thumbSig) {
    if (mapRender(53, 158, 134, 54, false)) { thumbSig = sig; thumbDirty = false; }
  }
  snprintf(b, sizeof(b), "%.1f m2", mapAreaM2());
  field(4, 53, 217, 134, 16, VISTA_GLASS, VISTA_LIGHT_BLUE, 2, MC_DATUM, b);

  // dolna belka: adres IP (po lewej) i temperatura (po prawej)
  IPAddress ip = net_ip();
  if (ip[0] == 0) strcpy(b, "Brak adresu IP"); else snprintf(b, sizeof(b), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  field(6, 6, 288, 162, 26, VISTA_DARK_BLUE, ip[0] ? BAR_CYAN : BAR_YELLOW, strlen(b) * 12 <= 160 ? 2 : 1, ML_DATUM, b);
  snprintf(b, sizeof(b), "%.1fC", getFormattedTemperature());
  field(1, 170, 288, 66, 26, VISTA_DARK_BLUE, BAR_YELLOW, 2, MR_DATUM, b);
}

// ====================================================================== USTAWIENIA
static char setSub[8][30];

static void buildSettings() {
  rowsReset(HDR_H);
  char* b;
  String sid = net_ssid();
  rowAdd(R_NONE, "POLACZENIA", "", I_NONE, 3);
  rowAdd(R_WIFI, "Wi-Fi", sid.length() ? sid.c_str() : "Brak polaczenia", I_WIFI, 1);
  int bt = btKeyboardState();
  rowAdd(R_BT, "Bluetooth", bt == 1 ? "Klawiatura: polaczona" : bt == 0 ? "Klawiatura: rozlaczona" : "Uruchamianie...", I_BT, 0);
  rowAdd(R_NONE, "ROBOT", "", I_NONE, 3);
  rowAdd(R_CTRL, "Sterowanie", "Skan, jazda, powrot", I_SCAN, 1);
  rowAdd(R_SENS, "Czujniki", "Podglad na zywo", I_SENS, 1);
  rowAdd(R_NONE, "MAPA", "", I_NONE, 3);
  b = setSub[1]; snprintf(b, 30, "%.1f m2 zbadane", mapAreaM2());
  rowAdd(R_MAPVIEW, "Podglad mapy", b, I_VIEW, 1);
  rowAdd(R_MAPMGR, "Mapy", "Zapis, nowa mapa", I_MAP, 1);
  rowAdd(R_NONE, "SYSTEM", "", I_NONE, 3);
  b = setSub[2];
  if (clockValid()) snprintf(b, 30, "%02d:%02d  %02d.%02d.%04d", dt.hour, dt.minute, dt.day, dt.month, dt.year); else strcpy(b, "Nie ustawiono");
  rowAdd(R_TIME, "Data i czas", b, I_CLOCK, 1);
  b = setSub[3];
  if (sdOk) { char z[12]; fmtSize(z, sizeof(z), sdSizeMB); snprintf(b, 30, "Wlozona, %s", z); } else strcpy(b, "Brak karty");
  rowAdd(R_SD, "Karta SD", b, I_SD, 1);
  rowAdd(R_THEME, "Motyw", THEME_NAMES[cfgTheme % 2], I_VIEW, 0);
  rowAdd(R_TOUCH, "Dotyk", "Kalibracja ekranu", I_TOUCH, 1);
  rowAdd(R_INFO, "Informacje", "IP, pamiec, flash", I_INFO, 1);
  rowAdd(R_RESTART, "Restart", "", I_POWER, 0, false, 0xFBAE);
  rowsFinish();
}

static int settingsSig() {
  String sid = net_ssid();
  return btKeyboardState() * 1000 + (int)sdOk * 100 + (int)sid.length() * 3 + (clockValid() ? dt.minute : 99) * 7;
}

// ---- Wi-Fi
static bool wifiScanShown = false;

static void drawWifiMode() {
  bool ap = net_mode() == 1;
  drawButton(8, 47, 110, 22, "Siec", ap ? 2 : 0, 2);
  drawButton(122, 47, 110, 22, "Punkt AP", ap ? 0 : 2, 2);
}

static void drawWifiCurrent() {
  drawCard(8, 70, 224, 56);
  bool wifi = WiFi.status() == WL_CONNECTED;
  char b[44], c[40];
  if (net_apOn() && !wifi) {                                 // pokazujemy dane własnej sieci robota
    fit(c, sizeof(c), net_apSsid(), 14);
    snprintf(b, sizeof(b), "AP: %s", c); T(b, 18, 76, 2, VISTA_TEXT);
    snprintf(b, sizeof(b), "haslo: %s", net_apPass()); T(b, 18, 98, 1, VISTA_YELLOW);
    IPAddress ip = net_ip();
    snprintf(b, sizeof(b), "adres: %d.%d.%d.%d%s", ip[0], ip[1], ip[2], ip[3], net_mode() == 1 ? "" : "  (awaryjny)"); T(b, 18, 111, 1, C_CYAN);
    return;
  }
  String sid = net_ssid();
  if (sid.length()) marquee(sid.c_str(), 18, 84, 11, 2, VISTA_TEXT, VISTA_GLASS); else T("Brak polaczenia", 18, 84, 2, VISTA_YELLOW, ML_DATUM);
  IPAddress ip = net_ip();
  if (ip[0]) snprintf(b, sizeof(b), "%d.%d.%d.%d  %d dBm", ip[0], ip[1], ip[2], ip[3], wifi ? WiFi.RSSI() : 0); else strcpy(b, "niepolaczono");
  T(b, 18, 100, 1, C_CYAN);
  if (net_hasSaved()) drawButton(150, 74, 76, 20, "Zapomnij", 2, 1);
  else T("siec domyslna z WiFi_config", 18, 112, 1, VISTA_YELLOW);
}

static void marqueeTick() {                                 // odswieza tylko dlugie nazwy (co 500 ms)
  String sid = net_ssid();
  if (sid.length() > 11 && !(net_apOn() && WiFi.status() != WL_CONNECTED)) marquee(sid.c_str(), 18, 84, 11, 2, VISTA_TEXT, VISTA_GLASS);
  int base = listTop - scrollY;
  for (int i = 0; i < rowsN; i++) {
    int y = base + rowOff[i];
    if (rows[i].kind != 5 || strlen(rows[i].title) <= (size_t)((RW - 76) / 12)) continue;
    if (y < listTop || y + rows[i].h > listBot) continue;
    marquee(rows[i].title, 46, y + rows[i].h / 2, (RW - 76) / 12, 2, VISTA_TEXT, VISTA_GLASS);
  }
}

static void drawWifiTop() {
  drawHeader("Wi-Fi", true);
  drawWifiMode();
  drawWifiCurrent();
  tft.fillRect(0, 128, SCR_W, 26, C_PANEL);
  if (net_mode() == 0) {
    T("DOSTEPNE SIECI", 12, 141, 1, VISTA_LIGHT_BLUE, ML_DATUM);
    drawButton(160, 129, 72, 24, netScanning ? "..." : "Szukaj", 2, 1);
  }
}

static void buildWifi() {
  rowsReset(154);
  if (net_mode() == 1) emptyMsg = "Tryb punktu dostepu";
  else if (netScanning) emptyMsg = "Szukam sieci...";
  else if (netListN == 0) emptyMsg = wifiScanShown ? "Nie znaleziono sieci" : "Dotknij: Szukaj";
  if (net_mode() == 0) for (int i = 0; i < netListN; i++) {
    Row& r = rowAdd(R_LIST + i, netList[i].ssid, "", I_NONE, 5);
    r.bars = sigBars(netList[i].rssi); r.lock = netList[i].secure;
  }
  rowsFinish();
}

// ---- klawiatura ekranowa
static char kbBuf[64];
static uint8_t kbLen = 0;
static bool kbShift = false, kbSym = false, kbShow = false;
static char kbSsid[33];
static bool kbSecure = true, kbRename = false;

static const char* const KB_ROWS[3][4] = {
  {"1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm"},
  {"1234567890", "QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"},
  {"1234567890", "!@#$%^&*()", "-_=+.,?/;", "~:'\"<>|"},
};
#define KB_Y0 84
#define KB_PITCH 40
#define KB_H 36

static int kbRowX(int r) { return r == 2 ? 12 : r == 3 ? 40 : 2; }

static void drawKey(int x, int y, int w, const char* label, bool special = false) {
  tft.fillRoundRect(x, y, w, KB_H, 5, special ? VISTA_DARK_GREY : VISTA_GLASS);
  tft.drawRoundRect(x, y, w, KB_H, 5, VISTA_LIGHT_BLUE);
  T(label, x + w / 2, y + KB_H / 2 + 1, 2, VISTA_TEXT, MC_DATUM);
}

static void drawKbField() {
  tft.fillRect(0, 40, SCR_W, 38, C_PANEL);
  tft.fillRoundRect(6, 44, 228, 30, 6, VISTA_DARK_BLUE);
  tft.drawRoundRect(6, 44, 228, 30, 6, VISTA_LIGHT_BLUE);
  char shown[24];
  int start = kbLen > 13 ? kbLen - 13 : 0, k = 0;
  for (int i = start; i < kbLen && k < 23; i++) shown[k++] = kbShow ? kbBuf[i] : '*';
  shown[k] = 0;
  T(shown, 14, 59, 2, VISTA_WHITE, ML_DATUM);
  int ex = 214, ey = 59;                                   // oko: pokaz/ukryj haslo
  tft.drawCircle(ex, ey, 6, VISTA_GREY); tft.fillCircle(ex, ey, 2, VISTA_GREY);
  if (!kbShow) tft.drawLine(ex - 8, ey + 7, ex + 8, ey - 7, VISTA_RED);
}

static void drawKeys() {
  int layer = kbSym ? 2 : (kbShift ? 1 : 0);
  tft.fillRect(0, KB_Y0 - 2, SCR_W, 220, C_PANEL);
  for (int r = 0; r < 4; r++) {
    const char* row = KB_ROWS[layer][r];
    for (int i = 0; row[i]; i++) { char l[2] = {row[i], 0}; drawKey(kbRowX(r) + i * 24, KB_Y0 + r * KB_PITCH, 22, l); }
  }
  int y3 = KB_Y0 + 3 * KB_PITCH;
  drawKey(2, y3, 34, "Aa", kbShift);
  drawKey(208, y3, 30, "<", true);
  int y4 = KB_Y0 + 4 * KB_PITCH;
  drawKey(2, y4, 48, kbSym ? "abc" : "?12", true);
  drawKey(54, y4, 124, " ");
  bool okEnabled = !kbSecure || kbLen >= 8;
  tft.fillRoundRect(182, y4, 56, KB_H, 5, okEnabled ? VISTA_BLUE : VISTA_DARK_GREY);
  tft.drawRoundRect(182, y4, 56, KB_H, 5, VISTA_LIGHT_BLUE);
  T("OK", 210, y4 + KB_H / 2 + 1, 2, okEnabled ? VISTA_WHITE : VISTA_GREY, MC_DATUM);
}

static void drawKeyboard() {
  tft.fillScreen(C_PANEL);
  char t[16]; fit(t, sizeof(t), kbSsid, 12);
  drawHeader(t, true);
  drawKbField();
  drawKeys();
  char b[40];
  if (kbRename) strcpy(b, "Nowa nazwa mapy (litery, cyfry, _ -)");
  else if (kbSecure) snprintf(b, sizeof(b), "Haslo sieci (min. 8 znakow): %d", kbLen); else strcpy(b, "Siec otwarta - dotknij OK");
  T(b, SCR_W / 2, 308, 1, VISTA_YELLOW, MC_DATUM);
}

static void kbStartFor(const NetEntry& e) {
  strncpy(kbSsid, e.ssid, 32); kbSsid[32] = 0;
  kbSecure = e.secure; kbRename = false; kbBuf[0] = 0; kbLen = 0; kbShift = kbSym = kbShow = false;
}

// ---- laczenie
static unsigned long connectShownMs = 0;
static void drawConnect() {
  tft.fillScreen(C_PANEL);
  drawHeader("Wi-Fi", false);
  char b[40]; fit(b, sizeof(b), kbSsid, 16);
  T(b, SCR_W / 2, 90, 2, VISTA_TEXT, MC_DATUM);
  if (netState == NET_CONNECTING) {
    T("Laczenie", SCR_W / 2, 140, 2, VISTA_LIGHT_BLUE, MC_DATUM);
    int dots = (millis() / 400) % 4;
    for (int i = 0; i < 3; i++) tft.fillCircle(100 + i * 20, 175, 5, i < dots ? VISTA_LIGHT_BLUE : VISTA_DARK_GREY);
  } else if (netState == NET_OK) {
    IPAddress ip = WiFi.localIP();
    tft.fillCircle(SCR_W / 2, 150, 24, VISTA_GREEN);
    tft.drawLine(SCR_W / 2 - 11, 150, SCR_W / 2 - 3, 159, VISTA_WHITE); tft.drawLine(SCR_W / 2 - 3, 159, SCR_W / 2 + 12, 140, VISTA_WHITE);
    tft.drawLine(SCR_W / 2 - 11, 151, SCR_W / 2 - 3, 160, VISTA_WHITE); tft.drawLine(SCR_W / 2 - 3, 160, SCR_W / 2 + 12, 141, VISTA_WHITE);
    T("Polaczono", SCR_W / 2, 196, 2, VISTA_GREEN, MC_DATUM);
    snprintf(b, sizeof(b), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    T(b, SCR_W / 2, 222, 2, C_CYAN, MC_DATUM);
  } else {
    tft.fillCircle(SCR_W / 2, 150, 24, VISTA_RED);
    tft.drawLine(SCR_W / 2 - 10, 140, SCR_W / 2 + 10, 160, VISTA_WHITE); tft.drawLine(SCR_W / 2 + 10, 140, SCR_W / 2 - 10, 160, VISTA_WHITE);
    T("Nie udalo sie", SCR_W / 2, 196, 2, VISTA_RED, MC_DATUM);
    T("Sprawdz haslo i zasieg", SCR_W / 2, 220, 1, VISTA_YELLOW, MC_DATUM);
    drawButton(20, 256, 94, 40, "Ponow", 0);
    drawButton(126, 256, 94, 40, "Wstecz", 2);
  }
}

// ---- Zarzadzanie mapa
static void drawMapMgrTop() {
  drawHeader("Mapa", true);
  drawCard(8, 48, 224, 44);
  uint32_t o, f; gridMap.stats(o, f);
  char b[44];
  snprintf(b, sizeof(b), "%.1f m2 zbadane", mapAreaM2());
  T(b, 18, 55, 2, VISTA_TEXT);
  snprintf(b, sizeof(b), "%u zajetych komorek, siatka %d cm", (unsigned)o, (int)MAP_CELL_CM);
  T(b, 18, 77, 1, VISTA_YELLOW);
  drawToastLine(300);
}

static void buildMapMgr() {
  rowsReset(98);
  listBot = 298;                                          // ponizej jest pasek komunikatow
  rowAdd(R_MSAVE, "Zapisz teraz", "do pamieci robota", I_SAVE, 0);
  { char* b = setSub[0]; snprintf(b, 30, "Co %d cm jazdy", (int)gScanEveryCm);
    rowAdd(R_SCAN, "Skanowanie", b, I_SCAN, 1); }
  rowAdd(R_MNEW, "Nowa mapa", "usuwa obecna mape", I_TRASH, 0, false, 0xFBAE);
  if (sdOk) {
    rowAdd(R_MSD, "Zapisz na karte", "do biblioteki map", I_SD, 0);
    rowAdd(R_SDLOAD, "Wczytaj z karty", "podglad, nazwa, usuwanie", I_LOAD, 1);
  }
  rowsFinish();
}

// ---- Karta SD
static void drawSdTop() {
  drawHeader("Karta SD", true);
  drawCard(8, 48, 224, 54);
  char b[40];
  if (sdOk) {
    char z[12]; fmtSize(z, sizeof(z), sdSizeMB);
    T("Karta wlozona", 18, 55, 2, VISTA_GREEN);
    snprintf(b, sizeof(b), "Pojemnosc %s", z); T(b, 18, 78, 1, VISTA_YELLOW);
  } else {
    T("Brak karty", 18, 55, 2, VISTA_YELLOW);
    snprintf(b, sizeof(b), "Dziala bez niej. Pin CS: %d", (int)SD_CS_PIN); T(b, 18, 78, 1, VISTA_YELLOW);
  }
  drawButton(150, 76, 74, 22, "Sprawdz", 2, 1);
  drawToastLine(300);
}

static void buildSd() {
  rowsReset(108);
  listBot = 298;
  if (sdOk) {
    rowAdd(R_SDLOG, "Zapisz dziennik", "biezaca sesja do pliku", I_INFO, 0);
    rowAdd(R_SDFMT, "Formatuj karte", "usuwa wszystkie pliki", I_TRASH, 0, false, 0xFBAE);
  }
  rowsFinish();
}

// ---- lista map na karcie
#define SD_NAMES_MAX 16
static char sdNames[SD_NAMES_MAX][32];
static int sdNamesN = 0, sdSelected = -1;
static char sdLabels[SD_NAMES_MAX][24];

static void prettyName(char* out, size_t n, const char* f) {          // mapa_20261115_193012.bin -> 15.11 19:30
  unsigned y, mo, d, h, mi, s;
  if (sscanf(f, "mapa_%4u%2u%2u_%2u%2u%2u", &y, &mo, &d, &h, &mi, &s) == 6) snprintf(out, n, "%02u.%02u %02u:%02u", d, mo, h, mi);
  else { char t[32]; strncpy(t, f, 31); t[31] = 0; size_t L = strlen(t); if (L > 4) t[L - 4] = 0; fit(out, n, t, 14); }
}

static void buildTheme() {
  rowsReset(HDR_H);
  for (int i = 0; i < 2; i++) rowAdd(R_LIST + i, THEME_NAMES[i], "", I_NONE, 6, cfgTheme % 2 == i);
  rowsFinish();
}

// ---- lista map zapisanych na karcie
static void buildSdMaps() {
  rowsReset(HDR_H);
  sdNamesN = sd_listMaps(sdNames, SD_NAMES_MAX);
  if (!sdNamesN) emptyMsg = "Brak zapisanych map";
  for (int i = 0; i < sdNamesN; i++) {
    prettyName(sdLabels[i], sizeof(sdLabels[0]), sdNames[i]);
    rowAdd(R_LIST + i, sdLabels[i], "", I_MAP, 1);
  }
  rowsFinish();
}

// ---- akcje dla wybranej mapy z karty
static void drawSdAct() {
  tft.fillScreen(C_PANEL);
  char t[24]; prettyName(t, sizeof(t), sdNames[sdSelected]);
  drawHeader(t, true);
  drawButton(12, 56, 216, 48, "Wczytaj jako mape", 0, 2);
  drawButton(12, 116, 216, 48, "Zmien nazwe", 2, 2);
  drawButton(12, 176, 216, 48, "Usun", 1, 2);
  T(sdNames[sdSelected], SCR_W / 2, 284, 1, VISTA_YELLOW, MC_DATUM);
}

// ---- Informacje
static void infoLine(int i, const char* label, const char* value) {
  int y = 47 + i * 19;
  tft.fillRect(8, y, 224, 17, VISTA_GLASS);
  T(label, 14, y + 9, 1, VISTA_YELLOW, ML_DATUM);
  T(value, 226, y + 9, 1, VISTA_TEXT, MR_DATUM);
}

static void fmtDur(char* b, size_t n, unsigned long s) {
  snprintf(b, n, "%lud %02lu:%02lu:%02lu", s / 86400, (s / 3600) % 24, (s / 60) % 60, s % 60);
}

static void infoValues() {
  char b[40];
  IPAddress ip = net_ip();
  snprintf(b, sizeof(b), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);  infoLine(0, "Adres IP", b);
  infoLine(1, "Siec", net_ssid().c_str());
  snprintf(b, sizeof(b), "%d dBm", WiFi.RSSI());                      infoLine(2, "Sygnal", b);
  fmtDur(b, sizeof(b), millis() / 1000);                              infoLine(3, "Czas od wlaczenia", b);
  fmtDur(b, sizeof(b), odometry_runSeconds());                        infoLine(4, "Czas pracy (od 19.11)", b);
  double m = odometry_totalCm() / 100.0;
  if (m >= 1000) snprintf(b, sizeof(b), "%.2f km", m / 1000.0); else snprintf(b, sizeof(b), "%.1f m", m);
  infoLine(5, "Przebieg (od 19.11)", b);
  snprintf(b, sizeof(b), "%u KB, blok %u KB", (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMaxAllocHeap() / 1024)); infoLine(6, "Wolny RAM", b);
  const esp_partition_t* part = esp_ota_get_running_partition();
  uint32_t used = ESP.getSketchSize(), total = part ? part->size : 0;
  if (total) snprintf(b, sizeof(b), "%u/%u KB (%u%%)", (unsigned)(used / 1024), (unsigned)(total / 1024), (unsigned)(used * 100ULL / total));
  else snprintf(b, sizeof(b), "%u KB", (unsigned)(used / 1024));
  infoLine(7, "Flash (program)", b);
  snprintf(b, sizeof(b), "%u KB wolne", (unsigned)((SPIFFS.totalBytes() - SPIFFS.usedBytes()) / 1024)); infoLine(8, "Pamiec plikow", b);
  if (sdOk) { char z[12]; fmtSize(z, sizeof(z), sdSizeMB); snprintf(b, sizeof(b), "%s", z); } else strcpy(b, "brak");
  infoLine(9, "Karta SD", b);
  snprintf(b, sizeof(b), "%.1f C", getFormattedTemperature());        infoLine(10, "Temperatura", b);
  snprintf(b, sizeof(b), "%d hPa", (int)cachedPressure);              infoLine(11, "Cisnienie", b);
  snprintf(b, sizeof(b), "%d m", getFormattedAltitude());             infoLine(12, "Wysokosc", b);
  infoLine(13, "Wersja", __DATE__);
}

static void drawInfo() {
  tft.fillScreen(C_PANEL);
  drawHeader("Informacje", true);
  infoValues();
}

// ---- Czujniki na zywo (widok z gory, przod robota u gory)
// Rysunek robota w skali 6 px/cm (korpus 14,2 x 26 cm); polozenia czujnikow wg Config.hpp
#define SB_PX 6
#define SB_X0 77
#define SB_X1 163
#define SB_Y0 106
#define SB_Y1 262
#define SB_AXLE (SB_Y0 + (int)(WHEEL_AXLE_FROM_FRONT_CM * SB_PX))

struct SensView { uint8_t idx; int x, y; int8_t dx, dy; int lx, ly; };      // dx,dy: kierunek wiazki; (lx,ly): srodek napisu
static const SensView SENS[6] = {
  {3,  96, SB_Y0 + 3, -1, -1,  58,  64},     // przod - lewy (45 st.)
  {4, 120, SB_Y0 + 4,  0, -1, 120,  54},     // przod - srodek
  {5, 144, SB_Y0 + 3,  1, -1, 182,  64},     // przod - prawy (45 st.)
  {2, SB_X0, SB_Y0 + 44, -1,  0,  36, SB_Y0 + 44},     // lewy bok - przod
  {7, SB_X0, SB_Y0 + 112, -1,  0,  36, SB_Y0 + 112},  // lewy bok - tyl
  {6, 120, SB_Y1 - 9,  0,  1, 172, SB_Y1 + 20},     // tyl (schowany 1,5 cm od konca)
};
static int sensShown[6];

static int sensCm(uint8_t i) { return i == 2 ? measurement2 : i == 3 ? measurement3 : i == 4 ? measurement4 : i == 5 ? measurement5 : i == 6 ? measurement6 : measurement7; }

static void sensDraw(int k, bool force) {
  const SensView& s = SENS[k];
  bool fresh = sensorAgeMs(s.idx) <= SENSOR_MAX_AGE_MS;
  int cm = sensCm(s.idx);
  int state = !fresh ? -1 : cm;
  if (!force && state == sensShown[k]) return;
  sensShown[k] = state;
  uint16_t col = !fresh ? VISTA_DARK_GREY : cm < 15 ? VISTA_RED : cm < 40 ? VISTA_YELLOW : VISTA_GREEN;
  int len = !fresh ? 0 : (cm > 100 ? 100 : cm) * 36 / 100;
  // wyczysc obszar wiazki i napisu
  if (s.dx && s.dy) tft.fillRect(s.x < 120 ? s.x - 34 : s.x - 2, s.y - 36, 36, 38, C_PANEL);
  else if (s.dy) { int y0 = s.dy < 0 ? 72 : SB_Y1 + 1; tft.fillRect(s.x - 19, y0, 38, s.dy < 0 ? SB_Y0 - 72 : 34, C_PANEL); }
  else tft.fillRect(0, s.ly - 10, SB_X0 - 1, 20, C_PANEL);
  if (len > 0) {
    if (s.dx && s.dy) {
      int d = len * 7 / 10;
      for (int o = -2; o <= 2; o++) tft.drawLine(s.x + o, s.y, s.x + s.dx * d + o, s.y + s.dy * d, col);
    }
    else if (s.dy) { int y0 = s.dy < 0 ? s.y - len : (k == 5 ? SB_Y1 : s.y); tft.fillRect(s.x - 5, y0, 10, len, col); }
    else tft.fillRect(s.x - len, s.y - 5, len, 10, col);
  }
  char b[8];
  if (!fresh) strcpy(b, "--"); else if (cm >= 150) strcpy(b, "far"); else snprintf(b, sizeof(b), "%d", cm);
  tft.fillRect(s.lx - 20, s.ly - 8, 40, 16, C_PANEL);
  T(b, s.lx, s.ly, 2, col, MC_DATUM);
  tft.fillRect(s.x - 4, s.y - 2, 8, 4, VISTA_TEXT);        // czujnik (zostaje po wyczyszczeniu)
}

static void drawSensStatic() {
  tft.fillScreen(C_PANEL);
  drawHeader("Czujniki", true);
  int wh = (int)(WHEEL_DIAMETER_CM * SB_PX);                              // kola (os ok. 13 cm od przodu)
  tft.fillRoundRect(SB_X0 - 5, SB_AXLE - wh / 2, 5, wh, 2, VISTA_DARK_GREY);
  tft.fillRoundRect(SB_X1, SB_AXLE - wh / 2, 5, wh, 2, VISTA_DARK_GREY);
  tft.fillRoundRect(SB_X0, SB_Y0, SB_X1 - SB_X0, SB_Y1 - SB_Y0, 10, VISTA_GLASS);
  tft.drawRoundRect(SB_X0, SB_Y0, SB_X1 - SB_X0, SB_Y1 - SB_Y0, 10, C_EDGE);
  tft.fillTriangle(120, SB_Y0 + 60, 108, SB_Y0 + 84, 132, SB_Y0 + 84, VISTA_DARK_GREY);   // kierunek jazdy
  int sy = SB_Y0 + (int)(SERVO_AXIS_FROM_FRONT_CM * SB_PX);
  tft.fillCircle(120, sy, 11, VISTA_DARK_BLUE);                           // beben serwa (skan mapy)
  tft.drawCircle(120, sy, 11, VISTA_GREY);
  T("PRZOD", 120, SB_Y0 + 96, 1, VISTA_YELLOW, MC_DATUM);
  // legenda
  const uint16_t lc[4] = {VISTA_RED, VISTA_YELLOW, VISTA_GREEN, VISTA_DARK_GREY};
  static const char* lt[4] = {"<15", "15-40", ">40", "brak"};
  static const int lx[4] = {8, 64, 128, 184};
  for (int i = 0; i < 4; i++) { tft.fillRect(lx[i], 308, 8, 8, lc[i]); T(lt[i], lx[i] + 12, 312, 1, VISTA_YELLOW, ML_DATUM); }
  for (int k = 0; k < 6; k++) sensDraw(k, true);
}

// ---- Podglad mapy (zoom i przesuwanie)
static int mapViewSig = 0;
// przyciski nalozone na mape: - / + po lewej u gory, strzalki jak w joysticku po prawej, 1:1 w srodku
struct MvBtn { int16_t x, y, w, h; const char* lbl; };
static const MvBtn MV_BTN[7] = {
  {4, HDR_H + 48, 46, 40, "-"}, {4, HDR_H + 4, 46, 40, "+"},
  {120, 246, 38, 36, "<"}, {160, 246, 38, 36, "1:1"}, {200, 246, 38, 36, ">"},
  {160, 204, 38, 36, "^"}, {160, 284, 38, 36, "v"}};
static void drawMapViewBar() {
  for (int i = 0; i < 7; i++) {
    const MvBtn& m = MV_BTN[i];
    bool dim = (i == 0 && mapZoomI == 0) || (i == 1 && mapZoomI == 7) || (i == 3 && mapPanX == 0 && mapPanY == 0 && mapZoomI == MAP_ZOOM_DEF);
    tft.fillRoundRect(m.x, m.y, m.w, m.h, 6, dim ? VISTA_DARK_GREY : VISTA_BLUE);
    tft.drawRoundRect(m.x, m.y, m.w, m.h, 6, dim ? VISTA_GREY : VISTA_WHITE);
    uint16_t fg = dim ? VISTA_GREY : VISTA_WHITE;
    int cx = m.x + m.w / 2, cy = m.y + m.h / 2;
    if (i <= 1) { T(m.lbl, cx, cy + 1, 3, fg, MC_DATUM); continue; }
    if (i == 3) { T(m.lbl, cx, cy + 1, 1, fg, MC_DATUM); continue; }
    if (i == 5) tft.fillTriangle(cx - 8, cy + 5, cx + 8, cy + 5, cx, cy - 6, fg);        // takie same trojkaty jak w przyciskach przewijania list
    else if (i == 6) tft.fillTriangle(cx - 8, cy - 5, cx + 8, cy - 5, cx, cy + 6, fg);
    else if (i == 2) tft.fillTriangle(cx + 5, cy - 8, cx + 5, cy + 8, cx - 6, cy, fg);
    else tft.fillTriangle(cx - 5, cy - 8, cx - 5, cy + 8, cx + 6, cy, fg);
  }
}

static void drawMapViewStatic() {
  tft.fillScreen(MAP_UNKNOWN);
  drawHeader(mapSrc ? "Podglad" : "Mapa", true);
  drawMapViewBar();
  mapViewSig = 0;
}

static void mapViewUpdate(bool force) {
  int sig = mapSig() ^ (mapZoomI * 977) ^ (mapPanX * 131) ^ (mapPanY * 17) ^ (mapSrc ? 0x5555 : 0);
  if (!force && sig == mapViewSig) return;
  if (!mapRender(0, HDR_H, SCR_W, SCR_H - HDR_H, true)) return;
  mapViewSig = sig;
  char b[24];
  float area;
  if (mapSrc) { long f = 0; for (long i = 0; i < (long)MAP_CELLS * MAP_CELLS; i++) if (mapPackedAt(mapSrc, i) < 0) f++; area = f * (MAP_CELL_CM / 100.0f) * (MAP_CELL_CM / 100.0f); }
  else area = mapAreaM2();
  snprintf(b, sizeof(b), "%d%%  %.1f m2", (int)MAP_ZOOMS[mapZoomI], area);
  tft.fillRect(56, HDR_H, strlen(b) * 6 + 8, 13, MAP_UNKNOWN);
  T(b, 60, HDR_H + 7, 1, VISTA_WALL, ML_DATUM);
  // pasek skali: dobieramy "ladna" dlugosc
  static const int steps[] = {10, 20, 25, 50, 100, 200, 500};
  int best = steps[0];
  for (unsigned i = 0; i < sizeof(steps) / sizeof(steps[0]); i++) if (steps[i] * mapPxPerCm <= 70) best = steps[i];
  int w = (int)(best * mapPxPerCm + 0.5f);
  int bx = 8;                                           // skala w lewym dolnym rogu (po prawej sa strzalki)
  tft.drawFastHLine(bx, 306, w, VISTA_WALL); tft.drawFastVLine(bx, 302, 9, VISTA_WALL); tft.drawFastVLine(bx + w, 302, 9, VISTA_WALL);
  snprintf(b, sizeof(b), best >= 100 ? "%d m" : "%d cm", best >= 100 ? best / 100 : best);
  T(b, bx + w / 2, 295, 1, VISTA_WALL, MC_DATUM);
  drawMapViewBar();
}

static void mapViewTouch(int tx, int ty) {
  int i = -1;
  for (int k = 0; k < 7; k++) if (inRect(tx, ty, MV_BTN[k].x, MV_BTN[k].y, MV_BTN[k].w, MV_BTN[k].h)) { i = k; break; }
  if (i < 0) return;
  // numeracja zachowana z poprzedniej wersji: 0 -, 1 +, 2 <, 3 gora, 4 dol, 5 >, 6 1:1
  static const uint8_t map2old[7] = {0, 1, 2, 6, 5, 3, 4};
  i = map2old[i];
  int stepX = mapViewCols / 3 > 0 ? mapViewCols / 3 : 1, stepY = mapViewRows / 3 > 0 ? mapViewRows / 3 : 1;
  int oldZ = mapZoomI;
  bool moved = true;                                     // przesuwanie dziala przy kazdym powiekszeniu
  if (i == 0 && mapZoomI > 0) mapZoomI--;
  else if (i == 1 && mapZoomI < 7) mapZoomI++;
  else if (i == 2 && moved) mapPanX -= stepX;
  else if (i == 3 && moved) mapPanY += stepY;
  else if (i == 4 && moved) mapPanY -= stepY;
  else if (i == 5 && moved) mapPanX += stepX;
  else if (i == 6) { mapZoomI = MAP_ZOOM_DEF; mapPanX = mapPanY = 0; }
  else return;
  if (oldZ != mapZoomI) drawMapViewBar();
  mapViewUpdate(true);
}

// ---- Data i czas
static int tmV[5];                           // rok, miesiac, dzien, godzina, minuta
static unsigned long timeSavedMs = 0;
static const int TCOL_X[5] = {4, 60, 104, 148, 192}, TCOL_W[5] = {52, 40, 40, 40, 40};
static const char* const TCOL_L[5] = {"ROK", "MIES", "DZIEN", "GODZ", "MIN"};

static int daysIn(int y, int m) {
  static const int d[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  return (m == 2 && (y % 4 == 0)) ? 29 : d[m - 1];
}

static void timeLoadFromRtc() {
  tmV[0] = clockValid() ? dt.year : 2026; tmV[1] = clockValid() ? dt.month : 1; tmV[2] = clockValid() ? dt.day : 1;
  tmV[3] = clockValid() ? dt.hour : 12; tmV[4] = clockValid() ? dt.minute : 0;
}

static void drawTimeCard() {
  drawCard(8, 48, 224, 58);
  char b[24];
  if (clockValid()) snprintf(b, sizeof(b), "%02d:%02d:%02d", dt.hour, dt.minute, dt.second); else strcpy(b, "--:--:--");
  T(b, 120, 66, 3, VISTA_TEXT, MC_DATUM);
  if (clockValid()) snprintf(b, sizeof(b), "%04d-%02d-%02d", dt.year, dt.month, dt.day); else strcpy(b, "nie ustawiono");
  T(b, 120, 91, 2, VISTA_LIGHT_BLUE, MC_DATUM);
}

static void drawTimeSteppers() {
  tft.fillRect(0, 200, SCR_W, 120, C_PANEL);
  for (int i = 0; i < 5; i++) {
    int x = TCOL_X[i], w = TCOL_W[i];
    T(TCOL_L[i], x + w / 2, 205, 1, VISTA_YELLOW, MC_DATUM);
    drawButton(x, 214, w, 22, "+", 2, 2);
    tft.fillRect(x, 238, w, 24, th->card == 0xFFFF ? 0xEF7D : VISTA_DARK_BLUE);       // w jasnym motywie jasne pole, zeby bylo widac ciemne cyfry
    tft.drawRect(x, 238, w, 24, VISTA_LIGHT_BLUE);
    char b[8]; snprintf(b, sizeof(b), i == 0 ? "%04d" : "%02d", tmV[i]);
    T(b, x + w / 2, 251, 2, VISTA_TEXT, MC_DATUM);
    drawButton(x, 264, w, 22, "-", 2, 2);
  }
  bool saved = millis() - timeSavedMs < 2000;
  drawButton(8, 291, 224, 26, saved ? "Zapisano" : "Ustaw czas", saved ? 3 : 0, 2);
}

static void drawTimeTop() { drawHeader("Data i czas", true); drawTimeCard(); }

static void buildTime() {
  rowsReset(108);
  listBot = 200;
  rowAdd(R_NTP, "Auto czas", ntp_status(), I_CLOCK, 2, cfgNtp);
  bool wifi = WiFi.status() == WL_CONNECTED;
  rowAdd(R_NTPNOW, "Synchronizuj", wifi ? "serwer NTP" : "wymaga Wi-Fi", I_WIFI, 0, false, wifi ? VISTA_TEXT : VISTA_GREY);
  rowsFinish();
}

// ---- Sterowanie robotem (te same polecenia co w panelu WWW)
static char ctrlMsg[40] = "";
static unsigned long ctrlMsgMs = 0;

static const char* robotStatusText() {
  if (scanning) return "Skanuje...";
  if (AUTO_DRIVE_state) {
    switch (lastAutoMode) { case 1: return "Wraca na start"; case 2: return "Przy scianie"; case 4: return "Zwiekszam dokl."; default: return "Mapuje pokoj"; }
  }
  if (busy_motors) return "Jedzie (reczne)";
  return "Gotowy";
}

static void drawCtrlTop() {
  drawHeader("Sterowanie", true);
  drawCard(8, 48, 224, 38);
  tft.fillRect(12, 51, 216, 32, VISTA_GLASS);
  bool busy = scanning || AUTO_DRIVE_state || busy_motors;
  T(robotStatusText(), 18, 53, 2, busy ? VISTA_YELLOW : VISTA_GREEN);
  char b[40];
  if (ctrlMsg[0] && millis() - ctrlMsgMs < 4000) T(ctrlMsg, 18, 73, 1, VISTA_YELLOW);
  else { Pose2 p = odometry_get(); snprintf(b, sizeof(b), "x %d  y %d  kat %d", (int)p.x, (int)p.y, (int)(mgWrap360(p.h) + 0.5f) % 360); T(b, 18, 73, 1, VISTA_YELLOW); }
  tft.fillRect(0, 280, SCR_W, SCR_H - 280, C_PANEL);
  drawButton(8, 284, 224, 32, "STOP", 1, 2);
}

static void buildCtrl() {
  rowsReset(90);
  listBot = 280;
  rowAdd(R_C_SCAN, "Skanuj mape", "Jeden skan z miejsca", I_SCAN, 0);
  rowAdd(R_C_AUTO, "Zmapuj pokoj", "Automatycznie, z powrotem", I_MAP, 0);
  rowAdd(R_C_WALL, "Przy scianie", "Jazda przy lewej scianie", I_SENS, 0);
  rowAdd(R_C_LOC, "Odnajdz sie", "Gdzie jestem na mapie", I_VIEW, 0);
  rowAdd(R_C_HOME, "Na start", "Powrot na pozycje startu", I_HOME, 0);
  rowAdd(R_C_REF, "Dokladnosc", "Zwieksz dokladnosc mapy", I_SCAN, 0);
  rowsFinish();
}

static void ctrlSay(const char* m) { strncpy(ctrlMsg, m, sizeof(ctrlMsg) - 1); ctrlMsg[sizeof(ctrlMsg) - 1] = 0; ctrlMsgMs = millis(); }

static void ctrlRun(const char* cmd, const char* okMsg) {
  const char* err = robotCommand(cmd);
  ctrlSay(err ? err : okMsg);
  drawCtrlTop();
}

// ---- okno potwierdzenia
static void drawConfirm() {
  tft.fillRoundRect(16, 96, 208, 128, 10, VISTA_DARK_BLUE);
  tft.drawRoundRect(16, 96, 208, 128, 10, VISTA_LIGHT_BLUE);
  const char *l1 = "", *l2 = "", *ok = "Tak";
  if (confirmKind == CF_NEWMAP) { l1 = "Usunac mape?"; l2 = "Zacznie od pustej mapy"; ok = "Usun"; }
  else if (confirmKind == CF_LOADMAP) { l1 = "Wczytac te mape?"; l2 = "Zastapi obecna mape"; ok = "Wczytaj"; }
  else if (confirmKind == CF_DELMAP) { l1 = "Usunac mape z karty?"; l2 = "Tego nie da sie cofnac"; ok = "Usun"; }
  else if (confirmKind == CF_SDFORMAT) { l1 = "Formatowac karte?"; l2 = "Usunie WSZYSTKIE pliki"; ok = "Formatuj"; }
  else if (confirmKind == CF_RESTART) { l1 = "Zrestartowac?"; l2 = "Robot uruchomi sie ponownie"; ok = "Restart"; }
  else { l1 = "Zapomniec siec?"; l2 = "Zapisane haslo zniknie"; ok = "Zapomnij"; }
  T(l1, SCR_W / 2, 118, strlen(l1) > 16 ? 1 : 2, VISTA_WHITE, MC_DATUM);
  T(l2, SCR_W / 2, 146, 1, BAR_YELLOW, MC_DATUM);
  drawButton(24, 170, 86, 40, "Anuluj", 2);
  drawButton(116, 170, 100, 40, ok, confirmKind == CF_LOADMAP ? 0 : 1, 2);
}

// ====================================================================== dotyk
static bool runTouchCalibration() {
  tft.fillScreen(TFT_BLACK);
  T("Dotknij ekran", 120, 150, 2, VISTA_WHITE, MC_DATUM);
  T("aby skalibrowac", 120, 175, 2, VISTA_WHITE, MC_DATUM);
  uint16_t x, y;
  unsigned long t0 = millis();
  bool touched = false;
  while (millis() - t0 < 10000) {
    if (tft.getTouch(&x, &y, 300)) { touched = true; break; }
    delay(20);
  }
  if (!touched) return false;
  while (tft.getTouch(&x, &y, 300)) delay(20);
  uint16_t cal[5];
  tft.fillScreen(TFT_BLACK);
  tft.calibrateTouch(cal, TFT_MAGENTA, TFT_BLACK, 15);
  memcpy(touchCal, cal, sizeof(touchCal));
  tft.setTouch(touchCal);
  Preferences prefs;
  if (prefs.begin("touch", false)) { prefs.putBytes("cal", touchCal, sizeof(touchCal)); prefs.end(); touchCalStored = true; }
  return true;
}

// Ekran z przewijana lista rysuje sie przez drawList(); chrome = gorna czesc (naglowek, karty)
static void drawChrome() {
  switch (scr) {
    case SC_SETTINGS: drawHeader("Ustawienia", true); break;
    case SC_WIFI: drawWifiTop(); break;
    case SC_MAP: drawMapMgrTop(); break;
    case SC_SD: drawSdTop(); break;
    case SC_SDMAPS: drawHeader("Mapy na SD", true); break;
    case SC_THEME: drawHeader("Motyw", true); break;
    case SC_TIME: drawTimeTop(); break;
    case SC_CTRL: drawCtrlTop(); break;
    default: break;
  }
}

static Screen navStack[8];
static uint8_t navN = 0;
static bool uiArmed = true;                 // po zmianie ekranu nowe dotkniecie liczy sie dopiero po puszczeniu palca
static unsigned long uiLockMs = 0;

static void gotoScreen(Screen s, bool keepScroll = false) {
  if (s == SC_HOME) navN = 0;
  uiArmed = false; uiLockMs = millis() + 150;
  if (scr == SC_MAPVIEW && s != SC_MAPVIEW) mapPreviewFree();
  if (s == SC_MAPVIEW && scr != SC_MAPVIEW) { mapZoomI = MAP_ZOOM_DEF; mapPanX = mapPanY = 0; }
  scr = s;
  if (!keepScroll) scrollY = 0;
  toastMsg[0] = 0;
  switch (s) {
    case SC_HOME: homeDraw(); homeUpdate(true); break;
    case SC_SETTINGS: tft.fillScreen(C_PANEL); buildSettings(); drawList(); break;
    case SC_WIFI: tft.fillScreen(C_PANEL); buildWifi(); drawList(); break;
    case SC_KEY: drawKeyboard(); break;
    case SC_CONNECT: drawConnect(); break;
    case SC_MAP: tft.fillScreen(C_PANEL); buildMapMgr(); drawList(); break;
    case SC_SD: tft.fillScreen(C_PANEL); buildSd(); drawList(); break;
    case SC_SDMAPS: tft.fillScreen(C_PANEL); buildSdMaps(); drawList(); break;
    case SC_INFO: drawInfo(); break;
    case SC_SENS: drawSensStatic(); break;
    case SC_MAPVIEW: drawMapViewStatic(); mapViewUpdate(true); break;
    case SC_SDACT: drawSdAct(); break;
    case SC_THEME: tft.fillScreen(C_PANEL); buildTheme(); drawList(); break;
    case SC_CTRL: ctrlMsg[0] = 0; tft.fillScreen(C_PANEL); buildCtrl(); drawList(); break;
    case SC_TIME: timeLoadFromRtc(); tft.fillScreen(C_PANEL); buildTime(); drawList(); drawTimeSteppers(); break;
    default: break;
  }
  tftLastUpdate = millis();
}

static int navScroll[8];                    // pozycja listy, z ktorej weszlismy glebiej (po cofnieciu wracamy w to samo miejsce)
static void navTo(Screen s) { if (navN < 8) { navScroll[navN] = scrollY; navStack[navN++] = scr; } gotoScreen(s); }
static void goBack() {
  Screen s = navN ? navStack[--navN] : SC_HOME;
  if (s != SC_HOME) scrollY = navScroll[navN];
  gotoScreen(s, s != SC_HOME);
}

static void startConfirm(ConfirmKind k) { confirmKind = k; scrBack = scr; scr = SC_CONFIRM; drawConfirm(); }

static void cycleScanInterval() {
  gScanEveryCm = gScanEveryCm < 50 ? 60 : gScanEveryCm < 75 ? 90 : 40;
  Settings_save();
}

static void kbPress(char c) {
  if (kbLen < (kbRename ? 24 : 63)) { kbBuf[kbLen++] = c; kbBuf[kbLen] = 0; }
  if (kbShift) { kbShift = false; drawKeys(); }
  drawKbField();
}

// Ekrany z listami reaguja na PUSZCZENIE palca (zeby dalo sie przewijac), reszta na dotkniecie
// Ekrany z listami reaguja na PUSZCZENIE palca (mniej przypadkowych wejsc), reszta na dotkniecie
static bool tapOnRelease() { return scr == SC_SETTINGS || scr == SC_SDMAPS || scr == SC_WIFI || scr == SC_THEME || scr == SC_MAP || scr == SC_SD || scr == SC_TIME || scr == SC_CTRL; }

static bool backHit(int tx, int ty) { return inRect(tx, ty, 0, 0, 90, HDR_H); }

static void onTouch(int tx, int ty) {
  if ((scr == SC_SETTINGS || scr == SC_WIFI || scr == SC_SDMAPS || scr == SC_MAP || scr == SC_SD || scr == SC_TIME || scr == SC_CTRL) && listScrollTap(tx, ty)) return;
  switch (scr) {
    case SC_HOME:
      if (inRect(tx, ty, 190, 0, 50, 46)) navTo(SC_SETTINGS);
      else if (inRect(tx, ty, 0, 0, 90, 46)) {                       // dotkniecie godziny: pelna data w gornej belce przez 4 s
        homeDateUntil = millis() + 5000; if (!homeDateUntil) homeDateUntil = 1;
        drawHomeDate();
      }
      else if (inRect(tx, ty, 45, 132, 150, 104)) navTo(SC_MAPVIEW);
      else if (inRect(tx, ty, 30, 244, 180, 30)) navTo(SC_CTRL);
      else if (ty >= 282) navTo(SC_WIFI);   // dolna belka z adresem IP
      break;
    case SC_SETTINGS: {
      if (backHit(tx, ty)) { goBack(); break; }
      int i = rowHit(tx, ty);
      if (i < 0) break;
      switch (rows[i].id) {
        case R_WIFI: navTo(SC_WIFI); break;
        case R_CTRL: navTo(SC_CTRL); break;
        case R_SENS: navTo(SC_SENS); break;
        case R_MAPVIEW: navTo(SC_MAPVIEW); break;
        case R_MAPMGR: navTo(SC_MAP); break;
        case R_TIME: navTo(SC_TIME); break;
        case R_SD: navTo(SC_SD); break;
        case R_TOUCH: runTouchCalibration(); gotoScreen(SC_SETTINGS, true); break;
        case R_INFO: navTo(SC_INFO); break;
        case R_THEME: navTo(SC_THEME); break;
        case R_RESTART: if (robotBusyNow()) { showToast("Zatrzymaj robota"); } else startConfirm(CF_RESTART); break;
      }
      break;
    }
    case SC_WIFI: {
      if (backHit(tx, ty)) { goBack(); break; }
      if (inRect(tx, ty, 8, 45, 110, 26)) { if (net_mode() != 0) { net_setMode(0); gotoScreen(SC_WIFI); } break; }
      if (inRect(tx, ty, 122, 45, 110, 26)) { if (net_mode() != 1) { net_setMode(1); gotoScreen(SC_WIFI); } break; }
      if (net_mode() == 0 && net_hasSaved() && !(net_apOn() && WiFi.status() != WL_CONNECTED) && inRect(tx, ty, 146, 70, 88, 28)) { startConfirm(CF_FORGET); break; }
      if (net_mode() == 0 && inRect(tx, ty, 156, 127, 80, 28)) { if (!netScanning) { net_scanStart(); wifiScanShown = true; scrollY = 0; buildWifi(); drawList(); } break; }
      int i = rowHit(tx, ty);
      if (i >= 0 && !netScanning) {
        int n = rows[i].id - R_LIST;
        if (n >= 0 && n < netListN) {
          kbStartFor(netList[n]);
          if (netList[n].secure) navTo(SC_KEY);
          else { net_connect(kbSsid, ""); navTo(SC_CONNECT); }
        }
      }
      break;
    }
    case SC_KEY: {
      if (backHit(tx, ty)) { goBack(); break; }
      if (inRect(tx, ty, 196, 44, 40, 30)) { kbShow = !kbShow; drawKbField(); break; }
      int layer = kbSym ? 2 : (kbShift ? 1 : 0);
      int r = (ty - KB_Y0) / KB_PITCH;
      if (ty < KB_Y0 || r < 0 || r > 4) break;
      if (r < 4) {
        if (r == 3 && tx < 38) { kbShift = !kbShift; drawKeys(); break; }
        if (r == 3 && tx >= 206) { if (kbLen) kbBuf[--kbLen] = 0; drawKbField(); drawKeys(); break; }
        int idx = (tx - kbRowX(r)) / 24;
        const char* row = KB_ROWS[layer][r];
        if (tx >= kbRowX(r) && idx >= 0 && idx < (int)strlen(row)) kbPress(row[idx]);
      } else {
        if (tx < 52) { kbSym = !kbSym; kbShift = false; drawKeys(); }
        else if (tx < 180) kbPress(' ');
        else if (kbRename) {
          if (kbLen < 1) break;
          int rr = sd_renameMap(sdNames[sdSelected], kbBuf);
          if (navN) navN--;                                           // z listy map (odswiezy sie z nowa nazwa)
          goBack();
          showToast(rr == 1 ? "Zmieniono nazwe" : rr < 0 ? "Taka nazwa juz istnieje" : "Nie udalo sie zmienic nazwy"); drawToastLine(300);
        }
        else if (!kbSecure || kbLen >= 8) { net_connect(kbSsid, kbBuf); gotoScreen(SC_CONNECT); }
      }
      break;
    }
    case SC_CONNECT:
      if (netState == NET_FAIL) {
        if (inRect(tx, ty, 20, 256, 94, 40)) { if (kbSecure) gotoScreen(SC_KEY); else goBack(); }
        else if (inRect(tx, ty, 126, 256, 94, 40)) goBack();
      } else if (netState == NET_OK) goBack();
      break;
    case SC_MAP: {
      if (backHit(tx, ty)) { goBack(); break; }
      int i = rowHit(tx, ty);
      if (i < 0) break;
      switch (rows[i].id) {
        case R_MSAVE: showToast(MapStore_save() ? "Mapa zapisana" : "Zapis nie powiodl sie"); drawToastLine(300); break;
        case R_MNEW: if (robotBusyNow()) { showToast("Zatrzymaj robota"); drawToastLine(300); } else startConfirm(CF_NEWMAP); break;
        case R_SCAN: cycleScanInterval(); buildMapMgr(); drawList(); break;
        case R_SDLOAD: navTo(SC_SDMAPS); break;
        case R_MSD: showToast(sd_saveMapNamed() ? "Zapisano na karcie" : "Zapis na karte nie powiodl sie"); drawToastLine(300); break;
      }
      break;
    }
    case SC_SD: {
      if (backHit(tx, ty)) { goBack(); break; }
      if (inRect(tx, ty, 146, 72, 88, 30)) { sd_recheck(); gotoScreen(SC_SD); showToast(sdOk ? "Karta dziala" : "Nie wykryto karty"); drawToastLine(300); break; }
      int i = rowHit(tx, ty);
      if (i < 0) break;
      switch (rows[i].id) {
        case R_SDLOG: showToast(sd_saveLog() ? "Dziennik zapisany" : "Blad zapisu"); drawToastLine(300); break;
        case R_SDFMT: startConfirm(CF_SDFORMAT); break;
      }
      break;
    }
    case SC_THEME: {
      if (backHit(tx, ty)) { goBack(); break; }
      int i = rowHit(tx, ty);
      if (i < 0) break;
      cfgTheme = rows[i].id - R_LIST; th = &THEMES[cfgTheme % 2]; Settings_save();
      tft.fillScreen(C_PANEL); buildTheme(); drawList();
      break;
    }
    case SC_SDMAPS: {
      if (backHit(tx, ty)) { goBack(); break; }
      int i = rowHit(tx, ty);
      if (i >= 0) { sdSelected = rows[i].id - R_LIST; navTo(SC_SDACT); }
      break;
    }
    case SC_SDACT: {
      if (backHit(tx, ty)) { goBack(); break; }
      if (inRect(tx, ty, 12, 56, 216, 48)) {
        if (robotBusyNow()) { showToast("Zatrzymaj robota"); drawToastLine(300); } else startConfirm(CF_LOADMAP);
      } else if (inRect(tx, ty, 12, 116, 216, 48)) {
        char st[32]; strncpy(st, sdNames[sdSelected], 31); st[31] = 0;
        size_t L = strlen(st); if (L > 4) st[L - 4] = 0;
        kbRename = true; kbSecure = false; kbShift = kbSym = false; kbShow = true;
        if (!strncmp(st, "mapa_", 5)) st[0] = 0;                 // nazwa automatyczna (data) - zaczynamy od pustego pola
        strncpy(kbBuf, st, 24); kbBuf[24] = 0; kbLen = strlen(kbBuf);
        strcpy(kbSsid, "Nazwa mapy");
        navTo(SC_KEY);
      } else if (inRect(tx, ty, 12, 176, 216, 48)) {
        startConfirm(CF_DELMAP);
      }
      break;
    }
    case SC_CTRL: {
      if (backHit(tx, ty)) { goBack(); break; }
      if (ty >= 284) { ctrlRun("stop", "Zatrzymano"); break; }
      int i = rowHit(tx, ty);
      if (i < 0) break;
      switch (rows[i].id) {
        case R_C_SCAN: ctrlRun("scan", "Skanuje..."); break;
        case R_C_AUTO: ctrlRun("auto", "Mapuje pokoj"); break;
        case R_C_WALL: ctrlRun("wall", "Jedzie przy scianie"); break;
        case R_C_LOC: ctrlRun("locate", "Szukam pozycji..."); break;
        case R_C_HOME: ctrlRun("home", "Wraca na start"); break;
        case R_C_REF: ctrlRun("refine", "Zwiekszam dokladnosc"); break;
      }
      break;
    }
    case SC_INFO: case SC_SENS:
      if (backHit(tx, ty)) goBack();
      break;
    case SC_MAPVIEW:
      if (backHit(tx, ty)) goBack(); else mapViewTouch(tx, ty);
      break;
    case SC_TIME: {
      if (backHit(tx, ty)) { goBack(); break; }
      int i = rowHit(tx, ty);
      if (i >= 0) {
        if (rows[i].id == R_NTP) { cfgNtp = !cfgNtp; Settings_save(); if (cfgNtp) ntp_request(); buildTime(); drawList(); }
        else if (rows[i].id == R_NTPNOW && WiFi.status() == WL_CONNECTED) { ntp_request(); buildTime(); drawList(); }
        break;
      }
      for (int c = 0; c < 5; c++) {
        if (tx < TCOL_X[c] || tx >= TCOL_X[c] + TCOL_W[c]) continue;
        int dir = inRect(tx, ty, TCOL_X[c], 210, TCOL_W[c], 30) ? 1 : inRect(tx, ty, TCOL_X[c], 260, TCOL_W[c], 30) ? -1 : 0;
        if (!dir) break;
        static const int lo[5] = {2024, 1, 1, 0, 0}, hi[5] = {2099, 12, 31, 23, 59};
        int h = c == 2 ? daysIn(tmV[0], tmV[1]) : hi[c];
        tmV[c] += dir;
        if (tmV[c] > h) tmV[c] = lo[c]; else if (tmV[c] < lo[c]) tmV[c] = h;
        if (tmV[2] > daysIn(tmV[0], tmV[1])) tmV[2] = daysIn(tmV[0], tmV[1]);
        drawTimeSteppers();
        break;
      }
      if (inRect(tx, ty, 8, 291, 224, 26)) { if (Time_set(tmV[0], tmV[1], tmV[2], tmV[3], tmV[4], 0)) { timeSavedMs = millis(); drawTimeCard(); drawTimeSteppers(); } }
      break;
    }
    case SC_CONFIRM: {
      bool ok = inRect(tx, ty, 116, 170, 100, 40), cancel = inRect(tx, ty, 24, 170, 86, 40);
      if (!ok && !cancel) break;
      Screen back = scrBack;
      if (ok) {
        if (confirmKind == CF_NEWMAP) { mapNewMap(); showToast("Mapa usunieta"); }
        else if (confirmKind == CF_LOADMAP) { showToast(sd_loadMapFile(sdNames[sdSelected]) ? "Mapa wczytana" : "Nie udalo sie wczytac"); back = SC_MAP; navN = navN >= 2 ? navN - 2 : 0; }
        else if (confirmKind == CF_DELMAP) { showToast(sd_deleteMap(sdNames[sdSelected]) ? "Mapa usunieta" : "Nie udalo sie usunac"); back = SC_SDMAPS; if (navN) navN--; }
        else if (confirmKind == CF_SDFORMAT) {
          tft.fillScreen(VISTA_DARK_BLUE);
          T("FORMATOWANIE", SCR_W / 2, 130, 3, VISTA_WHITE, MC_DATUM);
          T("Prosze czekac...", SCR_W / 2, 175, 2, BAR_YELLOW, MC_DATUM);
          showToast(sd_format() ? "Karta sformatowana" : "Formatowanie niepelne");
        }
        else if (confirmKind == CF_RESTART) {
          tft.fillScreen(VISTA_DARK_BLUE);                                  // od razu widac, ze to restart, a nie zawieszenie
          T("RESTART", SCR_W / 2, 120, 4, VISTA_WHITE, MC_DATUM);
          T("Prosze czekac...", SCR_W / 2, 170, 2, BAR_YELLOW, MC_DATUM);
          tft.drawRoundRect(30, 205, 180, 16, 6, VISTA_LIGHT_BLUE);
          tft.fillRoundRect(33, 208, 18, 10, 3, VISTA_LIGHT_BLUE);
          MapStore_save(); odometry_saveCounters();
          for (int p = 1; p <= 10; p++) { tft.fillRoundRect(33, 208, 18 * p, 10, 3, VISTA_LIGHT_BLUE); delay(80); }
          ESP.restart();
        }
        else net_forget();
      }
      gotoScreen(back, true);
      if (ok && (confirmKind == CF_NEWMAP || confirmKind == CF_LOADMAP || confirmKind == CF_DELMAP || confirmKind == CF_SDFORMAT)) drawToastLine(300);
      break;
    }
  }
}

// ====================================================================== start i petla
void TFTsetup() {
  th = &THEMES[cfgTheme % 2];
  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);
  Preferences prefs;
  if (prefs.begin("touch", true)) {
    if (prefs.getBytesLength("cal") == sizeof(touchCal)) { prefs.getBytes("cal", touchCal, sizeof(touchCal)); touchCalStored = true; }
    prefs.end();
  }
  tft.setTouch(touchCal);
  if (!touchCalStored) runTouchCalibration();
  gotoScreen(SC_HOME);
  forceDisplayUpdate = true;
}

void TFTbootScreen(const char* msg) {
  drawBackground();
  T("ROBOT ESP32", SCR_W / 2, 140, 3, VISTA_WHITE, MC_DATUM);
  T(msg, SCR_W / 2, 180, 2, VISTA_LIGHT_BLUE, MC_DATUM);
}

void refreshTFT() {
  static unsigned long lastTouchPoll = 0, lastAction = 0;
  static bool down = false, stopFired = false;
  static int x0 = 0, y0 = 0;
  unsigned long now = millis();

  net_poll();
  if (net_scanPoll() && scr == SC_WIFI) { buildWifi(); drawList(); }

  if (now - lastTouchPoll >= 15) {
    lastTouchPoll = now;
    uint16_t tx = 0, ty = 0;
    bool touched = tft.getTouch(&tx, &ty, 350);
    static uint8_t idle = 0;
    static Screen downScr = SC_HOME;
    if (!touched) { if (idle < 255) idle++; if (idle >= 2 && now >= uiLockMs) uiArmed = true; } else idle = 0;
    if (touched) {
      if (!down && uiArmed) {
        down = true; stopFired = false; x0 = tx; y0 = ty; downScr = scr;
        if (scr == SC_CTRL && ty >= 284) { stopFired = true; lastAction = now; onTouch(tx, ty); }    // STOP reaguje od razu
        else if (!tapOnRelease()) { lastAction = now; onTouch(tx, ty); }
      }
    } else if (down) {
      down = false;
      // reakcja na puszczenie tylko na tym samym ekranie, na ktorym zaczelo sie dotkniecie
      if (tapOnRelease() && !stopFired && scr == downScr && uiArmed) { lastAction = now; onTouch(x0, y0); }
    }
  }
  { static unsigned long lastMq = 0;
    if (scr == SC_WIFI && uiArmed && now - lastMq >= 500) { lastMq = now; marqueeTick(); } }

  if (scr == SC_CONNECT) {
    static NetState lastState = NET_IDLE;
    static unsigned long lastAnim = 0;
    if (netState != lastState || (netState == NET_CONNECTING && now - lastAnim > 400)) { lastState = netState; lastAnim = now; drawConnect(); connectShownMs = now; }
    if (netState == NET_OK && now - connectShownMs > 2500) goBack();
  }

  if (scr == SC_SENS && now - tftLastUpdate >= 200) { tftLastUpdate = now; for (int k = 0; k < 6; k++) sensDraw(k, false); }
  else if (scr == SC_MAPVIEW && now - tftLastUpdate >= 1000) { tftLastUpdate = now; mapViewUpdate(false); }
  else if (now - tftLastUpdate >= 1000 && scr != SC_SENS && scr != SC_MAPVIEW) {
    tftLastUpdate = now;
    switch (scr) {
      case SC_HOME: homeUpdate(forceDisplayUpdate); forceDisplayUpdate = false; break;
      case SC_INFO: infoValues(); break;
      case SC_CTRL: drawCtrlTop(); break;
      case SC_SETTINGS: { static int sig = 0; int s = settingsSig(); if (s != sig) { sig = s; buildSettings(); drawList(); } break; }
      case SC_WIFI: if (!netScanning) drawWifiCurrent(); break;
      case SC_TIME: { drawTimeCard(); buildTime(); drawList(); if (timeSavedMs && now - timeSavedMs > 2000 && now - timeSavedMs < 3100) drawTimeSteppers(); break; }
      default: break;
    }
  }
}
