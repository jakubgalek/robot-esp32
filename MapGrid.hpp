#pragma once
// ---------------------------------------------------------------------------
// Siatka zajętości (occupancy grid) - czysta logika, bez zależności od Arduino,
// dzięki czemu jest testowana na komputerze (test/host_test.cpp).
//
// - Świat: cm, początek (0,0) w środku siatki, oś X w prawo, Y "do góry".
// - Kąt: stopnie, 0 = oś X, rośnie przeciwnie do ruchu wskazówek zegara.
// - Komórka przechowuje log-szanse (int8): > 0 zajęta, < 0 wolna, 0 nieznana.
// ---------------------------------------------------------------------------
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Config.hpp"

#define MG_DEG2RAD 0.017453292519943295f

#define L_HIT      30     // trafienie promienia
#define L_FREE     4      // promień przeszedł przez komórkę
#define L_FREE_WEAK 2     // jw., ale dla "brak echa"
#define L_MIN      (-60)
#define L_MAX      100
#define L_OCC_T    20     // >= : rysujemy jako przeszkodę
#define L_FREE_T   (-4)   // <= : rysujemy jako wolne

struct Pose2 { float x, y, h; };

// Jeden pomiar skanu we współrzędnych robota
struct Beam {
  float relDeg;     // kierunek względem osi robota (0 = przód, + = w lewo)
  float rangeCm;
  uint8_t kind;     // 1 = trafienie, 0 = brak echa (tylko wolna przestrzeń)
  uint8_t sensor;   // 0 = czujnik "przedni" serwa, 1 = "tylny" (do interpolacji między kierunkami)
};

// Kiedy ufamy dopasowaniu skanu do mapy. Przy mapie złożonej z komórek 5 cm przypadkowe "zyski"
// rzędu kilku % są szumem kwantyzacji, więc wymagamy wyraźnej poprawy.
static inline bool mgMatchAccepted(float base, float score, int hits) {
  float gain = score - base;
  return gain >= 4.0f && gain >= 0.10f * hits && score >= 0.5f * hits;
}

// Różnica kątów sprowadzona do (-45, 45]: ściany pokoju są co 90 stopni
static inline float mgWrapPm45(float a) {
  a = fmodf(a, 90.0f);
  if (a > 45.0f) a -= 90.0f;
  if (a <= -45.0f) a += 90.0f;
  return a;
}

// Uczenie skali żyroskopu: d = o ile (według ścian) nasz kąt był za duży po obrotach o łącznie S stopni
// (ze znakiem: lewo +). Błąd skali daje błąd proporcjonalny do obrotu.
static inline float mgScaleUpdate(float scale, float d, float S) {
  if (fabsf(S) < 45.0f || fabsf(d) > 15.0f) return scale;
  float k = 1.0f - d / S;
  if (k < 0.8f || k > 1.2f) return scale;
  float ns = scale * (1.0f + 0.3f * (k - 1.0f));
  if (ns < 0.9f) ns = 0.9f;
  if (ns > 1.1f) ns = 1.1f;
  return ns;
}

static inline float mgWrap360(float a) {
  a = fmodf(a, 360.0f);
  if (a < 0) a += 360.0f;
  return a;
}

class OccGrid {
 public:
  int8_t* cells;
  uint32_t version;     // rośnie przy każdej zmianie (UI pobiera mapę tylko, gdy się zmieniła)

  // Układ mapy jest wyrównany do ścian pokoju: ściany leżą wzdłuż osi X i Y (modulo 90 st.).
  bool scanned;         // true po pierwszym skanie (albo wczytaniu mapy) - wcześniej czujniki stałe nie piszą na mapę
  bool refValid;        // true = pierwszy dobry skan wyrównał układ do ścian
  float startHeading;   // kierunek robota na starcie w tym układzie (do powrotu na start)
  OccGrid() : cells(NULL), version(0), scanned(false), refValid(false), startHeading(ROBOT_START_HEADING), yieldHook(NULL) {}

  bool begin() {
    if (!cells) cells = (int8_t*)calloc((size_t)MAP_CELLS * MAP_CELLS, 1);
    return cells != NULL;
  }
  void clear() {
    refValid = false; scanned = false; startHeading = ROBOT_START_HEADING;
    if (cells) memset(cells, 0, (size_t)MAP_CELLS * MAP_CELLS);
    version++;
  }

  static int toCell(float v) { return (int)floorf(v / MAP_CELL_CM) + MAP_CELLS / 2; }
  static bool inside(int cx, int cy) { return cx >= 0 && cy >= 0 && cx < MAP_CELLS && cy < MAP_CELLS; }
  // Pod robotem jest z definicji wolno: czyści komórki obrysu (fałszywe trafienia z czujników
  // stałych albo dryf nie mogą zostawić robota "w ścianie").
  void clearFootprint(float x, float y, float hDeg, float len, float wid) {
    if (!cells) return;
    float c = cosf(hDeg * MG_DEG2RAD), s = sinf(hDeg * MG_DEG2RAD);
    for (float a = -len / 2; a <= len / 2 + 0.01f; a += MAP_CELL_CM / 2.0f)
      for (float b = -wid / 2; b <= wid / 2 + 0.01f; b += MAP_CELL_CM / 2.0f) {
        int cx = toCell(x + c * a - s * b), cy = toCell(y + s * a + c * b);
        if (inside(cx, cy) && cells[cy * MAP_CELLS + cx] > -10) cells[cy * MAP_CELLS + cx] = -10;
      }
  }
  int8_t at(int cx, int cy) const { return inside(cx, cy) ? cells[cy * MAP_CELLS + cx] : 0; }

  void add(int cx, int cy, int d) {
    if (!inside(cx, cy)) return;
    int v = cells[cy * MAP_CELLS + cx] + d;
    if (v > L_MAX) v = L_MAX;
    if (v < L_MIN) v = L_MIN;
    cells[cy * MAP_CELLS + cx] = (int8_t)v;
  }

  // Pozycja czujnika w świecie dla pozycji robota i przesunięcia czujnika
  static void sensorOrigin(const Pose2& p, float offFwd, float offLeft, float& sx, float& sy) {
    float c = cosf(p.h * MG_DEG2RAD), s = sinf(p.h * MG_DEG2RAD);
    sx = p.x + c * offFwd - s * offLeft;
    sy = p.y + s * offFwd + c * offLeft;
  }

  // Wpisuje jeden promień do mapy
  void integrateBeam(float sx, float sy, float angDeg, float rangeCm, bool hit) {
    float a = angDeg * MG_DEG2RAD;
    float dx = cosf(a), dy = sinf(a);
    int endX = toCell(sx + dx * rangeCm), endY = toCell(sy + dy * rangeCm);
    int freeW = hit ? L_FREE : L_FREE_WEAK;
    const float step = MAP_CELL_CM * 0.5f;
    int lastX = -1000, lastY = -1000;
    for (float t = 0; t < rangeCm; t += step) {
      int cx = toCell(sx + dx * t), cy = toCell(sy + dy * t);
      if (cx == lastX && cy == lastY) continue;
      lastX = cx; lastY = cy;
      if (hit && cx == endX && cy == endY) break;   // komórkę trafienia obsługujemy niżej
      add(cx, cy, -freeW);
    }
    if (hit) add(endX, endY, L_HIT);
  }

  // Jeden promień: trafienie (kind 1) albo brak echa (kind 0, tylko wolna przestrzeń)
  void integrateOne(float sx, float sy, float angDeg, uint8_t kind, float rangeCm) {
    if (rangeCm <= 0) return;
    if (kind == 1) {
      integrateBeam(sx, sy, angDeg, rangeCm, true);
    } else {
      integrateBeam(sx, sy, angDeg, rangeCm > MAP_NOECHO_FREE_CM ? (float)MAP_NOECHO_FREE_CM : rangeCm, false);
    }
  }

  // Wpisuje cały skan (po jednym pomiarze na kierunek) dla danej pozycji robota.
  // Między sąsiednimi kierunkami tego samego czujnika wstawiamy dodatkowy promień
  // (interpolowany), bo przy kroku 3 deg i zasięgu 1,2 m promienie są dalej niż komórka
  // (6 cm > 5 cm) i zostawiałyby w mapie nieznane "dziury".
  void integrateScan(const Pose2& p, const Beam* beams, int n) {
    if (!cells) return;
    float sx, sy;
    sensorOrigin(p, SERVO_SENSOR_OFFSET_FWD_CM, SERVO_SENSOR_OFFSET_LEFT_CM, sx, sy);
    for (int i = 0; i < n; i++) {
      integrateOne(sx, sy, p.h + beams[i].relDeg, beams[i].kind, beams[i].rangeCm);
      // sąsiad: następny pomiar tego samego czujnika o podobnym kierunku
      for (int j = i + 1; j < n && j <= i + 3; j++) {
        if (beams[j].sensor != beams[i].sensor) continue;
        float dRel = fabsf(beams[j].relDeg - beams[i].relDeg);
        if (dRel > SCAN_STEP_DEG * 1.5f || dRel < 0.01f) break;
        float mid = p.h + 0.5f * (beams[i].relDeg + beams[j].relDeg);
        const Beam &a = beams[i], &b = beams[j];
        if (a.kind == 1 && b.kind == 1) {
          if (fabsf(a.rangeCm - b.rangeCm) <= 10.0f) integrateOne(sx, sy, mid, 1, 0.5f * (a.rangeCm + b.rangeCm));
          else integrateOne(sx, sy, mid, 0, fminf(a.rangeCm, b.rangeCm) - MAP_CELL_CM);   // tylko wolne
        } else if (a.kind == 0 && b.kind == 0) {
          integrateOne(sx, sy, mid, 0, MAP_NOECHO_FREE_CM);
        }
        break;
      }
    }
    version++;
  }

  // ------------------------------------------------------------------
  // Dopasowanie skanu do istniejącej mapy
  // ------------------------------------------------------------------
  // Ocena pozycji: suma "pewności", że końce promieni trafiają w znane przeszkody.
  float scoreAt(const Pose2& p, const Beam* beams, int n) const {
    float sx, sy;
    sensorOrigin(p, SERVO_SENSOR_OFFSET_FWD_CM, SERVO_SENSOR_OFFSET_LEFT_CM, sx, sy);
    float s = 0;
    for (int i = 0; i < n; i++) {
      if (beams[i].kind != 1) continue;
      float a = (p.h + beams[i].relDeg) * MG_DEG2RAD;
      s += beamScore(sx + cosf(a) * beams[i].rangeCm, sy + sinf(a) * beams[i].rangeCm);
    }
    return s;
  }

  // Ocena rozszerzona (do rozstrzygania niejednoznaczności): jak scoreAt, ale karze promienie, które
  // przechodzą przez znane przeszkody, i końce promieni w miejscach nigdy niewidzianych.
  float scoreFull(const Pose2& p, const Beam* beams, int n) const {
    float sx, sy;
    sensorOrigin(p, SERVO_SENSOR_OFFSET_FWD_CM, SERVO_SENSOR_OFFSET_LEFT_CM, sx, sy);
    float s = scoreAt(p, beams, n);
    for (int i = 0; i < n; i++) {
      if (beams[i].kind != 1) continue;
      float a = (p.h + beams[i].relDeg) * MG_DEG2RAD;
      float dx = cosf(a), dy = sinf(a);
      for (float t = 6.0f; t < beams[i].rangeCm - 9.0f; t += MAP_CELL_CM) {
        if (at(toCell(sx + dx * t), toCell(sy + dy * t)) >= L_OCC_T) { s -= 0.6f; break; }
      }
      int ex = toCell(sx + dx * beams[i].rangeCm), ey = toCell(sy + dy * beams[i].rangeCm);
      if (at(ex, ey) == 0 && at(ex + 1, ey) == 0 && at(ex - 1, ey) == 0 && at(ex, ey + 1) == 0 && at(ex, ey - 1) == 0) s -= 0.25f;
    }
    return s;
  }

  // Szuka najlepszej pozycji w pobliżu "prior" (nie dalej niż radiusCm / radiusDeg). Zwraca ocenę
  // w pozycji prior (base) i najlepszą (best); out = najlepsza pozycja. Dwa etapy: zgrubny i dokładny.
  // Jeśli dopasowanie jest NIEJEDNOZNACZNE (np. skan widzi tylko jedną ścianę i można się przesunąć
  // wzdłuż niej, albo drugi, odległy układ pasuje niemal tak samo dobrze) - zwracamy prior (zysk = 0),
  // bo wtedy korekta byłaby zgadywaniem.
  void refinePose(const Pose2& prior, const Beam* beams, int n,
                  float radiusCm, float radiusDeg, Pose2& out, float& base, float& best) const {
    base = scoreAt(prior, beams, n);
    best = base;
    out = prior;
    Pose2 centre = prior;
    float stepXY[2] = {(float)MAP_CELL_CM, MAP_CELL_CM * 0.5f};
    float stepA[2] = {2.0f, 1.0f};
    float radXY[2] = {radiusCm, (float)MAP_CELL_CM};
    float radA[2] = {radiusDeg, 2.0f};
    for (int stage = 0; stage < 2; stage++) {
      Pose2 stageBest = centre;
      float stageScore = -1e9f, stageRaw = 0;
      for (int pass = 0; pass < (stage == 0 ? 2 : 1); pass++) {
        float second = 0;                                   // pass 1 (tylko etap zgrubny): najlepszy ODLEGŁY kandydat
        for (float da = -radA[stage]; da <= radA[stage] + 0.01f; da += stepA[stage]) {
          for (float dx = -radXY[stage]; dx <= radXY[stage] + 0.01f; dx += stepXY[stage]) {
            for (float dy = -radXY[stage]; dy <= radXY[stage] + 0.01f; dy += stepXY[stage]) {
              Pose2 c = {centre.x + dx, centre.y + dy, centre.h + da};
              float ddx = c.x - prior.x, ddy = c.y - prior.y, dda = c.h - prior.h;
              float dist = sqrtf(ddx * ddx + ddy * ddy);
              if (dist > radiusCm + 0.01f || fabsf(dda) > radiusDeg + 0.01f) continue;   // limit korekty
              float raw = scoreAt(c, beams, n);
              if (pass == 0) {
                float sc = raw - (0.03f * dist + 0.05f * fabsf(dda));   // kara za odejście od odometrii
                if (sc > stageScore) { stageScore = sc; stageBest = c; stageRaw = raw; }
              } else {
                float bx = c.x - stageBest.x, by = c.y - stageBest.y, ba = c.h - stageBest.h;
                if (sqrtf(bx * bx + by * by) >= 12.0f || fabsf(ba) >= 5.0f) { if (raw > second) second = raw; }
              }
            }
          }
        }
        if (pass == 1 && stageRaw > 0 && second >= 0.9f * stageRaw) return;   // niejednoznaczne: bez korekty
      }
      centre = stageBest;
    }
    out = centre;
    best = scoreAt(out, beams, n);
  }

  // ------------------------------------------------------------------
  // Statystyki i kodowanie dla UI
  // ------------------------------------------------------------------
  void stats(uint32_t& occ, uint32_t& freeCells) const {
    occ = freeCells = 0;
    if (!cells) return;
    for (int i = 0; i < MAP_CELLS * MAP_CELLS; i++) {
      if (cells[i] >= L_OCC_T) occ++;
      else if (cells[i] <= L_FREE_T) freeCells++;
    }
  }

  // Klasa komórki do wyświetlania: 0 nieznana, 1 wolna, 2 przeszkoda
  static uint8_t classOf(int8_t v) { return v >= L_OCC_T ? 2 : (v <= L_FREE_T ? 1 : 0); }

  // RLE: pary (liczba 1..255, klasa). emit(const uint8_t*, size_t) dostaje kolejne porcje.
  template <class F>
  size_t encodeRle(F emit) const {
    uint8_t buf[256];
    size_t bl = 0, total = 0;
    if (!cells) return 0;
    uint8_t cur = classOf(cells[0]);
    int run = 0;
    const int total_cells = MAP_CELLS * MAP_CELLS;
    for (int i = 0; i <= total_cells; i++) {
      uint8_t c = (i < total_cells) ? classOf(cells[i]) : 255;
      if (c == cur && run < 255) { run++; continue; }
      if (run > 0) {
        buf[bl++] = (uint8_t)run;
        buf[bl++] = cur;
        total += 2;
        if (bl >= sizeof(buf)) { emit(buf, bl); bl = 0; }
      }
      cur = c;
      run = 1;
    }
    if (bl) emit(buf, bl);
    return total;
  }

 
  // ---- Orientacja ścian ze skanu -------------------------------------------------------------
  // Z par sąsiednich trafień (odległych o 2-3 kroki serwa) bierzemy kierunek "cięciwy" - to kierunek
  // powierzchni. Kierunki składamy modulo 90 stopni (mnożymy kąt przez 4, średnia kołowa), więc
  // ściany prostopadłe do siebie głosują na tę samą orientację. conf = długość średniej kołowej (0..1).
  // Wynik: theta w świecie dla podanej pozycji (-45, 45].
  static bool wallOrientation(const Pose2& pose, const Beam* b, int n, float& theta, float& conf, float* prec = NULL) {
    theta = 0; conf = 0; if (prec) *prec = 0;
    float sc = 0, ss = 0, sw = 0;
    int idx[2][128]; int m[2] = {0, 0};
    for (int sensor = 0; sensor < 2; sensor++)
      for (int i = 0; i < n && m[sensor] < 128; i++) if (b[i].sensor == sensor && b[i].kind == 1) idx[sensor][m[sensor]++] = i;
    // Przebieg 1: krótkie cięciwy (2-3 kroki) - zgrubna orientacja i "pewność"
    for (int pass = 0; pass < 2; pass++) {
      float c2 = 0, s2 = 0, w2 = 0;
      for (int sensor = 0; sensor < 2; sensor++) {
        for (int a = 0; a < m[sensor]; a++) for (int k = (pass ? 2 : 2); k <= (pass ? 10 : 3) && a + k < m[sensor]; k++) {
          const Beam& p = b[idx[sensor][a]]; const Beam& q = b[idx[sensor][a + k]];
          float dAng = fabsf(q.relDeg - p.relDeg);
          if (dAng > (pass ? 33.0f : 13.0f) || dAng < 3.0f) continue;   // luka w danych albo za blisko
          float a1 = (pose.h + p.relDeg) * MG_DEG2RAD, a2 = (pose.h + q.relDeg) * MG_DEG2RAD;
          float x1 = cosf(a1) * p.rangeCm, y1 = sinf(a1) * p.rangeCm;
          float x2 = cosf(a2) * q.rangeCm, y2 = sinf(a2) * q.rangeCm;
          float dx = x2 - x1, dy = y2 - y1;
          float L = sqrtf(dx * dx + dy * dy);
          if (L < 4.0f || L > 80.0f) continue;
          if (fabsf(q.rangeCm - p.rangeCm) > 0.8f * L) continue;       // powierzchnia niemal wzdłuż promienia: niewiarygodne
          float phi = atan2f(dy, dx);
          if (pass == 0) { sc += L * cosf(4.0f * phi); ss += L * sinf(4.0f * phi); sw += L; }
          else {
            // Przebieg 2: tylko długie cięciwy zgodne ze zgrubną orientacją (±8°) - one dają dokładny kąt
            float e = mgWrapPm45(phi / MG_DEG2RAD - theta);
            if (fabsf(e) > 8.0f || L < 8.0f) continue;
            c2 += L * L * cosf(4.0f * phi); s2 += L * L * sinf(4.0f * phi); w2 += L * L;   // waga ~ L^2 (szum kąta ~ 1/L)
          }
        }
      }
      if (pass == 0) {
        if (sw < 150.0f) return false;                              // za mało powierzchni, żeby coś twierdzić
        conf = sqrtf(sc * sc + ss * ss) / sw;
        theta = mgWrapPm45(atan2f(ss, sc) / 4.0f / MG_DEG2RAD);
      } else if (w2 > 3000.0f) {
        theta = mgWrapPm45(atan2f(s2, c2) / 4.0f / MG_DEG2RAD);
        if (prec) *prec = w2;   // suma L^2 długich, zgodnych cięciw: im większa, tym dokładniejszy kąt
      }
    }
    return true;
  }

  // ---- Globalna lokalizacja ("porwany robot") ------------------------------------------------
  // Szuka pozycji skanu w CAŁEJ znanej mapie, a nie tylko w pobliżu odometrii. Kandydaci: znane wolne
  // komórki (co ~10 cm) x kierunki. Kierunki bierzemy ze ścian skanu (modulo 90 st., mapa jest wyrównana
  // do osi), a gdy ściany są nieczytelne - co 10 st. Najlepsze szczyty dopracowuje refinePose.
  // Wynik odrzucamy, gdy jest niejednoznaczny (drugi, odległy szczyt prawie tak dobry - np. symetryczny pokój).
  struct GlobalInfo {
    bool ok; Pose2 pose; float score, second; int hits; int cands; bool byWalls; const char* why;
  };
  void (*yieldHook)();          // wołane co jakiś czas podczas długiego przeszukiwania (oddaje procesor)

  // hintR > 0: użytkownik wskazał miejsce (prior.x, prior.y) - szukamy tylko w jego promieniu, za to we wszystkich kierunkach.
  bool globalLocalize(const Pose2& prior, const Beam* b, int n, GlobalInfo& gi, float hintR = 0.0f) const {
    gi.ok = false; gi.pose = prior; gi.score = gi.second = 0; gi.hits = 0; gi.cands = 0; gi.byWalls = false; gi.why = "";
    if (!cells) { gi.why = "brak mapy"; return false; }
    for (int i = 0; i < n; i++) if (b[i].kind == 1) gi.hits++;
    uint32_t occ, fr; stats(occ, fr);
    if (occ < 40) { gi.why = "mapa jest jeszcze za mala"; return false; }
    if (gi.hits < SCAN_MATCH_MIN_HITS) { gi.why = "za malo trafien w skanie"; return false; }

    // kierunki
    float heads[36]; int nh = 0;
    float th, conf;
    Pose2 zero = {0, 0, 0};
    if (hintR > 0) {
      for (int k = 0; k < 36; k++) heads[nh++] = 10.0f * k;
    } else if (refValid && wallOrientation(zero, b, n, th, conf) && conf >= 0.5f) {
      // przy h=0 th to orientacja ścian względem robota; w mapie ściany są na 0 (mod 90) => h = -th + k*90
      for (int k = 0; k < 4; k++) heads[nh++] = mgWrap360(-th + 90.0f * k);
      gi.byWalls = true;
    } else {
      for (int k = 0; k < 36; k++) heads[nh++] = 10.0f * k;
    }
    const float stepXY = hintR > 0 ? 10.0f : (gi.byWalls ? 10.0f : 15.0f);

    // szczyty (top-K, rozdzielone)
    const int K = 8;
    Pose2 pk[K]; float ps[K]; int np = 0;
    int cnt = 0;
    for (int cy = 0; cy < MAP_CELLS; cy++) {
      for (int cx = 0; cx < MAP_CELLS; cx++) {
        float wx = (cx - MAP_CELLS / 2 + 0.5f) * MAP_CELL_CM, wy = (cy - MAP_CELLS / 2 + 0.5f) * MAP_CELL_CM;
        if (hintR > 0) { if (hypotf(wx - prior.x, wy - prior.y) > hintR) continue; if (at(cx, cy) >= L_OCC_T) continue; }
        else if (at(cx, cy) > L_FREE_T) continue;            // tylko znane wolne
        // siatka ~stepXY: bierzemy co 2 / 3 komórkę
        int stc = (int)(stepXY / MAP_CELL_CM + 0.5f);
        if ((cx % stc) != 0 || (cy % stc) != 0) continue;
        // robot musi się zmieścić: sąsiedzi (promień ~8 cm) nie mogą być zajęci
        bool clear = true;
        for (int dy = -2; dy <= 2 && clear; dy++) for (int dx = -2; dx <= 2; dx++) if (at(cx + dx, cy + dy) >= L_OCC_T) { clear = false; break; }
        if (!clear) continue;
        if ((++cnt & 63) == 0 && yieldHook) yieldHook();
        for (int hi = 0; hi < nh; hi++) {
          Pose2 c = {wx, wy, heads[hi]};
          float sc = scoreAt(c, b, n);
          gi.cands++;
          // wstaw do top-K; szczyt "blisko" istniejącego (30 cm / 25 st.) zastępuje go tylko gdy lepszy
          int near = -1;
          for (int j = 0; j < np; j++) {
            float ddx = pk[j].x - c.x, ddy = pk[j].y - c.y;
            float dh = fabsf(mgWrapPm45(pk[j].h - c.h)) ; float dh2 = fabsf(fmodf(fabsf(pk[j].h - c.h), 360.0f)); if (dh2 > 180) dh2 = 360 - dh2;
            (void)dh;
            if (ddx * ddx + ddy * ddy < 900.0f && dh2 < 25.0f) { near = j; break; }
          }
          if (near >= 0) { if (sc > ps[near]) { pk[near] = c; ps[near] = sc; } }
          else if (np < K) { pk[np] = c; ps[np] = sc; np++; }
          else {
            int w = 0; for (int j = 1; j < np; j++) if (ps[j] < ps[w]) w = j;
            if (sc > ps[w]) { pk[w] = c; ps[w] = sc; }
          }
        }
      }
    }
    if (np == 0) { gi.why = "brak wolnych miejsc na mapie"; return false; }

    // dopracowanie szczytów
    Pose2 rp[K]; float rs[K];
    for (int j = 0; j < np; j++) {
      Pose2 out; float base, best;
      if (yieldHook) yieldHook();
      refinePose(pk[j], b, n, 12.0f, (gi.byWalls && hintR <= 0) ? 6.0f : 8.0f, out, base, best);
      if (best < base) { out = pk[j]; best = base; }
      rp[j] = out; rs[j] = scoreFull(out, b, n);
    }
    int bi = 0; for (int j = 1; j < np; j++) if (rs[j] > rs[bi]) bi = j;
    float second = 0;
    for (int j = 0; j < np; j++) {
      if (j == bi) continue;
      float ddx = rp[j].x - rp[bi].x, ddy = rp[j].y - rp[bi].y;
      float dh = fabsf(fmodf(fabsf(rp[j].h - rp[bi].h), 360.0f)); if (dh > 180) dh = 360 - dh;
      if (ddx * ddx + ddy * ddy < 900.0f && dh < 20.0f) continue;   // ten sam szczyt
      if (rs[j] > second) second = rs[j];
    }
    gi.pose = rp[bi]; gi.score = rs[bi]; gi.second = second;
    if (gi.score < 0.55f * gi.hits || gi.score < 8.0f) { gi.why = "skan nie pasuje do zadnego miejsca mapy"; return false; }
    if (second >= (hintR > 0 ? 0.93f : 0.85f) * gi.score) { gi.why = "niejednoznaczne (podobne miejsce gdzie indziej, np. symetryczny pokoj) - przesun robota, zeby widzial cos charakterystycznego, albo ustaw pozycje recznie"; return false; }
    gi.ok = true;
    return true;
  }

  struct LocalizeInfo {
    int result;            // 0 brak, 1 zastosowano korektę z dopasowania, 2 odrzucono, 3 pominięto
    float dx, dy, dh;      // korekta z dopasowania do mapy
    bool wallApplied, wallMeasured; float wallDelta, wallConf;   // korekta kierunku względem ścian (measured: kierunek zmierzony, nawet jeśli różnica < 1 st.)
    bool wallRefSet;       // ten skan ustalił orientację odniesienia
  };

  // Wyznacza pozycję robota dla skanu: (1) korekta kierunku względem ścian, (2) dopasowanie do mapy.
  // Nie zapisuje skanu do mapy (to robi wołający), poza ustaleniem orientacji odniesienia.
  Pose2 localize(const Pose2& pose, const Beam* b, int n, bool locate, LocalizeInfo& info) {
    info.result = 3; info.dx = info.dy = info.dh = 0; info.wallApplied = false; info.wallMeasured = false; info.wallDelta = 0; info.wallConf = 0; info.wallRefSet = false;
    Pose2 cur = pose;
    int hits = 0;
    for (int i = 0; i < n; i++) if (b[i].kind == 1) hits++;
#if WALL_FIX_ENABLED
    float th, conf, prec;
    if (wallOrientation(cur, b, n, th, conf, &prec)) {
      info.wallConf = conf;
      float d = mgWrapPm45(th);   // orientacja ścian w układzie mapy powinna wynosić 0: odchyłka = błąd naszego kierunku
      if (!refValid) {
        // Pierwszy dobry skan wyznacza układ mapy: obracamy układ tak, by ściany leżały wzdłuż osi.
        // Skan z początku nie ma jeszcze dryfu, a błędy tego jednego skanu naprawiają kolejne (cel zawsze 0).
        if (conf >= 0.6f) {
          cur.h = mgWrap360(cur.h - d);
          startHeading = mgWrap360(startHeading - d);
          refValid = true; info.wallRefSet = true; info.wallDelta = d; info.wallApplied = (fabsf(d) >= 1.0f);
        }
      } else if (conf >= WALL_FIX_MIN_CONF) {
        info.wallDelta = d; info.wallMeasured = true;
        float lim = locate ? 40.0f : WALL_FIX_MAX_DEG;
        if (fabsf(d) <= lim && fabsf(d) >= 1.0f) {   // poniżej 1 stopnia to szum pomiaru
          cur.h = mgWrap360(cur.h - d * (conf >= 0.6f ? 1.0f : 0.7f));
          info.wallApplied = true;
        }
      }
    }
#endif
#if SCAN_MATCH_ENABLED
    uint32_t occ, fr;
    stats(occ, fr);
    if (occ > 40 && hits >= SCAN_MATCH_MIN_HITS) {
      float radius = locate ? 30.0f : (float)SCAN_MATCH_MAX_SHIFT_CM;
      float radiusA = locate ? 15.0f : (float)SCAN_MATCH_MAX_TURN_DEG;
      Pose2 best; float base, score;
      refinePose(cur, b, n, radius, radiusA, best, base, score);
      float dh = best.h - cur.h;
      if (dh > 180) dh -= 360;
      if (dh < -180) dh += 360;
      info.dx = best.x - cur.x; info.dy = best.y - cur.y; info.dh = dh;
      if (mgMatchAccepted(base, score, hits)) { cur = best; info.result = 1; }
      else info.result = 2;
    }
#endif
    return cur;
  }

 private:
  float cellWeight(int cx, int cy) const {
    int v = at(cx, cy);
    return v > 0 ? (v > 60 ? 1.0f : v / 60.0f) : 0.0f;
  }
  float beamScore(float wx, float wy) const {
    int cx = toCell(wx), cy = toCell(wy);
    if (!inside(cx, cy)) return 0;
    float c = cellWeight(cx, cy);
    float nb = cellWeight(cx - 1, cy);
    float t;
    if ((t = cellWeight(cx + 1, cy)) > nb) nb = t;
    if ((t = cellWeight(cx, cy - 1)) > nb) nb = t;
    if ((t = cellWeight(cx, cy + 1)) > nb) nb = t;
    float s = c > 0.5f * nb ? c : 0.5f * nb;
    if (at(cx, cy) <= -8) s -= 0.3f;      // koniec promienia w znanej wolnej przestrzeni
    return s;
  }
};

// Podglad mapy z karty: 2 bity na komorke, ale w osobnych kawalkach po 50 B (wiersz siatki) - nie wymaga duzego,
// jednolitego bloku RAM (przy pofragmentowanej pamieci jeden blok 10 KB bywal niedostepny). 0 nieznane, 1 wolne, 2 zajete
struct PackedMap { uint8_t* row[MAP_CELLS]; };
static void packedFree(PackedMap* m) { if (!m) return; for (int i = 0; i < MAP_CELLS; i++) free(m->row[i]); free(m); }
static PackedMap* packedAlloc() {
  PackedMap* m = (PackedMap*)calloc(1, sizeof(PackedMap));
  if (!m) return NULL;
  for (int i = 0; i < MAP_CELLS; i++) { m->row[i] = (uint8_t*)calloc((MAP_CELLS + 3) / 4, 1); if (!m->row[i]) { packedFree(m); return NULL; } }
  return m;
}
static inline uint8_t packedCls(const PackedMap* m, size_t i) { size_t c = i % MAP_CELLS; return (m->row[i / MAP_CELLS][c >> 2] >> ((c & 3) * 2)) & 3; }
static inline int8_t mapPackedAt(const PackedMap* m, size_t i) { uint8_t v = packedCls(m, i); return v == 2 ? 127 : (v == 1 ? -127 : 0); }
