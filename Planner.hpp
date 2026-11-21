#pragma once
// ---------------------------------------------------------------------------
// Planowanie eksploracji - czysta logika (bez Arduino), testowana na komputerze
// (test/explore_sim.cpp) na symulowanym pokoju z szumem odometrii.
//
// Idea: robot jedzie do NAJBLIŻSZEJ "granicy" (wolna komórka sąsiadująca z nieznanym),
// tam skanuje, i tak aż nie zostanie żadna osiągalna granica = pokój zamknięty.
// Trasa liczona jest na grubszej siatce 10 cm z "napompowanymi" przeszkodami (obrys robota),
// po znanej wolnej przestrzeni. Nieznane nigdy nie jest traktowane jako przejezdne.
// ---------------------------------------------------------------------------
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "MapGrid.hpp"

// Tablica podzielona na kawalki (kazdy osobny malloc) - nie wymaga duzego ciaglego bloku RAM
template <typename T, int CH, int COUNT> struct Chunked {
  enum { NC = (COUNT + CH - 1) / CH };
  T* c[NC];
  Chunked() { for (int i = 0; i < NC; i++) c[i] = NULL; }
  bool alloc() {
    for (int i = 0; i < NC; i++) if (!c[i]) { c[i] = (T*)malloc(sizeof(T) * CH); if (!c[i]) { release(); return false; } }
    return true;
  }
  void release() { for (int i = 0; i < NC; i++) { free(c[i]); c[i] = NULL; } }
  T& operator[](int i) { return c[i / CH][i % CH]; }
  const T& operator[](int i) const { return c[i / CH][i % CH]; }
};

#define PLAN_DS          2                              // komórek mapy na komórkę planera
#define PLAN_N           (MAP_CELLS / PLAN_DS)
#define PLAN_QN          2048                           // kolejka BFS jako bufor cykliczny (4 KB); front fali jest znacznie mniejszy niz cala siatka
#define PLAN_CELL_CM     (MAP_CELL_CM * PLAN_DS)
#define PLAN_INFLATE     2                              // 2 x 10 cm = 20 cm > promień obrysu 25x15 (14,6 cm)
#define PLAN_ESCAPE      4                              // 4 x 10 cm: wokół robota inflacja nie blokuje wyjścia
#define PLAN_MAX_WP      12
#define PLAN_BLACKLIST   24
#define PLAN_BLACKLIST_RADIUS_CM 30.0f
#define PLAN_MIN_UNKNOWN 6                              // ile nieznanych w oknie 5x5, by granica była warta wizyty

#ifndef EXPLORE_SEGMENT_CM
#define EXPLORE_SEGMENT_CM   40.0f                      // maks. odcinek jazdy bez ponownego planowania
#endif
#ifndef EXPLORE_ARRIVE_CM
#define EXPLORE_ARRIVE_CM    15.0f
#endif
#ifndef EXPLORE_TURN_MIN_DEG
#define EXPLORE_TURN_MIN_DEG 5.0f
#endif
#ifndef EXPLORE_MAX_STEPS
#define EXPLORE_MAX_STEPS    250
#endif

enum { PLAN_OK = 0, PLAN_DONE = 1, PLAN_ERROR = 2, PLAN_NOMEM = 3 };

struct PlanResult {
  int status;
  float tx, ty;                       // cel (komórka granicy)
  int n;                              // liczba punktów trasy
  float wx[PLAN_MAX_WP], wy[PLAN_MAX_WP];
  int frontiers;                      // ile komórek granicy jest osiągalnych
};

class Planner {
 public:
  float bx[PLAN_BLACKLIST], by[PLAN_BLACKLIST];
  int bn, bhead;
  bool allowUnknown = false;   // powrót awaryjny: nieznane komórki traktuj jak wolne (czujniki i tak zatrzymają robota przed przeszkodą)
  bool goalMode = false; float goalX = 0, goalY = 0;   // tryb "jedź do punktu" (powrót na start) zamiast szukania granic
  int inflate = PLAN_INFLATE;   // zmniejszany awaryjnie, gdy robot utknął w ciasnym miejscu

  Planner() : bn(0), bhead(0) {}
  void reset() { bn = 0; bhead = 0; }
  // Bufory (10 KB + 4 KB) tylko na czas planowania. Alokowane w malych kawalkach (100 B / 256 B), a nie jako dwa duze bloki:
  // pofragmentowana sterta (Wi-Fi + BLE + WWW) nie miala dwoch ciaglych blokow 10 KB i 8 KB, choc wolnego RAM bylo 25 KB.
  Chunked<uint8_t, PLAN_N, PLAN_N * PLAN_N> bufSt;
  Chunked<uint16_t, 128, PLAN_QN> bufQ;
  bool acquire() { return bufSt.alloc() && bufQ.alloc(); }
  void release() { bufSt.release(); bufQ.release(); }
  ~Planner() { release(); }
  void blacklist(float x, float y) {
    if (bn < PLAN_BLACKLIST) { bx[bn] = x; by[bn] = y; bn++; }
    else { bx[bhead] = x; by[bhead] = y; bhead = (bhead + 1) % PLAN_BLACKLIST; }
  }
  bool isBlacklisted(float x, float y) const {
    for (int i = 0; i < bn; i++) {
      float dx = x - bx[i], dy = y - by[i];
      if (dx * dx + dy * dy < PLAN_BLACKLIST_RADIUS_CM * PLAN_BLACKLIST_RADIUS_CM) return true;
    }
    return false;
  }

  static int toP(float v) { return (int)floorf(v / PLAN_CELL_CM) + PLAN_N / 2; }
  static float fromP(int c) { return (c - PLAN_N / 2 + 0.5f) * PLAN_CELL_CM; }

  void plan(const OccGrid& g, const Pose2& robot, PlanResult& out) {
    out.status = PLAN_ERROR; out.n = 0; out.frontiers = 0; out.tx = out.ty = 0;
    if (!g.cells) return;
    const int N = PLAN_N, NN = PLAN_N * PLAN_N;
    // st: bity 0-1 klasa, bit 2 zablokowane, bity 4-7 kierunek rodzica (15 = nieodwiedzone).
    // Bufory trzymane między planowaniami (acquire/release), żeby nie fragmentować sterty.
    if (!acquire()) { out.status = PLAN_NOMEM; return; }
    Chunked<uint8_t, PLAN_N, PLAN_N * PLAN_N>& st = bufSt;
    Chunked<uint16_t, 128, PLAN_QN>& q = bufQ;

    // 1. gruba siatka: zajęta jeśli którakolwiek "pewna" komórka zajęta; wolna jeśli >= 3 z 4 znane.
    //    Pewna = z sąsiadem zajętym albo trafiona wielokrotnie. Pojedyncze, samotne trafienia
    //    (fałszywe odczyty) nie blokują planera - inaczej drobny szum dzieliłby pokój na kawałki.
    for (int py = 0; py < N; py++) for (int px = 0; px < N; px++) {
      int occ = 0, known = 0;
      for (int dy = 0; dy < PLAN_DS; dy++) for (int dx = 0; dx < PLAN_DS; dx++) {
        int fx = px * PLAN_DS + dx, fy = py * PLAN_DS + dy;
        uint8_t c = OccGrid::classOf(g.cells[fy * MAP_CELLS + fx]);
        if (c == 2 && solidOcc(g, fx, fy)) occ++;
        if (c) known++;
      }
      st[py * N + px] = occ ? 2 : (known >= 3 ? 1 : 0);
    }
    // 2. blokady: nieznane + przeszkody napompowane o obrys robota
    for (int i = 0; i < NN; i++) if ((st[i] & 3) == 0 && !allowUnknown) st[i] |= 4;
    for (int py = 0; py < N; py++) for (int px = 0; px < N; px++) {
      if ((st[py * N + px] & 3) != 2) continue;
      for (int dy = -inflate; dy <= inflate; dy++) for (int dx = -inflate; dx <= inflate; dx++) {
        int x = px + dx, y = py + dy;
        if (x >= 0 && y >= 0 && x < N && y < N) st[y * N + x] |= 4;
      }
    }
    // 3. Robot stoi już w strefie napompowanej wokół ściany (np. 15 cm od niej)? Wtedy pozwalamy mu
    //    z niej "wyjść": wolne komórki w promieniu 40 cm nie są blokowane. Gdy stoi poza strefą,
    //    obowiązuje pełna inflacja (żadnych wyjątków - inaczej robot jeździłby tuż przy ścianach).
    int sx = toP(robot.x), sy = toP(robot.y);
    if (sx < 0 || sy < 0 || sx >= N || sy >= N) return;
    if (st[sy * N + sx] & 4) {
      for (int dy = -PLAN_ESCAPE; dy <= PLAN_ESCAPE; dy++) for (int dx = -PLAN_ESCAPE; dx <= PLAN_ESCAPE; dx++) {
        if (dx * dx + dy * dy > PLAN_ESCAPE * PLAN_ESCAPE) continue;
        int x = sx + dx, y = sy + dy;
        if (x >= 0 && y >= 0 && x < N && y < N && (st[y * N + x] & 3) == 1) st[y * N + x] &= ~4;
      }
    }
    // Robot fizycznie tu stoi, więc jego komórka jest wolna - także gdy mapa jej nie zna (jazda ręczna poza
    // zmapowanym obszarem) albo ma tam fałszywą przeszkodę. Najbliższe nieznane komórki (20 cm) też otwieramy.
    for (int dy = -2; dy <= 2; dy++) for (int dx = -2; dx <= 2; dx++) {
      int x = sx + dx, y = sy + dy;
      if (x < 0 || y < 0 || x >= N || y >= N) continue;
      if (dx == 0 && dy == 0) st[y * N + x] = 1;
      else if ((st[y * N + x] & 3) == 0) st[y * N + x] = 1;
    }
    st[sy * N + sx] &= ~4;
    if (goalMode) {   // okolica celu (np. start tuż przy ścianie) musi być osiągalna, jeśli nie jest przeszkodą
      for (int py = 0; py < N; py++) for (int px = 0; px < N; px++)
        if ((st[py * N + px] & 3) == 1 && hypotf(fromP(px) - goalX, fromP(py) - goalY) <= 6.0f) st[py * N + px] &= ~4;
    }

    // 4. BFS po przejezdnych komórkach; pierwsza napotkana granica = najbliższa
    static const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    for (int i = 0; i < NN; i++) st[i] |= 0xF0;
    int head = 0, tail = 0;
    q[tail++ & (PLAN_QN - 1)] = (uint16_t)(sy * N + sx);
    st[sy * N + sx] = (st[sy * N + sx] & 0x0F) | 0x80;
    int target = -1;
    float bestD = 1e9f; int bestCell = -1;
    while (head < tail) {
      int cur = q[head++ & (PLAN_QN - 1)];
      int cx = cur % N, cy = cur / N;
      if (goalMode) {   // cel = osiągalna komórka najbliżej punktu (sam punkt może leżeć w strefie przy ścianie)
        float dd = hypotf(fromP(cx) - goalX, fromP(cy) - goalY);
        if (dd < bestD - 0.01f) { bestD = dd; bestCell = cur; }
      } else if (cur != sy * N + sx && isFrontier(st, cx, cy)) {
        out.frontiers++;
        if (target < 0 && !isBlacklisted(fromP(cx), fromP(cy))) target = cur;
      }
      for (int d = 0; d < 8; d++) {
        int nx = cx + DX[d], ny = cy + DY[d];
        if (nx < 0 || ny < 0 || nx >= N || ny >= N) continue;
        int ni = ny * N + nx;
        if ((st[ni] & 0xF0) != 0xF0 || (st[ni] & 4)) continue;
        if (d >= 4 && ((st[cy * N + nx] & 4) || (st[ny * N + cx] & 4))) continue;   // bez ścinania rogów
        if (tail - head >= PLAN_QN) continue;                   // kolejka pelna: pomijamy (komorka zostaje nieodwiedzona)
        st[ni] = (st[ni] & 0x0F) | (uint8_t)(d << 4);
        q[tail++ & (PLAN_QN - 1)] = (uint16_t)ni;
      }
    }

    if (goalMode && bestCell >= 0 && bestCell != sy * N + sx) target = bestCell;   // brak lepszej komórki = już najbliżej
    if (target < 0) { out.status = PLAN_DONE; return; }

    // 5. trasa od celu do robota, potem "naciąganie sznurka" (najdalszy widoczny punkt)
    int len = 0;
    for (int c = target; c != sy * N + sx; ) {
      if (len >= PLAN_QN) { out.status = PLAN_ERROR; return; }
      q[len++] = (uint16_t)c;                      // BFS skończył - kolejka jest wolna do ponownego użycia
      int d = st[c] >> 4;
      c = (c / N - DY[d]) * N + (c % N - DX[d]);
    }
    // q[0] = cel ... q[len-1] = komórka sąsiadująca ze startem; odwracamy
    for (int i = 0; i < len / 2; i++) { uint16_t t = q[i]; q[i] = q[len - 1 - i]; q[len - 1 - i] = t; }

    int curC = sy * N + sx, idx = 0;
    out.n = 0;
    while (idx < len && out.n < PLAN_MAX_WP) {
      int far = idx;
      for (int j = len - 1; j >= idx; j--) { if (lineClear(st, curC, q[j])) { far = j; break; } }
      curC = q[far];
      out.wx[out.n] = fromP(curC % N);
      out.wy[out.n] = fromP(curC / N);
      out.n++;
      idx = far + 1;
    }
    out.tx = fromP(target % N);
    out.ty = fromP(target / N);
    out.status = PLAN_OK;
  }

 private:
  static bool solidOcc(const OccGrid& g, int fx, int fy) {
    if (g.at(fx, fy) >= 60) return true;
    for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++) {
      if (!dx && !dy) continue;
      if (g.at(fx + dx, fy + dy) >= L_OCC_T) return true;
    }
    return false;
  }

  template <class S> static bool isFrontier(const S& st, int cx, int cy) {
    const int N = PLAN_N;
    if ((st[cy * N + cx] & 3) != 1) return false;
    bool adj = false;
    static const int AX[4] = {1, -1, 0, 0}, AY[4] = {0, 0, 1, -1};
    for (int d = 0; d < 4; d++) {
      int x = cx + AX[d], y = cy + AY[d];
      if (x >= 0 && y >= 0 && x < N && y < N && (st[y * N + x] & 3) == 0) { adj = true; break; }
    }
    if (!adj) return false;
    int unknown = 0;
    for (int dy = -2; dy <= 2; dy++) for (int dx = -2; dx <= 2; dx++) {
      int x = cx + dx, y = cy + dy;
      if (x >= 0 && y >= 0 && x < N && y < N && (st[y * N + x] & 3) == 0) unknown++;
    }
    return unknown >= PLAN_MIN_UNKNOWN;
  }

  // Czy odcinek między komórkami nie przechodzi przez zablokowane (Bresenham, bez ścinania rogów)
  template <class S> static bool lineClear(const S& st, int a, int b) {
    const int N = PLAN_N;
    int x0 = a % N, y0 = a / N, x1 = b % N, y1 = b / N;
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int x = x0, y = y0;
    while (!(x == x1 && y == y1)) {
      int e2 = 2 * err;
      int nx = x, ny = y;
      bool mx = false, my = false;
      if (e2 > -dy) { err -= dy; nx += sx; mx = true; }
      if (e2 < dx) { err += dx; ny += sy; my = true; }
      if (mx && my && ((st[y * N + nx] & 4) || (st[ny * N + x] & 4))) return false;
      if (st[ny * N + nx] & 4) return false;
      x = nx; y = ny;
    }
    return true;
  }
};

// ---------------------------------------------------------------------------
// Eksploracja: jeden krok = (opcjonalny skan) + plan + obrót + odcinek jazdy.
// R to "robot": prawdziwy (Explore.hpp) albo symulowany (test/explore_sim.cpp):
//   Pose2 pose();  const OccGrid& map();  bool scan();
//   bool turn(float deg);        // + = w lewo; false = nie udało się / przerwano
//   bool drive(float cm);        // false = zatrzymany przez przeszkodę / przerwano
//   bool aborted();
//   void note(const char* text);   // wpis do dziennika (widoczny w panelu)
// ---------------------------------------------------------------------------
enum { EX_RUNNING = 0, EX_DONE = 1, EX_STUCK = 2, EX_ERROR = 3, EX_ABORT = 4, EX_LIMIT = 5 };

class Explorer {
 public:
  Planner planner;
  PlanResult last;
  bool needScan, arrived, haveTarget;
  float sinceScan, tx, ty;
  int stuck, totalStuck, steps, resets, tight;

  bool homing; float hx, hy, hh;
  bool noHeading; int maxVerifies;   // cel pośredni (nie start): bez ustawiania kierunku, jedno sprawdzenie skanem
  bool wideScan; int verifies;   // powrót: pierwszy skan i skany przy dojeździe są "szerokie" (większe okno dopasowania)
  Explorer() { homing = false; hx = hy = hh = 0; noHeading = false; maxVerifies = 2; begin(); }
  // Dojazd do punktu (np. kolejny punkt skanu): jak powrót, ale bez ustawiania kierunku; na końcu jeden skan sprawdzający i uzupełniający mapę
  void beginGoal(float x, float y, bool skipFirstScan) {
    beginHome(x, y, 0.0f); noHeading = true; maxVerifies = 1;
    if (skipFirstScan) { needScan = false; wideScan = false; }
  }
  // Powrót do punktu (np. startu): ta sama maszyna stanów, ale cel to punkt, a na końcu ustawienie kierunku
  void beginHome(float x, float y, float h) { begin(); wideScan = true; homing = true; hx = x; hy = y; hh = h; planner.goalMode = true; planner.goalX = x; planner.goalY = y; }
  void begin() {
    homing = false; planner.goalMode = false; wideScan = false; verifies = 0; noHeading = false; maxVerifies = 2;
    planner.reset();
    needScan = true; arrived = false; haveTarget = false;
    sinceScan = 0; tx = ty = 0; stuck = totalStuck = steps = resets = tight = 0;
    last.status = PLAN_ERROR; last.n = 0; last.frontiers = 0;
  }

  template <class R>
  int stepHome(R& r) {
    char nb[88];
    if (needScan) {
      bool ok = wideScan ? r.locate() : r.scan();
      wideScan = false;
      if (r.aborted()) return EX_ABORT;
      if (!ok) { r.note("skan sie nie udal"); if (++totalStuck >= 12) return EX_STUCK; }
      needScan = false; sinceScan = 0;
    }
    Pose2 p = r.pose();
    float d = hypotf(hx - p.x, hy - p.y);
    if (d < 8.0f && verifies < maxVerifies) {   // wygląda na to, że jesteśmy na miejscu - sprawdź skanem, bo odometria mogła się rozjechać
      verifies++; needScan = true; wideScan = true;
      r.note("powrot: weryfikuje pozycje skanem");
      return EX_RUNNING;
    }
    if (d < 8.0f) {
      float err = hh - p.h;
      while (err > 180) err -= 360;
      while (err < -180) err += 360;
      if (!noHeading && fabsf(err) > EXPLORE_TURN_MIN_DEG) r.turn(err);
      snprintf(nb, sizeof(nb), "powrot: dotarlem na start (odleglosc %.0f cm)", d);
      r.note(nb);
      return r.aborted() ? EX_ABORT : EX_DONE;
    }
    planner.goalMode = true; planner.goalX = hx; planner.goalY = hy;
    planner.plan(r.map(), p, last);
    if (last.status != PLAN_OK && tight < 3 && (last.status != PLAN_DONE || d > 15.0f)) {
      tight++; planner.inflate = 1;
      r.note("powrot: brak drogi - probuje z mniejszym marginesem");
      planner.plan(r.map(), p, last);
    }
    if (last.status != PLAN_OK && last.status != PLAN_NOMEM && !(last.status == PLAN_DONE && d <= 40.0f)) {
      // brak drogi po znanej mapie (np. robot wyjechał ręcznie poza zmapowany obszar): idź przez nieznane
      planner.allowUnknown = true; planner.inflate = PLAN_INFLATE;
      r.note("powrot: brak drogi po znanej mapie - probuje przez nieznane pola");
      planner.plan(r.map(), p, last);
      planner.allowUnknown = false;
    }
    if (last.status == PLAN_OK) planner.inflate = PLAN_INFLATE;
    if (last.status == PLAN_NOMEM) { r.note("planer: za malo wolnej pamieci RAM na plan trasy"); return EX_ERROR; }
    if (last.status == PLAN_DONE && d <= 40.0f) {   // jesteśmy już najbliżej, jak się da (start leży przy ścianie/przeszkodzie)
      float err = hh - p.h;
      while (err > 180) err -= 360;
      while (err < -180) err += 360;
      if (!noHeading && fabsf(err) > EXPLORE_TURN_MIN_DEG) r.turn(err);
      snprintf(nb, sizeof(nb), "powrot: stoje najblizej startu jak sie da (%.0f cm od niego)", d);
      r.note(nb);
      return r.aborted() ? EX_ABORT : EX_DONE;
    }
    if (last.status != PLAN_OK) { r.note("powrot: nie znalazlem drogi na start po znanej mapie"); return EX_STUCK; }
    tx = hx; ty = hy; haveTarget = true;
    float gx = last.wx[0], gy = last.wy[0];
    float dist = hypotf(gx - p.x, gy - p.y);
    float bearing = atan2f(gy - p.y, gx - p.x) / MG_DEG2RAD;
    float err = bearing - p.h;
    while (err > 180) err -= 360;
    while (err < -180) err += 360;
    bool ok = true;
    if (fabsf(err) > EXPLORE_TURN_MIN_DEG) ok = r.turn(err);
    if (r.aborted()) return EX_ABORT;
    if (ok) {
      float seg = dist < EXPLORE_SEGMENT_CM ? dist : EXPLORE_SEGMENT_CM;
      if (seg < 3.0f) seg = 3.0f;
      Pose2 before = r.pose();
      ok = r.drive(seg);
      Pose2 after = r.pose();
      sinceScan += hypotf(after.x - before.x, after.y - before.y);
    }
    if (r.aborted()) return EX_ABORT;
    if (!ok) {
      stuck++; totalStuck++; needScan = true;
      Pose2 q = r.pose();
      if (stuck >= 2 && hypotf(hx - q.x, hy - q.y) <= 40.0f) {   // blisko startu i dalej nie da się (ściana/przeszkoda) - wystarczy
        snprintf(nb, sizeof(nb), "powrot: zatrzymany przez przeszkode %.0f cm od startu - koniec", hypotf(hx - q.x, hy - q.y));
        r.note(nb);
        return EX_DONE;
      }
      if (totalStuck >= 12) return EX_STUCK;
    } else stuck = 0;
    if (sinceScan >= gScanEveryCm) needScan = true;
    return EX_RUNNING;
  }

  template <class R>
  int step(R& r) {
    if (r.aborted()) return EX_ABORT;
    if (steps++ > EXPLORE_MAX_STEPS) return EX_LIMIT;
    if (homing) return stepHome(r);

    if (needScan) {
      bool ok = r.scan();
      if (r.aborted()) return EX_ABORT;
      if (!ok) r.note("skan sie nie udal");
      needScan = false;
      sinceScan = 0;
      if (arrived && haveTarget) { planner.blacklist(tx, ty); haveTarget = false; }
      arrived = false;
      if (!ok) { if (++totalStuck >= 12) return EX_STUCK; }
    }

    Pose2 p = r.pose();
    planner.plan(r.map(), p, last);
    char nb[88];
    if (last.status == PLAN_NOMEM) { r.note("planer: za malo wolnej pamieci RAM na plan trasy"); return EX_ERROR; }
    if (last.status == PLAN_ERROR) { r.note("planer: blad (robot stoi w przeszkodzie na mapie - zrob skan lub Nowa mapa)"); return EX_ERROR; }
    if (last.status == PLAN_DONE && tight < 3) {
      tight++;
      // nic nie znaleziono przy pelnym marginesie: sprobuj z mniejszym (robot mogl utknac w ciasnym miejscu)
      planner.inflate = 1;
      r.note("planer: brak granic - probuje z mniejszym marginesem");
      planner.plan(r.map(), p, last);
    }
    if (last.status == PLAN_OK) planner.inflate = PLAN_INFLATE;
    if (last.status == PLAN_DONE && planner.bn > 0 && resets < 2) {
      // granice odrzucone wczesniej (np. przez chwilowy szum) dostaja druga szanse
      resets++;
      planner.bn = 0;
      r.note("planer: brak granic - czyszcze liste wykluczen i probuje ponownie");
      return EX_RUNNING;
    }
    if (last.status == PLAN_DONE) {
      snprintf(nb, sizeof(nb), "planer: brak osiagalnych granic (na liscie wykluczen: %d) - koniec", planner.bn);
      r.note(nb);
      return EX_DONE;
    }
    tx = last.tx; ty = last.ty; haveTarget = true;
    snprintf(nb, sizeof(nb), "plan: cel (%.0f, %.0f), granic %d, punktow trasy %d", tx, ty, last.frontiers, last.n);
    r.note(nb);

    float gx = last.wx[0], gy = last.wy[0];
    float dist = hypotf(gx - p.x, gy - p.y);
    float distTarget = hypotf(tx - p.x, ty - p.y);
    if (distTarget < EXPLORE_ARRIVE_CM) { needScan = true; arrived = true; return EX_RUNNING; }

    float bearing = atan2f(gy - p.y, gx - p.x) / MG_DEG2RAD;
    float err = bearing - p.h;
    while (err > 180) err -= 360;
    while (err < -180) err += 360;
    bool ok = true;
    if (fabsf(err) > EXPLORE_TURN_MIN_DEG) {
      ok = r.turn(err);
      snprintf(nb, sizeof(nb), "obrot o %.0f st. -> %s", err, ok ? "ok" : "NIEPOWODZENIE");
      r.note(nb);
    }
    if (r.aborted()) return EX_ABORT;
    if (ok) {
      float seg = dist < EXPLORE_SEGMENT_CM ? dist : EXPLORE_SEGMENT_CM;
      if (seg < 3.0f) seg = 3.0f;
      Pose2 before = r.pose();
      ok = r.drive(seg);
      Pose2 after = r.pose();
      snprintf(nb, sizeof(nb), "jazda %.0f cm -> %s (przejechano %.0f cm)", seg, ok ? "ok" : "ZATRZYMANA",
               hypotf(after.x - before.x, after.y - before.y));
      r.note(nb);
      sinceScan += hypotf(after.x - before.x, after.y - before.y);
    }
    if (r.aborted()) return EX_ABORT;
    if (!ok) {
      stuck++; totalStuck++;
      needScan = true;                       // przeszkoda mogła być nieznana - zobacz ją
      if (stuck >= 2) { planner.blacklist(tx, ty); haveTarget = false; stuck = 0; }
      if (totalStuck >= 12) return EX_STUCK;
    } else {
      stuck = 0;
    }
    if (sinceScan >= gScanEveryCm) needScan = true;
    return EX_RUNNING;
  }
};
