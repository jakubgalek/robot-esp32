// Symulacja całego mapowania pokoju: robot z szumem odometrii, skan 360, dopasowanie skanu,
// eksploracja granic. Sprawdza, że eksploracja się KOŃCZY, pokrywa pokój i nie uderza w ściany.
//   g++ -std=c++17 -O2 -I.. explore_sim.cpp -o explore_sim && ./explore_sim
#include <cstdio>
#include <vector>
#include <random>
#include <cmath>
#include "../Planner.hpp"

struct Seg { float x1, y1, x2, y2; };
static std::vector<Seg> world;
static void addRect(float x0, float y0, float x1, float y1) {
  world.push_back({x0, y0, x1, y0}); world.push_back({x1, y0, x1, y1});
  world.push_back({x1, y1, x0, y1}); world.push_back({x0, y1, x0, y0});
}
static float distToWorld(float x, float y) {
  float best = 1e9;
  for (auto& s : world) {
    float vx = s.x2 - s.x1, vy = s.y2 - s.y1;
    float t = ((x - s.x1) * vx + (y - s.y1) * vy) / (vx * vx + vy * vy);
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    best = fminf(best, hypotf(x - (s.x1 + t * vx), y - (s.y1 + t * vy)));
  }
  return best;
}
static float castRay(float ox, float oy, float angDeg) {
  float a = angDeg * MG_DEG2RAD, dx = cosf(a), dy = sinf(a), best = 1e9;
  for (auto& s : world) {
    float sx = s.x2 - s.x1, sy = s.y2 - s.y1, den = dx * sy - dy * sx;
    if (fabsf(den) < 1e-9) continue;
    float t = ((s.x1 - ox) * sy - (s.y1 - oy) * sx) / den, u = ((s.x1 - ox) * dy - (s.y1 - oy) * dx) / den;
    if (t > 0 && u >= 0 && u <= 1 && t < best) best = t;
  }
  return best;
}
// punkt wewnątrz pokoju i poza przeszkodami (do liczenia pokrycia)
static bool insideRect(float x, float y, float x0, float y0, float x1, float y1) { return x > x0 && x < x1 && y > y0 && y < y1; }
static bool freeSpace(float x, float y) {
  if (!insideRect(x, y, -120, -150, 180, 250)) return false;
  return distToWorld(x, y) > 12.0f;
}

struct SimRobot {
  std::mt19937 rng;
  Pose2 T, O;                 // prawdziwa pozycja i pozycja z odometrii
  OccGrid map;
  int collisions = 0, scans = 0;
  float gyroScale = 1.0f;   // prawdziwy stosunek: żyroskop pokazuje 1/gyroScale prawdziwego obrotu (0.9 = czyta 10% za mało)
  float learned = 1.0f;     // skala wyuczona przez program (jak w firmware)
  float turnSinceScan = 0;
  bool inColl = false;
  float traveled = 0;
  float n(float sd) { return std::normal_distribution<float>(0, sd)(rng); }

  SimRobot(unsigned seed, float sx = 0, float sy = 0, float sh = 90) : rng(seed) { T = {sx, sy, sh}; O = T; map.begin(); map.clear(); }
  Pose2 pose() { return O; }
  const OccGrid& mapRef() { return map; }
  const OccGrid& map_() { return map; }
  bool aborted() { return false; }
  void note(const char*) {}

  bool locate() { return scan(true); }
  bool scan(bool wide = false) {
    std::vector<Beam> b;
    float sx, sy; OccGrid::sensorOrigin(T, SERVO_SENSOR_OFFSET_FWD_CM, SERVO_SENSOR_OFFSET_LEFT_CM, sx, sy);
    for (int a = 0; a <= 180; a += SCAN_STEP_DEG) for (int s = 0; s < 2; s++) {
      float rel = SERVO_DIRECTION * a + (s ? 180 : 0);
      float r = castRay(sx, sy, T.h + rel);
      Beam bm; bm.relDeg = rel; bm.sensor = s;
      float u = std::uniform_real_distribution<float>(0, 1)(rng);
      if (u < 0.015f) { bm.kind = 1; bm.rangeCm = 10 + 90 * std::uniform_real_distribution<float>(0, 1)(rng); }   // fałszywy odczyt
      else if (u < 0.05f || r > MAP_MAX_RANGE_CM) { bm.kind = 0; bm.rangeCm = MAP_NOECHO_FREE_CM; }              // brak echa
      else { bm.kind = 1; bm.rangeCm = r + n(1.2f); }
      b.push_back(bm);
    }
    int hits = 0; for (auto& x : b) hits += x.kind == 1;
    OccGrid::LocalizeInfo li;
    O = map.localize(O, b.data(), b.size(), wide, li);
    if (li.wallMeasured) learned = mgScaleUpdate(learned, li.wallDelta, turnSinceScan);
    turnSinceScan = 0;
    map.integrateScan(O, b.data(), b.size());
    scans++;
    return true;
  }

  bool turn(float deg) {
    float tdeg = deg * (1 + n(0.03f));
    // robot kończy obrót, gdy żyroskop (ze swoją skalą) pokaże cel; fizycznie obraca się o cel / (gyroScale * learned_ratio)
    float meas = deg * (1 + n(0.005f)) + n(0.3f);
    T.h = mgWrap360(T.h + meas / (gyroScale * learned));
    O.h = mgWrap360(O.h + meas);
    turnSinceScan += meas;
    return true;
  }

  bool collides(const Pose2& p) {
    float c = cosf(p.h * MG_DEG2RAD), s = sinf(p.h * MG_DEG2RAD);
    const float L = ROBOT_LENGTH_CM / 2, W = ROBOT_WIDTH_CM / 2;
    float cx[4] = {L, L, -L, -L}, cy[4] = {W, -W, W, -W};
    for (int i = 0; i < 4; i++) if (distToWorld(p.x + c * cx[i] - s * cy[i], p.y + s * cx[i] + c * cy[i]) < 1.5f) return true;
    return false;
  }

  bool drive(float cm) {
    float done = 0;
    bool blocked = false;
    while (done < cm) {
      // czujnik przedni: przeszkoda w odległości <= 15 cm od przedniej krawędzi => stop
      float fx = T.x + cosf(T.h * MG_DEG2RAD) * ROBOT_LENGTH_CM / 2, fy = T.y + sinf(T.h * MG_DEG2RAD) * ROBOT_LENGTH_CM / 2;
      float r = castRay(fx, fy, T.h);
      if (r <= 15.0f) {
        blocked = true;
        // robot dopisuje do mapy to, co zobaczył czujnik przedni
        float hx = O.x + cosf(O.h * MG_DEG2RAD) * (ROBOT_LENGTH_CM / 2 + r), hy = O.y + sinf(O.h * MG_DEG2RAD) * (ROBOT_LENGTH_CM / 2 + r);
        float ox = O.x + cosf(O.h * MG_DEG2RAD) * ROBOT_LENGTH_CM / 2, oy = O.y + sinf(O.h * MG_DEG2RAD) * ROBOT_LENGTH_CM / 2;
        map.integrateBeam(ox, oy, O.h, hypotf(hx - ox, hy - oy), true);
        map.version++;
        break;
      }
      float step = 2.0f;
      float dh = n(0.35f);                              // zbaczanie
      T.h = mgWrap360(T.h + dh);
      O.h = mgWrap360(O.h + dh * 0.85f + n(0.1f));       // żyroskop widzi większość
      float tdist = step * (1 + n(0.01f));
      T.x += cosf(T.h * MG_DEG2RAD) * tdist; T.y += sinf(T.h * MG_DEG2RAD) * tdist;
      float odist = tdist * (1 + n(0.01f));
      O.x += cosf(O.h * MG_DEG2RAD) * odist; O.y += sinf(O.h * MG_DEG2RAD) * odist;
      bool cNow = collides(T);
      if (cNow && !inColl) collisions++;   // liczymy zdarzenia (wejście w przeszkodę), nie kroki
      inColl = cNow;
      done += step; traveled += tdist;
    }
    return !blocked;
  }
};
// adapter, żeby Explorer widział map() jako metodę
struct Adapter {
  SimRobot& r;
  Pose2 pose() { return r.pose(); }
  const OccGrid& map() { return r.map; }
  bool scan() { return r.scan(); }
  bool locate() { return r.locate(); }
  bool turn(float d) { return r.turn(d); }
  bool drive(float c) { return r.drive(c); }
  bool aborted() { return false; }
  void note(const char* t) { if (verbose) printf("      %s\n", t); }
  bool verbose = false;
};

int main() {
  int fails = 0;
  // Pozycje startowe w mapie świata: środek, róg przy ścianie (20-30 cm od niej, robot ma 26 cm długości), drugi róg, kierunek wzdłuż ściany
  const float starts[3][3] = {{0, 0, 90}, {-100, -132, 180}, {150, 222, 180}};
  for (unsigned seed = 1; seed <= 9; seed++) {
    const float* sp = starts[(seed - 1) % 3];
    world.clear();
    addRect(-120, -150, 180, 250);                 // pokój 300 x 400
    addRect(-120, 58, 70, 62);                     // przegroda z przejściem 110 cm
    addRect(110, 150, 160, 190);                   // skrzynia
    addRect(-60, 160, -30, 190);                   // mała szafka
    addRect(37, -103, 43, -97);                    // noga stołu 6 x 6
    addRect(-80, 110, -74, 116);                   // druga noga
    addRect(120, -40, 126, -34);                   // trzecia noga
    SimRobot sim(seed, sp[0], sp[1], sp[2]);
    if (getenv("GS")) sim.gyroScale = atof(getenv("GS"));
    Adapter ad{sim};
    Explorer ex;
    int st;
    while ((st = ex.step(ad)) == EX_RUNNING) {}
    // powrót na start (0,0 w układzie odometrii = miejsce startu)
    int exploreColl = sim.collisions;
    Explorer back;
    back.beginHome(sp[0], sp[1], sp[2]);
    int hst;
    while ((hst = back.step(ad)) == EX_RUNNING) {}
    float homeErr = hypotf(sim.T.x - sp[0], sim.T.y - sp[1]);

    // pokrycie: znane wolne / prawdziwe wolne (osiągalne, >= 12 cm od ścian)
    int tot = 0, known = 0;
    for (float y = -150; y < 250; y += 5) for (float x = -120; x < 180; x += 5) {
      if (!freeSpace(x, y)) continue;
      tot++;
      int8_t v = sim.map.at(OccGrid::toCell(x), OccGrid::toCell(y));
      if (v <= L_FREE_T) known++;
    }
    // dokładność: zajęte komórki blisko prawdziwej ściany
    int occ = 0, good = 0;
    for (int cy = 0; cy < MAP_CELLS; cy++) for (int cx = 0; cx < MAP_CELLS; cx++) if (sim.map.at(cx, cy) >= L_OCC_T) {
      occ++;
      float wx = (cx - MAP_CELLS / 2 + 0.5f) * MAP_CELL_CM, wy = (cy - MAP_CELLS / 2 + 0.5f) * MAP_CELL_CM;
      if (distToWorld(wx, wy) <= 10.0f) good++;
    }
    float cov = 100.0f * known / tot, acc = 100.0f * good / (occ ? occ : 1);
    float oerr = hypotf(sim.O.x - sim.T.x, sim.O.y - sim.T.y);
    bool ok = (st == EX_DONE) && cov >= 85.0f && acc >= 80.0f && exploreColl <= 5 && sim.collisions - exploreColl <= 8 && hst == EX_DONE && homeErr <= 25.0f;
    printf("%s seed %u start(%.0f,%.0f): status=%d kroki=%d skany=%d droga=%.1f m pokrycie=%.0f%% dokładność=%.0f%% kolizje=%d(+%d w powrocie) błąd odometrii na końcu=%.1f cm powrót: status=%d odległość od startu=%.0f cm\n",
           ok ? "ok:  " : "FAIL:", seed, sp[0], sp[1], st, ex.steps, sim.scans, sim.traveled / 100, cov, acc, exploreColl, sim.collisions - exploreColl, oerr, hst, homeErr);
    if (!ok) fails++;
  }
  printf(fails ? "\nBŁĘDÓW: %d\n" : "\nWSZYSTKO OK\n", fails);
  return fails ? 1 : 0;
}
