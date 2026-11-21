// Test logiki mapy na komputerze: g++ -std=c++17 -O2 -I.. host_test.cpp -o host_test && ./host_test
#include <cstdio>
#include <vector>
#include <random>
#include <cmath>
#include "../MapGrid.hpp"

struct Seg { float x1, y1, x2, y2; };
static std::vector<Seg> world;

static void addRect(float x0, float y0, float x1, float y1) {
  world.push_back({x0, y0, x1, y0}); world.push_back({x1, y0, x1, y1});
  world.push_back({x1, y1, x0, y1}); world.push_back({x0, y1, x0, y0});
}
// odległość do najbliższego odcinka
static float distToWorld(float x, float y) {
  float best = 1e9;
  for (auto& s : world) {
    float vx = s.x2 - s.x1, vy = s.y2 - s.y1;
    float t = ((x - s.x1) * vx + (y - s.y1) * vy) / (vx * vx + vy * vy);
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    float px = s.x1 + t * vx, py = s.y1 + t * vy;
    best = fminf(best, hypotf(x - px, y - py));
  }
  return best;
}
static float castRay(float ox, float oy, float angDeg) {
  float a = angDeg * MG_DEG2RAD, dx = cosf(a), dy = sinf(a), best = 1e9;
  for (auto& s : world) {
    float sx = s.x2 - s.x1, sy = s.y2 - s.y1;
    float den = dx * sy - dy * sx;
    if (fabsf(den) < 1e-9) continue;
    float t = ((s.x1 - ox) * sy - (s.y1 - oy) * sx) / den;
    float u = ((s.x1 - ox) * dy - (s.y1 - oy) * dx) / den;
    if (t > 0 && u >= 0 && u <= 1 && t < best) best = t;
  }
  return best;
}
static std::mt19937 rng(7);
static float noise(float sd) { return std::normal_distribution<float>(0, sd)(rng); }

// Symulacja skanu 360 z serwa: kąty 0..180, czujnik 0 -> rel = a, czujnik 1 -> rel = 180 + a
static std::vector<Beam> simulateScan(const Pose2& truth) {
  std::vector<Beam> b;
  float sx, sy; OccGrid::sensorOrigin(truth, SERVO_SENSOR_OFFSET_FWD_CM, SERVO_SENSOR_OFFSET_LEFT_CM, sx, sy);
  for (int a = 0; a <= 180; a += SCAN_STEP_DEG) {
    for (int s = 0; s < 2; s++) {
      float rel = SERVO_DIRECTION * a + (s ? 180 : 0);
      float r = castRay(sx, sy, truth.h + rel);
      Beam bm; bm.relDeg = rel; bm.sensor = s;
      if (r > MAP_MAX_RANGE_CM) { bm.kind = 0; bm.rangeCm = 1000; }
      else { bm.kind = 1; bm.rangeCm = r + noise(1.2f); }
      b.push_back(bm);
    }
  }
  return b;
}

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } else { printf("ok:   "); printf(__VA_ARGS__); printf("\n"); } } while (0)

int main() {
  // Pokój 400 x 300 cm (x -200..200, y -150..150) + skrzynia 40 x 40 w (80,40)
  addRect(-200, -150, 200, 150);
  addRect(60, 20, 100, 60);

  OccGrid g; g.begin(); g.clear();
  CHECK(g.cells != NULL, "alokacja siatki %d x %d = %d B", MAP_CELLS, MAP_CELLS, MAP_CELLS * MAP_CELLS);

  // --- 1. Skala: ściana 100 cm przed robotem musi wylądować dokładnie 100 cm dalej ---
  {
    OccGrid t; t.begin(); t.clear();
    world.clear(); addRect(-300, -300, 300, 100);       // ściana przodu na y = 100
    Pose2 p = {0, 0, 90};
    auto b = simulateScan(p);
    t.integrateScan(p, b.data(), b.size());
    // przód to kierunek +Y: szukamy komórki zajętej w kolumnie x = 0
    int cx = OccGrid::toCell(0);
    int found = -1;
    for (int cy = MAP_CELLS - 1; cy >= MAP_CELLS / 2; cy--) if (t.at(cx, cy) >= L_OCC_T) { found = cy; break; }
    float yEdge = (found - MAP_CELLS / 2) * (float)MAP_CELL_CM;
    CHECK(found >= 0 && fabsf(yEdge + MAP_CELL_CM / 2.0f - 100.0f) <= MAP_CELL_CM,
          "skala: ściana w 100 cm wykryta na ok. %.1f cm", yEdge + MAP_CELL_CM / 2.0f);
    world.clear(); addRect(-200, -150, 200, 150); addRect(60, 20, 100, 60);
  }

  // --- 2. Mapa z kilku pozycji ---
  Pose2 stops[] = {{-120, -80, 90}, {-120, 80, 0}, {0, 0, 45}, {120, -80, 200}, {-60, 80, 135}, {130, 90, 300}};
  for (auto& p : stops) {
    auto b = simulateScan(p);
    g.integrateScan(p, b.data(), b.size());
  }
  uint32_t occ, fr; g.stats(occ, fr);
  int near = 0, tot = 0;
  for (int cy = 0; cy < MAP_CELLS; cy++) for (int cx = 0; cx < MAP_CELLS; cx++) {
    if (g.at(cx, cy) >= L_OCC_T) {
      tot++;
      float wx = (cx - MAP_CELLS / 2 + 0.5f) * MAP_CELL_CM, wy = (cy - MAP_CELLS / 2 + 0.5f) * MAP_CELL_CM;
      if (distToWorld(wx, wy) <= 1.6f * MAP_CELL_CM) near++;
    }
  }
  CHECK(tot > 150 && near >= 0.97 * tot, "dokładność: %d/%d zajętych komórek <= 8 cm od prawdziwej ściany", near, tot);
  CHECK(occ > 0 && fr > 3000, "komórki: %u zajętych, %u wolnych (%.1f m2 wolnej)", occ, fr, fr * 0.0025);

  // wnętrze skrzyni i świat poza ścianą muszą być nieznane/nie wolne
  CHECK(g.at(OccGrid::toCell(80), OccGrid::toCell(40)) > L_FREE_T, "wnętrze skrzyni nie jest oznaczone jako wolne");
  CHECK(g.at(OccGrid::toCell(-260), OccGrid::toCell(0)) == 0, "poza ścianą pokoju - nieznane");

  // --- 3. Dopasowanie: odometria z błędem, skan z prawdziwej pozycji ---
  {
    Pose2 truth = {-135, 62, 60};
    auto b = simulateScan(truth);
    Pose2 odo = {truth.x + 7, truth.y - 5, truth.h + 4};
    Pose2 out; float base, best;
    g.refinePose(odo, b.data(), b.size(), SCAN_MATCH_MAX_SHIFT_CM, SCAN_MATCH_MAX_TURN_DEG, out, base, best);
    float e0 = hypotf(odo.x - truth.x, odo.y - truth.y), e1 = hypotf(out.x - truth.x, out.y - truth.y);
    CHECK(e1 < e0 && e1 <= 5.0f && fabsf(out.h - truth.h) <= 2.0f,
          "dopasowanie: błąd %.1f cm/%.1f° -> %.1f cm/%.1f° (wynik %.0f -> %.0f)", e0, odo.h - truth.h, e1, out.h - truth.h, base, best);
    // pozycja już prawidłowa nie może "uciec"
    Pose2 o2; float b2, s2;
    g.refinePose(truth, b.data(), b.size(), SCAN_MATCH_MAX_SHIFT_CM, SCAN_MATCH_MAX_TURN_DEG, o2, b2, s2);
    CHECK(hypotf(o2.x - truth.x, o2.y - truth.y) <= 5.0f && fabsf(o2.h - truth.h) <= 1.6f,
          "dopasowanie z poprawnej pozycji zostaje blisko (%.1f cm, %.1f°)", hypotf(o2.x - truth.x, o2.y - truth.y), o2.h - truth.h);
    // szerokie okno (po restarcie): 30 cm / 15 stopni
    Pose2 odo3 = {truth.x - 22, truth.y + 18, truth.h - 11};
    Pose2 o3; float b3, s3;
    g.refinePose(odo3, b.data(), b.size(), 30, 15, o3, b3, s3);
    CHECK(hypotf(o3.x - truth.x, o3.y - truth.y) <= 4.0f && fabsf(o3.h - truth.h) <= 2.0f,
          "lokalizacja po restarcie (okno 30 cm/15°): błąd %.1f cm, %.1f°", hypotf(o3.x - truth.x, o3.y - truth.y), o3.h - truth.h);
  }

  // --- 4. RLE ---
  {
    std::vector<uint8_t> enc;
    size_t n = g.encodeRle([&](const uint8_t* p, size_t l) { enc.insert(enc.end(), p, p + l); });
    CHECK(n == enc.size(), "RLE: %zu B zamiast %d B", enc.size(), MAP_CELLS * MAP_CELLS);
    std::vector<uint8_t> dec;
    for (size_t i = 0; i + 1 < enc.size(); i += 2) dec.insert(dec.end(), enc[i], enc[i + 1]);
    bool same = dec.size() == (size_t)MAP_CELLS * MAP_CELLS;
    for (size_t i = 0; same && i < dec.size(); i++) same = dec[i] == OccGrid::classOf(g.cells[i]);
    CHECK(same, "RLE: dekodowanie zgodne z mapą");
  }

  // --- 5. Ruchome zakłócenie zanika (przeszkoda znika) ---
  {
    OccGrid t; t.begin(); t.clear();
    Pose2 p = {2.5f, 0, 90};
    Beam hit = {0, 60, 1, 0}, far = {0, 100, 1, 0};
    t.integrateScan(p, &hit, 1);
    int cy = OccGrid::toCell(0 + 0) ; (void)cy;
    float ox_, oy_; OccGrid::sensorOrigin(p, SERVO_SENSOR_OFFSET_FWD_CM, SERVO_SENSOR_OFFSET_LEFT_CM, ox_, oy_);
    int hx = OccGrid::toCell(ox_), hy = OccGrid::toCell(oy_ + 60);   // 60 cm w osi 0 stopni = oś X... sprawdzamy w kierunku h+rel=90 => +Y
    CHECK(t.at(hx, hy) >= L_OCC_T, "pojedyncze trafienie tworzy przeszkodę");
    for (int i = 0; i < 6; i++) t.integrateScan(p, &far, 1);
    CHECK(t.at(hx, hy) < L_OCC_T, "po kilku skanach z wolną drogą przeszkoda znika");
  }

  // --- 6. Orientacja ścian (korekta kierunku) także dla pokoju obróconego względem osi ---
  {
    world.clear();
    const float rot = 27.0f * MG_DEG2RAD, c = cosf(rot), sn = sinf(rot);
    float P[4][2] = {{-110, -90}, {110, -90}, {110, 90}, {-110, 90}};
    for (int i = 0; i < 4; i++) {
      int j = (i + 1) % 4;
      world.push_back({P[i][0] * c - P[i][1] * sn, P[i][0] * sn + P[i][1] * c, P[j][0] * c - P[j][1] * sn, P[j][0] * sn + P[j][1] * c});
    }
    // Prawdziwy kierunek robota 100, my uważamy, że 107 (błąd +7 st.); ściany mają orientację 27 st. = 27 mod 90
    int good = 0, total = 0;
    for (int t = 0; t < 20; t++) {
      Pose2 truth = {noise(15), noise(15), 100};
      std::vector<Beam> b = simulateScan(truth);
      Pose2 believed = truth; believed.h = 107;
      float th, conf;
      if (!OccGrid::wallOrientation(believed, b.data(), (int)b.size(), th, conf)) continue;
      total++;
      // oczekiwane: ściany względem NASZEGO kierunku wyglądają na obrócone o (27 + 7) mod 90 = 34
      if (fabsf(mgWrapPm45(th - 34.0f)) <= 1.0f && conf > 0.6f) good++;
    }
    CHECK(total >= 18 && good >= 0.9 * total, "orientacja ścian: %d/%d skanów w granicach 1 st. (pokój obrócony o 27 st.)", good, total);
  }


  // --- 5. Globalna lokalizacja: robot "porwany" w losowe miejsce znanej mapy ---
  for (int mode = 0; mode < 2; mode++) {
    world.clear(); addRect(-170, -125, 170, 125); addRect(50, 10, 90, 50); addRect(-130, 70, -90, 100); addRect(-20, -125, 20, -85); addRect(100, -125, 170, -70); addRect(-170, -30, -140, 20);
    OccGrid m; m.begin(); m.clear();
    Pose2 st2[] = {{-100, -60, 90}, {-100, 40, 0}, {0, 0, 45}, {120, -40, 200}, {-60, 70, 135}, {130, 90, 300}, {0, -60, 10}, {40, 80, 270}, {-110, 0, 180}, {60, -80, 90}};
    for (auto& p : st2) { auto b = simulateScan(p); m.integrateScan(p, b.data(), b.size()); }
    m.refValid = (mode == 1);
    int good = 0, wrong = 0, none = 0, tot = 0;
    for (int t = 0; t < 24; t++) {
      Pose2 truth;
      do { truth = {(float)(noise(80)), (float)(noise(60)), (float)(rng() % 360)}; }
      while (distToWorld(truth.x, truth.y) < 25.0f || fabsf(truth.x) > 145 || fabsf(truth.y) > 100 || (truth.x > 25 && truth.x < 115 && truth.y > -15 && truth.y < 75));
      auto b = simulateScan(truth);
      Pose2 prior = {truth.x + 90, truth.y - 50, (float)fmodf(truth.h + 130.0f, 360.0f)};
      OccGrid::GlobalInfo gi;
      tot++;
      if (m.globalLocalize(prior, b.data(), (int)b.size(), gi)) {
        float dh = fabsf(fmodf(fabsf(gi.pose.h - truth.h), 360.0f)); if (dh > 180) dh = 360 - dh;
        if (hypotf(gi.pose.x - truth.x, gi.pose.y - truth.y) <= 10.0f && dh <= 6.0f) good++; else { wrong++; printf("   zla: truth (%.0f,%.0f,%.0f) -> (%.0f,%.0f,%.0f) %.0f/%.0f\n", truth.x, truth.y, truth.h, gi.pose.x, gi.pose.y, gi.pose.h, gi.score, gi.second); }
      } else { none++; printf("   odrz: truth (%.0f,%.0f,%.0f) -> (%.0f,%.0f,%.0f) %.0f/%.0f hits %d: %.40s\n", truth.x, truth.y, truth.h, gi.pose.x, gi.pose.y, gi.pose.h, gi.score, gi.second, gi.hits, gi.why); }
    }
    CHECK(good >= 0.75 * tot && wrong <= 1, "globalna lokalizacja (%s): trafnie %d/%d, bledne %d, odrzucone %d", mode ? "kierunki ze scian" : "wszystkie kierunki", good, tot, wrong, none);
  }


  // --- 6. Lokalizacja z podpowiedzią (użytkownik wskazał miejsce na mapie z błędem do 20 cm) ---
  {
    OccGrid m; m.begin(); m.clear();
    Pose2 st2[] = {{-100, -60, 90}, {-100, 40, 0}, {0, 0, 45}, {120, -40, 200}, {-60, 70, 135}, {130, 90, 300}, {0, -60, 10}, {40, 80, 270}, {-110, 0, 180}, {60, -80, 90}};
    for (auto& p : st2) { auto b = simulateScan(p); m.integrateScan(p, b.data(), b.size()); }
    int good = 0, tot = 0, none = 0;
    for (int t = 0; t < 24; t++) {
      Pose2 truth;
      do { truth = {(float)(noise(80)), (float)(noise(60)), (float)(rng() % 360)}; }
      while (distToWorld(truth.x, truth.y) < 25.0f || fabsf(truth.x) > 145 || fabsf(truth.y) > 100 || (truth.x > 25 && truth.x < 115 && truth.y > -15 && truth.y < 75));
      auto b = simulateScan(truth);
      Pose2 tap = {truth.x + 14, truth.y - 12, 0};
      OccGrid::GlobalInfo gi; tot++;
      if (m.globalLocalize(tap, b.data(), (int)b.size(), gi, 30.0f)) {
        float dh = fabsf(fmodf(fabsf(gi.pose.h - truth.h), 360.0f)); if (dh > 180) dh = 360 - dh;
        if (hypotf(gi.pose.x - truth.x, gi.pose.y - truth.y) <= 10.0f && dh <= 6.0f) good++;
      } else none++;
    }
    CHECK(good >= 0.7 * tot, "lokalizacja z podpowiedzia: trafnie %d/%d, odrzucone %d", good, tot, none);
  }

  printf(fails ? "\nBŁĘDÓW: %d\n" : "\nWSZYSTKO OK\n", fails);
  return fails ? 1 : 0;
}
