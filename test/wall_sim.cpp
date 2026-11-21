// g++ -std=c++17 -O2 -I.. wall_sim.cpp -o wall_sim && ./wall_sim
#include <algorithm>
#include <cstdio>
#include <vector>
#include <random>
#include <cmath>
#include "../WallFollow.hpp"
struct Seg { float x1, y1, x2, y2; };
static std::vector<Seg> W;
static void rect(float a, float b, float c, float d) { W.push_back({a,b,c,b}); W.push_back({c,b,c,d}); W.push_back({c,d,a,d}); W.push_back({a,d,a,b}); }
static float ray(float ox, float oy, float ang) {
  float a = ang * M_PI / 180, dx = cosf(a), dy = sinf(a), best = 1e9;
  for (auto& s : W) { float sx = s.x2 - s.x1, sy = s.y2 - s.y1, den = dx * sy - dy * sx; if (fabsf(den) < 1e-9) continue;
    float t = ((s.x1 - ox) * sy - (s.y1 - oy) * sx) / den, u = ((s.x1 - ox) * dy - (s.y1 - oy) * dx) / den;
    if (t > 0 && u >= 0 && u <= 1 && t < best) best = t; }
  return best;
}
static float distTo(float x, float y) { float best = 1e9; for (auto& s : W) { float vx = s.x2 - s.x1, vy = s.y2 - s.y1, t = ((x - s.x1) * vx + (y - s.y1) * vy) / (vx * vx + vy * vy); t = t < 0 ? 0 : t > 1 ? 1 : t; best = fminf(best, hypotf(x - s.x1 - t * vx, y - s.y1 - t * vy)); } return best; }
static std::mt19937 rng(3);
static float nz(float sd) { return std::normal_distribution<float>(0, sd)(rng); }
struct Sim {
  float x, y, h; int coll = 0; bool inColl = false; float path = 0; float gs;
  bool footprintHit() { // 4 rogi + środki boków robota 25x15
    float c = cosf(h * M_PI / 180), s = sinf(h * M_PI / 180);
    for (float a : {-12.5f, 0.f, 12.5f}) for (float b : {-7.5f, 7.5f}) if (distTo(x + c * a - s * b, y + s * a + c * b) < 1.0f) return true;
    for (float b : {-7.5f, 0.f, 7.5f}) for (float a : {-12.5f, 12.5f}) if (distTo(x + c * a - s * b, y + s * a + c * b) < 1.0f) return true;
    return false; }
  float sensor(float fwd, float left, float ang) { float c = cosf(h * M_PI / 180), s = sinf(h * M_PI / 180);
    float r = ray(x + c * fwd - s * left, y + s * fwd + c * left, h + ang); return r > 400 ? 1000 : r + nz(0.6f); }
  int side(int i) { float r = sensor(i == 2 ? 8 : -8, 7.5f, 90); return r > WALLF_SEEN_CM ? -1 : (int)r; }
  int front() { float m = fminf(sensor(12, 0, 0), sensor(11, -4, -20)); float l = sensor(11, 4, 20); if (l <= 15 && l < m) m = l; return m > 250 ? 250 : (int)m; }
  int turns = 0; float sumDist = 0; int nd = 0;
  bool aborted() { return false; }
  void note(const char* t) { printf("   [%s]\n", t); }
  bool turn(float d) { turns++; h += d * gs * (1 + nz(0.05f)); return true; }
  bool drive(float cm) {
    float c = cosf(h * M_PI / 180), s = sinf(h * M_PI / 180); float done = 0;
    while (done < cm) { if (front() <= STOP_FRONT_CENTER_CM) return false; float st = fminf(1.0f, cm - done); x += c * st; y += s * st; done += st; path += st;
      bool hit = footprintHit(); if (hit && !inColl) coll++; inColl = hit; }
    h += nz(0.4f); return true; }
};
static int run(const char* name, float x, float y, float h, int steps, float gs) {
  Sim r; r.x = x; r.y = y; r.h = h; r.gs = gs; WallFollower wf; int st = 0, it = 0;
  // pokrycie: komórki 30 cm przy obwodzie
  std::vector<std::pair<int,int>> seen;
  for (; it < steps; it++) { st = wf.step(r); if (wf.lastKind == 0 && wf.lastDist > 0) { r.sumDist += wf.lastDist; r.nd++; } if (st != WF_RUNNING) break; seen.push_back({(int)floorf(r.x / 30), (int)floorf(r.y / 30)}); }
  std::sort(seen.begin(), seen.end()); seen.erase(std::unique(seen.begin(), seen.end()), seen.end());
  printf("%-22s gs=%.2f: kroki %d, status %d, droga %.0f cm, kolizje %d, obroty %d (%.1f na metr), sr. odl. %.1f\n", name, gs, it, st, r.path, r.coll, r.turns, r.turns / (r.path / 100), r.nd ? r.sumDist / r.nd : 0.f);
  return (st == WF_RUNNING && r.coll <= 2 && r.path > 800) ? 0 : 1;
}
int main() {
  int bad = 0;
  for (float gs : {1.0f, 0.93f, 1.07f}) {
    W.clear(); rect(0, 0, 400, 300); rect(150, 100, 210, 160);
    bad += run("pokoj+skrzynia", 20, 14, 0, 500, gs);
    W.clear(); rect(0, 0, 400, 300); W.push_back({0,150,200,150}); W.push_back({200,150,200,300}); // przegroda
    bad += run("pokoj+przegroda", 30, 14, 0, 600, gs);
    W.clear(); // L-ksztalt
    W.push_back({0,0,400,0}); W.push_back({400,0,400,150}); W.push_back({400,150,200,150}); W.push_back({200,150,200,300}); W.push_back({200,300,0,300}); W.push_back({0,300,0,0});
    bad += run("pokoj L", 30, 14, 0, 600, gs);
    W.clear(); rect(0, 0, 400, 300);
    bad += run("start dalej od sciany", 100, 60, 20, 500, gs);
  }
  printf(bad ? "BLEDOW: %d\n" : "WSZYSTKO OK\n", bad); return bad ? 1 : 0;
}
