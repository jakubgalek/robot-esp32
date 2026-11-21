#pragma once
// ---------------------------------------------------------------------------
// Drugi przebieg skanowania: skany z siatki punktów po całym pokoju (uzupełniają luki w mapie).
// Logika jest niezależna od sprzętu (jak Planner.hpp) i testowana w symulacji.
// Interfejs robota R jak w Planner.hpp: pose(), map(), scan(), locate(), turn(), drive(), aborted(), note().
// ---------------------------------------------------------------------------
#include "Planner.hpp"

// Czy środek robota może stać w (x, y): komórka znana i wolna, a w promieniu "clear" nie ma przeszkody
static inline bool covFeasible(const OccGrid& m, float x, float y, float clear) {
  if (m.at(OccGrid::toCell(x), OccGrid::toCell(y)) > L_FREE_T) return false;
  for (float dy = -clear; dy <= clear + 0.01f; dy += MAP_CELL_CM / 2.0f)
    for (float dx = -clear; dx <= clear + 0.01f; dx += MAP_CELL_CM / 2.0f)
      if (dx * dx + dy * dy <= clear * clear && m.at(OccGrid::toCell(x + dx), OccGrid::toCell(y + dy)) >= L_OCC_T) return false;
  return true;
}

// Drugi przebieg: skany z siatki punktów w całym wolnym obszarze (Explorer w trybie "cel" dojeżdża i robi skan)
#define REFINE_MAX_SPOTS 40
class RefinePass {
 public:
  float px[REFINE_MAX_SPOTS], py[REFINE_MAX_SPOTS];
  int n, idx, fails, steps;
  bool active, first;
  Explorer ex;
  RefinePass() { n = idx = fails = steps = 0; active = false; first = true; }

  void begin(const OccGrid& m, const Pose2& from) {
    n = idx = fails = steps = 0; active = false; first = true;
    float cand[160][2]; int c = 0;
    for (float y = -MAP_CELLS * MAP_CELL_CM / 2; y < MAP_CELLS * MAP_CELL_CM / 2 && c < 160; y += REFINE_GRID_CM)
      for (float x = -MAP_CELLS * MAP_CELL_CM / 2; x < MAP_CELLS * MAP_CELL_CM / 2 && c < 160; x += REFINE_GRID_CM)
        if (covFeasible(m, x, y, REFINE_CLEAR_CM)) { cand[c][0] = x; cand[c][1] = y; c++; }
    // kolejność: najbliższy sąsiad od pozycji robota
    bool used[160] = {false};
    float cx = from.x, cy = from.y;
    while (n < REFINE_MAX_SPOTS) {
      int b = -1; float bd = 1e9f;
      for (int i = 0; i < c; i++) if (!used[i]) { float d = hypotf(cand[i][0] - cx, cand[i][1] - cy); if (d < bd) { bd = d; b = i; } }
      if (b < 0) break;
      used[b] = true; px[n] = cand[b][0]; py[n] = cand[b][1]; cx = px[n]; cy = py[n]; n++;
    }
  }

  template <class R> int step(R& r) {
    if (r.aborted()) return EX_ABORT;
    if (steps++ > 3000) return EX_LIMIT;
    if (idx >= n) return EX_DONE;
    if (!active) {
      ex.beginGoal(px[idx], py[idx], !first);   // pierwszy punkt: skan z miejsca startu (robot mógł być przeniesiony)
      first = false; active = true;
      char nb[80];
      snprintf(nb, sizeof(nb), "drugi przebieg: punkt %d/%d (%.0f, %.0f)", idx + 1, n, (double)px[idx], (double)py[idx]);
      r.note(nb);
    }
    int s = ex.step(r);
    if (s == EX_RUNNING) return EX_RUNNING;
    active = false; idx++;
    if (s == EX_ABORT) return EX_ABORT;
    if (s != EX_DONE) { if (++fails >= 6) return EX_STUCK; }
    return EX_RUNNING;
  }
};
