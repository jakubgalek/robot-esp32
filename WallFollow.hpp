#pragma once
// ---------------------------------------------------------------------------
// Jazda wzdłuż lewej ściany (zasada "lewej ręki"), bez skanowania i bez mapy.
// Sterownik jest niezależny od sprzętu: dostaje "robota" R z funkcjami
//   int side(int idx)  - odczyt lewego czujnika bocznego (2 = przód, 7 = tył) w cm,
//                        -1 = brak ściany w zasięgu, -2 = dane nieaktualne
//   int front()        - najmniejszy odczyt czujników przednich w cm (-2 = dane nieaktualne)
//   bool turn(float deg)  - obrót w miejscu, + = w lewo
//   bool drive(float cm)  - jazda prosto; false = zatrzymała przeszkoda / przerwanie / czas
//   bool aborted()
//   void note(const char*)
// Jedna iteracja step() = (korekta kierunku) + odcinek jazdy. Dzięki temu ten sam kod testujemy w symulacji.
// ---------------------------------------------------------------------------
#include <math.h>
#include <stdio.h>
#include "Config.hpp"

enum { WF_RUNNING = 0, WF_ABORT = 1, WF_STUCK = 2 };

struct WallFollower {
  int lost;          // ile kolejnych kroków bez ściany
  int fails;         // kolejne nieudane jazdy
  int cornerTurns;   // kolejne skręty z powodu przeszkody z przodu
  int stale;         // kolejne kroki bez świeżych danych
  int steps;
  int lastKind;      // 0 jazda przy ścianie, 1 róg wewnętrzny, 2 róg zewnętrzny / otwarcie, 3 szukanie ściany
  float lastDist;    // ostatnia odległość od ściany [cm]

  WallFollower() { reset(); }
  void reset() { lost = fails = cornerTurns = stale = steps = 0; lastKind = 0; lastDist = -1; }

  static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  template <class R> int step(R& r) {
    if (r.aborted()) return WF_ABORT;
    steps++;
    int F = r.front(), A = r.side(2), B = r.side(7);
    if (F == -2 || A == -2 || B == -2) {
      if (++stale > 40) { r.note("jazda przy scianie: brak swiezych danych z czujnikow - stop"); return WF_STUCK; }
      return WF_RUNNING;   // wołający czeka chwilę i woła ponownie
    }
    stale = 0;

    // 1) Przeszkoda z przodu: ściana po lewej + przeszkoda przed nami = róg wewnętrzny -> w prawo
    if (F <= WALLF_FRONT_TURN_CM) {
      lastKind = 1;
      if (++cornerTurns > 5) { r.note("jazda przy scianie: robot jest zablokowany ze wszystkich stron - stop"); return WF_STUCK; }
      lost = 0;
      r.turn(-90.0f);
      return r.aborted() ? WF_ABORT : WF_RUNNING;
    }
    cornerTurns = 0;

    bool a = (A >= 0 && A <= WALLF_SEEN_CM), b = (B >= 0 && B <= WALLF_SEEN_CM);

    // 2) Jest ściana po lewej: utrzymuj odległość i równoległość
    if (a || b) {
      lost = 0; lastKind = 0;
      float d, ang = 0.0f;
      if (a && b) {
        d = 0.5f * (A + B);
        ang = atan2f((float)(A - B), WALLF_SENSOR_SPACING_CM) * 57.29578f;   // + = trzeba skręcić w lewo
      } else d = a ? (float)A : (float)B;
      lastDist = d;
      float e = d - WALLF_TARGET_CM;
      if (fabsf(ang) < 3.0f) ang = 0.0f;                                   // szum pomiaru, nie korygujemy
      float lat = fabsf(e) < 1.0f ? 0.0f : clampf(e * 1.3f, -20.0f, 20.0f); // zbliżanie/odsuwanie od ściany: im dalej, tym mocniej
      if (e > 10.0f) lat = clampf(e * 1.3f, 0.0f, 28.0f);                  // odjechał daleko: wyraźnie wróć do ściany
      float want = ang + lat;
      if (d <= 5.0f && want > -12.0f) want = -12.0f;                       // za blisko: ostro od ściany
      want = clampf(want, -30.0f, 30.0f);
      if (fabsf(want) >= 4.0f) { r.turn(want); if (r.aborted()) return WF_ABORT; }
      float seg = WALLF_SEG_CM;
      if (fabsf(e) > 3.0f || fabsf(ang) > 6.0f) seg = 8.0f;               // poza torem: krótkie odcinki = szybsza korekta
      if (F < 90) { float lim = clampf((float)(F - WALLF_FRONT_TURN_CM + 4), 4.0f, seg); if (lim < seg) seg = lim; }   // nie wjeżdżaj w strefę zakrętu
      return afterDrive(r, r.drive(seg));
    }

    // 3) Ściany nie widać: róg zewnętrzny / otwarcie / start w otwartej przestrzeni
    lost++; lastKind = 2;
    if (lost == 1) {                       // przejedź tułowiem za róg
      return afterDrive(r, r.drive(WALLF_CORNER_CM));
    } else if (lost == 2) {                // ściana skręciła w lewo: skręć za nią
      r.turn(90.0f);
      if (r.aborted()) return WF_ABORT;
      return afterDrive(r, r.drive(WALLF_SEG_CM + 6.0f));
    }
    lastKind = 3;                          // dalej: jedź prosto, aż coś znajdziemy
    if (lost > 25) { r.note("jazda przy scianie: nie znajduje zadnej sciany - stop"); return WF_STUCK; }
    return afterDrive(r, r.drive(WALLF_SEG_CM * 2.0f));
  }

  template <class R> int afterDrive(R& r, bool ok) {
    if (r.aborted()) return WF_ABORT;
    if (ok) { fails = 0; return WF_RUNNING; }
    fails++;
    if (fails >= 8) { r.note("jazda przy scianie: robot wielokrotnie napotyka przeszkody - stop"); return WF_STUCK; }
    if (fails % 3 == 0) r.turn(-30.0f);    // coś blokuje bok/przód mimo wolnego środka: odsuń się od ściany
    return WF_RUNNING;
  }
};
