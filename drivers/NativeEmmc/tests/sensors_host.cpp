// Host test of GuampSensorsCore.h: mirror packet decoding, synthetic poses and the orientation rule.
#include "GuampSensorsCore.h"
#include <stdio.h>
#include <string.h>
static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)
using namespace guamp_sensors;

static unsigned packet(uint8_t *b, unsigned kind, int32_t x, int32_t y, int32_t z) {
  memcpy(b, "GSNS", 4);
  b[4] = (uint8_t)kind;
  const int32_t v[3] = {x, y, z};
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned k = 0; k < 4; ++k) b[5 + 4 * i + k] = (uint8_t)((uint32_t)v[i] >> (8 * k));
  return 17;
}

int main() {
  uint8_t b[64];
  Sample s;
  unsigned n = packet(b, kAccel, -1000, 25, 980);
  CHECK(decodeMirror(b, n, &s) && s.kind == kAccel && s.v[0] == -1000 && s.v[1] == 25 && s.v[2] == 980);
  CHECK(!decodeMirror(b, 16, &s));                                   // short
  CHECK(!decodeMirror(nullptr, 17, &s) && !decodeMirror(b, 17, nullptr));
  b[3] = 'X'; CHECK(!decodeMirror(b, n, &s));                        // magic
  n = packet(b, 0, 0, 0, 0); CHECK(!decodeMirror(b, n, &s));          // kind 0
  n = packet(b, kKinds, 0, 0, 0); CHECK(!decodeMirror(b, n, &s));     // kind 6
  n = packet(b, kAls, 320, 0, 0); CHECK(decodeMirror(b, n, &s) && s.kind == kAls && s.v[0] == 320);
  n = packet(b, kPose, 3, 0, 0); CHECK(decodeMirror(b, n, &s) && s.kind == kPose && s.v[0] == 3);
  n = packet(b, kPose, 6, 0, 0); CHECK(!decodeMirror(b, n, &s));
  n = packet(b, kPose, -1, 0, 0); CHECK(!decodeMirror(b, n, &s));
  memcpy(b, "GTCH", 4); CHECK(!decodeMirror(b, 17, &s));             // touch packets stay with the touch path

  // Every pose: its gravity gives back its own orientation, and the gravity is 1 g on one axis.
  for (unsigned p = 0; p < kPoses; ++p) {
    int32_t g[3], o;
    CHECK(pose(p, g, &o));
    CHECK(orientationOf(g) == o);
    CHECK((g[0] != 0) + (g[1] != 0) + (g[2] != 0) == 1);
  }
  int32_t g[3], o;
  CHECK(!pose(kPoses, g, &o));
  CHECK(pose(0, g, &o) && g[1] == -1000 && o == kOrientPortrait);
  CHECK(pose(1, g, &o) && g[0] == -1000 && o == kOrientLandscapeLeft);
  CHECK(pose(4, g, &o) && g[2] == -1000 && o == kOrientFaceUp);
  const int32_t tilt[3] = {-700, -650, 0};    // halfway between landscape left and portrait
  CHECK(orientationOf(tilt) == kOrientAmbiguous);
  const int32_t weak[3] = {0, -300, 100};     // free fall / shaking
  CHECK(orientationOf(weak) == kOrientAmbiguous);
  const int32_t lean[3] = {150, -950, 250};   // portrait, slightly tilted
  CHECK(orientationOf(lean) == kOrientPortrait);
  const int32_t ur[3] = {100, 980, -60};
  CHECK(orientationOf(ur) == kOrientUpsideDown);
  printf("sensors_host: %s\n", fails ? "FAIL" : "PASS");
  return fails ? 1 : 0;
}
