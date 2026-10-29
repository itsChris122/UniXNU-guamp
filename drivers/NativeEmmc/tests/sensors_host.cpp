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

  // SEE floats without floating point in the kernel: compare with the host FPU
  auto bits = [](float f) { uint32_t b; memcpy(&b, &f, 4); return b; };
  const float samples[] = {0.0f, -0.0f, 1.0f, -1.0f, 9.80665f, -9.80665f, 0.0123f, 3.5e-7f, 78.4532f, -156.9f, 34.9f,
                           1e-40f, 12345.678f, 65535.0f, 1e9f, -1e12f, 0.5f, 2.5f, -2.5f};
  for (float f : samples) {
    const double e1 = (double)f * 101971621.0 / 1000000.0, e2 = (double)f * 1000.0, e3 = (double)f;
    auto near = [](int32_t got, double want) {
      if (want > 2147483647.0) return got == INT32_MAX;
      if (want < -2147483648.0) return got == INT32_MIN;
      return got - want <= 0.5001 && want - got <= 0.5001;
    };
    CHECK(near(f32Scaled(bits(f), 101971621, 1000000), e1));
    CHECK(near(f32Scaled(bits(f), 1000, 1), e2));
    CHECK(near(f32Scaled(bits(f), 1, 1), e3));
    CHECK(near(f32Scaled(bits(f), -101971621, 1000000), -e1));
  }
  CHECK(f32Scaled(0x7fc00000u, 1000, 1) == 0 && f32Scaled(0x7f800000u, 1000, 1) == 0);   // NaN, infinity
  // Android frame -> iOS units: portrait upright at rest (+g on y) reads y = -1000 mg
  int32_t out[3];
  const uint32_t rest[3] = {bits(0.0f), bits(9.80665f), bits(0.0f)};
  CHECK(fromSee(kAccel, rest, 3, out) && out[0] == 0 && out[1] == -1000 && out[2] == 0);
  const uint32_t flat[3] = {bits(0.1f), bits(-0.2f), bits(9.81f)};
  CHECK(fromSee(kAccel, flat, 3, out) && out[2] == -1000 && orientationOf(out) == kOrientFaceUp);
  const uint32_t spin[3] = {bits(0.5f), bits(-1.25f), bits(3.0f)};
  CHECK(fromSee(kGyro, spin, 3, out) && out[0] == 500 && out[1] == -1250 && out[2] == 3000);
  const uint32_t lux[2] = {bits(321.6f), bits(1500.0f)};
  CHECK(fromSee(kAls, lux, 2, out) && out[0] == 322 && out[1] == 1500);
  const uint32_t dark[1] = {bits(-3.0f)};
  CHECK(fromSee(kAls, dark, 1, out) && out[0] == 0);
  CHECK(!fromSee(kAccel, rest, 2, out) && !fromSee(kProx, rest, 3, out) && !fromSee(kAccel, nullptr, 3, out));
  int32_t fv[3] = {1, 2, 3};
  flipAxes(fv, 8 | 1);
  CHECK(fv[0] == -2 && fv[1] == 1 && fv[2] == 3);
  // the orientation debounce: kStable equal samples, never ambiguous ones, no repeat
  OrientFilter of;
  int32_t o2 = 0;
  const int32_t portrait[3] = {20, -990, 50}, left[3] = {-990, 30, 40}, tilted[3] = {-700, -690, 0};
  for (unsigned i = 1; i < OrientFilter::kStable; ++i) CHECK(!of.update(portrait, &o2));
  CHECK(of.update(portrait, &o2) && o2 == kOrientPortrait);
  CHECK(!of.update(portrait, &o2));
  CHECK(!of.update(left, &o2) && !of.update(left, &o2) && !of.update(tilted, &o2));   // interrupted
  for (unsigned i = 1; i < OrientFilter::kStable; ++i) CHECK(!of.update(left, &o2));
  CHECK(of.update(left, &o2) && o2 == kOrientLandscapeLeft);
  printf("sensors_host: %s\n", fails ? "FAIL" : "PASS");
  return fails ? 1 : 0;
}
