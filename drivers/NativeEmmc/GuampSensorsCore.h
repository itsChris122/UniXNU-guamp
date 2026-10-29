// Sensor hub core (analisi/sensori/SENSORI-20261029.md): pure logic shared by the kernel glue (GuampSensors.inc) and the
// host test (tests/sensors_host.cpp). Kinds match the NativeHidInput sensor devices (scripts/guamp-hid-variant.py step 4).
// Units: accelerometer milli-g, gyro mrad/s, orientation 1..8 (AppleVendorMotion 0xFF0C usage 0x64 + value - 1),
// proximity 0 far / 1 near, ambient light lux.
#pragma once
#include <stdint.h>
#ifndef INT32_MIN
#define INT32_MIN (-2147483647 - 1)
#define INT32_MAX 2147483647
#endif

namespace guamp_sensors {

enum : unsigned { kAccel = 1, kGyro = 2, kOrient = 3, kProx = 4, kAls = 5, kKinds = 6, kPose = 0x10 };

// Orientation values of the device-orientation report (IOHIDFamily IOHIDDeviceOrientationCoreMotionDescriptor.h order)
enum : int32_t { kOrientAmbiguous = 1, kOrientPortrait = 2, kOrientUpsideDown = 3, kOrientLandscapeLeft = 4,
                 kOrientLandscapeRight = 5, kOrientFaceUp = 6, kOrientFaceDown = 7 };

// Poses of the synthetic source: 0 portrait, 1 landscape left (home/bottom edge on the right), 2 upside down,
// 3 landscape right, 4 face up, 5 face down.
enum : unsigned { kPoses = 6 };

struct Sample {
  unsigned kind = 0;
  int32_t v[3] = {0, 0, 0};
};

static inline int32_t le32(const volatile uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24);
}

// USB mirror bulk OUT packet: "GSNS", kind u8, x/y/z int32 little endian (17 bytes). Kind kPose carries a pose in x.
static inline bool decodeMirror(const volatile uint8_t *b, uint32_t n, Sample *out) {
  if (!b || !out || n < 17 || b[0] != 'G' || b[1] != 'S' || b[2] != 'N' || b[3] != 'S') return false;
  const unsigned kind = b[4];
  if (!((kind >= kAccel && kind < kKinds) || kind == kPose)) return false;
  out->kind = kind;
  for (unsigned i = 0; i < 3; ++i) out->v[i] = le32(b + 5 + 4 * i);
  if (kind == kPose && (out->v[0] < 0 || out->v[0] >= (int32_t)kPoses)) return false;
  return true;
}

// Gravity as iOS reports it at rest (x right, y up the portrait screen, z out of the screen; 1 g = 1000) and the
// orientation value of a pose. Portrait upright reads y = -1 g, face up z = -1 g, landscape left x = -1 g.
static inline bool pose(unsigned p, int32_t g[3], int32_t *orientation) {
  static const int32_t kG[kPoses][3] = {{0, -1000, 0}, {-1000, 0, 0}, {0, 1000, 0}, {1000, 0, 0}, {0, 0, -1000}, {0, 0, 1000}};
  static const int32_t kO[kPoses] = {kOrientPortrait, kOrientLandscapeLeft, kOrientUpsideDown, kOrientLandscapeRight,
                                     kOrientFaceUp, kOrientFaceDown};
  if (p >= kPoses || !g || !orientation) return false;
  for (unsigned i = 0; i < 3; ++i) g[i] = kG[p][i];
  *orientation = kO[p];
  return true;
}

// Orientation from a gravity vector (milli-g): the axis that carries most of it, ambiguous below 0.5 g or when two
// axes are within 0.2 g of each other. The same rule serves the synthetic source and, later, the ADSP accelerometer.
static inline int32_t orientationOf(const int32_t g[3]) {
  int32_t a[3];
  for (unsigned i = 0; i < 3; ++i) a[i] = g[i] < 0 ? -g[i] : g[i];
  unsigned m = 0;
  for (unsigned i = 1; i < 3; ++i) if (a[i] > a[m]) m = i;
  if (a[m] < 500) return kOrientAmbiguous;
  for (unsigned i = 0; i < 3; ++i) if (i != m && a[m] - a[i] < 200) return kOrientAmbiguous;
  if (m == 0) return g[0] < 0 ? kOrientLandscapeLeft : kOrientLandscapeRight;
  if (m == 1) return g[1] < 0 ? kOrientPortrait : kOrientUpsideDown;
  return g[2] < 0 ? kOrientFaceUp : kOrientFaceDown;
}

// ---- real samples from the ADSP SEE (GuampSnsCore.h, guamp-sensors=2) ----
// SEE reports IEEE-754 floats in the Android device frame (sns_accel.proto: x right, y up the portrait screen, z out of
// the screen; the driver already applies the registry axis map lsm6dso_0_platform.orient), accel in m/s2 with +g up at
// rest, gyro in rad/s (right hand), light in lux. The kernel has no floating point: the bits are scaled with integers.
// round(f * mul / div), saturated to int32; NaN and infinities give 0. |mul| <= 2^27.
static inline int32_t f32Scaled(uint32_t bits, int64_t mul, int64_t div) {
  const uint32_t e = bits >> 23 & 0xff, frac = bits & 0x7fffff;
  if (e == 0xff || !div) return 0;
  const bool neg = (bits >> 31) != 0;
  const uint64_t m = e ? (frac | 0x800000u) : frac;   // value = m * 2^(e - 150) (subnormal: e = 1)
  int shift = (int)(e ? e : 1) - 150;
  const bool negMul = (mul < 0) != (div < 0);
  const uint64_t um = (uint64_t)(mul < 0 ? -mul : mul), ud = (uint64_t)(div < 0 ? -div : div);
  if (um > (1u << 27)) return 0;
  uint64_t q = (m * um << 8) / ud;   // m * mul * 2^8 / div: m < 2^24, mul <= 2^27
  shift -= 8;
  uint64_t r;
  if (shift >= 0) {
    if (shift > 30 || q > (0x7fffffffull >> shift)) r = 0x80000000ull;
    else r = q << shift;
  } else if (shift <= -63) {
    r = 0;
  } else {
    r = (q + (1ull << (-shift - 1))) >> -shift;
  }
  const bool minus = neg != negMul;
  if (r > 0x7fffffffull) return minus ? INT32_MIN : INT32_MAX;
  return minus ? -(int32_t)r : (int32_t)r;
}
// guamp-sensors-flip bits, applied to accel and gyro after the unit conversion: 1 negate x, 2 negate y, 4 negate z,
// 8 swap x and y (first). Default 0: the SEE frame already is the screen frame.
static inline void flipAxes(int32_t v[3], unsigned flip) {
  if (flip & 8) { const int32_t t = v[0]; v[0] = v[1]; v[1] = t; }
  for (unsigned i = 0; i < 3; ++i) if (flip >> i & 1) v[i] = -v[i];
}
// One SEE sample (float bits, n values) in the units of the HID devices. Accel: milli-g with the iOS sign (at rest
// portrait upright reads y = -1000: Android reports the reaction to gravity, iOS gravity itself); gyro: mrad/s;
// light: lux (data[0] = lux, data[1] = raw ADC). False when the sample is too short or the kind has no float form.
static inline bool fromSee(unsigned kind, const uint32_t *bits, unsigned n, int32_t out[3]) {
  if (!bits || !out) return false;
  out[0] = out[1] = out[2] = 0;
  if (kind == kAccel || kind == kGyro) {
    if (n < 3) return false;
    for (unsigned i = 0; i < 3; ++i)
      out[i] = kind == kAccel ? f32Scaled(bits[i], -101971621, 1000000) : f32Scaled(bits[i], 1000, 1);
    return true;
  }
  if (kind == kAls) {
    if (n < 1) return false;
    const int32_t lux = f32Scaled(bits[0], 1, 1);
    out[0] = lux < 0 ? 0 : lux;
    out[1] = n > 1 ? f32Scaled(bits[1], 1, 1) : 0;
    return true;
  }
  return false;
}
// Device orientation from a stream of accelerometer samples: a new value only after kStable consecutive samples agree
// and differ from the last one sent; ambiguous samples (tilted, shaken, free fall) restart the count and are never sent.
struct OrientFilter {
  static constexpr unsigned kStable = 4;
  int32_t sent = 0, candidate = 0;
  unsigned count = 0;
  // true with *orientation set when a new orientation is due
  bool update(const int32_t g[3], int32_t *orientation) {
    const int32_t o = orientationOf(g);
    if (o == kOrientAmbiguous) { count = 0; candidate = 0; return false; }
    if (o != candidate) { candidate = o; count = 1; } else if (count < kStable) ++count;
    if (count < kStable || o == sent) return false;
    sent = o;
    *orientation = o;
    return true;
  }
};

}  // namespace guamp_sensors
