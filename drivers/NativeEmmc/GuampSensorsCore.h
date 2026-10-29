// Sensor hub core (analisi/sensori/SENSORI-20261029.md): pure logic shared by the kernel glue (GuampSensors.inc) and the
// host test (tests/sensors_host.cpp). Kinds match the NativeHidInput sensor devices (scripts/guamp-hid-variant.py step 4).
// Units: accelerometer milli-g, gyro mrad/s, orientation 1..8 (AppleVendorMotion 0xFF0C usage 0x64 + value - 1),
// proximity 0 far / 1 near, ambient light lux.
#pragma once
#include <stdint.h>

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

}  // namespace guamp_sensors
