// Sensor hub (analisi/sensori/SENSORI-20261029.md, step 3): one sink per HID sensor device of NativeHidInput (step 4 of
// scripts/guamp-hid-variant.py: 1 accelerometer, 2 gyro, 3 device orientation, 4 proximity). Producers call
// guamp_sensor_push from any thread; the sink runs the report on the device's work loop (synchronous runAction).
// guamp-sensors: 0 off (default, no HID sensor devices either), 1 synthetic source = the USB mirror ("GSNS" packets from
// strumenti/specchio-ios.py), 2 reserved for the ADSP SEE client. Ambient light (kind 5) is kept for the log and for
// guamp_sensor_last only: IOHIDEventDriver has no ambient light element, the HID path for it is still open.
// Included before UsbMirror.inc (the mirror hands its OUT packets to sensors::fromMirror first).
#include "GuampSensorsCore.h"

namespace sensors {
using namespace guamp_sensors;

struct Sink {
  void (*volatile fn)(void *, const int32_t *);
  void *volatile ctx;
};
static Sink gSink[kKinds];
static unsigned gMode = 0;
}  // namespace sensors

extern "C" {
volatile int32_t guamp_sensor_last[guamp_sensors::kKinds][3];
volatile uint32_t guamp_sensor_pushes[guamp_sensors::kKinds], guamp_sensor_delivered[guamp_sensors::kKinds];

// Called by NativeHidInput sensor devices in start (fn set) and stop (fn null). Weak import on their side.
void native_sensor_set_sink(unsigned kind, void (*fn)(void *, const int32_t *), void *ctx) {
  using namespace sensors;
  if (kind < kAccel || kind >= kKinds) return;
  gSink[kind].fn = nullptr;
  __asm__ volatile("dmb ish" ::: "memory");
  gSink[kind].ctx = ctx;
  __asm__ volatile("dmb ish" ::: "memory");
  gSink[kind].fn = fn;
  IOLog("GuampSensors: sink %u %s\n", kind, fn ? "attached" : "detached");
}

// One sample. Values are copied before the sink runs, so the caller's array may be temporary.
void guamp_sensor_push(unsigned kind, int32_t x, int32_t y, int32_t z) {
  using namespace sensors;
  if (kind < kAccel || kind >= kKinds) return;
  const int32_t v[3] = {x, y, z};
  for (unsigned i = 0; i < 3; ++i) guamp_sensor_last[kind][i] = v[i];
  guamp_sensor_pushes[kind] = guamp_sensor_pushes[kind] + 1;
  void (*fn)(void *, const int32_t *) = gSink[kind].fn;
  __asm__ volatile("dmb ish" ::: "memory");
  void *ctx = gSink[kind].ctx;
  if (!fn) return;
  fn(ctx, v);
  guamp_sensor_delivered[kind] = guamp_sensor_delivered[kind] + 1;
}
}  // extern "C"

namespace sensors {

// USB mirror bulk OUT: true when the packet was a sensor packet (consumed even with guamp-sensors=0, then dropped).
static bool fromMirror(const volatile uint8_t *b, uint32_t n) {
  Sample s;
  if (!decodeMirror(b, n, &s)) return false;
  if (gMode != 1) return true;
  if (s.kind == kPose) {
    int32_t g[3], o;
    if (!pose((unsigned)s.v[0], g, &o)) return true;
    guamp_sensor_push(kAccel, g[0], g[1], g[2]);
    guamp_sensor_push(kOrient, o, 0, 0);
    IOLog("GuampSensors: mirror pose %d: accel %d %d %d mg, orientation %d (sinks accel %s, orientation %s)\n", s.v[0],
          g[0], g[1], g[2], o, gSink[kAccel].fn ? "on" : "off", gSink[kOrient].fn ? "on" : "off");
    return true;
  }
  guamp_sensor_push(s.kind, s.v[0], s.v[1], s.v[2]);
  // Accelerometer samples also drive the orientation device, as the AOP gesture service would.
  if (s.kind == kAccel) guamp_sensor_push(kOrient, orientationOf(s.v), 0, 0);
  IOLog("GuampSensors: mirror kind %u: %d %d %d (sink %s)\n", s.kind, s.v[0], s.v[1], s.v[2],
        gSink[s.kind].fn ? "on" : "off");
  return true;
}

void start() {
  PE_parse_boot_argn("guamp-sensors", &gMode, sizeof(gMode));
  IOLog("GuampSensors: mode %u (%s)\n", gMode,
        gMode == 1 ? "synthetic from the USB mirror" : gMode == 2 ? "ADSP SEE (not yet)" : "off");
}

}  // namespace sensors
