// Host test of the kernel side of the camera 3A: GuampCamera3ACore.h (exposure plan and CCI write lists of the four
// sensors, lens control against a simulated voice-coil driver, EEPROM AF calibration) and the photo flash policy of
// GuampTorchCore.h (current codes, safety timer, cool-down and energy budget).
#include "../GuampCamera3ACore.h"
#include "../GuampTorchCore.h"
#include <stdio.h>
#include <string.h>
#include <vector>

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

using namespace gcam3a;

// ---- simulated VCM driver (DW9767 / GT9772 register model) ------------------------------------------------------------
struct Vcm {
  uint8_t reg[256] = {};
  bool present = true, powered = false;
  uint16_t dac = 0;
  int maxJump = 0;          // largest DAC change in one write
  unsigned writes = 0, nacks = 0;
  uint64_t us = 0;          // simulated time
  std::vector<uint16_t> path;
};
static Vcm V;
static int vcmWrite(void *, uint8_t cci, uint8_t slave, const Write &w) {
  if (cci != 0 || slave != 0x0c || !V.present) { ++V.nacks; return -6; }
  ++V.writes;
  if (w.addr == kLensDac && w.addrBytes == 1 && w.dataBytes == 2) {
    if (w.data > kDacMax) return -22;
    const int d = static_cast<int>(w.data) - static_cast<int>(V.dac);
    if ((d < 0 ? -d : d) > V.maxJump) V.maxJump = d < 0 ? -d : d;
    V.dac = w.data;
    V.path.push_back(w.data);
    return 0;
  }
  if (w.addrBytes != 1 || w.dataBytes != 1) return -22;
  V.reg[w.addr] = static_cast<uint8_t>(w.data);
  if (w.addr == kLensCtrl) V.powered = !(w.data & 1);
  return 0;
}
static void vcmDelay(void *, unsigned us) { V.us += us; }
static const LensIo kIo = {nullptr, vcmWrite, vcmDelay};

static void testExposure() {
  // Main sensor, 1080p60 mode: 10 ms -> lines, FLL unchanged
  const ExpoSensor &g = *expoSensor(0);
  Exposure e{};
  CHECK(plan(g, 1, linesForNs(g, 1, 10000000), 0, 256, &e) == kOk);
  CHECK(e.fll == g.modes[1].fll);
  CHECK(e.exposureNs > 9990000 && e.exposureNs < 10010000);
  CHECK(e.gainCode == 32 && e.gainQ8 == 256);
  // 50 ms does not fit a 60 fps frame: FLL grows, frame rate drops, offset kept
  CHECK(plan(g, 1, linesForNs(g, 1, 50000000), 0, 256, &e) == kOk);
  CHECK(e.fll == e.lines + g.verticalOffset && e.frameNs >= e.exposureNs);
  // gain clamps: 0.5x -> 1x, 64x -> 16x, 2.5x exact
  CHECK(gainCode(g, 128) == 32 && gainCode(g, 256 * 64) == 512 && gainCode(g, 640) == 80);
  // never beyond the sensor limits
  CHECK(plan(g, 1, 0xffffffffu, 0, 256, &e) == kOk && e.fll <= g.maxFll && e.lines + g.verticalOffset <= e.fll);
  CHECK(plan(g, 1, 0, 0, 256, &e) == kOk && e.lines == g.minLines);
  CHECK(plan(g, 7, 100, 0, 256, &e) == kErrInval);
  // SMIA write list: hold, FLL, coarse, gain, release
  Write w[kMaxWrites];
  CHECK(plan(g, 1, 1000, 0, 512, &e) == kOk);
  unsigned n = writes(g, e, w);
  CHECK(n == 5 && w[0].addr == 0x0104 && w[0].data == 1 && w[4].addr == 0x0104 && w[4].data == 0);
  CHECK(w[1].addr == 0x0340 && w[1].data == e.fll && w[2].addr == 0x0202 && w[2].data == 1000 && w[3].data == 64);
  // OV02B: page, vblank = FLL - 1220, exposure, gain x16, trigger
  const ExpoSensor &o = *expoSensor(3);
  CHECK(plan(o, 0, 2000, 0, 256 * 4, &e) == kOk);
  n = writes(o, e, w);
  CHECK(n == 7 && w[0].addr == 0xfd && w[0].data == 1 && w[6].addr == 0xfe && w[6].data == 2);
  const unsigned vb = (w[1].data << 8) | w[2].data;
  CHECK(vb == e.fll - kOvVblankBase);
  CHECK(((w[3].data << 8) | w[4].data) == 2000 && w[5].data == 64);
  for (unsigned c = 0; c < kSensors; ++c) {
    const ExpoSensor &s = *expoSensor(c);
    for (unsigned m = 0; m < s.nModes; ++m)
      for (uint64_t ns = 10000; ns < 2000000000ull; ns *= 3) {
        CHECK(plan(s, m, linesForNs(s, m, ns), 0, 300, &e) == kOk);
        CHECK(e.lines >= s.minLines && e.lines + s.verticalOffset <= e.fll && e.fll <= s.maxFll);
        CHECK(e.gainCode >= s.gainMinCode && e.gainCode <= s.gainMaxCode);
        CHECK(writes(s, e, w) <= kMaxWrites);
      }
  }
  CHECK(expoSensor(4) == nullptr);
}

static void testLens() {
  const Lens &l = *lensForCell(0);
  CHECK(lensForCell(3) && lensForCell(1) == nullptr);
  LensCtl c;
  V = Vcm{};
  CHECK(c.powerUp(&kIo, &l, AfCal{100, 600, true}) == kOk);
  CHECK(c.on && V.powered && V.reg[0x06] == 0x0a && V.reg[0x07] == 0x01 && V.reg[0x08] == 0x43);
  CHECK(V.us >= 1000);                                    // POR wait after leaving power-down
  CHECK(c.lo == 50 && c.hi == 650);                       // 10 % margin
  CHECK(c.move(0) == kOk && V.dac == 50);                 // clamped
  CHECK(c.move(1023) == kOk && V.dac == 650 && V.maxJump <= kLensJump);
  CHECK(c.move(640) == kOk && V.dac == 640);              // small AF step: one write
  const unsigned before = V.writes;
  CHECK(c.move(640) == kOk && V.writes == before);       // no-op move writes nothing
  V.maxJump = 0;
  CHECK(c.park() == kOk && V.dac == l.park && !V.powered && !c.on && V.maxJump <= kParkStep);
  CHECK(c.move(300) == kErrInval);                        // closed
  // missing driver: NACK reported, never "on"
  V = Vcm{}; V.present = false;
  LensCtl d;
  CHECK(d.powerUp(&kIo, &l, AfCal{l.defInf, l.defMacro, false}) == -6 && !d.on && d.lastError == -6);
  // macro GT9772: unlock 0xed first, then power-down cycle
  V = Vcm{};
  LensCtl m;
  CHECK(m.powerUp(&kIo, lensForCell(3), AfCal{200, 900, false}) == kOk && V.reg[0xed] == 0xab && V.reg[0x06] == 0x84);
  CHECK(m.hi == 970 && m.lo == 130);
  uint16_t out[4];
  CHECK(lensRamp(0, 1023, 300, out, 4) == 4 && out[3] == 1023 && out[0] == 300);
  CHECK(lensRamp(5, 5, 10, out, 4) == 0);
}

static void put16(uint8_t *b, unsigned off, uint16_t v) { b[off - kAfBlock] = static_cast<uint8_t>(v >> 8); b[off - kAfBlock + 1] = static_cast<uint8_t>(v); }
static void testCal() {
  const Lens &l = kLens[0];
  uint8_t b[kAfLen + 2];
  for (unsigned i = 0; i < sizeof(b); ++i) b[i] = static_cast<uint8_t>(i * 7);
  put16(b, kAfInf, 120); put16(b, kAfMacro, 540);
  uint16_t c = crc16Arc(b, kAfLen);
  b[kAfLen] = static_cast<uint8_t>(c >> 8); b[kAfLen + 1] = static_cast<uint8_t>(c);
  AfCal a = parseAfCal(b, sizeof(b), l);
  CHECK(a.valid && a.inf == 120 && a.macro == 540);
  c = crc16Buypass(b, kAfLen);
  b[kAfLen] = static_cast<uint8_t>(c >> 8); b[kAfLen + 1] = static_cast<uint8_t>(c);
  CHECK(parseAfCal(b, sizeof(b), l).valid);
  b[3] ^= 1;                                              // corrupted
  a = parseAfCal(b, sizeof(b), l);
  CHECK(!a.valid && a.inf == l.defInf && a.macro == l.defMacro);
  b[3] ^= 1;
  put16(b, kAfMacro, 130);                                // inverted / too close
  c = crc16Arc(b, kAfLen); b[kAfLen] = static_cast<uint8_t>(c >> 8); b[kAfLen + 1] = static_cast<uint8_t>(c);
  CHECK(!parseAfCal(b, sizeof(b), l).valid);
  CHECK(!parseAfCal(b, 10, l).valid && !parseAfCal(nullptr, 26, l).valid);
  // known vectors: "123456789" -> ARC 0xBB3D, BUYPASS 0xFEE8
  const uint8_t v[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(crc16Arc(v, 9) == 0xbb3d && crc16Buypass(v, 9) == 0xfee8);
}

static void testFlash() {
  using namespace guamp_torch;
  CHECK(flashCode(500, kFlashCapMa) == 39 && maForCode(39) == 500);
  CHECK(maForCode(flashCode(5000, kFlashCapMa)) == kFlashCapMa);
  CHECK(maForCode(flashCode(5000, kCapMa)) == kCapMa);       // pre-flash capped at the torch cap
  CHECK(flashCapFor(0x10, 0) == 0 && flashCapFor(0, 0x02) == kFlashHotMa && flashCapFor(0, 0) == kFlashCapMa);
  for (unsigned ms = 0; ms < 3000; ++ms) {
    const uint8_t t = safetyTimer(ms);
    CHECK(t & kSafetyTmrEnable);
    const unsigned cut = ((t & 0x7fu) + 1u) * 10u;          // the PMIC switches off after this
    CHECK(cut >= (ms > 1280 ? 1280 : ms) && cut <= (ms < 10 ? 10 : ms + 10));
  }
  CHECK(clampFlashMs(0, kFlashMaxMs) == kFlashMinMs && clampFlashMs(9999, kFlashMaxMs) == kFlashMaxMs);
  FlashBudget b;
  CHECK(b.allow(1000, 750, 500));
  b.record(1000, 750, 500);
  CHECK(!b.allow(1000 + kFlashMinGapMs - 1, 100, 100));        // cool-down
  CHECK(b.allow(1000 + kFlashMinGapMs, 750, 500));
  b.record(3000, 750, 500);
  b.record(5000, 750, 500);
  CHECK(!b.allow(7000, 750, 500));                             // three full flashes in the window: budget gone
  CHECK(b.allow(7000, 100, 100) == (3ull * 750 * 500 + 100 * 100 <= kFlashBudgetMaMs));
  CHECK(b.allow(5000 + kFlashBudgetWindowMs, 750, 500));        // all three left the window
  CHECK(!b.allow(500, 100, 100));                              // clock before the last flash: refuse
}

int main() {
  testExposure();
  testLens();
  testCal();
  testFlash();
  if (fails) { printf("camera3a_host: %d FAILED\n", fails); return 1; }
  printf("camera3a_host: all checks passed\n");
  return 0;
}
