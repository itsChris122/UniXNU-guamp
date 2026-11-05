// Host test of GuampScanoutHw.h (run-scanout-tests.sh): the VIG0 register values of guamp-scanout-hwscale against the Linux
// dpu1 arithmetic (mult_frac phase steps, QSEED3LITE op mode, pixel extension), the round trip back to the boot loader's
// setup (every register a scaled setup writes is one the probe saved), the 1:1 setup and the slot layout.
#include <stdint.h>
#include <stdio.h>
#include <map>
#include "../GuampScanoutHw.h"

static int failures = 0;
#define CHECK(c, ...) do { if (!(c)) { ++failures; printf("FAIL %s:%d ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

using namespace gscanhw;

// The registers as the phone showed them after the boot loader (log GuampScanout: pipe VIG0 ... format 237ff, 720x1600, 2880)
// plus plausible values for the ones never logged.
static void bootSaved(u32 *saved)
{
  for (unsigned i = 0; i < kSavedCount; ++i) {
    u32 v = 0x1000 + i;   // distinct, to see a register that is not put back
    if (kSaved[i] == kSrcSize) v = (1600u << 16) | 720;
    if (kSaved[i] == kSrcXY) v = 0;
    if (kSaved[i] == kYStride0) v = 2880;
    if (kSaved[i] == kSrcOpMode) v = 0x00000000;
    if (kSaved[i] == kScaler + kScOpMode) v = 0;
    saved[i] = v;
  }
}
static u32 savedAt(const u32 *saved, u32 off)
{
  for (unsigned i = 0; i < kSavedCount; ++i) if (kSaved[i] == off) return saved[i];
  return 0xdeadbeef;
}
static void applyRegs(std::map<u32, u32> &regs, const Config &c) { for (unsigned i = 0; i < c.count; ++i) regs[c.regs[i].off] = c.regs[i].value; }

int main()
{
  // mult_frac: exact floor(x * n / d) for the values a canvas can take.
  for (u32 n = 1; n < 4096; n += 7)
    for (u32 d = 1; d < 4096; d += 13) {
      const u64 want = ((u64)(1u << kPhaseShift) * n) / d;
      if (want <= 0xffffffULL) CHECK(multFrac(1u << kPhaseShift, n, d) == want, "multFrac %u/%u", n, d);   // 24-bit step field
    }
  CHECK(liteScaler(0x30000000) && liteScaler(0x30010000) && !liteScaler(0x20040000) && !liteScaler(0x10020000), "version");

  u32 saved[kSavedCount];
  bootSaved(saved);
  const u32 out = (1600u << 16) | 720, fmt = 0x237ff;   // VIG0 format on the phone: 8 bpc, alpha (SRCC3_EN), 4 bytes, unpack 4

  // The canvas of the variant, 750x1666, scaled to the panel.
  Config c;
  CHECK(scaled(c, saved, out, fmt, 750, 1666, slotStride(750)), "scaled 750x1666");
  u32 v = 0;
  CHECK(slotStride(750) == 3008 && slotStride(720) == 2880, "slot stride");
  CHECK(c.has(kSrcSize, &v) && v == ((1666u << 16) | 750), "src size %x", v);
  CHECK(c.has(kYStride0, &v) && v == 3008, "ystride %u", v);
  CHECK(c.has(kSrcXY, &v) && v == 0, "src xy");
  CHECK(c.has(kSrcOpMode, &v) && v == (savedAt(saved, kSrcOpMode) | kPeOverride), "src op mode %x", v);
  CHECK(c.has(kPeC0Req, &v) && v == ((1666u << 16) | 750) && c.has(kPeC12Req, &v) && v == ((1666u << 16) | 750) &&
        c.has(kPeC3Req, &v) && v == ((1666u << 16) | 750), "pe req");
  CHECK(c.has(kPeC0LR, &v) && !v && c.has(kPeC0TB, &v) && !v && c.has(kPeC3LR, &v) && !v, "pe lr/tb");
  // dpu_plane.c: mult_frac(1 << 21, 750, 720) = 2184533, mult_frac(1 << 21, 1666, 1600) = 2183659.
  CHECK(c.has(kScaler + kScStepYH, &v) && v == 2184533, "step x %u", v);
  CHECK(c.has(kScaler + kScStepYV, &v) && v == 2183659, "step y %u", v);
  CHECK(c.has(kScaler + kScStepUVH, &v) && v == 2184533 && c.has(kScaler + kScStepUVV, &v) && v == 2183659, "uv steps");
  CHECK(c.has(kScaler + kScPreload, &v) && v == 0x02040204, "preload %x", v);
  CHECK(c.has(kScaler + kScSrcY, &v) && v == ((1666u << 16) | 750) && c.has(kScaler + kScSrcUV, &v) && v == ((1666u << 16) | 750),
        "scaler src");
  CHECK(c.has(kScaler + kScDst, &v) && v == out, "scaler dst %x", v);
  CHECK(c.has(kScaler + kScInitYH, &v) && !v && c.has(kScaler + kScInitUVV, &v) && !v, "init phase");
  // BIT0 enable, bilinear 3 << 16, blend 1 << 31, BIT14 not DX, BIT10 alpha with the bilinear alpha filter 1 << 29.
  CHECK(c.has(kScaler + kScOpMode, &v) && v == 0xA0034401u, "scaler op mode %x", v);
  CHECK(c.count && c.regs[c.count - 1].off == kScaler + kScOpMode, "scaler op mode written last");
  // Without alpha in the format: no BIT10 / alpha filter.
  CHECK(scaled(c, saved, out, fmt & ~0x100u, 750, 1666, 3008) && c.has(kScaler + kScOpMode, &v) && v == 0x80034001u, "no alpha %x", v);

  // Refused: the same size, 2:1 or more, smaller than the panel, a stride too small / unaligned, a 16-bit format.
  CHECK(!scaled(c, saved, out, fmt, 720, 1600, 2880), "1:1 is not scaled");
  CHECK(!scaled(c, saved, out, fmt, 1440, 1666, 5760) && !scaled(c, saved, out, fmt, 750, 3200, 3008), "2:1");
  CHECK(!scaled(c, saved, out, fmt, 700, 1666, 2800) && !scaled(c, saved, out, fmt, 750, 1500, 3008), "smaller");
  CHECK(!scaled(c, saved, out, fmt, 750, 1666, 2996) && !scaled(c, saved, out, fmt, 750, 1666, 3004), "stride");
  CHECK(!scaled(c, saved, out, 0x2355, 750, 1666, 3008), "16-bit format");
  CHECK(!scaled(c, saved, 0, fmt, 750, 1666, 3008), "no output size");

  // Round trip: the pipe after scaled() then original() is the boot loader's pipe again, register by register.
  std::map<u32, u32> regs, boot;
  for (unsigned i = 0; i < kSavedCount; ++i) regs[kSaved[i]] = saved[i];
  boot = regs;
  CHECK(scaled(c, saved, out, fmt, 750, 1666, 3008), "scaled");
  for (unsigned i = 0; i < c.count; ++i) {
    bool known = false;
    for (unsigned j = 0; j < kSavedCount; ++j) known |= kSaved[j] == c.regs[i].off;
    CHECK(known, "register %x written by the scaled setup is not saved", c.regs[i].off);
  }
  applyRegs(regs, c);
  CHECK(regs != boot, "scaled changes the pipe");
  applyRegs(regs, original(saved));
  CHECK(regs == boot, "original restores the pipe");

  // 1:1 frame in a slot: the boot loader's setup with this size and stride; only the panel's size.
  CHECK(panel(c, saved, out, 720, 1600, 2880), "panel");
  CHECK(c.has(kSrcSize, &v) && v == out && c.has(kYStride0, &v) && v == 2880, "panel size/stride");
  CHECK(c.has(kScaler + kScOpMode, &v) && v == savedAt(saved, kScaler + kScOpMode), "panel scaler as the boot loader's");
  CHECK(c.has(kSrcOpMode, &v) && v == savedAt(saved, kSrcOpMode), "panel op mode");
  CHECK(!panel(c, saved, out, 750, 1666, 3008), "panel refuses the canvas");
  CHECK(!panel(c, saved, out, 720, 1600, 2000), "panel stride");
  applyRegs(regs, c);
  CHECK(regs == boot, "panel 720x1600/2880 = boot setup");

  // Slots after the GOP frame (0x5C000000, 2880 x 1600) below the cont_splash end (+0xF00000).
  Slots s;
  const u64 gop = 0x5C000000ULL, frame = 2880ULL * 1600;
  CHECK(slots(s, gop, frame, 0x500000, 0x500000, gop + 0xF00000), "slots");
  CHECK(s.pa[0] == 0x5C500000ULL && s.pa[1] == 0x5CA00000ULL && s.size == 0x500000, "slot addresses %llx %llx", s.pa[0], s.pa[1]);
  CHECK(3008ULL * 1666 <= s.size && 2880ULL * 1600 <= s.size && 3072ULL * 1666 <= s.size, "a canvas frame fits a slot");
  CHECK(s.pa[1] + s.size <= gop + 0xF00000, "below the dfps part");
  CHECK(!slots(s, gop, frame, 0x400000, 0x500000, gop + 0xF00000), "first slot over the GOP frame");
  CHECK(!slots(s, gop, frame, 0x500000, 0x600000, gop + 0xF00000), "past the end");
  CHECK(!slots(s, gop, frame, 0x500800, 0x500000, gop + 0x1000000), "unaligned");
  CHECK(!slots(s, 0xFFC00000ULL, frame, 0x500000, 0x500000, 0x101000000ULL), "above 4 GiB");

  printf(failures ? "scanout hw: %d failures\n" : "scanout hw: all checks passed\n", failures);
  return failures ? 1 : 0;
}
