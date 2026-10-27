// Host test of GuampGpuG2.h (G2: PLL1 + gfx3d RCG, GPU clock measurement, interrupt accounting) against a fake
// GPUCC / GPU register file with a time base driven by udelay.
#include <stdio.h>
#include <string.h>
#include <map>
#include <vector>
#include "../GuampGpuG2.h"

static int gChecks, gFails;
#define CHECK(c)                                                              \
  do {                                                                        \
    gChecks = gChecks + 1;                                                    \
    if (!(c)) {                                                               \
      gFails = gFails + 1;                                                    \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                     \
    }                                                                         \
  } while (0)

struct Write { unsigned block; uint32_t off, v; uint64_t t; };
struct Fake {
  std::map<uint32_t, uint32_t> cc, gpu;
  std::vector<Write> log;
  uint64_t nowUs = 0;
  // PLL model
  bool lockable = true;
  unsigned lockAfterUs = 20;
  uint64_t resetAt = 0;
  bool resetSeen = false;
  // RCG model
  bool rcgStuck = false;
  // GPU clock model: cycles = Hz of the RCG output when PLL1 runs and the RCG selects it, XO otherwise
  uint64_t cyclesBase = 0, aonBase = 0, lastT = 0;
  uint64_t hz() const {
    const uint32_t cfg = cc.count(GG2_GFX_CFG) ? cc.at(GG2_GFX_CFG) : 0;
    const uint32_t src = (cfg >> 8) & 7;
    const uint32_t mode = cc.count(GG2_PLL1 + GG2_PLL_MODE) ? cc.at(GG2_PLL1 + GG2_PLL_MODE) : 0;
    const uint64_t srcHz = src == 0 ? GG2_XO_HZ : (src == GG2_SRC_PLL1_AUX && (mode & GG2_MODE_OUTCTRL)) ? 640000000ull : 0;
    return gg2_rcg_hz(cfg, srcHz);
  }
  void advance(uint64_t us) {
    cyclesBase += hz() * us / 1000000ull;
    aonBase += GG2_XO_HZ * us / 1000000ull;
    nowUs += us;
  }
  uint32_t ccRead(uint32_t off) {
    uint32_t v = cc.count(off) ? cc[off] : 0;
    if (off == GG2_PLL1 + GG2_PLL_MODE) {
      v &= ~GG2_MODE_LOCK_DET;
      if (resetSeen && lockable && nowUs >= resetAt + lockAfterUs) v |= GG2_MODE_LOCK_DET;
      if (cc.count(0xfff0)) v |= GG2_MODE_LOCK_DET;   // pre-locked
    }
    return v;
  }
  void ccWrite(uint32_t off, uint32_t v) {
    log.push_back({GG2_GPUCC, off, v, nowUs});
    if (off == GG2_PLL1 + GG2_PLL_MODE) {
      const uint32_t old = cc.count(off) ? cc[off] : 0;
      if ((v & GG2_MODE_RESET_N) && !(old & GG2_MODE_RESET_N)) {
        resetSeen = true;
        resetAt = nowUs;
      }
      if (!(v & GG2_MODE_RESET_N)) {
        resetSeen = false;
        cc.erase(0xfff0);
      }
      cc[off] = v & ~GG2_MODE_LOCK_DET;
      return;
    }
    if (off == GG2_GFX_CMD) {
      cc[off] = rcgStuck ? v : (v & ~GG2_RCG_UPDATE);
      return;
    }
    cc[off] = v;
  }
  uint32_t gpuRead(uint32_t reg) {
    if (reg == GG2_RBBM_PERFCTR_CP_0_LO) return static_cast<uint32_t>(cyclesBase);
    if (reg == GG2_CP_ALWAYS_ON_COUNTER_LO) return static_cast<uint32_t>(aonBase);
    return gpu.count(reg) ? gpu[reg] : 0;
  }
  void gpuWrite(uint32_t reg, uint32_t v) {
    log.push_back({GG2_GPU, reg, v, nowUs});
    if (reg == GG2_RBBM_INT_CLEAR_CMD) {
      gpu[GG2_RBBM_INT_0_STATUS] &= ~v;
      return;
    }
    gpu[reg] = v;
  }
};

static Fake *F;
static uint32_t rd(void *, unsigned b, uint32_t off) { return b == GG2_GPUCC ? F->ccRead(off) : F->gpuRead(off / 4); }
static void wr(void *, unsigned b, uint32_t off, uint32_t v) {
  if (b == GG2_GPUCC) F->ccWrite(off, v);
  else F->gpuWrite(off / 4, v);
}
static void ud(void *, unsigned us) { F->advance(us); }
static const gg2_io kIo = {nullptr, rd, wr, ud};

static const Write *lastWrite(uint32_t off) {
  for (size_t i = F->log.size(); i > 0; i--)
    if (F->log[i - 1].block == GG2_GPUCC && F->log[i - 1].off == off) return &F->log[i - 1];
  return nullptr;
}

static void testMath() {
  const uint64_t hz = gg2_pll_hz(0x21, 0x55555555u, 0x55);
  CHECK(hz > 639999000ull && hz <= 640000001ull);
  CHECK(gg2_pll_hz(0x3e, 0, 0x80) == 19200000ull * 62 + 9600000ull);   // pll0: 1200 MHz configuration
  CHECK(gg2_rcg_cfg(0, GG2_SRC_PLL1_AUX, 4) == 0x303u);                 // F(320 MHz, PLL1_OUT_AUX, 2)
  CHECK(gg2_rcg_cfg(0xffff0000u, GG2_SRC_XO, 0) == 0xffff0000u);
  CHECK(gg2_rcg_hz(0x303u, 640000000ull) == 320000000ull);
  CHECK(gg2_rcg_hz(0x0u, GG2_XO_HZ) == GG2_XO_HZ);
  CHECK((kGg2Pll1.userVal & ~kGg2Pll1.userMask) == 0);
  CHECK(((kGg2Pll1.userVal >> 15) & 7u) == 0);   // post divider 1
}

static void testPllFromOff() {
  Fake f;
  F = &f;
  gg2_pll p;
  CHECK(gg2_pll1_up(&kIo, &kGg2Pll1, &p));
  CHECK(p.reconfigured && p.locked && !p.why);
  const uint32_t mode = f.ccRead(GG2_PLL1 + GG2_PLL_MODE);
  CHECK((mode & GG2_MODE_ON) == GG2_MODE_ON);
  CHECK(mode & GG2_MODE_UPDATE_BYPASS);
  CHECK(f.cc[GG2_PLL1 + GG2_PLL_L] == 0x21 && f.cc[GG2_PLL1 + GG2_PLL_ALPHA] == 0x55555555u);
  CHECK(f.cc[GG2_PLL1 + GG2_PLL_ALPHA_U] == 0x55 && f.cc[GG2_PLL1 + GG2_PLL_CONFIG] == 0x4001055bu);
  CHECK((f.cc[GG2_PLL1 + GG2_PLL_USER] & kGg2Pll1.userMask) == kGg2Pll1.userVal);
  CHECK((f.cc[GG2_PLL1 + GG2_PLL_TEST_U] & 1u) == 0);
  // order: BYPASSNL, >= 5 us, RESET_N, lock, then OUTCTRL
  uint64_t tBypass = 0, tReset = 0, tOut = 0;
  for (const Write &w : f.log) {
    if (w.block != GG2_GPUCC || w.off != GG2_PLL1 + GG2_PLL_MODE) continue;
    if ((w.v & GG2_MODE_BYPASSNL) && !tBypass) tBypass = w.t + 1;
    if ((w.v & GG2_MODE_RESET_N) && !tReset) tReset = w.t + 1;
    if ((w.v & GG2_MODE_OUTCTRL) && !tOut) tOut = w.t + 1;
  }
  CHECK(tBypass && tReset && tOut);
  CHECK(tReset >= tBypass + 5);
  CHECK(tOut >= tReset + f.lockAfterUs);
  // L/alpha written before the reset release
  const Write *l = lastWrite(GG2_PLL1 + GG2_PLL_L);
  CHECK(l && l->t + 1 <= tReset);
  CHECK(p.hz > 639999000ull && p.hz <= 640000001ull);
}

static void testPllRefusals() {
  {
    Fake f;
    F = &f;
    f.cc[GG2_PLL1 + GG2_PLL_MODE] = GG2_MODE_VOTE_FSM;
    gg2_pll p;
    CHECK(!gg2_pll1_up(&kIo, &kGg2Pll1, &p));
    CHECK(p.why && f.log.empty());
  }
  {
    Fake f;
    F = &f;
    f.cc[GG2_GFX_CFG] = (GG2_SRC_PLL1_AUX << 8) | 3;
    gg2_pll p;
    CHECK(!gg2_pll1_up(&kIo, &kGg2Pll1, &p));
    CHECK(p.why && f.log.empty());
  }
  {
    Fake f;
    F = &f;
    f.lockable = false;
    gg2_pll p;
    CHECK(!gg2_pll1_up(&kIo, &kGg2Pll1, &p));
    CHECK(!p.locked && p.why && !strcmp(p.why, "pll1 no lock"));
    CHECK(!(f.ccRead(GG2_PLL1 + GG2_PLL_MODE) & GG2_MODE_OUTCTRL));
    CHECK(p.waitedUs >= GG2_LOCK_US && p.waitedUs <= GG2_LOCK_US + 1);
  }
}

static void testPllAlreadyRunning() {
  Fake f;
  F = &f;
  f.cc[GG2_PLL1 + GG2_PLL_MODE] = GG2_MODE_ON;
  f.cc[0xfff0] = 1;   // locked
  f.cc[GG2_PLL1 + GG2_PLL_L] = 0x21;
  f.cc[GG2_PLL1 + GG2_PLL_ALPHA] = 0x55555555u;
  f.cc[GG2_PLL1 + GG2_PLL_ALPHA_U] = 0x55;
  f.cc[GG2_PLL1 + GG2_PLL_USER] = kGg2Pll1.userVal;
  gg2_pll p;
  CHECK(gg2_pll1_up(&kIo, &kGg2Pll1, &p));
  CHECK(!p.reconfigured && p.locked);
  // running with another L: disabled first (OUTCTRL cleared before the L write), then reconfigured
  Fake g;
  F = &g;
  g.cc[GG2_PLL1 + GG2_PLL_MODE] = GG2_MODE_ON;
  g.cc[0xfff0] = 1;
  g.cc[GG2_PLL1 + GG2_PLL_L] = 0x30;
  CHECK(gg2_pll1_up(&kIo, &kGg2Pll1, &p));
  CHECK(p.reconfigured && p.locked);
  CHECK(!g.log.empty() && g.log[0].off == GG2_PLL1 + GG2_PLL_MODE && !(g.log[0].v & GG2_MODE_OUTCTRL));
  CHECK(g.cc[GG2_PLL1 + GG2_PLL_L] == 0x21);
}

static void testRcgAndMeasure() {
  Fake f;
  F = &f;
  uint32_t cyc = 0, aon = 0;
  uint64_t hz = gg2_measure_hz(&kIo, 1000, &cyc, &aon);
  CHECK(hz > 19000000ull && hz < 19400000ull);
  CHECK(f.gpu[GG2_RBBM_PERFCTR_CNTL] == 1 && f.gpu.count(GG2_CP_PERFCTR_CP_SEL_0) && f.gpu[GG2_CP_PERFCTR_CP_SEL_0] == 0);
  gg2_pll p;
  CHECK(gg2_pll1_up(&kIo, &kGg2Pll1, &p));
  gg2_rcg r;
  CHECK(gg2_gfx_set(&kIo, GG2_SRC_PLL1_AUX, 4, &r));
  CHECK((r.cfg & GG2_RCG_CFG_MASK) == 0x303u && !(r.cmd & GG2_RCG_UPDATE));
  hz = gg2_measure_hz(&kIo, 1000, &cyc, &aon);
  CHECK(hz > 319000000ull && hz < 321000000ull);
  CHECK(gg2_rcg_hz(r.cfg, p.hz) > 319990000ull && gg2_rcg_hz(r.cfg, p.hz) <= 320000001ull);
  // back to XO and PLL1 down
  CHECK(gg2_gfx_set(&kIo, GG2_SRC_XO, 0, &r));
  gg2_pll1_down(&kIo);
  CHECK(!(f.ccRead(GG2_PLL1 + GG2_PLL_MODE) & GG2_MODE_ON));
  hz = gg2_measure_hz(&kIo, 1000, &cyc, &aon);
  CHECK(hz > 19000000ull && hz < 19400000ull);
  // stuck UPDATE: bounded, reported
  f.rcgStuck = true;
  CHECK(!gg2_gfx_set(&kIo, GG2_SRC_PLL1_AUX, 4, &r));
  CHECK(r.waitedUs >= GG2_RCG_US && r.waitedUs <= GG2_RCG_US + 1);
  // root off reported
  f.rcgStuck = false;
  f.cc[GG2_GFX_CMD] = GG2_RCG_ROOT_OFF;
  CHECK(!gg2_gfx_set(&kIo, GG2_SRC_XO, 0, &r));
}

static void testIrq() {
  Fake f;
  F = &f;
  gg2_irq_stats s = {};
  CHECK((GG2_INT_MASK & (1u << GG2_INT_CP_CACHE_FLUSH_TS)) && (GG2_INT_MASK & (1u << GG2_INT_CP_HW_ERROR)));
  CHECK(!(GG2_INT_MASK & 1u));   // RBBM_GPU_IDLE not enabled (Linux A6XX_INT_MASK)
  CHECK(GG2_INT_MASK == 0x13d0e3c2u);
  f.gpu[GG2_RBBM_INT_0_STATUS] = (1u << GG2_INT_CP_CACHE_FLUSH_TS) | (1u << GG2_INT_CP_RB);
  gg2_irq_service(&kIo, &s);
  CHECK(s.total == 1 && s.fence == 1 && s.cpRb == 1 && s.hwError == 0 && f.gpu[GG2_RBBM_INT_0_STATUS] == 0);
  CHECK(!f.log.empty() && f.log.back().off == GG2_RBBM_INT_CLEAR_CMD &&
        f.log.back().v == ((1u << GG2_INT_CP_CACHE_FLUSH_TS) | (1u << GG2_INT_CP_RB)));
  f.gpu[GG2_RBBM_INT_0_STATUS] = (1u << GG2_INT_CP_HW_ERROR) | (1u << GG2_INT_CP_AHB_ERROR) | (1u << GG2_INT_RBBM_HANG_DETECT) |
                                 (1u << GG2_INT_UCHE_OOB_ACCESS) | (1u << GG2_INT_RBBM_ATB_BUS_OVERFLOW) | (1u << 3);
  gg2_irq_service(&kIo, &s);
  CHECK(s.hwError == 1 && s.ahbError == 1 && s.hang == 1 && s.uche == 1 && s.atb == 1 && s.other == 1);
  const size_t before = f.log.size();
  gg2_irq_service(&kIo, &s);   // status 0: spurious, no clear write
  CHECK(s.spurious == 1 && f.log.size() == before);
  // storm: a level that never clears gets masked once
  gg2_irq_stats t = {};
  f.gpu[GG2_RBBM_INT_0_MASK] = GG2_INT_MASK;
  for (unsigned i = 0; i < GG2_IRQ_STORM + 5; i++) {
    f.gpu[GG2_RBBM_INT_0_STATUS] = 1u << GG2_INT_CP_SW;
    gg2_irq_service(&kIo, &t);
  }
  CHECK(t.masked == 1 && f.gpu[GG2_RBBM_INT_0_MASK] == 0);
  CHECK(GG2_GPU_INTID == 209u);
}

int main() {
  testMath();
  testPllFromOff();
  testPllRefusals();
  testPllAlreadyRunning();
  testRcgAndMeasure();
  testIrq();
  printf("gpu_g2_host: %d checks, %d failures\n", gChecks, gFails);
  return gFails ? 1 : 0;
}
