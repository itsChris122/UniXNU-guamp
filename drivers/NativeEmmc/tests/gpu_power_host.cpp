// Host test of GuampGpuPower.h (A610 HWCG, CP protect, power levels, DCVS, slumber / wake) against a fake GPUCC / GPU
// register file and fake RPM / GDSC / CP callbacks that record the order of every step.
#include <stdio.h>
#include <string.h>
#include <map>
#include <string>
#include <vector>
#include "../GuampGpuPower.h"

static int gChecks, gFails;
#define CHECK(c)                                                              \
  do {                                                                        \
    gChecks = gChecks + 1;                                                    \
    if (!(c)) {                                                               \
      gFails = gFails + 1;                                                    \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                     \
    }                                                                         \
  } while (0)

struct Fake {
  std::map<uint32_t, uint32_t> cc, gpu;
  std::vector<std::string> ev;   // ordered events: register writes of interest and callbacks
  bool lockable = true, rcgStuck = false, gpuBusy = false;
  int refuseCorner = -1;         // rail whose raise is refused
  std::map<uint32_t, bool> pllReset;
  unsigned gpuWrites = 0;
  uint32_t ccRead(uint32_t off) {
    uint32_t v = cc.count(off) ? cc[off] : 0;
    for (uint32_t base : {GP_PLL0_BASE, (uint32_t)GG2_PLL1})
      if (off == base + GG2_PLL_MODE) {
        v &= ~GG2_MODE_LOCK_DET;
        if (lockable && (v & GG2_MODE_RESET_N)) v |= GG2_MODE_LOCK_DET;
      }
    if (off == GG2_GFX_CMD && rcgStuck) v |= GG2_RCG_UPDATE;
    return v;
  }
  void ccWrite(uint32_t off, uint32_t v) {
    if (off == GG2_GFX_CMD) {
      v &= ~GG2_RCG_UPDATE;   // the update completes at once
      ev.push_back("rcg " + std::to_string((cc[GG2_GFX_CFG] >> 8) & 7) + "/" + std::to_string(cc[GG2_GFX_CFG] & 0x1f));
    }
    if (off == GP_PLL0_BASE + GG2_PLL_L) ev.push_back("pll0 L " + std::to_string(v));
    if (off == GG2_PLL1 + GG2_PLL_L) ev.push_back("pll1 L " + std::to_string(v));
    cc[off] = v;
  }
  uint32_t gpuRead(uint32_t reg) {
    if (reg == GP_RBBM_STATUS) return gpuBusy ? 0x80000001u : 0x1u;
    return gpu.count(reg) ? gpu[reg] : 0;
  }
  void gpuWrite(uint32_t reg, uint32_t v) {
    gpuWrites = gpuWrites + 1;
    if (reg == GP_RBBM_CLOCK_CNTL) ev.push_back(v ? "hwcg on" : "hwcg off");
    if (reg == GP_CP_PROTECT_CNTL) ev.push_back("protect");
    gpu[reg] = v;
  }
};
static Fake *gF;
static uint32_t rd(void *, unsigned block, uint32_t off) { return block == GG2_GPUCC ? gF->ccRead(off) : gF->gpuRead(off / 4); }
static void wr(void *, unsigned block, uint32_t off, uint32_t v) {
  if (block == GG2_GPUCC) gF->ccWrite(off, v);
  else gF->gpuWrite(off / 4, v);
}
static void ud(void *, unsigned) {}
static int corner(void *, unsigned rail, uint32_t level) {
  if (static_cast<int>(rail) == gF->refuseCorner && level) return -6;
  gF->ev.push_back(std::string(rail == GP_RAIL_CX ? "cx " : "mx ") + std::to_string(level));
  return 0;
}
static int bus(void *, uint32_t khz) {
  gF->ev.push_back("bus " + std::to_string(khz));
  return 0;
}
static bool gdsc(void *, bool gx, bool on) {
  gF->ev.push_back(std::string(gx ? "gx " : "cxgdsc ") + (on ? "on" : "off"));
  return true;
}
static bool restart(void *) {
  gF->ev.push_back("restart");
  return true;
}
static bool smmu(void *) {
  gF->ev.push_back("smmu");
  return true;
}
static const gg2_io kIo = {nullptr, rd, wr, ud};
static const gp_ops kOps = {nullptr, corner, bus, gdsc, restart, smmu};

static int indexOf(const Fake &f, const std::string &e) {
  for (size_t i = 0; i < f.ev.size(); ++i)
    if (f.ev[i] == e) return static_cast<int>(i);
  return -1;
}

static void testHwcgProtect() {
  Fake f;
  gF = &f;
  CHECK(GP_HWCG_COUNT == 46);
  CHECK(gp_hwcg_set(&kIo, true));
  CHECK(f.gpu[GP_RBBM_CLOCK_CNTL] == 0xaaa8aa82u);
  CHECK(f.gpu[0x0b0] == 0x22222222u && f.gpu[0x533] == 0x182u && f.gpu[0x11a] == 0x555u && f.gpu[0x100] == 0x00040f00u);
  CHECK(f.gpuWrites == GP_HWCG_COUNT + 1);
  CHECK(f.ev.back() == "hwcg on");   // the top-level enable is the last write
  CHECK(!gp_hwcg_set(&kIo, true));   // already on: nothing written
  CHECK(f.gpuWrites == GP_HWCG_COUNT + 1);
  CHECK(gp_hwcg_set(&kIo, false));
  CHECK(f.gpu[0x0b0] == 0 && f.gpu[GP_RBBM_CLOCK_CNTL] == 0);
  CHECK(!gp_hwcg_set(&kIo, false));
  CHECK(GP_PROTECT_COUNT == 32);
  gp_protect_init(&kIo);
  CHECK(f.gpu[GP_CP_PROTECT_CNTL] == 0xbu);
  CHECK(f.gpu[0x850] == (0x4ffu << 18));
  CHECK(f.gpu[0x853] == (0x50eu | (1u << 31)));
  CHECK(f.gpu[0x85f] == (0x3cc4u | ((0x5cc3u - 0x3cc4u) << 18)));
  CHECK(f.gpu[0x86e] == (0xfc00u | ((0x11bffu - 0xfc00u) << 18)));
  CHECK(f.gpu[0x86f] == (0x11c00u | (1u << 31)));
}

static void testLalpha() {
  gp_lalpha a = gp_pll_lalpha(640000000ull);
  CHECK(a.l == 0x21 && a.alpha == 0x55555600u && a.alphaHi == 0x55);
  a = gp_pll_lalpha(930000000ull);
  CHECK(a.l == 48 && a.alpha == 0 && a.alphaHi == 0x70);
  a = gp_pll_lalpha(1200000000ull);
  CHECK(a.l == 0x3e && a.alpha == 0 && a.alphaHi == 0x80);   // gpu_cc_pll0_config
  a = gp_pll_lalpha(1920000000ull);
  CHECK(a.l == 100 && a.alpha == 0 && a.alphaHi == 0);
  for (const gp_level &l : kGpLevels) {
    if (l.pll == GP_PLL_XO) continue;
    const uint64_t hz = static_cast<uint64_t>(l.pllMhz) * 1000000ull;
    a = gp_pll_lalpha(hz);
    const uint64_t got = gg2_pll_hz(a.l, a.alpha, a.alphaHi);
    CHECK(got >= hz && got - hz < 1000);   // rounded up by at most one alpha step
    CHECK(l.pllMhz == 2 * l.mhz);
    CHECK(l.pll == GP_PLL1 ? (l.pllMhz >= 500 && l.pllMhz <= 1000) : (l.pllMhz >= 1000 && l.pllMhz <= 2000));
    CHECK(l.pll == GP_PLL1 ? l.mx == GP_RPM_MIN_SVS : l.mx == GP_RPM_NOM);   // vdd_mx rate_max
  }
  // pll configs: VCO mode and outputs
  gg2_pll_cfg c = gp_pll_cfg(GP_PLL0, 1200000000ull);
  CHECK((c.userVal & (3u << 20)) == 0 && (c.userVal & 7u) == 7u && (c.userVal & (1u << 24)));
  c = gp_pll_cfg(GP_PLL1, 640000000ull);
  CHECK((c.userVal & (3u << 20)) == (2u << 20) && (c.userVal & 3u) == 3u);
  CHECK(gp_bin_top(0) == 0 && gp_bin_top(206) == 0 && gp_bin_top(200) == 1 && gp_bin_top(157) == 3 && gp_bin_top(127) == 4);
  CHECK(gp_bin_top(77) == 4);
  CHECK(gp_bus_bimc_khz(4) == 451000u && gp_bus_bimc_khz(11) == 1804000u && gp_bus_bimc_khz(0) == 0 && gp_bus_bimc_khz(12) == 0);
}

static void testLevels() {
  Fake f;
  gF = &f;
  gp_state s;
  s.top = gp_bin_top(127);
  // XO -> 320: corners, bus, then PLL1 and the RCG on PLL1_OUT_AUX / 2
  CHECK(gp_set_level(&kIo, &kOps, &s, GP_LEVEL_320) == GP_OK);
  CHECK(s.level == GP_LEVEL_320 && s.pllOn == GP_PLL1 && s.cx == GP_RPM_LOW_SVS && s.mx == GP_RPM_MIN_SVS);
  CHECK(indexOf(f, "cx 64") >= 0 && indexOf(f, "cx 64") < indexOf(f, "mx 48"));
  CHECK(indexOf(f, "mx 48") < indexOf(f, "bus 451000"));
  CHECK(indexOf(f, "bus 451000") < indexOf(f, "pll1 L 33"));
  CHECK(indexOf(f, "pll1 L 33") < indexOf(f, "rcg 3/3"));
  CHECK(indexOf(f, "rcg 0/0") >= 0 && indexOf(f, "rcg 0/0") < indexOf(f, "pll1 L 33"));   // parked on XO first
  // 320 -> 600: PLL0 at 1200 MHz, PLL1 off afterwards, MX at NOM
  f.ev.clear();
  CHECK(gp_set_level(&kIo, &kOps, &s, 4) == GP_OK);
  CHECK(s.level == 4 && s.pllOn == GP_PLL0 && s.cx == GP_RPM_SVS_L1 && s.mx == GP_RPM_NOM);
  CHECK(indexOf(f, "cx 192") < indexOf(f, "mx 256") && indexOf(f, "mx 256") < indexOf(f, "bus 1017000"));
  CHECK(indexOf(f, "bus 1017000") < indexOf(f, "pll0 L 62") && indexOf(f, "pll0 L 62") < indexOf(f, "rcg 2/3"));
  CHECK((f.cc[GG2_PLL1 + GG2_PLL_MODE] & GG2_MODE_OUTCTRL) == 0);
  CHECK((f.cc[GP_PLL0_BASE + GG2_PLL_MODE] & GG2_MODE_ON) == GG2_MODE_ON);
  CHECK(f.cc[GP_PLL0_BASE + GG2_PLL_ALPHA_U] == 0x80);
  // the bin forbids 980 MHz
  f.ev.clear();
  CHECK(gp_set_level(&kIo, &kOps, &s, 0) == GP_E_LEVEL && s.level == 4 && f.ev.empty());
  // 600 -> 465 (slower): clock first, then bus, then corners
  CHECK(gp_set_level(&kIo, &kOps, &s, 5) == GP_OK);
  CHECK(s.level == 5 && s.pllOn == GP_PLL1);
  CHECK(indexOf(f, "rcg 3/3") < indexOf(f, "bus 768000"));
  CHECK(indexOf(f, "bus 768000") < indexOf(f, "mx 48") && indexOf(f, "mx 48") < indexOf(f, "cx 128"));
  CHECK(indexOf(f, "pll1 L 48") >= 0);
  CHECK((f.cc[GP_PLL0_BASE + GG2_PLL_MODE] & GG2_MODE_OUTCTRL) == 0);
  CHECK(s.cx == GP_RPM_SVS && s.mx == GP_RPM_MIN_SVS);
  // a refused corner raise changes nothing else
  f.ev.clear();
  f.refuseCorner = GP_RAIL_CX;
  CHECK(gp_set_level(&kIo, &kOps, &s, 4) == GP_E_CORNER && s.level == 5 && f.ev.empty());
  f.refuseCorner = -1;
  // a PLL that never locks leaves the RCG on XO and the level at XO
  f.lockable = false;
  CHECK(gp_set_level(&kIo, &kOps, &s, 4) == GP_E_PLL);
  CHECK(s.level == GP_LEVEL_XO && s.pllOn == GP_PLL_XO && ((f.cc[GG2_GFX_CFG] >> 8) & 7) == 0);
  CHECK((f.cc[GG2_PLL1 + GG2_PLL_MODE] & GG2_MODE_OUTCTRL) == 0 && (f.cc[GP_PLL0_BASE + GG2_PLL_MODE] & GG2_MODE_OUTCTRL) == 0);
  f.lockable = true;
  // a stuck RCG update is reported
  f.rcgStuck = true;
  CHECK(gp_set_level(&kIo, &kOps, &s, GP_LEVEL_320) == GP_E_RCG);
  f.rcgStuck = false;
  CHECK(gp_set_level(&kIo, &kOps, &s, GP_LEVEL_320) == GP_OK && s.level == GP_LEVEL_320);
  CHECK(!strcmp(gp_err_name(GP_E_PLL), "pll no lock") && !strcmp(gp_err_name(42), "?"));
}

static void testDcvs() {
  gp_state s;
  s.top = 4;
  s.level = GP_LEVEL_320;
  gp_dcvs d;
  uint32_t pct = 7;
  CHECK(gp_dcvs_sample(&d, &s, 0, 0, 0, &pct) == GP_ACT_NONE && pct == 0);   // first sample primes
  CHECK(gp_dcvs_sample(&d, &s, 900000, 1000000, 0, &pct) == GP_ACT_UP && pct == 90);
  CHECK(gp_dcvs_next(&s, GP_ACT_UP) == 5);
  s.level = 4;   // at the bin top: no further up
  CHECK(gp_dcvs_sample(&d, &s, 1900000, 2000000, 0, &pct) == GP_ACT_NONE && pct == 100);
  CHECK(gp_dcvs_next(&s, GP_ACT_UP) == 4);
  CHECK(gp_dcvs_sample(&d, &s, 2000000, 3000000, 0, &pct) == GP_ACT_NONE && pct == 10);   // first low sample
  CHECK(gp_dcvs_sample(&d, &s, 2100000, 4000000, 0, &pct) == GP_ACT_DOWN);                // second: step down
  CHECK(gp_dcvs_next(&s, GP_ACT_DOWN) == 5);
  CHECK(gp_dcvs_sample(&d, &s, 2150000, 5000000, 0, &pct) == GP_ACT_NONE);                // run restarted
  CHECK(gp_dcvs_sample(&d, &s, 2650000, 6000000, 0, &pct) == GP_ACT_NONE && pct == 50);   // middle band resets it
  CHECK(gp_dcvs_sample(&d, &s, 2650000, 7000000, 0, &pct) == GP_ACT_NONE);
  s.level = GP_LEVEL_320;   // at the bottom: no further down
  CHECK(gp_dcvs_sample(&d, &s, 2650000, 8000000, 0, &pct) == GP_ACT_NONE);
  CHECK(gp_dcvs_sample(&d, &s, 2650000, 9000000, GP_IDLE_TIMEOUT_MS, &pct) == GP_ACT_SLUMBER);
  s.level = GP_LEVEL_XO;
  CHECK(gp_dcvs_next(&s, GP_ACT_UP) == GP_LEVEL_320);
  // wrap-safe deltas
  gp_dcvs w;
  w.primed = true;
  w.lastBusy = 0xffffff80u;
  w.lastAon = 0xffffff00u;
  CHECK(gp_dcvs_sample(&w, &s, 0x120u, 0x100u, 0, &pct) == GP_ACT_UP && pct == 81);
  CHECK(gp_busy_pct(5, 0) == 0 && gp_busy_pct(300, 200) == 100);
}

static void testSlumberWake() {
  Fake f;
  gF = &f;
  gp_state s;
  s.top = 4;
  CHECK(gp_set_level(&kIo, &kOps, &s, 4) == GP_OK);
  gp_hwcg_set(&kIo, true);
  gp_saved sv;
  f.gpuBusy = true;
  CHECK(gp_slumber(&kIo, &kOps, &s, true, &sv) == GP_E_BUSY && s.level == 4);
  f.gpuBusy = false;
  f.ev.clear();
  CHECK(gp_slumber(&kIo, &kOps, &s, true, &sv) == GP_OK);
  CHECK(sv.level == 4 && sv.cxOff && s.level == GP_LEVEL_XO && s.busKhz == 0 && s.cx == 0 && s.mx == 0);
  CHECK(indexOf(f, "hwcg off") == 0);
  CHECK(indexOf(f, "rcg 0/0") < indexOf(f, "bus 0") && indexOf(f, "bus 0") < indexOf(f, "gx off"));
  CHECK(indexOf(f, "gx off") < indexOf(f, "cxgdsc off") && indexOf(f, "cxgdsc off") < indexOf(f, "mx 0"));
  CHECK(indexOf(f, "mx 0") < indexOf(f, "cx 0"));
  CHECK((f.cc[GP_PLL0_BASE + GG2_PLL_MODE] & GG2_MODE_OUTCTRL) == 0);
  f.ev.clear();
  CHECK(gp_wake(&kIo, &kOps, &s, &sv) == GP_OK);
  CHECK(s.level == 4 && s.pllOn == GP_PLL0);
  CHECK(indexOf(f, "cx 64") == 0 && indexOf(f, "cxgdsc on") == 1 && indexOf(f, "smmu") == 2 && indexOf(f, "gx on") == 3);
  CHECK(indexOf(f, "gx on") < indexOf(f, "restart") && indexOf(f, "restart") < indexOf(f, "hwcg on"));
  CHECK(indexOf(f, "hwcg on") < indexOf(f, "protect") && indexOf(f, "protect") < indexOf(f, "pll0 L 62"));
  CHECK(f.gpu[GP_RBBM_PERFCTR_GPU_BUSY_MASKED] == 0xffffffffu && f.gpu[GP_RBBM_PERFCTR_CNTL] == 1);
  // a slumber that keeps CX skips the SMMU restore on wake
  CHECK(gp_slumber(&kIo, &kOps, &s, false, &sv) == GP_OK && !sv.cxOff);
  f.ev.clear();
  CHECK(gp_wake(&kIo, &kOps, &s, &sv) == GP_OK && indexOf(f, "smmu") < 0 && indexOf(f, "cxgdsc on") < 0);
  // waking from XO goes to the initial 320 MHz level
  s.level = GP_LEVEL_XO;
  CHECK(gp_set_level(&kIo, &kOps, &s, GP_LEVEL_XO) == GP_OK);
  CHECK(gp_slumber(&kIo, &kOps, &s, false, &sv) == GP_OK && sv.level == GP_LEVEL_320);
}

// The RBBM_STATUS governor of GuampGpuDcvs.inc: limits, windows, hysteresis, the rail order of the switches it makes
// and the failure path, driven through gp_set_level on the fake register file.
static void testGovernor() {
  CHECK(GP_RBBM_STATUS * 4u < 0xf000u);   // the only GPU register the governor reads
  CHECK(gp_gov_busy(0x2u) && !gp_gov_busy(0x1u) && !gp_gov_busy(0x80000001u) && gp_gov_busy(0x00800000u));
  // speed bin: 0x6006 bits 5..12 = dword 0x6004 bits 21..28
  CHECK(gp_speed_bin(0) == 0 && gp_speed_bin(206u << 21) == 206 && gp_speed_bin((157u << 21) | 0x1fffffu | (7u << 29)) == 157);
  // limits: bin top, cap in MHz, never slower than 320
  CHECK(gp_gov_top(0, 0) == 0 && gp_gov_top(206, 0) == 0 && gp_gov_top(157, 0) == 3 && gp_gov_top(99, 0) == 4);
  CHECK(gp_gov_top(0, 980) == 0 && gp_gov_top(0, 979) == 1 && gp_gov_top(0, 600) == 4 && gp_gov_top(0, 465) == 5);
  CHECK(gp_gov_top(0, 320) == GP_LEVEL_320 && gp_gov_top(0, 100) == GP_LEVEL_320 && gp_gov_top(127, 900) == 4);
  // windows
  gp_state s;
  s.level = GP_LEVEL_320;
  s.top = 0;
  gp_gov g;
  CHECK(gp_gov_window(&g, &s, GP_GOV_UP) == 0);              // 70 % busy: straight to the top
  CHECK(gp_gov_window(&g, &s, GP_GOV_UP - 1) == GP_LEVEL_320);
  s.level = 0;
  for (unsigned i = 1; i < GP_GOV_QUIET; ++i) CHECK(gp_gov_window(&g, &s, GP_GOV_DOWN - 1) == 0);
  CHECK(gp_gov_window(&g, &s, 0) == 1 && g.quiet == 0);      // tenth quiet window: one level down
  for (unsigned i = 1; i < GP_GOV_QUIET; ++i) CHECK(gp_gov_window(&g, &s, 0) == 0);
  CHECK(gp_gov_window(&g, &s, GP_GOV_DOWN) == 0 && g.quiet == 0);   // a middle window restarts the quiet run
  for (unsigned i = 1; i < GP_GOV_QUIET; ++i) CHECK(gp_gov_window(&g, &s, 0) == 0);
  CHECK(gp_gov_window(&g, &s, 0) == 1);
  s.level = GP_LEVEL_320;
  for (unsigned i = 0; i < 3 * GP_GOV_QUIET; ++i) CHECK(gp_gov_window(&g, &s, 0) == GP_LEVEL_320);   // the bottom holds
  s.level = GP_LEVEL_XO;
  CHECK(gp_gov_window(&g, &s, 0) == GP_LEVEL_320 && gp_gov_window(&g, &s, GP_GOV_WINDOW) == 0);
  // a cap: busy goes to the capped top only
  s.level = GP_LEVEL_320;
  s.top = gp_gov_top(0, 465);
  CHECK(gp_gov_window(&g, &s, GP_GOV_WINDOW) == 5);
  // failure bookkeeping
  s.top = 0;
  CHECK(gp_gov_failed(&s, 0, GP_LEVEL_320) == GP_LEVEL_320 && s.top == 1);
  CHECK(gp_gov_failed(&s, 4, 5) == 5 && s.top == 5);
  CHECK(gp_gov_failed(&s, 2, 5) == 5 && s.top == 5);           // already capped lower
  CHECK(gp_gov_failed(&s, GP_LEVEL_320, 5) == 5 && s.top == 5); // a failed step down caps nothing
  CHECK(gp_gov_failed(&s, GP_LEVEL_320, GP_LEVEL_XO) == GP_LEVEL_320);
  // A switch the governor asks for, through gp_set_level: up raises CX then MX before the PLL and the RCG, down
  // lowers them after the RCG.
  Fake f;
  gF = &f;
  gp_state r;
  r.level = GP_LEVEL_320;
  r.pllOn = GP_PLL1;
  r.cx = GP_RPM_LOW_SVS;
  r.mx = GP_RPM_MIN_SVS;
  r.top = gp_gov_top(0, 0);
  gp_gov h;
  unsigned t = gp_gov_window(&h, &r, GP_GOV_WINDOW);
  CHECK(t == 0 && gp_set_level(&kIo, &kOps, &r, t) == GP_OK && r.level == 0 && r.pllOn == GP_PLL0);
  CHECK(indexOf(f, "cx 416") >= 0 && indexOf(f, "cx 416") < indexOf(f, "mx 256"));
  CHECK(indexOf(f, "mx 256") < indexOf(f, "pll0 L 102") && indexOf(f, "pll0 L 102") < indexOf(f, "rcg 2/3"));
  CHECK((f.cc[GG2_PLL1 + GG2_PLL_MODE] & GG2_MODE_OUTCTRL) == 0);
  for (unsigned i = 0; i < GP_GOV_QUIET; ++i) t = gp_gov_window(&h, &r, 0);
  f.ev.clear();
  CHECK(t == 1 && gp_set_level(&kIo, &kOps, &r, t) == GP_OK && r.level == 1 && r.cx == GP_RPM_TURBO);
  CHECK(indexOf(f, "rcg 2/3") >= 0 && indexOf(f, "rcg 2/3") < indexOf(f, "cx 384") && indexOf(f, "mx 256") < 0);
  // down to 320 one level per quiet run: PLL1 back, MX and CX released after the RCG
  f.ev.clear();
  for (unsigned lv = 2; lv <= GP_LEVEL_320; ++lv) {
    for (unsigned i = 0; i < GP_GOV_QUIET; ++i) t = gp_gov_window(&h, &r, 0);
    CHECK(t == lv && gp_set_level(&kIo, &kOps, &r, t) == GP_OK && r.level == lv);
  }
  CHECK(r.pllOn == GP_PLL1 && r.cx == GP_RPM_LOW_SVS && r.mx == GP_RPM_MIN_SVS);
  CHECK(indexOf(f, "rcg 3/3") >= 0 && indexOf(f, "rcg 3/3") < indexOf(f, "mx 48"));   // 600 -> 465: PLL1, then MX
  CHECK(indexOf(f, "mx 48") < indexOf(f, "cx 128") && indexOf(f, "cx 128") < indexOf(f, "cx 64"));
  CHECK((f.cc[GP_PLL0_BASE + GG2_PLL_MODE] & GG2_MODE_OUTCTRL) == 0);
  // a PLL0 that never locks: the switch fails on XO, the top drops below it and 320 comes back on PLL1
  f.lockable = false;
  t = gp_gov_window(&h, &r, GP_GOV_WINDOW);
  CHECK(t == 0 && gp_set_level(&kIo, &kOps, &r, t) == GP_E_PLL && r.level == GP_LEVEL_XO);
  f.lockable = true;
  const unsigned back = gp_gov_failed(&r, t, GP_LEVEL_320);
  CHECK(back == GP_LEVEL_320 && r.top == 1);
  CHECK(r.cx == GP_RPM_TURBO_L1 && r.mx == GP_RPM_NOM);   // raised before the PLL failed
  f.ev.clear();
  CHECK(gp_set_level(&kIo, &kOps, &r, back) == GP_OK && r.level == GP_LEVEL_320 && r.pllOn == GP_PLL1);
  CHECK(r.cx == GP_RPM_LOW_SVS && r.mx == GP_RPM_MIN_SVS);   // released once 320 runs again, after its RCG switch
  CHECK(indexOf(f, "rcg 3/3") >= 0 && indexOf(f, "rcg 3/3") < indexOf(f, "mx 48") && indexOf(f, "mx 48") < indexOf(f, "cx 64"));
  CHECK(gp_set_level(&kIo, &kOps, &r, 0) == GP_E_LEVEL && r.level == GP_LEVEL_320);   // never retried
  // a refused CX raise: nothing switched, the level stays
  f.refuseCorner = GP_RAIL_CX;
  f.ev.clear();
  CHECK(gp_set_level(&kIo, &kOps, &r, 1) == GP_E_CORNER && r.level == GP_LEVEL_320 && f.ev.empty());
  f.refuseCorner = -1;
}

int main() {
  testHwcgProtect();
  testLalpha();
  testLevels();
  testDcvs();
  testSlumberWake();
  testGovernor();
  printf("gpu_power_host: %d checks, %d failures\n", gChecks, gFails);
  return gFails ? 1 : 0;
}
