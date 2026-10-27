// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Adreno 610 power management (analisi/gpu-ios/POWER-A610.md): hardware clock gating, CP register protection,
// the bengal power levels (PLL0/PLL1 + gfx3d RCG + RPM CX/MX corners + GPU bus vote), a simple DCVS governor and the
// nap / slumber idle states of a GPU without GMU. Pure code behind gg2_io (GuampGpuG2.h, read only here) and the
// gp_ops callbacks; host-tested by tests/gpu_power_host.cpp. Nothing here is wired into the boot path yet.
// Sources, Motorola kernel sm6225 (msm-4.19):
//   HWCG     drivers/gpu/msm/adreno-gpulist.h a612_hwcg_regs (used by adreno_gpu_core_a610 .hwcg) and
//            adreno_a6xx.c a6xx_hwcg_set: every register to its value (0 when off), then RBBM_CLOCK_CNTL
//            __get_rbbm_clock_cntl_on (A610/A612 0xAAA8AA82); A610 has no GMU: the SPTPRAC rmw is skipped.
//   Protect  adreno-gpulist.h a630_protected_regs (adreno_gpu_core_a610 .protected_regs) and a6xx_protect_init:
//            CP_PROTECT_CNTL = bit0 | bit1 | bit3; CP_PROTECT_REG + i = start | (end - start) << 18 | noaccess << 31.
//   Levels   arch/arm64/boot/dts/vendor/qcom/bengal-gpu.dtsi qcom,gpu-pwrlevel-bins (speed bins 0, 206, 200, 157, 127),
//            initial-pwrlevel 6 (320 MHz), idle-timeout 80 ms, msm-bus vectors-KBps of grp3d (26 -> 512, ib only).
//            drivers/clk/qcom/gpucc-bengal.c ftbl_gpu_cc_gx_gfx3d_clk_src: 320/465 MHz from PLL1_OUT_AUX / 2, 600+ MHz
//            from PLL0_OUT_AUX2 / 2; gfx3d rate_max on vdd_cx; PLL0 (VCO 1-2 GHz, mode 0) and PLL1 (VCO 0.5-1 GHz,
//            mode 2) on vdd_mx with [VDD_MIN] = 1 GHz, [VDD_NOMINAL] = 2 GHz; vdd-level-bengal.h vdd_corner (RPM levels).
//            clk-alpha-pll.c alpha_pll_round_rate (L = rate / XO, 32-bit alpha rounded up, << 8 into the 40-bit register).
//   Busy     adreno_a6xx.c a6xx_start RBBM_PERFCTR_GPU_BUSY_MASKED = ~0 and the RBBM perfcounter 0 (countable 0, XO
//            ticks: adreno.c adreno_power_stats busy_time = ticks * 10 / 192).
//   Idle     kgsl_pwrctrl.c kgsl_pwrctrl_clk (NAP: branch clocks disabled, state kept; SLUMBER: clocks unprepared and the
//            rate dropped to the lowest level) and kgsl_pwrctrl_disable: regulator_disable, axi (bus) off, clocks off,
//            pwrrail (GX then CX GDSC) off; the wake path is adreno_start (a6xx_start + CP init + MMU start) again.
#ifndef GUAMP_GPU_POWER_H
#define GUAMP_GPU_POWER_H

#include "GuampGpuG2.h"

// --- hardware clock gating -----------------------------------------------------------------------------------------
#define GP_RBBM_CLOCK_CNTL 0x0aeu
#define GP_RBBM_CLOCK_CNTL_ON 0xaaa8aa82u   // A610 / A612
struct gp_reg { uint16_t reg; uint32_t val; };
// a612_hwcg_regs, dword offsets (a6xx_reg.h), in the kgsl order.
static constexpr gp_reg kGpHwcg[] = {
    {0x0b0, 0x22222222}, {0x0b4, 0x02222220}, {0x0b8, 0x00000081}, {0x0bc, 0x0000f3cf},   // SP0 CNTL CNTL2 DELAY HYST
    {0x0c0, 0x22222222}, {0x0c4, 0x22222222}, {0x0c8, 0x22222222}, {0x0cc, 0x00022222},   // TP0 CNTL..CNTL4
    {0x0d0, 0x11111111}, {0x0d4, 0x11111111}, {0x0d8, 0x11111111}, {0x0dc, 0x00011111},   // TP0 DELAY..DELAY4
    {0x0e0, 0x77777777}, {0x0e4, 0x77777777}, {0x0e8, 0x77777777}, {0x0ec, 0x00077777},   // TP0 HYST..HYST4
    {0x0f0, 0x22222222}, {0x0f4, 0x01202222}, {0x0f8, 0x00002220}, {0x100, 0x00040f00},   // RB0 CNTL CNTL2, CCU0, HYST_RB_CCU0
    {0x104, 0x05522022}, {0x105, 0x00005555}, {0x106, 0x00000011}, {0x107, 0x00445044},   // RAC CNTL CNTL2 DELAY HYST
    {0x108, 0x04222222}, {0x111, 0x00002222}, {0x114, 0x02222222}, {0x117, 0x00000002},   // TSE_RAS_RBBM, MODE_VFD, MODE_GPC, DELAY_HLSQ_2
    {0x11b, 0x00002222}, {0x109, 0x00004000}, {0x112, 0x00002222}, {0x115, 0x00000200},   // MODE_HLSQ, DELAY_TSE_RAS_RBBM, DELAY_VFD, DELAY_GPC
    {0x11c, 0x00000000}, {0x10a, 0x00000000}, {0x113, 0x00000000}, {0x116, 0x04104004},   // DELAY_HLSQ, HYST_TSE_RAS_RBBM, HYST_VFD, HYST_GPC
    {0x11d, 0x00000000}, {0x10b, 0x22222222}, {0x110, 0x00000004}, {0x10f, 0x00000002},   // HYST_HLSQ, UCHE CNTL HYST DELAY
    {0x533, 0x00000182}, {0x044, 0x00000000}, {0x042, 0x00000000},                        // ISDB_CNT, RAC_THRESHOLD, SP_HYST
    {0x118, 0x00000222}, {0x119, 0x00000111}, {0x11a, 0x00000555},                        // CLOCK CNTL/DELAY/HYST_GMU_GX
};
#define GP_HWCG_COUNT (sizeof(kGpHwcg) / sizeof(kGpHwcg[0]))

static inline uint32_t gp_gr(const gg2_io *io, uint32_t reg) { return gg2_gr(io, reg); }
static inline void gp_gw(const gg2_io *io, uint32_t reg, uint32_t v) { gg2_gw(io, reg, v); }

// a6xx_hwcg_set: returns false when nothing had to change (already in the requested state).
static inline bool gp_hwcg_set(const gg2_io *io, bool on) {
  const uint32_t cur = gp_gr(io, GP_RBBM_CLOCK_CNTL);
  if (on && cur == GP_RBBM_CLOCK_CNTL_ON) return false;
  if (!on && cur == 0) return false;
  for (size_t i = 0; i < GP_HWCG_COUNT; ++i) gp_gw(io, kGpHwcg[i].reg, on ? kGpHwcg[i].val : 0);
  gp_gw(io, GP_RBBM_CLOCK_CNTL, on ? GP_RBBM_CLOCK_CNTL_ON : 0);
  return true;
}

// --- CP register protection ----------------------------------------------------------------------------------------
#define GP_CP_PROTECT_CNTL 0x84fu
#define GP_CP_PROTECT_REG 0x850u
#define GP_CP_PROTECT_CNTL_VAL ((1u << 0) | (1u << 1) | (1u << 3))   // access protect, fault on violation, last span to end
struct gp_protect { uint32_t start, end; bool noaccess; };
// a630_protected_regs (32 spans, CP_PROTECT_REG + 0..31)
static constexpr gp_protect kGpProtect[] = {
    {0x00000, 0x004ff, false}, {0x00501, 0x00506, false}, {0x0050b, 0x007ff, false}, {0x0050e, 0x0050e, true},
    {0x00510, 0x00510, true},  {0x00534, 0x00534, true},  {0x00800, 0x00882, true},  {0x008a0, 0x008a8, true},
    {0x008ab, 0x008cf, true},  {0x008d0, 0x0098c, false}, {0x00900, 0x0094d, true},  {0x0098d, 0x00bff, true},
    {0x00e00, 0x00e01, true},  {0x00e03, 0x00e0f, true},  {0x03c00, 0x03cc3, true},  {0x03cc4, 0x05cc3, false},
    {0x08630, 0x087ff, true},  {0x08e00, 0x08e00, true},  {0x08e08, 0x08e08, true},  {0x08e50, 0x08e6f, true},
    {0x09624, 0x097ff, true},  {0x09e70, 0x09e71, true},  {0x09e78, 0x09fff, true},  {0x0a630, 0x0a7ff, true},
    {0x0ae02, 0x0ae02, true},  {0x0ae50, 0x0b17f, true},  {0x0b604, 0x0b604, true},  {0x0be02, 0x0be03, true},
    {0x0be20, 0x0d5ff, true},  {0x0f000, 0x0fbff, true},  {0x0fc00, 0x11bff, false}, {0x11c00, 0x11c00, true},
};
#define GP_PROTECT_COUNT (sizeof(kGpProtect) / sizeof(kGpProtect[0]))
static inline uint32_t gp_protect_word(const gp_protect &p) {
  return (p.start & 0x3ffffu) | (((p.end - p.start) & 0x1fffu) << 18) | (p.noaccess ? 1u << 31 : 0u);
}
static inline void gp_protect_init(const gg2_io *io) {
  gp_gw(io, GP_CP_PROTECT_CNTL, GP_CP_PROTECT_CNTL_VAL);
  for (size_t i = 0; i < GP_PROTECT_COUNT; ++i) gp_gw(io, GP_CP_PROTECT_REG + static_cast<uint32_t>(i), gp_protect_word(kGpProtect[i]));
}

// --- power levels --------------------------------------------------------------------------------------------------
// RPM regulator levels (dt-bindings rpm-smd-regulator.h), key "vlvl": CX = smpa3, MX = smpa5 (bengal-regulator.dtsi).
enum : uint32_t {
  GP_RPM_MIN_SVS = 48, GP_RPM_LOW_SVS = 64, GP_RPM_SVS = 128, GP_RPM_SVS_L1 = 192, GP_RPM_NOM = 256,
  GP_RPM_NOM_L1 = 320, GP_RPM_TURBO = 384, GP_RPM_TURBO_L1 = 416
};
enum gp_pll_id : unsigned { GP_PLL_XO = 0, GP_PLL1 = 1, GP_PLL0 = 2 };
#define GP_PLL0_BASE 0x000u
#define GP_SRC_PLL0_AUX2 2u
// grp3d msm-bus ib per bus index (KBps); index 0 = off. bimc KHz = ib / 8 (8 bytes per DDR clock: 800000 = 100 MHz).
static constexpr uint32_t kGpBusIbKBps[12] = {0,       800000,  1600000, 2400000,  3608000,  4376000,
                                               5448000, 6144000, 8136000, 10824000, 12440000, 14432000};
struct gp_level {
  uint32_t mhz;               // GPU core clock
  gp_pll_id pll;              // source PLL
  uint32_t pllMhz;            // PLL output (2x the core clock, the RCG divides by 2)
  uint32_t cx;                // VDD_CX corner the gfx3d rate needs (gpucc rate_max)
  uint32_t mx;                // VDD_MX corner the PLL rate needs (PLL rate_max on vdd_mx)
  uint8_t bus, busMin, busMax;  // grp3d bus indices
};
// bengal-gpu.dtsi speed bin 0 (default), fastest first, plus the XO level kgsl does not list (our boot state).
static constexpr gp_level kGpLevels[] = {
    {980, GP_PLL0, 1960, GP_RPM_TURBO_L1, GP_RPM_NOM, 11, 10, 11},
    {900, GP_PLL0, 1800, GP_RPM_TURBO, GP_RPM_NOM, 11, 10, 11},
    {820, GP_PLL0, 1640, GP_RPM_NOM_L1, GP_RPM_NOM, 10, 10, 11},
    {745, GP_PLL0, 1490, GP_RPM_NOM, GP_RPM_NOM, 9, 8, 10},
    {600, GP_PLL0, 1200, GP_RPM_SVS_L1, GP_RPM_NOM, 8, 8, 9},
    {465, GP_PLL1, 930, GP_RPM_SVS, GP_RPM_MIN_SVS, 7, 5, 8},
    {320, GP_PLL1, 640, GP_RPM_LOW_SVS, GP_RPM_MIN_SVS, 4, 3, 5},
    {19, GP_PLL_XO, 19, GP_RPM_LOW_SVS, GP_RPM_MIN_SVS, 1, 0, 3},   // 19.2 MHz, not a kgsl level
};
#define GP_NLEVELS (sizeof(kGpLevels) / sizeof(kGpLevels[0]))
#define GP_LEVEL_320 6u   // qcom,initial-pwrlevel
#define GP_LEVEL_XO 7u

// Fastest allowed level for a fused speed bin (FMAX / 4.8 MHz + 2, bengal-gpu.dtsi): 0 / 206 -> 980 MHz, 200 -> 950
// (not in our PLL0 table: capped at 900), 157 -> 745, 127 -> 600; an unknown bin gets 600 MHz.
static inline unsigned gp_bin_top(uint32_t speedBin) {
  switch (speedBin) {
    case 0: case 206: return 0;
    case 200: return 1;
    case 157: return 3;
    default: return 4;
  }
}
static inline uint32_t gp_bus_bimc_khz(unsigned busIdx) {
  return busIdx < 12 ? kGpBusIbKBps[busIdx] / 8u : 0;
}

// clk-alpha-pll.c alpha_pll_round_rate for a 40-bit alpha PLL fed by XO: L = rate / XO, alpha = ceil(rem * 2^32 / XO)
// shifted left by 8 into ALPHA (low 32 bits) / ALPHA_U (high 8 bits).
struct gp_lalpha { uint32_t l, alpha, alphaHi; };
static inline gp_lalpha gp_pll_lalpha(uint64_t hz) {
  gp_lalpha r = {};
  r.l = static_cast<uint32_t>(hz / GG2_XO_HZ);
  const uint64_t rem = hz % GG2_XO_HZ;
  if (!rem) return r;
  uint64_t a = (rem << 32) / GG2_XO_HZ;
  if ((rem << 32) % GG2_XO_HZ) a = a + 1;
  const uint64_t a40 = a << 8;
  r.alpha = static_cast<uint32_t>(a40);
  r.alphaHi = static_cast<uint32_t>(a40 >> 32) & 0xffu;
  return r;
}
// USER_CTL of each PLL (gpucc-bengal.c configs): outputs, VCO mode, ALPHA_EN, post divider 1.
static inline gg2_pll_cfg gp_pll_cfg(gp_pll_id pll, uint64_t hz) {
  const gp_lalpha la = gp_pll_lalpha(hz);
  if (pll == GP_PLL0)   // main | aux | aux2, VCO 0, aux2 post div [11:8] = 0
    return {la.l, la.alpha, la.alphaHi, 0x4001055bu, 0x7u | (3u << 20) | (1u << 24) | (0xfu << 8),
            0x7u | (0u << 20) | (1u << 24), 0x1u, 0x0u};
  return {la.l, la.alpha, la.alphaHi, 0x4001055bu, 0x1u | 0x2u | (3u << 20) | (1u << 24) | (7u << 15),
          0x1u | 0x2u | (2u << 20) | (1u << 24), 0x1u, 0x0u};
}
static inline uint32_t gp_pll_base(gp_pll_id pll) { return pll == GP_PLL0 ? GP_PLL0_BASE : GG2_PLL1; }

// gg2_pll1_up generalised to either PLL (clk_alpha_pll_configure + _enable from reset). The caller has parked the
// gfx3d RCG on XO when the PLL feeds it.
struct gp_pll_result { uint32_t mode, l, status, waitedUs; uint64_t hz; bool locked; };
static inline bool gp_pll_up(const gg2_io *io, gp_pll_id pll, const gg2_pll_cfg *c, gp_pll_result *r) {
  *r = {};
  const uint32_t base = gp_pll_base(pll);
  if (gg2_cr(io, base + GG2_PLL_MODE) & GG2_MODE_VOTE_FSM) return false;
  if (gg2_cr(io, base + GG2_PLL_MODE) & GG2_MODE_ON) {
    gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_OUTCTRL, 0);
    io->udelay(io->ctx, 1);
    gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_BYPASSNL | GG2_MODE_RESET_N, 0);
  }
  gg2_cw(io, base + GG2_PLL_L, c->l);
  gg2_cw(io, base + GG2_PLL_ALPHA, c->alpha);
  gg2_cw(io, base + GG2_PLL_CONFIG, c->configCtl);
  gg2_cw(io, base + GG2_PLL_ALPHA_U, c->alphaHi);
  gg2_cu(io, base + GG2_PLL_USER, c->userMask, c->userVal);
  gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_UPDATE_BYPASS, GG2_MODE_UPDATE_BYPASS);
  gg2_cu(io, base + GG2_PLL_TEST_U, c->testUMask, c->testUVal);
  gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_BYPASSNL, GG2_MODE_BYPASSNL);
  io->udelay(io->ctx, 5);
  gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_RESET_N, GG2_MODE_RESET_N);
  for (;;) {
    if (gg2_cr(io, base + GG2_PLL_MODE) & GG2_MODE_LOCK_DET) {
      r->locked = true;
      break;
    }
    if (r->waitedUs >= GG2_LOCK_US) break;
    io->udelay(io->ctx, 1);
    r->waitedUs = r->waitedUs + 1;
  }
  if (r->locked) gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_OUTCTRL, GG2_MODE_OUTCTRL);
  r->mode = gg2_cr(io, base + GG2_PLL_MODE);
  r->l = gg2_cr(io, base + GG2_PLL_L);
  r->status = gg2_cr(io, base + GG2_PLL_STATUS);
  r->hz = gg2_pll_hz(r->l, gg2_cr(io, base + GG2_PLL_ALPHA), gg2_cr(io, base + GG2_PLL_ALPHA_U));
  return r->locked && (r->mode & GG2_MODE_OUTCTRL);
}
static inline void gp_pll_down(const gg2_io *io, gp_pll_id pll) {
  const uint32_t base = gp_pll_base(pll);
  gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_OUTCTRL, 0);
  io->udelay(io->ctx, 1);
  gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_BYPASSNL | GG2_MODE_RESET_N, 0);
}

// Callbacks into the glue (RPM, GDSCs, CP and SMMU re-init). Return 0 / true on success.
enum : unsigned { GP_RAIL_CX = 0, GP_RAIL_MX = 1 };
struct gp_ops {
  void *ctx;
  int (*corner)(void *ctx, unsigned rail, uint32_t level);   // RPM "vlvl" vote, active set; 0 = acked
  int (*bus)(void *ctx, uint32_t bimcKhz);                   // GPU bus vote (bimc KHz from the grp3d ib), 0 = off
  bool (*gdsc)(void *ctx, bool gx, bool on);                 // CX / GX GDSC (GuampGpuM1 sequences)
  bool (*restart)(void *ctx);                                // a6xx_start subset + CP init (GuampGpuM2) after power-up
  bool (*smmu)(void *ctx);                                   // stream 0 path again (GuampGpuM2 M3) after a CX collapse
};

enum gp_err : int {
  GP_OK = 0, GP_E_LEVEL = -1, GP_E_CORNER = -2, GP_E_BUS = -3, GP_E_PLL = -4, GP_E_RCG = -5, GP_E_GDSC = -6,
  GP_E_RESTART = -7, GP_E_SMMU = -8, GP_E_BUSY = -9
};
static inline const char *gp_err_name(int e) {
  switch (e) {
    case GP_OK: return "ok";
    case GP_E_LEVEL: return "bad level";
    case GP_E_CORNER: return "rpm corner not acked";
    case GP_E_BUS: return "bus vote failed";
    case GP_E_PLL: return "pll no lock";
    case GP_E_RCG: return "rcg update stuck";
    case GP_E_GDSC: return "gdsc";
    case GP_E_RESTART: return "cp restart";
    case GP_E_SMMU: return "smmu restore";
    case GP_E_BUSY: return "gpu busy";
    default: return "?";
  }
}

struct gp_state {
  unsigned level = GP_LEVEL_XO;   // current level index into kGpLevels
  unsigned top = 4;               // fastest allowed (gp_bin_top)
  unsigned bottom = GP_LEVEL_320; // slowest DCVS level (XO only on request)
  uint32_t cx = 0, mx = 0, busKhz = 0;   // votes currently held
  gp_pll_id pllOn = GP_PLL_XO;    // PLL currently feeding the RCG
  int lastErr = GP_OK;
};

// Moves the GPU clock to kGpLevels[target]. Order (clk framework vdd_class pre/post + kgsl bus vote): going faster,
// corners first (CX for the gfx3d rate, MX for the PLL rate), then bus, then the clock; going slower, clock first, then
// bus, then corners. Changing the clock parks the RCG on XO, reprograms the PLL from reset, then selects it /2; a PLL no
// longer feeding the RCG is turned off. On a clock failure the RCG stays on XO (always valid) and the level becomes XO.
static inline int gp_set_level(const gg2_io *io, const gp_ops *ops, gp_state *s, unsigned target) {
  if (target >= GP_NLEVELS || (target < s->top && target != GP_LEVEL_XO)) return s->lastErr = GP_E_LEVEL;
  const gp_level &t = kGpLevels[target];
  const bool faster = t.mhz > kGpLevels[s->level].mhz;
  const uint32_t busKhz = gp_bus_bimc_khz(t.bus);
  if (faster) {
    if (t.cx > s->cx) {
      if (ops->corner(ops->ctx, GP_RAIL_CX, t.cx)) return s->lastErr = GP_E_CORNER;
      s->cx = t.cx;
    }
    if (t.mx > s->mx) {
      if (ops->corner(ops->ctx, GP_RAIL_MX, t.mx)) return s->lastErr = GP_E_CORNER;
      s->mx = t.mx;
    }
    if (busKhz > s->busKhz) {
      if (ops->bus(ops->ctx, busKhz)) return s->lastErr = GP_E_BUS;
      s->busKhz = busKhz;
    }
  }
  if (target != s->level) {
    gg2_rcg r;
    if (!gg2_gfx_set(io, GG2_SRC_XO, 0, &r)) return s->lastErr = GP_E_RCG;
    const gp_pll_id old = s->pllOn;
    s->pllOn = GP_PLL_XO;
    s->level = GP_LEVEL_XO;
    if (t.pll != GP_PLL_XO) {
      const gg2_pll_cfg c = gp_pll_cfg(t.pll, static_cast<uint64_t>(t.pllMhz) * 1000000ull);
      gp_pll_result pr;
      if (!gp_pll_up(io, t.pll, &c, &pr)) {
        gp_pll_down(io, t.pll);
        if (old != GP_PLL_XO && old != t.pll) gp_pll_down(io, old);
        return s->lastErr = GP_E_PLL;
      }
      if (!gg2_gfx_set(io, t.pll == GP_PLL0 ? GP_SRC_PLL0_AUX2 : GG2_SRC_PLL1_AUX, 4, &r)) return s->lastErr = GP_E_RCG;
      s->pllOn = t.pll;
    }
    if (old != GP_PLL_XO && old != t.pll) gp_pll_down(io, old);
    s->level = target;
  }
  if (!faster) {
    if (busKhz < s->busKhz) {
      if (ops->bus(ops->ctx, busKhz)) return s->lastErr = GP_E_BUS;
      s->busKhz = busKhz;
    }
    if (t.mx < s->mx && !ops->corner(ops->ctx, GP_RAIL_MX, t.mx)) s->mx = t.mx;   // a refused release keeps the vote
    if (t.cx < s->cx && !ops->corner(ops->ctx, GP_RAIL_CX, t.cx)) s->cx = t.cx;
  }
  return s->lastErr = GP_OK;
}

// --- DCVS -----------------------------------------------------------------------------------------------------------
#define GP_RBBM_PERFCTR_CNTL 0x500u
#define GP_RBBM_PERFCTR_RBBM_SEL_0 0x507u
#define GP_RBBM_PERFCTR_RBBM_0_LO 0x41cu
#define GP_RBBM_PERFCTR_GPU_BUSY_MASKED 0x50bu
#define GP_RBBM_STATUS 0x210u
#define GP_BUSY_MASK 0xfffffffeu         // adreno_gpu_core_a610 .busy_mask
#define GP_IDLE_TIMEOUT_MS 80u           // qcom,idle-timeout
// Busy counter: every block contributes, RBBM counter 0 counts busy XO ticks (countable 0).
static inline void gp_busy_init(const gg2_io *io) {
  gp_gw(io, GP_RBBM_PERFCTR_GPU_BUSY_MASKED, 0xffffffffu);
  gp_gw(io, GP_RBBM_PERFCTR_CNTL, 1);
  gp_gw(io, GP_RBBM_PERFCTR_RBBM_SEL_0, 0);
}
static inline bool gp_gpu_idle(const gg2_io *io) { return (gp_gr(io, GP_RBBM_STATUS) & GP_BUSY_MASK) == 0; }

struct gp_dcvs {
  uint32_t upPct = 80, downPct = 30;   // thresholds on the busy share of a sample
  unsigned downSamples = 2;            // consecutive low samples before stepping down (hysteresis)
  unsigned low = 0;                    // current run of low samples
  uint32_t lastBusy = 0, lastAon = 0;
  bool primed = false;
};
enum gp_action : unsigned { GP_ACT_NONE = 0, GP_ACT_UP = 1, GP_ACT_DOWN = 2, GP_ACT_NAP = 3, GP_ACT_SLUMBER = 4 };
// Busy share in percent from the two counters (wrap-safe 32-bit deltas); aon is 19.2 MHz, busy counts XO ticks.
static inline uint32_t gp_busy_pct(uint32_t dBusy, uint32_t dAon) {
  if (!dAon) return 0;
  const uint64_t p = static_cast<uint64_t>(dBusy) * 100u / dAon;
  return p > 100 ? 100 : static_cast<uint32_t>(p);
}
// One governor sample: idleMs = time since the last submission retired. Faster = a lower level index.
static inline gp_action gp_dcvs_sample(gp_dcvs *d, const gp_state *s, uint32_t busy, uint32_t aon, uint32_t idleMs,
                                       uint32_t *pctOut) {
  const uint32_t dBusy = busy - d->lastBusy, dAon = aon - d->lastAon;
  d->lastBusy = busy;
  d->lastAon = aon;
  const bool primed = d->primed;
  d->primed = true;
  if (pctOut) *pctOut = 0;
  if (idleMs >= GP_IDLE_TIMEOUT_MS) {
    d->low = 0;
    return GP_ACT_SLUMBER;
  }
  if (!primed) return GP_ACT_NONE;
  const uint32_t pct = gp_busy_pct(dBusy, dAon);
  if (pctOut) *pctOut = pct;
  if (pct >= d->upPct) {
    d->low = 0;
    return s->level == GP_LEVEL_XO || s->level > s->top ? GP_ACT_UP : GP_ACT_NONE;
  }
  if (pct <= d->downPct) {
    d->low = d->low + 1;
    if (d->low >= d->downSamples && s->level < s->bottom) {
      d->low = 0;
      return GP_ACT_DOWN;
    }
    return GP_ACT_NONE;
  }
  d->low = 0;
  return GP_ACT_NONE;
}
// The level an action leads to (XO steps up to the initial 320 MHz level).
static inline unsigned gp_dcvs_next(const gp_state *s, gp_action a) {
  if (a == GP_ACT_UP) return s->level == GP_LEVEL_XO ? GP_LEVEL_320 : (s->level > s->top ? s->level - 1 : s->level);
  if (a == GP_ACT_DOWN) return s->level < s->bottom ? s->level + 1 : s->level;
  return s->level;
}

// --- idle states ----------------------------------------------------------------------------------------------------
// SLUMBER (kgsl_pwrctrl_disable without GMU): GPU idle required; clock dropped to XO (lowest), PLLs off, bus vote 0,
// GX GDSC off then CX GDSC off (collapseCx), corners released. Everything the GPU held is lost: wake runs the full
// start again (CX on, SMMU stream path, GX on, level, CP restart = microcode, ring, a6xx_start registers, then HWCG and
// CP protect).
struct gp_saved { unsigned level; bool cxOff; };
static inline int gp_slumber(const gg2_io *io, const gp_ops *ops, gp_state *s, bool collapseCx, gp_saved *sv) {
  if (!gp_gpu_idle(io)) return s->lastErr = GP_E_BUSY;
  sv->level = s->level == GP_LEVEL_XO ? GP_LEVEL_320 : s->level;
  sv->cxOff = false;
  gp_hwcg_set(io, false);
  const int e = gp_set_level(io, ops, s, GP_LEVEL_XO);
  if (e) return e;
  if (ops->bus(ops->ctx, 0) == 0) s->busKhz = 0;
  if (!ops->gdsc(ops->ctx, true, false)) return s->lastErr = GP_E_GDSC;
  if (collapseCx) {
    if (!ops->gdsc(ops->ctx, false, false)) return s->lastErr = GP_E_GDSC;
    sv->cxOff = true;
  }
  if (!ops->corner(ops->ctx, GP_RAIL_MX, 0)) s->mx = 0;
  if (!ops->corner(ops->ctx, GP_RAIL_CX, 0)) s->cx = 0;
  return s->lastErr = GP_OK;
}
static inline int gp_wake(const gg2_io *io, const gp_ops *ops, gp_state *s, const gp_saved *sv) {
  if (ops->corner(ops->ctx, GP_RAIL_CX, kGpLevels[GP_LEVEL_XO].cx)) return s->lastErr = GP_E_CORNER;
  s->cx = kGpLevels[GP_LEVEL_XO].cx;
  if (sv->cxOff) {
    if (!ops->gdsc(ops->ctx, false, true)) return s->lastErr = GP_E_GDSC;
    if (!ops->smmu(ops->ctx)) return s->lastErr = GP_E_SMMU;
  }
  if (!ops->gdsc(ops->ctx, true, true)) return s->lastErr = GP_E_GDSC;
  if (ops->bus(ops->ctx, gp_bus_bimc_khz(kGpLevels[GP_LEVEL_XO].bus))) return s->lastErr = GP_E_BUS;
  s->busKhz = gp_bus_bimc_khz(kGpLevels[GP_LEVEL_XO].bus);
  if (!ops->restart(ops->ctx)) return s->lastErr = GP_E_RESTART;
  gp_hwcg_set(io, true);
  gp_protect_init(io);
  gp_busy_init(io);
  return gp_set_level(io, ops, s, sv->level);
}

#endif  // GUAMP_GPU_POWER_H
