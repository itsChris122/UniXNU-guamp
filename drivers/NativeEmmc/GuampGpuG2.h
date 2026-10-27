// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Adreno 610 milestone G2 (analisi/gpu-ios/PROGETTO-GPU-IOS.md): GPU clock above XO, the GPU interrupt and a
// CP recovery. Pure code (register sequencing and decoding) behind gg2_io; the kernel glue is in GpuService.inc
// (namespace gpug2, guamp-gpu-g2=1|2), host-tested by tests/gpu_g2_host.cpp. Sources, Motorola kernel sm6225:
//   PLL      drivers/clk/qcom/gpucc-bengal.c gpu_cc_pll1 (offset 0x100, CLK_ALPHA_PLL_TYPE_DEFAULT, VCO 500-1000 MHz
//            mode 2, "640MHz configuration": L 0x21, alpha 0x55_55555555, config_ctl 0x4001055b) and gpu_cc_pll1_out_aux
//            (post div shift 15 width 3, table {0: /1}); clk-alpha-pll.c clk_alpha_pll_configure / _enable / _disable
//            (default register layout: L 4, ALPHA 8, ALPHA_U c, USER_CTL 10, USER_CTL_U 14, CONFIG_CTL 18,
//            TEST_CTL 1c, TEST_CTL_U 20, STATUS 24; MODE bits OUTCTRL 0, BYPASSNL 1, RESET_N 2, VOTE_FSM_ENA 20,
//            UPDATE_BYPASS 23, LOCK_DET 31). The config sets test_ctl_hi_mask 1 with test_ctl_hi_val 0 (only
//            test_ctl_hi1_val is 1, unused by this layout): clk_alpha_pll_configure clears TEST_CTL_U bit 0; same here.
//   RCG      gpu_cc_gx_gfx3d_clk_src cmd_rcgr 0x101c (CFG +4: SRC_SEL [10:8], SRC_DIV [4:0] = 2 * div - 1), parent_map_1
//            P_GPU_CC_PLL1_OUT_AUX = 3; ftbl F(320000000, P_GPU_CC_PLL1_OUT_AUX, 2, 0, 0) -> 640 MHz / 2. clk-rcg2.c
//            update_config: CMD bit 0 UPDATE, polled until clear; CMD bit 31 ROOT_OFF.
//   Voltage  gpucc rate_max vdd_cx: [VDD_LOWER] = 320 MHz; vdd-level-bengal.h VDD_LOWER = RPM_SMD_REGULATOR_LEVEL_LOW_SVS
//            (64); bengal.dtsi gpu_cx_gdsc / gpu_gx_gdsc parent-supply VDD_CX_LEVEL = bengal-regulator.dtsi
//            rpm-regulator-smpa3 (qcom,use-voltage-level: RPM key "vlvl"). bengal-gpu.dtsi OPP 320 MHz LOW_SVS.
//            The PLL's vdd_mx VDD_MIN (<= 1 GHz) is not voted here (MX carries the DDR, above MIN_SVS while running).
//   IRQ      bengal-gpu.dtsi kgsl_3d0 interrupts <0 177 IRQ_TYPE_LEVEL_HIGH> -> INTID 209; adreno_a6xx.c A6XX_INT_MASK;
//            adreno.c adreno_irq_handler: read RBBM_INT_0_STATUS, write it to RBBM_INT_CLEAR_CMD, dispatch.
//   Clock    a6xx perfcounter CP countable 0 (PERF_CP_ALWAYS_COUNT, GPU core clock) against CP_ALWAYS_ON_COUNTER
//            (19.2 MHz): A6XX_RBBM_PERFCTR_CNTL 0x500 = 1, A6XX_CP_PERFCTR_CP_SEL_0 0x8d0 = 0, A6XX_RBBM_PERFCTR_CP_0_LO 0x400.
// First attempt never goes above the lowest OPP over XO (320 MHz) and only after the CX level vote was acked.
#ifndef GUAMP_GPU_G2_H
#define GUAMP_GPU_G2_H
#include <stddef.h>
#include <stdint.h>

enum gg2_block : unsigned { GG2_GPUCC = 0, GG2_GPU = 1 };

struct gg2_io {
  void *ctx;
  uint32_t (*rd)(void *ctx, unsigned block, uint32_t byteOff);
  void (*wr)(void *ctx, unsigned block, uint32_t byteOff, uint32_t v);
  void (*udelay)(void *ctx, unsigned us);
};

// --- GPUCC: PLL1 and the gfx3d RCG ----------------------------------------------------------------------------------
#define GG2_PLL1 0x100u
#define GG2_PLL_MODE 0x00u
#define GG2_PLL_L 0x04u
#define GG2_PLL_ALPHA 0x08u
#define GG2_PLL_ALPHA_U 0x0cu
#define GG2_PLL_USER 0x10u
#define GG2_PLL_CONFIG 0x18u
#define GG2_PLL_TEST_U 0x20u
#define GG2_PLL_STATUS 0x24u
#define GG2_MODE_OUTCTRL (1u << 0)
#define GG2_MODE_BYPASSNL (1u << 1)
#define GG2_MODE_RESET_N (1u << 2)
#define GG2_MODE_VOTE_FSM (1u << 20)
#define GG2_MODE_UPDATE_BYPASS (1u << 23)
#define GG2_MODE_LOCK_DET (1u << 31)
#define GG2_MODE_ON (GG2_MODE_OUTCTRL | GG2_MODE_BYPASSNL | GG2_MODE_RESET_N)
#define GG2_GFX_CMD 0x101cu
#define GG2_GFX_CFG 0x1020u
#define GG2_RCG_UPDATE (1u << 0)
#define GG2_RCG_ROOT_OFF (1u << 31)
#define GG2_RCG_CFG_MASK 0x71fu
#define GG2_SRC_XO 0u
#define GG2_SRC_PLL1_AUX 3u
#define GG2_SRC_PLL1_AUX2 4u
#define GG2_XO_HZ 19200000ull
#define GG2_LOCK_US 200u     // wait_for_pll_enable_lock polls 100 x udelay(1); twice that
#define GG2_RCG_US 500u      // clk-rcg2.c update_config: 500 x udelay(1)

struct gg2_pll_cfg {
  uint32_t l, alpha, alphaHi, configCtl, userMask, userVal, testUMask, testUVal;
};
// gpu_cc_pll1_config + gpu_cc_pll1_out_aux post divider 1: USER_CTL main (0) and aux (1) outputs, VCO mode 2 [21:20],
// ALPHA_EN (24), post div [17:15] = 0.
static constexpr gg2_pll_cfg kGg2Pll1 = {0x21, 0x55555555u, 0x55, 0x4001055bu,
                                         0x1u | 0x2u | (3u << 20) | (1u << 24) | (7u << 15),
                                         0x1u | 0x2u | (2u << 20) | (1u << 24), 0x1u, 0x0u};

// Output of an alpha PLL with a 40-bit alpha: XO * (L + alpha / 2^40).
static inline uint64_t gg2_pll_hz(uint32_t l, uint32_t alpha, uint32_t alphaHi) {
  const uint64_t a = (static_cast<uint64_t>(alphaHi & 0xffu) << 32) | alpha;
  return GG2_XO_HZ * l + ((GG2_XO_HZ * (a >> 8)) >> 32);
}
// RCG output for a CFG value fed by srcHz (SRC_DIV = 2 * div - 1, 0 = bypass).
static inline uint64_t gg2_rcg_hz(uint32_t cfg, uint64_t srcHz) {
  const uint32_t hid = cfg & 0x1fu;
  return hid ? srcHz * 2 / (hid + 1) : srcHz;
}
static inline uint32_t gg2_rcg_cfg(uint32_t cfg, uint32_t src, uint32_t div2) {   // div2: the divider times 2
  return (cfg & ~GG2_RCG_CFG_MASK) | ((src & 7u) << 8) | ((div2 ? div2 - 1 : 0) & 0x1fu);
}

static inline uint32_t gg2_cr(const gg2_io *io, uint32_t off) { return io->rd(io->ctx, GG2_GPUCC, off); }
static inline void gg2_cw(const gg2_io *io, uint32_t off, uint32_t v) { io->wr(io->ctx, GG2_GPUCC, off, v); }
static inline void gg2_cu(const gg2_io *io, uint32_t off, uint32_t mask, uint32_t val) {
  gg2_cw(io, off, (gg2_cr(io, off) & ~mask) | (val & mask));
}

struct gg2_pll {
  uint32_t mode0, l0, alpha0, user0, modeAfter, lAfter, userAfter, status, waitedUs;
  uint64_t hz;
  bool reconfigured, locked;
  const char *why;   // null when the PLL runs with kGg2Pll1, else the stop reason
};

// Brings PLL1 to the 640 MHz configuration and turns its outputs on (clk_alpha_pll_configure + clk_alpha_pll_enable).
// Refuses when the PLL is in hardware FSM vote mode or when the gfx3d RCG currently selects one of its outputs.
static inline bool gg2_pll1_up(const gg2_io *io, const gg2_pll_cfg *c, gg2_pll *p) {
  *p = {};
  const uint32_t base = GG2_PLL1;
  p->mode0 = gg2_cr(io, base + GG2_PLL_MODE);
  p->l0 = gg2_cr(io, base + GG2_PLL_L);
  p->alpha0 = gg2_cr(io, base + GG2_PLL_ALPHA);
  p->user0 = gg2_cr(io, base + GG2_PLL_USER);
  const uint32_t src = (gg2_cr(io, GG2_GFX_CFG) >> 8) & 7u;
  if (p->mode0 & GG2_MODE_VOTE_FSM) p->why = "pll1 in FSM vote mode";
  else if (src == GG2_SRC_PLL1_AUX || src == GG2_SRC_PLL1_AUX2) p->why = "gfx3d rcg already on pll1";
  if (p->why) return false;
  const bool on = (p->mode0 & GG2_MODE_ON) == GG2_MODE_ON && (p->mode0 & GG2_MODE_LOCK_DET);
  const bool same = p->l0 == c->l && p->alpha0 == c->alpha && (p->user0 & c->userMask) == c->userVal;
  if (!(on && same)) {
    if (p->mode0 & (GG2_MODE_ON)) {   // clk_alpha_pll_disable: outputs off, then bypass and reset
      gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_OUTCTRL, 0);
      io->udelay(io->ctx, 1);
      gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_BYPASSNL | GG2_MODE_RESET_N, 0);
    }
    gg2_cw(io, base + GG2_PLL_L, c->l);
    gg2_cw(io, base + GG2_PLL_ALPHA, c->alpha);
    gg2_cw(io, base + GG2_PLL_CONFIG, c->configCtl);
    gg2_cw(io, base + GG2_PLL_ALPHA_U, c->alphaHi);
    gg2_cu(io, base + GG2_PLL_USER, c->userMask, c->userVal);
    gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_UPDATE_BYPASS, GG2_MODE_UPDATE_BYPASS);   // SUPPORTS_DYNAMIC_UPDATE
    gg2_cu(io, base + GG2_PLL_TEST_U, c->testUMask, c->testUVal);
    gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_UPDATE_BYPASS, GG2_MODE_UPDATE_BYPASS);
    gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_BYPASSNL, GG2_MODE_BYPASSNL);
    io->udelay(io->ctx, 5);   // 5 us between bypass off and reset release
    gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_RESET_N, GG2_MODE_RESET_N);
    p->reconfigured = true;
  }
  for (;;) {
    if (gg2_cr(io, base + GG2_PLL_MODE) & GG2_MODE_LOCK_DET) {
      p->locked = true;
      break;
    }
    if (p->waitedUs >= GG2_LOCK_US) break;
    io->udelay(io->ctx, 1);
    p->waitedUs = p->waitedUs + 1;
  }
  if (p->locked) gg2_cu(io, base + GG2_PLL_MODE, GG2_MODE_OUTCTRL, GG2_MODE_OUTCTRL);
  else p->why = "pll1 no lock";
  p->modeAfter = gg2_cr(io, base + GG2_PLL_MODE);
  p->lAfter = gg2_cr(io, base + GG2_PLL_L);
  p->userAfter = gg2_cr(io, base + GG2_PLL_USER);
  p->status = gg2_cr(io, base + GG2_PLL_STATUS);
  p->hz = gg2_pll_hz(p->lAfter, gg2_cr(io, base + GG2_PLL_ALPHA), gg2_cr(io, base + GG2_PLL_ALPHA_U));
  if (p->locked && !(p->modeAfter & GG2_MODE_OUTCTRL)) p->why = "pll1 outputs did not stay on";
  return !p->why;
}

// Turns PLL1 off again (clk_alpha_pll_disable); only valid with the RCG back on XO.
static inline void gg2_pll1_down(const gg2_io *io) {
  gg2_cu(io, GG2_PLL1 + GG2_PLL_MODE, GG2_MODE_OUTCTRL, 0);
  io->udelay(io->ctx, 1);
  gg2_cu(io, GG2_PLL1 + GG2_PLL_MODE, GG2_MODE_BYPASSNL | GG2_MODE_RESET_N, 0);
}

struct gg2_rcg { uint32_t cmd0, cfg0, cmd, cfg, waitedUs; bool ok; };
// Selects src / divider on the gfx3d RCG (clk-rcg2.c configure + update_config).
static inline bool gg2_gfx_set(const gg2_io *io, uint32_t src, uint32_t div2, gg2_rcg *r) {
  *r = {};
  r->cmd0 = gg2_cr(io, GG2_GFX_CMD);
  r->cfg0 = gg2_cr(io, GG2_GFX_CFG);
  gg2_cw(io, GG2_GFX_CFG, gg2_rcg_cfg(r->cfg0, src, div2));
  gg2_cw(io, GG2_GFX_CMD, gg2_cr(io, GG2_GFX_CMD) | GG2_RCG_UPDATE);
  for (;;) {
    r->cmd = gg2_cr(io, GG2_GFX_CMD);
    if (!(r->cmd & GG2_RCG_UPDATE)) break;
    if (r->waitedUs >= GG2_RCG_US) break;
    io->udelay(io->ctx, 1);
    r->waitedUs = r->waitedUs + 1;
  }
  r->cfg = gg2_cr(io, GG2_GFX_CFG);
  r->ok = !(r->cmd & GG2_RCG_UPDATE) && !(r->cmd & GG2_RCG_ROOT_OFF) &&
          (r->cfg & GG2_RCG_CFG_MASK) == (gg2_rcg_cfg(r->cfg0, src, div2) & GG2_RCG_CFG_MASK);
  return r->ok;
}

// --- GPU core clock measurement ------------------------------------------------------------------------------------
#define GG2_RBBM_PERFCTR_CNTL 0x500u
#define GG2_CP_PERFCTR_CP_SEL_0 0x8d0u
#define GG2_RBBM_PERFCTR_CP_0_LO 0x400u
#define GG2_CP_ALWAYS_ON_COUNTER_LO 0x980u
static inline uint32_t gg2_gr(const gg2_io *io, uint32_t reg) { return io->rd(io->ctx, GG2_GPU, reg * 4u); }
static inline void gg2_gw(const gg2_io *io, uint32_t reg, uint32_t v) { io->wr(io->ctx, GG2_GPU, reg * 4u, v); }

// GPU clock in Hz from GPU cycles (CP countable 0) against the 19.2 MHz always-on counter over windowUs; 0 when the
// cycle counter did not move.
static inline uint64_t gg2_measure_hz(const gg2_io *io, unsigned windowUs, uint32_t *cycles, uint32_t *aonTicks) {
  gg2_gw(io, GG2_RBBM_PERFCTR_CNTL, 1);
  gg2_gw(io, GG2_CP_PERFCTR_CP_SEL_0, 0);
  io->udelay(io->ctx, 2);
  const uint32_t c0 = gg2_gr(io, GG2_RBBM_PERFCTR_CP_0_LO), a0 = gg2_gr(io, GG2_CP_ALWAYS_ON_COUNTER_LO);
  io->udelay(io->ctx, windowUs);
  const uint32_t c1 = gg2_gr(io, GG2_RBBM_PERFCTR_CP_0_LO), a1 = gg2_gr(io, GG2_CP_ALWAYS_ON_COUNTER_LO);
  *cycles = c1 - c0;
  *aonTicks = a1 - a0;
  return *aonTicks ? static_cast<uint64_t>(*cycles) * GG2_XO_HZ / *aonTicks : 0;
}

// --- GPU interrupt -------------------------------------------------------------------------------------------------
#define GG2_GPU_INTID (32u + 177u)
#define GG2_RBBM_INT_CLEAR_CMD 0x37u
#define GG2_RBBM_INT_0_MASK 0x38u
#define GG2_RBBM_INT_0_STATUS 0x201u
#define GG2_INT_CP_AHB_ERROR 1u
#define GG2_INT_ATB_ASYNCFIFO_OVERFLOW 6u
#define GG2_INT_RBBM_GPC_ERROR 7u
#define GG2_INT_CP_SW 8u
#define GG2_INT_CP_HW_ERROR 9u
#define GG2_INT_CP_IB2 13u
#define GG2_INT_CP_IB1 14u
#define GG2_INT_CP_RB 15u
#define GG2_INT_CP_CACHE_FLUSH_TS 20u
#define GG2_INT_RBBM_ATB_BUS_OVERFLOW 22u
#define GG2_INT_RBBM_HANG_DETECT 23u
#define GG2_INT_UCHE_OOB_ACCESS 24u
#define GG2_INT_UCHE_TRAP_INTR 25u
#define GG2_INT_TSB_WRITE_ERROR 28u
#define GG2_INT_MASK                                                                                               \
  ((1u << GG2_INT_CP_AHB_ERROR) | (1u << GG2_INT_ATB_ASYNCFIFO_OVERFLOW) | (1u << GG2_INT_RBBM_GPC_ERROR) |       \
   (1u << GG2_INT_CP_SW) | (1u << GG2_INT_CP_HW_ERROR) | (1u << GG2_INT_CP_IB2) | (1u << GG2_INT_CP_IB1) |         \
   (1u << GG2_INT_CP_RB) | (1u << GG2_INT_CP_CACHE_FLUSH_TS) | (1u << GG2_INT_RBBM_ATB_BUS_OVERFLOW) |              \
   (1u << GG2_INT_RBBM_HANG_DETECT) | (1u << GG2_INT_UCHE_OOB_ACCESS) | (1u << GG2_INT_UCHE_TRAP_INTR) |           \
   (1u << GG2_INT_TSB_WRITE_ERROR))
#define GG2_IRQ_STORM 100000u   // interrupts before the handler masks everything (a level line that never clears)

struct gg2_irq_stats {
  volatile uint32_t total, spurious, fence, cpRb, hwError, ahbError, atb, hang, uche, gpc, other, last, masked;
};
// Counts one RBBM_INT_0_STATUS value (adreno_irq_handler dispatch, counters only).
static inline void gg2_irq_account(gg2_irq_stats *s, uint32_t status) {
  s->total = s->total + 1;
  if (!status) {
    s->spurious = s->spurious + 1;
    return;
  }
  s->last = status;
  if (status & (1u << GG2_INT_CP_CACHE_FLUSH_TS)) s->fence = s->fence + 1;
  if (status & ((1u << GG2_INT_CP_RB) | (1u << GG2_INT_CP_IB1) | (1u << GG2_INT_CP_IB2) | (1u << GG2_INT_CP_SW)))
    s->cpRb = s->cpRb + 1;
  if (status & (1u << GG2_INT_CP_HW_ERROR)) s->hwError = s->hwError + 1;
  if (status & (1u << GG2_INT_CP_AHB_ERROR)) s->ahbError = s->ahbError + 1;
  if (status & ((1u << GG2_INT_ATB_ASYNCFIFO_OVERFLOW) | (1u << GG2_INT_RBBM_ATB_BUS_OVERFLOW))) s->atb = s->atb + 1;
  if (status & (1u << GG2_INT_RBBM_HANG_DETECT)) s->hang = s->hang + 1;
  if (status & ((1u << GG2_INT_UCHE_OOB_ACCESS) | (1u << GG2_INT_UCHE_TRAP_INTR))) s->uche = s->uche + 1;
  if (status & (1u << GG2_INT_RBBM_GPC_ERROR)) s->gpc = s->gpc + 1;
  if (status & ~GG2_INT_MASK) s->other = s->other + 1;
}
// The handler body: read, acknowledge, count; masks everything after GG2_IRQ_STORM interrupts.
static inline void gg2_irq_service(const gg2_io *io, gg2_irq_stats *s) {
  const uint32_t status = gg2_gr(io, GG2_RBBM_INT_0_STATUS);
  if (status) gg2_gw(io, GG2_RBBM_INT_CLEAR_CMD, status);
  gg2_irq_account(s, status);
  if (s->total >= GG2_IRQ_STORM && !s->masked) {
    gg2_gw(io, GG2_RBBM_INT_0_MASK, 0);
    s->masked = 1;
  }
}
#endif
