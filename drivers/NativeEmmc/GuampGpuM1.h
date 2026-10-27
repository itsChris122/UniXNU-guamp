// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp (Moto G9 Play, SM6115 / bengal) Adreno 610 milestone M1: power-up sequencing and the first GPU register reads.
// Pure code: every register access goes through gm_io (GpuService.inc maps the blocks, tests/gpu_host.cpp is a fake
// register file). No TrustZone call, no firmware, no command submission, no GPU write at all, no RBBM_SW_RESET (A610
// reset line erratum, Linux a6xx_gpu.c). Every constant states its source in the vendor tree
// android_kernel_motorola_sm6225 (gcc-bengal.c, gpucc-bengal.c, gdsc-regulator.c, clk-branch.c, clk-rcg2.c,
// clk-alpha-pll.c, bengal.dtsi, bengal-gdsc.dtsi, bengal-gpu.dtsi, msm-arm-smmu-bengal.dtsi, arm-smmu.c,
// kgsl_pwrctrl.c, a6xx_reg.h).
//
// Sequence (gm_power_up), the Linux order: gpucc probe, then arm_smmu_power_on (kgsl_smmu: vdd = gpu_cx_gdsc, then its
// clocks), then kgsl_pwrctrl_enable (pwrrail: vddcx, vdd = gpu_gx_gdsc; then kgsl_pwrctrl_clk, clocks[] walked from
// the end). Each wait bounded; branch waits follow clk_branch_wait (see gm_branch_enable).
//   GCC     gpu_cfg_ahb (CLK_IS_CRITICAL, the GPUCC/GPU config interface: gate for any GPUCC access), vote
//           gcc_gpu_gpll0_clk_src (GPLL0 main to GPUCC mux input 5)
//   CX      GDSC 0x106c: clear SW_COLLAPSE, poll PWR_ON in GDS_HW_CTRL 0x1540 (hw-ctrl-addr, as poll_gdsc_status)
//   SMMU    gcc gpu_memnoc_gfx, gcc gpu_snoc_dvm_gfx, gpu_cc_ahb, gpu_cc_hlos1_vote_gpu_smmu: only now, with CX on (the
//           GCC GPU branches report CLK_OFF until the GPU CX domain is powered: on the phone memnoc stayed CLK_OFF
//           when it was enabled before the GDSC)
//   GX      GDSC 0x100c (parent CX): only when it is off, BLK_ARES pulse in GPU_GX_BCR 0x1008 and CLAMP_IO cleared in
//           0x1508 (gdsc_enable toggle_logic order), clear SW_COLLAPSE, poll PWR_ON in GDSCR
//   kgsl    gx_gfx3d_clk_src on XO (19.2 MHz, no PLL programming), gmu_clk_src at 200 MHz (GPLL0 / 3, only when GPLL0
//           is active; otherwise the boot source is kept), then cx_gmu, iref, cxo, bimc_gpu_axi, gx_gfx3d, cx_gfx3d
//   A branch with a status check (BRANCH_HALT / BRANCH_VOTED) that does not come on is not fatal at once: the sequence
//   goes on (no GPU access yet), re-polls it at the end (reported "late") and stops there when it is still off.
//   gate    both GDSCs PWR_ON and every register-path branch on, re-read right before; only then gm_io::map_gpu and the
//           reads VBIF_VERSION, RBBM_STATUS, CP_ALWAYS_ON_COUNTER_LO twice. Branches Linux never checks
//           (BRANCH_HALT_DELAY / BRANCH_HALT_SKIP) must still show CLK_OFF clear unless gm_state::relaxed (then: enable
//           bit set is enough).
// gm_power_down undoes exactly what gm_power_up changed, in reverse order (kgsl clocks, GX collapse + clamp, SMMU
// clocks, CX collapse, votes).
#ifndef GUAMP_GPU_M1_H
#define GUAMP_GPU_M1_H
#include <stdint.h>
#include <stddef.h>

// --- blocks (bengal.dtsi gcc 0x1400000 len 0x1f0000; gpucc 0x5990000 len 0x9000; bengal-gpu.dtsi kgsl-3d0
// 0x5900000 len 0x90000) ------------------------------------------------------------------------------------------
enum gm_block : unsigned { GM_GCC = 0, GM_GPUCC = 1, GM_GPU = 2 };
#define GM_GCC_PHYS 0x01400000ULL
#define GM_GCC_SIZE 0x80000ULL              // up to 0x7900c, the second branch vote register
#define GM_GPUCC_PHYS 0x05990000ULL
#define GM_GPUCC_SIZE 0x9000ULL
#define GM_GPU_PHYS 0x05900000ULL
#define GM_GPU_SIZE 0x90000ULL

// --- GCC (gcc-bengal.c) -------------------------------------------------------------------------------------------
#define GM_GCC_GPLL0_MODE 0x0               // gpll0 .offset 0, PLL_MODE (clk-alpha-pll.c); vote 0x79000 BIT(0)
#define GM_GCC_GPU_CFG_AHB 0x36004          // BRANCH_HALT, hwcg_reg 0x36004, CLK_IS_CRITICAL
#define GM_GCC_GPU_MEMNOC_GFX 0x3600c       // BRANCH_VOTED (polled on enable), hwcg_reg 0x3600c
#define GM_GCC_GPU_SNOC_DVM_GFX 0x36018     // BRANCH_HALT
#define GM_GCC_QMIP_GPU_CFG_AHB 0x36040     // BRANCH_HALT_VOTED, enable 0x7900c BIT(4): not used by Linux on bengal, logged
#define GM_GCC_GPU_THROTTLE_CORE 0x36048    // BRANCH_HALT_VOTED, enable 0x79004 BIT(31): not used by kgsl, logged
#define GM_GCC_GPU_IREF 0x36100             // BRANCH_HALT_DELAY
#define GM_GCC_BIMC_GPU_AXI 0x71154         // BRANCH_HALT_DELAY, hwcg_reg 0x71154
#define GM_GCC_BRANCH_VOTE 0x79004          // gcc_gpu_gpll0_clk_src BIT(15) (parent gpll0); BIT(16) is
#define GM_VOTE_GPU_GPLL0 (1u << 15)        // gcc_gpu_gpll0_div_clk_src (gpll0_out_aux2, GPUCC input 6): not used
#define GM_GCC_BRANCH_VOTE2 0x7900c         // read only here (gcc_qmip_gpu_cfg_ahb BIT(4))
#define GM_PLL_ACTIVE (1u << 30)            // PLL_ACTIVE_FLAG
#define GM_PLL_LOCK_DET (1u << 31)          // PLL_LOCK_DET

// --- GPUCC (gpucc-bengal.c, bengal-gdsc.dtsi) ----------------------------------------------------------------------
#define GM_CC_GX_BCR 0x1008                 // gpu_gx_sw_reset syscon@5991008
#define GM_CC_GX_GDSCR 0x100c               // gpu_gx_gdsc qcom,gdsc@599100c
#define GM_CC_GFX3D_RCG 0x101c              // gpu_cc_gx_gfx3d_clk_src cmd_rcgr, hid 5, mnd 0, parent_map_1
#define GM_CC_GX_GFX3D 0x1054               // gpu_cc_gx_gfx3d_clk, BRANCH_HALT_SKIP
#define GM_CC_CX_GDSCR 0x106c               // gpu_cx_gdsc qcom,gdsc@599106c, gds-timeout 500
#define GM_CC_AHB 0x1078                    // gpu_cc_ahb_clk, BRANCH_HALT_DELAY, CLK_IS_CRITICAL
#define GM_CC_CX_GMU 0x1098                 // gpu_cc_cx_gmu_clk, BRANCH_HALT, parent gpu_cc_gmu_clk_src
#define GM_CC_CXO 0x109c                    // gpu_cc_cxo_clk (rbbmtimer_clk), BRANCH_HALT
#define GM_CC_CX_GFX3D 0x10a4               // gpu_cc_cx_gfx3d_clk, BRANCH_HALT_DELAY, parent gx_gfx3d_clk_src
#define GM_CC_GMU_RCG 0x1120                // gpu_cc_gmu_clk_src cmd_rcgr, hid 5, mnd 0, parent_map_0
#define GM_CC_GX_CLAMP 0x1508               // gpu_gx_domain_addr syscon@5991508
#define GM_CC_CX_HW_CTRL 0x1540             // gpu_cx_hw_ctrl syscon@5991540 (polled for CX PWR_ON)
#define GM_CC_SMMU_VOTE 0x5000              // gpu_cc_hlos1_vote_gpu_smmu_clk, BRANCH_VOTED (halt_reg = enable_reg)

// gdsc-regulator.c
#define GM_GDSC_SW_COLLAPSE (1u << 0)
#define GM_GDSC_HW_CONTROL (1u << 1)        // set: Linux refuses the enable ("under HW control")
#define GM_GDSC_PWR_ON (1u << 31)
#define GM_CLAMP_IO (1u << 0)               // GMEM_CLAMP_IO_MASK
#define GM_BLK_ARES (1u << 0)               // BCR_BLK_ARES_BIT
// clk-branch.c / clk-branch.h
#define GM_CBCR_EN (1u << 0)
#define GM_CBCR_HWCG (1u << 1)              // hwcg_bit 1 (HW_CTL): clock in hardware gating mode, halt not checked
#define GM_CBCR_FSM_MASK (7u << 28)
#define GM_CBCR_FSM_ON (2u << 28)           // BRANCH_NOC_FSM_STATUS_ON
#define GM_CBCR_CLK_OFF (1u << 31)
#define GM_BRANCH_DELAY_US 10               // clk_branch_wait: BRANCH_HALT_DELAY udelay(10)
// clk-rcg2.c
#define GM_RCG_CMD 0x0
#define GM_RCG_CFG 0x4
#define GM_RCG_UPDATE (1u << 0)
#define GM_RCG_ROOT_OFF (1u << 31)
#define GM_RCG_DIV_MASK 0x1fu               // hid_width 5, field = 2 * divider - 1 (F() macro)
#define GM_RCG_SEL_MASK (7u << 8)
#define GM_RCG_MODE_MASK (3u << 12)
#define GM_RCG_MUX_MASK (GM_RCG_DIV_MASK | GM_RCG_SEL_MASK | GM_RCG_MODE_MASK)
// ftbl_gpu_cc_gmu_clk_src F(200000000, P_GPLL0_OUT_MAIN, 3, 0, 0): parent_map_0 select 5, field 5 (GPLL0 600 MHz)
#define GM_GMU_200M_CFG ((5u << 8) | 5u)
// gx_gfx3d_clk_src on P_BI_TCXO (parent_map_1 select 0), divider 1 (field 1; 0 is bypass, also XO)
#define GM_GFX3D_XO_CFG 1u

// --- GPU (a6xx_reg.h, dword offsets) -------------------------------------------------------------------------------
#define GM_A6XX_VBIF_VERSION 0x3000
#define GM_A6XX_RBBM_STATUS 0x210
#define GM_A6XX_CP_ALWAYS_ON_COUNTER_LO 0x980

#define GM_GDSC_POLL_US 1000                // Linux: 500 (CX gds-timeout) / 100 (GX default) one-microsecond polls
#define GM_BRANCH_POLL_US 500               // Linux: 200
#define GM_RCG_POLL_US 500                  // update_config: 500
#define GM_AON_GAP_US 100                   // ~1920 counts at 19.2 MHz

struct gm_io {
  void *ctx;
  uint32_t (*rd)(void *ctx, unsigned block, uint32_t off);
  void (*wr)(void *ctx, unsigned block, uint32_t off, uint32_t v);   // GCC and GPUCC only
  void (*udelay)(void *ctx, unsigned us);
  bool (*map_gpu)(void *ctx);   // called at most once, only after every gate passed; false = no GPU access
};

// Every register gm_power_up may write (or read for a decision or the log), in log order: the before/after snapshot.
enum gm_reg : unsigned {
  GMR_VOTE, GMR_PLL0, GMR_CFG_AHB, GMR_MEMNOC, GMR_SNOC_DVM, GMR_IREF, GMR_AXI, GMR_THROTTLE, GMR_QMIP, GMR_VOTE2,   // GCC
  GMR_CX_GDSCR, GMR_CX_HWCTRL, GMR_AHB, GMR_CXO, GMR_GMU_CMD, GMR_GMU_CFG, GMR_CX_GMU, GMR_SMMU,     // GPUCC CX
  GMR_GX_BCR, GMR_GX_CLAMP, GMR_GX_GDSCR, GMR_GFX_CMD, GMR_GFX_CFG, GMR_GX_GFX3D, GMR_CX_GFX3D,     // GPUCC GX
  GMR_COUNT
};
static_assert(GMR_COUNT <= 32, "gm_state bit masks are 32 bits wide");
#define GMR_GCC_END GMR_CX_GDSCR            // [0, GMR_GCC_END): GCC, readable at any time
struct gm_reg_desc { const char *name; unsigned block; uint32_t off; bool ro; };   // ro: never written
static const gm_reg_desc kGmRegs[GMR_COUNT] = {
    {"vote", GM_GCC, GM_GCC_BRANCH_VOTE, false}, {"pll0", GM_GCC, GM_GCC_GPLL0_MODE, true},
    {"cfg_ahb", GM_GCC, GM_GCC_GPU_CFG_AHB, false}, {"memnoc", GM_GCC, GM_GCC_GPU_MEMNOC_GFX, false},
    {"snoc_dvm", GM_GCC, GM_GCC_GPU_SNOC_DVM_GFX, false}, {"iref", GM_GCC, GM_GCC_GPU_IREF, false},
    {"axi", GM_GCC, GM_GCC_BIMC_GPU_AXI, false}, {"throttle", GM_GCC, GM_GCC_GPU_THROTTLE_CORE, true},
    {"qmip", GM_GCC, GM_GCC_QMIP_GPU_CFG_AHB, true}, {"vote2", GM_GCC, GM_GCC_BRANCH_VOTE2, true},
    {"cx_gdscr", GM_GPUCC, GM_CC_CX_GDSCR, false}, {"cx_hwctrl", GM_GPUCC, GM_CC_CX_HW_CTRL, true},
    {"ahb", GM_GPUCC, GM_CC_AHB, false}, {"cxo", GM_GPUCC, GM_CC_CXO, false},
    {"gmu_cmd", GM_GPUCC, GM_CC_GMU_RCG + GM_RCG_CMD, false}, {"gmu_cfg", GM_GPUCC, GM_CC_GMU_RCG + GM_RCG_CFG, false},
    {"cx_gmu", GM_GPUCC, GM_CC_CX_GMU, false}, {"smmu", GM_GPUCC, GM_CC_SMMU_VOTE, false},
    {"gx_bcr", GM_GPUCC, GM_CC_GX_BCR, false}, {"gx_clamp", GM_GPUCC, GM_CC_GX_CLAMP, false},
    {"gx_gdscr", GM_GPUCC, GM_CC_GX_GDSCR, false}, {"gfx_cmd", GM_GPUCC, GM_CC_GFX3D_RCG + GM_RCG_CMD, false},
    {"gfx_cfg", GM_GPUCC, GM_CC_GFX3D_RCG + GM_RCG_CFG, false}, {"gx_gfx3d", GM_GPUCC, GM_CC_GX_GFX3D, false},
    {"cx_gfx3d", GM_GPUCC, GM_CC_CX_GFX3D, false},
};

// clk-branch.h halt_check values, as Linux handles them when enabling (clk_branch_wait)
enum gm_halt : unsigned char {
  GM_HALT,          // BRANCH_HALT: poll CLK_OFF clear (or NOC FSM ON)
  GM_HALT_VOTED,    // BRANCH_VOTED / BRANCH_HALT_VOTED: same poll when enabling
  GM_HALT_DELAY,    // BRANCH_HALT_DELAY: no status bit, 10 us delay
  GM_HALT_SKIP,     // BRANCH_HALT_SKIP: nothing
};
static inline const char *gm_halt_name(gm_halt h) {
  switch (h) {
    case GM_HALT: return "HALT";
    case GM_HALT_VOTED: return "VOTED";
    case GM_HALT_DELAY: return "DELAY";
    default: return "SKIP";
  }
}
// Enable phase: GCC (gpucc probe time), SMMU (arm_smmu_power_on after gpu_cx_gdsc), KGSL (kgsl_pwrctrl_clk after
// both GDSCs).
enum gm_phase : unsigned char { GM_PH_GCC, GM_PH_SMMU, GM_PH_KGSL };
// gate: must be on before any GPU register read; axi and iref (BRANCH_HALT_DELAY, master side) are enabled and
// reported only. Every enable bit here is BIT(0) of the halt register itself.
struct gm_branch { unsigned reg; gm_halt halt; bool hwcg, gate; gm_phase phase; };
static const gm_branch kGmBranches[] = {
    {GMR_CFG_AHB, GM_HALT, true, true, GM_PH_GCC},
    // msm-arm-smmu-bengal.dtsi kgsl_smmu clocks, in order
    {GMR_MEMNOC, GM_HALT_VOTED, true, true, GM_PH_SMMU}, {GMR_SNOC_DVM, GM_HALT, false, true, GM_PH_SMMU},
    {GMR_AHB, GM_HALT_DELAY, false, true, GM_PH_SMMU}, {GMR_SMMU, GM_HALT_VOTED, false, true, GM_PH_SMMU},
    // kgsl_pwrctrl_clk: clocks[] from the end (gmu_clk, iref_clk, rbbmtimer_clk, mem_clk = memnoc (already on),
    // iface_clk = bimc_gpu_axi, core_clk), then gpu_cc_cx_gfx3d: child of the core RCG, not in the kgsl list of
    // bengal-gpu.dtsi, so report only (its enable bit reads back 0 here, CLK_OFF clear)
    {GMR_CX_GMU, GM_HALT, false, true, GM_PH_KGSL}, {GMR_IREF, GM_HALT_DELAY, false, false, GM_PH_KGSL},
    {GMR_CXO, GM_HALT, false, true, GM_PH_KGSL}, {GMR_AXI, GM_HALT_DELAY, true, false, GM_PH_KGSL},
    {GMR_GX_GFX3D, GM_HALT_SKIP, false, true, GM_PH_KGSL}, {GMR_CX_GFX3D, GM_HALT_DELAY, false, false, GM_PH_KGSL},
};
#define GM_NBRANCH (sizeof kGmBranches / sizeof kGmBranches[0])

struct gm_state {
  uint32_t before[GMR_COUNT], after[GMR_COUNT], off[GMR_COUNT];
  bool haveBefore[GMR_COUNT], haveAfter, haveOff;
  bool relaxed;              // set by the caller: BRANCH_HALT_DELAY / SKIP branches gate on the enable bit only
  const char *stop;          // null = every step passed; otherwise the first failure, no GPU access happened
  const char *cx, *gx, *clocks, *gmu, *gfx;   // words for the RESULT line
  bool gpuccOk;              // gcc cfg_ahb seen on: GPUCC may be accessed (snapshots, power-down)
  bool gpuRead, aonOk, mapped;
  uint32_t vbif, status, aon0, aon1;
  bool cxTouched, gxTouched, gxReset;          // power_down undoes only these
  uint32_t enabled;                            // bit per gm_reg: branch enable bit (or vote) set by us
  uint32_t stuck;                              // bit per gm_reg: status check timed out at enable
  uint32_t late;                               // bit per gm_reg: stuck at enable, on at the final re-poll
};

static inline uint32_t gm_rd(const gm_io *io, unsigned r) { return io->rd(io->ctx, kGmRegs[r].block, kGmRegs[r].off); }
static inline void gm_wr(const gm_io *io, unsigned r, uint32_t v) { io->wr(io->ctx, kGmRegs[r].block, kGmRegs[r].off, v); }

// clk_branch2_check_halt (enabling) plus clk_branch_in_hwcg_mode (enable bit required here as well)
static inline bool gm_branch_is_on(uint32_t v, bool hwcg) {
  if (hwcg && (v & GM_CBCR_HWCG) && (v & GM_CBCR_EN)) return true;
  return !(v & GM_CBCR_CLK_OFF) || (v & (GM_CBCR_CLK_OFF | GM_CBCR_FSM_MASK)) == GM_CBCR_FSM_ON;
}
static inline bool gm_pll_active(uint32_t mode) { return (mode & GM_PLL_LOCK_DET) || (mode & GM_PLL_ACTIVE); }

// What one CBCR value means for a branch: the per-branch log word and the gate decision.
enum gm_bstate : unsigned char {
  GM_BS_ON,          // enable bit set, status on (CLK_OFF clear / NOC FSM ON)
  GM_BS_HWCG,        // enable bit and HW_CTL set: hardware gated, Linux checks nothing
  GM_BS_DISABLED,    // enable bit clear
  GM_BS_CLK_OFF,     // enabled, checked branch (HALT / VOTED), CLK_OFF set
  GM_BS_UNCHECKED,   // enabled, DELAY / SKIP branch, CLK_OFF set (Linux would not look)
};
static inline gm_bstate gm_branch_state(uint32_t v, const gm_branch &b) {
  if (!(v & GM_CBCR_EN)) return GM_BS_DISABLED;
  if (b.hwcg && (v & GM_CBCR_HWCG)) return GM_BS_HWCG;
  if (gm_branch_is_on(v, false)) return GM_BS_ON;
  return b.halt == GM_HALT || b.halt == GM_HALT_VOTED ? GM_BS_CLK_OFF : GM_BS_UNCHECKED;
}
static inline const char *gm_bstate_name(gm_bstate s) {
  switch (s) {
    case GM_BS_ON: return "on";
    case GM_BS_HWCG: return "hwcg";
    case GM_BS_DISABLED: return "disabled";
    case GM_BS_CLK_OFF: return "CLK_OFF";
    default: return "CLK_OFF(unchecked)";
  }
}
static inline bool gm_bstate_ok(gm_bstate s, bool relaxed) {
  return s == GM_BS_ON || s == GM_BS_HWCG || (relaxed && s == GM_BS_UNCHECKED);
}

static inline bool gm_poll(const gm_io *io, unsigned r, uint32_t mask, bool wantSet, unsigned us) {
  for (unsigned n = 0;; n = n + 1) {
    const uint32_t v = gm_rd(io, r);
    if (((v & mask) != 0) == wantSet) return true;
    if (n >= us) return false;
    io->udelay(io->ctx, 1);
  }
}

// Status wait of a checked branch: true when on (or hardware gated) within GM_BRANCH_POLL_US.
static inline bool gm_branch_wait(const gm_io *io, const gm_branch &b) {
  for (unsigned n = 0;; n = n + 1) {
    const gm_bstate s = gm_branch_state(gm_rd(io, b.reg), b);
    if (s == GM_BS_ON || s == GM_BS_HWCG) return true;
    if (n >= GM_BRANCH_POLL_US) return false;
    io->udelay(io->ctx, 1);
  }
}

// clk_branch_toggle(en) + clk_branch_wait: set the enable bit, then by halt type: HALT / VOTED poll the status (none
// in hardware gating mode), DELAY waits 10 us, SKIP nothing. False only for a checked branch that stays off.
static inline bool gm_branch_enable(const gm_io *io, gm_state *st, const gm_branch &b) {
  const uint32_t v = gm_rd(io, b.reg);
  if (!(v & GM_CBCR_EN)) {
    gm_wr(io, b.reg, v | GM_CBCR_EN);
    st->enabled |= 1u << b.reg;
  }
  switch (b.halt) {
    case GM_HALT_SKIP: return true;
    case GM_HALT_DELAY: io->udelay(io->ctx, GM_BRANCH_DELAY_US); return true;
    default: return gm_branch_wait(io, b);
  }
}

// Every branch of one phase, in table order. A checked branch that stays off is recorded in st->stuck (re-polled at
// the end of gm_power_up) and does not stop the phase.
static inline void gm_enable_phase(const gm_io *io, gm_state *st, gm_phase ph) {
  for (const gm_branch &b : kGmBranches)
    if (b.phase == ph && !gm_branch_enable(io, st, b)) st->stuck |= 1u << b.reg;
}

// RCG mux/divider change (clk_rcg2 configure + update_config); unchanged when the mux already selects the source
// (for XO either divider encoding). Returns false when CMD_UPDATE does not clear.
static inline bool gm_rcg_set(const gm_io *io, unsigned cmdReg, unsigned cfgReg, uint32_t want, bool *changed) {
  *changed = false;
  const uint32_t cfg = gm_rd(io, cfgReg);
  const uint32_t mux = cfg & GM_RCG_MUX_MASK;
  if (mux == want || (want == GM_GFX3D_XO_CFG && mux == 0)) return true;
  gm_wr(io, cfgReg, (cfg & ~GM_RCG_MUX_MASK) | want);
  gm_wr(io, cmdReg, gm_rd(io, cmdReg) | GM_RCG_UPDATE);
  *changed = true;
  return gm_poll(io, cmdReg, GM_RCG_UPDATE, false, GM_RCG_POLL_US);
}

static inline bool gm_cx_on(const gm_io *io) { return (gm_rd(io, GMR_CX_HWCTRL) & GM_GDSC_PWR_ON) != 0; }
static inline bool gm_gx_on(const gm_io *io) { return (gm_rd(io, GMR_GX_GDSCR) & GM_GDSC_PWR_ON) != 0; }

static inline void gm_snap(const gm_io *io, uint32_t *out, unsigned from, unsigned to) {
  for (unsigned r = from; r < to; r = r + 1) out[r] = gm_rd(io, r);
}

// The final gate, re-read from the hardware: returns null when the GPU block may be read, else what is off. cfg_ahb
// first: no GPUCC read without it.
static inline const char *gm_gate(const gm_io *io, bool relaxed) {
  if (!gm_bstate_ok(gm_branch_state(gm_rd(io, GMR_CFG_AHB), kGmBranches[0]), false)) return "cfg_ahb";
  if (!gm_cx_on(io)) return "cx";
  if (!gm_gx_on(io)) return "gx";
  for (const gm_branch &b : kGmBranches)
    if (b.gate && !gm_bstate_ok(gm_branch_state(gm_rd(io, b.reg), b), relaxed)) return kGmRegs[b.reg].name;
  return nullptr;
}

static inline void gm_init(gm_state *st) {
  *st = gm_state{};
  st->cx = st->gx = st->clocks = st->gmu = st->gfx = "-";
}

// Power-up. Returns false (st->stop set) at the first failure; the GPU block is then never touched.
static inline bool gm_power_up(const gm_io *io, gm_state *st) {
  // GCC: always readable. Snapshot, then cfg_ahb (critical, normally on since boot): it gates every GPUCC access.
  gm_snap(io, st->before, 0, GMR_GCC_END);
  for (unsigned r = 0; r < GMR_GCC_END; r = r + 1) st->haveBefore[r] = true;
  st->clocks = "off";
  if (!gm_branch_enable(io, st, kGmBranches[0])) { st->stop = "gcc cfg_ahb off (GPUCC not touched)"; return false; }
  st->gpuccOk = true;
  const bool pll0 = gm_pll_active(gm_rd(io, GMR_PLL0));
  if (pll0 && !(gm_rd(io, GMR_VOTE) & GM_VOTE_GPU_GPLL0)) {
    gm_wr(io, GMR_VOTE, gm_rd(io, GMR_VOTE) | GM_VOTE_GPU_GPLL0);
    st->enabled |= 1u << GMR_VOTE;
    io->udelay(io->ctx, GM_BRANCH_DELAY_US);   // BRANCH_HALT_DELAY
  }

  // GPUCC before-snapshot (nothing in GPUCC written yet)
  gm_snap(io, st->before, GMR_GCC_END, GMR_COUNT);
  for (unsigned r = GMR_GCC_END; r < GMR_COUNT; r = r + 1) st->haveBefore[r] = true;

  // CX GDSC (kgsl_smmu vdd-supply, enabled before its clocks)
  st->cx = "off";
  uint32_t v = gm_rd(io, GMR_CX_GDSCR);
  if (v & GM_GDSC_HW_CONTROL) { st->cx = "hwctrl"; st->stop = "cx gdsc under HW control"; return false; }
  if (v & GM_GDSC_SW_COLLAPSE) {
    gm_wr(io, GMR_CX_GDSCR, v & ~GM_GDSC_SW_COLLAPSE);
    st->cxTouched = true;
    io->udelay(io->ctx, 1);
  }
  if (!gm_poll(io, GMR_CX_HWCTRL, GM_GDSC_PWR_ON, true, GM_GDSC_POLL_US)) { st->cx = "timeout"; st->stop = "cx gdsc PWR_ON timeout"; return false; }
  st->cx = "on";
  io->udelay(io->ctx, 2);

  // SMMU clocks: memnoc, snoc_dvm, gpu_cc_ahb, smmu vote
  gm_enable_phase(io, st, GM_PH_SMMU);

  // GX GDSC (kgsl vdd)
  st->gx = "off";
  v = gm_rd(io, GMR_GX_GDSCR);
  if (v & GM_GDSC_HW_CONTROL) { st->gx = "hwctrl"; st->stop = "gx gdsc under HW control"; return false; }
  if (!(v & GM_GDSC_PWR_ON)) {
    const uint32_t bcr = gm_rd(io, GMR_GX_BCR);
    gm_wr(io, GMR_GX_BCR, bcr | GM_BLK_ARES);
    io->udelay(io->ctx, 1);
    gm_wr(io, GMR_GX_BCR, bcr & ~GM_BLK_ARES);
    st->gxReset = true;
  }
  const uint32_t clamp = gm_rd(io, GMR_GX_CLAMP);
  if (clamp & GM_CLAMP_IO) { gm_wr(io, GMR_GX_CLAMP, clamp & ~GM_CLAMP_IO); st->gxTouched = true; }
  v = gm_rd(io, GMR_GX_GDSCR);
  if (v & GM_GDSC_SW_COLLAPSE) {
    gm_wr(io, GMR_GX_GDSCR, v & ~GM_GDSC_SW_COLLAPSE);
    st->gxTouched = true;
    io->udelay(io->ctx, 1);
  }
  if (!gm_poll(io, GMR_GX_GDSCR, GM_GDSC_PWR_ON, true, GM_GDSC_POLL_US)) { st->gx = "timeout"; st->stop = "gx gdsc PWR_ON timeout"; return false; }
  st->gx = "on";
  io->udelay(io->ctx, 2);

  // kgsl clock rates (core on XO, gmu 200 MHz), then the kgsl clocks
  bool changed = false;
  if (!gm_rcg_set(io, GMR_GFX_CMD, GMR_GFX_CFG, GM_GFX3D_XO_CFG, &changed)) { st->gfx = "stuck"; st->stop = "gx_gfx3d_clk_src update stuck"; return false; }
  st->gfx = changed ? "xo" : "xo-kept";
  if (pll0) {
    if (!gm_rcg_set(io, GMR_GMU_CMD, GMR_GMU_CFG, GM_GMU_200M_CFG, &changed)) { st->gmu = "stuck"; st->stop = "gmu_clk_src update stuck"; return false; }
    st->gmu = changed ? "200M" : "200M-kept";
  } else {
    st->gmu = "boot(no-gpll0)";
  }
  gm_enable_phase(io, st, GM_PH_KGSL);

  // Branches that did not come on at enable: one more bounded poll now that everything upstream is on.
  for (const gm_branch &b : kGmBranches) {
    if (!(st->stuck & (1u << b.reg))) continue;
    if (gm_branch_wait(io, b)) st->late |= 1u << b.reg;
    else if (!st->stop) st->stop = kGmRegs[b.reg].name;
  }
  if (st->stop) return false;
  st->clocks = "on";
  return true;
}

// After the power-up: the after-snapshot (GPUCC only when cfg_ahb was seen on), the gate re-read from the hardware and
// the GPU mapping. Returns false (st->stop set) when the GPU block must not be read.
static inline bool gm_prepare_read(const gm_io *io, gm_state *st) {
  gm_snap(io, st->after, 0, st->gpuccOk ? GMR_COUNT : GMR_GCC_END);
  st->haveAfter = true;
  if (st->stop) return false;
  const char *off = gm_gate(io, st->relaxed);
  if (off) { st->clocks = "gate"; st->stop = off; return false; }
  if (!io->map_gpu(io->ctx)) { st->stop = "gpu map failed"; return false; }
  st->mapped = true;
  return true;
}

// The only GPU accesses (reads), the gate re-checked immediately before the first one.
static inline bool gm_do_reads(const gm_io *io, gm_state *st) {
  if (!st->mapped || st->stop) return false;
  const char *off = gm_gate(io, st->relaxed);
  if (off) { st->clocks = "gate"; st->stop = off; return false; }
  st->vbif = io->rd(io->ctx, GM_GPU, GM_A6XX_VBIF_VERSION * 4u);
  st->status = io->rd(io->ctx, GM_GPU, GM_A6XX_RBBM_STATUS * 4u);
  st->aon0 = io->rd(io->ctx, GM_GPU, GM_A6XX_CP_ALWAYS_ON_COUNTER_LO * 4u);
  io->udelay(io->ctx, GM_AON_GAP_US);
  st->aon1 = io->rd(io->ctx, GM_GPU, GM_A6XX_CP_ALWAYS_ON_COUNTER_LO * 4u);
  const uint32_t d = st->aon1 - st->aon0;
  st->aonOk = d != 0 && d < 0x80000000u;
  st->gpuRead = true;
  return true;
}

static inline bool gm_read_gpu(const gm_io *io, gm_state *st) { return gm_prepare_read(io, st) && gm_do_reads(io, st); }

// Clear the enable bit of every branch of one phase that gm_power_up enabled, in reverse table order.
static inline void gm_disable_phase(const gm_io *io, const gm_state *st, gm_phase ph) {
  for (size_t i = GM_NBRANCH; i > 0; i = i - 1) {
    const gm_branch &b = kGmBranches[i - 1];
    if (b.phase == ph && (st->enabled & (1u << b.reg))) gm_wr(io, b.reg, gm_rd(io, b.reg) & ~GM_CBCR_EN);
  }
}

// Undo, in reverse order, only what gm_power_up changed (gdsc_disable order for GX: collapse, poll, clamp). The RCG
// selections stay (XO / GPLL0 mux, harmless with the branches off). Returns false when GX did not report off.
static inline bool gm_power_down_gpucc(const gm_io *io, gm_state *st) {
  bool ok = true;
  gm_disable_phase(io, st, GM_PH_KGSL);
  if (st->gxTouched) {
    if (st->before[GMR_GX_GDSCR] & GM_GDSC_SW_COLLAPSE) {
      gm_wr(io, GMR_GX_GDSCR, gm_rd(io, GMR_GX_GDSCR) | GM_GDSC_SW_COLLAPSE);
      io->udelay(io->ctx, 1);
      ok = gm_poll(io, GMR_GX_GDSCR, GM_GDSC_PWR_ON, false, GM_GDSC_POLL_US);
    }
    if (st->before[GMR_GX_CLAMP] & GM_CLAMP_IO) gm_wr(io, GMR_GX_CLAMP, gm_rd(io, GMR_GX_CLAMP) | GM_CLAMP_IO);
  }
  gm_disable_phase(io, st, GM_PH_SMMU);
  if (st->cxTouched) {   // qcom,no-status-check-on-disable: fixed wait instead of a poll
    gm_wr(io, GMR_CX_GDSCR, gm_rd(io, GMR_CX_GDSCR) | GM_GDSC_SW_COLLAPSE);
    io->udelay(io->ctx, 100);
  }
  return ok;
}

static inline bool gm_power_down(const gm_io *io, gm_state *st) {
  const bool ok = st->gpuccOk ? gm_power_down_gpucc(io, st) : true;
  if (st->enabled & (1u << GMR_VOTE)) gm_wr(io, GMR_VOTE, gm_rd(io, GMR_VOTE) & ~GM_VOTE_GPU_GPLL0);
  gm_disable_phase(io, st, GM_PH_GCC);
  gm_snap(io, st->off, 0, st->gpuccOk ? GMR_COUNT : GMR_GCC_END);
  st->haveOff = true;
  return ok;
}

#endif
