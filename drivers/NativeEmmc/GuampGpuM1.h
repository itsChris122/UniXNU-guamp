// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp (Moto G9 Play, SM6115 / bengal) Adreno 610 milestone M1: power-up sequencing and the first GPU register reads.
// Pure code: every register access goes through gm_io (GpuService.inc maps the blocks, tests/gpu_host.cpp is a fake
// register file). No TrustZone call, no firmware, no command submission, no GPU write at all, no RBBM_SW_RESET (A610
// reset line erratum, Linux a6xx_gpu.c). Every constant states its source in the vendor tree
// android_kernel_motorola_sm6225 (gcc-bengal.c, gpucc-bengal.c, gdsc-regulator.c, clk-branch.c, clk-rcg2.c,
// clk-alpha-pll.c, bengal.dtsi, bengal-gdsc.dtsi, bengal-gpu.dtsi, a6xx_reg.h).
//
// Sequence (gm_power_up), each wait bounded:
//   GCC     gpu_cfg_ahb (GPUCC/GPU config interface: gate for any GPUCC access), gpu_memnoc_gfx, gpu_snoc_dvm_gfx,
//           gpu_iref, bimc_gpu_axi, vote gcc_gpu_gpll0_clk_src (GPLL0 main to GPUCC mux input 5)
//   CX      GDSC 0x106c: clear SW_COLLAPSE, poll PWR_ON in GDS_HW_CTRL 0x1540 (hw-ctrl-addr, as poll_gdsc_status)
//           then branches ahb, cxo, gmu_clk_src at 200 MHz (GPLL0 / 3, only when GPLL0 is active; otherwise the
//           boot source is kept), cx_gmu, hlos1_vote_gpu_smmu
//   GX      GDSC 0x100c (parent CX): only when it is off, BLK_ARES pulse in GPU_GX_BCR 0x1008 and CLAMP_IO cleared in
//           0x1508 (gdsc_enable toggle_logic order), clear SW_COLLAPSE, poll PWR_ON in GDSCR
//           gx_gfx3d_clk_src on XO (19.2 MHz, no PLL programming), gx_gfx3d and cx_gfx3d branches
//   gate    both GDSCs PWR_ON and every branch on the register path on (CLK_OFF clear) re-read right before; only
//           then gm_io::map_gpu and the reads VBIF_VERSION, RBBM_STATUS, CP_ALWAYS_ON_COUNTER_LO twice.
// gm_power_down undoes exactly what gm_power_up changed, in reverse order (GX collapse + clamp, CX collapse, votes).
#ifndef GUAMP_GPU_M1_H
#define GUAMP_GPU_M1_H
#include <stdint.h>
#include <stddef.h>

// --- blocks (bengal.dtsi gcc 0x1400000 len 0x1f0000; gpucc 0x5990000 len 0x9000; bengal-gpu.dtsi kgsl-3d0
// 0x5900000 len 0x90000) ------------------------------------------------------------------------------------------
enum gm_block : unsigned { GM_GCC = 0, GM_GPUCC = 1, GM_GPU = 2 };
#define GM_GCC_PHYS 0x01400000ULL
#define GM_GCC_SIZE 0x80000ULL              // up to 0x79004, the branch vote register
#define GM_GPUCC_PHYS 0x05990000ULL
#define GM_GPUCC_SIZE 0x9000ULL
#define GM_GPU_PHYS 0x05900000ULL
#define GM_GPU_SIZE 0x90000ULL

// --- GCC (gcc-bengal.c) -------------------------------------------------------------------------------------------
#define GM_GCC_GPLL0_MODE 0x0               // gpll0 .offset 0, PLL_MODE (clk-alpha-pll.c); vote 0x79000 BIT(0)
#define GM_GCC_GPU_CFG_AHB 0x36004          // BRANCH_HALT, hwcg_reg 0x36004, CLK_IS_CRITICAL
#define GM_GCC_GPU_MEMNOC_GFX 0x3600c       // BRANCH_VOTED, hwcg_reg 0x3600c
#define GM_GCC_GPU_SNOC_DVM_GFX 0x36018     // BRANCH_HALT
#define GM_GCC_GPU_IREF 0x36100             // BRANCH_HALT_DELAY
#define GM_GCC_BIMC_GPU_AXI 0x71154         // BRANCH_HALT_DELAY
#define GM_GCC_BRANCH_VOTE 0x79004          // gcc_gpu_gpll0_clk_src BIT(15) (parent gpll0); BIT(16) is
#define GM_VOTE_GPU_GPLL0 (1u << 15)        // gcc_gpu_gpll0_div_clk_src (gpll0_out_aux2, GPUCC input 6): not used
#define GM_PLL_ACTIVE (1u << 30)            // PLL_ACTIVE_FLAG
#define GM_PLL_LOCK_DET (1u << 31)          // PLL_LOCK_DET

// --- GPUCC (gpucc-bengal.c, bengal-gdsc.dtsi) ----------------------------------------------------------------------
#define GM_CC_GX_BCR 0x1008                 // gpu_gx_sw_reset syscon@5991008
#define GM_CC_GX_GDSCR 0x100c               // gpu_gx_gdsc qcom,gdsc@599100c
#define GM_CC_GFX3D_RCG 0x101c              // gpu_cc_gx_gfx3d_clk_src cmd_rcgr, hid 5, mnd 0, parent_map_1
#define GM_CC_GX_GFX3D 0x1054               // gpu_cc_gx_gfx3d_clk, BRANCH_HALT_SKIP in Linux, gated here anyway
#define GM_CC_CX_GDSCR 0x106c               // gpu_cx_gdsc qcom,gdsc@599106c, gds-timeout 500
#define GM_CC_AHB 0x1078                    // gpu_cc_ahb_clk, CLK_IS_CRITICAL
#define GM_CC_CX_GMU 0x1098                 // gpu_cc_cx_gmu_clk, BRANCH_HALT, parent gpu_cc_gmu_clk_src
#define GM_CC_CXO 0x109c                    // gpu_cc_cxo_clk (rbbmtimer_clk), BRANCH_HALT
#define GM_CC_CX_GFX3D 0x10a4               // gpu_cc_cx_gfx3d_clk, parent gx_gfx3d_clk_src
#define GM_CC_GMU_RCG 0x1120                // gpu_cc_gmu_clk_src cmd_rcgr, hid 5, mnd 0, parent_map_0
#define GM_CC_GX_CLAMP 0x1508               // gpu_gx_domain_addr syscon@5991508
#define GM_CC_CX_HW_CTRL 0x1540             // gpu_cx_hw_ctrl syscon@5991540 (polled for CX PWR_ON)
#define GM_CC_SMMU_VOTE 0x5000              // gpu_cc_hlos1_vote_gpu_smmu_clk, BRANCH_VOTED

// gdsc-regulator.c
#define GM_GDSC_SW_COLLAPSE (1u << 0)
#define GM_GDSC_HW_CONTROL (1u << 1)        // set: Linux refuses the enable ("under HW control")
#define GM_GDSC_PWR_ON (1u << 31)
#define GM_CLAMP_IO (1u << 0)               // GMEM_CLAMP_IO_MASK
#define GM_BLK_ARES (1u << 0)               // BCR_BLK_ARES_BIT
// clk-branch.c / clk-branch.h
#define GM_CBCR_EN (1u << 0)
#define GM_CBCR_HWCG (1u << 1)              // hwcg_bit default 1: clock in hardware gating mode, halt not checked
#define GM_CBCR_FSM_MASK (7u << 28)
#define GM_CBCR_FSM_ON (2u << 28)           // BRANCH_NOC_FSM_STATUS_ON
#define GM_CBCR_CLK_OFF (1u << 31)
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

// Every register gm_power_up may write (or read for a decision), in log order: the before/after snapshot.
enum gm_reg : unsigned {
  GMR_VOTE, GMR_PLL0, GMR_CFG_AHB, GMR_MEMNOC, GMR_SNOC_DVM, GMR_IREF, GMR_AXI,                      // GCC
  GMR_CX_GDSCR, GMR_CX_HWCTRL, GMR_AHB, GMR_CXO, GMR_GMU_CMD, GMR_GMU_CFG, GMR_CX_GMU, GMR_SMMU,     // GPUCC CX
  GMR_GX_BCR, GMR_GX_CLAMP, GMR_GX_GDSCR, GMR_GFX_CMD, GMR_GFX_CFG, GMR_GX_GFX3D, GMR_CX_GFX3D,     // GPUCC GX
  GMR_COUNT
};
struct gm_reg_desc { const char *name; unsigned block; uint32_t off; };
static const gm_reg_desc kGmRegs[GMR_COUNT] = {
    {"vote", GM_GCC, GM_GCC_BRANCH_VOTE}, {"pll0", GM_GCC, GM_GCC_GPLL0_MODE}, {"cfg_ahb", GM_GCC, GM_GCC_GPU_CFG_AHB},
    {"memnoc", GM_GCC, GM_GCC_GPU_MEMNOC_GFX}, {"snoc_dvm", GM_GCC, GM_GCC_GPU_SNOC_DVM_GFX}, {"iref", GM_GCC, GM_GCC_GPU_IREF},
    {"axi", GM_GCC, GM_GCC_BIMC_GPU_AXI},
    {"cx_gdscr", GM_GPUCC, GM_CC_CX_GDSCR}, {"cx_hwctrl", GM_GPUCC, GM_CC_CX_HW_CTRL}, {"ahb", GM_GPUCC, GM_CC_AHB},
    {"cxo", GM_GPUCC, GM_CC_CXO}, {"gmu_cmd", GM_GPUCC, GM_CC_GMU_RCG + GM_RCG_CMD},
    {"gmu_cfg", GM_GPUCC, GM_CC_GMU_RCG + GM_RCG_CFG}, {"cx_gmu", GM_GPUCC, GM_CC_CX_GMU}, {"smmu", GM_GPUCC, GM_CC_SMMU_VOTE},
    {"gx_bcr", GM_GPUCC, GM_CC_GX_BCR}, {"gx_clamp", GM_GPUCC, GM_CC_GX_CLAMP}, {"gx_gdscr", GM_GPUCC, GM_CC_GX_GDSCR},
    {"gfx_cmd", GM_GPUCC, GM_CC_GFX3D_RCG + GM_RCG_CMD}, {"gfx_cfg", GM_GPUCC, GM_CC_GFX3D_RCG + GM_RCG_CFG},
    {"gx_gfx3d", GM_GPUCC, GM_CC_GX_GFX3D}, {"cx_gfx3d", GM_GPUCC, GM_CC_CX_GFX3D},
};

// Branches, in enable order. gate: must be on before any GPU register read (register path); the two GCC
// BRANCH_HALT_DELAY clocks (no status in Linux) are enabled and reported only.
struct gm_branch { unsigned reg; bool hwcg, gate; };
static const gm_branch kGmGccBranches[] = {
    {GMR_CFG_AHB, true, true}, {GMR_MEMNOC, true, true}, {GMR_SNOC_DVM, false, true}, {GMR_IREF, false, false},
    {GMR_AXI, false, false}};
static const gm_branch kGmCxBranches[] = {{GMR_AHB, false, true}, {GMR_CXO, false, true}};   // cx_gmu after its RCG
static const gm_branch kGmCxGmu = {GMR_CX_GMU, false, true};
static const gm_branch kGmSmmu = {GMR_SMMU, false, true};
static const gm_branch kGmGxBranches[] = {{GMR_GX_GFX3D, false, true}, {GMR_CX_GFX3D, false, true}};

struct gm_state {
  uint32_t before[GMR_COUNT], after[GMR_COUNT], off[GMR_COUNT];
  bool haveBefore[GMR_COUNT], haveAfter, haveOff;
  const char *stop;          // null = every step passed; otherwise the first failure, no GPU access happened
  const char *cx, *gx, *clocks, *gmu, *gfx;   // words for the RESULT line
  bool gpuccOk;              // gcc cfg_ahb seen on: GPUCC may be accessed (snapshots, power-down)
  bool gpuRead, aonOk, mapped;
  uint32_t vbif, status, aon0, aon1;
  bool cxTouched, gxTouched, gxReset;          // power_down undoes only these
  uint32_t enabled;                            // bit per gm_reg: branch enable bit (or vote) set by us
};

static inline uint32_t gm_rd(const gm_io *io, unsigned r) { return io->rd(io->ctx, kGmRegs[r].block, kGmRegs[r].off); }
static inline void gm_wr(const gm_io *io, unsigned r, uint32_t v) { io->wr(io->ctx, kGmRegs[r].block, kGmRegs[r].off, v); }

// clk_branch2_check_halt (enabling) plus clk_branch_in_hwcg_mode
static inline bool gm_branch_is_on(uint32_t v, bool hwcg) {
  if (hwcg && (v & GM_CBCR_HWCG) && (v & GM_CBCR_EN)) return true;
  return !(v & GM_CBCR_CLK_OFF) || (v & (GM_CBCR_CLK_OFF | GM_CBCR_FSM_MASK)) == GM_CBCR_FSM_ON;
}
static inline bool gm_pll_active(uint32_t mode) { return (mode & GM_PLL_LOCK_DET) || (mode & GM_PLL_ACTIVE); }

static inline bool gm_poll(const gm_io *io, unsigned r, uint32_t mask, bool wantSet, unsigned us) {
  for (unsigned n = 0;; n = n + 1) {
    const uint32_t v = gm_rd(io, r);
    if (((v & mask) != 0) == wantSet) return true;
    if (n >= us) return false;
    io->udelay(io->ctx, 1);
  }
}

static inline bool gm_branch_enable(const gm_io *io, gm_state *st, const gm_branch &b) {
  const uint32_t v = gm_rd(io, b.reg);
  if (!(v & GM_CBCR_EN)) {
    gm_wr(io, b.reg, v | GM_CBCR_EN);
    st->enabled |= 1u << b.reg;
  }
  for (unsigned n = 0;; n = n + 1) {
    if (gm_branch_is_on(gm_rd(io, b.reg), b.hwcg)) return true;
    if (n >= GM_BRANCH_POLL_US) return false;
    io->udelay(io->ctx, 1);
  }
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

// The final gate, re-read from the hardware: returns null when the GPU block may be read, else what is off.
static inline const char *gm_gate(const gm_io *io) {
  if (!gm_branch_is_on(gm_rd(io, GMR_CFG_AHB), true)) return "cfg_ahb";
  if (!gm_cx_on(io)) return "cx";
  if (!gm_gx_on(io)) return "gx";
  const gm_branch *lists[4] = {kGmGccBranches, kGmCxBranches, kGmGxBranches, nullptr};
  const size_t sizes[3] = {sizeof kGmGccBranches / sizeof kGmGccBranches[0], sizeof kGmCxBranches / sizeof kGmCxBranches[0],
                           sizeof kGmGxBranches / sizeof kGmGxBranches[0]};
  for (unsigned l = 0; lists[l]; l = l + 1)
    for (size_t i = 0; i < sizes[l]; i = i + 1)
      if (lists[l][i].gate && !gm_branch_is_on(gm_rd(io, lists[l][i].reg), lists[l][i].hwcg)) return kGmRegs[lists[l][i].reg].name;
  if (!gm_branch_is_on(gm_rd(io, kGmCxGmu.reg), false)) return "cx_gmu";
  if (!gm_branch_is_on(gm_rd(io, kGmSmmu.reg), false)) return "smmu";
  return nullptr;
}

static inline void gm_init(gm_state *st) {
  *st = gm_state{};
  st->cx = st->gx = st->clocks = st->gmu = st->gfx = "-";
}

// Power-up. Returns false (st->stop set) at the first failure; the GPU block is then never touched.
static inline bool gm_power_up(const gm_io *io, gm_state *st) {
  // GCC: always readable. Snapshot, then the GPU branches; cfg_ahb gates every GPUCC access.
  gm_snap(io, st->before, GMR_VOTE, GMR_CX_GDSCR);
  for (unsigned r = GMR_VOTE; r < GMR_CX_GDSCR; r = r + 1) st->haveBefore[r] = true;
  st->clocks = "off";
  for (const gm_branch &b : kGmGccBranches) {
    if (!gm_branch_enable(io, st, b) && b.gate) { st->stop = b.reg == GMR_CFG_AHB ? "gcc cfg_ahb off (GPUCC not touched)" : kGmRegs[b.reg].name; return false; }
  }
  st->gpuccOk = true;
  const bool pll0 = gm_pll_active(gm_rd(io, GMR_PLL0));
  if (pll0 && !(gm_rd(io, GMR_VOTE) & GM_VOTE_GPU_GPLL0)) {
    gm_wr(io, GMR_VOTE, gm_rd(io, GMR_VOTE) | GM_VOTE_GPU_GPLL0);
    st->enabled |= 1u << GMR_VOTE;
    io->udelay(io->ctx, 10);   // BRANCH_HALT_DELAY
  }

  // GPUCC before-snapshot (nothing in GPUCC written yet)
  gm_snap(io, st->before, GMR_CX_GDSCR, GMR_COUNT);
  for (unsigned r = GMR_CX_GDSCR; r < GMR_COUNT; r = r + 1) st->haveBefore[r] = true;

  // CX GDSC
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

  // CX branches
  for (const gm_branch &b : kGmCxBranches)
    if (!gm_branch_enable(io, st, b)) { st->stop = kGmRegs[b.reg].name; return false; }
  bool changed = false;
  if (pll0) {
    if (!gm_rcg_set(io, GMR_GMU_CMD, GMR_GMU_CFG, GM_GMU_200M_CFG, &changed)) { st->gmu = "stuck"; st->stop = "gmu_clk_src update stuck"; return false; }
    st->gmu = changed ? "200M" : "200M-kept";
  } else {
    st->gmu = "boot(no-gpll0)";
  }
  if (!gm_branch_enable(io, st, kGmCxGmu)) { st->stop = "cx_gmu"; return false; }
  if (!gm_branch_enable(io, st, kGmSmmu)) { st->stop = "smmu"; return false; }

  // GX GDSC
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

  // core clock on XO, then its branches
  if (!gm_rcg_set(io, GMR_GFX_CMD, GMR_GFX_CFG, GM_GFX3D_XO_CFG, &changed)) { st->gfx = "stuck"; st->stop = "gx_gfx3d_clk_src update stuck"; return false; }
  st->gfx = changed ? "xo" : "xo-kept";
  for (const gm_branch &b : kGmGxBranches)
    if (!gm_branch_enable(io, st, b)) { st->stop = kGmRegs[b.reg].name; return false; }
  st->clocks = "on";
  return true;
}

// After the power-up: the after-snapshot (GPUCC only when cfg_ahb was seen on), the gate re-read from the hardware and
// the GPU mapping. Returns false (st->stop set) when the GPU block must not be read.
static inline bool gm_prepare_read(const gm_io *io, gm_state *st) {
  gm_snap(io, st->after, 0, st->gpuccOk ? GMR_COUNT : GMR_CX_GDSCR);
  st->haveAfter = true;
  if (st->stop) return false;
  const char *off = gm_gate(io);
  if (off) { st->clocks = "gate"; st->stop = off; return false; }
  if (!io->map_gpu(io->ctx)) { st->stop = "gpu map failed"; return false; }
  st->mapped = true;
  return true;
}

// The only GPU accesses (reads), the gate re-checked immediately before the first one.
static inline bool gm_do_reads(const gm_io *io, gm_state *st) {
  if (!st->mapped || st->stop) return false;
  const char *off = gm_gate(io);
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

// Undo, in reverse order, only what gm_power_up changed (gdsc_disable order for GX: collapse, poll, clamp). The RCG
// selections stay (XO / GPLL0 mux, harmless with the branches off). Returns false when GX did not report off.
static inline bool gm_power_down_gpucc(const gm_io *io, gm_state *st) {
  bool ok = true;
  for (int i = static_cast<int>(sizeof kGmGxBranches / sizeof kGmGxBranches[0]) - 1; i >= 0; i = i - 1) {
    const unsigned r = kGmGxBranches[i].reg;
    if (st->enabled & (1u << r)) gm_wr(io, r, gm_rd(io, r) & ~GM_CBCR_EN);
  }
  if (st->gxTouched) {
    if (st->before[GMR_GX_GDSCR] & GM_GDSC_SW_COLLAPSE) {
      gm_wr(io, GMR_GX_GDSCR, gm_rd(io, GMR_GX_GDSCR) | GM_GDSC_SW_COLLAPSE);
      io->udelay(io->ctx, 1);
      ok = gm_poll(io, GMR_GX_GDSCR, GM_GDSC_PWR_ON, false, GM_GDSC_POLL_US);
    }
    if (st->before[GMR_GX_CLAMP] & GM_CLAMP_IO) gm_wr(io, GMR_GX_CLAMP, gm_rd(io, GMR_GX_CLAMP) | GM_CLAMP_IO);
  }
  const unsigned cx[4] = {GMR_SMMU, GMR_CX_GMU, GMR_CXO, GMR_AHB};
  for (unsigned r : cx)
    if (st->enabled & (1u << r)) gm_wr(io, r, gm_rd(io, r) & ~GM_CBCR_EN);
  if (st->cxTouched) {   // qcom,no-status-check-on-disable: fixed wait instead of a poll
    gm_wr(io, GMR_CX_GDSCR, gm_rd(io, GMR_CX_GDSCR) | GM_GDSC_SW_COLLAPSE);
    io->udelay(io->ctx, 100);
  }
  return ok;
}

static inline bool gm_power_down(const gm_io *io, gm_state *st) {
  const bool ok = st->gpuccOk ? gm_power_down_gpucc(io, st) : true;
  if (st->enabled & (1u << GMR_VOTE)) gm_wr(io, GMR_VOTE, gm_rd(io, GMR_VOTE) & ~GM_VOTE_GPU_GPLL0);
  for (int i = static_cast<int>(sizeof kGmGccBranches / sizeof kGmGccBranches[0]) - 1; i >= 0; i = i - 1) {
    const unsigned r = kGmGccBranches[i].reg;
    if (st->enabled & (1u << r)) gm_wr(io, r, gm_rd(io, r) & ~GM_CBCR_EN);
  }
  gm_snap(io, st->off, 0, st->gpuccOk ? GMR_COUNT : GMR_CX_GDSCR);
  st->haveOff = true;
  return ok;
}

#endif
