// Host tests of the Adreno 610 M1 sequencing (GuampGpuM1.h) against a fake GCC / GPUCC / GPU register file that
// models the GDSC PWR_ON and branch CLK_OFF behaviour, RCG updates and the bus rules the kernel must respect: no
// GPUCC access while gcc cfg_ahb is off, no GPU access unless both GDSCs and the register-path clocks are on (a real
// unclocked read aborts the bus), no GPU write at all, no write outside the M1 register list.
#include <stdio.h>
#include <string.h>
#include <map>
#include "../GuampGpuM1.h"

static int failures = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

struct Fake {
  std::map<uint32_t, uint32_t> gcc, cc;
  // knobs
  bool cfgAhbStuck = false, cxNever = false, gxNever = false, gxGfx3dStuck = false, gfxRcgStuck = false, pll0 = true,
       aonStuck = false, dropGxClock = false;
  // observations
  uint64_t us = 0;
  unsigned aresPulses = 0, gpuReads = 0, gpuUnclocked = 0, gpuWrites = 0, gpuccNoAhb = 0, gpuccAccesses = 0,
           badWrites = 0, gxWrites = 0, cxWrites = 0, mapCalls = 0;
  bool ares = false;

  Fake() {   // the boot state seen on the phone (kpan 2026-10-25, GuampGpuProbe) plus reset values
    gcc[GM_GCC_BRANCH_VOTE] = 0x00000400;
    gcc[GM_GCC_GPU_CFG_AHB] = GM_CBCR_EN | GM_CBCR_HWCG;
    gcc[GM_GCC_GPU_MEMNOC_GFX] = 0x80000000;
    gcc[GM_GCC_GPU_SNOC_DVM_GFX] = 0x80000000;
    gcc[GM_GCC_GPU_IREF] = 0x80000000;
    gcc[GM_GCC_BIMC_GPU_AXI] = 0x80000000;
    cc[GM_CC_CX_GDSCR] = 0x00222001;
    cc[GM_CC_GX_GDSCR] = 0x00222001;
    cc[GM_CC_GX_CLAMP] = GM_CLAMP_IO;
    cc[GM_CC_GX_BCR] = 0;
    cc[GM_CC_CX_HW_CTRL] = 0;
    cc[GM_CC_AHB] = 0x80000001;
    cc[GM_CC_CXO] = 0x80000000;
    cc[GM_CC_CX_GMU] = 0x80004220;
    cc[GM_CC_SMMU_VOTE] = 0x80000000;
    cc[GM_CC_CX_GFX3D] = 0x80000000;
    cc[GM_CC_GX_GFX3D] = 0x80004220;
    cc[GM_CC_GMU_RCG] = 0x80000000; cc[GM_CC_GMU_RCG + 4] = 0;
    cc[GM_CC_GFX3D_RCG] = 0x80000000; cc[GM_CC_GFX3D_RCG + 4] = 0;
  }
  bool gccOn(uint32_t off) const {
    const uint32_t v = gcc.at(off);
    if (off == GM_GCC_GPU_CFG_AHB && cfgAhbStuck) return false;
    return v & GM_CBCR_EN;
  }
  bool cxOn() const { return !cxNever && !(cc.at(GM_CC_CX_GDSCR) & GM_GDSC_SW_COLLAPSE); }
  bool gxOn() const { return !gxNever && cxOn() && !ares && !(cc.at(GM_CC_GX_GDSCR) & GM_GDSC_SW_COLLAPSE); }
  bool rcgOk(uint32_t rcg) const {
    const uint32_t sel = (cc.at(rcg + 4) >> 8) & 7;
    if (sel == 0) return true;
    if (sel == 5) return pll0 && (gcc.at(GM_GCC_BRANCH_VOTE) & GM_VOTE_GPU_GPLL0);
    return false;   // GPUCC PLLs are never programmed in M1: off
  }
  bool ccOn(uint32_t off) const {
    if (!(cc.at(off) & GM_CBCR_EN) || !cxOn()) return false;
    switch (off) {
    case GM_CC_CX_GMU: return rcgOk(GM_CC_GMU_RCG);
    case GM_CC_CX_GFX3D: return rcgOk(GM_CC_GFX3D_RCG) && gxOn();
    case GM_CC_GX_GFX3D: return rcgOk(GM_CC_GFX3D_RCG) && gxOn() && !gxGfx3dStuck && !dropGxClock;
    default: return true;
    }
  }
  bool gpuClocked() const {
    return gccOn(GM_GCC_GPU_CFG_AHB) && cxOn() && gxOn() && ccOn(GM_CC_AHB) && ccOn(GM_CC_CXO) && ccOn(GM_CC_CX_GMU) &&
           ccOn(GM_CC_CX_GFX3D) && ccOn(GM_CC_GX_GFX3D);
  }
  static uint32_t withOff(uint32_t v, bool on) { return on ? (v & ~GM_CBCR_CLK_OFF) : (v | GM_CBCR_CLK_OFF); }

  uint32_t read(unsigned block, uint32_t off) {
    if (block == GM_GCC) {
      if (off == GM_GCC_GPLL0_MODE) return pll0 ? 0xC0000007u : 0u;
      if (!gcc.count(off)) { badWrites++; return 0; }
      const uint32_t v = gcc[off];
      if (off == GM_GCC_BRANCH_VOTE) return v;
      if (off == GM_GCC_GPU_CFG_AHB) return cfgAhbStuck ? (v & ~GM_CBCR_HWCG) | GM_CBCR_CLK_OFF : v;   // hwcg: CLK_OFF unreliable
      return withOff(v, gccOn(off));
    }
    if (block == GM_GPUCC) {
      gpuccAccesses++;
      if (!gccOn(GM_GCC_GPU_CFG_AHB)) { gpuccNoAhb++; return 0xdeadbeef; }
      if (!cc.count(off)) { badWrites++; return 0; }
      const uint32_t v = cc[off];
      switch (off) {
      case GM_CC_CX_GDSCR: return cxOn() ? v | GM_GDSC_PWR_ON : v & ~GM_GDSC_PWR_ON;
      case GM_CC_CX_HW_CTRL: return cxOn() ? GM_GDSC_PWR_ON : 0;
      case GM_CC_GX_GDSCR: return gxOn() ? v | GM_GDSC_PWR_ON : v & ~GM_GDSC_PWR_ON;
      case GM_CC_AHB: case GM_CC_CXO: case GM_CC_CX_GMU: case GM_CC_SMMU_VOTE: case GM_CC_CX_GFX3D: case GM_CC_GX_GFX3D:
        return withOff(v, ccOn(off));
      default: return v;
      }
    }
    gpuReads++;
    if (!gpuClocked()) { gpuUnclocked++; return 0xdeadbeef; }
    switch (off) {
    case GM_A6XX_VBIF_VERSION * 4: return 0x10070000;
    case GM_A6XX_RBBM_STATUS * 4: return 0;
    case GM_A6XX_CP_ALWAYS_ON_COUNTER_LO * 4: return aonStuck ? 0x1234 : static_cast<uint32_t>(us * 192 / 10);
    default: return 0;
    }
  }
  void write(unsigned block, uint32_t off, uint32_t v) {
    if (block == GM_GPU) { gpuWrites++; return; }
    bool known = false;
    for (const gm_reg_desc &d : kGmRegs) known = known || (d.block == block && d.off == off);
    if (!known || (block == GM_GCC && off == GM_GCC_GPLL0_MODE) || (block == GM_GPUCC && off == GM_CC_CX_HW_CTRL)) { badWrites++; return; }
    // bit 31 is read-only status (CLK_OFF, PWR_ON, ROOT_OFF) except in the vote register and the RCG CFG
    const bool ro31 = !(block == GM_GCC && off == GM_GCC_BRANCH_VOTE) && off != GM_CC_GMU_RCG + 4 && off != GM_CC_GFX3D_RCG + 4;
    if (ro31) v &= ~0x80000000u;
    if (block == GM_GCC) { gcc[off] = (gcc[off] & 0x80000000u) | v; return; }
    gpuccAccesses++;
    if (!gccOn(GM_GCC_GPU_CFG_AHB)) { gpuccNoAhb++; return; }
    if (off == GM_CC_GX_BCR || off == GM_CC_GX_CLAMP || off == GM_CC_GX_GDSCR) gxWrites++;
    if (off == GM_CC_CX_GDSCR) cxWrites++;
    if (off == GM_CC_GX_BCR) {
      const bool a = v & GM_BLK_ARES;
      if (ares && !a) aresPulses++;
      ares = a;
    }
    if ((off == GM_CC_GMU_RCG || off == GM_CC_GFX3D_RCG) && (v & GM_RCG_UPDATE)) {
      if (!(off == GM_CC_GFX3D_RCG && gfxRcgStuck)) v &= ~GM_RCG_UPDATE;
    }
    cc[off] = ro31 ? (cc[off] & 0x80000000u) | v : v;
  }
};

static uint32_t fRd(void *c, unsigned b, uint32_t o) { return static_cast<Fake *>(c)->read(b, o); }
static void fWr(void *c, unsigned b, uint32_t o, uint32_t v) { static_cast<Fake *>(c)->write(b, o, v); }
static void fDelay(void *c, unsigned us) { static_cast<Fake *>(c)->us += us; }
static bool fMap(void *c) { static_cast<Fake *>(c)->mapCalls++; return true; }

struct Run { Fake f; gm_io io; gm_state st; bool up = false, read = false; };
static void run(Run &r, bool dropAfterUp = false) {
  r.io = gm_io{&r.f, fRd, fWr, fDelay, fMap};
  gm_init(&r.st);
  r.up = gm_power_up(&r.io, &r.st);
  if (dropAfterUp) r.f.dropGxClock = true;
  r.read = gm_read_gpu(&r.io, &r.st);
}
static void noGpu(const Run &r) {
  CHECK(r.f.gpuReads == 0);
  CHECK(r.f.mapCalls == 0);
  CHECK(!r.st.gpuRead && !r.st.mapped);
}
static void common(const Run &r) {
  CHECK(r.f.gpuWrites == 0);
  CHECK(r.f.badWrites == 0);
  CHECK(r.f.gpuUnclocked == 0);
  CHECK(r.f.gpuccNoAhb == 0);
  CHECK(r.f.us < 20000);   // every wait bounded
}

int main() {
  {  // nominal, from the observed boot state
    Run r;
    run(r);
    common(r);
    CHECK(r.up && r.read && !r.st.stop);
    CHECK(r.f.gpuReads == 4 && r.f.mapCalls == 1);
    CHECK(r.st.vbif == 0x10070000 && r.st.status == 0);
    CHECK(r.st.aonOk && r.st.aon1 - r.st.aon0 >= 1900 && r.st.aon1 - r.st.aon0 <= 1940);
    CHECK(r.f.aresPulses == 1 && !r.f.ares);
    CHECK(!(r.f.cc[GM_CC_GX_CLAMP] & GM_CLAMP_IO));
    CHECK(r.f.cc[GM_CC_GMU_RCG + 4] == GM_GMU_200M_CFG);
    CHECK(r.f.cc[GM_CC_GFX3D_RCG + 4] == 0);   // already XO: not rewritten
    CHECK(r.f.gcc[GM_GCC_BRANCH_VOTE] == (0x400u | GM_VOTE_GPU_GPLL0));
    CHECK(!strcmp(r.st.cx, "on") && !strcmp(r.st.gx, "on") && !strcmp(r.st.clocks, "on"));
    CHECK(!strcmp(r.st.gmu, "200M") && !strcmp(r.st.gfx, "xo-kept"));
    CHECK(r.st.before[GMR_AHB] == 0x80000001 && r.st.before[GMR_CX_GDSCR] == 0x00222001 && r.st.before[GMR_GX_CLAMP] == 1);
    CHECK((r.st.after[GMR_GX_GDSCR] & GM_GDSC_PWR_ON) && (r.st.after[GMR_CX_HWCTRL] & GM_GDSC_PWR_ON));
    CHECK(!(r.st.after[GMR_GX_GFX3D] & GM_CBCR_CLK_OFF) && !(r.st.after[GMR_CX_GMU] & GM_CBCR_CLK_OFF));
    CHECK(r.st.after[GMR_GX_BCR] == 0);
    // power back down: every enable / collapse / clamp / vote bit as before
    const unsigned gpuReadsBefore = r.f.gpuReads;
    CHECK(gm_power_down(&r.io, &r.st));
    common(r);
    CHECK(r.f.gpuReads == gpuReadsBefore);
    CHECK(!r.f.gxOn() && !r.f.cxOn());
    CHECK(r.f.cc[GM_CC_GX_CLAMP] & GM_CLAMP_IO);
    CHECK(r.f.cc[GM_CC_CX_GDSCR] == 0x00222001 && r.f.cc[GM_CC_GX_GDSCR] == 0x00222001);
    const uint32_t ccRegs[] = {GM_CC_AHB, GM_CC_CXO, GM_CC_CX_GMU, GM_CC_SMMU_VOTE, GM_CC_CX_GFX3D, GM_CC_GX_GFX3D};
    for (uint32_t o : ccRegs) CHECK((r.f.cc[o] & GM_CBCR_EN) == (Fake().cc[o] & GM_CBCR_EN));
    const uint32_t gccRegs[] = {GM_GCC_BRANCH_VOTE, GM_GCC_GPU_CFG_AHB, GM_GCC_GPU_MEMNOC_GFX, GM_GCC_GPU_SNOC_DVM_GFX,
                                GM_GCC_GPU_IREF, GM_GCC_BIMC_GPU_AXI};
    for (uint32_t o : gccRegs) CHECK(r.f.gcc[o] == Fake().gcc[o]);
    CHECK(r.st.haveOff && !(r.st.off[GMR_GX_GDSCR] & GM_GDSC_PWR_ON));
  }
  {  // CX never powers: stop before GX, no GPU access
    Run r; r.f.cxNever = true;
    run(r);
    common(r); noGpu(r);
    CHECK(!r.up && r.st.stop && !strcmp(r.st.cx, "timeout"));
    CHECK(r.f.gxWrites == 0 && r.f.aresPulses == 0);
    CHECK(r.st.haveAfter && !(r.st.after[GMR_CX_HWCTRL] & GM_GDSC_PWR_ON));
    gm_power_down(&r.io, &r.st);
    CHECK(r.f.cc[GM_CC_CX_GDSCR] == 0x00222001);
  }
  {  // GX never powers
    Run r; r.f.gxNever = true;
    run(r);
    common(r); noGpu(r);
    CHECK(!r.up && !strcmp(r.st.gx, "timeout") && !strcmp(r.st.cx, "on"));
  }
  {  // gx_gfx3d CLK_OFF stuck
    Run r; r.f.gxGfx3dStuck = true;
    run(r);
    common(r); noGpu(r);
    CHECK(!r.up && r.st.stop && !strcmp(r.st.stop, "gx_gfx3d"));
  }
  {  // gcc cfg_ahb does not come on: GPUCC never accessed
    Run r; r.f.cfgAhbStuck = true; r.f.gcc[GM_GCC_GPU_CFG_AHB] = 0x80000000;
    run(r);
    common(r); noGpu(r);
    CHECK(!r.up && r.f.gpuccAccesses == 0 && !r.st.gpuccOk);
    gm_power_down(&r.io, &r.st);
    CHECK(r.f.gpuccAccesses == 0);
    CHECK(r.f.gcc[GM_GCC_GPU_CFG_AHB] == 0x80000000);
  }
  {  // CX GDSC under hardware control: refused before any CX write
    Run r; r.f.cc[GM_CC_CX_GDSCR] = 0x00222003;
    run(r);
    common(r); noGpu(r);
    CHECK(!r.up && !strcmp(r.st.cx, "hwctrl") && r.f.cxWrites == 0 && r.f.gxWrites == 0);
  }
  {  // gfx3d RCG left on a GPUCC PLL that is off: update never completes
    Run r; r.f.gfxRcgStuck = true; r.f.cc[GM_CC_GFX3D_RCG + 4] = (2u << 8) | 1u;
    run(r);
    common(r); noGpu(r);
    CHECK(!r.up && !strcmp(r.st.gfx, "stuck"));
  }
  {  // gfx3d RCG on a PLL, update works: moved to XO
    Run r; r.f.cc[GM_CC_GFX3D_RCG + 4] = (2u << 8) | 1u | (1u << 20);
    run(r);
    common(r);
    CHECK(r.up && r.read && !strcmp(r.st.gfx, "xo") && r.f.cc[GM_CC_GFX3D_RCG + 4] == ((1u << 20) | 1u));
  }
  {  // GPLL0 not active: no vote, gmu source kept on XO, still works
    Run r; r.f.pll0 = false;
    run(r);
    common(r);
    CHECK(r.up && r.read && !strcmp(r.st.gmu, "boot(no-gpll0)"));
    CHECK(r.f.gcc[GM_GCC_BRANCH_VOTE] == 0x400 && r.f.cc[GM_CC_GMU_RCG + 4] == 0);
  }
  {  // GX already powered (e.g. left on by a previous run): no BLK_ARES pulse
    Run r; r.f.cc[GM_CC_CX_GDSCR] = 0x00222000; r.f.cc[GM_CC_GX_GDSCR] = 0x00222000; r.f.cc[GM_CC_GX_CLAMP] = 0;
    run(r);
    common(r);
    CHECK(r.up && r.read && r.f.aresPulses == 0 && !r.st.gxReset && !r.st.gxTouched && !r.st.cxTouched);
    gm_power_down(&r.io, &r.st);
    CHECK(r.f.gxOn());   // was on before: left on
  }
  {  // always-on counter not moving
    Run r; r.f.aonStuck = true;
    run(r);
    common(r);
    CHECK(r.read && !r.st.aonOk);
  }
  {  // a clock drops between power-up and the reads: the gate re-read stops before mapping
    Run r;
    run(r, true);
    common(r); noGpu(r);
    CHECK(r.up && !r.read && r.st.stop && !strcmp(r.st.stop, "gx_gfx3d") && !strcmp(r.st.clocks, "gate"));
  }
  {  // a clock drops after the mapping (the service sleeps 1.5 s there): re-checked before the first GPU read
    Run r;
    r.io = gm_io{&r.f, fRd, fWr, fDelay, fMap};
    gm_init(&r.st);
    CHECK(gm_power_up(&r.io, &r.st) && gm_prepare_read(&r.io, &r.st) && r.st.mapped);
    r.f.dropGxClock = true;
    CHECK(!gm_do_reads(&r.io, &r.st));
    common(r);
    CHECK(r.f.gpuReads == 0 && !r.st.gpuRead && !strcmp(r.st.stop, "gx_gfx3d"));
  }
  {  // pure helpers
    CHECK(gm_branch_is_on(0x00000001, false));
    CHECK(!gm_branch_is_on(0x80000001, false));
    CHECK(gm_branch_is_on(0x20000001, false) && !gm_branch_is_on(0xA0000001, false));   // NOC FSM ON needs CLK_OFF clear
    CHECK(gm_branch_is_on(0x80000003, true) && !gm_branch_is_on(0x80000003, false));
    CHECK(GM_GMU_200M_CFG == 0x505);
    CHECK(GM_A6XX_CP_ALWAYS_ON_COUNTER_LO * 4 == 0x2600 && GM_A6XX_VBIF_VERSION * 4 == 0xc000);
    CHECK(GM_A6XX_VBIF_VERSION * 4 < GM_GPU_SIZE);
  }
  printf("gpu_host: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
