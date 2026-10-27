// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Adreno 610 milestone M1, included by NativeEmmc.cpp after the other services (mapPhysical, the boot log). A
// one-shot diagnostic on its own kernel thread, opt-in with boot argument guamp-gpu-m1=1 (default off):
//   power   the Linux order (GuampGpuM1.h): gcc cfg_ahb + GPLL0 vote, CX GDSC, the kgsl_smmu clocks (gcc memnoc,
//           gcc snoc_dvm, gpu_cc_ahb, smmu vote), GX GDSC (BLK_ARES pulse, CLAMP_IO off), gx_gfx3d_clk_src on XO
//           19.2 MHz, gmu_clk_src 200 MHz from GPLL0 / 3, the kgsl clocks (cx_gmu, iref, cxo, bimc_gpu_axi, gx/cx gfx3d)
//   read    only when both GDSCs report PWR_ON and every register-path clock reports CLK_OFF clear (re-read just
//           before): the GPU block 0x5900000 is mapped and VBIF_VERSION, RBBM_STATUS, CP_ALWAYS_ON_COUNTER_LO (twice,
//           must increase) are read. An unclocked GPU read aborts the bus, so any failure stops before the mapping.
//           guamp-gpu-relaxed=1: the BRANCH_HALT_DELAY / SKIP clocks (Linux never reads their status: gpu_cc_ahb,
//           gx_gfx3d, cx_gfx3d) pass the gate with their enable bit set even when CLK_OFF reads set.
// Strict scope: no SCM/SMC, no zap shader, no firmware, no command submission, no GPU register write, no
// RBBM_SW_RESET, no PLL programming. Writes stay limited to the GCC/GPUCC registers listed in kGmRegs.
// Other boot arguments: guamp-gpu-delay=<s> (start later, so the USB mirror sees the lines), guamp-gpu-off=1 (undo
// the power-up at the end; default: the GPU stays powered). guamp-gpu-rpm=1: before the GCC GPU branches, the RPM
// votes Linux holds for kgsl on bengal (GuampRpm.inc, kRpmGpuVotes): qdss Enab 1 (active + sleep), bimc / snoc / cnoc
// KHz (active) at guamp-gpu-rpm-khz (default RPM_KHZ_MAX: the RPM clamps to its top level, as rpmcc's probe vote for
// bimc; a low value would slow the DDR / eMMC path of the whole system), " rpm <state>" appended to RESULT.
// Log: before/after snapshots (3 lines each), one "after br" line per branch (enable bit, HW_CTL, CLK_OFF, NOC FSM,
// Linux halt type, gate/report, verdict, stuck/late), an optional stop and off line, and the single
// "GuampGpu: RESULT ..." line. Host-tested by tests/gpu_host.cpp.

#include "GuampGpuM1.h"
#include "GuampRpmCore.h"

namespace rpm {   // GuampRpm.inc
int set(uint32_t type, uint32_t id, uint32_t key, uint32_t value, unsigned which, unsigned timeoutMs);
}

namespace gpum1 {
volatile uint8_t *gBase[3];

uint32_t rd(void *, unsigned block, uint32_t off) {
  return *reinterpret_cast<volatile uint32_t *>(gBase[block] + off);
}
void wr(void *, unsigned block, uint32_t off, uint32_t v) {
  if (block == GM_GPU) return;   // never written in M1
  *reinterpret_cast<volatile uint32_t *>(gBase[block] + off) = v;
  __asm__ volatile("dsb sy" ::: "memory");
}
void udelay(void *, unsigned us) { IODelay(us); }
bool mapGpu(void *) {
  gBase[GM_GPU] = mapPhysical(GM_GPU_PHYS, GM_GPU_SIZE);
  return gBase[GM_GPU] != nullptr;
}

void logSnap(const char *when, const uint32_t *v, bool gpucc) {
  IOLog("GuampGpu: %s gcc vote %08x pll0 %08x cfg_ahb %08x memnoc %08x snoc_dvm %08x iref %08x axi %08x throttle %08x "
        "qmip %08x vote2 %08x\n", when, v[GMR_VOTE], v[GMR_PLL0], v[GMR_CFG_AHB], v[GMR_MEMNOC], v[GMR_SNOC_DVM], v[GMR_IREF],
        v[GMR_AXI], v[GMR_THROTTLE], v[GMR_QMIP], v[GMR_VOTE2]);
  if (!gpucc) {
    IOLog("GuampGpu: %s gpucc not read (gcc cfg_ahb %08x not on)\n", when, v[GMR_CFG_AHB]);
    return;
  }
  IOLog("GuampGpu: %s cx gdscr %08x hwctrl %08x ahb %08x cxo %08x gmu_rcg %08x/%08x cx_gmu %08x smmu %08x\n", when,
        v[GMR_CX_GDSCR], v[GMR_CX_HWCTRL], v[GMR_AHB], v[GMR_CXO], v[GMR_GMU_CMD], v[GMR_GMU_CFG], v[GMR_CX_GMU], v[GMR_SMMU]);
  IOLog("GuampGpu: %s gx bcr %08x clamp %08x gdscr %08x gfx_rcg %08x/%08x gx_gfx3d %08x cx_gfx3d %08x\n", when,
        v[GMR_GX_BCR], v[GMR_GX_CLAMP], v[GMR_GX_GDSCR], v[GMR_GFX_CMD], v[GMR_GFX_CFG], v[GMR_GX_GFX3D], v[GMR_CX_GFX3D]);
}

// One line per branch of kGmBranches (GPUCC ones only when GPUCC was readable).
void logBranches(const char *when, const uint32_t *v, const gm_state &st) {
  for (const gm_branch &b : kGmBranches) {
    if (kGmRegs[b.reg].block == GM_GPUCC && !st.gpuccOk) continue;
    const uint32_t x = v[b.reg], bit = 1u << b.reg;
    IOLog("GuampGpu: %s br %s %08x en %u hwctl %u clk_off %u fsm %u halt %s %s: %s%s%s\n", when, kGmRegs[b.reg].name, x,
          x & GM_CBCR_EN, (x & GM_CBCR_HWCG) >> 1, x >> 31, (x & GM_CBCR_FSM_MASK) >> 28, gm_halt_name(b.halt),
          b.gate ? "gate" : "report", gm_bstate_name(gm_branch_state(x, b)),
          st.late & bit ? " (late: on at the re-poll)" : st.stuck & bit ? " (stuck at enable)" : "",
          st.enabled & bit ? " (enabled here)" : "");
  }
}

gm_state gSt;

// guamp-gpu-rpm=1, before any GCC GPU branch is touched. Returns a short word for the RESULT line.
const char *rpmVotes() {
  unsigned khz = RPM_KHZ_MAX;
  PE_parse_boot_argn("guamp-gpu-rpm-khz", &khz, sizeof(khz));
  if (!khz) khz = RPM_KHZ_MAX;
  const uint32_t m0 = rd(nullptr, GM_GCC, GM_GCC_GPU_MEMNOC_GFX), s0 = rd(nullptr, GM_GCC, GM_GCC_GPU_SNOC_DVM_GFX),
                 a0 = rd(nullptr, GM_GCC, GM_GCC_BIMC_GPU_AXI);
  unsigned ok = 0;
  int first = RPM_OK;
  for (const rpm_vote &v : kRpmGpuVotes) {
    const uint32_t value = v.key == RPM_KEY_KHZ ? khz : 1u;
    int rc = rpm::set(v.type, v.id, v.key, value, RPM_SET_ACTIVE, 500);
    if (!rc && v.sleep) rc = rpm::set(v.type, v.id, v.key, value, RPM_SET_SLEEP, 500);
    if (!rc) ok = ok + 1;
    else if (first == RPM_OK) first = rc;
    if (rc == RPM_ERR_NOMAP || rc == RPM_ERR_TOC || rc == RPM_ERR_FIFO || rc == RPM_ERR_VERSION || rc == RPM_ERR_OPEN) break;
  }
  IODelay(100);
  IOLog("GuampGpu: rpm votes %u/4 (first error %s), KHz %u: gcc memnoc %08x->%08x snoc_dvm %08x->%08x axi %08x->%08x\n", ok,
        rpm_err_name(first), khz, m0, rd(nullptr, GM_GCC, GM_GCC_GPU_MEMNOC_GFX), s0, rd(nullptr, GM_GCC, GM_GCC_GPU_SNOC_DVM_GFX),
        a0, rd(nullptr, GM_GCC, GM_GCC_BIMC_GPU_AXI));
  return ok == 4 ? "ok" : ok ? "partial" : rpm_err_name(first);
}

void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-gpu-m1");
  unsigned delay = 0, off = 0, rpmOn = 0, relaxed = 0;
  PE_parse_boot_argn("guamp-gpu-delay", &delay, sizeof(delay));
  PE_parse_boot_argn("guamp-gpu-relaxed", &relaxed, sizeof(relaxed));
  PE_parse_boot_argn("guamp-gpu-off", &off, sizeof(off));
  PE_parse_boot_argn("guamp-gpu-rpm", &rpmOn, sizeof(rpmOn));
  if (delay && delay <= 600) IOSleep(delay * 1000);
  gm_state &st = gSt;
  gm_init(&st);
  st.relaxed = relaxed == 1;
  gBase[GM_GCC] = mapPhysical(GM_GCC_PHYS, GM_GCC_SIZE);
  gBase[GM_GPUCC] = mapPhysical(GM_GPUCC_PHYS, GM_GPUCC_SIZE);
  if (!gBase[GM_GCC] || !gBase[GM_GPUCC]) {
    IOLog("GuampGpu: RESULT cx - gx - clocks - vbif - status - aon - stop gcc/gpucc map failed\n");
    return;
  }
  const gm_io io = {nullptr, rd, wr, udelay, mapGpu};
  const char *rpmState = rpmOn == 1 ? rpmVotes() : nullptr;
  gm_power_up(&io, &st);
  const bool ready = gm_prepare_read(&io, &st);
  // Logged before the first GPU read: if that read still aborts the bus, the kpan mirror (every second) and the panic
  // copy keep the snapshots.
  logSnap("before", st.before, st.gpuccOk);
  logSnap("after", st.after, st.gpuccOk);
  logBranches("after", st.after, st);
  if (ready) {
    IOSleep(1500);
    gm_do_reads(&io, &st);
  }
  if (st.stop) IOLog("GuampGpu: stopped at %s, GPU block %s\n", st.stop, st.mapped ? "mapped, not read" : "not touched");
  if (off == 1) {
    const bool down = gm_power_down(&io, &st);
    IOLog("GuampGpu: off (guamp-gpu-off=1) gx %s gx_gdscr %08x clamp %08x cx_gdscr %08x hwctrl %08x vote %08x\n",
          down ? "collapsed" : "PWR_ON still set", st.off[GMR_GX_GDSCR], st.off[GMR_GX_CLAMP], st.off[GMR_CX_GDSCR],
          st.off[GMR_CX_HWCTRL], st.off[GMR_VOTE]);
  }
  if (st.gpuRead)
    IOLog("GuampGpu: RESULT cx %s gx %s clocks %s gmu %s gfx %s vbif %08x status %08x aon %08x->%08x %s%s%s%s%s\n", st.cx, st.gx,
          st.clocks, st.gmu, st.gfx, st.vbif, st.status, st.aon0, st.aon1, st.aonOk ? "ok" : "stuck",
          off == 1 ? " (powered off)" : " (left powered)", rpmState ? " rpm " : "", rpmState ? rpmState : "",
          st.relaxed ? " relaxed" : "");
  else
    IOLog("GuampGpu: RESULT cx %s gx %s clocks %s gmu %s gfx %s vbif - status - aon - stop %s%s%s%s\n", st.cx, st.gx,
          st.clocks, st.gmu, st.gfx, st.stop ? st.stop : "?", rpmState ? " rpm " : "", rpmState ? rpmState : "",
          st.relaxed ? " relaxed" : "");
}

void start() {
  unsigned on = 0;
  if (!PE_parse_boot_argn("guamp-gpu-m1", &on, sizeof(on)) || on != 1) return;
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}
}  // namespace gpum1
