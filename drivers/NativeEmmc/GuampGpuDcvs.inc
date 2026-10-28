// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Adreno 610 GPU frequency scaling, included by GpuService.inc below gpug2. On by default since 28/10 (owner request,
// 320..900 MHz verified with 0 failures); guamp-gpu-dcvs=0 keeps the guamp-gpu-freq rate (320 MHz) and nothing here runs. Started by the
// GPU thread once NativeAdreno is up with the core clock on PLL1 (guamp-gpu-freq 320 or 465), then on its own thread:
//   limits  speed bin from the qfprom fuse (gpu_speed_bin, GuampGpuPower.h gp_speed_bin / gp_bin_top: bins 0 and 206
//           reach 980 MHz, the top of bengal-gpu.dtsi; tb128fu's 1114.8 MHz is a khaje level this SoC does not have) or
//           guamp-gpu-bin=<n> (no fuse read); guamp-gpu-max=<MHz> caps it (465: PLL1 only, the PLL already verified).
//   votes   the RPM rail levels kgsl's vdd_cx / vdd_mx regulators send on bengal ("rwcx" / "rwmx" id 0, "vlvl",
//           active set): "swen" and the start level once, then gp_set_level raises CX (and MX for the PLL0 levels)
//           before the clock and lowers them after it. Floors guamp-gpu-cx-min=<level> (default 64, LOW_SVS: what
//           guamp-gpu-freq already votes) and guamp-gpu-mx-min=<level> (default 48, MIN_SVS): nothing else here votes
//           these rails for the rest of the SoC. The GPU bus vote is left alone (guamp-gpu-rpm's bimc vote holds DDR
//           for the whole system).
//   sample  RBBM_STATUS every millisecond, one decision per 50 ms window (gp_gov_window, the tb128fu governor).
//   fail    a switch that fails (rail not acked, PLL no lock, RCG stuck) never tries that level or a faster one
//           again and puts the previous level back (gp_gov_failed); after GP_GOV_MAX_FAILS failures in a row the GPU
//           goes to 320 MHz and the thread ends.
//   log     "GuampGpu: dcvs ..." start line, every switch failure, every 10 s the level, switches, failures, the busy
//           share of the last window and the ms spent at each level.
// guamp_gpu_dcvs_hold != 0 (a GPU recovery in progress, for instance) pauses sampling and switching until it is 0.

extern "C" {
volatile uint32_t guamp_gpu_dcvs_hold;
}

namespace gpudcvs {
volatile uint8_t *gCc, *gGpu;
gp_state gS;
gp_gov gG;
uint32_t gCxMin = GP_RPM_LOW_SVS, gMxMin = GP_RPM_MIN_SVS;

uint32_t rd(void *, unsigned block, uint32_t off) {
  return *reinterpret_cast<volatile uint32_t *>((block == GG2_GPUCC ? gCc : gGpu) + off);
}
void wr(void *, unsigned block, uint32_t off, uint32_t v) {   // GPUCC only: gp_set_level writes no GPU register
  *reinterpret_cast<volatile uint32_t *>((block == GG2_GPUCC ? gCc : gGpu) + off) = v;
  __asm__ volatile("dsb sy" ::: "memory");
}
void udelay(void *, unsigned us) { IODelay(us); }
const gg2_io kIo = {nullptr, rd, wr, udelay};

int corner(void *, unsigned rail, uint32_t level) {
  const uint32_t floor = rail == GP_RAIL_CX ? gCxMin : gMxMin;
  return rpm::set(rail == GP_RAIL_CX ? GP_RPM_RES_RWCX : GP_RPM_RES_RWMX, 0, GP_RPM_KEY_VLVL, level < floor ? floor : level,
                  RPM_SET_ACTIVE, 500);
}
int bus(void *, uint32_t) { return 0; }
// No nap / slumber here: the GDSC, restart and SMMU callbacks are never reached.
bool gdsc(void *, bool, bool) { return false; }
bool none(void *) { return false; }
const gp_ops kOps = {nullptr, corner, bus, gdsc, none, none};

void logState(unsigned busy, unsigned switches, unsigned fails, const uint32_t *ms) {
  IOLog("GuampGpu: dcvs now %u MHz top %u | sw %u fail %u | busy %u%% | ms 980:%u 900:%u 820:%u 745:%u 600:%u 465:%u "
        "320:%u xo:%u\n", kGpLevels[gS.level].mhz, kGpLevels[gS.top].mhz, switches, fails, busy * 100u / GP_GOV_WINDOW,
        ms[0], ms[1], ms[2], ms[3], ms[4], ms[5], ms[6], ms[7]);
}

void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-gpu-dcvs");
  uint32_t ms[GP_NLEVELS] = {};
  unsigned switches = 0, fails = 0, run = 0, busy = 0;
  for (unsigned window = 1;; ++window) {
    busy = 0;
    bool held = false;
    for (unsigned i = 0; i < GP_GOV_WINDOW; ++i) {
      if (guamp_gpu_dcvs_hold) {
        held = true;
        break;
      }
      if (gp_gov_busy(gg2_gr(&kIo, GP_RBBM_STATUS))) ++busy;
      IOSleep(1);
    }
    if (held) {
      IOSleep(GP_GOV_WINDOW);
      continue;
    }
    const unsigned prev = gS.level, target = gp_gov_window(&gG, &gS, busy);
    if (target != prev) {
      const int e = gp_set_level(&kIo, &kOps, &gS, target);
      if (!e) {
        ++switches;
        run = 0;
      } else {
        ++fails;
        ++run;
        const unsigned back = gp_gov_failed(&gS, target, prev);
        const int e2 = gS.level == back ? GP_OK : gp_set_level(&kIo, &kOps, &gS, back);
        IOLog("GuampGpu: dcvs %u -> %u MHz failed (%s), top now %u MHz, back to %u MHz %s\n", kGpLevels[prev].mhz,
              kGpLevels[target].mhz, gp_err_name(e), kGpLevels[gS.top].mhz, kGpLevels[back].mhz,
              e2 ? gp_err_name(e2) : "ok");
        if (run >= GP_GOV_MAX_FAILS) {
          const int e3 = gS.level == GP_LEVEL_320 ? GP_OK : gp_set_level(&kIo, &kOps, &gS, GP_LEVEL_320);
          IOLog("GuampGpu: dcvs stopped after %u failures in a row, GPU at %u MHz (%s)\n", run, kGpLevels[gS.level].mhz,
                gp_err_name(e3));
          logState(busy, switches, fails, ms);
          return;
        }
      }
    }
    ms[gS.level] += GP_GOV_WINDOW;
    if (window % 200 == 0) logState(busy, switches, fails, ms);
  }
}

// After NativeAdreno started with the core clock at mhz (gpug2::setFreq): checks the RCG really runs from PLL1 / 2,
// reads the limits, takes the rails at the current level and starts the governor thread.
void start(volatile uint8_t *gpucc, volatile uint8_t *gpu, unsigned mhz) {
  unsigned on = 1;   // default on (28/10); the speed-bin fuse caps the top level (900 MHz on the owner's phone)
  PE_parse_boot_argn("guamp-gpu-dcvs", &on, sizeof(on));
  if (on != 1) return;
  gCc = gpucc;
  gGpu = gpu;
  if (!gCc || !gGpu) return;
  unsigned level = GP_NLEVELS;
  for (unsigned i = 0; i < GP_NLEVELS; ++i)
    if (kGpLevels[i].mhz == mhz && kGpLevels[i].pll == GP_PLL1) level = i;
  const uint32_t cfg = gg2_cr(&kIo, GG2_GFX_CFG);
  if (level == GP_NLEVELS || ((cfg >> 8) & 7u) != GG2_SRC_PLL1_AUX || (cfg & 0x1fu) != 3u) {
    IOLog("GuampGpu: dcvs off: core clock not on PLL1 at 320 / 465 MHz (guamp-gpu-freq %u, gfx cfg %08x)\n", mhz, cfg);
    return;
  }
  uint32_t bin = 0;
  const char *binFrom = "guamp-gpu-bin";
  if (!PE_parse_boot_argn("guamp-gpu-bin", &bin, sizeof(bin))) {
    volatile uint8_t *q = mapPhysical(GP_QFPROM_PHYS + (GP_SPEED_BIN_DWORD & ~0xfffu), 0x1000);
    if (!q) {
      IOLog("GuampGpu: dcvs off: qfprom map failed (guamp-gpu-bin=<n> skips the fuse)\n");
      return;
    }
    bin = gp_speed_bin(*reinterpret_cast<volatile uint32_t *>(q + (GP_SPEED_BIN_DWORD & 0xfffu)));
    binFrom = "fuse";
  }
  unsigned cap = 0;
  PE_parse_boot_argn("guamp-gpu-max", &cap, sizeof(cap));
  PE_parse_boot_argn("guamp-gpu-cx-min", &gCxMin, sizeof(gCxMin));
  PE_parse_boot_argn("guamp-gpu-mx-min", &gMxMin, sizeof(gMxMin));
  gS = gp_state();
  gS.level = level;
  gS.pllOn = GP_PLL1;
  gS.top = gp_gov_top(bin, cap);
  gS.bottom = GP_LEVEL_320;
  gS.busKhz = gp_bus_bimc_khz(kGpLevels[level].bus);
  if (gS.top == GP_LEVEL_320 && level == GP_LEVEL_320) {
    IOLog("GuampGpu: dcvs off: nothing above 320 MHz (bin %u from %s, guamp-gpu-max %u)\n", bin, binFrom, cap);
    return;
  }
  // The rails at the current level before any switch: gp_set_level only votes what changes.
  int cx = rpm::set(GP_RPM_RES_RWCX, 0, RPM_KEY_SWEN, 1, RPM_SET_ACTIVE, 500);
  if (!cx) cx = corner(nullptr, GP_RAIL_CX, kGpLevels[level].cx);
  int mx = cx ? cx : rpm::set(GP_RPM_RES_RWMX, 0, RPM_KEY_SWEN, 1, RPM_SET_ACTIVE, 500);
  if (!mx) mx = corner(nullptr, GP_RAIL_MX, kGpLevels[level].mx);
  if (cx || mx) {
    IOLog("GuampGpu: dcvs off: rail vote not acked (cx %s, mx %s)\n", rpm_err_name(cx), cx ? "-" : rpm_err_name(mx));
    return;
  }
  gS.cx = kGpLevels[level].cx;
  gS.mx = kGpLevels[level].mx;
  IOLog("GuampGpu: dcvs on: %u MHz, levels %u..%u MHz (bin %u from %s, guamp-gpu-max %u), cx floor %u mx floor %u\n",
        kGpLevels[level].mhz, kGpLevels[gS.bottom].mhz, kGpLevels[gS.top].mhz, bin, binFrom, cap, gCxMin, gMxMin);
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}
}  // namespace gpudcvs
