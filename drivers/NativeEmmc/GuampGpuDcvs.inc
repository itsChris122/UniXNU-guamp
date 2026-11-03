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
// Interactive tuning (03/11; fin34/fin35: the display waited 7-54 ms a frame for the GPU while the busy share stayed under 30 %, so
// the 70 % rule kept the GPU at 320 MHz for most frames; 950 MHz verified with 0 failed switches in 106):
//   guamp-gpu-boost=1  wait boost: a CPU thread blocked on a GPU fence (NativeAdreno's fence waits, the display's guamp-copy-gpu-wait:
//                      GuampAdrenoProbe.cpp sets guamp_gpu_dcvs_boost) ends the window at once and sends the GPU to the top; the steps
//                      down are the usual ones (500 ms of near idle per level). Default 0 (off).
//   guamp-gpu-up=<%>   busy share of a window that sends the GPU to the top (default 70).
//   guamp-gpu-min=<MHz> the slowest level the governor goes down to (default 0: 320 MHz).
//   One line "GuampGpu: dcvs tuning ..." at start; the 10 s line gains "boosts N".

extern "C" {
volatile uint32_t guamp_gpu_dcvs_hold;
volatile uint32_t guamp_gpu_dcvs_boost;   // set by a blocked GPU fence wait, cleared here
// patches/native-adreno/0010 (weak stand-ins for a kernel without it): the submissions' cost and the mappings their maintenance skipped.
__attribute__((weak)) volatile uint64_t native_adreno_submit_cost[8];
__attribute__((weak)) volatile uint64_t native_adreno_publish_skipped[2];
// patches/native-adreno/0011: the maintained bytes by kind (imported, write-combined while distrusted, other owned; [3] distrusted).
__attribute__((weak)) volatile uint64_t native_adreno_publish_kinds[4];
// patches/native-adreno/0012: imports left to their client ({mappings, bytes}), the BoMap flags honoured / not, the re-assertion's
// counters (NativeAdrenoSubmit.cpp: [0] submissions that re-asserted, [1] with none to do, [2] whole checks, [3] pages a check found the
// event count missed, [4] BOs re-asserted, [5] fallen back to every submission) and the event count itself.
__attribute__((weak)) volatile uint64_t native_adreno_publish_client[2];
__attribute__((weak)) volatile uint32_t native_adreno_client_cleans_maps[2];
__attribute__((weak)) volatile uint64_t native_adreno_reassert_stats[6];
__attribute__((weak)) volatile uint32_t native_adreno_wc_epoch;
}

namespace gpudcvs {
volatile uint8_t *gCc, *gGpu;
gp_state gS;
gp_gov gG;
uint32_t gCxMin = GP_RPM_LOW_SVS, gMxMin = GP_RPM_MIN_SVS;
unsigned gBoostOn = 0, gBoosts = 0;

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
  IOLog("GuampGpu: dcvs now %u MHz top %u | sw %u fail %u | busy %u%% | ms 980:%u 950:%u 900:%u 820:%u 745:%u 600:%u "
        "465:%u 320:%u xo:%u | boosts %u\n", kGpLevels[gS.level].mhz, kGpLevels[gS.top].mhz, switches, fails,
        busy * 100u / GP_GOV_WINDOW, ms[0], ms[1], ms[2], ms[3], ms[4], ms[5], ms[6], ms[7], ms[8], gBoosts);
}

// Every 10 s with the level line: the GPU submissions since the previous line, their mean time in the kernel and in its cache
// maintenance, the MiB maintained a submission, the longest of each so far, and the mappings outside the submitter's window left alone.
void logSubmits() {
  static uint64_t last[8];
  const uint64_t n = native_adreno_submit_cost[0], us = native_adreno_submit_cost[1], pub = native_adreno_submit_cost[2],
                 kib = native_adreno_submit_cost[3];
  const uint64_t dn = n - last[0];
  if (dn)
    IOLog("GuampGpu: submits +%llu, kernel %llu us each (maintenance %llu us, %llu KiB each), longest %llu / %llu us; skipped other "
          "windows %llu mappings %llu MiB in all\n", dn, (us - last[1]) / dn, (pub - last[2]) / dn, (kib - last[3]) / dn,
          native_adreno_submit_cost[4], native_adreno_submit_cost[5], native_adreno_publish_skipped[0], native_adreno_publish_skipped[1] >> 20);
  last[0] = n; last[1] = us; last[2] = pub; last[3] = kib;
  // patch 0011: per submission, the KiB maintained of each kind and the write-combine re-assertion's us.
  const uint64_t imp = native_adreno_publish_kinds[0], wc = native_adreno_publish_kinds[1], own = native_adreno_publish_kinds[2],
                 re = native_adreno_submit_cost[6];
  if (dn)
    IOLog("GuampGpu: submit maintenance KiB each: imported %llu, write-combined %llu (distrusted %llu), owned %llu; re-assert %llu us each\n",
          (imp - last[4]) / dn >> 10, (wc - last[5]) / dn >> 10, native_adreno_publish_kinds[3], (own - last[6]) / dn >> 10,
          (re - last[7]) / dn);
  last[4] = imp; last[5] = wc; last[6] = own; last[7] = re;
  // patch 0012: per submission, the imports left to their client (their KiB), and how often the re-assertion had work.
  static uint64_t prev[6];
  const uint64_t cm = native_adreno_publish_client[0], cb = native_adreno_publish_client[1], rw = native_adreno_reassert_stats[0],
                 rn = native_adreno_reassert_stats[1], rc = native_adreno_reassert_stats[2], rb = native_adreno_reassert_stats[4];
  if (dn)
    IOLog("GuampGpu: submit coherence: client-cleaned imports %llu mappings %llu KiB each (maps flagged %u, refused %u); re-assert "
          "ran %llu skipped %llu checks %llu BOs %llu, missed %llu pages, fallback %llu, events %u\n",
          (cm - prev[0]) / dn, (cb - prev[1]) / dn >> 10, native_adreno_client_cleans_maps[0], native_adreno_client_cleans_maps[1],
          rw - prev[2], rn - prev[3], rc - prev[4], rb - prev[5], native_adreno_reassert_stats[3], native_adreno_reassert_stats[5],
          native_adreno_wc_epoch);
  prev[0] = cm; prev[1] = cb; prev[2] = rw; prev[3] = rn; prev[4] = rc; prev[5] = rb;
}

void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-gpu-dcvs");
  uint32_t ms[GP_NLEVELS] = {};
  unsigned switches = 0, fails = 0, run = 0, busy = 0;
  for (unsigned window = 1;; ++window) {
    busy = 0;
    bool held = false, boosted = false;
    unsigned sampled = 0;
    for (unsigned i = 0; i < GP_GOV_WINDOW; ++i) {
      if (guamp_gpu_dcvs_hold) {
        held = true;
        break;
      }
      if (gBoostOn && guamp_gpu_dcvs_boost) {
        guamp_gpu_dcvs_boost = 0;
        if (gS.level != gS.top) {
          boosted = true;
          break;
        }
        gG.quiet = 0;   // already at the top: waits on the GPU keep it there (no step down while they go on)
      }
      if (gp_gov_busy(gg2_gr(&kIo, GP_RBBM_STATUS))) ++busy;
      ++sampled;
      IOSleep(1);
    }
    if (held) {
      IOSleep(GP_GOV_WINDOW);
      continue;
    }
    if (boosted) ++gBoosts;
    const unsigned prev = gS.level, target = boosted ? gp_gov_boost(&gG, &gS) : gp_gov_window(&gG, &gS, busy);
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
    ms[gS.level] += boosted ? sampled : GP_GOV_WINDOW;
    if (window % 200 == 0) {
      logState(busy, switches, fails, ms);
      logSubmits();
    }
  }
}

// After NativeAdreno started with the core clock at mhz (gpug2::setFreq): checks the RCG really runs from PLL1 / 2,
// reads the limits, takes the rails at the current level and starts the governor thread.
void start(volatile uint8_t *gpucc, volatile uint8_t *gpu, unsigned mhz) {
  unsigned on = 1;   // default on (28/10); the speed-bin fuse caps the top level (950 MHz on the owner's phone, bin 200)
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
  unsigned up = 0, minMhz = 0;
  PE_parse_boot_argn("guamp-gpu-boost", &gBoostOn, sizeof(gBoostOn));
  PE_parse_boot_argn("guamp-gpu-up", &up, sizeof(up));
  PE_parse_boot_argn("guamp-gpu-min", &minMhz, sizeof(minMhz));
  gBoostOn = gBoostOn == 1;
  gG = gp_gov();
  gG.up = gp_gov_up_samples(up);
  gS.bottom = gp_gov_bottom(gS.top, minMhz);
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
  IOLog("GuampGpu: dcvs tuning: boost %u, up %u/%u samples, min %u MHz (bottom %u MHz)\n", gBoostOn, gG.up, GP_GOV_WINDOW, minMhz,
        kGpLevels[gS.bottom].mhz);
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}
}  // namespace gpudcvs
