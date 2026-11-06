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
//                      down are the usual ones (500 ms of near idle per level). Default 1 since 06/11 (every test since fin38 passed
//                      guamp-gpu-boost=1, 0 failed switches); guamp-gpu-boost=0 turns it off.
//   guamp-gpu-up=<%>   busy share of a window that sends the GPU to the top (default 25 since 06/11, what fin38..fin50 passed;
//                      it was 70: guamp-gpu-up=70 gives the old rule back).
//   guamp-gpu-min=<MHz> the slowest level the governor goes down to (default 0: 320 MHz).
//   One line "GuampGpu: dcvs tuning ..." at start; the 10 s line gains "boosts N".
// Sampling cost (07/11, perf-cpu; fin53 live-20261106-092610: ~880 samples/s for good = ~880 timer wake-ups/s of a kernel
// thread, one of the main sources of the ~760 timer interrupts/s on every Gold core with the screen still):
//   guamp-gpu-dcvs-step=<1..10>  ms between two RBBM_STATUS reads in a window with GPU work (default 1, the old rate);
//   guamp-gpu-dcvs-idle=<1..25>  ms between reads in the window after one that saw no busy sample (default = step).
//   Each read stands for the ms slept after it (busy, samples and the MHz x samples sums stay in ms), so the window rules,
//   the up share and the 10 s line keep their meaning. A boost or wake request is seen at the next read (<= step / idle ms).
//   guamp-perfmode=1 sets 4 and 10.
// What it sees (06/11, GuampGpuPower.h gp_gov_kind / gp_meas): every 10 s one more line "GuampGpu: dcvs real ..." with the core
// clock measured at each level (CP perfcounter 0 against CP_ALWAYS_ON_COUNTER over fully busy windows; 0 = no such window yet)
// and the samples of the last 10 s split into work (a pipe or memory unit busy), cp-only (only the CP bits: a CP that waits
// on a fence or for idle counts as busy for the governor) and idle. A level whose measured clock is more than 5 % off logs
// "GuampGpu: dcvs level N MHz measures M MHz" once.

extern "C" {
volatile uint32_t guamp_gpu_dcvs_hold;
volatile uint32_t guamp_gpu_dcvs_boost;   // set by a blocked GPU fence wait, cleared here
// GuampBacklight.inc sets it when the display comes on (standby -> awake): the top for guamp-gpu-wake-ms (default 1500, 0 off) whatever
// the busy share, then the usual steps down. fin45: after the wake the GPU sat at 320 MHz while the wallpaper and the render server's
// surfaces were drawn again, and only went to 900 MHz later (wallpaper late).
volatile uint32_t guamp_gpu_dcvs_wake;
// guamp-gpu-perf (GuampAdreno/GuampGpuPerf.inc): the GPU clock now in MHz (also set from guamp-gpu-freq when the governor is off),
// and since the start [0] busy samples, [1] samples, [2] MHz x samples (1 ms each), for the busy share and the mean clock.
volatile uint32_t guamp_gpu_mhz_now;
volatile uint64_t guamp_gpu_dcvs_busy[3];
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
// patches/native-adreno/0016: cache-mode events that named their BO (only that BO is re-asserted); native_adreno_wc_epoch then counts
// the unnamed ones (imports, non-BO views), which reach only BOs with a CPU view.
__attribute__((weak)) volatile uint64_t native_adreno_wc_bo_events;
// patches/native-adreno/0017: the soft priority's waits ([0] submissions that waited, [1] to the limit, [2] us, [3] longest us) and
// the render server's last fence.
__attribute__((weak)) volatile uint64_t native_adreno_throttle_stats[4];
__attribute__((weak)) volatile uint32_t native_adreno_compositor_fence;
// patches/native-adreno/0019: job slots kept for the render server ([0] app waits, [1] app refusals, [2] app us, [3] render server
// waits, [4] render server refusals, [5] render server us, [6] render server longest us, [7] app longest us).
__attribute__((weak)) volatile uint64_t native_adreno_slot_stats[8];
// GuampAdrenoProbe.cpp guamp-gpu-idle-us: fence-wait sleeps {slept, woken by the GPU interrupt, by the backstop, not slept}.
__attribute__((weak)) volatile uint64_t native_gpu_idle_stats[4];
__attribute__((weak)) volatile uint64_t native_gpu_idle_missed;   // backstops with a fence retired and no interrupt (07/11)
// patches/native-adreno/0013: read-only imports left out of a submission's maintenance {mappings, bytes, sweeps}.
__attribute__((weak)) volatile uint64_t native_adreno_publish_ro_owned[3];
// patches/native-adreno/0019: the re-assertion's pages ([0] checked, [1] found reset and written back, [2] BO passes) and its time
// (mach_absolute_time ticks, NativeAdrenoCpu.cpp), to tell the page checks from the pmap's resets.
__attribute__((weak)) volatile uint64_t native_adreno_reassert_pages[3];
__attribute__((weak)) volatile uint64_t native_adreno_reassert_ticks;
// patches/native-adreno/0021: GPU-written imports cleaned by physical address ([0] passes after a job, [1] mappings, [2] bytes, [3-5] the
// same before a job, [6] armed mappings left over a pass's cap, [7] mappings past their budget, [8] their bytes, [9] render server jobs
// left out, [10] us in the passes, [11] the longest pass in us).
__attribute__((weak)) volatile uint64_t native_adreno_written_stats[12];
}

namespace gpudcvs {
volatile uint8_t *gCc, *gGpu;
gp_state gS;
gp_gov gG;
uint32_t gCxMin = GP_RPM_LOW_SVS, gMxMin = GP_RPM_MIN_SVS;
unsigned gBoostOn = 1, gBoosts = 0, gWakeMs = 1500, gWakes = 0;
unsigned gStepMs = 1, gIdleStepMs = 0;   // guamp-gpu-dcvs-step / -idle (0: same as the step)
unsigned gWakeWindows = 0;   // governor windows the top is still held for after a display wake
gp_meas gM;                  // measured core clock per level
uint32_t gOffLogged = 0;     // levels already reported as off

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
        "465:%u 320:%u xo:%u | boosts %u wakes %u\n", kGpLevels[gS.level].mhz, kGpLevels[gS.top].mhz, switches, fails,
        busy * 100u / GP_GOV_WINDOW, ms[0], ms[1], ms[2], ms[3], ms[4], ms[5], ms[6], ms[7], ms[8], gBoosts, gWakes);
}

// The measured clock per level (0 when no fully busy window ran there) and the sample split since the previous line.
void logReal(const uint32_t *kinds) {
  uint32_t m[GP_NLEVELS];
  for (unsigned i = 0; i < GP_NLEVELS; ++i) m[i] = gp_meas_mhz(&gM, i);
  IOLog("GuampGpu: dcvs real MHz 980:%u 950:%u 900:%u 820:%u 745:%u 600:%u 465:%u 320:%u | samples work %u cp-only %u idle %u\n",
        m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], kinds[GP_SAMPLE_WORK], kinds[GP_SAMPLE_CP], kinds[GP_SAMPLE_IDLE]);
}

// CP perfcounter 0 on PERF_CP_ALWAYS_COUNT (NativeAdreno start sets it; a GPU recovery may have reset it).
void measSelect() {
  gg2_gw(&kIo, GG2_RBBM_PERFCTR_CNTL, 1);
  gg2_gw(&kIo, GG2_CP_PERFCTR_CP_SEL_0, 0);
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
          "ran %llu skipped %llu checks %llu BOs %llu, missed %llu pages, fallback %llu, events %u, BO events %llu\n",
          (cm - prev[0]) / dn, (cb - prev[1]) / dn >> 10, native_adreno_client_cleans_maps[0], native_adreno_client_cleans_maps[1],
          rw - prev[2], rn - prev[3], rc - prev[4], rb - prev[5], native_adreno_reassert_stats[3], native_adreno_reassert_stats[5],
          native_adreno_wc_epoch, native_adreno_wc_bo_events);
  prev[0] = cm; prev[1] = cb; prev[2] = rw; prev[3] = rn; prev[4] = rc; prev[5] = rb;
  // patch 0019: per submission, the pages the re-assertion checked and reset, and the us of its page passes alone (the rest of
  // "re-assert us each" is the walk over the mappings).
  static uint64_t rp[4];
  const uint64_t pc = native_adreno_reassert_pages[0], pr = native_adreno_reassert_pages[1], pp = native_adreno_reassert_pages[2],
                 pt = native_adreno_reassert_ticks;
  if (dn) {
    uint64_t ns = 0;
    absolutetime_to_nanoseconds(pt - rp[3], &ns);
    IOLog("GuampGpu: re-assert pages each: checked %llu, reset %llu; BO passes %llu, page passes %llu us each\n", (pc - rp[0]) / dn,
          (pr - rp[1]) / dn, pp - rp[2], ns / 1000 / dn);
  }
  rp[0] = pc; rp[1] = pr; rp[2] = pp; rp[3] = pt;
  // patch 0017: app submissions held back so the render server's jobs do not queue behind their backlog.
  static uint64_t th[3];
  const uint64_t tw = native_adreno_throttle_stats[0], tl = native_adreno_throttle_stats[1], tu = native_adreno_throttle_stats[2];
  if (tw != th[0])
    IOLog("GuampGpu: app submissions held +%llu (to the limit +%llu), %llu us each, longest %llu us; render server fence %u\n",
          tw - th[0], tl - th[1], (tu - th[2]) / (tw - th[0]), native_adreno_throttle_stats[3], native_adreno_compositor_fence);
  th[0] = tw; th[1] = tl; th[2] = tu;
  // patch 0019: waits for a job slot. A render server refusal is what set backboardd's device lost in fin50 (fence 462 frozen).
  static uint64_t sl[6];
  bool slots_moved = false;
  for (unsigned i = 0; i < 6; ++i) slots_moved |= native_adreno_slot_stats[i] != sl[i];
  if (slots_moved)
    IOLog("GuampGpu: job slots: apps waited +%llu (refused +%llu) %llu us each, longest %llu us; render server waited +%llu "
          "(refused +%llu) %llu us each, longest %llu us\n", native_adreno_slot_stats[0] - sl[0], native_adreno_slot_stats[1] - sl[1],
          native_adreno_slot_stats[0] != sl[0] ? (native_adreno_slot_stats[2] - sl[2]) / (native_adreno_slot_stats[0] - sl[0]) : 0,
          native_adreno_slot_stats[7], native_adreno_slot_stats[3] - sl[3], native_adreno_slot_stats[4] - sl[4],
          native_adreno_slot_stats[3] != sl[3] ? (native_adreno_slot_stats[5] - sl[5]) / (native_adreno_slot_stats[3] - sl[3]) : 0,
          native_adreno_slot_stats[6]);
  for (unsigned i = 0; i < 6; ++i) sl[i] = native_adreno_slot_stats[i];
  // guamp-gpu-idle-us: the fence waits' sleeps (each one a CPU wakeup on the waiting thread; fin51 backboardd 293 wakes / s).
  static uint64_t gs[4], gm;
  if (native_gpu_idle_stats[0] != gs[0] || native_gpu_idle_stats[3] != gs[3])
    IOLog("GuampGpu: fence sleeps +%llu (GPU interrupt +%llu, backstop +%llu of which fence retired without interrupt +%llu), "
          "not slept +%llu\n", native_gpu_idle_stats[0] - gs[0], native_gpu_idle_stats[1] - gs[1], native_gpu_idle_stats[2] - gs[2],
          native_gpu_idle_missed - gm, native_gpu_idle_stats[3] - gs[3]);
  for (unsigned i = 0; i < 4; ++i) gs[i] = native_gpu_idle_stats[i];
  gm = native_gpu_idle_missed;
  // patch 0013: per submission, the read-only imports left to their map-time clean, and the sweeps that cleaned them anyway.
  static uint64_t ro[3];
  const uint64_t om = native_adreno_publish_ro_owned[0], ob = native_adreno_publish_ro_owned[1], os = native_adreno_publish_ro_owned[2];
  if (dn)
    IOLog("GuampGpu: read-only imports left to their map clean: %llu mappings %llu KiB each, sweeps %llu\n", (om - ro[0]) / dn,
          (ob - ro[1]) / dn >> 10, os - ro[2]);
  ro[0] = om; ro[1] = ob; ro[2] = os;
  // patch 0021: the GPU-written imports cleaned and invalidated by physical address (app switcher snapshots), what it cost, what it left.
  static uint64_t wr[12];
  const volatile uint64_t *w = native_adreno_written_stats;
  if (w[0] != wr[0] || w[3] != wr[3] || w[9] != wr[9] || w[7] != wr[7]) {
    const uint64_t passes = w[0] - wr[0] + w[3] - wr[3];
    IOLog("GuampGpu: written imports by PA: after jobs %llu passes %llu maps %llu KiB, before jobs %llu passes %llu maps %llu KiB; "
          "%llu us each (longest %llu), %llu KiB a submission; past budget %llu maps %llu MiB, over the cap %llu; render server jobs "
          "left out %llu\n", w[0] - wr[0], w[1] - wr[1], (w[2] - wr[2]) >> 10, w[3] - wr[3], w[4] - wr[4], (w[5] - wr[5]) >> 10,
          passes ? (w[10] - wr[10]) / passes : 0, w[11], dn ? ((w[2] - wr[2]) + (w[5] - wr[5])) / dn >> 10 : 0, w[7] - wr[7],
          (w[8] - wr[8]) >> 20, w[6] - wr[6], w[9] - wr[9]);
  }
  for (unsigned i = 0; i < 12; ++i) wr[i] = w[i];
}

void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-gpu-dcvs");
  uint32_t ms[GP_NLEVELS] = {};
  unsigned switches = 0, fails = 0, run = 0, busy = 0;
  uint32_t kinds[3] = {};
  measSelect();
  bool quietWindow = false;   // the previous window saw no busy sample: guamp-gpu-dcvs-idle
  for (unsigned window = 1;; ++window) {
    const unsigned step = quietWindow ? gIdleStepMs : gStepMs;
    busy = 0;
    bool held = false, boosted = false;
    unsigned sampled = 0;
    const unsigned ran = gS.level;
    if (!guamp_gpu_dcvs_hold)   // no GPU read while a recovery holds the governor (the loop below checks it the same way)
      gp_meas_begin(&gM, gg2_gr(&kIo, GG2_RBBM_PERFCTR_CP_0_LO), gg2_gr(&kIo, GG2_CP_ALWAYS_ON_COUNTER_LO));
    else
      gM.primed = false;
    for (unsigned i = 0; i < GP_GOV_WINDOW;) {
      if (guamp_gpu_dcvs_hold) {
        held = true;
        break;
      }
      if (gWakeMs && guamp_gpu_dcvs_wake) {   // the display came on: straight to the top and held there
        guamp_gpu_dcvs_wake = 0;
        gWakeWindows = (gWakeMs + GP_GOV_WINDOW - 1) / GP_GOV_WINDOW;
        gWakes = gWakes + 1;
        if (gS.level != gS.top) {
          boosted = true;
          break;
        }
      }
      if (gBoostOn && guamp_gpu_dcvs_boost) {
        guamp_gpu_dcvs_boost = 0;
        if (gS.level != gS.top) {
          boosted = true;
          break;
        }
        gG.quiet = 0;   // already at the top: waits on the GPU keep it there (no step down while they go on)
      }
      const gp_sample_kind k = gp_gov_kind(gg2_gr(&kIo, GP_RBBM_STATUS));
      const unsigned w = GP_GOV_WINDOW - i < step ? GP_GOV_WINDOW - i : step;   // ms this read stands for
      if (k != GP_SAMPLE_IDLE) busy += w;
      kinds[k] = kinds[k] + w;
      sampled += w;
      i += w;
      IOSleep(w);
    }
    if (held) {
      IOSleep(GP_GOV_WINDOW);
      if (!guamp_gpu_dcvs_hold) measSelect();   // after a recovery: the counter may be back to its reset select
      continue;
    }
    guamp_gpu_dcvs_busy[0] = guamp_gpu_dcvs_busy[0] + busy;
    guamp_gpu_dcvs_busy[1] = guamp_gpu_dcvs_busy[1] + sampled;
    guamp_gpu_dcvs_busy[2] = guamp_gpu_dcvs_busy[2] + uint64_t(kGpLevels[gS.level].mhz) * sampled;
    {
      const bool allBusy = sampled >= GP_GOV_WINDOW / 5 && busy == sampled;
      const uint32_t mhz = gp_meas_end(&gM, ran, allBusy, gg2_gr(&kIo, GG2_RBBM_PERFCTR_CP_0_LO),
                                       gg2_gr(&kIo, GG2_CP_ALWAYS_ON_COUNTER_LO));
      if (allBusy && !mhz) measSelect();
      if (ran < GP_NLEVELS && !(gOffLogged & (1u << ran)) && gM.aon[ran] >= GG2_XO_HZ && gp_meas_off(&gM, ran)) {
        gOffLogged |= 1u << ran;
        IOLog("GuampGpu: dcvs level %u MHz measures %u MHz\n", kGpLevels[ran].mhz, gp_meas_mhz(&gM, ran));
      }
    }
    if (boosted) ++gBoosts;
    quietWindow = !boosted && sampled != 0 && busy == 0;
    const bool waking = gWakeWindows != 0;
    if (waking) gWakeWindows = gWakeWindows - 1;
    if (waking) gG.quiet = 0;   // no step down while the wake hold lasts
    const unsigned prev = gS.level,
                   target = boosted || (waking && gS.level != gS.top) ? gp_gov_boost(&gG, &gS)
                                                                        : waking ? gS.level : gp_gov_window(&gG, &gS, busy);
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
    guamp_gpu_mhz_now = kGpLevels[gS.level].mhz;
    if (window % 200 == 0) {
      logState(busy, switches, fails, ms);
      logReal(kinds);
      kinds[0] = kinds[1] = kinds[2] = 0;
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
  unsigned up = 25, minMhz = 0;
  PE_parse_boot_argn("guamp-gpu-boost", &gBoostOn, sizeof(gBoostOn));
  PE_parse_boot_argn("guamp-gpu-wake-ms", &gWakeMs, sizeof(gWakeMs));
  PE_parse_boot_argn("guamp-gpu-up", &up, sizeof(up));
  PE_parse_boot_argn("guamp-gpu-min", &minMhz, sizeof(minMhz));
  PE_parse_boot_argn("guamp-gpu-dcvs-step", &gStepMs, sizeof(gStepMs));
  PE_parse_boot_argn("guamp-gpu-dcvs-idle", &gIdleStepMs, sizeof(gIdleStepMs));
  gStepMs = gStepMs < 1 ? 1 : gStepMs > 10 ? 10 : gStepMs;
  gIdleStepMs = gIdleStepMs == 0 ? gStepMs : gIdleStepMs > 25 ? 25 : gIdleStepMs;
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
  guamp_gpu_mhz_now = kGpLevels[gS.level].mhz;
  IOLog("GuampGpu: dcvs on: %u MHz, levels %u..%u MHz (bin %u from %s, guamp-gpu-max %u), cx floor %u mx floor %u\n",
        kGpLevels[level].mhz, kGpLevels[gS.bottom].mhz, kGpLevels[gS.top].mhz, bin, binFrom, cap, gCxMin, gMxMin);
  IOLog("GuampGpu: dcvs tuning: boost %u, up %u/%u samples, min %u MHz (bottom %u MHz), read every %u ms (%u ms after a quiet "
        "window)\n", gBoostOn, gG.up, GP_GOV_WINDOW, minMhz, kGpLevels[gS.bottom].mhz, gStepMs, gIdleStepMs);
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}
}  // namespace gpudcvs
