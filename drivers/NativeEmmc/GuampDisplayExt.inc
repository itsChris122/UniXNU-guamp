// SPDX-License-Identifier: BSD-2-Clause-Patent
// IOMobileFramebuffer's iomfb_match_callback types a framebuffer as external (IOMFB display type 1) when its
// "external" registry property is true; QuartzCore then opens it as AppleExternalDisplay, FrontBoard's display
// identity says isExternal and UIKit's -[UIScreen _isEmbeddedScreen] is false, so the status bar falls back to
// _UIStatusBarVisualProvider_Fallback (red, rdar://45025538). The panel is the built-in one: on first publish,
// before backboardd enumerates, record what the framebuffer published and clear "external".
// guamp-display-internal=0 only records.
// Display copy counters of package/LegacyDisplayProbe.cpp (CPU copy of each swapped frame into the GOP scanout, counter
// ticks) and the swap counters; weak because a kernel without the legacy display probe has none of them.
extern "C" volatile uint64_t native_legacy_copy_n __attribute__((weak_import));
extern "C" volatile uint64_t native_legacy_copy_sum __attribute__((weak_import));
extern "C" volatile uint64_t native_legacy_copy_max __attribute__((weak_import));
extern "C" volatile uint64_t native_legacy_copy_bytes_sum __attribute__((weak_import));
extern "C" volatile uint32_t native_legacy_swap_accepted_count __attribute__((weak_import));
extern "C" volatile uint32_t native_legacy_swap_rejected_count __attribute__((weak_import));
// zzzz-guamp-ram-fb-wc.patch defines it (pmap); 0 here while that patch is out of the series (static link).
extern "C" __attribute__((weak)) unsigned int guamp_fb_wc_enabled = 0;
namespace dispext {
static volatile uint32_t gSeen = 0, gExternalBefore = 0, gBuiltIn = 0, gForced = 0;
static bool gForce = true;

static bool published(void *, void *, IOService *service, IONotifier *)
{
  OSObject *external = service->getProperty("external");
  // 0 absent, 1 true, 2 false, 3 another type
  uint32_t before = !external ? 0 : external == kOSBooleanTrue ? 1 : external == kOSBooleanFalse ? 2 : 3;
  gExternalBefore = before;
  gBuiltIn = service->getProperty("BuiltIn") == kOSBooleanTrue;
  if (gForce && (before == 1 || before == 3)) {
    service->setProperty("external", kOSBooleanFalse);
    gForced = gForced + 1;
  }
  gSeen = gSeen + 1;
  return true;
}

static void start()
{
  unsigned force = 1;
  PE_parse_boot_argn("guamp-display-internal", &force, sizeof(force));
  gForce = force != 0;
  OSDictionary *matching = IOService::serviceMatching("IOMobileFramebuffer");
  if (!matching) return;
  IONotifier *notifier = IOService::addMatchingNotification(gIOFirstPublishNotification, matching, published, nullptr);
  matching->release();
  IOLog("GuampDisplay: framebuffer publish hook %s, force internal %u\n", notifier ? "armed" : "failed", gForce);
}

// One line per report for the first few minutes: the USB mirror connects after the framebuffer is published.
// Every report: frames copied since the previous one and the CPU copy cost (mean / maximum since the previous report,
// microseconds per frame), with the scanout mapping mode (guamp-fb-wc=1: write-combining).
// scripts/guamp-canvas-variant.py (always applied to the legacy display): source mapping and read-probe counters.
extern "C" volatile uint64_t guamp_copy_split[16];
// The same variant's guamp-copy-gpu-wait counters ([0] reached, [1] timed out, [2] no ring, [3] us waited, [4] longest us,
// [5] reached at once, [6] the bound in ms; 0 = off).
extern "C" volatile uint64_t guamp_copy_gpu_wait[8];
// Section 10 of the variant, latest-frame-wins: [0] frames started, [1] coalesced, [2] stale-skipped, [3] GPU frames, [4] CPU frames,
// [5] in-thread refreshes, [6] trailing re-shows, [7] GPU failures, [8] CPU copy failures, [9] on.
extern "C" volatile uint64_t guamp_copy_latest[10];
// drivers/NativeAdreno/NativeAdrenoPublish.cpp: the Metal plugin's connections and tunnel calls (only the leading counters of its
// NativeAdrenoMetalDiag, declared first there) and its telemetry rows, which otherwise reach only the boot page (hidden under Metal).
struct GuampMetalDiagHead {
  uint32_t opens, busy, bad, closes, reports, property_reads, foreign_opens, foreign_type, name_reads, calls, last_selector, last_status,
      ext_entries, ext_selector, ext_status, ext_in, ext_out, tunnel_calls;
};
extern "C" GuampMetalDiagHead native_adreno_metal_diag;
extern "C" char native_adreno_metal_rows[48][96];
extern "C" volatile uint32_t native_adreno_metal_rows_seq;
// patches/native-adreno/0002-import-scanout-physical.patch: [0] imports checked, [1] scanout imports (spans from the kernel's scanout
// descriptor), [2] of those whose IOKit spans differed, [3] imports whose first PTE address differed from IOKit's, [4] last scanout
// import's IOKit first address, [5] its IOKit span count, [6] its address used, [7] last import's IOKit first address, [8] its span
// count, [9] its PTE address.
extern "C" volatile uint64_t native_adreno_import_pa[10];

static void importLog()
{
  static uint64_t lastChecked = 0;
  const uint64_t checked = native_adreno_import_pa[0];
  if (checked == lastChecked) return;
  IOLog("GuampMetal: imports %llu (+%llu), pte!=iokit %llu; scanout imports %llu, iokit differed %llu, last: iokit pa %llx spans %llu "
        "-> pa %llx; last import: iokit pa %llx spans %llu pte %llx\n", checked, checked - lastChecked, native_adreno_import_pa[3],
        native_adreno_import_pa[1], native_adreno_import_pa[2], native_adreno_import_pa[4], native_adreno_import_pa[5],
        native_adreno_import_pa[6], native_adreno_import_pa[7], native_adreno_import_pa[8], native_adreno_import_pa[9]);
  lastChecked = checked;
}

static void metalLog()
{
  importLog();
  static uint32_t lastCalls = 0, lastTunnel = 0, lastSeq = 0;
  const GuampMetalDiagHead &d = native_adreno_metal_diag;
  const uint32_t seq = native_adreno_metal_rows_seq;
  if (d.calls == lastCalls && d.tunnel_calls == lastTunnel && seq == lastSeq) return;
  IOLog("GuampMetal: opens %u busy %u bad %u closes %u calls %u last sel %x status %x; ext %u sel %x status %x; tunnel %u\n", d.opens,
        d.busy, d.bad, d.closes, d.calls, d.last_selector, d.last_status, d.ext_entries, d.ext_selector, d.ext_status, d.tunnel_calls);
  if (seq != lastSeq)
    for (unsigned i = 0; i < 48; i++)
      if (native_adreno_metal_rows[i][0]) IOLog("GuampMetal: row %u %.96s\n", i, native_adreno_metal_rows[i]);
  lastCalls = d.calls; lastTunnel = d.tunnel_calls; lastSeq = seq;
}

static void copyLog()
{
  static uint64_t lastN = 0, lastSum = 0, lastBytes = 0, lastMax = 0;
  static uint32_t lastAccepted = 0, lastRejected = 0;
  if (!&native_legacy_copy_n) return;
  const uint64_t n = native_legacy_copy_n, sum = native_legacy_copy_sum, bytes = native_legacy_copy_bytes_sum;
  const uint64_t max = native_legacy_copy_max;
  const uint32_t accepted = &native_legacy_swap_accepted_count ? native_legacy_swap_accepted_count : 0;
  const uint32_t rejected = &native_legacy_swap_rejected_count ? native_legacy_swap_rejected_count : 0;
  uint64_t f = 0;
  __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(f));
  if (!f) f = 19200000;
  const uint64_t frames = n - lastN;
  const uint64_t meanUs = frames ? (sum - lastSum) / frames * 1000000 / f : 0;
  const uint64_t kib = frames ? ((bytes - lastBytes) / frames) >> 10 : 0;
  // guamp-copy-gpu-wait=<ms>: the deferred copies' waits for the GPU since the previous report (reached / timed out / no ring,
  // of which already reached), the mean and all-time longest wait.
  static uint64_t lastWaits = 0, lastTimeouts = 0, lastNoRing = 0, lastWaitUs = 0, lastReady = 0;
  const uint64_t waits = guamp_copy_gpu_wait[0], timeouts = guamp_copy_gpu_wait[1], noRing = guamp_copy_gpu_wait[2];
  const uint64_t waitUs = guamp_copy_gpu_wait[3], ready = guamp_copy_gpu_wait[5];
  const uint64_t waited = (waits - lastWaits) + (timeouts - lastTimeouts);
  // Latest-frame-wins (variant section 10), since the previous report: frames started, coalesced (replaced before they started),
  // stale-skipped (a newer frame queued during the GPU wait), shown by the GPU / the CPU, refreshes, trailing re-shows, failures.
  char latest[200] = "";
  {
    static uint64_t last[9] = {};
    uint64_t now[9];
    for (unsigned i = 0; i < 9; i++) now[i] = guamp_copy_latest[i];
    if (guamp_copy_latest[9])
      snprintf(latest, sizeof(latest), "; latest: frames +%llu coalesced +%llu stale-skipped +%llu gpu +%llu cpu +%llu refresh +%llu "
               "trail +%llu fail gpu +%llu cpu +%llu", now[0] - last[0], now[1] - last[1], now[2] - last[2], now[3] - last[3],
               now[4] - last[4], now[5] - last[5], now[6] - last[6], now[7] - last[7], now[8] - last[8]);
    for (unsigned i = 0; i < 9; i++) last[i] = now[i];
  }
  if (guamp_copy_gpu_wait[6])
    IOLog("GuampDisplay: copy frames %llu (total %llu) mean %llu us max %llu us (all-time max %s) %llu KiB/frame, "
          "swaps accepted +%u rejected +%u, scanout %s; gpu wait %llu ms: waits +%llu (ready +%llu) timeouts +%llu no ring +%llu, "
          "mean %llu ms max %llu ms%s\n", frames, n, meanUs, max * 1000000 / f,
          max > lastMax ? "new" : "same", kib, accepted - lastAccepted, rejected - lastRejected,
          guamp_fb_wc_enabled == 1 ? "write-combining" : "device", guamp_copy_gpu_wait[6], waits - lastWaits, ready - lastReady,
          timeouts - lastTimeouts, noRing - lastNoRing, waited ? (waitUs - lastWaitUs) / waited / 1000 : 0,
          guamp_copy_gpu_wait[4] / 1000, latest);
  else
    IOLog("GuampDisplay: copy frames %llu (total %llu) mean %llu us max %llu us (all-time max %s) %llu KiB/frame, "
          "swaps accepted +%u rejected +%u, scanout %s%s\n", frames, n, meanUs, max * 1000000 / f,
          max > lastMax ? "new" : "same", kib, accepted - lastAccepted, rejected - lastRejected,
          guamp_fb_wc_enabled == 1 ? "write-combining" : "device", latest);
  lastWaits = waits; lastTimeouts = timeouts; lastNoRing = noRing; lastWaitUs = waitUs; lastReady = ready;
  {
    // Mapping (and, with guamp-fast-copy=1, cache maintenance) per frame and the read-only probe of guamp-copy-probe=N.
    static uint64_t lastMaps = 0, lastMapTicks = 0, lastProbes = 0, lastProbeTicks = 0;
    const uint64_t maps = guamp_copy_split[0], mapTicks = guamp_copy_split[1];
    const uint64_t probes = guamp_copy_split[2], probeTicks = guamp_copy_split[3];
    if (maps != lastMaps || probes != lastProbes)
      IOLog("GuampDisplay: source map %llu us/frame (%llu frames), cache mode %llu, cacheable %llu, refused %llu; "
            "async copies %llu (skipped: off %llu, bypass %llu, no call %llu, busy %llu; options %llx, snapshot %llu us total), "
            "cached maps reused %llu; read probe %llu us/frame over %llu KiB (%llu frames)\n",
            maps > lastMaps ? (mapTicks - lastMapTicks) / (maps - lastMaps) * 1000000 / f : 0, maps - lastMaps,
            guamp_copy_split[4], guamp_copy_split[5], guamp_copy_split[7],
            guamp_copy_split[8], guamp_copy_split[10], guamp_copy_split[11], guamp_copy_split[12], guamp_copy_split[13],
            guamp_copy_split[14], guamp_copy_split[15] * 1000000 / f, guamp_copy_split[9],
            probes > lastProbes ? (probeTicks - lastProbeTicks) / (probes - lastProbes) * 1000000 / f : 0,
            guamp_copy_split[6] >> 10, probes - lastProbes);
    lastMaps = maps; lastMapTicks = mapTicks; lastProbes = probes; lastProbeTicks = probeTicks;
  }
  lastN = n; lastSum = sum; lastBytes = bytes; lastMax = max;
  lastAccepted = accepted; lastRejected = rejected;
}

static void log()
{
  static unsigned reports = 0;
  copyLog();
  metalLog();
  if (reports++ >= 30) return;
  IOLog("GuampDisplay: IOMFB seen %u external-before %u (0 absent 1 true 2 false 3 other) builtin %u forced %u\n",
        gSeen, gExternalBefore, gBuiltIn, gForced);
}
}
