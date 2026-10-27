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
extern "C" volatile uint64_t guamp_copy_split[8];

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
  IOLog("GuampDisplay: copy frames %llu (total %llu) mean %llu us max %llu us (all-time max %s) %llu KiB/frame, "
        "swaps accepted +%u rejected +%u, scanout %s\n", frames, n, meanUs, max * 1000000 / f,
        max > lastMax ? "new" : "same", kib, accepted - lastAccepted, rejected - lastRejected,
        guamp_fb_wc_enabled == 1 ? "write-combining" : "device");
  {
    // Mapping (and, with guamp-fast-copy=1, cache maintenance) per frame and the read-only probe of guamp-copy-probe=N.
    static uint64_t lastMaps = 0, lastMapTicks = 0, lastProbes = 0, lastProbeTicks = 0;
    const uint64_t maps = guamp_copy_split[0], mapTicks = guamp_copy_split[1];
    const uint64_t probes = guamp_copy_split[2], probeTicks = guamp_copy_split[3];
    if (maps != lastMaps || probes != lastProbes)
      IOLog("GuampDisplay: source map %llu us/frame (%llu frames), cache mode %llu, cacheable %llu, refused %llu; "
            "read probe %llu us/frame over %llu KiB (%llu frames)\n",
            maps > lastMaps ? (mapTicks - lastMapTicks) / (maps - lastMaps) * 1000000 / f : 0, maps - lastMaps,
            guamp_copy_split[4], guamp_copy_split[5], guamp_copy_split[7],
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
  if (reports++ >= 30) return;
  IOLog("GuampDisplay: IOMFB seen %u external-before %u (0 absent 1 true 2 false 3 other) builtin %u forced %u\n",
        gSeen, gExternalBefore, gBuiltIn, gForced);
}
}
