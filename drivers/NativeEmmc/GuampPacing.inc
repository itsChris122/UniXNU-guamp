// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp frame pacing, included by NativeEmmc.cpp after GuampPanelCycle.inc (mapPhysical, the guamp_backlight clock constants,
// gscan::held()). Two parts:
// 1. guamp_display_scan(): the panel's timing for the legacy display's refresh tick (scripts/guamp-canvas-variant.py section 14,
//    the vsync message to the render server). The video-mode timing engine of INTF_1 (bengal-sde.dtsi: MDSS 0x5e00000 +
//    qcom,sde-intf-off 0x6b800, the same window GuampPanelCycleCore.h uses) counts frames and lines (sde_hw_intf.c /
//    dpu_hw_intf.c): INTF_FRAME_COUNT +0x0ac (one per vsync), INTF_LINE_COUNT +0x0b0 (0 at the vsync, up to vtotal), and
//    vtotal = INTF_VSYNC_PERIOD_F0 +0x00c (vtotal * htotal) / the hsync period (INTF_HSYNC_CTL +0x008 >> 16): 1600 + 186 + 12 + 2
//    = 1800 lines for the HX83102D (Panel_hlt_hx83102d_video.xml). Read only while the display clocks run, the timing engine is on
//    and the panel power cycle does not hold the scanout. A frame counter that stands still for 50 ms (fin43: after the panel
//    cycle the counter enable can read 0) makes the reads unavailable until it moves again; with guamp-vsync-hw=1 (default)
//    INTF_FRAME_LINE_COUNT_EN is then set to 3 as dpu_hw_intf_setup_timing_engine does, at most 4 times per boot, at most once a
//    second. guamp-vsync-hw=2 reads only, 0 never reads (the tick stays a plain timer; the variant reads the same argument).
// 2. gpacing::log(), every display report (10 s, dispext::log): "GuampPacing:" with the render server's swap rate (CA fps), the
//    interval between swaps, the buffers it rotates, the swap -> completion latency, the vsync messages sent (on the panel's timing
//    or on the timer), how long after the real vblank they left, the panel's measured rate and the INTF reads.
extern "C" volatile uint64_t guamp_pacing[24];   // scripts/guamp-canvas-variant.py section 14

namespace gpacing {
constexpr uint64_t kIntf1Page = 0x05e6b000ULL;
constexpr uint32_t kIntf1Off = 0x800;
enum : uint32_t { rTimingEn = 0x000, rHsyncCtl = 0x008, rVsyncPeriod = 0x00c, rCountEn = 0x0a8, rFrame = 0x0ac, rLine = 0x0b0 };
struct State {
  int mode = -1;
  bool mapped = false, seen = false, stuck = false;
  volatile uint8_t *intf = nullptr, *dispcc = nullptr, *gcc = nullptr;
  uint32_t lastFrame = 0, vtotal = 0;
  uint64_t lastChange = 0, vtotalAt = 0, lastEnable = 0;
  volatile uint64_t reads = 0, stuckTimes = 0, enables = 0, clocksOff = 0, engineOff = 0, held = 0, badTiming = 0;
};
static State gP;

static inline uint32_t rd(uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(gP.intf + kIntf1Off + off); }
static inline void wr(uint32_t off, uint32_t v)
{
  *reinterpret_cast<volatile uint32_t *>(gP.intf + kIntf1Off + off) = v;
  __asm__ volatile("dsb sy" ::: "memory");
}
static uint64_t toAbs(uint64_t ns)
{
  uint64_t t = 0;
  nanoseconds_to_absolutetime(ns, &t);
  return t;
}
static uint64_t toNs(uint64_t t)
{
  uint64_t n = 0;
  absolutetime_to_nanoseconds(t, &n);
  return n;
}
// MDSS registers answer only behind gcc_disp_ahb_clk and disp_cc_mdss_ahb_clk.
static bool clocksOn()
{
  using namespace guamp_backlight;
  return guamp_backlight::cbcrRunning(*reinterpret_cast<volatile uint32_t *>(gP.gcc + kGccDispAhbOff)) &&
         guamp_backlight::cbcrRunning(*reinterpret_cast<volatile uint32_t *>(gP.dispcc + kCbcrMdssAhb));
}
}  // namespace gpacing

// 1 = frame counter, line and vtotal, read at mach time *at; 0 = not available now. Called by the refresh tick (display work
// loop) and by the 10 s report; a rare overlap only disturbs the stuck-counter bookkeeping of one call.
extern "C" int guamp_display_scan(uint32_t *frame, uint32_t *line, uint32_t *vtotal, uint64_t *at)
{
  using namespace gpacing;
  if (gP.mode < 0) {
    unsigned v = 1;
    PE_parse_boot_argn("guamp-vsync-hw", &v, sizeof(v));
    gP.mode = v > 2 ? 1 : (int)v;
  }
  if (!gP.mode || !frame || !line || !vtotal || !at) return 0;
  if (gscan::held()) { gP.held = gP.held + 1; gP.seen = false; return 0; }
  if (!gP.mapped) {
    gP.mapped = true;
    gP.intf = mapPhysical(kIntf1Page, 0x1000);
    gP.gcc = mapPhysical(guamp_backlight::kGccDispAhbPage, 0x1000);
    gP.dispcc = mapPhysical(guamp_backlight::kDispccPhys, 0x3000);
  }
  if (!gP.intf || !gP.gcc || !gP.dispcc) return 0;
  if (!clocksOn()) { gP.clocksOff = gP.clocksOff + 1; gP.seen = false; return 0; }
  if (!(rd(rTimingEn) & 1u)) { gP.engineOff = gP.engineOff + 1; gP.seen = false; return 0; }
  uint64_t t = mach_absolute_time();
  if (!gP.vtotal || t - gP.vtotalAt > toAbs(1000000000ull)) {   // the mode, again every second
    const uint32_t hsync = rd(rHsyncCtl) >> 16, period = rd(rVsyncPeriod);
    const uint32_t vt = hsync ? period / hsync : 0;
    gP.vtotalAt = t;
    if (hsync < 100 || vt < 1000 || vt > 4096) { gP.vtotal = 0; gP.badTiming = gP.badTiming + 1; return 0; }
    gP.vtotal = vt;
  }
  uint32_t f = rd(rFrame), l = rd(rLine);
  t = mach_absolute_time();
  const uint32_t again = rd(rFrame);
  if (again != f) { f = again; l = rd(rLine); t = mach_absolute_time(); }   // a vsync between the two reads
  if (!gP.seen || f != gP.lastFrame) {
    gP.seen = true; gP.stuck = false; gP.lastFrame = f; gP.lastChange = t;
  } else if (gP.stuck || t - gP.lastChange > toAbs(50000000ull)) {
    if (!gP.stuck) { gP.stuck = true; gP.stuckTimes = gP.stuckTimes + 1; }
    if (gP.mode == 1 && gP.enables < 4 && (!gP.lastEnable || t - gP.lastEnable > toAbs(1000000000ull)) && (rd(rCountEn) & 3u) != 3u) {
      wr(rCountEn, 3u);   // dpu_hw_intf_setup_timing_engine: DPU_REG_WRITE(c, INTF_FRAME_LINE_COUNT_EN, 0x3)
      gP.enables = gP.enables + 1; gP.lastEnable = t;
      IOLog("GuampPacing: INTF_1 frame counter stood at %u (line %u): counter enable set (%llu)\n", f, l, gP.enables);
    }
    return 0;
  }
  if (l >= gP.vtotal) return 0;
  *frame = f; *line = l; *vtotal = gP.vtotal; *at = t;
  gP.reads = gP.reads + 1;
  return 1;
}

namespace gpacing {
void log()
{
  static uint64_t last[24], lastHw[5], lastAt = 0;
  uint64_t now[24];
  for (unsigned i = 0; i < 24; ++i) now[i] = guamp_pacing[i];
  const uint64_t hw[5] = {gP.reads, gP.stuckTimes, gP.clocksOff, gP.engineOff, gP.held};
  const uint64_t t = mach_absolute_time();
  // fin44: the INTF counters were read only by the refresh tick, which never ran (no vsync client): sampled here as well, so the
  // real panel rate is in every line (frames counted over the report, in mHz).
  static uint32_t sampleFrame = 0;
  static uint64_t sampleAt = 0;
  uint32_t f = 0, l = 0, vt = 0;
  uint64_t at = 0, sampledMhz = 0;
  const bool sampled = guamp_display_scan(&f, &l, &vt, &at) == 1;
  if (sampled) {
    const uint64_t spanNs = sampleAt && at > sampleAt ? toNs(at - sampleAt) : 0;
    if (spanNs) sampledMhz = (uint64_t)(uint32_t)(f - sampleFrame) * 1000000000000ull / spanNs;
    sampleFrame = f; sampleAt = at;
  } else sampleAt = 0;
  const bool moved = now[0] != last[0] || now[7] != last[7] || sampled;
  if (lastAt && moved) {
    const uint64_t span = toNs(t - lastAt);
    const uint64_t swaps = now[0] - last[0], intervals = now[21] - last[21];
    const uint64_t fps10 = span ? swaps * 10000000000ull / span : 0;
    const uint64_t meanUs = intervals ? toNs((now[1] - last[1]) / intervals) / 1000 : 0, maxUs = toNs(now[2]) / 1000;
    const uint64_t retired = now[3] - last[3];
    const uint64_t latUs = retired ? toNs((now[4] - last[4]) / retired) / 1000 : 0, latMaxUs = toNs(now[5]) / 1000;
    const uint64_t sent = now[7] - last[7], panel = now[8] - last[8];
    const uint64_t afterUs = panel ? toNs((now[11] - last[11]) / panel) / 1000 : 0;
    const uint64_t periodNs = toNs(now[12]);
    const uint64_t mhz = periodNs ? 1000000000000ull / periodNs : 0;
    IOLog("GuampPacing: CA swaps +%llu (%llu.%llu fps), interval mean %llu.%llu ms max %llu.%llu ms, idle gaps +%llu, buffers %llu; "
          "swap->complete mean %llu us max %llu us; vsync %llu Hz clients %02llx (vsync %s): sent +%llu (panel +%llu timer +%llu, "
          "late +%llu), after the vblank %llu us; panel %llu.%03llu Hz (%llu measures) vtotal %llu frames +%u; INTF reads +%llu "
          "stuck +%llu enables %llu, clocks off +%llu engine off +%llu held +%llu; sampled %s %llu.%03llu Hz line %u/%u; "
          "IOMFBDisplayRefresh %s\n",
          swaps, fps10 / 10, fps10 % 10, meanUs / 1000, (meanUs / 100) % 10, maxUs / 1000, (maxUs / 100) % 10, now[19] - last[19],
          now[6], latUs, latMaxUs, now[17], now[15], now[16] ? "on" : "off", sent, panel, now[9] - last[9], now[10] - last[10],
          afterUs, mhz / 1000, mhz % 1000, now[18], now[13], (uint32_t)(now[14] - last[14]), hw[0] - lastHw[0], hw[1] - lastHw[1],
          gP.enables, hw[2] - lastHw[2], hw[3] - lastHw[3], hw[4] - lastHw[4], sampled ? "yes" : "no", sampledMhz / 1000,
          sampledMhz % 1000, l, vt, now[22] ? "published (60 Hz fixed)" : "not published");
  }
  if (moved || !lastAt) {
    guamp_pacing[2] = 0;   // the longest interval / latency are per report
    guamp_pacing[5] = 0;
    now[2] = now[5] = 0;
  }
  for (unsigned i = 0; i < 24; ++i) last[i] = now[i];
  for (unsigned i = 0; i < 5; ++i) lastHw[i] = hw[i];
  lastAt = t;
}
}  // namespace gpacing
