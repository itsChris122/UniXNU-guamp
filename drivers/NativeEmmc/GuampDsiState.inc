// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp DSI / MDSS / panel state dump (guamp-dsi-dump=1, default 0), included by NativeEmmc.cpp after GuampBacklight.inc
// (DSI window, clock check and lock). What and why: GuampDsiStateCore.h. Own thread: one dump as soon as the backlight
// service has the DSI window ("boot"), one at 60 s of uptime ("60s": only the 8-word lines that changed since "boot").
// guamp-dsi-dump-dcs=0: no DCS reads (registers only); guamp-dsi-dump-lp=0: the DCS reads in HS instead of LP.
// The Himax DD registers through SPI are read by the touch thread (TouchService.inc ddDump, same boot argument).
// Lines: "GuampDsi: <when> <region> +<off>: 8 words" (all-zero lines counted, not printed), "GuampDsi: <when> key ..."
// (decoded summary), "GuampDsi: <when> dcs <cmd> ...". The TWRP counterpart: strumenti/twrp-dsi-stato.sh.
#include "GuampDsiStateCore.h"

namespace dsistate {
using namespace guamp_dsistate;
volatile uint8_t *gWin[kRegionCount];
uint32_t gBoot[kRegionCount][kMaxWords];
bool gHaveBoot;
unsigned gDcs = 1, gLp = 1;

uint64_t uptimeMs() {
  uint64_t ns = 0;
  absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
  return ns / 1000000u;
}
bool mapAll() {
  for (unsigned i = 0; i < kRegionCount; ++i) {
    const uint64_t page = kRegions[i].phys & ~0xfffULL, in = kRegions[i].phys & 0xfffULL;
    volatile uint8_t *p = mapPhysical(page, (in + kRegions[i].len + 0xfffULL) & ~0xfffULL);
    if (!p) { IOLog("GuampDsi: map of %s (%llx) failed\n", kRegions[i].name, (unsigned long long)kRegions[i].phys); return false; }
    gWin[i] = p + in;
  }
  return true;
}
uint32_t rdw(unsigned region, uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(gWin[region] + off); }
unsigned regionIndex(const char *name) {
  for (unsigned i = 0; i < kRegionCount; ++i) if (!strcmp(kRegions[i].name, name)) return i;
  return 0;
}

void dumpRegs(const char *when, uint32_t (*snap)[kMaxWords]) {
  for (unsigned i = 0; i < kRegionCount; ++i) {
    const unsigned words = kRegions[i].len / 4;
    for (unsigned w = 0; w < words; ++w) snap[i][w] = rdw(i, 4u * w);
  }
  for (unsigned i = 0; i < kRegionCount; ++i) {
    const unsigned words = kRegions[i].len / 4;
    unsigned zero = 0, same = 0, changed = 0;
    for (unsigned w = 0; w < words; w += kWordsPerLine) {
      const unsigned n = words - w < kWordsPerLine ? words - w : kWordsPerLine;
      bool allZero = true, diff = false;
      for (unsigned k = 0; k < n; ++k) {
        if (snap[i][w + k]) allZero = false;
        if (gHaveBoot && snap[i][w + k] != gBoot[i][w + k]) diff = true;
      }
      if (gHaveBoot && !diff) { same++; continue; }
      if (!gHaveBoot && allZero) { zero++; continue; }
      changed++;
      char line[8 * 9 + 1];
      unsigned len = 0;
      for (unsigned k = 0; k < n; ++k) len += (unsigned)snprintf(line + len, sizeof(line) - len, " %08x", snap[i][w + k]);
      IOLog("GuampDsi: %s %s +%03x:%s\n", when, kRegions[i].name, 4u * w, line);
    }
    IOLog("GuampDsi: %s %s (%llx, twrp %s +%x, %u bytes): %u lines printed, %u all zero, %u unchanged\n", when,
          kRegions[i].name, (unsigned long long)kRegions[i].phys, kRegions[i].twrp, kRegions[i].twrpOff, kRegions[i].len,
          changed, zero, same);
  }
}

// The fields that decide the link and timing, decoded (offsets: dsi_ctrl_reg.h, sde_hw_intf.c).
void keyLine(const char *when, uint32_t (*snap)[kMaxWords]) {
  const unsigned d = regionIndex("dsi0"), f = regionIndex("intf1");
  const uint32_t *s = snap[d], *t = snap[f];
  const uint32_t ctrl = s[0x004 / 4], vm = s[0x010 / 4];
  IOLog("GuampDsi: %s key dsi ctrl %08x (en %u vid %u cmd %u lanes %x clk %u) video %08x (traffic %u: %s, lp11 hsa %u hbp %u "
        "hfp %u bllp %u eof %u, pulse_hsa_he %u, fmt %u, vc %u) active-h %08x active-v %08x total %08x hsync %08x vsync %08x "
        "vpos %08x lane-ctrl %08x (clk hs forced %u) lane-status %08x clkout %08x eot %08x lp-timer %08x hs-timer %08x "
        "dma %08x trig %08x int %08x err-mask %08x\n",
        when, ctrl, ctrl & 1, (ctrl >> 1) & 1, (ctrl >> 2) & 1, (ctrl >> 4) & 0xf, (ctrl >> 8) & 1, vm, (vm >> 8) & 3,
        ((vm >> 8) & 3) == 0 ? "non-burst sync pulse" : ((vm >> 8) & 3) == 1 ? "non-burst sync event" : "burst",
        (vm >> 16) & 1, (vm >> 20) & 1, (vm >> 24) & 1, (vm >> 12) & 1, (vm >> 15) & 1, (vm >> 28) & 1, (vm >> 4) & 3, vm & 3,
        s[0x024 / 4], s[0x028 / 4], s[0x02c / 4], s[0x030 / 4], s[0x034 / 4], s[0x038 / 4], s[0x0ac / 4], (s[0x0ac / 4] >> 28) & 1,
        s[0x0a8 / 4], s[0x0c4 / 4], s[0x0cc / 4], s[0x0b8 / 4], s[0x0bc / 4], s[0x03c / 4], s[0x084 / 4], s[0x110 / 4], s[0x10c / 4]);
  IOLog("GuampDsi: %s key intf1 en %08x config %08x hsync-ctl %08x vsync-period %08x vsync-pw %08x disp-v %08x..%08x "
        "active-v %08x..%08x disp-hctl %08x active-hctl %08x skew %08x polarity %08x panel-format %08x config2 %08x "
        "prog-fetch %08x frame %08x line %08x\n",
        when, t[0x000 / 4], t[0x004 / 4], t[0x008 / 4], t[0x00c / 4], t[0x014 / 4], t[0x01c / 4], t[0x024 / 4], t[0x02c / 4],
        t[0x034 / 4], t[0x03c / 4], t[0x040 / 4], t[0x04c / 4], t[0x050 / 4], t[0x090 / 4], t[0x060 / 4], t[0x170 / 4],
        t[0x0ac / 4], t[0x0b0 / 4]);
}

void dcsReads(const char *when) {
  if (!gDcs) return;
  backlight::DsiGuard guard;
  backlight::Io io;
  unsigned fails = 0;
  for (unsigned i = 0; i < kDcsReadCount; ++i) {
    ReadBack r;
    const int rc = readDcsVideo(io, kDcsReads[i].cmd, kDcsReads[i].len, gLp != 0, r);
    uint8_t wire[16] = {};
    const unsigned n = rc == kClocksOff ? 0 : wireBytes(r, wire);
    char hex[16 * 3 + 1] = "";
    unsigned len = 0;
    for (unsigned k = 0; k < n; ++k) len += (unsigned)snprintf(hex + len, sizeof(hex) - len, " %02x", wire[k]);
    IOLog("GuampDsi: %s dcs %02x len %u %s: %s, rdbk %08x [%08x %08x %08x %08x] type %08x %08x int %08x status %08x ack %08x, "
          "%u us, wire:%s\n", when, kDcsReads[i].cmd, kDcsReads[i].len, gLp ? "lp" : "hs", resultName(rc), r.ctrl, r.d[0],
          r.d[1], r.d[2], r.d[3], r.type0, r.type1, r.intCtrl, r.status, r.ack, r.waitedUs, n ? hex : " -");
    if (rc == kClocksOff || rc == kCtrlOff || rc == kNotVideo || rc == kBadVersion || rc == kBadTiming || rc == kBusy) break;
    fails = rc == kSent ? 0 : fails + 1;
    if (fails >= 4) { IOLog("GuampDsi: %s dcs: 4 failures in a row, reads stopped\n", when); break; }
    IOSleep(20);
  }
}

void dump(const char *when) {
  static uint32_t snap[kRegionCount][kMaxWords];
  if (!backlight::clocksOn()) { IOLog("GuampDsi: %s: display clocks off, nothing read\n", when); return; }
  {
    backlight::DsiGuard guard;                           // no command DMA in flight while the windows are read
    dumpRegs(when, gHaveBoot ? snap : gBoot);
  }
  keyLine(when, gHaveBoot ? snap : gBoot);
  gHaveBoot = true;
  dcsReads(when);
}

void thread(void *, wait_result_t) {
  if (!mapAll()) return;
  dump("boot");
  const uint64_t now = uptimeMs();
  if (now < 60000) IOSleep((uint32_t)(60000 - now));
  dump("60s");
}

void start() {
  unsigned on = 0;
  PE_parse_boot_argn("guamp-dsi-dump", &on, sizeof(on));
  if (!on) return;
  PE_parse_boot_argn("guamp-dsi-dump-dcs", &gDcs, sizeof(gDcs));
  PE_parse_boot_argn("guamp-dsi-dump-lp", &gLp, sizeof(gLp));
  if (!backlight::gDsi) { IOLog("GuampDsi: no DSI window (guamp-backlight=0?): nothing dumped\n"); return; }
  IOLog("GuampDsi: dump on (dcs %u, %s), %u regions\n", gDcs, gLp ? "lp" : "hs", kRegionCount);
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}
}  // namespace dsistate
