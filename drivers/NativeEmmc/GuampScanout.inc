// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp scanout, included by NativeEmmc.cpp after GuampBacklight.inc (mapPhysical, guamp_backlight clock constants).
// The legacy display's copy thread (scripts/guamp-canvas-variant.py section 12, latestCopyOnce) hands every frame to
// guamp_scanout_present() first; 0 = not taken (the variant copies as before), 1 = drawn, -1 = refused (the variant copies).
// Everything is off by default (the hook returns 0 at once):
//   guamp-copy-diff=1        only what changed is drawn: source rows hashed in 64-pixel bands (GuampScanoutCore.h), changed
//                            bands scaled into the scanout; a frame with no change costs one read pass of the surface.
//                            guamp-copy-diff-full=<n> (default 600, 0 never): everything is redrawn every n-th frame.
//   guamp-copy-neon=1        the scanout stores of a drawn span go in 64-byte NEON blocks (ldp/stp q; inline asm only).
//   guamp-scanout-flip=1     double buffering: the frame is drawn into a second buffer in the boot loader's display region
//                            (0x5C000000-0x5D000000, the GOP frame at its start; guamp-flip-offset=<bytes>, default 0x500000)
//                            and the MDSS source pipe that scans the GOP frame (found by reading its SRC0_ADDR) is pointed
//                            at it: SSPP SRC0_ADDR + CTL_0 FLUSH, taken by the hardware at the next vsync. Before drawing into
//                            the buffer just left, the flush bit must have cleared (the vsync passed): no frame is ever
//                            shown half drawn. guamp-flip-idle-ms=<ms> (default 500; was 60): with no new frame the GOP frame catches
//                            up and is shown again (the USB mirror, the boot progress and the panic screen use it); a panic
//                            or a restart points the pipe back at once. Any doubt (clocks off, pipe not found, flush never
//                            taken in 50 ms) turns the flip off for the rest of the boot, back on the GOP frame.
//   guamp-scanout-hwscale=1  (implies guamp-scanout-flip=1) no CPU scaler: the swapped surface is copied 1:1 (only what changed
//                            with guamp-copy-diff=1) into one of two slots of the display region (GOP + 0x500000 and
//                            + 0xA00000, 5 MiB each, below the cont_splash end at + 0xF00000) and the VIG0 pipe scans the slot:
//                            a canvas frame (750x1666) through its QSEED3LITE scaler to the panel (GuampScanoutHw.h, Linux dpu1
//                            values), a 1:1 frame (composition) with the boot loader's setup. Every SSPP/scaler register goes with
//                            the flip's CTL flush, at the vsync. With no frame for guamp-hwscale-idle-ms (default 1000, 0 never)
//                            the GOP frame is drawn from the shown slot (CPU scaler, outside the lock, given up if a frame comes)
//                            and shown again with the boot loader's setup (USB mirror, panic screen). Any doubt (scaler not
//                            QSEED3LITE, pipe not VIG0, output not the panel, slot outside the region) leaves the flip as before.
// Idle default (06/11): fin50 had 60 ms, shorter than the render server's frame interval at 10-15 fps (70-100 ms), so almost
// every frame was followed by an idle return: a catch-up read of the write-combining buffer (slow) in a kernel-high thread
// call (5-37% of a core) and a second flip. 500 ms keeps the return for a really idle screen only.
// Logs: "GuampScanout:" at the first frame (mode, pipe probe) and in every display report (frames, drawn share, box, flips).
#include "GuampScanoutCore.h"
#include "GuampScanoutHw.h"

namespace gscan {
// dpu_6_3_sm6115 / bengal-sde: MDSS at 0x5E00000, MDP top 0x1000 above; CTL_0 MDP+0x1000, SSPP VIG0 MDP+0x4000, DMA0 MDP+0x24000.
constexpr uint64_t kMdssPhys = 0x05E00000ULL, kMdssBytes = 0x30000ULL;
constexpr uint32_t kCtl0 = 0x2000, kCtlLayer0 = 0x000, kCtlFlush = 0x018, kCtlLayerExt0 = 0x040;
constexpr uint32_t kSrcSize = 0x00, kSrc0Addr = 0x14, kSrcYStride0 = 0x24, kSrcFormat = 0x30, kCurrentSrc0 = 0xA4;
struct Pipe { const char *name; uint32_t offset, flush; };
constexpr Pipe kPipes[] = {{"VIG0", 0x5000, 1u << 0}, {"DMA0", 0x25000, 1u << 11}};
constexpr uint64_t kRegionBase = 0x5C000000ULL, kRegionEnd = 0x5D000000ULL;   // guampPkg MemoryMapLib "Display Reserved"
constexpr unsigned kLatchUs = 50000;
constexpr uint32_t kFlushCtl = 1u << 17;   // dpu_hw_ctl CTL_FLUSH_MASK_CTL, sent with every plane flush by Linux
// guamp-touch-panelcycle=5 (GuampPanelCycle.inc) stops the INTF timing engine for ~0.6 s: no flip may wait for a latch
// then. While held, present() answers 0 (the variant copies into the GOP frame itself, which the copy counter notices)
// and the idle catch-up / external paths leave the pipe alone.
volatile int gHold = 0;

struct Kernel {
  bool read = false, diff = false, neon = false, flipWanted = false;
  unsigned fullEvery = 600, idleMs = 500;
  unsigned earlyFrames = 12;   // guamp-scanout-early=<n> (default 12, 0 = none): the first n frames logged one by one
  uint64_t flipOffset = 0x500000;
  uint32_t flushMask = 0;     // pipe bit (+ the CTL bit, guamp-flip-flush-ctl=0 leaves it out)
  IOLock *lock = nullptr;
  State st;
  void *mem = nullptr;
  vm_size_t memBytes = 0;
  const uint32_t *keyColumns = nullptr, *keyRows = nullptr;
  uint32_t keyDims[4] = {};
  uint64_t keyStride = 0;
  bool ready = false;
  unsigned sinceFull = 0;
  uint64_t expectCopyN = ~0ull;
  // flip
  int probe = 0;              // 0 not tried, 1 found, <0 why not (logged)
  int pipe = -1;
  bool flipOn = false;
  volatile uint8_t *mdss = nullptr, *dispcc = nullptr, *gccDisp = nullptr;
  uint64_t pa[2] = {};
  volatile uint8_t *va[2] = {};
  uint64_t stride = 0;
  unsigned shown = 0;          // buffer the pipe points at (or will at the next vsync)
  bool latchPending = false;
  uint64_t presents = 0;       // present counter seen by the idle call
  thread_call_t idleCall = nullptr;
  uint32_t probeRegs[6] = {};
  // stats (log)
  volatile uint64_t frames = 0, unchanged = 0, rows = 0, pixels = 0, full = 0, flips = 0, latchUs = 0, latchMax = 0,
                    latchTimeouts = 0, idleBacks = 0, refused = 0, externals = 0, scanTicks = 0, drawTicks = 0;
  volatile uint32_t box[4] = {};
  volatile unsigned lastRefusal = 0;
  uint64_t area = 0;            // output pixels of the last frame's geometry (log)
  // guamp-scanout-hwscale: two slots scanned by VIG0, 1:1 copies into them, the pipe's scaler for canvas frames.
  bool hwWanted = false, hwOn = false;
  int hwProbe = 0;              // 0 not tried, 1 on, <0 why not (logged)
  unsigned hwIdleMs = 1000;
  uint32_t saved[gscanhw::kSavedCount] = {}, outSize = 0, srcFormat = 0, scVersion = 0;
  gscanhw::Slots slots = {};
  volatile uint8_t *slotVa[2] = {};
  int shownSlot = -1;           // -1: the GOP frame (boot loader setup), 0/1: that slot
  unsigned slotKind[2] = {};    // 0 nothing yet, 1 a 1:1 frame, 2 a canvas frame
  uint32_t slotStride[2] = {};
  uint32_t cfgKey[4] = {};      // kind, width, height, stride of the configuration in the pipe (kind 0: the boot loader's)
  State ss;                     // 1:1 copy into the slots (pending[i] = slot i)
  void *ssMem = nullptr;
  vm_size_t ssBytes = 0;
  uint32_t ssKey[3] = {};
  uint64_t ssStride = 0;
  bool ssReady = false;
  // idle return: the GOP frame drawn from the shown slot by the CPU scaler (own state and own copy of the taps)
  State gs;
  void *gsMem = nullptr;
  vm_size_t gsBytes = 0;
  uint32_t *idleTaps = nullptr;   // columns (panel width) then rows (panel height), copied from the last canvas frame
  vm_size_t idleTapBytes = 0;
  uint32_t tapDims[4] = {};       // source w, h, panel w, h of idleTaps
  bool tapsValid = false;
  thread_call_t hwIdleCall = nullptr;
  volatile bool hwIdleBusy = false;
  volatile uint64_t hwFrames = 0, hwCanvas = 0, hwPanel = 0, hwIdleBacks = 0, hwIdleAborts = 0, hwIdleTicks = 0, hwConfigs = 0;
};
static Kernel gK;

static inline uint64_t ticks()
{
  uint64_t t;
  __asm__ volatile("isb\n\tmrs %0, cntpct_el0" : "=r"(t));
  return t;
}
static inline uint32_t rd(uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(gK.mdss + off); }
static inline void wr(uint32_t off, uint32_t v)
{
  *reinterpret_cast<volatile uint32_t *>(gK.mdss + off) = v;
  __asm__ volatile("dsb sy" ::: "memory");
}

// 64-byte NEON blocks (guamp-copy-neon=1). The kernel's exception entry and context switch keep the vector registers of a
// kernel thread (corecrypto uses them too); the compiler is never given one (-mgeneral-regs-only), only this asm.
static void neonStore(volatile void *to, const void *from, u64 bytes)
{
  __asm__ volatile(".arch_extension fp\n\t.arch_extension simd\n"
                   "1:\n\tldp q0, q1, [%1], #32\n\tldp q2, q3, [%1], #32\n\tstp q0, q1, [%0], #32\n\tstp q2, q3, [%0], #32\n"
                   "\tsubs %2, %2, #64\n\tb.ne 1b"
                   : "+r"(to), "+r"(from), "+r"(bytes)::"v0", "v1", "v2", "v3", "memory", "cc");
}

static void readOptions()
{
  if (gK.read) return;
  unsigned v = 0;
  PE_parse_boot_argn("guamp-copy-diff", &v, sizeof(v)); gK.diff = v == 1;
  v = 0; PE_parse_boot_argn("guamp-copy-neon", &v, sizeof(v)); gK.neon = v == 1;
  v = 0; PE_parse_boot_argn("guamp-scanout-flip", &v, sizeof(v)); gK.flipWanted = v == 1;
  v = 0; PE_parse_boot_argn("guamp-scanout-hwscale", &v, sizeof(v)); gK.hwWanted = v == 1;
  if (gK.hwWanted) gK.flipWanted = true;
  PE_parse_boot_argn("guamp-hwscale-idle-ms", &gK.hwIdleMs, sizeof(gK.hwIdleMs));
  if (gK.hwIdleMs > 10000) gK.hwIdleMs = 10000;
  PE_parse_boot_argn("guamp-copy-diff-full", &gK.fullEvery, sizeof(gK.fullEvery));
  PE_parse_boot_argn("guamp-flip-idle-ms", &gK.idleMs, sizeof(gK.idleMs));
  PE_parse_boot_argn("guamp-scanout-early", &gK.earlyFrames, sizeof(gK.earlyFrames));
  if (gK.idleMs > 1000) gK.idleMs = 1000;
  uint32_t off = 0x500000;
  PE_parse_boot_argn("guamp-flip-offset", &off, sizeof(off)); gK.flipOffset = off;
  // The GPU presenter (sheng-fb-gpu=1) writes the GOP frame behind the copy: no flip, no diff for its frames.
  unsigned handoff = 0;
  PE_parse_boot_argn("sheng-fb-gpu", &handoff, sizeof(handoff));
  if (handoff == 1) gK.flipWanted = gK.hwWanted = false;
  gK.lock = IOLockAlloc();
  if (!gK.lock) gK.diff = gK.flipWanted = false;
  __atomic_store_n(&gK.read, true, __ATOMIC_SEQ_CST);   // after the lock: hold() takes it once read is seen
}

// GuampPanelCycle.inc, around the panel power cycle: on = no present / catch-up / external touches the pipe from now on
// (one in flight is waited for through the lock); off = back to normal.
void hold(bool on)
{
  __atomic_store_n(&gHold, on ? 1 : 0, __ATOMIC_SEQ_CST);
  if (on && __atomic_load_n(&gK.read, __ATOMIC_SEQ_CST) && gK.lock) { IOLockLock(gK.lock); IOLockUnlock(gK.lock); }
}
static bool held() { return __atomic_load_n(&gHold, __ATOMIC_SEQ_CST) != 0; }

static bool clocksOn()
{
  using namespace guamp_backlight;
  if (!gK.dispcc || !gK.gccDisp) return false;
  const uint32_t gcc = *reinterpret_cast<volatile uint32_t *>(gK.gccDisp + kGccDispAhbOff);
  const uint32_t ahb = *reinterpret_cast<volatile uint32_t *>(gK.dispcc + kCbcrMdssAhb);
  return cbcrRunning(gcc) && cbcrRunning(ahb);
}

static bool latched() { return !(rd(kCtl0 + kCtlFlush) & gK.flushMask); }

// Waits (sleeping) until the last flip was taken by the hardware; false after kLatchUs.
static bool waitLatch()
{
  if (!gK.latchPending) return true;
  const uint64_t start = ticks();
  uint64_t f = 0;
  __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(f));
  if (!f) f = 19200000;
  for (;;) {
    if (latched()) break;
    const uint64_t us = (ticks() - start) * 1000000 / f;
    if (us > kLatchUs) { gK.latchTimeouts = gK.latchTimeouts + 1; return false; }
    if (us < 1000) IODelay(100); else IOSleep(1);
  }
  const uint64_t us = (ticks() - start) * 1000000 / f;
  gK.latchUs = gK.latchUs + us;
  if (us > gK.latchMax) gK.latchMax = us;
  gK.latchPending = false;
  return true;
}

static void point(unsigned t)
{
  wr(kPipes[gK.pipe].offset + kSrc0Addr, (uint32_t)gK.pa[t]);
  wr(kCtl0 + kCtlFlush, gK.flushMask);
  gK.shown = t;
  gK.latchPending = true;
  gK.flips = gK.flips + 1;
}

// guamp-scanout-hwscale: the pipe's setup for one kind of frame (key: kind, width, height, stride; kind 0 = the boot loader's).
// Taken with the next flush, like SRC0_ADDR.
static bool sameKey(const uint32_t a[4], const uint32_t b[4]) { return a[0] == b[0] && a[1] == b[1] && a[2] == b[2] && a[3] == b[3]; }
static void writeConfig(const gscanhw::Config &c, const uint32_t key[4])
{
  for (unsigned i = 0; i < c.count; ++i) wr(kPipes[gK.pipe].offset + c.regs[i].off, c.regs[i].value);
  for (unsigned i = 0; i < 4; ++i) gK.cfgKey[i] = key[i];
  gK.hwConfigs = gK.hwConfigs + 1;
}
// The GOP frame with the boot loader's setup again (point(0) with the scaler setup undone when a slot setup is in the pipe).
static void pointGop()
{
  if (gK.cfgKey[0]) {
    const uint32_t key[4] = {0, 0, 0, 0};
    writeConfig(gscanhw::original(gK.saved), key);
  }
  gK.shownSlot = -1;
  point(0);
}
static void pointSlot(unsigned t, const gscanhw::Config &c, const uint32_t key[4])
{
  if (!sameKey(gK.cfgKey, key)) writeConfig(c, key);
  wr(kPipes[gK.pipe].offset + kSrc0Addr, (uint32_t)gK.slots.pa[t]);
  wr(kCtl0 + kCtlFlush, gK.flushMask);
  gK.shownSlot = (int)t;
  gK.shown = 0;   // the legacy second buffer is not used while the slots are
  gK.latchPending = true;
  gK.flips = gK.flips + 1;
}
static bool slotShown() { return gK.shownSlot >= 0 || gK.cfgKey[0] != 0; }

// Back on the GOP frame for good (first doubt): pointed back and waited for, best effort.
static void flipOff(unsigned why)
{
  gK.lastRefusal = why;
  if (gK.flipOn && gK.mdss && clocksOn()) {
    if (gK.shown != 0 || slotShown()) { gK.latchPending = false; pointGop(); }
    waitLatch();
  }
  if (gK.hwOn) IOLog("GuampScanout: hardware scaling off (reason %u)\n", why);
  gK.hwOn = false;
  gK.shownSlot = -1;
  gK.flipOn = false;
  gK.shown = 0;
  gK.st.bufValid[0] = false;
  IOLog("GuampScanout: double buffering off (reason %u), the GOP frame is the scanout again\n", why);
}

// guamp-scanout-hwscale, the idle return (thread call): the GOP frame drawn from the shown slot (the CPU scaler for a canvas
// frame, a copy for a 1:1 one) without the lock, then shown with the boot loader's setup unless a frame came meanwhile.
static void hwIdle()
{
  IOLockLock(gK.lock);
  const int slot = gK.shownSlot;
  bool go = !held() && gK.hwOn && slot >= 0 && slot < 2 && gK.slotVa[slot] && gK.va[0] && !gK.hwIdleBusy && clocksOn();
  const unsigned kind = go ? gK.slotKind[slot] : 0;
  if (kind != 1 && kind != 2) go = false;
  if (kind == 2 && !gK.tapsValid) go = false;
  const uint64_t presents = gK.presents;
  const uint32_t sw = gK.ssKey[0], sh = gK.ssKey[1], stride = go ? gK.slotStride[slot] : 0;
  const uint32_t pw = kind == 2 ? gK.tapDims[2] : sw, ph = kind == 2 ? gK.tapDims[3] : sh;
  if (kind == 2 && (gK.tapDims[0] != sw || gK.tapDims[1] != sh)) go = false;
  if (go) gK.hwIdleBusy = true;
  IOLockUnlock(gK.lock);
  if (!go) return;
  const uint64_t t0 = ticks();
  bool drawn = false;
  const uint64_t need = bytesFor(sw, sh, pw, ph);
  if (need > gK.gsBytes) {
    if (gK.gsMem) IOFree(gK.gsMem, gK.gsBytes);
    gK.gsMem = IOMalloc((vm_size_t)need);
    gK.gsBytes = gK.gsMem ? (vm_size_t)need : 0;
  }
  const uint32_t *columns = kind == 2 ? gK.idleTaps : nullptr, *rows = kind == 2 ? gK.idleTaps + pw : nullptr;
  if (gK.gsMem && layout(gK.gs, gK.gsMem, gK.gsBytes, sw, sh, pw, ph, columns, rows)) {
    // The slot is write-combining memory: slow to read, but this runs once per idle period, outside the lock.
    const u8 *from = const_cast<const u8 *>(reinterpret_cast<const volatile u8 *>(gK.slotVa[slot]));
    scan(gK.gs, from, stride, false);
    accumulate(gK.gs);
    draw(gK.gs, 0, gK.va[0], gK.stride, from, stride, gK.neon ? neonStore : nullptr);
    __asm__ volatile("dsb sy" ::: "memory");
    drawn = true;
  }
  IOLockLock(gK.lock);
  gK.hwIdleBusy = false;
  if (drawn && presents == gK.presents && gK.hwOn && gK.shownSlot == slot && !held() && clocksOn() && waitLatch()) {
    pointGop();
    gK.hwIdleBacks = gK.hwIdleBacks + 1;
  } else {
    gK.hwIdleAborts = gK.hwIdleAborts + 1;
  }
  gK.hwIdleTicks = gK.hwIdleTicks + (ticks() - t0);
  IOLockUnlock(gK.lock);
}

// Lock held: lets an idle return that is drawing the GOP frame finish before someone else writes it (at most ~200 ms).
static void waitIdleDone()
{
  for (unsigned i = 0; gK.hwIdleBusy && i < 200; ++i) { IOLockUnlock(gK.lock); IOSleep(1); IOLockLock(gK.lock); }
}

// The hardware scaling probe, after the pipe was found (read only; logged once). VIG0 only (DMA0 has no scaler).
static void probeHw(uint64_t gopPa, uint64_t frame, uint32_t w, uint32_t h)
{
  using namespace gscanhw;
  gK.hwProbe = -1;
  const uint32_t pipe = kPipes[gK.pipe].offset;
  if (gK.pipe != 0) gK.hwProbe = -2;
  if (gK.hwProbe == -1) {
    gK.scVersion = rd(pipe + kScaler + kScVersion);
    gK.outSize = rd(pipe + kOutSize);
    gK.srcFormat = rd(pipe + kSrcFormat);
    for (unsigned i = 0; i < kSavedCount; ++i) gK.saved[i] = rd(pipe + kSaved[i]);
    if (!liteScaler(gK.scVersion)) gK.hwProbe = -3;
    else if (gK.outSize != ((h << 16) | w)) gK.hwProbe = -4;
    else if ((gK.srcFormat & 0xff) != 0xff || ((gK.srcFormat >> 9) & 3) != 3) gK.hwProbe = -5;
  }
  if (gK.hwProbe == -1 && !slots(gK.slots, gopPa, frame, 0x500000, 0x500000, gopPa + 0xF00000)) gK.hwProbe = -6;
  for (unsigned t = 0; gK.hwProbe == -1 && t < 2; ++t) {
    IOMemoryDescriptor *range = IOMemoryDescriptor::withPhysicalAddress(gK.slots.pa[t], gK.slots.size, kIODirectionInOut);
    IOMemoryMap *map = range ? range->map(kIOMapWriteCombineCache) : nullptr;
    OSSafeReleaseNULL(range);
    if (!map || !map->getVirtualAddress()) { OSSafeReleaseNULL(map); gK.hwProbe = -7; }
    else gK.slotVa[t] = reinterpret_cast<volatile uint8_t *>(map->getVirtualAddress());   // kept for good
  }
  if (gK.hwProbe == -1 && !gK.hwIdleCall)
    gK.hwIdleCall = thread_call_allocate_with_options([](thread_call_param_t, thread_call_param_t) { hwIdle(); }, nullptr,
                                                      THREAD_CALL_PRIORITY_KERNEL, 0);
  if (gK.hwProbe == -1) { gK.hwProbe = 1; gK.hwOn = true; gK.shownSlot = -1; }
  uint32_t req = 0, op = 0, scOp = 0;
  for (unsigned i = 0; i < kSavedCount; ++i) {
    if (kSaved[i] == kPeC0Req) req = gK.saved[i];
    if (kSaved[i] == kSrcOpMode) op = gK.saved[i];
    if (kSaved[i] == kScaler + kScOpMode) scOp = gK.saved[i];
  }
  IOLog("GuampScanout: hardware scaling %s (probe %d): scaler version %x op mode %x, out %ux%u, format %x, src op mode %x, "
        "pe req %x, slots %llx %llx (%llu bytes), idle %u ms\n", gK.hwOn ? "on" : "off", gK.hwProbe, gK.scVersion, scOp,
        gK.outSize & 0xffff, gK.outSize >> 16, gK.srcFormat, op, req, gK.slots.pa[0], gK.slots.pa[1], gK.slots.size, gK.hwIdleMs);
}

// Finds the source pipe that scans the GOP frame (read only). Logged once.
static void probeFlip(volatile uint8_t *gopVa, uint64_t gopPa, uint64_t stride, uint32_t w, uint32_t h, uint64_t gopBytes)
{
  gK.probe = -1;
  const uint64_t frame = stride * h;
  const uint64_t back = gopPa + gK.flipOffset;
  if (!gopVa || gopPa < kRegionBase || back < gopPa + frame || back + frame > kRegionEnd || (back & 0xfff) || (gopPa >> 32)) {
    gK.probe = -2;
  } else {
    gK.gccDisp = mapPhysical(guamp_backlight::kGccDispAhbPage, 0x1000);
    gK.dispcc = mapPhysical(guamp_backlight::kDispccPhys, 0x3000);
    gK.mdss = mapPhysical(kMdssPhys, kMdssBytes);
    if (!gK.mdss || !clocksOn()) gK.probe = -3;
  }
  if (gK.probe == -1) {
    for (unsigned i = 0; i < sizeof(kPipes) / sizeof(kPipes[0]); ++i) {
      const uint32_t addr = rd(kPipes[i].offset + kSrc0Addr), ystride = rd(kPipes[i].offset + kSrcYStride0) & 0xffff;
      const uint32_t size = rd(kPipes[i].offset + kSrcSize);
      IOLog("GuampScanout: pipe %s src0 %x ystride %u size %ux%u format %x current %x\n", kPipes[i].name, addr, ystride,
            size & 0xffff, size >> 16, rd(kPipes[i].offset + kSrcFormat), rd(kPipes[i].offset + kCurrentSrc0));
      if (addr == (uint32_t)gopPa && ystride == stride && (size & 0xffff) == w && (size >> 16) == h) gK.pipe = (int)i;
    }
    const uint32_t layer = rd(kCtl0 + kCtlLayer0), ext = rd(kCtl0 + kCtlLayerExt0), flush = rd(kCtl0 + kCtlFlush);
    gK.probeRegs[0] = layer; gK.probeRegs[1] = ext; gK.probeRegs[2] = flush;
    if (gK.pipe < 0) gK.probe = -4;
    else if (!layer && !ext) gK.probe = -5;              // CTL_0 mixes nothing: not the active path
    else {
      unsigned withCtl = 1;
      PE_parse_boot_argn("guamp-flip-flush-ctl", &withCtl, sizeof(withCtl));
      gK.flushMask = kPipes[gK.pipe].flush | (withCtl ? kFlushCtl : 0);
      if (flush & gK.flushMask) gK.probe = -6;   // a flush still pending from the boot loader
    }
  }
  if (gK.probe == -1) {
    IOMemoryDescriptor *range = IOMemoryDescriptor::withPhysicalAddress(back, frame, kIODirectionInOut);
    IOMemoryMap *map = range ? range->map(kIOMapWriteCombineCache) : nullptr;
    OSSafeReleaseNULL(range);
    if (!map || !map->getVirtualAddress()) { OSSafeReleaseNULL(map); gK.probe = -7; }
    else { gK.va[1] = reinterpret_cast<volatile uint8_t *>(map->getVirtualAddress()); gK.pa[1] = back; }   // kept for good
  }
  if (gK.probe == -1) {
    // fin37 (03/11): va[0] was never set and the idle catch-up wrote the GOP frame at address 0 (panic in the thread call).
    gK.va[0] = gopVa; gK.pa[0] = gopPa; gK.stride = stride; gK.probe = 1; gK.flipOn = true; gK.shown = 0;
    if (!gK.idleCall)
      gK.idleCall = thread_call_allocate_with_options([](thread_call_param_t, thread_call_param_t) {
        IOLockLock(gK.lock);
        // No frame since the flip to the second buffer: the GOP frame catches up from it and is shown again.
        if (!held() && gK.flipOn && gK.shown == 1 && gK.va[0] && gK.va[1] && clocksOn() && waitLatch()) {
          catchUp(gK.st, 0, gK.va[0], gK.va[1], gK.stride);
          __asm__ volatile("dsb sy" ::: "memory");
          point(0);
          gK.idleBacks = gK.idleBacks + 1;
        }
        IOLockUnlock(gK.lock);
      }, nullptr, THREAD_CALL_PRIORITY_KERNEL_HIGH, 0);
  }
  if (gK.probe == 1 && gK.hwWanted) probeHw(gopPa, frame, w, h);
  IOLog("GuampScanout: double buffering %s (probe %d): GOP %llx stride %llu %ux%u (%llu bytes), second %llx, pipe %s, "
        "ctl0 layer %x ext %x flush %x\n", gK.probe == 1 ? "on" : "off", gK.probe, gopPa, stride, w, h, gopBytes, gK.pa[1],
        gK.pipe >= 0 ? kPipes[gK.pipe].name : "-", gK.probeRegs[0], gK.probeRegs[1], gK.probeRegs[2]);
}

// The geometry changed (or first frame): working memory, taps, everything invalid.
static bool setup(uint32_t sw, uint32_t sh, uint32_t w, uint32_t h, uint64_t srcStride, const uint32_t *columns, const uint32_t *rows)
{
  if (gK.ready && gK.keyColumns == columns && gK.keyRows == rows && gK.keyDims[0] == sw && gK.keyDims[1] == sh &&
      gK.keyDims[2] == w && gK.keyDims[3] == h && gK.keyStride == srcStride)
    return true;
  gK.ready = false;
  const uint64_t need = bytesFor(sw, sh, w, h);
  if (need > gK.memBytes) {
    if (gK.mem) IOFree(gK.mem, gK.memBytes);
    gK.mem = IOMalloc((vm_size_t)need);
    gK.memBytes = gK.mem ? (vm_size_t)need : 0;
  }
  if (!gK.mem || !layout(gK.st, gK.mem, gK.memBytes, sw, sh, w, h, columns, rows)) return false;
  gK.keyColumns = columns; gK.keyRows = rows; gK.keyDims[0] = sw; gK.keyDims[1] = sh; gK.keyDims[2] = w; gK.keyDims[3] = h;
  gK.keyStride = srcStride; gK.ready = true; gK.sinceFull = 0;
  return true;
}

static int refuse(unsigned why)
{
  gK.refused = gK.refused + 1; gK.lastRefusal = why;
  gK.ready = false;   // the variant's own copy writes the GOP frame: start again from nothing
  gK.ssReady = false;
  waitIdleDone();     // an idle return drawing the GOP frame must not race the variant's copy
  if (gK.flipOn) flipOff(why);
  return -1;
}

// guamp-scanout-hwscale, lock held, flip on: the frame copied 1:1 into the slot that is not shown, then the pipe pointed at it
// (with the scaler for a canvas frame). 1 = drawn and flipped; -1 = refused (back on the GOP frame, the variant copies).
static int presentHw(const u8 *src, uint64_t srcStride, uint32_t srcW, uint32_t srcH, uint32_t dstW, uint32_t dstH,
                     const uint32_t *columns, const uint32_t *rows)
{
  using namespace gscanhw;
  const bool canvas = columns && rows;
  if (!canvas && (srcW != dstW || srcH != dstH)) return refuse(10);
  const uint32_t stride = canvas ? gscanhw::slotStride(srcW) : (uint32_t)gK.stride;   // 1:1: the GOP stride (guamp_scanout_shown)
  if ((uint64_t)stride * srcH > gK.slots.size) return refuse(11);
  Config cfg;
  const bool ok = canvas ? scaled(cfg, gK.saved, gK.outSize, gK.srcFormat, srcW, srcH, stride)
                         : panel(cfg, gK.saved, gK.outSize, srcW, srcH, stride);
  if (!ok) return refuse(12);
  const uint32_t key[4] = {canvas ? 2u : 1u, srcW, srcH, stride};
  // The 1:1 copy state, per geometry (a change redraws both slots whole).
  if (!gK.ssReady || gK.ssKey[0] != srcW || gK.ssKey[1] != srcH || gK.ssKey[2] != key[0] || gK.ssStride != srcStride) {
    gK.ssReady = false;
    const uint64_t need = bytesFor(srcW, srcH, srcW, srcH);
    if (need > gK.ssBytes) {
      if (gK.ssMem) IOFree(gK.ssMem, gK.ssBytes);
      gK.ssMem = IOMalloc((vm_size_t)need);
      gK.ssBytes = gK.ssMem ? (vm_size_t)need : 0;
    }
    if (!gK.ssMem || !layout(gK.ss, gK.ssMem, gK.ssBytes, srcW, srcH, srcW, srcH, nullptr, nullptr)) return refuse(13);
    gK.ssKey[0] = srcW; gK.ssKey[1] = srcH; gK.ssKey[2] = key[0]; gK.ssStride = srcStride; gK.ssReady = true;
    gK.sinceFull = 0;
  }
  // The taps of the idle return (the variant's tables, copied: they may be freed with its scaler).
  if (canvas && !gK.hwIdleBusy && (!gK.tapsValid || gK.tapDims[0] != srcW || gK.tapDims[1] != srcH || gK.tapDims[2] != dstW ||
                                   gK.tapDims[3] != dstH)) {
    gK.tapsValid = false;
    const vm_size_t need = ((vm_size_t)dstW + dstH) * 4;
    if (need > gK.idleTapBytes) {
      if (gK.idleTaps) IOFree(gK.idleTaps, gK.idleTapBytes);
      gK.idleTaps = static_cast<uint32_t *>(IOMalloc(need));
      gK.idleTapBytes = gK.idleTaps ? need : 0;
    }
    if (gK.idleTaps) {
      for (uint32_t x = 0; x < dstW; ++x) gK.idleTaps[x] = columns[x];
      for (uint32_t y = 0; y < dstH; ++y) gK.idleTaps[dstW + y] = rows[y];
      gK.tapDims[0] = srcW; gK.tapDims[1] = srcH; gK.tapDims[2] = dstW; gK.tapDims[3] = dstH; gK.tapsValid = true;
    }
  }
  if (gK.fullEvery && ++gK.sinceFull >= gK.fullEvery) {
    gK.sinceFull = 0; gK.ss.hashValid = false; gK.ss.bufValid[0] = gK.ss.bufValid[1] = false; gK.full = gK.full + 1;
  }
  const unsigned target = gK.shownSlot == 0 ? 1 : 0;
  // A slot that holds another kind of frame (or another stride) is redrawn whole.
  for (unsigned t = 0; t < 2; ++t)
    if (gK.slotKind[t] != key[0] || gK.slotStride[t] != stride) gK.ss.bufValid[t] = false;
  const uint64_t t0 = ticks();
  const u32 changed = scan(gK.ss, src, srcStride, gK.diff);
  accumulate(gK.ss);
  const uint64_t t1 = ticks();
  if (!waitLatch()) return refuse(5);   // the slot just left may still be scanned: never draw into it
  draw(gK.ss, target, gK.slotVa[target], stride, src, srcStride, gK.neon ? neonStore : nullptr);
  __asm__ volatile("dsb sy" ::: "memory");
  const uint64_t t2 = ticks();
  gK.slotKind[target] = key[0];
  gK.slotStride[target] = stride;
  if (gK.ss.rowsDrawn || gK.shownSlot < 0 || !sameKey(gK.cfgKey, key)) pointSlot(target, cfg, key);
  gK.st.bufValid[0] = false;   // the GOP frame is behind now (the idle return redraws it whole)
  gK.frames = gK.frames + 1;
  gK.hwFrames = gK.hwFrames + 1;
  if (canvas) gK.hwCanvas = gK.hwCanvas + 1; else gK.hwPanel = gK.hwPanel + 1;
  if (!changed) gK.unchanged = gK.unchanged + 1;
  gK.area = (uint64_t)srcW * srcH;
  if (gK.frames <= gK.earlyFrames)
    IOLog("GuampScanout: early frame %llu: luma %u, changed rows %u, drawn rows %llu box %u,%u-%u,%u, slot %u (%s)\n", gK.frames,
          meanLuma(src, srcStride, srcW, srcH), changed, gK.ss.rowsDrawn, gK.ss.box[0], gK.ss.box[1], gK.ss.box[2], gK.ss.box[3],
          target, canvas ? "scaled by the pipe" : "1:1");
  gK.rows = gK.rows + gK.ss.rowsDrawn; gK.pixels = gK.pixels + gK.ss.pixelsDrawn;
  if (gK.ss.rowsDrawn) for (unsigned i = 0; i < 4; ++i) gK.box[i] = gK.ss.box[i];
  gK.scanTicks = gK.scanTicks + (t1 - t0); gK.drawTicks = gK.drawTicks + (t2 - t1);
  gK.presents = gK.presents + 1;
  if (gK.hwIdleCall && gK.hwIdleMs) {
    uint64_t deadline = 0;
    clock_interval_to_deadline(gK.hwIdleMs, kMillisecondScale, &deadline);
    thread_call_enter_delayed(gK.hwIdleCall, deadline);   // re-armed by every frame
  }
  return 1;
}
}  // namespace gscan

extern "C" volatile uint64_t native_legacy_copy_n __attribute__((weak_import));

// The copy thread of the legacy display (one call at a time). dst: the GOP frame (dstPhys), src: the swapped surface (kernel
// mapping, cache maintained by the variant). columns/rows: the canvas scaler taps, null when unscaled.
extern "C" int guamp_scanout_present(volatile void *dst, uint64_t dstBytes, uint64_t dstStride, uint64_t dstPhys, const void *src,
                                     uint64_t srcBytes, uint64_t srcStride, uint32_t srcW, uint32_t srcH, uint32_t dstW, uint32_t dstH,
                                     const uint32_t *columns, const uint32_t *rows)
{
  using namespace gscan;
  readOptions();
  // guamp-backlight-wake-wait (GuampBacklightCore.h): a frame the variant copies itself counts by its source's luma (64 reads).
  const bool gateFrame = guamp_backlight::gWakeGate.waiting && src && srcW && srcH && srcStride * (srcH - 1) + (uint64_t)srcW * 4 <= srcBytes;
  if (!gK.diff && !gK.flipWanted) {
    if (gateFrame) guamp_backlight::wakeNoteFrame(meanLuma(reinterpret_cast<const u8 *>(src), srcStride, srcW, srcH));
    return 0;
  }
  if (held()) {
    if (gateFrame) guamp_backlight::wakeNoteFrame(meanLuma(reinterpret_cast<const u8 *>(src), srcStride, srcW, srcH));
    return 0;
  }
  IOLockLock(gK.lock);
  if (held()) { IOLockUnlock(gK.lock); return 0; }
  int result = 1;
  do {
    if (!dst || !src || ((uintptr_t)dst & 7) || ((uintptr_t)src & 7) || (dstStride & 7) || (srcStride & 7) ||
        dstStride < (uint64_t)dstW * 4 || srcStride < (uint64_t)srcW * 4 || dstStride * dstH > dstBytes ||
        srcStride * (srcH - 1) + (uint64_t)srcW * 4 > srcBytes || !srcH || !dstH) { result = refuse(1); break; }
    if (!setup(srcW, srcH, dstW, dstH, srcStride, columns, rows)) { result = refuse(2); break; }
    // Someone else wrote the GOP frame since our last frame (the variant's own copy, a blank fill counted by the copy counter).
    const uint64_t copies = &native_legacy_copy_n ? native_legacy_copy_n : 0;
    if (copies != gK.expectCopyN) gK.st.bufValid[0] = false;
    if (gK.flipWanted && !gK.probe) probeFlip(reinterpret_cast<volatile uint8_t *>(dst), dstPhys, dstStride, dstW, dstH, dstBytes);
    if (gK.flipOn && (dstPhys != gK.pa[0] || dst != gK.va[0] || dstStride != gK.stride)) flipOff(3);
    if (gK.flipOn && !clocksOn()) flipOff(4);
    if (gK.hwOn && gK.flipOn) {   // guamp-scanout-hwscale: 1:1 into a slot, the pipe scales
      result = presentHw(reinterpret_cast<const u8 *>(src), srcStride, srcW, srcH, dstW, dstH, columns, rows);
      gK.expectCopyN = copies + 1;
      if (result == 1 && gateFrame) guamp_backlight::wakeNoteFrame(meanLuma(reinterpret_cast<const u8 *>(src), srcStride, srcW, srcH));
      break;
    }
    gK.area = (uint64_t)gK.st.dstW * gK.st.dstH;
    if (gK.fullEvery && ++gK.sinceFull >= gK.fullEvery) {
      gK.sinceFull = 0; gK.st.hashValid = false; gK.st.bufValid[0] = gK.st.bufValid[1] = false; gK.full = gK.full + 1;
    }
    const uint64_t t0 = ticks();
    const u32 changed = scan(gK.st, reinterpret_cast<const u8 *>(src), srcStride, gK.diff);
    accumulate(gK.st);
    const uint64_t t1 = ticks();
    unsigned target = 0;
    if (gK.flipOn) {
      target = gK.shown ^ 1;
      if (!waitLatch()) { flipOff(5); target = 0; }   // the buffer just left may still be scanned: never draw into it
    }
    volatile u8 *to = target ? gK.va[1] : reinterpret_cast<volatile u8 *>(dst);
    draw(gK.st, target, to, dstStride, reinterpret_cast<const u8 *>(src), srcStride, gK.neon ? neonStore : nullptr);
    __asm__ volatile("dsb sy" ::: "memory");
    const uint64_t t2 = ticks();
    if (gK.flipOn && gK.st.rowsDrawn && target != gK.shown) point(target);
    gK.frames = gK.frames + 1;
    if (!changed) gK.unchanged = gK.unchanged + 1;
    // guamp-backlight-wake-wait: this frame is drawn (and, with the flip, pointed at): the light may come up with it.
    if (gateFrame) guamp_backlight::wakeNoteFrame(meanLuma(reinterpret_cast<const u8 *>(src), srcStride, srcW, srcH));
    // The first frames of the boot, one line each (the log line's time is when the panel got it): which picture reached the
    // screen when (boot logo / lock screen, 05/11: off ~35 s after the display power-on, ~20 s after the first frame).
    if (gK.frames <= gK.earlyFrames)
      IOLog("GuampScanout: early frame %llu: luma %u, changed rows %u, drawn rows %llu box %u,%u-%u,%u, buffer %u (shown %u)\n",
            gK.frames, meanLuma(reinterpret_cast<const u8 *>(src), srcStride, srcW, srcH), changed, gK.st.rowsDrawn,
            gK.st.box[0], gK.st.box[1], gK.st.box[2], gK.st.box[3], target, gK.shown);
    gK.rows = gK.rows + gK.st.rowsDrawn; gK.pixels = gK.pixels + gK.st.pixelsDrawn;
    if (gK.st.rowsDrawn) for (unsigned i = 0; i < 4; ++i) gK.box[i] = gK.st.box[i];
    gK.scanTicks = gK.scanTicks + (t1 - t0); gK.drawTicks = gK.drawTicks + (t2 - t1);
    gK.expectCopyN = copies + 1;   // the variant counts this frame after we return
    gK.presents = gK.presents + 1;
    if (gK.flipOn && gK.shown == 1 && gK.idleCall && gK.idleMs) {
      uint64_t deadline = 0;
      clock_interval_to_deadline(gK.idleMs, kMillisecondScale, &deadline);
      thread_call_enter_delayed(gK.idleCall, deadline);   // re-armed by every frame
    }
  } while (false);
  IOLockUnlock(gK.lock);
  // Refused: the variant copies it itself.
  if (result != 1 && gateFrame) guamp_backlight::wakeNoteFrame(meanLuma(reinterpret_cast<const u8 *>(src), srcStride, srcW, srcH));
  return result;
}

// Section 15 of the variant (guamp-compose-keep): the buffer the panel shows now (the second buffer while flipped), so a composed
// frame with an undrawable layer can start from the screen. 1 = va/stride set; 0 = no flip (the variant reads the GOP frame).
extern "C" int guamp_scanout_shown(const volatile unsigned char **va, uint64_t *stride)
{
  using namespace gscan;
  if (!va || !stride || !gK.read || !gK.flipOn || !gK.lock) return 0;
  IOLockLock(gK.lock);
  const unsigned at = gK.shown ? 1 : 0;
  int ok = gK.flipOn && gK.va[at] && gK.stride ? 1 : 0;
  if (ok) { *va = gK.va[at]; *stride = gK.stride; }
  // guamp-scanout-hwscale: a 1:1 slot is the screen as it is; a canvas slot is not panel-sized (the variant then reads the GOP
  // frame, as old as the last idle return).
  const int slot = gK.shownSlot;
  if (ok && gK.hwOn && slot >= 0 && slot < 2) {
    ok = gK.slotKind[slot] == 1 && gK.slotStride[slot] == gK.stride && gK.slotVa[slot] ? 1 : 0;
    if (ok) *va = gK.slotVa[slot];
  }
  IOLockUnlock(gK.lock);
  return ok;
}

// The variant is about to write the GOP frame itself (a blank swap): show it and redraw it whole afterwards.
extern "C" void guamp_scanout_external(void)
{
  using namespace gscan;
  if (!gK.read || (!gK.diff && !gK.flipWanted)) return;
  IOLockLock(gK.lock);
  gK.externals = gK.externals + 1;
  waitIdleDone();   // guamp-scanout-hwscale: the idle return must not draw the GOP frame under the variant's fill
  if (!held() && gK.flipOn && (gK.shown != 0 || slotShown()) && clocksOn() && waitLatch()) { pointGop(); waitLatch(); }
  gK.st.bufValid[0] = false;
  IOLockUnlock(gK.lock);
}

namespace gscan {
// guamp_emmc_flush (restart, power off, panic): the GOP frame is the scanout again, without waiting or locking.
static void restore()
{
  if (!gK.flipOn || (gK.shown == 0 && !slotShown()) || !gK.mdss || !clocksOn()) return;
  if (gK.cfgKey[0]) {   // guamp-scanout-hwscale: the boot loader's 1:1 setup again, with the same flush
    const uint32_t key[4] = {0, 0, 0, 0};
    writeConfig(gscanhw::original(gK.saved), key);
  }
  gK.shownSlot = -1;
  wr(kPipes[gK.pipe].offset + kSrc0Addr, (uint32_t)gK.pa[0]);
  wr(kCtl0 + kCtlFlush, gK.flushMask);
  gK.shown = 0;
}

void log()
{
  if (!gK.read || (!gK.diff && !gK.flipWanted)) return;
  static uint64_t last[8];
  const uint64_t now[8] = {gK.frames, gK.unchanged, gK.rows, gK.pixels, gK.flips, gK.latchUs, gK.scanTicks, gK.drawTicks};
  const uint64_t frames = now[0] - last[0];
  if (!frames && now[4] == last[4]) return;
  uint64_t f = 0;
  __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(f));
  if (!f) f = 19200000;
  const uint64_t area = gK.area;
  const uint64_t flips = now[4] - last[4];
  IOLog("GuampScanout: frames +%llu (unchanged +%llu), drawn %llu.%llu%% of the pixels (rows/frame %llu), scan %llu us draw %llu us "
        "per frame, last box %u,%u-%u,%u; diff %u neon %u full redraws %llu; flip %s: flips +%llu latch mean %llu us max %llu, "
        "timeouts %llu, idle returns %llu, externals %llu, refused %llu (last reason %u)\n",
        frames, now[1] - last[1], frames && area ? (now[3] - last[3]) * 100 / (frames * area) : 0,
        frames && area ? (now[3] - last[3]) * 1000 / (frames * area) % 10 : 0, frames ? (now[2] - last[2]) / frames : 0,
        frames ? (now[6] - last[6]) / frames * 1000000 / f : 0, frames ? (now[7] - last[7]) / frames * 1000000 / f : 0,
        gK.box[0], gK.box[1], gK.box[2], gK.box[3], gK.diff, gK.neon, gK.full, gK.flipOn ? "on" : gK.flipWanted ? "off" : "not asked",
        flips, flips ? (now[5] - last[5]) / flips : 0, gK.latchMax, gK.latchTimeouts, gK.idleBacks, gK.externals, gK.refused,
        gK.lastRefusal);
  if (gK.hwWanted) {
    static uint64_t lastHw[6];
    const uint64_t hw[6] = {gK.hwFrames, gK.hwCanvas, gK.hwPanel, gK.hwIdleBacks, gK.hwIdleAborts, gK.hwIdleTicks};
    const uint64_t backs = (hw[3] - lastHw[3]) + (hw[4] - lastHw[4]);
    IOLog("GuampScanout: hardware scaling %s (probe %d): frames +%llu (scaled by the pipe +%llu, 1:1 +%llu), slot shown %d, "
          "setups %llu; idle returns +%llu given up +%llu, %llu us each\n", gK.hwOn ? "on" : "off", gK.hwProbe, hw[0] - lastHw[0],
          hw[1] - lastHw[1], hw[2] - lastHw[2], gK.shownSlot, gK.hwConfigs, hw[3] - lastHw[3], hw[4] - lastHw[4],
          backs ? (hw[5] - lastHw[5]) / backs * 1000000 / f : 0);
    for (unsigned i = 0; i < 6; ++i) lastHw[i] = hw[i];
  }
  for (unsigned i = 0; i < 8; ++i) last[i] = now[i];
}
}  // namespace gscan
