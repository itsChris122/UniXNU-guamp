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
//                            shown half drawn. guamp-flip-idle-ms=<ms> (default 60): with no new frame the GOP frame catches
//                            up and is shown again (the USB mirror, the boot progress and the panic screen use it); a panic
//                            or a restart points the pipe back at once. Any doubt (clocks off, pipe not found, flush never
//                            taken in 50 ms) turns the flip off for the rest of the boot, back on the GOP frame.
// Logs: "GuampScanout:" at the first frame (mode, pipe probe) and in every display report (frames, drawn share, box, flips).
#include "GuampScanoutCore.h"

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

struct Kernel {
  bool read = false, diff = false, neon = false, flipWanted = false;
  unsigned fullEvery = 600, idleMs = 60;
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
  PE_parse_boot_argn("guamp-copy-diff-full", &gK.fullEvery, sizeof(gK.fullEvery));
  PE_parse_boot_argn("guamp-flip-idle-ms", &gK.idleMs, sizeof(gK.idleMs));
  if (gK.idleMs > 1000) gK.idleMs = 1000;
  uint32_t off = 0x500000;
  PE_parse_boot_argn("guamp-flip-offset", &off, sizeof(off)); gK.flipOffset = off;
  // The GPU presenter (sheng-fb-gpu=1) writes the GOP frame behind the copy: no flip, no diff for its frames.
  unsigned handoff = 0;
  PE_parse_boot_argn("sheng-fb-gpu", &handoff, sizeof(handoff));
  if (handoff == 1) gK.flipWanted = false;
  gK.lock = IOLockAlloc();
  if (!gK.lock) gK.diff = gK.flipWanted = false;
  gK.read = true;
}

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

// Back on the GOP frame for good (first doubt): pointed back and waited for, best effort.
static void flipOff(unsigned why)
{
  gK.lastRefusal = why;
  if (gK.flipOn && gK.mdss && clocksOn()) {
    if (gK.shown != 0) { gK.latchPending = false; point(0); }
    waitLatch();
  }
  gK.flipOn = false;
  gK.shown = 0;
  gK.st.bufValid[0] = false;
  IOLog("GuampScanout: double buffering off (reason %u), the GOP frame is the scanout again\n", why);
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
        if (gK.flipOn && gK.shown == 1 && gK.va[0] && gK.va[1] && clocksOn() && waitLatch()) {
          catchUp(gK.st, 0, gK.va[0], gK.va[1], gK.stride);
          __asm__ volatile("dsb sy" ::: "memory");
          point(0);
          gK.idleBacks = gK.idleBacks + 1;
        }
        IOLockUnlock(gK.lock);
      }, nullptr, THREAD_CALL_PRIORITY_KERNEL_HIGH, 0);
  }
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
  if (gK.flipOn) flipOff(why);
  return -1;
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
  if (!gK.diff && !gK.flipWanted) return 0;
  IOLockLock(gK.lock);
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
  return result;
}

// The variant is about to write the GOP frame itself (a blank swap): show it and redraw it whole afterwards.
extern "C" void guamp_scanout_external(void)
{
  using namespace gscan;
  if (!gK.read || (!gK.diff && !gK.flipWanted)) return;
  IOLockLock(gK.lock);
  gK.externals = gK.externals + 1;
  if (gK.flipOn && gK.shown != 0 && clocksOn() && waitLatch()) { point(0); waitLatch(); }
  gK.st.bufValid[0] = false;
  IOLockUnlock(gK.lock);
}

namespace gscan {
// guamp_emmc_flush (restart, power off, panic): the GOP frame is the scanout again, without waiting or locking.
static void restore()
{
  if (!gK.flipOn || gK.shown == 0 || !gK.mdss || !clocksOn()) return;
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
  const uint64_t area = (uint64_t)gK.st.dstW * gK.st.dstH;
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
  for (unsigned i = 0; i < 8; ++i) last[i] = now[i];
}
}  // namespace gscan
