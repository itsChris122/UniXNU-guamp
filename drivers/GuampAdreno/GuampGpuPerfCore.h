// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp-gpu-perf: the frame accounting behind the "GuampGpuPerf:" lines (GuampGpuPerf.inc), kept free of kernel calls so that
// tests/gpu_perf_host.cpp runs it on the host. Three inputs, all per process (pid; at most kGpProcs at a time, the least recently
// active one gives its slot up):
//   jobs    patches/native-adreno/0019: per user job the CP_ALWAYS_ON_COUNTER (19.2 MHz) when the kernel took it, the CP's own
//           samples after the job's prologue and after its IBs, and the kernel's microseconds in submit();
//   reports patches/adreno-metal/0023 (NMB_PERF): about once a second each Metal process sends its frame numbers as markers
//           0x2000..0x3fff (IOServiceOpen type 0x4147xxxx, the same refused open as the plugin's progress markers): a field is two
//           markers, 0x3000 | field << 8 | value >> 12, then 0x2000 | (value & 0xfff) (20 bits); field 15 ends the report, which then
//           counts. Fields: GpFieldFrames .. GpFieldScanWait, times in units of 10 us;
//   board   the GPU's busy share and clock from the governor (GuampGpuDcvs.inc) and the display copy's waits (passed in by the caller).
// gp_take() hands the interval's numbers out and starts the next interval; gp_format_*() write the lines (no printf needed).
#ifndef GUAMP_GPU_PERF_CORE_H
#define GUAMP_GPU_PERF_CORE_H
#include <stddef.h>
#include <stdint.h>

enum : unsigned {kGpProcs = 8, kGpFields = 12, kGpFieldEnd = 15};
enum : unsigned {
  GpFieldFrames = 0,    // frames presented (a drawable presented, or for a process that presents none a commit with a render pass)
  GpFieldLost = 1,      // vsync periods (16.7 ms) skipped between two frames of one animation (gaps under 100 ms)
  GpFieldCpu = 2,       // wall time inside the bridge's encoder, resource and commit calls
  GpFieldRes = 3,       // the bridge's own waits for a resource the GPU still uses
  GpFieldWait = 4,      // waitUntilCompleted / waitUntilScheduled by the process, minus the resource waits
  GpFieldC2c = 5,       // commit to completion handler, summed over the command buffers
  GpFieldCommits = 6,   // command buffers committed
  GpFieldWindow = 7,    // the report's own window in ms
  GpFieldCompile = 8,   // pipeline, library and function creation (shader compiles)
  GpFieldScanFrames = 9,  // the GPU scanout copy (backboardd's GpuScanoutPresenter): frames copied
  GpFieldScanCost = 10,   // its import, texture, encode, submit and wait, summed
  GpFieldScanWait = 11    // of which the wait for the copy's completion
};
constexpr uint64_t kGpTicksPerMs = 19200;   // CP_ALWAYS_ON_COUNTER on bengal: the 19.2 MHz XO

struct GpProc {
  int32_t pid;
  uint32_t gen;            // bumped when the slot changes owner (a job's tag carries it)
  char name[17];
  uint64_t lastActive;     // GpState::clock at the last job or marker
  // GPU jobs released in this interval
  uint32_t jobs, timed;
  uint64_t gpuTicks, queueTicks, gpuMaxTicks, kernelUs;
  // frame reports completed in this interval (sums)
  uint32_t reports;
  uint64_t user[kGpFields];
  // marker decoder
  uint32_t pending[kGpFields];
  int32_t field;
  uint32_t high;
};
struct GpState {
  GpProc p[kGpProcs];
  uint64_t clock;
  uint64_t busyTicks;      // GPU time of every timed job of the interval
  uint32_t jobs, badSamples, lostReports;
};
struct GpBoard {          // what the caller adds for the interval
  uint64_t spanUs;
  uint64_t rbbmBusy, rbbmSamples, mhzSamples;   // governor deltas (samples of 1 ms); rbbmSamples 0: no governor
  uint32_t mhzNow;
  bool scan;               // the display copy's counters exist
  uint64_t copies, copyWaitUs, copyMapUs, swaps, swapDone, swapDoneUs, vsyncLate;
};

inline void gp_init(GpState *s) {
  unsigned char *b = reinterpret_cast<unsigned char *>(s);
  for (size_t i = 0; i < sizeof(*s); ++i) b[i] = 0;
  for (unsigned i = 0; i < kGpProcs; ++i) s->p[i].field = -1;
}
inline int gp_find(const GpState *s, int32_t pid) {
  for (unsigned i = 0; i < kGpProcs; ++i)
    if (s->p[i].pid == pid && (s->p[i].name[0] || s->p[i].lastActive)) return int(i);
  return -1;
}
// The slot of pid, taken if it has none (an empty slot, else the least recently active one); name is copied for a new owner.
inline int gp_claim(GpState *s, int32_t pid, const char *name) {
  int i = gp_find(s, pid);
  if (i < 0) {
    unsigned pick = 0;
    for (unsigned k = 0; k < kGpProcs; ++k) {
      if (!s->p[k].name[0] && !s->p[k].lastActive) {pick = k; break;}
      if (s->p[k].lastActive < s->p[pick].lastActive) pick = k;
    }
    GpProc &p = s->p[pick];
    const uint32_t gen = p.gen + 1;
    unsigned char *b = reinterpret_cast<unsigned char *>(&p);
    for (size_t k = 0; k < sizeof(p); ++k) b[k] = 0;
    p.gen = gen;
    p.pid = pid;
    p.field = -1;
    unsigned n = 0;
    if (name) for (; n < 16 && name[n]; ++n) p.name[n] = name[n];
    if (!n) p.name[n++] = '?';
    p.name[n] = 0;
    i = int(pick);
  }
  s->p[i].lastActive = ++s->clock;
  return i;
}
// A job's tag: slot and generation, so a job released after its slot changed owner is not given to the new one.
inline int32_t gp_tag(const GpState *s, int slot) { return slot < 0 ? -1 : int32_t((s->p[slot].gen & 0xffffffu) << 4 | unsigned(slot)); }
inline void gp_job(GpState *s, int32_t tag, uint64_t submitted, uint64_t start, uint64_t end, uint32_t kernelUs) {
  if (tag < 0) return;
  const unsigned slot = unsigned(tag) & 15u;
  if (slot >= kGpProcs || (s->p[slot].gen & 0xffffffu) != (uint32_t(tag) >> 4)) return;
  GpProc &p = s->p[slot];
  ++s->jobs;
  ++p.jobs;
  p.kernelUs += kernelUs;
  if (!start && !end) return;
  if (end < start || end - start > 2000 * kGpTicksPerMs) {++s->badSamples; return;}   // a slot the CP never wrote, or wrapped
  const uint64_t gpu = end - start;
  ++p.timed;
  p.gpuTicks += gpu;
  s->busyTicks += gpu;
  if (gpu > p.gpuMaxTicks) p.gpuMaxTicks = gpu;
  if (submitted && start >= submitted && start - submitted < 2000 * kGpTicksPerMs) p.queueTicks += start - submitted;
}
// One marker value (0x2000..0x3fff) of the process in slot.
inline void gp_mark(GpState *s, int slot, uint32_t value) {
  if (slot < 0 || unsigned(slot) >= kGpProcs || value < 0x2000 || value >= 0x4000) return;
  GpProc &p = s->p[slot];
  const uint32_t v = value - 0x2000;
  if (v & 0x1000) {   // a field starts: its number and the value's top 8 bits
    p.field = int32_t((v >> 8) & 15);
    p.high = v & 0xff;
    return;
  }
  if (p.field < 0) {++s->lostReports; return;}   // a low half without its start
  const uint32_t full = p.high << 12 | (v & 0xfff);
  const unsigned f = unsigned(p.field);
  p.field = -1;
  if (f == kGpFieldEnd) {
    for (unsigned k = 0; k < kGpFields; ++k) {p.user[k] += p.pending[k]; p.pending[k] = 0;}
    ++p.reports;
  } else if (f < kGpFields) {
    p.pending[f] = full;
  }
}
// The interval's processes with jobs or reports into out (at most kGpProcs), then every interval counter back to 0.
inline unsigned gp_take(GpState *s, GpProc *out, uint64_t *busyTicks, uint32_t *jobs) {
  unsigned n = 0;
  *busyTicks = s->busyTicks;
  *jobs = s->jobs;
  for (unsigned i = 0; i < kGpProcs; ++i) {
    GpProc &p = s->p[i];
    if (p.jobs || p.reports) out[n++] = p;
    p.jobs = p.timed = p.reports = 0;
    p.gpuTicks = p.queueTicks = p.gpuMaxTicks = p.kernelUs = 0;
    for (unsigned k = 0; k < kGpFields; ++k) p.user[k] = 0;
  }
  s->busyTicks = 0;
  s->jobs = 0;
  return n;
}

// ---- line writing ----
struct GpLine {
  char *b;
  size_t n, at;
  void put(const char *t) { while (*t && at + 1 < n) b[at++] = *t++; b[at] = 0; }
  void num(uint64_t v) {
    char d[24];
    unsigned k = 0;
    do {d[k++] = char('0' + v % 10); v /= 10;} while (v && k < sizeof d);
    while (k && at + 1 < n) b[at++] = d[--k];
    b[at] = 0;
  }
  void ms(uint64_t us) {   // microseconds as milliseconds with two decimals
    const uint64_t c = (us + 5) / 10;
    num(c / 100);
    put(".");
    if (c % 100 < 10) put("0");
    num(c % 100);
  }
};
inline uint64_t gp_ticks_us(uint64_t ticks) { return ticks * 1000 / kGpTicksPerMs; }
// "<name>[pid] fr F lost L | cpu C res R wait W c2c X comp Y | gpu G queue Q max M jobs J kern K [per frame]": with frames the
// times are per frame, without them per second (the line then says "per s").
inline size_t gp_format_proc(char *buf, size_t size, const GpProc &p, uint64_t spanUs, const GpBoard *scan) {
  GpLine l{buf, size, 0};
  if (size) buf[0] = 0;
  l.put(p.name);
  l.put("[");
  l.num(uint64_t(uint32_t(p.pid)));
  l.put("]");
  const uint64_t frames = p.user[GpFieldFrames];
  const bool perFrame = frames != 0;
  const uint64_t secs10 = spanUs / 100000 ? spanUs / 100000 : 1;   // the interval in tenths of a second
  auto per = [&](uint64_t us) { return perFrame ? us / frames : us * 10 / secs10; };
  if (p.reports) {
    l.put(" fr ");
    l.num(frames);
    l.put(" lost ");
    l.num(p.user[GpFieldLost]);
    l.put(" | cpu ");
    l.ms(per(p.user[GpFieldCpu] * 10));
    l.put(" res ");
    l.ms(per(p.user[GpFieldRes] * 10));
    l.put(" wait ");
    l.ms(per(p.user[GpFieldWait] * 10));
    l.put(" c2c ");
    const uint64_t commits = p.user[GpFieldCommits];
    l.ms(commits ? p.user[GpFieldC2c] * 10 / commits : 0);
    l.put(" comp ");
    l.ms(per(p.user[GpFieldCompile] * 10));
    l.put(" cb ");
    l.num(commits);
    if (const uint64_t copies = p.user[GpFieldScanFrames]) {   // per copied frame
      l.put(" | scanout ");
      l.num(copies);
      l.put(" cost ");
      l.ms(p.user[GpFieldScanCost] * 10 / copies);
      l.put(" wait ");
      l.ms(p.user[GpFieldScanWait] * 10 / copies);
    }
  } else {
    l.put(" (no frame report)");
  }
  l.put(" | gpu ");
  l.ms(per(gp_ticks_us(p.gpuTicks)));
  l.put(" queue ");
  l.ms(per(gp_ticks_us(p.queueTicks)));
  l.put(" max ");
  l.ms(gp_ticks_us(p.gpuMaxTicks));
  l.put(" jobs ");
  l.num(p.jobs);
  if (p.timed != p.jobs) {
    l.put("/");
    l.num(p.timed);
  }
  l.put(" kern ");
  l.ms(per(p.kernelUs));
  if (scan && scan->scan) {
    l.put(" | scan copies ");
    l.num(scan->copies);
    l.put(" wait ");
    l.ms(scan->copies ? scan->copyWaitUs / scan->copies : 0);
    l.put(" map ");
    l.ms(scan->copies ? scan->copyMapUs / scan->copies : 0);
    l.put(" swaps ");
    l.num(scan->swaps);
    l.put(" swap>done ");
    l.ms(scan->swapDone ? scan->swapDoneUs / scan->swapDone : 0);
    l.put(" late ");
    l.num(scan->vsyncLate);
  }
  l.put(perFrame ? " [ms/fr]" : " [ms per s]");
  return l.at;
}
// "<span>s gpu B% (ts) R% (rbbm) | M MHz (now N) | jobs J": B from the jobs' own CP samples, R the governor's RBBM_STATUS samples.
inline size_t gp_format_gpu(char *buf, size_t size, uint64_t busyTicks, uint32_t jobs, const GpBoard &b, uint32_t badSamples) {
  GpLine l{buf, size, 0};
  if (size) buf[0] = 0;
  l.num(b.spanUs / 1000000);
  l.put(".");
  l.num(b.spanUs / 100000 % 10);
  l.put("s gpu ");
  const uint64_t busyUs = gp_ticks_us(busyTicks);
  l.num(b.spanUs ? (busyUs * 100 + b.spanUs / 2) / b.spanUs : 0);
  l.put("% (ts) ");
  if (b.rbbmSamples) {
    l.num((b.rbbmBusy * 100 + b.rbbmSamples / 2) / b.rbbmSamples);
    l.put("% (rbbm) | ");
    l.num(b.mhzSamples / b.rbbmSamples);
    l.put(" MHz (now ");
  } else {
    l.put("-% (rbbm) | fixed (now ");
  }
  l.num(b.mhzNow);
  l.put(") | jobs ");
  l.num(jobs);
  if (badSamples) {
    l.put(" bad samples ");
    l.num(badSamples);
  }
  return l.at;
}
#endif
