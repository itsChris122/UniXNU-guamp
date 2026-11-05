// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp-gpu-perf=<seconds> (default 0 = off; 1..60): where a frame's time goes, one "GuampGpuPerf:" line per interval for the GPU
// and one per Metal process that submitted or reported in it (included by GuampAdrenoProbe.cpp; the arithmetic and the line text
// are GuampGpuPerfCore.h, host-tested by tests/gpu_perf_host.cpp). With the argument:
//   - patches/native-adreno/0019 has every A6XX user job copy CP_ALWAYS_ON_COUNTER (19.2 MHz) after its prologue and after its IBs
//     (8-9 ring words, a CP_WAIT_FOR_IDLE moved from the next job's prologue), the kernel reads the counter when it takes the job
//     (three register reads) and accounts each job when it is released;
//   - "NMB_PERF=1" is added to NativeMetalEnv (guamp-m2v-env), so every Metal process's bridge (patches/adreno-metal/0023) sends
//     its frame numbers once a second: about 20 refused IOServiceOpen calls a second per process that draws, none while idle;
//   - nothing is printed for an interval without jobs or reports.
// Lines (times in ms, two decimals):
//   GuampGpuPerf: 2.0s gpu 41% (ts) 38% (rbbm) | 702 MHz (now 745) | jobs 240
//       ts: the jobs' own GPU time (end - start samples) over the interval; rbbm: the governor's RBBM_STATUS samples (GuampGpuDcvs.inc,
//       "-" without the governor); MHz: the governor's time-weighted clock, "fixed" without it; jobs: user jobs released.
//   GuampGpuPerf: backboardd[61] fr 118 lost 3 | cpu 3.10 res 0.20 wait 4.00 c2c 8.40 comp 0.00 cb 236 | scanout 118 cost 1.50
//       wait 0.50 | gpu 5.20 queue 1.10
//       max 9.80 jobs 236 kern 0.31 | scan copies 118 wait 2.00 map 0.40 swaps 118 swap>done 9.00 late 0 [ms/fr]
//     fr/lost  frames and skipped vsync periods inside animations (the bridge's count, adreno-metal 0023)
//     cpu      time inside the bridge's calls (encode, resources, commit); res: its waits for resources still in GPU use;
//     wait     the process's own waits on command buffers (waitUntilCompleted/Scheduled) beyond those; c2c: commit to completion
//              handler per command buffer (not per frame); comp: shader compiles; cb: command buffers
//     gpu      GPU time from the CP samples; queue: from the kernel taking the job to the CP starting it (waits behind other jobs);
//              max: the longest job; jobs: released (a "/n" says only n carried samples); kern: the kernel's time in submit()
//     scanout  the bridge's GPU scanout copy (backboardd's GpuScanoutPresenter): copies, cost (import, texture, encode, submit,
//              wait) and its wait for completion, per copy
//     scan     backboardd only (guamp-canvas-variant.py counters): CPU copies of the swapped surface, the wait for the GPU before
//              each (guamp-copy-gpu-wait) and the map + cache maintenance, CA swaps, swap to completion, late vsync messages
//   [ms/fr] the times are per frame; "[ms per s]" when the process reported no frame (then per second of the interval).
// Reading: a frame takes about cpu + res + wait on the CPU; gpu + queue on the GPU; for the screen, backboardd's scan wait/map
// come after. fr below 60 per second with lost > 0 is the jank; which column is large says where to look.

extern "C" {
// GuampGpuDcvs.inc (NativeEmmc): the clock now, and [0] busy samples, [1] samples, [2] MHz x samples; weak stand-ins (zero).
__attribute__((weak)) volatile uint32_t guamp_gpu_mhz_now;
__attribute__((weak)) volatile uint64_t guamp_gpu_dcvs_busy[3];
// scripts/guamp-canvas-variant.py (the legacy display): the copy's GPU waits, its map split, the pacing counters.
__attribute__((weak)) volatile uint64_t guamp_copy_gpu_wait[8];
__attribute__((weak)) volatile uint64_t guamp_copy_split[16];
__attribute__((weak)) volatile uint64_t guamp_pacing[24];
}

namespace gpuperf {
GpState gS;
IOLock *gLock;
unsigned gSeconds;

int32_t tagFor(int32_t pid) {
  IOLockLock(gLock);
  int slot = gp_find(&gS, pid);
  if (slot >= 0) gS.p[slot].lastActive = ++gS.clock;
  int32_t tag = gp_tag(&gS, slot);
  IOLockUnlock(gLock);
  if (tag >= 0) return tag;
  char name[17] = {};
  proc_selfname(name, sizeof name);
  IOLockLock(gLock);
  slot = gp_claim(&gS, pid, name);
  tag = gp_tag(&gS, slot);
  IOLockUnlock(gLock);
  return tag;
}
// native_adreno_perf_begin: in the submitting process, under the GPU lock.
int32_t begin(uint64_t *ticks) {
  *ticks = 0;
  const int32_t tag = tagFor(proc_selfpid());
  if (gTimer && gTimer->read(ticks)) *ticks = 0;
  return tag;
}
// native_adreno_perf_end: a job released (any process, under the GPU lock).
void end(int32_t tag, uint64_t submitted, uint64_t start, uint64_t finish, uint32_t kernelUs) {
  IOLockLock(gLock);
  gp_job(&gS, tag, submitted, start, finish, kernelUs);
  IOLockUnlock(gLock);
}
// native_adreno_perf_mark: a frame report marker, in the sending process.
void mark(uint32_t value) {
  const int32_t tag = tagFor(proc_selfpid());
  IOLockLock(gLock);
  gp_mark(&gS, tag < 0 ? -1 : int(tag & 15), value);
  IOLockUnlock(gLock);
}
uint64_t absUs(uint64_t t) {
  uint64_t ns = 0;
  absolutetime_to_nanoseconds(t, &ns);
  return ns / 1000;
}

void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-gpu-perf");
  uint64_t lastAt = mach_absolute_time();
  uint64_t dcvs[3] = {guamp_gpu_dcvs_busy[0], guamp_gpu_dcvs_busy[1], guamp_gpu_dcvs_busy[2]};
  uint64_t scan[7] = {};
  static GpProc procs[kGpProcs];
  static char line[400];
  for (;;) {
    IOSleep(gSeconds * 1000);
    const uint64_t now = mach_absolute_time();
    GpBoard b = {};
    b.spanUs = absUs(now - lastAt);
    lastAt = now;
    const uint64_t d[3] = {guamp_gpu_dcvs_busy[0], guamp_gpu_dcvs_busy[1], guamp_gpu_dcvs_busy[2]};
    b.rbbmBusy = d[0] - dcvs[0];
    b.rbbmSamples = d[1] - dcvs[1];
    b.mhzSamples = d[2] - dcvs[2];
    for (unsigned i = 0; i < 3; ++i) dcvs[i] = d[i];
    b.mhzNow = guamp_gpu_mhz_now;
    const uint64_t s[7] = {guamp_copy_split[0], guamp_copy_gpu_wait[3], guamp_copy_split[1], guamp_pacing[0], guamp_pacing[3],
                           guamp_pacing[4], guamp_pacing[10]};
    b.copies = s[0] - scan[0];
    b.copyWaitUs = s[1] - scan[1];
    b.copyMapUs = absUs(s[2] - scan[2]);
    b.swaps = s[3] - scan[3];
    b.swapDone = s[4] - scan[4];
    b.swapDoneUs = absUs(s[5] - scan[5]);
    b.vsyncLate = s[6] - scan[6];
    b.scan = b.copies || b.swaps;
    for (unsigned i = 0; i < 7; ++i) scan[i] = s[i];
    uint64_t busy = 0;
    uint32_t jobs = 0;
    IOLockLock(gLock);
    const uint32_t bad = gS.badSamples;
    gS.badSamples = 0;
    const unsigned n = gp_take(&gS, procs, &busy, &jobs);
    IOLockUnlock(gLock);
    if (!n && !jobs) continue;
    gp_format_gpu(line, sizeof line, busy, jobs, b, bad);
    IOLog("GuampGpuPerf: %s\n", line);
    for (unsigned i = 0; i < n; ++i) {
      const bool server = !strncmp(procs[i].name, "backboardd", sizeof procs[i].name);
      gp_format_proc(line, sizeof line, procs[i], b.spanUs, server ? &b : nullptr);
      IOLog("GuampGpuPerf: %s\n", line);
    }
  }
}

// At guamp_adreno_start, before the ring is attached (the ring drops the samples if its fence page cannot hold them).
void init() {
  gSeconds = bootArg("guamp-gpu-perf", 0);
  if (!gSeconds) return;
  if (gSeconds > 60) gSeconds = 60;
  gLock = IOLockAlloc();
  if (!gLock) {gSeconds = 0; return;}
  gp_init(&gS);
  native_adreno_perf_ts = 1;
  native_adreno_perf_begin = &begin;
  native_adreno_perf_end = &end;
  native_adreno_perf_mark = &mark;
  thread_t t;
  const bool started = kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS;
  if (started) thread_deallocate(t);
  IOLog("GuampGpuPerf: on, a line every %u s (job GPU samples, frame reports NMB_PERF=1)%s\n", gSeconds,
        started ? "" : "; no thread, nothing printed");
}
}  // namespace gpuperf
