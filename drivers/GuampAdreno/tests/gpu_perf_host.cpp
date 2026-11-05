// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of guamp-gpu-perf's accounting and line text (GuampGpuPerfCore.h): slots per process, the job tags across a change of
// owner, the CP samples (bad ones refused), the frame report markers (adreno-metal 0023's encoding) and the two lines.
#include "../GuampGpuPerfCore.h"
#include <cstdio>
#include <cstring>
static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %d %s\n", __LINE__, #c); ++failures; } } while (0)
// What the bridge sends for one field (adreno-metal 0023 perfSend).
static void send(GpState *s, int slot, unsigned field, uint32_t value) {
  if (value > 0xfffff) value = 0xfffff;
  gp_mark(s, slot, 0x3000u | field << 8 | value >> 12);
  gp_mark(s, slot, 0x2000u | (value & 0xfff));
}
int main() {
  static GpState s;
  gp_init(&s);
  // Slots: one per pid, the name kept from the first claim; a ninth process takes the least recently active slot.
  const int bb = gp_claim(&s, 61, "backboardd");
  const int maps = gp_claim(&s, 812, "Maps");
  CHECK(bb >= 0 && maps >= 0 && bb != maps);
  CHECK(gp_find(&s, 61) == bb && gp_claim(&s, 61, "other") == bb && !std::strcmp(s.p[bb].name, "backboardd"));
  CHECK(gp_find(&s, 9999) < 0);
  const int32_t mapsTag = gp_tag(&s, maps);
  for (int32_t pid = 100; pid < 106; ++pid) gp_claim(&s, pid, "App");
  gp_claim(&s, 61, nullptr);           // backboardd active again: Maps is now the oldest
  const int ninth = gp_claim(&s, 200, "Ninth");
  CHECK(ninth == maps && gp_find(&s, 812) < 0);
  // A job of Maps released after its slot went to another process is not counted for the new owner.
  gp_job(&s, mapsTag, 0, 100, 200, 5);
  CHECK(s.p[ninth].jobs == 0 && s.jobs == 0);
  // Jobs: GPU time end - start, queue start - submitted, the longest; a job without samples counts but is not timed.
  const int32_t bbTag = gp_tag(&s, bb);
  gp_job(&s, bbTag, 1000, 1000 + 19200, 1000 + 19200 + 96000, 300);   // queue 1 ms, gpu 5 ms
  gp_job(&s, bbTag, 0, 0, 0, 100);                                     // no samples
  gp_job(&s, bbTag, 5, 50, 10, 0);                                     // end before start: refused
  gp_job(&s, bbTag, 5, 10, 10 + 3000 * kGpTicksPerMs, 0);              // 3 s: refused
  CHECK(s.p[bb].jobs == 4 && s.p[bb].timed == 1 && s.badSamples == 2);
  CHECK(s.p[bb].gpuTicks == 96000 && s.p[bb].queueTicks == 19200 && s.p[bb].gpuMaxTicks == 96000 && s.p[bb].kernelUs == 400);
  CHECK(s.busyTicks == 96000 && s.jobs == 4);
  // Markers: a report counts only once it ends; a low half without its start is dropped; fields saturate at 20 bits.
  send(&s, bb, GpFieldFrames, 2);
  send(&s, bb, GpFieldCpu, 620);       // 6.2 ms
  CHECK(s.p[bb].reports == 0 && s.p[bb].user[GpFieldFrames] == 0);
  send(&s, bb, kGpFieldEnd, 2);
  CHECK(s.p[bb].reports == 1 && s.p[bb].user[GpFieldFrames] == 2 && s.p[bb].user[GpFieldCpu] == 620);
  gp_mark(&s, bb, 0x2005);
  CHECK(s.lostReports == 1);
  send(&s, bb, GpFieldLost, 1);
  send(&s, bb, GpFieldCommits, 4);
  send(&s, bb, GpFieldC2c, 3200);      // 32 ms over 4 command buffers
  send(&s, bb, GpFieldWait, 0x123456); // saturates
  send(&s, bb, kGpFieldEnd, 4);
  CHECK(s.p[bb].reports == 2 && s.p[bb].user[GpFieldLost] == 1 && s.p[bb].user[GpFieldWait] == 0xfffff);
  gp_mark(&s, bb, 0x1fff);
  gp_mark(&s, bb, 0x4000);
  gp_mark(&s, -1, 0x3000);
  CHECK(s.p[bb].reports == 2);
  // The lines.
  GpProc out[kGpProcs];
  uint64_t busy = 0;
  uint32_t jobs = 0;
  const unsigned n = gp_take(&s, out, &busy, &jobs);
  CHECK(n == 1 && busy == 96000 && jobs == 4 && !std::strcmp(out[0].name, "backboardd"));
  CHECK(s.p[bb].jobs == 0 && s.p[bb].reports == 0 && s.busyTicks == 0 && s.p[bb].user[GpFieldFrames] == 0);
  char line[400];
  GpBoard b = {};
  b.spanUs = 2000000;
  b.rbbmBusy = 760;
  b.rbbmSamples = 2000;
  b.mhzSamples = 2000ull * 702;
  b.mhzNow = 745;
  gp_format_gpu(line, sizeof line, busy, jobs, b, 2);
  std::printf("%s\n", line);
  CHECK(!std::strcmp(line, "2.0s gpu 0% (ts) 38% (rbbm) | 702 MHz (now 745) | jobs 4 bad samples 2"));
  b.scan = true;
  b.copies = 2;
  b.copyWaitUs = 4000;
  b.copyMapUs = 800;
  b.swaps = 2;
  b.swapDone = 2;
  b.swapDoneUs = 18000;
  gp_format_proc(line, sizeof line, out[0], b.spanUs, &b);
  std::printf("%s\n", line);
  CHECK(!std::strcmp(line, "backboardd[61] fr 2 lost 1 | cpu 3.10 res 0.00 wait 5242.88 c2c 8.00 comp 0.00 cb 4 | gpu 2.50 queue 0.50 "
                           "max 5.00 jobs 4/1 kern 0.20 | scan copies 2 wait 2.00 map 0.40 swaps 2 swap>done 9.00 late 0 [ms/fr]"));
  // Without frames: per second of the interval; without a report: said so; no governor: fixed clock.
  GpProc quiet = {};
  std::strcpy(quiet.name, "Ninth");
  quiet.pid = 200;
  quiet.jobs = quiet.timed = 2;
  quiet.gpuTicks = 2 * kGpTicksPerMs * 10;   // 20 ms over 2 s
  gp_format_proc(line, sizeof line, quiet, 2000000, nullptr);
  std::printf("%s\n", line);
  CHECK(!std::strcmp(line, "Ninth[200] (no frame report) | gpu 10.00 queue 0.00 max 0.00 jobs 2 kern 0.00 [ms per s]"));
  GpBoard fixed = {};
  fixed.spanUs = 1000000;
  fixed.mhzNow = 320;
  gp_format_gpu(line, sizeof line, 19200 * 250, 9, fixed, 0);
  CHECK(!std::strcmp(line, "1.0s gpu 25% (ts) -% (rbbm) | fixed (now 320) | jobs 9"));
  // A short buffer is cut, never overrun.
  char small[12];
  std::memset(small, 'x', sizeof small);
  gp_format_proc(small, 8, out[0], b.spanUs, &b);
  CHECK(std::strlen(small) == 7 && small[8] == 'x');
  std::printf("gpu perf: %s\n", failures ? "FAIL" : "PASS");
  return failures ? 1 : 0;
}
