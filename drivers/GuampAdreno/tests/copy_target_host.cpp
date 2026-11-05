// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of GuampCopyTarget.h (the fence the display copy waits for, fin49): off = the ring's newest; on = the later of the render
// server's last fence and the ring's newest at the last accepted swap, never past the ring's newest, across the 32-bit wrap. Then the
// in-order ring model of proc_fence_host.cpp's kind: a frame's surface is written by a job handed out before the swap that is not the
// render server's last one (an earlier render server job, or app work it samples); the copy must never read it before that job ran.
#include <stdint.h>
#include <stdio.h>
#include <vector>
#include "../GuampCopyTarget.h"

static int failures = 0;
#define CHECK(c) do {if (!(c)) {printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++failures;}} while (0)

static bool retired(uint32_t done, uint32_t fence) {return int32_t(done - fence) >= 0;}

int main() {
  CHECK(guampCopyTarget(100, 40, 90, false) == 100);
  CHECK(guampCopyTarget(100, 0, 0, true) == 100);    // nothing known: the newest
  CHECK(guampCopyTarget(100, 40, 0, true) == 40);    // 0017 alone
  CHECK(guampCopyTarget(100, 0, 70, true) == 70);
  CHECK(guampCopyTarget(100, 40, 90, true) == 90);   // the swap's mark is later
  CHECK(guampCopyTarget(100, 95, 90, true) == 95);   // the render server submitted after the swap: its fence
  CHECK(guampCopyTarget(100, 40, 120, true) == 100); // past the newest (stale state): the newest
  CHECK(guampCopyTarget(5, 0xfffffff0u, 3, true) == 3);           // across the wrap
  CHECK(guampCopyTarget(5, 4, 0xfffffffeu, true) == 4);
  CHECK(guampCopyTarget(0xfffffff8u, 0xfffffff0u, 0xfffffff4u, true) == 0xfffffff4u);

  // Ring model, 2000 frames: per frame an app job (the surface the render server samples) and up to two render server jobs, the
  // swap, then more app work queued after it. Jobs retire in order; the copy reads the surface once its target retired.
  uint32_t seq = 0, compositor = 0, lastCompositor = 0, early = 0, earlyAlone = 0, stalls = 0;
  uint32_t rng = 7;
  for (int frame = 0; frame < 2000; frame++) {
    rng = rng * 1664525u + 1013904223u;
    const uint32_t appJob = ++seq;                   // the app's frame the render server composites
    const uint32_t renderJob = ++seq; compositor = renderJob;
    if (rng & 1) {++seq; compositor = seq;}          // a second render server job (a small blit) after the frame
    const uint32_t swapMark = seq;                   // guamp_adreno_note_swap at the accepted swap
    const uint32_t after = (rng >> 8) % 6;           // app work queued after the swap, before the copy runs
    seq += after;
    // A render server fence that lags its frame (its jobs not all seen as the render server's: another thread's queue, a missed
    // table update) one frame in four: 0017 alone then reads early, the swap's mark does not.
    const uint32_t seen = (rng >> 4) % 4 == 0 ? lastCompositor : compositor;
    lastCompositor = compositor;
    const uint32_t target = guampCopyTarget(seq, seen, swapMark, true);
    const uint32_t alone = guampCopyTarget(seq, seen, 0, true);
    if (!retired(alone, renderJob)) earlyAlone++;
    // The copy reads once `target` retired: every job up to it ran, in order.
    if (!retired(target, renderJob) || !retired(target, appJob)) early++;
    if (int32_t(target - swapMark) > 0) stalls++;    // waited for work queued after the swap
  }
  // guamp-copy-gpu-wait-max (guampCopyWaitGiveUp): within the caller's bound never; past it only at the hard bound or after a whole
  // bound without a retired fence; a hard bound at or under the caller's one keeps the old answer.
  CHECK(!guampCopyWaitGiveUp(100, 100, 500000, 2000000));
  CHECK(guampCopyWaitGiveUp(500000, 0, 500000, 0));            // off: as before
  CHECK(guampCopyWaitGiveUp(500000, 0, 500000, 500000));
  CHECK(!guampCopyWaitGiveUp(700000, 120000, 500000, 2000000)); // fences still retiring: wait on
  CHECK(guampCopyWaitGiveUp(700000, 500000, 500000, 2000000));  // stalled a whole bound
  CHECK(guampCopyWaitGiveUp(2000000, 0, 500000, 2000000));      // hard bound
  // fin50's case: the frame's fence 1200 ms away, a fence retiring every 150 ms. The old bound copies an unfinished frame at 500 ms;
  // the new one waits until 1200 ms. A GPU that stops at 300 ms is given up at 800 ms.
  {
    uint64_t t = 0, progress = 0; bool gave = false, done = false;
    for (; t <= 3000000; t += 1000) {
      if (t % 150000 == 0) progress = t;
      if (t >= 1200000) {done = true; break;}
      if (guampCopyWaitGiveUp(t, t - progress, 500000, 2000000)) {gave = true; break;}
    }
    CHECK(done && !gave);
    t = 0; progress = 0; gave = false;
    for (; t <= 3000000; t += 1000) {
      if (t % 150000 == 0 && t <= 300000) progress = t;
      if (guampCopyWaitGiveUp(t, t - progress, 500000, 2000000)) {gave = true; break;}
    }
    CHECK(gave && t == 800000);
  }
  CHECK(early == 0);
  CHECK(stalls == 0);
  CHECK(earlyAlone > 0);   // the model does show 0017's failure
  printf("copy target model: early reads %u (0017 alone %u), waits on work after the swap %u\n", early, earlyAlone, stalls);
  printf("copy target: %s\n", failures ? "FAIL" : "PASS");
  return failures ? 1 : 0;
}
