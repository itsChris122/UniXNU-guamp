// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of patches/native-adreno/0017 (NativeAdrenoProcFence.h, from the patched sources): a model of the single in-order ring
// with a render server submitting one short job per frame and an app (Maps) submitting long jobs as fast as it can. Checks the
// process table (slots, eviction, the render server by name), the throttle target, and what the display copy waits for: the ring's
// newest fence (before) against the render server's last fence (0017), with and without the soft priority.
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include "NativeAdrenoProcFence.h"

static int failures = 0;
#define CHECK(c) do {if (!(c)) {printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++failures;}} while (0)

static NativeAdrenoProcFence table[NativeAdrenoProcFenceSlots];
static void note(int32_t pid, const char *name, uint32_t fence) {   // as noteProcessFence
  bool found = false;
  const unsigned slot = nativeAdrenoProcSlot(table, pid, &found);
  if (slot >= NativeAdrenoProcFenceSlots) return;
  NativeAdrenoProcFence &p = table[slot];
  if (!found) {p = NativeAdrenoProcFence{}; strncpy(p.name, name, sizeof p.name - 1); p.compositor = nativeAdrenoIsCompositor(p.name); p.pid = pid;}
  nativeAdrenoNoteFence(p, fence);
}
static NativeAdrenoProcFence *find(int32_t pid) {for (auto &p : table) if (p.pid == pid && pid) return &p; return nullptr;}

// Ring model: jobs run in order, each takes its time on the GPU. Time in ms.
struct Job {uint32_t fence; double cost, at;};
struct Sim {
  std::vector<Job> queue; uint32_t seq = 0, retired = 0; double busyUntil = 0, now = 0;
  uint32_t submit(double cost, bool) {queue.push_back({++seq, cost, now}); return seq;}
  void run(double until) {   // the GPU works through the queue until `until`
    while (!queue.empty()) {
      const double start = busyUntil > queue.front().at ? busyUntil : queue.front().at;
      if (start + queue.front().cost > until) break;
      busyUntil = start + queue.front().cost; retired = queue.front().fence; queue.erase(queue.begin());
    }
    now = until;
  }
};
// 10 s: an app job of `appMs` whenever it may submit (always, or with at most `depth` in flight), a render server frame every 16 ms
// whose display copy waits (at most 500 ms) for `compositorFence` or the ring's newest. Returns the copies that timed out.
static unsigned simulate(double appMs, uint32_t depth, bool compositorFence, double *meanWait) {
  memset(table, 0, sizeof table);
  Sim s; unsigned timeouts = 0, copies = 0; double waitSum = 0;
  for (double t = 0; t < 10000; t += 16) {
    s.run(t);
    // The app fills the ring up to its limit (no limit: keeps 30 jobs queued, as Turnip with deep frame pipelining).
    for (;;) {
      NativeAdrenoProcFence *app = find(200);
      const uint32_t target = app && depth ? nativeAdrenoThrottleTarget(*app, depth) : 0;
      if (depth && !nativeAdrenoFenceRetired(s.retired, target)) break;
      if (!depth && s.seq - s.retired >= 30) break;
      note(200, "Maps", s.submit(appMs, false));
    }
    note(55, "backboardd", s.submit(2, true));
    const uint32_t want = compositorFence ? find(55)->last[(find(55)->count - 1) % NativeAdrenoProcFenceHistory] : s.seq;
    // The copy waits until `want` retired: advance the GPU in 1 ms steps, at most 500 ms.
    double waited = 0;
    Sim probe = s;
    while (!nativeAdrenoFenceRetired(probe.retired, want) && waited < 500) {waited += 1; probe.run(t + waited);}
    if (!nativeAdrenoFenceRetired(probe.retired, want)) ++timeouts;
    waitSum += waited; ++copies;
  }
  *meanWait = waitSum / copies;
  return timeouts;
}

int main() {
  // Table: one slot per pid, the render server known by name, the kernel (pid 0) never recorded, eviction of the oldest.
  {
    memset(table, 0, sizeof table);
    note(0, "kernel_task", 1);
    CHECK(!find(0));
    note(55, "backboardd", 2); note(200, "Maps", 3); note(55, "backboardd", 4);
    CHECK(find(55) && find(55)->compositor && find(55)->count == 2); CHECK(find(200) && !find(200)->compositor);
    CHECK(!nativeAdrenoIsCompositor("backboardd2") && !nativeAdrenoIsCompositor("backboard"));
    for (int32_t pid = 300; pid < 300 + NativeAdrenoProcFenceSlots; ++pid) note(pid, "w", 100 + (uint32_t)pid);
    CHECK(find(55) == nullptr || find(200) == nullptr);   // the oldest last fences gave their slots
    CHECK(find(300 + NativeAdrenoProcFenceSlots - 1));
  }
  // Throttle target: the fence `depth` submissions back; none until that many; depth 0 or above the history: none.
  {
    NativeAdrenoProcFence p{}; p.pid = 1;
    CHECK(nativeAdrenoThrottleTarget(p, 2) == 0);
    nativeAdrenoNoteFence(p, 10); CHECK(nativeAdrenoThrottleTarget(p, 2) == 0);
    nativeAdrenoNoteFence(p, 11); CHECK(nativeAdrenoThrottleTarget(p, 2) == 10);
    nativeAdrenoNoteFence(p, 15); CHECK(nativeAdrenoThrottleTarget(p, 2) == 11); CHECK(nativeAdrenoThrottleTarget(p, 1) == 15);
    for (uint32_t f = 16; f < 30; ++f) nativeAdrenoNoteFence(p, f);
    CHECK(nativeAdrenoThrottleTarget(p, 4) == 26); CHECK(nativeAdrenoThrottleTarget(p, 0) == 0); CHECK(nativeAdrenoThrottleTarget(p, 5) == 0);
    CHECK(nativeAdrenoFenceRetired(5, 0) && nativeAdrenoFenceRetired(0x2, 0xfffffffeu) && !nativeAdrenoFenceRetired(0xfffffffeu, 0x2));
  }
  // The ring: a 40 ms app job (Maps) against 2 ms render server frames.
  {
    double before = 0, own = 0, prio = 0;
    const unsigned tBefore = simulate(40, 0, false, &before), tOwn = simulate(40, 0, true, &own), tPrio = simulate(40, 2, true, &prio);
    printf("copy waits: ring newest %.1f ms (%u timeouts), render server fence %.1f ms (%u), + soft priority depth 2 %.1f ms (%u)\n",
           before, tBefore, own, tOwn, prio, tPrio);
    CHECK(tBefore > 0);                         // as on 05/11: the copy waits for the app's backlog and times out
    CHECK(own <= before);                        // its own fence is never later than the newest
    CHECK(tPrio == 0); CHECK(prio < 100);        // at most two app jobs ahead of each frame
  }
  printf("proc_fence_host: %s\n", failures ? "FAIL" : "ok");
  return failures ? 1 : 0;
}
