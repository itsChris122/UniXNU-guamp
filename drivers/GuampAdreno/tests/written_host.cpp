// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of patches/native-adreno/0021 (NativeAdrenoWrittenPlan.h, from the patched sources): a model of a window's GPU-written
// imports (IOSurfaces) as NativeAdrenoVm::maintainWritten walks them before and after each job, with a CPU cache that holds stale
// lines of a surface the CPU read (or a prefetch brought in) before the GPU wrote it again. Checks the plan's rules, then three
// scenarios and what they cost in MB walked:
//   1. SpringBoard: a new snapshot surface, then the same surface reused after a pause (CA's capture pool): the CPU reads what the GPU
//      wrote every time (no stale read), whoever maps the pages (the walk is by physical address).
//   2. An app that draws frame after frame (three drawables): walked only at its start (limit jobs), not at every frame.
//   3. limit 0: every job walks every mapping (the cost the budget saves).
#include <stdint.h>
#include <stdio.h>
#include <vector>
#include "NativeAdrenoWrittenPlan.h"

static int failures = 0;
#define CHECK(c) do {if (!(c)) {printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++failures;}} while (0)

struct Surface {uint32_t walks = 0; uint64_t seen = 0, bytes = 0; uint32_t dram = 0; bool cached = false; uint32_t line = 0;};
struct Window {
  std::vector<Surface> s;
  uint32_t limit = 4; uint64_t gap = 500000;   // us
  bool before = false;   // guamp-adreno-written-clean bit 1 (default off): walk before the job too
  uint64_t walkedBytes = 0, walks = 0;
  void pass(uint64_t now, bool completed) {
    for (auto &m : s) {
      const NativeAdrenoWrittenStep st = nativeAdrenoWrittenStep(m.walks, m.seen, now, limit, gap, completed);
      if (completed) {m.walks = st.walks; m.seen = st.seen;}
      if (!st.walk) continue;
      m.cached = false;   // DC CIVAC: the CPU's line goes (clean: nothing dirty in this model)
      walkedBytes += m.bytes; ++walks;
    }
  }
  // One job of the window that writes surface `w` (-1 none) with value v.
  void job(uint64_t now, int w, uint32_t v) {
    if (before) pass(now, false);
    if (w >= 0) s[w].dram = v;   // the GPU writes DRAM
    // a prefetch while the GPU ran may bring the old line back
    pass(now + 1000, true);
  }
  uint32_t cpuRead(int w) {   // the CPU reads through its cache
    Surface &m = s[w];
    if (!m.cached) {m.cached = true; m.line = m.dram;}
    return m.line;
  }
};

int main() {
  // The rules.
  {
    NativeAdrenoWrittenStep a = nativeAdrenoWrittenStep(0, 0, 100, 4, 500, true);
    CHECK(a.walk && a.walks == 1 && a.seen == 100);
    a = nativeAdrenoWrittenStep(4, 100, 200, 4, 500, true);
    CHECK(!a.walk && a.walks == 4 && a.seen == 200);       // spent, busy window: left alone
    a = nativeAdrenoWrittenStep(4, 100, 600, 4, 500, true);
    CHECK(a.walk && a.walks == 1 && a.seen == 600);         // rested 500: armed again
    a = nativeAdrenoWrittenStep(4, 100, 600, 4, 500, false);
    CHECK(a.walk && a.walks == 0 && a.seen == 100);         // before a job: nothing recorded
    a = nativeAdrenoWrittenStep(9, 100, 200, 0, 500, true);
    CHECK(a.walk && a.walks == 9);                          // limit 0: every job, no count
    a = nativeAdrenoWrittenStep(4, 100, 1u << 30, 4, 0, true);
    CHECK(!a.walk);                                         // gap 0: armed only at map
    a = nativeAdrenoWrittenStep(0, 0, 0, 4, 500, true);
    CHECK(a.seen == 1);                                     // time 0 is never "none yet"
  }
  // 1. SpringBoard snapshots: new surface, then reused after pauses; the CPU (snapshot to file) reads after each completion.
  {
    Window sb; sb.s.resize(1); sb.s[0].bytes = 5760000;   // 720x1600 'b3a8' (4 + 1 bytes a pixel)
    uint64_t t = 1000000; uint32_t stale = 0, reads = 0;
    for (uint32_t snap = 1; snap <= 20; ++snap) {
      (void)sb.cpuRead(0);                // an earlier read keeps the old snapshot in the CPU cache
      sb.job(t, 0, snap);                 // the GPU writes the new one
      ++reads; if (sb.cpuRead(0) != snap) ++stale;
      for (int k = 0; k < 3; ++k) sb.job(t + 20000 * (k + 1), -1, 0);   // SpringBoard's other jobs right after
      t += 2000000;                       // the next app switch two seconds later
    }
    printf("snapshots: %u reads, %u stale, %.1f MB walked (%.2f MB a snapshot)\n", reads, stale, sb.walkedBytes / 1e6,
           sb.walkedBytes / 1e6 / 20);
    CHECK(stale == 0);
  }
  // 1b. The same surface reused while the window never rests (a busy writer): the plan cannot see the write (jobs name only their
  // IBs); the bridge's own walk (adreno-metal 0010) is what covers it. The model shows it, the test records it.
  {
    Window busy; busy.s.resize(1); busy.s[0].bytes = 4608000;
    uint64_t t = 1000000; uint32_t stale = 0;
    for (uint32_t f = 1; f <= 600; ++f) {
      const bool snap = f % 120 == 0;
      if (snap) (void)busy.cpuRead(0);
      busy.job(t, snap ? 0 : -1, f);
      if (snap && busy.cpuRead(0) != f) ++stale;
      t += 16667;
    }
    printf("busy window, reused surface: %u of 5 snapshots stale without the bridge's walk (expected: not covered)\n", stale);
    CHECK(stale == 5);   // mapped at the first frame: its budget was spent long before the first snapshot
  }
  // 2. An app drawing 600 frames into three drawables at 60 fps.
  {
    Window app; app.s.resize(3); for (auto &m : app.s) m.bytes = 4608000;
    uint64_t t = 1000000;
    for (uint32_t f = 0; f < 600; ++f) {app.job(t, (int)(f % 3), f); t += 16667;}
    printf("app 600 frames: %.1f MB walked (%.3f MB a frame; limit 0 would be %.1f MB)\n", app.walkedBytes / 1e6,
           app.walkedBytes / 1e6 / 600, 600.0 * 3 * 4608000 / 1e6);
    CHECK(app.walkedBytes == (uint64_t)3 * 4608000 * 4);   // the first 4 completions, each over the 3 drawables
  }
  // 3. limit 0: every job.
  {
    Window all; all.limit = 0; all.before = true; all.s.resize(3); for (auto &m : all.s) m.bytes = 4608000;
    uint64_t t = 1000000;
    for (uint32_t f = 0; f < 60; ++f) {all.job(t, (int)(f % 3), f); t += 16667;}
    CHECK(all.walkedBytes == (uint64_t)60 * 2 * 3 * 4608000);
  }
  // An app that pauses (a scroll that stops) is armed again at each start.
  {
    Window app; app.s.resize(1); app.s[0].bytes = 1 << 20;
    uint64_t t = 1000000;
    for (int burst = 0; burst < 5; ++burst) {
      for (int f = 0; f < 30; ++f) {app.job(t, 0, (uint32_t)f); t += 16667;}
      t += 1000000;
    }
    CHECK(app.walks == 5 * 4);
  }
  printf("written_host: %s\n", failures ? "FAIL" : "ok");
  return failures ? 1 : 0;
}
