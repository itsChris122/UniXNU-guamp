// Host test of GuampMemQueuesCore.h (the "GuampMem: queues" line of GuampPerf.inc): when the line prints, and the MiB values.
#include "../GuampMemQueuesCore.h"
#include <cstdio>
#include <initializer_list>

static int gFail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); gFail++; } } while (0)

int main() {
  GuampMemQueues a = {{0, 0, 3000, 1500, 1500, 0xffffffffu, 0, 0, 8}}, b = a;
  // First report always prints, even with nothing known yet.
  CHECK(guampMemQueuesMoved(a, b, true, kGuampMemQueuesStepPages));
  CHECK(!guampMemQueuesMoved(a, b, false, kGuampMemQueuesStepPages));
  // Queue counters: below the step stays quiet, at the step prints, both directions.
  b = a; b.v[0] = kGuampMemQueuesStepPages - 1; CHECK(!guampMemQueuesMoved(b, a, false, kGuampMemQueuesStepPages));
  b = a; b.v[0] = kGuampMemQueuesStepPages; CHECK(guampMemQueuesMoved(b, a, false, kGuampMemQueuesStepPages));
  b = a; a.v[1] = 5000; b.v[1] = 5000 - kGuampMemQueuesStepPages; CHECK(guampMemQueuesMoved(b, a, false, kGuampMemQueuesStepPages));
  a.v[1] = 0;
  // The jetsam available count is a counter too (it moves all the time by a few pages).
  a.v[5] = 20000; b = a; b.v[5] = 20000 + 10; CHECK(!guampMemQueuesMoved(b, a, false, kGuampMemQueuesStepPages));
  b.v[5] = 20000 - kGuampMemQueuesStepPages; CHECK(guampMemQueuesMoved(b, a, false, kGuampMemQueuesStepPages));
  // Thresholds and the mode print on any change.
  for (unsigned i : {2u, 3u, 4u, 6u, 7u, 8u}) { b = a; b.v[i] += 1; CHECK(guampMemQueuesMoved(b, a, false, kGuampMemQueuesStepPages)); }
  // MiB: 4 KiB pages, rounded down; 16 KiB pages; no 32-bit overflow on large counts.
  CHECK(guampMemQueuesMiB(256, 4096) == 1);
  CHECK(guampMemQueuesMiB(255, 4096) == 0);
  CHECK(guampMemQueuesMiB(76800, 4096) == 300);   // fin48: ~300 MiB of anonymous pages left the counts
  CHECK(guampMemQueuesMiB(64, 16384) == 1);
  CHECK(guampMemQueuesMiB(0xfffffffeu, 4096) == 16777215);   // (2^32 - 2) / 256
  CHECK(!guampMemQueuesAvailableKnown(0xffffffffu));
  CHECK(guampMemQueuesAvailableKnown(0));
  std::printf("mem queues host: %s\n", gFail ? "FAIL" : "PASS");
  return gFail ? 1 : 0;
}
