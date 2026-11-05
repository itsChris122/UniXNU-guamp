// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp 05/11 (app launch): what the "GuampMem: MiB free ..." line cannot show. With the compressor off (vm_compressor_mode 8,
// "compressions 0") anonymous pages have nowhere to go: the pageout scan moves them to the throttled queue, which host_statistics64
// counts in no field, so "anon" fell from 309 to 8 MiB while Maps opened (fin48, 231-251 s) and ~330 MiB vanished from the sums.
// Meanwhile the file cache is what gets reclaimed: 80 % of the 4 KiB pages read from the eMMC during the launch had been read
// before. One more line, GuampPerf.inc, every 10 s report in which a value moved by kGuampMemQueuesStepPages or more (and on the
// first one): the throttled and cleaned queues, the pageout thresholds and the jetsam view (memorystatus available pages against
// its pressure / critical / idle-critical marks), with the compressor mode in use. Pure, so tests/mem_queues_host.cpp runs it.
#pragma once
#include <stdint.h>

enum : unsigned { kGuampMemQueuesCount = 9, kGuampMemQueuesStepPages = 1024 };   // 4 MiB with 4 KiB pages

// Values, in pages except the mode: [0] throttled [1] cleaned [2] free target [3] free min [4] throttle limit
// [5] memorystatus available [6] pressure mark [7] critical mark [8] compressor mode.
struct GuampMemQueues {
  uint32_t v[kGuampMemQueuesCount];
};

// True when the line is worth printing: the first report, a changed threshold or mode ([2]-[4], [6]-[8]), or a queue /
// available count ([0], [1], [5]) that moved by step pages or more since the last printed report.
static inline bool guampMemQueuesMoved(const GuampMemQueues &now, const GuampMemQueues &last, bool first, uint32_t step) {
  if (first) return true;
  for (unsigned i = 0; i < kGuampMemQueuesCount; i++) {
    const bool counter = i == 0 || i == 1 || i == 5;
    const uint32_t d = now.v[i] > last.v[i] ? now.v[i] - last.v[i] : last.v[i] - now.v[i];
    if (counter ? d >= step : d != 0) return true;
  }
  return false;
}

// Pages to MiB, rounded down, for a page size in bytes (4096 here; 16384 on Apple's own kernels).
static inline uint32_t guampMemQueuesMiB(uint32_t pages, uint32_t pageBytes) {
  return static_cast<uint32_t>((static_cast<uint64_t>(pages) * pageBytes) >> 20);
}

// memorystatus_available_pages is (unsigned)-1 until the jetsam thread first computes it.
static inline bool guampMemQueuesAvailableKnown(uint32_t available) { return available != 0xffffffffu; }
