// Host test of patches/native-adreno/0019 (run-tests.sh): the write-combine re-assertion's page check over the pmap's attribute table
// (NativeAdrenoWcScan.h) must find exactly the pages pmap_cache_attributes() would call non-write-combined, with the XNU encoding
// (pp_attr_t, WIMG in the low 6 bits, 0 = VM_WIMG_DEFAULT, flag bits above), and only for spans wholly inside the table; the mailbox
// poller's pause (NativeAdrenoPollPlan.h) is 1 ms only while the mailbox is served, mapped and recently used.
#include "NativeAdrenoWcScan.h"
#include "NativeAdrenoPollPlan.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
static int failures, checks;
#define CHECK(x) do {++checks; if (!(x)) {++failures; std::fprintf(stderr, "line %d: %s\n", __LINE__, #x);}} while (0)
constexpr unsigned kWimgDefault = 0x2, kWimgWcomb = 0x6, kWimgIo = 0x7, kWimgMask = 0xff;
static const uint64_t kFirst = 0x40000000ull, kPages = 4096, kLast = kFirst + kPages * 4096;
static std::vector<uint16_t> table(kPages);
// osfmk/arm/pmap/pmap.c pmap_cache_attributes() for a page inside the table.
static unsigned reference(uint64_t pa) {
  const uint16_t attr = table[(pa - kFirst) >> 12];
  return (attr & 0x3f) ? (attr & 0x3f) : kWimgDefault;
}
static uint32_t rng = 12345;
static uint32_t next() { rng = rng * 1103515245u + 12345u; return rng >> 8; }
int main() {
  for (unsigned round = 0; round < 200; ++round) {
    for (auto &a : table) {
      const uint32_t r = next() % 10;
      uint16_t wimg = r < 6 ? kWimgWcomb : r < 8 ? 0u : r < 9 ? kWimgDefault : kWimgIo;
      a = uint16_t(wimg | ((next() & 3) << 6) | ((next() & 1) << 9));   // referenced / modified / other flag bits must be ignored
    }
    const uint64_t first = next() % (kPages - 1), pages = 1 + next() % (kPages - first), pa = kFirst + first * 4096;
    std::vector<uint64_t> want, got;
    for (uint64_t i = 0; i < pages; ++i)
      if ((reference(pa + i * 4096) & kWimgMask) != kWimgWcomb) want.push_back(pa + i * 4096);
    const uint32_t found = nativeAdrenoWcScan(table.data(), kFirst, pa, pages * 4096, kWimgWcomb,
                                              [&](uint64_t p) { got.push_back(p); });
    CHECK(found == want.size());
    CHECK(got == want);
  }
  // Bounds: inside, touching the end, past the end, below the start, empty.
  CHECK(nativeAdrenoWcScanInTable(kFirst, 4096, kFirst, kLast));
  CHECK(nativeAdrenoWcScanInTable(kLast - 4096, 4096, kFirst, kLast));
  CHECK(nativeAdrenoWcScanInTable(kFirst, kPages * 4096, kFirst, kLast));
  CHECK(!nativeAdrenoWcScanInTable(kLast - 4096, 8192, kFirst, kLast));
  CHECK(!nativeAdrenoWcScanInTable(kFirst - 4096, 8192, kFirst, kLast));
  CHECK(!nativeAdrenoWcScanInTable(kLast, 4096, kFirst, kLast));
  CHECK(!nativeAdrenoWcScanInTable(kFirst, 0, kFirst, kLast));
  CHECK(!nativeAdrenoWcScanInTable(kFirst, ~0ull, kFirst, kLast));
  // Mailbox pause.
  CHECK(nativeAdrenoMailboxSleepMs(false, 1, 0, 8) == 8);    // not served
  CHECK(nativeAdrenoMailboxSleepMs(true, 0, 0, 8) == 8);     // never mapped: nobody can write a request
  CHECK(nativeAdrenoMailboxSleepMs(true, 1, 0, 8) == 1);     // just used
  CHECK(nativeAdrenoMailboxSleepMs(true, 1, 999, 8) == 1);
  CHECK(nativeAdrenoMailboxSleepMs(true, 1, 1000, 8) == 8);  // idle for 1000 polls
  CHECK(nativeAdrenoMailboxSleepMs(true, 1, 5000, 0) == 1);  // 0 = 1 ms always
  CHECK(nativeAdrenoMailboxSleepMs(false, 0, 0, 1) == 1);
  // Wakeups a second of an unused but mapped mailbox: 1000 before, about 125 now.
  unsigned idle = 0, slept = 0, wakes = 0;
  while (slept < 10000) { slept += nativeAdrenoMailboxSleepMs(true, 1, idle, 8); ++idle; ++wakes; }
  std::printf("kcost_host: mailbox wakeups in 10 s idle %u (1 ms polling: 10000)\n", wakes);
  CHECK(wakes < 2200);
  std::printf("kcost_host: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
