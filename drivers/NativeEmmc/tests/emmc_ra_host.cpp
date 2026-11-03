// Host test of the eMMC read-ahead and request histogram (namespace emmcra and readAhead() of NativeEmmc.cpp).
// run-emmc-ra-tests.sh cuts both out of the driver into emmc_ra_extract.inc; here they run against a simulated card whose
// block i holds bytes derived from i, so every byte served (from a slot or from the card) can be checked.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <vector>

static uint32_t gArgRa = 0, gArgHist = 1, gArgSlots = 0, gArgAdapt = 0;   // 0 = argument absent (slots, adapt)
static bool PE_parse_boot_argn(const char *name, void *out, int) {
  if (!strcmp(name, "guamp-emmc-ra")) { memcpy(out, &gArgRa, 4); return true; }
  if (!strcmp(name, "guamp-emmc-hist")) { memcpy(out, &gArgHist, 4); return true; }
  if (!strcmp(name, "guamp-emmc-ra-slots") && gArgSlots) { memcpy(out, &gArgSlots, 4); return true; }
  if (!strcmp(name, "guamp-emmc-ra-adapt") && gArgAdapt) { memcpy(out, &gArgAdapt, 4); return true; }
  return false;
}
static void IOLog(const char *fmt, ...) { va_list a; va_start(a, fmt); vprintf(fmt, a); va_end(a); }
static void *IOMallocZero(size_t n) { return calloc(1, n); }
static void IOFree(void *p, size_t) { free(p); }
constexpr uint32_t kBlock = 512, kBounceBlocks = 128;

static uint8_t pattern(uint64_t lba, uint32_t i) { return static_cast<uint8_t>((lba * 131 + i * 7 + (lba >> 8)) & 0xFF); }
// The simulated card: contents can change (a write) to prove invalidation.
static uint64_t gWriteEpoch = 0, gWrittenLba = ~0ULL;
static uint8_t value(uint64_t lba, uint32_t i) { return lba == gWrittenLba ? static_cast<uint8_t>(pattern(lba, i) ^ (0x5A + gWriteEpoch)) : pattern(lba, i); }
struct FakeCard {
  uint64_t commands = 0, blocksRead = 0;
  bool fail = false;
  bool readBlocks(uint64_t lba, uint32_t count, uint8_t *out) {
    if (fail) return false;
    commands++;
    blocksRead += count;
    for (uint32_t b = 0; b < count; b++)
      for (uint32_t i = 0; i < kBlock; i++) out[b * kBlock + i] = value(lba + b, i);
    return true;
  }
};
static FakeCard gCard;
struct IOMemoryDescriptor {
  std::vector<uint8_t> bytes;
  uint64_t writeBytes(uint64_t offset, const void *src, uint64_t n) {
    if (offset + n > bytes.size()) return 0;
    memcpy(bytes.data() + offset, src, n);
    return n;
  }
};

#include "emmc_ra_extract.inc"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); failures++; } } while (0)

static bool readRun(uint64_t lba, uint32_t n, uint64_t end, int *how) {
  IOMemoryDescriptor d;
  d.bytes.assign(n * kBlock, 0);
  int r = readAhead(lba, n, end, &d, 0);
  *how = r;
  if (r == 0) gCard.readBlocks(lba, n, d.bytes.data());   // the caller's usual path
  for (uint32_t b = 0; b < n; b++)
    for (uint32_t i = 0; i < kBlock; i++)
      if (d.bytes[b * kBlock + i] != value(lba + b, i)) return false;
  return true;
}

int main() {
  int how = 0;
  // 1. Off by default: nothing allocated, every run goes to the card.
  emmcra::configure();
  CHECK(emmcra::gBlocks == 0);
  CHECK(readRun(1000, 8, 100000, &how) && how == 0);
  // 2. Out of range values are refused (4 KiB, 128 KiB > DMA buffer).
  gArgRa = 4; emmcra::configure(); CHECK(emmcra::gBlocks == 0);
  gArgRa = 128; emmcra::configure(); CHECK(emmcra::gBlocks == 0);
  // 3. 64 KiB: a small read becomes one 128-block command, the next reads inside it are hits.
  gArgRa = 64; emmcra::configure();
  CHECK(emmcra::gBlocks == 128);
  uint64_t before = gCard.commands;
  CHECK(readRun(5000, 12, 100000, &how) && how == 1);
  CHECK(gCard.commands == before + 1 && emmcra::gFills == 1);
  CHECK(readRun(5012, 12, 100000, &how) && how == 1);
  CHECK(readRun(5100, 28, 100000, &how) && how == 1);
  CHECK(gCard.commands == before + 1 && emmcra::gHits == 2);
  // 4. A read crossing the slot end is a new fill, not a partial hit.
  CHECK(readRun(5120, 16, 100000, &how) && how == 1);
  CHECK(gCard.commands == before + 2);
  // 5. Window end: never read past it; a run that already reaches the end is bypassed.
  CHECK(readRun(99990, 8, 100000, &how) && how == 1);
  CHECK(emmcra::find(99990, 10) != nullptr && emmcra::find(99990, 11) == nullptr);
  CHECK(readRun(99992, 8, 100000, &how) && how == 1);   // hit inside the clamped slot
  CHECK(readRun(199992, 8, 200000, &how) && how == 0);  // 8 blocks to the end: bypass
  // 6. Reads of the read-ahead size or more are not touched.
  CHECK(readRun(7000, 128, 100000, &how) && how == 0);
  // 7. A write invalidates the overlapping slot: the next read returns the new data.
  CHECK(readRun(20000, 8, 100000, &how) && how == 1);
  gWrittenLba = 20004; gWriteEpoch++;
  emmcra::invalidate(20004, 1);
  uint64_t fills = emmcra::gFills;
  CHECK(readRun(20002, 8, 100000, &how) && how == 1);
  CHECK(emmcra::gFills == fills + 1);
  // 8. Card error during the fill: falls back to the usual path (0), no slot kept.
  gCard.fail = true;
  CHECK(readAhead(40000, 8, 100000, nullptr, 0) == 0);
  gCard.fail = false;
  CHECK(emmcra::find(40000, 8) == nullptr);
  // 9. Replacement: 40 fills in 32 slots keep the newest; the oldest is gone.
  for (unsigned k = 0; k < 40; k++) CHECK(readRun(300000 + k * 1000, 4, 1000000, &how) && how == 1);
  CHECK(emmcra::find(300000, 4) == nullptr && emmcra::find(339000, 4) != nullptr);
  // 10. Histogram: sizes and sequential streams.
  emmcra::gReqs = 0; memset(emmcra::gHist, 0, sizeof(emmcra::gHist)); emmcra::gSeq = 0;
  emmcra::note(100, 8); emmcra::note(108, 8); emmcra::note(116, 16); emmcra::note(9000, 1); emmcra::note(132, 200);
  CHECK(emmcra::gReqs == 5 && emmcra::gSeq == 3);
  CHECK(emmcra::gHist[0] == 1 && emmcra::gHist[1] == 2 && emmcra::gHist[2] == 1 && emmcra::gHist[5] == 1);
  gArgHist = 0; emmcra::configure(); emmcra::note(1, 1); CHECK(emmcra::gReqs == 5);
  emmcra::report();
  // 11. Unused blocks: a fill of 128 blocks for an 8-block read, served once more up to block 24, then evicted -> 104 unused.
  gArgHist = 1; gArgSlots = 8; emmcra::configure();
  CHECK(emmcra::kSlots == 8);
  for (unsigned i = 0; i < emmcra::kSlots; i++) emmcra::gSlot[i].count = 0;
  uint64_t unused = emmcra::gUnused;
  CHECK(readRun(500000, 8, 1000000, &how) && how == 1);
  CHECK(readRun(500016, 8, 1000000, &how) && how == 1);   // hit, reach 24
  for (unsigned k = 0; k < 8; k++) CHECK(readRun(600000 + k * 1000, 8, 1000000, &how) && how == 1);   // evicts it
  CHECK(emmcra::find(500000, 8) == nullptr);
  CHECK(emmcra::gUnused - unused >= 104);
  // 12. Slots: clamped to 8..256.
  gArgSlots = 4; emmcra::configure(); CHECK(emmcra::kSlots == 8);
  gArgSlots = 999; emmcra::configure(); CHECK(emmcra::kSlots == 256);
  // 13. guamp-emmc-ra-adapt=1: a read outside the streams fills only to the end of its 16 KiB block (32 blocks);
  // the next read that continues it gets the full 64 KiB.
  gArgSlots = 32; gArgAdapt = 1; emmcra::configure();
  for (unsigned i = 0; i < emmcra::kSlots; i++) emmcra::gSlot[i].count = 0;
  uint64_t blocks = gCard.blocksRead;
  emmcra::note(700008, 8);
  CHECK(readRun(700008, 8, 1000000, &how) && how == 1);
  CHECK(gCard.blocksRead - blocks == 24);   // 700008..700032
  CHECK(emmcra::find(700008, 24) != nullptr && emmcra::find(700008, 25) == nullptr);
  emmcra::note(700016, 8);   // continues the stream: hit inside the small slot
  CHECK(readRun(700016, 8, 1000000, &how) && how == 1 && gCard.blocksRead - blocks == 24);
  emmcra::note(700024, 16);  // crosses the slot end: new fill, full size because it is a stream
  CHECK(readRun(700024, 16, 1000000, &how) && how == 1);
  CHECK(gCard.blocksRead - blocks == 24 + 128);
  emmcra::note(800000, 32);  // a whole aligned 16 KiB read outside the streams: nothing to add, bypass
  CHECK(readRun(800000, 32, 1000000, &how) && how == 0);
  // Without the argument the same non-stream read takes the full 64 KiB.
  gArgAdapt = 0; emmcra::configure();
  for (unsigned i = 0; i < emmcra::kSlots; i++) emmcra::gSlot[i].count = 0;
  blocks = gCard.blocksRead;
  emmcra::note(900008, 8);
  CHECK(readRun(900008, 8, 1000000, &how) && how == 1 && gCard.blocksRead - blocks == 128);
  emmcra::report();
  printf("emmc read-ahead host test: %s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
  return failures ? 1 : 0;
}
