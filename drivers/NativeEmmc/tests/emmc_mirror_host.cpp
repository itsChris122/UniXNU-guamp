// Host test of the batched kpan log mirror (Card::mirrorRun / Card::mirrorRange of NativeEmmc.cpp, 07/11), cut out of
// the driver into mirror_extract.inc by run-emmc-mirror-tests.sh and run against a fake host and kpan partition.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
constexpr uint32_t kBlock = 512, kBounceBlocks = 128;
static uint32_t native_emmc_dma_write = 1;
static uint64_t native_emmc_errors = 0, guamp_emmc_log_bytes = 0;
namespace emmcra { static bool gPanic = false; static uint64_t inval = 0; static void invalidate(uint64_t, uint64_t c) { inval += c; } }
static std::vector<uint8_t> disk(20000 * kBlock);
struct Host {
  uint64_t dmaRuns = 0, failNext = 0;
  bool writeDma(uint64_t lba, uint32_t count, const uint8_t *in) {
    if (count == 0 || count > kBounceBlocks) { printf("FAIL bad count %u\n", count); return false; }
    if (failNext) { failNext--; return false; }
    dmaRuns++;
    memcpy(&disk[lba * kBlock], in, count * kBlock);
    return true;
  }
  void recover() {}
};
struct Card {
  Host host;
  uint8_t bounce[kBounceBlocks * kBlock];
  uint64_t logFirst = 100, logBlocks = 15000;
  uint32_t flushType = 0;
  uint64_t pio = 0;
  bool writeLogBlock(uint64_t index, const uint8_t *in) {
    if (!logBlocks || index >= logBlocks) return false;
    pio++;
    memcpy(&disk[(logFirst + index) * kBlock], in, kBlock);
    return true;
  }
#include "mirror_extract.inc"
};
static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)
int main() {
  static Card card;
  const uint32_t bytes = 300 * kBlock + 100;   // 301 blocks, the last one partial
  std::vector<uint8_t> src(bytes), shadow(bytes + kBlock, 0xA5);
  for (uint32_t i = 0; i < bytes; i++) src[i] = static_cast<uint8_t>(i * 7 + 3);
  auto verify = [&](uint64_t first) {
    for (uint32_t b = 0; b * kBlock < bytes; b++) {
      uint8_t page[kBlock] = {};
      uint32_t len = bytes - b * kBlock < kBlock ? bytes - b * kBlock : kBlock;
      memcpy(page, &src[b * kBlock], len);
      if (memcmp(page, &disk[(card.logFirst + first + b) * kBlock], kBlock) || memcmp(page, &shadow[b * kBlock], kBlock)) return false;
    }
    return true;
  };
  // 1. First pass: everything differs, 301 blocks in 3 DMA runs (128 + 128 + 45), no PIO.
  CHECK(card.mirrorRange(1, src.data(), shadow.data(), bytes));
  CHECK(verify(1) && card.pio == 0 && card.host.dmaRuns == 3 && guamp_emmc_log_bytes == 301ULL * kBlock);
  // 2. Scattered changes (blocks 5-7 and 200): two runs, contents and shadow right.
  for (uint32_t i = 5 * kBlock; i < 8 * kBlock; i++) src[i] ^= 0xFF;
  src[200 * kBlock + 9] ^= 1;
  uint64_t runs = card.host.dmaRuns;
  CHECK(card.mirrorRange(1, src.data(), shadow.data(), bytes));
  CHECK(verify(1) && card.host.dmaRuns == runs + 2 && card.pio == 0);
  // 3. Nothing changed: nothing written.
  runs = card.host.dmaRuns;
  CHECK(card.mirrorRange(1, src.data(), shadow.data(), bytes) && card.host.dmaRuns == runs);
  // 4. A failed DMA run goes out block by block.
  for (uint32_t i = 10 * kBlock; i < 14 * kBlock; i++) src[i] ^= 0x33;
  card.host.failNext = 1;
  CHECK(card.mirrorRange(1, src.data(), shadow.data(), bytes) && verify(1) && card.pio == 4 && native_emmc_errors == 1);
  // 5. Restart / power-off / panic flush (flushType set) and a panic: single-block PIO only.
  card.flushType = 3;
  src[0] ^= 1;
  src[299 * kBlock] ^= 1;
  runs = card.host.dmaRuns;
  CHECK(card.mirrorRange(1, src.data(), shadow.data(), bytes) && verify(1) && card.host.dmaRuns == runs && card.pio == 6);
  card.flushType = 0;
  emmcra::gPanic = true;
  src[1] ^= 1;
  CHECK(card.mirrorRange(1, src.data(), shadow.data(), bytes) && verify(1) && card.host.dmaRuns == runs && card.pio == 7);
  emmcra::gPanic = false;
  // 6. DMA writes not verified at boot: PIO as before.
  native_emmc_dma_write = 0;
  src[2] ^= 1;
  CHECK(card.mirrorRange(1, src.data(), shadow.data(), bytes) && verify(1) && card.host.dmaRuns == runs && card.pio == 8);
  native_emmc_dma_write = 1;
  // 7. A run past the end of kpan is refused before anything of it is written.
  card.logBlocks = 100;
  std::vector<uint8_t> before(disk);
  src[150 * kBlock] ^= 1;
  CHECK(!card.mirrorRange(1, src.data(), shadow.data(), bytes));
  CHECK(!memcmp(&before[(card.logFirst + 100) * kBlock], &disk[(card.logFirst + 100) * kBlock], 200 * kBlock));
  printf("emmc mirror host test: %s (%d failures)\n", fails ? "FAIL" : "PASS", fails);
  return fails != 0;
}
