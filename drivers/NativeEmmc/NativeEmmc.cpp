// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// eMMC storage for guamp (Moto G9 Play, SM6115): SDHC1, Qualcomm sdhci-msm-v5 at 0x04744000.
//
// Polled PIO only (no DMA, no interrupts), one 512-byte block per command. The sequence was proven on the
// phone before XNU (strumenti/guampPkg-extra/patch-darwinboot-emmctest.py): after ExitBootServices the
// firmware leaves the host fully reset (bus power off, 1-bit, legacy timing) while the card is still in
// the transfer state at HS200/HS400, so the card is brought back to HS timing with an 8-bit SDR bus at a
// slow clock.
//
// Jobs:
//  - the whole user area is published as one read-only IOBlockStorageDevice (IOGUIDPartitionScheme
//    splits it into the GPT partitions);
//  - the root windows of a SHNGROOT image inside "userdata" (scripts/unixnu/userdata.py layout; here at
//    userdata + 0 or + 512 MiB, after the FAT payload) are published as read-only devices of their own, so
//    APFS finds the OS and SystemOS containers and XNU the root volume by UUID; a window whose manifest
//    flags lack bit 0 (read-only) is published writable: the persistent iPadOS Data container, mounted on
//    /private/var by the owner Data path (native_ram_block_volume_seed below);
//  - the kernel message buffer and the console framebuffer are mirrored to the empty "kpan" partition, so
//    the boot can be read from TWRP afterwards. kpan and the writable windows are the only block ranges this
//    driver ever writes
//    (strumenti/leggi-log-xnu.sh). The platform's haltRestart() flushes them once more before a restart,
//    power off or panic (guamp_emmc_flush). Boot argument guamp-emmc-reset=N: PSCI reset after N mirrors.
//
// Started by the device ArmDTPlatform at the end of configure() (guamp_emmc_start), no personality.

#include <IOKit/IOLib.h>
#include <IOKit/storage/IOBlockStorageDevice.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOBSD.h>
#include <IOKit/IOKitKeys.h>
#include <sys/param.h>
#include <kern/clock.h>
#include <kern/thread_call.h>
#include <kern/thread.h>
#include <libkern/OSKextLib.h>
#include <libkern/crypto/sha2.h>
#include <pexpert/pexpert.h>

struct guamp_msgbuf {               // bsd/sys/msgbuf.h
  int msg_magic;
  int msg_size;
  int msg_bufx;
  int msg_bufr;
  char *msg_bufc;
};

extern "C" {
extern struct guamp_msgbuf *msgbufp;
// Boot progress kept by XNU and by the UniXNU owner-root patches (bsd_init.c, kern_exec.c, IOKitBSDInit.cpp),
// copied into the kpan header on every mirror pass: they say where the boot is even when the log text stops
extern int nprocs;
extern volatile int native_owner_root_apple_autherr, native_owner_root_decision, native_owner_root_accepted;
extern volatile int native_owner_cache_stage, native_owner_cache_status;
extern volatile int native_owner_data_stage, native_owner_data_status;
extern volatile int native_owner_boot_manifest_status;
void guamp_emmc_start(IOService *platform);
void guamp_emmc_flush(unsigned type);
// Observation: last failing init stage (0 = ready), blocks moved, mirror passes.
volatile uint32_t native_emmc_stage = 0xff;
uint64_t native_emmc_reads = 0, native_emmc_writes = 0, native_emmc_errors = 0, native_emmc_mirrors = 0;   // under the gate
uint64_t native_emmc_window_writes = 0, native_emmc_multi_failures = 0;
// Read pattern: 4 KiB pages of the card read before (a page cache that keeps dropping what it read shows up here)
uint64_t native_emmc_rereads = 0, native_emmc_last_lba = 0;
uint64_t native_emmc_recoveries = 0, native_emmc_stuck_busy = 0;   // card recoveries, of them busy past 10 s
uint32_t native_emmc_last_error = 0;                              // last error status (0x30) seen by wait
extern unsigned int vm_page_free_count;
extern task_t kernel_task;
// Who issues the reads: the process of the last request (the cluster layer reads in the faulting thread)
char native_emmc_last_proc[32];
// Time inside doAsyncReadWrite (mach absolute units) and the number of read requests: is the driver the bottleneck?
uint64_t native_emmc_busy = 0, native_emmc_requests = 0;
// ADMA2 reads (boot argument guamp-emmc-dma=1): 1 when verified and in use, and the failed DMA runs
uint32_t native_emmc_dma = 0;
uint64_t native_emmc_dma_failures = 0;
int native_emmc_last_pid = -1;
void proc_selfname(char *, int);
int proc_selfpid(void);
// Panic region (osfmk/kern/debug.c, pexpert/arm/pe_init.c). guamp has none (no RAM survives a reset and /pram stopped
// the boot), so without one XNU keeps no panic text at all; the driver gives it plain memory (startLater) and copies
// it to kpan in the panic flush. Asm names: pexpert.h declares some of them with private types.
extern vm_offset_t guamp_panic_base __asm__("_gPanicBase");
extern unsigned int guamp_panic_size __asm__("_gPanicSize");
extern void *guamp_panic_info __asm__("_panic_info");
void debug_log_init(void);
}

namespace {

constexpr uint64_t kSdhcPhys = 0x04744000, kGccSdccPhys = 0x01438000;
constexpr uint32_t kBlock = 512;
constexpr uint32_t kBounceBlocks = 128;

// Console framebuffer (the UEFI GOP surface XNU's console draws into: initialize_screen b=5C000000 w=720
// h=1600 r=2880, 32 bpp) and where its copy goes in kpan
constexpr uint64_t kFbPhys = 0x5C000000;
constexpr uint32_t kFbWidth = 720, kFbHeight = 1600, kFbRowBytes = 2880;
constexpr uint32_t kFbBytes = kFbRowBytes * kFbHeight;
constexpr uint32_t kLogFirstBlock = 1, kLogMaxBlocks = 1023, kFbFirstBlock = 1024;
constexpr uint32_t kFbEvery = 5;      // mirror passes per framebuffer copy
constexpr uint32_t kPanicFirstBlock = 12288, kPanicBytes = 64 * 1024;   // panic region copy in kpan

// SDHCI registers
enum : uint32_t {
  rArg = 0x08, rXfer = 0x0C, rCmd = 0x0E, rResp = 0x10, rData = 0x20, rPresent = 0x24, rHostCtl = 0x28,
  rPower = 0x29, rClock = 0x2C, rTimeout = 0x2E, rReset = 0x2F, rStatus = 0x30, rStatusEn = 0x34, rSignalEn = 0x38,
  rHostCtl2 = 0x3E, rAdmaAddr = 0x58, rBlockSize = 0x04, rBlockCount = 0x06,
  // msm-v5 vendor ("MCI removed" layout)
  rPwrStatus = 0x240, rPwrMask = 0x244, rPwrClear = 0x248, rPwrCtl = 0x24C,
  // CQHCI, one page above the host
  rCqCfg = 0x1000 + 0x08,
};

enum : uint32_t { sCmdDone = 1u << 0, sXferDone = 1u << 1, sWriteReady = 1u << 4, sReadReady = 1u << 5, sErrors = 0xFFFF8000u };

// Uncached device mapping, kept for the life of the kernel
volatile uint8_t *mapPhysical(uint64_t physical, uint64_t length)
{
  IOMemoryDescriptor *range = IOMemoryDescriptor::withPhysicalAddress(physical, length, kIODirectionInOut);
  if (!range) return nullptr;
  IOMemoryMap *mapping = range->map(kIOMapInhibitCache);
  range->release();
  return mapping ? reinterpret_cast<volatile uint8_t *>(mapping->getVirtualAddress()) : nullptr;
}

class Host {
public:
  volatile uint8_t *base = nullptr;
  volatile uint8_t *gcc = nullptr;

  uint8_t r8(uint32_t o) const { return *reinterpret_cast<volatile uint8_t *>(base + o); }
  uint16_t r16(uint32_t o) const { return *reinterpret_cast<volatile uint16_t *>(base + o); }
  uint32_t r32(uint32_t o) const { return *reinterpret_cast<volatile uint32_t *>(base + o); }
  void w8(uint32_t o, uint8_t v) { *reinterpret_cast<volatile uint8_t *>(base + o) = v; }
  void w16(uint32_t o, uint16_t v) { *reinterpret_cast<volatile uint16_t *>(base + o) = v; }
  void w32(uint32_t o, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(base + o) = v; }

  static uint64_t deadline(uint64_t ms) {
    uint64_t interval = 0;
    nanoseconds_to_absolutetime(ms * 1000000ULL, &interval);
    return mach_absolute_time() + interval;
  }

  // 0 = one of bits, 1 = timeout, 2 = error status
  unsigned wait(uint32_t bits, uint64_t ms) {
    uint64_t end = deadline(ms);
    do {
      uint32_t status = r32(rStatus);
      if (status & sErrors) { native_emmc_last_error = status; w32(rStatus, status); return 2; }
      if (status & bits) { w32(rStatus, status & bits); return 0; }
    } while (mach_absolute_time() < end);
    return 1;
  }

  bool resetLines() {
    w8(rReset, 0x06);
    uint64_t end = deadline(100);
    while (r8(rReset) & 0x06) {
      if (mach_absolute_time() > end) return false;
    }
    end = deadline(100);
    while (r32(rPresent) & 0x3) {
      if (mach_absolute_time() > end) return false;
    }
    w32(rStatus, 0xFFFFFFFF);
    return true;
  }

  // DAT0 high (present state bit 20): the card is not busy programming
  bool waitIdle(uint64_t ms) {
    uint64_t end = deadline(ms);
    while (!(r32(rPresent) & (1u << 20))) {
      if (mach_absolute_time() > end) return false;
    }
    return true;
  }

  // After a failed transfer. resetLines only resets the host: under heavy writes (a 500 MB core dump to Data) the card
  // was left receiving, sending or programming, and every later command failed (I/O errors on all volumes at once).
  // CMD12 takes a card in a data state back to tran (in tran it is an illegal command, answered by nothing), then
  // the card's busy has to end before the next command.
  void recover() {
    native_emmc_recoveries++;
    uint32_t failure = native_emmc_last_error;            // keep the transfer error, not the CMD12 one
    resetLines();
    command(12, 0, true, 5000);
    native_emmc_last_error = failure;
    resetLines();
    if (!waitIdle(10000)) native_emmc_stuck_busy++;
    resetLines();
  }

  // R1/R1b command without data
  bool command(uint8_t index, uint32_t argument, bool busy, uint64_t busyMs = 1000) {
    w32(rStatus, 0xFFFFFFFF);
    w32(rArg, argument);
    w16(rXfer, 0);
    w16(rCmd, static_cast<uint16_t>((index << 8) | (1 << 4) | (1 << 3) | (busy ? 0x3 : 0x2)));
    if (wait(sCmdDone, 100)) return false;
    if (busy && wait(sXferDone, busyMs)) return false;
    return true;
  }

  // CMD6 FLUSH_CACHE (EXT_CSD 32): the card's volatile cache to flash; the card ignores it with the cache off
  bool flushCache() { return command(6, (3u << 24) | (32u << 16) | (1u << 8), true, 10000); }

  // Returns 0 when ready, else the failing stage (native_emmc_stage)
  unsigned init() {
    base = mapPhysical(kSdhcPhys, 0x2000);
    gcc = mapPhysical(kGccSdccPhys, 0x1000);
    if (!base || !gcc) return 10;
    // SDCC1 AHB + APPS branch clocks
    *reinterpret_cast<volatile uint32_t *>(gcc + 0x8) |= 1;
    *reinterpret_cast<volatile uint32_t *>(gcc + 0x4) |= 1;
    IODelay(100);
    w32(rCqCfg, r32(rCqCfg) & ~1u);                       // command queue engine off
    if (!(r8(rPower) & 1)) {
      // Bus power needs the msm PWRCTL handshake; try 3.3 V, 3.0 V, 1.8 V
      static const uint8_t levels[] = { 0x0F, 0x0D, 0x0B };
      unsigned level = 0;
      w32(rPwrMask, 0xF);
      for (; level < 3; level++) {
        w8(rPower, levels[level]);
        uint64_t end = deadline(500);
        while (!r32(rPwrStatus) && mach_absolute_time() < end) { }
        if (r32(rPwrStatus)) break;
      }
      if (level == 3) return 8;
      uint32_t request = r32(rPwrStatus);
      w32(rPwrClear, request);
      w32(rPwrCtl, ((request & 3) ? 1u : 0u) | ((request & 0xC) ? 4u : 0u));
      IODelay(100000);
    }
    // Slow SDR clock: base / 16
    w16(rClock, 0);
    w16(rClock, (8 << 8) | 1);
    uint64_t end = deadline(1000);
    while (!(r16(rClock) & 2)) {
      if (mach_absolute_time() > end) return 2;
    }
    w16(rClock, r16(rClock) | 4);
    w8(rHostCtl, r8(rHostCtl) & ~0x18);                    // no DMA
    if (!resetLines()) return 1;
    w32(rStatusEn, 0xFFFFFFFF);
    w32(rSignalEn, 0);                                     // polled: no interrupt lines
    w8(rTimeout, 0x0E);
    w16(rHostCtl2, r16(rHostCtl2) & ~7);                   // SDR12
    if (!command(6, (3u << 24) | (185u << 16) | (1u << 8), true)) return 9;   // HS_TIMING = HS
    if (!command(6, (3u << 24) | (183u << 16) | (2u << 8), true)) return 9;   // BUS_WIDTH = 8 bit SDR
    w8(rHostCtl, static_cast<uint8_t>((r8(rHostCtl) & ~0x06) | 0x20));
    return 0;
  }

  uint32_t divider = 8;                    // SDCLK = base / (2 * divider); 8 is the proven clock

  // SDCLK = base / (2 * n), 10-bit divided clock mode; false if the internal clock does not settle
  bool setClock(uint32_t n) {
    w16(rClock, static_cast<uint16_t>(r16(rClock) & ~4));
    w16(rClock, static_cast<uint16_t>(((n & 0xFF) << 8) | (((n >> 8) & 3) << 6) | 1));
    uint64_t end = deadline(1000);
    while (!(r16(rClock) & 2)) {
      if (mach_absolute_time() > end) return false;
    }
    w16(rClock, static_cast<uint16_t>(r16(rClock) | 4));
    divider = n;
    return true;
  }

  // CMD18 with block count and auto CMD12, at the same proven clock: one command per run instead of per block
  bool readMulti(uint64_t lba, uint32_t count, uint8_t *out) {
    w32(rStatus, 0xFFFFFFFF);
    w16(rBlockSize, kBlock);
    w16(rBlockCount, static_cast<uint16_t>(count));
    w32(rArg, static_cast<uint32_t>(lba));
    w16(rXfer, (1 << 5) | (1 << 4) | (1 << 2) | (1 << 1));
    w16(rCmd, (18 << 8) | (1 << 5) | (1 << 4) | (1 << 3) | 0x2);
    if (wait(sCmdDone, 100)) return false;
    for (uint32_t b = 0; b < count; b++) {
      if (wait(sReadReady, 1000)) return false;
      uint32_t *words = reinterpret_cast<uint32_t *>(out + static_cast<uint64_t>(b) * kBlock);
      for (unsigned i = 0; i < kBlock / 4; i++) words[i] = r32(rData);
    }
    if (wait(sXferDone, 1000)) return false;
    native_emmc_reads += count;
    return true;
  }

  // ADMA2 with 32-bit descriptors: the controller writes the blocks into a physically contiguous buffer below 4 GiB
  // (uncached, so no cache maintenance) and the CPU copies them out; PIO reads one 32-bit data word at a time
  // (~1.3 us each on this bus), which kept the driver 92% busy at 2.4 MB/s.
  IOBufferMemoryDescriptor *dmaBuffer = nullptr;
  volatile uint8_t *dmaData = nullptr;
  volatile uint32_t *dmaTable = nullptr;
  uint32_t dmaDataPhys = 0, dmaTablePhys = 0;
  static constexpr uint32_t kDmaBytes = kBounceBlocks * kBlock, kDmaChunk = 32768;

  bool setupDma() {
    dmaBuffer = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
        static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous) |
        static_cast<IOOptionBits>(kIOMapInhibitCache), kDmaBytes + 4096, 0xFFFFF000ULL);
    if (!dmaBuffer) return false;
    uint8_t *virt = static_cast<uint8_t *>(dmaBuffer->getBytesNoCopy());
    uint64_t phys = dmaBuffer->getPhysicalSegment(0, nullptr, kIOMemoryMapperNone);
    if (!virt || !phys || phys + kDmaBytes + 4096 > 0x100000000ULL) return false;
    dmaData = virt;
    dmaTable = reinterpret_cast<volatile uint32_t *>(virt + kDmaBytes);
    dmaDataPhys = static_cast<uint32_t>(phys);
    dmaTablePhys = static_cast<uint32_t>(phys + kDmaBytes);
    return true;
  }

  // CMD18 (auto CMD12) of count blocks into the DMA buffer, then copied to out
  bool readDma(uint64_t lba, uint32_t count, uint8_t *out) {
    uint32_t bytes = count * kBlock;
    if (!dmaData || !count || bytes > kDmaBytes) return false;
    unsigned descriptors = 0;
    for (uint32_t at = 0; at < bytes; at += kDmaChunk, descriptors++) {
      uint32_t length = bytes - at < kDmaChunk ? bytes - at : kDmaChunk;
      bool last = at + length >= bytes;
      dmaTable[2 * descriptors] = (length << 16) | 0x20 | (last ? 0x2 : 0) | 0x1;   // tran, end, valid
      dmaTable[2 * descriptors + 1] = dmaDataPhys + at;
    }
    __asm__ volatile("dsb sy" ::: "memory");
    w32(rStatus, 0xFFFFFFFF);
    w32(rAdmaAddr, dmaTablePhys);
    w32(rAdmaAddr + 4, 0);
    w8(rHostCtl, static_cast<uint8_t>((r8(rHostCtl) & ~0x18) | 0x10));     // ADMA2, 32-bit
    w16(rBlockSize, kBlock);
    w16(rBlockCount, static_cast<uint16_t>(count));
    w32(rArg, static_cast<uint32_t>(lba));
    w16(rXfer, (1 << 5) | (1 << 4) | (1 << 2) | (1 << 1) | (1 << 0));
    w16(rCmd, (18 << 8) | (1 << 5) | (1 << 4) | (1 << 3) | 0x2);
    bool ok = !wait(sCmdDone, 100) && !wait(sXferDone, 2000);
    w8(rHostCtl, static_cast<uint8_t>(r8(rHostCtl) & ~0x18));                // PIO paths: no DMA select
    if (!ok) return false;
    __asm__ volatile("dsb sy" ::: "memory");
    for (uint32_t i = 0; i < bytes; i += 8) {
      uint64_t word = *reinterpret_cast<volatile uint64_t *>(dmaData + i);
      memcpy(out + i, &word, 8);
    }
    native_emmc_reads += count;
    return true;
  }

  bool readBlock(uint64_t lba, uint8_t *out) {
    for (unsigned attempt = 0; attempt < 3; attempt++) {
      w32(rStatus, 0xFFFFFFFF);
      w16(rBlockSize, kBlock);
      w16(rBlockCount, 1);
      w32(rArg, static_cast<uint32_t>(lba));
      w16(rXfer, (1 << 4) | (1 << 1));
      w16(rCmd, (17 << 8) | (1 << 5) | (1 << 4) | (1 << 3) | 0x2);
      if (!wait(sCmdDone, 100) && !wait(sReadReady, 1000)) {
        uint32_t *words = reinterpret_cast<uint32_t *>(out);
        for (unsigned i = 0; i < kBlock / 4; i++) words[i] = r32(rData);
        if (!wait(sXferDone, 100)) { native_emmc_reads++; return true; }
      }
      native_emmc_errors++;
      recover();
    }
    return false;
  }

  bool writeBlock(uint64_t lba, const uint8_t *in) {
    for (unsigned attempt = 0; attempt < 3; attempt++) {
      if (!waitIdle(5000)) native_emmc_stuck_busy++;
      w32(rStatus, 0xFFFFFFFF);
      w16(rBlockSize, kBlock);
      w16(rBlockCount, 1);
      w32(rArg, static_cast<uint32_t>(lba));
      w16(rXfer, (1 << 1));
      w16(rCmd, (24 << 8) | (1 << 5) | (1 << 4) | (1 << 3) | 0x2);
      if (!wait(sCmdDone, 100) && !wait(sWriteReady, 1000)) {
        const uint32_t *words = reinterpret_cast<const uint32_t *>(in);
        for (unsigned i = 0; i < kBlock / 4; i++) w32(rData, words[i]);
        if (!wait(sXferDone, 5000)) { native_emmc_writes++; return true; }
      }
      native_emmc_errors++;
      recover();
    }
    return false;
  }
};

// Everything shared by the published devices: one host, one lock, one bounce buffer
struct Card {
  Host host;
  IOLock *gate = nullptr;
  uint8_t *bounce = nullptr;               // kBounceBlocks blocks
  uint64_t capacity = 0;                   // blocks of the user area
  uint64_t logFirst = 0, logBlocks = 0;    // "kpan"
  uint64_t dataFirst = 0, dataBlocks = 0;  // "userdata"
  uint8_t *shadow = nullptr;               // last mirrored message buffer
  uint32_t shadowSize = 0;
  uint8_t *fbShadow = nullptr;             // last mirrored framebuffer
  volatile uint8_t *fb = nullptr;
  uint32_t sequence = 0, fbSequence = 0, resetAfter = 0, flushType = 0;
  thread_call_t mirrorCall = nullptr;
  bool online = false;

  uint8_t *seen = nullptr;                 // one bit per 4 KiB page of the user area
  void notePages(uint64_t lba, uint32_t count) {
    native_emmc_last_lba = lba;
    if (!seen) return;
    for (uint64_t page = lba / 8; page <= (lba + count - 1) / 8 && page / 8 < (capacity + 63) / 64; page++) {
      uint8_t bit = static_cast<uint8_t>(1u << (page & 7));
      if (seen[page / 8] & bit) native_emmc_rereads++;
      seen[page / 8] |= bit;
    }
  }
  // One CMD18 per run; after a failed multi-block command (counted, lines reset) the run is read block by block
  bool readBlocks(uint64_t lba, uint32_t count, uint8_t *out) {
    notePages(lba, count);
    if (native_emmc_dma) {
      if (host.readDma(lba, count, out)) return true;
      native_emmc_errors++;
      host.recover();
      if (++native_emmc_dma_failures >= 8) native_emmc_dma = 0;    // back to PIO for good
    }
    if (count > 1 && multiOk) {
      if (host.readMulti(lba, count, out)) return true;
      native_emmc_errors++;
      native_emmc_multi_failures++;
      host.recover();
      if (native_emmc_multi_failures >= 8) multiOk = false;     // keep the proven path if CMD18 keeps failing
    }
    for (uint32_t i = 0; i < count; i++) {
      if (!host.readBlock(lba + i, out + static_cast<uint64_t>(i) * kBlock)) return false;
    }
    return true;
  }
  bool multiOk = true;
  // Writes of a writable window (its device checks the range)
  bool writeBlocks(uint64_t lba, uint32_t count, const uint8_t *in) {
    for (uint32_t i = 0; i < count; i++) {
      if (!host.writeBlock(lba + i, in + static_cast<uint64_t>(i) * kBlock)) return false;
      native_emmc_window_writes++;
    }
    return true;
  }
  // Log writes, confined to the log partition
  bool writeLogBlock(uint64_t index, const uint8_t *in) {
    if (!logBlocks || index >= logBlocks) return false;
    return host.writeBlock(logFirst + index, in);
  }
  // Writes the blocks of source that differ from shadow; false on a write error
  bool mirrorRange(uint64_t firstBlock, const volatile uint8_t *source, uint8_t *copy, uint32_t bytes) {
    uint32_t blocks = (bytes + kBlock - 1) / kBlock;
    for (uint32_t b = 0; b < blocks; b++) {
      uint8_t *page = bounce;
      uint32_t offset = b * kBlock, length = bytes - offset < kBlock ? bytes - offset : kBlock;
      memset(page, 0, kBlock);
      for (uint32_t i = 0; i < length; i++) page[i] = source[offset + i];
      if (!memcmp(page, copy + offset, kBlock)) continue;
      if (!writeLogBlock(firstBlock + b, page)) return false;
      memcpy(copy + offset, page, kBlock);
    }
    return true;
  }
  bool findPartitions();
  void mirror(bool framebuffer);
  uint32_t panicCopied = 0;
  // The panic region (header + text) to kpan, from every panic flush: XNU calls haltRestart at panic begin, before
  // the text exists, and again at panic sync and restart
  void savePanic() {
    if (!guamp_panic_base) return;
    uint32_t bytes = guamp_panic_size < kPanicBytes ? guamp_panic_size : kPanicBytes;
    const uint8_t *region = reinterpret_cast<const uint8_t *>(guamp_panic_base);
    for (uint32_t b = 0; b * kBlock < bytes; b++) {
      memset(bounce, 0, kBlock);
      memcpy(bounce, region + b * kBlock, bytes - b * kBlock < kBlock ? bytes - b * kBlock : kBlock);
      if (!writeLogBlock(kPanicFirstBlock + b, bounce)) return;
    }
    panicCopied = bytes;
  }
};

Card gCard;
IOService *gPlatform = nullptr;

bool Card::findPartitions()
{
  uint8_t *block = bounce;
  if (!readBlocks(1, 1, block) || memcmp(block, "EFI PART", 8)) return false;
  uint64_t alternate, entriesLba;
  uint32_t entries, entrySize;
  memcpy(&alternate, block + 32, 8);
  memcpy(&entriesLba, block + 72, 8);
  memcpy(&entries, block + 80, 4);
  memcpy(&entrySize, block + 84, 4);
  if (!alternate || entrySize < 128 || entrySize > kBlock || (kBlock % entrySize) || entries > 1024) return false;
  capacity = alternate + 1;
  const uint32_t perBlock = kBlock / entrySize;
  for (uint32_t i = 0; i < entries; i++) {
    if (i % perBlock == 0 && !readBlocks(entriesLba + i / perBlock, 1, block)) return false;
    const uint8_t *entry = block + (i % perBlock) * entrySize;
    char name[16] = {};
    for (unsigned c = 0; c + 1 < sizeof(name); c++) {
      uint16_t unit = static_cast<uint16_t>(entry[56 + 2 * c] | (entry[57 + 2 * c] << 8));
      if (!unit || unit > 0x7f) break;
      name[c] = static_cast<char>(unit);
    }
    uint64_t first, last;
    memcpy(&first, entry + 32, 8);
    memcpy(&last, entry + 40, 8);
    if (first < 34 || last < first || last >= capacity) continue;
    if (!strcmp(name, "kpan") && last - first + 1 <= (64ULL << 20) / kBlock) { logFirst = first; logBlocks = last - first + 1; }
    if (!strcmp(name, "userdata")) { dataFirst = first; dataBlocks = last - first + 1; }
  }
  return logBlocks >= kPanicFirstBlock + kPanicBytes / kBlock;
}

// Log layout in kpan: block 0 = header, blocks 1.. = the raw message ring (msg_size bytes, at most 1023
// blocks), blocks 1024.. = the framebuffer (kFbRowBytes * kFbHeight bytes, XRGB little endian).
//   0 "GUAMPLOG"  8 version 2  12 msg_size  16 msg_bufx  20 sequence  24 init stage  28 msg_magic
//   32 mach_absolute_time  40 reads  48 writes  56 errors
//   64 fb first block  68 width  72 height  76 row bytes  80 fb sequence (0 = no copy yet)  84 unused  88 SD clock divider
//   92 int32: root Apple autherr, root owner decision, root accepted, SystemOS stage, SystemOS status, Data stage,
//      Data status, boot manifest status, nprocs, flush type (0 = periodic, else haltRestart type + 1), text ok
//   136 panic region copy: first block, bytes (0 = none)  144 uint64 Data window block writes
//   152 uint64 failed CMD18 runs (8 failures switch back to single-block reads)
//   160 uint32 ADMA2 reads in use  168 uint64 failed ADMA2 runs (8 failures switch back to PIO)
void Card::mirror(bool framebuffer)
{
  guamp_msgbuf *ring = msgbufp;
  if (!online || !ring || !ring->msg_bufc || ring->msg_size <= 0) return;
  uint32_t size = static_cast<uint32_t>(ring->msg_size);
  if (size > kLogMaxBlocks * kBlock) size = kLogMaxBlocks * kBlock;
  if (shadowSize != size) {
    if (shadow) IOFree(shadow, static_cast<vm_size_t>(shadowSize) + kBlock);
    shadowSize = size;
    shadow = static_cast<uint8_t *>(IOMallocZero(static_cast<vm_size_t>(size) + kBlock));
    if (shadow) memset(shadow, 0xA5, static_cast<size_t>(size) + kBlock);   // force a first full write
  }
  bool ok = shadow && mirrorRange(kLogFirstBlock, reinterpret_cast<volatile uint8_t *>(ring->msg_bufc), shadow, size);
  if (ok && framebuffer && fb && fbShadow) {
    ok = mirrorRange(kFbFirstBlock, fb, fbShadow, kFbBytes);
    if (ok) fbSequence = sequence + 1;
  }
  uint8_t *header = bounce;
  memset(header, 0, kBlock);
  memcpy(header, "GUAMPLOG", 8);
  uint32_t words[6] = { 2, size, static_cast<uint32_t>(ring->msg_bufx), ++sequence, native_emmc_stage,
                        static_cast<uint32_t>(ring->msg_magic) };
  memcpy(header + 8, words, sizeof(words));
  uint64_t counters[4] = { mach_absolute_time(), native_emmc_reads, native_emmc_writes, native_emmc_errors };
  memcpy(header + 32, counters, sizeof(counters));
  uint32_t fbWords[7] = { kFbFirstBlock, kFbWidth, kFbHeight, kFbRowBytes, fbSequence, 0, host.divider };
  memcpy(header + 64, fbWords, sizeof(fbWords));
  int32_t progress[11] = { native_owner_root_apple_autherr, native_owner_root_decision, native_owner_root_accepted,
                           native_owner_cache_stage, native_owner_cache_status, native_owner_data_stage,
                           native_owner_data_status, native_owner_boot_manifest_status, nprocs,
                           static_cast<int32_t>(flushType), ok ? 1 : 0 };
  memcpy(header + 92, progress, sizeof(progress));
  uint32_t panicWords[2] = { kPanicFirstBlock, panicCopied };
  memcpy(header + 136, panicWords, sizeof(panicWords));
  memcpy(header + 144, &native_emmc_window_writes, sizeof(native_emmc_window_writes));
  memcpy(header + 152, &native_emmc_multi_failures, sizeof(native_emmc_multi_failures));
  memcpy(header + 160, &native_emmc_dma, sizeof(native_emmc_dma));
  memcpy(header + 168, &native_emmc_dma_failures, sizeof(native_emmc_dma_failures));
  // The header goes out even when a block of the text failed: it carries the progress words
  writeLogBlock(0, header);
  native_emmc_mirrors++;
}

void psci(uint64_t function)
{
  register uint64_t x0 __asm__("x0") = function;
  __asm__ volatile("smc #0" : "+r"(x0) : : "memory");
}

void mirrorEntry(thread_call_param_t, thread_call_param_t)
{
  Card &card = gCard;
  IOLockLock(card.gate);
  card.mirror(card.sequence % kFbEvery == 0);
  uint64_t reads = native_emmc_reads, rereads = native_emmc_rereads, last = native_emmc_last_lba;
  IOLockUnlock(card.gate);
  static uint64_t lastReads = 0, lastBusy = 0, lastRequests = 0, lastTime = 0;
  if (card.sequence % 10 == 0) {
    uint64_t now = mach_absolute_time(), busy = native_emmc_busy, requests = native_emmc_requests;
    uint64_t busyNs = 0, spanNs = 0;
    absolutetime_to_nanoseconds(busy - lastBusy, &busyNs);
    absolutetime_to_nanoseconds(now - lastTime, &spanNs);
    IOLog("NativeEmmc: %llu read requests in the last period, %llu blocks each on average, driver busy %llu%%\n",
          requests - lastRequests, requests > lastRequests ? (reads - lastReads) / (requests - lastRequests) : 0,
          spanNs ? busyNs * 100 / spanNs : 0);
    lastBusy = busy; lastRequests = requests; lastTime = now;
    IOLog("NativeEmmc: %llu reads (+%llu in 10 s), %llu re-read 4K pages, last LBA %llu, %u free pages, last reader %d %s\n",
          reads, reads - lastReads, rereads, last, vm_page_free_count, native_emmc_last_pid, native_emmc_last_proc);
    lastReads = reads;
    if (native_emmc_errors)
      IOLog("NativeEmmc: %llu errors, %llu recoveries (%llu busy past the limit), last error status 0x%08x, %llu writes\n",
            native_emmc_errors, native_emmc_recoveries, native_emmc_stuck_busy, native_emmc_last_error, native_emmc_writes);
  }
  if (card.resetAfter && card.sequence >= card.resetAfter) psci(0x84000009);   // SYSTEM_RESET
  uint64_t when = 0;
  clock_interval_to_deadline(1, kSecondScale, &when);
  thread_call_enter_delayed(card.mirrorCall, when);
}

} // namespace

// IOBlockStorageDevice lives in the IOStorageFamily kext, not in the kernel this driver is linked into. Its
// constructor and destructor only chain to IOService, so they are provided here as tail calls; the other base
// methods it would inherit are reimplemented below from the public IOStorageFamily source. The class
// hierarchy is joined to the kext's real metaclass at run time (linkMetaClass).
__asm__(
  ".text\n"
  ".p2align 2\n"
  ".globl __ZN20IOBlockStorageDeviceC2EPK11OSMetaClass\n"
  "__ZN20IOBlockStorageDeviceC2EPK11OSMetaClass:\n"
  "  b __ZN9IOServiceC2EPK11OSMetaClass\n"
  ".globl __ZN20IOBlockStorageDeviceD2Ev\n"
  "__ZN20IOBlockStorageDeviceD2Ev:\n"
  "  b __ZN9IOServiceD2Ev\n"
  // Placeholder metaclass until linkMetaClass() points our superclass link at the kext's real one
  ".section __DATA,__data\n"
  ".p2align 3\n"
  ".globl __ZN20IOBlockStorageDevice10gMetaClassE\n"
  "__ZN20IOBlockStorageDevice10gMetaClassE:\n"
  "  .space 128\n"
  ".globl __ZN20IOBlockStorageDevice9metaClassE\n"
  "__ZN20IOBlockStorageDevice9metaClassE:\n"
  "  .quad __ZN20IOBlockStorageDevice10gMetaClassE\n"
  ".text\n");

// One block range of the card: the whole user area or a root window (read-only), or the Data window (writable)
class NativeEmmc : public IOBlockStorageDevice {
  OSDeclareDefaultStructors(NativeEmmc)
  uint64_t first = 0, blocks = 0;
  bool writable = false;
  char product[40] = {};

public:
  bool setRange(uint64_t start, uint64_t count, const char *name, bool write) {
    first = start;
    blocks = count;
    writable = write;
    snprintf(product, sizeof(product), "eMMC %s", name);
    return count != 0;
  }
  bool isWritable() const { return writable; }
  uint64_t firstBlock() const { return first; }
  uint64_t blockCount() const { return blocks; }
  // Base methods of IOBlockStorageDevice (IOStorageFamily IOBlockStorageDevice.cpp, iOS build)
  using IOBlockStorageDevice::init;
  bool init(OSDictionary *properties) override {
    return IOService::init(properties) && setProperty(kIOBlockStorageDeviceTypeKey, kIOBlockStorageDeviceTypeGeneric);
  }
  using IOBlockStorageDevice::getProperty;
  OSObject *getProperty(const OSSymbol *key) const override {
    if (key->isEqualTo(kIOBlockStorageDeviceWriteCacheStateKey)) return kOSBooleanFalse;
    return IOService::getProperty(key);
  }
  IOReturn setProperties(OSObject *properties) override { return IOService::setProperties(properties); }
  IOReturn doUnmap(IOBlockStorageDeviceExtent *, UInt32, IOStorageUnmapOptions) override { return kIOReturnUnsupported; }
  IOReturn doSetPriority(IOBlockStorageDeviceExtent *, UInt32, IOStoragePriority) override { return kIOReturnUnsupported; }
  IOReturn doGetProvisionStatus(UInt64, UInt64, UInt32 *, IOBlockStorageProvisionDeviceExtent *,
                                IOStorageGetProvisionStatusOptions) override { return kIOReturnUnsupported; }
  IOReturn requestIdle() override { return kIOReturnUnsupported; }
  IOReturn doEjectMedia() override { return kIOReturnUnsupported; }
  IOReturn doFormatMedia(UInt64) override { return kIOReturnUnsupported; }
  UInt32 doGetFormatCapacities(UInt64 *, UInt32) const override { return 0; }
  char *getVendorString() override { return const_cast<char *>("Qualcomm"); }
  char *getProductString() override { return product; }
  char *getRevisionString() override { return const_cast<char *>("1.0"); }
  char *getAdditionalDeviceInfoString() override {
    return const_cast<char *>(writable ? "SDHC1, polled PIO" : "SDHC1, polled PIO, read-only");
  }
  IOReturn reportBlockSize(UInt64 *value) override { *value = kBlock; return kIOReturnSuccess; }
  IOReturn reportEjectability(bool *value) override { *value = false; return kIOReturnSuccess; }
  IOReturn reportMaxValidBlock(UInt64 *value) override { *value = blocks - 1; return kIOReturnSuccess; }
  IOReturn reportMediaState(bool *present, bool *changed) override {
    *present = gCard.online; if (changed) *changed = false; return kIOReturnSuccess;
  }
  IOReturn reportRemovability(bool *value) override { *value = false; return kIOReturnSuccess; }
  IOReturn reportWriteProtection(bool *value) override { *value = !writable; return kIOReturnSuccess; }
  IOReturn doSynchronizeCache() override;
  IOReturn doSynchronize(UInt64, UInt64, IOOptionBits) override { return doSynchronizeCache(); }
  IOReturn getWriteCacheState(bool *value) override { *value = false; return kIOReturnSuccess; }
  IOReturn setWriteCacheState(bool) override { return kIOReturnUnsupported; }
  IOReturn doAsyncReadWrite(IOMemoryDescriptor *, UInt64, UInt64, IOStorageAttributes *, IOStorageCompletion *) override;
};
OSDefineMetaClassAndStructors(NativeEmmc, IOBlockStorageDevice)

IOReturn NativeEmmc::doAsyncReadWrite(IOMemoryDescriptor *buffer, UInt64 block, UInt64 count,
    IOStorageAttributes *, IOStorageCompletion *completion)
{
  if (!buffer || !completion || !completion->action || !count || block >= blocks || count > blocks - block ||
      buffer->getLength() < count * kBlock) return kIOReturnBadArgument;
  bool write = buffer->getDirection() == kIODirectionOut;
  if (!write && buffer->getDirection() != kIODirectionIn) return kIOReturnBadArgument;
  if (write && !writable) return kIOReturnNotWritable;
  Card &card = gCard;
  if (!card.online) return kIOReturnNotReady;
  uint64_t started = mach_absolute_time();
  if (!write) {
    native_emmc_requests++;
    native_emmc_last_pid = proc_selfpid();
    proc_selfname(native_emmc_last_proc, sizeof(native_emmc_last_proc));
  }
  uint64_t done = 0;
  IOReturn status = kIOReturnSuccess;
  // The gate is taken per bounce-buffer run, so a long request does not starve the log mirror
  while (done < count) {
    uint32_t n = count - done > kBounceBlocks ? kBounceBlocks : static_cast<uint32_t>(count - done);
    uint64_t bytes = static_cast<uint64_t>(n) * kBlock;
    IOLockLock(card.gate);
    bool ok = write ? buffer->readBytes(done * kBlock, card.bounce, bytes) == bytes &&
                          card.writeBlocks(first + block + done, n, card.bounce)
                    : card.readBlocks(first + block + done, n, card.bounce) &&
                          buffer->writeBytes(done * kBlock, card.bounce, bytes) == bytes;
    IOLockUnlock(card.gate);
    if (!ok) { status = kIOReturnIOError; break; }
    done += n;
  }
  __atomic_fetch_add(&native_emmc_busy, mach_absolute_time() - started, __ATOMIC_RELAXED);
  completion->action(completion->target, completion->parameter, status, done * kBlock);
  return kIOReturnSuccess;
}

IOReturn NativeEmmc::doSynchronizeCache()
{
  Card &card = gCard;
  if (!card.online) return kIOReturnNotReady;
  if (!writable) return kIOReturnSuccess;
  IOLockLock(card.gate);
  bool ok = card.host.flushCache();
  IOLockUnlock(card.gate);
  return ok ? kIOReturnSuccess : kIOReturnIOError;
}

// Our metaclass must name the kext's real IOBlockStorageDevice metaclass as its superclass (IOKit matching and
// OSDynamicCast walk that chain). Load the kext, then check and set the link (OSMetaClass: vptr, reserved,
// superClassLink, ...) before the first instance exists.
static bool linkMetaClass()
{
  if (OSKextLoadKextWithIdentifier("com.apple.iokit.IOStorageFamily") != kOSReturnSuccess) return false;
  const OSSymbol *name = OSSymbol::withCStringNoCopy("IOBlockStorageDevice");
  if (!name) return false;
  const OSMetaClass *real = OSMetaClass::copyMetaClassWithName(name);
  name->release();
  if (!real) return false;
  const OSMetaClass **link = reinterpret_cast<const OSMetaClass **>(
      reinterpret_cast<uintptr_t>(NativeEmmc::metaClass) + 2 * sizeof(void *));
  bool ok = *link == real || *link == reinterpret_cast<const OSMetaClass *>(IOBlockStorageDevice::metaClass);
  if (ok) *link = real;
  real->releaseMetaClass();
  return ok && NativeEmmc::metaClass->getSuperClass() == real;
}

static void publish(uint64_t first, uint64_t blocks, const char *name, bool writable = false)
{
  NativeEmmc *disk = OSTypeAlloc(NativeEmmc);
  if (!disk) return;
  if (disk->init(nullptr) && disk->setRange(first, blocks, name, writable) && disk->attach(gPlatform)) {
    if (disk->start(gPlatform)) disk->registerService();
    else disk->detach(gPlatform);
  }
  disk->release();
}

// SHNGROOT manifest (scripts/unixnu/userdata.py) at the start of userdata or after the 512 MiB payload
static void publishWindows()
{
  Card &card = gCard;
  static const uint64_t offsets[] = { 0, (512ULL << 20) / kBlock };
  uint8_t *manifest = static_cast<uint8_t *>(IOMallocZero(4096));
  if (!manifest || !card.dataBlocks) { if (manifest) IOFree(manifest, 4096); return; }
  for (uint64_t offset : offsets) {
    uint64_t base = card.dataFirst + offset;
    IOLockLock(card.gate);
    bool read = card.readBlocks(base, 8, manifest);
    IOLockUnlock(card.gate);
    if (!read || memcmp(manifest, "SHNGROOT", 8)) continue;
    uint8_t digest[32];
    SHA256_CTX context;
    SHA256_Init(&context);
    SHA256_Update(&context, manifest, 4064);
    SHA256_Final(digest, &context);
    uint32_t version, count;
    uint64_t imageBytes;
    memcpy(&version, manifest + 8, 4);
    memcpy(&count, manifest + 12, 4);
    memcpy(&imageBytes, manifest + 16, 8);
    if (memcmp(digest, manifest + 4064, 32) || version != 1 || count > 31 ||
        imageBytes / kBlock > card.dataBlocks - offset) {
      IOLog("NativeEmmc: SHNGROOT at userdata+%llu refused\n", offset);
      continue;
    }
    for (uint32_t i = 0; i < count; i++) {
      const uint8_t *entry = manifest + 24 + 128 * i;
      char name[33] = {};
      memcpy(name, entry, 32);
      uint64_t start, size;
      uint32_t flags;
      memcpy(&start, entry + 32, 8);
      memcpy(&size, entry + 40, 8);
      memcpy(&flags, entry + 52, 4);
      if ((start % kBlock) || (size % kBlock) || start + size > imageBytes) continue;
      bool writable = !(flags & 1);
      IOLog("NativeEmmc: window %s at userdata+%llu, %llu blocks%s\n", name, offset + start / kBlock, size / kBlock,
            writable ? ", writable" : "");
      publish(base + start / kBlock, size / kBlock, name, writable);
    }
    break;
  }
  IOFree(manifest, 4096);
}


// USB probe (first step of a live screen over USB): does the firmware leave the USB 3.0 controller (DWC3 at 0x04E00000,
// QUSB2 PHY at 0x01613000) powered and clocked after ExitBootServices? GCC registers can always be read; the DWC3 and
// PHY registers only when their GDSC and clocks are on (an unclocked access can hang the bus). Offsets from Linux
// gcc-sm6115.c: GDSC bit 31 = power on; branch CBCR bit 31 = clock off, bit 0 = enable.
static void usbProbe()
{
  volatile uint8_t *gcc = mapPhysical(0x0141A000, 0x4000);     // 0x1a000-0x1dfff of GCC at 0x01400000
  if (!gcc) { IOLog("NativeEmmc USB: GCC not mapped\n"); return; }
  auto g = [gcc](uint32_t offset) { return *reinterpret_cast<volatile uint32_t *>(gcc + offset - 0x1a000); };
  uint32_t gdsc = g(0x1a004), master = g(0x1a010), sleep = g(0x1a014), utmi = g(0x1a018), sysnoc = g(0x1a080),
           cfgnoc = g(0x1a084), ahb2phy = g(0x1d008);
  IOLog("NativeEmmc USB: GDSC %08x master %08x sleep %08x utmi %08x sysnoc %08x cfgnoc %08x ahb2phy %08x\n",
        gdsc, master, sleep, utmi, sysnoc, cfgnoc, ahb2phy);
  bool on = (gdsc & (1u << 31)) && !(master & (1u << 31)) && !(cfgnoc & (1u << 31)) && !(sysnoc & (1u << 31));
  if (on) {
    volatile uint8_t *dwc = mapPhysical(0x04E0C000, 0x1000);
    if (dwc) {
      auto d = [dwc](uint32_t offset) { return *reinterpret_cast<volatile uint32_t *>(dwc + offset - 0xC000); };
      IOLog("NativeEmmc USB: DWC3 GSNPSID %08x GCTL %08x GUSB2PHYCFG %08x DCFG %08x DCTL %08x DSTS %08x GEVNTSIZ %08x\n",
            d(0xC120), d(0xC110), d(0xC200), d(0xC700), d(0xC704), d(0xC70C), d(0xC408));
    }
  } else {
    IOLog("NativeEmmc USB: controller not powered or clocked, registers not read\n");
  }
  if (!(ahb2phy & (1u << 31))) {
    volatile uint8_t *phy = mapPhysical(0x01613000, 0x1000);
    if (phy) IOLog("NativeEmmc USB: QUSB2 PLL_STATUS %02x POWERDOWN %02x TUNE1 %02x\n",
                   phy[0x38], phy[0xb4], phy[0x80]);
  }
}

#include "UsbMirror.inc"

static void startLater(thread_call_param_t, thread_call_param_t)
{
  Card &card = gCard;
  if (!linkMetaClass()) { native_emmc_stage = 12; IOLog("NativeEmmc: IOStorageFamily metaclass not linked\n"); return; }
  card.gate = IOLockAlloc();
  card.bounce = static_cast<uint8_t *>(IOMallocZero(kBounceBlocks * kBlock));
  if (!card.gate || !card.bounce) return;
  native_emmc_stage = card.host.init();
  if (native_emmc_stage) { IOLog("NativeEmmc: init failed at stage %u\n", native_emmc_stage); return; }
  if (!card.findPartitions()) { native_emmc_stage = 11; IOLog("NativeEmmc: GPT, kpan or userdata not found\n"); return; }
  // Boot argument guamp-emmc-div=N: faster SD clock (base / 2N, N < 8) kept only if the GPT header and a CMD18 run
  // of the partition entries read back identical to the proven clock; otherwise back to N = 8
  uint32_t wanted = 8;
  if (PE_parse_boot_argn("guamp-emmc-div", &wanted, sizeof(wanted)) && wanted >= 1 && wanted < 8) {
    uint8_t *reference = static_cast<uint8_t *>(IOMallocZero(33 * kBlock)), *check = static_cast<uint8_t *>(IOMallocZero(33 * kBlock));
    bool ok = reference && check && card.readBlocks(1, 33, reference) && card.host.setClock(wanted);
    for (unsigned pass = 0; ok && pass < 4; pass++) {
      memset(check, 0, 33 * kBlock);
      ok = card.readBlocks(1, 33, check) && !memcmp(reference, check, 33 * kBlock);
    }
    if (!ok) {
      card.host.resetLines();
      card.host.setClock(8);
    }
    IOLog("NativeEmmc: clock divider %u requested, %u in use\n", wanted, card.host.divider);
    if (reference) IOFree(reference, 33 * kBlock);
    if (check) IOFree(check, 33 * kBlock);
  }
  // Boot argument guamp-emmc-dma=1: ADMA2 reads, kept only if three DMA reads of the GPT header and entries match PIO
  uint32_t dma = 0;
  if (PE_parse_boot_argn("guamp-emmc-dma", &dma, sizeof(dma)) && dma) {
    uint8_t *reference = static_cast<uint8_t *>(IOMallocZero(33 * kBlock)), *check = static_cast<uint8_t *>(IOMallocZero(33 * kBlock));
    bool ok = reference && check && card.host.setupDma() && card.readBlocks(1, 33, reference);
    for (unsigned pass = 0; ok && pass < 3; pass++) {
      memset(check, 0, 33 * kBlock);
      ok = card.host.readDma(1, 33, check) && !memcmp(reference, check, 33 * kBlock);
    }
    if (!ok) card.host.resetLines();
    native_emmc_dma = ok ? 1 : 0;
    IOLog("NativeEmmc: ADMA2 reads %s\n", ok ? "verified, in use" : "refused, PIO");
    if (reference) IOFree(reference, 33 * kBlock);
    if (check) IOFree(check, 33 * kBlock);
  }
  IOLog("NativeEmmc: %llu blocks, log kpan at %llu (%llu blocks), userdata at %llu (%llu blocks)\n",
        card.capacity, card.logFirst, card.logBlocks, card.dataFirst, card.dataBlocks);
  card.fb = mapPhysical(kFbPhys, kFbBytes);
  card.fbShadow = static_cast<uint8_t *>(IOMallocZero(kFbBytes + kBlock));
  if (card.fbShadow) memset(card.fbShadow, 0xA5, kFbBytes + kBlock);
  card.seen = static_cast<uint8_t *>(IOMallocZero((card.capacity + 63) / 64));
  card.online = true;
  if (!guamp_panic_base) {
    void *region = IOMallocZero(kPanicBytes);
    if (region) {
      guamp_panic_size = kPanicBytes;
      guamp_panic_info = region;
      guamp_panic_base = reinterpret_cast<vm_offset_t>(region);
      debug_log_init();
      IOLog("NativeEmmc: panic region %u bytes, copied to kpan block %u on panic\n", kPanicBytes, kPanicFirstBlock);
    }
  }
  PE_parse_boot_argn("guamp-emmc-reset", &card.resetAfter, sizeof(card.resetAfter));
  card.mirrorCall = thread_call_allocate(mirrorEntry, nullptr);
  if (card.mirrorCall) mirrorEntry(nullptr, nullptr);
  publish(0, card.capacity, "user area");
  publishWindows();
  usbProbe();
  usbMirrorStart();
}

// Called by ArmDTPlatform::configure; the kext load must not run inside the platform's own start
extern "C" void guamp_emmc_start(IOService *platform)
{
  gPlatform = platform;
  platform->retain();
  thread_call_t call = thread_call_allocate(startLater, nullptr);
  if (call) thread_call_enter(call);
}

// ---------------------------------------------------------------------------------------------------------------
// Root and SystemOS hooks of the UniXNU owner-root kernel patches (offline-owner-*: XNU_VERIFIED_ROOT_PROBE,
// OWNER_ROOT_AUDIT/TRUST, OWNER_CACHE_MOUNT/STARTUP, OWNER_BOOT_MANIFEST). On the UniXNU reference boards a
// verified block driver supplies them from a signed policy; here the root and SystemOS volumes are the unchanged
// IPSW images in the eMMC windows above, published read-only by this driver, and these hooks accept exactly the
// media that sit on NativeEmmc (CONFIG_XNU_VERIFIED_ROOT_ARCHIVE links this archive in place of that driver).
// ---------------------------------------------------------------------------------------------------------------

// The NativeEmmc device under this media, or null
static NativeEmmc *emmcUnder(IOService *media)
{
  unsigned depth = 0;
  for (IOService *provider = media; provider && depth++ < 64; provider = provider->getProvider()) {
    if (NativeEmmc *disk = OSDynamicCast(NativeEmmc, provider)) return disk;
  }
  return nullptr;
}

// Media of the unchanged IPSW images: on a read-only NativeEmmc range
static bool onEmmc(IOService *media)
{
  NativeEmmc *disk = emmcUnder(media);
  return disk && !disk->isWritable();
}

// The IOMedia for a BSD device number, retained, or null
static IOService *mediaForDevice(dev_t device)
{
  OSDictionary *matching = IOService::serviceMatching("IOMedia");
  if (!matching) return nullptr;
  OSIterator *iterator = IOService::getMatchingServices(matching);
  matching->release();
  if (!iterator) return nullptr;
  IOService *found = nullptr;
  while (OSObject *object = iterator->getNextObject()) {
    IOService *media = OSDynamicCast(IOService, object);
    OSNumber *mjr = media ? OSDynamicCast(OSNumber, media->getProperty(kIOBSDMajorKey)) : nullptr;
    OSNumber *mnr = media ? OSDynamicCast(OSNumber, media->getProperty(kIOBSDMinorKey)) : nullptr;
    if (mjr && mnr && mjr->unsigned32BitValue() == static_cast<uint32_t>(major(device)) &&
        mnr->unsigned32BitValue() == static_cast<uint32_t>(minor(device))) {
      found = media;
      found->retain();
      break;
    }
  }
  iterator->release();
  return found;
}

extern "C" void native_verified_root_probe(void) {}

// The boot-time GPU addon of the owner Data mount (zzy-native-gpu-addon-mount) comes from NativeRamBlock on the
// reference boards; guamp has no GPU addon
extern "C" int native_ram_block_gpu_addon(const uint8_t **data, uint64_t *bytes)
{
  if (data) *data = nullptr;
  if (bytes) *bytes = 0;
  return 0;
}

extern "C" int native_verified_block_root_authorized(dev_t device)
{
  IOService *media = mediaForDevice(device);
  bool ok = media && onEmmc(media);
  OSSafeReleaseNULL(media);
  return ok;
}

// The single IOMedia with this APFS volume UUID on a NativeEmmc range of the wanted kind (read-only image or the
// writable Data window); waits for APFS to publish it (it runs right after the root mount, while the containers
// may still be probed). Returns that range, or null.
static NativeEmmc *findVolume(const char *uuid, bool writable, dev_t *device, char *path, size_t size)
{
  if (!uuid || !device || !path || size < 6) return nullptr;
  *device = NODEV;
  path[0] = 0;
  OSDictionary *matching = IOService::serviceMatching("IOMedia");
  OSDictionary *properties = OSDictionary::withCapacity(1);
  OSString *wanted = OSString::withCString(uuid);
  bool ready = matching && properties && wanted && properties->setObject("UUID", wanted) &&
      matching->setObject(gIOPropertyMatchKey, properties);
  OSSafeReleaseNULL(wanted);
  OSSafeReleaseNULL(properties);
  if (!ready) { OSSafeReleaseNULL(matching); return nullptr; }
  IOService *media = IOService::waitForMatchingService(matching, 60ULL * NSEC_PER_SEC);
  matching->release();
  if (!media) { IOLog("NativeEmmc: volume %s not published\n", uuid); return nullptr; }
  OSString *name = OSDynamicCast(OSString, media->getProperty(kIOBSDNameKey));
  OSNumber *mjr = OSDynamicCast(OSNumber, media->getProperty(kIOBSDMajorKey));
  OSNumber *mnr = OSDynamicCast(OSNumber, media->getProperty(kIOBSDMinorKey));
  NativeEmmc *disk = emmcUnder(media);
  if (!disk || disk->isWritable() != writable) disk = nullptr;
  if (disk && name && mjr && mnr && name->getLength() + 6 < size) {
    *device = static_cast<dev_t>(makedev(mjr->unsigned32BitValue(), mnr->unsigned32BitValue()));
    snprintf(path, size, "/dev/%s", name->getCStringNoCopy());
    IOLog("NativeEmmc: volume %s is %s\n", uuid, path);
  } else {
    IOLog("NativeEmmc: volume %s refused (not on a %s eMMC range)\n", uuid, writable ? "writable" : "read-only");
    disk = nullptr;
  }
  media->release();
  return disk;
}

extern "C" int native_verified_block_volume_device(const char *uuid, dev_t *device, char *path, size_t size)
{
  return findVolume(uuid, false, device, path, size) != nullptr;
}

// Data volume of the owner Data mount (kern_exec.c native_owner_data_prepare, XNU_NATIVE_RAM_BLOCK path; the
// reference boards answer it from NativeRamBlock): the volume with this UUID in the writable Data window. Seed
// base and size are the window's eMMC byte range; the digest only names the volume (the window is not a fixed seed).
extern "C" int native_ram_block_volume_seed(const char *uuid, dev_t *device, char *path, size_t size,
    uint64_t *base, uint64_t *bytes, uint8_t digest[32])
{
  if (!base || !bytes || !digest) return 0;
  *base = 0;
  *bytes = 0;
  memset(digest, 0, 32);
  NativeEmmc *disk = findVolume(uuid, true, device, path, size);
  if (!disk) return 0;
  *base = disk->firstBlock() * kBlock;
  *bytes = disk->blockCount() * kBlock;
  static const char domain[] = "guamp.NativeEmmc.data-window.v1";
  SHA256_CTX context;
  SHA256_Init(&context);
  SHA256_Update(&context, domain, sizeof(domain));
  SHA256_Update(&context, uuid, strlen(uuid));
  SHA256_Final(digest, &context);
  return 1;
}

// Identity of a volume for the owner boot manifest: SHA-384 over a fixed domain and the volume UUID
extern "C" int native_verified_block_manifest_sha384(dev_t device, const char *uuid, uint8_t digest[48])
{
  if (!uuid || !native_verified_block_root_authorized(device)) return 0;
  static const char domain[] = "guamp.NativeEmmc.unchanged-ipsw-volume.v1";
  SHA384_CTX context;
  SHA384_Init(&context);
  SHA384_Update(&context, domain, sizeof(domain));
  SHA384_Update(&context, uuid, strlen(uuid));
  SHA384_Final(digest, &context);
  return 1;
}

// Called by ArmDTPlatform::haltRestart before the restart, power off or panic step of type (IOPlatformExpert
// kPE* values). A panic may stop the machine while the gate is held, so a panic flush does not wait for it.
extern "C" void guamp_emmc_flush(unsigned type)
{
  Card &card = gCard;
  if (!card.online || !card.gate) return;
  bool panic = type >= kPEPanicRestartCPU && type != kPEPagingOff;
  bool locked = panic ? IOLockTryLock(card.gate) : (IOLockLock(card.gate), true);
  // A panic may have stopped another thread in the middle of a transfer: start from clean CMD/DAT lines
  if (panic) card.host.resetLines();
  card.flushType = type + 1;
  if (panic) card.savePanic();
  card.mirror(true);
  card.host.flushCache();                  // Data window writes still in the card cache
  if (locked) IOLockUnlock(card.gate);
}
