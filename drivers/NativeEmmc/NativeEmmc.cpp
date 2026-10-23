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
// Two jobs:
//  - the whole user area is published as one read-only IOBlockStorageDevice (IOGUIDPartitionScheme
//    splits it into the GPT partitions);
//  - the kernel message buffer is mirrored every second to the empty "kpan" partition, the only block
//    range this driver ever writes, so the boot can be read from TWRP afterwards
//    (strumenti/leggi-log-xnu.sh). Boot argument guamp-emmc-reset=N: PSCI reset after N mirrors.
//
// Started by the device ArmDTPlatform at the end of configure() (guamp_emmc_start), no personality.

#include <IOKit/IOLib.h>
#include <IOKit/storage/IOBlockStorageDevice.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <kern/clock.h>
#include <kern/thread_call.h>
#include <machine/machine_routines.h>
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
void guamp_emmc_start(IOService *platform);
// Observation: last failing init stage (0 = ready), blocks moved, mirror passes.
volatile uint32_t native_emmc_stage = 0xff;
uint64_t native_emmc_reads = 0, native_emmc_writes = 0, native_emmc_errors = 0, native_emmc_mirrors = 0;   // under the gate
}

namespace {

constexpr uint64_t kSdhcPhys = 0x04744000, kGccSdccPhys = 0x01438000;
constexpr uint32_t kBlock = 512;
constexpr uint32_t kBounceBlocks = 128;

// SDHCI registers
enum : uint32_t {
  rArg = 0x08, rXfer = 0x0C, rCmd = 0x0E, rResp = 0x10, rData = 0x20, rPresent = 0x24, rHostCtl = 0x28,
  rPower = 0x29, rClock = 0x2C, rTimeout = 0x2E, rReset = 0x2F, rStatus = 0x30, rStatusEn = 0x34, rSignalEn = 0x38,
  rHostCtl2 = 0x3E, rBlockSize = 0x04, rBlockCount = 0x06,
  // msm-v5 vendor ("MCI removed" layout)
  rPwrStatus = 0x240, rPwrMask = 0x244, rPwrClear = 0x248, rPwrCtl = 0x24C,
  // CQHCI, one page above the host
  rCqCfg = 0x1000 + 0x08,
};

enum : uint32_t { sCmdDone = 1u << 0, sXferDone = 1u << 1, sWriteReady = 1u << 4, sReadReady = 1u << 5, sErrors = 0xFFFF8000u };

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
      if (status & sErrors) { w32(rStatus, status); return 2; }
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

  // R1/R1b command without data
  bool command(uint8_t index, uint32_t argument, bool busy) {
    w32(rStatus, 0xFFFFFFFF);
    w32(rArg, argument);
    w16(rXfer, 0);
    w16(rCmd, static_cast<uint16_t>((index << 8) | (1 << 4) | (1 << 3) | (busy ? 0x3 : 0x2)));
    if (wait(sCmdDone, 100)) return false;
    if (busy && wait(sXferDone, 1000)) return false;
    return true;
  }

  // Returns 0 when ready, else the failing stage (native_emmc_stage)
  unsigned init() {
    base = reinterpret_cast<volatile uint8_t *>(ml_io_map(kSdhcPhys, 0x2000));
    gcc = reinterpret_cast<volatile uint8_t *>(ml_io_map(kGccSdccPhys, 0x1000));
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
      resetLines();
    }
    return false;
  }

  bool writeBlock(uint64_t lba, const uint8_t *in) {
    for (unsigned attempt = 0; attempt < 3; attempt++) {
      w32(rStatus, 0xFFFFFFFF);
      w16(rBlockSize, kBlock);
      w16(rBlockCount, 1);
      w32(rArg, static_cast<uint32_t>(lba));
      w16(rXfer, (1 << 1));
      w16(rCmd, (24 << 8) | (1 << 5) | (1 << 4) | (1 << 3) | 0x2);
      if (!wait(sCmdDone, 100) && !wait(sWriteReady, 1000)) {
        const uint32_t *words = reinterpret_cast<const uint32_t *>(in);
        for (unsigned i = 0; i < kBlock / 4; i++) w32(rData, words[i]);
        if (!wait(sXferDone, 2000)) { native_emmc_writes++; return true; }
      }
      native_emmc_errors++;
      resetLines();
    }
    return false;
  }
};

} // namespace

class NativeEmmc : public IOBlockStorageDevice {
  OSDeclareDefaultStructors(NativeEmmc)
  IOLock *gate = nullptr;
  Host host;
  uint64_t capacity = 0;                 // blocks
  uint64_t logFirst = 0, logBlocks = 0;  // "kpan"
  uint8_t *bounce = nullptr;             // kBounceBlocks blocks
  uint8_t *shadow = nullptr;             // last mirrored message buffer
  uint32_t shadowSize = 0;
  uint32_t sequence = 0, resetAfter = 0;
  thread_call_t mirrorCall = nullptr;
  bool online = false;

  bool readBlocks(uint64_t lba, uint32_t count, uint8_t *out) {
    for (uint32_t i = 0; i < count; i++) {
      if (!host.readBlock(lba + i, out + static_cast<uint64_t>(i) * kBlock)) return false;
    }
    return true;
  }
  // The only writes of this driver, confined to the log partition
  bool writeLogBlock(uint64_t index, const uint8_t *in) {
    if (!logBlocks || index >= logBlocks) return false;
    return host.writeBlock(logFirst + index, in);
  }
  bool findLogPartition();
  void mirror();
  static void mirrorEntry(thread_call_param_t self, thread_call_param_t) { static_cast<NativeEmmc *>(self)->mirror(); }
  void armMirror() {
    uint64_t when = 0;
    clock_interval_to_deadline(1, kSecondScale, &when);
    thread_call_enter_delayed(mirrorCall, when);
  }

public:
  bool bringUp();
  IOReturn doEjectMedia() override { return kIOReturnUnsupported; }
  IOReturn doFormatMedia(UInt64) override { return kIOReturnUnsupported; }
  UInt32 doGetFormatCapacities(UInt64 *, UInt32) const override { return 0; }
  char *getVendorString() override { return const_cast<char *>("Qualcomm"); }
  char *getProductString() override { return const_cast<char *>("SDHC1 eMMC"); }
  char *getRevisionString() override { return const_cast<char *>("1.0"); }
  char *getAdditionalDeviceInfoString() override { return const_cast<char *>("polled PIO, read-only"); }
  IOReturn reportBlockSize(UInt64 *value) override { *value = kBlock; return kIOReturnSuccess; }
  IOReturn reportEjectability(bool *value) override { *value = false; return kIOReturnSuccess; }
  IOReturn reportMaxValidBlock(UInt64 *value) override { *value = capacity - 1; return kIOReturnSuccess; }
  IOReturn reportMediaState(bool *present, bool *changed) override {
    *present = online; if (changed) *changed = false; return kIOReturnSuccess;
  }
  IOReturn reportRemovability(bool *value) override { *value = false; return kIOReturnSuccess; }
  IOReturn reportWriteProtection(bool *value) override { *value = true; return kIOReturnSuccess; }
  IOReturn doSynchronizeCache() override { return online ? kIOReturnSuccess : kIOReturnNotReady; }
  IOReturn doSynchronize(UInt64, UInt64, IOOptionBits) override { return doSynchronizeCache(); }
  IOReturn getWriteCacheState(bool *value) override { *value = false; return kIOReturnSuccess; }
  IOReturn setWriteCacheState(bool) override { return kIOReturnUnsupported; }
  IOReturn doAsyncReadWrite(IOMemoryDescriptor *, UInt64, UInt64, IOStorageAttributes *, IOStorageCompletion *) override;
};
OSDefineMetaClassAndStructors(NativeEmmc, IOBlockStorageDevice)

bool NativeEmmc::findLogPartition()
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
  static const char name[] = "kpan";
  const uint32_t perBlock = kBlock / entrySize;
  for (uint32_t i = 0; i < entries; i++) {
    if (i % perBlock == 0 && !readBlocks(entriesLba + i / perBlock, 1, block)) return false;
    const uint8_t *entry = block + (i % perBlock) * entrySize;
    bool match = true;
    for (unsigned c = 0; c < sizeof(name); c++) {
      uint16_t unit = static_cast<uint16_t>(entry[56 + 2 * c] | (entry[57 + 2 * c] << 8));
      if (unit != static_cast<uint8_t>(name[c])) { match = false; break; }
    }
    if (!match) continue;
    uint64_t first, last;
    memcpy(&first, entry + 32, 8);
    memcpy(&last, entry + 40, 8);
    if (first < 34 || last < first || last >= capacity || last - first + 1 > (64ULL << 20) / kBlock) return false;
    logFirst = first;
    logBlocks = last - first + 1;
    return true;
  }
  return false;
}

// Log layout in kpan: block 0 = header, blocks 1.. = the raw message ring (msg_size bytes).
//   0 "GUAMPLOG"  8 version 1  12 msg_size  16 msg_bufx  20 sequence  24 init stage  28 msg_magic
//   32 mach_absolute_time  40 reads  48 writes  56 errors
void NativeEmmc::mirror()
{
  IOLockLock(gate);
  guamp_msgbuf *ring = msgbufp;
  if (ring && ring->msg_bufc && ring->msg_size > 0) {
    uint32_t size = static_cast<uint32_t>(ring->msg_size);
    uint32_t blocks = (size + kBlock - 1) / kBlock;
    if (blocks + 1 > logBlocks) blocks = static_cast<uint32_t>(logBlocks - 1);
    if (shadowSize != size) {
      if (shadow) IOFree(shadow, static_cast<vm_size_t>(shadowSize) + kBlock);
      shadowSize = size;
      shadow = static_cast<uint8_t *>(IOMallocZero(static_cast<vm_size_t>(size) + kBlock));
      if (shadow) memset(shadow, 0xA5, static_cast<size_t>(size) + kBlock);   // force a first full write
    }
    bool ok = shadow != nullptr;
    for (uint32_t b = 0; ok && b < blocks; b++) {
      uint8_t *page = bounce;
      uint32_t offset = b * kBlock, length = size - offset < kBlock ? size - offset : kBlock;
      memset(page, 0, kBlock);
      memcpy(page, ring->msg_bufc + offset, length);
      if (!memcmp(page, shadow + offset, kBlock)) continue;
      ok = writeLogBlock(1 + b, page);
      if (ok) memcpy(shadow + offset, page, kBlock);
    }
    uint8_t *header = bounce;
    memset(header, 0, kBlock);
    memcpy(header, "GUAMPLOG", 8);
    uint32_t words[6] = { 1, size, static_cast<uint32_t>(ring->msg_bufx), ++sequence, native_emmc_stage,
                          static_cast<uint32_t>(ring->msg_magic) };
    memcpy(header + 8, words, sizeof(words));
    uint64_t counters[4] = { mach_absolute_time(), native_emmc_reads, native_emmc_writes, native_emmc_errors };
    memcpy(header + 32, counters, sizeof(counters));
    if (ok) writeLogBlock(0, header);
    native_emmc_mirrors++;
  }
  IOLockUnlock(gate);
  if (resetAfter && sequence >= resetAfter) {
    register uint64_t x0 __asm__("x0") = 0x84000009;   // PSCI SYSTEM_RESET
    __asm__ volatile("smc #0" : "+r"(x0) : : "memory");
  }
  armMirror();
}

bool NativeEmmc::bringUp()
{
  gate = IOLockAlloc();
  bounce = static_cast<uint8_t *>(IOMallocZero(kBounceBlocks * kBlock));
  if (!gate || !bounce) return false;
  native_emmc_stage = host.init();
  if (native_emmc_stage) { IOLog("NativeEmmc: init failed at stage %u\n", native_emmc_stage); return false; }
  if (!findLogPartition()) { native_emmc_stage = 11; IOLog("NativeEmmc: GPT or kpan not found\n"); return false; }
  IOLog("NativeEmmc: %llu blocks, log kpan at %llu (%llu blocks)\n", capacity, logFirst, logBlocks);
  online = true;
  PE_parse_boot_argn("guamp-emmc-reset", &resetAfter, sizeof(resetAfter));
  mirrorCall = thread_call_allocate(&NativeEmmc::mirrorEntry, this);
  if (mirrorCall) mirror();
  return true;
}

IOReturn NativeEmmc::doAsyncReadWrite(IOMemoryDescriptor *buffer, UInt64 block, UInt64 count,
    IOStorageAttributes *, IOStorageCompletion *completion)
{
  if (!buffer || !completion || !completion->action || !count || block >= capacity || count > capacity - block ||
      buffer->getLength() < count * kBlock) return kIOReturnBadArgument;
  if (buffer->getDirection() != kIODirectionIn) return kIOReturnNotWritable;
  IOLockLock(gate);
  if (!online) { IOLockUnlock(gate); return kIOReturnNotReady; }
  uint64_t done = 0;
  IOReturn status = kIOReturnSuccess;
  while (done < count) {
    uint32_t n = count - done > kBounceBlocks ? kBounceBlocks : static_cast<uint32_t>(count - done);
    if (!readBlocks(block + done, n, bounce)) { status = kIOReturnIOError; break; }
    uint64_t bytes = static_cast<uint64_t>(n) * kBlock;
    if (buffer->writeBytes(done * kBlock, bounce, bytes) != bytes) { status = kIOReturnIOError; break; }
    done += n;
  }
  IOLockUnlock(gate);
  completion->action(completion->target, completion->parameter, status, done * kBlock);
  return kIOReturnSuccess;
}

extern "C" void guamp_emmc_start(IOService *platform)
{
  NativeEmmc *disk = OSTypeAlloc(NativeEmmc);
  if (!disk) return;
  if (disk->init() && disk->bringUp() && disk->attach(platform)) {
    if (disk->start(platform)) disk->registerService();
    else disk->detach(platform);
  }
  disk->release();
}
