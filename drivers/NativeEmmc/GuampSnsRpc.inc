// SPDX-License-Identifier: BSD-2-Clause-Patent
// Kernel glue of the FastRPC listener for the ADSP sensors PD (GuampFastRpcCore.h, GuampSnsFiles.h;
// analisi/sensori/SENSORI-VERI-20261029.md). snsrpc::start() registers adspipc::gRpcGlue before adsp::start(); the
// guamp-adsp thread calls prepare() once the channel fastrpcglink-apps-dsp is up (guamp-sensors=2, guamp-sensors-rpc=1,
// the default). prepare():
//   files   persist (ext4, read-only: /sensors -> /mnt/vendor/persist/sensors), the optional ADT property
//           /chosen/guamp-sensors-config (vendor json copy, scripts/guamp-sensors-config.py), synthesized sysfs
//           answers; the PD's writes stay in RAM (lost at reboot, the persist partition is never written)
//   buffer  256 KiB physically contiguous, uncached, below 4 GiB: the invoke payloads. ADSP address by
//           guamp-sensors-rpc-mem: 0 (default) the apps SMMU context bank of the first adsprpc compute session
//           (DT msm_fastrpc_compute_cb10: iommus <&apps_smmu 0x1c3 0>, IOVA pool 0x80000000; adsprpc passes
//           phys + (sid & 0xf) << 32, so 0x3_8000_0000), 1 physical address with msw 0, 2 physical with msw 3
// Included by NativeEmmc.cpp after GuampAudioPlay.inc (GuampSmmu.inc, mapPhysical, IOBufferMemoryDescriptor).
#include <IOKit/IODeviceTreeSupport.h>
#include "GuampSmmu.inc"
#include "GuampSnsFiles.h"

namespace snsrpc {

constexpr uint32_t kRegion = 256 * 1024, kBufs = 64 * 1024;
constexpr uint64_t kSmmuPhys = 0x0c600000, kIova = 0x80000000ull;
constexpr uint32_t kStream = 0x1c3, kBank = kStream & 0xf;
using RpcSmmu = guampwifi::SmmuContext<kStream, 0, 0x80000000ull, 0x80400000ull>;

unsigned gMem = 0;
IOBufferMemoryDescriptor *gBuf = nullptr, *gTables = nullptr;
volatile uint8_t *gRegs = nullptr;
uint8_t *gTablesCpu = nullptr;
RpcSmmu gSmmu;
uint32_t gLastFsr = 0;

uint32_t regRd(void *, uint32_t at) { return *reinterpret_cast<volatile uint32_t *>(gRegs + at); }
void regWr(void *, uint32_t at, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(gRegs + at) = v; }
uint32_t tblRd(void *, uint32_t at) { return *reinterpret_cast<volatile uint32_t *>(gTablesCpu + at); }
void tblWr(void *, uint32_t at, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(gTablesCpu + at) = v; }
void fence(void *) { __asm__ volatile("dsb sy" ::: "memory"); }
void delayUs(void *, uint32_t us) { IODelay(us); }
void *vfsAlloc(void *, uint32_t n) { return IOMalloc(n ? n : 1); }
void vfsFree(void *, void *p, uint32_t n) { IOFree(p, n ? n : 1); }

// The context bank for stream 0x1c3, once per boot
bool smmuUp() {
  gRegs = mapPhysical(kSmmuPhys, 0x80000);
  gTables = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
      kernel_task, static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous) |
                       static_cast<IOOptionBits>(kIOMapInhibitCache), RpcSmmu::TableBytes, 0xFFFFF000ULL);
  gTablesCpu = gTables ? static_cast<uint8_t *>(gTables->getBytesNoCopy()) : nullptr;
  const uint64_t tables = gTables ? gTables->getPhysicalSegment(0, nullptr, kIOMemoryMapperNone) : 0;
  if (!gRegs || !gTablesCpu || !tables || (tables & 0xfff)) {
    IOLog("GuampRpc: SMMU context: mapping/table allocation failed\n");
    return false;
  }
  const guampwifi::Memory regs = {nullptr, 0x80000, regRd, regWr, fence};
  const guampwifi::Memory tbl = {nullptr, RpcSmmu::TableBytes, tblRd, tblWr, fence};
  const int e = gSmmu.start(regs, tbl, tables, guamp_phys_base, guamp_phys_size, true, nullptr, delayUs);
  IOLog("GuampRpc: SMMU context for stream %x: %s (SMR slot %u%s, context bank %u, ASID %u)\n", kStream,
        guampwifi::errorName(e), gSmmu.slot, gSmmu.preexisting ? " reused" : "", gSmmu.cb, gSmmu.asid);
  return e == guampwifi::Ok && gSmmu.active;
}

void report(const char *when) {
  if (!gSmmu.active) return;
  const uint32_t fsr = gSmmu.faultStatus(), faults = fsr & 0xc00001feu;
  if (faults == gLastFsr) return;
  gLastFsr = faults;
  if (!faults) return;
  IOLog("GuampRpc: SMMU stream %x %s: FAULT FSR %x FAR %x%08x FSYNR0 %x\n", kStream, when, fsr, gSmmu.faultAddressHigh(),
        gSmmu.faultAddressLow(), gSmmu.faultSyndrome());
  gSmmu.clearFault();
}

// persist /sensors into the Vfs (read-only ext4)
void loadPersist(guamp_rpc::Vfs *vfs) {
  wifiprep::Window window = {};
  uint64_t attributes = 0;
  if (!wifiprep::findPartition("persist", &window.first, &window.blocks, &attributes)) {
    IOLog("GuampRpc: partition persist not found: no registry\n");
    return;
  }
  uint8_t *scratch = static_cast<uint8_t *>(IOMallocZero(ext4::kScratchBytes));
  ext4::Fs *fs = static_cast<ext4::Fs *>(IOMallocZero(sizeof(ext4::Fs)));
  if (scratch && fs) {
    const int r = fs->mount(wifiprep::readWindow, &window, window.blocks, scratch);
    if (r) {
      IOLog("GuampRpc: persist ext4 mount: %s\n", ext4::errorName(r));
    } else {
      const uint64_t t0 = mach_absolute_time();
      const guamp_rpc::TreeStats st = guamp_rpc::loadExt4(*fs, "/sensors", guamp_rpc::kPersistMount, *vfs);
      uint64_t ns = 0;
      absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
      IOLog("GuampRpc: persist /sensors: %u files, %u directories, %u bytes, %u skipped, %s, %llu ms\n", st.files, st.dirs,
            st.bytes, st.skipped, ext4::errorName(st.error), ns / 1000000);
    }
  }
  if (scratch) IOFree(scratch, ext4::kScratchBytes);
  if (fs) IOFree(fs, sizeof(ext4::Fs));
}

// The optional vendor json copy in the ADT (kept retained: the Vfs borrows its bytes)
void loadAdt(guamp_rpc::Vfs *vfs) {
  IORegistryEntry *chosen = IORegistryEntry::fromPath("/chosen", gIODTPlane);
  OSData *d = chosen ? OSDynamicCast(OSData, chosen->getProperty("guamp-sensors-config")) : nullptr;
  uint32_t files = 0;
  if (d) {
    d->retain();
    const bool ok = guamp_rpc::loadBlob(static_cast<const uint8_t *>(d->getBytesNoCopy()), d->getLength(), *vfs, &files);
    IOLog("GuampRpc: ADT guamp-sensors-config: %u bytes, %u files%s\n", d->getLength(), files, ok ? "" : " (MALFORMED)");
  } else {
    IOLog("GuampRpc: no ADT guamp-sensors-config: sns_reg_config synthesized, no json (enough while the persist registry is "
          "current)\n");
  }
  OSSafeReleaseNULL(chosen);
}

guamp_rpc::Listener *prepare() {
  guamp_rpc::Vfs *vfs = static_cast<guamp_rpc::Vfs *>(IOMallocZero(sizeof(guamp_rpc::Vfs)));
  guamp_rpc::Listener *l = static_cast<guamp_rpc::Listener *>(IOMallocZero(sizeof(guamp_rpc::Listener)));
  uint8_t *in = static_cast<uint8_t *>(IOMallocZero(kBufs)), *out = static_cast<uint8_t *>(IOMallocZero(kBufs));
  if (!vfs || !l || !in || !out) {
    IOLog("GuampRpc: out of memory\n");
    return nullptr;
  }
  vfs->hooks.alloc = vfsAlloc;
  vfs->hooks.free = vfsFree;
  loadPersist(vfs);
  loadAdt(vfs);
  const uint32_t synthetic = guamp_rpc::addSynthetic(*vfs);
  IOLog("GuampRpc: files ready: %u entries, %u bytes in RAM, %u synthesized\n", vfs->count, vfs->ramBytes, synthetic);

  gBuf = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
      kernel_task, static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous) |
                       static_cast<IOOptionBits>(kIOMapInhibitCache), kRegion, 0xFFFFF000ULL);
  uint8_t *cpu = gBuf ? static_cast<uint8_t *>(gBuf->getBytesNoCopy()) : nullptr;
  const uint64_t phys = gBuf ? gBuf->getPhysicalSegment(0, nullptr, kIOMemoryMapperNone) : 0;
  if (!cpu || !phys || (phys & 0xfff) || phys + kRegion > 0x100000000ULL) {
    IOLog("GuampRpc: ADSP buffer allocation failed (phys %llx)\n", static_cast<unsigned long long>(phys));
    return nullptr;
  }
  uint64_t dsp = 0;
  if (gMem == 0) {
    if (!smmuUp()) {
      IOLog("GuampRpc: no SMMU context for stream %x; physical modes: guamp-sensors-rpc-mem=1 (msw 0) or =2 (msw 3)\n", kStream);
      return nullptr;
    }
    const int e = gSmmu.map(kIova, phys, kRegion);
    IOLog("GuampRpc: buffer phys %llx -> IOVA %llx (%u bytes): %s\n", static_cast<unsigned long long>(phys),
          static_cast<unsigned long long>(kIova), kRegion, guampwifi::errorName(e));
    if (e) return nullptr;
    dsp = kIova | static_cast<uint64_t>(kBank) << 32;
  } else {
    dsp = phys | (gMem == 2 ? static_cast<uint64_t>(kBank) << 32 : 0);
    IOLog("GuampRpc: buffer phys %llx, ADSP address %llx (guamp-sensors-rpc-mem=%u)\n", static_cast<unsigned long long>(phys),
          static_cast<unsigned long long>(dsp), gMem);
  }
  l->shm.cpu = cpu;
  l->shm.dsp = dsp;
  l->shm.bytes = kRegion;
  l->shm.zero(0, kRegion);
  l->server.vfs = vfs;
  l->inStage = in;
  l->outStage = out;
  l->bufsCap = kBufs;
  return l;
}

// Before adsp::start(): the glue the guamp-adsp thread uses with guamp-sensors=2
void start() {
  PE_parse_boot_argn("guamp-sensors-rpc-mem", &gMem, sizeof(gMem));
  if (gMem > 2) gMem = 0;
  adspipc::gRpcGlue.prepare = prepare;
  adspipc::gRpcGlue.report = report;
}

}  // namespace snsrpc
