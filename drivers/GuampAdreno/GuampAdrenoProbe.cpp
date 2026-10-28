// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp board probe of NativeAdreno on the Adreno 610 (SM6115 bengal: a6xx gen1, no GMU, the CPU owns the GPU's power domains).
// Adapted from UniXNU-tb128fu package/GpuProbe.cpp stages 7 and 8 (xVanTuring, BSD-2-Clause-Patent; SM6225 khaje, the same
// Adreno 610), with these differences:
//   - power: NativeEmmc's GPU thread (GpuService.inc, GuampGpuM1.h + RPM votes) has already powered the GPU; this file never
//     touches GCC/GPUCC
//   - secure mode: by default no zap shader; guamp-adreno-secvid=1 writes RBBM_SECVID_TRUST_CNTL = 0 (mainline's fallback; it
//     stalls on guamp: TrustZone-owned). guamp-adreno-zap=1: the original signed a610_zap started through TrustZone PAS by
//     NativeEmmc (GuampGpuZap.inc), then CP_SET_SECURE_MODE 0 on the ring, as mainline a6xx_hw_init and tb128fu zapLoad
//   - chip id 0x06010000 (bengal-gpu.dtsi qcom,chipid), the rest of the device facts as on tb128fu (A610: GMEM 128 + 4 KiB,
//     UBWC 1.0, highest bank bit 14, bengal-gpu.dtsi qcom,ubwc-mode 1 / qcom,highest-bank-bit 14)
// Default off: guamp-adreno=<level> (GuampAdreno.h). Every step logs a "GuampAdreno:" line before it touches hardware, so a bus
// hang shows where it stopped.
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOUserClient.h>
#include <libkern/c++/OSSharedPtr.h>
#include <libkern/c++/OSMetaClass.h>
#include <libkern/c++/OSSymbol.h>
#include <mach/kmod.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include "NativeAdrenoGpuContext.h"
#include "NativeAdrenoRing.h"
#include "NativeAdrenoCpInit.h"
#include "NativeAdrenoIrq.h"
#include "NativeAdrenoSubmit.h"
#include "NativeAdrenoTimer.h"
#include "NativeAdrenoABI.h"
#include "NativeAdrenoUserClient.h"
#include "GuampAdreno.h"
#include "GuampGpuRecoverCore.h"

extern "C" {
// GuampCaMetal.inc (NativeEmmc): the render server gate learns that the Metal device is up (1) or unusable (0), and that the GPU
// was parked after hangs it could not recover (guamp-gpu-recover, GuampAdrenoRecover.inc).
void guamp_ca_metal_gpu_ready(int ok);
void guamp_ca_metal_gpu_lost(void);
// GuampAdrenoRecover.inc: [0] hangs detected, [1] recoveries that passed their check, [2] that failed, [3] 1 once parked, [4] the
// last reason (grc_reason), [5] the last stuck fence, [6] fences signalled by the last recovery, [7] hangs injected.
volatile uint32_t guamp_gpu_recover[8];
// GuampGpuDcvs.inc (NativeEmmc): nonzero pauses the frequency governor; held during a reset and re-init, and for good once parked.
// Weak stand-in for a kernel built without the governor (static link: no weak_import).
__attribute__((weak)) volatile uint32_t guamp_gpu_dcvs_hold;
// Fence waits that slept until a GPU interrupt or spun.
volatile uint64_t native_gpu_idle_sleeps, native_gpu_idle_spins;
boolean_t ml_get_interrupts_enabled(void);
boolean_t ml_at_interrupt_context(void);
int get_preemption_level(void);
#include "AdrenoPM4.h"
IOReturn NativeAdrenoGpuContextBringup(volatile uint8_t *, size_t, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned,
                                       void (*)(uint32_t), NativeAdrenoGmuContextReport *);
IOReturn NativeAdrenoGpuContextBuffers(NativeAdrenoGpuBuffer out[5]);
NativeAdrenoVm *NativeAdrenoGpuContextVm();
IOReturn native_adreno_publish(NativeAdrenoVm *, NativeAdrenoSubmit *, NativeAdrenoRing *, const NativeAdrenoRingClock *,
                               const NativeAdrenoTimer *, const NativeAdrenoAbiDeviceInfo *, uint32_t, uint32_t);
IOService *native_adreno_published_service(void);
extern volatile uint32_t native_adreno_smmu_coherent_walk, native_adreno_pool_mib, native_adreno_wc_enabled,
    native_adreno_written_imports_owned;
// ArmCpu's SPI attach (also used by the touch service).
int armcpu_spi_attach(uint32_t, bool, void (*)(void *), void *, uint32_t *, uint64_t *) __attribute__((weak_import));
// The staged AGXAccelerator (build-native-adreno-accelerator.py adds it to the archive); absent unless that builder ran.
// The kernel is linked statically, where an undefined weak_import still fails: weak definitions stand in (instance size 0
// makes level 2 log "accelerator: ... size 0" and stop) and the staged objects override them.
void native_adreno_accelerator_static_init(void) __asm__("__GLOBAL__sub_I_NativeAdrenoAccelerator.cpp");
__attribute__((weak)) void native_adreno_accelerator_static_init(void) {}
__attribute__((weak)) IOService *native_adreno_accelerator_allocate(void) { return nullptr; }
__attribute__((weak)) unsigned native_adreno_accelerator_instance_size(void) { return 0; }
// NativeAdrenoPublish.cpp exports its boot log through sheng's UFS driver; guamp has none (-1 = not stored).
__attribute__((weak)) int boot_log_export_put(uint32_t, uint32_t, uint32_t, const void *, uint32_t) { return -1; }
kern_return_t OSKextLoadKextWithIdentifier(const char *);
extern kmod_info_t g_kernel_kmod_info;

// NativeAdrenoPublish.cpp hands scanout and present calls to the GOP presenter (NativeGOPPresenter.cpp on tb128fu). guamp's
// legacy display has no GPU hand-off yet: these weak stand-ins answer "not ready / unsupported" until a presenter defines them.
__attribute__((weak)) IOMemoryDescriptor *native_gop_handoff_memory(void) {return nullptr;}
__attribute__((weak)) IOReturn native_gop_handoff_call(uint32_t, task_t, const void *, size_t, void *, size_t) {
  return kIOReturnUnsupported;
}
}

namespace {
// kgsl_smmu (msm-arm-smmu-bengal.dtsi): base 0x59a0000, 0x10000; context banks at 0x8000 (IDR1 PAGESIZE 4K, NUMPAGENDXB 2,
// as the M2 probe read: idr1 20000005).
constexpr uint64_t kSmmu = 0x059A0000, kSmmuBytes = 0x10000;
constexpr uint32_t kGpuIntid = 32 + 177;   // bengal-gpu.dtsi kgsl_3d0_irq: SPI 177, level high

struct Region {OSSharedPtr<IOMemoryMap> keep; uintptr_t va = 0;};
bool mapRegion(uint64_t phys, uint64_t length, Region &region) {
  IODeviceMemory *memory = IODeviceMemory::withRange(phys, length);
  if (!memory) return false;
  region.keep = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
  memory->release();
  region.va = region.keep ? static_cast<uintptr_t>(region.keep->getVirtualAddress()) : 0;
  return region.va != 0;
}
uint32_t rd(const Region &r, uint32_t reg) {return *reinterpret_cast<volatile uint32_t *>(r.va + reg);}
void wr(const Region &r, uint32_t reg, uint32_t value) {
  *reinterpret_cast<volatile uint32_t *>(r.va + reg) = value;
  __asm__ volatile("dsb sy" ::: "memory");
}
bool poll(const Region &r, uint32_t reg, uint32_t mask, uint32_t want, unsigned us) {
  for (unsigned i = 0; i <= us; i = i + 1) {
    if ((rd(r, reg) & mask) == want) return true;
    IODelay(1);
  }
  return false;
}

struct Session {
  volatile uint8_t *gpu = nullptr;
  Region smmu;
  NativeAdrenoGmuContextReport report;
  NativeAdrenoGpuBuffer nb[5];
  OSSharedPtr<IOMemoryMap> nmap[5];
  volatile uint8_t *ncpu[5];
};
Session gSession;

// Stuck-core breadcrumb (GuampPerf.inc in NativeEmmc): the GPU register about to be accessed, published before the access.
extern "C" volatile uint32_t guamp_mmio_tag[3];
void mmioTag(uint32_t tag, uint32_t dword) {
  guamp_mmio_tag[0] = tag;
  guamp_mmio_tag[1] = dword;
  guamp_mmio_tag[2] = guamp_mmio_tag[2] + 1;
  __asm__ volatile("dmb ish" ::: "memory");
}
void gw(const Session &s, uint32_t dword, uint32_t value) {
  mmioTag('a', dword);
  *reinterpret_cast<volatile uint32_t *>(s.gpu + dword * 4) = value;
  __asm__ volatile("dsb sy" ::: "memory");
}
uint32_t gr(const Session &s, uint32_t dword) {
  mmioTag('A', dword);
  return *reinterpret_cast<volatile uint32_t *>(s.gpu + dword * 4);
}
void gw64(const Session &s, uint32_t dword, uint64_t value) {gw(s, dword, uint32_t(value)); gw(s, dword + 1, uint32_t(value >> 32));}

// guamp-gpu-recover=1 (GuampAdrenoRecover.inc): read at start, armed after the self test. The ring object's write index as its last
// CP_RB_WPTR write left it, the new submissions counted there, and the parked state in which CP_RB_RPTR reads as a full ring.
bool gRecEnabled;
volatile bool gRecArmed, gGpuParked;
volatile uint32_t gLastWptr, gSubmitCount;
void recoverNoteWptr(uint32_t value);
void recoverPoll();

// NativeAdreno's register, cache and clock callbacks: GPU registers by dword index through the caller's uncached window.
int naWrite(void *c, uint32_t reg, uint32_t value) {
  if (reg >= 0x90000 / 4) return 1;
  if (gRecEnabled && reg == 0x807) recoverNoteWptr(value);            // CP_RB_WPTR
  gw(*static_cast<Session *>(c), reg, value);
  return 0;
}
int naRead(void *c, uint32_t reg, uint32_t *value) {
  if (reg >= 0x90000 / 4) return 1;
  if (gGpuParked && reg == 0x806) {                                    // CP_RB_RPTR of a parked GPU: one past the write index
    *value = (gLastWptr + 1) & (uint32_t(gSession.nb[1].bytes / 4) - 1);
    return 0;
  }
  *value = gr(*static_cast<Session *>(c), reg);
  return 0;
}
int naBarrier(void *) {__asm__ volatile("dsb sy" ::: "memory"); return 0;}
// The kernel buffers are Normal non-cacheable for the GPU and write-combined for the CPU: ordering only, no cache maintenance.
// Every fence read of the ring passes here, so the armed hang detector runs here too (under the GPU lock of the client call).
int cachePublish(void *, const volatile uint32_t *, size_t) {__asm__ volatile("dsb sy" ::: "memory"); return 0;}
int cacheAcquire(void *, volatile uint32_t *, size_t) {
  __asm__ volatile("dsb sy" ::: "memory");
  if (gRecArmed) recoverPoll();
  return 0;
}
uint64_t naNow(void *) {
  uint64_t ns = 0;
  absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
  return ns / 1000;
}
// The GPU interrupt, as a6xx_irq: read RBBM_INT_0_STATUS and clear it through RBBM_INT_CLEAR_CMD first, so the level line drops
// before the EOI. The same non-error status 100000 times in a row masks the GPU interrupts (the waiters then poll every 100 us).
volatile bool gIrqAttached;
volatile uint32_t gIrqCount, gIrqSeen, gIrqLast, gIrqRepeat, gIrqMasked;
// Error causes the hang detector counts (guamp-gpu-recover): RBBM_INT_0 AHB error, CP_HW_ERROR, RBBM_HANG_DETECT, UCHE out of bounds.
constexpr uint32_t kRecoverIrqErrors = (1u << 1) | (1u << 9) | (1u << 23) | (1u << 24);
volatile uint32_t gIrqErrors;
void gpuIrq(void *c) {
  Session &s = *static_cast<Session *>(c);
  const uint32_t status = gr(s, 0x201);
  if (status) gw(s, 0x37, status);
  if (gRecEnabled && (status & kRecoverIrqErrors)) gIrqErrors = gIrqErrors | (status & kRecoverIrqErrors);
  gIrqRepeat = (status == gIrqLast && !(status & (1u << 20))) ? gIrqRepeat + 1 : 0;
  gIrqLast = status;
  gIrqSeen = gIrqSeen | status;
  gIrqCount = gIrqCount + 1;
  if (gIrqRepeat > 100000) {gw(s, 0x38, 0); gIrqMasked = 1;}
  thread_wakeup(event_t(&gIrqCount));
}
int naIdle(void *) {
  if (gIrqAttached && ml_get_interrupts_enabled() && !ml_at_interrupt_context() && get_preemption_level() == 0) {
    assert_wait_timeout(event_t(&gIrqCount), THREAD_UNINT, 100, NSEC_PER_USEC);
    thread_block(THREAD_CONTINUE_NULL);
    native_gpu_idle_sleeps = native_gpu_idle_sleeps + 1;
  } else {
    IODelay(20);
    native_gpu_idle_spins = native_gpu_idle_spins + 1;
  }
  return 0;
}
volatile uint32_t gContextStep;
void naProgress(uint32_t step) {gContextStep = step;}
int naTimerRead(void *c, uint32_t reg, uint32_t *value) {return naRead(c, reg, value);}

NativeAdrenoRing *gRing;
NativeAdrenoCpInit *gCpInit;
NativeAdrenoSubmit *gSubmit;
NativeAdrenoTimer *gTimer;
NativeAdrenoRingIo gRio;
NativeAdrenoRingCache gCache{nullptr, cachePublish, cacheAcquire};
NativeAdrenoRingClock gClock{nullptr, naNow, naIdle};
NativeAdrenoTimeIo gTimerIo;
// Facts for the Vulkan driver: chip id 0x06010000 (bengal-gpu.dtsi), GMEM 128 KiB + 4 KiB at the 16 MiB window hw_init programs
// (Linux's 1 MiB window would hold NativeAdreno's SQE buffer at 0x100000), the a6xx UCHE trap base, the client VA window above
// the kernel buffers and below the 2 GiB aperture, UBWC 1.0 with highest bank bit 14, swizzle 7, macrotile mode 0.
const NativeAdrenoAbiDeviceInfo kDeviceInfo = {0x06010000u, 0x21000u, 0x1000000ull, 0x1fffffffff000ull, 0x20000000ull,
                                               0x60000000ull, 14, 7, 0, 1, NativeAdrenoDeviceClientIova, 0};

uint32_t smmuBank0(const Session &s) {
  // IDR1: PAGESIZE (bit 31) and NUMPAGENDXB (bits 30:28): the context banks start after 2^(NUMPAGENDXB + 1) pages.
  const uint32_t id1 = rd(s.smmu, 0x24);
  const uint32_t page = (id1 >> 31) ? 0x10000 : 0x1000;
  return page << (((id1 >> 28) & 7) + 1);
}

unsigned bootArg(const char *name, unsigned fallback) {
  unsigned value = fallback;
  PE_parse_boot_argn(name, &value, sizeof value);
  return value;
}

// The GPU's context: context bank 0 and stream-map slots 0 and 1 for SIDs 0 and 1 (bengal-gpu.dtsi gfx3d_user iommus
// <&kgsl_smmu 0 1>: SID 0 mask 1), a 2 GiB aperture of 4 KiB pages, the five kernel buffers. The GPU SMMU walks are not coherent
// (no dma-coherent on kgsl_smmu). tb128fu found that this firmware ignores fault writes to S2CR and turns a bypass write into
// fault, while translate writes take: NativeAdreno only writes translate entries.
bool naContext(Session &s) {
  IOLog("GuampAdreno: context: smmu slots 0/1 before smr %08x %08x s2cr %08x %08x\n", rd(s.smmu, 0x800), rd(s.smmu, 0x804),
        rd(s.smmu, 0xc00), rd(s.smmu, 0xc04));
  for (unsigned i = 0; i < 2; i = i + 1) {wr(s.smmu, 0x800 + 4 * i, 0); wr(s.smmu, 0xc00 + 4 * i, 0x20000);}
  native_adreno_smmu_coherent_walk = 0;
  native_adreno_wc_enabled = 1;
  // The A610 is not IO-coherent: render targets the GPU writes are cleaned when mapped and left alone afterwards.
  native_adreno_written_imports_owned = 1;
  // The BO pool, one contiguous block taken early. 4 GiB of RAM: 0 by default: a 256 MiB pool left no contiguous room for the context (e00002bd on guamp); guamp-adreno-pool=<MiB>.
  native_adreno_pool_mib = bootArg("guamp-adreno-pool", 0);
  const IOReturn ctx = NativeAdrenoGpuContextBringup(reinterpret_cast<volatile uint8_t *>(s.smmu.va), kSmmuBytes, 0, 2, 0, 0, 1, 1,
                                                     naProgress, &s.report);
  IOLog("GuampAdreno: context bringup %x (step %u, output bits %u)\n", ctx, gContextStep, s.report.output_bits);
  if (ctx) return false;
  // CB0 ACTLR without CPRE (arm_mmu500_reset, MMU-500 errata 841119/826419), then TLBIALL, TLBSYNC and TLBSTATUS of that bank.
  const uint32_t bank0 = smmuBank0(s);
  const uint32_t actlr = rd(s.smmu, bank0 + 0x4);
  wr(s.smmu, bank0 + 0x4, actlr & ~2u);
  wr(s.smmu, bank0 + 0x618, 0);
  wr(s.smmu, bank0 + 0x7f0, 0);
  const bool synced = poll(s.smmu, bank0 + 0x7f4, 1, 0, 1000);
  IOLog("GuampAdreno: context ready: cb0 at %x actlr %08x -> %08x, tlb sync %s, s2cr %08x %08x\n", bank0, actlr,
        rd(s.smmu, bank0 + 0x4), synced ? "ok" : "timeout", rd(s.smmu, 0xc00), rd(s.smmu, 0xc04));
  return true;
}

// Hardware clock gating of the A610 (kgsl adreno-gpulist.h a612_hwcg_regs, mainline a6xx_catalog.c a612_hwcg), then
// RBBM_CLOCK_CNTL 0xAAA8AA82. Without a GMU only these RBBM registers are written.
constexpr uint32_t kHwcg[][2] = {
    {0x00B0, 0x22222222}, {0x00B4, 0x02222220}, {0x00B8, 0x00000081}, {0x00BC, 0x0000F3CF}, {0x00C0, 0x22222222}, {0x00C4, 0x22222222},
    {0x00C8, 0x22222222}, {0x00CC, 0x00022222}, {0x00D0, 0x11111111}, {0x00D4, 0x11111111}, {0x00D8, 0x11111111}, {0x00DC, 0x00011111},
    {0x00E0, 0x77777777}, {0x00E4, 0x77777777}, {0x00E8, 0x77777777}, {0x00EC, 0x00077777}, {0x00F0, 0x22222222}, {0x00F4, 0x01202222},
    {0x00F8, 0x00002220}, {0x0100, 0x00040F00}, {0x0104, 0x05522022}, {0x0105, 0x00005555}, {0x0106, 0x00000011}, {0x0107, 0x00445044},
    {0x0108, 0x04222222}, {0x0111, 0x00002222}, {0x0114, 0x02222222}, {0x0117, 0x00000002}, {0x011B, 0x00002222}, {0x0109, 0x00004000},
    {0x0112, 0x00002222}, {0x0115, 0x00000200}, {0x011C, 0x00000000}, {0x010A, 0x00000000}, {0x0113, 0x00000000}, {0x0116, 0x04104004},
    {0x011D, 0x00000000}, {0x010B, 0x22222222}, {0x0110, 0x00000004}, {0x010F, 0x00000002}, {0x0533, 0x00000182}, {0x0044, 0x00000000},
    {0x0042, 0x00000000}, {0x0118, 0x00000222}, {0x0119, 0x00000111}, {0x011A, 0x00000555},
};
// CP register protection (mainline a6xx_catalog.c a630_protect, kgsl a630_protected_regs): A6XX_PROTECT_RDONLY / _NORDWR.
constexpr uint32_t rdonly(uint32_t reg, uint32_t len) {return ((len & 0x3FFF) << 18) | (reg & 0x3FFFF);}
constexpr uint32_t nordwr(uint32_t reg, uint32_t len) {return (1u << 31) | rdonly(reg, len);}
constexpr uint32_t kProtect[32] = {
    rdonly(0x00000, 0x04ff), rdonly(0x00501, 0x0005), rdonly(0x0050b, 0x02f4), nordwr(0x0050e, 0x0000), nordwr(0x00510, 0x0000),
    nordwr(0x00534, 0x0000), nordwr(0x00800, 0x0082), nordwr(0x008a0, 0x0008), nordwr(0x008ab, 0x0024), rdonly(0x008de, 0x00ae),
    nordwr(0x00900, 0x004d), nordwr(0x0098d, 0x0272), nordwr(0x00e00, 0x0001), nordwr(0x00e03, 0x000c), nordwr(0x03c00, 0x00c3),
    rdonly(0x03cc4, 0x1fff), nordwr(0x08630, 0x01cf), nordwr(0x08e00, 0x0000), nordwr(0x08e08, 0x0000), nordwr(0x08e50, 0x001f),
    nordwr(0x09624, 0x01db), nordwr(0x09e70, 0x0001), nordwr(0x09e78, 0x0187), nordwr(0x0a630, 0x01cf), nordwr(0x0ae02, 0x0000),
    nordwr(0x0ae50, 0x032f), nordwr(0x0b604, 0x0000), nordwr(0x0be02, 0x0001), nordwr(0x0be20, 0x17df), nordwr(0x0f000, 0x0bff),
    rdonly(0x0fc00, 0x1fff), nordwr(0x11c00, 0x0000),
};

// hw_init of mainline a6xx_gpu.c for the A610 (a610_family) with the kgsl a6xx_start values where they agree. Clock gating
// (guamp-adreno-hwcg=0 leaves the clocks on) and CP register protection (guamp-adreno-protect=0 leaves it off).
void naHwInit(Session &s) {
  gw(s, 0x3C45, 0); (void)gr(s, 0x3C45);                               // GBIF_HALT
  gw(s, 0x0016, 0); (void)gr(s, 0x0016);                               // RBBM_GBIF_HALT
  gw(s, 0xF803, 0);                                                    // RBBM_SECVID_TSB_CNTL
  gw64(s, 0xF800, 0); gw(s, 0xF802, 0);                                // TSB_TRUSTED_BASE, _SIZE: no trusted range
  static const uint32_t addrMode[] = {0x842, 0xC01, 0x8601, 0x8E05, 0x9E01, 0xBE05, 0xA601, 0x9601, 0xE00, 0xAE01, 0xB601, 0xF810};
  for (uint32_t reg : addrMode) gw(s, reg, 1);                         // *_ADDR_MODE_CNTL: 64-bit
  if (bootArg("guamp-adreno-hwcg", 1) && gr(s, 0x00AE) != 0xAAA8AA82) {
    for (const auto &reg : kHwcg) gw(s, reg[0], reg[1]);
    gw(s, 0x00AE, 0xAAA8AA82);                                         // RBBM_CLOCK_CNTL
  }
  for (uint32_t reg = 0x3C03; reg <= 0x3C06; reg = reg + 1) gw(s, reg, 0x00071620);   // GBIF_QSB_SIDE0..3
  gw(s, 0x0011, 0x3);                                                  // RBBM_GBIF_CLIENT_QOS_CNTL
  gw(s, 0x050B, 0xffffffff);                                           // RBBM_PERFCTR_GPU_BUSY_MASKED
  gw64(s, 0xE05, 0x1fffffffff000ull + 0xfc0);                          // UCHE_WRITE_RANGE_MAX
  gw64(s, 0xE09, 0x1fffffffff000ull);                                  // UCHE_TRAP_BASE
  gw64(s, 0xE07, 0x1fffffffff000ull);                                  // UCHE_WRITE_THRU_BASE
  gw64(s, 0xE0B, kDeviceInfo.gmem_base);                               // UCHE_GMEM_RANGE_MIN
  gw64(s, 0xE0D, kDeviceInfo.gmem_base + kDeviceInfo.gmem_bytes - 1);  // UCHE_GMEM_RANGE_MAX
  gw(s, 0xE18, 0x804); gw(s, 0xE17, 0x4);                              // UCHE_FILTER_CNTL, UCHE_CACHE_WAYS
  gw(s, 0x8C2, 0x00800060); gw(s, 0x8C1, 0x40201B16);                  // CP_ROQ_THRESHOLDS_2/_1
  gw(s, 0x8C3, 48); gw(s, 0x90E, 47);                                  // CP_MEM_POOL_SIZE, CP_MEM_POOL_DBG_ADDR (A610)
  gw(s, 0x9E00, 0x00080000);                                           // PC_DBG_ECO_CNTL: prim_fifo_threshold
  gw(s, 0x98D, 1);                                                     // CP_AHB_CNTL: error response
  gw(s, 0x500, 1);                                                     // RBBM_PERFCTR_CNTL
  gw(s, 0x8D0, 0);                                                     // CP_PERFCTR_CP_SEL(0): PERF_CP_ALWAYS_COUNT
  gw(s, 0x8E08, 0xb); gw(s, 0xB604, 0xb); gw(s, 0xAE02, 0xb);          // RB, TPL1, SP _NC_MODE_CNTL (UBWC 1.0, hbb 14)
  gw(s, 0xE01, (1u << 23) | (1u << 21));                               // UCHE_MODE_CNTL
  gw(s, 0x534, 0);                                                     // RBBM_NC_MODE_CNTL: macrotile mode 0
  gw(s, 0x001F, (1u << 30) | 0x3FFFF);                                 // RBBM_INTERFACE_HANG_INT_CNTL (A610)
  gw(s, 0xE19, (1u << 7) | 1u);                                        // UCHE_CLIENT_PF
  if (bootArg("guamp-adreno-protect", 1)) {                            // a6xx_set_cp_protect
    gw(s, 0x84F, (1u << 0) | (1u << 1) | (1u << 3));                   // CP_PROTECT_CNTL: enable, fault, last span unbounded
    for (uint32_t i = 0; i < 32; i = i + 1) gw(s, 0x850 + i, kProtect[i]);
  }
  gw(s, 0x038, NativeAdrenoIrq::maskA6xx());                           // RBBM_INT_0_MASK: A6XX_INT_MASK
}

void cpState(const Session &s, const char *when) {
  IOLog("GuampAdreno: %s: rbbm status %08x rptr %u wptr %u cp hw_fault %08x int %08x, rbbm int %08x, cb0 fsr %08x\n", when,
        gr(s, 0x210), gr(s, 0x806), gr(s, 0x807), gr(s, 0x821), gr(s, 0x823), gr(s, 0x201), rd(s.smmu, smmuBank0(s) + 0x58));
}

// SQE, hw_init, ring 0 (A6XX forms), CP_ME_INIT, then a CP_MEM_WRITE into the scratch page that the CPU reads back.
int naCp(Session &s, const GuampAdrenoStart *in) {
  if (NativeAdrenoGpuContextBuffers(s.nb)) return GuampAdrenoBuffers;
  for (unsigned i = 0; i < 5; i = i + 1) {
    IODeviceMemory *m = IODeviceMemory::withRange(s.nb[i].physical, s.nb[i].bytes);
    if (m) {
      s.nmap[i] = OSSharedPtr<IOMemoryMap>(m->map(kIOMapAnywhere | kIOMapWriteCombineCache), OSNoRetain);
      m->release();
    }
    if (!s.nmap[i]) return GuampAdrenoBuffers;
    s.ncpu[i] = reinterpret_cast<volatile uint8_t *>(s.nmap[i]->getVirtualAddress());
  }
  // The SQE microcode, already without the file's first dword (adreno_fw_create_bo; GpuService.inc loadSqe).
  const uint32_t sqeBytes = in->sqeBytes;
  if (sqeBytes < 4 || (sqeBytes & 3) || sqeBytes > s.nb[0].bytes) return GuampAdrenoSqe;
  for (uint32_t i = 0; i < sqeBytes; i = i + 1) s.ncpu[0][i] = in->sqe[i];
  __asm__ volatile("dsb sy" ::: "memory");
  for (uint32_t i = 0; i < sqeBytes; i = i + 1)
    if (s.ncpu[0][i] != in->sqe[i]) {
      IOLog("GuampAdreno: sqe copy differs at byte %u\n", i);
      return GuampAdrenoSqe;
    }
  IOLog("GuampAdreno: sqe %u bytes at iova %llx, hw_init\n", sqeBytes, static_cast<unsigned long long>(s.nb[0].iova));
  naHwInit(s);
  IOLog("GuampAdreno: hw_init done: rbbm clock cntl %08x cp protect cntl %08x\n", gr(s, 0x00AE), gr(s, 0x84F));
  gw64(s, 0x830, s.nb[0].iova);                                        // CP_SQE_INSTR_BASE, before CP_SQE_CNTL
  gRio = NativeAdrenoRingIo{&s, naWrite, naRead, naBarrier, nullptr, nullptr};   // no GMU, no AHB fence to check
  gRing = new NativeAdrenoRing;
  gCpInit = new NativeAdrenoCpInit;
  if (!gRing || !gCpInit || gRing->setGeneration(6)) return GuampAdrenoRing;
  const IOReturn att = gRing->attach(&gRio, &gCache, const_cast<uint8_t *>(s.ncpu[1]), s.nb[1].bytes, s.nb[1].iova,
                                     const_cast<uint8_t *>(s.ncpu[2]), s.nb[2].bytes, s.nb[2].iova);
  IOLog("GuampAdreno: ring attach %x (ring iova %llx, %llu bytes)\n", att, static_cast<unsigned long long>(s.nb[1].iova),
        static_cast<unsigned long long>(s.nb[1].bytes));
  if (att) return GuampAdrenoRing;
  gw(s, 0x808, 1);                                                     // CP_SQE_CNTL: start the microcode
  NativeAdrenoCpInitConfig config{s.nb[0].iova, 0, 1};
  config.generation = 6;
  uint32_t fence = 0, seen = 0;
  const IOReturn init = gCpInit->run(&gRio, gRing, &config, &fence);
  const unsigned waited = init ? 0 : unsigned(gRing->wait(fence, &gClock, 200000, &seen));
  IOLog("GuampAdreno: CP_ME_INIT run %x fence %u wait %u (seen %u)\n", init, fence, waited, seen);
  if (init || waited) {cpState(s, "cp init stopped"); return GuampAdrenoCpInit;}
  volatile uint32_t *scratch = reinterpret_cast<volatile uint32_t *>(s.ncpu[4]);
  scratch[0] = 0;
  __asm__ volatile("dsb sy" ::: "memory");
  uint32_t words[4] = {};
  if (AdrenoPM4Type7(0x3d, 3, &words[0]) != AdrenoPM4Success) return GuampAdrenoMemWrite;   // CP_MEM_WRITE
  words[1] = uint32_t(s.nb[4].iova); words[2] = uint32_t(s.nb[4].iova >> 32); words[3] = 0xA6100C0D;
  if (gRing->submit(words, 4, &fence)) return GuampAdrenoMemWrite;
  const unsigned memWait = unsigned(gRing->wait(fence, &gClock, 200000, &seen));
  IOLog("GuampAdreno: CP_MEM_WRITE fence %u wait %u, scratch %08x (want a6100c0d)\n", fence, memWait, scratch[0]);
  cpState(s, "after mem write");
  return memWait || scratch[0] != 0xA6100C0D ? GuampAdrenoMemWrite : GuampAdrenoOk;
}

// Secure mode without the zap shader: mainline a6xx_hw_init writes RBBM_SECVID_TRUST_CNTL (0xF400) = 0 when no zap shader
// exists. Whether the normal world may change it here is exactly what this line records; the stages go on either way, and a
// GPU left in secure mode shows up as faults of the user submissions.
// guamp: reading 0xF400 stalled the core on the phone (TrustZone-protected here), so it is skipped unless guamp-adreno-secvid=1.
void naSecureFallback(Session &s) {
  // Write only: kgsl never reads it back (adreno.c), and a read stalls the core here. guamp-adreno-secvid=1 writes 0.
  if (bootArg("guamp-adreno-secvid", 0) != 1) {
    IOLog("GuampAdreno: secvid trust cntl not touched (protected on guamp; guamp-adreno-secvid=1 writes 0 without reading)\n");
    return;
  }
  gw(s, 0xF400, 0);
  IOLog("GuampAdreno: secvid trust cntl written 0 (not read back: a read stalls the core)\n");
}

// The GPU SMMU's global fault state, read only (kgsl_smmu GR0 at 0x59a0000: GFSR 0x48, GFSYNR0-2 0x50-0x58, the non-secure
// copies; naContext already reads GR0). A transaction of a stream no SMR matches (a secure SID 2 access of a GPU left in secure
// mode) shows up as GFSR.USF (bit 1) with its stream id in GFSYNR1. No GPU register is read here.
void naGfsr(const Session &s, const char *when) {
  IOLog("GuampAdreno: zap smmu %s: gfsr %08x gfsynr0 %08x gfsynr1 %08x gfsynr2 %08x, cb0 fsr %08x\n", when, rd(s.smmu, 0x48),
        rd(s.smmu, 0x50), rd(s.smmu, 0x54), rd(s.smmu, 0x58), rd(s.smmu, smmuBank0(s) + 0x58));
}

// CP_SET_SECURE_MODE 0 on ring 0 (mainline a6xx_hw_init once a6xx_zap_shader_init succeeded), then its fence. True when the CP
// retired it.
bool gZapUp;
bool naSecureMode(Session &s, const char *when) {
  uint32_t words[2] = {}, fence = 0, seen = 0;
  if (AdrenoPM4Type7(0x66, 1, &words[0]) != AdrenoPM4Success) return false;   // CP_SET_SECURE_MODE
  words[1] = 0;
  const IOReturn queued = gRing->submit(words, 2, &fence);
  const unsigned waited = queued ? 0 : unsigned(gRing->wait(fence, &gClock, 200000, &seen));
  IOLog("GuampAdreno: zap CP_SET_SECURE_MODE 0 (%s): submit %x fence %u wait %u (seen %u)\n", when, queued, fence, waited, seen);
  cpState(s, "zap after secure mode 0");
  return !queued && !waited;
}

// guamp-adreno-zap=1|2 (GuampAdreno.h): true when the option is set, so the caller never writes RBBM_SECVID_TRUST_CNTL; *failed
// when the CP did not retire CP_SET_SECURE_MODE. A refused zap start leaves the GPU secure, exactly as without the option.
bool naZap(Session &s, const GuampAdrenoStart *in, bool *failed) {
  *failed = false;
  const unsigned mode = bootArg("guamp-adreno-zap", 0);
  if (mode != 1 && mode != 2) return false;
  if (!in->zap) {
    IOLog("GuampAdreno: zap mode %u: no zap loader from the caller; GPU left secure, secvid not touched\n", mode);
    return true;
  }
  naGfsr(s, "before zap");
  IOLog("GuampAdreno: zap mode %u: a610_zap through TrustZone PAS (id 13)\n", mode);
  const int loaded = in->zap(in->zapContext, mode);
  IOLog("GuampAdreno: zap loader %d (%s)\n", loaded,
        loaded == 0 ? "authenticated start accepted" : loaded == 1 ? "queries only" : "refused: GPU left secure, secvid not touched");
  if (loaded) return true;
  gZapUp = naSecureMode(s, "bring-up");
  *failed = !gZapUp;
  naGfsr(s, "after secure mode 0");
  return true;
}

// guamp-adreno-zap: one more GFSR snapshot once Metal clients have drawn (guamp-adreno-gfsr-delay=<s>, default 60, 0 = none).
void gfsrLater(void *, wait_result_t) {
  IOSleep(bootArg("guamp-adreno-gfsr-delay", 60) * 1000);
  naGfsr(gSession, "later");
}
void gfsrArm() {
  const unsigned delay = bootArg("guamp-adreno-gfsr-delay", 60);
  if (!delay || delay > 3600) return;
  thread_t t = nullptr;
  if (kernel_thread_start(gfsrLater, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}

// The GPU interrupt line, then one CP_NOP fence that must raise it.
int naIrq(Session &s) {
  if (!&armcpu_spi_attach) {
    IOLog("GuampAdreno: irq: armcpu_spi_attach missing, waits keep polling\n");
    return GuampAdrenoOk;
  }
  uint32_t readback = 0;
  uint64_t router = ~0ull;
  const int attached = armcpu_spi_attach(kGpuIntid, false, gpuIrq, &s, &readback, &router);
  gIrqAttached = attached == 0;
  IOSleep(20);
  const uint32_t before = gIrqCount;
  uint32_t nop = 0, fence = 0, seen = 0;
  const bool ok = AdrenoPM4Type7(0x10, 0, &nop) == AdrenoPM4Success && !gRing->submit(&nop, 1, &fence) &&
                  !gRing->wait(fence, &gClock, 200000, &seen);
  IOSleep(20);
  IOLog("GuampAdreno: irq intid %u attach %d readback %08x router %llx; nop fence %s, interrupts %u -> %u seen %08x\n", kGpuIntid,
        attached, readback, static_cast<unsigned long long>(router), ok ? "ok" : "failed", before, gIrqCount, gIrqSeen);
  return attached ? GuampAdrenoIrq : GuampAdrenoOk;
}

// The submission object, the CP always-on counter as the GPU clock, and the device published for user clients.
int naPublish(Session &s) {
  NativeAdrenoVm *vm = NativeAdrenoGpuContextVm();
  gSubmit = new NativeAdrenoSubmit;
  gTimer = new NativeAdrenoTimer;
  if (!vm || !gSubmit || !gTimer) return GuampAdrenoPublish;
  const IOReturn sub = gSubmit->attach(vm, gRing);
  gTimerIo = NativeAdrenoTimeIo{&s, naTimerRead};
  const IOReturn tim = gTimer->attach(&gTimerIo, NativeAdrenoTimeCp);
  const IOReturn pub = sub ? sub : native_adreno_publish(vm, gSubmit, gRing, &gClock, tim ? nullptr : gTimer, &kDeviceInfo, 31,
                                                         s.report.output_bits);
  IOLog("GuampAdreno: publish: submit attach %x timer %x publish %x\n", sub, tim, pub);
  return sub || pub ? GuampAdrenoPublish : GuampAdrenoOk;
}

#include "GuampAdrenoRecover.inc"

// The user-client self test, ported from UniXNU-tb128fu package/GpuProbe.cpp (naSelfTest / ucRound, BSD-2-Clause-Patent): a kernel
// connection of the plugin's open type, Query and GetDeviceInfo, two 4 KiB BOs mapped in the client window and in the kernel, an IB
// of three CP_MEM_WRITEs submitted and waited for, the target read back, then clientClose. Two rounds: the second proves the client
// slot and the IOVAs recycle. No import test (kernel_task imports are refused; tb128fu-033). guamp-adreno-selftest=0 skips it.
struct UcStats { uint32_t passed, steps, firstError, firstStep, memStatus; };

IOReturn ucCall(IOUserClient *uc, uint32_t method, const void *in, void *out) {
  IOExternalMethodArguments a{};
  a.version = kIOExternalMethodArgumentsCurrentVersion;
  a.selector = method;
  const size_t is = NativeAdrenoUserClient::inputSize(method), os = NativeAdrenoUserClient::outputSize(method);
  a.structureInput = is ? in : nullptr;
  a.structureInputSize = uint32_t(is);
  a.structureOutput = os ? out : nullptr;
  a.structureOutputSize = uint32_t(os);
  return uc->externalMethod(method, &a, nullptr, nullptr, nullptr);
}

// Returns the matching dwords of the target (4 = the three writes landed and the fourth word is untouched) or -1.
int ucRound(IOService *svc, UcStats &st) {
  auto step = [&](IOReturn status) {
    const uint32_t at = st.steps;
    st.steps = st.steps + 1;
    if (!status) st.passed = st.passed + 1;
    else if (!st.firstError) {st.firstError = uint32_t(status); st.firstStep = at;}
    return !status;
  };
  constexpr UInt32 kPluginOpenType = 0x4147;   // NativeAdrenoPublish.cpp NativeAdrenoPluginOpenType; type 0 is refused
  IOUserClient *uc = nullptr;
  if (!step(svc->newUserClient(kernel_task, nullptr, kPluginOpenType, nullptr, &uc)) || !uc) return -1;
  IOUserClient *second = nullptr;
  step(svc->newUserClient(kernel_task, nullptr, kPluginOpenType, nullptr, &second) == kIOReturnBusy ? kIOReturnSuccess
                                                                                                    : kIOReturnError);
  if (second) second->release();
  NativeAdrenoAbiParams params{};
  step(ucCall(uc, NativeAdrenoMethodQuery, nullptr, &params));
  NativeAdrenoAbiDeviceInfo info{};
  step(ucCall(uc, NativeAdrenoMethodGetDeviceInfo, nullptr, &info));
  step(info.chip_id == kDeviceInfo.chip_id && info.gmem_bytes == kDeviceInfo.gmem_bytes ? kIOReturnSuccess : kIOReturnError);
  NativeAdrenoAbiBoAlloc alloc{4096, 12, 0};
  NativeAdrenoAbiBoAllocReply ib{}, tgt{};
  int matched = -1;
  IOMemoryDescriptor *desc[2] = {};
  IOMemoryMap *maps[2] = {};
  if (step(ucCall(uc, NativeAdrenoMethodBoAlloc, &alloc, &ib)) && step(ucCall(uc, NativeAdrenoMethodBoAlloc, &alloc, &tgt))) {
    const uint64_t ibIova = 0x20004000, tgtIova = 0x20005000;   // inside the client window of kDeviceInfo
    NativeAdrenoAbiBoMap mapIb{ib.handle, ibIova, 1, 0, 2, 0}, mapTgt{tgt.handle, tgtIova, 3, 0, 2, 0};
    NativeAdrenoAbiBoMapReply r{};
    NativeAdrenoAbiBoCpuMap cmIb{ib.handle}, cmTgt{tgt.handle};
    NativeAdrenoAbiBoCpuMapReply crIb{}, crTgt{};
    if (step(ucCall(uc, NativeAdrenoMethodBoMap, &mapIb, &r)) && step(ucCall(uc, NativeAdrenoMethodBoMap, &mapTgt, &r)) &&
        step(ucCall(uc, NativeAdrenoMethodBoCpuMap, &cmIb, &crIb)) && step(ucCall(uc, NativeAdrenoMethodBoCpuMap, &cmTgt, &crTgt))) {
      IOOptionBits opt[2] = {};
      const uint32_t tokens[2] = {crIb.token, crTgt.token};
      bool mapped = true;
      for (int i = 0; i < 2; i = i + 1) {
        const IOReturn status = uc->clientMemoryForType(tokens[i], &opt[i], &desc[i]);
        if (!st.memStatus) st.memStatus = uint32_t(status);
        if (status || !desc[i]) {mapped = false; continue;}
        maps[i] = desc[i]->createMappingInTask(kernel_task, 0, opt[i]);
        if (!maps[i]) {mapped = false; if (!st.memStatus) st.memStatus = 0xbad;}
      }
      IOMemoryDescriptor *none = nullptr;   // type 0 and a stale token must be refused
      IOOptionBits o = 0;
      step(uc->clientMemoryForType(0, &o, &none) != kIOReturnSuccess && !none ? kIOReturnSuccess : kIOReturnError);
      step(mapped ? kIOReturnSuccess : kIOReturnNoMemory);
      if (mapped) {
        volatile uint32_t *iw = reinterpret_cast<volatile uint32_t *>(maps[0]->getVirtualAddress());
        volatile uint32_t *tw = reinterpret_cast<volatile uint32_t *>(maps[1]->getVirtualAddress());
        for (unsigned i = 0; i < 1024; i = i + 1) tw[i] = 0xdeadbeef;
        const uint32_t values[3] = {0x0badc0deu, 0xfeed0001u, 0x600d0003u};
        uint32_t words[12];
        bool built = true;
        for (unsigned i = 0; i < 3; i = i + 1) {
          built = built && AdrenoPM4Type7(0x3d, 3, &words[i * 4]) == AdrenoPM4Success;   // CP_MEM_WRITE
          words[i * 4 + 1] = uint32_t(tgtIova + i * 4);
          words[i * 4 + 2] = uint32_t(tgtIova >> 32);
          words[i * 4 + 3] = values[i];
        }
        for (unsigned i = 0; i < 12; i = i + 1) iw[i] = words[i];
        __asm__ volatile("dsb sy" ::: "memory");
        NativeAdrenoAbiSubmit submit{};
        submit.buffer_count = 1;
        submit.buffers[0] = {ib.handle, ibIova, sizeof words};
        NativeAdrenoAbiSubmitReply sr{};
        if (step(built ? kIOReturnSuccess : kIOReturnInternalError) && step(ucCall(uc, NativeAdrenoMethodSubmit, &submit, &sr))) {
          NativeAdrenoAbiWaitFence wait{sr.fence, 200000000ull};
          if (step(ucCall(uc, NativeAdrenoMethodWaitFence, &wait, nullptr))) {
            __asm__ volatile("dsb sy" ::: "memory");
            matched = (tw[0] == values[0]) + (tw[1] == values[1]) + (tw[2] == values[2]) + (tw[3] == 0xdeadbeef);
          }
        }
        NativeAdrenoAbiTimestamp ts{};
        step(ucCall(uc, NativeAdrenoMethodGetTimestamp, nullptr, &ts));
      }
    }
  }
  for (auto *m : maps) if (m) m->release();
  for (auto *d : desc) if (d) d->release();
  step(uc->clientClose());
  uc->release();
  return matched;
}

// One line per round; false when a round did not see its three writes. Not fatal for the stages.
bool naSelfTest() {
  if (bootArg("guamp-adreno-selftest", 1) == 0) return true;
  IOService *svc = native_adreno_published_service();
  if (!svc) {
    IOLog("GuampAdreno: self test: no published service\n");
    return false;
  }
  bool ok = true;
  for (unsigned round = 0; round < 2; round = round + 1) {
    UcStats st{};
    const int matched = ucRound(svc, st);
    IOLog("GuampAdreno: self test round %u: user IB writes matched %d/4, steps %u/%u, first error %x at step %u, "
          "client memory %x\n", round, matched, st.passed, st.steps, st.firstError, st.firstStep, st.memStatus);
    ok = ok && matched == 4;
  }
  return ok;
}

// The staged AGXAccelerator for the original Metal discovery (tb128fu naMetal). Its static initializer runs only after the
// original IOGPUFamily kext has loaded, because the parent metaclass does not exist before that.
int naAccelerator() {
  if (!native_adreno_accelerator_instance_size()) {   // the weak stand-in answers 0
    IOLog("GuampAdreno: accelerator: staged class not linked (build-native-adreno-accelerator.py did not run)\n");
    return GuampAdrenoAccelerator;
  }
  const kern_return_t kext = OSKextLoadKextWithIdentifier("com.apple.iokit.IOGPUFamily");
  const OSSymbol *name = OSSymbol::withCString("IOAcceleratorES");
  const OSMetaClass *parent = name ? OSMetaClass::getMetaClassWithName(name) : nullptr;
  OSSafeReleaseNULL(name);
  IOLog("GuampAdreno: accelerator: IOGPUFamily load %x, IOAcceleratorES %s size %u\n", kext, parent ? "found" : "missing",
        parent ? parent->getClassSize() : 0);
  if (kext || !parent || parent->getClassSize() != 136) return GuampAdrenoAccelerator;
  void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
  if (!handle) return GuampAdrenoAccelerator;
  native_adreno_accelerator_static_init();
  const IOReturn loaded = OSMetaClass::postModLoad(handle);
  if (loaded || native_adreno_accelerator_instance_size() != 136) {
    IOLog("GuampAdreno: accelerator: metaclass %x size %u\n", loaded, native_adreno_accelerator_instance_size());
    return GuampAdrenoAccelerator;
  }
  IOService *accelerator = native_adreno_accelerator_allocate();
  if (!accelerator) return GuampAdrenoAccelerator;
  if (!accelerator->init()) {accelerator->release(); return GuampAdrenoAccelerator;}
  // The keys the original _MTLRegisterDevices helper reads (the plugin name is ours, no Apple device identity is claimed), and the
  // processes whose opens the kernel grants past the sandbox (patches/xnu/zzw-native-adreno-sandbox-grant.patch). The plugin
  // itself is not installed yet (no GPU addon on guamp): Metal discovery will fail to load it and keep the CPU renderer.
  accelerator->setProperty("MetalPluginName", "NativeAdrenoMetal");
  accelerator->setProperty("NativeSandboxGrantProcess", "backboardd,iconservicesagent,Preferences");
  accelerator->setProperty("MetalPluginClassName", "NativeAdrenoMetalDevice");
  accelerator->setProperty("NativeTurnipDebug", "nocb,noconform,sysmem");
  accelerator->setProperty("NativeMetalWithhold", "apple-families");
  if (!accelerator->attach(IOService::getPlatform())) {accelerator->release(); return GuampAdrenoAccelerator;}
  accelerator->registerService();
  IOLog("GuampAdreno: accelerator AGXAccelerator registered (IOAcceleratorES)\n");
  return GuampAdrenoOk;
}
}  // namespace

// guamp-copy-gpu-wait (scripts/guamp-canvas-variant.py): the deferred CPU copy of a swapped surface first waits until the CP has
// retired every submission handed out so far (the render server swaps after waitUntilScheduled, before its frame is done). Reads
// only the ring object's newest fence number and the CP-written timestamp of the fence page (+4, write-combined kernel memory, no
// GPU register); never retires or accounts a fence and never runs the hang detector, so it needs no GPU lock. Sleeps until the GPU
// interrupt or 200 us per round; with interrupts off or preemption disabled it spins 20 us per round.
extern "C" int guamp_adreno_wait_submitted(uint32_t timeout_us, uint32_t *waited_us) {
  if (waited_us) *waited_us = 0;
  NativeAdrenoRing *ring = gRing;
  const volatile uint8_t *page = gSession.ncpu[2];
  if (!ring || !page || gSession.nb[2].bytes < 8 || ring->poisoned() || gGpuParked) return -1;
  const uint32_t target = ring->lastSubmitted();
  if (!target) return 0;
  const uint64_t start = naNow(nullptr);
  for (;;) {
    __asm__ volatile("dsb sy" ::: "memory");
    const uint32_t done = *reinterpret_cast<const volatile uint32_t *>(page + 4);
    const uint64_t spent = naNow(nullptr) - start;
    if (waited_us) *waited_us = spent > 0xffffffffull ? 0xffffffffu : uint32_t(spent);
    if (int32_t(done - target) >= 0) return 0;
    if (spent >= timeout_us || ring->poisoned() || gGpuParked) return 1;
    if (gIrqAttached && ml_get_interrupts_enabled() && !ml_at_interrupt_context() && get_preemption_level() == 0) {
      assert_wait_timeout(event_t(&gIrqCount), THREAD_UNINT, 200, NSEC_PER_USEC);
      thread_block(THREAD_CONTINUE_NULL);
    } else {
      IODelay(20);
    }
  }
}

extern "C" int guamp_adreno_start(const GuampAdrenoStart *in) {
  static bool started = false;
  if (!in || !in->gpu || in->gpuBytes < 0x90000 || !in->sqe || !in->level || started) return GuampAdrenoBadArgs;
  started = true;
  Session &s = gSession;
  s.gpu = in->gpu;
  IOLog("GuampAdreno: start level %u: rbbm status %08x, sqe %u bytes\n", in->level, gr(s, 0x210), in->sqeBytes);
  int result = GuampAdrenoOk;
  bool tested = false;
  gRecEnabled = bootArg("guamp-gpu-recover", 0) == 1;   // before the ring: its write index is followed from attach on
  do {
    if (!mapRegion(kSmmu, kSmmuBytes, s.smmu)) {result = GuampAdrenoSmmuMap; break;}
    if (!naContext(s)) {result = GuampAdrenoContext; break;}
    if ((result = naCp(s, in))) break;
    // guamp-adreno-zap: the zap shader and CP_SET_SECURE_MODE 0 replace the SECVID write (never written with the option set).
    bool zapFailed = false;
    const bool zap = naZap(s, in, &zapFailed);
    if (zapFailed) {result = GuampAdrenoZap; break;}
    if (!zap) naSecureFallback(s);
    if ((result = naIrq(s))) break;
    if ((result = naPublish(s))) break;
    tested = naSelfTest();   // user IBs: the first submissions that do not come from this probe's ring
    IOLog("GuampAdreno: self test %s\n", tested ? "ok" : "FAILED (the stages go on)");
    if (zap) {naGfsr(s, "after self test"); gfsrArm();}
    recoverArm(s);   // guamp-gpu-recover=1 only (GuampAdrenoRecover.inc)
    // The staged AGXAccelerator (IOAcceleratorES) makes every Metal client of the system (CoreImage in Settings > Screen Time,
    // QuartzCore...) load the plugin; until the plugin is proven on guamp it is registered only with guamp-adreno-accel=1. The
    // user clients of level 1 (NativeAdrenoGpuPublished, used by adrenoprobed and the self test) stay published either way.
    if (in->level >= 2 && bootArg("guamp-adreno-accel", 0) == 1) result = naAccelerator();
    else if (in->level >= 2) IOLog("GuampAdreno: accelerator not registered (guamp-adreno-accel=1 to register IOAcceleratorES)\n");
  } while (false);
  IOLog("GuampAdreno: RESULT %d (level %u, context step %u, irqs %u, idle sleeps %llu spins %llu)\n", result, in->level,
        gContextStep, gIrqCount, native_gpu_idle_sleeps, native_gpu_idle_spins);
  // guamp-ca-metal (GuampCaMetal.inc): the render server chose its renderer long before this point; with the gate open it is
  // restarted once on the device just published, or the plugin's marker is parked when the device is not usable.
  if (in->level >= 2) guamp_ca_metal_gpu_ready(result == GuampAdrenoOk && tested);
  return result;
}
