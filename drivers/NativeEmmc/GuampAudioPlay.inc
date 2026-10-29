// SPDX-License-Identifier: BSD-2-Clause-Patent
// Kernel glue of the ADSP playback chain (guamp-adsp-ipc=5|6, apr::Playback in GuampAsm.inc, driven by the guamp-adsp
// thread in GuampAdspIpc.inc): the shared PCM buffer the ADSP reads and the /dev/guamp-audio ring hooks. Registered by
// audioplay::start() before adsp::start(). Included by NativeEmmc.cpp after GuampAudioDev.inc.
// Shared buffer (ASM_CMD_SHARED_MEM_MAP_REGIONS, pool SHMEM8_4K, property 0 = physical/IOVA): 8 slots x 4 KiB, XNU
// owned (IOBufferMemoryDescriptor, physically contiguous, below 4 GiB, mapped uncached: the ADSP does not snoop the
// CPU caches), allocated once per boot and never freed (the ADSP may still hold the mapping after an err-fatal).
// The ADSP reaches DDR through the apps SMMU 0x0C600000, stream 0x1c1 (guamp DT msm-audio-ion iommus <&apps_smmu
// 0x1c1 0x0>; Linux maps an IOVA in 0x10000000..0x1fffffff and passes msw = 1 = SID bits). XNU programs no context
// bank for it, so the global registers are READ (sCR0, IDR0, every SMR/S2CR; nothing written) and
//   guamp-adsp-pcm-mem=0 (default) uses the physical address with msw 1 if stream 0x1c1 bypasses translation, else
//                        the context bank below (the boot p2 probe: "no SMR, unmatched streams fault")
//   guamp-adsp-pcm-mem=3 always the context bank
// Context bank (as Linux msm_audio_ion + arm-smmu for the msm-audio-ion node: iommus <&apps_smmu 0x1c1 0x0>,
// qcom,iommu-dma-addr-pool 0x10000000.., smmu_sid_bits = (0x1c1 & 0xf) << 32): GuampSmmu.inc AdspAudioSmmu, the WLAN
// SmmuContext code with SID 0x1c1 mask 0 and IOVA 0x10000000..0x10400000 (S1, 4K tables Normal-NC, a free SMR slot
// and bank, global lock shared with Wi-Fi). Playback buffer at IOVA 0x10000000, capture at 0x10100000; the ADSP gets
// lsw = IOVA, msw = 1. Set up once at the first buffer, never torn down (the ADSP may keep the mapping).
// Previous text, kept for the physical modes:
//   (mode 0 physical case) uses the physical address with msw 1 only if stream 0x1c1 bypasses translation
//                        (SMMU disabled, S2CR bypass, or no SMR with unmatched streams bypassing); otherwise no PCM
//   guamp-adsp-pcm-mem=1 physical address, msw 0 (forced, whatever the probe says)
//   guamp-adsp-pcm-mem=2 physical address, msw 1 (forced)
// Faults of the bank (FSR/FAR/FSYNR0) are logged by report() at the PCM verdict and once a second while streaming.
// The capture side (GuampMicDev.inc, ASM session 2, its own SHARED_MEM_MAP handle) gets a SECOND buffer of the same
// kind through guamp_audio_shm_get() (GuampAudioMicHw.h): same SMMU decision and msw, separate memory, never freed.
#ifndef GUAMP_AUDIO_PLAY_INC
#define GUAMP_AUDIO_PLAY_INC
#include "GuampAsm.inc"
#include "GuampAudioDev.inc"
#include "GuampSmmu.inc"
#include "GuampAudioMicHw.h"
#if __has_include("GuampAudioHwExt.h")   // the HW side (codex/audio-hw-20261029): its RX macro asks for LPASS clocks
#include "GuampAudioHwExt.h"
#define GUAMP_AUDIO_PLAY_CLOCK_HOOK 1
#endif

namespace audioplay {

constexpr uint64_t kSmmuPhys = 0x0c600000;
constexpr uint32_t kStream = 0x1c1, kBytes = apr::PcmPath::kSlots * apr::PcmPath::kSlotStride;
IOBufferMemoryDescriptor *gBuf = nullptr;      // playback (apr::Playback, session 1)
IOBufferMemoryDescriptor *gCapBuf = nullptr;   // capture (apr::MicPath, session 2)
uint32_t gCapBytes = 0;
unsigned gMemMode = 0;
constexpr uint64_t kIovaPlay = 0x10000000ull, kIovaCapture = 0x10100000ull;
guampwifi::AdspAudioSmmu gSmmu;
IOBufferMemoryDescriptor *gTables = nullptr;
volatile uint8_t *gSmmuRegs = nullptr;
uint8_t *gTablesCpu = nullptr;
bool gSmmuTried = false, gPlayMapped = false, gCapMapped = false;
uint32_t gLastFsr = 0;

uint32_t regRd(void *, uint32_t at) { return *reinterpret_cast<volatile uint32_t *>(gSmmuRegs + at); }
void regWr(void *, uint32_t at, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(gSmmuRegs + at) = v; }
uint32_t tblRd(void *, uint32_t at) { return *reinterpret_cast<volatile uint32_t *>(gTablesCpu + at); }
void tblWr(void *, uint32_t at, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(gTablesCpu + at) = v; }
void fence(void *) { __asm__ volatile("dsb sy" ::: "memory"); }
void delayUs(void *, uint32_t us) { IODelay(us); }

// The context bank for 0x1c1, once per boot
bool smmuUp() {
  if (gSmmuTried) return gSmmu.active;
  gSmmuTried = true;
  using S = guampwifi::AdspAudioSmmu;
  gSmmuRegs = mapPhysical(kSmmuPhys, 0x80000);
  gTables = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
      kernel_task, static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous) |
                       static_cast<IOOptionBits>(kIOMapInhibitCache), S::TableBytes, 0xFFFFF000ULL);
  gTablesCpu = gTables ? static_cast<uint8_t *>(gTables->getBytesNoCopy()) : nullptr;
  const uint64_t tables = gTables ? gTables->getPhysicalSegment(0, nullptr, kIOMemoryMapperNone) : 0;
  if (!gSmmuRegs || !gTablesCpu || !tables || (tables & 0xfff)) {
    IOLog("GuampAudioPlay: SMMU context: mapping/table allocation failed (regs %d, tables %llx)\n", gSmmuRegs != nullptr,
          static_cast<unsigned long long>(tables));
    return false;
  }
  const guampwifi::Memory regs = {nullptr, 0x80000, regRd, regWr, fence};
  const guampwifi::Memory tbl = {nullptr, S::TableBytes, tblRd, tblWr, fence};
  const int e = gSmmu.start(regs, tbl, tables, guamp_phys_base, guamp_phys_size, true, nullptr, delayUs);
  IOLog("GuampAudioPlay: SMMU context for stream %x mask %x: %s (SMR slot %u%s, context bank %u, ASID %u, %u SMRs / %u banks, "
        "tables %llx, IOVA %llx..%llx, msw 1)\n", S::Stream, S::StreamMask, guampwifi::errorName(e), gSmmu.slot,
        gSmmu.preexisting ? " reused" : "", gSmmu.cb, gSmmu.asid, gSmmu.groupsSeen, gSmmu.banksSeen,
        static_cast<unsigned long long>(tables), static_cast<unsigned long long>(S::IovaBase),
        static_cast<unsigned long long>(S::IovaEnd));
  return e == guampwifi::Ok && gSmmu.active;
}

// FSR/FAR/FSYNR0 of the bank when they change (0 = no fault)
void report(const char *when) {
  if (!gSmmu.active) return;
  const uint32_t fsr = gSmmu.faultStatus();
  if (fsr == gLastFsr) return;
  gLastFsr = fsr;
  IOLog("GuampAudioPlay: SMMU stream %x %s: FSR %x FAR %x%08x FSYNR0 %x%s\n", guampwifi::AdspAudioSmmu::Stream, when, fsr,
        gSmmu.faultAddressHigh(), gSmmu.faultAddressLow(), gSmmu.faultSyndrome(), fsr ? " (ADSP access FAULTED)" : "");
}

apr::SmmuView probe() {
  apr::SmmuView v;
  volatile uint8_t *r = mapPhysical(kSmmuPhys, 0x1000);
  if (!r) {
    IOLog("GuampAudioPlay: apps SMMU %llx not mapped\n", kSmmuPhys);
    return v;
  }
  auto rd = [r](uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(r + off); };
  const uint32_t scr0 = rd(0x0), idr0 = rd(0x20);
  uint32_t groups = idr0 & 0xffu;
  if (groups > 128) groups = 128;
  uint32_t smr[128], s2cr[128];
  for (uint32_t i = 0; i < groups; i++) {
    smr[i] = rd(0x800 + 4 * i);
    s2cr[i] = rd(0xc00 + 4 * i);
  }
  v = apr::smmuDecide(scr0, smr, s2cr, groups, kStream);
  IOLog("GuampAudioPlay: apps SMMU sCR0 %x IDR0 %x, %u SMRs; stream %x: %s (slot %d, SMR %x, S2CR %x, cb %u)\n", scr0,
        idr0, groups, kStream, apr::smmuPathName(v.path), v.slot == 0xffffffffu ? -1 : static_cast<int>(v.slot), v.smr,
        v.s2cr, v.cb);
  return v;
}

// One ADSP-visible buffer: probe, address mode (physical or the 0x1c1 context bank at `iova`), allocation (once per
// slot and boot), zeroed. who = log name.
bool alloc(IOBufferMemoryDescriptor **slot, uint32_t bytes, const char *who, apr::Shm *out, uint64_t iova = 0,
           bool *mapped = nullptr) {
  const apr::SmmuView v = probe();
  uint32_t msw = 1;
  const bool viaSmmu = gMemMode == 3 || (gMemMode == 0 && !v.usable());
  if (gMemMode == 1) msw = 0;
  if (viaSmmu && (!iova || !mapped || !smmuUp())) {
    IOLog("GuampAudioPlay: stream %x (%s): no SMMU context, no %s buffer for the ADSP; physical modes: "
          "guamp-adsp-pcm-mem=2 (msw 1) or =1 (msw 0)\n", kStream, apr::smmuPathName(v.path), who);
    return false;
  }
  if (!*slot)
    *slot = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
        kernel_task, static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous) |
                         static_cast<IOOptionBits>(kIOMapInhibitCache), bytes, 0xFFFFF000ULL);
  IOBufferMemoryDescriptor *buf = *slot;
  uint8_t *cpu = buf ? static_cast<uint8_t *>(buf->getBytesNoCopy()) : nullptr;
  const uint64_t phys = buf ? buf->getPhysicalSegment(0, nullptr, kIOMemoryMapperNone) : 0;
  if (!cpu || !phys || phys + bytes > 0x100000000ULL || (phys & 0xfff) || buf->getLength() < bytes) {
    IOLog("GuampAudioPlay: %s buffer allocation failed (phys %llx)\n", who, static_cast<unsigned long long>(phys));
    return false;
  }
  for (uint32_t i = 0; i < bytes; i++) cpu[i] = 0;
  if (viaSmmu) {
    if (!*mapped) {
      const int e = gSmmu.map(iova, phys, (bytes + 0xfffu) & ~0xfffu);
      IOLog("GuampAudioPlay: %s buffer phys %llx -> IOVA %llx (%u bytes) in the 0x1c1 context: %s\n", who,
            static_cast<unsigned long long>(phys), static_cast<unsigned long long>(iova), bytes, guampwifi::errorName(e));
      if (e) return false;
      *mapped = true;
    }
    *out = {cpu, static_cast<uint32_t>(iova), 1, bytes};
    report("after map");
    return true;
  }
  *out = {cpu, static_cast<uint32_t>(phys), msw, bytes};
  IOLog("GuampAudioPlay: %s buffer phys %llx, %u bytes uncached, ADSP address msw %u lsw %x (mode %u: %s)\n", who,
        static_cast<unsigned long long>(phys), bytes, msw, static_cast<uint32_t>(phys), gMemMode,
        gMemMode ? "forced" : "probe says bypass");
  return true;
}

bool shm(apr::Shm *out) { return alloc(&gBuf, kBytes, "shared (playback)", out, kIovaPlay, &gPlayMapped); }

// The /dev/guamp-audio ring as the ADSP source (guamp-adsp-ipc=6)
uint32_t source(void *, uint8_t *dst, uint32_t frames, uint32_t bits) {
  if (!audiodev::gLock) {
    for (uint32_t i = 0; i < frames * 2 * (bits == 24 ? 4u : 2u); i++) dst[i] = 0;
    return 0;
  }
  IOLockLock(audiodev::gLock);
  const uint32_t n = audiodev::gStream.pull(dst, frames, bits, mach_absolute_time());
  IOLockUnlock(audiodev::gLock);
  return n;
}
bool wanted(void *, uint32_t *route) {
  if (!audiodev::gLock) return false;
  IOLockLock(audiodev::gLock);
  const bool w = audiodev::gOpen && audiodev::gStream.running;
  *route = audiodev::gStream.route;
  IOLockUnlock(audiodev::gLock);
  return w;
}
void attach(void *, bool on) {
  if (!audiodev::gLock) return;
  IOLockLock(audiodev::gLock);
  audiodev::gStream.setSink(on ? guampaudio::SinkAdsp : guampaudio::SinkNull, mach_absolute_time());
  const uint32_t queued = audiodev::gStream.used() / GUAMP_AUDIO_BYTES_PER_FRAME;
  IOLockUnlock(audiodev::gLock);
  IOLog("GuampAudioPlay: /dev/guamp-audio sink %s (%u frames queued)\n", on ? "ADSP" : "null", queued);
  report(on ? "sink attached" : "sink detached");
}

// Before adsp::start: the glue the guamp-adsp thread uses at levels 5/6
void start() {
  PE_parse_boot_argn("guamp-adsp-pcm-mem", &gMemMode, sizeof(gMemMode));
  if (gMemMode > 3) gMemMode = 0;
  adspipc::gPlayGlue.shm = shm;
  adspipc::gPlayGlue.source = source;
  adspipc::gPlayGlue.wanted = wanted;
  adspipc::gPlayGlue.attach = attach;
  adspipc::gPlayGlue.report = report;
#ifdef GUAMP_AUDIO_PLAY_CLOCK_HOOK
  guamp_audio_hw_set_lpass_clock_hook(adspipc::playLpassClock);
#endif
}

}  // namespace audioplay

// GuampAudioMicHw.h: the capture's buffer (one per boot, at most 64 KiB, size fixed by the first call)
extern "C" int guamp_audio_shm_get(uint32_t bytes, struct guamp_audio_shm *out) {
  using namespace audioplay;
  if (!out || !bytes || bytes > 0x10000) return -22;
  bytes = (bytes + 0xfffu) & ~0xfffu;
  if (gCapBuf && bytes > gCapBytes) return -22;
  if (!gCapBuf) gCapBytes = bytes;
  apr::Shm s;
  if (!alloc(&gCapBuf, gCapBytes, "capture", &s, kIovaCapture, &gCapMapped)) return -12;
  *out = {s.cpu, s.iovaLsw, s.msw, s.bytes};
  return 0;
}
// Never freed: the ADSP may still hold the mapping (err-fatal, unanswered unmap); the next get reuses it.
extern "C" void guamp_audio_shm_put(struct guamp_audio_shm *shm) {
  if (shm) *shm = {};
}
#endif
