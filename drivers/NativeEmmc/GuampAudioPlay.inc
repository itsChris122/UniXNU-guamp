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
//   guamp-adsp-pcm-mem=0 (default) uses the physical address with msw 1 only if stream 0x1c1 bypasses translation
//                        (SMMU disabled, S2CR bypass, or no SMR with unmatched streams bypassing); otherwise no PCM
//   guamp-adsp-pcm-mem=1 physical address, msw 0 (forced, whatever the probe says)
//   guamp-adsp-pcm-mem=2 physical address, msw 1 (forced)
// Next step if the probe says "translate" or "fault": an own context bank for 0x1c1 (GuampSmmu.inc pattern) - not here.
#ifndef GUAMP_AUDIO_PLAY_INC
#define GUAMP_AUDIO_PLAY_INC
#include "GuampAsm.inc"
#include "GuampAudioDev.inc"

namespace audioplay {

constexpr uint64_t kSmmuPhys = 0x0c600000;
constexpr uint32_t kStream = 0x1c1, kBytes = apr::PcmPath::kSlots * apr::PcmPath::kSlotStride;
IOBufferMemoryDescriptor *gBuf = nullptr;
unsigned gMemMode = 0;

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

bool shm(apr::Shm *out) {
  const apr::SmmuView v = probe();
  uint32_t msw = 1;
  if (gMemMode == 1) msw = 0;
  else if (gMemMode == 0 && !v.usable()) {
    IOLog("GuampAudioPlay: stream %x is not in bypass (%s): no physical buffer for the ADSP; try guamp-adsp-pcm-mem=2 "
          "(msw 1) or =1 (msw 0)\n", kStream, apr::smmuPathName(v.path));
    return false;
  }
  if (!gBuf)
    gBuf = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
        kernel_task, static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous) |
                         static_cast<IOOptionBits>(kIOMapInhibitCache), kBytes, 0xFFFFF000ULL);
  uint8_t *cpu = gBuf ? static_cast<uint8_t *>(gBuf->getBytesNoCopy()) : nullptr;
  const uint64_t phys = gBuf ? gBuf->getPhysicalSegment(0, nullptr, kIOMemoryMapperNone) : 0;
  if (!cpu || !phys || phys + kBytes > 0x100000000ULL || (phys & 0xfff)) {
    IOLog("GuampAudioPlay: shared buffer allocation failed (phys %llx)\n", static_cast<unsigned long long>(phys));
    return false;
  }
  for (uint32_t i = 0; i < kBytes; i++) cpu[i] = 0;
  *out = {cpu, static_cast<uint32_t>(phys), msw, kBytes};
  IOLog("GuampAudioPlay: shared buffer phys %llx, %u bytes uncached, ADSP address msw %u lsw %x (mode %u: %s)\n",
        static_cast<unsigned long long>(phys), kBytes, msw, static_cast<uint32_t>(phys), gMemMode,
        gMemMode ? "forced" : "probe says bypass");
  return true;
}

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
}

// Before adsp::start: the glue the guamp-adsp thread uses at levels 5/6
void start() {
  PE_parse_boot_argn("guamp-adsp-pcm-mem", &gMemMode, sizeof(gMemMode));
  if (gMemMode > 2) gMemMode = 0;
  adspipc::gPlayGlue.shm = shm;
  adspipc::gPlayGlue.source = source;
  adspipc::gPlayGlue.wanted = wanted;
  adspipc::gPlayGlue.attach = attach;
}

}  // namespace audioplay
#endif
