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
// The capture side (GuampMicDev.inc, ASM session 2, its own SHARED_MEM_MAP handle) gets a SECOND buffer of the same
// kind through guamp_audio_shm_get() (GuampAudioMicHw.h): same SMMU decision and msw, separate memory, never freed.
#ifndef GUAMP_AUDIO_PLAY_INC
#define GUAMP_AUDIO_PLAY_INC
#include "GuampAsm.inc"
#include "GuampAudioDev.inc"
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

// One ADSP-visible buffer: probe, msw decision, allocation (once per slot and boot), zeroed. who = log name.
bool alloc(IOBufferMemoryDescriptor **slot, uint32_t bytes, const char *who, apr::Shm *out) {
  const apr::SmmuView v = probe();
  uint32_t msw = 1;
  if (gMemMode == 1) msw = 0;
  else if (gMemMode == 0 && !v.usable()) {
    IOLog("GuampAudioPlay: stream %x is not in bypass (%s): no physical %s buffer for the ADSP; try guamp-adsp-pcm-mem=2 "
          "(msw 1) or =1 (msw 0)\n", kStream, apr::smmuPathName(v.path), who);
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
  *out = {cpu, static_cast<uint32_t>(phys), msw, bytes};
  IOLog("GuampAudioPlay: %s buffer phys %llx, %u bytes uncached, ADSP address msw %u lsw %x (mode %u: %s)\n", who,
        static_cast<unsigned long long>(phys), bytes, msw, static_cast<uint32_t>(phys), gMemMode,
        gMemMode ? "forced" : "probe says bypass");
  return true;
}

bool shm(apr::Shm *out) { return alloc(&gBuf, kBytes, "shared (playback)", out); }

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
  if (!alloc(&gCapBuf, gCapBytes, "capture", &s)) return -12;
  *out = {s.cpu, s.iovaLsw, s.msw, s.bytes};
  return 0;
}
// Never freed: the ADSP may still hold the mapping (err-fatal, unanswered unmap); the next get reuses it.
extern "C" void guamp_audio_shm_put(struct guamp_audio_shm *shm) {
  if (shm) *shm = {};
}
#endif
