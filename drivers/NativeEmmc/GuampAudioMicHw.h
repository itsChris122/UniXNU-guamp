// SPDX-License-Identifier: BSD-2-Clause-Patent
// C interface between the microphone side of guamp audio (agent 3: GuampMicCodec.inc / GuampMicPath.inc /
// GuampMicDev.inc) and the other two audio sides. Same rules as GuampAudioHw.h: extern "C", static link, weak stand-ins
// (never weak_import) in GuampMicDev.inc, left out when the provider is in the same translation unit
// (GUAMP_WCD937X_POWER_PROVIDED / GUAMP_AUDIO_SHM_PROVIDED = 1 before GuampMicDev.inc is included).
//
// Consumed by the mic side (provided elsewhere):
//   guamp_wcd937x_power(on)   agent 2 (RX macro + amplifier, owner of the codec power): wcd937x reset TLMM gpio92
//                             (sleep -> 20 us -> active -> 20 us, wcd937x_reset), cdc-vdd-rxtx/vddpx = L9A 1.8 V static,
//                             cdc-vdd-buck = L14A on demand. Refcounted by the provider (RX and TX both need it).
//                             Stand-in: GUAMP_AUDIO_HW_ABSENT, the mic side then tries anyway (state left by ABL).
//   guamp_audio_shm_get()     agent 1 (DSP playback chain, owner of the apps-SMMU context bank of stream 0x1c1): a
//                             physically contiguous, CPU-uncached buffer and its ADSP view (IOVA lsw + msw with the SID
//                             bit). Stand-in: ABSENT, the capture stops at the AFE port (no samples, node gives silence).
// Provided by the mic side (strong definitions in GuampMicDev.inc), for agent 2:
//   guamp_wcd937x_read/write  every wcd937x register (RX ones too: HPH/EAR) is reached over the VA SoundWire master
//                             (TX slave, enumeration id 0x0A01170223), which the mic side brings up. -19 (ENODEV) while
//                             the link is down.
//   guamp_bolero_fsgen(on)    the bolero clock-resource fs-gen sequence (DT 0x3000 1, 0x3004 1, 0x3080 2 in the VA
//                             macro), refcounted: RX and TX must not each write it on their own.
#ifndef GUAMP_AUDIO_MIC_HW_H
#define GUAMP_AUDIO_MIC_HW_H
#include <stdint.h>
#include "GuampAudioHw.h"
#ifdef __cplusplus
extern "C" {
#endif

struct guamp_audio_shm {
  void *cpu;           // kernel virtual address, uncached (Normal-NC)
  uint32_t iova_lsw;   // ADSP view, low 32 bits
  uint32_t msw;        // high 32 bits | SMMU SID bits (bengal: 1 for an IOVA of the 0x10000000 pool)
  uint32_t bytes;      // >= the size asked
};

int guamp_wcd937x_power(int on);
int guamp_audio_shm_get(uint32_t bytes, struct guamp_audio_shm *out);
void guamp_audio_shm_put(struct guamp_audio_shm *shm);

int guamp_wcd937x_read(uint16_t reg, uint8_t *value);
int guamp_wcd937x_write(uint16_t reg, uint8_t value);
int guamp_bolero_fsgen(int on);

#ifdef __cplusplus
}
#endif
#endif
