// SPDX-License-Identifier: BSD-2-Clause-Patent
// Weak stand-ins of GuampAudioHw.h for a kernel without the codec/amplifier side: they answer GUAMP_AUDIO_HW_ABSENT,
// so the ADSP tone runs its DSP part only (no sound expected). Weak definitions, not weak_import (static link).
// Left out when the HW provider is in this translation unit (GUAMP_AUDIO_HW_PROVIDED = 1, see GuampAudioHw.h).
#ifndef GUAMP_AUDIO_HW_WEAK_INC
#define GUAMP_AUDIO_HW_WEAK_INC
#include "GuampAudioHw.h"
#if !defined(GUAMP_AUDIO_HW_PROVIDED) || !GUAMP_AUDIO_HW_PROVIDED
extern "C" {
__attribute__((weak)) int guamp_audio_hw_prepare(uint32_t, const struct guamp_audio_hw_format *) {
  return GUAMP_AUDIO_HW_ABSENT;
}
__attribute__((weak)) int guamp_audio_hw_amp(uint32_t, int) { return GUAMP_AUDIO_HW_ABSENT; }
__attribute__((weak)) int guamp_audio_hw_volume(uint32_t, int32_t) { return GUAMP_AUDIO_HW_ABSENT; }
__attribute__((weak)) void guamp_audio_hw_release(uint32_t) {}
}
#endif
#endif
