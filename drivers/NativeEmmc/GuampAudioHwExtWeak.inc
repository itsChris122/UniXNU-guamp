// SPDX-License-Identifier: BSD-2-Clause-Patent
// Weak stand-ins of GuampAudioHwExt.h for a kernel without GuampAudioHw.inc. Left out when the provider is in the
// same translation unit (GUAMP_AUDIO_HW_PROVIDED = 1).
#ifndef GUAMP_AUDIO_HW_EXT_WEAK_INC
#define GUAMP_AUDIO_HW_EXT_WEAK_INC
#include "GuampAudioHwExt.h"
#if !defined(GUAMP_AUDIO_HW_PROVIDED) || !GUAMP_AUDIO_HW_PROVIDED
extern "C" {
__attribute__((weak)) int guamp_audio_hw_mute(uint32_t, int) { return GUAMP_AUDIO_HW_ABSENT; }
__attribute__((weak)) int guamp_audio_hw_get_state(uint32_t, struct guamp_audio_hw_state *) { return GUAMP_AUDIO_HW_ABSENT; }
__attribute__((weak)) void guamp_audio_hw_set_lpass_clock_hook(guamp_audio_lpass_clock_fn) {}
__attribute__((weak)) int guamp_audio_hw_bolero_fsgen(int) { return GUAMP_AUDIO_HW_ABSENT; }
}
#endif
#endif
