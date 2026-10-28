// SPDX-License-Identifier: BSD-2-Clause-Patent
// Extra C interface of the guamp audio output hardware (GuampAudioHw.inc), next to GuampAudioHw.h (the four calls the
// ADSP side already makes). For the DSP agent (AFE/ADM/ASM, /dev/guamp-audio) and the capture agent (TX/VA macro).
// Stand-ins for kernels without the HW side: GuampAudioHwExtWeak.inc (same GUAMP_AUDIO_HW_PROVIDED rule).
#ifndef GUAMP_AUDIO_HW_EXT_H
#define GUAMP_AUDIO_HW_EXT_H
#include <stdint.h>
#include "GuampAudioHw.h"
#ifdef __cplusplus
extern "C" {
#endif

struct guamp_audio_hw_state {
  uint32_t enabled;     // guamp-audio-hw boot argument (0 = HW side off: every call answers GUAMP_AUDIO_HW_ABSENT)
  uint32_t prepared;    // speaker: profile written, amplifier powered down
  uint32_t running;     // speaker: PWDN 0, PLLS+CLKS locked
  uint32_t muted;
  uint32_t chip_id;     // aw882xx register 0x00 (0x1852), 0 if never read
  uint32_t att_cb;      // amplifier attenuation now in force (centibel, >= the floor)
  uint32_t sysst;       // last aw882xx SYSST read
  int32_t last_rc;      // last error of this module (0 = none)
};

// Mute without powering down (speaker: aw882xx HMUTE; headphone: rx macro PGA mute). 0 / ABSENT / -errno.
int guamp_audio_hw_mute(uint32_t route, int mute);
int guamp_audio_hw_get_state(uint32_t route, struct guamp_audio_hw_state *st);

// The headphone path needs the LPASS RX macro clocks (rx_core 0x30e, rx_npl 0x30f, 22.5792 MHz) that only the ADSP
// can switch (AFE CLOCK_SET, DSP side). The DSP side registers its implementation here (any time, may be null);
// it must return 0 when the clock runs. Without it the RX macro registers are never touched (bus hang risk).
typedef int (*guamp_audio_lpass_clock_fn)(uint32_t clk_id, uint32_t hz, int on);
void guamp_audio_hw_set_lpass_clock_hook(guamp_audio_lpass_clock_fn fn);

// The shared bolero FS generator is guamp_bolero_fsgen (GuampAudioMicHw.h, one refcounted owner in GuampMicDev.inc).

#ifdef __cplusplus
}
#endif
#endif
