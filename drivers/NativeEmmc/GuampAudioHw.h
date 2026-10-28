// SPDX-License-Identifier: BSD-2-Clause-Patent
// C interface between the ADSP side of guamp audio (GuampAfe.inc / GuampAdspIpc.inc, branch codex/audio-dsp-20261028)
// and the codec/amplifier side (wt-audio-hw: aw882xx over QUP0 SE1 I2C, LPASS LPI pinctrl, later bolero/SoundWire).
// The DSP side calls these from its own thread (guamp-adsp; may sleep, may take tens of ms). The HW side defines them
// (strong, extern "C"). Until it does, GuampAudioHwWeak.inc gives weak stand-ins answering GUAMP_AUDIO_HW_ABSENT: the
// kernel links statically, so never weak_import. A HW provider compiled in the same translation unit as
// GuampAdsp.inc (an .inc included by NativeEmmc.cpp) defines GUAMP_AUDIO_HW_PROVIDED to 1 before GuampAdsp.inc is
// included, so the stand-ins are left out (one TU cannot hold a weak and a strong definition of the same function).
//
// Call order for the speaker (route GUAMP_AUDIO_ROUTE_SPEAKER, AFE port PRIMARY_MI2S_RX 0x1000, i2s1 = LPI gpio6..9):
//   1. AFE LPASS core vote, block DCODEC (4), held by the DSP side (AFE 0x100f4) -> LPI 0x0A7C0000 reachable
//   2. guamp_audio_hw_prepare(route, fmt): LPI i2s1 pins func2 8 mA; aw882xx reset (gpio97), chip id 0x1852, cold
//      profile aw882xx_spk_reg.bin, left powered down (PWDN 1) and muted. The I2S clocks are NOT running yet.
//   3. AFE CLOCK_SET IBIT 1.536 MHz, I2S_CONFIG, DEVICE_START: BCLK/WS now run on gpio6/7
//   4. guamp_audio_hw_amp(route, 1): PWDN 0, poll SYSST PLLS+CLKS (proves the clocks), HMUTE 0
//   5. tone (AFE DTMF) or PCM (ASM/ADM)
//   6. guamp_audio_hw_amp(route, 0): mute, PWDN 1 - before DEVICE_STOP, so the amplifier never loses its clock live
//   7. AFE DEVICE_STOP, clock off, devote
//   8. guamp_audio_hw_release(route): pins back to their sleep state (optional)
// Return values: 0 ok; GUAMP_AUDIO_HW_ABSENT no HW side linked; other negative = errno-style failure (logged by the
// DSP side, which goes on: a HW failure means no sound, not a DSP error).
#ifndef GUAMP_AUDIO_HW_H
#define GUAMP_AUDIO_HW_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define GUAMP_AUDIO_HW_ABSENT (-1000)

enum guamp_audio_route {
  GUAMP_AUDIO_ROUTE_SPEAKER = 1,     // aw882xx, PRIMARY_MI2S_RX 0x1000
  GUAMP_AUDIO_ROUTE_EARPIECE = 2,    // wcd937x EAR, RX_CODEC_DMA_RX_0 0xB030 (later)
  GUAMP_AUDIO_ROUTE_HEADPHONE = 3,   // wcd937x HPH, RX_CODEC_DMA_RX_0 0xB030 (later)
};

struct guamp_audio_hw_format {
  uint32_t rate;       // 48000
  uint32_t bits;       // 16 (slot width = bits: BCLK = rate x 2 x bits = 1.536 MHz, aw882xx I2SCTRL 0x1408)
  uint32_t channels;   // 2 on the wire (aw882xx CHSEL left)
  uint32_t afe_port;   // 0x1000
};

int guamp_audio_hw_prepare(uint32_t route, const struct guamp_audio_hw_format *fmt);
int guamp_audio_hw_amp(uint32_t route, int on);
int guamp_audio_hw_volume(uint32_t route, int32_t centibel);   // amplifier gain, 0 = 0 dB, -9600 = mute floor
void guamp_audio_hw_release(uint32_t route);

#ifdef __cplusplus
}
#endif
#endif
