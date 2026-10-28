// SPDX-License-Identifier: BSD-2-Clause-Patent
// Input half of GuampAudio.driver without HAL/OS calls (host test: tests/guamp_audio_core_test.c): s16 mono from
// /dev/guamp-mic (guamp_mic_dev.h) to the HAL's Float32 input buffer, and the latency rule of the capture reader.
// The kernel ring keeps up to ~341 ms; the HAL asks for `frames` per IO cycle on its own clock (mach time at 48 kHz). The
// two clocks are the same oscillator family but not locked, so the queue can slowly grow: when more than
// GUAMP_MIC_KEEP_FRAMES + frames are waiting, the excess (oldest) is skipped. A short read (nothing captured yet, or the
// source fell behind) is padded with silence and counted, never blocks the IO thread.
#ifndef GUAMP_MIC_CORE_H
#define GUAMP_MIC_CORE_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define GUAMP_MIC_KEEP_FRAMES 960u   // 20 ms of slack in the kernel ring after each cycle

// s16 -> Float32 for `channels` interleaved output channels (the same mono sample on each), [-1, 1)
static inline void guamp_s16_to_f32_dup(const int16_t *in, float *out, size_t frames, uint32_t channels) {
  for (size_t i = 0; i < frames; i++) {
    const float v = (float)in[i] * (1.0f / 32768.0f);
    for (uint32_t c = 0; c < channels; c++) out[i * channels + c] = v;
  }
}
// Frames to drop before reading `frames` when `queued` are waiting in the kernel
static inline uint32_t guamp_mic_excess(uint32_t queued, uint32_t frames) {
  const uint64_t keep = (uint64_t)frames + GUAMP_MIC_KEEP_FRAMES;
  return queued > keep ? (uint32_t)(queued - keep) : 0;
}

#endif
