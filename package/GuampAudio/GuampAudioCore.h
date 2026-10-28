// SPDX-License-Identifier: BSD-2-Clause-Patent
// The parts of GuampAudio.driver that do not touch the HAL or the OS, so a host test (tests/guamp_audio_core_test.c) runs them
// exactly as the plug-in does: the zero time stamp clock, the volume law, Float32 -> s16 conversion and the staging FIFO that
// absorbs short writes to /dev/guamp-audio. Plain C11, header only, no allocation, no locking (the caller serialises).
#ifndef GUAMP_AUDIO_CORE_H
#define GUAMP_AUDIO_CORE_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// ---- clock: zero time stamps every periodFrames frames of host time ---------------------------------------------------------
// The HAL anchors its own clock on the (sample time, host time) pairs GetZeroTimeStamp returns. They must be exactly
// periodFrames apart in sample time and periodFrames * ticksPerFrame apart in host time, and advance one period at a time
// (the HAL asks often, so a late caller catches up one period per call, as Apple's NullAudio sample does). The seed changes
// only when the time line is restarted (StartIO from idle), which tells the HAL to drop its filter state.
typedef struct {
  double ticksPerFrame;   // host ticks (mach_absolute_time units) per frame
  uint32_t periodFrames;  // kAudioDevicePropertyZeroTimeStampPeriod
  uint64_t anchor;        // host time of sample 0
  uint64_t periods;       // periods elapsed (sample time = periods * periodFrames)
  uint64_t seed;
} GuampClock;

static inline double guamp_ticks_per_frame(uint32_t numer, uint32_t denom, double rate) {
  // mach_timebase_info: nanoseconds = ticks * numer / denom
  return (1.0e9 / rate) * (double)denom / (double)numer;
}
static inline void guamp_clock_init(GuampClock *c, double ticksPerFrame, uint32_t periodFrames) {
  memset(c, 0, sizeof(*c));
  c->ticksPerFrame = ticksPerFrame;
  c->periodFrames = periodFrames;
}
static inline void guamp_clock_start(GuampClock *c, uint64_t now) {
  c->anchor = now;
  c->periods = 0;
  c->seed++;
}
static inline uint64_t guamp_clock_host_of(const GuampClock *c, uint64_t periods) {
  return c->anchor + (uint64_t)(c->ticksPerFrame * (double)c->periodFrames * (double)periods + 0.5);
}
static inline void guamp_clock_zero(GuampClock *c, uint64_t now, double *sampleTime, uint64_t *hostTime, uint64_t *seed) {
  if (guamp_clock_host_of(c, c->periods + 1) <= now) c->periods++;
  *sampleTime = (double)c->periods * (double)c->periodFrames;
  *hostTime = guamp_clock_host_of(c, c->periods);
  *seed = c->seed;
}

// ---- volume law ---------------------------------------------------------------------------------------------------------------
// The volume control is linear in dB between minDb and maxDb (scalar 0..1), as the iOS volume steps expect.
static inline float guamp_clampf(float v, float lo, float hi) {
  if (v != v) return lo;   // NaN
  return v < lo ? lo : v > hi ? hi : v;
}
static inline float guamp_scalar_to_db(float scalar, float minDb, float maxDb) {
  return minDb + guamp_clampf(scalar, 0.0f, 1.0f) * (maxDb - minDb);
}
static inline float guamp_db_to_scalar(float db, float minDb, float maxDb) {
  return (guamp_clampf(db, minDb, maxDb) - minDb) / (maxDb - minDb);
}
// Linear gain, Q16 (65536 = unity): 0 when muted or at the bottom of the range (the bottom step is silence on iOS).
static inline uint32_t guamp_gain_q16(float db, float minDb, float maxDb, int mute) {
  db = guamp_clampf(db, minDb, maxDb);
  if (mute || db <= minDb) return 0;
  const double g = pow(10.0, (double)db / 20.0) * 65536.0 + 0.5;
  return g >= 65536.0 ? 65536u : (uint32_t)g;
}

// ---- Float32 -> s16 ----------------------------------------------------------------------------------------------------------
// Scales by gainQ16, rounds to nearest, saturates; NaN becomes 0.
static inline void guamp_f32_to_s16(const float *in, int16_t *out, size_t samples, uint32_t gainQ16) {
  const float g = (float)gainQ16 * (32767.0f / 65536.0f);
  for (size_t i = 0; i < samples; i++) {
    float v = in[i];
    if (v != v) v = 0.0f;
    v *= g;
    v = v >= 32767.0f ? 32767.0f : v <= -32768.0f ? -32768.0f : v;
    out[i] = (int16_t)lrintf(v);
  }
}

// ---- staging FIFO of s16 frames -----------------------------------------------------------------------------------------------
// WriteMix converts into the FIFO, then flushes as much as the device takes; what the device refuses (ring full) waits for the
// next cycle. When the FIFO itself is full, the oldest frames are dropped (bounded latency: the newest audio wins).
#define GUAMP_FIFO_FRAMES 4096u   // power of two; 85 ms at 48 kHz
#define GUAMP_FIFO_CHANNELS 2u
typedef struct {
  int16_t data[GUAMP_FIFO_FRAMES * GUAMP_FIFO_CHANNELS];
  uint64_t head, tail;   // frames ever pushed / ever consumed (tail <= head <= tail + GUAMP_FIFO_FRAMES)
  uint64_t dropped;      // frames discarded because the FIFO was full
  uint64_t written;      // frames the sink took
} GuampFifo;

static inline void guamp_fifo_reset(GuampFifo *f) {
  f->head = f->tail = 0;
}
static inline uint32_t guamp_fifo_used(const GuampFifo *f) {
  return (uint32_t)(f->head - f->tail);
}
// Push frames (interleaved stereo s16); returns the frames that had to be dropped from the old end to make room.
static inline uint32_t guamp_fifo_push(GuampFifo *f, const int16_t *frames, uint32_t count) {
  uint32_t lost = 0;
  if (count > GUAMP_FIFO_FRAMES) {   // only the newest GUAMP_FIFO_FRAMES can ever be kept
    lost += count - GUAMP_FIFO_FRAMES;
    frames += (size_t)(count - GUAMP_FIFO_FRAMES) * GUAMP_FIFO_CHANNELS;
    count = GUAMP_FIFO_FRAMES;
  }
  const uint32_t room = GUAMP_FIFO_FRAMES - guamp_fifo_used(f);
  if (count > room) {
    lost += count - room;
    f->tail += count - room;
  }
  for (uint32_t done = 0; done < count;) {
    const uint32_t at = (uint32_t)(f->head % GUAMP_FIFO_FRAMES), run = GUAMP_FIFO_FRAMES - at < count - done ? GUAMP_FIFO_FRAMES - at : count - done;
    memcpy(&f->data[(size_t)at * GUAMP_FIFO_CHANNELS], &frames[(size_t)done * GUAMP_FIFO_CHANNELS],
           (size_t)run * GUAMP_FIFO_CHANNELS * sizeof(int16_t));
    f->head += run;
    done += run;
  }
  f->dropped += lost;
  return lost;
}
// The sink takes bytes and returns how many it took (>= 0, a multiple of the frame size) or < 0 for "no room now" / error.
typedef long (*GuampSink)(void *ctx, const void *bytes, size_t length);
// Flushes until the FIFO is empty or the sink stops taking; returns the last negative sink result, or 0.
static inline long guamp_fifo_flush(GuampFifo *f, GuampSink sink, void *ctx) {
  const size_t frameBytes = GUAMP_FIFO_CHANNELS * sizeof(int16_t);
  while (guamp_fifo_used(f)) {
    const uint32_t at = (uint32_t)(f->tail % GUAMP_FIFO_FRAMES), used = guamp_fifo_used(f);
    const uint32_t run = GUAMP_FIFO_FRAMES - at < used ? GUAMP_FIFO_FRAMES - at : used;
    const long took = sink(ctx, &f->data[(size_t)at * GUAMP_FIFO_CHANNELS], (size_t)run * frameBytes);
    if (took < 0) return took;
    const uint32_t frames = (uint32_t)((size_t)took / frameBytes);
    f->tail += frames;
    f->written += frames;
    if (frames < run) break;   // short write: the device ring is full for now
  }
  return 0;
}

#endif
