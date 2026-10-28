// SPDX-License-Identifier: BSD-2-Clause-Patent
// The PCM ring of /dev/guamp-audio (guamp_audio_dev.h) and its playback clock, without any kernel call: the device glue
// (GuampAudioDev.inc) holds the lock and passes the time in host ticks; tests/audio_pcm_host.cpp drives it on the PC.
// Producer: write() of the plug-in. Consumer: the null sink (consume(now): frames due at 48 kHz since the anchor) or the
// ADSP path (pull(): one ASM period at a time, GuampAsm.inc). Counters are 64-bit byte totals: used = wr - rd.
#ifndef GUAMP_AUDIO_RING_INC
#define GUAMP_AUDIO_RING_INC
#include "guamp_audio_dev.h"
namespace guampaudio {

struct Stream {
  uint8_t *buf = nullptr;
  uint32_t bytes = 0;                // power of two, multiple of 4
  uint64_t wr = 0, rd = 0;
  uint64_t ticksPerSec = 0;          // host clock (mach_absolute_time) frequency
  uint64_t anchorTicks = 0, anchorFrame = 0;   // null-sink clock: frame anchorFrame was due at anchorTicks
  bool running = false, clockRunning = false;
  uint32_t sink = GUAMP_AUDIO_SINK_NULL, route = GUAMP_AUDIO_DEV_ROUTE_SPEAKER, underruns = 0;
  int32_t volume = 0;
  uint32_t muted = 0;

  uint32_t used() const { return static_cast<uint32_t>(wr - rd); }
  uint32_t space() const { return bytes - used(); }
  uint64_t framesPlayed() const { return rd / GUAMP_AUDIO_FRAME_BYTES; }
  void reset() {
    wr = rd = 0;
    running = clockRunning = false;
    underruns = 0;
  }
  // Producer: copies up to n bytes (whole frames) that fit; returns the bytes taken
  uint32_t push(const uint8_t *src, uint32_t n, uint64_t now) {
    uint32_t take = n < space() ? n : space();
    take -= take % GUAMP_AUDIO_FRAME_BYTES;
    for (uint32_t i = 0; i < take; i++) buf[(wr + i) & (bytes - 1)] = src[i];
    wr += take;
    if (take && running && sink == GUAMP_AUDIO_SINK_NULL && !clockRunning) anchor(now);   // restart after a dry ring
    return take;
  }
  // ADSP consumer: up to n bytes into dst (silence for the missing part while running: an underrun)
  uint32_t pull(uint8_t *dst, uint32_t n) {
    const uint32_t have = used() < n ? used() : n;
    for (uint32_t i = 0; i < have; i++) dst[i] = buf[(rd + i) & (bytes - 1)];
    for (uint32_t i = have; i < n; i++) dst[i] = 0;
    rd += have;
    if (have < n && running) underruns++;
    return have;
  }
  // Null sink: consume what is due at now. The clock stops when the ring runs dry (underrun) and restarts at a write.
  void consume(uint64_t now) {
    if (!running || sink != GUAMP_AUDIO_SINK_NULL || !clockRunning || !ticksPerSec || now < anchorTicks) return;
    const uint64_t due = anchorFrame + (now - anchorTicks) * GUAMP_AUDIO_RATE / ticksPerSec;
    const uint64_t played = framesPlayed(), queued = used() / GUAMP_AUDIO_FRAME_BYTES;
    if (due <= played) return;
    if (due - played <= queued) {
      rd += (due - played) * GUAMP_AUDIO_FRAME_BYTES;
      return;
    }
    rd = wr;   // ran dry before now
    clockRunning = false;
    underruns++;
  }
  void anchor(uint64_t now) {
    anchorTicks = now;
    anchorFrame = framesPlayed();
    clockRunning = true;
  }
  // The host time at which framesPlayed() was (or is) exact
  uint64_t positionTicks(uint64_t now) const {
    if (!clockRunning || !ticksPerSec) return now;
    return anchorTicks + (framesPlayed() - anchorFrame) * ticksPerSec / GUAMP_AUDIO_RATE;
  }

  // ioctl core: errno-style result (0 = ok). data is the kernel copy of the argument (sized by the command).
  int ioctl(uint32_t cmd, void *data, uint64_t now, uint64_t framesWritten) {
    switch (cmd) {
      case GUAMP_AUDIO_IOC_INFO: {
        consume(now);
        guamp_audio_info *i = static_cast<guamp_audio_info *>(data);
        *i = {GUAMP_AUDIO_ABI, GUAMP_AUDIO_RATE, GUAMP_AUDIO_CHANNELS, GUAMP_AUDIO_BITS, bytes, GUAMP_AUDIO_PERIOD_FRAMES,
              running ? 1u : 0u, sink, route, volume, muted, underruns, framesWritten, framesPlayed()};
        return 0;
      }
      case GUAMP_AUDIO_IOC_START:
        if (!running) {
          running = true;
          clockRunning = false;
          if (used() && sink == GUAMP_AUDIO_SINK_NULL) anchor(now);
        }
        return 0;
      case GUAMP_AUDIO_IOC_STOP:
        running = clockRunning = false;
        rd = wr;
        return 0;
      case GUAMP_AUDIO_IOC_VOLUME: {
        const guamp_audio_volume *v = static_cast<const guamp_audio_volume *>(data);
        volume = v->centibel > 0 ? 0 : v->centibel < -9600 ? -9600 : v->centibel;
        muted = v->muted ? 1 : 0;
        return 0;
      }
      case GUAMP_AUDIO_IOC_ROUTE: {
        const uint32_t r = *static_cast<const uint32_t *>(data);
        if (r < GUAMP_AUDIO_DEV_ROUTE_SPEAKER || r > GUAMP_AUDIO_DEV_ROUTE_HEADPHONE) return 22;   // EINVAL
        if (r != GUAMP_AUDIO_DEV_ROUTE_SPEAKER) return 19;                                         // ENODEV: not wired yet
        route = r;
        return 0;
      }
      case GUAMP_AUDIO_IOC_POSITION: {
        consume(now);
        guamp_audio_position *p = static_cast<guamp_audio_position *>(data);
        *p = {framesPlayed(), positionTicks(now), used() / GUAMP_AUDIO_FRAME_BYTES, underruns};
        return 0;
      }
      default: return 25;   // ENOTTY (DRAIN is handled by the glue: it sleeps)
    }
  }
};

}  // namespace guampaudio
#endif
