// SPDX-License-Identifier: BSD-2-Clause-Patent
// The PCM ring of /dev/guamp-audio (ABI v2, package/GuampAudio/guamp_audio_dev.h: the one header shared with the
// plug-in) and its playback clock, without any kernel call: the device glue (GuampAudioDev.inc) holds the lock and
// passes the time in host ticks; tests/audio_pcm_host.cpp drives it on the PC.
// Producer: write() of the plug-in. Consumers: the null sink (consume(now): frames due at 48 kHz since the anchor) or
// the ADSP sink (pull(): one ASM period at a time, apr::Playback in GuampAsm.inc), which also applies the SET_VOLUME
// gain and widens to 24-bit when the ADSP stream is 24-bit. Counters are 64-bit byte totals: used = wr - rd.
#ifndef GUAMP_AUDIO_RING_INC
#define GUAMP_AUDIO_RING_INC
#include "../../package/GuampAudio/guamp_audio_dev.h"
namespace guampaudio {

constexpr uint32_t kRingBytes = GUAMP_AUDIO_RING_FRAMES * GUAMP_AUDIO_BYTES_PER_FRAME;
enum Sink : uint32_t { SinkNull = 0, SinkAdsp = 1 };

// gainQ16 0..0x10000 (unity = copy)
inline int32_t scale(int16_t s, uint32_t gainQ16) {
  return gainQ16 >= GUAMP_AUDIO_GAIN_UNITY ? s : static_cast<int32_t>((static_cast<int64_t>(s) * gainQ16) >> 16);
}

struct Stream {
  uint8_t *buf = nullptr;
  uint32_t bytes = 0;                // power of two, multiple of 4
  uint64_t wr = 0, rd = 0;
  uint64_t ticksPerSec = 0;          // host clock (mach_absolute_time) frequency
  uint64_t anchorTicks = 0, anchorFrame = 0;   // clock: frame anchorFrame was due (null) / taken (ADSP) at anchorTicks
  bool running = false, clockRunning = false;
  uint32_t sink = SinkNull, route = GUAMP_AUDIO_DEV_ROUTE_SPEAKER, underruns = 0;
  uint32_t routes = 1u << GUAMP_AUDIO_DEV_ROUTE_SPEAKER;   // wired routes (set by the device glue)
  uint32_t gainQ16 = GUAMP_AUDIO_GAIN_UNITY, mute = 0, dspBits = 0;
  int32_t centiDb = 0;
  bool adspEnabled = false, hwReady = false;   // ADSP sink configured in this boot / path streaming now
  uint32_t routeChanges = 0, volumeChanges = 0;
  // On-demand ADSP sink (guamp-adsp-ipc=6). iOS starts IO for each sound and stops it right after (a key click: StartIO, about
  // 300 ms of PCM, StopIO), while the ADSP path needs about 0.45 s to come up. So:
  //  - hold: a sound written while the path is down is kept in the ring (the null sink stops consuming) until the path takes
  //    it, at most kHoldSec; then it is dropped (path failed);
  //  - draining: STOP/close with that sound still queued keeps it (and the volume) for the ADSP, wanted until played;
  //  - idle gate (guamp-audio-idle-ms, idleTicks): a running stream that writes only silence for that long stops being wanted,
  //    so the path and the amplifier go down even if a client keeps IO running;
  //  - trim: once the path streams, runs of silence at the head beyond kTargetFrames are skipped, so the delay of the held
  //    start disappears at the first pause; if the client is still writing when the path comes up, the queue is cut to
  //    2 x kTargetFrames at once (live edge).
  static constexpr int32_t kSilence = 2;                   // |sample| <= this counts as silence
  static constexpr uint32_t kTargetFrames = 4 * GUAMP_AUDIO_PERIOD_FRAMES, kMinSilentRun = 48;
  static constexpr uint64_t kHoldSec = 2;
  uint64_t idleTicks = 0, lastSoundTicks = 0, holdSince = 0;
  bool hold = false, draining = false;
  uint32_t holds = 0, holdDrops = 0, drains = 0;
  uint64_t trimmedFrames = 0;

  uint32_t used() const { return static_cast<uint32_t>(wr - rd); }
  uint32_t space() const { return bytes - used(); }
  uint64_t framesPlayed() const { return rd / GUAMP_AUDIO_BYTES_PER_FRAME; }
  // open/close: a draining sound keeps its queue and its volume (the next open appends after it)
  void reset() {
    running = false;
    underruns = 0;
    if (draining && used()) return;
    wr = rd = 0;
    clockRunning = hold = draining = false;
    gainQ16 = GUAMP_AUDIO_GAIN_UNITY;
    mute = 0;
    centiDb = 0;
  }
  static bool loud(const uint8_t *p, uint32_t n) {
    for (uint32_t i = 0; i + 1 < n; i += 2) {
      const int32_t v = static_cast<int16_t>(static_cast<uint16_t>(p[i] | (p[i + 1] << 8)));
      if (v > kSilence || v < -kSilence) return true;
    }
    return false;
  }
  bool silentFrameAt(uint64_t at) const {
    for (uint32_t b = 0; b < GUAMP_AUDIO_BYTES_PER_FRAME; b += 2) {
      const int32_t v = static_cast<int16_t>(
          static_cast<uint16_t>(buf[(at + b) & (bytes - 1)] | (buf[(at + b + 1) & (bytes - 1)] << 8)));
      if (v > kSilence || v < -kSilence) return false;
    }
    return true;
  }
  // Whether the ADSP path should be up (the sink glue's wanted(); the device lock is held)
  bool wantAdsp(uint64_t now) const {
    if (!adspEnabled) return false;
    if (hold || (draining && used())) return true;
    if (!running) return false;
    if (!idleTicks) return true;
    return lastSoundTicks && now - lastSoundTicks < idleTicks;
  }
  void dropHold() {
    hold = draining = false;
    holdDrops++;
    rd = wr;
    clockRunning = false;
  }
  // Producer: copies up to n bytes (whole frames) that fit; returns the bytes taken
  uint32_t push(const uint8_t *src, uint32_t n, uint64_t now) {
    uint32_t take = n < space() ? n : space();
    take -= take % GUAMP_AUDIO_BYTES_PER_FRAME;
    for (uint32_t i = 0; i < take; i++) buf[(wr + i) & (bytes - 1)] = src[i];
    wr += take;
    if (take && adspEnabled && loud(src, take)) {
      lastSoundTicks = now ? now : 1;
      if (sink == SinkNull && !hold) {
        hold = true;
        holdSince = now;
        holds++;
      }
    }
    if (take && running && sink == SinkNull && !clockRunning) anchor(now);   // restart after a dry ring
    return take;
  }
  // ADSP consumer: `frames` frames into dst as `bits`-bit samples (16: s16 LE; 24: s24 in the top 24 bits of an s32 LE,
  // the playback rule of asm_multi_channel_pcm_fmt_blk_v2), with the volume gain; silence for the missing part (an
  // underrun while running). Returns the frames taken from the ring.
  uint32_t pull(uint8_t *dst, uint32_t frames, uint32_t bits, uint64_t now) {
    trim();
    const uint32_t avail = used() / GUAMP_AUDIO_BYTES_PER_FRAME, have = avail < frames ? avail : frames;
    const uint32_t width = bits == 24 ? 4 : 2, samples = frames * GUAMP_AUDIO_CHANNELS;
    for (uint32_t i = 0; i < samples; i++) {
      int32_t v = 0;
      if (i < have * GUAMP_AUDIO_CHANNELS) {
        const uint64_t at = rd + 2ull * i;
        const uint16_t raw = static_cast<uint16_t>(buf[at & (bytes - 1)] | (buf[(at + 1) & (bytes - 1)] << 8));
        v = scale(static_cast<int16_t>(raw), gainQ16);
      }
      const uint32_t out = width == 4 ? static_cast<uint32_t>(v) << 16 : static_cast<uint32_t>(v);
      for (uint32_t b = 0; b < width; b++) dst[i * width + b] = static_cast<uint8_t>(out >> (8 * b));
    }
    rd += static_cast<uint64_t>(have) * GUAMP_AUDIO_BYTES_PER_FRAME;
    if (have < frames && running) underruns++;
    if (draining && !used()) draining = false;
    anchorTicks = now;          // position: frames handed to the ADSP so far, at the time they were taken
    anchorFrame = framesPlayed();
    clockRunning = true;
    return have;
  }
  // Null sink: consume what is due at now. The clock stops when the ring runs dry (underrun) and restarts at a write.
  void consume(uint64_t now) {
    if (hold && sink == SinkNull) {
      if (!ticksPerSec || now - holdSince < kHoldSec * ticksPerSec) return;   // the ADSP path is coming up
      dropHold();
      return;
    }
    if (!running || sink != SinkNull || !clockRunning || !ticksPerSec || now < anchorTicks) return;
    const uint64_t due = anchorFrame + (now - anchorTicks) * GUAMP_AUDIO_RATE / ticksPerSec;
    const uint64_t played = framesPlayed(), queued = used() / GUAMP_AUDIO_BYTES_PER_FRAME;
    if (due <= played) return;
    if (due - played <= queued) {
      rd += (due - played) * GUAMP_AUDIO_BYTES_PER_FRAME;
      return;
    }
    rd = wr;   // ran dry before now
    clockRunning = false;
    underruns++;
  }
  // ADSP consumer, before each period: skip head silence (runs of at least kMinSilentRun frames) beyond kTargetFrames queued
  void trim() {
    const uint32_t queued = used() / GUAMP_AUDIO_BYTES_PER_FRAME;
    if (queued <= kTargetFrames) return;
    const uint32_t excess = queued - kTargetFrames;
    uint32_t run = 0;
    while (run < excess && silentFrameAt(rd + static_cast<uint64_t>(run) * GUAMP_AUDIO_BYTES_PER_FRAME)) run++;
    if (run < kMinSilentRun) return;
    rd += static_cast<uint64_t>(run) * GUAMP_AUDIO_BYTES_PER_FRAME;
    trimmedFrames += run;
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
  // Switches the consumer (under the device lock): the ADSP path streaming or not
  void setSink(uint32_t s, uint64_t now) {
    if (s == sink) return;
    sink = s;
    hwReady = s == SinkAdsp;
    clockRunning = false;
    hold = false;
    if (s == SinkNull) draining = false;
    // Path up while the client still writes (music, video): start near the live edge, so the picture is not followed by the
    // path's start-up delay for the whole stream. A sound that already ended (draining) plays whole, late.
    if (s == SinkAdsp && running && !draining && used() > 2 * kTargetFrames * GUAMP_AUDIO_BYTES_PER_FRAME) {
      const uint64_t keep = 2ull * kTargetFrames * GUAMP_AUDIO_BYTES_PER_FRAME;
      trimmedFrames += (used() - keep) / GUAMP_AUDIO_BYTES_PER_FRAME;
      rd = wr - keep;
    }
    if (running && s == SinkNull && used()) anchor(now);
  }
  uint32_t flags() const {
    return (running ? GUAMP_AUDIO_INFO_RUNNING : 0u) | (hwReady ? GUAMP_AUDIO_INFO_HW_READY : 0u) |
           GUAMP_AUDIO_INFO_HW_VOLUME | (adspEnabled ? GUAMP_AUDIO_INFO_SINK_ADSP : 0u);
  }

  // ioctl core: errno-style result (0 = ok). data is the kernel copy of the argument (sized by the command).
  int ioctl(uint32_t cmd, void *data, uint64_t now, uint64_t framesWritten) {
    switch (cmd) {
      case GUAMP_AUDIO_GET_INFO: {
        consume(now);
        guamp_audio_info *i = static_cast<guamp_audio_info *>(data);
        *i = {GUAMP_AUDIO_ABI_VERSION, GUAMP_AUDIO_RATE, GUAMP_AUDIO_CHANNELS, GUAMP_AUDIO_BITS,
              bytes / GUAMP_AUDIO_BYTES_PER_FRAME, GUAMP_AUDIO_PERIOD_FRAMES, flags(), routes, framesWritten,
              framesPlayed(), underruns, route, adspEnabled ? dspBits : 0u};
        return 0;
      }
      case GUAMP_AUDIO_START:
        if (!running) {
          running = true;
          draining = false;   // the new stream carries on after what is queued (hold, if any, stays)
          clockRunning = false;
          if (used() && sink == SinkNull) anchor(now);
        }
        return 0;
      case GUAMP_AUDIO_STOP:
        running = false;
        if (adspEnabled && used() && (hold || sink == SinkAdsp)) {   // the end of a sound the ADSP has not played yet
          draining = true;
          drains++;
          return 0;
        }
        clockRunning = hold = draining = false;
        rd = wr;
        return 0;
      case GUAMP_AUDIO_SET_VOLUME: {
        const guamp_audio_volume *v = static_cast<const guamp_audio_volume *>(data);
        gainQ16 = v->gain_q16 > GUAMP_AUDIO_GAIN_UNITY ? GUAMP_AUDIO_GAIN_UNITY : v->gain_q16;
        centiDb = v->centi_db;
        mute = v->mute ? 1 : 0;
        volumeChanges++;
        return 0;
      }
      case GUAMP_AUDIO_SET_ROUTE: {
        const uint32_t r = *static_cast<const uint32_t *>(data);
        if (r < GUAMP_AUDIO_DEV_ROUTE_SPEAKER || r > GUAMP_AUDIO_DEV_ROUTE_HEADPHONE) return 22;   // EINVAL
        if (!(routes & (1u << r))) return 19;                                                       // ENODEV
        if (r != route) routeChanges++;
        route = r;
        return 0;
      }
      case GUAMP_AUDIO_GET_ROUTE: *static_cast<uint32_t *>(data) = route; return 0;
      case GUAMP_AUDIO_GET_POSITION: {
        consume(now);
        guamp_audio_position *p = static_cast<guamp_audio_position *>(data);
        *p = {framesPlayed(), positionTicks(now), used() / GUAMP_AUDIO_BYTES_PER_FRAME, underruns};
        return 0;
      }
      default: return 25;   // ENOTTY (DRAIN is handled by the glue: it sleeps)
    }
  }
};

}  // namespace guampaudio
#endif
