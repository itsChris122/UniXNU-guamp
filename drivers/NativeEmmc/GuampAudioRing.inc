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
  // Live-edge cut only when more than this is queued at attach: the path now comes up in ~100-130 ms, and cutting a held
  // system sound (lock, key click: 50-300 ms, written while IO keeps running) to its last 80 ms removed its attack.
  static constexpr uint32_t kLiveEdgeFrames = 9600;   // 200 ms
  uint64_t idleTicks = 0, lastSoundTicks = 0, holdSince = 0;
  bool hold = false, draining = false;
  uint32_t holds = 0, holdDrops = 0, drains = 0;
  uint64_t trimmedFrames = 0;
  // Pre-roll (ADSP sink, prerollFrames > 0; the device glue sets it from guamp-audio-preroll-ms, 0 = the old behaviour).
  // The ADSP takes one 10 ms period per WRITE_DONE (8 in flight) while the plug-in writes one HAL buffer (1024 frames,
  // 21 ms) per IO cycle, just in time: without a margin most periods found less than 480 frames and went out as the frames
  // there plus zeros, i.e. a gap inside the sound every 10-20 ms (the crackle; fin8 logs: 400 underruns in 7 s). Now a
  // running stream only plays whole periods: when a period cannot be filled, a period of silence goes out, nothing is taken
  // from the ring, and playback restarts once prerollFrames are queued again (a single clean gap instead of a buzz).
  // A stopped stream (draining) plays its tail to the end, zero-padded.
  uint32_t prerollFrames = 0;
  bool primed = false;
  // per session (beginSession() at open): periods of pre-roll silence, the deepest queue seen by the ADSP, write sizes
  uint32_t prerollWaits = 0, maxQueued = 0;
  uint64_t sessionBase = 0;   // framesPlayed() at open (a draining tail of the previous sound may precede it)

  uint32_t used() const { return static_cast<uint32_t>(wr - rd); }
  uint32_t space() const { return bytes - used(); }
  uint64_t framesPlayed() const { return rd / GUAMP_AUDIO_BYTES_PER_FRAME; }
  // Device clock for GET_POSITION: frames played plus the silence the ADSP played while the stream ran (pre-roll waits,
  // underruns). With framesPlayed() alone the clock stood still during every pre-roll wait, the HAL saw the device slow down
  // and wrote less, which caused the next wait (fin14: 11-19 underruns and ~3 s of inserted silence per 3.3 s sound).
  uint64_t silentFrames = 0, anchorClock = 0;
  uint64_t clockFrames() const { return framesPlayed() + silentFrames; }
  // Free-running device clock of a running stream (GET_POSITION): 48 kHz of host time from START. The HAL writes about one
  // buffer ahead of the clock it is given; a clock that moved only when the ADSP took a period (8 x 10 ms in flight) kept the
  // writes just in time, so a pre-roll above ~80 ms was never reached (fin16: pre-roll 300 ms, max queue 82 ms, 30 underruns
  // in 3.3 s). From START the HAL now writes at a steady rate while the ADSP waits for the pre-roll, and the queue then holds
  // about the pre-roll minus the periods in flight.
  bool freeClock = false;
  uint64_t freeTicks = 0, freeFrame = 0, lastPosition = 0;
  uint64_t positionFrame(uint64_t now) const {
    if (!freeClock || !ticksPerSec || now < freeTicks) return clockFrames();
    return freeFrame + (now - freeTicks) * GUAMP_AUDIO_RATE / ticksPerSec;
  }
  // open/close: a draining sound keeps its queue and its volume (the next open appends after it)
  void reset() {
    running = false;
    if (draining && used()) return;
    wr = rd = 0;
    clockRunning = hold = draining = primed = false;
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
  // Session counters (open): underruns, pre-roll, deepest queue, hold/drain/trim totals of this sound only
  void beginSession() {
    underruns = prerollWaits = maxQueued = 0;
    holds = holdDrops = drains = 0;
    routeChanges = volumeChanges = 0;
    trimmedFrames = 0;
    lastWriteTicks = maxWriteGap = 0;
    lateWrites = 0;
    pulls = burst = maxBurst = 0;
    firstPull = lastPull = 0;
    sessionBase = framesPlayed();
  }
  uint64_t sessionPlayed() const { return framesPlayed() - sessionBase; }
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
  // Write timing of this sound (fin17: 29 s written in ~36 s with the right pitch): the longest pause between two writes of a
  // running stream and how many pauses exceeded 32 ms (1.5 HAL buffers of 1024 frames), i.e. late IO cycles of coreaudiod
  uint64_t lastWriteTicks = 0, maxWriteGap = 0;
  uint32_t lateWrites = 0;
  uint32_t push(const uint8_t *src, uint32_t n, uint64_t now) {
    if (running && now && ticksPerSec) {
      if (lastWriteTicks && now > lastWriteTicks) {
        const uint64_t gap = now - lastWriteTicks;
        if (gap > maxWriteGap) maxWriteGap = gap;
        if (gap > ticksPerSec * 32 / 1000) lateWrites++;
      }
      lastWriteTicks = now;
    }
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
  // ADSP pace of this sound (31/10: underruns with on-time writes and 300 ms queued): periods taken, first/last time, and the
  // largest number taken within 2 ms of each other (a burst of WRITE_DONEs)
  uint32_t pulls = 0, burst = 0, maxBurst = 0;
  uint64_t firstPull = 0, lastPull = 0;
  uint32_t pull(uint8_t *dst, uint32_t frames, uint32_t bits, uint64_t now) {
    if (running && ticksPerSec) {
      if (!pulls) firstPull = now;
      burst = (pulls && now - lastPull < ticksPerSec / 500) ? burst + 1 : 1;
      if (burst > maxBurst) maxBurst = burst;
      lastPull = now;
      pulls++;
    }
    trim();
    const uint32_t avail = used() / GUAMP_AUDIO_BYTES_PER_FRAME;
    if (avail > maxQueued) maxQueued = avail;
    uint32_t have = avail < frames ? avail : frames;
    if (prerollFrames && running) {
      if (!primed && avail >= prerollFrames) primed = true;
      if (primed && avail < frames) {   // ran dry while playing: one clean gap, then pre-roll again
        primed = false;
        underruns++;
      }
      if (!primed) {
        if (avail) prerollWaits++;
        have = 0;
      }
    }
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
    if (have < frames && running && !prerollFrames) underruns++;
    if (have < frames && running) silentFrames += frames - have;
    if (draining && !used()) draining = false;
    anchorTicks = now;          // position: frames handed to the ADSP so far, at the time they were taken
    anchorFrame = framesPlayed();
    anchorClock = clockFrames();
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
  // With pre-roll the floor is the pre-roll depth, not kTargetFrames: trimming a pause inside a sound down to 40 ms ate the
  // 120 ms margin the pre-roll had built and the next late write was an underrun (fin13: 2-6 per 3 s sound, ~68% trimmed).
  void trim() {
    const uint32_t queued = used() / GUAMP_AUDIO_BYTES_PER_FRAME;
    const uint32_t keep = prerollFrames > kTargetFrames ? prerollFrames : kTargetFrames;
    if (queued <= keep) return;
    const uint32_t excess = queued - keep;
    uint32_t run = 0;
    while (run < excess && silentFrameAt(rd + static_cast<uint64_t>(run) * GUAMP_AUDIO_BYTES_PER_FRAME)) run++;
    if (run < kMinSilentRun) return;
    rd += static_cast<uint64_t>(run) * GUAMP_AUDIO_BYTES_PER_FRAME;
    trimmedFrames += run;
  }
  void anchor(uint64_t now) {
    anchorTicks = now;
    anchorFrame = framesPlayed();
    anchorClock = clockFrames();
    clockRunning = true;
  }
  // The host time at which clockFrames() was (or is) exact
  uint64_t positionTicks(uint64_t now) const {
    if (!clockRunning || !ticksPerSec) return now;
    return anchorTicks + (clockFrames() - anchorClock) * ticksPerSec / GUAMP_AUDIO_RATE;
  }
  // Switches the consumer (under the device lock): the ADSP path streaming or not
  void setSink(uint32_t s, uint64_t now) {
    if (s == sink) return;
    sink = s;
    hwReady = s == SinkAdsp;
    clockRunning = false;
    hold = false;
    primed = false;   // pre-roll again on the new consumer
    if (s == SinkNull) draining = false;
    // Path up while the client still writes (music, video): start near the live edge, so the picture is not followed by the
    // path's start-up delay for the whole stream. A sound that already ended (draining) plays whole, late.
    if (s == SinkAdsp && running && !draining && used() > kLiveEdgeFrames * GUAMP_AUDIO_BYTES_PER_FRAME) {
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
          if (!(draining && used())) primed = false;   // a fresh stream pre-rolls; one following a tail carries on
          draining = false;   // the new stream carries on after what is queued (hold, if any, stays)
          clockRunning = false;
          if (used() && sink == SinkNull) anchor(now);
          freeClock = true;
          freeTicks = now;
          freeFrame = clockFrames() > lastPosition ? clockFrames() : lastPosition;
        }
        return 0;
      case GUAMP_AUDIO_STOP:
        running = false;
        freeClock = false;
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
        const bool freeRun = running && freeClock && sink == SinkAdsp;   // the null sink keeps its own real-time clock
        uint64_t frame = freeRun ? positionFrame(now) : clockFrames();
        if (frame < lastPosition) frame = lastPosition;   // never backwards across sounds
        lastPosition = frame;
        *p = {frame, freeRun ? now : positionTicks(now), used() / GUAMP_AUDIO_BYTES_PER_FRAME, underruns};
        return 0;
      }
      default: return 25;   // ENOTTY (DRAIN is handled by the glue: it sleeps)
    }
  }
};

}  // namespace guampaudio
#endif
