// SPDX-License-Identifier: BSD-2-Clause-Patent
// The capture ring of /dev/guamp-mic (guamp_mic_dev.h) and its 48 kHz clock, without any kernel call: the device glue
// (GuampMicDev.inc) holds the lock and passes the time in host ticks; tests/mic_capture_host.cpp drives it on the PC.
// Producers: the ADSP read session (push(): one ASM period per READ_DONE, GuampMicPath.inc) or the null source
// (produce(now): silence for the frames due at 48 kHz since the anchor). Consumer: read() of the plug-in.
// Overflow drops the OLDEST frames (a recorder wants the newest audio); counters are 64-bit byte totals.
#ifndef GUAMP_MIC_RING_INC
#define GUAMP_MIC_RING_INC
#include "guamp_mic_dev.h"
namespace guampmic {

struct Capture {
  uint8_t *buf = nullptr;
  uint32_t bytes = 0;                       // power of two, multiple of 2
  uint64_t wr = 0, rd = 0, read = 0;        // read: bytes handed to the reader (rd also moves on overflow)
  uint64_t ticksPerSec = 0, anchorTicks = 0, anchorFrame = 0;
  bool running = false;
  uint32_t source = GUAMP_MIC_SOURCE_NULL, mic = GUAMP_MIC_AMIC3, overruns = 0, peak = 0;
  int32_t gainCb = 0;

  uint32_t used() const { return static_cast<uint32_t>(wr - rd); }
  uint64_t framesCaptured() const { return wr / GUAMP_MIC_FRAME_BYTES; }
  void reset() {
    wr = rd = read = 0;
    running = false;
    overruns = peak = 0;
  }
  void anchor(uint64_t now) {
    anchorTicks = now;
    anchorFrame = framesCaptured();
  }
  // Producer: whole frames; the oldest frames make room. Returns the frames stored.
  uint32_t push(const uint8_t *src, uint32_t n, uint64_t now) {
    if (!running) return 0;
    n -= n % GUAMP_MIC_FRAME_BYTES;
    if (n > bytes) {   // more than the ring: keep the newest part
      src += n - bytes;
      overruns += (n - bytes) / GUAMP_MIC_FRAME_BYTES;
      n = bytes;
    }
    const uint32_t room = bytes - used();
    if (n > room) {
      overruns += (n - room) / GUAMP_MIC_FRAME_BYTES;
      rd += n - room;
    }
    uint32_t p = 0;
    for (uint32_t i = 0; i < n; i++) buf[(wr + i) & (bytes - 1)] = src[i];
    for (uint32_t i = 0; i + 1 < n; i += 2) {
      int32_t s = static_cast<int16_t>(src[i] | (src[i + 1] << 8));
      if (s < 0) s = -s;
      if (static_cast<uint32_t>(s) > p) p = static_cast<uint32_t>(s);
    }
    if (n) peak = p > 32767 ? 32767 : p;
    wr += n;
    anchor(now);   // the ADSP period just completed: its last frame is "now"
    return n / GUAMP_MIC_FRAME_BYTES;
  }
  // Null source: silence for what is due at now (the ADSP source pushes instead)
  void produce(uint64_t now) {
    if (!running || source != GUAMP_MIC_SOURCE_NULL || !ticksPerSec || now < anchorTicks) return;
    const uint64_t due = anchorFrame + (now - anchorTicks) * GUAMP_MIC_RATE / ticksPerSec;
    uint64_t have = framesCaptured();
    if (due <= have) return;
    static const uint8_t zero[256] = {};
    uint64_t missing = (due - have) * GUAMP_MIC_FRAME_BYTES;
    if (missing > bytes) {   // the reader slept: only the last ring's worth matters
      overruns += static_cast<uint32_t>((missing - bytes) / GUAMP_MIC_FRAME_BYTES);
      rd = wr = wr + missing - bytes;
      missing = bytes;
    }
    const uint64_t keepTicks = anchorTicks, keepFrame = anchorFrame;
    while (missing) {
      const uint32_t n = missing < sizeof(zero) ? static_cast<uint32_t>(missing) : static_cast<uint32_t>(sizeof(zero));
      push(zero, n, now);
      missing -= n;
    }
    anchorTicks = keepTicks;   // the null clock keeps its own anchor (no drift from rounding)
    anchorFrame = keepFrame;
    peak = 0;
  }
  // Consumer: up to n bytes (whole frames); returns the bytes copied
  // Reads that found less than asked (31/10: recordings crackle at the right speed; the plug-in pads a short read)
  uint32_t shortReads = 0, emptyReads = 0;
  uint32_t take(uint8_t *dst, uint32_t n) {
    n -= n % GUAMP_MIC_FRAME_BYTES;
    const uint32_t have = used() < n ? used() : n;
    if (have < n) {
      if (have) shortReads++;
      else emptyReads++;
    }
    for (uint32_t i = 0; i < have; i++) dst[i] = buf[(rd + i) & (bytes - 1)];
    rd += have;
    read += have;
    return have;
  }
  uint64_t positionTicks(uint64_t now) const {
    if (!running || !ticksPerSec) return now;
    return anchorTicks + (framesCaptured() - anchorFrame) * ticksPerSec / GUAMP_MIC_RATE;
  }

  // ioctl core: errno-style result. data is the kernel copy of the argument. GAIN is applied by the glue (codec).
  int ioctl(uint32_t cmd, void *data, uint64_t now) {
    switch (cmd) {
      case GUAMP_MIC_IOC_INFO: {
        produce(now);
        guamp_mic_info *i = static_cast<guamp_mic_info *>(data);
        *i = {GUAMP_MIC_ABI, GUAMP_MIC_RATE, GUAMP_MIC_CHANNELS, GUAMP_MIC_BITS, bytes, GUAMP_MIC_PERIOD_FRAMES,
              running ? 1u : 0u, source, mic, gainCb, overruns, peak, framesCaptured(), read / GUAMP_MIC_FRAME_BYTES};
        return 0;
      }
      case GUAMP_MIC_IOC_START:
        if (!running) {
          running = true;
          anchor(now);
        }
        return 0;
      case GUAMP_MIC_IOC_STOP:
        running = false;
        rd = wr;
        return 0;
      case GUAMP_MIC_IOC_POSITION: {
        produce(now);
        guamp_mic_position *p = static_cast<guamp_mic_position *>(data);
        *p = {framesCaptured(), positionTicks(now), used() / GUAMP_MIC_FRAME_BYTES, overruns};
        return 0;
      }
      case GUAMP_MIC_IOC_GAIN: {
        const int32_t g = *static_cast<const int32_t *>(data);
        gainCb = g < -8400 ? -8400 : g > 4000 ? 4000 : g;
        return 0;
      }
      case GUAMP_MIC_IOC_SET_MIC: {
        const uint32_t m = *static_cast<const uint32_t *>(data);
        if (m != GUAMP_MIC_AMIC1 && m != GUAMP_MIC_AMIC3) return 22;   // EINVAL
        if (running && m != mic) return 16;                             // EBUSY
        mic = m;
        return 0;
      }
      default: return 25;   // ENOTTY
    }
  }
};

}  // namespace guampmic
#endif
