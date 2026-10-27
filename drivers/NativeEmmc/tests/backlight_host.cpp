// Host test of GuampBacklightCore.h: the DCS packet against a re-implementation of the Linux path (mipi_dsi_dcs_write ->
// mipi_dsi_create_packet of this kernel -> dsi_ctrl_copy_and_pad_cmd), clamps, fade, rate limit, and the DSI sequence
// against a fake controller (register order, restore, refusals, timeouts).
#include <stdio.h>
#include <string.h>
#include <initializer_list>
#include "../GuampBacklightCore.h"
using namespace guamp_backlight;

static int failures;
static void check(bool ok, const char *what, unsigned long long a = 0, unsigned long long b = 0) {
  if (!ok) { printf("FAIL %s (%llx, %llx)\n", what, a, b); failures = failures + 1; }
}

// Linux reference: bytes of the FIFO buffer for mipi_dsi_dcs_set_display_brightness_2bytes(level), LASTCOMMAND set
static unsigned linuxBuffer(uint32_t level, uint8_t *buf) {
  const uint8_t tx[3] = {0x51, (uint8_t)((level & 0xff00) >> 8), (uint8_t)(level & 0xff)};
  const uint8_t type = sizeof(tx) == 1 ? 0x05 : sizeof(tx) == 2 ? 0x15 : 0x39;       // mipi_dsi_dcs_write_buffer
  uint8_t header[4] = {(uint8_t)(sizeof(tx) & 0xff), (uint8_t)((sizeof(tx) >> 8) & 0xff), (uint8_t)((0 << 6) | type), 0};
  const unsigned size = 4 + sizeof(tx), len = (size + 3) & ~3u;
  for (unsigned i = 0; i < len; i++) buf[i] = i >= size ? 0xff : i < 4 ? header[i] : tx[i - 4];
  buf[3] |= 1u << 6;                                                                   // payload_length > 0
  buf[3] |= 1u << 7;                                                                   // MIPI_DSI_MSG_LASTCOMMAND
  return len;
}

// Fake DSI controller: a register file; DMA_DONE rises doneAfter polls after the trigger if the command is well formed.
struct FakeDsi {
  uint32_t reg[0x200 / 4 + 1] = {};
  bool clocks = true, triggered = false, hang = false, busyForever = false;
  int doneAfter = 3, polls = 0;
  uint64_t us = 0;
  uint32_t fifo[8] = {}, fifoCount = 0, writes = 0, wrOff[256] = {}, wrVal[256] = {};
  uint32_t atTrigger[0x200 / 4 + 1] = {};
  uint32_t &r(uint32_t o) { return reg[o / 4]; }
  uint32_t rd(uint32_t o) {
    check(clocks, "register read with clocks off", o);
    if (o == rStatus && busyForever) return kStatusDmaBusy;
    if (o == rIntCtrl && triggered && !hang && ++polls >= doneAfter) r(rIntCtrl) |= kIntDmaDone;
    return r(o);
  }
  void wr(uint32_t o, uint32_t v) {
    check(clocks, "register write with clocks off", o, v);
    if (writes < 256) { wrOff[writes] = o; wrVal[writes] = v; }
    writes = writes + 1;
    if (o == rIntCtrl) { r(o) = (r(o) & ~(v & ~kIntEnables)) ; r(o) = (r(o) & ~kIntEnables) | (v & kIntEnables); return; }
    if (o == rTpgFifoReset) { if (v) fifoCount = 0; r(o) = v; return; }
    if (o == rTpgFifo) { if (fifoCount < 8) fifo[fifoCount++] = v; return; }
    if (o == rFifoStatus) { r(o) &= ~(v & kOverflowClear); return; }
    if (o == rSwTrigger) {
      memcpy(atTrigger, reg, sizeof(reg));
      triggered = true;
      polls = 0;
      return;
    }
    r(o) = v;
  }
  bool clocksOn() { return clocks; }
  void delayUs(uint32_t n) { us += n; }
  void sleepMs(uint32_t n) { us += n * 1000ull; }
  uint64_t nowUs() { return us; }
  void bootLoaderState() {
    r(rHwVersion) = 0x20040000;                                  // v2.4
    r(rCtrl) = kCtrlEnable | kCtrlVideo | (1u << 4) | (1u << 8);   // + lanes
    r(rActiveV) = (1614u << 16) | 14u;                           // vsw 2 + vbp 12, 1600 lines
    r(rTotal) = (1799u << 16) | 900u;                            // 2 + 12 + 1600 + 186 - 1
    r(rTrigCtrl) = 0x00000000;                                   // no DMA trigger chosen by the boot loader
    r(rDmaCtrl) = (1u << 26) | (1u << 31);                       // stale LP + broadcast bits
    r(rDmaSchedule) = 0x0000abcd;
    r(rErrIntMask0) = 0x12345678 & ~kOverflowMask;
    r(rIntCtrl) = (1u << 17) | (1u << 16) | (1u << 24) | kIntDmaDone;   // video-done enable+status, error status, stale done
    r(rFifoStatus) = (1u << 10) | (1u << 18) | 0x4;
  }
};

int main() {
  // Packet = Linux buffer, little-endian dwords
  for (uint32_t level : {0u, 1u, 16u, 0x100u, 0x666u, 0xcccu, 0xfffu}) {
    uint8_t ref[16];
    const unsigned len = linuxBuffer(level, ref);
    uint32_t w[2];
    encodeBrightness(level, w);
    uint8_t got[8];
    for (unsigned i = 0; i < 8; i++) got[i] = (uint8_t)(w[i / 4] >> (8 * (i % 4)));
    check(len == kPacketBytes, "8 bytes, even dword count", len);
    check(!memcmp(ref, got, 8), "packet = Linux FIFO buffer", level, w[1]);
  }
  uint32_t w[2];
  encodeBrightness(0xccc, w);
  check(w[0] == 0xc0390003u && w[1] == 0xffcc0c51u, "max level words", w[0], w[1]);
  encodeBrightness(0, w);
  check(w[1] == 0xff000051u, "off words", w[1]);

  // Clamps and fade period
  check(clampLevel(0) == 0 && clampLevel(-5) == 0, "0 and negative = off");
  check(clampLevel(1) == kMinLevel && clampLevel(15) == kMinLevel && clampLevel(16) == 16, "bl-min");
  check(clampLevel(0xccc) == 0xccc && clampLevel(0xfff) == 0xccc && clampLevel(1ll << 40) == 0xccc, "normal max, no HBM");
  check(fadeMsFromFixed(0) == 0 && fadeMsFromFixed(0x10000) == 1000 && fadeMsFromFixed(0x8000) == 500, "16.16 s -> ms");
  check(fadeMsFromFixed(0xffffffffu) == kMaxFadeMs, "fade capped", fadeMsFromFixed(0xffffffffu));

  // Fade: ends exact, monotonic both ways, bounded steps
  for (uint32_t from : {0u, 16u, 0x666u, 0xcccu})
    for (uint32_t to : {0u, 16u, 0x800u, 0xcccu})
      for (uint64_t period : {0ull, 1ull, 300ull, 5000ull}) {
        uint32_t prev = fadeLevel(from, to, 0, period);
        check(period == 0 ? prev == to : prev == from, "fade start", prev, from);
        unsigned steps = 0;
        for (uint64_t t = kStepMs; t < period + kStepMs; t += kStepMs) {
          const uint32_t v = fadeLevel(from, to, t, period);
          check(to >= from ? v >= prev && v <= to : v <= prev && v >= to, "fade monotonic within range", v, prev);
          prev = v;
          steps = steps + 1;
        }
        check(fadeLevel(from, to, period, period) == to && fadeLevel(from, to, period + 99, period) == to, "fade end");
        check(steps <= period / kStepMs + 1, "steps", steps);
      }

  // Register helpers
  check(scheduleLine((1614u << 16) | 14u, (1799u << 16) | 900u) == 1615, "schedule line = 1 + vsw + vbp + vactive");
  check(scheduleLine(0, 0) == 0 && scheduleLine((14u << 16) | 14u, 1799u << 16) == 0 && scheduleLine((1799u << 16) | 14u, 1799u << 16) == 0,
        "implausible timing refused");
  check(dmaCtrlFor(0xffffffffu) == (0xffffffffu & ~(kDmaBroadcast | kDmaMaster | kDmaLpm)), "dma ctrl");
  check(dmaCtrlFor(0) == kDmaEmbedded, "embedded");
  check(scheduleFor(0x0000ffffu, 1615) == (kScheduleEnable | 1615u), "schedule replaces the line (Linux only ORs it)");
  check(ackDmaDone(0xffffffffu) == (kIntEnables | 1u), "ack clears DMA_DONE only, keeps enables", ackDmaDone(0xffffffffu));
  check(!(ackDmaDone(0xffffffffu) & (1u << 24)), "error status never written");
  check(cbcrRunning(0) && !cbcrRunning(0x80000000u) && cbcrRunning(0x80000003u), "cbcr");
  check(readiness(0x20040000, kCtrlEnable | kCtrlVideo) == kSent && readiness(0x20040000, kCtrlVideo) == kCtrlOff &&
        readiness(0x20040000, kCtrlEnable | kCtrlCmd) == kNotVideo && readiness(0x20010000, 3) == kBadVersion &&
        readiness(0x10000000, 3) == kBadVersion, "readiness");

  // Sequence, good path
  {
    FakeDsi f;
    f.bootLoaderState();
    const uint32_t ctrl0 = f.r(rCtrl), trig0 = f.r(rTrigCtrl), sched0 = f.r(rDmaSchedule), mask0 = f.r(rErrIntMask0);
    Diag d;
    const int r = sendBrightness(f, 0x123, d);
    check(r == kSent, "sent", r);
    check(f.triggered, "triggered");
    check(d.line == 1615, "line", d.line);
    const uint32_t *t = f.atTrigger;
    check(t[rCtrl / 4] == (ctrl0 | kCtrlCmd), "command engine on at trigger", t[rCtrl / 4]);
    check((t[rTrigCtrl / 4] & 7) == kTrigSw, "SW trigger at trigger");
    check(t[rTpgCtrl / 4] == kTpgCmdFifo, "TPG FIFO mode at trigger");
    check(t[rDmaLength / 4] == 8, "length 8");
    check(t[rDmaCtrl / 4] == kDmaEmbedded, "DMA ctrl embedded, HS, no broadcast", t[rDmaCtrl / 4]);
    check(t[rDmaSchedule / 4] == (kScheduleEnable | 1615u), "scheduled into the vertical front porch", t[rDmaSchedule / 4]);
    check((t[rErrIntMask0 / 4] & kOverflowMask) == kOverflowMask, "overflow masked during the transfer");
    check(!(t[rIntCtrl / 4] & kIntDmaDone), "stale DMA_DONE cleared before trigger");
    check((t[rIntCtrl / 4] & (1u << 24)) && (t[rIntCtrl / 4] & (1u << 16)), "other status bits untouched");
    check(!(t[rIntCtrl / 4] & (1u << 1)), "DMA_DONE interrupt never enabled");
    encodeBrightness(0x123, w);
    check(f.fifoCount == 0, "FIFO reset after the transfer");
    // FIFO content at trigger: find the two FIFO writes in the log
    unsigned n = 0; uint32_t seen[4] = {};
    for (unsigned i = 0; i < f.writes && i < 256; i++) if (f.wrOff[i] == rTpgFifo && n < 4) seen[n++] = f.wrVal[i];
    check(n == 2 && seen[0] == w[0] && seen[1] == w[1], "exactly the two packet dwords", seen[0], seen[1]);
    // the reset pulse precedes the FIFO words, the trigger is the last write before the poll
    int resetAt = -1, fifoAt = -1, trigAt = -1, lenAt = -1;
    for (unsigned i = 0; i < f.writes && i < 256; i++) {
      if (f.wrOff[i] == rTpgFifoReset && f.wrVal[i] == 1 && resetAt < 0) resetAt = (int)i;
      if (f.wrOff[i] == rTpgFifo && fifoAt < 0) fifoAt = (int)i;
      if (f.wrOff[i] == rDmaLength) lenAt = (int)i;
      if (f.wrOff[i] == rSwTrigger) trigAt = (int)i;
    }
    check(resetAt >= 0 && resetAt < fifoAt && fifoAt < lenAt && lenAt < trigAt, "order: reset, FIFO, length, trigger");
    check(f.r(rCtrl) == ctrl0 && f.r(rTrigCtrl) == trig0 && f.r(rDmaSchedule) == sched0 && f.r(rErrIntMask0) == mask0,
          "ctrl, trigger, schedule and error mask restored");
    check(f.r(rTpgCtrl) == 0, "TPG off afterwards");
    check(!(f.r(rIntCtrl) & kIntDmaDone), "DMA_DONE acked");
    check(!(f.r(rFifoStatus) & kOverflowClear) && (f.r(rFifoStatus) & 0x4), "overflow sticky bits cleared, others kept");
    check((f.r(rIntCtrl) & (1u << 17)), "enables kept");
  }
  // Command engine already on and SW trigger already chosen: left as they were
  {
    FakeDsi f;
    f.bootLoaderState();
    f.r(rCtrl) |= kCtrlCmd;
    f.r(rTrigCtrl) = 0x24;
    Diag d;
    check(sendBrightness(f, 0, d) == kSent, "sent (cmd engine on)");
    check(f.r(rCtrl) & kCtrlCmd, "cmd engine left on");
    check(f.r(rTrigCtrl) == 0x24, "trigger left");
    for (unsigned i = 0; i < f.writes && i < 256; i++)
      check(f.wrOff[i] != rCtrl && f.wrOff[i] != rTrigCtrl, "no needless ctrl/trigger write", f.wrOff[i]);
  }
  // Refusals: nothing is touched (clocks off: not even read)
  {
    FakeDsi f;
    f.bootLoaderState();
    f.clocks = false;
    Diag d;
    check(sendBrightness(f, 0x100, d) == kClocksOff && f.writes == 0, "clocks off: no access");
    f.clocks = true;
    f.r(rCtrl) = kCtrlEnable;                                  // command mode / video engine off
    check(sendBrightness(f, 0x100, d) == kNotVideo && f.writes == 0, "not video: no write");
    f.r(rCtrl) = 0;
    check(sendBrightness(f, 0x100, d) == kCtrlOff && f.writes == 0, "controller off: no write");
    f.bootLoaderState();
    f.r(rActiveV) = 0;
    check(sendBrightness(f, 0x100, d) == kBadTiming && f.writes == 0, "timing: no write");
    f.bootLoaderState();
    f.busyForever = true;
    check(sendBrightness(f, 0x100, d) == kBusy && f.writes == 0 && !f.triggered, "busy: bounded wait, no write");
    check(f.us >= kBusyTimeoutUs && f.us < kBusyTimeoutUs + 1000, "busy wait bounded", f.us);
  }
  // No DMA_DONE: bounded wait, everything restored, FIFO reset
  {
    FakeDsi f;
    f.bootLoaderState();
    f.hang = true;
    const uint32_t ctrl0 = f.r(rCtrl), sched0 = f.r(rDmaSchedule), mask0 = f.r(rErrIntMask0);
    Diag d;
    check(sendBrightness(f, 0x200, d) == kTimeout, "timeout reported");
    check(d.waitedUs >= kDoneTimeoutUs && d.waitedUs < kDoneTimeoutUs + 2000, "done wait bounded", d.waitedUs);
    check(f.r(rCtrl) == ctrl0 && f.r(rDmaSchedule) == sched0 && f.r(rErrIntMask0) == mask0 && f.r(rTpgCtrl) == 0,
          "restored after timeout");
    check(f.fifoCount == 0, "FIFO reset after timeout");
  }
  // A slow DONE (the scheduled line is a frame away): still sent within the bound
  {
    FakeDsi f;
    f.bootLoaderState();
    f.doneAfter = 60;                                           // 40 x 25 us spin, then 1 ms sleeps
    Diag d;
    check(sendBrightness(f, 0x300, d) == kSent && d.waitedUs < kDoneTimeoutUs, "slow done", d.waitedUs);
  }

  // Rate limit: burst, then one per refill period, dropped lines counted
  {
    LogGate g;
    unsigned ok = 0;
    for (unsigned i = 0; i < 100; i++) ok += g.allow(1000000 + i);
    check(ok == LogGate::kBurst, "burst", ok);
    check(g.takeDropped() == 100 - LogGate::kBurst && g.takeDropped() == 0, "dropped counted once");
    check(!g.allow(1000000 + LogGate::kRefillMs - 1) && g.allow(1000000 + LogGate::kRefillMs + 10), "refill");
    unsigned perMinute = 0;
    for (uint64_t t = 2000000; t < 2000000 + 60000; t += 5) perMinute += g.allow(t);
    check(perMinute <= LogGate::kBurst + 60000 / LogGate::kRefillMs, "lines per minute bounded", perMinute);
    check(g.allow(0) || true, "clock going back does not crash");
  }

  printf("%s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
  return failures ? 1 : 0;
}
