// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of GuampAudioCore.h (the plug-in's clock, volume law, Float32 -> s16 and staging FIFO) and of the shared ABI of
// guamp_audio_dev.h. Built and run by check-guamp-audio.sh with clang (ASan/UBSan) and gcc.
#include "../GuampAudioCore.h"
#include "../guamp_audio_dev.h"
#include "../GuampMicCore.h"
#include "../guamp_mic_dev.h"
#include <stdio.h>
#include <stdlib.h>

static int failures, checks;
#define CHECK(cond)                                                                    \
  do {                                                                                 \
    checks++;                                                                          \
    if (!(cond)) { failures++; fprintf(stderr, "%s:%d: FAIL %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

static void testClock(void) {
  // Apple arm64 timebase (24 MHz): numer 125, denom 3 -> 41.67 ns per tick, 500 ticks per 48 kHz frame
  const double tpf = guamp_ticks_per_frame(125, 3, 48000.0);
  CHECK(tpf > 499.999 && tpf < 500.001);
  // 1 ns ticks (host-like timebase)
  const double ns = guamp_ticks_per_frame(1, 1, 48000.0);
  CHECK(ns > 20833.33 && ns < 20833.34);

  GuampClock c;
  guamp_clock_init(&c, tpf, 16384);
  guamp_clock_start(&c, 1000);
  double t;
  uint64_t h, seed;
  guamp_clock_zero(&c, 1000, &t, &h, &seed);
  CHECK(t == 0 && h == 1000 && seed == 1);
  const uint64_t period = 16384ull * 500;   // ticks per zero time stamp period
  guamp_clock_zero(&c, 1000 + period - 1, &t, &h, &seed);
  CHECK(t == 0 && h == 1000);
  guamp_clock_zero(&c, 1000 + period, &t, &h, &seed);
  CHECK(t == 16384 && h == 1000 + period);
  // a late caller catches up one period per call, never backwards, always exactly on the grid
  double lastT = t;
  uint64_t lastH = h;
  for (int i = 0; i < 10; i++) {
    guamp_clock_zero(&c, 1000 + 5 * period + 7, &t, &h, &seed);
    CHECK(t >= lastT && h >= lastH);
    CHECK((uint64_t)t % 16384 == 0);
    CHECK(h - 1000 == (uint64_t)t / 16384 * period);
    lastT = t;
    lastH = h;
  }
  CHECK(t == 5 * 16384.0);
  // the time stamp never runs ahead of "now"
  CHECK(h <= 1000 + 5 * period + 7);
  // restart: new anchor, new seed, sample time back to 0
  guamp_clock_start(&c, 99999);
  guamp_clock_zero(&c, 99999, &t, &h, &seed);
  CHECK(t == 0 && h == 99999 && seed == 2);
  // a fractional ticks-per-frame (1 ns timebase) keeps host times within half a tick of the exact value after many periods
  guamp_clock_init(&c, ns, 16384);
  guamp_clock_start(&c, 0);
  for (int i = 0; i < 100000; i++) guamp_clock_zero(&c, UINT64_C(1) << 62, &t, &h, &seed);
  const double exact = 100000.0 * 16384.0 * (1.0e9 / 48000.0);
  CHECK(t == 100000.0 * 16384.0);
  CHECK((double)h > exact - 1.0 && (double)h < exact + 1.0);
}

static void testVolume(void) {
  CHECK(guamp_gain_q16(0.0f, -48.0f, 0.0f, 0) == 65536);
  const uint32_t half = guamp_gain_q16(-6.0206f, -48.0f, 0.0f, 0);
  CHECK(half >= 32767 && half <= 32769);
  CHECK(guamp_gain_q16(-48.0f, -48.0f, 0.0f, 0) == 0);    // bottom step = silence
  CHECK(guamp_gain_q16(-60.0f, -48.0f, 0.0f, 0) == 0);    // clamped
  CHECK(guamp_gain_q16(6.0f, -48.0f, 0.0f, 0) == 65536);  // clamped
  CHECK(guamp_gain_q16(0.0f, -48.0f, 0.0f, 1) == 0);      // mute
  CHECK(guamp_gain_q16(-45.0f, -48.0f, 0.0f, 0) > 0);     // first step above the bottom still sounds
  const float nan = strtof("nan", NULL);
  CHECK(guamp_gain_q16(nan, -48.0f, 0.0f, 0) == 0);
  CHECK(guamp_scalar_to_db(0.0f, -48.0f, 0.0f) == -48.0f);
  CHECK(guamp_scalar_to_db(1.0f, -48.0f, 0.0f) == 0.0f);
  CHECK(guamp_scalar_to_db(0.5f, -48.0f, 0.0f) == -24.0f);
  CHECK(guamp_scalar_to_db(2.0f, -48.0f, 0.0f) == 0.0f);
  CHECK(guamp_db_to_scalar(-24.0f, -48.0f, 0.0f) == 0.5f);
  CHECK(guamp_db_to_scalar(-100.0f, -48.0f, 0.0f) == 0.0f);
  for (int step = 0; step <= 16; step++) {   // the 16 iOS volume steps round-trip
    const float s = (float)step / 16.0f, back = guamp_db_to_scalar(guamp_scalar_to_db(s, -48.0f, 0.0f), -48.0f, 0.0f);
    CHECK(back > s - 1e-6f && back < s + 1e-6f);
  }
}

static void testConvert(void) {
  const float nan = strtof("nan", NULL);
  const float in[] = {0.0f, 1.0f, -1.0f, 2.0f, -2.0f, 0.5f, -0.5f, nan, 1e-6f, -1.0001f};
  int16_t out[10];
  guamp_f32_to_s16(in, out, 10, GUAMP_AUDIO_GAIN_UNITY);
  CHECK(out[0] == 0);
  CHECK(out[1] == 32767);
  CHECK(out[2] == -32767);
  CHECK(out[3] == 32767);
  CHECK(out[4] == -32768);
  CHECK(out[5] == 16384 || out[5] == 16383);
  CHECK(out[6] == -16384 || out[6] == -16383);
  CHECK(out[7] == 0);
  CHECK(out[8] == 0);
  CHECK(out[9] == -32768 || out[9] == -32767);
  guamp_f32_to_s16(in, out, 10, 0);
  for (int i = 0; i < 10; i++) CHECK(out[i] == 0);
  guamp_f32_to_s16(in, out, 2, GUAMP_AUDIO_GAIN_UNITY / 2);
  CHECK(out[1] == 16384 || out[1] == 16383);
}

// ---- FIFO against a simulated device ring -------------------------------------------------------------------------------------
typedef struct {
  uint32_t room;       // frames the device ring accepts right now
  int fail;            // return -1 (EAGAIN) instead of a short count when full
  uint64_t got;        // frames taken
  int16_t expectNext;  // left sample of the next frame (sequence check)
  int outOfOrder;
} FakeDevice;
static long fakeSink(void *ctx, const void *bytes, size_t length) {
  FakeDevice *d = ctx;
  if (length % 4) return -2;   // never a partial frame
  uint32_t frames = (uint32_t)(length / 4);
  if (!d->room && d->fail) return -1;
  if (frames > d->room) frames = d->room;
  const int16_t *s = bytes;
  for (uint32_t i = 0; i < frames; i++) {
    if (s[2 * i] != d->expectNext || s[2 * i + 1] != (int16_t)~d->expectNext) d->outOfOrder++;
    d->expectNext = (int16_t)(s[2 * i] + 1);
  }
  d->room -= frames;
  d->got += frames;
  return (long)frames * 4;
}
static void fill(int16_t *buf, uint32_t frames, int16_t *seq) {
  for (uint32_t i = 0; i < frames; i++) {
    buf[2 * i] = *seq;
    buf[2 * i + 1] = (int16_t)~*seq;
    *seq = (int16_t)(*seq + 1);
  }
}

static GuampFifo fifo;   // 16 KiB: static, like the plug-in's

static void testFifo(void) {
  static int16_t buf[8192 * 2];
  int16_t seq = 0;
  FakeDevice dev = {0, 0, 0, 0, 0};
  guamp_fifo_reset(&fifo);
  fifo.dropped = fifo.written = 0;
  // short writes: the device takes 37 frames, the rest waits in order
  fill(buf, 100, &seq);
  CHECK(guamp_fifo_push(&fifo, buf, 100) == 0);
  dev.room = 37;
  CHECK(guamp_fifo_flush(&fifo, fakeSink, &dev) == 0);
  CHECK(dev.got == 37 && guamp_fifo_used(&fifo) == 63 && fifo.written == 37);
  // EAGAIN: nothing consumed, error reported
  dev.fail = 1;
  CHECK(guamp_fifo_flush(&fifo, fakeSink, &dev) == -1);
  CHECK(guamp_fifo_used(&fifo) == 63);
  dev.fail = 0;
  dev.room = 1000;
  CHECK(guamp_fifo_flush(&fifo, fakeSink, &dev) == 0);
  CHECK(dev.got == 100 && guamp_fifo_used(&fifo) == 0 && !dev.outOfOrder);
  // wrap-around: many odd-sized pushes and flushes keep the order
  for (int i = 0; i < 1000; i++) {
    const uint32_t n = (uint32_t)(i * 37 % 701) + 1;
    fill(buf, n, &seq);
    CHECK(guamp_fifo_push(&fifo, buf, n) == 0);
    dev.room = n;
    guamp_fifo_flush(&fifo, fakeSink, &dev);
  }
  CHECK(!dev.outOfOrder && fifo.dropped == 0 && guamp_fifo_used(&fifo) == 0);
  // overflow: the device is stuck, the FIFO keeps the newest GUAMP_FIFO_FRAMES and counts the rest as dropped
  dev.room = 0;
  fill(buf, 3000, &seq);
  CHECK(guamp_fifo_push(&fifo, buf, 3000) == 0);
  fill(buf, 3000, &seq);
  CHECK(guamp_fifo_push(&fifo, buf, 3000) == 6000 - GUAMP_FIFO_FRAMES);
  CHECK(guamp_fifo_used(&fifo) == GUAMP_FIFO_FRAMES && fifo.dropped == 6000 - GUAMP_FIFO_FRAMES);
  dev.expectNext = (int16_t)(seq - GUAMP_FIFO_FRAMES);   // the oldest kept frame
  dev.room = 100000;
  guamp_fifo_flush(&fifo, fakeSink, &dev);
  CHECK(!dev.outOfOrder && guamp_fifo_used(&fifo) == 0 && dev.expectNext == seq);
  // a push larger than the FIFO keeps its own newest frames
  dev.room = 0;
  fill(buf, 8192, &seq);
  const uint64_t droppedBefore = fifo.dropped;
  CHECK(guamp_fifo_push(&fifo, buf, 8192) == 8192 - GUAMP_FIFO_FRAMES);
  CHECK(fifo.dropped - droppedBefore == 8192 - GUAMP_FIFO_FRAMES);
  dev.expectNext = (int16_t)(seq - GUAMP_FIFO_FRAMES);
  dev.room = 100000;
  guamp_fifo_flush(&fifo, fakeSink, &dev);
  CHECK(!dev.outOfOrder && dev.expectNext == seq);
}

// End to end: HAL IO cycles of 512 frames into a device ring of 4096 frames drained at the same 48 kHz with jitter. Nothing may
// be lost or reordered while the device keeps up on average.
static void testSteadyState(void) {
  static int16_t buf[512 * 2];
  int16_t seq = 0;
  FakeDevice dev = {0, 0, 0, 0, 0};
  guamp_fifo_reset(&fifo);
  fifo.dropped = fifo.written = 0;
  uint32_t ringUsed = 0;
  const uint32_t ringFrames = 4096;
  uint64_t produced = 0;
  srand(1);
  for (int cycle = 0; cycle < 20000; cycle++) {
    // device drains 512 +- 64 frames per cycle (period jitter)
    const uint32_t drain = 448 + (uint32_t)(rand() % 129);
    ringUsed = ringUsed > drain ? ringUsed - drain : 0;
    fill(buf, 512, &seq);
    produced += 512;
    guamp_fifo_push(&fifo, buf, 512);
    dev.room = ringFrames - ringUsed;
    const uint64_t before = dev.got;
    guamp_fifo_flush(&fifo, fakeSink, &dev);
    ringUsed += (uint32_t)(dev.got - before);
  }
  CHECK(!dev.outOfOrder);
  CHECK(fifo.dropped == 0);
  CHECK(dev.got + guamp_fifo_used(&fifo) == produced);
}

static void testAbi(void) {
  CHECK(sizeof(struct guamp_audio_info) == 64);
  CHECK(sizeof(struct guamp_audio_volume) == 16);
  CHECK(GUAMP_AUDIO_BYTES_PER_FRAME == GUAMP_AUDIO_CHANNELS * GUAMP_AUDIO_BITS / 8);
  CHECK(GUAMP_AUDIO_START != GUAMP_AUDIO_STOP && GUAMP_AUDIO_SET_VOLUME != GUAMP_AUDIO_SET_ROUTE);
  CHECK(GUAMP_FIFO_FRAMES && !(GUAMP_FIFO_FRAMES & (GUAMP_FIFO_FRAMES - 1)));
}

static void testMic(void) {
  const int16_t in[4] = {0, 16384, -32768, 32767};
  float out[8];
  guamp_s16_to_f32_dup(in, out, 4, 2);
  CHECK(out[0] == 0.0f && out[1] == 0.0f && out[2] == 0.5f && out[3] == 0.5f && out[4] == -1.0f && out[6] < 1.0f && out[7] > 0.999f);
  guamp_s16_to_f32_dup(in, out, 4, 1);
  CHECK(out[1] == 0.5f && out[2] == -1.0f);
  CHECK(guamp_mic_excess(0, 512) == 0 && guamp_mic_excess(512 + GUAMP_MIC_KEEP_FRAMES, 512) == 0);
  CHECK(guamp_mic_excess(16384, 512) == 16384 - 512 - GUAMP_MIC_KEEP_FRAMES);
  CHECK(sizeof(struct guamp_mic_info) == 64 && sizeof(struct guamp_mic_position) == 24);
  CHECK(GUAMP_MIC_IOC_INFO == 0x40404d01u && GUAMP_MIC_IOC_START == 0x20004d02u && GUAMP_MIC_IOC_STOP == 0x20004d03u);
  CHECK(GUAMP_MIC_IOC_POSITION == 0x40184d04u && GUAMP_MIC_IOC_GAIN == 0x80044d05u && GUAMP_MIC_IOC_SET_MIC == 0x80044d06u);
  CHECK(GUAMP_MIC_RATE == GUAMP_AUDIO_RATE && GUAMP_MIC_FRAME_BYTES == 2);
}

int main(void) {
  testMic();
  testClock();
  testVolume();
  testConvert();
  testFifo();
  testSteadyState();
  testAbi();
  printf("guamp_audio_core_test: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
