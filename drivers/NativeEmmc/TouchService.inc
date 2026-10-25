// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Himax HX83102D touch service, included by NativeEmmc.cpp after PmicService.inc (mapPhysical and the boot log of
// that file). GuampTouch.cpp reduced to what this SoC needs: FIFO mode only (no SE DMA, no SMMU bypass: the stage 2 log
// line says if the engine has its FIFO interface disabled), the finger set published through TouchState.cpp of the
// display archive (native_touch_* and native_touch_published, which NativeHidInput consumes).
// Boot argument guamp-touch=<n>: 1 read only, 2 clocks and pins, 3 SPI and chip id, 4 firmware, start and events.
// Board facts: GuampTouchBoard.h; chip protocol: HxProtocol.c (host-tested by tests/hx_host.c).

#include <IOKit/IODeviceTreeSupport.h>   // gIODTPlane
#include "GeniSpi.h"
#include "HxProtocol.h"
#include "GuampTouchBoard.h"
#include "GeniSpiCpp.inc"
#include "HxProtocolCpp.inc"

extern "C" {
extern volatile uint32_t native_touch_seq, native_touch_count;
extern volatile int native_touch_xs[12], native_touch_ys[12], native_touch_ids[12];
void native_touch_published(void);
// Empty weak defaults: a generated firmware object (strong symbols) replaces them; the kernel links statically, so an
// undefined weak import does not link.
__attribute__((weak)) extern const uint8_t guamp_touch_firmware[1] = {0};
__attribute__((weak)) extern const uint32_t guamp_touch_firmware_size = 0;
extern int armcpu_spi_attach(uint32_t, bool, void (*)(void *), void *, uint32_t *, uint64_t *) __attribute__((weak_import));
}

namespace touch {
volatile uint8_t *gGcc, *gTlmm, *gWrap, *gSe;
uint32_t rd(volatile uint8_t *b, uint32_t o) { return *reinterpret_cast<volatile uint32_t *>(b + o); }
void wr(volatile uint8_t *b, uint32_t o, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(b + o) = v; __asm__ volatile("dsb sy" ::: "memory"); }
uint32_t seRd(void *ctx, uint32_t reg) { return rd(static_cast<volatile uint8_t *>(ctx), reg); }
void seWr(void *ctx, uint32_t reg, uint32_t v) { wr(static_cast<volatile uint8_t *>(ctx), reg, v); }
void delayUs(void *, unsigned us) { IODelay(us); }

struct geni_spi gSpi;
struct hx_bus gBus;
struct hx_chip gChip;
uint8_t *gFrame, *gRx, *gCfg;
uint64_t gLevelHz[GT_DFS_LEVELS];
bool gDfs;
volatile uint32_t gStep, gEvents, gBad, gIrqs;
volatile int gError;

void fail(unsigned step, int code) { if (!gError) { gStep = step; gError = code ? code : -1; } IOLog("GuampTouch: step %u failed (%d)\n", step, code); }
void pinWrite(unsigned gpio, uint32_t off, uint32_t v) { wr(gTlmm, gt_pin(gpio) + off, v); }
uint32_t pinRead(unsigned gpio, uint32_t off) { return rd(gTlmm, gt_pin(gpio) + off); }
int busXfer(void *, const uint8_t *tx, uint8_t *rx, size_t len) { return geni_spi_transfer(&gSpi, tx, rx, len, 0); }
void busSleep(void *, unsigned ms) { IOSleep(ms ? ms : 1); }
void busReset(void *, int level) { pinWrite(GT_GPIO_RESET, 4, level ? 2u : 0u); }

bool snapshot() {
  gStep = 1;
  gGcc = mapPhysical(GT_GCC_BASE, 0x80000);
  gTlmm = mapPhysical(GT_TLMM_WEST, 0x51000);
  if (!gGcc || !gTlmm) { fail(1, -12); return false; }
  gDfs = (rd(gGcc, GT_S2_RCG + GT_RCG_DFSR_CMD) & 1) != 0;
  for (unsigned l = 0; l < GT_DFS_LEVELS; ++l)
    gLevelHz[l] = gDfs ? gt_level_hz(rd(gGcc, GT_S2_RCG + GT_RCG_PERF_CFG(l)), rd(gGcc, GT_S2_RCG + GT_RCG_PERF_M(l)),
                                     rd(gGcc, GT_S2_RCG + GT_RCG_PERF_N(l))) : 0;
  IOLog("GuampTouch: gcc vote %08x s2 %08x dfs %d level0 %llu, gpio6 %08x gpio66 %08x gpio67 %08x\n", rd(gGcc, GT_GCC_VOTE_ENABLE),
        rd(gGcc, GT_HALT_S2), gDfs, (unsigned long long)gLevelHz[0], pinRead(GT_GPIO_SPI_A, 0), pinRead(GT_GPIO_RESET, 0), pinRead(GT_GPIO_IRQ, 0));
  return true;
}

bool enable() {
  gStep = 2;
  wr(gGcc, GT_GCC_GPLL0_ENABLE, rd(gGcc, GT_GCC_GPLL0_ENABLE) | 1u);
  for (unsigned i = 0; i < 10000 && !(rd(gGcc, GT_GCC_GPLL0_MODE) & (1u << 31)); ++i) IODelay(10);
  wr(gGcc, GT_GCC_VOTE_ENABLE, rd(gGcc, GT_GCC_VOTE_ENABLE) | GT_VOTE_ALL);
  bool ok = false;
  for (unsigned i = 0; i < 2000 && !ok; ++i) {
    uint32_t m = rd(gGcc, GT_HALT_M_AHB), s = rd(gGcc, GT_HALT_S_AHB);
    ok = (!(m & GT_CBCR_CLK_OFF) || (m & 2u)) && (!(s & GT_CBCR_CLK_OFF) || (s & 2u)) && !(rd(gGcc, GT_HALT_CORE) & GT_CBCR_CLK_OFF) &&
         !(rd(gGcc, GT_HALT_CORE_2X) & GT_CBCR_CLK_OFF) && !(rd(gGcc, GT_HALT_S2) & GT_CBCR_CLK_OFF);
    if (!ok) IODelay(10);
  }
  if (!ok) { fail(2, -110); return false; }
  if (!gDfs) {
    wr(gGcc, GT_S2_RCG + GT_RCG_CFG, 0);
    wr(gGcc, GT_S2_RCG + GT_RCG_CMD, rd(gGcc, GT_S2_RCG + GT_RCG_CMD) | 1u);
    for (unsigned i = 0; i < 1000 && (rd(gGcc, GT_S2_RCG + GT_RCG_CMD) & 1u); ++i) IODelay(1);
    gLevelHz[0] = 19200000ULL;
  }
  const uint32_t spi = gt_pin_cfg(GT_FUNC_QUP2, GT_DRIVE_6MA, 0);
  pinWrite(GT_GPIO_SPI_A, 0, spi); pinWrite(GT_GPIO_SPI_B, 0, spi); pinWrite(GT_GPIO_SPI_C, 0, spi); pinWrite(GT_GPIO_SPI_D, 0, spi);
  pinWrite(GT_GPIO_RESET, 4, 2u);
  pinWrite(GT_GPIO_RESET, 0, gt_pin_cfg(0, GT_DRIVE_6MA, 0) | GT_PIN_OE);
  pinWrite(GT_GPIO_IRQ, 0, gt_pin_cfg(0, GT_DRIVE_6MA, 3));
  IOLog("GuampTouch: pins after config: gpio6 %08x gpio7 %08x gpio71 %08x gpio80 %08x gpio66 %08x/%08x gpio67 %08x/%08x\n",
        pinRead(GT_GPIO_SPI_A, 0), pinRead(GT_GPIO_SPI_B, 0), pinRead(GT_GPIO_SPI_C, 0), pinRead(GT_GPIO_SPI_D, 0),
        pinRead(GT_GPIO_RESET, 0), pinRead(GT_GPIO_RESET, 4), pinRead(GT_GPIO_IRQ, 0), pinRead(GT_GPIO_IRQ, 4));
  gWrap = mapPhysical(GT_QUP_WRAPPER_BASE, 0x2000);
  gSe = mapPhysical(GT_SE_BASE, GT_SE_SIZE);
  if (!gWrap || !gSe) { fail(2, -12); return false; }
  gSpi.se.ctx = const_cast<uint8_t *>(gSe); gSpi.se.rd = seRd; gSpi.se.wr = seWr; gSpi.se.delay_us = delayUs;
  gSpi.wrapper.ctx = const_cast<uint8_t *>(gWrap); gSpi.wrapper.rd = seRd; gSpi.wrapper.wr = seWr; gSpi.wrapper.delay_us = delayUs;
  int r = geni_spi_read_info(&gSpi);
  IOLog("GuampTouch: qup %08x fw %08x proto %u if_disable %08x depth %u width %u\n", gSpi.info.qup_hw_version, gSpi.info.fw_revision,
        gSpi.info.protocol, gSpi.info.if_disable, gSpi.info.tx_fifo_depth, gSpi.info.fifo_width_bits);
  if (r || gSpi.info.protocol != GENI_PROTO_SPI) { fail(2, r ? r : GENI_ERR_PROTO); return false; }
  if (gSpi.info.fifo_disabled) { fail(2, GENI_ERR_NOTSUP); return false; }   // would need the SE DMA path of GuampTouch.cpp
  return true;
}

volatile uint32_t gSeenHigh, gSeenLow, gSamples;

bool openBus() {
  gStep = 3;
  gSpi.poll_limit = 400000;
  gSpi.poll_only = 1;
  int r = geni_spi_init(&gSpi);
  if (r) { fail(3, r); return false; }
  struct gt_clock_choice c = {};
  if (gt_choose_clock(gLevelHz, GT_DFS_LEVELS, gSpi.oversampling, GT_SPI_MAX_HZ, &c)) { fail(3, -22); return false; }
  r = geni_spi_set_clock(&gSpi, static_cast<unsigned>(c.actual_hz), static_cast<unsigned>(c.sclk_hz), gDfs ? c.level : 0);
  if (!r) r = geni_spi_setup(&gSpi, GENI_SPI_CPHA, 0);
  IOLog("GuampTouch: SPI level %u div %u -> %llu Hz (rc %d)\n", c.level, c.div, (unsigned long long)c.actual_hz, r);
  if (r) { fail(3, r); return false; }
  static const uint8_t pattern[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
  uint8_t back[8] = {};
  geni_spi_setup(&gSpi, GENI_SPI_LOOP | GENI_SPI_CPHA, 0);
  r = geni_spi_transfer(&gSpi, pattern, back, sizeof pattern, 0);
  geni_spi_setup(&gSpi, GENI_SPI_CPHA, 0);
  IOLog("GuampTouch: loopback rc %d rx %02x%02x%02x%02x%02x%02x%02x%02x\n", r, back[0], back[1], back[2], back[3], back[4], back[5],
        back[6], back[7]);
  if (r || memcmp(pattern, back, sizeof pattern)) { fail(31, r ? r : -5); return false; }
  gFrame = static_cast<uint8_t *>(IOMalloc(HX_WRITE_CHUNK + 8));
  gRx = static_cast<uint8_t *>(IOMalloc(HX_WRITE_CHUNK + 8));
  gCfg = static_cast<uint8_t *>(IOMalloc(HX_ZF_CFG_MAX));
  if (!gFrame || !gRx || !gCfg) { fail(3, -12); return false; }
  gBus.ctx = nullptr; gBus.xfer = busXfer; gBus.sleep_ms = busSleep; gBus.reset = busReset;
  memset(&gChip, 0, sizeof gChip);
  gChip.bus = &gBus; gChip.frame = gFrame; gChip.rx = gRx;
  // The DT says spi-cpha (mode 1); the first boot got no answer in that mode, so every mode is tried and logged with
  // the raw replies of the wake-up exchange (dummy read of 0x08, read back of 0x13/0x0D after writing 0x31/0x10).
  // Reset variants (in-cell panel: the touch reset may be shared with or gated by the display): none, vendor timing,
  // long timing, inverted polarity; each with every SPI mode.
  static const unsigned modes[4] = {GENI_SPI_CPHA, 0, GENI_SPI_CPHA | GENI_SPI_CPOL, GENI_SPI_CPOL};
  static const struct {unsigned low, high; int invert, skip;} resets[4] = {{0, 0, 0, 1}, {0, 0, 0, 0}, {50, 300, 0, 0}, {20, 100, 1, 0}};
  r = HX_ERR_CHIP;
  for (unsigned v = 0; v < 4 * 4 && r; ++v) {
    const unsigned m = v % 4;
    geni_spi_setup(&gSpi, modes[m], 0);
    gChip.reset_low_ms = resets[v / 4].low; gChip.reset_high_ms = resets[v / 4].high;
    gChip.reset_invert = resets[v / 4].invert; gChip.reset_skip = resets[v / 4].skip;
    r = hx_identify(&gChip);
    IOLog("GuampTouch: reset %u mode %u identify rc %d stage %d dummy %02x%02x%02x%02x conti %02x incr4 %02x id %02x %02x %02x %02x central %02x\n",
          v / 4, modes[m], r, gChip.stage, gChip.dbg_dummy[0], gChip.dbg_dummy[1], gChip.dbg_dummy[2], gChip.dbg_dummy[3], gChip.dbg_conti,
          gChip.dbg_incr4, gChip.id[3], gChip.id[2], gChip.id[1], gChip.id[0], gChip.central_state);
  }
  // Pad levels while a 4 KiB read runs at 1 MHz (TLMM in bit of each pin, sampled on every FIFO poll): which of
  // gpio6/7/71/80 (lanes 0..3: MISO, MOSI, CLK, CS) and the IRQ line ever read high and low. A lane that never moves is
  // not driven: the CS lane decides whether the chip is selected at all, the MISO lane whether it answers. Only after
  // a failed identify (the clock stays at 1 MHz).
  if (r) {
    static const unsigned pins[5] = {GT_GPIO_SPI_A, GT_GPIO_SPI_B, GT_GPIO_SPI_C, GT_GPIO_SPI_D, GT_GPIO_IRQ};
    gSeenHigh = gSeenLow = 0; gSamples = 0;
    geni_spi_sample_hook = [] {
      for (unsigned i = 0; i < 5; i++) {
        if (pinRead(pins[i], 4) & 1) gSeenHigh |= 1u << i; else gSeenLow |= 1u << i;
      }
      gSamples++;
    };
    geni_spi_setup(&gSpi, GENI_SPI_CPHA, 0);
    geni_spi_set_clock(&gSpi, 1000000, static_cast<unsigned>(gLevelHz[0]), 0);   // level 0 (19.2 MHz) / 19
    gFrame[0] = HX_SPI_READ; gFrame[1] = HX_BUS_RDATA;
    for (unsigned i = 2; i < 4096; i++) gFrame[i] = static_cast<uint8_t>(i * 37);
    int pr = geni_spi_transfer(&gSpi, gFrame, gRx, 4096, 0);
    geni_spi_sample_hook = nullptr;
    unsigned nonzero = 0;
    for (unsigned i = 0; i < 4096; i++) nonzero += gRx[i] != 0;
    IOLog("GuampTouch: pads during 4 KiB at 1 MHz (rc %d, %u samples, %u rx bytes not 0): high %02x low %02x (bits gpio6 7 71 80 irq67)\n",
          pr, gSamples, nonzero, gSeenHigh, gSeenLow);
    // Same read with a pull-up on the MISO pad: 0xff back = nobody drives MISO (the chip is not answering), 0x00 = the
    // line is held low (another driver, or the pad is not MISO).
    const uint32_t misoCtl = pinRead(GT_GPIO_SPI_A, 0);
    pinWrite(GT_GPIO_SPI_A, 0, (misoCtl & ~3u) | 3u);
    pr = geni_spi_transfer(&gSpi, gFrame, gRx, 64, 0);
    pinWrite(GT_GPIO_SPI_A, 0, misoCtl);
    IOLog("GuampTouch: with MISO pull-up (rc %d): rx %02x %02x %02x %02x ... %02x %02x\n", pr, gRx[0], gRx[1], gRx[2], gRx[3], gRx[62], gRx[63]);
  }
  // GENI state after the last attempt: output control, CPHA/CPOL, demux, trans cfg, IO pin levels (GENI_IOS 0x908)
  IOLog("GuampTouch: geni out %08x cpha %08x cpol %08x demux %08x inv %08x trans %08x ios %08x clk %08x sel %08x\n",
        seRd(const_cast<uint8_t *>(gSe), GENI_OUTPUT_CTRL), seRd(const_cast<uint8_t *>(gSe), SE_SPI_CPHA),
        seRd(const_cast<uint8_t *>(gSe), SE_SPI_CPOL), seRd(const_cast<uint8_t *>(gSe), SE_SPI_DEMUX_SEL),
        seRd(const_cast<uint8_t *>(gSe), SE_SPI_DEMUX_OUTPUT_INV), seRd(const_cast<uint8_t *>(gSe), SE_SPI_TRANS_CFG),
        seRd(const_cast<uint8_t *>(gSe), 0x908), seRd(const_cast<uint8_t *>(gSe), GENI_SER_M_CLK_CFG),
        seRd(const_cast<uint8_t *>(gSe), SE_GENI_CLK_SEL));
  if (r) { fail(32, r); return false; }
  return true;
}

volatile uint32_t gIrqEvents;
volatile uint32_t *gIrqPin;
void irq(void *) {
  if (!(gIrqPin[0xc / 4] & 1)) return;
  gIrqPin[0xc / 4] = 0;
  gIrqs = gIrqs + 1;
  __atomic_fetch_add(&gIrqEvents, 1, __ATOMIC_RELEASE);
  thread_wakeup((event_t)&gIrqEvents);
}
bool armIrq() {
  gIrqPin = reinterpret_cast<volatile uint32_t *>(gTlmm + gt_pin(GT_GPIO_IRQ));
  uint32_t cfg = gIrqPin[0x8 / 4];
  cfg &= ~((7u << 5) | (3u << 2) | (1u << 1) | 1u);
  cfg |= (3u << 5) | (2u << 2) | (1u << 4);
  gIrqPin[0x8 / 4] = cfg;
  gIrqPin[0xc / 4] = 0;
  __asm__ volatile("dsb sy" ::: "memory");
  if (!&armcpu_spi_attach || armcpu_spi_attach(32 + GT_TLMM_SUMMARY_SPI, false, irq, nullptr, nullptr, nullptr)) return false;
  gIrqPin[0x8 / 4] = cfg | 1u;
  return true;
}

void publish(const struct hx_report &r) {
  const int xmax = gChip.x_res ? gChip.x_res : GT_PANEL_X, ymax = gChip.y_res ? gChip.y_res : GT_PANEL_Y;
  native_touch_seq = native_touch_seq + 1;
  __asm__ volatile("dmb ish" ::: "memory");
  native_touch_count = r.count;
  for (unsigned i = 0; i < r.count && i < 12; ++i) {
    native_touch_xs[i] = r.point[i].x * 30479 / (xmax - 1);
    native_touch_ys[i] = r.point[i].y * 20319 / (ymax - 1);
    native_touch_ids[i] = r.slot[i];
  }
  __asm__ volatile("dmb ish" ::: "memory");
  native_touch_seq = native_touch_seq + 1;
  native_touch_published();
}

void thread(void *param, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-touch");
  const unsigned stage = static_cast<unsigned>(reinterpret_cast<uintptr_t>(param));
  if (!snapshot() || stage < 2 || !enable() || stage < 3 || !openBus() || stage < 4) return;
  gStep = 4;
  // The firmware is the board's vendor/firmware/hlt_Himax_firmware.bin (proprietary, kept out of the public sources):
  // make-guamp-adt.py copies it from the local tree into the ADT as /chosen/guamp-touch-firmware.
  const uint8_t *fwData = guamp_touch_firmware;
  uint32_t fwSize = guamp_touch_firmware_size;
  if (!fwSize) {
    IORegistryEntry *chosen = IORegistryEntry::fromPath("/chosen", gIODTPlane);   // kept retained: the data stays in use
    OSData *data = chosen ? OSDynamicCast(OSData, chosen->getProperty("guamp-touch-firmware")) : nullptr;
    if (data) { fwData = static_cast<const uint8_t *>(data->getBytesNoCopy()); fwSize = data->getLength(); }
  }
  if (!fwSize) { fail(4, -2); return; }
  struct hx_fw fw;
  int r = hx_fw_parse(&fw, fwData, fwSize);
  IOLog("GuampTouch: firmware %u bytes, %u partitions, overlay %d (rc %d)\n", fwSize, fw.count, fw.overlay, r);
  if (r) { fail(4, r); return; }
  r = hx_download(&gChip, &fw, gCfg);
  IOLog("GuampTouch: download rc %d stage %d cfg crc hw %08x sw %08x\n", r, gChip.stage, gChip.cfg_crc_hw, gChip.cfg_crc_sw);
  if (r) { fail(41, r); return; }
  r = hx_start(&gChip);
  IOLog("GuampTouch: start rc %d res %ux%u pt %u edge %d\n", r, gChip.x_res, gChip.y_res, gChip.max_pt, gChip.int_is_edge);
  if (r) { fail(42, r); return; }
  gStep = 5;
  const bool useIrq = armIrq();
  IOLog("GuampTouch: running, %s\n", useIrq ? "interrupt driven" : "polled every 10 ms");
  uint8_t ev[HX_EVENT_LEN];
  uint32_t seen = gIrqEvents;
  bool down = false;
  for (;;) {
    if (useIrq) {
      assert_wait((event_t)&gIrqEvents, THREAD_UNINT);
      if (gIrqEvents != seen) clear_wait(current_thread(), THREAD_AWAKENED);
      else thread_block(THREAD_CONTINUE_NULL);
      seen = gIrqEvents;
    } else {
      IOSleep(10);
    }
    struct hx_report rep;
    if (hx_read_event(&gChip, ev) || hx_parse_event(ev, gChip.x_res ? gChip.x_res : GT_PANEL_X, gChip.y_res ? gChip.y_res : GT_PANEL_Y, &rep)) {
      gBad = gBad + 1;
      continue;
    }
    if (!rep.count && !down) continue;
    down = rep.count != 0;
    gEvents = gEvents + 1;
    publish(rep);
    if ((gEvents & 63) == 1)
      IOLog("GuampTouch: %u events, %u bad, %u irqs, contact %d,%d\n", gEvents, gBad, gIrqs, rep.count ? rep.point[0].x : -1,
            rep.count ? rep.point[0].y : -1);
  }
}

void start() {
  unsigned stage = 0;
  if (!PE_parse_boot_argn("guamp-touch", &stage, sizeof(stage)) || !stage) return;
  if (stage > 4) stage = 4;
  thread_t t = nullptr;
  if (kernel_thread_start(thread, reinterpret_cast<void *>(static_cast<uintptr_t>(stage)), &t) == KERN_SUCCESS) thread_deallocate(t);
}
}  // namespace touch
