// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Himax HX83102D touch service, included by NativeEmmc.cpp after PmicService.inc (mapPhysical and the boot log of
// that file). GuampTouch.cpp reduced to what this SoC needs: FIFO mode only (no SE DMA, no SMMU bypass: the stage 2 log
// line says if the engine has its FIFO interface disabled), the finger set published through TouchState.cpp of the
// display archive (native_touch_* and native_touch_published, which NativeHidInput consumes).
// Boot argument guamp-touch=<n>: 1 read only, 2 clocks and pins, 3 SPI and chip id, 4 firmware, start and events.
// guamp-touch-panel=0 skips the read-only panel power probe (SPMI L9A/LCDB, MDSS INTF1); guamp-touch-verbose=1 logs
// every identify variant; guamp-touch-scan=1 / guamp-touch-linuxseq=1 run bit-banged diagnostics when identify fails.
// Outcome of a boot: the single "GuampTouch: RESULT ..." line (stage 3+) and the "GuampTouch: panel" line (every stage).
// Writes stay limited to the touch pins, SE2 and its GCC clock votes.
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

// PMIC service (PmicService.inc, included after this file): SPMI reads of the panel rails.
namespace pmic {
extern volatile int gArbReady;
int read8(unsigned sid, uint16_t addr, uint8_t *v);
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
volatile uint32_t gStep, gEvents, gBad, gIrqs, gReleases;
volatile int gError;

void fail(unsigned step, int code) { if (!gError) { gStep = step; gError = code ? code : -1; } IOLog("GuampTouch: step %u failed (%d)\n", step, code); }
void pinWrite(unsigned gpio, uint32_t off, uint32_t v) { wr(gTlmm, gt_pin(gpio) + off, v); }
uint32_t pinRead(unsigned gpio, uint32_t off) { return rd(gTlmm, gt_pin(gpio) + off); }
// gGpioCs: chip select driven as a plain GPIO around each frame (the pads showed the GENI CS lane never high).
bool gGpioCs = false;
uint8_t gRxOr;                                                     // OR of every byte the chip ever returned (MISO ever high?)
int busXfer(void *, const uint8_t *tx, uint8_t *rx, size_t len) {
  int r;
  if (!gGpioCs) {
    r = geni_spi_transfer(&gSpi, tx, rx, len, 0);
  } else {
    pinWrite(GT_GPIO_SPI_D, 4, 0);                                 // CS low
    IODelay(2);
    r = geni_spi_transfer(&gSpi, tx, rx, len, 0);
    IODelay(2);
    pinWrite(GT_GPIO_SPI_D, 4, 2);                                 // CS high
    IODelay(10);
  }
  if (!r && rx) for (size_t i = 0; i < len; ++i) gRxOr = static_cast<uint8_t>(gRxOr | rx[i]);
  return r;
}
void busSleep(void *, unsigned ms) { IOSleep(ms ? ms : 1); }
void busReset(void *, int level) { pinWrite(GT_GPIO_RESET, 4, level ? 2u : 0u); }

// --- read-only probes ------------------------------------------------------------------------------------------------
// Level of a pad with only the TLMM pull applied (GPIO function, input, 500 us settle), pull-up then pull-down; the pin
// configuration is restored afterwards. pu 1 / pd 0: nobody drives the line (a powered SPI slave tri-states MISO while
// CS is high). pu 0 / pd 0: the line is held low against the pull-up: the far side is unpowered (its ESD diode clamps
// the pad) or drives it. Only used while the GENI engine is idle (CS high).
struct PadFloat { uint8_t pu, pd; };
PadFloat padFloat(unsigned gpio) {
  const uint32_t saved = pinRead(gpio, 0);
  const uint32_t base = saved & ~(0x3fu | GT_PIN_OE);
  PadFloat f;
  pinWrite(gpio, 0, base | 3u); IODelay(500);
  f.pu = static_cast<uint8_t>(pinRead(gpio, 4) & 1u);
  pinWrite(gpio, 0, base | 1u); IODelay(500);
  f.pd = static_cast<uint8_t>(pinRead(gpio, 4) & 1u);
  pinWrite(gpio, 0, saved); IODelay(20);
  return f;
}

// Panel state: the HX83102D touch block is inside the display driver IC, so it needs the panel's VDDIO (L9A), the LCDB
// rails (AVDD/AVEE), the LCD reset released and (in practice) the panel initialised. All reads; MDSS is only touched
// when its interface clocks report running. guamp-touch-panel=0 skips the SPMI and MDSS reads.
struct PanelState {
  int spmi;                                                        // 0 ok, -2 PMIC service not ready, -95 skipped, else first SPMI error
  uint8_t l9Type, l9St, l9Lb, l9Ub, l9En, lcdbSts1, lcdbSts3, lcdbSts4, lcdbEn;
  uint32_t lcdRstCfg, dispAhb, mdssAhb, pclk0, intfEn, frame0, frame1;
  unsigned lcdRst, teEdges;
  int intfRead;
};
PanelState gPanel;
bool gPanelProbe = true;
void spmiGet(unsigned sid, uint16_t addr, uint8_t *v) {
  *v = 0xff;
  int r = pmic::read8(sid, addr, v);
  if (r && !gPanel.spmi) gPanel.spmi = r;
}
bool cbcrRunning(uint32_t v) { return !(v & GT_CBCR_CLK_OFF) || (v & 3u) == 3u; }   // enabled under hardware gating
void panelProbe() {
  PanelState &p = gPanel;
  memset(&p, 0, sizeof p);
  p.dispAhb = p.mdssAhb = p.pclk0 = 0xffffffffu;
  p.lcdRstCfg = pinRead(GT_GPIO_LCD_RESET, 0);
  p.lcdRst = pinRead(GT_GPIO_LCD_RESET, 4) & 1u;
  unsigned last = pinRead(GT_GPIO_LCD_TE, 4) & 1u;
  for (unsigned i = 0; i < 5000; ++i) {                           // 50 ms: a running panel toggles TE (if enabled)
    unsigned b = pinRead(GT_GPIO_LCD_TE, 4) & 1u;
    if (b != last) { ++p.teEdges; last = b; }
    IODelay(10);
  }
  if (!gPanelProbe) { p.spmi = -95; return; }
  for (unsigned i = 0; i < 300 && !pmic::gArbReady; ++i) IOSleep(10);
  if (!pmic::gArbReady) {
    p.spmi = -2;
  } else {
    spmiGet(GT_SID_PM6125, GT_L9A_BASE + GT_VREG_TYPE, &p.l9Type);
    spmiGet(GT_SID_PM6125, GT_L9A_BASE + GT_VREG_STATUS1, &p.l9St);
    spmiGet(GT_SID_PM6125, GT_L9A_BASE + GT_VREG_VSET_LB, &p.l9Lb);
    spmiGet(GT_SID_PM6125, GT_L9A_BASE + GT_VREG_VSET_UB, &p.l9Ub);
    spmiGet(GT_SID_PM6125, GT_L9A_BASE + GT_VREG_ENABLE, &p.l9En);
    spmiGet(GT_SID_PMI632_LCDB, GT_LCDB_BASE + GT_LCDB_STS1, &p.lcdbSts1);
    spmiGet(GT_SID_PMI632_LCDB, GT_LCDB_BASE + GT_LCDB_STS3, &p.lcdbSts3);
    spmiGet(GT_SID_PMI632_LCDB, GT_LCDB_BASE + GT_LCDB_STS4, &p.lcdbSts4);
    spmiGet(GT_SID_PMI632_LCDB, GT_LCDB_BASE + GT_LCDB_ENABLE, &p.lcdbEn);
  }
  p.dispAhb = rd(gGcc, GT_GCC_DISP_AHB_CBCR);
  if (!cbcrRunning(p.dispAhb)) return;                             // DISPCC itself is behind gcc_disp_ahb_clk
  volatile uint8_t *dcc = mapPhysical(GT_DISPCC_BASE, 0x3000);
  if (!dcc) return;
  p.mdssAhb = rd(dcc, GT_DISPCC_MDSS_AHB_CBCR);
  p.pclk0 = rd(dcc, GT_DISPCC_MDSS_PCLK0_CBCR);
  if (p.mdssAhb & GT_CBCR_CLK_OFF) return;                         // MDSS registers unclocked: do not touch them
  volatile uint8_t *intf = mapPhysical(GT_MDSS_INTF1 & ~0xfffULL, 0x1000);
  if (!intf) return;
  intf += GT_MDSS_INTF1 & 0xfffULL;
  p.intfEn = rd(intf, GT_INTF_TIMING_ENGINE_EN);
  p.frame0 = rd(intf, GT_INTF_FRAME_COUNT);
  IOSleep(50);
  p.frame1 = rd(intf, GT_INTF_FRAME_COUNT);
  p.intfRead = 1;
}
// 1: some probe says the panel (and so the touch block) is not powered or not running; 0: nothing seen against it.
int panelOff() {
  const PanelState &p = gPanel;
  if (!p.lcdRst) return 1;                                         // LCD (DDIC) reset held low
  if (!p.spmi && p.l9Type == 0x04 && !(p.l9En & 0x80) && !(p.l9St & 0x80)) return 1;   // VDDIO 1.8 V off
  if (!p.spmi && !(p.lcdbEn & 0x80) && !(p.lcdbSts3 & 0x80) && !(p.lcdbSts4 & 0x80)) return 1;   // AVDD/AVEE off
  if (p.pclk0 != 0xffffffffu && (p.pclk0 & GT_CBCR_CLK_OFF)) return 1;   // DSI pixel clock stopped: no video stream
  if (p.intfRead && (!(p.intfEn & 1) || p.frame0 == p.frame1)) return 1; // timing engine off or frozen
  return 0;
}
void logPanel() {
  const PanelState &p = gPanel;
  IOLog("GuampTouch: panel lcd-rst gpio82 %u (cfg %08x) te-edges %u | L9A(vddio) spmi %d type %02x st %02x vset %02x%02x en %02x | "
        "lcdb st %02x/%02x/%02x en %02x | gcc disp_ahb %08x dispcc ahb %08x pclk0 %08x intf1 %s en %x frames %u->%u | %s\n",
        p.lcdRst, p.lcdRstCfg, p.teEdges, p.spmi, p.l9Type, p.l9St, p.l9Ub, p.l9Lb, p.l9En, p.lcdbSts1, p.lcdbSts3, p.lcdbSts4,
        p.lcdbEn, p.dispAhb, p.mdssAhb, p.pclk0, p.intfRead ? "read" : "skipped", p.intfEn, p.frame0, p.frame1,
        panelOff() ? "PANEL OFF" : "panel on/unknown");
}

bool snapshot() {
  gStep = 1;
  gGcc = mapPhysical(GT_GCC_BASE, 0x80000);
  gTlmm = mapPhysical(GT_TLMM_WEST, GT_TLMM_WEST_SIZE);
  if (!gGcc || !gTlmm) { fail(1, -12); return false; }
  unsigned panelArg = 1;
  PE_parse_boot_argn("guamp-touch-panel", &panelArg, sizeof(panelArg));
  gPanelProbe = panelArg != 0;
  gDfs = (rd(gGcc, GT_S2_RCG + GT_RCG_DFSR_CMD) & 1) != 0;
  for (unsigned l = 0; l < GT_DFS_LEVELS; ++l)
    gLevelHz[l] = gDfs ? gt_level_hz(rd(gGcc, GT_S2_RCG + GT_RCG_PERF_CFG(l)), rd(gGcc, GT_S2_RCG + GT_RCG_PERF_M(l)),
                                     rd(gGcc, GT_S2_RCG + GT_RCG_PERF_N(l))) : 0;
  IOLog("GuampTouch: gcc vote %08x s2 %08x dfs %d level0 %llu, gpio6 %08x gpio66 %08x/%x gpio67 %08x/%x\n", rd(gGcc, GT_GCC_VOTE_ENABLE),
        rd(gGcc, GT_HALT_S2), gDfs, (unsigned long long)gLevelHz[0], pinRead(GT_GPIO_SPI_A, 0), pinRead(GT_GPIO_RESET, 0),
        pinRead(GT_GPIO_RESET, 4), pinRead(GT_GPIO_IRQ, 0), pinRead(GT_GPIO_IRQ, 4));
  panelProbe();
  logPanel();
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
  // qupv3_se2_spi_active (bengal-pinctrl.dtsi): qup2, 6 mA, bias-disable. ts_reset_active / ts_int_active
  // (bengal-guam-touchscreen-overlay.dtsi): gpio, 8 mA, bias-pull-up; reset driven high (himax_gpio_power_config).
  const uint32_t spi = gt_pin_cfg(GT_FUNC_QUP2, GT_DRIVE_6MA, 0);
  pinWrite(GT_GPIO_SPI_A, 0, spi); pinWrite(GT_GPIO_SPI_B, 0, spi); pinWrite(GT_GPIO_SPI_C, 0, spi); pinWrite(GT_GPIO_SPI_D, 0, spi);
  pinWrite(GT_GPIO_RESET, 4, 2u);
  pinWrite(GT_GPIO_RESET, 0, gt_pin_cfg(0, GT_DRIVE_8MA, 3) | GT_PIN_OE);
  pinWrite(GT_GPIO_IRQ, 0, gt_pin_cfg(0, GT_DRIVE_8MA, 3));
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

// Identify attempts, Linux order first. himax_platform.c:1035 sets spi->mode = SPI_MODE_3 in probe (the DT spi-cpha,
// mode 1, is overridden); hx83102_pin_reset() holds reset low 20 ms and waits 50 ms (himax_ic_HX83102.c:159).
struct Variant { unsigned mode, low, high; uint8_t skip, gpioCs; };
const Variant kVariants[] = {
  {GENI_SPI_CPHA | GENI_SPI_CPOL, 0, 0, 1, 0},    // 0: no reset, state left by the boot firmware
  {GENI_SPI_CPHA | GENI_SPI_CPOL, 20, 50, 0, 0},  // 1: Linux exactly (mode 3, vendor reset)
  {GENI_SPI_CPHA, 20, 50, 0, 0},                  // 2: DT mode 1
  {0, 20, 50, 0, 0},                              // 3: mode 0
  {GENI_SPI_CPOL, 20, 50, 0, 0},                  // 4: mode 2
  {GENI_SPI_CPHA | GENI_SPI_CPOL, 50, 300, 0, 0}, // 5: long reset
  {GENI_SPI_CPHA | GENI_SPI_CPOL, 20, 50, 0, 1},  // 6: chip select as GPIO
  {GENI_SPI_CPHA, 20, 50, 0, 1},                  // 7: chip select as GPIO, mode 1
};
const unsigned kVariantCount = sizeof kVariants / sizeof kVariants[0];

// guamp-touch-scan=1: bit-banged pin-role scan. Under Linux (TWRP) the chip answers, and in XNU it drives its IRQ low
// (powered), so the failure is in our transaction. Every assignment of gpio6/7/71/80 to MISO/MOSI/CLK/CS (24) is tried
// in the four SPI modes at ~100 kHz with the Himax bus read F3 00 + 4 bytes; any non-zero MISO byte is logged.
// The four pads are restored to their saved configuration afterwards.
uint8_t bbByte(unsigned miso, unsigned mosi, unsigned clk, unsigned mode, uint8_t out) {
  const unsigned cpol = (mode >> 1) & 1u, cpha = mode & 1u;
  uint8_t in = 0;
  for (int bit = 7; bit >= 0; --bit) {
    const unsigned b = (out >> bit) & 1u;
    if (!cpha) {
      pinWrite(mosi, 4, b ? 2u : 0u); IODelay(5);
      pinWrite(clk, 4, cpol ? 0u : 2u); IODelay(2);
      in = static_cast<uint8_t>((in << 1) | (pinRead(miso, 4) & 1u)); IODelay(3);
      pinWrite(clk, 4, cpol ? 2u : 0u);
    } else {
      pinWrite(clk, 4, cpol ? 0u : 2u);
      pinWrite(mosi, 4, b ? 2u : 0u); IODelay(5);
      pinWrite(clk, 4, cpol ? 2u : 0u); IODelay(2);
      in = static_cast<uint8_t>((in << 1) | (pinRead(miso, 4) & 1u)); IODelay(3);
    }
  }
  return in;
}
void pinScan() {
  unsigned scan = 0;
  PE_parse_boot_argn("guamp-touch-scan", &scan, sizeof(scan));
  if (scan != 1) return;
  static const unsigned pins[4] = {GT_GPIO_SPI_A, GT_GPIO_SPI_B, GT_GPIO_SPI_C, GT_GPIO_SPI_D};
  uint32_t saved[4];
  for (unsigned i = 0; i < 4; ++i) saved[i] = pinRead(pins[i], 0);
  unsigned hits = 0;
  for (unsigned a = 0; a < 4; ++a) for (unsigned b = 0; b < 4; ++b) for (unsigned c = 0; c < 4; ++c) {
    if (a == b || a == c || b == c) continue;
    const unsigned d = 6 - a - b - c;
    const unsigned miso = pins[a], mosi = pins[b], clk = pins[c], cs = pins[d];
    for (unsigned mode = 0; mode < 4; ++mode) {
      pinWrite(miso, 0, gt_pin_cfg(0, GT_DRIVE_6MA, 3));
      pinWrite(cs, 4, 2u); pinWrite(cs, 0, gt_pin_cfg(0, GT_DRIVE_6MA, 0) | GT_PIN_OE);
      pinWrite(clk, 4, (mode & 2u) ? 2u : 0u); pinWrite(clk, 0, gt_pin_cfg(0, GT_DRIVE_6MA, 0) | GT_PIN_OE);
      pinWrite(mosi, 4, 0u); pinWrite(mosi, 0, gt_pin_cfg(0, GT_DRIVE_6MA, 0) | GT_PIN_OE);
      IODelay(50);
      pinWrite(cs, 4, 0u); IODelay(10);
      bbByte(miso, mosi, clk, mode, 0xf3); bbByte(miso, mosi, clk, mode, 0x00);
      uint8_t rx[4], any = 0;
      for (unsigned i = 0; i < 4; ++i) { rx[i] = bbByte(miso, mosi, clk, mode, 0x00); any = static_cast<uint8_t>(any | rx[i]); }
      pinWrite(cs, 4, 2u); IODelay(20);
      if (any) {
        hits = hits + 1;
        IOLog("GuampTouch: scan miso %u mosi %u clk %u cs %u mode %u rx %02x %02x %02x %02x\n", miso, mosi, clk, cs, mode,
              rx[0], rx[1], rx[2], rx[3]);
      }
    }
  }
  for (unsigned i = 0; i < 4; ++i) pinWrite(pins[i], 0, saved[i]);
  IOLog("GuampTouch: scan done, %u of 96 combinations got MISO activity\n", hits);
}

// guamp-touch-linuxseq=1: the Linux probe that works under TWRP, replayed byte for byte by a GPIO bit-banged SPI master
// at ~60 kHz, first in SPI mode 3 (himax_platform.c:1035 sets spi->mode = SPI_MODE_3; spi-geni-qcom applies it per
// message), then in mode 0. Sequence: himax_gpio_power_config (reset low, 2 ms, high), hx83102_chip_detect:
// hx83102_pin_reset (low 20 ms, high 50 ms), bus read 0x13 (1 byte, ignored), hx83102_sense_off(false) (FW_ISR
// 0x9000005C = A5 without burst enable, 20 ms, central state 0x900000A8 [0x9000005C too if it reads 05]; then up to 16
// times: password 0x31 = 27, 0x32 = 95, read 0x900000A8; 0C -> TCON/ADC reset and done, else 10 ms and pin reset), then
// the id at 0x900000D0, up to 5 reads (Linux reads it only after a successful sense_off: here once anyway).
// hx83102_register_read (4 bytes): F2 13 31, F2 0D 10, F2 00 <addr le>, F2 0C 00, F3 08 00 + 4 bytes. Each
// himax_bus_read/write is one spi_message: CS stays low across its command and rx transfers (no cs_change; spi-geni-qcom
// marks every transfer but the last FRAGMENTATION), MOSI 0 during the rx transfer, CS high between messages.
// Log: "L<mode>.<n> tx <bytes> rx <bytes clocked in> m <MISO right after CS low, then after each byte>", at most 26
// lines a pass (safe-mode retries after the first only when MISO moves), a line per pass, one RESULT line. MISO has the
// TLMM pull-up: a tri-stated MISO reads ff. Expected under Linux: A8 reads 0c, D0 reads xx 2d 10 83. The SPI pads and
// the reset pad are restored afterwards.
struct LsPass { unsigned mode, idle, smIter, ones, samples; uint8_t st, sm, rxOr, rxAnd, id[4]; bool safe; };
struct LsBus { unsigned mode, half, seq, lines, budget; bool verbose; LsPass *p; };
LsBus gLs;

unsigned lsMiso() { return pinRead(GT_GPIO_SPI_A, 4) & 1u; }
void lsClk(unsigned high) { pinWrite(GT_GPIO_SPI_C, 4, high ? 2u : 0u); }
uint8_t lsByte(uint8_t out) {
  const unsigned cpol = (gLs.mode >> 1) & 1u, cpha = gLs.mode & 1u, h = gLs.half;
  unsigned in = 0;
  for (unsigned i = 0; i < 8; ++i) {
    const unsigned b = (out >> (7 - i)) & 1u;
    if (!cpha) {                                                   // set up before the leading edge, sampled on it
      pinWrite(GT_GPIO_SPI_B, 4, b ? 2u : 0u); IODelay(h);
      lsClk(cpol ^ 1u);
      in = (in << 1) | lsMiso(); IODelay(h);
      lsClk(cpol);
    } else {                                                       // shifted on the leading edge, sampled on the trailing
      lsClk(cpol ^ 1u);
      pinWrite(GT_GPIO_SPI_B, 4, b ? 2u : 0u); IODelay(h);
      in = (in << 1) | lsMiso();
      lsClk(cpol); IODelay(h);
    }
  }
  return static_cast<uint8_t>(in);
}
void lsHex(char *o, const uint8_t *p, unsigned n) {
  static const char k[] = "0123456789abcdef";
  for (unsigned i = 0; i < n; ++i) { o[2 * i] = k[p[i] >> 4]; o[2 * i + 1] = k[p[i] & 15u]; }
  o[2 * n] = 0;
}
// One spi_message: ncmd bytes (tx transfer), then ndata bytes clocked with MOSI 0 (rx transfer), CS low across both.
void lsMsg(const uint8_t *cmd, unsigned ncmd, uint8_t *data, unsigned ndata) {
  uint8_t tx[12] = {}, rx[12] = {};
  char m[14], th[25], rh[25];
  const unsigned n = ncmd + ndata;                                 // at most 10 (flash_write_burst)
  for (unsigned i = 0; i < ncmd; ++i) tx[i] = cmd[i];
  pinWrite(GT_GPIO_SPI_D, 4, 0u); IODelay(2 * gLs.half);           // CS low
  m[0] = static_cast<char>('0' + lsMiso());
  for (unsigned i = 0; i < n; ++i) {
    rx[i] = lsByte(tx[i]);
    m[i + 1] = static_cast<char>('0' + lsMiso());
    IODelay(gLs.half);
  }
  m[n + 1] = 0;
  IODelay(2 * gLs.half);
  pinWrite(GT_GPIO_SPI_D, 4, 2u); IODelay(20);                    // CS high between messages
  LsPass &p = *gLs.p;
  bool moved = false;
  for (unsigned i = 0; i < n; ++i) {
    p.rxOr = static_cast<uint8_t>(p.rxOr | rx[i]);
    p.rxAnd = static_cast<uint8_t>(p.rxAnd & rx[i]);
    if (rx[i]) moved = true;
  }
  for (unsigned i = 0; i <= n; ++i) if (m[i] == '1') { p.ones = p.ones + 1; moved = true; }
  p.samples = p.samples + n + 1;
  for (unsigned i = 0; i < ndata; ++i) data[i] = rx[ncmd + i];
  gLs.seq = gLs.seq + 1;
  if (gLs.lines < gLs.budget && (gLs.verbose || moved)) {
    lsHex(th, tx, n); lsHex(rh, rx, n);
    IOLog("GuampTouch: L%u.%02u tx %s rx %s m %s\n", gLs.mode, gLs.seq, th, rh, m);
    gLs.lines = gLs.lines + 1;
  }
}
void lsWrite(uint8_t reg, const uint8_t *d, unsigned n) {         // himax_bus_write: F2 reg data
  uint8_t b[10] = {0xF2, reg};
  for (unsigned i = 0; i < n && i < 8; ++i) b[2 + i] = d[i];
  lsMsg(b, n + 2, nullptr, 0);
}
void lsRead(uint8_t reg, uint8_t *d, unsigned n) {                // himax_bus_read: F3 reg 00, then n bytes
  const uint8_t c[3] = {0xF3, reg, 0x00};
  lsMsg(c, 3, d, n);
}
void lsLe(uint8_t *b, uint32_t v) {
  b[0] = static_cast<uint8_t>(v); b[1] = static_cast<uint8_t>(v >> 8);
  b[2] = static_cast<uint8_t>(v >> 16); b[3] = static_cast<uint8_t>(v >> 24);
}
void lsWriteBurst(uint32_t addr, uint32_t val) {                  // hx83102_flash_write_burst: F2 00 addr data
  uint8_t b[8];
  lsLe(b, addr); lsLe(b + 4, val);
  lsWrite(0x00, b, 8);
}
void lsRegRead(uint32_t addr, uint8_t *d) {                       // hx83102_register_read(addr, 4)
  uint8_t v = 0x31, a[4];
  lsWrite(0x13, &v, 1);                                            // hx83102_burst_enable(0)
  v = 0x10; lsWrite(0x0D, &v, 1);
  lsLe(a, addr); lsWrite(0x00, a, 4);
  v = 0x00; lsWrite(0x0C, &v, 1);
  lsRead(0x08, d, 4);
}
void lsPinReset() { pinWrite(GT_GPIO_RESET, 4, 0u); IOSleep(20); pinWrite(GT_GPIO_RESET, 4, 2u); IOSleep(50); }
bool lsIdOk(const uint8_t *id) {
  return id[3] == 0x83 && id[2] == 0x10 && (id[1] == 0x2A || id[1] == 0x2B || id[1] == 0x2D || id[1] == 0x2E);
}
void lsRun(LsPass &p, unsigned mode) {
  memset(&p, 0, sizeof p);
  p.mode = mode; p.rxAnd = 0xff;
  gLs.mode = mode; gLs.seq = 0; gLs.lines = 0; gLs.verbose = true; gLs.p = &p;
  const uint32_t out = gt_pin_cfg(0, GT_DRIVE_6MA, 0) | GT_PIN_OE;
  pinWrite(GT_GPIO_SPI_D, 4, 2u); pinWrite(GT_GPIO_SPI_D, 0, out);                 // CS idle high
  pinWrite(GT_GPIO_SPI_C, 4, (mode & 2u) ? 2u : 0u); pinWrite(GT_GPIO_SPI_C, 0, out); // CLK idle at CPOL
  pinWrite(GT_GPIO_SPI_B, 4, 0u); pinWrite(GT_GPIO_SPI_B, 0, out);
  pinWrite(GT_GPIO_SPI_A, 0, gt_pin_cfg(0, GT_DRIVE_6MA, 3));                     // MISO input, pull-up
  IODelay(200);
  pinWrite(GT_GPIO_RESET, 4, 0u); IOSleep(2); pinWrite(GT_GPIO_RESET, 4, 2u);       // himax_gpio_power_config
  lsPinReset();                                                                    // hx83102_chip_detect
  p.idle = lsMiso();
  IOLog("GuampTouch: L%u pass: SPI mode %u ~%u kHz, CS high MISO %u (pull-up) irq %u reset %u\n", mode, mode,
        500 / gLs.half, p.idle, pinRead(GT_GPIO_IRQ, 4) & 1u, pinRead(GT_GPIO_RESET, 4) & 1u);
  uint8_t d[4] = {};
  lsRead(0x13, d, 1);
  lsWriteBurst(0x9000005Cu, 0xA5u);                                // sense_off: first loop, once (check_en false)
  IOSleep(20);
  lsRegRead(0x900000A8u, d);
  p.st = d[0];
  if (d[0] == 0x05) lsRegRead(0x9000005Cu, d);
  for (unsigned cnt = 0; cnt < 16 && !p.safe; ++cnt) {             // do { ... } while (cnt++ < 15)
    gLs.verbose = cnt == 0;
    uint8_t v = 0x27;
    lsWrite(0x31, &v, 1);
    v = 0x95; lsWrite(0x32, &v, 1);
    lsRegRead(0x900000A8u, d);
    p.sm = d[0]; p.smIter = cnt + 1;
    if (d[0] == 0x0C) {
      gLs.verbose = true;
      lsWriteBurst(0x80020020u, 0); IODelay(1000); lsWriteBurst(0x80020020u, 1);   // TCON reset
      lsWriteBurst(0x80020094u, 0); IODelay(1000); lsWriteBurst(0x80020094u, 1);   // ADC reset
      p.safe = true;
    } else {
      IOSleep(10);
      lsPinReset();
    }
  }
  gLs.verbose = true;
  for (unsigned i = 0; i < (p.safe ? 5u : 1u); ++i) {
    lsRegRead(0x900000D0u, p.id);
    if (lsIdOk(p.id)) break;
  }
  IOLog("GuampTouch: L%u done: status %02x safe-mode %02x after %u%s, id %02x %02x %02x, rx-or %02x rx-and %02x, MISO high %u/%u\n",
        mode, p.st, p.sm, p.smIter, p.safe ? " (entered)" : "", p.id[3], p.id[2], p.id[1], p.rxOr, p.rxAnd, p.ones, p.samples);
}
void linuxSeq() {
  unsigned on = 0;
  PE_parse_boot_argn("guamp-touch-linuxseq", &on, sizeof(on));
  if (on != 1) return;
  static const unsigned pins[5] = {GT_GPIO_SPI_A, GT_GPIO_SPI_B, GT_GPIO_SPI_C, GT_GPIO_SPI_D, GT_GPIO_RESET};
  uint32_t cfg[5], io[5];
  for (unsigned i = 0; i < 5; ++i) { cfg[i] = pinRead(pins[i], 0); io[i] = pinRead(pins[i], 4); }
  gLs.half = 8;                                                    // 8 us half period: ~60 kHz
  gLs.budget = 26;
  LsPass p3, p0;
  lsRun(p3, 3);
  lsRun(p0, 0);
  for (unsigned i = 0; i < 5; ++i) { pinWrite(pins[i], 4, io[i] & 2u); pinWrite(pins[i], 0, cfg[i]); }
  IODelay(20);
  const LsPass *ok = lsIdOk(p3.id) ? &p3 : lsIdOk(p0.id) ? &p0 : nullptr;
  const bool floats = p3.rxAnd == 0xff && p3.ones == p3.samples && p0.rxAnd == 0xff && p0.ones == p0.samples;
  const bool moves = p3.rxOr || p0.rxOr || p3.ones || p0.ones;
  const char *verdict =
      ok ? (ok == &p3 ? "OK in mode 3: chip and pin roles fine, our GENI transfer differs from Linux (clock/CS/config)"
                      : "OK only in mode 0: chip and pin roles fine, the phase is wrong")
      : floats ? "MISO FLOATS: pulled up and never driven, the chip never sees CS (gpio80) or is not in SPI mode"
      : moves ? "MISO MOVES, no valid id: compare the L lines with the expected replies (A8 0c, D0 xx 2d 10 83)"
      : "SILENT: MISO low in every sample, also with CS high and the pull-up on: gpio6 is driven low, not a tri-stated MISO";
  IOLog("GuampTouch: L RESULT m3 st %02x sm %02x id %02x%02x%02x or %02x hi %u/%u | m0 st %02x sm %02x id %02x%02x%02x or %02x "
        "hi %u/%u | %s\n", p3.st, p3.sm, p3.id[3], p3.id[2], p3.id[1], p3.rxOr, p3.ones, p3.samples, p0.st, p0.sm, p0.id[3],
        p0.id[2], p0.id[1], p0.rxOr, p0.ones, p0.samples, verdict);
}

bool openBus() {
  gStep = 3;
  unsigned verbose = 0;
  PE_parse_boot_argn("guamp-touch-verbose", &verbose, sizeof(verbose));
  gSpi.poll_limit = 400000;
  gSpi.poll_only = 1;
  int r = geni_spi_init(&gSpi);
  if (r) { fail(3, r); return false; }
  struct gt_clock_choice c = {};
  if (gt_choose_clock(gLevelHz, GT_DFS_LEVELS, gSpi.oversampling, GT_SPI_MAX_HZ, &c)) { fail(3, -22); return false; }
  r = geni_spi_set_clock(&gSpi, static_cast<unsigned>(c.actual_hz), static_cast<unsigned>(c.sclk_hz), gDfs ? c.level : 0);
  if (!r) r = geni_spi_setup(&gSpi, GENI_SPI_CPHA | GENI_SPI_CPOL, 0);
  if (r) { fail(3, r); return false; }
  static const uint8_t pattern[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
  uint8_t back[8] = {};
  geni_spi_setup(&gSpi, GENI_SPI_LOOP | GENI_SPI_CPHA | GENI_SPI_CPOL, 0);
  r = geni_spi_transfer(&gSpi, pattern, back, sizeof pattern, 0);
  geni_spi_setup(&gSpi, GENI_SPI_CPHA | GENI_SPI_CPOL, 0);
  const bool loopOk = !r && !memcmp(pattern, back, sizeof pattern);
  IOLog("GuampTouch: SPI level %u div %u -> %llu Hz, loopback %s (rc %d)\n", c.level, c.div, (unsigned long long)c.actual_hz,
        loopOk ? "ok" : "BAD", r);
  if (!loopOk) { fail(31, r ? r : -5); return false; }
  gFrame = static_cast<uint8_t *>(IOMalloc(HX_WRITE_CHUNK + 8));
  gRx = static_cast<uint8_t *>(IOMalloc(HX_WRITE_CHUNK + 8));
  gCfg = static_cast<uint8_t *>(IOMalloc(HX_ZF_CFG_MAX));
  if (!gFrame || !gRx || !gCfg) { fail(3, -12); return false; }
  gBus.ctx = nullptr; gBus.xfer = busXfer; gBus.sleep_ms = busSleep; gBus.reset = busReset;
  memset(&gChip, 0, sizeof gChip);
  gChip.bus = &gBus; gChip.frame = gFrame; gChip.rx = gRx;
  // guamp-touch-lcdreset=1: redo the panel-side reset Linux performs before the touch probe (bengal-guam-display.dtsi
  // reset sequence 1/5, 0/2, 1/60 ms on gpio82), with the touch reset held low across it, then release the touch reset.
  // gpio82 is already a GPIO output (cfg 0x2c3); only its level is written. The display stays dark until re-initialised.
  unsigned lcdReset = 0;
  PE_parse_boot_argn("guamp-touch-lcdreset", &lcdReset, sizeof(lcdReset));
  if (lcdReset == 1) {
    pinWrite(GT_GPIO_RESET, 4, 0u);
    pinWrite(GT_GPIO_LCD_RESET, 4, 2u); IOSleep(5);
    pinWrite(GT_GPIO_LCD_RESET, 4, 0u); IOSleep(2);
    pinWrite(GT_GPIO_LCD_RESET, 4, 2u); IOSleep(60);
    pinWrite(GT_GPIO_RESET, 4, 2u); IOSleep(50);
    IOLog("GuampTouch: lcd reset cycle done, lcd-rst %u tp-rst %u irq %u\n", pinRead(GT_GPIO_LCD_RESET, 4) & 1u,
          pinRead(GT_GPIO_RESET, 4) & 1u, pinRead(GT_GPIO_IRQ, 4) & 1u);
  }
  // Electrical check before any chip access: engine idle, CS high, reset released.
  const PadFloat miso0 = padFloat(GT_GPIO_SPI_A), irq0 = padFloat(GT_GPIO_IRQ);
  r = HX_ERR_CHIP;
  int ok = -1, stageMax = 0;
  uint32_t failMask = 0;
  gRxOr = 0;
  for (unsigned v = 0; v < kVariantCount && r; ++v) {
    const Variant &k = kVariants[v];
    if (k.gpioCs && !gGpioCs) {                                    // gpio80 as GPIO output, idle high
      pinWrite(GT_GPIO_SPI_D, 4, 2);
      pinWrite(GT_GPIO_SPI_D, 0, gt_pin_cfg(0, GT_DRIVE_6MA, 0) | GT_PIN_OE);
      gGpioCs = true;
    }
    geni_spi_setup(&gSpi, k.mode, 0);
    gChip.reset_low_ms = k.low; gChip.reset_high_ms = k.high; gChip.reset_invert = 0; gChip.reset_skip = k.skip;
    r = hx_identify(&gChip);
    if (gChip.stage > stageMax) stageMax = gChip.stage;
    if (r) failMask |= 1u << v; else ok = static_cast<int>(v);
    if (verbose)
      IOLog("GuampTouch: variant %u mode %u rc %d stage %d dummy %02x%02x%02x%02x conti %02x incr4 %02x id %02x %02x %02x %02x central %02x\n",
            v, k.mode, r, gChip.stage, gChip.dbg_dummy[0], gChip.dbg_dummy[1], gChip.dbg_dummy[2], gChip.dbg_dummy[3],
            gChip.dbg_conti, gChip.dbg_incr4, gChip.id[3], gChip.id[2], gChip.id[1], gChip.id[0], gChip.central_state);
  }
  const PadFloat miso1 = r ? padFloat(GT_GPIO_SPI_A) : miso0;
  const int off = panelOff();
  const char *verdict = !r ? "OK: HX83102D answered"
                      : off ? "PANEL OFF: the TDDI touch block is unpowered; needs the display powered/initialised, not an SPI fix"
                      : gRxOr ? "CHIP ANSWERS GARBAGE: MISO moves but no valid reply (mode/framing)"
                      : (!miso0.pu && !miso1.pu) ? "MISO CLAMPED LOW against pull-up with CS high: touch IO side unpowered or held in reset"
                      : "CHIP SILENT: MISO floats (pull-up reads 1) but the chip never drives it (not selected, in reset, wrong CS)";
  // The one line that tells the outcome of this boot.
  IOLog("GuampTouch: RESULT stage %d rc %d variant %d mode %u id %02x%02x%02x fail-mask %02x rx-or %02x | miso pu/pd %u%u after %u%u "
        "irq pu/pd %u%u | panel %s | %s\n",
        stageMax, r, ok, ok >= 0 ? kVariants[ok].mode : 0u, gChip.id[3], gChip.id[2], gChip.id[1], failMask, gRxOr,
        miso0.pu, miso0.pd, miso1.pu, miso1.pd, irq0.pu, irq0.pd, off ? "OFF" : "on/unknown", verdict);
  if (r) pinScan();
  if (r) linuxSeq();
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

// Raw coordinates are panel pixels, the same ones Linux reports (himax_common.c passes x = buf[0]<<8|buf[1],
// y = buf[2]<<8|buf[3] unchanged, abs range 0..719 x 0..1599 from himax,panel-coords <0 720 0 1600>, no swap or flip;
// the panel node is a plain 720x1600 scanout): x left to right, y top to bottom, as the framebuffer. Only 0..max-1 is
// on the glass: the chin under the panel reports y = 1600 (Linux drops it, here it is clamped to the bottom row).
// Edge starts: NativeHidInput marks a contact as coming from an edge (kIOHIDDigitizerEventFromEdgeTip, which every
// SpringBoard screen-edge gesture needs) only when its first position lies within 48/2032 of the HID y axis (38 px of
// 1600) or 48/3048 of x (11 px of 720). The USB mirror's mouse starts at row 0, a finger on the top edge of this panel
// does not: the pull then reaches the Home screen (Spotlight) instead of Notification / Control Center. A contact that
// begins inside an edge band is moved onto that edge and the shift fades out over the next kEdgeFade pixels of travel
// (continuous and monotonic; the finger and the point agree again once it has left the band). Bands in panel pixels:
// guamp-touch-edge-top (default 88, the 44 pt status bar), guamp-touch-edge-side (24), guamp-touch-edge-bottom (40; the
// chin itself reports the bottom row). guamp-touch-flip bit 0 mirrors x, bit 1 mirrors y. guamp-touch-maplog=<n>:
// one line per contact (start and end, raw -> mapped) for the first n contacts (default 48), then every 32nd.
const int kEdgeFade = 160;
int gEdgeTop = 88, gEdgeSide = 24, gEdgeBottom = 40, gFlip = 0;
unsigned gMapLog = 48, gContacts = 0;
struct contact { bool down; unsigned n, reports; int sx, sy, smx, smy, offX, offY, rx, ry, mx, my; };
contact gContact[HX_MAX_PT];

void mapInit() {
  unsigned v;
  if (PE_parse_boot_argn("guamp-touch-edge-top", &v, sizeof(v)) && v < 128) gEdgeTop = static_cast<int>(v);
  if (PE_parse_boot_argn("guamp-touch-edge-side", &v, sizeof(v)) && v < 128) gEdgeSide = static_cast<int>(v);
  if (PE_parse_boot_argn("guamp-touch-edge-bottom", &v, sizeof(v)) && v < 128) gEdgeBottom = static_cast<int>(v);
  if (PE_parse_boot_argn("guamp-touch-flip", &v, sizeof(v))) gFlip = static_cast<int>(v & 3);
  if (PE_parse_boot_argn("guamp-touch-maplog", &v, sizeof(v))) gMapLog = v;
}

int clampTo(int v, int max) { return v < 0 ? 0 : v > max - 1 ? max - 1 : v; }
int fade(int v, int start, int off) {
  const int d = v > start ? v - start : start - v;
  return d >= kEdgeFade ? v : v - off * (kEdgeFade - d) / kEdgeFade;
}

void logContact(int slot, const contact &c, const char *what) {
  if (c.n > gMapLog && (c.n & 31)) return;
  // pt = px / 2 (360x800 pt); HID = px * 32767 / (max - 1)
  IOLog("GuampTouch: map #%u slot %d %s: start %d,%d -> %d,%d px = %d,%d pt (edge shift %d,%d), last raw %d,%d -> %d,%d px "
        "= %d,%d pt, %u reports\n", c.n, slot, what, c.sx, c.sy, c.smx, c.smy, c.smx / 2, c.smy / 2, c.offX, c.offY,
        c.rx, c.ry, c.mx, c.my, c.mx / 2, c.my / 2, c.reports);
}

void publish(const struct hx_report &r) {
  const int xmax = gChip.x_res ? gChip.x_res : GT_PANEL_X, ymax = gChip.y_res ? gChip.y_res : GT_PANEL_Y;
  int xs[HX_MAX_PT], ys[HX_MAX_PT];
  bool seen[HX_MAX_PT] = {};
  const unsigned count = r.count < HX_MAX_PT ? r.count : HX_MAX_PT;
  for (unsigned i = 0; i < count; ++i) {
    const int s = r.slot[i];
    int x = clampTo(r.point[i].x, xmax), y = clampTo(r.point[i].y, ymax);
    if (gFlip & 1) x = xmax - 1 - x;
    if (gFlip & 2) y = ymax - 1 - y;
    if (s < 0 || s >= HX_MAX_PT) { xs[i] = x; ys[i] = y; continue; }
    contact &c = gContact[s];
    seen[s] = true;
    if (!c.down) {   // the contact begins: its start decides the edge shift for its whole life
      c.down = true; c.reports = 0; c.sx = x; c.sy = y;
      gContacts = gContacts + 1; c.n = gContacts;
      c.offX = gEdgeSide && x < gEdgeSide ? x : gEdgeSide && x > xmax - 1 - gEdgeSide ? x - (xmax - 1) : 0;
      c.offY = gEdgeTop && y < gEdgeTop ? y : gEdgeBottom && y > ymax - 1 - gEdgeBottom ? y - (ymax - 1) : 0;
    }
    c.reports = c.reports + 1;
    c.rx = r.point[i].x; c.ry = r.point[i].y;
    c.mx = clampTo(c.offX ? fade(x, c.sx, c.offX) : x, xmax);
    c.my = clampTo(c.offY ? fade(y, c.sy, c.offY) : y, ymax);
    if (c.reports == 1) { c.smx = c.mx; c.smy = c.my; }
    xs[i] = c.mx; ys[i] = c.my;
  }
  native_touch_seq = native_touch_seq + 1;
  __asm__ volatile("dmb ish" ::: "memory");
  native_touch_count = count;
  for (unsigned i = 0; i < count; ++i) {
    // Same scale as the USB mirror (UsbMirror.inc touch()): pixel 0..max-1 -> 0..30479 / 0..20319, which
    // NativeHidInput turns into 0..32767 on both axes, i.e. the full 360x800 pt screen.
    native_touch_xs[i] = xs[i] * 30479 / (xmax - 1);
    native_touch_ys[i] = ys[i] * 20319 / (ymax - 1);
    native_touch_ids[i] = r.slot[i];
  }
  __asm__ volatile("dmb ish" ::: "memory");
  native_touch_seq = native_touch_seq + 1;
  native_touch_published();
  for (int s = 0; s < HX_MAX_PT; ++s) {
    if (gContact[s].down && !seen[s]) {
      gContact[s].down = false;
      logContact(s, gContact[s], "up");
    }
  }
}

void thread(void *param, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-touch");
  const unsigned stage = static_cast<unsigned>(reinterpret_cast<uintptr_t>(param));
  // guamp-touch-delay=<s>: start later, so the USB mirror (it connects after the first seconds) sees every line.
  unsigned delay = 0;
  PE_parse_boot_argn("guamp-touch-delay", &delay, sizeof(delay));
  if (delay && delay <= 600) IOSleep(delay * 1000);
  if (!snapshot() || stage < 2 || !enable() || stage < 3) return;
  if (!openBus()) {
    // The boot-time panel line is gone from kpan and the USB mirror connects later: repeat it once a minute for 15 minutes.
    for (unsigned i = 0; i < 15 && gPanelProbe; ++i) { IOSleep(60000); panelProbe(); logPanel(); }
    return;
  }
  if (stage < 4) return;
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
  mapInit();
  IOLog("GuampTouch: map %dx%d px -> HID 0..32767 (360x800 pt), edge bands top %d side %d bottom %d px, fade %d, flip %d, "
        "log %u contacts\n", gChip.x_res ? gChip.x_res : GT_PANEL_X, gChip.y_res ? gChip.y_res : GT_PANEL_Y, gEdgeTop, gEdgeSide,
        gEdgeBottom, kEdgeFade, gFlip, gMapLog);
  uint8_t ev[HX_EVENT_LEN];
  uint32_t seen = gIrqEvents;
  bool down = false;
  uint32_t polls = 0;
  uint64_t lastGood = 0, releaseTicks = 0;
  nanoseconds_to_absolutetime(100 * 1000 * 1000ULL, &releaseTicks);
  for (;;) {
    if (useIrq) {
      // Woken by the IRQ, or after 20 ms at the latest: works even if the IRQ line never fires.
      assert_wait_timeout((event_t)&gIrqEvents, THREAD_UNINT, 20, 1000 * 1000);
      if (gIrqEvents != seen) clear_wait(current_thread(), THREAD_AWAKENED);
      else thread_block(THREAD_CONTINUE_NULL);
      seen = gIrqEvents;
    } else {
      IOSleep(10);
    }
    polls = polls + 1;
    if (polls == 50 || (polls & 2047) == 0)
      IOLog("GuampTouch: polls %u irqs %u events %u bad %u released %u info %ux%u pt %u fallback %d\n", polls, gIrqs, gEvents, gBad, gReleases,
            gChip.x_res, gChip.y_res, gChip.max_pt, gChip.info_fallback);
    struct hx_report rep;
    if (hx_read_event(&gChip, ev) || hx_parse_event(ev, gChip.x_res ? gChip.x_res : GT_PANEL_X, gChip.y_res ? gChip.y_res : GT_PANEL_Y, &rep)) {
      gBad = gBad + 1;
      // A lost "finger up" report left iOS with a stuck contact: release after 100 ms without a valid report.
      if (down && mach_absolute_time() - lastGood > releaseTicks) {
        struct hx_report up = {};
        down = false;
        gReleases = gReleases + 1;
        publish(up);
      }
      continue;
    }
    lastGood = mach_absolute_time();
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
