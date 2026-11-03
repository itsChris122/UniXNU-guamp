// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Himax HX83102D touch service, included by NativeEmmc.cpp after PmicService.inc (mapPhysical and the boot log of
// that file). GuampTouch.cpp reduced to what this SoC needs: FIFO mode only (no SE DMA, no SMMU bypass: the stage 2 log
// line says if the engine has its FIFO interface disabled), the finger set published through TouchState.cpp of the
// display archive (native_touch_* and native_touch_published, which NativeHidInput consumes).
// Boot argument guamp-touch=<n>: 1 read only, 2 clocks and pins, 3 SPI and chip id, 4 firmware, start and events.
// guamp-touch-panel=0 skips the read-only panel power probe (SPMI L9A/LCDB, MDSS INTF1); guamp-touch-verbose=1 logs
// every identify variant; guamp-touch-scan=1 / guamp-touch-linuxseq=1 run bit-banged diagnostics when identify fails.
// Stage 4: guamp-touch-usb=0|1 (cable state told to the firmware, default 1), guamp-touch-ovl=<partition> (boot code
// overlay, default the border one), guamp-touch-recode=0, guamp-touch-diag=<n> (raw stack of the first n interrupts and
// firmware status words, default 16, 0 off), guamp-touch-irqonly=1 (read the stack only after an interrupt).
// guamp-touch-wake=1 (default 0): tap-to-wake with the screen off, see the "wake" block before thread().
// guamp-touch-events=1|2 (default 0): la catena display -> touch di Linux (resume con download completo a ogni
// accensione del display di iOS, suspend allo spegnimento), vedi il blocco "events" dopo displayState().
// guamp-touch-rawdump=<tipo> (default 0): mappe diagnostiche mutual + self periodiche nel log, vedi namespace rawdump.
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
// Panel power cycle (GuampPanelCycle.inc, included after GuampScanout.inc): guamp-touch-panelcycle=5.
namespace panelcycle {
int run();
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
volatile uint32_t gGridDrops;   // contacts that began on the electrode grid point (guamp-touch-grid), not sent
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
  // guamp 03/11: variant 0 (no reset) answers first and left reset_skip = 1 for the whole session, so the pin reset in
  // hx_start (sense on after the download) was skipped: the chip stayed in the safe mode entered by sense_off (central
  // state 0C after start) and never ran the firmware -> no IRQ, empty event stack. Linux always resets (status 04).
  gChip.reset_skip = 0;
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
// Wake from the screen off (guamp-touch-wake=1), as himax_0flash_mmi suspend / resume with SMWP_enable (HX_CODE_OVERLAY,
// HX83102D, HX_RESUME_HW_RESET, HX_RESUME_SET_FW skipped under SMWP):
//   display off  GuampBacklight sends level 0, then the panel off-command (28/10, guamp-touch-wake-dsi=0 skips it) and
//                calls displayState(false). Here: SMWP 0x10007F10 = A55AA55A, then hx83102d_0f_overlay(2): wait for the
//                gesture request 0x11 in 0x10007FFC, write overlay 11 (gesture), reply 0x22. The interrupt stays on.
//   gesture      himax_wake_event_parse: 4 equal ids + CC 44 in the event stack (0x80 = double tap, the only one the
//                Motorola driver reports, gesture_cust_en[0] via the tap sensor; the firmware decides which it detects,
//                every listed id is accepted here and logged) -> one short Power press through the PMIC button sink
//                (gWakeRequests): backboardd wakes the display on Power exactly as on the hardware key, while a touch
//                with the display off is dropped by backboardd (no AppleMultitouch wake mode behind this digitizer).
//                The PMIC thread presses only while the display is still off, so a late gesture never locks the phone.
//   display on   GuampBacklight sends the on-command (11/29) before the first level, displayState(true). Here: ic_reset
//                (pin reset), SMWP off, cable state, hx83102d_0f_overlay(3): wait for the border request 0x55, overlay 12,
//                reply 0x66, ic_reset (sense on).
// No request within guamp-touch-wake-wait ms (default 500): guamp-touch-wake-force=1 (default) swaps from safe mode
// (sense off, overlay, reply, sense on), 0 leaves the overlay as it is.
// Default 0: the panel sleep-in over DSI while the boot loader's video stream runs and the firmware's gesture mode have
// not been tried on the device yet; a failure there costs the touch until the next swap (the Power key still wakes).
struct hx_fw gFw;
bool gFwOk = false;
volatile uint32_t gWakeArmed, gWakeDsi = 1, gDisplayOn = 1, gDisplayGen, gWakeRequests;
// GuampBacklight's work loop: the panel changed state. Wakes the touch thread, which does the swap.
void displayState(bool on) {
  if ((gDisplayOn != 0) == on) return;
  gDisplayOn = on ? 1u : 0u;
  __atomic_fetch_add(&gDisplayGen, 1, __ATOMIC_RELEASE);
  thread_wakeup((event_t)&gIrqEvents);
}
// guamp-touch-events (default 0): la catena display -> touch del kernel Motorola (TWRP: himax_v3_mmi_hx83102d, guamp-
// default.config: HX_ZERO_FLASH, HX_RESUME_SET_FW, HX_RESUME_HW_RESET, HX_SMART_WAKEUP con SMWP_enable 0, niente
// panel notifier MMI) riprodotta sugli stati del display di iOS.
// Linux: himax_fb_register (2,5 s dopo il probe) registra drm_notifier_callback sul drm_panel del dtbo ("panel" del nodo
// himax_0flash_ts). sde_kms.c _sde_kms_drm_check_dpms manda DRM_PANEL_EARLY_EVENT_BLANK in sde_kms_prepare_commit
// (PRIMA di dsi_panel_power_on / reset / on-command 11-29 e del video) e DRM_PANEL_EVENT_BLANK dopo il kickoff; per
// l HX83102D conta solo l EARLY (l altro e escluso da #if):
//   POWERDOWN  himax_common_suspend -> _himax_chip_common_suspend (SMWP 0): int disable, report_all_leave, niente reset
//              ne alimentazione; poi il pannello fa off-command 28/10 e post_unprepare.
//   UNBLANK    himax_common_resume -> himax_resume_thread_func: lavoro ritardato di 60 ms (HX83102D), in parallelo col
//              power on / reset / on-command del pannello -> _himax_chip_common_resume: se suspended == false "It had
//              entered resume, skip" (nessun resume implicito alla prima accensione dopo il probe: in TWRP il pannello
//              si accende a 2,72 s, il probe e a 4,4 s, il notifier a 7,9 s), altrimenti fp_0f_operation_dirly
//              (int disable, himax_mcu_firmware_update_0f = system reset, sense off, SRAM + CRC, config, overlay di
//              bordo, smwp/usb), fp_reload_disable = hx_dis_rload_0f (0x10007F00 <- 0x00009AA9 qualunque argomento),
//              fp_sense_on(0) (ic_reset a pin + reload_to_active), controllo 0x900000A8 == 05 (altrimenti
//              resume_update_fail e nuovo tentativo al resume dopo), fp_resend_cmd_func (SMWP off, cavo USB),
//              report_all_leave, resume_ic_action (vuota con HX_RESUME_HW_RESET), int enable.
// Qui GuampBacklight chiama displayEvent() dal suo work loop PRIMA dell on-command / del primo livello > 0 (EARLY
// UNBLANK) e prima del livello 0 (EARLY POWERDOWN); il thread del touch esegue la sequenza. 1: resume anche alla prima
// accensione di iOS (il primo livello > 0 di backboardd: il pannello di UEFI non ha mai visto la catena); 2: fedele a
// Linux (prima accensione ignorata, resume solo dopo uno spegnimento). guamp-touch-events-delay=<ms> (default 60),
// guamp-touch-events-rl=0 scrive 0 in 0x10007F00 (come hx_start) invece di 9AA9, guamp-touch-events-dsi=0 non manda
// off/on-command al pannello (default 1: 28/10 dopo il livello 0, 11/29 prima del primo livello, come guamp-touch-wake).
// Escluso con guamp-touch-wake armato (ha il suo suspend/resume con SMWP). Non tocca il ciclo del pannello.
volatile uint32_t gEventsMode, gEventsDsi = 1, gEvGen, gEvOn = 1;
void displayEvent(bool on) {
  if (!gEventsMode) return;
  gEvOn = on ? 1u : 0u;
  const uint32_t gen = __atomic_add_fetch(&gEvGen, 1, __ATOMIC_RELEASE);
  IOLog("GuampTouch: events: display %s (early, gen %u)\n", on ? "on" : "off", gen);
  thread_wakeup((event_t)&gIrqEvents);
}
unsigned gWakeWait = 500, gWakeForce = 1;
void logSwap(const char *what, int r) {
  uint32_t cs = 0;
  hx_reg_read32(&gChip, HX_ADDR_CENTRAL_STATE, &cs);
  IOLog("GuampTouch: %s: overlay %d %s (handshake seen %08x after %u ms, reply %02x read %08x) smwp %08x, central %08x\n",
        what, gChip.ovl_loaded, r == 1 ? "on request" : r == 2 ? "forced" : r == 0 ? "NOT swapped (no request)" : "FAILED",
        gChip.ovl_seen, gChip.ovl_wait_ms, gChip.ovl_reply, gChip.ovl_handshake, gChip.smwp_word, cs);
  if (r < 0) IOLog("GuampTouch: %s: error %d stage %d\n", what, r, gChip.stage);
}
void wakeSuspend() {
  hx_set_smwp(&gChip, 1);
  logSwap("suspend (display off)", hx_overlay_swap(&gChip, &gFw, gFw.ovl[1], HX_OVL_GESTURE_REQUEST, HX_OVL_GESTURE_REPLY,
                                                   0, gWakeWait, static_cast<int>(gWakeForce)));
}
void wakeResume() {
  hx_pin_reset(&gChip);
  hx_set_smwp(&gChip, 0);
  gChip.usb_word = write32_verify(&gChip, HX_ADDR_USB_DETECT, gChip.usb_in ? HX_HANDSHAKE_PWD : 0);
  logSwap("resume (display on)", hx_overlay_swap(&gChip, &gFw, gFw.ovl[2], HX_OVL_BORDER_REQUEST, HX_OVL_BORDER_REPLY,
                                                 1, gWakeWait, static_cast<int>(gWakeForce)));
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

// Firmware status words (diagnostics, guamp-touch-diag): central state 900000A8 (05 running), fw state 900000F8, ISR
// 9000005C, reload 90000048 (01EC), overlay handshake 10007FFC (11/55/99 = the firmware requests an overlay, 22/66/AA
// = reply), sorting 10007F04, SMWP 10007F10, USB 10007F38, debug message 10007F40 (4 words), raw out 800204F4.
void logStatus(uint32_t polls) {
  uint32_t cs = 0, st = 0, isr = 0, rl = 0, hs = 0, so = 0, sm = 0, usb = 0, raw = 0;
  uint8_t dbg[16] = {};
  hx_reg_read32(&gChip, HX_ADDR_CENTRAL_STATE, &cs);
  hx_reg_read32(&gChip, HX_ADDR_FW_STATE, &st);
  hx_reg_read32(&gChip, HX_ADDR_CTRL_FW_ISR, &isr);
  hx_reg_read32(&gChip, HX_ADDR_RELOAD_TO_ACTIVE, &rl);
  hx_reg_read32(&gChip, HX_ADDR_OVL_HANDSHAKE, &hs);
  hx_reg_read32(&gChip, HX_ADDR_SORTING_MODE, &so);
  hx_reg_read32(&gChip, HX_ADDR_SMWP_ENABLE, &sm);
  hx_reg_read32(&gChip, HX_ADDR_USB_DETECT, &usb);
  hx_reg_read32(&gChip, HX_ADDR_RAW_OUT_SEL, &raw);
  hx_reg_read(&gChip, HX_ADDR_FW_DBG_MSG, dbg, sizeof(dbg));
  IOLog("GuampTouch: status poll %u irqs %u: central %08x fwstate %08x isr %08x reload %08x ovl-hs %08x sort %08x smwp %08x "
        "usb %08x rawout %08x dbg %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n", polls, gIrqs, cs, st,
        isr, rl, hs, so, sm, usb, raw, dbg[0], dbg[1], dbg[2], dbg[3], dbg[4], dbg[5], dbg[6], dbg[7], dbg[8], dbg[9], dbg[10],
        dbg[11], dbg[12], dbg[13], dbg[14], dbg[15]);
}

// guamp-touch-regdump=0 turns it off (default on): the regions of the TWRP reference read (log-xnu/twrp-himax-reg-*.txt,
// /proc/android_touch/debug register reads), 64 bytes each, at the named moment, to compare with Linux byte by byte.
unsigned gRegDump = 1;
// GuampBacklight registers it once DSI0 is mapped: the panel's off-command (which 0: 28, 10), on-command (1: 11, 29)
// or both (2), with the dtbo waits, from this thread. Returns the GuampBacklightCore result (0 = sent).
int (*volatile gPanelCycle)(int which) = nullptr;
// guamp-touch-panelcycle (TDDI experiment, default 0): under Linux the panel's sleep out / display on comes from the
// kernel before the touch reset; here UEFI sent it long before. 1: cycle the panel after the download, before the
// sense on; 2: after the start, then wait 1 s for the reload markers again; 3: the TWRP suspend / resume order
// (_himax_chip_common_suspend / _resume without SMWP, HX_RESUME_SET_FW) before the download: panel off-command, touch
// reset held low, panel on-command, reset 1 / 2 ms, 0 / 5 ms, 1 / 2 ms, then the full download and start. In TWRP the
// verified working touch (S1@391,840 at 58.7 s) came after such a screen off / on cycle (second 0f at 21.6 s).
// 4: the Linux boot order (LCD reset sequence and the full on-command before the touch), see the block in thread().
// 5: the full Linux panel power cycle with the LCDB rails (video stopped, LP off-command, rails down / up, reset, LP
// on-command, video back) before the SPI bus is opened: GuampPanelCycle.inc / GuampPanelCycleCore.h.
int panelCycle(const char *when, int which = 2) {
  for (unsigned i = 0; i < 60 && !gPanelCycle; ++i) IOSleep(50);
  int (*fn)(int) = gPanelCycle;
  const int r = fn ? fn(which) : -1;
  IOLog("GuampTouch: panel cycle %s: %s (%d)\n", when, !fn ? "no DSI (GuampBacklight absent)" : r == 0 ? "sent" : "refused", r);
  return r;
}
unsigned gHxfw = 0;   // guamp-touch-hxfw, see thread()

void regDump(const char *when) {
  if (!gRegDump) return;
  static const uint32_t regions[] = {0x10007000u, 0x10007080u, 0x100072C0u, 0x10007F00u, 0x10007FC0u, 0x10005800u,
                                     0x900000A8u, 0x90000048u, 0x80050000u, 0x90000080u, 0x900000C0u,
                                     0x100070C0u, 0x10007100u};
  for (uint32_t a : regions) {
    if (!gHxfw && (a == 0x100070C0u || a == 0x10007100u)) continue;
    uint8_t b[64] = {};
    const int r = hx_reg_read(&gChip, a, b, sizeof(b));
    char hex[2 * 64 + 16 + 1] = "";
    unsigned n = 0;
    for (unsigned i = 0; i < sizeof(b); i++) {
      n += static_cast<unsigned>(snprintf(hex + n, sizeof(hex) - n, "%02x", b[i]));
      if ((i & 3) == 3 && i + 1 < sizeof(b)) hex[n++] = ' ';
    }
    hex[n] = 0;
    IOLog("GuampTouch: reg %s %08x rc %d: %s\n", when, a, r, hex);
  }
}

// guamp-touch-hxfw: what the running firmware did with the config, compared with TWRP (log-xnu/twrp-himax-reg-*.txt):
// 7F00 A9 9A 9A A9 (upper half = the firmware's acknowledge), 7F04 99 99 (normal mode acknowledged), 72C0 C0 72,
// 7FC8 max DC 0000FDE8, 900000CC a running counter. 100070F8 byte 1 bit 2 = XY swap in the file (07); Linux reads
// HX_XY_REVERSE 0 at run time. Optional idle off (bit 2) as himax_mcu_idle_mode(1).
void hxfwAfterStart() {
  uint32_t f00 = 0, f04 = 0, c0 = 0, dc = 0, cnt0 = 0, cnt1 = 0, xy = 0, mode = 0, mode1 = 0;
  hx_reg_read32(&gChip, HX_ADDR_FLASH_RELOAD, &f00);
  hx_reg_read32(&gChip, HX_ADDR_SORTING_MODE, &f04);
  hx_reg_read32(&gChip, HX_ADDR_2ND_FLASH_RELOAD, &c0);
  hx_reg_read32(&gChip, HX_ADDR_MAX_DC, &dc);
  hx_reg_read32(&gChip, HX_ADDR_XY_RES_ENABLE, &xy);
  hx_reg_read32(&gChip, HX_ADDR_FW_MODE_STATUS, &mode);
  hx_reg_read32(&gChip, HX_ADDR_FRAME_CNT, &cnt0);
  IOSleep(100);
  hx_reg_read32(&gChip, HX_ADDR_FRAME_CNT, &cnt1);
  IOLog("GuampTouch: hxfw after start: 7f00 %08x (ack %s) 7f04 %08x (normal %s) 72c0 %08x max-dc %08x | 100070f8 %08x "
        "(xy swap bit %u, info xyrev %d) mode 10007088 %08x | counter 900000cc %08x -> %08x in 100 ms (%s)\n", f00,
        (f00 >> 16) == 0xA99Au ? "yes" : "no", f04, (f04 & 0xFFFFu) == 0x9999u ? "yes" : "no", c0, dc, xy,
        (xy >> 10) & 1u, gChip.xy_reverse, mode, cnt0, cnt1, cnt0 != cnt1 ? "running" : "still");
  if (gHxfw & 2u) {
    unsigned i = 0;
    for (; i < 20; ++i) {
      hx_reg_read32(&gChip, HX_ADDR_FW_MODE_STATUS, &mode);
      hx_reg_write32(&gChip, HX_ADDR_FW_MODE_STATUS, (mode & ~0xFFu) | HX_FW_IDLE_DIS);
      hx_reg_read32(&gChip, HX_ADDR_FW_MODE_STATUS, &mode1);
      if ((mode1 & 0xFFu) == HX_FW_IDLE_DIS) break;
      IOSleep(10);
    }
    IOLog("GuampTouch: hxfw idle off: 10007088 %08x -> %08x after %u tries (%s)\n", mode, mode1, i + 1,
          (mode1 & 0xFFu) == HX_FW_IDLE_DIS ? "ok" : "not taken");
  }
}

// guamp 03/11 (fin37/38): sotto XNU il firmware gira e manda pacchetti validi, ma con punti fantasma sulla riga TX0
// (y = 20, x a passi di 50 = centro di ogni RX su 1600) e il dito non compare. Per capire se il sensore vede il dito:
// la mappa diagnostica come /proc/android_touch/diag/{iir_s,dc_s,bank_s} del driver Motorola (hx_diag_read).
// guamp-touch-rawdump=<tipo> (default 0 = spento): 1 delta (IIR), 2 raw (DC), 3 bank (baseline), 4..7 gli altri diag,X,
// 8 = a turno 1, 2, 3. guamp-touch-rawdump-ms=<ms> (default 2000, minimo 200) tra due mappe, guamp-touch-rawdump-n=<n>
// (default 90, 0 = senza fine) mappe in tutto, guamp-touch-rawdump-thr=<v> (default 30) soglia per il conteggio dei nodi
// "accesi", guamp-touch-rawdump-diff=1 righe come differenza dalla prima mappa dello stesso tipo (a riposo; il delta
// IIR resta com'e'), guamp-touch-rawdump-keep=1 lascia il raw out select acceso tra due mappe (default 0: rimesso a 0
// come fa himax_sram_read). Righe: "rawdump #n" (riepilogo: min/max con posizione t<TX>,r<RX>, media, nodi oltre
// soglia, self, differenza dalla mappa di riferimento, byte di stato del pacchetto), poi "rd<n> t<TX>:" 32 valori RX per
// riga (le ripetizioni come valore*volte), "rd<n> srx:" / "rd<n> stx:" i self RX e TX.
namespace rawdump {
constexpr unsigned kCells = 1024, kSelf = 96;
uint8_t gBuf[HX_RAW_BYTES(32u, 32u)];
int32_t gMut[kCells], gSelf[kSelf], gRef[8][kCells];
bool gHaveRef[8];
unsigned gMode, gEveryMs = 2000, gLeft = 90, gThr = 30, gDiff, gKeep, gSeq;
uint64_t gNext, gGap;

void row(unsigned seq, const char *tag, const int32_t *v, unsigned n) {
  char line[768];
  unsigned len = 0;
  line[0] = 0;
  for (unsigned i = 0; i < n;) {
    unsigned run = 1;
    while (i + run < n && v[i + run] == v[i]) ++run;
    const int w = run >= 3 ? snprintf(line + len, sizeof(line) - len, " %d*%u", v[i], run)
                           : snprintf(line + len, sizeof(line) - len, " %d", v[i]);
    if (w < 0 || static_cast<unsigned>(w) >= sizeof(line) - len) break;
    len += static_cast<unsigned>(w);
    i += run >= 3 ? run : 1;
  }
  IOLog("GuampTouch: rd%u %s:%s\n", seq, tag, line);
}

void init() {
  PE_parse_boot_argn("guamp-touch-rawdump", &gMode, sizeof(gMode));
  PE_parse_boot_argn("guamp-touch-rawdump-ms", &gEveryMs, sizeof(gEveryMs));
  PE_parse_boot_argn("guamp-touch-rawdump-n", &gLeft, sizeof(gLeft));
  PE_parse_boot_argn("guamp-touch-rawdump-thr", &gThr, sizeof(gThr));
  PE_parse_boot_argn("guamp-touch-rawdump-diff", &gDiff, sizeof(gDiff));
  PE_parse_boot_argn("guamp-touch-rawdump-keep", &gKeep, sizeof(gKeep));
  if (gMode > 8) gMode = 0;
  if (gEveryMs < 200) gEveryMs = 200;
  if (!gLeft) gLeft = ~0u;
  nanoseconds_to_absolutetime(static_cast<uint64_t>(gEveryMs) * 1000 * 1000, &gGap);
  gNext = mach_absolute_time() + gGap;
  if (gMode)
    IOLog("GuampTouch: rawdump tipo %u ogni %u ms, %u mappe, soglia %u, diff %u, keep %u, rx %u tx %u (righe t<TX>: valori RX)\n",
          gMode, gEveryMs, gLeft, gThr, gDiff, gKeep, gChip.rx_num, gChip.tx_num);
}

// ev: l'ultimo pacchetto letto (byte di stato hx_state_info a HX_EVENT_LEN - 3, come himax_distribute_touch_data).
void dump(const uint8_t *ev) {
  if (!gMode || !gLeft || mach_absolute_time() < gNext) return;
  gLeft--;
  const unsigned seq = ++gSeq;
  const unsigned rx = gChip.rx_num, tx = gChip.tx_num;
  const unsigned type = gMode == 8 ? 1 + (seq - 1) % 3 : gMode;
  if (!rx || !tx || rx * tx > kCells || rx + tx > kSelf || HX_RAW_BYTES(rx, tx) > sizeof(gBuf)) {
    IOLog("GuampTouch: rawdump #%u: canali %ux%u fuori misura\n", seq, rx, tx);
    gLeft = 0;
    return;
  }
  const uint8_t sel = static_cast<uint8_t>(type | HX_RAW_DSRAM);
  const uint32_t back = hx_diag_select(&gChip, sel);
  unsigned waitMs = 0;
  const int rc = hx_diag_read(&gChip, rx, tx, gBuf, sizeof(gBuf), gMut, gSelf, &waitMs);
  if (!gKeep) hx_diag_select(&gChip, 0);
  gNext = mach_absolute_time() + gGap;
  if (rc) {
    IOLog("GuampTouch: rawdump #%u tipo %u sel %02x (letto %08x) rc %d dopo %u ms, testa %02x%02x%02x%02x\n", seq, type, sel,
          back, rc, waitMs, gBuf[0], gBuf[1], gBuf[2], gBuf[3]);
    return;
  }
  const unsigned n = rx * tx;
  int32_t mn = gMut[0], mx = gMut[0];
  unsigned mnAt = 0, mxAt = 0, over = 0;
  int64_t sum = 0;
  for (unsigned i = 0; i < n; ++i) {
    if (gMut[i] < mn) { mn = gMut[i]; mnAt = i; }
    if (gMut[i] > mx) { mx = gMut[i]; mxAt = i; }
    sum += gMut[i];
  }
  // Riferimento: la prima mappa riuscita di ogni tipo (a riposo, se nessuno tocca all'avvio).
  const bool first = !gHaveRef[type];
  if (first) { for (unsigned i = 0; i < n; ++i) gRef[type][i] = gMut[i]; gHaveRef[type] = true; }
  int32_t dmn = 0, dmx = 0;
  unsigned dmnAt = 0, dmxAt = 0;
  for (unsigned i = 0; i < n; ++i) {
    const int32_t d = type == 1 ? gMut[i] : gMut[i] - gRef[type][i];
    if (d < dmn) { dmn = d; dmnAt = i; }
    if (d > dmx) { dmx = d; dmxAt = i; }
    if ((d < 0 ? -d : d) >= static_cast<int32_t>(gThr)) over++;
  }
  int32_t sxMn = gSelf[0], sxMx = gSelf[0], styMn = gSelf[rx], styMx = gSelf[rx];
  for (unsigned i = 0; i < rx; ++i) { sxMn = gSelf[i] < sxMn ? gSelf[i] : sxMn; sxMx = gSelf[i] > sxMx ? gSelf[i] : sxMx; }
  for (unsigned i = rx; i < rx + tx; ++i) { styMn = gSelf[i] < styMn ? gSelf[i] : styMn; styMx = gSelf[i] > styMx ? gSelf[i] : styMx; }
  const uint8_t s0 = ev[HX_EVENT_LEN - 3], s1 = ev[HX_EVENT_LEN - 2];
  IOLog("GuampTouch: rawdump #%u tipo %u sel %02x attesa %u ms | mutual min %d@t%u,r%u max %d@t%u,r%u media %d | %s min %d@t%u,r%u "
        "max %d@t%u,r%u, |v|>=%u: %u nodi | self rx %d..%d tx %d..%d | stato %02x %02x (recal %u palm %u ac %u water %u "
        "glove %u txhop %u base %u osrhop %u) irqs %u\n", seq, type, sel, waitMs, mn, mnAt / rx, mnAt % rx, mx, mxAt / rx,
        mxAt % rx, static_cast<int>(sum / static_cast<int64_t>(n)), type == 1 ? "delta" : first ? "rif (questa)" : "diff dal rif",
        dmn, dmnAt / rx, dmnAt % rx, dmx, dmxAt / rx, dmxAt % rx, gThr, over, sxMn, sxMx, styMn, styMx, s0, s1, s0 & 1u,
        (s0 >> 1) & 1u, (s0 >> 2) & 1u, (s0 >> 3) & 1u, (s0 >> 4) & 1u, (s0 >> 5) & 1u, (s0 >> 6) & 1u, (s1 >> 3) & 1u, gIrqs);
  int32_t line[kSelf];
  char tag[8];
  for (unsigned t = 0; t < tx; ++t) {
    for (unsigned r = 0; r < rx; ++r) {
      const unsigned i = t * rx + r;
      line[r] = gDiff && type != 1 && !first ? gMut[i] - gRef[type][i] : gMut[i];
    }
    snprintf(tag, sizeof(tag), "t%02u", t);
    row(seq, tag, line, rx);
  }
  row(seq, "srx", gSelf, rx);
  row(seq, "stx", gSelf + rx, tx);
}
}  // namespace rawdump

// guamp-touch-events: suspend / resume del driver Linux (vedi displayEvent). L interrupt si maschera nel TLMM come
// himax_int_enable(0) e si riattiva a fine resume con lo stato azzerato.
unsigned gEvDelay = 60, gEvReloadWord = 0x00009AA9u;
volatile uint32_t gEvResumes, gEvSuspends;
void evIrq(bool on) {
  if (!gIrqPin) return;
  const uint32_t cfg = gIrqPin[0x8 / 4];
  if (!on) { gIrqPin[0x8 / 4] = cfg & ~1u; __asm__ volatile("dsb sy" ::: "memory"); return; }
  gIrqPin[0xc / 4] = 0;
  __asm__ volatile("dsb sy" ::: "memory");
  gIrqPin[0x8 / 4] = cfg | 1u;
  __asm__ volatile("dsb sy" ::: "memory");
}
void evSuspend(bool useIrq) {
  if (useIrq) evIrq(false);
  struct hx_report up = {};
  publish(up);                                          // himax_report_all_leave_event
  gEvSuspends = gEvSuspends + 1;
  IOLog("GuampTouch: events: suspend %u (display off): irq %s, contacts released, chip left running (SMWP 0)\n",
        gEvSuspends, useIrq ? "masked" : "polling paused");
}
// Restituisce lo stato centrale (05 = in esecuzione).
uint32_t evResume(bool useIrq, const char *why) {
  if (useIrq) evIrq(false);
  if (gEvDelay) IOSleep(gEvDelay);                      // himax_resume_thread_func: queue_delayed_work 60 ms
  uint64_t t0 = 0, t1 = 0;
  absolutetime_to_nanoseconds(mach_absolute_time(), &t0);
  int r = gFwOk ? hx_download(&gChip, &gFw, gCfg) : HX_ERR_FW;   // fp_0f_operation_dirly -> firmware_update_0f
  const int dl = r, dlStage = gChip.stage;
  int rl = -1, so = -1;
  if (!r) {
    rl = hx_reg_write32(&gChip, HX_ADDR_FLASH_RELOAD, gEvReloadWord);   // fp_reload_disable = hx_dis_rload_0f
    so = hx_sense_on(&gChip);                                           // fp_sense_on(0x00)
  }
  uint32_t central = 0, reload = 0, fl = 0, fl2 = 0;
  hx_reg_read32(&gChip, HX_ADDR_CENTRAL_STATE, &central);              // addr_sts_chk
  hx_reg_read32(&gChip, HX_ADDR_RELOAD_TO_ACTIVE, &reload);
  hx_reg_read32(&gChip, HX_ADDR_FLASH_RELOAD, &fl);
  hx_reg_read32(&gChip, HX_ADDR_2ND_FLASH_RELOAD, &fl2);
  hx_set_smwp(&gChip, 0);                                               // fp_resend_cmd_func: SMWP off, cavo
  gChip.usb_word = write32_verify(&gChip, HX_ADDR_USB_DETECT, gChip.usb_in ? HX_HANDSHAKE_PWD : 0);
  struct hx_report up = {};
  publish(up);                                                          // himax_report_all_leave_event
  if (useIrq) evIrq(true);                                              // himax_int_enable(1)
  absolutetime_to_nanoseconds(mach_absolute_time(), &t1);
  gEvResumes = gEvResumes + 1;
  IOLog("GuampTouch: events: resume %u (%s) after %u ms: download rc %d stage %d cfg crc hw %08x sw %08x ovl %d reply %02x | "
        "0x10007F00 <- %08x (rc %d), sense on rc %d | central %08x (%s) reload %08x 7f00 %08x 72c0 %08x | smwp %08x usb %08x "
        "| %llu ms\n", gEvResumes, why, gEvDelay, dl, dlStage, gChip.cfg_crc_hw, gChip.cfg_crc_sw, gChip.ovl_loaded,
        gChip.ovl_reply, gEvReloadWord, rl, so, central,
        (central & 0xFF) == 0x05 ? "running" : "NOT running: retry at the next display on", reload, fl, fl2, gChip.smwp_word,
        gChip.usb_word, (unsigned long long)((t1 - t0) / 1000000ull));
  regDump("events-resume");
  return central;
}

void thread(void *param, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-touch");
  const unsigned stage = static_cast<unsigned>(reinterpret_cast<uintptr_t>(param));
  // guamp-touch-delay=<s>: start later, so the USB mirror (it connects after the first seconds) sees every line.
  unsigned delay = 0;
  PE_parse_boot_argn("guamp-touch-delay", &delay, sizeof(delay));
  if (delay && delay <= 600) IOSleep(delay * 1000);
  if (!snapshot() || stage < 2 || !enable()) return;
  {
    // guamp-touch-panelcycle=5: the Linux panel power cycle (GuampPanelCycle.inc) as early as the touch pins allow,
    // before the SPI bus, the reset, the download and the start (all unchanged after it), and long before the first
    // frame of the display copy (the scanout is held anyway). Then the panel probe again.
    unsigned cyc = 0;
    PE_parse_boot_argn("guamp-touch-panelcycle", &cyc, sizeof(cyc));
    if (cyc == 5) {
      const int pc = panelcycle::run();
      IOLog("GuampTouch: panel power cycle (panelcycle 5): result %d\n", pc);
      panelProbe();
      logPanel();
    }
  }
  if (stage < 3) return;
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
  // guamp 03/11: in TWRP (Linux himax driver) the new screen's touch works with the same firmware and the same config CRC,
  // here it never reports. On 28/10 events arrived only after Linux had loaded the firmware (info "fallback"): with
  // guamp-touch-keepfw=1 the firmware left in the chip's SRAM by Linux is kept (no download), only sense on is redone.
  unsigned keepFw = 0;
  PE_parse_boot_argn("guamp-touch-keepfw", &keepFw, sizeof(keepFw));
  int r = 0;
  struct hx_fw &fw = gFw;
  gFwOk = hx_fw_parse(&gFw, fwData, fwSize) == HX_OK;
  PE_parse_boot_argn("guamp-touch-regdump", &gRegDump, sizeof(gRegDump));
  // guamp 04/11 (firmware/config analysis, agent hxfw): guamp-touch-hxfw=<bits> (default 0 = previous behaviour).
  // 1: 0x10007F00 = 0x00009AA9 before every sense on instead of 0. The zero-flash build of himax_0flash_mmi maps
  //    fp_reload_disable to hx_dis_rload_0f, which writes zf_data_dis_flash_reload whatever the argument: Linux tells
  //    the firmware "no flash reload, the config is already in SRAM" (TWRP read_FW_ver data A9 9A, C0 72 at 0x100072C0,
  //    0x10007F00 = A9 9A 9A A9 at run time). With 0 the firmware never writes C0 72 nor its run-time words (max DC
  //    0x10007FC8, 0x10007FD0.., counter 0x900000CC) and reports unscaled electrode-grid points on TX0.
  // 2: idle mode off after the start (himax_mcu_idle_mode(1): byte 0 of 0x10007088 = 0x17; the file has 0x3F).
  // 4: leave the SMWP word 0x10007F10 as found (TWRP never writes it at boot).
  // Any bit: "hxfw:" rows after the start (markers, firmware run-time words, XY flags, counter 100 ms apart) and the
  // extra regions 0x100070C0 / 0x10007100 in the reg dumps.
  PE_parse_boot_argn("guamp-touch-hxfw", &gHxfw, sizeof(gHxfw));
  gChip.zf_dis_reload = (gHxfw & 1u) != 0;
  gChip.no_smwp_write = (gHxfw & 4u) != 0;
  if (gHxfw) IOLog("GuampTouch: hxfw %u: 7f00 word %08x, idle off %d, smwp write %d\n", gHxfw,
                   gChip.zf_dis_reload ? HX_ZF_DIS_RELOAD : 0u, (gHxfw & 2u) != 0, !gChip.no_smwp_write);
  regDump("identify");
  // guamp 03/11 (fin38 + TWRP reference): in TWRP the running chip holds a config that is NOT the one uploaded (CID
  // 1205, TOUCH 03, date 20200925 at 0x10007000; the file carries 1206 / 06 / 20210219, same CRC 3D3C19AF in the TWRP
  // dmesg), and the reload markers A9 9A / C0 72: after the upload the firmware reloads the panel's own config. Here the
  // reload never happens and the uploaded config makes electrode-grid phantoms. guamp-touch-flashfirst=1 (default):
  // first a sense on without download; if the firmware reloads by itself within 1 s, its own code and config are kept.
  // fin39 + TWRP (03/11): the markers are transient and read 0 in a working TWRP too, so the probe proves nothing:
  // default off now (it cost 1 s and a sense on).
  unsigned flashFirst = 0;
  PE_parse_boot_argn("guamp-touch-flashfirst", &flashFirst, sizeof(flashFirst));
  if (!keepFw && flashFirst) {
    const int p = hx_reload_probe(&gChip, 100);
    IOLog("GuampTouch: flash-first: reload %s (7f00 %08x 72c0 %08x, rc %d, tight %u)%s\n", p == 1 ? "seen" : "not seen",
          gChip.reload1, gChip.reload2, p, gChip.reload_tight, p == 1 ? ": no download, the chip's own firmware and config are used" : ", download");
    regDump("flashfirst");
    if (p == 1) keepFw = 2;
    else hx_sense_off(&gChip);
  }
  {
    unsigned cyc = 0;
    PE_parse_boot_argn("guamp-touch-panelcycle", &cyc, sizeof(cyc));
    if (cyc == 4) {
      // Linux (TWRP 03/11, touch working without any suspend): dsi_panel_disable 2.475, post_unprepare 2.576 (LCD
      // reset low, rails down), power_on 2.592 (rails up, reset sequence 1/5, 0/2, 1/60 ms of the dtsi), enable 2.724
      // (on-command), then the touch probe at 4.8 s. Here without the rails (L9A / LCDB stay on: no safe SPMI write
      // path for them in this service): touch reset low, LCD reset low 20 ms, the reset sequence, the full on-command
      // over DSI while the MDSS video stream keeps running, touch reset released, identify again, download, start.
      pinWrite(GT_GPIO_RESET, 4, 0u);
      pinWrite(GT_GPIO_LCD_RESET, 4, 0u); IOSleep(20);
      pinWrite(GT_GPIO_LCD_RESET, 4, 2u); IOSleep(5);
      pinWrite(GT_GPIO_LCD_RESET, 4, 0u); IOSleep(2);
      pinWrite(GT_GPIO_LCD_RESET, 4, 2u); IOSleep(60);
      panelCycle("lcd reset + on-command", 3);
      IOSleep(100);
      pinWrite(GT_GPIO_RESET, 4, 2u); IOSleep(50);
      const int id = hx_identify(&gChip);
      IOLog("GuampTouch: after panel re-init: lcd-rst %u, identify rc %d id %02x%02x%02x central %02x\n",
            pinRead(GT_GPIO_LCD_RESET, 4) & 1u, id, gChip.id[3], gChip.id[2], gChip.id[1], gChip.central_state);
      regDump("panel-reinit");
    }
    if (cyc == 3 && !keepFw) {
      panelCycle("resume order: off-command", 0);
      pinWrite(GT_GPIO_RESET, 4, 0u);                    // himax_rst_gpio_set(rst, 0) in suspend
      IOSleep(20);
      panelCycle("resume order: on-command", 1);
      pinWrite(GT_GPIO_RESET, 4, 2u); IOSleep(2);        // _himax_chip_common_resume, HX_RESUME_SET_FW
      pinWrite(GT_GPIO_RESET, 4, 0u); IOSleep(5);
      pinWrite(GT_GPIO_RESET, 4, 2u); IOSleep(2);
      regDump("resume-order");
    }
  }
  if (!keepFw) {
    r = gFwOk ? HX_OK : HX_ERR_FW;
    IOLog("GuampTouch: firmware %u bytes, %u partitions, overlay %d (sorting %d gesture %d border %d) (rc %d)\n", fwSize,
          fw.count, fw.overlay, fw.ovl[0], fw.ovl[1], fw.ovl[2], r);
    if (r) { fail(4, r); return; }
    // guamp 03/11 (fin36): the firmware ran (central 05) with interrupts at ~0.9 Hz and an always empty event stack.
    // Differences with himax_zf_part_info (HX83102D): overlay 11 (gesture, smart wake-up) loaded instead of 12 (border),
    // no overlay reply 0x66 at 0x10007FFC, SMWP / USB / sorting words left as found, no second firmware write.
    // guamp-touch-usb=0|1 (default 1, as TWRP with the cable in), guamp-touch-ovl=<partition> (11 = the old choice),
    // guamp-touch-recode=0 (skip the second firmware write).
    unsigned v = 1;
    PE_parse_boot_argn("guamp-touch-usb", &v, sizeof(v));
    gChip.usb_in = v != 0;
    v = 0;
    if (PE_parse_boot_argn("guamp-touch-ovl", &v, sizeof(v))) gChip.ovl_force = static_cast<int>(v);
    v = 1;
    PE_parse_boot_argn("guamp-touch-recode", &v, sizeof(v));
    gChip.no_recode = v == 0;
    r = hx_download(&gChip, &fw, gCfg);
    IOLog("GuampTouch: download rc %d stage %d cfg crc hw %08x sw %08x\n", r, gChip.stage, gChip.cfg_crc_hw, gChip.cfg_crc_sw);
    IOLog("GuampTouch: overlay loaded %d reply %02x handshake %08x | recode %s crc %08x | smwp %08x usb %08x (%s) sort %08x\n",
          gChip.ovl_loaded, gChip.ovl_reply, gChip.ovl_handshake, gChip.no_recode ? "off" : "on", gChip.recode_crc,
          gChip.smwp_word, gChip.usb_word, gChip.usb_in ? "in" : "out", gChip.sort_word);
    if (r) { fail(41, r); return; }
  } else {
    IOLog("GuampTouch: keepfw: no download, the firmware already in the chip is used\n");
  }
  // Sense on timing (guamp-touch-sense-high=<ms>, default 10 as Linux zero-flash) and attempts until "reload OK"
  // (guamp-touch-reload-tries, default 3). guamp-touch-redl=0: no second download when the reload is never seen.
  {
    unsigned v = 0;
    if (PE_parse_boot_argn("guamp-touch-sense-high", &v, sizeof(v)) && v <= 300) gChip.sense_high_ms = v ? v : 10;
    v = 0;
    if (PE_parse_boot_argn("guamp-touch-reload-tries", &v, sizeof(v)) && v && v <= 10) gChip.reload_tries = v;
  }
  regDump("download");
  unsigned cycle = 0;
  PE_parse_boot_argn("guamp-touch-panelcycle", &cycle, sizeof(cycle));
  if (cycle == 1) panelCycle("before sense on");
  r = hx_start(&gChip);
  regDump("start");
  if (gHxfw && !r) hxfwAfterStart();
  if (cycle == 2 && !r) {
    panelCycle("after start");
    uint32_t a = 0, b = 0;
    unsigned i = 0;
    for (; i < 100; ++i) {
      hx_reg_read32(&gChip, HX_ADDR_FLASH_RELOAD, &a);
      hx_reg_read32(&gChip, HX_ADDR_2ND_FLASH_RELOAD, &b);
      if ((a & 0xFFFF) == 0x9AA9 || (a & 0xFFFF) == 0x3AA3 || (b & 0xFFFF) == 0x72C0) break;
      IOSleep(10);
    }
    IOLog("GuampTouch: after panel cycle: reload %s (7f00 %08x 72c0 %08x after %u polls)\n", i < 100 ? "seen" : "NOT seen", a, b, i);
    regDump("cycle");
  }
  unsigned redl = 0;                                   // reload markers are transient: no second download by default
  PE_parse_boot_argn("guamp-touch-redl", &redl, sizeof(redl));
  if (!r && !keepFw && redl && gChip.reload_polls >= 200) {
    IOLog("GuampTouch: reload NOT seen after %u sense on: firmware download again\n", gChip.reload_attempts);
    r = hx_download(&gChip, &fw, gCfg);
    if (!r) r = hx_start(&gChip);
  }
  {
    uint32_t central = 0, isr = 0, reload = 0;
    hx_reg_read32(&gChip, HX_ADDR_CENTRAL_STATE, &central);
    hx_reg_read32(&gChip, HX_ADDR_CTRL_FW_ISR, &isr);
    hx_reg_read32(&gChip, HX_ADDR_RELOAD_TO_ACTIVE, &reload);
    IOLog("GuampTouch: after start: central state %08x (05 = running), fw isr %08x, reload %08x\n", central, isr, reload);
    // Linux (TWRP): reload OK with data 9A A9 / 72 C0, PANEL_VER 18, FW_VER 1305, TOUCH_VER 6, DISPLAY_VER 0, CID 1206.
    IOLog("GuampTouch: reload %s after %u tight reads + %u polls, %u sense on (7f00 %08x 72c0 %08x) | panel %02x fw %04x touch %02x display %02x cid %04x\n",
          gChip.reload_polls < 200 ? "OK" : "NOT seen", gChip.reload_tight, gChip.reload_polls, gChip.reload_attempts, gChip.reload1, gChip.reload2, gChip.panel_ver,
          gChip.fw_ver, gChip.touch_cfg_ver, gChip.display_cfg_ver, gChip.cid_ver);
  }
  IOLog("GuampTouch: start rc %d res %ux%u pt %u edge %d\n", r, gChip.x_res, gChip.y_res, gChip.max_pt, gChip.int_is_edge);
  if (r) { fail(42, r); return; }
  gStep = 5;
  const bool useIrq = armIrq();
  IOLog("GuampTouch: running, %s\n", useIrq ? "interrupt driven" : "polled every 10 ms");
  mapInit();
  IOLog("GuampTouch: map %dx%d px -> HID 0..32767 (360x800 pt), edge bands top %d side %d bottom %d px, fade %d, flip %d, "
        "log %u contacts\n", gChip.x_res ? gChip.x_res : GT_PANEL_X, gChip.y_res ? gChip.y_res : GT_PANEL_Y, gEdgeTop, gEdgeSide,
        gEdgeBottom, kEdgeFade, gFlip, gMapLog);
  uint8_t ev[HX_EVENT_LEN] = {};
  rawdump::init();
  // guamp 03/11: new screen, the chip answers and runs but every report is an empty one even under a finger.
  // guamp-touch-raw=<n> (default 64): the event bytes are logged whenever they change, at most n times.
  uint8_t lastRaw[HX_EVENT_LEN] = {};
  unsigned rawLeft = 64;
  PE_parse_boot_argn("guamp-touch-raw", &rawLeft, sizeof(rawLeft));
  // guamp-touch-diag=<n> (default 16): the whole event stack of the first n interrupts, zeros included, and the firmware
  // status words with every "polls" line. guamp-touch-irqonly=1: read the stack only after an interrupt (as Linux does)
  // instead of every 20 ms as well.
  // guamp 03/11 (fin37): irqonly is the default now. A read without an interrupt finds the stack already consumed (all
  // zero), which was taken as "all fingers up" between two reports. guamp-touch-irqonly=0 polls every 20 ms as well.
  // guamp-touch-dump=<n>: the whole stack of every n-th interrupt (0 = off), for long captures.
  // Filters (phantom reports of fin37, electrode-grid coordinates): a report whose >= 4 points share one x or one y is
  // dropped (guamp-touch-line=0 keeps it); >= 3 points identical for more than guamp-touch-repeat (default 32) reports
  // in a row are dropped until they change (0 = off).
  unsigned diagLeft = 16, irqOnly = 1, dumpEvery = 0, lineFilter = 1, repeatMax = 32;
  PE_parse_boot_argn("guamp-touch-dump", &dumpEvery, sizeof(dumpEvery));
  PE_parse_boot_argn("guamp-touch-line", &lineFilter, sizeof(lineFilter));
  PE_parse_boot_argn("guamp-touch-repeat", &repeatMax, sizeof(repeatMax));
  // guamp-touch-grid (03/11 idle, default 1, =0 off): live-20261103-195840 has ~2600 contacts that begin on the electrode grid
  // of the top row, raw y = 20 and x = 25 + 50 k (75, 125 ... 675), almost all one report long: a phantom tap every ~0.4 s that
  // NativeHidInput sends as a digitizer event, i.e. user activity for backboardd (AttentionAwareness) and the auto-lock never
  // runs out. A contact whose first point lies on that grid point is not published; it is sent from its first point off the
  // grid (a finger that started elsewhere is never cut). guamp-touch-grid-y (default 20), guamp-touch-grid-pitch (50) and
  // guamp-touch-grid-offset (25) in raw panel pixels.
  unsigned gridFilter = 1, gridY = 20, gridPitch = 50, gridOffset = 25;
  PE_parse_boot_argn("guamp-touch-grid", &gridFilter, sizeof(gridFilter));
  PE_parse_boot_argn("guamp-touch-grid-y", &gridY, sizeof(gridY));
  PE_parse_boot_argn("guamp-touch-grid-pitch", &gridPitch, sizeof(gridPitch));
  PE_parse_boot_argn("guamp-touch-grid-offset", &gridOffset, sizeof(gridOffset));
  if (!gridPitch) gridFilter = 0;
  IOLog("GuampTouch: grid filter %s (row %u, x = %u + %u k)\n", gridFilter ? "on" : "off", gridY, gridOffset, gridPitch);
  uint32_t phantoms = 0, repeats = 0, empties = 0;
  unsigned sameRun = 0;
  struct hx_report lastRep = {};
  PE_parse_boot_argn("guamp-touch-diag", &diagLeft, sizeof(diagLeft));
  PE_parse_boot_argn("guamp-touch-irqonly", &irqOnly, sizeof(irqOnly));
  const bool diag = diagLeft != 0;
  unsigned wake = 0;
  PE_parse_boot_argn("guamp-touch-wake", &wake, sizeof(wake));
  PE_parse_boot_argn("guamp-touch-wake-wait", &gWakeWait, sizeof(gWakeWait));
  PE_parse_boot_argn("guamp-touch-wake-force", &gWakeForce, sizeof(gWakeForce));
  unsigned wakeDsi = 1;
  PE_parse_boot_argn("guamp-touch-wake-dsi", &wakeDsi, sizeof(wakeDsi));
  if (gWakeWait > 5000) gWakeWait = 5000;
  gWakeDsi = wakeDsi != 0;
  const bool wakeOk = wake && useIrq && gFwOk && gFw.ovl[1] > 0 && gFw.ovl[2] > 0 && gFw.part[gFw.ovl[1]].size;
  if (wake)
    IOLog("GuampTouch: wake %s (gesture overlay %d, border %d, irq %d, panel commands %s, wait %u ms, force %u)\n",
          wakeOk ? "armed" : "NOT armed", gFw.ovl[1], gFw.ovl[2], useIrq, gWakeDsi ? "on" : "off", gWakeWait, gWakeForce);
  uint32_t displayGen = gDisplayGen;
  bool suspended = false;
  unsigned gestLog = 0;
  uint64_t lastWake = 0, wakeGapTicks = 0;
  nanoseconds_to_absolutetime(1500 * 1000 * 1000ULL, &wakeGapTicks);
  if (wakeOk) gWakeArmed = 1;
  {
    unsigned v = 0;
    if (PE_parse_boot_argn("guamp-touch-events-delay", &v, sizeof(v)) && v <= 1000) gEvDelay = v;
    v = 1;
    PE_parse_boot_argn("guamp-touch-events-rl", &v, sizeof(v));
    gEvReloadWord = v ? 0x00009AA9u : 0u;
    if (gEventsMode && (gWakeArmed || !gFwOk)) {
      IOLog("GuampTouch: events mode %u NOT armed (%s)\n", gEventsMode, gWakeArmed ? "guamp-touch-wake armed" : "firmware not parsed");
      gEventsMode = 0;
    } else if (gEventsMode) {
      IOLog("GuampTouch: events mode %u armed (%s, delay %u ms, 0x10007F00 <- %08x, panel commands %s, %u display events "
            "already seen)\n", gEventsMode,
            gEventsMode == 1 ? "resume at every display on, the first included" : "Linux: resume after an off only", gEvDelay,
            gEvReloadWord, gEventsDsi ? "on" : "off", gEvGen);
    }
  }
  uint32_t evGen = 0;
  bool evSuspended = false, evFirstDone = false, evFailed = false;
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
    } else {
      IOSleep(10);
    }
    const bool fromIrq = useIrq && gIrqEvents != seen;
    seen = gIrqEvents;
    polls = polls + 1;
    if (gEventsMode && gEvGen != evGen) {
      evGen = gEvGen;
      const bool on = gEvOn != 0;
      if (!on && !evSuspended) {
        down = false;
        evSuspend(useIrq);
        evSuspended = true;
      } else if (on && (evSuspended || evFailed || (gEventsMode == 1 && !evFirstDone))) {
        const char *why = evSuspended ? "display on again" : evFailed ? "last resume failed" : "first display on of iOS";
        evSuspended = false;
        evFailed = (evResume(useIrq, why) & 0xFF) != 0x05;
      } else if (on && !evFirstDone) {
        IOLog("GuampTouch: events: first display on: no resume (Linux: not suspended, \"It had entered resume, skip\")\n");
      }
      if (on) evFirstDone = true;
      seen = gIrqEvents;
      continue;
    }
    if (evSuspended) continue;                         // Linux: interrupt disabilitato fino al resume
    if (gWakeArmed && gDisplayGen != displayGen) {
      displayGen = gDisplayGen;
      const bool off = !gDisplayOn;
      if (off && !suspended) {
        if (down) { struct hx_report up = {}; down = false; publish(up); }
        wakeSuspend();
        suspended = true;
        gestLog = 0;
      } else if (!off && suspended) {
        wakeResume();
        suspended = false;
      }
      continue;
    }
    if (suspended) {
      // Gesture mode: only interrupts matter, and only a gesture report (8 bytes of the stack) counts.
      if (!fromIrq) continue;
      if (hx_read_event(&gChip, ev)) continue;
      const int id = hx_parse_gesture(ev);
      if (gestLog < 24 || id) {
        gestLog++;
        IOLog("GuampTouch: suspended irq %u: %02x %02x %02x %02x %02x %02x %02x %02x -> %s %02x\n", gIrqs, ev[0], ev[1], ev[2],
              ev[3], ev[4], ev[5], ev[6], ev[7], id ? "gesture" : "no gesture", id);
      }
      if (id && !gDisplayOn && mach_absolute_time() - lastWake > wakeGapTicks) {
        lastWake = mach_absolute_time();
        __atomic_fetch_add(&gWakeRequests, 1, __ATOMIC_RELEASE);
        IOLog("GuampTouch: wake: gesture %02x (%s), Power press requested (%u)\n", id,
              id == HX_GEST_DOUBLE_TAP ? "double tap" : id == 0x81 ? "finger gesture" : "other", gWakeRequests);
      }
      continue;
    }
    if (polls == 50 || (polls & 2047) == 0) {
      IOLog("GuampTouch: polls %u irqs %u events %u bad %u released %u info %ux%u pt %u fallback %d | empty %u phantom %u "
            "repeat %u\n", polls, gIrqs, gEvents, gBad, gReleases, gChip.x_res, gChip.y_res, gChip.max_pt, gChip.info_fallback,
            empties, phantoms, repeats);
      if (diag) logStatus(polls);
    }
    rawdump::dump(ev);
    if (irqOnly && useIrq && !fromIrq) {
      // No interrupt for 20 ms: only the lost "finger up" guard.
      if (down && mach_absolute_time() - lastGood > releaseTicks) {
        struct hx_report up = {};
        down = false;
        gReleases = gReleases + 1;
        publish(up);
      }
      continue;
    }
    struct hx_report rep;
    const int readError = hx_read_event(&gChip, ev);
    if (dumpEvery && fromIrq && gIrqs % dumpEvery == 0) {
      char hex[3 * HX_EVENT_LEN + 1] = "";
      for (unsigned i = 0; i < sizeof(ev); i++) snprintf(hex + 3 * i, 4, "%02x ", ev[i]);
      IOLog("GuampTouch: dump irq %u rc %d: %s\n", gIrqs, readError, hex);
    }
    if (diagLeft && fromIrq) {
      diagLeft--;
      char hex[3 * HX_EVENT_LEN + 1] = "";
      for (unsigned i = 0; i < sizeof(ev); i++) snprintf(hex + 3 * i, 4, "%02x ", ev[i]);
      IOLog("GuampTouch: diag irq %u poll %u rc %d: %s\n", gIrqs, polls, readError, hex);
      if (diagLeft < 4 || (diagLeft & 3) == 0) logStatus(polls);
    }
    if (rawLeft && !readError && memcmp(ev, lastRaw, sizeof(ev))) {
      rawLeft--;
      memcpy(lastRaw, ev, sizeof(ev));
      char hex[3 * 24 + 1] = "";
      for (unsigned i = 0; i < sizeof(ev) && i < 24; i++) snprintf(hex + 3 * i, 4, "%02x ", ev[i]);
      IOLog("GuampTouch: raw poll %u irqs %u: %s\n", polls, gIrqs, hex);
    }
    const int parsed = readError ? HX_ERR_BUS
                                 : hx_parse_event(ev, gChip.x_res ? gChip.x_res : GT_PANEL_X, gChip.y_res ? gChip.y_res : GT_PANEL_Y, &rep);
    bool drop = parsed != HX_OK;
    if (!drop && lineFilter && rep.count >= 4) {
      bool sameX = true, sameY = true;
      for (unsigned i = 1; i < rep.count; ++i) {
        sameX = sameX && rep.point[i].x == rep.point[0].x;
        sameY = sameY && rep.point[i].y == rep.point[0].y;
      }
      if (sameX || sameY) {
        drop = true;
        phantoms = phantoms + 1;
        if (phantoms <= 8 || (phantoms & 1023) == 0)
          IOLog("GuampTouch: phantom report %u dropped: %u points on one %s (%d,%d)\n", phantoms, rep.count, sameY ? "row" : "column",
                rep.point[0].x, rep.point[0].y);
      }
    }
    if (!drop && repeatMax && rep.count >= 3) {
      bool same = rep.count == lastRep.count;
      for (unsigned i = 0; same && i < rep.count; ++i)
        same = rep.slot[i] == lastRep.slot[i] && rep.point[i].x == lastRep.point[i].x && rep.point[i].y == lastRep.point[i].y;
      sameRun = same ? sameRun + 1 : 0;
      lastRep = rep;
      if (sameRun > repeatMax) {
        drop = true;
        repeats = repeats + 1;
        if (repeats == 1 || (repeats & 1023) == 0)
          IOLog("GuampTouch: repeated report dropped (%u, %u points identical %u times)\n", repeats, rep.count, sameRun);
      }
    }
    if (!drop && gridFilter && rep.count) {
      unsigned kept = 0;
      for (unsigned i = 0; i < rep.count; ++i) {
        const int s = rep.slot[i], x = rep.point[i].x, y = rep.point[i].y;
        const bool live = s >= 0 && s < HX_MAX_PT && gContact[s].down;
        if (!live && y == static_cast<int>(gridY) && x >= 0 && static_cast<unsigned>(x) % gridPitch == gridOffset) {
          gGridDrops = gGridDrops + 1;
          if (gGridDrops <= 8 || (gGridDrops & 1023) == 0)
            IOLog("GuampTouch: grid contact %u not sent: slot %d at %d,%d (%u points in the report)\n", gGridDrops, s, x, y,
                  rep.count);
          continue;
        }
        if (kept != i) { rep.slot[kept] = rep.slot[i]; rep.point[kept] = rep.point[i]; }
        kept++;
      }
      rep.count = kept;
    }
    if (parsed == HX_ERR_EMPTY) empties = empties + 1;
    if (drop) {
      if (parsed != HX_ERR_EMPTY) gBad = gBad + 1;
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
  // guamp 03/11: screen replaced (no more ghost touches) -> touch ON by default at stage 4; guamp-touch=0 turns it off.
  stage = 4;
  PE_parse_boot_argn("guamp-touch", &stage, sizeof(stage));
  if (!stage) return;
  if (stage > 4) stage = 4;
  {
    unsigned ev = 0, evDsi = 1;
    PE_parse_boot_argn("guamp-touch-events", &ev, sizeof(ev));
    PE_parse_boot_argn("guamp-touch-events-dsi", &evDsi, sizeof(evDsi));
    gEventsDsi = evDsi != 0;
    if (stage == 4 && (ev == 1 || ev == 2)) gEventsMode = ev;   // GuampBacklight notifica da qui in poi
  }
  thread_t t = nullptr;
  if (kernel_thread_start(thread, reinterpret_cast<void *>(static_cast<uintptr_t>(stage)), &t) == KERN_SUCCESS) thread_deallocate(t);
}
}  // namespace touch
