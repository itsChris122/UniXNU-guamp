// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp audio output hardware, included by NativeEmmc.cpp BEFORE GuampAdsp.inc (it provides GuampAudioHw.h strongly,
// so GuampAudioHwWeak.inc leaves its stand-ins out). Speaker (route 1): LPASS LPI i2s1 pins + Awinic aw882xx on QUP0
// SE1 I2C (0x34, reset gpio97). Earpiece (route 2, wcd937x EAR, "handset" path) and headphones (route 3, wcd937x
// HPHL/HPHR): bolero RX macro (needs the DSP clock hook) -> RX SoundWire link (GuampAudioRxSwr.h, rx_swr 0x0a610000,
// wcd937x RX slave) -> wcd937x DACs/PAs. wcd937x registers (RX ones too) are reached only through the VA SoundWire
// master of the capture side (guamp_wcd937x_read/write, GuampMicDev.inc): routes 2/3 need guamp-audio-mic=1.
// The earpiece is NOT on the aw882xx: the guamp DT has one aw882xx (0x34) and Motorola's mixer_paths.xml "handset"
// path drives wcd937x EAR; aw882xx_rcv_reg.bin (receiver mode of the same amplifier) ships with the generic Awinic
// driver and is never selected (no aw882xx_receiver_switch in the HAL or the mixer paths).
// This file also owns the wcd937x power (guamp_wcd937x_power: L9A + L14A RPM votes, reset gpio92, refcounted for the
// RX and TX sides). The bolero fs-gen has one owner: guamp_bolero_fsgen (GuampMicDev.inc, refcounted).
// Sequences in GuampAudioHwCore.h, I2C in GeniI2cCpp.inc; this file owns MMIO, GPIO, the lock and the "GuampAudioHw:"
// log lines (one per step).
// Boot arguments:
//   guamp-audio-hw=0      default: every call answers GUAMP_AUDIO_HW_ABSENT (same as without this file)
//   guamp-audio-hw=1      speaker path live for the DSP side (prepare/amp/volume/release)
//   guamp-audio-hw=2      =1 plus a probe on its own thread at boot: SE1 I2C up, aw882xx reset + chip id, fsa4480
//                         id; amplifier left powered down and muted (no LPI access: that needs the DSP's LPASS vote)
//   guamp-audio-hw-delay=<s>   start the probe later (USB mirror), 0..600
//   guamp-audio-hw-att=<dB>    attenuation at prepare (default 30 dB), guamp-audio-hw-floor=<dB> minimum attenuation
//                              any volume call can reach (default 6 dB, 0 = full scale allowed)
//   guamp-audio-hw-i2c=<kHz>   100, 400 (default, as Linux) or 1000
//   guamp-audio-hw-hph=<dB>    headphone PA gain (HPHL/HPHR Volume), -30..0, default -12 (vendor default -6)
//   guamp-audio-hw-ear=<dB>    earpiece PA gain ("EAR PA GAIN"), -18..6, default -12 (vendor default +6)
//   The RX macro digital gain of routes 2/3 follows guamp-audio-hw-att / -floor like the speaker amplifier.
#define GUAMP_AUDIO_HW_PROVIDED 1
#define GUAMP_WCD937X_POWER_PROVIDED 1   // guamp_wcd937x_power below; GuampMicDev.inc leaves its stand-in out
#include "GuampAudioHw.h"
#include "GuampAudioHwExt.h"
#include "GuampAudioMicHw.h"
#include "GuampAudioHwCore.h"
#include "GuampAudioRxSwr.h"
#include "GuampTouchBoard.h"
#include "GeniI2cCpp.inc"

namespace audiohw {
unsigned gMode, gAttDb = 30, gFloorDb = 6, gKhz = 400, gDelay;
int gHphDb = -12, gEarDb = -12;
IOLock *gLock;
volatile uint8_t *gGcc, *gTlmm, *gSe, *gWrap, *gLpi, *gRxmMap, *gVamMap;
geni_i2c gI2c;
int gBus;                       // 0 never tried, 1 up, <0 failed (error kept)
unsigned gSpk;                  // 0 off, 1 prepared (PWDN 1), 2 running
bool gMuted = true;
unsigned gCdc;                  // routes 2/3: 0 off, 1 prepared (RX macro muted, RX SoundWire up), 2 PA on
uint32_t gCdcRoute;             // GUAMP_AUDIO_ROUTE_EARPIECE or _HEADPHONE while gCdc
uint32_t gWcdChip;
guamprx::RxLink gRx;
struct RxWindow { uint64_t phys, bytes; volatile uint8_t *va; };
RxWindow gRxWin[4] = {{guamprx::kRxSwrm, 0x2000, nullptr}, {guamprx::kRxSwrmHctl & ~0xFFFull, 0x1000, nullptr},
                      {guampmic::kLpiSlew, 0x1000, nullptr}, {guamprx::kRxMacro, 0x1000, nullptr}};
uint16_t gChip, gSysst;
int32_t gLevelCb = INT32_MIN;   // last volume request; INT32_MIN = use guamp-audio-hw-att
int32_t gLastRc;
guamp_audio_lpass_clock_fn gClock;
IOLock *gWcdLock;               // guamp_wcd937x_power (allocated whatever guamp-audio-hw says: the mic side needs it)
unsigned gWcdUsers;
bool gWcdRails;

uint32_t mrd(volatile uint8_t *b, uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(b + off); }
void mwr(volatile uint8_t *b, uint32_t off, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(b + off) = v; __asm__ volatile("dsb sy" ::: "memory"); }
uint32_t seRd(void *c, uint32_t r) { return mrd(static_cast<volatile uint8_t *>(c), r); }
void seWr(void *c, uint32_t r, uint32_t v) { mwr(static_cast<volatile uint8_t *>(c), r, v); }
void delayUs(void *, unsigned us) { IODelay(us); }

void note(void *, const char *step, int rc, uint32_t a, uint32_t b) {
  if (rc) gLastRc = rc;
  IOLog("GuampAudioHw: %s: rc %d (0x%x 0x%x)\n", step, rc, a, b);
}

int i2cW16(void *, uint8_t dev, uint8_t reg, uint16_t v) {
  uint8_t b[3] = {reg, static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v)};
  geni_i2c_msg m = {b, 3, 0};
  int r = -1;
  for (unsigned t = 0; t < 3 && r; ++t) r = geni_i2c_xfer(&gI2c, dev, &m, 1);
  if (r) IOLog("GuampAudioHw: i2c write 0x%02x reg 0x%02x = 0x%04x failed %d (status 0x%x cancel %d)\n", dev, reg, v, r,
               gI2c.last_status, gI2c.last_cancel);
  return r;
}
int i2cRead(uint8_t dev, uint8_t reg, uint8_t *out, uint16_t n) {
  geni_i2c_msg m[2] = {{&reg, 1, 0}, {out, n, 1}};
  int r = -1;
  for (unsigned t = 0; t < 3 && r; ++t) r = geni_i2c_xfer(&gI2c, dev, m, 2);
  if (r) IOLog("GuampAudioHw: i2c read 0x%02x reg 0x%02x failed %d (status 0x%x cancel %d)\n", dev, reg, r, gI2c.last_status,
               gI2c.last_cancel);
  return r;
}
int i2cR16(void *, uint8_t dev, uint8_t reg, uint16_t *v) {
  uint8_t b[2] = {0, 0};
  int r = i2cRead(dev, reg, b, 2);
  if (!r) *v = static_cast<uint16_t>(b[0] << 8 | b[1]);
  return r;
}

// TLMM WEST: +0 cfg (func 5:2, drive 8:6, OE 9), +4 io (out bit 1).
void gpioOut(void *, unsigned gpio, int high) {
  const unsigned drive = gpio == GAH_GPIO_WCD_RESET ? 7u : 0u;          // wcd937x reset 16 mA, aw882xx reset 2 mA
  const uint32_t cfg = gt_pin_cfg(0, drive, 0) | GT_PIN_OE;
  if (mrd(gTlmm, gt_pin(gpio)) != cfg) mwr(gTlmm, gt_pin(gpio), cfg);
  mwr(gTlmm, gt_pin(gpio) + 4, high ? 2u : 0u);
}
uint32_t lpiRd(void *, uint32_t off) { return mrd(gLpi, off); }
void lpiWr(void *, uint32_t off, uint32_t v) { mwr(gLpi, off, v); }
volatile uint8_t *cdcAt(uint32_t reg, uint32_t *off) {
  if (reg >= 0x1000 && reg < 0x2000) { *off = reg - 0x1000; return gRxmMap; }
  if (reg >= 0x3000 && reg < 0x4000) { *off = reg - 0x3000; return gVamMap; }
  return nullptr;
}
uint32_t cdcRd(void *, uint32_t reg) { uint32_t o = 0; volatile uint8_t *b = cdcAt(reg, &o); return b ? mrd(b, o) : 0; }
void cdcWr(void *, uint32_t reg, uint32_t v) { uint32_t o = 0; volatile uint8_t *b = cdcAt(reg, &o); if (b) mwr(b, o, v); }
void sleepMs(void *, unsigned ms) { IOSleep(ms ? ms : 1); }

// wcd937x registers: the one SoundWire access of the capture side (VA master, TX slave); -19 while that link is down
int wcdRd(void *, uint16_t reg, uint8_t *v) { return guamp_wcd937x_read(reg, v); }
int wcdWr(void *, uint16_t reg, uint8_t v) { return guamp_wcd937x_write(reg, v); }

int swrRx(void *, int on, uint8_t ch) { return gRx.datapath(on != 0, ch); }

gah_ops gOps = {nullptr, i2cW16, i2cR16, gpioOut, lpiRd, lpiWr, cdcRd, cdcWr, wcdRd, wcdWr, sleepMs, delayUs, note, swrRx};

// guampmic::Bus for the RX SoundWire link: the RX master, its hctl page, the LPI pins (gLpi) and slew register, the RX macro
volatile uint32_t *rxReg(uint64_t phys) {
  if (gLpi && phys >= GAH_LPI_BASE && phys + 4 <= GAH_LPI_BASE + GAH_LPI_SIZE)
    return reinterpret_cast<volatile uint32_t *>(gLpi + (phys - GAH_LPI_BASE));
  for (RxWindow &w : gRxWin)
    if (w.va && phys >= w.phys && phys + 4 <= w.phys + w.bytes) return reinterpret_cast<volatile uint32_t *>(w.va + (phys - w.phys));
  return nullptr;
}
unsigned gRxBad;
uint32_t rxBusRd(void *, uint64_t phys) {
  volatile uint32_t *r = rxReg(phys);
  if (!r) { gRxBad++; return 0xFFFFFFFFu; }
  const uint32_t v = *r;
  __asm__ volatile("dsb sy" ::: "memory");
  return v;
}
void rxBusWr(void *, uint64_t phys, uint32_t v) {
  volatile uint32_t *r = rxReg(phys);
  if (!r) { gRxBad++; return; }
  __asm__ volatile("dsb sy" ::: "memory");
  *r = v;
  __asm__ volatile("dsb sy" ::: "memory");
}
void rxBusDelay(void *, uint32_t us) {
  if (us >= 1000) IOSleep((us + 999) / 1000);
  else IODelay(us);
}
void rxBusLog(void *, const char *what, uint32_t a, uint32_t b) { IOLog("GuampAudioHw: %s: 0x%x 0x%x\n", what, a, b); }

uint32_t floorCb() { return gFloorDb * 100u; }
uint32_t attNowCb() { return gah_att_from_level(gLevelCb == INT32_MIN ? -static_cast<int32_t>(gAttDb * 100u) : gLevelCb, floorCb()); }

// QUP0 SE1: GPLL0, wrapper AHB/core + SE1 branch votes, 19.2 MHz source (DFS level or TCXO), pins, engine, clock.
int busUp() {
  if (gBus) return gBus > 0 ? 0 : gBus;
  gBus = -12;
  gGcc = mapPhysical(GT_GCC_BASE, 0x80000);
  gTlmm = mapPhysical(GT_TLMM_WEST, GAH_TLMM_WEST_SIZE);
  gSe = mapPhysical(GAH_SE1_BASE, GAH_SE1_SIZE);
  gWrap = mapPhysical(GAH_QUP0_WRAPPER, 0x2000);
  if (!gGcc || !gTlmm || !gSe || !gWrap) { note(nullptr, "bus: map gcc/tlmm/se1/qup0", -12, 0, 0); return gBus; }
  mwr(gGcc, GT_GCC_GPLL0_ENABLE, mrd(gGcc, GT_GCC_GPLL0_ENABLE) | 1u);
  for (unsigned i = 0; i < 10000 && !(mrd(gGcc, GT_GCC_GPLL0_MODE) & (1u << 31)); ++i) IODelay(10);
  const uint32_t votes = GT_VOTE_WRAP0_M_AHB | GT_VOTE_WRAP0_S_AHB | GT_VOTE_WRAP0_CORE | GT_VOTE_WRAP0_CORE_2X | GAH_GCC_VOTE_SE1;
  const uint32_t before = mrd(gGcc, GT_GCC_VOTE_ENABLE);
  mwr(gGcc, GT_GCC_VOTE_ENABLE, before | votes);
  bool ok = false;
  for (unsigned i = 0; i < 2000 && !ok; ++i) {
    ok = !(mrd(gGcc, GT_HALT_CORE) & GT_CBCR_CLK_OFF) && !(mrd(gGcc, GT_HALT_CORE_2X) & GT_CBCR_CLK_OFF) &&
         !(mrd(gGcc, GAH_GCC_HALT_SE1) & GT_CBCR_CLK_OFF);
    if (!ok) IODelay(10);
  }
  note(nullptr, "bus: gcc votes (0x7900c before, se1 cbcr)", ok ? 0 : -110, before, mrd(gGcc, GAH_GCC_HALT_SE1));
  if (!ok) { gBus = -110; return gBus; }
  const bool dfs = (mrd(gGcc, GAH_GCC_RCG_SE1 + GT_RCG_DFSR_CMD) & 1u) != 0;
  unsigned level = 0;
  uint64_t hz = 19200000ULL;
  if (dfs) {
    hz = 0;
    for (unsigned l = 0; l < GT_DFS_LEVELS; ++l) {
      uint64_t f = gt_level_hz(mrd(gGcc, GAH_GCC_RCG_SE1 + GT_RCG_PERF_CFG(l)), mrd(gGcc, GAH_GCC_RCG_SE1 + GT_RCG_PERF_M(l)),
                               mrd(gGcc, GAH_GCC_RCG_SE1 + GT_RCG_PERF_N(l)));
      if (f == 19200000ULL) { level = l; hz = f; break; }
    }
  } else {
    mwr(gGcc, GAH_GCC_RCG_SE1 + GT_RCG_CFG, 0);                          // TCXO, no divider
    mwr(gGcc, GAH_GCC_RCG_SE1 + GT_RCG_CMD, mrd(gGcc, GAH_GCC_RCG_SE1 + GT_RCG_CMD) | 1u);
    for (unsigned i = 0; i < 1000 && (mrd(gGcc, GAH_GCC_RCG_SE1 + GT_RCG_CMD) & 1u); ++i) IODelay(1);
  }
  note(nullptr, "bus: se1 clock (dfs, level) 19.2 MHz", hz == 19200000ULL ? 0 : -22, dfs, level);
  if (hz != 19200000ULL) { gBus = -22; return gBus; }
  const uint32_t pin = gt_pin_cfg(GAH_FUNC_QUP1, 0, 0);                    // qupv3_se1_i2c_active: qup1, 2 mA, no bias
  mwr(gTlmm, gt_pin(GAH_GPIO_I2C_SDA), pin);
  mwr(gTlmm, gt_pin(GAH_GPIO_I2C_SCL), pin);
  gI2c.se = {const_cast<uint8_t *>(gSe), seRd, seWr, delayUs};
  gI2c.wrapper = {const_cast<uint8_t *>(gWrap), seRd, seWr, delayUs};
  gI2c.allow_fifo_disabled = 1;
  int r = geni_i2c_init(&gI2c);
  IOLog("GuampAudioHw: bus: se1 fw 0x%x proto %u if_disable 0x%x depth %u width %u status 0x%x: rc %d\n", gI2c.info.fw_revision,
        gI2c.info.protocol, gI2c.info.if_disable, gI2c.info.tx_fifo_depth, gI2c.info.fifo_width_bits, gI2c.info.geni_status, r);
  if (!r) r = geni_i2c_set_clock(&gI2c, gKhz, level);
  note(nullptr, "bus: i2c ready (kHz)", r, gKhz, 0);
  gBus = r ? r : 1;
  return r;
}

int lpiUp() {
  if (!gLpi) gLpi = mapPhysical(GAH_LPI_BASE, GAH_LPI_SIZE);
  return gLpi ? 0 : -12;
}

int speakerPrepare(const guamp_audio_hw_format *fmt) {
  if (fmt && (fmt->rate != 48000 || fmt->bits != 16 || fmt->channels != 2)) {
    note(nullptr, "speaker prepare: profile is 48 kHz 16 bit stereo only (rate, bits)", -22, fmt->rate, fmt->bits);
    return -22;
  }
  int r = busUp();
  if (r) return r;
  if (lpiUp()) { note(nullptr, "speaker prepare: map lpi", -12, 0, 0); return -12; }
  gah_lpi_i2s1(&gOps, 1);
  r = gah_aw_probe(&gOps, &gChip);
  if (r) return r;
  r = gah_aw_cold(&gOps, gah_aw_att_code(attNowCb()));
  if (r) return r;
  gSpk = 1;
  gMuted = true;
  return 0;
}

bool isEar(uint32_t route) { return route == GUAMP_AUDIO_ROUTE_EARPIECE; }
int32_t rxmLevelDb() { return -static_cast<int32_t>(attNowCb()) / 100; }

// Routes 2/3, prepare (AFE port configured, NOT started yet; the DSP side holds the LPASS votes and has switched
// rx_core/rx_npl on, the hook only confirms): wcd937x reachable, codec power ref, fs-gen, RX macro MCLK, RX SoundWire
// link (pins, clock, master init, RX slave enumeration), interpolators set up and muted. Undone on any failure.
void cdcUndo(unsigned stage) {
  if (stage >= 4) gah_rxm_path(&gOps, 0, isEar(gCdcRoute), 0);
  if (stage >= 3) gRx.bringDown();
  if (stage >= 2) {
    gah_rxm_mclk(&gOps, 0);
    guamp_bolero_fsgen(0);
  }
  if (stage >= 1) guamp_wcd937x_power(0);
}
int codecPrepare(uint32_t route, const guamp_audio_hw_format *fmt) {
  const char *name = isEar(route) ? "earpiece" : "headphone";
  if (gCdc) { note(nullptr, "codec route already prepared (route, asked)", -16, gCdcRoute, route); return -16; }
  const uint32_t want = isEar(route) ? 1u : 2u;
  if (fmt && (fmt->rate != 48000 || fmt->channels != want)) {
    note(nullptr, "codec route: 48 kHz, 1 ch (earpiece) / 2 ch (headphone) only (rate, channels)", -22, fmt->rate, fmt->channels);
    return -22;
  }
  if (!gClock) {
    note(nullptr, "codec route: no LPASS clock hook from the DSP side; RX macro left alone", GUAMP_AUDIO_HW_ABSENT, route, 0);
    return GUAMP_AUDIO_HW_ABSENT;
  }
  int r = gClock(GAH_LPASS_CLK_RX_CORE, GAH_LPASS_CLK_RX_HZ, 1);
  if (!r) r = gClock(GAH_LPASS_CLK_RX_NPL, GAH_LPASS_CLK_RX_HZ, 1);
  note(nullptr, "codec route: rx_core + rx_npl clock (22.5792 MHz)", r, GAH_LPASS_CLK_RX_CORE, GAH_LPASS_CLK_RX_NPL);
  if (r) return r;
  uint8_t id = 0;
  r = guamp_wcd937x_read(WCD_DIG_CHIP_ID0, &id);
  if (r) {
    IOLog("GuampAudioHw: %s: wcd937x registers unreachable (%d): the VA SoundWire link of the capture side is down; "
          "boot with guamp-audio-mic=1 (hold 1)\n", name, r);
    return r;
  }
  gWcdChip = id;
  guamp_wcd937x_power(1);   // a reference only: the capture side brought the codec up and owns its reset
  if (lpiUp()) { note(nullptr, "codec route: map lpi", -12, 0, 0); cdcUndo(1); return -12; }
  if (!gRxmMap) gRxmMap = mapPhysical(GAH_RXM_BASE, 0x1000);
  if (!gVamMap) gVamMap = mapPhysical(GAH_VAM_BASE, 0x1000);
  for (RxWindow &w : gRxWin)
    if (!w.va) w.va = w.phys == guamprx::kRxMacro ? gRxmMap : mapPhysical(w.phys, w.bytes);
  bool mapped = gRxmMap && gVamMap;
  for (const RxWindow &w : gRxWin) mapped = mapped && w.va;
  if (!mapped) { note(nullptr, "codec route: map rx/va macro, rx swr, slew", -12, 0, 0); cdcUndo(1); return -12; }
  gCdcRoute = route;
  guamp_bolero_fsgen(1);
  gah_rxm_mclk(&gOps, 1);
  gRx.bus = {nullptr, rxBusRd, rxBusWr, rxBusDelay, rxBusLog};
  gRx.failedAt = 0;
  r = gRx.bringUp();
  if (r) {
    IOLog("GuampAudioHw: %s: RX SoundWire link failed at step %d (1 master init: RX macro unclocked?, 2 RX slave "
          "missing: wcd937x in reset?), %u accesses outside the windows\n", name, gRx.failedAt, gRxBad);
    cdcUndo(3);
    return r;
  }
  gah_rxm_path(&gOps, 1, isEar(route), rxmLevelDb());
  gCdc = 1;
  gMuted = true;
  IOLog("GuampAudioHw: %s prepared: wcd937x chip id0 0x%02x, rx swr dev %u, digital gain %d dB, PA gain %d dB\n", name,
        id, gRx.swr.devNum, rxmLevelDb(), isEar(route) ? gEarDb : gHphDb);
  return 0;
}
// amp on after AFE DEVICE_START (RX macro clocked by the codec DMA port): wcd937x PA sequence (SoundWire port +
// datapath inside, at Linux's point), then the RX macro unmute. Off: mute first, PA sequence down, datapath off.
int codecAmp(int on) {
  if (!gCdc) return -22;
  const bool ear = isEar(gCdcRoute);
  int r = 0;
  if (on && gCdc == 1) {
    r = ear ? gah_wcd_ear(&gOps, 1, gah_wcd_ear_gain(gEarDb * 100)) : gah_wcd_hph(&gOps, 1, gah_wcd_gain_code(gHphDb * 100));
    if (!r) {
      gah_rxm_mute(&gOps, ear, 0);
      gCdc = 2;
      gMuted = false;
    }
  } else if (!on && gCdc == 2) {
    gah_rxm_mute(&gOps, ear, 1);
    r = ear ? gah_wcd_ear(&gOps, 0, 0) : gah_wcd_hph(&gOps, 0, 0);
    gCdc = 1;
    gMuted = true;
  }
  return r;
}
void codecRelease() {
  if (!gCdc) return;
  if (gCdc == 2) codecAmp(0);
  cdcUndo(4);
  gCdc = 0;
  note(nullptr, "codec route released (route)", 0, gCdcRoute, 0);
}

void probeThread(void *, wait_result_t) {
  if (gDelay) IOSleep(gDelay * 1000u);
  IOLockLock(gLock);
  int r = busUp();
  if (!r) r = gah_aw_probe(&gOps, &gChip);
  uint8_t fsa = 0;
  int f = gBus > 0 ? i2cRead(GAH_FSA_ADDR, 0x00, &fsa, 1) : -1;
  IOLog("GuampAudioHw: PROBE %s: aw882xx id 0x%04x rc %d, fsa4480@42 reg0 0x%02x rc %d (absent is normal: 3.5 mm jack board)\n",
        r ? "FAILED" : "OK", gChip, r, fsa, f);
  IOLockUnlock(gLock);
}

void start() {
  gWcdLock = IOLockAlloc();
  PE_parse_boot_argn("guamp-audio-hw", &gMode, sizeof(gMode));
  if (!gMode) return;
  PE_parse_boot_argn("guamp-audio-hw-att", &gAttDb, sizeof(gAttDb));
  PE_parse_boot_argn("guamp-audio-hw-floor", &gFloorDb, sizeof(gFloorDb));
  PE_parse_boot_argn("guamp-audio-hw-i2c", &gKhz, sizeof(gKhz));
  PE_parse_boot_argn("guamp-audio-hw-delay", &gDelay, sizeof(gDelay));
  PE_parse_boot_argn("guamp-audio-hw-hph", &gHphDb, sizeof(gHphDb));
  PE_parse_boot_argn("guamp-audio-hw-ear", &gEarDb, sizeof(gEarDb));
  if (gHphDb < -30) gHphDb = -30;
  if (gHphDb > 0) gHphDb = 0;
  if (gEarDb < -18) gEarDb = -18;
  if (gEarDb > 6) gEarDb = 6;
  if (gAttDb > 95) gAttDb = 95;
  if (gFloorDb > 95) gFloorDb = 95;
  if (gKhz != 100 && gKhz != 1000) gKhz = 400;
  if (gDelay > 600) gDelay = 600;
  gLock = IOLockAlloc();
  IOLog("GuampAudioHw: mode %u, att %u dB, floor %u dB, i2c %u kHz, hph %d dB, ear %d dB\n", gMode, gAttDb, gFloorDb, gKhz,
        gHphDb, gEarDb);
  if (!gLock) { gMode = 0; return; }
  if (gMode >= 2) {
    thread_t th;
    if (kernel_thread_start(probeThread, nullptr, &th) == KERN_SUCCESS) thread_deallocate(th);
  }
}
}  // namespace audiohw

extern "C" int guamp_audio_hw_prepare(uint32_t route, const struct guamp_audio_hw_format *fmt) {
  using namespace audiohw;
  if (!gMode || !gLock) return GUAMP_AUDIO_HW_ABSENT;
  IOLockLock(gLock);
  IOLog("GuampAudioHw: prepare route %u port 0x%x\n", route, fmt ? fmt->afe_port : 0);
  int r = route == GUAMP_AUDIO_ROUTE_SPEAKER ? speakerPrepare(fmt)
          : (route == GUAMP_AUDIO_ROUTE_HEADPHONE || route == GUAMP_AUDIO_ROUTE_EARPIECE) ? codecPrepare(route, fmt) : -95;
  if (r) gLastRc = r;
  IOLog("GuampAudioHw: prepare route %u: rc %d\n", route, r);
  IOLockUnlock(gLock);
  return r;
}

extern "C" int guamp_audio_hw_amp(uint32_t route, int on) {
  using namespace audiohw;
  if (!gMode || !gLock) return GUAMP_AUDIO_HW_ABSENT;
  IOLockLock(gLock);
  int r = 0;
  if (route != GUAMP_AUDIO_ROUTE_SPEAKER) {
    r = gCdc && route == gCdcRoute ? codecAmp(on) : -22;
    IOLog("GuampAudioHw: codec amp %s route %u: rc %d (state %u)\n", on ? "on" : "off", route, r, gCdc);
    if (r) gLastRc = r;
    IOLockUnlock(gLock);
    return r;
  }
  if (on) {
    if (gSpk < 1) r = -22;
    else if (gSpk == 1) { r = gah_aw_start(&gOps, &gSysst); if (!r) { gSpk = 2; gMuted = false; } }
  } else if (gSpk == 2) {
    r = gah_aw_stop(&gOps);
    gSpk = 1;
    gMuted = true;
  }
  IOLog("GuampAudioHw: amp %s: rc %d (state %u, sysst 0x%04x)\n", on ? "on" : "off", r, gSpk, gSysst);
  if (r) gLastRc = r;
  IOLockUnlock(gLock);
  return r;
}

extern "C" int guamp_audio_hw_volume(uint32_t route, int32_t centibel) {
  using namespace audiohw;
  if (!gMode || !gLock) return GUAMP_AUDIO_HW_ABSENT;
  IOLockLock(gLock);
  gLevelCb = centibel;
  int r = 0;
  if (route == GUAMP_AUDIO_ROUTE_SPEAKER && gSpk) r = gah_aw_set_att(&gOps, gah_aw_att_code(attNowCb()));
  else if (route != GUAMP_AUDIO_ROUTE_SPEAKER && gCdc && route == gCdcRoute) gah_rxm_volume(&gOps, isEar(route), rxmLevelDb());
  IOLog("GuampAudioHw: volume route %u request %d cb -> attenuation %u cb (floor %u): rc %d\n", route, centibel, attNowCb(), floorCb(), r);
  IOLockUnlock(gLock);
  return r;
}

extern "C" void guamp_audio_hw_release(uint32_t route) {
  using namespace audiohw;
  if (!gMode || !gLock) return;
  IOLockLock(gLock);
  if (route == GUAMP_AUDIO_ROUTE_SPEAKER && gSpk) {
    if (gSpk == 2) gah_aw_stop(&gOps);
    if (gLpi) gah_lpi_i2s1(&gOps, 0);
    gSpk = 0;
    gMuted = true;
    IOLog("GuampAudioHw: speaker released (amp PWDN 1, i2s1 pins sleep)\n");
  } else if (route != GUAMP_AUDIO_ROUTE_SPEAKER && route == gCdcRoute) {
    codecRelease();
  }
  IOLockUnlock(gLock);
}

extern "C" int guamp_audio_hw_mute(uint32_t route, int mute) {
  using namespace audiohw;
  if (!gMode || !gLock) return GUAMP_AUDIO_HW_ABSENT;
  IOLockLock(gLock);
  int r = -22;
  if (route == GUAMP_AUDIO_ROUTE_SPEAKER && gSpk == 2) { r = gah_aw_mute(&gOps, mute); if (!r) gMuted = mute != 0; }
  else if (route != GUAMP_AUDIO_ROUTE_SPEAKER && gCdc == 2 && route == gCdcRoute) {
    gah_rxm_mute(&gOps, isEar(route), mute);
    gMuted = mute != 0;
    r = 0;
  }
  IOLockUnlock(gLock);
  return r;
}

extern "C" int guamp_audio_hw_get_state(uint32_t route, struct guamp_audio_hw_state *st) {
  using namespace audiohw;
  if (!st) return -22;
  st->enabled = gMode;
  const bool spk = route == GUAMP_AUDIO_ROUTE_SPEAKER, mine = !spk && gCdc && route == gCdcRoute;
  st->prepared = spk ? gSpk >= 1 : mine;
  st->running = spk ? gSpk == 2 : mine && gCdc == 2;
  st->muted = gMuted;
  st->chip_id = spk ? gChip : gWcdChip;
  st->att_cb = attNowCb();
  st->sysst = gSysst;
  st->last_rc = gLastRc;
  return gMode ? 0 : GUAMP_AUDIO_HW_ABSENT;
}

extern "C" void guamp_audio_hw_set_lpass_clock_hook(guamp_audio_lpass_clock_fn fn) { audiohw::gClock = fn; }

// wcd937x power (wcd937x.c probe + wcd937x_reset, guamp DT): cdc-vdd-rxtx/vddpx = L9A 1.8 V (static), cdc-vdd-buck =
// L14A 1.8 V, one RPM vote each (active set, swen 1, as BtService.inc votes its rails; never voted off: L9A is the panel
// VDDIO too), then reset gpio92 sleep -> 20 us -> active -> 20 us. Refcounted: only the first user resets the codec,
// the last one puts it back in reset (SoundWire slave detached; the next user enumerates again).
extern "C" int guamp_wcd937x_power(int on) {
  using namespace audiohw;
  if (!gWcdLock) return GUAMP_AUDIO_HW_ABSENT;
  IOLockLock(gWcdLock);
  int r = 0;
  if (!gTlmm) gTlmm = mapPhysical(GT_TLMM_WEST, GAH_TLMM_WEST_SIZE);
  if (!gTlmm) {
    r = -12;
  } else if (on) {
    if (gWcdUsers++ == 0) {
      if (!gWcdRails) {
        const rpm_kvp kv[2] = {{RPM_KEY_SWEN, 4, 1}, {RPM_KEY_UV, 4, 1800000}};
        const int l9 = rpm::request(RPM_SET_ACTIVE, RPM_RES_LDOA, 9, kv, 2, 500);
        const int l14 = rpm::request(RPM_SET_ACTIVE, RPM_RES_LDOA, 14, kv, 2, 500);
        IOLog("GuampAudioHw: wcd937x rails: rpm L9A %s, L14A %s (1.8 V, swen 1)\n", rpm_err_name(l9), rpm_err_name(l14));
        gWcdRails = !l9 && !l14;
        if (!gWcdRails) r = l9 ? l9 : l14;   // logged; the reset is pulsed anyway (ABL may have left the rails on)
      }
      gpioOut(nullptr, GAH_GPIO_WCD_RESET, 0);
      IODelay(20);
      gpioOut(nullptr, GAH_GPIO_WCD_RESET, 1);
      IODelay(20);
      IOLog("GuampAudioHw: wcd937x power on: reset gpio92 low 20 us, high 20 us (rails rc %d)\n", r);
      r = 0;
    }
  } else if (gWcdUsers && --gWcdUsers == 0) {
    gpioOut(nullptr, GAH_GPIO_WCD_RESET, 0);
    IOLog("GuampAudioHw: wcd937x power off: reset gpio92 low (rails left on)\n");
  }
  const unsigned users = gWcdUsers;
  IOLockUnlock(gWcdLock);
  if (on && users > 1) IOLog("GuampAudioHw: wcd937x power: already on (%u users)\n", users);
  return r;
}
