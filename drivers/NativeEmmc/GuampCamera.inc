// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp camera, step 1: PM8008 rails, MCLK, reset/pwdn GPIOs, CCI masters 0/1 and the chip id of the four sensors,
// then everything off again. No CSIPHY/CSID/TFE, no streaming (the RDI skeleton is GuampCameraRdi.h, unused here).
// Included by NativeEmmc.cpp AFTER GuampAudioHw.inc: the PM8008 sits on the same QUP0 SE1 I2C bus as the aw882xx
// amplifier, so this file uses audiohw::busUp()/gI2c under the audio lock (or its own lock when audio is off).
// Sequences and board facts: GuampCameraCore.h. Every step logs one "GuampCamera:" line.
// Boot arguments:
//   guamp-camera=0        default: nothing is touched
//   guamp-camera=1        probe thread: PM8008 on, GDSC + CCI clock, CCI init, per sensor power up -> chip id ->
//                         power down, then CCI clock/GDSC off and PM8008 chip-enable back as found
//   guamp-camera-delay=<s>   start the probe later (default 15, 0..600)
//   guamp-camera-mask=<m>    sensors to probe, bit = cell (default 0xf: gm1st, 4h7, ov02b1b, ov02b10)
//   guamp-camera-mclk=24|19  24 MHz from GPLL9 (default; falls back to 19.2 MHz TCXO if GPLL9 does not lock) or 19.2
//   guamp-camera-cci=100|400|1000  CCI timing set (default 400; the CCI clock is the 19.2 MHz TCXO, so the bus
//                         runs at about half the nominal rate)
//   guamp-camera-pmgpio=1    also program PM8008 GPIO1 as the DT does (func1 output); default: only log it
#include "GuampCameraCore.h"
#include "GuampCameraRdi.h"   // skeleton only (step 2), nothing in this file calls it

namespace camera {
// GCC (gcc-bengal.c)
constexpr uint32_t kGccGpll0Vote = 0x79000;      // bit 0 gpll0, bit 9 gpll9
constexpr uint32_t kGpll9 = 0x9000;              // brammo PLL: mode 0, L 4, alpha 8/0xc, user 0x10, config 0x18, test 0x1c
constexpr uint32_t kGdscCamssTop = 0x58004;      // gcc_camss_top_gdsc (DT qcom,gdsc@1458004): bit 0 SW_COLLAPSE, bit 31 PWR_ON
constexpr uint32_t kCbcrCameraAhb = 0x17008, kCbcrTopAhb = 0x58028, kRcgTopAhb = 0x58010;
constexpr uint32_t kRcgCci = 0x56000, kCbcrCci = 0x56018;
constexpr uint32_t kRcgMclk[4] = {0x51000, 0x5101c, 0x51038, 0x51054};
constexpr uint32_t kCbcrMclk[4] = {0x51018, 0x51034, 0x51050, 0x5106c};
constexpr uint32_t kCbcrOff = 1u << 31;
constexpr uint64_t kCciPhys = 0x05c1b000;
// TLMM tiles (pinctrl-bengal.c): EAST = 0xd00000 (gpio 18..30), WEST = 0x500000 (gpio 68), SOUTH = 0x900000 (gpio 103)
constexpr uint64_t kTlmmEast = 0x00d00000, kTlmmWest = 0x00500000;
constexpr unsigned kGpioPm8008Reset = 26;        // pm8008_active: output high, pull-up, 2 mA
constexpr unsigned kCciPins[2][2] = {{22, 23}, {29, 30}};   // cci0/cci1 sda, scl: function cci_i2c (slot 1), pull-up

unsigned gMode, gDelay = 15, gMask = 0xf, gMclkMhz = 24, gCciKhz = 400, gPmGpio;
IOLock *gOwnLock;
volatile uint8_t *gGcc, *gCci, *gPin[128];
bool gMclk24;                       // GPLL9 locked: 24 MHz, else 19.2 MHz
gcam::LdoState gLdo;

uint32_t rd(volatile uint8_t *b, uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(b + off); }
void wr(volatile uint8_t *b, uint32_t off, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(b + off) = v; __asm__ volatile("dsb sy" ::: "memory"); }

volatile uint8_t *pin(unsigned gpio) {
  if (gpio >= 128) return nullptr;
  if (!gPin[gpio]) gPin[gpio] = mapPhysical((gpio == 68 ? kTlmmWest : gpio == 103 ? 0x00900000ull : kTlmmEast) + gpio * 0x1000ull, 0x1000);
  return gPin[gpio];
}
void pinCfg(unsigned gpio, uint32_t cfg) { if (volatile uint8_t *p = pin(gpio)) wr(p, 0, cfg); }
void pinOut(unsigned gpio, int high) {
  volatile uint8_t *p = pin(gpio);
  if (!p) return;
  const uint32_t cfg = gt_pin_cfg(0, 0, 0) | GT_PIN_OE;   // gpio function, 2 mA, no pull, output
  if (rd(p, 0) != cfg) wr(p, 0, cfg);
  wr(p, 4, high ? 2u : 0u);
}

// --- ops for GuampCameraCore.h ---------------------------------------------------------------------------------------
int pmRead(void *, uint8_t dev, uint16_t reg, uint8_t *v) {
  uint8_t a[2] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg)};
  geni_i2c_msg m[2] = {{a, 2, 0}, {v, 1, 1}};
  int r = -1;
  for (unsigned t = 0; t < 3 && r; ++t) r = geni_i2c_xfer(&audiohw::gI2c, dev, m, 2);
  return r;
}
int pmWrite(void *, uint8_t dev, uint16_t reg, uint8_t v) {
  uint8_t b[3] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg), v};
  geni_i2c_msg m = {b, 3, 0};
  int r = -1;
  for (unsigned t = 0; t < 3 && r; ++t) r = geni_i2c_xfer(&audiohw::gI2c, dev, &m, 1);
  return r;
}
uint32_t cciRd(void *, uint32_t off) { return rd(gCci, off); }
void cciWr(void *, uint32_t off, uint32_t v) { wr(gCci, off, v); }
void gpio(void *, unsigned g, int high) { pinOut(g, high); }
void delayUs(void *, unsigned us) { IODelay(us); }
void sleepMs(void *, unsigned ms) { IOSleep(ms ? ms : 1); }
void note(void *, const char *step, int rc, uint32_t a, uint32_t b) { IOLog("GuampCamera: %s: rc %d (0x%x 0x%x)\n", step, rc, a, b); }

bool branch(uint32_t cbcr, bool on) {
  wr(gGcc, cbcr, on ? (rd(gGcc, cbcr) | 1u) : (rd(gGcc, cbcr) & ~1u));
  for (unsigned i = 0; i < 500; ++i) {
    const bool off = rd(gGcc, cbcr) & kCbcrOff;
    if (off != on) return true;
    IODelay(2);
  }
  return false;
}
bool rcgUpdate(uint32_t rcg) {
  wr(gGcc, rcg, rd(gGcc, rcg) | 1u);
  for (unsigned i = 0; i < 1000; ++i) { if (!(rd(gGcc, rcg) & 1u)) return true; IODelay(1); }
  return false;
}

// GPLL9 (576 MHz main output) for the 24 MHz MCLK: configured like gpll9_config only when L is still 0, then voted.
bool gpll9() {
  if (rd(gGcc, kGpll9) & (1u << 31)) return true;
  if (!rd(gGcc, kGpll9 + 4)) {
    wr(gGcc, kGpll9 + 4, 0x3c);                                       // L: 60 x 19.2 = 1152 MHz
    wr(gGcc, kGpll9 + 8, 0);
    wr(gGcc, kGpll9 + 0xc, 0);
    wr(gGcc, kGpll9 + 0x10, (rd(gGcc, kGpll9 + 0x10) & ~0x300u) | 0x100u | 1u);   // post div 2, main output
    wr(gGcc, kGpll9 + 0x18, 0x00004289);
    wr(gGcc, kGpll9 + 0x1c, 0x08000000);
  }
  wr(gGcc, kGccGpll0Vote, rd(gGcc, kGccGpll0Vote) | (1u << 9));
  for (unsigned i = 0; i < 1000; ++i) { if (rd(gGcc, kGpll9) & (1u << 31)) return true; IODelay(2); }
  return false;
}

int mclk(void *, unsigned idx, unsigned g, int on) {
  if (idx > 3) return gcam::kErrInval;
  if (!on) {
    branch(kCbcrMclk[idx], false);
    pinCfg(g, gt_pin_cfg(1, 0, 1));                                   // cam_mclk suspend: pull-down
    return 0;
  }
  const uint32_t r = kRcgMclk[idx];
  if (gMclk24) {
    wr(gGcc, r + 8, 1);                                               // M
    wr(gGcc, r + 0xc, ~(24u - 1u) & 0xffu);                           // N = ~(n - m)
    wr(gGcc, r + 0x10, ~24u & 0xffu);                                 // D = ~n
    wr(gGcc, r + 4, 5u << 8 | 2u << 12 | 1u);                         // GPLL9_OUT_MAIN, dual edge, div 1
  } else {
    wr(gGcc, r + 4, 0);                                               // TCXO 19.2 MHz
  }
  if (!rcgUpdate(r) || !branch(kCbcrMclk[idx], true)) {
    note(nullptr, "mclk clock (index, cbcr)", gcam::kErrTimeout, idx, rd(gGcc, kCbcrMclk[idx]));
    return gcam::kErrTimeout;
  }
  pinCfg(g, gt_pin_cfg(1, 0, 0));                                     // cam_mclk, 2 mA, no pull
  return 0;
}

const gcam::Ops gOps = {nullptr, pmRead, pmWrite, cciRd, cciWr, gpio, mclk, delayUs, sleepMs, note};

// GDSC + AHB + CCI clock; CCI registers are touched only when all of them report on.
bool camssUp() {
  wr(gGcc, kGdscCamssTop, rd(gGcc, kGdscCamssTop) & ~1u);
  bool gdsc = false;
  for (unsigned i = 0; i < 1000 && !gdsc; ++i) { gdsc = rd(gGcc, kGdscCamssTop) & (1u << 31); if (!gdsc) IODelay(2); }
  const bool cam = branch(kCbcrCameraAhb, true);
  wr(gGcc, kRcgTopAhb + 4, 0);                                        // TCXO
  const bool top = rcgUpdate(kRcgTopAhb) && branch(kCbcrTopAhb, true);
  wr(gGcc, kRcgCci + 4, 0);                                           // TCXO 19.2 MHz (37.5 MHz needs GPLL0 aux2)
  const bool cci = rcgUpdate(kRcgCci) && branch(kCbcrCci, true);
  IOLog("GuampCamera: camss: gdsc %s (0x%x), camera_ahb %d, top_ahb %d, cci_0_clk %d\n", gdsc ? "on" : "OFF",
        rd(gGcc, kGdscCamssTop), cam, top, cci);
  return gdsc && top && cci;
}
void camssDown() {
  branch(kCbcrCci, false);
  branch(kCbcrTopAhb, false);
  wr(gGcc, kGdscCamssTop, rd(gGcc, kGdscCamssTop) | 1u);
  IOLog("GuampCamera: camss: cci clock, top ahb off, gdsc collapse requested (0x%x)\n", rd(gGcc, kGdscCamssTop));
}

void cciPins(bool active) {
  for (unsigned m = 0; m < 2; ++m)
    for (unsigned k = 0; k < 2; ++k) pinCfg(kCciPins[m][k], gt_pin_cfg(1, 0, active ? 3 : 1));   // cci_i2c, 2 mA
}

void pmGpio1() {
  uint8_t sub = 0, mode = 0, src = 0, en = 0;
  pmRead(nullptr, gcam::kPmChip, gcam::kPmGpio1 + 5, &sub);
  pmRead(nullptr, gcam::kPmChip, gcam::kPmGpio1 + 0x40, &mode);
  pmRead(nullptr, gcam::kPmChip, gcam::kPmGpio1 + 0x44, &src);
  pmRead(nullptr, gcam::kPmChip, gcam::kPmGpio1 + 0x46, &en);
  IOLog("GuampCamera: pm8008 gpio1: subtype 0x%x mode 0x%x out_src 0x%x en 0x%x%s\n", sub, mode, src, en,
        gPmGpio ? ", programming func1 output (guamp-camera-pmgpio=1)" : "");
  if (!gPmGpio) return;
  pmWrite(nullptr, gcam::kPmChip, gcam::kPmGpio1 + 0x40, 1);           // digital output
  pmWrite(nullptr, gcam::kPmChip, gcam::kPmGpio1 + 0x41, 1);           // power-source 1
  pmWrite(nullptr, gcam::kPmChip, gcam::kPmGpio1 + 0x42, 5);           // bias disable
  pmWrite(nullptr, gcam::kPmChip, gcam::kPmGpio1 + 0x44, 2);           // func1
  pmWrite(nullptr, gcam::kPmChip, gcam::kPmGpio1 + 0x46, 0x80);        // enable
}

void probeThread(void *, wait_result_t) {
  if (gDelay) IOSleep(gDelay * 1000u);
  IOLock *lock = audiohw::gLock ? audiohw::gLock : gOwnLock;
  IOLockLock(lock);
  unsigned ok = 0, tried = 0;
  bool chipWasOn = false, pmUp = false, camss = false;
  int r = audiohw::busUp();
  if (r) { IOLog("GuampCamera: RESULT FAILED: SE1 I2C bus %d\n", r); IOLockUnlock(lock); return; }
  gGcc = mapPhysical(GT_GCC_BASE, 0x80000);
  gCci = mapPhysical(kCciPhys, 0x1000);
  if (!gGcc || !gCci) { IOLog("GuampCamera: RESULT FAILED: map gcc/cci\n"); IOLockUnlock(lock); return; }
  // PM8008: reset_n high (pm8008_active), revid, chip enable
  pinOut(kGpioPm8008Reset, 1);
  IOSleep(5);
  uint8_t type = 0, sub = 0, en = 0;
  r = pmRead(nullptr, gcam::kPmChip, gcam::kPmRevidType, &type);
  if (!r) r = pmRead(nullptr, gcam::kPmChip, gcam::kPmRevidSubtype, &sub);
  if (!r) r = pmRead(nullptr, gcam::kPmChip, gcam::kPmChipEnable, &en);
  chipWasOn = en & 1;
  IOLog("GuampCamera: pm8008 revid type 0x%x subtype 0x%x, chip-en 0x%x: rc %d\n", type, sub, en, r);
  if (!r) { pmGpio1(); r = gcam::pmChip(gOps, true); }
  pmUp = !r;
  if (!pmUp) IOLog("GuampCamera: RESULT FAILED: pm8008 not reachable (rc %d)\n", r);
  if (pmUp) camss = camssUp();
  if (pmUp && !camss) IOLog("GuampCamera: RESULT FAILED: camss clocks/gdsc, CCI not touched\n");
  if (camss) {
    gMclk24 = gMclkMhz == 24 && gpll9();
    IOLog("GuampCamera: mclk %s\n", gMclk24 ? "24 MHz (GPLL9 576 MHz / 24)" : "19.2 MHz (TCXO)");
    const gcam::CciTiming &t = gCciKhz == 100 ? gcam::kCciStandard : gCciKhz == 1000 ? gcam::kCciFastPlus : gcam::kCciFast;
    cciPins(true);
    gcam::Cci cci = {&gOps};
    r = cci.init(t);
    IOLog("GuampCamera: cci hw version 0x%x, init rc %d (status 0x%x), timing %u kHz set\n", rd(gCci, gcam::kCciHwVersion), r,
          cci.lastStatus, gCciKhz);
    for (unsigned i = 0; !r && i < gcam::kSensors; ++i) {
      const gcam::Sensor &s = gcam::kBoard[i];
      if (!(gMask & (1u << s.cell))) continue;
      ++tried;
      gcam::ProbeResult p = gcam::probe(gOps, gLdo, cci, s, t);
      if (!p.rc) ++ok;
      IOLog("GuampCamera: %s chip id 0x%04x (expected 0x%04x, cci%u slave 0x%02x reg 0x%04x): %s rc %d status 0x%x\n", s.name, p.id, s.id,
            s.cci, s.slave7, s.idReg, p.rc == 0 ? "OK" : p.rc == gcam::kErrMismatch ? "WRONG ID" : p.rc == gcam::kErrNack ? "NACK" : "FAILED",
            p.rc, p.cciStatus);
    }
    cciPins(false);
    camssDown();
  }
  if (pmUp) {
    for (unsigned n = 1; n <= gcam::kLdos; ++n)
      while (gLdo.users[n - 1]) gcam::ldoOff(gOps, gLdo, n);       // nothing should be left; force it anyway
    if (!chipWasOn) gcam::pmChip(gOps, false);
  }
  IOLog("GuampCamera: RESULT %u/%u sensors answered; off: ldo users %u, pm8008 chip-en %s, mclk off, camss off\n", ok, tried,
        gcam::ldoUsers(gLdo), pmUp && !chipWasOn ? "cleared" : chipWasOn ? "left on (was on)" : "untouched");
  IOLockUnlock(lock);
}

void start() {
  PE_parse_boot_argn("guamp-camera", &gMode, sizeof(gMode));
  if (!gMode) return;
  PE_parse_boot_argn("guamp-camera-delay", &gDelay, sizeof(gDelay));
  PE_parse_boot_argn("guamp-camera-mask", &gMask, sizeof(gMask));
  PE_parse_boot_argn("guamp-camera-mclk", &gMclkMhz, sizeof(gMclkMhz));
  PE_parse_boot_argn("guamp-camera-cci", &gCciKhz, sizeof(gCciKhz));
  PE_parse_boot_argn("guamp-camera-pmgpio", &gPmGpio, sizeof(gPmGpio));
  if (gDelay > 600) gDelay = 600;
  if (gCciKhz != 100 && gCciKhz != 1000) gCciKhz = 400;
  gMask &= 0xf;
  gOwnLock = IOLockAlloc();
  IOLog("GuampCamera: mode %u, delay %u s, mask 0x%x, mclk %u MHz, cci %u kHz, pmgpio %u (step 1: chip id only)\n", gMode, gDelay,
        gMask, gMclkMhz, gCciKhz, gPmGpio);
  if (!gOwnLock && !audiohw::gLock) return;
  thread_t th;
  if (kernel_thread_start(probeThread, nullptr, &th) == KERN_SUCCESS) thread_deallocate(th);
}
}  // namespace camera
