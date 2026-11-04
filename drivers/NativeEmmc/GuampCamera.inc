// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp camera: PM8008 rails, MCLK, reset/pwdn GPIOs, CCI masters 0/1, chip id of the four sensors (step 1) and the
// sensor side of streaming (init, mode, exposure/gain, stream on/off; GuampCameraSensor.h). No CSIPHY/CSID/TFE here:
// the MIPI facts of the chosen mode are logged and returned by sensorOpen() for the CAMSS code (RDI skeleton:
// GuampCameraRdi.h, unused here).
// Included by NativeEmmc.cpp AFTER GuampAudioHw.inc: the PM8008 sits on the same QUP0 SE1 I2C bus as the aw882xx
// amplifier, so every PM8008 transfer takes the audio lock (or its own lock when audio is off) around
// audiohw::busUp()/gI2c. Sequences and board facts: GuampCameraCore.h / GuampCameraSensor.h. Each step logs one
// "GuampCamera:" line.
// Boot arguments:
//   guamp-camera=0        default: nothing is touched
//   guamp-camera=1        probe thread: PM8008 on, GDSC + CCI clock, CCI init, per sensor power up -> chip id ->
//                         power down, then CCI clock/GDSC off and PM8008 chip-enable back as found
//   guamp-camera=2        probe as 1, then a session on one sensor: power up, chip id, init + mode + exposure from
//                         the GCAM table (/chosen/guamp-camera-gcam), status readback; streaming only with
//                         guamp-camera-stream=1; then everything off
//   guamp-camera-delay=<s>   start the probe later (default 15, 0..600)
//   guamp-camera-mask=<m>    sensors to probe, bit = cell (default 0xf: gm1st, 4h7, ov02b1b, ov02b10)
//   guamp-camera-mclk=24|19  24 MHz from GPLL9 (default; falls back to 19.2 MHz TCXO if GPLL9 does not lock) or 19.2
//                         (sensor PLLs are set for 24 MHz: at 19.2 every lane rate and fps is 0.8x)
//   guamp-camera-cci=100|400|1000  CCI timing set (default 400; the CCI clock is the 19.2 MHz TCXO, so the bus
//                         runs at about half the nominal rate)
//   guamp-camera-af=1|3      after the sensors: lens driver + EEPROM AF probe (GuampCamera3A.inc), 3 = also macro
//   guamp-camera-pmgpio=1    also program PM8008 GPIO1 as the DT does (func1 output); default: only log it
//   guamp-camera-sensor=<cell>  session sensor for mode 2 (default 0 = s5kgm1st; 1 s5k4h7, 2 ov02b1b, 3 ov02b10)
//   guamp-camera-res=0|1     0 low (default: gm1st 1280x720, 4h7 1632x1224 binned, ov02b 1600x1200), 1 full
//   guamp-camera-modeidx=<n> force vendor mode n of the table (default: from guamp-camera-res)
//   guamp-camera-stream=0|1  1: stream on for guamp-camera-stream-ms (default 1000, at most 10000), frame counter
//                         read before and after (SMIA sensors), then stream off. Default 0: never streams
//   guamp-camera-exp-us=<us> first exposure (default 10000), guamp-camera-gain=<Q8> first gain (default 256 = 1x)
#include "GuampCameraCore.h"
#include "GuampCameraSensor.h"
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
unsigned gSensor, gRes, gModeIdx = 0xff, gStream, gStreamMs = 1000, gExpUs = 10000, gGainQ8 = 256;
IOLock *gOwnLock, *gSessionLock;
IOLock *busLock() { return audiohw::gLock ? audiohw::gLock : gOwnLock; }
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
// One PM8008 transfer under the SE1 bus lock (shared with the amplifier), 3 tries.
int pmXfer(uint8_t dev, geni_i2c_msg *m, unsigned n) {
  IOLock *lock = busLock();
  IOLockLock(lock);
  int r = audiohw::busUp();
  if (!r) { r = -1; for (unsigned t = 0; t < 3 && r; ++t) r = geni_i2c_xfer(&audiohw::gI2c, dev, m, n); }
  IOLockUnlock(lock);
  return r;
}
int pmRead(void *, uint8_t dev, uint16_t reg, uint8_t *v) {
  uint8_t a[2] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg)};
  geni_i2c_msg m[2] = {{a, 2, 0}, {v, 1, 1}};
  return pmXfer(dev, m, 2);
}
int pmWrite(void *, uint8_t dev, uint16_t reg, uint8_t v) {
  uint8_t b[3] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg), v};
  geni_i2c_msg m = {b, 3, 0};
  return pmXfer(dev, &m, 1);
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

namespace af { void probe(gcam::Cci &cci, const gcam::CciTiming &t); void start(); void lensClose(); }   // GuampCamera3A.inc

// ---- hardware up/down (shared by the probe, the session test and the API) ------------------------------------------
bool gHwUp, gChipWasOn, gPmUp, gCamss;
gcam::Cci gCciDev = {nullptr};
const gcam::CciTiming *gTiming = &gcam::kCciFast;

void hwDown() {
  if (gCamss) { cciPins(false); camssDown(); gCamss = false; }
  if (gPmUp) {
    for (unsigned n = 1; n <= gcam::kLdos; ++n)
      while (gLdo.users[n - 1]) gcam::ldoOff(gOps, gLdo, n);       // nothing should be left; force it anyway
    if (!gChipWasOn) gcam::pmChip(gOps, false);
    IOLog("GuampCamera: off: ldo users %u, pm8008 chip-en %s, mclk off, camss off\n", gcam::ldoUsers(gLdo),
          gChipWasOn ? "left on (was on)" : "cleared");
  }
  gPmUp = false;
  gHwUp = false;
}

// PM8008 reachable and enabled, GDSC + clocks on, MCLK source chosen, CCI pins and block initialised.
int hwUp() {
  if (gHwUp) return 0;
  gGcc = mapPhysical(GT_GCC_BASE, 0x80000);
  gCci = mapPhysical(kCciPhys, 0x1000);
  if (!gGcc || !gCci) { IOLog("GuampCamera: RESULT FAILED: map gcc/cci\n"); return gcam::kErrNoMem; }
  // PM8008: reset_n high (pm8008_active), revid, chip enable
  pinOut(kGpioPm8008Reset, 1);
  IOSleep(5);
  uint8_t type = 0, sub = 0, en = 0;
  int r = pmRead(nullptr, gcam::kPmChip, gcam::kPmRevidType, &type);
  if (!r) r = pmRead(nullptr, gcam::kPmChip, gcam::kPmRevidSubtype, &sub);
  if (!r) r = pmRead(nullptr, gcam::kPmChip, gcam::kPmChipEnable, &en);
  gChipWasOn = en & 1;
  IOLog("GuampCamera: pm8008 revid type 0x%x subtype 0x%x, chip-en 0x%x: rc %d\n", type, sub, en, r);
  if (!r) { pmGpio1(); r = gcam::pmChip(gOps, true); }
  gPmUp = !r;
  if (!gPmUp) { IOLog("GuampCamera: RESULT FAILED: pm8008 not reachable (rc %d)\n", r); return r ? r : gcam::kErrIo; }
  gCamss = camssUp();
  if (!gCamss) { IOLog("GuampCamera: RESULT FAILED: camss clocks/gdsc, CCI not touched\n"); hwDown(); return gcam::kErrTimeout; }
  gMclk24 = gMclkMhz == 24 && gpll9();
  IOLog("GuampCamera: mclk %s\n", gMclk24 ? "24 MHz (GPLL9 576 MHz / 24)" : "19.2 MHz (TCXO): sensor lane rates and fps x0.8");
  gTiming = gCciKhz == 100 ? &gcam::kCciStandard : gCciKhz == 1000 ? &gcam::kCciFastPlus : &gcam::kCciFast;
  cciPins(true);
  gCciDev = {&gOps};
  r = gCciDev.init(*gTiming);
  IOLog("GuampCamera: cci hw version 0x%x, init rc %d (status 0x%x, rst_done %d), timing %u kHz set\n", rd(gCci, gcam::kCciHwVersion), r,
        gCciDev.lastStatus, gCciDev.resetSeen, gCciKhz);
  gHwUp = true;
  if (r) { hwDown(); return r; }
  return 0;
}

const char *rcName(int rc) {
  return rc == 0 ? "OK" : rc == gcam::kErrMismatch ? "WRONG ID" : rc == gcam::kErrNack ? "NACK" : rc == gcam::kErrTimeout ? "TIMEOUT" : "FAILED";
}

// Chip id of every sensor in the mask (step 1). Returns the number that answered.
unsigned probeAll(unsigned *tried) {
  unsigned ok = 0;
  *tried = 0;
  for (unsigned i = 0; i < gcam::kSensors; ++i) {
    const gcam::Sensor &s = gcam::kBoard[i];
    if (!(gMask & (1u << s.cell))) continue;
    ++*tried;
    gcam::ProbeResult p = gcam::probe(gOps, gLdo, gCciDev, s, *gTiming);
    if (!p.rc) ++ok;
    IOLog("GuampCamera: %s chip id 0x%04x (expected 0x%04x, cci%u slave 0x%02x reg 0x%04x): %s rc %d status 0x%x\n", s.name, p.id, s.id,
          s.cci, s.slave7, s.idReg, rcName(p.rc), p.rc, p.cciStatus);
  }
  return ok;
}

// ---- GCAM table from the ADT (make-guamp-adt.py copies the local guamp-camera.gcam; never in the repo) --------------
const uint8_t *gGcamData;
uint32_t gGcamLen;
bool gcamLoad() {
  if (gGcamData) return true;
  IORegistryEntry *chosen = IORegistryEntry::fromPath("/chosen", gIODTPlane);   // kept retained: the data stays in use
  OSData *data = chosen ? OSDynamicCast(OSData, chosen->getProperty("guamp-camera-gcam")) : nullptr;
  if (!data || data->getLength() < 12) { IOLog("GuampCamera: no /chosen/guamp-camera-gcam: no sensor tables, chip id only\n"); return false; }
  gGcamData = static_cast<const uint8_t *>(data->getBytesNoCopy());
  gGcamLen = data->getLength();
  IOLog("GuampCamera: gcam table %u bytes, version %u, %u sensors\n", gGcamLen, gcam::le32(gGcamData + 4), gcam::le32(gGcamData + 8));
  return true;
}

// ---- session API: one sensor at a time, for the CAMSS (agent 2), 3A (agent 4) and iOS service code ---------------
gcam::Session gSess;
gcam::Gcam gTable;

void logStatus(const char *when) {
  gcam::Status st;
  const int r = gSess.status(&st);
  IOLog("GuampCamera: %s status %s: state %u, id 0x%04x, mode_select %u, frame count %u, out %ux%u, fll %u, lines %u, gain code 0x%x: rc %d\n",
        gSess.s ? gSess.s->name : "-", when, st.state, st.id, st.modeSelect, st.frameCount, st.width, st.height, st.fll, st.lines,
        st.gainCode, r);
}

void logMipi(const gcam::Mipi &m) {
  IOLog("GuampCamera: mipi: cell %u csiphy %u, %u lane (assign 0x%04x, combo %u), %u Mbit/s per lane, dt 0x%02x vc %u, %ux%u RAW10 "
        "(stride %u), settle %u, %u.%03u fps, cfa %u, extra %ux%u dt 0x%02x%s\n", m.cell, m.csiphy, m.lanes, m.laneAssign, m.combo,
        gMclk24 ? m.laneMbps : m.laneMbps * 4 / 5, m.dt, m.vc, m.width, m.height, m.strideBytes, m.settle, m.fpsMilli / 1000,
        m.fpsMilli % 1000, m.cfa, m.extraW, m.extraH, m.extraDt, gMclk24 ? "" : " (MCLK 19.2: x0.8)");
}

// Powers and configures one sensor, left in standby (not streaming). res 0 low / 1 full, modeIdx forces a vendor mode
// (>= the table's count: from res). mipi (optional) receives what CSIPHY/CSID need.
int sensorOpen(unsigned cell, unsigned res, unsigned modeIdx, uint32_t expUs, uint32_t gainQ8, gcam::Mipi *mipi) {
  if (cell >= gcam::kSensors) return gcam::kErrInval;
  if (!gSessionLock) return gcam::kErrNoMem;
  IOLockLock(gSessionLock);
  const gcam::Sensor &s = gcam::kBoard[cell];
  int r = gSess.state != gcam::kOff ? gcam::kErrInval : 0;
  if (!r && !gcamLoad()) r = gcam::kErrInval;
  if (!r) {
    r = gcam::gcamFind(gGcamData, gGcamLen, s.name, &gTable);
    if (r) IOLog("GuampCamera: %s not in the gcam table (version 2 expected): rc %d\n", s.name, r);
  }
  if (!r) r = hwUp();
  if (!r) {
    r = gSess.open(gOps, gLdo, gCciDev, *gTiming, s, &gTable);
    IOLog("GuampCamera: %s open (power up + chip id): %s rc %d\n", s.name, rcName(r), r);
  }
  if (!r) {
    const gcam::Res rs = res ? gcam::kResFull : gcam::kResLow;
    const int idx = modeIdx < gTable.nModes ? static_cast<int>(modeIdx) : -1;
    gcam::Mode m = {};
    gcam::Reg ov[gcam::kBinRegs];
    unsigned nov = 0;
    gcam::pickMode(gTable, gcam::kExp[cell], rs, idx, &m, ov, &nov);
    const gcam::ExposureRequest q = {gcam::linesForNs(m, static_cast<uint64_t>(expUs) * 1000u), gainQ8 ? gainQ8 : 256u, 0};
    r = gSess.configure(rs, idx, q);
    IOLog("GuampCamera: %s configure mode %u%s %ux%u: init %u + mode %u + overlay %u writes, exposure %u lines (%u us) fll %u gain code 0x%x: %s rc %d\n",
          s.name, gSess.mode.index, gSess.mode.derived ? " (2x2 binned, derived)" : "", gSess.mode.width, gSess.mode.height, gTable.nInit,
          gSess.mode.nRegs, gSess.nOverlay, gSess.exp.lines, static_cast<unsigned>(gSess.exp.exposureNs / 1000u), gSess.exp.fll,
          gSess.exp.code, rcName(r), r);
    if (!r) {
      const gcam::Mipi mi = gcam::mipiOf(s, gSess.mode);
      logMipi(mi);
      if (mipi) *mipi = mi;
      logStatus("configured");
    } else {
      gSess.close();
    }
  }
  if (r && gHwUp) hwDown();
  IOLockUnlock(gSessionLock);
  return r;
}

int sensorStream(bool on) {
  if (!gSessionLock) return gcam::kErrNoMem;
  IOLockLock(gSessionLock);
  const int r = on ? gSess.streamOn() : gSess.streamOff();
  IOLog("GuampCamera: %s stream %s: rc %d\n", gSess.s ? gSess.s->name : "-", on ? "on" : "off", r);
  IOLockUnlock(gSessionLock);
  return r;
}

// For 3A: lines / Q8 gain / frame length (0 = mode default); applied receives what the sensor really got.
int sensorExposure(const gcam::ExposureRequest &q, gcam::ExposureApplied *applied) {
  if (!gSessionLock) return gcam::kErrNoMem;
  IOLockLock(gSessionLock);
  const int r = gSess.setExposure(q);
  if (applied) *applied = gSess.exp;
  IOLockUnlock(gSessionLock);
  return r;
}

int sensorStatus(gcam::Status *st) {
  if (!gSessionLock) return gcam::kErrNoMem;
  IOLockLock(gSessionLock);
  const int r = gSess.status(st);
  IOLockUnlock(gSessionLock);
  return r;
}

void sensorClose() {
  if (!gSessionLock) return;
  IOLockLock(gSessionLock);
  af::lensClose();                                                   // lens opened by the iOS service (GuampCamIosHooks.inc)
  if (gSess.state != gcam::kOff) {
    const int r = gSess.close();
    IOLog("GuampCamera: %s close (stream off + power down): rc %d\n", gSess.s ? gSess.s->name : "-", r);
  }
  if (gHwUp) hwDown();
  IOLockUnlock(gSessionLock);
}

// guamp-camera=2: one sensor brought up from the GCAM table, streamed only with guamp-camera-stream=1, then off.
void sessionTest() {
  const gcam::Sensor &s = gcam::kBoard[gSensor];
  int r = sensorOpen(gSensor, gRes, gModeIdx, gExpUs, gGainQ8, nullptr);
  if (r) { IOLog("GuampCamera: SESSION %s FAILED rc %d\n", s.name, r); sensorClose(); return; }
  const bool smia = gcam::kExp[gSensor].kind == gcam::kExpSmia;
  unsigned frames = 0, expected = 0;
  bool counted = false;
  if (gStream) {
    gcam::Status a, b;
    sensorStatus(&a);
    const uint64_t t0 = mach_absolute_time();
    r = sensorStream(true);
    if (!r) {
      IOSleep(gStreamMs);
      const int rb = sensorStatus(&b);
      uint64_t ns = 0;
      absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
      counted = !rb && smia;
      frames = static_cast<uint8_t>(b.frameCount - a.frameCount);
      expected = static_cast<unsigned>(static_cast<uint64_t>(gSess.mode.fpsMilli) * (gMclk24 ? 5u : 4u) / 5u * (ns / 1000000u) / 1000000u);
      logStatus("streaming");
      IOLog("GuampCamera: %s streamed %u ms: frame counter %u -> %u (%s; mod 256, expected about %u)\n", s.name,
            static_cast<unsigned>(ns / 1000000u), a.frameCount, b.frameCount, counted ? "SMIA 0x0005" : "not available", expected & 0xff);
      sensorStream(false);
      logStatus("stopped");
    }
  }
  IOLog("GuampCamera: SESSION %s mode %u %ux%u: configure OK, stream %s%s\n", s.name, gSess.mode.index, gSess.mode.width, gSess.mode.height,
        !gStream ? "not requested (guamp-camera-stream=0)" : r ? "FAILED" : counted && frames ? "OK, frames counted" : "on/off OK",
        gStream && !r && counted && !frames ? " but the frame counter did not move" : "");
  sensorClose();
}

void probeThread(void *, wait_result_t) {
  if (gDelay) IOSleep(gDelay * 1000u);
  IOLockLock(gSessionLock);
  unsigned ok = 0, tried = 0;
  int r = hwUp();
  if (!r) ok = probeAll(&tried);
  if (!r) af::probe(gCciDev, *gTiming);                            // guamp-camera-af (lens drivers, EEPROM AF block)
  if (gHwUp) hwDown();
  IOLog("GuampCamera: RESULT %u/%u sensors answered (step 1)%s\n", ok, tried, r ? ", hardware not up" : "");
  IOLockUnlock(gSessionLock);
  if (gMode == 2) sessionTest();
}

void start() {
  PE_parse_boot_argn("guamp-camera", &gMode, sizeof(gMode));
  if (!gMode) return;
  PE_parse_boot_argn("guamp-camera-delay", &gDelay, sizeof(gDelay));
  PE_parse_boot_argn("guamp-camera-mask", &gMask, sizeof(gMask));
  PE_parse_boot_argn("guamp-camera-mclk", &gMclkMhz, sizeof(gMclkMhz));
  PE_parse_boot_argn("guamp-camera-cci", &gCciKhz, sizeof(gCciKhz));
  PE_parse_boot_argn("guamp-camera-pmgpio", &gPmGpio, sizeof(gPmGpio));
  PE_parse_boot_argn("guamp-camera-sensor", &gSensor, sizeof(gSensor));
  PE_parse_boot_argn("guamp-camera-res", &gRes, sizeof(gRes));
  PE_parse_boot_argn("guamp-camera-modeidx", &gModeIdx, sizeof(gModeIdx));
  PE_parse_boot_argn("guamp-camera-stream", &gStream, sizeof(gStream));
  PE_parse_boot_argn("guamp-camera-stream-ms", &gStreamMs, sizeof(gStreamMs));
  PE_parse_boot_argn("guamp-camera-exp-us", &gExpUs, sizeof(gExpUs));
  PE_parse_boot_argn("guamp-camera-gain", &gGainQ8, sizeof(gGainQ8));
  af::start();
  if (gMode > 2) gMode = 1;
  if (gDelay > 600) gDelay = 600;
  if (gCciKhz != 100 && gCciKhz != 1000) gCciKhz = 400;
  gMask &= 0xf;
  if (gSensor >= gcam::kSensors) gSensor = 0;
  gRes = gRes ? 1 : 0;
  gStream = gStream == 1;
  if (gStreamMs < 100) gStreamMs = 100;
  if (gStreamMs > 10000) gStreamMs = 10000;
  if (gExpUs < 10 || gExpUs > 1000000) gExpUs = 10000;
  if (!gGainQ8 || gGainQ8 > 16 * 256) gGainQ8 = 256;
  gOwnLock = IOLockAlloc();
  gSessionLock = IOLockAlloc();
  IOLog("GuampCamera: mode %u, delay %u s, mask 0x%x, mclk %u MHz, cci %u kHz, pmgpio %u; session sensor %u res %u modeidx %u "
        "stream %u (%u ms) exp %u us gain %u/256 (%s)\n", gMode, gDelay, gMask, gMclkMhz, gCciKhz, gPmGpio, gSensor, gRes, gModeIdx,
        gStream, gStreamMs, gExpUs, gGainQ8, gMode == 2 ? "step 1 + sensor session" : "step 1: chip id only");
  if ((!gOwnLock && !audiohw::gLock) || !gSessionLock) return;
  thread_t th;
  if (kernel_thread_start(probeThread, nullptr, &th) == KERN_SUCCESS) thread_deallocate(th);
}
}  // namespace camera
