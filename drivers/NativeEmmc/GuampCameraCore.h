// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp camera, step 1 (no streaming): board facts, PM8008 LDOs over I2C, CCI (camera control interface) queue
// commands, sensor power sequences and the chip id probe. Pure code over an ops table, host-tested by
// tests/camera_host.cpp; MMIO, clocks, GPIO and the SE1 I2C bus belong to GuampCamera.inc.
// Sources (studied, no code copied):
//   DT boot_a/dtbo_a (guamp): qcom,cci0 0x5c1b000 + i2c timing nodes, cam-sensor0..3, pm8008@8/@9 on i2c@4a84000
//   Linux 4.19 guamp: drivers/regulator/qcom_pm8008-regulator.c (MISC 0x950, LDO base + 0x40/0x46/0x08/0x3b),
//   drivers/mfd/qcom-i2c-pmic.c (16-bit register address, big endian), techpack/camera-bengal cam_cci_hwreg.h +
//   cam_cci_core.c (queue commands), mainline i2c-qcom-cci.c (same queue model, polled here).
//   Sensor facts (slave address, chip id register/value, power order and delays) come from the vendor
//   sensormodule blobs, decoded by analisi/camera/tools/sensormodule.py; the register tables are NOT in this repo.
#ifndef GUAMP_CAMERA_CORE_H
#define GUAMP_CAMERA_CORE_H
#include <stdint.h>
#include <stddef.h>

namespace gcam {

// ---- errors (negative errno style, like the other cores) ------------------------------------------------------------
enum : int { kOk = 0, kErrIo = -5, kErrNack = -6, kErrNoMem = -12, kErrInval = -22, kErrRange = -34,
             kErrTimeout = -110, kErrMismatch = -1000 };

// ---- PM8008 (camera PMIC, QUP0 SE1 I2C) -----------------------------------------------------------------------------
constexpr uint8_t kPmChip = 0x08;           // qcom,pm8008@8: MISC, revid, GPIOs
constexpr uint8_t kPmLdo = 0x09;            // qcom,pm8008@9: LDO1..7 (L1P..L7P)
constexpr uint16_t kPmRevidType = 0x0104;   // qcom,revid@100: PERPH_TYPE / SUBTYPE
constexpr uint16_t kPmRevidSubtype = 0x0105;
constexpr uint16_t kPmChipEnable = 0x0950;  // MISC_CHIP_ENABLE_REG, bit 0
constexpr uint16_t kPmGpio1 = 0xc000;       // pinctrl@c000 GPIO1 (pm8008_gpio1_active, pinctrl-0 of pm8008@9)
constexpr uint16_t ldoBase(unsigned n) { return static_cast<uint16_t>(0x4000 + (n - 1) * 0x100); }   // n = 1..7
constexpr uint16_t kLdoStatus1 = 0x08;      // bit 7 VREG_READY, bit 5 OCP
constexpr uint16_t kLdoStepper = 0x3b;      // bits 1:0: step rate = 38400 >> v uV/us
constexpr uint16_t kLdoVsetLb = 0x40;       // mV, 16 bit little endian (LB, UB)
constexpr uint16_t kLdoEnable = 0x46;       // bit 7
constexpr uint8_t kLdoReady = 0x80, kLdoOn = 0x80;
constexpr unsigned kLdos = 7;
// DT voltage ranges (min..max uV) of L1P..L7P; the driver keeps a request inside them.
constexpr uint32_t kLdoMinUv[kLdos] = {528000, 528000, 2800000, 2800000, 2800000, 2800000, 1800000};
constexpr uint32_t kLdoMaxUv[kLdos] = {1200000, 1056000, 2800000, 2800000, 2800000, 2800000, 1800000};

// Linux pm8008_write_voltage: mV rounded up to the 8 mV step.
constexpr uint32_t ldoMv(uint32_t uv) { return ((uv + 999) / 1000 + 7) / 8 * 8; }

// ---- CCI (cam_cci_hwreg.h / i2c-qcom-cci.c) -------------------------------------------------------------------------
constexpr uint32_t kCciHwVersion = 0x000;
constexpr uint32_t kCciReset = 0x004;
constexpr uint32_t kCciResetAll = 0x0f73f3f7, kCciResetM0 = 0x3f1, kCciResetM1 = 0x3f001;
constexpr uint32_t kCciQueueStart = 0x008;
constexpr uint32_t kCciHaltReq = 0x034;
constexpr uint32_t mScl(unsigned m) { return 0x100 + 0x100 * m; }        // thigh << 16 | tlow
constexpr uint32_t mSda0(unsigned m) { return 0x104 + 0x100 * m; }       // tsu_sto << 16 | tsu_sta
constexpr uint32_t mSda1(unsigned m) { return 0x108 + 0x100 * m; }       // thd_dat << 16 | thd_sta
constexpr uint32_t mSda2(unsigned m) { return 0x10c + 0x100 * m; }       // tbuf
constexpr uint32_t mMisc(unsigned m) { return 0x110 + 0x100 * m; }       // stretch << 8 | trdhld << 4 | tsp
constexpr uint32_t mReadData(unsigned m) { return 0x118 + 0x100 * m; }
constexpr uint32_t mReadLevel(unsigned m) { return 0x11c + 0x100 * m; }
constexpr uint32_t qExec(unsigned m, unsigned q) { return 0x300 + 0x200 * m + 0x100 * q; }
constexpr uint32_t qCur(unsigned m, unsigned q) { return 0x304 + 0x200 * m + 0x100 * q; }
constexpr uint32_t qLoad(unsigned m, unsigned q) { return 0x310 + 0x200 * m + 0x100 * q; }
constexpr uint32_t kCciIrqGlobalClear = 0xc00, kCciIrqMask0 = 0xc04, kCciIrqClear0 = 0xc08, kCciIrqStatus0 = 0xc0c;
constexpr uint32_t kCciIrqMask0All = 0x7fff7ff7;
constexpr uint32_t kIrqRstDone = 1u << 24;
constexpr uint32_t irqHaltAck(unsigned m) { return 1u << (25 + m); }
constexpr uint32_t irqRdDone(unsigned m) { return m ? 1u << 12 : 1u << 0; }
constexpr uint32_t irqReport(unsigned m, unsigned q) { return 1u << (4 + 12 * m + 4 * q); }   // M0Q0 4, M0Q1 8, M1Q0 16, M1Q1 20
constexpr uint32_t irqNack(unsigned m) { return m ? 0x60000000u : 0x18000000u; }
constexpr uint32_t irqError(unsigned m) { return m ? 0x60EE6000u : 0x18000EE6u; }            // includes NACK and RD errors
// queue commands (cam_cci_dev.h enum cam_cci_cmd_type)
enum : uint32_t { kCmdSetParam = 1, kCmdReport = 8, kCmdWrite = 9, kCmdRead = 10, kCmdWriteNoStop = 11 };
constexpr uint32_t kReportIrq = 1u << 8;
constexpr unsigned kQ0Words = 64, kQ1Words = 16;   // CCI v1.2 queue sizes
constexpr unsigned kReadMax = 14;                  // CCI_READ_MAX_V_1_2
constexpr unsigned kRetries = 3;                   // SET_PARAM retries field (bits 17:16)

struct CciTiming { uint16_t thigh, tlow, tsuSto, tsuSta, thdDat, thdSta, tbuf; uint8_t stretch, trdhld, tsp; };
// qcom,cci0 timing nodes (cci-clk-src 37.5 MHz); with the 19.2 MHz TCXO source the bus runs at about half speed.
constexpr CciTiming kCciStandard = {0xc9, 0xae, 0xcc, 0xe7, 0x16, 0xa2, 0xe3, 0, 6, 3};
constexpr CciTiming kCciFast = {0x26, 0x38, 0x28, 0x28, 0x16, 0x23, 0x3e, 0, 6, 3};
constexpr CciTiming kCciFastPlus = {0x10, 0x16, 0x11, 0x12, 0x10, 0x0f, 0x18, 0, 3, 3};

// ---- sensors (DT cell + sensormodule facts) -------------------------------------------------------------------------
enum PwrType : uint8_t { kMclk = 0, kVana = 1, kVdig = 2, kVio = 3, kVaf = 4, kReset = 8, kStandby = 9 };
struct PwrStep { uint8_t type, on; uint16_t delayMs; };
struct Rail { uint8_t ldo; uint32_t uv; };   // ldo 0 = none

struct Sensor {
  const char *name;
  uint8_t cell, cci, slave7, addrBytes, idBytes;
  uint16_t idReg, id;
  uint8_t mclk, mclkGpio, resetGpio, csiphy;
  Rail vio, vana, vdig;
  const PwrStep *up; uint8_t nUp;
  const PwrStep *down; uint8_t nDown;
};

// power-up/down orders and delays from the sensormodule blobs (sensormodule.py "power up/down")
constexpr PwrStep kGm1Up[] = {{kMclk, 1, 1}, {kReset, 0, 1}, {kVio, 1, 0}, {kVana, 1, 0}, {kVdig, 1, 1}, {kReset, 1, 4}};
constexpr PwrStep kGm1Down[] = {{kMclk, 0, 3}, {kReset, 0, 1}, {kVdig, 0, 0}, {kVana, 0, 0}, {kVio, 0, 1}};
constexpr PwrStep k4h7Up[] = {{kMclk, 1, 0}, {kReset, 0, 1}, {kVana, 1, 0}, {kVdig, 1, 0}, {kVio, 1, 0}, {kReset, 1, 4}};
constexpr PwrStep k4h7Down[] = {{kMclk, 0, 0}, {kReset, 0, 0}, {kVio, 0, 0}, {kVdig, 0, 0}, {kVana, 0, 0}};
constexpr PwrStep kOvUp[] = {{kReset, 0, 0}, {kMclk, 1, 0}, {kVio, 1, 0}, {kVana, 1, 5}, {kReset, 1, 10}};
constexpr PwrStep kOvDown[] = {{kReset, 0, 0}, {kVana, 0, 0}, {kVio, 0, 0}, {kMclk, 0, 0}};

constexpr unsigned kSensors = 4;
// cell = DT cell-index = blob cameraId. MCLK index = GCC_CAMSS_MCLKn; the DT label "CAMIF_MCLK3" of cell 3 is wrong
// (gpio21 = MCLK1). OV02B10 (colour, actuator gt9772) is cell 3, OV02B1B (mono) is cell 2.
constexpr Sensor kBoard[kSensors] = {
  {"s5kgm1st", 0, 0, 0x2d, 2, 2, 0x0000, 0xf8d1, 0, 20, 18, 0, {7, 1800000}, {4, 2800000}, {2, 1056000},
   kGm1Up, 6, kGm1Down, 5},
  {"s5k4h7", 1, 1, 0x10, 2, 2, 0x0000, 0x487b, 2, 27, 24, 2, {7, 1800000}, {6, 2800000}, {1, 1200000},
   k4h7Up, 6, k4h7Down, 5},
  {"ov02b1b", 2, 1, 0x3d, 1, 1, 0x0003, 0x002b, 3, 28, 68, 1, {7, 1800000}, {3, 2800000}, {0, 0},
   kOvUp, 5, kOvDown, 4},
  {"ov02b10", 3, 0, 0x3c, 1, 1, 0x0003, 0x002b, 1, 21, 19, 1, {7, 1800000}, {3, 2800000}, {0, 0},
   kOvUp, 5, kOvDown, 4},
};

// ---- ops ------------------------------------------------------------------------------------------------------------
struct Ops {
  void *ctx;
  int (*pmRead)(void *, uint8_t dev7, uint16_t reg, uint8_t *v);
  int (*pmWrite)(void *, uint8_t dev7, uint16_t reg, uint8_t v);
  uint32_t (*cciRd)(void *, uint32_t off);
  void (*cciWr)(void *, uint32_t off, uint32_t v);
  void (*gpio)(void *, unsigned gpio, int high);
  int (*mclk)(void *, unsigned index, unsigned gpio, int on);   // clock branch + pin function, 0 or error
  void (*delayUs)(void *, unsigned us);
  void (*sleepMs)(void *, unsigned ms);
  void (*note)(void *, const char *step, int rc, uint32_t a, uint32_t b);
};

inline void note(const Ops &o, const char *s, int rc, uint32_t a, uint32_t b) { if (o.note) o.note(o.ctx, s, rc, a, b); }
inline void sleepMs(const Ops &o, unsigned ms) { if (ms) o.sleepMs(o.ctx, ms); }

// ---- PM8008 -----------------------------------------------------------------------------------------------------------
inline int pmUpdate(const Ops &o, uint8_t dev, uint16_t reg, uint8_t mask, uint8_t val) {
  uint8_t v = 0;
  int r = o.pmRead(o.ctx, dev, reg, &v);
  if (r) return r;
  const uint8_t n = static_cast<uint8_t>((v & ~mask) | (val & mask));
  return n == v ? 0 : o.pmWrite(o.ctx, dev, reg, n);
}

inline int pmChip(const Ops &o, bool on) { return pmUpdate(o, kPmChip, kPmChipEnable, 1, on ? 1 : 0); }

struct LdoState { uint8_t users[kLdos]; uint16_t mv[kLdos]; };

inline int ldoSet(const Ops &o, unsigned n, uint32_t uv) {
  if (n < 1 || n > kLdos || uv < kLdoMinUv[n - 1] || uv > kLdoMaxUv[n - 1]) return kErrRange;
  const uint32_t mv = ldoMv(uv);
  const uint16_t b = ldoBase(n);
  int r = o.pmWrite(o.ctx, kPmLdo, b + kLdoVsetLb, static_cast<uint8_t>(mv));
  if (!r) r = o.pmWrite(o.ctx, kPmLdo, b + kLdoVsetLb + 1, static_cast<uint8_t>(mv >> 8));
  return r;
}

// Enable: write the setpoint, set bit 7, then wait VREG_READY like Linux (20 us + mV / step rate, 10 tries).
inline int ldoOn(const Ops &o, LdoState &s, unsigned n, uint32_t uv) {
  if (n < 1 || n > kLdos) return kErrInval;
  if (s.users[n - 1]) {
    if (ldoMv(uv) != s.mv[n - 1]) return kErrInval;      // a shared rail at another voltage
    ++s.users[n - 1];
    return kOk;
  }
  const uint16_t b = ldoBase(n);
  int r = ldoSet(o, n, uv);
  uint8_t step = 0;
  if (!r) r = o.pmRead(o.ctx, kPmLdo, b + kLdoStepper, &step);
  if (!r) r = pmUpdate(o, kPmLdo, b + kLdoEnable, kLdoOn, kLdoOn);
  if (r) { note(o, "ldo enable write", r, n, uv); return r; }
  const uint32_t rate = 38400u >> (step & 3);              // uV per us
  const unsigned wait = 20 + (ldoMv(uv) * 1000 + rate - 1) / rate;
  uint8_t st = 0;
  for (unsigned t = 0; t < 10; ++t) {
    if (wait > 20000) o.sleepMs(o.ctx, (wait + 999) / 1000); else o.delayUs(o.ctx, wait);
    r = o.pmRead(o.ctx, kPmLdo, b + kLdoStatus1, &st);
    if (r) break;
    if (st & kLdoReady) {
      s.users[n - 1] = 1;
      s.mv[n - 1] = static_cast<uint16_t>(ldoMv(uv));
      note(o, "ldo on (L<n>P, mV)", 0, n, ldoMv(uv));
      return kOk;
    }
  }
  pmUpdate(o, kPmLdo, b + kLdoEnable, kLdoOn, 0);
  note(o, "ldo VREG_READY missing (L<n>P, status1)", r ? r : kErrTimeout, n, st);
  return r ? r : kErrTimeout;
}

inline int ldoOff(const Ops &o, LdoState &s, unsigned n) {
  if (n < 1 || n > kLdos || !s.users[n - 1]) return kOk;
  if (--s.users[n - 1]) return kOk;
  int r = pmUpdate(o, kPmLdo, ldoBase(n) + kLdoEnable, kLdoOn, 0);
  note(o, "ldo off (L<n>P)", r, n, 0);
  return r;
}

inline unsigned ldoUsers(const LdoState &s) { unsigned t = 0; for (unsigned i = 0; i < kLdos; ++i) t += s.users[i]; return t; }

// ---- CCI --------------------------------------------------------------------------------------------------------------
struct Cci {
  const Ops *o;
  unsigned pollUs = 50, timeoutUs = 100000;
  uint32_t lastStatus = 0;
  uint32_t rd(uint32_t off) const { return o->cciRd(o->ctx, off); }
  void wr(uint32_t off, uint32_t v) const { o->cciWr(o->ctx, off, v); }
  void clear(uint32_t bits) const { wr(kCciIrqClear0, bits); wr(kCciIrqGlobalClear, 1); }

  // Wait for any of want (or an error of master m), then clear what was seen.
  int wait(uint32_t want, uint32_t errors) {
    for (unsigned t = 0; t <= timeoutUs; t += pollUs) {
      const uint32_t s = rd(kCciIrqStatus0);
      if (s & (want | errors)) {
        lastStatus = s;
        clear(s);
        if (s & errors) return (s & errors & (irqNack(0) | irqNack(1))) ? kErrNack : kErrIo;
        return kOk;
      }
      o->delayUs(o->ctx, pollUs);
    }
    lastStatus = rd(kCciIrqStatus0);
    return kErrTimeout;
  }

  int reset(uint32_t mask) {
    clear(0xffffffffu);
    wr(kCciReset, mask);
    return wait(kIrqRstDone, 0);
  }

  void timing(unsigned m, const CciTiming &t) const {
    wr(mScl(m), static_cast<uint32_t>(t.thigh) << 16 | t.tlow);
    wr(mSda0(m), static_cast<uint32_t>(t.tsuSto) << 16 | t.tsuSta);
    wr(mSda1(m), static_cast<uint32_t>(t.thdDat) << 16 | t.thdSta);
    wr(mSda2(m), t.tbuf);
    wr(mMisc(m), static_cast<uint32_t>(t.stretch) << 8 | static_cast<uint32_t>(t.trdhld) << 4 | t.tsp);
  }

  // Whole block reset, interrupt status enabled (the SPI stays unclaimed: everything is polled), both masters timed.
  int init(const CciTiming &t) {
    int r = reset(kCciResetAll);
    if (r) { note(*o, "cci reset (status)", r, lastStatus, 0); return r; }
    wr(kCciIrqMask0, kCciIrqMask0All);
    clear(0xffffffffu);
    timing(0, t);
    timing(1, t);
    return kOk;
  }

  // Halt + reset of one master after an error (cam_cci_core.c recovery), timing restored.
  void recover(unsigned m, const CciTiming &t) {
    wr(kCciHaltReq, 1u << m);
    wait(irqHaltAck(m), 0);
    reset(m ? kCciResetM1 : kCciResetM0);
    timing(m, t);
  }

  void load(unsigned m, unsigned q, uint32_t w) const { wr(qLoad(m, q), w); }

  int run(unsigned m, unsigned q, uint32_t want) {
    wr(qExec(m, q), rd(qCur(m, q)));
    wr(kCciQueueStart, 1u << (m * 2 + q));
    return wait(want, irqError(m));
  }

  static uint32_t setParam(uint8_t slave7) { return kCmdSetParam | static_cast<uint32_t>(slave7 & 0x7f) << 4 | kRetries << 16; }
  // WRITE_DISABLE_P with the register address, MSB first in bytes 1..n (cam_cci_read)
  static uint32_t addrNoStop(uint32_t reg, unsigned addrBytes) {
    uint32_t v = kCmdWriteNoStop | static_cast<uint32_t>(addrBytes) << 4;
    for (unsigned i = 0; i < addrBytes; ++i) v |= ((reg >> (8 * i)) & 0xffu) << (8 * (addrBytes - i));
    return v;
  }

  // Register read on queue 1 (as Linux): SET_PARAM, address without STOP, READ n; data after the echoed slave id byte.
  int read(unsigned m, uint8_t slave7, uint32_t reg, unsigned addrBytes, uint8_t *out, unsigned n) {
    if (m > 1 || !addrBytes || addrBytes > 3 || !n || n > kReadMax) return kErrInval;
    if (rd(qCur(m, 1))) return kErrIo;                    // queue not empty: caller recovers
    load(m, 1, setParam(slave7));
    load(m, 1, addrNoStop(reg, addrBytes));
    load(m, 1, kCmdRead | n << 4);
    int r = run(m, 1, irqRdDone(m));
    if (r) return r;
    const uint32_t words = rd(mReadLevel(m));
    unsigned got = 0;
    bool first = true;
    for (uint32_t w = 0; w < words; ++w) {
      const uint32_t d = rd(mReadData(m));
      for (unsigned b = 0; b < 4 && got < n; ++b) {
        if (first) { first = false; continue; }
        out[got++] = static_cast<uint8_t>(d >> (8 * b));
      }
    }
    return got == n ? kOk : kErrIo;
  }

  // Register write on queue 0: SET_PARAM, WRITE (address + data, <= 11 bytes), REPORT with interrupt.
  int write(unsigned m, uint8_t slave7, uint32_t reg, unsigned addrBytes, uint32_t val, unsigned dataBytes) {
    if (m > 1 || !addrBytes || addrBytes > 3 || !dataBytes || dataBytes > 4) return kErrInval;
    if (rd(qCur(m, 0))) return kErrIo;
    uint8_t b[12] = {};
    unsigned n = 0;
    const unsigned len = addrBytes + dataBytes;
    b[n++] = static_cast<uint8_t>(kCmdWrite | len << 4);
    for (unsigned i = addrBytes; i-- > 0;) b[n++] = static_cast<uint8_t>(reg >> (8 * i));
    for (unsigned i = dataBytes; i-- > 0;) b[n++] = static_cast<uint8_t>(val >> (8 * i));
    load(m, 0, setParam(slave7));
    for (unsigned i = 0; i < n; i += 4)
      load(m, 0, b[i] | static_cast<uint32_t>(b[i + 1]) << 8 | static_cast<uint32_t>(b[i + 2]) << 16 | static_cast<uint32_t>(b[i + 3]) << 24);
    load(m, 0, kCmdReport | kReportIrq);
    return run(m, 0, irqReport(m, 0));
  }
};

// ---- sensor power + chip id -------------------------------------------------------------------------------------------
struct Power {
  const Ops *o;
  LdoState *ldo;
  const Sensor *s;
  bool mclkOn = false, resetHigh = false;
  uint8_t railsOn = 0;   // bit per PwrType (kVana, kVdig, kVio, kVaf)

  const Rail *rail(uint8_t t) const { return t == kVio ? &s->vio : t == kVana ? &s->vana : t == kVdig ? &s->vdig : nullptr; }

  int step(const PwrStep &p) {
    int r = kOk;
    switch (p.type) {
    case kMclk:
      r = o->mclk(o->ctx, s->mclk, s->mclkGpio, p.on);
      if (!r) mclkOn = p.on;
      break;
    case kReset: case kStandby:
      o->gpio(o->ctx, s->resetGpio, p.on);
      resetHigh = p.on;
      break;
    default: {
      const Rail *ra = rail(p.type);
      if (!ra || !ra->ldo) break;                          // vaf (actuator) and absent rails: nothing for an id read
      const uint8_t bit = static_cast<uint8_t>(1u << p.type);
      if (p.on && !(railsOn & bit)) { r = ldoOn(*o, *ldo, ra->ldo, ra->uv); if (!r) railsOn |= bit; }
      else if (!p.on && (railsOn & bit)) { r = ldoOff(*o, *ldo, ra->ldo); railsOn &= static_cast<uint8_t>(~bit); }
    }
    }
    if (!r) sleepMs(*o, p.delayMs);
    return r;
  }

  int up() {
    for (unsigned i = 0; i < s->nUp; ++i) {
      int r = step(s->up[i]);
      if (r) { note(*o, "power up step failed (index, type)", r, i, s->up[i].type); return r; }
    }
    return kOk;
  }

  // The blob's power-down order, then anything a failed power-up left on (reset low first, rails, MCLK last).
  int down() {
    int first = kOk;
    for (unsigned i = 0; i < s->nDown; ++i) {
      const PwrStep &p = s->down[i];
      if (p.type == kMclk && !mclkOn) continue;
      int r = step(p);
      if (r && !first) first = r;
    }
    if (resetHigh) step({kReset, 0, 0});
    const uint8_t order[3] = {kVdig, kVana, kVio};
    for (unsigned i = 0; i < 3; ++i)
      if (railsOn & (1u << order[i])) { int r = step({order[i], 0, 0}); if (r && !first) first = r; }
    if (mclkOn) { int r = step({kMclk, 0, 0}); if (r && !first) first = r; }
    return first;
  }
};

struct ProbeResult { int rc; uint16_t id; uint32_t cciStatus; };

inline ProbeResult probe(const Ops &o, LdoState &ldo, Cci &cci, const Sensor &s, const CciTiming &t) {
  ProbeResult res = {kOk, 0, 0};
  Power p = {&o, &ldo, &s};
  res.rc = p.up();
  if (!res.rc) {
    uint8_t b[2] = {};
    for (unsigned tr = 0; tr < 3; ++tr) {
      res.rc = cci.read(s.cci, s.slave7, s.idReg, s.addrBytes, b, s.idBytes);
      res.cciStatus = cci.lastStatus;
      if (!res.rc) break;
      cci.recover(s.cci, t);
    }
    if (!res.rc) {
      res.id = s.idBytes == 2 ? static_cast<uint16_t>(b[0] << 8 | b[1]) : b[0];
      if (res.id != s.id) res.rc = kErrMismatch;
    }
  }
  int d = p.down();
  if (!res.rc && d) res.rc = d;
  return res;
}

// ---- GCAM v1 tables (sensormodule.py --gcam), loaded at run time, never committed --------------------------------
struct GcamReg { uint16_t addr, data; uint8_t addrBytes, dataBytes; uint16_t delayUs; };
struct GcamSensor {
  char name[16];
  uint8_t slave7, addrBytes, dataBytes, mode;
  uint16_t idReg, id, width, height, lanes, dt;
  uint32_t laneMbps;
  uint16_t nPowerUp, nInit, nMode, nStreamOn, nStreamOff;
  const uint8_t *powerUp;      // nPowerUp x 8 bytes {u8 type, u8 pad, u16 delayMs, u32 value}
  const uint8_t *regs[4];      // init, mode, stream on, stream off: n x 8 bytes {u16 addr, u16 data, u8, u8, u16 delayUs}
};

inline uint16_t le16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | p[1] << 8); }
inline uint32_t le32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | static_cast<uint32_t>(p[3]) << 24; }
inline GcamReg gcamReg(const uint8_t *p) { return {le16(p), le16(p + 2), p[4], p[5], le16(p + 6)}; }

// Finds a sensor by name ("mot_s5kgm1st" or "s5kgm1st"); every length is checked against len.
inline int gcamFind(const uint8_t *blob, size_t len, const char *name, GcamSensor *out) {
  if (!blob || len < 12 || blob[0] != 'G' || blob[1] != 'C' || blob[2] != 'A' || blob[3] != 'M') return kErrInval;
  if (le32(blob + 4) != 1) return kErrInval;
  const uint32_t count = le32(blob + 8);
  size_t at = 12;
  for (uint32_t i = 0; i < count; ++i) {
    if (len - at < 48) return kErrRange;
    const uint8_t *h = blob + at;
    GcamSensor s = {};
    for (unsigned k = 0; k < 15; ++k) s.name[k] = static_cast<char>(h[k]);
    s.slave7 = h[16]; s.addrBytes = h[17]; s.dataBytes = h[18]; s.mode = h[19];
    s.idReg = le16(h + 20); s.id = le16(h + 22); s.width = le16(h + 24); s.height = le16(h + 26);
    s.lanes = le16(h + 28); s.dt = le16(h + 30); s.laneMbps = le32(h + 32);
    s.nPowerUp = le16(h + 36); s.nInit = le16(h + 38); s.nMode = le16(h + 40);
    s.nStreamOn = le16(h + 42); s.nStreamOff = le16(h + 44);
    at += 48;
    const size_t need = 8ull * (s.nPowerUp + s.nInit + s.nMode + s.nStreamOn + s.nStreamOff);
    if (len - at < need) return kErrRange;
    s.powerUp = blob + at;
    at += 8ull * s.nPowerUp;
    const uint16_t n[4] = {s.nInit, s.nMode, s.nStreamOn, s.nStreamOff};
    for (unsigned k = 0; k < 4; ++k) { s.regs[k] = blob + at; at += 8ull * n[k]; }
    const char *a = s.name, *b = name;
    if (a[0] == 'm' && a[1] == 'o' && a[2] == 't' && a[3] == '_' && !(b[0] == 'm' && b[1] == 'o' && b[2] == 't' && b[3] == '_')) a += 4;
    while (*a && *a == *b) { ++a; ++b; }
    if (!*a && !*b) { *out = s; return kOk; }
  }
  return kErrRange;
}

}  // namespace gcam
#endif
