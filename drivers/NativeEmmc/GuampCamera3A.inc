// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp camera 3A, kernel side: lens drivers (voice coil DW9767 of the main camera, GT9772 of the macro camera), the AF
// calibration of the main module EEPROM and the sensor exposure write lists, all over the CCI of GuampCamera.inc.
// Included by NativeEmmc.cpp right after GuampCamera.inc (uses camera::gOps, gLdo, pinOut and the CCI mapping).
// The algorithms (auto exposure, white balance, contrast AF, flash metering) run in user space (package/GuampCamera3A);
// the kernel only applies their results: exposure/gain (exposure()), lens DAC (lens*()), flash (GuampTorch.inc).
// Pure logic: GuampCamera3ACore.h (host test tests/camera3a_host.cpp).
//
// Boot argument (only with guamp-camera=1, inside the step 1 probe thread, CCI up, sensors already off again):
//   guamp-camera-af=0    default: no lens/EEPROM access
//   guamp-camera-af=1    main lens: VIO + VAF on, EEPROM AF block read, DW9767 init, move infinity -> middle -> macro,
//                        park, power-down bit, rails off. One "GuampCamera3A:" line per step, RESULT line at the end.
//   guamp-camera-af=3    the same, then the macro lens (GT9772, VAF = gpio103 regulator)
// Never on by default: the lens moves (audible click if the init values are wrong). No sensor is streamed here.
#include "GuampCamera3ACore.h"

namespace camera {
namespace af {
unsigned gMode;
gcam::Cci *gCciNow;                    // valid while a caller holds the camera lock and the CCI is initialised
const gcam::CciTiming *gTiming;
gcam3a::LensCtl gLens;                 // one lens at a time (both drivers answer at slave 0x0c of CCI master 0)
unsigned gLensRails;                   // bit 0 VIO, bit 1 VAF (PM8008), bit 2 VAF (gpio103)

int cciWrite(void *, uint8_t m, uint8_t slave7, const gcam3a::Write &w) {
  if (!gCciNow) return gcam::kErrInval;
  int r = gCciNow->write(m, slave7, w.addr, w.addrBytes, w.data, w.dataBytes);
  if (r && gTiming) {                  // one retry after a master recovery (NACK on a lens that was still powering up)
    gCciNow->recover(m, *gTiming);
    r = gCciNow->write(m, slave7, w.addr, w.addrBytes, w.data, w.dataBytes);
  }
  return r;
}
void lensDelay(void *, unsigned us) { if (us >= 1000) IOSleep((us + 999) / 1000); else IODelay(us); }
const gcam3a::LensIo gLensIo = {nullptr, cciWrite, lensDelay};

// VIO 1.8 V (L7P: I2C pull-ups of the module, EEPROM supply) and the lens VAF rail.
int railsOn(const gcam3a::Lens &l) {
  int r = gcam::ldoOn(gOps, gLdo, 7, 1800000);
  if (r) return r;
  gLensRails |= 1;
  if (l.vaf == gcam3a::kVafPm8008) {
    r = gcam::ldoOn(gOps, gLdo, l.vafLdo, l.vafUv);
    if (r) return r;
    gLensRails |= 2;
  } else {
    pinOut(l.vafGpio, 1);
    IODelay(l.vafRampUs);
    gLensRails |= 4;
  }
  IOSleep(2);                          // driver POR before the first command
  return gcam::kOk;
}
void railsOff(const gcam3a::Lens &l) {
  if (gLensRails & 2) gcam::ldoOff(gOps, gLdo, l.vafLdo);
  if (gLensRails & 4) pinOut(l.vafGpio, 0);
  if (gLensRails & 1) gcam::ldoOff(gOps, gLdo, 7);
  gLensRails = 0;
}

// EEPROM AF block (2-byte addresses, CCI reads of at most kReadMax bytes).
gcam3a::AfCal readCal(gcam::Cci &cci, const gcam3a::Lens &l, int *rcOut) {
  uint8_t buf[gcam3a::kAfLen + 2] = {};
  int r = gcam::kOk;
  for (unsigned off = 0; off < sizeof(buf) && !r;) {
    const unsigned n = sizeof(buf) - off < gcam::kReadMax ? static_cast<unsigned>(sizeof(buf) - off) : gcam::kReadMax;
    r = cci.read(l.cci, gcam3a::kEepromSlave7, gcam3a::kAfBlock + off, 2, buf + off, n);
    off += n;
  }
  if (rcOut) *rcOut = r;
  const gcam3a::AfCal c = r ? gcam3a::AfCal{l.defInf, l.defMacro, false} : gcam3a::parseAfCal(buf, sizeof(buf), l);
  IOLog("GuampCamera3A: eeprom 0x%02x af block rc %d: %02x%02x %02x%02x .. crc %02x%02x -> inf %u macro %u (%s)\n",
        gcam3a::kEepromSlave7, r, buf[gcam3a::kAfInf - gcam3a::kAfBlock], buf[gcam3a::kAfInf - gcam3a::kAfBlock + 1],
        buf[gcam3a::kAfMacro - gcam3a::kAfBlock], buf[gcam3a::kAfMacro - gcam3a::kAfBlock + 1], buf[gcam3a::kAfLen],
        buf[gcam3a::kAfLen + 1], c.inf, c.macro, c.valid ? "calibrated" : "defaults");
  return c;
}

// ---- API for the streaming code (camera lock held, CCI initialised, sensor of `cell` powered) -------------------------
int lensOpen(gcam::Cci &cci, const gcam::CciTiming &t, unsigned cell) {
  const gcam3a::Lens *l = gcam3a::lensForCell(cell);
  if (!l) return gcam::kErrInval;
  if (gLens.on) return gLens.l == l ? gcam::kOk : gcam::kErrInval;   // the other lens still holds slave 0x0c
  gCciNow = &cci;
  gTiming = &t;
  int r = railsOn(*l);
  gcam3a::AfCal cal = {l->defInf, l->defMacro, false};
  if (!r && cell == 0) cal = readCal(cci, *l, nullptr);
  if (!r) r = gLens.powerUp(&gLensIo, l, cal);
  IOLog("GuampCamera3A: lens %s open: rc %d, range %u..%u, writes %u\n", l->name, r, gLens.lo, gLens.hi, gLens.writes);
  if (r) { railsOff(*l); gCciNow = nullptr; }
  return r;
}
int lensSet(uint16_t dac) { return gLens.on ? gLens.move(dac) : gcam::kErrInval; }
uint16_t lensPos() { return gLens.dac; }
void lensClose() {
  if (!gLens.l) return;
  const int r = gLens.park();
  railsOff(*gLens.l);
  IOLog("GuampCamera3A: lens %s parked: rc %d, writes %u\n", gLens.l->name, r, gLens.writes);
  gCciNow = nullptr;
}

// Exposure from user space (AE): time in ns + analog gain Q8 -> lines/FLL/gain code, written inside the sensor hold.
int exposure(gcam::Cci &cci, const gcam::Sensor &s, unsigned mode, uint64_t ns, uint32_t gainQ8, gcam3a::Exposure *out) {
  const gcam3a::ExpoSensor *e = gcam3a::expoSensor(s.cell);
  if (!e || !out) return gcam::kErrInval;
  int r = gcam3a::plan(*e, mode, gcam3a::linesForNs(*e, mode, ns), 0, gainQ8, out);
  if (r) return r;
  gcam3a::Write w[gcam3a::kMaxWrites];
  const unsigned n = gcam3a::writes(*e, *out, w);
  for (unsigned k = 0; k < n && !r; ++k) r = cci.write(s.cci, s.slave7, w[k].addr, w[k].addrBytes, w[k].data, w[k].dataBytes);
  return r;
}

// ---- probe (guamp-camera-af) --------------------------------------------------------------------------------------------
void probe(gcam::Cci &cci, const gcam::CciTiming &t) {
  if (!gMode) return;
  unsigned ok = 0, tried = 0;
  for (unsigned cell = 0; cell < 4; ++cell) {
    const gcam3a::Lens *l = gcam3a::lensForCell(cell);
    if (!l || !(gMode & (cell == 0 ? 1u : 2u))) continue;
    ++tried;
    int r = lensOpen(cci, t, cell);
    const uint16_t stops[3] = {gLens.lo, static_cast<uint16_t>((gLens.lo + gLens.hi) / 2), gLens.hi};
    for (unsigned k = 0; !r && k < 3; ++k) {
      r = lensSet(stops[k]);
      IOLog("GuampCamera3A: lens %s dac %u: rc %d\n", l->name, gLens.dac, r);
      IOSleep(30);
    }
    lensClose();
    if (!r && !gLens.lastError) ++ok;
    IOLog("GuampCamera3A: lens %s (cci%u slave 0x%02x): %s rc %d last error %d\n", l->name, l->cci, l->slave7,
          !r && !gLens.lastError ? "OK" : r == gcam::kErrNack ? "NACK" : "FAILED", r, gLens.lastError);
  }
  IOLog("GuampCamera3A: RESULT %u/%u lens drivers answered; rails %u\n", ok, tried, gLensRails);
}

void start() {
  PE_parse_boot_argn("guamp-camera-af", &gMode, sizeof(gMode));
  gMode &= 3;
}
}  // namespace af
}  // namespace camera
