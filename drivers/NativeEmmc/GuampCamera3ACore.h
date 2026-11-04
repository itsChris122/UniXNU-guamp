// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp camera 3A, kernel side without hardware access (host-tested by tests/camera3a_host.cpp):
//   - exposure / frame length / analog gain of the four sensors as CCI write lists inside the sensor's grouped hold;
//   - the two voice-coil lens drivers (DW9767 of the main camera, GT9772 of the macro camera): init, DAC moves, park;
//   - the AF calibration block of the main module EEPROM (infinity / macro DAC) with its CRC.
// Used by GuampCamera3A.inc (CCI through gcam::Cci of GuampCameraCore.h).
// Sources (studied, no code copied, no vendor tables):
//   sensor register addresses, gain/line limits, vertical offset, mode timing: the sensormodule blobs as decoded by
//   analisi/camera/tools/sensormodule.py (regAddr, exposure, groupHoldOn/Off, resolutions);
//   gain formulas and the OV02B write sequence: disassembly of com.qti.sensor.mot_{s5kgm1st,s5k4h7,ov02b10,ov02b1b}.so
//   (RealToRegisterGain / FillExposureSettings): Samsung code = gain x 32 (1x..16x), OV02B code = gain x 16
//   (1x..15.5x), digital gain fixed at 1x (never written); OV02B: page 1 (0xfd), vblank = FLL - 1220 in 0x14/0x15,
//   exposure lines in 0x0e/0x0f, gain 0x22, then 0xfe = 0x02 (frame-synchronous update, the OV02B "group hold");
//   lens drivers: techpack camera-bengal cam_actuator/mot_actuator.c (GT9767/GT9772 init lists, DAC word at 0x03,
//   slave 0x0c on CCI master 0, park/launch in steps) and the guamp DT (actuator0 cam_vaf = PM8008 L5P 2.8 V,
//   actuator1 cam_vaf = fixed regulator on TLMM gpio103, 135 us ramp);
//   EEPROM AF block: disassembly of com.mot.eeprom.mot_dw9767_s5kgm1st_eeprom.so (format_afc): bytes 0x27..0x3e with a
//   big endian CRC16 at 0x3f, infinity DAC at 0x2d, macro DAC at 0x31 (both big endian).
#ifndef GUAMP_CAMERA_3A_CORE_H
#define GUAMP_CAMERA_3A_CORE_H
#include <stdint.h>
#include <stddef.h>

namespace gcam3a {

enum : int { kOk = 0, kErrInval = -22, kErrRange = -34, kErrCrc = -84 };

// ---- one CCI write --------------------------------------------------------------------------------------------------
struct Write { uint16_t addr; uint16_t data; uint8_t addrBytes, dataBytes; };

// ---- sensors ----------------------------------------------------------------------------------------------------------
enum Family : uint8_t { kSmia = 0, kOv02b = 1 };
struct Mode { uint16_t width, height, fll, llp; uint32_t linePs; };   // line period in picoseconds = 1e12 / (fps * fll)

struct ExpoSensor {
  const char *name;
  uint8_t cell, family;
  uint16_t regFll, regCoarse, regGain, regHold;   // regHold: SMIA grouped hold (0x0104); OV02B: unused
  uint16_t gainMinCode, gainMaxCode;              // 1x and maximum analog gain in register code
  uint16_t gainOne;                               // register code of 1x (Samsung 32, OV02B 16)
  uint8_t verticalOffset;                         // coarse integration <= FLL - offset
  uint16_t minLines;
  uint32_t maxFll;
  const Mode *modes; uint8_t nModes;
};

// Mode tables: size, FLL, LLP and the line period derived from the blob frame rate (fps x FLL x LLP = VT clock, about
// 482 MHz on the GM1ST, 272 MHz on the 4H7, 16.45 MHz on the OV02B).
constexpr Mode kGm1Modes[] = {
  {4000, 3000, 3181, 5024, 10423294},   // 30.16 fps
  {1920, 1080, 3197, 2512, 5211484},    // 60.02 fps, 2x2 binning (the mode GuampCamera uses)
  {1280, 720, 816, 5024, 10211567},     // 120.01 fps
};
constexpr Mode k4h7Modes[] = {{3264, 2448, 2530, 3580, 13162068}};    // 30.03 fps
constexpr Mode kOv02b10Modes[] = {{1600, 1200, 1221, 448, 27236476}}; // 30.07 fps
constexpr Mode kOv02b1bModes[] = {{1600, 1200, 1221, 448, 27155200}}; // 30.16 fps

constexpr uint16_t kOvVblankBase = 1220;   // FLL - vblank (0x4c4 in the vendor library)
constexpr uint8_t kOvPage = 0xfd, kOvVblankHi = 0x14, kOvVblankLo = 0x15, kOvExpHi = 0x0e, kOvExpLo = 0x0f,
                  kOvGain = 0x22, kOvTrigger = 0xfe;

constexpr unsigned kSensors = 4;
constexpr ExpoSensor kExpo[kSensors] = {
  {"s5kgm1st", 0, kSmia, 0x0340, 0x0202, 0x0204, 0x0104, 32, 512, 32, 4, 2, 47974, kGm1Modes, 3},
  {"s5k4h7", 1, kSmia, 0x0340, 0x0202, 0x0204, 0x0104, 32, 512, 32, 5, 2, 65530, k4h7Modes, 1},
  {"ov02b1b", 2, kOv02b, 0, kOvExpHi, kOvGain, 0, 16, 248, 16, 8, 4, kOvVblankBase + 0x7fff < 65527 ? kOvVblankBase + 0x7fff : 65527,
   kOv02b1bModes, 1},
  {"ov02b10", 3, kOv02b, 0, kOvExpHi, kOvGain, 0, 16, 248, 16, 8, 4, 32759, kOv02b10Modes, 1},
};

inline const ExpoSensor *expoSensor(unsigned cell) { return cell < kSensors ? &kExpo[cell] : nullptr; }

// Gain in Q8 (256 = 1x) -> register code, rounded to nearest and clamped; codeToQ8 is the exact inverse of a code.
inline uint16_t gainCode(const ExpoSensor &s, uint32_t gainQ8) {
  const uint32_t c = (gainQ8 * s.gainOne + 128u) / 256u;
  return static_cast<uint16_t>(c < s.gainMinCode ? s.gainMinCode : c > s.gainMaxCode ? s.gainMaxCode : c);
}
inline uint32_t codeToQ8(const ExpoSensor &s, uint16_t code) { return (static_cast<uint32_t>(code) * 256u) / s.gainOne; }

// Exposure request -> what will really be programmed.
struct Exposure { uint32_t lines, fll; uint16_t gainCode; uint32_t gainQ8; uint64_t exposureNs, frameNs; };

// lines: wanted coarse integration in lines; minFll: frame length wanted (0 = the mode's own), it grows by itself when the
// exposure needs it (lower frame rate) and never goes above maxFll.
inline int plan(const ExpoSensor &s, unsigned mode, uint32_t lines, uint32_t minFll, uint32_t gainQ8, Exposure *out) {
  if (mode >= s.nModes || !out) return kErrInval;
  const Mode &m = s.modes[mode];
  uint32_t fll = minFll < m.fll ? m.fll : minFll;
  if (fll > s.maxFll) fll = s.maxFll;
  const uint32_t maxLines = s.maxFll - s.verticalOffset;
  if (lines < s.minLines) lines = s.minLines;
  if (lines > maxLines) lines = maxLines;
  if (lines + s.verticalOffset > fll) fll = lines + s.verticalOffset;
  out->lines = lines;
  out->fll = fll;
  out->gainCode = gainCode(s, gainQ8);
  out->gainQ8 = codeToQ8(s, out->gainCode);
  out->exposureNs = static_cast<uint64_t>(lines) * m.linePs / 1000u;
  out->frameNs = static_cast<uint64_t>(fll) * m.linePs / 1000u;
  return kOk;
}

// Exposure time in ns -> lines (rounded to nearest, at least minLines) for a mode.
inline uint32_t linesForNs(const ExpoSensor &s, unsigned mode, uint64_t ns) {
  if (mode >= s.nModes) return s.minLines;
  const uint64_t l = (ns * 1000u + s.modes[mode].linePs / 2) / s.modes[mode].linePs;
  return l < s.minLines ? s.minLines : l > 0xffffffffu ? 0xffffffffu : static_cast<uint32_t>(l);
}

constexpr unsigned kMaxWrites = 8;

// CCI writes for one exposure, in order. SMIA (both Samsung sensors): grouped hold on, FLL, coarse, analog gain
// (16-bit each), hold off: the sensor applies all of them on the same frame. OV02B: page 1, vblank hi/lo, exposure
// hi/lo, gain, then the frame-synchronous trigger 0xfe = 0x02 (exactly the vendor library's 7 writes).
inline unsigned writes(const ExpoSensor &s, const Exposure &e, Write *w) {
  unsigned n = 0;
  if (s.family == kSmia) {
    w[n++] = {s.regHold, 1, 2, 1};
    w[n++] = {s.regFll, static_cast<uint16_t>(e.fll), 2, 2};
    w[n++] = {s.regCoarse, static_cast<uint16_t>(e.lines), 2, 2};
    w[n++] = {s.regGain, e.gainCode, 2, 2};
    w[n++] = {s.regHold, 0, 2, 1};
    return n;
  }
  const uint32_t vb = e.fll > kOvVblankBase ? e.fll - kOvVblankBase : 1u;
  w[n++] = {kOvPage, 0x01, 1, 1};
  w[n++] = {kOvVblankHi, static_cast<uint16_t>((vb >> 8) & 0x7f), 1, 1};
  w[n++] = {kOvVblankLo, static_cast<uint16_t>(vb & 0xff), 1, 1};
  w[n++] = {kOvExpHi, static_cast<uint16_t>((e.lines >> 8) & 0xff), 1, 1};
  w[n++] = {kOvExpLo, static_cast<uint16_t>(e.lines & 0xff), 1, 1};
  w[n++] = {kOvGain, e.gainCode, 1, 1};
  w[n++] = {kOvTrigger, 0x02, 1, 1};
  return n;
}

// ---- lens drivers (voice coil, 10-bit DAC) ---------------------------------------------------------------------------
enum LensVaf : uint8_t { kVafPm8008 = 0, kVafGpio = 1 };
struct LensStep { uint8_t reg, val; uint16_t delayUs; };
struct Lens {
  const char *name;
  uint8_t cell, cci, slave7, vaf;
  uint8_t vafLdo; uint32_t vafUv;            // kVafPm8008: PM8008 LDO number and voltage
  uint16_t vafGpio, vafRampUs;               // kVafGpio: TLMM gpio and regulator ramp
  const LensStep *init; uint8_t nInit;
  uint16_t defInf, defMacro;                 // DAC range used without EEPROM calibration
  uint16_t park;                             // DAC near which the lens rests unpowered
};
constexpr uint8_t kLensCtrl = 0x02, kLensDac = 0x03;   // DAC: 16-bit word, 10 bits used (MSB register 0x03)
constexpr uint16_t kDacMax = 1023;
// DW9767: same register set as the Giantec GT9767 clone of mot_actuator.c (power-down bit 0 of 0x02, SAC mode 0x06,
// resonance period 0x07/0x08). The guamp CamX actuator blob (com.qti.actuator.mot_dw9767) is not in the vendor
// partition we saved, so the values are those of the GT9767 list (caprip, same PM8008 platform): verify on the phone.
constexpr LensStep kDw9767Init[] = {{0x02, 0x00, 1000}, {0x06, 0x0a, 0}, {0x07, 0x01, 0}, {0x08, 0x43, 0}};
constexpr LensStep kGt9772Init[] = {{0xed, 0xab, 0}, {0x02, 0x01, 0}, {0x02, 0x00, 1000}, {0x06, 0x84, 0}, {0x07, 0x01, 0},
                                    {0x08, 0x62, 0}};
constexpr unsigned kLenses = 2;
constexpr Lens kLens[kLenses] = {
  {"dw9767", 0, 0, 0x0c, kVafPm8008, 5, 2800000, 0, 0, kDw9767Init, 4, 80, 600, 0},
  // The macro lens driver shares CCI master 0 and (per the mot_actuator.c defaults) slave 0x0c with the DW9767: only one
  // of the two may be powered at a time (GuampCamera3A.inc enforces it).
  {"gt9772", 3, 0, 0x0c, kVafGpio, 0, 0, 103, 135, kGt9772Init, 6, 200, 900, 0},
};
inline const Lens *lensForCell(unsigned cell) {
  for (unsigned i = 0; i < kLenses; ++i) if (kLens[i].cell == cell) return &kLens[i];
  return nullptr;
}
inline Write lensDac(uint16_t dac) { return {kLensDac, static_cast<uint16_t>(dac > kDacMax ? kDacMax : dac), 1, 2}; }
inline Write lensPowerDown() { return {kLensCtrl, 0x01, 1, 1}; }

// Moves from cur to target without a hard jump: steps of at most maxStep codes (the last one exact). Returns the number of
// DAC values written to out (<= cap); the caller waits stepUs between them.
inline unsigned lensRamp(uint16_t cur, uint16_t target, uint16_t maxStep, uint16_t *out, unsigned cap) {
  if (target > kDacMax) target = kDacMax;
  if (cur > kDacMax) cur = kDacMax;
  if (!maxStep) maxStep = 1;
  unsigned n = 0;
  while (cur != target && n < cap) {
    const uint16_t d = cur < target ? static_cast<uint16_t>(target - cur) : static_cast<uint16_t>(cur - target);
    const uint16_t s = d < maxStep ? d : maxStep;
    cur = cur < target ? static_cast<uint16_t>(cur + s) : static_cast<uint16_t>(cur - s);
    out[n++] = cur;
  }
  return n;
}

// Lens control over any CCI writer (GuampCamera3A.inc: gcam::Cci; tests: a simulated VCM). Moves of up to kLensJump codes
// are written at once (an AF step); longer ones in kLensJump steps kLensStepUs apart, park in kParkStep steps
// kParkStepUs apart, so the lens never slams into its end stop (the click Linux avoids with its launch/park tables).
constexpr uint16_t kLensJump = 120, kParkStep = 40;
constexpr unsigned kLensStepUs = 2000, kParkStepUs = 4000;
struct AfCal { uint16_t inf, macro; bool valid; };   // EEPROM calibration (parseAfCal below) or the defaults
struct LensIo {
  void *ctx;
  int (*write)(void *, uint8_t cci, uint8_t slave7, const Write &w);   // 0 or negative error
  void (*delayUs)(void *, unsigned us);
};
struct LensCtl {
  const LensIo *io = nullptr;
  const Lens *l = nullptr;
  uint16_t dac = 0, lo = 0, hi = kDacMax;   // current DAC, usable range (calibrated infinity .. macro, widened a bit)
  bool on = false;
  unsigned writes = 0;
  int lastError = 0;
  int wr(const Write &w) {
    ++writes;
    const int r = io->write(io->ctx, l->cci, l->slave7, w);
    if (r) lastError = r;
    return r;
  }
  void wait(unsigned us) { if (us) io->delayUs(io->ctx, us); }
  // Rail already on (caller). Init list, then the lens stays at its rest position (park).
  int powerUp(const LensIo *i, const Lens *lens, const AfCal &cal) {
    if (!i || !lens) return kErrInval;
    io = i; l = lens; on = false; dac = lens->park; lastError = 0;
    // 10 % margin around the calibration: infinity of a hot module or a lens pointing down sits slightly outside it.
    const uint16_t span = cal.macro > cal.inf ? static_cast<uint16_t>(cal.macro - cal.inf) : 0;
    const uint16_t m = static_cast<uint16_t>(span / 10);
    lo = cal.inf > m ? static_cast<uint16_t>(cal.inf - m) : 0;
    hi = cal.macro + m > kDacMax ? kDacMax : static_cast<uint16_t>(cal.macro + m);
    for (unsigned k = 0; k < lens->nInit; ++k) {
      const int r = wr({lens->init[k].reg, lens->init[k].val, 1, 1});
      if (r) return r;
      wait(lens->init[k].delayUs);
    }
    on = true;
    return kOk;
  }
  uint16_t clamp(uint16_t t) const { return t < lo ? lo : t > hi ? hi : t; }
  // AF move: inside [lo, hi]; long moves ramped.
  int move(uint16_t target) {
    if (!on) return kErrInval;
    target = clamp(target);
    uint16_t steps[kDacMax / kLensJump + 2];
    const unsigned n = lensRamp(dac, target, kLensJump, steps, sizeof(steps) / sizeof(steps[0]));
    for (unsigned k = 0; k < n; ++k) {
      if (k) wait(kLensStepUs);
      const int r = wr(lensDac(steps[k]));
      if (r) return r;
      dac = steps[k];
    }
    return kOk;
  }
  // Back to rest in small steps, then the driver's power-down bit (the caller then drops the rail).
  int park() {
    if (!on) return kOk;
    uint16_t steps[kDacMax / kParkStep + 2];
    const unsigned n = lensRamp(dac, l->park, kParkStep, steps, sizeof(steps) / sizeof(steps[0]));
    int r = kOk;
    for (unsigned k = 0; k < n && !r; ++k) {
      r = wr(lensDac(steps[k]));
      if (!r) dac = steps[k];
      wait(kParkStepUs);
    }
    const int p = wr(lensPowerDown());
    on = false;
    return r ? r : p;
  }
};

// ---- EEPROM AF calibration (main module)-----------------------------------------------------------------------------
constexpr uint16_t kAfBlock = 0x27, kAfLen = 24, kAfCrc = 0x3f, kAfInf = 0x2d, kAfMacro = 0x31;
constexpr uint8_t kEepromSlave7 = 0x50;    // the usual 0xA0 module EEPROM address: not in the saved blobs, to verify

// CRC16 polynomial 0x8005 in both bit orders (the vendor check reflects bits; which variant is stored is not certain):
// reflected (CRC-16/ARC) and plain (CRC-16/BUYPASS), both with init 0.
inline uint16_t crc16Arc(const uint8_t *p, size_t n) {
  uint16_t c = 0;
  for (size_t i = 0; i < n; ++i) {
    c ^= p[i];
    for (unsigned b = 0; b < 8; ++b) c = (c & 1) ? static_cast<uint16_t>((c >> 1) ^ 0xa001) : static_cast<uint16_t>(c >> 1);
  }
  return c;
}
inline uint16_t crc16Buypass(const uint8_t *p, size_t n) {
  uint16_t c = 0;
  for (size_t i = 0; i < n; ++i) {
    c ^= static_cast<uint16_t>(p[i] << 8);
    for (unsigned b = 0; b < 8; ++b) c = (c & 0x8000) ? static_cast<uint16_t>((c << 1) ^ 0x8005) : static_cast<uint16_t>(c << 1);
  }
  return c;
}

// buf = EEPROM bytes kAfBlock .. kAfCrc + 1 (26 bytes). Valid when the CRC matches (either variant), both DACs are in
// range and macro > inf + 50 (the vendor library rejects inverted values).
inline AfCal parseAfCal(const uint8_t *buf, size_t len, const Lens &l) {
  AfCal c = {l.defInf, l.defMacro, false};
  if (!buf || len < kAfLen + 2) return c;
  const uint16_t want = static_cast<uint16_t>(buf[kAfLen] << 8 | buf[kAfLen + 1]);
  if (crc16Arc(buf, kAfLen) != want && crc16Buypass(buf, kAfLen) != want) return c;
  const uint16_t inf = static_cast<uint16_t>(buf[kAfInf - kAfBlock] << 8 | buf[kAfInf - kAfBlock + 1]);
  const uint16_t mac = static_cast<uint16_t>(buf[kAfMacro - kAfBlock] << 8 | buf[kAfMacro - kAfBlock + 1]);
  if (inf > kDacMax || mac > kDacMax || mac <= inf + 50) return c;
  return {inf, mac, true};
}

}  // namespace gcam3a
#endif
