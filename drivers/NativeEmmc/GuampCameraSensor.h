// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp camera, sensors: GCAM v2 tables (vendor init/mode/stream sequences, loaded at run time, never in this repo),
// mode choice (low / full), MIPI CSI-2 facts for the CAMSS side, exposure and gain registers for 3A, and a sensor
// session (power up -> chip id -> init -> mode -> exposure -> stream on/off -> status -> power down) over the CCI.
// Pure code over gcam::Ops, host-tested by tests/camera_host.cpp; the kernel glue is GuampCamera.inc.
// Sources (studied, no code or table copied):
//   - sensormodule blobs (CamX "Parameter Parser V2.0.0", decoded by analisi/camera/tools/sensormodule.py):
//     slave, chip id, resolutionData (lane count, outputPixelClock, line length, frame length, fps, binning),
//     streamConfiguration (vc, dt), register addresses of frame length / coarse integration / global gain,
//     exposure limits (verticalOffset, maxLineCount, minLineCount, maxAnalogGain);
//   - com.qti.sensor.mot_<sensor>.so (vendor exposure libraries, disassembled): gain code formulas and the exposure
//     register lists below (facts: register numbers and formulas, no code copied);
//   - SMIA/CCS standard register map (0x0005 frame count, 0x0100 mode select, 0x0104 grouped parameter hold,
//     0x0340 frame length, 0x0202 coarse integration, 0x0204 analogue gain, 0x034C/0x034E output size,
//     0x0900/0x0901 binning, 0x0383/0x0387 odd increments).
#ifndef GUAMP_CAMERA_SENSOR_H
#define GUAMP_CAMERA_SENSOR_H
#include "GuampCameraCore.h"

namespace gcam {

// ---- MIPI CSI-2 facts per board cell (DT csiphy-sd-index, blob moduleConfiguration) --------------------------------
// laneAssign: nibble k = physical data lane used for logical lane k (CamX); combo: the two OV02B share CSIPHY1.
struct Phy { uint8_t csiphy; uint16_t laneAssign; uint8_t combo; };
constexpr Phy kPhy[kSensors] = {{0, 0x3210, 0}, {2, 0x3210, 0}, {1, 0x0002, 1}, {1, 0x0000, 0}};
constexpr uint8_t kDtRaw10 = 0x2b, kDtPdaf = 0x30;   // PDAF stats of the GM1ST on the same VC (CSID must drop dt 0x30)

// ---- exposure and gain (vendor exposure libraries + blob limits) ----------------------------------------------------
// SMIA (S5KGM1ST, S5K4H7): frame length 0x0340/41, coarse integration 0x0202/03, analogue gain 0x0204/05 with
//   code = gain x 32 (0x20 = 1x .. 0x200 = 16x); GM1ST wraps the update in 0x0104 = 1 / 0 and has the long exposure
//   shifters 0x0702..0x0705 (0 = off, written once per configure). Lines <= frame length - verticalOffset.
// OV02B (both): page 0xFD = 1, 0x14/0x15 = frame length - 1220 (vertical blank, 15 bit, >= 1), 0x0E/0x0F = lines,
//   0x22 = gain x 16 (0x10 = 1x .. 0xF8 = 15.5x; steps 1/16, 1/8, 1/4, 1/2 above 1x, 2x, 4x, 8x), 0xFE = 0x02 applies.
enum ExpKind : uint8_t { kExpSmia = 0, kExpOv02b = 1 };
struct ExpModel {
  uint8_t kind, vOffset, minLines, gainShift;   // gainShift: Q8 gain >> shift = register code
  uint16_t gainMin, gainMax;                    // register codes
  uint32_t maxFll;                              // blob maxLineCount
  bool groupHold, longExp;
  uint16_t page;                                // S5KGM1ST: 0x6028 selects the page of direct accesses too (its
                                                // lists toggle 0x2000/0x4000 and end on 0x2000): 0x4000 first
};
constexpr uint16_t kOvVtsBase = 1220;
constexpr ExpModel kExp[kSensors] = {
  {kExpSmia, 4, 2, 3, 0x20, 0x200, 47974, true, true, 0x4000},   // s5kgm1st
  {kExpSmia, 5, 2, 3, 0x20, 0x200, 65530, false, false, 0},       // s5k4h7 (vendor group hold writes 0 twice: none)
  {kExpOv02b, 8, 4, 4, 0x10, 0xf8, 65527, false, false, 0},       // ov02b1b (15 bit blank: at most 1220 + 32767)
  {kExpOv02b, 8, 4, 4, 0x10, 0xf8, 32759, false, false, 0},       // ov02b10
};

// Request from 3A: lines of integration, gain in Q8 (256 = 1x), frame length (0 = keep the mode's, grown as needed).
struct ExposureRequest { uint32_t lines, gainQ8, fll; };
struct ExposureApplied { uint32_t lines, gainQ8, fll, code; uint64_t exposureNs; };

// ---- GCAM v2 ---------------------------------------------------------------------------------------------------------
// Little endian. "GCAM", u32 version = 2, u32 sensors; per sensor a 48 byte header, nModes 48 byte mode headers, then
// power-up steps (8 bytes {u8 type, u8 pad, u16 delayMs, u32 value}), init, stream on, stream off and each mode's
// register list (8 bytes {u16 addr, u16 data, u8 addrBytes, u8 dataBytes, u16 delayUs}). Made by
// analisi/camera/tools/sensormodule.py --gcam from the vendor blobs; the ADT carries it as /chosen/guamp-camera-gcam.
constexpr uint32_t kGcamVersion = 2;
constexpr unsigned kGcamSensorHdr = 48, kGcamModeHdr = 48, kGcamEntry = 8, kGcamMaxModes = 8;

inline uint16_t le16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | p[1] << 8); }
inline uint32_t le32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | static_cast<uint32_t>(p[3]) << 24; }
inline Reg gcamReg(const uint8_t *p) { return {le16(p), le16(p + 2), p[4], p[5], le16(p + 6)}; }

struct Mode {
  uint16_t width, height, x, y;
  uint8_t lanes, dt, vc, cfa;            // cfa: 0 BGGR, 1 GBRG, 2 GRBG, 3 RGGB, 4 Y (mono)
  uint32_t opPixClk;                     // outputPixelClock: lane bit rate = opPixClk x 10 / lanes
  uint32_t llpck, fll, fpsMilli;
  uint8_t hBin, vBin, settle, bits;
  uint16_t extraW, extraH;               // second stream (PDAF) on the same VC, 0 if none
  uint8_t extraDt, extraVc;
  uint16_t nRegs;
  const uint8_t *regs;
  uint8_t index, derived;                // derived: built here from a vendor mode (see deriveBinned)
  uint32_t laneMbps() const { return lanes ? static_cast<uint32_t>(static_cast<uint64_t>(opPixClk) * bits / lanes / 1000000u) : 0; }
  // line time from the frame timing CamX uses (vt clock = llpck x fll x fps): 1e12 / (fps x fll) ns, here in ps
  uint64_t linePs() const { return fpsMilli && fll ? 1000000000000000ull / (static_cast<uint64_t>(fpsMilli) * fll) : 0; }
};

struct Gcam {
  char name[16];
  uint8_t slave7, addrBytes, dataBytes, nModes;
  uint16_t idReg, id;
  uint16_t nPowerUp, nInit, nOn, nOff;
  uint16_t blackLevel, pixelW, pixelH;
  const uint8_t *powerUp, *init, *on, *off;
  Mode modes[kGcamMaxModes];
};

inline bool nameIs(const char *have, const char *want) {
  if (have[0] == 'm' && have[1] == 'o' && have[2] == 't' && have[3] == '_' &&
      !(want[0] == 'm' && want[1] == 'o' && want[2] == 't' && want[3] == '_')) have += 4;
  while (*have && *have == *want) { ++have; ++want; }
  return !*have && !*want;
}

// Finds a sensor by name ("mot_s5kgm1st" or "s5kgm1st"); every length and count is checked against len.
inline int gcamFind(const uint8_t *blob, size_t len, const char *name, Gcam *out) {
  if (!blob || len < 12 || blob[0] != 'G' || blob[1] != 'C' || blob[2] != 'A' || blob[3] != 'M') return kErrInval;
  if (le32(blob + 4) != kGcamVersion) return kErrInval;
  const uint32_t count = le32(blob + 8);
  size_t at = 12;
  for (uint32_t i = 0; i < count; ++i) {
    if (len - at < kGcamSensorHdr) return kErrRange;
    const uint8_t *h = blob + at;
    Gcam g = {};
    for (unsigned k = 0; k < 15; ++k) g.name[k] = static_cast<char>(h[k]);
    g.slave7 = h[16]; g.addrBytes = h[17]; g.dataBytes = h[18]; g.nModes = h[19];
    g.idReg = le16(h + 20); g.id = le16(h + 22);
    g.nPowerUp = le16(h + 24); g.nInit = le16(h + 26); g.nOn = le16(h + 28); g.nOff = le16(h + 30);
    g.blackLevel = le16(h + 32); g.pixelW = le16(h + 34); g.pixelH = le16(h + 36);
    at += kGcamSensorHdr;
    if (!g.nModes || g.nModes > kGcamMaxModes) return kErrRange;
    if (len - at < static_cast<size_t>(kGcamModeHdr) * g.nModes) return kErrRange;
    size_t regsTotal = static_cast<size_t>(g.nPowerUp) + g.nInit + g.nOn + g.nOff;
    for (unsigned m = 0; m < g.nModes; ++m) {
      const uint8_t *q = blob + at + kGcamModeHdr * m;
      Mode &d = g.modes[m];
      d.width = le16(q); d.height = le16(q + 2); d.x = le16(q + 4); d.y = le16(q + 6);
      d.lanes = q[8]; d.dt = q[9]; d.vc = q[10]; d.cfa = q[11];
      d.opPixClk = le32(q + 12); d.llpck = le32(q + 16); d.fll = le32(q + 20); d.fpsMilli = le32(q + 24);
      d.hBin = q[28]; d.vBin = q[29]; d.settle = q[30]; d.bits = q[31];
      d.extraW = le16(q + 32); d.extraH = le16(q + 34); d.extraDt = q[36]; d.extraVc = q[37]; d.nRegs = le16(q + 38);
      d.index = static_cast<uint8_t>(m);
      regsTotal += d.nRegs;
    }
    at += static_cast<size_t>(kGcamModeHdr) * g.nModes;
    if ((len - at) / kGcamEntry < regsTotal) return kErrRange;
    g.powerUp = blob + at; at += kGcamEntry * g.nPowerUp;
    g.init = blob + at; at += kGcamEntry * g.nInit;
    g.on = blob + at; at += kGcamEntry * g.nOn;
    g.off = blob + at; at += kGcamEntry * g.nOff;
    for (unsigned m = 0; m < g.nModes; ++m) { g.modes[m].regs = blob + at; at += kGcamEntry * g.modes[m].nRegs; }
    if (nameIs(g.name, name)) { *out = g; return kOk; }
  }
  return kErrRange;
}

// ---- modes -------------------------------------------------------------------------------------------------------------
enum Res : uint8_t { kResLow = 0, kResFull = 1 };

// SMIA 2x2 binning on top of a full vendor mode, same PLL and frame timing (so the same lane rate and fps): output
// size halved, binning 0x0900 = 1 / 0x0901 = 0x22, odd increments 3. Used for the S5K4H7, whose blob has only
// 3264x2448. Not a vendor setting: to be checked on the device (frame counter, then CSID line count).
constexpr unsigned kBinRegs = 8;
inline bool deriveBinned(const ExpModel &e, const Mode &full, Mode *out, Reg *overlay) {
  if (e.kind != kExpSmia || full.hBin != 1 || full.vBin != 1 || full.width < 640 || (full.width & 3) || (full.height & 3)) return false;
  *out = full;
  out->width = static_cast<uint16_t>(full.width / 2);
  out->height = static_cast<uint16_t>(full.height / 2);
  out->hBin = out->vBin = 2;
  out->derived = 1;
  out->extraW = out->extraH = 0;
  const Reg r[kBinRegs] = {
    {0x0900, 0x01, 2, 1, 0}, {0x0901, 0x22, 2, 1, 0}, {0x0383, 0x03, 2, 1, 0}, {0x0387, 0x03, 2, 1, 0},
    {0x034c, static_cast<uint16_t>(out->width >> 8), 2, 1, 0}, {0x034d, static_cast<uint16_t>(out->width & 0xff), 2, 1, 0},
    {0x034e, static_cast<uint16_t>(out->height >> 8), 2, 1, 0}, {0x034f, static_cast<uint16_t>(out->height & 0xff), 2, 1, 0},
  };
  for (unsigned i = 0; i < kBinRegs; ++i) overlay[i] = r[i];
  return true;
}

// MCLK 19.2 MHz instead of 24 (GPLL9 not locked): the sensor PLLs are programmed for 24 MHz, so every PLL output
// (pixel clock, lane rate, fps, line time) is 0.8x. Applied to the chosen mode so the lane rate given to CSIPHY, the
// fps and the exposure arithmetic (linePs) describe what the sensor really sends.
inline void modeScaleMclk(Mode *m, bool mclk24) {
  if (mclk24) return;
  m->opPixClk = static_cast<uint32_t>(static_cast<uint64_t>(m->opPixClk) * 4u / 5u);
  m->fpsMilli = m->fpsMilli * 4u / 5u;
}

// Low: the smallest vendor mode (GM1ST 1280x720 at 364 Mbit/s per lane, the easiest for a first CSIPHY lock); when
// the only vendor mode is wider than 2000 pixels, its SMIA 2x2 binned form. Full: the largest vendor mode.
// index < nModes forces that vendor mode. Returns false only when the table has no mode.
inline bool pickMode(const Gcam &g, const ExpModel &e, Res res, int index, Mode *out, Reg *overlay, unsigned *nOverlay) {
  *nOverlay = 0;
  if (!g.nModes) return false;
  if (index >= 0 && index < g.nModes) { *out = g.modes[index]; return true; }
  unsigned best = 0;
  for (unsigned m = 1; m < g.nModes; ++m) {
    const uint32_t a = static_cast<uint32_t>(g.modes[m].width) * g.modes[m].height;
    const uint32_t b = static_cast<uint32_t>(g.modes[best].width) * g.modes[best].height;
    if (res == kResFull ? a > b : a < b) best = m;
  }
  *out = g.modes[best];
  if (res == kResLow && out->width > 2000 && deriveBinned(e, g.modes[best], out, overlay)) *nOverlay = kBinRegs;
  return true;
}

// CamX colorFilterArrangement (blob, Mode::cfa: 0 BGGR, 1 GBRG, 2 GRBG, 3 RGGB, 4 Y) -> TFE pixel pattern order of
// GuampCamssApi.h (0 RGGB, 1 GRBG, 2 BGGR, 3 GBRG, 4 mono). The two numberings differ: never pass cfa through.
constexpr uint8_t kCfaToCamss[5] = {2, 3, 1, 0, 4};
constexpr uint8_t camssBayer(uint8_t cfa) { return cfa < 5 ? kCfaToCamss[cfa] : 0; }

// What the CAMSS side (CSIPHY/CSID/TFE RDI) needs for one stream.
struct Mipi {
  uint8_t cell, csiphy, lanes, combo, dt, vc, settle, cfa;   // cfa: CamX numbering (see camssBayer)
  uint16_t laneAssign, width, height;
  uint32_t laneMbps, strideBytes, fpsMilli;   // strideBytes: packed RAW10 line (width x 10 / 8)
  uint16_t extraW, extraH;
  uint8_t extraDt, bayer;                     // bayer: GuampCamssApi.h guamp_camss_bayer of cfa
};
inline Mipi mipiOf(const Sensor &s, const Mode &m) {
  const Phy &p = kPhy[s.cell];
  Mipi x = {};
  x.cell = s.cell; x.csiphy = p.csiphy; x.lanes = m.lanes; x.combo = p.combo; x.dt = m.dt; x.vc = m.vc; x.settle = m.settle;
  x.cfa = m.cfa; x.laneAssign = p.laneAssign; x.width = m.width; x.height = m.height; x.laneMbps = m.laneMbps();
  x.strideBytes = static_cast<uint32_t>(m.width) * 10u / 8u; x.fpsMilli = m.fpsMilli;
  x.extraW = m.extraW; x.extraH = m.extraH; x.extraDt = m.extraDt; x.bayer = camssBayer(m.cfa);
  return x;
}

// ---- exposure registers -----------------------------------------------------------------------------------------------
constexpr unsigned kExpRegsMax = 10;

inline uint32_t gainCode(const ExpModel &e, uint32_t gainQ8) {
  uint32_t c = gainQ8 >> e.gainShift;
  if (c < e.gainMin) c = e.gainMin;
  if (c > e.gainMax) c = e.gainMax;
  if (e.kind == kExpOv02b) c &= c >= 0x80 ? ~7u : c >= 0x40 ? ~3u : c >= 0x20 ? ~1u : ~0u;   // the sensor's real steps
  return c;
}

// Clamps the request to the sensor and mode, fills the register list (at most kExpRegsMax) and what was applied.
inline unsigned exposureRegs(const ExpModel &e, const Sensor &s, const Mode &m, const ExposureRequest &q, Reg *out,
                             ExposureApplied *applied) {
  uint32_t fll = q.fll ? q.fll : m.fll;
  if (fll < m.fll) fll = m.fll;
  uint32_t lines = q.lines < e.minLines ? e.minLines : q.lines;
  if (lines + e.vOffset > fll) fll = lines + e.vOffset;
  uint32_t maxFll = e.maxFll;
  if (e.kind == kExpOv02b && maxFll > kOvVtsBase + 0x7fffu) maxFll = kOvVtsBase + 0x7fffu;
  if (fll > maxFll) fll = maxFll;
  if (lines + e.vOffset > fll) lines = fll - e.vOffset;
  const uint32_t code = gainCode(e, q.gainQ8);
  unsigned n = 0;
  const uint8_t ab = s.addrBytes;
  auto put = [&](uint16_t a, uint32_t v) { out[n++] = {a, static_cast<uint16_t>(v & 0xff), ab, 1, 0}; };
  if (e.page) out[n++] = {0x6028, e.page, 2, 2, 0};
  if (e.kind == kExpSmia) {
    if (e.groupHold) put(0x0104, 1);
    put(0x0340, fll >> 8); put(0x0341, fll);
    put(0x0202, lines >> 8); put(0x0203, lines);
    put(0x0204, code >> 8); put(0x0205, code);
    if (e.groupHold) put(0x0104, 0);
  } else {
    uint32_t vb = fll > kOvVtsBase + 1 ? fll - kOvVtsBase : 1;
    put(0xfd, 0x01);
    put(0x14, (vb >> 8) & 0x7f); put(0x15, vb);
    put(0x0e, lines >> 8); put(0x0f, lines);
    put(0x22, code);
    put(0xfe, 0x02);
  }
  if (applied) {
    applied->lines = lines;
    applied->fll = fll;
    applied->code = code;
    applied->gainQ8 = code << e.gainShift;
    applied->exposureNs = m.linePs() * lines / 1000u;
  }
  return n;
}

// Lines for an exposure time (rounded to the nearest line, at least 1).
inline uint32_t linesForNs(const Mode &m, uint64_t ns) {
  const uint64_t lp = m.linePs();
  if (!lp) return 1;
  const uint64_t l = (ns * 1000u + lp / 2) / lp;
  return l ? static_cast<uint32_t>(l > 0xffffffu ? 0xffffffu : l) : 1;
}

// ---- session ---------------------------------------------------------------------------------------------------------
enum State : uint8_t { kOff = 0, kPowered = 1, kConfigured = 2, kStreaming = 3 };

struct Status {
  uint8_t state, modeSelect, frameCount, ok;   // ok: the readbacks below succeeded
  uint16_t id, width, height;                  // chip id, output size read back (SMIA 0x034C/0x034E; OV: mode's)
  uint32_t fll, lines, gainCode;               // read back from the sensor
};

struct Session {
  const Ops *o = nullptr;
  LdoState *ldo = nullptr;
  Cci *cci = nullptr;
  const CciTiming *timing = nullptr;
  const Sensor *s = nullptr;
  const Gcam *g = nullptr;
  Power pw = {};
  Mode mode = {};
  Reg overlay[kBinRegs] = {};
  unsigned nOverlay = 0;
  ExposureApplied exp = {};
  uint8_t state = kOff;
  bool mclk24 = true;                          // false: MCLK is 19.2 MHz, mode timing scaled by 0.8 (modeScaleMclk)
  unsigned written = 0;                        // registers written by the last list (for the logs)

  const ExpModel &model() const { return kExp[s->cell]; }

  int read(uint32_t reg, uint8_t *b, unsigned n) {
    int r = kErrIo;
    for (unsigned t = 0; t < 3; ++t) {
      r = cci->read(s->cci, s->slave7, reg, s->addrBytes, b, n);
      if (!r) break;
      cci->recover(s->cci, *timing);
    }
    return r;
  }
  int write8(uint16_t reg, uint8_t v) { const Reg x = {reg, v, s->addrBytes, 1, 0}; return cci->writeRegs(s->cci, s->slave7, &x, 1); }
  int list(const uint8_t *p, unsigned n) {
    written = 0;
    return cci->writeList(s->cci, s->slave7, n, [p](unsigned i) { return gcamReg(p + kGcamEntry * i); }, &written);
  }

  // Power up and chip id. On any error everything is off again.
  int open(const Ops &ops, LdoState &l, Cci &c, const CciTiming &t, const Sensor &sensor, const Gcam *table) {
    o = &ops; ldo = &l; cci = &c; timing = &t; s = &sensor; g = table;
    pw = {o, ldo, s};
    int r = pw.up();
    if (!r) {
      uint8_t b[2] = {};
      r = read(s->idReg, b, s->idBytes);
      const uint16_t id = s->idBytes == 2 ? static_cast<uint16_t>(b[0] << 8 | b[1]) : b[0];
      if (!r && id != s->id) { note(*o, "sensor: wrong chip id (read, expected)", kErrMismatch, id, s->id); r = kErrMismatch; }
    }
    if (r) { pw.down(); state = kOff; return r; }
    state = kPowered;
    return kOk;
  }

  // Init + mode + derived overlay + long exposure off + first exposure; the sensor stays in standby (no stream).
  int configure(Res res, int index, const ExposureRequest &first) {
    if (state < kPowered || !g) return kErrInval;
    if (g->slave7 != s->slave7 || g->id != s->id || g->addrBytes != s->addrBytes) return kErrMismatch;
    if (!pickMode(*g, model(), res, index, &mode, overlay, &nOverlay)) return kErrRange;
    modeScaleMclk(&mode, mclk24);
    int r = list(g->init, g->nInit);
    if (r) { note(*o, "sensor: init list failed (written, total)", r, written, g->nInit); return r; }
    r = list(mode.regs, mode.nRegs);
    if (r) { note(*o, "sensor: mode list failed (written, total)", r, written, mode.nRegs); return r; }
    if (nOverlay) {
      r = cci->writeRegs(s->cci, s->slave7, overlay, nOverlay, &written);
      if (r) { note(*o, "sensor: binning overlay failed (written)", r, written, nOverlay); return r; }
    }
    if (model().longExp) {
      const Reg pg = {0x6028, model().page, 2, 2, 0};
      if (model().page) r = cci->writeRegs(s->cci, s->slave7, &pg, 1);
      for (uint16_t a = 0x0702; a <= 0x0705 && !r; ++a) r = write8(a, 0);
    }
    if (!r) r = setExposure(first);
    if (r) return r;
    state = kConfigured;
    return kOk;
  }

  int setExposure(const ExposureRequest &q) {
    if (state < kPowered) return kErrInval;
    Reg regs[kExpRegsMax];
    const unsigned n = exposureRegs(model(), *s, mode, q, regs, &exp);
    return cci->writeRegs(s->cci, s->slave7, regs, n, &written);
  }

  int streamOn() {
    if (state != kConfigured) return state == kStreaming ? kOk : kErrInval;
    int r = list(g->on, g->nOn);
    if (!r) state = kStreaming;
    return r;
  }
  int streamOff() {
    if (state != kStreaming) return kOk;
    int r = list(g->off, g->nOff);
    state = kConfigured;
    return r;
  }

  // Readback for the diagnostics: SMIA mode select, frame counter, output size, timing; OV02B page 1 timing.
  int status(Status *st) {
    *st = {};
    st->state = state;
    st->width = mode.width;
    st->height = mode.height;
    if (state < kPowered) return kOk;
    uint8_t b[4] = {};
    int r = read(s->idReg, b, s->idBytes);
    if (!r) st->id = s->idBytes == 2 ? static_cast<uint16_t>(b[0] << 8 | b[1]) : b[0];
    if (model().kind == kExpSmia) {
      if (!r && model().page) { const Reg pg = {0x6028, model().page, 2, 2, 0}; r = cci->writeRegs(s->cci, s->slave7, &pg, 1); }
      if (!r) r = read(0x0005, b, 1);
      if (!r) st->frameCount = b[0];
      if (!r) r = read(0x0100, b, 1);
      if (!r) st->modeSelect = b[0];
      if (!r) r = read(0x034c, b, 4);
      if (!r) { st->width = static_cast<uint16_t>(b[0] << 8 | b[1]); st->height = static_cast<uint16_t>(b[2] << 8 | b[3]); }
      if (!r) r = read(0x0340, b, 2);
      if (!r) st->fll = static_cast<uint32_t>(b[0] << 8 | b[1]);
      if (!r) r = read(0x0202, b, 4);
      if (!r) { st->lines = static_cast<uint32_t>(b[0] << 8 | b[1]); st->gainCode = static_cast<uint32_t>(b[2] << 8 | b[3]); }
    } else {
      if (!r) r = write8(0xfd, 0x01);
      if (!r) r = read(0x14, b, 2);
      if (!r) st->fll = (static_cast<uint32_t>(b[0] & 0x7f) << 8 | b[1]) + kOvVtsBase;
      if (!r) r = read(0x0e, b, 2);
      if (!r) st->lines = static_cast<uint32_t>(b[0] << 8 | b[1]);
      if (!r) r = read(0x22, b, 1);
      if (!r) st->gainCode = b[0];
      st->modeSelect = state == kStreaming;
    }
    st->ok = !r;
    return r;
  }

  int close() {
    int r = streamOff();
    const int d = state >= kPowered ? pw.down() : kOk;
    state = kOff;
    return r ? r : d;
  }
};

}  // namespace gcam
#endif
