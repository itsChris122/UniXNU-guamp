// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp camera, the three secondary cameras: S5K4H7 front (cell 1), OV02B1B depth (cell 2), OV02B10 macro (cell 3).
// Board roles and lens facts, CSIPHY lane masks (the OV02B pair shares CSIPHY1: the depth sensor sits on the combo half,
// whose clock lane is DL3), the sensor-side orientation fix of the macro, which cameras can stream together and the
// facts the iOS service publishes. Pure code over gcam::Reg; included by GuampCameraSensor.h, host-tested by
// tests/camera_secondary_host.cpp.
// Sources (studied, no code or table copied):
//   - DT dtbo (guamp, fragment of qcom,cam-sensor0..3 and qcom,eeprom0..3): csiphy-sd-index, cci-master,
//     sensor-position-roll/yaw (roll = Android SENSOR_ORIENTATION), gpios (MCLK, reset/pwdn), actuator-src,
//     led-flash-src, eeprom-src, regulators;
//   - sensormodule blobs (decoded by analisi/camera/tools/sensormodule.py): moduleConfiguration (cameraId, laneAssign,
//     isComboMode, focalLength, fNumber, actuator, eeprom, flash), resolutionData; the OV02B1B mode list ends with
//     page 1 register 0x12 = 0x03, the OV02B10 one does not (same module, the depth one is mirrored and flipped);
//   - techpack camera-bengal cam_csiphy_core.c / cam_csiphy_2_0_hwreg.h: in combo mode the PHY splits into DL0/DL1 +
//     CLK for the first sensor and DL2 + DL3 (as clock) for the second; GuampCamssCore.h phyLanePlan models the same.
#ifndef GUAMP_CAMERA_SECONDARY_H
#define GUAMP_CAMERA_SECONDARY_H
#include "GuampCameraCore.h"

namespace gcam {

// ---- roles ----------------------------------------------------------------------------------------------------------
// Use values equal GuampCamIosPosition (GuampCamIosAbi.h): back 0, front 1, depth 2, macro 3.
enum Use : uint8_t { kUseMain = 0, kUseFront = 1, kUseDepth = 2, kUseMacro = 3 };
enum Facing : uint8_t { kFacingBack = 0, kFacingFront = 1 };
// How iOS sees the cell: its own capture stream, a source inside the back stream (macro, see GuampCamMacroPolicy.h),
// or not at all (depth: iOS has no use for a lone 2 MP mono sensor, see analisi/camera/cam7-secondarie.md).
enum IosRole : uint8_t { kIosNone = 0, kIosStream = 1, kIosInBack = 2 };

struct Role {
  uint8_t use, facing, ios;
  uint16_t rollDeg, yawDeg;      // DT sensor-position-roll / -yaw
  uint16_t uprightRollDeg;       // roll after the sensor-side fix of postModeRegs (kPostUpright): 90 = like the main
  uint8_t mono, hasAf, hasFlash; // hasAf: voice coil on this module (GT9772 for the macro); hasFlash: PMI632 on its side
  uint16_t focalUm;              // blob focalLength (mm) in um
  uint16_t fNumberQ8;            // blob fNumber, Q8
  uint16_t pixelNm;              // pixel pitch of the full readout (GM1ST: 2x2 binned 12 MP readout, 1.6 um)
  uint16_t hfovCdeg;             // horizontal field of view of the full readout, centidegrees (iOS VideoFieldOfView)
  uint16_t minFocusMm;           // closest focus, 0 = fixed focus (front) or unknown; estimates, see below
  uint8_t cciMaster, mclk;       // for the logs (kBoard has the authoritative copy)
};

// Field of view: 2 atan(pixels x pitch / 2 / f) on the full readout (GM1ST 6.40 mm / 4.71 mm, S5K4H7 3.656 / 2.67,
// OV02B 2.80 / 1.94): the macro sees about the same angle as the main camera (71.6 against 68.4 degrees), so switching
// the back stream to it does not change the framing, only the resolution (2 MP) and the focus range.
// minFocusMm: main 100 (as the J120 back camera plist), macro about 30 (module class, GT9772 lens: to be measured).
constexpr Role kRole[kSensors] = {
  {kUseMain, kFacingBack, kIosStream, 90, 180, 90, 0, 1, 1, 4710, 435, 1600, 6840, 100, 0, 0},    // s5kgm1st f/1.7
  {kUseFront, kFacingFront, kIosStream, 270, 0, 270, 0, 0, 0, 2670, 563, 1120, 6880, 0, 1, 2},    // s5k4h7 f/2.2
  {kUseDepth, kFacingBack, kIosNone, 90, 180, 90, 1, 0, 0, 1940, 614, 1750, 7160, 0, 1, 3},       // ov02b1b f/2.4
  {kUseMacro, kFacingBack, kIosInBack, 270, 180, 90, 0, 1, 1, 1940, 614, 1750, 7160, 30, 0, 1},   // ov02b10 f/2.4
};

constexpr int cellFor(Use u) {
  for (unsigned i = 0; i < kSensors; ++i)
    if (kRole[i].use == u) return static_cast<int>(i);
  return -1;
}

// ---- CSIPHY lane mask (GuampCamssApi.h guamp_camss_sensor_link::lane_mask) ------------------------------------------
// bit 0 DL0, bit 1 CLK, bit 2 DL1, bit 3 DL2, bit 4 DL3. A normal sensor: its data lanes + CLK. The combo-half sensor
// (OV02B1B, laneAssign 0x2, isComboMode 1): DL2 + DL3, which clocks that half; the generic laneMaskFrom() of the CAMSS
// would enable CLK + DL2 and never see its clock. 0 = not a valid layout.
constexpr uint32_t kLaneDl0 = 1u << 0, kLaneClk = 1u << 1, kLaneDl1 = 1u << 2, kLaneDl2 = 1u << 3, kLaneDl3 = 1u << 4;
constexpr uint32_t laneBit(unsigned phys) { return phys == 0 ? kLaneDl0 : phys < 4 ? 1u << (phys + 1) : 0u; }
constexpr uint32_t laneMaskOf(unsigned lanes, uint16_t laneAssign, bool combo) {
  if (!lanes || lanes > 4 || (combo && lanes > 1)) return 0;
  uint32_t m = 0;
  for (unsigned i = 0; i < lanes; ++i) {
    const unsigned p = (laneAssign >> (4 * i)) & 0xfu;
    if (p > 3 || (combo && p != 2) || (m & laneBit(p))) return 0;
    m |= laneBit(p);
  }
  return m | (combo ? kLaneDl3 : kLaneClk);
}

// ---- streaming together ----------------------------------------------------------------------------------------------
// Three CSID/TFE pairs, one PHY per CSIPHY. The OV02B pair on CSIPHY1 works together only with the PHY in combo mode
// (programmed once, mask 0x1b: DL0 + CLK for the macro, DL2 + DL3 for the depth); the current CAMSS code opens one
// sensor at a time, so this is for later (stereo depth, a seamless macro switch). MCLKs, CCI slaves and resets are all
// distinct; L7P (VIO) and L3P (VANA of both OV02B) are shared and reference counted by ldoOn/ldoOff.
enum Together : uint8_t { kTogetherNo = 0, kTogetherYes = 1, kTogetherCombo = 2 };
struct LaneLayout { uint8_t csiphy, lanes, combo; uint16_t laneAssign; };
// laneAssign/combo per cell, the same facts as gcam::kPhy (GuampCameraSensor.h), lanes from the vendor modes.
constexpr LaneLayout kLanes[kSensors] = {{0, 4, 0, 0x3210}, {2, 4, 0, 0x3210}, {1, 1, 1, 0x0002}, {1, 1, 0, 0x0000}};
constexpr uint32_t laneMaskOfCell(unsigned cell) {
  return cell < kSensors ? laneMaskOf(kLanes[cell].lanes, kLanes[cell].laneAssign, kLanes[cell].combo) : 0u;
}
constexpr Together together(unsigned a, unsigned b) {
  if (a >= kSensors || b >= kSensors || a == b) return kTogetherNo;
  if (kLanes[a].csiphy != kLanes[b].csiphy) return kTogetherYes;
  if (kLanes[a].combo == kLanes[b].combo) return kTogetherNo;           // both want the same half
  return (laneMaskOfCell(a) & laneMaskOfCell(b)) ? kTogetherNo : kTogetherCombo;
}

// ---- sensor-side fixes after the mode list -----------------------------------------------------------------------
// The macro module is mounted like the depth one, upside down against the main camera (DT roll 270 against 90). The
// vendor fixes the depth sensor on the sensor (its mode list writes page 1 0x12 = 0x03, mirror + flip) and leaves the
// macro to the Android rotation. Here the macro gets the same mirror + flip, so all back cameras deliver frames in the
// main camera's orientation (the iOS back stream switches between them without any rotation in the ISP). OV02B page 1
// timing/readout registers take effect with 0xFE = 0x02 (as in the vendor init and the exposure writes).
// Mirroring and flipping an even-sized window reverses the 2x2 CFA: BGGR -> RGGB. To be checked on the first frame: if
// red and blue are swapped, the sensor shifts its window by one pixel and the CFA stays BGGR (guamp-camera-macro-flip=2).
enum PostFlags : uint8_t { kPostUpright = 1u << 0, kPostKeepCfa = 1u << 1 };
constexpr uint8_t kPostDefault = kPostUpright;
constexpr unsigned kPostRegsMax = 3;
// CamX cfa (0 BGGR, 1 GBRG, 2 GRBG, 3 RGGB, 4 Y) after mirror + flip of an even-sized window.
constexpr uint8_t cfaRotated180(uint8_t cfa) { return cfa < 4 ? static_cast<uint8_t>(3 - cfa) : cfa; }
// Registers to write after the mode list (sensor in standby); *cfa is updated to what the sensor then sends.
inline unsigned postModeRegs(unsigned cell, uint8_t flags, Reg *out, uint8_t *cfa) {
  if (cell >= kSensors || !(flags & kPostUpright) || kRole[cell].use != kUseMacro) return 0;
  out[0] = {0xfd, 0x01, 1, 1, 0};
  out[1] = {0x12, 0x03, 1, 1, 0};
  out[2] = {0xfe, 0x02, 1, 1, 0};
  if (cfa && !(flags & kPostKeepCfa)) *cfa = cfaRotated180(*cfa);
  return kPostRegsMax;
}
// guamp-camera-macro-flip boot-arg: 0 off, 1 mirror + flip with the CFA reversed (default), 2 mirror + flip, CFA kept.
constexpr uint8_t postFlagsFromArg(unsigned arg) {
  return arg == 0 ? 0 : arg == 2 ? static_cast<uint8_t>(kPostUpright | kPostKeepCfa) : kPostDefault;
}

// ---- what the iOS service publishes -----------------------------------------------------------------------------------
// Readout geometry per GuampCamIosMode-like request (0 preview, 1 video, 2 photo): the front uses its derived 2x2
// binned mode for preview and video and the full 3264x2448 for photos; the OV02B have one vendor mode (1600x1200).
struct IosFacts {
  uint8_t use, facing, mono, hasAf, hasFlash;
  uint16_t fullW, fullH, previewW, previewH;
  uint16_t focalUm, fNumberQ8, pixelNm, hfovCdeg, minFocusMm;
  uint16_t maxGainX16;           // max analogue gain x 16 (OV02B 15.5x = 248, SMIA 16x = 256), blob maxAnalogGain
  uint16_t chipId;               // kBoard
};
inline IosFacts iosFacts(unsigned cell) {
  IosFacts f = {};
  if (cell >= kSensors) return f;
  const Role &r = kRole[cell];
  f.use = r.use; f.facing = r.facing; f.mono = r.mono; f.hasAf = r.hasAf; f.hasFlash = r.hasFlash;
  static constexpr uint16_t kFull[kSensors][4] = {{4000, 3000, 1280, 720}, {3264, 2448, 1632, 1224}, {1600, 1200, 1600, 1200},
                                                   {1600, 1200, 1600, 1200}};
  f.fullW = kFull[cell][0]; f.fullH = kFull[cell][1]; f.previewW = kFull[cell][2]; f.previewH = kFull[cell][3];
  f.focalUm = r.focalUm; f.fNumberQ8 = r.fNumberQ8; f.pixelNm = r.pixelNm; f.hfovCdeg = r.hfovCdeg; f.minFocusMm = r.minFocusMm;
  static constexpr uint16_t kGain[kSensors] = {256, 256, 248, 248};
  f.maxGainX16 = kGain[cell];
  f.chipId = kBoard[cell].id;
  return f;
}

}  // namespace gcam
#endif
