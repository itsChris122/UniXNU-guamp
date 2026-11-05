// SPDX-License-Identifier: BSD-2-Clause-Patent
// GuampCamMacroPolicy.h - when the iOS back stream should take its frames from the macro camera (OV02B10, cell 3)
// instead of the main one (S5KGM1ST, cell 0). Plain C (static inline, no headers beyond stdint), so the same file
// builds into the kernel (GuampCamIos.inc), the capture plug-in (package/GuampISP) and the host tests.
//
// Why inside the back stream: the iPad7,1 capture stack (CMCapture J120 plist) knows one back and one front camera;
// the macro control of the iOS Camera app exists only for virtual multi-camera devices (BuiltInTriple/DualWide with an
// ultra wide that focuses close), which need the Apple ISP to synchronise the constituents. The macro sees about the
// same angle as the main camera (71.6 against 68.4 degrees, GuampCameraSecondary.h) and, with the sensor-side flip,
// the same orientation: the plug-in can swap the source of the back stream like iPhone "auto macro" does, the app
// keeps its single back camera and its formats (gisp scales the 1600x1200 frame to the format's output size).
//
// Policy (per frame of the active source, hysteresis + dwell so it never flaps):
//   main -> macro: the main AF is parked at its near end (lens within kNearPermille of lensHi) and not in focus
//                  (state searching or failed) for kEnterFrames, with enough light (gain below kDarkGainQ8);
//   macro -> main: the macro AF sits at its far end (within kFarPermille of lensLo) for kExitFrames (subject went
//                  away), or the scene is too dark for the small 1.75 um sensor for kExitFrames;
//   after every switch, kDwellFrames before the next decision. Policy 0 = never, 2 = always macro.
#ifndef GUAMP_CAM_MACRO_POLICY_H
#define GUAMP_CAM_MACRO_POLICY_H
#include <stdint.h>

enum GuampMacroPolicy { kGuampMacroOff = 0, kGuampMacroAuto = 1, kGuampMacroAlways = 2 };
enum GuampMacroAf { kGuampAfIdle = 0, kGuampAfSearching = 1, kGuampAfLocked = 2, kGuampAfFailed = 3 };

#define GUAMP_MACRO_NEAR_PERMILLE 60u    // lens within 6 % of its near (macro) end
#define GUAMP_MACRO_FAR_PERMILLE  60u    // macro lens within 6 % of its far end
#define GUAMP_MACRO_ENTER_FRAMES  20u    // about 0.7 s at 30 fps
#define GUAMP_MACRO_EXIT_FRAMES   30u    // about 1 s
#define GUAMP_MACRO_DWELL_FRAMES  60u    // about 2 s after a switch
#define GUAMP_MACRO_DARK_GAIN_Q8  (8u << 8)   // main gain above 8x: too dark for the macro (no switch)
#define GUAMP_MACRO_EXIT_GAIN_Q8  (15u << 8)  // macro gain near its 15.5x maximum: back to the main camera

typedef struct GuampMacroSwitch {
  uint32_t policy;        // GuampMacroPolicy
  uint32_t onMacro;       // 1 = the back stream takes the macro (cell 3)
  uint32_t near, far, dark, dwell;
  uint32_t switches;      // for the logs
} GuampMacroSwitch;

typedef struct GuampMacroInput {
  uint32_t lensPos, lensLo, lensHi;   // AF DAC of the active source and its usable range (lo = infinity, hi = macro)
  uint32_t afState;                   // GuampMacroAf of the active source
  uint32_t gainQ8;                    // applied analogue x digital gain of the active source, Q8
} GuampMacroInput;

static inline void guamp_macro_init(GuampMacroSwitch *s, uint32_t policy) {
  s->policy = policy > (uint32_t)kGuampMacroAlways ? (uint32_t)kGuampMacroAuto : policy;
  s->onMacro = s->policy == kGuampMacroAlways;
  s->near = s->far = s->dark = s->dwell = s->switches = 0;
}

static inline int guamp_macro_lens_near(const GuampMacroInput *in, uint32_t permille, int nearEnd) {
  if (in->lensHi <= in->lensLo) return 0;
  const uint32_t span = in->lensHi - in->lensLo, band = span * permille / 1000u;
  return nearEnd ? in->lensPos + band >= in->lensHi : in->lensPos <= in->lensLo + band;
}

// One frame of the active source. Returns 1 when the back stream should be on the macro from the next frame.
static inline uint32_t guamp_macro_update(GuampMacroSwitch *s, const GuampMacroInput *in) {
  if (s->policy != kGuampMacroAuto) { s->onMacro = s->policy == kGuampMacroAlways; return s->onMacro; }
  if (s->dwell) { --s->dwell; return s->onMacro; }
  if (!s->onMacro) {
    const int stuck = guamp_macro_lens_near(in, GUAMP_MACRO_NEAR_PERMILLE, 1) &&
                      (in->afState == kGuampAfSearching || in->afState == kGuampAfFailed);
    s->near = stuck && in->gainQ8 < GUAMP_MACRO_DARK_GAIN_Q8 ? s->near + 1 : 0;
    if (s->near >= GUAMP_MACRO_ENTER_FRAMES) { s->onMacro = 1; s->near = s->far = s->dark = 0; s->dwell = GUAMP_MACRO_DWELL_FRAMES; ++s->switches; }
  } else {
    const int away = guamp_macro_lens_near(in, GUAMP_MACRO_FAR_PERMILLE, 0) && in->afState != kGuampAfSearching;
    s->far = away ? s->far + 1 : 0;
    s->dark = in->gainQ8 >= GUAMP_MACRO_EXIT_GAIN_Q8 ? s->dark + 1 : 0;
    if (s->far >= GUAMP_MACRO_EXIT_FRAMES || s->dark >= GUAMP_MACRO_EXIT_FRAMES) {
      s->onMacro = 0; s->near = s->far = s->dark = 0; s->dwell = GUAMP_MACRO_DWELL_FRAMES; ++s->switches;
    }
  }
  return s->onMacro;
}

#endif  // GUAMP_CAM_MACRO_POLICY_H
