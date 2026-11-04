// GuampCamIosHooks.inc - integration glue (agent 6, codex/cam-integrazione-20261105): the hardware hooks that
// GuampCamIos.inc resolves weakly, wired to the sensor session (GuampCamera.inc, agent 1), the lens drivers
// (GuampCamera3A.inc, agent 4) and the photo flash (GuampTorch.inc in PmicService.inc, agent 4).
// Included after GuampCamera.inc, GuampCamera3A.inc, PmicService.inc and GuampCamIos.inc. Nothing here runs unless the
// iOS service (guamp-camera-ios=1) has a client and a sensor session is open: all default off.
// Cells: 0 = s5kgm1st (back, lens dw9767), 1 = s5k4h7 (front, fixed focus), 3 = ov02b10 (macro, lens gt9772).

namespace camera {
// Exposure for the open session: time in us -> lines of the current mode, Q8 gain; reports what the sensor got.
int camiosExposure(uint32_t cell, uint32_t us, uint32_t gainQ8, uint32_t *appliedUs, uint32_t *appliedQ8) {
  if (!gSessionLock) return gcam::kErrNoMem;
  IOLockLock(gSessionLock);
  int r = gSess.state < gcam::kConfigured || !gSess.s || gSess.s->cell != cell ? gcam::kErrInval : 0;
  if (!r) {
    const gcam::ExposureRequest q = {gcam::linesForNs(gSess.mode, static_cast<uint64_t>(us) * 1000u), gainQ8 ? gainQ8 : 256u, 0};
    r = gSess.setExposure(q);
    if (appliedUs) *appliedUs = static_cast<uint32_t>(gSess.exp.exposureNs / 1000u);
    if (appliedQ8) *appliedQ8 = gSess.exp.gainQ8;
  }
  IOLockUnlock(gSessionLock);
  return r;
}

// Lens of the open session: opened on first use (EEPROM range for the back module), dac = UINT32_MAX only reports the
// range. Closed with the session (sensorClose -> af::lensClose).
int camiosFocus(uint32_t cell, uint32_t dac, uint32_t *lo, uint32_t *hi) {
  if (!gSessionLock) return gcam::kErrNoMem;
  IOLockLock(gSessionLock);
  int r = gSess.state < gcam::kPowered || !gSess.s || gSess.s->cell != cell || !gHwUp ? gcam::kErrInval : 0;
  if (!r && !af::gLens.on) r = af::lensOpen(gCciDev, *gTiming, cell);
  if (!r) {
    if (lo) *lo = af::gLens.lo;
    if (hi) *hi = af::gLens.hi;
    if (dac != UINT32_MAX) r = af::lensSet(static_cast<uint16_t>(dac > 0xffffu ? 0xffffu : dac));
  }
  IOLockUnlock(gSessionLock);
  return r;
}
}  // namespace camera

extern "C" {
int guamp_camios_hw_exposure(uint32_t cell, uint32_t exposureUs, uint32_t gainQ8, uint32_t *appliedUs, uint32_t *appliedQ8) {
  return camera::camiosExposure(cell, exposureUs, gainQ8, appliedUs, appliedQ8);
}
int guamp_camios_hw_focus(uint32_t cell, uint32_t dac, uint32_t *lo, uint32_t *hi) {
  return camera::camiosFocus(cell, dac, lo, hi);
}
// kind 1 pre-flash, 2 main flash, 0 off; mA/ms 0 = GuampTorch defaults. guamp-flash=0 makes the PMIC thread refuse it.
int guamp_camios_hw_flash(uint32_t kind, uint32_t ma, uint32_t ms) {
  if (kind > 2) return gcam::kErrInval;
  pmic::torch::flashRequest(kind, ma, ms);
  return 0;
}
}
