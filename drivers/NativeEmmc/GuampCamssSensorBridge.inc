// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp_camss_sensor_* (GuampCamssApi.h) on top of the camera sensor session of GuampCamera.inc
// (camera::sensorOpen/sensorStream/sensorClose, branch codex/cam-sensori-20261105). Included by GuampCamss.inc only
// when GUAMP_CAMSS_SENSOR_BRIDGE is 1, i.e. in a kernel that has that session code; otherwise the weak stand-ins
// answer GUAMP_CAMSS_SENSOR_ABSENT.
// mode: 0 = low (GM1ST 1280x720, S5K4H7 1632x1224 binned, OV02B 1600x1200), 1 = full, 16 + n = vendor mode n.
// The sensor session answers kErrBusy while the guamp-camera=2 session test of GuampCamera.inc runs: retried 120 s.
#ifndef GUAMP_CAMSS_SENSOR_BRIDGE_INC
#define GUAMP_CAMSS_SENSOR_BRIDGE_INC
#include "GuampCamssApi.h"

extern "C" int guamp_camss_sensor_prepare(uint32_t cell, uint32_t mode, struct guamp_camss_sensor_link *link) {
  if (!link) return GUAMP_CAMSS_EINVAL;
  const unsigned res = mode == 1 ? 1u : 0u, idx = mode >= 16 ? mode - 16 : 0xffffu;
  gcam::Mipi m = {};
  int r = -1;
  for (unsigned t = 0; t < 60; ++t) {
    r = camera::sensorOpen(cell, res, idx, 10000, 256, &m);
    if (r != gcam::kErrBusy || t == 59) break;   // session in use (guamp-camera=2 test): wait for it
    IOSleep(2000);
  }
  if (r) return r;
  link->abi = GUAMP_CAMSS_ABI;
  link->phy = m.csiphy;
  link->lanes = m.lanes;
  link->lane_assign = m.laneAssign;
  link->lane_mask = 0;
  link->settle_count = m.settle ? m.settle : 14;
  link->data_rate_mbps = m.laneMbps;
  link->vc = m.vc;
  link->dt = m.dt;
  link->width = m.width;
  link->height = m.height;
  link->bayer = m.bayer;   // guamp_camss_bayer (Mipi::cfa is the CamX numbering, not the TFE one)
  link->combo = m.combo;
  IOLog("GuampCamss: sensor cell %u mode %u: csiphy %u, %u lanes (%x), %u Mbit/s, settle %u, vc %u dt %x, %ux%u, extra %ux%u dt %x\n",
        cell, mode, m.csiphy, m.lanes, m.laneAssign, m.laneMbps, m.settle, m.vc, m.dt, m.width, m.height, m.extraW, m.extraH,
        m.extraDt);
  return GUAMP_CAMSS_OK;
}
extern "C" int guamp_camss_sensor_stream(uint32_t, int on) { return camera::sensorStream(on != 0); }
extern "C" void guamp_camss_sensor_release(uint32_t) { camera::sensorClose(); }
#endif
