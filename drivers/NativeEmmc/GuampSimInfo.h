/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * SIM snapshot shared by the Wi-Fi/modem runtime (producer, GuampSim.inc SimProbe) and the
 * GuampSim IOKit service in NativeEmmc (GuampSimService.inc), which publishes it in the
 * registry for the userland CoreTelephony stand-in (guamp-ctd). Plain C layout.
 */
#ifndef GUAMP_SIM_INFO_H
#define GUAMP_SIM_INFO_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
enum { GUAMP_SIM_INFO_VERSION = 1 };
enum GuampSimState {
  GUAMP_SIM_UNKNOWN = 0, GUAMP_SIM_ABSENT = 1, GUAMP_SIM_ERROR = 2, GUAMP_SIM_DETECTED = 3,
  GUAMP_SIM_PIN = 4, GUAMP_SIM_PUK = 5, GUAMP_SIM_BLOCKED = 6, GUAMP_SIM_PERSO = 7, GUAMP_SIM_READY = 8
};
struct GuampSimInfo {
  uint32_t version;
  uint32_t snapshots;
  uint8_t fake;            /* 1: guamp-sim-fake=1 test values, no modem involved */
  uint8_t state;           /* enum GuampSimState */
  uint8_t appType;         /* QMI UIM: 1 SIM, 2 USIM */
  uint8_t pin1Retries, puk1Retries;
  uint8_t operatingMode;   /* QMI DMS: 0 online, 1 low power, ... 0xFF unknown */
  uint8_t registration;    /* QMI NAS: 0 not registered, 1 registered, 2 searching, 3 denied, 4 unknown, 0xFF n/a */
  uint8_t csAttached, psAttached, roaming, radio;
  uint8_t hasPlmn;
  uint16_t mcc, mnc;
  char iccid[23], imsi[17], spn[17], msisdn[24], imei[17], meid[17], revision[64], plmn[33];
};
/* NativeEmmc: create/refresh the GuampSim registry service (thread context only). */
void guamp_sim_publish(const struct GuampSimInfo *info);
#ifdef __cplusplus
}
#endif
#endif
