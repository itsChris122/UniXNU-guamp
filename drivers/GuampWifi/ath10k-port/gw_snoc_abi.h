/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#ifndef GW_SNOC_ABI_H
#define GW_SNOC_ABI_H
#include <stdint.h>
#include <stddef.h>
#include "nw_abi.h"
#ifdef __cplusplus
extern "C" {
#endif
struct gw_ce_pipe { uint32_t number,direction,entries,bytes,flags,reserved; };
struct gw_ce_service { uint32_t service,direction,pipe; };
struct gw_ce_shadow { uint16_t engine,offset; };
struct gw_snoc_board {
  void *ctx;
  void *registers; size_t register_bytes;
  uint32_t firmware_chip_id; /* read from WLFW_CAP, not an invented chip id */
  int (*power)(void *,int);
  int (*wlan_enable)(void *,const struct gw_ce_pipe *,unsigned,
                      const struct gw_ce_service *,unsigned,const struct gw_ce_shadow *,unsigned);
  int (*wlan_disable)(void *);
  /* Physical prerequisites for ath10k probe: modem transport/WLFW service,
   * MSA, board votes, SMMU SID1a0 and DMA resources are available. This must
   * be true BEFORE WLAN QMI enable. WLFW firmware startup reaches FirmwareReady
   * independently during provider bootstrap; wlan_enable() supplies CE tables
   * and selects mission mode after that state, then waits for Enabled. */
  int (*ready)(void *);
};
int gw_snoc_start(const struct lx_platform *,const struct gw_snoc_board *);
/* Only the serialized driver owner may call this. Services CE and schedules
 * NAPI without entering the dispatcher or station state machine; safe from
 * lx_platform.poll while a nested ath10k wait is pumping the dispatcher. */
void gw_snoc_poll_ce(void);
int gw_snoc_step(void);
int gw_snoc_stop(void);
#ifdef __cplusplus
}
#endif
#endif
