/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/* Public runtime entry points for the WCN3990 SNOC adapter. */
#include "gw_snoc_abi.h"
#include <linux/errno.h>

static int g_started;

int nw_wifi_start_snoc(const struct lx_platform *platform,
                       const struct gw_snoc_board *board) {
  int expected = 0;
  if (!__atomic_compare_exchange_n(&g_started, &expected, -1, false,
                                   __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
    return -EBUSY;
  int error = gw_snoc_start(platform, board);
  __atomic_store_n(&g_started, error ? 0 : 1, __ATOMIC_RELEASE);
  return error;
}

int nw_wifi_step(void) {
  if (__atomic_load_n(&g_started, __ATOMIC_ACQUIRE) != 1) return -ENODEV;
  return gw_snoc_step();
}

void nw_wifi_stop(void) {
  int expected = 1;
  if (!__atomic_compare_exchange_n(&g_started, &expected, -1, false,
                                   __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
    return;
  int error = gw_snoc_stop();
  __atomic_store_n(&g_started, 0, __ATOMIC_RELEASE);
  (void)error; /* gw_snoc_stop retains DMA if firmware shutdown is unconfirmed. */
}
