/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#ifndef GUAMP_XNU_STATION_H
#define GUAMP_XNU_STATION_H

struct lx_platform;
struct gw_snoc_board;

#ifdef __cplusplus
extern "C" {
#endif

/* Call once after BSD networking is initialized and the SNOC/firmware board
 * provider is ready. Requires HackiOS-compatible redwood_sta/redwood_net. */
int guamp_xnu_wifi_attach(const struct lx_platform *platform,
    const struct gw_snoc_board *board);

#ifdef __cplusplus
}
#endif

#endif
