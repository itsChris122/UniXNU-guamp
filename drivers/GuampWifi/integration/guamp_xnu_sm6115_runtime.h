/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#ifndef GUAMP_XNU_SM6115_RUNTIME_H
#define GUAMP_XNU_SM6115_RUNTIME_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Single bootstrap owner, called by ModemBootHooks on the prep thread. prepare
 * requires that the loader has confirmed PAS4 stopped and verified its image.
 * Firmware-store callbacks must already be available. No caller-owned buffers
 * are retained. After start_station, the selected station worker owns polling.
 */
int guamp_sm6115_wifi_prepare(uint64_t managed_base,uint64_t managed_bytes);
int guamp_sm6115_wifi_modem_started(void);
void guamp_sm6115_wifi_shutdown_result(int stopped);
void guamp_sm6115_wifi_abort(void);
/* mode1: diagnostic scan worker; mode2: Redwood/iOS adapter (separate build
 * option GUAMP_WIFI_REDWOOD=1). Its bootstrap worker waits for the explicit
 * network_ready notification after BSD networking initialization.
 */
int guamp_sm6115_wifi_start_station(unsigned mode);
/* Call after redwood_net_init()/hakios_net_init() from the BSD init path. */
void guamp_sm6115_wifi_network_ready(void);
extern volatile unsigned guamp_wifi_provider_stage;
extern volatile int guamp_wifi_provider_error;
#ifdef __cplusplus
}
#endif
#endif
