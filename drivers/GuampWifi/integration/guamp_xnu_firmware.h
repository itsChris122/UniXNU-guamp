/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#ifndef GUAMP_XNU_FIRMWARE_H
#define GUAMP_XNU_FIRMWARE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* GuampWifiPrep.inc exports these read-only callbacks once its ext4 store is ready.
 * Accepted paths: /image/wlanmdsp.mbn and /image/bdwlan_guamp_ipa.bin only.
 * Return values use GuampWifiWire.inc Error codes (0 success, negative failure).
 */
int guamp_wifi_firmware_file_size(const char *path, uint32_t *bytes);
int guamp_wifi_firmware_file_read(const char *path, uint32_t offset,
                                  uint8_t *output, uint32_t bytes);
#ifdef __cplusplus
}
#endif
#endif
