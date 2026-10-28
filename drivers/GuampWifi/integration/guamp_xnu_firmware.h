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
/* RMTFS EFS partitions (GuampWifiPrep.inc): index 0 modemst1, 1 modemst2, 2 fsc, 3 fsg (fsg_<slot> on A/B).
 * Offsets/sizes are 512-byte multiples inside the partition. Writes: modemst1/2 only, and only with the boot
 * argument guamp-rmtfs-write=1 (Unsupported otherwise). */
int guamp_rmtfs_partition(unsigned index, const char *label, uint64_t *bytes);
int guamp_rmtfs_partition_read(unsigned index, uint64_t offset, uint8_t *output, uint32_t bytes);
int guamp_rmtfs_partition_write(unsigned index, uint64_t offset, const uint8_t *input, uint32_t bytes);
#ifdef __cplusplus
}
#endif
#endif
