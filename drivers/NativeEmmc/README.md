# NativeEmmc

guamp eMMC (SDHC1, sdhci-msm-v5) for XNU: polled PIO, the whole user area as a read-only IOBlockStorageDevice,
and the kernel message buffer mirrored to the `kpan` partition every second. Started by the device copy of
ArmDTPlatform (`guamp_emmc_start`), built by `scripts/build-arm-driver.py NativeEmmc`.
