# NativeEmmc

guamp eMMC (SDHC1, sdhci-msm-v5) for XNU: polled PIO, the whole user area as a read-only IOBlockStorageDevice,
and the kernel message buffer mirrored to the `kpan` partition every second. Started by the device copy of
ArmDTPlatform (`guamp_emmc_start`), built by `scripts/build-arm-driver.py NativeEmmc`.

Boot argument `guamp-wifi-prep=1` (GuampWifiPrep.inc): once after the devices are published, a read-only check for
the Wi-Fi work: SMEM partitions/items and the modem SMP2P state (GuampSmem.inc), and the modem firmware in the
active `modem_<slot>` ext4 partition (GuampExt4.inc) verified segment by segment against the SHA-384 hash table of
modem.mdt (GuampFirmware.inc). Host tests: `tests/run-tests.sh <modem_a.img> <extracted dir>` (WSL).

Boot argument `guamp-bt=1` (BtService.inc): Bluetooth milestone M0, a one-shot probe of the WCN39xx controller: the
PM6125 BT rails over SPMI (`guamp-bt-ldo=1` tries to enable the ones that are off), QUPv3 SE3 as a polled GENI UART
(GeniUart.h), the QCA power pulses, HCI_Reset, Read_Local_Version and the QCA EDL version, ending in one
`GuampBt: RESULT` line. Host tests: `tests/run-bt-tests.sh` (WSL).


Boot argument `guamp-torch=1` (GuampTorch.inc, default off): the PMI632 camera flash LED (SPMI slave 3, 0xd300) as a
torch at a conservative current (`guamp-torch-ma`, default 100, max 200 mA; Linux DT allows 500), auto-off after
`guamp-torch-sec` (default 600 s), off at every restart/power off/panic. Driven by the IOKit service `GuampTorch`
(root sets `TorchLevel` permille or `TorchOn`) or `guamp_torch_request()`. Host test: `tests/torch_host.cpp`.
