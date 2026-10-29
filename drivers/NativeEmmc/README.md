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
`GuampBt: RESULT` line. `guamp-bt=2` adds milestone M1 (btqca.c `qca_uart_setup` for the WCN3950-family controller):
`cmbtfw<rom>.tlv` and `cmnv<rom>.bin` read from `/image` of the `bluetooth_<slot>` partition (GuampExt4.inc, which
now also reads ext2 block maps), UART to 3 Mbit/s with the QCA vendor command (`guamp-bt-baud=`; RCG at 48 MHz),
rampatch and NVM in 243-byte segments, HCI_Reset, versions after the download and HCI_Read_BD_ADDR (never written);
the patched controller stays on (`guamp-bt-keep=0` powers it off). Host tests: `tests/run-bt-tests.sh
[bluetooth_a.img]` (WSL; the image is only read).

RPM client (GuampRpm.inc, protocol in GuampRpmCore.h): GLINK over the RPM message RAM (0x45f0000, TOC "grt0",
FIFOs ap2r/r2ap, APCS doorbell 0xf111008 bit 0, polled) and rpm-smd v0 requests, `rpm::set(type, id, key, value)` /
`rpm::request(...)` with timeouts, one request at a time. `guamp-rpm-test=1`: TOC/indices dump, handshake and one XO
enable vote (active set), ending in `GuampRpm: RESULT`; `=2` read-only dump. Users: `guamp-bt-ldo=3` votes L9A/L16A/
L17A/L23A on (swen + Linux's voltage, active and sleep set) before the UART step; `guamp-gpu-rpm=1` votes qdss and the
bimc/snoc/cnoc clocks (`guamp-gpu-rpm-khz`, default the RPM maximum) before the GCC GPU branches. Other arguments:
`guamp-rpm-ring=1` (keep the FIFO indices instead of Linux's reset), `guamp-rpm-fmt=1`, `guamp-rpm-log=0`,
`guamp-rpm-delay`. Host tests: `tests/run-rpm-tests.sh` (WSL).


Boot argument `guamp-torch=1` (GuampTorch.inc, default off): the PMI632 camera flash LED (SPMI slave 3, 0xd300) as a
torch at a conservative current (`guamp-torch-ma`, default 100, max 200 mA; Linux DT allows 500), auto-off after
`guamp-torch-sec` (default 600 s), off at every restart/power off/panic. Driven by the IOKit service `GuampTorch`
(root sets `TorchLevel` permille or `TorchOn`) or `guamp_torch_request()`. Host test: `tests/torch_host.cpp`.

Boot argument `guamp-backlight=1` (GuampBacklight.inc, default off): panel brightness and standby. The HX83102D's
level is its own DCS 0x51 (two bytes, high first, 16..0xccc, 0 = LEDs off) sent over DSI0 (0x5e94000, v2.4) through the
command TPG FIFO while the boot loader's video stream runs, scheduled into the vertical front porch as Linux does
(GuampBacklightCore.h); the DSI registers are touched only with the display clocks running and the controller in
video mode, every wait is bounded, 4 failed transfers in a row stop DSI access. Published as `AppleARMBacklight`
(conformance + `backlight-control`, sandbox grant `patches/xnu/zzwb-native-sandbox-grant-set-properties.patch`), so
CoreBrightness sets `brightness`/`FadePeriod`; display off (Power key, auto-lock) arrives as brightness 0.
`guamp-backlight-control=0` publishes without conformance, `guamp-backlight-test=1` dims to 1/8 for 3 s at start.
Host tests: `tests/run-backlight-tests.sh` (WSL).

Boot argument `guamp-camera=1` (GuampCamera.inc, default off): camera step 1, chip id only. After `guamp-camera-delay`
(default 15 s) a thread powers the PM8008 (QUP0 SE1 I2C, shared with the speaker amplifier under the audio lock),
turns on the CAMSS top GDSC and the CCI clock, then for each sensor in `guamp-camera-mask` (bit = DT cell, default
0xf) runs the power-up order of its sensormodule blob (L1P-L7P, MCLK 24 MHz from GPLL9 or 19.2 MHz TCXO with
`guamp-camera-mclk=19`, reset/pwdn GPIO), reads the chip id over CCI and powers it down; everything is off at the end.
One line per sensor: `GuampCamera: <sensor> chip id 0x.... (expected 0x....): OK|NACK|WRONG ID|FAILED`.
`guamp-camera-cci=100|400|1000` timing set, `guamp-camera-pmgpio=1` programs PM8008 GPIO1 like the DT.
GuampCameraRdi.h is the unused skeleton of the TPG -> CSID -> TFE RDI test frame. Sensor register tables are vendor
data and stay out of this repo (GCAM v1 file, parser `gcam::gcamFind`). Host tests: `tests/run-camera-tests.sh` (WSL).
