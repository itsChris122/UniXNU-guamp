# Wi-Fi mode 2: en0, station boundary, Apple80211

Built only with `scripts/build-arm-driver.py NativeEmmc --wifi-runtime --wifi-redwood` (GUAMP_WIFI_REDWOOD=1) and
active only with the boot argument `guamp-wifi-start=2`.

| File | Origin | Role |
|---|---|---|
| `sys/hakios.net.h`, `sys/hakios.sta.h` | HakiOS `xnu/files/bsd/sys/` | the boundary `integration/guamp_xnu_station.c` is written against |
| `guamp_net.c` | HakiOS `bsd/dev/hakios.net.c` | `en0` through the BSD interface KPI, link up/down events, the `bsd_init` hook `guamp_wifi_bsd_network_ready()` |
| `guamp_sta.c` | HakiOS `bsd/dev/hakios.sta.c` | one thread that runs scan/join/power/leave on the station, last scan kept |
| `guamp_apple80211.c` | HakiOS `bsd/dev/hakios.apple80211.c` | SIOCGA80211/SIOCSA80211 answers and the IEEE 802.11 kernel events for wifid |
| `../../NativeEmmc/GuampWlanNub.inc` | HakiOS `iokit/Kernel/hakios.wlan.cpp` | `IO80211Controller` + `IONetworkInterface` registry entries for en0 |

HakiOS: <https://github.com/oosexgang/hackios-xnu-kernel>, commit 8851317. Those kernel files are a derivative work of
Apple's XNU under the **Apple Public Source License 2.0** (HakiOS `LICENSE`, `LICENSE.APSL`); the copies here keep that
license and say at the top what was changed: built outside the XNU tree without the BSD-private layer (IOMallocData, lock groups made at
run time, no startup-section macros), no virtio link, Guamp log prefixes (`GuampWiFi: net:`, `GuampWiFi: sta:`,
`GuampWiFi: apple80211:`, `GuampWLAN:`) and boot arguments (`guamp-net-trace=N`, `guamp-net-names=1`).

The Apple80211 layouts are the ones HakiOS read out of iOS 16.5. On iPadOS 17.7.11 they are not verified: every request
is logged (process, get/set, type, length, refusal) to find out what wifid asks and accepts. WPA2 is not claimed to work.
