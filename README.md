# UniXNU-gta4l

Device repository for the Samsung Galaxy Tab A7 10.4 (2020) — `gta4l`,
SM-T500 / SM-T505, Qualcomm **SM6115** (Snapdragon 662, Kryo 260: 4× A53 +
4× A73).

It describes the board to the [UniXNU](https://github.com/UniXNU/UniXNU) build
driver: the target IPSW, the kernel configuration, the prelink roots, the
firmware checkout, the ESP payload and the Apple-format device tree.

## Status

| Layer | State |
| --- | --- |
| Board description | `board.toml` parses; `./unixnu defconfig` / `fetch` / `tree` run |
| Firmware | Builds on Linux: Mu-Silicium `gta4lPkg` + `UniXNUPkg` boot manager + `DarwinBootPkg`, `SILICIUM_UEFI.fd` = 3,145,728 B |
| ADT | `scripts/make-gta4l-adt.py` emits 13,180 B for the 8 Kryo 260 cores |
| Kernel | **Not built here** — the UniXNU kernel step needs macOS + Xcode |
| Payload / boot.img | **Not built** — depends on the kernel artifacts |
| Boot on hardware | **Not demonstrated under UniXNU**. The device currently boots the separate ios-topaz 16 KB build to the XNU console |

Nothing below claims more than the table.

## Hardware facts used here

Taken from the gta4l device tree and the live Android DT, not invented:

| Item | Value |
| --- | --- |
| SoC / chip id | SM6115, `0x1bc` (444) |
| Cores | Silver `0x0-0x3`, Gold `0x100-0x103` (MPIDR Affinity1 numbering) |
| RAM window | `0x8a000000` + `0x76000000` (3 GiB variant → `0x100000000`) |
| GIC | distributor `0x0f200000`/`0x10000`, redistributor `0x0f300000`/`0x100000`, stride `0x20000` |
| Timebase | 19.2 MHz |
| Display | `0x5c000000`, 1200×2000, stride 4800, `a8r8g8b8` |
| Storage | eMMC 5.1 at `0x04744000` (SDHCI) — **no UFS**, unlike sheng |
| Touch | SPI0 `novatek,nt36523-spi` (also himax/focaltech variants), IRQ GPIO 80, reset GPIO 74 |
| ABL DTB | `qcom,msm-id = <0x1bc 0x10000>`, `qcom,board-id = <0x1000b 0x00>`, boot header v2, page 4096 |
| Firmware | Mu-Silicium tree, `Resources/Configs/gta4l.toml`, `Platforms/Samsung/gta4lPkg/gta4l.dsc` |

## Build

The UniXNU driver is macOS-only for the kernel step; everything else runs on
Linux.

```sh
git clone --recurse-submodules https://github.com/UniXNU/UniXNU
git clone --recurse-submodules <this repo> UniXNU-gta4l

cd UniXNU
./unixnu defconfig gta4l_defconfig -D ../UniXNU-gta4l
./unixnu fetch
./unixnu tree
./unixnu build firmware --uefi ../mu-gta4l      # Linux ok
./unixnu build kernel                          # macOS + Xcode
./unixnu build package
./unixnu build payload --unique-chip-id 0x<device seed>
./unixnu build boot
```

`scripts/check-gta4l.py` runs as the firmware preflight: it checks the board
name, the CPU MPIDR set and that the build tree carries DarwinBoot, the driver
builder and NativePrelink.

### Firmware changes required for UniXNU

The stock Mu-Silicium `gta4lPkg` has no loaders, so this port adds to the local
`mu-gta4l` checkout:

- `Platforms/Samsung/gta4lPkg/gta4l.dsc`: `UniXNULib` mapping; components
  `UniXNUPkg/Application/UniXNU/UniXNU.inf`, `UniXNUPkg/Driver/UniXNUDxe/UniXNUDxe.inf`,
  `DarwinBootPkg/Driver/DarwinBoot.inf`
- `Platforms/Samsung/gta4lPkg/Include/DXE.inc`: the same three `INF` lines

`board.toml` already adds `{tree}/package` and `{tree}/drivers/DarwinBoot` to
the firmware package path.

## What is intentionally left out

- UFS driver (`CONFIG_XNU_NATIVE_UFS`) and sheng's PMGR/Adreno/display stack.
  gta4l boots from eMMC and its Adreno 610 is not the A740 the sheng driver
  targets.
- `[userdata]` LBAs are inherited from the sheng template and are **not**
  verified against gta4l's GPT. Read them on the device before an install.
- `dram_size` assumes the 3 GiB SM-T500. For the 4 GiB variant use
  `0xb6000000` (and matching `owned_memory_size`).

## License

BSD-2-Clause-Patent, same as UniXNU-sheng.
