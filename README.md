# UniXNU-guamp

UniXNU device repository for the **Motorola Moto G9 Play** (`guamp`, Qualcomm SM6115, 4 GB LPDDR4x, eMMC,
Himax HX83102D 720x1600 panel).

Derived from [UniXNU-gta4l](https://github.com/NoNameBuilder/UniXNU-gta4l) (Samsung Galaxy Tab A7 10.4, same SoC).
Independent community work; not maintained by the UniXNU organization.

## State

| Layer | State |
| --- | --- |
| UEFI firmware | Mu-Silicium `guampPkg` boots with `fastboot boot` and shows the UniXNU menu |
| Board description | `board.toml` with values read from the device; `fetch` and `tree` run |
| ADT | `scripts/make-guamp-adt.py` |
| Kernel | not built yet (macOS runner) |
| Hardware boot of XNU | not attempted |

The display of the development device is broken: firmware logs and screenshots are written to the `logfs`
partition and read back from recovery.

No Apple files are stored in this repository; the IPSW is read from Apple's servers by the build.
The platform seed (`unique_chip_id`) is never stored here.
