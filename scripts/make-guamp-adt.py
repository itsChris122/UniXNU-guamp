#!/usr/bin/env python3
"""Apple Device Tree (ADT) generator for the Motorola Moto G9 Play (guamp, SM6115)."""
import argparse
import pathlib
import struct
import sys

GUAMP_HARDWARE_SPEC = {
    # Values read from the device (dtbo_b / boot_b DTBs, fastboot getvar) on 2026-10-22.
    "model": "Motorola Moto G9 Play",
    "compatible": ["qcom,bengal-guamp", "qcom,sm6115"],
    "msm_id": [0x1A1, 0x1BC],
    "board_id": 0xB000,
    "clock_frequency": 19200000,
    "framebuffer": {
        # cont_splash_region, Himax HX83102D 720x1600 video mode panel, native portrait
        "base": 0x5C000000,
        "size": 720 * 1600 * 4,
        "width": 720,
        "height": 1600,
        "stride": 720 * 4,
        "format": "a8r8g8b8",
        "rotation": 0,
    },
    "reserved_memory": [
        {"name": "pil_adsp_mem",    "base": 0x52200000, "size": 0x01C00000},
        {"name": "pil_ipa_fw_mem",  "base": 0x53E00000, "size": 0x00010000},
        {"name": "pil_ipa_gsi_mem", "base": 0x53E10000, "size": 0x00005000},
        {"name": "pil_gpu_mem",     "base": 0x53E15000, "size": 0x00002000},
        {"name": "framebuffer0",    "base": 0x5C000000, "size": 0x00465000},
        # RAM log of guampPkg (RamLog.h): kept so later stages can append to it
        {"name": "guamp_ram_log",   "base": 0x13FE00000, "size": 0x00200000},  # live + previous boot log (RamLog.h)
        {"name": "removed_mem",     "base": 0x60000000, "size": 0x03900000},
    ],
}


def pack_adt_prop(name: str, value: bytes) -> bytes:
    name_bytes = name.encode("ascii")[:31].ljust(32, b"\x00")
    length = len(value)
    padded_len = (length + 3) & ~3
    return name_bytes + struct.pack("<I", length) + value.ljust(padded_len, b"\x00")


def u32(val: int) -> bytes:
    return struct.pack("<I", val)


def u64(val: int) -> bytes:
    return struct.pack("<Q", val)


def cstr(val: str) -> bytes:
    return val.encode("utf-8") + b"\x00"


def build_cpu_node(cpu_idx: int, mpidr: int) -> bytes:
    cluster_type = b"ECore\x00" if mpidr < 0x100 else b"PCore\x00"
    props = [
        pack_adt_prop("name", cstr(f"cpu{cpu_idx}")),
        pack_adt_prop("device_type", cstr("cpu")),
        pack_adt_prop("reg", u32(mpidr)),
        pack_adt_prop("cpu-id", u32(cpu_idx)),
        pack_adt_prop("cluster-type", cluster_type),
        pack_adt_prop("timebase-frequency", u32(19200000)),
        pack_adt_prop("fixed-frequency-khz", u32(1804800 if mpidr < 0x100 else 2016000)),
        # pe_identify_machine() reads the clocks only from the CPU whose "state" is "running" (the boot CPU)
        pack_adt_prop("state", cstr("running" if cpu_idx == 0 else "waiting")),
        # bus / timebase must be a whole ratio >= 1 (bus_to_dec_rate_den); 192 MHz = 10 x 19.2 MHz
        pack_adt_prop("bus-frequency", u32(192000000)),
        pack_adt_prop("memory-frequency", u32(1804000000)),
        pack_adt_prop("peripheral-frequency", u32(19200000)),
        pack_adt_prop("fixed-frequency", u32(19200000)),
        pack_adt_prop("clock-frequency", u32(1804800000 if mpidr < 0x100 else 2016000000)),
    ]
    return struct.pack("<II", len(props), 0) + b"".join(props)


def generate_adt(args: argparse.Namespace) -> bytes:
    cpu_regs = [int(x.strip(), 0) for x in args.cpu_regs.split(",") if x.strip()]
    cpu_children = [build_cpu_node(i, reg) for i, reg in enumerate(cpu_regs)]

    cpus_props = [
        pack_adt_prop("name", cstr("cpus")),
        pack_adt_prop("#address-cells", u32(1)),
        pack_adt_prop("#size-cells", u32(0)),
    ]
    cpus_node = (
        struct.pack("<II", len(cpus_props), len(cpu_children))
        + b"".join(cpus_props)
        + b"".join(cpu_children)
    )

    chosen_props = [
        pack_adt_prop("name", cstr("chosen")),
        pack_adt_prop("root-matching", cstr(args.root_match_volume_uuid)),
        pack_adt_prop("dram-base", u64(0x40000000)),
        pack_adt_prop("dram-size", u64(int(args.dram_size or args.owned_memory_size, 0))),
        pack_adt_prop("display-base", u64(GUAMP_HARDWARE_SPEC["framebuffer"]["base"])),
        pack_adt_prop("display-width", u32(GUAMP_HARDWARE_SPEC["framebuffer"]["width"])),
        pack_adt_prop("display-height", u32(GUAMP_HARDWARE_SPEC["framebuffer"]["height"])),
        pack_adt_prop("display-stride", u32(GUAMP_HARDWARE_SPEC["framebuffer"]["stride"])),
        pack_adt_prop("display-rotation", u32(GUAMP_HARDWARE_SPEC["framebuffer"]["rotation"])),
    ]
    if args.unique_chip_id:
        chosen_props.append(pack_adt_prop("unique-chip-id", u64(int(args.unique_chip_id, 0))))
    chosen_node = struct.pack("<II", len(chosen_props), 0) + b"".join(chosen_props)

    root_props = [
        pack_adt_prop("name", cstr("device-tree")),
        pack_adt_prop("model", cstr("iPad7,1")),
        pack_adt_prop("compatible", cstr("J120AP")),
        pack_adt_prop("board-id", u32(12)),
        pack_adt_prop("#address-cells", u32(2)),
        pack_adt_prop("#size-cells", u32(2)),
    ]
    # /arm-io: pe_identify_machine() returns before setting any clock (timebase 0 -> UBSan trap in
    # wfe_timeout_configure) when pe_arm_get_soc_base_phys() finds no "ranges" here. Child 0 maps to the
    # SM6115 peripheral space from 1 MiB up to 0x10000000 (TCSR 0x3C0000 ... APSS 0xF500000).
    arm_io_props = [
        pack_adt_prop("name", cstr("arm-io")),
        pack_adt_prop("device_type", cstr("bengal-io")),
        pack_adt_prop("compatible", cstr("arm-io,qcom-sm6115")),
        pack_adt_prop("ranges", u64(0x0) + u64(0x00100000) + u64(0x0FF00000)),
        pack_adt_prop("#address-cells", u32(2)),
        pack_adt_prop("#size-cells", u32(2)),
    ]
    arm_io_node = struct.pack("<II", len(arm_io_props), 0) + b"".join(arm_io_props)
    root_children = [cpus_node, chosen_node, arm_io_node]
    return (
        struct.pack("<II", len(root_props), len(root_children))
        + b"".join(root_props)
        + b"".join(root_children)
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Emit guamp ADT for UniXNU")
    parser.add_argument("--output", "-o", required=True)
    parser.add_argument("--dram-size", default=None)
    parser.add_argument("--owned-memory-size", default="0x40000000")
    parser.add_argument("--root-match-volume-uuid", default="04769D26-2CA0-4825-B443-E2E5F88300F2")
    parser.add_argument("--cpu-regs", default="0x0,0x1,0x2,0x3,0x100,0x101,0x102,0x103")
    parser.add_argument("--unique-chip-id", default="0x1122334455667788")
    # Accepted for the payload step of board.toml; this generator does not model them.
    parser.add_argument("--chip-id", default="0x1bc")
    parser.add_argument("--packed-data-ramdisk", action="store_true")
    parser.add_argument("--trust-cache", action="store_true")
    parser.add_argument("--ipad-identity", action="store_true")
    args, _unknown = parser.parse_known_args()

    out = pathlib.Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    blob = generate_adt(args)
    out.write_bytes(blob)
    print(f"[make-guamp-adt] Wrote {len(blob)} bytes to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
