#!/usr/bin/env python3
"""Apple Device Tree (ADT) generator for the Samsung Galaxy Tab A7 10.4 (gta4l, SM6115)."""
import argparse
import pathlib
import struct
import sys

GTA4L_HARDWARE_SPEC = {
    "model": "Samsung Galaxy Tab A7 10.4 (2020)",
    "compatible": ["samsung,gta4l", "qcom,sm6115"],
    "msm_id": [0x1A1, 0x1BC],
    "board_id": 0x1000B,
    "clock_frequency": 19200000,
    "framebuffer": {
        "base": 0x5C000000,
        "size": 2000 * 1200 * 4,
        "width": 1200,
        "height": 2000,
        "stride": 1200 * 4,
        "format": "a8r8g8b8",
        "rotation": 270,
    },
    "reserved_memory": [
        {"name": "pil_adsp_mem",    "base": 0x53800000, "size": 0x01E00000},
        {"name": "pil_ipa_fw_mem",  "base": 0x55600000, "size": 0x00010000},
        {"name": "pil_ipa_gsi_mem", "base": 0x55610000, "size": 0x00005000},
        {"name": "pil_gpu_mem",     "base": 0x55615000, "size": 0x00002000},
        {"name": "framebuffer0",    "base": 0x5C000000, "size": 0x00927C00},
        {"name": "ss_plog_mem",     "base": 0x71100000, "size": 0x00200000},
        {"name": "ramoops",         "base": 0x71300000, "size": 0x00100000},
        {"name": "sec_debug_mem",   "base": 0x71400000, "size": 0x00800000},
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
        pack_adt_prop("display-base", u64(GTA4L_HARDWARE_SPEC["framebuffer"]["base"])),
        pack_adt_prop("display-width", u32(GTA4L_HARDWARE_SPEC["framebuffer"]["width"])),
        pack_adt_prop("display-height", u32(GTA4L_HARDWARE_SPEC["framebuffer"]["height"])),
        pack_adt_prop("display-stride", u32(GTA4L_HARDWARE_SPEC["framebuffer"]["stride"])),
        pack_adt_prop("display-rotation", u32(GTA4L_HARDWARE_SPEC["framebuffer"]["rotation"])),
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
    root_children = [cpus_node, chosen_node]
    return (
        struct.pack("<II", len(root_props), len(root_children))
        + b"".join(root_props)
        + b"".join(root_children)
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Emit gta4l ADT for UniXNU")
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
    print(f"[make-gta4l-adt] Wrote {len(blob)} bytes to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
