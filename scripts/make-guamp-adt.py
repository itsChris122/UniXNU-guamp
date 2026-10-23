#!/usr/bin/env python3
"""Apple-format DT (ADT) for the Motorola Moto G9 Play (guamp, Qualcomm SM6115).

Structure of make-sheng-adt.py (UniXNU-sheng, the port that reaches userland) with the guamp values:

  - DRAM base 0x40000000 (4 GiB LPDDR4x, fastboot getvar ram)
  - GICv3: distributor 0x0F200000/0x10000, redistributors 0x0F300000/0x100000, stride 0x20000
    (sm6115.dtsi), as children of an arm-io window at 0x0F000000
  - timebase 19.2 MHz; PSCI 1.0 over smc (sm6115.dtsi)
  - Kryo 260 Silver cores only (MPIDR 0x0..0x3): the kernel uses the BCM2837 configuration, one SMP
    cluster of at most four CPUs, so there is no cluster-type and no Gold cluster yet

Each value below fixed a stop of the first boot (found with the UniXNU entry probe, 2026-10-23):
/arm-io + CPU clocks (pe_identify_machine), "running" boot CPU and integer properties
(ml_parse_cpu_topology), /chosen/memory-map (arm_vm_init), random-seed (early_random),
/defaults (pmap_bootstrap), /options and the NVRAM proxy (platform expert configure()).
"""
from pathlib import Path
import argparse
import os
import struct
import zlib

DRAM_BASE = 0x40000000
ARMIO_BASE = 0x0F000000          # APSS block with the GIC
ARMIO_SIZE = 0x01000000
GIC_DIST = 0x0F200000
GIC_DIST_SIZE = 0x10000
GIC_REDIST = 0x0F300000
GIC_REDIST_SIZE = 0x100000
GIC_REDIST_STRIDE = 0x20000
TIMEBASE = 19200000
# XNU panic region: guampPkg RamLog.h PANIC_REGION_BASE/SIZE (firmware "RAM Log" map entry 0x100000000)
PANIC_BASE = 0x100100000
PANIC_SIZE = 0x100000


def node(name, properties=(), children=()):
    properties = [('name', name.encode() + b'\0'), *properties]
    result = struct.pack('<II', len(properties), len(children))
    for key, data in properties:
        result += struct.pack('<32sI', key.encode(), len(data))
        result += data + bytes(-len(data) % 4)
    return result + b''.join(children)


def u32(value):
    return struct.pack('<I', value)


def u64(value):
    return struct.pack('<Q', value)


def empty_nvram_proxy():
    """A fresh volatile CHRP bank for upstream IODTNVRAM, with no controller (from make-sheng-adt.py)."""
    def header(name, size):
        raw = bytearray(struct.pack('<BBH12s', 0x70, 0, size // 16, name))
        checksum = sum(raw)
        while checksum > 255:
            checksum = (checksum & 255) + (checksum >> 8)
        raw[1] = checksum
        return raw

    image = bytearray(8192)
    image[:16] = header(b'nvram', 32)
    image[32:48] = header(b'common', 16 + 2048)
    free_at = 32 + 16 + 2048
    image[free_at:free_at + 16] = header(b'free', len(image) - free_at)
    struct.pack_into('<I', image, 16, zlib.adler32(image[20:]))
    return [('nvram-bank-size', u32(len(image))),
            ('nvram-bank-count', u32(1)),
            ('nvram-current-bank', u32(0)),
            ('nvram-proxy-data', bytes(image))]


def cpu_node(index, reg):
    return node('cpu%d' % index, [
        # pe_identify_machine() and ml_is_boot_cpu() look for the "running" boot CPU
        ('state', b'running\0' if index == 0 else b'waiting\0'),
        ('reg', u32(reg)),
        ('device_type', b'cpu\0'),
        ('compatible', b'qcom,kryo260\0arm,cortex-a53\0'),
        ('cpu-id', u32(index)),
        ('timebase-frequency', u32(TIMEBASE)),
        # bus / timebase must be a whole ratio >= 1 (bus_to_dec_rate_den): 192 MHz = 10 x 19.2 MHz
        ('bus-frequency', u32(192000000)),
        ('memory-frequency', u32(1804000000)),
        ('peripheral-frequency', u32(TIMEBASE)),
        ('fixed-frequency', u32(TIMEBASE)),
        ('clock-frequency', u32(1804800000)),
    ])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', '-o', type=Path, required=True)
    parser.add_argument('--dram-size', type=lambda v: int(v, 0), required=True)
    parser.add_argument('--owned-memory-size', type=lambda v: int(v, 0), default=None,
                        help='DarwinBoot window: 256 MiB..8 GiB in 2 MiB steps, inside one free run of the firmware map')
    parser.add_argument('--root-match-volume-uuid', default=None)
    parser.add_argument('--unique-chip-id', type=lambda v: int(v, 16), default=None)
    parser.add_argument('--chip-id', type=lambda v: int(v, 16), default=None)
    parser.add_argument('--cpu-regs', default='0x0,0x1,0x2,0x3')
    parser.add_argument('--psci-method', choices=['smc', 'hvc'], default='smc')
    parser.add_argument('--ipad-identity', action='store_true')
    parser.add_argument('--data-ramdisk', action='store_true')
    parser.add_argument('--packed-data-ramdisk', action='store_true')
    parser.add_argument('--persistent-data', action='store_true')
    parser.add_argument('--trust-cache', action='store_true')
    parser.add_argument('--gpu-addon', action='store_true')
    parser.add_argument('--text-fixup', action='append', default=[])
    args = parser.parse_args()

    if (args.unique_chip_id is None) != (args.chip_id is None):
        parser.error('--unique-chip-id and --chip-id must be supplied together')
    if args.owned_memory_size is not None and (not 0x10000000 <= args.owned_memory_size <= 0x200000000
                                               or args.owned_memory_size % 0x200000):
        parser.error('owned memory must be 256 MiB to 8 GiB, aligned to 2 MiB')
    cpu_regs = [int(v, 0) for v in args.cpu_regs.split(',')]
    if not cpu_regs or cpu_regs[0] != 0 or len(cpu_regs) > 4 or len(set(cpu_regs)) != len(cpu_regs):
        parser.error('--cpu-regs: up to four distinct Silver cores (BCM2837 kernel configuration), boot CPU 0x0 first')

    cpus = [cpu_node(index, reg) for index, reg in enumerate(cpu_regs)]
    ipad = args.ipad_identity

    tree = node('device-tree', [
        ('model', b'iPad7,1\0' if ipad else b'guamp-first-boot\0'),
        *([('target-type', b'J120\0')] if ipad else []),
        ('compatible', b'motorola,guamp\0qcom,bengal\0qcom,sm6115\0'),
        ('#address-cells', u32(2)),
        ('#size-cells', u32(2)),
    ], [
        node('chosen', [
            # bootseed_init() panics with fewer than 64 bytes (fixed per payload build)
            ('random-seed', os.urandom(64)),
            *([('unique-chip-id', u64(args.unique_chip_id)), ('chip-id', u32(args.chip_id))]
              if args.unique_chip_id is not None else []),
            ('osenvironment', b'main\0'),
            *([('certificate-production-status', u32(0)), ('effective-production-status-ap', u32(0)),
               ('certificate-security-mode', u32(1)), ('effective-security-mode-sep', u32(1))] if ipad else []),
            *([('ephemeral-storage', u32(1))]
              if (args.data_ramdisk or args.packed_data_ramdisk) and not args.persistent_data else []),
            *([('darwinboot-memory-size', u64(args.owned_memory_size))] if args.owned_memory_size else []),
            ('dram-base', u64(DRAM_BASE)),
            ('dram-size', u64(args.dram_size)),
            ('crypto-hash-method', b'sha2-384\0'),
            *([('root-matching', ('<dict><key>IOProviderClass</key><string>IOMedia</string>'
                                  '<key>IOPropertyMatch</key><dict><key>UUID</key><string>' +
                                  args.root_match_volume_uuid.upper() + '</string></dict></dict>\0').encode())]
              if args.root_match_volume_uuid else []),
            ('amfi-only-platform-code', u32(1)),
            *([('text-fixups', b''.join(f.encode() + b'\0' for f in args.text_fixup))] if args.text_fixup else []),
            *empty_nvram_proxy(),
        ], [
            # arm_vm_init() asserts /chosen/memory-map exists; DarwinBoot fills the placeholders
            node('memory-map', ([('PackedRAMDisk' if args.packed_data_ramdisk else 'RAMDisk', bytes(16))]
                                if (args.data_ramdisk or args.packed_data_ramdisk) else []) +
                               ([('TrustCache', bytes(16))] if args.trust_cache else []) +
                               ([('GpuAddon', bytes(16))] if args.gpu_addon else [])),
            node('manifest-properties'),
        ]),
        node('cpus', children=cpus),
        *([node('psci', [('compatible', b'arm,psci-1.0\0'), ('method', args.psci_method.encode() + b'\0')])]
          if len(cpus) > 1 else []),
        # pmap_bootstrap() asserts /defaults; no suspend on this board
        node('defaults', [('sleep-disabled', u32(1)), ('kern.maxvnodes', u32(66560))]),
        node('options'),
        # No /pram: with it the kernel stopped before load_init_program (2026-10-23); to be investigated
        *([node('buttons', [('compatible', b'buttons\0'), ('device_type', b'buttons\0'),
                            ('button-names', b'hold\0voldown\0volup\0'),
                            ('home-button-type', u32(2))])] if ipad else []),
        node('product', [
            ('product-name', b'iPad Pro (12.9-inch) (2nd generation)\0' if ipad else b'Moto G9 Play\0'),
            ('model', b'motorola,guamp\0'),
            *([('product-description', b'iPad Pro (12.9-inch) (2nd generation)\0'),
               ('artwork-device-idiom', b'pad\0'),
               ('artwork-device-subtype', u32(2732)),
               ('artwork-scale-factor', u32(2)),
               ('artwork-display-gamut', b'P3\0'),
               ('compatible-device-fallback', b'iPad6,7\0'),
               ('thin-bezel', u32(1))] if ipad else []),
        ]),
        # pe_arm_get_soc_base_phys() reads ranges[1]; pe_identify_machine() returns early without it
        node('arm-io', [
            ('device_type', b'arm-io\0'),
            ('#address-cells', u32(2)),
            ('#size-cells', u32(2)),
            ('ranges', struct.pack('<QQQ', 0, ARMIO_BASE, ARMIO_SIZE)),
        ], [
            node('intc', [
                ('compatible', b'arm,gic-v3\0'),
                ('reg', struct.pack('<4Q', GIC_DIST - ARMIO_BASE, GIC_DIST_SIZE,
                                    GIC_REDIST - ARMIO_BASE, GIC_REDIST_SIZE)),
                ('redistributor-stride', u64(GIC_REDIST_STRIDE)),
            ]),
        ]),
    ])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(tree)
    print(f'[make-guamp-adt] {len(tree)} bytes, dram {args.dram_size:#x}, {len(cpus)} CPUs, timebase {TIMEBASE}')


if __name__ == '__main__':
    main()
