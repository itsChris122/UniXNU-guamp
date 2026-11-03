#!/usr/bin/env python3
"""Cellular ADT variant (guamp-baseband=1, 04/11): adds to a guamp ADT what iPadOS 17.7.11 (21H461) looks for before CommCenter runs
its Qualcomm radio module, as UniXNU/QcomBaseband's Coachz reference does (Trogdor-ipadOS tools/xnu_screen.py baseband_tree,
c679400). Reuses the lossless parser of make-guamp-audio-adt.py.

  make-guamp-baseband-adt.py --base BASE/kernel.macho.adt OUTDIR

Why: MobileGestalt's HWModelStr is the first string of the root "compatible"; libTelephonyUtil's table of hardware models has no
J120 (Wi-Fi iPad) and no "motorola,guamp", so CommCenter runs NoBBRadioModule. J121AP (iPad7,4, cellular, Qualcomm Mav16) is the
cellular model of this IPSW, whose Mav16 firmware ABM boots through the bridge.

Changed / added (refuses a BASE that already has a different value):
  /compatible                  "J121AP" put FIRST, the existing strings kept after it (ArmDTPlatform matches the root by name,
                               not by compatible; the Motorola/Qualcomm strings stay for everything that looks for them)
  /chosen/coachz-baseband      u32 1: the bridge's own switch (the kernel also sets it with guamp-baseband=1)
  /baseband                    new node: name, device_type "baseband", compatible "baseband,n41", class u32 3, euicc-chip-id u32 2
                               (no GPIO functions or interrupts: no Apple kext matches it; the bridge publishes the services)
  /product/baseband-chipset    "mav16"
Model (iPhone10,3 or iPad7,1), target-type, artwork and every other property stay as they are. Writes OUTDIR/kernel.macho.adt and
OUTDIR/configurazione.json. After the first boot with it MobileGestalt must recompute its answers: guamp-mg-reset=N once."""
import argparse
import importlib.util
import json
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location('guamp_audio_adt', HERE / 'make-guamp-audio-adt.py')
A = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(A)

COMPATIBLE_FIRST = b'J121AP\0'


def prop(key, value):
    return [key, len(value), value]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', required=True, type=Path)
    ap.add_argument('outdir', type=Path)
    args = ap.parse_args()
    base_bytes = args.base.read_bytes()
    base = A.parse(base_bytes)
    if A.serialize(base) != base_bytes:
        sys.exit('base ADT does not round-trip')
    chosen, product = base.child('chosen'), base.child('product')
    if not chosen or not product:
        sys.exit('/chosen or /product missing')
    changes, changed_keys = [], set()

    compatible = base.get('compatible')
    if not compatible:
        sys.exit('root compatible missing')
    if not compatible[2].startswith(COMPATIBLE_FIRST):
        old = compatible[2]
        compatible[2] = COMPATIBLE_FIRST + old
        compatible[1] = (compatible[1] & 0x80000000) | len(compatible[2])
        changed_keys.add('/device-tree/compatible')
        changes.append(f'/compatible = {compatible[2]!r} (was {old!r})')

    def add(node, path, key, value):
        have = node.get(key)
        if have and have[2] != value:
            sys.exit(f'base {path}/{key} already {have[2]!r}')
        if not have:
            node.props.append(prop(key, value))
            changes.append(f'{path}/{key} = {value!r}')

    add(chosen, '/chosen', 'coachz-baseband', struct.pack('<I', 1))
    add(product, '/product', 'baseband-chipset', b'mav16\0')
    if base.child('baseband'):
        sys.exit('base already has /baseband')
    node = A.Node()
    for key, value in (('name', b'baseband\0'), ('device_type', b'baseband\0'), ('compatible', b'baseband,n41\0'),
                       ('class', struct.pack('<I', 3)), ('euicc-chip-id', struct.pack('<I', 2))):
        node.props.append(prop(key, value))
    base.children.append(node)
    changes.append('/baseband (device_type baseband, compatible baseband,n41, class 3, euicc-chip-id 2)')

    result = A.serialize(base)
    before, after = A.flatten(A.parse(base_bytes)), A.flatten(A.parse(result))
    if set(before) - set(after) or any(before[k] != after[k] for k in before if k not in changed_keys):
        sys.exit('an existing property changed')
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / 'kernel.macho.adt').write_bytes(result)
    meta = {'base': str(args.base), 'base_sha256': A.sha(base_bytes), 'adt_sha256': A.sha(result),
            'added': sorted(set(after) - set(before)), 'changed': sorted(changed_keys), 'changes': changes,
            'boot_args': 'guamp-baseband=1 guamp-slim-commcenter=1 guamp-wifi-start=1|2 (modem + QRTR), guamp-mg-reset=N once',
            'tool': 'scripts/make-guamp-baseband-adt.py (branch codex/ag-sim-20261104)'}
    (args.outdir / 'configurazione.json').write_text(json.dumps(meta, indent=2) + '\n')
    print('\n'.join(changes))
    print(f'{args.outdir / "kernel.macho.adt"}: {len(base_bytes)} -> {len(result)} bytes, sha256 {meta["adt_sha256"]}')


if __name__ == '__main__':
    main()
