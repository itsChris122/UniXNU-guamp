#!/usr/bin/env python3
"""Cellular ADT variant (guamp-baseband=1|2): adds to a guamp ADT what iPadOS 17.7.11 (21H461) looks for before CommCenter runs its
Qualcomm radio module, as UniXNU/QcomBaseband's Coachz reference does (Trogdor-ipadOS tools/xnu_screen.py baseband_tree, c679400),
with this phone's identity. Reuses the lossless parser of make-guamp-audio-adt.py.

  make-guamp-baseband-adt.py --base BASE/kernel.macho.adt OUTDIR

06/11: the root "compatible" stays the guamp one ("motorola,guamp" first: MobileGestalt's HWModelStr and hw.model). The 04/11
variant put "J121AP" first so that libTelephonyUtil's hardware model table chose the Qualcomm radio module; that also renamed the
phone for thermalmonitord (J121AP-Info.plist, battery "Service"), BlueTool (J121AP.boot.script) and every other per-model lookup.
Giving motorola,guamp a cellular entry is the business of the model table, not of the ADT. A BASE whose compatible starts with
"J121AP" (a 04-05/11 baseband ADT) gets that string removed; the result must not hold any J121 string.

Changed / added (refuses a BASE that already has a different value):
  /chosen/coachz-baseband      u32 1: the bridge's own switch (drivers/QcomBaseband; the kernel also sets it with guamp-baseband=1|2)
  /baseband                    new node, laid out as Apple's Qualcomm basebands have it (D22AP, Mav17: device_type "baseband",
                               compatible "baseband,n41", class 3), describing this modem:
                                 compatible "baseband,n41" then "qcom,sm6115-mpss-pas" (the SM6115 modem subsystem, Linux's name
                                   for it). The Apple string stays first: it is Apple's Qualcomm baseband class string, not a
                                   device; no kext matches either;
                                 class u32 3 (Apple's value on every cellular baseband node: D22AP, D321AP, D331pAP);
                                 guamp,sim-slots u32 2 (two physical nano-SIM slots; informational, nothing reads it);
                                 no euicc-chip-id: the Moto G9 Play has no eUICC (the 04/11 value 2 came from the Trogdor eSIM SKU;
                                   D22AP, an iPhone without eSIM, has none either), so nothing waits for a VINYL card; the bridge's
                                   eSIM layer asks UIM for the slots, finds no eUICC and reports none.
                               No GPIO functions or interrupts: no Apple kext drives it; the bridge publishes the services.
  /product/baseband-chipset    "mav16": the Apple firmware family ABM and BBUpdater serve (the only bbfw of this IPSW is
                               vinyl_02_Mav16; the bridge answers its SAHARA and BSP exchange, the SM6115 keeps its own firmware). It
                               names the firmware folder Baseband/Mav16, not the device.
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

COMPATIBLE_FIRST = b'motorola,guamp\0'
OLD_FIRST = b'J121AP\0'                    # 04-05/11 baseband ADTs
BASEBAND = (('name', b'baseband\0'), ('device_type', b'baseband\0'), ('compatible', b'baseband,n41\0qcom,sm6115-mpss-pas\0'),
            ('class', struct.pack('<I', 3)), ('guamp,sim-slots', struct.pack('<I', 2)))
# What the 04-05/11 tool wrote in /baseband: replaced as a whole when BASE is one of those ADTs.
OLD_BASEBAND = {'name': b'baseband\0', 'device_type': b'baseband\0', 'compatible': b'baseband,n41\0', 'class': struct.pack('<I', 3),
                'euicc-chip-id': struct.pack('<I', 2)}


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
    changes, changed_keys, replaced = [], set(), set()

    compatible = base.get('compatible')
    if not compatible:
        sys.exit('root compatible missing')
    old = compatible[2]
    value = old[len(OLD_FIRST):] if old.startswith(OLD_FIRST) else old
    if not value.startswith(COMPATIBLE_FIRST) or b'J121' in value:
        sys.exit(f'root compatible {old!r}: expected motorola,guamp first and no J121 string')
    if value != old:
        compatible[2] = value
        compatible[1] = (compatible[1] & 0x80000000) | len(value)
        changed_keys.add('/device-tree/compatible')
        changes.append(f'/compatible = {value!r} (was {old!r})')

    def add(node, path, key, value):
        have = node.get(key)
        if have and have[2] != value:
            sys.exit(f'base {path}/{key} already {have[2]!r}')
        if not have:
            node.props.append(prop(key, value))
            changes.append(f'{path}/{key} = {value!r}')

    add(chosen, '/chosen', 'coachz-baseband', struct.pack('<I', 1))
    add(product, '/product', 'baseband-chipset', b'mav16\0')
    at = len(base.children)
    have = base.child('baseband')
    if have:
        if {p[0]: p[2] for p in have.props} != OLD_BASEBAND or have.children:
            sys.exit('base already has a /baseband this tool did not write')
        at = base.children.index(have)
        base.children.remove(have)
        replaced = {f'/device-tree/baseband/{k}' for k in OLD_BASEBAND}
        changes.append('/baseband of the 04/11 tool replaced (euicc-chip-id removed)')
    node = A.Node()
    for key, value in BASEBAND:
        node.props.append(prop(key, value))
    base.children.insert(at, node)
    changes.append('/baseband (device_type baseband, compatible baseband,n41 + qcom,sm6115-mpss-pas, class 3, guamp,sim-slots 2)')

    result = A.serialize(base)
    before, after = A.flatten(A.parse(base_bytes)), A.flatten(A.parse(result))
    kept = changed_keys | replaced
    if set(before) - set(after) - replaced or any(before[k] != after[k] for k in before if k not in kept):
        sys.exit('an existing property changed')
    if b'J121' in result:
        sys.exit('the result still holds a J121 string')
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / 'kernel.macho.adt').write_bytes(result)
    meta = {'base': str(args.base), 'base_sha256': A.sha(base_bytes), 'adt_sha256': A.sha(result),
            'added': sorted(set(after) - set(before)), 'removed': sorted(set(before) - set(after)),
            'changed': sorted(k for k in kept if k in after and before.get(k) != after[k]), 'changes': changes,
            'boot_args': 'guamp-baseband=1|2 guamp-slim-commcenter=1 guamp-wifi-start=1|2 (modem + QRTR), guamp-mg-reset=N once',
            'tool': 'scripts/make-guamp-baseband-adt.py (branch codex/ag-modem-adt-20261106)'}
    (args.outdir / 'configurazione.json').write_text(json.dumps(meta, indent=2) + '\n')
    print('\n'.join(changes))
    print(f'{args.outdir / "kernel.macho.adt"}: {len(base_bytes)} -> {len(result)} bytes, sha256 {meta["adt_sha256"]}')


if __name__ == '__main__':
    main()
