#!/usr/bin/env python3
"""Siri / Touch ID ADT variant: adds to a guamp ADT what libMobileGestalt (21H461) needs to report the "assistant" and
"touch-id" capabilities, and nothing else. Reuses the lossless parser of make-guamp-audio-adt.py.

  make-guamp-siri-adt.py --base BASE/kernel.macho.adt --j120 DeviceTree.j120ap.im4p [--touch-id] [--lzfse PATH] OUTDIR

Why (static analysis of 21H461, 29/10):
  * Settings (PreferencesUI.framework/Settings.plist) shows "Siri" only with requiredCapabilities ["assistant"].
    MG "assistant" (answer function 0x1a021243c, not cached: table index 0xffff) is true iff IODeviceTree:/product has a
    property named "assistant". The J120 tree has /product/assistant = 1; guamp ADTs did not. SpringBoard's Siri button
    uses the same answer.
  * Preferences.app labels the row "Touch ID & Passcode" from MobileGestalt_get_touchIDCapability = MG "touch-id"
    (0x1a0229b30, cached at index 584) = MG HV7WDiidgMf7lwAu++Lk5w (0x1a022669c) = a node named "mesa" found under
    IODeviceTree:/arm-io (copyDeviceTreeProperty:withFirstChar:'m':propertyName:"name"). J120 has /arm-io/spi2/mesa.

Added (refuses a BASE that already has a different value):
  /product/assistant           copied byte for byte from J120 (u32 1)
  --touch-id: /arm-io/mesa      new node with only name="mesa" and device_type="mesa". No compatible, reg, interrupts or
                                function-* properties: AppleMesa/AppleMesaSEPDriver personalities match IONameMatch
                                "biosensor,mesa" only, so no Apple kext attaches. launchd's com.apple.biometrickitd.mesa
                                (_LimitLoadToDeviceTree "mesa") is kept disabled by the kernel (guamp-biokit=1 enables it).
Writes OUTDIR/kernel.macho.adt and OUTDIR/configurazione.json; every existing property is checked unchanged."""
import argparse
import importlib.util
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location('guamp_audio_adt', HERE / 'make-guamp-audio-adt.py')
A = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(A)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', required=True, type=Path)
    ap.add_argument('--j120', required=True, type=Path)
    ap.add_argument('--touch-id', action='store_true', help='also add /arm-io/mesa (MG touch-id)')
    ap.add_argument('--lzfse', default=str(Path.home() / 'moto/tools/lzfse/build/bin/lzfse'))
    ap.add_argument('outdir', type=Path)
    args = ap.parse_args()
    base_bytes = args.base.read_bytes()
    base, j120 = A.parse(base_bytes), A.j120_tree(args.j120, args.lzfse)
    if A.serialize(base) != base_bytes:
        sys.exit('base ADT does not round-trip')
    product, src_product = base.child('product'), j120.child('product')
    if not product or not src_product:
        sys.exit('/product missing')
    changes = []
    want = src_product.get('assistant')
    if not want:
        sys.exit('J120 /product/assistant missing')
    have = product.get('assistant')
    if have and have[2] != want[2]:
        sys.exit(f'base /product/assistant already {have[2].hex()}')
    if not have:
        product.props.append(list(want))
        changes.append(f'/product/assistant = {want[2].hex()}')
    if args.touch_id:
        armio = base.child('arm-io')
        if not armio:
            sys.exit('base /arm-io missing')
        if armio.child('mesa'):
            sys.exit('base already has /arm-io/mesa')
        mesa = A.Node()
        mesa.props.append(['name', len(b'mesa\0'), b'mesa\0'])
        mesa.props.append(['device_type', len(b'mesa\0'), b'mesa\0'])
        armio.children.append(mesa)
        changes.append('/arm-io/mesa (name, device_type = "mesa")')
    result = A.serialize(base)
    before, after = A.flatten(A.parse(base_bytes)), A.flatten(A.parse(result))
    added = sorted(set(after) - set(before))
    if set(before) - set(after) or any(before[k] != after[k] for k in before):
        sys.exit('an existing property changed')
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / 'kernel.macho.adt').write_bytes(result)
    meta = {'base': str(args.base), 'base_sha256': A.sha(base_bytes), 'j120_source': str(args.j120),
            'j120_sha256': A.sha(args.j120.read_bytes()), 'adt_sha256': A.sha(result), 'added': added,
            'changes': changes, 'touch_id': args.touch_id,
            'boot_args': 'guamp-mg-reset=1 once (or a kernel whose guamp-mg-adt-sync stamps /product/assistant and '
                         '/arm-io/mesa); guamp-biokit=1 only to let launchd start biometrickitd',
            'tool': 'scripts/make-guamp-siri-adt.py (worktree wt-siri, branch codex/siri-20261029)'}
    (args.outdir / 'configurazione.json').write_text(json.dumps(meta, indent=2) + '\n')
    print('\n'.join(changes))
    print(f'{args.outdir / "kernel.macho.adt"}: {len(base_bytes)} -> {len(result)} bytes, {len(added)} keys added, '
          f'sha256 {meta["adt_sha256"]}')


if __name__ == '__main__':
    main()
