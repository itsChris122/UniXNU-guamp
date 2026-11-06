#!/usr/bin/env python3
"""Model number ADT variant: Settings > General > About shows an iPhone X model number instead of N/A.

  make-guamp-model-number-adt.py --base BASE/kernel.macho.adt OUTDIR [--model MQAC2] [--region QL/A] [--regulatory A1901]

Why (07/11): libMobileGestalt 21H461 answers (answer table, ~/moto/siri-mg/libMobileGestalt.dylib):
  ModelNumber            IODeviceTree:/ "model-number"             (0x1a020fa50)
  RegionInfo             IOPlatformExpertDevice "region-info"      (0x1a0211984; XNU's IOPlatformExpertDevice carries the root ADT properties)
  RegulatoryModelNumber  IODeviceTree:/ "regulatory-model-number"  (0x1a022517c)
  RegionCode             RegionInfo up to "/"
On a real iPhone iBoot fills these three root properties from syscfg (template DeviceTree: syscfg/Mod#/0x20, syscfg/Regn/0x20,
syscfg/RMd#/0x20, 32 bytes, NUL padded). Guamp ADTs had none, so About showed N/A. About shows ModelNumber+RegionInfo
("MQAC2QL/A") and, tapped, RegulatoryModelNumber ("A1901").
Default: iPhone X 64 GB space grey sold in Italy, MQAC2QL/A, regulatory model A1901 (the European iPhone X; note the root
model stays iPhone10,3, whose regulatory models are A1865/A1902: purely cosmetic). Not touched: hw.model (guamp), the
root model, /product, the serial number (GuampBoot fills /serial-number from the ABL), UDID/ECID/IMEI and the other keys.
MobileGestalt caches these answers: the kernel stamp of guamp-mg-adt-sync includes the three properties (same commit), so the
first boot with this ADT resets the MobileGestalt cache once; with an older kernel use guamp-mg-reset=1 once.

Added (refuses a BASE that already has any of them), each 32 bytes NUL padded as iBoot writes them:
  /model-number  /region-info  /regulatory-model-number
Writes OUTDIR/kernel.macho.adt and OUTDIR/configurazione.json; every existing property is checked unchanged."""
import argparse
import importlib.util
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location('guamp_audio_adt', HERE / 'make-guamp-audio-adt.py')
A = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(A)

ROOM = 32


def padded(text):
    raw = text.encode('ascii')
    if not raw or len(raw) >= ROOM:
        sys.exit(f'{text!r}: 1..{ROOM - 1} ASCII characters')
    return raw + bytes(ROOM - len(raw))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', required=True, type=Path)
    ap.add_argument('--model', default='MQAC2')
    ap.add_argument('--region', default='QL/A')
    ap.add_argument('--regulatory', default='A1901')
    ap.add_argument('outdir', type=Path)
    args = ap.parse_args()
    if not re.fullmatch(r'[A-Z0-9]{5}', args.model) or not re.fullmatch(r'[A-Z]{1,3}/A', args.region) \
            or not re.fullmatch(r'A\d{4}', args.regulatory):
        sys.exit('expected --model like MQAC2, --region like QL/A, --regulatory like A1901')
    props = {'model-number': padded(args.model), 'region-info': padded(args.region),
             'regulatory-model-number': padded(args.regulatory)}
    base_bytes = args.base.read_bytes()
    base = A.parse(base_bytes)
    if A.serialize(base) != base_bytes:
        sys.exit('base ADT does not round-trip')
    for name in props:
        if base.get(name):
            sys.exit(f'base already has /{name}')
    for name, value in props.items():
        base.props.append([name, len(value), value])
    result = A.serialize(base)
    before, after = A.flatten(A.parse(base_bytes)), A.flatten(A.parse(result))
    added = sorted(set(after) - set(before))
    if set(before) - set(after) or any(before[k] != after[k] for k in before) or len(added) != len(props):
        sys.exit('an existing property changed')
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / 'kernel.macho.adt').write_bytes(result)
    shown = args.model + args.region
    meta = {'base': str(args.base), 'base_sha256': A.sha(base_bytes), 'adt_sha256': A.sha(result), 'added': added,
            'changes': [f'/model-number "{args.model}", /region-info "{args.region}", /regulatory-model-number '
                        f'"{args.regulatory}" (32 bytes, NUL padded, as iBoot from syscfg): About > Model number {shown}, '
                        f'tapped {args.regulatory}'],
            'boot': 'first boot: guamp-mg-adt-sync resets the MobileGestalt cache once (kernel with model-number in the '
                    'stamp), otherwise add guamp-mg-reset=1 for one boot',
            'tool': 'scripts/make-guamp-model-number-adt.py'}
    (args.outdir / 'configurazione.json').write_text(json.dumps(meta, indent=2) + '\n')
    print(meta['changes'][0])
    print(f"{args.outdir / 'kernel.macho.adt'}: {len(base_bytes)} -> {len(result)} bytes, sha256 {meta['adt_sha256']}")


if __name__ == '__main__':
    main()
