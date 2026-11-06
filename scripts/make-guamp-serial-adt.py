#!/usr/bin/env python3
"""Serial number ADT variant: adds the root placeholder GuampBoot fills with the phone's real serial number at boot.

  make-guamp-serial-adt.py --base BASE/kernel.macho.adt OUTDIR

Why (06/11): XNU publishes the root ADT property "serial-number" (a C string) as IOPlatformSerialNumber
(IOPlatformExpertDevice::configureDefaults, IOStartIOKit), libMobileGestalt reports it as SerialNumber and Settings >
General > About shows it. Guamp ADTs had none, so About had no serial number. The value is not written here: the
Motorola ABL hands androidboot.serialno=<serial> (the one of "fastboot getvar serialno") on the command line of every boot
image, and GuampBoot (loader/guampboot/payload/identity.c, branch codex/guampboot-20261104) renames the placeholder to
"serial-number" and fills it in place when that value is valid. Nothing is read from or written to the eMMC, and the
serial number never lands in a file on the PC.

Added (refuses a BASE that already has either property):
  /guamp-serial-slot   32 zero bytes (a serial of up to 31 characters, NUL padded)
Without GuampBoot (DarwinBoot/UEFI) the placeholder stays as it is and iOS behaves as before.
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

SLOT, SERIAL, ROOM = 'guamp-serial-slot', 'serial-number', 32


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', required=True, type=Path)
    ap.add_argument('outdir', type=Path)
    args = ap.parse_args()
    base_bytes = args.base.read_bytes()
    base = A.parse(base_bytes)
    if A.serialize(base) != base_bytes:
        sys.exit('base ADT does not round-trip')
    for key in (SLOT, SERIAL):
        if base.get(key):
            sys.exit(f'base already has /{key}')
    base.props.append([SLOT, ROOM, bytes(ROOM)])
    result = A.serialize(base)
    before, after = A.flatten(A.parse(base_bytes)), A.flatten(A.parse(result))
    added = sorted(set(after) - set(before))
    if set(before) - set(after) or any(before[k] != after[k] for k in before):
        sys.exit('an existing property changed')
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / 'kernel.macho.adt').write_bytes(result)
    meta = {'base': str(args.base), 'base_sha256': A.sha(base_bytes), 'adt_sha256': A.sha(result), 'added': added,
            'changes': [f'/{SLOT} = {ROOM} zero bytes (GuampBoot -> /{SERIAL} from androidboot.serialno)'],
            'boot': 'GuampBoot with payload/identity.c (29bf404 or later); the loader log shows '
                    '"identity: serial number ..XXXX from the ABL"',
            'tool': 'scripts/make-guamp-serial-adt.py'}
    (args.outdir / 'configurazione.json').write_text(json.dumps(meta, indent=2) + '\n')
    print(meta['changes'][0])
    print(f'{args.outdir / "kernel.macho.adt"}: {len(base_bytes)} -> {len(result)} bytes, sha256 {meta["adt_sha256"]}')


if __name__ == '__main__':
    main()
