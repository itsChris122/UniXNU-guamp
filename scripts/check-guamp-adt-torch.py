#!/usr/bin/env python3
"""Checks that a guamp ADT still carries what the torch needs (verified 27/10 with Codex/cpu-config-8-torcia, REGISTRO 27/10 ~20:05):
the /product/camera "flash" property and the 10 text fixups in /chosen/text-fixups:
  dyld_shared_cache_arm64.37,d4ac30          -[SBUIFlashlightController isAvailable] -> YES
  dyld_shared_cache_arm64.11,36e9bc8..36e9c48 -[AVFlashlight setFlashlightLevel:withError:] -> GuampTorchService (9 entries)
The ADTs are derived one from the other (prova5-logo -> audio -> finale -> siri ...): run this on every new ADT so the torch
fixups are not lost. Exit 1 when something is missing.
  check-guamp-adt-torch.py ADT [ADT...]"""
import re
import sys

FIXUPS = ['dyld_shared_cache_arm64.37,d4ac30,'] + \
    ['dyld_shared_cache_arm64.11,%x,' % (0x36e9bc8 + 16 * i) for i in range(9)]


def main():
    bad = 0
    for path in sys.argv[1:]:
        blob = open(path, 'rb').read()
        missing = [f for f in FIXUPS if f.encode() not in blob]
        # /product/camera carries a property keyed "flash" (32-byte zero-padded key) right after its name "camera"
        flash = re.search(rb'camera\0\0flash\0{27}', blob) is not None
        print('%s: torch fixups %d/%d%s, /product/camera/flash %s' % (path, len(FIXUPS) - len(missing), len(FIXUPS),
              (' missing ' + ' '.join(missing)) if missing else '', 'present' if flash else 'MISSING'))
        bad += bool(missing) or not flash
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
