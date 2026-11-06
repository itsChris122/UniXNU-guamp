#!/usr/bin/env python3
"""Apple Watch ADT variant: adds /product/watch-companion = 1, as in the iPhone X ADT (DeviceTree.d22ap).

  make-guamp-watch-adt.py --base BASE/kernel.macho.adt OUTDIR

Why (07/11, fin53): the Watch app (Bridge) never shows the welcome / pairing screen. COSGetStartedViewController waits
for "NR is ready", and -[NRPairedDeviceRegistry status] (NanoRegistry.framework 21H461, same code as the XS 21H16)
answers 0 (Invalid) without even asking nanoregistryd when -[NRPairedDeviceRegistry supportsWatch] is NO.
+[NRRegistry _supportsWatch] is IsUIBuild+demo mode, or DeviceClass == "Watch", or MGGetBoolAnswer("watch-companion");
nanoregistryd (XS) has the same MGGetBoolAnswer("watch-companion") gate. libMobileGestalt answers watch-companion
with the boolean property "watch-companion" of IODeviceTree:/product (descriptor fucd7llSuoCNHrrvLS1QQg, 0x1a021a1c4):
every iPhone ADT has it (d22ap: 01 00 00 00), iPad ADTs and so far the guamp ADTs do not, so the answer was false.
MobileGestalt may cache it: the kernel stamp of guamp-mg-adt-sync includes /product watch-companion (same commit),
so the first boot with this ADT resets the MobileGestalt cache once; with an older kernel use guamp-mg-reset=1 once.

Added (refuses a BASE that already has it):
  /product/watch-companion   u32 1
Writes OUTDIR/kernel.macho.adt and OUTDIR/configurazione.json; every existing property is checked unchanged."""
import argparse
import importlib.util
import json
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("guamp_audio_adt", HERE / "make-guamp-audio-adt.py")
A = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(A)

PROP = "watch-companion"
VALUE = struct.pack("<I", 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", required=True, type=Path)
    ap.add_argument("outdir", type=Path)
    args = ap.parse_args()
    base_bytes = args.base.read_bytes()
    base = A.parse(base_bytes)
    if A.serialize(base) != base_bytes:
        sys.exit("base ADT does not round-trip")
    product = base.child("product")
    if product is None:
        sys.exit("base ADT has no /product")
    if product.get(PROP):
        sys.exit(f"base already has /product/{PROP}")
    product.props.append([PROP, len(VALUE), VALUE])
    result = A.serialize(base)
    before, after = A.flatten(A.parse(base_bytes)), A.flatten(A.parse(result))
    added = sorted(set(after) - set(before))
    if set(before) - set(after) or any(before[k] != after[k] for k in before) or len(added) != 1:
        sys.exit("an existing property changed")
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / "kernel.macho.adt").write_bytes(result)
    meta = {"base": str(args.base), "base_sha256": A.sha(base_bytes), "adt_sha256": A.sha(result), "added": added,
            "changes": [f"/product/{PROP} = 1 (u32, as in DeviceTree.d22ap): MGGetBoolAnswer(watch-companion) true, "
                        "NanoRegistry supportsWatch YES, Bridge gets an NR status"],
            "boot": "first boot: guamp-mg-adt-sync resets the MobileGestalt cache once (kernel with watch-companion in the "
                    "stamp), otherwise add guamp-mg-reset=1 for one boot",
            "tool": "scripts/make-guamp-watch-adt.py"}
    (args.outdir / "configurazione.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(meta["changes"][0])
    print(f"{args.outdir / 'kernel.macho.adt'}: {len(base_bytes)} -> {len(result)} bytes, sha256 {meta['adt_sha256']}")


if __name__ == "__main__":
    main()
