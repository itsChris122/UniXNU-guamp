#!/usr/bin/env python3
"""Personal Hotspot ADT variant: adds /product/personal-hotspot = 1, as in the iPhone X ADT (DeviceTree.d22ap).

  make-guamp-hotspot-adt.py --base BASE/kernel.macho.adt OUTDIR

Why (07/11, agent mg-sim): MobileGestalt cellular audit for the SIM ("SOS" with iliad SIM inside, Settings > Cellular empty).
libMobileGestalt 21H461 keys read by CommCenter, CoreTelephony, libTelephonyUtil/Capabilities, BBUpdater, SpringBoard,
SystemStatus(UI/Server), Preferences(UI), SettingsCellular(UI), SIMSetupSupport (scripts/check-guamp-mg-cellular.py): the
ADT-derived cellular answers are already those of an iPhone (HasBaseband, cellular-data, contains-cellular-radio, telephony,
sms, mms, CellularTelephonyCapability true; data-plan false; no EUICCChipID; BasebandClass 3). SIM state, SIM tray, phone
number, IMEI/MEID, dual SIM capability, carrier bundles and MCC/MNC are asked to CommCenter at run time
(_CTServerConnection..., MGBasebandSupport.c), never read from the ADT, and no MobileGestalt key counts SIM slots; the
telephony product comes from hw.target ("motorola,guamp", i.e. HWModelStr), which libTelephonyUtil cannot parse
(text-fixup1 forces J121; D22 is not in the 21H461 radio table). So no MobileGestalt answer keeps the bar on "SOS".
The one iPhone telephony capability the guamp ADT lacked is personal-hotspot:
  personal-hotspot   IODeviceTree:/product "personal-hotspot" (answer 0x1a02227cc), read by Preferences
d22ap has it (01 00 00 00); without it Settings has no Personal Hotspot row (also missing inside Cellular).
Not added on purpose: /product/gps-capable (MG gps; no iOS GNSS driver, locationd would wait for a GPS that never answers),
/baseband region-sku, device-imei, imeisv (no MobileGestalt answer reads them from /baseband; IMEI comes from the modem).
MobileGestalt caches the answer: guamp-mg-adt-sync stamps /product personal-hotspot (same commit), so the first boot with
this ADT resets the MobileGestalt cache once; with an older kernel use guamp-mg-reset=1 once.
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

PROP = "personal-hotspot"
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
    if base.child("baseband") is None:
        sys.exit("base ADT has no /baseband: build it on a baseband ADT (make-guamp-baseband-adt.py)")
    product.props.append([PROP, len(VALUE), VALUE])
    result = A.serialize(base)
    before, after = A.flatten(A.parse(base_bytes)), A.flatten(A.parse(result))
    added = sorted(set(after) - set(before))
    if set(before) - set(after) or any(before[k] != after[k] for k in before) or len(added) != 1:
        sys.exit("an existing property changed")
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / "kernel.macho.adt").write_bytes(result)
    meta = {"base": str(args.base), "base_sha256": A.sha(base_bytes), "adt_sha256": A.sha(result), "added": added,
            "changes": [f"/product/{PROP} = 1 (u32, as in DeviceTree.d22ap): MGGetBoolAnswer(personal-hotspot) true, "
                        "Settings shows Personal Hotspot (top level and in Cellular)"],
            "boot": "first boot: guamp-mg-adt-sync resets the MobileGestalt cache once (kernel with personal-hotspot in the "
                    "stamp), otherwise add guamp-mg-reset=1 for one boot",
            "tool": "scripts/make-guamp-hotspot-adt.py"}
    (args.outdir / "configurazione.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(meta["changes"][0])
    print(f"{args.outdir / 'kernel.macho.adt'}: {len(base_bytes)} -> {len(result)} bytes, sha256 {meta['adt_sha256']}")


if __name__ == "__main__":
    main()
