#!/usr/bin/env python3
"""Camera capability ADT variant: adds to a guamp ADT what libMobileGestalt (21H461) needs to report the cameras, and
nothing else. Reuses the lossless parser of make-guamp-audio-adt.py.

  make-guamp-camera-adt.py --base BASE/kernel.macho.adt [--only rear|front] [--j120 DeviceTree.j120ap.im4p] OUTDIR

Why (static analysis of libMobileGestalt 21H461, 05/11; analisi/camera/cam6-integrazione.md):
  * Camera.app has SBAppTags ["still-camera"]: SpringBoard hides the icon while MG "still-camera" is false (the app
    still opens from Control Center, as seen on fin1).
  * MG "still-camera" (nv4RoLkNoPT0/rsO8Yaiew, answer 0x1a0225780) = front || rear.
  * MG "front-facing-camera" (mAAbkChrX3bpxPIffHG1BQ, 0x1a0229314) = MG H97G3GzDYb4wY5kwJ0AKfQ (0x1a02277a8) =
    MGIOKitHelper: entry named "isp" in the IODeviceTree plane has a property "camera-front".
  * MG "rear-facing-camera" (dp7SlZZQotrh8McQoH1xFA, 0x1a02261f4) = MG 6iI7/9cVUGIjdrSvIXEt4A (0x1a0217280) =
    IODeviceTree:/arm-io/isp has the property whose obfuscated name is X9NA3D+PguwY0i0Uffl07Q = "camera-rear".
  * MG "camera-flash" (jdLgKT+0BZmGrzd9J0cuPA, 0x1a0219fac) = IODeviceTree:/product/camera "flash" (already in the
    torch ADTs).
  J120 (iPad7,1) has exactly /arm-io/isp camera-front = camera-rear = u32 1.

Added (refuses a BASE that already has /arm-io/isp):
  /arm-io/isp   new node with only name="isp", camera-front=1 and/or camera-rear=1. No compatible, device_type, reg,
                interrupts or function-* properties: AppleH9CamIn matches IONameMatch "isp,s5l8960x" (compatible) only,
                so no Apple kext attaches (and the H9 kext is not in our kernelcache anyway).
  --j120: also copies the /product/camera properties of J120 that guamp lacks (auto-focus, hdr, burst, video fps...),
          only for the level with a real capture service; not needed for the icon.
The MobileGestalt cache keeps old answers: boot once with guamp-mg-reset=1, or use a kernel whose guamp-mg-adt-sync
stamps /arm-io/isp camera-front/camera-rear (branch codex/cam-capability-20261105).
Writes OUTDIR/kernel.macho.adt and OUTDIR/configurazione.json; every existing property is checked unchanged."""
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

U32_1 = struct.pack('<I', 1)
# /product/camera keys of J120 that describe a real capture pipeline; copied only with --j120
SKIP_J120 = {'AAPL,phandle', 'name'}


def prop(name, value):
    return [name, len(value), value]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', required=True, type=Path)
    ap.add_argument('--only', choices=('rear', 'front'), help='publish one camera only (default: both)')
    ap.add_argument('--j120', type=Path, help='also copy J120 /product/camera details (level with a capture service)')
    ap.add_argument('--lzfse', default=str(Path.home() / 'moto/tools/lzfse/build/bin/lzfse'))
    ap.add_argument('outdir', type=Path)
    args = ap.parse_args()
    base_bytes = args.base.read_bytes()
    base = A.parse(base_bytes)
    if A.serialize(base) != base_bytes:
        sys.exit('base ADT does not round-trip')
    armio, product = base.child('arm-io'), base.child('product')
    if not armio or not product:
        sys.exit('/arm-io or /product missing')
    if armio.child('isp'):
        sys.exit('base already has /arm-io/isp')
    changes = []
    isp = A.Node()
    isp.props.append(prop('name', b'isp\0'))
    for side in ('front', 'rear'):
        if args.only in (None, side):
            isp.props.append(prop(f'camera-{side}', U32_1))
            changes.append(f'/arm-io/isp/camera-{side} = 1')
    armio.children.append(isp)
    camera = product.child('camera')
    if not camera:
        camera = A.Node()
        camera.props.append(prop('name', b'camera\0'))
        product.children.append(camera)
        changes.append('/product/camera (new node)')
    if args.j120:
        src = A.j120_tree(args.j120, args.lzfse).child('product')
        src = src.child('camera') if src else None
        if not src:
            sys.exit('J120 /product/camera missing')
        for p in src.props:
            if p[0] in SKIP_J120 or camera.get(p[0]):
                continue
            camera.props.append(list(p))
            changes.append(f'/product/camera/{p[0]} = {p[2].hex()} (J120)')
    result = A.serialize(base)
    before, after = A.flatten(A.parse(base_bytes)), A.flatten(A.parse(result))
    added = sorted(set(after) - set(before))
    if set(before) - set(after) or any(before[k] != after[k] for k in before):
        sys.exit('an existing property changed')
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / 'kernel.macho.adt').write_bytes(result)
    meta = {'base': str(args.base), 'base_sha256': A.sha(base_bytes), 'adt_sha256': A.sha(result), 'added': added,
            'changes': changes, 'only': args.only, 'j120': str(args.j120) if args.j120 else None,
            'mg_keys': {'still-camera': 'front || rear', 'front-facing-camera': '/arm-io/isp camera-front',
                        'rear-facing-camera': '/arm-io/isp camera-rear', 'camera-flash': '/product/camera flash'},
            'boot_args': 'guamp-mg-reset=1 once (or a kernel with guamp-mg-adt-sync stamping /arm-io/isp, '
                         'codex/cam-capability-20261105); guamp-home-reset if the icon is still missing',
            'runtime_verified': False,
            'tool': 'scripts/make-guamp-camera-adt.py (worktree wt-cam-cap, branch codex/cam-capability-20261105)'}
    (args.outdir / 'configurazione.json').write_text(json.dumps(meta, indent=2) + '\n')
    print('\n'.join(changes))
    print(f'{args.outdir / "kernel.macho.adt"}: {len(base_bytes)} -> {len(result)} bytes, {len(added)} keys added, '
          f'sha256 {meta["adt_sha256"]}')


if __name__ == '__main__':
    main()
