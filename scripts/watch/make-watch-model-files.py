#!/usr/bin/env python3
"""Watch (Bridge.app, nanoregistryd) on the iPadOS root: the iPhone-only system plists the Watch stack reads, as model-files views.

  make-watch-model-files.py --base ADDON --xs-root XS_ROOT --ipad-root IPAD_ROOT --out OUT.gpuaddon [--report FILE]

Why (fin51, 06/11): Bridge (iPhone XS 17.7 21H16) talks to the Watch over IDS services that identityservicesd only knows from the
plists in /System/Library/IdentityServices/ServiceDefinitions. The iPadOS 17.7.11 volume has 63 of the XS's 114 definitions: it
lacks among others com.apple.private.alloy.pbbridge ("Setup Bridge", the pairing setup channel, Tinker relationship),
companionproxy, idstransfers, appregistrysync, preferencessync, otaupdate, pairedunlock, sockpuppet (app install), timesync,
watchconnectivity. /System/Library/FeatureFlags/Domain lacks Bridge.plist (ResumePairing, WatchSetupExpressMode, LinkUpgrade, ...)
and NanoRegistry.plist. All of them are plain plists (no code, nothing for the trust cache).

They travel like the per-model telephony files of make-guamp-baseband-addon.py --model-files: names model-files/<sealed dir>/<file>,
and model-files/views ("NGPUVIEWS1\n", one "<sealed dir>|<file>\n" per view) for zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-model-files.patch,
which mounts a read-only union bindfs view of /private/var/ngpu/model-files<sealed dir> over each listed sealed directory (lookup
and readdir fall through to the sealed directory for every other name, as the /Library/Ringtones view). Only names the iPadOS
directory does not have are taken: an existing name would replace the sealed file for every process. A model-files/views already
in --base (telephony files) is merged, never dropped. Apple files are read from the mounted roots and written only into OUT; nothing
of them goes into this repository."""
import argparse
import hashlib
import importlib.util
import json
import struct
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('bb', HERE.parent / 'make-guamp-baseband-addon.py')
bb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bb)

VIEWS = 'model-files/views'
# sealed directory -> names to take (None = every XS name the iPadOS directory lacks)
DIRECTORIES = {
    '/System/Library/IdentityServices/ServiceDefinitions': None,
    '/System/Library/FeatureFlags/Domain': ['Bridge.plist', 'NanoRegistry.plist'],
}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--xs-root', type=Path, required=True, help='mounted iPhone XS 21H16 OS volume')
    parser.add_argument('--ipad-root', type=Path, required=True, help='mounted iPadOS 21H461 OS volume')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    options = parser.parse_args()
    if options.out.exists():
        raise SystemExit(f'{options.out} exists: not overwritten')
    entries = bb.read_addon(options.base)
    names = {e[0] for e in entries}
    views = []
    for e in entries:
        if e[0] == VIEWS:
            text = zlib.decompress(e[3]).decode()
            assert text.startswith(bb.VIEWS_MAGIC), 'model-files/views of --base without its magic'
            views = [line for line in text[len(bb.VIEWS_MAGIC):].split('\n') if line]
    entries = [e for e in entries if e[0] != VIEWS]
    report = {}
    for point, wanted in DIRECTORIES.items():
        xs_dir, ipad_dir = options.xs_root / point.lstrip('/'), options.ipad_root / point.lstrip('/')
        if not ipad_dir.is_dir() or not xs_dir.is_dir():
            raise SystemExit(f'{point}: missing in one of the roots (the view needs the sealed directory)')
        have = {p.name for p in ipad_dir.iterdir()}
        pool = sorted(p.name for p in xs_dir.iterdir() if p.is_file() and not p.is_symlink())
        take = [n for n in (wanted if wanted is not None else pool) if n not in have]
        if wanted is not None and set(wanted) - set(pool):
            raise SystemExit(f'{point}: not in the XS root: {sorted(set(wanted) - set(pool))}')
        added = []
        for name in take:
            entry = f'model-files{point}/{name}'
            assert len(entry.encode()) + 1 <= 160 and '/' not in name and len(name) < 64, entry
            if entry in names:
                continue   # already in --base (rerun on its own output)
            data = (xs_dir / name).read_bytes()
            entries.append((entry, 0o644, len(data), zlib.compress(data, 9)))
            names.add(entry)
            added.append(name)
        if take and not any(v.startswith(point + '|') for v in views):
            views.append(f'{point}|{take[0]}')
        report[point] = dict(xs=len(pool), ipad=len(have), added=len(added), names=take)
    assert 0 < len(views) <= 128, 'the kernel mounts at most 128 views'
    listing = (bb.VIEWS_MAGIC + ''.join(v + '\n' for v in views)).encode()
    entries.append((VIEWS, 0o644, len(listing), zlib.compress(listing, 9)))
    assert len(entries) <= 4096
    body = b''
    for name, mode, size, packed in entries:
        encoded = name.encode() + b'\0'
        body += struct.pack('<IIQQ', len(encoded), mode, size, len(packed)) + bb.pad(encoded) + bb.pad(packed)
    addon = bb.MAGIC + struct.pack('<IIQ', len(entries), 0, 24 + len(body)) + body
    assert len(addon) <= bb.LIMIT
    options.out.parent.mkdir(parents=True, exist_ok=True)
    options.out.write_bytes(addon)
    summary = dict(base=str(options.base), entries=len(entries), bytes=len(addon), sha256=hashlib.sha256(addon).hexdigest(),
                   views=views, directories=report)
    if options.report:
        options.report.write_text(json.dumps(summary, indent=2))
    print(json.dumps({k: v for k, v in summary.items() if k != 'directories'} |
                     {'added': {p: r['added'] for p, r in report.items()}}, indent=2))


if __name__ == '__main__':
    main()
