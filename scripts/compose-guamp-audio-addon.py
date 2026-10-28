#!/usr/bin/env python3
"""Merge the audio plug-in build (CI workflow audio-plugin.yml, or package/GuampAudio/build-guamp-audio-plugin.sh on a Mac) into the
payload's addon and owner trust bundle, on the owner's PC (the Apple trust modules and the iPhone-resources addon never go to CI).

  compose-guamp-audio-addon.py --audio DIR --base-addon BASE.gpuaddon --base-trust BASE.trustcache OUTDIR

DIR holds GuampAudio.driver/ and trust.json. The CDHash is not taken on trust: it is recomputed from the executable's code
signature (CodeDirectory, SHA-256, 4 KiB pages required) and must equal trust.json's.
OUTDIR/kernel.macho.gpuaddon     BASE + audio/HAL/GuampAudio.driver/* (scripts/make-guamp-gpu-addon.py --no-gpu --audio)
OUTDIR/kernel.macho.trustcache   BASE's modules, with the audio CDHash merged into the owner module (module 0, recognised by its
                                 uuid5 identifier over its own CDHashes, as owner-trust-module.py / make-owner-trust-bundle.py
                                 build it); a bundle without owner module (the original Apple one) gets a new owner module in
                                 front. Apple modules stay byte for byte.
OUTDIR/audio-addon.json          hashes and provenance
Then: prepara-boot-smp-fix.py ... --gpuaddon OUTDIR/kernel.macho.gpuaddon --trustcache OUTDIR/kernel.macho.trustcache and boot-arg
guamp-audio-plugin=1 (patches/xnu/zzzzzzz-guamp-audio-plugin.patch)."""
import argparse
import hashlib
import json
import struct
import subprocess
import sys
import uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent
OWNER_NAMESPACES = ('org.blu-sharky.adreno-metal:', 'com.guamp.owner:')


def modules(data):
    """Modules of a trust bundle, with the checks of DarwinBoot ValidTrustCaches (as componi-gpu-addon.py)."""
    count = struct.unpack_from('<I', data)[0]
    if not 0 < count <= 128:
        sys.exit('trust bundle: module count')
    starts = list(struct.unpack_from(f'<{count}I', data, 4))
    out = []
    for start, end in zip(starts, starts[1:] + [len(data)]):
        if start < 4 + 4 * count or start & 3 or end > len(data) or end - start < 24:
            sys.exit('trust bundle: module bounds')
        if struct.unpack_from('<I', data, start)[0] != 2:
            sys.exit('trust bundle: module version')
        entries = struct.unpack_from('<I', data, start + 20)[0]
        if end - start != 24 + entries * 24:
            sys.exit('trust bundle: module size')
        for entry in range(entries):
            at = start + 24 + entry * 24
            if not 1 <= data[at + 20] <= 4 or (entry and data[at - 24:at - 4] >= data[at:at + 20]):
                sys.exit('trust bundle: hash type or order')
        out.append(data[start:end])
    return out


def bundle_of(parts):
    starts, cursor = [], 4 + 4 * len(parts)
    for part in parts:
        starts.append(cursor)
        cursor += len(part)
    return struct.pack(f'<{len(parts) + 1}I', len(parts), *starts) + b''.join(parts)


def module_hashes(module):
    return [module[at:at + 20] for at in range(24, len(module), 24)]


def owner_namespace(module):
    hashes = module_hashes(module)
    for namespace in OWNER_NAMESPACES:
        if uuid.uuid5(uuid.NAMESPACE_URL, namespace + ''.join(h.hex() for h in hashes)).bytes == module[4:20]:
            return namespace
    return None


def owner_module(hashes, namespace):
    hashes = sorted(set(hashes))
    identifier = uuid.uuid5(uuid.NAMESPACE_URL, namespace + ''.join(h.hex() for h in hashes))
    return struct.pack('<I16sI', 2, identifier.bytes, len(hashes)) + b''.join(h + bytes([2, 0, 0, 0]) for h in hashes)


def cdhash(executable):
    """CDHash (SHA-256 truncated to 20 bytes) of the best SHA-256 CodeDirectory of a thin arm64 Mach-O, and its page size."""
    data = executable.read_bytes()
    magic, cputype, _, _, ncmds = struct.unpack_from('<IiiII', data)
    if magic != 0xfeedfacf or cputype != 0x0100000c:
        sys.exit(f'{executable}: not a thin arm64 Mach-O')
    at = 32
    for _ in range(ncmds):
        cmd, size = struct.unpack_from('<II', data, at)
        if cmd == 0x1d:   # LC_CODE_SIGNATURE
            offset, length = struct.unpack_from('<II', data, at + 8)
            blob = data[offset:offset + length]
            smagic, _, count = struct.unpack_from('>III', blob)
            if smagic != 0xfade0cc0:
                sys.exit(f'{executable}: no embedded signature superblob')
            best = None
            for index in range(count):
                slot, where = struct.unpack_from('>II', blob, 12 + index * 8)
                if slot != 0 and not 0x1000 <= slot < 0x1005:
                    continue
                cmagic, clen = struct.unpack_from('>II', blob, where)
                if cmagic != 0xfade0c02:
                    continue
                directory = blob[where:where + clen]
                hash_type, page_log2 = directory[37], directory[39]
                if hash_type == 2 and best is None:
                    best = (hashlib.sha256(directory).digest()[:20], 1 << page_log2 if page_log2 else 0)
            if best is None:
                sys.exit(f'{executable}: no SHA-256 CodeDirectory')
            return best
        at += size
    sys.exit(f'{executable}: unsigned')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--audio', type=Path, required=True, help='GuampAudio.driver/ + trust.json (CI output guamp-audio/)')
    parser.add_argument('--base-addon', type=Path, required=True)
    parser.add_argument('--base-trust', type=Path, required=True)
    parser.add_argument('--allow-16k', action='store_true', help='accept a 16 KiB-page signature (test only)')
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    trust = json.loads((args.audio / 'trust.json').read_text())
    digest, page = cdhash(args.audio / 'GuampAudio.driver' / 'GuampAudio')
    if digest.hex() != trust.get('cdhash') or trust.get('cdhashes') != [digest.hex()]:
        sys.exit(f'trust.json does not name the executable\'s CDHash {digest.hex()}')
    if page != 4096 and not args.allow_16k:
        sys.exit(f'code signature page size {page}, the guamp kernel needs 4096')
    if (args.output / 'kernel.macho.gpuaddon').exists():
        sys.exit(f'refuse to overwrite {args.output}')
    args.output.mkdir(parents=True, exist_ok=True)

    parts = modules(args.base_trust.read_bytes())
    namespace = owner_namespace(parts[0])
    if namespace:
        hashes = module_hashes(parts[0])
        if digest in hashes:
            sys.exit('the owner module already lists this CDHash')
        parts[0] = owner_module(hashes + [digest], namespace)
        how = f'merged into the owner module ({namespace} {len(hashes) + 1} CDHashes)'
    else:
        parts.insert(0, owner_module([digest], OWNER_NAMESPACES[1]))
        how = 'new owner module in front of the original modules'
    bundle = bundle_of(parts)
    if modules(bundle) != parts:
        sys.exit('trust bundle round trip')
    (args.output / 'kernel.macho.trustcache').write_bytes(bundle)

    addon = args.output / 'kernel.macho.gpuaddon'
    subprocess.run([sys.executable, HERE / 'make-guamp-gpu-addon.py', '--base', args.base_addon, '--no-gpu', '--audio', args.audio,
                    addon], check=True)
    report = dict(cdhash=digest.hex(), page_size=page, trust=how,
                  gpuaddon_sha256=hashlib.sha256(addon.read_bytes()).hexdigest(),
                  trustcache_sha256=hashlib.sha256(bundle).hexdigest(),
                  base_addon_sha256=hashlib.sha256(args.base_addon.read_bytes()).hexdigest(),
                  base_trust_sha256=hashlib.sha256(args.base_trust.read_bytes()).hexdigest(),
                  provenance=(args.audio / 'provenance.txt').read_text() if (args.audio / 'provenance.txt').exists() else None,
                  boot_arg='guamp-audio-plugin=1')
    (args.output / 'audio-addon.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f'CDHash {digest.hex()} ({how}); addon {report["gpuaddon_sha256"][:16]}, trust {report["trustcache_sha256"][:16]}')


if __name__ == '__main__':
    main()
