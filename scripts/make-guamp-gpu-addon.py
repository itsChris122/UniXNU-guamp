#!/usr/bin/env python3
"""Merge the owner-built Adreno userspace (Turnip + NativeMetal plugin + adrenoprobed) into guamp's GPU addon.

  make-guamp-gpu-addon.py --base BASE.gpuaddon [--plugin DIR] [--publish] [--clients NAME,...] [--prime-list FILE] OUTPUT

guamp already ships one NGPUADD2 addon (the iPhone resources of scripts/make-guamp-iphone-res.py, iphone-res/...), and the loader hands
the kernel a single /chosen/memory-map/GpuAddon. This tool keeps every entry of BASE unchanged and adds the files that UniXNU's
make-gpu-addon.py would add (same paths below /private/var/ngpu, same modes): addons/NativeAdrenoMetal.bundle/*, the optional
.publish/.clients/.primelist markers, bin/adrenoprobed and daemons/*.plist, taken from DIR (default build/adreno-metal-plugin of the
UniXNU build tree, produced by build-adreno-metal-plugin.py after build-turnip-darwin.py --target ios).

Nothing here is enabled by default: the plugin files only matter with the kernel's addon mount (sheng-gpu-addon=1) and the A610
accelerator (guamp-adreno=1), and the code only runs when its CDHashes (DIR/trust.json) are in the payload trust cache. Without
--publish the plugin runs its self-tests only and backboardd keeps the software renderer.

Layout (UniXNU make-gpu-addon.py), little endian: "NGPUADD2", u32 count, u32 0, u64 total bytes; per file u32 path bytes (with NUL,
at most 160), u32 mode, u64 size, u64 compressed size, the path padded to 8 bytes, the zlib stream padded to 8 bytes."""
import argparse
import struct
import sys
import zlib
from pathlib import Path

MAGIC = b'NGPUADD2'
EXECUTABLES = ('NativeAdrenoMetal', 'libvulkan_freedreno.dylib')


def pad(data):
    return data + bytes(-len(data) % 8)


def read_addon(path):
    blob = path.read_bytes()
    if len(blob) < 24 or blob[:8] != MAGIC:
        sys.exit(f'{path}: not an NGPUADD2 addon')
    count, zero, total = struct.unpack_from('<IIQ', blob, 8)
    if zero or total != len(blob):
        sys.exit(f'{path}: bad header (count {count}, total {total}, size {len(blob)})')
    at, files = 24, []
    for _ in range(count):
        name_bytes, mode, size, packed = struct.unpack_from('<IIQQ', blob, at)
        at += 24
        name = blob[at:at + name_bytes - 1].decode()
        at += name_bytes + (-name_bytes % 8)
        data = zlib.decompress(blob[at:at + packed])
        at += packed + (-packed % 8)
        if len(data) != size:
            sys.exit(f'{path}: {name} inflates to {len(data)} bytes, header says {size}')
        files.append((name, mode, data))
    if at != len(blob):
        sys.exit(f'{path}: {len(blob) - at} trailing bytes')
    return files


def plugin_files(plugin, publish, clients, prime_list):
    bundle = plugin / 'NativeAdrenoMetal.bundle'
    if not (bundle / 'NativeAdrenoMetal').is_file() or not (bundle / 'libvulkan_freedreno.dylib').is_file():
        sys.exit(f'{plugin}: build the plugin first (build-turnip-darwin.py --target ios, then build-adreno-metal-plugin.py)')
    if not (plugin / 'trust.json').is_file():
        sys.exit(f'{plugin}: trust.json missing (the CDHashes the payload trust cache must carry)')
    files = []
    for path in sorted(p for p in bundle.rglob('*') if p.is_file()):
        mode = 0o755 if path.name in EXECUTABLES else 0o644
        files.append(('addons/NativeAdrenoMetal.bundle/' + path.relative_to(bundle).as_posix(), mode, path.read_bytes()))
    if publish:
        files.append(('addons/NativeAdrenoMetal.publish', 0o644, b'1\n'))
    if clients:
        files.append(('addons/NativeAdrenoMetal.clients', 0o644, ('\n'.join(clients) + '\n').encode()))
    if prime_list:
        lines = prime_list.read_text().split()
        if not lines or any(not line.startswith('/') or not line.endswith('.metallib') for line in lines):
            sys.exit('--prime-list takes a file of absolute .metallib paths')
        files.append(('addons/NativeAdrenoMetal.primelist', 0o644, prime_list.read_bytes()))
    files.append(('bin/adrenoprobed', 0o755, (plugin / 'adrenoprobed').read_bytes()))
    for plist in sorted(plugin.glob('*.plist')):
        if not plist.name.endswith('-entitlements.plist'):   # build input, not a launchd job
            files.append(('daemons/' + plist.name, 0o644, plist.read_bytes()))
    return files


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--base', type=Path, required=True, help='the existing guamp addon (iphone-res)')
    parser.add_argument('--plugin', type=Path, default=Path('build/adreno-metal-plugin'))
    parser.add_argument('--publish', action='store_true', help='backboardd gets the native Metal device (GPU compositing)')
    parser.add_argument('--clients', help='other processes with their own Metal device, comma separated ("*" = all)')
    parser.add_argument('--prime-list', type=Path, help='absolute .metallib paths the render server compiles ahead')
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    clients = [name for name in (args.clients or '').split(',') if name]
    if any(not all(c.isalnum() or c in '._-*' for c in name) for name in clients):
        sys.exit('--clients takes process names separated by commas')
    if args.output.exists():
        sys.exit(f'Refuse to overwrite {args.output}')
    base = read_addon(args.base)
    added = plugin_files(args.plugin, args.publish, clients, args.prime_list)
    names = {name for name, _, _ in base}
    clash = sorted(name for name, _, _ in added if name in names)
    if clash:
        sys.exit('BASE already has: ' + ', '.join(clash))
    body = b''
    for name, mode, data in base + added:
        encoded = name.encode() + b'\0'
        if len(encoded) > 160 or '..' in name.split('/') or name.startswith('/'):
            sys.exit('Bad addon path ' + name)
        packed = zlib.compress(data, 9)
        body += struct.pack('<IIQQ', len(encoded), mode, len(data), len(packed)) + pad(encoded) + pad(packed)
    blob = MAGIC + struct.pack('<IIQ', len(base) + len(added), 0, 24 + len(body)) + body
    args.output.write_bytes(blob)
    for name, mode, data in added:
        print(f'{mode:o} {len(data):>10} {name}')
    print(f'{args.output}: {len(base)} base + {len(added)} GPU files, {len(blob)} bytes')


if __name__ == '__main__':
    main()
