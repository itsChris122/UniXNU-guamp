#!/usr/bin/env python3
"""Pack the iPhone-only resource files for the guamp iPadOS overlay (patches/xnu/zzzz-guamp-iphone-res.patch) into a boot addon and
add the addon's slot to a copy of the ADT.

  make-guamp-iphone-res.py --adt-in ADT --adt-out ADT --addon-out FILE [--res DIR] [--ipad-root DIR] [--adt-lib DIR]

--res (default ~/moto/iphone-res) holds the files at their iPhone paths, System/Library/PrivateFrameworks/<Framework>.framework/<file>
(copied from a read-only iPhone root, never from this repository). Only the frameworks the kernel mounts are accepted, only files
directly in the framework directory (the union view falls through to the sealed directory for root names only), never Info.plist,
_CodeSignature or *.lproj, and with --ipad-root (default ~/moto/mnt-osroot/root when present) only names the iPadOS directory does not
have: an existing name would replace the sealed file for every process.

The addon is the NGPUADD2 layout of scripts/make-gpu-addon.py (zlib per file), names iphone-res/<Framework>.framework/<file>, mode
0644. It goes into the payload FAT as EFI/UniXNU/Payloads/kernel.macho.gpuaddon; DarwinBoot then needs the empty 16-byte
/chosen/memory-map/GpuAddon slot in kernel.macho.adt (and refuses a slot without the file and the file without the slot). The kernel
(boot-arg guamp-iphone-res=1) writes every file whose content or mode differs into Data's /private/var/ngpu and mounts the views.
The ADT copy differs from the input only by that one property; no ADT value is printed (it holds the unique chip id)."""
import argparse
import hashlib
import struct
import sys
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# Must match native_guamp_iphone_res[] in patches/xnu/zzzz-guamp-iphone-res.patch
FRAMEWORKS = {'TextInputUI.framework', 'ControlCenterUI.framework'}
PREFIX = Path('System/Library/PrivateFrameworks')
MAGIC = b'NGPUADD2'


def pad(data):
    return data + bytes(-len(data) % 8)


def collect(res, ipad_root):
    files = []
    base = res / PREFIX
    for path in sorted(p for p in res.rglob('*') if p.is_file() or p.is_symlink()):
        rel = path.relative_to(res)
        parts = rel.parts
        if path.is_symlink() or rel.parent.parent != PREFIX or parts[3] not in FRAMEWORKS:
            sys.exit(f'Not an accepted overlay file (only regular files directly in {sorted(FRAMEWORKS)}): {rel}')
        name = parts[4]
        if name == 'Info.plist' or name == '_CodeSignature' or name.endswith('.lproj') or name.startswith('.'):
            sys.exit(f'Refused name: {rel}')
        if ipad_root:
            sealed = ipad_root / rel.parent
            if not sealed.is_dir():
                sys.exit(f'{sealed} missing: the kernel mounts over that directory')
            if (ipad_root / rel).exists() or (ipad_root / rel).is_symlink():
                sys.exit(f'{rel} exists in the iPadOS root: only new names may be overlaid')
        entry = f'iphone-res/{parts[3]}/{name}'
        if len(entry.encode()) + 1 > 160:
            sys.exit('Addon path too long: ' + entry)
        files.append((entry, 0o644, path.read_bytes()))
    if not files:
        sys.exit(f'No files below {base}')
    return files


def build(files):
    body = b''
    for name, mode, data in files:
        encoded = name.encode() + b'\0'
        packed = zlib.compress(data, 9)
        body += struct.pack('<IIQQ', len(encoded), mode, len(data), len(packed)) + pad(encoded) + pad(packed)
    return MAGIC + struct.pack('<IIQ', len(files), 0, 24 + len(body)) + body


def check(blob, files):
    """The kernel's own validation (native_gpu_addon_install), then the content."""
    count, _, total = struct.unpack_from('<IIQ', blob, 8)
    assert blob[:8] == MAGIC and total == len(blob) and 0 < count <= 4096
    at, seen = 24, []
    for _ in range(count):
        name_bytes, mode, size, packed = struct.unpack_from('<IIQQ', blob, at)
        at += 24
        name = blob[at:at + name_bytes]
        assert 2 <= name_bytes <= 160 and name[-1] == 0 and b'\0' not in name[:-1] and name[0:1] != b'/' and b'..' not in name
        assert not mode & ~0o777 and size <= 0x7fffffff and 0 < packed <= 0x7fffffff
        at += (name_bytes + 7) & ~7
        data = zlib.decompress(blob[at:at + packed])
        assert len(data) == size
        seen.append((name[:-1].decode(), mode, data))
        at += (packed + 7) & ~7
    assert at == len(blob) and seen == files


def add_slot(adt_mod, data):
    tree, end = adt_mod.decode(data)
    assert end == len(data) and adt_mod.encode(tree) == data, 'ADT does not round-trip'
    before = adt_mod.flatten(tree)

    def find(node, names):
        props, children = node
        if not names:
            return node
        for child in children:
            name = next(v for k, v, _ in child[0] if k.rstrip(b'\0') == b'name').rstrip(b'\0').decode()
            if name == names[0]:
                return find(child, names[1:])
        sys.exit('ADT has no /' + '/'.join(names))

    memory_map = find(tree, ['chosen', 'memory-map'])
    key = b'GpuAddon'.ljust(32, b'\0')
    existing = [v for k, v, _ in memory_map[0] if k == key]
    if existing:
        if existing[0] != bytes(16):
            sys.exit('/chosen/memory-map/GpuAddon exists and is not an empty 16-byte slot')
        print('ADT already has the empty GpuAddon slot')
    else:
        memory_map[0].append((key, bytes(16), b''))
    out = adt_mod.encode(tree)
    check_tree, end = adt_mod.decode(out)
    after = adt_mod.flatten(check_tree)
    assert end == len(out)
    added = set(after) - set(before)
    assert all(after[k] == v for k, v in before.items()), 'an existing ADT property changed'
    assert added <= {'/device-tree/chosen/memory-map/GpuAddon'} and after['/device-tree/chosen/memory-map/GpuAddon'] == bytes(16)
    return out


def main():
    home = Path.home()
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--res', type=Path, default=home / 'moto/iphone-res')
    parser.add_argument('--ipad-root', type=Path, default=None)
    parser.add_argument('--adt-in', type=Path, required=True)
    parser.add_argument('--adt-out', type=Path, required=True)
    parser.add_argument('--addon-out', type=Path, required=True)
    parser.add_argument('--adt-lib', type=Path, default=ROOT.parent / 'interfaccia-iphone', help='directory of adt.py')
    args = parser.parse_args()
    ipad_root = args.ipad_root
    if ipad_root is None and (home / 'moto/mnt-osroot/root/System').is_dir():
        ipad_root = home / 'moto/mnt-osroot/root'
    if ipad_root is None:
        print('warning: no iPadOS root to check names against (--ipad-root)')
    sys.path.insert(0, str(args.adt_lib.resolve()))
    import adt as adt_mod   # Codex/interfaccia-iphone/adt.py: lossless ADT codec

    for out in (args.adt_out, args.addon_out):
        if out.resolve() == args.adt_in.resolve():
            sys.exit('Refuse to overwrite the input ADT')
    files = collect(args.res.resolve(), ipad_root)
    blob = build(files)
    check(blob, files)
    adt_out = add_slot(adt_mod, args.adt_in.read_bytes())
    args.addon_out.write_bytes(blob)
    args.adt_out.write_bytes(adt_out)
    for name, mode, data in files:
        print(f'{mode:o} {len(data):>10} {hashlib.sha256(data).hexdigest()} {name}')
    print(f'{args.addon_out}: {len(files)} files, {len(blob)} bytes, sha256 {hashlib.sha256(blob).hexdigest()}')
    print(f'{args.adt_out}: {len(adt_out)} bytes (+{len(adt_out) - args.adt_in.stat().st_size}), '
          f'sha256 {hashlib.sha256(adt_out).hexdigest()}')


if __name__ == '__main__':
    main()
