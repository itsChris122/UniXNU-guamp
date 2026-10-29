#!/usr/bin/env python3
"""Delta addon (NGPUADD2) for the persistent addon of zzzzzzzzzzzzzzzzz-guamp-addon-manifest.patch: given the addon whose files are
already in Data (--old, the last one booted) and the wanted one (--new), writes only the entries that are new or changed (name, mode
or content) and, with --delete, a delete entry (mode 0x10000, size 0) for every file of --old that --new no longer has.
With nothing to do it writes an empty addon (24-byte header): the loader accepts it and the patched kernel writes nothing.
A kernel without the manifest patch rejects delete entries and empty addons as a whole (EINVAL, nothing written): use --delete and
empty deltas only with the patched kernel.

  compose-guamp-delta-addon.py --old OLD.gpuaddon --new NEW.gpuaddon --out DELTA.gpuaddon [--delete]"""
from pathlib import Path
import argparse
import hashlib
import json
import struct
import zlib

MAGIC = b'NGPUADD2'
DELETE = 0x10000


def pad(blob):
    return blob + bytes(-len(blob) % 8)


def read(path):
    blob = Path(path).read_bytes()
    assert blob[:8] == MAGIC
    count, _, total = struct.unpack_from('<IIQ', blob, 8)
    assert total == len(blob)
    at, entries = 24, {}
    for _ in range(count):
        name_bytes, mode, size, packed = struct.unpack_from('<IIQQ', blob, at)
        at += 24
        name = blob[at:at + name_bytes - 1].decode()
        at += (name_bytes + 7) & ~7
        data = zlib.decompress(blob[at:at + packed]) if mode != DELETE else b''
        at += (packed + 7) & ~7
        assert len(data) == size and name not in entries
        entries[name] = (mode, data)
    assert at == len(blob)
    return entries


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--old', required=True)
    parser.add_argument('--new', required=True)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--delete', action='store_true')
    options = parser.parse_args()
    assert not options.out.exists(), 'exists: %s' % options.out
    old, new = read(options.old), read(options.new)
    changed = [(n, m, d) for n, (m, d) in sorted(new.items()) if m != DELETE and old.get(n) != (m, d)]
    removed = sorted(n for n in old if n not in new) if options.delete else []
    body = b''
    entries = changed + [(n, DELETE, b'') for n in removed]
    assert len(entries) <= 4096
    for name, mode, data in entries:
        encoded = name.encode() + b'\0'
        assert len(encoded) <= 160 and '..' not in name.split('/') and not name.startswith('/'), name
        packed = zlib.compress(data, 9)
        body += struct.pack('<IIQQ', len(encoded), mode, len(data), len(packed)) + pad(encoded) + pad(packed)
    addon = MAGIC + struct.pack('<IIQ', len(entries), 0, 24 + len(body)) + body
    assert len(addon) <= 64 * 1024 * 1024
    options.out.write_bytes(addon)
    print(json.dumps(dict(changed=len(changed), deleted=len(removed), bytes=len(addon),
                          sha256=hashlib.sha256(addon).hexdigest()), indent=2))


if __name__ == '__main__':
    main()
