#!/usr/bin/env python3
"""Addon (NGPUADD2) with the Mav16 baseband firmware for the cellular bridge (guamp-baseband=1, 04/11).

ABM/BBUpdater boot the baseband through the bridge's SAHARA channel only when the Mav16 firmware of the selected IPSW is in their
firmware folder (UniXNU/QcomBaseband platform/coachz/README.md: the reference archive CZBBFW01 under /private/var/coachz/baseband/Mav16).
On guamp the files travel in the payload addon instead: every file of Firmware/vinyl_02_Mav16-11.00.01.Release.bbfw (21H461) becomes
the entry baseband/standalone/Baseband/Mav16/<path>, which native_gpu_addon_install writes into Data's /private/var/ngpu and
zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-baseband.patch shows over /usr/standalone/firmware (Baseband/Mav16). The addon is persistent
(zzzzzzzzzzzzzzzzz-guamp-addon-manifest.patch): after one boot with it the files stay in Data and later addons need not carry them.

  make-guamp-baseband-addon.py (--bbfw FILE.bbfw | --ipsw URL_OR_FILE) [--base ADDON] --out OUT.gpuaddon

--base merges the entries of an existing addon (for example the delta of the current image) into the output, so one boot payload
carries both; an entry name present in both is refused. Apple firmware: it only goes into the boot payload, never into a repository.
A ".guamp-keep" of --base (zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-addon-prune.patch) is rebuilt with the firmware names and
baseband/ added; without one in --base the output has none and the kernel prunes nothing.
"""
from pathlib import Path
import argparse
import hashlib
import io
import json
import struct
import urllib.request
import zipfile
import zlib

MAGIC = b'NGPUADD2'
DELETE = 0x10000
PREFIX = 'baseband/standalone/Baseband/Mav16/'
# Firmware/vinyl_02_Mav16-11.00.01.Release.bbfw of iPad_Pro_HFR_17.7.11_21H461_Restore.ipsw (board.toml [ipsw] url), 04/11.
BBFW_NAME = 'Firmware/vinyl_02_Mav16-11.00.01.Release.bbfw'
BBFW_SHA256 = 'f0066dd01c4407f3104cff4fde86172ca5c14cec355e6b25f4777a2c61bee1d2'
LIMIT = 64 * 1024 * 1024


def pad(blob):
    return blob + bytes(-len(blob) % 8)


class Remote(io.RawIOBase):
    """Seekable HTTP range reader: only the IPSW's central directory and the bbfw are downloaded."""

    def __init__(self, url):
        self.url, self.at = url, 0
        self.size = int(urllib.request.urlopen(urllib.request.Request(url, method='HEAD')).headers['Content-Length'])

    def seekable(self):
        return True

    def readable(self):
        return True

    def tell(self):
        return self.at

    def seek(self, offset, whence=0):
        self.at = offset if whence == 0 else self.at + offset if whence == 1 else self.size + offset
        return self.at

    def readinto(self, buffer):
        if self.at >= self.size:
            return 0
        end = min(self.size, self.at + len(buffer)) - 1
        data = urllib.request.urlopen(urllib.request.Request(self.url, headers={'Range': f'bytes={self.at}-{end}'})).read()
        buffer[:len(data)] = data
        self.at += len(data)
        return len(data)


def bbfw_bytes(options):
    if options.bbfw:
        return options.bbfw.read_bytes()
    source = options.ipsw
    handle = io.BufferedReader(Remote(source), buffer_size=1 << 20) if source.startswith(('http://', 'https://')) else open(source, 'rb')
    with zipfile.ZipFile(handle) as ipsw:
        return ipsw.read(BBFW_NAME)


def read_addon(path):
    blob = Path(path).read_bytes()
    assert blob[:8] == MAGIC, 'not an NGPUADD2 addon: %s' % path
    count, _, total = struct.unpack_from('<IIQ', blob, 8)
    assert total == len(blob)
    at, entries = 24, []
    for _ in range(count):
        name_bytes, mode, size, packed = struct.unpack_from('<IIQQ', blob, at)
        at += 24
        name = blob[at:at + name_bytes - 1].decode()
        at += (name_bytes + 7) & ~7
        entries.append((name, mode, size, blob[at:at + packed]))
        at += (packed + 7) & ~7
    assert at == len(blob)
    return entries


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--bbfw', type=Path, help='the Mav16 .bbfw already extracted from the IPSW')
    source.add_argument('--ipsw', help='the 21H461 IPSW (file or URL, board.toml [ipsw] url): only the bbfw is read')
    parser.add_argument('--base', type=Path, help='an existing addon whose entries are kept in the output')
    parser.add_argument('--out', required=True, type=Path)
    options = parser.parse_args()
    assert not options.out.exists(), 'exists: %s' % options.out
    bbfw = bbfw_bytes(options)
    digest = hashlib.sha256(bbfw).hexdigest()
    if digest != BBFW_SHA256:
        raise SystemExit(f'unexpected bbfw SHA-256 {digest} (expected {BBFW_SHA256}, {BBFW_NAME} of 21H461)')
    entries = []
    if options.base:
        entries = read_addon(options.base)
    keep = [line for name, mode, size, packed in entries if name == '.guamp-keep'
            for line in zlib.decompress(packed).decode().split('\n')[1:] if line]
    had_keep = any(name == '.guamp-keep' for name, *_ in entries)
    entries = [entry for entry in entries if entry[0] != '.guamp-keep']
    names = {name for name, *_ in entries}
    added = 0
    with zipfile.ZipFile(io.BytesIO(bbfw)) as archive:
        for info in archive.infolist():
            if info.is_dir():
                continue
            name = PREFIX + info.filename
            encoded = name.encode() + b'\0'
            assert len(encoded) <= 160 and '..' not in info.filename.split('/') and not info.filename.startswith('/'), name
            assert name not in names, 'entry already in --base: %s' % name
            data = archive.read(info)
            entries.append((name, 0o644, len(data), zlib.compress(data, 9)))
            names.add(name)
            added += 1
    if had_keep:
        lines = sorted((set(keep) - {p for p in keep if p.endswith('/')}) | names | {'.guamp-keep'})
        lines += sorted({p for p in keep if p.endswith('/')} | {'baseband/'})
        listing = ('NGPUKEEP1\n' + ''.join(line + '\n' for line in lines)).encode()
        entries.append(('.guamp-keep', 0o644, len(listing), zlib.compress(listing, 9)))
    assert len(entries) <= 4096
    body = b''
    for name, mode, size, packed in entries:
        encoded = name.encode() + b'\0'
        body += struct.pack('<IIQQ', len(encoded), mode, size, len(packed)) + pad(encoded) + pad(packed)
    addon = MAGIC + struct.pack('<IIQ', len(entries), 0, 24 + len(body)) + body
    if len(addon) > LIMIT:
        raise SystemExit(f'addon of {len(addon)} bytes is over the {LIMIT}-byte limit: build it without --base and boot it once')
    options.out.write_bytes(addon)
    print(json.dumps(dict(bbfw_sha256=digest, firmware_files=added, base_entries=len(entries) - added, entries=len(entries),
                          bytes=len(addon), sha256=hashlib.sha256(addon).hexdigest()), indent=2))


if __name__ == '__main__':
    main()
