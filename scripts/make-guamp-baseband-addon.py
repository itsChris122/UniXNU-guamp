#!/usr/bin/env python3
"""Addon (NGPUADD2) with the Mav16 baseband firmware for the cellular bridge (guamp-baseband=1, 04/11).

ABM/BBUpdater boot the baseband through the bridge's SAHARA channel only when the Mav16 firmware of the selected IPSW is in their
firmware folder (UniXNU/QcomBaseband platform/coachz/README.md: the reference archive CZBBFW01 under /private/var/coachz/baseband/Mav16).
On guamp the files travel in the payload addon instead: every file of Firmware/vinyl_02_Mav16-11.00.01.Release.bbfw (21H461) becomes
the entry baseband/standalone/Baseband/Mav16/<path>, which native_gpu_addon_install writes into Data's /private/var/ngpu and
zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-baseband.patch shows over /usr/standalone/firmware (Baseband/Mav16). The addon is persistent
(zzzzzzzzzzzzzzzzz-guamp-addon-manifest.patch): after one boot with it the files stay in Data and later addons need not carry them.

  make-guamp-baseband-addon.py (--bbfw FILE.bbfw | --ipsw URL_OR_FILE) [--base ADDON] [--stub qdsp6sw.mbn] [--kernel KERNEL.macho]
                               --out OUT.gpuaddon

Size (05/11): the Mav16 files take 33.8 MB compressed, 30.7 MB of it qdsp6sw.mbn; next to the 54.4 MB kernel the 80 MiB ramdisk
leaves about 28 MB for the whole addon. --stub qdsp6sw.mbn (see elf_skeleton) brings the firmware to about 3.2 MB; --kernel prints
the room left.

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
RAMDISK = 80 * 1024 * 1024   # GuampBoot FAT ramdisk (boot image at most 96 MiB)


def pad(blob):
    return blob + bytes(-len(blob) % 8)


def elf_skeleton(data):
    """Same size, same ELF header, program headers and MBN hash/signature segment; the bytes of every PT_LOAD segment zeroed.

    Why (05/11): qdsp6sw.mbn alone is 30.7 MB compressed and does not fit the 80 MiB ramdisk next to the 54.4 MB kernel, even in an
    addon of its own. The bridge never asks BBUpdater for it: its SAHARA end requests only image 0x15 (sbl1.mbn, coachz_baseband.cpp
    bb_sahara_images) and throws the bytes away, and the Qualcomm modem runs its own firmware from modem_<slot>. ABM/BBUpdater still
    find the file, its size and its ELF layout. If BBUpdater ever digests the whole image, rebuild without --stub (and with a smaller
    kernel or a bigger ramdisk)."""
    assert data[:4] == b'\x7fELF' and data[4] in (1, 2), 'not an ELF image'
    out = bytearray(data)
    if data[4] == 1:
        phoff, = struct.unpack_from('<I', data, 0x1c)
        phent, phnum = struct.unpack_from('<HH', data, 0x2a)
        segments = [struct.unpack_from('<8I', data, phoff + phent * i) for i in range(phnum)]
        loads = [(off, size) for kind, off, _, _, size, _, flags, _ in segments if kind == 1]
    else:
        phoff, = struct.unpack_from('<Q', data, 0x20)
        phent, phnum = struct.unpack_from('<HH', data, 0x36)
        segments = [struct.unpack_from('<IIQQQQQQ', data, phoff + phent * i) for i in range(phnum)]
        loads = [(off, size) for kind, flags, off, _, _, size, _, _ in segments if kind == 1]
    for off, size in loads:
        if not size:
            continue   # empty segments may point past the end of the file (qdsp6sw.mbn segment 28)
        assert off + size <= len(data) and off >= phoff + phent * phnum, 'segment past the end or over the headers'
        out[off:off + size] = bytes(size)
    return bytes(out)


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
    parser.add_argument('--stub', action='append', default=[], metavar='NAME',
                        help='firmware file (path inside Mav16, e.g. qdsp6sw.mbn) carried as an ELF skeleton: see elf_skeleton()')
    parser.add_argument('--thermal-from', type=Path, metavar='ADDON',
                        help='addon holding iphone-res/ThermalMonitor/motorola,guamp-Info.plist (e.g. resa37-watch): copied as J121AP-Info.plist')
    parser.add_argument('--kernel', type=Path, help='kernel.macho of the image: prints the GBPK estimate against the 80 MiB ramdisk')
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
    stubs, stubbed = list(options.stub), []
    with zipfile.ZipFile(io.BytesIO(bbfw)) as archive:
        for info in archive.infolist():
            if info.is_dir():
                continue
            name = PREFIX + info.filename
            encoded = name.encode() + b'\0'
            assert len(encoded) <= 160 and '..' not in info.filename.split('/') and not info.filename.startswith('/'), name
            assert name not in names, 'entry already in --base: %s' % name
            data = archive.read(info)
            if info.filename in stubs:
                data = elf_skeleton(data)
                stubbed.append(info.filename)
            entries.append((name, 0o644, len(data), zlib.compress(data, 9)))
            names.add(name)
            added += 1
    if set(stubs) - set(stubbed):
        raise SystemExit(f'--stub names not in the bbfw: {sorted(set(stubs) - set(stubbed))}')
    if options.thermal_from:
        # The baseband ADT makes hw.model J121AP: thermalmonitord then loads the stock iPad Pro J121AP-Info.plist (sensors this phone
        # never reports) instead of iphone-res/ThermalMonitor/motorola,guamp-Info.plist (zzzz-guamp-iphone-res.patch), and never
        # publishes the battery-health notifications Settings waits for (fin49: battery "Service", Battery Health page empty). The
        # same profile under the J121AP name shadows the stock one in the same union view.
        source = 'iphone-res/ThermalMonitor/motorola,guamp-Info.plist'
        found = [entry for entry in read_addon(options.thermal_from) if entry[0] == source]
        if not found:
            raise SystemExit(f'{source} not in {options.thermal_from}')
        target = 'iphone-res/ThermalMonitor/J121AP-Info.plist'
        assert target not in names, 'entry already in --base: %s' % target
        entries.append((target,) + tuple(found[0][1:]))
        names.add(target)
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
    report = dict(bbfw_sha256=digest, firmware_files=added, stubs=stubbed, base_entries=len(entries) - added, entries=len(entries),
                  bytes=len(addon), sha256=hashlib.sha256(addon).hexdigest())
    if options.kernel:
        # GBPK = kernel + ADT + trust cache + addon (+ ~12 KiB of headers, fin48); the FAT ramdisk is 80 MiB, its tables take a few
        # hundred KiB: 512 KiB of margin and 256 KiB for ADT and trust cache.
        room = RAMDISK - 512 * 1024 - 256 * 1024 - options.kernel.stat().st_size
        report.update(kernel_bytes=options.kernel.stat().st_size, addon_room=room, fits_ramdisk=len(addon) <= room)
    options.out.write_bytes(addon)
    print(json.dumps(report, indent=2))
    if options.kernel and len(addon) > report['addon_room']:
        print(f'WARNING: {len(addon)} bytes of addon do not fit the 80 MiB ramdisk next to this kernel ({report["addon_room"]} bytes left)')


if __name__ == '__main__':
    main()
