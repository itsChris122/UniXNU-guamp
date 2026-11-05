#!/usr/bin/env python3
"""Text fixup that removes the 5 s ambient light sensor wait from backboardd's start (boot speed, 05/11/2026).

At every boot backboardd's window server start (BLControl start, CoreBrightness) asks CBU_ShouldWaitForALS(); on this
model list it answers yes, so -[BLControl waitForALSArrival] waits on a semaphore for a HID service matching the device
pair 0xFF00/0x04 with a 5 s timeout (dispatch time 0x12a05f200 ns). Our ambient light HID device (sensor kind 5,
scripts/guamp-hid-variant.py) is created by the kernel at ~2 s but no IOHIDEventDriver attaches to it, so the service
never appears and the wait always runs out (fin47 log: "service not found, wait for ALS arrival" at 26.2 s,
"timeout waiting for ALS arrival" at 31.2 s, backboardd "window server ready in 10.99 s"); everything after
the window server (display on, SpringBoard's lock screen, PosterBoard) starts 5 s later.

With CBU_ShouldWaitForALS() returning 0 the block goes straight to startHIDSystemClient, the same code that runs after
the timeout today: an ambient light service already present (the kernel publishes it ~15 s before backboardd starts)
is still found there, so nothing that works today is lost and a fixed ALS device is used as soon as it exists.

  _CBU_ShouldWaitForALS (CoreBrightness, iPadOS 17.7.11 21H461): VM 0x1b625e348 = dyld_shared_cache_arm64.17 file
  offset 0x29ea348; original sub sp,sp,#0x150; stp x28,x27,[sp,#0x130]; stp x29,x30,[sp,#0x140]; add x29,sp,#0x140
  replacement             mov w0,#0; ret; nop; nop

Entry: <file>,<offset>,<original>,<replacement> (16 bytes, cs_text_fixup of patches/xnu/native-text-fixup.patch, the
kernel changes the page only when the original bytes match). Use it as a free text-fixupN boot argument (fin48 uses
text-fixup0 for launchd and text-fixup1..4 for the lock screen floor) or in the ADT table (make-guamp-adt.py --text-fixup).
Usage: gen-guamp-als-wait-fixup.py [--dsc DIR_OR_dyld_shared_cache_arm64.17] [--check]"""
import argparse, os, shutil, struct, subprocess, sys, tempfile

FILE = 'dyld_shared_cache_arm64.17'
FILE_OFF = 0x29ea348
VM = 0x1b625e348
ORIGINAL = bytes.fromhex('ff4305d1fc6f13a9fd7b14a9fd030591')
REPLACEMENT = struct.pack('<4I', 0x52800000, 0xd65f03c0, 0xd503201f, 0xd503201f)   # mov w0,#0; ret; nop; nop


def entry():
    assert len(ORIGINAL) == len(REPLACEMENT) == 16
    assert (FILE_OFF & 0x3fff) + 16 <= 0x4000, 'the fixup must sit in one 16 KiB page'
    return '%s,%x,%s,%s' % (FILE, FILE_OFF, ORIGINAL.hex(), REPLACEMENT.hex())


def vm_to_file(path, vm):
    """File offset of vm in this subcache (its own mapping table), or None."""
    with open(path, 'rb') as f:
        head = f.read(0x200)
        if not head.startswith(b'dyld_v1'):
            return None
        offset, count = struct.unpack_from('<II', head, 0x10)
        f.seek(offset)
        for _ in range(count):
            address, size, file_offset, _max, _init = struct.unpack('<QQQII', f.read(32))
            if address <= vm < address + size:
                return file_offset + vm - address
    return None


def check_dsc(where):
    path = os.path.join(where, FILE) if os.path.isdir(where) else where
    at = vm_to_file(path, VM)
    assert at == FILE_OFF, 'VM 0x%x is not at %s+0x%x (got %r): not the 21H461 cache' % (VM, FILE, FILE_OFF, at)
    with open(path, 'rb') as f:
        f.seek(FILE_OFF)
        have = f.read(16)
    assert have == ORIGINAL, 'original bytes differ (%s): not the 21H461 CoreBrightness' % have.hex()
    print('original bytes checked in', path)


def disassemble(data):
    objdump = shutil.which('aarch64-linux-gnu-objdump')
    if objdump:
        with tempfile.TemporaryDirectory() as d:
            raw = os.path.join(d, 'code.bin')
            open(raw, 'wb').write(data)
            out = subprocess.run([objdump, '-D', '-b', 'binary', '-m', 'aarch64', '--adjust-vma=0x%x' % VM, raw],
                                 capture_output=True, text=True)
            if out.returncode == 0:
                pcs = {'%x' % (VM + 4 * i) for i in range(4)}
                return [l.split('\t', 2)[-1].strip() for l in out.stdout.splitlines() if l.strip().split(':')[0] in pcs]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dsc', help='directory with the 21H461 subcaches, or dyld_shared_cache_arm64.17 itself')
    ap.add_argument('--check', action='store_true', help='host test: encodings, page bound, disassembly if available')
    a = ap.parse_args()
    if a.dsc:
        check_dsc(a.dsc)
    if a.check:
        words = struct.unpack('<4I', REPLACEMENT)
        assert words[0] == 0x52800000 and words[1] == 0xd65f03c0 and words[2] == words[3] == 0xd503201f
        orig = struct.unpack('<4I', ORIGINAL)
        assert orig == (0xd10543ff, 0xa9136ffc, 0xa9147bfd, 0x910503fd), [hex(w) for w in orig]   # the 21H461 prologue
        lines = disassemble(REPLACEMENT)
        if lines is not None:
            want = ['mov\tw0, #0x0', 'ret', 'nop', 'nop']
            assert len(lines) == 4, lines
            for got, exp in zip(lines, want):
                assert ' '.join(got.split()).startswith(' '.join(exp.split())), (got, exp)
            for l in lines:
                print('  ' + l)
        else:
            print('  (aarch64-linux-gnu-objdump absent: encodings checked by value)')
        print('check ok')
    print(entry())


if __name__ == '__main__':
    sys.exit(main())
