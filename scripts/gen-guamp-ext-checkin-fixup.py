#!/usr/bin/env python3
"""Text fixup: launchd keeps at least MIN seconds (default 30) for an extension's launch check-in (iPadOS 17.7.11 21H461).

fin40b-fin55 (07/11): the Translate app's CacheDelete extension
(SequoiaTranslator.app/PlugIns/CacheDeleteExtension.appex) is killed by launchd every time it is started, 10.0 s after
exec: "guamp exit: CacheDeleteExten[..] rv 0x9 ... reason namespace 7 code 0x3 / reason \"extension check-in timeout\""
(the "image IconServices" in that line is only the nearest image: the PC is _mach_msg2_trap, main thread idle in
xpc_main -> NSXPCListener serviceListener -> PlugInKit -> _EXExtensionMain). A PlugInKit extension checks in with launchd
(+[EXXPCUtil checkInWithLaunchd], launch_extension_check_in_live_4UIKit, routine 0x2cc, "extension checked in as alive")
only after its first XPC event, i.e. when the host (CacheDeleteAppContainerCaches, 20-38 % of a core, the eMMC's main
reader) has sent its first message; here that takes more than 10 s, so the extension is killed and the host starts it
again at the next purge pass. With Data almost full (fin55: 288 MB available, 24 VQ_LOWDISK|VQ_DESIRED_DISK
notifications in 1150 s) the passes come every 10-40 s: 84 kills in one session (fin50 with free space: 3).

launchd (offsets of the 21H461 binary, = analisi/ramdisk/launchd): the launch watchdog of an extension is the timer
[job+0x330] armed in the spawn path with [job+0x33c] seconds (0x10001e6a0; 0 = "Skipping extension launch watchdog");
when it fires (0x100024c5c "Extension is hanging on launch. Killing.") the job is stopped with mode 2 = reason
"extension check-in timeout" (0x100020410). [job+0x33c] defaults to ExtensionWatchdogTimeout (0x100075990, 5 s, already
30 s through text-fixup0=launchd,75988: AegirPoster now gets 30 s) but a "_LaunchWatchdogTimeOut" key of the job's
overlay replaces it (0x1000068c4, "setting launch watchdog timeout to %u secs from overlay"): the CacheDelete extension
gets 10 s from there (exec -> kill exactly 10.0 s before and after the 30 s fixup). ExtensionFoundation puts @0 in that
key when the extension point disables the check-in timeout (-[_EXExtensionIdentity(Overlay) xpcOverlayDictionary...]).

Fixup (16 bytes at file offset 0x68cc, after "str x21,[sp]; adr x2,<format>" which stay and are now unused):
    nop; mov x0,x19; mov w1,#4; bl <log>            (the info log line of the overlay value)
 -> cmp w21,#MIN; mov w8,#MIN; ccmp w21,#0,#4,lo; csel w21,w21,w8,eq
then the original "str w21,[x19,#0x33c]; b <return>": a value of 0 (watchdog off) and values >= MIN are kept, 1..MIN-1
become MIN. w8 is free (the function returns right after). The kernel changes the page only when the original bytes
match (cs_text_fixup, patches/xnu/native-text-fixup.patch), so another launchd is left alone.
It does not make the host faster: it gives the extension 3x the time and cuts the restarts (and their dyld/eMMC work).
The storm itself goes with free space on Data (guamp-data-space, 76e83e9).

Output: one entry for the ADT (scripts/make-guamp-adt.py --text-fixup ...) or a boot argument text-fixupN=...
Usage: gen-guamp-ext-checkin-fixup.py [--min SECONDS] [--check path/to/launchd]"""
import argparse, struct, sys

FILE = 'launchd'
OFF = 0x68cc
ORIGINAL = bytes.fromhex('1f2003d5' 'e00313aa' '81008052' '305a0094')
# context that must surround it in the 21H461 binary
BEFORE = bytes.fromhex('f50300f9' 'c2032810')   # 0x68c4 str x21,[sp]; 0x68c8 adr x2,"setting launch watchdog ..."
AFTER = bytes.fromhex('753e03b9')                # 0x68dc str w21,[x19,#0x33c]
FORMAT_AT = 0x68c8 + 327800                      # adr target (file offset = vm - 0x100000000 in __TEXT)
FORMAT = b'setting launch watchdog timeout to %u secs from overlay\0'


def replacement(minimum):
    assert 1 <= minimum <= 4095, 'MIN must fit cmp (12-bit immediate)'
    cmp = 0x71000000 | (minimum << 10) | (21 << 5) | 31          # subs wzr, w21, #MIN
    mov = 0x52800000 | (minimum << 5) | 8                        # movz w8, #MIN
    ccmp = 0x7a400800 | (0 << 16) | (0x3 << 12) | (21 << 5) | 4  # ccmp w21, #0, #4 (Z), lo
    csel = 0x1a800000 | (8 << 16) | (0x0 << 12) | (21 << 5) | 21  # csel w21, w21, w8, eq
    return struct.pack('<4I', cmp, mov, ccmp, csel)


def model(value, minimum):
    """What the four instructions leave in w21 (flags of cmp/ccmp written out)."""
    z = (value == 0) if value < minimum else True
    return value if z else minimum


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--min', type=int, default=30)
    ap.add_argument('--check', metavar='LAUNCHD')
    a = ap.parse_args()
    new = replacement(a.min)
    if a.check:
        data = open(a.check, 'rb').read()
        assert data[OFF:OFF + 16] == ORIGINAL, 'original bytes differ at 0x%x: %s' % (OFF, data[OFF:OFF + 16].hex())
        assert data[OFF - 8:OFF] == BEFORE and data[OFF + 16:OFF + 20] == AFTER, 'context differs'
        assert data[FORMAT_AT:FORMAT_AT + len(FORMAT)] == FORMAT, 'format string not where adr points'
        for v, want in ((0, 0), (1, a.min), (5, a.min), (10, a.min), (a.min - 1, a.min), (a.min, a.min), (60, 60)):
            assert model(v, a.min) == want, (v, model(v, a.min), want)
        print('check ok: %s 0x%x' % (a.check, OFF), file=sys.stderr)
    print('%s,%x,%s,%s' % (FILE, OFF, ORIGINAL.hex(), new.hex()))


if __name__ == '__main__':
    main()
