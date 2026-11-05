#!/usr/bin/env python3
"""Host test of the boot-relative text-fixup immediates (05/11).

patches/xnu/zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-text-fixup-uptime.patch rewrites, inside a fixup replacement, the
pair "movz xN,#S; movk xN,#0xb007,lsl #16" to the timebase seconds at the kernel start + S. Its added C is compiled on
the host with stand-ins (the fixup table, mach_absolute_time, absolutetime_to_nanoseconds with a 1 ns tick) and run on
the entries of scripts/gen-guamp-lockmin-fixup.py: the pair must become boot + limit (capped at 2^32-1), every other
byte must stay, and a pair split over two entries or with another register must be left alone."""
import os, re, struct, subprocess, sys, tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[1]
patch = (root / 'patches/xnu/zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-text-fixup-uptime.patch').read_text()
added = '\n'.join(l[1:] for l in patch.split('\n') if l.startswith('+') and not l.startswith('+++'))
block = added[added.index('uint32_t cs_text_fixup_rebased;'):added.index('STARTUP(EARLY_BOOT, STARTUP_RANK_LAST')]
gen = subprocess.run([sys.executable, '-I', str(root / 'scripts/gen-guamp-lockmin-fixup.py'), '--limit', '180'],
                     capture_output=True, text=True, check=True).stdout.split()
entries = [e.split(',') for e in gen if e.startswith('dyld_shared_cache_arm64.25,')]
assert len(entries) == 4

def carr(b): return '{' + ','.join('0x%02x' % x for x in b) + '}'
rows = []
for name, off, orig, repl in entries:
    rows.append('{"%s", 0x%s, %s, %s, 16}' % (name, off, carr(bytes.fromhex(orig)), carr(bytes.fromhex(repl))))
# a split pair (movz at the end of one entry, the mark at the start of the next) and a pair with another register
movz9 = struct.pack('<I', 0xd2800000 | (5 << 5) | 9); mark9 = struct.pack('<I', 0xf2a00000 | (0xb007 << 5) | 9)
mark8 = struct.pack('<I', 0xf2a00000 | (0xb007 << 5) | 8)
rows.append('{"x", 0, {0}, %s, 16}' % carr(b'\0' * 12 + movz9))
rows.append('{"x", 16, {0}, %s, 16}' % carr(mark9 + b'\0' * 12))
rows.append('{"y", 0, {0}, %s, 16}' % carr(movz9 + mark8 + b'\0' * 8))
prog = r'''#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define NSEC_PER_SEC 1000000000ull
#define CS_TEXT_FIXUP_BYTES 16
struct cs_text_fixup { char name[64]; uint64_t offset; uint8_t original[CS_TEXT_FIXUP_BYTES]; uint8_t replacement[CS_TEXT_FIXUP_BYTES]; uint32_t length; };
static struct cs_text_fixup cs_text_fixups[] = {''' + ',\n'.join(rows) + r'''};
static uint32_t cs_text_fixup_count = sizeof(cs_text_fixups) / sizeof(cs_text_fixups[0]);
static uint64_t now_ns;
static uint64_t mach_absolute_time(void) { return now_ns; }
static void absolutetime_to_nanoseconds(uint64_t t, uint64_t *ns) { *ns = t; }
''' + block + r'''
int main(int argc, char **argv) {
	if (argc < 2) return 2;
	now_ns = strtoull(argv[1], 0, 0);
	cs_text_fixup_boot_rebase();
	fwrite(cs_text_fixups, 1, sizeof(cs_text_fixups), stderr);
	return 0;
}
'''
work = tempfile.mkdtemp()
open(os.path.join(work, 't.c'), 'w').write(prog)
exe = os.path.join(work, 't')
subprocess.check_call(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-format', '-fsanitize=address,undefined',
                       os.path.join(work, 't.c'), '-o', exe])
size = 64 + 8 + 16 + 16 + 4 + 4   # struct with tail padding
for boot_s, want in ((2, 182), (570, 750), (4294967200, 0xffffffff)):
    r = subprocess.run([exe, str(boot_s * 10**9)], capture_output=True, check=True)
    raw = r.stderr
    assert len(raw) == size * 7, len(raw)
    reps = [raw[i * size + 88:i * size + 104] for i in range(7)]
    new = b''.join(reps[:4])
    old = b''.join(bytes.fromhex(e[3]) for e in entries)
    w4, w5 = struct.unpack_from('<2I', new, 16)
    got = ((w4 >> 5) & 0xffff) | (((w5 >> 5) & 0xffff) << 16)
    assert w4 & 0xffe0001f == 0xd2800009 and w5 & 0xffe0001f == 0xf2a00009, (hex(w4), hex(w5))
    assert got == want, (boot_s, got, want)
    assert new[:16] == old[:16] and new[24:] == old[24:], 'other bytes changed'
    assert reps[4] == b'\0' * 12 + movz9 and reps[5] == mark9 + b'\0' * 12, 'split pair rewritten'
    assert reps[6] == movz9 + mark8 + b'\0' * 8, 'pair with another register rewritten'
    print('timebase at %d s: limit -> %d s; %s' % (boot_s, got, r.stdout.decode().strip().split('\n')[-1]))
print('text-fixup uptime: PASS')
