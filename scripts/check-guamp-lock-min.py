#!/usr/bin/env python3
"""Host test of guamp-lock-min (patches/xnu/zzzz-guamp-setup-skip.patch, 05/11).

The kernel writes com.apple.springboard.plist (native_guamp_pref3) at every boot; since 05/11 it carries
SBMinimumLockscreenIdleTime as a 2-byte integer whose value bytes native_guamp_setup_skip() sets from guamp-lock-min.
The added C of the patch (the array and the guamp-lock-min block) is compiled on the host with stand-ins for
PE_parse_boot_argn and printf; for each boot argument the bytes it leaves must be a valid binary plist with the asked
floor (default 60, at most 600, 0 = none) and the other keys unchanged."""
import os, plistlib, re, subprocess, sys, tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[1]
patch = (root / 'patches/xnu/zzzz-guamp-setup-skip.patch').read_text()
src = '\n'.join(l[1:] for l in patch.split('\n') if l.startswith('+') and not l.startswith('+++'))
at = int(re.search(r'#define NATIVE_GUAMP_PREF3_LOCK_MIN (\d+)', src).group(1))
m = re.search(r'static uint8_t native_guamp_pref3\[(\d+)\] = \{([^}]*)\};', src)
data = bytes(int(x, 16) for x in m.group(2).split(','))
assert len(data) == int(m.group(1)) and data[at - 1] == 0x11, 'value bytes not after an int16 marker'
assert src.count('native_guamp_pref3') >= 6
array = src[src.index('#define NATIVE_GUAMP_PREF3_LOCK_MIN'):src.index('};', src.index('static uint8_t native_guamp_pref3')) + 2]
s = src.index('\tunsigned int guamp_lock_min = 60;')
e = src.index('\t}\n', src.index('if (native_guamp_pref3[NATIVE_GUAMP_PREF3_LOCK_MIN - 1] == 0x11)')) + 3
prog = r'''#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned arg;
static int has;
static int PE_parse_boot_argn(const char *n, void *p, int s) { (void)s; if (strcmp(n, "guamp-lock-min") || !has) return 0; memcpy(p, &arg, 4); return 1; }
''' + array + '''
int main(int argc, char **argv) {
	if (argc > 1) { has = 1; arg = (unsigned)strtoul(argv[1], 0, 0); }
''' + src[s:e] + '''
	fwrite(native_guamp_pref3, 1, sizeof(native_guamp_pref3), stderr);
	return 0;
}
'''
work = tempfile.mkdtemp()
open(os.path.join(work, 't.c'), 'w').write(prog)
exe = os.path.join(work, 't')
subprocess.check_call(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined', os.path.join(work, 't.c'), '-o', exe])
base = None
for arg, want in ((None, 60), ('0', 0), ('45', 45), ('600', 600), ('601', 600), ('100000', 600)):
    r = subprocess.run([exe] + ([arg] if arg else []), capture_output=True, check=True)
    v = plistlib.loads(r.stderr)
    assert v.pop('SBMinimumLockscreenIdleTime') == want, (arg, want)
    assert base is None or v == base
    base = v
    print('guamp-lock-min=%s: %s' % (arg if arg else '(default)', r.stdout.decode().strip()))
assert base == {'BKNoWatchdogs': True, 'FBLaunchWatchdogExceptions': {'com.apple.purplebuddy': 100.0},
                'MCForceShowCloudConfigurationUI': False}, base
print('guamp-lock-min: PASS')
