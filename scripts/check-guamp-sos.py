#!/usr/bin/env python3
"""Host test of guamp-sos (patches/xnu/zzzz-guamp-setup-skip.patch, 07/11, side button + volume held).

The kernel writes /private/var/mobile/Library/Preferences/com.apple.SOS.plist at every boot. SOS.framework's
+[SOSManager deviceSupportsSOS] returns SOSDeviceSupportedKey of the com.apple.SOS user defaults when present (else the
MobileGestalt answer); with false SpringBoard keeps the shutdown recognizer (side + volume long press -> power down slider)
instead of the SOS "claw", whose com.apple.sos Mach service no job of the iPadOS root provides. The added C (the two
arrays, the guamp-sos argument and the block that picks the data) is compiled on the host with stand-ins for
PE_parse_boot_argn and printf; for each boot argument the bytes written must be the expected binary plist."""
import os, plistlib, re, subprocess, sys, tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[1]
patch = (root / 'patches/xnu/zzzz-guamp-setup-skip.patch').read_text()
src = '\n'.join(l[1:] for l in patch.split('\n') if l.startswith('+') and not l.startswith('+++'))
arrays = ''
for name in ('native_guamp_sosprefs', 'native_guamp_sosprefs_stock'):
    m = re.search(r'static const uint8_t %s\[(\d+)\] = \{([^}]*)\};' % name, src)
    data = bytes(int(x, 16) for x in m.group(2).split(','))
    assert len(data) == int(m.group(1)), name
    arrays += m.group(0) + '\n'
row = '{"/private/var/mobile/Library/Preferences/com.apple.SOS.plist", native_guamp_sosprefs, sizeof(native_guamp_sosprefs), 501, 0600, 1},'
assert src.count(row) == 1, 'table row missing'
assert src.index(row) < src.index('} native_guamp_setup_files[] = {') + 2000
a = src.index('\tunsigned int guamp_sos = 0;')
arg = src[a:src.index('\n', src.index('PE_parse_boot_argn("guamp-sos"', a)) + 1]
b = src.index('\t\tif (data == native_guamp_sosprefs) {')
block = src[b:src.index('\t\t}\n', src.index('printf("guamp-sos=', b)) + 4]
assert src.index(arg) < b, 'guamp-sos parsed after its use'
prog = r'''#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned value;
static int has;
static int PE_parse_boot_argn(const char *n, void *p, int s) { (void)s; if (strcmp(n, "guamp-sos") || !has) return 0; memcpy(p, &value, 4); return 1; }
''' + arrays + '''
int main(int argc, char **argv) {
	if (argc > 1) { has = 1; value = (unsigned)strtoul(argv[1], 0, 0); }
''' + arg + '''
	const uint8_t *data = native_guamp_sosprefs;
	size_t size = sizeof(native_guamp_sosprefs);
''' + block + '''
	fwrite(data, 1, size, stderr);
	return 0;
}
'''
work = tempfile.mkdtemp()
open(os.path.join(work, 't.c'), 'w').write(prog)
exe = os.path.join(work, 't')
subprocess.check_call(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined', os.path.join(work, 't.c'), '-o', exe])
for a, want in ((None, {'SOSDeviceSupportedKey': False}), ('0', {'SOSDeviceSupportedKey': False}), ('1', {}),
                ('2', {'SOSDeviceSupportedKey': False})):
    r = subprocess.run([exe] + ([a] if a else []), capture_output=True, check=True)
    got = plistlib.loads(r.stderr)
    assert got == want, (a, got)
    print('guamp-sos=%s: %s -> %s' % (a if a else '(default)', r.stdout.decode().strip(), got))
print('guamp-sos: PASS')
