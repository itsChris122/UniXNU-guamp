#!/usr/bin/env python3
"""Host test of BlueTool's override script names (05/11, fin49 "Bluetooth not available").

patches/xnu/zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-bt-tool-model.patch makes native_guamp_bt_tool write the scripts for
J121AP too (the baseband ADT puts J121AP first in the root compatible, MobileGestalt's HWModelStr) and for the first root
compatible string of the ADT that booted. Its added block is compiled on the host with stand-ins for the device tree calls
and run on the compatible strings of the ADTs in use: the model list must hold the HWModel BlueTool asks for, without
duplicates, and a missing or odd compatible must not add an empty name or a path."""
import os, subprocess, sys, tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[1]
patch = (root / 'patches/xnu/zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-bt-tool-model.patch').read_text()
added = '\n'.join(l[1:] for l in patch.split('\n') if l.startswith('+') and not l.startswith('+++'))
block = added[added.index('\tconst char *models[5]'):]
block = block[:block.index('\n\t}\n') + 4]
assert 'J121AP' in block and 'SecureDTGetProperty' in block
assert '&& models[m]; m++)' in patch, 'the script loop must stop at the first empty slot'

prog = r'''#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define kSuccess 0
#define kError (-1)
typedef const void *DTEntry;
static const char *dt_value; static unsigned int dt_size; static int dt_ok;
static int SecureDTLookupEntry(DTEntry s, const char *p, DTEntry *e) { (void)s; *e = p; return strcmp(p, "/") ? kError : kSuccess; }
static int SecureDTGetProperty(DTEntry e, const char *n, void const **v, unsigned int *z) {
	(void)e; if (!dt_ok || strcmp(n, "compatible")) return kError; *v = dt_value; *z = dt_size; return kSuccess; }
static void run(void) {
''' + block + r'''
	for (size_t m = 0; m < 5 && models[m]; m++) printf("%s|", models[m]);
	printf("\n");
}
int main(int argc, char **argv) {
	if (argc < 2) return 2;
	dt_ok = strcmp(argv[1], "-");
	static char buf[256]; size_t n = 0;
	for (const char *c = argv[1]; *c && n < sizeof buf; c++) buf[n++] = *c == ';' ? 0 : *c;
	dt_value = buf; dt_size = argc > 2 ? (unsigned)atoi(argv[2]) : (unsigned)n + 1;
	run();
	return 0;
}
'''
work = tempfile.mkdtemp()
src = os.path.join(work, 't.c'); exe = os.path.join(work, 't')
open(src, 'w').write(prog)
subprocess.check_call(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined', src, '-o', exe])

def models(*args):
    out = subprocess.run([exe, *args], capture_output=True, text=True, check=True).stdout
    return out.split('\n')[-2].rstrip('|').split('|')

base = ['motorola,guamp', 'D22AP', 'J120AP', 'J121AP']
cases = [
    (('J121AP;motorola,guamp;qcom,bengal;qcom,sm6115',), base),        # fin49 baseband ADT
    (('motorola,guamp;qcom,bengal;qcom,sm6115',), base),               # earlier ADTs
    (('J320xAP;motorola,guamp',), base + ['J320xAP']),                 # a later ADT: its model is added
    (('-',), base),                                                    # no compatible
    (('',), base),                                                     # empty first string
    (('ab/../x;y',), base + ['ab']),                                   # never a path
    (('X' * 100,), base + ['X' * 40]),                                 # bounded copy
    (('K9AP', '2'), base + ['K9']),                                    # unterminated property: size wins
]
bad = 0
for args, want in cases:
    got = models(*args)
    ok = got == want
    bad += not ok
    print('%s %-40.40s -> %s' % ('ok  ' if ok else 'FAIL', args[0], '|'.join(got)))
print('bt-tool model: %d/%d ok' % (len(cases) - bad, len(cases)))
sys.exit(1 if bad else 0)
