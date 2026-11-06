#!/usr/bin/env python3
"""Host test of make-guamp-baseband-addon.py --model-files and of the view list reader of
patches/xnu/zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-model-files.patch (06/11).

The script runs on a small fake 21H461 root (carrier bundles with J121, J121_J208 and J208 overlays): each J121 overlay must become
overrides_<NAME>.plist without the Mav16/J121 hardware keys, a bundle left empty must be skipped, nothing may carry the J121 name, the
view list must name exactly the bundles with a file, and a rebuilt .guamp-keep must keep model-files/. The kernel's added block
(native_guamp_model_views_parse and native_guamp_model_files_mount) is compiled on the host with stand-ins for the vnode calls and the
bindfs mount, fed the script's own list and malformed ones. With GUAMP_ROOTFS=<mounted 21H461 OS volume> the script also runs on the
real carrier bundles (Default.bundle and --carrier all). Needs cc and python3; no device, no kernel."""
import importlib.util, os, plistlib, struct, subprocess, sys, tempfile, zlib
from pathlib import Path

root = Path(__file__).resolve().parents[1]
script = root / 'scripts/make-guamp-baseband-addon.py'
patch = (root / 'patches/xnu/zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-model-files.patch').read_text()
spec = importlib.util.spec_from_file_location('addon', script)
addon = importlib.util.module_from_spec(spec)
spec.loader.exec_module(addon)
failures = 0


def check(cond, what):
    global failures
    if not cond:
        print('FAIL', what)
        failures += 1


work = Path(tempfile.mkdtemp())
fake = work / 'root'
bundles = fake / 'System/Library/Carrier Bundles/iPad'
HW = {'DerPriFileName': 'overrides_J121.der.pri', 'DerPriFileVersion': '0.1.146', 'GRIFileName': 'global_setting_A.gri',
      'GRIVersion': '10.2.0', 'OnBodyThresholdParametersFile': 'com.apple.bodythreshold_D.plist'}
layout = {
    'Default.bundle': {'overrides_J121.plist': dict(HW, Show4GSwitch=True, Enable4GByDefault=True, ShowLTEWarningUnsupportedCarrier=False),
                       'overrides_J208.plist': {'Show4GSwitch': False}},
    'Wind_Very_it.bundle': {'overrides_J121_J208.plist': {'DerPriFileName': 'overrides_J121_J208.der.pri', 'DerPriFileVersion': '1'}},
    'Verizon_LTE_US.bundle': {'overrides_J121_J208.plist': {'EFSFiles': [{'carrier.ims': '/shared/qpconfig.xml'}],
                                                            'apns': [{'apn': 'vzwinternet', 'type-mask': 49}], 'OTAActivation': {'x': False}}},
    'Only_J208.bundle': {'overrides_J208.plist': {'Show4GSwitch': True}},
    'No_Overlay.bundle': {},
}
for bundle, files in layout.items():
    (bundles / bundle / 'signatures').mkdir(parents=True)
    (bundles / bundle / 'Info.plist').write_bytes(plistlib.dumps({'CFBundleIdentifier': bundle}))
    for name, content in files.items():
        (bundles / bundle / name).write_bytes(plistlib.dumps(content, fmt=plistlib.FMT_BINARY))
        (bundles / bundle / 'signatures' / name).write_bytes(plistlib.dumps({'CBSignature3': b'\0' * 8}))


def run(*args, ok=True):
    out = work / f'out{run.n}.gpuaddon'
    run.n += 1
    result = subprocess.run([sys.executable, str(script), *args, '--out', str(out)], capture_output=True, text=True)
    if ok and result.returncode:
        print(result.stdout, result.stderr)
    check((result.returncode == 0) == ok, f'exit {result.returncode} for {args}')
    return (addon.read_addon(out) if ok and out.exists() else None), result


run.n = 0
entries, _ = run('--model-files', 'guamp', '--rootfs', str(fake))
names = {e[0]: zlib.decompress(e[3]) for e in entries}
point = '/System/Library/Carrier Bundles/iPad/Default.bundle'
check(sorted(names) == [f'model-files{point}/overrides_guamp.plist', 'model-files/views'], f'default entries {sorted(names)}')
default = plistlib.loads(names[f'model-files{point}/overrides_guamp.plist'])
check(default == {'Show4GSwitch': True, 'Enable4GByDefault': True, 'ShowLTEWarningUnsupportedCarrier': False}, f'Default {default}')
check(names['model-files/views'] == f'NGPUVIEWS1\n{point}|overrides_guamp.plist\n'.encode(), 'default view list')
check(all(e[1] == 0o644 and e[2] == len(names[e[0]]) for e in entries), 'modes and sizes')

entries, result = run('--model-files', 'motorola,guamp', '--rootfs', str(fake), '--carrier', 'all')
names = {e[0]: zlib.decompress(e[3]) for e in entries}
expect = ['Default.bundle', 'Verizon_LTE_US.bundle']
check(sorted(names) == sorted([f'model-files/System/Library/Carrier Bundles/iPad/{b}/overrides_motorola,guamp.plist' for b in expect]
                              + ['model-files/views']), f'all entries {sorted(names)}')
verizon = plistlib.loads(names['model-files/System/Library/Carrier Bundles/iPad/Verizon_LTE_US.bundle/overrides_motorola,guamp.plist'])
check(verizon == {'apns': [{'apn': 'vzwinternet', 'type-mask': 49}], 'OTAActivation': {'x': False}}, f'Verizon {verizon}')
views_all = names['model-files/views']
check(views_all.decode().splitlines()[1:] == [f'/System/Library/Carrier Bundles/iPad/{b}|overrides_motorola,guamp.plist' for b in expect],
      'all view list')
check(all(b'J121' not in name.encode() and b'J121' not in data for name, data in names.items()), 'nothing named or holding J121')
check('Wind_Very_it.bundle' in result.stdout and 'skipped' in result.stdout, 'report names the skipped bundle')

for bad in (['--model-files', 'J121'], ['--model-files', 'a_b'], ['--model-files', 'x/y'], ['--model-files', ''],
            ['--model-files', 'guamp'], ['--model-files', 'guamp', '--rootfs', str(work)],
            ['--model-files', 'guamp', '--rootfs', str(fake), '--carrier', 'Missing.bundle'],
            ['--carrier', 'Default.bundle', '--rootfs', str(fake)], ['--stub', 'qdsp6sw.mbn', '--model-files', 'guamp', '--rootfs', str(fake)], []):
    run(*bad, ok=False)

# --base with a keep list: the model files join it and model-files/ becomes a kept prefix.
keep = b'NGPUKEEP1\nold/file\naddons/\n'
body = b''
for name, data in (('old/file', b'x'), ('.guamp-keep', keep)):
    packed = zlib.compress(data)
    body += struct.pack('<IIQQ', len(name) + 1, 0o644, len(data), len(packed)) + addon.pad(name.encode() + b'\0') + addon.pad(packed)
base = work / 'base.gpuaddon'
base.write_bytes(addon.MAGIC + struct.pack('<IIQ', 2, 0, 24 + len(body)) + body)
entries, _ = run('--model-files', 'guamp', '--rootfs', str(fake), '--base', str(base))
names = {e[0]: zlib.decompress(e[3]) for e in entries}
lines = names['.guamp-keep'].decode().split('\n')
check(lines[0] == 'NGPUKEEP1' and f'model-files{point}/overrides_guamp.plist' in lines and 'model-files/views' in lines
      and 'old/file' in lines and 'model-files/' in lines and 'addons/' in lines, f'keep list {lines}')
entries, _ = run('--model-files', 'guamp', '--rootfs', str(fake), '--base', str(work / 'out0.gpuaddon'), ok=False)

real = os.environ.get('GUAMP_ROOTFS')
if real:
    entries, _ = run('--model-files', 'guamp', '--rootfs', real)
    names = {e[0]: zlib.decompress(e[3]) for e in entries}
    default = plistlib.loads(names[f'model-files{point}/overrides_guamp.plist'])
    check(not set(default) & set(addon.HARDWARE_KEYS) and 'Show4GSwitch' in default, f'real Default {default}')
    entries, _ = run('--model-files', 'guamp', '--rootfs', real, '--carrier', 'all')
    real_views = zlib.decompress([e for e in entries if e[0] == 'model-files/views'][0][3])
    check(real_views.count(b'\n') - 1 == len(entries) - 1 and len(entries) - 1 <= 128, f'real views {len(entries) - 1}')
    print('real 21H461 root: Default.bundle keeps', sorted(default), '; --carrier all:', len(entries) - 1, 'bundles')
    views_all = real_views

# The kernel's block on the host.
added = '\n'.join(line[1:] for line in patch.split('\n') if line.startswith('+') and not line.startswith('+++'))
block = added[added.index('/* guamp-model-files pure BEGIN */'):]
block = block[:block.index('\n}\n', block.index('native_guamp_model_files_mount(proc_t p')) + 3]
check('native_gpu_addon_mount(p, scratch, 0x400' in block and 'guamp-model-files' in block, 'kernel block found')
check('native_guamp_model_files_mount(p, scratch);' in patch, 'the block is called from native_gpu_addon_prepare')
prog = r'''#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef int proc_t; typedef uint64_t user_addr_t; typedef FILE *vnode_t; typedef int vfs_context_t; typedef char *caddr_t;
typedef size_t vm_size_t;
#define NULLVP NULL
#define FREAD 1
#define O_NOFOLLOW 2
#define UIO_READ 0
#define UIO_SYSSPACE 0
#define IO_NOAUTH 0
#define Z_WAITOK 0
struct vnode_attr { uint64_t va_data_size; };
#define VATTR_INIT(v) ((v)->va_data_size = 0)
#define VATTR_WANTED(v, f) ((void)0)
static const char *list_file; static uint32_t boot_arg = 1; static int have_arg; static int mounts, mount_stage = 5, allocs;
static char mounted[200][256];
static int PE_parse_boot_argn(const char *n, void *v, int s) { (void)s; if (!have_arg || strcmp(n, "guamp-model-files")) return 0;
	*(uint32_t *)v = boot_arg; return 1; }
static vfs_context_t vfs_context_current(void) { return 0; }
static int vfs_context_ucred(vfs_context_t c) { return c; }
static int vfs_context_proc(vfs_context_t c) { return c; }
static int vnode_open(const char *path, int f, int m, int x, vnode_t *vp, vfs_context_t c) { (void)f; (void)m; (void)x; (void)c;
	if (strcmp(path, "/private/var/ngpu/model-files/views")) { return EINVAL; }
	if (!list_file) { return ENOENT; }
	*vp = fopen(list_file, "rb"); return *vp ? 0 : ENOENT; }
static int vnode_getattr(vnode_t vp, struct vnode_attr *va, vfs_context_t c) { (void)c; fseek(vp, 0, SEEK_END);
	va->va_data_size = (uint64_t)ftell(vp); return 0; }
static int vn_rdwr(int rw, vnode_t vp, caddr_t b, int n, long off, int seg, int fl, int cr, int *resid, int pr) {
	(void)rw; (void)seg; (void)fl; (void)cr; (void)pr; fseek(vp, off, SEEK_SET); *resid = n - (int)fread(b, 1, (size_t)n, vp); return 0; }
static int vnode_close(vnode_t vp, int f, vfs_context_t c) { (void)f; (void)c; fclose(vp); return 0; }
static void *kalloc_data(vm_size_t n, int f) { (void)f; allocs++; return malloc(n); }
static void kfree_data(void *p, size_t n) { (void)n; allocs--; free(p); }
static char *strnstr(const char *s, const char *f, size_t n) { size_t l = strlen(f);  /* XNU has it, glibc does not */
	for (; n >= l && *s; s++, n--) { if (!strncmp(s, f, l)) { return (char *)s; } } return NULL; }
static void native_gpu_addon_mount(proc_t p, user_addr_t s, user_addr_t o, const char *source, size_t source_size, const char *point,
    size_t point_size, const char *visible, const char *own, volatile int *stage, volatile int *status) {
	(void)p; (void)s; (void)o;
	if (source_size != strlen(source) + 1 || point_size != strlen(point) + 1 || strcmp(visible, point) || point_size > 0xe0) abort();
	snprintf(mounted[mounts++ % 200], 256, "%s>%s>%s", source, point, own); *stage = mount_stage; *status = 0; }
''' + block + r'''
int main(int argc, char **argv)
{
	list_file = argc > 1 && strcmp(argv[1], "-") ? argv[1] : NULL;
	if (argc > 2) { have_arg = 1; boot_arg = (uint32_t)atoi(argv[2]); }
	if (argc > 3) mount_stage = atoi(argv[3]);
	native_guamp_model_files_mount(0, 0);
	for (int i = 0; i < mounts; i++) printf("M %s\n", mounted[i]);
	printf("R %d %d %d\n", native_guamp_model_files_views, native_guamp_model_files_mounted, allocs);
	return 0;
}
'''
src, exe = work / 't.c', work / 't'
src.write_text(prog)
subprocess.check_call(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function', '-fsanitize=address,undefined',
                       str(src), '-o', str(exe)])


def kernel(content, *args):
    path = '-'
    if content is not None:
        path = str(work / f'views{kernel.n}')
        kernel.n += 1
        Path(path).write_bytes(content)
    out = subprocess.run([str(exe), path, *args], capture_output=True, text=True)
    check(out.returncode == 0, f'kernel stand-in crashed: {out.stderr[-400:]}')
    lines = out.stdout.splitlines()
    return [l[2:] for l in lines if l.startswith('M ')], lines[-1].split()[1:], out.stdout


kernel.n = 0
views = views_all.decode().splitlines()[1:]
mounts, result, _ = kernel(views_all)
check(result == [str(len(views)), str(len(views)), '0'], f'script list: {result}')
check(mounts == ['/private/var/ngpu/model-files%s>%s>%s/%s' % (v.split('|')[0], v.split('|')[0], v.split('|')[0], v.split('|')[1])
                 for v in views], f'mounts {mounts[:2]}')
mounts, result, text = kernel(views_all, '1', '3')
check(result[:2] == [str(len(views)), '0'] and 'first failure /System/' in text and 'stage 3' in text, f'failed mounts: {text[-200:]}')
mounts, result, _ = kernel(views_all, '0')
check(mounts == [] and result == ['-1', '0', '0'], 'guamp-model-files=0 skips')
mounts, result, _ = kernel(None)
check(mounts == [] and result == ['-1', '0', '0'], 'no list, nothing mounted')
ok = b'NGPUVIEWS1\n/System/Library/Carrier Bundles/iPad/Default.bundle|overrides_guamp.plist\n'
bad = [b'', b'NGPUVIEWS1\n', b'NGPUVIEWS2\n/System/a|b\n', ok[:-1], ok.replace(b'|', b''), ok.replace(b'|', b'||'),
       ok.replace(b'/System/', b'/private/'), ok.replace(b'Default.bundle', b'../x'), ok.replace(b'iPad/', b'iPad//'),
       ok.replace(b'Default.bundle|', b'Default.bundle/|'), ok.replace(b'overrides_guamp.plist', b'a/b'),
       ok.replace(b'overrides_guamp.plist', b'..'), ok.replace(b'overrides_guamp.plist', b''), ok.replace(b'|', b'\0|'),
       ok.replace(b'Default.bundle', b'D' * 200), ok.replace(b'overrides_guamp.plist', b'o' * 64), b'NGPUVIEWS1\n/System/|x\n',
       b'NGPUVIEWS1\n/Applications/|x\n', b'NGPUVIEWS1\n/Applications|x\n', b'NGPUVIEWS1\n/ApplicationsX/a|x\n',
       b'NGPUVIEWS1\n/Applications/a/../b|x\n', b'NGPUVIEWS1\n/Applications/a.app/|x\n',
       b'NGPUVIEWS1\n' + b'/System/x|y\n' * 129, b'NGPUVIEWS1\n' + b'/System/x|y\n' * 1400]
for content in bad:
    mounts, result, _ = kernel(content)
    check(mounts == [] and result == ['-1', '0', '0'], f'malformed list mounted something: {content[:60]!r} -> {result}')
mounts, result, _ = kernel(b'NGPUVIEWS1\n' + b'/System/x|y\n' * 128)
check(len(mounts) == 128 and result == ['128', '128', '0'], f'128 views: {result}')
mounts, result, _ = kernel(ok)
check(mounts == ['/private/var/ngpu/model-files/System/Library/Carrier Bundles/iPad/Default.bundle>/System/Library/Carrier Bundles/'
                 'iPad/Default.bundle>/System/Library/Carrier Bundles/iPad/Default.bundle/overrides_guamp.plist'], f'one view {mounts}')

app = b'NGPUVIEWS1\n/Applications/SIMSetupUIService.app|Localizable.loctable\n'
mounts, result, _ = kernel(app)
check(mounts == ['/private/var/ngpu/model-files/Applications/SIMSetupUIService.app>/Applications/SIMSetupUIService.app>'
                 '/Applications/SIMSetupUIService.app/Localizable.loctable'] and result[:2] == ['1', '1'], f'app view {mounts} {result}')

print('check-guamp-model-files:', 'FAIL' if failures else 'PASS', f'({failures} failures)')
sys.exit(1 if failures else 0)
