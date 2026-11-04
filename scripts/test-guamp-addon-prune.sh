#!/bin/bash
# Host test of the pure part of patches/xnu/zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-addon-prune.patch (the addon name -> Data path
# check and the ".guamp-keep" list) and of the keep entry written by the addon scripts. Needs cc and python3; no device, no kernel.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PATCH=$ROOT/patches/xnu/zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-addon-prune.patch
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
sed -n '/guamp-addon-prune pure BEGIN/,/guamp-addon-prune pure END/p' "$PATCH" | sed 's/^+//' > "$TMP/pure.inc"
[ "$(wc -l < "$TMP/pure.inc")" -gt 20 ] || { echo "pure block not found in $PATCH"; exit 1; }
cat > "$TMP/test.c" <<'EOF'
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define NATIVE_GPU_KEEP_PREFIXES 16
struct native_gpu_manifest_record { char name[160]; uint64_t hash; uint64_t size; uint32_t mode; uint32_t used; };
#include "pure.inc"
static int failures;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)
static struct native_gpu_manifest_record records[8];
static uint32_t add(const char *name, uint32_t used) { static uint32_t n; strcpy(records[n].name, name); records[n].used = used; return n++; }
static int mark(const char *text, uint8_t *seen) { char buf[2048]; size_t n = strlen(text); memcpy(buf, text, n); memset(seen, 0, 8);
	return native_gpu_keep_mark(buf, n, records, 8, seen); }
int main(void)
{
	char path[192];
	CHECK(!native_gpu_addon_target("addons/NativeAdrenoMetal.bundle/libvulkan_freedreno.dylib", path, sizeof(path)) &&
	    !strcmp(path, "/private/var/ngpu/addons/NativeAdrenoMetal.bundle/libvulkan_freedreno.dylib"));
	CHECK(!native_gpu_addon_target("@mp/com.apple.coreanimation.plist", path, sizeof(path)) &&
	    !strcmp(path, "/private/var/Managed Preferences/mobile/com.apple.coreanimation.plist"));
	const char *bad[] = {"", "/etc/passwd", "../x", "a/../../b", "a/..", "a//b", "dir/", "@mp/", "a..b"};
	for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		CHECK(native_gpu_addon_target(bad[i], path, sizeof(path)) == EINVAL);
	}
	CHECK(native_gpu_addon_target(".guamp-manifest", path, sizeof(path)) == EPERM);
	char tiny[20];
	CHECK(native_gpu_addon_target("addons/long-name", tiny, sizeof(tiny)) == ENAMETOOLONG);

	uint32_t dylib = add("addons/NativeAdrenoMetal.bundle/libvulkan_freedreno.dylib", 1);
	uint32_t plugin = add("addons/NativeAdrenoMetal.bundle/NativeAdrenoMetal", 1);
	uint32_t bb = add("baseband/standalone/Baseband/Mav16/a.bin", 1);
	uint32_t old = add("iphone-res/old.car", 0);
	uint32_t mp = add("@mp/com.apple.coreanimation.plist", 1);
	uint8_t seen[8];
	CHECK(mark("NGPUKEEP1\naddons/NativeAdrenoMetal.bundle/NativeAdrenoMetal\n@mp/com.apple.coreanimation.plist\nbaseband/\n", seen) == 3);
	CHECK(!seen[dylib] && seen[plugin] && seen[bb] && !seen[old] && seen[mp]);
	CHECK(mark("NGPUKEEP1\n", seen) == -1);                       /* empty list: prune nothing */
	CHECK(mark("NGPUKEEP0\nbaseband/\n", seen) == -1);            /* wrong magic */
	CHECK(mark("NGPUKEEP1\nbaseband/", seen) == -1);              /* last line without newline */
	char nul[] = "NGPUKEEP1\nab\0c\n";
	memset(seen, 0, 8);
	CHECK(native_gpu_keep_mark(nul, sizeof(nul) - 1, records, 8, seen) == -1);
	char many[1024] = "NGPUKEEP1\n";
	for (int i = 0; i < 17; i++) sprintf(many + strlen(many), "p%d/\n", i);
	CHECK(mark(many, seen) == -1);                                 /* too many prefixes */
	CHECK(mark("NGPUKEEP1\n\naddons/\n", seen) == 1 && seen[dylib] && seen[plugin] && !seen[bb]);
	printf(failures ? "addon prune: %d failures\n" : "addon prune: pure checks ok\n", failures);
	return failures != 0;
}
EOF
cc -std=gnu11 -Wall -Wextra -Werror -Wno-unused-function -I "$TMP" -o "$TMP/test" "$TMP/test.c"
"$TMP/test"

# The scripts: a complete addon carries .guamp-keep with every entry plus baseband/, the delta carries the complete list.
python3 - "$ROOT" "$TMP" <<'EOF'
import struct, subprocess, sys, zlib
from pathlib import Path
root, tmp = Path(sys.argv[1]), Path(sys.argv[2])
MAGIC = b'NGPUADD2'
def pad(b): return b + bytes(-len(b) % 8)
def write(path, files):
    body = b''
    for name, mode, data in files:
        enc = name.encode() + b'\0'; packed = zlib.compress(data, 9)
        body += struct.pack('<IIQQ', len(enc), mode, len(data), len(packed)) + pad(enc) + pad(packed)
    path.write_bytes(MAGIC + struct.pack('<IIQ', len(files), 0, 24 + len(body)) + body)
def read(path):
    blob = path.read_bytes(); count, _, total = struct.unpack_from('<IIQ', blob, 8); assert total == len(blob)
    at, out = 24, {}
    for _ in range(count):
        nb, mode, size, packed = struct.unpack_from('<IIQQ', blob, at); at += 24
        name = blob[at:at + nb - 1].decode(); at += (nb + 7) & ~7
        out[name] = (mode, zlib.decompress(blob[at:at + packed])); at += (packed + 7) & ~7
    return out
old = tmp / 'old.gpuaddon'; new = tmp / 'new.gpuaddon'; delta = tmp / 'delta.gpuaddon'
write(old, [('iphone-res/a.car', 0o644, b'a'), ('addons/NativeAdrenoMetal.bundle/libvulkan_freedreno.dylib', 0o755, b'v'),
            ('.guamp-keep', 0o644, b'NGPUKEEP1\nstale\n')])
write(new, [('iphone-res/a.car', 0o644, b'a'), ('iphone-res/b.car', 0o644, b'b')])
subprocess.run([sys.executable, str(root / 'scripts/compose-guamp-delta-addon.py'), '--old', str(old), '--new', str(new),
                '--out', str(delta)], check=True, stdout=subprocess.DEVNULL)
entries = read(delta)
assert set(entries) == {'iphone-res/b.car', '.guamp-keep'}, entries.keys()
keep = entries['.guamp-keep'][1].decode().split('\n')
assert keep[0] == 'NGPUKEEP1' and keep[-1] == '' and set(keep[1:-1]) == {'.guamp-keep', 'iphone-res/a.car', 'iphone-res/b.car', 'baseband/'}, keep
nokeep = tmp / 'nokeep.gpuaddon'
subprocess.run([sys.executable, str(root / 'scripts/compose-guamp-delta-addon.py'), '--old', str(old), '--new', str(new),
                '--out', str(nokeep), '--no-keep'], check=True, stdout=subprocess.DEVNULL)
assert set(read(nokeep)) == {'iphone-res/b.car'}
print('addon prune: delta keep list ok')
EOF
