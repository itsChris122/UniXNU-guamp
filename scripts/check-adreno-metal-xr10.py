#!/usr/bin/env python3
"""Host check of patches/adreno-metal/0021-xr10-surface-imports.patch (WSL or macOS, clang++ or g++).

The helpers the patch adds to NativeBridgeMetalFrontend.mm (xr10SurfaceFormat, isXr10PixelFormat, fourccText) are taken from the
patch's added lines and compiled with a stand-in NSUInteger: every fourcc maps to the Metal format it means (planes and plane
included), nothing else maps, and the log text of a fourcc is printable. The call site is checked too: only format 0 is mapped,
render targets only with NMB_XR10_TARGETS, the log line carries the fourcc. With --m2v <metal2vulkan checkout>, the three Metal
format numbers are checked against its generated table.
"""
import argparse, pathlib, re, shutil, subprocess, sys, tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
PATCH = ROOT / 'patches/adreno-metal/0021-xr10-surface-imports.patch'

TEST = r'''
#include <cstdint>
#include <cstdio>
#include <cstring>
typedef unsigned long NSUInteger;
#include "helpers.h"
static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %d %s\n", __LINE__, #c); ++failures; } } while (0)
static uint32_t cc(const char *s) { return (uint32_t)(unsigned char)s[0] << 24 | (uint32_t)(unsigned char)s[1] << 16 | (uint32_t)(unsigned char)s[2] << 8 | (unsigned char)s[3]; }
int main() {
  CHECK(xr10SurfaceFormat(cc("w30r"), 0, 0) == 554 && xr10SurfaceFormat(cc("&w3r"), 1, 0) == 554);
  CHECK(xr10SurfaceFormat(cc("w40a"), 0, 0) == 552 && xr10SurfaceFormat(cc("w40m"), 1, 0) == 552);
  CHECK(xr10SurfaceFormat(cc("&w4a"), 0, 0) == 552 && xr10SurfaceFormat(cc("&w4m"), 0, 0) == 552);
  CHECK(xr10SurfaceFormat(cc("b3a8"), 2, 0) == 550);
  CHECK(xr10SurfaceFormat(cc("b3a8"), 2, 1) == 0 && xr10SurfaceFormat(cc("b3a8"), 0, 0) == 0);   // plane 1, or no planes
  CHECK(xr10SurfaceFormat(cc("w30r"), 2, 0) == 0 && xr10SurfaceFormat(cc("w40a"), 2, 1) == 0);   // a planar spelling is not these
  const char *others[] = {"BGRA", "&BGA", "420v", "420f", "RGhA", "L008", "l10r", "w30R", "b3a9"};
  for (const char *o : others) CHECK(xr10SurfaceFormat(cc(o), 0, 0) == 0 && xr10SurfaceFormat(cc(o), 2, 0) == 0);
  for (NSUInteger f = 0; f < 1000; f++) CHECK(isXr10PixelFormat(f) == (f >= 550 && f <= 555));
  char name[5];
  fourccText(cc("b3a8"), name); CHECK(!std::strcmp(name, "b3a8"));
  fourccText(cc("&w4a"), name); CHECK(!std::strcmp(name, "&w4a"));
  fourccText(0x00ff7f41u, name); CHECK(!std::strcmp(name, "...A"));
  std::printf("xr10 helpers: %s\n", failures ? "FAIL" : "PASS");
  return failures ? 1 : 0;
}
'''

def added_text(patch):
    lines = []
    for line in patch.splitlines():
        if line.startswith('+') and not line.startswith('+++'): lines.append(line[1:])
    return '\n'.join(lines) + '\n'

def function(text, signature):
    at = text.find(signature)
    assert at >= 0, 'not in the patch: ' + signature
    depth, i = 0, text.index('{', at)
    while True:
        if text[i] == '{': depth += 1
        elif text[i] == '}':
            depth -= 1
            if not depth: return text[at:i + 1] + '\n'
        i += 1

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--m2v', type=pathlib.Path, help='metal2vulkan checkout: check the Metal format numbers')
    args = parser.parse_args()
    patch = PATCH.read_text()
    added = added_text(patch)
    helpers = ''.join(function(added, s) for s in ('static uint32_t xr10SurfaceFormat(', 'static bool isXr10PixelFormat(', 'static void fourccText('))
    # The call site: only an Invalid format is mapped, targets only with the switch, the log carries the fourcc.
    rc = 0
    for need in ('if(xrFormat && askedFormat==MTLPixelFormatInvalid)', '(!target || xr10TargetsOn())', 'mapped.pixelFormat=(MTLPixelFormat)xrFormat',
                 "native-metal: XR10 surface '%s' (0x%08x)", "native-metal: texture over surface '%s' fmt %lu refused"):
        if need not in added: print('FAIL call site:', need); rc = 1
    if args.m2v:
        table = (args.m2v / 'crates/metal2vulkan/src/metal/generated.rs').read_text()
        for name, value in (('RGB10A82PXR10', 550), ('BGRA10XR', 552), ('BGR10XR', 554)):
            if not re.search(r'\b%s = %d,' % (name, value), table): print('FAIL metal2vulkan', name, value); rc = 1
    compiler = shutil.which('clang++') or shutil.which('g++')
    with tempfile.TemporaryDirectory() as work:
        w = pathlib.Path(work)
        (w / 'helpers.h').write_text('#include <string.h>\n' + helpers)
        (w / 'test.cpp').write_text(TEST)
        subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', work, str(w / 'test.cpp'), '-o', str(w / 'test')], check=True)
        rc |= subprocess.run([str(w / 'test')]).returncode
    print('== result:', 'PASS' if not rc else 'FAIL')
    return rc

if __name__ == '__main__':
    sys.exit(main())
