#!/usr/bin/env python3
"""Weak definitions of hooks that the patched XNU and the display archive reference from components guamp leaves out.

The kernel link needs each symbol even where its caller never runs or treats it as absent (as UniXNU-qemu's
package/LinkStubs.c). All weak: a linked component that defines one (the display archive defines
native_legacy_display_probe) replaces it. sheng's boot progress screen is not part of guamp: the display may always draw,
keeps no diagnostic mapping and reports no stages; sheng's button probe is not either (no buttons sink).
"""
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'build/guamp-display-stub'

STUB = '''
#include <stdint.h>
#define WEAK __attribute__((weak))
WEAK void native_legacy_display_probe(void) {}
WEAK int boot_progress_prepare_scanout(void) { return 1; }
WEAK void boot_progress_cancel_scanout(void) {}
WEAK void boot_progress_stage(const char *message) { (void)message; }
WEAK char boot_progress_ext[8][48];
WEAK int boot_progress_pulse_anchor(uintptr_t address, uint64_t bytes, unsigned width, unsigned height, unsigned row)
{ (void)address; (void)bytes; (void)width; (void)height; (void)row; return 0; }
WEAK void native_buttons_set_sink(void (*sink)(void *, uint32_t), void *context) { (void)sink; (void)context; }
'''


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    source = OUT / 'guamp-display-stub.c'
    obj = OUT / 'guamp-display-stub.o'
    source.write_text(STUB)
    sdk = subprocess.check_output(['xcrun', '--sdk', 'iphoneos', '--show-sdk-path'], text=True).strip()
    subprocess.run(['xcrun', '--sdk', 'iphoneos', 'clang', '-c', '-target', 'arm64-apple-ios17.0',
                    '-isysroot', sdk, '-mkernel', '-ffreestanding', '-O2',
                    '-o', str(obj), str(source)], check=True)
    archive = OUT / 'libGuampDisplayStub.a'
    archive.unlink(missing_ok=True)
    subprocess.run(['xcrun', 'ar', 'rcs', str(archive), str(obj)], check=True)
    print(f'display stub: {archive}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
