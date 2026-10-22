#!/usr/bin/env python3
"""Link target for the display probe the DEVELOPMENT-only probe patch calls unconditionally.

guamp has no display bring-up yet: the real NativeLegacyDisplay archive needs the sheng ABI
generator. The probe only runs for the offline-legacy-register boot argument, which this board
does not pass, so an empty function keeps the link honest without pretending to drive the panel.
"""
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'build/guamp-display-stub'

STUB = '''
void native_legacy_display_probe(void) {}
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
