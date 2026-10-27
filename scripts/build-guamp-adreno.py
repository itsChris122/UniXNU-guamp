#!/usr/bin/env python3
"""guamp's NativeAdreno archive (CONFIG_XNU_NATIVE_ADRENO links build/native-adreno/libNativeAdreno.a): the GPU half of NativeAdreno
for the Adreno 610 (a6xx gen1, no GMU) with the SMMU and PM4 kits and the board probe drivers/GuampAdreno/GuampAdrenoProbe.cpp,
which NativeEmmc's GPU thread calls (guamp_adreno_start) when guamp-adreno=<level> is set. Adapted from UniXNU-tb128fu
scripts/build-adreno610.py (BSD-2-Clause-Patent). drivers/NativeAdreno must be the a6xx-capable fork (xVanTuring/Adreno610,
see scripts/guamp-adreno-modules.sh): the pinned UniXNU NativeAdreno has no A6XX ring/CP-init generation. Left out, as on tb128fu:
the GMU and its platform nodes, the HFI firmware path, NativeAdrenoService.cpp and its board glue. The staged accelerator class
(build-native-adreno-accelerator.py, run next when wanted) adds itself to this archive. No firmware is embedded: the probe takes
a630_sqe.fw from the caller, which reads it from vendor at boot."""
import json, re, subprocess, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
c_recipe = json.loads((ROOT / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/bsd/DEVELOPMENT/bsd_init.o.json').read_text().strip().rstrip(','))
cxx_recipe = json.loads((ROOT / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/iokit/DEVELOPMENT/IOPlatformExpert.cpo.json').read_text().strip().rstrip(','))
out = ROOT / 'build/native-adreno'
out.mkdir(parents=True, exist_ok=True)
driver = ROOT / 'drivers/NativeAdreno'
probe = ROOT / 'drivers/GuampAdreno'
if not (driver / 'NativeAdrenoIrq.h').read_text().count('maskA6xx'):
    sys.exit('drivers/NativeAdreno has no A6XX generation: check out the Adreno 610 fork (scripts/guamp-adreno-modules.sh)')
sources = sorted((ROOT / 'kits/ArmSmmu4K').glob('ArmSmmu4K*.c')) + [ROOT / 'kits/AdrenoPM4/AdrenoPM4.c']
sources += [driver / (name + '.cpp') for name in (
    'NativeAdrenoGpuContext', 'NativeAdrenoSmmuContext', 'NativeAdrenoSmmuMemory',
    'NativeAdrenoMemory', 'NativeAdrenoCpu', 'NativeAdrenoVm', 'NativeAdrenoRing', 'NativeAdrenoSubmit', 'NativeAdrenoTimer',
    'NativeAdrenoUserClient', 'NativeAdrenoPublish', 'NativeAdrenoCpInit', 'NativeAdrenoIrq')]
sources += [probe / 'GuampAdrenoProbe.cpp']
objects = []
for source in sources:
    recipe = c_recipe if source.suffix == '.c' else cxx_recipe
    obj = out / (source.stem + ('.kit.o' if source.suffix == '.c' else '.o'))
    command, i = [], 0
    while i < len(recipe['arguments']):
        value = recipe['arguments'][i]
        if value == recipe['file']:
            command.append(str(source))
        elif value == '-o':
            command += ['-o', str(obj)]
            i += 1
        elif value in ('-MF', '-MJ'):
            i += 1
        elif value == '-Xclang' and recipe['arguments'][i + 1].startswith('-W'):
            i += 1
        elif not value.startswith('-W') and value not in ('-MD', '-MP'):
            command.append(value)
        i += 1
    command += ['-I', str(ROOT / 'kits/AdrenoPM4'), '-I', str(ROOT / 'kits/ArmSmmu4K'), '-I', str(ROOT / '.cache/xnu-work/osfmk'),
                '-I', str(driver), '-I', str(probe), '-Wall', '-Wextra', '-Werror',
                '-Wno-unused-parameter', '-Wno-incompatible-sysroot', '-Wno-c99-designator',
                '-Wno-reorder-init-list', '-Wno-deprecated-enum-enum-conversion',
                '-Wno-unnecessary-virtual-specifier']
    result = subprocess.run(command, cwd=recipe['directory'], capture_output=True, text=True)
    if result.returncode:
        sys.exit(str(source) + ':\n' + result.stderr[-2500:])
    objects.append(obj)
archive = out / 'libNativeAdreno.a'
archive.unlink(missing_ok=True)
subprocess.run(['xcrun', 'ar', 'rcs', archive, *objects], check=True)
# Every NativeAdreno or kit symbol the objects use must be defined by them; the rest is the kernel's.
nm = subprocess.run(['xcrun', 'nm', '-gP', archive], capture_output=True, text=True, check=True).stdout.splitlines()
defined = {f.split()[0] for f in nm if len(f.split()) > 1 and f.split()[1] in 'TSDBC'}
undefined = {f.split()[0] for f in nm if len(f.split()) > 1 and f.split()[1] == 'U'}
missing = sorted(s for s in undefined - defined
                 if re.search(r'NativeAdreno|native_adreno|ArmSmmu4K|AdrenoPM4|Sheng|native_gop', s)
                 and 'accelerator' not in s and 'NativeAdrenoAccelerator' not in s)
if missing:
    sys.exit('NativeAdreno symbols used but not built here: ' + ', '.join(missing))
required = ('_NativeAdrenoGpuContextBringup', '_NativeAdrenoGpuContextBuffers', '_NativeAdrenoGpuContextVm', '_native_adreno_publish',
            '_ArmSmmu4KAttachStreams', '_AdrenoPM4Type7', '_guamp_adreno_start')
absent = [s for s in required if s not in defined]
if absent:
    sys.exit('Required definitions absent: ' + ', '.join(absent))
print('PASS: %d NativeAdreno a6xx objects archived with the guamp probe' % len(objects))
