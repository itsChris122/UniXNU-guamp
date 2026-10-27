#!/bin/bash
# -Werror syntax check of the guamp NativeAdreno probe (WSL): GuampAdrenoProbe.cpp against the real NativeAdreno and kit headers
# (the Adreno 610 fork) and authored kernel stand-ins (kernel_shim.h), on arm64-apple-ios. Usage:
#   run-tests.sh [NativeAdreno dir] [ArmSmmu4K dir] [AdrenoPM4 dir]
# (defaults: the device repository's drivers/NativeAdreno and kits/ArmSmmu4K, kits/AdrenoPM4 of the build tree or ~/moto/riferimenti).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)
NA=${1:-$REPO/drivers/NativeAdreno}
SMMU=${2:-$REPO/kits/ArmSmmu4K}
PM4=${3:-$HOME/moto/riferimenti/up-AdrenoPM4}
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
for h in IOKit/IOService.h IOKit/IOLib.h IOKit/IOMemoryDescriptor.h IOKit/IOBufferMemoryDescriptor.h IOKit/IOPlatformExpert.h \
         IOKit/IOUserClient.h IOKit/IOTypes.h libkern/c++/OSSharedPtr.h libkern/c++/OSMetaClass.h libkern/c++/OSSymbol.h \
         mach/kmod.h kern/thread.h kern/clock.h pexpert/pexpert.h string.h; do
  mkdir -p "$WORK/shim/$(dirname "$h")"
  echo '#include "kernel_shim.h"' > "$WORK/shim/$h"
done
rc=0
echo "== GuampAdrenoProbe.cpp: arm64-apple-ios, -Werror (NativeAdreno $NA)"
grep -q maskA6xx "$NA/NativeAdrenoIrq.h" || { echo "NativeAdreno at $NA has no A6XX generation"; exit 1; }
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -fsyntax-only \
  -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -Wno-c99-designator -Wno-reorder-init-list \
  -I "$WORK/shim" -I "$HERE" -I "$NA" -I "$SMMU" -I "$PM4" -I "$HERE/.." "$HERE/../GuampAdrenoProbe.cpp" && echo "probe: compiled" || rc=1
echo "== no raw newline inside a string literal"
(cd "$REPO/drivers/NativeEmmc" && python3 -I tests/check_literals.py ../GuampAdreno/GuampAdrenoProbe.cpp ../GuampAdreno/GuampAdreno.h) || rc=1
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
