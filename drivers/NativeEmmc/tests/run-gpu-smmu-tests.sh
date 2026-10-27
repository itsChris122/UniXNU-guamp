#!/bin/bash
# Host tests of GuampGpuSmmu.h (WSL): literals, arm64-apple-ios -Werror syntax check against the ArmSmmu4K kit
# headers, then the host test linked with the kit's own C sources (ASan + UBSan, clang and gcc).
# ARMSMMU4K: the kit directory (UniXNU kits/ArmSmmu4K, pin 245aca1a); default the local UniXNU checkout.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
KIT=${ARMSMMU4K:-$HOME/moto/UniXNU/kits/ArmSmmu4K}
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
[ -f "$KIT/ArmSmmu4KLive.h" ] || { echo "ArmSmmu4K kit not found at $KIT"; exit 1; }
KITC="ArmSmmu4K ArmSmmu4KAudit ArmSmmu4KContext ArmSmmu4KLive ArmSmmu4KProgram ArmSmmu4KStreams ArmSmmu4KTableMemory"
FLAGS="-std=gnu++20 -O1 -g -Wall -Wextra -Werror -Wno-unused-parameter -fno-exceptions -fno-rtti -I $HERE/.. -I $KIT"
SAN="-fsanitize=address,undefined -fno-sanitize-recover=undefined"
rc=0
echo "== no raw newline inside a string literal"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampGpuSmmu.h) || rc=1
echo "== GuampGpuSmmu.h: arm64-apple-ios, -Werror, integer registers only"
echo '#include "GuampGpuSmmu.h"' | clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions \
  -fno-rtti -mgeneral-regs-only -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -fsyntax-only \
  -I "$HERE/.." -I "$KIT" -x c++ - && echo "smmu: compiled" || rc=1
for cc in clang gcc; do
  cxx=$([ $cc = clang ] && echo clang++ || echo g++)
  objs=""
  for f in $KITC; do
    $cc -std=c11 -O1 -g $SAN -c "$KIT/$f.c" -o "$WORK/$cc-$f.o" || { rc=1; continue 2; }
    objs="$objs $WORK/$cc-$f.o"
  done
  $cxx $FLAGS $SAN "$HERE/gpu_smmu_host.cpp" $objs -o "$WORK/gpu_smmu_host_$cc" && "$WORK/gpu_smmu_host_$cc" || rc=1
done
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
