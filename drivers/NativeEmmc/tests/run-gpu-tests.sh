#!/bin/bash
# Host tests of the Adreno 610 M1 code (WSL): kernel glue -Werror check on arm64-apple-ios, the string-literal check,
# then the power-up sequencing (GuampGpuM1.h) against a fake GCC/GPUCC/GPU register file, built with clang and gcc.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -fno-exceptions -fno-rtti"
rc=0
echo "== kernel glue: arm64-apple-ios, -Werror, integer registers only"
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -O2 \
  -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -c "$HERE/gpu_glue_check.cpp" -o "$WORK/glue.o" && echo "glue: compiled" || rc=1
echo "== no raw newline inside a string literal of the new kernel sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py GpuService.inc GuampGpuM1.h) || rc=1
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/gpu_host.cpp" -o "$WORK/gpu_host" || exit 1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/gpu_host.cpp" -o "$WORK/gpu_host_gcc" || rc=1
"$WORK/gpu_host" || rc=1
[ -x "$WORK/gpu_host_gcc" ] && { "$WORK/gpu_host_gcc" >/dev/null || rc=1; }
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
