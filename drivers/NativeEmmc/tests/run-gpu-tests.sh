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
(cd "$HERE/.." && python3 -I tests/check_literals.py GpuService.inc GuampGpuM1.h GuampGpuM2.h GuampGpuBlit.h GuampGpuG2.h) || rc=1
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/gpu_host.cpp" -o "$WORK/gpu_host" || exit 1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/gpu_host.cpp" -o "$WORK/gpu_host_gcc" || rc=1
"$WORK/gpu_host" || rc=1
[ -x "$WORK/gpu_host_gcc" ] && { "$WORK/gpu_host_gcc" >/dev/null || rc=1; }
clang++ $FLAGS "$HERE/gpu_m2_host.cpp" -o "$WORK/gpu_m2_host" && "$WORK/gpu_m2_host" || rc=1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/gpu_m2_host.cpp" -o "$WORK/gpu_m2_host_gcc" && "$WORK/gpu_m2_host_gcc" >/dev/null || rc=1
echo "== GPU 2D fill (GuampGpuBlit.h): arm64-apple-ios syntax check, host test"
printf '#include "GuampGpuBlit.h"
' | clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti   -mgeneral-regs-only -Wall -Wextra -Werror -Wdeprecated-volatile -fsyntax-only -I "$HERE/.." -x c++ - && echo "blit: compiled" || rc=1
clang++ $FLAGS "$HERE/gpu_blit_host.cpp" -o "$WORK/gpu_blit_host" && "$WORK/gpu_blit_host" || rc=1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/gpu_blit_host.cpp" -o "$WORK/gpu_blit_host_gcc" && "$WORK/gpu_blit_host_gcc" >/dev/null || rc=1
echo '#include "GuampGpu3D.h"' | clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -fsyntax-only -I "$HERE/.." -x c++ - && clang++ $FLAGS "$HERE/gpu_3d_host.cpp" -o "$WORK/gpu_3d_host" && "$WORK/gpu_3d_host" && python3 -I "$HERE/check_gpu3d_regs.py" "$HERE/../GuampGpu3D.h" || rc=1
echo "== GPU G2 (GuampGpuG2.h): arm64-apple-ios syntax check, host test"
echo '#include "GuampGpuG2.h"' | clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -fsyntax-only -I "$HERE/.." -x c++ - && echo "g2: compiled" || rc=1
clang++ $FLAGS "$HERE/gpu_g2_host.cpp" -o "$WORK/gpu_g2_host" && "$WORK/gpu_g2_host" || rc=1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/gpu_g2_host.cpp" -o "$WORK/gpu_g2_host_gcc" && "$WORK/gpu_g2_host_gcc" >/dev/null || rc=1
echo "== GPU power (GuampGpuPower.h): literals, arm64-apple-ios syntax check, host test"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampGpuPower.h GuampGpuDcvs.inc) || rc=1
echo '#include "GuampGpuPower.h"' | clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -fsyntax-only -I "$HERE/.." -x c++ - && echo "power: compiled" || rc=1
clang++ $FLAGS "$HERE/gpu_power_host.cpp" -o "$WORK/gpu_power_host" && "$WORK/gpu_power_host" || rc=1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/gpu_power_host.cpp" -o "$WORK/gpu_power_host_gcc" && "$WORK/gpu_power_host_gcc" >/dev/null || rc=1
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
