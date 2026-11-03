#!/bin/bash
# Host test of the differential / double-buffered scanout copy (GuampScanoutCore.h, WSL): the reference scaler taps
# (guampLerp, guampLerp2, guampTap) are taken from scripts/guamp-canvas-variant.py so the core stays bit-exact with it.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -fno-exceptions -fno-rtti"
rc=0
python3 -I - "$REPO/scripts/guamp-canvas-variant.py" "$WORK/taps.h" <<'PY' || exit 1
import sys
text = open(sys.argv[1]).read()
a = text.index('static inline uint32_t guampLerp(')
b = text.index('struct GuampScaler {', a)
assert text.count('static inline uint32_t guampLerp(') == 1
open(sys.argv[2], 'w').write(text[a:b])
PY
clang++ $FLAGS -fsanitize=address,undefined "-DVARIANT_TAPS=\"$WORK/taps.h\"" "$HERE/scanout_host.cpp" -o "$WORK/host" && "$WORK/host" || rc=1
g++ $FLAGS "-DVARIANT_TAPS=\"$WORK/taps.h\"" "$HERE/scanout_host.cpp" -o "$WORK/host_gcc" && "$WORK/host_gcc" >/dev/null || rc=1
# The kernel side must compile for arm64 as a kext would (no FP/SIMD registers for the compiler, NEON only in inline asm).
clang++ -target arm64-apple-ios17 -mkernel -mgeneral-regs-only -std=gnu++20 -O2 -Wall -Wextra -Werror -fsyntax-only \
  -x c++ - <<EOF || rc=1
#include "$HERE/../GuampScanoutCore.h"
void f(gscan::State &s, volatile unsigned char *d, const unsigned char *p) { gscan::scan(s, p, 3008, true); gscan::accumulate(s);
  gscan::draw(s, 0, d, 2880, p, 3008, nullptr); }
EOF
echo "== kernel glue: GuampScanout.inc on arm64-apple-ios, -Werror, integer registers only"
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -O2   -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -c "$HERE/scanout_glue_check.cpp" -o "$WORK/glue.o" && echo "glue: compiled" || rc=1
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampScanout.inc GuampScanoutCore.h) || rc=1
[ $rc = 0 ] && echo "== result: PASS" || echo "== result: FAIL"
exit $rc
