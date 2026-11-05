#!/bin/bash
# Host tests of the user-space 3A (g3a.c algorithms, g3a_still.c photo + flash sequencer): clang with ASan/UBSan, gcc,
# the 3A on real gisp statistics (package/guamp-isp), the C++ include check and an arm64-apple-ios syntax check.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
GISP=${GISP_DIR:-$HERE/../../guamp-isp}
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=c11 -O1 -g -Wall -Wextra -Werror -Wshadow"
SAN="-fsanitize=address,undefined -fno-sanitize-recover=all"
rc=0
echo "== algorithms (g3a_test.c, simulated scene): clang ASan/UBSan, gcc -O2"
clang $FLAGS $SAN "$HERE/../g3a.c" "$HERE/g3a_test.c" -lm -o "$WORK/a" && "$WORK/a" || rc=1
gcc ${FLAGS/-O1/-O2} "$HERE/../g3a.c" "$HERE/g3a_test.c" -lm -o "$WORK/g" && "$WORK/g" >/dev/null || rc=1
echo "== photo + flash sequencer (g3a_still_test.c): clang ASan/UBSan, gcc -O2"
clang $FLAGS $SAN "$HERE/../g3a.c" "$HERE/../g3a_still.c" "$HERE/g3a_still_test.c" -lm -o "$WORK/s" && "$WORK/s" || rc=1
gcc ${FLAGS/-O1/-O2} "$HERE/../g3a.c" "$HERE/../g3a_still.c" "$HERE/g3a_still_test.c" -lm -o "$WORK/sg" && "$WORK/sg" >/dev/null || rc=1
echo "== 3A on gisp statistics (g3a_gisp_test.c, GISP_DIR=$GISP)"
if [ -f "$GISP/include/gisp.h" ]; then
  clang ${FLAGS/-O1/-O2} $SAN -D_POSIX_C_SOURCE=200809L -I"$GISP/include" -I"$GISP/src" -I"$GISP/tests" "$HERE/../g3a.c" \
    "$GISP/src/gisp_core.c" "$GISP/src/gisp_stats.c" "$HERE/g3a_gisp_test.c" -lm -o "$WORK/x" && "$WORK/x" || rc=1
else
  echo "gisp not found: skipped"
fi
echo "== headers from C++ (gcampipe, campipe_host.cpp)"
printf '#include "g3a.h"\n#include "g3a_still.h"\nint main() { g3a_af_state a = {}; g3a_still s = {}; return a.state + s.state + G3A_AF_FOCUSED; }\n' > "$WORK/cxx.cpp"
clang++ -std=c++17 -Wall -Wextra -Werror -I"$HERE/.." -fsyntax-only "$WORK/cxx.cpp" && echo "c++: ok" || rc=1
echo "== arm64-apple-ios17 syntax check (no SDK here: libm/string prototypes from a stand-in header)"
mkdir -p "$WORK/inc"
cat > "$WORK/inc/math.h" <<H
float expf(float); float logf(float); float log2f(float); float exp2f(float); float sqrtf(float); float fabsf(float);
float fmaxf(float, float); float floorf(float); float powf(float, float);
double floor(double); double fabs(double); double log2(double); double exp2(double); double fmax(double, double);
H
cat > "$WORK/inc/string.h" <<H
typedef __SIZE_TYPE__ size_t;
void *memset(void *, int, size_t); void *memcpy(void *, const void *, size_t); void *memmove(void *, const void *, size_t);
H
clang --target=arm64-apple-ios17 -std=c11 -O2 -Wall -Wextra -Werror -ffreestanding -isystem "$WORK/inc" -fsyntax-only \
  "$HERE/../g3a.c" "$HERE/../g3a_still.c" && echo "ios: syntax ok" || rc=1
[ $rc = 0 ] && echo "G3A TESTS PASS" || echo "G3A TESTS FAIL"
exit $rc
