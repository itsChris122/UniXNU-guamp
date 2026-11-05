#!/bin/bash
# Host tests of GuampBootMarksCore.h (GuampBoot checkpoints read back by the kernel), clang and gcc, plus the
# string-literal check of the driver. Binaries go to $OUT (default ~/moto/btcheck/gb-marks-out).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${OUT:-$HOME/moto/btcheck/gb-marks-out}
mkdir -p "$OUT"
rc=0
echo "== no raw newline inside a string literal of the driver"
(cd "$HERE/.." && python3 -I tests/check_literals.py NativeEmmc.cpp) || rc=1
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function"
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/gb_marks_host.cpp" -o "$OUT/gb_marks_host" || exit 1
g++ $FLAGS "$HERE/gb_marks_host.cpp" -o "$OUT/gb_marks_host_gcc" || rc=1
"$OUT/gb_marks_host" || rc=1
[ -x "$OUT/gb_marks_host_gcc" ] && { "$OUT/gb_marks_host_gcc" >/dev/null || rc=1; }
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
