#!/bin/bash
# Host tests of the PM6125 PON decoding and held-key watch (GuampPonCore.h), clang and gcc, plus the string-literal
# check of the kernel sources that use it. Binaries go to $OUT (default ~/moto/btcheck/pon-out).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${OUT:-$HOME/moto/btcheck/pon-out}
mkdir -p "$OUT"
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -I$HERE/.."
rc=0
echo "== no raw newline inside a string literal of the kernel sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampPonCore.h PmicService.inc) || rc=1
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/pon_host.cpp" -o "$OUT/pon_host" || exit 1
g++ $FLAGS "$HERE/pon_host.cpp" -o "$OUT/pon_host_gcc" || rc=1
"$OUT/pon_host" || rc=1
[ -x "$OUT/pon_host_gcc" ] && { "$OUT/pon_host_gcc" >/dev/null || rc=1; }
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
