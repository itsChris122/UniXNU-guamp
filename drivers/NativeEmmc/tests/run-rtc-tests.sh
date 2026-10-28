#!/bin/bash
# Host tests of the guamp wall clock (WSL): GuampRtcCore.h against a simulated PM6125 RTC and two kpan record slots,
# built with clang and gcc, then the string-literal check of the kernel sources. Binaries go to $OUT
# (default ~/moto/btcheck/rtc-out).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${OUT:-$HOME/moto/btcheck/rtc-out}
mkdir -p "$OUT"
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -I$HERE/.."
rc=0
echo "== no raw newline inside a string literal of the kernel sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampRtcCore.h GuampRtc.inc PmicService.inc) || rc=1
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/rtc_host.cpp" -o "$OUT/rtc_host" || exit 1
g++ $FLAGS "$HERE/rtc_host.cpp" -o "$OUT/rtc_host_gcc" || rc=1
"$OUT/rtc_host" || rc=1
[ -x "$OUT/rtc_host_gcc" ] && { "$OUT/rtc_host_gcc" >/dev/null || rc=1; }
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
