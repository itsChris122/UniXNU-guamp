#!/bin/bash
# Host tests of the battery health values for iOS 17 Settings (GuampBatteryCore.h: health, serial, WeightedRa), built
# with clang and gcc, then the string-literal check of the battery sources. Binaries go to $OUT
# (default ~/moto/btcheck/battery-out).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${OUT:-$HOME/moto/btcheck/battery-out}
mkdir -p "$OUT"
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -I$HERE/.."
rc=0
echo "== no raw newline inside a string literal of the battery sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampBatteryCore.h GuampBatteryService.inc) || rc=1
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/battery_health_host.cpp" -o "$OUT/battery_host" || exit 1
g++ $FLAGS "$HERE/battery_health_host.cpp" -o "$OUT/battery_host_gcc" || rc=1
"$OUT/battery_host" || rc=1
[ -x "$OUT/battery_host_gcc" ] && { "$OUT/battery_host_gcc" >/dev/null || rc=1; }
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
