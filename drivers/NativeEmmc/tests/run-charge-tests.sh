#!/bin/bash
# Host tests of the PMI632 charger controller (WSL): GuampChargeReplugCore.h against a simulated SMB5 (boot, unplug,
# replug, wall chargers by source type, weak-adapter steps, Type-C recovery, ceilings on every write), built with clang
# and gcc, then the string-literal check of the kernel sources. Binaries go to $OUT (default ~/moto/btcheck/charge-out).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${OUT:-$HOME/moto/btcheck/charge-out}
mkdir -p "$OUT"
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -I$HERE/.."
rc=0
echo "== no raw newline inside a string literal of the kernel sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampChargeReplugCore.h PmicService.inc) || rc=1
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/charge_replug_host.cpp" -o "$OUT/charge_host" || exit 1
g++ $FLAGS "$HERE/charge_replug_host.cpp" -o "$OUT/charge_host_gcc" || rc=1
"$OUT/charge_host" > "$OUT/charge_host.log" || rc=1
tail -1 "$OUT/charge_host.log"
[ -x "$OUT/charge_host_gcc" ] && { "$OUT/charge_host_gcc" >/dev/null || rc=1; }
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
