#!/bin/bash
# Host tests of the eMMC read-ahead (guamp-emmc-ra) and request histogram (guamp-emmc-hist): the namespace emmcra and
# readAhead() are cut out of NativeEmmc.cpp into $OUT/emmc_ra_extract.inc and built with clang and gcc against a
# simulated card (emmc_ra_host.cpp), then the string-literal check of the driver. Binaries go to $OUT
# (default ~/moto/btcheck/emmc-ra-out).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${OUT:-$HOME/moto/btcheck/emmc-ra-out}
mkdir -p "$OUT"
SRC="$HERE/../NativeEmmc.cpp"
rc=0
echo "== no raw newline inside a string literal of the driver"
(cd "$HERE/.." && python3 -I tests/check_literals.py NativeEmmc.cpp) || rc=1
echo "== extract emmcra and readAhead"
{
  sed -n '/^namespace emmcra {/,/^}  \/\/ namespace emmcra/p' "$SRC"
  sed -n '/^static int readAhead(/,/^}/p' "$SRC"
} > "$OUT/emmc_ra_extract.inc"
grep -q "namespace emmcra" "$OUT/emmc_ra_extract.inc" && grep -q "readAhead(" "$OUT/emmc_ra_extract.inc" || { echo "extract failed"; exit 1; }
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -I$OUT"
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/emmc_ra_host.cpp" -o "$OUT/emmc_ra_host" || exit 1
g++ $FLAGS "$HERE/emmc_ra_host.cpp" -o "$OUT/emmc_ra_host_gcc" || rc=1
"$OUT/emmc_ra_host" | tail -4 || rc=1
"$OUT/emmc_ra_host" >/dev/null || rc=1
[ -x "$OUT/emmc_ra_host_gcc" ] && { "$OUT/emmc_ra_host_gcc" >/dev/null || rc=1; }
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
