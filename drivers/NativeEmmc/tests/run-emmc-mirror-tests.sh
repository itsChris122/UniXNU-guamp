#!/bin/bash
# Host test of the batched kpan log mirror (07/11): Card::mirrorRun and Card::mirrorRange are cut out of NativeEmmc.cpp
# into $OUT/mirror_extract.inc and built with clang and gcc against a fake host (emmc_mirror_host.cpp).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${OUT:-$HOME/moto/btcheck/emmc-mirror-out}
mkdir -p "$OUT"
sed -n '/^  bool mirrorRun(/,/^  bool findPartitions();/p' "$HERE/../NativeEmmc.cpp" | sed '$d' > "$OUT/mirror_extract.inc"
grep -q "bool mirrorRange(" "$OUT/mirror_extract.inc" || { echo "extract failed"; exit 1; }
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -I$OUT"
rc=0
clang++ $FLAGS "$HERE/emmc_mirror_host.cpp" -o "$OUT/emmc_mirror_host" && "$OUT/emmc_mirror_host" || rc=1
g++ $FLAGS "$HERE/emmc_mirror_host.cpp" -o "$OUT/emmc_mirror_host_gcc" && "$OUT/emmc_mirror_host_gcc" >/dev/null || rc=1
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
