#!/bin/bash
# Host tests of the RPM client (WSL): kernel glue -Werror check on arm64-apple-ios (GuampRpm.inc with BtService.inc and
# GpuService.inc, plus the existing BT and GPU glue checks), the string-literal check, then the protocol core
# (GuampRpmCore.h) against a fake message RAM and RPM peer, built with clang and gcc.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -fno-exceptions -fno-rtti"
KFLAGS="--target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter"
rc=0
echo "== kernel glue: arm64-apple-ios, -Werror, integer registers only"
for g in rpm_glue_check bt_glue_check gpu_glue_check; do
  clang++ $KFLAGS -c "$HERE/$g.cpp" -o "$WORK/$g.o" && echo "$g: compiled" || rc=1
done
echo "== no raw newline inside a string literal of the new or changed kernel sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampRpm.inc GuampRpmCore.h BtService.inc GpuService.inc NativeEmmc.cpp) || rc=1
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/rpm_host.cpp" -o "$WORK/rpm_host" || exit 1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/rpm_host.cpp" -o "$WORK/rpm_host_gcc" || rc=1
"$WORK/rpm_host" || rc=1
[ -x "$WORK/rpm_host_gcc" ] && { "$WORK/rpm_host_gcc" >/dev/null || rc=1; }
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
