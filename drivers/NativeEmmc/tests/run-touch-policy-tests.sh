#!/bin/bash
# Host tests (WSL) of the touch wake (guamp-touch-wake) and of the release guard: GuampTouchPolicyCore.h on the kernel
# target, the string-literal check, then tests/touch_policy_host.cpp (clang and gcc) on the TWRP firmware when present.
# Usage: run-touch-policy-tests.sh [hlt_Himax_firmware.bin]
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
FW=${1:-/mnt/c/Users/Chris/Desktop/Moto/firmware-touch/twrp-20261103/hlt_Himax_firmware.bin}
[ -f "$FW" ] || FW=
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -fno-exceptions -fno-rtti"
rc=0
echo "== core for the kernel: arm64-apple-ios, -Werror, integer registers only"
printf '#include "%s/../GuampTouchPolicyCore.h"\nint f(unsigned m, int id, bool e) { return guamp_touch_wake::decide(m, id, true, e).verdict; }\nint g(int rc, int p, unsigned char c, unsigned n, unsigned long long s) { return guamp_touch_guard::verdict(rc, p, c, n, s, 30000); }\nint h(unsigned m, unsigned long long t) { static guamp_touch_storm::State st; guamp_touch_storm::Cfg c; bool d = false; guamp_touch_storm::step(st, c, m, t, &d); return d; }\n' "$HERE" > "$WORK/k.cpp"
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -O2 -Wall -Wextra \
  -Werror -c "$WORK/k.cpp" -o "$WORK/k.o" && echo "core: compiled" || rc=1
echo "== no raw newline inside a string literal"
(cd "$HERE/.." && python3 -I tests/check_literals.py TouchService.inc GuampTouchPolicyCore.h) || rc=1
echo "== host (clang and gcc)"
clang++ $FLAGS "$HERE/touch_policy_host.cpp" -o "$WORK/tp" && "$WORK/tp" $FW || rc=1
g++ $FLAGS "$HERE/touch_policy_host.cpp" -o "$WORK/tp_gcc" && "$WORK/tp_gcc" $FW | tail -1 || rc=1
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
