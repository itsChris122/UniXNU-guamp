#!/bin/bash
# Host tests of the Bluetooth M2 shim (WSL): kernel glue -Werror check of GuampBtSerial.inc on arm64-apple-ios, no raw
# newline in string literals, then GuampBtShim.h with clang (ASan + UBSan) and gcc.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O1 -g -Wall -Wextra -Werror -Wno-unused-parameter -fno-exceptions -fno-rtti"
rc=0
echo "== kernel glue: arm64-apple-ios, -Werror, integer registers only"
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -O2 \
  -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -c "$HERE/bt_shim_glue_check.cpp" -o "$WORK/glue.o" \
  && echo "glue: compiled" || rc=1
echo "== no raw newline inside a string literal"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampBtShim.h GuampBtSerial.inc) || rc=1
echo "== host: clang ASan/UBSan, gcc"
clang++ $FLAGS -Wdeprecated-volatile -fsanitize=address,undefined -fno-sanitize-recover=all "$HERE/bt_shim_host.cpp" \
  -o "$WORK/shim" || exit 1
g++ $FLAGS -Wvolatile "$HERE/bt_shim_host.cpp" -o "$WORK/shim_gcc" || rc=1
"$WORK/shim" || rc=1
[ -x "$WORK/shim_gcc" ] && { "$WORK/shim_gcc" >/dev/null || rc=1; }
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
