#!/bin/bash
# Host tests of the Bluetooth M0 code (WSL): kernel glue -Werror check on arm64-apple-ios, then the HCI/H4 framing,
# QCA version decoding, board helpers and GENI UART engine tests built with clang and gcc.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -fno-exceptions -fno-rtti"
rc=0
echo "== kernel glue: arm64-apple-ios, -Werror, integer registers only"
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -O2 \
  -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -c "$HERE/bt_glue_check.cpp" -o "$WORK/glue.o" && echo "glue: compiled" || rc=1
echo "== no raw newline inside a string literal of the new kernel sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py BtService.inc GeniUartCpp.inc BtHci.h GeniUart.h GuampBtBoard.h) || rc=1
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/bt_host.cpp" -o "$WORK/bt_host" || exit 1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/bt_host.cpp" -o "$WORK/bt_host_gcc" || rc=1
"$WORK/bt_host" || rc=1
[ -x "$WORK/bt_host_gcc" ] && { "$WORK/bt_host_gcc" >/dev/null || rc=1; }
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
