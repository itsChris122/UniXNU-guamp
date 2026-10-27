#!/bin/bash
# Host tests of the panel backlight (WSL): kernel glue -Werror check on arm64-apple-ios, the string-literal check, then
# GuampBacklightCore.h (DCS packet vs the Linux buffer, clamps, fade, rate limit, DSI sequence on a fake controller),
# built with clang and gcc.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -fno-exceptions -fno-rtti"
rc=0
echo "== kernel glue: arm64-apple-ios, -Werror, integer registers only"
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -O2 \
  -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -c "$HERE/backlight_glue_check.cpp" -o "$WORK/glue.o" && echo "glue: compiled" || rc=1
echo "== no raw newline inside a string literal of the new kernel sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampBacklight.inc GuampBacklightCore.h) || rc=1
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/backlight_host.cpp" -o "$WORK/backlight_host" || exit 1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/backlight_host.cpp" -o "$WORK/backlight_host_gcc" || rc=1
"$WORK/backlight_host" || rc=1
[ -x "$WORK/backlight_host_gcc" ] && { "$WORK/backlight_host_gcc" >/dev/null || rc=1; }
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
