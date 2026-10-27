#!/bin/bash
# Host tests of the guamp-ca-metal render server watchdog (WSL): kernel glue -Werror check on arm64-apple-ios, the
# string-literal check, then the decision (GuampCaMetalCore.h) with clang (ASan/UBSan) and gcc.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -fno-exceptions -fno-rtti"
rc=0
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -O2 \
  -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -c "$HERE/ca_metal_glue_check.cpp" -o "$WORK/glue.o" \
  && echo "glue: compiled" || rc=1
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampCaMetal.inc GuampCaMetalCore.h) || rc=1
clang++ $FLAGS -Wdeprecated-volatile -fsanitize=address,undefined "$HERE/ca_metal_host.cpp" -o "$WORK/host" && "$WORK/host" || rc=1
g++ $FLAGS -Wvolatile "$HERE/ca_metal_host.cpp" -o "$WORK/host_gcc" && "$WORK/host_gcc" >/dev/null || rc=1
[ $rc = 0 ] && echo "== result: PASS" || echo "== result: FAIL"
exit $rc
