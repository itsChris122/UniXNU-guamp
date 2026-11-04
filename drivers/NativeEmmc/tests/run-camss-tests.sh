#!/bin/bash
# Host tests of the CAMSS RAW capture core (GuampCamssCore.h): clang (ASan/UBSan) and gcc builds of camss_host.cpp,
# plus the literal check of the new kernel sources. The kernel glue (GuampCamss.inc) is checked with the real XNU
# headers by the NativeEmmc syntax build (~/moto/cc-camera.sh with S = this worktree), not here.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O1 -g -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function"
rc=0
echo "== no raw newline inside a string literal of the new kernel sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampCamss.inc GuampCamssCore.h GuampCamssApi.h) || rc=1
echo "== clang ASan/UBSan"
clang++ $FLAGS -fsanitize=address,undefined -fno-sanitize-recover=all "$HERE/camss_host.cpp" -o "$WORK/a" && "$WORK/a" || rc=1
echo "== gcc"
g++ $FLAGS "$HERE/camss_host.cpp" -o "$WORK/g" && "$WORK/g" >/dev/null || rc=1
[ $rc = 0 ] && echo "CAMSS TESTS PASS" || echo "CAMSS TESTS FAIL"
exit $rc
