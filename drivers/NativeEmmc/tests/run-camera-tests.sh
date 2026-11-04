#!/bin/bash
# Host tests of the camera step 1 code (GuampCameraCore.h, GuampCameraSensor.h, GuampCameraRdi.h): clang (ASan/UBSan) and gcc builds of
# camera_host.cpp, plus the literal check of the new kernel sources. Optional argument: a .gcam file made by
# analisi/camera/tools/sensormodule.py --gcam (vendor tables: never in the repo). The kernel glue (GuampCamera.inc)
# is checked with the real XNU headers by ~/moto/cc-camera.sh (S= this worktree), not here.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O1 -g -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function"
rc=0
echo "== no raw newline inside a string literal of the new kernel sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampCamera.inc GuampCameraCore.h GuampCameraSensor.h GuampCameraRdi.h GuampCamera3A.inc GuampCamera3ACore.h) || rc=1
echo "== clang ASan/UBSan"
clang++ $FLAGS -Wdeprecated-volatile -fsanitize=address,undefined -fno-sanitize-recover=all "$HERE/camera_host.cpp" -o "$WORK/a" && "$WORK/a" "$@" || rc=1
echo "== gcc"
g++ $FLAGS -Wvolatile "$HERE/camera_host.cpp" -o "$WORK/g" && "$WORK/g" "$@" >/dev/null || rc=1
echo "== 3A kernel side (GuampCamera3ACore.h, flash policy of GuampTorchCore.h)"
clang++ $FLAGS -Wdeprecated-volatile -fsanitize=address,undefined -fno-sanitize-recover=all "$HERE/camera3a_host.cpp" -o "$WORK/b" && "$WORK/b" || rc=1
g++ $FLAGS -Wvolatile "$HERE/camera3a_host.cpp" -o "$WORK/h" && "$WORK/h" >/dev/null || rc=1
[ $rc = 0 ] && echo "CAMERA TESTS PASS" || echo "CAMERA TESTS FAIL"
exit $rc
