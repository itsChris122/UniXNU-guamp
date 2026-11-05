#!/bin/bash
# Integration skeleton of the camera (agent 6): fake CAMSS (sensor model or TPG) -> GuampCamIos.inc (real) -> gcampipe
# (package/GuampCamPipe) -> gisp (package/guamp-isp) + g3a (package/GuampCamera3A) -> NV12. clang ASan/UBSan and gcc,
# both sources (CAMIOS_ARG_MODE=1 sensor, 3 TPG). No arguments. CAMPIPE_PPM=/path/out.ppm keeps the last preview frame.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
WORK=$(mktemp -d "${TMPDIR:-/tmp}/campipe.XXXXXX")
trap 'rm -rf "$WORK"' EXIT
GISP=$ROOT/package/guamp-isp
G3A=$ROOT/package/GuampCamera3A
PIPE=$ROOT/package/GuampCamPipe
INC="-I$GISP/include -I$G3A -I$PIPE -I$HERE"
CSRC="$GISP/src/gisp_core.c $GISP/src/gisp_stats.c $GISP/src/gisp_fast.c $G3A/g3a.c $PIPE/gcampipe.c"
CXXF="-std=gnu++20 -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -fno-exceptions -fno-rtti -DCAMIOS_NO_MAIN"
CF="-std=c11 -Wall -Wextra -Werror -D_POSIX_C_SOURCE=200809L"
rc=0
build() {   # name cc cxx flags
  local n=$1 cc=$2 cxx=$3 fl=$4 objs=""
  mkdir -p "$WORK/$n"
  for f in $CSRC; do
    $cc $CF $fl $INC -c "$f" -o "$WORK/$n/$(basename "$f" .c).o" || return 1
    objs="$objs $WORK/$n/$(basename "$f" .c).o"
  done
  for m in 1 3; do
    $cxx $CXXF $fl -DCAMIOS_ARG_MODE=$m $INC "$HERE/campipe_host.cpp" $objs -lm -lpthread -o "$WORK/$n/campipe$m" || return 1
  done
}
run() { "$1" | grep -E "^(campipe|preview|FAIL)" ; return "${PIPESTATUS[0]}"; }
echo "== clang ASan/UBSan"
if build asan clang clang++ "-O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all"; then
  run "$WORK/asan/campipe1" || rc=1
  run "$WORK/asan/campipe3" || rc=1
else rc=1; fi
echo "== gcc -O2"
if build gcc gcc g++ "-O2"; then
  run "$WORK/gcc/campipe1" || rc=1
  run "$WORK/gcc/campipe3" || rc=1
else rc=1; fi
echo "== C ABI check of gcampipe.h / GuampCamIosAbi.h (C11, -Werror)"
clang -std=c11 -Wall -Wextra -Werror -fsyntax-only $INC "$PIPE/gcampipe.c" && echo "c abi: ok" || rc=1
[ $rc = 0 ] && echo "CAMPIPE TESTS PASS" || echo "CAMPIPE TESTS FAIL"
exit $rc
