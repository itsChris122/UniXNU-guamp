#!/bin/bash
# Prove sul PC del piano B del touch (GuampSoftTouchCore.h): mappe sintetiche sulle mappe TWRP a riposo e mappe TWRP "dito".
# Uso: run-softtouch-tests.sh [cartella log-xnu] (default /mnt/c/Users/Chris/Desktop/Moto/log-xnu)
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
DIR=${1:-/mnt/c/Users/Chris/Desktop/Moto/log-xnu}
WORK=$(mktemp -d)
trap "rm -rf \"$WORK\"" EXIT
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -fno-exceptions -fno-rtti"
rc=0
echo "== core per il kernel: arm64-apple-ios, -Werror, solo registri interi"
printf "#include \"%s/../GuampSoftTouchCore.h\"\nguamp_softtouch::Detector d;\nunsigned f(const int32_t *v, guamp_softtouch::Contact *c) { return d.frame(v, c); }\n" "$HERE" > "$WORK/k.cpp"
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -O2 -Wall -Wextra \
  -Werror -c "$WORK/k.cpp" -o "$WORK/k.o" && echo "core: compilato" || rc=1
echo "== prove (clang e gcc)"
clang++ $FLAGS "$HERE/softtouch_host.cpp" -o "$WORK/st" -lm && "$WORK/st" "$DIR" || rc=1
g++ $FLAGS "$HERE/softtouch_host.cpp" -o "$WORK/st_gcc" -lm && "$WORK/st_gcc" "$DIR" | tail -1 || rc=1
exit $rc
