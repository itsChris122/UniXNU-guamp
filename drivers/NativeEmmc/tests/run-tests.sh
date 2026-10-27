#!/bin/bash
# Host tests of the guamp Wi-Fi preparation code (WSL). Reads the backup modem image in place (read-only) and the
# files extracted from it earlier; builds a throwaway mke2fs image under $WORK. Usage: run-tests.sh <modem_a.img> <extracted dir>
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
IMG=$1; EXTRACTED=$2
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -fno-exceptions -fno-rtti"
rc=0
echo "== kernel glue: arm64-apple-ios, -Werror, integer registers only"
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -O2 \
  -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -c "$HERE/glue_check.cpp" -o "$WORK/glue.o" && echo "glue: compiled" || rc=1
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -O2 \
  -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -c "$HERE/torch_glue_check.cpp" -o "$WORK/torch_glue.o" && echo "torch glue: compiled" || rc=1
echo "== torch core (GuampTorchCore.h)"
clang++ $FLAGS "$HERE/torch_host.cpp" -o "$WORK/torch_host" && "$WORK/torch_host" || rc=1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/torch_host.cpp" -o "$WORK/torch_host_gcc" && "$WORK/torch_host_gcc" >/dev/null || rc=1
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/wifi_host.cpp" -o "$WORK/wifi_host" || exit 1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/wifi_host.cpp" -o "$WORK/wifi_host_gcc" || rc=1
echo "== SHA-384"
"$WORK/wifi_host" sha | tail -1 || rc=1
bad=0
for n in 0 1 55 111 112 127 128 129 239 240 1000 4096 65537 300001; do
  head -c $n /dev/urandom > "$WORK/r"
  a=$("$WORK/wifi_host" shastream < "$WORK/r"); b=$(python3 -c "import hashlib,sys;print(hashlib.sha384(open(sys.argv[1],'rb').read()).hexdigest())" "$WORK/r")
  [ "$a" = "$b" ] || { echo "sha384 mismatch at $n bytes"; bad=1; rc=1; }
done
[ $bad = 0 ] && echo "sha384 = hashlib on 14 random lengths"
echo "== SMEM (synthetic)"
"$WORK/wifi_host" smem || rc=1
echo "== modem_a.img (ext4 + firmware)"
"$WORK/wifi_host" modem "$IMG" "$EXTRACTED" || rc=1
echo "== mke2fs images (1 KiB blocks + 64bit + htree directory; 4 KiB default)"
mkdir -p "$WORK/src/image/deep/er" "$WORK/src/many"
for i in $(seq 1 3000); do printf "file %d\n" $i > "$WORK/src/many/f$i.txt"; done
head -c 3000000 /dev/urandom > "$WORK/src/image/deep/er/big.bin"
head -c 777 /dev/urandom > "$WORK/src/image/small"
: > "$WORK/src/image/empty"
mke2fs -q -F -t ext4 -b 1024 -O 64bit,^has_journal -d "$WORK/src" "$WORK/k1.img" 64M >/dev/null 2>&1 || { echo "mke2fs failed"; rc=1; }
"$WORK/wifi_host" tree "$WORK/k1.img" "$WORK/src" || rc=1
mke2fs -q -F -t ext4 -d "$WORK/src" "$WORK/k4.img" 64M >/dev/null 2>&1 || { echo "mke2fs failed"; rc=1; }
"$WORK/wifi_host" tree "$WORK/k4.img" "$WORK/src" || rc=1
mke2fs -q -F -t ext4 -O inline_data -d "$WORK/src" "$WORK/inl.img" 64M >/dev/null 2>&1
"$WORK/wifi_host" tree "$WORK/inl.img" "$WORK/src" >/dev/null 2>&1 && { echo "inline_data NOT refused"; rc=1; } || echo "inline_data file system refused at mount"
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
