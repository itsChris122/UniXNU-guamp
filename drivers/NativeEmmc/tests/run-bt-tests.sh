#!/bin/bash
# Host tests of the Bluetooth M0/M1 code (WSL): kernel glue -Werror check on arm64-apple-ios, then the HCI/H4 framing,
# QCA version decoding, board helpers and GENI UART engine tests, the M1 rampatch/NVM code and the ext2 block-map
# reader, built with clang and gcc. Usage: run-bt-tests.sh [bluetooth_a.img] (a copy of the backup; read only).
# Without the image the real-file checks are skipped; the mke2fs -t ext2 images then use random files only.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
IMG=${1:-}
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -fno-exceptions -fno-rtti"
rc=0
echo "== kernel glue: arm64-apple-ios, -Werror, integer registers only"
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -O2 \
  -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -c "$HERE/bt_glue_check.cpp" -o "$WORK/glue.o" && echo "glue: compiled" || rc=1
echo "== no raw newline inside a string literal of the new kernel sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py BtService.inc GeniUartCpp.inc BtHci.h BtQcaFw.h GeniUart.h GuampBtBoard.h GuampExt4.inc) || rc=1
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/bt_host.cpp" -o "$WORK/bt_host" || exit 1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/bt_host.cpp" -o "$WORK/bt_host_gcc" || rc=1
clang++ $FLAGS "$HERE/bt_fw_host.cpp" -o "$WORK/bt_fw_host" || exit 1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/bt_fw_host.cpp" -o "$WORK/bt_fw_host_gcc" || rc=1
"$WORK/bt_host" || rc=1
[ -x "$WORK/bt_host_gcc" ] && { "$WORK/bt_host_gcc" >/dev/null || rc=1; }
echo "== M1: names, baud codes, TLV/NVM, segments, acks, BD address, IBS, RCG"
"$WORK/bt_fw_host" unit || rc=1
[ -x "$WORK/bt_fw_host_gcc" ] && { "$WORK/bt_fw_host_gcc" unit >/dev/null || rc=1; }
mkdir -p "$WORK/src/image/deep"
if [ -n "$IMG" ]; then
  echo "== bluetooth_a.img: /image via GuampExt4.inc against debugfs, then the cmbtfw12/cmnv12 M1 path"
  mkdir -p "$WORK/fw"; debugfs -R "rdump /image $WORK/fw" "$IMG" >/dev/null 2>&1 && "$WORK/bt_fw_host" image "$IMG" "$WORK/fw/image" || rc=1
  cp "$WORK"/fw/image/* "$WORK/src/image/" 2>/dev/null
else
  echo "== bluetooth_a.img not given: real-file checks skipped"
fi
echo "== mke2fs -t ext2 images (block maps: direct, single and double indirect, holes; 1 KiB and 4 KiB blocks)"
head -c 3000000 /dev/urandom > "$WORK/src/image/deep/big.bin"        # 1 KiB blocks: double indirect from 268 KiB
head -c 12288 /dev/urandom > "$WORK/src/image/direct12"              # exactly the 12 direct blocks at 1 KiB
dd if=/dev/urandom of="$WORK/src/image/sparse" bs=1024 count=3 seek=2000 status=none   # a hole across the indirect blocks
head -c 777 /dev/urandom > "$WORK/src/image/small"
: > "$WORK/src/image/empty"
for bs in 1024 4096; do
  mke2fs -q -F -t ext2 -b $bs -d "$WORK/src" "$WORK/e$bs.img" 16M >/dev/null 2>&1 || { echo "mke2fs failed"; rc=1; continue; }
  "$WORK/bt_fw_host" tree "$WORK/e$bs.img" "$WORK/src" blockmap || rc=1
done
mke2fs -q -F -t ext4 -d "$WORK/src" "$WORK/x4.img" 16M >/dev/null 2>&1 && { "$WORK/bt_fw_host" tree "$WORK/x4.img" "$WORK/src" || rc=1; }
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
