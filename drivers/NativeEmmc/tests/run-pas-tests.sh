#!/bin/bash
# Host tests of the authenticated PAS loader, of the ADSP transports (phase 2) and of the ADSP glue (WSL). Reads the backup modem image in place
# (read-only) and the files extracted from it. Usage: run-pas-tests.sh <modem_a.img> <extracted /image directory>
# e.g. run-pas-tests.sh /mnt/c/Users/Chris/Desktop/Moto/backup/guamp-20261022/modem_a.img \
#        /mnt/c/Users/Chris/Desktop/Moto/firmware-partizioni/modem/image
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
IMG=$1; EXTRACTED=$2
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -fno-exceptions -fno-rtti"
KFLAGS="--target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter"
rc=0
echo "== kernel glue: arm64-apple-ios, -Werror, integer registers only, smc present"
clang++ $KFLAGS -S "$HERE/adsp_glue_check.cpp" -o "$WORK/adsp_glue.s" && grep -q 'smc' "$WORK/adsp_glue.s" \
  && echo "adsp_glue_check: compiled, smc emitted" || rc=1
echo "== no raw newline inside a string literal of the new kernel sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampAdsp.inc GuampAdspIpc.inc GuampGpuZap.inc GuampSmemWrite.inc GuampSmemLock.inc GuampWifiWire.inc GuampSmp2pEdge.inc GuampGlinkEdge.inc GuampApr.inc GuampPas.inc GuampPasCore.inc GuampScm.inc GuampScmArm64.inc NativeEmmc.cpp) || rc=1
echo "== SCM transport (clang, gcc)"
clang++ $FLAGS "$HERE/scm_host.cpp" -o "$WORK/scm" && "$WORK/scm" || rc=1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/scm_host.cpp" -o "$WORK/scm_gcc" && "$WORK/scm_gcc" >/dev/null || rc=1
echo "== PAS loader on the real ADSP and modem firmware (clang + ASan/UBSan, gcc)"
clang++ ${FLAGS/-O2/-O1} -g -fsanitize=address,undefined -fno-omit-frame-pointer "$HERE/pas_host.cpp" -o "$WORK/pas" \
  && "$WORK/pas" "$IMG" "$EXTRACTED" || rc=1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/pas_host.cpp" -o "$WORK/pas_gcc" && "$WORK/pas_gcc" "$IMG" "$EXTRACTED" >/dev/null || rc=1
echo "== ADSP transports (SMEM alloc, SMP2P, GLINK, APR, GuampAdspIpc.inc glue) against a simulated ADSP (clang + ASan/UBSan, gcc)"
clang++ ${FLAGS/-O2/-O1} -g -fsanitize=address,undefined -fno-omit-frame-pointer "$HERE/adsp_ipc_host.cpp" -o "$WORK/ipc" \
  && "$WORK/ipc" || rc=1
g++ ${FLAGS/-Wdeprecated-volatile/-Wvolatile} "$HERE/adsp_ipc_host.cpp" -o "$WORK/ipc_gcc" && "$WORK/ipc_gcc" >/dev/null || rc=1
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
