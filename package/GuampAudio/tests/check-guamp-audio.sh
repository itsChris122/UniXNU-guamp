#!/bin/bash
# PC checks of GuampAudio.driver without an Apple SDK (the real build is build-guamp-audio-plugin.sh on the macOS runner):
#   1. host test of GuampAudioCore.h + guamp_audio_dev.h ABI: clang with ASan/UBSan, then gcc, -Wall -Wextra -Werror
#   2. GuampAudio.c on the host (stubs/ for the Apple frameworks), -Werror, compiled to an object
#   3. GuampAudio.c and guamp_audio_dev.h with -target arm64-apple-ios17: -fsyntax-only against stubs/ + darwin-libc/
#      (Darwin values of the few libc declarations used), plus the ioctl numbers as the arm64 iOS compiler computes them
#   4. guamp_audio_dev.h as C++ with -DKERNEL (how the kernel side includes it), -Werror
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$(dirname "$HERE")
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT
WARN=(-Wall -Wextra -Werror -Wno-multichar -Wno-four-char-constants)

clang -std=gnu17 -O1 -g "${WARN[@]}" -fsanitize=address,undefined -fno-sanitize-recover=all "$HERE/guamp_audio_core_test.c" -lm \
  -o "$OUT/t-clang"
"$OUT/t-clang"
gcc -std=gnu17 -O2 -Wall -Wextra -Werror "$HERE/guamp_audio_core_test.c" -lm -o "$OUT/t-gcc"
"$OUT/t-gcc"

clang -std=gnu17 -O2 "${WARN[@]}" -fvisibility=hidden -I "$HERE/stubs" -c "$SRC/GuampAudio.c" -o "$OUT/GuampAudio.o"
echo "host object: OK ($(stat -c %s "$OUT/GuampAudio.o") bytes)"

IOS=(-target arm64-apple-ios17.0 -nostdlibinc -I "$HERE/stubs" -I "$HERE/darwin-libc")
clang -std=gnu17 "${WARN[@]}" "${IOS[@]}" -fsyntax-only "$SRC/GuampAudio.c"
cat > "$OUT/abi.c" <<'C'
#include "guamp_audio_dev.h"
const unsigned int abi_numbers[] = {GUAMP_AUDIO_GET_INFO, GUAMP_AUDIO_START, GUAMP_AUDIO_STOP, GUAMP_AUDIO_SET_VOLUME,
                                    GUAMP_AUDIO_SET_ROUTE, GUAMP_AUDIO_GET_ROUTE};
C
clang -std=gnu17 "${IOS[@]}" -I "$SRC" -O1 -S -o "$OUT/abi.s" "$OUT/abi.c"
paste <(printf '%s\n' GET_INFO START STOP SET_VOLUME SET_ROUTE GET_ROUTE) \
      <(sed -n 's/^[[:space:]]*\.long[[:space:]]\+\([0-9]\+\).*/\1/p' "$OUT/abi.s") |
  while read -r name value; do printf '  GUAMP_AUDIO_%-12s 0x%08x\n' "$name" "$value"; done
cat > "$OUT/mic.c" <<'C'
#include "guamp_mic_dev.h"
const unsigned int mic_numbers[] = {GUAMP_MIC_IOC_INFO, GUAMP_MIC_IOC_START, GUAMP_MIC_IOC_STOP, GUAMP_MIC_IOC_POSITION,
                                    GUAMP_MIC_IOC_GAIN, GUAMP_MIC_IOC_SET_MIC};
C
clang -std=gnu17 "${IOS[@]}" -I "$SRC" -O1 -S -o "$OUT/mic.s" "$OUT/mic.c"
paste <(printf '%s\n' INFO START STOP POSITION GAIN SET_MIC) \
      <(sed -n 's/^[[:space:]]*\.long[[:space:]]\+\([0-9]\+\).*/\1/p' "$OUT/mic.s") |
  while read -r name value; do printf '  GUAMP_MIC_IOC_%-9s 0x%08x\n' "$name" "$value"; done
KERNEL_MIC="$SRC/../../drivers/NativeEmmc/guamp_mic_dev.h"
if [ -f "$KERNEL_MIC" ]; then cmp "$SRC/guamp_mic_dev.h" "$KERNEL_MIC" && echo "guamp_mic_dev.h identical to the kernel copy"; fi
echo "arm64-apple-ios17 syntax: OK"

cat > "$OUT/k.cpp" <<'CXX'
#include "guamp_audio_dev.h"
int guamp_audio_dev_cxx_check(void) { return static_cast<int>(sizeof(guamp_audio_info) + sizeof(guamp_audio_volume)); }
CXX
clang++ -std=c++17 "${WARN[@]}" -DKERNEL -target arm64-apple-ios17.0 -nostdlibinc -I "$HERE/darwin-libc" -I "$SRC" -fsyntax-only \
  "$OUT/k.cpp"
echo "kernel C++ include: OK"
