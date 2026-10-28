#!/bin/bash
# Host tests of the audio output HW code (GeniI2cCpp.inc, GuampAudioHwCore.h): clang (ASan/UBSan) and gcc builds of
# audio_hw_host.cpp, plus the literal check of the new kernel sources. The kernel glue (GuampAudioHw.inc) is checked
# with the real XNU headers by the cc.sh copy (S= this worktree), not here.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O1 -g -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function"
rc=0
echo "== no raw newline inside a string literal of the new kernel sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampAudioHw.inc GuampAudioHwCore.h GeniI2c.h GeniI2cCpp.inc GuampAudioHwExt.h GuampAudioHwExtWeak.inc) || rc=1
echo "== clang ASan/UBSan"
clang++ $FLAGS -Wdeprecated-volatile -fsanitize=address,undefined -fno-sanitize-recover=all "$HERE/audio_hw_host.cpp" -o "$WORK/a" && "$WORK/a" || rc=1
echo "== gcc"
g++ $FLAGS -Wvolatile "$HERE/audio_hw_host.cpp" -o "$WORK/g" && "$WORK/g" || rc=1
[ $rc = 0 ] && echo "AUDIO HW TESTS PASS" || echo "AUDIO HW TESTS FAIL"
exit $rc
