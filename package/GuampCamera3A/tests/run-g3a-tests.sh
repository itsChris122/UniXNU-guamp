#!/bin/bash
# Host tests of the user-space 3A (g3a.c): clang with ASan/UBSan, gcc, and an arm64-apple-ios syntax/codegen check.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=c11 -O1 -g -Wall -Wextra -Werror -Wshadow"
rc=0
echo "== clang ASan/UBSan"
clang $FLAGS -fsanitize=address,undefined -fno-sanitize-recover=all "$HERE/../g3a.c" "$HERE/g3a_test.c" -lm -o "$WORK/a" && "$WORK/a" || rc=1
echo "== gcc -O2"
gcc ${FLAGS/-O1/-O2} "$HERE/../g3a.c" "$HERE/g3a_test.c" -lm -o "$WORK/g" && "$WORK/g" >/dev/null || rc=1
echo "== arm64-apple-ios17 compile (no SDK: freestanding headers only for g3a.c)"
clang --target=arm64-apple-ios17 -std=c11 -O2 -Wall -Wextra -Werror -ffreestanding -fsyntax-only "$HERE/../g3a.c" 2>"$WORK/ios.txt" \
  && echo "ios: compiled" || { grep -q "math.h" "$WORK/ios.txt" && echo "ios: skipped (no math.h without the SDK)" || { cat "$WORK/ios.txt"; rc=1; }; }
[ $rc = 0 ] && echo "G3A TESTS PASS" || echo "G3A TESTS FAIL"
exit $rc
