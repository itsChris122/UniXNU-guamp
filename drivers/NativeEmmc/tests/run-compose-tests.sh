#!/bin/bash
# Host test of the display's composition core (scripts/guamp-canvas-variant.py section 12, WSL): the text between the
# guamp-compose-core markers of the variant script is compiled into compose_host.cpp with clang (ASan/UBSan) and gcc.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -fno-exceptions -fno-rtti"
rc=0
python3 -I - "$REPO/scripts/guamp-canvas-variant.py" "$WORK/core.h" <<'PY' || exit 1
import sys
text = open(sys.argv[1]).read()
a = text.index('// guamp-compose-core-begin')
b = text.index('// guamp-compose-core-end', a)
assert text.count('// guamp-compose-core-begin') == 1
open(sys.argv[2], 'w').write(text[a:b])
PY
clang++ $FLAGS -Wdeprecated-volatile -fsanitize=address,undefined "-DCOMPOSE_CORE=\"$WORK/core.h\"" "$HERE/compose_host.cpp" \
  -o "$WORK/host" && "$WORK/host" || rc=1
g++ $FLAGS -Wvolatile "-DCOMPOSE_CORE=\"$WORK/core.h\"" "$HERE/compose_host.cpp" -o "$WORK/host_gcc" && "$WORK/host_gcc" >/dev/null || rc=1
[ $rc = 0 ] && echo "== result: PASS" || echo "== result: FAIL"
exit $rc
