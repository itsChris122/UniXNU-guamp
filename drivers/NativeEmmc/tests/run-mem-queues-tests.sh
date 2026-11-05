#!/bin/bash
# Host tests of the "GuampMem: queues" line (GuampMemQueuesCore.h + GuampPerf.inc): the sampler cut out of GuampPerf.inc and
# compiled for arm64-apple-ios17 with -Werror, the string-literal check, then the pure logic with clang and gcc.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -fno-exceptions -fno-rtti"
rc=0
echo "== kernel glue: arm64-apple-ios, -Werror"
awk '/^\/\/ GuampMemQueuesCore.h: the pageout queues/{on=1} on{print} on&&/^}$/{exit}' "$HERE/../GuampPerf.inc" > "$WORK/mem_queues_extract.inc"
grep -q "guampMemQueuesSample" "$WORK/mem_queues_extract.inc" || { echo "extract failed"; exit 1; }
cp "$HERE/mem_queues_glue_check.cpp" "$WORK/"
mkdir -p "$WORK/tests" && cp "$HERE/../GuampMemQueuesCore.h" "$WORK/"
mv "$WORK/mem_queues_glue_check.cpp" "$WORK/tests/" && mv "$WORK/mem_queues_extract.inc" "$WORK/tests/"
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only -O2 \
  -Wall -Wextra -Werror -Wno-unused-parameter -c "$WORK/tests/mem_queues_glue_check.cpp" -o "$WORK/glue.o" && echo "glue: compiled" || rc=1
if [ -f "$WORK/glue.o" ]; then
  # The asm labels must name the kernel's own globals (Mach-O: leading underscore).
  for s in _vm_page_throttled_count _vm_page_cleaned_count _vm_page_free_target _vm_page_free_min _vm_page_throttle_limit \
           _vm_compressor_mode _memorystatus_available_pages _memorystatus_available_pages_pressure \
           _memorystatus_available_pages_critical; do
    llvm-nm "$WORK/glue.o" 2>/dev/null | grep -q " U $s$" || nm "$WORK/glue.o" 2>/dev/null | grep -q " U $s$" || { echo "missing undefined $s"; rc=1; }
  done
  [ $rc = 0 ] && echo "glue: kernel symbols referenced by name"
fi
echo "== no raw newline inside a string literal of the new kernel sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampMemQueuesCore.h GuampPerf.inc) || rc=1
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/mem_queues_host.cpp" -o "$WORK/mq" || exit 1
g++ $FLAGS "$HERE/mem_queues_host.cpp" -o "$WORK/mq_gcc" || rc=1
"$WORK/mq" || rc=1
[ -x "$WORK/mq_gcc" ] && { "$WORK/mq_gcc" >/dev/null || rc=1; }
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
