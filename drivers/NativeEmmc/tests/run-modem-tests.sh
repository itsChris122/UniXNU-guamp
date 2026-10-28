#!/usr/bin/env bash
set -euo pipefail
test_root=$(cd "$(dirname "$0")" && pwd)
test_work=$(mktemp -d)
trap 'rm -rf "$test_work"' EXIT
clang++ -std=c++17 -O2 -Wall -Wextra -Werror "$test_root/scm_host.cpp" -o "$test_work/scm"
"$test_work/scm"
clang++ -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer \
  "$test_root/modem_host.cpp" -o "$test_work/modem"
"$test_work/modem" "$1" "$2"
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only \
  -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -S \
  "$test_root/glue_check.cpp" -o "$test_work/glue.s"
grep -q 'smc' "$test_work/glue.s"
echo 'PASS: loader sanitizers and actual ARM64/iOS kernel glue including SMC instruction'
