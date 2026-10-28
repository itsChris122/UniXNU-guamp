#!/usr/bin/env bash
set -euo pipefail
test_root=$(cd "$(dirname "$0")" && pwd)
test_work=$(mktemp -d)
trap 'rm -rf "$test_work"' EXIT
clang++ -std=c++17 -O1 -g -Wall -Wextra -Werror \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  "$test_root/wifi_transport_host.cpp" -o "$test_work/transports"
"$test_work/transports"
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only \
  -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter \
  -c "$test_root/wifi_transport_arm64.cpp" -o "$test_work/transport-arm64.o"
echo 'PASS: actual transport source ARM64/iOS object compilation (individual driver object only)'
