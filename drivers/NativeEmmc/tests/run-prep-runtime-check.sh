#!/usr/bin/env bash
# Compile both prep configurations and verify the opt-in runtime link boundary.
# Individual ARM64 objects only; no kernel link, device I/O, or firmware copies.
set -euo pipefail
test_root=$(cd "$(dirname "$0")" && pwd)
if [[ -d "$test_root/../../GuampWifi/integration" ]]; then
  integration_root=$(cd "$test_root/../../GuampWifi/integration" && pwd)
elif [[ -d "$test_root/../../../../wifi/integration" ]]; then
  integration_root=$(cd "$test_root/../../../../wifi/integration" && pwd)
else
  integration_root=$(cd "$test_root/../../../integration" && pwd)
fi
test_work=$(mktemp -d)
trap 'rm -rf "$test_work"' EXIT
flags=(--target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions
  -fno-rtti -mgeneral-regs-only -O2 -Wall -Wextra -Werror
  -Wdeprecated-volatile -Wno-unused-parameter -I "$integration_root")
clang++ "${flags[@]}" -c "$test_root/glue_check.cpp" -o "$test_work/prep-default.o"
clang++ "${flags[@]}" -DGUAMP_WIFI_RUNTIME=1 -c "$test_root/glue_check.cpp" -o "$test_work/prep-runtime.o"
llvm-nm -u "$test_work/prep-default.o" > "$test_work/default-symbols"
llvm-nm -u "$test_work/prep-runtime.o" > "$test_work/runtime-symbols"
if grep -q '_guamp_sm6115_wifi_' "$test_work/default-symbols"; then
  echo 'FAIL: default read-only prep unexpectedly requires radio runtime'
  exit 1
fi
for callback in prepare modem_started shutdown_result abort start_station; do
  grep -Eq "(^|[[:space:]])_guamp_sm6115_wifi_${callback}$" "$test_work/runtime-symbols" || {
    echo "FAIL: opt-in prep does not link runtime callback ${callback}"
    exit 1
  }
done
echo 'PASS: default and opt-in prep ARM64 objects; radio callbacks linked only with GUAMP_WIFI_RUNTIME=1'
