#!/usr/bin/env bash
# GuampSim.inc (+ GuampQrtr.inc client ports): host test (sanitizers) + ARM64/iOS freestanding object.
set -euo pipefail
test_root=$(cd "$(dirname "$0")" && pwd)
test_work=$(mktemp -d)
trap "rm -rf \"$test_work\"" EXIT
clang++ -std=c++17 -O1 -g -Wall -Wextra -Werror \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  "$test_root/sim_host.cpp" -o "$test_work/sim"
"$test_work/sim"
cat > "$test_work/sim_arm64.cpp" <<CPP
#include "$test_root/kernel_stub.h"
#include "$test_root/../GuampSim.inc"
int simCodegen(guampwifi::Qrtr *q,guampsim::SimProbe *s,uint64_t now) {
  int e=s->start(q,now); if(e) return e;
  return s->pump(now+1);
}
CPP
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only \
  -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter \
  -c "$test_work/sim_arm64.cpp" -o "$test_work/sim-arm64.o"
echo "PASS: GuampSim.inc ARM64/iOS object compilation"
