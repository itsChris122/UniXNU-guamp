#!/usr/bin/env bash
# GuampRmtfs.inc: host test (sanitizers) + ARM64/iOS freestanding object of the same source.
set -euo pipefail
test_root=$(cd "$(dirname "$0")" && pwd)
test_work=$(mktemp -d)
trap "rm -rf \"$test_work\"" EXIT
clang++ -std=c++17 -O1 -g -Wall -Wextra -Werror \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  "$test_root/rmtfs_host.cpp" -o "$test_work/rmtfs"
"$test_work/rmtfs"
cat > "$test_work/rmtfs_arm64.cpp" <<CPP
#include "$test_root/kernel_stub.h"
#include "$test_root/../GuampRmtfs.inc"
int rmtfsCodegen(guampwifi::Rmtfs *r,const guampwifi::Rmtfs::Storage &s,const uint8_t *p,uint32_t n) {
  int e=r->start(s,nullptr,nullptr); if(e) return e;
  e=r->receive(0,0x4001,p,n); r->step(); r->stop(); return e;
}
CPP
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only \
  -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter \
  -c "$test_work/rmtfs_arm64.cpp" -o "$test_work/rmtfs-arm64.o"
echo "PASS: GuampRmtfs.inc ARM64/iOS object compilation"
