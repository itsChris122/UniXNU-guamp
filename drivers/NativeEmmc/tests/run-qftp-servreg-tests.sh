#!/usr/bin/env bash
# GuampQftp.inc + GuampServreg.inc: host test (sanitizers) + ARM64/iOS freestanding object of the same sources.
set -euo pipefail
test_root=$(cd "$(dirname "$0")" && pwd)
test_work=$(mktemp -d)
trap "rm -rf \"$test_work\"" EXIT
clang++ -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer \
  "$test_root/qftp_servreg_host.cpp" -o "$test_work/qftp"
"$test_work/qftp"
cat > "$test_work/qftp_arm64.cpp" <<CPP
#include "$test_root/kernel_stub.h"
#include "$test_root/../GuampWifiWire.inc"
#include "$test_root/../GuampQmi.inc"
#include "$test_root/../GuampQftp.inc"
#include "$test_root/../GuampServreg.inc"
int qftpCodegen(guampwifi::Qftp *q,guampwifi::GuampServreg *s,const uint8_t *p,uint32_t n,uint64_t now) {
  int e=q->receive(guampwifi::Qftp::Port,0,0x40,p,n,now); q->step(now); return e|s->receive(0,4,p,n);
}
CPP
clang++ --target=arm64-apple-ios17 -std=gnu++20 -ffreestanding -fno-exceptions -fno-rtti -mgeneral-regs-only \
  -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -c "$test_work/qftp_arm64.cpp" -o "$test_work/qftp.o"
echo "PASS: GuampQftp.inc/GuampServreg.inc ARM64/iOS object compilation"
