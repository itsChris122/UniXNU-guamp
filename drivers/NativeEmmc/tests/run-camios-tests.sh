#!/bin/bash
# Host tests of GuampCamIos.inc (AppleH6CamIn + user client): test pattern backend, fake CAMSS ABI 2 backend, and the
# arm64-apple-ios syntax check of the same file. No arguments (WSL or macOS with clang++).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
F="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wdeprecated-volatile -Wno-unused-parameter -fno-exceptions -fno-rtti"
rc=0
clang++ $F "$HERE/camios_host.cpp" -o "$WORK/pattern" -lpthread && "$WORK/pattern" | tail -1 || rc=1
clang++ $F -DCAMIOS_FAKE_CAMSS "$HERE/camios_host.cpp" -o "$WORK/camss" -lpthread && "$WORK/camss" | tail -1 || rc=1
clang++ --target=arm64-apple-ios17 -ffreestanding -mgeneral-regs-only $F -DCAMIOS_SYNTAX_ONLY -fsyntax-only "$HERE/camios_host.cpp"   && echo "camios arm64-ios syntax: ok" || rc=1
exit $rc
