#!/bin/bash
# Host tests of the sensor hub (GuampSensorsCore.h) with clang and gcc, the HID descriptor check of the NativeHidInput
# sensor devices (scripts/guamp-hid-variant.py step 4) and the string-literal check. Binaries go to $OUT.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${OUT:-$HOME/moto/btcheck/sensors-out}
mkdir -p "$OUT"
FLAGS="-std=gnu++20 -O2 -Wall -Wextra -Werror -Wno-unused-parameter -I$HERE/.."
rc=0
echo "== no raw newline inside a string literal of the kernel sources"
(cd "$HERE/.." && python3 -I tests/check_literals.py GuampSensorsCore.h GuampSensors.inc UsbMirror.inc) || rc=1
echo "== HID descriptors of the sensor devices"
python3 -I "$HERE/check_sensor_hid.py" || rc=1
echo "== host build (clang and gcc)"
clang++ $FLAGS "$HERE/sensors_host.cpp" -o "$OUT/sensors_host" || exit 1
g++ $FLAGS "$HERE/sensors_host.cpp" -o "$OUT/sensors_host_gcc" || rc=1
"$OUT/sensors_host" || rc=1
[ -x "$OUT/sensors_host_gcc" ] && { "$OUT/sensors_host_gcc" >/dev/null || rc=1; }
echo "== result: $([ $rc = 0 ] && echo PASS || echo FAIL)"
exit $rc
