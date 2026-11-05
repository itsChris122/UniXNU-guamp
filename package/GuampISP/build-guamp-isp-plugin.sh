#!/bin/bash
# Build the guamp capture plug-in H6ISP.mediacapture (macOS with the iPhoneOS SDK, like package/GuampAudio).
#   build-guamp-isp-plugin.sh OUTDIR
# Inputs: this directory (FigCaptureDevice/Stream, kernel link), the per-frame core gcampipe (../GuampCamPipe, agent 6),
# the software ISP gisp (agent 3, GISP_DIR, default
# ../guamp-isp: include/gisp.h + src/*.c) and the 3A g3a (agent 4, G3A_DIR, default ../GuampCamera3A).
# Output: OUTDIR/H6ISP.mediacapture (a dylib, ad-hoc signed with 4 KiB pages) and OUTDIR/trust.json (its CDHash).
# The kernel shows OUTDIR's file at /System/Library/MediaCapture/H6ISP.mediacapture with guamp-camera-ios=1|2
# (patches/xnu/zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-camera-plugin.patch, addon files mediacapture/*); CMCapture in
# mediaserverd dlopen()s it once IOServiceMatching("AppleH6CamIn") finds the kernel service, and it only loads with its
# CDHash in the payload trust cache.
set -euo pipefail
OUT=${1:?usage: build-guamp-isp-plugin.sh OUTDIR}
HERE=$(cd "$(dirname "$0")" && pwd)
GISP_DIR=${GISP_DIR:-$HERE/../guamp-isp}
G3A_DIR=${G3A_DIR:-$HERE/../GuampCamera3A}
[ -f "$GISP_DIR/include/gisp.h" ] || { echo "GISP_DIR=$GISP_DIR: gisp.h missing (agent 3, ~/moto/guamp-isp)"; exit 1; }
[ -f "$G3A_DIR/g3a.h" ] || { echo "G3A_DIR=$G3A_DIR: g3a.h missing (agent 4, package/GuampCamera3A)"; exit 1; }
SDK=$(xcrun --sdk iphoneos --show-sdk-path)
IDENTIFIER=org.guamp.camera.H6ISP
mkdir -p "$OUT"
DYLIB="$OUT/H6ISP.mediacapture"
xcrun --sdk iphoneos clang -target arm64-apple-ios17.0 -isysroot "$SDK" -std=gnu17 -O3 -mcpu=cortex-a73 -fblocks \
  -Wall -Wextra -Werror -Wno-unused-parameter -fvisibility=hidden \
  -I"$GISP_DIR/include" -I"$GISP_DIR/src" -I"$G3A_DIR" -I"$HERE/../GuampCamPipe" \
  -dynamiclib -install_name /System/Library/MediaCapture/H6ISP.mediacapture \
  -framework CoreFoundation -framework CoreMedia -framework CoreVideo -framework IOSurface \
  -o "$DYLIB" "$HERE"/Guamp*.c "$GISP_DIR"/src/gisp_*.c "$G3A_DIR/g3a.c" "$HERE/../GuampCamPipe/gcampipe.c"
xcrun strip -x "$DYLIB"
[ "$(lipo -archs "$DYLIB")" = arm64 ]
grep -q ' _H6ISPCaptureDeviceCreate$' <<<"$(nm -gU "$DYLIB")" || { echo "entry point H6ISPCaptureDeviceCreate not exported"; exit 1; }
codesign --force --sign - --timestamp=none --pagesize 4096 --identifier "$IDENTIFIER" "$DYLIB"
codesign --verify --strict "$DYLIB"
DETAIL=$(codesign --display --verbose=4 "$DYLIB" 2>&1)
grep -qx 'Page size=4096' <<<"$DETAIL" || { echo "the signature must use 4 KiB pages"; exit 1; }
CDHASH=$(sed -n 's/^CDHash=//p' <<<"$DETAIL")
[ ${#CDHASH} -eq 40 ] || { echo "no CDHash"; exit 1; }
printf '{\n  "file": "H6ISP.mediacapture",\n  "identifier": "%s",\n  "cdhash": "%s",\n  "cdhashes": ["%s"]\n}\n' \
  "$IDENTIFIER" "$CDHASH" "$CDHASH" > "$OUT/trust.json"
echo "H6ISP.mediacapture built, CDHash $CDHASH"
