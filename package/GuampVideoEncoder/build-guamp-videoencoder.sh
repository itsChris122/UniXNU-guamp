#!/bin/bash
# Build the guamp software H.264 encoder plug-in H264H7.videoencoder (macOS with the iPhoneOS SDK, like package/GuampAudio).
#   build-guamp-videoencoder.sh OUTDIR
# Output: OUTDIR/H264H7.videoencoder (a flat dylib, ad-hoc signed with 4 KiB pages) and OUTDIR/trust.json (its CDHash).
# The kernel shows it at /System/Library/VideoEncoders/H264H7.videoencoder with guamp-venc=1 (or guamp-camera-ios=1|2)
# (patches/xnu/zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-camera-venc.patch, addon files videoencoders/*); VideoToolbox's
# VTLoadVideoEncoders dlopen()s it in every process that creates a compression session and calls H264H7EncoderRegister.
# AMFI loads it only with its CDHash in the payload trust cache.
set -euo pipefail
OUT=${1:?usage: build-guamp-videoencoder.sh OUTDIR}
HERE=$(cd "$(dirname "$0")" && pwd)
SDK=$(xcrun --sdk iphoneos --show-sdk-path)
IDENTIFIER=org.guamp.videoencoder.H264H7
mkdir -p "$OUT"
DYLIB="$OUT/H264H7.videoencoder"
xcrun --sdk iphoneos clang -target arm64-apple-ios17.0 -isysroot "$SDK" -std=gnu17 -O2 \
  -Wall -Wextra -Werror -fvisibility=hidden \
  -dynamiclib -install_name /System/Library/VideoEncoders/H264H7.videoencoder \
  -framework CoreFoundation -o "$DYLIB" "$HERE/GuampH264SW.c"
xcrun strip -x "$DYLIB"
# An ordinary arm64 (A10-class userland, not arm64e) dylib exporting the name VTLoadVideoEncoders dlsym()s
[ "$(lipo -archs "$DYLIB")" = arm64 ]
grep -q ' _H264H7EncoderRegister$' <<<"$(nm -gU "$DYLIB")" || { echo "entry point H264H7EncoderRegister not exported"; exit 1; }
codesign --force --sign - --timestamp=none --pagesize 4096 --identifier "$IDENTIFIER" "$DYLIB"
codesign --verify --strict "$DYLIB"
DETAIL=$(codesign --display --verbose=4 "$DYLIB" 2>&1)
grep -qx 'Page size=4096' <<<"$DETAIL" || { echo "the signature must use 4 KiB pages"; exit 1; }
CDHASH=$(sed -n 's/^CDHash=//p' <<<"$DETAIL")
[ ${#CDHASH} -eq 40 ] || { echo "no CDHash"; exit 1; }
printf '{\n  "file": "H264H7.videoencoder",\n  "identifier": "%s",\n  "cdhash": "%s",\n  "cdhashes": ["%s"]\n}\n' \
  "$IDENTIFIER" "$CDHASH" "$CDHASH" > "$OUT/trust.json"
echo "H264H7.videoencoder built, CDHash $CDHASH"
