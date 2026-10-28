#!/bin/bash
# Builds GuampAudio.driver for iPadOS 17 arm64 and writes its CDHash for the owner trust cache. NOT run yet (step 2 of the
# iOS audio plan only needs the source); intended for the macOS runner of the private build repo, like the Adreno Metal
# plugin (Xcode toolchain, iPhoneOS SDK). Usage: build-guamp-audio-plugin.sh OUTDIR
#   OUTDIR/GuampAudio.driver/{Info.plist,GuampAudio}   bundle, ad hoc signed (no team, no entitlements: the HAL loads it
#                                                       into audiomxd, whose own entitlements apply)
#   OUTDIR/trust.json                                  {"cdhashes": [...]} for make-owner-trust-bundle.py
# Placement on the phone: the NGPUADD2 addon (scripts/make-guamp-gpu-addon.py style) carries it as
# audio/HAL/GuampAudio.driver/* below /private/var/ngpu, and the kernel's boot-time bindfs union (guamp-audio-plugin=1,
# DESIGN.md) shows it inside /System/Library/Audio/Plug-Ins/HAL. AMFI accepts it only with its CDHash in the trust cache.
set -euo pipefail
OUT=${1:?usage: build-guamp-audio-plugin.sh OUTDIR}
HERE=$(cd "$(dirname "$0")" && pwd)
SDK=$(xcrun --sdk iphoneos --show-sdk-path)
BUNDLE="$OUT/GuampAudio.driver"
rm -rf "$BUNDLE"
mkdir -p "$BUNDLE"
xcrun --sdk iphoneos clang -target arm64-apple-ios17.0 -isysroot "$SDK" -std=gnu17 -O2 -Wall -Wextra -Werror \
  -Wno-four-char-constants -Wno-multichar -fvisibility=hidden -bundle \
  -framework CoreFoundation -o "$BUNDLE/GuampAudio" "$HERE/GuampAudio.c"
cp "$HERE/Info.plist" "$BUNDLE/Info.plist"
plutil -lint "$BUNDLE/Info.plist"
codesign --force --sign - --timestamp=none "$BUNDLE"
codesign --verify --strict "$BUNDLE"
CDHASH=$(codesign -dvvv "$BUNDLE" 2>&1 | sed -n 's/^CDHash=//p')
[ ${#CDHASH} -eq 40 ] || { echo "no CDHash"; exit 1; }
printf '{\n  "bundle": "GuampAudio.driver",\n  "cdhashes": ["%s"]\n}\n' "$CDHASH" > "$OUT/trust.json"
echo "GuampAudio.driver built, CDHash $CDHASH"
