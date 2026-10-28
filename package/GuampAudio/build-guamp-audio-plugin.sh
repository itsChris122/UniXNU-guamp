#!/bin/bash
# Builds GuampAudio.driver for iPadOS 17 arm64 and writes its CDHash for the owner trust cache. Runs on the macOS runner of the
# private build repo (guamp-kernel-build .github/workflows/audio-plugin.yml) or any Mac with Xcode: iPhoneOS SDK, clang, codesign.
# Usage: build-guamp-audio-plugin.sh OUTDIR
#   OUTDIR/GuampAudio.driver/{Info.plist,GuampAudio,_CodeSignature/}   flat iOS bundle, ad hoc signed with 4 KiB code pages
#                                                       (the guamp kernel runs 4K pages, as the Adreno Metal plugin), no team,
#                                                       no entitlements: the HAL loads it into audiomxd, whose own apply
#   OUTDIR/trust.json                                   {"bundle", "identifier", "cdhash", "cdhashes": [...]}
# Placement on the phone: the NGPUADD2 addon (scripts/make-guamp-gpu-addon.py --audio OUTDIR) carries it as
# audio/HAL/GuampAudio.driver/* below /private/var/ngpu, and the kernel's boot-time bindfs union (guamp-audio-plugin=1,
# patches/xnu/zzzzz-guamp-audio-plugin.patch) shows it inside /System/Library/Audio/Plug-Ins/HAL. AMFI accepts it only with its
# CDHash in the owner trust cache.
set -euo pipefail
OUT=${1:?usage: build-guamp-audio-plugin.sh OUTDIR}
HERE=$(cd "$(dirname "$0")" && pwd)
SDK=$(xcrun --sdk iphoneos --show-sdk-path)
IDENTIFIER=com.guamp.audio.GuampAudio
BUNDLE="$OUT/GuampAudio.driver"
rm -rf "$BUNDLE"
mkdir -p "$BUNDLE"
xcrun --sdk iphoneos clang -target arm64-apple-ios17.0 -isysroot "$SDK" -std=gnu17 -O2 -Wall -Wextra -Werror \
  -Wno-four-char-constants -Wno-multichar -fvisibility=hidden -bundle \
  -framework CoreFoundation -o "$BUNDLE/GuampAudio" "$HERE/GuampAudio.c"
xcrun strip -x "$BUNDLE/GuampAudio"
# An ordinary arm64 (subtype 0, not arm64e: A10-class userland) Mach-O bundle exporting the factory named in Info.plist
[ "$(lipo -archs "$BUNDLE/GuampAudio")" = arm64 ]
grep -q BUNDLE <<<"$(otool -hv "$BUNDLE/GuampAudio")"
grep -q ' _GuampAudio_Create$' <<<"$(nm -gU "$BUNDLE/GuampAudio")"
cp "$HERE/Info.plist" "$BUNDLE/Info.plist"
plutil -lint "$BUNDLE/Info.plist"
[ "$(plutil -extract CFBundleIdentifier raw "$BUNDLE/Info.plist")" = "$IDENTIFIER" ]
codesign --force --sign - --timestamp=none --pagesize 4096 --identifier "$IDENTIFIER" "$BUNDLE"
codesign --verify --strict "$BUNDLE"
DETAIL=$(codesign --display --verbose=4 "$BUNDLE" 2>&1)
grep -qx 'Page size=4096' <<<"$DETAIL" || { echo "the signature must use 4 KiB pages"; exit 1; }
CDHASH=$(sed -n 's/^CDHash=//p' <<<"$DETAIL")
[ ${#CDHASH} -eq 40 ] || { echo "no CDHash"; exit 1; }
printf '{\n  "bundle": "GuampAudio.driver",\n  "identifier": "%s",\n  "cdhash": "%s",\n  "cdhashes": ["%s"]\n}\n' \
  "$IDENTIFIER" "$CDHASH" "$CDHASH" > "$OUT/trust.json"
echo "GuampAudio.driver built, CDHash $CDHASH"
