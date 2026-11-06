#!/bin/bash
# Host tests of package/GuampKeybagCore.h (guamp-aks-keybag=1). Run from anywhere.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
for cc in "cc -std=c11" "c++ -x c++ -std=gnu++17"; do
  $cc -O1 -Wall -Wextra -Werror -fsanitize=address,undefined "$here/guamp_keybag_host.c" -o "$out/kb"
  "$out/kb" "$out/systembag.kb"
done
# keybagd reads systembag.kb with CFPropertyList (load_binary_dict): the file must be a valid binary plist
# { KeyBagKeys: <1096-byte guamp bag> }.
python3 -I - "$out/systembag.kb" <<'PY'
import plistlib, struct, sys
d = plistlib.load(open(sys.argv[1], 'rb'))
assert list(d) == ['KeyBagKeys'] and len(d['KeyBagKeys']) == 1096, d.keys()
magic, version, typ, classes = struct.unpack_from('<4I', d['KeyBagKeys'])
assert (magic, version, classes) == (0x3142474b, 1, 32), (hex(magic), version, classes)
print('PASS systembag.kb is a binary plist {KeyBagKeys: 1096 bytes}')
PY
echo "ALL KEYBAG TESTS PASSED"
