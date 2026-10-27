#!/bin/bash
# Records, in the guamp device repository, the GPU modules that override UniXNU's pinned ones (unixnu feeds: the device
# repository's drivers/ and kits/ win over the base repository's):
#   drivers/NativeAdreno  xVanTuring/Adreno610 81a0b01: UniXNU/Adreno740 main 5c8c145 merged into the tb128fu branch, which adds
#                         the A6XX generation (ring, CP init, interrupt decode; commit 3ef7068) the Adreno 610 needs. 81a0b01
#                         still includes ../NativeMetal/OriginalAcceleratorABI.h (the base's NativeMetal has it); 4384823
#                         moved that include to ../AdrenoMetal.
#   kits/ArmSmmu4K        xVanTuring/ArmSmmu4K b464afc: UniXNU/ArmSmmu4K main merged into tb128fu, the kit Adreno610 is built
#                         with (the base pins 4d0d659, three commits older, without ArmSmmu4KLiveMapContextBatchExclusive).
# kits/AdrenoPM4 stays the base's (445f8fe; upstream main only adds documentation).
# Run once from the repository root, review "git status", then commit .gitmodules and the two gitlinks.
set -euo pipefail
cd "$(dirname "$0")/.."
add() {   # path url commit
  if [ ! -e "$1/.git" ]; then
    git -c core.autocrlf=false submodule add "$2" "$1" && git -C "$1" config core.autocrlf false
  fi
  git -C "$1" fetch --depth 50 origin "$3" 2>/dev/null || git -C "$1" fetch origin
  git -C "$1" checkout --detach "$3"
  git add "$1"
}
add drivers/NativeAdreno https://github.com/xVanTuring/Adreno610.git 81a0b01d0c434e3753213f312ae824d2f546e73f
add kits/ArmSmmu4K https://github.com/xVanTuring/ArmSmmu4K.git b464afc93869a239a01c697a912d5d6ff789f27c
git add .gitmodules
git status --short .gitmodules drivers/NativeAdreno kits/ArmSmmu4K
