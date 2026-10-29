#!/bin/bash
# Compone Bridge.app (Watch: originale XS 17.7 + 2 eseguibili convertiti + 6 framework convertiti da Codex) in ~/moto/watch/Bridge.app
# e la firma ad-hoc con ldid. Strada (c): i 6 framework che sull'XS esistono solo nella cache condivisa vanno in Bridge.app/Frameworks
# (bundle originali XS + binario convertito) e Bridge li carica via @rpath (LC_RPATH @executable_path/Frameworks c'è già);
# scripts/macho-rpath-frameworks.py riscrive i 6 load command al loro posto (nome più corto, nessun offset si sposta).
# Tolti per lo spazio della FAT di boot: filmati e immagini di fine abbinamento (DeviceAssets Outro1/Outro2/Screen/WatchSideBySide,
# VictoryBackground 926h/844h@3x.png) e il modello beam_bridge (riconoscimento visivo dell'orologio); tolti dai framework i 3
# eseguibili arm64e non convertiti (nanobackupd, subridged, XPC ppNotifierService). Info.plist senza SBAppTags watch-companion.
# Ordine di firma: framework, estensione, app (superficiale); flag adhoc (cd-flag-adhoc.py). File Apple solo su questo PC.
set -euo pipefail
L=~/moto/tools/ldid/ldid_linux_x86_64
M=/mnt/c/Users/Chris/Desktop/Moto
O=$M/app-da-convertire/Bridge.app
C=$M/app-da-convertire/convertite/Bridge.app
LIB=$M/app-da-convertire/convertite/librerie-watch
XSF=~/moto/watch/xs-frameworks          # copie dei bundle XS /System/Library/PrivateFrameworks/<N>.framework (dal mount in sola lettura)
W=~/moto/watch
RP=${RP:-$HOME/moto/wt-watch/scripts/macho-rpath-frameworks.py}
FLAG="python3 $M/strumenti/cd-flag-adhoc.py"
FW="NanoAudioControl SoftwareUpdateBridge BridgeCommons BridgeReporting NanoBackup PairingProximity"
X=PlugIns/GreenfieldThumbnailExtension.appex
EXES="Bridge $X/GreenfieldThumbnailExtension"
mkdir -p $W/ent
rm -rf $W/Bridge.app
cp -r $O $W/Bridge.app
chmod -R u+w $W/Bridge.app
cd $W/Bridge.app
rm -rf DeviceAssets/Outro1* DeviceAssets/Outro2* DeviceAssets/Screen* DeviceAssets/WatchSideBySide* \
  VictoryBackground-926h@3x.png VictoryBackground-844h@3x.png beam_bridge_2022_v2.mlmodelc
for f in $EXES; do
  $L -e $O/$f > $W/ent/$(basename $f).xml
  cp $C/$f $f
  chmod 0755 $f
done
python3 -I $RP Bridge $FW --need-rpath
python3 - <<'PY'
import plistlib
p = 'Info.plist'
d = plistlib.load(open(p, 'rb'))
assert d.pop('SBAppTags') == ['watch-companion']
plistlib.dump(d, open(p, 'wb'), fmt=plistlib.FMT_BINARY)
PY
mkdir -p Frameworks
for n in $FW; do
  cp -r $XSF/$n.framework Frameworks/
  rm -rf Frameworks/$n.framework/_CodeSignature Frameworks/$n.framework/nanobackupd Frameworks/$n.framework/Support \
    Frameworks/$n.framework/XPCServices
  cp $LIB/$n Frameworks/$n.framework/$n
  chmod 0755 Frameworks/$n.framework/$n
  $L -S Frameworks/$n.framework
  $FLAG Frameworks/$n.framework/$n
done
$L -S$W/ent/GreenfieldThumbnailExtension.xml $X
$FLAG $X/GreenfieldThumbnailExtension
$L -w -S$W/ent/Bridge.xml .
$FLAG Bridge
for f in $EXES; do
  $L -e $f | cmp - $W/ent/$(basename $f).xml
done
for f in $EXES $(for n in $FW; do echo Frameworks/$n.framework/$n; done); do
  echo "$f $($L -h $f | grep -E '^CDHash=' | cut -d= -f2) $($L -h $f | grep CodeDirectory)"
done
