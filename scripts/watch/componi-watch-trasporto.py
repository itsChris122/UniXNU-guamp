#!/usr/bin/env python3
"""Da WSL (29/10, richiesta utente: Watch completa, nessun file tolto): compone
  - gli addon di TRASPORTO (analisi/gpu-ci/build/watch-trasporto/N/): solo i file grandi di Bridge-full.app (filmati e immagini di
    abbinamento, modello beam_bridge) divisi in parti che stanno ognuna nella FAT di boot accanto al kernel; la seconda parte porta anche
    iphone-res/Ringtones/Reflection.m4r. Il kernel scrive in Data /private/var/ngpu ogni voce e non cancella mai quelle assenti:
    le immagini di trasporto si avviano una volta (ordine 1, 2), poi i file restano in Data;
  - l'addon NORMALE resa14b-watch = resa13-audio + Bridge-full.app SENZA i file grandi (stessi byte firmati: _CodeSignature/CodeResources
    è quello del bundle completo, quindi sigilla anche i file arrivati col trasporto) + Reflection.m4r;
  - una sola trust cache (copiata in ogni cartella) = moduli della base + gli 8 CDHash di Bridge-full nel modulo owner.

  python3 componi-watch-trasporto.py [--app ~/moto/watch/Bridge-full.app] [--part-limit 40000000]
Si ferma se le uscite esistono. File Apple solo su questo PC."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import struct
import zlib

here = Path(__file__).resolve().parent
moto = here.parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


compose = load('compose', Path.home() / 'moto/wt-watch/scripts/compose-guamp-audio-addon.py')
mgga = load('mgga', moto / 'Codex/sorgenti-ram-nc-posted/scripts/make-guamp-gpu-addon.py')

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument('--app', type=Path, default=Path.home() / 'moto/watch/Bridge-full.app')
parser.add_argument('--ringtone', type=Path, default=moto / 'analisi/suonerie/Ringtones/Reflection.m4r')
parser.add_argument('--base', type=Path, default=moto / 'analisi/gpu-ci/build/40794769537-resa13-audio')
parser.add_argument('--part-limit', type=int, default=40_000_000, help='byte massimi di un addon di trasporto')
options = parser.parse_args()
build = moto / 'analisi/gpu-ci/build'
normal_dir = build / '40794769537-resa14b-watch'
transport_dir = build / 'watch-trasporto'
assert not normal_dir.exists() and not transport_dir.exists(), 'uscite già presenti'

frameworks = ['NanoAudioControl', 'SoftwareUpdateBridge', 'BridgeCommons', 'BridgeReporting', 'NanoBackup', 'PairingProximity']
executables = ['Bridge', 'PlugIns/GreenfieldThumbnailExtension.appex/GreenfieldThumbnailExtension'] + \
    [f'Frameworks/{n}.framework/{n}' for n in frameworks]
BIG_PREFIXES = ('DeviceAssets/Outro1', 'DeviceAssets/Outro2', 'DeviceAssets/Screen', 'DeviceAssets/WatchSideBySide',
                'VictoryBackground-926h@3x.png', 'VictoryBackground-844h@3x.png', 'beam_bridge_2022_v2.mlmodelc/')
prefix = 'iphone-res/Applications/Bridge.app/'
ring = ('iphone-res/Ringtones/Reflection.m4r', 0o644, options.ringtone.read_bytes())

digests = {}
for name in executables:
    digest, page = compose.cdhash(options.app / name)
    assert page == 4096, name
    digests[name] = digest

big, small = [], []
for path in sorted(p for p in options.app.rglob('*') if p.is_file()):
    rel = path.relative_to(options.app).as_posix()
    data = path.read_bytes()
    mode = 0o755 if rel in executables else 0o644
    assert (data[:4] == b'\xcf\xfa\xed\xfe') == (mode == 0o755), rel
    entry = (prefix + rel, mode, data)
    (big if rel.startswith(BIG_PREFIXES) else small).append(entry)
assert sum(m == 0o755 for _, m, _ in small) == len(executables) and not any(m == 0o755 for _, m, _ in big)
assert any(n.endswith('/_CodeSignature/CodeResources') and n.count('/') == 4 for n, _, _ in small)


def pack(entries):
    body = b''
    for name, mode, data in entries:
        encoded = name.encode() + bytes(1)
        assert len(encoded) <= 160 and '..' not in name.split('/') and not name.startswith('/'), name
        packed = zlib.compress(data, 9)
        body += struct.pack('<IIQQ', len(encoded), mode, len(data), len(packed)) + mgga.pad(encoded) + mgga.pad(packed)
    assert 0 < len(entries) <= 4096
    addon = mgga.MAGIC + struct.pack('<IIQ', len(entries), 0, 24 + len(body)) + body
    assert len(addon) <= 64 * 1024 * 1024, 'oltre il limite del loader (64 MiB)'
    return addon


def write(folder, entries, trust):
    folder.mkdir(parents=True)
    addon = pack(entries)
    (folder / 'kernel.macho.gpuaddon').write_bytes(addon)
    assert [(n, m) for n, m, _ in mgga.read_addon(folder / 'kernel.macho.gpuaddon')] == [(n, m) for n, m, _ in entries]
    (folder / 'kernel.macho.trustcache').write_bytes(trust)
    return dict(folder=str(folder), entries=len(entries), raw_bytes=sum(len(d) for _, _, d in entries), gpuaddon_bytes=len(addon),
                gpuaddon_sha256=hashlib.sha256(addon).hexdigest())


# trust cache: base + 8 CDHash di Bridge-full
base_trust = (options.base / 'kernel.macho.trustcache').read_bytes()
parts = compose.modules(base_trust)
namespace = compose.owner_namespace(parts[0])
assert namespace
hashes = compose.module_hashes(parts[0])
assert not set(digests.values()) & set(hashes)
parts[0] = compose.owner_module(hashes + list(digests.values()), namespace)
trust = compose.bundle_of(parts)
assert compose.modules(trust)[1:] == compose.modules(base_trust)[1:]

# parti di trasporto: riempimento per dimensione compressa, la suoneria nell'ultima
sized = sorted(((len(zlib.compress(d, 9)), (n, m, d)) for n, m, d in big), key=lambda x: -x[0])
bins = []
for size, entry in sized:
    for b in bins:
        if b[0] + size + 512 <= options.part_limit:
            b[0] += size + 512
            b[1].append(entry)
            break
    else:
        bins.append([size + 512, [entry]])
bins[-1][1].append(ring)
reports = []
for i, (_, entries) in enumerate(bins, 1):
    reports.append(write(transport_dir / str(i), sorted(entries), trust))

# addon normale
entries = mgga.read_addon(options.base / 'kernel.macho.gpuaddon')
names = {n for n, _, _ in entries}
for e in small + [ring]:
    assert e[0] not in names, e[0]
normal = write(normal_dir, entries + small + [ring], trust)
normal['growth'] = normal['gpuaddon_bytes'] - (options.base / 'kernel.macho.gpuaddon').stat().st_size

report = dict(app=str(options.app), base=str(options.base), cdhash={k: v.hex() for k, v in digests.items()},
              owner_cdhashes=len(hashes) + len(digests), trustcache_bytes=len(trust),
              trustcache_sha256=hashlib.sha256(trust).hexdigest(), transport=reports, normal=normal,
              big_files=len(big), big_raw_bytes=sum(len(d) for _, _, d in big))
for folder in [normal_dir, transport_dir]:
    (folder / 'watch-trasporto.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
