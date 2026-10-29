#!/usr/bin/env python3
"""Da WSL: addon + trust cache «resa14-watch» = resa13-audio + Watch (Bridge.app XS 17.7 convertita arm64 con i suoi 6 framework in
Bridge.app/Frameworks, firmata ad-hoc da strumenti/firma-watch.sh) + suoneria Reflection.m4r.

  python3 componi-watch-addon.py [--app ~/moto/watch/Bridge.app] [--base ...resa13-audio] [--out 40794769537-resa14-watch]

Voci aggiunte: iphone-res/Applications/Bridge.app/... (Mach-O 0755: app, estensione e 6 framework; il resto 0644), vista union
/Applications come Telefono/Wallet; iphone-res/Ringtones/Reflection.m4r (0644), nona vista su /Library/Ringtones (patch
zzzzzzzzzzzzzzzz-guamp-watch-ls-auto, ramo codex/watch-20261029). Trust: gli 8 CDHash uniti al modulo owner della base, moduli Apple
byte per byte. Si ferma se l'uscita esiste. File Apple solo su questo PC."""
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
parser.add_argument('--app', type=Path, default=Path.home() / 'moto/watch/Bridge.app')
parser.add_argument('--ringtone', type=Path, default=moto / 'analisi/suonerie/Ringtones/Reflection.m4r')
parser.add_argument('--base', type=Path, default=moto / 'analisi/gpu-ci/build/40794769537-resa13-audio')
parser.add_argument('--out', default='40794769537-resa14-watch')
options = parser.parse_args()
output = moto / 'analisi/gpu-ci/build' / options.out
assert not (output / 'kernel.macho.gpuaddon').exists(), 'già composto: ' + str(output)

frameworks = ['NanoAudioControl', 'SoftwareUpdateBridge', 'BridgeCommons', 'BridgeReporting', 'NanoBackup', 'PairingProximity']
executables = ['Bridge', 'PlugIns/GreenfieldThumbnailExtension.appex/GreenfieldThumbnailExtension'] + \
    [f'Frameworks/{n}.framework/{n}' for n in frameworks]
digests = {}
for name in executables:
    digest, page = compose.cdhash(options.app / name)
    assert page == 4096, name
    digests[name] = digest

entries = mgga.read_addon(options.base / 'kernel.macho.gpuaddon')
base_names = {n for n, _, _ in entries}
added = []
prefix = 'iphone-res/Applications/Bridge.app/'
for path in sorted(p for p in options.app.rglob('*') if p.is_file()):
    rel = path.relative_to(options.app).as_posix()
    name = prefix + rel
    assert name not in base_names, 'la base ha già ' + name
    data = path.read_bytes()
    mode = 0o755 if rel in executables else 0o644
    assert (data[:4] == b'\xcf\xfa\xed\xfe') == (mode == 0o755), 'Mach-O non previsto o eseguibile mancante: ' + rel
    entries.append((name, mode, data))
    added.append((name, mode, len(data)))
assert sum(1 for _, m, _ in added if m == 0o755) == len(executables)
ring = 'iphone-res/Ringtones/Reflection.m4r'
assert ring not in base_names
data = options.ringtone.read_bytes()
entries.append((ring, 0o644, data))
added.append((ring, 0o644, len(data)))

body = b''
for name, mode, data in entries:
    encoded = name.encode() + bytes(1)
    assert len(encoded) <= 160 and '..' not in name.split('/') and not name.startswith('/'), name
    packed = zlib.compress(data, 9)
    body += struct.pack('<IIQQ', len(encoded), mode, len(data), len(packed)) + mgga.pad(encoded) + mgga.pad(packed)
addon = mgga.MAGIC + struct.pack('<IIQ', len(entries), 0, 24 + len(body)) + body
assert len(entries) <= 4096
output.mkdir(parents=True, exist_ok=True)
(output / 'kernel.macho.gpuaddon').write_bytes(addon)
assert [(n, m) for n, m, _ in mgga.read_addon(output / 'kernel.macho.gpuaddon')] == [(n, m) for n, m, _ in entries]

base_trust = (options.base / 'kernel.macho.trustcache').read_bytes()
parts = compose.modules(base_trust)
namespace = compose.owner_namespace(parts[0])
assert namespace, 'la base non ha un modulo owner in testa'
hashes = compose.module_hashes(parts[0])
assert not set(digests.values()) & set(hashes)
parts[0] = compose.owner_module(hashes + list(digests.values()), namespace)
bundle = compose.bundle_of(parts)
assert compose.modules(bundle) == parts and compose.modules(bundle)[1:] == compose.modules(base_trust)[1:]
assert set(digests.values()) <= set(compose.module_hashes(compose.modules(bundle)[0]))
(output / 'kernel.macho.trustcache').write_bytes(bundle)

report = dict(base=str(options.base), cdhash={k: v.hex() for k, v in digests.items()}, owner_namespace=namespace,
              owner_cdhashes=len(hashes) + len(digests), added_files=len(added), added_bytes=sum(s for _, _, s in added),
              entries=len(entries), gpuaddon_bytes=len(addon),
              gpuaddon_growth=len(addon) - (options.base / 'kernel.macho.gpuaddon').stat().st_size,
              gpuaddon_sha256=hashlib.sha256(addon).hexdigest(),
              trustcache_bytes=len(bundle), trustcache_sha256=hashlib.sha256(bundle).hexdigest(),
              kernel='codex/watch-20261029 (nona vista /Library/Ringtones, guamp-ls-auto)')
(output / 'watch-addon.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
