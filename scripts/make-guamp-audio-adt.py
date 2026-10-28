#!/usr/bin/env python3
"""Audio ADT variant for the iOS "null audio device" step (package/GuampAudio): adds to a guamp ADT the /product audio facts of
the iPad7,1 (J120) device tree of the same IPSW, taken byte for byte from DeviceTree.j120ap.im4p, and nothing else.

  make-guamp-audio-adt.py --base BASE/kernel.macho.adt --j120 DeviceTree.j120ap.im4p [--lzfse PATH] OUTDIR

Added (only if absent in BASE; the script refuses a BASE that already has a different value):
  /product: builtin-mics, has-boot-chime, hearingaid-audio-equalization, hearingaid-low-energy-audio
  /product/audio (new node): name, wireless-splitter
Deliberately NOT added:
  /product/audio supports-always-listening (Hey Siri on the AOP low-power microphone: guamp has no AOP and no LP mic,
    corespeechd/VoiceTrigger would look for BuiltinAudioPlugin's ASDTIOPAudioLPMicDevice), the speaker-* tuning keys
    (J120 speaker calibration, meaningless for the aw882xx speaker), and every MCA/I2S/SIO/codec/amp/PDM node (they would
    match AppleAE2Audio, AppleSmartIO, AppleCS42L83Audio, AppleMAX98723Amp... which drive Apple hardware).
Writes OUTDIR/kernel.macho.adt and OUTDIR/configurazione.json (sha256 of base, J120 source and result, list of changes).
The ADT keeps every other byte; the result is re-parsed and compared key by key with the base."""
import argparse
import hashlib
import json
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

PRODUCT_KEYS = ('builtin-mics', 'has-boot-chime', 'hearingaid-audio-equalization', 'hearingaid-low-energy-audio')
AUDIO_KEYS = ('wireless-splitter',)


class Node:
    def __init__(self):
        self.props = []      # [name, size word (flags kept), value bytes]
        self.children = []

    def get(self, key):
        return next((p for p in self.props if p[0] == key), None)

    def name(self):
        p = self.get('name')
        return p[2].rstrip(b'\0').decode() if p else ''

    def child(self, name):
        return next((c for c in self.children if c.name() == name), None)


def parse(data):
    def node(pos):
        n = Node()
        nprops, nchildren = struct.unpack_from('<II', data, pos)
        if nprops > 4096 or nchildren > 4096:
            sys.exit(f'ADT: bad node at {pos:#x}')
        pos += 8
        for _ in range(nprops):
            raw, word = struct.unpack_from('<32sI', data, pos)
            pos += 36
            size = word & 0x7fffffff
            n.props.append([raw.rstrip(b'\0').decode(), word, data[pos:pos + size]])
            pos += (size + 3) & ~3
        for _ in range(nchildren):
            c, pos = node(pos)
            n.children.append(c)
        return n, pos
    root, end = node(0)
    if end != len(data):
        sys.exit(f'ADT: {len(data) - end} trailing bytes')
    return root


def serialize(n):
    out = bytearray(struct.pack('<II', len(n.props), len(n.children)))
    for key, word, value in n.props:
        out += struct.pack('<32sI', key.encode(), word) + value + bytes(-len(value) % 4)
    for c in n.children:
        out += serialize(c)
    return bytes(out)


def flatten(n, path='', out=None):
    out = {} if out is None else out
    here = path + '/' + n.name()
    for key, word, value in n.props:
        out[here + '/' + key] = (word, value)
    for c in n.children:
        flatten(c, here, out)
    return out


def j120_tree(im4p, lzfse):
    blob = im4p.read_bytes()
    at = blob.find(b'bvx2')
    if at < 0:
        sys.exit(f'{im4p}: no lzfse payload')
    with tempfile.TemporaryDirectory() as work:
        src, dst = Path(work) / 'adt.lzfse', Path(work) / 'adt'
        src.write_bytes(blob[at:])
        subprocess.run([lzfse, '-decode', '-i', str(src), '-o', str(dst)], check=True)
        return parse(dst.read_bytes())


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', required=True, type=Path)
    ap.add_argument('--j120', required=True, type=Path)
    ap.add_argument('--lzfse', default=str(Path.home() / 'moto/tools/lzfse/build/bin/lzfse'))
    ap.add_argument('outdir', type=Path)
    args = ap.parse_args()
    base_bytes = args.base.read_bytes()
    base, j120 = parse(base_bytes), j120_tree(args.j120, args.lzfse)
    if serialize(base) != base_bytes:
        sys.exit('base ADT does not round-trip')
    src_product = j120.child('product')
    src_audio = src_product.child('audio') if src_product else None
    product = base.child('product')
    if not src_product or not src_audio or not product:
        sys.exit('J120 /product/audio or base /product missing')
    changes = []
    for key in PRODUCT_KEYS:
        want = src_product.get(key)
        if not want:
            sys.exit(f'J120 /product/{key} missing')
        have = product.get(key)
        if have and have[2] != want[2]:
            sys.exit(f'base /product/{key} already set to {have[2].hex()}, J120 {want[2].hex()}')
        if not have:
            product.props.append(list(want))
            changes.append(f'/product/{key} = {want[2].hex()}')
    audio = product.child('audio')
    if audio:
        sys.exit('base already has /product/audio')
    audio = Node()
    audio.props.append(list(src_audio.get('name')))
    for key in AUDIO_KEYS:
        want = src_audio.get(key)
        if not want:
            sys.exit(f'J120 /product/audio/{key} missing')
        audio.props.append(list(want))
        changes.append(f'/product/audio/{key} = {want[2].hex()}')
    product.children.append(audio)
    result = serialize(base)
    before, after = flatten(parse(base_bytes)), flatten(parse(result))
    added = sorted(set(after) - set(before))
    if set(before) - set(after) or any(before[k] != after[k] for k in before):
        sys.exit('an existing property changed')
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / 'kernel.macho.adt').write_bytes(result)
    meta = {'base': str(args.base), 'base_sha256': sha(base_bytes), 'j120_source': str(args.j120),
            'j120_sha256': sha(args.j120.read_bytes()), 'adt_sha256': sha(result), 'added': added, 'changes': changes,
            'not_added': ['/product/audio/supports-always-listening (no AOP low-power mic)', 'speaker-* tuning',
                          'MCA/I2S/SIO/codec/amp/PDM nodes (Apple audio kexts would match)'],
            'tool': 'scripts/make-guamp-audio-adt.py (worktree wt-audio2)'}
    (args.outdir / 'configurazione.json').write_text(json.dumps(meta, indent=2) + '\n')
    print('\n'.join(changes))
    print(f'{args.outdir / "kernel.macho.adt"}: {len(base_bytes)} -> {len(result)} bytes, {len(added)} keys added, sha256 {meta["adt_sha256"]}')


if __name__ == '__main__':
    main()
