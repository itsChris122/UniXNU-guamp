#!/usr/bin/env python3
"""Vendor sensor configuration for the ADSP sensors PD as an ADT property (optional; GuampSnsFiles.h loadBlob).

The FastRPC listener (GuampSnsRpc.inc) serves the persist registry by itself and synthesizes sns_reg_config. The json
files of /vendor/etc/sensors/config are only parsed by the PD when sns_reg_config's version differs from the persist
registry's (both 4 on guamp), e.g. after a factory reset of persist. For that case this tool packs, from the vendor
backup, ONLY configuration data: etc/sensors/sns_reg_config and etc/sensors/config/*.json (byte for byte).
Anything else (libraries, firmware, any file that is not plain json/config text, any file mentioning Apple) is refused.
The result never goes into the repository: the property lives in the locally built ADT.

  guamp-sensors-config.py --vendor vendor.tar|vendor-dir --out blob.bin [--adt-in in.adt --adt-out out.adt]

Blob: "GSNSCFG1", u32 count, per file u32 path length, u32 data length, path, data (each padded to 4 bytes).
ADT: the blob is added as /chosen/guamp-sensors-config (an existing one is replaced).
"""
import argparse, struct, tarfile
from pathlib import Path

ALLOWED_DIR = 'etc/sensors/config/'
REG_CONFIG = 'etc/sensors/sns_reg_config'
PROP = b'guamp-sensors-config'


def vendor_files(src):
    """(relative path, bytes) of the allowed files, from a tar of /vendor or an extracted vendor directory."""
    out = []
    p = Path(src)
    if p.is_dir():
        for f in sorted(p.rglob('*')):
            if f.is_file():
                out.append((str(f.relative_to(p)).replace('\\', '/'), f))
        read = lambda f: f.read_bytes()
    else:
        tf = tarfile.open(src)
        for m in tf.getmembers():
            if m.isfile():
                out.append((m.name[2:] if m.name.startswith('./') else m.name, m))
        read = lambda m: tf.extractfile(m).read()
    files = []
    for rel, h in out:
        if rel == REG_CONFIG or (rel.startswith(ALLOWED_DIR) and rel.endswith('.json') and '/' not in rel[len(ALLOWED_DIR):]):
            files.append((rel, read(h)))
    return files


def check_and_minify(rel, data):
    if b'\0' in data:
        raise SystemExit(f'{rel}: binary content refused')
    text = data.decode('utf-8')
    if 'apple' in text.lower():
        raise SystemExit(f'{rel}: mentions Apple, refused (only Qualcomm/Motorola configuration data is packed)')
    if rel.endswith('.json'):
        # kept byte for byte: the SEE parser accepts what strict json does not (trailing commas)
        if not text.lstrip().startswith('{') or not text.rstrip().endswith('}'):
            raise SystemExit(f'{rel}: not a json object')
        return data
    for line in text.splitlines():
        if line.strip() and '=' not in line:
            raise SystemExit(f'{rel}: unexpected line {line!r}')
    return data


def blob(files):
    b = bytearray(b'GSNSCFG1' + struct.pack('<I', len(files)))
    for rel, data in files:
        path = ('/vendor/' + rel).encode()
        assert len(path) < 128, path
        b += struct.pack('<II', len(path), len(data)) + path
        b += bytes(-len(b) % 4) + data + bytes(-len(data) % 4)
    return bytes(b)


# ---- ADT: node = u32 nprops, u32 nchildren, props (name[32], u32 length | flags, value padded to 4), children ----
def parse_node(d, at):
    nprops, nchildren = struct.unpack_from('<II', d, at)
    start = at
    at += 8
    props = []
    for _ in range(nprops):
        name = d[at:at + 32].split(b'\0')[0]
        length = struct.unpack_from('<I', d, at + 32)[0]
        n = length & 0x7fffffff
        props.append((name, at, at + 36 + ((n + 3) & ~3), d[at + 36:at + 36 + n]))
        at += 36 + ((n + 3) & ~3)
    props_end = at
    children = []
    for _ in range(nchildren):
        child, at = parse_node(d, at)
        children.append(child)
    return {'start': start, 'props': props, 'props_end': props_end, 'children': children, 'end': at,
            'name': next((v.split(b'\0')[0] for (k, _, _, v) in props if k == b'name'), b'')}, at


def add_to_chosen(adt, value):
    root, end = parse_node(adt, 0)
    if end != len(adt):
        raise SystemExit(f'ADT: parsed {end} of {len(adt)} bytes')
    chosen = next((c for c in root['children'] if c['name'] == b'chosen'), None)
    if not chosen:
        raise SystemExit('ADT: no /chosen node')
    prop = PROP.ljust(32, b'\0') + struct.pack('<I', len(value)) + value + bytes(-len(value) % 4)
    old = next((p for p in chosen['props'] if p[0] == PROP), None)
    d = bytearray(adt)
    if old:
        d[old[1]:old[2]] = prop
    else:
        d[chosen['props_end']:chosen['props_end']] = prop
        nprops = struct.unpack_from('<I', d, chosen['start'])[0]
        struct.pack_into('<I', d, chosen['start'], nprops + 1)
    parse_node(bytes(d), 0)   # still well formed
    return bytes(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--vendor', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--adt-in')
    ap.add_argument('--adt-out')
    a = ap.parse_args()
    files = [(rel, check_and_minify(rel, data)) for rel, data in vendor_files(a.vendor)]
    if not any(rel == REG_CONFIG for rel, _ in files) or len(files) < 2:
        raise SystemExit(f'expected sns_reg_config and the config/*.json, found {len(files)} files')
    b = blob(files)
    Path(a.out).write_bytes(b)
    print(f'{a.out}: {len(files)} files, {len(b)} bytes ({sum(len(x) for _, x in files)} of data)')
    if a.adt_in:
        out = add_to_chosen(Path(a.adt_in).read_bytes(), b)
        Path(a.adt_out or a.adt_in).write_bytes(out)
        print(f'{a.adt_out or a.adt_in}: /chosen/{PROP.decode()} {len(b)} bytes, ADT {len(out)} bytes')


if __name__ == '__main__':
    main()
