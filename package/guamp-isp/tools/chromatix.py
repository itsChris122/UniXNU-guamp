#!/usr/bin/env python3
"""Estrae dai com.qti.tuned.*.bin di CamX (chromatix "Parameter Parser V2.0.0") i dati per guamp-isp e scrive un
file .gisp (vedi gisp_tuning in include/gisp.h).

Uso:
  chromatix.py TUNED.bin -o OUT.gisp [--cfa gbrg] [--sensor nome] [--json OUT.json]
  chromatix.py TUNED.bin --dump [REGEX]      tabella dei simboli (per il reverse)

Formato del contenitore (lo stesso dei sensormodule, vedi analisi/camera/tools/sensormodule.py):
  0x98: u32 offset, u32 n sezioni -> n x {u64 id, u64 offset, u64 size}; sezione 0 = simboli da 72 byte
  {u64 id, char name[32], i64 count, u64 x, u64 off, u64 size} (off relativo alla sezione 1 = dati).
Strutture ricostruite sul GM1ST (tutti i campi a 32 bit occupano 8 byte, tabelle f32 impacchettate e allineate a 8):
  mod_*_cct_data / mod_*_aec_data   iniziano con il trigger {f32 start, pad, f32 end, pad} (16 byte)
  demuxblklevel14 cct_data (48)     trigger + 4 x f32 (nero per canale, in unita' a 12 bit: 256 = 64 a 10 bit)
  lsc35 cct_data (3568)             trigger + 4 tabelle 17x13 f32 (R, Gr, Gb, B), ciascuna allineata a 8 (888 byte)
  cc13 cct_data (72 per elemento)   trigger + c_tab 9 x f32 (per righe, RGB) + k_tab 3 x f32 + pad
  gamma15 channel_data (3120)       3 x {u64 canale, 257 x f32 (uscita 0..255 per ingresso i * 4 su 1024)}
  refPtV1 (288)                     intestazione 0x30 byte, poi coppie f32 (R/G, B/G) allineate a 8, fino agli zeri
Le CCT dei punti AWB non sono nel blob: le stimo con una retta in mired su log(R/G / B/G) passante per il 2o punto
(D65, 6500 K) e il 8o (A, 2850 K), l'ordine abituale dei punti Qualcomm. Sono marcate come stime nel JSON.
"""
import argparse
import json
import math
import re
import struct
import sys

MAGIC, VERSION = 0x50534947, 1
LSC_MAX, MAX_CCM, MAX_REF = 1024, 8, 16
CFAS = {'rggb': 0, 'grbg': 1, 'gbrg': 2, 'bggr': 3, 'mono': 4}


class Chromatix:
    def __init__(self, path):
        d = self.d = open(path, 'rb').read()
        if not d.startswith(b'QTI Chromatix Header') or b'Parameter Parser V2.0.0' not in d[:0x60]:
            raise ValueError(f'{path}: non e\' un chromatix CamX V2')
        off, n = struct.unpack_from('<II', d, 0x98)
        secs = [struct.unpack_from('<QQQ', d, off + i * 24) for i in range(n)]
        _, so, ss = secs[0]
        self.base = secs[1][1]
        self.syms = []
        for p in range(so, so + ss, 72):
            sid, = struct.unpack_from('<Q', d, p)
            name = d[p + 8:p + 40].split(b'\0')[0].decode(errors='replace')
            cnt, x, o, s = struct.unpack_from('<qQQQ', d, p + 40)
            self.syms.append((sid, name, cnt, x, o, s))

    def find(self, name, size=None):
        return [s for s in self.syms if s[1] == name and (size is None or s[5] == size)]

    def f32(self, off):
        return struct.unpack_from('<f', self.d, self.base + off)[0]

    def u64(self, off):
        return struct.unpack_from('<Q', self.d, self.base + off)[0]

    def trigger(self, off):
        return self.f32(off), self.f32(off + 8)

    def table(self, off, n):
        return list(struct.unpack_from(f'<{n}f', self.d, self.base + off))


def extract(c):
    out = {'note': []}
    # nero
    bl = c.find('mod_demuxblklevel14_cct_data', 48)
    if bl:
        o = bl[0][4]
        v = [c.f32(o + 16 + 8 * i) for i in range(4)]
        out['black12'] = v
        out['black'] = [x / 4 for x in v]
    else:
        out['black'] = [64.0] * 4
        out['note'].append('nero non trovato: 64')
    # LSC
    ls = c.find('mod_lsc35_cct_data', 3568)
    out['lsc_sets'] = []
    for s in ls[:3]:
        o = s[4]
        tabs = [c.table(o + 16 + 888 * k, 221) for k in range(4)]
        out['lsc_sets'].append({'trigger': c.trigger(o), 'tables': tabs})
    # CCM: la lista piu' lunga di elementi da 72 byte nel primo blocco cc13
    cc = c.find('mod_cc13_cct_data')
    best = max(cc, key=lambda s: s[5]) if cc else None
    out['ccm'] = []
    if best:
        o, size = best[4], best[5]
        for e in range(size // 72):
            b = o + 72 * e
            lo, hi = c.trigger(b)
            m = c.table(b + 16, 9)
            out['ccm'].append({'cct': [lo, hi], 'm': m})
    # gamma
    g = c.find('mod_gamma15_channel_data', 3120)
    if g:
        o = g[0][4]
        chans = []
        for k in range(3):
            b = o + 1040 * k
            chans.append({'channel': c.u64(b), 'table': c.table(b + 8, 257)})
        out['gamma_channels'] = chans
    # AWB
    r = c.find('refPtV1')
    out['ref'] = []
    if r:
        o, size = r[0][4], r[0][5]
        vals = []
        p = 0x30
        while p + 16 <= size:
            a, b = c.f32(o + p), c.f32(o + p + 8)
            if a == 0 and b == 0:
                break
            vals.append((a, b))
            p += 16
        out['ref'] = vals
    return out


def estimate_cct(refs):
    """mired = a + b * log(rg / bg), con il punto 1 a 6500 K e il punto 7 a 2850 K"""
    if len(refs) < 8:
        return [0.0] * len(refs)
    t = [math.log(rg / bg) for rg, bg in refs]
    m1, m7 = 1e6 / 6500, 1e6 / 2850
    b = (m7 - m1) / (t[7] - t[1])
    a = m1 - b * t[1]
    out = []
    for x in t:
        m = a + b * x
        out.append(round(1e6 / m) if m > 50 else 20000.0)
    return out


def build(info, sensor, cfa):
    blob = bytearray()
    blob += sensor.encode()[:31].ljust(32, b'\0')
    blob += struct.pack('<I', cfa)
    blob += struct.pack('<4f', *info['black'])
    blob += struct.pack('<f', 1023.0)
    if info['lsc_sets']:
        tabs = info['lsc_sets'][0]['tables']
        blob += struct.pack('<II', 17, 13)
        for k in range(4):
            blob += struct.pack(f'<{LSC_MAX}f', *(tabs[k] + [0.0] * (LSC_MAX - 221)))
    else:
        blob += struct.pack('<II', 0, 0) + b'\0' * (4 * LSC_MAX * 4)
    ccm = info['ccm'][:MAX_CCM]
    blob += struct.pack('<I', len(ccm))
    ccts = [(e['cct'][0] + e['cct'][1]) / 2 for e in ccm]
    blob += struct.pack(f'<{MAX_CCM}f', *(ccts + [0.0] * (MAX_CCM - len(ccts))))
    for i in range(MAX_CCM):
        blob += struct.pack('<9f', *(ccm[i]['m'] if i < len(ccm) else [0.0] * 9))
    refs = info['ref'][:MAX_REF]
    rc = estimate_cct(refs)
    blob += struct.pack('<I', len(refs))
    pad = [0.0] * (MAX_REF - len(refs))
    blob += struct.pack(f'<{MAX_REF}f', *([r[0] for r in refs] + pad))
    blob += struct.pack(f'<{MAX_REF}f', *([r[1] for r in refs] + pad))
    blob += struct.pack(f'<{MAX_REF}f', *(rc + pad))
    if info.get('gamma_channels'):
        tab = info['gamma_channels'][0]['table']
        blob += struct.pack('<257f', *[v / 255.0 for v in tab])
    else:  # sRGB
        g = []
        for i in range(257):
            x = i / 256
            g.append(12.92 * x if x <= 0.0031308 else 1.055 * x ** (1 / 2.4) - 0.055)
        blob += struct.pack('<257f', *g)
    return struct.pack('<III', MAGIC, VERSION, len(blob)) + bytes(blob), rc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('tuned')
    ap.add_argument('-o')
    ap.add_argument('--cfa', default='gbrg')
    ap.add_argument('--sensor')
    ap.add_argument('--json')
    ap.add_argument('--dump', nargs='?', const='.')
    a = ap.parse_args()
    c = Chromatix(a.tuned)
    if a.dump:
        rx = re.compile(a.dump)
        for i, s in enumerate(c.syms):
            if rx.search(s[1]):
                print(i, s[0], s[1], s[2], s[3], hex(s[4]), s[5])
        return
    info = extract(c)
    sensor = a.sensor or re.sub(r'.*tuned\.(mot_guamp_)?|\.bin$', '', a.tuned)
    blob, rc = build(info, sensor, CFAS[a.cfa])
    if a.o:
        open(a.o, 'wb').write(blob)
    summ = {
        'sensor': sensor, 'black10': info['black'],
        'lsc': [{'trigger': s['trigger'], 'corner_max': [round(max(t), 3) for t in s['tables']],
                 'center': [round(t[6 * 17 + 8], 3) for t in s['tables']]} for s in info['lsc_sets']],
        'ccm': [{'cct': e['cct'], 'm': [round(x, 4) for x in e['m']]} for e in info['ccm']],
        'gamma': {'channels': [g['channel'] for g in info.get('gamma_channels', [])],
                  'samples': [round(info['gamma_channels'][0]['table'][i], 1) for i in (0, 16, 64, 128, 192, 256)]
                  if info.get('gamma_channels') else []},
        'awb_ref': [{'rg': round(r[0], 4), 'bg': round(r[1], 4), 'cct_stimata': rc[i]} for i, r in enumerate(info['ref'])],
        'note': info['note'],
    }
    if a.json:
        json.dump(summ, open(a.json, 'w'), indent=1)
    print(json.dumps(summ, indent=None)[:4000])


if __name__ == '__main__':
    sys.exit(main())
