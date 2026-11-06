#!/usr/bin/env python3
"""MobileGestalt cellular capabilities on guamp (06/11, host test; no device needed).

The project answers MobileGestalt through the ADT (libMobileGestalt reads IODeviceTree and hw.product) plus guamp-mg-adt-sync
(zzzz-guamp-setup-skip.patch), which resets the MobileGestalt cache once for every new stamp of the ADT properties it caches.
This test holds the 21H461 logic of the keys that make CommCenter, Settings (Cellular row) and the status bar treat the device
as cellular, as read from libMobileGestalt.dylib 21H461 (answer table: obfuscated key -> function):

  HasBaseband                  an IODeviceTree entry named "baseband" exists (IOServiceNameMatching in the IODeviceTree plane)
  cellular-data                = HasBaseband   (Preferences, PreferencesUI, SettingsCellular, SystemStatusUI, SpringBoard, ...)
  contains-cellular-radio      = HasBaseband   (SystemStatusServer, SpringBoard)
  telephony                    DeviceClass iPhone -> true; Watch -> HasBaseband; any other class (iPad) -> false
  sms, CellularTelephonyCapability = telephony (PreferencesUI, SystemStatusServer, UIKitCore)
  mms                          DeviceClass == iPhone
  data-plan                    DeviceClass == iPad and HasBaseband (the iPad "cellular data plan" flow; false on every iPhone)
  DeviceClass                  prefix of hw.product (XNU: the root "model" of the ADT): iPhone 1, iPod 2, iPad 3, AppleTV 4, Watch 6
  BasebandClass                IODeviceTree:/baseband "class"
  EUICCChipID                  IODeviceTree:/baseband "euicc-chip-id" (absent: no eUICC)
  BasebandChipset              IODeviceTree:/product "baseband-chipset"
  personal-hotspot             IODeviceTree:/product "personal-hotspot" (07/11, Preferences: Personal Hotspot row; d22ap has it)
SIMStatus/SIMTrayStatus are asked to CommCenter at run time; "sim", "apn", "lte-device", "umts-device", "call-waiting" and the other
CoreTelephony words have no answer function in 21H461; no MobileGestalt key gives the number of SIM slots (CommCenter learns the
slots from the baseband). So the guamp answers that describe the real phone (SM6115 modem, two nano-SIM slots, voice and data,
no eSIM) follow from: a /baseband node, a root model of the iPhone class, no euicc-chip-id. No identity key is involved.

  check-guamp-mg-cellular.py                       self test (+ libMobileGestalt check if --mg or ~/moto/siri-mg is there)
  check-guamp-mg-cellular.py --adt ADT [--guamp]   answers of an ADT; --guamp asserts the guamp phone answers
  check-guamp-mg-cellular.py --mg libMobileGestalt.dylib   verifies the logic above against the 21H461 binary"""
import argparse
import base64
import hashlib
import importlib.util
import re
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / 'patches/xnu/zzzz-guamp-setup-skip.patch'
DEFAULT_MG = Path.home() / 'moto/siri-mg/libMobileGestalt.dylib'
CLASSES = (('iPhone', 1), ('iPod', 2), ('iPad', 3), ('AppleTV', 4), ('iFPGA', 5), ('Watch', 6))
PTR = 0xfffffffff          # chained-fixup pointers of the extracted dylib: target in the low 36 bits


def obf(key):
    return base64.b64encode(hashlib.md5(b'MGCopyAnswer' + key.encode()).digest()).decode()[:22]


def text(v):
    return v.split(b'\0')[0].decode() if v is not None else None


def u32(v):
    return struct.unpack_from('<I', v)[0] if v is not None and len(v) >= 4 else None


def answers(adt):
    """adt: {"/device-tree/...": bytes}. MobileGestalt 21H461 cellular answers."""
    model = text(adt.get('/device-tree/model'))
    cls = next((n for p, n in CLASSES if model and model.startswith(p)), 0)
    bb = any(k.endswith('/name') and v.split(b'\0')[0] == b'baseband' for k, v in adt.items())
    tel = cls == 1 or (cls == 6 and bb)
    return {'DeviceClassNumber': cls, 'HasBaseband': bb, 'cellular-data': bb, 'contains-cellular-radio': bb,
            'telephony': tel, 'sms': tel, 'CellularTelephonyCapability': tel, 'mms': cls == 1, 'data-plan': cls == 3 and bb,
            'BasebandClass': u32(adt.get('/device-tree/baseband/class')),
            'EUICCChipID': u32(adt.get('/device-tree/baseband/euicc-chip-id')),
            'BasebandChipset': text(adt.get('/device-tree/product/baseband-chipset')),
            'personal-hotspot': bool(u32(adt.get('/device-tree/product/personal-hotspot')))}


# The real phone: SM6115 modem with voice, SMS and data, two physical SIM slots, no eUICC.
GUAMP = {'DeviceClassNumber': 1, 'HasBaseband': True, 'cellular-data': True, 'contains-cellular-radio': True, 'telephony': True,
         'sms': True, 'CellularTelephonyCapability': True, 'mms': True, 'data-plan': False, 'EUICCChipID': None,
         'personal-hotspot': True}
# ADT properties the answers above come from: guamp-mg-adt-sync must stamp all of them.
STAMPED = {('/baseband', 'name'), ('/baseband', 'class'), ('/baseband', 'euicc-chip-id'), ('/product', 'baseband-chipset'),
           ('/', 'model'), ('/', 'model-number'), ('/', 'region-info'), ('/', 'regulatory-model-number'),
           ('/product', 'personal-hotspot')}


def check_guamp(a, where):
    bad = {k: (a[k], v) for k, v in GUAMP.items() if a[k] != v}
    assert not bad, f'{where}: not the guamp phone answers (got, want): {bad}'
    assert a['BasebandChipset'], f'{where}: /product/baseband-chipset missing (ABM picks its firmware family from it)'


def check_patch():
    s = PATCH.read_text()
    block = s[s.index('static const char *const props[][2] = {'):]
    block = block[:block.index('};')]
    pairs = re.findall(r'\{"([^"]+)", "([^"]+)"\}', block)
    missing = STAMPED - set(pairs)
    assert not missing, f'guamp-mg-adt-sync does not stamp {sorted(missing)}'
    assert len(set(pairs)) == len(pairs), 'duplicate stamp entries'
    return pairs


STAMP_PROG = r'''#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
typedef const char *DTEntry;
#define kSuccess 0
static const char *have_node, *have_prop, *have_value;
static int SecureDTLookupEntry(void *s, const char *p, DTEntry *e) { (void)s; *e = p; return kSuccess; }
static int SecureDTGetProperty(DTEntry e, const char *n, const void **v, unsigned *z) {
	if (!have_node || strcmp(e, have_node) || strcmp(n, have_prop)) return -1;
	*v = have_value; *z = (unsigned)strlen(have_value) + 1; return kSuccess; }
int main(int argc, char **argv) {
	if (argc == 4) { have_node = argv[1]; have_prop = argv[2]; have_value = argv[3]; }
@BLOCK@	printf("%08x %zu\n", stamp, sizeof(props) / sizeof(props[0]));
	return 0;
}
'''


def compile_stamp(pairs):
    """Builds the guamp-mg-adt-sync stamp loop of the patch on the host: every cellular property must move the stamp."""
    import shutil, subprocess, tempfile
    cc = shutil.which('cc') or shutil.which('clang') or shutil.which('gcc')
    if not cc:
        return False
    added = '\n'.join(l[1:] for l in PATCH.read_text().split('\n') if l.startswith('+') and not l.startswith('+++'))
    block = added[added.index('\t\tstatic const char *const props[][2]'):]
    block = block[:block.index('\t\tsnprintf(mg_marker')]
    with tempfile.TemporaryDirectory() as work:
        src, exe = Path(work) / 'stamp.c', Path(work) / 'stamp'
        src.write_text(STAMP_PROG.replace('@BLOCK@', block))
        subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror', str(src), '-o', str(exe)], check=True)
        run = lambda *a: subprocess.run([str(exe), *a], capture_output=True, text=True, check=True).stdout.split()
        empty, count = run()
        assert int(count) == len(pairs), (count, len(pairs))
        seen = {empty}
        for node, name in sorted(STAMPED):
            stamp = run(node, name, 'baseband' if name == 'name' else 'x')[0]
            assert stamp not in seen, f'{node} {name} does not change the stamp'
            seen.add(stamp)
    return True


def self_test():
    phone = {'/device-tree/model': b'iPhone10,3\0', '/device-tree/compatible': b'motorola,guamp\0qcom,bengal\0qcom,sm6115\0',
             '/device-tree/product/baseband-chipset': b'mav16\0', '/device-tree/baseband/name': b'baseband\0',
             '/device-tree/baseband/compatible': b'baseband,n41\0qcom,sm6115-mpss-pas\0',
             '/device-tree/baseband/class': struct.pack('<I', 3), '/device-tree/baseband/guamp,sim-slots': struct.pack('<I', 2),
             '/device-tree/product/personal-hotspot': struct.pack('<I', 1)}
    check_guamp(answers(phone), 'guamp ADT (07/11 layout)')
    fin49 = dict(phone, **{'/device-tree/compatible': b'J121AP\0motorola,guamp\0qcom,bengal\0qcom,sm6115\0',
                           '/device-tree/baseband/euicc-chip-id': struct.pack('<I', 2)})
    a = answers(fin49)
    assert a['telephony'] and a['cellular-data'] and a['EUICCChipID'] == 2, a          # fin49: cellular, with an eSIM it has not
    try:
        check_guamp(a, 'fin49')
        failed = False
    except AssertionError as e:
        failed = 'EUICCChipID' in str(e)
    assert failed, 'fin49 euicc-chip-id must fail the guamp check'
    no_bb = {k: v for k, v in phone.items() if '/baseband' not in k}
    a = answers(no_bb)
    assert not a['HasBaseband'] and not a['cellular-data'] and a['telephony'], a         # iPhone class, no baseband node
    j120 = answers({'/device-tree/model': b'iPad7,1\0'})                               # J120: Wi-Fi iPad
    assert j120['DeviceClassNumber'] == 3 and not any(v for k, v in j120.items() if k != 'DeviceClassNumber'), j120
    a = answers(dict(phone, **{'/device-tree/model': b'iPad7,2\0'}))                   # J121 (iPad7,2): data, no voice
    assert a['cellular-data'] and a['data-plan'] and not a['telephony'] and not a['sms'] and not a['mms'], a
    a = answers({'/device-tree/model': b'iPhone10,3\0', '/device-tree/arm-io/baseband/name': b'baseband\0'})
    assert a['HasBaseband'], a                                                            # matched by name, anywhere
    a = answers(dict(phone, **{'/device-tree/model': b'motorola,guamp\0'}))
    assert a['DeviceClassNumber'] == 0 and not a['telephony'] and a['cellular-data'], a  # non-Apple root model: no voice
    return compile_stamp(check_patch())


class Dylib:
    def __init__(self, path):
        self.d = Path(path).read_bytes()
        assert struct.unpack_from('<I', self.d)[0] == 0xfeedfacf, 'not a 64-bit Mach-O'
        ncmds, q, self.segs = struct.unpack_from('<I', self.d, 16)[0], 32, {}
        for _ in range(ncmds):
            cmd, size = struct.unpack_from('<II', self.d, q)
            if cmd == 0x19:
                self.segs[self.d[q + 8:q + 24].rstrip(b'\0').decode()] = struct.unpack_from('<QQQQ', self.d, q + 24)
            q += size

    def in_seg(self, va, name):
        va0, vs, _, _ = self.segs.get(name, (0, 0, 0, 0))
        return va0 <= va < va0 + vs

    def off(self, va):
        for va0, vs, fo, fs in self.segs.values():
            if va0 <= va < va0 + min(vs, fs):
                return fo + va - va0

    def cstr(self, va):
        o = self.off(va)
        return None if o is None else self.d[o:self.d.index(b'\0', o)].decode('latin1')

    def string(self, va):
        """C string in __TEXT, or the bytes of a CFString in __DATA_CONST."""
        if self.in_seg(va, '__TEXT'):
            return self.cstr(va)
        if self.in_seg(va, '__DATA_CONST'):
            ptr = struct.unpack_from('<Q', self.d, self.off(va) + 16)[0] & PTR
            return self.cstr(ptr) if self.in_seg(ptr, '__TEXT') else None

    def answer_fn(self, key):
        o = self.d.find(obf(key).encode(), 0, self.segs['__TEXT'][3])
        assert o >= 0, f'{key}: not in the answer table'
        va = self.segs['__TEXT'][0] + o
        _, _, fo, fs = self.segs['__DATA_CONST']
        for q in range(fo, fo + fs - 16, 8):
            if struct.unpack_from('<Q', self.d, q)[0] & PTR == va:
                fn = struct.unpack_from('<Q', self.d, q + 8)[0] & PTR
                if self.in_seg(fn, '__TEXT'):
                    return fn
        raise AssertionError(f'{key}: no answer function')

    def scan(self, fn, count=24):
        """Small ARM64 reader: strings loaded with adrp+add, cmp w0 immediates, branch targets; stops at b or ret."""
        regs, refs, cmps, calls = {}, [], [], []
        for i in range(count):
            pc = fn + 4 * i
            ins = struct.unpack_from('<I', self.d, self.off(pc))[0]
            rd, rn = ins & 31, (ins >> 5) & 31
            if ins & 0x9f000000 == 0x90000000:                                   # adrp
                imm = ((ins >> 5) & 0x7ffff) << 2 | (ins >> 29) & 3
                imm -= (imm & 1 << 20) << 1
                regs[rd] = (pc & ~0xfff) + (imm << 12)
            elif ins & 0xff800000 == 0x91000000 and rn in regs:                  # add xd, xn, #imm
                regs[rd] = regs[rn] + (((ins >> 10) & 0xfff) << (12 if ins >> 22 & 1 else 0))
                s = self.string(regs[rd])
                if s:
                    refs.append(s)
            elif ins & 0xff80001f == 0x7100001f and rn == 0:                     # cmp w0, #imm
                cmps.append((ins >> 10) & 0xfff)
            elif ins & 0x7c000000 == 0x14000000:                                 # b / bl
                imm = ins & 0x3ffffff
                imm -= (imm & 1 << 25) << 1
                calls.append(pc + 4 * imm)
                if not ins >> 31:
                    break
            elif ins == 0xd65f03c0 and i > 4:                                    # ret (past the early class returns)
                break
        return refs, cmps, calls


KEYS = ('HasBaseband', 'cellular-data', 'contains-cellular-radio', 'telephony', 'sms', 'CellularTelephonyCapability', 'mms',
        'data-plan', 'BasebandClass', 'EUICCChipID', 'BasebandChipset',
        'ModelNumber', 'RegionInfo', 'RegulatoryModelNumber',   # 07/11 About (scripts/make-guamp-model-number-adt.py)
        'personal-hotspot')   # 07/11 Personal Hotspot (scripts/make-guamp-hotspot-adt.py)


def check_mg(path):
    mg = Dylib(path)
    s = {k: mg.scan(mg.answer_fn(k)) for k in KEYS}
    assert s['HasBaseband'][0] == ['baseband'], s['HasBaseband']
    for k in ('cellular-data', 'contains-cellular-radio'):
        assert s[k][0] == [obf('HasBaseband')], (k, s[k])
    for k in ('sms', 'CellularTelephonyCapability'):
        assert s[k][0] == [obf('telephony')], (k, s[k])
    copy = {s[k][2][-1] for k in ('cellular-data', 'contains-cellular-radio', 'sms', 'CellularTelephonyCapability')}
    assert len(copy) == 1, f'the derived keys must share one answer copy: {copy}'
    assert s['telephony'][1][:2] == [1, 6] and s['telephony'][0] == [obf('HasBaseband')], s['telephony']
    assert s['mms'][1] == [1] and not s['mms'][0], s['mms']
    assert s['data-plan'][1] == [3] and s['data-plan'][0] == ['baseband'], s['data-plan']
    assert s['BasebandClass'][0] == ['IODeviceTree:/baseband', obf('class')], s['BasebandClass']
    assert s['EUICCChipID'][0] == ['IODeviceTree:/baseband', obf('euicc-chip-id')], s['EUICCChipID']
    assert s['BasebandChipset'][0] == ['IODeviceTree:/product', obf('baseband-chipset')], s['BasebandChipset']
    assert s['ModelNumber'][0] == ['IODeviceTree:/', 'model-number'], s['ModelNumber']
    assert s['RegionInfo'][0] == ['IOPlatformExpertDevice', 'region-info'], s['RegionInfo']
    assert s['RegulatoryModelNumber'][0] == ['IODeviceTree:/', obf('regulatory-model-number')], s['RegulatoryModelNumber']
    assert s['personal-hotspot'][0] == ['IODeviceTree:/product', obf('personal-hotspot')], s['personal-hotspot']
    return s


def load_adt(path):
    spec = importlib.util.spec_from_file_location('guamp_audio_adt', ROOT / 'scripts/make-guamp-audio-adt.py')
    A = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(A)
    return {k: v for k, (_, v) in A.flatten(A.parse(Path(path).read_bytes())).items()}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--adt', type=Path)
    ap.add_argument('--guamp', action='store_true', help='with --adt: assert the answers of the real phone')
    ap.add_argument('--mg', type=Path)
    args = ap.parse_args()
    built = self_test()
    print('check-guamp-mg-cellular: self test PASS (MobileGestalt model, guamp-mg-adt-sync stamp'
          + (', stamp loop built and run on the host)' if built else '; no C compiler, stamp loop not built)'))
    mg = args.mg or (DEFAULT_MG if DEFAULT_MG.exists() else None)
    if mg:
        check_mg(mg)
        print(f'check-guamp-mg-cellular: {mg}: {len(KEYS)} answer functions match the model PASS')
    else:
        print('check-guamp-mg-cellular: libMobileGestalt.dylib not found, binary check skipped')
    if args.adt:
        a = answers(load_adt(args.adt))
        print('\n'.join(f'  {k} = {v}' for k, v in a.items()))
        if args.guamp:
            check_guamp(a, str(args.adt))
            print(f'check-guamp-mg-cellular: {args.adt}: guamp phone answers PASS')


if __name__ == '__main__':
    main()
