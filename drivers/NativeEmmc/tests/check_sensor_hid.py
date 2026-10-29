"""Check of the NativeHidInput sensor descriptors (scripts/guamp-hid-variant.py step 4) with a small HID item parser.

For each device: one application collection whose (page, usage) pair is in an IOHIDEventDriver personality of the
pinned kernelcache (HID SensorEvent Driver 0x20/0x73 and 0x20/0x7A, HID Built-In Orientation Driver 0x20/0x8A, HID
Pointing/Consumer/System Control Driver page 0x0C), the input usages IOHIDEventDriver parses (parseAccelElement,
parseGyroElement, parseDeviceOrientationElement, parseProximityElement) and an input report size equal to what
sensorAction sends (13, 2, 2, 2 bytes with the report ID).
"""
import re, sys
from pathlib import Path

root = Path(__file__).resolve().parents[3]
src = (root / 'scripts/guamp-hid-variant.py').read_text()
block = src[src.index("SENSOR_DESCRIPTORS=r'''"):src.index("SENSOR_METHODS=")]

def array(name):
    m = re.search(r'static const unsigned char ' + name + r'\[\]=\{(.*?)\};', block, re.S)
    return bytes(int(x, 16) for x in re.findall(r'0x[0-9a-fA-F]{2}', m.group(1)))

def parse(d):
    """Returns (collections, inputs, features, input bits per report id)."""
    i, g, local, stack = 0, {'page': 0, 'size': 0, 'count': 0, 'rid': 0, 'lmin': 0, 'lmax': 0, 'exp': 0}, [], []
    cols, inputs, features, bits = [], [], [], {}
    umin = umax = None
    while i < len(d):
        b = d[i]; n = (0, 1, 2, 4)[b & 3]; typ = (b >> 2) & 3; tag = b >> 4
        raw = d[i + 1:i + 1 + n]; val = int.from_bytes(raw, 'little') if n else 0
        sval = int.from_bytes(raw, 'little', signed=True) if n else 0
        i += 1 + n
        if typ == 1:   # global
            if tag == 0: g['page'] = val
            elif tag == 1: g['lmin'] = sval
            elif tag == 2: g['lmax'] = sval
            elif tag == 5: g['exp'] = val
            elif tag == 7: g['size'] = val
            elif tag == 8: g['rid'] = val
            elif tag == 9: g['count'] = val
        elif typ == 2:   # local
            full = val if n == 4 else (g['page'] << 16) | val
            if tag == 0: local.append(full)
            elif tag == 1: umin = full
            elif tag == 2: umax = full
        else:            # main
            if tag == 0xA:
                cols.append((local[0] >> 16, local[0] & 0xffff, val)); stack.append(cols[-1])
            elif tag == 0xC:
                stack.pop()
            elif tag in (0x8, 0xB):
                usages = list(local) or (list(range(umin, umax + 1)) if umin is not None else [])
                entry = (g['rid'], usages, dict(g))
                (inputs if tag == 0x8 else features).append(entry)
                if tag == 0x8: bits[g['rid']] = bits.get(g['rid'], 0) + g['size'] * g['count']
            local, umin, umax = [], None, None
    assert not stack, 'unbalanced collections'
    return cols, inputs, features, bits

fails = 0
def check(c, what):
    global fails
    if not c:
        print('FAIL', what); fails += 1

expect = {
    'accelDescriptor': ((0x20, 0x73), 4, [0x200453, 0x200454, 0x200455], 13),
    'gyroDescriptor': ((0x20, 0x7A), 5, [0x200457, 0x200458, 0x200459], 13),
    'orientDescriptor': ((0x20, 0x8A), 6, list(range(0xFF0C0064, 0xFF0C006C)), 2),
    'proxDescriptor': ((0x0C, 0x01), 7, [0x0C0109], 2),
}
for name, (pair, rid, usages, size) in expect.items():
    cols, inputs, features, bits = parse(array(name))
    check(len([c for c in cols if c[2] == 1]) == 1 and (cols[0][0], cols[0][1]) == pair, f'{name}: application collection {pair}')
    got = [u for (r, us, _) in inputs for u in us]
    check(got == usages, f'{name}: input usages {[hex(u) for u in got]}')
    check(all(r == rid for (r, _, _) in inputs), f'{name}: report id {rid}')
    check(1 + bits.get(rid, 0) // 8 == size and bits.get(rid, 0) % 8 == 0, f'{name}: input report {size} bytes ({bits})')
    if rid in (4, 5):
        g = inputs[0][2]
        check(g['exp'] == 0xD and g['lmin'] == -32768 and g['lmax'] == 32767 and g['size'] == 32, f'{name}: axis format')
        check([u for (_, us, _) in features for u in us] == [0x20030E], f'{name}: Report Interval feature')
print('check_sensor_hid:', 'FAIL' if fails else 'PASS')
sys.exit(1 if fails else 0)
