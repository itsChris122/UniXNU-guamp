"""Host model of the two-finger path (scripts/guamp-hid-variant.py step 6): NativeHidInput::touchSend packing of report 3
and IOHIDEventDriver::handleDigitizerCollectionReport / createDigitizerTransducerEventForReport of the pinned IOHIDFamily
(touch state per Contact Identifier, a finger field handled when one of its elements changed or its tip is down).
Scenario of Mappe (fin51): fingers on panel slots 0 and 1, slot 1 lifts first, slot 0 keeps moving. With unused fields at
identifier 0 the driver lifted finger 0 at 0,0 and began it again; with 0xFF finger 0 gets no event but its moves."""
import sys

FINGERS = 10


def touch_send(state, frame, pad_id):
    """frame: list of (id, x, y) down; returns the 10 fields (tip, valid, id, x, y)."""
    fields, slot, nxt = [], 0, []
    for fid, x, y in frame:
        fields.append((1, 1, fid, x, y)); nxt.append((fid, x, y)); slot += 1
    for fid, x, y in state['prev']:
        if any(f == fid for f, _, _ in frame):
            continue
        fields.append((0, 1, fid, x, y)); slot += 1
    state['prev'] = nxt
    while len(fields) < FINGERS:
        fields.append((0, 0, pad_id, 0, 0))
    return fields


def driver(dstate, fields):
    """Events (id, touch, mask) the event driver emits for one report."""
    out = []
    for pos, f in enumerate(fields):
        tip, valid, fid, x, y = f
        changed = dstate['elem'].get(pos) != f
        dstate['elem'][pos] = f
        if not (changed or tip):
            continue
        st = dstate['ts'].setdefault(fid, {'touch': 0, 'range': 0})
        touch = bool(tip); in_range = touch
        mask = (2 if touch != st['touch'] else 0) | (1 if in_range != st['range'] else 0)
        drop = st['touch'] == touch and not touch and not in_range
        st['touch'], st['range'] = touch, in_range
        if not drop:
            out.append((fid, int(touch), mask))
    return out


def run(pad_id):
    s, d, events = {'prev': []}, {'elem': {}, 'ts': {}}, []
    frames = [[(0, 100, 100)], [(0, 100, 100), (1, 300, 300)], [(0, 110, 110), (1, 290, 290)],
              [(0, 120, 120)], [(0, 130, 130)], [(0, 140, 140)], []]
    for fr in frames:
        events.append(driver(d, touch_send(s, fr, pad_id)))
    return events


def finger0_lifts(events):
    return sum(1 for rep in events[:-1] for fid, touch, mask in rep if fid == 0 and not touch)


old, new = run(0), run(0xFF)
ok = True
if finger0_lifts(old) == 0:
    print('FAIL: the model no longer shows the old false lift of finger 0'); ok = False
if finger0_lifts(new) != 0:
    print('FAIL: finger 0 still lifted while down with padding 0xFF:', new); ok = False
if not any(fid == 0 and not touch for fid, touch, _ in new[-1]):
    print('FAIL: finger 0 lift missing at the end:', new[-1]); ok = False
if not any(fid == 1 and not touch for rep in new for fid, touch, _ in rep):
    print('FAIL: finger 1 lift missing:', new); ok = False
print('touch padding:', 'ok' if ok else 'FAILED', '| old false lifts of finger 0:', finger0_lifts(old))
sys.exit(0 if ok else 1)
