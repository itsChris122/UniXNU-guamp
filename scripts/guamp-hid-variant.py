"""Apply the owned HID collection variant to the exact pinned build-tree source.

Step 1 (unchanged): opt-in Consumer-only primary collection (guamp-hid-buttons-only=1).
Step 2: ring/silent switch. Report 2 keeps its two bytes and both collections keep their usages: the four constant
padding bits after Menu/Power/VolUp/VolDown become one Telephony (0x0B) usage 0x2E input bit (bit 4) and three padding
bits. backboardd (BKButtonHIDEventProcessor, page 0x0B usage 0x2E -> _processRingerSwitchEvent:down:) and SpringBoard
(__handleHIDEvent: 0xB002E -> _handleRingerSwitchEvent:) read it as a level: 1 ring, 0 silent. IOHIDEventDriver
stores Telephony elements as keyboard elements and dispatches them on value change. The button sink keeps bit 4.
"""
from pathlib import Path
import hashlib
root=Path(__file__).resolve().parents[1]
path=root/'drivers/NativeHidInput/NativeHidInput.cpp'
PRISTINE='0445e5f451ffcc827bdd603e822180444d998427fa3a6d63559b1a7b8f850998'
CONSUMER='dc43b25ae0dc0191ab7a23674c3604262149769d7fa55c1372abe5135eea2929'
FINAL=None
FINAL_RECORDED='f5fa8fa8d63f498d648435957e0e2f5fa3dabba09d59cff930a34fc4e9bc58d0'
raw=path.read_bytes();digest=hashlib.sha256(raw).hexdigest()
text=raw.decode()
if digest==PRISTINE:
    assert text.count('// Report 3: ten-finger Touch Screen collection')==1 and text.count('            size=sizeof(descriptor);memcpy(combined,descriptor,size);')==1
    text=text.replace('// Report 3: ten-finger Touch Screen collection','static const unsigned char guampConsumerDescriptor[] = {0x05,0x0c,0x09,0x01,0xa1,0x01,0x85,0x02,0x09,0x40,0x09,0x30,0x09,0xe9,0x09,0xea,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x04,0x81,0x02,0x75,0x04,0x95,0x01,0x81,0x03,0xc0};\n'+'// Report 3: ten-finger Touch Screen collection').replace('            size=sizeof(descriptor);memcpy(combined,descriptor,size);','            unsigned buttonsOnly=0;\n            PE_parse_boot_argn("guamp-hid-buttons-only", &buttonsOnly, sizeof(buttonsOnly));\n            const unsigned char *primary=buttonsOnly==1?guampConsumerDescriptor:descriptor;\n            size=buttonsOnly==1?sizeof(guampConsumerDescriptor):sizeof(descriptor);\n            memcpy(combined,primary,size);\n            IOLog("GuampHid: keyboard collection %s; consumer report 2 preserved; touch enabled %u\\n",\n                  buttonsOnly==1?"disabled":"enabled", touchEnabled?1u:0u);')
    assert hashlib.sha256(text.encode()).hexdigest()==CONSUMER
    digest=CONSUMER
    print('GuampHid: Consumer-only opt-in prepared; report 2 and digitizer retained')
if digest==CONSUMER:
    ringer='0x05,0x0b,0x09,0x2e,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x01,0x81,0x02,0x75,0x03,0x95,0x01,0x81,0x03,'
    old_main='    0x75,0x04,0x95,0x01,0x81,0x03,0xc0\n};'
    new_main=('    // bit 4: Telephony (0x0B) usage 0x2E, the ring/silent switch (level: 1 ring, 0 silent); bits 5..7 constant\n'
              '    '+ringer[:-1].replace(',0x75,0x03',',\n    0x75,0x03')+',0xc0\n};')
    old_consumer='0x75,0x01,0x95,0x04,0x81,0x02,0x75,0x04,0x95,0x01,0x81,0x03,0xc0};'
    new_consumer='0x75,0x01,0x95,0x04,0x81,0x02,'+ringer+'0xc0};'
    old_mask='self->buttons=static_cast<uint8_t>(uintptr_t(mask)&0xe);'
    new_mask='self->buttons=static_cast<uint8_t>(uintptr_t(mask)&0x1e);   // bit 4: ring/silent switch level'
    for old in (old_main,old_consumer,old_mask):
        assert text.count(old)==1,'HID ringer variant: anchor not unique: '+old
    text=text.replace(old_main,new_main).replace(old_consumer,new_consumer).replace(old_mask,new_mask)
    FINAL=hashlib.sha256(text.encode()).hexdigest()
    assert FINAL==FINAL_RECORDED,'HID ringer variant result differs from the recorded source'
    path.write_text(text)
    print('GuampHid: ring/silent switch (Telephony 0x0B/0x2E) added as report 2 bit 4; collections unchanged')
elif digest==FINAL_RECORDED:
    print('GuampHid: pinned Consumer-only opt-in and ring/silent switch already prepared')
else:
    raise SystemExit('Pinned NativeHidInput source differs; do not guess an ABI or source layout')
