"""Apply the owned HID collection variant to the exact pinned build-tree source."""
from pathlib import Path
import hashlib
root=Path(__file__).resolve().parents[1]
path=root/'drivers/NativeHidInput/NativeHidInput.cpp'
raw=path.read_bytes();digest=hashlib.sha256(raw).hexdigest()
if digest=='dc43b25ae0dc0191ab7a23674c3604262149769d7fa55c1372abe5135eea2929':
    print('GuampHid: pinned Consumer-only opt-in already prepared')
else:
    assert digest=='0445e5f451ffcc827bdd603e822180444d998427fa3a6d63559b1a7b8f850998','Pinned NativeHidInput source differs; do not guess an ABI or source layout'
    text=raw.decode()
    assert text.count('// Report 3: ten-finger Touch Screen collection')==1 and text.count('            size=sizeof(descriptor);memcpy(combined,descriptor,size);')==1
    text=text.replace('// Report 3: ten-finger Touch Screen collection','static const unsigned char guampConsumerDescriptor[] = {0x05,0x0c,0x09,0x01,0xa1,0x01,0x85,0x02,0x09,0x40,0x09,0x30,0x09,0xe9,0x09,0xea,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x04,0x81,0x02,0x75,0x04,0x95,0x01,0x81,0x03,0xc0};\n'+'// Report 3: ten-finger Touch Screen collection').replace('            size=sizeof(descriptor);memcpy(combined,descriptor,size);','            unsigned buttonsOnly=0;\n            PE_parse_boot_argn("guamp-hid-buttons-only", &buttonsOnly, sizeof(buttonsOnly));\n            const unsigned char *primary=buttonsOnly==1?guampConsumerDescriptor:descriptor;\n            size=buttonsOnly==1?sizeof(guampConsumerDescriptor):sizeof(descriptor);\n            memcpy(combined,primary,size);\n            IOLog("GuampHid: keyboard collection %s; consumer report 2 preserved; touch enabled %u\\n",\n                  buttonsOnly==1?"disabled":"enabled", touchEnabled?1u:0u);')
    assert hashlib.sha256(text.encode()).hexdigest()=='dc43b25ae0dc0191ab7a23674c3604262149769d7fa55c1372abe5135eea2929'
    path.write_text(text)
    print('GuampHid: Consumer-only opt-in prepared; report 2 and digitizer retained')
