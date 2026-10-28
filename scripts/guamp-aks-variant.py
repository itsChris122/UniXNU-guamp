"""Apply the owned key-wrap variant to the exact pinned NativeKeyStoreEvents source (guamp-aks-wrap=1).

securityd links AppleKeyStore_libs statically and calls IOConnectCallMethod itself: ks_crypt (keychain item and metadata class
keys) uses selector 10 (aks_wrap_key: scalars {keybag handle, key class}, structure in = key, scalar out = key class, structure
out = wrapped blob) and selector 11 (aks_unwrap_key: same scalars, structure in = blob, structure out = key), as the unchanged
21H461 AppleKeyStore.framework aks_wrap_key/aks_unwrap_key and securityd 0x100019bf0/0x100014c70 show. The pinned stand-in fails
both with kIOReturnUnsupported-like 0xe00002c7, so every keychain write fails; bluetoothd cannot store its LE root keys
(_BTKCSetData, kSecUseSystemKeychain) and stops with "LeSecurityManager hit a keychain problem." (stackshot 288, 0xbd15dead).

With guamp-aks-wrap=1 the two selectors answer: the blob is an 8-byte tag followed by the key (the size of an RFC 3394 wrap). It
is NOT encryption and protects nothing, like the rest of this stand-in (the Data volume has no data protection either). Default on
since 28/10 (package/KeyStoreProbe.cpp); with guamp-aks-wrap=0 the selectors fail exactly as before.
"""
from pathlib import Path
import hashlib
root=Path(__file__).resolve().parents[1]
path=root/'drivers/NativeKeyStoreEvents/NativeKeyStoreEvents.cpp'
PRISTINE='273b030afee897e48c30bc917c4df8a9c4d94a189d073ef80c0e0a15caef82ee'
FINAL_RECORDED='145d1297b9385ac03b716d12849d18353b781502e347d7de18233ad567aa8437'
raw=path.read_bytes();digest=hashlib.sha256(raw).hexdigest()
if digest==FINAL_RECORDED:
    print('GuampAks: pinned key-wrap variant already prepared')
    raise SystemExit(0)
if digest!=PRISTINE:
    raise SystemExit('Pinned NativeKeyStoreEvents source differs; do not guess an ABI or source layout')
text=raw.decode()
edits=[
 ('extern "C" void native_aks_opened(void);\n',
  'extern "C" void native_aks_opened(void);\n'
  'extern "C" bool native_aks_wrap_enabled(void);\n'),
 ('constexpr uint32_t kSelLockState = 7, kSelGetSystem = 14, kSelDeviceState = 17, kSelExtendedDeviceState = 35;\n',
  'constexpr uint32_t kSelLockState = 7, kSelGetSystem = 14, kSelDeviceState = 17, kSelExtendedDeviceState = 35;\n'
  'constexpr uint32_t kSelKeyWrap = 10, kSelKeyUnwrap = 11;   // guamp-aks-wrap=1 (scripts/guamp-aks-variant.py)\n'),
 ('class AppleKeyStoreUserClient final : public IOUserClient {\n',
  '// guamp-aks-wrap=1: aks_wrap_key (10) and aks_unwrap_key (11) as securityd\'s ks_crypt calls them. The blob is an 8-byte tag\n'
  '// followed by the key, the size of an RFC 3394 wrap; it is not encryption and protects nothing (scripts/guamp-aks-variant.py).\n'
  'namespace {\n'
  'constexpr uint8_t kWrapTag[8] = {\'G\', \'U\', \'A\', \'M\', \'P\', \'A\', \'K\', \'S\'};\n'
  'constexpr size_t kWrapMaxKey = 64;\n'
  'uint32_t gKeyOpLogs;\n'
  'IOReturn keyOp(uint32_t selector, IOExternalMethodArguments *args) {\n'
  '    if (args->scalarInputCount != 2 || !args->scalarInput) return kIOReturnBadArgument;\n'
  '    uint8_t in[kWrapMaxKey + sizeof kWrapTag], out[kWrapMaxKey + sizeof kWrapTag];\n'
  '    const size_t inLen = args->structureInputDescriptor ? args->structureInputDescriptor->getLength() : args->structureInputSize;\n'
  '    if (!inLen || inLen > sizeof in) return kIOReturnBadArgument;\n'
  '    if (args->structureInputDescriptor) {\n'
  '        if (args->structureInputDescriptor->readBytes(0, in, inLen) != inLen) return kIOReturnBadArgument;\n'
  '    } else if (args->structureInput) {\n'
  '        memcpy(in, args->structureInput, inLen);\n'
  '    } else {\n'
  '        return kIOReturnBadArgument;\n'
  '    }\n'
  '    size_t outLen = 0;\n'
  '    IOReturn rc = kIOReturnSuccess;\n'
  '    if (selector == kSelKeyWrap) {\n'
  '        if (inLen > kWrapMaxKey || args->scalarOutputCount < 1 || !args->scalarOutput) rc = kIOReturnBadArgument;\n'
  '        else { memcpy(out, kWrapTag, sizeof kWrapTag); memcpy(out + sizeof kWrapTag, in, inLen); outLen = inLen + sizeof kWrapTag; }\n'
  '    } else if (inLen <= sizeof kWrapTag || memcmp(in, kWrapTag, sizeof kWrapTag) != 0) {\n'
  '        rc = kIOReturnBadArgument;                          // not a blob of this stand-in: nothing to unwrap\n'
  '    } else {\n'
  '        outLen = inLen - sizeof kWrapTag;\n'
  '        memcpy(out, in + sizeof kWrapTag, outLen);\n'
  '    }\n'
  '    if (rc == kIOReturnSuccess) {\n'
  '        if (args->structureOutputDescriptor) {\n'
  '            if (outLen > args->structureOutputDescriptor->getLength()) rc = kIOReturnNoSpace;\n'
  '            else { args->structureOutputDescriptor->writeBytes(0, out, outLen); args->structureOutputDescriptorSize = (uint32_t)outLen; }\n'
  '        } else if (args->structureOutput && outLen <= args->structureOutputSize) {\n'
  '            memcpy(args->structureOutput, out, outLen);\n'
  '            args->structureOutputSize = (uint32_t)outLen;\n'
  '        } else {\n'
  '            rc = kIOReturnNoSpace;\n'
  '        }\n'
  '    }\n'
  '    if (rc == kIOReturnSuccess && selector == kSelKeyWrap) {\n'
  '        args->scalarOutput[0] = args->scalarInput[1];      // the class asked for is the class used\n'
  '        args->scalarOutputCount = 1;\n'
  '    }\n'
  '    if (gKeyOpLogs < 24) {\n'
  '        gKeyOpLogs = gKeyOpLogs + 1;\n'
  '        IOLog("GuampAks: %s bag %lld class %llu in %zu out %zu rc 0x%x\\n", selector == kSelKeyWrap ? "wrap" : "unwrap",\n'
  '              (long long)args->scalarInput[0], (unsigned long long)args->scalarInput[1], inLen, outLen, rc);\n'
  '    }\n'
  '    memset(in, 0, sizeof in);\n'
  '    memset(out, 0, sizeof out);\n'
  '    return rc;\n'
  '}\n'
  '}  // namespace\n'
  '\n'
  'class AppleKeyStoreUserClient final : public IOUserClient {\n'),
 ('        default: break;\n',
  '        case kSelKeyWrap:\n'
  '        case kSelKeyUnwrap:\n'
  '            if (native_aks_wrap_enabled()) rc = keyOp(selector, args);\n'
  '            break;\n'
  '        default: break;\n'),
]
for old,new in edits:
    assert text.count(old)==1,'AKS key-wrap variant: anchor not unique: '+old
    text=text.replace(old,new)
final=hashlib.sha256(text.encode()).hexdigest()
assert final==FINAL_RECORDED,'AKS key-wrap variant result differs from the recorded source: '+final
path.write_text(text)
print('GuampAks: aks_wrap_key/aks_unwrap_key (selectors 10/11) prepared behind guamp-aks-wrap=1')
