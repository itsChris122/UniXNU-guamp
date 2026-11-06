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

guamp-aks-keybag=1 (default 0, 07/11): the user client first asks native_aks_keybag_method() (package/KeyStoreProbe.cpp +
package/GuampKeybagCore.h): a real software keybag in /private/var/keybags/systembag.kb, RFC 3394 wrap per class,
entitlement check (rights taken at open), keybagd --init flow (aks_get_system NotFound -> load/set_system/unload).
Keys wrapped by the GUAMPAKS stand-in still unwrap (guamp-aks-legacy=1); with the keybag off, keys wrapped during a
keybag boot still unwrap from systembag.kb (read only). See analisi/KEYBAG-REALE-20261107.md.
"""
from pathlib import Path
import hashlib
root=Path(__file__).resolve().parents[1]
path=root/'drivers/NativeKeyStoreEvents/NativeKeyStoreEvents.cpp'
PRISTINE='273b030afee897e48c30bc917c4df8a9c4d94a189d073ef80c0e0a15caef82ee'
FINAL_RECORDED='886943cfbd7187a772302e0c09d60d3f2d82c72abd59e8183970ae0b18191692'
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
  'uint32_t gKeyOpLogs, gUnsupportedLogs;\n'
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
 ('        native_aks_call(selector, (int32_t)rc);\n',
  '        // Selectors this stand-in does not provide (no keybag, no secret): 2 create bag, 3 save bag, 15 change secret (set,\n'
  '        // change or remove a passcode), 49 verify password. They fail as before; logged so a passcode attempt is visible.\n'
  '        if (rc == kAksUnsupported && (selector == 2 || selector == 3 || selector == 15 || selector == 49) && gUnsupportedLogs < 32) {\n'
  '            gUnsupportedLogs = gUnsupportedLogs + 1;\n'
  '            IOLog("GuampAks: selector %u not provided rc 0x%x%s\\n", selector, rc,\n'
  '                  selector == 15 ? " (aks_change_secret: passcode not set, keybag unchanged)" : "");\n'
  '        }\n'
  '        native_aks_call(selector, (int32_t)rc);\n'),
 ('extern "C" bool native_aks_wrap_enabled(void);\n',
  'extern "C" bool native_aks_wrap_enabled(void);\n'
  '// guamp-aks-keybag=1 (default 0): a real software keybag (package/GuampKeybagCore.h, package/KeyStoreProbe.cpp) answers\n'
  '// selectors 3/4/5/6/10/11/14/23 before this stand-in; the client rights come from its entitlements at open. With it off,\n'
  '// only an unwrap of a key wrapped during a keybag boot (read-only, from systembag.kb), so a rollback loses nothing.\n'
  'extern "C" uint32_t native_aks_client_rights(task_t task);\n'
  'extern "C" bool native_aks_keybag_method(uint32_t selector, IOExternalMethodArguments *args, uint32_t rights, IOReturn *result);\n'),
 ('    OSDeclareDefaultStructors(AppleKeyStoreUserClient)\npublic:\n',
  '    OSDeclareDefaultStructors(AppleKeyStoreUserClient)\npublic:\n'
  '    uint32_t keybagRights = 0;   // guamp-aks-keybag: 1 = com.apple.keystore.device (keybagd), 2 = access-keychain-keys (securityd)\n'),
 ('        else switch (selector) {\n',
  '        else if (native_aks_keybag_method(selector, args, keybagRights, &rc)) {}\n'
  '        else switch (selector) {\n'),
 ('        if (!client->start(this)) { client->detach(this); client->release(); return kIOReturnError; }\n',
  '        if (!client->start(this)) { client->detach(this); client->release(); return kIOReturnError; }\n'
  '        client->keybagRights = native_aks_client_rights(task);\n'),
 ('extern "C" uint32_t native_aks_lock_state(void);\n',
  'extern "C" uint32_t native_aks_lock_state(void);\nextern "C" uint32_t native_aks_state_flags(void);   // guamp-aks-nopass (scripts/guamp-aks-variant.py)\n'),
 ('constexpr uint64_t kUnlockedSinceBoot = 4;\nconstexpr uint32_t kStateFlags = 4;\n',
  '// guamp-aks-nopass=1 (default): the keybag state flags come from native_aks_state_flags() = 0x6 (bit 0x2\n// "no passcode" | bit 0x4 "unlocked since boot"); =0 restores the old 0x4-only value (scripts/guamp-aks-variant.py).\n'),
 ('            if (rc == kIOReturnSuccess) args->scalarOutput[0] = kUnlockedSinceBoot;\n',
  '            if (rc == kIOReturnSuccess) args->scalarOutput[0] = native_aks_state_flags();\n'),
 ('    const Entry entries[] = {{"ss", kStateFlags}, {"sls", lockState}, {"sb", 0}, {"sfa", 0}, {"sgs", 0},\n',
  '    const Entry entries[] = {{"ss", native_aks_state_flags()}, {"sls", lockState}, {"sb", 0}, {"sfa", 0}, {"sgs", 0},\n'),
]
for old,new in edits:
    assert text.count(old)==1,'AKS key-wrap variant: anchor not unique: '+old
    text=text.replace(old,new)
final=hashlib.sha256(text.encode()).hexdigest()
assert final==FINAL_RECORDED,'AKS key-wrap variant result differs from the recorded source: '+final
path.write_text(text)
print('GuampAks: aks_wrap_key/aks_unwrap_key (selectors 10/11) prepared behind guamp-aks-wrap=1')
