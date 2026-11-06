// SPDX-License-Identifier: BSD-2-Clause-Patent
// Mutually exclusive offline probes: unchanged target keystore loading, or
// ephemeral notification-only service. Neither supplies keys or unlock state.
#include <IOKit/IOService.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <libkern/c++/OSKext.h>
#include <libkern/c++/OSData.h>
#include <libkern/c++/OSArray.h>
#include <libkern/c++/OSString.h>
#include <libkern/c++/OSSymbol.h>
#include <pexpert/pexpert.h>
#include <mach/kmod.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <sys/fcntl.h>
#include <sys/mount.h>
#include <sys/proc.h>
#include <sys/random.h>
#include <sys/vnode.h>
#include "GuampKeybagCore.h"
extern kmod_info_t g_kernel_kmod_info;
extern "C" void native_keyevents_static_init(void) __asm__("__GLOBAL__sub_I_NativeKeyStoreEvents.cpp");
extern "C" IOService *native_keyevents_allocate(void);
extern "C" {
volatile uint32_t native_keyevents_stage, native_keyevents_refused_opens;
volatile int native_keyevents_status = -1;
volatile uint64_t native_keyevents_service;
void native_keyevents_open_refused(void) {
    native_keyevents_refused_opens = native_keyevents_refused_opens + 1;
}
// sheng-aks=1: the owned AppleKeyStore answers the state queries (no passcode, nothing locked); counters feed the diagnostic
// page's extension rows (slots 0-1): opens/calls, then selector:count per selector seen ('e' = the last answer was an error).
extern char boot_progress_ext[8][48];
volatile uint32_t native_aks_opens, native_aks_calls;
static volatile uint32_t aks_count[96];
static volatile int32_t aks_rc[96];
bool native_aks_state_enabled(void) {
    static int cached = -1;
    if (cached < 0) {
        unsigned value = 1;   // default on (verified 28/10): sheng-aks=0 disables
        PE_parse_boot_argn("sheng-aks", &value, sizeof(value));
        cached = value == 1;
    }
    return cached == 1;
}
// guamp-aks-wrap (default 1 since 28/10, =0 off): the owned AppleKeyStore also answers aks_wrap_key/aks_unwrap_key (selectors 10/11) so securityd can store
// keychain items, for instance the LE root keys of bluetoothd (scripts/guamp-aks-variant.py). The blob is not encrypted.
bool native_aks_wrap_enabled(void) {
    static int cached = -1;
    if (cached < 0) {
        unsigned value = 1;   // default on (verified 28/10, bluetoothd root keys); guamp-aks-wrap=0 disables
        PE_parse_boot_argn("guamp-aks-wrap", &value, sizeof(value));
        cached = value == 1;
    }
    return cached == 1;
}
// sheng-aks-sls=<n> overrides the reported lock state (default 3 since 28/10 = MobileKeyBag's "no passcode"; 0 = unlocked).
uint32_t native_aks_lock_state(void) {
    static int cached = -1;
    if (cached < 0) {
        unsigned value = 3;
        PE_parse_boot_argn("sheng-aks-sls", &value, sizeof(value));
        if (value > 7) value = 0;
        cached = (int)value;
    }
    return (uint32_t)cached;
}
// guamp-aks-nopass=<n> (default 1, =0 off): the keybag STATE FLAGS the owned AppleKeyStore reports for
// aks_get_lock_state (selector 7) and the device-state DER field "ss". A passcode-less iPhone reports bit
// 0x2 ("no passcode") together with bit 0x4 ("unlocked since boot") = 0x6. Reporting only 0x4 makes
// LocalAuthentication/coreauthd believe a passcode exists, so Photos Hidden/Recently-Deleted ask for a code
// (whose entry view cannot show a keyboard without a SEP/ACM context). With 0x6, DeviceOwnerAuthentication
// fails with passcodeNotSet and Photos opens the album directly. guamp-aks-nopass=0 restores the old 0x4.
uint32_t native_aks_state_flags(void) {
    static int cached = -1;
    if (cached < 0) {
        unsigned value = 1;
        PE_parse_boot_argn("guamp-aks-nopass", &value, sizeof(value));
        cached = value == 1 ? 6 : 4;   // 0x2 no passcode | 0x4 unlocked since boot
    }
    return (uint32_t)cached;
}
static void aks_publish(void) {
    // Slot 0: counters; slot 1: selector:count tokens ('e' = the last answer was an error). Slots 2-3 belong to the Data store.
    char row[48];
    unsigned used = 0;
    row[0] = 0;
    for (unsigned sel = 0; sel < 96; ++sel) {
        if (!aks_count[sel]) continue;
        char token[16];
        int n = snprintf(token, sizeof token, "%u:%u%s ", sel, aks_count[sel], aks_rc[sel] ? "e" : "");
        if (n <= 0 || used + (unsigned)n > 46) break;
        memcpy(row + used, token, (size_t)n + 1);
        used += (unsigned)n;
    }
    char head[48];
    snprintf(head, sizeof head, "AKS opens=%u calls=%u", native_aks_opens, native_aks_calls);
    for (unsigned i = 0; i < 47; ++i) boot_progress_ext[0][i] = head[i] ? head[i] : ' ';
    boot_progress_ext[0][47] = 0;
    unsigned i = 0;
    for (; i < 47 && row[i]; ++i) boot_progress_ext[1][i] = row[i];
    for (; i < 47; ++i) boot_progress_ext[1][i] = ' ';
    boot_progress_ext[1][47] = 0;
}
void native_aks_opened(void) {
    native_aks_opens = native_aks_opens + 1;
    aks_publish();
}
void native_aks_call(uint32_t selector, int32_t result) {
    native_aks_calls = native_aks_calls + 1;
    if (selector < 96) { aks_count[selector] = aks_count[selector] + 1; aks_rc[selector] = result; }
    aks_publish();
}
void native_aks_keybag_boot(void);   // guamp-aks-keybag (end of this file)
static void native_keyevents_probe(void) {
    unsigned enabled = 1, original = 0;   // offline-key-events: default on (verified 28/10), =0 off
    PE_parse_boot_argn("offline-key-events", &enabled, sizeof(enabled));
    if (enabled != 1 || native_keyevents_stage) return;
    native_keyevents_stage = 1;
    PE_parse_boot_argn("offline-keystore", &original, sizeof(original));
    native_keyevents_status = kIOReturnNotPermitted;
    // An explicit plane requires a path without a second plane prefix.
    IORegistryEntry *chosen = IORegistryEntry::fromPath("/chosen", gIODTPlane);
    if (!chosen) { native_keyevents_status = kIOReturnNoDevice; return; }
    OSData *data = chosen ? OSDynamicCast(OSData, chosen->getProperty("ephemeral-storage")) : nullptr;
    uint32_t ephemeral = 0;
    if (data && data->getLength() == sizeof(ephemeral))
        memcpy(&ephemeral, data->getBytesNoCopy(), sizeof(ephemeral));
    OSSafeReleaseNULL(chosen);
    // A persistent Data store (no ephemeral-storage) still has no SEP: sheng-aks keeps the owned service for it.
    if ((ephemeral != 1 && !native_aks_state_enabled()) || original) return;
    const OSSymbol *name = OSSymbol::withCString("AppleKeyStore");
    bool absent = name && !OSMetaClass::getMetaClassWithName(name);
    OSSafeReleaseNULL(name);
    if (!absent) return;
    IOService *provider = IOService::getResourceService();
    if (!provider) return;
    void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!handle) return;
    native_keyevents_static_init();
    native_keyevents_status = OSMetaClass::postModLoad(handle);
    if (native_keyevents_status) return;
    native_keyevents_stage = 2;
    IOService *service = native_keyevents_allocate();
    bool initialized = service && service->init();
    bool attached = initialized && service->attach(provider);
    bool started = attached && service->start(provider);
    if (!started) {
        if (attached) service->detach(provider);
        OSSafeReleaseNULL(service);
        native_keyevents_status = kIOReturnError;
        return;
    }
    native_keyevents_service = (uintptr_t)service;
    native_aks_keybag_boot();
    native_keyevents_status = kIOReturnSuccess;
    native_keyevents_stage = 3;
}
volatile uint32_t native_keystore_stage;
volatile int native_keystore_load_status = -1;
volatile uint64_t native_keystore_service;
volatile uint32_t native_keystore_class_size;
__attribute__((noinline)) void native_keystore_checkpoint(void) {
    __asm__ volatile("" ::: "memory");
}
void native_keystore_probe(void) {
    native_keyevents_probe();
    unsigned enabled = 0;
    if (!PE_parse_boot_argn("offline-keystore", &enabled, sizeof(enabled)) || enabled != 1 ||
        native_keystore_stage) return;
    native_keystore_stage = 1;
    native_keystore_load_status = OSKext::loadKextWithIdentifier(
        "com.apple.driver.AppleSEPKeyStore", true, false,
        kOSKextExcludeNone, kOSKextExcludeNone, nullptr);
    native_keystore_stage = 2;
    if (!native_keystore_load_status) {
        OSDictionary *matching = IOService::serviceMatching("AppleKeyStore");
        IOService *service = matching ? IOService::waitForMatchingService(matching, 5000000000ULL) : nullptr;
        native_keystore_service = (uintptr_t)service;
        if (service) native_keystore_class_size = service->getMetaClass()->getClassSize();
        OSSafeReleaseNULL(service);
        OSSafeReleaseNULL(matching);
    }
    native_keystore_stage = 3;
    native_keystore_checkpoint();
}
}

// ---- guamp-aks-keybag=1 (default 0): a real software keybag behind the owned AppleKeyStore -------------------------------
// The logic is package/GuampKeybagCore.h (host test package/tests/run-keybag-tests.sh); here: boot-args, entitlements,
// systembag.kb through the VFS, randomness, logs, and the IOExternalMethodArguments of selectors 3/4/5/6/10/11/14/23
// (21H461 libaks: aks_save_bag 3, aks_unload_bag 4, aks_set_system 5, aks_load_bag 6, aks_wrap_key 10, aks_unwrap_key 11,
// aks_get_system 14, aks_get_bag_uuid 23). NativeKeyStoreEvents asks native_aks_keybag_method() first (scripts/guamp-aks-variant.py).
// Sub-options: guamp-aks-ent=1 (default) entitlement check, =0 off, =2 entitlement or process name keybagd/securityd;
// guamp-aks-legacy=1 (default) keys wrapped by the old stand-in (GUAMPAKS tag) still unwrap, =0 refused.
namespace {
const char kSystemBagPath[] = "/private/var/keybags/systembag.kb";
IOLock *gKbLock;
gkb_state gKb;
uint32_t gKbLogs, gKbRefusals, gKbLegacy;

unsigned kbArg(const char *name, unsigned fallback) {
    unsigned value = fallback;
    PE_parse_boot_argn(name, &value, sizeof(value));
    return value;
}
int kbExists(void *) {
    vnode_t vp = nullptr;
    if (vnode_lookup(kSystemBagPath, 0, &vp, vfs_context_kernel()) != 0) return -1;
    vnode_put(vp);
    return 0;
}
int kbRead(void *, uint8_t *buf, size_t cap, size_t *len) {
    vfs_context_t ctx = vfs_context_kernel();
    vnode_t vp = nullptr;
    if (vnode_open(kSystemBagPath, FREAD | O_NOFOLLOW, 0, 0, &vp, ctx) != 0) return -1;
    int resid = 0;
    int error = vn_rdwr(UIO_READ, vp, (caddr_t)buf, (int)cap, 0, UIO_SYSSPACE, IO_NOAUTH, vfs_context_ucred(ctx), &resid,
                        vfs_context_proc(ctx));
    vnode_close(vp, FREAD, ctx);
    if (error || resid < 0 || (size_t)resid > cap) return -1;
    *len = cap - (size_t)resid;
    return 0;
}
int kbWrite(void *, const uint8_t *buf, size_t len) {
    // Create only (O_EXCL): an existing systembag.kb is never replaced, whatever is in it.
    vfs_context_t ctx = vfs_context_kernel();
    vnode_t vp = nullptr;
    int error = vnode_open(kSystemBagPath, O_CREAT | O_EXCL | FWRITE | O_NOFOLLOW, 0600, 0, &vp, ctx);
    if (error) return error;
    int resid = 0;
    error = vn_rdwr(UIO_WRITE, vp, (caddr_t)buf, (int)len, 0, UIO_SYSSPACE, IO_NOAUTH | IO_SYNC, vfs_context_ucred(ctx), &resid,
                    vfs_context_proc(ctx));
    if (!error && resid) error = EIO;
    if (!error) error = VNOP_FSYNC(vp, MNT_WAIT, ctx);
    vnode_close(vp, FWRITE, ctx);
    return error;
}
void kbRandom(void *, void *buf, size_t len) { read_random(buf, (u_int)len); }
void kbLog(void *, const char *event, long value) {
    if (gKbLogs >= 48) return;
    gKbLogs = gKbLogs + 1;
    IOLog("GuampKeybag: %s (%ld)\n", event, value);
}
const gkb_io gKbIo = {nullptr, kbExists, kbRead, kbWrite, kbRandom, kbLog};

IOReturn kbReturn(int rc) {
    switch (rc) {
    case GKB_OK: return kIOReturnSuccess;
    case GKB_NOT_FOUND: return kIOReturnNotFound;            // 0xe00002f0, what keybagd --init expects without a bag
    case GKB_BAD_ARGUMENT: return kIOReturnBadArgument;
    case GKB_NOT_PRIVILEGED: return kIOReturnNotPrivileged;
    case GKB_NOT_PERMITTED: return kIOReturnNotPermitted;
    case GKB_NO_RESOURCES: return kIOReturnNoResources;
    default: return kIOReturnNoSpace;
    }
}
bool kbIn(IOExternalMethodArguments *args, uint8_t *buf, size_t cap, size_t *len) {
    const size_t n = args->structureInputDescriptor ? args->structureInputDescriptor->getLength() : args->structureInputSize;
    if (!n || n > cap) return false;
    if (args->structureInputDescriptor) {
        if (args->structureInputDescriptor->readBytes(0, buf, n) != n) return false;
    } else if (args->structureInput) {
        memcpy(buf, args->structureInput, n);
    } else {
        return false;
    }
    *len = n;
    return true;
}
IOReturn kbOut(IOExternalMethodArguments *args, const void *bytes, size_t len) {
    if (args->structureOutputDescriptor) {
        if (len > args->structureOutputDescriptor->getLength()) return kIOReturnNoSpace;
        args->structureOutputDescriptor->writeBytes(0, bytes, len);
        args->structureOutputDescriptorSize = (uint32_t)len;
        return kIOReturnSuccess;
    }
    if (!args->structureOutput || len > args->structureOutputSize) return kIOReturnNoSpace;
    memcpy(args->structureOutput, bytes, len);
    args->structureOutputSize = (uint32_t)len;
    return kIOReturnSuccess;
}
void kbScalarOut(IOExternalMethodArguments *args, uint64_t value) {
    if (args->scalarOutputCount < 1 || !args->scalarOutput) return;
    args->scalarOutput[0] = value;
    args->scalarOutputCount = 1;
}
bool kbHeld(task_t task, const char *name) {
    OSObject *value = IOUserClient::copyClientEntitlement(task, name);
    bool held = value == kOSBooleanTrue;
    OSSafeReleaseNULL(value);
    return held;
}
bool kbProfile(task_t task, const char *name) {
    OSObject *value = IOUserClient::copyClientEntitlement(task, "seatbelt-profiles");
    OSArray *profiles = OSDynamicCast(OSArray, value);
    bool held = false;
    if (profiles) for (unsigned i = 0; i < profiles->getCount(); i++) {
        OSString *profile = OSDynamicCast(OSString, profiles->getObject(i));
        if (profile && profile->isEqualTo(name)) { held = true; break; }
    }
    OSSafeReleaseNULL(value);
    return held;
}
void kbSelfName(char name[32]) {
    name[0] = '?';
    name[1] = 0;
    proc_selfname(name, 32);
}
}  // namespace

static bool gKbOn;   // guamp-aks-keybag=1 (default OFF: try in RAM first)

extern "C" bool native_aks_keybag_enabled(void) {
    return gKbOn && gKbLock;
}

// Once, when the owned AppleKeyStore service starts. The state and lock always exist (the OFF rescue below needs them);
// /defaults and the keybag selectors only with guamp-aks-keybag=1.
extern "C" void native_aks_keybag_boot(void) {
    if (gKbLock) return;
    gkb_init(&gKb);
    gKbOn = kbArg("guamp-aks-keybag", 0) == 1;
    if (!gKbOn) { gKbLock = IOLockAlloc(); return; }
    // keybagd (21H461 0x100004848) reads systembag.kb as a plain plist when IODeviceTree:/defaults has
    // no-effaceable-storage (or when /defaults is missing); otherwise it decrypts it with an effaceable-storage key
    // this phone has not got. Said here, as HakiOS does.
    IORegistryEntry *defaults = IORegistryEntry::fromPath("/defaults", gIODTPlane);
    OSData *yes = OSData::withBytes("\1\0\0\0", 4);
    if (defaults && yes) defaults->setProperty("no-effaceable-storage", yes);
    IOLog("GuampKeybag: on (ent=%u legacy=%u), /defaults %s, bag in %s\n", kbArg("guamp-aks-ent", 1), kbArg("guamp-aks-legacy", 1),
          defaults ? "no-effaceable-storage set" : "absent (keybagd reads plain anyway)", kSystemBagPath);
    OSSafeReleaseNULL(yes);
    OSSafeReleaseNULL(defaults);
    gKbLock = IOLockAlloc();
}

// Rights of a new client, from its entitlements (called in AppleKeyStore::open, in the caller's context).
extern "C" uint32_t native_aks_client_rights(task_t task) {
    if (!native_aks_keybag_enabled()) return 0;
    const unsigned mode = kbArg("guamp-aks-ent", 1);
    if (mode == 0) return GKB_RIGHT_DEVICE | GKB_RIGHT_KEYCHAIN;
    uint32_t rights = gkb_client_rights(kbHeld(task, "com.apple.keystore.device"),
        kbHeld(task, "com.apple.keystore.access-keychain-keys"), kbHeld(task, "platform-application"),
        kbHeld(task, "com.apple.mkb.usersession.info"), kbProfile(task, "com.apple.GSSCred"));
    if (mode == 2 && !rights) {
        char name[32];
        kbSelfName(name);
        if (!strcmp(name, "keybagd")) rights = GKB_RIGHT_DEVICE;
        else if (!strcmp(name, "securityd")) rights = GKB_RIGHT_KEYCHAIN;
    }
    return rights;
}

// Keybag OFF, guamp-aks-wrap=1: an unwrap the GUAMPAKS stand-in cannot do (a key wrapped during a guamp-aks-keybag=1 boot)
// is done with the bag of systembag.kb, read only. So going back to the plain stand-in after a keybag test loses nothing.
static bool kbRescue(uint32_t selector, IOExternalMethodArguments *args, IOReturn *result) {
    static uint32_t rescued;
    uint8_t key[80], out[80];
    size_t len = 0;
    if (selector != 11 || !native_aks_wrap_enabled() || args->scalarInputCount < 2 || !args->scalarInput ||
        !kbIn(args, key, sizeof key, &len)) return false;
    IOLockLock(gKbLock);
    const int rc = gkb_rescue_unwrap(&gKb, &gKbIo, args->scalarInput[1], key, len, out);
    IOLockUnlock(gKbLock);
    if (rc == GKB_OK) {
        *result = kbOut(args, out, len - 8);
        if (rescued < 4) { rescued = rescued + 1; IOLog("GuampKeybag: off, key of a keybag boot unwrapped from systembag.kb -> 0x%x\n", *result); }
    }
    gkb_zero(key, sizeof key);
    gkb_zero(out, sizeof out);
    return rc == GKB_OK;
}

// true when the selector belongs to the keybag (its answer in *result); false: the stand-in answers as before.
extern "C" bool native_aks_keybag_method(uint32_t selector, IOExternalMethodArguments *args, uint32_t rights, IOReturn *result) {
    if (!gKbLock || !args || !result) return false;
    if (!gKbOn) return kbRescue(selector, args, result);
    if (selector != 3 && selector != 4 && selector != 5 && selector != 6 && selector != 10 && selector != 11 &&
        selector != 14 && selector != 23) return false;
    const uint64_t *in = args->scalarInput;
    const uint32_t nin = args->scalarInputCount;
    int rc = GKB_BAD_ARGUMENT;
    IOReturn ret = kIOReturnBadArgument;
    uint64_t handle = 0;
    IOLockLock(gKbLock);
    switch (selector) {
    case 14:                                           // aks_get_system: 0 scalars in (keybagd), 1 out
        if (args->scalarOutputCount < 1 || !args->scalarOutput) break;
        rc = gkb_get_system(&gKb, &gKbIo, rights, &handle);
        if (rc == GKB_OK) kbScalarOut(args, handle);
        ret = kbReturn(rc);
        break;
    case 6: {                                          // aks_load_bag: structure in = the bag, 1 scalar out
        static uint8_t bytes[sizeof(gkb_blob) + 8];
        size_t len = 0;
        if (args->scalarOutputCount < 1 || !args->scalarOutput) break;
        if (!kbIn(args, bytes, sizeof bytes, &len)) { kbLog(nullptr, "aks_load_bag: not a guamp bag (size)", (long)args->structureInputSize); break; }
        rc = gkb_load_bag(&gKb, &gKbIo, rights, bytes, len, &handle);
        gkb_zero(bytes, sizeof bytes);
        if (rc == GKB_OK) kbScalarOut(args, handle);
        ret = kbReturn(rc);
        break;
    }
    case 5:                                            // aks_set_system(handle, ...): 4 scalars in
        if (nin < 1 || !in) break;
        ret = kbReturn(gkb_set_system(&gKb, &gKbIo, rights, in[0]));
        break;
    case 4:                                            // aks_unload_bag(handle)
        if (nin < 1 || !in) break;
        ret = kbReturn(gkb_unload_bag(&gKb, rights, in[0]));
        break;
    case 3: {                                          // aks_save_bag(handle): structure out up to 0x8000
        gkb_blob blob;
        if (nin < 1 || !in) break;
        rc = gkb_save_bag(&gKb, rights, in[0], &blob);
        ret = rc == GKB_OK ? kbOut(args, &blob, sizeof blob) : kbReturn(rc);
        gkb_zero(&blob, sizeof blob);
        break;
    }
    case 23: {                                         // aks_get_bag_uuid(handle): 16 bytes
        gkb_bag *bag = (nin >= 1 && in) ? gkb_bag_for(&gKb, in[0]) : nullptr;
        ret = bag ? kbOut(args, bag->uuid, sizeof bag->uuid) : kIOReturnNotFound;
        break;
    }
    case 10:
    case 11: {                                         // aks_wrap_key / aks_unwrap_key(handle, class), structure in/out
        uint8_t key[80], out[80];
        size_t len = 0;
        int legacy = 0;
        if (nin < 2 || !in || !kbIn(args, key, sizeof key, &len)) break;
        if (selector == 10) {
            rc = gkb_wrap(&gKb, &gKbIo, rights, in[0], in[1], key, len, out);
            ret = rc == GKB_OK ? kbOut(args, out, len + 8) : kbReturn(rc);
            if (ret == kIOReturnSuccess) kbScalarOut(args, in[1]);      // the class asked for is the class used
        } else {
            rc = gkb_unwrap(&gKb, &gKbIo, rights, in[0], in[1], key, len, out, kbArg("guamp-aks-legacy", 1) == 1, &legacy);
            ret = rc == GKB_OK ? kbOut(args, out, len - 8) : kbReturn(rc);
            if (legacy && gKbLegacy < 8) {
                gKbLegacy = gKbLegacy + 1;
                IOLog("GuampKeybag: old stand-in key unwrapped (class %llu): not protected until the item is rewritten\n",
                      (unsigned long long)(in[1] & 0xff));
            }
        }
        gkb_zero(key, sizeof key);
        gkb_zero(out, sizeof out);
        break;
    }
    default: break;
    }
    IOLockUnlock(gKbLock);
    if (ret == kIOReturnNotPrivileged && gKbRefusals < 16) {
        char name[32];
        kbSelfName(name);
        gKbRefusals = gKbRefusals + 1;
        IOLog("GuampKeybag: %s[%d] may not use selector %u (no com.apple.keystore entitlement; guamp-aks-ent=0 or 2 relaxes)\n",
              name, proc_selfpid(), selector);
    }
    // every bag call, and wrap/unwrap only when they fail (securityd makes hundreds)
    if (gKbLogs < 48 && ((selector != 10 && selector != 11) || ret != kIOReturnSuccess)) {
        char name[32];
        kbSelfName(name);
        gKbLogs = gKbLogs + 1;
        IOLog("GuampKeybag: %s sel %u rights %u -> 0x%x\n", name, selector, rights, ret);
    }
    *result = ret;
    return true;
}
