// SPDX-License-Identifier: APSL-2.0
// guamp Wi-Fi mode 2 registry entries, included by NativeEmmc.cpp only with GUAMP_WIFI_REDWOOD=1
// (scripts/build-arm-driver.py NativeEmmc --wifi-runtime --wifi-redwood). Nothing is published unless the iOS station
// of guamp-wifi-start=2 attaches en0 (drivers/GuampWifi/ios/guamp_net.c calls guamp_wlan_publish once).
//
// From HakiOS xnu/files/iokit/Kernel/hakios.wlan.cpp (github.com/oosexgang/hackios-xnu-kernel 8851317, Apple Public
// Source License 2.0, see drivers/GuampWifi/ios/README.md); changed: the service is GuampWLAN, log lines "GuampWLAN:".
//
// en0 is made with the BSD interface KPI and has no IOKit object behind it of its own; iOS looks for two, as
// IO80211Family would have made them:
//   - wifid's device is whatever service has IO80211InterfaceRole = "Infrastructure", and IO80211.framework binds to an
//     interface through a service whose IOInterfaceName is the interface's name (IOPropertyMatch). On iPadOS 17.7.11
//     (IO80211.framework Apple80211BindToInterfaceWithService, read out of the dyld cache on 29/10) the bind FAILS when
//     IOServiceOpen(service, type 0) fails: HakiOS's "no user client" of iOS 16.5 is not enough any more. So there is
//     a small user client (IO80211UserClient below) that answers only what the bind asks over it:
//       selector 0, struct in 0x3c30 = { u32 ioctl; u32 pad; u8 data[0x3c00]; struct apple80211req at 0x3c08 }, the
//                   "sanity test": DRIVER_VERSION (43), out = the version as a C string (up to 0x100 bytes);
//       selector 1, struct out 1 byte = useIOUCWhenPossible: 0, so every Apple80211 request after the bind goes the
//                   ioctl path (SIOCGA80211/SIOCSA80211 on a socket), answered in guamp_apple80211.c;
//       selector 5 (useIOUCEventPipe): refused, so the framework listens on the PF_SYSTEM kernel-event socket
//                   (KEV_IEEE80211_CLASS) that guamp_apple80211.c posts to.
//     Everything else is refused (kIOReturnUnsupported) and logged a few times.
//   - SystemConfiguration lists network interfaces as IONetworkInterface objects and calls one Wi-Fi when its parent is
//     an IO80211Controller; the hardware address is a property of the controller.
// Both classes are ours, named as IO80211Family and IONetworkingFamily name theirs (neither family is in this kernel or
// its prelinked set); neither has anything but properties. How iPadOS 17.7.11's wifid takes them is not verified on the
// phone yet (analisi/wifi/IOS-WIFID-20261029.md).

class IO80211Controller : public IOService {
  OSDeclareDefaultStructors(IO80211Controller)
};
OSDefineMetaClassAndStructors(IO80211Controller, IOService)

class IONetworkInterface : public IOService {
  OSDeclareDefaultStructors(IONetworkInterface)
};
OSDefineMetaClassAndStructors(IONetworkInterface, IOService)

// The user client of en0's service (see the top). Named as IO80211Family names its own, in case a sandbox profile
// lists the class.
class IO80211UserClient : public IOUserClient {
  OSDeclareDefaultStructors(IO80211UserClient)
public:
  IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *arguments, IOExternalMethodDispatch *dispatch,
                          OSObject *target, void *reference) APPLE_KEXT_OVERRIDE;
  IOReturn clientClose() APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(IO80211UserClient, IOUserClient)

class GuampWLAN : public IONetworkInterface {
  OSDeclareDefaultStructors(GuampWLAN)
public:
  IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties,
                         IOUserClient **handler) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(GuampWLAN, IONetworkInterface)

namespace wlannub {
static unsigned gUserClientAsked;
static unsigned gSelectorsSaid;
constexpr uint32_t kIoctlBlockSize = 0x3c30;   // IO80211.framework 17.7.11: the selector-0 input
constexpr uint32_t kIoctlRequestAt = 0x3c08;   // struct apple80211req inside it
constexpr uint32_t kGetA80211 = 0xc02869c9u;   // SIOCGA80211
constexpr int32_t kDriverVersion = 43;         // APPLE80211_IOC_DRIVER_VERSION
static const char kVersion[] = "guamp ath10k WCN3990 (Apple80211 over ioctl)";
// attach, start, register: the service, or nullptr
static IOService *publish(IOService *service, IOService *provider, const char *name) {
  if (!service || !service->attach(provider)) {
    OSSafeReleaseNULL(service);
    return nullptr;
  }
  service->setName(name);
  if (!service->start(provider)) {
    service->detach(provider);
    service->release();
    return nullptr;
  }
  service->registerService();
  return service;
}
}  // namespace wlannub

// Type 0 only (what Apple80211BindToInterfaceWithService opens); that somebody asked shows that the service was found.
IOReturn GuampWLAN::newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *, IOUserClient **handler) {
  bool said = wlannub::gUserClientAsked < 8;
  if (said) {
    wlannub::gUserClientAsked = wlannub::gUserClientAsked + 1;
  }
  if (!handler || type != 0) {
    if (said) {
      IOLog("GuampWLAN: a user client of type %u is asked for: refused\n", static_cast<unsigned>(type));
    }
    return kIOReturnUnsupported;
  }
  IO80211UserClient *client = OSTypeAlloc(IO80211UserClient);
  if (!client || !client->initWithTask(owningTask, securityID, type)) {
    OSSafeReleaseNULL(client);
    return kIOReturnNoMemory;
  }
  if (!client->attach(this)) {
    client->release();
    return kIOReturnError;
  }
  if (!client->start(this)) {
    client->detach(this);
    client->release();
    return kIOReturnError;
  }
  if (said) {
    char who[32] = "?";
    proc_selfname(who, sizeof(who));
    IOLog("GuampWLAN: user client of type 0 opened by %s\n", who);
  }
  *handler = client;
  return kIOReturnSuccess;
}

IOReturn IO80211UserClient::clientClose() {
  if (!isInactive()) {
    terminate();
  }
  return kIOReturnSuccess;
}

IOReturn IO80211UserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *arguments,
                                           IOExternalMethodDispatch *, OSObject *, void *) {
  IOReturn result = kIOReturnUnsupported;
  int32_t type = -1;
  if (!arguments) {
    return kIOReturnBadArgument;
  }
  IOMemoryDescriptor *descriptor = arguments->structureInputDescriptor;
  uint32_t size = descriptor ? static_cast<uint32_t>(descriptor->getLength()) : arguments->structureInputSize;
  switch (selector) {
  case 0: {  // one Apple80211 request in a block; only the bind's sanity test is answered here
    uint8_t request[0x28] = {};
    uint32_t command = 0;
    if (size < wlannub::kIoctlBlockSize) {
      result = kIOReturnBadArgument;
      break;
    }
    if (descriptor) {
      // (0x3c30 bytes are more than fit in the message: they come as a memory descriptor)
      if (descriptor->prepare() != kIOReturnSuccess) {
        result = kIOReturnVMError;
        break;
      }
      bool read = descriptor->readBytes(0, &command, sizeof(command)) == sizeof(command) &&
                  descriptor->readBytes(wlannub::kIoctlRequestAt, request, sizeof(request)) == sizeof(request);
      descriptor->complete();
      if (!read) {
        result = kIOReturnVMError;
        break;
      }
    } else if (arguments->structureInput) {
      memcpy(&command, arguments->structureInput, sizeof(command));
      memcpy(request, static_cast<const uint8_t *>(arguments->structureInput) + wlannub::kIoctlRequestAt, sizeof(request));
    } else {
      result = kIOReturnBadArgument;
      break;
    }
    memcpy(&type, request + 16, sizeof(type));  // req_if_name[16], then req_type
    if (command == wlannub::kGetA80211 && type == wlannub::kDriverVersion && arguments->structureOutput &&
        arguments->structureOutputSize > 0) {
      uint32_t length = arguments->structureOutputSize < sizeof(wlannub::kVersion)
                            ? arguments->structureOutputSize
                            : static_cast<uint32_t>(sizeof(wlannub::kVersion));
      bzero(arguments->structureOutput, arguments->structureOutputSize);
      memcpy(arguments->structureOutput, wlannub::kVersion, length - 1);
      arguments->structureOutputSize = length;
      result = kIOReturnSuccess;
    }
    break;
  }
  case 1:  // useIOUCWhenPossible: no, the ioctl path
    if (arguments->structureOutput && arguments->structureOutputSize >= 1) {
      bzero(arguments->structureOutput, arguments->structureOutputSize);
      result = kIOReturnSuccess;
    }
    break;
  default:  // 5 = useIOUCEventPipe and the rest: refused (events go over the kernel-event socket)
    break;
  }
  if (wlannub::gSelectorsSaid < 40) {
    wlannub::gSelectorsSaid = wlannub::gSelectorsSaid + 1;
    IOLog("GuampWLAN: user client selector %u (type %d, in %u, out %u): %s\n", selector, static_cast<int>(type), size,
          arguments->structureOutputSize, result == kIOReturnSuccess ? "answered" : "refused");
  }
  return result;
}

// Once, when en0 of the radio exists (guamp_net.c hakios_net_attach, on the station's attach thread).
extern "C" void guamp_wlan_publish(const char *interface, const uint8_t *address) {
  IOService *platform = IOService::getPlatform();
  IO80211Controller *controller = OSTypeAlloc(IO80211Controller);
  GuampWLAN *wlan = OSTypeAlloc(GuampWLAN);
  if (!platform || !controller || !wlan || !controller->init() || !wlan->init() || !interface || !address) {
    OSSafeReleaseNULL(controller);
    OSSafeReleaseNULL(wlan);
    IOLog("GuampWLAN: cannot create the services\n");
    return;
  }
  if (OSData *mac = OSData::withBytes(address, 6)) {
    controller->setProperty("IOMACAddress", mac);
    mac->release();
  }
  controller->setProperty("IOBuiltin", true);
  controller = static_cast<IO80211Controller *>(wlannub::publish(controller, platform, "GuampWLANController"));
  if (!controller) {
    wlan->release();
    IOLog("GuampWLAN: the controller did not start\n");
    return;
  }
  wlan->setProperty("IOInterfaceName", interface);
  wlan->setProperty("BSD Name", interface);
  wlan->setProperty("IOInterfaceNamePrefix", "en");
  if (OSNumber *number = OSNumber::withNumber(6ULL, 32)) {   // IFT_ETHER
    wlan->setProperty("IOInterfaceType", number);
    number->release();
  }
  if (OSNumber *number = OSNumber::withNumber(static_cast<unsigned long long>(interface[2] - '0'), 32)) {
    wlan->setProperty("IOInterfaceUnit", number);
    number->release();
  }
  wlan->setProperty("IOBuiltin", true);
  wlan->setProperty("IOPrimaryInterface", true);
  wlan->setProperty("IO80211InterfaceRole", "Infrastructure");
  if (!wlannub::publish(wlan, controller, "GuampWLAN")) {
    IOLog("GuampWLAN: the service did not start\n");
  } else {
    IOLog("GuampWLAN: IO80211Controller and IONetworkInterface %s published\n", interface);
    wlan->release();
  }
  controller->release();
}
