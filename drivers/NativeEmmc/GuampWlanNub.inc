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
//     interface through a service whose IOInterfaceName is the interface's name. The service has no user client, on
//     purpose: the framework falls back to the ioctl path (SIOCGA80211/SIOCSA80211), answered in guamp_apple80211.c.
//   - SystemConfiguration lists network interfaces as IONetworkInterface objects and calls one Wi-Fi when its parent is
//     an IO80211Controller; the hardware address is a property of the controller.
// Both classes are ours, named as IO80211Family and IONetworkingFamily name theirs (neither family is in this kernel or
// its prelinked set); neither has anything but properties. How iPadOS 17.7.11's wifid takes them is not verified yet.

class IO80211Controller : public IOService {
  OSDeclareDefaultStructors(IO80211Controller)
};
OSDefineMetaClassAndStructors(IO80211Controller, IOService)

class IONetworkInterface : public IOService {
  OSDeclareDefaultStructors(IONetworkInterface)
};
OSDefineMetaClassAndStructors(IONetworkInterface, IOService)

class GuampWLAN : public IONetworkInterface {
  OSDeclareDefaultStructors(GuampWLAN)
public:
  IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties,
                         IOUserClient **handler) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(GuampWLAN, IONetworkInterface)

namespace wlannub {
static unsigned gUserClientAsked;
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

// There is none (see the top); that somebody asked shows that the service was found.
IOReturn GuampWLAN::newUserClient(task_t, void *, UInt32 type, OSDictionary *, IOUserClient **) {
  if (wlannub::gUserClientAsked < 8) {
    wlannub::gUserClientAsked = wlannub::gUserClientAsked + 1;
    IOLog("GuampWLAN: a user client of type %u is asked for: there is none (ioctl path)\n", static_cast<unsigned>(type));
  }
  return kIOReturnUnsupported;
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
