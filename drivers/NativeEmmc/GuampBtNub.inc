// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Bluetooth registry nub, included by NativeEmmc.cpp after BtService.inc. Off unless guamp-bt-nub=1.
//
// bluetoothd (iPadOS 17.7 /usr/sbin/bluetoothd, FindLocalMacAddress at 0x10009abc0) reads the controller address as
//   IOServiceGetMatchingService(IOServiceNameMatching("marconi-bt"), then "bluetooth")
//   IORegistryEntrySearchCFProperty(service, "IODeviceTree", "local-mac-address", recursive) -> 6 bytes
// and leaves with errx "No bluetooth on this device" (0x10009a374) when no service matches ("Unable to match bluetooth
// service in device tree") or the six bytes are zero, before it ever opens /dev/cu.bluetooth. On Apple hardware the
// UART driver publishes its bluetooth child; here ArmDTPlatform publishes nubs for the direct /arm-io children only
// (uart1 gets one, its bluetooth child none), so the ADT node alone is not found.
//
// This attaches the /arm-io/uart1/bluetooth entry itself (an IOService outside the IOService plane, like every
// device-tree entry) below the uart1 nub in the IOService plane and registers it, so the name match finds it and the
// IODeviceTree search reads its own local-mac-address (guamp-bt-addr updates stay visible). No driver matches it and
// nothing else changes; the class is deliberately not AppleBluetoothModule / AppleBTMgmt (bluetoothd would take its
// PCIe path).

namespace btnub {
// The boot-time lines come before the USB log mirror attaches: a thread call repeats the outcome every 60 s (15 times,
// then every 10 min), looked up the way bluetoothd does it (service name match, then local-mac-address through the
// IODeviceTree plane, recursive), so the line tells what bluetoothd itself would find at that moment.
static const char *gPublish = "not run";
static thread_call_t gStatusCall;
static unsigned gStatusRound;
static const char *macState(IOService *s) {
  if (!s) return "-";
  OSObject *o = s->copyProperty("local-mac-address", gIODTPlane, kIORegistryIterateRecursively);
  OSData *d = OSDynamicCast(OSData, o);
  const uint8_t *b = d && d->getLength() >= 6 ? static_cast<const uint8_t *>(d->getBytesNoCopy()) : nullptr;
  const char *r = !b ? "missing" : (b[0] | b[1] | b[2] | b[3] | b[4] | b[5]) ? "set" : "all zero";
  if (o) o->release();
  return r;
}
static IOService *byName(const char *name) {
  OSDictionary *m = IOService::nameMatching(name);
  IOService *s = m ? IOService::copyMatchingService(m) : nullptr;
  if (m) m->release();
  return s;
}
static void status(thread_call_param_t, thread_call_param_t) {
  IOService *marconi = byName("marconi-bt"), *bt = marconi ? nullptr : byName("bluetooth");
  IOService *s = marconi ? marconi : bt;
  gStatusRound = gStatusRound + 1;
  IOLog("GuampBtNub: status %u: publish %s; name match %s -> %s, local-mac-address %s\n", gStatusRound, gPublish,
        marconi ? "marconi-bt" : "bluetooth", s ? s->getMetaClass()->getClassName() : "none (bluetoothd: no bluetooth)",
        macState(s));
  if (s) s->release();
  uint64_t deadline = 0;
  clock_interval_to_deadline(gStatusRound < 15 ? 60 : 600, kSecondScale, &deadline);
  thread_call_enter_delayed(gStatusCall, deadline);
}
static void publish();
void start() {
  unsigned enabled = 0;
  PE_parse_boot_argn("guamp-bt-nub", &enabled, sizeof(enabled));
  if (enabled != 1) return;
  publish();
  gStatusCall = thread_call_allocate(status, nullptr);
  if (gStatusCall) {
    uint64_t deadline = 0;
    clock_interval_to_deadline(30, kSecondScale, &deadline);
    thread_call_enter_delayed(gStatusCall, deadline);
  }
}
static void publish() {
  IORegistryEntry *node = IORegistryEntry::fromPath("/arm-io/uart1/bluetooth", gIODTPlane);
  IORegistryEntry *parentEntry = IORegistryEntry::fromPath("/arm-io/uart1", gIODTPlane);
  IOService *parent = OSDynamicCast(IOService, parentEntry);
  if (!node || !parent) {
    gPublish = !node ? "no (no /arm-io/uart1/bluetooth node)" : "no (no uart1 nub)";
    IOLog("GuampBtNub: %s missing, not published\n", !node ? "/arm-io/uart1/bluetooth" : "uart1 nub");
    if (node) node->release();
    if (parentEntry) parentEntry->release();
    return;
  }
  OSData *mac = OSDynamicCast(OSData, node->getProperty("local-mac-address"));
  const uint8_t *b = mac && mac->getLength() == 6 ? static_cast<const uint8_t *>(mac->getBytesNoCopy()) : nullptr;
  const bool zero = !b || !(b[0] | b[1] | b[2] | b[3] | b[4] | b[5]);
  // Device-tree entries are IOService objects already (IODeviceTreeSupport MakeReferenceTable: new IOService, named
  // from the node's "name"), just not in the IOService plane: such an entry is attached below the uart1 nub and
  // registered as it is. Only an entry that is not a service gets a separate nub taking its place.
  IOService *service = OSDynamicCast(IOService, node);
  bool ok = false;
  if (service && service->inPlane(gIOServicePlane)) {
    gPublish = "already registered";
    ok = true;
  } else if (service) {
    ok = service->attach(parent);
    if (ok) service->registerService();
    gPublish = ok ? "yes (device-tree entry below uart1)" : "no (attach failed)";
  } else {
    IOService *nub = OSTypeAlloc(IOService);
    ok = nub && nub->init(node, gIODTPlane);
    if (ok) {
      nub->setName("bluetooth");
      ok = nub->attach(parent);
    }
    if (ok) nub->registerService();
    gPublish = ok ? "yes (new nub below uart1)" : "no (init/attach failed)";
    if (nub) nub->release();
  }
  IOLog("GuampBtNub: /arm-io/uart1/bluetooth %s (local-mac-address %s)\n", gPublish,
        !b ? "missing" : zero ? "all zero: bluetoothd will still exit" : "set");
  node->release();
  parentEntry->release();
}
}  // namespace btnub
