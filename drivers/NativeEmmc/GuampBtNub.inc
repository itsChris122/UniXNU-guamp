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
// This publishes /arm-io/uart1/bluetooth as a plain IOService that takes the device-tree entry's place in the IODeviceTree
// plane (IORegistryEntry::init(from, plane), as IOPlatformExpert::createNub does): it shares the node's property table
// (name "bluetooth", local-mac-address, transport-speed ...; guamp-bt-addr updates stay visible) and is attached in the
// IOService plane below the uart1 nub, then registered. No driver matches it and nothing else changes; the class is
// deliberately not AppleBluetoothModule / AppleBTMgmt (bluetoothd would take its PCIe path).

namespace btnub {
void start() {
  unsigned enabled = 0;
  PE_parse_boot_argn("guamp-bt-nub", &enabled, sizeof(enabled));
  if (enabled != 1) return;
  IORegistryEntry *node = IORegistryEntry::fromPath("/arm-io/uart1/bluetooth", gIODTPlane);
  IORegistryEntry *parentEntry = IORegistryEntry::fromPath("/arm-io/uart1", gIODTPlane);
  IOService *parent = OSDynamicCast(IOService, parentEntry);
  if (!node || !parent) {
    IOLog("GuampBtNub: %s missing, not published\n", !node ? "/arm-io/uart1/bluetooth" : "uart1 nub");
    if (node) node->release();
    if (parentEntry) parentEntry->release();
    return;
  }
  if (OSDynamicCast(IOService, node)) {   // already a service (another publisher): only report it
    IOLog("GuampBtNub: bluetooth node is already a %s, left as is\n", node->getMetaClass()->getClassName());
    node->release();
    parentEntry->release();
    return;
  }
  OSData *mac = OSDynamicCast(OSData, node->getProperty("local-mac-address"));
  const uint8_t *b = mac && mac->getLength() == 6 ? static_cast<const uint8_t *>(mac->getBytesNoCopy()) : nullptr;
  const bool zero = !b || !(b[0] | b[1] | b[2] | b[3] | b[4] | b[5]);
  IOService *nub = OSTypeAlloc(IOService);
  bool ok = nub && nub->init(node, gIODTPlane);
  if (ok) {
    nub->setName("bluetooth");
    ok = nub->attach(parent);
  }
  if (ok) nub->registerService();
  IOLog("GuampBtNub: /arm-io/uart1/bluetooth %s (local-mac-address %s)\n", ok ? "published below uart1" : "not published",
        !b ? "missing" : zero ? "all zero: bluetoothd will still exit" : "set");
  if (nub) nub->release();
  node->release();
  parentEntry->release();
}
}  // namespace btnub
