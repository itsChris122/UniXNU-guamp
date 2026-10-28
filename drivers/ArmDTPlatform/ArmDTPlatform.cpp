// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Generic AArch64 Darwin platform expert for a supplied Apple-format device
// tree. It derives from XNU's real IODTPlatformExpert and relies on the
// platform's own DT root name ("device-tree") for matching. No board name,
// SoC address or vendor identity is compiled into this source.
//
// The DT must contain the nodes XNU's IODT platform path consumes:
//   /chosen with a CHRP NVRAM proxy (nvram-bank-size, nvram-proxy-data)
//   /options, /cpus, /arm-io and /defaults
// The NVRAM proxy is required so the real IODTNVRAM is published before any
// security initialization. This driver does not fabricate NVRAM or suppress
// a missing one; absent inputs make configure() fail.

#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <libkern/c++/OSData.h>
#include <libkern/c++/OSIterator.h>
#include <pexpert/pexpert.h>

// guamp: eMMC driver (drivers/NativeEmmc), started without a personality once the platform is configured
extern "C" void guamp_emmc_start (IOService *platform);
extern "C" void guamp_emmc_flush (unsigned type);
// guamp: wall clock from the PM6125 RTC + saved offset (drivers/NativeEmmc/GuampRtc.inc); 0 = not available
extern "C" int guamp_rtc_get (clock_sec_t *secs, clock_nsec_t *nsecs);
extern "C" int guamp_rtc_set (clock_sec_t secs, clock_nsec_t nsecs);

// Observation counters describe actual published nubs, not started drivers.
extern "C" {
volatile uint32_t armdt_platform_configured = 0;
volatile uint32_t armdt_platform_child_count = 0;
volatile uint32_t armdt_platform_cpu_count = 0;
volatile uint32_t armdt_platform_bus_count = 0;
volatile uint32_t armdt_platform_defaults_count = 0;
volatile uint32_t armdt_platform_nvram_proxy = 0;
volatile uint32_t armdt_platform_device_count = 0;
}

class ArmDTPlatform : public IODTPlatformExpert {
  OSDeclareDefaultStructors (ArmDTPlatform)
public:
  bool configure (IOService *provider) override;
  const char *deleteList (void) override;
  const char *excludeList (void) override;
  int haltRestart (unsigned int type) override;
  void getUTCTimeOfDay (clock_sec_t *secs, clock_nsec_t *nsecs) override;
  void setUTCTimeOfDay (clock_sec_t secs, clock_nsec_t nsecs) override;
private:
  bool hasNVRAMProxy (IOService *provider);
  bool createDeviceNubs (IOService *provider);
};

OSDefineMetaClassAndStructors (ArmDTPlatform, IODTPlatformExpert)

// The platform clock of PEGetUTCTimeOfDay (clock_initialize_calendar) and PESetUTCTimeOfDay (settimeofday), as
// Apple's RTC drivers provide it; without the guamp RTC the IOPlatformExpert defaults stay (0 / ignored).
void ArmDTPlatform::getUTCTimeOfDay (clock_sec_t *secs, clock_nsec_t *nsecs)
{
  if (!guamp_rtc_get (secs, nsecs)) {
    IODTPlatformExpert::getUTCTimeOfDay (secs, nsecs);
  }
}

void ArmDTPlatform::setUTCTimeOfDay (clock_sec_t secs, clock_nsec_t nsecs)
{
  if (!guamp_rtc_set (secs, nsecs)) {
    IODTPlatformExpert::setUTCTimeOfDay (secs, nsecs);
  }
}

// Serialized empty name lists: XNU's processTopLevel() treats a null pointer
// as "every entry" (remove every child), not as "nothing". An empty list
// genuinely selects no entry, so the supplied device tree is preserved.
const char *ArmDTPlatform::deleteList (void)
{
  return "()";
}

const char *ArmDTPlatform::excludeList (void)
{
  return "()";
}

bool ArmDTPlatform::hasNVRAMProxy (IOService *provider)
{
  IORegistryEntry *chosen = provider->childFromPath ("chosen", gIODTPlane);
  if (chosen == nullptr) {
    return false;
  }
  OSData *bankSize = OSDynamicCast (OSData, chosen->getProperty ("nvram-bank-size"));
  OSData *proxyData = OSDynamicCast (OSData, chosen->getProperty ("nvram-proxy-data"));
  bool valid = (bankSize != nullptr) && (bankSize->getLength () == sizeof (uint32_t)) &&
               (proxyData != nullptr) && (proxyData->getLength () != 0);
  chosen->release ();
  return valid;
}

bool ArmDTPlatform::createDeviceNubs (IOService *provider)
{
  IORegistryEntry *bus = provider->childFromPath ("arm-io", gIODTPlane);
  if (bus == nullptr) {
    return false;
  }
  // IODTPlatformExpert's IOPlatformDevice nubs resolve resources through
  // their provider's IOPlatformExpert::getNubResources. They must therefore
  // be attached to this platform expert, not to the arm-io IOPlatformDevice
  // (ArmVirtPlatform uses the same parent relationship). The device-tree
  // parent remains arm-io, so reg/ranges resolution still follows the bus.
  // Publish every real arm-io child, not only the first pattern match. The
  // GIC lives here, and the UFS host/PHY nodes must be visible to their native
  // drivers too. processTopLevel() still uses deleteList()/excludeList().
  OSIterator *iterator = bus->getChildIterator (gIODTPlane);
  bus->release ();
  if (iterator == nullptr) {
    return false;
  }
  uint32_t count = 0;
  IORegistryEntry *entry;
  while ((entry = OSDynamicCast (IORegistryEntry, iterator->getNextObject ())) != nullptr) {
    IOService *nub = createNub (entry);
    if (nub == nullptr) {
      continue;
    }
    if (!nub->attach (this)) {
      nub->release ();
      continue;
    }
    nub->registerService ();
    nub->release ();
    count++;
  }
  iterator->release ();
  armdt_platform_device_count = count;
  return true;
}

static void platformStageReset (void)
{
  // PSCI SYSTEM_RESET via the same SMC conduit as the entry reset probe.
  register uint64_t x0 __asm__ ("x0") = 0x84000009;
  register uint64_t x1 __asm__ ("x1") = 0;
  register uint64_t x2 __asm__ ("x2") = 0;
  register uint64_t x3 __asm__ ("x3") = 0;
  __asm__ volatile ("smc #0" : "+r" (x0) : "r" (x1), "r" (x2), "r" (x3) : "memory");
  for (;;) { __asm__ volatile ("wfe"); }
}

// guamp: the inherited haltRestart has no way to reset or power off this board, so a reboot request (or a
// panic) left the machine hung. Save the eMMC log and framebuffer copy (drivers/NativeEmmc), then use PSCI.
static void platformPsci (uint64_t function)
{
  register uint64_t x0 __asm__ ("x0") = function;
  __asm__ volatile ("smc #0" : "+r" (x0) : : "memory");
}

int ArmDTPlatform::haltRestart (unsigned int type)
{
  guamp_emmc_flush (type);
  switch (type) {
  case kPEHaltCPU:
  case kPEUPSDelayHaltCPU:
    platformPsci (0x84000008);   // SYSTEM_OFF
    break;
  case kPERestartCPU:
  case kPEPanicRestartCPU:
  case kPEPanicRestartCPUNoCallouts:
  case kPEPanicEnd:
    platformPsci (0x84000009);   // SYSTEM_RESET
    break;
  default:
    break;
  }
  return IODTPlatformExpert::haltRestart (type);
}

bool ArmDTPlatform::configure (IOService *provider)
{
  IORegistryEntry *cpus = provider->childFromPath ("cpus", gIODTPlane);
  IORegistryEntry *bus = provider->childFromPath ("arm-io", gIODTPlane);
  IORegistryEntry *defaults = provider->childFromPath ("defaults", gIODTPlane);
  IORegistryEntry *options = provider->childFromPath ("options", gIODTPlane);
  bool structure = (cpus != nullptr) && (bus != nullptr) &&
                   (defaults != nullptr) && (options != nullptr);
  if (cpus != nullptr) { cpus->release (); }
  if (bus != nullptr) { bus->release (); }
  if (defaults != nullptr) { defaults->release (); }
  if (options != nullptr) { options->release (); }
  if (!structure) {
    IOLog ("ArmDTPlatform: required /cpus, /arm-io, /defaults or /options node is absent\n");
    return false;
  }
  if (!hasNVRAMProxy (provider)) {
    IOLog ("ArmDTPlatform: /chosen does not supply a usable CHRP NVRAM proxy\n");
    return false;
  }
  unsigned platformStage = 0;
  PE_parse_boot_argn ("sheng-platform-stage", &platformStage, sizeof (platformStage));
  if (platformStage == 1) {
    platformStageReset ();
  }

  // The inherited implementation publishes real IODTNVRAM from /chosen,
  // then attaches the CPU and top-level nubs. No identity or resource is
  // invented here; failure is returned instead of replaced with a stub.
  if (!IODTPlatformExpert::configure (provider)) {
    return false;
  }
  if (platformStage == 2) {
    platformStageReset ();
  }

  OSIterator *iterator = getChildIterator (gIOServicePlane);
  if (iterator == nullptr) {
    return false;
  }
  uint32_t count = 0;
  uint32_t cpuCount = 0;
  uint32_t busCount = 0;
  uint32_t defaultsCount = 0;
  IORegistryEntry *child;
  while ((child = OSDynamicCast (IORegistryEntry, iterator->getNextObject ())) != nullptr) {
    count++;
    OSData *type = OSDynamicCast (OSData, child->getProperty ("device_type"));
    if ((type != nullptr) && (type->getLength () == 4) &&
        (memcmp (type->getBytesNoCopy (), "cpu", 4) == 0)) {
      cpuCount++;
    }
    if (strcmp (child->getName (), "arm-io") == 0) {
      busCount++;
    }
    if (strcmp (child->getName (), "defaults") == 0) {
      defaultsCount++;
    }
  }
  iterator->release ();
  armdt_platform_child_count = count;
  armdt_platform_cpu_count = cpuCount;
  armdt_platform_bus_count = busCount;
  armdt_platform_defaults_count = defaultsCount;
  armdt_platform_nvram_proxy = 1;
  armdt_platform_configured = 1;
  // Publish the bus child devices (interrupt controller, VirtIO MMIO, ...)
  // as real IOPlatformDevice nubs. No driver is started here.
  if (!createDeviceNubs (provider)) {
    return false;
  }
  if (platformStage == 3) {
    platformStageReset ();
  }
  // Explicit hardware A/B diagnostic: reach the end of the real platform
  // configuration and panic before publishing the service. The payload's
  // panic-screen probe makes this observable without a serial console.
  unsigned platformProbe = 0;
  PE_parse_boot_argn ("sheng-platform-panic", &platformProbe, sizeof (platformProbe));
  if (platformProbe != 0 || platformStage == 4) {
    platformStageReset ();
  }
  registerService ();
  guamp_emmc_start (this);
  if (platformStage == 5) {
    platformStageReset ();
  }
  return true;
}
