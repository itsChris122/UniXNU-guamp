// SPDX-License-Identifier: BSD-2-Clause-Patent
// GuampSim IOKit service: the SIM snapshot (GuampSimInfo.h) as registry properties, read by the userland
// CoreTelephony stand-in guamp-ctd (IOServiceNameMatching("GuampSim")). Producers: the Wi-Fi/modem runtime
// (guamp-sim=1, GuampSim.inc SimProbe via guamp_sim_publish) and guamp-sim-fake=1 (fixed 3GPP test values,
// published at start, no modem). Read only for user space: setProperties is not implemented.
// Real identifiers go only to the registry; the kernel log shows the last 4 digits.
#include "GuampSimInfo.h"

class GuampSimService : public IOService {
  OSDeclareDefaultStructors(GuampSimService)
public:
  void number(const char *key, uint64_t value) { setProperty(key, value, 32); }
  void text(const char *key, const char *value, size_t cap) {
    char buffer[72] = {};
    size_t n = 0;
    while (n < cap && n + 1 < sizeof(buffer) && value[n]) { buffer[n] = value[n]; n++; }
    setProperty(key, buffer);
  }
};
OSDefineMetaClassAndStructors(GuampSimService, IOService)

namespace simsvc {
IOLock *gLock;
GuampSimService *gService;

const char *stateName(uint8_t s) {
  static const char *const names[] = {"unknown", "absent", "error", "detected", "pin", "puk", "blocked", "perso", "ready"};
  return s < sizeof(names) / sizeof(names[0]) ? names[s] : "unknown";
}

GuampSimService *create() {
  IOService *platform = IOService::getPlatform();
  GuampSimService *made = OSTypeAlloc(GuampSimService);
  if (!platform || !made || !made->init()) {
    if (made) made->release();
    IOLog("GuampSim: IOKit service not created\n");
    return nullptr;
  }
  made->setName("GuampSim");
  if (!made->attach(platform)) { made->release(); IOLog("GuampSim: IOKit service not attached\n"); return nullptr; }
  if (!made->start(platform)) {
    made->detach(platform);
    made->release();
    IOLog("GuampSim: IOKit service not started\n");
    return nullptr;
  }
  return made;
}

void publish(const GuampSimInfo &i) {
  if (!gLock) return;
  IOLockLock(gLock);
  bool fresh = false;
  if (!gService) { gService = create(); fresh = gService != nullptr; }
  if (GuampSimService *s = gService) {
    s->number("SimInfoVersion", i.version);
    s->number("SimSnapshots", i.snapshots);
    s->setProperty("SimFake", i.fake ? kOSBooleanTrue : kOSBooleanFalse);
    s->setProperty("SimPresent", i.state >= GUAMP_SIM_ERROR ? kOSBooleanTrue : kOSBooleanFalse);
    s->setProperty("SimState", stateName(i.state));
    s->number("SimAppType", i.appType);
    s->number("SimPin1Retries", i.pin1Retries);
    s->number("SimPuk1Retries", i.puk1Retries);
    s->number("ModemOperatingMode", i.operatingMode);
    s->number("Registration", i.registration);
    s->number("CsAttached", i.csAttached);
    s->number("PsAttached", i.psAttached);
    s->number("Roaming", i.roaming);
    s->number("RadioInterface", i.radio);
    s->setProperty("HasPlmn", i.hasPlmn ? kOSBooleanTrue : kOSBooleanFalse);
    s->number("MCC", i.mcc);
    s->number("MNC", i.mnc);
    s->text("ICCID", i.iccid, sizeof(i.iccid));
    s->text("IMSI", i.imsi, sizeof(i.imsi));
    s->text("SPN", i.spn, sizeof(i.spn));
    s->text("MSISDN", i.msisdn, sizeof(i.msisdn));
    s->text("IMEI", i.imei, sizeof(i.imei));
    s->text("MEID", i.meid, sizeof(i.meid));
    s->text("ModemRevision", i.revision, sizeof(i.revision));
    s->text("PlmnName", i.plmn, sizeof(i.plmn));
    if (fresh) s->registerService();
    else s->messageClients(kIOMessageServicePropertyChange);
  }
  IOLockUnlock(gLock);
}

// startLater (thread call), before wifiprep::start(): lock, then the optional fake snapshot.
void start() {
  if (gLock) return;
  gLock = IOLockAlloc();
  unsigned fake = 0;
  PE_parse_boot_argn("guamp-sim-fake", &fake, sizeof(fake));
  if (fake != 1) return;
  GuampSimInfo info;
  guampsim::fakeInfo(&info);
  publish(info);
  IOLog("GuampSim: guamp-sim-fake=1: test SIM published (state %s, PLMN %03u/%02u \"%s\"), no modem involved\n",
        stateName(info.state), info.mcc, info.mnc, info.plmn);
}
}  // namespace simsvc

extern "C" void guamp_sim_publish(const struct GuampSimInfo *info) {
  if (info && info->version == GUAMP_SIM_INFO_VERSION) simsvc::publish(*info);
}
