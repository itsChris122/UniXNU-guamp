// SPDX-License-Identifier: BSD-2-Clause-Patent
// Included inside pmic, after read8/ioRd/ioWr. Called only by the PMIC thread.
// Uses read-only QG/charger registers; does not initialize or change charge hardware.
// Scale/register facts: qg-reg.h, qg-defs.h, qg-util.c, smb5-reg.h in the guamp
// kernel c65462bf08da2a8900b98e705bea376de7f7f19c. Profile: JK50-SWD, 5029mAh @25C.
class GuampBattery : public IOPMPowerSource {
  OSDeclareDefaultStructors(GuampBattery)
public:
  void number(const char *key, uint64_t value) {
    const OSSymbol *symbol = OSSymbol::withCString(key);
    OSNumber *n = OSNumber::withNumber(value, 64);
    if (symbol && n) setPSProperty(symbol, n);
    if (symbol) symbol->release();
    if (n) n->release();
  }
  void flag(const char *key, bool value) {
    const OSSymbol *symbol = OSSymbol::withCString(key);
    if (symbol) { setPSProperty(symbol, value ? kOSBooleanTrue : kOSBooleanFalse); symbol->release(); }
  }
  void report(unsigned percent, unsigned uv, int ua, bool external, unsigned state) {
    setBatteryInstalled(true);
    setExternalConnected(external);
    setExternalChargeCapable(external);
    setIsCharging(external && state >= 1 && state <= 4 && ua > 0);
    setCurrentCapacity(percent); setMaxCapacity(100);
    setVoltage(uv / 1000); setAmperage(ua / 1000);
    setTimeRemaining(-1);
    setAtWarnLevel(!external && percent <= 10);
    setAtCriticalLevel(!external && percent <= 3);
    number("InstantAmperage", static_cast<uint64_t>(static_cast<int64_t>(ua / 1000)));
    number("DesignCapacity", 5029);
    flag("FullyCharged", percent >= 99 && guamp_battery::fullAtProfileVoltage(external, state, uv));
    flag("AppleRawExternalConnected", external);
    flag("GuampLevelEstimated", true);
    flag("GuampSampleValid", true);
    updateStatus();
  }
  void invalid() { flag("GuampSampleValid", false); updateStatus(); }
};
OSDefineMetaClassAndStructors(GuampBattery, IOPMPowerSource)
static GuampBattery *gBattery;
static guamp_battery::Balance gBalance;
static bool gBatteryEnabled;
static unsigned gBatteryErrors, gBatterySamples, gBatterySkipped;

static int batteryRead16(uint16_t address, uint16_t &out) {
  int apid = apidOf(2, address);
  if (apid < 0) return apid;
  uint32_t channel = 0x80u * static_cast<uint32_t>(apid);
  uint16_t previous = 0;
  for (unsigned n = 0; n < 4; ++n) {
    ioWr(nullptr, SPMI_OBSRVR, channel, (1u << 27) | (static_cast<uint32_t>(address & 0xff) << 4) | 1u);
    int error = waitDone(SPMI_OBSRVR, channel);
    if (error) return error;
    uint16_t value = static_cast<uint16_t>(ioRd(nullptr, SPMI_OBSRVR, channel + 0x18));
    if (n && value == previous) { out = value; return 0; }
    previous = value;
  }
  return SPMI_ERR_IO;
}
static void batteryPoll() {
  if (!gBatteryEnabled) return;
  uint8_t type = 0, subtype = 0, status1 = 0, status2 = 0, usb = 0, chargerState = 0;
  uint16_t vraw = 0, iraw = 0;
  int error = read8(2, 0x4804, &type);
  if (!error) error = read8(2, 0x4805, &subtype);
  if (!error) error = read8(2, 0x4808, &status1);
  if (!error) error = read8(2, 0x4809, &status2);
  if (!error && (type != 0x0d || !(status1 & 0x80) || !(status1 & 1) || (status2 & 8))) error = SPMI_ERR_NODEV;
  if (!error) error = batteryRead16(0x48c0, vraw);
  if (!error) error = batteryRead16(0x48c6, iraw);
  if (!error) error = read8(2, 0x1310, &usb);
  if (!error) error = read8(2, 0x1006, &chargerState);
  unsigned uv = 0; int ua = 0;
  if (!error && !guamp_battery::decode(subtype, vraw, iraw, uv, ua)) error = SPMI_ERR_PARAM;
  uint64_t ns = 0;
  absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
  if (!error && !gBalance.ready) {
    uint8_t valid = 0, soc = 0, monotonic = 0;
    error = read8(2, 0xb146, &valid);
    if (!error) error = read8(2, 0xb147, &soc);
    if (!error) error = read8(2, 0x48bf, &monotonic);
    int monoPercent = (static_cast<unsigned>(monotonic) * 100 + 127) / 255;
    if (!error && (!(valid & 1) || soc > 100 || monoPercent - soc > 3 || soc - monoPercent > 3)) error = SPMI_ERR_PARAM;
    if (!error) {
      gBalance.seed(soc, 5029, ns, ua);
      IOLog("GuampBattery: initial %u%% from saved QG SOC (not a live hardware percentage); integration estimate, QG subtype %u\n", soc, subtype);
    }
  } else if (!error) {
    if (!gBalance.sample(ns, ua, guamp_battery::fullAtProfileVoltage((usb & 0x10) != 0, chargerState & 7, uv))) ++gBatterySkipped;
  }
  if (error) {
    ++gBatteryErrors;
    if (gBalance.ready) gBalance.interrupted = true;
    if (gBattery) gBattery->invalid();
    if (gBatteryErrors == 1 || gBatteryErrors % 6 == 0)
      IOLog("GuampBattery: read refused %d, QG type %02x subtype %02x status %02x/%02x; errors %u; previous level retained\n",
            error, type, subtype, status1, status2, gBatteryErrors);
    return;
  }
  unsigned percent = gBalance.percent();
  bool external = (usb & 0x10) != 0;
  if (!gBattery) {
    IOService *platform = IOService::getPlatform();
    GuampBattery *made = OSTypeAlloc(GuampBattery);
    if (!platform || !made || !made->init()) { if (made) made->release(); return; }
    made->report(percent, uv, ua, external, chargerState & 7);
    if (!made->attach(platform)) { made->release(); return; }
    made->setName("GuampBattery");
    if (!made->start(platform)) { made->detach(platform); made->release(); return; }
    made->registerService();
    gBattery = made;
    IOLog("GuampBattery: IOPMPowerSource registered; hardware QG voltage/current, estimated capacity; charger configuration unchanged\n");
  } else {
    gBattery->report(percent, uv, ua, external, chargerState & 7);
  }
  if (gBatterySamples++ % 6 == 0)
    IOLog("GuampBattery: estimated %u%% voltage %u uV current %d uA (positive=charging), external %u charger-state %u, raw %04x/%04x, errors %u gaps %u\n",
          percent, uv, ua, external, chargerState & 7, vraw, iraw, gBatteryErrors, gBatterySkipped);
}
