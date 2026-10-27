// SPDX-License-Identifier: BSD-2-Clause-Patent
// Included inside pmic, after read8/write8/ioRd/ioWr. Called only by the PMIC thread, every 5 s.
// Fuel gauge (GuampBatteryCore.h): Linux QG power-on SOC, coulomb counting with the JK50-SWD profile, OCV corrections.
// Reads: PMI632 QG 0x4800 (status, last ADC V, burst-average I, S2 averages, S7/S3 OCVs), QG SDAM 0xb100 (Linux shutdown
// SOC/OCV/time/temperature, cycle buckets, learned capacity), SMB5 status, PM6125 RTC 0x6048 (SID 0).
// Writes: none to the charger or to QG configuration. guamp-battery-temp (default 1) runs one BAT_THERM conversion per
// minute on the PMI632 ADC5 user registers 0x3142-0x3147 with the values Linux leaves there (TWRP dump: 18 80 2a 02 80).
// guamp-battery-sdam=1 (default 0) also stores SOC/OCV/temperature/RTC time in the QG SDAM like qg_store_soc_params, so
// that the next boot (Linux or this driver) can use it as shutdown SOC.
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
  void batteryData(unsigned designMah, unsigned cycles) {
    const OSSymbol *symbol = OSSymbol::withCString("BatteryData");
    OSDictionary *d = OSDictionary::withCapacity(2);
    OSNumber *design = OSNumber::withNumber(designMah, 32), *count = OSNumber::withNumber(cycles, 32);
    if (symbol && d && design && count) {
      d->setObject("DesignCapacity", design);
      d->setObject("CycleCount", count);
      setPSProperty(symbol, d);
    }
    if (symbol) symbol->release();
    if (d) d->release();
    if (design) design->release();
    if (count) count->release();
  }
  void report(const guamp_battery::Gauge &g, unsigned uv, int ua, bool external, unsigned state, bool tempValid,
              int deciC, unsigned cycles) {
    unsigned percent = static_cast<unsigned>(g.msoc);
    setBatteryInstalled(true);
    setExternalConnected(external);
    setExternalChargeCapable(external);
    setIsCharging(external && state >= 1 && state <= 4 && ua > 0);
    setCurrentCapacity(percent); setMaxCapacity(100);
    setVoltage(uv / 1000); setAmperage(g.iAvg / 1000);
    setTimeRemaining(-1);
    setCycleCount(cycles);
    setAtWarnLevel(!external && percent <= 10);
    setAtCriticalLevel(!external && percent <= 3);
    number("InstantAmperage", static_cast<uint64_t>(static_cast<int64_t>(ua / 1000)));
    // powerd (iOS 17) reads DesignCapacity, NominalChargeCapacity, AppleRawMaxCapacity, AppleRawCurrentCapacity,
    // CycleCount, Temperature and BatteryData, and derives MaximumCapacityPercent from NominalCapacity/DesignCapacity.
    number("DesignCapacity", guamp_battery::kDesignMah);
    number("NominalChargeCapacity", g.capMah);
    number("AppleRawMaxCapacity", g.capMah);
    number("AppleRawCurrentCapacity", g.chargeMah());
    unsigned health = (g.capMah * 100 + guamp_battery::kDesignMah / 2) / guamp_battery::kDesignMah;
    number("MaximumCapacityPercent", health > 100 ? 100 : health);
    batteryData(guamp_battery::kDesignMah, cycles);
    if (tempValid) number("Temperature", static_cast<uint64_t>(static_cast<int64_t>(deciC) * 10));   // 0.01 C
    flag("FullyCharged", g.full);
    flag("AppleRawExternalConnected", external);
    flag("GuampLevelEstimated", false);
    flag("GuampSampleValid", true);
    updateStatus();
  }
  void invalid() { flag("GuampSampleValid", false); updateStatus(); }
};
OSDefineMetaClassAndStructors(GuampBattery, IOPMPowerSource)
static GuampBattery *gBattery;
static guamp_battery::Gauge gGauge;
static bool gBatteryEnabled, gBatteryTempEnabled = true, gBatterySdamStore, gBatteryTempValid;
static int gBatteryTemp = 250;
static unsigned gBatteryErrors, gBatterySamples, gBatterySkipped, gBatteryCycles, gBatteryGoodOcvRaw = 0x8000;
static unsigned gBatteryTempErrors, gBatteryStoreErrors;

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
static int batteryReadBytes(unsigned sid, uint16_t address, uint8_t *out, unsigned count) {
  for (unsigned k = 0; k < count; ++k) {
    int error = read8(sid, static_cast<uint16_t>(address + k), &out[k]);
    if (error) return error;
  }
  return 0;
}
// PM6125 RTC (rtc-pm8xxx.c pm8941_regs: ctrl 0x6046 bit 7 enable, read 0x6048..0x604b): the clock of the QG SDAM time.
static int batteryRtc(uint32_t &seconds) {
  uint8_t ctrl = 0, v[4], again = 0;
  int error = read8(0, 0x6046, &ctrl);
  if (error) return error;
  if (!(ctrl & 0x80)) return SPMI_ERR_PARAM;
  for (unsigned n = 0; n < 3; ++n) {
    error = batteryReadBytes(0, 0x6048, v, 4);
    if (!error) error = read8(0, 0x6048, &again);
    if (error) return error;
    if (again == v[0]) {         // no carry between the byte reads (pm8xxx_rtc_read_time)
      seconds = v[0] | (static_cast<uint32_t>(v[1]) << 8) | (static_cast<uint32_t>(v[2]) << 16) |
                (static_cast<uint32_t>(v[3]) << 24);
      return 0;
    }
  }
  return SPMI_ERR_IO;
}
// One ADC5 conversion of ADC_BAT_THERM_PU1 (guamp CONFIG_BAT_NTC_10K: QG "batt-therm-ntc-10k"), as adc_configure and
// adc_poll_wait_eoc in qcom-spmi-adc5.c: DIG_PARAM ratiometric/decimation 2, 1 sample, 200 us settle, enable, request.
static int batteryTherm(int &deciC) {
  uint8_t p[5];
  int error = batteryReadBytes(2, 0x3142, p, 5);
  if (error) return error;
  p[0] = static_cast<uint8_t>((p[0] & ~0x7cu) | 0x18u);
  p[1] = static_cast<uint8_t>(p[1] & ~0x07u);
  p[2] = 0x2a;
  p[3] = static_cast<uint8_t>((p[3] & ~0x0fu) | 0x02u);
  p[4] = static_cast<uint8_t>(p[4] | 0x80u);
  for (unsigned k = 0; k < 5 && !error; ++k) error = write8(2, static_cast<uint16_t>(0x3142 + k), p[k]);
  uint8_t channel = 0;
  if (!error) error = read8(2, 0x3144, &channel);
  if (!error && channel != 0x2a) {       // adc_channel_check (PMI632): one more write, then give up
    error = write8(2, 0x3144, 0x2a);
    if (!error) error = read8(2, 0x3144, &channel);
    if (!error && channel != 0x2a) error = SPMI_ERR_IO;
  }
  if (!error) error = write8(2, 0x3147, 0x80);
  if (error) return error;
  uint8_t status = 0;
  unsigned n = 0;
  for (; n < 40; ++n) {
    IOSleep(1);
    error = read8(2, 0x3108, &status);
    if (error) return error;
    if ((status & 3u) == 1u) break;
  }
  if (n == 40) return SPMI_ERR_TIMEOUT;
  uint8_t lo = 0, hi = 0;
  error = read8(2, 0x3150, &lo);
  if (!error) error = read8(2, 0x3151, &hi);
  if (error) return error;
  return guamp_battery::thermDeciC(lo | (static_cast<unsigned>(hi) << 8), deciC) ? 0 : SPMI_ERR_PARAM;
}
// qg_store_soc_params subset (opt-in): SOC, temperature, OCV, RTC time, then VALID.
static int batteryStore(const guamp_battery::Gauge &g) {
  uint32_t now = 0;
  int error = batteryRtc(now);
  uint8_t b[] = {static_cast<uint8_t>(g.msoc),
                 static_cast<uint8_t>(gBatteryTemp & 0xff), static_cast<uint8_t>((gBatteryTemp >> 8) & 0xff)};
  for (unsigned k = 0; k < 3 && !error; ++k) error = write8(2, static_cast<uint16_t>(0xb147 + k), b[k]);
  for (unsigned k = 0; k < 4 && !error; ++k)
    error = write8(2, static_cast<uint16_t>(0xb14c + k), static_cast<uint8_t>(g.lastOcvUv >> (8 * k)));
  for (unsigned k = 0; k < 4 && !error; ++k)
    error = write8(2, static_cast<uint16_t>(0xb154 + k), static_cast<uint8_t>(now >> (8 * k)));
  if (!error) error = write8(2, 0xb146, 1);
  return error;
}

static void batteryPoll() {
  if (!gBatteryEnabled) return;
  uint8_t type = 0, subtype = 0, status1 = 0, status2 = 0, usb = 0, chargerState = 0;
  uint16_t vraw = 0, iraw = 0, vavgRaw = 0x8000, iavgRaw = 0x8000;
  int error = read8(2, 0x4804, &type);
  if (!error) error = read8(2, 0x4805, &subtype);
  if (!error) error = read8(2, 0x4808, &status1);
  if (!error) error = read8(2, 0x4809, &status2);
  if (!error && (type != 0x0d || !(status1 & 0x80) || !(status1 & 1) || (status2 & 8))) error = SPMI_ERR_NODEV;
  if (!error) error = batteryRead16(0x48c0, vraw);     // QG_LAST_ADC_V
  if (!error) error = batteryRead16(0x48c6, iraw);     // QG_LAST_BURST_AVG_I
  if (!error) error = read8(2, 0x1310, &usb);
  if (!error) error = read8(2, 0x1006, &chargerState);
  unsigned uv = 0; int ua = 0;
  if (!error && !guamp_battery::decode(subtype, vraw, iraw, uv, ua)) error = SPMI_ERR_PARAM;
  if (!error) { batteryRead16(0x4880, vavgRaw); batteryRead16(0x4882, iavgRaw); }   // S2 FIFO averages, logged only
  if (error) {
    ++gBatteryErrors;
    if (gGauge.ready) gGauge.interrupted = true;
    if (gBattery) gBattery->invalid();
    if (gBatteryErrors == 1 || gBatteryErrors % 6 == 0)
      IOLog("GuampBattery: read refused %d, QG type %02x subtype %02x status %02x/%02x; errors %u; previous level retained\n",
            error, type, subtype, status1, status2, gBatteryErrors);
    return;
  }
  if (gBatteryTempEnabled && (gBatterySamples % 12 == 0 || !gBatteryTempValid)) {
    int t = 0;
    int terror = batteryTherm(t);
    if (!terror) { gBatteryTemp = t; gBatteryTempValid = true; }
    else if (gBatteryTempErrors++ % 10 == 0) IOLog("GuampBattery: BAT_THERM conversion refused %d\n", terror);
  }
  bool external = (usb & 0x10) != 0;
  guamp_battery::Sample s;
  absolutetime_to_nanoseconds(mach_absolute_time(), &s.ns);
  s.uv = uv; s.ua = ua; s.external = external; s.chargerState = chargerState & 7; s.temp = gBatteryTemp;
  unsigned events = 0;
  if (!gGauge.ready) {
    guamp_battery::PonInput in;
    uint16_t s7 = 0x8000, s3 = 0x8000, sdamPon = 0;
    batteryRead16(0x4870, s7);                         // QG_S7_PON_OCV_V
    batteryRead16(0x48cc, s3);                         // QG_LAST_S3_SLEEP_V
    batteryRead16(0xb1bc, sdamPon);                    // QG_SDAM_PON_OCV (PBS)
    uint16_t good = 0x8000;
    batteryRead16(0x4874, good);                       // QG_S3_GOOD_OCV_V: later changes are new rest OCVs
    gBatteryGoodOcvRaw = good;
    in.s7Raw = s7; in.s3LastRaw = s3; in.sdamPonRaw = sdamPon;
    in.sdamOk = batteryReadBytes(2, 0xb146, in.sdam.b, sizeof(in.sdam.b)) == 0;
    in.rtcOk = batteryRtc(in.rtc) == 0;
    in.temp = gBatteryTempValid ? gBatteryTemp : (in.sdamOk ? in.sdam.temp() : 250);
    in.uv = uv; in.ua = ua;
    guamp_battery::PonResult pon = guamp_battery::determinePonSoc(in);
    unsigned learned = in.sdamOk ? in.sdam.learnedMah() : 0;
    unsigned mah = learned >= guamp_battery::kDesignMah / 2 && learned <= guamp_battery::kDesignMah * 11 / 10
                       ? learned : guamp_battery::nominalMah(in.temp, false);
    gBatteryCycles = in.sdamOk ? in.sdam.cycles() : 0;
    gGauge.seed(pon, mah, s);
    IOLog("GuampBattery: PON soc %d%% from %s ocv %u uV (profile %d%%, full %d%% cutoff %d%%, live %d%%), temp %d dC %s; "
          "SDAM valid %u soc %u time %u rtc %u%s; capacity %u mAh (learned %u), cycles %u, profile %s\n",
          pon.soc, guamp_battery::ponName(pon.source), pon.ocvUv, pon.ponSoc, pon.scale.fullSoc, pon.scale.cutoffSoc,
          pon.liveSoc, in.temp, gBatteryTempValid ? "adc" : "assumed", in.sdamOk ? in.sdam.valid() : 0,
          in.sdamOk ? in.sdam.soc() : 0, in.sdamOk ? in.sdam.time() : 0, in.rtc, in.rtcOk ? "" : " (no rtc)", mah,
          learned, gBatteryCycles, guamp_profile::kBatteryType);
  } else {
    events = gGauge.sample(s);
    if (events & guamp_battery::kGap) ++gBatterySkipped;
    if (gBatterySamples % 12 == 0) {                   // QG S3 good OCV: new value = new rest measurement
      uint16_t good = 0x8000;
      if (!batteryRead16(0x4874, good) && good != gBatteryGoodOcvRaw && good != 0x8000 && !external) {
        gGauge.goodOcv(guamp_battery::vrawToUv(good), s.temp);
        events |= guamp_battery::kRestOcv;
      }
      if (good != 0x8000) gBatteryGoodOcvRaw = good;
    }
    if (events & (guamp_battery::kFull | guamp_battery::kEmpty | guamp_battery::kRestOcv))
      IOLog("GuampBattery: %s%s%s at %u uV, sys %d.%02d%%\n", events & guamp_battery::kFull ? "charge full " : "",
            events & guamp_battery::kEmpty ? "cutoff reached " : "", events & guamp_battery::kRestOcv ? "rest OCV " : "",
            gGauge.vAvg, gGauge.centi() / 100, gGauge.centi() % 100);
  }
  if (gBatterySdamStore && ((events & guamp_battery::kStep) || gBatterySamples % 60 == 0)) {
    int serror = batteryStore(gGauge);
    if (serror && gBatteryStoreErrors++ % 10 == 0) IOLog("GuampBattery: SDAM store refused %d\n", serror);
  }
  if (!gBattery) {
    IOService *platform = IOService::getPlatform();
    GuampBattery *made = OSTypeAlloc(GuampBattery);
    if (!platform || !made || !made->init()) { if (made) made->release(); return; }
    made->report(gGauge, uv, ua, external, chargerState & 7, gBatteryTempValid, gBatteryTemp, gBatteryCycles);
    if (!made->attach(platform)) { made->release(); return; }
    made->setName("GuampBattery");
    if (!made->start(platform)) { made->detach(platform); made->release(); return; }
    made->registerService();
    gBattery = made;
    IOLog("GuampBattery: IOPMPowerSource registered; QG fuel gauge, profile %s; charger configuration unchanged\n",
          guamp_profile::kBatteryType);
  } else {
    gBattery->report(gGauge, uv, ua, external, chargerState & 7, gBatteryTempValid, gBatteryTemp, gBatteryCycles);
  }
  if (gBatterySamples++ % 6 == 0)
    IOLog("GuampBattery: soc %d%% sys %d.%02d%% voltage %u uV avg %u current %d uA avg %d (positive=charging), "
          "temp %d dC%s, charge %u/%u mAh, cycles %u, external %u charger-state %u full %u, s2 %04x/%04x, errors %u gaps %u\n",
          gGauge.msoc, gGauge.centi() / 100, gGauge.centi() % 100, uv, gGauge.vAvg, ua, gGauge.iAvg, gBatteryTemp,
          gBatteryTempValid ? "" : " (assumed)", gGauge.chargeMah(), gGauge.capMah, gBatteryCycles, external,
          chargerState & 7, gGauge.full, vavgRaw, iavgRaw, gBatteryErrors, gBatterySkipped);
}
