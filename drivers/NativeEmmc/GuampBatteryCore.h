// SPDX-License-Identifier: BSD-2-Clause-Patent
// Fuel gauge for the guamp PMI632 QG, modelled on the Linux kernel QG driver (guamp kernel c65462bf, drivers/power/
// supply/qcom: qpnp-qg.c qg_determine_pon_soc, qg-soc.c qg_adjust_sys_soc/update_msoc, qg-profile-lib.c lookups) and the
// JK50-SWD profile (GuampBatteryProfile.h, generated). The Linux running SOC comes from a closed userspace daemon; here
// it is coulomb counting with the profile capacity plus OCV corrections. Integer math only, no libc, no hardware access.
#ifndef GUAMP_BATTERY_CORE_H
#define GUAMP_BATTERY_CORE_H
#include <stdint.h>
#include "GuampBatteryProfile.h"
namespace guamp_battery {
namespace prof = guamp_profile;

// bengal-guamp-common-overlay.dtsi &pmi632_qg: qcom,vbatt-cutoff-mv 3200, qcom,vbatt-empty-mv 3000, qcom,qg-iterm-ma
// 350, qcom,ignore-shutdown-soc-secs 1800. qpnp-qg.c: DEFAULT_SHUTDOWN_TEMP_DIFF 60, S7_ERROR_MARGIN_UV 20000,
// VBAT_LOW_HYST_UV 50000, DEFAULT_RECHARGE_SOC 95; qg-soc.c qg_delta_soc_interval_ms 20000.
constexpr unsigned kCutoffUv = 3200000, kEmptyUv = 3000000, kVbatLowHystUv = 50000, kS7ErrorMarginUv = 20000;
constexpr unsigned kIgnoreShutdownSocSecs = 1800, kRechargeSoc = 95;
constexpr int kShutdownTempDiff = 60;
constexpr uint64_t kDeltaSocIntervalNs = 20000000000ULL;
constexpr unsigned kRawReset = 0x8000;            // FIFO_V_RESET_VAL: register never written since reset
// Battery + connector resistance used to estimate OCV under load: TWRP power_supply bms (batteria/letture-recovery.json)
// voltage_now 4357533 uV, voltage_ocv 4284933 uV at 482941 uA -> 150 mOhm.
constexpr int kRbattMohm = 150;
constexpr unsigned kDesignMah = 5029;             // qg_get_nominal_capacity(250, charging=true): fcc1 @ 25 C

inline unsigned vrawToUv(unsigned raw) { return static_cast<unsigned>((194637ULL * raw) / 1000); }   // V_RAW_TO_UV

inline bool decode(unsigned subtype, uint16_t voltage, uint16_t current, unsigned &uv, int &appleUa) {
  if ((subtype != 3 && subtype != 4) || voltage == kRawReset || current == kRawReset) return false;
  uv = vrawToUv(voltage);
  if (uv < 2800000 || uv > 4600000) return false;
  int signedRaw = current >= 0x8000 ? static_cast<int>(current) - 65536 : current;
  // QG is negative while charging. IOPMPowerSource uses positive charging current.
  appleUa = -static_cast<int>((static_cast<int64_t>(signedRaw) * (subtype == 3 ? 152588 : 305176)) / 1000);
  return true;
}

inline int absInt(int v) { return v < 0 ? -v : v; }
inline int capInt(int lo, int hi, int v) { return v < lo ? lo : (v > hi ? hi : v); }

// qg-profile-lib.c qg_linear_interpolate
inline int linear(int y0, int x0, int y1, int x1, int x) {
  if (y0 == y1 || x == x0) return y0;
  if (x1 == x0 || x == x1) return y1;
  return y0 + ((y1 - y0) * (x - x0) / (x1 - x0));
}
inline bool between(int left, int right, int value) {    // qg-defs.h is_between
  return (left >= right && left >= value && value >= right) || (left <= right && left <= value && value <= right);
}

// qg-profile-lib.c qg_interpolate_soc: OCV (100 uV units) and temperature (0.1 C) -> SOC in 0.01 %
inline int interpolateSoc(const uint16_t *data, const uint16_t *socRow, const int8_t *temps, unsigned rows,
                          unsigned cols, int temp, int ocv) {
  auto at = [&](unsigned r, unsigned c) { return static_cast<int>(data[r * cols + c]); };
  int rowsI = static_cast<int>(rows);
  if (temp < temps[0] * 10) temp = temps[0] * 10;
  if (temp > temps[cols - 1] * 10) temp = temps[cols - 1] * 10;
  unsigned j = 0;
  while (j < cols && temp > temps[j] * 10) j = j + 1;
  if (temp == temps[j] * 10) {
    if (ocv >= at(0, j)) return socRow[0];
    if (ocv <= at(rows - 1, j)) return socRow[rows - 1];
    for (unsigned i = 1; i < rows; i = i + 1)
      if (ocv >= at(i, j))
        return ocv == at(i, j) ? socRow[i] : linear(socRow[i], at(i, j), socRow[i - 1], at(i - 1, j), ocv);
  }
  if (ocv >= at(0, j)) return socRow[0];
  if (ocv <= at(rows - 1, j - 1)) return socRow[rows - 1];
  int high = 0, low = 0;
  for (int i = 0; i < rowsI - 1; i = i + 1) {
    unsigned u = static_cast<unsigned>(i);
    if (high == 0 && between(at(u, j), at(u + 1, j), ocv))
      high = linear(socRow[u], at(u, j), socRow[u + 1], at(u + 1, j), ocv);
    if (low == 0 && between(at(u, j - 1), at(u + 1, j - 1), ocv))
      low = linear(socRow[u], at(u, j - 1), socRow[u + 1], at(u + 1, j - 1), ocv);
    if (high && low) return linear(low, temps[j - 1] * 10, high, temps[j] * 10, temp);
  }
  if (high) return high;
  if (low) return low;
  return 10000;
}
// qg-battery-profile.c lookup_soc_ocv without the final rounding: 0.01 %. charging selects pc-temp-v1, else v2.
inline int lookupSocCenti(unsigned ocvUv, int temp, bool charging) {
  int ocv = static_cast<int>(ocvUv / 100);
  int soc = charging ? interpolateSoc(prof::kOcv1, prof::kOcv1Soc, prof::kOcv1Temp, prof::kOcv1Rows, prof::kOcv1Cols, temp, ocv)
                     : interpolateSoc(prof::kOcv2, prof::kOcv2Soc, prof::kOcv2Temp, prof::kOcv2Rows, prof::kOcv2Cols, temp, ocv);
  return capInt(0, 10000, soc);
}
inline int lookupSoc(unsigned ocvUv, int temp, bool charging) {     // lookup_soc_ocv: percent, DIV_ROUND_CLOSEST
  return capInt(0, 100, (lookupSocCenti(ocvUv, temp, charging) + 50) / 100);
}
// qg-profile-lib.c qg_interpolate_single_row_lut (DEGC_SCALE 10) and qg_get_nominal_capacity: mAh
inline unsigned nominalMah(int temp, bool charging) {
  const int8_t *t = charging ? prof::kFcc1Temp : prof::kFcc2Temp;
  const uint16_t *d = charging ? prof::kFcc1 : prof::kFcc2;
  unsigned cols = charging ? sizeof(prof::kFcc1) / sizeof(prof::kFcc1[0]) : sizeof(prof::kFcc2) / sizeof(prof::kFcc2[0]);
  int r;
  if (temp < t[0] * 10) r = d[0];
  else if (temp > t[cols - 1] * 10) r = d[cols - 1];
  else {
    unsigned i = 0;
    while (i < cols && temp > t[i] * 10) i = i + 1;
    r = temp == t[i] * 10 ? d[i] : linear(d[i - 1], t[i - 1] * 10, d[i], t[i] * 10, temp);
  }
  return static_cast<unsigned>(capInt(100, 16000, r));
}

// Battery-relative SOC (profile) -> system SOC as qg_determine_pon_soc: 0 % at the cutoff voltage, 100 % at the
// profile float voltage (full_soc capped at 99 there).
struct Scale {
  int fullSoc = 99, cutoffSoc = 0;
  void at(int temp) {
    fullSoc = capInt(0, 99, lookupSoc(prof::kMaxVoltageUv, temp, true));
    cutoffSoc = lookupSoc(kCutoffUv, temp, false);
  }
  bool valid() const { return fullSoc > cutoffSoc; }
  int ponPercent(int ponSoc) const {           // DIV_ROUND_UP as Linux
    if (!valid() || ponSoc <= cutoffSoc) return ponSoc;
    return capInt(0, 100, ((ponSoc - cutoffSoc) * 100 + (fullSoc - cutoffSoc) - 1) / (fullSoc - cutoffSoc));
  }
  int centiFromOcv(unsigned ocvUv, int temp) const {   // 0.01 % system SOC from an OCV
    int c = lookupSocCenti(ocvUv, temp, false);
    if (!valid()) return c;
    return capInt(0, 10000, (c - cutoffSoc * 100) * 100 / (fullSoc - cutoffSoc));
  }
};

// ADC5 BAT_THERM_PU1 (30k pull-up, NTC 10k) code -> 0.1 C: qcom_vadc_scale_hw_calib_batt_therm_30 with
// adc_vdd_ref 1875 mV and full_scale_code_volt 0x70e4 (data_pmic5_lite), then qcom_vadc_map_voltage_temp.
inline bool thermDeciC(unsigned code, int &deciC) {
  if (code == 0x8000) return false;                // ADC_USR_DATA_CHECK
  if (code > 0x7fff) code = 0;
  int mv = static_cast<int>((static_cast<uint64_t>(code) * 1875) / 0x70e4);
  unsigned i = 0;
  while (i < prof::kThermPoints && prof::kThermMv[i] >= mv) i = i + 1;
  if (i == 0) deciC = prof::kThermDeciC[0];
  else if (i == prof::kThermPoints) deciC = prof::kThermDeciC[prof::kThermPoints - 1];
  else deciC = (prof::kThermDeciC[i] - prof::kThermDeciC[i - 1]) * (mv - prof::kThermMv[i - 1]) /
               (prof::kThermMv[i] - prof::kThermMv[i - 1]) + prof::kThermDeciC[i - 1];
  return deciC > -400 && deciC < 900;              // ends of the map: open or shorted thermistor
}

// QG SDAM (qg-reg.h QG_SDAM_*_OFFSET, base 0xb100), bytes 0x46..0x69 as read.
struct Sdam {
  uint8_t b[0x6a - 0x46];
  unsigned u8(unsigned off) const { return b[off - 0x46]; }
  unsigned u16(unsigned off) const { return b[off - 0x46] | (static_cast<unsigned>(b[off - 0x45]) << 8); }
  uint32_t u32(unsigned off) const { return u16(off) | (static_cast<uint32_t>(u16(off + 2)) << 16); }
  unsigned valid() const { return u8(0x46); }
  unsigned soc() const { return u8(0x47); }
  int temp() const { unsigned t = u16(0x48); return t >= 0x8000 ? static_cast<int>(t) - 65536 : static_cast<int>(t); }
  uint32_t ocvUv() const { return u32(0x4c); }
  uint32_t time() const { return u32(0x54); }
  unsigned cycles() const {                        // fg-alg.c get_cycle_count: mean of the 8 SOC buckets
    unsigned sum = 0;
    for (unsigned k = 0; k < 8; k = k + 1) sum = sum + u16(0x58 + 2 * k);
    return sum / 8;
  }
  unsigned learnedMah() const { unsigned v = u16(0x68); return v >= 0x8000 ? 0 : v; }
};

// Battery health as iOS 17 reads it. powerd computes "Maximum Capacity Percent" = ceil(NominalChargeCapacity * 100 /
// DesignCapacity) (accepted 1..150) and Settings > Battery > Battery Health shows that number. Design capacity: the
// profile FCC at 25 C (kDesignMah, the gauge's 100 % reference for a new JK50-SWD cell, rated 4850 / typical 5000 mAh).
// Nominal capacity: the capacity the Linux QG learned (QG SDAM 0x68, written by Linux capacity learning), capped at the
// design value so a cell measured above the profile reads 100 %, never more. A value outside 50..110 % of the design
// is not a learning result (SDAM never written or corrupted): the capacity is unknown and the design value is reported,
// i.e. 100 % (owner request 28/10), flagged as not learned in the log and in the GuampCapacityLearned property.
struct Health {
  unsigned nominalMah = kDesignMah, percent = 100;
  bool learned = false;
};
inline bool plausibleLearned(unsigned mah) { return mah >= kDesignMah / 2 && mah <= kDesignMah * 11 / 10; }
inline Health health(unsigned learnedMah) {
  Health h;
  if (plausibleLearned(learnedMah)) {
    h.learned = true;
    h.nominalMah = learnedMah > kDesignMah ? kDesignMah : learnedMah;
  }
  h.percent = (h.nominalMah * 100 + kDesignMah - 1) / kDesignMah;     // powerd rounds up (fcvtps)
  return h;
}
// "Serial" for powerd: iOS 17 powerd keeps its battery health record per battery serial and sets the record to
// "unknown" (service state -1, Settings spins forever) when the power source has no Serial. The JK50 pack has no
// readable serial number (only the batt-id resistor that selects the profile), so this is a fixed model identifier,
// not a unit serial: 17 characters like Apple's (powerd accepts 16..18), no digits where Apple keeps the
// year/week of manufacture (characters 3..5), so no manufacture date is derived from it.
static const char kBatterySerial[] = "GUAMP-JK50SWD-QG1";
// "WeightedRa" (BatteryData) is compared by powerd/thermalmonitord with the age-aware P0 threshold (default 500). The QG
// keeps no resistance table; the value is the battery + connector resistance measured on this phone (kRbattMohm,
// TWRP voltage_now/voltage_ocv at 483 mA), the same constant the gauge uses for its OCV estimate.
constexpr unsigned kWeightedRa = static_cast<unsigned>(kRbattMohm);

enum PonSource { kShutdownSoc, kS7Pon, kS3Last, kSdamPon, kLoadedVoltage };
inline const char *ponName(PonSource s) {
  return s == kShutdownSoc ? "SHUTDOWN_SOC" : s == kS7Pon ? "S7_PON_SOC" : s == kS3Last ? "S3_LAST_SOC"
       : s == kSdamPon ? "SDAM_PON_SOC" : "LOADED_VOLTAGE";
}
struct PonInput {
  unsigned s7Raw = kRawReset, s3LastRaw = kRawReset, sdamPonRaw = 0;   // QG 0x70, 0xcc; SDAM 0xbc
  bool sdamOk = false, rtcOk = false;
  Sdam sdam{};
  uint32_t rtc = 0;
  int temp = 250;
  unsigned uv = 0; int ua = 0;                                         // first live sample (stale-OCV guard)
};
struct PonResult { int soc = 0, ponSoc = 0; unsigned ocvUv = 0; PonSource source = kS7Pon; Scale scale; int liveSoc = 0; };

// Estimate of the open-circuit voltage from a loaded sample (positive ua = charging).
inline unsigned loadedOcvUv(unsigned uv, int ua) {
  int64_t ocv = static_cast<int64_t>(uv) - static_cast<int64_t>(ua) * kRbattMohm / 1000;
  return ocv < 0 ? 0 : static_cast<unsigned>(ocv);
}

// qpnp-qg.c qg_determine_pon_soc, PMI632 path (QG_PON_OCV_WA: qcom,qg-use-s7-ocv absent in the guamp DT).
// Added guard: the S7 PON OCV is measured at the last PMIC power-on, which can be hours old after warm reboots without
// Linux, and the shutdown SOC can be stale after a reboot within 30 min without Linux; a result more than 15 % away
// from the live loaded-voltage estimate is replaced by that estimate.
inline PonResult determinePonSoc(const PonInput &in) {
  PonResult r;
  r.scale.at(in.temp);
  unsigned s7 = vrawToUv(in.s7Raw), s3 = vrawToUv(in.s3LastRaw), sdamPon = vrawToUv(in.sdamPonRaw);
  bool shutdownValid = in.sdamOk && in.sdam.valid();
  bool useShutdown = shutdownValid && in.rtcOk;
  if (useShutdown) {
    int64_t age = static_cast<int64_t>(in.rtc) - static_cast<int64_t>(in.sdam.time());
    if (age < 0 || age > kIgnoreShutdownSocSecs) useShutdown = false;
    int st = in.sdam.temp();
    if (!between(0, kShutdownTempDiff, absInt(st - in.temp)) && (st < 0 || in.temp < 0)) useShutdown = false;
    if (in.sdam.soc() > 100) useShutdown = false;
  }
  if (useShutdown) {
    r.source = kShutdownSoc; r.ocvUv = in.sdam.ocvUv(); r.soc = static_cast<int>(in.sdam.soc());
    r.ponSoc = lookupSoc(r.ocvUv, in.temp, false);
  } else {
    if (in.s3LastRaw == kRawReset) {
      if (!sdamPon || sdamPon <= s7) { r.source = kS7Pon; r.ocvUv = s7; }
      else if (!shutdownValid && sdamPon - s7 > kS7ErrorMarginUv) { r.source = kS7Pon; r.ocvUv = s7; }
      else { r.source = kSdamPon; r.ocvUv = sdamPon; }
    } else if (s3 >= s7) { r.source = kS3Last; r.ocvUv = s3; }
    else { r.source = kS7Pon; r.ocvUv = s7; }
    if (r.ocvUv < 2000000) r.ocvUv = 2000000;      // CAP(QG_MIN_OCV_UV, QG_MAX_OCV_UV)
    if (r.ocvUv > 5000000) r.ocvUv = 5000000;
    r.ponSoc = lookupSoc(r.ocvUv, in.temp, false);
    r.soc = r.scale.ponPercent(r.ponSoc);
  }
  // An OCV register still at its reset value (never measured) is not an OCV.
  bool unmeasured = (r.source == kS7Pon && (in.s7Raw == kRawReset || !in.s7Raw)) ||
                    (r.source == kSdamPon && in.sdamPonRaw == kRawReset);
  if (in.uv) {
    unsigned live = loadedOcvUv(in.uv, in.ua);
    r.liveSoc = (r.scale.centiFromOcv(live, in.temp) + 50) / 100;
    if (unmeasured || absInt(r.liveSoc - r.soc) > 15) {
      r.source = kLoadedVoltage; r.ocvUv = live; r.soc = r.liveSoc;
    }
  }
  return r;
}

// Running gauge: system SOC in 1e-7 of full (kScale = 100 %), the displayed msoc moved 1 % per step as Linux does.
struct Sample {
  uint64_t ns = 0;
  unsigned uv = 0;           // QG last ADC voltage
  int ua = 0;                // QG burst-average current, positive = charging
  bool external = false;
  unsigned chargerState = 0; // SMB5 BATTERY_CHARGER_STATUS_1 [2:0]: 1..4 charging, 5 TERMINATE, 6 PAUSE, 7 DISABLE
  int temp = 250;
};
enum : unsigned { kIntegrated = 1, kGap = 2, kFull = 4, kEmpty = 8, kRestOcv = 16, kLowCorrected = 32, kStep = 64 };

struct Gauge {
  static constexpr int64_t kScale = 10000000;
  int64_t soc = 0, capUams = 0;
  unsigned capMah = 0, vAvg = 0, restPolls = 0, cutoffPolls = 0, lastOcvUv = 0;
  int iAvg = 0, lastUa = 0, msoc = 0, lastAdj = 0;
  uint64_t lastNs = 0, lastStepNs = 0;
  bool ready = false, interrupted = false, full = false;
  Scale scale;

  int centi() const { return static_cast<int>(soc / 1000); }
  unsigned chargeMah() const { return static_cast<unsigned>(static_cast<int64_t>(capMah) * soc / kScale); }
  void setCenti(int c) { soc = static_cast<int64_t>(capInt(0, 10000, c)) * 1000; }

  void seed(const PonResult &pon, unsigned mah, const Sample &s) {
    scale = pon.scale;
    capMah = mah;
    // Usable charge between the cutoff and the float voltage, the span of the system SOC.
    int span = scale.valid() ? scale.fullSoc - scale.cutoffSoc : 100;
    capUams = static_cast<int64_t>(mah) * 3600000000LL * span / 100;
    setCenti(pon.soc * 100);
    msoc = lastAdj = capInt(0, 100, pon.soc);
    lastOcvUv = pon.ocvUv;
    lastNs = lastStepNs = s.ns; lastUa = iAvg = s.ua; vAvg = s.uv;
    ready = capUams > 0; interrupted = false; full = false; restPolls = cutoffPolls = 0;
  }
  // QG S3 good OCV, or our own rest detection: Linux hands it to the daemon, which restarts from that OCV.
  void goodOcv(unsigned ocvUv, int temp) { lastOcvUv = ocvUv; setCenti(scale.centiFromOcv(ocvUv, temp)); }

  // qg-soc.c qg_adjust_sys_soc
  int adjusted() {
    int c = centi(), a;
    if (c >= 10000) a = 100;
    else if (c >= 9900) a = lastAdj == 100 ? 100 : 99;
    else a = (c + 50) / 100;
    if (a == 0 && vAvg >= kCutoffUv + kVbatLowHystUv) a = 1;
    lastAdj = a;
    return a;
  }

  unsigned sample(const Sample &s) {
    if (!ready) return 0;
    unsigned ev = 0;
    if (!interrupted && s.ns >= lastNs) {
      uint64_t ms = (s.ns - lastNs) / 1000000;
      if (ms <= 15000) {      // never invent consumption through a missing/stalled observation interval
        int64_t dq = (static_cast<int64_t>(lastUa) + s.ua) * static_cast<int64_t>(ms) / 2;   // uA*ms
        soc = soc + dq * kScale / capUams;
        ev |= kIntegrated;
      } else ev |= kGap;
    } else ev |= kGap;
    lastNs = s.ns; lastUa = s.ua; interrupted = false;
    vAvg = (3 * vAvg + s.uv) / 4;
    iAvg = (3 * iAvg + s.ua) / 4;
    bool charging = s.external && s.chargerState >= 1 && s.chargerState <= 4;
    bool discharging = !charging && iAvg < 0;

    // Full (qpnp-qg.c qg_charge_full_update): charge_done with msoc >= 99, or termination at the profile CC/CV
    // threshold. A termination below it (conservative charger float voltage) is not a full battery.
    if (!full && s.external && s.chargerState == 5 && (msoc >= 99 || vAvg >= prof::kCcCvThresholdUv)) {
      full = true; soc = kScale; ev |= kFull;
    } else if (full && (!s.external || msoc < static_cast<int>(kRechargeSoc))) {
      full = false;
    }
    if (!full && soc > kScale - 1000) soc = kScale - 1000;   // 99.99 % until the charger reports done
    if (soc < 0) soc = 0;
    if (soc > kScale) soc = kScale;

    // Empty: below the cutoff voltage on discharge for three polls, or below the empty voltage at once.
    if (discharging && vAvg < kCutoffUv) cutoffPolls = cutoffPolls + 1; else cutoffPolls = 0;
    if (discharging && (cutoffPolls >= 3 || vAvg < kEmptyUv)) { soc = 0; ev |= kEmpty; }

    // Rest: |I| < 20 mA without input for 10 min (QG S3 entry is 15 mA): the voltage is the OCV.
    if (!s.external && absInt(s.ua) < 20000) restPolls = restPolls + 1; else restPolls = 0;
    if (restPolls == 120) { goodOcv(vAvg, s.temp); ev |= kRestOcv; }

    // Low end (steep OCV region): on discharge pull the count down towards the loaded-voltage OCV estimate, at most
    // 0.05 % per sample, when it is more than 3 % above it.
    if (discharging && !(ev & kEmpty)) {
      int v = scale.centiFromOcv(loadedOcvUv(vAvg, iAvg), s.temp);
      int c = centi();
      if (v < 1500 && c > v + 300) {
        int step = c - v - 300 < 5 ? c - v - 300 : 5;
        soc = soc - static_cast<int64_t>(step) * 1000;
        ev |= kLowCorrected;
      }
    }

    // qg-soc.c is_scaling_required/update_msoc: 1 % per step, at most every 20 s, up only with input present.
    int target = 100;
    if (full) lastAdj = 100; else target = adjusted();
    if (s.ns - lastStepNs >= kDeltaSocIntervalNs) {
      if (target < msoc) { msoc = msoc - 1; lastStepNs = s.ns; ev |= kStep; }
      else if (target > msoc && s.external && (charging || full || msoc == 0)) {
        msoc = msoc + 1; lastStepNs = s.ns; ev |= kStep;
      }
    }
    return ev;
  }
};
}  // namespace guamp_battery
#endif
