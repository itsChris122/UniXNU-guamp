// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp PMI632 (SMB5) charger input controller, the part of Linux smb5-lib.c/qpnp-smb5.c this port needs so that
// charging comes back after an unplug/replug: plug-in/unplug edges (smblib_usb_plugin_locked, typec_src_removal),
// BC1.2 source detection (APSD, smblib_apsd_enable/smblib_rerun_apsd), the Type-C sink role (smblib_force_dr_mode),
// the input current limit (smblib_set_icl_current, high-current mode), the weak-adapter AICL work-around of
// usbin_uv_irq_handler (WEAK_ADAPTER_WA is set for PMI632), input suspend recovery, and a Type-C recovery for a
// source that shows Rp on CC but never turns VBUS on (C-to-C replug on a PC): CC terminations off, then on again.
// No hardware access of its own: every register access goes through Bus, so the whole sequence also runs on the host
// (batteria/test-replug.cpp, simulated PMI632). Bus provides
//   int rd(uint16_t addr, uint8_t *v); int wr(uint16_t addr, uint8_t v);   0 = success (PMI632, SPMI slave 2)
//   void sleepMs(unsigned ms); void log(const char *format, ...)            one complete line per call
//   int vbusMv(unsigned *mv)                                                 USBIN voltage (ADC5 USB_IN_V_16)
// Limits: float voltage kFvMv (4.35 V, DT qcom,fv-max-uv 4.40 V), ICL <= 3000 mA and FCC <= 3000 mA (DT usb-icl-ua,
// fcc-max-ua); an SDP or unknown source gets at most 500 mA, only APSD OCP/CDP/DCP the requested ICL.
#ifndef GUAMP_CHARGE_REPLUG_CORE_H
#define GUAMP_CHARGE_REPLUG_CORE_H
#include <stdint.h>

namespace guamp_charge {
namespace reg {   // smb5-reg.h offsets in the PMI632 0x1000 charger blocks
constexpr uint16_t kStatus1 = 0x1006, kStatus2 = 0x1007, kStatus7 = 0x100d, kChgEnable = 0x1042, kChgPause = 0x1043,
                   kFcc = 0x1061, kFv = 0x1070, kJeitaEn = 0x1090, kJeitaThresholds = 0x1094,
                   kIclStatus = 0x1107, kAiclStatus = 0x110a, kPowerPath = 0x110b, kOtgCmd = 0x1140, kOtgCfg = 0x1153,
                   kBatifPullUp = 0x1286,
                   kApsdStatus = 0x1307, kApsdResult = 0x1308, kUsbinRt = 0x1310, kUsbinCmdIl = 0x1340, kCmdApsd = 0x1341,
                   kCmdIclOverride = 0x1342, kTypeCCfg = 0x1358, kUsbinOptions1 = 0x1362, kUsbinLoadCfg = 0x1365,
                   kIclOptions = 0x1366, kIclCfg = 0x1370, kAiclOptions = 0x1380, kAicl5v = 0x1381, kAiclCont = 0x1384,
                   kTypecSnkStatus = 0x1506, kTypecSrcStatus = 0x1508, kTypecStateMachine = 0x1509,
                   kTypecStatus0a = 0x150a, kTypecMisc = 0x150b, kTypecLegacy = 0x150d, kTypecStatus0e = 0x150e,
                   kTypecRt = 0x1510, kTypecModeCfg = 0x1544, kTypecCcoutControl = 0x1548,
                   kTypecDebugAccessSink = 0x154a, kTypecExitStateCfg = 0x1550, kTypecDebounceOption = 0x1562,
                   kUsbinAdapterAllow = 0x1360, kUsbinOptions2 = 0x1363, kTypecIntEn1 = 0x155e, kTypecIntEn2 = 0x1560,
                   kWdogPet = 0x1643, kAiclCmd = 0x1644, kAiclRerunTime = 0x1661;
}
constexpr unsigned kFvMv = 4350, kFvMaxMv = 4400, kIclMaxMa = 3000, kFccMaxMa = 3000, kSdpMa = 500;
static_assert(kFvMv <= kFvMaxMv && (kFvMv - 3600) / 10 <= 0xff, "float voltage above the guamp DT limit");
// JEITA thresholds 0x1094-0x109f (soft hot/cold, hard hot/cold, AFP hot/cold; ADC codes, MSB first) as Linux runs
// with them (TWRP dump analisi/pmic/twrp-pc.txt; Linux only reads them, guamp's battery profiles have no
// qcom,jeita-*-thresholds). With the 30 kOhm pull-up and the 10 kOhm NTC they are 45/0 C, 60/-20 C and 71 C (code =
// ~29000 * R/(R + 30k), all six ratios check). The UEFI QcomChargerDxe programs its own codes for 45/60/71 C from
// another NTC table: with them the comparators flagged HOT_SOFT at 29 C and the hard hot limit sits near 31 C.
constexpr uint8_t kJeitaLinux[12] = {0x0f, 0xe1, 0x35, 0xc3, 0x0a, 0x52, 0x4e, 0x42, 0x07, 0x9a, 0x61, 0x64};
// Configuration registers (offsets 0x40-0xcf of CHGR 0x1000, DCDC 0x1100, BATIF 0x1200, USBIN 0x1300, TYPEC 0x1500,
// MISC 0x1600) as Linux runs with them on this PC and cable (TWRP dump analisi/pmic/twrp-pc.txt): differences are
// logged (Controller::linuxDiff), not written blindly.
constexpr uint16_t kLinuxBlocks[6] = {0x1000, 0x1100, 0x1200, 0x1300, 0x1500, 0x1600};
constexpr uint8_t kLinuxCfg[6 * 0x90] = {
    0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x2e, 0x3f, 0x20, 0x7f, 0x00, 0x01, 0x01,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3c, 0x04, 0x02, 0x00, 0x07, 0x3c, 0xfd, 0x65, 0x01, 0x46, 0x03, 0x0a, 0x0a, 0x00, 0x00,
    0x50, 0x00, 0x01, 0x00, 0x00, 0x00, 0x7f, 0x12, 0x0f, 0x0f, 0x00, 0x00, 0x10, 0xfc, 0x54, 0x4a, 0x05, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x14, 0x14, 0x0f, 0xe1, 0x35, 0xc3, 0x0a, 0x52, 0x4e, 0x42, 0x07, 0x9a, 0x61, 0x64,
    0x03, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x02, 0x02, 0x28, 0x00, 0x00, 0x3f, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8b, 0x00, 0x00, 0x03, 0x05, 0x05, 0x26, 0x00,
    0x00, 0x00, 0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xac, 0x55, 0x55, 0x94, 0x0a, 0x0c, 0x1f, 0x01, 0x00, 0x00, 0xf1, 0xba, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xbe, 0xc5, 0x00, 0x37, 0x64, 0x01, 0x00,
    0x00, 0x08, 0x00, 0x00, 0x24, 0x00, 0x0a, 0x00, 0x00, 0xa2, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x54, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x4c, 0x18, 0x11, 0x35, 0x03, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x01,
    0x01, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x12, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0xfb, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x45, 0x79, 0x47, 0xe3, 0x14, 0xa3, 0x18, 0x1f, 0x1c, 0x22, 0x20, 0xb9, 0x00, 0x00, 0x00, 0x00, 0x18, 0x1f, 0x1d, 0xe6, 0x20, 0xb9, 0x23, 0xc1,
    0x14, 0xa3, 0x18, 0x1f, 0x1c, 0x22, 0x20, 0xb9, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
// Registers this driver sets on purpose to other values (FCC, FV, JEITA hard limit kept, ICL, Type-C role) or commands
inline bool linuxDiffSkip(uint16_t a) {
  return a == 0x1061 || a == 0x1070 || a == 0x1090 || a == 0x1370 || a == 0x1544 || a == 0x1643 || a == 0x1644 ||
         (a >= 0x1340 && a <= 0x1347);
}
// AICL thresholds Linux starts from (TWRP: 0x1381 = 04 -> 4400 mV, 0x1384 = 00 -> 4000 mV) and its weak-adapter steps
constexpr unsigned kAicl5vDefaultMv = 4400, kAiclContDefaultMv = 4000, kAiclStepMv = 200, kAiclMaxMv = 4800;
constexpr unsigned kWeakAdapterIclMa = 100;

inline uint8_t aicl5vCode(unsigned mv) {   // param aicl_5v_threshold: 4000..4700 mV, 100 mV steps
  if (mv < 4000) mv = 4000;
  if (mv > 4700) mv = 4700;
  return static_cast<uint8_t>((mv - 4000) / 100);
}
inline uint8_t aiclContCode(unsigned mv) {   // smblib_set_aicl_cont_threshold: 4000..8800 mV, two ranges
  if (mv < 4000) mv = 4000;
  if (mv > 8800) mv = 8800;
  return static_cast<uint8_t>(mv >= 5600 ? 16 + (mv - 5600) / 200 : (mv - 4000) / 100);
}
// Type-C role (TYPE_C_MODE_CFG 0x1544: bit 4 try.SNK, 3 try.SRC, 2 source only, 1 sink only, 0 disable). Sink only
// (default): the phone never sources VBUS here and always presents Rd, so a Type-C charger turns VBUS on. drp: what
// Linux's smb5_configure_typec leaves (try.SNK, no forced role, TWRP 0x10).
inline uint8_t typecMode(uint8_t value, bool drp) {
  return static_cast<uint8_t>((value & 0xe0u) | (drp ? 0x10u : 0x02u));
}
// Input current by source (APSD_RESULT_STATUS 0x1308: bit 0 SDP, 1 OCP, 2 CDP, 3 DCP, 4 float, 5 QC2, 6 QC3)
inline unsigned inputMa(unsigned requested, uint8_t result) {
  if (requested > kIclMaxMa) requested = kIclMaxMa;
  return (result & 0x0e) || requested < kSdpMa ? requested : kSdpMa;
}
inline const char *apsdName(uint8_t result) {
  if (result & 0x40) return "QC3";
  if (result & 0x20) return "QC2";
  if (result & 0x08) return "DCP";
  if (result & 0x04) return "CDP";
  if (result & 0x02) return "OCP";
  if (result & 0x10) return "FLOAT";
  if (result & 0x01) return "SDP";
  return "none";
}
inline const char *rpName(uint8_t snk) {   // TYPE_C_SNK_STATUS 0x1506
  if (snk & 0x02) return "3.0A";
  if (snk & 0x04) return "1.5A";
  if (snk & 0x08) return "default";
  return "none";
}
inline const char *stateName(uint8_t s1) {
  static const char *const k[8] = {"INHIBIT", "TRICKLE", "PRECHG", "FULLON", "TAPER", "TERM", "PAUSE", "DISABLED"};
  return k[s1 & 7];
}

struct Snapshot {
  uint8_t s1, s2, s7, apsd, result, usbin, pp, icl, aicl, cmdIl, sm, misc, snk, legacy, fv, src, sm2, cc, typecRt;
  bool present() const { return (usbin & 0x10) != 0; }
  // A source on CC without VBUS. TYPEC 0x1509 bits 3:0 are the sink state machine (logs: 1 with nothing plugged and
  // 0x150e ff, 3 attached, 2 = waiting for VBUS); 0x150b bit 3 tCC debounce done, bit 4 VBUS error (Rp seen, VBUS never
  // valid). After a C-to-C replug on the PC it stayed at 0x1509 02, 0x150b 18, USBIN 26 (UV, VASHDN, no plug-in).
  bool partnerWithoutVbus() const { return !present() && ((sm & 0x0f) >= 2 || (misc & 0x18)); }
  bool apsdDone() const { return (apsd & 0x01) != 0; }
  bool hwSuspended() const { return (pp & 0x40) != 0 && !(cmdIl & 0x01); }
  bool typecAttached() const { return (sm & 0x20) != 0; }
  // No input current although the cable is in: PAUSE/DISABLED or a 0 mA ICL, with the input collapsed or under
  // voltage (USBIN UV/VASHDN/COLLAPSE or AICL USBIN collapse). Battery-temperature hard limits and BAT_OV are real
  // reasons to stop and never count; FULLON/TAPER with a passing collapse bit do not either.
  bool stuck() const {
    const unsigned st = s1 & 7u;
    if ((s7 & 0x0c) || (s2 & 0x02)) return false;
    if (st != 6 && st != 7 && icl != 0) return false;
    return (usbin & 0x07) || (aicl & 0x10);
  }
};

enum Phase : uint8_t { kAbsent, kDetecting, kConfigured };

struct Controller {
  // configuration (PmicService.inc fills these from boot arguments)
  unsigned iclMa = 1500, fccMa = 2000;
  bool typecDrp = false, recover = true;
  // state
  Phase phase = kAbsent;
  bool started = false, present = false, attached = false, suspended = false, aiclMax = false;
  uint64_t phaseAt = 0, stuckSince = 0, lastRecovery = 0, maxSince = 0, absentSince = 0, lastTypecReset = 0;
  // Type-C recovery without VBUS: CC terminations off (disable command) from ccOffAt until ccOnAt, non-blocking
  uint64_t ccOnAt = 0, noVbusSince = 0, settleUntil = 0, noPartnerSince = 0;
  unsigned lastVbusMv = 0;   // last USBIN ADC reading (taken with Rp and no VBUS), 0 = none
  unsigned ccRecoveries = 0;
  bool roleDrp = false;   // DRP try.SNK for this connection (alternate recovery attempts), back to the default on unplug
  uint32_t lastTypecKey = 0xffffffffu;
  unsigned apsdReruns = 0, aicl5vMv = kAicl5vDefaultMv, aiclContMv = kAiclContDefaultMv;
  unsigned plugins = 0, unplugs = 0, weakSteps = 0, redetects = 0, suspendRecoveries = 0, typecResets = 0, errors = 0;
  unsigned otgOffs = 0;
  uint8_t iclCode = 0, fccCode = 0, lastIcl = 0xff, lastAicl = 0xff, lastResult = 0xff;
  unsigned appliedMa = 0;
  Snapshot snap = {};

  template <class Bus> int masked(Bus &b, uint16_t a, uint8_t mask, uint8_t value, uint8_t *old = nullptr) {
    uint8_t v = 0;
    int e = b.rd(a, &v);
    if (e) return e;
    if (old) *old = v;
    const uint8_t want = static_cast<uint8_t>((v & ~mask) | (value & mask));
    return want == v ? 0 : b.wr(a, want);
  }
  template <class Bus> void fail(Bus &b, const char *what, int e) {
    errors = errors + 1;
    if (errors == 1 || errors % 20 == 0) b.log("GuampChg: %s failed (SPMI %d), errors %u\n", what, e, errors);
  }

  // Static configuration Linux leaves on this board (smb5_init_hw, smb5_configure_typec, mmi init, guamp DT), checked
  // against the TWRP dump; once at boot and again at every configuration, one log line with the values found.
  template <class Bus> int linuxConfig(Bus &b, const char *when) {
    uint8_t aiclOpt = 0, rerun = 0, pull = 0, jeita = 0, pause = 0, cc = 0, bc12 = 0, dbg = 0, th[12] = {};
    int e = masked(b, reg::kAiclOptions, 0x9c, 0x14, &aiclOpt);   // AICL + periodic rerun, no ADC AICL, no suspend on collapse
    if (!e) e = masked(b, reg::kAiclRerunTime, 0xff, 0x01, &rerun);   // 12 s
    if (!e) e = masked(b, reg::kBatifPullUp, 0x03, 0x01, &pull);      // BAT_THERM 30 kOhm (CONFIG_BAT_NTC_10K)
    if (!e) e = masked(b, reg::kJeitaEn, 0x0f, 0x00, &jeita);         // no soft-JEITA FV/FCC compensation
    if (!e) e = masked(b, reg::kChgPause, 0x01, 0x00, &pause);
    if (!e) e = masked(b, reg::kTypeCCfg, 0x80, 0x00, &cc);           // BC1.2 not gated on CC (no debug accessory)
    if (!e) e = masked(b, reg::kUsbinOptions1, 0x08, 0x08, &bc12);    // BC1.2 source detection on (smblib_apsd_enable)
    if (!e) e = masked(b, reg::kTypecDebugAccessSink, 0x1f, 0x03, &dbg);
    // smb5_configure_typec / smblib_set_prop_pr_swap_in_progress outside a role swap: CC threshold 1.6 V in source
    // mode, no sink exit on CC alone, no VSAFE0V bypass, normal tCC debounce, CC-out not software driven (TWRP 06/00/00)
    uint8_t exitCfg = 0, debounce = 0, ccout = 0;
    if (!e) e = masked(b, reg::kTypecExitStateCfg, 0x0d, 0x04, &exitCfg);
    if (!e) e = masked(b, reg::kTypecDebounceOption, 0x04, 0x00, &debounce);
    if (!e) e = masked(b, reg::kTypecCcoutControl, 0x01, 0x00, &ccout);
    // Left by the UEFI, differing from Linux in the XNU dumps (UEFI / Linux): USBIN_ADAPTER_ALLOW_CFG 03 / 08
    // (smblib_set_adapter_allowance 5V_TO_9V, mmi init and every Type-C removal; an input window, no charge limit),
    // USBIN_OPTIONS_2 float/DCD 19 / 18 (smb5_init_hw float_cfg 0), TYPE_C_INTERRUPT_EN_CFG_1 ff / 00 and _CFG_2 5f / 40
    // (smb5_configure_typec simple writes: no water detection on PMI632, lpd disabled, no VBUS-error/state interrupts)
    uint8_t allow = 0, opt2 = 0, int1 = 0, int2 = 0;
    if (!e) e = b.rd(reg::kUsbinAdapterAllow, &allow);
    if (!e && allow != 0x08) e = b.wr(reg::kUsbinAdapterAllow, 0x08);
    if (!e) e = masked(b, reg::kUsbinOptions2, 0x27, 0x00, &opt2);
    if (!e) e = b.rd(reg::kTypecIntEn1, &int1);
    if (!e && int1 != 0x00) e = b.wr(reg::kTypecIntEn1, 0x00);
    if (!e) e = b.rd(reg::kTypecIntEn2, &int2);
    if (!e && int2 != 0x40) e = b.wr(reg::kTypecIntEn2, 0x40);
    bool same = true;
    for (unsigned k = 0; k < 12 && !e; k = k + 1) {
      e = b.rd(static_cast<uint16_t>(reg::kJeitaThresholds + k), &th[k]);
      if (th[k] != kJeitaLinux[k]) same = false;
    }
    if (!e && !same) {
      // Hard limits off while the 16-bit codes are half written, then back as they were
      uint8_t en = 0;
      e = masked(b, reg::kJeitaEn, 0x30, 0x00, &en);
      for (unsigned k = 0; k < 12 && !e; k = k + 1) e = b.wr(static_cast<uint16_t>(reg::kJeitaThresholds + k), kJeitaLinux[k]);
      if (!e) e = masked(b, reg::kJeitaEn, 0x30, en);
    }
    if (e) { fail(b, "Linux config", e); return e; }
    static const char hexd[] = "0123456789abcdef";
    char text[25];
    for (unsigned k = 0; k < 12; k = k + 1) {
      text[2 * k] = hexd[th[k] >> 4];
      text[2 * k + 1] = hexd[th[k] & 15];
    }
    text[24] = 0;
    b.log("GuampChg: Linux config (%s), before: AICL-opt %02x rerun %02x therm-pull %02x JEITA-en %02x pause %02x "
          "TypeC-cfg %02x BC1.2 %02x dbg-sink %02x exit %02x debounce %02x ccout %02x allow %02x opt2 %02x tc-int %02x %02x "
          "JEITA thresholds %s%s\n",
          when, aiclOpt, rerun, pull, jeita, pause, cc, bc12, dbg, exitCfg, debounce, ccout, allow, opt2, int1, int2, text,
          same ? " (Linux)" : " -> Linux 45/0/60/-20/71 C");
    return 0;
  }

  template <class Bus> int readSnapshot(Bus &b, Snapshot &s) {
    struct { uint16_t a; uint8_t *v; } const list[] = {
        {reg::kStatus1, &s.s1}, {reg::kStatus2, &s.s2}, {reg::kStatus7, &s.s7}, {reg::kApsdStatus, &s.apsd},
        {reg::kApsdResult, &s.result}, {reg::kUsbinRt, &s.usbin}, {reg::kPowerPath, &s.pp}, {reg::kIclStatus, &s.icl},
        {reg::kAiclStatus, &s.aicl}, {reg::kUsbinCmdIl, &s.cmdIl}, {reg::kTypecStateMachine, &s.sm},
        {reg::kTypecMisc, &s.misc}, {reg::kTypecSnkStatus, &s.snk}, {reg::kTypecLegacy, &s.legacy}, {reg::kFv, &s.fv},
        {reg::kTypecSrcStatus, &s.src}, {reg::kTypecStatus0a, &s.sm2}, {reg::kTypecStatus0e, &s.cc},
        {reg::kTypecRt, &s.typecRt}};
    for (const auto &r : list) {
      int e = b.rd(r.a, r.v);
      if (e) return e;
    }
    return 0;
  }

  template <class Bus> int setAiclThresholds(Bus &b) {
    int e = b.wr(reg::kAicl5v, aicl5vCode(aicl5vMv));
    return e ? e : b.wr(reg::kAiclCont, aiclContCode(aiclContMv));
  }

  // Edge work shared by plug-in and unplug (Linux: WEAK_ADAPTER_WA thresholds back to default on removal,
  // smblib_apsd_enable(true) on Type-C detach, input not suspended).
  template <class Bus> int edgeReset(Bus &b) {
    aicl5vMv = kAicl5vDefaultMv;
    aiclContMv = kAiclContDefaultMv;
    aiclMax = false;
    maxSince = stuckSince = 0;
    int e = setAiclThresholds(b);
    if (!e) e = masked(b, reg::kUsbinOptions1, 0x08, 0x08);
    if (!e) e = masked(b, reg::kUsbinCmdIl, 0x01, 0x00);
    return e;
  }

  // smblib_set_icl_current in high-current mode (SW_OVERRIDE_HC_MODE: USBIN_MODE_CHG, no USB51 command override,
  // ICL kept after APSD), then FV/FCC, the charge enable toggle (inhibit re-evaluated with the new float voltage),
  // unsuspend and a read back of everything written.
  template <class Bus> int configure(Bus &b, unsigned ma) {
    if (ma > kIclMaxMa) ma = kIclMaxMa;
    unsigned fcc = fccMa > kFccMaxMa ? kFccMaxMa : fccMa;
    const uint8_t icl = static_cast<uint8_t>(ma / 50), fc = static_cast<uint8_t>(fcc / 50),
                  fv = static_cast<uint8_t>((kFvMv - 3600) / 10);
    int e = b.wr(reg::kIclCfg, icl);
    if (!e) e = masked(b, reg::kIclOptions, 0x03, 0x03);
    if (!e) e = masked(b, reg::kCmdIclOverride, 0x01, 0x00);
    if (!e) e = masked(b, reg::kUsbinLoadCfg, 0x10, 0x10);
    if (!e) e = b.wr(reg::kFv, fv);
    if (!e) e = b.wr(reg::kFcc, fc);
    if (!e && !(snap.s2 & 0x02)) {   // battery over-voltage: no off/on restart, only the enable bit below
      e = masked(b, reg::kChgEnable, 0x01, 0x00);
      b.sleepMs(1);
    }
    if (!e) e = masked(b, reg::kChgEnable, 0x01, 0x01);
    if (!e) e = masked(b, reg::kUsbinCmdIl, 0x01, 0x00);
    uint8_t rIcl = 0, rFcc = 0, rFv = 0, rEn = 0;
    if (!e) e = b.rd(reg::kIclCfg, &rIcl);
    if (!e) e = b.rd(reg::kFcc, &rFcc);
    if (!e) e = b.rd(reg::kFv, &rFv);
    if (!e) e = b.rd(reg::kChgEnable, &rEn);
    if (!e && (rIcl != icl || rFcc != fc || rFv != fv || !(rEn & 1))) e = -5;
    if (!e) { iclCode = icl; fccCode = fc; appliedMa = ma; }
    return e;
  }

  // usbin_uv_irq_handler with WEAK_ADAPTER_WA: suspend, VASHDN deglitch, AICL thresholds +200 mV (5 V and
  // continuous); past 4.8 V the ICL is held at 100 mA instead; restart AICL, unsuspend.
  template <class Bus> int weakStep(Bus &b) {
    int e = masked(b, reg::kUsbinCmdIl, 0x01, 0x01);
    if (e) return e;
    b.sleepMs(20);
    if (aicl5vMv > kAiclMaxMv) {
      aiclMax = true;
    } else {
      aicl5vMv = aicl5vMv + kAiclStepMv;
      aiclContMv = aiclContMv + kAiclStepMv;
      e = setAiclThresholds(b);
    }
    if (!e && aiclMax) e = b.wr(reg::kIclCfg, static_cast<uint8_t>(kWeakAdapterIclMa / 50));
    int u = masked(b, reg::kUsbinCmdIl, 0x01, 0x00);   // never leave the input suspended
    if (!e) e = u;
    if (!e) e = masked(b, reg::kAiclCmd, 0x02, 0x02);   // RESTART_AICL
    return e;
  }

  // One log line per 12 configuration registers that differ from Linux (kLinuxCfg), at most 8 lines, the input side
  // first: TYPEC, USBIN, DCDC, MISC, CHGR, BATIF
  template <class Bus> void linuxDiff(Bus &b, const char *when) {
    static const char hexd[] = "0123456789abcdef";
    char text[12 * 11 + 1];
    static const uint8_t kOrder[6] = {4, 3, 1, 5, 0, 2};
    unsigned at = 0, n = 0, lines = 0;
    for (unsigned i = 0; i < 6 && lines < 8; i = i + 1)
      for (unsigned o = 0, blk = kOrder[i]; o < 0x90 && lines < 8; o = o + 1) {
        const uint16_t a = static_cast<uint16_t>(kLinuxBlocks[blk] + 0x40 + o);
        uint8_t v = 0;
        if (linuxDiffSkip(a) || b.rd(a, &v) || v == kLinuxCfg[blk * 0x90 + o]) continue;
        const uint8_t l = kLinuxCfg[blk * 0x90 + o];
        const char item[10] = {' ', hexd[(a >> 12) & 15], hexd[(a >> 8) & 15], hexd[(a >> 4) & 15], hexd[a & 15], ' ',
                               hexd[v >> 4], hexd[v & 15], '/', hexd[l >> 4]};
        for (unsigned k = 0; k < 10; k = k + 1) text[at + k] = item[k];
        text[at + 10] = hexd[l & 15];
        at = at + 11;
        n = n + 1;
        if (n % 12 == 0) {
          text[at] = 0;
          b.log("GuampChg: vs Linux (%s, XNU/Linux):%s\n", when, text);
          at = 0;
          lines = lines + 1;
        }
      }
    if (at || !n) {
      text[at] = 0;
      b.log("GuampChg: vs Linux (%s, XNU/Linux):%s%s\n", when, text, n ? "" : " no difference");
    }
  }

  // OTG boost off (DCDC_CMD_OTG_REG OTG_EN_BIT 0) and OTG enabled by that command only (DCDC_OTG_CFG_REG
  // OTG_EN_SRC_CFG_BIT 0, qpnp-smb5.c smb5_init_hw). Linux sets OTG_EN only as USB host (smblib_vbus_regulator_enable,
  // smblib_notify_usb_host); this port is sink only. The XNU logs after a C-to-C replug showed 0x1140 = 01 (Linux 00,
  // nothing in XNU writes it: left by the boot chain): the phone drove its own 5 V on VBUS (USBIN REVI + UV/VASHDN, USB_IN
  // ADC ~5.0 V "present, not accepted", Type-C VBUS_ERROR), so the PC never turned its VBUS back on.
  template <class Bus> int otgOff(Bus &b, const char *when) {
    uint8_t cmd = 0, cfg = 0, usbin = 0;
    int e = b.rd(reg::kOtgCmd, &cmd);
    if (!e) e = masked(b, reg::kOtgCfg, 0x02, 0x00, &cfg);
    if (!e && (cmd & 0x01)) e = b.wr(reg::kOtgCmd, static_cast<uint8_t>(cmd & ~0x01u));
    if (e) { fail(b, "OTG off", e); return e; }
    if ((cmd & 0x01) || (cfg & 0x02)) {
      otgOffs = otgOffs + 1;
      b.rd(reg::kUsbinRt, &usbin);
      if (otgOffs <= 3 || otgOffs % 20 == 0)
        b.log("GuampChg: OTG boost was on (%s: DCDC_CMD_OTG %02x, OTG_CFG %02x, USBIN %02x): off, sink only (%u)\n", when,
              cmd, cfg, usbin, otgOffs);
    }
    return 0;
  }

  // Type-C disable command (CC terminations off) now, the role back on at the first poll after ms + offMs
  template <class Bus> void typecOff(Bus &b, uint64_t ms, uint8_t role, unsigned offMs, const char *why) {
    lastTypecReset = ms;
    int e = b.wr(reg::kTypecModeCfg, static_cast<uint8_t>(role | 0x01));
    if (e) { fail(b, "Type-C disable", e); return; }
    ccOnAt = ms + offMs;
    typecResets = typecResets + 1;
    if (why && (typecResets == 1 || typecResets % 20 == 0))
      b.log("GuampChg: Type-C off for %u ms (%s), reset %u\n", offMs, why, typecResets);
  }

  template <class Bus> void beginDetection(Bus &b, uint64_t ms, bool rerun, const char *why) {
    phase = kDetecting;
    phaseAt = ms;
    apsdReruns = 0;
    if (rerun) {
      int e = b.wr(reg::kCmdApsd, 0x01);
      if (e) fail(b, "APSD rerun", e);
      else apsdReruns = 1;
    }
    b.log("GuampChg: source detection (%s)%s, APSD %02x result %02x\n", why, rerun ? ", APSD rerun" : "", snap.apsd,
          snap.result);
  }

  template <class Bus> void poll(Bus &b, uint64_t ms) {
    int e = b.wr(reg::kWdogPet, 1);
    uint8_t mode = 0;
    if (!e) e = b.rd(reg::kTypecModeCfg, &mode);
    const bool drp = typecDrp || roleDrp;
    bool justOn = false;
    const uint8_t wantMode = typecMode(mode, drp);
    if (!e && ccOnAt) {
      if (ms < ccOnAt) return;   // CC terminations still off: the partner must see a detach
      ccOnAt = 0;
      justOn = true;
      settleUntil = ms + 3000;   // unattached -> waiting for VBUS takes a moment; judge the result after that
      e = b.wr(reg::kTypecModeCfg, wantMode);
      if (!e && ccRecoveries) b.log("GuampChg: Type-C on again, role %02x (%s)\n", wantMode, drp ? "DRP try.SNK" : "sink only");
    } else if (!e && mode != wantMode) {
      e = b.wr(reg::kTypecModeCfg, wantMode);
      if (!e) b.log("GuampChg: Type-C role %02x -> %02x (%s)\n", mode, wantMode, drp ? "DRP try.SNK" : "sink only");
    }
    Snapshot s = {};
    if (!e) e = readSnapshot(b, s);
    if (e) { fail(b, "status read", e); return; }
    snap = s;
    if (!started) {
      started = true;
      linuxDiff(b, "boot, before");
      linuxConfig(b, "boot");
    }
    otgOff(b, present ? "plugged" : "no VBUS");   // every poll: whatever turned it on, the next poll undoes it

    // --- transitions, one line each ---------------------------------------------------------------------------
    if (s.typecAttached() != attached) {
      attached = s.typecAttached();
      if (attached)
        b.log("GuampChg: Type-C attach, Rp %s, CC%u, %s%s, state %02x misc %02x\n", rpName(s.snk),
              (s.misc & 0x02) ? 2u : 1u, (s.legacy & 0x02) ? "legacy cable" : "Type-C cable",
              (s.legacy & 0x01) ? " (non-compliant)" : "", s.sm, s.misc);
      else
        b.log("GuampChg: Type-C detach, state %02x misc %02x\n", s.sm, s.misc);
    }
    const uint32_t typecKey = (static_cast<uint32_t>(s.sm) << 24) | (static_cast<uint32_t>(s.misc & 0x1b) << 16) |
                              (static_cast<uint32_t>(s.snk & 0x0f) << 8);
    if (typecKey != lastTypecKey && !justOn) {   // not in the poll that just turned CC back on (still settling)
      if (lastTypecKey != 0xffffffffu || s.sm != 0x01)
        b.log("GuampChg: Type-C state %02x%s misc %02x%s%s%s%s Rp %s src %02x 0a %02x 0e %02x legacy %02x rt %02x, USBIN %02x%s\n",
              s.sm, (s.sm & 0x20) ? " ATTACHED" : (s.sm & 0x0f) == 2 ? " WAIT_VBUS" : (s.sm & 0x0f) == 1 ? " UNATTACHED" : "",
              s.misc, (s.misc & 0x01) ? " CC_ATTACHED" : "", (s.misc & 0x02) ? " CC2" : "",
              (s.misc & 0x08) ? " DEBOUNCED" : "", (s.misc & 0x10) ? " VBUS_ERROR" : "", rpName(s.snk), s.src, s.sm2,
              s.cc, s.legacy, s.typecRt, s.usbin, s.present() ? " PLUGIN" : " no VBUS");
      lastTypecKey = typecKey;
    }
    if (s.hwSuspended() != suspended) {
      suspended = s.hwSuspended();
      b.log("GuampChg: input %s, power path %02x, USBIN %02x\n", suspended ? "suspended by the charger" : "resumed",
            s.pp, s.usbin);
    }
    if (lastIcl == 0xff || (lastIcl == 0) != (s.icl == 0) || (s.icl > lastIcl ? s.icl - lastIcl : lastIcl - s.icl) >= 4) {
      if (lastIcl != 0xff || s.present())
        b.log("GuampChg: ICL %u -> %u mA (AICL %02x, %s)\n", lastIcl == 0xff ? 0u : lastIcl * 50u, s.icl * 50u, s.aicl,
              stateName(s.s1));
      lastIcl = s.icl;
    }
    if (lastAicl == 0xff || ((lastAicl ^ s.aicl) & 0x17)) {
      if (lastAicl != 0xff || s.present())
        b.log("GuampChg: AICL %02x%s%s%s%s\n", s.aicl, (s.aicl & 0x01) ? " DONE" : "", (s.aicl & 0x02) ? " FAIL" : "",
              (s.aicl & 0x04) ? " ICL_MIN" : "", (s.aicl & 0x10) ? " USBIN_COLLAPSE" : "");
      lastAicl = s.aicl;
    }

    // --- unplug / plug-in -----------------------------------------------------------------------------------
    if (!s.present()) {
      if (present || phase != kAbsent) {
        unplugs = unplugs + 1;
        e = edgeReset(b);
        if (!e) {   // next source starts from the safe SDP current (a weak-adapter 100 mA must not stick)
          e = b.wr(reg::kIclCfg, static_cast<uint8_t>(kSdpMa / 50));
          if (!e) iclCode = static_cast<uint8_t>(kSdpMa / 50);
        }
        if (e) fail(b, "unplug reset", e);
        b.log("GuampChg: unplug %u, USBIN %02x, %s, AICL %u/%u mV, ICL %u mA, input on\n", unplugs, s.usbin,
              stateName(s.s1), aicl5vMv, aiclContMv, kSdpMa);
        if (unplugs == 1) linuxDiff(b, "unplug");
        present = false;
        phase = kAbsent;
        absentSince = ms;
        lastResult = 0xff;
      }
      if (ms < settleUntil) return;   // the state machine is still re-attaching after a recovery
      if (!s.partnerWithoutVbus()) {
        // Only 10 s without anything on CC count as a real unplug: right after CC comes back on the state machine
        // passes through "unattached", and that must not restart the recovery ladder at attempt 1
        if (!noPartnerSince) noPartnerSince = ms;
        if (ms - noPartnerSince >= 10000) {
          noVbusSince = 0;
          ccRecoveries = 0;
          roleDrp = false;   // nothing on CC: back to the configured role
        }
        // A Type-C state machine that sees nothing at all (wall charger without attach) gets the disable command for
        // one poll every 15 s
        if (ms - absentSince >= 15000 && ms - lastTypecReset >= 15000) typecOff(b, ms, wantMode, 1000, "nothing on CC");
        return;
      }
      noPartnerSince = 0;
      // Rp on CC but VBUS never valid (0x150b VBUS_ERROR: the sink saw Rp, VBUS did not reach the valid window;
      // USBIN UV+VASHDN = below the plug-in threshold). The source only turns VBUS (back) on for a new attach: take
      // the sink's Rd off for longer than the source's detach debounce and VBUS discharge, escalating 2/3/5/10 s and
      // alternating sink only with Linux's DRP try.SNK; first after 5 s, then every 20 s, after 6 attempts every 60 s.
      // The USBIN ADC tells whether VBUS is really missing (PC port off) or present but not accepted by the charger.
      if (!noVbusSince) noVbusSince = ms;
      const uint64_t gap = ccRecoveries == 0 ? 5000 : ccRecoveries < 6 ? 20000 : 60000;
      if (ms - noVbusSince >= 5000 && ms - lastTypecReset >= gap) {
        static const unsigned kOffMs[4] = {2000, 3000, 5000, 10000};
        const unsigned offMs = kOffMs[ccRecoveries % 4];
        ccRecoveries = ccRecoveries + 1;
        if (!typecDrp) roleDrp = (ccRecoveries % 2) == 0;
        unsigned mv = 0;
        const int ve = b.vbusMv(&mv);
        lastVbusMv = ve ? 0 : mv;
        uint8_t otg = 0, allow = 0;
        b.rd(reg::kOtgCmd, &otg);
        b.rd(reg::kUsbinAdapterAllow, &allow);
        if (ccRecoveries == 1) linuxDiff(b, "Rp without VBUS");
        b.log("GuampChg: Rp without VBUS for %u s (state %02x misc %02x 0e %02x, USBIN %02x, VBUS ADC %u mV%s, OTG %02x "
              "cmd-il %02x allow %02x): Type-C recovery %u, CC off %u s, then %s\n",
              static_cast<unsigned>((ms - noVbusSince) / 1000), s.sm, s.misc, s.cc, s.usbin, ve ? 0u : mv,
              ve ? " (ADC error)" : mv >= 4000 ? ((otg & 1) ? " OWN OTG BOOST" : " PRESENT, NOT ACCEPTED")
                 : mv >= 800 ? " not discharged" : "",
              otg, s.cmdIl, allow, ccRecoveries, offMs / 1000, (typecDrp || roleDrp) ? "DRP try.SNK" : "sink only");
        typecOff(b, ms, typecMode(wantMode, typecDrp || roleDrp), offMs, nullptr);
      }
      return;
    }
    if (!present) {
      present = true;
      plugins = plugins + 1;
      if (ccRecoveries)
        b.log("GuampChg: VBUS back after Type-C recovery %u (%s)\n", ccRecoveries, drp ? "DRP try.SNK" : "sink only");
      noVbusSince = 0;
      noPartnerSince = 0;
      ccRecoveries = 0;
      e = edgeReset(b);
      if (e) fail(b, "plug-in reset", e);
      b.log("GuampChg: plug-in %u (%s), USBIN %02x, %s, Type-C %s Rp %s, APSD %02x result %02x\n", plugins,
            plugins == 1 && unplugs == 0 ? "at boot" : "edge", s.usbin, stateName(s.s1),
            s.typecAttached() ? "attached" : "unattached", rpName(s.snk), s.apsd, s.result);
      // At boot APSD ran before this driver: rerun only when it has no result (smblib_rerun_apsd_if_required);
      // on a real plug-in the hardware runs it by itself.
      beginDetection(b, ms, plugins == 1 && unplugs == 0 && !s.apsdDone(), "plug-in");
    }

    // --- source detection and configuration ---------------------------------------------------------------------
    if (phase == kDetecting) {
      if (s.apsdDone() || (apsdReruns >= 2 && ms - phaseAt >= 10000)) {
        const uint8_t result = s.apsdDone() ? s.result : 0;
        const unsigned ma = inputMa(iclMa, result);
        int c = linuxConfig(b, "configure");
        if (!c) c = configure(b, ma);
        b.log("GuampChg: APSD %s result %02x %s -> ICL %u mA FCC %u mA FV %u mV: %s (SPMI %d)\n",
              s.apsdDone() ? "done" : "timeout", result, apsdName(result), ma, fccCode * 50u, kFvMv,
              c ? "error" : "configured", c);
        if (c) { fail(b, "configure", c); return; }   // retried on the next poll
        lastResult = result;
        phase = kConfigured;
        phaseAt = ms;
        stuckSince = 0;
      } else if (ms - phaseAt >= 3000u * (apsdReruns + 1) && apsdReruns < 2) {
        e = b.wr(reg::kCmdApsd, 0x01);
        if (e) fail(b, "APSD rerun", e);
        else {
          apsdReruns = apsdReruns + 1;
          b.log("GuampChg: APSD rerun %u, status %02x\n", apsdReruns, s.apsd);
        }
      }
      return;
    }

    // --- configured: watch for a changed source and a stuck input ------------------------------------------------
    if (s.apsdDone() && s.result != lastResult && lastResult != 0xff) {
      b.log("GuampChg: APSD result %02x %s -> %02x %s\n", lastResult, apsdName(lastResult), s.result, apsdName(s.result));
      beginDetection(b, ms, false, "source change");
      return;
    }
    if (!recover) return;
    if (s.hwSuspended() && ms - lastRecovery >= 30000) {
      // Suspended on collapse before the AICL options were set: the suspend command toggle releases it, as
      // Linux's usb_icl vote 0 -> ICL does
      lastRecovery = ms;
      e = masked(b, reg::kUsbinCmdIl, 0x01, 0x01);
      b.sleepMs(20);
      int e2 = masked(b, reg::kUsbinCmdIl, 0x01, 0x00);
      suspendRecoveries = suspendRecoveries + 1;
      b.log("GuampChg: suspend toggle %u (power path %02x): %s\n", suspendRecoveries, s.pp, e || e2 ? "error" : "done");
      return;
    }
    if (!s.stuck()) { stuckSince = 0; return; }
    if (!stuckSince) stuckSince = ms;
    if (ms - stuckSince < 10000 || ms - lastRecovery < 15000) return;
    const unsigned stuckS = static_cast<unsigned>((ms - stuckSince) / 1000);
    if (aiclMax) {
      // Linux does nothing more once the AICL threshold is at its maximum; here, after 60 s more without input
      // current, start over as for a new plug-in (a missed unplug edge, Codex's controller)
      if (ms - maxSince < 60000) return;
      lastRecovery = ms;
      redetects = redetects + 1;
      e = edgeReset(b);
      if (e) fail(b, "re-detect reset", e);
      b.log("GuampChg: no input current for %u s at 100 mA, re-detect %u\n", stuckS, redetects);
      beginDetection(b, ms, true, "re-detect");
      return;
    }
    lastRecovery = ms;
    weakSteps = weakSteps + 1;
    e = weakStep(b);
    if (aiclMax) maxSince = ms;
    b.log("GuampChg: no input current for %u s (%s, USBIN %02x, AICL %02x, ICL %u mA): weak-adapter step %u, AICL "
          "%u/%u mV%s: %s\n",
          stuckS, stateName(s.s1), s.usbin, s.aicl, s.icl * 50u, weakSteps,
          aicl5vMv, aiclContMv, aiclMax ? ", ICL 100 mA" : "", e ? "error" : "AICL restarted");
    if (e) fail(b, "weak-adapter step", e);
  }
};
}  // namespace guamp_charge
#endif
