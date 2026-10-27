// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp PMIC service, included by NativeEmmc.cpp after UsbMirror.inc (same file for the mapping helper and the boot log).
// One kernel thread on the SPMI PMIC arbiter (GuampSpmi.c logic, GuampPmicBoard.h facts):
//   buttons   Power and Volume Down from the PM6125 PON real-time status, Volume Up from PM6125 GPIO 5, polled every
//             25 ms with a 2-sample debounce, sent to NativeHidInput's button sink (report 2: bit 1 Power, bit 2 Volume
//             Up, bit 3 Volume Down) when the pressed set changes
//   charger   PMI632 SMB5, GuampChargeReplugCore.h polled every second: pets the charger watchdog, keeps the Type-C sink
//             role, and on every plug-in (also at boot) runs source detection (APSD), the input current limit, FV, FCC
//             and charge enable as Linux smb5-lib.c does; on unplug it resets AICL thresholds, ICL and input suspend;
//             a cable that stays in without input current gets Linux's weak-adapter AICL steps, then a re-detection.
//             One log line per transition (plug, unplug, APSD, Type-C attach/detach, ICL, AICL, suspend)
//   ringer    the Assistant key (TLMM GPIO 33, active low) is the ring/silent switch: every debounced press toggles a
//             latched state that starts at "ring"; it is report 2 bit 4 (Telephony page 0x0B usage 0x2E, level: 1 ring,
//             0 silent). backboardd BKButtonHIDEventProcessor -processEvent:sender:dispatcher: routes a keyboard event
//             with page 0x0B usage 0x2E to -_processRingerSwitchEvent:down: (down != 0 -> "tone1", 0 -> "silent") and
//             SpringBoard -__handleHIDEvent: 0xB002E -> -_handleRingerSwitchEvent: (gated by MobileGestalt ringer-switch
//             = ADT /buttons property function-button_ringerab, see make-guamp-adt.py) -> _updateRingerState:.
// Boot arguments: guamp-pmic=0 turns it off; guamp-icl-ma (default 1500, used for OCP/CDP/DCP only, SDP 500) and
// guamp-fcc-ma (default 2000), both capped at 3000 (guamp DT); guamp-typec-drp=1 keeps Linux's DRP try.SNK role
// instead of sink only; guamp-chg-recover=0 turns off the stuck-input recovery (weak-adapter steps, re-detection);
// guamp-ringer=0 leaves GPIO 33 alone, guamp-ringer-start=0 starts at "silent".
// Every 30 s one log line: buttons, charger status bytes, APSD result, the codes written, SPMI errors.

#include <stdarg.h>
#include "GuampSpmi.h"
#include "GuampPmicBoard.h"
#include "GuampChargeReplugCore.h"

extern "C" {
extern uint32_t cs_text_fixup_parsed, cs_text_fixup_applied;
extern uint32_t cs_text_fixup_chunk_calls, cs_text_fixup_name_matches, cs_text_fixup_byte_mismatches;
extern uint32_t native_guamp_home_reset_id, native_guamp_home_reset_deleted,
    native_guamp_home_reset_absent, native_guamp_home_reset_error, native_guamp_home_reset_completed;
extern uint32_t native_guamp_ui_cache_cycle_id, native_guamp_ui_cache_deleted,
    native_guamp_ui_cache_absent, native_guamp_ui_cache_error, native_guamp_ui_cache_completed;
}
namespace pmic {
// --- SPMI arbiter v5 (copy of GuampSpmi.c in C++ form, host-tested there) --------------------------------------
volatile uint8_t *gCore, *gCnfg, *gObs, *gChn;
uint32_t ioRd(void *, unsigned region, uint32_t off) {
  volatile uint8_t *base = region == SPMI_CORE ? gCore : region == SPMI_CNFG ? gCnfg : region == SPMI_OBSRVR ? gObs : gChn;
  return *reinterpret_cast<volatile uint32_t *>(base + off);
}
void ioWr(void *, unsigned region, uint32_t off, uint32_t v) {
  volatile uint8_t *base = region == SPMI_CORE ? gCore : region == SPMI_CNFG ? gCnfg : region == SPMI_OBSRVR ? gObs : gChn;
  *reinterpret_cast<volatile uint32_t *>(base + off) = v;
  __asm__ volatile("dsb sy" ::: "memory");
}
void ioDelay(void *, unsigned us) { IODelay(us); }

struct spmi_arb gArb;
volatile int gArbReady = 0;   // set once init() filled the APID table (TouchService reads the panel rails through read8)

int init() {
  struct spmi_io io = {nullptr, ioRd, ioWr, ioDelay};
  gArb.io = io;
  gArb.ee = 0;
  gArb.version = ioRd(nullptr, SPMI_CORE, 0);
  if (gArb.version < SPMI_V5_MIN || gArb.version >= SPMI_V7_MIN) return SPMI_ERR_VERSION;
  gArb.apids = ioRd(nullptr, SPMI_CORE, 4) & 0x7ffu;
  if (gArb.apids > SPMI_MAX_APIDS) gArb.apids = SPMI_MAX_APIDS;
  for (unsigned n = 0; n < gArb.apids; ++n) {
    uint32_t map = ioRd(nullptr, SPMI_CORE, 0x900 + 4 * n);
    if (!map) continue;
    uint16_t ppid = static_cast<uint16_t>((map >> 8) & 0xfffu);
    gArb.write_owner[n] = static_cast<uint8_t>(ioRd(nullptr, SPMI_CNFG, 0x700 + 4 * n) & 7u);
    if (!gArb.ppid_to_apid[ppid] || gArb.write_owner[n] == 0) gArb.ppid_to_apid[ppid] = static_cast<uint16_t>(n + 1);
  }
  return SPMI_OK;
}
int waitDone(unsigned region, uint32_t chan) {
  for (unsigned n = 0; n < SPMI_TIMEOUT_POLLS; ++n) {
    uint32_t st = ioRd(nullptr, region, chan + 0x08);
    gArb.last_status = st;
    if (st & 1u) return (st & 0xeu) ? ((st & 4u) ? SPMI_ERR_PERM : SPMI_ERR_IO) : SPMI_OK;
    IODelay(1);
  }
  return SPMI_ERR_TIMEOUT;
}
int apidOf(unsigned sid, uint16_t addr) {
  uint16_t e = gArb.ppid_to_apid[(sid << 8) | (addr >> 8)];
  return e ? e - 1 : SPMI_ERR_NODEV;
}
int read8(unsigned sid, uint16_t addr, uint8_t *v) {
  int apid = apidOf(sid, addr);
  if (apid < 0) return apid;
  uint32_t chan = 0x80u * static_cast<uint32_t>(apid);
  ioWr(nullptr, SPMI_OBSRVR, chan, (1u << 27) | (static_cast<uint32_t>(addr & 0xff) << 4));
  int r = waitDone(SPMI_OBSRVR, chan);
  if (!r) *v = static_cast<uint8_t>(ioRd(nullptr, SPMI_OBSRVR, chan + 0x18) & 0xff);
  return r;
}
int write8(unsigned sid, uint16_t addr, uint8_t v) {
  int apid = apidOf(sid, addr);
  if (apid < 0) return apid;
  if (gArb.write_owner[apid] != 0) return SPMI_ERR_PERM;
  uint32_t chan = 0x10000u * static_cast<uint32_t>(apid);
  ioWr(nullptr, SPMI_CHNLS, chan + 0x10, v);
  ioWr(nullptr, SPMI_CHNLS, chan, static_cast<uint32_t>(addr & 0xff) << 4);
  return waitDone(SPMI_CHNLS, chan);
}

#include "GuampBatteryService.inc"

// --- board ---------------------------------------------------------------------------------------------------------
constexpr unsigned kPm6125 = 0, kPmi632 = 2;
constexpr uint16_t kPonRtSts = 0x0810, kVolUpStatus = 0xc408;
// SMB5 charger registers for the status lines (smb5-reg.h; the controller's own list is guamp_charge::reg)
constexpr uint16_t kChgEnableCmd = 0x1042, kTypecModeCfg = 0x1544, kAiclOptions = 0x1380, kBatifPullUp = 0x1286,
                   kJeitaEnCfg = 0x1090, kChgPauseCmd = 0x1043, kIclStatus = 0x1107, kAiclStatus = 0x110a,
                   kPowerPath = 0x110b;

void (*volatile gSink)(void *, uint32_t);
void *volatile gSinkContext;
volatile uint32_t gButtons, gErrors;
volatile uint8_t gChg[8], gTypecMode;

uint32_t readButtons() {
  uint8_t pon = 0, vup = 1;
  uint32_t mask = 0;
  if (read8(kPm6125, kPonRtSts, &pon)) gErrors = gErrors + 1;
  else {
    if (pon & 1u) mask |= 1u << 1;     // Power
    if (pon & 2u) mask |= 1u << 3;     // Volume Down (RESIN)
  }
  if (read8(kPm6125, kVolUpStatus, &vup)) gErrors = gErrors + 1;
  else if (!(vup & 1u)) mask |= 1u << 2;   // Volume Up, active low
  return mask;
}

// The charger controller's register access: SPMI slave 2, IOSleep, one IOLog line per call
struct ChargeBus {
  int rd(uint16_t addr, uint8_t *v) { return read8(kPmi632, addr, v); }
  int wr(uint16_t addr, uint8_t v) { return write8(kPmi632, addr, v); }
  void sleepMs(unsigned ms) { IOSleep(ms); }
  // One ADC5 conversion of USB_IN_V_16 (channel 0x08, 1/16 prescaler, absolute calibration, qcom-spmi-adc5.c
  // adc_configure/adc_poll_wait_eoc and pmi632.dtsi), on the same user registers as batteryTherm (same thread).
  int vbusMv(unsigned *mv) {
    uint8_t p[5];
    int error = 0;
    for (unsigned k = 0; k < 5 && !error; ++k) error = read8(kPmi632, static_cast<uint16_t>(0x3142 + k), &p[k]);
    if (error) return error;
    p[0] = static_cast<uint8_t>((p[0] & ~0x7cu) | 0x28u);   // absolute calibration, decimation 2
    p[1] = static_cast<uint8_t>(p[1] & ~0x07u);              // 1 sample
    p[2] = 0x08;                                            // ADC_USB_IN_V_16
    p[3] = static_cast<uint8_t>((p[3] & ~0x0fu) | 0x02u);   // 200 us settle
    p[4] = static_cast<uint8_t>(p[4] | 0x80u);              // enable
    for (unsigned k = 0; k < 5 && !error; ++k) error = write8(kPmi632, static_cast<uint16_t>(0x3142 + k), p[k]);
    uint8_t channel = 0;
    if (!error) error = read8(kPmi632, 0x3144, &channel);
    if (!error && channel != 0x08) error = SPMI_ERR_IO;
    if (!error) error = write8(kPmi632, 0x3147, 0x80);
    if (error) return error;
    uint8_t status = 0;
    unsigned n = 0;
    for (; n < 40; ++n) {
      IOSleep(1);
      error = read8(kPmi632, 0x3108, &status);
      if (error) return error;
      if ((status & 3u) == 1u) break;
    }
    if (n == 40) return SPMI_ERR_TIMEOUT;
    uint8_t lo = 0, hi = 0;
    error = read8(kPmi632, 0x3150, &lo);
    if (!error) error = read8(kPmi632, 0x3151, &hi);
    if (error) return error;
    const unsigned code = lo | (static_cast<unsigned>(hi) << 8);
    if (code == 0x8000) return SPMI_ERR_PARAM;               // no valid data
    *mv = code > 0x7fff ? 0 : static_cast<unsigned>((static_cast<uint64_t>(code) * 1875u * 16u) / 0x70e4u);
    return 0;
  }
  __attribute__((format(printf, 2, 3))) void log(const char *format, ...) {
    va_list ap;
    va_start(ap, format);
    IOLogv(format, ap);
    va_end(ap);
  }
};
guamp_charge::Controller gCharge;

// One line per 5 s poll when the decoded state changed, else every 30 s: charger state, the STATUS_2/7 bits Linux
// defines, power path (DCDC 0x110b), ICL/AICL status, USBIN interrupt real-time bits, the Type-C status (state machine
// 0x1509, misc 0x150b, sink Rp 0x1506, 0x1508/0x150a/0x150e, legacy 0x150d, interrupt real-time 0x1510) and the gating
// configuration.
void chargerGatingLog(bool force) {
  static const char *const kState[8] = {"INHIBIT", "TRICKLE", "PRECHG", "FULLON", "TAPER", "TERM", "PAUSE", "DISABLED"};
  static uint32_t last = 0xffffffffu;
  uint8_t pp = 0, icl = 0, aiclSt = 0, aicl = 0, jeita = 0, pull = 0, pause = 0, en = 0;
  read8(kPmi632, kPowerPath, &pp); read8(kPmi632, kIclStatus, &icl); read8(kPmi632, kAiclStatus, &aiclSt);
  read8(kPmi632, kAiclOptions, &aicl); read8(kPmi632, kJeitaEnCfg, &jeita); read8(kPmi632, kBatifPullUp, &pull);
  read8(kPmi632, kChgPauseCmd, &pause); read8(kPmi632, kChgEnableCmd, &en);
  const uint8_t s1 = gChg[0], s2 = gChg[1], s7 = gChg[2], usbin = gChg[6];
  const guamp_charge::Snapshot &t = gCharge.snap;
  const uint32_t key = (static_cast<uint32_t>(s1 & 7) << 24) | (static_cast<uint32_t>(s7 & 0x3f) << 16) |
                       (static_cast<uint32_t>(pp & 0x5f) << 8) | (usbin & 0x1fu) | ((s2 & 2u) << 4);
  const uint32_t typecKey = (static_cast<uint32_t>(t.sm) << 8) | (t.misc & 0x1bu);
  static uint32_t lastTypec = 0xffffffffu;
  if (!force && key == last && typecKey == lastTypec) return;
  last = key;
  lastTypec = typecKey;
  IOLog("GuampChg: %s st1 %02x st2 %02x%s st7 %02x%s%s%s%s%s%s pp %02x%s%s%s icl %u mA aicl %02x%s usbin %02x%s%s%s%s%s "
        "typec sm %02x%s%s misc %02x%s%s%s rp %02x src %02x 0a %02x 0e %02x legacy %02x rt %02x mode %02x vbus-adc %u mV "
        "cfg aicl-opt %02x%s jeita %02x pull %02x pause %02x en %02x weak %u redetect %u suspend-toggle %u cc-recover %u\n",
        kState[s1 & 7], s1, s2, (s2 & 0x02) ? " BAT_OV" : "", s7, (s7 & 0x20) ? " HOT_SOFT" : "",
        (s7 & 0x10) ? " COLD_SOFT" : "", (s7 & 0x08) ? " TOO_HOT" : "", (s7 & 0x04) ? " TOO_COLD" : "",
        (s7 & 0x02) ? " HOT_AFP" : "", (s7 & 0x01) ? " COLD_AFP" : "", pp, (pp & 0x40) ? " USBIN_SUSPENDED" : "",
        (pp & 0x10) ? " USE_USBIN" : "", (pp & 0x01) ? " VALID" : " NO_VALID_INPUT", static_cast<unsigned>(icl) * 50u,
        aiclSt, (aiclSt & 0x01) ? " DONE" : "", usbin, (usbin & 0x10) ? " PLUGIN" : "", (usbin & 0x08) ? " OV" : "",
        (usbin & 0x04) ? " UV" : "", (usbin & 0x02) ? " VASHDN" : "", (usbin & 0x01) ? " COLLAPSE" : "", t.sm,
        (t.sm & 0x20) ? " ATTACHED" : "", (t.sm & 0x0f) == 2 ? " WAIT_VBUS" : "", t.misc,
        (t.misc & 0x01) ? " CC_ATTACHED" : "", (t.misc & 0x08) ? " DEBOUNCED" : "", (t.misc & 0x10) ? " VBUS_ERROR" : "",
        t.snk, t.src, t.sm2, t.cc, t.legacy, t.typecRt, gTypecMode, gCharge.lastVbusMv, aicl, (aicl & 0x80) ? " SUSPEND_ON_COLLAPSE" : "",
        jeita, pull, pause, en, gCharge.weakSteps, gCharge.redetects, gCharge.suspendRecoveries, gCharge.ccRecoveries);
}

// --- ringer switch (Assistant key, TLMM GPIO 33) -------------------------------------------------------------------
constexpr uint32_t kRingerBit = 1u << 4;     // report 2 bit 4: Telephony 0x0B / 0x2E (guamp-hid-variant.py)
volatile uint8_t *gAssist;                   // GPIO 33's own 0x1000 block, nothing else of the EAST tile
volatile uint32_t gRingerState, gRingerPresses, gSinkGeneration;

// Maps the pin and checks its configuration; only GPIO 33's cfg is ever written, and only when the boot chain left it
// in another function, as an output or without the pull-up (the key would then read pressed or float).
bool ringerInit() {
  unsigned enabled = 1, start = 1;
  PE_parse_boot_argn("guamp-ringer", &enabled, sizeof(enabled));
  PE_parse_boot_argn("guamp-ringer-start", &start, sizeof(start));
  gRingerState = start ? 1u : 0u;
  if (!enabled) { IOLog("GuampPmic: ringer disabled (guamp-ringer=0)\n"); return false; }
  gAssist = mapPhysical(GP_ASSIST_TLMM_BLOCK, 0x1000);
  if (!gAssist) { IOLog("GuampPmic: ringer GPIO 33 mapping failed\n"); return false; }
  volatile uint32_t *cfg = reinterpret_cast<volatile uint32_t *>(gAssist + GP_ASSIST_CFG);
  const uint32_t before = *cfg;
  if ((before & GP_ASSIST_CFG_MASK) != GP_ASSIST_CFG_INPUT_PU) {
    *cfg = (before & ~GP_ASSIST_CFG_MASK) | GP_ASSIST_CFG_INPUT_PU;
    __asm__ volatile("dsb sy" ::: "memory");
    IODelay(100);                              // let the pull-up charge the line before the first sample
  }
  const uint32_t io = *reinterpret_cast<volatile uint32_t *>(gAssist + GP_ASSIST_IN_OUT);
  IOLog("GuampPmic: ringer GPIO 33 cfg %08x -> %08x, level %u, start %s\n", before, *cfg, io & 1u,
        gRingerState ? "ring" : "silent");
  return true;
}
bool assistPressed() {   // active low
  return gAssist && !(*reinterpret_cast<volatile uint32_t *>(gAssist + GP_ASSIST_IN_OUT) & 1u);
}

void sinkFromHid(void (*sink)(void *, uint32_t), void *context) {
  gSinkGeneration = gSinkGeneration + 1;     // the PMIC thread sends the full state again to a new sink
  gSink = nullptr;
  __asm__ volatile("dmb ish" ::: "memory");
  gSinkContext = context;
  __asm__ volatile("dmb ish" ::: "memory");
  gSink = sink;
  IOLog("GuampPmic: HID button sink %s\n", sink ? "attached" : "detached");
}

void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-pmic");
  gCore = mapPhysical(GP_SPMI_CORE, GP_SPMI_CORE_SIZE);
  gCnfg = mapPhysical(GP_SPMI_CNFG, 0x26000);
  gObs = mapPhysical(GP_SPMI_OBSRVR, GP_SPMI_OBSRVR_SIZE);
  gChn = mapPhysical(GP_SPMI_CHNLS, GP_SPMI_CHNLS_SIZE);
  if (!gCore || !gCnfg || !gObs || !gChn) { IOLog("GuampPmic: mapping failed\n"); return; }
  int r = init();
  IOLog("GuampPmic: SPMI arbiter version %08x, %u APIDs, init %d\n", gArb.version, gArb.apids, r);
  if (r) return;
  __asm__ volatile("dmb ish" ::: "memory");
  gArbReady = 1;
  unsigned battery = 0;
  PE_parse_boot_argn("guamp-battery", &battery, sizeof(battery));
  gBatteryEnabled = battery == 1;
  unsigned batteryTemp = 1, batterySdam = 0;
  PE_parse_boot_argn("guamp-battery-temp", &batteryTemp, sizeof(batteryTemp));
  PE_parse_boot_argn("guamp-battery-sdam", &batterySdam, sizeof(batterySdam));
  gBatteryTempEnabled = batteryTemp != 0;
  gBatterySdamStore = batterySdam == 1;
  IOLog("GuampBattery: QG fuel gauge and power-source publisher %s, BAT_THERM ADC %s, SDAM store %s\n",
        gBatteryEnabled ? "enabled" : "disabled", gBatteryTempEnabled ? "on" : "off", gBatterySdamStore ? "on" : "off");
  unsigned iclMa = 1500, fccMa = 2000;
  PE_parse_boot_argn("guamp-icl-ma", &iclMa, sizeof(iclMa));
  PE_parse_boot_argn("guamp-fcc-ma", &fccMa, sizeof(fccMa));
  if (iclMa > 3000) iclMa = 3000;
  if (fccMa > 3000) fccMa = 3000;
  unsigned typecDrp = 0, chgRecover = 1;
  PE_parse_boot_argn("guamp-typec-drp", &typecDrp, sizeof(typecDrp));
  PE_parse_boot_argn("guamp-chg-recover", &chgRecover, sizeof(chgRecover));
  gCharge.iclMa = iclMa;
  gCharge.fccMa = fccMa;
  gCharge.typecDrp = typecDrp == 1;
  gCharge.recover = chgRecover != 0;
  IOLog("GuampChg: controller ICL %u mA (SDP %u) FCC %u mA FV %u mV, Type-C %s, recovery %s\n", iclMa,
        guamp_charge::kSdpMa, fccMa, guamp_charge::kFvMv, gCharge.typecDrp ? "DRP try.SNK" : "sink only",
        gCharge.recover ? "on" : "off");
  ChargeBus chargeBus;
  uint32_t stable = 0, last = 0, sent = 0;
  const bool ringer = ringerInit();
  bool keyLast = false, keyStable = false;
  unsigned keyCount = 0;
  uint32_t sinkSeen = gSinkGeneration;
  for (unsigned tick = 0;; ++tick) {
    if (ringer) {
      // Momentary key, debounced like the PON keys (the same level on two polls 25 ms apart; Linux uses 15 ms); each
      // press toggles the latched switch position, the release does nothing.
      const bool key = assistPressed();
      keyCount = key == keyLast ? keyCount + 1 : 0;
      keyLast = key;
      if (keyCount == 1 && key != keyStable) {
        keyStable = key;
        if (key) {
          gRingerState = gRingerState ^ 1u;
          gRingerPresses = gRingerPresses + 1;
          IOLog("GuampPmic: ringer %s (assist key press %u, HID sink %s)\n", gRingerState ? "ring" : "silent",
                gRingerPresses, gSink ? "attached" : "absent");
        }
      }
    }
    uint32_t now = readButtons() | (ringer && gRingerState ? kRingerBit : 0u);
    stable = now == last ? stable + 1 : 0;
    last = now;
    if (stable == 1 && now != sent) {             // the same set on two polls in a row
      sent = now;
      gButtons = now;
      if (void (*sink)(void *, uint32_t) = gSink) sink(gSinkContext, now);
      IOLog("GuampPmic: buttons %x, HID sink %s\n", now, gSink ? "attached" : "absent");
    } else if (tick % 40 == 0 || gSinkGeneration != sinkSeen) {
      // The latched ringer level must reach the HID element even when the sink or its event driver came after the last
      // change (NativeHidInput drops reports until its IOHIDEventDriver exists). The same report again is no event:
      // IOHIDEventDriver only dispatches keyboard elements whose value changed.
      sinkSeen = gSinkGeneration;
      if (void (*sink)(void *, uint32_t) = gSink) sink(gSinkContext, sent);
    }
    if (tick % 40 == 0) {                         // every second: the charger controller
      uint64_t ns = 0;
      absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
      gCharge.poll(chargeBus, ns / 1000000u);
      const guamp_charge::Snapshot &c = gCharge.snap;
      gChg[0] = c.s1; gChg[1] = c.s2; gChg[2] = c.s7; gChg[3] = c.apsd; gChg[4] = c.result; gChg[5] = c.fv;
      gChg[6] = c.usbin;
    }
    if (tick % 200 == 0) {                        // every 5 s: status line on change, battery, periodic dumps
      uint8_t typecRt = 0, typecMode = 0;
      read8(kPmi632, 0x1510, &typecRt);
      read8(kPmi632, kTypecModeCfg, &typecMode);
      gChg[7] = typecRt;
      gTypecMode = typecMode;
      chargerGatingLog(tick % 1200 == 0);
      batteryPoll();
      if (tick % 1200 == 0) {
        // Register dump: USBIN 0x1300-0x130f, 0x1340-0x134f, 0x1360-0x136f (ICL/APSD options), 0x1380-0x138f (AICL),
        // TYPEC 0x1500-0x150f and 0x1540-0x154f, MISC 0x1600-0x160f and 0x1640-0x164f, CHGR 0x1090-0x109f (JEITA),
        // BATIF 0x1280-0x128f (thermistor ADC), TYPEC 0x1550-0x157f (exit state, interrupt, debounce, uUSB/water)
        static const uint16_t blocks[13] = {0x1300, 0x1340, 0x1360, 0x1380, 0x1500, 0x1540, 0x1600, 0x1640, 0x1090,
                                            0x1280, 0x1550, 0x1560, 0x1570};
        char text[13 * 33 + 8];
        unsigned at = 0;
        for (unsigned b = 0; b < 13; ++b) {
          for (unsigned k = 0; k < 16; ++k) {
            uint8_t v = 0;
            read8(kPmi632, static_cast<uint16_t>(blocks[b] + k), &v);
            static const char hexd[] = "0123456789abcdef";
            text[at++] = hexd[v >> 4]; text[at++] = hexd[v & 15];
          }
          text[at++] = ' ';
        }
        text[at] = 0;
        IOLog("GuampPmic: regs 1300 1340 1360 1380 1500 1540 1600 1640 1090 1280 1550 1560 1570: %s typec resets %u plug %u unplug %u "
              "errors %u\n", text, gCharge.typecResets, gCharge.plugins, gCharge.unplugs, gCharge.errors);
      }
      if (tick % 1200 == 0) {
        IOLog("GuampUi: text-fixups configured %u applied %u chunks %u file-matches %u byte-mismatches %u; cache-PC cycle %u deleted %u absent %u error %u invalidation-complete %u\n",
              cs_text_fixup_parsed, cs_text_fixup_applied, cs_text_fixup_chunk_calls, cs_text_fixup_name_matches, cs_text_fixup_byte_mismatches, native_guamp_ui_cache_cycle_id, native_guamp_ui_cache_deleted,
              native_guamp_ui_cache_absent, native_guamp_ui_cache_error, native_guamp_ui_cache_completed);
        IOLog("GuampHome: reset %u deleted %u absent %u error %u complete %u\n",
              native_guamp_home_reset_id, native_guamp_home_reset_deleted, native_guamp_home_reset_absent,
              native_guamp_home_reset_error, native_guamp_home_reset_completed);
      }
      if (tick % 1200 == 0)
        IOLog("GuampPmic: ringer %s presses %u\n", gRingerState ? "ring" : "silent", gRingerPresses);
      if (tick % 1200 == 0)
        IOLog("GuampPmic: buttons %x, charger status %02x %02x %02x, APSD %02x result %02x, FV %02x, USBIN %02x TYPEC %02x mode %02x, ICL code %u FCC code %u, SPMI errors %u; touch step %u error %d events %u\n",
              gButtons, gChg[0], gChg[1], gChg[2], gChg[3], gChg[4], gChg[5], gChg[6], gChg[7], gTypecMode, gCharge.iclCode,
              gCharge.fccCode, gErrors,
              touch::gStep, touch::gError, touch::gEvents);
    }
    IOSleep(25);
  }
}

void start() {
  unsigned enabled = 1;
  PE_parse_boot_argn("guamp-pmic", &enabled, sizeof(enabled));
  if (!enabled) return;
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}
}  // namespace pmic

// NativeHidInput registers its button sink here (weak import on its side)
extern "C" void native_buttons_set_sink(void (*sink)(void *, uint32_t), void *context) { pmic::sinkFromHid(sink, context); }
