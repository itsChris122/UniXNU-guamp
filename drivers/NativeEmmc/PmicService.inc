// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp PMIC service, included by NativeEmmc.cpp after UsbMirror.inc (same file for the mapping helper and the boot log).
// One kernel thread on the SPMI PMIC arbiter (GuampSpmi.c logic, GuampPmicBoard.h facts):
//   buttons   Power and Volume Down from the PM6125 PON real-time status, Volume Up from PM6125 GPIO 5, polled every
//             25 ms with a 2-sample debounce, sent to NativeHidInput's button sink (report 2: bit 1 Power, bit 2 Volume
//             Up, bit 3 Volume Down) when the pressed set changes
//   charger   PMI632 SMB5: pets the charger watchdog, enables charging and raises the USB input current limit and the
//             fast charge current once a charger is detected; register offsets from Linux qcom_smbx.c (SMB2/SMB5 share
//             them); the current codes assume 50 mA steps, so a 25 mA hardware step only halves them (never exceeds)
//   ringer    the Assistant key (TLMM GPIO 33, active low) is the ring/silent switch: every debounced press toggles a
//             latched state that starts at "ring"; it is report 2 bit 4 (Telephony page 0x0B usage 0x2E, level: 1 ring,
//             0 silent). backboardd BKButtonHIDEventProcessor -processEvent:sender:dispatcher: routes a keyboard event
//             with page 0x0B usage 0x2E to -_processRingerSwitchEvent:down: (down != 0 -> "tone1", 0 -> "silent") and
//             SpringBoard -__handleHIDEvent: 0xB002E -> -_handleRingerSwitchEvent: (gated by MobileGestalt ringer-switch
//             = ADT /buttons property function-button_ringerab, see make-guamp-adt.py) -> _updateRingerState:.
// Boot arguments: guamp-pmic=0 turns it off; guamp-icl-ma (default 1500) and guamp-fcc-ma (default 2000);
// guamp-ringer=0 leaves GPIO 33 alone, guamp-ringer-start=0 starts at "silent".
// Every 30 s one log line: buttons, charger status bytes, APSD result, the codes written, SPMI errors.

#include "GuampSpmi.h"
#include "GuampPmicBoard.h"

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
// SMB5 charger (PMI632 0x1000 block, offsets of qcom_smbx.c relative to it)
constexpr uint16_t kChgStatus1 = 0x1006, kChgStatus2 = 0x1007, kChgStatus7 = 0x100d, kChgEnableCmd = 0x1042,
                   kFccCfg = 0x1061, kApsdStatus = 0x1307, kApsdResult = 0x1308, kUsbinCmdIl = 0x1340,
                   kIclOptions = 0x1366, kIclCfg = 0x1370, kWdogPet = 0x1643, kFvCfg = 0x1070;
// Float voltage: left at the hardware default (low) without a driver, which held the charger in INHIBIT (status 6) with a
// charged battery. SMB5 code = (mV - 3600) / 10; on the SMB2 encoding (3487.5 + 7.5 mV per code) the same code is lower.
constexpr unsigned kFvMv = 4350;
// Type-C mode (SMB5 TYPEC block 0x1500, TYPE_C_MODE_CFG at +0x44: bit 0 disable command, bit 1 sink only, bit 2 source
// only). A Type-C wall charger only turns VBUS on after it sees the sink's Rd on CC: on the wall charger USBIN showed
// under-voltage and TYPEC no attach, while the legacy A-to-C cable of the PC always carries VBUS. Sink only: the phone
// never supplies power here.
constexpr uint16_t kTypecModeCfg = 0x1544;
constexpr uint8_t kTypecSinkOnly = 0x10;   // value Linux (TWRP smb5) keeps: try-sink DRP; the UEFI left 0x02
// 0x154a: Linux 0x03, UEFI left 0x0f (after the first unplug the Type-C state machine never attached again)
constexpr uint16_t kTypecCfg4a = 0x154a;
constexpr uint8_t kTypecCfg4aLinux = 0x03;

// Charge gating registers that Linux configures and the UEFI does not (smb5_init_hw in qpnp-smb5.c, mmi_init in
// smb5-lib.c, guamp DT bengal-guamp-common-overlay.dtsi &pmi632_charger; values checked against the TWRP dump
// analisi/pmic/twrp-pc.txt where Linux charges). BATTERY_CHARGER_STATUS_1 bits 2:0 = 6 is PAUSE_CHARGE (smb5-reg.h:
// 0 inhibit .. 6 pause, 7 disable). It came with USBIN collapse/UV/VASHDN (0x1310 = 0x17/0x37) and the battery carrying
// the whole system load: the input was suspended on collapse.
//   0x1380 USBIN_AICL_OPTIONS_CFG mask 0x9c: AICL on, periodic AICL rerun on, ADC AICL off, SUSPEND_ON_COLLAPSE off
//          (DT qcom,disable-suspend-on-collapse) -> 0x14 under the mask; TWRP 0x54
//   0x1661 AICL_RERUN_TIME_CFG = 0x01 (12 s); TWRP 0x01
//   0x1286 BATIF_ADC_INTERNAL_PULL_UP bits 1:0 = 01, 30 kOhm for the 10 kOhm NTC (guamp CONFIG_BAT_NTC_10K=y): the
//          hardware JEITA comparators read 28 C as HOT_SOFT (STATUS_7 0x20) without it; TWRP 0x01, STATUS_7 0x00
//   0x1090 JEITA_EN_CFG bits 3:0 = 0, no hardware soft-JEITA FCC/FV compensation (smblib_disable_hw_jeita); the
//          hard-limit bits stay as the hardware has them (Motorola clears them too, but only because its mmi code does
//          software JEITA, which this port does not); TWRP 0x00
//   0x1043 CHARGING_PAUSE_CMD = 0; TWRP 0x00
constexpr uint16_t kAiclOptions = 0x1380, kAiclRerunTime = 0x1661, kBatifPullUp = 0x1286, kJeitaEnCfg = 0x1090,
                   kChgPauseCmd = 0x1043, kIclStatus = 0x1107, kAiclStatus = 0x110a, kPowerPath = 0x110b;

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

// Read-modify-write of the bits in mask only; no write when they already hold the value. Returns the old value.
uint8_t smbMasked(uint16_t addr, uint8_t mask, uint8_t value) {
  uint8_t old = 0;
  if (read8(kPmi632, addr, &old)) { gErrors = gErrors + 1; return 0; }
  const uint8_t want = static_cast<uint8_t>((old & ~mask) | (value & mask));
  if (want != old && write8(kPmi632, addr, want)) gErrors = gErrors + 1;
  return old;
}
void chargerLinuxConfig(const char *when) {
  const uint8_t aicl = smbMasked(kAiclOptions, 0x9c, 0x14);
  const uint8_t rerun = smbMasked(kAiclRerunTime, 0xff, 0x01);
  const uint8_t pull = smbMasked(kBatifPullUp, 0x03, 0x01);
  const uint8_t jeita = smbMasked(kJeitaEnCfg, 0x0f, 0x00);
  const uint8_t pause = smbMasked(kChgPauseCmd, 0x01, 0x00);
  IOLog("GuampChg: Linux config (%s), before: AICL-opt %02x rerun %02x therm-pull %02x JEITA-en %02x pause-cmd %02x\n",
        when, aicl, rerun, pull, jeita, pause);
}

// One line per 5 s poll when the decoded state changed, else every 30 s: charger state, the STATUS_2/7 bits Linux
// defines, power path (DCDC 0x110b), ICL/AICL status, USBIN interrupt real-time bits and the gating configuration.
void chargerGatingLog(bool force, unsigned recoveries) {
  static const char *const kState[8] = {"INHIBIT", "TRICKLE", "PRECHG", "FULLON", "TAPER", "TERM", "PAUSE", "DISABLED"};
  static uint32_t last = 0xffffffffu;
  uint8_t pp = 0, icl = 0, aiclSt = 0, aicl = 0, jeita = 0, pull = 0, pause = 0, en = 0;
  read8(kPmi632, kPowerPath, &pp); read8(kPmi632, kIclStatus, &icl); read8(kPmi632, kAiclStatus, &aiclSt);
  read8(kPmi632, kAiclOptions, &aicl); read8(kPmi632, kJeitaEnCfg, &jeita); read8(kPmi632, kBatifPullUp, &pull);
  read8(kPmi632, kChgPauseCmd, &pause); read8(kPmi632, kChgEnableCmd, &en);
  const uint8_t s1 = gChg[0], s2 = gChg[1], s7 = gChg[2], usbin = gChg[6];
  const uint32_t key = (static_cast<uint32_t>(s1 & 7) << 24) | (static_cast<uint32_t>(s7 & 0x3f) << 16) |
                       (static_cast<uint32_t>(pp & 0x5f) << 8) | (usbin & 0x1fu) | ((s2 & 2u) << 4);
  if (!force && key == last) return;
  last = key;
  IOLog("GuampChg: %s st1 %02x st2 %02x%s st7 %02x%s%s%s%s%s%s pp %02x%s%s%s icl %u mA aicl %02x%s usbin %02x%s%s%s%s%s "
        "cfg aicl-opt %02x%s jeita %02x pull %02x pause %02x en %02x recover %u\n",
        kState[s1 & 7], s1, s2, (s2 & 0x02) ? " BAT_OV" : "", s7, (s7 & 0x20) ? " HOT_SOFT" : "",
        (s7 & 0x10) ? " COLD_SOFT" : "", (s7 & 0x08) ? " TOO_HOT" : "", (s7 & 0x04) ? " TOO_COLD" : "",
        (s7 & 0x02) ? " HOT_AFP" : "", (s7 & 0x01) ? " COLD_AFP" : "", pp, (pp & 0x40) ? " USBIN_SUSPENDED" : "",
        (pp & 0x10) ? " USE_USBIN" : "", (pp & 0x01) ? " VALID" : " NO_VALID_INPUT", static_cast<unsigned>(icl) * 50u,
        aiclSt, (aiclSt & 0x01) ? " DONE" : "", usbin, (usbin & 0x10) ? " PLUGIN" : "", (usbin & 0x08) ? " OV" : "",
        (usbin & 0x04) ? " UV" : "", (usbin & 0x02) ? " VASHDN" : "", (usbin & 0x01) ? " COLLAPSE" : "", aicl,
        (aicl & 0x80) ? " SUSPEND_ON_COLLAPSE" : "", jeita, pull, pause, en, recoveries);
}

void charger(bool configure, unsigned iclMa, unsigned fccMa, uint8_t *iclCode, uint8_t *fccCode) {
  write8(kPmi632, kWdogPet, 1);
  uint8_t s1 = 0, s2 = 0, s7 = 0, apsd = 0, result = 0;
  read8(kPmi632, kChgStatus1, &s1); read8(kPmi632, kChgStatus2, &s2); read8(kPmi632, kChgStatus7, &s7);
  uint8_t fv = 0;
  read8(kPmi632, kApsdStatus, &apsd); read8(kPmi632, kApsdResult, &result); read8(kPmi632, kFvCfg, &fv);
  gChg[0] = s1; gChg[1] = s2; gChg[2] = s7; gChg[3] = apsd; gChg[4] = result; gChg[5] = fv;
  if (!configure) return;
  // Input current by source (APSD_RESULT_STATUS of qcom_smbx.c: bit 0 SDP, 1 CDP, 2 DCP, 3 OCP, 4 float): a PC port (SDP)
  // or an unknown source gets 500 mA; asking 1.5 A of a PC port collapsed VBUS (USBIN under-voltage) and the charger
  // stayed DISABLED until the next plug-in.
  if (!(result & 0x0e)) iclMa = iclMa < 500 ? iclMa : 500;
  chargerLinuxConfig("plug-in");
  *iclCode = static_cast<uint8_t>(iclMa / 50 > 0xff ? 0xff : iclMa / 50);
  *fccCode = static_cast<uint8_t>(fccMa / 50 > 0xff ? 0xff : fccMa / 50);
  write8(kPmi632, kFvCfg, static_cast<uint8_t>((kFvMv - 3600) / 10));
  write8(kPmi632, kChgEnableCmd, 0);       // off and on again: the charger re-evaluates inhibit with the new float voltage
  IODelay(1000);
  write8(kPmi632, kChgEnableCmd, 1);       // charging enable command
  write8(kPmi632, kFccCfg, *fccCode);
  write8(kPmi632, kIclCfg, *iclCode);
  write8(kPmi632, kIclOptions, 0x03);      // value Linux (TWRP) keeps
  write8(kPmi632, kUsbinCmdIl, 0x00);      // input not suspended
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
  uint32_t stable = 0, last = 0, sent = 0;
  uint8_t iclCode = 0, fccCode = 0;
  bool configured = false, apsdRequested = false;
  unsigned absentTicks = 0, typecResets = 0, suspendRecoveries = 0, suspendHold = 0;
  chargerLinuxConfig("boot");
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
    if (tick % 200 == 0) {                        // every 5 s: watchdog pet, status; configure once a charger is seen
      // Input present = USBIN plug-in real-time status (0x1310 bit 4), not the APSD result: after a cable change the
      // APSD result stayed 0 and the charger DISABLED. On every new plug-in: unsuspend the input, rerun APSD, configure.
      uint8_t usbinRt = 0, typecRt = 0, typecMode = 0;
      read8(kPmi632, kTypecModeCfg, &typecMode);
      if (typecMode != kTypecSinkOnly) write8(kPmi632, kTypecModeCfg, kTypecSinkOnly);
      uint8_t cfg4a = 0;
      read8(kPmi632, kTypecCfg4a, &cfg4a);
      if (cfg4a != kTypecCfg4aLinux) write8(kPmi632, kTypecCfg4a, kTypecCfg4aLinux);
      gTypecMode = typecMode;
      read8(kPmi632, 0x1310, &usbinRt);
      read8(kPmi632, 0x1510, &typecRt);
      gChg[6] = usbinRt; gChg[7] = typecRt;
      bool present = (usbinRt & 0x10) != 0;
      // After the first unplug the PMIC never saw VBUS again (USBIN under-voltage, TYPEC no attach), not even on the PC
      // port that carried the USB data: try a Type-C state machine reset (disable command, then sink only again) every
      // 15 s while nothing is detected.
      if (!present && ++absentTicks % 3 == 0) {
        write8(kPmi632, kTypecModeCfg, kTypecSinkOnly | 0x01);
        IOSleep(100);
        write8(kPmi632, kTypecModeCfg, kTypecSinkOnly);
        typecResets = typecResets + 1;
      }
      if (present) absentTicks = 0;
      if (present && !configured && !apsdRequested) {
        write8(kPmi632, kUsbinCmdIl, 0x00);
        write8(kPmi632, 0x1341, 0x01);         // CMD_APSD: rerun source detection, configure on a later tick
        apsdRequested = true;
      }
      bool apsdDone = (gChg[3] & 0x01) != 0;
      charger(present && !configured && apsdRequested && apsdDone, iclMa, fccMa, &iclCode, &fccCode);
      if (present && !configured && apsdRequested && apsdDone) configured = true;
      if (!present) { configured = false; apsdRequested = false; }
      // An input already suspended on collapse (before the AICL configuration above, e.g. by the boot chain) stays
      // suspended until the suspend command toggles, which is what Linux's usb_icl vote 0 -> ICL does: at most once per
      // 30 s, only with the cable in and our own suspend command off.
      uint8_t powerPath = 0, cmdIl = 0;
      if (suspendHold) suspendHold = suspendHold - 1;
      if (present && !suspendHold && !read8(kPmi632, kPowerPath, &powerPath) && (powerPath & 0x40) &&
          !read8(kPmi632, kUsbinCmdIl, &cmdIl) && !(cmdIl & 0x01)) {
        write8(kPmi632, kUsbinCmdIl, 0x01);
        IOSleep(20);                               // VASHDN deglitch, as usbin_uv_irq_handler
        write8(kPmi632, kUsbinCmdIl, 0x00);
        suspendRecoveries = suspendRecoveries + 1;
        suspendHold = 6;
      }
      chargerGatingLog(tick % 1200 == 0, suspendRecoveries);
      batteryPoll();
      if (tick % 1200 == 0) {
        // Register dump: USBIN 0x1300-0x130f and 0x1340-0x134f, TYPEC 0x1500-0x150f and 0x1540-0x154f, MISC 0x1600-0x160f
        static const uint16_t blocks[5] = {0x1300, 0x1340, 0x1500, 0x1540, 0x1600};
        char text[5 * 33 + 8];
        unsigned at = 0;
        for (unsigned b = 0; b < 5; ++b) {
          for (unsigned k = 0; k < 16; ++k) {
            uint8_t v = 0;
            read8(kPmi632, static_cast<uint16_t>(blocks[b] + k), &v);
            static const char hexd[] = "0123456789abcdef";
            text[at++] = hexd[v >> 4]; text[at++] = hexd[v & 15];
          }
          text[at++] = ' ';
        }
        text[at] = 0;
        IOLog("GuampPmic: regs 1300 1340 1500 1540 1600: %s typec resets %u\n", text, typecResets);
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
              gButtons, gChg[0], gChg[1], gChg[2], gChg[3], gChg[4], gChg[5], gChg[6], gChg[7], gTypecMode, iclCode, fccCode, gErrors,
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
