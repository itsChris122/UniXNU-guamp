// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp camera flash LED as a torch. Included by PmicService.inc inside namespace pmic (uses read8/write8 and the PMIC
// thread). Opt-in: guamp-torch=1 (default off); guamp-torch-ma = current at level 1.0 (default 100, 25..200),
// guamp-torch-sec = auto-off (default 600, max 1800). No button drives it: requests come through guamp_torch_request()
// (level in permille, 0 = off) from any thread and are applied by the PMIC thread within one 25 ms poll.
// IOKit endpoint: service GuampTorch (class GuampTorchService, child of the platform expert). A root process sets
// "TorchLevel" (integer permille, 0 = off) or "TorchOn" (boolean) with IORegistryEntrySetCFProperty; the service shows
// "TorchCurrentMA", "TorchMaxMA" and "TorchAutoOffSeconds". iOS itself does not use it: its flashlight goes through
// mediaserverd FigFlashlight -> H9ISP.mediacapture -> the ISP firmware of AppleH9CamIn.
//
// Hardware (guamp boot_b DTB ~/moto/guamp.dts = pmi632.dtsi; bengal-camera-sensor-mot-guamp-overlay.dtsi):
//   flash_led qcom,leds@d300 "qcom,qpnp-flash-led-v2" under qcom,pmi632@3, i.e. SPMI slave 3 (the charger is slave 2)
//   torch_0: qcom,id 0 (first LED channel), max-current 500, current-ma 300, ires-ua 12500, hdrm-voltage-mv 400,
//            hdrm-vol-hi-lo-win-mv 100, no duration-ms (a torch runs with the safety timer disabled)
//   led_switch_0 led-mask 3; camera-flash@0/@1 use torch-source pmi632_torch0 + switch-source pmi632_switch0
// Register use, from a study of drivers/leds/leds-qpnp-flash-v2.c (no code copied), offsets from 0xd300:
//   0x05 PERPH_SUBTYPE (PMI632 flash = 5)   0x08/0x09 LED_STATUS1/2   0x10 INT_RT_STS (bit 0 led-fault)
//   0x40 SAFETY_TMR (0x13 = disabled)       0x43 TGR_CURRENT bits 6:0  0x46 MOD_CTRL bit 7 module enable
//   0x47 IRES bits 1:0 (0 = 12.5 mA step)   0x49 STROBE_CTRL bit 2 (0 = software strobe)
//   0x4c EN_LED_CTRL bit 0                  0x4d HDRM_PRGM ((400-125)/25 << 4 | 100/25 = 0xb4)   0x50 HDRM auto mode
//   switch on (qpnp_flash_led_switch_set): IRES, STROBE_CTRL, TGR_CURRENT, SAFETY_TMR, MOD_CTRL enable, then on PMI632
//   wait for the charger's flash regulator VREG_OK (qpnp_flash_poll_vreg_ok -> schgm-flash.c: SCHGM_FLASH STATUS_2
//   0xa607 bit 4 on slave 2, read only here; STATUS_3 0xa608 bits 2:0 = 7 is a flash error), 60 x 5 ms, then
//   EN_LED_CTRL. Switch off (qpnp_flash_led_switch_disable): EN_LED_CTRL, MOD_CTRL, TGR_CURRENT 0, SAFETY_TMR 0x13.
//   HDRM_PRGM is the probe-time headroom value, written before every switch on.
// Not done: POWER_SUPPLY_PROP_FLASH_ACTIVE (charger side: forces 5 V from an HVDCP adapter), thermal derate, VPH droop,
// mitigation, ISC/warm-up delays and ramp rates stay at the PMIC defaults (all of them limit or protect a 1 A flash; a
// 100 mA torch is far from them). Every write goes through wr(), which can only address slave 3, 0xd300-0xd3ff.
// While on: LED fault (INT_RT_STS bit 0) or flash error checked every second -> off. Off at every haltRestart
// (restart, power off, panic) through guamp_emmc_flush.
#include "GuampTorchCore.h"

namespace torch {
constexpr unsigned kSid = GP_SID_PMI632_B;      // 3
constexpr uint16_t kBase = 0xd300;
constexpr uint16_t kSchgmStatus2 = 0xa607, kSchgmStatus3 = 0xa608, kSchgmTorchPriority = 0xa663;   // slave 2, read only
enum : uint8_t {
  kSubtype = 0x05, kStatus1 = 0x08, kStatus2 = 0x09, kRtSts = 0x10, kSafetyTmr = 0x40, kTgrCurrent = 0x43,
  kModCtrl = 0x46, kIres = 0x47, kStrobeCtrl = 0x49, kEnLedCtrl = 0x4c, kHdrmPrgm = 0x4d, kHdrmAuto = 0x50,
};
constexpr uint8_t kSubtypePmi632 = 5, kSafetyTmrDisabled = 0x13, kHdrm = 0xb4;

volatile uint32_t gReady, gOn, gMa, gRequest, gRequestSeq, gSwitches, gFaults;
uint32_t gSeenSeq;
unsigned gMaxMa = guamp_torch::kDefaultMa, gTimeoutS = guamp_torch::kDefaultTimeoutS;
uint64_t gOnMs;

void publishState();   // defined after GuampTorchService

uint64_t nowMs() {
  uint64_t ns = 0;
  absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
  return ns / 1000000u;
}
int rd(uint8_t off, uint8_t *v) { return read8(kSid, static_cast<uint16_t>(kBase + off), v); }
// The only write path of this file: slave 3, 0xd300 + an 8-bit offset, i.e. the flash LED peripheral and nothing else
int wr(uint8_t off, uint8_t v) { return write8(kSid, static_cast<uint16_t>(kBase + off), v); }
int update(uint8_t off, uint8_t mask, uint8_t v) {
  uint8_t cur = 0;
  int r = rd(off, &cur);
  if (r) return r;
  const uint8_t next = static_cast<uint8_t>((cur & ~mask) | (v & mask));
  return next == cur ? 0 : wr(off, next);
}

// Best effort: every step runs even when an earlier one failed; returns the first error.
int switchOff() {
  int e = update(kEnLedCtrl, 0x03, 0);
  int r = update(kModCtrl, 0x80, 0);
  if (!e) e = r;
  r = update(kTgrCurrent, 0x7f, 0);
  if (!e) e = r;
  r = wr(kSafetyTmr, kSafetyTmrDisabled);
  if (!e) e = r;
  gOn = 0;
  gMa = 0;
  return e;
}

void off(const char *why) {
  if (!gReady || !gOn) return;
  const uint64_t heldS = (nowMs() - gOnMs) / 1000u;
  const int e = switchOff();
  uint8_t en = 0, mod = 0, rt = 0;
  rd(kEnLedCtrl, &en); rd(kModCtrl, &mod); rd(kRtSts, &rt);
  gSwitches = gSwitches + 1;
  IOLog("GuampTorch: off (%s) after %llu s, en %02x mod %02x rt %02x, error %d\n", why,
        static_cast<unsigned long long>(heldS), en, mod, rt, e);
  publishState();
}

// Switches on at ma, or only changes the current when already on (TGR_CURRENT is the level; Linux rewrites it the same way
// for a torch level change).
void on(unsigned ma, const char *why) {
  if (!gReady) return;
  const uint8_t code = guamp_torch::codeForMa(ma);
  const unsigned real = guamp_torch::maForCode(code);
  if (gOn) {
    if (real == gMa) return;
    const int e = update(kTgrCurrent, 0x7f, code);
    if (e) { off("level change failed"); return; }
    IOLog("GuampTorch: on, level %u -> %u mA (code %u, %s)\n", gMa, real, code, why);
    gMa = real;
    publishState();
    return;
  }
  int e = wr(kHdrmPrgm, kHdrm);
  if (!e) e = update(kIres, 0x03, 0);              // first LED: 12.5 mA steps
  if (!e) e = update(kStrobeCtrl, 0x04, 0);        // software strobe
  if (!e) e = update(kTgrCurrent, 0x7f, code);
  if (!e) e = wr(kSafetyTmr, kSafetyTmrDisabled);  // torch: no safety timer
  if (!e) e = update(kModCtrl, 0x80, 0x80);
  uint8_t vreg = 0, state = 0;
  unsigned waited = 0;
  bool ok = false;
  for (; !e && waited < 60; ++waited) {
    IOSleep(5);
    e = read8(kPmi632, kSchgmStatus2, &vreg);
    if (!e && (vreg & 0x10u)) { ok = true; break; }
    if (!e) e = read8(kPmi632, kSchgmStatus3, &state);
    if (!e && (state & 7u) == 7u) break;
  }
  if (ok) e = update(kEnLedCtrl, 0x03, 0x01);
  if (!ok || e) {
    switchOff();
    IOLog("GuampTorch: on failed (%s): error %d, VREG_OK %s after %u ms, schgm status2 %02x status3 %02x\n", why, e,
          ok ? "yes" : "no", (waited + 1) * 5, vreg, state);
    publishState();
    return;
  }
  gOnMs = nowMs();
  gMa = real;
  gOn = 1;
  gSwitches = gSwitches + 1;
  uint8_t en = 0, mod = 0, cur = 0, rt = 0;
  rd(kEnLedCtrl, &en); rd(kModCtrl, &mod); rd(kTgrCurrent, &cur); rd(kRtSts, &rt);
  IOLog("GuampTorch: on %u mA (code %u, %s), VREG_OK after %u ms, en %02x mod %02x current %02x rt %02x, auto-off %u s\n",
        real, code, why, (waited + 1) * 5, en, mod, cur, rt, gTimeoutS);
  publishState();
}

// Any thread: the newest request wins; the PMIC thread applies it.
void request(uint32_t permille) {
  gRequest = permille;
  __asm__ volatile("dmb ish" ::: "memory");
  gRequestSeq = gRequestSeq + 1;
}

// PMIC thread, every 25 ms poll: applies the newest request, the auto-off timer, and every second the fault check.
void poll(unsigned tick) {
  if (!gReady) return;
  const uint32_t seq = gRequestSeq;
  if (seq != gSeenSeq) {
    gSeenSeq = seq;
    __asm__ volatile("dmb ish" ::: "memory");
    const unsigned ma = guamp_torch::maForPermille(gRequest, gMaxMa);
    if (ma) on(ma, "request");
    else off("request");
  }
  if (!gOn) return;
  if (guamp_torch::expired(gOnMs, nowMs(), gTimeoutS)) { off("auto-off timer"); return; }
  if (tick % 40 == 0) {
    uint8_t rt = 0, s3 = 0;
    if (!rd(kRtSts, &rt) && (rt & 1u)) { gFaults = gFaults + 1; off("LED fault"); return; }
    if (!read8(kPmi632, kSchgmStatus3, &s3) && (s3 & 7u) == 7u) { gFaults = gFaults + 1; off("flash regulator error"); }
  }
}

// haltRestart (guamp_emmc_flush), any thread, possibly in a panic: straight to the registers, no sleep, no log in a panic.
void shutdown(bool panic) {
  if (!gReady) return;
  const bool wasOn = gOn;
  switchOff();                                   // also when a switch on was still waiting for VREG_OK
  if (!panic && wasOn) IOLog("GuampTorch: off (restart/power off)\n");
}
}  // namespace torch

// IOKit endpoint (setProperties from user space). The registry properties are refreshed by the PMIC thread.
class GuampTorchService : public IOService {
  OSDeclareDefaultStructors(GuampTorchService)
public:
  void number(const char *key, uint64_t value) { setProperty(key, value, 32); }
  IOReturn setProperties(OSObject *properties) override {
    // Accept a dictionary {TorchLevel|TorchOn} (IORegistryEntrySetCFProperty) or a bare OSNumber/OSBoolean
    // (IORegistryEntrySetCFProperties with a CFNumber/CFBoolean), so a userland hook can drive the torch
    // without building a dictionary or a key CFString.
    OSNumber *level = OSDynamicCast(OSNumber, properties);
    OSBoolean *onOff = OSDynamicCast(OSBoolean, properties);
    if (OSDictionary *dict = OSDynamicCast(OSDictionary, properties)) {
      level = OSDynamicCast(OSNumber, dict->getObject("TorchLevel"));
      onOff = OSDynamicCast(OSBoolean, dict->getObject("TorchOn"));
    }
    if (!level && !onOff) return kIOReturnUnsupported;
    // SpringBoard (the flashlight UI) and mediaserverd both run as user mobile, not root, so the former
    // administrator check is dropped; the service is published only with guamp-torch=1. To keep a gate,
    // allow just the console user, e.g.:
    //   if (proc_getuid(current_proc()) != 501 &&
    //       IOUserClient::clientHasPrivilege(current_task(), kIOClientPrivilegeAdministrator) != kIOReturnSuccess)
    //     return kIOReturnNotPrivileged;
    torch::request(level ? level->unsigned32BitValue() : onOff->isTrue() ? 1000u : 0u);
    return kIOReturnSuccess;
  }
};
OSDefineMetaClassAndStructors(GuampTorchService, IOService)

namespace torch {
GuampTorchService *gService;

// PMIC thread only (never from shutdown(): no registry work in a panic)
void publishState() {
  if (GuampTorchService *service = gService) {
    service->number("TorchCurrentMA", gMa);
    service->setProperty("TorchOn", gOn ? kOSBooleanTrue : kOSBooleanFalse);
  }
}

void publish() {
  IOService *platform = IOService::getPlatform();
  GuampTorchService *made = OSTypeAlloc(GuampTorchService);
  if (!platform || !made || !made->init()) {
    if (made) made->release();
    IOLog("GuampTorch: IOKit service not created\n");
    return;
  }
  made->setName("GuampTorch");
  made->number("TorchMaxMA", gMaxMa);
  made->number("TorchAutoOffSeconds", gTimeoutS);
  made->number("TorchCurrentMA", 0);
  made->setProperty("TorchOn", kOSBooleanFalse);
  if (!made->attach(platform)) { made->release(); IOLog("GuampTorch: IOKit service not attached\n"); return; }
  if (!made->start(platform)) {
    made->detach(platform);
    made->release();
    IOLog("GuampTorch: IOKit service not started\n");
    return;
  }
  made->registerService();
  gService = made;
}

// PMIC thread, once after the arbiter is up: boot arguments, identification, one readback line; switches off a torch
// found on. Returns whether the service runs.
bool init() {
  unsigned enabled = 0, ma = guamp_torch::kDefaultMa, sec = guamp_torch::kDefaultTimeoutS;
  PE_parse_boot_argn("guamp-torch", &enabled, sizeof(enabled));
  if (enabled != 1) { IOLog("GuampTorch: disabled (guamp-torch=1 enables it)\n"); return false; }
  PE_parse_boot_argn("guamp-torch-ma", &ma, sizeof(ma));
  PE_parse_boot_argn("guamp-torch-sec", &sec, sizeof(sec));
  gMaxMa = guamp_torch::maForCode(guamp_torch::codeForMa(ma));
  gTimeoutS = guamp_torch::clampTimeoutS(sec);
  const int apid = apidOf(kSid, kBase);
  static const uint8_t regs[] = {kSubtype, kStatus1, kStatus2, kRtSts, kSafetyTmr, kTgrCurrent, kModCtrl, kIres,
                                 kStrobeCtrl, kEnLedCtrl, kHdrmPrgm, kHdrmAuto};
  uint8_t v[sizeof(regs)] = {};
  int e = 0;
  for (unsigned k = 0; k < sizeof(regs) && !e; ++k) e = rd(regs[k], &v[k]);
  uint8_t s2 = 0, s3 = 0, prio = 0;
  read8(kPmi632, kSchgmStatus2, &s2); read8(kPmi632, kSchgmStatus3, &s3); read8(kPmi632, kSchgmTorchPriority, &prio);
  IOLog("GuampTorch: init PMI632 slave %u 0xd300 apid %d owner %d read %d: subtype %02x status %02x %02x rt %02x "
        "safety %02x current %02x mod %02x ires %02x strobe %02x en %02x hdrm %02x hdrm-auto %02x; schgm status2 %02x "
        "status3 %02x torch-priority %02x; max %u mA (code %u), auto-off %u s\n",
        kSid, apid, apid >= 0 ? gArb.write_owner[apid] : -1, e, v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8],
        v[9], v[10], v[11], s2, s3, prio, gMaxMa, guamp_torch::codeForMa(gMaxMa), gTimeoutS);
  if (e || apid < 0 || gArb.write_owner[apid] != 0 || v[0] != kSubtypePmi632) {
    IOLog("GuampTorch: disabled (flash peripheral not readable/writable or not PMI632)\n");
    return false;
  }
  if ((v[9] & 0x03u) || (v[6] & 0x80u)) {
    const int r = switchOff();
    IOLog("GuampTorch: off (found on at init), error %d\n", r);
  }
  __asm__ volatile("dmb ish" ::: "memory");
  gReady = 1;
  publish();
  return true;
}

}  // namespace torch
