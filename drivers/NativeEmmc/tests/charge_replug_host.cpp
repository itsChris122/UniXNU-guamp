// Host test of the guamp charger controller (GuampChargeReplugCore.h) against a simulated PMI632 SMB5: boot with the
// cable in, unplug, Type-C reset while absent, replug (DCP, SDP, APSD that never finishes), the PAUSE/collapse state
// of the XNU logs (weak-adapter AICL steps, 100 mA, re-detection), suspend recovery, thermal/BAT_OV guards, SPMI
// errors, JEITA thresholds, the input current per source at a runtime attach (wall chargers: Type-C Rp 1.5/3.0 A on a
// C-to-C cable, DCP/QC on a USB-A port, Rp changes, guamp-icl-ma ceiling, measured values 15 s later), and the
// voltage/current ceilings on every single register write.
// Build: run-charge-tests.sh (clang++/g++ -std=gnu++20 -Wall -Wextra -Werror -I.. charge_replug_host.cpp)
#include <GuampChargeReplugCore.h>   // angle brackets: the -I copy (drivers/NativeEmmc)
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
using namespace guamp_charge;

struct Pmi632 {
  uint8_t r[0x10000] = {};
  uint64_t now = 0;
  int failAddr = -1, failCount = 0;
  // PC that shows Rp but keeps VBUS off: CC off -> state 00, CC on -> unattached (01) for 1.5 s, then waiting for VBUS
  // with VBUS error (02 / 18) again, as in the XNU log after a C-to-C replug
  bool pcStuck = false;
  uint64_t stuckAt = 0;
  unsigned vbus = 0, adcReads = 0;
  int ibat = 0;
  bool autoApsd = true;
  uint8_t pendingResult = 0;
  uint64_t apsdAt = 0;   // 0: none pending
  unsigned apsdReruns = 0, writes = 0, aiclRestarts = 0, typecDisablePulses = 0, enableOff = 0;
  std::vector<std::string> logs;
  std::vector<std::pair<uint16_t, uint8_t>> trace;

  Pmi632() {
    // UEFI leftovers seen under XNU: Type-C 0x154a 0x0f, mode 0x10, BC1.2 start on CC, JEITA hard limit on, UEFI
    // JEITA codes for another NTC, pull-up 100 kOhm, AICL thresholds raised
    r[reg::kTypecDebugAccessSink] = 0x0f;
    r[reg::kTypecModeCfg] = 0x10;
    r[reg::kTypeCCfg] = 0xc0;
    r[reg::kJeitaEn] = 0x1f;
    for (unsigned k = 0; k < 12; k = k + 1) r[reg::kJeitaThresholds + k] = static_cast<uint8_t>(0x20 + k);
    r[reg::kBatifPullUp] = 0x02;
    r[reg::kAiclOptions] = 0xd4;
    r[reg::kAicl5v] = 0x06;
    r[reg::kAiclCont] = 0x05;
    r[reg::kUsbinOptions1] = 0x44;
    r[reg::kStatus1] = 0x07;
    r[reg::kStatus7] = 0x20;
  }
  // --- Bus ---
  int rd(uint16_t a, uint8_t *v) {
    if (a == failAddr && failCount > 0) { failCount = failCount - 1; return -5; }
    *v = r[a];
    return 0;
  }
  int wr(uint16_t a, uint8_t v) {
    if (a == failAddr && failCount > 0) { failCount = failCount - 1; return -5; }
    writes = writes + 1;
    trace.push_back({a, v});
    checkLimits(a, v);
    if (a == reg::kCmdApsd && (v & 1)) { apsdReruns = apsdReruns + 1; startApsd(); return 0; }   // command, self-clearing
    if (a == reg::kAiclCmd) { if (v & 2) aiclRestarts = aiclRestarts + 1; return 0; }
    if (a == reg::kWdogPet) return 0;
    if (a == reg::kTypecModeCfg && (v & 1)) typecDisablePulses = typecDisablePulses + 1;
    if (a == reg::kTypecModeCfg && pcStuck) {
      if (v & 1) { r[reg::kTypecStateMachine] = 0x00; r[reg::kTypecMisc] = 0x00; stuckAt = 0; }
      else if (r[reg::kTypecModeCfg] & 1) { r[reg::kTypecStateMachine] = 0x01; r[reg::kTypecMisc] = 0x00; stuckAt = now + 1500; }
    }
    if (a == reg::kChgEnable && !(v & 1)) enableOff = enableOff + 1;
    r[a] = v;
    if (a == reg::kUsbinCmdIl) r[reg::kPowerPath] = static_cast<uint8_t>((r[reg::kPowerPath] & ~0x40) | ((v & 1) ? 0x40 : 0));
    return 0;
  }
  void sleepMs(unsigned ms) { now = now + ms; }
  int vbusMv(unsigned *mv) { adcReads = adcReads + 1; *mv = vbus; return 0; }
  int ibatMa(int *ma) { *ma = ibat; return 0; }
  __attribute__((format(printf, 2, 3))) void log(const char *f, ...) {
    char b[1024];
    va_list ap;
    va_start(ap, f);
    vsnprintf(b, sizeof b, f, ap);
    va_end(ap);
    std::string s(b);
    assert(!s.empty() && s.back() == '\n' && s.find('\n') == s.size() - 1);   // exactly one line per call
    logs.push_back(s);
  }
  // --- ceilings on every write ---
  void checkLimits(uint16_t a, uint8_t v) {
    if (a == reg::kFv) assert(v <= (4400 - 3600) / 10);
    if (a == reg::kFcc) assert(v <= 3000 / 50);
    if (a == reg::kIclCfg) {
      assert(v <= 3000 / 50);
      // independent of inputChoice: what the hardware state at the time of the write allows (update_sw_icl_max)
      const bool apsd = r[reg::kApsdStatus] & 1, typec = (r[reg::kTypecStateMachine] & 0x20) && !(r[reg::kTypecLegacy] & 0x02);
      const bool rp3 = (r[reg::kTypecStateMachine] & 0x20) && (r[reg::kTypecSnkStatus] & 0x02);
      const uint8_t res = r[reg::kApsdResult];
      unsigned max = 500;
      if (apsd && (res & 0x60)) max = 3000;
      if (typec && (r[reg::kTypecSnkStatus] & 0x02)) max = 3000;
      if (typec && (r[reg::kTypecSnkStatus] & 0x04) && max < 1500) max = 1500;
      if (apsd && (res & 0x0e) && max < (rp3 && (res & 0x0a) ? 3000u : 1500u)) max = rp3 && (res & 0x0a) ? 3000u : 1500u;
      assert(v * 50u <= max);
    }
    if (a == reg::kAicl5v) assert(v <= 7);
    if (a == reg::kTypecModeCfg) assert(!(v & 0x04));   // never source only
    if (a == reg::kOtgCmd) assert(!(v & 0x01));        // never the OTG boost
  }
  // --- hardware model ---
  void startApsd() {
    r[reg::kApsdStatus] = 0;
    r[reg::kApsdResult] = 0;
    if (r[reg::kUsbinRt] & 0x10) apsdAt = now + 1500;
  }
  void tick() {
    if (pcStuck && stuckAt && now >= stuckAt && !(r[reg::kTypecModeCfg] & 1)) {
      r[reg::kTypecStateMachine] = 0x02;
      r[reg::kTypecMisc] = 0x18;
      stuckAt = 0;
    }
    if (apsdAt && now >= apsdAt && (r[reg::kUsbinRt] & 0x10)) {
      r[reg::kApsdStatus] = 0x01;
      r[reg::kApsdResult] = pendingResult;
      apsdAt = 0;
    }
  }
  void plug(uint8_t result) {
    pendingResult = result;
    r[reg::kUsbinRt] = 0x10;
    r[reg::kTypecStateMachine] = 0x23;
    r[reg::kTypecMisc] = 0x29;
    r[reg::kTypecSnkStatus] = 0x02;
    r[reg::kTypecLegacy] = 0x07;
    r[reg::kPowerPath] = static_cast<uint8_t>(0x13 | (r[reg::kPowerPath] & 0x40));
    r[reg::kStatus1] = 0x03;
    r[reg::kIclStatus] = 0x0a;
    r[reg::kAiclStatus] = 0x41;
    r[reg::kApsdStatus] = 0;
    r[reg::kApsdResult] = 0;
    if (autoApsd) apsdAt = now + 1500;
  }
  // A source on a cable of its own: snk = TYPE_C_SNK_STATUS (02 Rp 3.0 A, 04 1.5 A, 08 default), legacy = 0x150d
  void plugCable(uint8_t result, uint8_t snk, uint8_t legacy) {
    plug(result);
    r[reg::kTypecSnkStatus] = snk;
    r[reg::kTypecLegacy] = legacy;
  }
  void unplug() {
    r[reg::kUsbinRt] = 0x26;
    r[reg::kTypecStateMachine] = 0x00;
    r[reg::kTypecMisc] = 0x00;
    r[reg::kTypecSnkStatus] = 0x00;
    r[reg::kPowerPath] = static_cast<uint8_t>(0x82 | (r[reg::kPowerPath] & 0x40));
    r[reg::kStatus1] = 0x47;
    r[reg::kIclStatus] = 0;
    r[reg::kAiclStatus] = 0;
    r[reg::kApsdStatus] = 0;
    r[reg::kApsdResult] = 0;
    apsdAt = 0;
  }
  void collapse() {   // the XNU log state after a replug: PAUSE, USBIN PLUGIN UV VASHDN COLLAPSE, ICL 0, AICL collapse
    r[reg::kStatus1] = 0x46;
    r[reg::kUsbinRt] = 0x17;
    r[reg::kIclStatus] = 0;
    r[reg::kAiclStatus] = 0x10;
    r[reg::kPowerPath] = 0x13;
  }
  void healthy() {
    r[reg::kStatus1] = 0x03;
    r[reg::kUsbinRt] = 0x10;
    r[reg::kIclStatus] = 0x0a;
    r[reg::kAiclStatus] = 0x41;
  }
  bool logged(const char *needle) const {
    for (const auto &s : logs) if (s.find(needle) != std::string::npos) return true;
    return false;
  }
  unsigned count(const char *needle) const {
    unsigned n = 0;
    for (const auto &s : logs) if (s.find(needle) != std::string::npos) n = n + 1;
    return n;
  }
};

static void run(Controller &c, Pmi632 &p, unsigned seconds) {
  for (unsigned k = 0; k < seconds; k = k + 1) {
    p.now = p.now + 1000;
    p.tick();
    c.poll(p, p.now);
  }
}

int main() {
  // pure helpers
  for (unsigned v = 0; v < 256; v = v + 1) {
    const uint8_t s = typecMode(static_cast<uint8_t>(v), false), d = typecMode(static_cast<uint8_t>(v), true);
    assert((s & 0x1f) == 0x02 && (s & 0xe0) == (v & 0xe0));
    assert((d & 0x1f) == 0x10 && (d & 0xe0) == (v & 0xe0));
  }
  // inputChoice: (cap, APSD done, result, Rp level 0 none/1 default/2 1.5 A/3 3.0 A, legacy cable)
  assert(inputChoice(3000, true, 0x01, 0, false).ma == 500 && inputChoice(3000, false, 0x08, 0, false).ma == 500);
  assert(inputChoice(3000, true, 0x10, 1, false).ma == 500 && inputChoice(3000, true, 0x00, 0, true).ma == 500);
  assert(inputChoice(3000, true, 0x01, 3, true).ma == 500);                // legacy cable: BC1.2 wins over Rp
  assert(inputChoice(3000, true, 0x01, 3, false).ma == 3000);               // Type-C cable: Rp wins (PD charger, PC)
  assert(inputChoice(3000, true, 0x10, 2, false).ma == 1500 && inputChoice(3000, false, 0, 3, false).ma == 3000);
  assert(inputChoice(3000, true, 0x08, 1, true).ma == 1500 && inputChoice(3000, true, 0x08, 3, true).ma == 3000);
  assert(inputChoice(3000, true, 0x02, 0, false).ma == 1500 && inputChoice(3000, true, 0x04, 3, true).ma == 1500);
  assert(inputChoice(3000, true, 0x28, 1, true).ma == 3000 && inputChoice(3000, true, 0x48, 0, false).ma == 3000);
  assert(inputChoice(1000, true, 0x08, 3, false).ma == 1000 && inputChoice(9000, true, 0x08, 3, false).ma == 3000);
  assert(inputChoice(100, true, 0x01, 0, false).ma == 100);
  assert(std::strcmp(inputChoice(3000, true, 0x28, 1, true).source, "QC2") == 0);
  assert(std::strcmp(inputChoice(3000, true, 0x10, 3, false).rule, "Type-C Rp") == 0);
  assert(rpLevel(0x23, 0x02) == 3 && rpLevel(0x03, 0x02) == 0 && rpLevel(0x23, 0x04) == 2 && rpLevel(0x23, 0x08) == 1);
  assert(aicl5vCode(4400) == 4 && aicl5vCode(4800) == 7 && aicl5vCode(3000) == 0);
  assert(aiclContCode(4000) == 0 && aiclContCode(4600) == 6 && aiclContCode(5600) == 16 && aiclContCode(9999) == 32);
  assert(std::strcmp(apsdName(0x01), "SDP") == 0 && std::strcmp(apsdName(0x28), "QC2") == 0);

  Pmi632 p;
  Controller c;
  // 1. Boot with a PC (SDP) cable in and APSD already done by the boot chain
  p.plug(0x01);
  p.r[reg::kApsdStatus] = 0x01;
  p.r[reg::kApsdResult] = 0x01;
  p.apsdAt = 0;
  run(c, p, 1);
  assert(p.logged("Linux config (boot)") && p.logged("-> Linux 45/0/60/-20/71 C"));
  for (unsigned k = 0; k < 12; k = k + 1) assert(p.r[reg::kJeitaThresholds + k] == kJeitaLinux[k]);
  assert((p.r[reg::kJeitaEn] & 0x0f) == 0 && (p.r[reg::kJeitaEn] & 0x10));   // soft comp off, hard limit kept
  assert(p.r[reg::kBatifPullUp] == 0x01 && (p.r[reg::kAiclOptions] & 0x9c) == 0x14);
  assert(p.r[reg::kTypecDebugAccessSink] == 0x03 && p.r[reg::kTypecModeCfg] == 0x02 && !(p.r[reg::kTypeCCfg] & 0x80));
  assert(p.logged("plug-in 1 (at boot)") && !p.logged("APSD rerun") && p.apsdReruns == 0);
  assert(p.logged("Type-C attach, Rp 3.0A") && p.logged("legacy cable (non-compliant)"));
  assert(c.phase == kConfigured && p.r[reg::kIclCfg] == 10 && p.r[reg::kFv] == 75 && p.r[reg::kFcc] == 40);
  assert(p.r[reg::kChgEnable] == 1 && p.r[reg::kIclOptions] == 0x03 && (p.r[reg::kUsbinLoadCfg] & 0x10));
  assert(!(p.r[reg::kCmdIclOverride] & 1) && !(p.r[reg::kUsbinCmdIl] & 1) && (p.r[reg::kUsbinOptions1] & 0x08));
  assert(p.r[reg::kAicl5v] == 4 && p.r[reg::kAiclCont] == 0);   // Linux defaults instead of the UEFI ones
  assert(p.logged("APSD done result 01 SDP, Type-C attached Rp 3.0A legacy cable -> source SDP (BC1.2 SDP): ICL 500 mA "
                  "FCC 2000 mA FV 4350 mV, VBUS 0 mV: configured"));
  // steady charging: nothing more is written but the watchdog
  unsigned w = p.writes;
  run(c, p, 30);
  assert(p.writes == w + 30);
  // FULLON with a passing collapse bit is not a stuck input
  p.r[reg::kUsbinRt] = 0x11;
  run(c, p, 40);
  assert(c.weakSteps == 0);
  p.healthy();

  // 2. Unplug: AICL thresholds back to default, ICL 500, input on, one line
  p.r[reg::kAicl5v] = 7;
  p.unplug();
  run(c, p, 1);
  assert(p.count("unplug 1") == 1 && p.logged("Type-C detach") && c.phase == kAbsent);
  assert(p.r[reg::kAicl5v] == 4 && p.r[reg::kIclCfg] == 10 && !(p.r[reg::kUsbinCmdIl] & 1));
  // 3. Absent: Type-C disable pulse every 15 s, role restored
  run(c, p, 31);
  assert(p.typecDisablePulses == 2 && c.typecResets == 2 && p.r[reg::kTypecModeCfg] == 0x02);
  assert(p.count("GuampChg: unplug") == 1);

  // 4. Replug a DCP wall charger: hardware APSD, then 1500 mA
  p.plug(0x08);
  run(c, p, 1);
  assert(p.logged("plug-in 2 (edge)") && c.phase == kDetecting);
  run(c, p, 2);
  assert(c.phase == kConfigured && p.r[reg::kIclCfg] == 60 &&
         p.logged("result 08 DCP, Type-C attached Rp 3.0A legacy cable -> source DCP (BC1.2 DCP, Rp based): ICL 3000 mA"));
  assert(p.apsdReruns == 0);

  // 5. Replug where APSD never completes by itself: two reruns, then the unknown-source 500 mA
  p.unplug();
  run(c, p, 2);
  p.autoApsd = false;
  p.pendingResult = 0;
  p.plug(0x08);
  run(c, p, 1);   // the simulated APSD is kept from ever finishing
  p.apsdAt = 0;
  for (unsigned k = 0; k < 12; k = k + 1) { run(c, p, 1); p.apsdAt = 0; }
  assert(p.apsdReruns == 2 && p.count("APSD rerun") == 2);
  assert(c.phase == kConfigured && p.r[reg::kIclCfg] == 10 && p.logged("APSD timeout result 00 none, Type-C attached Rp 3.0A legacy cable -> source none "
                                                                  "(unknown source, SDP level): ICL 500 mA"));
  p.autoApsd = true;

  // 6. The XNU failure: replug on the PC, then PAUSE with USBIN UV/VASHDN/COLLAPSE, ICL 0, AICL collapse
  p.unplug();
  run(c, p, 2);
  p.plug(0x01);
  run(c, p, 3);
  assert(c.phase == kConfigured && p.r[reg::kIclCfg] == 10);
  p.collapse();
  const unsigned restarts = p.aiclRestarts;
  run(c, p, 9);
  assert(c.weakSteps == 0);   // 10 s of evidence first
  run(c, p, 2);
  assert(c.weakSteps == 1 && p.r[reg::kAicl5v] == 6 && p.r[reg::kAiclCont] == 2 && p.aiclRestarts == restarts + 1);
  assert(!(p.r[reg::kUsbinCmdIl] & 1) && p.logged("weak-adapter step 1, AICL 4600/4200 mV: AICL restarted"));
  run(c, p, 15);
  assert(c.weakSteps == 2 && p.r[reg::kAicl5v] == 7 && p.r[reg::kAiclCont] == 4);
  run(c, p, 15);
  assert(c.weakSteps == 3 && p.r[reg::kAicl5v] == 7 && p.r[reg::kAiclCont] == 6 && !c.aiclMax);
  run(c, p, 15);
  assert(c.weakSteps == 4 && c.aiclMax && p.r[reg::kIclCfg] == 2 && p.logged("ICL 100 mA: AICL restarted"));
  run(c, p, 60);
  assert(c.redetects == 1 && p.logged("re-detect 1") && p.r[reg::kAicl5v] == 4 && !c.aiclMax);
  p.collapse();
  run(c, p, 3);   // APSD rerun answered -> configured again at 500 mA
  assert(c.phase == kConfigured && p.r[reg::kIclCfg] == 10 && p.apsdReruns >= 3);
  // the source recovers: no more steps, the ICL transition is logged
  p.healthy();
  const unsigned steps = c.weakSteps;
  run(c, p, 60);
  assert(c.weakSteps == steps && p.logged("ICL 0 -> 500 mA"));

  // 7. Battery too hot: PAUSE is legitimate, never "recovered"
  p.collapse();
  p.r[reg::kStatus7] = 0x08;
  run(c, p, 60);
  assert(c.weakSteps == steps);
  p.r[reg::kStatus7] = 0x00;
  p.healthy();

  // 8. Input suspended by the charger with our command off: one toggle per 30 s
  p.r[reg::kPowerPath] = 0x53;
  run(c, p, 1);
  assert(c.suspendRecoveries == 1 && p.logged("input suspended by the charger") && !(p.r[reg::kUsbinCmdIl] & 1));
  p.r[reg::kPowerPath] = 0x53;   // still suspended (the model clears it on the toggle)
  run(c, p, 29);
  p.r[reg::kPowerPath] = 0x53;
  run(c, p, 2);
  assert(c.suspendRecoveries == 2);
  p.r[reg::kPowerPath] = 0x13;
  run(c, p, 1);
  assert(p.logged("input resumed"));

  // 9. Recovery off by boot argument
  {
    Pmi632 q;
    Controller d;
    d.recover = false;
    q.plug(0x01);
    run(d, q, 3);
    q.collapse();
    run(d, q, 120);
    assert(d.weakSteps == 0 && d.redetects == 0 && d.phase == kConfigured);
  }
  // 10. Battery over-voltage: configured without the charge enable toggle
  {
    Pmi632 q;
    Controller d;
    q.r[reg::kStatus2] = 0x02;
    q.plug(0x01);
    run(d, q, 3);
    assert(d.phase == kConfigured && q.enableOff == 0);
  }
  // 11. SPMI error on the ICL write: logged, no "configured" until it went through, then retried
  {
    Pmi632 q;
    Controller d;
    q.plug(0x08);
    run(d, q, 1);
    q.failAddr = reg::kIclCfg;
    q.failCount = 1;
    run(d, q, 2);
    assert(q.logged("configure failed") || q.logged(": error (SPMI -5)"));
    run(d, q, 1);
    assert(d.phase == kConfigured && q.r[reg::kIclCfg] == 60);
  }
  // 12. Source change without a seen unplug (short replug between polls): re-detection from the new APSD result
  {
    Pmi632 q;
    Controller d;
    q.plug(0x01);
    run(d, q, 3);
    q.r[reg::kApsdResult] = 0x08;
    run(d, q, 1);
    assert(q.logged("APSD result 01 SDP -> 08 DCP"));
    run(d, q, 1);
    assert(d.phase == kConfigured && q.r[reg::kIclCfg] == 60);
  }
  // 13. DRP try.SNK role on request
  {
    Pmi632 q;
    Controller d;
    d.typecDrp = true;
    q.r[reg::kTypecModeCfg] = 0x02;
    run(d, q, 1);
    assert(q.r[reg::kTypecModeCfg] == 0x10 && q.logged("DRP try.SNK"));
  }
  // 14. Boot without cable: no plug/unplug lines, ICL/AICL lines only once something happens
  {
    Pmi632 q;
    Controller d;
    q.unplug();
    run(d, q, 5);
    assert(!q.logged("unplug") && !q.logged("plug-in") && !q.logged("ICL "));
  }
  // 15. C-to-C replug on the PC (logs of builds 40541050037 and 40546923445): Rp on CC, Type-C waiting for VBUS with
  //     VBUS error (0x1509 02, 0x150b 18), USBIN 26 without plug-in. Every CC re-enable passes through "unattached":
  //     that must not restart the ladder at attempt 1 (bug of 40546923445: "recovery 1" every 8 s). Escalation
  //     2/3/5/10 s, sink only / DRP try.SNK alternating, 5 s then 20 s gaps; VBUS ADC in each attempt line.
  {
    Pmi632 q;
    Controller d;
    q.plug(0x01);
    run(d, q, 3);
    assert(d.phase == kConfigured);
    assert(q.r[reg::kUsbinAdapterAllow] == 0x08 && q.r[reg::kTypecIntEn1] == 0x00 && q.r[reg::kTypecIntEn2] == 0x40);
    assert(q.logged("vs Linux (boot, before, XNU/Linux):"));
    q.unplug();
    q.r[reg::kTypecStateMachine] = 0x01;
    q.r[reg::kTypecStatus0e] = 0xff;
    run(d, q, 3);
    assert(q.logged("vs Linux (unplug, XNU/Linux):"));
    q.pcStuck = true;
    q.r[reg::kTypecStateMachine] = 0x02;
    q.r[reg::kTypecMisc] = 0x18;
    q.r[reg::kTypecStatus0e] = 0x8f;
    run(d, q, 4);
    assert(d.ccRecoveries == 0 && q.logged("Type-C state 02 WAIT_VBUS misc 18 DEBOUNCED VBUS_ERROR Rp none"));
    run(d, q, 2);
    assert(d.ccRecoveries == 1 && (q.r[reg::kTypecModeCfg] & 1) && q.adcReads == 2);   // configure + recovery
    assert(q.logged("VBUS ADC 0 mV, OTG 00 cmd-il 00 allow 08): Type-C recovery 1, CC off 2 s, then sink only"));
    assert(q.logged("vs Linux (Rp without VBUS, XNU/Linux):"));
    run(d, q, 1);
    assert(q.r[reg::kTypecModeCfg] & 1);   // still off: nothing (role enforcement included) turns it back on early
    run(d, q, 1);
    assert(q.r[reg::kTypecModeCfg] == 0x02 && q.r[reg::kTypecStateMachine] == 0x01);
    run(d, q, 17);
    assert(d.ccRecoveries == 1);           // unattached right after CC on did not reset the ladder
    q.vbus = 5050;
    run(d, q, 1);
    assert(d.ccRecoveries == 2 && q.r[reg::kTypecModeCfg] == 0x11 &&
           q.logged("VBUS ADC 5050 mV PRESENT, NOT ACCEPTED, OTG 00 cmd-il 00 allow 08): Type-C recovery 2, CC off 3 s, then DRP try.SNK"));
    q.vbus = 0;
    run(d, q, 3);
    assert(q.r[reg::kTypecModeCfg] == 0x10);
    run(d, q, 17);
    assert(d.ccRecoveries == 3 && q.logged("recovery 3, CC off 5 s, then sink only"));
    run(d, q, 20);
    assert(d.ccRecoveries == 4 && q.logged("recovery 4, CC off 10 s, then DRP try.SNK"));
    assert(q.count("Type-C recovery 1,") == 1 && d.lastVbusMv == 0);
    run(d, q, 15);
    q.pcStuck = false;
    q.plug(0x01);   // the PC turns VBUS on for the new attach
    run(d, q, 3);
    assert(d.phase == kConfigured && q.logged("VBUS back after Type-C recovery 4 (DRP try.SNK)") && d.ccRecoveries == 0);
    assert(q.r[reg::kTypecModeCfg] == 0x10);   // the role that attached stays for this connection
    q.unplug();
    q.r[reg::kTypecStateMachine] = 0x01;
    run(d, q, 5);
    assert(q.r[reg::kTypecModeCfg] == 0x10);   // 10 s without anything on CC before the role goes back
    run(d, q, 7);
    assert(q.r[reg::kTypecModeCfg] == 0x02 && q.logged("Type-C role 10 -> 02 (sink only)"));
    // never stuck forever: later attempts keep coming, every 60 s after the sixth
    q.pcStuck = true;
    q.r[reg::kTypecStateMachine] = 0x02;
    q.r[reg::kTypecMisc] = 0x18;
    run(d, q, 300);
    assert(d.ccRecoveries >= 8 && d.ccRecoveries <= 10);
    assert((q.r[reg::kTypecModeCfg] & 0x04) == 0);
    // Linux Type-C configuration written: exit state SEL_SRC_UPPER_REF, no CC-out override, normal debounce
    assert((q.r[reg::kTypecExitStateCfg] & 0x0d) == 0x04 && !(q.r[reg::kTypecCcoutControl] & 1));
  }
  // 16. Nothing on CC while unplugged: the 1 s disable every 15 s, without a log line each time
  {
    Pmi632 q;
    Controller d;
    q.unplug();
    q.r[reg::kTypecStateMachine] = 0x01;
    run(d, q, 61);
    assert(d.typecResets == 4 && q.count("Type-C off") == 1 && !q.logged("Type-C on again") && d.ccRecoveries == 0);
  }
  // 17. Boot chain left the OTG boost on (XNU log after a C-to-C replug: 0x1140 01, USBIN REVI+UV+VASHDN, ADC 5.0 V):
  // the first poll turns it off and makes the command the only OTG enable source, the next polls leave it alone
  {
    Pmi632 q;
    Controller d;
    q.r[reg::kOtgCmd] = 0x01;
    q.r[reg::kOtgCfg] = 0x2a;
    q.unplug();
    q.pcStuck = true;
    q.vbus = 5009;
    q.r[reg::kTypecStateMachine] = 0x02;
    q.r[reg::kTypecMisc] = 0x18;
    run(d, q, 1);
    assert(q.r[reg::kOtgCmd] == 0x00 && q.r[reg::kOtgCfg] == 0x28 && d.otgOffs == 1);
    assert(q.logged("OTG boost was on (no VBUS: DCDC_CMD_OTG 01, OTG_CFG 2a, USBIN 26)"));
    run(d, q, 30);
    assert(d.otgOffs == 1 && q.count("OTG boost was on") == 1);
    assert(q.logged("OTG 00 cmd-il 00 allow 08") && !q.logged("OWN OTG BOOST"));
    // something turns it on again later: undone at the next poll, logged again
    q.r[reg::kOtgCmd] = 0x01;
    run(d, q, 1);
    assert(q.r[reg::kOtgCmd] == 0x00 && d.otgOffs == 2);
  }
  // 18. The XNU report of 28/10: boot on the PC (SDP, legacy flag), unplug, then a 33 W PD wall charger on a C-to-C
  //     cable at runtime (PMI632 has no PD: Rp 3.0 A at 5 V, D+/D- open -> APSD FLOAT). Before: ICL stayed at the SDP
  //     500 mA (~2 W). Linux: non-legacy Rp 3.0 A -> 3000 mA. One line with source and ICL/FCC, then the measured values.
  {
    Pmi632 q;
    Controller d;
    q.plug(0x01);
    q.r[reg::kApsdStatus] = 0x01;
    q.r[reg::kApsdResult] = 0x01;
    q.apsdAt = 0;
    run(d, q, 2);
    assert(d.phase == kConfigured && q.r[reg::kIclCfg] == 10 && std::strcmp(d.choice.source, "SDP") == 0);
    q.unplug();
    run(d, q, 3);
    assert(q.logged("input on (was source SDP, 500 mA)"));
    q.vbus = 5020;
    q.ibat = 1850;
    q.plugCable(0x10, 0x02, 0x00);
    run(d, q, 3);
    assert(d.phase == kConfigured && q.r[reg::kIclCfg] == 60 && q.r[reg::kFcc] == 40 && q.r[reg::kFv] == 75);
    assert(!(q.r[reg::kUsbinCmdIl] & 1) && (q.r[reg::kChgEnable] & 1) && (q.r[reg::kIclOptions] & 1));
    assert(q.logged("APSD done result 10 FLOAT, Type-C attached Rp 3.0A -> source FLOAT (Type-C Rp): ICL 3000 mA FCC "
                    "2000 mA FV 4350 mV, VBUS 5020 mV: configured (SPMI 0)"));
    q.r[reg::kIclStatus] = 0x2c;   // AICL settled at 2200 mA
    run(d, q, 13);
    assert(!q.logged("input after 15 s"));
    run(d, q, 2);
    assert(q.count("input after 15 s") == 1 &&
           q.logged("input after 15 s: source FLOAT, ICL set 3000 mA, ICL status 2200 mA, AICL 41 DONE, VBUS 5020 mV, "
                    "battery 1850 mA, FULLON"));
    run(d, q, 30);
    assert(q.count("input after 15 s") == 1 && d.rpChanges == 0);
    q.unplug();
    run(d, q, 2);
    assert(q.logged("input on (was source FLOAT, 3000 mA)") && q.r[reg::kIclCfg] == 10);
  }
  // 19. Same charger reporting SDP on the data lines, and one with Rp 1.5 A: the Rp decides on a Type-C cable
  {
    Pmi632 q;
    Controller d;
    q.plugCable(0x01, 0x02, 0x00);
    run(d, q, 3);
    assert(q.r[reg::kIclCfg] == 60 && q.logged("-> source SDP (Type-C Rp): ICL 3000 mA"));
    q.unplug();
    run(d, q, 2);
    q.plugCable(0x08, 0x04, 0x00);
    run(d, q, 3);
    assert(q.r[reg::kIclCfg] == 30 && q.logged("Rp 1.5A -> source DCP (Type-C Rp): ICL 1500 mA"));
  }
  // 20. USB-A port of the charger (A-to-C cable: legacy, Rp default): BC1.2 DCP 1500 mA; a QC result (HVDCP
  //     detection by the PMIC) 3000 mA at 5 V, nothing raises VBUS
  {
    Pmi632 q;
    Controller d;
    q.plugCable(0x08, 0x08, 0x03);
    run(d, q, 3);
    assert(q.r[reg::kIclCfg] == 30 && q.logged("Rp default legacy cable -> source DCP (BC1.2 DCP, Rp based): ICL 1500 mA"));
    q.unplug();
    run(d, q, 2);
    q.plugCable(0x28, 0x08, 0x03);
    run(d, q, 3);
    assert(q.r[reg::kIclCfg] == 60 && q.logged("-> source QC2 (HVDCP at 5 V): ICL 3000 mA"));
    for (const auto &w : q.trace) assert(w.first != 0x1343);   // CMD_HVDCP_2: never a voltage request
    // a PC on a legacy cable with Rp default: SDP 500 as before
    q.unplug();
    run(d, q, 2);
    q.plugCable(0x01, 0x08, 0x03);
    run(d, q, 3);
    assert(q.r[reg::kIclCfg] == 10);
  }
  // 21. guamp-icl-ma ceiling and a lower FCC by boot argument
  {
    Pmi632 q;
    Controller d;
    d.iclMa = 1000;
    d.fccMa = 1500;
    q.plugCable(0x10, 0x02, 0x00);
    run(d, q, 3);
    assert(q.r[reg::kIclCfg] == 20 && q.r[reg::kFcc] == 30 && q.logged("ICL 1000 mA FCC 1500 mA"));
  }
  // 22. Rp change while configured (smblib_handle_rp_change): 1.5 A -> 3.0 A after two polls; a one-poll glitch does
  //     nothing; the weak-adapter 100 mA is never overridden
  {
    Pmi632 q;
    Controller d;
    q.plugCable(0x10, 0x04, 0x00);
    run(d, q, 3);
    assert(q.r[reg::kIclCfg] == 30);
    q.r[reg::kTypecSnkStatus] = 0x02;
    run(d, q, 1);
    q.r[reg::kTypecSnkStatus] = 0x04;
    run(d, q, 1);
    assert(q.r[reg::kIclCfg] == 30 && d.rpChanges == 0);
    q.r[reg::kTypecSnkStatus] = 0x02;
    run(d, q, 2);
    assert(q.r[reg::kIclCfg] == 60 && d.rpChanges == 1 &&
           q.logged("Type-C Rp 3.0A -> source FLOAT (Type-C Rp): ICL 1500 -> 3000 mA, change 1: configured (SPMI 0)"));
    run(d, q, 30);
    assert(d.rpChanges == 1);
    q.collapse();
    run(d, q, 70);
    assert(d.aiclMax && q.r[reg::kIclCfg] == 2 && d.rpChanges == 1);
  }
  for (const auto &s : p.logs) fputs(s.c_str(), stdout);
  printf("PASS: %zu log lines, %u writes; boot/unplug/replug (SDP, DCP, APSD timeout), Type-C reset, collapse -> "
         "weak-adapter steps -> 100 mA -> re-detect, thermal/BAT_OV guards, suspend toggle, SPMI error retry, source "
         "change, DRP, Type-C recovery of Rp without VBUS (C-to-C replug), wall chargers (Type-C Rp, DCP, QC at 5 V, Rp change, ICL cap), FV <= 4.40 V and ICL/FCC ceilings on every write\n",
         p.logs.size(), p.writes);
  return 0;
}
