// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp panel backlight, included by NativeEmmc.cpp after the other services (mapPhysical). Off unless guamp-backlight=1.
// The panel's level is its own DCS 0x51 sent over DSI0 while the boot loader's video stream keeps running
// (GuampBacklightCore.h: packet, clamps, fade and the Linux command-while-video sequence; host test tests/backlight_host.cpp).
//
// iOS side (iPadOS 17 backboardd + CoreBrightness, as UniXNU-sheng package/AppleARMBacklight.cpp found on the same
// kernelcache): CoreBrightness takes as the built-in panel's backlight a service with backlight-control = true for which
// IOObjectConformsTo(service, "AppleARMBacklight") holds, reads min/max from IODisplayParameters brightness and sets
// {brightness, FadePeriod (16.16 s)} through setProperties. Display off (Power key, idle lock) is backboardd's "Setting
// CoreBrightness backlight factor:0 with fade duration": CoreBrightness then sends deviceBrightness = brightness x factor
// = 0, and display on the user level again, so standby needs nothing else here. The IOMFB power request of a blanked
// display (BKDisplaySetBlankingRemovesPower) never reaches the panel: NativeLegacyDisplay has no platform subclass that
// acts on it, so hooking it would mean patching the display builder's vtable slot 354 for no gain.
// Conformance: the vtable copy's getMetaClass (slot 7) answers AppleARMBacklight (AppleARMPlatform's class, no device
// here); without it backlight-control is never published (backlight-control alone made QuartzCore block on a libEDR server
// CoreBrightness never started, sheng int-236). backboardd's sandbox refuses FadePeriod on that class:
// patches/xnu/zzwb-native-sandbox-grant-set-properties.patch (on top of zzw-native-adreno-sandbox-grant.patch) lets the
// processes named in NativeSandboxGrantProcess set its properties. guamp-backlight-control=0 publishes neither.
// guamp-backlight-test=1: at start, dim to 1/8 for 3 s and back (checks the DSI path without iOS).
// guamp-touch-wake=1 (TouchService.inc, armed only once the touch firmware runs): display off also sends the panel's
// off-command (28, 20 ms, 10, 80 ms: dtbo qcom,mdss-dsi-off-command) after level 0, so the HX83102D (TDDI) firmware
// enters its sleep / gesture mode as under Linux; the first level above 0 sends the on-command first (11, 160 ms, 29,
// 50 ms: qcom,mdss-dsi-on-command). guamp-touch-wake-dsi=0 keeps only the level. The touch service follows the state.
// guamp-touch-events=1|2 (TouchService.inc): ogni cambio acceso/spento (il primo livello > 0 di iOS compreso) va al touch
// prima dei comandi del pannello e del livello (EARLY_EVENT_BLANK di Linux); con guamp-touch-events-dsi=1 (default) anche
// off-command dopo il livello 0 e on-command prima del primo livello, come guamp-touch-wake.
// Logs: one line at start, one per brightness request (rate-limited, says display off/on), failed sends (rate-limited).
#include "GuampBacklightCore.h"

namespace backlight {
using namespace guamp_backlight;
volatile uint8_t *gDsi, *gDispcc, *gGccDisp;
volatile uint32_t gRequests, gSends, gFailures;
volatile uint32_t gLastLevel = guamp_backlight::kDefaultLevel;   // last level the panel was sent (panel re-init)
// One DSI command user at a time: the backlight work loop, the touch thread's panel commands and the panel power cycle
// (GuampPanelCycle.inc, which holds it across its whole sequence while the link is in command mode). Recursive: apply()
// sends the wake commands under it.
IORecursiveLock *gDsiLock;
void dsiLock() { if (gDsiLock) IORecursiveLockLock(gDsiLock); }
void dsiUnlock() { if (gDsiLock) IORecursiveLockUnlock(gDsiLock); }
struct DsiGuard { DsiGuard() { dsiLock(); } ~DsiGuard() { dsiUnlock(); } };

uint32_t rd32(volatile uint8_t *base, uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(base + off); }
uint64_t nowUs() {
  uint64_t ns = 0;
  absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
  return ns / 1000u;
}
// The DSI register file is reachable only behind gcc_disp_ahb_clk and disp_cc_mdss_ahb_clk; byte/esc/pixel clocks say
// the link and the video stream run. Any of them off: no DSI access at all.
bool clocksOn() {
  if (!gDsi || !gDispcc || !gGccDisp) return false;
  if (!cbcrRunning(rd32(gGccDisp, kGccDispAhbOff))) return false;
  static const uint32_t branches[] = {kCbcrMdssAhb, kCbcrByte0, kCbcrEsc0, kCbcrPclk0};
  for (uint32_t off : branches)
    if (rd32(gDispcc, off) & (1u << 31)) return false;
  return true;
}
struct Io {
  uint32_t rd(uint32_t o) { return rd32(gDsi, o); }
  void wr(uint32_t o, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(gDsi + o) = v; __asm__ volatile("dsb sy" ::: "memory"); }
  bool clocksOn() { return backlight::clocksOn(); }
  void delayUs(uint32_t us) { IODelay(us); }
  void sleepMs(uint32_t ms) { IOSleep(ms); }
  uint64_t nowUs() { return backlight::nowUs(); }
};
}  // namespace backlight

class GuampBacklight : public IOService {
  OSDeclareDefaultStructors(GuampBacklight)
  static constexpr uint32_t kMaxFailures = 4, kRetries = 3;
  IOWorkLoop *loop = nullptr;
  IOTimerEventSource *fade = nullptr, *request = nullptr, *test = nullptr;
  uint32_t level = guamp_backlight::kDefaultLevel, from = 0, to = guamp_backlight::kDefaultLevel;
  uint32_t failuresInRow = 0, retries = 0, testStage = 0;
  bool broken = false, known = false;                  // known: the panel has been sent a level by us
  uint64_t fadeStart = 0, period = 0;
  guamp_backlight::LogGate gate;
  volatile uint64_t pending = 0;                       // bit 63 new, goal << 16, fade ms (newest wins)

  bool mayLog() { return gate.allow(backlight::nowUs() / 1000u); }
  // Work loop only. Sends one level; a send that the hardware refused (busy, no DMA_DONE) counts against the controller,
  // and kMaxFailures in a row stop all DSI access until reboot. "Not ready" (clocks, controller, timing) touches nothing.
  bool panelAsleep = false;                            // off-command sent (guamp-touch-wake)
  // Panel off / on command (two DCS short writes with the dtbo waits); the outcome goes to the log and to the touch
  // service, which swaps its code overlay only when the panel really changed state.
  bool panelCommand(bool on) {
    guamp_backlight::Diag d;
    backlight::Io io;
    const uint32_t first = on ? guamp_backlight::kDcsSleepOut : guamp_backlight::kDcsDisplayOff;
    const uint32_t second = on ? guamp_backlight::kDcsDisplayOn : guamp_backlight::kDcsSleepIn;
    int r = guamp_backlight::sendDcsShort(io, first, d);
    if (r == guamp_backlight::kSent) {
      IOSleep(on ? 160 : 20);
      r = guamp_backlight::sendDcsShort(io, second, d);
      if (r == guamp_backlight::kSent) IOSleep(on ? 50 : 80);
    }
    IOLog("GuampBacklight: panel %s (DCS %02x %02x): %s\n", on ? "on-command" : "off-command", first, second,
          guamp_backlight::resultName(r));
    return r == guamp_backlight::kSent;
  }
  // guamp-touch-events (TouchService.inc, displayEvent): lo stato del display detto al touch come il
  // DRM_PANEL_EARLY_EVENT_BLANK di sde_kms, prima dei comandi del pannello e del livello; una volta per cambio.
  bool evKnown = false, evOn = false;
  void touchEvent(bool on) {
    if (!touch::gEventsMode || (evKnown && evOn == on)) return;
    evKnown = true;
    evOn = on;
    touch::displayEvent(on);
  }
  void apply(uint32_t next) {
    if (broken || (known && next == level)) return;
    backlight::DsiGuard guard;
    guamp_backlight::Diag d;
    backlight::Io io;
    if (next && (!known || level == 0)) touchEvent(true);   // EARLY UNBLANK (il primo livello di iOS compreso)
    if (!next && (!known || level != 0)) touchEvent(false); // EARLY POWERDOWN
    if (next && panelAsleep) {                         // wake the panel before its first visible level
      panelAsleep = false;
      panelCommand(true);
      touch::displayState(true);
    }
    const int r = guamp_backlight::sendBrightness(io, next, d);
    if (r == guamp_backlight::kSent) {
      const bool wasOn = !known || level != 0;
      level = next;
      known = true;
      backlight::gLastLevel = next;
      failuresInRow = 0;
      backlight::gSends = backlight::gSends + 1;
      if (!next && wasOn && touch::gWakeArmed) {
        if (touch::gWakeDsi) panelAsleep = panelCommand(false);
        touch::displayState(false);
      } else if (!next && wasOn && touch::gEventsMode && touch::gEventsDsi && !panelAsleep) {
        panelAsleep = panelCommand(false);               // dsi_panel_disable dopo il suspend del touch
      } else if (next && !wasOn) {
        touch::displayState(true);
      }
      return;
    }
    backlight::gFailures = backlight::gFailures + 1;
    if (r == guamp_backlight::kBusy || r == guamp_backlight::kTimeout) failuresInRow = failuresInRow + 1;
    if (failuresInRow >= kMaxFailures) broken = true;
    if (broken || mayLog())
      IOLog("GuampBacklight: level %u not sent: %s (hw %08x ctrl %08x line %u int %08x status %08x fifo %08x ack %08x "
            "timeout %08x, %u us)%s\n", next, guamp_backlight::resultName(r), d.hw, d.ctrl, d.line, d.intCtrl, d.status,
            d.fifo, d.ack, d.timeouts, d.waitedUs, broken ? "; DSI access stopped" : "");
  }
  static void fadeStep(OSObject *owner, IOTimerEventSource *sender) {
    GuampBacklight *self = static_cast<GuampBacklight *>(owner);   // static: getMetaClass answers AppleARMBacklight
    if (!self) return;
    const uint32_t next = guamp_backlight::fadeLevel(self->from, self->to, mach_absolute_time() - self->fadeStart, self->period);
    self->apply(next);
    if (next != self->to) { sender->setTimeoutMS(guamp_backlight::kStepMs); return; }
    if ((self->level != self->to || !self->known) && !self->broken && self->retries < kRetries) {    // last step refused: retry it
      self->retries = self->retries + 1;
      sender->setTimeoutMS(guamp_backlight::kStepMs);
    }
  }
  static void requestFired(OSObject *owner, IOTimerEventSource *) {
    GuampBacklight *self = static_cast<GuampBacklight *>(owner);
    if (!self) return;
    const uint64_t r = __atomic_exchange_n(&self->pending, 0ull, __ATOMIC_ACQUIRE);
    if (!(r >> 63)) return;
    const uint32_t goal = (uint32_t)(r >> 16) & 0xffffu, ms = (uint32_t)r & 0xffffu;
    if ((goal != self->to || !self->known) && self->mayLog()) {
      const uint32_t dropped = self->gate.takeDropped();
      IOLog("GuampBacklight: %u -> %u in %u ms%s (requests %u, sent %u, failed %u%s)\n", self->level, goal, ms,
            goal == 0 && self->level != 0 ? ", display off" : goal != 0 && self->level == 0 ? ", display on" : "",
            backlight::gRequests, backlight::gSends, backlight::gFailures, dropped ? ", log lines dropped" : "");
    }
    self->fade->cancelTimeout();
    self->from = self->level;
    self->to = goal;
    self->retries = 0;
    self->fadeStart = mach_absolute_time();
    nanoseconds_to_absolutetime((uint64_t)ms * 1000000ull, &self->period);
    fadeStep(self, self->fade);
  }
  static void testFired(OSObject *owner, IOTimerEventSource *sender) {
    GuampBacklight *self = static_cast<GuampBacklight *>(owner);
    if (!self) return;
    self->testStage = self->testStage + 1;
    if (self->testStage == 1) { self->post(guamp_backlight::kMaxLevel / 8, 500); sender->setTimeoutMS(3000); }
    else if (self->testStage == 2) self->post(guamp_backlight::kDefaultLevel, 500);
  }
  // Any thread: hand the newest request to the work loop without waiting for it (a runAction from backboardd's brightness
  // thread blocks it on the work loop's gate, sheng int-277).
  void post(uint32_t goal, uint32_t ms) {
    __atomic_store_n(&pending, 1ull << 63 | (uint64_t)goal << 16 | ms, __ATOMIC_RELEASE);
    request->setTimeoutUS(1);
  }
  void publish(uint32_t value) {
    OSDictionary *b = OSDictionary::withCapacity(3), *p = OSDictionary::withCapacity(1);
    OSNumber *mn = OSNumber::withNumber(0ULL, 32), *mx = OSNumber::withNumber(guamp_backlight::kMaxLevel, 32);
    OSNumber *v = OSNumber::withNumber(value, 32);
    if (b && p && mn && mx && v) {
      b->setObject("min", mn); b->setObject("max", mx); b->setObject("value", v);
      p->setObject("brightness", b);
      setProperty("IODisplayParameters", p);
    }
    OSSafeReleaseNULL(mn); OSSafeReleaseNULL(mx); OSSafeReleaseNULL(v); OSSafeReleaseNULL(b); OSSafeReleaseNULL(p);
  }
  static inline const OSMetaClass *gApple = nullptr;
  static const OSMetaClass *appleMeta(const OSMetaClassBase *) { return gApple; }
  // As sheng: AppleARMBacklight and IOService stay in getMetaClass's superclass chain, so OSDynamicCast to IOService works
  // (the sandbox and the grant need it) and to GuampBacklight does not (the event source actions cast statically). The
  // service is never freed, so the vtable copy is never freed either.
  bool impersonate() {
    enum : uint32_t { kSlots = 320, kGetMetaClass = 7 };
    const OSSymbol *name = OSSymbol::withCString("AppleARMBacklight");
    gApple = name ? OSMetaClass::getMetaClassWithName(name) : nullptr;
    OSSafeReleaseNULL(name);
    if (!gApple) return false;
    uintptr_t *copy = static_cast<uintptr_t *>(IOMallocZero(kSlots * sizeof(uintptr_t)));
    if (!copy) return false;
    const uintptr_t *original = *reinterpret_cast<uintptr_t **>(this);
    for (uint32_t i = 0; i < kSlots; ++i) copy[i] = original[i];
    copy[kGetMetaClass] = reinterpret_cast<uintptr_t>(&appleMeta);
    __asm__ volatile("dmb ish" ::: "memory");
    *reinterpret_cast<uintptr_t **>(this) = copy;
    return true;
  }
public:
  bool startBacklight(bool control, bool selfTest) {
    loop = IOWorkLoop::workLoop();
    fade = IOTimerEventSource::timerEventSource(this, &GuampBacklight::fadeStep);
    request = IOTimerEventSource::timerEventSource(this, &GuampBacklight::requestFired);
    test = IOTimerEventSource::timerEventSource(this, &GuampBacklight::testFired);
    if (!loop || !fade || !request || !test) return false;
    if (loop->addEventSource(fade) != kIOReturnSuccess || loop->addEventSource(request) != kIOReturnSuccess ||
        loop->addEventSource(test) != kIOReturnSuccess) return false;
    const bool apple = control && impersonate();
    if (apple) { setProperty("NativeSandboxGrantProcess", "backboardd"); setProperty("backlight-control", true); }
    publish(level);
    registerService();
    const bool clocks = backlight::clocksOn();
    uint32_t hw = 0, ctrl = 0, activeV = 0, total = 0, trig = 0, sched = 0;
    if (clocks) {
      backlight::Io io;
      hw = io.rd(guamp_backlight::rHwVersion); ctrl = io.rd(guamp_backlight::rCtrl);
      activeV = io.rd(guamp_backlight::rActiveV); total = io.rd(guamp_backlight::rTotal);
      trig = io.rd(guamp_backlight::rTrigCtrl); sched = io.rd(guamp_backlight::rDmaSchedule);
    }
    IOLog("GuampBacklight: DSI0 %s hw %08x ctrl %08x (%s) active-v %08x total %08x line %u trig %08x sched %08x; "
          "%s; levels %u-%u, assumed %u%s\n", clocks ? "clocked" : "UNCLOCKED (no access)", hw, ctrl,
          clocks ? guamp_backlight::resultName(guamp_backlight::readiness(hw, ctrl)) : "-", activeV, total,
          guamp_backlight::scheduleLine(activeV, total), trig, sched,
          apple ? "AppleARMBacklight conformance, backlight-control" : control ? "AppleARMBacklight MISSING: not a backlight for iOS"
                                                                             : "backlight-control off (guamp-backlight-control=0)",
          guamp_backlight::kMinLevel, guamp_backlight::kMaxLevel, level, selfTest ? ", self-test in 2 s" : "");
    if (selfTest) test->setTimeoutMS(2000);
    return true;
  }
  IOReturn setProperties(OSObject *properties) override {     // backboardd's thread: hand over, never wait
    OSDictionary *d = OSDynamicCast(OSDictionary, properties);
    if (!d) return kIOReturnBadArgument;
    OSNumber *b = OSDynamicCast(OSNumber, d->getObject("brightness"));
    if (!b) return kIOReturnSuccess;
    OSNumber *f = OSDynamicCast(OSNumber, d->getObject("FadePeriod"));
    const uint32_t goal = guamp_backlight::clampLevel((int32_t)b->unsigned32BitValue());
    backlight::gRequests = backlight::gRequests + 1;
    post(goal, f ? guamp_backlight::fadeMsFromFixed(f->unsigned32BitValue()) : 0u);
    publish(goal);
    return kIOReturnSuccess;
  }
};
OSDefineMetaClassAndStructors(GuampBacklight, IOService)

namespace backlight {
// Touch service experiment (guamp-touch-panelcycle): off-command then on-command, from the touch thread at boot.
// which 3: the dtbo on-command in full (qcom,mdss-dsi-on-command of hlt_hx83102d, waits included): 11 / 160 ms,
// 29 / 50 ms, 51 <level> (the last level sent instead of 00 00, so the screen does not stay dark), 55 01, 53 2C,
// B9 83 10 2D, CD 01 / 10 ms. Sent in HS during the video stream's vertical front porch, as the level is.
int panelInit() {
  const uint32_t lv = gLastLevel ? gLastLevel : guamp_backlight::kDefaultLevel;
  const uint8_t c11[] = {0x11}, c29[] = {0x29}, c51[] = {0x51, (uint8_t)(lv >> 8), (uint8_t)lv}, c55[] = {0x55, 0x01},
                c53[] = {0x53, 0x2C}, cB9[] = {0xB9, 0x83, 0x10, 0x2D}, cCD[] = {0xCD, 0x01};
  struct Step { const uint8_t *p; unsigned n, wait; };
  const Step steps[] = {{c11, 1, 160}, {c29, 1, 50}, {c51, 3, 0}, {c55, 2, 0}, {c53, 2, 0}, {cB9, 4, 0}, {cCD, 2, 10}};
  DsiGuard guard;
  guamp_backlight::Diag d;
  Io io;
  for (const Step &s : steps) {
    const int r = guamp_backlight::sendDcs(io, s.p, s.n, d);
    if (r != guamp_backlight::kSent) {
      IOLog("GuampBacklight: panel init %02x: %s (hw %08x ctrl %08x status %08x ack %08x)\n", s.p[0],
            guamp_backlight::resultName(r), d.hw, d.ctrl, d.status, d.ack);
      return r;
    }
    if (s.wait) IOSleep(s.wait);
  }
  return guamp_backlight::kSent;
}
int panelCycle(int which) {
  if (which == 3) return panelInit();
  static const uint32_t cmds[] = {guamp_backlight::kDcsDisplayOff, guamp_backlight::kDcsSleepIn, guamp_backlight::kDcsSleepOut,
                                  guamp_backlight::kDcsDisplayOn};
  static const uint32_t waits[] = {20, 120, 160, 50};
  DsiGuard guard;
  guamp_backlight::Diag d;
  Io io;
  const unsigned first = which == 1 ? 2u : 0u, last = which == 0 ? 2u : 4u;
  for (unsigned i = first; i < last; ++i) {
    const int r = guamp_backlight::sendDcsShort(io, cmds[i], d);
    if (r != guamp_backlight::kSent) return r;
    IOSleep(waits[i]);
  }
  return guamp_backlight::kSent;
}
void start() {
  unsigned enabled = 1, control = 1, selfTest = 0;   // default on (owner 28/10); guamp-backlight=0 to disable
  PE_parse_boot_argn("guamp-backlight", &enabled, sizeof(enabled));
  if (enabled != 1) return;
  PE_parse_boot_argn("guamp-backlight-control", &control, sizeof(control));
  PE_parse_boot_argn("guamp-backlight-test", &selfTest, sizeof(selfTest));
  gGccDisp = mapPhysical(kGccDispAhbPage, 0x1000);
  gDispcc = mapPhysical(kDispccPhys, 0x3000);
  gDsi = mapPhysical(kDsiPhys, 0x1000);
  if (!gGccDisp || !gDispcc || !gDsi) { IOLog("GuampBacklight: mapping failed\n"); gDsi = nullptr; return; }
  gDsiLock = IORecursiveLockAlloc();
  touch::gPanelCycle = &panelCycle;
  GuampBacklight *service = OSTypeAlloc(GuampBacklight);
  if (!service || !service->init() || !service->attach(IOService::getPlatform())) {
    IOLog("GuampBacklight: service not created\n");
    if (service) service->release();
    return;
  }
  if (!service->startBacklight(control == 1, selfTest == 1)) {
    IOLog("GuampBacklight: work loop not started\n");
    service->detach(IOService::getPlatform());
  }
  service->release();
}
}  // namespace backlight
