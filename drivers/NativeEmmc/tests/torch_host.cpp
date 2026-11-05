// Host test of GuampTorchCore.h: request level -> current code, clamps, auto-off timer.
#include <stdio.h>
#include "../GuampTorchCore.h"
using namespace guamp_torch;

static int failures;
static void check(bool ok, const char *what, unsigned a, unsigned b) {
  if (!ok) { printf("FAIL %s (%u, %u)\n", what, a, b); failures = failures + 1; }
}

int main() {
  // Codes match leds-qpnp-flash-v2.c get_current_reg_code for ires 12500 uA: DIV_ROUND_CLOSEST(mA * 1000, 12500) - 1
  for (unsigned ma = kMinMa; ma <= kCapMa; ++ma) {
    const unsigned ref = (ma * 1000u + 6250u) / 12500u - 1u;
    check(codeForMa(ma) == ref, "code = Linux code", ma, codeForMa(ma));
    check(maForCode(codeForMa(ma)) <= kCapMa, "never above the cap", ma, maForCode(codeForMa(ma)));
  }
  check(codeForMa(0) == codeForMa(kMinMa), "below minimum clamps up", 0, codeForMa(0));
  check(codeForMa(500) == 15 && maForCode(15) == 200, "DT maximum clamps to 200 mA", codeForMa(500), maForCode(15));
  check(codeForMa(100) == 7, "default 100 mA = code 7", codeForMa(100), 7);
  // Levels
  check(maForPermille(0, 100) == 0, "level 0 = off", maForPermille(0, 100), 0);
  check(maForPermille(1000, 100) == 100, "level 1.0 = max", maForPermille(1000, 100), 100);
  check(maForPermille(5000, 100) == 100, "level above 1.0 = max", maForPermille(5000, 100), 100);
  check(maForPermille(1, 100) == kMinMa, "tiny level = minimum", maForPermille(1, 100), kMinMa);
  check(maForPermille(500, 160) == 80, "half level", maForPermille(500, 160), 80);
  check(maForPermille(1000, 9999) == kCapMa, "max from boot-arg capped", maForPermille(1000, 9999), kCapMa);
  for (unsigned p = 1; p <= 1000; ++p)
    for (unsigned m = 0; m <= 600; m += 50) {
      const unsigned ma = maForPermille(p, m);
      check(ma >= kMinMa && ma <= kCapMa && ma <= clampMa(m), "level in range", p, m);
    }
  // Timer
  check(clampTimeoutS(0) == 600 && clampTimeoutS(99999) == kMaxTimeoutS && clampTimeoutS(30) == 30, "timeout clamp",
        clampTimeoutS(0), clampTimeoutS(99999));
  check(!expired(1000, 1000 + 599999, 600), "not yet at 599.999 s", 0, 0);
  check(expired(1000, 1000 + 600000, 600), "off at 600 s", 0, 0);
  check(!expired(5000, 1000, 600), "clock before on time: no expiry", 0, 0);
  // Owners (Control Center flashlight + camera) and the per-poll action, against a model of the PMIC poll of GuampTorch.inc
  check(ownersPermille(0, 0) == 0 && ownersPermille(1000, 0) == 1000 && ownersPermille(300, 600) == 600 &&
        ownersPermille(5000, 0) == 1000, "owners: the brighter one", ownersPermille(300, 600), 600);
  {
    struct Led { bool on = false; unsigned ma = 0; bool held = false; bool flash = false; unsigned switches = 0; bool failNext = false; };
    Led led;
    unsigned lv[kOwners] = {};
    auto poll = [&](bool request) {
      if (request) led.held = false;
      const unsigned want = maForPermille(ownersPermille(lv[kOwnerSystem], lv[kOwnerCamera]), 100);
      switch (torchAction(want, led.held, led.flash, led.on, led.ma)) {
        case kTorchOn:
          if (led.failNext) { led.failNext = false; led.on = false; led.ma = 0; led.held = true; break; }
          if (!led.on) led.switches++;
          led.on = true; led.ma = maForCode(codeForMa(want)); break;
        case kTorchOff: led.on = false; led.ma = 0; break;
        default: break;
      }
    };
    lv[kOwnerSystem] = 1000; poll(true);
    check(led.on && led.ma == 100, "Control Center on", led.ma, 100);
    lv[kOwnerCamera] = 500; poll(true);
    check(led.on && led.ma == 100, "camera dimmer: Control Center level stays", led.ma, 100);
    lv[kOwnerCamera] = 0; poll(true);
    check(led.on && led.ma == 100, "camera torch off keeps the flashlight", led.ma, 100);
    led.flash = true; led.on = false; led.ma = 0;          // flashFire: off("flash")
    for (int k = 0; k < 8; ++k) poll(false);
    check(!led.on, "flash owns the LED", led.on, 0);
    led.flash = false; poll(false);
    check(led.on && led.ma == 100, "torch back after the flash", led.ma, 100);
    led.on = false; led.ma = 0; led.held = true;           // auto-off timer
    for (int k = 0; k < 8; ++k) poll(false);
    check(!led.on, "auto-off holds until the next request", led.on, 0);
    poll(true);
    check(led.on, "new request after auto-off", led.on, 1);
    lv[kOwnerSystem] = 0; poll(true);
    check(!led.on, "Control Center off", led.on, 0);
    lv[kOwnerCamera] = 1000; led.failNext = true; poll(true);
    const unsigned sw = led.switches;
    for (int k = 0; k < 40; ++k) poll(false);
    check(!led.on && led.switches == sw, "failed switch on: no retry loop", led.switches, sw);
    poll(true);
    check(led.on && led.ma == 100, "retry on the next request", led.ma, 100);
    lv[kOwnerCamera] = 0; poll(true);
    check(!led.on, "camera off, nobody else: off", led.on, 0);
  }
  printf("torch core: %s\n", failures ? "FAIL" : "PASS");
  return failures ? 1 : 0;
}
