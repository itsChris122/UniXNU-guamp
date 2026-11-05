// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp torch policy without hardware access (host-tested by tests/torch_host.cpp): request level -> flash LED current
// code, the clamps and the auto-off timer. Used by GuampTorch.inc.
//   PMI632 flash LED (qcom,qpnp-flash-led-v2, guamp torch_0): ires-ua 12500 -> IRES code 0, one current step = 12.5 mA,
//   TGR_CURRENT code = round(mA * 1000 / 12500) - 1 (leds-qpnp-flash-v2.c get_current_reg_code), DT torch max 500 mA
//   (qcom,max-current), Linux torch default 300 mA (qcom,current-ma). This port stays far below: default 100 mA, hard
//   cap 200 mA.
#ifndef GUAMP_TORCH_CORE_H
#define GUAMP_TORCH_CORE_H
#include <stdint.h>

namespace guamp_torch {
constexpr unsigned kStepUa = 12500;             // ires-ua
constexpr unsigned kMinMa = 25;                 // code 1; code 0 (12.5 mA) is barely visible
constexpr unsigned kDefaultMa = 100;            // code 7
constexpr unsigned kCapMa = 200;                // code 15, 40 % of the DT torch maximum
constexpr unsigned kDtMaxMa = 500;
constexpr unsigned kDefaultTimeoutS = 600;      // 10 minutes
constexpr unsigned kMaxTimeoutS = 1800;

constexpr unsigned clampMa(unsigned ma) { return ma < kMinMa ? kMinMa : ma > kCapMa ? kCapMa : ma; }
constexpr uint8_t codeForMa(unsigned ma) {
  return static_cast<uint8_t>((clampMa(ma) * 1000u + kStepUa / 2) / kStepUa - 1u);
}
constexpr unsigned maForCode(uint8_t code) { return (static_cast<unsigned>(code) + 1u) * kStepUa / 1000u; }
// A request is a level in permille of the configured maximum (iOS TorchLevel 0.0-1.0 scaled by 1000): 0 = off, any
// other value is at least kMinMa and never above maxMa (itself clamped).
constexpr unsigned maForPermille(unsigned permille, unsigned maxMa) {
  return permille == 0 ? 0u
         : clampMa(static_cast<unsigned>((static_cast<uint64_t>(clampMa(maxMa)) * (permille > 1000u ? 1000u : permille)) / 1000u));
}
constexpr unsigned clampTimeoutS(unsigned s) { return s == 0 ? kDefaultTimeoutS : s > kMaxTimeoutS ? kMaxTimeoutS : s; }
constexpr bool expired(uint64_t onMs, uint64_t nowMs, unsigned timeoutS) {
  return nowMs >= onMs && nowMs - onMs >= static_cast<uint64_t>(timeoutS) * 1000u;
}
static_assert(codeForMa(100) == 7 && maForCode(7) == 100, "100 mA");
static_assert(codeForMa(10000) == codeForMa(kCapMa) && maForCode(codeForMa(kCapMa)) <= kDtMaxMa / 2, "cap");

// ---- torch owners (06/11) --------------------------------------------------------------------------------------------
// Two clients drive the same LED as a torch: the Control Center flashlight (GuampTorch service, SpringBoard / AVFlashlight
// fixup) and the camera (AVCaptureDevice TorchLevel through AppleH6CamIn). Each one only sets or clears its own level and
// the LED shows the brighter of the two, so the camera switching its torch off on close no longer kills a flashlight
// turned on from the Control Center, and the other way round.
// The PMIC thread evaluates torchAction() on every poll, not only on a new request: after a photo flash (which takes the
// LED) the torch comes back by itself. After an automatic switch off (auto-off timer, LED fault, failed switch on) the
// torch is held off until the next request of any owner, so a fault never turns into a retry loop.
enum TorchOwner : uint8_t { kOwnerSystem = 0, kOwnerCamera = 1, kOwners = 2 };
constexpr unsigned clampPermille(unsigned p) { return p > 1000u ? 1000u : p; }
constexpr unsigned ownersPermille(unsigned system, unsigned camera) {
  return clampPermille(system) > clampPermille(camera) ? clampPermille(system) : clampPermille(camera);
}
enum TorchAction : uint8_t { kTorchKeep = 0, kTorchOn = 1, kTorchOff = 2 };
// wantMa = maForPermille(owners), 0 = off; held = automatic switch off since the last request; flashLit = a photo flash
// owns the LED now; on / onMa = the torch state.
constexpr TorchAction torchAction(unsigned wantMa, bool held, bool flashLit, bool on, unsigned onMa) {
  return flashLit ? kTorchKeep
         : (wantMa && !held) ? (on && onMa == maForCode(codeForMa(wantMa)) ? kTorchKeep : kTorchOn)
         : on ? kTorchOff : kTorchKeep;
}
static_assert(torchAction(100, false, false, false, 0) == kTorchOn && torchAction(100, false, true, false, 0) == kTorchKeep &&
              torchAction(100, true, false, false, 0) == kTorchKeep && torchAction(0, false, false, true, 100) == kTorchOff &&
              torchAction(100, false, false, true, 100) == kTorchKeep && torchAction(50, false, false, true, 100) == kTorchOn,
              "torch action table");

// ---- photo flash (same LED, same TGR_CURRENT register, hardware safety timer ON) ------------------------------------
// guamp DT flash_0: max-current 1500 mA, current-ma 1000, duration-ms 1280, thermal-derate-current 200/500/1000 mA;
// Linux (qpnp_flash_led_calc_bharger_max_current) allows at most 1500 mA and less from battery OCV/ESR, which this port
// cannot compute yet. So: default 500 mA, hard cap 750 mA (half the DT maximum), 200 mA when the LED reports any
// over-temperature level (LED_STATUS2 bits 2:0) and no flash at all after a VPH droop fault (LED_STATUS1 bit 4).
// Every flash runs with the PMIC safety timer (SAFETY_TMR bit 7 | duration / 10 - 1, Linux get_safety_timer_code)
// set to the flash length, so the hardware switches the LED off even if this kernel stalls.
// Pre-flash = a torch-level pulse (metering, at most the torch cap), also with the safety timer.
// After a main flash: at least kFlashMinGapMs, and an energy budget (mA x ms over a sliding window).
constexpr unsigned kFlashDefaultMa = 500, kFlashCapMa = 750, kFlashHotMa = 200, kFlashDtMaxMa = 1500;
constexpr unsigned kFlashMinMs = 20, kFlashMaxMs = 500, kFlashDefaultMs = 200;
constexpr unsigned kPreflashMaxMs = 1000;
constexpr unsigned kFlashMinGapMs = 1500;
constexpr uint64_t kFlashBudgetMaMs = 750ull * 500ull * 3ull;   // three full-length flashes at the cap ...
constexpr unsigned kFlashBudgetWindowMs = 30000;               // ... per 30 s
constexpr uint8_t kSafetyTmrEnable = 0x80;

enum FlashKind : uint8_t { kFlashOff = 0, kFlashPre = 1, kFlashMain = 2 };

constexpr unsigned clampFlashMa(unsigned ma, unsigned cap) { return ma < kMinMa ? kMinMa : ma > cap ? cap : ma; }
// TGR_CURRENT code for a flash current (same 12.5 mA step as the torch, 7 bits)
constexpr uint8_t flashCode(unsigned ma, unsigned cap) {
  return static_cast<uint8_t>((clampFlashMa(ma, cap) * 1000u + kStepUa / 2) / kStepUa - 1u);
}
constexpr unsigned clampFlashMs(unsigned ms, unsigned maxMs) { return ms < kFlashMinMs ? kFlashMinMs : ms > maxMs ? maxMs : ms; }
// SAFETY_TMR value: enable bit + 10 ms steps (code 0 = 10 ms ... 127 = 1280 ms), rounded up so the timer never cuts a
// flash shorter than asked.
constexpr uint8_t safetyTimer(unsigned ms) {
  return static_cast<uint8_t>(kSafetyTmrEnable | ((ms + 9u) / 10u >= 128u ? 127u : ms < 10u ? 0u : (ms + 9u) / 10u - 1u));
}
// Current allowed now from the LED status registers: 0 = refuse.
constexpr unsigned flashCapFor(uint8_t status1, uint8_t status2) {
  return (status1 & 0x10u) ? 0u : (status2 & 0x07u) ? kFlashHotMa : kFlashCapMa;
}

// Main-flash admission: minimum gap since the previous main flash and the mA x ms budget over the window.
struct FlashBudget {
  uint64_t lastEndMs = 0;
  bool any = false;
  uint64_t t[8] = {}, e[8] = {};    // ring of recent main flashes: end time, mA x ms
  unsigned head = 0;
  uint64_t usedSince(uint64_t nowMs) const {
    uint64_t s = 0;
    for (unsigned i = 0; i < 8; ++i)
      if (e[i] && nowMs >= t[i] && nowMs - t[i] < kFlashBudgetWindowMs) s += e[i];
    return s;
  }
  bool allow(uint64_t nowMs, unsigned ma, unsigned ms) const {
    if (any && (nowMs < lastEndMs || nowMs - lastEndMs < kFlashMinGapMs)) return false;
    return usedSince(nowMs) + static_cast<uint64_t>(ma) * ms <= kFlashBudgetMaMs;
  }
  void record(uint64_t endMs, unsigned ma, unsigned ms) {
    t[head] = endMs; e[head] = static_cast<uint64_t>(ma) * ms; head = (head + 1) & 7u;
    lastEndMs = endMs; any = true;
  }
};
static_assert(flashCode(750, kFlashCapMa) == 59 && maForCode(59) == 750, "750 mA code");
static_assert(flashCode(5000, kFlashCapMa) == flashCode(kFlashCapMa, kFlashCapMa), "flash cap");
static_assert(safetyTimer(200) == (0x80 | 19) && safetyTimer(5000) == 0xff && safetyTimer(1) == 0x80, "safety timer");
}  // namespace guamp_torch
#endif
