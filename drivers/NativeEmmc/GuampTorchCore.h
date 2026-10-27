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
}  // namespace guamp_torch
#endif
