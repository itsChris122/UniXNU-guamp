// SPDX-License-Identifier: BSD-2-Clause-Patent
// QG register scaling and an explicitly estimated charge balance. No hardware writes.
#ifndef GUAMP_BATTERY_CORE_H
#define GUAMP_BATTERY_CORE_H
#include <stdint.h>
namespace guamp_battery {
inline bool fullAtProfileVoltage(bool external, unsigned chargerState, unsigned uv) {
  // The existing conservative charger ceiling can terminate below the pack's
  // full voltage. A termination code alone must never turn a saved 94% into 100%.
  // JK50-SWD profile: CC/CV threshold 4.390 V, maximum 4.400 V.
  return external && chargerState == 5 && uv >= 4390000 && uv <= 4600000;
}
inline bool decode(unsigned subtype, uint16_t voltage, uint16_t current,
                   unsigned &uv, int &appleUa) {
  if ((subtype != 3 && subtype != 4) || voltage == 0x8000 || current == 0x8000) return false;
  uv = static_cast<unsigned>((194637ULL * voltage) / 1000);
  if (uv < 2800000 || uv > 4600000) return false;
  int signedRaw = current >= 0x8000 ? static_cast<int>(current) - 65536 : current;
  // QG is negative while charging. IOPMPowerSource uses positive charging current.
  appleUa = -static_cast<int>((static_cast<int64_t>(signedRaw) * (subtype == 3 ? 152588 : 305176)) / 1000);
  return true;
}
struct Balance {
  int64_t charge = 0, full = 0;
  uint64_t lastNs = 0;
  int lastUa = 0;
  bool ready = false, interrupted = false;
  void seed(unsigned percent, unsigned fullMah, uint64_t ns, int ua) {
    full = static_cast<int64_t>(fullMah) * 3600000;
    charge = full * (percent <= 100 ? percent : 100) / 100;
    lastNs = ns; lastUa = ua; ready = fullMah != 0; interrupted = false;
  }
  bool sample(uint64_t ns, int ua, bool fullVerified) {
    if (!ready) return false;
    bool integrated = false;
    if (!interrupted && ns >= lastNs) {
      uint64_t ms = (ns - lastNs) / 1000000;
      // Do not invent consumption through a missing/stalled observation interval.
      if (ms <= 15000) {
        charge += (static_cast<int64_t>(lastUa) + ua) * static_cast<int64_t>(ms) / 2000;
        integrated = true;
      }
    }
    if (fullVerified) charge = full;
    if (charge < 0) charge = 0;
    if (charge > full) charge = full;
    lastNs = ns; lastUa = ua; interrupted = false;
    return integrated;
  }
  unsigned percent() const { return ready ? static_cast<unsigned>((charge * 100 + full / 2) / full) : 0; }
};
}
#endif
