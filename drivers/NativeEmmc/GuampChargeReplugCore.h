// SPDX-License-Identifier: BSD-2-Clause-Patent
// No hardware access: actions require explicit successful acknowledgement.
#ifndef GUAMP_CHARGE_REPLUG_CORE_H
#define GUAMP_CHARGE_REPLUG_CORE_H
#include <stdint.h>
namespace guamp_charge {
enum Action { None, BeginDetection, Configure };
inline uint8_t receiverMode(uint8_t value) { return static_cast<uint8_t>((value & 0xe0u) | 2u); }
inline unsigned inputMa(unsigned requested, uint8_t result) {
  if (requested > 3000) requested = 3000;
  return (result & 0x0e) || requested < 500 ? requested : 500;
}
struct Tracker {
  bool present = false, requested = false, configured = false;
  uint64_t began = 0, configuredAt = 0, retryAt = 0;
  unsigned attaches = 0;
  Action next(uint64_t ms, bool source, bool apsdDone, unsigned state, bool suspended) {
    if (!source) {
      present = requested = configured = false; retryAt = 0; return None;
    }
    if (!present) {
      present = true; requested = configured = false; retryAt = 0; ++attaches;
    }
    // Recover even if an unplug/replug was shorter than the polling interval.
    // Full, taper and thermal pauses are not interpreted as a failed connection.
    if (configured && (state == 7 || suspended) && ms >= configuredAt && ms-configuredAt >= 10000) {
      requested = configured = false; retryAt = ms;
    }
    if (ms < retryAt) return None;
    if (!requested) return BeginDetection;
    if (!configured && apsdDone) return Configure;
    if (!configured && ms >= began && ms-began >= 5000) {
      requested = false; retryAt = ms+1000;
    }
    return None;
  }
  void acknowledge(Action action, bool success, uint64_t ms) {
    if (!success) { retryAt = ms+1000; return; }
    if (action == BeginDetection) { requested = true; began = ms; }
    if (action == Configure) { configured = true; configuredAt = ms; retryAt = ms+10000; }
  }
};
}
#endif
