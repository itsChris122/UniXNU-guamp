// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp PM6125 power-on (PON) module: decoding of the key reset configuration and of the reset/power-off reasons, plus
// the watch of a held Power+RESIN combination. No hardware access: the owner (PmicService.inc) reads the registers, so
// all of it runs on the host (tests/pon_host.cpp).
//
// Why (BLOCCO-SCHERMO-20261029): two "freezes" after a Power click (wifi3 355.9 s, boot before 809.9 s) both read
// PON_RT_STS with KPDPWR and RESIN (Volume Down) set together and never released; iOS never asked for display off, and
// all logs stopped 6-15 s later = the PMIC's own KPDPWR_RESIN S1/S2 reset. The user did not touch Volume Down, so RESIN
// is suspect (pull-up / debounce left as the boot loader set them). This module tells us what the PMIC is set to do and
// logs the countdown while the combination is held, so the last kernel line says how close the PMIC reset was.
//
// Register layout (PM6125 qcom,power-on@800, Linux drivers/input/misc/qpnp-power-on.c offsets, no code copied):
//   +0x05 PERPH_SUBTYPE (4/5 = gen2: reasons at +0xC0..), +0x08.. gen1 reasons, +0x10 INT_RT_STS (bit0 KPDPWR_N,
//   bit1 RESIN_N, bit2 CBLPWR_N, bit4 RESIN_BARK, bit5 KPDPWR_RESIN_BARK), +0x40..43 KPDPWR S1 timer / S2 timer /
//   S2 type / S2 enable (bit7), +0x44..47 RESIN, +0x48..4B KPDPWR_RESIN, +0x5A/5B PS_HOLD type / enable,
//   +0x70 PULL_CTL (bit0 RESIN pull-up, bit1 KPDPWR pull-up), +0x71 DBC_CTL, +0x74 S3_SRC, +0xC0.. gen2 reasons.
#ifndef GUAMP_PON_CORE_H
#define GUAMP_PON_CORE_H
#include <stdint.h>

namespace guamp_pon {
constexpr uint16_t kBase = 0x0800;
constexpr uint16_t kSubtype = kBase + 0x05, kRtSts = kBase + 0x10, kPullCtl = kBase + 0x70, kDbcCtl = kBase + 0x71,
                   kS3Src = kBase + 0x74;
constexpr uint8_t kKpdpwr = 1u << 0, kResin = 1u << 1, kResinBark = 1u << 4, kComboBark = 1u << 5;
constexpr uint8_t kPullResin = 1u << 0, kPullKpdpwr = 1u << 1;

// One S1/S2 reset source: KPDPWR (+0x40), RESIN (+0x44), KPDPWR_RESIN (+0x48)
enum Source : unsigned { kSrcKpdpwr = 0, kSrcResin = 1, kSrcCombo = 2 };
constexpr uint16_t sourceBase(unsigned s) { return static_cast<uint16_t>(kBase + 0x40 + 4 * s); }
inline const char *sourceName(unsigned s) { return s == kSrcKpdpwr ? "KPDPWR" : s == kSrcResin ? "RESIN" : "KPDPWR+RESIN"; }

inline uint32_t s1Ms(uint8_t reg) {
  static const uint16_t t[16] = {0, 32, 56, 80, 138, 184, 272, 408, 608, 904, 1352, 2048, 3072, 4480, 6720, 10256};
  return t[reg & 0xf];
}
inline uint32_t s2Ms(uint8_t reg) {
  static const uint16_t t[8] = {0, 10, 50, 100, 250, 500, 1000, 2000};
  return t[reg & 0x7];
}
inline const char *resetType(uint8_t reg) {
  switch (reg & 0xf) {
  case 1: return "warm-reset";
  case 4: return "shutdown";
  case 5: return "dvdd-shutdown";
  case 7: return "hard-reset";
  case 8: return "dvdd-hard-reset";
  default: return "other";
  }
}

struct Reset {            // one decoded source
  uint32_t s1, s2;         // ms
  uint8_t type;            // raw S2_CNTL & 0xf
  bool enabled;            // S2_CNTL2 bit 7
  uint32_t totalMs() const { return s1 + s2; }
};
inline Reset decode(const uint8_t r[4]) {
  Reset x;
  x.s1 = s1Ms(r[0]);
  x.s2 = s2Ms(r[1]);
  x.type = static_cast<uint8_t>(r[2] & 0xf);
  x.enabled = (r[3] & 0x80u) != 0;
  return x;
}

// Gen2 PON (subtype 4/5) keeps the reasons at +0xC0.., gen1 at +0x08..
inline bool gen2(uint8_t subtype) { return subtype == 4 || subtype == 5; }

// Watch of a held combination, fed with every debounced poll (raw PON_RT_STS byte, time in ms). report() is true once
// per second of holding (and at the start) so the owner logs: the last line before a PMIC reset gives held ms and how
// far the armed reset was.
struct ComboWatch {
  uint8_t mask = kKpdpwr | kResin;   // the keys that must all be held
  bool active = false;
  uint32_t startMs = 0, lastReportMs = 0, longestMs = 0, combos = 0;
  // returns true when the owner should log now; *heldMs = how long both have been held
  bool feed(uint8_t rt, uint32_t nowMs, uint32_t *heldMs) {
    const bool both = (rt & mask) == mask;
    if (!both) {
      if (active) {
        const uint32_t held = nowMs - startMs;
        if (held > longestMs) longestMs = held;
      }
      active = false;
      return false;
    }
    if (!active) {
      active = true;
      startMs = nowMs;
      lastReportMs = nowMs;
      combos = combos + 1;
      *heldMs = 0;
      return true;
    }
    *heldMs = nowMs - startMs;
    if (nowMs - lastReportMs >= 1000) {
      lastReportMs = nowMs;
      return true;
    }
    return false;
  }
};
}  // namespace guamp_pon
#endif
