// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp touch: pure decisions of TouchService.inc (no hardware), host-tested by tests/touch_policy_host.cpp.
//   guamp_touch_wake   which firmware report wakes the display with guamp-touch-wake=<bits>, and whether the firmware
//                      really is in gesture mode after the display-off swap.
//   guamp_touch_guard  the long "lost finger up" guard: what to do once no interrupt and no valid report came for
//                      guamp-touch-release-ms, after reading the controller's event stack again.
#ifndef GUAMP_TOUCH_POLICY_CORE_H
#define GUAMP_TOUCH_POLICY_CORE_H
#include <stdint.h>

namespace guamp_touch_wake {
// guamp-touch-wake=<bits> (default 0 = off, the display wakes only by the Power key).
// Gesture ids are those of himax_common.c gest_event[]: 0x80 double tap, 0x81 "finger gesture" (the only other
// single-contact id the Himax table knows; HX83102D has no register selecting gestures, the firmware decides),
// 0x01..0x0F letters / swipes, 0x1D..0x54 others.
enum : unsigned {
  kFinger = 1u,        // arm the firmware gesture mode at display off; wake on id 0x81
  kOtherGestures = 2u, // also wake on any other listed id except 0x80
  kDoubleTap = 4u,     // also wake on 0x80 (double tap: off unless asked for)
  kAnyReport = 8u,     // also wake on any non-empty report the firmware sends while in gesture mode (no gesture id)
  kAll = 15u,
};
constexpr uint8_t kIdDoubleTap = 0x80, kIdFinger = 0x81;
constexpr uint8_t kGestureReply = 0x22, kBorderReply = 0x66;
constexpr uint32_t kSmwpOn = 0xA55AA55Au;

enum Verdict { kNone, kWake, kIgnored };
struct Decision {
  Verdict verdict;
  const char *why;
};

inline const char *idName(int id) {
  if (id == kIdDoubleTap) return "double tap";
  if (id == kIdFinger) return "finger gesture (single contact)";
  if (id >= 0x01 && id <= 0x0F) return "letter / swipe";
  return id ? "other gesture" : "no gesture id";
}

// id: hx_parse_gesture of the stack (0 = no gesture report); readOk: the stack read worked; nonEmpty: some byte non 0.
inline Decision decide(unsigned mode, int id, bool readOk, bool nonEmpty) {
  if (!readOk) return {kNone, "stack read failed"};
  if (id == kIdFinger) return (mode & kFinger) ? Decision{kWake, "0x81 accepted"} : Decision{kIgnored, "0x81 not enabled"};
  if (id == kIdDoubleTap)
    return (mode & kDoubleTap) ? Decision{kWake, "double tap accepted (bit 4)"} : Decision{kIgnored, "double tap excluded (bit 4 off)"};
  if (id) return (mode & kOtherGestures) ? Decision{kWake, "gesture accepted (bit 2)"} : Decision{kIgnored, "gesture not enabled (bit 2 off)"};
  if (!nonEmpty) return {kNone, "empty stack"};
  return (mode & kAnyReport) ? Decision{kWake, "firmware report in gesture mode accepted (bit 8)"} : Decision{kNone, "no gesture"};
}

// After the display-off swap (hx_overlay_swap: 1 on request, 2 forced, 0 none, <0 error): the gesture overlay is in
// SRAM, the firmware took the reply and the SMWP word reads A55AA55A. Otherwise the caller stops scanning (sense off).
inline bool gestureActive(int swap, int loaded, int gestureIdx, uint8_t reply, uint32_t smwp) {
  return (swap == 1 || swap == 2) && gestureIdx > 0 && loaded == gestureIdx && reply == kGestureReply && smwp == kSmwpOn;
}

// After the display-on swap: border overlay loaded and the firmware running (central state 05). Otherwise the caller
// reloads the whole firmware (the rescue path of hx83102d_0f_overlay).
inline bool resumeOk(int swap, int loaded, int borderIdx, uint8_t reply, uint32_t central) {
  return (swap == 1 || swap == 2) && borderIdx > 0 && loaded == borderIdx && reply == kBorderReply && (central & 0xFF) == 0x05;
}
}  // namespace guamp_touch_wake

namespace guamp_touch_guard {
// The Himax firmware reports when a finger lands or moves and then stays silent while it rests: silence is not a
// lift. Once guamp-touch-release-ms passed without interrupts or valid reports the event stack is read again:
//   bus error              release (the controller is not answering, iOS must not keep a stuck contact)
//   valid "all up" report  release (point_cnt 0xFF or 0 fingers)
//   contacts / no data     keep the contact and re-arm the guard
// and guamp-touch-release-max-ms (default 30000, 0 = never) of silence releases in any case, the last safety net.
enum Verdict { kRearmContacts, kRearmNoData, kReleaseUp, kReleaseBus, kReleaseLimit };

// readRc: hx_read_event; parsed: hx_parse_event (HX_OK = 0); pointCnt / count: from the parsed report;
// silentMs: since the last interrupt or valid report; maxMs: the safety net (0 = none).
inline Verdict verdict(int readRc, int parsed, uint8_t pointCnt, unsigned count, uint64_t silentMs, unsigned maxMs) {
  if (maxMs && silentMs >= maxMs) return kReleaseLimit;
  if (readRc) return kReleaseBus;
  if (parsed == 0 && !count && (pointCnt == 0xFF || !(pointCnt & 0x0F))) return kReleaseUp;
  if (parsed == 0 && (count || (pointCnt & 0x0F))) return kRearmContacts;
  return kRearmNoData;
}
inline bool releases(Verdict v) { return v == kReleaseUp || v == kReleaseBus || v == kReleaseLimit; }
inline const char *name(Verdict v) {
  switch (v) {
    case kRearmContacts: return "firmware still reports contacts, guard re-armed";
    case kRearmNoData: return "no report in the stack (finger at rest), guard re-armed";
    case kReleaseUp: return "firmware reports 0 contacts, contact released";
    case kReleaseBus: return "bus error reading the stack, contact released";
    case kReleaseLimit: return "safety limit reached, contact released";
  }
  return "?";
}
}  // namespace guamp_touch_guard
#endif
