// SPDX-License-Identifier: BSD-2-Clause-Patent
// GPU hang detection and recovery bookkeeping for guamp-gpu-recover=1 (GuampAdrenoRecover.inc): pure decisions and the ring layout
// of a CP restart, host tested (tests/recover_host.cpp). No hardware access here.
//   hang:    work pending and the completed fence unchanged for hangMs (guamp-gpu-recover-ms, default kGrcHangMs), or for kGrcFaultMs
//            when an error interrupt (CP_HW_ERROR, RBBM_HANG_DETECT, AHB error, UCHE out of bounds) or a fault of the GPU SMMU
//            context bank was seen meanwhile. Jobs on the A610 take ~5 ms, the heaviest frames ~300 ms at 320 MHz (tb128fu).
//   budget:  at most kGrcMaxRecoveries in kGrcWindowMs; one more hang in that window, or a restart that fails its check, parks the
//            GPU for the rest of the boot (the caller then refuses submissions and reports to guamp-ca-metal).
#pragma once
#include <stdint.h>

static const uint32_t kGrcHangMs = 2000, kGrcMinHangMs = 250, kGrcMaxHangMs = 60000, kGrcFaultMs = 100;
static const uint32_t kGrcMaxRecoveries = 3, kGrcWindowMs = 60000;

// What the caller found while a stall had lasted kGrcFaultMs or more.
enum : uint32_t { GRC_CAUSE_IRQ = 1u << 0, GRC_CAUSE_SMMU = 1u << 1 };
enum grc_reason { GRC_NONE = 0, GRC_TIMEOUT = 1, GRC_IRQ = 2, GRC_SMMU = 3 };
enum grc_action { GRC_KEEP = 0, GRC_RECOVER = 1, GRC_PARK = 2 };

struct grc_state {
  uint32_t hangMs;
  uint32_t completed;     // completed fence at the last observation
  uint32_t since;         // ms: start of the current stall (last progress, or the submission that ended an idle ring)
  bool busy;              // work was pending at the last observation
  bool parked;
  uint32_t windowStart;   // ms: first recovery of the current window
  uint32_t inWindow;      // recoveries in that window
  uint32_t recoveries;    // restarts that passed their check
  uint32_t failures;      // restarts that did not
};

static inline const char *grc_name(grc_reason r) {
  return r == GRC_TIMEOUT ? "fence timeout" : r == GRC_IRQ ? "error interrupt" : r == GRC_SMMU ? "smmu fault" : "none";
}

static inline void grc_init(grc_state *s, uint32_t hangMs) {
  *s = grc_state{};
  s->hangMs = hangMs < kGrcMinHangMs ? kGrcMinHangMs : hangMs > kGrcMaxHangMs ? kGrcMaxHangMs : hangMs;
}

// Fence a is newer than fence b (the ring's 32-bit timestamps wrap).
static inline bool grc_after(uint32_t a, uint32_t b) { return int32_t(a - b) > 0; }

// A new submission. When the ring had nothing pending before it, the stall clock starts now: an idle ring is never a stall, and the
// time since the last observation must not count against the new work.
static inline void grc_submitted(grc_state *s, uint32_t now, uint32_t completed, uint32_t newestBefore) {
  if (grc_after(newestBefore, completed)) return;
  s->busy = true;
  s->completed = completed;
  s->since = now;
}

// One observation of the fence page (completed) and the newest fence handed out. Returns how long (ms) the oldest pending fence
// has not moved; 0 when nothing is pending or the fence just moved.
static inline uint32_t grc_track(grc_state *s, uint32_t now, uint32_t completed, uint32_t newest) {
  if (!grc_after(newest, completed)) {
    s->busy = false;
    s->completed = completed;
    return 0;
  }
  if (!s->busy || completed != s->completed) {
    s->busy = true;
    s->completed = completed;
    s->since = now;
    return 0;
  }
  return now - s->since;
}

// Why a stall of stalled ms is a hang, given the causes the caller collected (GRC_CAUSE_*); GRC_NONE while it is not one yet.
static inline grc_reason grc_cause(const grc_state *s, uint32_t stalled, uint32_t causes) {
  if (stalled >= s->hangMs) return GRC_TIMEOUT;
  if (stalled < kGrcFaultMs) return GRC_NONE;
  if (causes & GRC_CAUSE_SMMU) return GRC_SMMU;
  if (causes & GRC_CAUSE_IRQ) return GRC_IRQ;
  return GRC_NONE;
}

// A hang was found: recover, or park when the window's budget is spent. A parked GPU stays parked.
static inline grc_action grc_decide(grc_state *s, uint32_t now, grc_reason why) {
  if (s->parked || why == GRC_NONE) return GRC_KEEP;
  if (!s->inWindow || now - s->windowStart >= kGrcWindowMs) {
    s->windowStart = now;
    s->inWindow = 0;
  }
  if (s->inWindow >= kGrcMaxRecoveries) {
    s->parked = true;
    return GRC_PARK;
  }
  s->inWindow = s->inWindow + 1;
  return GRC_RECOVER;
}

// The restart's check: ok resumes (every fence was signalled, so nothing is pending), a failure parks.
static inline grc_action grc_done(grc_state *s, bool ok) {
  if (!ok) {
    s->failures = s->failures + 1;
    s->parked = true;
    return GRC_PARK;
  }
  s->recoveries = s->recoveries + 1;
  s->busy = false;
  return GRC_KEEP;
}

// Ring layout of a restart. After the reset the CP reads from start (CP_RB_RPTR, 0 on the A610), while the ring object keeps its
// write index next (NativeAdrenoRing::next_ is private and its next submission is written there), so the words in between are
// rewritten: CP_ME_INIT (a6xx_cp_init, 9 words) first, then a CP_MEM_WRITE of a check value (4 words), then one CP_NOP whose payload
// ends exactly at next. That needs a distance of kGrcHeadWords, or at least kGrcHeadWords + 1 for the NOP header; a shorter
// distance is first lengthened with kernel submissions of one CP_NOP (kGrcPadWords each with the ring's 7-word fence), written
// while the CP is still stopped.
static const uint32_t kGrcMeInitWords = 9, kGrcCheckWords = 4, kGrcHeadWords = kGrcMeInitWords + kGrcCheckWords, kGrcPadWords = 8;
static const uint32_t kGrcMaxType7Count = 0x3fff;

struct grc_ring_plan {
  uint32_t pads;         // kernel padding submissions needed first (then plan again with the new write index)
  uint32_t distance;     // words from start to next once padded
  uint32_t nopAt;        // ring index of the CP_NOP header
  uint32_t nopPayload;   // its payload words
  bool nop;              // false when the head alone reaches next
};

static inline bool grc_plan_ring(uint32_t start, uint32_t next, uint32_t words, grc_ring_plan *p) {
  if (!p || words < 64 || (words & (words - 1)) || words > kGrcMaxType7Count + 1 || start >= words || next >= words) return false;
  *p = grc_ring_plan{};
  uint32_t d = (next - start) & (words - 1);
  while (d != kGrcHeadWords && d < kGrcHeadWords + 1) {
    d = d + kGrcPadWords;
    p->pads = p->pads + 1;
  }
  p->distance = d;
  p->nop = d > kGrcHeadWords;
  p->nopAt = (start + kGrcHeadWords) & (words - 1);
  p->nopPayload = p->nop ? d - kGrcHeadWords - 1 : 0;
  return true;
}

// The PM4 type-7 headers the caller built (AdrenoPM4Type7): CP_ME_INIT with 8 words, CP_MEM_WRITE with 3, CP_NOP with nopPayload.
struct grc_head_words {
  uint32_t meInit, memWrite, nop;
};

// Writes the head of a plan without padding into the ring (a power of two of words), wrapping as the CP reads it.
static inline bool grc_fill_ring(volatile uint32_t *ring, uint32_t words, uint32_t start, const grc_ring_plan *p,
                                 const grc_head_words *h, uint64_t checkIova, uint32_t checkValue) {
  if (!ring || !p || !h || p->pads) return false;
  const uint32_t mask = words - 1;
  const uint32_t head[kGrcHeadWords] = {h->meInit, 0x2f, 0x00000003, 0x20000000, 0, 0, 0, 0, 0,
                                        h->memWrite, uint32_t(checkIova), uint32_t(checkIova >> 32), checkValue};
  for (uint32_t i = 0; i < kGrcHeadWords; i = i + 1) ring[(start + i) & mask] = head[i];
  if (p->nop) ring[p->nopAt] = h->nop;
  return true;
}
