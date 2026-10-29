// SPDX-License-Identifier: BSD-2-Clause-Patent
// Render server watchdog for guamp-ca-metal=2 (zzzzzz-guamp-ca-metal.patch): pure decision, host tested (tests/ca_metal_host.cpp).
// Fed every few seconds with backboardd's pid (0 = not running) and the display's accepted-swap counter, it says when a Metal
// session is good (the boot count may be cleared) or bad (the next boots must use the software renderer).
//   good: backboardd up for kGcmGoodSec without any restart, and at least one frame presented since it started (when the counter exists)
//   bad:  kGcmMaxRestarts restarts of backboardd, or no frame within kGcmNoFrameSec of its start (boot UI always presents frames)
// A quiet screen presents no frames, so only the start of a backboardd instance is held to the frame rule.
// noFrameSec (29/10): the frame rule limit for this boot, 0 = kGcmNoFrameSec. Boots with one-shot work (guamp-ls-rebuild,
// guamp-install-staged-apps) keep the boot UI busy for minutes: GuampCaMetal.inc raises it to kGcmNoFrameOneShotSec there.
#pragma once
#include <stdint.h>

static const uint32_t kGcmGoodSec = 120, kGcmNoFrameSec = 60, kGcmNoFrameOneShotSec = 300, kGcmMaxRestarts = 2;

enum gcm_verdict { GCM_WAIT = 0, GCM_GOOD = 1, GCM_BAD_CRASH = 2, GCM_BAD_NOFRAMES = 3 };

struct gcm_state {
  int pid;            // backboardd instance being watched, 0 = none seen yet
  uint32_t since;     // seconds when that instance was first seen
  uint32_t frames0;   // swap counter then
  uint32_t restarts;  // instances after the first one
  bool framesSeen;    // the watched instance presented at least one frame
  bool started;       // a backboardd instance was seen at least once
  uint32_t noFrameSec;  // frame rule limit, 0 = kGcmNoFrameSec
};

static inline const char *gcm_name(gcm_verdict v) {
  return v == GCM_GOOD ? "good" : v == GCM_BAD_CRASH ? "crash loop" : v == GCM_BAD_NOFRAMES ? "no frames" : "wait";
}

// One observation; haveFrames false when the kernel has no swap counter (then only restarts decide).
static inline gcm_verdict gcm_step(gcm_state *s, uint32_t now, int pid, bool haveFrames, uint32_t frames) {
  if (pid != s->pid) {
    if (pid && s->started) s->restarts = s->restarts + 1;   // a new instance (the gap with pid 0 in between is not counted again)
    s->pid = pid;
    s->since = now;
    s->frames0 = frames;
    s->framesSeen = false;
    if (pid) s->started = true;
  }
  if (s->restarts >= kGcmMaxRestarts) return GCM_BAD_CRASH;
  if (!s->pid) return GCM_WAIT;
  if (haveFrames && frames != s->frames0) s->framesSeen = true;
  const uint32_t up = now - s->since;
  if (haveFrames && !s->framesSeen && up >= (s->noFrameSec ? s->noFrameSec : kGcmNoFrameSec)) return GCM_BAD_NOFRAMES;
  if (!s->restarts && up >= kGcmGoodSec && (s->framesSeen || !haveFrames)) return GCM_GOOD;
  return GCM_WAIT;
}
