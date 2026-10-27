// Host test of the guamp-ca-metal render server watchdog decision (GuampCaMetalCore.h).
#include <cstdio>
#include "../GuampCaMetalCore.h"

static unsigned checks, failures;
static void expect(bool ok, const char *what) {
  checks = checks + 1;
  if (!ok) {
    failures = failures + 1;
    std::printf("FAIL: %s\n", what);
  }
}

// Feeds a constant pid and a frame counter that moves once at moveAt (0 = never) until a verdict or the time limit.
static gcm_verdict run(int pid, bool haveFrames, uint32_t moveAt, uint32_t limit, uint32_t *at) {
  gcm_state s = {};
  uint32_t frames = 100;
  for (uint32_t t = 5; t <= limit; t += 5) {
    if (moveAt && t >= moveAt) frames = 101;
    const gcm_verdict v = gcm_step(&s, t, pid, haveFrames, frames);
    if (v != GCM_WAIT) {
      *at = t;
      return v;
    }
  }
  *at = 0;
  return GCM_WAIT;
}

int main() {
  uint32_t at = 0;
  expect(run(55, true, 10, 600, &at) == GCM_GOOD && at == 125, "frames after start, stable: good after 120 s of uptime");
  expect(run(55, true, 0, 600, &at) == GCM_BAD_NOFRAMES && at == 65, "no frame at all: bad after 60 s");
  expect(run(55, false, 0, 600, &at) == GCM_GOOD && at == 125, "no swap counter: restarts alone decide");
  expect(run(0, true, 10, 600, &at) == GCM_WAIT, "backboardd never seen: keep waiting");
  {
    // one crash (pid gone for one sample, then a new instance) is one restart, not two; the session never counts as good
    gcm_state s = {};
    gcm_verdict v = GCM_WAIT;
    const int pids[] = {55, 55, 0, 61, 61, 61};
    uint32_t t = 0, frames = 1;
    for (int pid : pids) {
      t += 5;
      frames = frames + 1;
      v = gcm_step(&s, t, pid, true, frames);
    }
    expect(s.restarts == 1 && v == GCM_WAIT, "a single restart is counted once");
    for (int i = 0; i < 60; ++i) {
      t += 5;
      frames = frames + 1;
      v = gcm_step(&s, t, 61, true, frames);
    }
    expect(v == GCM_WAIT, "after a restart the session is never confirmed good");
    v = gcm_step(&s, t + 5, 70, true, frames);
    expect(v == GCM_BAD_CRASH && s.restarts == 2, "the second restart is a crash loop");
  }
  {
    // a quiet screen later in the session does not matter once the instance presented frames
    gcm_state s = {};
    gcm_verdict v = gcm_step(&s, 5, 55, true, 1);
    v = gcm_step(&s, 10, 55, true, 2);
    for (uint32_t t = 15; t <= 120 && v == GCM_WAIT; t += 5) v = gcm_step(&s, t, 55, true, 2);
    expect(v == GCM_WAIT, "not good before 120 s of uptime");
    v = gcm_step(&s, 125, 55, true, 2);
    expect(v == GCM_GOOD, "good at 120 s with a quiet screen after the first frames");
  }
  {
    // a new instance must present its own frames: counter values from the old one do not count
    gcm_state s = {};
    (void)gcm_step(&s, 5, 55, true, 1);
    (void)gcm_step(&s, 10, 55, true, 9);
    gcm_verdict v = gcm_step(&s, 15, 61, true, 9);
    for (uint32_t t = 20; t <= 75 && v == GCM_WAIT; t += 5) v = gcm_step(&s, t, 61, true, 9);
    expect(v == GCM_BAD_NOFRAMES, "the restarted instance presenting nothing for 60 s is bad");
  }
  expect(gcm_name(GCM_GOOD)[0] == 'g' && gcm_name(GCM_BAD_CRASH)[0] == 'c', "names");
  std::printf("ca_metal_host: %u checks, %u failures\n", checks, failures);
  return failures ? 1 : 0;
}
