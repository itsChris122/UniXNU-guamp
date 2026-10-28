// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of the latest-frame-wins core of scripts/guamp-canvas-variant.py (section 10): run-latest-tests.sh extracts the text
// between "guamp-latest-core-begin" and "guamp-latest-core-end" into LATEST_CORE. Fixed scenarios, then a random walk of swaps,
// copy-thread and GPU completions, failures and trailing checks against these invariants:
//  - one frame in flight at most (copy thread or GPU), and only when nothing newer could have started instead;
//  - the screen never goes back to an older frame, and once the machine is idle it shows the newest swapped frame;
//  - every reference handed to the machine is released exactly once (none after clear()).
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include LATEST_CORE

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

struct Frame {
  int id = 0;   // 0 none, > 0 a surface, < 0 a blank frame (no reference)
};
struct Env;
using Core = GuampLatest<Frame, Env>;
struct Env {
  std::map<int, int> refs;
  bool gpuTakes = false;   // the presenter is waiting for a frame
  int cpuStarts = 0, gpuStarts = 0;
  void latestRetain(const Frame &f) { if (f.id > 0) ++refs[f.id]; }
  void latestRelease(const Frame &f) {
    if (f.id <= 0) return;
    if (refs[f.id] <= 0) { std::printf("FAIL release of frame %d without a reference\n", f.id); ++failures; return; }
    --refs[f.id];
  }
  bool latestOffer(const Frame &f) { if (f.id > 0 && gpuTakes) { ++gpuStarts; return true; } return false; }
  void latestStartCpu() { ++cpuStarts; }
  int held() const { int n = 0; for (auto &r : refs) n += r.second; return n; }
};
// What the machine holds must be exactly the references outstanding.
static int slots(const Core &c) {
  int n = 0;
  if (c.queued && c.next.id > 0) ++n;
  if (c.cpu && c.cpuFrame.id > 0) ++n;
  if (c.gpu && c.gpuFrame.id > 0) ++n;
  if (c.hasShown && c.shown.id > 0) ++n;
  return n;
}
static Frame swap(Env &env, Core &c, int id) {
  Frame f; f.id = id;
  env.latestRetain(f);   // the display retains the surface and hands the reference over
  c.swapped(env, f);
  return f;
}

static void scenarios() {
  {  // A burst while the copy thread is busy: the frames between are coalesced, the newest is copied next.
    Env env; Core c;
    swap(env, c, 1);
    CHECK(c.cpu && c.cpuFrame.id == 1 && env.cpuStarts == 1);
    for (int id = 2; id <= 5; ++id) swap(env, c, id);
    CHECK(c.coalesced == 3 && c.queued && c.next.id == 5);
    CHECK(c.superseded());
    c.cpuDone(env, 0);   // frame 1 copied, then frame 5 starts
    CHECK(c.shown.id == 1 && c.cpu && c.cpuFrame.id == 5 && !c.queued);
    c.cpuDone(env, 0);
    CHECK(c.idle() && c.shown.id == 5 && c.cpuFrames == 2 && env.held() == 1 && slots(c) == 1);
    c.clear(env);
    CHECK(env.held() == 0);
  }
  {  // A newer frame arrives during the GPU wait: the thread drops the old one (stale) and the newer one is copied.
    Env env; Core c;
    swap(env, c, 1);
    swap(env, c, 2);
    CHECK(c.superseded());
    c.cpuDone(env, 1);
    CHECK(c.stale == 1 && !c.hasShown && c.cpu && c.cpuFrame.id == 2);
    c.cpuDone(env, 0);
    CHECK(c.shown.id == 2 && env.held() == 1);
    c.clear(env);
    CHECK(env.held() == 0);
  }
  {  // GPU presenter: a lost frame goes to the copy thread; the next one is offered to the GPU again.
    Env env; Core c;
    env.gpuTakes = true;
    swap(env, c, 1);
    CHECK(c.gpu && c.gpuFrame.id == 1 && env.gpuStarts == 1);
    c.gpuDone(env, -1);
    CHECK(c.gpuFailures == 1 && c.cpu && c.cpuFrame.id == 1);
    c.cpuDone(env, 0);
    swap(env, c, 2);
    CHECK(c.gpu && c.gpuFrame.id == 2);
    swap(env, c, 3);   // queued behind the GPU frame
    c.gpuDone(env, 0);
    CHECK(c.shown.id == 2 && c.gpu && c.gpuFrame.id == 3);
    c.gpuDone(env, 0);
    CHECK(c.idle() && c.shown.id == 3 && c.gpuFrames == 2 && env.held() == 1);
    c.clear(env);
    CHECK(env.held() == 0);
  }
  {  // GPU loss with a newer frame queued: the lost one is dropped, the newer one goes to the copy thread once.
    Env env; Core c;
    env.gpuTakes = true;
    swap(env, c, 1);
    swap(env, c, 2);
    c.gpuDone(env, -1);
    CHECK(c.cpu && c.cpuFrame.id == 2 && env.refs[1] == 0);
    c.cpuDone(env, 0);
    CHECK(c.shown.id == 2 && env.held() == 1);
    c.clear(env);
    CHECK(env.held() == 0);
  }
  {  // Trailing check: the frame on screen is shown again with its own reference; nothing while busy.
    Env env; Core c;
    CHECK(!c.trail(env));
    swap(env, c, 7);
    CHECK(!c.trail(env));
    c.cpuDone(env, 0);
    CHECK(c.trail(env) && c.trails == 1 && c.cpu && c.cpuFrame.id == 7 && env.refs[7] == 2);
    c.cpuDone(env, 0);
    CHECK(c.shown.id == 7 && env.refs[7] == 1);
    c.clear(env);
    CHECK(env.held() == 0);
  }
  {  // Blank frames take no reference and keep their place in the order.
    Env env; Core c;
    swap(env, c, 1);
    Frame blank; blank.id = -2; c.swapped(env, blank);
    c.cpuDone(env, 0);
    CHECK(c.cpu && c.cpuFrame.id == -2);
    c.cpuDone(env, 0);
    CHECK(c.shown.id == -2 && env.held() == 0);
    c.clear(env);
  }
  {  // A failed copy releases the frame and does not count as shown.
    Env env; Core c;
    swap(env, c, 1);
    c.cpuDone(env, 2);
    CHECK(c.cpuFailures == 1 && !c.hasShown && env.held() == 0 && c.idle());
  }
}

static unsigned rng = 12345;
static unsigned next(unsigned n) { rng = rng * 1103515245u + 12345u; return (rng >> 8) % n; }

static void walk(unsigned seed) {
  rng = seed;
  Env env; Core c;
  int id = 0, screen = 0, newest = 0;
  for (int step = 0; step < 4000; ++step) {
    env.gpuTakes = next(3) != 0 && (seed & 1);
    switch (next(6)) {
    case 0: case 1: {
      ++id;
      if (next(10) == 0) { Frame b; b.id = -id; c.swapped(env, b); } else swap(env, c, id);
      newest = id;
      break;
    }
    case 2:
      if (c.cpu) {
        const int frame = c.cpuFrame.id < 0 ? -c.cpuFrame.id : c.cpuFrame.id;
        const int result = c.superseded() && next(2) ? 1 : (next(40) == 0 ? 2 : 0);
        if (!result) { CHECK(frame >= screen); screen = frame; }
        c.cpuDone(env, result);
      }
      break;
    case 3:
      if (c.gpu) {
        const int frame = c.gpuFrame.id;
        const int status = next(25) == 0 ? -1 : 0;
        if (!status) { CHECK(frame >= screen); screen = frame; }
        c.gpuDone(env, status);
      }
      break;
    case 4:
      if (next(4) == 0) c.trail(env);
      break;
    default:
      break;
    }
    CHECK(!(c.cpu && c.gpu));
    CHECK(slots(c) == env.held());
    if (c.queued) CHECK(c.cpu || c.gpu);   // a queued frame waits only behind one in flight
  }
  // Drain without failures: the newest swapped frame ends on screen.
  for (int guard = 0; guard < 100 && !c.idle(); ++guard) {
    if (c.cpu) {
      const int frame = c.cpuFrame.id < 0 ? -c.cpuFrame.id : c.cpuFrame.id;
      const int result = c.superseded() ? 1 : 0;
      if (!result) screen = frame;
      c.cpuDone(env, result);
    } else if (c.gpu) {
      screen = c.gpuFrame.id;
      c.gpuDone(env, 0);
    }
  }
  CHECK(c.idle());
  // Unless the newest frame's own copy failed on the way, it is the one on screen.
  if (newest && c.hasShown) {
    const int shown = c.shown.id < 0 ? -c.shown.id : c.shown.id;
    CHECK(shown == screen);
    if (shown != newest) {
      // Only a failed copy of the newest frame (no newer one to replace it) may leave an older frame shown.
      CHECK(c.cpuFailures > 0);
    }
  }
  c.clear(env);
  CHECK(env.held() == 0 && slots(c) == 0);
}

int main() {
  scenarios();
  for (unsigned seed = 1; seed <= 400; ++seed) walk(seed);
  std::printf("latest-frame-wins core: %s\n", failures ? "FAIL" : "PASS");
  return failures ? 1 : 0;
}
