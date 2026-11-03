// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of patches/native-adreno/0012's write-combine re-assertion plan (NativeAdrenoReassertPlan.h, from the patched sources):
// a model of the VM's write-combined BOs whose pages "XNU" puts back at the default attribute at a cache-mode event or during the
// quiet time after it, and the submissions' passes as NativeAdrenoVm::reassertWriteCombinedChanged runs them. Checks: no submission
// ever runs over a reset page while every reset follows a counted event; a reset nothing counted is caught by the next whole check,
// which falls back to every submission; mode 0 walks every BO every time; and how much walking the plan saves. Also the ABI flags.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "NativeAdrenoABI.h"
#include "NativeAdrenoReassertPlan.h"

static_assert(NativeAdrenoParamsClientCleans == 2 && NativeAdrenoMapClientCleans == 1, "0012 flag values");
static_assert((NativeAdrenoParamsClientCleans & NativeAdrenoParamsHostImport) == 0, "distinct params bits");
static_assert(sizeof(NativeAdrenoAbiBoMap) == 32, "the flag lives in the existing reserved word");

static int failures = 0;
#define CHECK(c) do {if (!(c)) {printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++failures;}} while (0)

struct Bo {uint32_t epoch = 0; std::vector<bool> wc;};
struct Model {
  std::vector<Bo> bos;
  uint32_t epoch = 1, mode = 1, check = 64, tick = 0;
  uint64_t lastEvent = 0, quiet = 100, walks = 0, missedPages = 0;
  bool fellBack = false;
  explicit Model(unsigned n, unsigned pages) : bos(n) {for (auto &b : bos) b.wc.assign(pages, true);}
  void pass(uint64_t now) {   // one submission's re-assertion, as reassertWindow + reassertWriteCombinedChanged
    if (!mode) {for (auto &b : bos) {++walks; for (size_t i = 0; i < b.wc.size(); ++i) b.wc[i] = true;} return;}
    const NativeAdrenoReassertPlan plan = nativeAdrenoReassertPlan(mode, check, ++tick, now, lastEvent, quiet);
    uint32_t missed = 0;
    for (auto &b : bos) {
      const bool current = b.epoch == epoch;
      if (!nativeAdrenoReassertWalks(b.epoch, epoch, plan.all)) continue;
      ++walks;
      for (size_t i = 0; i < b.wc.size(); ++i) if (!b.wc[i]) {b.wc[i] = true; if (current) ++missed;}
      if (plan.settle) b.epoch = epoch;
    }
    if (missed) {missedPages += missed; mode = 0; fellBack = true;}
  }
  bool clean() const {for (auto &b : bos) for (bool w : b.wc) if (!w) return false; return true;}
};

// Runs `steps` time units: a submission every 3 units; a counted event every `every` units, whose resets land at the event or up to
// `late` units after it; `uncountedAt` (0: never) resets a page with no event. Returns the submissions that ran over a reset page.
static unsigned run(Model &m, unsigned steps, unsigned every, unsigned late, unsigned uncountedAt, unsigned seed) {
  srand(seed);
  unsigned dirtySubmits = 0;
  std::vector<std::pair<uint64_t, std::pair<size_t, size_t>>> pending;   // (time, (bo, page)) of resets still to land
  for (uint64_t t = 1; t <= steps; ++t) {
    if (every && t % every == 0) {
      ++m.epoch; m.lastEvent = t + 1000;   // NativeAdrenoNoteCacheModeEvent (times start past one quiet time)
      const size_t bo = (size_t)rand() % m.bos.size();
      for (unsigned k = 0; k < 3; ++k) pending.push_back({t + (late ? (uint64_t)rand() % late : 0), {bo, (size_t)rand() % m.bos[bo].wc.size()}});
    }
    if (uncountedAt && t == uncountedAt) m.bos[0].wc[1] = false;
    for (size_t i = 0; i < pending.size();)
      if (pending[i].first <= t) {m.bos[pending[i].second.first].wc[pending[i].second.second] = false; pending.erase(pending.begin() + (long)i);}
      else ++i;
    if (t % 3 == 0) {m.pass(t + 1000); if (!m.clean()) ++dirtySubmits;}
  }
  return dirtySubmits;
}

int main() {
  // Counted events, resets within the quiet time: never a dirty submission, no fallback, and far fewer walks than mode 0.
  {
    Model m(40, 8), all(40, 8); all.mode = 0;
    const unsigned dirty = run(m, 30000, 500, 90, 0, 1);
    (void)run(all, 30000, 500, 90, 0, 1);
    CHECK(dirty == 0); CHECK(!m.fellBack); CHECK(m.missedPages == 0);
    CHECK(m.walks * 4 < all.walks);   // events every 500 units, 100 of them unquiet
    printf("counted events: walks %llu vs %llu every submission, dirty submissions %u\n", (unsigned long long)m.walks,
           (unsigned long long)all.walks, dirty);
  }
  // Mode 0 is the 0011 behaviour: every BO at every submission.
  {
    Model m(10, 4); m.mode = 0;
    CHECK(run(m, 3000, 100, 50, 0, 2) == 0); CHECK(m.walks == 10u * 1000u);
  }
  // No event at all: after the first pass only the whole checks walk.
  {
    Model m(10, 4);
    CHECK(run(m, 3 * 640, 0, 0, 0, 3) == 0);
    CHECK(m.walks == 10u + 10u * 10u);   // the first pass (epoch 0 -> 1) and 10 whole checks of 10 BOs
  }
  // A reset nothing counted: caught by the next whole check, then every submission re-asserts everything (fallback).
  {
    Model m(10, 4);
    const unsigned dirty = run(m, 3000, 0, 0, 100, 4);
    CHECK(m.fellBack); CHECK(m.missedPages == 1); CHECK(m.mode == 0);
    CHECK(dirty > 0 && dirty <= 64);   // at most one check period ran over it
    printf("uncounted reset: %u dirty submissions before the check caught it\n", dirty);
  }
  // check 0 never walks whole; a reset later than the quiet time after its event is a miss the plan cannot see (documented limit).
  {
    Model m(4, 2); m.check = 0;
    CHECK(run(m, 300, 0, 0, 0, 5) == 0); CHECK(m.walks == 4);
  }
  printf("coherence_host: %s\n", failures ? "FAIL" : "ok");
  return failures ? 1 : 0;
}
