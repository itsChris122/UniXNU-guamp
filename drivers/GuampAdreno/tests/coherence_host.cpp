// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of patches/native-adreno/0012's write-combine re-assertion plan (NativeAdrenoReassertPlan.h, from the patched sources):
// a model of the VM's write-combined BOs whose pages "XNU" puts back at the default attribute at a cache-mode event or during the
// quiet time after it, and the submissions' passes as NativeAdrenoVm::reassertWriteCombinedChanged runs them. Checks: no submission
// ever runs over a reset page while every reset follows a counted event; a reset nothing counted is caught by the next whole check,
// which falls back to every submission; mode 0 walks every BO every time; and how much walking the plan saves. Also the ABI flags.
// Patch 0016 (dirty per BO, nativeAdrenoReassertBo, as NativeAdrenoVm::reassertWriteCombinedDirty): events that name their BO reset
// only that BO's pages, unnamed ones (imports) only pages of BOs with a CPU view; the same guarantees, walking only the touched BOs.
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

// Patch 0016: the dirty state per BO. Each BO carries its own generation and last event time (NativeAdrenoMemory) and whether it has a
// CPU view; each mapping the generation and epoch it last settled at. Named events move one BO; unnamed ones (imports) move the epoch.
struct Bo2 {uint32_t gen = 0, mgen = 0, mepoch = 0; uint64_t event = 0; bool viewed = false; std::vector<bool> wc;};
struct Model2 {
  std::vector<Bo2> bos;
  uint32_t epoch = 1, mode = 2, check = 64, tick = 0;
  uint64_t lastEvent = 0, quiet = 100, walks = 0, missedPages = 0;
  bool fellBack = false;
  explicit Model2(unsigned n, unsigned pages) : bos(n) {for (auto &b : bos) {b.wc.assign(pages, true); b.gen = 1;}}   // makeWriteCombined
  void named(size_t bo, uint64_t now) {Bo2 &b = bos[bo]; b.event = now; if (!++b.gen) b.gen = 1;}
  void unnamed(uint64_t now) {++epoch; lastEvent = now;}
  void pass(uint64_t now) {   // one submission, as reassertWindow + reassertWriteCombinedDirty
    if (!mode) {for (auto &b : bos) {++walks; for (size_t i = 0; i < b.wc.size(); ++i) b.wc[i] = true;} return;}
    const NativeAdrenoReassertPlan plan = nativeAdrenoReassertPlan(mode, check, ++tick, now, lastEvent, quiet);
    uint32_t missed = 0;
    for (auto &b : bos) {
      const uint32_t gen = b.gen;
      const NativeAdrenoReassertBo d = nativeAdrenoReassertBo(b.mgen, gen, b.mepoch, epoch, b.viewed, plan.all, now, b.event, lastEvent, quiet);
      if (!d.walk) continue;
      ++walks;
      for (size_t i = 0; i < b.wc.size(); ++i) if (!b.wc[i]) {b.wc[i] = true; if (d.current) ++missed;}
      if (d.settle_gen) b.mgen = gen;
      if (d.settle_epoch) b.mepoch = epoch;
    }
    if (missed) {missedPages += missed; mode = 0; fellBack = true;}
  }
  bool clean() const {for (auto &b : bos) for (bool w : b.wc) if (!w) return false; return true;}
};
// Like run(): every `every` units a named event on a random BO (its resets land on that BO, up to `late` after it); every `importEvery`
// units an import (unnamed), whose resets land only on BOs with a view; `viewedEvery`: every n-th BO has a view; `uncountedAt` resets a
// page with no event. Returns the submissions that ran over a reset page.
static unsigned run2(Model2 &m, unsigned steps, unsigned every, unsigned importEvery, unsigned viewedEvery, unsigned late,
                     unsigned uncountedAt, unsigned seed) {
  srand(seed);
  for (size_t i = 0; i < m.bos.size(); ++i) m.bos[i].viewed = viewedEvery && i % viewedEvery == 0;
  unsigned dirtySubmits = 0;
  std::vector<std::pair<uint64_t, std::pair<size_t, size_t>>> pending;
  for (uint64_t t = 1; t <= steps; ++t) {
    const uint64_t now = t + 1000;
    if (every && t % every == 0) {
      const size_t bo = (size_t)rand() % m.bos.size();
      m.named(bo, now);
      for (unsigned k = 0; k < 3; ++k) pending.push_back({t + (late ? (uint64_t)rand() % late : 0), {bo, (size_t)rand() % m.bos[bo].wc.size()}});
    }
    if (importEvery && t % importEvery == 0) {
      m.unnamed(now);
      for (size_t bo = 0; bo < m.bos.size(); ++bo)
        if (m.bos[bo].viewed && rand() % 4 == 0)
          pending.push_back({t + (late ? (uint64_t)rand() % late : 0), {bo, (size_t)rand() % m.bos[bo].wc.size()}});
    }
    if (uncountedAt && t == uncountedAt) m.bos[0].wc[1] = false;
    for (size_t i = 0; i < pending.size();)
      if (pending[i].first <= t) {m.bos[pending[i].second.first].wc[pending[i].second.second] = false; pending.erase(pending.begin() + (long)i);}
      else ++i;
    if (t % 3 == 0) {m.pass(now); if (!m.clean()) ++dirtySubmits;}
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
  // 0016, Maps-like load: a named event every 2 units (a view made or served almost per submission). 0012's single count walks every
  // BO at almost every submission; per BO only the touched ones are walked. Never a dirty submission, no fallback.
  {
    Model one(400, 4);
    Model2 per(400, 4);
    const unsigned d1 = run(one, 30000, 2, 90, 0, 6);
    const unsigned d2 = run2(per, 30000, 2, 0, 0, 90, 0, 6);
    CHECK(d1 == 0); CHECK(d2 == 0); CHECK(!per.fellBack); CHECK(per.missedPages == 0);
    CHECK(per.walks * 5 < one.walks);   // ~50 BOs always inside their quiet time at this rate
    printf("0016 named events every 2 units: walks %llu per BO vs %llu one count (%.1f vs %.1f BOs per submission)\n",
           (unsigned long long)per.walks, (unsigned long long)one.walks, per.walks / 10000.0, one.walks / 10000.0);
  }
  // 0016, imports (unnamed) reach only BOs with a view: still coherent, and BOs without a view are not walked for them.
  {
    Model2 m(100, 4), noViews(100, 4);
    CHECK(run2(m, 30000, 50, 300, 10, 90, 0, 7) == 0); CHECK(!m.fellBack); CHECK(m.missedPages == 0);
    CHECK(run2(noViews, 30000, 50, 300, 0, 90, 0, 7) == 0); CHECK(!noViews.fellBack);
    CHECK(noViews.walks < m.walks);
    printf("0016 imports: walks %llu with 1 BO in 10 viewed, %llu with none\n", (unsigned long long)m.walks, (unsigned long long)noViews.walks);
  }
  // 0016, an uncounted reset is still caught by the next whole check, then every submission walks everything.
  {
    Model2 m(10, 4);
    const unsigned dirty = run2(m, 3000, 0, 0, 0, 0, 100, 8);
    CHECK(m.fellBack); CHECK(m.missedPages == 1); CHECK(m.mode == 0); CHECK(dirty > 0 && dirty <= 64);
  }
  // 0016, a BO whose own events never go quiet does not keep the others walking: BO 1 is walked once (its first pass) and at checks.
  {
    Model2 m(2, 2); m.check = 0;
    for (uint64_t t = 1; t <= 300; ++t) {m.named(0, t + 1000); if (t % 3 == 0) m.pass(t + 1000);}
    CHECK(m.walks == 100u + 1u);   // BO 0 at each of the 100 submissions, BO 1 once
  }
  // 0016, the generation skips 0 when it wraps (0 is a mapping never settled), and an event during the quiet time keeps the BO owed.
  {
    Model2 m(1, 2); m.check = 0; m.bos[0].gen = UINT32_MAX; m.bos[0].mgen = UINT32_MAX; m.bos[0].mepoch = m.epoch;
    m.named(0, 5000); CHECK(m.bos[0].gen == 1);
    m.pass(5010); CHECK(m.walks == 1); CHECK(m.bos[0].mgen == UINT32_MAX);   // too young: walked, not settled
    m.pass(5200); CHECK(m.walks == 2); CHECK(m.bos[0].mgen == 1);            // quiet: walked once more and settled
    m.pass(5300); CHECK(m.walks == 2);
  }
  printf("coherence_host: %s\n", failures ? "FAIL" : "ok");
  return failures ? 1 : 0;
}
