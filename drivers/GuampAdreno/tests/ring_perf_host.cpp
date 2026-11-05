// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of patches/native-adreno/0019 on the patched NativeAdrenoRing.cpp (A6XX): with native_adreno_perf_ts the framed job
// carries CP_REG_TO_MEM of CP_ALWAYS_ON_COUNTER after its prologue and, behind a CP_WAIT_FOR_IDLE while the prologue has one, after
// its IBs, into fence page slot fence & 63; perfSample gives a completed job's two values once; without the switch nothing changes.
// Built by run-tests.sh with NativeAdreno's tests/XnuMemoryFixture.h standing in for IOKit/IOLib.h and IOKit/IOTypes.h.
#include "XnuMemoryFixture.h"
#include "NativeAdrenoRing.h"
#include <cstdio>
#include <cstring>
static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %d %s\n", __LINE__, #c); ++failures; } } while (0)
extern "C" volatile uint32_t native_adreno_a6xx_prologue_wfi;
static uint32_t regs[0x1000];
static uint32_t lastWptr;
static int write32(void *, uint32_t reg, uint32_t value) { if (reg < 0x1000) regs[reg] = value; if (reg == 0x0807) lastWptr = value; return 0; }
static int read32(void *, uint32_t reg, uint32_t *value) { *value = reg < 0x1000 ? regs[reg] : 0; return 0; }
static int barrier(void *) { return 0; }
static int publish(void *, const volatile uint32_t *, size_t) { return 0; }
static unsigned acquires;
static int acquire(void *, volatile uint32_t *, size_t) { ++acquires; return 0; }
alignas(16384) static uint32_t ring[4096];
alignas(4096) static uint32_t fence[1024];
static unsigned op(uint32_t header) { return (header >> 28) == 7u ? (header >> 16) & 0x7fu : 0xffu; }
static unsigned count(uint32_t header) { return header & 0x3fffu; }
int main() {
  const NativeAdrenoRingIo io = {nullptr, write32, read32, barrier, nullptr, nullptr};
  const NativeAdrenoRingCache cache = {nullptr, publish, acquire};
  for (unsigned wfi = 0; wfi < 2; ++wfi) {
    std::memset(ring, 0, sizeof ring);
    std::memset(regs, 0, sizeof regs);
    native_adreno_a6xx_prologue_wfi = wfi;
    native_adreno_perf_ts = 1;
    NativeAdrenoRing r;
    CHECK(r.setGeneration(6) == kIOReturnSuccess);
    CHECK(r.attach(&io, &cache, ring, sizeof ring, 0x100000, fence, sizeof fence, 0x108000) == kIOReturnSuccess);
    CHECK(native_adreno_perf_ts == 1);
    const uint32_t boot[1] = {0x22222222};
    uint32_t f = 0;
    CHECK(r.submit(boot, 1, &f) == kIOReturnSuccess && f == 1 && lastWptr == 1 + 7);   // kernel submissions: no samples
    uint64_t a = 0, b = 0;
    CHECK(!r.perfSample(1, &a, &b));
    fence[1] = 1;
    const uint32_t words[2] = {0x33333333, 0x44444444};
    CHECK(r.submitFramed(words, 2, &f) == kIOReturnSuccess && f == 2);
    const uint32_t pro = 6 + wfi, at = 8 + pro;   // after the prologue
    CHECK(lastWptr == at + 4 + 2 + wfi + 4 + 7);
    const uint64_t slot = 0x108000 + 0x800 + 16 * 2;
    CHECK(op(ring[at]) == 0x3e && count(ring[at]) == 3 && ring[at + 1] == (0x0980u | 2u << 18 | 1u << 30) && ring[at + 2] == uint32_t(slot) &&
          ring[at + 3] == 0);
    CHECK(ring[at + 4] == words[0] && ring[at + 5] == words[1]);
    unsigned e = at + 6;
    if (wfi) {CHECK(op(ring[e]) == 0x26 && count(ring[e]) == 0); ++e;}
    CHECK(op(ring[e]) == 0x3e && ring[e + 1] == (0x0980u | 2u << 18 | 1u << 30) && ring[e + 2] == uint32_t(slot + 8));
    CHECK((ring[e + 4] >> 8 & 0x3ffffu) == 0x885 && ring[e + 5] == 2);   // then the fence as before
    // The CP wrote the samples; the job completes; perfSample reads them once (after an acquire of the slot).
    fence[(0x800 + 32) / 4] = 0x1000; fence[(0x800 + 32) / 4 + 1] = 1; fence[(0x800 + 32) / 4 + 2] = 0x5000; fence[(0x800 + 32) / 4 + 3] = 1;
    fence[1] = 2;
    uint32_t seen = 0;
    CHECK(r.wait(2, nullptr, 0, &seen) == NativeAdrenoRingFenceComplete);
    const unsigned before = acquires;
    CHECK(r.perfSample(2, &a, &b) && a == 0x100001000ull && b == 0x100005000ull && acquires == before + 1);
    CHECK(!r.perfSample(2, &a, &b));
    // Switched off: the framed job is as before and carries no sample.
    native_adreno_perf_ts = 0;
    const uint32_t start = lastWptr;
    CHECK(r.submitFramed(words, 2, &f) == kIOReturnSuccess && f == 3 && lastWptr == start + pro + 2 + 7);
    CHECK(!r.perfSample(3, &a, &b));
    fence[1] = 3;
    CHECK(r.wait(3, nullptr, 0, &seen) == NativeAdrenoRingFenceComplete && r.detach() == kIOReturnSuccess);
  }
  // A fence page too small for the slots turns the samples off at attach.
  {
    native_adreno_perf_ts = 1;
    NativeAdrenoRing r;
    CHECK(r.setGeneration(6) == kIOReturnSuccess);
    CHECK(r.attach(&io, &cache, ring, sizeof ring, 0x100000, fence, 1024, 0x108000) == kIOReturnSuccess && native_adreno_perf_ts == 0);
  }
  std::printf("ring perf samples: %s\n", failures ? "FAIL" : "PASS");
  return failures ? 1 : 0;
}
