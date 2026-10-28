// Host test of the guamp-gpu-recover decisions and ring layout (GuampGpuRecoverCore.h).
#include <cstdio>
#include <cstring>
#include "../GuampGpuRecoverCore.h"

static unsigned checks, failures;
static void expect(bool ok, const char *what) {
  checks = checks + 1;
  if (!ok) {
    failures = failures + 1;
    std::printf("FAIL: %s\n", what);
  }
}

// PM4 type-7 header as adreno_pm4.xml defines it (odd parity of count and opcode): what AdrenoPM4Type7 builds in the kernel.
static uint32_t parity(uint32_t v) {
  uint32_t p = 1;
  for (; v; v = v >> 1) p = p ^ (v & 1);
  return p;
}
static uint32_t type7(uint32_t opcode, uint32_t count) {
  return 0x70000000u | (parity(opcode) << 23) | (opcode << 16) | (parity(count) << 15) | count;
}

// The CP's walk from start: packet by packet (type-7 headers only, which is all the head holds) until it reaches next. Returns the
// opcodes seen, in order, or an empty string when a packet crosses next or a word is not a type-7 header.
static bool walk(const uint32_t *ring, uint32_t words, uint32_t start, uint32_t next, uint32_t *opcodes, unsigned *count) {
  uint32_t at = start, travelled = 0;
  const uint32_t distance = (next - start) & (words - 1);
  *count = 0;
  while (travelled < distance) {
    const uint32_t h = ring[at];
    if ((h >> 28) != 7) return false;
    const uint32_t n = h & 0x3fff;
    opcodes[(*count)++] = (h >> 16) & 0x7f;
    travelled = travelled + 1 + n;
    at = (at + 1 + n) & (words - 1);
  }
  return travelled == distance;
}

static void layout(uint32_t start, uint32_t next, uint32_t words, const char *what) {
  grc_ring_plan p;
  uint32_t padded = next;
  bool ok = grc_plan_ring(start, next, words, &p);
  // pads as the kernel makes them: each kernel submission adds 8 words at the write index
  if (ok && p.pads) {
    padded = (next + p.pads * kGrcPadWords) & (words - 1);
    ok = grc_plan_ring(start, padded, words, &p) && !p.pads;
  }
  static uint32_t ring[16384];
  std::memset(ring, 0xee, sizeof ring);
  const grc_head_words h = {type7(0x48, 8), type7(0x3d, 3), p.nop ? type7(0x10, p.nopPayload) : 0};
  ok = ok && grc_fill_ring(ring, words, start, &p, &h, 0x302004ull, 0xA6100000u);
  uint32_t ops[8] = {};
  unsigned n = 0;
  ok = ok && walk(ring, words, start, padded, ops, &n);
  ok = ok && n >= 2 && ops[0] == 0x48 && ops[1] == 0x3d && (n == 2 || (n == 3 && ops[2] == 0x10));
  ok = ok && ring[(start + 1) & (words - 1)] == 0x2f && ring[(start + 12) & (words - 1)] == 0xA6100000u;
  expect(ok, what);
}

int main() {
  // detection
  {
    grc_state s;
    grc_init(&s, 2000);
    expect(grc_track(&s, 1000, 5, 5) == 0 && !s.busy, "idle ring: no stall");
    grc_submitted(&s, 5000, 5, 5);
    expect(grc_track(&s, 5001, 5, 6) == 1, "an idle gap before a submission does not count against it");
    expect(grc_track(&s, 6000, 5, 6) == 1000, "stall measured from the submission");
    expect(grc_cause(&s, 1000, 0) == GRC_NONE, "1 s without a cause is not a hang");
    expect(grc_cause(&s, 2000, 0) == GRC_TIMEOUT, "2 s is a hang");
    expect(grc_cause(&s, 150, GRC_CAUSE_IRQ) == GRC_IRQ, "error interrupt shortens to 100 ms");
    expect(grc_cause(&s, 150, GRC_CAUSE_IRQ | GRC_CAUSE_SMMU) == GRC_SMMU, "smmu fault reported first");
    expect(grc_cause(&s, 50, GRC_CAUSE_SMMU) == GRC_NONE, "no cause shortens below 100 ms");
    expect(grc_track(&s, 6500, 6, 8) == 0, "progress restarts the clock");
    expect(grc_track(&s, 7000, 6, 8) == 500, "next fence measured from the progress");
    grc_submitted(&s, 7100, 6, 8);
    expect(grc_track(&s, 7200, 6, 9) == 700, "a submission behind pending work does not restart the clock");
  }
  {
    grc_state s;
    grc_init(&s, 10);
    expect(s.hangMs == kGrcMinHangMs, "hang time clamped low");
    grc_init(&s, 1000000);
    expect(s.hangMs == kGrcMaxHangMs, "hang time clamped high");
    // fence wrap
    grc_init(&s, 2000);
    expect(grc_track(&s, 0, 0xfffffffeu, 1) == 0 && s.busy, "pending across the 32-bit wrap");
    expect(grc_track(&s, 3000, 0xfffffffeu, 1) == 3000, "stall across the wrap");
    expect(grc_track(&s, 3001, 1, 1) == 0 && !s.busy, "completed across the wrap");
  }
  // budget
  {
    grc_state s;
    grc_init(&s, 2000);
    uint32_t t = 1000;
    for (int i = 0; i < 3; ++i) {
      expect(grc_decide(&s, t, GRC_TIMEOUT) == GRC_RECOVER, "first three hangs recover");
      expect(grc_done(&s, true) == GRC_KEEP, "check passed");
      t = t + 10000;
    }
    expect(grc_decide(&s, t, GRC_TIMEOUT) == GRC_PARK && s.parked, "fourth hang in a minute parks");
    expect(grc_decide(&s, t + 100000, GRC_TIMEOUT) == GRC_KEEP, "parked stays parked");
    expect(s.recoveries == 3 && s.failures == 0, "totals");
  }
  {
    grc_state s;
    grc_init(&s, 2000);
    for (int i = 0; i < 3; ++i) {
      (void)grc_decide(&s, 1000 + i * 1000, GRC_IRQ);
      (void)grc_done(&s, true);
    }
    expect(grc_decide(&s, 1000 + kGrcWindowMs, GRC_IRQ) == GRC_RECOVER, "a new window after a minute");
    expect(s.inWindow == 1, "window count restarted");
    expect(grc_decide(&s, 1000, GRC_NONE) == GRC_KEEP, "no reason, no action");
  }
  {
    grc_state s;
    grc_init(&s, 2000);
    expect(grc_decide(&s, 5, GRC_SMMU) == GRC_RECOVER, "smmu hang recovers");
    expect(grc_done(&s, false) == GRC_PARK && s.parked && s.failures == 1, "a failed restart parks");
  }
  // ring layout: the CP walks from the reset read index through ME_INIT, the check write and the NOP exactly to the write index
  const uint32_t words = 8192;
  layout(0, 13, words, "distance 13: head only");
  layout(0, 14, words, "distance 14: NOP without payload");
  layout(0, 4000, words, "middle of the ring");
  layout(0, 8191, words, "write index at the ring end");
  layout(8190, 100, words, "head wraps around the ring end");
  layout(8180, 8191, words, "distance 11 padded");
  for (uint32_t d = 0; d < 14; ++d) {
    char what[64];
    std::snprintf(what, sizeof what, "short distance %u padded", d);
    layout(0, d, words, what);
  }
  {
    grc_ring_plan p;
    expect(!grc_plan_ring(0, 8192, words, &p) && !grc_plan_ring(8192, 0, words, &p), "indices outside the ring refused");
    expect(!grc_plan_ring(0, 0, 6000, &p), "ring not a power of two refused");
    expect(!grc_plan_ring(0, 0, 32768, &p), "NOP payload could not span the ring: refused");
    expect(grc_plan_ring(0, 0, words, &p) && p.pads == 2, "empty ring takes two pads");
    expect(grc_plan_ring(0, 13, words, &p) && !p.pads && !p.nop, "exact head needs no NOP");
    const grc_head_words h = {1, 2, 3};
    uint32_t ring[64] = {};
    expect(grc_plan_ring(0, 5, 64, &p) && p.pads && !grc_fill_ring(ring, 64, 0, &p, &h, 0, 0), "a plan with pads is not written");
  }
  expect(grc_name(GRC_TIMEOUT)[0] == 'f' && grc_name(GRC_SMMU)[0] == 's', "names");
  std::printf("recover_host: %u checks, %u failures\n", checks, failures);
  return failures ? 1 : 0;
}
