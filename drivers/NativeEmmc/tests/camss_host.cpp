// Host test of GuampCamssCore.h: clock plan values, CSIPHY lane tables / lane enable / settle, CSID and TFE
// bring-up sequences against a simulated register file (reset acknowledgements, write-1-to-clear statuses), the
// TPG start word, the interrupt service + buffer ring against a simulated write master that latches its address at
// SOF after a reg update (consumer-held buffers, stop after N frames, error accounting), the consumer claim against
// the next-address race (redirect, stale, held overwrite), the stall watchdog / path restart, and the RAW10 statistics.
#include "../GuampCamssCore.h"
#include <stdio.h>
#include <string.h>
#include <map>
#include <vector>

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

using namespace camss;

struct Sim {
  std::map<uint32_t, uint32_t> r[kBlkCount];
  struct W { unsigned b; uint32_t off, v; };
  std::vector<W> trace;
  uint64_t now = 1000;
  // write master model
  uint32_t wmAddr = 0, latched = 0;
  bool rupPending = false;
  unsigned rdi = 0;
};
static Sim *S;
static uint32_t &reg(unsigned b, uint32_t o) { return S->r[b][o]; }

static uint32_t sRd(void *, unsigned b, uint32_t o) {
  auto it = S->r[b].find(o);
  return it == S->r[b].end() ? 0 : it->second;
}
static void sWr(void *, unsigned b, uint32_t o, uint32_t v) {
  S->trace.push_back({b, o, v});
  if (b == kBlkGcc) {
    if (o == kGdscCamssTop) { reg(b, o) = (v & 1u) ? (v & ~(1u << 31)) : (v | 1u << 31); return; }
    const bool isRcg = o == kRcgTopAhb || o == kRcgAxi || o == kRcgCphyRx || (o >= 0x52004 && o <= 0x520d4 && (o == rcgTfe(0) ||
                       o == rcgTfe(1) || o == rcgTfe(2) || o == rcgTfeCsid(0) || o == rcgTfeCsid(1) || o == rcgTfeCsid(2))) ||
                       o == rcgPhyTimer(0) || o == rcgPhyTimer(1) || o == rcgPhyTimer(2);
    if (isRcg) { reg(b, o) = v & ~1u; return; }   // update completes at once
    reg(b, o) = (v & 1u) ? (v & ~kCbcrOff) : (v | kCbcrOff);   // branch
    return;
  }
  if (b == kBlkCsid) {
    if (o == kCsidRstStrobes) { reg(b, kCsidTopIrqStatus) |= 1u; return; }
    if (o == kCsidTopIrqClear) { reg(b, kCsidTopIrqStatus) &= ~v; return; }
    if (o == kCsiRxIrqClear) { reg(b, kCsiRxIrqStatus) &= ~v; return; }
    for (unsigned n = 0; n < 3; ++n) if (o == rdiIrqClear(n)) { reg(b, rdiIrqStatus(n)) &= ~v; return; }
    if (o == kTfeGlobalReset) { reg(b, tfeTopStatus(0)) |= kTop0Reset; return; }
    for (unsigned n = 0; n < 3; ++n) if (o == tfeTopClear(n)) { reg(b, tfeTopStatus(n)) &= ~v; return; }
    for (unsigned n = 0; n < 2; ++n) if (o == busClear(n)) { reg(b, busStatus(n)) &= ~v; return; }
    if (o == wmBase(S->rdi) + kWmImageAddr) S->wmAddr = v;
    if (o == kTfeRegUpdate && (v & tfeRegUpdateRdi(S->rdi))) S->rupPending = true;
  }
  reg(b, o) = v;
}
static void sDelay(void *, unsigned) {}
static uint64_t sNow(void *) { return S->now; }
static const Io kIo = {nullptr, sRd, sWr, sDelay, sNow};

static uint32_t lastWrite(unsigned b, uint32_t off) {
  for (size_t i = S->trace.size(); i-- > 0;) if (S->trace[i].b == b && S->trace[i].off == off) return S->trace[i].v;
  return 0xdeadbeef;
}
static unsigned countWrites(unsigned b, uint32_t lo, uint32_t hi) {
  unsigned n = 0;
  for (auto &w : S->trace) if (w.b == b && w.off >= lo && w.off < hi) ++n;
  return n;
}

static void testPure() {
  CHECK(laneMaskFrom(4, 0x3210) == 0x1f);
  CHECK(laneEnable(0x1f) == 0xd5);
  CHECK(laneMaskFrom(1, 0x2) == 0xa);     // OV02B1B on physical lane 2
  CHECK(laneEnable(0xa) == 0x90);
  CHECK(laneMaskFrom(1, 0x0) == 0x3);
  CHECK(settleFromRate(1152) == 12);      // S5KGM1ST mode 1
  CHECK(settleFromRate(732) == 12);       // S5K4H7
  CHECK(settleFromRate(0) == 14);
  Rcg p[kMaxRcgs];
  unsigned n = clockPlan(0, 0, 0, true, p);
  CHECK(n == 6);
  CHECK(p[0].rcg == 0x58010 && p[0].cfg == 0x10e);
  CHECK(p[1].rcg == 0x5802c && p[1].cfg == 0x201);
  CHECK(p[2].rcg == 0x52064 && p[2].cfg == 0x104);
  CHECK(p[3].rcg == 0x52094 && p[3].cfg == 0x104);
  CHECK(p[4].rcg == 0x52004 && p[4].cfg == 0x103);
  CHECK(p[5].rcg == 0x59000 && p[5].cfg == 0x202);
  n = clockPlan(1, 1, 2, false, p);
  CHECK(n == 5 && p[2].cfg == 0x403 && p[3].rcg == 0x520b4 && p[3].cfg == 0x401 && p[4].rcg == 0x52024);
  CHECK(cbcrTfeCsid(2) == 0x520ec && cbcrTfe(1) == 0x5203c && cbcrTfeCphyRx(2) == 0x52084 && cbcrPhyTimer(2) == 0x59050);
  CHECK(wmBase(0) == 0x2300 && wmBase(2) == 0x2500);
  CHECK(busDone(0) == 1u << 13 && busRup(0) == 2u && tfeSof(1) == 0x100 && tfeEof(2) == 0x2000);
  CHECK(kIntidCsid[0] == 242 && kIntidTfe[0] == 243 && kIntidTfe[2] == 110);
  CHECK(raw10LineBytes(1920) == 2400 && raw10LineBytes(1600) == 2000);
}

static void testBringUp() {
  Sim sim; S = &sim;
  for (const Branch &b : kCommonBranches) reg(kBlkGcc, b.cbcr) = kCbcrOff;
  reg(kBlkGcc, kGdscCamssTop) = 1;
  CHECK(gdscOn(kIo));
  Rcg p[kMaxRcgs];
  const unsigned n = clockPlan(0, 0, 0, true, p);
  for (unsigned i = 0; i < n; ++i) CHECK(rcgApply(kIo, p[i]));
  for (const Branch &b : kCommonBranches) CHECK(branchOn(kIo, b.cbcr, b.haltDelay));
  CHECK(sRd(nullptr, kBlkGcc, 0x58044) == 1u);
  camnocInit(kIo);
  CHECK(lastWrite(kBlkCamnoc, 0x30) == 0x44443333 && lastWrite(kBlkCamnoc, 0x48) == 3);
  // CSIPHY, 4 lanes normal
  sim.trace.clear();
  PhyCfg pc = {4, 0x1f, 14, 14, false};
  csiphyConfig(kIo, pc);
  CHECK(lastWrite(kBlkPhy, 0x814) == 0xd5);
  CHECK(lastWrite(kBlkPhy, 0x008) == 14 && lastWrite(kBlkPhy, 0x708) == 14 && lastWrite(kBlkPhy, 0x608) == 14);
  CHECK(lastWrite(kBlkPhy, 0x000) == 0xd7 && lastWrite(kBlkPhy, 0x700) == 0xc0 && lastWrite(kBlkPhy, 0x728) == 0x04);
  CHECK(lastWrite(kBlkPhy, 0x028) == 0xdeadbeef && lastWrite(kBlkPhy, 0x00c) == 0xdeadbeef && lastWrite(kBlkPhy, 0x70c) == 0xff);
  CHECK(countWrites(kBlkPhy, 0x000, 0x100) == 13 && countWrites(kBlkPhy, 0x700, 0x800) == 15);
  CHECK(countWrites(kBlkPhy, 0x82c, 0x858) == 11 && lastWrite(kBlkPhy, 0x834) == 0xfb);
  // combo: DL3 becomes the second clock lane
  sim.trace.clear();
  PhyCfg cc = {1, 0x1f, 14, 20, true};
  csiphyConfig(kIo, cc);
  CHECK(lastWrite(kBlkPhy, 0x628) == 0x0e && lastWrite(kBlkPhy, 0x600) == 0xc0 && lastWrite(kBlkPhy, 0x428) == 0x0a);
  CHECK(lastWrite(kBlkPhy, 0x608) == 20 && lastWrite(kBlkPhy, 0x408) == 20 && lastWrite(kBlkPhy, 0x208) == 14);
  // CSID
  sim.trace.clear();
  CHECK(csidReset(kIo) == kOk);
  CsidCfg c = {4, 0x3210, 0, 0, 0x2b, 0};
  CHECK(csidConfig(kIo, c) == kOk);
  CHECK(lastWrite(kBlkCsid, kCsiRxCfg0) == 0x32103);
  CHECK(lastWrite(kBlkCsid, kCsiRxCfg1) == 0x41);
  CHECK(sRd(nullptr, kBlkCsid, rdiCfg0(0)) == 0x802bf005u);
  CHECK(lastWrite(kBlkCsid, rdiErrRecovery0(0)) == 9 && lastWrite(kBlkCsid, rdiCfg1(0)) == 2);
  CsidCfg c2 = {1, 0x2, 1, 1, 0x2b, 2};
  CHECK(csidConfig(kIo, c2) == kOk);
  CHECK(lastWrite(kBlkCsid, kCsiRxCfg0) == (0u | 0x2u << 4 | 1u << 20));
  CHECK(sRd(nullptr, kBlkCsid, rdiCfg0(2)) == (0x80000000u | 1u << 22 | 0x2bu << 16 | 0xf000u | 5u));
  csidStart(kIo, 0);
  CHECK(lastWrite(kBlkCsid, rdiCtrl(0)) == kResumeAtFrame && lastWrite(kBlkCsid, rdiIrqMask(0)) == 0x71206u);
  // a reset that never completes
  {
    Sim dead; S = &dead;
    Io noAck = kIo;
    noAck.wr = [](void *, unsigned b, uint32_t o, uint32_t v) { if (o != kCsidRstStrobes) S->r[b][o] = v; };
    CHECK(csidReset(noAck) == kErrTimeout);
    S = &sim;
  }
  // TFE
  sim.trace.clear();
  CHECK(tfeReset(kIo) == kOk);
  CHECK(lastWrite(kBlkCsid, kTfeGlobalReset) == 1);
  tfeConfig(kIo, 0, 0x20000000, 0x250000);
  const uint32_t w = wmBase(0);
  CHECK(lastWrite(kBlkCsid, w + kWmCfg) == 0x10001 && lastWrite(kBlkCsid, w + kWmImageCfg0) == 0xffff);
  CHECK(lastWrite(kBlkCsid, w + kWmPacker) == 0xa && lastWrite(kBlkCsid, w + kWmImageCfg2) == 0xffff);
  CHECK(lastWrite(kBlkCsid, w + kWmDropPattern) == 1 && lastWrite(kBlkCsid, w + kWmSubPattern) == 1);
  CHECK(lastWrite(kBlkCsid, w + kWmImageAddr) == 0x20000000 && lastWrite(kBlkCsid, w + kWmFrameIncr) == 0x250000);
  CHECK(sRd(nullptr, kBlkCsid, busMask(0)) == 0xd0002002u);
  tfeStart(kIo, 0, 2);
  CHECK(lastWrite(kBlkCsid, tfeRdiModule(0)) == (2u << 24 | 1u) && lastWrite(kBlkCsid, kTfeRegUpdate) == 2);
  CHECK(sRd(nullptr, kBlkCsid, tfeTopMask(0)) == (kTop0Bus | kTop0ErrMask) && sRd(nullptr, kBlkCsid, tfeTopMask(1)) == 0x30 &&
        sRd(nullptr, kBlkCsid, tfeTopMask(2)) == 0x3f);
  // TPG
  TpgCfg t = {0, 4, 1600, 1200, 0, 0x2b, 9, 2};
  tpgStart(kIo, t);
  CHECK(lastWrite(kBlkTpg, 0x60) == 0x37 && lastWrite(kBlkTpg, 0x70) == (1600u << 16 | 1200u));
  CHECK(lastWrite(kBlkTpg, 0x90) == 0x20008 && lastWrite(kBlkTpg, 0xa0) == 0x20 && (sRd(nullptr, kBlkCpas, kCpasTpgMux) & 1));
  tpgStop(kIo, 0);
  CHECK(lastWrite(kBlkTpg, 0x60) == 0 && !(sRd(nullptr, kBlkCpas, kCpasTpgMux) & 1));
}

// One frame of the simulated hardware: SOF (+RUP if an update is pending), service; then EOF + done, service.
static int simFrame(Stream &s, bool errorRx = false) {
  S->now += 1000;
  reg(kBlkCsid, tfeTopStatus(1)) |= tfeSof(s.rdi);
  if (S->rupPending) {
    S->latched = S->wmAddr;
    S->rupPending = false;
    reg(kBlkCsid, tfeTopStatus(0)) |= kTop0Bus;
    reg(kBlkCsid, busStatus(0)) |= busRup(s.rdi);
  }
  reg(kBlkCsid, rdiIrqStatus(s.rdi)) |= kRdiSof;
  if (errorRx) reg(kBlkCsid, kCsiRxIrqStatus) |= 1u << 19;
  CHECK(serviceTfe(kIo, s));
  CHECK(serviceCsid(kIo, s));
  const uint32_t written = S->latched;
  S->now += 30000;
  reg(kBlkCsid, tfeTopStatus(1)) |= tfeEof(s.rdi);
  reg(kBlkCsid, tfeTopStatus(0)) |= kTop0Bus;
  reg(kBlkCsid, busStatus(0)) |= busDone(s.rdi);
  reg(kBlkCsid, rdiIrqStatus(s.rdi)) |= kRdiEof;
  CHECK(serviceTfe(kIo, s));
  CHECK(serviceCsid(kIo, s));
  CHECK(!serviceTfe(kIo, s) && !serviceCsid(kIo, s));   // all cleared
  CHECK(s.lastBuf >= 0 && s.iova[s.lastBuf] == written);  // metadata names the buffer the hardware wrote
  return s.lastBuf;
}

static void testRing() {
  Sim sim; S = &sim;
  Stream s;
  s.rdi = 0; s.buffers = 3;
  s.iova[0] = 0x20000000; s.iova[1] = 0x20300000; s.iova[2] = 0x20600000;
  ringReset(s);
  s.target = 7; s.live = true;
  tfeConfig(kIo, 0, s.iova[0], 0x300000);
  tfeStart(kIo, 0, 0);   // reg update for the first address
  CHECK(sim.rupPending && sim.wmAddr == s.iova[0]);
  int seen[8];
  for (unsigned k = 0; k < 3; ++k) seen[k] = simFrame(s);
  CHECK(seen[0] == 0 && seen[1] == 1 && seen[2] == 2);
  CHECK(s.c.done == 3 && s.c.sof == 3 && s.c.rup == 3 && s.c.eof == 3 && s.c.csid_sof == 3 && s.c.csid_eof == 3);
  CHECK(s.meta[2].seq == 3 && s.meta[2].doneNs > s.meta[2].sofNs && s.meta[2].eofNs >= s.meta[2].sofNs);
  // the consumer holds buffer 1: it is skipped until released
  s.held = 1u << 1;
  for (unsigned k = 3; k < 6; ++k) seen[k] = simFrame(s, k == 4);
  CHECK(seen[3] == 0 && seen[4] == 2 && seen[5] == 0);
  CHECK(s.c.csid_rx_errors == 1 && s.c.first_rx_error == 1u << 19 && (s.meta[2].errors & kFeRx));
  CHECK(!(s.meta[0].errors & kFeRx));
  s.held = 0;
  seen[6] = simFrame(s);
  CHECK(s.c.done == 7 && s.stopRequest);
  // after the stop request no new address is programmed
  const size_t before = sim.trace.size();
  simFrame(s);
  bool reprogrammed = false;
  for (size_t i = before; i < sim.trace.size(); ++i) if (sim.trace[i].off == wmBase(0) + kWmImageAddr) reprogrammed = true;
  CHECK(!reprogrammed);
  // all buffers held: overwrite the current one, counted as a drop
  ringReset(s);
  s.live = true; s.target = 0;
  sim.rupPending = true; sim.wmAddr = s.iova[0];
  s.held = (1u << 1) | (1u << 2);
  simFrame(s);
  CHECK(s.c.drops == 1);
  // bus and TFE errors
  reg(kBlkCsid, tfeTopStatus(0)) |= kTop0Bus | (1u << 17);
  reg(kBlkCsid, busStatus(0)) |= 1u << 31;
  reg(kBlkCsid, kBusImageSize) = 0x80;
  CHECK(serviceTfe(kIo, s));
  CHECK(s.c.tfe_errors == 1 && (s.c.first_tfe_error[0] & (1u << 17)) && s.c.bus_errors == 1 && s.c.bus_image_size == 0x80);
  CHECK(pickNext(s, -1) == 0);
  // PDAF on VC 1 / DT 0x30: unmapped, counted apart, not an error
  const uint32_t rxBefore = s.c.csid_rx_errors;
  reg(kBlkCsid, kCsiRxIrqStatus) |= kRxUnmapped;
  reg(kBlkCsid, kCsiRxUnmappedHdr0) = 0x00300041;
  CHECK(serviceCsid(kIo, s));
  CHECK(s.c.csid_rx_unmapped == 1 && s.c.last_unmapped_hdr == 0x00300041 && s.c.csid_rx_errors == rxBefore);
}

// Half frames of the simulated hardware (TFE only): SOF with the RUP latch, then EOF + buffer done.
static std::vector<uint32_t> gLatches;   // every address the write master took, in order
static void simSof(Stream &s) {
  S->now += 1000;
  reg(kBlkCsid, tfeTopStatus(1)) |= tfeSof(s.rdi);
  if (S->rupPending) {
    S->latched = S->wmAddr;
    S->rupPending = false;
    gLatches.push_back(S->latched);
    reg(kBlkCsid, tfeTopStatus(0)) |= kTop0Bus;
    reg(kBlkCsid, busStatus(0)) |= busRup(s.rdi);
  }
  CHECK(serviceTfe(kIo, s));
}
static uint32_t simEof(Stream &s) {
  S->now += 30000;
  reg(kBlkCsid, tfeTopStatus(1)) |= tfeEof(s.rdi);
  reg(kBlkCsid, tfeTopStatus(0)) |= kTop0Bus;
  reg(kBlkCsid, busStatus(0)) |= busDone(s.rdi);
  CHECK(serviceTfe(kIo, s));
  CHECK(s.lastBuf >= 0 && s.iova[s.lastBuf] == S->latched);
  return S->latched;
}
static void streamSetup(Sim &sim, Stream &s, unsigned buffers) {
  S = &sim;
  gLatches.clear();
  s = Stream{};
  s.rdi = 0; s.buffers = buffers;
  for (unsigned i = 0; i < buffers; ++i) s.iova[i] = 0x20000000u + i * 0x300000u;
  ringReset(s);
  s.target = 0; s.live = true; s.lastProgressNs = sim.now;
  tfeConfig(kIo, 0, s.iova[0], 0x300000);
  tfeStart(kIo, 0, 0);
}
static bool latchedSince(size_t from, uint32_t iova) {
  for (size_t i = from; i < gLatches.size(); ++i) if (gLatches[i] == iova) return true;
  return false;
}

static void testClaim() {
  // 3 buffers, the consumer keeps buffer 0 and comes late for the next frame: the address after it already names it
  {
    Sim sim; Stream s;
    streamSetup(sim, s, 3);
    simFrame(s);                                   // frame 1 -> buffer 0
    CHECK(claimLatest(kIo, s) == 0 && s.held == 1u);
    simFrame(s);                                   // frame 2 -> buffer 1
    simSof(s);                                     // frame 3 starts in buffer 2; next address = 1 (0 is held)
    CHECK(s.latched == 2 && s.pending == 1 && s.writing == 2);
    const size_t mark = gLatches.size();
    CHECK(claimLatest(kIo, s) == 1);               // frame 2: its buffer is pending -> moved
    CHECK(s.c.claim_redirects == 1 && s.pending == 2 && s.held == 3u && s.c.drops == 1);
    simEof(s);
    for (unsigned k = 0; k < 4; ++k) { simSof(s); simEof(s); }
    CHECK(!latchedSince(mark, s.iova[1]) && !latchedSince(mark, s.iova[0]));   // held buffers never taken again
    CHECK(s.c.held_overwrites == 0);
    const Counters c = s.c;
    CHECK(c.claim_stale == 0);
    s.held = 0;                                    // released: the ring uses all three again
    const size_t mark2 = gLatches.size();
    for (unsigned k = 0; k < 4; ++k) { simSof(s); simEof(s); }
    CHECK(latchedSince(mark2, s.iova[0]) && latchedSince(mark2, s.iova[1]));
  }
  // 2 buffers, buffer 0 held: everything goes to buffer 1, whose frame is never handed out while it is rewritten
  {
    Sim sim; Stream s;
    streamSetup(sim, s, 2);
    simFrame(s);
    CHECK(claimLatest(kIo, s) == 0);
    simFrame(s);                                   // frame 2 -> buffer 1, next address = 1 again (overwrite)
    simSof(s);                                     // frame 3 is being written into buffer 1
    CHECK(s.writing == 1 && s.lastBuf == 1);
    CHECK(claimLatest(kIo, s) == kClaimStale && s.c.claim_stale == 1 && s.held == 1u);
    simEof(s);                                     // frame 3 done, next address is buffer 1 again
    CHECK(claimLatest(kIo, s) == kClaimStale && s.c.claim_stale == 2);
    s.held = 0;                                    // release 0: the pending address moves away from buffer 1
    CHECK(claimLatest(kIo, s) == 1 && s.pending == 0 && s.c.claim_redirects == 1);
    const size_t mark = gLatches.size();
    simSof(s); simEof(s);
    CHECK(!latchedSince(mark, s.iova[1]) && s.lastBuf == 0);
  }
  // a claim that lost the race (held buffer latched anyway) is counted
  {
    Sim sim; Stream s;
    streamSetup(sim, s, 3);
    simFrame(s); simFrame(s);
    simSof(s);
    s.held = s.held | (1u << s.pending);          // as if the hold came after the SOF took the address
    simEof(s);
    simSof(s);
    CHECK(s.c.held_overwrites == 1);
  }
  // pending statuses are serviced before deciding: a RUP not yet seen by the handler counts
  {
    Sim sim; Stream s;
    streamSetup(sim, s, 2);
    simFrame(s);                                   // frame 1 in buffer 0, next address 1
    CHECK(claimLatest(kIo, s) == 0);
    s.held = 0;
    simFrame(s);                                   // frame 2 in buffer 1, next address 0
    // SOF 3 latched buffer 0 in hardware, interrupt not serviced yet
    reg(kBlkCsid, tfeTopStatus(1)) |= tfeSof(0);
    S->latched = S->wmAddr; S->rupPending = false; gLatches.push_back(S->latched);
    reg(kBlkCsid, tfeTopStatus(0)) |= kTop0Bus;
    reg(kBlkCsid, busStatus(0)) |= busRup(0);
    CHECK(claimLatest(kIo, s) == 1 && s.c.rup == 3 && s.writing == 0 && s.pending == 0);
    CHECK(s.c.claim_redirects == 1);   // buffer 1 had become the next address after that RUP: moved back to 0
  }
  // stopped stream: any finished buffer can be held
  {
    Sim sim; Stream s;
    streamSetup(sim, s, 2);
    simFrame(s); simFrame(s);
    simSof(s);
    s.live = false;
    CHECK(claimLatest(kIo, s) == 1);
    Stream e; e.buffers = 2;
    CHECK(claimLatest(kIo, e) == kClaimNone);
  }
}

static void testRecovery() {
  Sim sim; Stream s;
  streamSetup(sim, s, 3);
  const uint64_t stall = 2000000000ull;
  simFrame(s); simFrame(s);
  CHECK(!stalled(s, sim.now + stall - 1, stall) && stalled(s, sim.now + stall, stall));
  CHECK(!stalled(s, sim.now + stall, 0));          // watchdog off
  s.target = 5;
  CHECK(!stalled(s, sim.now + stall, stall));      // counted capture: the caller times out itself
  s.target = 0; s.stopping = true;
  CHECK(!stalled(s, sim.now + stall, stall));
  s.stopping = false;
  // restart: counters, held buffers and the newest frame stay; first address avoids them
  CHECK(claimLatest(kIo, s) == 1);
  const uint32_t done = s.c.done;
  const int first = restartBuffer(s);
  CHECK(first == 2);
  ringRestart(s, first, sim.now + stall);
  CHECK(s.c.done == done && s.held == 2u && s.lastBuf == 1 && s.pending == 2 && s.latched == -1 && s.writing == -1);
  CHECK(!stalled(s, sim.now + stall, stall));
  sim.rupPending = false;
  tfeConfig(kIo, 0, s.iova[first], 0x300000);
  tfeStart(kIo, 0, 0);
  sim.now += stall;
  CHECK(simFrame(s) == 2 && s.c.done == done + 1 && s.meta[2].seq == done + 1);
  CHECK(s.lastProgressNs == sim.now);
}

static void testStats() {
  const uint32_t w = 64, h = 4, lb = raw10LineBytes(w);
  std::vector<uint8_t> f(lb * h);
  for (uint32_t y = 0; y < h; ++y)
    for (uint32_t g = 0; g < w / 4; ++g) {
      uint8_t *q = &f[y * lb + g * 5];
      uint8_t lsb = 0;
      for (unsigned k = 0; k < 4; ++k) {
        const uint32_t x = g * 4 + k, v = (x / 8) * 128 + 3;   // 8 bands, values 3, 131, ..., 899
        q[k] = static_cast<uint8_t>(v >> 2);
        lsb |= static_cast<uint8_t>((v & 3u) << (2 * k));
      }
      q[4] = lsb;
    }
  Stats st;
  raw10Stats(f.data(), w, h, lb, 1, st);
  CHECK(st.minV == 3 && st.maxV == 899 && st.samples == w * h && !st.untouchedWords);
  for (unsigned b = 0; b < 8; ++b) CHECK(st.band[b] == b * 128 + 3);
  CHECK(st.hist[0] == 32 && st.hist[2] == 32 && st.hist[14] == 32);
  uint32_t fnv = 2166136261u;
  for (uint8_t c : f) fnv = (fnv ^ c) * 16777619u;
  CHECK(st.fnv == fnv);
  // untouched tail
  std::vector<uint8_t> g(lb * h, 0xa5);
  memcpy(g.data(), f.data(), 160);
  Stats s2;
  raw10Stats(g.data(), w, h, lb, 2, s2);
  CHECK(s2.untouchedWords == (lb * h - 160) / 8 && s2.firstUntouched == 160);
}

int main() {
  testPure();
  testBringUp();
  testRing();
  testClaim();
  testRecovery();
  testStats();
  if (fails) { printf("camss host: %d FAIL\n", fails); return 1; }
  printf("camss host: PASS\n");
  return 0;
}
