// Host test of GuampCameraCore.h (PM8008 LDOs, CCI queue commands, batched writes, power sequences, chip id probe),
// GuampCameraSensor.h (GCAM v2 parser, mode choice, MIPI facts, exposure/gain registers, sensor session) and of the
// RDI skeleton plan (GuampCameraRdi.h), against a simulated PM8008 and a simulated CCI with four sensors that answer
// only when their rails, MCLK and reset line are right and keep a register file. Optional argument: a .gcam file made
// by analisi/camera/tools/sensormodule.py --gcam (vendor data, kept outside the repo) to run every sensor's real
// tables through the session.
#include "../GuampCameraCore.h"
#include "../GuampCameraSensor.h"
#include "../GuampCameraRdi.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <string>

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

using namespace gcam;

struct Sim {
  // PM8008
  uint8_t pm[2][0x10000] = {};        // [0] = 0x08, [1] = 0x09
  bool pmPresent = true;
  unsigned pmWrites = 0;
  // pins
  bool mclk[4] = {}, gpioHigh[128] = {};
  std::vector<std::string> trace;
  // CCI
  std::vector<uint32_t> q[2][2];
  std::vector<uint32_t> rdbuf[2];
  uint32_t status = 0, timing[2][5] = {};
  bool cciReset = false;
  bool sensorPresent[4] = {true, true, true, true};
  uint16_t sensorId[4] = {0xf8d1, 0x487b, 0x002b, 0x002b};
  uint32_t regPtr = 0;
  unsigned reads = 0;
  // sensor register files: SMIA byte map (S5K direct page 0x4000 only), OV02B {page << 8 | reg}
  std::vector<uint8_t> regs[4] = {std::vector<uint8_t>(0x10000), std::vector<uint8_t>(0x10000), std::vector<uint8_t>(0x10000),
                                  std::vector<uint8_t>(0x10000)};
  unsigned indirect[4] = {}, regWrites[4] = {}, writeCmds[4] = {}, s5kPage = 0x4000, maxQueue = 0, queueRuns = 0;
  uint8_t frame[4] = {};
};
static Sim *S;

static bool ldoOnSim(unsigned n) { return S->pm[1][ldoBase(n) + kLdoEnable] & 0x80; }
static uint32_t ldoMvSim(unsigned n) { return S->pm[1][ldoBase(n) + kLdoVsetLb] | S->pm[1][ldoBase(n) + kLdoVsetLb + 1] << 8; }

static bool powered(const Sensor &s) {
  if (!(S->pm[0][kPmChipEnable] & 1)) return false;
  auto ok = [](const Rail &r) { return !r.ldo || (ldoOnSim(r.ldo) && ldoMvSim(r.ldo) == ldoMv(r.uv)); };
  return ok(s.vio) && ok(s.vana) && ok(s.vdig) && S->mclk[s.mclk] && S->gpioHigh[s.resetGpio];
}

static bool ovSensor(const Sensor &s) { return s.addrBytes == 1; }
static uint32_t regKey(const Sensor &s, uint32_t a) { return ovSensor(s) ? (S->regs[s.cell][0xfd] << 8 | (a & 0xff)) : a & 0xffff; }
static bool streaming(const Sensor &s) { return ovSensor(s) ? (S->regs[s.cell][0x3c2] & 1) : (S->regs[s.cell][0x0100] & 1); }

// One WRITE command: address (sensor width) then data bytes; 0x6F12 = S5K indirect port, otherwise auto increment.
static void sensorWrite(const Sensor &s, const std::vector<uint8_t> &by) {
  if (by.size() <= s.addrBytes) { printf("sim: write without data\n"); ++fails; return; }
  uint32_t a = 0;
  for (unsigned k = 0; k < s.addrBytes; ++k) a = a << 8 | by[k];
  const size_t nd = by.size() - s.addrBytes;
  if (s.addrBytes == 2 && a == kSamsungDataPort) {
    if (nd % 2) { printf("sim: odd 0x6f12 data\n"); ++fails; }
    S->indirect[s.cell] += static_cast<unsigned>(nd / 2);
    S->regWrites[s.cell] += static_cast<unsigned>(nd / 2);
    return;
  }
  if (s.addrBytes == 2 && a == 0x6028 && nd == 2) { S->s5kPage = by[2] << 8 | by[3]; ++S->regWrites[s.cell]; return; }
  for (size_t k = 0; k < nd; ++k) {
    const uint32_t ra = a + static_cast<uint32_t>(k);
    if (s.cell == 0 && S->s5kPage != 0x4000 && ra < 0x6000) continue;   // another direct page: not modelled
    if (ovSensor(s) && (ra & 0xff) == 0xfd) { S->regs[s.cell][0xfd] = by[s.addrBytes + k]; continue; }   // OV page select, any page
    S->regs[s.cell][regKey(s, ra)] = by[s.addrBytes + k];
  }
  // a vendor entry with 2 data bytes counts as one register write
  S->regWrites[s.cell] += s.addrBytes == 2 && s.cell == 0 && nd == 2 ? 1 : static_cast<unsigned>(nd);
}
static uint8_t sensorRead(const Sensor &s, uint32_t a) {
  const uint16_t id = S->sensorId[s.cell];
  if (s.idBytes == 2 && a == s.idReg) return id >> 8;
  if (s.idBytes == 2 && a == s.idReg + 1u) return id & 0xff;
  if (s.idBytes == 1 && a == s.idReg && (!ovSensor(s) || !S->regs[s.cell][0xfd])) return id & 0xff;   // OV02B: page 0 only
  if (!ovSensor(s) && a == 0x0005) return streaming(s) ? (S->frame[s.cell] = static_cast<uint8_t>(S->frame[s.cell] + 7)) : S->frame[s.cell];
  return S->regs[s.cell][regKey(s, a)];
}

static int pmRead(void *, uint8_t dev, uint16_t reg, uint8_t *v) {
  if (!S->pmPresent || (dev != kPmChip && dev != kPmLdo)) return -6;
  uint8_t x = S->pm[dev - kPmChip][reg];
  if (dev == kPmLdo && (reg & 0xff) == kLdoStatus1) {     // VREG_READY when enabled and the chip is on
    const unsigned n = ((reg >> 8) & 0xff) - 0x40 + 1;
    x = (n >= 1 && n <= kLdos && ldoOnSim(n) && (S->pm[0][kPmChipEnable] & 1)) ? 0x80 : 0;
  }
  *v = x;
  return 0;
}
static int pmWrite(void *, uint8_t dev, uint16_t reg, uint8_t v) {
  if (!S->pmPresent || (dev != kPmChip && dev != kPmLdo)) return -6;
  S->pm[dev - kPmChip][reg] = v;
  ++S->pmWrites;
  if (dev == kPmLdo && (reg & 0xff) == kLdoEnable) {
    char b[64];
    snprintf(b, sizeof b, "L%uP=%s", ((reg >> 8) & 0xff) - 0x40 + 1, (v & 0x80) ? "on" : "off");
    S->trace.push_back(b);
  }
  return 0;
}

static const Sensor *sensorAt(unsigned m, uint8_t slave) {
  for (unsigned i = 0; i < kSensors; ++i)
    if (kBoard[i].cci == m && kBoard[i].slave7 == slave) return &kBoard[i];
  return nullptr;
}

static void execQueue(unsigned m, unsigned qq) {
  std::vector<uint32_t> &w = S->q[m][qq];
  if (w.size() > S->maxQueue) S->maxQueue = static_cast<unsigned>(w.size());
  if (w.size() > (qq ? kQ1Words : kQ0Words)) { printf("sim: queue %u overflow (%zu words)\n", qq, w.size()); ++fails; }
  ++S->queueRuns;
  uint8_t slave = 0;
  bool nack = false;
  size_t i = 0;
  while (i < w.size()) {
    const uint32_t word = w[i];
    const uint32_t cmd = word & 0xf;
    if (cmd == kCmdSetParam) { slave = (word >> 4) & 0x7f; ++i; continue; }
    if (cmd == kCmdReport) { S->status |= irqReport(m, qq); ++i; continue; }
    const Sensor *s = sensorAt(m, slave);
    const bool ack = s && S->sensorPresent[s->cell] && powered(*s);
    if (cmd == kCmdWriteNoStop) {
      const unsigned n = (word >> 4) & 0xf;
      uint32_t a = 0;
      for (unsigned k = 1; k <= n; ++k) a = a << 8 | ((word >> (8 * k)) & 0xff);
      S->regPtr = a;
      if (!ack) nack = true;
      ++i;
      continue;
    }
    if (cmd == kCmdRead) {
      const unsigned n = (word >> 4) & 0xf;
      if (!ack) { nack = true; ++i; continue; }
      std::vector<uint8_t> bytes = {static_cast<uint8_t>(slave << 1 | 1)};
      for (unsigned k = 0; k < n; ++k) bytes.push_back(sensorRead(*s, S->regPtr + k));
      for (size_t k = 0; k < bytes.size(); k += 4) {
        uint32_t d = 0;
        for (size_t j = 0; j < 4 && k + j < bytes.size(); ++j) d |= static_cast<uint32_t>(bytes[k + j]) << (8 * j);
        S->rdbuf[m].push_back(d);
      }
      ++S->reads;
      S->status |= irqRdDone(m);
      ++i;
      continue;
    }
    if (cmd == kCmdWrite) {
      const unsigned len = (word >> 4) & 0xf;
      const size_t words = (1 + len + 3) / 4;
      if (len > kPayload) { printf("sim: write payload %u > %u\n", len, kPayload); ++fails; }
      if (i + words > w.size()) { printf("sim: write command past the queue end\n"); ++fails; break; }
      if (!ack) nack = true;
      std::vector<uint8_t> by;
      for (size_t k = 1; k < 1 + len; ++k) by.push_back(static_cast<uint8_t>(w[i + k / 4] >> (8 * (k % 4))));
      if (ack) { sensorWrite(*s, by); ++S->writeCmds[s->cell]; }
      i += words;
      continue;
    }
    printf("sim: unknown cci command 0x%x\n", word);
    ++fails;
    ++i;
  }
  w.clear();
  if (nack) S->status |= m ? (qq ? 1u << 30 : 1u << 29) : (qq ? 1u << 28 : 1u << 27);
}

static uint32_t cciRd(void *, uint32_t off) {
  for (unsigned m = 0; m < 2; ++m) {
    for (unsigned qq = 0; qq < 2; ++qq) if (off == qCur(m, qq)) return static_cast<uint32_t>(S->q[m][qq].size());
    if (off == mReadLevel(m)) return static_cast<uint32_t>(S->rdbuf[m].size());
    if (off == mReadData(m)) {
      if (S->rdbuf[m].empty()) return 0;
      uint32_t d = S->rdbuf[m].front();
      S->rdbuf[m].erase(S->rdbuf[m].begin());
      return d;
    }
  }
  if (off == kCciIrqStatus0) return S->status;
  if (off == kCciHwVersion) return 0x10020009;
  return 0;
}
static void cciWr(void *, uint32_t off, uint32_t v) {
  if (off == kCciReset) { S->status |= kIrqRstDone; S->cciReset = true; for (auto &a : S->q) for (auto &b : a) b.clear(); return; }
  if (off == kCciHaltReq) { for (unsigned m = 0; m < 2; ++m) if (v & (1u << m)) S->status |= irqHaltAck(m); return; }
  if (off == kCciIrqClear0) { S->status &= ~v; return; }
  if (off == kCciQueueStart) { for (unsigned b = 0; b < 4; ++b) if (v & (1u << b)) execQueue(b / 2, b % 2); return; }
  for (unsigned m = 0; m < 2; ++m) {
    for (unsigned qq = 0; qq < 2; ++qq) if (off == qLoad(m, qq)) { S->q[m][qq].push_back(v); return; }
    if (off == mScl(m)) S->timing[m][0] = v;
    if (off == mMisc(m)) S->timing[m][4] = v;
  }
}
static void gpio(void *, unsigned g, int high) {
  if (S->gpioHigh[g] != !!high) S->trace.push_back("gpio" + std::to_string(g) + (high ? "=1" : "=0"));
  S->gpioHigh[g] = high;
}
static int mclk(void *, unsigned idx, unsigned, int on) {
  S->mclk[idx] = on;
  S->trace.push_back("mclk" + std::to_string(idx) + (on ? "=on" : "=off"));
  return 0;
}
static unsigned sleptMs = 0;
static void delayUs(void *, unsigned) {}
static void sleepMs(void *, unsigned ms) { sleptMs += ms; }
static void noteFn(void *, const char *s, int rc, uint32_t a, uint32_t b) { if (getenv("CAM_NOTES")) printf("  note %s rc %d 0x%x 0x%x\n", s, rc, a, b); }

static const Ops kOps = {nullptr, pmRead, pmWrite, cciRd, cciWr, gpio, mclk, delayUs, sleepMs, noteFn};

static std::string joined(size_t from) {
  std::string out;
  for (size_t i = from; i < S->trace.size(); ++i) out += (out.empty() ? "" : " ") + S->trace[i];
  return out;
}

static void testEncoding() {
  CHECK(ldoMv(1052000) == 1056 && ldoMv(1056000) == 1056 && ldoMv(1200000) == 1200 && ldoMv(2800000) == 2800 && ldoMv(1800000) == 1800);
  CHECK(ldoBase(1) == 0x4000 && ldoBase(7) == 0x4600);
  CHECK(Cci::setParam(0x2d) == (1u | 0x2du << 4 | 3u << 16));
  CHECK(Cci::addrNoStop(0x0003, 1) == 0x031b);
  CHECK(Cci::addrNoStop(0x1234, 2) == (0x2bu | 0x12u << 8 | 0x34u << 16));
  CHECK(irqReport(0, 0) == 0x10 && irqReport(0, 1) == 0x100 && irqReport(1, 0) == 0x10000 && irqReport(1, 1) == 0x100000);
  CHECK(irqRdDone(1) == 0x1000 && qLoad(1, 1) == 0x610 && qCur(0, 1) == 0x404 && qExec(1, 0) == 0x500);
}

static void testLdo() {
  Sim sim; S = &sim;
  LdoState st = {};
  CHECK(ldoOn(kOps, st, 7, 1800000) == kErrTimeout);               // chip enable still off: never ready, then disabled
  CHECK(!ldoOnSim(7) && !st.users[6]);
  CHECK(pmChip(kOps, true) == 0 && (sim.pm[0][kPmChipEnable] & 1));
  CHECK(ldoOn(kOps, st, 7, 1800000) == 0 && ldoOnSim(7) && ldoMvSim(7) == 1800);
  CHECK(ldoOn(kOps, st, 7, 1800000) == 0 && st.users[6] == 2);
  CHECK(ldoOn(kOps, st, 7, 1200000) == kErrInval);                  // shared rail at another voltage
  CHECK(ldoOff(kOps, st, 7) == 0 && ldoOnSim(7));
  CHECK(ldoOff(kOps, st, 7) == 0 && !ldoOnSim(7) && ldoUsers(st) == 0);
  CHECK(ldoSet(kOps, 3, 1800000) == kErrRange && ldoSet(kOps, 1, 1300000) == kErrRange && ldoSet(kOps, 8, 1800000) == kErrRange);
  sim.pmPresent = false;
  CHECK(ldoOn(kOps, st, 4, 2800000) == -6);
}

static void testProbe() {
  Sim sim; S = &sim;
  LdoState st = {};
  Cci cci = {&kOps};
  CHECK(cci.init(kCciFast) == 0 && sim.cciReset && cci.resetSeen);
  CHECK(sim.timing[0][0] == (0x26u << 16 | 0x38) && sim.timing[1][4] == (6u << 4 | 3));
  CHECK(pmChip(kOps, true) == 0);
  for (unsigned i = 0; i < kSensors; ++i) {
    const size_t from = sim.trace.size();
    ProbeResult p = probe(kOps, st, cci, kBoard[i], kCciFast);
    CHECK(p.rc == 0 && p.id == kBoard[i].id);
    CHECK(ldoUsers(st) == 0 && !sim.mclk[kBoard[i].mclk] && !sim.gpioHigh[kBoard[i].resetGpio]);
    for (unsigned n = 1; n <= kLdos; ++n) CHECK(!ldoOnSim(n));
    if (i == 0) {
      const std::string t = joined(from);
      CHECK(t == "mclk0=on L7P=on L4P=on L2P=on gpio18=1 mclk0=off gpio18=0 L2P=off L4P=off L7P=off");
      if (t != "mclk0=on L7P=on L4P=on L2P=on gpio18=1 mclk0=off gpio18=0 L2P=off L4P=off L7P=off") printf("  trace: %s\n", t.c_str());
    }
    if (i == 3) {
      const std::string t = joined(from);
      CHECK(t == "mclk1=on L7P=on L3P=on gpio19=1 gpio19=0 L3P=off L7P=off mclk1=off");
      if (t != "mclk1=on L7P=on L3P=on gpio19=1 gpio19=0 L3P=off L7P=off mclk1=off") printf("  trace: %s\n", t.c_str());
    }
  }
  CHECK(sim.reads == 4);
  // absent sensor: NACK after three tries with recovery, still clean
  sim.sensorPresent[1] = false;
  ProbeResult p = probe(kOps, st, cci, kBoard[1], kCciFast);
  CHECK(p.rc == kErrNack && ldoUsers(st) == 0 && !sim.mclk[2]);
  sim.sensorPresent[1] = true;
  // wrong id
  sim.sensorId[2] = 0x0056;
  p = probe(kOps, st, cci, kBoard[2], kCciFast);
  CHECK(p.rc == kErrMismatch && p.id == 0x56);
  // PM8008 gone mid-way: power-up fails at the first rail, nothing left on
  sim.pmPresent = false;
  p = probe(kOps, st, cci, kBoard[0], kCciFast);
  CHECK(p.rc == -6 && !sim.mclk[0] && !sim.gpioHigh[18] && ldoUsers(st) == 0);
  // a write goes out as SET_PARAM + WRITE + REPORT on queue 0
  sim.pmPresent = true;
  Power pw = {&kOps, &st, &kBoard[0]};
  CHECK(pw.up() == 0);
  CHECK(cci.write(0, 0x2d, 0x0100, 2, 0x01, 1) == 0 && sim.q[0][0].empty());
  CHECK(cci.write(0, 0x77, 0x0100, 2, 0x01, 1) == kErrNack);
  CHECK(pw.down() == 0 && ldoUsers(st) == 0);
}

// Batched writes: 0x6F12 packing (4 words per command), queue never above 64 words, delays flush, NACK reported.
static void testWriteList() {
  Sim sim; S = &sim;
  LdoState st = {};
  Cci cci = {&kOps};
  CHECK(cci.init(kCciFast) == 0 && pmChip(kOps, true) == 0);
  Power pw = {&kOps, &st, &kBoard[0]};
  CHECK(pw.up() == 0);
  std::vector<Reg> v;
  v.push_back({0x6028, 0x2000, 2, 2, 0});
  v.push_back({0x602a, 0x3f5c, 2, 2, 0});
  for (unsigned k = 0; k < 1001; ++k) v.push_back({0x6f12, static_cast<uint16_t>(k), 2, 2, 0});
  v.push_back({0x6028, 0x4000, 2, 2, 0});
  v.push_back({0x0340, 0x0c, 2, 1, 0});
  v.push_back({0x0341, 0x6d, 2, 1, 2000});                              // delay: flush + 2 ms sleep
  v.push_back({0x0100, 0x0103, 2, 2, 0});
  unsigned done = 0;
  sleptMs = 0;
  CHECK(cci.writeRegs(0, 0x2d, v.data(), static_cast<unsigned>(v.size()), &done) == 0 && done == v.size());
  CHECK(sim.indirect[0] == 1001 && sim.regWrites[0] == v.size());
  CHECK(sim.writeCmds[0] == 2 + 251 + 1 + 2 + 1);                       // 1001 words = 250 x 4 + 1
  CHECK(sim.maxQueue <= kQ0Words && sleptMs == 2);
  CHECK(sim.regs[0][0x0340] == 0x0c && sim.regs[0][0x0341] == 0x6d && sim.regs[0][0x0100] == 1 && sim.regs[0][0x0101] == 3);
  // the wrong slave: NACK, nothing counted as done past the failing run
  CHECK(cci.writeRegs(0, 0x77, v.data(), 4, &done) == kErrNack && done == 0);
  Reg bad = {0x0100, 1, 3, 1, 0};
  CHECK(cci.writeRegs(0, 0x2d, &bad, 1) == kErrInval);
  CHECK(pw.down() == 0);
}

static void testExposure() {
  Mode m = {};
  m.width = 1280; m.height = 720; m.lanes = 4; m.bits = 10; m.opPixClk = 145600000; m.llpck = 5024; m.fll = 816; m.fpsMilli = 120010;
  CHECK(m.laneMbps() == 364);
  CHECK(m.linePs() == 1000000000000000ull / (120010ull * 816));            // ~10.2 us
  const ExpModel &g = kExp[0];
  Reg r[kExpRegsMax];
  ExposureApplied a;
  unsigned n = exposureRegs(g, kBoard[0], m, {100, 512, 0}, r, &a);
  CHECK(n == 9 && r[0].addr == 0x6028 && r[0].data == 0x4000 && r[1].addr == 0x0104 && r[1].data == 1 && r[8].addr == 0x0104 && r[8].data == 0);
  CHECK(r[6].addr == 0x0204 && r[6].data == 0x00 && r[7].addr == 0x0205 && r[7].data == 0x40 && a.code == 0x40 && a.gainQ8 == 512);
  CHECK(r[2].addr == 0x0340 && (r[2].data << 8 | r[3].data) == 816 && (r[4].data << 8 | r[5].data) == 100 && a.lines == 100);
  // long exposure grows the frame length, clamps at maxLineCount; gain clamps to 16x and 1x
  n = exposureRegs(g, kBoard[0], m, {2000, 100000, 0}, r, &a);
  CHECK(a.fll == 2004 && a.lines == 2000 && a.code == 0x200);
  n = exposureRegs(g, kBoard[0], m, {60000, 10, 0}, r, &a);
  CHECK(a.fll == 47974 && a.lines == 47970 && a.code == 0x20);
  n = exposureRegs(g, kBoard[0], m, {0, 256, 0}, r, &a);
  CHECK(a.lines == 2);
  // S5K4H7: no group hold, no page
  n = exposureRegs(kExp[1], kBoard[1], m, {100, 256, 0}, r, &a);
  CHECK(n == 6 && r[0].addr == 0x0340 && r[5].addr == 0x0205 && r[5].data == 0x20);
  // OV02B: page, vertical blank, steps
  Mode ov = {};
  ov.fll = 1221; ov.fpsMilli = 30070; ov.llpck = 448;
  n = exposureRegs(kExp[3], kBoard[3], ov, {500, 256 * 3 + 100, 0}, r, &a);
  CHECK(n == 7 && r[0].addr == 0xfd && r[0].data == 1 && r[1].addr == 0x14 && r[2].data == 1 && r[6].addr == 0xfe && r[6].data == 2);
  CHECK(r[3].data == 1 && r[4].data == 0xf4 && r[5].addr == 0x22 && a.code == 0x36 && a.gainQ8 == 0x36 * 16);   // 3.39x -> 3.375x
  CHECK(gainCode(kExp[3], 256 * 10) == 0xa0 && gainCode(kExp[3], 256 * 9 + 50) == 0x90 && gainCode(kExp[3], 256 * 20) == 0xf8);
  CHECK(gainCode(kExp[3], 256 * 5 + 40) == 0x50 && gainCode(kExp[3], 100) == 0x10);
  n = exposureRegs(kExp[3], kBoard[3], ov, {3000, 256, 0}, r, &a);
  CHECK(a.fll == 3008 && (r[1].data << 8 | r[2].data) == 3008 - 1220);
  n = exposureRegs(kExp[3], kBoard[3], ov, {40000, 256, 0}, r, &a);
  CHECK(a.fll == 32759 && a.lines == 32751);
  CHECK(linesForNs(ov, 10000000ull) == 367);                               // 10 ms at 27.2 us per line
}

static void testRdi() {
  rdi::Write w[64];
  rdi::Cfg c;
  c.bufferIova = 0x80000000u;
  const unsigned n = rdi::plan(c, w, 64);
  CHECK(n > 20 && n < 64);
  CHECK(rdi::plan(c, w, 4) == 0);
  bool wmEnable = false, tpgEnable = false, cfg0 = false;
  for (unsigned i = 0; i < n; ++i) {
    if (w[i].block == rdi::kTfe && w[i].off == 0x2300 && w[i].val == 0x10001) wmEnable = true;
    if (w[i].block == rdi::kTpgBlk && w[i].off == rdi::kTpgCtrl && (w[i].val & 1)) tpgEnable = i == n - 1;
    if (w[i].block == rdi::kCsid && w[i].off == 0x300 && !w[i].rmwOr) cfg0 = w[i].val == (0x2bu << 16 | 0xfu << 12 | 5u);
  }
  CHECK(wmEnable && tpgEnable && cfg0);
  CHECK(rdi::raw10Stride(1600) == 2000 && rdi::raw10Stride(1920) == 2400);
  if (getenv("RDI_PLAN"))
    for (unsigned i = 0; i < n; ++i) printf("  %-5s +0x%04x %s 0x%08x  %s\n", w[i].block == 0 ? "top" : w[i].block == 1 ? "tpg" : w[i].block == 2 ? "csid" : "tfe",
                                            w[i].off, w[i].rmwOr ? "|=" : "= ", w[i].val, w[i].what);
}

// ---- synthetic GCAM v2 ---------------------------------------------------------------------------------------------
static void put16(std::vector<uint8_t> &b, uint16_t v) { b.push_back(v & 0xff); b.push_back(v >> 8); }
static void put32(std::vector<uint8_t> &b, uint32_t v) { put16(b, v & 0xffff); put16(b, v >> 16); }
static void putReg(std::vector<uint8_t> &b, uint16_t a, uint16_t d, uint8_t ab, uint8_t db, uint16_t dl = 0) {
  put16(b, a); put16(b, d); b.push_back(ab); b.push_back(db); put16(b, dl);
}
struct SynMode { uint16_t w, h; uint32_t pclk, llpck, fll, fps; std::vector<std::vector<uint16_t>> regs; };
static void synSensor(std::vector<uint8_t> &b, const char *name, const Sensor &s, uint8_t dataBytes, const std::vector<SynMode> &modes,
                      const std::vector<std::vector<uint16_t>> &init, const std::vector<std::vector<uint16_t>> &on,
                      const std::vector<std::vector<uint16_t>> &off) {
  char n[16] = {};
  strncpy(n, name, 15);
  b.insert(b.end(), n, n + 16);
  b.insert(b.end(), {s.slave7, s.addrBytes, dataBytes, static_cast<uint8_t>(modes.size())});
  put16(b, s.idReg); put16(b, s.id);
  put16(b, 1); put16(b, static_cast<uint16_t>(init.size())); put16(b, static_cast<uint16_t>(on.size())); put16(b, static_cast<uint16_t>(off.size()));
  put16(b, 64); put16(b, 4000); put16(b, 3000);
  b.insert(b.end(), 10, 0);
  for (const SynMode &m : modes) {
    const size_t at = b.size();
    put16(b, m.w); put16(b, m.h); put16(b, 0); put16(b, 0);
    b.insert(b.end(), {4, 0x2b, 0, 1});
    put32(b, m.pclk); put32(b, m.llpck); put32(b, m.fll); put32(b, m.fps);
    b.insert(b.end(), {1, 1, 14, 10});
    put16(b, 0); put16(b, 0); b.push_back(0); b.push_back(0); put16(b, static_cast<uint16_t>(m.regs.size()));
    b.insert(b.end(), 48 - (b.size() - at), 0);
  }
  b.insert(b.end(), {8, 0, 1, 0, 1, 0, 0, 0});                       // power: RESET 1 + 1 ms (informative)
  auto list = [&](const std::vector<std::vector<uint16_t>> &L) {
    for (const auto &r : L) putReg(b, r[0], r[1], s.addrBytes, static_cast<uint8_t>(r[2]), r.size() > 3 ? r[3] : 0);
  };
  list(init); list(on); list(off);
  for (const SynMode &m : modes) list(m.regs);
}

static std::vector<uint8_t> synGcam() {
  std::vector<uint8_t> b = {'G', 'C', 'A', 'M', 2, 0, 0, 0, 2, 0, 0, 0};
  // gm1st: two modes (full 4000x3000, small 1280x720), init with a 0x6F12 burst
  std::vector<std::vector<uint16_t>> init = {{0x6028, 0x4000, 2}, {0x6010, 0x0001, 2, 6000}, {0x6028, 0x2000, 2}, {0x602a, 0x3f5c, 2}};
  for (unsigned k = 0; k < 9; ++k) init.push_back({0x6f12, static_cast<uint16_t>(k), 2});
  SynMode full = {4000, 3000, 460800000, 5024, 3181, 30160, {{0x6028, 0x4000, 2}, {0x034c, 0x0fa0, 2}, {0x034e, 0x0bb8, 2}, {0x6028, 0x2000, 2}}};
  SynMode small = {1280, 720, 145600000, 5024, 816, 120010, {{0x6028, 0x4000, 2}, {0x034c, 0x0500, 2}, {0x034e, 0x02d0, 2}, {0x6028, 0x2000, 2}}};
  synSensor(b, "mot_s5kgm1st", kBoard[0], 2, {full, small}, init, {{0x6028, 0x4000, 2}, {0x0100, 0x0103, 2}}, {{0x0100, 0x00, 1}});
  // 4h7: one full mode
  SynMode f4 = {3264, 2448, 292800000, 3580, 2530, 30030, {{0x0100, 0, 1}, {0x034c, 0x0c, 1}, {0x034d, 0xc0, 1}, {0x034e, 0x09, 1}, {0x034f, 0x90, 1}}};
  synSensor(b, "mot_s5k4h7", kBoard[1], 1, {f4}, {{0x0b05, 1, 1}}, {{0x0100, 1, 1}}, {{0x0100, 0, 1}});
  return b;
}

static void testGcamSynthetic() {
  std::vector<uint8_t> b = synGcam();
  Gcam g;
  CHECK(gcamFind(b.data(), b.size(), "s5kgm1st", &g) == 0 && g.slave7 == 0x2d && g.nModes == 2 && g.nInit == 13 && g.id == 0xf8d1);
  CHECK(g.modes[1].width == 1280 && g.modes[1].laneMbps() == 364 && g.modes[0].laneMbps() == 1152 && g.modes[1].nRegs == 4);
  CHECK(gcamReg(g.init + 8).delayUs == 6000 && gcamReg(g.modes[1].regs + 8).data == 0x0500);
  CHECK(gcamFind(b.data(), b.size(), "mot_s5k4h7", &g) == 0 && g.nModes == 1 && g.modes[0].fll == 2530);
  CHECK(gcamFind(b.data(), b.size(), "ov02b10", &g) == kErrRange);
  for (size_t cut = 0; cut < b.size(); ++cut) CHECK(gcamFind(b.data(), cut, "s5k4h7", &g) != 0);
  std::vector<uint8_t> v1 = b;
  v1[4] = 1;
  CHECK(gcamFind(v1.data(), v1.size(), "s5kgm1st", &g) == kErrInval);
  // mode choice
  Mode m;
  Reg ov[kBinRegs];
  unsigned nov = 9;
  CHECK(gcamFind(b.data(), b.size(), "s5kgm1st", &g) == 0);
  CHECK(pickMode(g, kExp[0], kResLow, -1, &m, ov, &nov) && m.index == 1 && nov == 0);
  CHECK(pickMode(g, kExp[0], kResFull, -1, &m, ov, &nov) && m.index == 0 && m.width == 4000);
  CHECK(pickMode(g, kExp[0], kResLow, 0, &m, ov, &nov) && m.index == 0);
  CHECK(gcamFind(b.data(), b.size(), "s5k4h7", &g) == 0);
  CHECK(pickMode(g, kExp[1], kResLow, -1, &m, ov, &nov) && m.derived && m.width == 1632 && m.height == 1224 && nov == kBinRegs);
  CHECK(ov[1].addr == 0x0901 && ov[1].data == 0x22 && ov[4].addr == 0x034c && (ov[4].data << 8 | ov[5].data) == 1632);
  CHECK(pickMode(g, kExp[1], kResFull, -1, &m, ov, &nov) && !m.derived && m.width == 3264 && nov == 0);
  const Mipi mi = mipiOf(kBoard[1], m);
  CHECK(mi.csiphy == 2 && mi.lanes == 4 && mi.laneMbps == 732 && mi.dt == 0x2b && mi.strideBytes == 4080 && mi.laneAssign == 0x3210);
  CHECK(mi.bayer == camssBayer(m.cfa));
  // MCLK 19.2 MHz: lane rate and fps 0.8x, line time 1.25x
  Mode slow = m;
  const uint64_t lp = m.linePs();
  modeScaleMclk(&slow, false);
  CHECK(slow.laneMbps() >= 584 && slow.laneMbps() <= 586 && slow.fpsMilli == m.fpsMilli * 4 / 5);
  CHECK(slow.linePs() > lp * 124 / 100 && slow.linePs() < lp * 126 / 100);
  Mode same = m;
  modeScaleMclk(&same, true);
  CHECK(same.opPixClk == m.opPixClk && same.fpsMilli == m.fpsMilli);
}

// CamX colorFilterArrangement -> GuampCamssApi.h bayer (TFE pixel pattern): the blob CFAs of the four sensors.
static void testBayer() {
  CHECK(camssBayer(0) == 2);   // BGGR (ov02b10)
  CHECK(camssBayer(1) == 3);   // GBRG (s5kgm1st)
  CHECK(camssBayer(2) == 1);   // GRBG (s5k4h7)
  CHECK(camssBayer(3) == 0);   // RGGB
  CHECK(camssBayer(4) == 4);   // Y (ov02b1b)
  CHECK(camssBayer(9) == 0);
}

// Session over the simulator: open, configure, exposure, stream on/off with a moving frame counter, close.
static void sessionRun(const uint8_t *blob, size_t len, unsigned cell, Res res, bool verbose) {
  Sim *sim = new Sim; S = sim;
  LdoState st = {};
  Cci cci = {&kOps};
  CHECK(cci.init(kCciFast) == 0 && pmChip(kOps, true) == 0);
  Gcam g;
  CHECK(gcamFind(blob, len, kBoard[cell].name, &g) == 0);
  Session ss;
  CHECK(ss.open(kOps, st, cci, kCciFast, kBoard[cell], &g) == 0 && ss.state == kPowered);
  CHECK(ss.streamOn() == kErrInval);                                     // not configured yet
  Mode m;
  Reg ov[kBinRegs];
  unsigned nov;
  pickMode(g, kExp[cell], res, -1, &m, ov, &nov);
  const int rc = ss.configure(res, -1, {linesForNs(m, 10000000ull), 512, 0});
  CHECK(rc == 0 && ss.state == kConfigured);
  const unsigned expectWrites = g.nInit + ss.mode.nRegs + ss.nOverlay + (kExp[cell].longExp ? 4 : 0);
  CHECK(sim->regWrites[cell] >= expectWrites);
  CHECK(sim->maxQueue <= kQ0Words);
  Status a, b;
  CHECK(ss.status(&a) == 0 && a.ok && a.id == kBoard[cell].id && a.lines == ss.exp.lines && a.fll == ss.exp.fll && a.gainCode == ss.exp.code);
  if (kExp[cell].kind == kExpSmia) CHECK(a.width == ss.mode.width && a.height == ss.mode.height);
  CHECK(ss.streamOn() == 0 && ss.state == kStreaming && streaming(kBoard[cell]));
  CHECK(ss.status(&b) == 0);
  if (kExp[cell].kind == kExpSmia) CHECK(b.frameCount != a.frameCount && b.modeSelect == 1);
  ExposureApplied e;
  CHECK(ss.setExposure({ss.exp.lines * 2, 256 * 4, 0}) == 0);
  e = ss.exp;
  CHECK(ss.status(&b) == 0 && b.lines == e.lines && b.gainCode == e.code);
  CHECK(ss.streamOff() == 0 && !streaming(kBoard[cell]) && ss.state == kConfigured);
  CHECK(ss.close() == 0 && ss.state == kOff && ldoUsers(st) == 0 && !sim->mclk[kBoard[cell].mclk]);
  if (verbose) {
    const Mipi mi = mipiOf(kBoard[cell], ss.mode);
    printf("  session %-9s %s mode %u%s %ux%u: %u writes in %u commands, %u queue runs (max %u words), %u indirect; exposure %u lines "
           "(%llu us) fll %u gain 0x%x; mipi csiphy %u %u lane %u Mbit/s dt 0x%x settle %u\n", kBoard[cell].name, res ? "full" : "low ",
           ss.mode.index, ss.mode.derived ? "b" : "", ss.mode.width, ss.mode.height, sim->regWrites[cell], sim->writeCmds[cell], sim->queueRuns,
           sim->maxQueue, sim->indirect[cell], e.lines, static_cast<unsigned long long>(e.exposureNs / 1000), e.fll, e.code, mi.csiphy, mi.lanes,
           mi.laneMbps, mi.dt, mi.settle);
  }
  delete sim;
}

static void testSessionSynthetic() {
  std::vector<uint8_t> b = synGcam();
  sessionRun(b.data(), b.size(), 0, kResLow, false);
  sessionRun(b.data(), b.size(), 0, kResFull, false);
  sessionRun(b.data(), b.size(), 1, kResLow, false);
  // a sensor that NACKs during init: the session reports it and powers down on close
  Sim *sim = new Sim; S = sim;
  LdoState st = {};
  Cci cci = {&kOps};
  CHECK(cci.init(kCciFast) == 0 && pmChip(kOps, true) == 0);
  Gcam g;
  CHECK(gcamFind(b.data(), b.size(), "s5k4h7", &g) == 0);
  Session ss;
  CHECK(ss.open(kOps, st, cci, kCciFast, kBoard[1], &g) == 0);
  sim->sensorPresent[1] = false;
  CHECK(ss.configure(kResLow, -1, {100, 256, 0}) == kErrNack && ss.state == kPowered);
  CHECK(ss.close() == 0 && ldoUsers(st) == 0);
  // table of another sensor: refused
  CHECK(gcamFind(b.data(), b.size(), "s5kgm1st", &g) == 0);
  sim->sensorPresent[1] = true;
  CHECK(ss.open(kOps, st, cci, kCciFast, kBoard[1], &g) == 0 && ss.configure(kResLow, -1, {100, 256, 0}) == kErrMismatch);
  CHECK(ss.close() == 0);
  delete sim;
}

static void testGcamFile(const char *path) {
  FILE *f = fopen(path, "rb");
  if (!f) { printf("FAIL cannot open %s\n", path); ++fails; return; }
  std::vector<uint8_t> b;
  int c;
  while ((c = fgetc(f)) != EOF) b.push_back(static_cast<uint8_t>(c));
  fclose(f);
  for (unsigned i = 0; i < kSensors; ++i) {
    Gcam g;
    const int r = gcamFind(b.data(), b.size(), kBoard[i].name, &g);
    CHECK(r == 0);
    if (r) continue;
    CHECK(g.slave7 == kBoard[i].slave7 && g.idReg == kBoard[i].idReg && g.id == kBoard[i].id && g.addrBytes == kBoard[i].addrBytes);
    printf("  gcam %-9s slave 0x%02x id 0x%04x: power %u, init %u, on %u, off %u writes, black %u, %u modes\n", kBoard[i].name, g.slave7, g.id,
           g.nPowerUp, g.nInit, g.nOn, g.nOff, g.blackLevel, g.nModes);
    for (unsigned m = 0; m < g.nModes; ++m) {
      const Mode &d = g.modes[m];
      CHECK(d.dt == kDtRaw10 && d.bits == 10 && d.vc == 0);
      printf("    mode %u %ux%u @%u.%03u fps, %u lane %u Mbit/s, llpck %u fll %u, line %llu ns, bin %ux%u, cfa %u, settle %u, %u writes%s\n", m,
             d.width, d.height, d.fpsMilli / 1000, d.fpsMilli % 1000, d.lanes, d.laneMbps(), d.llpck, d.fll,
             static_cast<unsigned long long>(d.linePs() / 1000), d.hBin, d.vBin, d.cfa, d.settle, d.nRegs, d.extraW ? " + PDAF stream" : "");
    }
    sessionRun(b.data(), b.size(), i, kResLow, true);
    sessionRun(b.data(), b.size(), i, kResFull, true);
  }
}

int main(int argc, char **argv) {
  testEncoding();
  testBayer();
  testLdo();
  testProbe();
  testWriteList();
  testExposure();
  testRdi();
  testGcamSynthetic();
  testSessionSynthetic();
  if (argc > 1) testGcamFile(argv[1]);
  printf("camera_host: %s\n", fails ? "FAIL" : "PASS");
  return fails != 0;
}
