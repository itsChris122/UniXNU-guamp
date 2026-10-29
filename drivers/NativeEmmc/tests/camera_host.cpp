// Host test of GuampCameraCore.h (PM8008 LDOs, CCI queue commands, power sequences, chip id probe, GCAM parser) and
// of the RDI skeleton plan (GuampCameraRdi.h), against a simulated PM8008 and a simulated CCI with four sensors that
// answer only when their rails, MCLK and reset line are right. Optional argument: a .gcam file made by
// analisi/camera/tools/sensormodule.py --gcam (vendor data, kept outside the repo) to check the parser on real tables.
#include "../GuampCameraCore.h"
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
};
static Sim *S;

static bool ldoOnSim(unsigned n) { return S->pm[1][ldoBase(n) + kLdoEnable] & 0x80; }
static uint32_t ldoMvSim(unsigned n) { return S->pm[1][ldoBase(n) + kLdoVsetLb] | S->pm[1][ldoBase(n) + kLdoVsetLb + 1] << 8; }

static bool powered(const Sensor &s) {
  if (!(S->pm[0][kPmChipEnable] & 1)) return false;
  auto ok = [](const Rail &r) { return !r.ldo || (ldoOnSim(r.ldo) && ldoMvSim(r.ldo) == ldoMv(r.uv)); };
  return ok(s.vio) && ok(s.vana) && ok(s.vdig) && S->mclk[s.mclk] && S->gpioHigh[s.resetGpio];
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
      for (unsigned k = 0; k < n; ++k) {
        const uint32_t a = S->regPtr + k;
        uint8_t v = 0;
        const uint16_t id = S->sensorId[s->cell];
        if (s->idBytes == 2 && a == s->idReg) v = id >> 8;
        else if (s->idBytes == 2 && a == s->idReg + 1u) v = id & 0xff;
        else if (s->idBytes == 1 && a == s->idReg) v = id & 0xff;
        bytes.push_back(v);
      }
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
      if (!ack) nack = true;
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
static void delayUs(void *, unsigned) {}
static void sleepMs(void *, unsigned) {}
static void noteFn(void *, const char *, int, uint32_t, uint32_t) {}

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
  CHECK(cci.init(kCciFast) == 0 && sim.cciReset);
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

static void put16(std::vector<uint8_t> &b, uint16_t v) { b.push_back(v & 0xff); b.push_back(v >> 8); }
static void testGcamSynthetic() {
  std::vector<uint8_t> b = {'G', 'C', 'A', 'M', 1, 0, 0, 0, 1, 0, 0, 0};
  const char name[16] = "mot_ov02b10";
  b.insert(b.end(), name, name + 16);
  b.insert(b.end(), {0x3c, 1, 1, 0});
  put16(b, 3); put16(b, 0x2b); put16(b, 1600); put16(b, 1200); put16(b, 1); put16(b, 0x2b);
  b.insert(b.end(), {0x94, 2, 0, 0});
  put16(b, 1); put16(b, 2); put16(b, 1); put16(b, 1); put16(b, 1); put16(b, 0);
  b.insert(b.end(), {8, 0, 10, 0, 1, 0, 0, 0});                       // power: RESET 1 + 10 ms
  for (unsigned k = 0; k < 5; ++k) { put16(b, 0xfd); put16(b, k); b.push_back(1); b.push_back(1); put16(b, k == 1 ? 5000 : 0); }
  GcamSensor s;
  CHECK(gcamFind(b.data(), b.size(), "ov02b10", &s) == 0 && s.slave7 == 0x3c && s.width == 1600 && s.laneMbps == 660);
  CHECK(gcamFind(b.data(), b.size(), "mot_ov02b10", &s) == 0 && s.nInit == 2);
  CHECK(gcamReg(s.regs[0] + 8).delayUs == 5000 && gcamReg(s.regs[3]).data == 4);
  CHECK(gcamFind(b.data(), b.size(), "s5k4h7", &s) == kErrRange);
  for (size_t cut = 0; cut < b.size(); ++cut) CHECK(gcamFind(b.data(), cut, "ov02b10", &s) != 0);
}

static void testGcamFile(const char *path) {
  FILE *f = fopen(path, "rb");
  if (!f) { printf("FAIL cannot open %s\n", path); ++fails; return; }
  std::vector<uint8_t> b;
  int c;
  while ((c = fgetc(f)) != EOF) b.push_back(static_cast<uint8_t>(c));
  fclose(f);
  for (unsigned i = 0; i < kSensors; ++i) {
    GcamSensor s;
    const int r = gcamFind(b.data(), b.size(), kBoard[i].name, &s);
    CHECK(r == 0);
    if (r) continue;
    CHECK(s.slave7 == kBoard[i].slave7 && s.idReg == kBoard[i].idReg && s.id == kBoard[i].id && s.addrBytes == kBoard[i].addrBytes);
    printf("  gcam %-9s slave 0x%02x id 0x%04x mode %u %ux%u %u lane %u Mbit/s: power %u, init %u, mode %u, on %u, off %u writes\n",
           kBoard[i].name, s.slave7, s.id, s.mode, s.width, s.height, s.lanes, s.laneMbps, s.nPowerUp, s.nInit, s.nMode, s.nStreamOn, s.nStreamOff);
  }
}

int main(int argc, char **argv) {
  testEncoding();
  testLdo();
  testProbe();
  testRdi();
  testGcamSynthetic();
  if (argc > 1) testGcamFile(argv[1]);
  printf("camera_host: %s\n", fails ? "FAIL" : "PASS");
  return fails != 0;
}
