// Host test of the guamp microphone (audio agent 3): guamp_mic_dev.h ABI values, GuampMicRing.inc (capture ring, null
// source clock, overflow drops the oldest, ioctl core), GuampMicCodec.inc against a simulated LPASS (bolero TX/VA macro
// registers, LPI pins, the VA SoundWire master with its command FIFO, auto enumeration and banks, the wcd937x TX slave
// register file), and apr::MicPath of GuampMicPath.inc against a simulated ADSP (packets checked byte by byte against
// offsets written out from include/dsp/apr_audio-v2.h, order of the capture set-up, 4 reads in flight, READ_DONE into the
// sink, teardown in reverse, failures undone). No device access. Build: see run-pas-tests.sh.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <map>
#include <string>
#include <vector>
#include "../GuampMicRing.inc"
#include "../GuampMicCodec.inc"
#include "../GuampMicPath.inc"

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); exit(1); } } while (0)
static uint32_t u32(const std::vector<uint8_t> &p, uint32_t at) { uint32_t v; memcpy(&v, &p[at], 4); return v; }
static uint32_t u16(const std::vector<uint8_t> &p, uint32_t at) { uint16_t v; memcpy(&v, &p[at], 2); return v; }

// ---- ABI ----------------------------------------------------------------------------------------------------------------
static void abi() {
  CHECK(sizeof(guamp_mic_info) == 64 && sizeof(guamp_mic_position) == 24);
  CHECK(GUAMP_MIC_IOC_INFO == 0x40404d01u && GUAMP_MIC_IOC_START == 0x20004d02u && GUAMP_MIC_IOC_STOP == 0x20004d03u);
  CHECK(GUAMP_MIC_IOC_POSITION == 0x40184d04u && GUAMP_MIC_IOC_GAIN == 0x80044d05u && GUAMP_MIC_IOC_SET_MIC == 0x80044d06u);
  printf("abi: info 64, position 24; INFO %08x START %08x STOP %08x POSITION %08x GAIN %08x SET_MIC %08x\n",
         (unsigned)GUAMP_MIC_IOC_INFO, (unsigned)GUAMP_MIC_IOC_START, (unsigned)GUAMP_MIC_IOC_STOP,
         (unsigned)GUAMP_MIC_IOC_POSITION, (unsigned)GUAMP_MIC_IOC_GAIN, (unsigned)GUAMP_MIC_IOC_SET_MIC);
}

// ---- ring -----------------------------------------------------------------------------------------------------------
static void ring() {
  static uint8_t mem[GUAMP_MIC_RING_BYTES];
  guampmic::Capture c;
  c.buf = mem;
  c.bytes = sizeof(mem);
  c.ticksPerSec = 24000000;
  uint8_t pcm[960];
  for (uint32_t i = 0; i < sizeof(pcm); i += 2) { int16_t v = static_cast<int16_t>(i * 13 - 3000); memcpy(pcm + i, &v, 2); }
  CHECK(c.push(pcm, sizeof(pcm), 0) == 0);   // stopped: nothing captured
  CHECK(c.ioctl(GUAMP_MIC_IOC_START, nullptr, 24000000) == 0 && c.running);
  // null source: 10 ms = 480 frames of silence
  c.produce(24000000 + 240000);
  CHECK(c.framesCaptured() == 480 && c.used() == 960);
  uint8_t out[4096];
  CHECK(c.take(out, 961) == 960 && out[0] == 0 && c.used() == 0);
  // one second later with nobody reading: only the ring's worth stays, the rest is overrun
  c.produce(24000000 + 24000000);
  CHECK(c.framesCaptured() == 48000 && c.used() == GUAMP_MIC_RING_BYTES);
  CHECK(c.overruns == 48000 - 480 - GUAMP_MIC_RING_BYTES / 2);
  // ADSP source: periods, peak, overflow drops the oldest
  c.ioctl(GUAMP_MIC_IOC_STOP, nullptr, 0);
  c.reset();
  c.source = GUAMP_MIC_SOURCE_ADSP;
  c.ioctl(GUAMP_MIC_IOC_START, nullptr, 0);
  for (int i = 0; i < 40; i++) CHECK(c.push(pcm, sizeof(pcm), 100 + i) == 480);
  CHECK(c.framesCaptured() == 19200 && c.used() == GUAMP_MIC_RING_BYTES && c.overruns == 19200 - 16384);
  CHECK(c.peak == 3000 || c.peak > 3000);
  CHECK(c.take(out, 960) == 960 && memcmp(out, pcm + 960 - (GUAMP_MIC_RING_BYTES % 960), 0) == 0);
  // the newest frame is the last pushed one
  uint8_t all[GUAMP_MIC_RING_BYTES];
  const uint32_t n = c.take(all, sizeof(all));
  CHECK(n == GUAMP_MIC_RING_BYTES - 960 && memcmp(all + n - 960, pcm, 960) == 0);
  guamp_mic_info info;
  CHECK(c.ioctl(GUAMP_MIC_IOC_INFO, &info, 0) == 0 && info.abi == 1 && info.rate == 48000 && info.channels == 1 &&
        info.source == GUAMP_MIC_SOURCE_ADSP && info.frames_read == GUAMP_MIC_RING_BYTES / 2 && info.mic == GUAMP_MIC_AMIC3);
  uint32_t m = GUAMP_MIC_AMIC1;
  CHECK(c.ioctl(GUAMP_MIC_IOC_SET_MIC, &m, 0) == 16);   // running: EBUSY
  m = 2;
  CHECK(c.ioctl(GUAMP_MIC_IOC_SET_MIC, &m, 0) == 22);
  c.ioctl(GUAMP_MIC_IOC_STOP, nullptr, 0);
  m = GUAMP_MIC_AMIC1;
  CHECK(c.ioctl(GUAMP_MIC_IOC_SET_MIC, &m, 0) == 0 && c.mic == 1);
  int32_t g = 9999;
  CHECK(c.ioctl(GUAMP_MIC_IOC_GAIN, &g, 0) == 0 && c.gainCb == 4000);
  printf("ring: null source 48 kHz, overrun drops the oldest, peak, SET_MIC/GAIN rules\n");
}

// ---- simulated LPASS + SoundWire + wcd937x -----------------------------------------------------------------------------
struct Sim {
  std::map<uint64_t, uint32_t> mmio;
  std::map<uint16_t, uint8_t> wcd, slv;   // wcd937x register file, slave DP/SCP registers (dev 1)
  std::vector<std::string> trace;          // order of the interesting events
  uint32_t rdFifo = 0, rdCount = 0, bank = 0, broadcasts = 0, attachStatus = 1;
  uint64_t elapsedUs = 0;
  uint32_t &at(uint64_t a) { return mmio[a]; }
  void ev(const char *f, unsigned a, unsigned b) {
    char s[96];
    snprintf(s, sizeof(s), f, a, b);
    trace.push_back(s);
  }
  size_t find(const std::string &s) const {
    for (size_t i = 0; i < trace.size(); i++) if (trace[i] == s) return i;
    return static_cast<size_t>(-1);
  }
};
static Sim gSim;
static uint32_t simRd(void *, uint64_t a) {
  using namespace guampmic;
  if (a == kSwrm + kSwHwVersion) return 0x01050001;
  if (a == kSwrm + kSwCompParams) return (8u << 10) | (8u << 15);
  if (a == kSwrm + kSwFifoStatus) return gSim.rdCount << 16;
  if (a == kSwrm + kSwRdFifo) { gSim.rdCount = 0; return gSim.rdFifo; }
  if (a == kSwrm + kSwSlvStatus) return gSim.attachStatus << 2;
  if (a == kSwrm + swDevId1(1)) return 0x01170223;
  if (a == kSwrm + swDevId2(1)) return 0x0A;
  if (a == kSwrm + kSwMcpStatus) return gSim.bank;
  return gSim.at(a);
}
static void simWr(void *, uint64_t a, uint32_t v) {
  using namespace guampmic;
  if (a == kSwrm + kSwWrCmd || a == kSwrm + kSwRdCmd) {
    const uint16_t reg = static_cast<uint16_t>(v & 0xFFFF);
    const uint32_t id = (v >> 16) & 0xF, dev = (v >> 20) & 0xF, data = v >> 24;
    if (a == kSwrm + kSwRdCmd) {
      CHECK(dev == 1 && id <= 14);
      gSim.rdFifo = (id << 8) | (reg >= 0x3000 ? gSim.wcd[reg] : gSim.slv[reg]);
      gSim.rdCount = 1;
      return;
    }
    if (dev == 0xF) {
      CHECK(id == 0xF);
      gSim.broadcasts++;
      gSim.ev("bcast %x=%x", reg, data);
      if (reg == slvFrameCtl(0) || reg == slvFrameCtl(1)) gSim.bank = reg == slvFrameCtl(1) ? 1 : 0;
      return;
    }
    CHECK(dev == 1 && id <= 14);
    if (reg >= 0x3000) {
      if (gSim.wcd[reg] != data) gSim.ev("wcd %x=%x", reg, data);
      gSim.wcd[reg] = static_cast<uint8_t>(data);
    } else {
      gSim.slv[reg] = static_cast<uint8_t>(data);
      gSim.ev("slv %x=%x", reg, data);
    }
    return;
  }
  if (gSim.at(a) != v) gSim.ev("mmio %x=%x", static_cast<unsigned>(a), v);
  gSim.at(a) = v;
}
static void simDelay(void *, uint32_t us) { gSim.elapsedUs += us; }

static guampmic::Codec makeCodec() {
  gSim = Sim();
  gSim.wcd[0x3401] = 0x00; gSim.wcd[0x3402] = 0x00; gSim.wcd[0x3403] = 0x0A; gSim.wcd[0x3404] = 0x01;
  gSim.wcd[guampmic::kWEfuse16] = 0x01;
  gSim.at(guampmic::kTx + guampmic::kTxPathCfg0) = 0x00;   // HPF corner 4 Hz after reset
  guampmic::Codec c;
  c.bus = {nullptr, simRd, simWr, simDelay, nullptr};
  return c;
}

static void codec() {
  using namespace guampmic;
  guampmic::Codec c = makeCodec();
  CHECK(c.bringUp(nullptr, nullptr) == 0);
  CHECK(c.swr.devNum == 1 && c.swr.version == 0x01050001 && c.swr.wrDepth == 8 && c.chipId == 0x010A0000);
  CHECK((gSim.at(kVa + kVaMclkCtl) & 1) && (gSim.at(kVa + kVaFsCnt) & 1) && (gSim.at(kVa + kVaTopCfg0) & 2));
  CHECK((gSim.at(kTx + kTxFreqMclk) & 1) && (gSim.at(kTx + kTxMclkCtl) & 1) && (gSim.at(kTx + kTxFsCnt) & 1));
  CHECK((gSim.at(kTx + kTxPathSec7) & 0x3F) == 0x0A && (gSim.at(kVa + kVaSwrCtrl) & 0xF) == 0xA);
  CHECK(gSim.at(kVa + kVaSwrControl) == 1);   // enabled, reset pulse released
  CHECK(gSim.at(kLpi) == 0x104 && gSim.at(kLpi + 0x1000) == 0x106 && gSim.at(kLpi + 0x2000) == 0x106);
  CHECK(gSim.at(kLpiSlew) == ((3u << 0) | (3u << 2) | (3u << 4)));
  CHECK(gSim.at(kSwrm + swFrameCtl(0)) == ((47u << 16) | (1u << 3)));   // 50 x 2, ssp 48
  CHECK(gSim.at(kSwrm + kSwEnumCfg) == 1 && gSim.at(kSwrm + kSwFifoCfg) == 0x80000003u && gSim.at(kSwrm + kSwCompCfg) == 3);
  CHECK(gSim.at(kSwrm + kSwBusCtrl) == 2 && (gSim.at(kSwrm + kSwMcpCfg) & 0x3E0000) == 0x3E0000);
  CHECK(gSim.find("bcast 41=4") != static_cast<size_t>(-1));
  // wcd937x_init_reg (efuse16 = 1: Tanggu part, VBG +5 mV) and micbias voltages
  CHECK((gSim.wcd[kWSleepCtl] & 0xCE) == 0xCE && (gSim.wcd[kWVbgFine] & 0xF0) == 0xB0 && (gSim.wcd[kWAnaBias] & 0xC0) == 0x80);
  CHECK(gSim.wcd[kWMicb1Test1] == 0xFA && gSim.wcd[kWSurgeEn] == 0xD9);
  CHECK((gSim.wcd[kWMicb1] & 0x3F) == 0x10 && (gSim.wcd[kWMicb2] & 0x3F) == 0x20 && (gSim.wcd[kWMicb3] & 0x3F) == 0x10);
  gSim.trace.clear();
  // ---- amic3 capture path
  CHECK(c.pathOn() == 0);
  CHECK((gSim.at(kTx + kTxAdcMux0Cfg1) & 3) == 1 && (gSim.at(kTx + kTxAdcMux0Cfg0) & 0xF) == 5);
  CHECK((gSim.at(kTx + kTxPathCfg0) & 0x80) && (gSim.at(kTx + kTxSwrDmic0) & 0xE) == 0x4);
  CHECK(gSim.at(kTx + kTxPathCtl) == 0x24);   // 48 kHz, decimator on, unmuted
  CHECK((gSim.at(kTx + kTxPathCfg0) & 0x60) == 0 && (gSim.at(kTx + kTxPathSec2) & 3) == 1);   // HPF corner back to 4 Hz
  CHECK((gSim.wcd[kWTxCh2Sel] & 0x80) && (gSim.wcd[kWMicb3] & 0xC0) == 0x40 && (gSim.wcd[kWMicb1] & 0xC0) == 0);
  CHECK(gSim.wcd[kWTxCh2] == (0x80 | 12) && (gSim.wcd[kWTxCh1] & 0x80) && (gSim.wcd[kWTxCh3] & 0x80));
  CHECK((gSim.wcd[kWDigClk] & 0xF0) == 0xF0 && (gSim.wcd[kWAnaClk] & 0x18) == 0x18 && (gSim.wcd[kWReqCtl] & 3) == 2);
  // SoundWire: slave port 1 / master port 2 in both banks, frame 50 x 16 (ssp 6), switch to bank 1 then bank 0 tables
  for (uint32_t b = 0; b < 2; b++) {
    CHECK(gSim.slv[slvDp(1, b, kDpChEn)] == 1 && gSim.slv[slvDp(1, b, kDpSample1)] == 3 && gSim.slv[slvDp(1, b, kDpOffset1)] == 2);
    CHECK(gSim.at(kSwrm + swPortCtl(2, b)) == 0x01000203u && gSim.at(kSwrm + swPortHctl(2, b)) == 0xF0);
  }
  CHECK(gSim.at(kSwrm + swFrameCtl(1)) == ((5u << 16) | (1u << 3) | 7u) && gSim.bank == 1);
  const size_t micb = gSim.find("wcd 3025=50"), adc = gSim.find("wcd 3408=18"), dec = gSim.find("mmio a620400=34");
  const size_t req = gSim.find("wcd 300f=8c"), dp = gSim.find("slv 230=1"), sw = gSim.find("bcast 70=f");
  CHECK(micb != (size_t)-1 && adc != (size_t)-1 && dec != (size_t)-1 && req != (size_t)-1 && dp != (size_t)-1 && sw != (size_t)-1);
  CHECK(micb < adc && adc < dec && dec < req && req < dp && dp < sw);   // DAPM order: micbias, ADC, mux, mixers
  // ---- off
  c.pathOff();
  CHECK((gSim.wcd[kWMicb3] & 0xC0) == 0 && (gSim.wcd[kWTxCh2] & 0x80) == 0 && (gSim.wcd[kWAnaClk] & 0x18) == 0);
  CHECK((gSim.at(kTx + kTxPathCtl) & 0x30) == 0 && gSim.at(kSwrm + swPortCtl(2, 0)) == 0 && gSim.at(kSwrm + swPortCtl(2, 1)) == 0);
  CHECK(gSim.slv[slvDp(1, 0, kDpChEn)] == 0 && gSim.slv[slvDp(1, 1, kDpChEn)] == 0);
  c.bringDown(nullptr, nullptr);
  CHECK((gSim.at(kVa + kVaSwrControl) & 1) == 0 && (gSim.at(kTx + kTxMclkCtl) & 1) == 0 && (gSim.at(kVa + kVaMclkCtl) & 1) == 0);
  CHECK(c.errors == 0 && c.swr.rdFails == 0);
  // ---- amic1: SWR_MIC0, slave port 0 -> master port 1, MIC BIAS1
  guampmic::Codec a = makeCodec();
  a.mic = GUAMP_MIC_AMIC1;
  CHECK(a.bringUp(nullptr, nullptr) == 0 && a.pathOn() == 0);
  CHECK((gSim.at(kTx + kTxAdcMux0Cfg0) & 0xF) == 1 && !(gSim.at(kTx + kTxPathCfg0) & 0x80));
  CHECK((gSim.wcd[kWMicb1] & 0xC0) == 0x40 && (gSim.wcd[kWMicb3] & 0xC0) == 0 && !(gSim.wcd[kWTxCh2Sel] & 0x80));
  CHECK(gSim.at(kSwrm + swPortCtl(1, 1)) == 0x01000103u && gSim.slv[slvDp(0, 1, kDpChEn)] == 1);
  // ---- failures: no slave on the link, unclocked macro
  guampmic::Codec f = makeCodec();
  gSim.attachStatus = 0;
  CHECK(f.bringUp(nullptr, nullptr) == -2 && f.failedAt == 2 && f.swr.devNum == 0);
  printf("codec: bolero TX/VA clocks, LPI SWR pins, SWR master init/enumeration/banks, wcd937x init, amic3 + amic1 "
         "DAPM order, teardown, missing slave (%llu us of waits)\n", (unsigned long long)gSim.elapsedUs);
}

// ---- simulated ADSP for MicPath ---------------------------------------------------------------------------------------
struct Adsp {
  std::vector<std::vector<uint8_t>> sent;
  std::vector<uint32_t> opcodes;
  uint32_t refuse = 0, refuseStatus = 0;   // opcode to refuse (first time)
  bool dropAnswers = false;
  size_t answered = 0;
};
static Adsp gAdsp;
static std::vector<int> gHwCalls;
static std::vector<uint8_t> gSink;
static int adspSend(void *, const uint8_t *p, uint32_t n) {
  gAdsp.sent.emplace_back(p, p + n);
  gAdsp.opcodes.push_back(apr::rd32(p + 16));
  return 0;
}
static int hwStep(void *, int s) { gHwCalls.push_back(s); return 0; }
static void sink(void *, const uint8_t *p, uint32_t n) { gSink.insert(gSink.end(), p, p + n); }
// Answer the last command like the ADSP would
static void answer(apr::MicPath &m, uint32_t now) {
  const std::vector<uint8_t> c = gAdsp.sent.back();
  const uint32_t op = u32(c, 16), token = u32(c, 12);
  if (gAdsp.dropAnswers) return;
  std::vector<uint8_t> r(64);
  uint8_t pl[8] = {};
  uint32_t opRsp = apr::kBasicResult, n = 8;
  apr::wr32(pl, op);
  apr::wr32(pl + 4, op == gAdsp.refuse ? gAdsp.refuseStatus : 0);
  if (op == gAdsp.refuse) gAdsp.refuse = 0;
  if (op == apr::kAfeHwVote) { opRsp = apr::kAfeHwVoteRsp; apr::wr32(pl, 0x77); n = 4; }
  if (op == apr::kAsmMemMap) { opRsp = apr::kAsmMemMapRsp; apr::wr32(pl, 0x55); n = 4; }
  if (op == apr::kAdmOpenV5) { opRsp = apr::kAdmOpenRspV5; apr::wr32(pl, 0); apr::wr32(pl + 4, 9); }
  const uint32_t len = apr::buildPorts(r.data(), 64, c[8], u16(c, 10), u16(c, 6), token, opRsp, pl, n);
  // swap direction: from the ADSP to apps
  r[5] = apr::kDomainAdsp;
  r[9] = apr::kDomainApps;
  m.receive(r.data(), len, now);
}
static void readDone(apr::MicPath &m, uint32_t slotAddr, uint32_t now, int16_t value) {
  for (uint32_t i = 0; i < apr::MicPath::kPeriodBytes; i += 2) memcpy(m.shm.cpu + (slotAddr - m.shm.iovaLsw) + i, &value, 2);
  uint8_t pl[44] = {};
  apr::wr32(pl + 4, slotAddr);
  apr::wr32(pl + 8, 1);
  apr::wr32(pl + 12, 0x55);
  apr::wr32(pl + 16, apr::MicPath::kPeriodBytes);
  std::vector<uint8_t> r(80);
  const uint32_t len = apr::build(r.data(), 80, apr::kSvcAsm, m.port(), 0, apr::kAsmReadDoneV2, pl, sizeof(pl));
  m.receive(r.data(), len, now);
}
static void drive(apr::MicPath &m, uint32_t &now, unsigned maxSteps) {
  for (unsigned i = 0; i < maxSteps && !m.finished(); i++) {
    m.stepAt(now);
    bool progressed = false;
    while (m.waiting && gAdsp.answered < gAdsp.sent.size() && !gAdsp.dropAnswers) {
      gAdsp.answered = gAdsp.sent.size();
      answer(m, now);
      progressed = true;
    }
    if (!progressed && (m.capturing() || m.waiting)) break;
    now += 1;
  }
}

static void path() {
  static uint8_t shmMem[apr::MicPath::kSlots * apr::MicPath::kSlotStride];
  // ---- full chain with shared memory
  gAdsp = Adsp();
  gHwCalls.clear();
  gSink.clear();
  apr::MicPath m;
  m.send = adspSend;
  m.hw = hwStep;
  m.sink = sink;
  m.shm = {shmMem, 0x10000000, 1, sizeof(shmMem)};
  uint32_t now = 0;
  m.begin(now);
  drive(m, now, 100);
  CHECK(m.capturing() && !m.setupFailed && m.inFlight == 4 && m.reads == 4);
  const std::vector<uint32_t> want = {apr::kAfeHwVote, apr::kAfeHwVote, apr::kAfeSvcSetParam, apr::kAfeSvcSetParam,
                                      apr::kAfePortSetParamV2, apr::kAfeDeviceStart, apr::kAsmMemMap, apr::kAdmOpenV5,
                                      apr::kAsmOpenReadV3, apr::kAsmSetEncdec, apr::kAdmMatrixMapV5, apr::kAsmRunV2,
                                      apr::kAsmReadV2, apr::kAsmReadV2, apr::kAsmReadV2, apr::kAsmReadV2};
  CHECK(gAdsp.opcodes == want);
  CHECK(gHwCalls == std::vector<int>({apr::MicPath::CodecUp, apr::MicPath::PathOn}));
  // packets, offsets from the Linux structs (20-byte apr_hdr)
  const auto &vm = gAdsp.sent[0];
  CHECK(u32(vm, 20) == 3 && memcmp(&vm[24], "GMICM", 5) == 0 && (u32(vm, 12) & 0xFFFF0000u) == apr::kMicToken);
  const auto &ck = gAdsp.sent[2];   // afe_svc_cmd_set_param_v1 + afe_clk_set
  CHECK(u16(ck, 2) == 68 && u32(ck, 36) == 0x1028f && u32(ck, 40) == 0x10290 && u32(ck, 52) == 0x30C && u32(ck, 56) == 19200000);
  CHECK(u16(ck, 60) == 1 && u32(ck, 64) == 1 && u32(gAdsp.sent[3], 52) == 0x30D);
  const auto &cd = gAdsp.sent[4];   // afe_port_cmd_set_param_v2 + param_hdr_v1 + afe_param_id_cdc_dma_cfg_t (union 144)
  CHECK(u16(cd, 2) == 20 + 16 + 12 + 144 && u16(cd, 20) == 0xB037 && u16(cd, 22) == 156 && u32(cd, 36) == 0x1020c);
  CHECK(u32(cd, 40) == 0x102B8 && u16(cd, 44) == 144 && u32(cd, 48) == 1 && u32(cd, 52) == 48000 && u16(cd, 56) == 16);
  CHECK(u16(cd, 58) == 0 && u16(cd, 60) == 1 && u16(cd, 62) == 1);
  CHECK(u16(gAdsp.sent[5], 20) == 0xB037);
  const auto &mm = gAdsp.sent[6];
  CHECK(u16(mm, 20) == 3 && u16(mm, 22) == 1 && u32(mm, 28) == 0x10000000 && u32(mm, 32) == 1 && u32(mm, 36) == sizeof(shmMem));
  const auto &ao = gAdsp.sent[7];   // adm_cmd_device_open_v5
  CHECK(u16(ao, 2) == 48 && u16(ao, 6) == 0xB037 && u16(ao, 22) == 2 && u16(ao, 24) == 0xB037 && u16(ao, 26) == 0xFFFF);
  CHECK(u32(ao, 28) == 0x10312 && u16(ao, 32) == 1 && u16(ao, 34) == 16 && u32(ao, 36) == 48000 && ao[40] == 3);
  const auto &orr = gAdsp.sent[8];  // asm_stream_cmd_open_read_v3
  CHECK(u16(orr, 2) == 40 && u16(orr, 6) == 0x0201 && u32(orr, 20) == 0 && u32(orr, 24) == 0 && u32(orr, 28) == 0x10C68);
  CHECK(u32(orr, 32) == 0x10DA5 && u16(orr, 36) == 16);
  const auto &ec = gAdsp.sent[9];   // asm_multi_channel_pcm_enc_cfg_v2
  CHECK(u16(ec, 2) == 56 && u32(ec, 20) == 0x10DA3 && u32(ec, 24) == 28 && u32(ec, 28) == 0 && u32(ec, 32) == 20);
  CHECK(u16(ec, 36) == 1 && u16(ec, 38) == 16 && u32(ec, 40) == 48000 && u16(ec, 44) == 1 && ec[48] == 3);
  const auto &rt = gAdsp.sent[10];  // adm_cmd_matrix_map_routings_v5 TX
  CHECK(u32(rt, 20) == 1 && u32(rt, 24) == 1 && u16(rt, 28) == 2 && u16(rt, 30) == 1 && u16(rt, 32) == 9);
  const auto &rd = gAdsp.sent[12]; // asm_data_cmd_read_v2
  CHECK(u16(rd, 2) == 40 && u16(rd, 6) == 0x0201 && u32(rd, 20) == 0x10000000 && u32(rd, 24) == 1 && u32(rd, 28) == 0x55);
  CHECK(u32(rd, 32) == 960 && u32(gAdsp.sent[13], 20) == 0x10001000);
  // READ_DONE refills the slot and reaches the sink
  readDone(m, 0x10001000, now, 1234);
  CHECK(m.readDones == 1 && m.inFlight == 4 && m.reads == 5 && gSink.size() == 960 && m.peakMax == 1234 && m.rms() == 1234);
  readDone(m, 0x10001000, now, -2000);
  CHECK(m.readDones == 2 && m.peakMax == 2000 && gSink.size() == 1920);
  readDone(m, 0x10005000, now, 1);   // outside the slots: ignored
  CHECK(m.unexpected == 1 && m.readDones == 2);
  // stop: drain the 4 reads, then close in reverse
  m.stop();
  m.stepAt(now);
  gAdsp.opcodes.clear();
  for (uint32_t s = 0; s < 4; s++) readDone(m, 0x10000000 + s * 4096, now, 0);
  CHECK(gAdsp.opcodes.size() == 1 && gAdsp.opcodes[0] == apr::kAsmClose);   // no new reads while stopping
  drive(m, now, 100);
  CHECK(m.finished() && m.undoErrors == 0);
  CHECK(gAdsp.opcodes == std::vector<uint32_t>({apr::kAsmClose, apr::kAdmCloseV5, apr::kAsmMemUnmap, apr::kAfeDeviceStop,
                                                apr::kAfeSvcSetParam, apr::kAfeSvcSetParam, apr::kAfeHwDevote, apr::kAfeHwDevote}));
  CHECK(gHwCalls == std::vector<int>({apr::MicPath::CodecUp, apr::MicPath::PathOn, apr::MicPath::PathOff, apr::MicPath::CodecDown}));
  CHECK(u32(gAdsp.sent[gAdsp.sent.size() - 4], 52) == 0x30D);   // npl off before core off
  // ---- without shared memory: up to the codec path, then waits (verdict "MIC PATH UP")
  gAdsp = Adsp();
  gHwCalls.clear();
  apr::MicPath n;
  n.send = adspSend;
  n.hw = hwStep;
  now = 0;
  n.begin(now);
  drive(n, now, 100);
  CHECK(n.upWithoutMemory() && gAdsp.opcodes.size() == 6 && gHwCalls.size() == 2);
  // ---- cdc dma config refused at 144 bytes: one retry with 16
  gAdsp = Adsp();
  gHwCalls.clear();
  gAdsp.refuse = apr::kAfePortSetParamV2;
  gAdsp.refuseStatus = 2;
  apr::MicPath r;
  r.send = adspSend;
  r.hw = hwStep;
  now = 0;
  r.begin(now);
  drive(r, now, 100);
  CHECK(r.upWithoutMemory() && r.cdcParamBytes == 16 && u16(gAdsp.sent[5], 44) == 16 && u16(gAdsp.sent[5], 2) == 20 + 16 + 12 + 16);
  // ---- port start refused: codec undone, clocks off, devotes; no ASM/ADM traffic
  gAdsp = Adsp();
  gHwCalls.clear();
  gAdsp.refuse = apr::kAfeDeviceStart;
  gAdsp.refuseStatus = 0xd;
  apr::MicPath s;
  s.send = adspSend;
  s.hw = hwStep;
  now = 0;
  s.begin(now);
  drive(s, now, 100);
  CHECK(s.finished() && s.setupFailed && s.failedStep == apr::MicPath::PortStart && s.failedStatus == 0xd);
  CHECK(gHwCalls == std::vector<int>({apr::MicPath::CodecUp, apr::MicPath::CodecDown}));
  CHECK(gAdsp.opcodes.back() == apr::kAfeHwDevote && gAdsp.opcodes[gAdsp.opcodes.size() - 2] == apr::kAfeHwDevote);
  for (uint32_t op : gAdsp.opcodes) CHECK(op != apr::kAsmOpenReadV3 && op != apr::kAdmOpenV5 && op != apr::kAfeDeviceStop);
  // ---- silent ADSP: the first vote times out, the rest still tries; the clock times out -> undo without clock-off
  gAdsp = Adsp();
  gHwCalls.clear();
  gAdsp.dropAnswers = true;
  apr::MicPath t;
  t.send = adspSend;
  t.hw = hwStep;
  now = 0;
  t.begin(now);
  for (unsigned i = 0; i < 20 && !t.finished(); i++) { t.stepAt(now); now += 1001; }
  CHECK(t.finished() && t.setupFailed && t.failedStep == apr::MicPath::ClkCore && gHwCalls.empty());
  CHECK(gAdsp.opcodes == std::vector<uint32_t>({apr::kAfeHwVote, apr::kAfeHwVote, apr::kAfeSvcSetParam}));
  printf("path: votes, TX clocks, codec, CODEC_DMA_TX_3 config/start, mem map, ADM TX, ASM open read, enc cfg, route, run, "
         "4 reads, READ_DONE -> sink, stop in reverse; no shm; config retry; start refused; silent ADSP\n");
}

int main() {
  abi();
  ring();
  codec();
  path();
  printf("mic capture host test: OK\n");
  return 0;
}
