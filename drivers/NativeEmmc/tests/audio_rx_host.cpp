// Host test of the guamp earpiece/headphone transport: GuampAudioRxSwr.h (RX SoundWire link: LPI rx_swr pins, RX macro
// SWR clock, swrm_master_init on the RX master, enumeration of the wcd937x RX slave 0x0A01170224, data port 1 tables
// of bengal rx_frame_params_default in both banks with the bank switch, link down, failures) against a simulated RX
// master + slave register file, and the /dev/guamp-jack ABI (guamp_jack_dev.h). No device access.
// Build: see run-audio-hw-tests.sh.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <map>
#include <string>
#include <vector>
#include "../GuampAudioRxSwr.h"
#include "../GuampAudioHwCore.h"
#include "../guamp_jack_dev.h"

static int gChecks;
#define CHECK(c) do { ++gChecks; if (!(c)) { fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); exit(1); } } while (0)

struct Sim {
  std::map<uint64_t, uint32_t> mmio;
  std::map<uint16_t, uint8_t> slv;    // RX slave (dev 1) DP/SCP registers
  std::vector<std::string> trace;
  uint32_t version = 0x01060000, rdFifo = 0, rdCount = 0, bank = 0, attach = 1, broadcasts = 0;
  void ev(const char *f, unsigned a, unsigned b) {
    char s[96];
    snprintf(s, sizeof(s), f, a, b);
    trace.push_back(s);
  }
  size_t find(const std::string &s, size_t from = 0) const {
    for (size_t i = from; i < trace.size(); i++) if (trace[i] == s) return i;
    return static_cast<size_t>(-1);
  }
};
static Sim gSim;
static constexpr uint64_t M = guamprx::kRxSwrm;
static uint32_t simRd(void *, uint64_t a) {
  using namespace guampmic;
  if (a == M + kSwHwVersion) return gSim.version;
  if (a == M + kSwCompParams) return (8u << 10) | (8u << 15);
  if (a == M + kSwCompStatus) return 1;                       // link connected (version > 1.5.1 checks it)
  if (a == M + kSwFifoStatus) return gSim.rdCount << 16;
  if (a == M + kSwRdFifo) { gSim.rdCount = 0; return gSim.rdFifo; }
  if (a == M + kSwSlvStatus) return gSim.attach << 2;
  if (a == M + swDevId1(1)) return 0x01170224;                 // the RX slave, not the TX one (...223)
  if (a == M + swDevId2(1)) return 0x0A;
  if (a == M + kSwMcpStatus) return gSim.bank;
  return gSim.mmio[a];
}
static void simWr(void *, uint64_t a, uint32_t v) {
  using namespace guampmic;
  if (a == M + kSwWrCmd || a == M + kSwRdCmd) {
    const uint16_t reg = static_cast<uint16_t>(v & 0xFFFF);
    const uint32_t id = (v >> 16) & 0xF, dev = (v >> 20) & 0xF, data = v >> 24;
    if (a == M + kSwRdCmd) {
      gSim.rdFifo = (id << 8) | gSim.slv[reg];
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
    CHECK(reg < 0x3000);                                       // never a wcd937x register over the RX link
    gSim.slv[reg] = static_cast<uint8_t>(data);
    gSim.ev("slv %x=%x", reg, data);
    return;
  }
  if (gSim.mmio[a] != v) gSim.ev("mmio %x=%x", static_cast<unsigned>(a), v);
  gSim.mmio[a] = v;
}
static std::vector<std::string> gLog;
static void simLog(void *, const char *what, uint32_t a, uint32_t b) {
  char s[160];
  snprintf(s, sizeof(s), "%s %x %x", what, a, b);
  gLog.push_back(s);
}

static void link() {
  using namespace guamprx;
  using namespace guampmic;
  gSim = Sim();
  gSim.mmio[kRxSwrmHctl] = 0xFFFFFFFFu;
  gSim.mmio[kLpi + 0x3000] = 0x1234000;   // bits above the CTL field are kept
  RxLink l;
  l.bus = {nullptr, simRd, simWr, nullptr, simLog};
  CHECK(l.bringUp() == 0 && l.up && l.swr.devNum == 1 && l.swr.base == kRxSwrm);
  // pins: gpio3 clk func1 10 mA bias-disable, gpio4/5 data bus-hold; slew 3 at bits 8, 10, 12 of the slew register
  CHECK(gSim.mmio[kLpi + 0x3000] == (0x1234000u | 0x104) && gSim.mmio[kLpi + 0x4000] == 0x106 && gSim.mmio[kLpi + 0x5000] == 0x106);
  CHECK(gSim.mmio[kLpiSlew] == 0x3F00);
  // RX macro SWR_CONTROL: reset pulse around the clock enable, then the master (hctl gate of v1.6)
  const size_t r1 = gSim.find("mmio a600108=2"), r2 = gSim.find("mmio a600108=3"), r3 = gSim.find("mmio a600108=1");
  CHECK(r1 < r2 && r2 < r3 && gSim.mmio[kRxMacro + kRxMacroSwrControl] == 1);
  CHECK(r3 < gSim.find("mmio a610008=1"));                    // SW reset of the master after the clock
  CHECK(gSim.mmio[kRxSwrmHctl] == 0xFFFFFFFDu);
  CHECK(gSim.mmio[M + swFrameCtl(0)] == ((1u << 3) | (47u << 16)));   // 50 x 2 at init
  CHECK(gSim.broadcasts == 1);                                // SCP_INT_STATUS_MASK_1 only
  // datapath on, headphones (HPH_L + HPH_R): tables in bank 1, switch, tables in bank 0
  CHECK(l.datapath(true, 0x03) == 0 && l.portOn && l.slaveCh == 3);
  for (uint32_t b = 0; b < 2; b++) {
    CHECK(gSim.slv[sDp(0, b, kSChEn)] == 3 && gSim.slv[sDp(0, b, kSSample1)] == 3 && gSim.slv[sDp(0, b, kSOff1)] == 0);
    CHECK(gSim.slv.count(sDp(0, b, kSOff2)) == 1 && gSim.slv[sDp(0, b, kSLane)] == 1);
    CHECK(gSim.slv.count(sDp(0, b, kSHctl)) == 0 && gSim.slv.count(sDp(0, b, kSBlk3)) == 0 && gSim.slv.count(sDp(0, b, kSBlk2)) == 0);
    CHECK(gSim.mmio[M + mPortCtl(1, b)] == 0x03000003u && gSim.mmio[M + mPortCtl2(1, b)] == 1 && gSim.mmio[M + mHctl(1, b)] == 0xF0);
    CHECK(gSim.mmio.count(M + mBlk3(1, b)) == 0 && gSim.mmio.count(M + mBlk2(1, b)) == 0);
  }
  CHECK(gSim.slv[sBlk1(0)] == 1 && gSim.mmio[M + mBlk1(1)] == 1);   // word length 1
  CHECK(sDp(0, 1, kSChEn) == 0x130 && sBlk1(0) == 0x103 && mPortCtl(1, 1) == 0x1164 && mHctl(1, 0) == 0x1134);
  CHECK(gSim.mmio[M + swFrameCtl(1)] == ((1u << 3) | 7u | (5u << 16)));   // 50 x 16, ssp 6
  CHECK(gSim.bank == 1 && gSim.find("bcast 70=f") != static_cast<size_t>(-1));
  CHECK(gSim.find("slv 130=3") < gSim.find("bcast 70=f") && gSim.find("bcast 70=f") < gSim.find("slv 120=3"));
  CHECK(gSim.mmio[M + kSwCpuIntEn] == kSwIntMaskValue);
  // datapath off: channel enables off in the inactive bank (0), back to 50 x 2, switch, then bank 1
  CHECK(l.datapath(false, 0) == 0 && !l.portOn && l.slaveCh == 0);
  CHECK(gSim.slv[0x120] == 0 && gSim.slv[0x130] == 0 && gSim.mmio[M + mPortCtl(1, 0)] == 3 && gSim.mmio[M + mPortCtl(1, 1)] == 3);
  CHECK(gSim.bank == 0 && gSim.mmio[M + swFrameCtl(0)] == ((1u << 3) | (47u << 16)));
  CHECK(gSim.find("slv 120=0") < gSim.find("bcast 60=8") && gSim.find("bcast 60=8") < gSim.find("slv 130=0"));
  // earpiece: HPH_L only
  CHECK(l.datapath(true, 0x01) == 0 && gSim.slv[0x120] == 1 && gSim.slv[0x130] == 1 && gSim.mmio[M + mPortCtl(1, 0)] == 0x01000003u);
  // down with the port still on: datapath off first, clock off, pins to sleep (pull-down input)
  l.bringDown();
  CHECK(!l.up && !l.portOn && !l.clk && !l.pins && l.swr.devNum == 0);
  CHECK(gSim.slv[0x120] == 0 && gSim.slv[0x130] == 0 && gSim.mmio[kRxMacro + kRxMacroSwrControl] == 0);
  CHECK(gSim.mmio[kLpi + 0x3000] == (0x1234000u | 0x105) && gSim.mmio[kLpi + 0x4000] == 0x105 && gSim.mmio[kLpi + 0x5000] == 0x105);
  CHECK(l.datapath(true, 3) == -3 && l.failedAt == 3);         // no link: refused
  printf("rx link: pins, SWR clock pulse, master init v1.6, RX slave dev 1, port 1 {si 3, off 0/0, wd 1, lane 1} in both banks, 50x16 <-> 50x2\n");
}

static void failures() {
  using namespace guamprx;
  gSim = Sim();
  gSim.version = 0;   // RX macro not clocked: the master reads 0
  RxLink a;
  a.bus = {nullptr, simRd, simWr, nullptr, simLog};
  CHECK(a.bringUp() == -1 && a.failedAt == 1 && !a.up);
  a.bringDown();
  CHECK(gSim.mmio[kRxMacro + kRxMacroSwrControl] == 0);
  gSim = Sim();
  gSim.attach = 0;    // wcd937x in reset: no slave on the RX bus
  RxLink b;
  b.bus = {nullptr, simRd, simWr, nullptr, simLog};
  CHECK(b.bringUp(5) == -2 && b.failedAt == 2 && !b.up);
  printf("rx link failures: unclocked master (step 1), absent slave (step 2)\n");
}

static void jackAbi() {
  CHECK(sizeof(guamp_jack_state) == 72);
  CHECK(GUAMP_JACK_IOC_STATE == 0x40486a01u && GUAMP_JACK_IOC_WAIT == 0xc0486a02u && GUAMP_JACK_IOC_REDETECT == 0x20006a03u);
  CHECK(offsetof(guamp_jack_state, seq) == 4 && offsetof(guamp_jack_state, timeout_ms) == 28 &&
        offsetof(guamp_jack_state, host_time) == 32 && offsetof(guamp_jack_state, inserts) == 40);
  CHECK((int)GAH_JACK_HEADPHONE == (int)GUAMP_JACK_HEADPHONE && (int)GAH_JACK_HEADSET == (int)GUAMP_JACK_HEADSET &&
        (int)GAH_JACK_HIGH_HPH == (int)GUAMP_JACK_HIGH_IMPEDANCE && (int)GAH_JACK_UNKNOWN == (int)GUAMP_JACK_UNKNOWN &&
        (int)GAH_JACK_NONE == (int)GUAMP_JACK_NONE);
  CHECK(GUAMP_JACK_ROUTE_SPEAKER == 1 && GUAMP_JACK_ROUTE_HEADPHONE == 3);
  printf("jack abi: state 72 bytes, STATE %08x WAIT %08x REDETECT %08x\n", (unsigned)GUAMP_JACK_IOC_STATE,
         (unsigned)GUAMP_JACK_IOC_WAIT, (unsigned)GUAMP_JACK_IOC_REDETECT);
}

int main() {
  link();
  failures();
  jackAbi();
  printf("audio_rx_host: %d checks, all passed\n", gChecks);
  return 0;
}
