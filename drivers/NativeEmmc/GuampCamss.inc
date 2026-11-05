// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp CAMSS RAW capture (kernel glue of GuampCamssCore.h): GCC clocks + camss_top GDSC, RPM votes, CAMNOC QoS,
// apps SMMU context bank for stream 0x400 (GuampSmmu.inc SmmuContext), frame buffers, CSIPHY/CSID/TFE bring-up,
// TFE/CSID interrupts (armcpu_spi_attach, edge) with a polling pass as fallback, the C API of GuampCamssApi.h and a
// diagnostic test thread. Included by NativeEmmc.cpp after GuampCamera.inc (step 1) and GuampAudioPlay.inc (SMMU).
// Boot arguments (all off by default). Common camera arguments (integration): guamp-camera=2 runs this test after
// the sensor session of GuampCamera.inc, with the sensor guamp-camera-sensor (cell) / guamp-camera-res (mode 0|1),
// or with the TPG when guamp-camera-tpg=1. guamp-cam2 below overrides them.
//   guamp-cam2=0          default: nothing is touched, the API still answers (open powers up on demand)
//   guamp-cam2=1          test thread: TPG -> CSID -> TFE RDI -> N frames in memory, report, everything off
//   guamp-cam2=2          same with the sensor (needs a guamp_camss_sensor_* provider; the weak stand-in fails)
//   guamp-cam2-delay=<s>     start the test later (default 25, 0..600; waits for guamp-camera=1 to finish)
//   guamp-cam2-frames=<n>    frames to capture (default 8, 1..1000)
//   guamp-cam2-bufs=<n>      ring buffers (default 3, 2..4)
//   guamp-cam2-csid=<i>      CSID/TFE instance (default 0); guamp-cam2-rdi=<r> RDI path (default 0)
//   guamp-cam2-clk=0|1       0 = GPLL0 only (cphy_rx/csid 240 MHz, tfe 300); 1 = csid/cphy_rx 384 MHz on GPLL6
//   guamp-cam2-pattern=<p>   TPG pattern (default 9 = split colour bars); guamp-cam2-w/-h (1600 x 1200), -lanes (4)
//   guamp-cam2-cell=<c> guamp-cam2-mode=<m>   sensor cell / mode for =2 (default 0 = S5KGM1ST; mode 0 low, 1 full,
//                            16 + n vendor mode n: GuampCamssSensorBridge.inc)
//   guamp-cam2-irq=0|1       1 (default): attach the CSID/TFE interrupts (polling every 10 ms stays as a safety);
//                            0: polling every 1 ms only
//   guamp-cam2-rpm=0|1       1 (default): vote mmrt/mmnrt (RPM "mmxi" 1/0) at guamp-cam2-khz (default 300000). The
//                            vote is never withdrawn: the display shares mmrt and the apps vote replaces UEFI's
//   guamp-cam2-smmu=0|1      1 (default): own context bank for 0x400; 0: physical addresses (only when the stream is
//                            already in bypass), contiguous buffers below 4 GiB
//   guamp-cam2-timeout=<ms>  capture timeout (default 3000)
//   guamp-cam2-keep=1        leave the stream open after the test (buffers stay for the ISP agent / guamp_camss_*)
//   guamp-cam2-settle=<n>    CSIPHY settle count override (sensor only)
//   guamp-cam2-recover=<ms>  continuous streaming (start(0), the iOS camera service): restart the CSID/TFE path when
//                            no frame completed for <ms> (default 2000, 0 = off, at most 8 restarts per start). Read
//                            even with guamp-cam2=0, it is the API's behaviour
// Every line starts with "GuampCamss:"; the test ends with one "GuampCamss: RESULT ..." line.
#include "GuampCamssCore.h"

extern "C" int armcpu_spi_attach(uint32_t, bool, void (*)(void *), void *, uint32_t *, uint64_t *) __attribute__((weak_import));

#if defined(GUAMP_CAMSS_SENSOR_BRIDGE) && GUAMP_CAMSS_SENSOR_BRIDGE
#include "GuampCamssSensorBridge.inc"   // camera::sensorOpen/sensorStream/sensorClose (GuampCamera.inc sensor session)
#elif !defined(GUAMP_CAMSS_SENSOR_PROVIDED) || !GUAMP_CAMSS_SENSOR_PROVIDED
extern "C" {
__attribute__((weak)) int guamp_camss_sensor_prepare(uint32_t, uint32_t, struct guamp_camss_sensor_link *) {
  return GUAMP_CAMSS_SENSOR_ABSENT;
}
__attribute__((weak)) int guamp_camss_sensor_stream(uint32_t, int) { return GUAMP_CAMSS_SENSOR_ABSENT; }
__attribute__((weak)) void guamp_camss_sensor_release(uint32_t) {}
}
#endif

namespace camss {
unsigned gMode, gDelay = 25, gFrames = 8, gBufs = 3, gCsid, gRdi, gClk, gPattern = 9, gWidth = 1600, gHeight = 1200,
         gLanes = 4, gCell, gSensorMode = 0, gIrq = 1, gRpm = 1, gKhz = 300000, gSmmuOn = 1, gTimeoutMs = 3000, gKeep,
         gSettle, gRecoverMs = 2000;
constexpr unsigned kRecoverMax = 8;
constexpr uint64_t kSmmuPhys = 0x0c600000;
constexpr uint32_t kRpmMmxi = 0x69786d6du;   // "mmxi": QCOM_SMD_RPM_MMXI_CLK (bengal mmnrt 0, mmrt 1)
using CamSmmu = guampwifi::SmmuContext<kSmmuStream, 0, 0x20000000ull, 0x28000000ull>;   // 128 MiB of IOVA

volatile uint8_t *gMap[kBlkCount];
volatile uint8_t *gSmmuRegs;
IOLock *gApi;
IOSimpleLock *gSl;
Stream gS;
CamSmmu gSmmu;
IOBufferMemoryDescriptor *gTables, *gBuf[GUAMP_CAMSS_MAX_BUFFERS];
uint8_t *gTablesCpu;
uint32_t gBufBytes, gMapped[GUAMP_CAMSS_MAX_BUFFERS];
bool gIrqTried[3], gIrqOk[3];
volatile bool gPollRun, gPollAlive;
unsigned gPollMs = 10;

struct Opened {
  bool open = false, streaming = false, sensor = false, gdscWasOn = false;
  guamp_camss_config cfg = {};
  guamp_camss_sensor_link link = {};
  unsigned inst = 0, phy = 0, tpg = 0, lanes = 0;
  uint32_t width = 0, height = 0, lineBytes = 0, frameBytes = 0, bayer = 0;
  uint32_t turnedOn[24] = {};
  unsigned turnedOnCount = 0;
  CsidCfg csid = {};                 // kept for a path restart
  unsigned restarts = 0;             // per start
} gO;

uint32_t ioRd(void *, unsigned b, uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(gMap[b] + off); }
void ioWr(void *, unsigned b, uint32_t off, uint32_t v) {
  *reinterpret_cast<volatile uint32_t *>(gMap[b] + off) = v;
  __asm__ volatile("dsb sy" ::: "memory");
}
void ioDelay(void *, unsigned us) { if (us >= 2000) IOSleep((us + 999) / 1000); else IODelay(us); }
uint64_t ioNow(void *) { uint64_t ns = 0; absolutetime_to_nanoseconds(mach_absolute_time(), &ns); return ns; }
const Io gIo = {nullptr, ioRd, ioWr, ioDelay, ioNow};

uint32_t smRd(void *, uint32_t at) { return *reinterpret_cast<volatile uint32_t *>(gSmmuRegs + at); }
void smWr(void *, uint32_t at, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(gSmmuRegs + at) = v; }
uint32_t tbRd(void *, uint32_t at) { return *reinterpret_cast<volatile uint32_t *>(gTablesCpu + at); }
void tbWr(void *, uint32_t at, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(gTablesCpu + at) = v; }
void smFence(void *) { __asm__ volatile("dsb sy" ::: "memory"); }
void smDelay(void *, uint32_t us) { IODelay(us); }

// --- interrupts and polling ------------------------------------------------------------------------------------------
// Both handlers service CSID and TFE (one 0x5000 window). gS.live is false whenever the clocks may be off: the
// handlers stay attached for the rest of the boot (there is no detach) and must never touch the registers then.
bool service() {
  bool work = false;
  const IOInterruptState st = IOSimpleLockLockDisableInterrupt(gSl);
  if (gS.live) {
    work = serviceTfe(gIo, gS);
    work = serviceCsid(gIo, gS) || work;
  }
  IOSimpleLockUnlockEnableInterrupt(gSl, st);
  return work;
}
void tfeIrq(void *) { ++gS.c.tfe_irqs; service(); }
void csidIrq(void *) { ++gS.c.csid_irqs; service(); }
void attachIrqs(unsigned inst) {
  if (!gIrq || gIrqTried[inst]) return;
  gIrqTried[inst] = true;
  if (!&armcpu_spi_attach) { IOLog("GuampCamss: irq: armcpu_spi_attach missing, polling only\n"); return; }
  uint32_t rbT = 0, rbC = 0;
  uint64_t roT = 0, roC = 0;
  const int t = armcpu_spi_attach(kIntidTfe[inst], true, tfeIrq, nullptr, &rbT, &roT);
  const int c = armcpu_spi_attach(kIntidCsid[inst], true, csidIrq, nullptr, &rbC, &roC);
  gIrqOk[inst] = !t && !c;
  IOLog("GuampCamss: irq: tfe%u INTID %u rc %x (readback %x router %llx), csid%u INTID %u rc %x (readback %x router %llx)\n",
        inst, kIntidTfe[inst], t, rbT, static_cast<unsigned long long>(roT), inst, kIntidCsid[inst], c, rbC,
        static_cast<unsigned long long>(roC));
}
bool restartLocked();
bool stallNow() {
  const IOInterruptState st = IOSimpleLockLockDisableInterrupt(gSl);
  const bool s = stalled(gS, ioNow(nullptr), static_cast<uint64_t>(gRecoverMs) * 1000000u);
  IOSimpleLockUnlockEnableInterrupt(gSl, st);
  return s;
}
void pollThread(void *, wait_result_t) {
  gPollAlive = true;
  while (gPollRun) {
    if (service()) ++gS.c.polls;
    // stall watchdog; try-lock: stop/close hold gApi while they wait for this thread to end
    if (gRecoverMs && gO.restarts < kRecoverMax && stallNow() && IOLockTryLock(gApi)) {
      if (gO.streaming && gPollRun && stallNow()) restartLocked();
      IOLockUnlock(gApi);
    }
    IOSleep(gPollMs);
  }
  gPollAlive = false;
}

// --- power -----------------------------------------------------------------------------------------------------------
bool mapAll(unsigned inst, unsigned phy, unsigned tpg) {
  static volatile uint8_t *phyMap[3], *csidMap[3], *tpgMap[2];
  if (!gMap[kBlkGcc]) gMap[kBlkGcc] = mapPhysical(GT_GCC_BASE, 0x80000);
  if (!gMap[kBlkCpas]) gMap[kBlkCpas] = mapPhysical(kCpasTopPhys, kCpasTopBytes);
  if (!gMap[kBlkCamnoc]) gMap[kBlkCamnoc] = mapPhysical(kCamnocPhys, kCamnocBytes);
  if (!phyMap[phy]) phyMap[phy] = mapPhysical(kCsiphyPhys[phy], kCsiphyBytes);
  if (!csidMap[inst]) csidMap[inst] = mapPhysical(kCsidTfePhys[inst], kCsidTfeBytes);
  if (!tpgMap[tpg]) tpgMap[tpg] = mapPhysical(kTpgPhys[tpg], kTpgBytes);
  if (!gSmmuRegs) gSmmuRegs = mapPhysical(kSmmuPhys, 0x80000);
  gMap[kBlkPhy] = phyMap[phy];
  gMap[kBlkCsid] = csidMap[inst];
  gMap[kBlkTpg] = tpgMap[tpg];
  for (unsigned b = 0; b < kBlkCount; ++b) if (!gMap[b]) return false;
  return gSmmuRegs != nullptr;
}
void rpmVotes(bool sensor) {
  if (!gRpm) return;
  if (!rpm::gLock) { IOLog("GuampCamss: rpm: client not started, no votes (mmrt as left by the boot loader)\n"); return; }
  const int rt = rpm::set(kRpmMmxi, 1, RPM_KEY_KHZ, gKhz);
  const int nrt = rpm::set(kRpmMmxi, 0, RPM_KEY_KHZ, gKhz);
  int l18 = 1;
  if (sensor) {   // mipi-csi-vdd-supply = L18A 1.2 V (csi-vdd-voltage); the UFS ref clock rail too, 1.104-1.312 V
    const rpm_kvp kv[2] = {{RPM_KEY_UV, 4, 1200000}, {RPM_KEY_SWEN, 4, 1}};
    l18 = rpm::request(RPM_SET_ACTIVE, RPM_RES_LDOA, 18, kv, 2);
  }
  IOLog("GuampCamss: rpm: mmrt %u KHz rc %d, mmnrt rc %d, L18A 1.2 V %s rc %d (votes kept)\n", gKhz, rt, nrt,
        sensor ? "on" : "not needed (TPG)", sensor ? l18 : 0);
}
void branch(uint32_t cbcr, bool haltDelay, const char *name, bool &ok) {
  const bool wasOn = gIo.r(kBlkGcc, cbcr) & 1u;
  if (!branchOn(gIo, cbcr, haltDelay)) {
    ok = false;
    IOLog("GuampCamss: clock %s (cbcr %x = %x) did not turn on\n", name, cbcr, gIo.r(kBlkGcc, cbcr));
  }
  if (!wasOn && gO.turnedOnCount < 24) gO.turnedOn[gO.turnedOnCount++] = cbcr;
}
bool clocksUp(unsigned level) {
  gO.gdscWasOn = gIo.r(kBlkGcc, kGdscCamssTop) & (1u << 31);
  const bool gdsc = gdscOn(gIo);
  if (level >= 1) {
    gIo.w(kBlkGcc, kGpllVote, gIo.r(kBlkGcc, kGpllVote) | 1u << 6);
    if (!gIo.poll(kBlkGcc, kGpll6Mode, 1u << 31, 1u << 31, 1000, 2)) {
      IOLog("GuampCamss: clock: GPLL6 not locked (mode %x), level 0 instead\n", gIo.r(kBlkGcc, kGpll6Mode));
      level = 0;
    }
  }
  bool ok = gdsc;
  for (const Branch &b : kCommonBranches) branch(b.cbcr, b.haltDelay, b.name, ok);
  Rcg plan[kMaxRcgs];
  const unsigned n = clockPlan(level, gO.inst, gO.phy, gO.sensor, plan);
  for (unsigned i = 0; i < n; ++i)
    if (!rcgApply(gIo, plan[i])) { ok = false; IOLog("GuampCamss: clock: rcg %s (%x) update timeout\n", plan[i].name, plan[i].rcg); }
  branch(cbcrTfeCphyRx(gO.inst), false, "tfe_cphy_rx", ok);
  branch(cbcrTfeCsid(gO.inst), false, "tfe_csid", ok);
  branch(cbcrTfe(gO.inst), false, "tfe", ok);
  if (gO.sensor) {
    branch(cbcrCphy(gO.phy), false, "cphy", ok);
    branch(cbcrPhyTimer(gO.phy), false, "csiphy_timer", ok);
  } else {
    branch(cbcrCphy(gO.tpg), false, "cphy (tpg)", ok);
    if (gO.tpg != gO.inst) branch(cbcrTfeCphyRx(gO.tpg), false, "tfe_cphy_rx (tpg)", ok);
  }
  IOLog("GuampCamss: power: gdsc camss_top %s (was %s, %x), level %u: cphy_rx %u MHz, csid %u MHz, tfe 300 MHz, axi 300 MHz, "
        "%u clocks turned on here: %s\n", gdsc ? "on" : "OFF", gO.gdscWasOn ? "on" : "off", gIo.r(kBlkGcc, kGdscCamssTop),
        level, level ? 384 : 240, level ? 384 : 240, gO.turnedOnCount, ok ? "ok" : "FAILED");
  return ok;
}
void clocksDown() {
  while (gO.turnedOnCount) branchOff(gIo, gO.turnedOn[--gO.turnedOnCount]);
  if (!gO.gdscWasOn) gIo.w(kBlkGcc, kGdscCamssTop, gIo.r(kBlkGcc, kGdscCamssTop) | 1u);
}

// --- SMMU + buffers ----------------------------------------------------------------------------------------------------
bool smmuUp() {
  if (gSmmu.active) return true;
  if (!gTables)
    gTables = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
        kernel_task, static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous) |
                         static_cast<IOOptionBits>(kIOMapInhibitCache), CamSmmu::TableBytes, 0xFFFFF000ULL);
  gTablesCpu = gTables ? static_cast<uint8_t *>(gTables->getBytesNoCopy()) : nullptr;
  const uint64_t tables = gTables ? gTables->getPhysicalSegment(0, nullptr, kIOMemoryMapperNone) : 0;
  if (!gTablesCpu || !tables) { IOLog("GuampCamss: smmu: table allocation failed\n"); return false; }
  const guampwifi::Memory regs = {nullptr, 0x80000, smRd, smWr, smFence};
  const guampwifi::Memory tbl = {nullptr, CamSmmu::TableBytes, tbRd, tbWr, smFence};
  const int e = gSmmu.start(regs, tbl, tables, guamp_phys_base, guamp_phys_size, true, nullptr, smDelay);
  IOLog("GuampCamss: smmu: context for stream %x: %s (SMR slot %u%s, context bank %u, %u SMRs / %u banks, IOVA %llx..%llx)\n",
        kSmmuStream, guampwifi::errorName(e), gSmmu.slot, gSmmu.preexisting ? " reused" : "", gSmmu.cb, gSmmu.groupsSeen,
        gSmmu.banksSeen, static_cast<unsigned long long>(CamSmmu::IovaBase), static_cast<unsigned long long>(CamSmmu::IovaEnd));
  return e == guampwifi::Ok && gSmmu.active;
}
void smmuFault(const char *when) {
  if (!gSmmu.active) return;
  const uint32_t fsr = gSmmu.faultStatus(), faults = fsr & 0xc00001feu;
  if (!faults) { IOLog("GuampCamss: smmu %s: no fault (FSR %x)\n", when, fsr); return; }
  IOLog("GuampCamss: smmu %s: FAULT FSR %x FAR %x%08x FSYNR0 %x\n", when, fsr, gSmmu.faultAddressHigh(),
        gSmmu.faultAddressLow(), gSmmu.faultSyndrome());
  gSmmu.clearFault();
}
void freeBuffers() {
  for (unsigned i = 0; i < GUAMP_CAMSS_MAX_BUFFERS; ++i) {
    if (gMapped[i] && gSmmu.active) gSmmu.unmap(gS.iova[i], gMapped[i]);
    gMapped[i] = 0;
    if (gBuf[i]) { gBuf[i]->release(); gBuf[i] = nullptr; }
    gS.iova[i] = 0;
  }
}
int buffersUp(unsigned count, uint32_t frameBytes) {
  gBufBytes = (frameBytes + 0xffffu) & ~0xffffu;   // 64 KiB slots in the IOVA window
  if (gSmmuOn && static_cast<uint64_t>(gBufBytes) * count > CamSmmu::IovaEnd - CamSmmu::IovaBase) return GUAMP_CAMSS_ENOMEM;
  const IOOptionBits opt = static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMapWriteCombineCache) |
                           (gSmmuOn ? 0u : static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous));
  const uint64_t mask = gSmmuOn ? 0x0000000FFFFFF000ULL : 0x00000000FFFFF000ULL;
  for (unsigned i = 0; i < count; ++i) {
    gBuf[i] = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, opt, gBufBytes, mask);
    if (!gBuf[i]) { freeBuffers(); return GUAMP_CAMSS_ENOMEM; }
    memset(gBuf[i]->getBytesNoCopy(), 0xa5, gBufBytes);   // poison: untouched words are counted in the report
    if (!gSmmuOn) {
      IOByteCount len = 0;
      const uint64_t pa = gBuf[i]->getPhysicalSegment(0, &len, kIOMemoryMapperNone);
      if (!pa || len < gBufBytes || (pa + gBufBytes) > 0x100000000ull) { freeBuffers(); return GUAMP_CAMSS_ENOMEM; }
      gS.iova[i] = static_cast<uint32_t>(pa);
      continue;
    }
    const uint32_t pages = gBufBytes / 4096;
    uint64_t *pa = static_cast<uint64_t *>(IOMalloc(pages * sizeof(uint64_t)));
    if (!pa) { freeBuffers(); return GUAMP_CAMSS_ENOMEM; }
    bool okPa = true;
    for (uint32_t k = 0; k < pages && okPa; ++k) {
      pa[k] = gBuf[i]->getPhysicalSegment(static_cast<IOByteCount>(k) * 4096, nullptr, kIOMemoryMapperNone);
      okPa = pa[k] && !(pa[k] & 4095);
    }
    const uint64_t iova = CamSmmu::IovaBase + static_cast<uint64_t>(i) * gBufBytes;
    const int e = okPa ? gSmmu.mapPages(iova, pa, pages) : guampwifi::Range;
    IOFree(pa, pages * sizeof(uint64_t));
    if (e != guampwifi::Ok) {
      IOLog("GuampCamss: buffer %u: SMMU map %s\n", i, guampwifi::errorName(e));
      freeBuffers();
      return GUAMP_CAMSS_ESMMU;
    }
    gMapped[i] = gBufBytes;
    gS.iova[i] = static_cast<uint32_t>(iova);
  }
  return GUAMP_CAMSS_OK;
}

void stopPoll() {
  gPollRun = false;
  for (unsigned i = 0; i < 200 && gPollAlive; ++i) IOSleep(5);
}
void streamOff() {   // caller holds gApi
  if (!gO.streaming) return;
  gS.stopping = true;
  if (gO.sensor) guamp_camss_sensor_stream(gO.cfg.cell, 0);
  else tpgStop(gIo, gO.tpg);
  csidHalt(gIo, gS.rdi, kHaltAtFrame);
  IOSleep(60);
  csidHalt(gIo, gS.rdi, kHaltNow);
  tfeStop(gIo, gS.rdi);
  IOSleep(5);
  service();   // last statuses
  const IOInterruptState st = IOSimpleLockLockDisableInterrupt(gSl);
  gS.live = false;
  IOSimpleLockUnlockEnableInterrupt(gSl, st);
  stopPoll();
  gO.streaming = false;
}
// CSID/TFE path restart while the source keeps sending (the sensor stays in streaming, the TPG keeps running): halt,
// reset both blocks, same configuration, first address = a free buffer, resume at the next frame boundary.
bool restartLocked() {   // caller holds gApi, gO.streaming
  const unsigned r = gS.rdi;
  ++gO.restarts;
  Counters c;
  {
    const IOInterruptState st = IOSimpleLockLockDisableInterrupt(gSl);
    gS.stopping = true;
    c = gS.c;
    IOSimpleLockUnlockEnableInterrupt(gSl, st);
  }
  IOLog("GuampCamss: recover %u/%u: no frame for %u ms (done %u sof %u/%u eof %u rup %u, errors rx %u rdi %u tfe %u bus %u; "
        "csid rdi%u status %x, tfe rdi status %x, wm addr status %x)\n", gO.restarts, kRecoverMax, gRecoverMs, c.done, c.sof,
        c.csid_sof, c.eof, c.rup, c.csid_rx_errors, c.csid_rdi_errors, c.tfe_errors, c.bus_errors, r,
        gIo.r(kBlkCsid, rdiStatus(r)), gIo.r(kBlkCsid, tfeRdiStatus(r)), gIo.r(kBlkCsid, wmBase(r) + kWmAddrStatus0));
  smmuFault("before restart");
  csidHalt(gIo, r, kHaltNow);
  tfeStop(gIo, r);
  IOSleep(2);
  {
    const IOInterruptState st = IOSimpleLockLockDisableInterrupt(gSl);
    gS.live = false;   // handlers off while the blocks reset
    IOSimpleLockUnlockEnableInterrupt(gSl, st);
  }
  const int cr = csidReset(gIo);
  const int cc = cr ? cr : csidConfig(gIo, gO.csid);
  const int tr = tfeReset(gIo);
  if (cc || tr) {
    ++gS.c.recovery_failed;
    IOLog("GuampCamss: recover: csid reset %d config %d, tfe reset %d: FAILED, stream stays down\n", cr, cc, tr);
    return false;
  }
  int first;
  {
    const IOInterruptState st = IOSimpleLockLockDisableInterrupt(gSl);
    first = restartBuffer(gS);
    ringRestart(gS, first, ioNow(nullptr));
    gS.live = true;
    ++gS.c.recoveries;
    IOSimpleLockUnlockEnableInterrupt(gSl, st);
  }
  tfeConfig(gIo, r, gS.iova[first], gBufBytes);
  tfeStart(gIo, r, gO.bayer);
  csidStart(gIo, r);
  IOLog("GuampCamss: recover: path restarted, first buffer %u\n", first);
  return true;
}

void closeLocked() {
  if (!gO.open) return;
  streamOff();
  if (gO.sensor) csiphyReset(gIo);
  smmuFault("at close");
  freeBuffers();
  if (gSmmu.active) gSmmu.stop(true);
  clocksDown();
  if (gO.sensor) guamp_camss_sensor_release(gO.cfg.cell);
  gO.open = false;
  IOLog("GuampCamss: closed: clocks off, gdsc %s, SMMU context released, buffers freed\n", gO.gdscWasOn ? "left on (was on)" : "collapse requested");
}
}  // namespace camss

// ---- C API (GuampCamssApi.h) ----------------------------------------------------------------------------------------------
extern "C" int guamp_camss_open(const struct guamp_camss_config *cfg) {
  using namespace camss;
  if (!cfg || cfg->abi != GUAMP_CAMSS_ABI || cfg->csid > 2 || cfg->rdi > 2 || cfg->buffers < 2 ||
      cfg->buffers > GUAMP_CAMSS_MAX_BUFFERS || cfg->source > GUAMP_CAMSS_SRC_SENSOR)
    return GUAMP_CAMSS_EINVAL;
  if (!gApi || !gSl) return GUAMP_CAMSS_ENOTREADY;
  IOLockLock(gApi);
  if (gO.open) { IOLockUnlock(gApi); return GUAMP_CAMSS_EBUSY; }
  gO = Opened{};
  gO.cfg = *cfg;
  gO.sensor = cfg->source == GUAMP_CAMSS_SRC_SENSOR;
  gO.inst = cfg->csid;
  gO.tpg = cfg->csid < 2 ? cfg->csid : 0;
  int rc = GUAMP_CAMSS_OK;
  if (gO.sensor) {
    gO.link.abi = GUAMP_CAMSS_ABI;
    rc = guamp_camss_sensor_prepare(cfg->cell, cfg->mode, &gO.link);
    if (rc) {
      IOLog("GuampCamss: open: sensor cell %u mode %u prepare rc %d%s\n", cfg->cell, cfg->mode, rc,
            rc == GUAMP_CAMSS_SENSOR_ABSENT ? " (no sensor driver in this kernel)" : "");
      IOLockUnlock(gApi);
      return rc;
    }
    if (gO.link.phy > 2 || !gO.link.lanes || gO.link.lanes > 4 || !gO.link.width || !gO.link.height) {
      guamp_camss_sensor_release(cfg->cell);
      IOLockUnlock(gApi);
      return GUAMP_CAMSS_EINVAL;
    }
    gO.phy = gO.link.phy; gO.lanes = gO.link.lanes; gO.width = gO.link.width; gO.height = gO.link.height;
    gO.bayer = gO.link.bayer;
  } else {
    gO.phy = 0;
    gO.lanes = cfg->tpg_lanes ? cfg->tpg_lanes : 4;
    gO.width = cfg->width ? cfg->width : 1600;
    gO.height = cfg->height ? cfg->height : 1200;
    gO.bayer = GUAMP_CAMSS_RGGB;
    if (gO.lanes > 4) gO.lanes = 4;
  }
  gO.width &= ~3u;
  gO.lineBytes = raw10LineBytes(gO.width);
  gO.frameBytes = gO.lineBytes * gO.height;
  if (!gO.width || gO.width > 8192 || gO.height > 8192 || !mapAll(gO.inst, gO.phy, gO.tpg)) {
    if (gO.sensor) guamp_camss_sensor_release(cfg->cell);
    IOLockUnlock(gApi);
    return GUAMP_CAMSS_EINVAL;
  }
  rpmVotes(gO.sensor);
  gO.open = true;   // from here closeLocked() undoes what was done
  if (!clocksUp(cfg->clock_level)) rc = GUAMP_CAMSS_EHW;
  if (!rc) {
    IOLog("GuampCamss: versions: cpas top %x caps %x, csid%u %x, tfe%u %x, bus %x\n", gIo.r(kBlkCpas, kCpasHwVersion),
          gIo.r(kBlkCpas, kCpasHwCaps), gO.inst, gIo.r(kBlkCsid, kCsidHwVersion), gO.inst, gIo.r(kBlkCsid, kTfeHwVersion),
          gIo.r(kBlkCsid, kBusHwVersion));
    camnocInit(gIo);
    if (gSmmuOn && !smmuUp()) rc = GUAMP_CAMSS_ESMMU;
  }
  gS.rdi = cfg->rdi;
  gS.buffers = cfg->buffers;
  if (!rc) rc = buffersUp(cfg->buffers, gO.frameBytes);
  if (!rc && gO.sensor) {
    PhyCfg p = {};
    p.lanes = gO.lanes;
    p.laneMask = gO.link.lane_mask ? gO.link.lane_mask : laneMaskFrom(gO.lanes, gO.link.lane_assign);
    p.settle = gSettle ? gSettle : gO.link.settle_count ? gO.link.settle_count : settleFromRate(gO.link.data_rate_mbps);
    p.settleCombo = p.settle;
    p.combo = gO.link.combo;
    csiphyReset(gIo);
    csiphyConfig(gIo, p);
    IOLog("GuampCamss: csiphy%u: %u lanes, lane mask %x (enable %x), settle %u (formula %u at %u Mbit/s), combo %u\n",
          gO.phy, p.lanes, p.laneMask, laneEnable(p.laneMask), p.settle, settleFromRate(gO.link.data_rate_mbps),
          gO.link.data_rate_mbps, p.combo);
  }
  if (!rc) {
    const int cr = csidReset(gIo);
    CsidCfg c = {gO.lanes, gO.sensor ? gO.link.lane_assign : 0x3210u, gO.sensor ? gO.phy : gO.tpg,
                 gO.sensor ? gO.link.vc : 0u, gO.sensor ? gO.link.dt : 0x2bu, gS.rdi};
    const int cc = cr ? cr : csidConfig(gIo, c);
    gO.csid = c;
    const int tr = tfeReset(gIo);
    if (!cc && !tr) tfeConfig(gIo, gS.rdi, gS.iova[0], gBufBytes);
    IOLog("GuampCamss: csid%u reset %d config %d (phy_sel %u, %u lanes, lane cfg %x, vc %u dt %x -> rdi%u), tfe%u reset %d\n",
          gO.inst, cr, cc, c.phySel, c.lanes, c.laneAssign, c.vc, c.dt, gS.rdi, gO.inst, tr);
    if (cc || tr) rc = GUAMP_CAMSS_EHW;
  }
  if (!rc) attachIrqs(gO.inst);
  IOLog("GuampCamss: open: %s %ux%u RAW10 (%u bytes/line, %u bytes/frame), %u buffers at IOVA %x.. (%s), rc %d\n",
        gO.sensor ? "sensor" : "tpg", gO.width, gO.height, gO.lineBytes, gO.frameBytes, cfg->buffers, gS.iova[0],
        gSmmuOn ? "SMMU 0x400" : "physical", rc);
  if (rc) closeLocked();
  IOLockUnlock(gApi);
  return rc;
}

extern "C" int guamp_camss_start(uint32_t frames) {
  using namespace camss;
  if (!gApi) return GUAMP_CAMSS_ENOTREADY;
  IOLockLock(gApi);
  if (!gO.open || gO.streaming) { IOLockUnlock(gApi); return gO.open ? GUAMP_CAMSS_EBUSY : GUAMP_CAMSS_ENOTREADY; }
  {
    const IOInterruptState st = IOSimpleLockLockDisableInterrupt(gSl);
    const unsigned rdi = gS.rdi, bufs = gS.buffers;
    ringReset(gS);
    gS.rdi = rdi; gS.buffers = bufs; gS.target = frames; gS.held = 0;
    gS.lastProgressNs = ioNow(nullptr);
    gS.live = true;
    IOSimpleLockUnlockEnableInterrupt(gSl, st);
  }
  gO.restarts = 0;
  gPollMs = gIrqOk[gO.inst] ? 10 : 1;
  gPollRun = true;
  thread_t th;
  if (kernel_thread_start(pollThread, nullptr, &th) == KERN_SUCCESS) thread_deallocate(th);
  tfeStart(gIo, gS.rdi, gO.bayer);
  csidStart(gIo, gS.rdi);
  int rc = GUAMP_CAMSS_OK;
  if (gO.sensor) {
    rc = guamp_camss_sensor_stream(gO.cfg.cell, 1);
  } else {
    TpgCfg t = {gO.tpg, gO.lanes, gO.width, gO.height, 0, 0x2b, gO.cfg.tpg_pattern, 2};
    tpgStart(gIo, t);
  }
  gO.streaming = true;
  IOLog("GuampCamss: start: %u frames (0 = until stop), source %s rc %d, polling every %u ms%s\n", frames,
        gO.sensor ? "sensor" : "tpg", rc, gPollMs, gIrqOk[gO.inst] ? " + interrupts" : "");
  if (rc) streamOff();
  IOLockUnlock(gApi);
  return rc;
}

extern "C" int guamp_camss_wait_frame(uint32_t after, uint32_t timeoutMs, struct guamp_camss_frame *out) {
  using namespace camss;
  if (!out || !gSl) return GUAMP_CAMSS_EINVAL;
  for (uint32_t waited = 0;; waited += 1) {
    if (!gO.open) return GUAMP_CAMSS_ENOTREADY;
    const uint32_t seq = __atomic_load_n(&gS.lastSeq, __ATOMIC_ACQUIRE);
    if (seq > after) {
      const IOInterruptState st = IOSimpleLockLockDisableInterrupt(gSl);
      const int b = claimLatest(gIo, gS);   // services pending statuses, may move the next address
      const FrameMeta m = b >= 0 ? gS.meta[b] : FrameMeta{};
      const uint32_t nowSeq = gS.lastSeq;
      IOSimpleLockUnlockEnableInterrupt(gSl, st);
      if (b == kClaimStale) {   // being overwritten: wait for a newer one
        after = nowSeq;
        if (!gO.streaming || waited >= timeoutMs) return GUAMP_CAMSS_ETIMEDOUT;
        IOSleep(1);
        continue;
      }
      if (b < 0) return GUAMP_CAMSS_ENOTREADY;
      *out = guamp_camss_frame{};
      out->abi = GUAMP_CAMSS_ABI; out->seq = m.seq; out->buffer = static_cast<uint32_t>(b);
      out->format = GUAMP_CAMSS_FMT_RAW10_MIPI; out->width = gO.width; out->height = gO.height;
      out->line_bytes = gO.lineBytes; out->bytes = gO.frameBytes; out->bayer = gO.bayer; out->errors = m.errors;
      out->sof_ns = m.sofNs; out->eof_ns = m.eofNs; out->done_ns = m.doneNs; out->iova = gS.iova[b];
      out->cpu = gBuf[b] ? gBuf[b]->getBytesNoCopy() : nullptr;
      return GUAMP_CAMSS_OK;
    }
    if (!gO.streaming || waited >= timeoutMs) return GUAMP_CAMSS_ETIMEDOUT;
    IOSleep(1);
  }
}

extern "C" void guamp_camss_release(uint32_t buffer) {
  using namespace camss;
  if (buffer >= GUAMP_CAMSS_MAX_BUFFERS || !gSl) return;
  const IOInterruptState st = IOSimpleLockLockDisableInterrupt(gSl);
  gS.held = gS.held & ~(1u << buffer);
  IOSimpleLockUnlockEnableInterrupt(gSl, st);
}

extern "C" int guamp_camss_stop(void) {
  using namespace camss;
  if (!gApi) return GUAMP_CAMSS_ENOTREADY;
  IOLockLock(gApi);
  const bool was = gO.streaming;
  streamOff();
  IOLockUnlock(gApi);
  return was ? GUAMP_CAMSS_OK : GUAMP_CAMSS_ENOTREADY;
}

extern "C" void guamp_camss_close(void) {
  using namespace camss;
  if (!gApi) return;
  IOLockLock(gApi);
  closeLocked();
  IOLockUnlock(gApi);
}

extern "C" int guamp_camss_counters(struct guamp_camss_counters *out) {
  using namespace camss;
  if (!out || !gSl) return GUAMP_CAMSS_EINVAL;
  const IOInterruptState st = IOSimpleLockLockDisableInterrupt(gSl);
  *out = gS.c;
  IOSimpleLockUnlockEnableInterrupt(gSl, st);
  out->abi = GUAMP_CAMSS_ABI;
  return GUAMP_CAMSS_OK;
}

extern "C" void *guamp_camss_buffer_memory(uint32_t buffer) {
  return buffer < GUAMP_CAMSS_MAX_BUFFERS ? camss::gBuf[buffer] : nullptr;
}

namespace camss {
// --- diagnostic test ---------------------------------------------------------------------------------------------------
void report(uint32_t frames) {
  Counters c;
  guamp_camss_counters(&c);
  IOLog("GuampCamss: counters: tfe sof %u eof %u rup %u done %u drops %u | csid sof %u eof %u | irqs tfe %u csid %u, polls %u | "
        "unmapped vc/dt %u (last hdr %x, PDAF expected)\n", c.sof, c.eof, c.rup, c.done, c.drops, c.csid_sof, c.csid_eof,
        c.tfe_irqs, c.csid_irqs, c.polls, c.csid_rx_unmapped, c.last_unmapped_hdr);
  IOLog("GuampCamss: claims: redirected %u, stale %u, held overwritten %u | recoveries %u (failed %u)\n",
        c.claim_redirects, c.claim_stale, c.held_overwrites, c.recoveries, c.recovery_failed);
  IOLog("GuampCamss: errors: csi2 rx %u (first %x), rdi %u (first %x), tfe %u (first %x %x %x), bus %u (first %x %x; "
        "ccif %x overflow %x image size %x)\n", c.csid_rx_errors, c.first_rx_error, c.csid_rdi_errors, c.first_rdi_error,
        c.tfe_errors, c.first_tfe_error[0], c.first_tfe_error[1], c.first_tfe_error[2], c.bus_errors, c.first_bus_error[0],
        c.first_bus_error[1], c.bus_violation, c.bus_overflow, c.bus_image_size);
  const unsigned r = gS.rdi;
  IOLog("GuampCamss: csid: rx packets %u, ecc %x, crc errors %u, captured short %x long %x/%x ftr %x, unmapped hdr %x %x, "
        "rdi%u status %x byte count %u\n", gIo.r(kBlkCsid, kCsiRxTotalPkts), gIo.r(kBlkCsid, kCsiRxStatsEcc),
        gIo.r(kBlkCsid, kCsiRxTotalCrc), gIo.r(kBlkCsid, kCsiRxShortPkt0), gIo.r(kBlkCsid, kCsiRxLongPkt0),
        gIo.r(kBlkCsid, kCsiRxLongPkt0 + 4), gIo.r(kBlkCsid, kCsiRxLongPktFtr), gIo.r(kBlkCsid, kCsiRxUnmappedHdr0),
        gIo.r(kBlkCsid, kCsiRxUnmappedHdr0 + 4), r, gIo.r(kBlkCsid, rdiStatus(r)), gIo.r(kBlkCsid, rdiByteCntPing(r)));
  IOLog("GuampCamss: tfe: diag sensor %x %x frame count %x, violation %x, rdi%u status %x, wm addr status %x debug %x, camnoc "
        "errvld %x log0 %x\n", gIo.r(kBlkCsid, kTfeDiagSensor0), gIo.r(kBlkCsid, kTfeDiagSensor1),
        gIo.r(kBlkCsid, kTfeDiagFrameCnt), gIo.r(kBlkCsid, kTfeViolationStatus), r, gIo.r(kBlkCsid, tfeRdiStatus(r)),
        gIo.r(kBlkCsid, wmBase(r) + kWmAddrStatus0), gIo.r(kBlkCsid, wmBase(r) + kWmDebug0),
        gIo.r(kBlkCamnoc, kCamnocErrVld), gIo.r(kBlkCamnoc, kCamnocErrLog0));
  if (gO.sensor) {
    uint32_t ps[kPhyIrqRegs];
    csiphyStatus(gIo, ps);
    IOLog("GuampCamss: csiphy%u irq status %x %x %x %x %x %x %x %x %x %x %x\n", gO.phy, ps[0], ps[1], ps[2], ps[3], ps[4],
          ps[5], ps[6], ps[7], ps[8], ps[9], ps[10]);
  }
  smmuFault("after capture");
  unsigned good = 0;
  for (unsigned b = 0; b < gS.buffers; ++b) {
    const FrameMeta &m = gS.meta[b];
    if (!m.seq || !gBuf[b]) { IOLog("GuampCamss: buffer %u: no frame\n", b); continue; }
    Stats st;
    raw10Stats(static_cast<const uint8_t *>(gBuf[b]->getBytesNoCopy()), gO.width, gO.height, gO.lineBytes, 8, st);
    if (!st.untouchedWords) ++good;
    IOLog("GuampCamss: buffer %u frame %u: fnv %08x, untouched words %u (first at %lld), min %u max %u mean %llu, errors %x\n",
          b, m.seq, st.fnv, st.untouchedWords, static_cast<long long>(st.firstUntouched), st.minV, st.maxV,
          static_cast<unsigned long long>(st.samples ? st.sum / st.samples : 0), m.errors);
    IOLog("GuampCamss: buffer %u hist %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u | bands %u %u %u %u %u %u %u %u\n", b,
          st.hist[0], st.hist[1], st.hist[2], st.hist[3], st.hist[4], st.hist[5], st.hist[6], st.hist[7], st.hist[8],
          st.hist[9], st.hist[10], st.hist[11], st.hist[12], st.hist[13], st.hist[14], st.hist[15], st.band[0], st.band[1],
          st.band[2], st.band[3], st.band[4], st.band[5], st.band[6], st.band[7]);
  }
  const char *verdict = c.done >= frames && good ? "OK" : c.done ? "PARTIAL" : c.sof || c.csid_sof ? "SOF ONLY" :
                        c.csid_rx_errors ? "CSI2 ERRORS ONLY" : "NO FRAMES";
  IOLog("GuampCamss: RESULT %s: %u/%u frames done, %u buffers fully written, sof %u/%u (tfe/csid), errors rx %u rdi %u tfe %u "
        "bus %u, irqs %u/%u\n", verdict, c.done, frames, good, c.sof, c.csid_sof, c.csid_rx_errors, c.csid_rdi_errors,
        c.tfe_errors, c.bus_errors, c.tfe_irqs, c.csid_irqs);
}

void testThread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-camss");
  if (gDelay) IOSleep(gDelay * 1000u);
  // guamp-camera=1|2: the step 1 probe / sensor session thread of GuampCamera.inc uses the same GDSC and PM8008 and
  // switches them off at its end; start well after it (the sensor bridge also retries a busy session for 120 s).
  if (camera::gMode) {
    const unsigned after = camera::gDelay + 60;
    IOLog("GuampCamss: guamp-camera=%u: test starts %u s after boot, after the GuampCamera thread\n", camera::gMode, after);
    if (after > gDelay) IOSleep((after - gDelay) * 1000u);
  }
  guamp_camss_config cfg = {};
  cfg.abi = GUAMP_CAMSS_ABI;
  cfg.source = gMode == 2 ? GUAMP_CAMSS_SRC_SENSOR : GUAMP_CAMSS_SRC_TPG;
  cfg.cell = gCell; cfg.mode = gSensorMode; cfg.buffers = gBufs; cfg.csid = gCsid; cfg.rdi = gRdi; cfg.clock_level = gClk;
  cfg.tpg_pattern = gPattern; cfg.width = gWidth; cfg.height = gHeight; cfg.tpg_lanes = gLanes;
  int rc = guamp_camss_open(&cfg);
  if (rc) { IOLog("GuampCamss: RESULT FAILED: open rc %d\n", rc); return; }
  rc = guamp_camss_start(gFrames);
  uint32_t last = 0;
  const uint64_t t0 = ioNow(nullptr);
  while (!rc) {
    const uint64_t el = (ioNow(nullptr) - t0) / 1000000u;
    if (el >= gTimeoutMs || gS.stopRequest) break;
    guamp_camss_frame f;
    const int w = guamp_camss_wait_frame(last, gTimeoutMs - static_cast<uint32_t>(el), &f);
    if (w) break;
    guamp_camss_release(f.buffer);
    if (f.seq <= 4 || f.seq == gFrames || !(f.seq % 30))
      IOLog("GuampCamss: frame %u buffer %u iova %llx: sof->done %llu us, eof->done %llu us, errors %x\n", f.seq, f.buffer,
            static_cast<unsigned long long>(f.iova), static_cast<unsigned long long>(f.sof_ns && f.done_ns ? (f.done_ns - f.sof_ns) / 1000 : 0),
            static_cast<unsigned long long>(f.eof_ns && f.done_ns ? (f.done_ns - f.eof_ns) / 1000 : 0), f.errors);
    last = f.seq;
  }
  guamp_camss_stop();
  if (rc) IOLog("GuampCamss: start rc %d\n", rc);
  report(gFrames);
  if (!gKeep) guamp_camss_close();
  else IOLog("GuampCamss: kept open (guamp-cam2-keep=1): buffers available through guamp_camss_wait_frame/buffer_memory\n");
}

void start() {
  gApi = IOLockAlloc();
  gSl = IOSimpleLockAlloc();
  unsigned camera = 0, tpg = 0;
  PE_parse_boot_argn("guamp-camera", &camera, sizeof(camera));
  PE_parse_boot_argn("guamp-camera-tpg", &tpg, sizeof(tpg));
  if (camera == 2) {   // common arguments; guamp-cam2-* below still override
    gMode = tpg ? 1 : 2;
    PE_parse_boot_argn("guamp-camera-sensor", &gCell, sizeof(gCell));
    PE_parse_boot_argn("guamp-camera-res", &gSensorMode, sizeof(gSensorMode));
    if (!tpg) gSensorMode = gSensorMode ? 1 : 0;
  }
  PE_parse_boot_argn("guamp-cam2", &gMode, sizeof(gMode));
  PE_parse_boot_argn("guamp-cam2-recover", &gRecoverMs, sizeof(gRecoverMs));
  if (gRecoverMs && gRecoverMs < 200) gRecoverMs = 200;
  if (!gMode) return;
  PE_parse_boot_argn("guamp-cam2-delay", &gDelay, sizeof(gDelay));
  PE_parse_boot_argn("guamp-cam2-frames", &gFrames, sizeof(gFrames));
  PE_parse_boot_argn("guamp-cam2-bufs", &gBufs, sizeof(gBufs));
  PE_parse_boot_argn("guamp-cam2-csid", &gCsid, sizeof(gCsid));
  PE_parse_boot_argn("guamp-cam2-rdi", &gRdi, sizeof(gRdi));
  PE_parse_boot_argn("guamp-cam2-clk", &gClk, sizeof(gClk));
  PE_parse_boot_argn("guamp-cam2-pattern", &gPattern, sizeof(gPattern));
  PE_parse_boot_argn("guamp-cam2-w", &gWidth, sizeof(gWidth));
  PE_parse_boot_argn("guamp-cam2-h", &gHeight, sizeof(gHeight));
  PE_parse_boot_argn("guamp-cam2-lanes", &gLanes, sizeof(gLanes));
  PE_parse_boot_argn("guamp-cam2-cell", &gCell, sizeof(gCell));
  PE_parse_boot_argn("guamp-cam2-mode", &gSensorMode, sizeof(gSensorMode));
  PE_parse_boot_argn("guamp-cam2-irq", &gIrq, sizeof(gIrq));
  PE_parse_boot_argn("guamp-cam2-rpm", &gRpm, sizeof(gRpm));
  PE_parse_boot_argn("guamp-cam2-khz", &gKhz, sizeof(gKhz));
  PE_parse_boot_argn("guamp-cam2-smmu", &gSmmuOn, sizeof(gSmmuOn));
  PE_parse_boot_argn("guamp-cam2-timeout", &gTimeoutMs, sizeof(gTimeoutMs));
  PE_parse_boot_argn("guamp-cam2-keep", &gKeep, sizeof(gKeep));
  PE_parse_boot_argn("guamp-cam2-settle", &gSettle, sizeof(gSettle));
  if (gDelay > 600) gDelay = 600;
  if (!gFrames || gFrames > 1000) gFrames = 8;
  if (gBufs < 2 || gBufs > GUAMP_CAMSS_MAX_BUFFERS) gBufs = 3;
  if (gCsid > 2) gCsid = 0;
  if (gRdi > 2) gRdi = 0;
  if (gLanes < 1 || gLanes > 4) gLanes = 4;
  if (gTimeoutMs < 100 || gTimeoutMs > 60000) gTimeoutMs = 3000;
  if (!gKhz) gKhz = 300000;
  IOLog("GuampCamss: mode %u (%s), delay %u s, %u frames, %u buffers, csid/tfe %u rdi %u, clock level %u, %s, irq %u, rpm %u, "
        "smmu %u, timeout %u ms\n", gMode, gMode == 2 ? "sensor" : "tpg", gDelay, gFrames, gBufs, gCsid, gRdi, gClk,
        gMode == 2 ? "sensor cell/mode from guamp-cam2-cell/-mode" : "tpg pattern/size from guamp-cam2-pattern/-w/-h",
        gIrq, gRpm, gSmmuOn, gTimeoutMs);
  if (!gApi || !gSl) return;
  thread_t th;
  if (kernel_thread_start(testThread, nullptr, &th) == KERN_SUCCESS) thread_deallocate(th);
}
}  // namespace camss
