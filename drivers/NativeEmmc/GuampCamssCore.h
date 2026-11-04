// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp CAMSS hardware chain up to RAW frames in memory: GCC clocks/GDSC, CPAS/CAMNOC, CSIPHY 2.0.1 (D-PHY), CSID 530
// (CSI-2 RX -> RDI path), TFE 530 RDI write master (frame based, RAW10 passthrough) and the TPG v1 test source.
// Pure code over an Io table, host-tested by tests/camss_host.cpp; MMIO mapping, interrupts, SMMU, buffers and
// boot arguments belong to GuampCamss.inc.
// Sources (studied, no code copied), Motorola sm6225 kernel:
//   techpack/camera-bengal: cam_csiphy_core.c + cam_csiphy_2_0_hwreg.h (lane tables, lane enable, settle),
//   tfe_csid_hw/cam_tfe_csid_core.c + cam_tfe_csid530.h (global reset, enable_csi2, init/enable/disable RDI, IRQ),
//   tfe_hw/cam_tfe_core.c + cam_tfe_bus.c + cam_tfe530.h (reset, RDI start, WM frame mode, comp group / RUP masks),
//   top_tpg/cam_top_tpg_core.c + cam_top_tpg_v1.h, cam_cpas/cpas_top/cpastop_v520_100.h (CAMNOC TFE QoS);
//   drivers/clk/qcom/gcc-bengal.c (RCG/CBCR offsets, frequency tables, parent maps);
//   arch/arm64/boot/dts/vendor/qcom/camera/bengal-camera.dtsi (bases, IRQs, clock levels, SMMU SID 0x400).
// Structure (one RDI, one write master per RDI, frame based) follows mainline QCM2290 CAMSS (camss-csid-340.c,
// camss-vfe-340.c), the same Spectra 340 / TFE IP. Nothing here is verified on the device yet.
#ifndef GUAMP_CAMSS_CORE_H
#define GUAMP_CAMSS_CORE_H
#include <stdint.h>
#include <stddef.h>
#include "GuampCamssApi.h"

namespace camss {

enum : int { kOk = 0, kErrInval = GUAMP_CAMSS_EINVAL, kErrTimeout = GUAMP_CAMSS_ETIMEDOUT, kErrHw = GUAMP_CAMSS_EHW };

// ---- board facts (bengal-camera.dtsi) ---------------------------------------------------------------------------------
constexpr uint64_t kCpasTopPhys = 0x05c11000, kCamnocPhys = 0x05c13000;
constexpr uint32_t kCpasTopBytes = 0x1000, kCamnocBytes = 0x4000;
constexpr uint64_t kCsiphyPhys[3] = {0x05c52000, 0x05c53000, 0x05c54000};   // 0x1000 each
constexpr uint64_t kCsidTfePhys[3] = {0x05c6e000, 0x05c75000, 0x05c7c000};  // CSID at +0, TFE at +0x1000, 0x5000
constexpr uint64_t kTpgPhys[2] = {0x05c66000, 0x05c68000};                  // 0x400 each
constexpr uint32_t kCsidTfeBytes = 0x5000, kCsiphyBytes = 0x1000, kTpgBytes = 0x400;
// GIC INTID = SPI + 32; all edge rising in the DT
constexpr uint32_t kIntidCsid[3] = {210 + 32, 212 + 32, 77 + 32};
constexpr uint32_t kIntidTfe[3] = {211 + 32, 213 + 32, 78 + 32};
constexpr uint32_t kIntidCsiphy[3] = {72 + 32, 73 + 32, 74 + 32};
constexpr uint32_t kSmmuStream = 0x400;          // msm_cam_smmu_tfe: iommus <&apps_smmu 0x400 0x000>

enum Blk : unsigned { kBlkGcc = 0, kBlkCpas, kBlkCamnoc, kBlkPhy, kBlkCsid, kBlkTpg, kBlkCount };

struct Io {
  void *ctx;
  uint32_t (*rd)(void *, unsigned blk, uint32_t off);
  void (*wr)(void *, unsigned blk, uint32_t off, uint32_t v);
  void (*delayUs)(void *, unsigned us);
  uint64_t (*nowNs)(void *);
  uint32_t r(unsigned b, uint32_t o) const { return rd(ctx, b, o); }
  void w(unsigned b, uint32_t o, uint32_t v) const { wr(ctx, b, o, v); }
  void us(unsigned n) const { delayUs(ctx, n); }
  bool poll(unsigned b, uint32_t o, uint32_t mask, uint32_t want, unsigned tries, unsigned stepUs) const {
    for (unsigned i = 0; i < tries; ++i) {
      if ((r(b, o) & mask) == want) return true;
      us(stepUs);
    }
    return (r(b, o) & mask) == want;
  }
};

// ---- GCC (gcc-bengal.c) -----------------------------------------------------------------------------------------------
constexpr uint32_t kGdscCamssTop = 0x58004;          // bit 0 SW_COLLAPSE, bit 31 PWR_ON
constexpr uint32_t kGpllVote = 0x79000;              // bit 0 GPLL0, bit 6 GPLL6, bit 9 GPLL9, bit 10 GPLL10
constexpr uint32_t kGpll6Mode = 0x6000;              // bit 31 lock (configured by the boot loader, ro postdiv)
constexpr uint32_t kCbcrOff = 1u << 31;
struct Branch { uint32_t cbcr; uint8_t haltDelay; const char *name; };
constexpr Branch kCommonBranches[] = {
  {0x17008, 1, "camera_ahb"}, {0x17028, 0, "camera_xo"}, {0x58028, 0, "camss_top_ahb"}, {0x58044, 0, "camss_axi"},
  {0x5805c, 0, "camss_rt_axi"}, {0x58054, 0, "camss_nrt_axi"}, {0x5804c, 1, "camnoc_atb"}, {0x58050, 1, "camnoc_nts_xo"},
};
constexpr uint32_t cbcrTfeCsid(unsigned i) { return 0x520ac + 0x20 * i; }
constexpr uint32_t cbcrTfeCphyRx(unsigned i) { return 0x5207c + 4 * i; }
constexpr uint32_t cbcrTfe(unsigned i) { return 0x5201c + 0x20 * i; }
constexpr uint32_t cbcrCphy(unsigned p) { return 0x52088 + 4 * p; }
constexpr uint32_t cbcrPhyTimer(unsigned p) { return 0x59018 + 0x1c * p; }
constexpr uint32_t kRcgTopAhb = 0x58010, kRcgAxi = 0x5802c, kRcgCphyRx = 0x52064;
constexpr uint32_t rcgTfeCsid(unsigned i) { return 0x52094 + 0x20 * i; }
constexpr uint32_t rcgTfe(unsigned i) { return 0x52004 + 0x20 * i; }
constexpr uint32_t rcgPhyTimer(unsigned p) { return 0x59000 + 0x1c * p; }
// RCG2: CMD +0 (bit 0 UPDATE, bit 31 ROOT_OFF), CFG +4 (SRC_DIV 4:0 = 2*div-1, SRC_SEL 10:8, MODE 13:12), M/N/D +8/+c/+10
constexpr uint32_t rcgCfg(unsigned sel, unsigned halfDivMinus1) { return (sel & 7u) << 8 | (halfDivMinus1 & 31u); }
struct Rcg { uint32_t rcg; uint32_t cfg; uint32_t hz; const char *name; };
// Level 0: only GPLL0 (always on). Level 1: DT svs_l1 for cphy_rx/csid (GPLL6 early 768 MHz / main 384 MHz).
// TFE: 300 MHz from GPLL0 early / 2 (not in the GCC table, which uses GPLL10 that nobody configures under XNU).
// Phy timer 200 MHz = GPLL0 aux2 300 / 1.5: the settle count below is in units of this clock (techpack / 200 MHz).
inline unsigned clockPlan(unsigned level, unsigned inst, unsigned phy, bool sensor, Rcg *out) {
  unsigned n = 0;
  out[n++] = {kRcgTopAhb, rcgCfg(1, 14), 80000000, "camss_top_ahb_src"};       // GPLL0 early 600 / 7.5
  out[n++] = {kRcgAxi, rcgCfg(2, 1), 300000000, "camss_axi_src"};              // GPLL0 aux2 300 / 1
  if (level >= 1) {
    out[n++] = {kRcgCphyRx, rcgCfg(4, 3), 384000000, "tfe_cphy_rx_src"};       // GPLL6 early 768 / 2
    out[n++] = {rcgTfeCsid(inst), rcgCfg(4, 1), 384000000, "tfe_csid_src"};    // GPLL6 main 384 / 1
  } else {
    out[n++] = {kRcgCphyRx, rcgCfg(1, 4), 240000000, "tfe_cphy_rx_src"};       // GPLL0 early 600 / 2.5
    out[n++] = {rcgTfeCsid(inst), rcgCfg(1, 4), 240000000, "tfe_csid_src"};
  }
  out[n++] = {rcgTfe(inst), rcgCfg(1, 3), 300000000, "tfe_src"};               // GPLL0 early 600 / 2
  if (sensor) out[n++] = {rcgPhyTimer(phy), rcgCfg(2, 2), 200000000, "csiphy_timer_src"};
  return n;
}
constexpr unsigned kMaxRcgs = 6;

inline bool rcgApply(const Io &io, const Rcg &c) {
  io.w(kBlkGcc, c.rcg + 4, c.cfg);
  io.w(kBlkGcc, c.rcg, io.r(kBlkGcc, c.rcg) | 1u);
  return io.poll(kBlkGcc, c.rcg, 1u, 0u, 1000, 1);
}
// true when the branch reports on (halt-delay branches have no reliable status: short wait instead)
inline bool branchOn(const Io &io, uint32_t cbcr, bool haltDelay) {
  io.w(kBlkGcc, cbcr, io.r(kBlkGcc, cbcr) | 1u);
  if (haltDelay) { io.us(10); return true; }
  return io.poll(kBlkGcc, cbcr, kCbcrOff, 0u, 500, 2);
}
inline void branchOff(const Io &io, uint32_t cbcr) { io.w(kBlkGcc, cbcr, io.r(kBlkGcc, cbcr) & ~1u); }
inline bool gdscOn(const Io &io) {
  io.w(kBlkGcc, kGdscCamssTop, io.r(kBlkGcc, kGdscCamssTop) & ~1u);
  return io.poll(kBlkGcc, kGdscCamssTop, 1u << 31, 1u << 31, 1000, 2);
}

// ---- CPAS top / CAMNOC (cpastop_v520_100.h) ---------------------------------------------------------------------------
constexpr uint32_t kCpasHwVersion = 0x0, kCpasHwCaps = 0x4, kCpasTpgMux = 0x1c;
struct Wr32 { uint32_t off, val; };
constexpr Wr32 kCamnocTfeQos[] = {{0x30, 0x44443333}, {0x34, 0x66665555}, {0x38, 0x00001030}, {0x40, 0xffff0000},
                                  {0x48, 0x00000003}};
constexpr uint32_t kCamnocErrMainCtl = 0xd08, kCamnocErrVld = 0xd10, kCamnocErrClr = 0xd18, kCamnocErrLog0 = 0xd20;
inline void camnocInit(const Io &io) {
  for (const Wr32 &q : kCamnocTfeQos) io.w(kBlkCamnoc, q.off, q.val);
  io.w(kBlkCamnoc, kCamnocErrMainCtl, 1);
  io.w(kBlkCamnoc, kCamnocErrClr, 1);
}

// ---- CSIPHY 2.0.1, D-PHY (cam_csiphy_2_0_hwreg.h) ---------------------------------------------------------------------
constexpr uint32_t kPhyLaneEnable = 0x814, kPhyIrqStatus0 = 0x8b0, kPhyIrqClear0 = 0x858, kPhyGlblIrqCmd = 0x828;
constexpr unsigned kPhyIrqRegs = 11;
// lane_mask bit -> register block: bit 0 DL0 0x000, bit 1 CLK 0x700, bit 2 DL1 0x200, bit 3 DL2 0x400, bit 4 DL3 0x600
constexpr uint32_t kPhyLaneBase[5] = {0x000, 0x700, 0x200, 0x400, 0x600};
// physical data lane p -> lane_mask bit
constexpr unsigned phyLaneBit(unsigned p) { return p == 0 ? 0u : p + 1u; }
inline uint32_t laneMaskFrom(unsigned lanes, uint32_t laneAssign) {
  uint32_t m = 1u << 1;   // clock lane
  for (unsigned i = 0; i < lanes && i < 4; ++i) {
    const unsigned p = (laneAssign >> (4 * i)) & 0xfu;
    if (p < 4) m |= 1u << phyLaneBit(p);
  }
  return m;
}
// cam_csiphy_config_dev, 2-phase: DL0 bit 0, CLK 0x80, DL1 bit 2, DL2 bit 4, DL3 bit 6
inline uint32_t laneEnable(uint32_t laneMask) {
  uint32_t v = 0;
  if (laneMask & 1u) v |= 1u;
  if (laneMask & 2u) v |= 0x80u;
  for (unsigned b = 2; b < 5; ++b)
    if (laneMask & (1u << b)) v |= 1u << ((b - 1) * 2);
  return v;
}
// mainline camss-csiphy-3ph-1-0.c formula (for the log and the default): t_hs_settle = 85 ns + 6 UI, UI in ps of the
// data rate, timer period 5 ns at 200 MHz, minus 6 cycles. The sensor blobs say 14 for all guamp sensors.
inline unsigned settleFromRate(uint32_t mbpsPerLane, uint32_t timerHz = 200000000) {
  if (!mbpsPerLane || !timerHz) return 14;
  const uint64_t uiPs = 1000000ull / mbpsPerLane;
  const uint64_t tPs = 85000ull + 6 * uiPs, periodPs = 1000000000000ull / timerHz;
  const uint64_t c = tPs / periodPs;
  return c > 6 ? static_cast<unsigned>(c - 6) : 1u;
}
struct PhyCfg { unsigned lanes; uint32_t laneMask; unsigned settle, settleCombo; bool combo; };
enum : uint8_t { kPhyVal = 0, kPhySettle = 1, kPhySkip = 2 };
// per lane block: 15 registers in the techpack order (offset inside the block, value or kind)
struct PhyLaneReg { uint16_t off; uint8_t val; uint8_t kind; };
inline unsigned phyLanePlan(unsigned pos, bool combo, unsigned settle, PhyLaneReg *o) {
  const bool clk = pos == 1, comboClk = combo && pos == 4;
  const uint8_t k28 = (clk || (combo && (pos == 3 || pos == 4))) ? kPhyVal : kPhySkip;
  const uint8_t v28 = clk ? 0x04 : pos == 3 ? 0x0a : 0x0e;
  const PhyLaneReg l[15] = {{0x30, 0x02, kPhyVal}, {0x2c, 0x01, kPhyVal}, {0x34, 0x01, kPhyVal}, {0x28, v28, k28},
                            {0x3c, 0xb8, kPhyVal}, {0x1c, 0x0a, kPhyVal},
                            {0x00, static_cast<uint8_t>(clk || comboClk ? 0xc0 : 0xd7), kPhyVal}, {0x04, 0x08, kPhyVal},
                            {0x20, 0x00, kPhyVal}, {0x08, static_cast<uint8_t>(settle & 0xff), kPhySettle},
                            {0x0c, 0xff, static_cast<uint8_t>(clk || comboClk ? kPhyVal : kPhySkip)},
                            {0x10, 0x50, kPhyVal}, {0x38, 0x01, kPhyVal}, {0x60, 0x00, kPhyVal}, {0x64, 0x3f, kPhyVal}};
  for (unsigned i = 0; i < 15; ++i) o[i] = l[i];
  return 15;
}
constexpr Wr32 kPhyIrqCfg[kPhyIrqRegs] = {{0x82c, 0xff}, {0x830, 0xff}, {0x834, 0xfb}, {0x838, 0xff}, {0x83c, 0x7f},
                                          {0x840, 0xff}, {0x844, 0xff}, {0x848, 0xef}, {0x84c, 0xff}, {0x850, 0xff},
                                          {0x854, 0xff}};
inline void csiphyReset(const Io &io) {   // csiphy_reset_reg_2_0
  io.w(kBlkPhy, 0x814, 0); io.us(5000);
  io.w(kBlkPhy, 0x818, 0);
  io.w(kBlkPhy, 0x81c, 0);
  io.w(kBlkPhy, 0x800, 1); io.us(1000);
  io.w(kBlkPhy, 0x800, 0);
}
inline void csiphyConfig(const Io &io, const PhyCfg &c) {
  const uint32_t en = laneEnable(c.laneMask);
  // csiphy_common_reg_2_0 (the 0x81c entry is 3-phase only)
  io.w(kBlkPhy, 0x814, en);
  io.w(kBlkPhy, 0x818, 1);
  io.w(kBlkPhy, 0x800, 1); io.us(1000);
  io.w(kBlkPhy, 0x800, 0);
  io.w(kBlkPhy, 0x164, 0); io.w(kBlkPhy, 0x364, 0); io.w(kBlkPhy, 0x564, 0);
  for (unsigned pos = 0; pos < 5; ++pos) {
    if (!(c.laneMask & (1u << pos))) continue;
    PhyLaneReg l[15];
    const unsigned settle = c.combo && pos >= 3 ? c.settleCombo : c.settle;
    const unsigned n = phyLanePlan(pos, c.combo, settle, l);
    for (unsigned i = 0; i < n; ++i)
      if (l[i].kind != kPhySkip) io.w(kBlkPhy, kPhyLaneBase[pos] + l[i].off, l[i].val);
  }
  for (unsigned i = 0; i < kPhyIrqRegs; ++i) {
    io.w(kBlkPhy, kPhyIrqCfg[i].off, kPhyIrqCfg[i].val);
    if (i == 1) io.us(1000);
  }
}
inline void csiphyStatus(const Io &io, uint32_t out[kPhyIrqRegs]) {
  for (unsigned i = 0; i < kPhyIrqRegs; ++i) out[i] = io.r(kBlkPhy, kPhyIrqStatus0 + 4 * i);
}

// ---- CSID 530 (cam_tfe_csid530.h) -------------------------------------------------------------------------------------
constexpr uint32_t kCsidHwVersion = 0x0, kCsidRstStrobes = 0x10;
constexpr uint32_t kCsidTopIrqStatus = 0x70, kCsidTopIrqMask = 0x74, kCsidTopIrqClear = 0x78, kCsidIrqCmd = 0x80;
constexpr uint32_t kCsiRxIrqStatus = 0x20, kCsiRxIrqMask = 0x24, kCsiRxIrqClear = 0x28;
constexpr uint32_t kCsiRxCfg0 = 0x100, kCsiRxCfg1 = 0x104, kCsiRxCaptureCtrl = 0x108;
constexpr uint32_t kCsiRxUnmappedHdr0 = 0x120, kCsiRxShortPkt0 = 0x128, kCsiRxLongPkt0 = 0x130, kCsiRxLongPktFtr = 0x138;
constexpr uint32_t kCsiRxTotalPkts = 0x160, kCsiRxStatsEcc = 0x164, kCsiRxTotalCrc = 0x168;
constexpr uint32_t rdiIrqStatus(unsigned r) { return 0x40 + 0x10 * r; }
constexpr uint32_t rdiIrqMask(unsigned r) { return 0x44 + 0x10 * r; }
constexpr uint32_t rdiIrqClear(unsigned r) { return 0x48 + 0x10 * r; }
constexpr uint32_t rdiCfg0(unsigned r) { return 0x300 + 0x100 * r; }
constexpr uint32_t rdiCfg1(unsigned r) { return 0x304 + 0x100 * r; }
constexpr uint32_t rdiCtrl(unsigned r) { return 0x308 + 0x100 * r; }
constexpr uint32_t rdiStatus(unsigned r) { return 0x350 + 0x100 * r; }
constexpr uint32_t rdiErrRecovery0(unsigned r) { return 0x3b0 + 0x100 * r; }
constexpr uint32_t rdiByteCntPing(unsigned r) { return 0x3e0 + 0x100 * r; }
constexpr uint32_t kCsidRstStb = 0x1e, kCsidRegRstStb = 0x1, kCsiRxIrqAll = 0x0fffffff, kRdiIrqAll = 0x3ffff;
// CSI2 RX interrupt bits (cam_tfe_csid_core.h)
constexpr uint32_t kRxErrMask = (1u << 11) | (1u << 12) | (1u << 13) | (0xfu << 15) | (1u << 19) | (1u << 20) |
                                (1u << 21) | (1u << 23) | (1u << 24);
// UNMAPPED_VC_DT: only counted, not an error. The GM1ST sends PDAF on VC 1 / DT 0x30 in its modes 0 and 1; no path
// maps it, so the CSID drops those packets and raises this bit once per frame.
constexpr uint32_t kRxUnmapped = 1u << 22;
constexpr uint32_t kRxIrqEnable = (1u << 27) | kRxErrMask | kRxUnmapped;
// RDI path bits: 1 RST_DONE, 2 FIFO_OVERFLOW, 9 EOF, 10 EOL, 11 SOL, 12 SOF, 16 FRAME_DROP, 17 OVERFLOW, 18 CCIF
constexpr uint32_t kRdiSof = 1u << 12, kRdiEof = 1u << 9;
constexpr uint32_t kRdiErrMask = (1u << 2) | (1u << 16) | (1u << 17) | (1u << 18);
constexpr uint32_t kRdiIrqEnable = (1u << 1) | kRdiErrMask | kRdiSof | kRdiEof;
enum : uint32_t { kHaltAtFrame = 0, kResumeAtFrame = 1, kHaltNow = 2 };   // cam_tfe_csid_halt_cmd

struct CsidCfg { unsigned lanes; uint32_t laneAssign; unsigned phySel; unsigned vc, dt, rdi; };

// cam_tfe_csid_global_reset: hardware reset (strobes 0x1e), then software registers (strobe 1), both acknowledged by
// top IRQ status bit 0 (latched whatever the mask; polled here).
inline int csidReset(const Io &io) {
  io.w(kBlkCsid, kCsidTopIrqMask, 0);
  io.w(kBlkCsid, kCsiRxIrqMask, 0);
  for (unsigned r = 0; r < 3; ++r) io.w(kBlkCsid, rdiIrqMask(r), 0);
  io.w(kBlkCsid, kCsidTopIrqClear, 1);
  io.w(kBlkCsid, kCsiRxIrqClear, kCsiRxIrqAll);
  for (unsigned r = 0; r < 3; ++r) io.w(kBlkCsid, rdiIrqClear(r), kRdiIrqAll);
  io.w(kBlkCsid, kCsidIrqCmd, 1);
  io.w(kBlkCsid, kCsiRxCfg1, 0x80);
  io.w(kBlkCsid, kCsidRstStrobes, kCsidRstStb);
  const bool hw = io.poll(kBlkCsid, kCsidTopIrqStatus, 1u, 1u, 1000, 10);
  io.w(kBlkCsid, kCsidTopIrqClear, 1);
  io.w(kBlkCsid, kCsidIrqCmd, 1);
  io.w(kBlkCsid, kCsidRstStrobes, kCsidRegRstStb);
  const bool sw = io.poll(kBlkCsid, kCsidTopIrqStatus, 1u, 1u, 1000, 10);
  io.w(kBlkCsid, kCsidTopIrqClear, 1);
  io.w(kBlkCsid, kCsidIrqCmd, 1);
  io.us(3000);
  return hw && sw ? kOk : kErrTimeout;
}
// enable_csi2 + init_config_rdi_path + reset_retain_sw_reg
inline int csidConfig(const Io &io, const CsidCfg &c) {
  const unsigned lanes = c.lanes ? c.lanes : 1;
  io.w(kBlkCsid, kCsiRxCfg0, (lanes - 1) | (c.laneAssign & 0xffffu) << 4 | 0u << 24 | (c.phySel & 3u) << 20);
  io.w(kBlkCsid, kCsiRxCfg1, 1u << 6 | 1u);                       // MISR + ECC correction
  io.w(kBlkCsid, kCsiRxIrqMask, kRxIrqEnable);
  const unsigned r = c.rdi;
  // vc 22, dt 16, cid 27 (0), decode format 0xf = MIPI passthrough (RAW10 in, RAW10 out), plain 0, timestamps, enable
  const uint32_t cfg0 = (c.vc & 0x1fu) << 22 | (c.dt & 0x3fu) << 16 | 0xfu << 12 | 1u << 2 | 1u;
  io.w(kBlkCsid, rdiCfg0(r), cfg0);
  io.w(kBlkCsid, rdiCfg1(r), 2);                                  // TFE_CSID_TIMESTAMP_STB_POST_IRQ
  io.w(kBlkCsid, rdiErrRecovery0(r), 0x9);                        // error detection, overflow detect
  io.w(kBlkCsid, rdiCtrl(r), kHaltAtFrame);
  io.w(kBlkCsid, rdiCfg0(r), io.r(kBlkCsid, rdiCfg0(r)) | 1u << 31);
  // packet capture of the first short (frame start) and long packet header of vc/dt: read back in the report
  io.w(kBlkCsid, kCsiRxCaptureCtrl, 1u << 1 | (c.vc & 3u) << 12 | 1u | (c.dt & 0x3fu) << 4 | (c.vc & 3u) << 10);
  // reset_retain_sw_reg
  io.w(kBlkCsid, kCsidTopIrqMask, 0);
  io.w(kBlkCsid, kCsidTopIrqClear, 1);
  io.w(kBlkCsid, kCsidIrqCmd, 1);
  io.w(kBlkCsid, kCsidRstStrobes, kCsidRstStb);
  const bool ok = io.poll(kBlkCsid, kCsidTopIrqStatus, 1u, 1u, 1000, 10);
  io.w(kBlkCsid, kCsidTopIrqClear, 1);
  io.w(kBlkCsid, kCsidIrqCmd, 1);
  return ok ? kOk : kErrTimeout;
}
inline void csidStart(const Io &io, unsigned r) {   // enable_rdi_path (SOF/EOF on: they are the frame counters)
  io.w(kBlkCsid, rdiCtrl(r), kResumeAtFrame);
  io.w(kBlkCsid, rdiIrqMask(r), kRdiIrqEnable);
}
inline void csidHalt(const Io &io, unsigned r, uint32_t cmd) {
  io.w(kBlkCsid, rdiIrqMask(r), 0);
  io.w(kBlkCsid, rdiCtrl(r), (io.r(kBlkCsid, rdiCtrl(r)) & ~3u) | (cmd & 3u));
}

// ---- TFE 530 (cam_tfe530.h); offsets from the shared CSID/TFE window ------------------------------------------------------
constexpr uint32_t kTfeHwVersion = 0x1000, kTfeGlobalReset = 0x1014, kTfeRegUpdate = 0x102c, kTfeTopIrqCmd = 0x1030;
constexpr uint32_t tfeTopMask(unsigned n) { return 0x1034 + 4 * n; }
constexpr uint32_t tfeTopClear(unsigned n) { return 0x1040 + 4 * n; }
constexpr uint32_t tfeTopStatus(unsigned n) { return 0x104c + 4 * n; }
constexpr uint32_t kTfeDiagConfig = 0x1060, kTfeDiagSensor0 = 0x1064, kTfeDiagSensor1 = 0x1068, kTfeDiagFrameCnt = 0x106c;
constexpr uint32_t kTfeViolationStatus = 0x1070;
constexpr uint32_t tfeRdiStatus(unsigned r) { return 0x1404 + 0x200 * r; }
constexpr uint32_t tfeRdiModule(unsigned r) { return 0x1460 + 0x200 * r; }
constexpr uint32_t tfeRdiEpoch(unsigned r) { return 0x1470 + 0x200 * r; }
constexpr uint32_t kBusHwVersion = 0x1a00;
constexpr uint32_t busMask(unsigned n) { return 0x1a18 + 4 * n; }
constexpr uint32_t busClear(unsigned n) { return 0x1a20 + 4 * n; }
constexpr uint32_t busStatus(unsigned n) { return 0x1a28 + 4 * n; }
constexpr uint32_t kBusIrqCmd = 0x1a30, kBusOverflowClear = 0x1a60, kBusCcifViolation = 0x1a64, kBusOverflow = 0x1a68,
                   kBusImageSize = 0x1a70;
constexpr uint32_t wmBase(unsigned rdi) { return 0x1c00 + 0x100 * (7 + rdi); }   // bus clients 7/8/9 = RDI0/1/2
constexpr uint32_t kWmCfg = 0x00, kWmImageAddr = 0x04, kWmFrameIncr = 0x08, kWmImageCfg0 = 0x0c, kWmImageCfg1 = 0x10,
                   kWmImageCfg2 = 0x14, kWmPacker = 0x18, kWmBwLimit = 0x1c, kWmSubPeriod = 0x30, kWmSubPattern = 0x34,
                   kWmDropPeriod = 0x38, kWmDropPattern = 0x3c, kWmAddrStatus0 = 0x68, kWmDebug0 = 0x7c;
// interrupt bits
constexpr uint32_t kTop0Reset = 1u << 0, kTop0Bus = 1u << 1, kTop0ErrMask = 0x000f0f00, kTop2ErrMask = 0x3f;
constexpr uint32_t tfeSof(unsigned r) { return 0x10u << (4 * r); }   // top status 1
constexpr uint32_t tfeEof(unsigned r) { return 0x20u << (4 * r); }
constexpr uint32_t busRup(unsigned r) { return 1u << (1 + r); }       // RUP group 1/2/3
constexpr uint32_t busDone(unsigned r) { return 1u << (8 + 5 + r); }  // comp group 5/6/7, comp_done_shift 8
constexpr uint32_t kBusErrMask = 0xd0000000;
constexpr uint32_t tfeRegUpdateRdi(unsigned r) { return 0x2u << r; }

inline int tfeReset(const Io &io) {   // cam_tfe_reset, HW and registers
  for (unsigned n = 0; n < 3; ++n) io.w(kBlkCsid, tfeTopClear(n), io.r(kBlkCsid, tfeTopStatus(n)));
  io.w(kBlkCsid, kTfeTopIrqCmd, 1);
  for (unsigned n = 0; n < 3; ++n) io.w(kBlkCsid, tfeTopMask(n), 0);
  io.w(kBlkCsid, tfeTopMask(0), kTop0Reset);
  io.w(kBlkCsid, kTfeGlobalReset, 1);
  const bool ok = io.poll(kBlkCsid, tfeTopStatus(0), kTop0Reset, kTop0Reset, 1000, 10);
  io.w(kBlkCsid, tfeTopClear(0), kTop0Reset);
  io.w(kBlkCsid, kTfeTopIrqCmd, 1);
  io.w(kBlkCsid, tfeTopMask(0), 0);
  return ok ? kOk : kErrTimeout;
}
// bus init_hw + acquire_rdi_wm(frame based) + start_wm/comp group/tfe_out + HFR (no drop, no subsample) + first address
inline void tfeConfig(const Io &io, unsigned r, uint32_t iova, uint32_t frameIncr) {
  const uint32_t w = wmBase(r);
  io.w(kBlkCsid, busMask(0), kBusErrMask);
  io.w(kBlkCsid, busMask(1), 0);
  io.w(kBlkCsid, w + kWmBwLimit, 0xf);
  io.w(kBlkCsid, w + kWmImageCfg0, 0u << 16 | 0xffffu);          // frame based: height 0, width 0xffff
  io.w(kBlkCsid, w + kWmPacker, 0xa);
  io.w(kBlkCsid, w + kWmImageCfg2, 0xffff);                      // stride default (frame based)
  io.w(kBlkCsid, w + kWmImageCfg1, 0);                           // offset
  io.w(kBlkCsid, w + kWmDropPeriod, 0);
  io.w(kBlkCsid, w + kWmDropPattern, 1);
  io.w(kBlkCsid, w + kWmSubPeriod, 0);
  io.w(kBlkCsid, w + kWmSubPattern, 1);
  io.w(kBlkCsid, w + kWmImageAddr, iova);
  io.w(kBlkCsid, w + kWmFrameIncr, frameIncr);
  io.w(kBlkCsid, w + kWmCfg, 1u << 16 | 1u);
  io.w(kBlkCsid, busMask(0), io.r(kBlkCsid, busMask(0)) | busDone(r) | busRup(r));
}
// cam_tfe_top_start, RDI: module config (pixel pattern, out enable), epoch, reg update, subscribe + error masks
inline void tfeStart(const Io &io, unsigned r, unsigned bayer) {
  io.w(kBlkCsid, kTfeDiagConfig, io.r(kBlkCsid, kTfeDiagConfig) | 1u);
  io.w(kBlkCsid, tfeRdiModule(r), (bayer & 7u) << 24 | 1u);
  io.w(kBlkCsid, tfeRdiEpoch(r), 0x00140014);
  io.w(kBlkCsid, kTfeRegUpdate, tfeRegUpdateRdi(r));
  io.w(kBlkCsid, tfeTopMask(0), io.r(kBlkCsid, tfeTopMask(0)) | kTop0Bus | kTop0ErrMask);
  io.w(kBlkCsid, tfeTopMask(1), io.r(kBlkCsid, tfeTopMask(1)) | tfeSof(r) | tfeEof(r));
  io.w(kBlkCsid, tfeTopMask(2), io.r(kBlkCsid, tfeTopMask(2)) | kTop2ErrMask);
}
inline void tfeStop(const Io &io, unsigned r) {
  io.w(kBlkCsid, wmBase(r) + kWmCfg, 0);
  io.w(kBlkCsid, tfeRdiModule(r), 0);
  io.w(kBlkCsid, kTfeRegUpdate, tfeRegUpdateRdi(r));
  for (unsigned n = 0; n < 3; ++n) io.w(kBlkCsid, tfeTopMask(n), 0);
  io.w(kBlkCsid, busMask(0), 0);
  io.w(kBlkCsid, busMask(1), 0);
}

// ---- TPG v1 (cam_top_tpg_v1.h, cam_top_tpg_core.c start/stop) ---------------------------------------------------------
struct TpgCfg { unsigned index, lanes, width, height, vc, dt, pattern, encode /* RAW10 = 2 */; };
inline void tpgStart(const Io &io, const TpgCfg &c) {
  io.w(kBlkTpg, 0x6c, 0x12345678);                                 // LFSR seed
  io.w(kBlkTpg, 0x70, (c.width & 0xffffu) << 16 | (c.height & 0x3fffu));
  io.w(kBlkTpg, 0x80, c.dt);
  io.w(kBlkTpg, 0x90, (c.encode & 0xfu) << 16 | 0x8u);
  io.w(kBlkTpg, 0x64, 0u << 8 | (c.vc & 0xfu));                    // one DT (tpg_num_dt_map[0] = 0), vc
  io.w(kBlkTpg, 0x68, 0x2581f4);                                   // h blank 500, v blank 600
  uint32_t cfg2;
  if (c.pattern <= 7) {
    cfg2 = (c.encode & 0xfu) << 16 | c.pattern;
  } else if (c.pattern == 8) {
    io.w(kBlkTpg, 0xa0, 1u | 1u << 2);                             // unicolour bar
    cfg2 = (c.encode & 0xfu) << 16 | 8u;
  } else {
    io.w(kBlkTpg, 0xa0, 1u << 5);                                  // split colour bars
    cfg2 = (c.encode & 0xfu) << 16 | 8u;                           // CAM_TOP_TPG_DEFAULT_PATTERN
  }
  io.w(kBlkTpg, 0x90, cfg2);
  io.w(kBlkTpg, 0xa4, 0);                                          // common gen cfg: pixel pattern 0
  io.w(kBlkTpg, 0xa8, 0xafff);                                     // VBI
  io.w(kBlkCpas, kCpasTpgMux, io.r(kBlkCpas, kCpasTpgMux) | 1u << c.index);
  const unsigned lanes = c.lanes ? c.lanes : 1;
  // active lanes 4, FE 2, FS 1, phy_sel 3 = lane type (0 D-PHY), enable 0
  io.w(kBlkTpg, 0x60, (lanes - 1) << 4 | 1u << 2 | 1u << 1 | 0u << 3 | 1u);
}
inline void tpgStop(const Io &io, unsigned index) {
  io.w(kBlkTpg, 0x60, 0);
  io.w(kBlkCpas, kCpasTpgMux, io.r(kBlkCpas, kCpasTpgMux) & ~(1u << index));
}

// ---- stream state, interrupt service, buffer ring ---------------------------------------------------------------------
enum : uint32_t { kFeRx = 1, kFeRdi = 2, kFeTfe = 4, kFeBus = 8, kFeDrop = 16 };
typedef struct guamp_camss_counters Counters;   // the function guamp_camss_counters hides the bare type name
struct FrameMeta { uint32_t seq; uint32_t errors; uint64_t sofNs, eofNs, doneNs; };
struct Stream {
  unsigned rdi = 0, buffers = 0;
  uint32_t iova[GUAMP_CAMSS_MAX_BUFFERS] = {};
  uint32_t target = 0;                 // frames, 0 = until stop
  volatile uint32_t held = 0;          // consumer-held buffers (never programmed while held)
  int latched = -1, pending = -1, writing = -1;
  bool live = false, stopping = false;
  volatile bool stopRequest = false;
  uint64_t curSof = 0, curEof = 0;
  uint32_t curErrors = 0;
  FrameMeta meta[GUAMP_CAMSS_MAX_BUFFERS] = {};
  volatile uint32_t lastSeq = 0;
  volatile int lastBuf = -1;
  Counters c = {};
};
// next buffer after `from` that the consumer does not hold; `from` itself (overwrite) when all others are held
inline int pickNext(const Stream &s, int from) {
  for (unsigned k = 1; k <= s.buffers; ++k) {
    const int b = static_cast<int>((static_cast<unsigned>(from < 0 ? -1 : from) + k) % s.buffers);
    if (b != from && !(s.held & (1u << b))) return b;
  }
  return from < 0 ? 0 : from;
}
inline void program(const Io &io, Stream &s, int buf) {
  io.w(kBlkCsid, wmBase(s.rdi) + kWmImageAddr, s.iova[buf]);
  io.w(kBlkCsid, kTfeRegUpdate, tfeRegUpdateRdi(s.rdi));
  s.pending = buf;
}
// before tfeStart: the first buffer is part of tfeConfig; this only sets the ring state
inline void ringReset(Stream &s) {
  s.latched = -1; s.writing = -1; s.pending = 0; s.lastSeq = 0; s.lastBuf = -1; s.curErrors = 0;
  s.curSof = s.curEof = 0; s.stopRequest = false; s.stopping = false;
  for (auto &m : s.meta) m = FrameMeta{};
  s.c = Counters{};
  s.c.abi = GUAMP_CAMSS_ABI;
}
inline void noteErr(uint32_t &count, uint32_t &first, uint32_t bits) { if (!count++) first = bits; }

// TFE interrupt: returns true when any status bit was set. Order inside one call: EOF and buffer done (frame k),
// then RUP done (address of frame k+1 latched at its SOF) and the next programming, then SOF (frame k+1 starts).
inline bool serviceTfe(const Io &io, Stream &s) {
  uint32_t top[3], bus[2] = {0, 0};
  for (unsigned n = 0; n < 3; ++n) top[n] = io.r(kBlkCsid, tfeTopStatus(n));
  if (!(top[0] | top[1] | top[2])) return false;
  for (unsigned n = 0; n < 3; ++n) io.w(kBlkCsid, tfeTopClear(n), top[n]);
  io.w(kBlkCsid, kTfeTopIrqCmd, 1);
  if (top[0] & kTop0Bus) {
    for (unsigned n = 0; n < 2; ++n) bus[n] = io.r(kBlkCsid, busStatus(n));
    const uint32_t viol = io.r(kBlkCsid, kBusCcifViolation), ovf = io.r(kBlkCsid, kBusOverflow),
                   img = io.r(kBlkCsid, kBusImageSize);
    for (unsigned n = 0; n < 2; ++n) io.w(kBlkCsid, busClear(n), bus[n]);
    io.w(kBlkCsid, kBusIrqCmd, 1);
    if ((bus[0] & kBusErrMask) || bus[1]) {
      if (!s.c.bus_errors++) { s.c.first_bus_error[0] = bus[0]; s.c.first_bus_error[1] = bus[1]; }
      s.c.bus_violation |= viol; s.c.bus_overflow |= ovf; s.c.bus_image_size |= img;
      if (ovf) io.w(kBlkCsid, kBusOverflowClear, 1);
      s.curErrors |= kFeBus;
    }
  }
  const unsigned r = s.rdi;
  if ((top[0] & kTop0ErrMask) || (top[2] & kTop2ErrMask)) {
    if (!s.c.tfe_errors++) { s.c.first_tfe_error[0] = top[0]; s.c.first_tfe_error[1] = top[1]; s.c.first_tfe_error[2] = top[2]; }
    s.curErrors |= kFeTfe;
  }
  const uint64_t now = io.nowNs ? io.nowNs(io.ctx) : 0;
  if (top[1] & tfeEof(r)) { ++s.c.eof; s.curEof = now; }
  if (bus[0] & busDone(r)) {
    ++s.c.done;
    const int b = s.writing >= 0 ? s.writing : (s.latched >= 0 ? s.latched : 0);
    FrameMeta &m = s.meta[b];
    m.seq = s.c.done; m.errors = s.curErrors; m.sofNs = s.curSof; m.eofNs = s.curEof; m.doneNs = now;
    s.lastBuf = b;
    __atomic_store_n(&s.lastSeq, s.c.done, __ATOMIC_RELEASE);
    s.curErrors = 0; s.curEof = 0;
    if (s.target && s.c.done >= s.target) s.stopRequest = true;
  }
  if (bus[0] & busRup(r)) {
    ++s.c.rup;
    if (s.pending >= 0) s.latched = s.pending;
    s.pending = -1;
    if (s.live && !s.stopping && !s.stopRequest && s.buffers > 1) {
      const int next = pickNext(s, s.latched);
      if (next == s.latched) { ++s.c.drops; s.curErrors |= kFeDrop; }
      program(io, s, next);
    }
  }
  if (top[1] & tfeSof(r)) {
    ++s.c.sof;
    s.writing = s.latched >= 0 ? s.latched : 0;
    s.curSof = now;
  }
  return true;
}
inline bool serviceCsid(const Io &io, Stream &s) {
  const uint32_t top = io.r(kBlkCsid, kCsidTopIrqStatus), rx = io.r(kBlkCsid, kCsiRxIrqStatus);
  uint32_t rdi[3];
  for (unsigned n = 0; n < 3; ++n) rdi[n] = io.r(kBlkCsid, rdiIrqStatus(n));
  if (!(top | rx | rdi[0] | rdi[1] | rdi[2])) return false;
  io.w(kBlkCsid, kCsidTopIrqClear, top);
  io.w(kBlkCsid, kCsiRxIrqClear, rx);
  for (unsigned n = 0; n < 3; ++n) io.w(kBlkCsid, rdiIrqClear(n), rdi[n]);
  io.w(kBlkCsid, kCsidIrqCmd, 1);
  const uint32_t mine = rdi[s.rdi];
  if (mine & kRdiSof) ++s.c.csid_sof;
  if (mine & kRdiEof) ++s.c.csid_eof;
  if (rx & kRxUnmapped) { ++s.c.csid_rx_unmapped; s.c.last_unmapped_hdr = io.r(kBlkCsid, kCsiRxUnmappedHdr0); }
  if (rx & kRxErrMask) { noteErr(s.c.csid_rx_errors, s.c.first_rx_error, rx); s.curErrors |= kFeRx; }
  if (mine & kRdiErrMask) { noteErr(s.c.csid_rdi_errors, s.c.first_rdi_error, mine); s.curErrors |= kFeRdi; }
  return true;
}

// ---- RAW10 frame statistics (diagnostic report, host-checkable) ---------------------------------------------------------
constexpr uint64_t kPoison64 = 0xa5a5a5a5a5a5a5a5ull;
struct Stats {
  uint32_t fnv = 2166136261u;          // FNV-1a 32 over all bytes
  uint32_t untouchedWords = 0;         // 64-bit words still equal to the poison: never written by the WM
  int64_t firstUntouched = -1;         // byte offset
  uint32_t hist[16] = {};              // 10-bit values >> 6, sampled
  uint32_t minV = 1023, maxV = 0, samples = 0;
  uint64_t sum = 0;
  uint32_t band[8] = {};               // mean of the sampled pixels of 8 vertical bands (colour bars)
  uint32_t bandN[8] = {};
};
// p: lineBytes * height bytes (frame-based: lines back to back); every rowStep-th line, every 5-byte group sampled
inline void raw10Stats(const uint8_t *p, uint32_t width, uint32_t height, uint32_t lineBytes, unsigned rowStep, Stats &st) {
  const uint64_t bytes = static_cast<uint64_t>(lineBytes) * height;
  for (uint64_t i = 0; i + 8 <= bytes; i += 8) {
    uint64_t w;
    __builtin_memcpy(&w, p + i, 8);
    if (w == kPoison64) { if (st.firstUntouched < 0) st.firstUntouched = static_cast<int64_t>(i); ++st.untouchedWords; }
    for (unsigned k = 0; k < 8; ++k) st.fnv = (st.fnv ^ static_cast<uint8_t>(w >> (8 * k))) * 16777619u;
  }
  for (uint64_t i = bytes & ~7ull; i < bytes; ++i) st.fnv = (st.fnv ^ p[i]) * 16777619u;
  if (!rowStep) rowStep = 1;
  uint64_t bandSum[8] = {};
  for (uint32_t y = 0; y < height; y += rowStep) {
    const uint8_t *l = p + static_cast<uint64_t>(y) * lineBytes;
    for (uint32_t g = 0; g * 4 + 3 < width && g * 5 + 4 < lineBytes; ++g) {
      const uint8_t *q = l + g * 5;
      for (unsigned k = 0; k < 4; ++k) {
        const uint32_t v = static_cast<uint32_t>(q[k]) << 2 | ((q[4] >> (2 * k)) & 3u);
        ++st.hist[v >> 6];
        if (v < st.minV) st.minV = v;
        if (v > st.maxV) st.maxV = v;
        st.sum += v; ++st.samples;
        const unsigned x = g * 4 + k, b = static_cast<unsigned>(static_cast<uint64_t>(x) * 8 / width);
        if (b < 8) { bandSum[b] += v; ++st.bandN[b]; }
      }
    }
  }
  for (unsigned b = 0; b < 8; ++b) st.band[b] = st.bandN[b] ? static_cast<uint32_t>(bandSum[b] / st.bandN[b]) : 0;
}
constexpr uint32_t raw10LineBytes(uint32_t width) { return width * 10 / 8; }

}  // namespace camss
#endif
