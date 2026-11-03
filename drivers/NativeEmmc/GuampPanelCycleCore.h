// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp panel power cycle without kernel dependencies (host-tested by tests/backlight_host.cpp), guamp-touch-panelcycle=5.
// The HX83102D is a TDDI: its touch block lives on the display's rails and state. In TWRP (log-xnu/twrp-dmesg-20261103.txt)
// Linux power-cycles the panel before the touch probe: dsi_panel_disable 2.470, post_unprepare 2.571, power_on 2.586,
// dsi_panel_reset 2.650, dsi_panel_enable 2.717; touch sense on 4.8 s. Here the panel was only ever powered by the boot
// loader. This file replays that cycle from XNU with only the LCDB (lab/ibb) rails; L9A (vddio) is never touched (owner's
// decision 03/11: it is always-on in the dtbo anyway, qcom,vdd-io-always-on).
//
// Sources (android_kernel_motorola_sm6225 and the device dtbo, analisi/dtb/dtbo_a-0.dts):
//  (1) sde_encoder_phys_vid_disable: INTF_TIMING_ENGINE_EN (sde_hw_intf.c 0x000) = 0, then one vblank wait; then
//      dsi_display_disable -> dsi_display_vid_engine_disable: DSI_CTRL bit 1 (VIDEO_MODE_EN) cleared
//      (dsi_ctrl_hw_cmn_video_engine_en), command engine bit 2 on (dsi_display_cmd_engine_enable).
//  (2) dsi_panel_disable: qcom,mdss-dsi-off-command = 05 .. 14 .. 28 (20 ms), 05 .. 50 .. 10 (80 ms). The dtbo sends them
//      in HS; here in LP (the link carries no video any more, LP needs no HS clock request).
//  (3) dsi_panel_power_off: the reset line is pulled low only with qcom,mdss-dsi-reset-force-pull-low, which this dtbo
//      does NOT set (pinctrl "panel_suspend" only lowers the drive and adds a pull-down). Opts::resetLow (default on, as
//      asked) drives gpio82 low before the rails go; resetLow = 0 keeps it high as Linux does.
//  (4) dsi_pwr_enable_vregs(false), reverse order: ibb pre-off-sleep 2 ms (dtbo), then the first LCDB regulator disable
//      reaches qpnp_lcdb_disable (lab and ibb are the LDO and NCP of the one PMI632 LCDB module, sid 3 base 0xEC00,
//      boot dtb /soc/qcom,spmi@1c40000/qcom,pmi632@3/qpnp-lcdb@ec00). PMI632 = FORCE_PD_ENABLE_WA
//      (qpnp_lcdb_pmic_config): LDO_FORCE_PD_CTL 0x79 = 0x81, ENABLE_CTL1 0x46 = 0, 10 ms, 0x79 = 0.
//  (5) dsi_panel_power_on -> dsi_pwr_enable_vregs(true): vddio post-on-sleep 20 ms (only the wait: L9A untouched), lab:
//      regulator_set_voltage(5.6 V, the dtbo supply-min-voltage "\0Us" = 0x00557300) then qpnp_lcdb_enable with
//      qcom,voltage-step-ramp (boot dtb): LDO/NCP 4.5 V, ENABLE_CTL1 = MODULE_EN 0x80, poll INT_RT_STS 0x10 bit 0
//      (VREG_OK), 10 ms, then +500 mV steps 1 ms apart up to the target; ibb post-on-sleep 20 ms. The boost follows
//      qpnp_lcdb_set_bst_voltage: max(LDO, NCP) + 150 mV headroom, 25 mV steps above 4.7 V (BST_OUTPUT_VOLTAGE 0x41).
//      Then dsi_panel_reset: qcom,mdss-dsi-reset-sequence <1 5>, <0 2>, <1 60> on gpio82.
//  (6) dsi_panel_enable: qcom,mdss-dsi-on-command in LP (on-command-state dsi_lp_mode): 11, 29 (dtbo waits 10 / 50 ms;
//      160 / 50 here, as the owner's sequence asks: a longer wait after sleep out is always allowed), 51 <level> (the
//      dtbo sends 00 00, the last level keeps the screen lit), 55 01, 53 2C, B9 83 10 2D, CD 01 (10 ms).
//  (7) dsi_display_vid_engine_enable then sde_encoder_phys_vid_enable / kickoff: DSI_CTRL back as the boot loader left it
//      (video engine on), INTF_TIMING_ENGINE_EN = 1. The MDSS pipe, the framebuffer (0x5C000000, stride 2880, 720x1600),
//      the DSI PHY and PLL are never touched, so nothing has to be reprogrammed: the boot loader's video setup resumes.
//      Checked with the INTF frame counter AND DSI_STATUS bit 3 (video engine busy) 50 ms later; if not, timing engine
//      off, DSI video off, controller soft reset, video on, timing engine on, checked again (fin43: dark screen with the
//      counter moving). Linux also soft-resets the controller before dsi_panel_prepare (dsi_display_enable without
//      continuous splash): done here before the reset sequence. The panel power mode (DCS read 0x0A in LP with BTA, the
//      dtbo status check, expected 9c) is read with the video stopped before the off-command and after the on-command.
// Every write is logged with the value read back; the cycle refuses before its first write when anything is not as
// expected (clocks, controller, timing engine, LCDB state, SPMI write permission, reset pin), and after the first write
// every path ends with the rails up, the reset released, the on-command sent and the video running again.
#ifndef GUAMP_PANEL_CYCLE_CORE_H
#define GUAMP_PANEL_CYCLE_CORE_H
#include <stdint.h>
#include "GuampBacklightCore.h"

namespace guamp_panelcycle {
// sde_hw_intf.c
constexpr uint64_t kIntf1Phys = 0x05e6b800ULL;   // bengal-sde.dtsi mdp_phys 0x5e00000 + qcom,sde-intf-off 0x6b800 (INTF_1, DSI)
enum : uint32_t { rIntfTimingEn = 0x000, rIntfFrameLineEn = 0x0a8, rIntfFrameCount = 0x0ac, rIntfLineCount = 0x0b0, rIntfStatus = 0x26c };

// qpnp-lcdb-regulator.c (PMI632: sid 3, base 0xEC00)
constexpr unsigned kLcdbSid = 3;
constexpr uint16_t kLcdbBase = 0xec00;
enum : uint16_t {
  rSts1 = 0x08, rIntRtSts = 0x10, rBstVout = 0x41, rModuleRdy = 0x45, rEnableCtl1 = 0x46, rLdoVout = 0x71, rLdoForcePd = 0x79,
  rNcpVout = 0x81,
};
constexpr uint8_t kVregOk = 1u << 0, kScError = 1u << 1;                 // INT_RT_STATUS
constexpr uint8_t kModuleEn = 1u << 7, kHwenRdy = 1u << 6;               // ENABLE_CTL1
constexpr uint8_t kForcePdEn = 1u << 0, kForcePdMode = 1u << 7;          // LDO_FORCE_PD_CTL
constexpr uint8_t kVoutMask = 0x1f, kBstMask = 0x3f;                     // SET_OUTPUT_VOLTAGE_MASK, BST_OUTPUT_VOLTAGE_MASK
constexpr uint32_t kMinMv = 4000, kMaxMv = 6000, kRampStartMv = 4500, kRampStepMv = 500, kBstHeadroomMv = 150;
constexpr uint32_t kBstMinMv = 4700, kBstMaxMv = 6275;
constexpr uint32_t kLinuxPanelMv = 5600;                                 // dtbo lab/ibb qcom,supply-min-voltage

// qpnp_lcdb_set_voltage / qpnp_lcdb_get_voltage (LDO and NCP: 100 mV steps from 4.0 V, 50 mV steps from 4.95 V = code 10)
constexpr uint8_t voutCode(uint32_t mv) {
  return mv < 4950 ? (uint8_t)((mv - 4000 + 99) / 100) : (uint8_t)((mv - 4950 + 49) / 50 + 10);
}
constexpr uint32_t voutMv(uint8_t code) {
  return (code & kVoutMask) < 10 ? 4000u + 100u * (code & kVoutMask) : 4950u + 50u * ((code & kVoutMask) - 10u);
}
// qpnp_lcdb_set_bst_voltage: max(LDO, NCP) + headroom, clamped to 4.7..6.275 V, 25 mV steps (rounded up)
constexpr uint8_t bstCode(uint32_t outMv) {
  return (uint8_t)(((outMv + kBstHeadroomMv < kBstMinMv ? kBstMinMv : outMv + kBstHeadroomMv > kBstMaxMv ? kBstMaxMv
                                                                                                       : outMv + kBstHeadroomMv) -
                    kBstMinMv + 24) / 25);
}
constexpr uint32_t bstMv(uint8_t code) { return kBstMinMv + 25u * (code & kBstMask); }

struct Options {
  bool lcdb = true;        // rails down / up (guamp-touch-pc-lcdb=0: DSI off / on and the reset only)
  bool resetLow = true;    // LCD reset low while the rails are off (Linux with this dtbo: high)
  uint32_t targetMv = kLinuxPanelMv;   // LDO / NCP after the ramp (0 = the boot loader's value)
  uint32_t offMs = 10;     // rails off time after the force pull-down is released
  uint32_t level = guamp_backlight::kDefaultLevel;   // 51 parameter in the on-command
  bool softReset = true;   // DSI controller soft reset before the panel reset (dsi_display_enable, not continuous splash)
  bool hsOn = false;       // on-command in HS instead of LP (guamp-touch-pc-hs=1)
  bool readStatus = true;  // DCS read 0x0A (power mode) in LP with BTA, before and after (dtbo status check)
};
enum Result : int {
  kDone = 0, kNoClocks = 1, kDsiNotReady = 2, kIntfOff = 3, kLcdbRead = 4, kLcdbOff = 5, kSpmiReadOnly = 6, kResetPin = 7,
  kBadVoltage = 8,
  // after the first write: the cycle ran to the end but something is worth a look
  kWarnOffCmd = 16, kWarnLcdbDown = 32, kWarnLcdbUp = 64, kWarnOnCmd = 128, kWarnVideo = 256, kWarnFrozen = 512,
  kWarnPanelMode = 1024, kWarnDsiIdle = 2048,
};
struct Report {
  int result = 0;
  uint32_t dsiCtrl = 0, intfEn = 0, frames[6] = {}, offFails = 0, onFails = 0, pollsUp = 0;
  uint8_t enBefore = 0, rtBefore = 0, ldoBefore = 0, ncpBefore = 0, bstBefore = 0, pdBefore = 0;
  uint8_t enDown = 0, rtDown = 0, enUp = 0, rtUp = 0, ldoUp = 0, ncpUp = 0, bstUp = 0;
  bool railsDown = false;
  int modeBeforeRc = -1, modeAfterRc = -1;
  uint8_t modeBefore = 0, modeAfter = 0;
  uint32_t dsiStatusAfter = 0, laneAfter = 0;
};
// DSI_STATUS bit 3 VIDEO_MODE_ENGINE_BUSY: set while the video engine carries the INTF stream (boot loader: 0x2008)
constexpr uint32_t kStatusVideoBusy = 1u << 3;
// dsi_ctrl_reg.h DSI_SOFT_RESET 0x118; dsi_ctrl_hw_cmn.c DSI_CTRL_DYNAMIC_FORCE_ON = 0x23F | BIT(8) | BIT(9) | BIT(11) | BIT(21)
constexpr uint32_t rSoftReset = 0x118, kClkForceOn = 0x23fu | 1u << 8 | 1u << 9 | 1u << 11 | 1u << 21;

// Io: the backlight Io (rd, wr, clocksOn, delayUs, sleepMs, nowUs on the DSI window) plus
//   uint32_t intfRd(uint32_t), void intfWr(uint32_t, uint32_t)        INTF_1 window
//   int spmiRd(uint16_t off, uint8_t *), int spmiWr(uint16_t off, uint8_t), bool spmiWritable()   LCDB, offsets from 0xEC00
//   bool lcdResetIsOutput(), unsigned lcdResetLevel(), void lcdReset(bool), void tpReset(bool)
//   void log(const char *fmt, ...) (printf format; every line starts with "GuampPanel: ")
template <class Io> struct Cycle {
  Io &io;
  const Options &o;
  Report &r;
  int writeLcdb(uint16_t off, uint8_t v, uint8_t *back) {
    const int e = io.spmiWr(off, v);
    uint8_t rb = 0xff;
    const int e2 = io.spmiRd(off, &rb);
    if (back) *back = rb;
    io.log("GuampPanel: lcdb %04x <- %02x: write %d, reads %02x (%d)\n", (unsigned)(kLcdbBase + off), v, e, rb, e2);
    return e;
  }
  int maskedLcdb(uint16_t off, uint8_t mask, uint8_t v) {   // regmap_update_bits
    uint8_t cur = 0;
    int e = io.spmiRd(off, &cur);
    if (e) { io.log("GuampPanel: lcdb %04x read failed (%d)\n", (unsigned)(kLcdbBase + off), e); return e; }
    const uint8_t next = (uint8_t)((cur & ~mask) | (v & mask));
    if (next == cur) return 0;
    return writeLcdb(off, next, nullptr);
  }
  void readLcdb(uint8_t &en, uint8_t &rt) {
    en = rt = 0xff;
    io.spmiRd(rEnableCtl1, &en);
    io.spmiRd(rIntRtSts, &rt);
  }
  uint32_t frames() { return io.intfRd(rIntfFrameCount); }
  void dsiState(const char *when) {
    io.log("GuampPanel: dsi %s: ctrl %08x status %08x fifo %08x ack %08x timeout %08x dln0 %08x lane %08x/%08x clk %08x/%08x "
           "video %08x dma %08x trig %08x sched %08x int %08x | intf1 en %x status %08x frames %u line %u\n",
           when, io.rd(guamp_backlight::rCtrl), io.rd(guamp_backlight::rStatus), io.rd(guamp_backlight::rFifoStatus),
           io.rd(guamp_backlight::rAckErr), io.rd(guamp_backlight::rTimeoutStatus), io.rd(guamp_backlight::rDln0PhyErr),
           io.rd(guamp_backlight::rLaneStatus), io.rd(guamp_backlight::rLaneCtrl), io.rd(guamp_backlight::rClkStatus),
           io.rd(guamp_backlight::rClkCtrl), io.rd(guamp_backlight::rVideoModeCtrl), io.rd(guamp_backlight::rDmaCtrl),
           io.rd(guamp_backlight::rTrigCtrl), io.rd(guamp_backlight::rDmaSchedule), io.rd(guamp_backlight::rIntCtrl),
           io.intfRd(rIntfTimingEn), io.intfRd(rIntfStatus), io.intfRd(rIntfFrameCount), io.intfRd(rIntfLineCount));
  }
  // DCS list in LP, command engine, video stopped. Returns the number of refused commands.
  struct Cmd { uint8_t b[4]; uint8_t n; uint16_t waitMs; };
  uint32_t sendList(const char *what, const Cmd *c, unsigned count, bool lp = true) {
    uint32_t fails = 0;
    for (unsigned i = 0; i < count; ++i) {
      guamp_backlight::Diag d{};
      const int e = guamp_backlight::sendDcsCmd(io, c[i].b, c[i].n, lp, d);
      if (e != guamp_backlight::kSent) fails = fails + 1;
      io.log("GuampPanel: %s %02x (%u bytes, %s): %s, %u us, status %08x ack %08x timeout %08x fifo %08x\n", what, c[i].b[0],
             (unsigned)c[i].n, lp ? "LP" : "HS", guamp_backlight::resultName(e), d.waitedUs, d.status, d.ack, d.timeouts, d.fifo);
      if (c[i].waitMs) io.sleepMs(c[i].waitMs);
    }
    return fails;
  }
  // 0x0A get_power_mode: bit 7 booster, 4 sleep out, 3 normal mode, 2 display on (0x9c / 0x9d expected when on)
  int powerMode(const char *when, uint8_t &mode) {
    guamp_backlight::Diag d{};
    const int e = guamp_backlight::readDcsCmd(io, 0x0a, true, &mode, d);
    io.log("GuampPanel: power mode %s: %s %02x (bta %u us, rdbk ctrl %08x data %08x type %08x, ack %08x timeout %08x)%s\n",
           when, guamp_backlight::resultName(e), mode, d.btaUs, d.rdCtrl, d.rd0, d.rdType, d.ack, d.timeouts,
           e ? "" : (mode & 0x14) == 0x14 ? ": sleep out, display on" : (mode & 0x10) ? ": sleep out, display off" : ": in sleep");
    return e;
  }
  // dsi_ctrl_hw_cmn_soft_reset: engines off, clocks forced on, SOFT_RESET pulse, clocks and DSI_CTRL back
  void softReset(const char *when) {
    using namespace guamp_backlight;
    const uint32_t ctrl = io.rd(rCtrl), clk = io.rd(rClkCtrl);
    io.wr(rCtrl, ctrl & ~7u);
    io.wr(rClkCtrl, clk | kClkForceOn);
    io.wr(rSoftReset, 1);
    io.delayUs(1);
    io.wr(rSoftReset, 0);
    io.wr(rClkCtrl, clk);
    io.wr(rCtrl, ctrl);
    io.log("GuampPanel: DSI soft reset (%s): ctrl %08x clk %08x status %08x\n", when, ctrl, clk, io.rd(guamp_backlight::rStatus));
  }
  // video really back: INTF frame counter moving and the DSI video engine busy with it
  bool videoBack(uint32_t &a, uint32_t &b) {
    a = frames();
    io.sleepMs(50);
    b = frames();
    r.dsiStatusAfter = io.rd(guamp_backlight::rStatus);
    r.laneAfter = io.rd(guamp_backlight::rLaneStatus);
    io.log("GuampPanel: video check: frames %u -> %u, DSI status %08x (video engine %s), lane status %08x\n", a, b, r.dsiStatusAfter,
           (r.dsiStatusAfter & kStatusVideoBusy) ? "busy" : "IDLE", r.laneAfter);
    return b != a && (r.dsiStatusAfter & kStatusVideoBusy);
  }
  bool pollVregOk(uint8_t &rt) {
    for (unsigned i = 0; i < 20; ++i) {     // Linux: 10 polls x (soft-start + debounce); here 1 ms x 20
      rt = 0xff;
      if (!io.spmiRd(rIntRtSts, &rt) && (rt & kVregOk)) { r.pollsUp = i + 1; return true; }
      io.sleepMs(1);
    }
    r.pollsUp = 20;
    return false;
  }
  // qpnp_lcdb_enable with voltage-step-ramp; returns true when VREG_OK came
  bool lcdbUp(uint32_t target) {
    const uint8_t want = bstCode(target);
    if ((r.bstBefore & kBstMask) != want) maskedLcdb(rBstVout, kBstMask, want);
    maskedLcdb(rLdoVout, kVoutMask, voutCode(kRampStartMv));
    maskedLcdb(rNcpVout, kVoutMask, voutCode(kRampStartMv));
    writeLcdb(rEnableCtl1, kModuleEn, &r.enUp);
    uint8_t rt = 0;
    bool ok = pollVregOk(rt);
    if (!ok) {
      uint8_t sts[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
      for (unsigned i = 0; i < 6; ++i) io.spmiRd((uint16_t)(rSts1 + i), &sts[i]);
      io.log("GuampPanel: lcdb VREG_OK not seen (rt %02x) STS1..6 %02x %02x %02x %02x %02x %02x: module off and on again\n", rt,
             sts[0], sts[1], sts[2], sts[3], sts[4], sts[5]);
      writeLcdb(rEnableCtl1, 0, nullptr);
      io.sleepMs(10);
      writeLcdb(rEnableCtl1, kModuleEn, &r.enUp);
      ok = pollVregOk(rt);
    }
    io.log("GuampPanel: lcdb module on: VREG_OK %s after %u polls (rt %02x)\n", ok ? "yes" : "NO", r.pollsUp, rt);
    io.sleepMs(10);
    for (uint32_t mv = kRampStartMv + kRampStepMv;; mv += kRampStepMv) {   // qpnp_lcdb_set_voltage_step
      const uint32_t v = mv < target ? mv : target;
      maskedLcdb(rLdoVout, kVoutMask, voutCode(v));
      maskedLcdb(rNcpVout, kVoutMask, voutCode(v));
      if (mv >= target) break;
      io.sleepMs(1);
    }
    readLcdb(r.enUp, r.rtUp);
    io.spmiRd(rLdoVout, &r.ldoUp);
    io.spmiRd(rNcpVout, &r.ncpUp);
    io.spmiRd(rBstVout, &r.bstUp);
    io.log("GuampPanel: lcdb up: en %02x rt %02x ldo %02x (%u mV) ncp %02x (%u mV) bst %02x (%u mV)\n", r.enUp, r.rtUp, r.ldoUp,
           voutMv(r.ldoUp), r.ncpUp, voutMv(r.ncpUp), r.bstUp, bstMv(r.bstUp));
    return ok && (r.rtUp & kVregOk);
  }
  bool timingRuns(uint32_t &a, uint32_t &b) {
    a = frames();
    io.sleepMs(50);                                      // three frames at 60 Hz
    b = frames();
    return b != a;
  }

  int run() {
    using namespace guamp_backlight;
    // --- checks, no write
    if (!io.clocksOn()) { io.log("GuampPanel: refused: display clocks off\n"); return r.result = kNoClocks; }
    const uint32_t hw = io.rd(rHwVersion);
    r.dsiCtrl = io.rd(rCtrl);
    r.intfEn = io.intfRd(rIntfTimingEn);
    dsiState("before");
    if (readiness(hw, r.dsiCtrl) != kSent) {
      io.log("GuampPanel: refused: DSI hw %08x ctrl %08x: %s\n", hw, r.dsiCtrl, resultName(readiness(hw, r.dsiCtrl)));
      return r.result = kDsiNotReady;
    }
    // fin43: FRAME_LINE_COUNT_EN read 0 while the counter moved (1520 -> 1523): the counter itself is the test.
    if (!(r.intfEn & 1u) || !timingRuns(r.frames[0], r.frames[1])) {
      io.log("GuampPanel: refused: INTF1 timing engine %x, frame counter %u -> %u (count enable %08x)\n", r.intfEn, r.frames[0],
             r.frames[1], io.intfRd(rIntfFrameLineEn));
      return r.result = kIntfOff;
    }
    const bool counting = true;
    if (!io.lcdResetIsOutput() || !io.lcdResetLevel()) {
      io.log("GuampPanel: refused: gpio82 (LCD reset) not a driven-high output (level %u)\n", io.lcdResetLevel());
      return r.result = kResetPin;
    }
    uint32_t target = o.targetMv;
    if (o.lcdb) {
      int e = io.spmiRd(rEnableCtl1, &r.enBefore);
      if (!e) e = io.spmiRd(rIntRtSts, &r.rtBefore);
      if (!e) e = io.spmiRd(rLdoVout, &r.ldoBefore);
      if (!e) e = io.spmiRd(rNcpVout, &r.ncpBefore);
      if (!e) e = io.spmiRd(rBstVout, &r.bstBefore);
      if (!e) e = io.spmiRd(rLdoForcePd, &r.pdBefore);
      io.log("GuampPanel: lcdb before: en %02x rt %02x ldo %02x (%u mV) ncp %02x (%u mV) bst %02x (%u mV) force-pd %02x, "
             "read %d, writable %d\n", r.enBefore, r.rtBefore, r.ldoBefore, voutMv(r.ldoBefore), r.ncpBefore, voutMv(r.ncpBefore),
             r.bstBefore, bstMv(r.bstBefore), r.pdBefore, e, io.spmiWritable() ? 1 : 0);
      if (e) { io.log("GuampPanel: refused: LCDB not readable (%d)\n", e); return r.result = kLcdbRead; }
      if (!(r.enBefore & kModuleEn) || !(r.rtBefore & kVregOk)) {
        io.log("GuampPanel: refused: LCDB not on as expected\n");
        return r.result = kLcdbOff;
      }
      if (!io.spmiWritable()) { io.log("GuampPanel: refused: LCDB peripheral not writable by this EE\n"); return r.result = kSpmiReadOnly; }
      if (!target) target = voutMv(r.ldoBefore) > voutMv(r.ncpBefore) ? voutMv(r.ldoBefore) : voutMv(r.ncpBefore);
      if (target < kMinMv || target > kMaxMv) {
        io.log("GuampPanel: refused: target %u mV out of 4000..6000\n", target);
        return r.result = kBadVoltage;
      }
    }
    io.log("GuampPanel: cycle start (lcdb %s, reset %s while off, target %u mV, off %u ms, level %u, soft reset %u, on-command %s)\n",
           o.lcdb ? "on" : "off", o.resetLow ? "low" : "high", target, o.offMs, o.level, o.softReset ? 1u : 0u, o.hsOn ? "HS" : "LP");
    int warn = 0;

    // --- (1) video off: timing engine, one vblank, video engine off / command engine on
    io.intfWr(rIntfTimingEn, 0);
    io.sleepMs(40);
    r.frames[2] = frames();
    io.sleepMs(20);
    r.frames[3] = frames();
    io.wr(rCtrl, (r.dsiCtrl & ~kCtrlVideo) | kCtrlCmd);
    io.delayUs(100);
    dsiState("video stopped");
    if (counting && r.frames[2] != r.frames[3]) io.log("GuampPanel: warning: frame counter still moving %u -> %u\n", r.frames[2], r.frames[3]);

    if (o.readStatus) r.modeBeforeRc = powerMode("before (video stopped)", r.modeBefore);

    // --- (2) off-command in LP
    static const Cmd off[] = {{{0x28}, 1, 20}, {{0x10}, 1, 80}};
    r.offFails = sendList("off-command", off, 2);
    if (r.offFails) warn |= kWarnOffCmd;

    // --- (3) touch reset low (released after the panel init), LCD reset low if asked
    io.tpReset(false);
    if (o.resetLow) { io.lcdReset(false); io.sleepMs(3); }
    io.log("GuampPanel: resets: lcd gpio82 %u, touch reset low\n", io.lcdResetLevel());

    // --- (4) LCDB down (ibb pre-off-sleep 2 ms, FORCE_PD_ENABLE_WA)
    if (o.lcdb) {
      io.sleepMs(2);
      const int e1 = writeLcdb(rLdoForcePd, kForcePdEn | kForcePdMode, nullptr);
      const int e2 = e1 ? -1 : writeLcdb(rEnableCtl1, 0, &r.enDown);
      io.sleepMs(10);
      writeLcdb(rLdoForcePd, r.pdBefore & (uint8_t)~(kForcePdEn | kForcePdMode), nullptr);
      readLcdb(r.enDown, r.rtDown);
      r.railsDown = !e2 && !(r.enDown & kModuleEn);
      io.log("GuampPanel: lcdb down: %s (en %02x rt %02x)\n", r.railsDown ? "off" : "NOT off", r.enDown, r.rtDown);
      io.sleepMs(o.offMs);
      if (r.railsDown) {
        // --- (5) LCDB up: vddio post-on-sleep, ramp, ibb post-on-sleep
        io.sleepMs(20);
        if (!lcdbUp(target)) warn |= kWarnLcdbUp;
        io.sleepMs(20);
      } else {
        // the module never went off: no ramp (it would pull a live AVDD / AVEE down to 4.5 V); make sure it is on
        warn |= kWarnLcdbDown;
        if (!(r.enDown & kModuleEn)) writeLcdb(rEnableCtl1, kModuleEn, &r.enUp);
        readLcdb(r.enUp, r.rtUp);
        io.log("GuampPanel: lcdb not cycled, left on (en %02x rt %02x)\n", r.enUp, r.rtUp);
      }
    } else {
      io.sleepMs(o.offMs);
    }
    if (o.softReset) softReset("before the panel reset");
    // dsi_panel_reset: <1 5>, <0 2>, <1 60>
    io.lcdReset(true); io.sleepMs(5);
    io.lcdReset(false); io.sleepMs(2);
    io.lcdReset(true); io.sleepMs(60);
    io.log("GuampPanel: reset sequence 1/5 0/2 1/60 done, lcd gpio82 %u\n", io.lcdResetLevel());

    // --- (6) on-command in LP
    const Cmd on[] = {{{0x11}, 1, 160}, {{0x29}, 1, 50}, {{0x51, (uint8_t)(o.level >> 8), (uint8_t)o.level}, 3, 0},
                      {{0x55, 0x01}, 2, 0}, {{0x53, 0x2c}, 2, 0}, {{0xb9, 0x83, 0x10, 0x2d}, 4, 0}, {{0xcd, 0x01}, 2, 10}};
    r.onFails = sendList("on-command", on, 7, !o.hsOn);
    if (r.onFails) warn |= kWarnOnCmd;
    if (o.readStatus) {
      r.modeAfterRc = powerMode("after on-command", r.modeAfter);
      if (r.modeAfterRc || (r.modeAfter & 0x14) != 0x14) warn |= kWarnPanelMode;
    }

    // --- (7) video back: DSI_CTRL as found, timing engine on, frame counter checked
    io.wr(rCtrl, r.dsiCtrl);
    io.delayUs(100);
    io.intfWr(rIntfTimingEn, r.intfEn);
    bool runs = !counting || videoBack(r.frames[4], r.frames[5]);
    if (!runs) {
      // dsi_display_vid_engine_disable / enable around a controller soft reset, then the timing engine again
      io.log("GuampPanel: video not back: timing engine off, DSI video off, soft reset, video on, timing engine on\n");
      io.intfWr(rIntfTimingEn, 0);
      io.sleepMs(20);
      io.wr(rCtrl, (r.dsiCtrl & ~kCtrlVideo) | kCtrlCmd);
      softReset("video restart");
      io.wr(rCtrl, r.dsiCtrl);
      io.delayUs(100);
      io.intfWr(rIntfTimingEn, r.intfEn);
      runs = videoBack(r.frames[4], r.frames[5]);
      warn |= kWarnVideo;
      if (!runs) warn |= (r.frames[4] == r.frames[5]) ? kWarnFrozen : kWarnDsiIdle;
    }
    dsiState("video restored");
    io.log("GuampPanel: video %s (frames %u -> %u, DSI status %08x)\n", runs ? "running" : "NOT running", r.frames[4], r.frames[5],
           r.dsiStatusAfter);

    // --- (8) touch reset released (the touch service then resets, downloads and starts as usual)
    io.tpReset(true);
    io.sleepMs(50);
    r.result = warn;
    io.log("GuampPanel: cycle end: result %d (off-command refused %u, on-command refused %u, rails %s, power mode %02x -> %02x)\n",
           warn, r.offFails, r.onFails, o.lcdb ? (r.railsDown ? "cycled" : "NOT cycled") : "kept", r.modeBefore, r.modeAfter);
    return warn;
  }
};
template <class Io> int run(Io &io, const Options &o, Report &r) {
  Cycle<Io> c{io, o, r};
  return c.run();
}
}  // namespace guamp_panelcycle
#endif
