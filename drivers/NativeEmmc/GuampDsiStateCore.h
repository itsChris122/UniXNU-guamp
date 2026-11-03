// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp DSI / MDSS / panel state dump without kernel dependencies (guamp-dsi-dump=1, GuampDsiState.inc). Read only, apart
// from the DCS reads, which use the command DMA exactly as Linux's ESD status check does on this video-mode panel.
//
// Why: the HX83102D is a TDDI (display and touch in one chip). Its touch works in TWRP (Linux) and not under XNU with the
// same firmware, config and SPI protocol; the panel under XNU is the one the Motorola boot loader lit for its splash, while
// Linux power-cycles it and programs the link itself. This dumps the same windows that strumenti/twrp-dsi-stato.sh reads
// in TWRP through the sde_dbg debugfs (<base>_off / <base>_reg), so the two can be compared word for word:
//   MDSS_HW, MDP_TOP, CTL_0, LM_0, INTF_1 (timing engine: porches, pulse widths, polarity, panel format), PINGPONG_0,
//   DSI_0 controller (video mode: traffic mode, LP11 in HSA/HBP/HFP/BLLP, lanes, clock lane control, EOT, timers),
//   DSI_0 PHY 14nm + PLL (lane timings, PLL dividers, SSC), DISP_CC (byte / pixel / escape RCGs and branches).
// DSI_VIDEO_MODE_CTRL (0x010, dsi_ctrl_hw_cmn_video_engine_setup): 31 last line interleave, 28 pulse_mode_hsa_he,
// 24 hfp_lp11, 20 hbp_lp11, 16 hsa_lp11, 15 eof_bllp_lp11, 12 bllp_lp11, 9:8 traffic mode (0 non-burst sync pulse,
// 1 non-burst sync event = this dtbo, 2 burst), 5:4 format, 1:0 VC. DSI_CTRL (0x004): 0 enable, 1 video, 2 command,
// 4..7 data lanes, 8 clock lane. DSI_LANE_CTRL (0x0ac) bit 28 = clock lane forced HS (continuous clock).
// The dtbo (dsi_hlt_hx83102d_video): non_burst_sync_event, h-sync-pulse 1, bllp + bllp-eof power mode, tx-eot-append,
// 4 lanes, h 40/40/40, v fp 186 bp 12 pw 2, 60 Hz, t-clk-post 0x0b, t-clk-pre 0x23, PLL SSC down-spread (boot dtb).
//
// DCS reads with the video stream running (dsi_display_status_reg_read: the dtbo status check reads 0x0A in LP every few
// seconds under Linux, expected 0x9c/0x9d): SET_MAXIMUM_RETURN_PACKET_SIZE, clear the read-back, DCS READ 0x06 with the
// embedded BTA, both scheduled into the vertical front porch (186 lines, ~3.4 ms) as sendPacket does, LP when asked
// (DMA_CTRL bit 26: the link drops to LP11 in BLLP thanks to bllp-power-mode). The raw read-back registers are logged
// as they are: RDBK_DATA_CTRL 31:16 = bytes received, RDBK_DATA0..3, DATATYPE0/1 (dsi_ctrl_hw_cmn_get_cmd_read_data
// reads DATA[cnt-1] .. DATA0, each byte-swapped: the first byte on the wire is the top byte of DATA[cnt-1]).
#ifndef GUAMP_DSI_STATE_CORE_H
#define GUAMP_DSI_STATE_CORE_H
#include <stdint.h>
#include "GuampBacklightCore.h"

namespace guamp_dsistate {
// using-declarations, not a using-directive: NativeEmmc.cpp has its own rStatus & co. in an anonymous namespace
using guamp_backlight::Diag;
using guamp_backlight::kSent;
using guamp_backlight::kClocksOff;
using guamp_backlight::kCtrlOff;
using guamp_backlight::kNotVideo;
using guamp_backlight::kBadVersion;
using guamp_backlight::kBadTiming;
using guamp_backlight::kBusy;
using guamp_backlight::kTimeout;
using guamp_backlight::kNoBta;
using guamp_backlight::readiness;
using guamp_backlight::scheduleLine;
using guamp_backlight::scheduleFor;
using guamp_backlight::dmaCtrlFor;
using guamp_backlight::ackDmaDone;
using guamp_backlight::resultName;
using guamp_backlight::rHwVersion;
using guamp_backlight::rCtrl;
using guamp_backlight::rStatus;
using guamp_backlight::rFifoStatus;
using guamp_backlight::rActiveV;
using guamp_backlight::rTotal;
using guamp_backlight::rDmaCtrl;
using guamp_backlight::rDmaLength;
using guamp_backlight::rAckErr;
using guamp_backlight::rTrigCtrl;
using guamp_backlight::rSwTrigger;
using guamp_backlight::rTimeoutStatus;
using guamp_backlight::rDmaSchedule;
using guamp_backlight::rErrIntMask0;
using guamp_backlight::rIntCtrl;
using guamp_backlight::rTpgCtrl;
using guamp_backlight::rTpgFifo;
using guamp_backlight::rTpgFifoReset;
using guamp_backlight::rRdbkData0;
using guamp_backlight::rRdbkType0;
using guamp_backlight::rRdbkCtrl;
using guamp_backlight::kCtrlCmd;
using guamp_backlight::kStatusDmaBusy;
using guamp_backlight::kIntDmaDone;
using guamp_backlight::kIntBtaDone;
using guamp_backlight::kHdrBta;
using guamp_backlight::kIntEnables;
using guamp_backlight::kTrigMask;
using guamp_backlight::kTrigSw;
using guamp_backlight::kDmaLpm;
using guamp_backlight::kTpgCmdFifo;
using guamp_backlight::kOverflowMask;
using guamp_backlight::kOverflowClear;
using guamp_backlight::kBusyTimeoutUs;
using guamp_backlight::kDoneTimeoutUs;
using guamp_backlight::kPacketBytes;
using guamp_backlight::kDcsRead;
using guamp_backlight::kSetMaxReturn;

// One register window: physical address (inside a mapped page set), length in bytes, name as in the TWRP script.
struct Region { const char *name; uint64_t phys; uint32_t len; const char *twrp; uint32_t twrpOff; };
// bengal-sde.dtsi / boot dtb: mdp_phys 0x5e00000 (qcom,sde-off 0x1000 len 0x494, ctl 0x2000 size 0x1dc, mixer 0x45000
// size 0x320, intf 0x6b800 size 0x2b8, pp 0x71000 size 0xd4), dsi_ctrl 0x5e94000 0x400 (v2.4), dsi_phy 0x5e94400 0x588
// (14nm, the PLL shares the window), dispcc 0x5f00000. twrp = sde_dbg base name, twrpOff = offset inside that base.
constexpr Region kRegions[] = {
  {"mdss", 0x05e00000ULL, 0x040, "sde", 0x00000},
  {"top", 0x05e01000ULL, 0x494, "sde", 0x01000},
  {"ctl0", 0x05e02000ULL, 0x1dc, "sde", 0x02000},
  {"lm0", 0x05e45000ULL, 0x040, "sde", 0x45000},
  {"intf1", 0x05e6b800ULL, 0x2b8, "sde", 0x6b800},
  {"pp0", 0x05e71000ULL, 0x0d4, "sde", 0x71000},
  {"dsi0", 0x05e94000ULL, 0x400, "dsi0_ctrl", 0x000},
  {"phy0", 0x05e94400ULL, 0x588, "dsi0_phy", 0x000},
  {"dispcc", 0x05f02000ULL, 0x200, "-", 0x000},
  {"dispccpll", 0x05f00000ULL, 0x040, "-", 0x000},
};
constexpr unsigned kRegionCount = sizeof(kRegions) / sizeof(kRegions[0]);
constexpr uint32_t kMaxWords = 0x588 / 4;     // the longest window (PHY + PLL)
constexpr uint32_t kWordsPerLine = 8;

// DCS registers read through DSI (cmd, bytes asked). 1 byte: the MIPI DCS status set; 0x45 scanline (2); the Himax
// manufacturer registers are asked 10 bytes (4 header + 10 + 2 CRC = 16 = the whole read-back window).
struct DcsRead { uint8_t cmd, len; };
constexpr DcsRead kDcsReads[] = {
  {0x0a, 1}, {0x0b, 1}, {0x0c, 1}, {0x0d, 1}, {0x0e, 1}, {0x0f, 1}, {0x52, 1}, {0x54, 1}, {0x56, 1}, {0x45, 2},
  {0xda, 1}, {0xdb, 1}, {0xdc, 1}, {0x09, 4},
  {0xb9, 10}, {0xba, 10}, {0xb1, 10}, {0xb2, 10}, {0xb4, 10}, {0xbd, 10}, {0xbf, 10}, {0xc0, 10}, {0xc9, 10}, {0xcb, 10},
  {0xcc, 10}, {0xcd, 10}, {0xd1, 10}, {0xd2, 10}, {0xd3, 10}, {0xe7, 10}, {0xe9, 10}, {0xeb, 10},
};
constexpr unsigned kDcsReadCount = sizeof(kDcsReads) / sizeof(kDcsReads[0]);

// Read-back of one DCS read.
struct ReadBack { uint32_t ctrl, d[4], type0, type1, intCtrl, status, ack; int rc; uint32_t waitedUs; };

// One command packet (2 dwords) with the video engine running, scheduled into the vertical front porch like sendPacket,
// in LP when asked, and for a read request (BTA bit in the header) waiting for BTA_DONE too. Same register discipline as
// guamp_backlight::sendPacket: everything changed is restored on every path after the trigger.
template <class Io> int sendVideoPacket(Io &io, const uint32_t words[2], bool lp, Diag &d) {
  d = Diag{};
  if (!io.clocksOn()) return kClocksOff;
  d.hw = io.rd(rHwVersion);
  d.ctrl = io.rd(rCtrl);
  const int ready = readiness(d.hw, d.ctrl);
  if (ready != kSent) return ready;
  d.line = scheduleLine(io.rd(rActiveV), io.rd(rTotal));
  if (!d.line) return kBadTiming;
  const uint64_t t0 = io.nowUs();
  while (io.rd(rStatus) & kStatusDmaBusy) {
    if (io.nowUs() - t0 >= kBusyTimeoutUs) { d.status = io.rd(rStatus); return kBusy; }
    io.delayUs(50);
  }
  const uint32_t trig = io.rd(rTrigCtrl), schedule = io.rd(rDmaSchedule), mask0 = io.rd(rErrIntMask0), dma = io.rd(rDmaCtrl);
  if (!(d.ctrl & kCtrlCmd)) io.wr(rCtrl, d.ctrl | kCtrlCmd);
  if ((trig & kTrigMask) != kTrigSw) io.wr(rTrigCtrl, (trig & ~kTrigMask) | kTrigSw);
  io.wr(rErrIntMask0, mask0 | kOverflowMask);
  io.wr(rIntCtrl, ackDmaDone(io.rd(rIntCtrl)));
  io.wr(rTpgCtrl, 0);
  io.wr(rTpgFifoReset, 1);
  io.delayUs(1);
  io.wr(rTpgFifoReset, 0);
  io.wr(rTpgCtrl, kTpgCmdFifo);
  io.wr(rTpgFifo, words[0]);
  io.wr(rTpgFifo, words[1]);
  io.wr(rDmaCtrl, dmaCtrlFor(dma) | (lp ? kDmaLpm : 0u));
  io.wr(rDmaSchedule, scheduleFor(schedule, d.line));
  io.wr(rDmaLength, kPacketBytes);
  io.wr(rSwTrigger, 1);
  const uint64_t t1 = io.nowUs();
  bool done = false;
  for (;;) {
    d.intCtrl = io.rd(rIntCtrl);
    if (d.intCtrl & kIntDmaDone) { done = true; break; }
    const uint64_t waited = io.nowUs() - t1;
    if (waited >= kDoneTimeoutUs) break;
    if (waited < 1000) io.delayUs(25);
    else io.sleepMs(1);
  }
  d.waitedUs = (uint32_t)(io.nowUs() - t1);
  bool bta = !(words[0] & kHdrBta);
  if (done && !bta) {
    for (;;) {
      const uint32_t v = io.rd(rIntCtrl);
      if (v & kIntBtaDone) { bta = true; io.wr(rIntCtrl, (v & kIntEnables) | kIntBtaDone); break; }
      if (io.nowUs() - t1 >= kDoneTimeoutUs) break;
      io.delayUs(25);
    }
    d.btaUs = (uint32_t)(io.nowUs() - t1);
  }
  d.status = io.rd(rStatus);
  d.fifo = io.rd(rFifoStatus);
  d.ack = io.rd(rAckErr);
  d.timeouts = io.rd(rTimeoutStatus);
  if (done) io.wr(rIntCtrl, ackDmaDone(io.rd(rIntCtrl)));
  io.wr(rTpgCtrl, 0);
  io.wr(rTpgFifoReset, 1);
  io.delayUs(1);
  io.wr(rTpgFifoReset, 0);
  io.wr(rFifoStatus, io.rd(rFifoStatus) | kOverflowClear);
  io.wr(rErrIntMask0, mask0);
  io.wr(rDmaCtrl, dma);
  io.wr(rDmaSchedule, schedule);
  if ((trig & kTrigMask) != kTrigSw) io.wr(rTrigCtrl, trig);
  if (!(d.ctrl & kCtrlCmd)) io.wr(rCtrl, io.rd(rCtrl) & ~kCtrlCmd);
  return !done ? kTimeout : !bta ? kNoBta : kSent;
}

// SET_MAXIMUM_RETURN_PACKET_SIZE {len, 0}, clear the read-back (clear_rdbk_reg), DCS READ {cmd, 0} + BTA.
template <class Io> int readDcsVideo(Io &io, uint8_t cmd, uint8_t len, bool lp, ReadBack &out) {
  out = ReadBack{};
  Diag d;
  uint32_t w[2] = {(uint32_t)len | (kSetMaxReturn & 0x3fu) << 16 | 1u << 31, 0};
  int e = sendVideoPacket(io, w, lp, d);
  if (e == kSent) {
    io.wr(rRdbkCtrl, 1);
    io.wr(rRdbkCtrl, 0);
    w[0] = cmd | (kDcsRead & 0x3fu) << 16 | kHdrBta | 1u << 31;
    e = sendVideoPacket(io, w, lp, d);
  }
  out.rc = e;
  out.waitedUs = d.btaUs ? d.btaUs : d.waitedUs;
  out.intCtrl = d.intCtrl;
  out.status = d.status;
  out.ack = d.ack;
  if (io.clocksOn() && e != kClocksOff && e != kCtrlOff && e != kNotVideo && e != kBadVersion) {
    out.ctrl = io.rd(rRdbkCtrl);
    for (unsigned i = 0; i < 4; ++i) out.d[i] = io.rd(rRdbkData0 + 4u * i);
    out.type0 = io.rd(rRdbkType0);
    out.type1 = io.rd(rRdbkType0 + 4u);
  }
  return e;
}

// The bytes as they came on the wire (Linux order: DATA[cnt-1] .. DATA0, each big-endian), cnt = received / 4 rounded up,
// at most 4. Returns the number of bytes put in buf (<= 16).
inline unsigned wireBytes(const ReadBack &r, uint8_t buf[16]) {
  unsigned got = r.ctrl >> 16;
  if (!got) return 0;
  unsigned cnt = (got + 3u) / 4u;
  if (cnt > 4) cnt = 4;
  unsigned n = 0;
  for (unsigned i = cnt; i-- > 0;)
    for (unsigned b = 0; b < 4; ++b) buf[n++] = (uint8_t)(r.d[i] >> (24u - 8u * b));
  return n;
}

// Himax DD (display driver) registers through the touch side, two ways, the same in TWRP and here:
//  (a) firmware handshake (himax_mcu_read_DD_status, /proc/android_touch/debug "dd_debug,r:x<reg>,x<sub>,x<len>"):
//      write {len, sub, reg, 0xAA} to 0x900000FC, poll its byte 3 for 0xBB (100 x 10 ms), then len bytes at 0x10007F80.
//      Read only for the display; the firmware itself fetches the register.
//  (b) direct AHB window (himax_mcu_dd_reg_read): parameter p of register r in bank b at 0x30000000 | r << 12 | b << 10 | p,
//      one byte per 4-byte read, after hx83102d_dd_clk_set(true) (0xDD to 0x900880A8) and hx83102d_dd_reg_en (0xA5 to
//      0x900880E0, B9 = A5 83 10 2D at 0x300B9000); the clock is set back to 0 afterwards, as hx83102d_ic_id_read does.
//      Writes into the chip: only with guamp-dsi-dump-ddahb=1.
constexpr uint32_t kDdHandshake = 0x900000fcu, kDdData = 0x10007f80u, kDdRequest = 0xaau, kDdAck = 0xbbu;
constexpr uint32_t kOscEn = 0x900880a8u, kOscPw = 0x900880e0u;
constexpr uint32_t ddAhbAddr(uint8_t reg, uint8_t bank, uint8_t param) {
  return 0x30000000u | (uint32_t)reg << 12 | (uint32_t)(bank & 3u) << 10 | param;
}
constexpr uint32_t ddRequestWord(uint8_t reg, uint8_t sub, uint8_t len) {
  return (uint32_t)len | (uint32_t)sub << 8 | (uint32_t)reg << 16 | kDdRequest << 24;
}
constexpr uint8_t kDdFirst = 0xb0, kDdLast = 0xef, kDdLen = 0x20, kDdAhbLen = 16;
}  // namespace guamp_dsistate
#endif
