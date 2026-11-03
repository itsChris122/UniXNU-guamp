// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp panel backlight without kernel dependencies (host-tested by tests/backlight_host.cpp): the DCS packet, the level
// clamps, the fade and the DSI command-while-video sequence over an abstract register window. Used by GuampBacklight.inc.
//
// Hardware (bengal-guam-display.dtsi &dsi_hlt_hx83102d_video, device dtbo): qcom,mdss-dsi-bl-pmic-control-type =
// "bl_ctrl_dcs", qcom,bklt-dcs-2bytes-enabled, bl-min-level 16; hbm-off command 39 .. 51 0C CC and hbm-on 51 0F FF, so
// 0xccc is the normal maximum (0xfff is HBM, never used here). The level is the HX83102D's own DCS 0x51
// (SET_DISPLAY_BRIGHTNESS) with two bytes, high byte first (drm_mipi_dsi.c mipi_dsi_dcs_set_display_brightness_2bytes ->
// mipi_dsi_dcs_write, 3 bytes = MIPI_DSI_DCS_LONG_WRITE 0x39, HS: the panel sends its hbm commands in dsi_hs_mode), and 0
// turns the LEDs off. No WLED/PWM/I2C backlight IC: the only path is DSI0 (dsi_ctrl0 <0x5e94000 0x400>, v2.4).
//
// Sequence = techpack/display/msm/dsi dsi_message_tx + dsi_kickoff_msg_tx for a FIFO_STORE command on a video-mode
// panel with controller >= 2.2 (no wait_for_video_done there: the DMA is scheduled into the vertical blanking instead,
// dsi_ctrl_hw_22_schedule_dma_cmd with calculate_schedule_line = 1 + vsw + vbp + vactive = 1 + ACTIVE_V end), with the
// command engine switched on around the transfer as dsi_display_cmd_engine_enable/disable do:
//   wait CMD_MODE_DMA_BUSY clear; CMD_MODE_EN on; mask FIFO overflow errors (dsi_ctrl_mask_overflow); clear a stale
//   DMA_DONE; TPG FIFO mode + 2 dwords (kickoff_fifo_command); DMA_CTRL embedded/HS/no broadcast; schedule line; length;
//   SW trigger; poll DMA_DONE (no interrupt is enabled: the raw status bit is set regardless, as Linux's timeout path
//   reads it); ack; reset_cmd_fifo; unmask overflow (clearing its sticky bits); restore what was changed.
#ifndef GUAMP_BACKLIGHT_CORE_H
#define GUAMP_BACKLIGHT_CORE_H
#include <stdint.h>

namespace guamp_backlight {
// Physical windows
constexpr uint64_t kDsiPhys = 0x05e94000ULL;          // bengal-sde.dtsi mdss_dsi0 reg <0x5e94000 0x400>
constexpr uint64_t kDispccPhys = 0x05f00000ULL;       // bengal.dtsi qcom,dispcc@5f00000
constexpr uint64_t kGccDispAhbPage = 0x01417000ULL;   // gcc-bengal.c gcc_disp_ahb_clk halt_reg 0x1700c
constexpr uint32_t kGccDispAhbOff = 0x00c;
// dispcc-bengal.c branch halt registers: everything the DSI register file and its byte/escape/pixel clocks hang on
constexpr uint32_t kCbcrMdssAhb = 0x2044, kCbcrByte0 = 0x2024, kCbcrEsc0 = 0x202c, kCbcrPclk0 = 0x2004;

// dsi_ctrl_reg.h (+ dsi_ctrl_hw_2_2.c DSI_DMA_SCHEDULE_CTRL)
enum : uint32_t {
  rHwVersion = 0x000, rCtrl = 0x004, rStatus = 0x008, rFifoStatus = 0x00c, rActiveV = 0x028, rTotal = 0x02c,
  rDmaCtrl = 0x03c, rDmaLength = 0x04c, rAckErr = 0x068, rTrigCtrl = 0x084, rSwTrigger = 0x090, rTimeoutStatus = 0x0c0,
  rDmaSchedule = 0x100, rErrIntMask0 = 0x10c, rIntCtrl = 0x110, rTpgCtrl = 0x15c, rTpgFifo = 0x17c, rTpgFifoReset = 0x1ec,
  rVideoModeCtrl = 0x010, rLaneStatus = 0x0a8, rLaneCtrl = 0x0ac, rDln0PhyErr = 0x0b4, rClkCtrl = 0x11c, rClkStatus = 0x120,
};
constexpr uint32_t kCtrlEnable = 1u << 0, kCtrlVideo = 1u << 1, kCtrlCmd = 1u << 2;        // ctrl_en, video/cmd_engine_en
constexpr uint32_t kStatusDmaBusy = 1u << 1;                                               // DSI_STATUS0_CMD_MODE_DMA_BUSY
constexpr uint32_t kIntDmaDone = 1u << 0;                                                  // get_interrupt_status
constexpr uint32_t kIntBtaDone = 1u << 20;                                                 // DSI_BTA_DONE
// Read back (dsi_ctrl_reg.h): RDBK_DATA0 0x06c, DATATYPE0 0x07c, RDBK_DATA_CTRL 0x1d4 (count in 31:16, bit 0 clears)
constexpr uint32_t rRdbkData0 = 0x06c, rRdbkType0 = 0x07c, rRdbkCtrl = 0x1d4;
constexpr uint32_t kHdrBta = 1u << 29;                                                     // dsi_ctrl_copy_and_pad_cmd buf[3] BIT(5)
// INT_CTRL: even bits are write-1-to-clear status, odd bits the enables (enable_status_interrupts; 25 = error mask)
constexpr uint32_t kIntEnables = (1u << 1) | (1u << 9) | (1u << 11) | (1u << 13) | (1u << 15) | (1u << 17) | (1u << 21) |
                                 (1u << 25) | (1u << 29) | (1u << 31);
constexpr uint32_t kTrigMask = 7u, kTrigSw = 4u;                                           // trigger_map[DSI_TRIGGER_SW]
constexpr uint32_t kDmaBroadcast = 1u << 31, kDmaMaster = 1u << 30, kDmaEmbedded = 1u << 28, kDmaLpm = 1u << 26;
constexpr uint32_t kTpgCmdFifo = (1u << 1) | (1u << 2) | (3u << 16);   // CMD_DMA_TPG_EN, TPG_DMA_FIFO_MODE, custom pattern
constexpr uint32_t kScheduleEnable = 1u << 28;
constexpr uint32_t kOverflowMask = (0x1fu << 16) | (1u << 9);          // mask_error_intr DSI_FIFO_OVERFLOW
constexpr uint32_t kOverflowClear = (1u << 10) | (1u << 18) | (1u << 22) | (1u << 26) | (1u << 30);

// Levels
constexpr uint32_t kMaxLevel = 0xccc, kMinLevel = 16, kDefaultLevel = 0x666;
constexpr uint32_t kStepMs = 16, kMaxFadeMs = 5000;
constexpr uint32_t kBusyTimeoutUs = 20000, kDoneTimeoutUs = 50000;     // a scheduled DMA leaves within one frame (16.7 ms)

// 0 stays 0 (LEDs off); any other request is kept between bl-min and the normal maximum. Negative = 0.
constexpr uint32_t clampLevel(int64_t asked) {
  return asked <= 0 ? 0u : asked < (int64_t)kMinLevel ? kMinLevel : asked > (int64_t)kMaxLevel ? kMaxLevel : (uint32_t)asked;
}
// CoreBrightness FadePeriod: 16.16 fixed seconds -> milliseconds, capped
constexpr uint32_t fadeMsFromFixed(uint32_t fixed) {
  return (uint32_t)((((uint64_t)fixed * 1000u) >> 16) > kMaxFadeMs ? kMaxFadeMs : (((uint64_t)fixed * 1000u) >> 16));
}
// Linear fade: the level at elapsed of period (any time unit); from/to at the ends.
constexpr uint32_t fadeLevel(uint32_t from, uint32_t to, uint64_t elapsed, uint64_t period) {
  return (period == 0 || elapsed >= period) ? to
         : (uint32_t)((int64_t)from + ((int64_t)to - (int64_t)from) * (int64_t)elapsed / (int64_t)period);
}

// DCS long write {0x51, hi, lo} on virtual channel 0 as the MSM command buffer holds it (mipi_dsi_create_packet of this
// kernel: header WC low, WC high, VC << 6 | type, 0; dsi_ctrl_copy_and_pad_cmd: bit 6 of byte 3 = long packet, bit 7 =
// last command, payload padded with 0xff to 32 bits), as little-endian dwords for the TPG FIFO. 8 bytes = 2 dwords (even,
// so no extra zero dword).
constexpr uint32_t kDcsLongWrite = 0x39, kDcsSetBrightness = 0x51, kPacketBytes = 8;
constexpr uint32_t packetHeader(uint32_t type, uint32_t wordCount, bool last) {
  return (wordCount & 0xffu) | ((wordCount >> 8) & 0xffu) << 8 | (type & 0x3fu) << 16 | (1u << 30) | (last ? 1u << 31 : 0u);
}
inline void encodeBrightness(uint32_t level, uint32_t out[2]) {
  out[0] = packetHeader(kDcsLongWrite, 3, true);
  out[1] = kDcsSetBrightness | ((level >> 8) & 0xffu) << 8 | (level & 0xffu) << 16 | 0xffu << 24;
}

// DCS short write without parameter (MIPI_DSI_DCS_SHORT_WRITE 0x05: display off 0x28, sleep in 0x10, sleep out 0x11,
// display on 0x29), header {cmd, 0, VC 0 | type, last} with bit 6 of byte 3 clear (short packet). 4 bytes: one dword,
// odd, so the zero dword dsi_ctrl appends makes it 8 bytes.
constexpr uint32_t kDcsShortWrite = 0x05;
constexpr uint32_t kDcsDisplayOff = 0x28, kDcsSleepIn = 0x10, kDcsSleepOut = 0x11, kDcsDisplayOn = 0x29;
inline void encodeDcsShort(uint32_t cmd, uint32_t out[2]) {
  out[0] = (cmd & 0xffu) | (kDcsShortWrite & 0x3fu) << 16 | 1u << 31;
  out[1] = 0;
}

// Any DCS write of 1..4 bytes {cmd, params...} in one 8-byte FIFO image, as dsi_panel sends the on-command:
// 1 byte = DCS short write 0x05, 2 = short write with parameter 0x15 {cmd, param, type, last}, 3..4 = long write 0x39
// (header with the word count, payload padded with 0xff). 0 = too long / empty.
inline bool encodeDcs(const uint8_t *p, unsigned n, uint32_t out[2]) {
  if (!n || n > 4) return false;
  if (n <= 2) {
    out[0] = p[0] | (n == 2 ? (uint32_t)p[1] << 8 : 0u) | ((n == 2 ? 0x15u : kDcsShortWrite) & 0x3fu) << 16 | 1u << 31;
    out[1] = 0;
    return true;
  }
  out[0] = packetHeader(kDcsLongWrite, n, true);
  out[1] = 0;
  for (unsigned i = 0; i < 4; i++) out[1] |= (uint32_t)(i < n ? p[i] : 0xffu) << (8 * i);
  return true;
}

// Register value helpers
constexpr uint32_t dmaCtrlFor(uint32_t old) { return (old & ~(kDmaBroadcast | kDmaMaster | kDmaLpm)) | kDmaEmbedded; }
constexpr uint32_t scheduleFor(uint32_t old, uint32_t line) { return (old & ~(kScheduleEnable | 0xffffu)) | kScheduleEnable | (line & 0xffffu); }
constexpr uint32_t ackDmaDone(uint32_t intCtrl) { return (intCtrl & kIntEnables) | kIntDmaDone; }   // clears DMA_DONE only
constexpr bool cbcrRunning(uint32_t v) { return !(v & (1u << 31)) || (v & 3u) == 3u; }           // or enabled under HW gating
// DMA scheduling line in the vertical front porch from the timing the boot loader programmed (set_video_timing:
// ACTIVE_V = end << 16 | start, TOTAL = (v_total - 1) << 16 | h). 0 = implausible timing.
constexpr uint32_t scheduleLine(uint32_t activeV, uint32_t total) {
  return ((activeV >> 16) > (activeV & 0xffffu) + 15u && (activeV >> 16) + 1u <= (total >> 16)) ? (activeV >> 16) + 1u : 0u;
}

enum Result : int {
  kSent = 0, kClocksOff = 1, kCtrlOff = 2, kNotVideo = 3, kBadVersion = 4, kBadTiming = 5, kBusy = 6, kTimeout = 7,
  kVideoOn = 8, kNoBta = 9, kNoData = 10,
};
inline const char *resultName(int r) {
  switch (r) {
  case kSent: return "sent";
  case kClocksOff: return "display clocks off";
  case kCtrlOff: return "DSI controller off";
  case kNotVideo: return "DSI not in video mode";
  case kBadVersion: return "DSI version unsupported";
  case kBadTiming: return "video timing implausible";
  case kBusy: return "command DMA busy";
  case kTimeout: return "DMA_DONE timeout";
  case kVideoOn: return "video engine still on";
  case kNoBta: return "no BTA_DONE (panel did not answer)";
  case kNoData: return "no DCS short read response";
  default: return "?";
  }
}
// Whether the registers may be touched at all, from the clocks alone (the DSI register file is unclocked otherwise).
// Then whether this controller may carry a command now: enabled, video engine on, v2.2 or later (DMA scheduling).
inline int readiness(uint32_t hwVersion, uint32_t ctrl) {
  if (!(ctrl & kCtrlEnable)) return kCtrlOff;
  if (!(ctrl & kCtrlVideo)) return kNotVideo;
  if ((hwVersion >> 28) != 2 || ((hwVersion >> 16) & 0xfffu) < 2) return kBadVersion;
  return kSent;
}

// What one send saw, for the log
struct Diag { uint32_t hw, ctrl, line, intCtrl, status, fifo, ack, timeouts, waitedUs, btaUs, rdCtrl, rd0, rdType; };

// Io: uint32_t rd(uint32_t), void wr(uint32_t, uint32_t), bool clocksOn(), void delayUs(uint32_t), void sleepMs(uint32_t),
// uint64_t nowUs(). Never touches a register unless clocksOn() and the controller is enabled in video mode; every wait
// is bounded; whatever it changed is restored on every path after the trigger.
template <class Io> int sendPacket(Io &io, const uint32_t words[2], Diag &d) {
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
  const uint32_t trig = io.rd(rTrigCtrl), schedule = io.rd(rDmaSchedule), mask0 = io.rd(rErrIntMask0);
  if (!(d.ctrl & kCtrlCmd)) io.wr(rCtrl, d.ctrl | kCtrlCmd);
  if ((trig & kTrigMask) != kTrigSw) io.wr(rTrigCtrl, (trig & ~kTrigMask) | kTrigSw);
  io.wr(rErrIntMask0, mask0 | kOverflowMask);
  io.wr(rIntCtrl, ackDmaDone(io.rd(rIntCtrl)));
  io.wr(rTpgCtrl, 0);                                   // reset_cmd_fifo: a stale word must not lead the packet
  io.wr(rTpgFifoReset, 1);
  io.delayUs(1);
  io.wr(rTpgFifoReset, 0);
  io.wr(rTpgCtrl, kTpgCmdFifo);
  io.wr(rTpgFifo, words[0]);
  io.wr(rTpgFifo, words[1]);
  io.wr(rDmaCtrl, dmaCtrlFor(io.rd(rDmaCtrl)));
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
    if (waited < 1000) io.delayUs(25);                  // a vblank line is usually close: spin briefly, then sleep
    else io.sleepMs(1);
  }
  d.waitedUs = (uint32_t)(io.nowUs() - t1);
  d.status = io.rd(rStatus);
  d.fifo = io.rd(rFifoStatus);
  d.ack = io.rd(rAckErr);
  d.timeouts = io.rd(rTimeoutStatus);
  if (done) io.wr(rIntCtrl, ackDmaDone(d.intCtrl));
  io.wr(rTpgCtrl, 0);                                   // reset_cmd_fifo
  io.wr(rTpgFifoReset, 1);
  io.delayUs(1);
  io.wr(rTpgFifoReset, 0);
  io.wr(rFifoStatus, io.rd(rFifoStatus) | kOverflowClear);   // mask_overflow(false): sticky overflow bits, then unmask
  io.wr(rErrIntMask0, mask0);
  io.wr(rDmaSchedule, schedule);
  if ((trig & kTrigMask) != kTrigSw) io.wr(rTrigCtrl, trig);
  if (!(d.ctrl & kCtrlCmd)) io.wr(rCtrl, io.rd(rCtrl) & ~kCtrlCmd);
  return done ? kSent : kTimeout;
}
// Command-mode transfer (guamp-touch-panelcycle=5): the MDSS timing engine is stopped and the DSI video engine is off, as
// Linux has them when dsi_panel_disable / dsi_panel_enable send the off- and on-command (sde_encoder_phys_vid_disable then
// dsi_display_vid_engine_disable before dsi_panel_disable; dsi_panel_enable before dsi_display_vid_engine_enable). Same
// FIFO sequence as sendPacket, but nothing to schedule (DMA_SCHEDULE enable cleared: the DMA leaves at once) and, for an
// LP transfer, DMA_CTRL LOW_POWER bit 26 (kickoff_fifo_command use_lpm; on-command-state dsi_lp_mode). Everything changed
// is restored on every path after the trigger, DMA_CTRL included.
template <class Io> int sendPacketCmd(Io &io, const uint32_t words[2], bool lp, Diag &d) {
  d = Diag{};
  if (!io.clocksOn()) return kClocksOff;
  d.hw = io.rd(rHwVersion);
  d.ctrl = io.rd(rCtrl);
  if (!(d.ctrl & kCtrlEnable)) return kCtrlOff;
  if (d.ctrl & kCtrlVideo) return kVideoOn;
  if ((d.hw >> 28) != 2 || ((d.hw >> 16) & 0xfffu) < 2) return kBadVersion;
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
  io.wr(rTpgCtrl, 0);                                   // reset_cmd_fifo
  io.wr(rTpgFifoReset, 1);
  io.delayUs(1);
  io.wr(rTpgFifoReset, 0);
  io.wr(rTpgCtrl, kTpgCmdFifo);
  io.wr(rTpgFifo, words[0]);
  io.wr(rTpgFifo, words[1]);
  io.wr(rDmaCtrl, dmaCtrlFor(dma) | (lp ? kDmaLpm : 0u));
  if (schedule & kScheduleEnable) io.wr(rDmaSchedule, schedule & ~kScheduleEnable);
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
  if (done && !bta) {                                   // read request: the panel answers after the bus turnaround
    for (;;) {
      const uint32_t v = io.rd(rIntCtrl);
      if (v & kIntBtaDone) { bta = true; io.wr(rIntCtrl, (v & kIntEnables) | kIntBtaDone); break; }
      if (io.nowUs() - t1 >= kDoneTimeoutUs) break;
      io.delayUs(25);
    }
    d.btaUs = (uint32_t)(io.nowUs() - t1);
    d.rdCtrl = io.rd(rRdbkCtrl);
    d.rd0 = io.rd(rRdbkData0);
    d.rdType = io.rd(rRdbkType0);
  }
  d.status = io.rd(rStatus);
  d.fifo = io.rd(rFifoStatus);
  d.ack = io.rd(rAckErr);
  d.timeouts = io.rd(rTimeoutStatus);
  if (done) io.wr(rIntCtrl, ackDmaDone(io.rd(rIntCtrl)));
  io.wr(rTpgCtrl, 0);                                   // reset_cmd_fifo
  io.wr(rTpgFifoReset, 1);
  io.delayUs(1);
  io.wr(rTpgFifoReset, 0);
  io.wr(rFifoStatus, io.rd(rFifoStatus) | kOverflowClear);
  io.wr(rErrIntMask0, mask0);
  io.wr(rDmaCtrl, dma);
  if (schedule & kScheduleEnable) io.wr(rDmaSchedule, schedule);
  if ((trig & kTrigMask) != kTrigSw) io.wr(rTrigCtrl, trig);
  if (!(d.ctrl & kCtrlCmd)) io.wr(rCtrl, io.rd(rCtrl) & ~kCtrlCmd);
  return !done ? kTimeout : !bta ? kNoBta : kSent;
}
// DCS short read of one byte in command mode (dsi_message_rx, short_resp): SET_MAXIMUM_RETURN_PACKET_SIZE 0x37 {1, 0},
// clear the read-back registers (clear_rdbk_reg), DCS_READ 0x06 {cmd, 0} with the embedded BTA, then RDBK_DATA0 as
// get_cmd_read_data reads it (bits 31:24 = response type, 0x21 DCS short read 1 byte / 0x02 ack-error report).
// The panel status check of the dtbo is this read of 0x0A in LP (expected 0x9c / 0x9d: booster, sleep out, display on).
constexpr uint32_t kDcsRead = 0x06, kSetMaxReturn = 0x37, kRespDcsShort1 = 0x21, kRespAckErr = 0x02;
template <class Io> int readDcsCmd(Io &io, uint8_t cmd, bool lp, uint8_t *value, Diag &d) {
  *value = 0;
  uint32_t w[2] = {1u | (kSetMaxReturn & 0x3fu) << 16 | 1u << 31, 0};
  int e = sendPacketCmd(io, w, lp, d);
  if (e != kSent) return e;
  io.wr(rRdbkCtrl, 1);
  io.wr(rRdbkCtrl, 0);
  w[0] = cmd | (kDcsRead & 0x3fu) << 16 | kHdrBta | 1u << 31;
  e = sendPacketCmd(io, w, lp, d);
  if (e != kSent) return e;
  // get_cmd_read_data keeps ntohl(RDBK_DATA0) in memory and takes byte 0 as the type: register bits 31:24 = type,
  // 23:16 = the byte read (dsi_parse_short_read1_resp: rx_buf[0] = buff[1])
  if ((d.rdCtrl >> 16) == 0 || (d.rd0 >> 24) != kRespDcsShort1) return kNoData;
  *value = (uint8_t)(d.rd0 >> 16);
  return kSent;
}
template <class Io> int sendDcsCmd(Io &io, const uint8_t *p, unsigned n, bool lp, Diag &d) {
  uint32_t words[2];
  if (!encodeDcs(p, n, words)) return kBadTiming;
  return sendPacketCmd(io, words, lp, d);
}
template <class Io> int sendBrightness(Io &io, uint32_t level, Diag &d) {
  uint32_t words[2];
  encodeBrightness(level, words);
  return sendPacket(io, words, d);
}
template <class Io> int sendDcs(Io &io, const uint8_t *p, unsigned n, Diag &d) {
  uint32_t words[2];
  if (!encodeDcs(p, n, words)) return kBadTiming;
  return sendPacket(io, words, d);
}
template <class Io> int sendDcsShort(Io &io, uint32_t cmd, Diag &d) {
  uint32_t words[2];
  encodeDcsShort(cmd, words);
  return sendPacket(io, words, d);
}

// Log rate limit: a burst of kBurst lines, then one per kRefillMs; what was dropped is counted for the next line.
struct LogGate {
  static constexpr uint32_t kBurst = 4, kRefillMs = 2000;
  uint32_t tokens = kBurst, dropped = 0;
  uint64_t lastMs = 0;
  bool allow(uint64_t nowMs) {
    if (nowMs > lastMs) {
      const uint64_t refill = (nowMs - lastMs) / kRefillMs;
      if (refill) {
        tokens = refill >= kBurst || tokens + refill >= kBurst ? kBurst : tokens + (uint32_t)refill;
        lastMs = lastMs + refill * kRefillMs;
      }
    } else if (nowMs < lastMs) {
      lastMs = nowMs;
    }
    if (!tokens) { dropped = dropped + 1; return false; }
    tokens = tokens - 1;
    return true;
  }
  uint32_t takeDropped() { const uint32_t n = dropped; dropped = 0; return n; }
};
}  // namespace guamp_backlight
#endif
