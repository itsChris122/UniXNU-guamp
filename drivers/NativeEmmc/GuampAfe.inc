// SPDX-License-Identifier: BSD-2-Clause-Patent
// q6afe client of the guamp ADSP (audio milestone 4): APR v2 packet builders for the AFE service (svc 4) and the
// first-tone sequence on the speaker port, without shared memory. Transport independent like GuampApr.inc: the owner
// gives send(), feeds AFE packets to receive(), drives step() with a ms clock; tests/adsp_ipc_host.cpp runs it.
// Protocol studied in the guamp kernel techpack/audio (no code copied): dsp/q6afe.c (q6afe_set_params_v2,
// q6afe_svc_set_params_v1, afe_set_lpass_clk_cfg, __afe_port_start, afe_send_cmd_port_start, afe_close,
// afe_dtmf_generate_rx, afe_vote_lpass_core_hw), include/dsp/apr_audio-v2.h, include/dsp/q6afe-v2.h,
// asoc/bengal.c (mi2s_clk), asoc/msm-dai-q6-v2.c (msm_dai_q6_mi2s_hw_params). Sizes checked by compiling the header:
// sizeof(afe_port_cmd_set_param_v2) 36, param_hdr_v1 12, afe_param_id_i2s_cfg 20, union afe_port_config 144,
// afe_svc_cmd_set_param_v1 36, afe_clk_set 20, afe_dtmf_generation_command 40 (all with the 20-byte apr_hdr).
//   LPASS vote  AFE_CMD_REMOTE_LPASS_CORE_HW_VOTE_REQUEST 0x100f4 {hw_block_id, client_name[8]} -> 0x100f5
//               {client_handle}; devote 0x100f6 {hw_block_id, client_handle}. Block 4 = DCODEC (LPI pins, bolero).
//   clock       AFE_SVC_CMD_SET_PARAM 0x100f3 {payload_size, mem_hdr 0 0 0, param_hdr_v1 {AFE_MODULE_CLOCK_SET
//               0x1028f, AFE_PARAM_ID_CLOCK_SET 0x10290, 20, 0}, afe_clk_set {1, PRI_MI2S_IBIT 0x100, 1536000,
//               COUPLE_NO 1, root 0, enable}}: 48 kHz x 2 x 16 bit (bengal.c update_mi2s_clk_val)
//   port config AFE_PORT_CMD_SET_PARAM_V2 0x100ef {port_id, payload_size, mem_hdr 0, param_hdr_v1
//               {AFE_MODULE_AUDIO_DEV_INTERFACE 0x1020c, AFE_PARAM_ID_I2S_CONFIG 0x1020d, 144 (the union, as Linux),
//               0}, afe_param_id_i2s_cfg {1, 16 bit, SD0 1, stereo 1, ws_src internal 1, 48000, PCM 0, 0} + 0 pad}
//   start/stop  AFE_PORT_CMD_DEVICE_START 0x100e5 / DEVICE_STOP 0x100e6 {port_id, reserved}
//   tone        AFE_PORTS_CMD_DTMF_CTL 0x10102 {int64 duration_ms (-1 continuous, 0 off), high, low, gain,
//               num_ports 1, port_ids, reserved}
// Every command is answered with APR_BASIC_RSP_RESULT {opcode, status} (0 = ADSP_EOK), the vote with 0x100f5.
// Port: PRIMARY_MI2S_RX 0x1000 (speaker, aw882xx; DTBO0 afe-rx-portid, bengal.c LPASS_BE_PRI_MI2S_RX).
#ifndef GUAMP_AFE_INC
#define GUAMP_AFE_INC
#include "GuampApr.inc"
namespace apr {

constexpr uint32_t kAfePortSetParamV2 = 0x000100ef, kAfeSvcSetParam = 0x000100f3, kAfeDeviceStart = 0x000100e5;
constexpr uint32_t kAfeDeviceStop = 0x000100e6, kAfeHwVote = 0x000100f4, kAfeHwVoteRsp = 0x000100f5;
constexpr uint32_t kAfeHwDevote = 0x000100f6, kAfeDtmf = 0x00010102;
constexpr uint32_t kAfeModuleDevInterface = 0x0001020c, kAfeParamI2sConfig = 0x0001020d;
constexpr uint32_t kAfeModuleClockSet = 0x0001028f, kAfeParamClockSet = 0x00010290;
constexpr uint16_t kPortPriMi2sRx = 0x1000, kPortPriMi2sTx = 0x1001;
constexpr uint32_t kClkPriMi2sIbit = 0x100, kClkAttrCoupleNo = 1, kHwBlockDcodec = 4, kPortConfigUnion = 144;
constexpr uint32_t kI2sCfgBytes = 20, kClkSetBytes = 20, kParamHdrBytes = 12, kMemHdrBytes = 12;

// AFE_CMD_REMOTE_LPASS_CORE_HW_VOTE_REQUEST: client_name is 8 bytes, NUL padded (strlcpy in Linux)
inline uint32_t afeHwVote(uint8_t *out, uint32_t cap, uint32_t token, uint32_t block, const char *name) {
  uint8_t p[12] = {};
  wr32(p, block);
  for (uint32_t i = 0; i < 7 && name && name[i]; i++) p[4 + i] = static_cast<uint8_t>(name[i]);
  return build(out, cap, kSvcAfe, 0, token, kAfeHwVote, p, sizeof(p));
}
inline uint32_t afeHwDevote(uint8_t *out, uint32_t cap, uint32_t token, uint32_t block, uint32_t handle) {
  uint8_t p[8];
  wr32(p, block);
  wr32(p + 4, handle);
  return build(out, cap, kSvcAfe, 0, token, kAfeHwDevote, p, sizeof(p));
}
// AFE_SVC_CMD_SET_PARAM with AFE_PARAM_ID_CLOCK_SET (afe_set_lpass_clk_cfg)
inline uint32_t afeClock(uint8_t *out, uint32_t cap, uint32_t token, uint32_t clkId, uint32_t hz, uint16_t attri,
                         uint16_t root, bool enable) {
  uint8_t p[4 + kMemHdrBytes + kParamHdrBytes + kClkSetBytes] = {};
  wr32(p, kParamHdrBytes + kClkSetBytes);
  uint8_t *h = p + 4 + kMemHdrBytes, *d = h + kParamHdrBytes;
  wr32(h, kAfeModuleClockSet);
  wr32(h + 4, kAfeParamClockSet);
  wr16(h + 8, kClkSetBytes);
  wr32(d, 1);   // AFE_API_VERSION_CLOCK_SET
  wr32(d + 4, clkId);
  wr32(d + 8, hz);
  wr16(d + 12, attri);
  wr16(d + 14, root);
  wr32(d + 16, enable ? 1 : 0);
  return build(out, cap, kSvcAfe, 0, token, kAfeSvcSetParam, p, sizeof(p));
}

struct I2sConfig {
  uint16_t port = kPortPriMi2sRx, bits = 16, lines = 1 /* AFE_PORT_I2S_SD0 */, stereo = 1, wsSrc = 1;
  uint32_t rate = 48000, paramBytes = kPortConfigUnion;   // Linux: sizeof(union afe_port_config); 20 also accepted?
};
// AFE_PORT_CMD_SET_PARAM_V2 + AFE_PARAM_ID_I2S_CONFIG in band (__afe_port_start -> q6afe_set_params_v2)
inline uint32_t afeI2s(uint8_t *out, uint32_t cap, uint32_t token, const I2sConfig &c) {
  uint8_t p[4 + kMemHdrBytes + kParamHdrBytes + kPortConfigUnion] = {};
  if (c.paramBytes < kI2sCfgBytes || c.paramBytes > kPortConfigUnion || (c.paramBytes & 3)) return 0;
  const uint32_t n = 4 + kMemHdrBytes + kParamHdrBytes + c.paramBytes;
  wr16(p, c.port);
  wr16(p + 2, static_cast<uint16_t>(kParamHdrBytes + c.paramBytes));
  uint8_t *h = p + 4 + kMemHdrBytes, *d = h + kParamHdrBytes;
  wr32(h, kAfeModuleDevInterface);
  wr32(h + 4, kAfeParamI2sConfig);
  wr16(h + 8, static_cast<uint16_t>(c.paramBytes));
  wr32(d, 1);   // AFE_API_VERSION_I2S_CONFIG
  wr16(d + 4, c.bits);
  wr16(d + 6, c.lines);
  wr16(d + 8, c.stereo);
  wr16(d + 10, c.wsSrc);
  wr32(d + 12, c.rate);   // data_format 0 = linear PCM, reserved 0
  return build(out, cap, kSvcAfe, 0, token, kAfePortSetParamV2, p, n);
}
// DEVICE_START / DEVICE_STOP
inline uint32_t afePort(uint8_t *out, uint32_t cap, uint32_t token, uint32_t opcode, uint16_t port) {
  uint8_t p[4] = {};
  wr16(p, port);
  return build(out, cap, kSvcAfe, 0, token, opcode, p, sizeof(p));
}
// AFE_PORTS_CMD_DTMF_CTL on one RX port
inline uint32_t afeDtmf(uint8_t *out, uint32_t cap, uint32_t token, int64_t ms, uint16_t high, uint16_t low,
                        uint16_t gain, uint16_t port) {
  uint8_t p[20] = {};
  const uint64_t d = static_cast<uint64_t>(ms);
  wr32(p, static_cast<uint32_t>(d));
  wr32(p + 4, static_cast<uint32_t>(d >> 32));
  wr16(p + 8, high);
  wr16(p + 10, low);
  wr16(p + 12, gain);
  wr16(p + 14, 1);
  wr16(p + 16, port);
  return build(out, cap, kSvcAfe, 0, token, kAfeDtmf, p, sizeof(p));
}

// The first tone: vote DCODEC, codec side prepare, MI2S clock, port config, start, amplifier on, DTMF for toneMs,
// then everything undone in reverse. Each command waits for its answer (1 s, as TIMEOUT_MS of q6afe.c). A failed
// set-up step skips to the undo steps; undo steps run whatever their result. Vote and codec-side failures do not stop
// the DSP part (its answers are the point of the test), they only mean no sound.
struct AfeTone {
  enum Step : int {
    Vote, HwPrepare, ClockOn, PortConfig, Start, AmpOn, DtmfOn, Play, DtmfOff, AmpOff, Stop, ClockOff, Devote, HwRelease,
    Done
  };
  static constexpr uint32_t kAnswerMs = 1000;
  void *ctx = nullptr;
  int (*send)(void *, const uint8_t *, uint32_t) = nullptr;   // 0 = queued, <0 = retry later
  void (*log)(void *, const char *what, uint32_t a, uint32_t b) = nullptr;
  int (*hw)(void *, int step, bool on) = nullptr;               // HwPrepare/AmpOn/AmpOff/HwRelease; 0 = ok
  I2sConfig port;
  uint32_t toneMs = 3000, clockHz = 1536000;
  uint16_t high = 1209, low = 697, gain = 0x0800;
  int step = Vote;
  bool waiting = false, voted = false, clockOn = false, started = false, ampOn = false, dtmfOn = false, setupFailed = false;
  bool played = false;
  uint32_t token = 0, waitOpcode = 0, deadline = 0, playUntil = 0, handle = 0, configTries = 0;
  uint32_t failedStep = 0, failedStatus = 0, undoErrors = 0, unexpected = 0, hwErrors = 0, answered = 0;

  void begin(uint32_t now) {
    step = Vote;
    waiting = false;
    deadline = now;
  }
  bool finished() const { return step == Done; }
  bool ok() const { return finished() && played && !setupFailed; }
  static const char *stepName(int s) {
    static const char *const n[] = {"lpass vote", "codec prepare", "mi2s clock on", "port config", "port start",
                                    "amp on", "dtmf on", "playing", "dtmf off", "amp off", "port stop",
                                    "mi2s clock off", "lpass devote", "codec release", "done"};
    return s >= 0 && s <= Done ? n[s] : "?";
  }

  void stepAt(uint32_t now) {
    for (unsigned guard = 0; guard < 16 && step != Done; guard++) {
      if (waiting) {
        if (now < deadline) return;
        waiting = false;
        result(now, false, 0xffffffffu);   // no answer in 1 s
        continue;
      }
      if (step == Play) {
        if (now < playUntil) return;
        played = true;
        step = DtmfOff;
        continue;
      }
      if (!issue(now)) return;   // channel busy: next pass
      if (!waiting) continue;    // synchronous step done
      return;
    }
  }

  void receive(const uint8_t *p, uint32_t n, uint32_t now) {
    Header h;
    const uint8_t *pl = nullptr;
    uint32_t pn = 0;
    if (!parse(p, n, &h, &pl, &pn) || h.srcSvc != kSvcAfe) {
      unexpected++;
      return note("malformed or foreign packet", n, 0);
    }
    if (h.opcode == kAfeHwVoteRsp && pn >= 4 && waiting && waitOpcode == kAfeHwVote && h.token == token) {
      handle = rd32(pl);
      answered++;
      waiting = false;
      note("vote handle", handle, 0);
      result(now, true, 0);
      return stepAt(now);
    }
    if (h.opcode == kBasicResult && pn >= 8) {
      const uint32_t op = rd32(pl), status = rd32(pl + 4);
      if (waiting && op == waitOpcode && h.token == token) {
        answered++;
        waiting = false;
        result(now, status == 0, status);
        return stepAt(now);
      }
      unexpected++;
      return note("stale afe result", op, status);
    }
    unexpected++;
    note("unexpected afe opcode", h.opcode, pn);
  }

private:
  void note(const char *what, uint32_t a, uint32_t b) {
    if (log) log(ctx, what, a, b);
  }
  bool command(uint32_t now, uint32_t opcode, const uint8_t *pkt, uint32_t n) {
    if (!n || !send || send(ctx, pkt, n) < 0) return false;
    waiting = true;
    waitOpcode = opcode;
    deadline = now + kAnswerMs;
    note(stepName(step), token, opcode);
    return true;
  }
  int callHw(int s, bool on) {
    const int e = hw ? hw(ctx, s, on) : -1;
    if (e) hwErrors++;
    return e;
  }
  // Sends the command of the current step (or runs a synchronous one). false = could not send, try again later.
  bool issue(uint32_t now) {
    uint8_t pkt[kMaxPacket];
    uint32_t n = 0;
    const uint32_t t = token + 1;
    switch (step) {
      case Vote: n = afeHwVote(pkt, sizeof(pkt), t, kHwBlockDcodec, "GUAMP"); break;
      case HwPrepare: {
        const int e = callHw(HwPrepare, true);
        note("codec prepare result", static_cast<uint32_t>(e), 0);
        step = ClockOn;
        return true;
      }
      case ClockOn: n = afeClock(pkt, sizeof(pkt), t, kClkPriMi2sIbit, clockHz, kClkAttrCoupleNo, 0, true); break;
      case PortConfig: n = afeI2s(pkt, sizeof(pkt), t, port); break;
      case Start: n = afePort(pkt, sizeof(pkt), t, kAfeDeviceStart, port.port); break;
      case AmpOn: {
        const int e = callHw(AmpOn, true);
        ampOn = e == 0;
        note("amp on result", static_cast<uint32_t>(e), 0);
        step = DtmfOn;
        return true;
      }
      case DtmfOn: n = afeDtmf(pkt, sizeof(pkt), t, -1, high, low, gain, port.port); break;
      case DtmfOff:
        if (!dtmfOn) { step = AmpOff; return true; }
        n = afeDtmf(pkt, sizeof(pkt), t, 0, high, low, gain, port.port);
        break;
      case AmpOff:
        if (ampOn) note("amp off result", static_cast<uint32_t>(callHw(AmpOff, false)), 0);
        ampOn = false;
        step = Stop;
        return true;
      case Stop:
        if (!started) { step = ClockOff; return true; }
        n = afePort(pkt, sizeof(pkt), t, kAfeDeviceStop, port.port);
        break;
      case ClockOff:
        if (!clockOn) { step = Devote; return true; }
        n = afeClock(pkt, sizeof(pkt), t, kClkPriMi2sIbit, clockHz, kClkAttrCoupleNo, 0, false);
        break;
      case Devote:
        if (!voted) { step = HwRelease; return true; }
        n = afeHwDevote(pkt, sizeof(pkt), t, kHwBlockDcodec, handle);
        break;
      case HwRelease:
        note("codec release result", static_cast<uint32_t>(callHw(HwRelease, false)), 0);
        step = Done;
        note("sequence done", played ? 1 : 0, setupFailed ? failedStep : 0);
        return true;
      default: return true;
    }
    if (!n || !command(now, rd32(pkt + 16), pkt, n)) return false;
    token = t;
    return true;
  }
  // The answer (or timeout, status ~0) of the step in flight
  void result(uint32_t now, bool good, uint32_t status) {
    if (!good) note(status == 0xffffffffu ? "no answer" : "refused", static_cast<uint32_t>(step), status);
    if (!good && step >= DtmfOff) undoErrors++;
    switch (step) {
      case Vote: voted = good; step = HwPrepare; break;   // no vote: LPI may be unreachable, the DSP part still runs
      case ClockOn:
        clockOn = good;
        if (good) step = PortConfig;
        else fail(status);
        break;
      case PortConfig:
        if (!good && configTries == 0 && port.paramBytes != kI2sCfgBytes) {
          configTries = 1;
          port.paramBytes = kI2sCfgBytes;   // retry once with the bare 20-byte struct
          note("port config retry with param size", kI2sCfgBytes, 0);
        } else if (good) {
          step = Start;
        } else {
          fail(status);
        }
        break;
      case Start:
        started = good;
        if (good) step = AmpOn;
        else fail(status);
        break;
      case DtmfOn:
        dtmfOn = good;
        if (good) {
          step = Play;
          playUntil = now + toneMs;
        } else {
          fail(status);
        }
        break;
      case DtmfOff: dtmfOn = false; step = AmpOff; break;
      case Stop: started = false; step = ClockOff; break;
      case ClockOff: clockOn = false; step = Devote; break;
      case Devote: voted = false; step = HwRelease; break;
      default: break;
    }
  }
  void fail(uint32_t status) {
    if (!setupFailed) {
      setupFailed = true;
      failedStep = static_cast<uint32_t>(step);
      failedStatus = status;
    }
    step = DtmfOff;   // undo from here: each undo step checks what was really done
  }
};

}  // namespace apr
#endif
