// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp microphone, DSP side (audio agent 3): the capture chain AFE TX_CODEC_DMA_TX_3 -> ADM TX COPP -> ASM read session,
// as the guamp Linux kernel sets it up for "audio-record" + "handset-mic" (studied, no code copied):
//   dsp/q6afe.c        afe_vote_lpass_core_hw (MACRO 3, DCODEC 4), afe_set_lpass_clk_cfg (tx core 0x30C, tx npl
//                      0x30D at 19.2 MHz: dtbo fragment@26 tx_core_clk/tx_npl_clk, audio-ext-clk-up.c), __afe_port_start
//                      with AFE_PARAM_ID_CODEC_DMA_CONFIG 0x102B8 (param_size = sizeof(union afe_port_config) = 144)
//   asoc/msm-dai-q6-v2.c  msm_dai_q6_cdc_dma_hw_params (v1, 48000, 16 bit, data_format 0, 1 channel, mask 0x1)
//   dsp/q6adm.c        adm_open v5, mode TX 2, endpoint 0xB037; adm_matrix_map with ADM_MATRIX_ID_AUDIO_TX 1
//   dsp/q6asm.c        __q6asm_open_read (ASM_STREAM_CMD_OPEN_READ_V3 0x10DB4: mode 0 legacy, src matrix 0, preproc
//                      none, ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2, 16 bit), q6asm_enc_cfg_blk_pcm (SET_ENCDEC_PARAM 0x10C10,
//                      ENC_CFG_BLK_V2 0x10DA3: frames_per_buf 0, blk 20, 1 ch, 16 bit, 48000, signed, map {FC 3}),
//                      q6asm_run, q6asm_read (ASM_DATA_CMD_READ_V2 0x10DAC {addr lsw, msw, handle, size, seq}) ->
//                      ASM_DATA_EVENT_READ_DONE_V2 0x10D9A {status, lsw, msw, handle, size, offset, ts lsw, ts msw,
//                      flags, num_frames, seq}
// Layouts checked against include/dsp/apr_audio-v2.h with comments stripped (afe_param_id_cdc_dma_cfg_t 16 bytes,
// asm_stream_cmd_open_read_v3 20 + 20, asm_multi_channel_pcm_enc_cfg_v2 20 + 36, asm_data_event_read_done_v2 44).
// Order (DPCM capture): votes, TX clocks, codec bring-up, AFE port config + start, codec DAPM path, then the front end:
// shared memory map, ADM open, ASM open read, encoder config, matrix route, run, 4 reads of 10 ms in flight.
// Without shared memory (no SMMU mapping for stream 0x1c1 yet, GuampAsm.inc §) the chain stops after the codec path:
// AFE port and codec are up, no samples (verdict "MIC PATH UP (no shared memory)").
// Transport independent like GuampAfe.inc. Tokens carry kMicToken so the owner can route AFE/ADM answers here; ASM
// packets are ours by port (session 2).
#ifndef GUAMP_MIC_PATH_INC
#define GUAMP_MIC_PATH_INC
#include "GuampAsm.inc"
#include "guamp_mic_dev.h"
namespace apr {

constexpr uint32_t kMicToken = 0x4D490000u, kMicTokenMask = 0xFFFF0000u;
constexpr uint16_t kPortTxCdcDma3 = 0xB037;
// kAfeParamCdcDmaConfig, kCdcDmaCfgBytes, kHwBlockMacro and the CODEC_DMA_CONFIG builder: GuampAfe.inc (shared)
constexpr uint32_t kClkTxCore = 0x30C, kClkTxNpl = 0x30D, kClkTxHz = 19200000;
constexpr uint32_t kAsmOpenReadV3 = 0x00010DB4, kAsmSetEncdec = 0x00010C10, kAsmEncCfgBlkV2 = 0x00010DA3;
constexpr uint32_t kAsmReadV2 = 0x00010DAC, kAsmReadDoneV2 = 0x00010D9A, kAdmMatrixTx = 1, kPcmChannelFc = 3;

// ADM_CMD_DEVICE_OPEN_V5, TX path COPP on a TX AFE port, mono (front centre)
inline uint32_t admOpenTx(uint8_t *out, uint32_t cap, uint32_t token, uint16_t afePort, uint32_t topology, uint32_t rate) {
  uint8_t p[28] = {};
  wr16(p + 2, 2);   // ADM_CMD_COPP_OPEN_MODE_OF_OPERATION_TX_PATH_COPP
  wr16(p + 4, afePort);
  wr16(p + 6, 0xffff);
  wr32(p + 8, topology);
  wr16(p + 12, 1);
  wr16(p + 14, 16);
  wr32(p + 16, rate);
  p[20] = kPcmChannelFc;
  return build(out, cap, kSvcAdm, afePort, token, kAdmOpenV5, p, sizeof(p));
}
// ADM_CMD_MATRIX_MAP_ROUTINGS_V5 on the TX matrix: copp -> session
inline uint32_t admRouteTx(uint8_t *out, uint32_t cap, uint32_t token, uint16_t session, uint16_t copp) {
  uint8_t p[16] = {};
  wr32(p, kAdmMatrixTx);
  wr32(p + 4, 1);
  wr16(p + 8, session);
  wr16(p + 10, 1);
  wr16(p + 12, copp);
  return build(out, cap, kSvcAdm, 0, token, kAdmMatrixMapV5, p, sizeof(p));
}
inline uint32_t asmOpenRead(uint8_t *out, uint32_t cap, uint32_t token, uint16_t port, uint16_t bits) {
  uint8_t p[20] = {};
  wr32(p + 8, kPoppNone);          // mode 0 (legacy), src matrix 0, preprocessing none
  wr32(p + 12, kFmtMultiChPcmV2);
  wr16(p + 16, bits);
  return build(out, cap, kSvcAsm, port, token, kAsmOpenReadV3, p, sizeof(p));
}
inline uint32_t asmEncCfgPcm(uint8_t *out, uint32_t cap, uint32_t token, uint16_t port, uint32_t rate) {
  uint8_t p[36] = {};
  wr32(p, kAsmEncCfgBlkV2);
  wr32(p + 4, 28);    // sizeof(enc_cfg) - hdr - encdec
  wr32(p + 8, 0);     // frames_per_buf
  wr32(p + 12, 20);   // enc_cfg_blk_size
  wr16(p + 16, 1);
  wr16(p + 18, 16);
  wr32(p + 20, rate);
  wr16(p + 24, 1);    // signed
  p[28] = kPcmChannelFc;
  return build(out, cap, kSvcAsm, port, token, kAsmSetEncdec, p, sizeof(p));
}
inline uint32_t asmRead(uint8_t *out, uint32_t cap, uint32_t token, uint16_t port, uint32_t lsw, uint32_t msw,
                        uint32_t handle, uint32_t size, uint32_t seq) {
  uint8_t p[20] = {};
  wr32(p, lsw);
  wr32(p + 4, msw);
  wr32(p + 8, handle);
  wr32(p + 12, size);
  wr32(p + 16, seq);
  return build(out, cap, kSvcAsm, port, token, kAsmReadV2, p, sizeof(p));
}

// The whole capture. The owner sets send/log/hw/sink (+ shm when it has one), begin(), feeds every AFE packet whose token
// has kMicToken, every ADM packet with kMicToken and every ASM packet for port() to receive(), and steps with a ms clock.
// stop() tears everything down in reverse (each undo step checks what was really done).
struct MicPath {
  enum Step : int {
    VoteMacro, VoteDcodec, ClkCore, ClkNpl, CodecUp, PortConfig, PortStart, PathOn, MemMap, AdmOpen, AsmOpen, EncCfg,
    Route, Run, Capturing, AsmClose, AdmClose, MemUnmap, PathOff, PortStop, CodecDown, ClkNplOff, ClkCoreOff,
    DevoteDcodec, DevoteMacro, Done
  };
  static constexpr uint32_t kSlots = 4, kSlotStride = 4096, kAnswerMs = 1000;
  static constexpr uint32_t kPeriodBytes = GUAMP_MIC_PERIOD_FRAMES * GUAMP_MIC_FRAME_BYTES;   // 960
  void *ctx = nullptr;
  int (*send)(void *, const uint8_t *, uint32_t) = nullptr;
  void (*log)(void *, const char *what, uint32_t a, uint32_t b) = nullptr;
  int (*hw)(void *, int step) = nullptr;                                       // CodecUp/PathOn/PathOff/CodecDown, 0 = ok
  void (*sink)(void *, const uint8_t *pcm, uint32_t bytes) = nullptr;          // captured period (s16 mono 48 kHz)
  Shm shm;                                                                      // cpu == nullptr: no shared memory
  uint16_t session = 2, stream = 1, afePort = kPortTxCdcDma3;
  uint32_t cdcParamBytes = kPortConfigUnion;
  // AFE_LPASS_CORE_HW_MACRO_BLOCK: the guamp DT has no lpass_core_hw_vote clock (only lpass_audio_hw_vote = DCODEC),
  // so Linux never sends it on bengal; the ADSP answered 1 (ADSP_EFAILED) on p5. Off unless guamp-audio-mic-macro=1.
  bool voteMacro = false;
  int step = VoteMacro;
  bool waiting = false, stopping = false, setupFailed = false;
  bool votedMacro = false, votedDcodec = false, clkCore = false, clkNpl = false, codecUp = false, portUp = false;
  bool codecTried = false, pathUp = false, mapped = false, admOpened = false, asmOpened = false, configTried = false;
  uint32_t token = kMicToken, waitOpcode = 0, deadline = 0, handleMacro = 0, handleDcodec = 0, memHandle = 0, copp = 0;
  uint32_t failedStep = 0, failedStatus = 0, answered = 0, unexpected = 0, hwErrors = 0, undoErrors = 0;
  uint32_t seq = 0, inFlight = 0, reads = 0, readDones = 0, readErrors = 0, framesIn = 0, peakMax = 0;
  uint64_t sumSquares = 0, samples = 0;
  bool slotBusy[kSlots] = {};

  uint16_t port() const { return asmPort(session, stream); }
  bool finished() const { return step == Done; }
  bool capturing() const { return step == Capturing; }
  bool upWithoutMemory() const { return step == Capturing && !shm.cpu; }
  void begin(uint32_t now) {
    step = VoteMacro;
    deadline = now;
  }
  void stop() { stopping = true; }
  static const char *stepName(int s) {
    static const char *const n[] = {
      "lpass macro vote", "lpass dcodec vote", "tx core clock", "tx npl clock", "codec bring-up", "cdc dma port config",
      "cdc dma port start", "codec capture path", "asm memory map", "adm open tx", "asm open read", "asm encoder config",
      "adm route tx", "asm run", "capturing", "asm close", "adm close", "asm memory unmap", "codec path off",
      "cdc dma port stop", "codec down", "tx npl clock off", "tx core clock off", "lpass dcodec devote",
      "lpass macro devote", "done"};
    return s >= 0 && s <= Done ? n[s] : "?";
  }
  // RMS of everything captured so far, in units of a 16-bit sample
  uint32_t rms() const {
    if (!samples) return 0;
    const uint64_t m = sumSquares / samples;
    uint64_t x = 0;
    while ((x + 1) * (x + 1) <= m) x++;
    return static_cast<uint32_t>(x);
  }

  void stepAt(uint32_t now) {
    for (unsigned guard = 0; guard < 24 && step != Done; guard++) {
      if (waiting) {
        if (now < deadline) return;
        waiting = false;
        result(now, false, 0xffffffffu);
        continue;
      }
      if (step == Capturing) {
        if (stopping && !inFlight) {
          step = AsmClose;
          continue;
        }
        if (!stopping) feed();
        return;
      }
      if (!issue(now)) return;
      if (waiting) return;
    }
  }

  void receive(const uint8_t *p, uint32_t n, uint32_t now) {
    Header h;
    const uint8_t *pl = nullptr;
    uint32_t pn = 0;
    if (!parse(p, n, &h, &pl, &pn)) {
      unexpected++;
      return;
    }
    if (h.srcSvc == kSvcAsm && h.opcode == kAsmReadDoneV2 && pn >= 44) {
      readDone(pl);
      return stepAt(now);
    }
    if (!waiting || h.token != token) {
      unexpected++;
      return note("stale or foreign mic packet", h.opcode, h.token);
    }
    bool good = false;
    uint32_t status = 0;
    if (h.opcode == kAfeHwVoteRsp && pn >= 4 && waitOpcode == kAfeHwVote) {
      (step == VoteMacro ? handleMacro : handleDcodec) = rd32(pl);
      good = true;
    } else if (h.opcode == kAsmMemMapRsp && pn >= 4 && waitOpcode == kAsmMemMap) {
      memHandle = rd32(pl);
      good = true;
    } else if (h.opcode == kAdmOpenRspV5 && pn >= 8 && waitOpcode == kAdmOpenV5) {
      status = rd32(pl);
      copp = rd32(pl + 4);
      good = status == 0;
    } else if (h.opcode == kBasicResult && pn >= 8 && rd32(pl) == waitOpcode) {
      status = rd32(pl + 4);
      good = status == 0;
    } else {
      unexpected++;
      return note("unexpected mic answer", h.opcode, pn);
    }
    answered++;
    waiting = false;
    result(now, good, status);
    stepAt(now);
  }

private:
  void note(const char *what, uint32_t a, uint32_t b) {
    if (log) log(ctx, what, a, b);
  }
  bool command(uint32_t now, const uint8_t *pkt, uint32_t n, uint32_t t) {
    if (!n || !send || send(ctx, pkt, n) < 0) return false;
    token = t;
    waiting = true;
    waitOpcode = rd32(pkt + 16);
    deadline = now + kAnswerMs;
    note(stepName(step), t, waitOpcode);
    return true;
  }
  int callHw(int s) {
    const int e = hw ? hw(ctx, s) : -1;
    if (e) hwErrors++;
    note("codec step result", static_cast<uint32_t>(s), static_cast<uint32_t>(e));
    return e;
  }
  // Sends the command of the current step, or runs a synchronous one. false = channel busy, try later.
  bool issue(uint32_t now) {
    uint8_t pkt[kMaxPacket];
    uint32_t n = 0;
    const uint32_t t = kMicToken | ((token + 1) & 0xFFFF);
    switch (step) {
      case VoteMacro:
        if (!voteMacro) { step = VoteDcodec; return true; }
        n = afeHwVote(pkt, sizeof(pkt), t, kHwBlockMacro, "GMICM");
        break;
      case VoteDcodec: n = afeHwVote(pkt, sizeof(pkt), t, kHwBlockDcodec, "GMICD"); break;
      case ClkCore: n = afeClock(pkt, sizeof(pkt), t, kClkTxCore, kClkTxHz, kClkAttrCoupleNo, 0, true); break;
      case ClkNpl: n = afeClock(pkt, sizeof(pkt), t, kClkTxNpl, kClkTxHz, kClkAttrCoupleNo, 0, true); break;
      case CodecUp:
        codecTried = true;
        codecUp = callHw(CodecUp) == 0;
        if (!codecUp) fail(0xfffffffeu);   // no wcd937x: nothing to capture, undo
        else step = PortConfig;
        return true;
      case PortConfig:
        {
          I2sConfig c;   // mono codec DMA: channels 1, mask 0x1
          c.port = afePort;
          c.bits = 16;
          c.stereo = 0;
          c.rate = GUAMP_MIC_RATE;
          c.paramBytes = cdcParamBytes;
          c.cdcDma = true;
          n = afeCdcDma(pkt, sizeof(pkt), t, c);
        }
        break;
      case PortStart: n = apr::afePort(pkt, sizeof(pkt), t, kAfeDeviceStart, afePort); break;
      case PathOn:
        pathUp = callHw(PathOn) == 0;
        if (!pathUp) fail(0xfffffffeu);
        else if (!shm.cpu) {
          note("no shared memory for the ADSP: capture stops at the AFE port (codec and port up)", 0, 0);
          step = Capturing;
        } else {
          step = MemMap;
        }
        return true;
      case MemMap: n = asmMemMap(pkt, sizeof(pkt), t, shm.iovaLsw, shm.msw, shm.bytes); break;
      case AdmOpen: n = admOpenTx(pkt, sizeof(pkt), t, afePort, kNullCoppTopology, GUAMP_MIC_RATE); break;
      case AsmOpen: n = asmOpenRead(pkt, sizeof(pkt), t, port(), 16); break;
      case EncCfg: n = asmEncCfgPcm(pkt, sizeof(pkt), t, port(), GUAMP_MIC_RATE); break;
      case Route: n = admRouteTx(pkt, sizeof(pkt), t, session, static_cast<uint16_t>(copp)); break;
      case Run: n = asmRun(pkt, sizeof(pkt), t, port()); break;
      case AsmClose:
        if (!asmOpened) { step = AdmClose; return true; }
        n = asmClose(pkt, sizeof(pkt), t, port());
        break;
      case AdmClose:
        if (!admOpened) { step = MemUnmap; return true; }
        n = admClose(pkt, sizeof(pkt), t, afePort, static_cast<uint16_t>(copp));
        break;
      case MemUnmap:
        if (!mapped) { step = PathOff; return true; }
        n = asmMemUnmap(pkt, sizeof(pkt), t, memHandle);
        break;
      case PathOff:
        if (pathUp) callHw(PathOff);
        pathUp = false;
        step = PortStop;
        return true;
      case PortStop:
        if (!portUp) { step = CodecDown; return true; }
        n = apr::afePort(pkt, sizeof(pkt), t, kAfeDeviceStop, afePort);
        break;
      case CodecDown:
        if (codecTried) callHw(CodecDown);
        codecTried = false;
        codecUp = false;
        step = ClkNplOff;
        return true;
      case ClkNplOff:
        if (!clkNpl) { step = ClkCoreOff; return true; }
        n = afeClock(pkt, sizeof(pkt), t, kClkTxNpl, kClkTxHz, kClkAttrCoupleNo, 0, false);
        break;
      case ClkCoreOff:
        if (!clkCore) { step = DevoteDcodec; return true; }
        n = afeClock(pkt, sizeof(pkt), t, kClkTxCore, kClkTxHz, kClkAttrCoupleNo, 0, false);
        break;
      case DevoteDcodec:
        if (!votedDcodec) { step = DevoteMacro; return true; }
        n = afeHwDevote(pkt, sizeof(pkt), t, kHwBlockDcodec, handleDcodec);
        break;
      case DevoteMacro:
        if (!votedMacro) { step = Done; note("mic sequence done", readDones, setupFailed ? failedStep : 0); return true; }
        n = afeHwDevote(pkt, sizeof(pkt), t, kHwBlockMacro, handleMacro);
        break;
      default: return true;
    }
    return command(now, pkt, n, t);
  }
  void fail(uint32_t status) {
    if (!setupFailed) {
      setupFailed = true;
      failedStep = static_cast<uint32_t>(step);
      failedStatus = status;
    }
    step = AsmClose;   // undo from the top: every undo step checks what exists
  }
  void result(uint32_t now, bool good, uint32_t status) {
    (void)now;
    if (!good) note(status == 0xffffffffu ? "mic: no answer" : "mic: refused", static_cast<uint32_t>(step), status);
    const bool undo = step >= AsmClose;
    if (!good && undo) undoErrors++;
    switch (step) {
      case VoteMacro: votedMacro = good; step = VoteDcodec; break;    // a missing vote is logged, the rest still runs
      case VoteDcodec: votedDcodec = good; step = ClkCore; break;
      case ClkCore: clkCore = good; if (good) step = ClkNpl; else fail(status); break;
      case ClkNpl: clkNpl = good; if (good) step = CodecUp; else fail(status); break;
      case PortConfig:
        if (!good && !configTried && cdcParamBytes != kCdcDmaCfgBytes) {
          configTried = true;
          cdcParamBytes = kCdcDmaCfgBytes;   // once more with the bare 16-byte struct
          note("cdc dma config retry with param size", kCdcDmaCfgBytes, 0);
        } else if (good) {
          step = PortStart;
        } else {
          fail(status);
        }
        break;
      case PortStart: portUp = good; if (good) step = PathOn; else fail(status); break;
      case MemMap: mapped = good; if (good) step = AdmOpen; else fail(status); break;
      case AdmOpen: admOpened = good; if (good) step = AsmOpen; else fail(status); break;
      case AsmOpen: asmOpened = good; if (good) step = EncCfg; else fail(status); break;
      case EncCfg: if (good) step = Route; else fail(status); break;
      case Route: if (good) step = Run; else fail(status); break;
      case Run: if (good) step = Capturing; else fail(status); break;
      case AsmClose: asmOpened = false; step = AdmClose; break;
      case AdmClose: admOpened = false; step = MemUnmap; break;
      case MemUnmap: mapped = false; step = PathOff; break;
      case PortStop: portUp = false; step = CodecDown; break;
      case ClkNplOff: clkNpl = false; step = ClkCoreOff; break;
      case ClkCoreOff: clkCore = false; step = DevoteDcodec; break;
      case DevoteDcodec: votedDcodec = false; step = DevoteMacro; break;
      case DevoteMacro:
        votedMacro = false;
        step = Done;
        note("mic sequence done", readDones, setupFailed ? failedStep : 0);
        break;
      default: break;
    }
    if (stopping && step > PathOn && step < Capturing) fail(0xfffffffdu);   // stop requested during set-up
  }
  // Keep every free slot queued with ASM_DATA_CMD_READ_V2
  void feed() {
    for (uint32_t s = 0; s < kSlots && shm.cpu; s++) {
      if (slotBusy[s]) continue;
      uint8_t pkt[kHdrBytes + 20];
      const uint32_t n = asmRead(pkt, sizeof(pkt), seq, port(), shm.iovaLsw + s * kSlotStride, shm.msw, memHandle,
                                 kPeriodBytes, seq);
      if (!n || !send || send(ctx, pkt, n) < 0) return;
      slotBusy[s] = true;
      inFlight++;
      reads++;
      seq++;
    }
  }
  void readDone(const uint8_t *pl) {
    const uint32_t status = rd32(pl), lsw = rd32(pl + 4), size = rd32(pl + 16), offset = rd32(pl + 20);
    const uint32_t slot = (lsw - shm.iovaLsw) / kSlotStride;
    if (slot >= kSlots || !slotBusy[slot]) {
      unexpected++;
      return;
    }
    slotBusy[slot] = false;
    inFlight--;
    readDones++;
    if (status) {
      readErrors++;
      return;
    }
    if (offset > kSlotStride || size > kSlotStride - offset) {
      readErrors++;
      return;
    }
    const uint8_t *pcm = shm.cpu + slot * kSlotStride + offset;
    const uint32_t bytes = size - size % GUAMP_MIC_FRAME_BYTES;
    for (uint32_t i = 0; i < bytes; i += 2) {
      const int32_t v = static_cast<int16_t>(pcm[i] | (pcm[i + 1] << 8));
      const uint32_t a = static_cast<uint32_t>(v < 0 ? -v : v);
      if (a > peakMax) peakMax = a;
      sumSquares += static_cast<uint64_t>(static_cast<int64_t>(v) * v);
      samples++;
    }
    framesIn += bytes / GUAMP_MIC_FRAME_BYTES;
    if (sink && bytes) sink(ctx, pcm, bytes);
  }
};

}  // namespace apr
#endif
