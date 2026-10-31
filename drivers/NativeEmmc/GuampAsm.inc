// SPDX-License-Identifier: BSD-2-Clause-Patent
// PCM playback through the ADSP (audio milestone 5/6): q6asm stream + q6adm COPP routed to the AFE port that
// apr::AfeTone holds open (useDtmf false, holdPort true), PCM periods written into a shared buffer the ADSP reads
// (ASM_CMD_SHARED_MEM_MAP_REGIONS), refilled on every ASM_DATA_EVENT_WRITE_DONE_V2. apr::Playback runs the whole
// chain (AFE port up -> map -> ADM -> ASM -> route -> format -> run -> stream -> close in reverse -> AFE down) for one
// route, either as a test (generated 1 kHz sine, guamp-adsp-ipc=5) or as the sink of /dev/guamp-audio (ipc 6: up while
// the plug-in plays, down 2 s after it stops). Transport independent like GuampApr.inc / GuampAfe.inc: host-tested by
// tests/audio_pcm_host.cpp; the kernel glue is GuampAdspIpc.inc + GuampAudioShm.inc (buffer, SMMU probe).
// Protocol studied in the guamp kernel techpack/audio (no code copied): dsp/q6asm.c (__q6asm_open_write,
// q6asm_memory_map_regions, q6asm_media_format_block_multi_ch_pcm_v2, q6asm_run, q6asm_write, q6asm_cmd CLOSE),
// dsp/q6adm.c (adm_open, adm_matrix_map, adm_close), asoc/msm-pcm-routing-v2.c, include/dsp/apr_audio-v2.h.
// Sizes from the compiled header (20-byte apr_hdr): adm_cmd_device_open_v5 48, asm_stream_cmd_open_write_v3 36,
// avs_cmd_shared_mem_map_regions 28 + region 12, asm_multi_channel_pcm_fmt_blk_v2 44, asm_session_cmd_run_v2 32,
// asm_data_cmd_write_v2 52, adm_cmd_matrix_map_routings_v5 28 + node 4 + copp list (u16, padded to 4).
//   map     ASM_CMD_SHARED_MEM_MAP_REGIONS 0x10d92 (svc ASM, ports 0) {pool SHMEM8_4K 3, 1 region, property 0 =
//           physical/IOVA} {addr lsw, addr msw, bytes} -> ASM_CMDRSP_SHARED_MEM_MAP_REGIONS 0x10d93 {handle}.
//           msw on bengal = upper 32 bits of the address | SMMU SID bits: msm_audio_ion sid = 0x1c1 & 0xf = 1 -> 1 << 32
//   adm     ADM_CMD_DEVICE_OPEN_V5 0x10326 (svc ADM, ports = AFE port) {flags 0, mode RX 1, endpoint port, 0xffff,
//           NULL_COPP_TOPOLOGY 0x10312, channels, 16 bit, 48000, map {FL 1, FR 2} or {FC 3}} ->
//           ADM_CMDRSP_DEVICE_OPEN_V5 0x10329 {status, copp_id}; close ADM_CMD_DEVICE_CLOSE_V5 0x10327 (src port = AFE
//           port, dest port = copp id)
//   stream  ASM_STREAM_CMD_OPEN_WRITE_V3 0x10db3 (ports session << 8 | stream) {mode_flags 0 (legacy), sink matrix 0,
//           16|24 bit, POPP none 0x10c68, ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2 0x10da5}
//   route   ADM_CMD_MATRIX_MAP_ROUTINGS_V5 0x10325 (ports 0) {matrix AUDIO_RX 0, 1 session} {session, 1 copp} {copp, 0}
//   format  ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2 0x10d98 {fmt_blk_size 20, 2 ch, 16|24 bit, 48000, signed 1, 0, {1, 2}}
//           (24-bit playback samples sit in the top 24 bits of 32-bit words)
//   run     ASM_SESSION_CMD_RUN_V2 0x10daa {flags 0 = now, 0, 0}
//   data    ASM_DATA_CMD_WRITE_V2 0x10dab {addr lsw, msw, handle, size, seq, ts 0 0, flags 0} -> event
//           ASM_DATA_EVENT_WRITE_DONE_V2 0x10d99 {addr lsw, msw, handle, status}
//   stop    ASM_STREAM_CMD_CLOSE 0x10bcd, adm close, ASM_CMD_SHARED_MEM_UNMAP_REGIONS 0x10d94 {handle}
// The ADM/AFE side stays 16-bit (the port's I2S/codec DMA width); the ADSP matrix converts a 24-bit stream.
#ifndef GUAMP_ASM_INC
#define GUAMP_ASM_INC
#include "GuampAfe.inc"
#include "../../package/GuampAudio/guamp_audio_dev.h"
namespace apr {

constexpr uint32_t kAsmMemMap = 0x00010d92, kAsmMemMapRsp = 0x00010d93, kAsmMemUnmap = 0x00010d94;
constexpr uint32_t kAsmOpenWriteV3 = 0x00010db3, kAsmMediaFmtV2 = 0x00010d98, kAsmRunV2 = 0x00010daa;
constexpr uint32_t kAsmWriteV2 = 0x00010dab, kAsmWriteDoneV2 = 0x00010d99, kAsmClose = 0x00010bcd;
constexpr uint32_t kAdmOpenV5 = 0x00010326, kAdmOpenRspV5 = 0x00010329, kAdmCloseV5 = 0x00010327;
constexpr uint32_t kAdmMatrixMapV5 = 0x00010325;
constexpr uint32_t kNullCoppTopology = 0x00010312, kPoppNone = 0x00010c68, kFmtMultiChPcmV2 = 0x00010da5;
constexpr uint16_t kPoolShmem4k = 3;

inline uint16_t asmPort(uint32_t session, uint32_t stream) {
  return static_cast<uint16_t>(((session << 8) & 0xff00) | (stream & 0xff));
}
inline uint32_t asmMemMap(uint8_t *out, uint32_t cap, uint32_t token, uint32_t lsw, uint32_t msw, uint32_t bytes) {
  uint8_t p[20] = {};
  wr16(p, kPoolShmem4k);
  wr16(p + 2, 1);
  wr32(p + 8, lsw);
  wr32(p + 12, msw);
  wr32(p + 16, bytes);
  return build(out, cap, kSvcAsm, 0, token, kAsmMemMap, p, sizeof(p));
}
inline uint32_t asmMemUnmap(uint8_t *out, uint32_t cap, uint32_t token, uint32_t handle) {
  uint8_t p[4];
  wr32(p, handle);
  return build(out, cap, kSvcAsm, 0, token, kAsmMemUnmap, p, sizeof(p));
}
inline uint32_t admOpen(uint8_t *out, uint32_t cap, uint32_t token, uint16_t afePort, uint32_t topology, uint16_t channels,
                        uint16_t bits, uint32_t rate) {
  uint8_t p[28] = {};
  wr16(p + 2, 1);   // ADM_CMD_COPP_OPEN_MODE_OF_OPERATION_RX_PATH_COPP
  wr16(p + 4, afePort);
  wr16(p + 6, 0xffff);
  wr32(p + 8, topology);
  wr16(p + 12, channels);
  wr16(p + 14, bits);
  wr32(p + 16, rate);
  if (channels == 1) p[20] = 3;   // PCM_CHANNEL_FC
  else for (uint32_t c = 0; c < channels && c < 8; c++) p[20 + c] = static_cast<uint8_t>(c + 1);   // FL, FR, ...
  return build(out, cap, kSvcAdm, afePort, token, kAdmOpenV5, p, sizeof(p));
}
inline uint32_t admClose(uint8_t *out, uint32_t cap, uint32_t token, uint16_t afePort, uint16_t copp) {
  return buildPorts(out, cap, kSvcAdm, afePort, copp, token, kAdmCloseV5, nullptr, 0);
}
inline uint32_t admRoute(uint8_t *out, uint32_t cap, uint32_t token, uint16_t session, uint16_t copp) {
  uint8_t p[16] = {};
  wr32(p + 4, 1);   // matrix AUDIO_RX 0, 1 session
  wr16(p + 8, session);
  wr16(p + 10, 1);
  wr16(p + 12, copp);
  return build(out, cap, kSvcAdm, 0, token, kAdmMatrixMapV5, p, sizeof(p));
}
inline uint32_t asmOpenWrite(uint8_t *out, uint32_t cap, uint32_t token, uint16_t port, uint16_t bits) {
  uint8_t p[16] = {};
  wr16(p + 6, bits);   // mode_flags 0, sink matrix 0
  wr32(p + 8, kPoppNone);
  wr32(p + 12, kFmtMultiChPcmV2);
  return build(out, cap, kSvcAsm, port, token, kAsmOpenWriteV3, p, sizeof(p));
}
inline uint32_t asmMediaFmt(uint8_t *out, uint32_t cap, uint32_t token, uint16_t port, uint32_t rate, uint16_t channels,
                            uint16_t bits) {
  uint8_t p[24] = {};
  wr32(p, 20);
  wr16(p + 4, channels);
  wr16(p + 6, bits);
  wr32(p + 8, rate);
  wr16(p + 12, 1);   // signed
  for (uint32_t c = 0; c < channels && c < 8; c++) p[16 + c] = static_cast<uint8_t>(c + 1);
  return build(out, cap, kSvcAsm, port, token, kAsmMediaFmtV2, p, sizeof(p));
}
inline uint32_t asmRun(uint8_t *out, uint32_t cap, uint32_t token, uint16_t port) {
  uint8_t p[12] = {};
  return build(out, cap, kSvcAsm, port, token, kAsmRunV2, p, sizeof(p));
}
inline uint32_t asmWrite(uint8_t *out, uint32_t cap, uint32_t token, uint16_t port, uint32_t lsw, uint32_t msw,
                         uint32_t handle, uint32_t size, uint32_t seq) {
  uint8_t p[32] = {};
  wr32(p, lsw);
  wr32(p + 4, msw);
  wr32(p + 8, handle);
  wr32(p + 12, size);
  wr32(p + 16, seq);
  return build(out, cap, kSvcAsm, port, token, kAsmWriteV2, p, sizeof(p));
}
inline uint32_t asmClose(uint8_t *out, uint32_t cap, uint32_t token, uint16_t port) {
  return build(out, cap, kSvcAsm, port, token, kAsmClose, nullptr, 0);
}

// The shared buffer: kSlots periods at a 4 KiB stride, the CPU view (uncached) and the ADSP view (address lsw + msw)
struct Shm {
  uint8_t *cpu = nullptr;
  uint32_t iovaLsw = 0, msw = 0, bytes = 0;
};

// ---- apps SMMU (0x0C600000) state of the ADSP's HLOS stream 0x1c1, read-only decision -------------------------------
// From the global registers ABL/the hypervisor left: sCR0 (CLIENTPD bit 0, USFCFG bit 10), SMR n (valid bit 31, mask
// 30:16, id 14:0), S2CR n (type 17:16: 0 translate, 1 bypass, 2 fault; CBNDX 7:0). Physical addresses can be handed to
// the ADSP (msw = SID bit 1, as msm_audio_ion) only when that stream bypasses the SMMU.
enum SmmuPath : uint32_t { SmmuDisabled, SmmuBypass, SmmuTranslate, SmmuFault, SmmuUnmatchedBypass, SmmuUnmatchedFault };
struct SmmuView {
  uint32_t path = SmmuUnmatchedFault, slot = 0xffffffffu, smr = 0, s2cr = 0, cb = 0;
  bool usable() const { return path == SmmuDisabled || path == SmmuBypass || path == SmmuUnmatchedBypass; }
};
inline const char *smmuPathName(uint32_t p) {
  static const char *const n[] = {"smmu disabled (CLIENTPD)", "S2CR bypass", "S2CR translate (context bank)",
                                  "S2CR fault", "no SMR, unmatched streams bypass", "no SMR, unmatched streams fault"};
  return p <= SmmuUnmatchedFault ? n[p] : "?";
}
inline SmmuView smmuDecide(uint32_t scr0, const uint32_t *smr, const uint32_t *s2cr, uint32_t groups, uint32_t stream) {
  SmmuView v;
  if (scr0 & 1u) {
    v.path = SmmuDisabled;
    return v;
  }
  for (uint32_t i = 0; i < groups; i++) {
    if (!(smr[i] & (1u << 31))) continue;
    const uint32_t mask = (smr[i] >> 16) & 0x7fffu, id = smr[i] & 0x7fffu;
    if (((id ^ stream) & ~mask & 0x7fffu) != 0) continue;
    v.slot = i;
    v.smr = smr[i];
    v.s2cr = s2cr[i];
    v.cb = s2cr[i] & 0xffu;
    const uint32_t type = (s2cr[i] >> 16) & 3u;
    v.path = type == 0 ? SmmuTranslate : type == 1 ? SmmuBypass : SmmuFault;
    return v;
  }
  v.path = (scr0 & (1u << 10)) ? SmmuUnmatchedFault : SmmuUnmatchedBypass;
  return v;
}

// One ASM/ADM session. The owner brings the AFE port up first (AfeTone hold mode), then calls begin(); feeds ASM/ADM
// packets to receive(); steps with a ms clock; stop() closes everything in reverse.
struct PcmPath {
  enum Step : int { MemMap, AdmOpen, AsmOpen, Route, MediaFmt, Run, Streaming, AsmClose, AdmClose, MemUnmap, Done };
  static constexpr uint32_t kSlots = 8, kSlotStride = 4096, kAnswerMs = 1000;   // 8 x 10 ms in flight
  static constexpr uint32_t kFrames = GUAMP_AUDIO_PERIOD_FRAMES;
  void *ctx = nullptr;
  int (*send)(void *, const uint8_t *, uint32_t) = nullptr;
  void (*log)(void *, const char *what, uint32_t a, uint32_t b) = nullptr;
  // kFrames stereo frames into dst as `bits`-bit samples (16: s16, 24: s24 in s32), silence-padded; returns real frames
  uint32_t (*fill)(void *, uint8_t *dst, uint32_t frames, uint32_t bits) = nullptr;
  Shm shm;
  uint16_t afePort = kPortPriMi2sRx, session = 1, stream = 1;
  uint16_t bits = 16, devChannels = 2;   // ASM stream width (16|24), COPP/port channels (1|2)
  int step = MemMap;
  bool waiting = false, stopping = false, failed = false, mapped = false, admOpened = false, asmOpened = false;
  uint32_t token = 0, waitOpcode = 0, deadline = 0, handle = 0, copp = 0, failedStep = 0, failedStatus = 0;
  uint32_t seq = 0, inFlight = 0, writes = 0, writeDones = 0, writeErrors = 0, lastError = 0, unexpected = 0;
  uint32_t sendBusy = 0, maxLateMs = 0, lastDoneAt = 0, blockedAt = 0;
  bool slotBusy[kSlots] = {};

  bool finished() const { return step == Done; }
  bool streaming() const { return step == Streaming; }
  uint16_t port() const { return asmPort(session, stream); }
  uint32_t periodBytes() const { return kFrames * 2 * (bits == 24 ? 4u : 2u); }   // 1920 or 3840, inside the stride
  static const char *stepName(int s) {
    static const char *const n[] = {"mem map", "adm open", "asm open", "matrix route", "media format", "run",
                                    "streaming", "asm close", "adm close", "mem unmap", "done"};
    return s >= 0 && s <= Done ? n[s] : "?";
  }
  void begin(uint32_t now) {
    step = MemMap;
    deadline = now;
    note("pcm begin", (static_cast<uint32_t>(afePort) << 16) | bits, shm.iovaLsw);
  }
  void stop() { stopping = true; }

  void stepAt(uint32_t now) {
    for (unsigned guard = 0; guard < 16 && step != Done; guard++) {
      if (waiting) {
        if (now < deadline) return;
        waiting = false;
        note("pcm TIMEOUT: no answer in 1 s to opcode", waitOpcode, static_cast<uint32_t>(step));
        result(false, 0xffffffffu);
        continue;
      }
      if (step == Streaming) {
        if (stopping && !inFlight) {
          note("pcm drained", writes, writeDones);
          step = AsmClose;
          continue;
        }
        if (!stopping) feed(now);
        return;
      }
      if (!issue(now)) {   // channel busy (no GLINK intent): next pass, but not forever
        if (!blockedAt) blockedAt = now + 1;   // +1: 0 means "not blocked"
        if (now + 1 - blockedAt < kAnswerMs) return;
        blockedAt = 0;
        note("pcm SEND BLOCKED for 1 s at step", static_cast<uint32_t>(step), 0);
        result(false, 0xfffffffeu);
        continue;
      }
      blockedAt = 0;
      if (waiting) return;
    }
  }

  void receive(const uint8_t *p, uint32_t n, uint32_t now) {
    Header h;
    const uint8_t *pl = nullptr;
    uint32_t pn = 0;
    if (!parse(p, n, &h, &pl, &pn) || (h.srcSvc != kSvcAsm && h.srcSvc != kSvcAdm)) {
      unexpected++;
      return;
    }
    if (h.opcode == kAsmWriteDoneV2 && pn >= 16) {   // the callback: one period consumed by the ADSP
      const uint32_t slot = (rd32(pl) - shm.iovaLsw) / kSlotStride;
      if (slot < kSlots && slotBusy[slot] && rd32(pl + 8) == handle) {
        slotBusy[slot] = false;
        inFlight--;
        writeDones++;
        if (writeDones > 1 && now - lastDoneAt > maxLateMs) maxLateMs = now - lastDoneAt;
        lastDoneAt = now;
        if (rd32(pl + 12)) {
          if (!writeErrors) note("pcm write done error", slot, rd32(pl + 12));
          writeErrors++;
          lastError = rd32(pl + 12);
        }
        if (writeDones == 1) note("pcm first write done", slot, now);
      } else {
        unexpected++;
        note("pcm write done unknown buffer", rd32(pl), rd32(pl + 8));
      }
      return stepAt(now);
    }
    if (!waiting || h.token != token) {
      unexpected++;
      note("pcm unexpected", h.opcode, h.token);
      return;
    }
    if (h.opcode == kAsmMemMapRsp && waitOpcode == kAsmMemMap && pn >= 4) {
      handle = rd32(pl);
      waiting = false;
      note("pcm mem map handle", handle, shm.msw);
      result(true, 0);
    } else if (h.opcode == kAdmOpenRspV5 && waitOpcode == kAdmOpenV5 && pn >= 8) {
      copp = rd32(pl + 4);
      waiting = false;
      note("pcm adm copp", copp, rd32(pl));
      result(rd32(pl) == 0, rd32(pl));
    } else if (h.opcode == kBasicResult && pn >= 8 && rd32(pl) == waitOpcode) {
      waiting = false;
      result(rd32(pl + 4) == 0, rd32(pl + 4));
    } else {
      unexpected++;
      note("pcm unexpected", h.opcode, pn);
      return;
    }
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
    return true;
  }
  bool issue(uint32_t now) {
    uint8_t pkt[kMaxPacket];
    uint32_t n = 0;
    const uint32_t t = token + 1;
    switch (step) {
      case MemMap: n = asmMemMap(pkt, sizeof(pkt), t, shm.iovaLsw, shm.msw, shm.bytes); break;
      case AdmOpen: n = admOpen(pkt, sizeof(pkt), t, afePort, kNullCoppTopology, devChannels, 16, GUAMP_AUDIO_RATE); break;
      case AsmOpen: n = asmOpenWrite(pkt, sizeof(pkt), t, port(), bits); break;
      case Route: n = admRoute(pkt, sizeof(pkt), t, session, static_cast<uint16_t>(copp)); break;
      case MediaFmt: n = asmMediaFmt(pkt, sizeof(pkt), t, port(), GUAMP_AUDIO_RATE, 2, bits); break;
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
        if (!mapped) { step = Done; note("pcm closed", writes, writeDones); return true; }
        n = asmMemUnmap(pkt, sizeof(pkt), t, handle);
        break;
      default: return true;
    }
    if (!command(now, pkt, n, t)) return false;
    note(stepName(step), t, waitOpcode);
    return true;
  }
  void result(bool good, uint32_t status) {
    if (!good) note(status == 0xffffffffu ? "pcm no answer" : "pcm refused", static_cast<uint32_t>(step), status);
    const bool setup = step < Streaming;
    if (!good && setup && !failed) {
      failed = true;
      failedStep = static_cast<uint32_t>(step);
      failedStatus = status;
    }
    switch (step) {
      case MemMap: mapped = good; break;
      case AdmOpen: admOpened = good; break;
      case AsmOpen: asmOpened = good; break;
      case AsmClose: asmOpened = false; break;
      case AdmClose: admOpened = false; break;
      case MemUnmap: mapped = false; break;
      default: break;
    }
    if (setup && (!good || stopping)) step = AsmClose;   // undo what exists
    else if (step == MemUnmap) { step = Done; note("pcm closed", writes, writeDones); }
    else {
      step++;
      if (step == Streaming) note("pcm streaming", kSlots, periodBytes());
    }
  }
  // Keep every free slot queued: fill it (silence when short), then ASM_DATA_CMD_WRITE_V2
  // A slot filled for a write the channel refused (busy) keeps its period and is sent as it is on the next pass: refilling it
  // took a new period out of the ring and threw the first away (fin23: 917 writes, 1295 periods taken, 12 gaps in 7 s).
  bool slotFilled[kSlots] = {};
  uint32_t resent = 0;
  void feed(uint32_t now) {
    for (uint32_t s = 0; s < kSlots && shm.cpu; s++) {
      if (slotBusy[s]) continue;
      uint8_t *dst = shm.cpu + s * kSlotStride;
      if (!slotFilled[s]) {
        if (fill) fill(ctx, dst, kFrames, bits);
        slotFilled[s] = true;
      } else {
        resent++;
      }
      uint8_t pkt[kHdrBytes + 32];
      const uint32_t n = asmWrite(pkt, sizeof(pkt), seq, port(), shm.iovaLsw + s * kSlotStride, shm.msw, handle,
                                  periodBytes(), seq);
      if (!n || !send || send(ctx, pkt, n) < 0) {   // channel busy (no remote intent): next pass
        sendBusy++;
        return;
      }
      slotBusy[s] = true;
      slotFilled[s] = false;
      inFlight++;
      writes++;
      seq++;
      if (writes == 1) note("pcm first write", s, now);
    }
  }
};

// The whole playback chain for one route, as a test (Mode Test: 1 kHz sine for testMs, then down, verdict) or as the
// /dev/guamp-audio sink (Mode Sink: up when wanted(), down lingerMs after it stops being wanted or when the route
// changes; again on the next demand).
struct Playback {
  enum Mode : int { Test, Sink };
  enum State : int { Idle, PortUp, PathUp, Streaming, PathDown, PortDown, Finished };
  struct Hooks {
    void *ctx = nullptr;
    int (*send)(void *, const uint8_t *, uint32_t) = nullptr;
    void (*log)(void *, const char *what, uint32_t a, uint32_t b) = nullptr;
    // GuampAudioHw.h for this route: step = AfeTone::HwPrepare / AmpOn / AmpOff / HwRelease
    int (*hw)(void *, uint32_t route, uint16_t afePort, uint32_t channels, int step, bool on) = nullptr;
    // Sink mode: the device ring (pull) and whether the plug-in plays (route out); attach: ring consumer on/off
    uint32_t (*source)(void *, uint8_t *dst, uint32_t frames, uint32_t bits) = nullptr;
    bool (*wanted)(void *, uint32_t *route) = nullptr;
    void (*attach)(void *, bool on) = nullptr;
  };
  Hooks hooks;
  int mode = Test, state = Idle;
  uint32_t route = GUAMP_AUDIO_DEV_ROUTE_SPEAKER, bits = 16, testMs = 3000, lingerMs = 2000, retryMs = 5000;
  Shm shm;
  AfeTone afe;
  PcmPath pcm;
  uint32_t stopAt = 0, idleSince = 0, retryAt = 0, phase = 0, nextReport = 0, sessions = 0, failures = 0;
  bool lastOk = false, attached = false;

  static uint16_t portOf(uint32_t r) { return r == GUAMP_AUDIO_DEV_ROUTE_SPEAKER ? kPortPriMi2sRx : kPortRxCdcDma0; }
  static uint32_t channelsOf(uint32_t r) { return r == GUAMP_AUDIO_DEV_ROUTE_EARPIECE ? 1 : 2; }
  static const char *stateName(int s) {
    static const char *const n[] = {"idle", "port up", "path up", "streaming", "path down", "port down", "finished"};
    return s >= 0 && s <= Finished ? n[s] : "?";
  }
  bool finished() const { return state == Finished; }
  bool active() const { return state != Idle && state != Finished; }

  // APR packets of AFE, ASM and ADM; false if not ours
  bool receive(const uint8_t *p, uint32_t n, uint32_t now) {
    Header h;
    const uint8_t *pl = nullptr;
    uint32_t pn = 0;
    if (!parse(p, n, &h, &pl, &pn)) return false;
    if (h.srcSvc == kSvcAfe && (state == PortUp || state == PortDown || state == PathUp || state == PathDown ||
                                state == Streaming)) {
      afe.receive(p, n, now);
      return true;
    }
    if (h.srcSvc == kSvcAsm || h.srcSvc == kSvcAdm) {
      if (state == PathUp || state == Streaming || state == PathDown) pcm.receive(p, n, now);
      return true;
    }
    return false;
  }

  void stepAt(uint32_t now) {
    switch (state) {
      case Idle: {
        uint32_t want = route;
        if (mode == Sink && (!hooks.wanted || !hooks.wanted(hooks.ctx, &want) || now < retryAt)) return;
        if (want >= GUAMP_AUDIO_DEV_ROUTE_SPEAKER && want <= GUAMP_AUDIO_DEV_ROUTE_HEADPHONE) route = want;
        startPort(now);
        return;
      }
      case PortUp:
        afe.stepAt(now);
        if (afe.portRunning()) {
          pcm = PcmPath{};
          pcm.ctx = this;
          pcm.send = sendThunk;
          pcm.log = logThunk;
          pcm.fill = fillThunk;
          pcm.shm = shm;
          pcm.afePort = portOf(route);
          pcm.bits = static_cast<uint16_t>(bits);
          pcm.devChannels = static_cast<uint16_t>(channelsOf(route));
          state = PathUp;
          note("play path up", route, now);
          pcm.begin(now);
          pcm.stepAt(now);
        } else if (afe.finished()) {
          endSession(now, false);
        }
        return;
      case PathUp:
        pcm.stepAt(now);
        afe.stepAt(now);
        if (pcm.streaming()) {
          state = Streaming;
          stopAt = now + testMs;
          idleSince = 0;
          nextReport = now + 1000;
          if (mode == Sink && hooks.attach) hooks.attach(hooks.ctx, true);
          attached = mode == Sink;
          note("play streaming", route, bits);
        } else if (pcm.finished()) {
          afe.release();
          state = PortDown;
          note("play port down", route, pcm.failedStep);
        }
        return;
      case Streaming: {
        pcm.stepAt(now);
        afe.stepAt(now);
        if (now >= nextReport) {
          nextReport = now + 1000;
          note("play stats", pcm.writeDones, (pcm.writeErrors << 16) | (pcm.maxLateMs & 0xffff));
        }
        bool down = false;
        if (mode == Test) {
          down = now >= stopAt;
        } else {
          uint32_t want = route;
          const bool w = hooks.wanted && hooks.wanted(hooks.ctx, &want);
          if (w && want != route) {
            note("play route change", route, want);
            down = true;
          } else if (w) {
            idleSince = 0;
          } else if (!idleSince) {
            idleSince = now ? now : 1;
          } else if (now - idleSince >= lingerMs) {
            down = true;
          }
        }
        if (down || pcm.finished()) {
          detach();
          pcm.stop();
          state = PathDown;
          note("play path down", pcm.writes, pcm.writeDones);
        }
        return;
      }
      case PathDown:
        pcm.stepAt(now);
        afe.stepAt(now);
        if (pcm.finished()) {
          afe.release();
          state = PortDown;
          note("play port down", route, pcm.failed ? pcm.failedStep : 0);
        }
        return;
      case PortDown:
        afe.stepAt(now);
        if (afe.finished()) endSession(now, true);
        return;
      default: return;
    }
  }

  // The verdict of the last session
  bool ok() const { return lastOk; }

private:
  static int sendThunk(void *c, const uint8_t *p, uint32_t n) {
    Playback *b = static_cast<Playback *>(c);
    return b->hooks.send ? b->hooks.send(b->hooks.ctx, p, n) : -1;
  }
  static void logThunk(void *c, const char *what, uint32_t a, uint32_t b) {
    Playback *pb = static_cast<Playback *>(c);
    if (pb->hooks.log) pb->hooks.log(pb->hooks.ctx, what, a, b);
  }
  static int hwThunk(void *c, int step, bool on) {
    Playback *b = static_cast<Playback *>(c);
    return b->hooks.hw ? b->hooks.hw(b->hooks.ctx, b->route, portOf(b->route), channelsOf(b->route), step, on) : -1;
  }
  static uint32_t fillThunk(void *c, uint8_t *dst, uint32_t frames, uint32_t bits) {
    Playback *b = static_cast<Playback *>(c);
    if (b->mode == Sink) {
      if (b->hooks.source) return b->hooks.source(b->hooks.ctx, dst, frames, bits);
      for (uint32_t i = 0; i < frames * 2 * (bits == 24 ? 4u : 2u); i++) dst[i] = 0;
      return 0;
    }
    return b->sine(dst, frames, bits);
  }
  // 1 kHz, -12 dBFS, both channels (48 samples per cycle at 48 kHz)
  uint32_t sine(uint8_t *dst, uint32_t frames, uint32_t bits) {
    static const int16_t kTable[48] = {0,     1069,  2120,  3135,  4096,  4987,  5793,  6499,  7094,  7568,
                                       7913,  8122,  8192,  8122,  7913,  7568,  7094,  6499,  5793,  4987,
                                       4096,  3135,  2120,  1069,  0,     -1069, -2120, -3135, -4096, -4987,
                                       -5793, -6499, -7094, -7568, -7913, -8122, -8192, -8122, -7913, -7568,
                                       -7094, -6499, -5793, -4987, -4096, -3135, -2120, -1069};
    const uint32_t width = bits == 24 ? 4 : 2;
    for (uint32_t f = 0; f < frames; f++) {
      const int32_t v = kTable[phase];
      phase = (phase + 1) % 48;
      const uint32_t out = width == 4 ? static_cast<uint32_t>(v) << 16 : static_cast<uint32_t>(v);
      for (uint32_t ch = 0; ch < 2; ch++)
        for (uint32_t b = 0; b < width; b++) dst[(f * 2 + ch) * width + b] = static_cast<uint8_t>(out >> (8 * b));
    }
    return frames;
  }
  void note(const char *what, uint32_t a, uint32_t b) {
    if (hooks.log) hooks.log(hooks.ctx, what, a, b);
  }
  void detach() {
    if (attached && hooks.attach) hooks.attach(hooks.ctx, false);
    attached = false;
  }
  void startPort(uint32_t now) {
    afe = AfeTone{};
    afe.ctx = this;
    afe.send = sendThunk;
    afe.log = logThunk;
    afe.hw = hwThunk;
    afe.useDtmf = false;
    afe.holdPort = true;
    afe.port.port = portOf(route);
    afe.port.cdcDma = route != GUAMP_AUDIO_DEV_ROUTE_SPEAKER;
    afe.port.stereo = channelsOf(route) == 2 ? 1 : 0;
    pcm = PcmPath{};
    state = PortUp;
    note("play port up", route, afe.port.port);
    afe.begin(now);
    afe.stepAt(now);
  }
  void endSession(uint32_t now, bool reachedPath) {
    detach();
    sessions++;
    lastOk = reachedPath && !afe.setupFailed && !pcm.failed && pcm.writeDones > 0 && !pcm.writeErrors;
    if (!lastOk) failures++;
    note(lastOk ? "play session ok" : "play session failed", sessions, failures);
    if (mode == Test) {
      state = Finished;
    } else {
      state = Idle;
      retryAt = lastOk ? now : now + retryMs;
    }
  }
};

}  // namespace apr
#endif
