// SPDX-License-Identifier: BSD-2-Clause-Patent
// PCM playback through the ADSP (audio milestone 5, SKELETON: builders and state machine host-tested, not wired to a
// boot argument yet): q6asm stream + q6adm COPP routed to the AFE port that apr::AfeTone holds open (useDtmf false,
// holdPort true), PCM periods taken from the /dev/guamp-audio ring (GuampAudioRing.inc) into a buffer the ADSP reads
// through the apps SMMU. Transport independent like GuampApr.inc / GuampAfe.inc.
// Protocol studied in the guamp kernel techpack/audio (no code copied): dsp/q6asm.c (__q6asm_open_write,
// q6asm_memory_map_regions, q6asm_media_format_block_multi_ch_pcm_v2, q6asm_run, q6asm_write, q6asm_cmd CLOSE),
// dsp/q6adm.c (adm_open, adm_matrix_map, adm_close), asoc/msm-pcm-routing-v2.c, include/dsp/apr_audio-v2.h.
// Sizes from the compiled header (20-byte apr_hdr): adm_cmd_device_open_v5 48, asm_stream_cmd_open_write_v3 36,
// avs_cmd_shared_mem_map_regions 28 + region 12, asm_multi_channel_pcm_fmt_blk_v2 44, asm_session_cmd_run_v2 32,
// asm_data_cmd_write_v2 52, adm_cmd_matrix_map_routings_v5 28 + node 4 + copp list (u16, padded to 4).
//   map     ASM_CMD_SHARED_MEM_MAP_REGIONS 0x10d92 (svc ASM, ports 0) {pool SHMEM8_4K 3, 1 region, property 0 =
//           physical/IOVA} {addr lsw, addr msw, bytes} -> ASM_CMDRSP_SHARED_MEM_MAP_REGIONS 0x10d93 {handle}.
//           msw on bengal = upper 32 bits of the IOVA | SMMU SID bits: msm_audio_ion sid = 0x1c1 & 0xf = 1 -> 1 << 32,
//           so msw = 1 for an IOVA in qcom,iommu-dma-addr-pool 0x10000000..0x1fffffff.
//   adm     ADM_CMD_DEVICE_OPEN_V5 0x10326 (svc ADM, ports = AFE port) {flags 0, mode RX 1, endpoint 0x1000, 0xffff,
//           NULL_COPP_TOPOLOGY 0x10312, 2 ch, 16 bit, 48000, map {FL 1, FR 2}} -> ADM_CMDRSP_DEVICE_OPEN_V5 0x10329
//           {status, copp_id}; close ADM_CMD_DEVICE_CLOSE_V5 0x10327 (src port = AFE port, dest port = copp id)
//   stream  ASM_STREAM_CMD_OPEN_WRITE_V3 0x10db3 (ports session << 8 | stream) {mode_flags 0 (legacy), sink matrix 0,
//           16 bit, POPP none 0x10c68, ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2 0x10da5}
//   route   ADM_CMD_MATRIX_MAP_ROUTINGS_V5 0x10325 (ports 0) {matrix AUDIO_RX 0, 1 session} {session, 1 copp} {copp, 0}
//   format  ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2 0x10d98 {fmt_blk_size 20, 2 ch, 16 bit, 48000, signed 1, 0, {1, 2, 0..}}
//   run     ASM_SESSION_CMD_RUN_V2 0x10daa {flags 0 = now, 0, 0}
//   data    ASM_DATA_CMD_WRITE_V2 0x10dab {addr lsw, msw, handle, size, seq, ts 0 0, flags 0} -> event
//           ASM_DATA_EVENT_WRITE_DONE_V2 0x10d99 {addr lsw, msw, handle, status}
//   stop    ASM_STREAM_CMD_CLOSE 0x10bcd, adm close, ASM_CMD_SHARED_MEM_UNMAP_REGIONS 0x10d94 {handle}
// Memory (ADSP-FASE2 §2.2, AUDIO-DSP-20261028.md §4): the buffer must be XNU-owned, physically contiguous, mapped
// uncached on the CPU side, and visible to the ADSP through the apps SMMU (0x0C600000) context bank of stream 0x1c1
// at an IOVA in 0x10000000..0x1fffffff; Shm carries both views. Nothing here touches the SMMU.
#ifndef GUAMP_ASM_INC
#define GUAMP_ASM_INC
#include "GuampAfe.inc"
#include "guamp_audio_dev.h"
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
  for (uint32_t c = 0; c < channels && c < 8; c++) p[20 + c] = static_cast<uint8_t>(c + 1);   // FL, FR, ...
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

// The shared buffer: kSlots periods at a 4 KiB stride, the CPU view (uncached) and the ADSP view (IOVA + msw)
struct Shm {
  uint8_t *cpu = nullptr;
  uint32_t iovaLsw = 0, msw = 0, bytes = 0;
};

// One playback session. The owner: brings the AFE port up first (AfeTone hold mode) and sets portUp, then calls
// begin(); feeds ASM/ADM packets to receive(); steps with a ms clock; stop() closes everything in reverse.
struct PcmPath {
  enum Step : int { MemMap, AdmOpen, AsmOpen, Route, MediaFmt, Run, Streaming, AsmClose, AdmClose, MemUnmap, Done };
  static constexpr uint32_t kSlots = 4, kSlotStride = 4096, kAnswerMs = 1000;
  static constexpr uint32_t kPeriodBytes = GUAMP_AUDIO_PERIOD_FRAMES * GUAMP_AUDIO_FRAME_BYTES;   // 1920
  void *ctx = nullptr;
  int (*send)(void *, const uint8_t *, uint32_t) = nullptr;
  void (*log)(void *, const char *what, uint32_t a, uint32_t b) = nullptr;
  uint32_t (*fill)(void *, uint8_t *dst, uint32_t n) = nullptr;   // GuampAudioRing pull(): pads with silence
  Shm shm;
  uint16_t afePort = kPortPriMi2sRx, session = 1, stream = 1;
  int step = MemMap;
  bool waiting = false, stopping = false, failed = false, mapped = false, admOpened = false, asmOpened = false;
  uint32_t token = 0, waitOpcode = 0, deadline = 0, handle = 0, copp = 0, failedStep = 0, failedStatus = 0;
  uint32_t seq = 0, inFlight = 0, writes = 0, writeDones = 0, writeErrors = 0, unexpected = 0;
  bool slotBusy[kSlots] = {};

  bool finished() const { return step == Done; }
  bool streaming() const { return step == Streaming; }
  uint16_t port() const { return asmPort(session, stream); }
  void begin(uint32_t now) {
    step = MemMap;
    deadline = now;
  }
  void stop() { stopping = true; }

  void stepAt(uint32_t now) {
    for (unsigned guard = 0; guard < 16 && step != Done; guard++) {
      if (waiting) {
        if (now < deadline) return;
        waiting = false;
        result(false, 0xffffffffu);
        continue;
      }
      if (step == Streaming) {
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
    if (!parse(p, n, &h, &pl, &pn) || (h.srcSvc != kSvcAsm && h.srcSvc != kSvcAdm)) {
      unexpected++;
      return;
    }
    if (h.opcode == kAsmWriteDoneV2 && pn >= 16) {
      const uint32_t slot = (rd32(pl) - shm.iovaLsw) / kSlotStride;
      if (slot < kSlots && slotBusy[slot]) {
        slotBusy[slot] = false;
        inFlight--;
        writeDones++;
        if (rd32(pl + 12)) writeErrors++;
      } else {
        unexpected++;
      }
      return stepAt(now);
    }
    if (!waiting || h.token != token) {
      unexpected++;
      return;
    }
    if (h.opcode == kAsmMemMapRsp && waitOpcode == kAsmMemMap && pn >= 4) {
      handle = rd32(pl);
      waiting = false;
      result(true, 0);
    } else if (h.opcode == kAdmOpenRspV5 && waitOpcode == kAdmOpenV5 && pn >= 8) {
      copp = rd32(pl + 4);
      waiting = false;
      result(rd32(pl) == 0, rd32(pl));
    } else if (h.opcode == kBasicResult && pn >= 8 && rd32(pl) == waitOpcode) {
      waiting = false;
      result(rd32(pl + 4) == 0, rd32(pl + 4));
    } else {
      unexpected++;
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
      case AdmOpen: n = admOpen(pkt, sizeof(pkt), t, afePort, kNullCoppTopology, 2, 16, GUAMP_AUDIO_RATE); break;
      case AsmOpen: n = asmOpenWrite(pkt, sizeof(pkt), t, port(), 16); break;
      case Route: n = admRoute(pkt, sizeof(pkt), t, session, static_cast<uint16_t>(copp)); break;
      case MediaFmt: n = asmMediaFmt(pkt, sizeof(pkt), t, port(), GUAMP_AUDIO_RATE, 2, 16); break;
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
        if (!mapped) { step = Done; return true; }
        n = asmMemUnmap(pkt, sizeof(pkt), t, handle);
        break;
      default: return true;
    }
    if (!command(now, pkt, n, t)) return false;
    note("pcm command", static_cast<uint32_t>(step), waitOpcode);
    return true;
  }
  void result(bool good, uint32_t status) {
    if (!good) note("pcm refused or no answer", static_cast<uint32_t>(step), status);
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
    else if (step == MemUnmap) step = Done;
    else step++;
  }
  // Keep every free slot queued: fill it from the ring (silence when short), then ASM_DATA_CMD_WRITE_V2
  void feed() {
    for (uint32_t s = 0; s < kSlots && shm.cpu; s++) {
      if (slotBusy[s]) continue;
      uint8_t *dst = shm.cpu + s * kSlotStride;
      if (fill) fill(ctx, dst, kPeriodBytes);
      uint8_t pkt[kHdrBytes + 32];
      const uint32_t n = asmWrite(pkt, sizeof(pkt), seq, port(), shm.iovaLsw + s * kSlotStride, shm.msw, handle,
                                  kPeriodBytes, seq);
      if (!n || !send || send(ctx, pkt, n) < 0) return;   // channel busy: next pass
      slotBusy[s] = true;
      inFlight++;
      writes++;
      seq++;
    }
  }
};

}  // namespace apr
#endif
