// SPDX-License-Identifier: BSD-2-Clause-Patent
// APR (Asynchronous Packet Router, msm-audio APR v2 - not GPR/AudioReach) packets and the first q6core client of the
// guamp ADSP, audio phase 2. Transport independent: the owner gives a send() over the GLINK channel "apr_audio_svc"
// and feeds every received message to receive(); time is passed in (ms) so tests/adsp_ipc_host.cpp can drive it.
// Protocol studied in the guamp kernel techpack/audio (no code copied): include/ipc/apr.h, ipc/apr.c apr_send_pkt,
// ipc/apr_tal_rpmsg.c, dsp/q6core.c, include/dsp/q6core.h, include/dsp/apr_audio-v2.h:
//   header (struct apr_hdr, 20 bytes, little endian): hdr_field = msg_type << 8 | hdr_len_words << 4 | version
//   (APR_HDR_FIELD(SEQ_CMD 2, APR_HDR_LEN(APR_HDR_SIZE) = 5 words, v0) = 0x0250), pkt_size (header included),
//   src_svc, src_domain, src_port, dest_svc, dest_domain, dest_port, token, opcode. apr_cb_func accepts a header of
//   5 words or more and version 0 or 1, so parse() does the same. (Phase 2 sent 8 words / 0x0280 with 12 undefined
//   bytes and refused the ADSP's 20-byte headers: fixed 28/10, checked against sizeof(struct apr_hdr) = 20.)
//   apr_send_pkt fills src_domain APPS (5), src_svc = dest_svc = the client's service (q6core: ADSP_CORE 3),
//   dest_domain ADSP (4); q6core uses ports 0. Responses come back with src/dest swapped.
//   q6core_is_adsp_ready: AVCS_CMD_ADSP_EVENT_GET_STATE 0x1290c (no payload) -> AVCS_CMDRSP_ADSP_EVENT_GET_STATE
//   0x1290d {state}, state != 0 = ready; one try waits 100 ms (Q6_READY_TIMEOUT_MS), tries every 50 ms for
//   ADSP_STATE_READY_TIMEOUT_MS (3 s; 10 s here). q6core_send_get_avcs_fwk_ver_cmd: AVCS_CMD_GET_FWK_VERSION 0x1292c
//   -> AVCS_CMDRSP_GET_FWK_VERSION 0x1292d {build major, minor, branch, subbranch, num_services} + num_services x
//   {service_id, api_version, api_branch_version}; waits 1 s (TIMEOUT_MS). Errors: APR_BASIC_RSP_RESULT 0x110e8
//   {opcode, status} (status 0 = success; ADSP_EUNSUPPORTED etc. otherwise).
#ifndef GUAMP_APR_INC
#define GUAMP_APR_INC
namespace apr {

constexpr uint16_t kHdrField = (2u << 8) | (5u << 4);   // SEQ_CMD, 20-byte header, version 0
constexpr uint32_t kHdrBytes = 20, kMaxPacket = 512;     // sizeof(struct apr_hdr); APR_MAX_BUF of apr_tal_rpmsg.c
constexpr uint8_t kDomainAdsp = 4, kDomainApps = 5, kSvcCore = 3, kSvcAfe = 4, kSvcAsm = 7, kSvcAdm = 8;
constexpr uint32_t kBasicResult = 0x000110e8, kGetState = 0x0001290c, kGetStateRsp = 0x0001290d;
constexpr uint32_t kGetFwk = 0x0001292c, kGetFwkRsp = 0x0001292d;

struct Header {
  uint16_t hdrField, pktSize;
  uint8_t srcSvc, srcDomain;
  uint16_t srcPort;
  uint8_t destSvc, destDomain;
  uint16_t destPort;
  uint32_t token, opcode;
};

inline uint16_t rd16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline uint32_t rd32(const uint8_t *p) { return rd16(p) | (static_cast<uint32_t>(rd16(p + 2)) << 16); }
inline void wr16(uint8_t *p, uint16_t v) { p[0] = static_cast<uint8_t>(v); p[1] = static_cast<uint8_t>(v >> 8); }
inline void wr32(uint8_t *p, uint32_t v) { wr16(p, static_cast<uint16_t>(v)); wr16(p + 2, static_cast<uint16_t>(v >> 16)); }

// A command from apps to an ADSP service (apr_send_pkt); returns the packet size, 0 if it does not fit. The port is
// both src_port and dest_port (q6core/q6afe 0, q6asm session << 8 | stream, q6adm the AFE port id).
inline uint32_t build(uint8_t *out, uint32_t cap, uint8_t svc, uint16_t port, uint32_t token, uint32_t opcode,
                      const uint8_t *payload, uint32_t n) {
  if (cap < kHdrBytes || n > cap - kHdrBytes || n > kMaxPacket - kHdrBytes || (n & 3)) return 0;
  wr16(out, kHdrField);
  wr16(out + 2, static_cast<uint16_t>(kHdrBytes + n));
  out[4] = svc;
  out[5] = kDomainApps;
  wr16(out + 6, port);
  out[8] = svc;
  out[9] = kDomainAdsp;
  wr16(out + 10, port);
  wr32(out + 12, token);
  wr32(out + 16, opcode);
  for (uint32_t i = 0; i < n; i++) out[kHdrBytes + i] = payload[i];
  return kHdrBytes + n;
}

// apr_cb_func checks: header length >= 5 words and inside the packet, pkt_size inside the message, version <= 1
inline bool parse(const uint8_t *p, uint32_t n, Header *h, const uint8_t **payload, uint32_t *payloadBytes) {
  if (n < kHdrBytes) return false;
  h->hdrField = rd16(p);
  h->pktSize = rd16(p + 2);
  h->srcSvc = p[4];
  h->srcDomain = p[5];
  h->srcPort = rd16(p + 6);
  h->destSvc = p[8];
  h->destDomain = p[9];
  h->destPort = rd16(p + 10);
  h->token = rd32(p + 12);
  h->opcode = rd32(p + 16);
  const uint32_t hdrLen = ((h->hdrField >> 4) & 0xf) * 4;
  if ((h->hdrField & 0xf) > 1 || hdrLen < kHdrBytes || h->pktSize < hdrLen || h->pktSize > n) return false;
  *payload = p + hdrLen;
  *payloadBytes = h->pktSize - hdrLen;
  return true;
}

struct Service { uint32_t id, api, branch; };

// q6core bring-up queries: ready state, then framework version. step() is called with a monotonic ms clock.
struct Core {
  enum Phase : int { Idle, StateWait, StatePause, FwkWait, Done, Failed };
  static constexpr uint32_t kMaxServices = 24, kTryMs = 100, kPauseMs = 50, kFwkMs = 1000;
  void *ctx = nullptr;
  int (*send)(void *, const uint8_t *, uint32_t) = nullptr;   // 0 = queued, <0 = retry later
  void (*log)(void *, const char *what, uint32_t a, uint32_t b) = nullptr;
  uint32_t readyBudgetMs = 10000;
  int phase = Idle;
  uint32_t token = 0, started = 0, deadline = 0, tries = 0, answers = 0, state = 0, basicStatus = 0, basicOpcode = 0;
  uint32_t unexpected = 0, malformed = 0;
  bool stateAnswered = false, fwkAnswered = false, fwkRefused = false;
  uint32_t major = 0, minor = 0, branch = 0, subbranch = 0, services = 0, listed = 0;
  Service service[kMaxServices] = {};

  void begin(uint32_t now) {
    phase = StatePause;
    started = now;
    deadline = now;
  }
  bool finished() const { return phase == Done || phase == Failed; }
  void step(uint32_t now) {
    if (phase == StatePause && state) {   // ready: the framework version, once
      if (command(kGetFwk)) {
        phase = FwkWait;
        deadline = now + kFwkMs;
      }
      return;
    }
    if (phase == StatePause && now >= deadline) {
      if (now - started >= readyBudgetMs) return fail("ready budget exhausted", tries, answers);
      if (command(kGetState)) {
        tries++;
        phase = StateWait;
        deadline = now + kTryMs;
      } else {
        deadline = now + kPauseMs;   // channel busy: try again
      }
    } else if (phase == StateWait && now >= deadline) {
      phase = StatePause;            // no answer within 100 ms (q6core_is_adsp_ready returns false)
      deadline = now + kPauseMs;
    } else if (phase == FwkWait && now >= deadline) {
      note("fwk version: no answer", kFwkMs, 0);
      phase = Done;                  // the ready query was answered: the APR path works
    }
  }
  void receive(const uint8_t *p, uint32_t n, uint32_t now) {
    Header h;
    const uint8_t *pl = nullptr;
    uint32_t pn = 0;
    if (!parse(p, n, &h, &pl, &pn)) {
      malformed++;
      return note("malformed packet", n, 0);
    }
    if (h.srcSvc != kSvcCore || h.destSvc != kSvcCore || h.destDomain != kDomainApps) {
      unexpected++;
      return note("packet for another service", (h.srcDomain << 8) | h.srcSvc, h.opcode);
    }
    if (h.opcode == kGetStateRsp && pn >= 4) {
      if (h.token != token) return note("stale get-state answer", h.token, token);
      answers++;
      stateAnswered = true;
      state = rd32(pl);
      note("get-state answer", state, now - started);
      if (phase == StateWait) {
        phase = StatePause;
        deadline = state ? now : now + kPauseMs;
      }
      if (state) step(now);
    } else if (h.opcode == kGetFwkRsp && pn >= 20) {
      major = rd32(pl);
      minor = rd32(pl + 4);
      branch = rd32(pl + 8);
      subbranch = rd32(pl + 12);
      services = rd32(pl + 16);
      listed = 0;
      for (uint32_t i = 0; i < services && i < kMaxServices && 20 + 12 * (i + 1) <= pn; i++, listed++)
        service[i] = {rd32(pl + 20 + 12 * i), rd32(pl + 24 + 12 * i), rd32(pl + 28 + 12 * i)};
      fwkAnswered = true;
      note("fwk version answer", services, listed);
      if (phase == FwkWait) phase = Done;
    } else if (h.opcode == kBasicResult && pn >= 8) {
      basicOpcode = rd32(pl);
      basicStatus = rd32(pl + 4);
      note("basic result", basicOpcode, basicStatus);
      if (basicOpcode == kGetFwk && phase == FwkWait) {
        fwkRefused = true;
        phase = Done;
      } else if (basicOpcode == kGetState && phase == StateWait) {
        answers++;
        stateAnswered = true;   // answered, but not with a state: keep asking until the budget ends
        phase = StatePause;
        deadline = now + kPauseMs;
      }
    } else {
      unexpected++;
      note("unexpected core opcode", h.opcode, pn);
    }
  }

private:
  void note(const char *what, uint32_t a, uint32_t b) {
    if (log) log(ctx, what, a, b);
  }
  void fail(const char *why, uint32_t a, uint32_t b) {
    phase = Failed;
    note(why, a, b);
  }
  bool command(uint32_t opcode) {
    uint8_t pkt[kHdrBytes];
    const uint32_t n = build(pkt, sizeof(pkt), kSvcCore, 0, token + 1, opcode, nullptr, 0);
    if (!n || !send || send(ctx, pkt, n) < 0) return false;
    token++;
    note(opcode == kGetState ? "get-state sent" : "fwk version sent", token, 0);
    return true;
  }
};

}  // namespace apr
#endif
