// SPDX-License-Identifier: BSD-2-Clause-Patent
// Sensors on the ADSP (SEE, analisi/sensori/SENSORI-20261029.md step 6): a QRTR client on the ADSP GLINK channel
// "IPCRTR" and the QMI SNS client service (id 400) with the SUID lookup and the attribute request. Pure logic, no
// kernel calls (host test tests/sns_host.cpp); the glue is in GuampAdspIpc.inc (guamp-sensors=2).
//   QRTR v1 as GuampQrtr.inc (header 32 bytes: version 1, type, src node/port, confirm, size, dst node/port); apps node
//   1, client port kPort; HELLO to the broadcast control port, then NEW_LOOKUP(0) so the ADSP name server lists all its
//   services (NEW_SERVER ... and an empty 0/0 entry at the end); confirm_rx DATA is answered with RESUME_TX.
//   QMI without QMUX: type (0 request, 2 response, 4 indication), transaction u16, message u16, length u16, TLVs.
//   SNS_CLIENT_REQ 0x20: TLV 0x01 payload (u16 length + protobuf sns_client_request_msg, max 1000 bytes),
//   TLV 0x10 use_jumbo_report (u8). Response 0x20: TLV 0x02 result/error, TLV 0x10 client id (u64).
//   Indications 0x21 (report) and 0x22 (jumbo report): TLV 0x01 client id (u64), TLV 0x02 payload (u16 length +
//   sns_client_event_msg). Message ids and field numbers from the public SEE protos (sns_client.proto,
//   sns_std.proto, sns_suid.proto, sns_std_sensor.proto); the firmware carries sns_client.proto and
//   SNS_CLIENT_SVC_..._v01.
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace guamp_sns {

enum : uint32_t {
  kService = 400, kAppsNode = 1, kPort = 0x4010, kCtrl = 0xfffffffeu, kBroadcast = 0xffffffffu,
  kQrtrData = 1, kQrtrHello = 2, kQrtrBye = 3, kQrtrNewServer = 4, kQrtrDelServer = 5, kQrtrDelClient = 6,
  kQrtrResume = 7, kQrtrNewLookup = 10,
  kQmiReq = 0x20, kQmiReport = 0x21, kQmiJumbo = 0x22, kMaxPayload = 1000,
  kMsgAttrReq = 1, kMsgAttrEvent = 128, kMsgSuidReq = 512, kMsgSuidEvent = 768, kMsgErrorEvent = 6,
  kProcApss = 1, kDeliveryWakeup = 0
};
static constexpr uint64_t kLookupSuid = 0xababababababababull;   // the SUID lookup sensor (low and high)

static inline uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static inline uint32_t rd32(const uint8_t *p) { return (uint32_t)rd16(p) | (uint32_t)rd16(p + 2) << 16; }
static inline uint64_t rd64(const uint8_t *p) { return (uint64_t)rd32(p) | (uint64_t)rd32(p + 4) << 32; }
static inline void wr16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void wr32(uint8_t *p, uint32_t v) { wr16(p, v); wr16(p + 2, v >> 16); }
static inline void wr64(uint8_t *p, uint64_t v) { wr32(p, (uint32_t)v); wr32(p + 4, (uint32_t)(v >> 32)); }

// ---- protobuf (wire types 0 varint, 1 fixed64, 2 length-delimited, 5 fixed32) ----
struct PbWriter {
  uint8_t *p;
  uint32_t cap, n = 0;
  bool ok = true;
  void byte(uint8_t b) { if (n < cap) p[n++] = b; else ok = false; }
  void varint(uint64_t v) { do { byte((uint8_t)((v & 0x7f) | (v > 0x7f ? 0x80 : 0))); v >>= 7; } while (v); }
  void key(uint32_t field, uint32_t type) { varint((uint64_t)field << 3 | type); }
  void uvar(uint32_t field, uint64_t v) { key(field, 0); varint(v); }
  void fixed64(uint32_t field, uint64_t v) { key(field, 1); for (unsigned i = 0; i < 8; ++i) byte((uint8_t)(v >> 8 * i)); }
  void fixed32(uint32_t field, uint32_t v) { key(field, 5); for (unsigned i = 0; i < 4; ++i) byte((uint8_t)(v >> 8 * i)); }
  void bytes(uint32_t field, const void *data, uint32_t len) {
    key(field, 2); varint(len);
    const uint8_t *d = static_cast<const uint8_t *>(data);
    for (uint32_t i = 0; i < len; ++i) byte(d[i]);
  }
};

struct PbField {
  uint32_t field, type;
  uint64_t value;        // varint / fixed
  const uint8_t *data;   // length-delimited
  uint32_t len;
};
struct PbReader {
  const uint8_t *p;
  uint32_t n, at = 0;
  bool bad = false;
  bool varint(uint64_t *v) {
    *v = 0;
    for (unsigned s = 0; s < 64; s += 7) {
      if (at >= n) return false;
      const uint8_t b = p[at++];
      *v |= (uint64_t)(b & 0x7f) << s;
      if (!(b & 0x80)) return true;
    }
    return false;
  }
  // Next field; false at the end or on malformed input (then bad is set)
  bool next(PbField *f) {
    if (bad || at >= n) return false;
    uint64_t k;
    if (!varint(&k) || !(k >> 3) || (k >> 3) > 0xffffffffu) { bad = true; return false; }
    *f = {(uint32_t)(k >> 3), (uint32_t)(k & 7), 0, nullptr, 0};
    switch (f->type) {
      case 0: if (!varint(&f->value)) { bad = true; return false; } return true;
      case 1: if (n - at < 8) { bad = true; return false; } f->value = rd64(p + at); at += 8; return true;
      case 5: if (n - at < 4) { bad = true; return false; } f->value = rd32(p + at); at += 4; return true;
      case 2: {
        uint64_t len;
        if (!varint(&len) || len > n - at) { bad = true; return false; }
        f->data = p + at; f->len = (uint32_t)len; at += (uint32_t)len; return true;
      }
      default: bad = true; return false;
    }
  }
};

struct Suid { uint64_t low = 0, high = 0; };

static inline bool parseSuid(const uint8_t *p, uint32_t n, Suid *s) {
  PbReader r{p, n};
  PbField f;
  bool lo = false, hi = false;
  while (r.next(&f)) {
    if (f.field == 1 && f.type == 1) { s->low = f.value; lo = true; }
    else if (f.field == 2 && f.type == 1) { s->high = f.value; hi = true; }
  }
  return !r.bad && lo && hi;
}

// sns_client_request_msg { suid 1, msg_id 2 (fixed32), susp_config 3 {client_proc_type 1, delivery_type 2},
// request 4 (sns_std_request { payload 2 }) }
static inline uint32_t clientRequest(uint8_t *out, uint32_t cap, const Suid &suid, uint32_t msgId, const uint8_t *payload,
                                     uint32_t payloadLen) {
  uint8_t s[24], c[8], q[kMaxPayload];
  PbWriter ws{s, sizeof(s)};
  ws.fixed64(1, suid.low); ws.fixed64(2, suid.high);
  PbWriter wc{c, sizeof(c)};
  wc.uvar(1, kProcApss); wc.uvar(2, kDeliveryWakeup);
  PbWriter wq{q, sizeof(q)};
  if (payloadLen) wq.bytes(2, payload, payloadLen);
  PbWriter w{out, cap};
  w.bytes(1, s, ws.n); w.fixed32(2, msgId); w.bytes(3, c, wc.n); w.bytes(4, q, wq.n);
  return ws.ok && wc.ok && wq.ok && w.ok ? w.n : 0;
}

// sns_suid_req { data_type 1, register_updates 2, default_only 3 }: every sensor of the type, not only the default
static inline uint32_t suidLookup(uint8_t *out, uint32_t cap, const char *dataType) {
  uint8_t r[64];
  uint32_t len = 0;
  while (dataType[len] && len < 48) ++len;
  PbWriter w{r, sizeof(r)};
  w.bytes(1, dataType, len); w.uvar(2, 1); w.uvar(3, 0);
  if (!w.ok || dataType[len]) return 0;
  return clientRequest(out, cap, Suid{kLookupSuid, kLookupSuid}, kMsgSuidReq, r, w.n);
}
static inline uint32_t attrRequest(uint8_t *out, uint32_t cap, const Suid &suid) {
  return clientRequest(out, cap, suid, kMsgAttrReq, nullptr, 0);
}

// QMI SNS_CLIENT_REQ around a protobuf request; use_jumbo_report on (IPCRTR intents go up to 0x4400 bytes)
static inline uint32_t qmiRequest(uint8_t *out, uint32_t cap, uint16_t txn, const uint8_t *pb, uint32_t len) {
  const uint32_t body = 3 + 2 + len + 3 + 1;
  if (!len || len > kMaxPayload || cap < 7 + body) return 0;
  out[0] = 0; wr16(out + 1, txn); wr16(out + 3, kQmiReq); wr16(out + 5, body);
  uint8_t *t = out + 7;
  t[0] = 0x01; wr16(t + 1, 2 + len); wr16(t + 3, len);
  for (uint32_t i = 0; i < len; ++i) t[5 + i] = pb[i];
  t += 5 + len;
  t[0] = 0x10; wr16(t + 1, 1); t[3] = 1;
  return 7 + body;
}

static inline const uint8_t *tlv(const uint8_t *p, uint32_t n, uint8_t tag, uint32_t *len) {
  for (uint32_t at = 0; at + 3 <= n;) {
    const uint32_t l = rd16(p + at + 1);
    if (l > n - at - 3) return nullptr;
    if (p[at] == tag) { *len = l; return p + at + 3; }
    at += 3 + l;
  }
  return nullptr;
}

// QRTR v1 packet; returns its size (payload padded to 4 bytes)
static inline uint32_t qrtrPacket(uint8_t *out, uint32_t cap, uint32_t type, uint32_t srcPort, uint32_t dstNode,
                                  uint32_t dstPort, const uint8_t *data, uint32_t n, bool confirm = false) {
  const uint32_t bytes = 32 + ((n + 3) & ~3u);
  if (bytes > cap) return 0;
  for (uint32_t i = 0; i < bytes; ++i) out[i] = 0;
  wr32(out, 1); wr32(out + 4, type); wr32(out + 8, kAppsNode); wr32(out + 12, srcPort); wr32(out + 16, confirm ? 1 : 0);
  wr32(out + 20, n); wr32(out + 24, dstNode); wr32(out + 28, dstPort);
  for (uint32_t i = 0; i < n; ++i) out[32 + i] = data[i];
  return bytes;
}
static inline uint32_t qrtrControl(uint8_t *out, uint32_t cap, uint32_t cmd, uint32_t dstNode, uint32_t a, uint32_t b,
                                   uint32_t c, uint32_t d) {
  uint8_t m[20];
  wr32(m, cmd); wr32(m + 4, a); wr32(m + 8, b); wr32(m + 12, c); wr32(m + 16, d);
  return qrtrPacket(out, cap, cmd, kCtrl, dstNode, kCtrl, m, sizeof(m));
}

struct QrtrHeader { uint32_t type, sn, sp, dn, dp, size, header; bool confirm; };
// v1 and v2 headers (GuampQrtr.inc decode, without its WLFW checks)
static inline bool qrtrDecode(const uint8_t *p, uint32_t n, QrtrHeader *h) {
  if (!p || n < 16) return false;
  *h = {};
  if (rd32(p) == 1) {
    if (n < 32) return false;
    h->type = rd32(p + 4); h->sn = rd32(p + 8); h->sp = rd32(p + 12); h->confirm = rd32(p + 16) != 0;
    h->size = rd32(p + 20); h->dn = rd32(p + 24); h->dp = rd32(p + 28); h->header = 32;
  } else if (p[0] == 3) {
    h->type = p[1]; h->confirm = p[2] & 1; h->size = rd32(p + 4); h->header = 16 + p[3];
    h->sn = rd16(p + 8); h->sp = rd16(p + 10); h->dn = rd16(p + 12); h->dp = rd16(p + 14);
    if (h->sn == 0xffff) h->sn = kBroadcast;
    if (h->dn == 0xffff) h->dn = kBroadcast;
  } else {
    return false;
  }
  if (h->sp == 0xffff) h->sp = kCtrl;
  if (h->dp == 0xffff) h->dp = kCtrl;
  return h->header <= n && h->size <= n - h->header;
}

// ---- the client: QRTR hello/lookup, SUID lookups of a list of data types, then attributes of each SUID ----
struct Client {
  enum : uint32_t { kQueue = 8, kPacket = 1200, kTypes = 12, kSuids = 24, kWaitMs = 1500 };
  struct Hooks {
    void *ctx = nullptr;
    int (*send)(void *, const uint8_t *, uint32_t) = nullptr;   // 0 sent, >0 try again, <0 error
    // what: "server", "server-gone", "suid", "attr", "event", "lookup-done", "error"; text may be empty
    void (*log)(void *, const char *what, const char *text, uint64_t a, uint64_t b) = nullptr;
  } hooks;
  struct Packet { uint8_t data[kPacket]; uint32_t n; };
  Packet queue[kQueue];
  uint32_t qr = 0, qw = 0, qn = 0;
  const char *const *types = nullptr;
  uint32_t typeCount = 0, typeAt = 0, attrAt = 0;
  struct Found { Suid suid; uint32_t type; bool attrs; };
  Found found[kSuids];
  uint32_t foundCount = 0, servers = 0, events = 0, dropped = 0;
  uint32_t snsNode = 0, snsPort = 0, remoteNode = 0;
  uint16_t txn = 0;
  bool started = false, hello = false, waiting = false, done = false;
  uint64_t waitSince = 0;

  void note(const char *what, const char *text, uint64_t a, uint64_t b) {
    if (hooks.log) hooks.log(hooks.ctx, what, text ? text : "", a, b);
  }
  bool push(const uint8_t *p, uint32_t n) {
    if (!n || n > kPacket || qn == kQueue) { ++dropped; return false; }
    Packet &q = queue[qw];
    for (uint32_t i = 0; i < n; ++i) q.data[i] = p[i];
    q.n = n; qw = (qw + 1) % kQueue; ++qn;
    return true;
  }
  void start(const char *const *list, uint32_t count) {
    types = list; typeCount = count > kTypes ? kTypes : count;
    uint8_t b[64];
    push(b, qrtrControl(b, sizeof(b), kQrtrHello, kBroadcast, 0, 0, 0, 0));
    started = true;
  }
  // Transmit queued packets; returns <0 on a transport error
  int flush() {
    while (qn) {
      const int e = hooks.send(hooks.ctx, queue[qr].data, queue[qr].n);
      if (e > 0) return 0;
      if (e < 0) return e;
      qr = (qr + 1) % kQueue; --qn;
    }
    return 0;
  }
  bool sendQmi(const uint8_t *pb, uint32_t len) {
    uint8_t m[kPacket - 32];
    const uint32_t n = qmiRequest(m, sizeof(m), ++txn, pb, len);
    uint8_t b[kPacket];
    return n && push(b, qrtrPacket(b, sizeof(b), kQrtrData, kPort, snsNode, snsPort, m, n));
  }
  // Lookups first (one at a time), then one attribute request per SUID found
  void step(uint64_t nowMs) {
    if (!started || done || !snsPort) return;
    if (waiting && nowMs - waitSince < kWaitMs) return;
    waiting = false;
    uint8_t pb[kMaxPayload];
    if (typeAt < typeCount) {
      const uint32_t n = suidLookup(pb, sizeof(pb), types[typeAt]);
      if (n && sendQmi(pb, n)) { waiting = true; waitSince = nowMs; }
      ++typeAt;
      return;
    }
    while (attrAt < foundCount && found[attrAt].attrs) ++attrAt;
    if (attrAt < foundCount) {
      const uint32_t n = attrRequest(pb, sizeof(pb), found[attrAt].suid);
      if (n && sendQmi(pb, n)) { waiting = true; waitSince = nowMs; }
      found[attrAt++].attrs = true;
      return;
    }
    done = true;
    note("lookup-done", "", foundCount, servers);
  }
  void suidEvent(const uint8_t *p, uint32_t n) {
    PbReader r{p, n};
    PbField f;
    char type[33] = {};
    uint32_t count = 0;
    Suid list[8];
    while (r.next(&f)) {
      if (f.field == 1 && f.type == 2) {
        const uint32_t l = f.len < 32 ? f.len : 32;
        for (uint32_t i = 0; i < l; ++i) type[i] = (char)f.data[i];
      } else if (f.field == 2 && f.type == 2 && count < 8 && parseSuid(f.data, f.len, &list[count])) {
        ++count;
      }
    }
    if (r.bad) { note("error", "malformed suid event", n, 0); return; }
    if (!count) note("suid", type, 0, 0);
    uint32_t typeIndex = typeCount;
    for (uint32_t t = 0; t < typeCount; ++t) {
      uint32_t i = 0;
      while (types[t][i] && types[t][i] == type[i]) ++i;
      if (!types[t][i] && !type[i]) typeIndex = t;
    }
    for (uint32_t i = 0; i < count; ++i) {
      note("suid", type, list[i].low, list[i].high);
      bool known = false;
      for (uint32_t k = 0; k < foundCount; ++k)
        known |= found[k].suid.low == list[i].low && found[k].suid.high == list[i].high;
      if (!known && foundCount < kSuids) found[foundCount++] = {list[i], typeIndex, false};
    }
    if (waiting && typeAt && typeIndex == typeAt - 1) waiting = false;   // answer to the current lookup: go on
  }
  // sns_std_attr_event { repeated attr 1 { attr_id 1, value 2 { repeated data 1 { str 2, flt 3, sint 4, boolean 5 }}}}:
  // strings of name (0), vendor (1), type (2) and the first value of version (4) and hw id (18)
  void attrEvent(const Suid &s, const uint8_t *p, uint32_t n) {
    PbReader r{p, n};
    PbField a;
    while (r.next(&a)) {
      if (a.field != 1 || a.type != 2) continue;
      PbReader ra{a.data, a.len};
      PbField f;
      int64_t id = -1;
      const uint8_t *value = nullptr;
      uint32_t valueLen = 0;
      while (ra.next(&f)) {
        if (f.field == 1 && f.type == 0) id = (int64_t)f.value;
        else if (f.field == 2 && f.type == 2) { value = f.data; valueLen = f.len; }
      }
      if (id != 0 && id != 1 && id != 2 && id != 4 && id != 18) continue;
      if (!value) continue;
      PbReader rv{value, valueLen};
      PbField d;
      while (rv.next(&d)) {
        if (d.field != 1 || d.type != 2) continue;
        PbReader rd{d.data, d.len};
        PbField x;
        while (rd.next(&x)) {
          char text[40] = {};
          if (x.field == 2 && x.type == 2) {
            const uint32_t l = x.len < 39 ? x.len : 39;
            for (uint32_t i = 0; i < l; ++i) text[i] = (char)x.data[i];
            note("attr", text, (uint64_t)id, s.low);
          } else if (x.field == 4 && x.type == 0) {
            note("attr", "", (uint64_t)id, x.value >> 1 ^ (0 - (x.value & 1)));   // zigzag
          } else if (x.field == 5 && x.type == 0) {
            note("attr", x.value ? "true" : "false", (uint64_t)id, s.low);
          }
        }
        break;   // first value only
      }
    }
  }
  // sns_client_event_msg { suid 1, repeated event 2 { msg_id 1 (fixed32), timestamp 2 (fixed64), payload 3 } }
  void clientEvent(const uint8_t *p, uint32_t n) {
    PbReader r{p, n};
    PbField f;
    Suid s;
    bool haveSuid = false;
    while (r.next(&f)) {
      if (f.field == 1 && f.type == 2) { haveSuid = parseSuid(f.data, f.len, &s); continue; }
      if (f.field != 2 || f.type != 2) continue;
      PbReader re{f.data, f.len};
      PbField e;
      uint32_t msg = 0;
      const uint8_t *payload = nullptr;
      uint32_t payloadLen = 0;
      while (re.next(&e)) {
        if (e.field == 1 && e.type == 5) msg = (uint32_t)e.value;
        else if (e.field == 3 && e.type == 2) { payload = e.data; payloadLen = e.len; }
      }
      ++events;
      if (!haveSuid) continue;
      if (msg == kMsgSuidEvent && s.low == kLookupSuid && s.high == kLookupSuid) suidEvent(payload, payloadLen);
      else if (msg == kMsgAttrEvent) { attrEvent(s, payload, payloadLen); if (waiting && typeAt >= typeCount) waiting = false; }
      else note("event", msg == kMsgErrorEvent ? "error event" : "", msg, s.low);
    }
    if (r.bad) note("error", "malformed client event", n, 0);
  }
  void qmi(const uint8_t *p, uint32_t n) {
    if (n < 7 || (uint32_t)7 + rd16(p + 5) > n) { note("error", "short qmi", n, 0); return; }
    const uint16_t msg = rd16(p + 3);
    const uint8_t *body = p + 7;
    const uint32_t len = rd16(p + 5);
    uint32_t l = 0;
    if (p[0] == 2 && msg == kQmiReq) {
      const uint8_t *r = tlv(body, len, 0x02, &l);
      if (!r || l < 4) { note("error", "response without result", msg, 0); return; }
      if (rd16(r)) note("error", "request refused", rd16(r), rd16(r + 2));
      return;
    }
    if (p[0] == 4 && (msg == kQmiReport || msg == kQmiJumbo)) {
      const uint8_t *d = tlv(body, len, 0x02, &l);
      if (!d || l < 2 || (uint32_t)2 + rd16(d) > l) { note("error", "indication without payload", msg, l); return; }
      clientEvent(d + 2, rd16(d));
      return;
    }
    note("error", "unexpected qmi message", p[0], msg);
  }
  // One QRTR packet from the channel
  void receive(const uint8_t *p, uint32_t n) {
    QrtrHeader h;
    if (!qrtrDecode(p, n, &h)) { note("error", "bad qrtr packet", n, 0); return; }
    const uint8_t *d = p + h.header;
    uint8_t b[64];
    if (h.type == kQrtrData && h.dp == kPort) {
      if (h.confirm) push(b, qrtrControl(b, sizeof(b), kQrtrResume, h.sn, kAppsNode, kPort, 0, 0));
      qmi(d, h.size);
      return;
    }
    if (h.type == kQrtrData && h.dp != kCtrl) return;
    if (h.size < 4) return;
    const uint32_t cmd = rd32(d);
    if (cmd == kQrtrHello) {
      remoteNode = h.sn;
      if (!hello) {
        hello = true;
        push(b, qrtrControl(b, sizeof(b), kQrtrNewLookup, h.sn, 0, 0, 0, 0));   // every service of the node
      }
      note("hello", "", h.sn, 0);
    } else if ((cmd == kQrtrNewServer || cmd == kQrtrDelServer) && h.size >= 20) {
      const uint32_t service = rd32(d + 4), instance = rd32(d + 8), node = rd32(d + 12), port = rd32(d + 16);
      if (!node && !port) { note("server", "end of list", servers, 0); return; }
      const bool up = cmd == kQrtrNewServer;
      if (up) ++servers;
      note(up ? "server" : "server-gone", "", (uint64_t)service << 32 | instance, (uint64_t)node << 32 | port);
      if (service == kService && up && !snsPort) { snsNode = node; snsPort = port; }
      if (service == kService && !up && node == snsNode && port == snsPort) snsNode = snsPort = 0;
    } else if ((cmd == kQrtrBye && h.sn == snsNode) || (cmd == kQrtrDelClient && h.size >= 12 && rd32(d + 4) == snsNode &&
                                                         rd32(d + 8) == snsPort)) {
      note("server-gone", "sns", snsNode, snsPort);
      snsNode = snsPort = 0;
    }
  }
};

}  // namespace guamp_sns
