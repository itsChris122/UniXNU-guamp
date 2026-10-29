// Host test of GuampSnsCore.h against a simulated ADSP (QRTR name server + SEE SNS client service): HELLO and
// NEW_LOOKUP, the directory, one SUID lookup per data type (protobuf decoded back field by field), SUID events,
// attribute requests and events, RESUME_TX on confirm_rx, malformed input.
#include "GuampSnsCore.h"
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
using namespace guamp_sns;
static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

static std::vector<std::vector<uint8_t>> sent;
static std::vector<std::string> logs;
static int busy = 0;
static int sendHook(void *, const uint8_t *p, uint32_t n) {
  if (busy) { --busy; return 1; }
  sent.emplace_back(p, p + n);
  return 0;
}
static void logHook(void *, const char *what, const char *text, uint64_t a, uint64_t b) {
  char line[200];
  snprintf(line, sizeof(line), "%s|%s|%llx|%llx", what, text, (unsigned long long)a, (unsigned long long)b);
  logs.push_back(line);
}
static bool logged(const char *prefix) {
  for (auto &l : logs) if (l.rfind(prefix, 0) == 0) return true;
  return false;
}

// ---- simulated ADSP side ----
constexpr uint32_t kAdsp = 5, kSnsPort = 0x22;
static std::vector<uint8_t> v1(uint32_t type, uint32_t sn, uint32_t sp, uint32_t dn, uint32_t dp, const std::vector<uint8_t> &d,
                               bool confirm = false) {
  std::vector<uint8_t> p(32 + ((d.size() + 3) & ~3u));
  wr32(&p[0], 1); wr32(&p[4], type); wr32(&p[8], sn); wr32(&p[12], sp); wr32(&p[16], confirm);
  wr32(&p[20], (uint32_t)d.size()); wr32(&p[24], dn); wr32(&p[28], dp);
  memcpy(&p[32], d.data(), d.size());
  return p;
}
static std::vector<uint8_t> ctrl(uint32_t cmd, uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
  std::vector<uint8_t> m(20);
  wr32(&m[0], cmd); wr32(&m[4], a); wr32(&m[8], b); wr32(&m[12], c); wr32(&m[16], d);
  return v1(cmd, kAdsp, kCtrl, kBroadcast, kCtrl, m);
}
static void pbKey(std::vector<uint8_t> &o, uint32_t f, uint32_t t) { uint64_t k = (uint64_t)f << 3 | t; do { o.push_back((k & 0x7f) | (k > 0x7f ? 0x80 : 0)); k >>= 7; } while (k); }
static void pbVar(std::vector<uint8_t> &o, uint64_t v) { do { o.push_back((v & 0x7f) | (v > 0x7f ? 0x80 : 0)); v >>= 7; } while (v); }
static void pbBytes(std::vector<uint8_t> &o, uint32_t f, const std::vector<uint8_t> &d) { pbKey(o, f, 2); pbVar(o, d.size()); o.insert(o.end(), d.begin(), d.end()); }
static void pbStr(std::vector<uint8_t> &o, uint32_t f, const char *s) { pbBytes(o, f, std::vector<uint8_t>(s, s + strlen(s))); }
static void pbF64(std::vector<uint8_t> &o, uint32_t f, uint64_t v) { pbKey(o, f, 1); for (int i = 0; i < 8; ++i) o.push_back(v >> 8 * i); }
static void pbF32(std::vector<uint8_t> &o, uint32_t f, uint32_t v) { pbKey(o, f, 5); for (int i = 0; i < 4; ++i) o.push_back(v >> 8 * i); }
static std::vector<uint8_t> suid(uint64_t lo, uint64_t hi) { std::vector<uint8_t> s; pbF64(s, 1, lo); pbF64(s, 2, hi); return s; }
static std::vector<uint8_t> clientEvent(uint64_t lo, uint64_t hi, uint32_t msg, const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> e, m;
  pbF32(e, 1, msg); pbF64(e, 2, 123456); pbBytes(e, 3, payload);
  pbBytes(m, 1, suid(lo, hi)); pbBytes(m, 2, e);
  return m;
}
static std::vector<uint8_t> indication(uint16_t msg, const std::vector<uint8_t> &pb, bool confirm = false) {
  std::vector<uint8_t> q(7), t;
  t.push_back(0x01); t.push_back(8); t.push_back(0); for (int i = 0; i < 8; ++i) t.push_back(i == 0 ? 7 : 0);
  t.push_back(0x02); t.push_back((pb.size() + 2) & 0xff); t.push_back((pb.size() + 2) >> 8);
  t.push_back(pb.size() & 0xff); t.push_back(pb.size() >> 8); t.insert(t.end(), pb.begin(), pb.end());
  q[0] = 4; wr16(&q[1], 0); wr16(&q[3], msg); wr16(&q[5], (uint32_t)t.size()); q.insert(q.end(), t.begin(), t.end());
  return v1(kQrtrData, kAdsp, kSnsPort, kAppsNode, kPort, q, confirm);
}
static std::vector<uint8_t> response(uint16_t txn, uint16_t result) {
  std::vector<uint8_t> q = {2, 0, 0, 0x20, 0, 7, 0, 0x02, 4, 0, 0, 0, 0, 0};
  wr16(&q[1], txn); wr16(&q[10], result);
  return v1(kQrtrData, kAdsp, kSnsPort, kAppsNode, kPort, q);
}
struct Request { uint16_t txn; uint64_t lo, hi; uint32_t msg, proc, delivery; std::string type; bool reg, defOnly, jumbo; uint32_t dn, dp; };
// Decodes one request the client sent, the way the SEE service would
static bool decodeRequest(const std::vector<uint8_t> &p, Request *r) {
  QrtrHeader h;
  if (!qrtrDecode(p.data(), (uint32_t)p.size(), &h) || h.type != kQrtrData || h.sp != kPort) return false;
  const uint8_t *q = p.data() + 32;
  if (q[0] != 0 || rd16(q + 3) != 0x20 || 7u + rd16(q + 5) != h.size) return false;
  *r = {};
  r->txn = rd16(q + 1); r->dn = h.dn; r->dp = h.dp;
  uint32_t l = 0;
  const uint8_t *d = tlv(q + 7, rd16(q + 5), 0x01, &l);
  const uint8_t *j = tlv(q + 7, rd16(q + 5), 0x10, &l);
  r->jumbo = j && j[0] == 1;
  d = tlv(q + 7, rd16(q + 5), 0x01, &l);
  if (!d || rd16(d) + 2u != l) return false;
  PbReader top{d + 2, rd16(d)};
  PbField f;
  while (top.next(&f)) {
    if (f.field == 1) { Suid s; if (!parseSuid(f.data, f.len, &s)) return false; r->lo = s.low; r->hi = s.high; }
    else if (f.field == 2 && f.type == 5) r->msg = (uint32_t)f.value;
    else if (f.field == 3) { PbReader c{f.data, f.len}; PbField x; while (c.next(&x)) { if (x.field == 1) r->proc = (uint32_t)x.value; if (x.field == 2) r->delivery = (uint32_t)x.value; } }
    else if (f.field == 4) {
      PbReader rq{f.data, f.len}; PbField x;
      while (rq.next(&x)) if (x.field == 2) {
        PbReader s{x.data, x.len}; PbField y;
        while (s.next(&y)) {
          if (y.field == 1) r->type.assign((const char *)y.data, y.len);
          if (y.field == 2) r->reg = y.value;
          if (y.field == 3) r->defOnly = y.value;
        }
      }
    }
  }
  return !top.bad;
}

int main() {
  static Client c;
  c.hooks.send = sendHook;
  c.hooks.log = logHook;
  static const char *const types[] = {"accel", "proximity", "mag"};
  c.start(types, 3);
  CHECK(c.flush() == 0 && sent.size() == 1);
  QrtrHeader h;
  CHECK(qrtrDecode(sent[0].data(), 52, &h) && h.type == kQrtrHello && h.dn == kBroadcast && h.dp == kCtrl && h.sp == kCtrl);
  c.step(0);
  CHECK(sent.size() == 1);   // nothing before the SNS service is known
  // the ADSP answers HELLO and lists its services
  c.receive(ctrl(kQrtrHello, 0, 0, 0, 0).data(), 52);
  CHECK(c.flush() == 0 && sent.size() == 2);
  CHECK(qrtrDecode(sent[1].data(), 52, &h) && h.type == kQrtrNewLookup && h.dn == kAdsp && rd32(sent[1].data() + 36) == 0);
  c.receive(ctrl(kQrtrNewServer, 0x42, 0x4a01, kAdsp, 0x10).data(), 52);
  c.receive(ctrl(kQrtrNewServer, kService, 1, kAdsp, kSnsPort).data(), 52);
  c.receive(ctrl(kQrtrNewServer, 0, 0, 0, 0).data(), 52);
  CHECK(c.snsNode == kAdsp && c.snsPort == kSnsPort && c.servers == 2);
  CHECK(logged("server|end of list|2"));
  // lookup 1: accel
  c.step(100);
  CHECK(c.flush() == 0 && sent.size() == 3);
  Request r;
  CHECK(decodeRequest(sent[2], &r));
  CHECK(r.dn == kAdsp && r.dp == kSnsPort && r.msg == kMsgSuidReq && r.lo == kLookupSuid && r.hi == kLookupSuid);
  CHECK(r.type == "accel" && r.reg && !r.defOnly && r.proc == kProcApss && r.delivery == 0 && r.jumbo);
  c.receive(response(r.txn, 0).data(), (uint32_t)response(r.txn, 0).size());
  c.step(200);
  CHECK(sent.size() == 3);   // still waiting for the SUID event
  std::vector<uint8_t> ev;
  pbStr(ev, 1, "accel"); pbBytes(ev, 2, suid(0x1111, 0x2222)); pbBytes(ev, 2, suid(0x3333, 0x4444));
  auto ind = indication(kQmiReport, clientEvent(kLookupSuid, kLookupSuid, kMsgSuidEvent, ev), true);
  c.receive(ind.data(), (uint32_t)ind.size());
  CHECK(c.foundCount == 2 && c.found[0].suid.low == 0x1111 && c.found[1].suid.high == 0x4444);
  CHECK(c.flush() == 0 && sent.size() == 4);   // RESUME_TX for the confirm_rx indication
  CHECK(qrtrDecode(sent[3].data(), 52, &h) && h.type == kQrtrResume && rd32(sent[3].data() + 36) == kAppsNode &&
        rd32(sent[3].data() + 40) == kPort);
  // lookup 2 goes out at once (event answered lookup 1); its answer is a jumbo indication with no SUID
  c.step(210);
  CHECK(c.flush() == 0 && sent.size() == 5 && decodeRequest(sent[4], &r) && r.type == "proximity");
  std::vector<uint8_t> none;
  pbStr(none, 1, "proximity");
  ind = indication(kQmiJumbo, clientEvent(kLookupSuid, kLookupSuid, kMsgSuidEvent, none));
  c.receive(ind.data(), (uint32_t)ind.size());
  CHECK(logged("suid|proximity|0|0"));
  // lookup 3 gets no answer: the client moves on after the wait
  c.step(220);
  CHECK(c.flush() == 0 && sent.size() == 6 && decodeRequest(sent[5], &r) && r.type == "mag");
  c.step(220 + 1000);
  CHECK(sent.size() == 6);
  // a refused request is logged
  c.receive(response(r.txn, 1).data(), (uint32_t)response(r.txn, 1).size());
  CHECK(logged("error|request refused|1"));
  // attributes of each SUID, transport busy once
  busy = 1;
  c.step(220 + Client::kWaitMs);
  CHECK(c.flush() == 0 && sent.size() == 6 && c.qn == 1);
  CHECK(c.flush() == 0 && sent.size() == 7 && decodeRequest(sent[6], &r) && r.msg == kMsgAttrReq && r.lo == 0x1111 && r.type.empty());
  std::vector<uint8_t> attrs, a1, v1s, d1, a2, v2, d2;
  pbStr(d1, 2, "lsm6dso"); pbBytes(v1s, 1, d1); pbKey(a1, 1, 0); pbVar(a1, 0); pbBytes(a1, 2, v1s); pbBytes(attrs, 1, a1);
  pbKey(d2, 4, 0); pbVar(d2, 2 * 0x10e); pbBytes(v2, 1, d2); pbKey(a2, 1, 0); pbVar(a2, 4); pbBytes(a2, 2, v2); pbBytes(attrs, 1, a2);
  ind = indication(kQmiReport, clientEvent(0x1111, 0x2222, kMsgAttrEvent, attrs));
  c.receive(ind.data(), (uint32_t)ind.size());
  CHECK(logged("attr|lsm6dso|0|1111") && logged("attr||4|10e"));
  c.step(5000);
  CHECK(c.flush() == 0 && sent.size() == 8 && decodeRequest(sent[7], &r) && r.lo == 0x3333);
  c.step(5000 + Client::kWaitMs);
  CHECK(c.done && logged("lookup-done||2|2"));
  // malformed input never crashes and is reported
  const uint8_t junk[40] = {1, 0, 0, 0, 1};
  c.receive(junk, 10);
  c.receive(junk, sizeof(junk));
  auto bad = indication(kQmiReport, {0x0a, 0x7f, 0x01});
  c.receive(bad.data(), (uint32_t)bad.size());
  CHECK(logged("error|bad qrtr packet") && logged("error|malformed client event"));
  // the service goes away
  c.receive(ctrl(kQrtrDelServer, kService, 1, kAdsp, kSnsPort).data(), 52);
  CHECK(!c.snsPort);
  // builders refuse what does not fit
  uint8_t small[16];
  CHECK(!suidLookup(small, sizeof(small), "accel"));
  CHECK(!qmiRequest(small, sizeof(small), 1, small, 0));
  printf("sns_host: %s (%zu packets, %zu log lines)\n", fails ? "FAIL" : "PASS", sent.size(), logs.size());
  return fails ? 1 : 0;
}
