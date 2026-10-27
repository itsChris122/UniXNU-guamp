// Host tests of the RPM client core (GuampRpmCore.h): rpm-smd request/ack encoding, the GLINK FIFO ring logic and the
// session/request state machine against a fake message RAM (TOC at the end, ap2r / r2ap FIFOs) with a fake RPM peer
// that runs on every doorbell and every delay, reads the apps->RPM FIFO and answers like the RPM firmware would
// (VERSION_ACK, remote OPEN, OPEN_ACK, acks with "msg#" / "err\0", v1 ACK/NACK). Rules checked on every access: apps
// writes only its tx FIFO, its tx head and its rx tail; every apps packet is 8-byte padded with zeros.
#include <stdio.h>
#include <string.h>
#include <deque>
#include <string>
#include <vector>
#include "../GuampRpmCore.h"
#include "../GuampBtBoard.h"

static int failures = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

typedef std::vector<uint8_t> Bytes;
static void put(Bytes &b, uint32_t v) { for (int k = 0; k < 4; ++k) b.push_back(static_cast<uint8_t>(v >> (8 * k))); }
static uint32_t get(const Bytes &b, size_t at) { return rpm_get32(&b[at]); }

struct Fake {
  // layout
  uint32_t size = RPM_MSG_RAM_SIZE, txDesc = 0x100, txLen = 0x400, rxDesc = 0x600, rxLen = 0x400;
  std::vector<uint32_t> ram;
  // knobs
  bool dead = false, remoteOpens = true, openAck = true, peerVersion = true, splitAck = false, noise = false;
  bool otherOpen = false, oversize = false;
  unsigned fmt = RPM_FMT_V0, ackDelay = 0, dropAcks = 0, readStallTicks = 0, consumePerTick = 0x7fffffff;
  uint32_t peerFeatures = 0;
  // peer state
  uint16_t rcid = 7, appsLcid = 0;
  bool sentOpen = false, gotOpenAck = false;
  unsigned tick = 0;
  std::deque<Bytes> outbox;
  struct Due { unsigned at; Bytes ack; };
  std::deque<Due> pending;
  struct Req { unsigned set; uint32_t type, id, msgId; std::vector<std::pair<uint32_t, uint64_t>> kv; };
  std::vector<Req> reqs;
  // observations
  unsigned kicks = 0, badWrites = 0, badReads = 0, badPad = 0, versions = 0, versionAcks = 0, opens = 0, closeAcks = 0,
           readNotifs = 0, unknown = 0, badData = 0;
  uint64_t us = 0;
  rpm_io io;

  Fake() { ram.assign(size / 4, 0); io = {this, sRd, sWr, sBar, sKick, sDelay}; }
  void toc(bool goodMagic = true, bool withRx = true) {
    const uint32_t b = (size - RPM_TOC_SIZE) / 4;
    ram[b] = goodMagic ? RPM_TOC_MAGIC : 0x12345678;
    ram[b + 1] = withRx ? 3 : 1;
    ram[b + 2] = RPM_TX_FIFO_ID; ram[b + 3] = txDesc; ram[b + 4] = txLen;
    ram[b + 5] = 0x11111111; ram[b + 6] = 0x10; ram[b + 7] = 0x10;   // unrelated entry: ignored
    ram[b + 8] = RPM_RX_FIFO_ID; ram[b + 9] = rxDesc; ram[b + 10] = rxLen;
  }
  uint32_t &txTail() { return ram[txDesc / 4]; }
  uint32_t &txHead() { return ram[txDesc / 4 + 1]; }
  uint32_t &rxTail() { return ram[rxDesc / 4]; }
  uint32_t &rxHead() { return ram[rxDesc / 4 + 1]; }

  static uint32_t sRd(void *c, uint32_t off) {
    Fake *f = static_cast<Fake *>(c);
    if (off % 4 || off >= f->size) { f->badReads++; return 0xdeadbeef; }
    return f->ram[off / 4];
  }
  static void sWr(void *c, uint32_t off, uint32_t v) {
    Fake *f = static_cast<Fake *>(c);
    const bool ok = off % 4 == 0 && (off == f->txDesc + 4 || off == f->rxDesc ||
                                     (off >= f->txDesc + 8 && off < f->txDesc + 8 + f->txLen));
    if (!ok) { f->badWrites++; return; }
    f->ram[off / 4] = v;
  }
  static void sBar(void *) {}
  static void sKick(void *c) { Fake *f = static_cast<Fake *>(c); f->kicks++; f->run(); }
  static void sDelay(void *c, unsigned us) { Fake *f = static_cast<Fake *>(c); f->us += us; f->tick++; f->run(); }

  // --- peer side ---
  Bytes readTx(uint32_t off, uint32_t n) {   // from the peer's tail
    Bytes b;
    uint32_t pos = (txTail() + off) % txLen;
    for (uint32_t i = 0; i < n; i += 4) {
      put(b, ram[(txDesc + 8 + pos) / 4]);
      pos = (pos + 4) % txLen;
    }
    b.resize(n);
    return b;
  }
  void queue(const Bytes &pkt) { outbox.push_back(pkt); }
  void cmd(uint16_t c, uint16_t p1, uint32_t p2) { Bytes b; put(b, c | static_cast<uint32_t>(p1) << 16); put(b, p2); queue(b); }
  void open(uint16_t id, const char *name) {
    Bytes b; const uint32_t n = static_cast<uint32_t>(strlen(name)) + 1;
    put(b, GL_OPEN | static_cast<uint32_t>(id) << 16); put(b, n | 0x10000u /* prio */);
    for (uint32_t k = 0; k < n; ++k) b.push_back(static_cast<uint8_t>(name[k]));
    queue(b);
  }
  void data(uint16_t c, const Bytes &payload, uint32_t left) {
    Bytes b; put(b, c | static_cast<uint32_t>(rcid) << 16); put(b, 0); put(b, static_cast<uint32_t>(payload.size())); put(b, left);
    b.insert(b.end(), payload.begin(), payload.end());
    queue(b);
  }
  void sendMessage(const Bytes &m) {
    if (noise) {
      Bytes in; put(in, GL_INTENT | static_cast<uint32_t>(rcid) << 16); put(in, 2); put(in, 64); put(in, 1); put(in, 64); put(in, 2);
      queue(in);
      cmd(GL_SIGNALS, rcid, 3); cmd(GL_RX_DONE, rcid, 1); cmd(GL_READ_NOTIF, 0, 0); cmd(GL_RX_INTENT_REQ_ACK, rcid, 1);
    }
    if (splitAck && m.size() > 8) {
      data(GL_TX_DATA, Bytes(m.begin(), m.begin() + 8), static_cast<uint32_t>(m.size() - 8));
      data(GL_TX_DATA_CONT, Bytes(m.begin() + 8, m.end()), 0);
    } else {
      data(GL_TX_DATA, m, 0);
    }
  }
  Bytes ackFor(const Req &r) {
    Bytes a;
    const char *err = r.type == 0xdead0001u ? "resource does not exist" : r.type == 0xdead0002u ? "invalid key" : nullptr;
    if (fmt == RPM_FMT_V1) {
      put(a, 4u | (err ? RPM_V1_NACK : RPM_V1_ACK) << 16 | 1u << 24); put(a, r.msgId);
      if (err) put(a, 0x22);
      return a;
    }
    Bytes body; put(body, RPM_ACK_MSG_ID); put(body, 4); put(body, r.msgId);
    if (err) {
      const uint32_t n = static_cast<uint32_t>(strlen(err)) + 1;
      put(body, RPM_ACK_ERR); put(body, n);
      for (uint32_t k = 0; k < n; ++k) body.push_back(static_cast<uint8_t>(err[k]));
      while (body.size() % 4) body.push_back(0);
    }
    put(a, RPM_SERVICE_REQ); put(a, static_cast<uint32_t>(body.size()));
    a.insert(a.end(), body.begin(), body.end());
    return a;
  }
  void request(const Bytes &m) {
    Req r{};
    size_t at;
    uint32_t dataLen;
    if (fmt == RPM_FMT_V1) {
      if (m.size() < 16) { badData++; return; }
      const uint32_t h = get(m, 0);
      if ((h >> 24) != 1 || ((h >> 16) & 0xff) != 0 || (h & 0xffff) != m.size() - 4) badData++;
      r.msgId = get(m, 4); r.type = get(m, 8);
      const uint32_t d = get(m, 12);
      dataLen = d & 0xffff; r.id = (d >> 16) & 0xfff; r.set = d >> 28;
      at = 16;
    } else {
      if (m.size() < 28) { badData++; return; }
      if (get(m, 0) != RPM_SERVICE_REQ || get(m, 4) != m.size() - 8) badData++;
      r.msgId = get(m, 8); r.set = get(m, 12); r.type = get(m, 16); r.id = get(m, 20); dataLen = get(m, 24);
      at = 28;
    }
    if (at + dataLen != m.size()) badData++;
    while (at + 8 <= m.size()) {
      const uint32_t k = get(m, at), n = get(m, at + 4);
      uint64_t v = get(m, at + 8);
      if (n == 8) v |= static_cast<uint64_t>(get(m, at + 12)) << 32;
      r.kv.push_back({k, v});
      at += 8 + n;
    }
    reqs.push_back(r);
    if (dropAcks) { dropAcks--; return; }
    if (oversize) { oversize = false; data(GL_TX_DATA, Bytes(200, 0x5a), 0); }
    pending.push_back({tick + ackDelay, ackFor(r)});
  }
  void consume() {
    if (dead || tick < readStallTicks) return;
    uint32_t budget = consumePerTick;
    for (;;) {
      const uint32_t used = rpm_ring_used(txHead(), txTail(), txLen);
      if (used < 8) return;
      const Bytes h = readTx(0, used >= 16 ? 16 : 8);
      const uint16_t c = static_cast<uint16_t>(get(h, 0)), p1 = static_cast<uint16_t>(get(h, 0) >> 16);
      const uint32_t p2 = get(h, 4);
      uint32_t len = 8;
      if (c == GL_OPEN) len = 8 + p2;
      if (c == GL_TX_DATA) len = 16 + get(h, 8);
      const uint32_t padded = rpm_align8(len);
      if (padded > used || padded > budget) return;
      const Bytes pkt = readTx(0, padded);
      for (uint32_t k = len; k < padded; ++k) if (pkt[k]) badPad++;
      budget -= padded;
      txTail() = (txTail() + padded) % txLen;
      switch (c) {
        case GL_VERSION:
          versions++;
          cmd(GL_VERSION_ACK, GL_VERSION_1, peerFeatures);
          if (peerVersion && versions == 1) cmd(GL_VERSION, GL_VERSION_1, 0);
          if (remoteOpens && !sentOpen) {
            if (otherOpen) open(9, "glink_ssr");
            open(rcid, "rpm_requests");
            sentOpen = true;
          }
          break;
        case GL_VERSION_ACK: versionAcks++; break;
        case GL_OPEN:
          opens++;
          if (std::string(reinterpret_cast<const char *>(&pkt[8])) != "rpm_requests") badData++;
          appsLcid = p1;
          if (openAck) cmd(GL_OPEN_ACK, p1, 0);
          if (!sentOpen && openAck) { open(rcid, "rpm_requests"); sentOpen = true; }
          break;
        case GL_OPEN_ACK: if (p1 == rcid) gotOpenAck = true; break;
        case GL_CLOSE_ACK: closeAcks++; break;
        case GL_READ_NOTIF: readNotifs++; break;
        case GL_TX_DATA:
          if (p1 != appsLcid || !gotOpenAck || get(h, 12) != 0 || p2 != 0) badData++;
          request(Bytes(pkt.begin() + 16, pkt.begin() + len));
          break;
        default: unknown++; break;
      }
    }
  }
  bool writeRx(const Bytes &pkt) {
    const uint32_t padded = rpm_align8(static_cast<uint32_t>(pkt.size()));
    if (padded >= rpm_ring_free(rxHead(), rxTail(), rxLen)) return false;
    uint32_t pos = rxHead();
    for (uint32_t i = 0; i < padded; i += 4) {
      uint32_t w = 0;
      for (uint32_t k = 0; k < 4; ++k) if (i + k < pkt.size()) w |= static_cast<uint32_t>(pkt[i + k]) << (8 * k);
      ram[(rxDesc + 8 + pos) / 4] = w;
      pos = (pos + 4) % rxLen;
    }
    rxHead() = pos;
    return true;
  }
  void run() {
    if (dead) return;
    consume();
    while (!pending.empty() && pending.front().at <= tick) { sendMessage(pending.front().ack); pending.pop_front(); }
    while (!outbox.empty() && writeRx(outbox.front())) outbox.pop_front();
  }
};

static bool openLink(Fake &f, rpm_link &l, unsigned ring = RPM_RING_LINUX, int want = RPM_OK) {
  rpm_link_init(&l, &f.io, f.size, f.fmt);
  const int rc = rpm_link_open(&l, ring, 200000, 20000, 50);
  if (rc != want) printf("  open rc %d (%s), want %d\n", rc, rpm_err_name(rc), want);
  return rc == want;
}

static void testRing() {
  CHECK(rpm_ring_used(0, 0, 64) == 0);
  CHECK(rpm_ring_used(16, 0, 64) == 16);
  CHECK(rpm_ring_used(8, 56, 64) == 16);
  CHECK(rpm_ring_free(0, 0, 64) == 64);
  CHECK(rpm_ring_free(56, 8, 64) == 16);
  CHECK(rpm_ring_free(8, 16, 64) == 8);
  CHECK(rpm_align8(0) == 0 && rpm_align8(1) == 8 && rpm_align8(8) == 8 && rpm_align8(21) == 24);
}

static void testEncode() {
  uint8_t b[128];
  const rpm_kvp kv[2] = {{RPM_KEY_SWEN, 4, 1}, {RPM_KEY_UV, 4, 1800000}};
  uint32_t n = rpm_encode_request(b, sizeof b, RPM_FMT_V0, 5, RPM_SET_ACTIVE, RPM_RES_LDOA, 9, kv, 2);
  const uint32_t want[] = {0x00716572, 44, 5, 0, 0x616f646c, 9, 24, 0x6e657773, 4, 1, 0x00007675, 4, 1800000};
  CHECK(n == sizeof want);
  for (unsigned i = 0; n == sizeof want && i < sizeof want / 4; ++i) CHECK(rpm_get32(b + 4 * i) == want[i]);
  CHECK(!memcmp(b + 16, "ldoa", 4) && !memcmp(b + 28, "swen", 4) && !memcmp(b + 40, "uv", 2));
  // clk2 bimc KHz (sleep), v1: hdr len = data + 12, rsc id 12 bits, set in bits 28..31
  const rpm_kvp k1 = {RPM_KEY_KHZ, 4, 19200};
  n = rpm_encode_request(b, sizeof b, RPM_FMT_V1, 9, RPM_SET_SLEEP, RPM_RES_CLK2, 0x123, &k1, 1);
  CHECK(n == 28);
  CHECK(rpm_get32(b) == (12u + 12u) + (1u << 24));
  CHECK(rpm_get32(b + 4) == 9 && rpm_get32(b + 8) == RPM_RES_CLK2);
  CHECK(rpm_get32(b + 12) == (12u | 0x123u << 16 | 1u << 28));
  CHECK(!memcmp(b + 16, "KHz", 4) && rpm_get32(b + 20) == 4 && rpm_get32(b + 24) == 19200);
  // 8-byte value (bus bandwidth)
  const rpm_kvp bw = {RPM_KEY_BW, 8, 0x0000000123456789ull};
  n = rpm_encode_request(b, sizeof b, RPM_FMT_V0, 3, RPM_SET_ACTIVE, RPM_RES_BSLV, 0, &bw, 1);
  CHECK(n == 28 + 16 && rpm_get32(b + 24) == 16 && rpm_get32(b + 32) == 8 && rpm_get32(b + 36) == 0x23456789 && rpm_get32(b + 40) == 1);
  // refusals
  CHECK(!rpm_encode_request(b, sizeof b, RPM_FMT_V0, 3, RPM_SET_ACTIVE, RPM_RES_LDOA, 9, kv, 0));
  const rpm_kvp odd = {RPM_KEY_UV, 3, 1};
  CHECK(!rpm_encode_request(b, sizeof b, RPM_FMT_V0, 3, RPM_SET_ACTIVE, RPM_RES_LDOA, 9, &odd, 1));
  CHECK(!rpm_encode_request(b, sizeof b, RPM_FMT_V1, 3, RPM_SET_ACTIVE, RPM_RES_LDOA, 0x1000, kv, 1));
  CHECK(!rpm_encode_request(b, sizeof b, RPM_FMT_V0, 3, 2, RPM_RES_LDOA, 9, kv, 1));
  CHECK(!rpm_encode_request(b, 40, RPM_FMT_V0, 3, RPM_SET_ACTIVE, RPM_RES_LDOA, 9, kv, 2));
  const rpm_kvp five[5] = {k1, k1, k1, k1, k1};
  CHECK(!rpm_encode_request(b, sizeof b, RPM_FMT_V0, 3, RPM_SET_ACTIVE, RPM_RES_LDOA, 9, five, 5));
}

static void testDecode() {
  Fake f;
  uint32_t id, code;
  char t[32];
  Fake::Req r{}; r.msgId = 77;
  Bytes a = f.ackFor(r);
  CHECK(a.size() == 20);
  CHECK(rpm_decode_ack(a.data(), 20, RPM_FMT_V0, &id, &code, t, sizeof t) == RPM_OK && id == 77 && !t[0]);
  r.type = 0xdead0001u; a = f.ackFor(r);
  CHECK(rpm_decode_ack(a.data(), static_cast<uint32_t>(a.size()), RPM_FMT_V0, &id, &code, t, sizeof t) == RPM_ERR_INV_RSC && id == 77);
  CHECK(!strcmp(t, "resource does not exist"));
  r.type = 0xdead0002u; a = f.ackFor(r);
  CHECK(rpm_decode_ack(a.data(), static_cast<uint32_t>(a.size()), RPM_FMT_V0, &id, &code, t, sizeof t) == RPM_ERR_NACK && !strcmp(t, "invalid key"));
  CHECK(rpm_decode_ack(a.data(), 7, RPM_FMT_V0, &id, &code, t, sizeof t) == RPM_ERR_BADACK);
  Bytes noid; put(noid, RPM_SERVICE_REQ); put(noid, 8); put(noid, 0x11223344); put(noid, 0);
  CHECK(rpm_decode_ack(noid.data(), 16, RPM_FMT_V0, &id, &code, t, sizeof t) == RPM_ERR_BADACK);
  Bytes lie; put(lie, RPM_SERVICE_REQ); put(lie, 12); put(lie, RPM_ACK_MSG_ID); put(lie, 400); put(lie, 5);
  CHECK(rpm_decode_ack(lie.data(), 20, RPM_FMT_V0, &id, &code, t, sizeof t) == RPM_ERR_BADACK);
  f.fmt = RPM_FMT_V1; r.type = 0; a = f.ackFor(r);
  CHECK(rpm_decode_ack(a.data(), 8, RPM_FMT_V1, &id, &code, t, sizeof t) == RPM_OK && id == 77);
  r.type = 0xdead0001u; a = f.ackFor(r);
  CHECK(rpm_decode_ack(a.data(), 12, RPM_FMT_V1, &id, &code, t, sizeof t) == RPM_ERR_NACK && code == 0x22);
  Bytes other; put(other, 4u | 0u << 16 | 1u << 24); put(other, 9);
  CHECK(rpm_decode_ack(other.data(), 8, RPM_FMT_V1, &id, &code, t, sizeof t) == RPM_ERR_BADACK);
}

static int vote(rpm_link &l, unsigned set, uint32_t type, uint32_t id, uint32_t key, uint32_t value, rpm_result *res = nullptr) {
  const rpm_kvp kv = {key, 4, value};
  return rpm_link_request(&l, set, type, id, &kv, 1, 100000, 50, res);
}

static void testSessionRemoteOpen() {
  Fake f; f.toc(); f.otherOpen = true;
  rpm_link l;
  CHECK(openLink(f, l));
  CHECK(l.up && !l.local_first && l.rcid == 7 && l.lcid == RPM_LCID && l.version_acked && l.other_opens == 1);
  CHECK(f.versions == 1 && f.versionAcks == 1 && f.opens == 1 && f.gotOpenAck && f.appsLcid == RPM_LCID);
  rpm_result res;
  const rpm_kvp kv[2] = {{RPM_KEY_SWEN, 4, 1}, {RPM_KEY_UV, 4, 1304000}};
  CHECK(rpm_link_request(&l, RPM_SET_ACTIVE, RPM_RES_LDOA, 17, kv, 2, 100000, 50, &res) == RPM_OK);
  CHECK(res.msg_id == 2 && !res.text[0]);
  CHECK(f.reqs.size() == 1 && f.reqs[0].type == RPM_RES_LDOA && f.reqs[0].id == 17 && f.reqs[0].set == RPM_SET_ACTIVE &&
        f.reqs[0].msgId == 2 && f.reqs[0].kv.size() == 2 && f.reqs[0].kv[0].first == RPM_KEY_SWEN && f.reqs[0].kv[1].second == 1304000);
  CHECK(vote(l, RPM_SET_SLEEP, RPM_RES_CLK0, RPM_CLK0_XO, RPM_KEY_ENAB, 1, &res) == RPM_OK && res.msg_id == 3);
  CHECK(f.reqs.size() == 2 && f.reqs[1].set == RPM_SET_SLEEP);
  CHECK(f.badWrites == 0 && f.badReads == 0 && f.badPad == 0 && f.badData == 0 && f.unknown == 0);
  CHECK(l.stale_acks == 0 && l.acks == 2);
}

static void testSessionLocalOpen() {
  Fake f; f.toc(); f.remoteOpens = false; f.peerVersion = false;
  rpm_link l;
  CHECK(openLink(f, l));
  CHECK(l.up && l.local_first && f.opens == 1 && f.gotOpenAck);
  CHECK(f.us >= 20000);                      // waited local_after_us for the RPM to open first
  CHECK(vote(l, RPM_SET_ACTIVE, RPM_RES_CLK2, RPM_CLK2_BIMC, RPM_KEY_KHZ, RPM_KHZ_MAX) == RPM_OK);
  CHECK(f.badWrites == 0 && f.badPad == 0 && f.badData == 0);
}

static void testV1() {
  Fake f; f.toc(); f.fmt = RPM_FMT_V1;
  rpm_link l;
  CHECK(openLink(f, l));
  CHECK(vote(l, RPM_SET_ACTIVE, RPM_RES_CLK1, RPM_CLK1_SNOC, RPM_KEY_KHZ, 19200) == RPM_OK);
  rpm_result res;
  CHECK(vote(l, RPM_SET_ACTIVE, 0xdead0001u, 1, RPM_KEY_ENAB, 1, &res) == RPM_ERR_NACK && res.code == 0x22);
  CHECK(f.badData == 0 && f.reqs.size() == 2 && f.reqs[0].id == RPM_CLK1_SNOC);
}

static void testNacks() {
  Fake f; f.toc();
  rpm_link l;
  CHECK(openLink(f, l));
  rpm_result res;
  CHECK(vote(l, RPM_SET_ACTIVE, 0xdead0001u, 1, RPM_KEY_ENAB, 1, &res) == RPM_ERR_INV_RSC && !strcmp(res.text, "resource does not exist"));
  CHECK(vote(l, RPM_SET_ACTIVE, 0xdead0002u, 1, RPM_KEY_ENAB, 1, &res) == RPM_ERR_NACK && !strcmp(res.text, "invalid key"));
  CHECK(vote(l, RPM_SET_ACTIVE, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1, &res) == RPM_OK && !res.text[0]);
}

// Small FIFOs, many requests: both rings wrap many times; noise commands, split acks, delayed acks.
static void testWrapStress() {
  Fake f; f.txLen = 0x100; f.rxLen = 0xc0; f.rxDesc = 0x300; f.toc();
  rpm_link l;
  CHECK(openLink(f, l));
  unsigned ok = 0;
  for (unsigned i = 0; i < 400; ++i) {
    f.noise = i % 3 == 0;
    f.splitAck = i % 2 == 0;
    f.ackDelay = i % 5;
    const rpm_kvp kv[3] = {{RPM_KEY_SWEN, 4, i & 1}, {RPM_KEY_UV, 4, 1000000 + i}, {RPM_KEY_BW, 8, 0x100000000ull * i + 7}};
    const unsigned n = 1 + i % 3;
    rpm_result res;
    const int rc = rpm_link_request(&l, i & 1, RPM_RES_LDOA, i % 30, kv, n, 200000, 50, &res);
    if (rc == RPM_OK && f.reqs.size() == i + 1 && f.reqs[i].kv.size() == n && f.reqs[i].kv[n - 1].second == kv[n - 1].value &&
        f.reqs[i].msgId == res.msg_id && f.reqs[i].set == (i & 1))
      ok++;
    else
      printf("  wrap: request %u rc %d\n", i, rc);
  }
  CHECK(ok == 400);
  CHECK(l.intents > 100 && l.signals > 100 && l.read_notifs > 100 && l.stale_acks == 0);
  CHECK(f.badWrites == 0 && f.badReads == 0 && f.badPad == 0 && f.badData == 0 && f.unknown == 0);
  CHECK(l.kicks >= 400 + l.read_notifs);
}

static void testTimeouts() {
  {   // RPM never reads: no VERSION_ACK
    Fake f; f.toc(); f.dead = true;
    rpm_link l;
    CHECK(openLink(f, l, RPM_RING_LINUX, RPM_ERR_VERSION));
    CHECK(!l.up && vote(l, 0, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1) == RPM_ERR_CLOSED);
  }
  {   // version ok, the channel never opens
    Fake f; f.toc(); f.remoteOpens = false; f.openAck = false;
    rpm_link l;
    CHECK(openLink(f, l, RPM_RING_LINUX, RPM_ERR_OPEN));
    CHECK(l.version_acked && l.open_sent && !l.up);
  }
  {   // one ack lost: ack timeout; the next request works; a late ack counts as stale
    Fake f; f.toc();
    rpm_link l;
    CHECK(openLink(f, l));
    f.dropAcks = 1;
    CHECK(vote(l, 0, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1) == RPM_ERR_ACK);
    CHECK(vote(l, 0, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1) == RPM_OK);
    f.ackDelay = 100000;   // ack arrives after the timeout
    rpm_result res;
    const rpm_kvp en = {RPM_KEY_ENAB, 4, 1};
    CHECK(rpm_link_request(&l, 0, RPM_RES_CLK0, 0, &en, 1, 2000, 50, &res) == RPM_ERR_ACK);
    f.ackDelay = 0;
    f.pending.front().at = 0;
    CHECK(vote(l, 0, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1) == RPM_OK && l.stale_acks == 1);
  }
}

static void testProtocolErrors() {
  {   // unknown command from the RPM: broken, nothing more read or written
    Fake f; f.toc();
    rpm_link l;
    CHECK(openLink(f, l));
    f.cmd(0x55, 0, 0); f.run();
    CHECK(vote(l, 0, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1) == RPM_ERR_PROTO && l.broken && l.bad_cmd == 0x55);
    CHECK(vote(l, 0, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1) == RPM_ERR_CLOSED);
  }
  {   // corrupt rx head
    Fake f; f.toc();
    rpm_link l;
    CHECK(openLink(f, l));
    f.rxHead() = f.rxLen + 8;
    CHECK(rpm_poll(&l) == RPM_ERR_PROTO && l.broken);
  }
  {   // remote close: CLOSE_ACK, request refused, a new session works
    Fake f; f.toc();
    rpm_link l;
    CHECK(openLink(f, l));
    f.cmd(GL_CLOSE, f.rcid, 0); f.run();
    CHECK(vote(l, 0, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1) == RPM_ERR_CLOSED || !l.up);
    CHECK(f.closeAcks == 1 && l.closes == 1);
    f.sentOpen = false; f.gotOpenAck = false;
    CHECK(openLink(f, l, RPM_RING_RESUME));   // a reopen keeps the indices both sides use
    CHECK(vote(l, 0, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1) == RPM_OK);
  }
  {   // oversize unsolicited message (> 128 bytes): dropped, the real ack still matched
    Fake f; f.toc();
    rpm_link l;
    CHECK(openLink(f, l));
    f.oversize = true;
    CHECK(vote(l, 0, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1) == RPM_OK && l.dropped == 1);
  }
}

static void testToc() {
  rpm_link l;
  { Fake f; f.toc(false); rpm_link_init(&l, &f.io, f.size, 0); CHECK(rpm_parse_toc(&l) == RPM_ERR_TOC); CHECK(f.badWrites == 0); }
  { Fake f; f.toc(true, false); rpm_link_init(&l, &f.io, f.size, 0); CHECK(rpm_parse_toc(&l) == RPM_ERR_FIFO); }
  { Fake f; f.toc(); f.ram[(f.size - RPM_TOC_SIZE) / 4 + 1] = 100; rpm_link_init(&l, &f.io, f.size, 0); CHECK(rpm_parse_toc(&l) == RPM_ERR_TOC); }
  { Fake f; f.rxDesc = f.size - 0x80; f.toc(); rpm_link_init(&l, &f.io, f.size, 0);   // r2ap runs past the RAM
    CHECK(rpm_parse_toc(&l) == RPM_ERR_FIFO && l.toc_bad == 1); }
  { Fake f; f.rxDesc = 0x200; f.toc(); rpm_link_init(&l, &f.io, f.size, 0); CHECK(rpm_parse_toc(&l) == RPM_ERR_FIFO); }   // overlap
  { Fake f; f.toc(); f.txTail() = 0x40; f.txHead() = 0x48; f.rxTail() = 8; f.rxHead() = 0x10;
    rpm_link_init(&l, &f.io, f.size, 0);
    CHECK(rpm_parse_toc(&l) == RPM_OK && l.tx.desc == 0x100 && l.tx.len == 0x400 && l.rx.desc == 0x600 && l.rx.len == 0x400);
    CHECK(l.tx_tail0 == 0x40 && l.tx_head0 == 0x48 && l.rx_tail0 == 8 && l.rx_head0 == 0x10 && f.badWrites == 0);
    CHECK(f.txHead() == 0x48 && f.rxTail() == 8); }   // parse writes nothing
}

static void testRingModes() {
  {   // Linux: our indices reset to 0
    Fake f; f.toc();
    rpm_link l;
    CHECK(openLink(f, l));
    CHECK(l.tx_head0 == 0 && f.txTail() == f.txHead());
  }
  {   // resume: tx head kept at 0x40 (RPM tail there too), stale RPM->apps bytes dropped (tail jumps to head)
    Fake f; f.toc();
    f.txTail() = f.txHead() = 0x40;
    f.rxTail() = 0; f.rxHead() = 0x30;
    for (unsigned i = 0; i < 12; ++i) f.ram[(f.rxDesc + 8) / 4 + i] = 0x55;   // garbage that would be an unknown command
    rpm_link l;
    CHECK(openLink(f, l, RPM_RING_RESUME));
    CHECK(l.tx_head0 == 0x40 && l.rx_head0 == 0x30 && !l.broken && l.bad_cmd == 0);
    CHECK(vote(l, 0, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1) == RPM_OK);
    CHECK(f.badWrites == 0 && f.badData == 0);
  }
  {   // resume refuses misaligned indices
    Fake f; f.toc(); f.txHead() = f.txTail() = 0x44;
    rpm_link l;
    CHECK(openLink(f, l, RPM_RING_RESUME, RPM_ERR_PROTO));
  }
}

static void testTxFull() {
  {   // the RPM reads late: the first request times out still queued, the second finds the FIFO full, sends
      // READ_NOTIF and goes through once the RPM reads again; the first ack then arrives stale
    Fake f; f.txLen = 0x80; f.toc();
    rpm_link l;
    CHECK(openLink(f, l));
    f.readStallTicks = f.tick + 200;
    const rpm_kvp kv[2] = {{RPM_KEY_SWEN, 4, 1}, {RPM_KEY_UV, 4, 3200000}};
    CHECK(rpm_link_request(&l, 0, RPM_RES_LDOA, 23, kv, 2, 2000, 50, nullptr) == RPM_ERR_ACK);
    CHECK(rpm_link_request(&l, 0, RPM_RES_LDOA, 23, kv, 2, 100000, 50, nullptr) == RPM_OK);
    CHECK(f.readNotifs == 1 && l.stale_acks == 1 && f.reqs.size() == 2);
    CHECK(f.badPad == 0 && f.badData == 0 && f.badWrites == 0);
  }
  {   // fill the FIFO directly: never completely (head == tail would look empty)
    Fake f; f.txLen = 0x40; f.toc(); f.dead = true;
    rpm_link l;
    rpm_link_init(&l, &f.io, f.size, 0);
    CHECK(rpm_parse_toc(&l) == RPM_OK);
    uint8_t m[24] = {1};
    CHECK(rpm_tx_try(&l, m, 24) == RPM_OK && rpm_tx_try(&l, m, 24) == RPM_OK);   // 48 of 64 used
    CHECK(rpm_tx_try(&l, m, 16) == RPM_ERR_TXFULL);                              // would leave 0 free
    CHECK(rpm_tx_try(&l, m, 8) == RPM_OK && f.txHead() == 56);
  }
  {   // the RPM never reads: tx-full after the timeout
    Fake f; f.txLen = 0x80; f.toc();
    rpm_link l;
    CHECK(openLink(f, l));
    f.readStallTicks = 0xffffffff;
    int rc = RPM_OK;
    for (unsigned i = 0; i < 6 && rc == RPM_OK; ++i) rc = vote(l, 0, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1);
    CHECK(rc == RPM_ERR_ACK || rc == RPM_ERR_TXFULL);
    rc = RPM_OK;
    for (unsigned i = 0; i < 4 && rc != RPM_ERR_TXFULL; ++i) rc = vote(l, 0, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1);
    CHECK(rc == RPM_ERR_TXFULL);
  }
}

static void testMsgIdWrap() {
  Fake f; f.toc();
  rpm_link l;
  CHECK(openLink(f, l));
  l.next_id = 0xffffffffu;
  rpm_result r;
  CHECK(vote(l, 0, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1, &r) == RPM_OK && r.msg_id == 0xffffffffu);
  CHECK(vote(l, 0, RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1, &r) == RPM_OK && r.msg_id == 2);
}

static void testBoard() {
  for (unsigned i = 0; i < 4; ++i) {
    CHECK(!strcmp(kRpmBtLdo[i].name, kBtRails[i].name));
    CHECK(bt_ldo_mv_ok(&kBtRails[i], kRpmBtLdo[i].uv / 1000));            // inside bt_wcn3990's range
    CHECK(kRpmBtLdo[i].id == static_cast<uint32_t>((kBtRails[i].base - 0x4000) / 0x100 + 1));   // same LDO as SPMI
  }
  CHECK(kRpmBtLdo[0].uv == 1800000 && kRpmBtLdo[1].uv == 1704000 && kRpmBtLdo[2].uv == 1304000 && kRpmBtLdo[3].uv == 3200000);
  char s[5];
  rpm_fourcc(RPM_RES_LDOA, s); CHECK(!strcmp(s, "ldoa"));
  rpm_fourcc(RPM_RES_CLK2, s); CHECK(!strcmp(s, "clk2"));
  rpm_fourcc(RPM_KEY_UV, s); CHECK(!strcmp(s, "uv"));
  rpm_fourcc(RPM_KEY_KHZ, s); CHECK(!strcmp(s, "KHz"));
  rpm_fourcc(RPM_KEY_ENAB, s); CHECK(!strcmp(s, "Enab"));
  rpm_fourcc(RPM_RES_BSLV, s); CHECK(!strcmp(s, "bslv"));
  rpm_fourcc(RPM_TOC_MAGIC, s); CHECK(!strcmp(s, "0trg"));   // "grt0" read as a big-endian word
  CHECK(kRpmGpuVotes[0].type == RPM_RES_CLK0 && kRpmGpuVotes[0].id == 1 && kRpmGpuVotes[0].sleep);
  CHECK(kRpmGpuVotes[1].type == RPM_RES_CLK2 && kRpmGpuVotes[1].id == 0 && !kRpmGpuVotes[1].sleep);
  CHECK(kRpmGpuVotes[2].type == RPM_RES_CLK1 && kRpmGpuVotes[2].id == 2 && kRpmGpuVotes[3].id == 1);
  for (int rc = 0; rc >= -13; --rc) CHECK(strcmp(rpm_err_name(rc), "?"));
  CHECK(!strcmp(rpm_err_name(-99), "?"));
}

int main() {
  testRing();
  testEncode();
  testDecode();
  testSessionRemoteOpen();
  testSessionLocalOpen();
  testV1();
  testNacks();
  testWrapStress();
  testTimeouts();
  testProtocolErrors();
  testToc();
  testRingModes();
  testTxFull();
  testMsgIdWrap();
  testBoard();
  printf("rpm_host: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
