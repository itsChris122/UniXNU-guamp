// Host test of the ADSP transports of audio phase 2: GuampSmemWrite.inc (the SMEM layer shared with Wi-Fi R1),
// GuampSmemLock.inc (the one SMEM lock, local + TCSR mutex), GuampSmp2pEdge.inc, GuampGlinkEdge.inc,
// GuampApr.inc and the kernel glue GuampAdspIpc.inc itself (prepare / onReady / service / run), against a simulated
// 2 MiB SMEM (v12 partitions, as XBL leaves it) and a simulated ADSP written independently from the Linux sources:
// it allocates its SMP2P item 429 (ready, restart-done) and, once the apps GLINK descriptor 478 exists, its RX FIFO
// 480 (small, to force wrap-around), speaks GLINK (own VERSION, OPEN of IPCRTR and apr_audio_svc, intents, SIGNALS,
// fragmented DATA, RX_INTENT_REQ for a reply bigger than the apps intents) and answers APR q6core like the AVS
// firmware (state 0 twice, then 1; framework version with 40 services = 532 bytes). Then unit tests of the edge
// cases (allocation layout, EEXIST, lock timeout, full partition, SMP2P layout/negotiation, TX FIFO full ->
// READ_NOTIF, protocol errors, APR parsing). No device access. Build: see run-pas-tests.sh.
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include <vector>
#include <string>
#include "../GuampExt4.inc"
#include "../GuampFirmware.inc"
#include "../GuampPasCore.inc"
#include "../GuampSmem.inc"

static std::string logs;
static uint64_t gNowNs;
static void IOLog(const char *fmt, ...) {
  char b[1024];
  va_list args;
  va_start(args, fmt);
  vsnprintf(b, sizeof(b), fmt, args);
  va_end(args);
  logs += b;
  if (getenv("IPC_LOG")) fputs(b, stdout);
}
static size_t heapBytes;
static void *IOMallocZero(size_t n) { heapBytes += n; return calloc(1, n); }
static void IOFree(void *p, size_t n) { heapBytes -= n; free(p); }
static uint64_t mach_absolute_time() { return gNowNs; }
static void absolutetime_to_nanoseconds(uint64_t t, uint64_t *ns) { *ns = t; }
struct IOLock { bool held; unsigned takes; };
static bool gRemoteHolds;
static unsigned lockAllocs;
static IOLock *IOLockAlloc() { lockAllocs++; return static_cast<IOLock *>(calloc(1, sizeof(IOLock))); }
static void IOLockFree(IOLock *l) { free(l); }
static void IOLockLock(IOLock *l) { if (l->held) { fprintf(stderr, "IOLock taken twice\n"); abort(); } l->held = true; l->takes++; }
static void IOLockUnlock(IOLock *l) { if (!l->held) { fprintf(stderr, "IOLock released twice\n"); abort(); } l->held = false; }
static void IOSleep(unsigned ms);
static std::vector<uint8_t> smemImage(0x200000), apcsPage(0x1000), mutexPage(0x1000);
static unsigned barriers;
static void guampModemBarrier() {
  barriers++;
  if (gRemoteHolds && mutexPage[0] == 1) mutexPage[0] = 2;   // the remote (id 2) holds the TCSR mutex
}
static unsigned long guamp_phys_base = 0x80000000ul, guamp_phys_size = 0xb8000000ul;
namespace wifiprep { constexpr uint64_t kSmemPhys = 0x46000000, kSmemBytes = 0x200000; }
static volatile uint8_t *mapPhysical(uint64_t physical, uint64_t length) {
  if (physical == wifiprep::kSmemPhys && length == wifiprep::kSmemBytes) return smemImage.data();
  if (physical == 0x0f111000 && length == 0x1000) return apcsPage.data();
  if (physical == 0x343000 && length == 0x1000) return mutexPage.data();
  fprintf(stderr, "unexpected mapping %llx + %llx\n", (unsigned long long)physical, (unsigned long long)length);
  abort();
}
#include "../GuampAdspIpc.inc"

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); exit(1); } } while (0)

// ---- simulated SMEM layout (v12, partition table in the last 4 KiB) ----------------------------------------------
static uint32_t rd(const std::vector<uint8_t> &m, uint32_t at) { uint32_t v; memcpy(&v, &m[at], 4); return v; }
static void wr(std::vector<uint8_t> &m, uint32_t at, uint32_t v) { memcpy(&m[at], &v, 4); }
constexpr uint32_t kPartAdsp = 0x10000, kPartAdspBytes = 0x10000, kPartGlobal = 0x30000, kPartGlobalBytes = 0x8000;
constexpr uint32_t kPartModem = 0x40000, kPartModemBytes = 0x8000;
static void partition(std::vector<uint8_t> &m, uint32_t index, uint32_t at, uint32_t bytes, uint16_t h0, uint16_t h1) {
  const uint32_t table = static_cast<uint32_t>(m.size()) - 4096, e = table + 32 + 48 * index;
  wr(m, e, at);
  wr(m, e + 4, bytes);
  wr(m, e + 8, 0);
  wr(m, e + 12, h0 | (static_cast<uint32_t>(h1) << 16));
  wr(m, e + 16, 64);
  wr(m, at, 0x54525024);
  wr(m, at + 4, h0 | (static_cast<uint32_t>(h1) << 16));
  wr(m, at + 8, bytes);
  wr(m, at + 12, 32);
  wr(m, at + 16, bytes);
}
static void buildSmem(std::vector<uint8_t> &m) {
  std::fill(m.begin(), m.end(), 0);
  wr(m, 64 + 28, 12u << 16);
  wr(m, 192, 1);
  const uint32_t table = static_cast<uint32_t>(m.size()) - 4096;
  wr(m, table, 0x434f5424);
  wr(m, table + 4, 1);
  wr(m, table + 8, 3);
  partition(m, 0, kPartAdsp, kPartAdspBytes, 0, 2);
  partition(m, 1, kPartGlobal, kPartGlobalBytes, 0xfffe, 0xfffe);
  partition(m, 2, kPartModem, kPartModemBytes, 0, 1);
  wr(m, table + 32 + 48 * 3, 0x49494953);
  wr(m, table + 32 + 48 * 3 + 12, 512);
}
// independent qcom_smem_alloc_private / qcom_smem_get_private of the simulated remote
static uint32_t remoteAlloc(std::vector<uint8_t> &m, uint32_t part, uint16_t item, uint32_t bytes) {
  const uint32_t freeAt = rd(m, part + 12), padded = (bytes + 7) & ~7u, hdr = part + freeAt;
  wr(m, hdr, 0xa5a5u | (static_cast<uint32_t>(item) << 16));
  wr(m, hdr + 4, padded);
  wr(m, hdr + 8, padded - bytes);
  wr(m, hdr + 12, 0);
  wr(m, part + 12, freeAt + 16 + padded);
  return hdr + 16;
}
static bool remoteGet(const std::vector<uint8_t> &m, uint32_t part, uint16_t item, uint32_t *at, uint32_t *bytes) {
  const uint32_t end = rd(m, part + 12);
  for (uint32_t e = 32; e + 16 <= end;) {
    const uint32_t w0 = rd(m, part + e), size = rd(m, part + e + 4), pad = rd(m, part + e + 8) & 0xffff;
    if ((w0 & 0xffff) != 0xa5a5) return false;
    if ((w0 >> 16) == item) { *at = part + e + 16; *bytes = size - pad; return true; }
    e += 16 + size;
  }
  return false;
}

// ---- simulated ADSP ------------------------------------------------------------------------------------------------
struct Peer {
  std::vector<uint8_t> &m = smemImage;
  uint32_t in429 = 0, desc = 0, txFifo = 0, txLen = 0, rxFifo = 0, rxLen = 1024;
  uint64_t readyAtMs = 300, handoverAtMs = 420;
  bool glinkUp = false, versionAcked = false, aprOpenSent = false, aprAcked = false, fatalSent = false;
  uint16_t aprLcid = 0;                 // apps' local id for apr_audio_svc
  static constexpr uint16_t kRcidApr = 7, kRcidQrtr = 3;
  struct Intent { uint32_t id, size; bool busy; };
  std::vector<Intent> appsIntents;
  std::vector<std::vector<uint8_t>> outbox;   // APR replies waiting for an apps intent
  bool requestOut = false;
  uint32_t stateQueries = 0, fwkQueries = 0, rxDones = 0, rxDoneReuse = 0, intentReqAcks = 0, opensSeen = 0, readNotif = 0;
  uint64_t fwkSentAtMs = 0;
  std::vector<uint8_t> assembly;
  uint32_t assemblyLeft = 0, lastIntent = 0, appsFeatures = 0, versionAcksRx = 0;
  uint32_t kicksSmp2p = 0, kicksGlink = 0;

  uint32_t nowMs() const { return static_cast<uint32_t>(gNowNs / 1000000); }
  // apps TX (479): head desc[1] (apps), tail desc[0] (peer)
  uint32_t txAvail() const {
    const uint32_t h = rd(m, desc + 4), t = rd(m, desc);
    return h >= t ? h - t : txLen - t + h;
  }
  void txPeek(uint32_t off, uint8_t *dst, uint32_t n) const {
    const uint32_t t = rd(m, desc);
    for (uint32_t i = 0; i < n; i++) dst[i] = m[txFifo + (t + off + i) % txLen];
  }
  void txAdvance(uint32_t n) { wr(m, desc, (rd(m, desc) + n) % txLen); }
  // peer -> apps (480): head desc[3] (peer), tail desc[2] (apps)
  bool put(const uint8_t *a, uint32_t an, const uint8_t *b = nullptr, uint32_t bn = 0) {
    const uint32_t h = rd(m, desc + 12), t = rd(m, desc + 8), used = h >= t ? h - t : rxLen - t + h;
    const uint32_t bytes = (an + bn + 7) & ~7u;
    if (rxLen - used <= bytes + 8) return false;
    uint32_t pos = h;
    for (uint32_t i = 0; i < bytes; i++) {
      m[rxFifo + pos] = i < an ? a[i] : (i - an < bn ? b[i - an] : 0);
      pos = (pos + 1) % rxLen;
    }
    wr(m, desc + 12, pos);
    return true;
  }
  bool cmd(uint16_t c, uint16_t p1, uint32_t p2, const uint8_t *x = nullptr, uint32_t n = 0) {
    uint8_t h[8];
    memcpy(h, &c, 2);
    memcpy(h + 2, &p1, 2);
    memcpy(h + 4, &p2, 4);
    return put(h, 8, x, n);
  }
  void smp2p() {
    if (!in429 && nowMs() >= 100) {   // the ADSP allocates and publishes its inbound item early in its boot
      uint32_t at = 0, bytes = 0;
      CHECK(remoteGet(m, kPartAdsp, 443, &at, &bytes) && bytes == 340);   // apps item exists before the remote runs
      in429 = remoteAlloc(m, kPartAdsp, 429, 340);
      wr(m, in429, 0x504d5324);
      wr(m, in429 + 8, 2);              // local 2, remote 0
      wr(m, in429 + 12, 16 | (1u << 16));
      wr(m, in429 + 16, 1);             // restart-done
      memcpy(&m[in429 + 20], "slave-kernel", 13);
      wr(m, in429 + 4, 1 | (1u << 8));
    }
    if (in429) {
      uint32_t v = 0;
      if (nowMs() >= readyAtMs) v |= 2;
      if (nowMs() >= handoverAtMs) v |= 4;
      if (fatalSent) v |= 1;
      wr(m, in429 + 36, v);
    }
  }
  void glinkStart() {
    uint32_t bytes = 0;
    if (glinkUp || !remoteGet(m, kPartAdsp, 478, &desc, &bytes)) return;
    CHECK(bytes == 32 && remoteGet(m, kPartAdsp, 479, &txFifo, &txLen) && txLen == 0x4000);
    rxFifo = remoteAlloc(m, kPartAdsp, 480, rxLen);
    wr(m, desc, 0);                     // the remote resets its read index of the apps FIFO (our tail)
    wr(m, desc + 12, 0);
    glinkUp = true;
    CHECK(cmd(0, 1, 1));                // VERSION 1, INTENT_REUSE
    const char q[] = "IPCRTR";
    CHECK(cmd(2, kRcidQrtr, sizeof(q), reinterpret_cast<const uint8_t *>(q), sizeof(q)));
  }
  void sendAprOpen() {
    if (aprOpenSent) return;
    const char a[] = "apr_audio_svc";
    CHECK(cmd(2, kRcidApr, sizeof(a), reinterpret_cast<const uint8_t *>(a), sizeof(a)));
    aprOpenSent = true;
    uint8_t pairs[16];
    const uint32_t v[4] = {512, 100, 512, 101};
    memcpy(pairs, v, 16);
    CHECK(cmd(5, kRcidApr, 2, pairs, 16));
    CHECK(cmd(15, kRcidApr, 0x3));     // SIGNALS
  }
  void reply(uint32_t token, uint32_t opcode, const std::vector<uint32_t> &payload) {
    std::vector<uint8_t> p(32 + 4 * payload.size());
    const uint16_t f = 0x0280, size = static_cast<uint16_t>(p.size());
    memcpy(&p[0], &f, 2);
    memcpy(&p[2], &size, 2);
    p[4] = 3; p[5] = 4; p[8] = 3; p[9] = 5;
    memcpy(&p[12], &token, 4);
    memcpy(&p[16], &opcode, 4);
    memcpy(&p[32], payload.data(), 4 * payload.size());
    outbox.push_back(p);
  }
  void apr(const std::vector<uint8_t> &p) {
    CHECK(p.size() == 32 && p[0] == 0x80 && p[1] == 2 && p[2] == 32 && p[4] == 3 && p[5] == 5 && p[8] == 3 && p[9] == 4);
    uint32_t token, opcode;
    memcpy(&token, &p[12], 4);
    memcpy(&opcode, &p[16], 4);
    if (opcode == 0x1290c) {
      stateQueries++;
      reply(token, 0x1290d, {stateQueries >= 3 ? 1u : 0u});
    } else if (opcode == 0x1292c) {
      fwkQueries++;
      std::vector<uint32_t> v = {2, 8, 3, 1, 40};
      for (uint32_t i = 0; i < 40; i++) { v.push_back(i + 1); v.push_back(i % 5); v.push_back(0); }
      reply(token, 0x1292d, v);      // 32 + 20 + 480 = 532 bytes: needs a bigger apps intent
      fwkSentAtMs = nowMs();
    } else {
      CHECK(!"unexpected APR opcode");
    }
  }
  void flushOutbox() {
    while (!outbox.empty()) {
      const auto &p = outbox.front();
      Intent *best = nullptr;
      for (auto &i : appsIntents)
        if (!i.busy && i.size >= p.size() && (!best || i.size < best->size)) best = &i;
      if (!best) {
        if (!requestOut) { CHECK(cmd(7, kRcidApr, static_cast<uint32_t>(p.size()))); requestOut = true; }
        return;
      }
      const uint32_t n = static_cast<uint32_t>(p.size()), first = n > 256 ? 256 : n;
      uint8_t h[16];
      const uint16_t c = 9, id = kRcidApr;
      uint32_t left = n - first;
      memcpy(h, &c, 2); memcpy(h + 2, &id, 2); memcpy(h + 4, &best->id, 4); memcpy(h + 8, &first, 4); memcpy(h + 12, &left, 4);
      if (!put(h, 16, p.data(), first)) return;
      if (left) {
        const uint16_t cc = 12;
        const uint32_t zero = 0;
        memcpy(h, &cc, 2); memcpy(h + 8, &left, 4); memcpy(h + 12, &zero, 4);
        CHECK(put(h, 16, p.data() + first, left));
      }
      best->busy = true;
      outbox.erase(outbox.begin());
    }
  }
  void glink() {
    if (!glinkUp) return;
    for (;;) {
      const uint32_t avail = txAvail();
      if (avail < 8) break;
      uint8_t h[16];
      txPeek(0, h, 8);
      uint16_t c, p1;
      uint32_t p2;
      memcpy(&c, h, 2); memcpy(&p1, h + 2, 2); memcpy(&p2, h + 4, 4);
      uint32_t bytes = 8;
      switch (c) {
        case 0: CHECK(p1 == 1 && p2 == 1); CHECK(cmd(1, 1, 1)); break;          // VERSION -> VERSION_ACK
        case 1: versionAcksRx++; appsFeatures = p2; CHECK(p1 == 1); versionAcked = true; break;
        case 2: {                                                                // OPEN
          const uint32_t n = p2 & 0xffff;
          bytes += (n + 7) & ~7u;
          char name[40] = {};
          txPeek(8, reinterpret_cast<uint8_t *>(name), n);
          CHECK(!strcmp(name, "apr_audio_svc"));
          opensSeen++;
          aprLcid = p1;
          CHECK(cmd(4, p1, 0));       // OPEN_ACK with the apps' id
          sendAprOpen();
          break;
        }
        case 4: CHECK(p1 == kRcidApr); aprAcked = true; break;                   // OPEN_ACK of our open
        case 5: {                                                                // INTENT
          CHECK(p1 == aprLcid && p2 >= 1 && p2 <= 5);
          bytes += 8 * p2;
          for (uint32_t i = 0; i < p2; i++) {
            uint32_t pair[2];
            txPeek(8 + 8 * i, reinterpret_cast<uint8_t *>(pair), 8);
            bool replaced = false;
            for (auto &x : appsIntents)
              if (x.id == pair[1]) { x = {pair[1], pair[0], false}; replaced = true; }
            if (!replaced) appsIntents.push_back({pair[1], pair[0], false});
          }
          break;
        }
        case 6: case 14: {                                                       // RX_DONE (w/ reuse)
          CHECK(p1 == aprLcid);
          bool found = false;
          for (size_t i = 0; i < appsIntents.size(); i++)
            if (appsIntents[i].id == p2) {
              CHECK(appsIntents[i].busy);
              found = true;
              if (c == 14) { appsIntents[i].busy = false; rxDoneReuse++; }
              else { appsIntents.erase(appsIntents.begin() + i); rxDones++; }
              break;
            }
          CHECK(found);
          break;
        }
        case 7: {                                                                // RX_INTENT_REQ from apps
          CHECK(p1 == aprLcid && p2 <= 512);
          const uint32_t v[4] = {512, 200 + p2 % 7, 0, 0};
          CHECK(cmd(5, kRcidApr, 1, reinterpret_cast<const uint8_t *>(v), 8));
          CHECK(cmd(8, kRcidApr, 1));
          break;
        }
        case 8: CHECK(p1 == aprLcid); intentReqAcks++; if (p2) requestOut = false; break;
        case 9: case 12: {                                                       // TX_DATA(_CONT)
          txPeek(0, h, 16);
          uint32_t chunk, left;
          memcpy(&chunk, h + 8, 4); memcpy(&left, h + 12, 4);
          bytes = 16 + ((chunk + 7) & ~7u);
          CHECK(p1 == aprLcid && (p2 == 100 || p2 == 101 || p2 >= 200));
          std::vector<uint8_t> d(chunk);
          txPeek(16, d.data(), chunk);
          if (c == 9) assembly.clear();
          assembly.insert(assembly.end(), d.begin(), d.end());
          lastIntent = p2;
          if (!left) {
            CHECK(cmd(14, kRcidApr, lastIntent));                                // RX_DONE_W_REUSE
            apr(assembly);
          }
          break;
        }
        case 13: readNotif++; break;
        default: CHECK(!"unexpected GLINK command from apps");
      }
      CHECK(avail >= bytes);
      txAdvance(bytes);
    }
    if (aprAcked) flushOutbox();
    if (fwkSentAtMs && outbox.empty() && nowMs() >= fwkSentAtMs + 200) fatalSent = true;   // ends run()
  }
  void step() {
    const uint32_t ipc = rd(apcsPage, 8);
    if (ipc & (1u << 10)) kicksSmp2p++;
    if (ipc & (1u << 8)) kicksGlink++;
    wr(apcsPage, 8, 0);
    smp2p();
    if (nowMs() >= readyAtMs) glinkStart();
    glink();
  }
};
static Peer *gPeer;
static void IOSleep(unsigned ms) {
  gNowNs += 1000000ull * ms;
  if (gPeer) gPeer->step();
  CHECK(gNowNs < 200ull * 1000000000ull);
}

// ---- the kernel glue end to end ------------------------------------------------------------------------------------
static void endToEnd() {
  buildSmem(smemImage);
  Peer peer;
  gPeer = &peer;
  adspipc::gLevel = 3;
  CHECK(adspipc::prepare(nullptr) == qipc::Ok);   // pas::Hooks::prepare: 0 = go on
  uint32_t at = 0, bytes = 0;
  CHECK(remoteGet(smemImage, kPartAdsp, 443, &at, &bytes) && bytes == 340);
  CHECK(rd(smemImage, at) == 0x504d5324 && rd(smemImage, at + 4) == (1u | (1u << 8)) && rd(smemImage, at + 8) == (2u << 16));
  CHECK(rd(smemImage, at + 12) == (16u | (3u << 16)));
  CHECK(!strcmp(reinterpret_cast<const char *>(&smemImage[at + 20]), "master-kernel"));
  CHECK(!strcmp(reinterpret_cast<const char *>(&smemImage[at + 40]), "rdbg"));
  CHECK(!strcmp(reinterpret_cast<const char *>(&smemImage[at + 60]), "sleepstate") && rd(smemImage, at + 76) == 0x1000);
  CHECK(rd(smemImage, at - 16) == (0xa5a5u | (443u << 16)) && rd(smemImage, at - 12) == 344 && rd(smemImage, at - 8) == 4);
  CHECK(rd(apcsPage, 8) == 1u << 10 && rd(mutexPage, 0) == 0);
  CHECK(!remoteGet(smemImage, kPartAdsp, 478, &at, &bytes));   // GLINK only after err-ready
  // the awaitReady loop of GuampAdsp.inc
  bool ready = false;
  for (unsigned t = 0; t < 2000 && !(ready && (rd(smemImage, peer.in429 + 36) & 4)); t += 50) {
    if (peer.in429 && (rd(smemImage, peer.in429 + 36) & 2) && !ready) { ready = true; adspipc::onReady(); }
    adspipc::service();
    IOSleep(50);
  }
  CHECK(ready && adspipc::gReady && adspipc::gGlink && adspipc::gGlink->running);
  CHECK(adspipc::gSmp2p.open && adspipc::gSmp2p.ssrEnabled && adspipc::gSmp2p.acks == 1);
  CHECK((rd(smemImage, adspipc::gSmp2p.outbound + 16) & 2) == 2);   // restart-done acknowledged
  adspipc::run();   // returns on the simulated err-fatal after the APR exchange
  if (getenv("IPC_LOG") == nullptr && logs.find("ADSP APR OK") == std::string::npos) fputs(logs.c_str(), stderr);
  CHECK(logs.find("GuampAdsp: ADSP APR OK (state 1 after 3 tries, fwk answered 2.8.3.1, 40 services)") != std::string::npos);
  CHECK(logs.find("glink open ignored (no local user) IPCRTR 3") != std::string::npos);
  CHECK(logs.find("glink signals rx apr_audio_svc 7 3") != std::string::npos);
  CHECK(logs.find("ipc ADSP err-fatal") != std::string::npos);
  CHECK(peer.stateQueries == 3 && peer.fwkQueries == 1 && peer.versionAcked && peer.appsFeatures == 1);
  CHECK(adspipc::gCore.listed == 24 && adspipc::gCore.services == 40 && adspipc::gCore.service[23].id == 24);
  CHECK(peer.rxDoneReuse == 3 && peer.rxDones == 1);   // 3 state answers reused, the big reply in a one-shot intent
  CHECK(peer.kicksSmp2p >= 1 && peer.kicksGlink >= 3);
  const qipc::Glink::Channel *c = adspipc::gGlink->channel(adspipc::gApr);
  uint32_t local = 0;
  for (const auto &li : c->intent) local += li.data ? 1 : 0;
  CHECK(local == 20 && c->rxPackets == 4 && c->txPackets == 4 && c->remote == Peer::kRcidApr);
  CHECK(adspipc::gGlink->ignoredOpens == 1 && adspipc::gGlink->rx.length == 1024);
  printf("end to end: SMP2P 443 + negotiation + SSR ack, GLINK version/open/intents/signals, APR state x3 + fwk "
         "(532-byte reply via RX_INTENT_REQ, fragmented, RX FIFO wrapped), ADSP APR OK, err-fatal exit\n");
  gPeer = nullptr;
}

// ---- unit tests ----------------------------------------------------------------------------------------------------
static std::vector<uint8_t> unitImage(0x200000);
static uint32_t uRd(void *, uint32_t at) { return rd(unitImage, at); }
static void uWr(void *, uint32_t at, uint32_t v) { wr(unitImage, at, v); }
static void uFence(void *) {}
static bool lockOk = true;
static unsigned locks, unlocks;
static bool uLock(void *) { locks++; return lockOk; }
static void uUnlock(void *) { unlocks++; }
static smem::Info unitInfo;
static std::vector<qipc::SmemArena::Note> notes;
static void uNote(void *, const qipc::SmemArena::Note &n) { notes.push_back(n); }
static qipc::SmemArena arena() {
  qipc::SmemArena a = {};
  a.mem = {nullptr, static_cast<uint32_t>(unitImage.size()), uRd, uWr, uFence};
  a.info = &unitInfo;
  a.lock = uLock;
  a.unlock = uUnlock;
  a.note = uNote;
  return a;
}
// The one SMEM lock: local lock taken first, owner 1 in the TCSR word, released to 0; remote owner -> 1 s timeout
static void sharedLock() {
  const unsigned allocs = lockAllocs;
  CHECK(guamp_smem_hwlock_acquire() && smemlock::gLocal && smemlock::gLocal->held && rd(mutexPage, 0) == 1);
  guamp_smem_hwlock_release();
  CHECK(!smemlock::gLocal->held && rd(mutexPage, 0) == 0 && lockAllocs == allocs);
  // A remote owner (id 2) that never lets go: every barrier after our write shows its id again
  const uint64_t t0 = gNowNs;
  gRemoteHolds = true;
  CHECK(!guamp_smem_hwlock_acquire() && !smemlock::gLocal->held && smemlock::gTimeouts == 1 && gNowNs - t0 >= 999000000ull);
  gRemoteHolds = false;
  wr(mutexPage, 0, 0);
  printf("smem lock: owner 1 written/read back, released to 0, local lock held across, remote owner -> timeout: ok\n");
}
static void allocation() {
  buildSmem(unitImage);
  qipc::SmemArena a = arena();
  uint32_t at = 0, again = 0;
  bool created = false;
  CHECK(a.allocate(2, 478, 32, &at, &created) == qipc::Ok && created && at == kPartAdsp + 32 + 16);
  CHECK(rd(unitImage, kPartAdsp + 12) == 32 + 16 + 32);
  CHECK(a.allocate(2, 479, 0x4000, &again, &created) == qipc::Ok && created);
  CHECK(a.allocate(2, 478, 32, &again, &created) == qipc::Ok && !created && again == at);     // -EEXIST: reused
  CHECK(notes.empty());                                                                       // same size: silent
  uint32_t actual = 0;
  CHECK(a.allocate(2, 478, 24, &again, &created, &actual) == qipc::Ok && !created && again == at && actual == 32);
  CHECK(notes.size() == 1 && notes[0].item == 478 && notes[0].wanted == 24 && notes[0].found == 32 && !notes[0].result);
  CHECK(a.allocate(2, 478, 40, &again, &created, &actual) == qipc::Range && actual == 32);  // too small: refused
  CHECK(notes.size() == 2 && notes[1].wanted == 40 && notes[1].found == 32 && notes[1].result == qipc::Range);
  CHECK(a.allocations == 3 - 1 && a.existing == 2 && rd(unitImage, kPartAdsp + 12) == 32 + 16 + 32 + 16 + 0x4000);
  CHECK(a.allocate(2, 5, 8, &again, &created) == qipc::Invalid);                              // fixed item
  CHECK(a.allocate(2, 600, 8, &again, &created) == qipc::Invalid);
  CHECK(a.allocate(2, 450, 0x20000, &again, &created) == qipc::NoMemory);                     // partition full
  CHECK(a.allocate(5, 450, 13, &again, &created) == qipc::Ok && created && again > kPartGlobal &&
        again < kPartGlobal + kPartGlobalBytes);                                              // no 0-5 partition
  CHECK(rd(unitImage, again - 8) == 3);                                                       // padding_data
  lockOk = false;
  const uint32_t before = rd(unitImage, kPartAdsp + 12);
  CHECK(a.allocate(2, 451, 8, &again, &created) == qipc::Timeout && rd(unitImage, kPartAdsp + 12) == before);
  lockOk = true;
  CHECK(locks == unlocks + 1);
  wr(unitImage, kPartAdsp + 32, 0x1234);                                                      // bad canary
  CHECK(a.allocate(2, 452, 8, &again, &created) == qipc::Protocol);
  wr(unitImage, 192, 0);
  CHECK(a.allocate(5, 452, 8, &again, &created) == qipc::Protocol);                           // not initialized
  printf("smem alloc: layout, EEXIST, larger item reused (actual size, note), smaller refused (note), fixed/out-of-range "
         "items, full, global fallback, lock timeout, bad canary, uninitialized: ok\n");
}
static unsigned unitKicks;
static void uKick(void *) { unitKicks++; }
static void smp2pUnit() {
  buildSmem(unitImage);
  qipc::SmemArena a = arena();
  qipc::Smp2pEdge e;
  const char *names[2] = {"master-kernel", "sleepstate"};
  const uint32_t values[2] = {0, 0x1000};
  const char *bad[1] = {"a-name-too-long-for-smp2p"};
  CHECK(e.start(&a, 0, 2, 443, 429, bad, values, 1, uKick, nullptr) == qipc::Invalid);
  CHECK(e.start(&a, 0, 2, 443, 429, names, values, 2, uKick, nullptr) == qipc::Ok && unitKicks == 1);
  smem::Smp2p s = {};
  bool has = true;
  uint32_t v = 7;
  CHECK(e.poll(&s, "slave-kernel", &v, &has) == qipc::Again && !has && !v);
  const uint32_t in = remoteAlloc(unitImage, kPartAdsp, 429, 340);
  wr(unitImage, in, 0x504d5324);
  wr(unitImage, in + 8, 2);
  wr(unitImage, in + 12, 16 | (1u << 16));
  memcpy(&unitImage[in + 20], "slave-kernel", 13);
  wr(unitImage, in + 36, 0x6);
  CHECK(e.poll(&s, "slave-kernel", &v, &has) == qipc::Again);                  // version 0: not published yet
  wr(unitImage, in + 4, 1);                                                    // features 0: no SSR ack
  CHECK(e.poll(&s, "slave-kernel", &v, &has) == qipc::Ok && has && v == 6 && e.open && !e.ssrEnabled);
  CHECK(rd(unitImage, e.outbound + 4) == 1u);                                  // features negotiated down
  wr(unitImage, in + 16, 1);
  CHECK(e.poll(&s, "slave-kernel", &v, &has) == qipc::Ok && !e.acks);          // restart-done ignored without SSR ack
  CHECK(e.update(0, 1, 1) == qipc::Ok && e.value(0) == 1 && unitKicks == 2);   // force-stop bit
  CHECK(e.update(0, 1, 1) == qipc::Ok && unitKicks == 2);                      // unchanged: no kick
  CHECK(e.update(2, 1, 1) == qipc::Invalid);
  wr(unitImage, in + 8, 3);                                                    // wrong pids
  CHECK(e.poll(&s, "slave-kernel", &v, &has) == qipc::Protocol);
  printf("smp2p: entry names, publication order, negotiation, update/kick, bad pids: ok\n");
}
static std::vector<std::string> traces;
static void uTrace(void *, int t, const char *name, uint32_t, uint32_t) { traces.push_back(std::string(qipc::Glink::traceName(t)) + " " + name); }
static void uData(void *, uint16_t, const uint8_t *, uint32_t) {}
static void *uAlloc(void *, uint32_t n) { return calloc(1, n); }
static void uFree(void *, void *p, uint32_t) { free(p); }
static const qipc::GlinkChannelSpec kSpec[1] = {{"apr_audio_svc", 1, {{0x200, 2}, {0, 0}, {0, 0}}}};
static void glinkStart(qipc::Glink &g, uint32_t txLen) {
  std::fill(unitImage.begin(), unitImage.end(), 0);
  qipc::GlinkPipe tx, rx;
  tx.mem = rx.mem = {nullptr, static_cast<uint32_t>(unitImage.size()), uRd, uWr, uFence};
  tx.tx = true; tx.tail = 0; tx.head = 4; tx.fifo = 0x100; tx.length = txLen;
  rx.tail = 8; rx.head = 12;
  wr(unitImage, 4, 0x48); wr(unitImage, 8, 0x30);                              // stale indices: reset by start
  const qipc::Allocator heap = {nullptr, uAlloc, uFree};
  CHECK(g.start(tx, rx, heap, kSpec, 1, nullptr, uKick, uTrace, uData) == qipc::Ok);
  CHECK(rd(unitImage, 4) == 0 && rd(unitImage, 8) == 0);
}
static void rxPut(uint32_t &head, uint16_t c, uint16_t p1, uint32_t p2) {
  wr(unitImage, 0x8000 + head, c | (static_cast<uint32_t>(p1) << 16));
  wr(unitImage, 0x8000 + head + 4, p2);
  head += 8;
  wr(unitImage, 12, head);
}
static void glinkUnit() {
  static qipc::Glink g;
  glinkStart(g, 64);
  CHECK(g.step() == qipc::Ok && rd(unitImage, 4) == 8);                        // VERSION written
  CHECK(g.attachRx(0x8001, 1024) == qipc::Invalid && g.attachRx(0x8000, 1024) == qipc::Ok);
  uint32_t head = 0;
  rxPut(head, 0, 1, 1);                                                        // remote VERSION -> ack, negotiated
  CHECK(g.step() == qipc::Ok && g.negotiated && rd(unitImage, 4) == 16);
  uint16_t id = 0;
  CHECK(g.open("not-in-table", &id) == qipc::Invalid);
  CHECK(g.open("apr_audio_svc", &id) == qipc::Ok && id == 1);
  CHECK(g.step() == qipc::Ok && rd(unitImage, 4) == 40);                       // OPEN 8 + 16 name bytes
  // 64-byte FIFO: 40 used, the remote does not read; the OPEN_ACK + remote OPEN make apps answer OPEN_ACK + 1 INTENT
  rxPut(head, 4, 1, 0);                                                        // OPEN_ACK of our open
  const char name[16] = "apr_audio_svc";
  wr(unitImage, 0x8000 + head, 2 | (5u << 16)); wr(unitImage, 0x8000 + head + 4, 14);
  memcpy(&unitImage[0x8000 + head + 8], name, 16);
  head += 24; wr(unitImage, 12, head);
  traces.clear();
  CHECK(g.step() == qipc::Ok);                                                 // TX full: READ_NOTIF in the reserve
  CHECK(g.qCount > 0 && g.notifySent);
  bool notified = false;
  for (const auto &t : traces) notified |= t == "read-notif tx ";
  CHECK(notified);
  wr(unitImage, 0, rd(unitImage, 4));                                          // the remote consumed everything
  CHECK(g.step() == qipc::Ok && g.qCount == 0 && g.ready(id));
  rxPut(head, 10, 0, 0);                                                       // unknown command: link stops
  CHECK(g.step() == qipc::Unsupported && !g.running && !g.ready(id));
  // DATA for an intent apps never advertised
  glinkStart(g, 4096);
  CHECK(g.attachRx(0x8000, 1024) == qipc::Ok);
  head = 0;
  rxPut(head, 0, 1, 1);
  CHECK(g.step() == qipc::Ok && g.open("apr_audio_svc", &id) == qipc::Ok);
  rxPut(head, 4, 1, 0);
  wr(unitImage, 0x8000 + head, 2 | (5u << 16)); wr(unitImage, 0x8000 + head + 4, 14);
  memcpy(&unitImage[0x8000 + head + 8], name, 16);
  head += 24; wr(unitImage, 12, head);
  CHECK(g.step() == qipc::Ok && g.ready(id));
  uint8_t pkt[32] = {};
  CHECK(g.send(id, pkt, 32) == qipc::Again && g.channel(id)->requestPending);   // no remote intent: RX_INTENT_REQ
  rxPut(head, 9, 5, 99);
  wr(unitImage, 0x8000 + head, 8); wr(unitImage, 0x8000 + head + 4, 0); head += 16; wr(unitImage, 12, head);   // chunk 8 + 8 data bytes
  CHECK(g.step() == qipc::Protocol && !g.running);
  printf("glink: index reset, rx attach checks, version, open, TX full -> READ_NOTIF, unknown command, bad intent: ok\n");
}
static void aprUnit() {
  uint8_t p[64];
  CHECK(apr::build(p, sizeof(p), 3, 0, 9, 0x1290c, nullptr, 0) == 32);
  CHECK(p[0] == 0x80 && p[1] == 2 && p[2] == 32 && p[5] == 5 && p[9] == 4 && p[12] == 9);
  CHECK(apr::build(p, sizeof(p), 3, 0, 9, 0x1290c, p, 3) == 0);                 // payload must be words
  apr::Header h;
  const uint8_t *pl;
  uint32_t n;
  CHECK(apr::parse(p, 32, &h, &pl, &n) && n == 0 && h.opcode == 0x1290c);
  CHECK(!apr::parse(p, 31, &h, &pl, &n));
  p[2] = 40;
  CHECK(!apr::parse(p, 32, &h, &pl, &n));                                      // pkt_size beyond the message
  p[2] = 32; p[0] = 0x70;
  CHECK(!apr::parse(p, 32, &h, &pl, &n));                                      // header of 7 words
  p[0] = 0x81;
  CHECK(!apr::parse(p, 32, &h, &pl, &n));                                      // version 1
  apr::Core core;
  int sent = 0;
  core.send = [](void *c, const uint8_t *, uint32_t) { ++*static_cast<int *>(c); return 0; };
  core.ctx = &sent;
  core.readyBudgetMs = 1000;
  core.begin(0);
  for (uint32_t t = 0; t <= 1100 && !core.finished(); t += 10) core.step(t);
  CHECK(core.phase == apr::Core::Failed && !core.stateAnswered && sent >= 6 && sent <= 8);   // 100 + 50 ms per try
  printf("apr: header build/parse, malformed packets, ready budget without answers: ok\n");
}

int main() {
  endToEnd();
  allocation();
  sharedLock();
  smp2pUnit();
  glinkUnit();
  aprUnit();
  printf("adsp_ipc_host: PASS\n");
  return 0;
}
