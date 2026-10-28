// SPDX-License-Identifier: BSD-2-Clause-Patent
// GLINK native protocol over the SMEM transport of one edge (apps side), audio phase 2. Design and interface of the
// Wi-Fi line's Glink (Codex sorgenti-wifi-modem GuampGlink.inc: GlinkPipe, start/open/send/step, single serialized
// worker), in namespace qipc, with the changes the ADSP edge needs:
//   - the RX FIFO (item 480) is allocated by the remote after it boots: start() needs only the TX FIFO and the
//     descriptor words, attachRx() adds the RX FIFO when it appears (qcom_glink_smem.c acquires it lazily too);
//   - channels and their RX intents come from a table (the DT "qcom,intents" of each channel node); a remote OPEN of a
//     channel not in the table is left unanswered, as Linux does when no driver binds it;
//   - RX_INTENT_REQ is answered like qcom_glink_handle_intent_req: granted at once if a reusable intent is big enough,
//     else a new single-use intent of the requested size is advertised first;
//   - a VERSION with version 0 is ignored (qcom_glink_receive_version); every protocol step is reported to trace().
// Host-tested by tests/adsp_ipc_host.cpp against an independent simulated remote. Protocol studied in the guamp kernel
// drivers/rpmsg/qcom_glink_native.c and qcom_glink_smem.c (no code copied):
//   descriptor item (32 bytes): [0] TX tail (remote writes), [1] TX head (apps), [2] RX tail (apps), [3] RX head
//   (remote); TX FIFO 479 = 16 KiB allocated by apps, RX FIFO 480 allocated by the remote; at register Linux sets RX
//   tail = TX head = 0. Messages are 8-byte {cmd:16, param1:16, param2:32} + payload, each padded to 8 bytes; the
//   writer keeps 16 bytes free (FIFO_FULL_RESERVE + TX_BLOCKED_CMD_RESERVE) and, when full, writes READ_NOTIF into
//   the reserve; the remote answers by kicking. Version 1, feature INTENT_REUSE (bit 0) for SMEM edges.
//   ids: OPEN, CLOSE, INTENT, RX_DONE, RX_INTENT_REQ(_ACK), TX_DATA carry the sender's local channel id; OPEN_ACK and
//   CLOSE_ACK carry the receiver's. TX_DATA: {cmd, lcid, intent id} + chunk_size + left_size, then the chunk.
#ifndef GUAMP_GLINK_EDGE_INC
#define GUAMP_GLINK_EDGE_INC
#include "GuampSmemWrite.inc"
namespace qipc {

struct GlinkPipe {
  Memory mem = {};
  uint32_t fifo = 0, length = 0, head = 0, tail = 0;   // offsets inside mem
  bool tx = false;
  bool attached() const { return length != 0; }
  int validate() const {
    if (!mem.read || !mem.write || !mem.barrier || (head & 3) || (tail & 3) || !mem.valid(head, 4) || !mem.valid(tail, 4))
      return Invalid;
    if (attached() && (length < 64 || (length & 7) || (fifo & 7) || !mem.valid(fifo, length))) return Invalid;
    return Ok;
  }
  int count(uint32_t *n) const {
    *n = 0;
    if (!attached()) return Again;
    mem.fence();
    const uint32_t h = mem.r32(head), t = mem.r32(tail);
    if (h >= length || t >= length || (h & 7) || (t & 7)) return Protocol;
    const uint32_t used = h >= t ? h - t : length - t + h;
    *n = tx ? (length - used >= 16 ? length - used - 16 : 0) : used;
    return Ok;
  }
  bool peek(uint32_t relative, uint8_t *dst, uint32_t n) const {
    if (relative > length || n > length - relative) return false;
    const uint32_t p = (mem.r32(tail) + relative) % length, first = n < length - p ? n : length - p;
    return mem.copyOut(fifo + p, dst, first) && mem.copyOut(fifo, dst + first, n - first);
  }
  int advance(uint32_t n) const {
    uint32_t avail = 0;
    const int e = count(&avail);
    if (e) return e;
    if (tx || (n & 7) || n > avail) return Range;
    mem.w32(tail, (mem.r32(tail) + n) % length);
    mem.fence();
    return Ok;
  }
  // a (header) + b (payload), zero-padded to 8 bytes, as aligned words; head published after the data
  int push(const uint8_t *a, uint32_t an, const uint8_t *b = nullptr, uint32_t bn = 0, bool reserve = false) const {
    if (!tx || an > length || bn > length - an || an + bn > length - 16) return Range;
    uint32_t avail = 0;
    int e = count(&avail);
    if (e) return e;
    if (reserve) {   // READ_NOTIF may use the TX_BLOCKED_CMD_RESERVE
      const uint32_t h = mem.r32(head), t = mem.r32(tail), used = h >= t ? h - t : length - t + h;
      avail = length - used >= 8 ? length - used - 8 : 0;
    }
    const uint32_t bytes = align8(an + bn);
    if (bytes > avail) return Again;
    uint32_t pos = mem.r32(head);
    for (uint32_t i = 0; i < bytes; i += 4) {
      uint32_t w = 0;
      for (uint32_t j = 0; j < 4; j++) {
        const uint32_t k = i + j;
        const uint8_t v = k < an ? a[k] : (k - an < bn ? b[k - an] : 0);
        w |= static_cast<uint32_t>(v) << (8 * j);
      }
      mem.w32(fifo + pos, w);
      pos += 4;
      if (pos == length) pos = 0;
    }
    mem.fence();
    mem.w32(head, pos);
    mem.fence();
    return Ok;
  }
};

struct GlinkIntentSpec { uint32_t size, count; };
struct GlinkChannelSpec {
  const char *name;
  uint32_t kinds;
  GlinkIntentSpec intent[3];   // qcom,intents = <size count ...>
};

struct Glink {
  enum Command : uint16_t {
    Version = 0, VersionAck = 1, Open = 2, Close = 3, OpenAck = 4, Intent = 5, RxDone = 6, IntentReq = 7, IntentReqAck = 8,
    Data = 9, CloseAck = 11, DataCont = 12, ReadNotif = 13, RxDoneReuse = 14, Signals = 15
  };
  enum Trace : int {
    TrVersionTx, TrVersionRx, TrVersionAckTx, TrVersionAckRx, TrNegotiated, TrOpenTx, TrOpenRx, TrOpenAckTx, TrOpenAckRx,
    TrOpenIgnored, TrIntentTx, TrIntentRx, TrIntentReqTx, TrIntentReqRx, TrIntentReqAckTx, TrIntentReqAckRx, TrDataTx,
    TrDataRx, TrRxDoneTx, TrRxDoneRx, TrReadNotifTx, TrReadNotifRx, TrSignalsRx, TrCloseRx, TrCloseAckRx, TrReady,
    TrRxAttached, TrIgnored, TrCount
  };
  static const char *traceName(int t) {
    static const char *const names[TrCount] = {
      "version tx", "version rx", "version-ack tx", "version-ack rx", "negotiated", "open tx", "open rx", "open-ack tx",
      "open-ack rx", "open ignored (no local user)", "intent tx", "intent rx", "intent-req tx", "intent-req rx",
      "intent-req-ack tx", "intent-req-ack rx", "data tx", "data rx", "rx-done tx", "rx-done rx", "read-notif tx",
      "read-notif rx", "signals rx", "close rx", "close-ack rx", "channel ready", "rx fifo attached", "ignored"};
    return t >= 0 && t < TrCount ? names[t] : "?";
  }
  static constexpr uint32_t kMaxChannels = 4, kMaxLocal = 28, kMaxRemote = 32, kMaxPacket = 0x4400, kMaxRequest = 0x10000;
  static constexpr uint32_t kQueue = 64, kPairsPerMessage = 5, kFeatureIntentReuse = 1;
  struct LocalIntent { uint8_t *data; uint32_t id, size, used, remaining; bool active, reuse; };
  struct RemoteIntent { uint32_t id, size; bool used; };
  struct Channel {
    char name[32];
    const GlinkChannelSpec *spec;
    uint16_t local, remote;
    bool openSent, openAcked, remoteOpen, ready, requestPending;
    LocalIntent intent[kMaxLocal];
    RemoteIntent remoteIntent[kMaxRemote];
    uint8_t *pending;
    uint32_t pendingSize, pendingAt, pendingIntent, rxPackets, txPackets, requests;
  };
  struct Control { uint8_t bytes[48]; uint32_t size; };

  GlinkPipe tx = {}, rx = {};
  Allocator allocator = {};
  const GlinkChannelSpec *specs = nullptr;
  uint32_t specCount = 0;
  void *ctx = nullptr;
  void (*kick)(void *) = nullptr;
  void (*trace)(void *, int, const char *, uint32_t, uint32_t) = nullptr;
  void (*received)(void *, uint16_t, const uint8_t *, uint32_t) = nullptr;
  Channel channels[kMaxChannels] = {};
  Control control[kQueue] = {};
  uint32_t qRead = 0, qWrite = 0, qCount = 0, features = kFeatureIntentReuse, rxCommands = 0, ignoredOpens = 0;
  bool negotiated = false, notifySent = false, running = false;
  int error = 0;

  int start(const GlinkPipe &out, const GlinkPipe &in, const Allocator &a, const GlinkChannelSpec *table, uint32_t n,
            void *opaque, void (*notify)(void *), void (*tr)(void *, int, const char *, uint32_t, uint32_t),
            void (*data)(void *, uint16_t, const uint8_t *, uint32_t)) {
    if (running) return Busy;
    if (out.validate() || in.validate() || !out.tx || !out.attached() || in.tx || !a.alloc || !a.free || !notify || !data ||
        (n && !table))
      return Invalid;
    tx = out;
    rx = in;
    allocator = a;
    specs = table;
    specCount = n;
    ctx = opaque;
    kick = notify;
    trace = tr;
    received = data;
    memset(channels, 0, sizeof(channels));
    memset(control, 0, sizeof(control));
    qRead = qWrite = qCount = rxCommands = ignoredOpens = 0;
    features = kFeatureIntentReuse;
    error = 0;
    negotiated = notifySent = false;
    tx.mem.w32(tx.head, 0);   // qcom_glink_smem_register: *rx tail = *tx head = 0; the remote's words untouched
    rx.mem.w32(rx.tail, 0);
    tx.mem.fence();
    running = true;
    note(TrVersionTx, "", 1, features);
    return queue(Version, 1, features);
  }
  int attachRx(uint32_t fifo, uint32_t length) {
    if (rx.attached()) return Ok;
    GlinkPipe p = rx;
    p.fifo = fifo;
    p.length = length;
    if (p.validate()) return Invalid;
    rx = p;
    note(TrRxAttached, "", fifo, length);
    return Ok;
  }
  void stop(int reason) {
    if (!running) return;
    running = false;
    error = reason;
    for (auto &c : channels) release(c);
    qCount = qRead = qWrite = 0;
    negotiated = false;
  }
  // Local open of a channel of the table (qcom_glink_create_local); Ok with *id even before the remote answers
  int open(const char *name, uint16_t *id) {
    if (!running || !negotiated) return NotReady;
    const GlinkChannelSpec *s = specFor(name);
    if (!s) return Invalid;
    Channel *c = byName(name);
    if (!c) c = create(name, s);
    if (!c) return NoMemory;
    *id = c->local;
    if (c->openSent) return Ok;
    const int e = sendOpen(*c);
    return e;
  }
  bool ready(uint16_t id) {
    Channel *c = local(id);
    return running && c && c->ready;
  }
  Channel *channel(uint16_t id) { return local(id); }
  // Queues one message; Again while no remote intent fits (an RX_INTENT_REQ is then sent once) or one is in flight
  int send(uint16_t id, const uint8_t *data, uint32_t n) {
    Channel *c = local(id);
    if (!running || !c || !c->ready) return NotReady;
    if (!data || !n || n > kMaxPacket) return Range;
    if (c->pending) return Again;
    RemoteIntent *ri = nullptr;
    for (auto &r : c->remoteIntent)
      if (r.id && !r.used && r.size >= n && (!ri || r.size < ri->size)) ri = &r;
    if (!ri) {
      if (!c->requestPending) {
        const int e = queue(IntentReq, c->local, n);
        if (e) return e;
        note(TrIntentReqTx, c->name, n, 0);
        c->requestPending = true;
        c->requests++;
      }
      return Again;
    }
    uint8_t *p = static_cast<uint8_t *>(allocator.alloc(allocator.ctx, n));
    if (!p) return NoMemory;
    memcpy(p, data, n);
    c->pending = p;
    c->pendingSize = n;
    c->pendingAt = 0;
    c->pendingIntent = ri->id;
    ri->used = true;
    return Ok;
  }
  // One service pass: pending control messages, up to budget received commands, then pending data
  int step(uint32_t budget = 32) {
    if (!running) return error ? error : NotReady;
    int e = flush();
    if (e != Ok && e != Again) return fail(e);
    for (uint32_t k = 0; k < budget; k++) {
      if (qCount > kQueue - 12) break;   // room for the replies of the next command
      e = receiveOne();
      if (e == Again) break;
      if (e) return fail(e);
      e = flush();
      if (e != Ok && e != Again) return fail(e);
    }
    if (!qCount) {
      for (auto &c : channels) {
        if (!c.pending) continue;
        e = sendFragment(c);
        if (e == Again) break;
        if (e) return fail(e);
      }
    }
    return Ok;
  }

private:
  void note(int t, const char *name, uint32_t a, uint32_t b) {
    if (trace) trace(ctx, t, name, a, b);
  }
  int fail(int e) {
    stop(e);
    return e;
  }
  const GlinkChannelSpec *specFor(const char *name) const {
    for (uint32_t i = 0; i < specCount; i++)
      if (nameEquals(specs[i].name, name, 32)) return &specs[i];
    return nullptr;
  }
  Channel *local(uint16_t id) {
    for (auto &c : channels)
      if (id && c.local == id) return &c;
    return nullptr;
  }
  Channel *remote(uint16_t id) {
    for (auto &c : channels)
      if (id && c.remote == id) return &c;
    return nullptr;
  }
  Channel *byName(const char *name) {
    for (auto &c : channels)
      if (c.local && nameEquals(c.name, name, 32)) return &c;
    return nullptr;
  }
  Channel *create(const char *name, const GlinkChannelSpec *s) {
    uint32_t n = 0;
    while (n < 31 && name[n]) n++;
    if (!n || name[n]) return nullptr;
    for (uint32_t k = 0; k < kMaxChannels; k++) {
      if (channels[k].local) continue;
      Channel &c = channels[k];
      memset(&c, 0, sizeof(c));
      c.local = static_cast<uint16_t>(k + 1);
      c.spec = s;
      memcpy(c.name, name, n + 1);
      return &c;
    }
    return nullptr;
  }
  void release(Channel &c) {
    if (c.pending) allocator.free(allocator.ctx, c.pending, c.pendingSize);
    for (auto &i : c.intent)
      if (i.data) allocator.free(allocator.ctx, i.data, i.size);
    memset(&c, 0, sizeof(c));
  }
  int queue(uint16_t cmd, uint16_t p1, uint32_t p2, const uint8_t *extra = nullptr, uint32_t n = 0) {
    if (n > sizeof(control[0].bytes) - 8) return Range;
    if (qCount == kQueue) return NoMemory;
    Control &p = control[qWrite];
    memset(p.bytes, 0, sizeof(p.bytes));
    put16(p.bytes, cmd);
    put16(p.bytes + 2, p1);
    put32(p.bytes + 4, p2);
    if (n) memcpy(p.bytes + 8, extra, n);
    p.size = 8 + align8(n);
    qWrite = (qWrite + 1) % kQueue;
    qCount++;
    return Ok;
  }
  int sendOpen(Channel &c) {
    uint32_t n = 0;
    while (c.name[n]) n++;
    const int e = queue(Open, c.local, n + 1, reinterpret_cast<const uint8_t *>(c.name), n + 1);
    if (e) return e;
    c.openSent = true;
    note(TrOpenTx, c.name, c.local, n + 1);
    return Ok;
  }
  int requestRead() {
    if (notifySent) return Again;
    uint8_t hdr[8] = {};
    put16(hdr, ReadNotif);
    const int e = tx.push(hdr, 8, nullptr, 0, true);
    if (e) return e;
    notifySent = true;
    note(TrReadNotifTx, "", 0, 0);
    kick(ctx);
    return Again;
  }
  int flush() {
    bool sent = false;
    while (qCount) {
      const Control &p = control[qRead];
      const int e = tx.push(p.bytes, p.size);
      if (e == Again) {
        if (sent) kick(ctx);
        return requestRead();
      }
      if (e) return e;
      notifySent = false;
      sent = true;
      qRead = (qRead + 1) % kQueue;
      qCount--;
    }
    if (sent) kick(ctx);
    return Ok;
  }
  // Advertises intents [first, first + count) of c in messages of up to kPairsPerMessage pairs
  int advertise(Channel &c, uint32_t first, uint32_t count) {
    for (uint32_t i = 0; i < count;) {
      uint8_t pairs[8 * kPairsPerMessage] = {};
      uint32_t k = 0;
      for (; k < kPairsPerMessage && i < count; k++, i++) {
        const LocalIntent &li = c.intent[first + i];
        put32(pairs + 8 * k, li.size);
        put32(pairs + 8 * k + 4, li.id);
      }
      const int e = queue(Intent, c.local, k, pairs, 8 * k);
      if (e) return e;
      note(TrIntentTx, c.name, k, c.intent[first + i - 1].size);
    }
    return Ok;
  }
  LocalIntent *newIntent(Channel &c, uint32_t size, bool reuse) {
    for (uint32_t k = 0; k < kMaxLocal; k++) {
      LocalIntent &li = c.intent[k];
      if (li.data) continue;
      li.data = static_cast<uint8_t *>(allocator.alloc(allocator.ctx, size));
      if (!li.data) return nullptr;
      li.id = k + 1;
      li.size = size;
      li.used = li.remaining = 0;
      li.active = false;
      li.reuse = reuse;
      return &li;
    }
    return nullptr;
  }
  // both opens done: the channel's DT intents (qcom_glink_announce_create), then the channel is usable
  int maybeReady(Channel &c) {
    if (c.ready || !c.openAcked || !c.remoteOpen) return Ok;
    uint32_t made = 0;
    for (uint32_t k = 0; k < c.spec->kinds && k < 3; k++)
      for (uint32_t i = 0; i < c.spec->intent[k].count; i++) {
        if (!newIntent(c, c.spec->intent[k].size, true)) return NoMemory;
        made++;
      }
    const int e = advertise(c, 0, made);
    if (e) return e;
    c.ready = true;
    note(TrReady, c.name, c.local, c.remote);
    return Ok;
  }
  int sendFragment(Channel &c) {
    uint32_t space = 0;
    int e = tx.count(&space);
    if (e) return e;
    if (space <= 16) return requestRead();
    uint32_t n = c.pendingSize - c.pendingAt;
    if (n > 8192) n = 8192;
    const uint32_t capacity = (space - 16) & ~7u;
    if (n > capacity) n = capacity;
    if (!n) return requestRead();
    uint8_t hdr[16] = {};
    put16(hdr, c.pendingAt ? DataCont : Data);
    put16(hdr + 2, c.local);
    put32(hdr + 4, c.pendingIntent);
    put32(hdr + 8, n);
    put32(hdr + 12, c.pendingSize - c.pendingAt - n);
    e = tx.push(hdr, 16, c.pending + c.pendingAt, n);
    if (e) return e;
    notifySent = false;
    kick(ctx);
    note(TrDataTx, c.name, n, c.pendingIntent);
    c.pendingAt += n;
    if (c.pendingAt == c.pendingSize) {
      allocator.free(allocator.ctx, c.pending, c.pendingSize);
      c.pending = nullptr;
      c.pendingSize = c.pendingAt = c.pendingIntent = 0;
      c.txPackets++;
    }
    return Ok;
  }
  int intentRequest(Channel &c, uint32_t size) {
    note(TrIntentReqRx, c.name, size, 0);
    for (const auto &li : c.intent)
      if (li.data && li.reuse && li.size >= size) {   // as Linux: any reusable intent big enough
        note(TrIntentReqAckTx, c.name, 1, size);
        return queue(IntentReqAck, c.local, 1);
      }
    LocalIntent *li = size && size <= kMaxRequest ? newIntent(c, size, false) : nullptr;
    if (li) {
      const int e = advertise(c, li->id - 1, 1);
      if (e) return e;
    }
    note(TrIntentReqAckTx, c.name, li ? 1 : 0, size);
    return queue(IntentReqAck, c.local, li ? 1 : 0);
  }
  int dataIn(Channel &c, uint16_t cmd, uint32_t iid, const uint8_t *hdr) {
    LocalIntent *li = nullptr;
    for (auto &i : c.intent)
      if (i.data && i.id == iid) li = &i;
    const uint32_t chunk = le32(hdr + 8), left = le32(hdr + 12);
    if (!li || (cmd == Data && li->active) || (cmd == DataCont && !li->active)) return Protocol;
    if (chunk > li->size - li->used || left > li->size - li->used - chunk) return Range;
    if (li->active && li->remaining != chunk + left) return Protocol;
    rx.peek(16, li->data + li->used, chunk);
    li->used += chunk;
    li->remaining = left;
    li->active = true;
    if (left) return Ok;
    note(TrDataRx, c.name, li->used, iid);
    c.rxPackets++;
    received(ctx, c.local, li->data, li->used);
    if (!running) return error ? error : PeerReset;
    li->active = false;
    li->used = li->remaining = 0;
    int e;
    if (li->reuse && (features & kFeatureIntentReuse)) {
      e = queue(RxDoneReuse, c.local, iid);
      note(TrRxDoneTx, c.name, iid, 1);
    } else if (li->reuse) {   // no reuse feature: the intent is consumed; advertise it again
      e = queue(RxDone, c.local, iid);
      note(TrRxDoneTx, c.name, iid, 0);
      if (!e) e = advertise(c, li->id - 1, 1);
    } else {
      e = queue(RxDone, c.local, iid);
      note(TrRxDoneTx, c.name, iid, 0);
      allocator.free(allocator.ctx, li->data, li->size);
      memset(li, 0, sizeof(*li));
    }
    return e;
  }
  int receiveOne() {
    uint32_t available = 0;
    int e = rx.count(&available);
    if (e) return e;
    if (available < 8) return Again;
    uint8_t hdr[16] = {};
    rx.peek(0, hdr, 8);
    const uint16_t cmd = le16(hdr), p1 = le16(hdr + 2);
    const uint32_t p2 = le32(hdr + 4);
    uint32_t bytes = 8;
    if (cmd == Open) {
      const uint32_t n = p2 & 0xffffu;   // upper 16 bits: priority
      if (!n || n > 32) return Protocol;
      bytes += align8(n);
    } else if (cmd == Intent) {
      if (!p2 || p2 > kMaxRemote) return Protocol;
      bytes += 8 * p2;
    } else if (cmd == Data || cmd == DataCont) {
      if (available < 16) return Again;
      rx.peek(0, hdr, 16);
      const uint32_t chunk = le32(hdr + 8);
      if (!chunk || chunk > kMaxRequest || le32(hdr + 12) > kMaxRequest - chunk) return Protocol;
      bytes = 16 + align8(chunk);
    } else if (cmd > Signals || cmd == 10) {
      return Unsupported;
    }
    if (bytes > rx.length - 8) return Protocol;
    if (available < bytes) return Again;
    rxCommands++;
    Channel *c = remote(p1);
    e = Ok;
    switch (cmd) {
      case Version:
        note(TrVersionRx, "", p1, p2);
        if (!p1) break;                    // version 0: ignored
        if (p1 == 1) features &= p2;
        e = queue(VersionAck, 1, features);
        note(TrVersionAckTx, "", 1, features);
        if (!negotiated) note(TrNegotiated, "", 1, features);
        negotiated = true;
        break;
      case VersionAck:
        note(TrVersionAckRx, "", p1, p2);
        if (!p1) return Unsupported;       // negotiation failed on the remote side
        if (p1 == 1 && p2 == features) {
          if (!negotiated) note(TrNegotiated, "", 1, features);
          negotiated = true;
          break;
        }
        if (p1 == 1) features &= p2;
        e = queue(Version, 1, features);
        note(TrVersionTx, "", 1, features);
        break;
      case Open: {
        const uint32_t n = p2 & 0xffffu;
        char name[33] = {};
        rx.peek(8, reinterpret_cast<uint8_t *>(name), n);
        if (!p1 || name[n - 1] || !name[0] || c) return Protocol;
        note(TrOpenRx, name, p1, n);
        const GlinkChannelSpec *s = specFor(name);
        if (!s) {
          ignoredOpens++;
          note(TrOpenIgnored, name, p1, 0);
          break;
        }
        c = byName(name);
        if (!c) c = create(name, s);
        if (!c) return NoMemory;
        if (c->remote) return Protocol;
        c->remote = p1;
        c->remoteOpen = true;
        e = queue(OpenAck, p1, 0);         // qcom_glink_create_remote: open-ack, then our open
        note(TrOpenAckTx, c->name, p1, 0);
        if (!e && !c->openSent) e = sendOpen(*c);
        if (!e) e = maybeReady(*c);
        break;
      }
      case OpenAck:
        c = local(p1);
        if (!c || !c->openSent || c->openAcked) return Protocol;
        note(TrOpenAckRx, c->name, p1, p2);
        c->openAcked = true;
        e = maybeReady(*c);
        break;
      case Intent: {
        if (!c || !c->remoteOpen) {
          note(TrIgnored, "intent", p1, p2);
          break;
        }
        uint8_t pair[8];
        for (uint32_t i = 0; i < p2; i++) {
          rx.peek(8 + 8 * i, pair, 8);
          const uint32_t size = le32(pair), id = le32(pair + 4);
          RemoteIntent *slot = nullptr;
          for (auto &ri : c->remoteIntent)
            if (ri.id == id) slot = &ri;
          for (auto &ri : c->remoteIntent)
            if (!slot && !ri.id) slot = &ri;
          if (!slot || !id) continue;
          *slot = {id, size, false};
        }
        note(TrIntentRx, c->name, p2, le32(pair));
        c->requestPending = false;
        break;
      }
      case IntentReq:
        if (!c || !c->ready) {
          note(TrIgnored, "intent-req", p1, p2);
          break;
        }
        e = intentRequest(*c, p2);
        break;
      case IntentReqAck:
        note(TrIntentReqAckRx, c ? c->name : "", p1, p2);
        if (c && !p2) c->requestPending = false;
        break;
      case RxDone:
      case RxDoneReuse: {
        RemoteIntent *ri = nullptr;
        if (c)
          for (auto &r : c->remoteIntent)
            if (r.id && r.id == p2) ri = &r;
        if (!ri || !ri->used || (c->pending && c->pendingIntent == p2)) {
          note(TrIgnored, "rx-done", p1, p2);
          break;
        }
        note(TrRxDoneRx, c->name, p2, cmd == RxDoneReuse);
        if (cmd == RxDoneReuse) ri->used = false;
        else *ri = {};
        break;
      }
      case Data:
      case DataCont:
        if (!c || !c->ready) return Protocol;
        e = dataIn(*c, cmd, p2, hdr);
        break;
      case ReadNotif:
        note(TrReadNotifRx, "", 0, 0);
        kick(ctx);
        break;
      case Signals:
        note(TrSignalsRx, c ? c->name : "", p1, p2);
        break;
      case Close:
        note(TrCloseRx, c ? c->name : "", p1, 0);
        e = queue(CloseAck, p1, 0);
        if (c) release(*c);
        break;
      case CloseAck:
        c = local(p1);
        note(TrCloseAckRx, c ? c->name : "", p1, 0);
        if (c) release(*c);
        break;
      default:
        note(TrIgnored, "command", cmd, p2);   // RX_INTENT/other ids < 16 never sent on SMEM edges
        break;
    }
    if (e) return e;
    return rx.advance(bytes);
  }
};

}  // namespace qipc
#endif
