// SPDX-License-Identifier: BSD-2-Clause-Patent
// Apps side of one SMP2P edge (outbound item allocated and owned by apps, inbound item allocated by the remote), audio
// phase 2. Generalises the Wi-Fi line's modem-only Smp2pEdge (Codex GuampSmp2p.inc: 435/428, pid 1) to any remote:
// pids, items and the outbound entry list are parameters. Host-tested by tests/adsp_ipc_host.cpp.
// Protocol studied in the guamp kernel drivers/soc/qcom/smp2p.c and smp2p_sleepstate.c (no code copied):
//   item (340 bytes): magic "$SMP", version:8 + features:24, local_pid:16, remote_pid:16, total_entries:16 (16),
//   valid_entries:16, flags (bit 0 restart-done, bit 1 restart-ack), 16 x {name[16], value}.
//   qcom_smp2p_alloc_outbound_item: qcom_smem_alloc(remote, out item, 340), memset 0, magic/pids/total 16/features
//   SMP2P_FEATURE_SSR_ACK (1), wmb, version 1 (publishes the item), kick; then one entry per outbound DT child, in DT
//   order. Inbound (interrupt or here a poll): once the remote item exists and its version equals ours,
//   out.features = in.features & out.features (negotiation, no kick) and ssr-ack is enabled if bit 0 survives; while
//   open, in.flags bit 0 != the last ack -> flip out.flags bit 1 and kick (qcom_smp2p_do_ssr_ack).
//   update_bits(entry, mask, value): read-modify-write of the entry value, kick if changed.
// guamp ADSP edge (qcom,smp2p-adsp): smem <443 429>, local pid 0, remote pid 2, APCS IPC bit 10, GIC SPI 279 (polled
// here); outbound entries in DT order: "master-kernel" (bit 0 = force-stop), "rdbg", "sleepstate" (bit 12 = apps
// awake, set at smp2p-sleepstate probe and cleared only around system suspend).
#ifndef GUAMP_SMP2P_EDGE_INC
#define GUAMP_SMP2P_EDGE_INC
#include "GuampSmemWrite.inc"
namespace qipc {

struct Smp2pEdge {
  static constexpr uint32_t kBytes = 20 + 16 * 20, kMaxEntries = 16, kFeatureSsrAck = 1;
  SmemArena *arena = nullptr;
  void *kickCtx = nullptr;
  void (*kick)(void *) = nullptr;
  uint16_t local = 0, remote = 0;
  uint32_t outItem = 0, inItem = 0, outbound = 0, entries = 0;
  bool created = false, open = false, ssrEnabled = false, ssrAck = false;
  uint32_t inFeatures = 0, inFlags = 0, outFeatures = 0, acks = 0, kicks = 0;

  // Allocates (or takes the existing) outbound item and publishes it with the given entries. The caller must know
  // the remote is not running (Linux probes this before the remote boots): the item is rewritten from scratch.
  int start(SmemArena *a, uint16_t localPid, uint16_t remotePid, uint32_t out, uint32_t in, const char *const names[],
            const uint32_t values[], uint32_t n, void (*notify)(void *), void *ctx) {
    if (!a || !notify || n > kMaxEntries) return Invalid;
    for (uint32_t i = 0; i < n; i++) {
      uint32_t len = 0;
      while (names[i][len]) len++;
      if (!len || len > 15) return Invalid;
    }
    arena = a;
    kick = notify;
    kickCtx = ctx;
    local = localPid;
    remote = remotePid;
    outItem = out;
    inItem = in;
    open = ssrEnabled = ssrAck = false;
    inFeatures = inFlags = acks = 0;
    const int e = arena->allocate(remote, outItem, kBytes, &outbound, &created);
    if (e) return e;
    const Memory &m = arena->mem;
    m.w32(outbound + 4, 0);   // unpublished while it is rebuilt
    m.fence();
    m.zero(outbound + 8, kBytes - 8);
    m.w32(outbound, smem::kSmp2pMagic);
    m.w32(outbound + 8, local | (static_cast<uint32_t>(remote) << 16));
    for (uint32_t i = 0; i < n; i++) {
      uint8_t name[16] = {};
      for (uint32_t c = 0; c < 15 && names[i][c]; c++) name[c] = static_cast<uint8_t>(names[i][c]);
      m.copyIn(outbound + 20 + 20 * i, name, 16);
      m.w32(outbound + 36 + 20 * i, values[i]);
    }
    entries = n;
    m.w32(outbound + 12, kMaxEntries | (n << 16));
    outFeatures = kFeatureSsrAck;
    m.fence();
    m.w32(outbound + 4, 1u | (outFeatures << 8));   // version 1 last
    m.fence();
    notifyRemote();
    return Ok;
  }
  // Again: inbound item not there yet; Ok: *value = the named inbound entry (0 if absent); Protocol: bad item
  int poll(smem::Smp2p *scratch, const char *entry, uint32_t *value, bool *hasEntry) {
    *value = 0;
    *hasEntry = false;
    if (!arena || !outbound) return NotReady;
    uint32_t at = 0, bytes = 0;
    const int e = arena->find(remote, inItem, &at, &bytes);
    if (e) return e == NotFound ? Again : e;
    smem::readSmp2p(arena->view(), at, bytes, entry, scratch);
    if (!scratch->valid) return Again;   // header not published yet (version 0) or not complete
    if (scratch->localPid != remote || scratch->remotePid != local) return Protocol;
    inFeatures = scratch->features;
    inFlags = scratch->flags;
    const Memory &m = arena->mem;
    if (!open) {   // qcom_smp2p_negotiate: versions are both 1 once valid
      outFeatures = inFeatures & outFeatures;
      m.w32(outbound + 4, 1u | (outFeatures << 8));
      m.fence();
      ssrEnabled = (outFeatures & kFeatureSsrAck) != 0;
      open = true;
    }
    if (ssrEnabled && ((inFlags & 1) != 0) != ssrAck) {   // qcom_smp2p_check_ssr + do_ssr_ack
      ssrAck = !ssrAck;
      const uint32_t flags = m.r32(outbound + 16);
      m.w32(outbound + 16, (flags & ~2u) | (ssrAck ? 2u : 0u));
      m.fence();
      acks++;
      notifyRemote();
    }
    *hasEntry = scratch->hasEntry;
    *value = scratch->hasEntry ? scratch->value : 0;
    return Ok;
  }
  int update(uint32_t index, uint32_t mask, uint32_t value) {
    if (!arena || !outbound || index >= entries) return Invalid;
    const Memory &m = arena->mem;
    const uint32_t at = outbound + 36 + 20 * index, old = m.r32(at), now = (old & ~mask) | (value & mask);
    m.w32(at, now);
    m.fence();
    if (now != old) notifyRemote();
    return Ok;
  }
  uint32_t value(uint32_t index) const { return arena && outbound && index < entries ? arena->mem.r32(outbound + 36 + 20 * index) : 0; }

private:
  void notifyRemote() {
    kicks++;
    kick(kickCtx);
  }
};

}  // namespace qipc
#endif
