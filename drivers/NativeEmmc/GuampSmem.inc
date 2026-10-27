// SPDX-License-Identifier: BSD-2-Clause-Patent
// Read-only Qualcomm SMEM parser for the guamp Wi-Fi preparation (GuampWifiPrep.inc), host-tested by
// tests/wifi_host.cpp on synthetic images. Layout from the study of Linux drivers/soc/qcom/smem.c and smp2p.c (no code
// copied). The owner supplies an aligned 32-bit reader over the region; nothing is ever written and no hardware
// spinlock is taken (a reader may see an item half allocated; every walk is bounds-checked and stops on a bad canary).
//   header     region start: proc_comm[4] (64 bytes), version[32] (version[7] >> 16: 11 global heap, 12 partitions),
//              initialized, free_offset, available, reserved, toc[512] of {allocated, offset, size, aux_base} (v11)
//   ptable     last 4 KiB: "$TOC", version 1, num_entries, 5 reserved, entries of 48 bytes {offset, size, flags,
//              host0:16 host1:16, cacheline, 7 reserved}, then "SIII" info with num_items
//   partition  "$PRT", host0, host1, size, offset_free_uncached, offset_free_cached, 3 reserved (32 bytes);
//              uncached entries grow up from +32 {canary 0xa5a5, item, size, padding_data, padding_hdr, reserved}
//              (data after the header + padding_hdr), cached entries grow down from the end (data before the header)
// Only partitions that include the application processor (host 0) or the global one (0xfffe/0xfffe) are read:
// the others are firewalled for the apps CPU and a read could fault.
// SMP2P item (smp2p.c): magic "$SMP", version 1, features:24, local_pid, remote_pid, total_entries, valid_entries,
// flags, then 16 entries of {name[16], value}. guamp modem pair (bengal.dtsi qcom,smp2p-modem): 435 apps -> modem
// (outbound, allocated by the apps side), 428 modem -> apps (inbound "slave-kernel": bit 0 err-fatal, 1 err-ready,
// 2 proxy-unvote/handover, 3 stop-ack, 7 shutdown-ack); crash reason item 421 (qcom,smem-id of the modem PIL);
// GLINK items 478 (descriptor), 479 and 480 (FIFOs) in the apps-modem partition.

namespace smem {

typedef uint32_t (*Read32)(void *ctx, uint32_t offset);   // offset 4-byte aligned and below size

constexpr uint16_t kHostApps = 0, kHostGlobal = 0xfffe, kHostModem = 1;
constexpr uint32_t kMaxPartitions = 16, kMaxIds = 40, kItemCount = 512;
constexpr uint32_t kSmp2pMagic = 0x504d5324;              // "$SMP"

enum : int { kOk = 0, kErrFormat = -30, kErrVersion = -31, kErrNoTable = -32, kErrRange = -33 };

struct Reader {
  Read32 rd;
  void *ctx;
  uint32_t size;
  bool ok(uint32_t off, uint32_t bytes) const { return off <= size && bytes <= size - off; }
  uint32_t r32(uint32_t off) const { return (off & 3) || !ok(off, 4) ? 0 : rd(ctx, off); }
  uint16_t r16(uint32_t off) const { return static_cast<uint16_t>(r32(off & ~3u) >> (8 * (off & 2))); }
  uint8_t r8(uint32_t off) const { return static_cast<uint8_t>(r32(off & ~3u) >> (8 * (off & 3))); }
};

struct Partition {
  uint32_t offset, size, cacheline;
  uint16_t host0, host1;
  bool read;          // includes apps or global: walked
  int status;         // walk result
  uint32_t items;     // items seen
  uint16_t ids[kMaxIds];
};

struct Info {
  int error;
  uint32_t version, initialized, ptableVersion, entries, numItems;
  uint32_t partitions;
  Partition part[kMaxPartitions];
};

// Visits each item of partition p: visit(item, dataOffset, dataSize) returns true to stop
template <typename Visit> int walk(const Reader &r, const Partition &p, Visit visit) {
  uint32_t base = p.offset;
  if (!r.ok(base, p.size) || p.size < 32 || (base & 7)) return kErrRange;
  if (r.r32(base) != 0x54525024 || r.r16(base + 4) != p.host0 || r.r16(base + 6) != p.host1 || r.r32(base + 8) != p.size)
    return kErrFormat;   // "$PRT"
  uint32_t freeUncached = r.r32(base + 12), freeCached = r.r32(base + 16);
  if (freeUncached > freeCached || freeCached > p.size || freeUncached < 32) return kErrFormat;
  uint32_t guard = 0;
  for (uint32_t e = 32; e + 16 <= freeUncached; guard++) {
    if ((e & 3) || guard > 4096) return kErrFormat;
    uint32_t w0 = r.r32(base + e), size = r.r32(base + e + 4), w2 = r.r32(base + e + 8);
    if ((w0 & 0xffff) != 0xa5a5) return kErrFormat;
    uint32_t paddingData = w2 & 0xffff, paddingHdr = w2 >> 16;
    uint64_t data = static_cast<uint64_t>(e) + 16 + paddingHdr, next = data + size;
    if (next > freeUncached || next <= e || paddingData > size) return kErrFormat;
    if (visit(static_cast<uint16_t>(w0 >> 16), base + static_cast<uint32_t>(data), size - paddingData)) return kOk;
    e = static_cast<uint32_t>(next);
  }
  if (freeCached == p.size) return kOk;
  uint32_t line = p.cacheline ? p.cacheline : 16;
  if (line & (line - 1) || line > 4096) return kErrFormat;
  uint32_t hdr = (16 + line - 1) & ~(line - 1);
  if (p.size < hdr) return kErrFormat;
  guard = 0;
  for (uint32_t e = p.size - hdr; e > freeCached && e >= freeUncached; guard++) {
    if ((e & 3) || guard > 4096) return kErrFormat;
    uint32_t w0 = r.r32(base + e), size = r.r32(base + e + 4), w2 = r.r32(base + e + 8);
    if ((w0 & 0xffff) != 0xa5a5) return kErrFormat;
    uint32_t paddingData = w2 & 0xffff;
    if (size > e || paddingData > size || e - size < freeCached) return kErrFormat;
    if (visit(static_cast<uint16_t>(w0 >> 16), base + e - size, size - paddingData)) return kOk;
    if (e - size < hdr) break;
    e = e - size - hdr;
  }
  return kOk;
}

inline void parse(const Reader &r, Info *info) {
  memset(info, 0, sizeof(*info));
  if (r.size < 8192) { info->error = kErrRange; return; }
  info->version = r.r32(64 + 4 * 7) >> 16;
  info->initialized = r.r32(192);
  info->numItems = kItemCount;
  if (info->version != 11 && info->version != 12) { info->error = kErrVersion; return; }
  uint32_t table = r.size - 4096;
  if (r.r32(table) != 0x434f5424) {               // "$TOC"
    info->error = info->version == 12 ? kErrNoTable : kOk;
    return;
  }
  info->ptableVersion = r.r32(table + 4);
  info->entries = r.r32(table + 8);
  if (info->ptableVersion != 1 || info->entries > (4096 - 32) / 48) { info->error = kErrFormat; return; }
  uint32_t infoAt = table + 32 + 48 * info->entries;
  info->numItems = r.ok(infoAt, 16) && r.r32(infoAt) == 0x49494953 ? r.r16(infoAt + 12) : kItemCount;   // "SIII"
  for (uint32_t i = 0; i < info->entries && info->partitions < kMaxPartitions; i++) {
    uint32_t e = table + 32 + 48 * i;
    Partition &p = info->part[info->partitions];
    p.offset = r.r32(e);
    p.size = r.r32(e + 4);
    p.host0 = r.r16(e + 12);
    p.host1 = r.r16(e + 14);
    p.cacheline = r.r32(e + 16);
    if (!p.offset || !p.size) continue;
    info->partitions++;
    p.read = p.host0 == kHostApps || p.host1 == kHostApps || (p.host0 == kHostGlobal && p.host1 == kHostGlobal);
    if (!p.read) continue;
    p.status = walk(r, p, [&](uint16_t item, uint32_t, uint32_t) {
      if (p.items < kMaxIds) p.ids[p.items] = item;
      p.items++;
      return false;
    });
  }
}

inline const Partition *partitionFor(const Info &info, uint16_t remote) {
  for (uint32_t i = 0; i < info.partitions; i++) {
    const Partition &p = info.part[i];
    if (!p.read) continue;
    if (remote == kHostGlobal ? p.host0 == kHostGlobal && p.host1 == kHostGlobal
                              : (p.host0 == kHostApps && p.host1 == remote) || (p.host1 == kHostApps && p.host0 == remote))
      return &p;
  }
  return nullptr;
}

// Item lookup as qcom_smem_get: the partition shared with remote first, then the global partition (or the v11 heap)
inline bool find(const Reader &r, const Info &info, uint16_t remote, uint32_t item, uint32_t *offset, uint32_t *size) {
  if (item >= kItemCount || item >= info.numItems) return false;
  const Partition *list[2] = {partitionFor(info, remote), partitionFor(info, kHostGlobal)};
  for (const Partition *p : list) {
    if (!p || p->status) continue;
    bool found = false;
    walk(r, *p, [&](uint16_t id, uint32_t at, uint32_t bytes) {
      if (id != item) return false;
      *offset = at; *size = bytes; found = true;
      return true;
    });
    if (found) return true;
  }
  if (info.version == 11) {                        // global heap table of contents
    uint32_t e = 208 + 16 * item;
    if (r.r32(e) != 1 || r.r32(e + 12) & ~3u) return false;
    uint32_t at = r.r32(e + 4), bytes = r.r32(e + 8);
    if (!r.ok(at, bytes)) return false;
    *offset = at; *size = bytes;
    return true;
  }
  return false;
}

struct Smp2p {
  bool present, valid;
  uint32_t size, version, features, flags;
  uint16_t localPid, remotePid, total, validEntries;
  bool hasEntry;          // the named entry exists
  uint32_t value;         // its value
  char names[16][17];
  uint32_t values[16];
};

inline void readSmp2p(const Reader &r, uint32_t at, uint32_t size, const char *entry, Smp2p *s) {
  memset(s, 0, sizeof(*s));
  s->present = true;
  s->size = size;
  if (size < 20 || (at & 3) || !r.ok(at, size) || r.r32(at) != kSmp2pMagic) return;
  uint32_t w1 = r.r32(at + 4);
  s->version = w1 & 0xff;
  s->features = w1 >> 8;
  s->localPid = r.r16(at + 8);
  s->remotePid = r.r16(at + 10);
  s->total = r.r16(at + 12);
  s->validEntries = r.r16(at + 14);
  s->flags = r.r32(at + 16);
  if (s->version != 1 || s->total > 16 || s->validEntries > s->total || size < 20 + 20u * s->total) return;
  s->valid = true;
  for (uint32_t i = 0; i < s->validEntries; i++) {
    uint32_t e = at + 20 + 20 * i;
    for (uint32_t c = 0; c < 16; c++) s->names[i][c] = static_cast<char>(r.r8(e + c));
    s->names[i][16] = 0;
    s->values[i] = r.r32(e + 16);
    uint32_t c = 0;
    while (entry[c] && entry[c] == s->names[i][c]) c++;
    if (!entry[c] && (c == 16 || !s->names[i][c])) { s->hasEntry = true; s->value = s->values[i]; }
  }
}

}  // namespace smem
