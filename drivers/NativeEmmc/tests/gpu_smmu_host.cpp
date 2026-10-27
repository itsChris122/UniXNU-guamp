// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of GuampGpuSmmu.h with the real ArmSmmu4K kit sources: an SMMUv2 register model with the IDs the phone
// reported (idr0 4c017e04 idr1 20000005 idr2 00005511, sCR0 00200406) and a stage-1 page walker that translates GPU
// addresses through the tables the kit built. No hardware is accessed.
#include "GuampGpuSmmu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned g_checks, g_fail;
#define CHECK(x)                                                                 \
  do {                                                                           \
    g_checks = g_checks + 1;                                                     \
    if (!(x)) {                                                                  \
      g_fail = g_fail + 1;                                                       \
      fprintf(stderr, "gpu_smmu_host.cpp:%d: CHECK(%s) failed\n", __LINE__, #x); \
    }                                                                            \
  } while (0)

// ---- SMMU register model ----
struct Model {
  uint8_t regs[GGS_SIZE];
  unsigned tlbiAsid, tlbSync, writes;
  uint32_t lastTlbiAsid;   // ASID of the last TLBIASID or per-page TLBIVA(L)
};
static Model M;
static uint32_t r32(size_t at) { uint32_t v; memcpy(&v, M.regs + at, 4); return v; }
static uint64_t r64(size_t at) { uint64_t v; memcpy(&v, M.regs + at, 8); return v; }
static void w32(size_t at, uint32_t v) { memcpy(M.regs + at, &v, 4); }
static void w64(size_t at, uint64_t v) { memcpy(M.regs + at, &v, 8); }
static const size_t kCb0 = 0x8000;
static size_t cbOff(unsigned bank) { return kCb0 + (size_t)bank * 0x1000; }

static int io_read32(void *, size_t at, uint32_t *v) {
  if (at + 4 > GGS_SIZE || (at & 3)) return 1;
  *v = r32(at);
  if (at >= kCb0 && (at & 0xfff) == 0x7f4) *v = 0;   // TLBSTATUS: sync always complete
  return 0;
}
static int io_read64(void *, size_t at, uint64_t *v) {
  if (at + 8 > GGS_SIZE || (at & 7)) return 1;
  *v = r64(at);
  return 0;
}
static int io_write32(void *, size_t at, uint32_t v) {
  if (at + 4 > GGS_SIZE || (at & 3)) return 1;
  M.writes = M.writes + 1;
  if (at >= kCb0) {
    const size_t off = at & 0xfff;
    if (off == 0x58) { w32(at, r32(at) & ~v); return 0; }                     // FSR, write 1 to clear
    if (off == 0x610) { M.tlbiAsid = M.tlbiAsid + 1; M.lastTlbiAsid = v; return 0; }   // TLBIASID
    if (off == 0x7f0) { M.tlbSync = M.tlbSync + 1; return 0; }                // TLBSYNC
  }
  w32(at, v);
  return 0;
}
static int io_write64(void *, size_t at, uint64_t v) {
  if (at + 8 > GGS_SIZE || (at & 7)) return 1;
  M.writes = M.writes + 1;
  if (at >= kCb0 && ((at & 0xfff) == 0x600 || (at & 0xfff) == 0x620 || (at & 0xfff) == 0x628)) {
    // TLBIVA / TLBIVAL (AArch64: ASID in [63:48]): a per-page invalidation (the tb128fu kit's unmap)
    M.tlbiAsid = M.tlbiAsid + 1;
    M.lastTlbiAsid = (uint32_t)(v >> 48);
    return 0;
  }
  w64(at, v);
  return 0;
}
static int io_barrier(void *) { return 0; }
static ArmSmmu4KIo g_io = {&M, GGS_SIZE, io_read32, io_read64, io_write32, io_write64, io_barrier};

static void model_reset() {
  memset(&M, 0, sizeof(M));
  w32(0x00, 0x00200406u);   // sCR0: GFRE, GFIE, USFCFG (fault unmatched)
  w32(0x20, 0x4c017e04u);
  w32(0x24, 0x20000005u);
  w32(0x28, 0x00005511u);
}

// ---- table pool in "physical" memory ----
static const uint64_t kPoolPhys = 0x80000000ULL;
static const size_t kPoolPages = 900;
alignas(4096) static uint64_t g_pool[kPoolPages * 512];
static unsigned g_published;
static int publish(void *, volatile uint64_t *page, size_t bytes) {
  const uintptr_t at = (uintptr_t)page, base = (uintptr_t)g_pool;
  if (bytes != 4096 || at < base || at - base >= sizeof(g_pool) || ((at - base) & 4095)) return 1;
  g_published = g_published + 1;
  return 0;
}
static const uint64_t *phys_table(uint64_t pa) {
  if (pa < kPoolPhys || pa - kPoolPhys >= sizeof(g_pool) || (pa & 4095)) return nullptr;
  return g_pool + (pa - kPoolPhys) / 8;
}

// Stage-1 walk of `bank` as the hardware would: SCTLR.M, TTBR0 root, T0SZ, 4K granule, 9 bits per level.
enum WalkResult { W_OK, W_DISABLED, W_TRANSLATION, W_ACCESS, W_PERMISSION, W_BAD_TABLE };
static WalkResult walk(unsigned bank, uint64_t va, bool write, uint64_t *pa) {
  const size_t cb = cbOff(bank);
  if (!(r32(cb + 0x0) & 1)) return W_DISABLED;
  const uint32_t tcr = r32(cb + 0x30);
  const unsigned inBits = 64 - (tcr & 0x3f);
  if (inBits < 25 || inBits > 48 || (va >> inBits)) return W_TRANSLATION;
  const unsigned start = 4 - (inBits - 12 + 8) / 9;
  uint64_t table = r64(cb + 0x20) & 0x0000fffffffff000ULL;
  for (unsigned level = start; level <= 3; ++level) {
    const unsigned shift = 12 + 9 * (3 - level);
    const unsigned width = level == start ? inBits - shift : 9;
    const uint64_t index = (va >> shift) & ((1ULL << width) - 1);
    const uint64_t *t = phys_table(table);
    if (!t) return W_BAD_TABLE;
    const uint64_t d = t[index];
    if ((d & 3) != 3) return W_TRANSLATION;   // invalid (no blocks are used by the kit)
    if (level < 3) {
      table = d & 0x0000fffffffff000ULL;
      continue;
    }
    if (!((d >> 10) & 1)) return W_ACCESS;
    if (write && ((d >> 7) & 1)) return W_PERMISSION;
    if (!((d >> 11) & 1)) return W_BAD_TABLE;   // the kit marks leaves non-global (ASID tagged)
    *pa = (d & 0x0000fffffffff000ULL) | (va & 0xfff);
    return W_OK;
  }
  return W_TRANSLATION;
}

static bool walks_to(unsigned bank, uint64_t va, uint64_t pa, bool write) {
  uint64_t got = 0;
  return walk(bank, va, write, &got) == W_OK && got == pa;
}

static ArmSmmu4KRange g_ranges[2048], g_scratch[2048];
static ggs_entry g_entries[2048];

static void test_geometry() {
  ggs_geometry g;
  CHECK(ggs_decode(0x4c017e04u, 0x20000005u, 0x00005511u, &g) == nullptr);
  CHECK(g.smrs == 4 && g.sidBits == 15 && g.banks == 5 && g.s2Banks == 0);
  CHECK(g.pageShift == 12 && g.globalPages == 8 && g.cbBase == 0x8000 && g.gr1 == 0x1000);
  CHECK(g.oas == 36 && g.ias == 36 && g.ubs == 48 && g.ptfs4k && g.s1 && g.sms && !g.exids);
  ggs_geometry x;
  CHECK(ggs_decode(0x4c017e04u, 0x20000005u, 0x00004511u, &x) != nullptr);   // no 4K tables
  CHECK(ggs_decode(0x0c017e04u, 0x20000005u, 0x00005511u, &x) != nullptr);   // no stage 1
  CHECK(ggs_decode(0x4c017e04u, 0xa0000005u, 0x00005511u, &x) != nullptr);   // 64K register pages
  CHECK(ggs_decode(0x4c017e01u, 0x20000005u, 0x00005511u, &x) != nullptr);   // 1 SMR
  ggs_profile p;
  CHECK(ggs_default_profile(g, &p));
  CHECK(p.bank == 4 && p.asid == 5 && p.slots[0] == 2 && p.slots[1] == 3 && p.sids[0] == 0 && p.sids[1] == 1);
  CHECK(p.outputBits == 36 && p.inputBits == 32 && p.coherentWalk == 0);
  CHECK(p.irqContext == 32 + 171 && p.irqGlobal == 32 + 163);
  // Windows: disjoint, in order, inside the aperture.
  for (unsigned w = 0; w < GGS_WINDOWS; ++w) {
    CHECK(kGgsWindowBase[w] >= GGS_APERTURE_IOVA);
    CHECK(kGgsWindowBase[w] + kGgsWindowBytes[w] <= GGS_APERTURE_IOVA + GGS_APERTURE_BYTES);
    if (w) CHECK(kGgsWindowBase[w] >= kGgsWindowBase[w - 1] + kGgsWindowBytes[w - 1]);
    CHECK(ggs_window_of(kGgsWindowBase[w]) == (int)w && ggs_window_of(kGgsWindowBase[w] + kGgsWindowBytes[w] - 1) == (int)w);
  }
  CHECK(ggs_window_of(0x1effffffULL) == -1 && ggs_window_of(0x7fffb000ULL) == -1);
  CHECK(ggs_align_for(4096) == 4096 && ggs_align_for(0x10000) == 0x10000 && ggs_align_for(0x200000) == 0x200000);
}

static bool setup(ggs_space *s, ggs_profile *p, ggs_geometry *g, bool actlr) {
  model_reset();
  memset(g_pool, 0, sizeof(g_pool));
  g_published = 0;
  if (ggs_decode(r32(0x20), r32(0x24), r32(0x28), g)) return false;
  if (!ggs_default_profile(*g, p)) return false;
  const ArmSmmu4KLiveVisibility vis = {nullptr, publish};
  int e = ggs_prepare(s, *p, g_pool, kPoolPhys, kPoolPages, g_ranges, g_entries, g_scratch, 2048, vis);
  if (e) {
    fprintf(stderr, "prepare: %s (kit %d)\n", ggs_err(e), s->last);
    return false;
  }
  static ArmSmmu4KStreamState streams;
  static ArmSmmu4KStreamResult routes;
  e = ggs_activate(s, &g_io, *p, actlr, &streams, &routes);
  if (e) fprintf(stderr, "activate: %s (kit %d)\n", ggs_err(e), s->last);
  return e == GGS_OK;
}

static void test_activate() {
  static ggs_space s;
  ggs_profile p;
  ggs_geometry g;
  CHECK(setup(&s, &p, &g, true));
  printf("tables: %zu pages (%zu KiB) for the 1552 MiB aperture, start level %u, %u pages published\n",
         s.state.tables.table_pages, s.state.tables.table_pages * 4, s.state.tables.start_level, g_published);
  CHECK(s.state.tables.table_pages <= GGS_TABLE_PAGES && s.state.tables.start_level == 1);
  CHECK(g_published == s.state.tables.table_pages);
  const size_t cb = cbOff(4);
  CHECK((r32(cb) & 1) == 1);                                           // SCTLR.M
  CHECK(r64(cb + 0x20) == ggs_ttbr0(kPoolPhys, 5));                    // TTBR0 = root | ASID 5 << 48
  CHECK(r32(cb + 0x30) == ((2u << 30) | 0x2000u | 32u | (1u << 23)));  // TG0/TG1 4K, outer NC walk, T0SZ 32, EPD1
  CHECK((r32(cb + 0x10) & 0xfu) == 1u);                                 // TCR2.PASize 36 bit
  CHECK(r32(cb + GGS_CB_ACTLR) == GGS_ACTLR);
  CHECK(r32(0x1000 + 4 * 4) == 0x1f300u);                              // CBAR: S1 translate, S2 bypass
  CHECK((r32(GGS_S2CR(2)) & 0x300ffu) == 4u && (r32(GGS_S2CR(3)) & 0x300ffu) == 4u);   // translate to bank 4
  CHECK(r32(GGS_SMR(2)) == 0x80000000u && r32(GGS_SMR(3)) == 0x80000001u);             // exact SID 0, SID 1
  CHECK(r32(GGS_S2CR(0)) == 0 && r32(GGS_SMR(0)) == 0);                // slot 0 (M3's) untouched
  uint64_t pa;
  CHECK(walk(4, 0x20000000ULL, false, &pa) == W_TRANSLATION);          // nothing mapped yet
}

static void test_map_unmap() {
  static ggs_space s;
  ggs_profile p;
  ggs_geometry g;
  CHECK(setup(&s, &p, &g, false));
  const unsigned rw = ArmSmmu4KRead | ArmSmmu4KWrite;
  // BO 1: three scattered chunks (4K + 64K + 8K = 76K -> 64K aligned) in the render server window.
  const ggs_chunk a[3] = {{0x90000000ULL, 0x1000}, {0x91230000ULL, 0x10000}, {0x95000000ULL, 0x2000}};
  uint64_t ia = 0;
  CHECK(ggs_map(&s, GGS_RENDER_WINDOW, 1, a, 3, rw, &ia) == GGS_OK);
  CHECK(ia == 0x20000000ULL);
  CHECK(walks_to(4, ia + 0x0, 0x90000000ULL, true));
  CHECK(walks_to(4, ia + 0x1abc, 0x91230abcULL, true));
  CHECK(walks_to(4, ia + 0x10fff, 0x9123ffffULL, true));
  CHECK(walks_to(4, ia + 0x11000, 0x95000000ULL, true));
  CHECK(walks_to(4, ia + 0x12ff8, 0x95001ff8ULL, true));
  uint64_t pa;
  CHECK(walk(4, ia + 0x13000, false, &pa) == W_TRANSLATION);
  CHECK(s.count == 3);
  // BO 2: 2 MiB + 4K, 2 MiB aligned after BO 1.
  const ggs_chunk b[1] = {{0xa0000000ULL, 0x201000}};
  uint64_t ib = 0;
  CHECK(ggs_map(&s, GGS_RENDER_WINDOW, 2, b, 1, rw, &ib) == GGS_OK);
  CHECK(ib == 0x20200000ULL);
  CHECK(walks_to(4, ib + 0x200123, 0xa0200123ULL, false));
  // BO 3: read-only in client window 2.
  const ggs_chunk c[1] = {{0xb0000000ULL, 0x3000}};
  uint64_t ic = 0;
  CHECK(ggs_map(&s, 2, 3, c, 1, ArmSmmu4KRead, &ic) == GGS_OK);
  CHECK(ic == kGgsWindowBase[2]);
  CHECK(walks_to(4, ic + 0x2004, 0xb0002004ULL, false));
  CHECK(walk(4, ic + 0x2004, true, &pa) == W_PERMISSION);
  // Same handle twice in a window, bad arguments.
  CHECK(ggs_map(&s, GGS_RENDER_WINDOW, 1, a, 1, rw, &ia) == GGS_ERR_ARG);
  const ggs_chunk bad[1] = {{0x90000800ULL, 0x1000}};
  uint64_t ix;
  CHECK(ggs_map(&s, GGS_RENDER_WINDOW, 9, bad, 1, rw, &ix) == GGS_ERR_ARG);
  CHECK(ggs_map(&s, GGS_RENDER_WINDOW, 9, b, 1, ArmSmmu4KWrite, &ix) == GGS_ERR_ARG);   // read required
  CHECK(ggs_map(&s, GGS_WINDOWS, 9, b, 1, rw, &ix) == GGS_ERR_ARG);
  // A table-pool page as a buffer: refused by the kit before any store, the space stays usable.
  const ggs_chunk pool[1] = {{kPoolPhys, 0x1000}};
  CHECK(ggs_map(&s, GGS_RENDER_WINDOW, 9, pool, 1, rw, &ix) == GGS_ERR_KIT && !s.poisoned);
  // Unmap BO 1: translation gone, ASID invalidated, BO 2 still mapped, registry compacted.
  const unsigned tlbi = M.tlbiAsid;
  CHECK(ggs_unmap(&s, GGS_RENDER_WINDOW, 1) == GGS_OK);
  CHECK(M.tlbiAsid > tlbi && M.lastTlbiAsid == 5);
  CHECK(walk(4, ia, false, &pa) == W_TRANSLATION && walk(4, ia + 0x11000, false, &pa) == W_TRANSLATION);
  CHECK(walks_to(4, ib, 0xa0000000ULL, true));
  CHECK(s.count == 2 && s.ranges[0].iova == ib && s.ranges[1].iova == ic);
  CHECK(ggs_unmap(&s, GGS_RENDER_WINDOW, 1) == GGS_ERR_NOTFOUND);
  // The gap is reused (first fit).
  const ggs_chunk d[1] = {{0x96000000ULL, 0x1000}};
  uint64_t id = 0;
  CHECK(ggs_map(&s, GGS_RENDER_WINDOW, 4, d, 1, rw, &id) == GGS_OK && id == 0x20000000ULL);
  CHECK(s.count == 3 && s.ranges[0].iova == id && s.ranges[1].iova == ib);
  // Fill client window 3 (0x0AAA9000, not 2 MiB aligned): two 80 MiB buffers at 2 MiB aligned IOVAs, then 16 MiB no
  // longer fits while 4K still does.
  const ggs_chunk big1[1] = {{0x100000000ULL, 0x5000000ULL}}, big2[1] = {{0x110000000ULL, 0x5000000ULL}};
  const ggs_chunk mid[1] = {{0x118000000ULL, 0x1000000ULL}};
  uint64_t i1 = 0, i2 = 0;
  CHECK(ggs_map(&s, 3, 10, big1, 1, rw, &i1) == GGS_OK && i1 == 0x6ac00000ULL);
  CHECK(ggs_map(&s, 3, 11, big2, 1, rw, &i2) == GGS_OK && i2 == 0x6fc00000ULL);
  CHECK(ggs_map(&s, 3, 12, mid, 1, rw, &ix) == GGS_ERR_NOSPACE);
  CHECK(walks_to(4, i2 + 0x4ffffff, 0x114ffffffULL, true));
  // A buffer larger than its window.
  const ggs_chunk huge[1] = {{0x120000000ULL, 0x0b000000ULL}};
  CHECK(ggs_map(&s, 2, 13, huge, 1, rw, &ix) == GGS_ERR_ARG);
  // Terminate client 3: both buffers go in one transaction, other windows untouched.
  size_t removed = 0;
  CHECK(ggs_unmap_window(&s, 3, &removed) == GGS_OK && removed == 2);
  CHECK(walk(4, i1, false, &pa) == W_TRANSLATION && walk(4, i2, false, &pa) == W_TRANSLATION);
  CHECK(walks_to(4, ic, 0xb0000000ULL, false) && walks_to(4, id, 0x96000000ULL, true));
  CHECK(s.count == 3);
  // Many small buffers: more than 64 updates, so the kit's whole-region re-verification runs again on the way.
  unsigned ok = 0;
  for (uint32_t h = 100; h < 180; ++h) {
    const ggs_chunk one[1] = {{0xc0000000ULL + (uint64_t)h * 0x1000, 0x1000}};
    uint64_t iv = 0;
    if (ggs_map(&s, 4, h, one, 1, rw, &iv) == GGS_OK && walks_to(4, iv, one[0].physical, true)) ok = ok + 1;
  }
  CHECK(ok == 80);
  for (uint32_t h = 100; h < 180; h += 2) CHECK(ggs_unmap(&s, 4, h) == GGS_OK);
  CHECK(s.count == 3 + 40 && !s.poisoned);
  for (size_t i = 1; i < s.count; ++i) CHECK(s.ranges[i - 1].iova + s.ranges[i - 1].bytes <= s.ranges[i].iova);
}

static void test_faults() {
  static ggs_space s;
  ggs_profile p;
  ggs_geometry g;
  CHECK(setup(&s, &p, &g, false));
  const ggs_chunk c[1] = {{0xb0000000ULL, 0x3000}};
  uint64_t ic = 0;
  CHECK(ggs_map(&s, 2, 3, c, 1, ArmSmmu4KRead, &ic) == GGS_OK);
  // Permission fault (write to the read-only BO) by SID 1.
  const size_t cb = cbOff(4);
  w32(cb + GGS_CB_FSR, 0x8u);
  w64(cb + GGS_CB_FAR, ic + 0x123);
  w32(cb + GGS_CB_FSYNR0, (1u << 4) | 3u);
  w32(0x1000 + GGS_GR1_CBFRSYNRA(4), 1);
  ggs_fault f;
  CHECK(ggs_take_fault(&s, &g_io, g, 4, &f));
  CHECK(f.kind == GGS_FK_PERMISSION && f.write && f.level == 3 && f.sid == 1);
  CHECK(f.window == 2 && f.entry == 0 && f.handle == 3 && f.terminateClient);
  CHECK(r32(cb + GGS_CB_FSR) == 0);   // cleared
  // Translation fault on an unmapped address of the render window.
  ggs_decode_fault(&s, 0x2u, 0x30000000ULL, 0, 0, 0, &f);
  CHECK(f.kind == GGS_FK_UNMAPPED && f.window == 1 && f.entry == -1 && f.terminateClient);
  // Kernel window, outside every window, external abort on a walk: the kernel's (GPU reset path).
  ggs_decode_fault(&s, 0x2u, 0x1f001000ULL, 0, 0, 0, &f);
  CHECK(f.kind == GGS_FK_UNMAPPED && f.window == 0 && !f.terminateClient);
  ggs_decode_fault(&s, 0x2u, 0x10000000ULL, 0, 0, 0, &f);
  CHECK(f.window == -1 && !f.terminateClient);
  ggs_decode_fault(&s, 0x10u, ic, 1u << 10, 0, 0, &f);
  CHECK(f.kind == GGS_FK_WALK && !f.terminateClient);
  ggs_decode_fault(&s, 0x80000002u, ic, 0, 0, 0, &f);
  CHECK(f.multi && f.kind == GGS_FK_UNMAPPED && f.entry == 0);
  ggs_decode_fault(&s, 0, 0, 0, 0, 0, &f);
  CHECK(f.kind == GGS_FK_NONE && !f.terminateClient);
  CHECK(strcmp(ggs_fault_kind_name(GGS_FK_PERMISSION), "permission") == 0);
}

static void test_conflict() {
  // A boot where the M3 diagnostic already routed SID 0 (mask 1) to bank 4 through slot 0: the kit must refuse.
  model_reset();
  w32(GGS_SMR(0), 0x80010000u);
  w32(GGS_S2CR(0), 4u);
  static ggs_space s;
  ggs_geometry g;
  ggs_profile p;
  CHECK(!ggs_decode(r32(0x20), r32(0x24), r32(0x28), &g) && ggs_default_profile(g, &p));
  memset(g_pool, 0, sizeof(g_pool));
  const ArmSmmu4KLiveVisibility vis = {nullptr, publish};
  CHECK(ggs_prepare(&s, p, g_pool, kPoolPhys, kPoolPages, g_ranges, g_entries, g_scratch, 2048, vis) == GGS_OK);
  static ArmSmmu4KStreamState streams;
  static ArmSmmu4KStreamResult routes;
  const int e = ggs_activate(&s, &g_io, p, false, &streams, &routes);
  CHECK(e == GGS_ERR_KIT && !s.poisoned && (r32(cbOff(4)) & 1) == 0);   // refused before programming the bank
  CHECK(r32(GGS_SMR(0)) == 0x80010000u && r32(GGS_S2CR(0)) == 4u);         // the other route untouched
  printf("M3-routed boot: activate %s (kit status %d), poisoned %d\n", ggs_err(e), s.last, s.poisoned);
  // Bypass-unmatched global policy (CLIENTPD or USFCFG clear): refused as well.
  model_reset();
  w32(0x00, 0x00200002u);
  memset(g_pool, 0, sizeof(g_pool));
  CHECK(ggs_prepare(&s, p, g_pool, kPoolPhys, kPoolPages, g_ranges, g_entries, g_scratch, 2048, vis) == GGS_OK);
  CHECK(ggs_activate(&s, &g_io, p, false, &streams, &routes) == GGS_ERR_KIT);
}

static void test_pt_switch() {
  // Independent encodings: WFI is 0x70268000 (also GuampGpuBlit.h), WAIT_FOR_ME 0x70138000,
  // CP_SMMU_TABLE_UPDATE count 4 0x70d30004, CP_MEM_WRITE count 5 0x703d8005, IB_PFE count 3 0x70bf8003,
  // SET_DRAW_STATE count 3 0x70438003, pkt4 CP_MISC_CNTL 0x840 x1 0x48084001.
  CHECK(ggs_pkt7(0x26, 0) == 0x70268000u);
  CHECK(ggs_pkt7(0x13, 0) == 0x70138000u);
  CHECK(ggs_pkt7(0x53, 4) == 0x70d30004u);
  CHECK(ggs_pkt7(0x3d, 5) == 0x703d8005u);
  CHECK(ggs_pkt7(0x3f, 3) == 0x70bf8003u);
  CHECK(ggs_pkt7(0x43, 3) == 0x70438003u);
  CHECK(ggs_pkt4(0x840, 1) == 0x48084001u);
  uint32_t out[GGS_PT_SWITCH_MAX_DWORDS];
  const uint64_t ttbr = ggs_ttbr0(0x87654000ULL, 7);
  CHECK(ttbr == 0x0007000087654000ULL);
  const uint32_t n = ggs_build_pt_switch(out, GGS_PT_SWITCH_MAX_DWORDS, ttbr, 7, 4, 0x20000100ULL, 0x20000200ULL, false);
  CHECK(n == 33);
  const uint32_t want[33] = {0x70268000u, 0x70138000u, 0x48084001u, 1u,
                             0x70138000u, 0x70bf8003u, 0x20000100u, 0u, 2u, 0x70268000u,
                             0x70268000u, 0x70138000u,
                             0x70d30004u, 0x87654000u, 0x00070000u, 7u, 4u,
                             0x703d8005u, 0x20000200u, 0u, 0x87654000u, 0x00070000u, 7u,
                             0x70138000u, 0x70268000u,
                             0x70438003u, 0x40000u, 0u, 0u,
                             0x70268000u, 0x70138000u, 0x48084001u, 0u};
  for (uint32_t i = 0; i < 33 && i < n; ++i) CHECK(out[i] == want[i]);
  const uint32_t m = ggs_build_pt_switch(out, GGS_PT_SWITCH_MAX_DWORDS, ttbr, 7, 4, 0x20000100ULL, 0x20000200ULL, true);
  CHECK(m == 42 && out[12] == ggs_pkt4(0x50e, 1) && out[13] == 1u && out[14] == 0x70d30004u);
  CHECK(out[27] == ggs_pkt7(0x3c, 6) && out[29] == 0x50fu);
  CHECK(ggs_build_pt_switch(out, 10, ttbr, 7, 4, 0x20000100ULL, 0x20000200ULL, false) == 0);
  CHECK(ggs_build_pt_switch(out, GGS_PT_SWITCH_MAX_DWORDS, ttbr, 7, 4, 0x20000102ULL, 0x20000200ULL, false) == 0);
}

int main() {
  test_geometry();
  test_activate();
  test_map_unmap();
  test_faults();
  test_conflict();
  test_pt_switch();
  printf("gpu_smmu_host: %u checks, %u failures\n", g_checks, g_fail);
  return g_fail ? 1 : 0;
}
