// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Adreno 610: the GPU SMMU (kgsl_smmu, qsmmu-v500 at 0x59a0000) driven through UniXNU's ArmSmmu4K kit
// (kits/ArmSmmu4K, the same qualified table/context/stream/live-update code sheng's NativeAdreno uses). This file adds
// only what is guamp specific; the descriptors, context image, stream routing and TLB maintenance stay the kit's:
//   1. the SMMU geometry decoded from the IDs the phone reported (M2 log: idr0 4c017e04 idr1 20000005 idr2 00005511):
//      4 SMRs, 15 SID bits, 5 context banks, 4 KiB pages, context banks at 0x8000, stage 1 only, OAS 36 bit, 4K tables;
//   2. the guamp profile: SIDs 0 and 1 (bengal-gpu.dtsi gfx3d_user iommus <&kgsl_smmu 0 1> = SID 0 mask 1; the kit
//      attaches exact matches only, so two SMRs), the bank, the ASID, non-coherent table walks (kgsl_smmu has no
//      dma-coherent in msm-arm-smmu-bengal.dtsi, unlike apps_smmu), the fault interrupts (SPI 163 global,
//      SPI 167 + bank per context bank), the downstream ACTLR (0x30b, +15 deep prefetch, all GFX banks);
//   3. one GPU address space split into fixed client windows, the model of NativeAdrenoPublish.cpp (kClientWindows),
//      with the client layout the tb128fu A610 port runs (render server 1 GiB, it reached 934 MiB there; three more
//      clients of 0x0AAA9000), plus a 16 MiB kernel window below them (ring, scratch, SQE, fences).
//      A registry of every mapped range, kept sorted by IOVA as the kit's live update requires ("before" lists ALL
//      mapped resources of the region), with an allocator per window and per-BO owner tags, maps and unmaps buffers
//      (BoAlloc/BoMap/BoImport/BoFree of the NativeAdreno ABI v1) in one kit transaction per buffer;
//   4. context fault decode (FSR, FAR, FSYNR0/1, CBFRSYNRA SID) to the owning client and an action;
//   5. the CP pagetable switch stream of kgsl for a6xx (adreno_iommu.c _adreno_iommu_set_pt_v2_a6xx), for a later
//      per-process TTBR0 mode. NOT used by the window model: the kit's live update re-validates the bank's TTBR0
//      against its configured root, so a bank whose TTBR0 the CP switched to another root can only be updated after
//      switching it back (see analisi/gpu-ios/SMMU-A610.md).
// Pure code: no allocation, no IOKit; register access through the kit's ArmSmmu4KIo, table visibility through
// ArmSmmu4KLiveVisibility. Host-tested by tests/gpu_smmu_host.cpp against the kit sources and a page-walking SMMU model.
// Not wired into GpuService.inc; the M3 diagnostic (GuampGpuM2.h) programs SMR 0 with a mask and must not run in the
// same boot (the kit refuses a valid masked SMR covering its SIDs).
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "ArmSmmu4KLive.h"
#include "ArmSmmu4KStreams.h"

// ---- 1. geometry -----------------------------------------------------------------------------------------------------
#define GGS_PHYS 0x059a0000ULL                    // msm-arm-smmu-bengal.dtsi kgsl_smmu reg "base"
#define GGS_SIZE 0x10000u
#define GGS_IRQ_GLOBAL_SPI 163u                    // #global-interrupts = <1>, then one per context bank
#define GGS_IRQ_CONTEXT_SPI0 167u
#define GGS_ACTLR 0x30bu                           // qcom,actlr <0x0 0x3ff 0x30B>
// Context bank register offsets (SMMUv2, from the bank base) and the GR1 fault syndrome.
#define GGS_CB_ACTLR 0x04u
#define GGS_CB_FSR 0x58u
#define GGS_CB_FAR 0x60u
#define GGS_CB_FSYNR0 0x68u
#define GGS_CB_FSYNR1 0x6cu
#define GGS_GR1_CBFRSYNRA(n) (0x400u + 4u * (n))   // in GR1 (page 1)

struct ggs_geometry {
  unsigned smrs, sidBits, banks, s2Banks, pageShift, globalPages, ias, oas, ubs;
  bool sms, s1, exids, ptfs4k;
  uint32_t cbBase;      // offset of context bank 0
  uint32_t gr1;         // offset of GR1
};

static inline unsigned ggs_width_bits(unsigned code) {
  switch (code) {
    case 0: return 32;
    case 1: return 36;
    case 2: return 40;
    case 3: return 42;
    case 4: return 44;
    case 5: return 48;
    default: return 0;
  }
}

// Decodes the three ID registers; returns null when the SMMU can host the guamp GPU context, else the reason.
static inline const char *ggs_decode(uint32_t id0, uint32_t id1, uint32_t id2, ggs_geometry *g) {
  *g = {};
  g->smrs = id0 & 0xffu;
  g->sidBits = (id0 >> 9) & 0xfu;
  g->sms = (id0 >> 27) & 1u;
  g->s1 = (id0 >> 30) & 1u;
  g->exids = (id0 >> 8) & 1u;
  g->banks = id1 & 0xffu;
  g->s2Banks = (id1 >> 16) & 0xffu;
  g->pageShift = (id1 >> 31) ? 16 : 12;
  g->globalPages = 1u << (((id1 >> 28) & 7u) + 1u);
  g->gr1 = 1u << g->pageShift;
  g->cbBase = g->globalPages << g->pageShift;
  g->ias = ggs_width_bits(id2 & 0xfu);
  g->oas = ggs_width_bits((id2 >> 4) & 0xfu);
  g->ubs = ggs_width_bits((id2 >> 8) & 0xfu);
  g->ptfs4k = (id2 >> 12) & 1u;
  if (!g->s1) return "no stage 1";
  if (!g->sms) return "no stream matching";
  if (!g->ptfs4k) return "no 4K tables";
  if (g->pageShift != 12) return "64K register pages";
  if (g->smrs < 2) return "fewer than 2 SMRs";
  if (!g->banks || g->banks <= g->s2Banks) return "no stage 1 bank";
  if (!g->oas) return "bad OAS";
  if (2ull * g->cbBase > GGS_SIZE) return "register space larger than the 64 KiB map";
  return nullptr;
}

// ---- 2. profile ------------------------------------------------------------------------------------------------------
struct ggs_profile {
  unsigned bank;            // context bank for the GPU user space
  unsigned asid;            // the bank's ASID (qcom,no-dynamic-asid: one fixed ASID per bank)
  unsigned slots[2];        // SMR/S2CR slots for SID 0 and SID 1
  uint16_t sids[2];
  unsigned outputBits;      // min(OAS, 36)
  unsigned inputBits;       // 32: the GPU VA space of the window model lies below 4 GiB
  unsigned coherentWalk;    // 0: kgsl_smmu is not dma-coherent
  unsigned irqContext;      // GIC INTID of the bank's context fault line
  unsigned irqGlobal;       // GIC INTID of the global fault line
};

// The default: the last bank (the one the M3 diagnostic found free, CBAR 0 at boot), ASID bank + 1, SMR slots 2 and 3
// (slot 0 is the one the M3 diagnostic uses; leaving 0 and 1 free keeps a diagnostic boot and this one apart).
static inline bool ggs_default_profile(const ggs_geometry &g, ggs_profile *p) {
  *p = {};
  if (g.smrs < 4 || !g.banks) return false;
  p->bank = g.banks - 1;
  p->asid = p->bank + 1;
  p->slots[0] = g.smrs - 2;
  p->slots[1] = g.smrs - 1;
  p->sids[0] = 0;
  p->sids[1] = 1;
  p->outputBits = g.oas < 36 ? g.oas : 36;
  p->inputBits = 32;
  p->coherentWalk = 0;
  p->irqContext = 32 + GGS_IRQ_CONTEXT_SPI0 + p->bank;
  p->irqGlobal = 32 + GGS_IRQ_GLOBAL_SPI;
  return true;
}

// ---- 3. windows, registry, allocator --------------------------------------------------------------------------------
// One aperture 0x1F000000..0x80000000 (32-bit input). Windows, in IOVA order:
#define GGS_APERTURE_IOVA 0x1f000000ULL
#define GGS_APERTURE_BYTES 0x61000000ULL
#define GGS_WINDOWS 5u
#define GGS_KERNEL_WINDOW 0u      // ring, scratch, SQE, fences, the kernel's own BOs
#define GGS_RENDER_WINDOW 1u      // backboardd / the render server
static const uint64_t kGgsWindowBase[GGS_WINDOWS] = {0x1f000000ULL, 0x20000000ULL, 0x60000000ULL, 0x6aaa9000ULL,
                                                      0x75552000ULL};
static const uint64_t kGgsWindowBytes[GGS_WINDOWS] = {0x01000000ULL, 0x40000000ULL, 0x0aaa9000ULL, 0x0aaa9000ULL,
                                                       0x0aaa9000ULL};
#define GGS_LEAF_BUDGET (GGS_APERTURE_BYTES / 4096u)   // 397312 leaves, every page of the aperture mappable
#define GGS_TABLE_PAGES 780u                           // what LivePrepare preallocates for it (host test: 779)

struct ggs_entry {
  uint32_t window;      // owning window / client
  uint32_t handle;      // owner's BO handle (0 = none)
};

// The caller owns all storage (no allocation here): ranges/entries of `capacity` elements, kept sorted by IOVA.
struct ggs_space {
  ArmSmmu4KLiveArena arena;
  ArmSmmu4KLiveState state;
  ArmSmmu4KContextConfig context;
  ArmSmmu4KLiveVisibility visibility;
  const ArmSmmu4KIo *io;
  unsigned bank, pollBudget;
  ArmSmmu4KRange *ranges;
  ggs_entry *entries;
  size_t count, capacity;
  ArmSmmu4KRange *scratch;   // capacity elements: the change list of one operation
  bool ready, poisoned;
  ArmSmmu4KStatus last;
};

// One physically contiguous piece of a buffer (4K aligned).
struct ggs_chunk { uint64_t physical, bytes; };

enum : int {
  GGS_OK = 0,
  GGS_ERR_ARG = -1,      // bad input
  GGS_ERR_NOSPACE = -2,  // window full / registry full
  GGS_ERR_NOTFOUND = -3, // no such BO
  GGS_ERR_KIT = -4,      // the kit refused or failed (space->last); a failure after a store poisons the space
  GGS_ERR_POISONED = -5, // a previous kit failure: tables and resources must stay owned until reset
};

static inline const char *ggs_err(int e) {
  switch (e) {
    case GGS_OK: return "ok";
    case GGS_ERR_ARG: return "bad argument";
    case GGS_ERR_NOSPACE: return "no space";
    case GGS_ERR_NOTFOUND: return "not found";
    case GGS_ERR_KIT: return "kit error";
    default: return "poisoned";
  }
}

// Alignment of a buffer's IOVA: 2 MiB for >= 2 MiB, 64 KiB for >= 64 KiB, else 4 KiB. The kit maps 4 KiB leaves only;
// the alignment keeps the IOVA layout ready for 64K/2M descriptors and matches Turnip's expectations of large BOs.
static inline uint64_t ggs_align_for(uint64_t bytes) {
  return bytes >= 0x200000ULL ? 0x200000ULL : bytes >= 0x10000ULL ? 0x10000ULL : 0x1000ULL;
}

// Prepares the (inactive, empty) table pool of the aperture. tableMapping/tablePhysical: a contiguous, 4K aligned
// pool, mapped non-cacheable (or published by the visibility hook) and outside every range that will be mapped.
static inline int ggs_prepare(ggs_space *s, const ggs_profile &p, volatile uint64_t *tableMapping, uint64_t tablePhysical,
                              size_t tablePages, ArmSmmu4KRange *ranges, ggs_entry *entries, ArmSmmu4KRange *scratch,
                              size_t capacity, const ArmSmmu4KLiveVisibility &visibility) {
  if (!s || !tableMapping || (tablePhysical & 4095) || !tablePages || !ranges || !entries || !scratch || !capacity ||
      !visibility.publish_table_page)
    return GGS_ERR_ARG;
  *s = {};
  s->arena.config = {p.inputBits, p.outputBits, tablePhysical, GGS_LEAF_BUDGET};
  s->arena.iova = GGS_APERTURE_IOVA;
  s->arena.bytes = GGS_APERTURE_BYTES;
  s->arena.upper_region = 0;
  s->arena.mapping = tableMapping;
  s->arena.capacity_pages = tablePages;
  s->visibility = visibility;
  s->ranges = ranges;
  s->entries = entries;
  s->scratch = scratch;
  s->capacity = capacity;
  s->bank = p.bank;
  s->pollBudget = 4096;
  s->last = ArmSmmu4KLivePrepare(&s->arena, &s->state);
  if (s->last != ArmSmmu4KSuccess) return GGS_ERR_KIT;
  ArmSmmu4KLiveResult audit;
  s->last = ArmSmmu4KLiveAudit(&s->arena, &s->state, nullptr, 0, &audit);
  if (s->last != ArmSmmu4KSuccess) {
    s->poisoned = true;
    return GGS_ERR_KIT;
  }
  // Every intermediate page the kit preallocated must reach the walker before the bank is enabled.
  for (size_t i = 0; i < s->state.tables.table_pages; ++i)
    if (visibility.publish_table_page(visibility.cookie, tableMapping + i * 512, 4096)) {
      s->poisoned = true;
      return GGS_ERR_KIT;
    }
  s->context = {};
  s->context.lower = &s->arena.config;
  s->context.lower_tables = &s->state.tables;
  s->context.asid = p.asid;
  s->context.coherent_walk = p.coherentWalk;
  s->context.stall_on_fault = 0;
  s->context.continue_after_fault = 0;   // terminate the faulting transaction; the GPU recovers through its hang path
  return GGS_OK;
}

// The kit attaches only to slots whose S2CR already faults. After reset an S2CR reads 0 (translate, bank 0), so each
// target slot with an INVALID SMR (nothing matches through it) is first set to FAULT (TYPE 2) and read back. A slot
// with a valid SMR is left alone (the kit then checks it is an exact match of our SID, else refuses).
#define GGS_SMR(n) (0x800u + 4u * (n))
#define GGS_S2CR(n) (0xc00u + 4u * (n))
#define GGS_S2CR_FAULT (2u << 16)
static inline bool ggs_fault_free_slots(const ArmSmmu4KIo *io, const ggs_profile &p) {
  for (unsigned i = 0; i < 2; ++i) {
    uint32_t smr = 0, s2cr = 0;
    if (io->read32(io->cookie, GGS_SMR(p.slots[i]), &smr)) return false;
    if (smr & 0x80000000u) continue;
    if (io->write32(io->cookie, GGS_S2CR(p.slots[i]), GGS_S2CR_FAULT) || io->barrier(io->cookie) ||
        io->read32(io->cookie, GGS_S2CR(p.slots[i]), &s2cr) || ((s2cr >> 16) & 3u) != 2u)
      return false;
  }
  return true;
}

// Programs the (disabled) bank, then routes SIDs 0 and 1 to it. sCR0 must fault unmatched streams (the phone:
// 00200406 / 00280406); GPU masters quiesced. Optional ACTLR as downstream.
static inline int ggs_activate(ggs_space *s, const ArmSmmu4KIo *io, const ggs_profile &p, bool actlr,
                               ArmSmmu4KStreamState *streams, ArmSmmu4KStreamResult *routes) {
  if (!s || !io || !streams || !routes || s->ready) return GGS_ERR_ARG;
  if (s->poisoned) return GGS_ERR_POISONED;
  if (!ggs_fault_free_slots(io, p)) {
    s->last = ArmSmmu4KIoError;
    return GGS_ERR_KIT;   // nothing routed yet: not poisoned
  }
  const ArmSmmu4KStream route[2] = {{p.slots[0], p.sids[0]}, {p.slots[1], p.sids[1]}};
  // Read-only preflight (global fault policy, target slots faulting, no other live route of SID 0/1 or of the bank,
  // e.g. the M3 diagnostic's masked SMR): a refusal here leaves the bank unprogrammed and the space reusable.
  s->last = ArmSmmu4KCheckFaultedStreams(io, route, 2, p.bank, routes);
  if (s->last != ArmSmmu4KSuccess) return GGS_ERR_KIT;
  ArmSmmu4KProgramResult program;
  s->last = ArmSmmu4KProgramDisabledContext(io, p.bank, &s->context, s->pollBudget, &program);
  if (s->last != ArmSmmu4KSuccess) {
    s->poisoned = true;
    return GGS_ERR_KIT;
  }
  if (actlr && io->write32(io->cookie, (size_t)program.context_offset + GGS_CB_ACTLR, GGS_ACTLR)) {
    s->poisoned = true;
    s->last = ArmSmmu4KIoError;
    return GGS_ERR_KIT;
  }
  *streams = {};
  s->last = ArmSmmu4KAttachStreams(io, streams, route, 2, p.bank, &s->context, routes);
  if (s->last != ArmSmmu4KSuccess) {
    s->poisoned = true;
    return GGS_ERR_KIT;
  }
  s->io = io;
  s->ready = true;
  return GGS_OK;
}

// First IOVA >= base, aligned, where `bytes` fit before the next range / window end. Ranges are sorted.
static inline bool ggs_find_gap(const ggs_space *s, unsigned window, uint64_t bytes, uint64_t *iova) {
  const uint64_t lo = kGgsWindowBase[window], hi = lo + kGgsWindowBytes[window], align = ggs_align_for(bytes);
  uint64_t at = (lo + align - 1) & ~(align - 1);
  for (size_t i = 0; i < s->count; ++i) {
    const ArmSmmu4KRange &r = s->ranges[i];
    if (r.iova + r.bytes <= at) continue;
    if (r.iova >= hi) break;
    if (r.iova >= at && r.iova - at >= bytes) break;
    at = (r.iova + r.bytes + align - 1) & ~(align - 1);
  }
  if (at < lo || at > hi || hi - at < bytes) return false;
  *iova = at;
  return true;
}

static inline size_t ggs_lower_bound(const ggs_space *s, uint64_t iova) {
  size_t lo = 0, hi = s->count;
  while (lo < hi) {
    const size_t mid = lo + (hi - lo) / 2;
    if (s->ranges[mid].iova < iova) lo = mid + 1;
    else hi = mid;
  }
  return lo;
}

static inline int ggs_kit_batch(ggs_space *s, size_t changes, unsigned map) {
  ArmSmmu4KLiveRegion lower = {&s->arena, &s->state, s->ranges, s->count};
  ArmSmmu4KLiveResult r;
  s->last = ArmSmmu4KLiveUpdateContextBatch(s->io, s->bank, &s->context, &lower, nullptr, 0, &s->visibility, s->scratch,
                                            changes, map, s->pollBudget, &r);
  if (s->last == ArmSmmu4KSuccess) return GGS_OK;
  if (s->state.poisoned) s->poisoned = true;
  return GGS_ERR_KIT;
}

// Maps a buffer made of `n` chunks at consecutive IOVAs in `window`, owner `handle`. permissions: ArmSmmu4KRead |
// optional Write (the GPU never executes from a BO through the SMMU on a6xx; privileged only for kernel buffers).
// writeback 0 (normal non-cacheable: guamp RAM is mapped NC for devices), shareability 2 (outer).
static inline int ggs_map(ggs_space *s, unsigned window, uint32_t handle, const ggs_chunk *chunks, size_t n,
                          unsigned permissions, uint64_t *iovaOut) {
  if (!s || !chunks || !n || !iovaOut || window >= GGS_WINDOWS || !handle || !(permissions & ArmSmmu4KRead))
    return GGS_ERR_ARG;
  if (s->poisoned) return GGS_ERR_POISONED;
  if (!s->ready) return GGS_ERR_ARG;
  if (n > s->capacity - s->count) return GGS_ERR_NOSPACE;
  uint64_t total = 0;
  for (size_t i = 0; i < n; ++i) {
    if (!chunks[i].bytes || ((chunks[i].physical | chunks[i].bytes) & 4095) || chunks[i].bytes > kGgsWindowBytes[window])
      return GGS_ERR_ARG;
    total += chunks[i].bytes;
    if (total > kGgsWindowBytes[window]) return GGS_ERR_NOSPACE;
  }
  for (size_t i = 0; i < s->count; ++i)
    if (s->entries[i].window == window && s->entries[i].handle == handle) return GGS_ERR_ARG;   // already mapped
  uint64_t iova;
  if (!ggs_find_gap(s, window, total, &iova)) return GGS_ERR_NOSPACE;
  uint64_t at = iova;
  for (size_t i = 0; i < n; ++i) {
    s->scratch[i] = {at, chunks[i].physical, chunks[i].bytes, permissions, 0, 2};
    at += chunks[i].bytes;
  }
  const int e = ggs_kit_batch(s, n, 1);
  if (e) return e;
  // Commit: insert the n ranges (consecutive, sorted) at their place.
  const size_t pos = ggs_lower_bound(s, iova);
  for (size_t i = s->count; i > pos; i = i - 1) {
    s->ranges[i - 1 + n] = s->ranges[i - 1];
    s->entries[i - 1 + n] = s->entries[i - 1];
  }
  for (size_t i = 0; i < n; ++i) {
    s->ranges[pos + i] = s->scratch[i];
    s->entries[pos + i] = {window, handle};
  }
  s->count += n;
  *iovaOut = iova;
  return GGS_OK;
}

// Unmaps every range of (window, handle). On success the translation is gone and the ASID invalidated (by the kit);
// the caller still owns the pages until the GPU work that used them has retired (fence), as with any unmap.
static inline int ggs_unmap(ggs_space *s, unsigned window, uint32_t handle) {
  if (!s || window >= GGS_WINDOWS || !handle) return GGS_ERR_ARG;
  if (s->poisoned) return GGS_ERR_POISONED;
  if (!s->ready) return GGS_ERR_ARG;
  size_t n = 0, first = 0;
  for (size_t i = 0; i < s->count; ++i)
    if (s->entries[i].window == window && s->entries[i].handle == handle) {
      if (!n) first = i;
      s->scratch[n] = s->ranges[i];
      n = n + 1;
    }
  if (!n) return GGS_ERR_NOTFOUND;
  const int e = ggs_kit_batch(s, n, 0);
  if (e) return e;
  // A buffer's ranges are consecutive in IOVA and so in the registry: remove [first, first + n).
  for (size_t i = first; i + n < s->count; ++i) {
    s->ranges[i] = s->ranges[i + n];
    s->entries[i] = s->entries[i + n];
  }
  s->count -= n;
  return GGS_OK;
}

// Removes every buffer of a client window (terminated client, after the GPU is quiesced). One kit transaction.
static inline int ggs_unmap_window(ggs_space *s, unsigned window, size_t *removed) {
  if (!s || window >= GGS_WINDOWS || !removed) return GGS_ERR_ARG;
  *removed = 0;
  if (s->poisoned) return GGS_ERR_POISONED;
  if (!s->ready) return GGS_ERR_ARG;
  size_t n = 0, first = 0;
  for (size_t i = 0; i < s->count; ++i)
    if (s->entries[i].window == window) {
      if (!n) first = i;
      s->scratch[n] = s->ranges[i];
      n = n + 1;
    }
  if (!n) return GGS_OK;
  const int e = ggs_kit_batch(s, n, 0);
  if (e) return e;
  for (size_t i = first; i + n < s->count; ++i) {
    s->ranges[i] = s->ranges[i + n];
    s->entries[i] = s->entries[i + n];
  }
  s->count -= n;
  *removed = n;
  return GGS_OK;
}

// The window an IOVA belongs to, or -1.
static inline int ggs_window_of(uint64_t iova) {
  for (unsigned w = 0; w < GGS_WINDOWS; ++w)
    if (iova >= kGgsWindowBase[w] && iova - kGgsWindowBase[w] < kGgsWindowBytes[w]) return (int)w;
  return -1;
}

// The registry entry holding iova, or -1.
static inline long ggs_find(const ggs_space *s, uint64_t iova) {
  const size_t i = ggs_lower_bound(s, iova + 1);
  if (!i) return -1;
  const ArmSmmu4KRange &r = s->ranges[i - 1];
  return iova >= r.iova && iova - r.iova < r.bytes ? (long)(i - 1) : -1;
}

// ---- 4. faults --------------------------------------------------------------------------------------------------------
// FSR (Linux arm-smmu.h ARM_SMMU_CB_FSR_*): TF 1, AFF 2, PF 3, EF 4, TLBMCF 5, TLBLKF 6, ASF 7, UUT 8, SS 30, MULTI 31.
// FSYNR0: PLVL [1:0], WNR 4, PNU 5, IND 6, NSATTR 8, PTWF 10, AFR 11, S1CBNDX [23:16]. CBFRSYNRA: SID [15:0].
#define GGS_FSR_FAULT 0x800001feu   // every fault bit + MULTI (SS excluded: stall mode is not used)
enum ggs_fault_kind : unsigned { GGS_FK_NONE, GGS_FK_UNMAPPED, GGS_FK_PERMISSION, GGS_FK_WALK, GGS_FK_OTHER };

struct ggs_fault {
  uint32_t fsr, fsynr0, fsynr1, frsynra;
  uint64_t far;
  unsigned sid, level;
  bool write, external, translation, permission, multi;
  int window;           // window of FAR, -1 outside every window
  long entry;           // registry index holding FAR, -1 when nothing is mapped there
  uint32_t handle;      // owner BO handle when entry >= 0
  ggs_fault_kind kind;
  bool terminateClient; // a client window: drop that client (unmap window after quiesce); kernel window: GPU reset
};

static inline const char *ggs_fault_kind_name(ggs_fault_kind k) {
  switch (k) {
    case GGS_FK_NONE: return "none";
    case GGS_FK_UNMAPPED: return "unmapped";
    case GGS_FK_PERMISSION: return "permission";
    case GGS_FK_WALK: return "table walk";
    default: return "other";
  }
}

// Decodes raw syndrome values (read by the caller from the bank and GR1) against the registry.
static inline void ggs_decode_fault(const ggs_space *s, uint32_t fsr, uint64_t far, uint32_t fsynr0, uint32_t fsynr1,
                                    uint32_t frsynra, ggs_fault *f) {
  *f = {};
  f->fsr = fsr;
  f->far = far;
  f->fsynr0 = fsynr0;
  f->fsynr1 = fsynr1;
  f->frsynra = frsynra;
  f->sid = frsynra & 0xffffu;
  f->level = fsynr0 & 3u;
  f->write = (fsynr0 >> 4) & 1u;
  f->translation = (fsr >> 1) & 1u;
  f->permission = (fsr >> 3) & 1u;
  f->external = (fsr >> 4) & 1u;
  f->multi = (fsr >> 31) & 1u;
  f->window = ggs_window_of(far);
  f->entry = s ? ggs_find(s, far) : -1;
  if (f->entry >= 0) f->handle = s->entries[f->entry].handle;
  if (!(fsr & GGS_FSR_FAULT)) f->kind = GGS_FK_NONE;
  else if (f->permission) f->kind = GGS_FK_PERMISSION;
  else if (f->external || ((fsynr0 >> 10) & 1u)) f->kind = GGS_FK_WALK;
  else if (f->translation) f->kind = GGS_FK_UNMAPPED;
  else f->kind = GGS_FK_OTHER;
  // A fault in a client window is that client's; anything else (kernel window, outside every window, table walk
  // errors) is the kernel's and takes the GPU reset path.
  f->terminateClient = f->kind != GGS_FK_NONE && f->kind != GGS_FK_WALK && f->window > (int)GGS_KERNEL_WINDOW;
}

// Reads the syndrome of the bank through the kit's I/O, decodes it and clears FSR (write 1 to clear). The register
// offsets come from the geometry. Returns false on an I/O error.
static inline bool ggs_take_fault(const ggs_space *s, const ArmSmmu4KIo *io, const ggs_geometry &g, unsigned bank,
                                  ggs_fault *f) {
  const size_t cb = (size_t)g.cbBase + ((size_t)bank << g.pageShift);
  uint32_t fsr = 0, s0 = 0, s1 = 0, fr = 0;
  uint64_t far = 0;
  if (io->read32(io->cookie, cb + GGS_CB_FSR, &fsr) || io->read64(io->cookie, cb + GGS_CB_FAR, &far) ||
      io->read32(io->cookie, cb + GGS_CB_FSYNR0, &s0) || io->read32(io->cookie, cb + GGS_CB_FSYNR1, &s1) ||
      io->read32(io->cookie, (size_t)g.gr1 + GGS_GR1_CBFRSYNRA(bank), &fr))
    return false;
  ggs_decode_fault(s, fsr, far, s0, s1, fr, f);
  if (fsr && (io->write32(io->cookie, cb + GGS_CB_FSR, fsr) || io->barrier(io->cookie))) return false;
  return true;
}

// ---- 5. CP pagetable switch (kgsl a6xx, for a later per-process mode) ------------------------------------------------
// adreno_iommu.c adreno_iommu_set_pt_generate_cmds for a6xx without ADRENO_APRIV (A610: adreno-gpulist.h:1399, features
// 64BIT | CONTENT_PROTECTION | PREEMPTION): APRIV on through CP_MISC_CNTL (0x840, ADRENO_REG_CP_CNTL), a NOP indirect
// buffer to stall prefetch, then _adreno_iommu_set_pt_v2_a6xx: idle, CP_SMMU_TABLE_UPDATE {ttbr0 lo, hi, contextidr,
// cb_num} (4 dwords), CP_MEM_WRITE of {ttbr0, contextidr} to the ring's pagetable info, WAIT_FOR_ME, idle; then
// cp_invalidate_state (CP_SET_DRAW_STATE 0x40000, 0, 0) and APRIV off. The optional perf counter SRAM clear of kgsl
// (RBBM_PERFCTR_SRAM_INIT_CMD 0x50e, wait 0x50f) is included when `perfClear` is set.
#define GGS_CP_WAIT_FOR_ME 0x13u
#define GGS_CP_WAIT_FOR_IDLE 0x26u
#define GGS_CP_WAIT_REG_MEM 0x3cu
#define GGS_CP_MEM_WRITE 0x3du
#define GGS_CP_INDIRECT_BUFFER_PFE 0x3fu
#define GGS_CP_SET_DRAW_STATE 0x43u
#define GGS_CP_SMMU_TABLE_UPDATE 0x53u
#define GGS_A6XX_CP_MISC_CNTL 0x840u
#define GGS_A6XX_RBBM_PERFCTR_SRAM_INIT_CMD 0x50eu
#define GGS_A6XX_RBBM_PERFCTR_SRAM_INIT_STATUS 0x50fu
#define GGS_PT_SWITCH_MAX_DWORDS 48u

static inline uint32_t ggs_odd_parity(uint32_t v) {   // adreno_pm4types.h pm4_calc_odd_parity_bit
  return (0x9669u >> (0xfu & (v ^ (v >> 4) ^ (v >> 8) ^ (v >> 12) ^ (v >> 16) ^ (v >> 20) ^ (v >> 24) ^ (v >> 28)))) & 1u;
}
static inline uint32_t ggs_pkt7(uint32_t op, uint32_t cnt) {
  return 0x70000000u | (cnt & 0x3fffu) | (ggs_odd_parity(cnt) << 15) | ((op & 0x7fu) << 16) |
         (ggs_odd_parity(op) << 23);
}
static inline uint32_t ggs_pkt4(uint32_t reg, uint32_t cnt) {
  return 0x40000000u | (cnt & 0x7fu) | (ggs_odd_parity(cnt) << 7) | ((reg & 0x3ffffu) << 8) |
         (ggs_odd_parity(reg) << 27);
}

// TTBR0 of a root with its ASID in [63:48] (what the kit writes, what kgsl's DOMAIN_ATTR_TTBR0 returns).
static inline uint64_t ggs_ttbr0(uint64_t root, unsigned asid) { return root | ((uint64_t)(asid & 0xffffu) << 48); }

// Builds the switch into out[cap]; nopIb: GPU address of a 2-dword NOP indirect buffer (kgsl setstate NOP),
// ptInfo: GPU address where CP_MEM_WRITE records the active {ttbr0, contextidr}. Returns dwords or 0.
static inline uint32_t ggs_build_pt_switch(uint32_t *out, uint32_t cap, uint64_t ttbr0, uint32_t contextidr,
                                           unsigned bank, uint64_t nopIb, uint64_t ptInfo, bool perfClear) {
  if (!out || cap < GGS_PT_SWITCH_MAX_DWORDS || (nopIb & 3) || (ptInfo & 3)) return 0;
  uint32_t n = 0;
  // adreno_iommu_set_apriv(1): wait for idle, wait for me, CP_CNTL = 1
  out[n++] = ggs_pkt7(GGS_CP_WAIT_FOR_IDLE, 0);
  out[n++] = ggs_pkt7(GGS_CP_WAIT_FOR_ME, 0);
  out[n++] = ggs_pkt4(GGS_A6XX_CP_MISC_CNTL, 1);
  out[n++] = 1;
  // _adreno_iommu_add_idle_indirect_cmds
  out[n++] = ggs_pkt7(GGS_CP_WAIT_FOR_ME, 0);
  out[n++] = ggs_pkt7(GGS_CP_INDIRECT_BUFFER_PFE, 3);
  out[n++] = (uint32_t)nopIb;
  out[n++] = (uint32_t)(nopIb >> 32);
  out[n++] = 2;
  out[n++] = ggs_pkt7(GGS_CP_WAIT_FOR_IDLE, 0);
  // _adreno_iommu_set_pt_v2_a6xx
  out[n++] = ggs_pkt7(GGS_CP_WAIT_FOR_IDLE, 0);
  out[n++] = ggs_pkt7(GGS_CP_WAIT_FOR_ME, 0);
  if (perfClear) {
    out[n++] = ggs_pkt4(GGS_A6XX_RBBM_PERFCTR_SRAM_INIT_CMD, 1);
    out[n++] = 1;
  }
  out[n++] = ggs_pkt7(GGS_CP_SMMU_TABLE_UPDATE, 4);
  out[n++] = (uint32_t)ttbr0;
  out[n++] = (uint32_t)(ttbr0 >> 32);
  out[n++] = contextidr;
  out[n++] = bank;
  out[n++] = ggs_pkt7(GGS_CP_MEM_WRITE, 5);   // cp_mem_packet(CP_MEM_WRITE, 4, 1) on a 64-bit GPU: 4 + 1
  out[n++] = (uint32_t)ptInfo;
  out[n++] = (uint32_t)(ptInfo >> 32);
  out[n++] = (uint32_t)ttbr0;
  out[n++] = (uint32_t)(ttbr0 >> 32);
  out[n++] = contextidr;
  out[n++] = ggs_pkt7(GGS_CP_WAIT_FOR_ME, 0);
  out[n++] = ggs_pkt7(GGS_CP_WAIT_FOR_IDLE, 0);
  if (perfClear) {
    out[n++] = ggs_pkt7(GGS_CP_WAIT_REG_MEM, 6);
    out[n++] = 0x3;
    out[n++] = GGS_A6XX_RBBM_PERFCTR_SRAM_INIT_STATUS;
    out[n++] = 0x0;
    out[n++] = 0x1;
    out[n++] = 0x1;
    out[n++] = 0x0;
  }
  // cp_invalidate_state (a5xx+)
  out[n++] = ggs_pkt7(GGS_CP_SET_DRAW_STATE, 3);
  out[n++] = 0x40000;
  out[n++] = 0;
  out[n++] = 0;
  // adreno_iommu_set_apriv(0)
  out[n++] = ggs_pkt7(GGS_CP_WAIT_FOR_IDLE, 0);
  out[n++] = ggs_pkt7(GGS_CP_WAIT_FOR_ME, 0);
  out[n++] = ggs_pkt4(GGS_A6XX_CP_MISC_CNTL, 1);
  out[n++] = 0;
  return n;
}
