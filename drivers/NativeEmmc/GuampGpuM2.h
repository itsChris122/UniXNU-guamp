// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Adreno 610 milestone M2: the CP (SQE) brought up on the power and clocks of M1 (GuampGpuM1.h). Pure code, no
// kernel calls: the owner (GpuService.inc) supplies register access, delays, cache maintenance, the physically
// contiguous memory and the SQE microcode, and logs the gm2_state this code fills. Host-tested by tests/gpu_host.cpp.
// References: the Motorola kgsl of this phone, drivers/gpu/msm in android_kernel_motorola_sm6225:
//   adreno-gpulist.h:1399  adreno_gpu_core_a610: sqefw "a630_sqe.fw", vbif = a640_vbif_regs (1135: GBIF QSB sides
//                          0x71620, RBBM_GBIF_CLIENT_QOS_CNTL 3), prim_fifo_threshold 0x80000, no APRIV feature
//   adreno_a6xx.c:1078     a6xx_soft_reset: RBBM_SW_RESET_CMD 1, dummy read, 0; adreno.h:1854 GBIF_HALT 0 and
//                          RBBM_GBIF_HALT 0 (no GMU, not a615 family)
//   adreno_a6xx.c:438      a6xx_start, the A610 values: UCHE write range / trap / write-thru, UCHE_FILTER_CNTL 0x804,
//                          UCHE_CACHE_WAYS 4, CP_ROQ_THRESHOLDS 0x40201b16 / 0x00800060, CP_MEM_POOL_SIZE 48 (dbg 47),
//                          PC_DBG_ECO_CNTL 0x80000, CP_AHB_CNTL 1, UCHE_CLIENT_PF 1; a6xx_enable_64bit (233): the CP
//                          one (CP_ADDR_MODE_CNTL 1). Not done here: hardware clock gating, CP protect, preemption,
//                          the UBWC/NC mode registers and the perf counters (no draw, no context switch in M2).
//   adreno_a6xx.c:885      a6xx_rb_start: CP_RB_RPTR_ADDR, CP_RB_CNTL (adreno_a6xx.h:155 BLKSZ log2(4) << 8 |
//                          log2(dwords / 2); 2444: bit 27 without APRIV, the RPTR is read from the register),
//                          CP_RB_BASE, CP_SQE_INSTR_BASE (= firmware after its first dword, _load_firmware 1002),
//                          CP_SQE_CNTL 1 (unhalt), then a6xx_send_cp_init (771): CP_ME_INIT; adreno.c set_unsecured
//                          mode without zap: RBBM_SECVID_TRUST_CNTL 0 (written directly, no zap shader, no SCM).
//   CP_ME_INIT ordinals as upstream a6xx_cp_init (mask 0x2f, 3, 0x20000000, 0, 0, 0, 0, 0): the downstream mask also
//   asks for the power-up register list (needs a kgsl buffer), left out. After it, one CP_MEM_WRITE of a magic word
//   to the scratch page proves CP memory writes through the GPU SMMU.
// GPU SMMU (msm-arm-smmu-bengal.dtsi kgsl_smmu 0x59a0000, qsmmu-v500, qcom,skip-init: the boot chain owns the global
// setup; kgsl-3d0 context bank "gfx3d_user" iommus <&kgsl_smmu 0 1>): read by gm2_smmu_read; M3 (gm2_smmu_setup,
// below) writes one stream-mapping entry and, if needed, one context bank, never the global configuration. The physical addresses handed to
// the CP are usable only when stream 0 is not translated: SMMU client port disabled, unmatched stream with bypass,
// S2CR bypass, or a context bank with SCTLR.M clear. Anything else stops before the CP start (level 2) or is only
// reported (level 3, forced).
#ifndef GUAMP_GPU_M2_H
#define GUAMP_GPU_M2_H
#include <stdint.h>
#include <stddef.h>

enum gm2_block : unsigned { GM2_GPU = 0, GM2_SMMU = 1 };
#define GM2_SMMU_PHYS 0x059a0000ULL
#define GM2_SMMU_SIZE 0x10000ULL

// --- a6xx_reg.h, dword offsets -------------------------------------------------------------------------------------
#define GM2_RBBM_GBIF_CLIENT_QOS_CNTL 0x00011
#define GM2_RBBM_GBIF_HALT 0x00016
#define GM2_RBBM_SW_RESET_CMD 0x00043
#define GM2_RBBM_INT_0_STATUS 0x201
#define GM2_RBBM_STATUS 0x210
#define GM2_RBBM_STATUS3 0x213
#define GM2_CP_RB_BASE 0x800
#define GM2_CP_RB_BASE_HI 0x801
#define GM2_CP_RB_CNTL 0x802
#define GM2_CP_RB_RPTR_ADDR_LO 0x804
#define GM2_CP_RB_RPTR_ADDR_HI 0x805
#define GM2_CP_RB_RPTR 0x806
#define GM2_CP_RB_WPTR 0x807
#define GM2_CP_SQE_CNTL 0x808
#define GM2_CP_HW_FAULT 0x821
#define GM2_CP_INTERRUPT_STATUS 0x823
#define GM2_CP_STATUS_1 0x825
#define GM2_CP_SQE_INSTR_BASE_LO 0x830
#define GM2_CP_SQE_INSTR_BASE_HI 0x831
#define GM2_CP_ADDR_MODE_CNTL 0x842
#define GM2_CP_PROTECT_CNTL 0x84F
#define GM2_CP_ROQ_THRESHOLDS_1 0x8C1
#define GM2_CP_ROQ_THRESHOLDS_2 0x8C2
#define GM2_CP_MEM_POOL_SIZE 0x8C3
#define GM2_CP_MEM_POOL_DBG_ADDR 0x90E
#define GM2_CP_AHB_CNTL 0x98D
#define GM2_UCHE_WRITE_RANGE_MAX_LO 0xE05
#define GM2_UCHE_WRITE_RANGE_MAX_HI 0xE06
#define GM2_UCHE_WRITE_THRU_BASE_LO 0xE07
#define GM2_UCHE_WRITE_THRU_BASE_HI 0xE08
#define GM2_UCHE_TRAP_BASE_LO 0xE09
#define GM2_UCHE_TRAP_BASE_HI 0xE0A
#define GM2_UCHE_CACHE_WAYS 0xE17
#define GM2_UCHE_FILTER_CNTL 0xE18
#define GM2_UCHE_CLIENT_PF 0xE19
#define GM2_GBIF_QSB_SIDE0 0x3c03
#define GM2_GBIF_HALT 0x3c45
#define GM2_PC_DBG_ECO_CNTL 0x9E00
#define GM2_RBBM_SECVID_TRUST_CNTL 0xF400

// --- adreno_pm4types.h ---------------------------------------------------------------------------------------------
#define GM2_CP_ME_INIT 0x48
#define GM2_CP_MEM_WRITE 0x3d
#define GM2_MAGIC 0x600dcafeu

// --- ARM SMMU v2 / MMU-500 (byte offsets) -------------------------------------------------------------------------
#define GM2_SMMU_SCR0 0x0
#define GM2_SMMU_IDR0 0x20
#define GM2_SMMU_IDR1 0x24
#define GM2_SMMU_GFSR 0x48
#define GM2_SMMU_SMR(n) (0x800u + 4u * (n))
#define GM2_SMMU_S2CR(n) (0xc00u + 4u * (n))
#define GM2_SMMU_CBAR(n) (4u * (n))              // in GR1 (one page after GR0)
#define GM2_CB_SCTLR 0x0
#define GM2_CB_TTBR0_LO 0x20
#define GM2_CB_TTBR0_HI 0x24
#define GM2_CB_FSR 0x58
#define GM2_CB_FAR_LO 0x60
#define GM2_CB_FAR_HI 0x64

#define GM2_RESET_US 100                         // after RBBM_SW_RESET_CMD 0
#define GM2_POLL_STEP_US 100
#define GM2_POLL_STEPS 2000                      // 200 ms for 13 dwords at XO 19.2 MHz (kgsl spins up to 2 s)

struct gm2_io {
  void *ctx;
  uint32_t (*rd)(void *ctx, unsigned block, uint32_t byteOff);
  void (*wr)(void *ctx, unsigned block, uint32_t byteOff, uint32_t v);   // GPU; SMMU only from gm2_smmu_setup (M3)
  void (*udelay)(void *ctx, unsigned us);
  void (*flush)(void *ctx, const void *p, size_t bytes);                // clean + invalidate to the point of coherency
};

// --- PM4 type 7 packets (adreno_pm4types.h cp_type7_packet) --------------------------------------------------------
static inline uint32_t gm2_parity(uint32_t v) {
  return (0x9669u >> (0xfu & (v ^ (v >> 4) ^ (v >> 8) ^ (v >> 12) ^ (v >> 16) ^ (v >> 20) ^ (v >> 24) ^ (v >> 28)))) & 1u;
}
static inline uint32_t gm2_pkt7(uint32_t op, uint32_t cnt) {
  return 0x70000000u | (cnt & 0x3fffu) | (gm2_parity(cnt) << 15) | ((op & 0x7fu) << 16) | (gm2_parity(op) << 23);
}
static inline uint32_t gm2_log2(uint32_t v) {
  uint32_t r = 0;
  while (v > 1) { v >>= 1; r = r + 1; }
  return r;
}

// --- Android dynamic partitions (system/core/fs_mgr/liblp metadata_format.h) ---------------------------------------
// super: geometry at byte 4096 (and its backup at 8192), metadata slots from byte 12288, metadata_max_size each.
#define GM2_LP_GEOMETRY_OFFSET 4096u
#define GM2_LP_METADATA_OFFSET 12288u
#define GM2_LP_GEOMETRY_MAGIC 0x616c4467u
#define GM2_LP_HEADER_MAGIC 0x414c5030u
#define GM2_LP_MAX_EXTENTS 8
struct gm2_extent { uint64_t sectors, physical; };   // 512-byte sectors, physical relative to the start of super

static inline uint32_t gm2_le32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | static_cast<uint32_t>(p[1]) << 8 | static_cast<uint32_t>(p[2]) << 16 |
         static_cast<uint32_t>(p[3]) << 24;
}
static inline uint64_t gm2_le64(const uint8_t *p) { return gm2_le32(p) | static_cast<uint64_t>(gm2_le32(p + 4)) << 32; }

// LpMetadataGeometry: magic, struct_size, checksum[32], metadata_max_size, metadata_slot_count, logical_block_size
static inline bool gm2_lp_geometry(const uint8_t *g, uint32_t *maxSize, uint32_t *slots) {
  if (gm2_le32(g) != GM2_LP_GEOMETRY_MAGIC) return false;
  *maxSize = gm2_le32(g + 40);
  *slots = gm2_le32(g + 44);
  return *maxSize >= 128 && *maxSize <= 0x20000 && !(*maxSize % 512) && *slots >= 1 && *slots <= 4;
}

enum : int { GM2_LP_OK = 0, GM2_LP_FORMAT = -1, GM2_LP_NOT_FOUND = -2, GM2_LP_UNSUPPORTED = -3 };
static inline const char *gm2_lp_err(int e) {
  return e == GM2_LP_OK ? "ok" : e == GM2_LP_FORMAT ? "bad format" : e == GM2_LP_NOT_FOUND ? "not found" : "unsupported";
}
// LpMetadataHeader (magic, major, minor, header_size, checksum, tables_size, checksum, then the table descriptors
// {offset, num_entries, entry_size} of partitions at 80, extents at 92), tables right after header_size bytes.
// LpMetadataPartition: name[36], attributes, first_extent_index, num_extents, group_index (52 bytes).
// LpMetadataExtent (packed): num_sectors u64, target_type u32 (0 linear, 1 zero), target_data u64, target_source u32.
static inline int gm2_lp_find(const uint8_t *m, uint32_t bytes, const char *name, gm2_extent *out, unsigned *count,
                              uint16_t *major, uint16_t *minor) {
  *count = 0;
  if (bytes < 128 || gm2_le32(m) != GM2_LP_HEADER_MAGIC) return GM2_LP_FORMAT;
  *major = static_cast<uint16_t>(m[4] | m[5] << 8);
  *minor = static_cast<uint16_t>(m[6] | m[7] << 8);
  const uint32_t hdr = gm2_le32(m + 8), tables = gm2_le32(m + 44);
  if (*major != 10 || hdr < 128 || hdr > bytes || tables > bytes - hdr) return GM2_LP_FORMAT;
  const uint8_t *t = m + hdr;
  const uint32_t pOff = gm2_le32(m + 80), pNum = gm2_le32(m + 84), pSize = gm2_le32(m + 88);
  const uint32_t eOff = gm2_le32(m + 92), eNum = gm2_le32(m + 96), eSize = gm2_le32(m + 100);
  if (pSize < 52 || eSize < 24 || pNum > 256 || eNum > 1024) return GM2_LP_FORMAT;
  if (pOff > tables || pNum * pSize > tables - pOff || eOff > tables || eNum * eSize > tables - eOff) return GM2_LP_FORMAT;
  for (uint32_t i = 0; i < pNum; i++) {
    const uint8_t *p = t + pOff + i * pSize;
    unsigned k = 0;
    while (k < 36 && name[k] && p[k] == static_cast<uint8_t>(name[k])) k = k + 1;
    if (k >= 36 || name[k] || p[k]) continue;
    const uint32_t first = gm2_le32(p + 40), num = gm2_le32(p + 44);
    if (num > GM2_LP_MAX_EXTENTS) return GM2_LP_UNSUPPORTED;
    if (first > eNum || num > eNum - first) return GM2_LP_FORMAT;
    for (uint32_t x = 0; x < num; x++) {
      const uint8_t *e = t + eOff + (first + x) * eSize;
      if (gm2_le32(e + 8) != 0 || gm2_le32(e + 20) != 0) return GM2_LP_UNSUPPORTED;   // zero fill / second block device
      out[x].sectors = gm2_le64(e);
      out[x].physical = gm2_le64(e + 12);
    }
    *count = num;
    return num ? GM2_LP_OK : GM2_LP_FORMAT;
  }
  return GM2_LP_NOT_FOUND;
}

// Logical sector of the partition -> sector of super; *run = sectors left in that extent. False past the end.
static inline bool gm2_lp_map(const gm2_extent *e, unsigned n, uint64_t sector, uint64_t *phys, uint64_t *run) {
  for (unsigned i = 0; i < n; i++) {
    if (sector < e[i].sectors) {
      *phys = e[i].physical + sector;
      *run = e[i].sectors - sector;
      return true;
    }
    sector -= e[i].sectors;
  }
  return false;
}

// --- SMMU path of stream 0 -----------------------------------------------------------------------------------------
enum gm2_path : unsigned { GM2_PATH_UNKNOWN, GM2_PATH_DISABLED, GM2_PATH_UNMATCHED_BYPASS, GM2_PATH_UNMATCHED_FAULT,
                           GM2_PATH_BYPASS, GM2_PATH_FAULT, GM2_PATH_CB_OFF, GM2_PATH_CB_TRANSLATE };
static inline const char *gm2_path_name(unsigned p) {
  switch (p) {
  case GM2_PATH_DISABLED: return "smmu client port disabled (identity)";
  case GM2_PATH_UNMATCHED_BYPASS: return "no stream match, bypass (identity)";
  case GM2_PATH_UNMATCHED_FAULT: return "no stream match, FAULT";
  case GM2_PATH_BYPASS: return "S2CR bypass (identity)";
  case GM2_PATH_FAULT: return "S2CR FAULT";
  case GM2_PATH_CB_OFF: return "context bank with MMU off (identity)";
  case GM2_PATH_CB_TRANSLATE: return "context bank TRANSLATING";
  default: return "unknown";
  }
}
static inline bool gm2_path_identity(unsigned p) {
  return p == GM2_PATH_DISABLED || p == GM2_PATH_UNMATCHED_BYPASS || p == GM2_PATH_BYPASS || p == GM2_PATH_CB_OFF;
}

struct gm2_smmu {
  uint32_t scr0, idr0, idr1, gfsr, smrs, pageSize, cbBase;
  int smr;                                       // matching stream mapping group, -1 = none
  uint32_t smrVal, s2cr, cb, sctlr, cbar, fsr;
  uint64_t ttbr0, far;
  unsigned path;
};

// Read only. The stream-match table is walked in index order (an MMU-500 with several matches reports a conflict).
static inline void gm2_smmu_read(const gm2_io *io, uint32_t sid, gm2_smmu *s) {
  *s = {};
  s->smr = -1;
  s->scr0 = io->rd(io->ctx, GM2_SMMU, GM2_SMMU_SCR0);
  s->idr0 = io->rd(io->ctx, GM2_SMMU, GM2_SMMU_IDR0);
  s->idr1 = io->rd(io->ctx, GM2_SMMU, GM2_SMMU_IDR1);
  s->gfsr = io->rd(io->ctx, GM2_SMMU, GM2_SMMU_GFSR);
  s->smrs = s->idr0 & 0xffu;
  s->pageSize = (s->idr1 >> 31) ? 0x10000u : 0x1000u;
  s->cbBase = s->pageSize << (((s->idr1 >> 28) & 7u) + 1u);
  if (s->scr0 & 1u) { s->path = GM2_PATH_DISABLED; return; }
  for (uint32_t i = 0; i < s->smrs && i < 128; i++) {
    const uint32_t v = io->rd(io->ctx, GM2_SMMU, GM2_SMMU_SMR(i));
    if (!(v >> 31)) continue;
    const uint32_t id = v & 0x7fffu, mask = (v >> 16) & 0x7fffu;
    if (((sid ^ id) & ~mask & 0x7fffu) == 0) { s->smr = static_cast<int>(i); s->smrVal = v; break; }
  }
  if (s->smr < 0) {
    s->path = (s->scr0 >> 10) & 1u ? GM2_PATH_UNMATCHED_FAULT : GM2_PATH_UNMATCHED_BYPASS;
    return;
  }
  s->s2cr = io->rd(io->ctx, GM2_SMMU, GM2_SMMU_S2CR(static_cast<uint32_t>(s->smr)));
  const uint32_t type = (s->s2cr >> 16) & 3u;
  if (type == 1) { s->path = GM2_PATH_BYPASS; return; }
  if (type != 0) { s->path = GM2_PATH_FAULT; return; }
  s->cb = s->s2cr & 0xffu;
  const uint32_t cbOff = s->cbBase + s->cb * s->pageSize;
  if (s->cbBase >= GM2_SMMU_SIZE || cbOff + s->pageSize > GM2_SMMU_SIZE || s->pageSize + GM2_SMMU_CBAR(s->cb) >= s->cbBase) {
    s->path = GM2_PATH_UNKNOWN;
    return;
  }
  s->cbar = io->rd(io->ctx, GM2_SMMU, s->pageSize + GM2_SMMU_CBAR(s->cb));
  s->sctlr = io->rd(io->ctx, GM2_SMMU, cbOff + GM2_CB_SCTLR);
  s->ttbr0 = io->rd(io->ctx, GM2_SMMU, cbOff + GM2_CB_TTBR0_LO) |
             static_cast<uint64_t>(io->rd(io->ctx, GM2_SMMU, cbOff + GM2_CB_TTBR0_HI)) << 32;
  s->fsr = io->rd(io->ctx, GM2_SMMU, cbOff + GM2_CB_FSR);
  s->far = io->rd(io->ctx, GM2_SMMU, cbOff + GM2_CB_FAR_LO) |
           static_cast<uint64_t>(io->rd(io->ctx, GM2_SMMU, cbOff + GM2_CB_FAR_HI)) << 32;
  s->path = (s->sctlr & 1u) ? GM2_PATH_CB_TRANSLATE : GM2_PATH_CB_OFF;
}

// Fault registers only, after the CP run (global and, when one was found, the context bank of stream 0).
static inline void gm2_smmu_faults(const gm2_io *io, gm2_smmu *s) {
  s->gfsr = io->rd(io->ctx, GM2_SMMU, GM2_SMMU_GFSR);
  if (s->path != GM2_PATH_CB_OFF && s->path != GM2_PATH_CB_TRANSLATE) return;
  const uint32_t cbOff = s->cbBase + s->cb * s->pageSize;
  s->fsr = io->rd(io->ctx, GM2_SMMU, cbOff + GM2_CB_FSR);
  s->far = io->rd(io->ctx, GM2_SMMU, cbOff + GM2_CB_FAR_LO) |
           static_cast<uint64_t>(io->rd(io->ctx, GM2_SMMU, cbOff + GM2_CB_FAR_HI)) << 32;
}

// --- CP bring-up ---------------------------------------------------------------------------------------------------
struct gm2_mem {
  const uint32_t *sqe;   // microcode after the file's first dword (kgsl _load_firmware), already flushed
  uint64_t sqePhys;
  uint32_t *ring;
  uint64_t ringPhys;
  uint32_t ringDwords;   // power of two, >= 64
  volatile uint32_t *scratch;
  uint64_t scratchPhys;  // [0] RPTR shadow (unused: RB_CNTL bit 27), [4] CP_MEM_WRITE target
};

struct gm2_cp {
  uint32_t resetRead, rbCntl, wptr, rptr, waitedUs;
  uint32_t status0, status, status3, hwFault, cpInt, rbbmInt, cpStatus1, protect;
  uint32_t scratchBefore, scratchAfter;
  uint32_t secvidBefore, secvidAfter;
  bool consumed, wrote;
};

static inline void gm2_w(const gm2_io *io, uint32_t reg, uint32_t v) { io->wr(io->ctx, GM2_GPU, reg * 4u, v); }
static inline uint32_t gm2_r(const gm2_io *io, uint32_t reg) { return io->rd(io->ctx, GM2_GPU, reg * 4u); }

// a6xx_soft_reset + the A610 subset of a6xx_start + a6xx_rb_start + a6xx_send_cp_init, then one CP_MEM_WRITE and the
// unsecure-mode register write. Bounded: one 200 ms poll; never loops on hardware state otherwise.
static inline void gm2_cp_start(const gm2_io *io, const gm2_mem *m, gm2_cp *c) {
  *c = {};
  c->status0 = gm2_r(io, GM2_RBBM_STATUS);
  gm2_w(io, GM2_RBBM_SW_RESET_CMD, 1);
  c->resetRead = gm2_r(io, GM2_RBBM_SW_RESET_CMD);   // dummy read: the reset takes effect
  gm2_w(io, GM2_RBBM_SW_RESET_CMD, 0);
  io->udelay(io->ctx, GM2_RESET_US);
  gm2_w(io, GM2_GBIF_HALT, 0);
  gm2_w(io, GM2_RBBM_GBIF_HALT, 0);
  for (uint32_t i = 0; i < 4; i++) gm2_w(io, GM2_GBIF_QSB_SIDE0 + i, 0x00071620);
  gm2_w(io, GM2_RBBM_GBIF_CLIENT_QOS_CNTL, 0x3);
  gm2_w(io, GM2_UCHE_WRITE_RANGE_MAX_LO, 0xffffffc0);
  gm2_w(io, GM2_UCHE_WRITE_RANGE_MAX_HI, 0x0001ffff);
  gm2_w(io, GM2_UCHE_TRAP_BASE_LO, 0xfffff000);
  gm2_w(io, GM2_UCHE_TRAP_BASE_HI, 0x0001ffff);
  gm2_w(io, GM2_UCHE_WRITE_THRU_BASE_LO, 0xfffff000);
  gm2_w(io, GM2_UCHE_WRITE_THRU_BASE_HI, 0x0001ffff);
  gm2_w(io, GM2_UCHE_FILTER_CNTL, 0x804);
  gm2_w(io, GM2_UCHE_CACHE_WAYS, 0x4);
  gm2_w(io, GM2_CP_ROQ_THRESHOLDS_2, 0x00800060);
  gm2_w(io, GM2_CP_ROQ_THRESHOLDS_1, 0x40201b16);
  gm2_w(io, GM2_CP_MEM_POOL_SIZE, 48);
  gm2_w(io, GM2_CP_MEM_POOL_DBG_ADDR, 47);
  gm2_w(io, GM2_PC_DBG_ECO_CNTL, 0x00080000);
  gm2_w(io, GM2_CP_AHB_CNTL, 0x1);
  gm2_w(io, GM2_UCHE_CLIENT_PF, 1);
  gm2_w(io, GM2_CP_ADDR_MODE_CNTL, 0x1);
  c->protect = gm2_r(io, GM2_CP_PROTECT_CNTL);   // left as reset: no register protection in M2

  // the ring: CP_ME_INIT (8 ordinals) and CP_MEM_WRITE {addr lo, hi, magic}
  m->scratch[4] = 0;
  c->scratchBefore = m->scratch[4];
  const uint64_t target = m->scratchPhys + 16;
  uint32_t n = 0;
  uint32_t *r = m->ring;
  r[n++] = gm2_pkt7(GM2_CP_ME_INIT, 8);
  r[n++] = 0x0000002f;   // ordinal mask: max context, error detection, header dump, default reset state, workarounds
  r[n++] = 0x00000003;   // multiple hardware contexts
  r[n++] = 0x20000000;   // error detection
  r[n++] = 0x00000000;   // header dump address
  r[n++] = 0x00000000;   // header dump enable and size
  r[n++] = 0x00000000;   // no ucode workarounds
  r[n++] = 0x00000000;
  r[n++] = 0x00000000;
  r[n++] = gm2_pkt7(GM2_CP_MEM_WRITE, 3);
  r[n++] = static_cast<uint32_t>(target);
  r[n++] = static_cast<uint32_t>(target >> 32);
  r[n++] = GM2_MAGIC;
  io->flush(io->ctx, m->ring, n * 4u);
  io->flush(io->ctx, const_cast<const uint32_t *>(m->scratch), 32);

  c->rbCntl = (1u << 27) | ((gm2_log2(4) << 8) & 0x1f00u) | (gm2_log2(m->ringDwords >> 1) & 0x3fu);
  gm2_w(io, GM2_CP_RB_RPTR_ADDR_LO, static_cast<uint32_t>(m->scratchPhys));
  gm2_w(io, GM2_CP_RB_RPTR_ADDR_HI, static_cast<uint32_t>(m->scratchPhys >> 32));
  gm2_w(io, GM2_CP_RB_CNTL, c->rbCntl);
  gm2_w(io, GM2_CP_RB_BASE, static_cast<uint32_t>(m->ringPhys));
  gm2_w(io, GM2_CP_RB_BASE_HI, static_cast<uint32_t>(m->ringPhys >> 32));
  gm2_w(io, GM2_CP_SQE_INSTR_BASE_LO, static_cast<uint32_t>(m->sqePhys));
  gm2_w(io, GM2_CP_SQE_INSTR_BASE_HI, static_cast<uint32_t>(m->sqePhys >> 32));
  gm2_w(io, GM2_CP_SQE_CNTL, 1);   // clear SQE_HALT
  c->wptr = n;
  gm2_w(io, GM2_CP_RB_WPTR, n);

  for (uint32_t i = 0; i < GM2_POLL_STEPS; i++) {
    c->rptr = gm2_r(io, GM2_CP_RB_RPTR);
    if (c->rptr == n) { c->consumed = true; break; }
    io->udelay(io->ctx, GM2_POLL_STEP_US);
    c->waitedUs = c->waitedUs + GM2_POLL_STEP_US;
  }
  io->udelay(io->ctx, GM2_POLL_STEP_US);   // the memory write lands after the fetch
  io->flush(io->ctx, const_cast<const uint32_t *>(m->scratch), 32);
  c->scratchAfter = m->scratch[4];
  c->wrote = c->scratchAfter == GM2_MAGIC;
  c->status = gm2_r(io, GM2_RBBM_STATUS);
  c->status3 = gm2_r(io, GM2_RBBM_STATUS3);
  c->hwFault = gm2_r(io, GM2_CP_HW_FAULT);
  c->cpInt = gm2_r(io, GM2_CP_INTERRUPT_STATUS);
  c->rbbmInt = gm2_r(io, GM2_RBBM_INT_0_STATUS);
  c->cpStatus1 = gm2_r(io, GM2_CP_STATUS_1);

  // adreno_set_unsecured_mode without a zap shader: the direct write (ignored if the secure world owns it)
  c->secvidBefore = gm2_r(io, GM2_RBBM_SECVID_TRUST_CNTL);
  gm2_w(io, GM2_RBBM_SECVID_TRUST_CNTL, 0);
  c->secvidAfter = gm2_r(io, GM2_RBBM_SECVID_TRUST_CNTL);
}

// === M3: GPU SMMU stream 0, ring submission with a fence, interrupt diagnostics ====================================
// The phone's SMMU (M2 log): sCR0 00200406 (client port on, USFCFG: unmatched streams fault), 4 SMRs none valid,
// 5 context banks at 0x8000, 4 KiB pages. Linux drivers/iommu/arm-smmu (arm-smmu.h offsets, arm_smmu_write_sme: S2CR
// before SMR; arm_smmu_write_context_bank: CBA2R, CBAR, TCR2, TCR, TTBR0, MAIR0, SCTLR last; arm-smmu-qcom.c
// qcom_smmu_cfg_probe: firmware that turns an S2CR BYPASS write into FAULT gets bypass emulated by a context bank of
// type S1_TRANS_S2_BYPASS with SCTLR.M clear). Modes (guamp-gpu-smmu):
//   1 S2CR BYPASS for SMR sid 0 mask 1; if the read-back is not BYPASS it falls back to 2
//   2 S2CR -> context bank (default the last one) with SCTLR.M clear: identity, the qcom bypass quirk
//   3 the same context bank with stage 1 on: AArch64 4 KiB granule, T0SZ 32, four 1 GiB identity blocks (first GiB
//     Device, the rest Normal non-cacheable), the table in the caller's uncached memory
#define GM2_SMMU_IDR2 0x28
#define GM2_SMMU_TLBGSYNC 0x70
#define GM2_SMMU_TLBGSTATUS 0x74
#define GM2_SMMU_CBA2R(n) (0x800u + 4u * (n))     // in GR1
#define GM2_CB_TCR2 0x10
#define GM2_CB_TCR 0x30
#define GM2_CB_CONTEXTIDR 0x34
#define GM2_CB_MAIR0 0x38
#define GM2_CB_MAIR1 0x3c
#define GM2_CB_TLBIALL 0x618
#define GM2_CB_TLBSYNC 0x7f0
#define GM2_CB_TLBSTATUS 0x7f4
#define GM2_GPU_SID 0u
#define GM2_GPU_SID_MASK 1u
#define GM2_SMR_VALID (1u << 31)
#define GM2_S2CR_BYPASS (1u << 16)
#define GM2_S2CR_FAULT (2u << 16)
#define GM2_CBAR_S1_BYPASS_S2 ((1u << 16) | (3u << 8) | (0xfu << 12))   // TYPE 1, BPSHCFG NSH, MEMATTR WB (Linux)
#define GM2_SCTLR_IDENTITY_ON 0x27u                // M, TRE, AFE, CFRE (no CFIE: no SMMU context interrupt handler)
#define GM2_TCR_IDENTITY ((32u) | (1u << 23) | (2u << 30))   // T0SZ 32, walks non-cacheable, EPD1, TG1 4K
#define GM2_TCR2_IDENTITY ((7u << 15) | 1u)        // SEP upstream, PASize 36 bit
#define GM2_MAIR0_IDENTITY 0x00000044u             // attr0 Normal NC, attr1 Device-nGnRnE
#define GM2_SMMU_SYNC_STEPS 1000                   // 1 us each

enum gm2_smmu_mode : unsigned { GM2_SM_NONE = 0, GM2_SM_BYPASS = 1, GM2_SM_CB_BYPASS = 2, GM2_SM_CB_IDENTITY = 3 };
static inline const char *gm2_sm_name(unsigned m) {
  return m == GM2_SM_BYPASS ? "s2cr-bypass" : m == GM2_SM_CB_BYPASS ? "cb-bypass" : m == GM2_SM_CB_IDENTITY ? "cb-identity" : "none";
}

struct gm2_smmu_cfg {
  unsigned want, used;         // requested mode, mode in force at the end (NONE when nothing took effect)
  int smr;                     // stream-mapping group written, -1 none
  unsigned cb;                 // context bank (modes 2 and 3)
  uint32_t idr2, gfsrBefore, gfsrAfter, bypassRead;
  uint32_t s2crWant, s2crRead, smrWant, smrRead, cbarWant, cbarRead, sctlrWant, sctlrRead, tcrRead, mairRead;
  uint64_t ttbrRead;
  bool sync, ok;
  const char *why;             // first failure, null when ok
};

// Block descriptor of one 1 GiB identity block: AF, AP[2:1] 01 (read/write), AttrIndx (GiB 0 Device), block.
static inline uint64_t gm2_block_1g(uint64_t i) {
  return (i << 30) | (1u << 10) | (1u << 6) | ((i == 0 ? 1u : 0u) << 2) | 1u;
}

static inline void gm2_sw(const gm2_io *io, uint32_t off, uint32_t v) { io->wr(io->ctx, GM2_SMMU, off, v); }
static inline uint32_t gm2_sr(const gm2_io *io, uint32_t off) { return io->rd(io->ctx, GM2_SMMU, off); }
static inline bool gm2_smmu_wait_clear(const gm2_io *io, uint32_t off) {
  for (uint32_t i = 0; i < GM2_SMMU_SYNC_STEPS; i++) {
    if (!(gm2_sr(io, off) & 1u)) return true;
    io->udelay(io->ctx, 1);
  }
  return false;
}

// One context bank for stream 0 (modes 2, 3). table/tablePhys: four uint64_t, 32-byte aligned, uncached (mode 3).
static inline bool gm2_smmu_cb(const gm2_io *io, const gm2_smmu *s, gm2_smmu_cfg *c, bool translate, uint64_t *table,
                               uint64_t tablePhys) {
  const uint32_t numCb = s->idr1 & 0xffu;
  if (c->cb >= numCb) c->cb = numCb ? numCb - 1 : 0;
  const uint32_t cbOff = s->cbBase + c->cb * s->pageSize;
  if (!numCb || cbOff + s->pageSize > GM2_SMMU_SIZE) { c->why = "no context bank"; return false; }
  if (translate) {
    if (!((c->idr2 >> 12) & 1u)) { c->why = "no AArch64 4K tables (IDR2)"; return false; }
    if (!table || (tablePhys & 31u) || tablePhys + 32 > 0x100000000ULL) { c->why = "table"; return false; }
    for (uint64_t i = 0; i < 4; i++) table[i] = gm2_block_1g(i);
    io->flush(io->ctx, table, 32);
  }
  gm2_sw(io, cbOff + GM2_CB_SCTLR, 0);                                 // off while it is rewritten
  gm2_sw(io, cbOff + GM2_CB_FSR, gm2_sr(io, cbOff + GM2_CB_FSR));      // write one to clear
  gm2_sw(io, s->pageSize + GM2_SMMU_CBA2R(c->cb), 1u);                 // VA64
  c->cbarWant = GM2_CBAR_S1_BYPASS_S2;
  gm2_sw(io, s->pageSize + GM2_SMMU_CBAR(c->cb), c->cbarWant);
  c->sctlrWant = 0;
  if (translate) {
    gm2_sw(io, cbOff + GM2_CB_TCR2, GM2_TCR2_IDENTITY);
    gm2_sw(io, cbOff + GM2_CB_TCR, GM2_TCR_IDENTITY);
    gm2_sw(io, cbOff + GM2_CB_TTBR0_LO, static_cast<uint32_t>(tablePhys));
    gm2_sw(io, cbOff + GM2_CB_TTBR0_HI, static_cast<uint32_t>(tablePhys >> 32));   // ASID 0
    gm2_sw(io, cbOff + GM2_CB_MAIR0, GM2_MAIR0_IDENTITY);
    gm2_sw(io, cbOff + GM2_CB_MAIR1, 0);
    gm2_sw(io, cbOff + GM2_CB_CONTEXTIDR, 0);
    gm2_sw(io, cbOff + GM2_CB_TLBIALL, 0);
    gm2_sw(io, cbOff + GM2_CB_TLBSYNC, 0);
    if (!gm2_smmu_wait_clear(io, cbOff + GM2_CB_TLBSTATUS)) { c->why = "cb tlb sync"; return false; }
    c->sctlrWant = GM2_SCTLR_IDENTITY_ON;
    gm2_sw(io, cbOff + GM2_CB_SCTLR, c->sctlrWant);
  }
  c->cbarRead = gm2_sr(io, s->pageSize + GM2_SMMU_CBAR(c->cb));
  c->sctlrRead = gm2_sr(io, cbOff + GM2_CB_SCTLR);
  c->tcrRead = gm2_sr(io, cbOff + GM2_CB_TCR);
  c->mairRead = gm2_sr(io, cbOff + GM2_CB_MAIR0);
  c->ttbrRead = gm2_sr(io, cbOff + GM2_CB_TTBR0_LO) | static_cast<uint64_t>(gm2_sr(io, cbOff + GM2_CB_TTBR0_HI)) << 32;
  if ((c->cbarRead & 0x3ffffu) != (c->cbarWant & 0x3ffffu) || (c->sctlrRead & 1u) != (c->sctlrWant & 1u)) {
    c->why = "cb writes ignored";
    return false;
  }
  if (translate && (c->ttbrRead & 0xffffffffffffull) != tablePhys) { c->why = "ttbr0 write ignored"; return false; }
  return true;
}

// Writes S2CR then SMR of one group (arm_smmu_write_sme order) and reads both back.
static inline bool gm2_smmu_sme(const gm2_io *io, gm2_smmu_cfg *c, uint32_t s2cr) {
  const uint32_t idx = static_cast<uint32_t>(c->smr);
  c->s2crWant = s2cr;
  c->smrWant = GM2_SMR_VALID | (GM2_GPU_SID_MASK << 16) | GM2_GPU_SID;
  gm2_sw(io, GM2_SMMU_S2CR(idx), s2cr);
  gm2_sw(io, GM2_SMMU_SMR(idx), c->smrWant);
  c->s2crRead = gm2_sr(io, GM2_SMMU_S2CR(idx));
  c->smrRead = gm2_sr(io, GM2_SMMU_SMR(idx));
  return (c->smrRead & 0xffff7fffu) == (c->smrWant & 0xffff7fffu) && (c->s2crRead & 0x300ffu) == (s2cr & 0x300ffu);
}

// Makes stream 0 reach memory untranslated (or identity-translated). s: a fresh gm2_smmu_read. cb: wanted context bank
// (out of range: the last one). Never touches sCR0 or the other groups; stops at the first write that does not stick
// and then leaves the group FAULT, as the boot chain had it (unmatched + USFCFG).
static inline void gm2_smmu_setup(const gm2_io *io, const gm2_smmu *s, unsigned mode, unsigned cb, uint64_t *table,
                                  uint64_t tablePhys, gm2_smmu_cfg *c) {
  *c = {};
  c->want = mode;
  c->smr = -1;
  c->cb = cb;
  c->gfsrBefore = s->gfsr;
  if (mode == GM2_SM_NONE || mode > GM2_SM_CB_IDENTITY) { c->why = "disabled"; return; }
  if (gm2_path_identity(s->path)) { c->ok = true; return; }   // already untranslated: nothing written
  c->idr2 = gm2_sr(io, GM2_SMMU_IDR2);
  if (s->smr >= 0) c->smr = s->smr;
  for (uint32_t i = 0; c->smr < 0 && i < s->smrs && i < 128; i++)
    if (!(gm2_sr(io, GM2_SMMU_SMR(i)) & GM2_SMR_VALID)) c->smr = static_cast<int>(i);
  if (c->smr < 0) { c->why = "no free SMR"; return; }
  if (c->gfsrBefore) gm2_sw(io, GM2_SMMU_GFSR, c->gfsrBefore);   // write one to clear
  if (mode == GM2_SM_BYPASS) {
    const bool stuck = gm2_smmu_sme(io, c, GM2_S2CR_BYPASS);
    c->bypassRead = c->s2crRead;
    if (stuck) { c->used = GM2_SM_BYPASS; c->ok = true; }
    else mode = GM2_SM_CB_BYPASS;   // qcom firmware turning BYPASS into FAULT: emulate it with a context bank
  }
  if (!c->ok) {
    if (!gm2_smmu_cb(io, s, c, mode == GM2_SM_CB_IDENTITY, table, tablePhys)) {
      gm2_smmu_sme(io, c, GM2_S2CR_FAULT);
    } else if (!gm2_smmu_sme(io, c, c->cb & 0xffu)) {
      c->why = "s2cr/smr writes ignored";
      gm2_smmu_sme(io, c, GM2_S2CR_FAULT);
    } else {
      c->used = mode;
      c->ok = true;
    }
  }
  gm2_sw(io, GM2_SMMU_TLBGSYNC, 0);
  c->sync = gm2_smmu_wait_clear(io, GM2_SMMU_TLBGSTATUS);
  c->gfsrAfter = gm2_sr(io, GM2_SMMU_GFSR);
}

// Physical addresses are valid GPU addresses after the set-up: the path itself is identity, or the context bank
// translates through the identity table written by mode 3.
static inline bool gm2_identity_after(const gm2_smmu *s, const gm2_smmu_cfg *c) {
  return gm2_path_identity(s->path) || (s->path == GM2_PATH_CB_TRANSLATE && c->ok && c->used == GM2_SM_CB_IDENTITY);
}

// --- ring submission with a CACHE_FLUSH_TS fence (adreno_ringbuffer.c addcmds: CP_EVENT_WRITE, 3 + 64-bit address) -
#define GM2_CP_NOP 0x10
#define GM2_CP_EVENT_WRITE 0x46
#define GM2_CACHE_FLUSH_TS 4u
#define GM2_FENCE_IRQ (1u << 31)          // kgsl sets it on internal commands: raises RBBM INT CP_CACHE_FLUSH_TS
#define GM2_FENCE_OFF 32u                 // byte offset of the fence word in the scratch page
#define GM2_SUBMIT_TAIL 5u                // dwords of the fence packet
#define GM2_RBBM_INT_CLEAR_CMD 0x37
#define GM2_RBBM_INT_0_MASK 0x38

// The ring as the CP left it after gm2_cp_start (wptr = gm2_cp.wptr, ts 0).
struct gm2_ring {
  uint32_t *ring;
  uint64_t ringPhys;
  uint32_t dwords, wptr, ts;
  volatile uint32_t *scratch;
  uint64_t scratchPhys;
};
enum : int { GM2_SUBMIT_OK = 0, GM2_SUBMIT_TOO_BIG = -1, GM2_SUBMIT_FULL = -2, GM2_SUBMIT_TIMEOUT = -3 };
static inline const char *gm2_submit_err(int e) {
  return e == GM2_SUBMIT_OK ? "ok" : e == GM2_SUBMIT_TOO_BIG ? "too big" : e == GM2_SUBMIT_FULL ? "ring full" : "fence timeout";
}

static inline uint32_t gm2_fence(const gm2_io *io, const gm2_ring *rb) {
  io->flush(io->ctx, const_cast<const uint32_t *>(rb->scratch), 64);
  return rb->scratch[GM2_FENCE_OFF / 4];
}

// Copies n dwords of PM4 packets into the ring (a CP_NOP pads to the end when they would wrap), appends the fence
// write of the next timestamp, moves WPTR and waits up to timeoutUs for the fence. *waitedUs: time spent waiting.
// Not reentrant: one caller at a time (the owner serializes).
static inline int gm2_submit(const gm2_io *io, gm2_ring *rb, const uint32_t *pkts, uint32_t n, uint32_t timeoutUs,
                             uint32_t *waitedUs) {
  *waitedUs = 0;
  const uint32_t need = n + GM2_SUBMIT_TAIL;
  if (need + 8 > rb->dwords) return GM2_SUBMIT_TOO_BIG;
  const uint32_t pad = rb->wptr + need > rb->dwords ? rb->dwords - rb->wptr : 0;
  for (;;) {
    const uint32_t rptr = gm2_r(io, GM2_CP_RB_RPTR) % rb->dwords;
    const uint32_t used = (rb->wptr + rb->dwords - rptr) % rb->dwords;
    if (rb->dwords - used - 1 >= pad + need) break;
    if (*waitedUs >= timeoutUs) return GM2_SUBMIT_FULL;
    io->udelay(io->ctx, GM2_POLL_STEP_US);
    *waitedUs = *waitedUs + GM2_POLL_STEP_US;
  }
  if (pad) {
    rb->ring[rb->wptr] = gm2_pkt7(GM2_CP_NOP, pad - 1);
    rb->wptr = 0;
  }
  uint32_t *r = rb->ring + rb->wptr;
  for (uint32_t i = 0; i < n; i++) r[i] = pkts[i];
  const uint32_t ts = rb->ts + 1;
  const uint64_t fence = rb->scratchPhys + GM2_FENCE_OFF;
  r[n + 0] = gm2_pkt7(GM2_CP_EVENT_WRITE, 4);
  r[n + 1] = GM2_CACHE_FLUSH_TS | GM2_FENCE_IRQ;
  r[n + 2] = static_cast<uint32_t>(fence);
  r[n + 3] = static_cast<uint32_t>(fence >> 32);
  r[n + 4] = ts;
  io->flush(io->ctx, rb->ring, rb->dwords * 4u);
  rb->ts = ts;
  rb->wptr = (rb->wptr + need) % rb->dwords;
  gm2_w(io, GM2_CP_RB_WPTR, rb->wptr);
  for (;;) {
    if (static_cast<int32_t>(gm2_fence(io, rb) - ts) >= 0) return GM2_SUBMIT_OK;
    if (*waitedUs >= timeoutUs) return GM2_SUBMIT_TIMEOUT;
    io->udelay(io->ctx, GM2_POLL_STEP_US);
    *waitedUs = *waitedUs + GM2_POLL_STEP_US;
  }
}

// RBBM_INT_0 bit names (a6xx_reg.h A6XX_INT_*), for the log; no interrupt handler is registered.
static inline void gm2_int_names(uint32_t v, char *out, size_t size) {
  static const struct { unsigned bit; const char *name; } kBits[] = {
      {0, "idle"}, {1, "cp_ahb_error"}, {6, "atb_fifo_overflow"}, {7, "gpc_error"}, {8, "cp_sw"}, {9, "cp_hw_error"},
      {13, "ib2"}, {14, "ib1"}, {15, "cp_rb"}, {17, "rb_done_ts"}, {18, "wt_done_ts"}, {20, "cache_flush_ts"},
      {22, "atb_bus_overflow"}, {23, "hang_detect"}, {24, "uche_oob"}, {25, "uche_trap"}, {28, "tsb_write_error"}};
  if (!size) return;
  size_t o = 0;
  out[0] = 0;
  for (const auto &b : kBits) {
    if (!((v >> b.bit) & 1u)) continue;
    for (const char *p = o ? "," : ""; *p && o + 1 < size; p++) out[o++] = *p;
    for (const char *p = b.name; *p && o + 1 < size; p++) out[o++] = *p;
    out[o] = 0;
  }
  if (!o) {
    const char *none = "none";
    for (const char *p = none; *p && o + 1 < size; p++) out[o++] = *p;
    out[o] = 0;
  }
}
#endif
