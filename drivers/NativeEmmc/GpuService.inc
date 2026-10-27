// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Adreno 610 milestone M1, included by NativeEmmc.cpp after the other services (mapPhysical, the boot log). A
// one-shot diagnostic on its own kernel thread, opt-in with boot argument guamp-gpu-m1=1 (default off):
//   power   the Linux order (GuampGpuM1.h): gcc cfg_ahb + GPLL0 vote, CX GDSC, the kgsl_smmu clocks (gcc memnoc,
//           gcc snoc_dvm, gpu_cc_ahb, smmu vote), GX GDSC (BLK_ARES pulse, CLAMP_IO off), gx_gfx3d_clk_src on XO
//           19.2 MHz, gmu_clk_src 200 MHz from GPLL0 / 3, the kgsl clocks (cx_gmu, iref, cxo, bimc_gpu_axi, gx/cx gfx3d)
//   read    only when both GDSCs report PWR_ON and every register-path clock reports CLK_OFF clear (re-read just
//           before): the GPU block 0x5900000 is mapped and VBIF_VERSION, RBBM_STATUS, CP_ALWAYS_ON_COUNTER_LO (twice,
//           must increase) are read. An unclocked GPU read aborts the bus, so any failure stops before the mapping.
//           guamp-gpu-relaxed=1: the BRANCH_HALT_DELAY / SKIP clocks (Linux never reads their status: gpu_cc_ahb,
//           gx_gfx3d, cx_gfx3d) pass the gate with their enable bit set even when CLK_OFF reads set.
// Strict scope of M1: no SCM/SMC, no zap shader, no firmware, no command submission, no GPU register write, no
// RBBM_SW_RESET, no PLL programming. Writes stay limited to the GCC/GPUCC registers listed in kGmRegs. M2 (below,
// guamp-gpu-m2) adds the SQE firmware, GPU register writes and the CP start; still no SCM/SMC and no zap shader.
// Other boot arguments: guamp-gpu-delay=<s> (start later, so the USB mirror sees the lines), guamp-gpu-off=1 (undo
// the power-up at the end; default: the GPU stays powered). guamp-gpu-rpm=1: before the GCC GPU branches, the RPM
// votes Linux holds for kgsl on bengal (GuampRpm.inc, kRpmGpuVotes): qdss Enab 1 (active + sleep), bimc / snoc / cnoc
// KHz (active) at guamp-gpu-rpm-khz (default RPM_KHZ_MAX: the RPM clamps to its top level, as rpmcc's probe vote for
// bimc; a low value would slow the DDR / eMMC path of the whole system), " rpm <state>" appended to RESULT.
// Log: before/after snapshots (3 lines each), one "after br" line per branch (enable bit, HW_CTL, CLK_OFF, NOC FSM,
// Linux halt type, gate/report, verdict, stuck/late), an optional stop and off line, and the single
// "GuampGpu: RESULT ..." line. Host-tested by tests/gpu_host.cpp.

#include "GuampGpuM1.h"
#include "GuampGpuM2.h"
#include "GuampGpuBlit.h"
#include "GuampGpuG2.h"
#include "GuampRpmCore.h"
#include "GuampExt4.inc"

namespace wifiprep {   // GuampWifiPrep.inc: GPT lookup by name, active slot
bool findPartition(const char *wanted, uint64_t *first, uint64_t *blocks, uint64_t *attributes);
char activeSlot();
}

// M2 (guamp-gpu-m2=1|2|3, only after M1 read the GPU with a running always-on counter), GuampGpuM2.h:
//   1  read only: GPU SMMU path of stream 0, and a630_sqe.fw from /firmware of the vendor_<slot> logical partition
//      (super, liblp metadata of the slot, ext4), header dwords logged; no GPU write
//   2  plus the CP start (soft reset, A610 a6xx_start subset, ring, SQE unhalt, CP_ME_INIT, CP_MEM_WRITE, SECVID
//      trust write) when the SMMU leaves stream 0 untranslated; otherwise it stops there
//   3  as 2 even with a translating / faulting SMMU path (forced: the CP fetches then fault inside the GPU)
// M3, levels 2 and 3, before the CP start when stream 0 is translated or faults (GuampGpuM2.h gm2_smmu_setup):
//   guamp-gpu-smmu=0 no SMMU write; 1 (default) S2CR bypass for SMR sid 0 mask 1, falling back to 2 when the
//   firmware does not keep it; 2 context bank with SCTLR.M clear (qcom bypass quirk); 3 context bank with a stage-1
//   identity table (four 1 GiB blocks, at 0xf800 of the buffer). guamp-gpu-smmu-cb=<n> context bank (default the last).
//   After a CP start that consumed the ring and wrote the magic word: one fence-only submission (CP_EVENT_WRITE
//   CACHE_FLUSH_TS, timestamp at scratch + 32) and the RBBM interrupt status, read only (no handler registered).
//   gpum2::submit() then accepts PM4 packets from this thread (one caller at a time).
// Step trace (levels 2, 3): every GPU register access from the CP start on is announced first ("GuampGpu: step N reg
// R") and stored in guamp_gpu_step[] (step, register, sequence), which GuampPerf prints every 10 s from another
// thread: if an access stalls the CPU, the last step shows which one. guamp-gpu-m2-stop=N stops before step N
// (GuampGpuM2.h GM2_STEP_*: 1 status, 2-4 soft reset, 5-6 GBIF halt, 7 QoS, 8 UCHE, 9 CP config, 10 protect, 11 ring
// registers, 12 SQE base, 13 unhalt, 14 WPTR, 15 RPTR poll, 16 status reads, 17 SECVID, 20 SMMU faults, 21 fence,
// 22 interrupt reads, 30 blits). guamp-gpu-m2-reset=1: RBBM soft reset before the start (default off: kgsl's first
// start after the GDSC power-up has none).
// 64 KiB physically contiguous, uncached, below 4 GiB: SQE at 0 (up to 56 KiB), ring 4 KiB at 0xe000, scratch at
// 0xf000. Never freed once the CP was started (it keeps the ring and microcode addresses).
extern "C" {
volatile uint32_t guamp_gpu_step[3];   // step, register (dword offset), sequence: GuampPerf.inc prints it
}

namespace gpum2 {
volatile uint8_t *gGpu, *gSmmu;

void stepLog(uint32_t step, uint32_t reg) { IOLog("GuampGpu: step %u reg %x\n", step, reg); }

uint32_t rd(void *, unsigned block, uint32_t off) {
  return *reinterpret_cast<volatile uint32_t *>((block == GM2_GPU ? gGpu : gSmmu) + off);
}
void wr(void *, unsigned block, uint32_t off, uint32_t v) {   // the SMMU only from gm2_smmu_setup (levels 2, 3)
  *reinterpret_cast<volatile uint32_t *>((block == GM2_GPU ? gGpu : gSmmu) + off) = v;
  __asm__ volatile("dsb sy" ::: "memory");
}
void udelay(void *, unsigned us) { IODelay(us); }
void flush(void *, const void *p, size_t bytes) {
  uintptr_t a = reinterpret_cast<uintptr_t>(p) & ~static_cast<uintptr_t>(63);
  const uintptr_t end = reinterpret_cast<uintptr_t>(p) + bytes;
  for (; a < end; a += 64) __asm__ volatile("dc civac, %0" :: "r"(a) : "memory");
  __asm__ volatile("dsb sy" ::: "memory");
}

constexpr uint32_t kMemBytes = 0x10000, kSqeCap = 0xe000, kRingOff = 0xe000, kScratchOff = 0xf000, kTableOff = 0xf800;
constexpr uint32_t kRingDwords = 1024;

struct Vendor { uint64_t superFirst, superBlocks; gm2_extent ext[GM2_LP_MAX_EXTENTS]; unsigned n; uint64_t sectors; };

// sectors of super (relative), at most kBounceBlocks per card read
bool readSuper(const Vendor *v, uint64_t sector, uint32_t count, uint8_t *out) {
  if (!count || sector >= v->superBlocks || count > v->superBlocks - sector) return false;
  Card &card = gCard;
  while (count) {
    const uint32_t now = count > kBounceBlocks ? kBounceBlocks : count;
    IOLockLock(card.gate);
    const bool ok = card.online && card.readBlocks(v->superFirst + sector, now, out);
    IOLockUnlock(card.gate);
    if (!ok) return false;
    sector += now;
    count -= now;
    out += now * kBlock;
  }
  return true;
}
// ext4::ReadFn over the logical partition: split at extent boundaries
bool readVendor(void *ctx, uint64_t sector, uint32_t count, uint8_t *out) {
  const Vendor *v = static_cast<const Vendor *>(ctx);
  while (count) {
    uint64_t phys, run;
    if (!gm2_lp_map(v->ext, v->n, sector, &phys, &run)) return false;
    const uint32_t now = run < count ? static_cast<uint32_t>(run) : count;
    if (!readSuper(v, phys, now, out)) return false;
    sector += now;
    count -= now;
    out += now * kBlock;
  }
  return true;
}

// a630_sqe.fw into dst (after its first dword, as kgsl _load_firmware). One log line. Returns the microcode bytes.
uint32_t loadSqe(uint8_t *dst, uint32_t cap, uint32_t *hdr0, uint32_t *hdr1) {
  char arg[4] = {};
  const char slot = PE_parse_boot_argn("guamp-gpu-slot", arg, sizeof(arg)) && (arg[0] == 'a' || arg[0] == 'b')
                        ? arg[0] : wifiprep::activeSlot();
  char part[12] = {'v', 'e', 'n', 'd', 'o', 'r', '_', slot, 0};
  Vendor *v = static_cast<Vendor *>(IOMallocZero(sizeof(Vendor)));
  uint8_t *geo = static_cast<uint8_t *>(IOMallocZero(4096));
  uint8_t *meta = nullptr, *scratch = nullptr, *file = nullptr;
  ext4::Fs *fs = nullptr;
  uint32_t maxSize = 0, slots = 0, size = 0, done = 0;
  uint16_t major = 0, minor = 0;
  uint64_t attr = 0;
  const char *step = "out of memory";
  int lp = GM2_LP_FORMAT, m = ext4::kErrIo, f = ext4::kErrIo;
  ext4::Inode ino;
  if (!v || !geo) goto out;
  step = "super not found";
  if (!gCard.online || !gCard.gate || !wifiprep::findPartition("super", &v->superFirst, &v->superBlocks, &attr)) goto out;
  step = "geometry";
  if (!readSuper(v, GM2_LP_GEOMETRY_OFFSET / kBlock, 4096 / kBlock, geo) || !gm2_lp_geometry(geo, &maxSize, &slots)) goto out;
  step = "metadata";
  meta = static_cast<uint8_t *>(IOMallocZero(maxSize));
  if (!meta) goto out;
  if (!readSuper(v, (GM2_LP_METADATA_OFFSET + (slot == 'b' && slots > 1 ? maxSize : 0)) / kBlock, maxSize / kBlock, meta)) goto out;
  lp = gm2_lp_find(meta, maxSize, part, v->ext, &v->n, &major, &minor);
  if (lp) goto out;
  for (unsigned i = 0; i < v->n; i++) v->sectors += v->ext[i].sectors;
  step = "ext4";
  scratch = static_cast<uint8_t *>(IOMallocZero(ext4::kScratchBytes));
  fs = static_cast<ext4::Fs *>(IOMallocZero(sizeof(ext4::Fs)));
  if (!scratch || !fs) goto out;
  m = fs->mount(readVendor, v, v->sectors, scratch);
  if (m) goto out;
  step = "file";
  f = fs->lookupPath("/firmware/a630_sqe.fw", &ino);
  if (!f && (!ino.isFile() || ino.size < 8 || ino.size > cap + 4u)) f = ext4::kErrRange;
  if (f) goto out;
  size = static_cast<uint32_t>(ino.size);
  file = static_cast<uint8_t *>(IOMallocZero(size));
  if (!file) { f = ext4::kErrIo; goto out; }
  f = fs->readFile(ino, 0, file, size);
  if (f) goto out;
  *hdr0 = gm2_le32(file);
  *hdr1 = gm2_le32(file + 4);
  memcpy(dst, file + 4, size - 4);
  done = size - 4;
  step = "read";
out:
  IOLog("GuampGpu: M2 sqe %s: super LBA %llu (%llu sectors), lp geometry max %u slots %u, metadata v%u.%u %s, %u extents "
        "%llu sectors, ext4 %s, /firmware/a630_sqe.fw %u bytes %s, header %08x %08x: %s\n", part,
        static_cast<unsigned long long>(v ? v->superFirst : 0), static_cast<unsigned long long>(v ? v->superBlocks : 0),
        maxSize, slots, major, minor, gm2_lp_err(lp), v ? v->n : 0, static_cast<unsigned long long>(v ? v->sectors : 0),
        ext4::errorName(m), size, ext4::errorName(f), *hdr0, *hdr1, step);
  if (file) IOFree(file, size);
  if (fs) IOFree(fs, sizeof(ext4::Fs));
  if (scratch) IOFree(scratch, ext4::kScratchBytes);
  if (meta) IOFree(meta, maxSize);
  if (geo) IOFree(geo, 4096);
  if (v) IOFree(v, sizeof(Vendor));
  return done;
}

gm2_ring gRing;
gm2_mem gMem;   // the CP buffers of run(), kept for gpug2 recovery (gm2_cp_start again)
bool gReady;   // the CP took the fence-only submission: submit() may be used

// PM4 packets from the guamp-gpu-m1 thread after run(); returns the gm2_submit result codes.
int submit(const uint32_t *pkts, uint32_t n, uint32_t timeoutUs, uint32_t *waitedUs) {
  *waitedUs = 0;
  if (!gReady) return GM2_SUBMIT_FULL;
  const gm2_io io = {nullptr, rd, wr, udelay, flush};
  return gm2_submit(&io, &gRing, pkts, n, timeoutUs, waitedUs);
}

void run(volatile uint8_t *gpu, unsigned level, unsigned smmuMode, unsigned smmuCb) {
  gGpu = gpu;
  unsigned stopAt = 0, softReset = 0;
  PE_parse_boot_argn("guamp-gpu-m2-stop", &stopAt, sizeof(stopAt));
  PE_parse_boot_argn("guamp-gpu-m2-reset", &softReset, sizeof(softReset));
  gm2_hook() = {guamp_gpu_step, level >= 2 ? stepLog : nullptr, stopAt};
  gSmmu = mapPhysical(GM2_SMMU_PHYS, GM2_SMMU_SIZE);
  if (!gGpu || !gSmmu) {
    IOLog("GuampGpu: M2 RESULT smmu - sqe - cp - stop map failed\n");
    return;
  }
  const gm2_io io = {nullptr, rd, wr, udelay, flush};
  gm2_smmu s;
  gm2_smmu_read(&io, 0, &s);
  IOLog("GuampGpu: M2 smmu scr0 %08x idr0 %08x idr1 %08x gfsr %08x, %u SMRs, page %x cb base %x: sid 0 smr %d (%08x) "
        "s2cr %08x cb %u cbar %08x sctlr %08x ttbr0 %llx fsr %08x far %llx: %s\n", s.scr0, s.idr0, s.idr1, s.gfsr,
        s.smrs, s.pageSize, s.cbBase, s.smr, s.smrVal, s.s2cr, s.cb, s.cbar, s.sctlr,
        static_cast<unsigned long long>(s.ttbr0), s.fsr, static_cast<unsigned long long>(s.far), gm2_path_name(s.path));
  bool identity = gm2_path_identity(s.path);

  IOBufferMemoryDescriptor *buf = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
      kernel_task, static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous) |
                       static_cast<IOOptionBits>(kIOMapInhibitCache), kMemBytes, 0xFFFFF000ULL);
  uint8_t *virt = buf ? static_cast<uint8_t *>(buf->getBytesNoCopy()) : nullptr;
  const uint64_t phys = buf ? buf->getPhysicalSegment(0, nullptr, kIOMemoryMapperNone) : 0;
  if (!virt || !phys || phys + kMemBytes > 0x100000000ULL) {
    IOLog("GuampGpu: M2 RESULT smmu %s sqe - cp - stop dma buffer (phys %llx)\n", identity ? "identity" : "translated",
          static_cast<unsigned long long>(phys));
    if (buf) buf->release();
    return;
  }
  memset(virt, 0, kMemBytes);
  uint32_t hdr0 = 0, hdr1 = 0;
  const uint32_t sqeBytes = loadSqe(virt, kSqeCap, &hdr0, &hdr1);
  flush(nullptr, virt, kMemBytes);
  gm2_smmu_cfg sc = {};
  if (sqeBytes && level >= 2 && !identity) {
    gm2_smmu_setup(&io, &s, smmuMode, smmuCb, reinterpret_cast<uint64_t *>(virt + kTableOff), phys + kTableOff, &sc);
    gm2_smmu_read(&io, 0, &s);
    identity = gm2_identity_after(&s, &sc);
    IOLog("GuampGpu: M3 smmu %s -> %s%s%s: smr %d %08x/%08x s2cr %08x/%08x (bypass try %08x) cb %u cbar %08x/%08x "
          "sctlr %08x/%08x tcr %08x mair %08x ttbr0 %llx, idr2 %08x gfsr %08x->%08x sync %s: path %s\n",
          gm2_sm_name(sc.want), gm2_sm_name(sc.used), sc.why ? ", stop: " : "", sc.why ? sc.why : "", sc.smr, sc.smrWant,
          sc.smrRead, sc.s2crWant, sc.s2crRead, sc.bypassRead, sc.cb, sc.cbarWant, sc.cbarRead, sc.sctlrWant, sc.sctlrRead,
          sc.tcrRead, sc.mairRead, static_cast<unsigned long long>(sc.ttbrRead), sc.idr2, sc.gfsrBefore, sc.gfsrAfter,
          sc.sync ? "ok" : "TIMEOUT", gm2_path_name(s.path));
  }
  const char *stop = !sqeBytes ? "sqe" : level < 2 ? nullptr : !identity && level != 3 ? "smmu" : nullptr;
  if (level < 2 || stop) {
    IOLog("GuampGpu: M2 RESULT smmu %s sqe %s (%u bytes, header %08x %08x) cp - %s%s\n", identity ? "identity" : "translated",
          sqeBytes ? "ok" : "missing", sqeBytes, hdr0, hdr1, stop ? "stop " : "read only (guamp-gpu-m2=1)", stop ? stop : "");
    buf->release();
    return;
  }
  const gm2_mem mem = {reinterpret_cast<const uint32_t *>(virt), phys, reinterpret_cast<uint32_t *>(virt + kRingOff),
                       phys + kRingOff, kRingDwords, reinterpret_cast<volatile uint32_t *>(virt + kScratchOff),
                       phys + kScratchOff};
  IOLog("GuampGpu: M2 cp start: sqe %llx (%u bytes) ring %llx (%u dwords) scratch %llx%s\n",
        static_cast<unsigned long long>(phys), sqeBytes, static_cast<unsigned long long>(mem.ringPhys), kRingDwords,
        static_cast<unsigned long long>(mem.scratchPhys), identity ? "" : " (forced, guamp-gpu-m2=3)");
  gm2_cp c;
  gm2_cp_start_ex(&io, &mem, &c, softReset == 1);
  if (c.stoppedAt) {
    IOLog("GuampGpu: M2 RESULT smmu %s sqe ok cp stopped before step %u (guamp-gpu-m2-stop=%u)%s\n",
          identity ? "identity" : "translated", c.stoppedAt, stopAt, c.softReset ? " soft reset" : "");
    return;
  }
  if (!gm2_step(GM2_STEP_SMMU_FAULTS, 0)) return;
  gm2_smmu_faults(&io, &s);
  IOLog("GuampGpu: M2 cp reset read %08x, protect %08x, rb_cntl %08x, wptr %u rptr %u after %u us, rbbm status %08x->%08x "
        "status3 %08x int %08x, cp hw_fault %08x int %08x status_1 %08x, scratch %08x->%08x, smmu gfsr %08x cb fsr %08x "
        "far %llx\n", c.resetRead, c.protect, c.rbCntl, c.wptr, c.rptr, c.waitedUs, c.status0, c.status, c.status3,
        c.rbbmInt, c.hwFault, c.cpInt, c.cpStatus1, c.scratchBefore, c.scratchAfter, s.gfsr, s.fsr,
        static_cast<unsigned long long>(s.far));
  IOLog("GuampGpu: M2 RESULT smmu %s sqe ok (%u bytes, header %08x %08x) cp %s me_init %s mem_write %s secvid %08x->%08x%s\n",
        identity ? "identity" : "translated", sqeBytes, hdr0, hdr1, c.rptr == 0 ? "no fetch" : "fetching",
        c.consumed ? "consumed" : "stuck", c.wrote ? "ok" : "none", c.secvidBefore, c.secvidAfter,
        c.secvidAfter == 0 ? " (unsecured)" : " (write ignored)");
  // buf stays allocated: the CP keeps the ring and microcode addresses
  if (!c.consumed || !c.wrote) return;
  gRing = {mem.ring, mem.ringPhys, kRingDwords, c.wptr, 0, mem.scratch, mem.scratchPhys};
  gMem = mem;
  uint32_t waited = 0;
  if (!gm2_step(GM2_STEP_FENCE, GM2_CP_RB_WPTR)) return;
  const int e = gm2_submit(&io, &gRing, nullptr, 0, 200000, &waited);
  gReady = e == GM2_SUBMIT_OK;
  if (!gm2_step(GM2_STEP_INT_READS, GM2_RBBM_INT_0_STATUS)) return;
  gm2_smmu_faults(&io, &s);
  const uint32_t intStatus = gm2_r(&io, GM2_RBBM_INT_0_STATUS), intMask = gm2_r(&io, GM2_RBBM_INT_0_MASK);
  char names[192];
  gm2_int_names(intStatus, names, sizeof(names));
  IOLog("GuampGpu: M3 fence ts %u %s after %u us (fence word %08x), rptr %u wptr %u, rbbm int %08x (%s) mask %08x, "
        "cp hw_fault %08x int %08x, smmu gfsr %08x cb fsr %08x far %llx\n", gRing.ts, gm2_submit_err(e), waited,
        gm2_fence(&io, &gRing), gm2_r(&io, GM2_CP_RB_RPTR), gRing.wptr, intStatus, names, intMask,
        gm2_r(&io, GM2_CP_HW_FAULT), gm2_r(&io, GM2_CP_INTERRUPT_STATUS), s.gfsr, s.fsr,
        static_cast<unsigned long long>(s.far));
  IOLog("GuampGpu: M3 RESULT smmu %s fence %s submit %s\n", gm2_sm_name(sc.used), gm2_submit_err(e),
        gReady ? "ready" : "off");
  // guamp-gpu-blit=N: N 2D solid fills (GuampGpuBlit.h) of a band of the GOP surface, 500 ms apart with changing
  // colours, so the GPU's writes stay visible over a still screen (iOS redraws the surface on its next frame).
  unsigned blits = 0;
  PE_parse_boot_argn("guamp-gpu-blit", &blits, sizeof(blits));
  if (!gReady || !blits || !gm2_step(GM2_STEP_BLIT, 0)) return;
  if (blits > 40) blits = 40;
  static const uint32_t kColors[4] = {0xffff0000u, 0xff00ff00u, 0xff0000ffu, 0xffff00ffu};
  uint32_t pkts[GMB_CLEAR_MAX_DWORDS];
  for (unsigned i = 0; i < blits; ++i) {
    gmb_clear c = {};
    c.dst = 0x5C000000ULL;   // kFbPhys, the GOP surface (NativeEmmc.cpp)
    c.pitch = 2880;
    c.surfW = 720;
    c.surfH = 1600;
    c.x = 60;
    c.y = 600;
    c.w = 600;
    c.h = 240;
    c.argb = kColors[i & 3];
    c.bgra = true;
    c.smmuIdentity = true;   // gReady implies the identity path at guamp-gpu-m2=2
    c.ts = gRing.scratchPhys + 0x100;
    c.fence = gRing.scratchPhys + 0x120;
    c.fenceValue = 0xb1170000u | i;
    const uint32_t n = gmb_build_clear(pkts, GMB_CLEAR_MAX_DWORDS, &c);
    if (!n) {
      IOLog("GuampGpu: blit %u not built: %s\n", i, gmb_check(&c));
      return;
    }
    uint32_t w = 0;
    const int r = gm2_submit(&io, &gRing, pkts, n, 200000, &w);
    const uint32_t fenceWord = gRing.scratch[0x120 / 4];
    gm2_smmu_faults(&io, &s);
    if (i < 3 || r != GM2_SUBMIT_OK)
      IOLog("GuampGpu: blit %u colour %08x %u dwords: %s after %u us, fence %08x (want %08x), rbbm status %08x "
            "cp hw_fault %08x, smmu gfsr %08x cb fsr %08x\n", i, c.argb, n, gm2_submit_err(r), w, fenceWord,
            c.fenceValue, gm2_r(&io, GM2_RBBM_STATUS), gm2_r(&io, GM2_CP_HW_FAULT), s.gfsr, s.fsr);
    if (r != GM2_SUBMIT_OK) return;
    IOSleep(500);
  }
  IOLog("GuampGpu: blit RESULT %u fills done\n", blits);
}
}  // namespace gpum2

namespace rpm {   // GuampRpm.inc
int set(uint32_t type, uint32_t id, uint32_t key, uint32_t value, unsigned which, unsigned timeoutMs);
}

namespace gpug2 {   // below gpum1
void run(volatile uint8_t *gpucc, unsigned level);
}

namespace gpum1 {
volatile uint8_t *gBase[3];

uint32_t rd(void *, unsigned block, uint32_t off) {
  return *reinterpret_cast<volatile uint32_t *>(gBase[block] + off);
}
void wr(void *, unsigned block, uint32_t off, uint32_t v) {
  if (block == GM_GPU) return;   // never written in M1
  *reinterpret_cast<volatile uint32_t *>(gBase[block] + off) = v;
  __asm__ volatile("dsb sy" ::: "memory");
}
void udelay(void *, unsigned us) { IODelay(us); }
bool mapGpu(void *) {
  gBase[GM_GPU] = mapPhysical(GM_GPU_PHYS, GM_GPU_SIZE);
  return gBase[GM_GPU] != nullptr;
}

void logSnap(const char *when, const uint32_t *v, bool gpucc) {
  IOLog("GuampGpu: %s gcc vote %08x pll0 %08x cfg_ahb %08x memnoc %08x snoc_dvm %08x iref %08x axi %08x throttle %08x "
        "qmip %08x vote2 %08x\n", when, v[GMR_VOTE], v[GMR_PLL0], v[GMR_CFG_AHB], v[GMR_MEMNOC], v[GMR_SNOC_DVM], v[GMR_IREF],
        v[GMR_AXI], v[GMR_THROTTLE], v[GMR_QMIP], v[GMR_VOTE2]);
  if (!gpucc) {
    IOLog("GuampGpu: %s gpucc not read (gcc cfg_ahb %08x not on)\n", when, v[GMR_CFG_AHB]);
    return;
  }
  IOLog("GuampGpu: %s cx gdscr %08x hwctrl %08x ahb %08x cxo %08x gmu_rcg %08x/%08x cx_gmu %08x smmu %08x\n", when,
        v[GMR_CX_GDSCR], v[GMR_CX_HWCTRL], v[GMR_AHB], v[GMR_CXO], v[GMR_GMU_CMD], v[GMR_GMU_CFG], v[GMR_CX_GMU], v[GMR_SMMU]);
  IOLog("GuampGpu: %s gx bcr %08x clamp %08x gdscr %08x gfx_rcg %08x/%08x gx_gfx3d %08x cx_gfx3d %08x\n", when,
        v[GMR_GX_BCR], v[GMR_GX_CLAMP], v[GMR_GX_GDSCR], v[GMR_GFX_CMD], v[GMR_GFX_CFG], v[GMR_GX_GFX3D], v[GMR_CX_GFX3D]);
}

// One line per branch of kGmBranches (GPUCC ones only when GPUCC was readable).
void logBranches(const char *when, const uint32_t *v, const gm_state &st) {
  for (const gm_branch &b : kGmBranches) {
    if (kGmRegs[b.reg].block == GM_GPUCC && !st.gpuccOk) continue;
    const uint32_t x = v[b.reg], bit = 1u << b.reg;
    IOLog("GuampGpu: %s br %s %08x en %u hwctl %u clk_off %u fsm %u halt %s %s: %s%s%s\n", when, kGmRegs[b.reg].name, x,
          x & GM_CBCR_EN, (x & GM_CBCR_HWCG) >> 1, x >> 31, (x & GM_CBCR_FSM_MASK) >> 28, gm_halt_name(b.halt),
          b.gate ? "gate" : "report", gm_bstate_name(gm_branch_state(x, b)),
          st.late & bit ? " (late: on at the re-poll)" : st.stuck & bit ? " (stuck at enable)" : "",
          st.enabled & bit ? " (enabled here)" : "");
  }
}

gm_state gSt;

// guamp-gpu-rpm=1, before any GCC GPU branch is touched. Returns a short word for the RESULT line.
const char *rpmVotes() {
  unsigned khz = RPM_KHZ_MAX;
  PE_parse_boot_argn("guamp-gpu-rpm-khz", &khz, sizeof(khz));
  if (!khz) khz = RPM_KHZ_MAX;
  const uint32_t m0 = rd(nullptr, GM_GCC, GM_GCC_GPU_MEMNOC_GFX), s0 = rd(nullptr, GM_GCC, GM_GCC_GPU_SNOC_DVM_GFX),
                 a0 = rd(nullptr, GM_GCC, GM_GCC_BIMC_GPU_AXI);
  unsigned ok = 0;
  int first = RPM_OK;
  for (const rpm_vote &v : kRpmGpuVotes) {
    const uint32_t value = v.key == RPM_KEY_KHZ ? khz : 1u;
    int rc = rpm::set(v.type, v.id, v.key, value, RPM_SET_ACTIVE, 500);
    if (!rc && v.sleep) rc = rpm::set(v.type, v.id, v.key, value, RPM_SET_SLEEP, 500);
    if (!rc) ok = ok + 1;
    else if (first == RPM_OK) first = rc;
    if (rc == RPM_ERR_NOMAP || rc == RPM_ERR_TOC || rc == RPM_ERR_FIFO || rc == RPM_ERR_VERSION || rc == RPM_ERR_OPEN) break;
  }
  IODelay(100);
  IOLog("GuampGpu: rpm votes %u/4 (first error %s), KHz %u: gcc memnoc %08x->%08x snoc_dvm %08x->%08x axi %08x->%08x\n", ok,
        rpm_err_name(first), khz, m0, rd(nullptr, GM_GCC, GM_GCC_GPU_MEMNOC_GFX), s0, rd(nullptr, GM_GCC, GM_GCC_GPU_SNOC_DVM_GFX),
        a0, rd(nullptr, GM_GCC, GM_GCC_BIMC_GPU_AXI));
  return ok == 4 ? "ok" : ok ? "partial" : rpm_err_name(first);
}

void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-gpu-m1");
  unsigned delay = 0, off = 0, rpmOn = 0, relaxed = 0, m2 = 0, smmuMode = GM2_SM_BYPASS, smmuCb = 0xff;
  PE_parse_boot_argn("guamp-gpu-m2", &m2, sizeof(m2));
  if (m2 > 3) m2 = 0;
  PE_parse_boot_argn("guamp-gpu-smmu", &smmuMode, sizeof(smmuMode));
  PE_parse_boot_argn("guamp-gpu-smmu-cb", &smmuCb, sizeof(smmuCb));
  PE_parse_boot_argn("guamp-gpu-delay", &delay, sizeof(delay));
  PE_parse_boot_argn("guamp-gpu-relaxed", &relaxed, sizeof(relaxed));
  PE_parse_boot_argn("guamp-gpu-off", &off, sizeof(off));
  PE_parse_boot_argn("guamp-gpu-rpm", &rpmOn, sizeof(rpmOn));
  if (delay && delay <= 600) IOSleep(delay * 1000);
  gm_state &st = gSt;
  gm_init(&st);
  st.relaxed = relaxed == 1;
  gBase[GM_GCC] = mapPhysical(GM_GCC_PHYS, GM_GCC_SIZE);
  gBase[GM_GPUCC] = mapPhysical(GM_GPUCC_PHYS, GM_GPUCC_SIZE);
  if (!gBase[GM_GCC] || !gBase[GM_GPUCC]) {
    IOLog("GuampGpu: RESULT cx - gx - clocks - vbif - status - aon - stop gcc/gpucc map failed\n");
    return;
  }
  const gm_io io = {nullptr, rd, wr, udelay, mapGpu};
  const char *rpmState = rpmOn == 1 ? rpmVotes() : nullptr;
  gm_power_up(&io, &st);
  const bool ready = gm_prepare_read(&io, &st);
  // Logged before the first GPU read: if that read still aborts the bus, the kpan mirror (every second) and the panic
  // copy keep the snapshots.
  logSnap("before", st.before, st.gpuccOk);
  logSnap("after", st.after, st.gpuccOk);
  logBranches("after", st.after, st);
  if (ready) {
    IOSleep(1500);
    gm_do_reads(&io, &st);
  }
  if (m2 && st.gpuRead && st.aonOk) gpum2::run(gBase[GM_GPU], m2, smmuMode, smmuCb);
  unsigned g2 = 0;
  PE_parse_boot_argn("guamp-gpu-g2", &g2, sizeof(g2));
  if (g2 == 1 || g2 == 2) gpug2::run(gBase[GM_GPUCC], g2);
  else if (m2) IOLog("GuampGpu: M2 RESULT smmu - sqe - cp - stop M1 (GPU %s)\n", st.gpuRead ? "counter stuck" : "not read");
  if (st.stop) IOLog("GuampGpu: stopped at %s, GPU block %s\n", st.stop, st.mapped ? "mapped, not read" : "not touched");
  if (off == 1) {
    const bool down = gm_power_down(&io, &st);
    IOLog("GuampGpu: off (guamp-gpu-off=1) gx %s gx_gdscr %08x clamp %08x cx_gdscr %08x hwctrl %08x vote %08x\n",
          down ? "collapsed" : "PWR_ON still set", st.off[GMR_GX_GDSCR], st.off[GMR_GX_CLAMP], st.off[GMR_CX_GDSCR],
          st.off[GMR_CX_HWCTRL], st.off[GMR_VOTE]);
  }
  if (st.gpuRead)
    IOLog("GuampGpu: RESULT cx %s gx %s clocks %s gmu %s gfx %s vbif %08x status %08x aon %08x->%08x %s%s%s%s%s\n", st.cx, st.gx,
          st.clocks, st.gmu, st.gfx, st.vbif, st.status, st.aon0, st.aon1, st.aonOk ? "ok" : "stuck",
          off == 1 ? " (powered off)" : " (left powered)", rpmState ? " rpm " : "", rpmState ? rpmState : "",
          st.relaxed ? " relaxed" : "");
  else
    IOLog("GuampGpu: RESULT cx %s gx %s clocks %s gmu %s gfx %s vbif - status - aon - stop %s%s%s%s\n", st.cx, st.gx,
          st.clocks, st.gmu, st.gfx, st.stop ? st.stop : "?", rpmState ? " rpm " : "", rpmState ? rpmState : "",
          st.relaxed ? " relaxed" : "");
}

void start() {
  unsigned on = 0;
  if (!PE_parse_boot_argn("guamp-gpu-m1", &on, sizeof(on)) || on != 1) return;
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}
}  // namespace gpum1

// G2 (guamp-gpu-g2=1|2, GuampGpuG2.h), after M3 left the CP ready (gpum2::gReady), on the same thread:
//   1. GPU clock on XO measured (CP countable 0 against the always-on counter) and one fence timed
//   2. the GPU interrupt (INTID 209, level) through armcpu_spi_attach: RBBM_INT_0_MASK = A6XX_INT_MASK, then a fence
//      whose CACHE_FLUSH_TS must raise it (counters in the log; the handler acks and counts only)
//   3. RPM vote VDD_CX (smpa3 "vlvl") LOW_SVS, active set; only when acked: PLL1 at 640 MHz, gfx3d RCG to PLL1_OUT_AUX
//      / 2 = 320 MHz (the lowest OPP over XO), clock measured, fence timed. A fence timeout there puts the RCG back on
//      XO, turns PLL1 off and recovers the CP. Without the ack the GPU stays on XO.
//   4. guamp-gpu-g2=2 also runs one forced recovery (RBBM soft reset + CP restart with the M2 buffers) and a fence.
// "GuampGpu: G2 RESULT ..." summarizes. Never above 320 MHz.
extern "C" int armcpu_spi_attach(uint32_t, bool, void (*)(void *), void *, uint32_t *, uint64_t *) __attribute__((weak_import));
namespace gpug2 {
volatile uint8_t *gCc;
gg2_irq_stats gIrq;

uint32_t rd(void *, unsigned block, uint32_t off) {
  return *reinterpret_cast<volatile uint32_t *>((block == GG2_GPUCC ? gCc : gpum2::gGpu) + off);
}
void wr(void *, unsigned block, uint32_t off, uint32_t v) {
  *reinterpret_cast<volatile uint32_t *>((block == GG2_GPUCC ? gCc : gpum2::gGpu) + off) = v;
  __asm__ volatile("dsb sy" ::: "memory");
}
void udelay(void *, unsigned us) { IODelay(us); }
const gg2_io kIo = {nullptr, rd, wr, udelay};

void irq(void *) { gg2_irq_service(&kIo, &gIrq); }

// One fence-only submission; returns the gm2_submit code, the wait in *us.
int fence(uint32_t *us) {
  const gm2_io io = {nullptr, gpum2::rd, gpum2::wr, gpum2::udelay, gpum2::flush};
  return gm2_submit(&io, &gpum2::gRing, nullptr, 0, 200000, us);
}

// gm2_cp_start again on the M2 buffers (it begins with the RBBM soft reset); the ring restarts after ME_INIT.
bool recover(const char *why) {
  const gm2_io io = {nullptr, gpum2::rd, gpum2::wr, gpum2::udelay, gpum2::flush};
  gm2_cp c;
  gm2_cp_start(&io, &gpum2::gMem, &c);
  gpum2::gRing.wptr = c.wptr;
  gpum2::gReady = c.consumed && c.wrote;
  gg2_gw(&kIo, GG2_RBBM_INT_0_MASK, GG2_INT_MASK);
  IOLog("GuampGpu: G2 recovery (%s): reset read %08x, wptr %u rptr %u after %u us, me_init %s mem_write %s, rbbm status "
        "%08x cp hw_fault %08x\n", why, c.resetRead, c.wptr, c.rptr, c.waitedUs, c.consumed ? "consumed" : "stuck",
        c.wrote ? "ok" : "none", c.status, c.hwFault);
  return gpum2::gReady;
}

void run(volatile uint8_t *gpucc, unsigned level) {
  gCc = gpucc;
  if (!gpum2::gReady || !gCc || !gpum2::gGpu) {
    IOLog("GuampGpu: G2 RESULT clock - irq - recovery - stop %s\n", gpum2::gReady ? "map" : "M3 not ready");
    return;
  }
  uint32_t cyc = 0, aon = 0, us = 0;
  const uint64_t xoHz = gg2_measure_hz(&kIo, 1000, &cyc, &aon);
  int e = fence(&us);
  IOLog("GuampGpu: G2 clock on XO: %llu Hz (%u cycles / %u aon ticks), gfx rcg %08x/%08x, fence %s after %u us\n",
        static_cast<unsigned long long>(xoHz), cyc, aon, gg2_cr(&kIo, GG2_GFX_CMD), gg2_cr(&kIo, GG2_GFX_CFG),
        gm2_submit_err(e), us);

  // Interrupt
  const char *irqState = "none";
  uint32_t readback = 0;
  uint64_t router = 0;
  if (!&armcpu_spi_attach) irqState = "no attach";
  else if (armcpu_spi_attach(GG2_GPU_INTID, false, irq, nullptr, &readback, &router)) irqState = "attach failed";
  else {
    gg2_gw(&kIo, GG2_RBBM_INT_CLEAR_CMD, 0xffffffffu);
    gg2_gw(&kIo, GG2_RBBM_INT_0_MASK, GG2_INT_MASK);
    const uint32_t before = gIrq.fence;
    e = fence(&us);
    IOSleep(5);
    irqState = gIrq.fence != before ? "fence irq ok" : "no fence irq";
    char names[192];
    gm2_int_names(gIrq.last, names, sizeof(names));
    IOLog("GuampGpu: G2 irq %u: readback %08x router %llx, mask %08x, fence %s after %u us, irqs %u (fence %u, last %08x "
          "%s, spurious %u, other %u): %s\n", GG2_GPU_INTID, readback, static_cast<unsigned long long>(router),
          gg2_gr(&kIo, GG2_RBBM_INT_0_MASK), gm2_submit_err(e), us, gIrq.total, gIrq.fence, gIrq.last, names,
          gIrq.spurious, gIrq.other, irqState);
  }

  // Voltage, PLL1, 320 MHz
  const char *clock = "xo";
  uint64_t hz = xoHz;
  const int vote = rpm::set(RPM_RES_SMPA, 3, 0x6c766c76u /* "vlvl" */, 64 /* LOW_SVS */, RPM_SET_ACTIVE, 500);
  if (vote) {
    IOLog("GuampGpu: G2 VDD_CX LOW_SVS vote failed (%s): GPU stays on XO\n", rpm_err_name(vote));
    clock = "xo (no cx vote)";
  } else {
    gg2_pll p;
    const bool pll = gg2_pll1_up(&kIo, &kGg2Pll1, &p);
    IOLog("GuampGpu: G2 pll1 mode %08x->%08x l %02x->%02x alpha %08x user %08x->%08x status %08x, %s, lock %s after %u us, "
          "%llu Hz%s%s\n", p.mode0, p.modeAfter, p.l0, p.lAfter, p.alpha0, p.user0, p.userAfter, p.status,
          p.reconfigured ? "configured" : "already running", p.locked ? "ok" : "NO", p.waitedUs,
          static_cast<unsigned long long>(p.hz), p.why ? ": " : "", p.why ? p.why : "");
    if (!pll) {
      clock = "xo (pll1)";
    } else {
      gg2_rcg r;
      const bool rcg = gg2_gfx_set(&kIo, GG2_SRC_PLL1_AUX, 4, &r);
      hz = gg2_measure_hz(&kIo, 1000, &cyc, &aon);
      e = fence(&us);
      IOLog("GuampGpu: G2 gfx rcg %08x/%08x -> %08x/%08x after %u us (%s), expected %llu Hz, measured %llu Hz (%u cycles / "
            "%u aon), fence %s after %u us\n", r.cmd0, r.cfg0, r.cmd, r.cfg, r.waitedUs, rcg ? "ok" : "FAILED",
            static_cast<unsigned long long>(gg2_rcg_hz(r.cfg, p.hz)), static_cast<unsigned long long>(hz), cyc, aon,
            gm2_submit_err(e), us);
      if (rcg && e == GM2_SUBMIT_OK) {
        clock = "320M";
      } else {
        gg2_rcg back;
        gg2_gfx_set(&kIo, GG2_SRC_XO, 0, &back);
        gg2_pll1_down(&kIo);
        clock = "xo (back)";
        if (e != GM2_SUBMIT_OK) recover("fence timeout at 320 MHz");
        hz = gg2_measure_hz(&kIo, 1000, &cyc, &aon);
      }
    }
  }

  // Forced recovery test
  const char *recovery = "-";
  if (level == 2 && gpum2::gReady) {
    const bool ok = recover("guamp-gpu-g2=2 test");
    e = ok ? fence(&us) : GM2_SUBMIT_FULL;
    recovery = ok && e == GM2_SUBMIT_OK ? "ok" : "FAILED";
    IOLog("GuampGpu: G2 fence after recovery %s after %u us, irqs %u fence %u\n", gm2_submit_err(e), us, gIrq.total,
          gIrq.fence);
  }
  IOLog("GuampGpu: G2 RESULT clock %s %llu Hz irq %s (%u, fence %u, hw_error %u, ahb %u, hang %u%s) recovery %s submit %s\n",
        clock, static_cast<unsigned long long>(hz), irqState, gIrq.total, gIrq.fence, gIrq.hwError, gIrq.ahbError,
        gIrq.hang, gIrq.masked ? ", MASKED: storm" : "", recovery, gpum2::gReady ? "ready" : "off");
}
}  // namespace gpug2
