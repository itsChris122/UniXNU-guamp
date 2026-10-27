// Host test of GuampGpuM2.h: PM4 type 7 headers (values seen in freedreno a6xx dumps), the liblp metadata walk on a
// synthetic super, the SMMU stream-0 decision, and the CP start against a fake register file whose CP consumes the
// ring (CP_MEM_WRITE executed) once SQE_CNTL is 1 and the WPTR is written.
#include <stdio.h>
#include <string.h>
#include <vector>
#include "../GuampGpuM2.h"

static int fails, checks;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = static_cast<uint8_t>(v >> (8 * i)); }
static void put64(uint8_t *p, uint64_t v) { put32(p, static_cast<uint32_t>(v)); put32(p + 4, static_cast<uint32_t>(v >> 32)); }

static void testPkt() {
  CHECK(gm2_pkt7(GM2_CP_ME_INIT, 8) == 0x70c80008u);
  CHECK(gm2_pkt7(GM2_CP_MEM_WRITE, 3) == 0x703d8003u);
  CHECK(gm2_log2(512) == 9 && gm2_log2(4) == 2 && gm2_log2(1) == 0);
}

static void testLp() {
  uint8_t g[4096] = {};
  put32(g, GM2_LP_GEOMETRY_MAGIC);
  put32(g + 40, 65536);
  put32(g + 44, 2);
  uint32_t maxSize = 0, slots = 0;
  CHECK(gm2_lp_geometry(g, &maxSize, &slots) && maxSize == 65536 && slots == 2);
  put32(g, 0);
  CHECK(!gm2_lp_geometry(g, &maxSize, &slots));

  std::vector<uint8_t> m(65536, 0);
  const uint32_t hdr = 256, pOff = 0, pNum = 3, eOff = pNum * 52, eNum = 4;
  put32(&m[0], GM2_LP_HEADER_MAGIC);
  m[4] = 10; m[6] = 2;
  put32(&m[8], hdr);
  put32(&m[44], eOff + eNum * 24);
  put32(&m[80], pOff); put32(&m[84], pNum); put32(&m[88], 52);
  put32(&m[92], eOff); put32(&m[96], eNum); put32(&m[100], 24);
  const char *names[3] = {"system_a", "vendor_a", "vendor_b"};
  const uint32_t first[3] = {0, 1, 3}, num[3] = {1, 2, 1};
  for (int i = 0; i < 3; i++) {
    uint8_t *p = &m[hdr + pOff + i * 52];
    memcpy(p, names[i], strlen(names[i]));
    put32(p + 40, first[i]);
    put32(p + 44, num[i]);
  }
  const uint64_t ext[4][2] = {{1000, 2048}, {500, 10000}, {300, 20000}, {800, 30000}};
  for (int i = 0; i < 4; i++) {
    uint8_t *e = &m[hdr + eOff + i * 24];
    put64(e, ext[i][0]);
    put64(e + 12, ext[i][1]);
  }
  gm2_extent out[GM2_LP_MAX_EXTENTS];
  unsigned n = 0;
  uint16_t ma = 0, mi = 0;
  CHECK(gm2_lp_find(m.data(), 65536, "vendor_a", out, &n, &ma, &mi) == GM2_LP_OK && n == 2 && ma == 10 && mi == 2);
  CHECK(out[0].sectors == 500 && out[0].physical == 10000 && out[1].sectors == 300 && out[1].physical == 20000);
  CHECK(gm2_lp_find(m.data(), 65536, "vendor", out, &n, &ma, &mi) == GM2_LP_NOT_FOUND);
  uint64_t phys = 0, run = 0;
  CHECK(gm2_lp_map(out, 2, 0, &phys, &run) && phys == 10000 && run == 500);
  CHECK(gm2_lp_map(out, 2, 499, &phys, &run) && phys == 10499 && run == 1);
  CHECK(gm2_lp_map(out, 2, 500, &phys, &run) && phys == 20000 && run == 300);
  CHECK(!gm2_lp_map(out, 2, 800, &phys, &run));
  put32(&m[hdr + eOff + 1 * 24 + 8], 1);   // zero-fill extent
  CHECK(gm2_lp_find(m.data(), 65536, "vendor_a", out, &n, &ma, &mi) == GM2_LP_UNSUPPORTED);
  put32(&m[0], 0);
  CHECK(gm2_lp_find(m.data(), 65536, "vendor_a", out, &n, &ma, &mi) == GM2_LP_FORMAT);
}

// --- fake hardware -------------------------------------------------------------------------------------------------
struct Fake {
  std::vector<uint32_t> gpu = std::vector<uint32_t>(0x90000 / 4), smmu = std::vector<uint32_t>(0x10000 / 4);
  uint32_t *ring = nullptr, *scratch = nullptr;
  uint64_t ringPhys = 0, scratchPhys = 0;
  uint32_t ringDwords = 1024;
  bool execute = true, secvidLocked = false;
  bool smmuIgnore = false, bypassBecomesFault = false;   // firmware behaviours seen on qcom SMMUs
  unsigned writes = 0, flushes = 0, smmuWrites = 0, resetWrites = 0;
};
static uint32_t frd(void *c, unsigned b, uint32_t off) {
  Fake *f = static_cast<Fake *>(c);
  return b == GM2_GPU ? f->gpu.at(off / 4) : f->smmu.at(off / 4);
}
static void fwr(void *c, unsigned b, uint32_t off, uint32_t v) {
  Fake *f = static_cast<Fake *>(c);
  if (b != GM2_GPU) {
    f->smmuWrites++;
    if (f->smmuIgnore) return;
    if (off == GM2_SMMU_GFSR) { f->smmu.at(off / 4) &= ~v; return; }   // write one to clear
    if (f->bypassBecomesFault && off >= GM2_SMMU_S2CR(0) && off < GM2_SMMU_S2CR(128) && ((v >> 16) & 3u) == 1)
      v = (v & ~(3u << 16)) | (2u << 16);
    f->smmu.at(off / 4) = v;
    return;
  }
  f->writes++;
  const uint32_t reg = off / 4;
  if (reg == GM2_RBBM_SECVID_TRUST_CNTL && f->secvidLocked) return;
  if (reg == GM2_RBBM_SW_RESET_CMD) f->resetWrites++;
  f->gpu.at(reg) = v;
  if (reg == GM2_CP_RB_WPTR && f->execute && f->gpu[GM2_CP_SQE_CNTL] == 1) {
    // walk the ring from RPTR to the new WPTR: NOP, ME_INIT, MEM_WRITE / EVENT_WRITE into the scratch page
    uint32_t i = f->gpu[GM2_CP_RB_RPTR];
    while (i != v) {
      const uint32_t h = f->ring[i], cnt = h & 0x3fff, op = (h >> 16) & 0x7f;
      if (op == GM2_CP_MEM_WRITE || op == GM2_CP_EVENT_WRITE) {
        const uint32_t k = op == GM2_CP_EVENT_WRITE ? 1 : 0;
        const uint64_t a = f->ring[i + 1 + k] | static_cast<uint64_t>(f->ring[i + 2 + k]) << 32;
        f->scratch[(a - f->scratchPhys) / 4] = f->ring[i + 3 + k];
        if (k) f->gpu[GM2_RBBM_INT_0_STATUS] |= 1u << 20;
      }
      i = (i + 1 + cnt) % f->ringDwords;
    }
    f->gpu[GM2_CP_RB_RPTR] = v;
  }
}
static void fdelay(void *, unsigned) {}
static void fflush(void *c, const void *, size_t) { static_cast<Fake *>(c)->flushes++; }

static void testSmmu() {
  Fake f;
  const gm2_io io = {&f, frd, fwr, fdelay, fflush};
  gm2_smmu s;
  f.smmu[0] = 1;   // CLIENTPD
  gm2_smmu_read(&io, 0, &s);
  CHECK(s.path == GM2_PATH_DISABLED && gm2_path_identity(s.path));
  f.smmu[0] = 1u << 10;   // USFCFG, no SMR valid
  f.smmu[GM2_SMMU_IDR0 / 4] = 16;
  f.smmu[GM2_SMMU_IDR1 / 4] = 2u << 28;   // 4 KiB pages, 8 pages of global space: CBs at 0x8000
  gm2_smmu_read(&io, 0, &s);
  CHECK(s.path == GM2_PATH_UNMATCHED_FAULT && !gm2_path_identity(s.path) && s.cbBase == 0x8000);
  f.smmu[GM2_SMMU_SMR(3) / 4] = (1u << 31) | (1u << 16) | 0;   // sid 0 mask 1
  f.smmu[GM2_SMMU_S2CR(3) / 4] = 2;                              // translate, cb 2
  f.smmu[(0x8000 + 2 * 0x1000) / 4] = 0;                         // SCTLR.M clear
  gm2_smmu_read(&io, 1, &s);
  CHECK(s.smr == 3 && s.cb == 2 && s.path == GM2_PATH_CB_OFF && gm2_path_identity(s.path));
  f.smmu[(0x8000 + 2 * 0x1000) / 4] = 1;
  f.smmu[(0x8000 + 2 * 0x1000 + GM2_CB_TTBR0_LO) / 4] = 0x1234000;
  gm2_smmu_read(&io, 0, &s);
  CHECK(s.path == GM2_PATH_CB_TRANSLATE && s.ttbr0 == 0x1234000 && !gm2_path_identity(s.path));
  f.smmu[GM2_SMMU_S2CR(3) / 4] = 1u << 16;
  gm2_smmu_read(&io, 0, &s);
  CHECK(s.path == GM2_PATH_BYPASS);
  gm2_smmu_read(&io, 2, &s);   // sid 2 does not match sid 0 / mask 1
  CHECK(s.smr == -1 && s.path == GM2_PATH_UNMATCHED_FAULT);
  CHECK(f.writes == 0 && f.smmuWrites == 0);   // read only
}

// The phone's SMMU as M2 read it: client port on, USFCFG, 4 SMRs none valid, 5 context banks at 0x8000, 4 KiB pages.
static void phoneSmmu(Fake &f, bool aarch64) {
  f.smmu[0] = 0x00200406;
  f.smmu[GM2_SMMU_IDR0 / 4] = 0x4c017e04;
  f.smmu[GM2_SMMU_IDR1 / 4] = 0x20000005;
  f.smmu[GM2_SMMU_IDR2 / 4] = aarch64 ? 0x00001055u : 0x00000055u;
}

static void testSmmuSetup() {
  std::vector<uint64_t> table(4, 0);
  const uint64_t tablePhys = 0x80000000ull + 0xf800;
  {   // mode 1, bypass kept
    Fake f;
    phoneSmmu(f, true);
    const gm2_io io = {&f, frd, fwr, fdelay, fflush};
    gm2_smmu s;
    gm2_smmu_read(&io, 0, &s);
    CHECK(s.path == GM2_PATH_UNMATCHED_FAULT && s.cbBase == 0x8000 && s.smrs == 4);
    gm2_smmu_cfg c;
    gm2_smmu_setup(&io, &s, GM2_SM_BYPASS, 0xff, table.data(), tablePhys, &c);
    CHECK(c.ok && c.used == GM2_SM_BYPASS && c.smr == 0 && !c.why && c.sync);
    CHECK(f.smmu[GM2_SMMU_SMR(0) / 4] == 0x80010000u && f.smmu[GM2_SMMU_S2CR(0) / 4] == 0x10000u);
    gm2_smmu_read(&io, 0, &s);
    CHECK(s.path == GM2_PATH_BYPASS && gm2_identity_after(&s, &c));
    gm2_smmu_read(&io, 1, &s);   // mask 1: sid 1 is the same group
    CHECK(s.path == GM2_PATH_BYPASS);
    CHECK(f.smmu[0] == 0x00200406u && f.writes == 0);   // sCR0 untouched, no GPU write
  }
  {   // mode 1 on firmware that writes BYPASS as FAULT: falls back to the context-bank bypass
    Fake f;
    phoneSmmu(f, true);
    f.bypassBecomesFault = true;
    f.smmu[GM2_SMMU_GFSR / 4] = 0x2;
    const gm2_io io = {&f, frd, fwr, fdelay, fflush};
    gm2_smmu s;
    gm2_smmu_read(&io, 0, &s);
    gm2_smmu_cfg c;
    gm2_smmu_setup(&io, &s, GM2_SM_BYPASS, 0xff, table.data(), tablePhys, &c);
    CHECK(c.ok && c.used == GM2_SM_CB_BYPASS && c.cb == 4 && ((c.bypassRead >> 16) & 3u) == 2);
    CHECK(c.gfsrBefore == 2 && c.gfsrAfter == 0);
    CHECK(f.smmu[(0x1000 + GM2_SMMU_CBAR(4)) / 4] == GM2_CBAR_S1_BYPASS_S2);
    CHECK(f.smmu[(0x1000 + GM2_SMMU_CBA2R(4)) / 4] == 1u && f.smmu[(0x8000 + 4 * 0x1000) / 4] == 0);
    CHECK(f.smmu[GM2_SMMU_S2CR(0) / 4] == 4u && f.smmu[GM2_SMMU_SMR(0) / 4] == 0x80010000u);
    gm2_smmu_read(&io, 0, &s);
    CHECK(s.path == GM2_PATH_CB_OFF && s.cb == 4 && gm2_identity_after(&s, &c));
  }
  {   // mode 3: stage-1 identity table on context bank 1
    Fake f;
    phoneSmmu(f, true);
    const gm2_io io = {&f, frd, fwr, fdelay, fflush};
    gm2_smmu s;
    gm2_smmu_read(&io, 0, &s);
    gm2_smmu_cfg c;
    gm2_smmu_setup(&io, &s, GM2_SM_CB_IDENTITY, 1, table.data(), tablePhys, &c);
    CHECK(c.ok && c.used == GM2_SM_CB_IDENTITY && c.cb == 1 && c.ttbrRead == tablePhys);
    CHECK(table[0] == 0x445u && table[1] == 0x40000441ull && table[3] == 0xc0000441ull);
    const uint32_t cb1 = 0x8000 + 0x1000;
    CHECK(f.smmu[(cb1 + GM2_CB_TCR) / 4] == GM2_TCR_IDENTITY && f.smmu[(cb1 + GM2_CB_MAIR0) / 4] == 0x44u);
    CHECK(f.smmu[(cb1 + GM2_CB_SCTLR) / 4] == GM2_SCTLR_IDENTITY_ON && f.smmu[GM2_SMMU_S2CR(0) / 4] == 1u);
    gm2_smmu_read(&io, 0, &s);
    CHECK(s.path == GM2_PATH_CB_TRANSLATE && gm2_identity_after(&s, &c));
  }
  {   // mode 3 without AArch64 tables: stops, the group stays FAULT
    Fake f;
    phoneSmmu(f, false);
    const gm2_io io = {&f, frd, fwr, fdelay, fflush};
    gm2_smmu s;
    gm2_smmu_read(&io, 0, &s);
    gm2_smmu_cfg c;
    gm2_smmu_setup(&io, &s, GM2_SM_CB_IDENTITY, 0xff, table.data(), tablePhys, &c);
    CHECK(!c.ok && c.used == GM2_SM_NONE && c.why && ((f.smmu[GM2_SMMU_S2CR(0) / 4] >> 16) & 3u) == 2);
    gm2_smmu_read(&io, 0, &s);
    CHECK(!gm2_identity_after(&s, &c));
  }
  {   // the hypervisor drops every SMMU write
    Fake f;
    phoneSmmu(f, true);
    f.smmuIgnore = true;
    const gm2_io io = {&f, frd, fwr, fdelay, fflush};
    gm2_smmu s;
    gm2_smmu_read(&io, 0, &s);
    gm2_smmu_cfg c;
    gm2_smmu_setup(&io, &s, GM2_SM_BYPASS, 0xff, table.data(), tablePhys, &c);
    CHECK(!c.ok && c.used == GM2_SM_NONE && c.why && f.smmuWrites > 0);
    gm2_smmu_read(&io, 0, &s);
    CHECK(s.path == GM2_PATH_UNMATCHED_FAULT && !gm2_identity_after(&s, &c));
  }
  {   // mode 0 and an already untranslated path: nothing written
    Fake f;
    phoneSmmu(f, true);
    const gm2_io io = {&f, frd, fwr, fdelay, fflush};
    gm2_smmu s;
    gm2_smmu_read(&io, 0, &s);
    gm2_smmu_cfg c;
    gm2_smmu_setup(&io, &s, GM2_SM_NONE, 0xff, table.data(), tablePhys, &c);
    CHECK(!c.ok && f.smmuWrites == 0);
    f.smmu[0] |= 1u;   // CLIENTPD
    gm2_smmu_read(&io, 0, &s);
    gm2_smmu_setup(&io, &s, GM2_SM_BYPASS, 0xff, table.data(), tablePhys, &c);
    CHECK(c.ok && c.used == GM2_SM_NONE && f.smmuWrites == 0);
  }
}

static void testSubmit(bool execute) {
  Fake f;
  f.execute = execute;
  std::vector<uint32_t> mem(0x10000 / 4, 0);
  f.ring = &mem[0xe000 / 4];
  f.scratch = &mem[0xf000 / 4];
  f.ringPhys = 0x80000000ull + 0xe000;
  f.scratchPhys = 0x80000000ull + 0xf000;
  const gm2_io io = {&f, frd, fwr, fdelay, fflush};
  const gm2_mem m = {&mem[0], 0x80000000ull, f.ring, f.ringPhys, 1024, f.scratch, f.scratchPhys};
  gm2_cp c;
  gm2_cp_start(&io, &m, &c);
  gm2_ring rb = {f.ring, f.ringPhys, 1024, c.wptr, 0, f.scratch, f.scratchPhys};
  uint32_t waited = 0;
  int e = gm2_submit(&io, &rb, nullptr, 0, 200000, &waited);   // fence only
  if (!execute) {
    CHECK(e == GM2_SUBMIT_TIMEOUT && waited >= 200000 && rb.ts == 1);
    return;
  }
  CHECK(e == GM2_SUBMIT_OK && waited == 0 && rb.ts == 1 && gm2_fence(&io, &rb) == 1 && rb.wptr == 18);
  CHECK(f.ring[13] == 0x70460004u && f.ring[14] == 0x80000004u && f.ring[15] == static_cast<uint32_t>(f.scratchPhys + 32));
  CHECK((f.gpu[GM2_RBBM_INT_0_STATUS] >> 20) & 1u);
  const uint64_t t = f.scratchPhys + 48;
  const uint32_t pk[4] = {gm2_pkt7(GM2_CP_MEM_WRITE, 3), static_cast<uint32_t>(t), static_cast<uint32_t>(t >> 32), 0x1234};
  e = gm2_submit(&io, &rb, pk, 4, 200000, &waited);
  CHECK(e == GM2_SUBMIT_OK && rb.ts == 2 && f.scratch[12] == 0x1234 && gm2_fence(&io, &rb) == 2);
  // 300-dword packets: the fourth one pads with a NOP and wraps to the start of the ring
  std::vector<uint32_t> big(300, 0);
  big[0] = gm2_pkt7(GM2_CP_NOP, 299);
  for (unsigned i = 0; i < 6; i++) {
    e = gm2_submit(&io, &rb, big.data(), 300, 200000, &waited);
    CHECK(e == GM2_SUBMIT_OK && gm2_fence(&io, &rb) == rb.ts && f.gpu[GM2_CP_RB_RPTR] == rb.wptr);
  }
  CHECK(rb.ts == 8 && rb.wptr < 1024);
  std::vector<uint32_t> huge(1020, 0);
  CHECK(gm2_submit(&io, &rb, huge.data(), 1020, 1000, &waited) == GM2_SUBMIT_TOO_BIG);
  char names[64];
  gm2_int_names((1u << 20) | (1u << 9), names, sizeof(names));
  CHECK(!strcmp(names, "cp_hw_error,cache_flush_ts"));
  gm2_int_names(0, names, sizeof(names));
  CHECK(!strcmp(names, "none"));
  gm2_int_names(0xffffffffu, names, 8);   // truncated, still terminated
  CHECK(strlen(names) == 7);
}

static void testCp(bool execute, bool locked) {
  Fake f;
  f.execute = execute;
  f.secvidLocked = locked;
  f.gpu[GM2_RBBM_SECVID_TRUST_CNTL] = 1;
  std::vector<uint32_t> mem(0x10000 / 4, 0);
  f.ring = &mem[0xe000 / 4];
  f.scratch = &mem[0xf000 / 4];
  f.ringPhys = 0x80000000ull + 0xe000;
  f.scratchPhys = 0x80000000ull + 0xf000;
  const gm2_io io = {&f, frd, fwr, fdelay, fflush};
  const gm2_mem m = {&mem[0], 0x80000000ull, f.ring, f.ringPhys, 1024, f.scratch, f.scratchPhys};
  gm2_cp c;
  gm2_cp_start(&io, &m, &c);
  CHECK(f.gpu[GM2_CP_RB_CNTL] == 0x08000209u && c.rbCntl == 0x08000209u);
  CHECK(f.gpu[GM2_CP_RB_BASE] == static_cast<uint32_t>(f.ringPhys) && f.gpu[GM2_CP_RB_BASE_HI] == 0);
  CHECK(f.gpu[GM2_CP_SQE_INSTR_BASE_LO] == 0x80000000u && f.gpu[GM2_CP_SQE_CNTL] == 1);
  CHECK(f.gpu[GM2_RBBM_SW_RESET_CMD] == 0 && f.gpu[GM2_GBIF_QSB_SIDE0 + 3] == 0x71620 && f.gpu[GM2_CP_MEM_POOL_SIZE] == 48);
  CHECK(f.ring[0] == 0x70c80008u && f.ring[9] == 0x703d8003u && c.wptr == 13 && f.gpu[GM2_CP_RB_WPTR] == 13);
  CHECK(f.ring[10] == static_cast<uint32_t>(f.scratchPhys + 16));
  CHECK(c.consumed == execute && c.wrote == execute && c.rptr == (execute ? 13u : 0u));
  CHECK(execute ? c.waitedUs == 0 : c.waitedUs == GM2_POLL_STEPS * GM2_POLL_STEP_US);
  CHECK(c.secvidBefore == 1 && c.secvidAfter == (locked ? 1u : 0u));
  CHECK(f.writes < 1000 && f.flushes >= 3);
}

static uint32_t gLogged, gLastLogged;
static void hookLog(uint32_t step, uint32_t) { gLogged++; gLastLogged = step; }

// Step trace, guamp-gpu-m2-stop and the start without soft reset (the first start of GpuService.inc).
static void testSteps() {
  volatile uint32_t progress[3] = {0, 0, 0};
  for (int pass = 0; pass < 2; pass++) {
    Fake f;
    std::vector<uint32_t> mem(0x10000 / 4, 0);
    f.ring = &mem[0xe000 / 4];
    f.scratch = &mem[0xf000 / 4];
    f.ringPhys = 0x80000000ull + 0xe000;
    f.scratchPhys = 0x80000000ull + 0xf000;
    const gm2_io io = {&f, frd, fwr, fdelay, fflush};
    const gm2_mem m = {&mem[0], 0x80000000ull, f.ring, f.ringPhys, 1024, f.scratch, f.scratchPhys};
    gLogged = 0;
    gm2_hook() = {progress, hookLog, pass == 0 ? GM2_STEP_UNHALT : 0u};
    gm2_cp c;
    gm2_cp_start_ex(&io, &m, &c, false);
    CHECK(f.resetWrites == 0 && !c.softReset);
    if (pass == 0) {   // stopped before the unhalt: microcode base written, CP still halted, nothing fetched
      CHECK(c.stoppedAt == GM2_STEP_UNHALT && f.gpu[GM2_CP_SQE_CNTL] == 0 && f.gpu[GM2_CP_RB_WPTR] == 0);
      CHECK(progress[0] == GM2_STEP_SQE_BASE && progress[1] == GM2_CP_SQE_INSTR_BASE_HI && gLastLogged == GM2_STEP_SQE_BASE);
      CHECK(f.gpu[GM2_CP_SQE_INSTR_BASE_LO] == 0x80000000u && !c.consumed);
    } else {           // full run: last step SECVID, every step logged once (1, 5-17 without the reset steps)
      CHECK(c.stoppedAt == 0 && c.consumed && c.wrote && progress[0] == GM2_STEP_SECVID && gLogged == 14);
      gm2_ring rb = {f.ring, f.ringPhys, 1024, c.wptr, 0, f.scratch, f.scratchPhys};
      uint32_t waited = 0;
      CHECK(gm2_submit(&io, &rb, nullptr, 0, 1000, &waited) == GM2_SUBMIT_OK && progress[0] == GM2_STEP_SUBMIT_WAIT);
    }
    CHECK(progress[2] > 10);
  }
  gm2_cp c2;
  {
    Fake f;
    std::vector<uint32_t> mem(0x10000 / 4, 0);
    f.ring = &mem[0xe000 / 4];
    f.scratch = &mem[0xf000 / 4];
    f.scratchPhys = 0x80000000ull + 0xf000;
    const gm2_io io = {&f, frd, fwr, fdelay, fflush};
    const gm2_mem m = {&mem[0], 0x80000000ull, f.ring, 0x80000000ull + 0xe000, 1024, f.scratch, f.scratchPhys};
    gm2_hook() = {progress, hookLog, GM2_STEP_RESET_ON};   // the recovery form resets: stop right there
    gm2_cp_start(&io, &m, &c2);
    CHECK(c2.stoppedAt == GM2_STEP_RESET_ON && f.resetWrites == 0 && c2.softReset);
  }
  gm2_hook() = {nullptr, nullptr, 0};
}

int main() {
  testPkt();
  testLp();
  testSmmu();
  testCp(true, false);
  testCp(false, true);
  testSmmuSetup();
  testSubmit(true);
  testSubmit(false);
  testSteps();
  printf("gpu_m2_host: %d checks, %d failures\n", checks, fails);
  return fails ? 1 : 0;
}
