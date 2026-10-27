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
  bool execute = true, secvidLocked = false;
  unsigned writes = 0, flushes = 0;
};
static uint32_t frd(void *c, unsigned b, uint32_t off) {
  Fake *f = static_cast<Fake *>(c);
  return b == GM2_GPU ? f->gpu.at(off / 4) : f->smmu.at(off / 4);
}
static void fwr(void *c, unsigned b, uint32_t off, uint32_t v) {
  Fake *f = static_cast<Fake *>(c);
  if (b != GM2_GPU) { f->writes += 1000; return; }
  f->writes++;
  const uint32_t reg = off / 4;
  if (reg == GM2_RBBM_SECVID_TRUST_CNTL && f->secvidLocked) return;
  f->gpu.at(reg) = v;
  if (reg == GM2_CP_RB_WPTR && f->execute && f->gpu[GM2_CP_SQE_CNTL] == 1) {
    // walk the ring: ME_INIT then MEM_WRITE at the given address (the scratch page)
    uint32_t i = 0;
    while (i < v) {
      const uint32_t h = f->ring[i], cnt = h & 0x3fff, op = (h >> 16) & 0x7f;
      if (op == GM2_CP_MEM_WRITE) {
        const uint64_t a = f->ring[i + 1] | static_cast<uint64_t>(f->ring[i + 2]) << 32;
        f->scratch[(a - f->scratchPhys) / 4] = f->ring[i + 3];
      }
      i += 1 + cnt;
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
  CHECK(f.writes == 0);   // read only
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

int main() {
  testPkt();
  testLp();
  testSmmu();
  testCp(true, false);
  testCp(false, true);
  printf("gpu_m2_host: %d checks, %d failures\n", checks, fails);
  return fails ? 1 : 0;
}
