// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of GuampGpu3D.h: an independent ir3 encoder written from the field positions of Mesa's isa/*.xml is first
// checked against the encodings listed in Mesa's ir3/tests/disasm.c, then used to rebuild the three shaders; the
// command stream is parsed back (type-4 / type-7 headers with parity, counts) into a register file and checked.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <vector>
#include "../GuampGpu3D.h"

static int fails = 0, checks = 0;
#define CHECK(c, ...) do { checks = checks + 1; if (!(c)) { fails = fails + 1; printf("FAIL %s:%d ", __FILE__, __LINE__); \
  printf(__VA_ARGS__); printf("\n"); } } while (0)

// ---- independent encoder (bit positions from ir3-cat0/1/2/5.xml, ir3-common.xml) ------------------------------------
static uint64_t b(uint64_t v, unsigned lo) { return v << lo; }
static unsigned gpr(unsigned n, unsigned c) { return n << 2 | c; }
static uint64_t cat0(unsigned opc, bool sy, unsigned rpt = 0, bool eq = false) {
  return b(opc, 55) | b(rpt, 40) | b(eq, 48) | b(sy, 60) | b(0, 61);
}
static uint64_t movConst(unsigned dst, unsigned cnst, unsigned type) {   // mov-const, same src/dst type
  return b(cnst, 0) | b(dst, 32) | b(type, 46) | b(type, 50) | b(1, 53) | b(1, 61);
}
static uint64_t baryF(unsigned dst, unsigned inloc, unsigned ij, bool ei, unsigned rpt, bool src1r) {
  const uint64_t src1 = inloc | b(4, 11);              // #mulitsrc-immed: 11-13 = 100
  const uint64_t src2 = ij;                            // #multisrc-gpr
  return src1 | b(src2, 16) | b(dst, 32) | b(rpt, 40) | b(src1r, 43) | b(ei, 47) | b(1, 52) | b(0x39, 53) | b(2, 61);
}
static uint64_t sam(unsigned dst, unsigned src1, unsigned s, unsigned t, unsigned wrmask, unsigned type, bool full = true) {
  return b(full, 0) | b(src1, 1) | b(s, 21) | b(t, 25) | b(dst, 32) | b(wrmask, 40) | b(type, 44) | b(3, 54) | b(5, 61);
}
static uint64_t at(const uint32_t *code, unsigned i) { return code[2 * i] | static_cast<uint64_t>(code[2 * i + 1]) << 32; }

static void testIsa() {
  // ir3/tests/disasm.c (Mesa 25.0.0), INSTR_6XX(hi_lo, text)
  CHECK(cat0(0, false) == 0x0000000000000000ull, "nop");
  CHECK(cat0(0, false, 2) == 0x0000020000000000ull, "(rpt2)nop");
  CHECK(cat0(0, false, 0, true) == 0x0001000000000000ull, "(eq)nop");
  CHECK(cat0(6, false) == 0x0300000000000000ull, "end");
  CHECK(movConst(gpr(0, 0), gpr(8, 0), 1) == 0x2024400000000020ull, "mov.f32f32 r0.x, c8.x");
  CHECK(baryF(gpr(0, 2), 0, gpr(0, 0), true, 2, true) == 0x47308a0200002000ull, "(rpt2)bary.f (ei)r0.z, (r)0, r0.x");
  CHECK(sam(gpr(1, 0), gpr(0, 2), 6, 6, 0xf, 1) == 0xa0c01f040cc00005ull, "sam (f32)(xyzw)r1.x, r0.z, s#6, t#6");
  CHECK(sam(0, gpr(0, 1), 2, 2, 0xf, 0, false) == 0xa0c00f0004400002ull, "sam (f16)(xyzw)hr0.x, hr0.y, s#2, t#2");

  // shaders rebuilt with the encoder
  CHECK(at(g3d_vs, 0) == cat0(6, false), "vs end");
  for (unsigned i = 1; i < G3D_SHADER_INSTRS; i = i + 1) CHECK(at(g3d_vs, i) == 0, "vs pad %u", i);
  for (unsigned c = 0; c < 4; c = c + 1)
    CHECK(at(g3d_fs_solid, c) == movConst(gpr(0, c), gpr(0, c), 1), "solid mov %u %016llx", c,
          static_cast<unsigned long long>(at(g3d_fs_solid, c)));
  CHECK(at(g3d_fs_solid, 4) == cat0(6, false), "solid end");
  for (unsigned i = 5; i < G3D_SHADER_INSTRS; i = i + 1) CHECK(at(g3d_fs_solid, i) == 0, "solid pad %u", i);
  CHECK(at(g3d_fs_tex, 0) == baryF(gpr(0, 2), 0, gpr(0, 0), false, 0, false), "tex bary u");
  CHECK(at(g3d_fs_tex, 1) == baryF(gpr(0, 3), 1, gpr(0, 0), true, 0, false), "tex bary v (ei)");
  for (unsigned i = 2; i < 8; i = i + 1) CHECK(at(g3d_fs_tex, i) == 0, "tex delay nop %u", i);
  CHECK(at(g3d_fs_tex, 8) == sam(gpr(1, 0), gpr(0, 2), 0, 0, 0xf, 1), "tex sam");
  CHECK(at(g3d_fs_tex, 9) == cat0(6, true), "tex (sy)end");
  for (unsigned i = 10; i < G3D_SHADER_INSTRS; i = i + 1) CHECK(at(g3d_fs_tex, i) == 0, "tex pad %u", i);
  // alu -> tex delay (ir3_delay.c): the last bary.f (1) to sam (8) leaves 6 instructions between them
  CHECK(8 - 1 - 1 == 6, "delay slots");
  CHECK(sizeof g3d_vs == G3D_SHADER_BYTES && G3D_SHADER_BYTES % 128 == 0, "shader size");
}

// ---- float helpers --------------------------------------------------------------------------------------------------
static uint32_t bits(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }
static void testFloat() {
  for (uint32_t n = 0; n < 5000; n = n + 1) CHECK(g3d_f32(n) == bits(static_cast<float>(n)), "f32 %u", n);
  CHECK(g3d_f32(16777215) == bits(16777215.0f), "f32 2^24-1");
  for (uint32_t v = 0; v < 256; v = v + 1)
    CHECK(g3d_unorm8(v) == bits(static_cast<float>(v / 255.0)), "unorm8 %u %08x %08x", v, g3d_unorm8(v),
          bits(static_cast<float>(v / 255.0)));
}

// ---- command stream parser ------------------------------------------------------------------------------------------
static uint32_t oddParity(uint32_t v) { uint32_t p = 1; while (v) { p ^= v & 1; v >>= 1; } return p; }
struct Stream {
  std::map<uint32_t, uint32_t> regs;
  std::map<uint32_t, unsigned> writes;
  std::vector<std::pair<uint32_t, std::vector<uint32_t>>> pkts;   // type 7: opcode, payload
  bool ok = true;
};
static Stream parse(const uint32_t *p, size_t n) {
  Stream s;
  size_t i = 0;
  while (i < n) {
    const uint32_t h = p[i];
    if ((h >> 28) == 4) {
      const uint32_t cnt = h & 0x7f, reg = (h >> 8) & 0x3ffff;
      if (((h >> 7) & 1) != oddParity(cnt) || ((h >> 27) & 1) != oddParity(reg) || ((h >> 26) & 1)) { s.ok = false; break; }
      if (i + 1 + cnt > n) { s.ok = false; break; }
      for (uint32_t k = 0; k < cnt; k = k + 1) { s.regs[reg + k] = p[i + 1 + k]; s.writes[reg + k] += 1; }
      i += 1 + cnt;
    } else if ((h >> 28) == 7) {
      const uint32_t cnt = h & 0x3fff, op = (h >> 16) & 0x7f;
      if (((h >> 15) & 1) != oddParity(cnt) || ((h >> 23) & 1) != oddParity(op)) { s.ok = false; break; }
      if (i + 1 + cnt > n) { s.ok = false; break; }
      s.pkts.push_back({op, std::vector<uint32_t>(p + i + 1, p + i + 1 + cnt)});
      i += 1 + cnt;
    } else {
      s.ok = false;
      break;
    }
  }
  return s;
}
static uint32_t reg(const Stream &s, uint32_t r) { auto it = s.regs.find(r); return it == s.regs.end() ? 0xdeadbeefu : it->second; }

static void testStream() {
  g3d_mem m = {0x40001000ull, 0x40001100ull, 0x40001200ull, 0x40001300ull, 0x40001400ull, 0x40001480ull};
  g3d_draw d = {};
  d.kind = G3D_SOLID;
  d.dst = {0x9c000000ull, 2880, 720, 1600, G3D_FMT6_8_8_8_8_UNORM, G3D_SWAP_WXYZ};
  d.x0 = 100; d.y0 = 200; d.x1 = 300; d.y1 = 500;
  d.color[0] = g3d_unorm8(255); d.color[1] = g3d_unorm8(0); d.color[2] = g3d_unorm8(128); d.color[3] = g3d_unorm8(255);
  d.seqno = 0x1234;
  static uint32_t buf[4096];
  const size_t n = g3d_build(buf, 4096, &m, &d, true);
  CHECK(n > 100 && n < 1200, "solid stream size %zu", n);
  CHECK(g3d_build(buf, 50, &m, &d, true) == 0, "overflow returns 0");
  g3d_build(buf, 4096, &m, &d, true);
  Stream s = parse(buf, n);
  CHECK(s.ok, "solid stream parses (headers, parity, counts)");
  // render target
  CHECK(reg(s, 0x8822) == (0x30u | 1u << 13), "RB_MRT0_BUF_INFO %08x", reg(s, 0x8822));
  CHECK(reg(s, 0x8823) == 2880 / 64, "RB_MRT0_PITCH");
  CHECK(reg(s, 0x8825) == 0x9c000000u && reg(s, 0x8826) == 0, "RB_MRT0_BASE");
  CHECK(reg(s, 0x8820) == 0x780, "RB_MRT0_CONTROL component enable");
  CHECK(reg(s, 0xa996) == 0x30, "SP_FS_MRT_REG0");
  CHECK(reg(s, 0x80f1) == (1599u << 16 | 719u), "window scissor BR %08x", reg(s, 0x80f1));
  CHECK(reg(s, 0x8e07) == 0x01000001u, "RB_CCU_CNTL %08x", reg(s, 0x8e07));
  CHECK(reg(s, 0x80a1) == 0x00e00000u && reg(s, 0x8800) == 0x00e00000u, "BIN_CONTROL sysmem");
  CHECK(reg(s, 0x8000) == 0x381, "GRAS_CL_CNTL");
  // shaders
  CHECK(reg(s, 0xa81c) == 0x40001000u && reg(s, 0xa81d) == 0, "SP_VS_OBJ_START");
  CHECK(reg(s, 0xa983) == 0x40001100u, "SP_FS_OBJ_START");
  CHECK(reg(s, 0xa824) == 1 && reg(s, 0xab05) == 1, "INSTRLEN");
  CHECK(reg(s, 0xa800) == (2u << 7 | 1u << 20), "SP_VS_CTRL_REG0");
  CHECK(reg(s, 0xa980) == (1u << 7 | 1u << 24 | 1u << 31), "SP_FS_CTRL_REG0 solid %08x", reg(s, 0xa980));
  CHECK(reg(s, 0xbb10) == (1u << 8 | 1u), "HLSQ_FS_CNTL constlen 4 vec4");
  CHECK(reg(s, 0xab04) == 0x100, "SP_FS_CONFIG solid");
  CHECK(reg(s, 0xa803) == 0xf00 && reg(s, 0x9301) == (4u | 0xffu << 16), "VS out / VPC pack solid");
  CHECK(reg(s, 0x9212) == 0xffffffffu, "VPC_VAR_DISABLE solid");
  CHECK(reg(s, 0xa99e) == 8, "PREFETCH_CNTL IJ_WRITE_DISABLE");
  // vertex fetch
  CHECK(reg(s, 0xa010) == 0x40001200u && reg(s, 0xa012) == 48 && reg(s, 0xa013) == 24, "VFD_FETCH0");
  CHECK(reg(s, 0xa090) == (0x82u << 20 | 3u << 30) && reg(s, 0xa092) == (16u << 5 | 0x67u << 20 | 3u << 30),
        "VFD_DECODE %08x %08x", reg(s, 0xa090), reg(s, 0xa092));
  CHECK(reg(s, 0xa0d0) == 0xf && reg(s, 0xa0d1) == 0x43, "VFD_DEST_CNTL");
  CHECK(reg(s, 0xa000) == 0x202, "VFD_CONTROL_0");
  // static init values (A610 magic)
  CHECK(reg(s, 0x8e04) == 0x04100000u && reg(s, 0x9804) == 0xf && reg(s, 0x0e19) == 4, "magic regs");
  CHECK(reg(s, 0xb302) == 0x40001400u && reg(s, 0xb180) == 0x40001400u, "border colour");
  // packets: shader loads, inline c0, draw, flush
  bool vsLoad = false, fsLoad = false, consts = false, draw = false, flush = false;
  size_t drawAt = 0, flushAt = 0;
  for (size_t k = 0; k < s.pkts.size(); k = k + 1) {
    const auto &pk = s.pkts[k];
    if (pk.first == 0x32 && pk.second.size() == 3 && pk.second[0] == (2u << 16 | 8u << 18 | 1u << 22) &&
        pk.second[1] == 0x40001000u)
      vsLoad = true;
    if (pk.first == 0x34 && pk.second.size() == 3 && pk.second[0] == (2u << 16 | 12u << 18 | 1u << 22)) fsLoad = true;
    if (pk.first == 0x34 && pk.second.size() == 7 && pk.second[0] == (1u << 14 | 12u << 18 | 1u << 22) &&
        pk.second[3] == 0x3f800000u && pk.second[4] == 0 && pk.second[6] == 0x3f800000u)
      consts = true;
    if (pk.first == 0x38 && pk.second.size() == 3 && pk.second[0] == 0x88 && pk.second[1] == 1 && pk.second[2] == 2) {
      draw = true;
      drawAt = k;
    }
    if (pk.first == 0x46 && pk.second.size() == 4 && pk.second[0] == 29 && pk.second[1] == 0x40001480u &&
        pk.second[3] == 0x1234) {
      flush = true;
      flushAt = k;
    }
  }
  CHECK(vsLoad && fsLoad && consts, "load state (vs %d fs %d c0 %d)", vsLoad, fsLoad, consts);
  CHECK(draw && flush && flushAt > drawAt, "draw then CCU flush");
  // vertices
  uint32_t v[G3D_VBO_DWORDS];
  g3d_fill_vertices(v, &d);
  CHECK(v[0] == bits(100.0f) && v[1] == bits(200.0f) && v[3] == bits(1.0f) && v[6] == bits(300.0f) &&
        v[7] == bits(500.0f), "vertices");

  // textured
  d.kind = G3D_TEXTURED;
  d.src = {0x9d000000ull, 1024, 256, 256, G3D_FMT6_8_8_8_8_UNORM, G3D_SWAP_WXYZ};
  d.u0 = 10; d.v0 = 20; d.linear = true;
  const size_t nt = g3d_build(buf, 4096, &m, &d, false);
  Stream t = parse(buf, nt);
  CHECK(t.ok && nt > 100, "textured stream parses");
  CHECK(reg(t, 0xa980) == (2u << 7 | 1u << 22 | 1u << 23 | 1u << 24 | 1u << 31), "SP_FS_CTRL_REG0 tex %08x", reg(t, 0xa980));
  CHECK(reg(t, 0xab04) == (0x100u | 1u << 9 | 1u << 17), "SP_FS_CONFIG tex");
  CHECK(reg(t, 0xa803) == 0x0f000304u && reg(t, 0xa813) == 0x200 && reg(t, 0x9301) == (6u | 2u << 8 | 0xffu << 16),
        "linkage uv loc 0, pos loc 2");
  CHECK(reg(t, 0x9212) == 0xfffffffcu && reg(t, 0x9304) == (2u | 0xffu << 8 | 1u << 16 | 0xffu << 24), "varyings");
  CHECK(reg(t, 0x9208) == 9, "PS_REPL_MODE S/T");
  CHECK(reg(t, 0xb984) == 0xfcfcfc00u && reg(t, 0x8005) == 1 && reg(t, 0x8809) == 0x401, "ij persp pixel r0.x");
  CHECK(reg(t, 0xa98e) == 4, "FS output r1.x");
  CHECK(reg(t, 0xa9e4) == 0x40001300u && reg(t, 0xa9e0) == 0x40001340u && reg(t, 0xa9a7) == 1, "tex/samp state");
  CHECK(t.writes.count(0x8e04) == 0, "no init when not asked");
  uint32_t tx[G3D_TEX_DWORDS];
  g3d_fill_texture(tx, &d.src, true);
  CHECK(tx[0] == 0x4c006880u && tx[1] == (256u | 256u << 15) && tx[2] == (1024u << 7 | 1u << 29) &&
        tx[4] == 0x9d000000u && tx[5] == 1u << 17, "tex const %08x %08x %08x", tx[0], tx[1], tx[2]);
  CHECK(tx[16] == (1u << 1 | 1u << 3 | 0x920u | 0x60000u) && tx[17] == 0x60, "sampler %08x", tx[16]);
  g3d_fill_vertices(v, &d);
  CHECK(v[4] == bits(10.0f) && v[5] == bits(20.0f) && v[10] == bits(210.0f) && v[11] == bits(320.0f), "uv");
}

int main() {
  testIsa();
  testFloat();
  testStream();
  printf("gpu_3d_host: %d checks, %d failed\n", checks, fails);
  return fails ? 1 : 0;
}
