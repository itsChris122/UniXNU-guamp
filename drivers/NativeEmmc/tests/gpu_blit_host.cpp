// Host test of GuampGpuBlit.h: packet headers (values of freedreno a6xx dumps), parity of every packet, the register
// values against the a6xx.xml fields, and the fill run by a fake CP/2D engine that decodes the ring the way the
// hardware fields are documented (rectangle, format, swap, pitch, address) on a fake BGRA/RGBA surface, with the
// fence written last.
#include <stdio.h>
#include <string.h>
#include <map>
#include <vector>
#include "../GuampGpuBlit.h"

static int fails, checks;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

struct FakeGpu {
  std::map<uint32_t, uint32_t> regs;
  uint64_t surfBase = 0;
  std::vector<uint32_t> surf;   // words
  uint64_t tsBase = 0, fenceAddr = 0;
  uint32_t ts[4] = {}, fence = 0;
  int blits = 0, events = 0, wfis = 0, badPackets = 0;
  bool fenceAfterBlit = false;

  void memWrite(uint64_t a, uint32_t v) {
    if (a == fenceAddr) { fence = v; fenceAfterBlit = blits > 0; return; }
    if (a >= tsBase && a < tsBase + 16) { ts[(a - tsBase) / 4] = v; return; }
    badPackets++;
  }
  void blit() {
    blits++;
    const uint32_t cntl = regs[GMB_RB_2D_BLIT_CNTL];
    if (cntl != regs[GMB_GRAS_2D_BLIT_CNTL] || !(cntl & 0x80u) || ((cntl >> 8) & 0xffu) != 0x30u) { badPackets++; return; }
    if (regs[GMB_RB_DBG_ECO_CNTL] != GMB_RB_DBG_ECO_CNTL_BLIT) { badPackets++; return; }
    const uint32_t info = regs[GMB_RB_2D_DST_INFO];
    const uint64_t dst = regs[GMB_RB_2D_DST_LO] | static_cast<uint64_t>(regs[GMB_RB_2D_DST_HI]) << 32;
    const uint32_t pitch = regs[GMB_RB_2D_DST_PITCH] << 6;
    const uint32_t tl = regs[GMB_GRAS_2D_DST_TL], br = regs[GMB_GRAS_2D_DST_BR];
    const uint32_t x0 = tl & 0x3fff, y0 = (tl >> 16) & 0x3fff, x1 = br & 0x3fff, y1 = (br >> 16) & 0x3fff;
    uint8_t rgba[4];
    for (int i = 0; i < 4; i++) rgba[i] = static_cast<uint8_t>(regs[GMB_RB_2D_SRC_SOLID_C0 + i]);
    const uint32_t swap = (info >> 10) & 3;
    uint8_t b[4];
    if (swap == GMB_SWAP_WXYZ) { b[0] = rgba[2]; b[1] = rgba[1]; b[2] = rgba[0]; b[3] = rgba[3]; }
    else { b[0] = rgba[0]; b[1] = rgba[1]; b[2] = rgba[2]; b[3] = rgba[3]; }
    const uint32_t word = b[0] | b[1] << 8 | b[2] << 16 | static_cast<uint32_t>(b[3]) << 24;
    for (uint32_t y = y0; y <= y1; y++)
      for (uint32_t x = x0; x <= x1; x++) {
        const uint64_t a = dst + static_cast<uint64_t>(y) * pitch + x * 4u;
        if (a < surfBase || a + 4 > surfBase + surf.size() * 4) { badPackets++; return; }
        surf[(a - surfBase) / 4] = word;
      }
  }
  void run(const uint32_t *r, uint32_t n) {
    uint32_t i = 0;
    while (i < n) {
      const uint32_t h = r[i];
      if ((h >> 28) == 4) {
        const uint32_t cnt = h & 0x7f, reg = (h >> 8) & 0x3ffff;
        if (((h >> 7) & 1) != gm2_parity(cnt) || ((h >> 27) & 1) != gm2_parity(reg)) badPackets++;
        for (uint32_t k = 0; k < cnt; k++) regs[reg + k] = r[i + 1 + k];
        i += 1 + cnt;
      } else if ((h >> 28) == 7) {
        const uint32_t cnt = h & 0x3fff, op = (h >> 16) & 0x7f;
        if (((h >> 15) & 1) != gm2_parity(cnt) || ((h >> 23) & 1) != gm2_parity(op)) badPackets++;
        const uint32_t *p = &r[i + 1];
        if (op == GMB_CP_WAIT_FOR_IDLE) wfis++;
        else if (op == GMB_CP_BLIT) { if (cnt == 1 && p[0] == GMB_BLIT_OP_SCALE) blit(); else badPackets++; }
        else if (op == GMB_CP_EVENT_WRITE) {
          events++;
          if (cnt == 4) {
            if (!(p[0] & GMB_EV_TIMESTAMP)) badPackets++;
            memWrite(p[1] | static_cast<uint64_t>(p[2]) << 32, p[3]);
          } else if (cnt != 1) badPackets++;
        } else badPackets++;
        i += 1 + cnt;
      } else { badPackets++; return; }
    }
    if (i != n) badPackets++;
  }
};

static void testHeaders() {
  CHECK(gm2_pkt7(GMB_CP_WAIT_FOR_IDLE, 0) == 0x70268000u);   // the familiar WFI dword of a6xx dumps
  CHECK(gm2_pkt7(GMB_CP_BLIT, 1) == 0x702c0001u);
  CHECK(gm2_pkt7(GMB_CP_EVENT_WRITE, 4) == 0x70460004u);
  CHECK(gm2_pkt7(GMB_CP_EVENT_WRITE, 1) == 0x70460001u);
  CHECK(gmb_pkt4(GMB_RB_2D_BLIT_CNTL, 1) == 0x408c0001u);
  CHECK(gmb_pkt4(GMB_GRAS_2D_DST_TL, 2) == 0x48840502u);
  CHECK(gmb_pkt4(GMB_RB_CCU_CNTL, 1) == 0x408e0701u);
  CHECK(GMB_BLIT_CNTL(GMB_FMT6_8_8_8_8_UNORM) == 0x10f03080u);
  CHECK(GMB_SP_2D_DST_FORMAT_VAL(GMB_FMT6_8_8_8_8_UNORM) == 0x0000f180u);
  CHECK(GMB_RB_CCU_CNTL_SYSMEM == 0x01000001u);
  CHECK(gmb_pixel(0xff112233u, true) == 0xff112233u);
  CHECK(gmb_pixel(0xff112233u, false) == 0xff332211u);
}

static gmb_clear base() {
  gmb_clear c = {};
  c.dst = 0x5C000000ull; c.pitch = 2880; c.surfW = 720; c.surfH = 1600;
  c.x = 100; c.y = 200; c.w = 300; c.h = 150; c.argb = 0xffff00ffu; c.bgra = true; c.smmuIdentity = true;
  c.ts = 0x80000000ull + 0xf020; c.fence = 0x80000000ull + 0xf030; c.fenceValue = 0x1234;
  return c;
}

static void runFill(gmb_clear c) {
  uint32_t ring[GMB_CLEAR_MAX_DWORDS] = {};
  const uint32_t n = gmb_build_clear(ring, GMB_CLEAR_MAX_DWORDS, &c);
  CHECK(n > 0 && n <= GMB_CLEAR_MAX_DWORDS);
  FakeGpu g;
  g.surfBase = c.dst;
  g.surf.assign(static_cast<size_t>(c.pitch / 4) * c.surfH, 0x0badf00du);
  g.tsBase = c.ts; g.fenceAddr = c.fence;
  g.run(ring, n);
  CHECK(g.badPackets == 0);
  CHECK(g.blits == 1 && g.wfis == 3 && g.events == 7);
  CHECK(g.fence == c.fenceValue && g.fenceAfterBlit);
  CHECK(g.regs[GMB_RB_DBG_ECO_CNTL] == 0 && g.regs[GMB_RB_CCU_CNTL] == GMB_RB_CCU_CNTL_SYSMEM);
  CHECK(g.regs[GMB_RB_2D_UNKNOWN_8C01] == 0 && g.regs[GMB_RB_UNKNOWN_8E01] == 1);
  const uint32_t want = gmb_pixel(c.argb, c.bgra);
  uint32_t inside = 0, outsideTouched = 0, insideWrong = 0;
  for (uint32_t y = 0; y < c.surfH; y++)
    for (uint32_t x = 0; x < c.pitch / 4; x++) {
      const uint32_t v = g.surf[y * (c.pitch / 4) + x];
      const bool in = x >= c.x && x < c.x + c.w && y >= c.y && y < c.y + c.h;
      if (in) { inside++; if (v != want) insideWrong++; }
      else if (v != 0x0badf00du) outsideTouched++;
    }
  CHECK(inside == c.w * c.h && insideWrong == 0 && outsideTouched == 0);
}

static void testFill() {
  runFill(base());
  gmb_clear c = base();
  c.bgra = false; c.argb = 0x80102030u;
  runFill(c);
  c = base();
  c.x = 0; c.y = 0; c.w = 720; c.h = 1600;   // whole surface, last pixel at 719,1599
  runFill(c);
  c = base();
  c.x = 719; c.y = 1599; c.w = 1; c.h = 1;
  runFill(c);
  c = base();
  c.dst = 0x1'0000'0040ull;                   // above 4 GiB: the high dword is used
  runFill(c);
}

static void testReject() {
  uint32_t ring[GMB_CLEAR_MAX_DWORDS];
  gmb_clear c = base();
  c.smmuIdentity = false;
  CHECK(gmb_build_clear(ring, GMB_CLEAR_MAX_DWORDS, &c) == 0 && gmb_check(&c));
  c.addrIsGpuVa = true;
  CHECK(gmb_build_clear(ring, GMB_CLEAR_MAX_DWORDS, &c) > 0);
  c = base(); c.dst += 4;
  CHECK(gmb_build_clear(ring, GMB_CLEAR_MAX_DWORDS, &c) == 0);
  c = base(); c.pitch = 2900;
  CHECK(gmb_build_clear(ring, GMB_CLEAR_MAX_DWORDS, &c) == 0);
  c = base(); c.pitch = 2816;                  // 704 pixels < 720
  CHECK(gmb_build_clear(ring, GMB_CLEAR_MAX_DWORDS, &c) == 0);
  c = base(); c.w = 0;
  CHECK(gmb_build_clear(ring, GMB_CLEAR_MAX_DWORDS, &c) == 0);
  c = base(); c.x = 700; c.w = 21;
  CHECK(gmb_build_clear(ring, GMB_CLEAR_MAX_DWORDS, &c) == 0);
  c = base(); c.y = 1600;
  CHECK(gmb_build_clear(ring, GMB_CLEAR_MAX_DWORDS, &c) == 0);
  c = base(); c.x = 0xffffffffu; c.w = 2;     // no wrap-around accepted
  CHECK(gmb_build_clear(ring, GMB_CLEAR_MAX_DWORDS, &c) == 0);
  c = base(); c.fence = 0;
  CHECK(gmb_build_clear(ring, GMB_CLEAR_MAX_DWORDS, &c) == 0);
  c = base(); c.ts += 2;
  CHECK(gmb_build_clear(ring, GMB_CLEAR_MAX_DWORDS, &c) == 0);
  c = base();
  CHECK(gmb_build_clear(ring, GMB_CLEAR_MAX_DWORDS - 1, &c) == 0);
}

int main() {
  testHeaders();
  testFill();
  testReject();
  printf("gpu_blit_host: %d checks, %d failures\n", checks, fails);
  return fails ? 1 : 0;
}
