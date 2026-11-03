// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of the composition core of scripts/guamp-canvas-variant.py (section 12): run-compose-tests.sh extracts the text between
// "guamp-compose-core-begin" and "guamp-compose-core-end" into COMPOSE_CORE. Every surface and output is allocated at its exact size
// (ASan catches any read or write outside), and every result is compared with a plain floating-point reference:
//  - premultiplied source-over (exhaustive over alpha, sampled colours), the BT.601/709 video and full range conversions;
//  - unscaled, scaled (the 750 x 1666 canvas onto the 720 x 1600 panel, upscaled video) and clipped layers, nearest and bilinear;
//  - invalid layers (source outside the surface, rows past the allocation, too little scratch) draw nothing.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include COMPOSE_CORE

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

static uint32_t rng = 12345;
static uint32_t rnd() { rng = rng * 1664525u + 1013904223u; return rng >> 8; }
static uint32_t pix(uint32_t b, uint32_t g, uint32_t r, uint32_t a) { return b | g << 8 | r << 16 | a << 24; }
static int ch(uint32_t v, int i) { return (int)((v >> (8 * i)) & 255); }
static int maxDiff(uint32_t a, uint32_t b) {
  int m = 0;
  for (int i = 0; i < 4; i++) { int d = std::abs(ch(a, i) - ch(b, i)); if (d > m) m = d; }
  return m;
}

// Reference bilinear sample of a BGRA surface at output (x, y) of a dw x dh destination mapped from the source rectangle.
static double refAxis(uint32_t i, uint32_t out, uint32_t in, int *i0, int *i1) {
  double p = (i + 0.5) * in / out - 0.5;
  if (p < 0) p = 0;
  int a = (int)std::floor(p);
  if (a >= (int)in - 1) { *i0 = *i1 = in - 1; return 0; }
  *i0 = a; *i1 = a + 1; return p - a;
}
struct Surface {
  uint32_t w, h; uint64_t stride; std::vector<unsigned char> *mem;
  uint32_t at(uint32_t x, uint32_t y) const { uint32_t v; std::memcpy(&v, mem->data() + y * stride + x * 4, 4); return v; }
};
static uint32_t refBilinear(const Surface &s, const GuampComposeLayer &l, uint32_t ox, uint32_t oy, bool filter) {
  int x0, x1, y0, y1;
  double fx = refAxis(ox, l.dw, l.sw, &x0, &x1), fy = refAxis(oy, l.dh, l.sh, &y0, &y1);
  if (!filter) {
    if (std::floor(fx * 256 + 0.5) >= 128 && x1 != x0) x0 = x1;
    if (std::floor(fy * 256 + 0.5) >= 128 && y1 != y0) y0 = y1;
    return s.at(l.sx + x0, l.sy + y0);
  }
  uint32_t out = 0;
  for (int c = 0; c < 4; c++) {
    double a = ch(s.at(l.sx + x0, l.sy + y0), c), b = ch(s.at(l.sx + x1, l.sy + y0), c);
    double cc = ch(s.at(l.sx + x0, l.sy + y1), c), d = ch(s.at(l.sx + x1, l.sy + y1), c);
    double top = a + (b - a) * fx, bottom = cc + (d - cc) * fx, v = top + (bottom - top) * fy;
    out |= (uint32_t)std::lround(v) << (8 * c);
  }
  return out;
}
static uint32_t refOver(uint32_t s, uint32_t d) {
  double a = ch(s, 3) / 255.0;
  uint32_t out = 0;
  for (int c = 0; c < 4; c++) {
    double v = ch(s, c) + ch(d, c) * (1 - a);
    if (v > 255) v = 255;
    out |= (uint32_t)std::lround(v) << (8 * c);
  }
  return out;
}

static Surface makeBgra(uint32_t w, uint32_t h, uint64_t stride, std::vector<unsigned char> &mem, bool premultiplied) {
  mem.assign(stride * (h - 1) + (uint64_t)w * 4, 0);   // exact: the last row ends at its last pixel
  for (uint32_t y = 0; y < h; y++)
    for (uint32_t x = 0; x < w; x++) {
      uint32_t a = premultiplied ? rnd() & 255 : 255;
      uint32_t v = pix(rnd() % (a + 1), rnd() % (a + 1), rnd() % (a + 1), a);
      std::memcpy(mem.data() + y * stride + x * 4, &v, 4);
    }
  return Surface{w, h, stride, &mem};
}
static GuampComposeLayer bgraLayer(const Surface &s, uint32_t sx, uint32_t sy, uint32_t sw, uint32_t sh, int64_t dx, int64_t dy,
                                   uint32_t dw, uint32_t dh, unsigned filter, bool blend) {
  GuampComposeLayer l = {};
  l.base = s.mem->data(); l.bytes = s.mem->size(); l.kind = 0; l.width = s.w; l.height = s.h; l.offset = 0; l.stride = s.stride;
  l.sx = sx; l.sy = sy; l.sw = sw; l.sh = sh; l.dx = dx; l.dy = dy; l.dw = dw; l.dh = dh; l.filter = filter; l.blend = blend;
  return l;
}
struct Scratch {
  std::vector<uint32_t> taps; std::vector<unsigned char> line;
  Scratch(uint32_t width, uint32_t height, uint32_t sw) : taps(width + height), line(((size_t)sw + 1) * 4) {}
};
static int64_t draw(std::vector<uint32_t> &out, uint32_t w, uint32_t h, const GuampComposeLayer &l) {
  Scratch s(w, h, l.sw);
  return guampComposeDraw(out.data(), w, h, l, s.taps.data(), s.taps.size(), s.line.data(), s.line.size(), 0, 0, w, h);
}
static int64_t drawClip(std::vector<uint32_t> &out, uint32_t w, uint32_t h, const GuampComposeLayer &l, int64_t x0, int64_t y0,
                        int64_t x1, int64_t y1) {
  Scratch s(w, h, l.sw);
  return guampComposeDraw(out.data(), w, h, l, s.taps.data(), s.taps.size(), s.line.data(), s.line.size(), x0, y0, x1, y1);
}

static void testOver() {
  int worst = 0;
  for (uint32_t a = 0; a < 256; a++)
    for (int n = 0; n < 200; n++) {
      uint32_t s = pix(rnd() % (a + 1), rnd() % (a + 1), rnd() % (a + 1), a), d = pix(rnd() & 255, rnd() & 255, rnd() & 255, rnd() & 255);
      int e = maxDiff(guampComposeOver(s, d), refOver(s, d));
      if (e > worst) worst = e;
    }
  CHECK(worst <= 1);
  CHECK(guampComposeOver(pix(1, 2, 3, 255), pix(9, 9, 9, 9)) == pix(1, 2, 3, 255));
  CHECK(guampComposeOver(0, pix(9, 8, 7, 6)) == pix(9, 8, 7, 6));
  // Not premultiplied (colour above alpha): saturates instead of carrying into the next channel.
  CHECK(guampComposeOver(pix(200, 200, 200, 10), pix(250, 250, 250, 255)) == pix(255, 255, 255, 255));
}

static void testUnscaled() {
  std::vector<unsigned char> mem;
  Surface s = makeBgra(40, 30, 40 * 4 + 64, mem, false);
  const uint32_t W = 64, H = 48;
  std::vector<uint32_t> out(W * H, 0x11223344u);
  GuampComposeLayer l = bgraLayer(s, 5, 7, 20, 10, 10, 20, 20, 10, 1, false);
  CHECK(draw(out, W, H, l) == 200);
  bool same = true, untouched = true;
  for (uint32_t y = 0; y < H; y++)
    for (uint32_t x = 0; x < W; x++) {
      bool inside = x >= 10 && x < 30 && y >= 20 && y < 30;
      if (inside) same = same && out[y * W + x] == s.at(5 + x - 10, 7 + y - 20);
      else untouched = untouched && out[y * W + x] == 0x11223344u;
    }
  CHECK(same && untouched);
}

static void testScaled(uint32_t sw, uint32_t sh, uint32_t dw, uint32_t dh, unsigned filter, int tolerance) {
  std::vector<unsigned char> mem;
  Surface s = makeBgra(sw, sh, (uint64_t)sw * 4, mem, false);
  std::vector<uint32_t> out((size_t)dw * dh, 0);
  GuampComposeLayer l = bgraLayer(s, 0, 0, sw, sh, 0, 0, dw, dh, filter, false);
  CHECK(draw(out, dw, dh, l) == (int64_t)dw * dh);
  int worst = 0;
  for (uint32_t y = 0; y < dh; y += (dh > 200 ? 7 : 1))
    for (uint32_t x = 0; x < dw; x++) {
      int e = maxDiff(out[(size_t)y * dw + x], refBilinear(s, l, x, y, filter != 0));
      if (e > worst) worst = e;
    }
  if (worst > tolerance) std::printf("scaled %ux%u -> %ux%u filter %u: worst %d\n", sw, sh, dw, dh, filter, worst);
  CHECK(worst <= tolerance);
}

static void testClipAndBlend() {
  const uint32_t W = 50, H = 40;
  std::vector<unsigned char> bottomMem, topMem;
  Surface bottom = makeBgra(W, H, W * 4, bottomMem, false);
  Surface top = makeBgra(30, 20, 30 * 4, topMem, true);
  std::vector<uint32_t> out(W * H, 0);
  CHECK(draw(out, W, H, bgraLayer(bottom, 0, 0, W, H, 0, 0, W, H, 1, false)) == W * H);
  std::vector<uint32_t> before = out;
  // The top layer half off the top left corner and past the right edge at once is impossible (it is narrower), so two cases.
  GuampComposeLayer a = bgraLayer(top, 0, 0, 30, 20, -10, -5, 30, 20, 1, true);
  CHECK(draw(out, W, H, a) == 20 * 15);
  bool ok = true;
  for (uint32_t y = 0; y < H; y++)
    for (uint32_t x = 0; x < W; x++) {
      uint32_t expect = before[y * W + x];
      if (x < 20 && y < 15) expect = refOver(top.at(x + 10, y + 5), expect);
      ok = ok && maxDiff(out[y * W + x], expect) <= 1;
    }
  CHECK(ok);
  out = before;
  GuampComposeLayer b = bgraLayer(top, 0, 0, 30, 20, 35, 30, 30, 20, 1, true);
  CHECK(draw(out, W, H, b) == 15 * 10);
  ok = true;
  for (uint32_t y = 0; y < H; y++)
    for (uint32_t x = 0; x < W; x++) {
      uint32_t expect = before[y * W + x];
      if (x >= 35 && y >= 30) expect = refOver(top.at(x - 35, y - 30), expect);
      ok = ok && maxDiff(out[y * W + x], expect) <= 1;
    }
  CHECK(ok);
  out = before;
  CHECK(draw(out, W, H, bgraLayer(top, 0, 0, 30, 20, 60, 0, 30, 20, 1, true)) == 0);   // entirely off the output
  CHECK(draw(out, W, H, bgraLayer(top, 0, 0, 30, 20, -40, 0, 30, 20, 1, true)) == 0);
  CHECK(out == before);
}

static void testInvalid() {
  const uint32_t W = 16, H = 16;
  std::vector<unsigned char> mem;
  Surface s = makeBgra(10, 10, 48, mem, false);
  std::vector<uint32_t> out(W * H, 7);
  CHECK(draw(out, W, H, bgraLayer(s, 1, 0, 10, 10, 0, 0, 10, 10, 1, false)) == -1);   // source past the surface width
  CHECK(draw(out, W, H, bgraLayer(s, 0, 0, 0, 10, 0, 0, 10, 10, 1, false)) == -1);    // empty source
  GuampComposeLayer l = bgraLayer(s, 0, 0, 10, 10, 0, 0, 10, 10, 1, false);
  l.bytes -= 1;   // the last row is one byte short
  CHECK(draw(out, W, H, l) == -1);
  l = bgraLayer(s, 0, 0, 10, 10, 0, 0, 10, 10, 1, false);
  l.stride = 36;  // rows narrower than the rectangle
  CHECK(draw(out, W, H, l) == -1);
  l = bgraLayer(s, 0, 0, 10, 10, 0, 0, 10, 10, 1, false);
  std::vector<uint32_t> taps(5);
  std::vector<unsigned char> line(44);
  CHECK(guampComposeDraw(out.data(), W, H, l, taps.data(), taps.size(), line.data(), line.size(), 0, 0, W, H) == -1);   // too few taps
  taps.resize(20); line.resize(40);
  CHECK(guampComposeDraw(out.data(), W, H, l, taps.data(), taps.size(), line.data(), line.size(), 0, 0, W, H) == -1);   // line too short
  for (uint32_t v : out) CHECK(v == 7);
}

// Reference Y'CbCr conversion.
static uint32_t refYuv(int y, int cb, int cr, bool full, bool bt709) {
  double kr = bt709 ? 0.2126 : 0.299, kb = bt709 ? 0.0722 : 0.114, kg = 1 - kr - kb;
  double Y = full ? y / 255.0 : (y - 16) / 219.0, Pb = full ? (cb - 128) / 255.0 : (cb - 128) / 224.0;
  double Pr = full ? (cr - 128) / 255.0 : (cr - 128) / 224.0;
  double r = Y + 2 * (1 - kr) * Pr, b = Y + 2 * (1 - kb) * Pb, g = (Y - kr * r - kb * b) / kg;
  auto c = [](double v) { v = std::lround(v * 255); return (uint32_t)(v < 0 ? 0 : v > 255 ? 255 : v); };
  return pix(c(b), c(g), c(r), 255);
}
static void testYuv() {
  int worst = 0;
  for (int t = 0; t < 4; t++)
    for (int n = 0; n < 20000; n++) {
      int y = (t < 2 ? 16 + rnd() % 220 : rnd() & 255), cb = (t < 2 ? 16 + rnd() % 225 : rnd() & 255), cr = (t < 2 ? 16 + rnd() % 225 : rnd() & 255);
      int e = maxDiff(guampComposeYuv(y, cb, cr, kGuampComposeYuv[t]), refYuv(y, cb, cr, t >= 2, t & 1));
      if (e > worst) worst = e;
    }
  if (worst > 2) std::printf("yuv worst %d\n", worst);
  CHECK(worst <= 2);
  CHECK(guampComposeYuv(235, 128, 128, kGuampComposeYuv[0]) == pix(255, 255, 255, 255));
  CHECK(guampComposeYuv(16, 128, 128, kGuampComposeYuv[1]) == pix(0, 0, 0, 255));

  // A 4:2:0 surface: luma 33 x 21 (odd sizes), padded rows, chroma plane after the luma plane; a source rectangle at an odd column.
  const uint32_t w = 33, h = 21, ls = 40, cs = 36, rows = h, crows = (h + 1) / 2;
  std::vector<unsigned char> mem(ls * rows + 8 + cs * (crows - 1) + ((w + 1) / 2) * 2);
  for (auto &b : mem) b = rnd() & 255;
  GuampComposeLayer l = {};
  l.base = mem.data(); l.bytes = mem.size(); l.kind = 1; l.matrix = 0; l.width = w; l.height = h;
  l.offset = 0; l.stride = ls; l.chromaOffset = ls * rows + 8; l.chromaStride = cs;
  l.sx = 3; l.sy = 1; l.sw = 30; l.sh = 20; l.dx = 2; l.dy = 3; l.dw = 45; l.dh = 30; l.blend = true;   // blend is ignored: opaque
  const uint32_t W = 50, H = 40;
  std::vector<uint32_t> out(W * H, 0x55555555u);
  CHECK(draw(out, W, H, l) == 45 * 30);
  bool ok = true;
  for (uint32_t oy = 0; oy < H; oy++)
    for (uint32_t ox = 0; ox < W; ox++) {
      uint32_t v = out[oy * W + ox];
      if (ox < 2 || ox >= 47 || oy < 3 || oy >= 33) { ok = ok && v == 0x55555555u; continue; }
      int x0, x1, y0, y1;
      double fx = refAxis(ox - 2, 45, 30, &x0, &x1), fy = refAxis(oy - 3, 30, 20, &y0, &y1);
      if (std::floor(fx * 256 + 0.5) >= 128 && x1 != x0) x0 = x1;
      if (std::floor(fy * 256 + 0.5) >= 128 && y1 != y0) y0 = y1;
      uint32_t col = 3 + x0, row = 1 + y0;
      int Y = mem[row * ls + col], cb = mem[l.chromaOffset + (row / 2) * cs + (col & ~1u)], cr = mem[l.chromaOffset + (row / 2) * cs + (col & ~1u) + 1];
      ok = ok && v == guampComposeYuv(Y, cb, cr, kGuampComposeYuv[0]);
    }
  CHECK(ok);
  l.chromaStride = 30;   // narrower than the chroma of the rectangle
  CHECK(draw(out, W, H, l) == -1);
  l.chromaStride = cs; l.bytes -= 1;   // the chroma plane one byte short
  CHECK(draw(out, W, H, l) == -1);
}

// A redraw clipped to a rectangle equals the whole draw inside it and leaves the rest alone; the union of rectangles.
static void testClip() {
  const uint32_t W = 96, H = 80;
  std::vector<unsigned char> mem;
  Surface s = makeBgra(100, 83, 100 * 4, mem, true);
  GuampComposeLayer l = bgraLayer(s, 0, 0, 100, 83, -3, 2, W + 5, H - 4, 1, true);
  std::vector<uint32_t> base(W * H), whole(W * H), part(W * H);
  for (auto &v : base) v = pix(rnd() & 255, rnd() & 255, rnd() & 255, 255);
  whole = base; part = base;
  CHECK(draw(whole, W, H, l) > 0);
  CHECK(drawClip(part, W, H, l, 17, 9, 61, 50) == 44 * 41);
  bool ok = true;
  for (uint32_t y = 0; y < H; y++)
    for (uint32_t x = 0; x < W; x++) {
      bool in = x >= 17 && x < 61 && y >= 9 && y < 50;
      ok = ok && part[y * W + x] == (in ? whole[y * W + x] : base[y * W + x]);
    }
  CHECK(ok);
  part = base;
  CHECK(drawClip(part, W, H, l, -20, -20, 200, 200) > 0);   // the clip is bounded by the output
  CHECK(part == whole);
  CHECK(drawClip(part, W, H, l, 50, 50, 50, 60) == 0);      // empty clip
  int64_t r[4] = {0, 0, 0, 0};
  guampComposeUnion(r, 10, 10, 5, 5, W, H);
  CHECK(r[0] == 10 && r[1] == 10 && r[2] == 15 && r[3] == 15);
  guampComposeUnion(r, -4, 30, 8, 100, W, H);
  CHECK(r[0] == 0 && r[1] == 10 && r[2] == 15 && r[3] == (int64_t)H);
  guampComposeUnion(r, 200, 0, 5, 5, W, H);   // off the output: unchanged
  CHECK(r[0] == 0 && r[1] == 10 && r[2] == 15 && r[3] == (int64_t)H);
}

static void testFill() {
  std::vector<uint32_t> a(13, 0), b(14, 0);
  guampComposeFill(a.data() + 1, 12, 0xff102030u);   // misaligned start
  guampComposeFill(b.data(), 14, 0xff000000u);
  CHECK(a[0] == 0);
  for (size_t i = 1; i < a.size(); i++) CHECK(a[i] == 0xff102030u);
  for (uint32_t v : b) CHECK(v == 0xff000000u);
}

int main() {
  testOver();
  testUnscaled();
  testScaled(750, 1666, 720, 1600, 1, 2);   // the canvas onto the panel (Display Zoom)
  testScaled(750, 1666, 720, 1600, 0, 0);
  testScaled(64, 36, 720, 405, 1, 2);       // a small video upscaled
  testScaled(31, 17, 31, 17, 1, 0);         // identity
  testScaled(97, 53, 40, 30, 1, 2);         // more than 2:1 down
  testClipAndBlend();
  testInvalid();
  testYuv();
  testClip();
  testFill();
  std::printf("compose core: %s\n", failures ? "FAIL" : "PASS");
  return failures ? 1 : 0;
}
