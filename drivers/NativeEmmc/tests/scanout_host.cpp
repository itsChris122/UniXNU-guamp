// Host test of GuampScanoutCore.h (run-scanout-tests.sh): the differential, double-buffered scanout copy must leave every
// buffer it draws bit-identical to a full guampScalePixels-style copy of the current source (canvas 750x1666 -> 720x1600,
// both filters, and 1:1), after any sequence of partial changes, buffer alternation, catch-ups and invalidations.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "../GuampScanoutCore.h"
// guampLerp, guampLerp2 and guampTap, extracted from scripts/guamp-canvas-variant.py by the runner.
#include VARIANT_TAPS

static int failures = 0;
#define CHECK(c, ...) do { if (!(c)) { ++failures; printf("FAIL %s:%d ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint64_t rng = 0x1234567;
static uint32_t rnd() { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (uint32_t)rng; }

// guampScalePixels of the variant (wide path), on host memory: the reference.
static void reference(uint8_t *dst, uint64_t dstStride, const uint8_t *src, uint64_t srcStride, uint32_t sw, uint32_t sh, uint32_t w,
                      uint32_t h, const uint32_t *columns, const uint32_t *rows)
{
  std::vector<uint8_t> blended((sw + 2) * 4);
  for (uint32_t y = 0; y < h; ++y) {
    if (!columns) { memcpy(dst + y * dstStride, src + y * srcStride, w * 4); continue; }
    const uint32_t row = rows[y] >> 9, weight = rows[y] & 511;
    const uint8_t *first = src + (uint64_t)row * srcStride, *line = first;
    if (weight) {
      const uint8_t *second = first + srcStride;
      for (uint32_t x = 0; x + 2 <= sw; x += 2) {
        uint64_t a, b; memcpy(&a, first + x * 4, 8); memcpy(&b, second + x * 4, 8);
        const uint64_t v = guampLerp2(a, b, weight); memcpy(&blended[x * 4], &v, 8);
      }
      if (sw & 1) {
        const uint64_t at = (sw - 1) * 4; uint32_t a, b; memcpy(&a, first + at, 4); memcpy(&b, second + at, 4);
        const uint32_t v = guampLerp(a, b, weight); memcpy(&blended[at], &v, 4);
      }
      line = blended.data();
    }
    for (uint32_t x = 0; x < w; ++x) {
      const uint32_t tap = columns[x]; const uint64_t at = (uint64_t)(tap >> 9) * 4; const uint32_t wt = tap & 511;
      uint32_t a; memcpy(&a, line + at, 4);
      if (wt) { uint32_t b; memcpy(&b, line + at + 4, 4); a = guampLerp(a, b, wt); }
      memcpy(dst + y * dstStride + x * 4, &a, 4);
    }
  }
}

static void bulk(volatile void *to, const void *from, gscan::u64 bytes) { memcpy((void *)to, from, bytes); }

static void run(uint32_t sw, uint32_t sh, uint32_t w, uint32_t h, int scaled, unsigned filter, bool useBulk, unsigned frames)
{
  std::vector<uint32_t> columns(w), rows(h);
  if (scaled) {
    for (uint32_t x = 0; x < w; ++x) columns[x] = guampTap(x, w, sw, filter);
    for (uint32_t y = 0; y < h; ++y) rows[y] = guampTap(y, h, sh, filter);
  }
  const uint64_t srcStride = (sw * 4 + 63) & ~63ull, dstStride = w * 4;   // 2880 like the panel
  std::vector<uint8_t> src(srcStride * sh), ref(dstStride * h), buf[2] = {std::vector<uint8_t>(dstStride * h + 16),
                                                                          std::vector<uint8_t>(dstStride * h + 16)};
  for (auto &b : src) b = (uint8_t)rnd();
  gscan::State s;
  const uint64_t bytes = gscan::bytesFor(sw, sh, w, h);
  std::vector<uint8_t> mem(bytes);
  CHECK(gscan::layout(s, mem.data(), bytes, sw, sh, w, h, scaled ? columns.data() : nullptr, scaled ? rows.data() : nullptr),
        "layout %ux%u", sw, sh);
  // Every band's output range must cover exactly the outputs that read it.
  for (uint32_t x = 0; x < w; ++x) {
    const uint32_t i = scaled ? columns[x] >> 9 : x, wt = scaled ? columns[x] & 511 : 0;
    CHECK(s.xlo[i >> 6] <= x && x < s.xhi[i >> 6], "band range x %u", x);
    if (wt) CHECK(s.xlo[(i + 1) >> 6] <= x && x < s.xhi[(i + 1) >> 6], "band range x+1 %u", x);
  }
  unsigned t = 0;
  uint64_t drawn = 0;
  for (unsigned f = 0; f < frames; ++f) {
    const unsigned kind = rnd() % 8;
    if (kind == 0) {
      // nothing changes
    } else if (kind == 1) {
      for (auto &b : src) b = (uint8_t)rnd();
    } else if (kind == 2) {
      // one word
      const uint32_t y = rnd() % sh, x = rnd() % sw; src[y * srcStride + x * 4 + rnd() % 4] ^= 1 + rnd() % 255;
    } else {
      const uint32_t rw = 1 + rnd() % (sw < 80 ? sw : 80), rh = 1 + rnd() % (sh < 80 ? sh : 80), rx = rnd() % (sw - rw + 1), ry = rnd() % (sh - rh + 1);
      for (uint32_t y = ry; y < ry + rh; ++y)
        for (uint32_t x = rx; x < rx + rw && x < sw; ++x) for (unsigned c = 0; c < 4; ++c) src[y * srcStride + x * 4 + c] = (uint8_t)rnd();
    }
    if (rnd() % 23 == 0) { s.bufValid[0] = s.bufValid[1] = false; }   // an external write (blank, GPU) or a periodic redraw
    const bool diff = rnd() % 17 != 0;
    gscan::scan(s, src.data(), srcStride, diff);
    gscan::accumulate(s);
    const bool flip = (f & 1) && rnd() % 3;
    if (flip) t ^= 1;
    gscan::draw(s, t, buf[t].data(), dstStride, src.data(), srcStride, useBulk ? bulk : nullptr);
    drawn += s.pixelsDrawn;
    reference(ref.data(), dstStride, src.data(), srcStride, sw, sh, w, h, scaled ? columns.data() : nullptr, scaled ? rows.data() : nullptr);
    CHECK(!memcmp(ref.data(), buf[t].data(), ref.size()), "frame %u buffer %u differs (%ux%u scaled %d filter %u)", f, t, sw, sh,
          scaled, filter);
    if (rnd() % 5 == 0) {
      // idle return: the other buffer catches up from this one
      gscan::catchUp(s, t ^ 1, buf[t ^ 1].data(), buf[t].data(), dstStride);
      CHECK(!memcmp(ref.data(), buf[t ^ 1].data(), ref.size()), "catch-up frame %u", f);
      t ^= 1;
    }
    if (failures > 10) return;
  }
  printf("%ux%u -> %ux%u scaled %d filter %u bulk %d: %u frames, %.1f%% of the pixels drawn\n", sw, sh, w, h, scaled, filter, useBulk,
         frames, 100.0 * (double)drawn / ((double)w * h * frames));
}

int main()
{
  // Unchanged source after a full draw: nothing to draw.
  {
    std::vector<uint32_t> c(720), r(1600);
    for (uint32_t x = 0; x < 720; ++x) c[x] = guampTap(x, 720, 750, 1);
    for (uint32_t y = 0; y < 1600; ++y) r[y] = guampTap(y, 1600, 1666, 1);
    std::vector<uint8_t> src(3008 * 1666, 7), dst(2880 * 1600), mem(gscan::bytesFor(750, 1666, 720, 1600));
    gscan::State s;
    CHECK(gscan::layout(s, mem.data(), mem.size(), 750, 1666, 720, 1600, c.data(), r.data()), "layout");
    CHECK(gscan::scan(s, src.data(), 3008, true) == 1666, "first scan all rows");
    gscan::accumulate(s); gscan::draw(s, 0, dst.data(), 2880, src.data(), 3008, nullptr);
    CHECK(s.rowsDrawn == 1600 && s.pixelsDrawn == 720ull * 1600, "first draw whole");
    CHECK(gscan::scan(s, src.data(), 3008, true) == 0, "second scan nothing");
    gscan::accumulate(s); gscan::draw(s, 0, dst.data(), 2880, src.data(), 3008, nullptr);
    CHECK(s.rowsDrawn == 0, "second draw nothing");
    src[3008 * 800 + 4 * 375] ^= 1;   // one pixel in the middle
    CHECK(gscan::scan(s, src.data(), 3008, true) == 1, "one row");
    gscan::accumulate(s); gscan::draw(s, 0, dst.data(), 2880, src.data(), 3008, nullptr);
    CHECK(s.rowsDrawn <= 2 && s.pixelsDrawn <= 2 * 64, "one pixel: rows %llu px %llu", (unsigned long long)s.rowsDrawn,
          (unsigned long long)s.pixelsDrawn);
    printf("one pixel changed: %llu rows, %llu pixels, box %u,%u-%u,%u\n", (unsigned long long)s.rowsDrawn,
           (unsigned long long)s.pixelsDrawn, s.box[0], s.box[1], s.box[2], s.box[3]);
    // Bad taps are refused.
    c[5] = (749u << 9) | 3;
    CHECK(!gscan::layout(s, mem.data(), mem.size(), 750, 1666, 720, 1600, c.data(), r.data()), "tap past the end refused");
    CHECK(!gscan::layout(s, mem.data(), mem.size() - 64, 750, 1666, 720, 1600, nullptr, nullptr), "short memory refused");
    CHECK(!gscan::layout(s, mem.data(), mem.size(), 750, 1666, 720, 1600, nullptr, nullptr), "unscaled size mismatch refused");
  }
  run(750, 1666, 720, 1600, 1, 1, false, 120);
  run(750, 1666, 720, 1600, 1, 1, true, 80);
  run(750, 1666, 720, 1600, 1, 0, true, 60);
  run(720, 1600, 720, 1600, 0, 1, true, 80);
  run(721, 999, 720, 640, 1, 1, false, 60);
  run(100, 37, 64, 36, 1, 1, true, 200);
  if (failures) { printf("%d failures\n", failures); return 1; }
  printf("scanout core: all checks passed\n");
  return 0;
}
