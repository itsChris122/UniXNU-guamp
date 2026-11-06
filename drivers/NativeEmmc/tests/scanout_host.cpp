// Host test of GuampScanoutCore.h (run-scanout-tests.sh): the differential, double-buffered scanout copy must leave every
// buffer it draws bit-identical to a full guampScalePixels-style copy of the current source (canvas 750x1666 -> 720x1600,
// both filters, and 1:1), after any sequence of partial changes, buffer alternation, catch-ups and invalidations.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
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

// scanDraw (1:1, one pass): every drawn buffer equals the current source, with alternation and invalidations; and the
// fin53 race: the old two-pass path (scan, then draw re-reading a surface that changed in between) leaves a row wrong for good,
// the one-pass path cannot (what is hashed is what is drawn).
static void runOnePass(uint32_t w, uint32_t h, bool useBulk, unsigned frames)
{
  const uint64_t srcStride = (w * 4 + 63) & ~63ull, dstStride = (w * 4 + 15) & ~15ull;
  std::vector<uint8_t> src(srcStride * h), buf[2] = {std::vector<uint8_t>(dstStride * h + 16, 0), std::vector<uint8_t>(dstStride * h + 16, 0)};
  for (auto &b : src) b = (uint8_t)rnd();
  gscan::State s;
  std::vector<uint8_t> mem(gscan::bytesFor(w, h, w, h));
  CHECK(gscan::layout(s, mem.data(), mem.size(), w, h, w, h, nullptr, nullptr), "layout one pass");
  unsigned t = 0;
  uint64_t drawn = 0;
  for (unsigned f = 0; f < frames; ++f) {
    const unsigned kind = rnd() % 6;
    if (kind == 1) for (auto &b : src) b = (uint8_t)rnd();
    else if (kind >= 2) {
      const uint32_t ry = rnd() % h, rh = 1 + rnd() % (h - ry < 60 ? h - ry : 60), rx = rnd() % w, rw = 1 + rnd() % (w - rx);
      for (uint32_t y = ry; y < ry + rh; ++y) for (uint32_t x = rx; x < rx + rw; ++x) src[y * srcStride + x * 4 + rnd() % 4] ^= 0x5a;
    }
    if (rnd() % 23 == 0) s.bufValid[0] = s.bufValid[1] = false;
    if (rnd() % 31 == 0) s.hashValid = false;
    const bool diff = rnd() % 17 != 0;
    if ((f & 1) && rnd() % 3) t ^= 1;
    gscan::scanDraw(s, t, buf[t].data(), dstStride, src.data(), srcStride, diff, useBulk ? bulk : nullptr);
    drawn += s.pixelsDrawn;
    bool same = true;
    for (uint32_t y = 0; y < h && same; ++y) same = !memcmp(&buf[t][y * dstStride], &src[y * srcStride], w * 4);
    CHECK(same, "one pass frame %u buffer %u differs (%ux%u)", f, t, w, h);
    if (failures > 10) return;
  }
  printf("one pass %ux%u bulk %d: %u frames, %.1f%% of the pixels drawn\n", w, h, useBulk, frames,
         100.0 * (double)drawn / ((double)w * h * frames));
}

static void raceCheck()
{
  const uint32_t w = 128, h = 8;
  const uint64_t stride = w * 4;
  std::vector<uint8_t> a(stride * h, 0x11), buf(stride * h + 16, 0x77), mem(gscan::bytesFor(w, h, w, h));
  gscan::State s;
  // Two passes: frame 1 scans A2 (row 3 changed), the surface is reused (row 3 black) before the draw; frame 2 is A2 again.
  CHECK(gscan::layout(s, mem.data(), mem.size(), w, h, w, h, nullptr, nullptr), "layout race");
  gscan::scan(s, a.data(), stride, true); gscan::accumulate(s); gscan::draw(s, 0, buf.data(), stride, a.data(), stride, nullptr);
  std::vector<uint8_t> a2 = a; a2[3 * stride + 8] = 0x99;
  std::vector<uint8_t> torn = a2; memset(&torn[3 * stride], 0, stride);
  gscan::scan(s, a2.data(), stride, true); gscan::accumulate(s); gscan::draw(s, 0, buf.data(), stride, torn.data(), stride, nullptr);
  gscan::scan(s, a2.data(), stride, true); gscan::accumulate(s); gscan::draw(s, 0, buf.data(), stride, a2.data(), stride, nullptr);
  const bool oldStale = memcmp(&buf[3 * stride], &a2[3 * stride], stride) != 0;
  // One pass: the torn read is what is hashed, so the next frame (A2 again) redraws the row.
  CHECK(gscan::layout(s, mem.data(), mem.size(), w, h, w, h, nullptr, nullptr), "layout race 2");
  gscan::scanDraw(s, 0, buf.data(), stride, a.data(), stride, true, nullptr);
  gscan::scanDraw(s, 0, buf.data(), stride, torn.data(), stride, true, nullptr);
  gscan::scanDraw(s, 0, buf.data(), stride, a2.data(), stride, true, nullptr);
  CHECK(!memcmp(buf.data(), a2.data(), stride * h), "one pass heals the torn row");
  CHECK(s.rowsDrawn == 1, "one pass redraws only the torn row (%llu)", (unsigned long long)s.rowsDrawn);
  printf("race: two passes leave the torn row %s, one pass heals it the next frame\n", oldStale ? "wrong for good" : "(not reproduced)");
  CHECK(oldStale, "the two-pass race should reproduce (test model)");
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
  // Early-frame fingerprint: black 0, white 255, pure green 149, empty source 0, stride honoured.
  {
    std::vector<uint8_t> px(3008 * 1666, 0);
    CHECK(gscan::meanLuma(px.data(), 3008, 750, 1666) == 0, "black luma");
    std::fill(px.begin(), px.end(), 255);
    CHECK(gscan::meanLuma(px.data(), 3008, 750, 1666) == 255, "white luma %u", gscan::meanLuma(px.data(), 3008, 750, 1666));
    for (size_t i = 0; i < px.size(); i += 4) { px[i] = 0; px[i + 1] = 255; px[i + 2] = 0; px[i + 3] = 255; }
    CHECK(gscan::meanLuma(px.data(), 3008, 750, 1666) == 149, "green luma %u", gscan::meanLuma(px.data(), 3008, 750, 1666));
    CHECK(gscan::meanLuma(nullptr, 3008, 750, 1666) == 0 && gscan::meanLuma(px.data(), 3008, 0, 1666) == 0, "empty");
    std::vector<uint8_t> one(4, 255);
    CHECK(gscan::meanLuma(one.data(), 4, 1, 1) == 255, "1x1");
  }
  // guamp-scanout-rowflush: the row prep runs once per source row, in order, before the row is read, with or without diff, and
  // changes nothing in the result.
  {
    static const uint8_t *base; static uint64_t calls, bad, next;
    struct P { static void prep(const uint8_t *row, unsigned long long bytes) {
      if (row != base + next * 3008 || bytes != 750ull * 4) ++bad;
      ++next; ++calls; } };
    std::vector<uint8_t> src(3008 * 1666, 9), mem(gscan::bytesFor(750, 1666, 750, 1666)), mem2(mem.size());
    gscan::State a, b;
    CHECK(gscan::layout(a, mem.data(), mem.size(), 750, 1666, 750, 1666, nullptr, nullptr) &&
          gscan::layout(b, mem2.data(), mem2.size(), 750, 1666, 750, 1666, nullptr, nullptr), "prep layout");
    base = src.data();
    for (int round = 0; round < 3; ++round) {
      if (round == 2) src[3008 * 1000 + 40] ^= 0x10;
      calls = next = 0;
      const unsigned ca = gscan::scan(a, src.data(), 3008, true, P::prep), cb = gscan::scan(b, src.data(), 3008, true);
      CHECK(calls == 1666 && !bad && ca == cb, "prep round %d: calls %llu bad %llu changed %u/%u", round,
            (unsigned long long)calls, (unsigned long long)bad, ca, cb);
      for (unsigned r = 0; r < 1666; ++r) if (a.srcMask[r] != b.srcMask[r]) { CHECK(false, "prep mask row %u", r); break; }
    }
    calls = next = 0;
    gscan::scan(a, src.data(), 3008, false, P::prep);
    CHECK(calls == 1666 && !bad, "prep without diff: calls %llu", (unsigned long long)calls);
    // scanDraw (1:1 in one pass): the same prep, once per row, before the row is read.
    std::vector<uint8_t> out(3008 * 1666, 0);
    for (int round = 0; round < 2; ++round) {
      if (round == 1) src[3008 * 7 + 4] ^= 0x20;
      calls = next = 0;
      const unsigned changed = gscan::scanDraw(a, 0, out.data(), 3008, src.data(), 3008, true, nullptr, P::prep);
      CHECK(calls == 1666 && !bad && (round == 0 || changed == 1), "scanDraw prep round %d: calls %llu changed %u", round,
            (unsigned long long)calls, changed);
    }
    bool same = true;
    for (unsigned r = 0; r < 1666; ++r) same = same && !memcmp(out.data() + r * 3008, src.data() + r * 3008, 3000);
    CHECK(same, "scanDraw prep: slot equals the source");
  }
  // guamp-scanout-flush-ahead (07/11 gpu-frame): the prep runs `ahead` rows early, still once per row, in order, before the row's
  // first read. Simulated stale cache: the "cache" (what scan/scanDraw read) holds old rows, the prep of a row copies the fresh
  // "memory" row into it (clean + invalidate). A read before the prep would hash / draw the stale row.
  {
    static uint8_t *cache; static const uint8_t *memory; static uint64_t calls, bad, next;
    struct P { static void prep(const uint8_t *row, unsigned long long bytes) {
      const uint64_t r = (uint64_t)(row - cache) / 3008;
      if (row != cache + next * 3008 || bytes != 750ull * 4) ++bad;
      memcpy(cache + r * 3008, memory + r * 3008, 3008);
      ++next; ++calls; } };
    const unsigned aheads[] = {0, 1, 8, 64, 1000};
    for (unsigned ai = 0; ai < sizeof(aheads) / sizeof(aheads[0]); ++ai) {
      const unsigned ahead = aheads[ai];
      std::vector<uint8_t> mem(3008 * 1666), stale(mem.size()), st(gscan::bytesFor(750, 1666, 750, 1666)),
          st2(st.size()), out(3008 * 1666, 0), ref(3008 * 1666, 0);
      gscan::State a, b;
      CHECK(gscan::layout(a, st.data(), st.size(), 750, 1666, 750, 1666, nullptr, nullptr) &&
            gscan::layout(b, st2.data(), st2.size(), 750, 1666, 750, 1666, nullptr, nullptr), "ahead layout");
      uint64_t seed = 77 + ahead;
      for (int round = 0; round < 4; ++round) {
        for (int k = 0; k < 40; ++k) {   // a few changed pixels per frame
          seed = seed * 6364136223846793005ull + 1442695040888963407ull;
          const uint64_t at = (seed >> 33) % (3008ull * 1666);
          mem[at] ^= (uint8_t)(1 + (seed >> 20));
        }
        if (round == 0) for (size_t i = 0; i < mem.size(); ++i) mem[i] = (uint8_t)(i * 13 + 5);
        for (size_t i = 0; i < stale.size(); ++i) stale[i] = (uint8_t)(i * 7 + round);   // never the frame's pixels
        cache = stale.data(); memory = mem.data(); calls = next = 0;
        const unsigned ca = gscan::scanDraw(a, round & 1, out.data(), 3008, stale.data(), 3008, true, nullptr, P::prep, ahead);
        const unsigned cb = gscan::scanDraw(b, round & 1, ref.data(), 3008, mem.data(), 3008, true, nullptr);
        CHECK(calls == 1666 && !bad && ca == cb, "ahead %u round %d: calls %llu bad %llu changed %u/%u", ahead, round,
              (unsigned long long)calls, (unsigned long long)bad, ca, cb);
        CHECK(!memcmp(out.data(), ref.data(), out.size()), "ahead %u round %d: slot differs from the fresh frame", ahead, round);
        // scan() (scaled path) with the same prep
        for (size_t i = 0; i < stale.size(); ++i) stale[i] = (uint8_t)(i * 11 + round);
        calls = next = 0;
        gscan::State c; std::vector<uint8_t> st3(st.size());
        gscan::layout(c, st3.data(), st3.size(), 750, 1666, 750, 1666, nullptr, nullptr);
        gscan::scan(c, stale.data(), 3008, true, P::prep, ahead);
        gscan::scan(c, mem.data(), 3008, true);   // same pixels: nothing may change
        bool none = true;
        for (unsigned r = 0; r < 1666; ++r) none = none && !c.srcMask[r];
        CHECK(calls == 1666 && !bad && none, "scan ahead %u round %d: calls %llu bad %llu", ahead, round,
              (unsigned long long)calls, (unsigned long long)bad);
      }
    }
  }
  // Four-lane hash: one changed 32-bit word anywhere in a band always changes that band (and only that band).
  {
    std::vector<uint8_t> row(750 * 4);
    for (size_t i = 0; i < row.size(); ++i) row[i] = (uint8_t)(i * 29 + 3);
    std::vector<gscan::u64> h(gscan::bandsFor(750));
    gscan::hashRow(row.data(), 750, h.data());
    unsigned misses = 0;
    for (unsigned x = 0; x < 750; ++x)
      for (unsigned bit = 0; bit < 32; bit += 7) {
        std::vector<gscan::u64> h2 = h;
        std::vector<uint8_t> r2 = row;
        r2[x * 4 + bit / 8] ^= (uint8_t)(1u << (bit % 8));
        const unsigned m = gscan::hashRow(r2.data(), 750, h2.data());
        if (m != (1u << (x >> 6))) ++misses;
      }
    CHECK(!misses, "four-lane hash: %u single-word changes missed or misplaced", misses);
    // two pixels swapped inside a band (a moving 1-pixel line) must change it too
    std::vector<uint8_t> r3 = row;
    for (int k = 0; k < 4; ++k) { const uint8_t v = r3[10 * 4 + k]; r3[10 * 4 + k] = r3[11 * 4 + k]; r3[11 * 4 + k] = v; }
    std::vector<gscan::u64> h3 = h;
    CHECK(gscan::hashRow(r3.data(), 750, h3.data()) == 1u, "four-lane hash: swapped pixels");
  }
  run(750, 1666, 720, 1600, 1, 1, false, 120);
  run(750, 1666, 720, 1600, 1, 1, true, 80);
  run(750, 1666, 720, 1600, 1, 0, true, 60);
  run(720, 1600, 720, 1600, 0, 1, true, 80);
  run(721, 999, 720, 640, 1, 1, false, 60);
  run(100, 37, 64, 36, 1, 1, true, 200);
  runOnePass(750, 1666, true, 60);
  runOnePass(720, 1600, false, 40);
  runOnePass(97, 33, true, 300);
  raceCheck();
  if (failures) { printf("%d failures\n", failures); return 1; }
  printf("scanout core: all checks passed\n");
  return 0;
}
