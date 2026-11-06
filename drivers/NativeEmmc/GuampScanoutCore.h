// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp scanout core (no kernel dependency; host test tests/scanout_host.cpp): the copy of a swapped frame into a scanout
// buffer, limited to what changed.
//
// - Each source row is hashed in bands of 64 pixels (two multiply-xor lanes; one changed word always changes the hash, two
//   or more collide with probability ~2^-64; the kernel side also redraws everything every guamp-copy-diff-full frames).
// - An output row depends on its source row (and the next one when its row tap has a weight); an output column on its
//   source column (and the next one). A changed band therefore maps to a fixed output column range (xlo/xhi per band).
// - Each scanout buffer keeps the band mask of every output row that it still has to redraw (pending), so a buffer that
//   missed frames (double buffering: every other frame) gets the union of what changed since it was last drawn.
// - The scaler is the arithmetic of guampScalePixels (scripts/guamp-canvas-variant.py: guampLerp/guampLerp2/guampTap,
//   premultiplied BGRA, 8-bit weights), bit-exact, but only over the columns of the span that is drawn; output pixels go to a
//   cached staging line first and then to the (write-combining or device) scanout in aligned 8/16-byte stores.
#ifndef GUAMP_SCANOUT_CORE_H
#define GUAMP_SCANOUT_CORE_H

namespace gscan {
typedef unsigned long long u64;
typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

constexpr u32 kBandShift = 6;                 // 64 source pixels per band
constexpr u32 kMaxBands = 32;                 // up to 2048 source pixels per row
constexpr u32 kMaxWidth = 2048, kMaxHeight = 4096;
inline u32 bandsFor(u32 w) { return (w + 63) >> kBandShift; }

// guampLerp / guampLerp2 of the canvas variant (same rounding; per channel the two are identical, see the host test).
inline u32 lerp(u32 a, u32 b, u32 w)
{
  const u32 v = 256 - w;
  const u32 rb = ((((a & 0x00ff00ffU) * v + (b & 0x00ff00ffU) * w) + 0x00800080U) >> 8) & 0x00ff00ffU;
  const u32 ga = ((((a >> 8) & 0x00ff00ffU) * v + ((b >> 8) & 0x00ff00ffU) * w) + 0x00800080U) & 0xff00ff00U;
  return rb | ga;
}
inline u32 sample(const u8 *line, u32 tap)
{
  const u64 at = (u64)(tap >> 9) * 4;
  const u32 weight = tap & 511;
  u32 a;
  __builtin_memcpy(&a, line + at, 4);
  if (!weight) return a;
  u32 b;
  __builtin_memcpy(&b, line + at + 4, 4);
  return lerp(a, b, weight);
}

inline u64 mix(u64 h, u64 w)
{
  h ^= w;
  h *= 0x9E3779B97F4A7C15ull;   // odd: a bijection of h for a fixed w
  return h ^ (h >> 29);          // xorshift: a bijection too
}
// The band hashes of one source row into h[0..bands); returns the mask of the bands whose hash changed.
inline u32 hashRow(const u8 *row, u32 width, u64 *h)
{
  u32 changed = 0;
  const u32 bands = bandsFor(width);
  for (u32 b = 0; b < bands; ++b) {
    const u32 x0 = b << kBandShift, x1 = x0 + 64 < width ? x0 + 64 : width;
    const u8 *p = row + (u64)x0 * 4;
    const u32 bytes = (x1 - x0) * 4;
    u64 a = 0x243F6A8885A308D3ull ^ b, c = 0x13198A2E03707344ull;
    u32 i = 0;
    for (; i + 16 <= bytes; i += 16) {
      u64 u, v;
      __builtin_memcpy(&u, p + i, 8);
      __builtin_memcpy(&v, p + i + 8, 8);
      a = mix(a, u);
      c = mix(c, v);
    }
    for (; i + 4 <= bytes; i += 4) {
      u32 u;
      __builtin_memcpy(&u, p + i, 4);
      a = mix(a, u);
    }
    const u64 value = a ^ (c * 0xC2B2AE3D27D4EB4Full);
    if (value != h[b]) {
      h[b] = value;
      changed |= 1u << b;
    }
  }
  return changed;
}

// Aligned stores of `bytes` (multiple of 4) from the cached `from` into the scanout `to` (4-byte aligned). wide: two 8-byte
// stores per 16 bytes (the compiler may pair them); a store function (NEON in the kernel) can take the 64-byte body.
typedef void (*BulkStore)(volatile void *to, const void *from, u64 bytes);   // bytes: multiple of 64, both 16-byte aligned
inline void storeSpan(volatile u8 *to, const u8 *from, u64 bytes, BulkStore bulk)
{
  u64 i = 0;
  if (((u64)(to) & 4) && bytes >= 4) {
    u32 v;
    __builtin_memcpy(&v, from, 4);
    *reinterpret_cast<volatile u32 *>(to) = v;
    i = 4;
  }
  if (bulk && !(((u64)(to + i)) & 15) && !(((u64)(from + i)) & 15) && bytes - i >= 64) {
    const u64 body = (bytes - i) & ~63ull;
    bulk(to + i, from + i, body);
    i += body;
  }
  for (; i + 16 <= bytes; i += 16) {
    u64 a, b;
    __builtin_memcpy(&a, from + i, 8);
    __builtin_memcpy(&b, from + i + 8, 8);
    reinterpret_cast<volatile u64 *>(to + i)[0] = a;
    reinterpret_cast<volatile u64 *>(to + i)[1] = b;
  }
  if (i + 8 <= bytes) {
    u64 a;
    __builtin_memcpy(&a, from + i, 8);
    *reinterpret_cast<volatile u64 *>(to + i) = a;
    i += 8;
  }
  if (i + 4 <= bytes) {
    u32 a;
    __builtin_memcpy(&a, from + i, 4);
    *reinterpret_cast<volatile u32 *>(to + i) = a;
  }
}

// The working memory of one geometry (one block, carved by layout()).
struct State {
  u32 srcW = 0, srcH = 0, dstW = 0, dstH = 0, bands = 0;
  const u32 *columns = nullptr, *rows = nullptr;   // the variant's scaler taps (null: unscaled 1:1)
  u64 *hash = nullptr;                             // srcH * bands
  u32 *srcMask = nullptr;                          // srcH: bands changed in this frame
  u32 *pending[2] = {nullptr, nullptr};            // dstH each: bands still to redraw in buffer 0 / 1
  u8 *line = nullptr;                              // blended source row (srcW + 2 pixels)
  u8 *stage = nullptr;                             // staged output row (dstW + 4 pixels, 16-byte aligned)
  u16 xlo[kMaxBands] = {}, xhi[kMaxBands] = {};    // output column range [xlo, xhi) of each source band
  bool hashValid = false;
  bool bufValid[2] = {false, false};
  // Per draw: output rows and pixels written, bounding box of the drawn area (x0, y0, x1, y1; x1 <= x0 when nothing).
  u64 rowsDrawn = 0, pixelsDrawn = 0;
  u32 box[4] = {0, 0, 0, 0};
};

inline u64 align16(u64 v) { return (v + 15) & ~15ull; }
inline u64 bytesFor(u32 srcW, u32 srcH, u32 dstW, u32 dstH)
{
  return align16((u64)srcH * bandsFor(srcW) * 8) + align16((u64)srcH * 4) + 2 * align16((u64)dstH * 4) +
         align16(((u64)srcW + 2) * 4) + align16(((u64)dstW + 4) * 4) + 16;
}

// Checks the geometry and the taps, carves `memory` (bytesFor() bytes) and computes the band column ranges. Everything is
// invalid afterwards (the first frame is drawn whole).
inline bool layout(State &s, void *memory, u64 memoryBytes, u32 srcW, u32 srcH, u32 dstW, u32 dstH, const u32 *columns,
                   const u32 *rows)
{
  s = State{};
  if (!memory || !srcW || !srcH || !dstW || !dstH || srcW > kMaxWidth || srcH > kMaxHeight || dstW > kMaxWidth ||
      dstH > kMaxHeight || memoryBytes < bytesFor(srcW, srcH, dstW, dstH))
    return false;
  if (!!columns != !!rows) return false;
  if (!columns && (srcW != dstW || srcH != dstH)) return false;
  if (columns) {
    for (u32 x = 0; x < dstW; ++x) {
      const u32 i = columns[x] >> 9, w = columns[x] & 511;
      if (w > 256 || i >= srcW || (w && i + 1 >= srcW)) return false;
    }
    for (u32 y = 0; y < dstH; ++y) {
      const u32 i = rows[y] >> 9, w = rows[y] & 511;
      if (w > 256 || i >= srcH || (w && i + 1 >= srcH)) return false;
    }
  }
  u8 *p = reinterpret_cast<u8 *>(align16((u64)memory));
  s.srcW = srcW; s.srcH = srcH; s.dstW = dstW; s.dstH = dstH; s.bands = bandsFor(srcW);
  s.columns = columns; s.rows = rows;
  s.hash = reinterpret_cast<u64 *>(p); p += align16((u64)srcH * s.bands * 8);
  s.srcMask = reinterpret_cast<u32 *>(p); p += align16((u64)srcH * 4);
  s.pending[0] = reinterpret_cast<u32 *>(p); p += align16((u64)dstH * 4);
  s.pending[1] = reinterpret_cast<u32 *>(p); p += align16((u64)dstH * 4);
  s.line = p; p += align16(((u64)srcW + 2) * 4);
  s.stage = p;
  for (u32 b = 0; b < s.bands; ++b) { s.xlo[b] = (u16)dstW; s.xhi[b] = 0; }
  for (u32 x = 0; x < dstW; ++x) {
    const u32 i = columns ? columns[x] >> 9 : x, w = columns ? columns[x] & 511 : 0;
    const u32 b0 = i >> kBandShift, b1 = w ? (i + 1) >> kBandShift : b0;
    for (u32 b = b0; b <= b1; ++b) {
      if (x < s.xlo[b]) s.xlo[b] = (u16)x;
      if (x + 1 > s.xhi[b]) s.xhi[b] = (u16)(x + 1);
    }
  }
  for (u32 i = 0; i < dstH; ++i) s.pending[0][i] = s.pending[1][i] = 0;
  return true;
}

inline u32 allBands(const State &s) { return s.bands >= 32 ? 0xffffffffu : (1u << s.bands) - 1; }

// Scans the source: srcMask[r] = the bands of row r that changed since the previous scan (all of them when diff is off or the
// hashes are not valid). Returns the number of source rows with a change.
inline u32 scan(State &s, const u8 *src, u64 srcStride, bool diff)
{
  u32 changedRows = 0;
  const bool compare = diff && s.hashValid;
  for (u32 r = 0; r < s.srcH; ++r) {
    u32 m = 0;
    if (diff) m = hashRow(src + (u64)r * srcStride, s.srcW, s.hash + (u64)r * s.bands);
    if (!compare) m = allBands(s);
    s.srcMask[r] = m;
    if (m) ++changedRows;
  }
  s.hashValid = diff;
  return changedRows;
}

// Adds this frame's changes (srcMask) to the pending masks of both buffers; a buffer that is not valid gets everything.
inline void accumulate(State &s)
{
  const u32 all = allBands(s);
  for (u32 y = 0; y < s.dstH; ++y) {
    u32 m;
    if (s.rows) {
      const u32 r = s.rows[y] >> 9, w = s.rows[y] & 511;
      m = s.srcMask[r] | (w ? s.srcMask[r + 1] : 0);
    } else {
      m = s.srcMask[y];
    }
    for (unsigned t = 0; t < 2; ++t) s.pending[t][y] = s.bufValid[t] ? (s.pending[t][y] | m) : all;
  }
}

inline void spanOf(const State &s, u32 mask, u32 *x0, u32 *x1)
{
  u32 lo = s.dstW, hi = 0;
  for (u32 b = 0; b < s.bands; ++b)
    if (mask & (1u << b)) {
      if (s.xlo[b] < lo) lo = s.xlo[b];
      if (s.xhi[b] > hi) hi = s.xhi[b];
    }
  *x0 = lo; *x1 = hi;
}

inline void boxAdd(State &s, u32 x0, u32 x1, u32 y)
{
  if (s.box[2] <= s.box[0]) { s.box[0] = x0; s.box[2] = x1; s.box[1] = y; s.box[3] = y + 1; return; }
  if (x0 < s.box[0]) s.box[0] = x0;
  if (x1 > s.box[2]) s.box[2] = x1;
  s.box[3] = y + 1;
}

// Draws the pending rows of buffer t from the source into `to` (stride toStride, at least dstH rows of dstW pixels) and
// clears its pending masks; the buffer is valid afterwards.
inline void draw(State &s, unsigned t, volatile u8 *to, u64 toStride, const u8 *src, u64 srcStride, BulkStore bulk)
{
  s.rowsDrawn = s.pixelsDrawn = 0;
  s.box[0] = s.box[1] = s.box[2] = s.box[3] = 0;
  for (u32 y = 0; y < s.dstH; ++y) {
    const u32 mask = s.pending[t][y];
    if (!mask) continue;
    s.pending[t][y] = 0;
    u32 x0, x1;
    spanOf(s, mask, &x0, &x1);
    if (x1 <= x0) continue;
    volatile u8 *out = to + (u64)y * toStride + (u64)x0 * 4;
    if (!s.rows) {
      storeSpan(out, src + (u64)y * srcStride + (u64)x0 * 4, (u64)(x1 - x0) * 4, bulk);
    } else {
      const u32 row = s.rows[y] >> 9, weight = s.rows[y] & 511;
      const u8 *first = src + (u64)row * srcStride, *line = first;
      if (weight) {
        // Only the source columns this span reads: [first tap, last tap + 1].
        const u32 s0 = s.columns[x0] >> 9;
        u32 s1 = (s.columns[x1 - 1] >> 9) + 1;
        if (s1 >= s.srcW) s1 = s.srcW - 1;
        const u8 *second = first + srcStride;
        for (u32 x = s0; x <= s1; ++x) {
          u32 a, b;
          __builtin_memcpy(&a, first + (u64)x * 4, 4);
          __builtin_memcpy(&b, second + (u64)x * 4, 4);
          const u32 v = lerp(a, b, weight);
          __builtin_memcpy(s.line + (u64)x * 4, &v, 4);
        }
        line = s.line;
      }
      // Staged at the same 16-byte phase as the destination so the bulk store can run.
      u8 *stage = s.stage + (((u64)out) & 15);
      for (u32 x = x0; x < x1; ++x) {
        const u32 v = sample(line, s.columns[x]);
        __builtin_memcpy(stage + (u64)(x - x0) * 4, &v, 4);
      }
      storeSpan(out, stage, (u64)(x1 - x0) * 4, bulk);
    }
    s.rowsDrawn += 1;
    s.pixelsDrawn += x1 - x0;
    boxAdd(s, x0, x1, y);
  }
  s.bufValid[t] = true;
}

// 1:1 only (no taps): scan + accumulate + draw of buffer t in one pass, row by row, from one read of the source. Each source row
// is first copied into the cached staging line, then hashed and stored from there: what the hash records is exactly what the
// buffer gets. With scan() and draw() apart (fin52b/fin53, "black bands in the app switcher") the source was read twice, ~3 ms
// and a latch wait apart; a surface that changed in between (the render server reusing a retired surface, the GPU still
// writing a frame) left rows drawn with other pixels than the hashed ones, and later frames equal to the hashed pixels never
// redrew them (horizontal bands until the next full redraw). Here a torn row is at most one frame old: the next frame compares
// against what was really drawn. Returns the number of source rows with a change.
inline u32 scanDraw(State &s, unsigned t, volatile u8 *to, u64 toStride, const u8 *src, u64 srcStride, bool diff, BulkStore bulk)
{
  s.rowsDrawn = s.pixelsDrawn = 0;
  s.box[0] = s.box[1] = s.box[2] = s.box[3] = 0;
  if (s.rows || s.columns || s.srcW != s.dstW || s.srcH != s.dstH) return 0;
  const u32 all = allBands(s);
  const bool compare = diff && s.hashValid;
  const u64 rowBytes = (u64)s.srcW * 4;
  u32 changedRows = 0;
  for (u32 y = 0; y < s.srcH; ++y) {
    volatile u8 *out = to + (u64)y * toStride;
    u8 *stage = s.stage + (((u64)out) & 15);   // same 16-byte phase as the destination row (bulk stores)
    __builtin_memcpy(stage, src + (u64)y * srcStride, rowBytes);
    u32 m = 0;
    if (diff) m = hashRow(stage, s.srcW, s.hash + (u64)y * s.bands);
    if (!compare) m = all;
    s.srcMask[y] = m;
    if (m) ++changedRows;
    for (unsigned b = 0; b < 2; ++b) s.pending[b][y] = s.bufValid[b] ? (s.pending[b][y] | m) : all;
    const u32 mask = s.pending[t][y];
    if (!mask) continue;
    s.pending[t][y] = 0;
    u32 x0, x1;
    spanOf(s, mask, &x0, &x1);
    if (x1 <= x0) continue;
    storeSpan(out + (u64)x0 * 4, stage + (u64)x0 * 4, (u64)(x1 - x0) * 4, bulk);
    s.rowsDrawn += 1;
    s.pixelsDrawn += x1 - x0;
    boxAdd(s, x0, x1, y);
  }
  s.hashValid = diff;
  s.bufValid[t] = true;
  return changedRows;
}

// Brings buffer t up to date from buffer `from` (the other scanout buffer, same layout): its pending rows are copied. Reading a
// write-combining or device buffer is slow, so this is for the small idle catch-up only.
inline void catchUp(State &s, unsigned t, volatile u8 *to, const volatile u8 *from, u64 stride)
{
  s.rowsDrawn = s.pixelsDrawn = 0;
  const u32 all = allBands(s);
  for (u32 y = 0; y < s.dstH; ++y) {
    const u32 mask = s.bufValid[t] ? s.pending[t][y] : all;
    if (!mask) continue;
    s.pending[t][y] = 0;
    u32 x0, x1;
    spanOf(s, mask, &x0, &x1);
    for (u32 x = x0; x < x1; ++x) {
      const u64 at = (u64)y * stride + (u64)x * 4;
      *reinterpret_cast<volatile u32 *>(to + at) = *reinterpret_cast<const volatile u32 *>(from + at);
    }
    if (x1 > x0) { s.rowsDrawn += 1; s.pixelsDrawn += x1 - x0; }
  }
  s.bufValid[t] = true;
}

// Mean luma (0..255, BT.601 integer weights 77/150/29 on premultiplied BGRA) of an 8x8 grid of source pixels: a cheap
// picture fingerprint for the early-frame log (boot logo ~black, lock screen wallpaper brighter). 0 for an empty source.
inline u32 meanLuma(const u8 *src, u64 stride, u32 w, u32 h)
{
  if (!src || !w || !h) return 0;
  u32 sum = 0;
  for (u32 j = 0; j < 8; ++j) {
    const u64 y = (u64)h * (2 * j + 1) / 16;
    for (u32 i = 0; i < 8; ++i) {
      const u64 x = (u64)w * (2 * i + 1) / 16;
      const u8 *p = src + y * stride + x * 4;   // B, G, R, A
      sum += (77u * p[2] + 150u * p[1] + 29u * p[0]) >> 8;
    }
  }
  return sum / 64;
}
}  // namespace gscan
#endif
