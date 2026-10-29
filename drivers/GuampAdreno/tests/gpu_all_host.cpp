// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of NativeAdrenoGpuAll.h (patches/native-adreno/0005-gpu-all-dynamic-windows.patch): the window allocator of guamp-gpu-all,
// the per-name open rate and the limits. Built by tests/run-tests.sh against the patched NativeAdreno copy, with ASan/UBSan and gcc.
#include <stdio.h>
#include <stdlib.h>
#include "NativeAdrenoGpuAll.h"

static int failures;
#define CHECK(c) do {if (!(c)) {printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++failures;}} while (0)

static void windows() {
  NagWindows w;
  w.init(0xC0000000ull);   // 3 GiB aperture: 1.5 GiB above the render server's window, 96 granules
  CHECK(w.granules() == 96);
  CHECK(w.freeBytes() == 0x60000000ull);
  uint64_t s[16], b[16];
  // eight 192 MiB windows fill 1.5 GiB exactly; the ninth gets nothing (minimum 64 MiB)
  for (int i = 0; i < 8; ++i) {
    CHECK(w.take(192ull << 20, 64ull << 20, &s[i], &b[i]));
    CHECK(b[i] == 192ull << 20);
    CHECK(s[i] >= 0x60000000ull && s[i] + b[i] <= 0xC0000000ull && s[i] % NagGranule == 0);
    for (int j = 0; j < i; ++j) CHECK(s[i] + b[i] <= s[j] || s[j] + b[j] <= s[i]);   // disjoint
  }
  uint64_t x = 0, y = 0;
  CHECK(!w.take(192ull << 20, 64ull << 20, &x, &y));
  CHECK(w.freeBytes() == 0);
  // give back two non-adjacent windows: a 64 MiB request takes the best fit, a 256 MiB one the largest run (192 MiB, at least 64)
  w.give(s[2], b[2]);
  w.give(s[5], b[5]);
  CHECK(w.freeBytes() == 384ull << 20);
  CHECK(w.take(256ull << 20, 64ull << 20, &x, &y) && y == 192ull << 20);
  CHECK(w.take(64ull << 20, 64ull << 20, &x, &y) && y == 64ull << 20);
  CHECK(w.take(200ull << 20, 64ull << 20, &x, &y) && y == 128ull << 20);   // the rest of that run
  CHECK(w.freeBytes() == 0);
  // garbage given back changes nothing
  w.give(0x10000000ull, NagGranule);
  w.give(0x60000001ull, NagGranule);
  w.give(0xB0000000ull, 0x20000000ull);
  CHECK(w.freeBytes() == 0);
  // best fit prefers the smaller hole that holds the request
  NagWindows v;
  v.init(0xC0000000ull);
  uint64_t a, ab, c, cb, d, db;
  CHECK(v.take(64ull << 20, 0, &a, &ab));      // [0, 64)
  CHECK(v.take(1024ull << 20, 0, &c, &cb));    // [64, 1088)
  CHECK(v.take(64ull << 20, 0, &d, &db));      // [1088, 1152), 384 MiB left at the top
  v.give(a, ab);                               // 64 MiB hole at the bottom
  uint64_t e, eb;
  CHECK(v.take(48ull << 20, 0, &e, &eb) && e == a && eb == 48ull << 20);   // rounded to 3 granules, in the small hole
  // 2 GiB aperture: 512 MiB above the render server's window
  NagWindows n;
  n.init(0x80000000ull);
  CHECK(n.freeBytes() == 0x20000000ull);
  CHECK(n.take(192ull << 20, 64ull << 20, &x, &y) && n.take(192ull << 20, 64ull << 20, &x, &y));
  CHECK(n.take(192ull << 20, 64ull << 20, &x, &y) && y == 128ull << 20 && x + y == 0x80000000ull);
  CHECK(!n.take(16ull << 20, 0, &x, &y));
}

static void rates() {
  NagRates r;
  for (unsigned i = 0; i < 6; ++i) CHECK(r.admit("AegirPoster", 1000 + i, 6));
  CHECK(!r.admit("AegirPoster", 2000, 6));           // seventh in the same minute
  CHECK(r.admit("KaleidoscopePost", 2000, 6));       // other names are not affected
  CHECK(r.admit("AegirPoster", 61000, 6));           // a minute after the first one
  CHECK(r.admit("x", 0, 0));                         // 0 = no limit
  // more than 64 names: the oldest row is reused and the newest names keep their counts
  char name[20];
  for (unsigned i = 0; i < 100; ++i) {
    snprintf(name, sizeof name, "proc%u", i);
    CHECK(r.admit(name, 100000 + i, 1));
  }
  CHECK(!r.admit("proc99", 100200, 1));
  CHECK(r.admit("proc0", 100200, 1));                // pushed out, starts afresh
}

static void limits() {
  CHECK(nagAllowMap(0, 0, 1));
  CHECK(!nagAllowMap(NagVmObjects - NagVmReservedObjects, 0, 1));
  CHECK(nagAllowMap(100, NagVmSpans - NagVmReservedSpans - 10, 10));
  CHECK(!nagAllowMap(100, NagVmSpans - NagVmReservedSpans - 10, 11));
  const uint64_t mib = 1ull << 20;
  CHECK(nagAllowBytes(0, 0, 64 * mib, 384 * mib, 1024 * mib));
  CHECK(nagAllowBytes(320 * mib, 0, 64 * mib, 384 * mib, 1024 * mib));
  CHECK(!nagAllowBytes(321 * mib, 0, 64 * mib, 384 * mib, 1024 * mib));
  CHECK(!nagAllowBytes(0, 1000 * mib, 64 * mib, 384 * mib, 1024 * mib));
  CHECK(!nagAllowBytes(0, 0, ~0ull, 384 * mib, 1024 * mib));   // no wrap-around
  CHECK(!nagAllowBytes(~0ull, 0, 4096, 384 * mib, 1024 * mib));
}

// patch 0006: the deny list and the table pressure checked at open.
static void admission() {
  const char *list = "SpringBoard, mediaserverd ,com.apple.WebKit.*,ScreenshotServicesService";
  CHECK(nagListed(list, "SpringBoard"));
  CHECK(!nagListed(list, "SpringBoardX"));
  CHECK(!nagListed(list, "Spring"));
  CHECK(nagListed(list, "mediaserverd"));
  CHECK(nagListed(list, "com.apple.WebKit"));    // the 16-character name of com.apple.WebKit.WebContent: a longer prefix covers it
  CHECK(!nagListed(list, "com.apple.WebKi"));
  CHECK(nagListed(list, "ScreenshotServic"));    // a long entry is compared on its first 16 characters
  CHECK(!nagListed(list, "Preferences"));
  CHECK(!nagListed("", "SpringBoard"));
  CHECK(!nagListed(nullptr, "SpringBoard"));
  CHECK(nagListed("*", "anything"));
  CHECK(nagListed("Kaleido*", "KaleidoscopePost"));
  CHECK(!nagListed(",,", "x"));
  CHECK(!nagTablePressure(4000, 60000));
  CHECK(nagTablePressure(NagVmObjects - NagVmReservedObjects, 0));
  CHECK(nagTablePressure(0, NagVmSpans - NagVmReservedSpans));
}

int main() {
  admission();
  windows();
  rates();
  limits();
  printf("gpu_all_host: %s\n", failures ? "FAILED" : "ok");
  return failures ? 1 : 0;
}
