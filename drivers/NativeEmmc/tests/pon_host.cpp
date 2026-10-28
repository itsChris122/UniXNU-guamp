// Host test of GuampPonCore.h: timer/type decoding and the held-key watch.
#include "GuampPonCore.h"
#include <stdio.h>
#include <string.h>
static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)
int main() {
  using namespace guamp_pon;
  CHECK(s1Ms(0) == 0 && s1Ms(0xe) == 6720 && s1Ms(0xf) == 10256 && s1Ms(0x1d) == 4480);
  CHECK(s2Ms(7) == 2000 && s2Ms(0xf) == 2000 && s2Ms(0) == 0);
  CHECK(!strcmp(resetType(1), "warm-reset") && !strcmp(resetType(0x84), "shutdown") && !strcmp(resetType(7), "hard-reset"));
  const uint8_t r[4] = {0x0e, 0x07, 0x01, 0x80};
  Reset x = decode(r);
  CHECK(x.s1 == 6720 && x.s2 == 2000 && x.type == 1 && x.enabled && x.totalMs() == 8720);
  const uint8_t off[4] = {0x0e, 0x07, 0x01, 0x00};
  CHECK(!decode(off).enabled);
  CHECK(sourceBase(kSrcKpdpwr) == 0x840 && sourceBase(kSrcResin) == 0x844 && sourceBase(kSrcCombo) == 0x848);
  CHECK(gen2(4) && gen2(5) && !gen2(1));
  ComboWatch w;
  uint32_t held = 99;
  CHECK(!w.feed(kKpdpwr, 0, &held));                 // Power alone: not the combination
  CHECK(w.feed(kKpdpwr | kResin, 100, &held) && held == 0 && w.combos == 1);
  CHECK(!w.feed(kKpdpwr | kResin | 0x10, 600, &held) && held == 500);
  CHECK(w.feed(kKpdpwr | kResin, 1100, &held) && held == 1000);
  CHECK(!w.feed(kKpdpwr | kResin, 1500, &held));
  CHECK(w.feed(kKpdpwr | kResin, 2100, &held) && held == 2000);
  CHECK(!w.feed(kKpdpwr, 2200, &held) && w.longestMs == 2100 && !w.active);
  CHECK(w.feed(kKpdpwr | kResin, 3000, &held) && w.combos == 2);
  ComboWatch p;
  p.mask = kKpdpwr;
  CHECK(p.feed(kKpdpwr | 0x10, 0, &held) && held == 0);
  CHECK(!p.feed(0x10, 25, &held));
  printf("pon_host: %s\n", fails ? "FAIL" : "PASS");
  return fails != 0;
}
