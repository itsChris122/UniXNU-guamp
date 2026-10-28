// Host test of the battery health values the guamp power source gives iOS 17 (GuampBatteryCore.h guamp_battery::health,
// kBatterySerial, kWeightedRa): powerd's "Maximum Capacity Percent" = ceil(NominalChargeCapacity * 100 / DesignCapacity)
// (BatteryTimeRemaining, fcvtps), accepted 1..150, recomputed here from the published values; the Linux QG learned
// capacity in SDAM 0x68 (5029 mAh on the phone, kpan 28/10 14:34) gives 100 %; a value outside 50..110 % of the design is
// not a learning result and reports the design value (100 %, owner request 28/10); the serial passes powerd's length
// check (16..18) and has no year/week digits at 3..5.
// Build: run-battery-tests.sh (clang++/g++ -std=gnu++20 -Wall -Wextra -Werror -I.. battery_health_host.cpp)
#include <GuampBatteryCore.h>   // angle brackets: the -I copy (drivers/NativeEmmc)
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
using namespace guamp_battery;

static int failures = 0;
static void check(bool ok, const char *what) {
  if (!ok) { std::printf("FAIL %s\n", what); failures = failures + 1; }
}
// powerd: NCC% = fcvtps(NCC / Design * 100.0), used only when 1..150
static int powerdPercent(unsigned ncc, unsigned design) {
  if (!ncc || design < 1) return -1;
  int p = static_cast<int>(std::ceil(static_cast<double>(ncc) / static_cast<double>(design) * 100.0));
  return p >= 1 && p <= 150 ? p : -1;
}

int main() {
  // The phone today: SDAM learned 5029 mAh = profile FCC, 1 cycle.
  Health h = health(5029);
  check(h.learned && h.nominalMah == 5029 && h.percent == 100, "learned 5029 -> 100 %");
  check(powerdPercent(h.nominalMah, kDesignMah) == 100, "powerd agrees: 100 %");

  // A worn cell: every learned value in range gives the same number powerd computes, never above 100.
  unsigned checked = 0;
  for (unsigned mah = kDesignMah / 2; mah <= kDesignMah * 11 / 10; mah = mah + 1) {
    Health w = health(mah);
    check(w.learned, "in-range value is learned");
    check(w.nominalMah <= kDesignMah, "nominal capped at design");
    check(static_cast<int>(w.percent) == powerdPercent(w.nominalMah, kDesignMah), "percent == powerd ceil");
    check(w.percent >= 50 && w.percent <= 100, "percent 50..100");
    checked = checked + 1;
  }
  check(health(4527).percent == 91, "4527 mAh -> 91 % (90.02 rounded up as powerd)");
  check(health(4526).percent == 90, "4526 mAh -> 90 %");
  check(health(5300).nominalMah == kDesignMah && health(5300).percent == 100, "above design -> 100 %");

  // Not a learning result: SDAM empty (0), reset value (learnedMah returns 0 for >= 0x8000), absurd values.
  const unsigned bogus[] = {0, 1, 100, kDesignMah / 2 - 1, kDesignMah * 11 / 10 + 1, 0x7fff};
  for (unsigned v : bogus) {
    Health b = health(v);
    check(!b.learned && b.nominalMah == kDesignMah && b.percent == 100, "unknown capacity -> design, 100 %");
  }
  Sdam sd{};
  sd.b[0x68 - 0x46] = 0xff; sd.b[0x69 - 0x46] = 0xff;
  check(sd.learnedMah() == 0 && health(sd.learnedMah()).percent == 100, "SDAM 0xffff -> 100 %, not learned");
  sd.b[0x68 - 0x46] = 0xa5; sd.b[0x69 - 0x46] = 0x13;      // 0x13a5 = 5029, as on the phone
  check(sd.learnedMah() == 5029 && health(sd.learnedMah()).learned, "SDAM bytes a5 13 -> 5029 learned");

  // Serial: powerd's BDC path accepts 16..18 characters; Apple keeps YWW at 3..5, ours has no digits there.
  size_t n = std::strlen(kBatterySerial);
  check(n >= 16 && n <= 18, "serial length 16..18");
  for (size_t k = 3; k <= 5; k = k + 1) check(kBatterySerial[k] < '0' || kBatterySerial[k] > '9', "no YWW digits");
  // WeightedRa below thermalmonitord's default P0 threshold (500): no "degraded" state from our constant.
  check(kWeightedRa > 0 && kWeightedRa < 500, "WeightedRa 1..499");

  std::printf("battery health: %u learned values, %s\n", checked, failures ? "FAIL" : "all checks passed");
  return failures ? 1 : 0;
}
