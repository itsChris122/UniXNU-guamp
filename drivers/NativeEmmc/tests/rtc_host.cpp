// Host test of the guamp wall clock (GuampRtcCore.h): build date floor, the Linux-style RTC counter read with byte 0
// rollover, record encode/decode/pick, boot time rules, the save threshold, and a whole sequence on a simulated
// PM6125 RTC + two kpan slots: first boot, time set by hand, reboots, RTC reset, torn write, power cut before saving.
// Build: run-rtc-tests.sh (clang++/g++ -std=gnu++20 -Wall -Wextra -Werror -I.. rtc_host.cpp)
#include <GuampRtcCore.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
using namespace guamp_rtc;

static unsigned gFailures = 0, gChecks = 0;
#define CHECK(c) do { gChecks++; if (!(c)) { gFailures++; std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

struct Pm6125Rtc {
  uint8_t ctrl = kRtcEn;
  uint32_t counter = 0;
  int tickAfterReads = -1;   // the counter advances after this many data reads (rollover between the bytes)
  int failAt = -1;           // read number that fails
  int reads = 0;
  int rd(uint16_t addr, uint8_t *v) {
    if (reads++ == failAt) return -5;
    if (addr == kCtrl) { *v = ctrl; return 0; }
    if (addr >= kRead && addr < kRead + 4) {
      *v = static_cast<uint8_t>(counter >> (8 * (addr - kRead)));
      if (tickAfterReads > 0 && --tickAfterReads == 0) counter++;
      return 0;
    }
    return -19;
  }
};
static int readRtc(Pm6125Rtc &r, uint32_t *s) { return readCounter([&](uint16_t a, uint8_t *v) { return r.rd(a, v); }, s); }

// Two kpan record slots as the kernel uses them
struct Kpan {
  uint8_t slot[2][kRecordBytes];
  Kpan() { memset(slot, 0xff, sizeof(slot)); }
};
struct Clock {   // the kernel side reduced to the core calls
  Kpan *kpan;
  explicit Clock(Kpan *k) : kpan(k) {}
  Record rec;
  bool have = false;
  int newest = -1;
  void load() {
    Record a, b;
    bool okA = decode(kpan->slot[0], &a), okB = decode(kpan->slot[1], &b);
    newest = pick(okA, a, okB, b);
    have = newest >= 0;
    if (have) rec = newest ? b : a;
  }
  void save(int64_t utc, uint32_t counter, bool trusted) {
    Record r = makeRecord(rec, have, utc, counter, trusted);
    unsigned s = nextSlot(newest);
    encode(r, kpan->slot[s]);
    rec = r; have = true; newest = static_cast<int>(s);
  }
  BootTime boot(Pm6125Rtc &rtc, int64_t floor) {
    load();
    uint32_t c = 0;
    int cr = readRtc(rtc, &c);
    BootTime t = bootTime(cr == 0, c, have, rec, floor);
    if (cr == 0 && t.source != kFromOffset) save(t.utc, c, false);
    return t;
  }
  void set(Pm6125Rtc &rtc, int64_t utc) {
    uint32_t c = 0;
    if (readRtc(rtc, &c)) return;
    if (needsSave(have, rec, utc, c)) save(utc, c, true);
  }
};

int main() {
  // dates
  CHECK(dateToUtc("Oct 29 2026") == 1793232000);
  CHECK(dateToUtc("Jan  1 1970") == 0);
  CHECK(dateToUtc("Feb 29 2028") == 1835395200);
  CHECK(dateToUtc("Dec 31 2026") == 1798675200);
  CHECK(dateToUtc("Foo 12 2026") == 0 && dateToUtc("Oct") == 0 && dateToUtc(nullptr) == 0);
  CHECK(floorUtc("Jan  1 2020") == kFloorUtc);
  CHECK(floorUtc("Nov  5 2026") == dateToUtc("Nov  5 2026") && floorUtc("Nov  5 2026") > kFloorUtc);
  CHECK(floorUtc(__DATE__) >= kFloorUtc);

  // counter read
  { Pm6125Rtc r; r.counter = 0x12345678; uint32_t s = 0; CHECK(readRtc(r, &s) == 0 && s == 0x12345678u); }
  { Pm6125Rtc r; r.ctrl = 0; r.counter = 5; uint32_t s = 7; CHECK(readRtc(r, &s) == 1 && s == 7); }
  { Pm6125Rtc r; r.counter = 0x000100ffu; r.tickAfterReads = 1; uint32_t s = 0;   // ff read, then 0x00010100
    CHECK(readRtc(r, &s) == 0 && s == 0x00010100u); }
  { Pm6125Rtc r; r.counter = 0x00ffffffu; r.tickAfterReads = 2; uint32_t s = 0;   // wraps into byte 3
    CHECK(readRtc(r, &s) == 0 && s == 0x01000000u); }
  { Pm6125Rtc r; r.counter = 0x10; r.tickAfterReads = 4; uint32_t s = 0;          // ticks after the 4 bytes: no reread
    CHECK(readRtc(r, &s) == 0 && s == 0x10u); }
  for (int f = 0; f < 6; f++) { Pm6125Rtc r; r.failAt = f; uint32_t s = 0; CHECK(readRtc(r, &s) < 0); }

  // records
  { Record r; r.sequence = 7; r.offset = -1234567890123LL; r.counter = 0xdeadbeef; r.flags = kTrusted; r.utc = 1800000000;
    uint8_t b[kRecordBytes]; encode(r, b); Record d; CHECK(decode(b, &d));
    CHECK(d.sequence == 7 && d.offset == r.offset && d.counter == r.counter && d.flags == kTrusted && d.utc == r.utc);
    b[17] ^= 1; CHECK(!decode(b, &d));
    uint8_t z[kRecordBytes] = {}; CHECK(!decode(z, &d)); }
  { Record a, b; a.sequence = 4; b.sequence = 5;
    CHECK(pick(true, a, true, b) == 1 && pick(true, b, true, a) == 0 && pick(false, a, true, b) == 1 && pick(true, a, false, b) == 0);
    CHECK(pick(false, a, false, b) == -1);
    a.sequence = 0xffffffffu; b.sequence = 0; CHECK(pick(true, a, true, b) == 1);   // sequence wrap
    CHECK(nextSlot(-1) == 0 && nextSlot(0) == 1 && nextSlot(1) == 0); }

  // boot rules
  const int64_t floor = kFloorUtc;
  { Record none; BootTime t = bootTime(true, 1000, false, none, floor); CHECK(t.utc == floor && t.source == kFromFloor); }
  { Record rec = makeRecord(Record(), false, 1800000000, 1000, true);
    BootTime t = bootTime(true, 4600, true, rec, floor); CHECK(t.utc == 1800003600 && t.source == kFromOffset);
    t = bootTime(true, 10, true, rec, floor); CHECK(t.utc == 1800000000 && t.source == kFromSavedUtc);   // RTC reset
    t = bootTime(false, 0, true, rec, floor); CHECK(t.utc == 1800000000 && t.source == kFromSavedUtc); }
  { Record user = makeRecord(Record(), false, 1600000000, 1000, true);   // 2020 chosen by hand: kept
    CHECK(bootTime(true, 1060, true, user, floor).utc == 1600000060); }
  { Record est = makeRecord(Record(), false, 1600000000, 1000, false);   // an estimate below the floor: floor
    CHECK(bootTime(true, 1060, true, est, floor).utc == floor); }

  // threshold
  { Record rec = makeRecord(Record(), false, 1800000000, 1000, true);
    CHECK(!needsSave(true, rec, 1800000100, 1100) && !needsSave(true, rec, 1800000102, 1100));
    CHECK(needsSave(true, rec, 1800000103, 1100) && needsSave(true, rec, 1800000097, 1100));
    CHECK(needsSave(false, rec, 1800000100, 1100));
    Record est = rec; est.flags = 0; CHECK(needsSave(true, est, 1800000100, 1100));   // first trusted set always saved
    Record next = makeRecord(rec, true, 1800000500, 1100, true); CHECK(next.sequence == 2 && next.offset == 1800000500 - 1100); }

  // whole sequence: kpan untouched (0xff), RTC counting from its own epoch
  { Kpan kpan; Pm6125Rtc rtc; rtc.counter = 50000;
    Clock k(&kpan);
    BootTime t = k.boot(rtc, floor);                  // first boot: floor, estimate saved
    CHECK(t.source == kFromFloor && t.utc == floor && k.have && !(k.rec.flags & kTrusted));
    rtc.counter += 100;
    t = k.boot(rtc, floor);                           // second boot without any set: continues, no write
    CHECK(t.source == kFromOffset && t.utc == floor + 100 && k.rec.sequence == 1);
    // user sets 2026-11-02 10:00:00 by hand at counter 50200
    rtc.counter = 50200;
    const int64_t chosen = 1793613600;
    k.set(rtc, chosen);
    CHECK(k.rec.flags & kTrusted && k.rec.offset == chosen - 50200 && k.rec.sequence == 2);
    k.set(rtc, chosen + 1);                           // timed nudges by 1 s: not written
    CHECK(k.rec.sequence == 2);
    rtc.counter += 3600 + 45;                         // reboot one hour and 45 s later (RAM boot)
    t = k.boot(rtc, floor);
    CHECK(t.source == kFromOffset && t.utc == chosen + 3645);
    // torn write of the next record: the previous one is used
    rtc.counter += 10;
    k.set(rtc, chosen + 3655 + 86400);                // user moves one day ahead
    unsigned torn = static_cast<unsigned>(k.newest);
    kpan.slot[torn][20] ^= 0x40;
    t = k.boot(rtc, floor);
    CHECK(t.source == kFromOffset && t.utc == chosen + 3655);
    // power cut before the deferred write: same as no set at all
    Clock k2(&kpan);
    rtc.counter += 5;
    t = k2.boot(rtc, floor);
    CHECK(t.utc == chosen + 3660);
    // battery out: counter starts again from 0; never earlier than the saved UTC, then continues from there
    rtc.counter = 3;
    t = k2.boot(rtc, floor);
    CHECK(t.source == kFromSavedUtc && t.utc == k2.rec.utc);
    int64_t after = t.utc;
    rtc.counter = 63;
    t = k2.boot(rtc, floor);
    CHECK(t.source == kFromOffset && t.utc == after + 60);
    // RTC disabled (RTC_EN 0): saved UTC, nothing written
    uint32_t seq = k2.rec.sequence;
    rtc.ctrl = 0;
    t = k2.boot(rtc, floor);
    CHECK(t.source == kFromSavedUtc && k2.rec.sequence == seq); }

  std::printf("rtc host: %u checks, %u failures\n", gChecks, gFailures);
  return gFailures ? 1 : 0;
}
