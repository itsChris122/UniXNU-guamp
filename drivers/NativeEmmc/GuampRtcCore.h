// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp wall clock from the PM6125 RTC (read only) plus a saved offset. No hardware access of its own: the owner
// supplies the PMIC register read and the two record blocks, so all of it runs on the host (tests/rtc_host.cpp).
//
// PM6125 RTC (DT qcom,pm6125_rtc, compatible qcom,pm8941-rtc, SPMI slave 0; interrupt peripheral 0x61 = alarm block),
// register layout studied from Linux drivers/rtc/rtc-pm8xxx.c pm8941_regs (no code copied):
//   0x6046 RTC_RW CTRL   bit 7 RTC_EN (counter running)
//   0x6048 RTC_RW READ   4 bytes, little endian, seconds since an arbitrary epoch (the PMIC's first power-up unless
//                        someone wrote 0x6040); read all four, read byte 0 again, and if it went down the counter
//                        rolled over between the bytes: read all four again (Linux pm8xxx_rtc_read_time)
//   0x6140.. alarm block, never touched here. Nothing is ever written to the RTC.
// Android keeps its wall clock offset in the Qualcomm time_daemon files on /data (ats_*), gone with the LineageOS
// userdata, so the offset is ours: offset = UTC - counter, saved after every settimeofday (timed, Settings) in two
// alternating 512-byte records (sequence + checksum: a torn write leaves the other record valid).
#ifndef GUAMP_RTC_CORE_H
#define GUAMP_RTC_CORE_H
#include <stdint.h>
#include <string.h>

namespace guamp_rtc {
constexpr unsigned kSid = 0;                       // PM6125
constexpr uint16_t kCtrl = 0x6046, kRead = 0x6048;
constexpr uint8_t kRtcEn = 0x80;
constexpr unsigned kRecordBytes = 512;
constexpr int64_t kSaveThreshold = 2;              // seconds: smaller offset changes are not written
// 2026-10-29 00:00:00 UTC: never start before this, even if the build date says otherwise
constexpr int64_t kFloorUtc = 1793232000;

// Days since 1970-01-01 of a proleptic Gregorian date (H. Hinnant's days_from_civil)
constexpr int64_t daysFromCivil(int64_t y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + static_cast<int64_t>(doe) - 719468;
}

// "Mmm dd yyyy" (__DATE__) -> UTC seconds at 00:00, 0 when it does not parse
inline int64_t dateToUtc(const char *date) {
  static const char kMonths[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  if (!date || strlen(date) < 11) return 0;
  unsigned month = 0;
  for (unsigned m = 0; m < 12; m++) {
    if (!memcmp(date, kMonths + 3 * m, 3)) month = m + 1;
  }
  unsigned day = 0, year = 0;
  for (unsigned i = 4; i < 6; i++) {
    if (date[i] >= '0' && date[i] <= '9') day = day * 10 + static_cast<unsigned>(date[i] - '0');
    else if (date[i] != ' ') return 0;
  }
  for (unsigned i = 7; i < 11; i++) {
    if (date[i] < '0' || date[i] > '9') return 0;
    year = year * 10 + static_cast<unsigned>(date[i] - '0');
  }
  if (!month || !day || day > 31 || year < 1970) return 0;
  return daysFromCivil(year, month, day) * 86400;
}

// Earliest wall clock: the build date of this driver (= kernel build) or kFloorUtc, whichever is later
inline int64_t floorUtc(const char *buildDate) {
  int64_t built = dateToUtc(buildDate);
  return built > kFloorUtc ? built : kFloorUtc;
}

// Counter read as Linux does. rd(ctx, addr, v) returns 0 on success. Return: 0 ok, 1 RTC not enabled, <0 read error.
template <typename Rd>
int readCounter(Rd rd, uint32_t *secs) {
  uint8_t ctrl = 0, v[4] = {}, again = 0;
  int r = rd(kCtrl, &ctrl);
  if (r) return r < 0 ? r : -1;
  if (!(ctrl & kRtcEn)) return 1;
  for (unsigned i = 0; i < 4; i++) {
    if ((r = rd(static_cast<uint16_t>(kRead + i), &v[i]))) return r < 0 ? r : -1;
  }
  if ((r = rd(kRead, &again))) return r < 0 ? r : -1;
  if (again < v[0]) {   // byte 0 wrapped: the upper bytes may belong to the old value
    for (unsigned i = 0; i < 4; i++) {
      if ((r = rd(static_cast<uint16_t>(kRead + i), &v[i]))) return r < 0 ? r : -1;
    }
  }
  *secs = static_cast<uint32_t>(v[0]) | static_cast<uint32_t>(v[1]) << 8 | static_cast<uint32_t>(v[2]) << 16 |
          static_cast<uint32_t>(v[3]) << 24;
  return 0;
}

// Saved offset. Block layout (little endian): 0 "GUAMPRTC", 8 version 1, 12 sequence, 16 int64 offset,
// 24 uint32 counter at save, 28 flags (bit 0 trusted = from settimeofday), 32 int64 UTC at save, 40 FNV-1a of 0..39
struct Record {
  uint32_t sequence = 0;
  int64_t offset = 0;
  uint32_t counter = 0;
  uint32_t flags = 0;
  int64_t utc = 0;
};
constexpr uint32_t kTrusted = 1;

inline uint32_t fnv1a(const uint8_t *p, unsigned n) {
  uint32_t h = 2166136261u;
  for (unsigned i = 0; i < n; i++) h = (h ^ p[i]) * 16777619u;
  return h;
}
inline void put32(uint8_t *p, uint32_t v) { for (unsigned i = 0; i < 4; i++) p[i] = static_cast<uint8_t>(v >> (8 * i)); }
inline void put64(uint8_t *p, uint64_t v) { for (unsigned i = 0; i < 8; i++) p[i] = static_cast<uint8_t>(v >> (8 * i)); }
inline uint32_t get32(const uint8_t *p) { uint32_t v = 0; for (unsigned i = 0; i < 4; i++) v |= static_cast<uint32_t>(p[i]) << (8 * i); return v; }
inline uint64_t get64(const uint8_t *p) { uint64_t v = 0; for (unsigned i = 0; i < 8; i++) v |= static_cast<uint64_t>(p[i]) << (8 * i); return v; }

inline void encode(const Record &r, uint8_t *block) {
  memset(block, 0, kRecordBytes);
  memcpy(block, "GUAMPRTC", 8);
  put32(block + 8, 1);
  put32(block + 12, r.sequence);
  put64(block + 16, static_cast<uint64_t>(r.offset));
  put32(block + 24, r.counter);
  put32(block + 28, r.flags);
  put64(block + 32, static_cast<uint64_t>(r.utc));
  put32(block + 40, fnv1a(block, 40));
}
inline bool decode(const uint8_t *block, Record *r) {
  if (memcmp(block, "GUAMPRTC", 8) || get32(block + 8) != 1 || get32(block + 40) != fnv1a(block, 40)) return false;
  r->sequence = get32(block + 12);
  r->offset = static_cast<int64_t>(get64(block + 16));
  r->counter = get32(block + 24);
  r->flags = get32(block + 28);
  r->utc = static_cast<int64_t>(get64(block + 32));
  return true;
}
// Newest valid record of the two slots: 0 / 1, or -1 for none
inline int pick(bool okA, const Record &a, bool okB, const Record &b) {
  if (okA && okB) return static_cast<int32_t>(b.sequence - a.sequence) > 0 ? 1 : 0;
  return okA ? 0 : okB ? 1 : -1;
}
// Slot for the next write: never the one holding the newest record
inline unsigned nextSlot(int newest) { return newest == 0 ? 1 : 0; }

enum Source : unsigned { kFromOffset, kFromSavedUtc, kFromFloor };
struct BootTime { int64_t utc; Source source; };
// Wall clock at boot. counterOk = readCounter() returned 0. With a record: counter + offset, unless the counter is
// below the saved counter (RTC reset, e.g. battery out) or the result is before the saved UTC: then the saved UTC.
// Without a record (first boot) or without a counter: the floor. Never below the floor unless a trusted record says so
// (the user chose that date).
inline BootTime bootTime(bool counterOk, uint32_t counter, bool haveRecord, const Record &rec, int64_t floor) {
  BootTime t{floor, kFromFloor};
  if (haveRecord && counterOk && counter >= rec.counter) {
    t = {static_cast<int64_t>(counter) + rec.offset, kFromOffset};
    if (t.utc < rec.utc) t = {rec.utc, kFromSavedUtc};
  } else if (haveRecord) {
    t = {rec.utc, kFromSavedUtc};
  }
  if (t.utc < floor && !(haveRecord && (rec.flags & kTrusted))) t = {floor, kFromFloor};
  return t;
}

// A settimeofday to utc while the counter read counter: does the saved record need rewriting?
inline bool needsSave(bool haveRecord, const Record &rec, int64_t utc, uint32_t counter) {
  if (!haveRecord || !(rec.flags & kTrusted)) return true;
  int64_t delta = (utc - static_cast<int64_t>(counter)) - rec.offset;
  return delta > kSaveThreshold || delta < -kSaveThreshold;
}
inline Record makeRecord(const Record &previous, bool haveRecord, int64_t utc, uint32_t counter, bool trusted) {
  Record r;
  r.sequence = haveRecord ? previous.sequence + 1 : 1;
  r.offset = utc - static_cast<int64_t>(counter);
  r.counter = counter;
  r.flags = trusted ? kTrusted : 0;
  r.utc = utc;
  return r;
}
}  // namespace guamp_rtc
#endif
