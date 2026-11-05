// GuampBoot checkpoints read back by XNU (boot speed, 05/11/2026). Pure code, host-tested (tests/gb_marks_host.cpp).
//
// GuampBoot (~/moto/wt-guampboot, core/log.c) keeps its log in a RAM ring at 0x13FC00000 (GB_RAMLOG_BASE, 1 MiB, just
// above the XNU window 0x8007C000 + 0xBFA00000, so XNU never reuses it): header {u32 magic "GBLG", u32 version 1,
// u64 text capacity, u64 bytes written, u64 CNTPCT at gb_log_init}, then the text. Every gb_mark(stage) prints
// "[%5u.%06u] MARK <stage>\n" with the microseconds since GuampBoot's entry. Without logfs these lines never reached
// a log on the phone, so GuampBoot's share of the boot was unknown; this parser turns them into one summary line:
//   "GuampBoot: entry at 41.234 s of CNTPCT, entry->handoff 1840 ms (fdt 3, display 12, payload 410, hw 30, prepare 1385);
//    handoff -> this line 2950 ms" (the last part: from the hand-off to the moment the kernel read the ring)
// The ring of the previous boot can survive a warm reset: it is used only when its CNTPCT stamp is older than the
// current counter by less than kMaxAgeTicks (the kernel reads it ~1-3 s after the hand-off).
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace gbmarks {

constexpr uint64_t kRingPhys = 0x13FC00000ULL, kRingBytes = 0x100000ULL;
constexpr uint32_t kMagic = 0x474C4247u;   // "GBLG"
constexpr size_t kHeader = 32, kMaxText = 16384, kMaxMarks = 16, kNameLen = 12;

struct Mark { char name[kNameLen]; uint64_t us; };
struct Result {
  bool valid = false;
  const char *why = "not read";
  uint64_t entryTicks = 0;   // CNTPCT at gb_log_init (~ GuampBoot entry)
  unsigned count = 0;
  Mark marks[kMaxMarks] = {};
};

inline uint32_t le32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | static_cast<uint32_t>(p[3]) << 24; }
inline uint64_t le64(const uint8_t *p) { return le32(p) | static_cast<uint64_t>(le32(p + 4)) << 32; }

// header: the first kHeader bytes of the ring; text/len: the start of the text (len bytes, at most kMaxText copied);
// nowTicks/freq: the current CNTPCT and CNTFRQ; maxAgeTicks: how old the ring may be.
inline Result parse(const uint8_t *header, const char *text, size_t len, uint64_t nowTicks, uint64_t maxAgeTicks) {
  Result r;
  if (le32(header) != kMagic) { r.why = "no GuampBoot ring (UEFI chain or overwritten)"; return r; }
  if (le32(header + 4) != 1) { r.why = "unknown ring version"; return r; }
  const uint64_t capacity = le64(header + 8), written = le64(header + 16);
  r.entryTicks = le64(header + 24);
  if (!capacity || capacity > kRingBytes - kHeader) { r.why = "bad ring size"; return r; }
  if (r.entryTicks == 0 || r.entryTicks > nowTicks || nowTicks - r.entryTicks > maxAgeTicks) {
    r.why = "stale ring (previous boot)";
    return r;
  }
  if (written > capacity) { r.why = "ring wrapped"; return r; }
  if (written < len) len = static_cast<size_t>(written);   // only what GuampBoot wrote, never an older tail
  for (size_t at = 0; at < len && r.count < kMaxMarks;) {
    size_t end = at;
    while (end < len && text[end] != '\n') end++;
    // "[%5u.%06u] MARK name"
    size_t i = at;
    if (i < end && text[i] == '[') {
      i++;
      while (i < end && text[i] == ' ') i++;
      uint64_t sec = 0, usec = 0;
      unsigned digits = 0;
      while (i < end && text[i] >= '0' && text[i] <= '9') sec = sec * 10 + static_cast<uint64_t>(text[i++] - '0');
      if (i < end && text[i] == '.') {
        i++;
        while (i < end && text[i] >= '0' && text[i] <= '9' && digits < 6) { usec = usec * 10 + static_cast<uint64_t>(text[i++] - '0'); digits++; }
      }
      static const char tag[] = "] MARK ";
      bool match = digits == 6;
      for (size_t k = 0; match && tag[k]; k++) match = i + k < end && text[i + k] == tag[k];
      if (match) {
        i += sizeof(tag) - 1;
        Mark &m = r.marks[r.count];
        size_t n = 0;
        while (i < end && n + 1 < kNameLen && text[i] > ' ' && text[i] < 0x7f) m.name[n++] = text[i++];
        m.name[n] = 0;
        m.us = sec * 1000000 + usec;
        if (n) r.count++;
      }
    }
    at = end + 1;
  }
  if (!r.count) { r.why = "no MARK lines"; return r; }
  r.valid = true;
  r.why = "ok";
  return r;
}

// One line for IOLog, "entry->handoff" plus each step's share in ms. nowTicks: CNTPCT when the kernel read the ring.
inline size_t format(const Result &r, uint64_t nowTicks, uint64_t freq, char *out, size_t cap) {
  size_t n = 0;
  auto put = [&](const char *s) { while (*s && n + 1 < cap) out[n++] = *s++; };
  auto num = [&](uint64_t v) {
    char d[24];
    int k = 0;
    do { d[k++] = static_cast<char>('0' + v % 10); v /= 10; } while (v && k < 23);
    while (k && n + 1 < cap) out[n++] = d[--k];
  };
  if (cap) out[0] = 0;
  if (!cap || !freq) return 0;
  put("GuampBoot: ");
  if (!r.valid) { put(r.why); out[n] = 0; return n; }
  const uint64_t entryMs = r.entryTicks * 1000 / freq;
  put("entry at "); num(entryMs / 1000); put("."); num(entryMs % 1000 / 100); num(entryMs % 100 / 10); num(entryMs % 10);
  put(" s of CNTPCT, entry->"); put(r.marks[r.count - 1].name); put(" "); num(r.marks[r.count - 1].us / 1000); put(" ms (");
  for (unsigned i = 1; i < r.count; i++) {
    if (i > 1) put(", ");
    put(r.marks[i].name); put(" ");
    num((r.marks[i].us - (r.marks[i].us >= r.marks[i - 1].us ? r.marks[i - 1].us : r.marks[i].us)) / 1000);
  }
  put(")");
  const uint64_t lastTicks = r.entryTicks + r.marks[r.count - 1].us * freq / 1000000;
  if (nowTicks >= lastTicks) { put("; "); put(r.marks[r.count - 1].name); put(" -> this line "); num((nowTicks - lastTicks) * 1000 / freq); put(" ms"); }
  out[n] = 0;
  return n;
}

}  // namespace gbmarks
