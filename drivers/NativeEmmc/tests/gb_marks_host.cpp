// Host tests of GuampBootMarksCore.h: a ring as GuampBoot's core/log.c writes it, parsed and formatted.
#include "../GuampBootMarksCore.h"
#include <cstdio>
#include <cstring>
#include <string>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = static_cast<uint8_t>(v >> (8 * i)); }
static void put64(uint8_t *p, uint64_t v) { for (int i = 0; i < 8; i++) p[i] = static_cast<uint8_t>(v >> (8 * i)); }

struct Ring { uint8_t head[gbmarks::kHeader] = {}; std::string text; };

static Ring ring(uint64_t entryTicks, const std::string &text, uint32_t magic = gbmarks::kMagic, uint64_t written = ~0ull) {
  Ring r;
  put32(r.head, magic);
  put32(r.head + 4, 1);
  put64(r.head + 8, gbmarks::kRingBytes - gbmarks::kHeader);
  put64(r.head + 16, written == ~0ull ? text.size() : written);
  put64(r.head + 24, entryTicks);
  r.text = text;
  return r;
}

static std::string mark(uint64_t us, const char *stage) {   // gb_mark's "[%5u.%06u] MARK %s\n"
  char b[64];
  std::snprintf(b, sizeof b, "[%5u.%06u] MARK %s\n", unsigned(us / 1000000), unsigned(us % 1000000), stage);
  return b;
}

int main() {
  const uint64_t freq = 19200000, entry = 41234ull * freq / 1000;   // GuampBoot entered at 41.234 s of CNTPCT
  std::string text = "GuampBoot 0.0-dev, EL1, FDT 0x404fe000\n" + mark(12, "entry") + "INFO something\n" + mark(3012, "fdt") +
                     mark(15012, "display") + mark(425012, "payload") + "WARN USB not powered\n" + mark(455012, "hw") +
                     mark(1840012, "prepare") + mark(1841012, "handoff") + "  fdt              +3000 us\n";
  {
    Ring r = ring(entry, text);
    const uint64_t now = entry + 4 * freq;   // read 4 s after the entry
    auto res = gbmarks::parse(r.head, r.text.data(), r.text.size(), now, 120 * freq);
    CHECK(res.valid);
    CHECK(res.count == 7);
    CHECK(!std::strcmp(res.marks[0].name, "entry") && res.marks[0].us == 12);
    CHECK(!std::strcmp(res.marks[6].name, "handoff") && res.marks[6].us == 1841012);
    char line[320];
    gbmarks::format(res, now, freq, line, sizeof line);
    std::printf("%s\n", line);
    CHECK(std::strstr(line, "entry at 41.234 s of CNTPCT"));
    CHECK(std::strstr(line, "entry->handoff 1841 ms"));
    CHECK(std::strstr(line, "(fdt 3, display 12, payload 410, hw 30, prepare 1385, handoff 1)"));
    CHECK(std::strstr(line, "handoff -> this line 2158 ms"));
    // a short output buffer stays terminated
    char tiny[24];
    gbmarks::format(res, now, freq, tiny, sizeof tiny);
    CHECK(std::strlen(tiny) == sizeof tiny - 1);
  }
  {  // UEFI chain: no magic
    Ring r = ring(entry, text, 0x12345678);
    auto res = gbmarks::parse(r.head, r.text.data(), r.text.size(), entry + freq, 120 * freq);
    CHECK(!res.valid && std::strstr(res.why, "no GuampBoot ring"));
    char line[128];
    gbmarks::format(res, entry + freq, freq, line, sizeof line);
    CHECK(!std::strcmp(line, "GuampBoot: no GuampBoot ring (UEFI chain or overwritten)"));
  }
  {  // ring of a previous boot after a warm reset: too old, or stamped after "now"
    Ring r = ring(entry, text);
    CHECK(!gbmarks::parse(r.head, r.text.data(), r.text.size(), entry + 200 * freq, 120 * freq).valid);
    CHECK(!gbmarks::parse(r.head, r.text.data(), r.text.size(), entry - 1, 120 * freq).valid);
  }
  {  // only the written bytes count; a stale tail after them is ignored
    std::string withTail = text + mark(9999999, "stale");
    Ring r = ring(entry, withTail, gbmarks::kMagic, text.size());
    auto res = gbmarks::parse(r.head, r.text.data(), r.text.size(), entry + freq, 120 * freq);
    CHECK(res.valid && res.count == 7);
  }
  {  // malformed lines: no digits, five decimals, missing tag
    std::string bad = "[    1.00000] MARK x\n[abc] MARK y\n[    1.000000] MARKz\n" + mark(5, "ok");
    Ring r = ring(entry, bad);
    auto res = gbmarks::parse(r.head, r.text.data(), r.text.size(), entry + freq, 120 * freq);
    CHECK(res.valid && res.count == 1 && !std::strcmp(res.marks[0].name, "ok"));
  }
  {  // wrapped ring and empty text
    Ring r = ring(entry, text, gbmarks::kMagic, gbmarks::kRingBytes);
    CHECK(!gbmarks::parse(r.head, r.text.data(), r.text.size(), entry + freq, 120 * freq).valid);
    Ring e = ring(entry, "");
    CHECK(!gbmarks::parse(e.head, e.text.data(), 0, entry + freq, 120 * freq).valid);
  }
  {  // more marks than kMaxMarks: kept bounded
    std::string many;
    for (int i = 0; i < 40; i++) many += mark(1000 * i, "m");
    Ring r = ring(entry, many);
    auto res = gbmarks::parse(r.head, r.text.data(), r.text.size(), entry + freq, 120 * freq);
    CHECK(res.valid && res.count == gbmarks::kMaxMarks);
  }
  std::printf("%s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
  return failures ? 1 : 0;
}
