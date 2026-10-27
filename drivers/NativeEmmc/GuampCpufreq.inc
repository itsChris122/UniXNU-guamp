// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp CPU frequency through the SM6115 cpufreq-hw (OSM) block, included by NativeEmmc.cpp after the other services
// (mapPhysical). Nothing in this port scaled the CPUs: they ran at the boot loader's performance state.
//
// DT qcom,cpufreq-hw: freq-domain0 0xf521000 (cpu0-3, silver), freq-domain1 0xf523000 (cpu4-7, gold), 0x1000 each,
// qcom,max-lut-entries 12. Register layout and LUT decoding from the Motorola kernel drivers/cpufreq/qcom-cpufreq-hw.c
// (cpufreq_qcom_std_offsets): ENABLE 0x0, FREQ_LUT 0x110 and VOLT_LUT 0x114 with 32-byte rows, PERF_STATE 0x920.
// FREQ_LUT row: [31:30] source (0 = the "alternate" GPLL0 rate), [18:16] core count, [7:0] multiplier of XO 19.2 MHz.
// The table ends at the first row repeating the previous frequency with the same core count. Rows whose core count is
// below the domain's are boost/thermal states (Linux marks them invalid); the hardware LMh still limits on temperature.
//
// guamp-cpufreq=0 (default): log the tables and current states only. =1: highest regular state on both domains.
// =2: the state below the highest (cooler). guamp-cpufreq-silver=<index> / guamp-cpufreq-gold=<index> force an index.

// One line per domain kept for GuampPerf, which prints it every minute (the boot-time lines precede the USB log mirror).
char guamp_cpufreq_summary[2][160];

namespace cpufreq {
constexpr uint64_t kDomainPhys[2] = {0x0f521000ULL, 0x0f523000ULL};
constexpr uint32_t kEnable = 0x0, kFreqLut = 0x110, kVoltLut = 0x114, kPerfState = 0x920, kRow = 32, kMaxRows = 12;
constexpr uint32_t kXoKhz = 19200;
static const char *const kNames[2] = {"silver cpu0-3", "gold cpu4-7"};

static uint32_t rd(volatile uint8_t *b, uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(b + off); }
static void wr(volatile uint8_t *b, uint32_t off, uint32_t v) {
  *reinterpret_cast<volatile uint32_t *>(b + off) = v;
  __asm__ volatile("dsb sy" ::: "memory");
}

void start() {
  unsigned mode = 0, force[2] = {0xff, 0xff};
  PE_parse_boot_argn("guamp-cpufreq", &mode, sizeof(mode));
  PE_parse_boot_argn("guamp-cpufreq-silver", &force[0], sizeof(force[0]));
  PE_parse_boot_argn("guamp-cpufreq-gold", &force[1], sizeof(force[1]));
  for (unsigned d = 0; d < 2; ++d) {
    volatile uint8_t *b = mapPhysical(kDomainPhys[d], 0x1000);
    if (!b) {
      IOLog("GuampCpufreq: %s mapping failed\n", kNames[d]);
      continue;
    }
    uint32_t khz[kMaxRows] = {}, cores[kMaxRows] = {}, mv[kMaxRows] = {};
    unsigned rows = 0;
    for (unsigned i = 0; i < kMaxRows; ++i) {
      const uint32_t f = rd(b, kFreqLut + i * kRow), v = rd(b, kVoltLut + i * kRow);
      const uint32_t src = f >> 30, lval = f & 0xff, cc = (f >> 16) & 7;
      khz[i] = src ? kXoKhz * lval : 300000;   // source 0: the alternate GPLL0 / 2 rate, 300 MHz on bengal
      cores[i] = cc;
      mv[i] = v & 0xfff;
      rows = i + 1;
      if (i > 0 && khz[i] == khz[i - 1] && cores[i] == cores[i - 1]) {
        rows = i;
        break;
      }
    }
    const uint32_t maxCores = rows ? cores[0] : 0;
    int highest = -1;
    for (unsigned i = 0; i < rows; ++i)
      if (cores[i] == maxCores) highest = static_cast<int>(i);
    const uint32_t enable = rd(b, kEnable), before = rd(b, kPerfState);
    char table[kMaxRows * 24] = {};
    size_t used = 0;
    for (unsigned i = 0; i < rows && used < sizeof(table); ++i) {
      const int n = snprintf(table + used, sizeof(table) - used, " %u:%u/%umV/c%u", i, khz[i] / 1000, mv[i], cores[i]);
      if (n <= 0) break;
      used += static_cast<size_t>(n);
    }
    IOLog("GuampCpufreq: %s enable %x perf state %u (%u MHz), %u rows:%s\n", kNames[d], enable, before,
          before < rows ? khz[before] / 1000 : 0, rows, table);
    int want = -1;
    if (force[d] < rows) want = static_cast<int>(force[d]);
    else if (mode == 1) want = highest;
    else if (mode == 2) want = highest > 0 ? highest - 1 : highest;
    snprintf(guamp_cpufreq_summary[d], sizeof(guamp_cpufreq_summary[d]), "%s state %u (%u MHz of max %u), enable %x, unchanged",
             kNames[d], before, before < rows ? khz[before] / 1000 : 0, highest >= 0 ? khz[highest] / 1000 : 0, enable);
    if (want < 0 || !(enable & 1)) {
      if (want >= 0) IOLog("GuampCpufreq: %s hardware not enabled, state left\n", kNames[d]);
      continue;
    }
    wr(b, kPerfState, static_cast<uint32_t>(want));
    IODelay(100);
    const uint32_t after = rd(b, kPerfState);
    IOLog("GuampCpufreq: %s perf state %u -> %u (%u -> %u MHz)%s\n", kNames[d], before, after,
          before < rows ? khz[before] / 1000 : 0, after < rows ? khz[after] / 1000 : 0,
          after == static_cast<uint32_t>(want) ? "" : " (not taken)");
    snprintf(guamp_cpufreq_summary[d], sizeof(guamp_cpufreq_summary[d]), "%s state %u -> %u (%u -> %u MHz of max %u)%s",
             kNames[d], before, after, before < rows ? khz[before] / 1000 : 0, after < rows ? khz[after] / 1000 : 0,
             highest >= 0 ? khz[highest] / 1000 : 0, after == static_cast<uint32_t>(want) ? "" : " not taken");
  }
}
}  // namespace cpufreq
