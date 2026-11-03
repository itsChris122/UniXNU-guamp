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
void guamp_cpufreq_live();   // GuampPerf, every minute after the summary

namespace cpufreq {
constexpr uint64_t kDomainPhys[2] = {0x0f521000ULL, 0x0f523000ULL};
constexpr uint32_t kEnable = 0x0, kFreqLut = 0x110, kVoltLut = 0x114, kPerfState = 0x920, kRow = 32, kMaxRows = 12;
constexpr uint32_t kXoKhz = 19200;
static const char *const kNames[2] = {"silver cpu0-3", "gold cpu4-7"};

// Live check (03/11, guamp-cpufreq-live=1 default, =0 off): every minute one line per domain with the PERF_STATE request
// read back, the LMh limit register (DT qcom,limits-dcvs: msm_lmh_dcvs.c reads domain + 0x704, frequency = bits 9:0 x XO) and
// the domain cycle counter (REG_CYCLE_CNTR 0x9c0; bengal has qcom,no-accumulative-counter = one counter per domain) as average
// MHz since the previous line. Only reads: whether the cores really run at the state we asked for, or the hardware limits them.
constexpr uint32_t kLmhReq = 0x704, kCycles = 0x9c0;
static volatile uint8_t *gBase[2];
static uint32_t gKhz[2][kMaxRows], gRows[2], gLastCycles[2];
static uint64_t gLastNs[2];
static uint32_t rd(volatile uint8_t *b, uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(b + off); }
static void wr(volatile uint8_t *b, uint32_t off, uint32_t v) {
  *reinterpret_cast<volatile uint32_t *>(b + off) = v;
  __asm__ volatile("dsb sy" ::: "memory");
}

void start() {
  unsigned mode = 1, force[2] = {0xff, 0xff};   // default 1 (owner 28/10): highest regular state; guamp-cpufreq=0 to keep the boot state
  PE_parse_boot_argn("guamp-cpufreq", &mode, sizeof(mode));
  PE_parse_boot_argn("guamp-cpufreq-silver", &force[0], sizeof(force[0]));
  PE_parse_boot_argn("guamp-cpufreq-gold", &force[1], sizeof(force[1]));
  for (unsigned d = 0; d < 2; ++d) {
    volatile uint8_t *b = mapPhysical(kDomainPhys[d], 0x1000);
    if (!b) {
      IOLog("GuampCpufreq: %s mapping failed\n", kNames[d]);
      continue;
    }
    gBase[d] = b;
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
    gRows[d] = rows;
    for (unsigned i = 0; i < rows; ++i) gKhz[d][i] = khz[i];
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

void guamp_cpufreq_live() {
  using namespace cpufreq;
  static int enabled = -1;
  if (enabled < 0) {
    unsigned v = 1;
    PE_parse_boot_argn("guamp-cpufreq-live", &v, sizeof(v));
    enabled = v ? 1 : 0;
  }
  if (!enabled) return;
  for (unsigned d = 0; d < 2; ++d) {
    volatile uint8_t *b = gBase[d];
    if (!b) continue;
    const uint32_t state = rd(b, kPerfState), lmh = rd(b, kLmhReq), cycles = rd(b, kCycles);
    uint64_t now = 0;
    absolutetime_to_nanoseconds(mach_absolute_time(), &now);
    const uint64_t span = gLastNs[d] && now > gLastNs[d] ? now - gLastNs[d] : 0;
    const uint32_t delta = cycles - gLastCycles[d];   // 32-bit counter, wraps
    gLastNs[d] = now;
    gLastCycles[d] = cycles;
    IOLog("GuampCpufreq: %s live: state %u (%u MHz), LMh limit reg %08x (%u MHz), cycle counter %08x avg %llu MHz over %llu ms\n",
          kNames[d], state, state < gRows[d] ? gKhz[d][state] / 1000 : 0, lmh, (lmh & 0x3ffu) * kXoKhz / 1000, cycles,
          span ? static_cast<unsigned long long>(delta) * 1000ULL / span : 0ULL, static_cast<unsigned long long>(span / 1000000ULL));
  }
}
