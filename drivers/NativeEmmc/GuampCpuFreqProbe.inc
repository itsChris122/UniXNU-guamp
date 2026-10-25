// SPDX-License-Identifier: MIT
// SM6115 Silver-domain snapshot. No voltage, clock or performance-state writes.
// Sources: guamp boot_a DT; Motorola drivers/cpufreq/qcom-cpufreq-hw.c,
// qcom,cpufreq-hw standard layout (NOT the EPSS layout used on redwood).
namespace guamp_cpu_freq {
static void start()
{
  unsigned enabled = 0;
  if (!PE_parse_boot_argn("guamp-cpufreq-probe", &enabled, sizeof(enabled)) || !enabled) return;
  if (enabled != 1) {
    IOLog("GuampCPU: unsupported probe level %u; no register access\n", enabled);
    return;
  }
  volatile uint8_t *base = mapPhysical(0x0f521000ULL, 0x1000);
  if (!base) { IOLog("GuampCPU: Silver frequency-domain mapping failed\n"); return; }
  uint32_t control = *reinterpret_cast<volatile uint32_t *>(base);
  IOLog("GuampCPU: Silver control %08x (standard CPUFREQ_HW, 12 LUT rows)\n", control);
  if (!(control & 1u)) {
    IOLog("GuampCPU: controller disabled; no LUT or state reads\n");
    return;
  }
  uint32_t state = *reinterpret_cast<volatile uint32_t *>(base + 0x920);
  IOLog("GuampCPU: requested Silver performance index %u (not a measured clock rate)\n", state);
  if (state >= 12) {
    IOLog("GuampCPU: index outside board LUT; no indexed access\n");
    return;
  }
  for (unsigned i = 0; i < 12; ++i) {
    uint32_t row = *reinterpret_cast<volatile uint32_t *>(base + 0x110 + 32 * i);
    IOLog("GuampCPU: Silver LUT %u raw %08x source %u multiplier %u%s\n",
          i, row, (row >> 30) & 3u, row & 0xffu, i == state ? " requested" : "");
  }
  IOLog("GuampCPU: snapshot complete; frequency and thermal controls unchanged\n");
}
}
