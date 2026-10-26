// SPDX-License-Identifier: BSD-2-Clause-Patent
// Read-only accounting through the kernel host API. No frequency/voltage changes.
#include <kern/host.h>
#include <mach/host_info.h>
extern "C" kern_return_t host_statistics(host_t, host_flavor_t, host_info_t, mach_msg_type_number_t *);
extern "C" kern_return_t host_info(host_t, host_flavor_t, host_info_t, mach_msg_type_number_t *);
extern "C" volatile uint32_t armcpu_core_timer[8], armcpu_core_sgi[8], armcpu_smp_step[8], armcpu_smp_fail[8];
extern "C" volatile uint32_t armcpu_gic_boot[8], armcpu_ipi_pending_mode;
extern "C" volatile uint64_t armcpu_ipi_sent, armcpu_ipi_interrupts;
extern "C" uint32_t guamp_smp_signal_flags(unsigned int);
static void guampPerfSample() {
  static bool checked = false, enabled = false, previousValid = false;
  static uint32_t previous[CPU_STATE_MAX] = {};
  if (!checked) {
    unsigned value = 0;
    enabled = PE_parse_boot_argn("guamp-perf", &value, sizeof(value)) && value == 1;
    checked = true;
  }
  if (!enabled) return;
  host_cpu_load_info_data_t load = {};
  mach_msg_type_number_t count = HOST_CPU_LOAD_INFO_COUNT;
  kern_return_t result = host_statistics(host_self(), HOST_CPU_LOAD_INFO, reinterpret_cast<host_info_t>(&load), &count);
  if (result != KERN_SUCCESS || count != HOST_CPU_LOAD_INFO_COUNT) {
    IOLog("GuampPerf: CPU accounting unavailable, status %d count %u\n", result, count);
    return;
  }
  host_basic_info_data_t basic = {};
  count = HOST_BASIC_INFO_COUNT;
  kern_return_t basicResult = host_info(host_self(), HOST_BASIC_INFO, reinterpret_cast<host_info_t>(&basic), &count);
  uint64_t total = 0, delta[CPU_STATE_MAX] = {};
  for (unsigned i = 0; i < CPU_STATE_MAX; ++i) {
    delta[i] = static_cast<uint32_t>(load.cpu_ticks[i] - previous[i]);
    previous[i] = load.cpu_ticks[i];
    total += delta[i];
  }
  if (previousValid && total) {
    IOLog("GuampPerf: CPU user %llu%% system %llu%% idle %llu%% nice %llu%%; available %d max %d (host status %d)\n",
          delta[CPU_STATE_USER] * 100 / total, delta[CPU_STATE_SYSTEM] * 100 / total,
          delta[CPU_STATE_IDLE] * 100 / total, delta[CPU_STATE_NICE] * 100 / total,
          basicResult == KERN_SUCCESS ? basic.avail_cpus : 0,
          basicResult == KERN_SUCCESS ? basic.max_cpus : 0, basicResult);
    IOLog("GuampSmp: IPI pending-mode %u sent %llu received %llu; GIC group %x enabled %x ctl %x typer %x\n",
          armcpu_ipi_pending_mode, armcpu_ipi_sent, armcpu_ipi_interrupts,
          armcpu_gic_boot[0], armcpu_gic_boot[1], armcpu_gic_boot[2], armcpu_gic_boot[3]);
    const unsigned cpuCount = basicResult == KERN_SUCCESS && basic.max_cpus > 0 && basic.max_cpus <= 8 ? basic.max_cpus : 0;
    for (unsigned cpu = 0; cpu < cpuCount; ++cpu)
      IOLog("GuampSmp: core %u stage %u fail %u timer %u IPI %u signal %x\n", cpu,
            armcpu_smp_step[cpu], armcpu_smp_fail[cpu], armcpu_core_timer[cpu],
            armcpu_core_sgi[cpu], guamp_smp_signal_flags(cpu));
  }
  previousValid = true;
}
