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
extern "C" volatile uint64_t guamp_timer_snapshot[8][27];
extern "C" volatile uint32_t guamp_timer_sequence[8];
extern "C" unsigned int guamp_ram_nc_enabled;
extern "C" volatile uint64_t guamp_irq_trace[8][3][14];
extern "C" volatile uint32_t guamp_irq_trace_sequence[8];
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
    IOLog("GuampRamCache: managed IO RAM to Normal NC %u; MMIO/native WB policy retained\n", guamp_ram_nc_enabled);
    IOLog("GuampSmp: IPI pending-mode %u sent %llu received %llu; GIC group %x enabled %x ctl %x typer %x\n",
          armcpu_ipi_pending_mode, armcpu_ipi_sent, armcpu_ipi_interrupts,
          armcpu_gic_boot[0], armcpu_gic_boot[1], armcpu_gic_boot[2], armcpu_gic_boot[3]);
    const unsigned cpuCount = basicResult == KERN_SUCCESS && basic.max_cpus > 0 && basic.max_cpus <= 8 ? basic.max_cpus : 0;
    for (unsigned cpu = 0; cpu < cpuCount; ++cpu)
      IOLog("GuampSmp: core %u stage %u fail %u timer %u IPI %u signal %x\n", cpu,
            armcpu_smp_step[cpu], armcpu_smp_fail[cpu], armcpu_core_timer[cpu],
            armcpu_core_sgi[cpu], guamp_smp_signal_flags(cpu));
    for (unsigned cpu = 0; cpu < cpuCount; ++cpu) {
      uint32_t sequence = __atomic_load_n(&guamp_timer_sequence[cpu], __ATOMIC_ACQUIRE);
      if (!sequence || (sequence & 1)) continue;
      uint64_t r[27];
      for (unsigned i = 0; i < 27; ++i) r[i] = guamp_timer_snapshot[cpu][i];
      __atomic_thread_fence(__ATOMIC_ACQUIRE);
      if (sequence != __atomic_load_n(&guamp_timer_sequence[cpu], __ATOMIC_ACQUIRE)) continue;
      IOLog("GuampTimer: core %u mode %llu mpidr %llx CNTFRQ %llu before %llu ctl %llx cval %llu rtc %llu dec %llx after %llu ctl %llx cval %llu rtc %llu dec %llx SCTLR %llx MAIR %llx TCR %llx actual-core %llu\n",
            cpu,r[15],r[16]&0xffffff,r[1],r[0],r[2],r[3],r[7],r[8],r[9],r[10],r[11],r[12],r[13],r[4],r[5],r[6],r[14]);
      IOLog("GuampGic: core %u ICC_CTLR %llx HPPIR1 %llx group %llx enabled %llx pending %llx groupmod %llx waker %llx distctl %llx priority %llx found %llu\n",
            cpu,r[17],r[18],r[19],r[20],r[21],r[22],r[23],r[24],r[25],r[26]);
      uint32_t irqSeq = __atomic_load_n(&guamp_irq_trace_sequence[cpu], __ATOMIC_ACQUIRE);
      if (!irqSeq || (irqSeq & 1)) continue;
      uint64_t trace[3][14];
      for (unsigned phase = 0; phase < 3; ++phase)
        for (unsigned j = 0; j < 14; ++j) trace[phase][j] = guamp_irq_trace[cpu][phase][j];
      __atomic_thread_fence(__ATOMIC_ACQUIRE);
      if (irqSeq != __atomic_load_n(&guamp_irq_trace_sequence[cpu], __ATOMIC_ACQUIRE)) continue;
      for (unsigned phase = 0; phase < 3; ++phase) {
        const uint64_t *q = trace[phase];
        IOLog("GuampIrq: core %u seq %u phase %u ct %llu vcval %llu vctl %llx pcval %llu pctl %llx pending %llx active %llx config %llx ctlr %llx rpr %llx hppir %llx iar %llx mpidr %llx freq %llu\n",
          cpu,irqSeq,phase,q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7],q[8],q[9],q[10],q[11],q[12]&0xffffff,q[13]);
      }
    }
  }
  previousValid = true;
}
