// SPDX-License-Identifier: BSD-2-Clause-Patent
// Read-only accounting through the kernel host API. No frequency/voltage changes.
#include <kern/host.h>
#include <mach/host_info.h>
#include <kern/thread_call.h>
#include <kern/clock.h>
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
extern "C" volatile uint32_t guamp_gpu_step[3];   // GpuService.inc: last GPU access step, register, sequence
extern "C" kern_return_t host_statistics64(host_t, host_flavor_t, host_info64_t, mach_msg_type_number_t *);
extern "C" void proc_iterate(unsigned int, int (*)(struct proc *, void *), void *, int (*)(struct proc *, void *), void *);
extern "C" task_t proc_task(struct proc *);
extern "C" int proc_pid(struct proc *);
extern "C" char *proc_best_name(struct proc *);
extern "C" uint64_t get_task_phys_footprint(task_t);
extern "C" uint64_t get_task_cpu_time(task_t);   // cpu_time ledger, absolute time units

// guamp-perf=1 memory line every sample, the largest footprints and CPU users every third sample (read only).
struct GuampMemTop {
  uint64_t bytes[6]; int pid[6]; char name[6][20]; unsigned procs; uint64_t total;
  uint64_t cpu[6]; int cpuPid[6]; char cpuName[6][20]; uint64_t cpuTotal;
};
// CPU time seen per pid at the previous walk (pids above the table size are not tracked).
static uint64_t guampCpuLast[2048];
static uint8_t guampCpuSeen[2048];
static void guampCpuTopInsert(GuampMemTop *top, struct proc *p, int pid, uint64_t delta) {
  unsigned slot = 6;
  while (slot > 0 && top->cpu[slot - 1] < delta) slot = slot - 1;
  if (slot >= 6) return;
  for (unsigned i = 5; i > slot; i = i - 1) {
    top->cpu[i] = top->cpu[i - 1];
    top->cpuPid[i] = top->cpuPid[i - 1];
    memcpy(top->cpuName[i], top->cpuName[i - 1], sizeof(top->cpuName[i]));
  }
  top->cpu[slot] = delta;
  top->cpuPid[slot] = pid;
  const char *name = proc_best_name(p);
  strlcpy(top->cpuName[slot], name ? name : "?", sizeof(top->cpuName[slot]));
}
static int guampMemTopVisit(struct proc *p, void *arg) {
  GuampMemTop *top = static_cast<GuampMemTop *>(arg);
  task_t task = proc_task(p);
  if (!task) return 0;
  const uint64_t bytes = get_task_phys_footprint(task);
  const int pid = proc_pid(p);
  if (pid >= 0 && pid < 2048) {
    const uint64_t cpu = get_task_cpu_time(task);
    const uint64_t delta = guampCpuSeen[pid] && cpu >= guampCpuLast[pid] ? cpu - guampCpuLast[pid] : 0;
    guampCpuLast[pid] = cpu;
    guampCpuSeen[pid] = 1;
    top->cpuTotal += delta;
    guampCpuTopInsert(top, p, pid, delta);
  }
  top->procs = top->procs + 1;
  top->total += bytes;
  unsigned slot = 6;
  while (slot > 0 && top->bytes[slot - 1] < bytes) slot = slot - 1;
  if (slot >= 6) return 0;
  for (unsigned i = 5; i > slot; i = i - 1) {
    top->bytes[i] = top->bytes[i - 1];
    top->pid[i] = top->pid[i - 1];
    memcpy(top->name[i], top->name[i - 1], sizeof(top->name[i]));
  }
  top->bytes[slot] = bytes;
  top->pid[slot] = proc_pid(p);
  const char *name = proc_best_name(p);
  strlcpy(top->name[slot], name ? name : "?", sizeof(top->name[slot]));
  return 0;   // PROC_RETURNED
}

static void guampMemTopCall(thread_call_param_t, thread_call_param_t);

static void guampMemSample() {
  static unsigned round = 0;
  vm_statistics64_data_t vm = {};
  mach_msg_type_number_t count = HOST_VM_INFO64_COUNT;
  kern_return_t result = host_statistics64(host_self(), HOST_VM_INFO64, reinterpret_cast<host_info64_t>(&vm), &count);
  if (result == KERN_SUCCESS) {
    const unsigned long long mb = PAGE_SIZE / 1024;   // pages -> KiB, printed as MiB below
    IOLog("GuampMem: MiB free %llu active %llu inactive %llu speculative %llu wired %llu compressed %llu (in %llu) "
          "file %llu anon %llu purgeable %llu; pageins %llu pageouts %llu compressions %llu decompressions %llu swapouts %llu\n",
          vm.free_count * mb / 1024, vm.active_count * mb / 1024, vm.inactive_count * mb / 1024,
          vm.speculative_count * mb / 1024, vm.wire_count * mb / 1024, vm.compressor_page_count * mb / 1024,
          vm.total_uncompressed_pages_in_compressor * mb / 1024, vm.external_page_count * mb / 1024,
          vm.internal_page_count * mb / 1024, vm.purgeable_count * mb / 1024, vm.pageins, vm.pageouts,
          vm.compressions, vm.decompressions, vm.swapouts);
  } else {
    IOLog("GuampMem: VM statistics unavailable, status %d count %u\n", result, count);
  }
  round = round + 1;
  if (round % 3) return;
  // The caller is the eMMC service path: walking the process list there could wait on a proc lock held by a task
  // that waits for eMMC I/O. The walk runs on its own thread call instead.
  static thread_call_t topCall = nullptr;
  if (!topCall) topCall = thread_call_allocate(guampMemTopCall, nullptr);
  if (topCall) thread_call_enter(topCall);
}

static void guampMemTopCall(thread_call_param_t, thread_call_param_t) {
  GuampMemTop top = {};
  proc_iterate(1u /* PROC_ALLPROCLIST */, guampMemTopVisit, &top, nullptr, nullptr);
  IOLog("GuampMem: %u processes, footprint %llu MiB; top %s[%d] %llu, %s[%d] %llu, %s[%d] %llu, %s[%d] %llu, %s[%d] %llu, %s[%d] %llu MiB\n",
        top.procs, top.total >> 20, top.name[0], top.pid[0], top.bytes[0] >> 20, top.name[1], top.pid[1], top.bytes[1] >> 20,
        top.name[2], top.pid[2], top.bytes[2] >> 20, top.name[3], top.pid[3], top.bytes[3] >> 20,
        top.name[4], top.pid[4], top.bytes[4] >> 20, top.name[5], top.pid[5], top.bytes[5] >> 20);
  // Per-process CPU since the previous walk, in permille of one core over the walk interval.
  static uint64_t lastWalk = 0;
  const uint64_t now = mach_absolute_time();
  const uint64_t span = lastWalk && now > lastWalk ? now - lastWalk : 0;
  lastWalk = now;
  if (!span) return;
  unsigned long long pm[6];
  for (unsigned i = 0; i < 6; ++i) pm[i] = top.cpu[i] * 1000 / span;
  IOLog("GuampCpu: all processes %llu%% of one core; top %s[%d] %llu.%llu, %s[%d] %llu.%llu, %s[%d] %llu.%llu, "
        "%s[%d] %llu.%llu, %s[%d] %llu.%llu, %s[%d] %llu.%llu %% of one core\n",
        (unsigned long long)(top.cpuTotal * 100 / span),
        top.cpuName[0], top.cpuPid[0], pm[0] / 10, pm[0] % 10, top.cpuName[1], top.cpuPid[1], pm[1] / 10, pm[1] % 10,
        top.cpuName[2], top.cpuPid[2], pm[2] / 10, pm[2] % 10, top.cpuName[3], top.cpuPid[3], pm[3] / 10, pm[3] % 10,
        top.cpuName[4], top.cpuPid[4], pm[4] / 10, pm[4] % 10, top.cpuName[5], top.cpuPid[5], pm[5] / 10, pm[5] % 10);
}
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
    guampMemSample();
    if (guamp_gpu_step[2]) IOLog("GuampGpu: progress step %u reg %x seq %u\n", guamp_gpu_step[0], guamp_gpu_step[1], guamp_gpu_step[2]);
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
