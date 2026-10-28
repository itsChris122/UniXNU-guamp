// SPDX-License-Identifier: BSD-2-Clause-Patent
// Read-only accounting through the kernel host API. No frequency/voltage changes.
#include <kern/host.h>
#include <mach/host_info.h>
#include <kern/thread_call.h>
#include <kern/clock.h>
#include <kern/timer_call.h>
#include <kern/sched_prim.h>
#include <kern/processor.h>
#include <kern/thread.h>
#include <vm/vm_shared_region.h>
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
// Last risky MMIO access before it is issued: [0] tag ('A'/'a' GuampAdrenoProbe read/write, 'M'/'m' GuampGpuM2 read/write),
// [1] register (dword index), [2] sequence. A core stalled on an access that never completes leaves it here; the stuck-core
// line below prints it. The store is ordered before the access (dmb), so it is visible while that core hangs.
extern "C" volatile uint32_t guamp_mmio_tag[3];
volatile uint32_t guamp_mmio_tag[3];
#define GUAMP_MMIO_TAG(t, r)                                   do {                                                           guamp_mmio_tag[0] = (t);                                     guamp_mmio_tag[1] = (r);                                     guamp_mmio_tag[2] = guamp_mmio_tag[2] + 1;                   __asm__ volatile("dmb ish" ::: "memory");                  } while (0)
extern "C" uint64_t guamp_stuck_cpu_mask(void);   // zzzz-guamp-stuck-cpu.patch (osfmk/arm/model_dep.c)
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

// guamp-prof=1: user PC sampler. Every 30 s, after the GuampCpu walk, the three busiest processes are sampled for 1 s by a
// 10 ms timer callout armed on each CPU (TIMER_CALL_LOCAL: it runs in the interrupt context of whatever thread that CPU was
// running, so current_thread() is the sampled thread). Only the thread's saved user state (upcb) is read, never user
// memory; a thread interrupted in the kernel shows the user PC it entered the kernel from. PCs inside the dyld shared
// cache are logged as offsets from the region's first mapping (strumenti/simboli-prof.py resolves them).
extern "C" struct arm_saved_state *get_user_regs(thread_t);
extern "C" struct proc *proc_find(int);
extern "C" int proc_rele(struct proc *);
extern "C" processor_t thread_bind(processor_t);   // kern/sched_prim.h (MACH_KERNEL_PRIVATE part)
namespace guampprof {
constexpr unsigned kCpus = 8, kTargets = 3, kSlots = 128, kTids = 8;
struct Hist { uint64_t pc[kSlots]; uint32_t n[kSlots]; uint64_t tid[kTids]; uint32_t tidN[kTids]; uint32_t total, lost; };
static Hist gHist[kCpus][kTargets];
static volatile int gPid[kTargets] = {-1, -1, -1};
static char gName[kTargets][20];
static volatile int gNextPid[kTargets] = {-1, -1, -1};
static char gNextName[kTargets][20];
static volatile uint32_t gActive;
static uint64_t gPeriod;
static timer_call_data_t gCall[kCpus];
static bool gStarted;

static void record(Hist &h, uint64_t pc, uint64_t tid) {
  h.total = h.total + 1;
  const unsigned i = static_cast<unsigned>(((pc >> 2) * 2654435761ULL) % kSlots);
  bool placed = false;
  for (unsigned probe = 0; probe < 8 && !placed; probe = probe + 1) {
    const unsigned s = (i + probe) % kSlots;
    if (h.n[s] && h.pc[s] == pc) { h.n[s] = h.n[s] + 1; placed = true; }
    else if (!h.n[s]) { h.pc[s] = pc; h.n[s] = 1; placed = true; }
  }
  if (!placed) h.lost = h.lost + 1;
  for (unsigned k = 0; k < kTids; k = k + 1) {
    if (h.tidN[k] && h.tid[k] == tid) { h.tidN[k] = h.tidN[k] + 1; return; }
    if (!h.tidN[k]) { h.tid[k] = tid; h.tidN[k] = 1; return; }
  }
}

static void tick(timer_call_param_t p0, timer_call_param_t) {
  const unsigned cpu = static_cast<unsigned>(reinterpret_cast<uintptr_t>(p0));
  if (!gActive || cpu >= kCpus) return;
  thread_t th = current_thread();
  task_t task = th ? get_threadtask(th) : nullptr;
  if (task && task != kernel_task) {
    struct proc *p = static_cast<struct proc *>(get_bsdtask_info(task));
    const int pid = p ? proc_pid(p) : -1;
    for (unsigned k = 0; k < kTargets; k = k + 1) {
      if (pid < 0 || pid != gPid[k]) continue;
      struct arm_saved_state *rs = get_user_regs(th);
      if (rs && rs->ash.flavor == ARM_SAVED_STATE64) record(gHist[cpu][k], rs->ss_64.pc, thread_tid(th));
      break;
    }
  }
  timer_call_enter(&gCall[cpu], mach_absolute_time() + gPeriod, TIMER_CALL_LOCAL | TIMER_CALL_SYS_CRITICAL);
}

static uint64_t gPcs[kCpus * kSlots];
static uint32_t gNs[kCpus * kSlots];

static void report(unsigned k) {
  unsigned used = 0, tidsUsed = 0;
  uint32_t total = 0, lost = 0, bestTidN = 0;
  uint64_t bestTid = 0;
  uint64_t tids[kCpus * kTids];
  uint32_t tidNs[kCpus * kTids];
  for (unsigned c = 0; c < kCpus; c = c + 1) {
    const Hist &h = gHist[c][k];
    total += h.total;
    lost += h.lost;
    for (unsigned s = 0; s < kSlots; s = s + 1) {
      if (!h.n[s]) continue;
      unsigned j = 0;
      while (j < used && gPcs[j] != h.pc[s]) j = j + 1;
      if (j == used) { gPcs[j] = h.pc[s]; gNs[j] = 0; used = used + 1; }
      gNs[j] += h.n[s];
    }
    for (unsigned s = 0; s < kTids; s = s + 1) {
      if (!h.tidN[s]) continue;
      unsigned j = 0;
      while (j < tidsUsed && tids[j] != h.tid[s]) j = j + 1;
      if (j == tidsUsed) { tids[j] = h.tid[s]; tidNs[j] = 0; tidsUsed = tidsUsed + 1; }
      tidNs[j] += h.tidN[s];
    }
  }
  for (unsigned j = 0; j < tidsUsed; j = j + 1)
    if (tidNs[j] > bestTidN) { bestTidN = tidNs[j]; bestTid = tids[j]; }
  if (!total) return;
  // The process's shared cache region start (its slid first mapping).
  mach_vm_offset_t start = 0;
  struct proc *p = proc_find(gPid[k]);
  if (p) {
    task_t task = proc_task(p);
    vm_shared_region_t sr = task ? vm_shared_region_get(task) : nullptr;
    if (sr) {
      if (vm_shared_region_start_address(sr, &start, task) != KERN_SUCCESS) start = 0;
      vm_shared_region_deallocate(sr);
    }
    proc_rele(p);
  }
  char line[640];
  int len = snprintf(line, sizeof(line), "GuampProf: %s[%d] samples %u (lost %u, top thread %llu %u%%, %u threads):", gName[k],
                     gPid[k], total, lost, static_cast<unsigned long long>(bestTid), bestTidN * 100 / total, tidsUsed);
  for (unsigned r = 0; r < 6 && len > 0 && static_cast<size_t>(len) < sizeof(line); r = r + 1) {
    unsigned best = used;
    for (unsigned j = 0; j < used; j = j + 1)
      if (gNs[j] && (best == used || gNs[j] > gNs[best])) best = j;
    if (best == used) break;
    const uint64_t pc = gPcs[best];
    const bool inCache = start && pc >= start && pc < start + 0x200000000ULL;
    len += snprintf(line + len, sizeof(line) - static_cast<size_t>(len), " %s0x%llx %u%%", inCache ? "sc+" : "pc ",
                    static_cast<unsigned long long>(inCache ? pc - start : pc), gNs[best] * 100 / total);
    gNs[best] = 0;
  }
  IOLog("%s\n", line);
}

static void loop(void *, wait_result_t) {
  for (;;) {
    IOSleep(10000);
    bool any = false;
    for (unsigned k = 0; k < kTargets; k = k + 1) {
      gPid[k] = gNextPid[k];
      memcpy(gName[k], gNextName[k], sizeof(gName[k]));
      any = any || gPid[k] > 0;
    }
    if (!any) continue;
    bzero(gHist, sizeof(gHist));
    gActive = 1;
    for (unsigned c = 0; c < kCpus; c = c + 1) {
      processor_t pr = cpu_to_processor(static_cast<int>(c));
      if (!pr) continue;
      thread_bind(pr);
      thread_block(THREAD_CONTINUE_NULL);
      timer_call_enter(&gCall[c], mach_absolute_time() + gPeriod, TIMER_CALL_LOCAL | TIMER_CALL_SYS_CRITICAL);
    }
    thread_bind(PROCESSOR_NULL);
    thread_block(THREAD_CONTINUE_NULL);
    IOSleep(1000);
    gActive = 0;
    IOSleep(40);
    for (unsigned k = 0; k < kTargets; k = k + 1)
      if (gPid[k] > 0) report(k);
    IOSleep(19000);
  }
}

// From the GuampCpu walk: the next round samples these processes (kernel_task excluded).
static void setTargets(const int *pids, const char (*names)[20], unsigned n) {
  static int enabled = -1;
  if (enabled < 0) {
    unsigned v = 0;
    enabled = PE_parse_boot_argn("guamp-prof", &v, sizeof(v)) && v == 1;
  }
  if (!enabled) return;
  unsigned k = 0;
  for (unsigned i = 0; i < n && k < kTargets; i = i + 1) {
    if (pids[i] <= 0) continue;
    gNextPid[k] = pids[i];
    strlcpy(gNextName[k], names[i], sizeof(gNextName[k]));
    k = k + 1;
  }
  for (; k < kTargets; k = k + 1) gNextPid[k] = -1;
  if (gStarted) return;
  gStarted = true;
  nanoseconds_to_absolutetime(10 * NSEC_PER_MSEC, &gPeriod);
  for (unsigned c = 0; c < kCpus; c = c + 1)
    timer_call_setup(&gCall[c], tick, reinterpret_cast<timer_call_param_t>(static_cast<uintptr_t>(c)));
  thread_t th = nullptr;
  if (kernel_thread_start(loop, nullptr, &th) == KERN_SUCCESS) {
    thread_set_thread_name(th, "guamp-prof");
    thread_deallocate(th);
    IOLog("GuampProf: sampler on (1 s every 30 s, 10 ms per CPU)\n");
  }
}
}  // namespace guampprof

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
  guampprof::setTargets(top.cpuPid, top.cpuName, 6);
}
extern char guamp_cpufreq_summary[2][160];   // GuampCpufreq.inc

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
    static unsigned cpufreqRound = 0;
    cpufreqRound = cpufreqRound + 1;
    if (cpufreqRound % 6 == 1)
      for (unsigned d = 0; d < 2; ++d)
        if (guamp_cpufreq_summary[d][0]) IOLog("GuampCpufreq: %s\n", guamp_cpufreq_summary[d]);
    if (guamp_gpu_step[2]) IOLog("GuampGpu: progress step %u reg %x seq %u\n", guamp_gpu_step[0], guamp_gpu_step[1], guamp_gpu_step[2]);
    // Per-core heartbeat: a core whose timer and IPI counts (ArmCpu) did not move over the sample is quiet; the debugger mask
    // marks cores that never answered a stackshot IPI. Printed while anything is wrong (first 3 times, then every minute).
    {
      static uint32_t lastBeat[8];
      static unsigned reports = 0;
      uint32_t quiet = 0;
      const unsigned cores = basicResult == KERN_SUCCESS && basic.max_cpus > 0 && basic.max_cpus <= 8 ? basic.max_cpus : 0;
      for (unsigned cpu = 0; cpu < cores; ++cpu) {
        const uint32_t beat = armcpu_core_timer[cpu] + armcpu_core_sgi[cpu];
        if (beat == lastBeat[cpu]) quiet |= 1u << cpu;
        lastBeat[cpu] = beat;
      }
      const uint64_t silent = guamp_stuck_cpu_mask();
      if (quiet || silent) {
        reports = reports + 1;
        if (reports <= 3 || reports % 6 == 0)
          IOLog("GuampStuck: quiet cores %02x (timer+IPI unchanged for 10 s), debugger-silent %02llx, last MMIO %c reg %x seq %u\n",
                quiet, static_cast<unsigned long long>(silent), guamp_mmio_tag[0] ? static_cast<char>(guamp_mmio_tag[0]) : '-',
                guamp_mmio_tag[1], guamp_mmio_tag[2]);
      }
    }
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
