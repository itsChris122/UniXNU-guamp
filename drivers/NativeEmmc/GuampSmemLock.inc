// SPDX-License-Identifier: BSD-2-Clause-Patent
// The ONE SMEM allocation lock of the kernel (qcom_smem_alloc takes hwspin_lock_timeout_irqsave on the SMEM hwlock):
// a local IOLock, because the TCSR mutex cannot tell two apps threads apart (both write owner 1 and read back 1),
// then the hardware mutex shared with the remote processors. Every apps-side SMEM writer passes this pair as its
// SmemArena lock (GuampSmemWrite.inc): the ADSP transports (GuampAdspIpc.inc, guamp-adsp thread) and the Wi-Fi R1
// provider (drivers/GuampWifi/integration/guamp_xnu_sm6115_provider.h, another translation unit: C linkage).
// guamp DT: hwlocks = <&tcsr_mutex 3>, tcsr_mutex syscon 0x340000 stride 0x1000 (lock 3 = 0x343000); apps owner id 1
// (qcom_hwspinlock.c: write the id, read back the id = owned, write 0 = release). Bounded wait 1 s, as before in
// both callers; the hardware mutex is held only around a few uncached SMEM stores (no log, no sleep inside).
// Needs mapPhysical (NativeEmmc.cpp), guampModemBarrier (GuampScmArm64.inc), IOLock*, IOSleep.
#ifndef GUAMP_SMEM_LOCK_INC
#define GUAMP_SMEM_LOCK_INC
namespace smemlock {
constexpr uint64_t kMutexPhys = 0x00340000 + 3 * 0x1000;
constexpr uint32_t kAppsOwner = 1, kTries = 1000;
constinit IOLock *gLocal = nullptr;
constinit volatile uint32_t *gMutex = nullptr;   // mapped once, under gLocal
constinit uint32_t gTimeouts = 0;

IOLock *local() {
  IOLock *l = __atomic_load_n(&gLocal, __ATOMIC_ACQUIRE);
  if (l) return l;
  IOLock *fresh = IOLockAlloc();
  if (!fresh) return nullptr;
  IOLock *expected = nullptr;
  if (__atomic_compare_exchange_n(&gLocal, &expected, fresh, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) return fresh;
  IOLockFree(fresh);   // another thread won the race
  return expected;
}
}  // namespace smemlock

extern "C" bool guamp_smem_hwlock_acquire(void) {
  IOLock *l = smemlock::local();
  if (!l) return false;
  IOLockLock(l);
  if (!smemlock::gMutex) smemlock::gMutex = reinterpret_cast<volatile uint32_t *>(mapPhysical(smemlock::kMutexPhys, 0x1000));
  if (smemlock::gMutex) {
    for (uint32_t n = 0; n < smemlock::kTries; n++) {
      *smemlock::gMutex = smemlock::kAppsOwner;
      guampModemBarrier();
      if (*smemlock::gMutex == smemlock::kAppsOwner) return true;
      IOSleep(1);
    }
  }
  smemlock::gTimeouts = smemlock::gTimeouts + 1;
  IOLockUnlock(l);
  return false;
}

extern "C" void guamp_smem_hwlock_release(void) {
  guampModemBarrier();   // every SMEM store of the critical section before the release
  *smemlock::gMutex = 0;
  guampModemBarrier();
  IOLockUnlock(smemlock::gLocal);
}
#endif
