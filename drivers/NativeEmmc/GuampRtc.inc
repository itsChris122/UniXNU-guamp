// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp wall clock, included by NativeEmmc.cpp after PmicService.inc (pmic::read8/ensureArb, the Card and kpan).
// GuampRtcCore.h has the PM6125 RTC registers, the record format and the rules; this file is the kernel side:
//   boot     rtc::boot() runs in startLater before the eMMC media are published, so before the root mount and
//            launchd: reads the RTC counter (read only, never written) and the two offset records (the last two
//            blocks of kpan), sets XNU's calendar with clock_set_calendar_microtime and keeps it as the platform
//            clock answer (guamp_rtc_get, used by ArmDTPlatform::getUTCTimeOfDay, so a later
//            clock_initialize_calendar gets the same time)
//   set      every settimeofday (timed with NTP, Settings -> General -> Date & Time by hand) ends in
//            PESetUTCTimeOfDay -> ArmDTPlatform::setUTCTimeOfDay -> guamp_rtc_set: only notes the time and wakes a
//            thread call (nothing from the syscall context); the thread call reads the counter and writes
//            offset = UTC - counter when it moved by more than kSaveThreshold seconds (alternating slots + flush cache)
//   flush    guamp_emmc_flush (restart/power off, not panic) writes a set that is still pending
// Boot argument guamp-rtc=0: none of this (XNU keeps its 1970 clock). Log lines start with "GuampRtc:".
#include "GuampRtcCore.h"

namespace rtc {
struct Pending { int64_t secs; uint32_t usecs; uint64_t abs; };

bool gEnabled = false;
volatile bool gReady = false;
volatile bool gSelfSet = false;                       // our own clock_set_calendar_microtime at boot
IOSimpleLock *gLock = nullptr;                        // gBase*, gPending, gReq
int64_t gBaseUtc = 0;
uint32_t gBaseUsec = 0;
uint64_t gBaseAbs = 0;                                // wall clock gBaseUtc.gBaseUsec at mach time gBaseAbs
bool gPending = false;
Pending gReq = {};
IOLock *gSaveLock = nullptr;                          // gRec, gHaveRecord, gNewest, gBlock
guamp_rtc::Record gRec;
bool gHaveRecord = false;
int gNewest = -1;
uint8_t *gBlock = nullptr;
thread_call_t gCall = nullptr;
uint32_t gSaves = 0, gErrors = 0;

int readCounter(uint32_t *counter) {
  return guamp_rtc::readCounter([](uint16_t addr, uint8_t *v) { return pmic::read8(guamp_rtc::kSid, addr, v); }, counter);
}
uint64_t slotIndex(unsigned slot) { return gCard.logBlocks - 2 + slot; }

// One record slot (under gSaveLock): false when unreadable or not a valid record
bool loadSlot(unsigned slot, guamp_rtc::Record *rec) {
  Card &card = gCard;
  IOLockLock(card.gate);
  bool ok = card.host.readBlock(card.logFirst + slotIndex(slot), gBlock);
  IOLockUnlock(card.gate);
  return ok && guamp_rtc::decode(gBlock, rec);
}

// Writes the record for utc at counter into the slot that does not hold the newest one, checks it by reading back
bool save(int64_t utc, uint32_t counter, bool trusted) {
  Card &card = gCard;
  IOLockLock(gSaveLock);
  guamp_rtc::Record rec = guamp_rtc::makeRecord(gRec, gHaveRecord, utc, counter, trusted);
  unsigned slot = guamp_rtc::nextSlot(gNewest);
  guamp_rtc::encode(rec, gBlock);
  IOLockLock(card.gate);
  bool ok = card.writeLogBlock(slotIndex(slot), gBlock);
  if (ok) card.host.flushCache();
  guamp_rtc::Record back;
  ok = ok && card.host.readBlock(card.logFirst + slotIndex(slot), gBlock) && guamp_rtc::decode(gBlock, &back) &&
       back.sequence == rec.sequence && back.offset == rec.offset;
  IOLockUnlock(card.gate);
  if (ok) {
    gRec = rec;
    gHaveRecord = true;
    gNewest = static_cast<int>(slot);
    gSaves++;
    IOLog("GuampRtc: offset %lld saved (UTC %lld, counter %u, %s, kpan slot %u, seq %u)\n", rec.offset, rec.utc,
          rec.counter, trusted ? "from settimeofday" : "boot estimate", slot, rec.sequence);
  } else {
    gErrors++;
    IOLog("GuampRtc: offset %lld NOT saved: write or readback of kpan slot %u failed\n", rec.offset, slot);
  }
  IOLockUnlock(gSaveLock);
  return ok;
}

// The newest settimeofday, if any: counter now, UTC now = the set time + the time since, saved when it changed
bool savePending(const char *why) {
  IOSimpleLockLock(gLock);
  if (!gPending) { IOSimpleLockUnlock(gLock); return true; }
  Pending p = gReq;
  gPending = false;
  IOSimpleLockUnlock(gLock);
  uint32_t counter = 0;
  int r = readCounter(&counter);
  uint64_t since = 0;
  absolutetime_to_nanoseconds(mach_absolute_time() - p.abs, &since);
  int64_t utc = p.secs + static_cast<int64_t>((p.usecs * 1000ULL + since) / 1000000000ULL);
  if (r) {
    gErrors++;
    IOSimpleLockLock(gLock);
    if (!gPending) { gReq = p; gPending = true; }    // the restart flush tries again
    IOSimpleLockUnlock(gLock);
    IOLog("GuampRtc: time set to %lld (%s) but RTC read failed (%d), offset not saved\n", utc, why, r);
    return false;
  }
  IOLockLock(gSaveLock);
  bool need = guamp_rtc::needsSave(gHaveRecord, gRec, utc, counter);
  int64_t old = gHaveRecord ? gRec.offset : 0;
  IOLockUnlock(gSaveLock);
  if (!need) {
    IOLog("GuampRtc: time set to %lld (%s), offset %lld unchanged (counter %u)\n", utc, why, old, counter);
    return true;
  }
  IOLog("GuampRtc: time set to %lld (%s), counter %u, offset %lld -> %lld\n", utc, why, counter, old,
        utc - static_cast<int64_t>(counter));
  return save(utc, counter, true);
}

void worker(thread_call_param_t, thread_call_param_t) { savePending("settimeofday"); }

void boot() {
  unsigned on = 1;
  PE_parse_boot_argn("guamp-rtc", &on, sizeof(on));
  if (!on) { IOLog("GuampRtc: disabled (guamp-rtc=0), wall clock left to XNU\n"); return; }
  Card &card = gCard;
  if (card.logBlocks < kPanicFirstBlock + kPanicBytes / kBlock + 2) { IOLog("GuampRtc: kpan too small, off\n"); return; }
  gLock = IOSimpleLockAlloc();
  gSaveLock = IOLockAlloc();
  gBlock = static_cast<uint8_t *>(IOMallocZero(kBlock));
  gCall = thread_call_allocate(worker, nullptr);
  if (!gLock || !gSaveLock || !gBlock || !gCall) { IOLog("GuampRtc: allocation failed, off\n"); return; }
  uint32_t counter = 0;
  int arb = pmic::ensureArb();
  int cr = arb ? arb : readCounter(&counter);
  guamp_rtc::Record a, b;
  bool okA = loadSlot(0, &a), okB = loadSlot(1, &b);
  gNewest = guamp_rtc::pick(okA, a, okB, b);
  gHaveRecord = gNewest >= 0;
  if (gHaveRecord) gRec = gNewest ? b : a;
  int64_t floor = guamp_rtc::floorUtc(__DATE__);
  guamp_rtc::BootTime t = guamp_rtc::bootTime(cr == 0, counter, gHaveRecord, gRec, floor);
  static const char *const kSource[] = {"counter + offset", "saved UTC (counter behind the record)", "floor (build date)"};
  IOSimpleLockLock(gLock);
  gBaseUtc = t.utc;
  gBaseUsec = 0;
  gBaseAbs = mach_absolute_time();
  IOSimpleLockUnlock(gLock);
  __asm__ volatile("dmb ish" ::: "memory");
  gReady = true;
  gEnabled = true;
  IOLog("GuampRtc: PM6125 RTC %s, counter %u; record %s (slot %d seq %u offset %lld %s); floor %lld; wall clock %lld from %s\n",
        cr == 0 ? "running" : cr == 1 ? "NOT enabled (RTC_EN 0)" : "read FAILED", counter,
        gHaveRecord ? "found" : "none", gNewest, gHaveRecord ? gRec.sequence : 0, gHaveRecord ? gRec.offset : 0LL,
        gHaveRecord && (gRec.flags & guamp_rtc::kTrusted) ? "trusted" : "estimate", floor, t.utc, kSource[t.source]);
  if (cr < 0) IOLog("GuampRtc: RTC read error %d (SPMI arbiter %d)\n", cr, arb);
  gSelfSet = true;
  clock_set_calendar_microtime(static_cast<clock_sec_t>(t.utc), 0);
  gSelfSet = false;
  // First boot or a reset counter: remember this estimate so the next boot continues from here instead of the floor
  if (cr == 0 && t.source != guamp_rtc::kFromOffset) save(t.utc, counter, false);
}

// Restart / power off (not panic): a settimeofday whose thread call has not run yet
void flush() {
  if (!gEnabled) return;
  savePending("restart flush");
}
}  // namespace rtc

// ArmDTPlatform::getUTCTimeOfDay: 1 with the wall clock, 0 before rtc::boot (or guamp-rtc=0): the platform default
extern "C" int guamp_rtc_get(clock_sec_t *secs, clock_nsec_t *nsecs)
{
  if (!rtc::gReady || !secs || !nsecs) return 0;
  IOSimpleLockLock(rtc::gLock);
  int64_t base = rtc::gBaseUtc;
  uint32_t usec = rtc::gBaseUsec;
  uint64_t abs = rtc::gBaseAbs;
  IOSimpleLockUnlock(rtc::gLock);
  uint64_t ns = 0;
  absolutetime_to_nanoseconds(mach_absolute_time() - abs, &ns);
  ns += usec * 1000ULL;
  *secs = static_cast<clock_sec_t>(base + static_cast<int64_t>(ns / 1000000000ULL));
  *nsecs = static_cast<clock_nsec_t>(ns % 1000000000ULL);
  return 1;
}

// ArmDTPlatform::setUTCTimeOfDay, from clock_set_calendar_microtime (settimeofday): note it, save in a thread call
extern "C" int guamp_rtc_set(clock_sec_t secs, clock_nsec_t nsecs)
{
  if (!rtc::gEnabled) return 0;
  if (rtc::gSelfSet) return 1;
  uint64_t now = mach_absolute_time();
  IOSimpleLockLock(rtc::gLock);
  rtc::gBaseUtc = static_cast<int64_t>(secs);
  rtc::gBaseUsec = static_cast<uint32_t>(nsecs / 1000);
  rtc::gBaseAbs = now;
  rtc::gReq = {static_cast<int64_t>(secs), static_cast<uint32_t>(nsecs / 1000), now};
  rtc::gPending = true;
  IOSimpleLockUnlock(rtc::gLock);
  thread_call_enter(rtc::gCall);
  return 1;
}
