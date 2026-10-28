/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * First physical XNU layer for the Moto G9 Play / SM6115 Wi-Fi provider.
 * Addresses and lock protocol are taken from the captured Guamp DT and the
 * Qualcomm TCSR mutex driver. Resource owners and RPM transport/power stepping
 * are provided here; the runtime owns modem/WLFW/SMMU/radio sequencing.
 */
#ifndef GUAMP_XNU_SM6115_PROVIDER_H
#define GUAMP_XNU_SM6115_PROVIDER_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOLib.h>
#include <kern/locks.h>
#include <mach/mach_types.h>
#include "GuampSmem.inc"
#include "GuampSmemWrite.inc"
#include "GuampWifiPower.inc"

// The kernel's one SMEM allocation lock (NativeEmmc GuampSmemLock.inc: local IOLock + TCSR mutex lock 3, owner 1),
// shared with the ADSP transports of audio phase 2 so that the two SMEM writers exclude each other.
extern "C" bool guamp_smem_hwlock_acquire(void);
extern "C" void guamp_smem_hwlock_release(void);

namespace guampwifi {
struct GuampXnuRegion {
  GuampXnuRegion()=default;
  GuampXnuRegion(const GuampXnuRegion &)=delete;
  GuampXnuRegion &operator=(const GuampXnuRegion &)=delete;
  GuampXnuRegion(GuampXnuRegion &&)=delete;
  GuampXnuRegion &operator=(GuampXnuRegion &&)=delete;
  IOMemoryDescriptor *descriptor=nullptr;
  IOMemoryMap *map=nullptr;
  volatile uint8_t *bytes=nullptr;
  uint64_t physical=0;
  uint32_t size=0;

  bool open(uint64_t at,uint32_t length,IODirection direction,
            uint64_t ramBase,uint64_t ramBytes) {
    if(map||!length||!ramBytes||at>UINT64_MAX-length||ramBase>UINT64_MAX-ramBytes||
       (at<ramBase+ramBytes&&ramBase<at+length)) return false;
    descriptor=IOMemoryDescriptor::withPhysicalAddress(at,length,direction);
    map=descriptor?descriptor->map(kIOMapInhibitCache):nullptr;
    if(!map) { close(); return false; }
    bytes=reinterpret_cast<volatile uint8_t *>(map->getVirtualAddress());
    if(!bytes) { close(); return false; }
    physical=at;size=length;return true;
  }
  void close() {
    bytes=nullptr;physical=0;size=0;
    if(map){map->release();map=nullptr;}
    if(descriptor){descriptor->release();descriptor=nullptr;}
  }
  uint32_t read32(uint32_t offset) const {
    if(!bytes||(offset&3)||offset>size||size-offset<4)return 0xffffffffu;
    __asm__ volatile("dmb oshld" ::: "memory");
    uint32_t value=*reinterpret_cast<volatile uint32_t *>(bytes+offset);
    __asm__ volatile("dmb oshld" ::: "memory");return value;
  }
  void write32(uint32_t offset,uint32_t value) const {
    if(!bytes||(offset&3)||offset>size||size-offset<4)return;
    __asm__ volatile("dmb oshst" ::: "memory");
    *reinterpret_cast<volatile uint32_t *>(bytes+offset)=value;
    __asm__ volatile("dsb sy" ::: "memory");
  }
};

// Contiguous, uncached XNU-owned DRAM for SCM payloads and SMMU tables. The
// allocation's cache policy is set at creation, avoiding a cached allocator
// alias alongside a new uncached mapping. Call retainForDevice() BEFORE its
// physical address is published to SCM/SMMU; failed/uncertain operations then
// keep the pages wired until the runtime can prove all remote access stopped.
struct GuampXnuBuffer {
  GuampXnuBuffer()=default;
  GuampXnuBuffer(const GuampXnuBuffer &)=delete;
  GuampXnuBuffer &operator=(const GuampXnuBuffer &)=delete;
  GuampXnuBuffer(GuampXnuBuffer &&)=delete;
  GuampXnuBuffer &operator=(GuampXnuBuffer &&)=delete;
  IOBufferMemoryDescriptor *descriptor=nullptr;
  IOMemoryMap *map=nullptr;
  volatile uint8_t *bytes=nullptr;
  uint64_t physical=0;
  uint32_t size=0;
  Memory memory{};
  bool prepared=false,retained=false;

  bool open(uint32_t length,uint64_t physicalMask,uint64_t managedBase,uint64_t managedBytes) {
    if(descriptor||map||!length||length>UINT32_MAX-4095u||!managedBytes||
       managedBase>UINT64_MAX-managedBytes)return false;
    const uint32_t rounded=(length+4095u)&~4095u;
    const uint64_t allocationMask=physicalMask&~uint64_t(4095);
    const uint64_t allowedMask=allocationMask|4095u;
    if(!allocationMask||uint64_t(rounded-1)>allowedMask)return false;
    const IOOptionBits options=static_cast<IOOptionBits>(kIODirectionInOut)|
      static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous)|
      static_cast<IOOptionBits>(kIOMemoryMapperNone)|static_cast<IOOptionBits>(kIOMapInhibitCache);
    descriptor=IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,options,rounded,allocationMask);
    if(!descriptor)return false;
    if(descriptor->prepare()!=kIOReturnSuccess){close();return false;}
    prepared=true;
    IOByteCount contiguous=0;
    const uint64_t pa=descriptor->getPhysicalSegment(0,&contiguous,kIOMemoryMapperNone);
    if(!pa||(pa&4095u)||contiguous<rounded||pa>UINT64_MAX-(rounded-1)||
       ((pa|uint64_t(pa+rounded-1))&~allowedMask)||pa<managedBase||
       pa-managedBase>=managedBytes||rounded>managedBytes-(pa-managedBase)) {
      close();return false;
    }
    map=descriptor->map(kIOMapAnywhere|kIOMapInhibitCache);
    bytes=map?reinterpret_cast<volatile uint8_t *>(map->getVirtualAddress()):nullptr;
    if(!bytes){close();return false;}
    physical=pa;size=rounded;memory={this,size,read,write,barrier};
    // Aligned word writes also work if the target represents the uncached
    // mapping using Device attributes rather than Normal-NC.
    for(uint32_t at=0;at<size;at+=4)*reinterpret_cast<volatile uint32_t *>(bytes+at)=0;
    barrier(this);return true;
  }
  bool retainForDevice() {
    if(!descriptor||!map||!prepared||!physical)return false;
    retained=true;return true;
  }
  bool close(bool remoteStopped=false) {
    if(retained&&!remoteStopped)return false;
    memory={};bytes=nullptr;physical=0;size=0;
    if(map){map->release();map=nullptr;}
    if(descriptor){if(prepared)descriptor->complete();descriptor->release();descriptor=nullptr;}
    prepared=retained=false;return true;
  }
private:
  static uint32_t read(void *ctx,uint32_t at) {
    auto *p=static_cast<GuampXnuBuffer *>(ctx);
    if(!p||!p->bytes||(at&3)||at>p->size||p->size-at<4)return 0;
    return *reinterpret_cast<volatile uint32_t *>(p->bytes+at);
  }
  static void write(void *ctx,uint32_t at,uint32_t value) {
    auto *p=static_cast<GuampXnuBuffer *>(ctx);
    if(p&&p->bytes&&!(at&3)&&at<=p->size&&p->size-at>=4)
      *reinterpret_cast<volatile uint32_t *>(p->bytes+at)=value;
  }
  static void barrier(void *) { __asm__ volatile("dsb sy" ::: "memory"); }
};

struct GuampXnuSm6115Provider {
  GuampXnuSm6115Provider()=default;
  GuampXnuSm6115Provider(const GuampXnuSm6115Provider &)=delete;
  GuampXnuSm6115Provider &operator=(const GuampXnuSm6115Provider &)=delete;
  GuampXnuSm6115Provider(GuampXnuSm6115Provider &&)=delete;
  GuampXnuSm6115Provider &operator=(GuampXnuSm6115Provider &&)=delete;
  // Captured Guamp DT: smem_region and APCS mailbox; RPM message RAM,
  // apps-SMMU and ICNSS WLAN windows. The qcom,tcsr-mutex (0x340000, lock 3)
  // is owned by guamp_smem_hwlock_acquire/release.
  static constexpr uint64_t SmemPhys=0x46000000ull;
  static constexpr uint64_t ApcsPhys=0x0f111000ull;
  static constexpr uint64_t RpmPhys=0x045f0000ull;
  static constexpr uint64_t SmmuPhys=0x0c600000ull;
  static constexpr uint64_t WlanPhys=0x0c800000ull;
  static constexpr uint32_t SmemSize=0x00200000u;
  static constexpr uint32_t ApcsSize=0x00001000u;
  static constexpr uint32_t RpmSize=0x00007000u;
  static constexpr uint32_t SmmuSize=0x00080000u;
  static constexpr uint32_t WlanSize=0x00800000u;
  static constexpr uint32_t ApcsIpcOffset=8;

  GuampXnuRegion smemMap{},apcsMap{},rpmMap{},smmuMap{},wlanMap{};
  Memory sharedMemory{},rpmMemory{},smmuRegisters{},wlanRegisters{};
  SmemArena arena{};
  Allocator allocator{};
  Rpm rpm{};
  WifiPower power{};
  uint64_t xnuBase=0,xnuBytes=0;
  bool active=false,rpmStarted=false;

  bool start(uint64_t managedBase,uint64_t managedBytes) {
    if(active||!managedBytes||managedBase>UINT64_MAX-managedBytes)return false;
    xnuBase=managedBase;xnuBytes=managedBytes;
    // Do not partially initialize a provider whose physical windows overlap
    // XNU-managed RAM or cannot be mapped in their entirety.
    if(!smemMap.open(SmemPhys,SmemSize,kIODirectionInOut,xnuBase,xnuBytes)||
       !apcsMap.open(ApcsPhys,ApcsSize,kIODirectionInOut,xnuBase,xnuBytes)||
       !rpmMap.open(RpmPhys,RpmSize,kIODirectionInOut,xnuBase,xnuBytes)||
       !smmuMap.open(SmmuPhys,SmmuSize,kIODirectionInOut,xnuBase,xnuBytes)||
       !wlanMap.open(WlanPhys,WlanSize,kIODirectionInOut,xnuBase,xnuBytes)) {
      closeMaps();return false;
    }
    sharedMemory={this,SmemSize,smemRead,smemWrite,smemBarrier};
    rpmMemory={&rpmMap,RpmSize,regionRead,regionWrite,regionBarrier};
    smmuRegisters={&smmuMap,SmmuSize,regionRead,regionWrite,regionBarrier};
    wlanRegisters={&wlanMap,WlanSize,regionRead,regionWrite,regionBarrier};
    arena.mem=sharedMemory;arena.lockCtx=this;arena.lock=smemLock;arena.unlock=smemUnlock;
    arena.noteCtx=this;arena.note=smemNote;
    active=true;return true;
  }
  bool stop(bool pasStopped) {
    // Caller first joins every local worker and confirms PAS/WLAN stopped;
    // this flag does not stop those users on the caller's behalf. Shared RPM
    // votes are deliberately left installed for this boot.
    if((rpmStarted||power.proxyHeld||power.wlanHeld)&&!pasStopped)return false;
    if(rpmStarted)rpm.stop();
    rpmStarted=false;power={};allocator={};closeMaps();return true;
  }

  // APCS mailbox channel numbers are reflected in the captured board wiring:
  // channel 14 wakes SMP2P and channel 12 wakes the modem GLINK transport.
  // RPM channel 0 is kept separate because RPM GLINK has its own notification.
  void kickSmp2p() { apcsMap.write32(ApcsIpcOffset,1u<<14); }
  void kickModemGlink() { apcsMap.write32(ApcsIpcOffset,1u<<12); }
  void kickRpm() { apcsMap.write32(ApcsIpcOffset,1u); }

  int startRpm() {
    if(!active||rpmStarted)return Busy;
    allocator={this,allocate,freeAllocation};
    int e=rpm.start(rpmMemory,allocator,this,rpmNotify);
    if(!e)rpmStarted=true;
    return e;
  }
  int stepRpm(uint64_t nowMs) {
    if(!active||!rpmStarted)return NotReady;
    return rpm.step(nowMs);
  }
  int beginPowerVotes(bool forWlan) {
    if(!active||!rpmStarted||!rpm.online())return NotReady;
    return power.begin(&rpm,forWlan);
  }
  int stepPowerVotes(uint64_t nowMs) {
    if(!active||!rpmStarted)return NotReady;
    return power.step(nowMs);
  }

private:
  void closeMaps() {
    // Only used for pre-publication rollback or after stop()'s quiescence
    // contract. Do not call with an in-flight SmemArena lock holder.
    active=false;arena={};sharedMemory={};rpmMemory={};smmuRegisters={};wlanRegisters={};
    wlanMap.close();smmuMap.close();rpmMap.close();apcsMap.close();smemMap.close();
  }
  static void *allocate(void *,uint32_t bytes) { return bytes?IOMallocZero(bytes):nullptr; }
  static void freeAllocation(void *,void *p,uint32_t bytes) { if(p)IOFree(p,bytes); }
  static void rpmNotify(void *ctx) { auto *p=static_cast<GuampXnuSm6115Provider *>(ctx);if(p)p->kickRpm(); }
  static uint32_t regionRead(void *ctx,uint32_t offset) {
    auto *r=static_cast<GuampXnuRegion *>(ctx);return r?r->read32(offset):0xffffffffu;
  }
  static void regionWrite(void *ctx,uint32_t offset,uint32_t value) {
    auto *r=static_cast<GuampXnuRegion *>(ctx);if(r)r->write32(offset,value);
  }
  static void regionBarrier(void *) { __asm__ volatile("dsb sy" ::: "memory"); }
  static uint32_t smemRead(void *ctx,uint32_t offset) {
    auto *p=static_cast<GuampXnuSm6115Provider *>(ctx);
    return p&&p->active?p->smemMap.read32(offset):0;
  }
  static void smemWrite(void *ctx,uint32_t offset,uint32_t value) {
    auto *p=static_cast<GuampXnuSm6115Provider *>(ctx);
    if(p&&p->active)p->smemMap.write32(offset,value);
  }
  static void smemBarrier(void *) { __asm__ volatile("dmb sy" ::: "memory"); }
  static bool smemLock(void *ctx) {
    auto *p=static_cast<GuampXnuSm6115Provider *>(ctx);
    return p&&p->active&&guamp_smem_hwlock_acquire();
  }
  static void smemUnlock(void *) { guamp_smem_hwlock_release(); }
  // An existing SMEM item of another size (smem.c -EEXIST): reused if large enough, else Range (the R1 -102 case)
  static void smemNote(void *,const SmemArena::Note &n) {
    IOLog("GuampWiFi: SMEM item %u (host %u) exists with %u bytes, wanted %u: %s\n",
      n.item,unsigned(n.host),n.found,n.wanted,n.result?"refused":"reused");
  }
};
} // namespace guampwifi
#endif
