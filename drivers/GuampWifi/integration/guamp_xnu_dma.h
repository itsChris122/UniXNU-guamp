/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * XNU-owned IOVA allocator for the WCN3990 SNOC DMA API.
 * Mapping is disabled until Guamp WlanSmmu has established SID 0x1a0.
 */
#ifndef GUAMP_XNU_DMA_H
#define GUAMP_XNU_DMA_H
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOLib.h>
#include <kern/locks.h>
#include <mach/mach_types.h>
#include <libkern/libkern.h>
#include "../ath10k-port/nw_abi.h"
#include "GuampSmmu.inc"

namespace guampwifi {
struct GuampXnuDma {
  static constexpr uint32_t Page=4096,MaxMappings=256,Magic=0x4757414c;
  struct AllocationHeader { uint64_t magic,bytes; };
  struct Slot {
    IOBufferMemoryDescriptor *descriptor;
    IOMemoryMap *uncached;
    void *address,*dmaAddress;
    uint64_t physical,iova,dmaBus;
    uint32_t mappedBytes,requestedBytes,direction;
    bool used,coherent,quarantined;
  };
  WlanSmmu *smmu=nullptr;
  IOLock *lock=nullptr;
  Slot slots[MaxMappings]={};
  uint64_t nextIova=WlanSmmu::IovaBase;
  uint64_t ramBase=0,ramBytes=0;

  bool init(WlanSmmu *owner,uint64_t managedBase,uint64_t managedBytes) {
    if(lock||!owner||!owner->active||!managedBytes)return false;
    lock=IOLockAlloc();if(!lock)return false;
    smmu=owner;ramBase=managedBase;ramBytes=managedBytes;nextIova=WlanSmmu::IovaBase;
    return true;
  }
  static bool installHooks(lx_platform *platform) {
    if(!platform||!active().lock)return false;
    platform->dma_alloc=dmaAllocate;platform->dma_free=dmaFree;
    platform->dma_map=dmaMap;platform->dma_sync=dmaSync;return true;
  }

  // Must be called only after the radio has stopped and its SMMU stream is idle.
  bool stop(bool radioStopped) {
    if(!radioStopped||!lock)return false;
    IOLockLock(lock);
    for(uint32_t i=0;i<MaxMappings;i++) if(slots[i].used) {
      if(!releaseSlot(slots[i])) { IOLockUnlock(lock);return false; }
    }
    IOLockUnlock(lock);IOLockFree(lock);lock=nullptr;smmu=nullptr;return true;
  }

  static void *allocate(size_t bytes) {
    if(!bytes||bytes>SIZE_MAX-sizeof(AllocationHeader))return nullptr;
    auto *h=static_cast<AllocationHeader *>(IOMallocZero(sizeof(AllocationHeader)+bytes));
    if(!h)return nullptr;h->magic=Magic;h->bytes=bytes;return h+1;
  }
  static void deallocate(void *p) {
    if(!p)return;auto *h=static_cast<AllocationHeader *>(p)-1;
    if(h->magic!=Magic)return;size_t bytes=sizeof(*h)+h->bytes;h->magic=0;IOFree(h,bytes);
  }

  static void *dmaAllocate(size_t bytes,uint64_t *bus) {
    return active().allocateCoherent(bytes,bus);
  }
  static void dmaFree(size_t bytes,void *p,uint64_t bus) {
    (void)active().freeCoherent(bytes,p,bus);
  }
  static uint64_t dmaMap(void *p,size_t bytes,int direction) {
    return active().mapStreaming(p,bytes,direction);
  }
  static void dmaSync(uint64_t bus,size_t bytes,int direction,int action) {
    active().syncStreaming(bus,bytes,direction,action);
  }

private:
  static GuampXnuDma &active() { extern GuampXnuDma gGuampXnuDma;return gGuampXnuDma; }
  static uint32_t rounded(size_t bytes) {
    if(!bytes||bytes>UINT32_MAX-(Page-1))return 0;
    return (static_cast<uint32_t>(bytes)+Page-1)&~(Page-1);
  }
  static uint64_t pageFloor(uint64_t v) { return v&~uint64_t(Page-1); }
  int findFree() { for(uint32_t i=0;i<MaxMappings;i++)if(!slots[i].used)return static_cast<int>(i);return -1; }
  int findBus(uint64_t bus) {
    for(uint32_t i=0;i<MaxMappings;i++)if(slots[i].used&&bus>=slots[i].dmaBus&&bus-slots[i].dmaBus<slots[i].requestedBytes)return static_cast<int>(i);
    return -1;
  }
  int findAddress(void *p) { for(uint32_t i=0;i<MaxMappings;i++)if(slots[i].used&&slots[i].address==p)return static_cast<int>(i);return -1; }

  bool reserveIova(uint32_t bytes,uint64_t *result) {
    uint64_t candidate=WlanSmmu::IovaBase;
    for(uint32_t pass=0;pass<=MaxMappings;pass++) {
      if(candidate<WlanSmmu::IovaBase||candidate>=WlanSmmu::IovaEnd||bytes>WlanSmmu::IovaEnd-candidate)return false;
      bool moved=false;
      for(uint32_t i=0;i<MaxMappings;i++)if(slots[i].used) {
        const Slot &s=slots[i];uint64_t end=candidate+bytes,otherEnd=s.iova+s.mappedBytes;
        if(candidate<otherEnd&&s.iova<end) {candidate=pageFloor(otherEnd+Page-1);moved=true;break;}
      }
      if(!moved) { *result=candidate;nextIova=candidate+bytes;return true; }
    }
    return false;
  }

  int allocateCoherentSlot(size_t bytes,uint64_t *bus,void **cpu) {
    if(!lock||!smmu||!bus||!cpu)return -1;uint32_t n=rounded(bytes);if(!n)return -1;
    IOLockLock(lock);int index=findFree();uint64_t iova=0;
    if(index<0||!reserveIova(n,&iova)){IOLockUnlock(lock);return -1;}
    IOOptionBits options=static_cast<IOOptionBits>(kIODirectionInOut)|
      static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous)|static_cast<IOOptionBits>(kIOMemoryMapperNone)|
      static_cast<IOOptionBits>(kIOMapInhibitCache);
    IOBufferMemoryDescriptor *d=IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,options,n,~uint64_t(0));
    if(!d||d->prepare()!=kIOReturnSuccess) {if(d)d->release();IOLockUnlock(lock);return -1;}
    IOByteCount contiguous=0;uint64_t pa=d->getPhysicalSegment(0,&contiguous,kIOMemoryMapperNone);
    IOMemoryMap *view=pa&&contiguous>=n?d->map(kIOMapAnywhere|kIOMapInhibitCache):nullptr;
    void *va=view?reinterpret_cast<void *>(view->getVirtualAddress()):nullptr;
    int mapError=(!va||!view||!pa||pa<ramBase||pa-ramBase>=ramBytes||n>ramBytes-(pa-ramBase))?-1:smmu->map(iova,pa,n);
    if(mapError) {
      // A failed TLB sync may leave table entries visible. Keep the pages and
      // IOVA quarantined until an explicit stop can prove unmap completed.
      if(mapError==Timeout && d&&view&&va&&pa) {
        slots[index]={d,view,va,va,pa,iova,iova,n,static_cast<uint32_t>(bytes),NW_DMA_BIDIRECTIONAL,true,true,true};
        IOLockUnlock(lock);return -1;
      }
      if(view)view->release();d->complete();d->release();IOLockUnlock(lock);return -1;
    }
    bzero(va,n);__asm__ volatile("dsb sy":::"memory");
    slots[index]={d,view,va,va,pa,iova,iova,n,static_cast<uint32_t>(bytes),NW_DMA_BIDIRECTIONAL,true,true,false};
    *bus=iova;*cpu=va;IOLockUnlock(lock);return 0;
  }

  void *allocateCoherent(size_t bytes,uint64_t *bus) {
    void *cpu=nullptr;if(allocateCoherentSlot(bytes,bus,&cpu))return nullptr;
    return cpu;
  }
  int freeCoherent(size_t bytes,void *p,uint64_t bus) {
    if(!lock||!p)return -1;IOLockLock(lock);int i=findAddress(p);
    if(i<0||!slots[i].coherent||(bus&&bus!=slots[i].dmaBus)||
       (bytes&&bytes>slots[i].requestedBytes)){IOLockUnlock(lock);return -1;}
    bool ok=releaseSlot(slots[i]);IOLockUnlock(lock);return ok?0:-1;
  }
  uint64_t mapStreaming(void *p,size_t bytes,int direction) {
    if(!lock||!smmu||!p||!bytes||bytes>UINT32_MAX||direction<NW_DMA_BIDIRECTIONAL||direction>NW_DMA_FROM_DEVICE)return 0;
    uintptr_t va=reinterpret_cast<uintptr_t>(p);uint32_t off=static_cast<uint32_t>(va&(Page-1));
    if(bytes>UINT32_MAX-off)return 0;uint32_t span=rounded(bytes+off);if(!span)return 0;
    if(span/Page>256)return 0; // bound contiguous allocation and lock-held copies to 1 MiB
    IOLockLock(lock);int i=findFree();uint64_t iova=0;
    if(i<0||!reserveIova(span,&iova)) {IOLockUnlock(lock);return 0;}
    IOOptionBits options=static_cast<IOOptionBits>(kIODirectionInOut)|
      static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous)|static_cast<IOOptionBits>(kIOMemoryMapperNone)|
      static_cast<IOOptionBits>(kIOMapInhibitCache);
    IOBufferMemoryDescriptor *d=IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,options,span,~uint64_t(0));
    if(!d||d->prepare()!=kIOReturnSuccess) {if(d)d->release();IOLockUnlock(lock);return 0;}
    IOByteCount contiguous=0;uint64_t pa=d->getPhysicalSegment(0,&contiguous,kIOMemoryMapperNone);
    IOMemoryMap *view=pa&&contiguous>=span?d->map(kIOMapAnywhere|kIOMapInhibitCache):nullptr;
    void *base=view?reinterpret_cast<void *>(view->getVirtualAddress()):nullptr;
    if(!base||!pa||pa<ramBase||pa-ramBase>=ramBytes||span>ramBytes-(pa-ramBase)) {
      if(view)view->release();d->complete();d->release();IOLockUnlock(lock);return 0;
    }
    uint8_t *dmaBase=static_cast<uint8_t *>(base);
    if(direction==NW_DMA_TO_DEVICE||direction==NW_DMA_BIDIRECTIONAL)bcopy(p,dmaBase+off,bytes);
    __asm__ volatile("dsb sy":::"memory");
    int mapped=smmu->map(iova,pa,span);
    if(mapped) {
      if(mapped==Timeout||mapped==Quarantined)
        slots[i]={d,view,p,dmaBase+off,pa,iova,iova,span,static_cast<uint32_t>(bytes),static_cast<uint32_t>(direction),true,false,true};
      else {view->release();d->complete();d->release();}
      IOLockUnlock(lock);return 0;
    }
    slots[i]={d,view,p,dmaBase+off,pa,iova,iova+off,span,static_cast<uint32_t>(bytes),static_cast<uint32_t>(direction),true,false,false};
    uint64_t bus=iova+off;IOLockUnlock(lock);return bus;
  }
  void syncStreaming(uint64_t bus,size_t bytes,int direction,int action) {
    if(!lock||!bus||!bytes||bytes>UINT32_MAX)return;IOLockLock(lock);int i=findBus(bus);
    if(i<0||slots[i].coherent||direction!=static_cast<int>(slots[i].direction)||
       bytes>slots[i].requestedBytes-(bus-slots[i].dmaBus)){IOLockUnlock(lock);return;}
    Slot &s=slots[i];
    if(action==NW_DMA_UNMAP) {
      if(direction==NW_DMA_FROM_DEVICE||direction==NW_DMA_BIDIRECTIONAL)
        bcopy(static_cast<uint8_t *>(s.dmaAddress)+(bus-s.dmaBus),static_cast<uint8_t *>(s.address)+(bus-s.dmaBus),bytes);
      if(releaseSlot(s)){}
    } else if(action==NW_DMA_SYNC_FOR_CPU) {
      if(direction==NW_DMA_FROM_DEVICE||direction==NW_DMA_BIDIRECTIONAL)
        bcopy(static_cast<uint8_t *>(s.dmaAddress)+(bus-s.dmaBus),static_cast<uint8_t *>(s.address)+(bus-s.dmaBus),bytes);
    } else if(action==NW_DMA_SYNC_FOR_DEVICE) {
      if(direction==NW_DMA_TO_DEVICE||direction==NW_DMA_BIDIRECTIONAL)
        bcopy(static_cast<uint8_t *>(s.address)+(bus-s.dmaBus),static_cast<uint8_t *>(s.dmaAddress)+(bus-s.dmaBus),bytes);
    }
    IOLockUnlock(lock);
  }
  bool releaseSlot(Slot &s) {
    if(!s.used)return true;
    if(smmu->unmap(s.iova,s.mappedBytes)) {s.quarantined=true;return false;}
    if(s.uncached){s.uncached->release();s.uncached=nullptr;}
    if(s.descriptor){s.descriptor->complete();s.descriptor->release();s.descriptor=nullptr;}
    s={};return true;
  }
};
} // namespace guampwifi
#endif
