/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * XNU-owned IOVA allocator for the WCN3990 SNOC DMA API.
 * Mapping is disabled until Guamp WlanSmmu has established SID 0x1a0.
 *
 * Memory type: the apps SMMU and WCN3990 are not IO-coherent (no
 * dma-coherent in the guamp DT), so like Linux (pgprot_dmacoherent) every
 * CPU view of DMA memory is Normal Non-Cacheable. On arm64 XNU that is
 * kIOMapWriteCombineCache (VM_WIMG_WCOMB, MAIR 0x44); kIOMapInhibitCache
 * would give Device-nGnRnE, where bzero's DC ZVA and the unaligned
 * accesses of bcopy/memset (ath10k ring memset, skb copies) fault.
 * Before the NC view is used, the cacheable physical-aperture alias is
 * cleaned+invalidated so no dirty line can later overwrite device data.
 *
 * Capacity: ath10k WCN3990 keeps ~2750 CE RX buffers (CE1/2/5/8/9/10/11)
 * plus 2047 HTT RX ring buffers mapped at once, each with its own bounce
 * page(s); MaxMappings covers that with headroom.
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
#include "guamp_xnu_iova.h"

namespace guampwifi {
struct GuampXnuDma {
  static constexpr uint32_t Page=4096,MaxMappings=8192,Magic=0x4757414c;
  static constexpr uint32_t IovaPages=uint32_t((WlanSmmu::IovaEnd-WlanSmmu::IovaBase)/Page);
  static constexpr IOOptionBits CpuView=static_cast<IOOptionBits>(kIOMapWriteCombineCache);
  struct AllocationHeader { uint64_t magic,bytes; };
  struct Slot {
    IOBufferMemoryDescriptor *descriptor;
    IOMemoryMap *uncached;
    void *address,*dmaAddress;
    uint64_t physical,iova,dmaBus;
    uint32_t mappedBytes,requestedBytes,direction;
    bool used,coherent,quarantined;
  };
  enum Failure : uint32_t { NoSlot,NoIova,NoMemory,NoView,SmmuMap,FailureKinds };
  WlanSmmu *smmu=nullptr;
  IOLock *lock=nullptr;
  Slot *slots=nullptr;
  uint16_t *pageOwner=nullptr;
  GuampIovaPages iova{};
  uint32_t freeHint=0,live=0,peak=0,coherentLive=0;
  uint32_t failures[FailureKinds]={};
  uint64_t ramBase=0,ramBytes=0;

  bool init(WlanSmmu *owner,uint64_t managedBase,uint64_t managedBytes) {
    if(lock||!owner||!owner->active||!managedBytes)return false;
    slots=static_cast<Slot *>(IOMallocZero(sizeof(Slot)*MaxMappings));
    pageOwner=static_cast<uint16_t *>(IOMallocZero(sizeof(uint16_t)*IovaPages));
    lock=IOLockAlloc();
    if(!slots||!pageOwner||!lock||!iova.init(pageOwner,IovaPages)) { freeTables();return false; }
    smmu=owner;ramBase=managedBase;ramBytes=managedBytes;
    IOLog("GuampWiFi: dma: IOVA 0x%llx-0x%llx (%u pages), %u mappings, CPU view Normal-NC\n",
      WlanSmmu::IovaBase,WlanSmmu::IovaEnd,IovaPages,MaxMappings);
    return true;
  }
  static bool installHooks(lx_platform *platform) {
    if(!platform||!active().lock)return false;
    platform->dma_alloc=dmaAllocate;platform->dma_free=dmaFree;
    platform->dma_map=dmaMap;platform->dma_sync=dmaSync;return true;
  }
  void report(const char *when) {
    if(!lock)return;
    IOLog("GuampWiFi: dma %s: %u mappings (%u coherent), peak %u, IOVA pages %u peak %u, "
      "failures slot %u iova %u mem %u view %u smmu %u\n",when,live,coherentLive,peak,iova.used,iova.peak,
      failures[NoSlot],failures[NoIova],failures[NoMemory],failures[NoView],failures[SmmuMap]);
  }

  // Must be called only after the radio has stopped and its SMMU stream is idle.
  bool stop(bool radioStopped) {
    if(!radioStopped||!lock)return false;
    IOLockLock(lock);
    for(uint32_t i=0;i<MaxMappings;i++) if(slots[i].used) {
      if(!releaseSlot(i)) { IOLockUnlock(lock);return false; }
    }
    IOLockUnlock(lock);freeTables();smmu=nullptr;return true;
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
  void freeTables() {
    if(lock){IOLockFree(lock);lock=nullptr;}
    if(slots){IOFree(slots,sizeof(Slot)*MaxMappings);slots=nullptr;}
    if(pageOwner){IOFree(pageOwner,sizeof(uint16_t)*IovaPages);pageOwner=nullptr;}
    iova={};
  }
  static uint32_t rounded(size_t bytes) {
    if(!bytes||bytes>UINT32_MAX-(Page-1))return 0;
    return (static_cast<uint32_t>(bytes)+Page-1)&~(Page-1);
  }
  void failed(Failure kind,uint32_t bytes) {
    // First occurrence of each kind, then every 1024th: enough to see an
    // exhausted pool without flooding the log from the RX refill path.
    uint32_t n=++failures[kind];
    if(n==1||!(n&1023))IOLog("GuampWiFi: dma failure %u (#%u) for %u bytes, %u live mappings\n",unsigned(kind),n,bytes,live);
  }
  int findFree() {
    for(uint32_t n=0;n<MaxMappings;n++) {
      uint32_t i=(freeHint+n)%MaxMappings;
      if(!slots[i].used){freeHint=(i+1)%MaxMappings;return static_cast<int>(i);}
    }
    return -1;
  }
  int findBus(uint64_t bus) {
    if(bus<WlanSmmu::IovaBase||bus>=WlanSmmu::IovaEnd)return -1;
    uint16_t tag=iova.at(uint32_t((bus-WlanSmmu::IovaBase)/Page));
    if(!tag)return -1;
    const Slot &s=slots[tag-1];
    return s.used&&bus>=s.dmaBus&&bus-s.dmaBus<s.requestedBytes?int(tag-1):-1;
  }
  int findAddress(void *p) { for(uint32_t i=0;i<MaxMappings;i++)if(slots[i].used&&slots[i].coherent&&slots[i].address==p)return static_cast<int>(i);return -1; }
  bool reserveIova(int index,uint32_t bytes,uint64_t *result) {
    uint32_t first=0;
    if(!iova.reserve(bytes/Page,uint16_t(index+1),&first)){failed(NoIova,bytes);return false;}
    *result=WlanSmmu::IovaBase+uint64_t(first)*Page;return true;
  }
  void unreserve(int index,uint64_t at,uint32_t bytes) {
    iova.release(uint32_t((at-WlanSmmu::IovaBase)/Page),bytes/Page,uint16_t(index+1));
  }
  // Contiguous XNU RAM with a Normal-NC kernel view. The physical-aperture
  // (cacheable) alias is flushed first: dirty lines left by the allocator's
  // zero fill must never be evicted over data the device writes later.
  bool backing(uint32_t n,IOBufferMemoryDescriptor **descriptor,IOMemoryMap **view,uint64_t *physical,uint8_t **cpu) {
    IOOptionBits options=static_cast<IOOptionBits>(kIODirectionInOut)|
      static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous)|static_cast<IOOptionBits>(kIOMemoryMapperNone)|CpuView;
    IOBufferMemoryDescriptor *d=IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,options,n,~uint64_t(0));
    if(!d||d->prepare()!=kIOReturnSuccess) {if(d)d->release();failed(NoMemory,n);return false;}
    IOByteCount contiguous=0;uint64_t pa=d->getPhysicalSegment(0,&contiguous,kIOMemoryMapperNone);
    if(!pa||contiguous<n||pa<ramBase||pa-ramBase>=ramBytes||n>ramBytes-(pa-ramBase)) {
      d->complete();d->release();failed(NoMemory,n);return false;
    }
    (void)d->performOperation(kIOMemoryIncoherentIOFlush,0,n);
    IOMemoryMap *m=d->map(kIOMapAnywhere|CpuView);
    uint8_t *va=m?reinterpret_cast<uint8_t *>(m->getVirtualAddress()):nullptr;
    if(!va) {if(m)m->release();d->complete();d->release();failed(NoView,n);return false;}
    *descriptor=d;*view=m;*physical=pa;*cpu=va;return true;
  }

  int allocateCoherentSlot(size_t bytes,uint64_t *bus,void **cpu) {
    if(!lock||!smmu||!bus||!cpu)return -1;uint32_t n=rounded(bytes);if(!n)return -1;
    IOLockLock(lock);int index=findFree();uint64_t at=0;
    if(index<0){failed(NoSlot,n);IOLockUnlock(lock);return -1;}
    if(!reserveIova(index,n,&at)){IOLockUnlock(lock);return -1;}
    IOBufferMemoryDescriptor *d=nullptr;IOMemoryMap *view=nullptr;uint64_t pa=0;uint8_t *va=nullptr;
    if(!backing(n,&d,&view,&pa,&va)){unreserve(index,at,n);IOLockUnlock(lock);return -1;}
    bzero(va,n);__asm__ volatile("dsb sy":::"memory");
    int mapError=smmu->map(at,pa,n);
    if(mapError) {
      failed(SmmuMap,n);
      // A failed TLB sync may leave table entries visible. Keep the pages and
      // IOVA quarantined until an explicit stop can prove unmap completed.
      if(mapError==Timeout) {
        slots[index]={d,view,va,va,pa,at,at,n,static_cast<uint32_t>(bytes),NW_DMA_BIDIRECTIONAL,true,true,true};
        live++;coherentLive++;IOLockUnlock(lock);return -1;
      }
      view->release();d->complete();d->release();unreserve(index,at,n);IOLockUnlock(lock);return -1;
    }
    slots[index]={d,view,va,va,pa,at,at,n,static_cast<uint32_t>(bytes),NW_DMA_BIDIRECTIONAL,true,true,false};
    live++;coherentLive++;if(live>peak)peak=live;
    *bus=at;*cpu=va;IOLockUnlock(lock);return 0;
  }

  void *allocateCoherent(size_t bytes,uint64_t *bus) {
    void *cpu=nullptr;if(allocateCoherentSlot(bytes,bus,&cpu))return nullptr;
    return cpu;
  }
  int freeCoherent(size_t bytes,void *p,uint64_t bus) {
    if(!lock||!p)return -1;IOLockLock(lock);int i=findAddress(p);
    if(i<0||(bus&&bus!=slots[i].dmaBus)||
       (bytes&&bytes>slots[i].requestedBytes)){IOLockUnlock(lock);return -1;}
    bool ok=releaseSlot(uint32_t(i));IOLockUnlock(lock);return ok?0:-1;
  }
  uint64_t mapStreaming(void *p,size_t bytes,int direction) {
    if(!lock||!smmu||!p||!bytes||bytes>UINT32_MAX||direction<NW_DMA_BIDIRECTIONAL||direction>NW_DMA_FROM_DEVICE)return 0;
    uintptr_t va=reinterpret_cast<uintptr_t>(p);uint32_t off=static_cast<uint32_t>(va&(Page-1));
    if(bytes>UINT32_MAX-off)return 0;uint32_t span=rounded(bytes+off);if(!span)return 0;
    if(span/Page>256)return 0; // bound contiguous allocation and lock-held copies to 1 MiB
    IOLockLock(lock);int i=findFree();uint64_t at=0;
    if(i<0){failed(NoSlot,span);IOLockUnlock(lock);return 0;}
    if(!reserveIova(i,span,&at)) {IOLockUnlock(lock);return 0;}
    IOBufferMemoryDescriptor *d=nullptr;IOMemoryMap *view=nullptr;uint64_t pa=0;uint8_t *dmaBase=nullptr;
    if(!backing(span,&d,&view,&pa,&dmaBase)){unreserve(i,at,span);IOLockUnlock(lock);return 0;}
    if(direction==NW_DMA_TO_DEVICE||direction==NW_DMA_BIDIRECTIONAL)bcopy(p,dmaBase+off,bytes);
    __asm__ volatile("dsb sy":::"memory");
    int mapped=smmu->map(at,pa,span);
    if(mapped) {
      failed(SmmuMap,span);
      if(mapped==Timeout||mapped==Quarantined) {
        slots[i]={d,view,p,dmaBase+off,pa,at,at,span,static_cast<uint32_t>(bytes),static_cast<uint32_t>(direction),true,false,true};
        live++;
      } else {view->release();d->complete();d->release();unreserve(i,at,span);}
      IOLockUnlock(lock);return 0;
    }
    slots[i]={d,view,p,dmaBase+off,pa,at,at+off,span,static_cast<uint32_t>(bytes),static_cast<uint32_t>(direction),true,false,false};
    live++;if(live>peak)peak=live;
    uint64_t bus=at+off;IOLockUnlock(lock);return bus;
  }
  void syncStreaming(uint64_t bus,size_t bytes,int direction,int action) {
    if(!lock||!bus||!bytes||bytes>UINT32_MAX)return;IOLockLock(lock);int i=findBus(bus);
    if(i<0||slots[i].coherent||direction!=static_cast<int>(slots[i].direction)||
       bytes>slots[i].requestedBytes-(bus-slots[i].dmaBus)){IOLockUnlock(lock);return;}
    Slot &s=slots[i];
    if(action==NW_DMA_UNMAP) {
      __asm__ volatile("dsb sy":::"memory");
      if(direction==NW_DMA_FROM_DEVICE||direction==NW_DMA_BIDIRECTIONAL)
        bcopy(static_cast<uint8_t *>(s.dmaAddress)+(bus-s.dmaBus),static_cast<uint8_t *>(s.address)+(bus-s.dmaBus),bytes);
      (void)releaseSlot(uint32_t(i));
    } else if(action==NW_DMA_SYNC_FOR_CPU) {
      __asm__ volatile("dsb sy":::"memory");
      if(direction==NW_DMA_FROM_DEVICE||direction==NW_DMA_BIDIRECTIONAL)
        bcopy(static_cast<uint8_t *>(s.dmaAddress)+(bus-s.dmaBus),static_cast<uint8_t *>(s.address)+(bus-s.dmaBus),bytes);
    } else if(action==NW_DMA_SYNC_FOR_DEVICE) {
      if(direction==NW_DMA_TO_DEVICE||direction==NW_DMA_BIDIRECTIONAL)
        bcopy(static_cast<uint8_t *>(s.address)+(bus-s.dmaBus),static_cast<uint8_t *>(s.dmaAddress)+(bus-s.dmaBus),bytes);
      __asm__ volatile("dsb sy":::"memory");
    }
    IOLockUnlock(lock);
  }
  bool releaseSlot(uint32_t index) {
    Slot &s=slots[index];
    if(!s.used)return true;
    if(smmu->unmap(s.iova,s.mappedBytes)) {s.quarantined=true;return false;}
    if(s.uncached){s.uncached->release();s.uncached=nullptr;}
    if(s.descriptor){s.descriptor->complete();s.descriptor->release();s.descriptor=nullptr;}
    unreserve(int(index),s.iova,s.mappedBytes);
    live--;if(s.coherent)coherentLive--;
    s={};return true;
  }
};
} // namespace guampwifi
#endif
