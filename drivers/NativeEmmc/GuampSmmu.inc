// SPDX-License-Identifier: BSD-2-Clause-Patent
// SM6115 apps SMMUv500 stage1, 4K tables / 32-bit WLAN IOVA. Register ABI from
// Motorola arm-smmu.c/arm-smmu-regs.h and msm-arm-smmu-bengal.dtsi.
// Preserves global setup and all other streams; never enables global bypass.
// Table memory must be coherent/Normal-NC, physically backed, and XNU-owned.
#ifndef GUAMP_SMMU_INC
#define GUAMP_SMMU_INC
#include "GuampWifiWire.inc"
namespace guampwifi {
struct WlanSmmu {
  static constexpr uint64_t IovaBase=0xa0000000ull,IovaEnd=0xb0000000ull;
  static constexpr uint32_t TableBytes=130*4096,Stream=0x1a0,StreamMask=1;
  Memory registers={},tables={};
  uint64_t tablePhysical=0,ramBase=0,ramBytes=0;
  uint32_t cb=0,slot=0,bank=0,gr1=0,oas=0,asid=0;
  uint32_t oldSmr=0,oldS2cr=0,oldCbar=0,oldCba2r=0;
  uint32_t oldCb[16]={}; // selected control/TTBR/MAIR words only
  bool active=false,owned=false;
  void *ctx=nullptr; void (*delayUs)(void *,uint32_t)=nullptr;
  static constexpr uint32_t SavedOffsets[16]={0,4,0x10,0x20,0x24,0x28,0x2c,0x30,
    0x34,0x38,0x3c,0x58,0x60,0x64,0x68,0x6c};
  bool inRam(uint64_t address,uint64_t n) const {
    return n&&address>=ramBase&&address-ramBase<ramBytes&&n<=ramBytes-(address-ramBase);
  }
  int start(const Memory &mmio,const Memory &pageTables,uint64_t physical,
            uint64_t managedBase,uint64_t managedBytes,bool radioStopped,
            void *opaque,void (*delay)(void *,uint32_t)) {
    if(active||owned) return Busy;
    if(!radioStopped) return Busy;
    if(!mmio.read||!mmio.write||!mmio.barrier||!pageTables.read||!pageTables.write||!pageTables.barrier||
       !delay||(physical&4095)||!pageTables.valid(0,TableBytes)||!mmio.valid(0,0x80000)) return Invalid;
    registers=mmio; tables=pageTables; tablePhysical=physical; ramBase=managedBase; ramBytes=managedBytes;
    ctx=opaque; delayUs=delay;
    if(!inRam(physical,TableBytes)) return Range;
    uint32_t id0=registers.r32(0x20),id1=registers.r32(0x24),id2=registers.r32(0x28);
    if(id0==0xffffffffu||!(id0&(1u<<30))||!(id0&(1u<<27))||(id1&(1u<<31))||!(id2&(1u<<12))) return Unsupported;
    uint32_t groups=id0&255u,banks=id1&255u;
    if(!groups||groups>128||!banks||banks>128) return Unsupported;
    // The OEM arm-smmu driver maps context banks at base + resource_size/2
    // (arm-smmu.c: smmu->cb_base), not immediately after ID1.NUMPAGENDXB.
    // Guamp's DT window is 0x80000, so this places CB0 at offset 0x40000.
    const uint32_t contextBase=registers.size/2;
    const uint32_t globalEnd=0xc00u+4u*groups;
    if((contextBase&4095u)||contextBase<globalEnd||banks>(registers.size-contextBase)/4096u) return Range;
    gr1=4096; uint32_t firstFree=0xffffffffu,match=0xffffffffu;
    bool used[128]={};
    for(uint32_t i=0;i<groups;i++) {
      uint32_t smr=registers.r32(0x800+4*i),s2cr=registers.r32(0xc00+4*i);
      if(!(smr&(1u<<31))) { if(firstFree==0xffffffffu) firstFree=i; continue; }
      uint32_t mask=(smr>>16)&0x7fffu,id=smr&0x7fffu;
      bool intersects=(((id^Stream)&~(mask|StreamMask)&0x7fff)==0);
      if(intersects) {
        if(mask!=StreamMask||id!=Stream||match!=0xffffffffu) return Busy;
        match=i;
      }
      if(((s2cr>>16)&3)==0) {
        uint32_t c=s2cr&255u; if(c>=banks) return Protocol; used[c]=true;
      }
    }
    slot=match!=0xffffffffu?match:firstFree; if(slot==0xffffffffu) return NoMemory;
    oldSmr=registers.r32(0x800+4*slot); oldS2cr=registers.r32(0xc00+4*slot);
    uint32_t chosen=0xffffffffu;
    for(uint32_t c=(id1>>16)&255u;c<banks;c++) if(!used[c]&&!(registers.r32(contextBase+4096*c)&1)) { chosen=c; break; }
    if(chosen==0xffffffffu) return NoMemory;
    static constexpr uint32_t OutputBits[6]={32,36,40,42,44,48};
    uint32_t encoding=(id2>>4)&15u; if(encoding>=6) return Unsupported;
    oas=OutputBits[encoding]; if((physical+TableBytes-1)>>oas) return Range;
    cb=chosen; bank=contextBase+4096*cb; asid=cb+1;
    oldCbar=registers.r32(gr1+4*cb); oldCba2r=registers.r32(gr1+0x800+4*cb);
    for(uint32_t i=0;i<16;i++) oldCb[i]=registers.r32(bank+SavedOffsets[i]);
    owned=true;
    tables.zero(0,TableBytes);
    putPte(0,2,(tablePhysical+4096)|3);
    for(uint32_t i=0;i<128;i++) putPte(4096,256+i,(tablePhysical+(2+i)*4096)|3);
    tables.fence();
    // Keep WLAN faulting until its new context is fully initialized and synced.
    registers.w32(0xc00+4*slot,2u<<16); registers.fence();
    registers.w32(bank,0);
    registers.w32(gr1+0x800+4*cb,1); // AArch64 context format
    registers.w32(gr1+4*cb,(1u<<16)|(3u<<8)|(15u<<12)); // S1, S2 bypass for THIS stream
    // Linux Guamp writes TTBCR2/TTBCR before TTBRs: these control registers
    // define how subsequent TTBR fields are interpreted. Keep that order even
    // while the context bank is disabled.
    registers.w32(bank+0x10,encoding|(7u<<15)); // PASIZE, sign-extension upstream
    registers.w32(bank+0x30,32u|(1u<<8)|(1u<<10)|(3u<<12)|(1u<<23)); // 32-bit VA, WB walks, EPD1
    uint64_t ttbr=tablePhysical|(uint64_t(asid)<<48);
    registers.w32(bank+0x24,uint32_t(ttbr>>32)); registers.w32(bank+0x20,uint32_t(ttbr));
    // 0x28/0x2c are the low/high halves of TTBR1. TTBR1 is disabled through
    // EPD1; the ASID is already encoded in TTBR0, never in TTBR1.
    registers.w32(bank+0x2c,0); registers.w32(bank+0x28,0);
    registers.w32(bank+0x38,0x44); registers.w32(bank+0x3c,0); // attribute0 = Normal-NC
    registers.w32(bank+0x58,0xffffffffu); registers.fence();
    int e=sync(); if(e) return e;
    registers.w32(bank,(1u<<12)|(1u<<8)|(1u<<5)|7u); // ASIDPNE,HUPCF,CFRE,TRE,AFE,M; IRQ off
    registers.fence();
    registers.w32(0x800+4*slot,(1u<<31)|(StreamMask<<16)|Stream);
    registers.w32(0xc00+4*slot,cb); registers.fence();
    if(registers.r32(0x800+4*slot)!=((1u<<31)|(StreamMask<<16)|Stream)||
       registers.r32(0xc00+4*slot)!=cb||!(registers.r32(bank)&1)) return Protocol;
    active=true; return Ok;
  }
  int map(uint64_t iova,uint64_t physical,uint32_t bytes) {
    if(!active) return NotReady;
    if((iova|physical|bytes)&4095u||!bytes||iova<IovaBase||iova>=IovaEnd||bytes>IovaEnd-iova||
       !inRam(physical,bytes)||((physical+bytes-1)>>oas)) return Range;
    if(physical<tablePhysical+TableBytes&&tablePhysical<physical+bytes) return Invalid;
    uint32_t first=uint32_t((iova-IovaBase)>>12),count=bytes>>12;
    for(uint32_t i=0;i<count;i++) if(getLeaf(first+i)&1) return Busy;
    for(uint32_t i=0;i<count;i++) {
      // AF, inner shareable, EL0/EL1 RW, NS, PXN+XN. No executable DMA mappings.
      uint64_t pte=(physical+4096*i)|3u|(1u<<5)|(1u<<6)|(3u<<8)|(1u<<10)|(3ull<<53);
      putLeaf(first+i,pte);
    }
    tables.fence(); return sync();
  }
  // Scatter-gather mapping for the ath10k streaming-DMA API. Publish every
  // page together so a mid-span failure cannot expose a partial buffer to CE.
  int mapPages(uint64_t iova,const uint64_t *physicalPages,uint32_t pages) {
    if(!active)return NotReady;
    if((iova&4095)||!physicalPages||!pages||pages>(IovaEnd-IovaBase)/4096||
       iova<IovaBase||iova>=IovaEnd||uint64_t(pages)*4096>IovaEnd-iova)return Range;
    for(uint32_t i=0;i<pages;i++) {
      uint64_t pa=physicalPages[i];
      if((pa&4095)||!inRam(pa,4096)||((pa+4095)>>oas)||
         (pa<tablePhysical+TableBytes&&tablePhysical<pa+4096))return Range;
      if(getLeaf(uint32_t((iova-IovaBase)>>12)+i)&1)return Busy;
    }
    const uint32_t first=uint32_t((iova-IovaBase)>>12);
    for(uint32_t i=0;i<pages;i++)putLeaf(first+i,makePte(physicalPages[i]));
    tables.fence();int e=sync();if(!e)return Ok;
    for(uint32_t i=0;i<pages;i++)putLeaf(first+i,0);
    tables.fence();return sync()?Quarantined:e;
  }
  int unmap(uint64_t iova,uint32_t bytes) {
    if(!active) return NotReady;
    if((iova|bytes)&4095u||!bytes||iova<IovaBase||iova>=IovaEnd||bytes>IovaEnd-iova) return Range;
    uint32_t first=uint32_t((iova-IovaBase)>>12),count=bytes>>12;
    bool changed=false;
    for(uint32_t i=0;i<count;i++)if(getLeaf(first+i)&1){putLeaf(first+i,0);changed=true;}
    if(changed)tables.fence();
    // Also sync an already-cleared range: the previous invalidation may have
    // timed out, and the caller keeps its IOVA quarantined until this succeeds.
    return sync();
  }
  uint32_t faultStatus() const { return owned?registers.r32(bank+0x58):0; }
  int stop(bool radioStopped) {
    if(!radioStopped) return Busy;
    if(!owned) return Ok;
    registers.w32(0xc00+4*slot,2u<<16); registers.fence();
    registers.w32(bank,0); int e=sync(); if(e) return e;
    // Restore only configuration words. TTBCR2/TTBCR must precede TTBRs,
    // as they determine how the TTBR fields are interpreted. Fault
    // syndrome/address registers are hardware status; never replay them.
    registers.w32(gr1+0x800+4*cb,oldCba2r); registers.w32(gr1+4*cb,oldCbar);
    registers.w32(bank+SavedOffsets[1],oldCb[1]); // ACTLR
    registers.w32(bank+0x10,oldCb[2]);           // TTBCR2
    registers.w32(bank+0x30,oldCb[7]);           // TTBCR
    registers.w32(bank+0x24,oldCb[4]); registers.w32(bank+0x20,oldCb[3]); // TTBR0 hi, lo
    registers.w32(bank+0x2c,oldCb[6]); registers.w32(bank+0x28,oldCb[5]); // TTBR1 hi, lo
    registers.w32(bank+0x34,oldCb[8]);           // CONTEXTIDR
    registers.w32(bank+0x38,oldCb[9]); registers.w32(bank+0x3c,oldCb[10]); // MAIR0/1
    registers.w32(bank,oldCb[0]); registers.fence();
    registers.w32(0x800+4*slot,oldSmr); registers.w32(0xc00+4*slot,oldS2cr); registers.fence();
    active=owned=false; return Ok;
  }
private:
  uint64_t makePte(uint64_t physical) const {
    // AF, inner shareable, EL0/EL1 RW, NS, PXN+XN. No executable DMA mappings.
    return (physical&~uint64_t(4095))|3u|(1u<<5)|(1u<<6)|(3u<<8)|(1u<<10)|(3ull<<53);
  }
  void putPte(uint32_t table,uint32_t index,uint64_t value) {
    uint32_t at=table+index*8;
    // Invalid low word first on clear; valid low word last on publication.
    tables.w32(at,0); tables.w32(at+4,uint32_t(value>>32)); tables.fence(); tables.w32(at,uint32_t(value));
  }
  uint64_t getLeaf(uint32_t page) const {
    uint32_t at=(2+page/512)*4096+(page%512)*8;
    return tables.r32(at)|(uint64_t(tables.r32(at+4))<<32);
  }
  void putLeaf(uint32_t page,uint64_t value) { putPte((2+page/512)*4096,page%512,value); }
  int sync() {
    registers.fence(); registers.w32(bank+0x610,asid); registers.w32(bank+0x7f0,0); registers.fence();
    for(uint32_t n=0;n<10000;n++) {
      if(!(registers.r32(bank+0x7f4)&1)) { registers.fence(); return Ok; }
      delayUs(ctx,1);
    }
    return Timeout;
  }
};
} // namespace guampwifi
#endif
