// SPDX-License-Identifier: BSD-2-Clause-Patent
// APPS-side SMEM allocator. Layout/protocol: Motorola drivers/soc/qcom/smem.c.
// A local mutex AND the remote hwspinlock (TCSR lock 3, owner 1) are mandatory.
#ifndef GUAMP_SMEM_WRITE_INC
#define GUAMP_SMEM_WRITE_INC
#include "GuampWifiWire.inc"
namespace guampwifi {
struct SmemArena {
  Memory mem;
  void *lockCtx;
  bool (*lock)(void *); // bounded wait; serializes local callers + hardware lock
  void (*unlock)(void *);
  static uint32_t reader(void *ctx,uint32_t at) { return static_cast<SmemArena *>(ctx)->mem.r32(at); }
  smem::Reader view() { return {reader,this,mem.size}; }
  int info(smem::Info *out) {
    auto r=view(); mem.fence(); smem::parse(r,out);
    return out->error || out->initialized!=1?Protocol:Ok;
  }
  int find(uint16_t peer,uint32_t id,uint32_t *at,uint32_t *size) {
    smem::Info i={}; int e=info(&i); if(e) return e;
    return smem::find(view(),i,peer,id,at,size)?Ok:NotFound;
  }
  int allocate(uint16_t peer,uint32_t id,uint32_t bytes,uint32_t *at,bool *created=nullptr) {
    if(created) *created=false;
    if(!bytes||bytes>0x100000||id<8||id>=512||!lock||!unlock) return Invalid;
    if(!lock(lockCtx)) return Timeout;
    int e=allocateLocked(peer,id,bytes,at,created);
    unlock(lockCtx); return e;
  }
private:
  int allocateLocked(uint16_t peer,uint32_t id,uint32_t bytes,uint32_t *at,bool *created) {
    smem::Info i={}; int e=info(&i); if(e) return e;
    if(id>=i.numItems) return Range;
    const auto *p=smem::partitionFor(i,peer);
    if(!p) p=smem::partitionFor(i,smem::kHostGlobal);
    uint32_t oldAt=0,oldSize=0;
    // Do not reuse a global duplicate when the selected private heap is present.
    bool found=false;
    if(p) {
      if(p->status) return Protocol;
      e=smem::walk(view(),*p,[&](uint16_t item,uint32_t off,uint32_t size) {
        if(item!=id) return false;
        oldAt=off; oldSize=size; found=true; return true;
      });
      if(e) return Protocol;
    } else if(i.version==11) found=smem::find(view(),i,peer,id,&oldAt,&oldSize);
    else return NotReady;
    if(found) {
      if(oldSize!=bytes && !(i.version==11&&!p&&oldSize==align8(bytes))) return Range;
      *at=oldAt; return Ok;
    }
    const uint32_t padded=align8(bytes);
    if(p) {
      const uint32_t base=p->offset, free=mem.r32(base+12), end=mem.r32(base+16);
      if(free<32||(free&7)||free>end||end>p->size||padded+16>end-free) return NoMemory;
      const uint32_t hdr=base+free;
      if(!mem.zero(hdr,padded+16)) return Range;
      mem.w32(hdr,0xa5a5u|(id<<16));
      mem.w32(hdr+4,padded);
      mem.w32(hdr+8,padded-bytes); // padding_data low16, padding_hdr high16
      mem.fence(); mem.w32(base+12,free+16+padded); mem.fence();
      *at=hdr+16;
    } else {
      uint32_t free=mem.r32(196),available=mem.r32(200),toc=208+16*id;
      if((free&7)||free<208+512*16||padded>available||!mem.valid(free,padded)) return NoMemory;
      if(!mem.zero(free,padded)) return Range;
      mem.w32(toc+4,free); mem.w32(toc+8,padded); mem.w32(toc+12,0);
      mem.fence(); mem.w32(toc,1);
      mem.w32(196,free+padded); mem.w32(200,available-padded); mem.fence();
      *at=free;
    }
    if(created) *created=true;
    return Ok;
  }
};
} // namespace guampwifi
#endif
