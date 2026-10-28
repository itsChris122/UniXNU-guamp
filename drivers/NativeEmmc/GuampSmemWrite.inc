// SPDX-License-Identifier: BSD-2-Clause-Patent
// APPS-side SMEM allocator, the ONE SMEM write layer of the kernel: Wi-Fi R1 (GuampSmp2p.inc, GuampWifiControl.inc,
// drivers/GuampWifi provider) and the ADSP transports of audio phase 2 (GuampSmp2pEdge.inc, GuampAdspIpc.inc) use it.
// Layout/protocol: Motorola drivers/soc/qcom/smem.c (qcom_smem_alloc + qcom_smem_get).
// A local mutex AND the remote hwspinlock (TCSR lock 3, owner 1) are mandatory: in the kernel both callers pass the
// shared pair guamp_smem_hwlock_acquire/release (GuampSmemLock.inc), so they also exclude each other.
// An existing item (Linux: -EEXIST, then qcom_smem_get) is reused when it holds at least the requested bytes; its real
// size is returned (*actual), so a FIFO can use the length the remote sees. A smaller one is Range. Either case is
// reported through note() after the lock is released (item, wanted and found sizes).
#ifndef GUAMP_SMEM_WRITE_INC
#define GUAMP_SMEM_WRITE_INC
#include "GuampWifiWire.inc"
namespace guampwifi {
struct SmemArena {
  struct Note { uint16_t host; uint32_t item,wanted,found; int result; };
  Memory mem;
  void *lockCtx;
  bool (*lock)(void *); // bounded wait; serializes local callers + hardware lock
  void (*unlock)(void *);
  smem::Info *info=nullptr;   // optional caller-owned parse buffer (~1.7 KiB); null = on the stack
  void *noteCtx=nullptr;
  void (*note)(void *,const Note &)=nullptr;   // existing item of another size (reused or refused); never under the lock
  uint32_t allocations=0,existing=0,lockFailures=0;
  static uint32_t reader(void *ctx,uint32_t at) { return static_cast<SmemArena *>(ctx)->mem.r32(at); }
  smem::Reader view() { return {reader,this,mem.size}; }
  int parse(smem::Info *out) {
    auto r=view(); mem.fence(); smem::parse(r,out);
    return out->error || out->initialized!=1?Protocol:Ok;
  }
  int refresh() { return info?parse(info):Invalid; }
  int find(uint16_t peer,uint32_t id,uint32_t *at,uint32_t *size) {
    if(info) { int e=parse(info); if(e) return e; return smem::find(view(),*info,peer,id,at,size)?Ok:NotFound; }
    smem::Info i={}; int e=parse(&i); if(e) return e;
    return smem::find(view(),i,peer,id,at,size)?Ok:NotFound;
  }
  // Ok with *at = data offset; *created: new (zeroed) item; *actual: its usable size (>= bytes)
  int allocate(uint16_t peer,uint32_t id,uint32_t bytes,uint32_t *at,bool *created=nullptr,uint32_t *actual=nullptr) {
    if(created) *created=false;
    if(!bytes||bytes>0x100000||id<8||id>=smem::kItemCount||!lock||!unlock) return Invalid;
    if(!lock(lockCtx)) { lockFailures++; return Timeout; }
    bool made=false; uint32_t size=0;
    int e;
    if(info) e=allocateLocked(*info,peer,id,bytes,at,&made,&size);
    else { smem::Info i={}; e=allocateLocked(i,peer,id,bytes,at,&made,&size); }
    mem.fence(); unlock(lockCtx);
    if(!e) { if(made) allocations++; else existing++; }
    if(created) *created=made;
    if(actual && !e) *actual=size;
    if(size && !made && (e || align8(size)!=align8(bytes)) && note) note(noteCtx,Note{peer,id,bytes,size,e});
    return e;
  }
private:
  int allocateLocked(smem::Info &i,uint16_t peer,uint32_t id,uint32_t bytes,uint32_t *at,bool *created,uint32_t *usable) {
    int e=parse(&i); if(e) return e;
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
    } else if(i.version==11) found=smem::find(view(),i,peer,id,&oldAt,&oldSize);   // v11 heap: padded sizes
    else return NotReady;
    if(found) {
      *usable=oldSize;
      if(oldSize<bytes) return Range;   // too small for this side: refused (the caller logs item and sizes)
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
      mem.fence(); mem.w32(base+12,free+16+padded); mem.fence();   // header before the free offset
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
    *usable=bytes; *created=true;
    return Ok;
  }
};
} // namespace guampwifi
// The ADSP transports (namespace qipc: GuampSmp2pEdge.inc, GuampGlinkEdge.inc, GuampAdspIpc.inc) use this same layer
// and the same wire primitives; their own classes keep their namespace (no clash with guampwifi::Smp2pEdge/Glink).
namespace qipc {
using guampwifi::Error; using guampwifi::Ok; using guampwifi::Again; using guampwifi::Invalid; using guampwifi::Range;
using guampwifi::NoMemory; using guampwifi::Protocol; using guampwifi::NotReady; using guampwifi::Timeout;
using guampwifi::PeerReset; using guampwifi::RemoteError; using guampwifi::Unsupported; using guampwifi::Busy;
using guampwifi::NotFound; using guampwifi::Io; using guampwifi::errorName;
using guampwifi::le16; using guampwifi::le32; using guampwifi::put16; using guampwifi::put32; using guampwifi::align8;
using guampwifi::nameEquals; using guampwifi::Memory; using guampwifi::Allocator; using guampwifi::SmemArena;
} // namespace qipc
#endif
