// SPDX-License-Identifier: BSD-2-Clause-Patent
// WCN3990-family SNOC copy engines, 16-byte DMA descriptors. Register layouts
// and wire formats checked against ath10k/hw.c, ce.c and ce.h (Linux v6.6).
// The caller owns verified SMMU mappings, coherent DMA storage and serialization.
// A physical XNU address is never treated as a WLAN IOVA.
#ifndef GUAMP_SNOC_CE_INC
#define GUAMP_SNOC_CE_INC
#include "GuampWifiWire.inc"
namespace guampwifi {
struct SnocCe {
  static constexpr uint32_t RegisterBase=0x240000,RegisterStride=0x1000;
  static constexpr uint64_t IovaBase=0xa0000000,IovaEnd=0xb0000000;
  struct Buffer { uint64_t iova; uint32_t bytes; void *token; };
  struct Slot { Buffer buffer; bool posted; };
  struct Ring {
    Memory descriptors; uint64_t iova; uint32_t count,write,read,outstanding;
    Slot *slots;
  };
  Memory registers={}; Ring source={},destination={};
  uint32_t engine=0,base=0,maximum=0;
  bool active=false;
  void *ctx=nullptr;
  void (*txDone)(void *,const Buffer &)=nullptr;
  void (*rxDone)(void *,const Buffer &,uint32_t)=nullptr;
  static bool dmaRange(uint64_t address,uint32_t bytes) {
    return bytes&&address>=IovaBase&&address<IovaEnd&&bytes<=IovaEnd-address;
  }
  static int validateRing(const Ring &r) {
    if(!r.count) return Ok;
    if(r.count<2||r.count>8192||(r.count&(r.count-1))||!r.slots||(r.iova&7)||
       !dmaRange(r.iova,r.count*16)||!r.descriptors.valid(0,r.count*16)||
       !r.descriptors.read||!r.descriptors.write||!r.descriptors.barrier) return Invalid;
    return Ok;
  }
  int start(const Memory &mmio,uint32_t ce,const Ring &src,const Ring &dst,uint32_t maxBytes,
            bool hardwareQuiesced,void *opaque,void (*tx)(void *,const Buffer &),
            void (*rx)(void *,const Buffer &,uint32_t)) {
    if(active) return Busy;
    if(!hardwareQuiesced) return NotReady;
    if(ce>=12||!maxBytes||maxBytes>65535||validateRing(src)||validateRing(dst)||
       (!src.count&&!dst.count)||(src.count&&!tx)||(dst.count&&!rx)||!mmio.read||!mmio.write||!mmio.barrier) return Invalid;
    registers=mmio; engine=ce; base=RegisterBase+ce*RegisterStride;
    if(!registers.valid(base,0x54)) return Range;
    source=src; destination=dst; maximum=maxBytes; ctx=opaque; txDone=tx; rxDone=rx;
    // A live/stale CE must be reset by its firmware before reusing its rings.
    if((src.count&&(r(0x3c)||r(0x44)))||(dst.count&&(r(0x40)||r(0x48)))) return Busy;
    w(0x2c,r(0x2c)&~1u); w(0x34,0); w(0x30,0x1f); w(0x38,0x3e0);
    uint32_t control=r(0x18);
    control&=~((1u<<17)|(1u<<18)|(1u<<19)|0xffffu); control|=maximum;
    w(0x18,control);
    if(source.count) {
      setup(source); w(0,uint32_t(source.iova)); w(4,uint32_t(source.iova>>32)); w(8,source.count);
      w(0x4c,source.count); w(0x3c,0);
    }
    if(destination.count) {
      setup(destination); w(0xc,uint32_t(destination.iova)); w(0x10,uint32_t(destination.iova>>32));
      w(0x14,destination.count); w(0x50,destination.count); w(0x40,0);
    }
    registers.fence(); active=true; return Ok;
  }
  int enableInterrupt(bool on) {
    if(!active) return NotReady;
    w(0x2c,(r(0x2c)&~1u)|(on?1u:0u)); registers.fence(); return Ok;
  }
  int transmit(const Buffer &buffer,uint16_t metadata=0) {
    if(!active||!source.count) return NotReady;
    if(!dmaRange(buffer.iova,buffer.bytes)||buffer.bytes>maximum||metadata>0xfff) return Range;
    if(source.outstanding==source.count-1) return Again;
    auto &slot=source.slots[source.write]; if(slot.posted) return Protocol;
    uint32_t at=source.write*16;
    source.descriptors.w32(at,uint32_t(buffer.iova));
    source.descriptors.w32(at+4,uint32_t(buffer.iova>>32)&0x1fu);
    source.descriptors.w32(at+8,buffer.bytes|(uint32_t(metadata)<<20));
    source.descriptors.w32(at+12,0);
    slot={buffer,true}; source.descriptors.fence();
    source.write=(source.write+1)&(source.count-1); source.outstanding++;
    w(0x3c,source.write); registers.fence(); return Ok;
  }
  int postReceive(const Buffer &buffer) {
    if(!active||!destination.count) return NotReady;
    if(!dmaRange(buffer.iova,buffer.bytes)||buffer.bytes<maximum) return Range;
    if(destination.outstanding==destination.count-1) return Again;
    auto &slot=destination.slots[destination.write]; if(slot.posted) return Protocol;
    uint32_t at=destination.write*16;
    destination.descriptors.w32(at,uint32_t(buffer.iova));
    destination.descriptors.w32(at+4,uint32_t(buffer.iova>>32)&0x1fu);
    destination.descriptors.w32(at+8,0); destination.descriptors.w32(at+12,0);
    slot={buffer,true}; destination.descriptors.fence();
    destination.write=(destination.write+1)&(destination.count-1); destination.outstanding++;
    w(0x40,destination.write); registers.fence(); return Ok;
  }
  int poll(uint32_t budget=64) {
    if(!active) return NotReady;
    uint32_t errors=r(0x38)&0x3e0u; if(errors) return Protocol;
    uint32_t completed=0;
    if(source.count) {
      uint32_t hw=r(0x44);
      if(hw>=source.count) return Protocol;
      uint32_t done=(hw-source.read)&(source.count-1);
      if(done>source.outstanding) return Protocol;
      while(source.read!=hw&&completed<budget) {
        auto &slot=source.slots[source.read]; if(!slot.posted) return Protocol;
        source.descriptors.fence(); Buffer b=slot.buffer; slot={};
        source.read=(source.read+1)&(source.count-1); source.outstanding--; completed++;
        txDone(ctx,b);
      }
    }
    if(destination.count) {
      while(destination.outstanding&&completed<budget) {
        auto &slot=destination.slots[destination.read]; if(!slot.posted) return Protocol;
        destination.descriptors.fence(); uint32_t at=destination.read*16;
        uint32_t n=destination.descriptors.r32(at+8)&0xffffu;
        if(!n) break; // descriptor completion may lag the hardware read index
        if(n>maximum||n>slot.buffer.bytes) return Range;
        destination.descriptors.w32(at+8,0); destination.descriptors.fence();
        Buffer b=slot.buffer; slot={}; destination.read=(destination.read+1)&(destination.count-1);
        destination.outstanding--; completed++; rxDone(ctx,b,n);
      }
    }
    w(0x30,1); registers.fence(); return Ok;
  }
  // Only after actual hardware/PAS shutdown. Never unmap/free a live DMA ring.
  int release(bool hardwareStopped) {
    if(!hardwareStopped) return Busy;
    if(active) { w(0x2c,r(0x2c)&~1u); registers.fence(); }
    active=false; source={}; destination={}; return Ok;
  }
private:
  uint32_t r(uint32_t off) const { registers.fence(); return registers.r32(base+off); }
  void w(uint32_t off,uint32_t v) const { registers.w32(base+off,v); }
  static void setup(Ring &ring) {
    ring.descriptors.zero(0,ring.count*16); ring.descriptors.fence();
    memset(ring.slots,0,ring.count*sizeof(Slot)); ring.write=ring.read=ring.outstanding=0;
  }
};
} // namespace guampwifi
#endif
