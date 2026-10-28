// SPDX-License-Identifier: BSD-2-Clause-Patent
// RPM message-RAM GLINK edge and request/ACK codec. Layouts checked against
// Motorola qcom_glink_rpm.c and smd-rpm.c. No vote is considered applied until
// its matching ACK arrives; unknown/late ACKs cannot complete a new request.
#ifndef GUAMP_WIFI_RPM_INC
#define GUAMP_WIFI_RPM_INC
#include "GuampGlink.inc"
namespace guampwifi {
#if defined(GUAMP_WIFI_RUNTIME) && GUAMP_WIFI_RUNTIME
// guamp kernel build: the RPM GLINK edge has ONE owner, the rpm:: client of NativeEmmc (GuampRpm.inc, also used by
// guamp-adsp, guamp-bt-ldo, guamp-gpu-rpm). Two clients on the same message RAM FIFO would corrupt each other, so
// with the runtime every Wi-Fi vote goes through that client (same rpm-smd request, ACK awaited before success).
// Same interface as the standalone edge below; vote() completes synchronously (done/error set on return).
extern "C" int guamp_rpm_shared_ready(void);
extern "C" int guamp_rpm_shared_vote(uint32_t resource, uint32_t id, uint32_t key, uint32_t value, uint32_t state,
                                     uint32_t timeoutMs);
struct Rpm {
  bool waiting=false,done=false,started=false;
  int error=0;
  int start(const Memory &,const Allocator &,void *,void (*kick)(void *),uint32_t=1) {
    if(!kick) return Invalid;
    started=true;waiting=done=false;error=0;return Ok;
  }
  bool online() { return started&&guamp_rpm_shared_ready(); }
  int vote(uint32_t resource,uint32_t id,uint32_t key,uint32_t value,uint32_t state,uint64_t) {
    if(waiting||done) return Busy;
    if(!online()) return NotReady;
    if(state>1) return Range;
    const int rc=guamp_rpm_shared_vote(resource,id,key,value,state,5000);
    done=true;error=rc?RemoteError:Ok;return Ok;
  }
  int step(uint64_t) { return Ok; }
  void consume() { done=false; error=0; }
  void stop() { started=waiting=done=false; }
};
#else
struct Rpm {
  static constexpr uint32_t Request=0x00716572,MessageId=0x2367736d,Error=0x00727265;
  Glink glink={}; Memory memory={};
  void *ctx=nullptr; void (*notify)(void *)=nullptr;
  uint16_t channel=0; uint32_t nextId=1,pendingId=0;
  uint8_t request[256]={}; uint32_t requestBytes=0;
  uint64_t deadline=0;
  bool waiting=false,queued=false,done=false;
  int error=0;
  static int pipes(const Memory &m,GlinkPipe *tx,GlinkPipe *rx) {
    if(m.size<512) return Range;
    uint32_t table=m.size-256;
    if(m.r32(table)!=0x67727430) return Protocol;
    uint32_t count=m.r32(table+4); if(count>20) return Protocol;
    bool haveTx=false,haveRx=false;
    for(uint32_t i=0;i<count;i++) {
      uint32_t e=table+8+12*i,id=m.r32(e),at=m.r32(e+4),bytes=m.r32(e+8);
      if(id!=0x61703272&&id!=0x72326170) continue;
      if((at&7)||bytes<64||(bytes&7)||at>table||bytes>table-at||table-at-bytes<8) return Range;
      GlinkPipe p={m,at+8,bytes,at+4,at,id==0x61703272};
      if(p.tx) { if(haveTx) return Protocol; *tx=p; haveTx=true; }
      else { if(haveRx) return Protocol; *rx=p; haveRx=true; }
    }
    if(!haveTx||!haveRx) return NotFound;
    uint32_t tStart=tx->fifo-8,tEnd=tx->fifo+tx->length;
    uint32_t rStart=rx->fifo-8,rEnd=rx->fifo+rx->length;
    return tStart<rEnd&&rStart<tEnd?Range:Ok;
  }
  int start(const Memory &m,const Allocator &a,void *opaque,void (*kick)(void *),uint32_t firstId=1) {
    if(!kick) return Invalid;
    GlinkPipe tx={},rx={}; int e=pipes(m,&tx,&rx); if(e) return e;
    memory=m; ctx=opaque; notify=kick; channel=0; nextId=firstId?firstId:1;
    pendingId=0; waiting=queued=done=false; error=0;
    return glink.start(tx,rx,a,this,glinkKick,glinkEvent,glinkReceive,true);
  }
  bool online() { return channel&&glink.online(channel); }
  int vote(uint32_t resource,uint32_t id,uint32_t key,uint32_t value,uint32_t state,uint64_t now) {
    uint8_t kv[12]; put32(kv,key); put32(kv+4,4); put32(kv+8,value);
    return send(resource,id,kv,sizeof(kv),state,now);
  }
  int send(uint32_t resource,uint32_t id,const uint8_t *kv,uint32_t n,uint32_t state,uint64_t now) {
    if(waiting||done) return Busy;
    if(!online()) return NotReady;
    if(state>1||!n||(n&3)||n>=256-28||!kv) return Range;
    // Validate key/value lengths, including each padded payload.
    for(uint32_t off=0;off<n;) {
      if(n-off<8) return Protocol;
      uint32_t len=le32(kv+off+4); if(len>n-off-8||len>216) return Protocol;
      uint32_t padded=(len+3)&~3u; if(padded>n-off-8) return Protocol;
      off+=8+padded;
    }
    pendingId=nextId++; if(!nextId) nextId=1;
    memset(request,0,sizeof(request)); put32(request,Request); put32(request+4,20+n);
    put32(request+8,pendingId); put32(request+12,state); put32(request+16,resource);
    put32(request+20,id); put32(request+24,n); memcpy(request+28,kv,n);
    requestBytes=28+n; waiting=true; queued=done=false; error=0; deadline=now+5000;
    return Ok;
  }
  int step(uint64_t now) {
    if(error&&!waiting) return error;
    int e=glink.step(); if(e) return fail(e);
    if(glink.negotiated&&!channel) { e=glink.open("rpm_requests",&channel); if(e!=Again&&e) return fail(e); }
    if(waiting) {
      if(now>=deadline) return fail(Timeout);
      if(!queued) {
        e=glink.send(channel,request,requestBytes);
        if(e==Again||e==NotReady) return Ok;
        if(e) return fail(e);
        queued=true;
      }
    }
    return error;
  }
  int receive(const uint8_t *p,uint32_t n) {
    if(!p||n<8||le32(p)!=Request||le32(p+4)!=n-8) return Protocol;
    bool haveId=false,failed=false; uint32_t id=0;
    for(uint32_t off=8;off<n;) {
      if(n-off<8) return Protocol;
      uint32_t type=le32(p+off),len=le32(p+off+4);
      if(len>n-off-8) return Protocol;
      if(type==MessageId) {
        if(haveId||len!=4) return Protocol;
        id=le32(p+off+8); haveId=true;
      } else if(type==Error) failed=true;
      uint32_t padded=(len+3)&~3u; if(padded>n-off-8) return Protocol;
      off+=8+padded;
    }
    if(!haveId) return Protocol;
    if(!waiting||!queued||id!=pendingId) return NotFound;
    waiting=queued=false; done=true; error=failed?RemoteError:Ok; return Ok;
  }
  void consume() { done=false; error=0; }
  void stop() { glink.stop(); waiting=queued=done=false; channel=0; }
private:
  int fail(int e) { waiting=queued=false; done=true; error=e; return e; }
  static void glinkKick(void *p) { auto &r=*static_cast<Rpm *>(p); r.notify(r.ctx); }
  static void glinkEvent(void *p,uint16_t id,const char *name,bool up) {
    auto &r=*static_cast<Rpm *>(p);
    if(!nameEquals(name,"rpm_requests",32)) return;
    if(up) r.channel=id; else { r.channel=0; r.fail(PeerReset); }
  }
  static void glinkReceive(void *p,uint16_t id,const uint8_t *v,uint32_t n) {
    auto &r=*static_cast<Rpm *>(p); if(id!=r.channel) return;
    int e=r.receive(v,n); if(e!=NotFound&&e) r.fail(e);
  }
};
#endif // GUAMP_WIFI_RUNTIME
} // namespace guampwifi
#endif
