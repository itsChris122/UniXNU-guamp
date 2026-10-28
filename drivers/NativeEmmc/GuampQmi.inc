// SPDX-License-Identifier: BSD-2-Clause-Patent
// QMI service transport (no QMUX wrapper on QRTR), strict TLV bounds and one
// outstanding transaction. The actual modem errors remain visible to callers.
#ifndef GUAMP_QMI_INC
#define GUAMP_QMI_INC
#include "GuampWifiWire.inc"
namespace guampwifi {
struct TlvView {
  const uint8_t *data; uint32_t size;
  int validate() const {
    uint32_t seen[8]={};
    for(uint32_t at=0;at<size;) {
      if(size-at<3) return Protocol;
      uint8_t tag=data[at]; uint32_t n=le16(data+at+1);
      if(n>size-at-3||(seen[tag/32]&(1u<<(tag%32)))) return Protocol;
      seen[tag/32]|=1u<<(tag%32); at+=3+n;
    }
    return Ok;
  }
  const uint8_t *find(uint8_t tag,uint32_t *n) const {
    for(uint32_t at=0;at<size;) {
      if(size-at<3) return nullptr;
      uint32_t bytes=le16(data+at+1); if(bytes>size-at-3) return nullptr;
      if(data[at]==tag) { *n=bytes; return data+at+3; }
      at+=3+bytes;
    }
    return nullptr;
  }
};
struct TlvWriter {
  uint8_t *data; uint32_t capacity,size=0;
  bool add(uint8_t tag,const void *p,uint32_t n) {
    if(n>65535||size>capacity||capacity-size<3||n>capacity-size-3||(!p&&n)) return false;
    data[size]=tag; put16(data+size+1,uint16_t(n));
    if(n) memcpy(data+size+3,p,n);
    size+=n+3; return true;
  }
  bool u8(uint8_t tag,uint8_t v) { return add(tag,&v,1); }
  bool u32(uint8_t tag,uint32_t v) { uint8_t p[4]; put32(p,v); return add(tag,p,4); }
  bool u64(uint8_t tag,uint64_t v) { uint8_t p[8]; put64(p,v); return add(tag,p,8); }
};
struct Qmi {
  static constexpr uint32_t MaxBody=6300;
  void *ctx=nullptr;
  int (*transmit)(void *,const uint8_t *,uint32_t)=nullptr;
  void (*indication)(void *,uint16_t,const TlvView &)=nullptr;
  uint8_t request[MaxBody+7]={},response[MaxBody]={};
  uint32_t requestSize=0,responseSize=0;
  uint16_t nextId=1,transaction=0,message=0,result=0,remoteError=0;
  uint64_t deadline=0;
  int status=0;
  bool pending=false,queued=false,done=false;
  void init(void *opaque,int (*send)(void *,const uint8_t *,uint32_t),
            void (*ind)(void *,uint16_t,const TlvView &)) {
    ctx=opaque; transmit=send; indication=ind;
    pending=queued=done=false; requestSize=responseSize=0; status=0;
  }
  void cancel(int reason=PeerReset) {
    if(pending) { status=reason; pending=queued=false; done=true; }
  }
  int begin(uint16_t msg,const TlvView &body,uint64_t now,uint32_t timeoutMs=10000) {
    if(pending||done) return Busy;
    if(!transmit||body.size>MaxBody||body.validate()||!timeoutMs) return Invalid;
    transaction=nextId++; if(!nextId) nextId=1;
    message=msg; result=remoteError=0; status=Ok; responseSize=0;
    request[0]=0; put16(request+1,transaction); put16(request+3,msg); put16(request+5,uint16_t(body.size));
    if(body.size) memcpy(request+7,body.data,body.size);
    requestSize=7+body.size; deadline=now+timeoutMs; pending=true; queued=done=false;
    return step(now);
  }
  int step(uint64_t now) {
    if(!pending) return done?status:Ok;
    if(now>=deadline) { pending=queued=false; done=true; status=Timeout; return status; }
    if(!queued) {
      int e=transmit(ctx,request,requestSize);
      if(e==Again||e==NotReady) return Ok;
      if(e) { pending=false; done=true; status=e; return e; }
      queued=true;
    }
    return Ok;
  }
  int receive(const uint8_t *p,uint32_t n) {
    if(!p||n<7||n!=7u+le16(p+5)||n>MaxBody+7) return Protocol;
    TlvView body={p+7,n-7}; int e=body.validate(); if(e) return e;
    uint16_t msg=le16(p+3);
    if(p[0]==4) { if(indication) indication(ctx,msg,body); return Ok; }
    if(p[0]!=2) return Protocol;
    if(!pending||!queued||le16(p+1)!=transaction||msg!=message) return NotFound;
    uint32_t bytes=0; const uint8_t *r=body.find(2,&bytes);
    if(!r||bytes!=4) return Protocol;
    result=le16(r); remoteError=le16(r+2);
    if(result>1) return Protocol;
    responseSize=body.size; if(body.size) memcpy(response,body.data,body.size);
    pending=queued=false; done=true; status=result?RemoteError:Ok;
    return Ok;
  }
  TlvView reply() const { return {response,responseSize}; }
  void consume() { done=false; responseSize=0; }
};
} // namespace guampwifi
#endif
