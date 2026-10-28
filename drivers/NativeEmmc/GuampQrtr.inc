// SPDX-License-Identifier: BSD-2-Clause-Patent
// Bounded QRTR endpoint. v1 TX, v1/v2 RX, service discovery,
// NEW/DEL_SERVER, BYE/DEL_CLIENT and confirmation/resume flow control.
#ifndef GUAMP_QRTR_INC
#define GUAMP_QRTR_INC
#include "GuampWifiWire.inc"
namespace guampwifi {
struct Qrtr {
  static constexpr uint32_t LocalNode=1,LocalPort=0x4000,Ctrl=0xfffffffeu,Broadcast=0xffffffffu;
  // QMI/QRTR packs version in bits7:0 and instance in bits31:8.
  // Guamp ICNSS looks up WLFW service0x45, version1, instance0 => wire1.
  static constexpr uint32_t Service=0x45,ServiceVersion=1,ServiceInstanceId=0;
  static constexpr uint32_t ServiceInstance=ServiceVersion|(ServiceInstanceId<<8),MaxPacket=0x4400;
  enum Type : uint32_t { Data=1,Hello=2,Bye=3,NewServer=4,DelServer=5,DelClient=6,
    Resume=7,Exit=8,Ping=9,NewLookup=10,DelLookup=11,DelProc=13 };
  struct Datagram { uint8_t *data; uint32_t bytes,type; };
  struct LocalService { uint32_t service,instance,port; void *ctx; void (*receive)(void *,uint32_t,uint32_t,const uint8_t *,uint32_t); bool used; };
  struct Header { uint32_t type,sn,sp,dn,dp,size,header; bool confirm; };
  Allocator allocator={};
  void *ctx=nullptr;
  int (*transmit)(void *,const uint8_t *,uint32_t)=nullptr;
  void (*serviceEvent)(void *,bool)=nullptr;
  void (*received)(void *,const uint8_t *,uint32_t)=nullptr;
  Datagram queue[16]={};
  LocalService local[4]={};
  uint32_t read=0,write=0,count=0,remoteNode=0,serverNode=0,serverPort=0;
  uint32_t flow=0;
  bool running=false,helloSent=false,remoteKnown=false;
  int start(const Allocator &a,void *opaque,int (*send)(void *,const uint8_t *,uint32_t),
            void (*event)(void *,bool),void (*data)(void *,const uint8_t *,uint32_t)) {
    if(running) return Busy;
    if(!a.alloc||!a.free||!send||!data) return Invalid;
    allocator=a; ctx=opaque; transmit=send; serviceEvent=event; received=data;
    memset(queue,0,sizeof(queue)); read=write=count=remoteNode=serverNode=serverPort=flow=0;
    running=true; helloSent=true;remoteKnown=false;
    int e=control(Hello,Broadcast,Ctrl,0,0,0,0);if(e)return e;
    // WifiControl registers QFTP/SERVREG before this transport is opened.
    // Publish those endpoints as soon as QRTR starts, as well as answering
    // explicit NEW_LOOKUP requests from the remote namespace.
    for(const auto &s:local)if(s.used) {e=announce(s,true);if(e)return e;}
    return Ok;
  }
  void stop() {
    if(serverPort&&serviceEvent) serviceEvent(ctx,false);
    serverNode=serverPort=0; running=false;
    for(auto &q:queue) { if(q.data) allocator.free(allocator.ctx,q.data,q.bytes); q={}; }
    read=write=count=flow=0;
  }
  bool serviceReady() const { return running&&serverPort; }
  int addLocalService(uint32_t service,uint32_t instance,uint32_t port,void *opaque,
                     void (*handler)(void *,uint32_t,uint32_t,const uint8_t *,uint32_t)) {
    if(!service||!port||port>=Ctrl||!handler)return Invalid;
    for(auto &s:local)if(s.used&&s.port==port)
      return s.service==service&&s.instance==instance&&s.ctx==opaque&&s.receive==handler?Ok:Busy;
    for(auto &s:local)if(!s.used) {
      LocalService candidate={service,instance,port,opaque,handler,true};
      int e=running?announce(candidate,true):Ok;
      // A full transport queue must not leave a silent registration behind:
      // the caller can retry and still publish NEW_SERVER to the remote.
      if(e)return e;
      s=candidate;return Ok;
    }
    return NoMemory;
  }
  int sendFrom(uint32_t sourcePort,uint32_t node,uint32_t port,const uint8_t *p,uint32_t n) {
    if(!running||!sourcePort||node==Broadcast||!port)return NotReady;
    return enqueuePacket(Data,sourcePort,node,port,p,n,false);
  }
  int send(const uint8_t *data,uint32_t bytes) {
    if(!serviceReady()) return NotReady;
    if(flow>=10) return Again;
    bool confirm=flow==4;
    int e=enqueuePacket(Data,LocalPort,serverNode,serverPort,data,bytes,confirm);
    if(!e) flow++;
    return e;
  }
  int step(uint32_t budget=8) {
    if(!running) return NotReady;
    for(uint32_t k=0;k<budget&&count;k++) {
      auto &p=queue[read]; int e=transmit(ctx,p.data,p.bytes);
      if(e==Again||e==NotReady) return Again;
      if(e) return e;
      allocator.free(allocator.ctx,p.data,p.bytes); p={}; read=(read+1)%16; count--;
    }
    return Ok;
  }
  static int decode(const uint8_t *p,uint32_t n,Header *h) {
    if(!p||!h||n<16) return Protocol;
    *h={};
    if(le32(p)==1) {
      if(n<32) return Protocol;
      h->type=le32(p+4); h->sn=le32(p+8); h->sp=le32(p+12);
      h->confirm=le32(p+16)!=0; h->size=le32(p+20);
      h->dn=le32(p+24); h->dp=le32(p+28); h->header=32;
      if(h->sp==0xffffu) h->sp=Ctrl;
      if(h->dp==0xffffu) h->dp=Ctrl;
    } else if(p[0]==3) {
      if(p[2]&~1u) return Protocol;
      h->type=p[1]; h->confirm=(p[2]&1)!=0; h->size=le32(p+4); h->header=16+p[3];
      h->sn=le16(p+8); h->sp=le16(p+10); h->dn=le16(p+12); h->dp=le16(p+14);
      if(h->sn==0xffffu) h->sn=Broadcast;
      if(h->dn==0xffffu) h->dn=Broadcast;
      if(h->sp==0xffffu) h->sp=Ctrl;
      if(h->dp==0xffffu) h->dp=Ctrl;
    } else return Unsupported;
    if(h->header>n||h->size>n-h->header||n-h->header-h->size>3) return Protocol;
    // QRTR node0 is valid (only node0/port0 together is a namespace EOF).
    if(h->sn==LocalNode||h->sn==Broadcast) return Protocol;
    if(h->dn!=LocalNode&&h->dn!=Broadcast) return NotFound;
    return Ok;
  }
  int receive(const uint8_t *packet,uint32_t n) {
    if(!running) return NotReady;
    Header h={}; int e=decode(packet,n,&h); if(e) return e;
    const uint8_t *p=packet+h.header;
    if(!remoteKnown) {remoteNode=h.sn;remoteKnown=true;}
    if(h.type==Data) {
      for(auto &s:local)if(s.used&&h.dp==s.port) {
        if(h.confirm) {
          e=control(Resume,h.sn,h.sp,LocalNode,s.port,0,0);if(e)return e;
        }
        s.receive(s.ctx,h.sn,h.sp,p,h.size);return Ok;
      }
    }
    if(h.type==Data&&h.dp==LocalPort) {
      if(!serviceReady()||h.sn!=serverNode||h.sp!=serverPort) return NotFound;
      if(h.confirm) {
        e=control(Resume,h.sn,h.sp,LocalNode,LocalPort,0,0); if(e) return e;
      }
      received(ctx,p,h.size); return Ok;
    }
    if(h.size<4) return Protocol;
    uint32_t cmd=le32(p);
    if(h.type!=Data&&h.type!=cmd) return Protocol;
    if(h.type==Data&&h.dp!=Ctrl) return NotFound;
    switch(cmd) {
      case Hello:
        // One HELLO per link; echoing every HELLO would cause an endless loop.
        if(!helloSent) { e=control(Hello,h.sn,Ctrl,0,0,0,0); if(e) return e; helloSent=true; }
        return control(NewLookup,h.sn,Ctrl,Service,ServiceInstance,0,0);
      case NewServer: case DelServer: {
        if(h.size!=20) return Protocol;
        uint32_t service=le32(p+4),instance=le32(p+8),node=le32(p+12),port=le32(p+16);
        if(service!=Service||instance!=ServiceInstance) return Ok;
        if(!node&&!port)return Ok; // namespace lookup EOF, not a service
        if(node==LocalNode||node==Broadcast||!port||port>=Ctrl) return Protocol;
        if(cmd==NewServer) {
          if(serverPort) return serverNode==node&&serverPort==port?Ok:Busy;
          serverNode=node; serverPort=port; flow=0;
          if(serviceEvent) serviceEvent(ctx,true);
        } else if(node==serverNode&&port==serverPort) lost();
        return Ok;
      }
      case Resume:
        if(h.size<12) return Protocol;
        if(le32(p+4)==serverNode&&le32(p+8)==serverPort) flow=0;
        return Ok;
      case DelClient:
        if(h.size<12) return Protocol;
        if(le32(p+4)==serverNode&&le32(p+8)==serverPort) lost();
        return Ok;
      case Bye: case Exit:
        if(h.sn==serverNode) lost();
        return Ok;
      case DelProc:
        if(h.size<12) return Protocol;
        if(le32(p+8)==serverNode) lost();
        return Ok;
      case NewLookup: {
        if(h.size!=20)return Protocol;
        uint32_t service=le32(p+4),instance=le32(p+8);
        for(const auto &s:local) {
          if(!s.used||s.service!=service||(instance&&s.instance!=instance))continue;
          // Act as the remote QRTR namespace for local endpoints: answer the
          // modem's lookup with the service's real local port. This is needed
          // before it can open service 4096 and request wlanmdsp.mbn by TFTP.
          e=controlFrom(Ctrl,NewServer,h.sn,h.sp,s.service,s.instance,LocalNode,s.port);
          if(e)return e;
        }
        return Ok;
      }
      case Ping: case DelLookup: return Ok;
      default: return Unsupported;
    }
  }
private:
  int announce(const LocalService &s,bool up) {
    // QRTR namespace announcements use the control port and broadcast node.
    // The payload carries the actual local node/port used by this endpoint.
    return control(up?NewServer:DelServer,Broadcast,Ctrl,s.service,s.instance,LocalNode,s.port);
  }
  void lost() {
    if(serviceEvent) serviceEvent(ctx,false);
    serverNode=serverPort=flow=0;
    // A dead peer's queued requests must never be replayed to its successor.
    for(auto &q:queue) { if(q.data) allocator.free(allocator.ctx,q.data,q.bytes); q={}; }
    read=write=count=0;
  }
  int enqueuePacket(uint32_t type,uint32_t sp,uint32_t dn,uint32_t dp,
              const uint8_t *data,uint32_t n,bool confirm=false) {
    if(!running) return NotReady;
    if(n>MaxPacket-32||(!data&&n)) return Range;
    if(count==16) return Again;
    uint32_t bytes=32+((n+3)&~3u);
    auto *p=static_cast<uint8_t *>(allocator.alloc(allocator.ctx,bytes)); if(!p) return NoMemory;
    memset(p,0,bytes); put32(p,1); put32(p+4,type); put32(p+8,LocalNode); put32(p+12,sp);
    put32(p+16,confirm?1u:0u); put32(p+20,n); put32(p+24,dn); put32(p+28,dp);
    if(n) memcpy(p+32,data,n);
    queue[write]={p,bytes,type}; write=(write+1)%16; count++;
    return Ok;
  }
  int control(uint32_t cmd,uint32_t dn,uint32_t dp,uint32_t a,uint32_t b,uint32_t c,uint32_t d) {
    return controlFrom(Ctrl,cmd,dn,dp,a,b,c,d);
  }
  int controlFrom(uint32_t source,uint32_t cmd,uint32_t dn,uint32_t dp,uint32_t a,uint32_t b,uint32_t c,uint32_t d) {
    uint8_t packet[20]={}; put32(packet,cmd); put32(packet+4,a); put32(packet+8,b);
    put32(packet+12,c); put32(packet+16,d);
    return enqueuePacket(cmd,source,dn,dp,packet,sizeof(packet));
  }
};
} // namespace guampwifi
#endif
