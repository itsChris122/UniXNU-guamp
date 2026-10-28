// SPDX-License-Identifier: BSD-2-Clause-Patent
// GLINK native v1 over the modem SMEM FIFOs (478/479/480).
// Protocol studied in Motorola qcom_glink_native.c and qcom_glink_smem.c.
// Single serialized worker; callbacks may enqueue sends, must not recurse into step().
#ifndef GUAMP_GLINK_INC
#define GUAMP_GLINK_INC
#include "GuampWifiWire.inc"
namespace guampwifi {
struct GlinkPipe {
  Memory mem={};
  uint32_t fifo=0,length=0,head=0,tail=0;
  bool tx=false;
  int validate() const {
    if(length<64||(length&7)||(fifo&7)||(head&3)||(tail&3)||
       !mem.read||!mem.write||!mem.barrier||!mem.valid(fifo,length)||!mem.valid(head,4)||!mem.valid(tail,4)) return Invalid;
    return Ok;
  }
  int count(uint32_t *n) const {
    if(validate()) return Invalid;
    mem.fence(); const uint32_t h=mem.r32(head),t=mem.r32(tail);
    if(h>=length||t>=length||(h&7)||(t&7)) return Protocol;
    uint32_t used=h>=t?h-t:length-t+h;
    *n=tx?(length-used>=16?length-used-16:0):used;
    return Ok;
  }
  bool peek(uint32_t relative,uint8_t *dst,uint32_t n) const {
    if(relative>length||n>length-relative) return false;
    uint32_t p=(mem.r32(tail)+relative)%length;
    uint32_t first=n<length-p?n:length-p;
    return mem.copyOut(fifo+p,dst,first)&&mem.copyOut(fifo,dst+first,n-first);
  }
  int advance(uint32_t n) const {
    uint32_t avail=0; int e=count(&avail); if(e) return e;
    if(tx||(n&7)||n>avail) return Range;
    mem.fence(); mem.w32(tail,(mem.r32(tail)+n)%length); mem.fence(); return Ok;
  }
  int push(const uint8_t *a,uint32_t an,const uint8_t *b=nullptr,uint32_t bn=0,bool reserved=false) const {
    if(!tx||an>length||bn>length-an||an+bn>length-8) return Range;
    uint32_t avail=0; int e=count(&avail); if(e) return e;
    const uint32_t bytes=align8(an+bn);
    if(reserved) {
      const uint32_t h=mem.r32(head),t=mem.r32(tail);
      const uint32_t used=h>=t?h-t:length-t+h;
      avail=length-used>=8?length-used-8:0;
    }
    if(bytes>avail) return Again;
    uint32_t pos=mem.r32(head);
    // All FIFO writes are aligned words; neither memcpy nor memset touches MMIO.
    for(uint32_t i=0;i<bytes;i+=4) {
      uint32_t w=0;
      for(uint32_t j=0;j<4;j++) {
        uint32_t k=i+j; uint8_t v=k<an?a[k]:(k-an<bn?b[k-an]:0);
        w|=uint32_t(v)<<(8*j);
      }
      mem.w32(fifo+pos,w); pos+=4; if(pos==length) pos=0;
    }
    mem.fence(); mem.w32(head,pos); mem.fence(); return Ok;
  }
};

struct Glink {
  enum Command : uint16_t { Version=0,VersionAck=1,Open=2,Close=3,OpenAck=4,
    Intent=5,RxDone=6,IntentReq=7,IntentReqAck=8,Data=9,CloseAck=11,
    DataCont=12,ReadNotif=13,RxDoneReuse=14,Signals=15 };
  static constexpr uint32_t MaxChannels=4,MaxLocal=10,MaxRemote=32,MaxPacket=0x4400;
  struct LocalIntent {
    uint8_t *data; uint32_t size,used,remaining; bool active,advertised;
  };
  struct RemoteIntent { uint32_t id,size; bool used; };
  struct Channel {
    char name[32]; uint16_t local,remote;
    bool openSent,openAck,remoteOpen,announced,requestPending;
    LocalIntent localIntent[MaxLocal];
    RemoteIntent remoteIntent[MaxRemote];
    uint8_t *pending; uint32_t pendingSize,pendingAt,pendingIntent;
  };
  struct Control { uint8_t bytes[96]; uint32_t size; };
  GlinkPipe tx={},rx={};
  Allocator allocator={};
  void *ctx=nullptr;
  void (*kick)(void *)=nullptr;
  void (*channelEvent)(void *,uint16_t,const char *,bool)=nullptr;
  void (*received)(void *,uint16_t,const uint8_t *,uint32_t)=nullptr;
  Channel channels[MaxChannels]={};
  Control control[32]={};
  uint32_t qRead=0,qWrite=0,qCount=0,features=1;
  uint32_t rxPackets=0,txPackets=0,readNotifications=0;
  bool negotiated=false,notifySent=false,running=false,intentless=false;
  int error=0;
  // Optional local-user filter for remote OPENs (null: every name, as the RPM edge). Diagnostics below.
  bool (*wants)(void *,const char *)=nullptr;
  uint32_t rxCommands=0,ignoredOpens=0,ignoredCommands=0,droppedIntents=0;
  char lastIgnored[32]={};
  const char *lastRefusal=nullptr;
  uint32_t refusedCmd=0,refusedP1=0,refusedP2=0,refusedBytes=0,refusedAvail=0,refusedTail=0,refusedHead=0;

  int start(const GlinkPipe &out,const GlinkPipe &in,const Allocator &a,
            void *opaque,void (*notify)(void *),
            void (*event)(void *,uint16_t,const char *,bool),
            void (*data)(void *,uint16_t,const uint8_t *,uint32_t),bool rpmIntentless=false) {
    if(running) return Busy;
    if(out.validate()||in.validate()||!out.tx||in.tx||!a.alloc||!a.free||!notify||!data) return Invalid;
    tx=out; rx=in; allocator=a; ctx=opaque; kick=notify; channelEvent=event; received=data;
    memset(channels,0,sizeof(channels)); memset(control,0,sizeof(control));
    qRead=qWrite=qCount=0; intentless=rpmIntentless; features=intentless?0u:1u; error=0; negotiated=notifySent=false;
    rxPackets=txPackets=readNotifications=0;
    rxCommands=ignoredOpens=ignoredCommands=droppedIntents=0; lastIgnored[0]=0; lastRefusal=nullptr;
    refusedCmd=refusedP1=refusedP2=refusedBytes=refusedAvail=refusedTail=refusedHead=0;
    // Remote-owned head/tail words are never cleared by APPS.
    tx.mem.w32(tx.head,0); rx.mem.w32(rx.tail,0); tx.mem.fence();
    running=true; return queue(Version,1,features);
  }
  void stop(int reason=PeerReset) {
    if(!running) return;
    running=false; error=reason;
    for(auto &c:channels) release(c);
    qCount=qRead=qWrite=0; negotiated=false;
  }
  int open(const char *name,uint16_t *id=nullptr) {
    if(!running||!negotiated) return NotReady;
    uint32_t n=0; while(n<32&&name[n]) n++;
    if(!n||n>=32) return Invalid;
    for(auto &c:channels) if(c.local&&nameEquals(c.name,name,32)) {
      if(id) *id=c.local;
      return Ok;
    }
    Channel *c=create(name,n);
    if(!c) return NoMemory;
    int e=queue(Open,c->local,n+1,reinterpret_cast<const uint8_t *>(name),n+1);
    if(e) { memset(c,0,sizeof(*c)); return e; }
    c->openSent=true; if(id) *id=c->local; return Ok;
  }
  bool online(uint16_t id) {
    Channel *c=local(id); return c&&c->announced&&running;
  }
  int send(uint16_t id,const uint8_t *data,uint32_t n) {
    Channel *c=local(id);
    if(!running||!c||!c->announced) return NotReady;
    if(!data||!n||n>MaxPacket) return Range;
    if(c->pending) return Again;
    RemoteIntent *ri=nullptr;
    for(auto &r:c->remoteIntent) if(r.id&&!r.used&&r.size>=n&&(!ri||r.size<ri->size)) ri=&r;
    if(!ri&&!intentless) {
      if(!c->requestPending) {
        int e=queue(IntentReq,c->local,n); if(e) return e;
        c->requestPending=true;
      }
      return Again;
    }
    auto *p=static_cast<uint8_t *>(allocator.alloc(allocator.ctx,n)); if(!p) return NoMemory;
    memcpy(p,data,n); c->pending=p; c->pendingSize=n; c->pendingAt=0;
    c->pendingIntent=ri?ri->id:0; if(ri) ri->used=true; return Ok;
  }
  int step(uint32_t budget=32) {
    if(!running) return error?error:NotReady;
    int e=flush(); if(e!=Ok&&e!=Again) return fail(e);
    for(uint32_t k=0;k<budget;k++) {
      if(qCount>28) break; // room for every command's immediate responses
      e=receiveOne();
      if(e==Again) break;
      if(e) return fail(e);
      e=flush(); if(e!=Ok&&e!=Again) return fail(e);
    }
    if(!qCount) {
      for(auto &c:channels) {
        if(c.pending) {
          e=sendFragment(c);
          if(e!=Ok&&e!=Again) return fail(e);
          if(e==Again) break;
        }
      }
    }
    return Ok;
  }
private:
  Channel *create(const char *name,uint32_t n) {
    for(uint32_t k=0;k<MaxChannels;k++) if(!channels[k].local) {
      Channel *c=&channels[k]; memset(c,0,sizeof(*c)); c->local=uint16_t(k+1); memcpy(c->name,name,n); c->name[n]=0; return c;
    }
    return nullptr;
  }
  int fail(int e) { stop(e); return e; }
  Channel *local(uint16_t id) {
    for(auto &c:channels) if(c.local==id&&id) return &c;
    return nullptr;
  }
  Channel *remote(uint16_t id) {
    for(auto &c:channels) if(c.remote==id&&id) return &c;
    return nullptr;
  }
  void release(Channel &c) {
    if(c.announced&&channelEvent) channelEvent(ctx,c.local,c.name,false);
    if(c.pending) allocator.free(allocator.ctx,c.pending,c.pendingSize);
    for(auto &i:c.localIntent) if(i.data) allocator.free(allocator.ctx,i.data,i.size);
    memset(&c,0,sizeof(c));
  }
  int queue(uint16_t cmd,uint16_t p1,uint32_t p2,const uint8_t *extra=nullptr,uint32_t n=0) {
    if(n>88) return Range;
    if(qCount==32) return Again;
    auto &p=control[qWrite]; memset(p.bytes,0,sizeof(p.bytes));
    put16(p.bytes,cmd); put16(p.bytes+2,p1); put32(p.bytes+4,p2);
    if(n) memcpy(p.bytes+8,extra,n);
    p.size=8+align8(n); qWrite=(qWrite+1)%32; qCount++; return Ok;
  }
  int requestRead() {
    if(notifySent) return Again;
    uint8_t hdr[8]={}; put16(hdr,ReadNotif);
    int e=tx.push(hdr,8,nullptr,0,true);
    if(!e) { notifySent=true; readNotifications++; kick(ctx); }
    return e?e:Again;
  }
  int flush() {
    while(qCount) {
      auto &p=control[qRead]; int e=tx.push(p.bytes,p.size);
      if(e==Again) return requestRead();
      if(e) return e;
      notifySent=false; qRead=(qRead+1)%32; qCount--; kick(ctx);
    }
    return Ok;
  }
  int prepare(Channel &c) {
    if(c.announced||!c.openAck||!c.remoteOpen) return Ok;
    if(intentless) {
      auto &li=c.localIntent[0]; li.size=MaxPacket;
      li.data=static_cast<uint8_t *>(allocator.alloc(allocator.ctx,MaxPacket));
      if(!li.data) return NoMemory;
      li.advertised=true; c.announced=true;
      if(channelEvent) channelEvent(ctx,c.local,c.name,true);
      return Ok;
    }
    uint8_t pairs[80]={};
    // Counts/sizes from the guamp IPCRTR DT, rather than a PCIe assumption.
    for(uint32_t i=0;i<MaxLocal;i++) {
      uint32_t size=i<5?0x800:(i<8?0x2000:0x4400);
      auto &li=c.localIntent[i];
      if(!li.data) {
        li.data=static_cast<uint8_t *>(allocator.alloc(allocator.ctx,size));
        if(!li.data) return NoMemory;
        li.size=size;
      }
      put32(pairs+i*8,size); put32(pairs+i*8+4,i+1); li.advertised=true;
    }
    int e=queue(Intent,c.local,MaxLocal,pairs,sizeof(pairs)); if(e) return e;
    c.announced=true;
    if(channelEvent) channelEvent(ctx,c.local,c.name,true);
    return Ok;
  }
  int sendFragment(Channel &c) {
    uint32_t free=0; int e=tx.count(&free); if(e) return e;
    if(free<=16) return requestRead();
    uint32_t n=c.pendingSize-c.pendingAt;
    if(n>8192) n=8192;
    uint32_t capacity=(free-16)&~7u;
    if(n>capacity) n=capacity;
    if(!n) return requestRead();
    uint8_t hdr[16]={}; put16(hdr,c.pendingAt?DataCont:Data);
    put16(hdr+2,c.local); put32(hdr+4,c.pendingIntent);
    put32(hdr+8,n); put32(hdr+12,c.pendingSize-c.pendingAt-n);
    e=tx.push(hdr,16,c.pending+c.pendingAt,n);
    if(e) return e;
    notifySent=false; kick(ctx); c.pendingAt+=n;
    if(c.pendingAt==c.pendingSize) {
      allocator.free(allocator.ctx,c.pending,c.pendingSize);
      c.pending=nullptr; c.pendingSize=c.pendingAt=c.pendingIntent=0; txPackets++;
    }
    return Ok;
  }
  // Diagnostics of the last refused RX command: its header, FIFO state and the rule that refused it.
  void refuse(const char *why,uint16_t cmd,uint16_t p1,uint32_t p2,uint32_t bytes,uint32_t avail) {
    lastRefusal=why; refusedCmd=cmd; refusedP1=p1; refusedP2=p2; refusedBytes=bytes; refusedAvail=avail;
    refusedTail=rx.mem.r32(rx.tail); refusedHead=rx.mem.r32(rx.head);
  }
  int receiveOne() {
    uint32_t available=0; int e=rx.count(&available);
    if(e) { refuse("rx fifo indices",0xffff,0,0,0,0); return e; }
    if(available<8) return Again;
    uint8_t hdr[16]={}; rx.peek(0,hdr,8);
    uint16_t cmd=le16(hdr),p1=le16(hdr+2); uint32_t p2=le32(hdr+4),bytes=8;
    if(cmd==Open) {
      uint32_t n=p2&0xffffu; if(!n||n>32) { refuse("open name length",cmd,p1,p2,0,available); return Protocol; }
      bytes+=align8(n);
    } else if(cmd==Intent) {
      if(!p2||p2>MaxRemote) { refuse("intent count",cmd,p1,p2,0,available); return Protocol; }
      bytes+=p2*8;
    } else if(cmd==Data||cmd==DataCont) {
      if(available<16) return Again;
      rx.peek(0,hdr,16); uint32_t chunk=le32(hdr+8);
      if(!chunk||chunk>MaxPacket||le32(hdr+12)>MaxPacket-chunk) { refuse("data chunk size",cmd,p1,p2,chunk,available); return Protocol; }
      bytes=16+align8(chunk);
    } else if(cmd>Signals||cmd==10) { refuse("unknown command",cmd,p1,p2,0,available); return Unsupported; }
    if(bytes>rx.length-8) { refuse("command larger than the fifo",cmd,p1,p2,bytes,available); return Protocol; }
    if(available<bytes) return Again;
    rxCommands++;
    Channel *c=remote(p1);
    // Tolerant like qcom_glink_native.c and the working ADSP edge (GuampGlinkEdge.inc): commands for channels without
    // a local user are consumed and counted, never fatal. Only DATA for an unknown channel or a malformed frame fails.
    switch(cmd) {
      case Version:
        if(!p1) break;                       // version 0: ignored (qcom_glink_receive_version)
        if(p1==1) features&=p2;
        e=queue(VersionAck,1,features);
        negotiated=true;                     // the remote's VERSION is an agreement as well
        break;
      case VersionAck:
        if(!p1) { refuse("version-ack 0 (remote refused)",cmd,p1,p2,bytes,available); return Unsupported; }
        if(p1==1&&p2==features) { negotiated=true; break; }
        if(p1==1) features&=p2;
        e=queue(Version,1,features);
        break;
      case Open: {
        if(!p1) { refuse("open rcid 0",cmd,p1,p2,bytes,available); return Protocol; }
        char name[33]={}; uint32_t n=p2&0xffffu;
        rx.peek(8,reinterpret_cast<uint8_t *>(name),n);
        if(name[n-1]||!name[0]) { refuse("open name not terminated",cmd,p1,p2,bytes,available); return Protocol; }
        if(c) { refuse("open of an rcid already open",cmd,p1,p2,bytes,available); return Protocol; }
        if(wants&&!wants(ctx,name)) {        // no local user: no ACK, as a Linux channel without a driver
          ignoredOpens++; memcpy(lastIgnored,name,n<32?n:31); lastIgnored[31]=0; break;
        }
        for(auto &x:channels) if(x.local&&nameEquals(x.name,name,32)) c=&x;
        if(!c) c=create(name,n-1);
        if(!c) { refuse("open: no local slot",cmd,p1,p2,bytes,available); e=NoMemory; break; }
        if(c->remote) { refuse("open: local channel has another rcid",cmd,p1,c->remote,bytes,available); return Protocol; }
        c->remote=p1; c->remoteOpen=true;
        // qcom_glink_create_remote: OPEN_ACK first, then our OPEN (unless we opened first).
        e=queue(OpenAck,p1,0);
        if(!e&&!c->openSent) { e=queue(Open,c->local,n,reinterpret_cast<const uint8_t *>(name),n); if(!e) c->openSent=true; }
        if(!e) e=prepare(*c);
        break;
      }
      case OpenAck:
        c=local(p1);
        if(!c||!c->openSent||c->openAck) { refuse("open-ack for an lcid not waiting",cmd,p1,p2,bytes,available); return Protocol; }
        c->openAck=true; e=prepare(*c); break;
      case Intent: {
        if(intentless||!c) { ignoredCommands++; break; }
        // Intents may precede the remote's OPEN_ACK of our OPEN (the remote side is open once it got our ACK).
        uint8_t pair[8];
        for(uint32_t i=0;i<p2;i++) {
          rx.peek(8+8*i,pair,8); uint32_t size=le32(pair),id=le32(pair+4);
          if(!size||!id) { ignoredCommands++; continue; }
          RemoteIntent *slot=nullptr;
          for(auto &ri:c->remoteIntent) if(ri.id==id) slot=&ri;
          for(auto &ri:c->remoteIntent) if(!slot&&!ri.id) slot=&ri;
          if(!slot) { droppedIntents++; continue; }
          if(slot->used&&slot->id==id) continue;   // in flight: keep it until its RX_DONE
          *slot={id,size,false};
        }
        c->requestPending=false; break;
      }
      case IntentReq: {
        if(!c||!c->announced) { ignoredCommands++; break; }
        bool granted=false;
        for(auto &li:c->localIntent) if(li.advertised&&!li.active&&li.size>=p2) granted=true;
        // Pre-advertised reusable intents suffice for bounded QMI packets.
        e=queue(IntentReqAck,c->local,granted?1u:0u); break;
      }
      case IntentReqAck:
        if(c&&!p2) c->requestPending=false;
        break;
      case RxDone: case RxDoneReuse: {
        if(intentless||!c) { ignoredCommands++; break; }
        RemoteIntent *ri=nullptr;
        for(auto &r:c->remoteIntent) if(r.id&&r.id==p2) ri=&r;
        if(!ri||!ri->used||(c->pending&&c->pendingIntent==p2)) { ignoredCommands++; break; }
        if(cmd==RxDoneReuse) ri->used=false; else *ri={};
        break;
      }
      case Data: case DataCont: {
        if(!c||!c->announced||(!intentless&&(p2<1||p2>MaxLocal))) { refuse("data for a channel/intent not ready",cmd,p1,p2,bytes,available); return Protocol; }
        auto &li=c->localIntent[intentless?0:p2-1];
        uint32_t chunk=le32(hdr+8),left=le32(hdr+12);
        if(!li.advertised||!li.data||(cmd==Data&&li.active)||(cmd==DataCont&&!li.active)) { refuse("data intent state",cmd,p1,p2,bytes,available); return Protocol; }
        if(chunk>li.size-li.used||left>li.size-li.used-chunk) { refuse("data larger than the intent",cmd,p1,p2,chunk+left,available); return Range; }
        if(li.active&&li.remaining!=chunk+left) { refuse("data continuation length",cmd,p1,p2,chunk+left,available); return Protocol; }
        rx.peek(16,li.data+li.used,chunk); li.used+=chunk; li.remaining=left; li.active=true;
        if(!left) {
          received(ctx,c->local,li.data,li.used); rxPackets++;
          if(!running) return error;
          bool reuse=(features&1)!=0;
          // Protocol RX_DONE carries our local channel id (see __qcom_glink_rx_done).
          if(!intentless) e=queue(reuse?RxDoneReuse:RxDone,c->local,p2);
          li.active=false; li.used=li.remaining=0;
          if(!intentless&&!reuse&&!e) {
            uint8_t pair[8]; put32(pair,li.size); put32(pair+4,p2);
            e=queue(Intent,c->local,1,pair,8);
          }
        }
        break;
      }
      case ReadNotif: kick(ctx); break;
      case Signals: break;
      case Close:
        e=queue(CloseAck,p1,0); if(c) release(*c); break;
      case CloseAck:
        c=local(p1); if(c) release(*c); break;
      default: ignoredCommands++; break;
    }
    if(e) { if(!lastRefusal) refuse("reply queue",cmd,p1,p2,bytes,available); return e; }
    return rx.advance(bytes);
  }
};
} // namespace guampwifi
#endif
