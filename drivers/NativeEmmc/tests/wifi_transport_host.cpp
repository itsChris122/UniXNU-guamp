// Actual transport source exercised with an independent wire peer and bounded
// shared-memory fixture; not a radio simulation and not hardware validation.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <memory>
#include <random>
#include <string>
#include "../GuampSmem.inc"
#include "../GuampSmp2p.inc"
#include "../GuampGlink.inc"
#include "../GuampQrtr.inc"
#include "../GuampWlfw.inc"
#include "../GuampSnocCe.inc"
#include "../GuampWifiRpm.inc"
#include "../GuampWifiPower.inc"
#include "../GuampScm.inc"
#include "../GuampMsa.inc"
#include "../GuampQftp.inc"
#include "../GuampServreg.inc"
#include "../GuampRmtfs.inc"
#include "../GuampWifiControl.inc"
using namespace guampwifi;
static unsigned checks=0;
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)
struct Fixture {
  std::vector<uint8_t> bytes;
  bool held=false,deny=false; uint32_t locks=0,fences=0,kicks=0;
  size_t live=0; int failAllocation=-1;
  explicit Fixture(uint32_t n=2*1024*1024):bytes(n) {}
  static uint32_t read(void *p,uint32_t at) {
    auto &f=*static_cast<Fixture *>(p); CHECK(!(at&3)&&at<=f.bytes.size()-4); return le32(f.bytes.data()+at);
  }
  static void write(void *p,uint32_t at,uint32_t value) {
    auto &f=*static_cast<Fixture *>(p); CHECK(!(at&3)&&at<=f.bytes.size()-4); put32(f.bytes.data()+at,value);
  }
  static void barrier(void *p) { static_cast<Fixture *>(p)->fences++; }
  static bool lock(void *p) {
    auto &f=*static_cast<Fixture *>(p); CHECK(!f.held); if(f.deny) return false;
    f.held=true; f.locks++; return true;
  }
  static void unlock(void *p) { auto &f=*static_cast<Fixture *>(p); CHECK(f.held); f.held=false; }
  static void *alloc(void *p,uint32_t n) {
    auto &f=*static_cast<Fixture *>(p);
    if(f.failAllocation==0) return nullptr;
    if(f.failAllocation>0) f.failAllocation--;
    void *v=calloc(1,n); if(v) f.live+=n; return v;
  }
  static void free(void *p,void *v,uint32_t n) { auto &f=*static_cast<Fixture *>(p); CHECK(f.live>=n); f.live-=n; ::free(v); }
  static void kick(void *p) { static_cast<Fixture *>(p)->kicks++; }
  Memory memory() { return {this,uint32_t(bytes.size()),read,write,barrier}; }
  Allocator allocator() { return {this,alloc,free}; }
  SmemArena arena() { return {memory(),this,lock,unlock}; }
  void w(uint32_t off,uint32_t v) { put32(bytes.data()+off,v); }
  void init(bool partitioned) {
    w(64+7*4,(partitioned?12u:11u)<<16); w(192,1); w(196,0x3000); w(200,uint32_t(bytes.size())-0x4000);
    if(partitioned) {
      uint32_t t=uint32_t(bytes.size())-4096; w(t,0x434f5424); w(t+4,1); w(t+8,1);
      w(t+32,0x10000); w(t+36,0x40000); w(t+44,1u<<16); w(t+48,64);
      w(0x10000,0x54525024); w(0x10004,1u<<16); w(0x10008,0x40000); w(0x1000c,32); w(0x10010,0x40000);
    }
  }
};
static void smemTest() {
  for(bool partitioned:{false,true}) {
    Fixture f; f.init(partitioned); auto a=f.arena(); uint32_t off=0,size=0; bool created=false;
    CHECK(a.allocate(1,435,340,&off,&created)==Ok&&created);
    CHECK(a.find(1,435,&off,&size)==Ok&&size==(partitioned?340u:344u));
    uint32_t saved=off; CHECK(a.allocate(1,435,340,&off,&created)==Ok&&!created&&off==saved);
    CHECK(a.allocate(1,435,349,&off)==Range);
    // Wi-Fi R1 -102: an existing item is reused when large enough (Linux -EEXIST + qcom_smem_get), with its own size
    static std::vector<SmemArena::Note> seen; seen.clear();
    a.note=[](void *,const SmemArena::Note &n) { seen.push_back(n); };
    uint32_t actual=0; CHECK(a.allocate(1,435,300,&off,&created,&actual)==Ok&&!created&&off==saved&&actual==(partitioned?340u:344u));
    CHECK(seen.size()==1&&seen[0].item==435&&seen[0].wanted==300&&seen[0].found==actual&&seen[0].result==Ok);
    CHECK(a.allocate(1,435,349,&off,&created,&actual)==Range&&seen.size()==2&&seen[1].result==Range&&seen[1].wanted==349);
    CHECK(a.allocate(1,435,340,&off)==Ok&&seen.size()==2);   // same size (v11: padded 344): no note
    a.note=nullptr;
    uint32_t before=f.locks; CHECK(a.allocate(1,7,16,&off)==Invalid&&f.locks==before);
    f.deny=true; auto bytes=f.bytes; CHECK(a.allocate(1,478,32,&off)==Timeout&&f.bytes==bytes); f.deny=false;
    CHECK(a.allocate(1,479,16384,&off)==Ok);
    for(uint32_t j=0;j<16384;j++) CHECK(f.bytes[off+j]==0);
    CHECK(!f.held);
    if(partitioned) {
      f.w(0x10020,0); auto broken=f.bytes;
      CHECK(a.allocate(1,480,16384,&off)==Protocol&&broken==f.bytes);
    }
  }
}
static void smpTest() {
  Fixture f; f.init(true); auto a=f.arena(); Smp2pEdge e;
  CHECK(e.start(&a,Fixture::kick,&f)==Ok);
  CHECK(e.poll()==Again&&!e.ready());
  uint32_t in=0; CHECK(a.allocate(1,428,340,&in)==Ok);
  f.w(in,0x504d5324); f.w(in+4,0x101); f.w(in+8,1); f.w(in+12,16u|(2u<<16));
  memcpy(f.bytes.data()+in+20,"slave-kernel",13); f.w(in+36,2);
  memcpy(f.bytes.data()+in+40,"wlan",5); f.w(in+56,0);
  CHECK(e.poll()==Ok&&e.ready()&&e.negotiated);
  CHECK(e.update(1u<<3,1u<<3)==Ok&&le32(f.bytes.data()+e.outbound+36)==8);
  CHECK(e.update(1u<<3,0)==Ok&&le32(f.bytes.data()+e.outbound+36)==0);
  f.w(in+16,1); CHECK(e.poll()==PeerReset&&le32(f.bytes.data()+e.outbound+16)==2);
  CHECK(e.poll()==Ok);
  f.w(in+56,1); CHECK(e.poll()==PeerReset);
  f.w(in+56,0); f.w(in+36,0); CHECK(e.poll()==PeerReset);
}
static std::vector<uint8_t> frame(uint16_t cmd,uint16_t p1,uint32_t p2,const std::vector<uint8_t> &data={}) {
  std::vector<uint8_t> p(8+align8(uint32_t(data.size()))); put16(p.data(),cmd); put16(p.data()+2,p1); put32(p.data()+4,p2);
  if(!data.empty()) memcpy(p.data()+8,data.data(),data.size()); return p;
}
struct Peer {
  Fixture f; std::unique_ptr<Glink> link=std::make_unique<Glink>(); GlinkPipe toHost,fromHost;
  uint32_t events=0; std::vector<std::vector<uint8_t>> delivered;
  static void kick(void *p) { static_cast<Peer *>(p)->f.kicks++; }
  static void event(void *p,uint16_t id,const char *name,bool up) {
    auto &a=*static_cast<Peer *>(p); CHECK(id==1&&strcmp(name,"IPCRTR")==0); if(up) a.events++;
  }
  static void rx(void *p,uint16_t id,const uint8_t *v,uint32_t n) {
    auto &a=*static_cast<Peer *>(p); CHECK(id==1); a.delivered.emplace_back(v,v+n);
  }
  explicit Peer(uint32_t length=16384,bool intentless=false) {
    auto m=f.memory(); GlinkPipe out={m,0x10000,length,4,0,true},in={m,0x20000,length,12,8,false};
    toHost={m,0x20000,length,12,8,true}; fromHost={m,0x10000,length,4,0,false};
    CHECK(link->start(out,in,f.allocator(),this,kick,event,rx,intentless)==Ok);
  }
  void inject(const std::vector<uint8_t> &p) { CHECK(toHost.push(p.data(),uint32_t(p.size()))==Ok); }
  std::vector<std::vector<uint8_t>> drain() {
    uint32_t n=0; CHECK(fromHost.count(&n)==Ok); std::vector<std::vector<uint8_t>> result;
    while(n) {
      uint8_t hdr[16]={}; CHECK(n>=8); fromHost.peek(0,hdr,8);
      uint16_t cmd=le16(hdr); uint32_t p2=le32(hdr+4),len=8;
      if(cmd==2) len+=align8(p2&0xffffu);
      if(cmd==5) len+=p2*8;
      if(cmd==9||cmd==12) { CHECK(n>=16); fromHost.peek(0,hdr,16); len=16+align8(le32(hdr+8)); }
      CHECK(len<=n); std::vector<uint8_t> p(len); fromHost.peek(0,p.data(),len);
      CHECK(fromHost.advance(len)==Ok); result.push_back(p); CHECK(fromHost.count(&n)==Ok);
    }
    return result;
  }
  void connected() {
    uint32_t features=link->intentless?0u:1u;
    CHECK(link->step()==Ok); auto p=drain(); CHECK(p.size()==1&&le16(p[0].data())==0&&le32(p[0].data()+4)==features);
    inject(frame(0,1,features)); inject(frame(1,1,features)); CHECK(link->step()==Ok); drain(); CHECK(link->negotiated);
    uint16_t id=0; CHECK(link->open("IPCRTR",&id)==Ok&&id==1); CHECK(link->step()==Ok); drain();
    // Deliberately different local (1) / remote (37) ids. A symmetric peer test hides this bug.
    inject(frame(2,37,7,{'I','P','C','R','T','R',0})); inject(frame(4,1,0));
    CHECK(link->step()==Ok); p=drain(); CHECK(link->online(1)&&events==1);
    bool ack=false,intent=false;
    for(const auto &c:p) {
      if(le16(c.data())==4) { CHECK(le16(c.data()+2)==37); ack=true; }
      if(le16(c.data())==5) { CHECK(le16(c.data()+2)==1&&le32(c.data()+4)==10); intent=true; }
    }
    CHECK(ack&&(intent||link->intentless));
  }
  void remoteIntent(uint32_t id,uint32_t size) {
    std::vector<uint8_t> b(8); put32(b.data(),size); put32(b.data()+4,id);
    inject(frame(5,37,1,b)); CHECK(link->step()==Ok); drain();
  }
  void data(uint32_t id,const std::vector<uint8_t> &v,uint32_t left,bool cont=false) {
    std::vector<uint8_t> b(8+v.size()); put32(b.data(),uint32_t(v.size())); put32(b.data()+4,left);
    memcpy(b.data()+8,v.data(),v.size()); inject(frame(cont?12:9,37,id,b));
  }
  ~Peer() { link->stop(); CHECK(f.live==0); }
};
static void glinkTest() {
  Peer p; p.connected(); p.remoteIntent(91,0x4400);
  std::vector<uint8_t> payload(15003); for(size_t i=0;i<payload.size();i++) payload[i]=uint8_t(i*41);
  CHECK(p.link->send(1,payload.data(),uint32_t(payload.size()))==Ok);
  CHECK(p.link->send(1,payload.data(),1)==Again);
  std::vector<uint8_t> sent;
  for(unsigned i=0;i<8&&sent.size()<payload.size();i++) {
    CHECK(p.link->step()==Ok);
    for(const auto &c:p.drain()) if(le16(c.data())==9||le16(c.data())==12) {
      CHECK(le16(c.data()+2)==1&&le32(c.data()+4)==91);
      uint32_t n=le32(c.data()+8); sent.insert(sent.end(),c.begin()+16,c.begin()+16+n);
      CHECK(le32(c.data()+12)==payload.size()-sent.size());
    }
  }
  CHECK(sent==payload);
  CHECK(p.link->send(1,payload.data(),20)==Again); // intent stays busy until remote RX_DONE
  CHECK(p.link->step()==Ok); p.drain();
  p.inject(frame(14,37,91)); CHECK(p.link->step()==Ok); p.drain();
  CHECK(p.link->send(1,payload.data(),20)==Ok); CHECK(p.link->step()==Ok); p.drain();
  p.data(9,std::vector<uint8_t>(payload.begin(),payload.begin()+8000),7003);
  CHECK(p.link->step()==Ok); CHECK(p.delivered.empty()); p.drain();
  p.data(9,std::vector<uint8_t>(payload.begin()+8000,payload.end()),0,true);
  CHECK(p.link->step()==Ok&&p.delivered.size()==1&&p.delivered[0]==payload);
  bool done=false; for(const auto &c:p.drain()) if(le16(c.data())==14) { CHECK(le16(c.data()+2)==1&&le32(c.data()+4)==9); done=true; }
  CHECK(done);
  // Same reusable local intent can receive another packet.
  p.data(9,{1,2,3},0); CHECK(p.link->step()==Ok&&p.delivered.size()==2); p.drain();
  // Malformed continuation is rejected without delivering partial data.
  p.data(9,{4},0,true); CHECK(p.link->step()==Protocol&&!p.link->running);
  Peer bad; bad.connected(); bad.data(11,{1},0); CHECK(bad.link->step()==Protocol);
  Peer index; index.f.w(12,16384); CHECK(index.link->step()==Protocol);
  Peer exhausted; exhausted.f.failAllocation=0;
  CHECK(exhausted.link->step()==Ok); exhausted.drain();
  exhausted.inject(frame(1,1,1)); CHECK(exhausted.link->step()==Ok);
  CHECK(exhausted.link->open("IPCRTR")==Ok); CHECK(exhausted.link->step()==Ok); exhausted.drain();
  exhausted.inject(frame(2,37,7,{'I','P','C','R','T','R',0})); exhausted.inject(frame(4,1,0));
  CHECK(exhausted.link->step()==NoMemory&&!exhausted.link->running);
  // A READ_NOTIF may consume only the extra eight reserved bytes. The final
  // eight bytes must remain free so full and empty indices cannot coincide.
  Fixture small(256); GlinkPipe t={small.memory(),64,64,4,0,true}; uint8_t fill[48]={};
  CHECK(t.push(fill,48)==Ok); uint8_t note[8]={13}; CHECK(t.push(note,8,nullptr,0,true)==Ok);
  auto saved=small.bytes; CHECK(t.push(note,8,nullptr,0,true)==Again&&small.bytes==saved);
  Peer intentless(16384,true); intentless.connected(); intentless.data(0x1234,{3,2,1},0);
  CHECK(intentless.link->step()==Ok&&intentless.delivered.size()==1);
  CHECK(intentless.drain().empty()); // RPM does not send RX_DONE or intents
  const uint8_t bytes[3]={4,5,6}; CHECK(intentless.link->send(1,bytes,3)==Ok);
  CHECK(intentless.link->step()==Ok); auto ip=intentless.drain(); CHECK(ip.size()==1&&le16(ip[0].data())==9);
}
static void rpmTest() {
  Fixture f(0x7000); uint32_t table=0x6f00;
  f.w(table,0x67727430); f.w(table+4,2);
  f.w(table+8,0x61703272); f.w(table+12,0x100); f.w(table+16,1024);
  f.w(table+20,0x72326170); f.w(table+24,0x1000); f.w(table+28,1024);
  GlinkPipe tx={},rx={}; CHECK(Rpm::pipes(f.memory(),&tx,&rx)==Ok);
  CHECK(tx.fifo==0x108&&tx.head==0x104&&tx.tail==0x100&&tx.tx);
  CHECK(rx.fifo==0x1008&&rx.head==0x1004&&rx.tail==0x1000&&!rx.tx);
  f.w(table+24,0x100); CHECK(Rpm::pipes(f.memory(),&tx,&rx)==Range); f.w(table+24,0x1000);
  auto r=std::make_unique<Rpm>(); CHECK(r->start(f.memory(),f.allocator(),&f,Fixture::kick,123)==Ok);
  // Test the RPM codec after the independent GLINK negotiation test above.
  r->channel=1; r->glink.channels[0].local=1; r->glink.channels[0].announced=true;
  CHECK(r->vote(0x616f646c,16,0x6e657773,1,0,100)==Ok);
  CHECK(le32(r->request)==0x716572&&le32(r->request+4)==32&&le32(r->request+8)==123);
  CHECK(le32(r->request+16)==0x616f646c&&le32(r->request+20)==16&&le32(r->request+24)==12);
  CHECK(le32(r->request+28)==0x6e657773&&le32(r->request+32)==4&&le32(r->request+36)==1);
  CHECK(r->step(100)==Ok&&r->queued); CHECK(r->step(101)==Ok);
  uint8_t ack[20]={}; put32(ack,0x716572); put32(ack+4,12); put32(ack+8,0x2367736d); put32(ack+12,4); put32(ack+16,122);
  CHECK(r->receive(ack,20)==NotFound&&r->waiting); put32(ack+16,123);
  CHECK(r->receive(ack,20)==Ok&&r->done&&!r->waiting&&!r->error); r->consume();
  CHECK(r->vote(0x616f646c,16,0x6e657773,1,0,100)==Ok);
  CHECK(r->step(5100)==Timeout&&r->done); r->consume(); r->stop(); CHECK(f.live==0);
}
struct MsaMonitor {
  unsigned queries=0,calls=0;
  scm::Registers last={}; bool reject=false;
  static void call(void *ctx,const scm::Registers &in,scm::Registers &out) {
    auto &m=*static_cast<MsaMonitor *>(ctx); out={};
    if((in.x[0]&0xffff)==0x601) { CHECK(in.x[2]==0x02000c16); out.x[1]=1; m.queries++; return; }
    m.calls++; m.last=in; if(m.reject) out.x[1]=1;
  }
};
static void msaTest() {
  for(bool wide:{true,false}) for(bool secure:{true,false}) {
    Fixture f(512); MsaMonitor monitor;
    scm::Transport t={MsaMonitor::call,nullptr,&monitor,wide};
    MsaAssignment m={f.memory(),0x60000000,&t,0};
    CHECK(m.assign(0x51900000,0x100000,secure)==Ok&&monitor.queries==1&&monitor.calls==1&&m.assigned==1);
    CHECK(monitor.last.x[0]==(wide?0x42000c16u:0x02000c16u)&&monitor.last.x[1]==0x1117);
    CHECK(monitor.last.x[2]==0x60000000&&monitor.last.x[3]==16&&monitor.last.x[4]==0x60000040&&monitor.last.x[5]==0x60000100);
    CHECK(le64(f.bytes.data())==0x51900000&&le64(f.bytes.data()+8)==0x100000);
    CHECK(le32(f.bytes.data()+64)==3&&le32(f.bytes.data()+128)==15&&le32(f.bytes.data()+132)==6);
    CHECK(le32(f.bytes.data()+152)==24&&le32(f.bytes.data()+156)==6);
    if(!secure) CHECK(le32(f.bytes.data()+176)==25&&le32(f.bytes.data()+180)==6);
    uint32_t n=secure?48u:72u;
    if(wide) CHECK(le64(f.bytes.data()+256)==4&&le64(f.bytes.data()+264)==0x60000080&&le64(f.bytes.data()+272)==n);
    else CHECK(le32(f.bytes.data()+256)==4&&le32(f.bytes.data()+260)==0x60000080&&le32(f.bytes.data()+264)==n);
    auto original=f.bytes;
    CHECK(m.assign(0x4ab00000,4096,false)==Range&&f.bytes==original&&monitor.calls==1);
    CHECK(m.assign(0x51900001,4096,false)==Range&&monitor.calls==1);
    monitor.reject=true; CHECK(m.assign(0x51900000,4096,false)==RemoteError&&m.assigned==1);
  }
  scm::Transport t; auto invalid=t.callExtended(12,22,7,0,0,0,1); CHECK(invalid.transport==scm::kInvalid);
  // rmtfs shared region (msm_sharedmem setup_shared_ram_perms, vm-nav-path): HLOS -> HLOS/MSS_MSA/NAV RW
  for(bool wide:{true,false}) {
    Fixture f(512); MsaMonitor monitor; scm::Transport ts={MsaMonitor::call,nullptr,&monitor,wide};
    MsaAssignment m={f.memory(),0x60000000,&ts,0};
    static const uint32_t vm[3]={3,15,0x2b};
    CHECK(m.assignShared(0x7f001000,0x200000,vm,3)==Ok&&monitor.calls==1&&monitor.last.x[1]==0x1117);
    CHECK(le64(f.bytes.data())==0x7f001000&&le64(f.bytes.data()+8)==0x200000&&le32(f.bytes.data()+64)==3);
    CHECK(le32(f.bytes.data()+128)==3&&le32(f.bytes.data()+132)==6&&le32(f.bytes.data()+152)==15&&
          le32(f.bytes.data()+156)==6&&le32(f.bytes.data()+176)==0x2b&&le32(f.bytes.data()+180)==6);
    if(wide) CHECK(le64(f.bytes.data()+272)==72); else CHECK(le32(f.bytes.data()+264)==72);
    CHECK(m.assignShared(0x51900000,4096,vm,3)==Range&&m.assignShared(0x7f001800,4096,vm,3)==Range&&
          m.assignShared(0x100000000ull,4096,vm,3)==Range&&m.assignShared(0x7f001000,4096,vm,0)==Range&&monitor.calls==1);
  }
}
static void powerPlanTest() {
  auto r=std::make_unique<Rpm>(); r->channel=1; r->glink.running=true;
  r->glink.channels[0].local=1; r->glink.channels[0].announced=true;
  WifiPower power; CHECK(power.begin(r.get(),true)==NotReady);
  CHECK(power.begin(r.get(),false)==Ok&&power.count==4&&power.busy&&!power.proxyHeld);
  CHECK(power.plan[0].type==0x306b6c63&&power.plan[0].key==0x62616e45&&power.plan[0].value==1);
  CHECK(power.plan[2].type==0x78637772&&power.plan[2].key==0x6c766c76&&power.plan[2].value==384);
  CHECK(power.plan[0].state==0&&power.plan[1].state==1);
  power.busy=false; power.proxyHeld=true;
  CHECK(power.begin(r.get(),true)==Ok&&power.count==16&&!power.wlanHeld);
  // Guamp DT overrides ICNSS's default CH0 value with a 3.3 V ceiling.
  uint32_t rails[4]={8,16,17,23},uv[4]={640000,1800000,1304000,3300000};
  for(uint32_t i=0;i<4;i++) {
    CHECK(power.plan[i*4].type==0x616f646c&&power.plan[i*4].id==rails[i]&&power.plan[i*4].key==0x7675&&power.plan[i*4].value==uv[i]);
    CHECK(power.plan[i*4+2].key==0x6e657773&&power.plan[i*4+2].value==1);
  }
  CHECK(power.releaseWlan(true)==Busy); power.busy=false; CHECK(power.releaseWlan(false)==Busy);
  r->glink.running=false;
}
struct CePeer {
  Fixture mmio=Fixture(0x300000),src=Fixture(256),dst=Fixture(256);
  SnocCe ce; SnocCe::Slot source[8]={},destination[8]={};
  uint32_t sent=0,received=0; uint64_t lastTx=0,lastRx=0;
  static void tx(void *p,const SnocCe::Buffer &b) { auto &f=*static_cast<CePeer *>(p); f.sent++; f.lastTx=b.iova; }
  static void rx(void *p,const SnocCe::Buffer &b,uint32_t n) { auto &f=*static_cast<CePeer *>(p); CHECK(n==73); f.received++; f.lastRx=b.iova; }
  static void regWrite(void *p,uint32_t at,uint32_t value) {
    auto &f=*static_cast<Fixture *>(p);
    if(at==0x243030||at==0x243038) value=le32(f.bytes.data()+at)&~value; // W1C status registers
    Fixture::write(p,at,value);
  }
  SnocCe::Ring sr() { return {src.memory(),0xa0000000,8,0,0,0,source}; }
  SnocCe::Ring dr() { return {dst.memory(),0xa0001000,8,0,0,0,destination}; }
  int start(bool quiesced=true) { auto m=mmio.memory(); m.write=regWrite; return ce.start(m,3,sr(),dr(),2048,quiesced,this,tx,rx); }
};
static void ceTest() {
  auto f=std::make_unique<CePeer>(); CHECK(f->start(false)==NotReady); CHECK(f->start()==Ok);
  CHECK(le32(f->mmio.bytes.data()+0x243000)==0xa0000000);
  CHECK(le32(f->mmio.bytes.data()+0x243008)==8);
  CHECK(le32(f->mmio.bytes.data()+0x24300c)==0xa0001000);
  CHECK(le32(f->mmio.bytes.data()+0x243014)==8);
  CHECK(!SnocCe::dmaRange(0x51900000,4096)); // firmware MSA is not a mapped host DMA buffer
  CHECK(!SnocCe::dmaRange(0x100000000,4096));
  CHECK(!SnocCe::dmaRange(0xaffffff0,32));
  SnocCe::Buffer b={0xa0100000,100,reinterpret_cast<void *>(uintptr_t(1))};
  CHECK(f->ce.transmit(b,0xabc)==Ok);
  CHECK(le64(f->src.bytes.data())==0xa0100000);
  CHECK(le16(f->src.bytes.data()+8)==100&&le16(f->src.bytes.data()+10)==0xabc0);
  CHECK(le32(f->mmio.bytes.data()+0x24303c)==1);
  CHECK(f->ce.poll()==Ok&&f->sent==0);
  f->mmio.w(0x243044,1); CHECK(f->ce.poll()==Ok&&f->sent==1&&f->lastTx==b.iova);
  SnocCe::Buffer input={0xa0200000,2048,reinterpret_cast<void *>(uintptr_t(2))};
  CHECK(f->ce.postReceive(input)==Ok&&le32(f->dst.bytes.data()+8)==0);
  CHECK(f->ce.poll()==Ok&&f->received==0);
  f->dst.w(8,73); CHECK(f->ce.poll()==Ok&&f->received==1&&f->lastRx==input.iova);
  CHECK(le32(f->dst.bytes.data()+8)==0);
  // Wrap several times and prove no buffer is reclaimed before HW advances.
  for(uint32_t round=0;round<4;round++) {
    for(uint32_t i=0;i<7;i++) CHECK(f->ce.transmit(b)==Ok);
    CHECK(f->ce.transmit(b)==Again);
    uint32_t end=le32(f->mmio.bytes.data()+0x24303c); f->mmio.w(0x243044,end);
    CHECK(f->ce.poll()==Ok&&f->ce.source.outstanding==0);
  }
  CHECK(f->sent==29);
  CHECK(f->ce.release(false)==Busy&&f->ce.active);
  CHECK(f->ce.release(true)==Ok&&!f->ce.active);
  auto stale=std::make_unique<CePeer>(); stale->mmio.w(0x243044,1); CHECK(stale->start()==Busy);
  auto bad=std::make_unique<CePeer>(); CHECK(bad->start()==Ok);
  CHECK(bad->ce.postReceive(input)==Ok); bad->dst.w(8,3000); CHECK(bad->ce.poll()==Range&&bad->received==0);
}
struct RouterPeer {
  Fixture f; Qrtr qrtr; bool ready=false,busy=false; unsigned changes=0;
  unsigned localMessages=0; uint32_t localNode=0,localPort=0;
  std::vector<std::vector<uint8_t>> sent,received;
  static int tx(void *p,const uint8_t *v,uint32_t n) {
    auto &r=*static_cast<RouterPeer *>(p); if(r.busy) return Again; r.sent.emplace_back(v,v+n); return Ok;
  }
  static void event(void *p,bool up) { auto &r=*static_cast<RouterPeer *>(p); r.ready=up; r.changes++; }
  static void rx(void *p,const uint8_t *v,uint32_t n) { static_cast<RouterPeer *>(p)->received.emplace_back(v,v+n); }
  static void localRx(void *p,uint32_t node,uint32_t port,const uint8_t *v,uint32_t n) {
    auto &r=*static_cast<RouterPeer *>(p);r.localMessages++;r.localNode=node;r.localPort=port;r.received.emplace_back(v,v+n);
  }
  RouterPeer() { CHECK(qrtr.start(f.allocator(),this,tx,event,rx)==Ok); }
  ~RouterPeer() { qrtr.stop(); CHECK(f.live==0); }
};
static std::vector<uint8_t> qrtrPacket(uint32_t type,uint32_t sp,uint32_t dp,const std::vector<uint8_t> &body,bool confirm=false,bool v2=false) {
  uint32_t header=v2?16:32; std::vector<uint8_t> p(header+((body.size()+3)&~3u));
  if(v2) { p[0]=3; p[1]=uint8_t(type); p[2]=confirm; put32(p.data()+4,uint32_t(body.size()));
    put16(p.data()+8,7); put16(p.data()+10,uint16_t(sp==Qrtr::Ctrl?0xffffu:sp)); put16(p.data()+12,1); put16(p.data()+14,uint16_t(dp==Qrtr::Ctrl?0xffffu:dp));
  } else { put32(p.data(),1); put32(p.data()+4,type); put32(p.data()+8,7); put32(p.data()+12,sp);
    put32(p.data()+16,confirm?1u:0u); put32(p.data()+20,uint32_t(body.size())); put32(p.data()+24,1); put32(p.data()+28,dp); }
  if(!body.empty()) memcpy(p.data()+header,body.data(),body.size()); return p;
}
static std::vector<uint8_t> ctrl(uint32_t cmd,uint32_t a=0,uint32_t b=0,uint32_t c=0,uint32_t d=0) {
  std::vector<uint8_t> p(20); put32(p.data(),cmd); put32(p.data()+4,a); put32(p.data()+8,b); put32(p.data()+12,c); put32(p.data()+16,d); return p;
}
static void qrtrTest() {
  RouterPeer r; CHECK(r.qrtr.step()==Ok&&r.sent.size()==1);
  auto incoming=qrtrPacket(2,Qrtr::Ctrl,Qrtr::Ctrl,ctrl(2)); CHECK(r.qrtr.receive(incoming.data(),incoming.size())==Ok);
  // Independent OEM/Linux wire fact: version1 occupies the low byte. The
  // earlier transposed0x100 identifies version0/instance1 and must be ignored.
  incoming=qrtrPacket(4,Qrtr::Ctrl,Qrtr::Ctrl,ctrl(4,0x45,0x100,7,900));
  CHECK(r.qrtr.receive(incoming.data(),incoming.size())==Ok&&!r.ready);
  incoming=qrtrPacket(4,Qrtr::Ctrl,Qrtr::Ctrl,ctrl(4,0x45,1,7,900));
  CHECK(r.qrtr.receive(incoming.data(),incoming.size())==Ok&&r.ready&&r.qrtr.serverPort==900);
  CHECK(r.qrtr.receive(incoming.data(),incoming.size())==Ok&&r.changes==1);
  r.busy=true; CHECK(r.qrtr.step()==Again); r.busy=false; CHECK(r.qrtr.step()==Ok);
  CHECK(le32(r.sent.back().data()+32)==Qrtr::NewLookup&&le32(r.sent.back().data()+40)==1);
  for(unsigned i=0;i<10;i++) CHECK(r.qrtr.send(reinterpret_cast<const uint8_t *>("abc"),3)==Ok);
  CHECK(r.qrtr.send(reinterpret_cast<const uint8_t *>("abc"),3)==Again);
  CHECK(r.qrtr.step(16)==Ok); CHECK(r.sent.size()==12);
  CHECK(le32(r.sent[6].data()+16)==1); // fifth DATA requests resume confirmation
  incoming=qrtrPacket(7,Qrtr::Ctrl,Qrtr::LocalPort,ctrl(7,7,900));
  CHECK(r.qrtr.receive(incoming.data(),incoming.size())==Ok&&r.qrtr.flow==0);
  incoming=qrtrPacket(1,900,Qrtr::LocalPort,{2,3,4},true,true);
  CHECK(r.qrtr.receive(incoming.data(),incoming.size())==Ok&&r.received.size()==1);
  CHECK(r.qrtr.step()==Ok); CHECK(le32(r.sent.back().data()+4)==7);
  CHECK(le32(r.sent.back().data()+36)==1&&le32(r.sent.back().data()+40)==Qrtr::LocalPort);
  auto truncated=incoming; put32(truncated.data()+4,1000); CHECK(r.qrtr.receive(truncated.data(),truncated.size())==Protocol);
  incoming=qrtrPacket(5,Qrtr::Ctrl,Qrtr::Ctrl,ctrl(5,0x45,1,7,900));
  CHECK(r.qrtr.receive(incoming.data(),incoming.size())==Ok&&!r.ready&&r.qrtr.count==0);

  RouterPeer local;CHECK(local.qrtr.step()==Ok);
  CHECK(local.qrtr.addLocalService(4096,0,4096,&local,RouterPeer::localRx)==Ok);
  CHECK(local.qrtr.step()==Ok);const auto &announcement=local.sent.back();
  CHECK(le32(announcement.data()+4)==Qrtr::NewServer&&le32(announcement.data()+12)==Qrtr::Ctrl);
  CHECK(le32(announcement.data()+24)==Qrtr::Broadcast&&le32(announcement.data()+28)==Qrtr::Ctrl);
  CHECK(le32(announcement.data()+36)==4096&&le32(announcement.data()+40)==0&&
        le32(announcement.data()+44)==Qrtr::LocalNode&&le32(announcement.data()+48)==4096);
  CHECK(local.qrtr.addLocalService(4096,0,4096,&local,RouterPeer::localRx)==Ok&&local.qrtr.count==0);
  incoming=qrtrPacket(Qrtr::NewLookup,Qrtr::Ctrl,Qrtr::Ctrl,ctrl(Qrtr::NewLookup,4096));
  CHECK(local.qrtr.receive(incoming.data(),incoming.size())==Ok&&local.qrtr.step()==Ok);
  CHECK(le32(local.sent.back().data()+24)==7&&le32(local.sent.back().data()+36)==4096);
  incoming=qrtrPacket(Qrtr::Data,700,4096,{3,2,1},true);
  CHECK(local.qrtr.receive(incoming.data(),incoming.size())==Ok&&local.localMessages==1&&local.localNode==7&&local.localPort==700);
  CHECK(local.qrtr.step()==Ok&&le32(local.sent.back().data()+4)==Qrtr::Resume);
  CHECK(le32(local.sent.back().data()+36)==Qrtr::LocalNode&&le32(local.sent.back().data()+40)==4096);
  const uint8_t one=1;
  for(unsigned i=0;i<16;i++)CHECK(local.qrtr.sendFrom(4096,7,700,&one,1)==Ok);
  CHECK(local.qrtr.addLocalService(55,0,5000,&local,RouterPeer::localRx)==Again);
  bool silent=false;for(const auto &s:local.qrtr.local)if(s.used&&s.port==5000)silent=true;CHECK(!silent);
  CHECK(local.qrtr.step(16)==Ok);
  CHECK(local.qrtr.addLocalService(55,0,5000,&local,RouterPeer::localRx)==Ok);
  CHECK(local.qrtr.step()==Ok&&le32(local.sent.back().data()+36)==55);
  RouterPeer zero;
  incoming=qrtrPacket(Qrtr::NewServer,Qrtr::Ctrl,Qrtr::Ctrl,ctrl(Qrtr::NewServer,0x45,1,0,900));
  put32(incoming.data()+8,0);
  CHECK(zero.qrtr.receive(incoming.data(),incoming.size())==Ok&&zero.ready&&zero.qrtr.serverNode==0);
  CHECK(zero.qrtr.sendFrom(4096,0,900,&one,1)==Ok);
  incoming=qrtrPacket(Qrtr::NewServer,Qrtr::Ctrl,Qrtr::Ctrl,ctrl(Qrtr::NewServer,0x45,1,0,0));
  put32(incoming.data()+8,0);CHECK(zero.qrtr.receive(incoming.data(),incoming.size())==Ok&&zero.ready);
  RouterPeer prepared;prepared.qrtr.stop();
  CHECK(prepared.qrtr.addLocalService(4096,1,4096,&prepared,RouterPeer::localRx)==Ok);
  CHECK(prepared.qrtr.start(prepared.f.allocator(),&prepared,RouterPeer::tx,RouterPeer::event,RouterPeer::rx)==Ok);
  CHECK(prepared.qrtr.step()==Ok&&prepared.sent.size()==2);
  CHECK(le32(prepared.sent.back().data()+32)==Qrtr::NewServer&&
        le32(prepared.sent.back().data()+36)==4096&&le32(prepared.sent.back().data()+40)==1);
}

struct FirmwareServicePeer {
  std::vector<uint8_t> reply;uint32_t destination=99;
  static int tx(void *p,uint32_t,uint32_t node,uint32_t,const uint8_t *v,uint32_t n) {
    auto &f=*static_cast<FirmwareServicePeer *>(p);f.destination=node;f.reply.assign(v,v+n);return Ok;
  }
  static int size(void *,const char *path,uint32_t *bytes) {
    CHECK(!strcmp(path,"/image/wlanmdsp.mbn"));*bytes=3;return Ok;
  }
  static int read(void *,const char *,uint32_t at,uint8_t *out,uint32_t n) {
    CHECK(at==0&&n==3);memcpy(out,"abc",3);return Ok;
  }
};
static void firmwareServiceTest() {
  CHECK(QftpReadOnly::ServiceInstance==1); // version1, instance0 on the wire
  FirmwareServicePeer f;QftpReadOnly q;
  CHECK(q.start(&f,FirmwareServicePeer::tx,FirmwareServicePeer::size,FirmwareServicePeer::read)==Ok);
  std::vector<uint8_t> request={0,1};
  const char name[]="/readonly/firmware/image/wlanmdsp.mbn";
  request.insert(request.end(),name,name+sizeof(name));
  const char mode[]="octet";request.insert(request.end(),mode,mode+sizeof(mode));
  CHECK(q.receive(0,700,request.data(),request.size())==Ok&&f.destination==0);
  CHECK(f.reply.size()==7&&f.reply[1]==3&&f.reply[3]==1&&!memcmp(f.reply.data()+4,"abc",3));
  const uint8_t ack[]={0,4,0,1};CHECK(q.receive(0,700,ack,sizeof(ack))==Ok&&!q.active);
  GuampServreg registry;CHECK(registry.start(&f,FirmwareServicePeer::tx)==Ok);
  uint8_t query[64]={};TlvWriter writer={query+7,sizeof(query)-7};
  CHECK(writer.add(1,"wlan/fw",7));put16(query+1,42);put16(query+3,0x21);put16(query+5,writer.size);
  CHECK(registry.receive(0,701,query,7+writer.size)==Ok&&f.destination==0&&f.reply.size()>7&&f.reply[0]==2);
}
struct WlanPeer {
  Wlfw wlan; std::vector<std::vector<uint8_t>> sent;
  std::vector<uint8_t> board=std::vector<uint8_t>(26328);
  uint32_t sends=0,assignments=0,powerOns=0,reads=0;
  bool rejectMsa=false;
  static int tx(void *p,const uint8_t *v,uint32_t n) { auto &f=*static_cast<WlanPeer *>(p); f.sent.emplace_back(v,v+n); f.sends++; return Ok; }
  static int assign(void *p,const Wlfw::Region *r,uint32_t n) {
    auto &f=*static_cast<WlanPeer *>(p); CHECK(n==1&&r[0].address==0x51900000&&r[0].size==0x100000&&!r[0].secure);
    f.assignments++; return f.rejectMsa?RemoteError:Ok;
  }
  static int power(void *p,bool on) { auto &f=*static_cast<WlanPeer *>(p); CHECK(on); f.powerOns++; return Ok; }
  static int size(void *p,const char *name,uint32_t *n) {
    auto &f=*static_cast<WlanPeer *>(p); CHECK(strcmp(name,"/image/bdwlan_guamp_ipa.bin")==0); *n=uint32_t(f.board.size()); return Ok;
  }
  static int read(void *p,const char *,uint32_t at,uint8_t *out,uint32_t n) {
    auto &f=*static_cast<WlanPeer *>(p); CHECK(at<=f.board.size()&&n<=f.board.size()-at); memcpy(out,f.board.data()+at,n); f.reads++; return Ok;
  }
  WlanPeer() { for(size_t i=0;i<board.size();i++) board[i]=uint8_t(i*31); Wlfw::Ops o={this,tx,assign,power,size,read,nullptr}; CHECK(wlan.start(o,0)==Ok); }
  void response(uint16_t expected,const std::vector<uint8_t> &extra={},uint16_t result=0,uint16_t error=0,uint16_t idAdjust=0) {
    CHECK(!sent.empty()); auto &last=sent.back(); CHECK(le16(last.data()+3)==expected);
    std::vector<uint8_t> p(14+extra.size()); p[0]=2; put16(p.data()+1,uint16_t(le16(last.data()+1)+idAdjust)); put16(p.data()+3,expected); put16(p.data()+5,uint16_t(p.size()-7));
    p[7]=2; put16(p.data()+8,4); put16(p.data()+10,result); put16(p.data()+12,error);
    if(!extra.empty()) memcpy(p.data()+14,extra.data(),extra.size()); CHECK(wlan.receive(p.data(),p.size())==(idAdjust?NotFound:Ok));
  }
  void indication(uint16_t msg) { uint8_t p[7]={4,0,0,0,0,0,0}; put16(p+3,msg); CHECK(wlan.receive(p,7)==Ok); }
  void prepare(uint64_t now=0) {
    CHECK(wlan.step(now)==Ok); response(0x20); CHECK(wlan.step(now+1)==Ok); CHECK(wlan.state==Wlfw::MsaInfo);
    // Mandatory MSA info response array: count u8 + address u64 + size u32 + secure u8.
    std::vector<uint8_t> region(17); region[0]=3; put16(region.data()+1,14); region[3]=1;
    put64(region.data()+4,0x51900000); put32(region.data()+12,0x100000);
    response(0x2d,region); CHECK(wlan.step(now+2)==Ok&&wlan.state==Wlfw::MsaAssign);
    CHECK(assignments==0); CHECK(wlan.step(now+21)==Ok&&assignments==0);
    CHECK(wlan.step(now+22)==Ok&&assignments==1); response(0x2e); CHECK(wlan.step(now+23)==Ok);
    std::vector<uint8_t> caps(11); caps[0]=0x10; put16(caps.data()+1,8); put32(caps.data()+3,0x3990); put32(caps.data()+7,1);
    response(0x24,caps); CHECK(wlan.step(now+24)==Ok&&wlan.state==Wlfw::WaitMsa&&powerOns==1);
  }
};
static void wlanTest() {
  auto p=std::make_unique<WlanPeer>(); p->prepare(); p->indication(0x2b); CHECK(p->wlan.step(25)==Ok);
  std::vector<uint8_t> transmitted;
  uint32_t segment=0;
  while(p->wlan.state==Wlfw::Board) {
    const auto &request=p->sent.back(); CHECK(le16(request.data()+3)==0x25);
    TlvView b={request.data()+7,uint32_t(request.size()-7)}; CHECK(b.validate()==Ok); uint32_t n=0;
    const uint8_t *s=b.find(0x12,&n); CHECK(s&&n==4&&le32(s)==segment++);
    const uint8_t *data=b.find(0x13,&n); CHECK(data&&n>=2&&le16(data)==n-2);
    transmitted.insert(transmitted.end(),data+2,data+n);
    const uint8_t *end=b.find(0x14,&n); CHECK(end&&n==1&&*end==(transmitted.size()==p->board.size()));
    p->response(0x25); CHECK(p->wlan.step(30+segment)==Ok);
  }
  CHECK(transmitted==p->board&&segment==5);
  CHECK(p->wlan.state==Wlfw::Calibration); p->response(0x26); CHECK(p->wlan.step(50)==Ok&&p->wlan.state==Wlfw::WaitFirmware);
  p->indication(0x21); CHECK(p->wlan.step(51)==Ok&&p->wlan.state==Wlfw::FirmwareReady);
  Wlfw::Pipe pipe={0,2,32,2048,0}; Wlfw::ServicePipe svc={0x100,2,0};
  CHECK(p->wlan.enable(&pipe,1,&svc,1,nullptr,0)==Ok); CHECK(p->wlan.step(52)==Ok); p->response(0x23);
  CHECK(p->wlan.step(53)==Ok&&p->wlan.state==Wlfw::Mode); p->response(0x22); CHECK(p->wlan.step(54)==Ok&&p->wlan.state==Wlfw::Enabled);
  CHECK(p->wlan.disable()==Ok&&p->wlan.state==Wlfw::Disable); CHECK(p->wlan.step(55)==Ok);
  TlvView stop={p->sent.back().data()+7,uint32_t(p->sent.back().size()-7)}; uint32_t size=0;
  const uint8_t *off=stop.find(1,&size); CHECK(off&&size==4&&le32(off)==4);
  p->response(0x22); CHECK(p->wlan.step(56)==Ok&&p->wlan.state==Wlfw::FirmwareReady);
  auto timeout=std::make_unique<WlanPeer>(); CHECK(timeout->wlan.step(0)==Ok);
  timeout->response(0x20,{},0,0,1); CHECK(timeout->wlan.qmi.pending);
  CHECK(timeout->wlan.step(10000)==Timeout&&timeout->wlan.state==Wlfw::Failed);
  auto remote=std::make_unique<WlanPeer>(); CHECK(remote->wlan.step(0)==Ok); remote->response(0x20,{},1,42);
  CHECK(remote->wlan.step(1)==RemoteError&&remote->wlan.qmi.remoteError==42);
  auto reset=std::make_unique<WlanPeer>(); reset->prepare(); reset->wlan.peerLost(); CHECK(reset->wlan.state==Wlfw::Failed&&reset->wlan.step(100)==PeerReset);
}
static void malformedTest() {
  std::mt19937 gen(0x6115);
  Qmi q; q.init(nullptr,[](void *,const uint8_t *,uint32_t)->int{return Ok;},nullptr);
  Qrtr::Header h={};
  for(uint32_t i=0;i<20000;i++) {
    uint32_t n=gen()%300; std::vector<uint8_t> p(n);
    for(auto &v:p) v=uint8_t(gen());
    TlvView v={p.data(),n}; v.validate(); uint32_t bytes=0; v.find(uint8_t(gen()),&bytes);
    Qrtr::decode(p.data(),n,&h); q.receive(p.data(),n);
  }
}
// guamp SMEM as read on the phone (28/10): smem_info "SIII" with reserved (+12) = 0 and num_items at +16. Reading
// +12 gave 0 items and every allocation was Range: the Wi-Fi R1 "APPS SMP2P/GLINK prepare failed -102".
static void siii(Fixture &f,uint16_t items) {
  uint32_t t=uint32_t(f.bytes.size())-4096,info=t+32+48*1;
  memcpy(f.bytes.data()+info,"SIII",4); f.w(info+4,0); f.w(info+8,0); f.w(info+12,0); f.w(info+16,items);
}
struct ControlRig {
  Fixture f; SmemArena a; std::unique_ptr<WifiControl> c=std::make_unique<WifiControl>();
  std::vector<std::string> lines;
  static void trace(void *p,const char *what,int e,uint32_t x,uint32_t y,uint32_t z) {
    char b[256]; snprintf(b,sizeof b,"%s %d %u %u %u",what,e,x,y,z); static_cast<ControlRig *>(p)->lines.emplace_back(b);
  }
  bool saw(const char *s) const { for(auto &l:lines) if(l.find(s)!=std::string::npos) return true; return false; }
  explicit ControlRig(uint16_t items) { f.init(true); siii(f,items); a=f.arena(); }
  const Rmtfs::Storage *rmtfs=nullptr;
  int prepare() {
    WifiControl::Ops o{}; o.ctx=this; o.trace=trace; o.rmtfs=rmtfs;
    o.smpKick=[](void *p){ static_cast<ControlRig *>(p)->f.kicks++; }; o.glinkKick=o.smpKick;
    o.wlan={this,nullptr,[](void *,const Wlfw::Region *,uint32_t)->int{return Ok;},[](void *,bool)->int{return Ok;},
      [](void *,const char *,uint32_t *n)->int{*n=0;return NotFound;},
      [](void *,const char *,uint32_t,uint8_t *,uint32_t)->int{return NotFound;},nullptr};
    return c->prepare(&a,f.allocator(),o,true);
  }
};
static void controlTest() {
  { ControlRig r(512);
    smem::Info i={}; CHECK(r.a.parse(&i)==Ok&&i.infoPresent&&i.numItems==512);
    CHECK(r.prepare()==Ok&&r.c->stage==WifiControl::Prepared);
    uint32_t at=0,n=0;
    CHECK(r.a.find(1,435,&at,&n)==Ok&&n==340&&r.a.find(1,478,&at,&n)==Ok&&n==32&&r.a.find(1,479,&at,&n)==Ok&&n==16384);
    // outbound "master-kernel" + "ipa" (bengal.dtsi order), version published
    const uint8_t *o=r.f.bytes.data()+r.c->smp.outbound;
    CHECK(le32(o+4)==0x101&&le32(o+12)==(16u|(2u<<16))&&!strcmp((const char *)o+20,"master-kernel")&&!strcmp((const char *)o+40,"ipa"));
    CHECK(r.saw("smem version/num_items/infoPresent 0 12 512 1")&&r.saw("smem item id/offset/bytes 0 478"));
    const uint32_t kicks=r.f.kicks; CHECK(kicks==1);   // only the SMP2P probe kick before the modem runs
    CHECK(r.c->step(5)==NotReady);
    CHECK(r.c->modemStarted(1000)==Ok&&r.c->step(1001)==Ok&&r.c->stage==WifiControl::AwaitModem);
    // inbound allocated but not published yet: keep waiting (it used to be a fatal Protocol)
    uint32_t in=0; CHECK(r.a.allocate(1,428,340,&in)==Ok);
    CHECK(r.c->step(1002)==Ok&&r.c->stage==WifiControl::AwaitModem&&r.saw("smp2p 428 inbound present"));
    r.f.w(in,0x504d5324); r.f.w(in+8,1); r.f.w(in+12,16u|(1u<<16));
    memcpy(r.f.bytes.data()+in+20,"slave-kernel",13); r.f.w(in+36,2); r.f.w(in+4,0x101);
    CHECK(r.c->step(1003)==Ok&&r.c->stage==WifiControl::AwaitFifo&&r.saw("smp2p modem ready"));
    CHECK(r.c->step(1004)==Ok&&r.c->stage==WifiControl::AwaitFifo);
    uint32_t rx=0; CHECK(r.a.allocate(1,480,16384,&rx)==Ok);
    CHECK(r.c->step(1005)==Ok&&r.c->stage==WifiControl::Running&&r.c->glink.running&&r.saw("smem 480 found"));
    // GLINK VERSION(1, features 1) is the first frame in the TX FIFO 479, head advanced, IRQ kicked
    CHECK(le16(r.f.bytes.data()+r.c->outboundFifo)==0&&le16(r.f.bytes.data()+r.c->outboundFifo+2)==1&&
          le32(r.f.bytes.data()+r.c->descriptor+4)==8&&r.f.kicks>kicks);
    CHECK(!strcmp(r.c->where(),"glink version (mpss edge)"));
    CHECK(r.c->step(1004+WifiControl::ServiceMs)==Ok);   // the WLFW deadline counts from the GLINK start (1005)
    CHECK(r.c->step(1005+WifiControl::ServiceMs)==Timeout&&r.saw("FAILED")&&r.saw(" glink running/negotiated/rx 0 1 0 0"));
  }
  { // RMTFS registered next to QFTP/SERVREG (announced by Qrtr::start); released with the transport
    ControlRig r(512); std::vector<uint8_t> shared(0x202000),part(0x10000);
    Rmtfs::Storage io{}; io.ctx=&shared;
    io.shared={&shared,uint32_t(shared.size()),[](void *p,uint32_t at)->uint32_t{return le32(static_cast<std::vector<uint8_t> *>(p)->data()+at);},
      [](void *p,uint32_t at,uint32_t v){put32(static_cast<std::vector<uint8_t> *>(p)->data()+at,v);},[](void *){}};
    io.sharedOffset=0x1000; io.physical=0x7f001000; io.bytes=0x200000; io.allocator=r.f.allocator();
    io.partition=[](void *,uint32_t,const char *,uint64_t *n)->int{*n=0x10000;return Ok;};
    io.read=[](void *,uint32_t,uint64_t,uint8_t *o,uint32_t n)->int{memset(o,0,n);return Ok;};
    r.rmtfs=&io;
    CHECK(r.prepare()==Ok&&r.c->rmtfs.ready);
    bool found=false; for(auto &l:r.c->qrtr.local) found|=l.used&&l.service==14&&l.instance==1&&l.port==0x4300;
    CHECK(found&&r.f.live>0);
    CHECK(r.c->modemStarted(0)==Ok&&r.c->step(WifiControl::ReadyMs)==Timeout&&!r.c->rmtfs.ready);
    CHECK(r.c->release(true)==Ok);
  }
  { ControlRig r(400);   // Linux semantics: item ids >= num_items are refused
    CHECK(r.prepare()==Range&&!strcmp(r.c->where(),"smem 435 smp2p out")&&r.saw("FAILED"));
  }
  { ControlRig r(512); CHECK(r.prepare()==Ok&&r.c->modemStarted(0)==Ok);
    CHECK(r.c->step(WifiControl::ReadyMs)==Timeout&&!strcmp(r.c->where(),"smp2p 428 slave-kernel ready (bit 1)"));
  }
}
int main() {
  smemTest(); smpTest(); glinkTest(); rpmTest(); msaTest(); powerPlanTest(); qrtrTest(); firmwareServiceTest(); wlanTest(); ceTest(); controlTest(); malformedTest();
  printf("PASS %u checks: SMEM/SMP2P/GLINK/QRTR/QMI/WLFW/SNOC-CE actual sources, independent wire peer, malformed inputs\n",checks);
}
