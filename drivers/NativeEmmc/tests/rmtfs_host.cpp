// GuampRmtfs.inc (actual source) against an independent QMI encoder/decoder written from linux-msm/rmtfs
// qmi_rmtfs.qmi, in-memory partitions and a word-only shared-memory fixture. Not a modem simulation.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include "../GuampWifiWire.inc"
#include "../GuampQmi.inc"
#include "../GuampQrtr.inc"
#include "../GuampRmtfs.inc"
using namespace guampwifi;
static unsigned checks=0;
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)

static constexpr uint64_t Phys=0x7f000000ull;      // allocation; region = Phys + 4 KiB
static constexpr uint32_t Guard=0x1000,Region=0x200000;
struct Packet { uint32_t sp,node,port; std::vector<uint8_t> data; };
struct Rig {
  std::vector<uint8_t> shared=std::vector<uint8_t>(Region+2*Guard,0xee);
  std::vector<uint8_t> part[4];
  bool present[4]={true,true,true,true};
  std::vector<Packet> sent;
  std::vector<std::string> lines;
  int sendResult=Ok; size_t live=0; uint32_t emmcReads=0,emmcWrites=0;
  Rmtfs r{};
  Rmtfs::Storage io{};
  static uint32_t rd(void *p,uint32_t at) { auto &g=*static_cast<Rig *>(p); CHECK(!(at&3)&&at+4<=g.shared.size()); return le32(&g.shared[at]); }
  static void wr(void *p,uint32_t at,uint32_t v) { auto &g=*static_cast<Rig *>(p); CHECK(!(at&3)&&at+4<=g.shared.size()); put32(&g.shared[at],v); }
  static void fence(void *) {}
  static void *alloc(void *p,uint32_t n) { static_cast<Rig *>(p)->live+=n; return calloc(1,n); }
  static void release(void *p,void *v,uint32_t n) { auto &g=*static_cast<Rig *>(p); CHECK(g.live>=n); g.live-=n; free(v); }
  static int partition(void *p,uint32_t i,const char *label,uint64_t *bytes) {
    static const char *names[4]={"modemst1","modemst2","fsc","fsg"};
    auto &g=*static_cast<Rig *>(p); CHECK(i<4&&!strcmp(label,names[i]));
    if(!g.present[i]) return NotFound; *bytes=g.part[i].size(); return Ok;
  }
  static int read(void *p,uint32_t i,uint64_t at,uint8_t *out,uint32_t n) {
    auto &g=*static_cast<Rig *>(p); CHECK(!(at%512)&&!(n%512)&&at+n<=g.part[i].size()); g.emmcReads++;
    memcpy(out,&g.part[i][at],n); return Ok;
  }
  static int write(void *p,uint32_t i,uint64_t at,const uint8_t *in,uint32_t n) {
    auto &g=*static_cast<Rig *>(p); CHECK(i<2&&!(at%512)&&!(n%512)&&at+n<=g.part[i].size()); g.emmcWrites++;
    memcpy(&g.part[i][at],in,n); return Ok;
  }
  static void trace(void *p,const char *what,const char *name,int e,uint64_t a,uint64_t b,uint64_t c) {
    char line[320]; snprintf(line,sizeof line,"%s%s%s: %llu/%llu/%llu %d",name?name:"",name?" ":"",what,
      (unsigned long long)a,(unsigned long long)b,(unsigned long long)c,e);
    static_cast<Rig *>(p)->lines.emplace_back(line);
  }
  static int tx(void *p,uint32_t sp,uint32_t node,uint32_t port,const uint8_t *v,uint32_t n) {
    auto &g=*static_cast<Rig *>(p); if(g.sendResult) return g.sendResult;
    g.sent.push_back({sp,node,port,std::vector<uint8_t>(v,v+n)}); return Ok;
  }
  bool saw(const char *s) const { for(auto &l:lines) if(l.find(s)!=std::string::npos) return true; return false; }
  explicit Rig(bool emmcWrite=false,uint32_t fsgBytes=1u<<20) {
    const uint32_t sizes[4]={2621440,2621440,131072,fsgBytes};
    for(int i=0;i<4;i++) { part[i].resize(sizes[i]); for(uint32_t k=0;k<sizes[i];k++) part[i][k]=uint8_t(k*7+i*31+(k>>9)); }
    io={this,{this,uint32_t(shared.size()),rd,wr,fence},Guard,Phys+Guard,Region,{this,alloc,release},
        partition,read,emmcWrite?write:nullptr,trace};
  }
  int start() { return r.start(io,this,tx); }
  ~Rig() { r.stop(); CHECK(live==0); }
};
// Independent encoder: QMI header (type 0 request, txn, msg, len) + TLVs.
struct Req {
  std::vector<uint8_t> b;
  Req(uint16_t txn,uint16_t msg) { b={0,uint8_t(txn),uint8_t(txn>>8),uint8_t(msg),uint8_t(msg>>8),0,0}; }
  Req &tlv(uint8_t t,const std::vector<uint8_t> &v) {
    b.push_back(t); b.push_back(uint8_t(v.size())); b.push_back(uint8_t(v.size()>>8)); b.insert(b.end(),v.begin(),v.end());
    uint16_t n=uint16_t(b.size()-7); b[5]=uint8_t(n); b[6]=uint8_t(n>>8); return *this;
  }
  Req &u32(uint8_t t,uint32_t v) { return tlv(t,{uint8_t(v),uint8_t(v>>8),uint8_t(v>>16),uint8_t(v>>24)}); }
  Req &u8(uint8_t t,uint8_t v) { return tlv(t,{v}); }
  Req &str(uint8_t t,const char *s,bool nul=false) { std::vector<uint8_t> v(s,s+strlen(s)); if(nul)v.push_back(0); return tlv(t,v); }
};
struct Entry { uint32_t sector,address,count; };
static Req iovecReq(uint16_t txn,uint32_t caller,uint8_t dir,std::vector<Entry> e,uint8_t force=0) {
  std::vector<uint8_t> a={uint8_t(e.size())};
  for(auto &x:e) for(uint32_t v:{x.sector,x.address,x.count}) for(int k=0;k<4;k++) a.push_back(uint8_t(v>>(8*k)));
  Req q(txn,3); q.u32(1,caller).u8(2,dir).tlv(3,a).u8(4,force); return q;
}
// Independent decoder of a response: checks header and returns result/error and an optional TLV.
struct Resp { uint16_t txn,msg,result,error; std::vector<uint8_t> opt; uint8_t optTag; };
static Resp decode(const Packet &p) {
  const auto &d=p.data; CHECK(d.size()>=14&&d[0]==2&&le16(&d[5])+7u==d.size()&&p.sp==Rmtfs::Port);
  Resp r={le16(&d[1]),le16(&d[3]),0xffff,0xffff,{},0};
  for(size_t at=7;at<d.size();) {
    uint8_t t=d[at]; uint16_t n=le16(&d[at+1]); CHECK(at+3+n<=d.size());
    if(t==2) { CHECK(n==4); r.result=le16(&d[at+3]); r.error=le16(&d[at+5]); }
    else { r.optTag=t; r.opt.assign(d.begin()+long(at)+3,d.begin()+long(at)+3+n); }
    at+=3+n;
  }
  CHECK(r.result!=0xffff); return r;
}
static Resp ask(Rig &g,const Req &q,uint32_t node=0,uint32_t port=0x5001) {
  size_t before=g.sent.size(); g.r.receive(node,port,q.b.data(),uint32_t(q.b.size()));
  CHECK(g.sent.size()==before+1&&g.sent.back().node==node&&g.sent.back().port==port);
  Resp r=decode(g.sent.back()); CHECK(r.txn==le16(&q.b[1])&&r.msg==le16(&q.b[3])); return r;
}
static uint32_t openCaller(Rig &g,const char *path,uint16_t txn=1,uint32_t node=0) {
  Resp r=ask(g,Req(txn,1).str(1,path),node); CHECK(r.result==0&&r.error==0&&r.optTag==0x10&&r.opt.size()==4);
  return le32(r.opt.data());
}
static const uint8_t *region(Rig &g,uint32_t off=0) { return &g.shared[Guard+off]; }

static void wireTest() {
  Rig g; CHECK(g.start()==Ok&&g.saw("server ready: service/instance/port: 14/1/17152 0"));
  CHECK(Rmtfs::Service==14&&Rmtfs::ServiceInstance==1&&Rmtfs::Port==0x4300);
  CHECK(g.saw("/boot/modem_fs1 partition: bytes/eMMC writes/index: 2621440/0/0 0"));
  // open: exact response bytes 02 txn 01 00 0e 00 | 02 04 00 00 00 00 00 | 10 04 00 00 00 00 00
  Req q(0x1234,1); q.str(1,"/boot/modem_fs1");
  g.r.receive(0,0x5001,q.b.data(),uint32_t(q.b.size()));
  const std::vector<uint8_t> want={2,0x34,0x12,1,0,14,0, 2,4,0,0,0,0,0, 0x10,4,0,0,0,0,0};
  CHECK(g.sent.size()==1&&g.sent[0].data==want&&g.sent[0].sp==0x4300&&g.sent[0].node==0&&g.sent[0].port==0x5001);
  CHECK(openCaller(g,"/boot/modem_fs1",2)==0);              // same node + partition: same handle
  CHECK(openCaller(g,"/boot/modem_fs2",3)==1);
  CHECK(openCaller(g,"/boot/modem_fsg",4)==2);
  CHECK(openCaller(g,"/boot/modem_fsc",5)==3);
  { Resp r=ask(g,Req(6,1).str(1,"/boot/modem_fs1",true)); CHECK(r.result==0&&le32(r.opt.data())==0); }  // trailing NUL
  { Resp r=ask(g,Req(7,1).str(1,"/etc/passwd")); CHECK(r.result==1&&r.error==1&&r.opt.empty()&&g.saw("unknown path")); }
  { Resp r=ask(g,Req(8,1)); CHECK(r.result==1&&r.error==2); }                                          // no path
  // alloc_buff: absolute physical base, u64 TLV 0x10
  { Resp r=ask(g,Req(9,4).u32(1,0).u32(2,Region)); CHECK(r.result==0&&r.optTag==0x10&&r.opt.size()==8&&le64(r.opt.data())==Phys+Guard); }
  { Resp r=ask(g,Req(10,4).u32(1,0).u32(2,Region+1)); CHECK(r.result==1&&r.error==1&&g.saw("TOO LARGE")); }
  { Resp r=ask(g,Req(11,4).u32(1,9).u32(2,16)); CHECK(r.result==1&&r.error==1); }                         // unknown caller
  { Resp r=ask(g,Req(12,4).u32(1,0).u32(2,16),5); CHECK(r.result==1&&r.error==1); }                       // other node
  // get_dev_error: success, status 0
  { Resp r=ask(g,Req(13,5).u32(1,1)); CHECK(r.result==0&&r.optTag==0x10&&r.opt.size()==1&&r.opt[0]==0); }
  { Resp r=ask(g,Req(14,5).u32(1,7)); CHECK(r.result==1&&r.error==1); }
  // unknown message: logged, not answered (rmtfs.c)
  { Req u(15,0x22); u.u32(1,0); size_t n=g.sent.size(); CHECK(g.r.receive(0,0x5001,u.b.data(),uint32_t(u.b.size()))==Unsupported&&g.sent.size()==n); }
  // a response/indication sent to the server is ignored
  { Req u(16,1); u.b[0]=2; size_t n=g.sent.size(); CHECK(g.r.receive(0,0x5001,u.b.data(),uint32_t(u.b.size()))==Unsupported&&g.sent.size()==n); }
  // bad TLV framing answered as MALFORMED
  { Req u(17,2); u.u32(1,0); u.b[8]=9; Resp r; size_t n=g.sent.size(); CHECK(g.r.receive(0,0x5001,u.b.data(),uint32_t(u.b.size()))==Protocol);
    CHECK(g.sent.size()==n+1); r=decode(g.sent.back()); CHECK(r.result==1&&r.error==2); }
  // ten handles max, per node
  for(uint32_t node=1;node<=6;node++) openCaller(g,"/boot/modem_fs1",20,node);
  { Resp r=ask(g,Req(30,1).str(1,"/boot/modem_fs2"),9); CHECK(r.result==1&&r.error==1&&g.saw("10 handles")); }
}
static void iovecTest() {
  Rig g; CHECK(g.start()==Ok);
  const uint32_t fs1=openCaller(g,"/boot/modem_fs1"),fsc=openCaller(g,"/boot/modem_fsc",2);
  const uint32_t base=uint32_t(Phys+Guard);
  // read: 2 entries, absolute physical addresses
  { Resp r=ask(g,iovecReq(3,fs1,0,{{0,base,8},{100,base+0x10000,4}})); CHECK(r.result==0&&r.error==0); }
  CHECK(!memcmp(region(g),&g.part[0][0],8*512)&&!memcmp(region(g,0x10000),&g.part[0][100*512],4*512));
  CHECK(g.shared[Guard-1]==0xee&&g.shared[Guard+Region]==0xee);     // guards untouched
  // a whole 2.5 MiB partition does not fit the 2 MiB region: the modem reads it in pieces
  { Resp r=ask(g,iovecReq(4,fs1,0,{{0,base,4096}})); CHECK(r.result==0); CHECK(!memcmp(region(g),&g.part[0][0],Region)); }
  { Resp r=ask(g,iovecReq(5,fs1,0,{{0,base,4097}})); CHECK(r.result==1&&r.error==1&&g.saw("outside the shared region")); }
  // relative offsets are not accepted (sharedmem.c uses absolute addresses)
  { Resp r=ask(g,iovecReq(6,fs1,0,{{0,0x1000,1}})); CHECK(r.result==1&&r.error==1); }
  // read past the end of fsc (128 KiB = 256 sectors): zero-filled
  memset(&g.shared[Guard],0x55,1024);
  { Resp r=ask(g,iovecReq(7,fsc,0,{{255,base,2}})); CHECK(r.result==0); }
  CHECK(!memcmp(region(g),&g.part[2][255*512],512)); for(int i=512;i<1024;i++) CHECK(region(g)[i]==0);
  // write (direction 1): RAM overlay, eMMC untouched, read back through the server
  auto original=g.part[0];
  for(uint32_t i=0;i<3*512;i++) g.shared[Guard+0x20000+i]=uint8_t(0xa0+i%13);
  std::vector<uint8_t> written(&g.shared[Guard+0x20000],&g.shared[Guard+0x20000]+3*512);
  { Resp r=ask(g,iovecReq(8,fs1,1,{{130,base+0x20000,3}},1)); CHECK(r.result==0); }
  CHECK(g.part[0]==original&&g.emmcWrites==0&&g.r.shadowed(0)&&g.r.shadowBytes==Rmtfs::ChunkBytes);
  CHECK(g.saw("RAM overlay created")&&g.saw("rw_iovec write (force sync)"));
  memset(&g.shared[Guard],0,0x30000);
  { Resp r=ask(g,iovecReq(9,fs1,0,{{128,base,6}})); CHECK(r.result==0); }
  CHECK(!memcmp(region(g),&original[128*512],2*512)&&!memcmp(region(g,1024),written.data(),3*512)&&
        !memcmp(region(g,5*512),&original[133*512],512));
  // a write crossing a 64 KiB chunk boundary (sectors 127..128) and one past the end
  { Resp r=ask(g,iovecReq(10,fs1,1,{{127,base,2}})); CHECK(r.result==0&&g.r.shadowBytes==2*Rmtfs::ChunkBytes); }
  { Resp r=ask(g,iovecReq(11,fs1,1,{{5120,base,1}})); CHECK(r.result==1&&r.error==1&&g.saw("past the partition end")); }
  // malformed: array count does not match its bytes; missing direction
  { Req q=iovecReq(12,fs1,0,{{0,base,1}}); q.b[7+7+4+3]=2; Resp r=ask(g,q); CHECK(r.result==1&&r.error==2); }
  { Resp r=ask(g,Req(13,3).u32(1,fs1)); CHECK(r.result==1&&r.error==2); }
  // close; the handle is gone, the overlay stays for the boot
  { Resp r=ask(g,Req(14,2).u32(1,fs1)); CHECK(r.result==0&&r.opt.empty()); }
  { Resp r=ask(g,iovecReq(15,fs1,0,{{130,base,1}})); CHECK(r.result==1&&r.error==1); }
  CHECK(openCaller(g,"/boot/modem_fs1",16)==fs1);
  { Resp r=ask(g,iovecReq(17,fs1,0,{{130,base,1}})); CHECK(r.result==0&&!memcmp(region(g),written.data(),512)); }
  { Resp r=ask(g,Req(18,2).u32(1,fs1)); CHECK(r.result==0); }
  { Resp r=ask(g,Req(19,2).u32(1,fs1)); CHECK(r.result==1&&r.error==1); }
}
static void emmcWriteTest() {
  Rig g(true); CHECK(g.start()==Ok&&g.saw("modemst writes to eMMC"));
  const uint32_t base=uint32_t(Phys+Guard),fs2=openCaller(g,"/boot/modem_fs2"),fsc=openCaller(g,"/boot/modem_fsc",2);
  for(uint32_t i=0;i<1024;i++) g.shared[Guard+i]=uint8_t(i^0x5a);
  { Resp r=ask(g,iovecReq(3,fs2,1,{{40,base,2}})); CHECK(r.result==0); }
  CHECK(g.emmcWrites==1&&!memcmp(&g.part[1][40*512],region(g),1024)&&!g.r.shadowed(1));
  auto fscBefore=g.part[2];
  { Resp r=ask(g,iovecReq(4,fsc,1,{{0,base,1}})); CHECK(r.result==0); }
  CHECK(g.emmcWrites==1&&g.part[2]==fscBefore&&g.r.shadowed(2));        // fsc/fsg never written to eMMC
}
static void overlayLimitTest() {
  Rig g(false,17u<<20); CHECK(g.start()==Ok);
  const uint32_t base=uint32_t(Phys+Guard),fsg=openCaller(g,"/boot/modem_fsg");
  for(uint32_t c=0;c<256;c++) { Resp r=ask(g,iovecReq(uint16_t(2+c),fsg,1,{{c*128,base,1}})); CHECK(r.result==0); }
  CHECK(g.r.shadowBytes==Rmtfs::ShadowLimit);
  { Resp r=ask(g,iovecReq(300,fsg,1,{{256*128,base,1}})); CHECK(r.result==1&&g.saw("RAM overlay full")); }
  { Resp r=ask(g,iovecReq(301,fsg,1,{{3,base,1}})); CHECK(r.result==0); }  // existing chunk still writable
}
static void missingPartitionTest() {
  Rig g; g.present[3]=false; CHECK(g.start()==Ok&&g.saw("/boot/modem_fsg partition MISSING"));
  { Resp r=ask(g,Req(1,1).str(1,"/boot/modem_fsg")); CHECK(r.result==1&&r.error==1&&g.saw("partition missing")); }
  Rig h; for(auto &p:h.present) p=false; CHECK(h.start()==Ok&&h.saw("server ready: service/instance/port: 14/1/17152 -111"));
}
static void retryTest() {
  Rig g; CHECK(g.start()==Ok);
  g.sendResult=Again; Req q(1,1); q.str(1,"/boot/modem_fs1");
  CHECK(g.r.receive(0,0x5001,q.b.data(),uint32_t(q.b.size()))==Ok&&g.sent.empty());
  CHECK(g.r.step()==Ok&&g.sent.empty());
  g.sendResult=Ok; CHECK(g.r.step()==Ok&&g.sent.size()==1);
  Resp r=decode(g.sent[0]); CHECK(r.result==0&&r.txn==1&&g.sent[0].port==0x5001);
  CHECK(g.r.step()==Ok&&g.sent.size()==1);
  // invalid configuration
  Rmtfs bad{}; Rmtfs::Storage s=g.io; s.bytes=0; CHECK(bad.start(s,&g,Rig::tx)==Invalid);
  s=g.io; s.sharedOffset=2; CHECK(bad.start(s,&g,Rig::tx)==Invalid);
  s=g.io; s.bytes=Region+Guard+1; CHECK(bad.start(s,&g,Rig::tx)==Invalid);
}
// QRTR: the server is announced (NEW_SERVER 14 / instance 1 / node 1 / port 0x4300) when the router starts and on
// the modem's NEW_LOOKUP; DATA to port 0x4300 reaches it and the answer leaves from port 0x4300.
struct Router {
  std::vector<std::vector<uint8_t>> out; Rig g; Qrtr q{};
  static void *alloc(void *,uint32_t n) { return calloc(1,n); }
  static void release(void *,void *p,uint32_t) { free(p); }
  static int tx(void *p,const uint8_t *v,uint32_t n) { static_cast<Router *>(p)->out.emplace_back(v,v+n); return Ok; }
  static void data(void *,const uint8_t *,uint32_t) {}
  static int serverTx(void *p,uint32_t sp,uint32_t dn,uint32_t dp,const uint8_t *v,uint32_t n) {
    return static_cast<Router *>(p)->q.sendFrom(sp,dn,dp,v,n);
  }
  static void rx(void *p,uint32_t node,uint32_t port,const uint8_t *v,uint32_t n) { static_cast<Router *>(p)->g.r.receive(node,port,v,n); }
};
static std::vector<uint8_t> v1(uint32_t type,uint32_t sn,uint32_t sp,uint32_t dn,uint32_t dp,const std::vector<uint8_t> &body) {
  std::vector<uint8_t> p(32+((body.size()+3)&~size_t(3)));
  put32(&p[0],1); put32(&p[4],type); put32(&p[8],sn); put32(&p[12],sp); put32(&p[20],uint32_t(body.size())); put32(&p[24],dn); put32(&p[28],dp);
  memcpy(&p[32],body.data(),body.size()); return p;
}
static void qrtrTest() {
  Router r; CHECK(r.g.r.start(r.g.io,&r,Router::serverTx)==Ok);
  CHECK(r.q.addLocalService(Rmtfs::Service,Rmtfs::ServiceInstance,Rmtfs::Port,&r,Router::rx)==Ok);
  CHECK(r.q.start({nullptr,Router::alloc,Router::release},&r,Router::tx,nullptr,Router::data)==Ok&&r.q.step()==Ok);
  bool announced=false;
  for(auto &p:r.out) if(le32(&p[4])==Qrtr::NewServer) {
    const uint8_t *b=&p[32]; announced=le32(b)==4&&le32(b+4)==14&&le32(b+8)==1&&le32(b+12)==1&&le32(b+16)==0x4300&&le32(&p[24])==Qrtr::Broadcast;
  }
  CHECK(announced); r.out.clear();
  std::vector<uint8_t> lookup(20); put32(&lookup[0],Qrtr::NewLookup); put32(&lookup[4],14); put32(&lookup[8],1);
  auto pkt=v1(Qrtr::NewLookup,0,0xfffffffeu,1,0xfffffffeu,lookup);
  CHECK(r.q.receive(pkt.data(),uint32_t(pkt.size()))==Ok&&r.q.step()==Ok&&r.out.size()==1&&le32(&r.out[0][32+16])==0x4300);
  r.out.clear();
  Req open(0x77,1); open.str(1,"/boot/modem_fs2");
  pkt=v1(Qrtr::Data,0,0x5002,1,0x4300,open.b);
  CHECK(r.q.receive(pkt.data(),uint32_t(pkt.size()))==Ok&&r.q.step()==Ok&&r.out.size()==1);
  const auto &o=r.out[0];
  CHECK(le32(&o[4])==Qrtr::Data&&le32(&o[8])==1&&le32(&o[12])==0x4300&&le32(&o[24])==0&&le32(&o[28])==0x5002);
  Packet p={0x4300,0,0x5002,std::vector<uint8_t>(o.begin()+32,o.begin()+32+le32(&o[20]))};
  Resp resp=decode(p); CHECK(resp.txn==0x77&&resp.result==0&&le32(resp.opt.data())==0);
  r.q.stop();
}
int main() {
  wireTest(); iovecTest(); emmcWriteTest(); overlayLimitTest(); missingPartitionTest(); retryTest(); qrtrTest();
  printf("PASS %u checks: RMTFS QMI open/close/rw_iovec/alloc_buff/get_dev_error, RAM overlay, eMMC write-through, QRTR service 14\n",checks);
}
