// GuampQftp.inc (tqftpserv behaviour) and GuampServreg.inc (pd-mapper GET_DOMAIN_LIST) against independent
// encoders/decoders written from tqftpserv.c/translate.c and service-locator-private.h. Not a modem simulation.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <map>
#include "../GuampWifiWire.inc"
#include "../GuampQmi.inc"
#include "../GuampQftp.inc"
#include "../GuampServreg.inc"
using namespace guampwifi;
static unsigned checks=0;
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)

struct Sent { uint32_t from,node,port; std::vector<uint8_t> data; };
struct Rig {
  std::map<std::string,std::vector<uint8_t>> files;
  std::vector<Sent> sent; std::vector<std::string> lines; size_t live=0; int sendResult=Ok;
  Qftp q{};
  static int tx(void *p,uint32_t from,uint32_t node,uint32_t port,const uint8_t *v,uint32_t n) {
    auto &g=*static_cast<Rig *>(p); if(g.sendResult) return g.sendResult; g.sent.push_back({from,node,port,std::vector<uint8_t>(v,v+n)}); return Ok;
  }
  static int size(void *p,const char *path,uint32_t *n) {
    auto &g=*static_cast<Rig *>(p); auto it=g.files.find(path); if(it==g.files.end()) return NotFound; *n=uint32_t(it->second.size()); return Ok;
  }
  static int read(void *p,const char *path,uint32_t at,uint8_t *out,uint32_t n) {
    auto &g=*static_cast<Rig *>(p); auto &f=g.files.at(path); CHECK(n<=4096&&at+n<=f.size()); memcpy(out,f.data()+at,n); return Ok;
  }
  static void *alloc(void *p,uint32_t n) { static_cast<Rig *>(p)->live+=n; return calloc(1,n); }
  static void release(void *p,void *v,uint32_t n) { auto &g=*static_cast<Rig *>(p); CHECK(g.live>=n); g.live-=n; free(v); }
  static void trace(void *p,const char *what,const char *name,int e,uint64_t a,uint64_t b,uint64_t c) {
    char l[300]; snprintf(l,sizeof l,"%s %s: %llu/%llu/%llu %d",name?name:"",what,(unsigned long long)a,(unsigned long long)b,(unsigned long long)c,e);
    static_cast<Rig *>(p)->lines.emplace_back(l);
  }
  bool saw(const char *s) const { for(auto &l:lines) if(l.find(s)!=std::string::npos) return true; return false; }
  explicit Rig(bool rw=true) {
    std::vector<uint8_t> big(5000); for(size_t i=0;i<big.size();i++) big[i]=uint8_t(i*13+7);
    files["/image/wlanmdsp.mbn"]=big; files["/image/modem_pr/mcfg/x.mbn"]={1,2,3};
    Allocator a={this,alloc,release};
    CHECK(q.start(this,tx,size,read,rw?&a:nullptr)==Ok); q.traceCtx=this; q.trace=trace;
  }
  ~Rig() { q.stop(); CHECK(live==0); }
};
static std::vector<uint8_t> req(uint16_t op,const char *name,std::vector<std::pair<const char *,const char *>> opts={},const char *mode="octet") {
  std::vector<uint8_t> p={uint8_t(op>>8),uint8_t(op)};
  auto put=[&](const char *s){ p.insert(p.end(),s,s+strlen(s)+1); };
  put(name); put(mode); for(auto &o:opts){ put(o.first); put(o.second); } return p;
}
static std::vector<uint8_t> ack(uint16_t b) { return {0,4,uint8_t(b>>8),uint8_t(b)}; }
static std::vector<uint8_t> data(uint16_t b,const std::vector<uint8_t> &d) { std::vector<uint8_t> p={0,3,uint8_t(b>>8),uint8_t(b)}; p.insert(p.end(),d.begin(),d.end()); return p; }
static uint16_t op(const Sent &s) { return uint16_t(s.data[0]<<8|s.data[1]); }
static uint16_t blk(const Sent &s) { return uint16_t(s.data[2]<<8|s.data[3]); }
static std::map<std::string,std::string> oack(const Sent &s) {
  std::map<std::string,std::string> m; CHECK(op(s)==6);
  for(size_t at=2;at<s.data.size();) { std::string k((const char *)&s.data[at]); at+=k.size()+1; std::string v((const char *)&s.data[at]); at+=v.size()+1; m[k]=v; }
  return m;
}
static int rx(Rig &g,uint32_t local,const std::vector<uint8_t> &p,uint32_t port=0x50,uint64_t now=0) { return g.q.receive(local,0,port,p.data(),uint32_t(p.size()),now); }

static void readTest() {
  { Rig g; const uint32_t s0=Qftp::sessionPort(0);
    // no options: DATA 1 at once, from the session port (tqftpserv: new socket per transfer)
    CHECK(rx(g,Qftp::Port,req(1,"/readonly/firmware/image/wlanmdsp.mbn"))==Ok);
    CHECK(g.sent.size()==1&&g.sent[0].from==s0&&g.sent[0].port==0x50&&op(g.sent[0])==3&&blk(g.sent[0])==1&&g.sent[0].data.size()==516);
    CHECK(!memcmp(g.sent[0].data.data()+4,g.files["/image/wlanmdsp.mbn"].data(),512));
    CHECK(rx(g,Qftp::Port,ack(1),0x58)==Ok&&op(g.sent.back())==5&&g.sent.back().from==Qftp::Port);   // ACK on the service port: error
    for(uint16_t b=1;b<=9;b++) { CHECK(rx(g,s0,ack(b))==Ok); }
    CHECK(g.sent.back().data.size()==4+5000-9*512&&blk(g.sent.back())==10);   // 392-byte last block
    CHECK(rx(g,s0,ack(10))==Ok&&!g.q.active()&&g.saw("RRQ done"));
    CHECK(rx(g,s0,ack(10))==NotFound); }
  { Rig g; const uint32_t s0=Qftp::sessionPort(0);
    // options: OACK echoes blksize (reduced to 4096), tsize = file size, wsize, timeoutms; ACK 0 starts a window
    CHECK(rx(g,Qftp::Port,req(1,"/readonly/firmware/image/wlanmdsp.mbn",{{"blksize","7680"},{"tsize","0"},{"wsize","10"},{"timeoutms","500"}}))==Ok);
    auto o=oack(g.sent.at(0)); CHECK(o["blksize"]=="4096"&&o["tsize"]=="5000"&&o["wsize"]=="10"&&o["timeoutms"]=="500");
    CHECK(rx(g,s0,ack(0))==Ok&&g.sent.size()==3&&blk(g.sent[1])==1&&blk(g.sent[2])==2&&g.sent[2].data.size()==4+904);
    CHECK(rx(g,s0,ack(2))==Ok&&!g.q.active()); }
  { Rig g; const uint32_t s0=Qftp::sessionPort(0);
    // seek + rsize; timeout retransmits the window; a client ERROR (code 9 End of Transfer) ends the session
    CHECK(rx(g,Qftp::Port,req(1,"/readonly/vendor/firmware_mnt/image/wlanmdsp.mbn",{{"seek","1000"},{"rsize","600"}}))==Ok);
    auto o=oack(g.sent.at(0)); CHECK(o["seek"]=="1000"&&o["rsize"]=="600");
    CHECK(rx(g,s0,ack(0),0x50,10)==Ok&&g.sent.size()==2&&!memcmp(g.sent[1].data.data()+4,g.files["/image/wlanmdsp.mbn"].data()+1000,512));
    CHECK(g.q.step(500)==Ok&&g.sent.size()==2); CHECK(g.q.step(1011)==Ok&&g.sent.size()==3&&blk(g.sent[2])==1);
    std::vector<uint8_t> err={0,5,0,9,'e','n','d',0};
    CHECK(rx(g,s0,err)==Ok&&!g.q.active()&&g.saw("ended by the client")); }
  { Rig g;  // path translation and refusals
    CHECK(rx(g,Qftp::Port,req(1,"/readonly/firmware/modem_pr/mcfg/x.mbn"))==Ok&&op(g.sent.back())==3&&g.sent.back().data.size()==7);
    CHECK(rx(g,Qftp::Port,req(1,"/readonly/firmware/image/../secret"),0x51)==Ok&&op(g.sent.back())==5&&g.sent.back().data[3]==2);
    CHECK(rx(g,Qftp::Port,req(1,"/readonly/firmware/image//x"),0x52)==Ok&&g.sent.back().data[3]==2);
    CHECK(rx(g,Qftp::Port,req(1,"/etc/passwd"),0x53)==Ok&&g.sent.back().data[3]==2);
    CHECK(rx(g,Qftp::Port,req(1,"/readonly/firmware/image/none.bin"),0x54)==Ok&&g.sent.back().data[3]==1);
    CHECK(rx(g,Qftp::Port,req(1,"/readonly/firmware/image/wlanmdsp.mbn",{},"netascii"),0x55)==Ok&&g.sent.back().data[3]==4);
    CHECK(rx(g,Qftp::Port,req(2,"/readonly/firmware/image/wlanmdsp.mbn"),0x56)==Ok&&g.sent.back().data[3]==2);  // no write there
    // four concurrent sessions, the fifth is told to retry
    for(uint32_t i=0;i<3;i++) CHECK(rx(g,Qftp::Port,req(1,"/readonly/firmware/image/wlanmdsp.mbn"),0x60+i)==Ok);
    CHECK(g.q.active()&&rx(g,Qftp::Port,req(1,"/readonly/firmware/image/wlanmdsp.mbn"),0x70)==Ok&&g.sent.back().data[3]==0&&g.saw("all sessions busy")); }
}
static void writeTest() {
  { Rig g; const uint32_t s0=Qftp::sessionPort(0);
    // WRQ without options: ACK 0; short block ends; content kept in RAM and readable back
    CHECK(rx(g,Qftp::Port,req(2,"/readwrite/server_check.txt"))==Ok&&g.sent.size()==1&&op(g.sent[0])==4&&blk(g.sent[0])==0&&g.sent[0].from==s0);
    std::vector<uint8_t> text={'h','e','l','l','o'};
    CHECK(rx(g,s0,data(1,text))==Ok&&op(g.sent.back())==4&&blk(g.sent.back())==1&&!g.q.active());
    CHECK(g.saw("WRQ done")&&g.saw("hello WRQ content"));
    const Qftp::StoreFile *f=g.q.stored("/readwrite/server_check.txt"); CHECK(f&&f->size==5&&!memcmp(f->data,"hello",5));
    CHECK(rx(g,Qftp::Port,req(1,"/readwrite/server_check.txt"),0x51)==Ok&&op(g.sent.back())==3&&g.sent.back().data.size()==9&&
          !memcmp(g.sent.back().data.data()+4,"hello",5));
    CHECK(rx(g,Qftp::Port,req(1,"/readwrite/missing"),0x52)==Ok&&g.sent.back().data[3]==1); }
  { Rig g; const uint32_t s0=Qftp::sessionPort(0);
    // options: OACK; window of 2 blocks of 512: ACK after block 2, then the short block 3
    CHECK(rx(g,Qftp::Port,req(2,"/readwrite/log.bin",{{"blksize","512"},{"wsize","2"},{"tsize","1100"}}))==Ok);
    auto o=oack(g.sent.at(0)); CHECK(o["blksize"]=="512"&&o["wsize"]=="2"&&o["tsize"]=="1100");
    std::vector<uint8_t> a(512,0xa1),b(512,0xb2),c(76,0xc3);
    CHECK(rx(g,s0,data(1,a))==Ok&&g.sent.size()==1);                 // mid-window: no ACK
    CHECK(rx(g,s0,data(2,b))==Ok&&g.sent.size()==2&&blk(g.sent[1])==2);
    CHECK(rx(g,s0,data(2,b))==Ok&&g.sent.size()==3&&blk(g.sent[2])==2);  // duplicate: re-ACK
    CHECK(rx(g,s0,data(3,c))==Ok&&blk(g.sent.back())==3&&!g.q.active());
    const Qftp::StoreFile *f=g.q.stored("/readwrite/log.bin"); CHECK(f&&f->size==1100&&f->data[0]==0xa1&&f->data[600]==0xb2&&f->data[1099]==0xc3); }
  { Rig g; const uint32_t s0=Qftp::sessionPort(0);
    CHECK(rx(g,Qftp::Port,req(2,"/readwrite/x"))==Ok);
    CHECK(rx(g,s0,data(3,{1}))==Ok&&op(g.sent.back())==5&&g.sent.back().data[3]==4&&!g.q.active()); }   // out of sequence
  { Rig g(false);   // no RAM store: WRQ refused, nothing allocated
    CHECK(rx(g,Qftp::Port,req(2,"/readwrite/x"))==Ok&&op(g.sent.back())==5&&g.sent.back().data[3]==2); }
  { Rig g; const uint32_t s0=Qftp::sessionPort(0);   // a file larger than the RAM limit: "Disk full"
    CHECK(rx(g,Qftp::Port,req(2,"/readwrite/big",{{"blksize","4096"}}))==Ok);
    std::vector<uint8_t> blockData(4096,1); uint16_t b=1;
    for(;b<=Qftp::StoreFileBytes/4096;b++) CHECK(rx(g,s0,data(b,blockData))==Ok);
    CHECK(rx(g,s0,data(b,blockData))==Ok&&op(g.sent.back())==5&&g.sent.back().data[3]==3); }
}
// SERVREG_LOC: independent decoder of qmi_servreg_loc_get_domain_list_resp (u8 list length, u8 name length).
struct Entry { std::string name; uint32_t instance; uint8_t valid; uint32_t data; };
struct Domains { uint16_t result,total,rev; bool hasList; std::vector<Entry> list; };
static Domains decodeDomains(const std::vector<uint8_t> &p) {
  CHECK(p.size()>=7&&p[0]==2&&le16(&p[3])==0x21&&le16(&p[5])+7u==p.size());
  Domains d={0xffff,0,0,false,{}};
  for(size_t at=7;at<p.size();) {
    uint8_t t=p[at]; uint16_t n=le16(&p[at+1]); const uint8_t *v=&p[at+3]; CHECK(at+3+n<=p.size());
    if(t==2) d.result=le16(v);
    if(t==0x10) { CHECK(n==2); d.total=le16(v); }
    if(t==0x11) { CHECK(n==2); d.rev=le16(v); }
    if(t==0x12) { d.hasList=true; size_t k=1; for(uint8_t i=0;i<v[0];i++) { Entry e; uint8_t l=v[k++]; e.name.assign((const char *)v+k,l); k+=l;
        e.instance=le32(v+k); k+=4; e.valid=v[k++]; e.data=le32(v+k); k+=4; d.list.push_back(e); } CHECK(k==n); }
    at+=3+n;
  }
  return d;
}
struct ServregRig {
  GuampServreg s{}; std::vector<uint8_t> reply; uint32_t from=0; std::vector<std::string> lines;
  static int tx(void *p,uint32_t f,uint32_t,uint32_t,const uint8_t *v,uint32_t n) { auto &r=*static_cast<ServregRig *>(p); r.from=f; r.reply.assign(v,v+n); return Ok; }
  static void trace(void *p,const char *w,const char *name,int,uint64_t a,uint64_t b,uint64_t c) {
    char l[200]; snprintf(l,sizeof l,"%s %s %llu/%llu/%llu",name?name:"",w,(unsigned long long)a,(unsigned long long)b,(unsigned long long)c); static_cast<ServregRig *>(p)->lines.emplace_back(l);
  }
  ServregRig() { CHECK(s.start(this,tx)==Ok); s.traceCtx=this; s.trace=trace; }
  Domains ask(const char *name,bool nul=false,int offset=-1) {
    std::vector<uint8_t> q={0,7,0,0x21,0,0,0}; q.push_back(1); size_t l=strlen(name)+(nul?1:0); q.push_back(uint8_t(l)); q.push_back(0);
    q.insert(q.end(),name,name+l); if(offset>=0) { q.push_back(0x10); q.push_back(4); q.push_back(0); for(int k=0;k<4;k++) q.push_back(uint8_t(offset>>(8*k))); }
    put16(&q[5],uint16_t(q.size()-7)); CHECK(s.receive(0,4,q.data(),uint32_t(q.size()))==Ok&&from==GuampServreg::Port&&le16(&reply[1])==7);
    return decodeDomains(reply);
  }
};
static void servregTest() {
  ServregRig r;
  Domains d=r.ask("wlan/fw"); CHECK(d.result==0&&d.total==1&&d.rev==1&&d.list.size()==1);
  CHECK(d.list[0].name=="msm/modem/wlan_pd"&&d.list[0].instance==180&&d.list[0].valid==0&&d.list[0].data==0);
  // exact bytes of the list TLV: 12 01 | 1a 00 ... -> 0x12, len, count u8 = 1, name length u8 = 17
  size_t at=r.reply.size()-(3+1+1+17+9); CHECK(r.reply[at]==0x12&&r.reply[at+3]==1&&r.reply[at+4]==17);
  d=r.ask("tms/servreg",false,0); CHECK(d.total==6&&d.list.size()==6&&d.list[0].name=="msm/modem/root_pd"&&d.list[5].name=="msm/cdsp/root_pd");
  d=r.ask("tms/servreg",false,4); CHECK(d.total==6&&d.list.size()==2&&d.list[0].name=="msm/adsp/audio_pd");
  d=r.ask("tms/servreg",false,6); CHECK(d.total==6&&!d.hasList);
  d=r.ask("kernel/elf_loader",true); CHECK(d.total==1&&d.list[0].name=="msm/modem/wlan_pd");   // a trailing NUL is tolerated
  d=r.ask("avs/audio"); CHECK(d.total==1&&d.list[0].name=="msm/adsp/audio_pd"&&d.list[0].instance==74);
  d=r.ask("tms/pdr_enabled"); CHECK(d.total==1&&d.list[0].instance==180);
  d=r.ask("nothing/here"); CHECK(d.result==0&&d.total==0&&!d.hasList);
  bool logged=false; for(auto &l:r.lines) logged|=l.find("wlan/fw GET_DOMAIN_LIST -> entries/total/offset 1/1/0")!=std::string::npos; CHECK(logged);
  std::vector<uint8_t> other={0,1,0,0x20,0,0,0}; CHECK(r.s.receive(0,4,other.data(),7)==Unsupported);
}
int main() {
  readTest(); writeTest(); servregTest();
  printf("PASS %u checks: QFTP (tqftpserv sessions, options, windows, /readwrite RAM store, translate) and SERVREG_LOC domain list\n",checks);
}
