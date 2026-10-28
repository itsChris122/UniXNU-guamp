// GuampSim.inc + GuampQrtr.inc client ports, against hand-encoded QMI fixtures written
// from libqmi (uim/dms/nas json) and TS 31.102 EF layouts. Not a modem simulation.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include "../GuampWifiWire.inc"
#include "../GuampQmi.inc"
#include "../GuampQrtr.inc"
#include "../GuampSim.inc"
using namespace guampwifi;
using namespace guampsim;
static unsigned checks=0;
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)

typedef std::vector<uint8_t> Bytes;
struct Rig {
  Qrtr q{}; SimProbe s{};
  std::vector<Bytes> wire; std::vector<std::string> notes;
  static void *al(void *,uint32_t n) { return malloc(n); }
  static void fr(void *,void *p,uint32_t) { free(p); }
  static int tx(void *c,const uint8_t *p,uint32_t n) { static_cast<Rig *>(c)->wire.push_back(Bytes(p,p+n)); return Ok; }
  static void data(void *,const uint8_t *,uint32_t) {}
  static void ev(void *c,const char *w,int st) { static_cast<Rig *>(c)->notes.push_back(std::string(w)+":"+std::to_string(st)); }
  Rig() {
    s.event=ev; s.eventCtx=this;
    CHECK(s.start(&q,0)==Ok);
    Allocator a={}; a.ctx=this; a.alloc=al; a.free=fr;
    CHECK(q.start(a,this,tx,nullptr,data)==Ok);
    flush();
  }
  ~Rig() { q.stop(); }
  void flush() { while(q.count) CHECK(q.step()==Ok||q.count==0); }
  static Bytes packet(uint32_t type,uint32_t sn,uint32_t sp,uint32_t dn,uint32_t dp,const Bytes &body) {
    Bytes p(32+((body.size()+3)&~size_t(3)),0);
    put32(&p[0],1); put32(&p[4],type); put32(&p[8],sn); put32(&p[12],sp);
    put32(&p[20],uint32_t(body.size())); put32(&p[24],dn); put32(&p[28],dp);
    if(!body.empty()) memcpy(&p[32],body.data(),body.size());
    return p;
  }
  void server(bool up,uint32_t svc,uint32_t port) {
    Bytes b(20); put32(&b[0],up?4:5); put32(&b[4],svc); put32(&b[8],1); put32(&b[12],0); put32(&b[16],port);
    Bytes p=packet(up?4:5,0,0xfffffffeu,0xffffffffu,0xfffffffeu,b);
    CHECK(q.receive(p.data(),uint32_t(p.size()))==Ok);
  }
  // Last data packet sent: returns QMI message, checks routing.
  Bytes take(uint32_t wantPort,uint32_t wantSrc,uint16_t *msg,uint16_t *txn) {
    flush(); CHECK(!wire.empty());
    Bytes p=wire.back(); wire.clear();
    CHECK(le32(&p[4])==1); CHECK(le32(&p[12])==wantSrc); CHECK(le32(&p[24])==0); CHECK(le32(&p[28])==wantPort);
    uint32_t n=le32(&p[20]); Bytes m(p.begin()+32,p.begin()+32+n);
    CHECK(m.size()>=7); CHECK(m[0]==0); *txn=le16(&m[1]); *msg=le16(&m[3]); CHECK(le16(&m[5])==m.size()-7);
    return Bytes(m.begin()+7,m.end());
  }
  void reply(uint32_t fromPort,uint32_t toPort,uint16_t msg,uint16_t txn,const Bytes &tlvs,uint16_t result=0,uint16_t err=0) {
    Bytes m(7,0); m[0]=2; put16(&m[1],txn); put16(&m[3],msg);
    Bytes r={0x02,4,0,0,0,0,0}; put16(&r[3],result); put16(&r[5],err);
    m.insert(m.end(),r.begin(),r.end()); m.insert(m.end(),tlvs.begin(),tlvs.end());
    put16(&m[5],uint16_t(m.size()-7));
    Bytes p=packet(1,0,fromPort,1,toPort,m);
    CHECK(q.receive(p.data(),uint32_t(p.size()))==Ok);
  }
};
static Bytes tlv(uint8_t t,const Bytes &v) { Bytes o={t,0,0}; put16(&o[1],uint16_t(v.size())); o.insert(o.end(),v.begin(),v.end()); return o; }
static Bytes cat(std::initializer_list<Bytes> l) { Bytes o; for(auto &b:l) o.insert(o.end(),b.begin(),b.end()); return o; }
static Bytes str(const char *s) { return Bytes(s,s+strlen(s)); }
static Bytes cardStatus(uint8_t cardState,uint8_t appState) {
  Bytes v={0x00,0x00, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 1};          // gw primary slot0/app0
  Bytes c={cardState,0,0,0,0, uint8_t(cardState==CardPresent?1:0)};
  if(cardState==CardPresent) {
    Bytes a={AppUsim,appState,0,0,0,0,16};
    for(int i=0;i<16;i++) a.push_back(uint8_t(0xA0+i));
    Bytes pins={0,1,3,10,0,3,10};
    a.insert(a.end(),pins.begin(),pins.end()); c.insert(c.end(),a.begin(),a.end());
  }
  v.insert(v.end(),c.begin(),c.end());
  return tlv(0x10,v);
}
static Bytes readResult(const Bytes &d) { Bytes v(2); put16(&v[0],uint16_t(d.size())); v.insert(v.end(),d.begin(),d.end()); return cat({tlv(0x10,{0x90,0x00}),tlv(0x11,v)}); }

static void decoders() {
  char b[32];
  CHECK(iccidFromEf((const uint8_t *)"\x98\x93\x10\x00\x00\x21\x43\x65\x87\x09",10,b)&&!strcmp(b,"89390100001234567890"));
  CHECK(iccidFromEf((const uint8_t *)"\x98\x93\x10\x00\x00\x21\x43\x65\x87\xF9",10,b)&&!strcmp(b,"8939010000123456789"));
  CHECK(!iccidFromEf((const uint8_t *)"\x98\x93",2,b)&&!b[0]);
  CHECK(imsiFromEf((const uint8_t *)"\x08\x29\x22\x10\x55\x15\x32\x54\x76",9,b)&&!strcmp(b,"222015551234567"));
  CHECK(!imsiFromEf((const uint8_t *)"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF",9,b));
  Bytes rec(14,0xFF); Bytes tail={0x07,0x91,0x93,0x33,0x13,0x32,0x54,0x76,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  rec.insert(rec.end(),tail.begin(),tail.end());
  CHECK(msisdnFromRecord(rec.data(),uint32_t(rec.size()),b)&&!strcmp(b,"+393331234567"));
  Bytes empty(28,0xFF); CHECK(!msisdnFromRecord(empty.data(),28,b));
  CHECK(spnFromEf((const uint8_t *)"\x01TIM\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF",17,b)&&!strcmp(b,"TIM"));
  // Truncated card status must fail, never read past the TLV.
  Bytes cs=cardStatus(CardPresent,AppReady); CardStatus c={};
  for(size_t cut=3;cut<cs.size();cut++) {
    Bytes t(cs.begin(),cs.begin()+cut); put16(&t[1],uint16_t(cut-3));
    CHECK(parseCardStatus(TlvView{t.data(),uint32_t(t.size())},&c)!=Ok||cut==cs.size());
  }
  CHECK(parseCardStatus(TlvView{cs.data(),uint32_t(cs.size())},&c)==Ok);
  CHECK(c.slots==1&&c.card[0].state==CardPresent&&c.card[0].apps==1&&c.card[0].app[0].type==AppUsim);
  CHECK(c.card[0].app[0].state==AppReady&&c.card[0].app[0].aidLen==16&&c.card[0].app[0].pin1Retries==3);
  Bytes ss=cat({tlv(0x01,{1,1,1,1,1,8}),tlv(0x10,{1}),tlv(0x12,cat({{0xDE,0x00,0x01,0x00,3},str("TIM")}))});
  ServingSystem s={}; CHECK(parseServingSystem(TlvView{ss.data(),uint32_t(ss.size())},&s)==Ok);
  CHECK(s.reg==1&&s.radios==1&&s.radio[0]==8&&s.hasPlmn&&s.mcc==222&&s.mnc==1&&!strcmp(s.plmn,"TIM")&&s.hasRoaming&&s.roaming==1);
  // UIM request body layout.
  uint8_t buf[64]; TlvWriter w{buf,sizeof(buf)}; static const uint8_t mf[2]={0x00,0x3F};
  CHECK(buildRead(w,SessionCardSlot1,0x2FE2,mf,2,0,0));
  Bytes want=cat({tlv(0x01,{6,0}),tlv(0x02,{0xE2,0x2F,2,0x00,0x3F}),tlv(0x03,{0,0,0,0})});
  CHECK(w.size==want.size()&&!memcmp(buf,want.data(),want.size()));
}

static void fullSequence() {
  Rig r;
  r.server(true,SvcDms,0x20); r.server(true,SvcUim,0x21); r.server(true,SvcNas,0x22);
  r.server(true,0x45,0x30);   // WLFW still handled by the original path
  CHECK(r.q.serverPort==0x30);
  CHECK(r.s.ready(0)&&r.s.ready(1)&&r.s.ready(2));
  uint64_t now=10; uint16_t msg=0,txn=0;
  CHECK(r.s.pump(now)==Ok);
  Bytes b=r.take(0x21,0x4011,&msg,&txn); CHECK(msg==UimRegisterEvents); CHECK(b==tlv(0x01,{1,0,0,0}));
  r.reply(0x21,0x4011,msg,txn,{}); r.s.pump(++now);
  b=r.take(0x21,0x4011,&msg,&txn); CHECK(msg==UimGetCardStatus&&b.empty());
  r.reply(0x21,0x4011,msg,txn,cardStatus(CardPresent,AppReady)); r.s.pump(++now);
  CHECK(r.s.simPresent()&&r.s.appReady());
  b=r.take(0x21,0x4011,&msg,&txn); CHECK(msg==UimReadTransparent);
  CHECK(b==cat({tlv(0x01,{6,0}),tlv(0x02,{0xE2,0x2F,2,0x00,0x3F}),tlv(0x03,{0,0,0,0})}));
  r.reply(0x21,0x4011,msg,txn,readResult({0x98,0x93,0x10,0x00,0x00,0x21,0x43,0x65,0x87,0x09})); r.s.pump(++now);
  b=r.take(0x21,0x4011,&msg,&txn); CHECK(msg==UimReadTransparent);
  CHECK(b==cat({tlv(0x01,{0,0}),tlv(0x02,{0x07,0x6F,4,0x00,0x3F,0xFF,0x7F}),tlv(0x03,{0,0,0,0})}));
  r.reply(0x21,0x4011,msg,txn,readResult({0x08,0x29,0x22,0x10,0x55,0x15,0x32,0x54,0x76})); r.s.pump(++now);
  b=r.take(0x21,0x4011,&msg,&txn); CHECK(msg==UimReadTransparent&&b[8]==0x46&&b[9]==0x6F);
  r.reply(0x21,0x4011,msg,txn,{},1,0x1B); r.s.pump(++now);   // SPN missing: remote error, keep going
  b=r.take(0x21,0x4011,&msg,&txn); CHECK(msg==UimReadRecord&&b[8]==0x40&&b[9]==0x6F);
  CHECK(b==cat({tlv(0x01,{0,0}),tlv(0x02,{0x40,0x6F,4,0x00,0x3F,0xFF,0x7F}),tlv(0x03,{1,0,0,0})}));
  Bytes rec(14,0xFF); Bytes tail={0x07,0x91,0x93,0x33,0x13,0x32,0x54,0x76,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  rec.insert(rec.end(),tail.begin(),tail.end());
  r.reply(0x21,0x4011,msg,txn,readResult(rec)); r.s.pump(++now);
  b=r.take(0x20,0x4010,&msg,&txn); CHECK(msg==DmsGetIds);
  r.reply(0x20,0x4010,msg,txn,cat({tlv(0x10,str("0")),tlv(0x11,str("356789012345678"))})); r.s.pump(++now);
  b=r.take(0x20,0x4010,&msg,&txn); CHECK(msg==DmsGetRevision);
  r.reply(0x20,0x4010,msg,txn,tlv(0x01,str("MPSS.TEST.1"))); r.s.pump(++now);
  b=r.take(0x20,0x4010,&msg,&txn); CHECK(msg==DmsGetOperatingMode);
  r.reply(0x20,0x4010,msg,txn,tlv(0x01,{1})); r.s.pump(++now);
  b=r.take(0x22,0x4012,&msg,&txn); CHECK(msg==NasGetServingSystem);
  r.reply(0x22,0x4012,msg,txn,tlv(0x01,{0,0,0,2,0})); r.s.pump(++now);
  CHECK(r.s.step==SimProbe::Idle&&r.s.snapshots==1);
  CHECK(!strcmp(r.s.iccid,"89390100001234567890")&&!strcmp(r.s.imsi,"222015551234567"));
  CHECK(!r.s.spn[0]);
  CHECK(!strcmp(r.s.msisdn,"+393331234567")&&!strcmp(r.s.imei,"356789012345678"));
  CHECK(!strcmp(r.s.revision,"MPSS.TEST.1")&&r.s.operatingMode==1&&r.s.servingStatus==Ok&&r.s.serving.reg==0);
  // Card status indication triggers a rescan; a pulled card clears the SIM strings.
  Bytes ind(7,0); ind[0]=4; put16(&ind[3],UimCardStatusInd);
  Bytes cs=cardStatus(CardAbsent,0); ind.insert(ind.end(),cs.begin(),cs.end()); put16(&ind[5],uint16_t(ind.size()-7));
  Bytes p=Rig::packet(1,0,0x21,1,0x4011,ind); CHECK(r.q.receive(p.data(),uint32_t(p.size()))==Ok);
  CHECK(!r.s.simPresent());
  r.s.pump(++now); b=r.take(0x21,0x4011,&msg,&txn); CHECK(msg==UimGetCardStatus);
  r.reply(0x21,0x4011,msg,txn,cardStatus(CardAbsent,0)); r.s.pump(++now);
  CHECK(!r.s.iccid[0]&&!r.s.imsi[0]&&!r.s.msisdn[0]);
  b=r.take(0x20,0x4010,&msg,&txn); CHECK(msg==DmsGetIds);
  // Modem restart: BYE drops every endpoint; requests stop until NEW_SERVER returns.
  Bytes bye(4); put32(&bye[0],3); p=Rig::packet(3,0,0xfffffffeu,1,0xfffffffeu,bye);
  CHECK(r.q.receive(p.data(),uint32_t(p.size()))==Ok);
  CHECK(!r.s.ready(0)&&!r.s.ready(1)&&!r.s.ready(2));
  // A stale reply after the restart is ignored.
  r.reply(0x20,0x4010,msg,txn,tlv(0x11,str("000000000000000")));
  CHECK(!strcmp(r.s.imei,"356789012345678"));
}

static void pinRequired() {
  Rig r; uint64_t now=1; uint16_t msg=0,txn=0;
  r.server(true,SvcUim,0x21); r.server(true,SvcDms,0x20);
  r.s.pump(now); CHECK(r.s.step==SimProbe::WaitServices);   // NAS not up yet
  now=r.s.waitMs+1; r.s.pump(now);
  Bytes b=r.take(0x21,0x4011,&msg,&txn); CHECK(msg==UimRegisterEvents);
  r.reply(0x21,0x4011,msg,txn,{},1,0x47); r.s.pump(++now);
  b=r.take(0x21,0x4011,&msg,&txn); CHECK(msg==UimGetCardStatus);
  r.reply(0x21,0x4011,msg,txn,cardStatus(CardPresent,AppPin1Required)); r.s.pump(++now);
  b=r.take(0x21,0x4011,&msg,&txn); CHECK(msg==UimReadTransparent&&b[8]==0xE2);
  r.reply(0x21,0x4011,msg,txn,readResult({0x98,0x93,0x10,0x00,0x00,0x21,0x43,0x65,0x87,0x09})); r.s.pump(++now);
  b=r.take(0x20,0x4010,&msg,&txn); CHECK(msg==DmsGetIds);           // IMSI/SPN/MSISDN skipped: PIN
  CHECK(r.s.iccid[0]&&!r.s.imsi[0]&&!r.s.appReady());
  // Timeouts are not fatal; NAS absent ends the snapshot.
  now+=r.s.timeoutMs+1; r.s.pump(now); b=r.take(0x20,0x4010,&msg,&txn); CHECK(msg==DmsGetRevision);
  now+=r.s.timeoutMs+1; r.s.pump(now); b=r.take(0x20,0x4010,&msg,&txn); CHECK(msg==DmsGetOperatingMode);
  now+=r.s.timeoutMs+1; r.s.pump(now);
  CHECK(r.s.step==SimProbe::Idle&&r.s.snapshots==1&&r.s.operatingMode==0xFF);
}

static void qrtrClients() {
  Qrtr q{}; auto h=[](void *,uint32_t,uint32_t,const uint8_t *,uint32_t){};
  CHECK(q.addLocalClient(0,nullptr,h)==Invalid);
  CHECK(q.addLocalClient(Qrtr::LocalPort,nullptr,h)==Invalid);
  CHECK(q.addLocalService(14,1,0x4300,nullptr,h)==Ok);
  CHECK(q.addLocalClient(0x4300,nullptr,h)==Busy);
  for(uint32_t i=0;i<7;i++) CHECK(q.addLocalClient(0x4400+i,nullptr,h)==Ok);
  CHECK(q.addLocalClient(0x4500,nullptr,h)==NoMemory);
  CHECK(q.addLocalClient(0x4400,nullptr,h)==Ok);   // idempotent
}

int main() {
  decoders(); qrtrClients(); fullSequence(); pinRequired();
  printf("PASS: GuampSim.inc %u checks\n",checks);
  return 0;
}
