// SPDX-License-Identifier: BSD-2-Clause-Patent
// SIM/identity reader over QRTR QMI (no QMUX): UIM 0x0B card status and EF reads
// (ICCID 2FE2, IMSI 6F07, SPN 6F46, MSISDN 6F40), DMS 0x02 IMEI/revision/operating
// mode, NAS 0x03 serving system. READ ONLY: it never sends SET_OPERATING_MODE, PIN
// or write requests, so the radio state and the card are left exactly as found.
// Message/TLV layouts follow libqmi (qmi-service-uim/dms/nas.json) and 3GPP TS 31.102.
#ifndef GUAMP_SIM_INC
#define GUAMP_SIM_INC
#include "GuampWifiWire.inc"
#include "GuampQmi.inc"
#include "GuampQrtr.inc"
#include "GuampSimInfo.h"
namespace guampsim {
using namespace guampwifi;
enum : uint32_t { SvcDms=0x02,SvcNas=0x03,SvcUim=0x0B };
enum : uint16_t {
  DmsGetRevision=0x23,DmsGetIds=0x25,DmsGetOperatingMode=0x2D,
  NasGetServingSystem=0x24,
  UimReadTransparent=0x20,UimReadRecord=0x21,UimRegisterEvents=0x2E,UimGetCardStatus=0x2F,
  UimCardStatusInd=0x32
};
enum : uint8_t { SessionPrimaryGw=0,SessionCardSlot1=6 };
enum : uint8_t { CardAbsent=0,CardPresent=1,CardError=2 };
enum : uint8_t { AppSim=1,AppUsim=2 };
enum : uint8_t { AppDetected=1,AppPin1Required=2,AppPuk1Required=3,AppPerso=4,AppPin1Blocked=5,AppIllegal=6,AppReady=7 };

struct App { uint8_t type,state,persoState,pin1State,pin1Retries,puk1Retries,aidLen,aid[16]; };
struct Card { uint8_t state,upinState,errorCode,apps; App app[4]; };
struct CardStatus { uint16_t gwPrimary,gwSecondary; uint8_t slots; Card card[2]; };
struct FileRead { uint8_t sw1,sw2; bool hasCardResult; uint16_t size; uint8_t data[256]; };
struct ServingSystem { uint8_t reg,cs,ps,network,radios,radio[8]; bool hasPlmn; uint16_t mcc,mnc; char plmn[33]; bool hasRoaming; uint8_t roaming; };

struct Reader {
  const uint8_t *p; uint32_t n,at=0; bool bad=false;
  uint8_t u8() { if(at>=n){bad=true;return 0;} return p[at++]; }
  uint16_t u16() { if(n-at<2||at>n){bad=true;return 0;} uint16_t v=le16(p+at); at+=2; return v; }
  const uint8_t *bytes(uint32_t k) { if(at>n||k>n-at){bad=true;return nullptr;} const uint8_t *r=p+at; at+=k; return r; }
};
inline void copyString(char *dst,uint32_t cap,const uint8_t *src,uint32_t n) {
  if(!cap) return;
  uint32_t k=0;
  for(uint32_t i=0;i<n&&k+1<cap;i++) { uint8_t c=src[i]; if(!c) break; dst[k++]=(c>=0x20&&c<0x7f)?char(c):'?'; }
  dst[k]=0;
}
// libqmi "Card Status" TLV 0x10.
inline int parseCardStatus(const TlvView &body,CardStatus *out) {
  uint32_t n=0; const uint8_t *t=body.find(0x10,&n);
  if(!t) return NotFound;
  *out={}; Reader r{t,n};
  out->gwPrimary=r.u16(); r.u16(); out->gwSecondary=r.u16(); r.u16();
  uint8_t cards=r.u8();
  for(uint8_t c=0;c<cards&&!r.bad;c++) {
    Card tmp={}; Card &k=c<2?out->card[c]:tmp;
    k.state=r.u8(); k.upinState=r.u8(); r.u8(); r.u8(); k.errorCode=r.u8();
    uint8_t apps=r.u8(); k.apps=apps<4?apps:4;
    for(uint8_t a=0;a<apps&&!r.bad;a++) {
      App tmpa={}; App &x=a<4?k.app[a]:tmpa;
      x.type=r.u8(); x.state=r.u8(); x.persoState=r.u8(); r.u8(); r.u8(); r.u8();
      uint8_t aid=r.u8(); const uint8_t *ap=r.bytes(aid);
      if(ap) { x.aidLen=aid<16?aid:16; memcpy(x.aid,ap,x.aidLen); }
      r.u8(); x.pin1State=r.u8(); x.pin1Retries=r.u8(); x.puk1Retries=r.u8(); r.u8(); r.u8(); r.u8();
    }
  }
  if(r.bad) return Protocol;
  out->slots=cards<2?cards:2;
  return Ok;
}
// UIM read response: TLV 0x10 card result (SW1/SW2), TLV 0x11 read result (u16 length + data).
inline int parseRead(const TlvView &body,FileRead *out) {
  *out={}; uint32_t n=0;
  const uint8_t *sw=body.find(0x10,&n);
  if(sw&&n==2) { out->sw1=sw[0]; out->sw2=sw[1]; out->hasCardResult=true; }
  const uint8_t *d=body.find(0x11,&n);
  if(!d) return NotFound;
  if(n<2||le16(d)!=n-2) return Protocol;
  uint32_t k=n-2; if(k>sizeof(out->data)) k=sizeof(out->data);
  out->size=uint16_t(k); memcpy(out->data,d+2,k);
  return Ok;
}
// Swapped-nibble BCD digits (ICCID, IMSI tail, MSISDN); 0xF ends the number.
inline uint32_t bcdDigits(const uint8_t *p,uint32_t n,char *dst,uint32_t cap,uint32_t k=0,bool skipFirstLow=false) {
  if(!cap) return 0;
  for(uint32_t i=0;i<n;i++) for(int h=0;h<2;h++) {
    if(skipFirstLow&&i==0&&h==0) continue;
    uint8_t d=h?uint8_t(p[i]>>4):uint8_t(p[i]&15);
    if(d==15||k+1>=cap) { dst[k]=0; return k; }
    dst[k++]=d<10?char('0'+d):(d==10?'*':d==11?'#':'?');
  }
  dst[k]=0;
  return k;
}
inline bool iccidFromEf(const uint8_t *p,uint32_t n,char out[23]) {
  if(n<10) { out[0]=0; return false; }
  return bcdDigits(p,10,out,23)>=18;
}
// EF_IMSI (TS 31.102 4.2.2): byte0 = length, byte1 low nibble = parity, then digits.
inline bool imsiFromEf(const uint8_t *p,uint32_t n,char out[17]) {
  out[0]=0;
  if(n<9||p[0]<1||p[0]>8) return false;
  uint32_t k=bcdDigits(p+1,p[0],out,17,0,true);
  return k>=6&&k<=15;
}
// EF_MSISDN record: alpha (n-14 bytes), BCD length, TON/NPI, 10 BCD bytes, CCP, EXT.
inline bool msisdnFromRecord(const uint8_t *p,uint32_t n,char out[24]) {
  out[0]=0;
  if(n<14) return false;
  const uint8_t *t=p+n-14; uint8_t len=t[0];
  if(len==0xFF||len<2||len>11) return false;
  uint32_t k=0; if((t[1]&0x70)==0x10) out[k++]='+';
  k=bcdDigits(t+2,uint32_t(len-1),out,24,k);
  return k>0&&out[k-1]!='+';
}
// EF_SPN: display condition + 16 bytes GSM 7-bit default alphabet (unpacked), 0xFF padding.
inline bool spnFromEf(const uint8_t *p,uint32_t n,char out[17]) {
  uint32_t k=0; out[0]=0;
  if(n<2) return false;
  for(uint32_t i=1;i<n&&i<17;i++) {
    uint8_t c=p[i]; if(c==0xFF) break;
    out[k++]=c==0x00?'@':c==0x02?'$':c==0x11?'_':(c>=0x20&&c<0x7f&&c!=0x24&&c!=0x40)?char(c):'?';
  }
  out[k]=0; return k>0;
}
inline int parseServingSystem(const TlvView &body,ServingSystem *out) {
  *out={}; uint32_t n=0; const uint8_t *t=body.find(0x01,&n);
  if(!t) return NotFound;
  Reader r{t,n}; out->reg=r.u8(); out->cs=r.u8(); out->ps=r.u8(); out->network=r.u8();
  uint8_t k=r.u8(); const uint8_t *ri=r.bytes(k);
  if(r.bad) return Protocol;
  out->radios=k<8?k:8; memcpy(out->radio,ri,out->radios);
  const uint8_t *ro=body.find(0x10,&n);
  if(ro&&n==1) { out->hasRoaming=true; out->roaming=ro[0]; }
  const uint8_t *pl=body.find(0x12,&n);
  if(pl&&n>=5) {
    Reader q{pl,n}; out->mcc=q.u16(); out->mnc=q.u16(); uint8_t l=q.u8(); const uint8_t *s=q.bytes(l);
    if(!q.bad) { out->hasPlmn=true; copyString(out->plmn,sizeof(out->plmn),s,l); }
  }
  return Ok;
}
inline bool stringTlv(const TlvView &body,uint8_t tag,char *dst,uint32_t cap) {
  uint32_t n=0; const uint8_t *s=body.find(tag,&n);
  if(!s) { if(cap) dst[0]=0; return false; }
  copyString(dst,cap,s,n); return dst[0]!=0;
}
// UIM request body (libqmi): Session 0x01, File 0x02, Read Information/Record 0x03.
inline bool buildRead(TlvWriter &w,uint8_t session,uint16_t file,const uint8_t *path,uint8_t pathLen,
                      uint16_t a,uint16_t b) {
  if(pathLen>8) return false;
  uint8_t s[2]={session,0},f[3+8]={},ri[4];
  put16(f,file); f[2]=pathLen; memcpy(f+3,path,pathLen);
  put16(ri,a); put16(ri+2,b);
  return w.add(0x01,s,2)&&w.add(0x02,f,3u+pathLen)&&w.add(0x03,ri,4);
}

struct SimProbe {
  static constexpr uint32_t ClientPortBase=0x4010;
  enum Step : uint8_t { WaitServices,Register,CardStatusStep,Iccid,Imsi,Spn,Msisdn,Ids,Revision,Mode,Serving,Idle };
  struct Endpoint { SimProbe *owner; uint32_t service,node,port,localPort; bool known; Qmi qmi; };
  Qrtr *qrtr=nullptr;
  Endpoint ep[3]={};          // 0 DMS, 1 UIM, 2 NAS
  Step step=WaitServices;
  uint64_t started=0,nextScan=0;
  uint32_t waitMs=30000,rescanMs=10000,timeoutMs=8000;
  bool cardChanged=false,requestSent=false;
  void (*event)(void *,const char *what,int status)=nullptr;
  void *eventCtx=nullptr;
  // Results: strings are valid when non-empty, cardStatus/servingStatus when Ok.
  CardStatus card={}; int cardStatus=NotReady;
  char iccid[23]={},imsi[17]={},spn[17]={},msisdn[24]={},imei[17]={},meid[17]={},revision[64]={};
  uint8_t operatingMode=0xFF; ServingSystem serving={}; int servingStatus=NotReady;
  uint16_t lastQmiError[3]={};
  uint32_t snapshots=0;

  int start(Qrtr *q,uint64_t now) {
    if(!q) return Invalid;
    qrtr=q; static const uint32_t svc[3]={SvcDms,SvcUim,SvcNas};
    for(uint32_t i=0;i<3;i++) {
      Endpoint &e=ep[i]; e.owner=this; e.service=svc[i]; e.node=e.port=0; e.known=false;
      e.localPort=ClientPortBase+i; e.qmi.init(&e,transmit,indication);
      int r=q->addLocalClient(e.localPort,&e,receive);
      if(r) return r;
    }
    q->directoryCtx=this; q->directory=directory;
    step=WaitServices; started=now; nextScan=0; cardStatus=servingStatus=NotReady; requestSent=false;
    return Ok;
  }
  // IPCRTR (re)opened or closed: forget the remote endpoints, wait again for NEW_SERVER.
  void reset(uint64_t now) {
    for(auto &e:ep) { e.known=false; e.node=e.port=0; e.qmi.cancel(); e.qmi.consume(); }
    step=WaitServices; started=now; requestSent=false; cardChanged=false;
  }
  bool ready(uint32_t i) const { return ep[i].known; }
  const App *gwApp() const {
    if(cardStatus||card.slots==0||card.card[0].state!=CardPresent) return nullptr;
    uint16_t idx=card.gwPrimary; uint8_t s=uint8_t(idx>>8),a=uint8_t(idx);
    if(idx==0xFFFF||s>=card.slots||a>=card.card[s].apps) {
      for(uint8_t k=0;k<card.card[0].apps;k++)
        if(card.card[0].app[k].type==AppUsim||card.card[0].app[k].type==AppSim) return &card.card[0].app[k];
      return nullptr;
    }
    return &card.card[s].app[a];
  }
  bool simPresent() const { return !cardStatus&&card.slots&&card.card[0].state==CardPresent; }
  bool appReady() const { const App *a=gwApp(); return a&&a->state==AppReady; }
  // Drive the read sequence from the transport pump (millisecond clock).
  int pump(uint64_t now) {
    for(auto &e:ep) if(e.qmi.pending) e.qmi.step(now);
    switch(step) {
      case WaitServices:
        if(ready(1)&&ready(0)&&(ready(2)||now-started>=waitMs)) return advance(Register,now);
        if(ready(1)&&now-started>=waitMs) return advance(Register,now);
        return Ok;
      case Idle:
        if(cardChanged||(rescanMs&&now>=nextScan)) { cardChanged=false; return advance(CardStatusStep,now); }
        return Ok;
      default: return run(now);
    }
  }
private:
  static int transmit(void *p,const uint8_t *d,uint32_t n) {
    auto &e=*static_cast<Endpoint *>(p);
    if(!e.known) return NotReady;
    return e.owner->qrtr->sendFrom(e.localPort,e.node,e.port,d,n);
  }
  static void indication(void *p,uint16_t msg,const TlvView &body) {
    auto &e=*static_cast<Endpoint *>(p);
    if(e.service==SvcUim&&msg==UimCardStatusInd) {
      CardStatus c={};
      if(!parseCardStatus(body,&c)) { e.owner->card=c; e.owner->cardStatus=Ok; }
      e.owner->cardChanged=true; e.owner->note("sim uim card status indication",Ok);
    }
  }
  static void receive(void *p,uint32_t node,uint32_t port,const uint8_t *d,uint32_t n) {
    auto &e=*static_cast<Endpoint *>(p);
    if(!e.known||node!=e.node||port!=e.port) return;
    e.qmi.receive(d,n);
  }
  static void directory(void *p,bool up,uint32_t service,uint32_t,uint32_t node,uint32_t port) {
    auto &s=*static_cast<SimProbe *>(p);
    for(auto &e:s.ep) {
      if(up&&service==e.service&&!e.known) { e.node=node; e.port=port; e.known=true; s.note("sim qmi service up",int(service)); }
      else if(!up&&e.known&&e.node==node&&(service==0||(service==e.service&&port==e.port))) {
        e.known=false; e.qmi.cancel(); e.qmi.consume(); s.note("sim qmi service down",int(e.service));
      }
    }
  }
  void note(const char *what,int status) { if(event) event(eventCtx,what,status); }
  int advance(Step s,uint64_t now) { step=s; requestSent=false; return run(now); }
  int request(uint32_t i,uint16_t msg,const TlvWriter &w,uint64_t now) {
    if(!ep[i].known) return NotReady;
    int e=ep[i].qmi.begin(msg,TlvView{w.data,w.size},now,timeoutMs);
    if(!e) requestSent=true;
    return e;
  }
  static uint32_t serviceOf(Step s) { return s==Ids||s==Revision||s==Mode?0u:s==Serving?2u:1u; }
  int run(uint64_t now) {
    if(step==WaitServices||step==Idle) return Ok;
    uint32_t svc=serviceOf(step);
    if(!ep[svc].known) {
      if(step==Serving) return finish(now);
      return advance(Step(step+1),now);
    }
    if(!requestSent) {
      uint8_t buf[64]; TlvWriter w{buf,sizeof(buf)};
      static const uint8_t mf[2]={0x00,0x3F},adf[4]={0x00,0x3F,0xFF,0x7F},df[4]={0x00,0x3F,0x20,0x7F};
      const App *a=gwApp(); const uint8_t *path=a&&a->type==AppSim?df:adf;
      int e=Ok;
      switch(step) {
        case Register: { uint8_t m[4]; put32(m,1u); w.add(0x01,m,4); e=request(1,UimRegisterEvents,w,now); break; }
        case CardStatusStep: e=request(1,UimGetCardStatus,w,now); break;
        case Iccid: buildRead(w,SessionCardSlot1,0x2FE2,mf,2,0,0); e=request(1,UimReadTransparent,w,now); break;
        case Imsi:
          if(!appReady()) { imsi[0]=spn[0]=msisdn[0]=0; return advance(Ids,now); }
          buildRead(w,SessionPrimaryGw,0x6F07,path,4,0,0); e=request(1,UimReadTransparent,w,now); break;
        case Spn: buildRead(w,SessionPrimaryGw,0x6F46,path,4,0,0); e=request(1,UimReadTransparent,w,now); break;
        case Msisdn: buildRead(w,SessionPrimaryGw,0x6F40,path,4,1,0); e=request(1,UimReadRecord,w,now); break;
        case Ids: e=request(0,DmsGetIds,w,now); break;
        case Revision: e=request(0,DmsGetRevision,w,now); break;
        case Mode: e=request(0,DmsGetOperatingMode,w,now); break;
        case Serving: e=request(2,NasGetServingSystem,w,now); break;
        default: return Ok;
      }
      if(e) {
        note("sim qmi request not sent",e);
        return step==Serving?finish(now):advance(Step(step+1),now);
      }
      return Ok;
    }
    Qmi &q=ep[svc].qmi;
    if(!q.done) return Ok;
    int st=q.status; lastQmiError[svc]=q.remoteError; TlvView r=q.reply(); FileRead f={};
    switch(step) {
      case Register: note("sim uim register events",st); break;
      case CardStatusStep:
        cardStatus=st?st:parseCardStatus(r,&card); note("sim uim card status",cardStatus);
        if(!simPresent()) { iccid[0]=imsi[0]=spn[0]=msisdn[0]=0; q.consume(); return advance(Ids,now); }
        break;
      case Iccid: if(!st&&!parseRead(r,&f)) iccidFromEf(f.data,f.size,iccid); note("sim uim read iccid",st); break;
      case Imsi: if(!st&&!parseRead(r,&f)) imsiFromEf(f.data,f.size,imsi); note("sim uim read imsi",st); break;
      case Spn: if(!st&&!parseRead(r,&f)) spnFromEf(f.data,f.size,spn); note("sim uim read spn",st); break;
      case Msisdn: if(!st&&!parseRead(r,&f)) msisdnFromRecord(f.data,f.size,msisdn); note("sim uim read msisdn",st); break;
      case Ids: if(!st) { stringTlv(r,0x11,imei,sizeof(imei)); stringTlv(r,0x12,meid,sizeof(meid)); } note("sim dms ids",st); break;
      case Revision: if(!st) stringTlv(r,0x01,revision,sizeof(revision)); note("sim dms revision",st); break;
      case Mode: {
        uint32_t n=0; const uint8_t *m=st?nullptr:r.find(0x01,&n);
        if(m&&n==1) operatingMode=m[0];
        note("sim dms operating mode",st); break;
      }
      case Serving: servingStatus=st?st:parseServingSystem(r,&serving); note("sim nas serving system",servingStatus); break;
      default: break;
    }
    q.consume();
    if(step==Serving) return finish(now);
    return advance(Step(step+1),now);
  }
  int finish(uint64_t now) {
    nextScan=now+rescanMs; step=Idle; requestSent=false; snapshots++; note("sim snapshot",Ok);
    return Ok;
  }
};
inline uint8_t simState(const SimProbe &s) {
  if(s.cardStatus) return GUAMP_SIM_UNKNOWN;
  if(!s.card.slots||s.card.card[0].state==CardAbsent) return GUAMP_SIM_ABSENT;
  if(s.card.card[0].state!=CardPresent) return GUAMP_SIM_ERROR;
  const App *a=s.gwApp();
  if(!a) return GUAMP_SIM_DETECTED;
  switch(a->state) {
    case AppReady: return GUAMP_SIM_READY;
    case AppPin1Required: return GUAMP_SIM_PIN;
    case AppPuk1Required: return GUAMP_SIM_PUK;
    case AppPin1Blocked: return GUAMP_SIM_BLOCKED;
    case AppPerso: return GUAMP_SIM_PERSO;
    case AppIllegal: return GUAMP_SIM_ERROR;
    default: return GUAMP_SIM_DETECTED;
  }
}
inline void fillInfo(const SimProbe &s,GuampSimInfo *o) {
  memset(o,0,sizeof(*o));
  o->version=GUAMP_SIM_INFO_VERSION; o->snapshots=s.snapshots; o->state=simState(s);
  const App *a=s.gwApp();
  if(a) { o->appType=a->type; o->pin1Retries=a->pin1Retries; o->puk1Retries=a->puk1Retries; }
  o->operatingMode=s.operatingMode;
  o->registration=s.servingStatus?0xFF:s.serving.reg;
  if(!s.servingStatus) {
    o->csAttached=s.serving.cs; o->psAttached=s.serving.ps; o->roaming=s.serving.hasRoaming?s.serving.roaming:0xFF;
    o->radio=s.serving.radios?s.serving.radio[0]:0; o->hasPlmn=s.serving.hasPlmn;
    o->mcc=s.serving.mcc; o->mnc=s.serving.mnc; memcpy(o->plmn,s.serving.plmn,sizeof(o->plmn));
  } else o->roaming=0xFF;
  memcpy(o->iccid,s.iccid,sizeof(o->iccid)); memcpy(o->imsi,s.imsi,sizeof(o->imsi));
  memcpy(o->spn,s.spn,sizeof(o->spn)); memcpy(o->msisdn,s.msisdn,sizeof(o->msisdn));
  memcpy(o->imei,s.imei,sizeof(o->imei)); memcpy(o->meid,s.meid,sizeof(o->meid));
  memcpy(o->revision,s.revision,sizeof(o->revision));
}
// guamp-sim-fake=1: fixed test values (3GPP test PLMN 001/01), never real identifiers.
inline void fakeInfo(GuampSimInfo *o) {
  memset(o,0,sizeof(*o));
  o->version=GUAMP_SIM_INFO_VERSION; o->snapshots=1; o->fake=1; o->state=GUAMP_SIM_READY; o->appType=AppUsim;
  o->pin1Retries=3; o->puk1Retries=10; o->operatingMode=0; o->registration=1; o->csAttached=1; o->psAttached=1;
  o->roaming=0; o->radio=8; o->hasPlmn=1; o->mcc=1; o->mnc=1;
  static const char iccid[]="8900100000000000001",imsi[]="001010000000001",spn[]="GUAMP TEST",
    msisdn[]="+10000000001",imei[]="000000000000000",rev[]="FAKE.MPSS",plmn[]="Test PLMN";
  memcpy(o->iccid,iccid,sizeof(iccid)); memcpy(o->imsi,imsi,sizeof(imsi)); memcpy(o->spn,spn,sizeof(spn));
  memcpy(o->msisdn,msisdn,sizeof(msisdn)); memcpy(o->imei,imei,sizeof(imei)); memcpy(o->revision,rev,sizeof(rev));
  memcpy(o->plmn,plmn,sizeof(plmn));
}
} // namespace guampsim
#endif
