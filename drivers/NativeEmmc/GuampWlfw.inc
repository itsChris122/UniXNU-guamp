// SPDX-License-Identifier: BSD-2-Clause-Patent
// WLFW service 0x45/v1, MSA + board data + WLAN configuration state machine.
// Motorola ICNSS and Linux v6.6 ath10k QMI wire schemas are the references.
// Firmware-ready/WLAN-enabled are NOT association or Internet connectivity.
#ifndef GUAMP_WLFW_INC
#define GUAMP_WLFW_INC
#include "GuampQmi.inc"
namespace guampwifi {
struct Wlfw {
  enum State { Off,Register,MsaInfo,MsaAssign,MsaReady,Capabilities,WaitMsa,
    Board,Calibration,WaitFirmware,FirmwareReady,Configure,Mode,Enabled,Disable,Failed };
  struct Region { uint64_t address; uint32_t size; bool secure; };
  struct Capability { bool chipValid,boardValid,socValid; uint32_t chip,family,board,soc; };
  struct Pipe { uint32_t number,direction,entries,bytes,flags; };
  struct ServicePipe { uint32_t service,direction,pipe; };
  struct Shadow { uint16_t id,offset; };
  struct Ops {
    void *ctx;
    int (*send)(void *,const uint8_t *,uint32_t);
    int (*assignMsa)(void *,const Region *,uint32_t); // actual SCM permissions, no success stub
    int (*power)(void *,bool);                       // actual board regulator/clock votes
    int (*fileSize)(void *,const char *,uint32_t *);
    int (*fileRead)(void *,const char *,uint32_t,uint8_t *,uint32_t);
    void (*event)(void *,State,int);
  };
  static constexpr uint64_t MsaBase=0x51900000;
  static constexpr uint32_t MsaBytes=0x100000;
  Qmi qmi={}; Ops ops={}; State state=Off;
  Capability capability={}; Region regions[2]={}; uint32_t regionCount=0;
  uint8_t work[6300]={},boardChunk[6146]={}; uint32_t workSize=0;
  uint32_t boardBytes=0,boardOffset=0,boardSegment=0,lastBoardChunk=0;
  uint64_t waitUntil=0,assignAfter=0;
  bool issued=false,msaInd=false,fwInd=false,alreadyRunning=false,powered=false,assigned=false;
  int error=0;
  static constexpr const char *BoardFile="/image/bdwlan_guamp_ipa.bin";
  static int send(void *ctx,const uint8_t *p,uint32_t n) {
    auto *w=static_cast<Wlfw *>(ctx); return w->ops.send(w->ops.ctx,p,n);
  }
  static void indication(void *ctx,uint16_t msg,const TlvView &) {
    auto *w=static_cast<Wlfw *>(ctx);
    if(msg==0x21) w->fwInd=true;
    if(msg==0x2b) w->msaInd=true;
    if(msg==0x39) w->fail(PeerReset); // firmware recovery/rejuvenation requires transport restart
  }
  int start(const Ops &platform,uint64_t now) {
    if(state!=Off&&state!=Failed) return Busy;
    if(!platform.send||!platform.assignMsa||!platform.power||!platform.fileRead||!platform.fileSize) return Invalid;
    ops=platform; qmi.init(this,send,indication); capability={}; regionCount=0;
    boardBytes=boardOffset=boardSegment=lastBoardChunk=0;
    waitUntil=now+30000; assignAfter=0; msaInd=fwInd=alreadyRunning=powered=assigned=issued=false; error=0;
    transition(Register); return Ok;
  }
  void peerLost() { qmi.cancel(PeerReset); fail(PeerReset); }
  int receive(const uint8_t *p,uint32_t n) { return qmi.receive(p,n); }
  int step(uint64_t now) {
    if(state==Off) return NotReady;
    if(state==Failed) return error;
    int e=qmi.step(now);
    if(e && !qmi.done) return fail(e);
    if(qmi.done) {
      if(qmi.status) return fail(qmi.status);
      e=complete(now); qmi.consume(); issued=false; if(e) return fail(e);
    }
    if(qmi.pending||state==FirmwareReady||state==Enabled) return Ok;
    TlvWriter b={work,sizeof(work)};
    uint16_t msg=0;
    switch(state) {
      case Register:
        if(!b.u8(0x10,1)||!b.u8(0x13,1)||!b.u8(0x14,1)||!b.u32(0x15,0x4b4e454c)) return fail(Range);
        msg=0x20; break;
      case MsaInfo:
        if(!b.u64(1,MsaBase)||!b.u32(2,MsaBytes)) return fail(Range);
        msg=0x2d; break;
      case MsaAssign:
        if(now<assignAfter) return Ok;
        e=ops.assignMsa(ops.ctx,regions,regionCount); if(e) return fail(e);
        assigned=true; transition(MsaReady); msg=0x2e; break;
      case MsaReady: msg=0x2e; break;
      case Capabilities: msg=0x24; break;
      case WaitMsa:
        if(!msaInd) return now>=waitUntil?fail(Timeout):Ok;
        e=ops.fileSize(ops.ctx,BoardFile,&boardBytes);
        if(e||!boardBytes||boardBytes>1024*1024) return fail(e?e:Range);
        transition(Board);
        return step(now);
      case Board: {
        uint32_t n=boardBytes-boardOffset; if(n>6144) n=6144;
        if(!n) return fail(Protocol);
        // QMI's variable data array has a u16 length inside its TLV.
        uint8_t *data=boardChunk;
        put16(data,uint16_t(n)); e=ops.fileRead(ops.ctx,BoardFile,boardOffset,data+2,n); if(e) return fail(e);
        if(!b.u8(1,1)||!b.u32(0x10,0)||!b.u32(0x11,boardBytes)||!b.u32(0x12,boardSegment)||
           !b.add(0x13,data,n+2)||!b.u8(0x14,boardOffset+n==boardBytes?1u:0u)) return fail(Range);
        lastBoardChunk=n; msg=0x25; break;
      }
      case Calibration: {
        uint8_t empty=0; if(!b.add(1,&empty,1)) return fail(Range);
        // No invented calibration: tell firmware that no cached calibration IDs are held.
        msg=0x26; break;
      }
      case WaitFirmware:
        if(!fwInd) return now>=waitUntil?fail(Timeout):Ok;
        transition(FirmwareReady); return Ok;
      case Configure:
        b.size=workSize; msg=0x23; break;
      case Mode:
        if(!b.u32(1,0)||!b.u8(0x10,0)) return fail(Range); // mission mode, hw_debug disabled
        msg=0x22; break;
      case Disable:
        if(!b.u32(1,4)||!b.u8(0x10,0)) return fail(Range);
        msg=0x22; break;
      default: return Protocol;
    }
    if(!msg) return Ok;
    e=qmi.begin(msg,{b.data,b.size},now); issued=!e;
    return e?fail(e):Ok;
  }
  int enable(const Pipe *pipes,uint32_t np,const ServicePipe *services,uint32_t ns,
             const Shadow *shadows,uint32_t nr) {
    if(state!=FirmwareReady||qmi.pending||qmi.done) return NotReady;
    if(!pipes||!services||np>12||!np||ns>24||!ns||nr>24||(!shadows&&nr)) return Invalid;
    // Values are caller-provided CE configuration, not guessed register addresses.
    uint8_t table[1+24*20]={}; TlvWriter b={work,sizeof(work)};
    table[0]=uint8_t(np);
    uint32_t seen=0;
    for(uint32_t i=0;i<np;i++) {
      const auto &p=pipes[i];
      if(p.number>=12||(seen&(1u<<p.number))||p.direction>4||p.entries>8192||p.bytes>65536) return Invalid;
      seen|=1u<<p.number; uint8_t *t=table+1+20*i;
      put32(t,p.number); put32(t+4,p.direction); put32(t+8,p.entries); put32(t+12,p.bytes); put32(t+16,p.flags);
    }
    if(!b.add(0x11,table,1+20*np)) return Range;
    table[0]=uint8_t(ns);
    for(uint32_t i=0;i<ns;i++) {
      if(services[i].pipe>=12||services[i].direction>3) return Invalid;
      uint8_t *t=table+1+12*i;
      put32(t,services[i].service); put32(t+4,services[i].direction); put32(t+8,services[i].pipe);
    }
    if(!b.add(0x12,table,1+12*ns)) return Range;
    table[0]=uint8_t(nr);
    for(uint32_t i=0;i<nr;i++) { put16(table+1+4*i,shadows[i].id); put16(table+3+4*i,shadows[i].offset); }
    if(!b.add(0x13,table,1+4*nr)) return Range;
    workSize=b.size; transition(Configure); return Ok;
  }
  int disable() {
    if(state==FirmwareReady) return Ok;
    if(state!=Enabled||qmi.pending||qmi.done) return NotReady;
    transition(Disable); return Ok;
  }
private:
  void transition(State s) { state=s; if(ops.event) ops.event(ops.ctx,s,0); }
  int fail(int e) {
    error=e; qmi.cancel(e); state=Failed;
    // Do not revoke MSA or switch off live hardware behind the firmware. The
    // owner must first stop PAS/WLAN and then release its board resources.
    if(ops.event) ops.event(ops.ctx,Failed,e);
    return e;
  }
  int complete(uint64_t now) {
    auto reply=qmi.reply(); uint32_t n=0; const uint8_t *p=nullptr;
    switch(state) {
      case Register:
        p=reply.find(0x10,&n); if(p&&n!=8) return Protocol;
        alreadyRunning=p&&(le64(p)&2);
        // Even an already-ready service must return its actual chip identity.
        transition(alreadyRunning?Capabilities:MsaInfo); break;
      case MsaInfo: {
        p=reply.find(3,&n); if(!p||!n||p[0]<1||p[0]>2||n!=1+13u*p[0]) return Protocol;
        regionCount=p[0];
        for(uint32_t i=0;i<regionCount;i++) {
          const uint8_t *r=p+1+13*i; uint64_t at=le64(r); uint32_t bytes=le32(r+8);
          if(!bytes||at<MsaBase||at-MsaBase>=MsaBytes||bytes>MsaBytes-(at-MsaBase)||r[12]>1) return Range;
          regions[i]={at,bytes,r[12]!=0};
          for(uint32_t j=0;j<i;j++) if(at<regions[j].address+regions[j].size&&regions[j].address<at+bytes) return Range;
        }
        assignAfter=now+20; transition(MsaAssign); break;
      }
      case MsaReady: transition(Capabilities); break;
      case Capabilities:
        p=reply.find(0x10,&n);
        if(p) { if(n!=8) return Protocol; capability.chipValid=true; capability.chip=le32(p); capability.family=le32(p+4); }
        p=reply.find(0x11,&n);
        if(p) { if(n!=4) return Protocol; capability.boardValid=true; capability.board=le32(p); }
        p=reply.find(0x12,&n);
        if(p) { if(n!=4) return Protocol; capability.socValid=true; capability.soc=le32(p); }
        if(!capability.chipValid) return Protocol;
        if(!powered) { int e=ops.power(ops.ctx,true); if(e) return e; powered=true; }
        waitUntil=now+30000; transition(alreadyRunning?FirmwareReady:WaitMsa); break;
      case Board:
        if(!lastBoardChunk||lastBoardChunk>boardBytes-boardOffset) return Protocol;
        boardOffset+=lastBoardChunk; boardSegment++;
        if(boardOffset==boardBytes) transition(Calibration);
        break;
      case Calibration: waitUntil=now+30000; transition(WaitFirmware); break;
      case Configure: transition(Mode); break;
      case Mode: transition(Enabled); break;
      case Disable: transition(FirmwareReady); break;
      default: return Protocol;
    }
    return Ok;
  }
};
} // namespace guampwifi
#endif
