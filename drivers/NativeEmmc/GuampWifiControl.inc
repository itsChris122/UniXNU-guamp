// SPDX-License-Identifier: BSD-2-Clause-Patent
// Connects the APPS SMEM/SMP2P edge to GLINK -> QRTR -> WLFW. The caller must
// supply real modem/WLAN resource and MSA operations; none are assumed successful.
#ifndef GUAMP_WIFI_CONTROL_INC
#define GUAMP_WIFI_CONTROL_INC
#include "GuampSmp2p.inc"
#include "GuampGlink.inc"
#include "GuampQrtr.inc"
#include "GuampQftp.inc"
#include "GuampQmi.inc"
#include "GuampServreg.inc"
#include "GuampRmtfs.inc"
#include "GuampWlfw.inc"
#include "GuampSnocCe.inc"
#include "GuampWifiRpm.inc"
namespace guampwifi {
struct WifiControl {
  enum Stage { Off,Prepared,AwaitModem,AwaitFifo,Running,Failed };
  struct Ops {
    void *ctx;
    void (*smpKick)(void *);  // APCS base 0x0f111000, register +8, bit14
    void (*glinkKick)(void *);// same register, bit12
    Wlfw::Ops wlan;
    void (*event)(void *,Stage,int);
    // Optional one-line diagnostics: what names the gate and the meaning of a/b/c. Never called under the SMEM lock.
    void (*trace)(void *,const char *what,int e,uint32_t a,uint32_t b,uint32_t c);
    // Optional RMTFS backing (shared region + EFS partitions). When set, service 14 is registered on QRTR next to
    // QFTP/SERVREG and announced as soon as IPCRTR opens, before the modem looks up its EFS server.
    const Rmtfs::Storage *rmtfs;
  };
  // Gate timeouts after the modem PAS start (Linux: subsys err_ready 10 s; icnss has no WLFW timeout, the WLAN PD
  // only registers 0x45 after the modem has fetched wlanmdsp.mbn over QRTR TFTP).
  static constexpr uint32_t ReadyMs=10000,FifoMs=10000,ServiceMs=60000,BeatMs=5000;
  SmemArena *arena=nullptr; Allocator allocator={}; Ops ops={};
  Smp2pEdge smp={}; Glink glink={}; Qrtr qrtr={}; QftpReadOnly tftp={};
  GuampServreg servreg={}; Rmtfs rmtfs={}; Wlfw wlan={};
  Stage stage=Off; int error=0;
  uint32_t descriptor=0,outboundFifo=0,outboundBytes=0; uint16_t channel=0;
  uint64_t deadline=0,now=0,started=0,beat=0;
  bool routerStarted=false,wlanStarted=false;
  // Diagnostics: the gate being waited for or the step that failed, and transitions already reported.
  const char *gate="idle";
  bool sawInbound=false,sawNegotiated=false,sawOpenSent=false,sawRemote=false,sawServer=false;
  int prepare(SmemArena *shared,const Allocator &a,const Ops &platform,bool pasStopped) {
    int e=Ok;
    if(stage!=Off&&stage!=Failed) return Busy;
    if(!pasStopped) return Busy;
    if(!shared||!a.alloc||!a.free||!platform.smpKick||!platform.glinkKick||
       !platform.wlan.assignMsa||!platform.wlan.power||!platform.wlan.fileSize||!platform.wlan.fileRead) return Invalid;
    arena=shared; allocator=a; ops=platform; error=0; channel=0;
    routerStarted=wlanStarted=false; descriptor=outboundFifo=outboundBytes=0;
    sawInbound=sawNegotiated=sawOpenSent=sawRemote=sawServer=false;
    gate="qrtr tftp/servreg";
    e=tftp.start(this,qftpTx,qftpSize,qftpRead);if(e)return fail(e);
    e=qrtr.addLocalService(QftpReadOnly::Service,QftpReadOnly::ServiceInstance,QftpReadOnly::Port,this,qftpRx);
    if(e)return fail(e);
    e=servreg.start(this,serviceTx);if(e)return fail(e);
    e=qrtr.addLocalService(GuampServreg::Service,GuampServreg::Instance,GuampServreg::Port,this,serviceRx);
    if(e)return fail(e);
    if(ops.rmtfs) {
      gate="qrtr rmtfs";
      e=rmtfs.start(*ops.rmtfs,this,serviceTx);if(e)return fail(e);
      e=qrtr.addLocalService(Rmtfs::Service,Rmtfs::ServiceInstance,Rmtfs::Port,this,rmtfsRx);if(e)return fail(e);
    }
    smemSummary();
    // qcom_smp2p probe (before the modem boots): outbound 435 in the APPS-MPSS partition, kick bit 14.
    gate="smem 435 smp2p out"; e=smp.start(arena,ops.smpKick,ops.ctx); if(e) return fail(e);
    itemTrace(435);
    // glink_smem items. Linux (glink_probe SUBSYS_AFTER_POWERUP / remoteproc glink subdev start) allocates them only
    // once the modem is up; they live in SMEM (found already present on guamp), so allocating them here only makes a
    // size/partition problem fail before PAS writes RAM. Descriptor words and the first IRQ are written by
    // glink.start() after the modem is ready, as qcom_glink_smem_register does (*rx_pipe->tail = *tx_pipe->head = 0).
    gate="smem 478 glink descriptor"; e=arena->allocate(1,478,32,&descriptor); if(e) return fail(e);
    itemTrace(478);
    // An existing TX FIFO is reused with its own size (glink_smem: the pipe length is the item size).
    gate="smem 479 glink tx fifo"; e=arena->allocate(1,479,16384,&outboundFifo,nullptr,&outboundBytes); if(e) return fail(e);
    itemTrace(479);
    gate="prepared"; transition(Prepared); return Ok;
  }
  int modemStarted(uint64_t millis) {
    if(stage!=Prepared) return NotReady;
    now=started=beat=millis; deadline=now+ReadyMs; gate="smp2p 428 slave-kernel ready (bit 1)";
    transition(AwaitModem); return Ok;
  }
  // The board/ath10k owner supplies the exact CE, service-pipe and shadow
  // tables for this WCN3990 target after the WLFW capabilities were read.
  // Keep that handoff explicit: firmware-ready alone is not radio-enabled.
  int enableWlan(const Wlfw::Pipe *pipes,uint32_t pipeCount,
                 const Wlfw::ServicePipe *services,uint32_t serviceCount,
                 const Wlfw::Shadow *shadows,uint32_t shadowCount) {
    if(stage!=Running||!routerStarted||!wlanStarted||!qrtr.serviceReady())return NotReady;
    return wlan.enable(pipes,pipeCount,services,serviceCount,shadows,shadowCount);
  }
  int disableWlan() {
    if(stage!=Running||!routerStarted||!wlanStarted)return NotReady;
    return wlan.disable();
  }
  bool firmwareReady() const { return wlan.state==Wlfw::FirmwareReady; }
  bool radioEnabled() const { return wlan.state==Wlfw::Enabled; }
  int step(uint64_t millis) {
    now=millis;
    if(stage==Failed) return error;
    if(stage==Off||stage==Prepared) return NotReady;
    if(error) return fail(error);
    if(now>=beat+BeatMs) { beat=now; heartbeat(); }
    int e=smp.poll();
    if(smp.inPresent&&!sawInbound) {
      sawInbound=true; trace("smp2p 428 inbound present: size/valid/features",0,smp.inSize,smp.inValid,smp.inFeatures);
    }
    if(e!=Ok&&e!=Again) return fail(e);
    if(stage==AwaitModem) {
      if(!smp.ready()) return now>=deadline?fail(Timeout):Ok;
      trace("smp2p modem ready: slave/wlan/ms",0,smp.slave,smp.wlan,uint32_t(now-started));
      transition(AwaitFifo); deadline=now+FifoMs; gate="smem 480 glink rx fifo (allocated by the modem)";
    }
    if(stage==AwaitFifo) {
      uint32_t incoming=0,size=0; e=arena->find(1,480,&incoming,&size);
      if(e==NotFound) return now>=deadline?fail(Timeout):Ok;
      if(e) return fail(e);
      trace("smem 480 found: offset/bytes/ms",0,incoming,size,uint32_t(now-started));
      gate="glink start";
      GlinkPipe out={arena->mem,outboundFifo,outboundBytes,descriptor+4,descriptor,true};
      GlinkPipe in={arena->mem,incoming,size,descriptor+12,descriptor+8,false};
      e=glink.start(out,in,allocator,this,kick,channelEvent,glinkRx); if(e) return fail(e);
      deadline=now+ServiceMs; transition(Running); gate="glink version (mpss edge)";
    }
    e=tftp.step(now);if(e!=Ok&&e!=Again)return fail(e);
    rmtfs.step();
    e=glink.step(); if(e||error) return fail(e?e:error);
    if(glink.negotiated&&!sawNegotiated) {
      sawNegotiated=true; gate="glink IPCRTR open/ack";
      trace("glink negotiated: features/rx/ms",0,glink.features,glink.rxPackets,uint32_t(now-started));
    }
    if(glink.negotiated&&!channel) { e=glink.open("IPCRTR",&channel); if(e!=Again&&e) return fail(e); }
    if(channel&&!sawOpenSent) { sawOpenSent=true; trace("glink IPCRTR open sent: lcid",0,channel,0,0); }
    if(routerStarted&&qrtr.remoteKnown&&!sawRemote) {
      sawRemote=true; gate="qrtr NEW_SERVER 0x45 (WLFW) from the modem ns";
      trace("qrtr first packet from node",0,qrtr.remoteNode,0,uint32_t(now-started));
    }
    if(qrtr.serviceReady()&&!sawServer) {
      sawServer=true; gate="wlfw qmi";
      trace("qrtr WLFW 0x45 server: node/port/ms",0,qrtr.serverNode,qrtr.serverPort,uint32_t(now-started));
    }
    if(!routerStarted||!qrtr.serviceReady()) {
      if(now>=deadline) return fail(Timeout);
    }
    if(routerStarted) { e=qrtr.step(); if(e!=Again&&e) return fail(e); }
    if(wlanStarted) { e=wlan.step(now); if(e) return fail(e); }
    return Ok;
  }
  const char *where() const { return gate; }
  // Call after PAS shutdown. This releases CPU buffers only, never powers off
  // or remaps an active peripheral behind the firmware's back.
  int release(bool pasStopped) {
    if(!pasStopped) return Busy;
    qrtr.stop(); glink.stop(); routerStarted=wlanStarted=false;
    tftp.stop(); servreg.stop(); rmtfs.stop();
    stage=Off; return Ok;
  }
private:
  void transition(Stage s) { stage=s; if(ops.event) ops.event(ops.ctx,s,0); }
  void trace(const char *what,int e,uint32_t a,uint32_t b,uint32_t c) { if(ops.trace) ops.trace(ops.ctx,what,e,a,b,c); }
  // One line with the state of every layer (also printed on failure): enough to tell which gate stalled.
  void heartbeat() {
    trace(gate,error,stage,uint32_t(now-started),0);
    trace(" smp2p 428 present/valid/slave",0,smp.inPresent,smp.inValid,smp.slave);
    trace(" smp2p wlan/flags/entries",0,smp.wlan,smp.inFlags,smp.inEntries);
    trace(" glink running/negotiated/rx",0,glink.running,glink.negotiated,glink.rxPackets);
    trace(" glink lcid/router/remoteNode",0,channel,routerStarted,qrtr.remoteKnown?qrtr.remoteNode:0xffffffffu);
    trace(" wlfw node/port/state",0,qrtr.serverNode,qrtr.serverPort,wlan.state);
    if(rmtfs.ready) trace(" rmtfs requests/failures/overlay bytes",0,rmtfs.requests,rmtfs.failures,rmtfs.shadowBytes);
  }
  void smemSummary() {
    smem::Info i={};
    const int e=arena->parse(&i);
    const auto *p=smem::partitionFor(i,smem::kHostModem);
    trace("smem version/num_items/infoPresent",e,i.version,i.numItems,i.infoPresent);
    trace("smem apps-mpss partition offset/size/items",p?Ok:NotFound,p?p->offset:0,p?p->size:0,p?p->items:0);
  }
  void itemTrace(uint32_t id) {
    uint32_t at=0,size=0; const int e=arena->find(1,id,&at,&size);
    trace("smem item id/offset/bytes",e,id,at,size);
  }
  int fail(int e) {
    if(ops.trace) { trace("FAILED: stage/ms (gate on the next line)",e,stage,uint32_t(now-started),0); heartbeat(); }
    error=e; stage=Failed; wlan.peerLost();
    qrtr.stop(); glink.stop(e); routerStarted=wlanStarted=false;
    tftp.stop(); servreg.stop(); rmtfs.stop();
    error=e;
    if(ops.event) ops.event(ops.ctx,Failed,e); return e;
  }
  static void kick(void *p) { auto &c=*static_cast<WifiControl *>(p); c.ops.glinkKick(c.ops.ctx); }
  static int glinkTx(void *p,const uint8_t *v,uint32_t n) {
    auto &c=*static_cast<WifiControl *>(p); return c.glink.send(c.channel,v,n);
  }
  static int qmiTx(void *p,const uint8_t *v,uint32_t n) { return static_cast<WifiControl *>(p)->qrtr.send(v,n); }
  static int qftpTx(void *p,uint32_t sp,uint32_t dn,uint32_t dp,const uint8_t *v,uint32_t n) {
    return static_cast<WifiControl *>(p)->qrtr.sendFrom(sp,dn,dp,v,n);
  }
  static int serviceTx(void *p,uint32_t sp,uint32_t dn,uint32_t dp,const uint8_t *v,uint32_t n) {
    return static_cast<WifiControl *>(p)->qrtr.sendFrom(sp,dn,dp,v,n);
  }
  static void serviceRx(void *p,uint32_t node,uint32_t port,const uint8_t *v,uint32_t n) {
    auto &c=*static_cast<WifiControl *>(p);int e=c.servreg.receive(node,port,v,n);
    if(e!=NotFound&&e!=Unsupported&&e!=Again)c.error=e;
  }
  // RMTFS answers every request itself (QMI failure results included); a bad request never fails the transport.
  static void rmtfsRx(void *p,uint32_t node,uint32_t port,const uint8_t *v,uint32_t n) {
    (void)static_cast<WifiControl *>(p)->rmtfs.receive(node,port,v,n);
  }
  static int qftpSize(void *p,const char *name,uint32_t *n) {
    auto &c=*static_cast<WifiControl *>(p);return c.ops.wlan.fileSize(c.ops.wlan.ctx,name,n);
  }
  static int qftpRead(void *p,const char *name,uint32_t at,uint8_t *out,uint32_t n) {
    auto &c=*static_cast<WifiControl *>(p);return c.ops.wlan.fileRead(c.ops.wlan.ctx,name,at,out,n);
  }
  static void qftpRx(void *p,uint32_t node,uint32_t port,const uint8_t *v,uint32_t n) {
    auto &c=*static_cast<WifiControl *>(p);int e=c.tftp.receive(node,port,v,n,c.now);
    if(e!=NotFound&&e!=Again&&e!=Protocol&&e!=Invalid&&e!=Unsupported)c.error=e;
  }
  static int assign(void *p,const Wlfw::Region *r,uint32_t n) {
    auto &c=*static_cast<WifiControl *>(p); return c.ops.wlan.assignMsa(c.ops.wlan.ctx,r,n);
  }
  static int power(void *p,bool on) {
    auto &c=*static_cast<WifiControl *>(p); return c.ops.wlan.power(c.ops.wlan.ctx,on);
  }
  static int fileSize(void *p,const char *name,uint32_t *n) {
    auto &c=*static_cast<WifiControl *>(p); return c.ops.wlan.fileSize(c.ops.wlan.ctx,name,n);
  }
  static int fileRead(void *p,const char *name,uint32_t at,uint8_t *out,uint32_t n) {
    auto &c=*static_cast<WifiControl *>(p); return c.ops.wlan.fileRead(c.ops.wlan.ctx,name,at,out,n);
  }
  static void wlanEvent(void *p,Wlfw::State state,int error) {
    auto &c=*static_cast<WifiControl *>(p);
    if(c.ops.wlan.event) c.ops.wlan.event(c.ops.wlan.ctx,state,error);
  }
  static void channelEvent(void *p,uint16_t id,const char *name,bool up) {
    auto &c=*static_cast<WifiControl *>(p);
    if(!nameEquals(name,"IPCRTR",32)) return;
    if(!up) { c.qrtr.stop(); c.routerStarted=false; c.wlan.peerLost(); c.wlanStarted=false; return; }
    c.channel=id;
    int e=c.qrtr.start(c.allocator,&c,glinkTx,serviceEvent,qrtrRx);
    if(e) c.error=e; else c.routerStarted=true;
  }
  static void serviceEvent(void *p,bool up) {
    auto &c=*static_cast<WifiControl *>(p);
    if(!up) { c.wlan.peerLost(); c.wlanStarted=false; c.error=PeerReset; return; }
    Wlfw::Ops w={&c,qmiTx,assign,power,fileSize,fileRead,wlanEvent};
    int e=c.wlan.start(w,c.now); if(e) c.error=e; else c.wlanStarted=true;
  }
  static void glinkRx(void *p,uint16_t id,const uint8_t *v,uint32_t n) {
    auto &c=*static_cast<WifiControl *>(p);
    if(id!=c.channel||!c.routerStarted) return;
    int e=c.qrtr.receive(v,n); if(e!=NotFound&&e!=Unsupported&&e!=Again&&e) c.error=e;
  }
  static void qrtrRx(void *p,const uint8_t *v,uint32_t n) {
    auto &c=*static_cast<WifiControl *>(p);
    int e=c.wlan.receive(v,n); if(e!=NotFound&&e) c.error=e;
  }
};
} // namespace guampwifi
#endif
