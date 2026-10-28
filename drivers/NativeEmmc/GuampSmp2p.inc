// SPDX-License-Identifier: BSD-2-Clause-Patent
// Modem SMP2P APPS edge, including feature negotiation and restart ACK.
#ifndef GUAMP_SMP2P_INC
#define GUAMP_SMP2P_INC
#include "GuampSmemWrite.inc"
namespace guampwifi {
struct Smp2pEdge {
  SmemArena *arena=nullptr;
  void *kickCtx=nullptr;
  void (*kick)(void *)=nullptr;
  uint32_t outbound=0;
  uint32_t slave=0,wlan=0;
  bool negotiated=false,ssr=false,ack=false,seenReady=false;
  int start(SmemArena *a,void (*notify)(void *),void *ctx) {
    if(!a||!notify) return Invalid;
    arena=a; kick=notify; kickCtx=ctx;
    int e=arena->allocate(1,435,340,&outbound); if(e) return e;
    // Caller must establish that PAS modem is stopped before resetting APPS state.
    arena->mem.zero(outbound,340);
    arena->mem.w32(outbound,smem::kSmp2pMagic);
    arena->mem.w32(outbound+8,1u<<16); // local=APPS0, remote=MODEM1
    arena->mem.w32(outbound+12,16);
    arena->mem.copyIn(outbound+20,reinterpret_cast<const uint8_t *>("master-kernel"),14);
    arena->mem.w32(outbound+12,16u|(1u<<16));
    arena->mem.fence();
    arena->mem.w32(outbound+4,1u|(1u<<8)); // version publication last
    arena->mem.fence(); kick(kickCtx);
    slave=wlan=0; negotiated=ssr=ack=seenReady=false;
    return Ok;
  }
  int poll() {
    if(!arena||!outbound) return NotReady;
    uint32_t at=0,size=0; int e=arena->find(1,428,&at,&size); if(e) return e==NotFound?Again:e;
    smem::Smp2p in={}; arena->mem.fence();
    smem::readSmp2p(arena->view(),at,size,"slave-kernel",&in);
    if(!in.valid||in.localPid!=1||in.remotePid!=0) return Protocol;
    if(!negotiated) {
      ssr=(in.features&1)!=0;
      arena->mem.w32(outbound+4,1u|(uint32_t(ssr)<<8));
      arena->mem.fence(); kick(kickCtx); negotiated=true;
    }
    bool restarted=ssr&&bool(in.flags&1)!=ack;
    if(restarted) {
      ack=bool(in.flags&1);
      uint32_t flags=arena->mem.r32(outbound+16);
      arena->mem.w32(outbound+16,(flags&~2u)|(uint32_t(ack)<<1));
      arena->mem.fence(); kick(kickCtx);
    }
    slave=in.hasEntry?in.value:0; wlan=0;
    for(uint32_t n=0;n<in.validEntries;n++) if(nameEquals(in.names[n],"wlan",17)) wlan=in.values[n];
    if((slave&1)||(wlan&3)||restarted||(seenReady&&!(slave&2))) return PeerReset;
    if(slave&2) seenReady=true;
    return in.hasEntry?Ok:Again;
  }
  int update(uint32_t mask,uint32_t value) {
    if(!arena||!outbound) return NotReady;
    uint32_t old=arena->mem.r32(outbound+36);
    arena->mem.w32(outbound+36,(old&~mask)|(value&mask));
    arena->mem.fence(); kick(kickCtx); return Ok;
  }
  bool ready() const { return seenReady&&(slave&2)&&!(slave&1); }
};
} // namespace guampwifi
#endif
