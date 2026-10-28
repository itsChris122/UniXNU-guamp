// Compile/codegen check only. No XNU kernel build, no IOKit SDK required.
#include "kernel_stub.h"
#include "../GuampSmem.inc"
#include "../GuampSmp2p.inc"
#include "../GuampGlink.inc"
#include "../GuampQrtr.inc"
#include "../GuampWlfw.inc"
#include "../GuampSnocCe.inc"
#include "../GuampWifiControl.inc"
using namespace guampwifi;
int transportCodegen(Smp2pEdge *s,Glink *g,Qrtr *r,Wlfw *w,SnocCe *ce,uint64_t now,const uint8_t *packet,uint32_t bytes) {
  int e=s->poll(); if(e) return e;
  e=g->step(); if(e) return e;
  e=r->step(); if(e) return e;
  e=w->step(now); if(e) return e;
  e=r->receive(packet,bytes); if(e) return e;
  return ce->poll();
}
int controlCodegen(WifiControl *c,SmemArena *a,const Allocator &allocator,const WifiControl::Ops &ops,uint64_t now) {
  int e=c->prepare(a,allocator,ops,true); if(e) return e;
  e=c->modemStarted(now); if(e) return e;
  return c->step(now+1);
}
