/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * Connects ath10k's WCN3990 board callback to the guamp WLFW client.
 * Uses the maintained NativeEmmc transport sources, not checkpoint copies.
 */
#ifndef GUAMP_WLFW_BRIDGE_H
#define GUAMP_WLFW_BRIDGE_H
#include "../ath10k-port/gw_snoc_abi.h"
#include "GuampSmem.inc"
#include "GuampWifiControl.inc"

namespace guampwifi {
struct Ath10kWlfwBridge {
  WifiControl *control=nullptr;
  void *ctx=nullptr;
  uint64_t (*nowMs)(void *)=nullptr;
  void (*sleepMs)(void *,uint32_t)=nullptr;
  void (*poll)(void *)=nullptr;
  uint32_t timeoutMs=30000;

  static int enable(void *opaque,const gw_ce_pipe *pipes,unsigned np,
                    const gw_ce_service *services,unsigned ns,
                    const gw_ce_shadow *shadows,unsigned nr) {
    auto *b=static_cast<Ath10kWlfwBridge *>(opaque);
    if(!b||!b->control||!b->nowMs||!b->sleepMs||!b->poll||
       !pipes||!np||np>12||!services||!ns||ns>24||nr>24||(!shadows&&nr))return Invalid;
    Wlfw::Pipe p[12]={}; Wlfw::ServicePipe s[24]={}; Wlfw::Shadow r[24]={};
    for(unsigned i=0;i<np;i++)p[i]={pipes[i].number,pipes[i].direction,pipes[i].entries,pipes[i].bytes,pipes[i].flags};
    for(unsigned i=0;i<ns;i++)s[i]={services[i].service,services[i].direction,services[i].pipe};
    for(unsigned i=0;i<nr;i++)r[i]={shadows[i].engine,shadows[i].offset};
    // WLFW starts firmware independently of ath10k's CE configuration. Wait
    // for BDF/calibration/FW_READY first; enableWlan() accepts only that state.
    if(b->control->radioEnabled())return Busy;
    int e=b->until(false);if(e)return e;
    e=b->control->enableWlan(p,np,s,ns,nr?r:nullptr,nr);if(e)return e;
    return b->until(true);
  }

  static int disable(void *opaque) {
    auto *b=static_cast<Ath10kWlfwBridge *>(opaque);
    if(!b||!b->control||!b->nowMs||!b->sleepMs||!b->poll)return Invalid;
    int e=b->control->disableWlan();if(e)return e;
    return b->until(false);
  }

private:
  int until(bool enabled) {
    uint64_t start=nowMs(ctx),deadline=start+timeoutMs;
    if(deadline<start)return Range;
    for(;;) {
      if(enabled?control->radioEnabled():control->firmwareReady())return Ok;
      if(control->stage==WifiControl::Failed)return control->error?control->error:RemoteError;
      uint64_t now=nowMs(ctx);if(now>=deadline)return Timeout;
      int e=control->step(now);if(e!=Ok&&e!=Again)return e;
      poll(ctx);
      sleepMs(ctx,1);
    }
  }
};
} // namespace guampwifi
#endif
