/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * Composes the physical board callbacks with the WLFW/ath10k CE bridge.
 * The caller owns this binding for the full gw_snoc_start() lifetime.
 */
#ifndef GUAMP_WLFW_BOARD_H
#define GUAMP_WLFW_BOARD_H
#include "guamp_wlfw_bridge.h"

namespace guampwifi {
struct WlfwBoardBinding {
  gw_snoc_board physical={};
  Ath10kWlfwBridge bridge={};

  bool bind(const gw_snoc_board *source,WifiControl *control,
            void *pumpContext,uint64_t (*now)(void *),
            void (*sleep)(void *,uint32_t),void (*poll)(void *),
            gw_snoc_board *out) {
    if(!source||!control||!now||!sleep||!poll||!out||!source->power||
       !source->ready||!source->registers||!source->register_bytes) return false;
    physical=*source;
    bridge.control=control;bridge.ctx=pumpContext;
    bridge.nowMs=now;bridge.sleepMs=sleep;bridge.poll=poll;
    *out=physical;out->ctx=this;out->power=power;out->ready=ready;
    out->wlan_enable=wlanEnable;out->wlan_disable=wlanDisable;
    return true;
  }

private:
  static int power(void *opaque,int on) {
    auto *b=static_cast<WlfwBoardBinding *>(opaque);
    return b&&b->physical.power?b->physical.power(b->physical.ctx,on):Invalid;
  }
  static int ready(void *opaque) {
    auto *b=static_cast<WlfwBoardBinding *>(opaque);
    // Cold WLFW bootstrap produces FirmwareReady before ath10k asks for CE
    // configuration/mission mode. The physical provider must finish that
    // bootstrap and establish MSA/SMMU/DMA before allowing the probe.
    return b&&b->physical.ready?b->physical.ready(b->physical.ctx):0;
  }
  static int wlanEnable(void *opaque,const gw_ce_pipe *pipes,unsigned np,
                       const gw_ce_service *services,unsigned ns,
                       const gw_ce_shadow *shadows,unsigned nr) {
    auto *b=static_cast<WlfwBoardBinding *>(opaque);
    return b?Ath10kWlfwBridge::enable(&b->bridge,pipes,np,services,ns,shadows,nr):Invalid;
  }
  static int wlanDisable(void *opaque) {
    auto *b=static_cast<WlfwBoardBinding *>(opaque);
    return b?Ath10kWlfwBridge::disable(&b->bridge):Invalid;
  }
};
} // namespace guampwifi
#endif
