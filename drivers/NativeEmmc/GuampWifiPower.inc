// SPDX-License-Identifier: BSD-2-Clause-Patent
// Boot-lifetime RPM power leases for SM6115. Parameters from bengal.dtsi,
// bengal-regulator.dtsi, pm6125-rpm-regulator.dtsi and clk-smd-rpm.c.
// Never treats a queued vote as applied. Shared APPS power votes are retained
// until reboot in this first port, rather than lowering unknown firmware votes.
#ifndef GUAMP_WIFI_POWER_INC
#define GUAMP_WIFI_POWER_INC
#include "GuampWifiRpm.inc"
namespace guampwifi {
struct WifiPower {
  struct Vote { uint32_t type,id,key,value,state; };
  Rpm *rpm=nullptr; Vote plan[24]={}; uint32_t count=0,index=0;
  bool busy=false,modem=false,wlan=false,proxyHeld=false,wlanHeld=false;
  int error=0;
  static constexpr uint32_t word(char a,char b,char c=0,char d=0) {
    return uint8_t(a)|(uint32_t(uint8_t(b))<<8)|(uint32_t(uint8_t(c))<<16)|(uint32_t(uint8_t(d))<<24);
  }
  int begin(Rpm *transport,bool forWlan) {
    if(busy) return Busy;
    if(!transport||!transport->online()) return NotReady;
    if(forWlan&&!proxyHeld) return NotReady;
    if(forWlan?wlanHeld:proxyHeld) { wlan=forWlan?wlanHeld:wlan; modem=proxyHeld; return Ok; }
    rpm=transport; count=index=0; error=0;
    if(!forWlan) {
      pair(word('c','l','k','0'),0,word('E','n','a','b'),1); // shared XO branch
      pair(word('r','w','c','x'),0,word('v','l','v','l'),384); // CX TURBO, as PIL DTS
    } else {
      static const uint32_t rail[4]={8,16,17,23};
      // Intersect ICNSS requests with the actual PM6125 regulator ranges.
      // Match ICNSS's effective DT-configured voltages. In particular,
      // vdd-3.3-ch0 is capped at 3.3 V by qcom,vdd-3.3-ch0-config; the
      // driver's default 3.312 V is overridden by that board property.
      static const uint32_t uv[4]={640000,1800000,1304000,3300000};
      for(uint32_t i=0;i<4;i++) {
        pair(word('l','d','o','a'),rail[i],word('u','v'),uv[i]);
        pair(word('l','d','o','a'),rail[i],word('s','w','e','n'),1);
      }
    }
    busy=true; return Ok;
  }
  int step(uint64_t now) {
    if(!rpm) return NotReady;
    int e=rpm->step(now); if(e) return fail(e);
    if(!busy) return error;
    if(rpm->done) {
      e=rpm->error; rpm->consume(); if(e) return fail(e);
      index++;
    }
    if(index==count) {
      busy=false;
      if(!proxyHeld) { proxyHeld=modem=true; } else wlanHeld=wlan=true;
      return Ok;
    }
    if(!rpm->waiting) {
      const auto &v=plan[index]; e=rpm->vote(v.type,v.id,v.key,v.value,v.state,now);
      if(e) return fail(e);
    }
    return Ok;
  }
  // Releases logical WLAN access after its confirmed OFF acknowledgement.
  // Physical shared votes remain held for this boot; no hidden rail power-off.
  int releaseWlan(bool firmwareStopped) {
    if(!firmwareStopped||busy) return Busy;
    wlan=false; return Ok;
  }
private:
  void pair(uint32_t type,uint32_t id,uint32_t key,uint32_t value) {
    plan[count++]={type,id,key,value,0}; plan[count++]={type,id,key,value,1};
  }
  int fail(int e) { busy=false; error=e; return e; }
};
} // namespace guampwifi
#endif
