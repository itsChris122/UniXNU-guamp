// SPDX-License-Identifier: BSD-2-Clause-Patent
// Normal SCM memory-protection assignment for the reserved WLAN MSA only.
// Payload ABI: Motorola secure_buffer.c/scm.c; source HLOS3 -> MSS_MSA15,
// WLAN24 and (nonsecure segments only) WLAN_CE25, RW. No signature bypass.
#ifndef GUAMP_MSA_INC
#define GUAMP_MSA_INC
#include "GuampWifiWire.inc"
namespace guampwifi {
struct MsaAssignment {
  Memory payload={}; uint64_t physical=0;
  scm::Transport *transport=nullptr;
  uint32_t assigned=0;
  bool validPayload() const {
    return transport&&physical&&!(physical&63)&&physical<0x100000000ull&&
           payload.valid(0,512)&&payload.read&&payload.write&&payload.barrier&&
           !(physical<0x51a00000ull&&physical+512>0x51900000ull);
  }
  int assign(uint64_t address,uint32_t bytes,bool secure) {
    if(!validPayload()) return Invalid;
    if(!bytes||address<0x51900000ull||address>=0x51a00000ull||bytes>0x51a00000ull-address||
       (address&4095)||(bytes&4095)) return Range;
    auto available=transport->available(0xc,0x16);
    if(available.transport||available.value!=1) return Unsupported;
    const uint32_t count=secure?2u:3u;
    payload.zero(0,512);
    uint8_t region[16]; put64(region,address); put64(region+8,bytes); payload.copyIn(0,region,16);
    payload.w32(64,3); // source VMID_HLOS
    for(uint32_t i=0;i<count;i++) {
      payload.w32(128+24*i,i==0?15u:(i==1?24u:25u));
      payload.w32(132+24*i,6); // PERM_READ(4) | PERM_WRITE(2)
    }
    uint8_t extra[56]={};
    if(transport->wide) {
      put64(extra,4); put64(extra+8,physical+128); put64(extra+16,count*24); put64(extra+24,0);
    } else {
      put32(extra,4); put32(extra+4,uint32_t(physical+128)); put32(extra+8,count*24); put32(extra+12,0);
    }
    payload.copyIn(256,extra,sizeof(extra)); payload.fence();
    auto result=transport->callExtended(0xc,0x16,0x1117,physical,16,physical+64,physical+256);
    payload.fence();
    if(result.transport||result.value) return RemoteError;
    assigned++; return Ok;
  }
};
} // namespace guampwifi
#endif
