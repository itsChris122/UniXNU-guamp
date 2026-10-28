// SPDX-License-Identifier: BSD-2-Clause-Patent
// Wire primitives shared by SMEM, GLINK, QRTR and QMI. No unaligned loads.
#ifndef GUAMP_WIFI_WIRE_INC
#define GUAMP_WIFI_WIRE_INC
namespace guampwifi {
enum Error : int {
  Ok=0, Again=-100, Invalid=-101, Range=-102, NoMemory=-103, Protocol=-104,
  NotReady=-105, Timeout=-106, PeerReset=-107, RemoteError=-108, Unsupported=-109,
  Busy=-110, NotFound=-111, Io=-112, Quarantined=-113
};
inline uint16_t le16(const uint8_t *p) { return uint16_t(p[0]) | (uint16_t(p[1])<<8); }
inline uint32_t le32(const uint8_t *p) { return uint32_t(le16(p)) | (uint32_t(le16(p+2))<<16); }
inline uint64_t le64(const uint8_t *p) { return uint64_t(le32(p)) | (uint64_t(le32(p+4))<<32); }
inline void put16(uint8_t *p,uint16_t v) { p[0]=uint8_t(v); p[1]=uint8_t(v>>8); }
inline void put32(uint8_t *p,uint32_t v) { put16(p,uint16_t(v)); put16(p+2,uint16_t(v>>16)); }
inline void put64(uint8_t *p,uint64_t v) { put32(p,uint32_t(v)); put32(p+4,uint32_t(v>>32)); }
inline uint32_t align8(uint32_t v) { return (v+7u)&~7u; }
inline bool nameEquals(const char *a,const char *b,uint32_t cap) {
  for(uint32_t i=0;i<cap;i++) { if(a[i]!=b[i]) return false; if(!a[i]) return true; }
  return false;
}
struct Memory {
  void *ctx;
  uint32_t size;
  uint32_t (*read)(void *,uint32_t);
  void (*write)(void *,uint32_t,uint32_t);
  void (*barrier)(void *);  // release/acquire and interconnect ordering; dsb sy on ARM64
  bool valid(uint32_t at,uint32_t n) const { return at<=size && n<=size-at; }
  uint32_t r32(uint32_t at) const { return (at&3)||!valid(at,4)?0:read(ctx,at); }
  void w32(uint32_t at,uint32_t v) const { if(!(at&3)&&valid(at,4)) write(ctx,at,v); }
  void fence() const { barrier(ctx); }
  bool copyOut(uint32_t at,uint8_t *dst,uint32_t n) const {
    if(!valid(at,n)) return false;
    for(uint32_t i=0;i<n;) {
      uint32_t w=r32((at+i)&~3u), shift=(at+i)&3u;
      do { dst[i++]=uint8_t(w>>(shift++*8)); } while(i<n && shift<4);
    }
    return true;
  }
  // Each affected word must be locally owned (no remote RMW on that word).
  bool copyIn(uint32_t at,const uint8_t *src,uint32_t n) const {
    if(!valid(at,n)) return false;
    for(uint32_t i=0;i<n;) {
      uint32_t wa=(at+i)&~3u, shift=(at+i)&3u;
      uint32_t w=(shift || n-i<4)?r32(wa):0;
      do { uint32_t s=shift++*8; w=(w&~(0xffu<<s))|(uint32_t(src[i++])<<s); } while(i<n&&shift<4);
      w32(wa,w);
    }
    return true;
  }
  bool zero(uint32_t at,uint32_t n) const {
    if((at&3)||(n&3)||!valid(at,n)) return false;
    for(uint32_t i=0;i<n;i+=4) w32(at+i,0);
    return true;
  }
};
struct Allocator {
  void *ctx;
  void *(*alloc)(void *,uint32_t);
  void (*free)(void *,void *,uint32_t);
};
} // namespace guampwifi
#endif
