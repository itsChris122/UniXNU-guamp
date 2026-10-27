// SPDX-License-Identifier: BSD-2-Clause-Patent
// Bounds/placement for the authenticated SM6115 modem image. No hardware access.
// The existing firmware verifier supplies the parsed original ELF/hash metadata.
namespace modem {
constexpr uint64_t kBase=0x4ab00000ULL,kBytes=0x6900000ULL;
constexpr unsigned kPas=4;
enum : int { kOk=0,kBadImage=-210,kRange=-211,kOverlap=-212,kNotVerified=-213 };
struct Segment {uint32_t index;uint64_t address,filesz,memsz;};
struct Plan {
  uint32_t count=0,metadataBytes=0;
  uint64_t first=0,span=0;
  bool relocatable=false;
  Segment segments[fw::kMaxSegments];
};
inline bool fits(uint64_t base,uint64_t bytes,uint64_t at,uint64_t count) {
  return bytes && base<=~0ULL-bytes && at>=base && at-base<=bytes && count<=bytes-(at-base);
}
inline bool overlap(uint64_t a,uint64_t an,uint64_t b,uint64_t bn) {
  if (!an || !bn) return false;
  if(a>~0ULL-an || b>~0ULL-bn) return true;
  return a<b+bn && b<a+an;
}
inline bool loadable(const fw::Segment &s) {
  const uint32_t type=(s.flags>>24)&7;
  return s.type==1 && s.memsz && type!=2 && type!=7;
}
inline int plan(const fw::Image &image,bool verified,uint64_t liveBase,uint64_t liveBytes,Plan *out) {
  *out=Plan();
  if (!verified) return kNotVerified;
  if (!image.count || image.count>fw::kMaxSegments || !image.headerBytes || !image.hashSeg || !image.hashSegBytes)
    return kBadImage;
  if (overlap(kBase,kBytes,liveBase,liveBytes)) return kOverlap;
  if (image.headerBytes>fw::kMdtMax || image.hashSegBytes>fw::kMdtMax-image.headerBytes) return kRange;
  out->metadataBytes=image.headerBytes+image.hashSegBytes;
  uint64_t low=~0ULL,high=0;
  for(uint32_t i=0;i<image.count;++i) {
    const fw::Segment &s=image.seg[i];if(!loadable(s))continue;
    if(s.filesz>s.memsz || s.paddr>~0ULL-s.memsz || s.paddr+s.memsz>~0ULL-4095) return kRange;
    if(s.flags&(1u<<27))out->relocatable=true;
    if(s.paddr<low)low=s.paddr;
    uint64_t end=(s.paddr+s.memsz+4095)&~4095ULL;if(end>high)high=end;
  }
  if(low==~0ULL || high<=low || high-low>kBytes || (low&4095)) return kRange;
  out->first=out->relocatable?kBase:low;out->span=high-low;
  if(!fits(kBase,kBytes,out->first,out->span) || out->metadataBytes>out->span) return kRange;
  for(uint32_t i=0;i<image.count;++i) {
    const fw::Segment &s=image.seg[i];if(!loadable(s))continue;
    uint64_t address=out->relocatable?kBase+(s.paddr-low):s.paddr;
    if(!fits(kBase,kBytes,address,s.memsz))return kRange;
    for(uint32_t j=0;j<out->count;++j)
      if(overlap(address,s.memsz,out->segments[j].address,out->segments[j].memsz))return kOverlap;
    out->segments[out->count++]={i,address,s.filesz,s.memsz};
  }
  return out->count?kOk:kBadImage;
}
} // namespace modem
