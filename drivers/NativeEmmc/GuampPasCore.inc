// SPDX-License-Identifier: BSD-2-Clause-Patent
// Peripheral-independent placement for an authenticated Qualcomm PAS image (TrustZone peripheral authentication
// service), generalised from wifi/driver-modem/GuampModemCore.inc: the carveout, the PAS id and the firmware name are
// parameters (pas::Peripheral) instead of modem constants. No hardware access; host-tested by tests/pas_host.cpp.
// Placement as pil_setup_region()/pil_init_mmap() of the guamp kernel (drivers/soc/qcom/peripheral-loader.c):
//   relocatable image (every loadable segment has p_flags bit 27): region_start = carveout base, a segment goes to
//     base + (p_paddr - lowest p_paddr), the region ends at the highest p_paddr + p_memsz rounded up to 4 KiB;
//   fixed image: region = [lowest p_paddr, highest end rounded to 4 KiB], which must lie inside the carveout.
// A segment is loadable as segment_is_loadable(): PT_LOAD, p_memsz != 0, not the hash segment (p_flags 26:24 == 2)
// and not 7 (the ELF/program header segment). Mixed relocatable/fixed images are refused (not seen on guamp; Linux
// would place the two groups separately). Range, overlap and XNU-RAM checks only refuse: nothing is adjusted.
#ifndef GUAMP_PAS_CORE_INC
#define GUAMP_PAS_CORE_INC
namespace pas {
struct Peripheral {
  const char *tag;        // log prefix, e.g. "GuampAdsp"
  unsigned id;            // qcom,pas-id
  uint64_t base, bytes;   // memory-region carveout (reserved-memory, not XNU RAM)
  const char *dir;        // directory of the split firmware on the ext4 partition, e.g. "/image"
  const char *firmware;   // qcom,firmware-name: <firmware>.mdt and <firmware>.bNN
};
enum : int { kOk=0,kBadImage=-210,kRange=-211,kOverlap=-212,kNotVerified=-213,kMixed=-214 };
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
// metadataBytes: what goes to region_start before INIT_IMAGE (the original .mdt, see GuampPas.inc)
inline int plan(const Peripheral &p,const fw::Image &image,bool verified,uint32_t metadataBytes,
                uint64_t liveBase,uint64_t liveBytes,Plan *out) {
  *out=Plan();
  if (!verified) return kNotVerified;
  if (!p.bytes || (p.base&4095) || (p.bytes&4095)) return kRange;
  if (!image.count || image.count>fw::kMaxSegments || !image.headerBytes || !metadataBytes) return kBadImage;
  if (overlap(p.base,p.bytes,liveBase,liveBytes)) return kOverlap;
  if (metadataBytes>fw::kMdtMax) return kRange;
  out->metadataBytes=metadataBytes;
  uint64_t low=~0ULL,high=0;
  uint32_t relocatable=0,fixed=0;
  for(uint32_t i=0;i<image.count;++i) {
    const fw::Segment &s=image.seg[i];if(!loadable(s))continue;
    if(s.filesz>s.memsz || s.paddr>~0ULL-s.memsz || s.paddr+s.memsz>~0ULL-4095) return kRange;
    if(s.flags&(1u<<27))++relocatable;else ++fixed;
    if(s.paddr<low)low=s.paddr;
    uint64_t end=(s.paddr+s.memsz+4095)&~4095ULL;if(end>high)high=end;
  }
  if(relocatable && fixed) return kMixed;
  out->relocatable=relocatable!=0;
  if(low==~0ULL || high<=low || high-low>p.bytes || (low&4095)) return kRange;
  out->first=out->relocatable?p.base:low;out->span=high-low;
  // The metadata may run past the image span inside the carveout (a610_zap: 6860 bytes of metadata, one 4 KiB page of image in
  // gpu_region 0x2000): it only has to be readable at region_start when INIT_IMAGE runs.
  if(!fits(p.base,p.bytes,out->first,out->span) || !fits(p.base,p.bytes,out->first,out->metadataBytes)) return kRange;
  for(uint32_t i=0;i<image.count;++i) {
    const fw::Segment &s=image.seg[i];if(!loadable(s))continue;
    uint64_t address=out->relocatable?p.base+(s.paddr-low):s.paddr;
    if(!fits(p.base,p.bytes,address,s.memsz))return kRange;
    for(uint32_t j=0;j<out->count;++j)
      if(overlap(address,s.memsz,out->segments[j].address,out->segments[j].memsz))return kOverlap;
    out->segments[out->count++]={i,address,s.filesz,s.memsz};
  }
  return out->count?kOk:kBadImage;
}
} // namespace pas
#endif
