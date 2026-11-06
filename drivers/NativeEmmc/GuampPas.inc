// SPDX-License-Identifier: BSD-2-Clause-Patent
// Peripheral-independent authenticated PAS loader, generalised from wifi/driver-modem/GuampModem.inc (modemBoot):
// the peripheral (PAS id, carveout, firmware name) is a pas::Peripheral and the peripheral-specific parts (peer state,
// proxy votes, ready signal, failure reason) are pas::Hooks. Host-tested by tests/pas_host.cpp with the real ADSP
// firmware of the backup modem_a.img and a simulated secure monitor.
// Sequence of the guamp kernel's pil_boot() (peripheral-loader.c) with the subsys-pil-tz.c ops:
//   proxy vote -> copy the ORIGINAL metadata to region_start -> PAS INIT_IMAGE(id, region_start) -> PAS MEM_SETUP(id,
//   region_start, size) -> load every <fw>.bNN unchanged + zero p_memsz - p_filesz -> PAS AUTH_AND_RESET(id) -> ready
//   (subsystem_restart wait_for_err_ready) -> proxy unvote at handover or timeout.
//   Failure after INIT_IMAGE: PAS SHUTDOWN (deinit_image); the region is cleared (clear_fw_region) only if SHUTDOWN
//   succeeded (TrustZone may still protect it otherwise: a write would fault) or if INIT_IMAGE was never accepted.
// TrustZone alone authenticates the signature: firmware, metadata and hashes are copied byte for byte, never altered;
// every PAS status is honoured (nonzero = stop and clean up). The SHA-384 check of fw::verify is only an integrity
// gate on the eMMC read (nothing is authenticated from a file that does not match its own hash table: checked before
// boot(), or during the copy with pas::Deferred, where a mismatch stops before AUTH_AND_RESET and clears the region).
// Disk read-only. Only the peripheral's carveout is written, through an uncached mapping with aligned 32-bit stores.
#ifndef GUAMP_PAS_INC
#define GUAMP_PAS_INC
namespace pas {
enum : int { kUp=0,kQueried=1,kErrScm=-220,kErrPeer=-221,kErrPlace=-222,kErrMap=-223,kErrVote=-224,kErrInit=-225,
             kErrMemory=-226,kErrLoad=-227,kErrAuth=-228,kErrReady=-229,kErrMemoryAlloc=-230,kErrPrepare=-231 };
enum : int { kReady=1,kNotReady=0,kFatal=-1 };   // Hooks::awaitReady
struct Hooks {
  void *ctx=nullptr;
  bool (*stopped)(void *)=nullptr;          // peer confirmed not running: otherwise no RAM is written
  bool (*vote)(void *,bool on)=nullptr;     // pil_proxy_vote / pil_remove_proxy_vote (rails, XO)
  int (*awaitReady)(void *)=nullptr;        // after AUTH_AND_RESET; returns once the proxy votes may go
  void (*failure)(void *)=nullptr;          // failure reason (log only)
  // Owner hand-off (the modem's ModemBootHooks, GuampModem.inc): prepare runs after placement, the stopped-peer check
  // and the carveout mapping, before any vote or RAM write (nonzero = refuse, nothing written); cleanup runs after the
  // PAS SHUTDOWN of a failed start that INIT_IMAGE had accepted, with the SHUTDOWN result. ADSP sets only prepare, with
  // guamp-adsp-ipc>=1 (GuampAdspIpc.inc: apps SMP2P item 429 the ADSP reads at boot).
  int (*prepare)(void *)=nullptr;
  void (*cleanup)(void *,bool stopped)=nullptr;
};

// Integrity check folded into the load (modem, GuampModem.inc): the segments of `mask` were NOT read by fw::verify
// (skip mask, res->deferred); boot() hashes each one while it copies it (fw::SegmentCheck, same verdict and counters
// as fw::verify), so the 53 MB of the modem image are read once instead of twice. A mismatch is a failed load: no
// AUTH_AND_RESET, PAS SHUTDOWN, region cleared as for any load failure. After a mismatch the remaining deferred
// segments are still read and hashed (not written) so the report counts every segment as fw::verify does.
// checked: segments boot() actually hashed (equal to mask once the load loop ran to the end).
struct Deferred {
  fw::Image *image=nullptr;   // with the hash table set (fw::setHash)
  fw::Result *res=nullptr;    // fw::verify's result, completed here
  uint64_t mask=0,checked=0;
  void *ctx=nullptr;
  void (*report)(void *)=nullptr;   // right after the load loop, once every deferred segment was hashed (log only)
};

inline bool scmOkay(const Peripheral &p,const char *stage,const scm::Result &r) {
  IOLog("%s: PAS %u %s transport %d status %d calls %u resumes %u busy %u\n",
    p.tag,p.id,stage,r.transport,scm::code(r.value),r.calls,r.resumes,r.busy);
  return !r.transport && !scm::code(r.value);
}

struct Ram {
  IOMemoryMap *map=nullptr;
  volatile uint8_t *bytes=nullptr;
  uint64_t base=0,size=0;
  ~Ram() {if(map)map->release();}
  bool open(uint64_t at,uint64_t count) {
    base=at;size=count;
    IOMemoryDescriptor *d=IOMemoryDescriptor::withPhysicalAddress(at,count,kIODirectionInOut);
    map=d?d->map(kIOMapInhibitCache):nullptr;if(d)d->release();
    bytes=map?reinterpret_cast<volatile uint8_t *>(map->getVirtualAddress()):nullptr;return bytes;
  }
  bool write(uint64_t address,const uint8_t *source,uint64_t count,bool zero=false) {
    if(!bytes || !fits(base,size,address,count) || (!zero && !source))return false;
    volatile uint8_t *dst=bytes+(address-base);
    // Reserved RAM is mapped Device: no memcpy, SIMD, unaligned pair stores or
    // DC ZVA. Use aligned 32-bit I/O stores and byte tails instead.
    uint64_t i=0;
    while(i<count && ((address+i)&3)) {dst[i]=zero?0:source[i];++i;}
    for(;count-i>=4;i+=4) {
      uint32_t word=zero?0:ext4::le32(source+i);
      *reinterpret_cast<volatile uint32_t *>(dst+i)=word;
    }
    for(;i<count;++i)dst[i]=zero?0:source[i];
    return true;
  }
  void synchronize() {guampModemBarrier();}
};

// The metadata TrustZone receives, original bytes only: the .mdt file as the guamp kernel's pil_init_image_trusted
// copies it (fw->data, fw->size) when the hash segment is inside it (hashWhere 1 or 2); ELF + program headers followed
// by the hash segment read from its own .bNN when it is not (hashWhere 3, upstream qcom_mdt_read_metadata layout).
inline uint32_t metadataBytes(const fw::Image &image,uint32_t mdtFileBytes) {
  if(image.hashWhere==1 || image.hashWhere==2) return mdtFileBytes;
  if(image.hashWhere==3 && image.hashSeg && image.hashSegBytes) return image.headerBytes+image.hashSegBytes;
  return 0;
}

// mode 2: SCM/PAS queries, integrity and placement only (nothing mapped or written); mode 1: authenticated start.
inline int boot(const Peripheral &p,const Hooks &h,scm::Transport &transport,ext4::Fs &fs,const uint8_t *mdt,
                uint32_t mdtFileBytes,const fw::Image &image,uint8_t *chunk,uint32_t chunkBytes,bool verified,
                unsigned mode,uint64_t liveBase,uint64_t liveBytes,Deferred *deferred=nullptr) {
  if(mode!=1 && mode!=2)return kQueried;
  if(deferred && (mode!=1 || !deferred->image || !deferred->res)) {   // queries need the full check before boot()
    IOLog("%s: integrity check deferred to a load that will not run; no RAM written\n",p.tag);return kErrPlace;
  }
  if(!transport.discover()) {IOLog("%s: SCM convention unavailable; no RAM written\n",p.tag);return kErrScm;}
  IOLog("%s: SCM%u detected, PAS id %u, mode %u\n",p.tag,transport.wide?64u:32u,p.id,mode);
  // PAS_IS_SUPPORTED is informational, as in Linux's remoteproc PAS driver (which never asks it): on guamp TrustZone answers 0
  // for the modem (28/10). The firmware is still authenticated by TrustZone in INIT_IMAGE and AUTH_AND_RESET, whose refusal stops here.
  scm::Result supported=transport.supported(p.id);
  IOLog("%s: PAS %u is-supported transport %d result %d (informational)\n",p.tag,p.id,supported.transport,
        scm::code(supported.value));
  static const unsigned commands[]={1,2,5,6};
  for(unsigned command : commands) {
    scm::Result r=transport.available(2,command);
    if(r.transport || !r.value) {
      IOLog("%s: PAS command %u unavailable (transport %d result %d); no RAM written\n",p.tag,command,r.transport,
            scm::code(r.value));
      return kErrScm;
    }
  }
  Plan *plan=static_cast<Plan *>(IOMallocZero(sizeof(Plan)));
  if(!plan)return kErrMemoryAlloc;
  const uint32_t meta=metadataBytes(image,mdtFileBytes);
  int placed=pas::plan(p,image,verified,meta,liveBase,liveBytes,plan);
  if(placed) {
    IOLog("%s: placement refused %d (verified %u, metadata %u bytes); no RAM written\n",p.tag,placed,verified?1u:0u,meta);
    IOFree(plan,sizeof(*plan));return kErrPlace;
  }
  IOLog("%s: plan %s, region %llx + %llx in carveout %llx + %llx, %u segments, metadata %u bytes (hash segment %d)\n",
        p.tag,plan->relocatable?"relocatable":"fixed",plan->first,plan->span,p.base,p.bytes,plan->count,meta,
        image.hashWhere);
  if(mode==2) {
    IOLog("%s: SCM/PAS capabilities, firmware integrity and placement checked only; not started\n",p.tag);
    IOFree(plan,sizeof(*plan));return kQueried;
  }
  // Refuse a running/stale remote instead of overwriting a live image.
  if(h.stopped && !h.stopped(h.ctx)) {
    IOLog("%s: SMEM is not a confirmed stopped peer; no RAM written\n",p.tag);
    IOFree(plan,sizeof(*plan));return kErrPeer;
  }
  Ram ram;
  if(!ram.open(p.base,p.bytes)) {IOLog("%s: carveout mapping failed\n",p.tag);IOFree(plan,sizeof(*plan));return kErrMap;}
  int result=kUp;
  if(h.prepare) {
    const int e=h.prepare(h.ctx);
    if(e) {IOLog("%s: owner prepare failed %d; no RAM written\n",p.tag,e);IOFree(plan,sizeof(*plan));return kErrPrepare;}
  }
  bool voted=h.vote?h.vote(h.ctx,true):true;
  if(!voted) {IOLog("%s: proxy vote failed; no RAM written\n",p.tag);result=kErrVote;}
  bool initialized=false,okay=voted;
  if(okay) {
    if(image.hashWhere==3) {
      okay=ram.write(plan->first,mdt,image.headerBytes) &&
           ram.write(plan->first+image.headerBytes,image.hashSeg,image.hashSegBytes);
    } else {
      okay=ram.write(plan->first,mdt,meta);
    }
    ram.synchronize();
    if(!okay)result=kErrLoad;
  }
  if(okay) {
    okay=scmOkay(p,"init-image",transport.initImage(p.id,plan->first));initialized=okay;
    if(!okay)result=kErrInit;
  }
  if(okay) {okay=scmOkay(p,"mem-setup",transport.memory(p.id,plan->first,plan->span));if(!okay)result=kErrMemory;}
  if(okay) {
    uint64_t loaded=0,zeroed=0;
    const uint64_t began=mach_absolute_time();
    bool hashBad=false;   // a deferred segment failed its check: keep hashing the rest, write nothing more
    for(uint32_t i=0;okay && i<plan->count;++i) {
      const Segment &s=plan->segments[i];char path[128];
      okay=fw::joinName(path,sizeof(path),p.dir,p.firmware,".b",static_cast<int>(s.index));
      if(okay && deferred && s.index<64 && (deferred->mask>>s.index&1)) {
        // One read for check and copy: same steps and verdict as fw::verify (lookup, regular file of p_filesz bytes,
        // nonzero table entry, SHA-384 of the file bytes), the chunk goes to the carveout only after it was hashed.
        fw::SegmentCheck check;check.begin(*deferred->image,s.index);
        ext4::Inode inode;
        if(fs.lookupPath(path,&inode) || !inode.isFile() || inode.size!=s.filesz)check.ok=false;
        for(uint64_t off=0;check.ok && off<s.filesz;) {
          uint32_t n=s.filesz-off<chunkBytes?static_cast<uint32_t>(s.filesz-off):chunkBytes;
          if(fs.readFile(inode,off,chunk,n)) {check.ok=false;break;}
          check.feed(chunk,n,deferred->res);
          if(!hashBad && !ram.write(s.address+off,chunk,n)) {okay=false;break;}
          off+=n;if(!hashBad)loaded+=n;
        }
        if(!okay)break;
        const uint32_t failedBefore=deferred->res->failed;
        check.finish(s.index,deferred->res);
        deferred->checked|=1ull<<s.index;
        if(deferred->res->failed!=failedBefore) {
          if(!hashBad)IOLog("%s: segment %u does not match its hash table entry: load stopped, start refused\n",p.tag,s.index);
          hashBad=true;
        }
        if(!hashBad && s.memsz>s.filesz) {
          okay=ram.write(s.address+s.filesz,nullptr,s.memsz-s.filesz,true);
          if(okay)zeroed+=s.memsz-s.filesz;
        }
        continue;
      }
      if(hashBad)continue;   // only the deferred segments are still read (for the report)
      // The file is looked up once per segment, not once per chunk (the /image directory walk cost the modem load
      // ~26 MB of extra eMMC reads, host-measured on modem_a.img); same checks as the former per-chunk readExact.
      ext4::Inode inode;
      if(okay && s.filesz)okay=!fs.lookupPath(path,&inode) && inode.isFile() && s.filesz<=inode.size;
      for(uint64_t off=0;okay && off<s.filesz;) {
        uint32_t n=s.filesz-off<chunkBytes?static_cast<uint32_t>(s.filesz-off):chunkBytes;
        okay=!fs.readFile(inode,off,chunk,n) && ram.write(s.address+off,chunk,n);
        if(okay){off+=n;loaded+=n;}
      }
      if(okay && s.memsz>s.filesz) {
        okay=ram.write(s.address+s.filesz,nullptr,s.memsz-s.filesz,true);
        if(okay)zeroed+=s.memsz-s.filesz;
      }
    }
    if(deferred && okay && (hashBad || deferred->checked!=deferred->mask)) {
      if(!hashBad)IOLog("%s: deferred segments %llx not all in the plan (%llx hashed): start refused\n",p.tag,
                        deferred->mask,deferred->checked);
      okay=false;
    }
    ram.synchronize();
    uint64_t ns=0;absolutetime_to_nanoseconds(mach_absolute_time()-began,&ns);
    IOLog("%s: segments %u loaded %llu zeroed %llu in %llu ms: %s\n",p.tag,plan->count,loaded,zeroed,ns/1000000,
          okay?"ok":"failed");
    if(deferred && deferred->report && deferred->checked==deferred->mask)deferred->report(deferred->ctx);
    if(!okay)result=kErrLoad;
  }
  if(okay) {okay=scmOkay(p,"auth-and-reset",transport.authenticate(p.id));if(!okay)result=kErrAuth;}
  if(okay) {
    IOLog("%s: authenticated start accepted by TrustZone; waiting for the ready signal\n",p.tag);
    int state=h.awaitReady?h.awaitReady(h.ctx):kReady;
    okay=state==kReady;
    if(okay)IOLog("%s: peripheral up\n",p.tag);
    else {IOLog("%s: %s\n",p.tag,state==kFatal?"fatal error reported":"no ready signal in time");result=kErrReady;}
  }
  if(!okay) {
    if(h.failure)h.failure(h.ctx);
    if(initialized) {
      bool down=scmOkay(p,"cleanup-shutdown",transport.shutdown(p.id));
      if(down) {   // the metadata may run past the span (GuampPasCore.inc plan): both cleared
        ram.write(plan->first,nullptr,plan->span>meta?plan->span:meta,true);ram.synchronize();IOLog("%s: region cleared\n",p.tag);
      }
      else IOLog("%s: shutdown refused: region left untouched\n",p.tag);
      if(h.cleanup)h.cleanup(h.ctx,down);
    } else if(voted) {
      ram.write(plan->first,nullptr,meta,true);ram.synchronize();
    }
  }
  if(voted && h.vote)h.vote(h.ctx,false);
  IOFree(plan,sizeof(*plan));
  return result;
}
} // namespace pas
#endif
