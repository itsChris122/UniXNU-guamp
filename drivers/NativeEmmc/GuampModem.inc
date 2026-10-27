// SPDX-License-Identifier: BSD-2-Clause-Patent
// Opt-in authenticated modem loader. Included inside wifiprep after its readers.
// Disk remains read-only. Only the reserved modem RAM is written. No TrustZone
// authentication result is bypassed. No claim of a working WLAN interface yet.
static void scmSleep(void *,uint32_t ms) {IOSleep(ms);}
static bool pasOkay(const char *stage,const scm::Result &r) {
  IOLog("GuampModem: PAS %s transport %d status %d calls %u resumes %u busy %u\n",
    stage,r.transport,scm::code(r.value),r.calls,r.resumes,r.busy);
  return !r.transport && !scm::code(r.value);
}
struct ModemRam {
  IOMemoryMap *map=nullptr;
  volatile uint8_t *bytes=nullptr;
  ~ModemRam() {if(map)map->release();}
  bool open() {
    IOMemoryDescriptor *d=IOMemoryDescriptor::withPhysicalAddress(modem::kBase,modem::kBytes,kIODirectionInOut);
    map=d?d->map(kIOMapInhibitCache):nullptr;if(d)d->release();
    bytes=map?reinterpret_cast<volatile uint8_t *>(map->getVirtualAddress()):nullptr;return bytes;
  }
  bool write(uint64_t address,const uint8_t *source,uint32_t count,bool zero=false) {
    if(!bytes || !modem::fits(modem::kBase,modem::kBytes,address,count) || (!zero && !source))return false;
    volatile uint8_t *dst=bytes+(address-modem::kBase);
    // Reserved RAM is mapped Device: no memcpy, SIMD, unaligned pair stores or
    // DC ZVA. Use aligned 32-bit I/O stores and byte tails instead.
    uint32_t i=0;
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

static bool readExact(ext4::Fs &fs,const char *path,uint64_t offset,uint8_t *out,uint32_t bytes) {
  ext4::Inode inode;
  return !fs.lookupPath(path,&inode) && inode.isFile() && offset<=inode.size && bytes<=inode.size-offset &&
         !fs.readFile(inode,offset,out,bytes);
}

static void modemBoot(ext4::Fs &fs,uint8_t *mdt,uint8_t *chunk,bool verified) {
  unsigned enabled=0;
  if(!PE_parse_boot_argn("guamp-modem-start",&enabled,sizeof(enabled)) || (enabled!=1 && enabled!=2))return;
  scm::Transport transport={guampScmInstruction,scmSleep,nullptr,true};
  if(!transport.discover()) {IOLog("GuampModem: SCM convention unavailable; no RAM written\n");return;}
  IOLog("GuampModem: SCM%u detected, authenticated PAS4 path, mode %u\n",transport.wide?64u:32u,enabled);
  scm::Result supported=transport.supported(modem::kPas);
  if(supported.transport || supported.value!=1) {
    IOLog("GuampModem: PAS4 unsupported transport %d result %d; no RAM written\n",supported.transport,scm::code(supported.value));return;
  }
  static const unsigned commands[]={1,2,5,6};
  for(unsigned command : commands) {
    scm::Result r=transport.available(2,command);
    if(r.transport || r.value!=1) {IOLog("GuampModem: PAS command %u unavailable; no RAM written\n",command);return;}
  }
  if(enabled==2) {IOLog("GuampModem: SCM/PAS capabilities checked only; modem not started\n");return;}
  // Refuse a running/stale remote instead of overwriting a live modem image.
  if(!modemStopped()) {IOLog("GuampModem: SMEM is not a confirmed stopped modem; no RAM written\n");return;}
  if(modem::overlap(modem::kBase,modem::kBytes,guamp_phys_base,guamp_phys_size)) {
    IOLog("GuampModem: reserved window overlaps XNU; refused\n");return;
  }
  fw::Image *image=static_cast<fw::Image *>(IOMallocZero(sizeof(fw::Image)));
  modem::Plan *plan=static_cast<modem::Plan *>(IOMallocZero(sizeof(modem::Plan)));
  if(!image || !plan) {if(image)IOFree(image,sizeof(*image));if(plan)IOFree(plan,sizeof(*plan));return;}
  ext4::Inode inode;int parsed=fs.lookupPath("/image/modem.mdt",&inode);
  if(!parsed && inode.size<=fw::kMdtMax/2) parsed=fw::parse(mdt,static_cast<uint32_t>(inode.size),image);
  else parsed=fw::kErrFormat;
  if(!parsed && image->hashWhere==3) {
    uint32_t hashAt=(static_cast<uint32_t>(inode.size)+63)&~63u;
    if(image->hashSegBytes>fw::kMdtMax-hashAt)parsed=fw::kErrFormat;
    else image->hashSeg=mdt+hashAt; // original bytes already read/verified by fw::verify
  }
  if(!parsed)parsed=modem::plan(*image,verified,guamp_phys_base,guamp_phys_size,plan);
  if(parsed) {IOLog("GuampModem: placement refused %d; no RAM written\n",parsed);}
  else {
    ModemRam ram;
    if(!ram.open())IOLog("GuampModem: reserved RAM mapping failed\n");
    else {
      // PAS portion of the board's peripheral-loader/subsys-pil-tz: copy original
      // metadata to region_start, INIT_IMAGE, MEM_SETUP, sequential segments,
      // AUTH_AND_RESET. TZ remains responsible for signature authentication.
      bool initialized=false;
      bool okay=ram.write(plan->first,mdt,image->headerBytes) &&
        ram.write(plan->first+image->headerBytes,image->hashSeg,image->hashSegBytes);
      ram.synchronize();
      if(okay) {okay=pasOkay("init",transport.initImage(modem::kPas,plan->first));initialized=okay;}
      if(okay)okay=pasOkay("memory",transport.memory(modem::kPas,plan->first,plan->span));
      uint64_t loaded=0,zeroed=0;
      for(uint32_t i=0;okay && i<plan->count;++i) {
        const modem::Segment &s=plan->segments[i];char path[128];
        okay=fw::joinName(path,sizeof(path),"/image","modem",".b",static_cast<int>(s.index));
        for(uint64_t off=0;okay && off<s.filesz;) {
          uint32_t n=s.filesz-off<kChunkBytes?static_cast<uint32_t>(s.filesz-off):kChunkBytes;
          okay=readExact(fs,path,off,chunk,n) && ram.write(s.address+off,chunk,n);
          if(okay){off+=n;loaded+=n;}
        }
        for(uint64_t off=s.filesz;okay && off<s.memsz;) {
          uint32_t n=s.memsz-off<kChunkBytes?static_cast<uint32_t>(s.memsz-off):kChunkBytes;
          okay=ram.write(s.address+off,nullptr,n,true);
          if(okay){off+=n;zeroed+=n;}
        }
      }
      ram.synchronize();
      IOLog("GuampModem: segments %u loaded %llu zeroed %llu result %s\n",plan->count,loaded,zeroed,okay?"ok":"failed");
      if(okay)okay=pasOkay("authenticate-reset",transport.authenticate(modem::kPas));
      if(okay) {
        IOLog("GuampModem: authenticated start accepted; observing actual ready state\n");
        okay=modemAwaitReady();
        if(okay)IOLog("GuampModem: firmware ready; WLAN transport is not implemented yet\n");
      }
      if(!okay && initialized)pasOkay("cleanup-shutdown",transport.shutdown(modem::kPas));
    }
  }
  IOFree(image,sizeof(*image));IOFree(plan,sizeof(*plan));
}
