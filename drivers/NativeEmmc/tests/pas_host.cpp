// Generic PAS loader (GuampPasCore.inc + GuampPas.inc) in a simulated IOKit / secure-monitor environment: the real
// ext4 reader and firmware verifier on the backup modem_a.img, the real ADSP (and modem) split firmware, the actual
// placement and the actual loader. No device operation. Usage: pas_host <modem_a.img> <extracted /image directory>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <vector>
#include <string>
#include "../GuampExt4.inc"
#include "../GuampFirmware.inc"
#include "../GuampScm.inc"
#include "../GuampPasCore.inc"
static std::vector<uint8_t> ram;
static std::string logs,extracted;
static pas::Peripheral peripheral;
static pas::Plan expected;
static fw::Image firmware;
static std::vector<uint8_t> metadata;
static unsigned maps,initializes,memories,authenticates,shutdowns,readyCalls,barriers,votesOn,votesOff,failures,discovers;
static bool stopped=true,voteOk=true,denyInit=false,denyAuth=false,denyMemory=false,denyShutdown=false;
static int readyState=1;   // pas::kReady (GuampPas.inc comes after the stand-ins)
static const uint64_t xnuBase=0x80000000ULL,xnuBytes=0xb8000000ULL;
static void IOLog(const char *fmt,...) {
  char b[512];va_list args;va_start(args,fmt);vsnprintf(b,sizeof(b),fmt,args);va_end(args);logs+=b;
  if(getenv("PAS_LOG"))fputs(b,stdout);
}
static void *IOMallocZero(size_t n) {return calloc(1,n);}
static void IOFree(void *p,size_t) {free(p);}
static uint64_t mach_absolute_time() {return 0;}
static void absolutetime_to_nanoseconds(uint64_t t,uint64_t *ns) {*ns=t;}
typedef unsigned IODirection;
constexpr unsigned kIODirectionInOut=3,kIOMapInhibitCache=0x100;
struct IOMemoryMap {uint64_t getVirtualAddress(){return reinterpret_cast<uint64_t>(ram.data());}void release(){}};
struct IOMemoryDescriptor {
  static IOMemoryDescriptor *withPhysicalAddress(uint64_t at,uint64_t bytes,unsigned direction) {
    assert(at==peripheral.base && bytes==peripheral.bytes && direction==kIODirectionInOut);++maps;
    ram.assign(bytes,0xcc);static IOMemoryDescriptor d;return &d;
  }
  IOMemoryMap *map(unsigned options){assert(options==kIOMapInhibitCache);static IOMemoryMap m;return &m;}
  void release(){}
};
static void guampModemBarrier(){++barriers;}
static uint8_t at(uint64_t address) {return ram[address-peripheral.base];}
static void verifyLoaded() {
  std::vector<uint8_t> bytes(65536);
  for(unsigned i=0;i<expected.count;++i) {
    const auto &s=expected.segments[i];char name[256];
    snprintf(name,sizeof(name),"%s/%s.b%02u",extracted.c_str(),peripheral.firmware,s.index);
    int fd=open(name,O_RDONLY);assert(fd>=0 || !s.filesz);uint64_t off=0;
    while(off<s.filesz) {
      size_t n=s.filesz-off<bytes.size()?s.filesz-off:bytes.size();assert(read(fd,bytes.data(),n)==static_cast<ssize_t>(n));
      assert(!memcmp(ram.data()+s.address-peripheral.base+off,bytes.data(),n));off+=n;
    }
    if(fd>=0)close(fd);
    for(off=s.filesz;off<s.memsz;++off)assert(at(s.address+off)==0);
  }
}
static void monitor(void *,const scm::Registers &in,scm::Registers &out) {
  uint32_t fn=static_cast<uint32_t>(in.x[0]);out=scm::Registers();
  unsigned service=(fn>>8)&255,command=fn&255;
  if(service==6 && command==1) {
    if(fn==0xc2000601) {++discovers;assert(in.x[1]==1 && in.x[2]==0x82000601);out.x[1]=1;return;}
    assert(fn==0x42000601 && in.x[1]==1 && (in.x[2]==0x02000201 || in.x[2]==0x02000202 || in.x[2]==0x02000205 ||
                                            in.x[2]==0x02000206));
    out.x[1]=1;return;
  }
  assert(service==2 && (fn&0xff000000u)==0x42000000u && in.x[2]==peripheral.id);
  if(command==7) {assert(in.x[1]==1);out.x[1]=1;return;}
  if(command==1) {
    ++initializes;assert(maps==1 && votesOn==1 && barriers>=1 && in.x[1]==0x82 && in.x[3]==expected.first);
    assert(!memcmp(ram.data()+expected.first-peripheral.base,metadata.data(),metadata.size()));   // original bytes
    if(denyInit)out.x[1]=static_cast<uint64_t>(-22);
    return;
  }
  if(command==2) {
    ++memories;assert(initializes==1 && in.x[1]==3 && in.x[3]==expected.first && in.x[4]==expected.span);
    if(denyMemory)out.x[1]=1;
    return;
  }
  if(command==5) {++authenticates;assert(memories==1 && in.x[1]==1);verifyLoaded();if(denyAuth)out.x[1]=1;return;}
  if(command==6) {++shutdowns;assert(in.x[1]==1);if(denyShutdown)out.x[1]=1;return;}
  assert(false);
}
static void guampScmInstruction(void *c,const scm::Registers &in,scm::Registers &out) {monitor(c,in,out);}
#include "../GuampPas.inc"
static bool hookStopped(void *) {return stopped;}
static bool hookVote(void *,bool on) {if(on){++votesOn;return voteOk;}++votesOff;return true;}
static int hookReady(void *) {++readyCalls;return readyState;}
static void hookFailure(void *) {++failures;}
static int diskFd;
static bool readDisk(void *,uint64_t sector,uint32_t count,uint8_t *out) {
  assert(count && count<=128);
  return pread(diskFd,out,count*512,static_cast<off_t>(sector*512))==static_cast<ssize_t>(count*512);
}
static void reset() {
  ram.clear();logs.clear();
  maps=initializes=memories=authenticates=shutdowns=readyCalls=barriers=votesOn=votesOff=failures=discovers=0;
  stopped=voteOk=true;denyInit=denyAuth=denyMemory=denyShutdown=false;readyState=pas::kReady;
}
static bool allZero(uint64_t from,uint64_t bytes) {
  for(uint64_t i=0;i<bytes;++i)if(at(from+i))return false;
  return true;
}
// One peripheral: verify, parse, plan, then every loader path
static unsigned exercise(ext4::Fs &fs,const pas::Peripheral &p,std::vector<uint8_t> &mdt,std::vector<uint8_t> &chunk,
                         uint32_t wantFirst,uint32_t wantSpan) {
  peripheral=p;
  fw::Result result;
  fw::verify(fs,p.dir,p.firmware,mdt.data(),chunk.data(),chunk.size(),&result);
  assert(!result.error && !result.failed && result.verified);
  char path[128];ext4::Inode ino;
  assert(fw::joinName(path,sizeof(path),p.dir,p.firmware,".mdt",-1) && !fs.lookupPath(path,&ino));
  firmware=fw::Image();assert(!fw::parse(mdt.data(),ino.size,&firmware));
  if(firmware.hashWhere==3)firmware.hashSeg=mdt.data()+((ino.size+63)&~63ULL);
  const uint32_t meta=pas::metadataBytes(firmware,ino.size);
  if(firmware.hashWhere==3) {
    metadata.assign(mdt.data(),mdt.data()+firmware.headerBytes);
    metadata.insert(metadata.end(),firmware.hashSeg,firmware.hashSeg+firmware.hashSegBytes);
  } else {
    metadata.assign(mdt.data(),mdt.data()+ino.size);   // the .mdt file exactly as on disk
  }
  assert(meta==metadata.size());
  assert(!pas::plan(p,firmware,true,meta,xnuBase,xnuBytes,&expected));
  if(wantFirst)assert(expected.first==wantFirst);
  if(wantSpan)assert(expected.span==wantSpan);
  {pas::Plan q;
   assert(pas::plan(p,firmware,false,meta,xnuBase,xnuBytes,&q)==pas::kNotVerified);
   assert(pas::plan(p,firmware,true,meta,p.base+p.bytes-4096,8192,&q)==pas::kOverlap);
   pas::Peripheral small=p;small.bytes=expected.span-4096;
   assert(pas::plan(small,firmware,true,meta,xnuBase,xnuBytes,&q)==pas::kRange);
   pas::Peripheral odd=p;odd.base=p.base+1;
   assert(pas::plan(odd,firmware,true,meta,xnuBase,xnuBytes,&q)==pas::kRange);
   fw::Image mixed=firmware;
   for(uint32_t i=0;i<mixed.count;++i)if(pas::loadable(mixed.seg[i])){mixed.seg[i].flags^=1u<<27;break;}
   assert(pas::plan(p,mixed,true,meta,xnuBase,xnuBytes,&q)==pas::kMixed);}
  pas::Hooks hooks;hooks.stopped=hookStopped;hooks.vote=hookVote;hooks.awaitReady=hookReady;hooks.failure=hookFailure;
  scm::Transport t={guampScmInstruction,[](void *,uint32_t){},nullptr,true};
  auto run=[&](unsigned mode,bool verified){return pas::boot(p,hooks,t,fs,mdt.data(),ino.size,firmware,chunk.data(),
                                                             chunk.size(),verified,mode,xnuBase,xnuBytes);};
  // success: every byte in place, gaps untouched, votes balanced, no shutdown
  reset();assert(run(1,true)==pas::kUp);
  assert(discovers==1 && initializes==1 && memories==1 && authenticates==1 && readyCalls==1 && !shutdowns && !failures);
  assert(votesOn==1 && votesOff==1);verifyLoaded();
  if(expected.span<p.bytes)assert(at(expected.first+expected.span)==0xcc);
  // queries only: no mapping, no RAM, no votes
  reset();assert(run(2,true)==pas::kQueried && !maps && !initializes && !votesOn);
  reset();assert(run(0,true)==pas::kQueried && !discovers);
  // refusals before any write
  reset();stopped=false;assert(run(1,true)==pas::kErrPeer && !maps && !initializes && !votesOn);
  reset();assert(run(1,false)==pas::kErrPlace && !maps && !initializes);
  reset();voteOk=false;assert(run(1,true)==pas::kErrVote && maps==1 && !initializes && votesOff==0);
  assert(at(expected.first)==0xcc);
  // TrustZone refusals: cleanup as pil_boot (shutdown only after an accepted INIT_IMAGE)
  reset();denyInit=true;assert(run(1,true)==pas::kErrInit && initializes==1 && !memories && !shutdowns && votesOff==1);
  assert(allZero(expected.first,meta));
  reset();denyMemory=true;assert(run(1,true)==pas::kErrMemory && memories==1 && !authenticates && shutdowns==1);
  assert(allZero(expected.first,expected.span) && votesOff==1);
  reset();denyAuth=true;assert(run(1,true)==pas::kErrAuth && authenticates==1 && !readyCalls && shutdowns==1 && failures==1);
  assert(allZero(expected.first,expected.span));
  reset();readyState=pas::kNotReady;assert(run(1,true)==pas::kErrReady && readyCalls==1 && shutdowns==1 && votesOff==1);
  reset();readyState=pas::kFatal;assert(run(1,true)==pas::kErrReady && shutdowns==1 && failures==1);
  // shutdown refused: the region must stay untouched (TrustZone may still protect it)
  reset();denyAuth=true;denyShutdown=true;assert(run(1,true)==pas::kErrAuth && shutdowns==1);
  verifyLoaded();assert(logs.find("left untouched")!=std::string::npos);
  return expected.count;
}
int main(int argc,char **argv) {
  assert(argc==3);diskFd=open(argv[1],O_RDONLY);assert(diskFd>=0);extracted=argv[2];
  struct stat st;assert(!fstat(diskFd,&st));std::vector<uint8_t> scratch(ext4::kScratchBytes),mdt(fw::kMdtMax),chunk(65536);
  static ext4::Fs fs;assert(!fs.mount(readDisk,nullptr,st.st_size/512,scratch.data()));
  // ADSP: qcom,pas-id 1, pil_adsp_region@52200000 0x1c00000 (relocatable image of exactly that size)
  const pas::Peripheral adsp={"GuampAdsp",1,0x52200000ULL,0x1c00000ULL,"/image","adsp"};
  unsigned a=exercise(fs,adsp,mdt,chunk,0x52200000u,0x1c00000u);
  assert(firmware.hashWhere==1 && expected.relocatable && metadata.size()==9580);
  // Same loader, modem parameters (the generalisation of GuampModem.inc): pas-id 4, modem_region 0x4ab00000 0x6900000
  const pas::Peripheral modem={"GuampModem",4,0x4ab00000ULL,0x6900000ULL,"/image","modem"};
  unsigned m=exercise(fs,modem,mdt,chunk,0,0);
  printf("PASS PAS loader: adsp %u segments at %llx+%llx, modem %u segments; metadata bytes, PAS order and encoding, "
         "byte/BSS comparison, queries-only, peer/integrity/vote refusals, init/mem/auth/ready/fatal cleanup, "
         "shutdown-refused region untouched, placement limits\n",a,0x52200000ULL,0x1c00000ULL,m);
  close(diskFd);
}
