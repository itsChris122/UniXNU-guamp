// Full loader in a simulated IOKit/secure-monitor environment. Real ext4 image,
// real firmware verifier, actual plan and actual loader; no device operations.
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
#include "../GuampModemCore.inc"
static std::vector<uint8_t> ram;
static std::string logs,extracted;
static modem::Plan expected;
static fw::Image firmware;
static unsigned mode=1,maps,initializes,memories,authenticates,shutdowns,readyCalls,barriers;
static bool stopped=true,ready=true,denyAuth=false,denyMemory=false;
static uint64_t guamp_phys_base=0x80000000ULL,guamp_phys_size=0xb8000000ULL;
static void IOLog(const char *fmt,...) {
  char b[512];va_list args;va_start(args,fmt);vsnprintf(b,sizeof(b),fmt,args);va_end(args);logs+=b;
}
static void *IOMallocZero(size_t n) {return calloc(1,n);}
static void IOFree(void *p,size_t) {free(p);}
static void IOSleep(unsigned) {}
static bool PE_parse_boot_argn(const char *key,void *value,size_t bytes) {
  assert(!strcmp(key,"guamp-modem-start") && bytes==sizeof(unsigned));memcpy(value,&mode,bytes);return true;
}
typedef unsigned IODirection;
constexpr unsigned kIODirectionInOut=3,kIOMapInhibitCache=0x100;
struct IOMemoryMap {uint64_t getVirtualAddress(){return reinterpret_cast<uint64_t>(ram.data());}void release(){}};
struct IOMemoryDescriptor {
  static IOMemoryDescriptor *withPhysicalAddress(uint64_t at,uint64_t bytes,unsigned direction) {
    assert(at==modem::kBase && bytes==modem::kBytes && direction==kIODirectionInOut);++maps;
    ram.assign(bytes,0xcc);static IOMemoryDescriptor d;return &d;
  }
  IOMemoryMap *map(unsigned options){assert(options==kIOMapInhibitCache);static IOMemoryMap m;return &m;}
  void release(){}
};
static void verifyLoaded() {
  std::vector<uint8_t> bytes(65536);
  for(unsigned i=0;i<expected.count;++i) {
    const auto &s=expected.segments[i];char name[128];snprintf(name,sizeof(name),"%s/modem.b%02u",extracted.c_str(),s.index);
    int fd=open(name,O_RDONLY);assert(fd>=0 || !s.filesz);uint64_t off=0;
    while(off<s.filesz) {
      size_t n=s.filesz-off<bytes.size()?s.filesz-off:bytes.size();assert(read(fd,bytes.data(),n)==static_cast<ssize_t>(n));
      assert(!memcmp(ram.data()+s.address-modem::kBase+off,bytes.data(),n));off+=n;
    }
    if(fd>=0)close(fd);
    for(off=s.filesz;off<s.memsz;++off)assert(ram[s.address-modem::kBase+off]==0);
  }
}
static void guampModemBarrier(){++barriers;}
static void guampScmInstruction(void *,const scm::Registers &in,scm::Registers &out) {
  uint32_t fn=static_cast<uint32_t>(in.x[0]);out=scm::Registers();
  unsigned service=(fn>>8)&255,command=fn&255;
  if(service==6 && command==1) {out.x[1]=1;return;}
  assert(service==2 && in.x[2]==4);
  if(command==7) {out.x[1]=1;return;}
  if(command==1) {
    ++initializes;assert(maps==1 && barriers>=1 && in.x[1]==0x82 && in.x[3]==expected.first);
    auto start=ram.data()+expected.first-modem::kBase;
    assert(!memcmp(start+firmware.headerBytes,firmware.hashSeg,firmware.hashSegBytes));return;
  }
  if(command==2) {
    ++memories;assert(initializes==1 && in.x[3]==expected.first && in.x[4]==expected.span);
    if(denyMemory)out.x[1]=1;return;
  }
  if(command==5) {++authenticates;assert(memories==1);verifyLoaded();if(denyAuth)out.x[1]=1;return;}
  if(command==6) {++shutdowns;return;}
  assert(false);
}
namespace wifiprep {
constexpr uint32_t kChunkBytes=65536;
static bool modemStopped(){return stopped;}
static bool modemAwaitReady(){++readyCalls;return ready;}
#include "../GuampModem.inc"
}
static int diskFd;
static bool readDisk(void *,uint64_t sector,uint32_t count,uint8_t *out) {
  assert(count && count<=128);
  return pread(diskFd,out,count*512,static_cast<off_t>(sector*512))==static_cast<ssize_t>(count*512);
}
static void reset() {
  ram.clear();logs.clear();maps=initializes=memories=authenticates=shutdowns=readyCalls=barriers=0;
  mode=1;stopped=ready=true;denyAuth=denyMemory=false;
}
int main(int argc,char **argv) {
  assert(argc==3);diskFd=open(argv[1],O_RDONLY);assert(diskFd>=0);extracted=argv[2];
  struct stat st;assert(!fstat(diskFd,&st));std::vector<uint8_t> scratch(ext4::kScratchBytes),mdt(fw::kMdtMax),chunk(65536);
  ext4::Fs fs;assert(!fs.mount(readDisk,nullptr,st.st_size/512,scratch.data()));fw::Result result;
  fw::verify(fs,"/image","modem",mdt.data(),chunk.data(),chunk.size(),&result);
  assert(!result.error && !result.failed && result.verified);ext4::Inode ino;assert(!fs.lookupPath("/image/modem.mdt",&ino));
  assert(!fw::parse(mdt.data(),ino.size,&firmware));
  if(firmware.hashWhere==3)firmware.hashSeg=mdt.data()+((ino.size+63)&~63ULL);
  assert(!modem::plan(firmware,true,guamp_phys_base,guamp_phys_size,&expected));
  {modem::Plan p;assert(modem::plan(firmware,false,guamp_phys_base,guamp_phys_size,&p)==modem::kNotVerified);
   assert(modem::plan(firmware,true,modem::kBase,4096,&p)==modem::kOverlap);
   assert(modem::overlap(~0ULL-4,8,0,4));assert(!modem::fits(modem::kBase,modem::kBytes,modem::kBase-1,1));}
  reset();wifiprep::modemBoot(fs,mdt.data(),chunk.data(),true);
  assert(initializes==1 && memories==1 && authenticates==1 && readyCalls==1 && !shutdowns);
  assert(logs.find("firmware ready")!=std::string::npos);
  reset();mode=2;wifiprep::modemBoot(fs,mdt.data(),chunk.data(),true);assert(!maps && !initializes);
  reset();stopped=false;wifiprep::modemBoot(fs,mdt.data(),chunk.data(),true);assert(!maps && !initializes);
  reset();wifiprep::modemBoot(fs,mdt.data(),chunk.data(),false);assert(!maps && !initializes);
  reset();denyMemory=true;wifiprep::modemBoot(fs,mdt.data(),chunk.data(),true);assert(initializes==1 && memories==1 && !authenticates && shutdowns==1);
  reset();denyAuth=true;wifiprep::modemBoot(fs,mdt.data(),chunk.data(),true);assert(authenticates==1 && !readyCalls && shutdowns==1);
  reset();ready=false;wifiprep::modemBoot(fs,mdt.data(),chunk.data(),true);assert(authenticates==1 && readyCalls==1 && shutdowns==1);
  printf("PASS actual modem loader: %u real segments, placement,byte/BSS comparison, metadata/PAS order, probe-only,live-peer refusal,integrity failure,auth failure,ready timeout cleanup\n",expected.count);
  close(diskFd);
}
