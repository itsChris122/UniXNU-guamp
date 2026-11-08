// SPDX-License-Identifier: BSD-2-Clause-Patent
// Read-only, allow-listed modem-partition firmware source for WLFW adapters.
// This is not a PDR/TFTP service and does not load wlanmdsp into a DSP.
#ifndef GUAMP_WIFI_FIRMWARE_INC
#define GUAMP_WIFI_FIRMWARE_INC
namespace guampwifi {
struct Ext4FirmwareSource {
  ext4::Fs *fs=nullptr;
  const char *root="/image/";
  static constexpr uint64_t MaxFileBytes=64ull*1024*1024;

  explicit Ext4FirmwareSource(ext4::Fs *mounted=nullptr,const char *under="/image/"):fs(mounted),root(under) {}

  static bool equals(const char *a,const char *b) {
    if(!a||!b) return false;
    while(*a&&*b&&*a==*b) { ++a; ++b; }
    return *a==*b;
  }
  // Any file under /image of the read-only modem firmware partition (what tqftpserv's translate_readonly serves from
  // the firmware directory): no "..", no empty component, nothing outside /image.
  static bool allowed(const char *path) { return allowedUnder(path,"/image/"); }
  // guamp 08/11: the same rules under another directory ending in '/' (the persist copy of the modem's /readwrite files).
  static bool allowedUnder(const char *path,const char *under) {
    if(!path||!under||!under[0]) return false;
    uint32_t n=0; for(;under[n];n++) if(path[n]!=under[n]) return false;
    if(under[n-1]!='/'||!path[n]) return false;
    for(const char *c=path+n-1;*c;) {
      if(*c=='/') { c++; if(*c=='/'||!*c) return false; if(c[0]=='.'&&c[1]=='.'&&(c[2]=='/'||!c[2])) return false; }
      else c++;
    }
    return true;
  }

  int size(const char *path,uint32_t *bytes) {
    if(!fs||!path||!bytes||!allowedUnder(path,root)) return Invalid;
    ext4::Inode ino{}; int e=fs->lookupPath(path,&ino);
    if(e) return e==ext4::kErrNotFound?NotFound:(e==ext4::kErrIo?Io:Unsupported);
    if(!ino.isFile()||!ino.size||ino.size>MaxFileBytes||ino.size>0xffffffffull) return Range;
    *bytes=static_cast<uint32_t>(ino.size); return Ok;
  }

  int read(const char *path,uint32_t offset,uint8_t *out,uint32_t bytes) {
    if(!fs||!path||(!out&&bytes)||!allowedUnder(path,root)||bytes>6144) return Invalid;
    ext4::Inode ino{}; int e=fs->lookupPath(path,&ino);
    if(e) return e==ext4::kErrNotFound?NotFound:(e==ext4::kErrIo?Io:Unsupported);
    if(!ino.isFile()||ino.size>MaxFileBytes||offset>ino.size||bytes>ino.size-offset) return Range;
    if(!bytes) return Ok;
    e=fs->readFile(ino,offset,out,bytes);
    return e==ext4::kOk?Ok:(e==ext4::kErrRange?Range:Io);
  }

  static int sizeCallback(void *ctx,const char *path,uint32_t *bytes) {
    return ctx?static_cast<Ext4FirmwareSource *>(ctx)->size(path,bytes):Invalid;
  }
  static int readCallback(void *ctx,const char *path,uint32_t offset,uint8_t *out,uint32_t bytes) {
    return ctx?static_cast<Ext4FirmwareSource *>(ctx)->read(path,offset,out,bytes):Invalid;
  }
};
} // namespace guampwifi
#endif
