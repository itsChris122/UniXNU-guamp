// SPDX-License-Identifier: BSD-2-Clause-Patent
// Read-only, allow-listed modem-partition firmware source for WLFW adapters.
// This is not a PDR/TFTP service and does not load wlanmdsp into a DSP.
#ifndef GUAMP_WIFI_FIRMWARE_INC
#define GUAMP_WIFI_FIRMWARE_INC
namespace guampwifi {
struct Ext4FirmwareSource {
  ext4::Fs *fs=nullptr;
  static constexpr uint64_t MaxFileBytes=64ull*1024*1024;

  explicit Ext4FirmwareSource(ext4::Fs *mounted=nullptr):fs(mounted) {}

  static bool equals(const char *a,const char *b) {
    if(!a||!b) return false;
    while(*a&&*b&&*a==*b) { ++a; ++b; }
    return *a==*b;
  }
  static bool allowed(const char *path) {
    return equals(path,"/image/bdwlan_guamp_ipa.bin") || equals(path,"/image/wlanmdsp.mbn");
  }

  int size(const char *path,uint32_t *bytes) {
    if(!fs||!path||!bytes||!allowed(path)) return Invalid;
    ext4::Inode ino{}; int e=fs->lookupPath(path,&ino);
    if(e) return e==ext4::kErrNotFound?NotFound:(e==ext4::kErrIo?Io:Unsupported);
    if(!ino.isFile()||!ino.size||ino.size>MaxFileBytes||ino.size>0xffffffffull) return Range;
    *bytes=static_cast<uint32_t>(ino.size); return Ok;
  }

  int read(const char *path,uint32_t offset,uint8_t *out,uint32_t bytes) {
    if(!fs||!path||(!out&&bytes)||!allowed(path)||bytes>6144) return Invalid;
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
