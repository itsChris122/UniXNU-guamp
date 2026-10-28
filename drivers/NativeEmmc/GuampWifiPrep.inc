// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Wi-Fi preparation, included by NativeEmmc.cpp after PmicService.inc (card, gate, IOLog). Read-only checks for
// the later Wi-Fi steps, all opt-in with the boot argument guamp-wifi-prep=1, run once by one kernel thread after the
// eMMC devices are published:
//   SMEM       0x46000000 (2 MiB, bengal.dtsi smem_region, not XNU RAM) mapped uncached and read-only for the duration
//              of the check: version, partitions, the items of the apps partitions, items 421/428/435/478-480 and
//              the modem SMP2P inbound state (GuampSmem.inc). Expected before any modem boot: no 428, no GLINK items.
//   firmware   the modem_<slot> partition (slot from the boot_a/boot_b GPT active bit, boot argument
//              guamp-wifi-slot=a|b overrides) read as ext4 (GuampExt4.inc); /image/modem.mdt parsed and every
//              /image/modem.bNN checked against its SHA-384 (GuampFirmware.inc); presence of wlanmdsp.mbn and bdwlan.*
// The default prep path only reads. guamp-modem-start=2 adds SCM/PAS queries;
// mode1 can load/authenticate reserved modem RAM (GuampModem.inc over the shared PAS loader).
// guamp-wifi-start=1 (implies the prep) starts the modem through ModemBootHooks into the
// Wi-Fi runtime (drivers/GuampWifi, compiled only with GUAMP_WIFI_RUNTIME=1): diagnostic scan.
// eMMC stays read-only, through Card::readBlocks under the card gate.

#include "GuampExt4.inc"
#include "GuampWifiWire.inc"
#include "GuampFirmware.inc"
#include "GuampWifiFirmware.inc"
#include "GuampSmem.inc"
#include "GuampScm.inc"
#include "GuampScmArm64.inc"
#include "GuampPasCore.inc"   // one PAS loader for modem (GuampModem.inc) and ADSP (GuampAdsp.inc)
#include "GuampPas.inc"
#include "GuampWifiControl.inc"
#if defined(GUAMP_WIFI_RUNTIME) && GUAMP_WIFI_RUNTIME
#include "guamp_xnu_sm6115_runtime.h"
#endif

extern "C" {
extern unsigned long guamp_phys_base __asm__("_gPhysBase");   // XNU-managed RAM (osfmk/arm64/arm_vm_init.c),
extern unsigned long guamp_phys_size __asm__("_gPhysSize");   // declared only for MACH_KERNEL_PRIVATE
}

namespace wifiprep {

constexpr uint64_t kSmemPhys = 0x46000000, kSmemBytes = 0x200000;
constexpr uint32_t kChunkBytes = kBounceBlocks * kBlock;   // 64 KiB per firmware read

struct Window { uint64_t first, blocks; };

bool readWindow(void *ctx, uint64_t sector, uint32_t count, uint8_t *out) {
  const Window *w = static_cast<const Window *>(ctx);
  if (!count || count > kBounceBlocks || sector >= w->blocks || count > w->blocks - sector) return false;
  Card &card = gCard;
  IOLockLock(card.gate);
  bool ok = card.online && card.readBlocks(w->first + sector, count, out);
  IOLockUnlock(card.gate);
  return ok;
}

// Persistent, read-only backing for later QRTR/TFTP and WLFW requests. The
// diagnostic path below used to free its ext4 scratch/window on return, which
// made an asynchronous firmware callback unsafe even if WifiControl were
// started later. This owner deliberately survives the prep thread.
struct FirmwareStore {
  static constexpr uint32_t Wcn3990BoardBytes=26328;
  Window window{};
  ext4::Fs fs{};
  IOLock *gate=nullptr;
  uint8_t *scratch=nullptr;
  uint8_t *ath10kBoard=nullptr;
  uint32_t ath10kBoardBytes=0;
  bool ready=false;
  bool isReady() const { return __atomic_load_n(&ready,__ATOMIC_ACQUIRE); }
  int mount(uint64_t first,uint64_t blocks) {
    if(isReady())return guampwifi::Ok;
    if(!blocks)return guampwifi::Invalid;
    if(!gate)gate=IOLockAlloc();
    if(!gate)return guampwifi::NoMemory;
    IOLockLock(gate);
    if(isReady()){IOLockUnlock(gate);return guampwifi::Ok;}
    if(!scratch)scratch=static_cast<uint8_t *>(IOMallocZero(ext4::kScratchBytes));
    if(!scratch){IOLockUnlock(gate);return guampwifi::NoMemory;}
    window={first,blocks};
    int e=fs.mount(readWindow,&window,blocks,scratch);
    if(e){memset(&fs,0,sizeof(fs));IOLockUnlock(gate);return e==ext4::kErrIo?guampwifi::Io:guampwifi::Unsupported;}
    __atomic_store_n(&ready,true,__ATOMIC_RELEASE);
    IOLockUnlock(gate);return guampwifi::Ok;
  }
  int sizeLocked(const char *path,uint32_t *bytes) {
    guampwifi::Ext4FirmwareSource source(&fs);return source.size(path,bytes);
  }
  int readLocked(const char *path,uint32_t at,uint8_t *out,uint32_t bytes) {
    guampwifi::Ext4FirmwareSource source(&fs);return source.read(path,at,out,bytes);
  }
  int size(const char *path,uint32_t *bytes) {
    if(!isReady())return guampwifi::NotReady;
    IOLockLock(gate);int e=isReady()?sizeLocked(path,bytes):guampwifi::NotReady;IOLockUnlock(gate);return e;
  }
  int read(const char *path,uint32_t at,uint8_t *out,uint32_t bytes) {
    if(!isReady())return guampwifi::NotReady;
    IOLockLock(gate);int e=isReady()?readLocked(path,at,out,bytes):guampwifi::NotReady;IOLockUnlock(gate);return e;
  }
  int loadAth10kBoard() {
    if(!isReady())return guampwifi::NotReady;
    IOLockLock(gate);
    if(!isReady()){IOLockUnlock(gate);return guampwifi::NotReady;}
    if(ath10kBoard&&ath10kBoardBytes){IOLockUnlock(gate);return guampwifi::Ok;}
    uint32_t n=0;int e=sizeLocked("/image/bdwlan_guamp_ipa.bin",&n);
    if(e){IOLockUnlock(gate);return e;}
    if(n!=Wcn3990BoardBytes){IOLockUnlock(gate);return guampwifi::Range;}
    uint8_t *data=static_cast<uint8_t *>(IOMalloc(n));
    if(!data){IOLockUnlock(gate);return guampwifi::NoMemory;}
    for(uint32_t at=0;at<n;) {
      uint32_t chunk=n-at;if(chunk>6144)chunk=6144;
      e=readLocked("/image/bdwlan_guamp_ipa.bin",at,data+at,chunk);
      if(e){IOFree(data,n);IOLockUnlock(gate);return e;}
      at+=chunk;
    }
    ath10kBoard=data;ath10kBoardBytes=n;IOLockUnlock(gate);return guampwifi::Ok;
  }
  static int sizeCallback(void *ctx,const char *path,uint32_t *bytes) {
    return ctx?static_cast<FirmwareStore *>(ctx)->size(path,bytes):guampwifi::Invalid;
  }
  static int readCallback(void *ctx,const char *path,uint32_t at,uint8_t *out,uint32_t bytes) {
    return ctx?static_cast<FirmwareStore *>(ctx)->read(path,at,out,bytes):guampwifi::Invalid;
  }
};
static FirmwareStore gFirmwareStore{};

// lx_platform::fw_get has no context. Keep this callback scoped to the one
// guamp SNOC instance. firmware-5.bin is the 60-byte upstream WCN3990 API5
// metadata record (not executable firmware); WLAN.HL payload remains the
// separately loaded wlanmdsp.mbn. The Motorola raw BDF is served through API1.
int ath10kFirmwareGet(const char *name,const uint8_t **data,size_t *bytes) {
  if(!name||!data||!bytes)return guampwifi::Invalid;
  const char *base=name;
  for(const char *p=name;*p;p++)if(*p=='/'||*p=='\\')base=p+1;
  if(guampwifi::Ext4FirmwareSource::equals(base,"firmware-5.bin")) {
    // linux-firmware ath10k/WCN3990/hw1.0/firmware-5.bin, retrieved from
    // linux-firmware.git. WHENCE marks it redistributable under
    // LicenseRef-LICENSE.QualcommAtheros_ath10k. It declares NON_BMI,
    // WMI TLV and HTT v3; it does not carry a WLAN firmware image.
    static const uint8_t metadata[]={
      0x51,0x43,0x41,0x2d,0x41,0x54,0x48,0x31,0x30,0x4b,0x00,0x77,
      0x01,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0xa4,0xe4,0xbe,0x5b,
      0x02,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x40,0x00,0x0c,0x77,
      0x05,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x04,0x00,0x00,0x00,
      0x06,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x03,0x00,0x00,0x00
    };
    *data=metadata;*bytes=sizeof(metadata);return guampwifi::Ok;
  }
  if(!guampwifi::Ext4FirmwareSource::equals(base,"board-snoc-guamp-wcn-snoc.bin")&&
     !guampwifi::Ext4FirmwareSource::equals(base,"board.bin"))return guampwifi::NotFound;
  int e=gFirmwareStore.loadAth10kBoard();if(e)return e;
  *data=gFirmwareStore.ath10kBoard;*bytes=gFirmwareStore.ath10kBoardBytes;
  return guampwifi::Ok;
}

// GPT entry by name (same walk as Card::findPartitions, own buffer, one block per gate hold)
bool findPartition(const char *wanted, uint64_t *first, uint64_t *blocks, uint64_t *attributes) {
  Card &card = gCard;
  uint8_t *block = static_cast<uint8_t *>(IOMallocZero(kBlock));
  if (!block) return false;
  bool found = false;
  IOLockLock(card.gate);
  bool ok = card.readBlocks(1, 1, block) && !memcmp(block, "EFI PART", 8);
  IOLockUnlock(card.gate);
  uint64_t entriesLba = 0;
  uint32_t entries = 0, entrySize = 0;
  if (ok) {
    memcpy(&entriesLba, block + 72, 8);
    memcpy(&entries, block + 80, 4);
    memcpy(&entrySize, block + 84, 4);
    ok = entrySize >= 128 && entrySize <= kBlock && !(kBlock % entrySize) && entries <= 1024;
  }
  const uint32_t perBlock = ok ? kBlock / entrySize : 1;
  for (uint32_t i = 0; ok && !found && i < entries; i++) {
    if (i % perBlock == 0) {
      IOLockLock(card.gate);
      ok = card.readBlocks(entriesLba + i / perBlock, 1, block);
      IOLockUnlock(card.gate);
      if (!ok) break;
    }
    const uint8_t *entry = block + (i % perBlock) * entrySize;
    char name[16] = {};
    for (unsigned c = 0; c + 1 < sizeof(name); c++) {
      uint16_t unit = static_cast<uint16_t>(entry[56 + 2 * c] | (entry[57 + 2 * c] << 8));
      if (!unit || unit > 0x7f) break;
      name[c] = static_cast<char>(unit);
    }
    uint64_t start, last;
    memcpy(&start, entry + 32, 8);
    memcpy(&last, entry + 40, 8);
    if (strcmp(name, wanted) || start < 34 || last < start || last >= card.capacity) continue;
    *first = start;
    *blocks = last - start + 1;
    memcpy(attributes, entry + 48, 8);
    found = true;
  }
  IOFree(block, kBlock);
  return found;
}

// Active slot: Qualcomm A/B attribute bit 50 (active) on boot_a / boot_b
char activeSlot() {
  char arg[4] = {};
  if (PE_parse_boot_argn("guamp-wifi-slot", arg, sizeof(arg)) && (arg[0] == 'a' || arg[0] == 'b')) return arg[0];
  uint64_t first, blocks, attrA = 0, attrB = 0;
  bool a = findPartition("boot_a", &first, &blocks, &attrA), b = findPartition("boot_b", &first, &blocks, &attrB);
  if (b && (attrB >> 50 & 1) && !(a && (attrA >> 50 & 1))) return 'b';
  return 'a';
}

static bool modemStopped();
static bool modemAwaitReady();
#include "GuampModem.inc"

static unsigned radioMode() {
  unsigned mode=0;
  if(!PE_parse_boot_argn("guamp-wifi-start",&mode,sizeof(mode))||mode>2)return 0;
  return mode;
}
#if defined(GUAMP_WIFI_RUNTIME) && GUAMP_WIFI_RUNTIME
static int runtimePrepare(void *) { return guamp_sm6115_wifi_prepare(guamp_phys_base,guamp_phys_size); }
static int runtimeStarted(void *) { return guamp_sm6115_wifi_modem_started(); }
static void runtimeShutdown(void *,bool stopped) { guamp_sm6115_wifi_shutdown_result(stopped); }
#endif

// --- SMEM -----------------------------------------------------------------------------------------------------------
uint32_t smemRead(void *ctx, uint32_t offset) {
  return *reinterpret_cast<const volatile uint32_t *>(static_cast<const volatile uint8_t *>(ctx) + offset);
}

static bool modemStopped() {
  if(pas::overlap(kSmemPhys,kSmemBytes,guamp_phys_base,guamp_phys_size))return false;
  IOMemoryDescriptor *d=IOMemoryDescriptor::withPhysicalAddress(kSmemPhys,kSmemBytes,kIODirectionIn);
  IOMemoryMap *map=d?d->map(kIOMapInhibitCache|kIOMapReadOnly):nullptr;if(d)d->release();
  if(!map)return false;
  smem::Info *info=static_cast<smem::Info *>(IOMallocZero(sizeof(smem::Info)));
  bool stopped=false;
  if(info) {
    smem::Reader reader={smemRead,reinterpret_cast<void *>(map->getVirtualAddress()),static_cast<uint32_t>(kSmemBytes)};
    smem::parse(reader,info);uint32_t at=0,bytes=0;
    const smem::Partition *peer=smem::partitionFor(*info,smem::kHostModem);
    stopped=!info->error && info->initialized && (!peer || !peer->status) &&
      !smem::find(reader,*info,smem::kHostModem,428,&at,&bytes);
    IOFree(info,sizeof(*info));
  }
  map->release();return stopped;
}

static bool modemAwaitReady() {
  if(pas::overlap(kSmemPhys,kSmemBytes,guamp_phys_base,guamp_phys_size))return false;
  IOMemoryDescriptor *d=IOMemoryDescriptor::withPhysicalAddress(kSmemPhys,kSmemBytes,kIODirectionIn);
  IOMemoryMap *map=d?d->map(kIOMapInhibitCache|kIOMapReadOnly):nullptr;if(d)d->release();
  if(!map)return false;
  smem::Info *info=static_cast<smem::Info *>(IOMallocZero(sizeof(smem::Info)));
  smem::Smp2p *state=static_cast<smem::Smp2p *>(IOMallocZero(sizeof(smem::Smp2p)));
  bool ready=false;
  if(info && state) {
    smem::Reader reader={smemRead,reinterpret_cast<void *>(map->getVirtualAddress()),static_cast<uint32_t>(kSmemBytes)};
    uint32_t last=~0u;
    for(unsigned attempt=0;attempt<100;++attempt) {
      __asm__ volatile("dmb sy" ::: "memory");
      smem::parse(reader,info);uint32_t at=0,bytes=0;
      if(!info->error && smem::find(reader,*info,smem::kHostModem,428,&at,&bytes)) {
        smem::readSmp2p(reader,at,bytes,"slave-kernel",state);
        if(state->valid && state->hasEntry && state->localPid==1 && state->remotePid==0) {
          uint32_t flags=state->value;
          if(flags!=last) {IOLog("GuampModem: SMP2P slave-kernel %x (ready %u fatal %u)\n",flags,flags>>1&1,flags&1);last=flags;}
          if(flags&1)break;
          if(flags&2) {ready=true;break;}
        }
      }
      IOSleep(100);
    }
    if(!ready)IOLog("GuampModem: ready handshake not complete after bounded observation; apps SMP2P/GLINK prerequisites remain\n");
  }
  if(info)IOFree(info,sizeof(*info));if(state)IOFree(state,sizeof(*state));map->release();return ready;
}

void smemReport() {
  if (kSmemPhys + kSmemBytes > guamp_phys_base && kSmemPhys < guamp_phys_base + guamp_phys_size) {
    IOLog("GuampSmem: region overlaps XNU RAM (%lx + %lx), not mapped\n", guamp_phys_base, guamp_phys_size);
    return;
  }
  IOMemoryDescriptor *range = IOMemoryDescriptor::withPhysicalAddress(kSmemPhys, kSmemBytes, kIODirectionIn);
  IOMemoryMap *map = range ? range->map(kIOMapInhibitCache | kIOMapReadOnly) : nullptr;
  if (range) range->release();
  if (!map) { IOLog("GuampSmem: mapping failed\n"); return; }
  smem::Reader r = {smemRead, reinterpret_cast<void *>(map->getVirtualAddress()), static_cast<uint32_t>(kSmemBytes)};
  smem::Info *info = static_cast<smem::Info *>(IOMallocZero(sizeof(smem::Info)));
  smem::Smp2p *sp = static_cast<smem::Smp2p *>(IOMallocZero(sizeof(smem::Smp2p)));
  if (info && sp) {
    smem::parse(r, info);
    IOLog("GuampSmem: version %u, initialized %u, ptable v%u, %u partitions, %u items max, %s\n", info->version,
          info->initialized, info->ptableVersion, info->partitions, info->numItems,
          info->error ? "PARSE ERROR" : "ok");
    for (uint32_t i = 0; i < info->partitions; i++) {
      const smem::Partition &p = info->part[i];
      char ids[6 * smem::kMaxIds + 8];
      uint32_t n = 0;
      for (uint32_t k = 0; p.read && k < p.items && k < smem::kMaxIds && n + 7 < sizeof(ids); k++) {
        uint32_t v = p.ids[k];
        ids[n++] = ' ';
        char digits[6];
        uint32_t d = 0;
        do { digits[d++] = static_cast<char>('0' + v % 10); v /= 10; } while (v && d < 5);
        while (d) ids[n++] = digits[--d];
      }
      ids[n] = 0;
      IOLog("GuampSmem: partition %u hosts %x-%x at %x size %x cacheline %u: %s %u items%s%s\n", i, p.host0, p.host1,
            p.offset, p.size, p.cacheline, !p.read ? "not apps, not read," : p.status ? "WALK ERROR," : "",
            p.items, p.items ? ":" : "", ids);
    }
    static const uint32_t wanted[] = {421, 428, 435, 478, 479, 480};
    uint32_t at[6] = {}, size[6] = {};
    bool found[6] = {};
    for (unsigned k = 0; k < 6 && !info->error; k++) found[k] = smem::find(r, *info, smem::kHostModem, wanted[k], &at[k], &size[k]);
    IOLog("GuampSmem: items crash-reason 421 %s %u, smp2p in 428 %s %u, smp2p out 435 %s %u, glink 478/479/480 %s/%s/%s\n",
          found[0] ? "present" : "absent", size[0], found[1] ? "present" : "absent", size[1],
          found[2] ? "present" : "absent", size[2], found[3] ? "present" : "absent", found[4] ? "present" : "absent",
          found[5] ? "present" : "absent");
    if (!found[1]) {
      IOLog("GuampSmem: modem SMP2P 428 absent: modem not running (expected)\n");
    } else {
      smem::readSmp2p(r, at[1], size[1], "slave-kernel", sp);
      uint32_t v = sp->value;
      IOLog("GuampSmem: modem SMP2P 428 %s, pids %u->%u, %u/%u entries, slave-kernel %s %x (fatal %u ready %u handover %u stop-ack %u shutdown-ack %u)\n",
            sp->valid ? "valid" : "INVALID", sp->localPid, sp->remotePid, sp->validEntries, sp->total,
            sp->hasEntry ? "=" : "missing", v, v & 1, v >> 1 & 1, v >> 2 & 1, v >> 3 & 1, v >> 7 & 1);
    }
  }
  if (info) IOFree(info, sizeof(smem::Info));
  if (sp) IOFree(sp, sizeof(smem::Smp2p));
  map->release();   // unmaps
}

// --- modem firmware -------------------------------------------------------------------------------------------------
void firmwareReport() {
  char part[8] = {'m', 'o', 'd', 'e', 'm', '_', activeSlot(), 0};
  Window window = {};
  uint64_t attributes = 0;
  if (!findPartition(part, &window.first, &window.blocks, &attributes)) {
    IOLog("GuampWifi: partition %s not found\n", part);
    return;
  }
  uint8_t *scratch = static_cast<uint8_t *>(IOMallocZero(ext4::kScratchBytes));
  uint8_t *mdt = static_cast<uint8_t *>(IOMallocZero(fw::kMdtMax));
  uint8_t *chunk = static_cast<uint8_t *>(IOMallocZero(kChunkBytes));
  ext4::Fs *fs = static_cast<ext4::Fs *>(IOMallocZero(sizeof(ext4::Fs)));
  fw::Result *res = static_cast<fw::Result *>(IOMallocZero(sizeof(fw::Result)));
  if (scratch && mdt && chunk && fs && res) {
    const uint64_t began = mach_absolute_time();
    int r = fs->mount(readWindow, &window, window.blocks, scratch);
    IOLog("GuampExt4: %s at LBA %llu, %llu sectors: %s, %llu blocks of %u, %u groups, inode %u bytes, features %x/%x/%x%s%x\n",
          part, window.first, window.blocks, ext4::errorName(r), fs->blocksCount, fs->blockSize, fs->groups,
          fs->inodeSize, fs->compat, fs->incompat, fs->roCompat, r == ext4::kErrUnsupported ? " refused " : " ",
          fs->refused);
    if (!r) {
      fw::verify(*fs, "/image", "modem", mdt, chunk, kChunkBytes, res);
      uint64_t us = 0;
      absolutetime_to_nanoseconds(mach_absolute_time() - began, &us);
      us /= 1000;
      if (res->error) {
        IOLog("GuampWifi: modem firmware not checked: %s (%d)\n",
              res->error < -19 ? "bad mdt or hash segment" : ext4::errorName(res->error), res->error);
      } else {
        IOLog("GuampWifi: modem firmware %u segments verified/%u failed, %llu bytes (%u program headers, MBN v%u, %u empty, failed mask %llx, %llu ms)\n",
              res->verified, res->failed, res->bytes, res->segments, res->mbnVersion, res->empty, res->failedMask,
              us / 1000);
      }
      // WLAN firmware the later QMI step hands over (presence only)
      ext4::Inode dir, file;
      uint32_t bdwlan = 0;
      int w = fs->lookupPath("/image/wlanmdsp.mbn", &file);
      if (!fs->lookupPath("/image", &dir))
        fs->forEachEntry(dir, [&](const char *name, uint32_t len, uint32_t, uint8_t) {
          if (len >= 7 && !memcmp(name, "bdwlan.", 7)) bdwlan++;
          return false;
        });
      IOLog("GuampWifi: wlanmdsp.mbn %s %llu bytes, %u bdwlan.* board files\n", w ? "absent" : "present",
            w ? 0ull : file.size, bdwlan);
      const bool verified=!res->error&&!res->failed&&res->verified>0;
      const unsigned mode=radioMode();
      if(mode) {
        // Only the radio opt-in keeps a mounted store past this thread (guamp-wifi-prep=1 alone stays as before).
        int persistent=gFirmwareStore.mount(window.first,window.blocks);
        IOLog("GuampWifi: persistent read-only firmware source %s; QRTR TFTP remains opt-in to the transport owner\n",
              persistent?"unavailable":"ready");
#if defined(GUAMP_WIFI_RUNTIME) && GUAMP_WIFI_RUNTIME
#if !defined(GUAMP_WIFI_REDWOOD) || !GUAMP_WIFI_REDWOOD
        if(mode==2)IOLog("GuampWifi: guamp-wifi-start=2 (iOS station) is not built (GUAMP_WIFI_REDWOOD); modem not started\n");
        else
#endif
        if(persistent||w||!verified)IOLog("GuampWifi: radio start refused: firmware store, WLAN image or verification unavailable\n");
        else {
          // The persistent store precedes PAS boot: QFTP may request firmware
          // while the started hook is negotiating GLINK/QRTR/WLFW.
          const ModemBootHooks hooks={nullptr,runtimePrepare,runtimeStarted,runtimeShutdown};
          if(modemBoot(*fs,mdt,chunk,verified,&hooks,true)) {
            const int e=guamp_sm6115_wifi_start_station(mode);
            if(e)IOLog("GuampWifi: station worker could not start %d; live modem resources retained\n",e);
          } else guamp_sm6115_wifi_abort();
        }
#else
        IOLog("GuampWifi: guamp-wifi-start requires the GUAMP_WIFI_RUNTIME component; prep completed only\n");
#endif
      } else (void)modemBoot(*fs,mdt,chunk,verified);
    }
  } else {
    IOLog("GuampWifi: out of memory\n");
  }
  if (scratch) IOFree(scratch, ext4::kScratchBytes);
  if (mdt) IOFree(mdt, fw::kMdtMax);
  if (chunk) IOFree(chunk, kChunkBytes);
  if (fs) IOFree(fs, sizeof(ext4::Fs));
  if (res) IOFree(res, sizeof(fw::Result));
}

void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-wifi-prep");
  smemReport();
  firmwareReport();
  unsigned modemMode=0;
  if(PE_parse_boot_argn("guamp-modem-start",&modemMode,sizeof(modemMode)) && modemMode==1)smemReport();
}

// Once, after publish(): read-only prep or explicit radio-start opt-in.
void start() {
  static bool started = false;
  unsigned enabled = 0;
  const bool prep=PE_parse_boot_argn("guamp-wifi-prep",&enabled,sizeof(enabled))&&enabled==1;
  if(started||(!prep&&!radioMode()))return;
  if (!gCard.online || !gCard.gate) return;
  started = true;
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}

}  // namespace wifiprep

// C ABI consumed by the XNU station glue. Preserve the bounded error codes so
// the glue can fall back to the caller's firmware provider on NotFound/NotReady.
extern "C" int guamp_ath10k_firmware_get(const char *name,const uint8_t **data,size_t *bytes) {
  return wifiprep::ath10kFirmwareGet(name,data,bytes);
}

// Narrow C ABI for the WLFW QFTP provider callbacks. QFTP itself accepts only
// these two read-only assets; do not expose an arbitrary ext4 path to callers.
extern "C" int guamp_wifi_firmware_file_size(const char *path,uint32_t *bytes) {
  if(!path||!bytes)return guampwifi::Invalid;
  if(strcmp(path,"/image/wlanmdsp.mbn")&&strcmp(path,"/image/bdwlan_guamp_ipa.bin"))
    return guampwifi::NotFound;
  return wifiprep::gFirmwareStore.size(path,bytes);
}
extern "C" int guamp_wifi_firmware_file_read(const char *path,uint32_t at,uint8_t *out,uint32_t bytes) {
  if(!path||(!out&&bytes)||!bytes)return guampwifi::Invalid;
  if(strcmp(path,"/image/wlanmdsp.mbn")&&strcmp(path,"/image/bdwlan_guamp_ipa.bin"))
    return guampwifi::NotFound;
  return wifiprep::gFirmwareStore.read(path,at,out,bytes);
}
