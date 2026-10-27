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
// Nothing is written: not the eMMC, not SMEM, not the modem region; no TrustZone call. eMMC reads go through
// Card::readBlocks under the card gate, at most kBounceBlocks (128) blocks per call, like every other reader.

#include "GuampExt4.inc"
#include "GuampFirmware.inc"
#include "GuampSmem.inc"

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

// --- SMEM -----------------------------------------------------------------------------------------------------------
uint32_t smemRead(void *ctx, uint32_t offset) {
  return *reinterpret_cast<const volatile uint32_t *>(static_cast<const volatile uint8_t *>(ctx) + offset);
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
}

// Once, after publish(): boot argument guamp-wifi-prep=1
void start() {
  static bool started = false;
  unsigned enabled = 0;
  if (started || !PE_parse_boot_argn("guamp-wifi-prep", &enabled, sizeof(enabled)) || enabled != 1) return;
  if (!gCard.online || !gCard.gate) return;
  started = true;
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}

}  // namespace wifiprep
