// SPDX-License-Identifier: BSD-2-Clause-Patent
// Minimal read-only ext4 reader for the guamp Wi-Fi preparation (GuampWifiPrep.inc), host-tested by
// tests/wifi_host.cpp against the backup modem_a.img. Pure code: no kernel or libc calls besides memcpy/memset/memcmp,
// integer math only; the owner supplies a sector reader (512-byte sectors relative to the file system start, at most
// kMaxSectors per call) and kScratchBytes of scratch memory.
//   superblock  byte 1024; block size 1, 2 or 4 KiB; revision 0 or 1
//   features    incompat allowed: filetype, extents, 64bit, flex_bg, mmp, csum_seed, largedir (anything else, e.g.
//               needs_recovery, meta_bg, inline_data, encrypt, casefold, compression, is refused at mount);
//               ro_compat: everything except bigalloc (checksums are not checked)
//   inodes      group descriptors right after the superblock block (32 bytes, or s_desc_size with 64bit)
//   files       extent trees of any depth (<= 5), holes and uninitialized extents read as zeros; block-mapped,
//               inline-data and encrypted inodes are refused per file
//   directories linear scan of the data blocks (htree directories read the same way: their index blocks look like
//               one empty entry); path lookup from the root inode, no symlinks
// Never writes.

namespace ext4 {

enum : int {
  kOk = 0, kErrIo = -1, kErrFormat = -2, kErrUnsupported = -3, kErrNotFound = -4, kErrNotDir = -5, kErrRange = -6,
};

inline const char *errorName(int e) {
  switch (e) {
  case kOk: return "ok";
  case kErrIo: return "I/O error";
  case kErrFormat: return "bad format";
  case kErrUnsupported: return "unsupported feature";
  case kErrNotFound: return "not found";
  case kErrNotDir: return "not a directory";
  case kErrRange: return "out of range";
  default: return "error";
  }
}

// count <= kMaxSectors; sector + count must lie inside the device (the reader checks it as well)
typedef bool (*ReadFn)(void *ctx, uint64_t sector, uint32_t count, uint8_t *out);

constexpr uint32_t kSector = 512, kMaxSectors = 128;
constexpr uint32_t kMaxBlock = 4096;
constexpr uint32_t kMaxDepth = 5;                                  // ext4 extent trees never go deeper
constexpr uint32_t kScratchBytes = (kMaxDepth + 2) * kMaxBlock;   // one block per tree level, a directory block, a temp

constexpr uint32_t kIncompatFiletype = 0x2, kIncompatRecover = 0x4, kIncompatMetaBg = 0x10, kIncompatExtents = 0x40,
                   kIncompat64bit = 0x80, kIncompatMmp = 0x100, kIncompatFlexBg = 0x200, kIncompatCsumSeed = 0x2000,
                   kIncompatLargeDir = 0x4000, kIncompatInlineData = 0x8000;
constexpr uint32_t kIncompatSupported = kIncompatFiletype | kIncompatExtents | kIncompat64bit | kIncompatMmp |
                                        kIncompatFlexBg | kIncompatCsumSeed | kIncompatLargeDir;
constexpr uint32_t kRoCompatBigalloc = 0x200;

constexpr uint32_t kFlagEncrypt = 0x800, kFlagExtents = 0x80000, kFlagInlineData = 0x10000000;
constexpr uint16_t kModeType = 0xF000, kModeDir = 0x4000, kModeFile = 0x8000;
constexpr uint32_t kRootInode = 2;

inline uint16_t le16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline uint32_t le32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

struct Inode {
  uint32_t number;
  uint16_t mode;
  uint32_t flags;
  uint64_t size;
  uint8_t block[60];   // i_block: extent tree root
  bool isDir() const { return (mode & kModeType) == kModeDir; }
  bool isFile() const { return (mode & kModeType) == kModeFile; }
};

struct Fs {   // plain data: zero it (IOMallocZero or = {}), then mount()
  ReadFn read;
  void *ctx;
  uint8_t *scratch;
  uint64_t sectors;                           // device size
  uint32_t blockSize, sectorsPerBlock, inodeSize, inodesPerGroup, blocksPerGroup;
  uint32_t descSize, groups, firstDataBlock;
  uint64_t blocksCount, inodesCount;
  uint32_t compat, incompat, roCompat;
  uint32_t refused;                           // unsupported feature bits seen by mount
  uint64_t sectorsRead;
  uint64_t levelBlock[kMaxDepth];             // block held by each tree level buffer (0 = none)

  uint8_t *levelBuf(uint32_t level) { return scratch + level * kMaxBlock; }
  uint8_t *dirBuf() { return scratch + kMaxDepth * kMaxBlock; }
  uint8_t *tmpBuf() { return scratch + (kMaxDepth + 1) * kMaxBlock; }

  bool readSectors(uint64_t sector, uint32_t count, uint8_t *out) {
    while (count) {
      uint32_t n = count < kMaxSectors ? count : kMaxSectors;
      if (sector > sectors || n > sectors - sector || !read(ctx, sector, n, out)) return false;
      sectorsRead += n;
      sector += n;
      out += static_cast<uint64_t>(n) * kSector;
      count -= n;
    }
    return true;
  }
  bool readBlocks(uint64_t block, uint32_t count, uint8_t *out) {
    if (block >= blocksCount || count > blocksCount - block) return false;
    return readSectors(block * sectorsPerBlock, count * sectorsPerBlock, out);
  }

  int mount(ReadFn reader, void *context, uint64_t deviceSectors, uint8_t *scratchMemory) {
    read = reader; ctx = context; sectors = deviceSectors; scratch = scratchMemory;
    for (uint32_t l = 0; l < kMaxDepth; l++) levelBlock[l] = 0;
    blocksCount = 0;
    uint8_t *sb = tmpBuf();
    if (!scratch || deviceSectors < 4 || !readSectors(2, 2, sb)) return kErrIo;
    if (le16(sb + 56) != 0xEF53) return kErrFormat;
    uint32_t logBlock = le32(sb + 24);
    if (logBlock > 2) { refused = 0; return kErrUnsupported; }       // block size above 4 KiB
    blockSize = 1024u << logBlock;
    sectorsPerBlock = blockSize / kSector;
    uint32_t rev = le32(sb + 76);
    if (rev > 1) return kErrUnsupported;
    compat = rev ? le32(sb + 92) : 0;
    incompat = rev ? le32(sb + 96) : 0;
    roCompat = rev ? le32(sb + 100) : 0;
    refused = (incompat & ~kIncompatSupported) | (roCompat & kRoCompatBigalloc);
    if (refused) return kErrUnsupported;
    inodeSize = rev ? le16(sb + 88) : 128;
    if (inodeSize < 128 || inodeSize > blockSize || (inodeSize & (inodeSize - 1))) return kErrFormat;
    blocksPerGroup = le32(sb + 32);
    inodesPerGroup = le32(sb + 40);
    firstDataBlock = le32(sb + 20);
    blocksCount = le32(sb + 4);
    if (incompat & kIncompat64bit) blocksCount |= static_cast<uint64_t>(le32(sb + 0x150)) << 32;
    inodesCount = le32(sb + 0);
    descSize = 32;
    if (incompat & kIncompat64bit) {
      descSize = le16(sb + 0xFE);
      if (descSize < 64 || descSize > 1024 || (descSize & (descSize - 1))) return kErrFormat;
    }
    if (!blocksPerGroup || blocksPerGroup > 8 * blockSize || !inodesPerGroup || firstDataBlock != (blockSize == 1024 ? 1u : 0u) ||
        blocksCount <= firstDataBlock || blocksCount > deviceSectors / sectorsPerBlock) {
      blocksCount = 0;
      return kErrFormat;
    }
    uint64_t g = (blocksCount - firstDataBlock + blocksPerGroup - 1) / blocksPerGroup;
    if (g > 0xFFFFFFFFull || inodesCount > g * inodesPerGroup) { blocksCount = 0; return kErrFormat; }
    groups = static_cast<uint32_t>(g);
    return kOk;
  }

  int readInode(uint32_t number, Inode *out) {
    if (!number || number > inodesCount) return kErrRange;
    uint32_t group = (number - 1) / inodesPerGroup, index = (number - 1) % inodesPerGroup;
    if (group >= groups) return kErrRange;
    uint64_t descByte = static_cast<uint64_t>(group) * descSize;
    uint64_t descBlock = firstDataBlock + 1 + descByte / blockSize;
    uint8_t *buf = tmpBuf();
    if (!readBlocks(descBlock, 1, buf)) return kErrIo;
    const uint8_t *desc = buf + descByte % blockSize;
    uint64_t table = le32(desc + 8);
    if (descSize >= 64) table |= static_cast<uint64_t>(le32(desc + 0x28)) << 32;
    uint64_t byte = static_cast<uint64_t>(index) * inodeSize;
    uint64_t block = table + byte / blockSize;
    if (!table || !readBlocks(block, 1, buf)) return kErrIo;
    const uint8_t *raw = buf + byte % blockSize;
    out->number = number;
    out->mode = le16(raw + 0);
    out->flags = le32(raw + 32);
    out->size = le32(raw + 4) | (static_cast<uint64_t>(le32(raw + 108)) << 32);
    memcpy(out->block, raw + 40, 60);
    return kOk;
  }

  // Logical block -> physical block and how many following logical blocks map contiguously (pblk 0 = hole or
  // uninitialized: zeros for *run blocks)
  int mapBlock(const Inode &ino, uint32_t lblk, uint64_t *pblk, uint32_t *run) {
    if (ino.flags & (kFlagInlineData | kFlagEncrypt)) return kErrUnsupported;
    if (!(ino.flags & kFlagExtents)) return kErrUnsupported;   // block-mapped files: not needed for the firmware
    const uint8_t *node = ino.block;
    uint32_t nodeBytes = 60, rootDepth = 0;
    uint64_t limit = 0x100000000ull;                           // first logical block past this node's range
    for (uint32_t level = 0;; level++) {
      if (le16(node) != 0xF30A) return kErrFormat;
      uint32_t entries = le16(node + 2), max = le16(node + 4), depth = le16(node + 6);
      if (entries > max || 12 + 12 * max > nodeBytes) return kErrFormat;
      if (level == 0) {
        rootDepth = depth;
        if (depth > kMaxDepth) return kErrFormat;
      } else if (depth != rootDepth - level) {
        return kErrFormat;
      }
      const uint8_t *e = node + 12;
      if (depth == 0) {
        for (uint32_t i = 0; i < entries; i++, e += 12) {
          uint32_t start = le32(e);
          uint32_t len = le16(e + 4);
          bool uninit = len > 32768;
          if (uninit) len -= 32768;
          if (lblk < start) { *pblk = 0; *run = start - lblk; return kOk; }
          if (static_cast<uint64_t>(lblk) < static_cast<uint64_t>(start) + len) {
            uint64_t phys = (static_cast<uint64_t>(le16(e + 6)) << 32 | le32(e + 8)) + (lblk - start);
            uint32_t n = start + len - lblk;
            if (!uninit && (phys >= blocksCount || n > blocksCount - phys)) return kErrFormat;
            *pblk = uninit ? 0 : phys;
            *run = n;
            return kOk;
          }
        }
        *pblk = 0;
        uint64_t n = limit - lblk;
        *run = n > 0x80000000ull ? 0x80000000u : static_cast<uint32_t>(n);
        return kOk;
      }
      if (!entries) return kErrFormat;
      if (le32(e) > lblk) { *pblk = 0; *run = le32(e) - lblk; return kOk; }
      uint32_t pick = 0;
      while (pick + 1 < entries && le32(e + 12 * (pick + 1)) <= lblk) pick++;
      const uint8_t *idx = e + 12 * pick;
      if (pick + 1 < entries) limit = le32(e + 12 * (pick + 1));
      uint64_t child = static_cast<uint64_t>(le16(idx + 8)) << 32 | le32(idx + 4);
      if (level >= kMaxDepth || !child || child >= blocksCount) return kErrFormat;
      if (levelBlock[level] != child) {
        levelBlock[level] = 0;
        if (!readBlocks(child, 1, levelBuf(level))) return kErrIo;
        levelBlock[level] = child;
      }
      node = levelBuf(level);
      nodeBytes = blockSize;
    }
  }

  int readFile(const Inode &ino, uint64_t offset, uint8_t *out, uint32_t bytes) {
    if (offset > ino.size || bytes > ino.size - offset) return kErrRange;
    while (bytes) {
      uint64_t l = offset / blockSize;
      if (l > 0xFFFFFFFFull) return kErrRange;
      uint32_t inBlock = static_cast<uint32_t>(offset % blockSize);
      uint64_t pblk = 0;
      uint32_t run = 0;
      int r = mapBlock(ino, static_cast<uint32_t>(l), &pblk, &run);
      if (r) return r;
      if (!run) return kErrFormat;
      uint32_t n;
      if (inBlock || bytes < blockSize) {
        n = blockSize - inBlock < bytes ? blockSize - inBlock : bytes;
        if (!pblk) memset(out, 0, n);
        else {
          if (!readBlocks(pblk, 1, tmpBuf())) return kErrIo;
          memcpy(out, tmpBuf() + inBlock, n);
        }
      } else {
        uint32_t blocks = bytes / blockSize;
        uint32_t cap = kMaxSectors / sectorsPerBlock;
        if (blocks > run) blocks = run;
        if (blocks > cap) blocks = cap;
        n = blocks * blockSize;
        if (!pblk) memset(out, 0, n);
        else if (!readBlocks(pblk, blocks, out)) return kErrIo;
      }
      out += n;
      offset += n;
      bytes -= n;
    }
    return kOk;
  }

  // Calls visit(name, nameLength, inode, fileType) for every live entry until it returns true
  template <typename Visit> int forEachEntry(const Inode &dir, Visit visit) {
    if (!dir.isDir()) return kErrNotDir;
    uint64_t blocks = (dir.size + blockSize - 1) / blockSize;
    if (blocks > 0xFFFFFFFFull) return kErrFormat;
    for (uint32_t b = 0; b < blocks; b++) {
      uint64_t pblk = 0;
      uint32_t run = 0;
      int r = mapBlock(dir, b, &pblk, &run);
      if (r) return r;
      if (!pblk) continue;
      uint8_t *buf = dirBuf();
      if (!readBlocks(pblk, 1, buf)) return kErrIo;
      for (uint32_t off = 0; off + 8 <= blockSize;) {
        const uint8_t *d = buf + off;
        uint32_t inode = le32(d), recLen = le16(d + 4), nameLen = d[6];
        if (recLen < 8 || (recLen & 3) || recLen > blockSize - off || 8 + nameLen > recLen) return kErrFormat;
        if (inode && nameLen) {
          if (inode > inodesCount) return kErrFormat;
          if (visit(reinterpret_cast<const char *>(d + 8), nameLen, inode, d[7])) return kOk;
        }
        off += recLen;
      }
    }
    return kOk;
  }

  int lookup(const Inode &dir, const char *name, uint32_t nameLen, uint32_t *inode) {
    uint32_t found = 0;
    int r = forEachEntry(dir, [&](const char *n, uint32_t len, uint32_t ino, uint8_t) {
      if (len != nameLen || memcmp(n, name, len)) return false;
      found = ino;
      return true;
    });
    if (r) return r;
    if (!found) return kErrNotFound;
    *inode = found;
    return kOk;
  }

  // Absolute path such as "/image/modem.mdt"
  int lookupPath(const char *path, Inode *out) {
    int r = readInode(kRootInode, out);
    if (r) return r;
    const char *p = path;
    for (;;) {
      while (*p == '/') p++;
      if (!*p) return kOk;
      const char *end = p;
      while (*end && *end != '/') end++;
      uint32_t len = static_cast<uint32_t>(end - p);
      if (len > 255) return kErrRange;
      if (!out->isDir()) return kErrNotDir;
      uint32_t inode = 0;
      r = lookup(*out, p, len, &inode);
      if (r) return r;
      r = readInode(inode, out);
      if (r) return r;
      p = end;
    }
  }
};

}  // namespace ext4
