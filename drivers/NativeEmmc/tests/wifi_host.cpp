// Host tests for GuampExt4.inc, GuampFirmware.inc and GuampSmem.inc (the kernel files, included unchanged).
//   wifi_host sha                         SHA-384 vectors
//   wifi_host shastream                   SHA-384 of stdin (compared with hashlib by run-tests.sh)
//   wifi_host modem <modem.img> <dir>     ext4 + firmware on the backup modem image; <dir> = files extracted earlier
//   wifi_host tree <fs.img> <srcdir>      every file of a mke2fs -d image against its source tree
//   wifi_host smem                        SMEM parser on synthetic regions
// Prints no identifiers (no UUIDs, labels or serials); file names and sizes only.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <vector>

#include "../GuampExt4.inc"
#include "../GuampFirmware.inc"
#include "../GuampSmem.inc"

static int failures = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static std::string hex(const uint8_t *p, size_t n) {
  std::string s;
  char b[3];
  for (size_t i = 0; i < n; i++) { snprintf(b, sizeof(b), "%02x", p[i]); s += b; }
  return s;
}
static std::string sha384(const void *p, size_t n) {
  fw::Sha384 s;
  s.init();
  s.update(static_cast<const uint8_t *>(p), n);
  uint8_t d[48];
  s.final(d);
  return hex(d, 48);
}

// --- SHA-384 ------------------------------------------------------------------------------------------------------
static void testSha() {
  struct { const char *msg; const char *digest; } v[] = {
    {"", "38b060a751ac96384cd9327eb1b1e36a21fdb71114be07434c0cc7bf63f6e1da274edebfe76f65fbd51ad2f14898b95b"},
    {"abc", "cb00753f45a35e8bb5a03d699ac65007272c32ab0eded1631a8b605a43ff5bed8086072ba1e7cc2358baeca134c825a7"},
    {"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
     "3391fdddfc8dc7393707a65b1b4709397cf8b1d162af05abfe8f450de5f36bc6b0455a8520bc4e6f5fe95b1fe3c8452b"},
    {"abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu",
     "09330c33f71147e83d192fc782cd1b4753111b173b3b05d22fa08086e3b0f712fcc7c71a557e2db966c3e9fa91746039"},
  };
  for (auto &t : v) CHECK(sha384(t.msg, strlen(t.msg)) == t.digest, "sha384(\"%s\")", t.msg);
  std::vector<uint8_t> million(1000000, 'a');
  CHECK(sha384(million.data(), million.size()) ==
            "9d0e1809716474cb086e834e310a4a1ced149e9c00f248527972cec5704c2a5b07b8b3dc38ecc4ebae97ddd87f3d8985",
        "sha384(million a)");
  // split updates give the same digest
  std::vector<uint8_t> data(1000);
  for (size_t i = 0; i < data.size(); i++) data[i] = static_cast<uint8_t>(i * 7 + 3);
  std::string whole = sha384(data.data(), data.size());
  for (size_t step : {1u, 3u, 63u, 64u, 111u, 112u, 127u, 128u, 129u, 500u}) {
    fw::Sha384 s;
    s.init();
    for (size_t o = 0; o < data.size(); o += step) s.update(data.data() + o, o + step > data.size() ? data.size() - o : step);
    uint8_t d[48];
    s.final(d);
    CHECK(hex(d, 48) == whole, "split %zu", step);
  }
}

// --- block device: image file + byte overlay; checks the 128-sector limit ----------------------------------------
struct Disk {
  int fd = -1;
  uint64_t base = 0, sectors = 0;
  std::vector<std::pair<uint64_t, std::vector<uint8_t>>> patches;
  uint64_t calls = 0, tooBig = 0;
  void patch(uint64_t at, const void *p, size_t n) {
    patches.push_back({at, std::vector<uint8_t>(static_cast<const uint8_t *>(p), static_cast<const uint8_t *>(p) + n)});
  }
  bool readBytes(uint64_t at, uint8_t *out, size_t n) {
    if (pread(fd, out, n, static_cast<off_t>(base + at)) != static_cast<ssize_t>(n)) return false;
    for (auto &pt : patches) {
      uint64_t a = pt.first, e = a + pt.second.size();
      for (uint64_t x = a > at ? a : at; x < e && x < at + n; x++) out[x - at] = pt.second[x - a];
    }
    return true;
  }
  static bool read(void *ctx, uint64_t sector, uint32_t count, uint8_t *out) {
    Disk *d = static_cast<Disk *>(ctx);
    d->calls++;
    if (count > 128 || !count) { d->tooBig++; return false; }
    if (sector + count > d->sectors) return false;
    return d->readBytes(sector * 512, out, count * 512ull);
  }
};

static bool openDisk(Disk &d, const char *path) {
  d.fd = open(path, O_RDONLY);
  if (d.fd < 0) return false;
  struct stat st;
  fstat(d.fd, &st);
  d.sectors = static_cast<uint64_t>(st.st_size) / 512;
  return true;
}

static std::vector<uint8_t> readHostFile(const std::string &path) {
  std::vector<uint8_t> v;
  FILE *f = fopen(path.c_str(), "rb");
  if (!f) return v;
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  v.resize(static_cast<size_t>(n));
  if (n && fread(v.data(), 1, v.size(), f) != v.size()) v.clear();
  fclose(f);
  return v;
}

static std::vector<uint8_t> readFs(ext4::Fs &fs, const ext4::Inode &ino, int *err) {
  std::vector<uint8_t> v(ino.size);
  // odd offsets and lengths on purpose: exercises the partial-block path too
  uint64_t off = 0;
  uint32_t step = 3 * 65536 + 1234;
  *err = 0;
  while (off < ino.size) {
    uint32_t n = ino.size - off < step ? static_cast<uint32_t>(ino.size - off) : step;
    int r = fs.readFile(ino, off, v.data() + off, n);
    if (r) { *err = r; break; }
    off += n;
  }
  return v;
}

struct Entry { std::string name; uint32_t inode; uint8_t type; };
static std::vector<Entry> listDir(ext4::Fs &fs, const ext4::Inode &dir, int *err) {
  std::vector<Entry> out;
  *err = fs.forEachEntry(dir, [&](const char *n, uint32_t len, uint32_t ino, uint8_t t) {
    out.push_back({std::string(n, len), ino, t});
    return false;
  });
  return out;
}

static uint8_t gScratch[ext4::kScratchBytes];

static int mountWith(Disk &d, ext4::Fs &fs) {
  fs = ext4::Fs{};
  return fs.mount(Disk::read, &d, d.sectors, gScratch);
}

// Byte offset of an inode's 128-byte record
static uint64_t inodeOffset(ext4::Fs &fs, Disk &d, uint32_t number) {
  uint32_t group = (number - 1) / fs.inodesPerGroup, index = (number - 1) % fs.inodesPerGroup;
  uint64_t descAt = (fs.firstDataBlock + 1) * static_cast<uint64_t>(fs.blockSize) + static_cast<uint64_t>(group) * fs.descSize;
  uint8_t desc[64];
  d.readBytes(descAt, desc, sizeof(desc));
  uint64_t table = ext4::le32(desc + 8);
  if (fs.descSize >= 64) table |= static_cast<uint64_t>(ext4::le32(desc + 0x28)) << 32;
  return table * fs.blockSize + static_cast<uint64_t>(index) * fs.inodeSize;
}

static void put16(uint8_t *p, uint32_t v) { p[0] = static_cast<uint8_t>(v); p[1] = static_cast<uint8_t>(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = static_cast<uint8_t>(v >> (8 * i)); }

// Rebuilds a file's extent tree as depth 2 (root index -> 2 index blocks -> leaves of 8-block pieces) in free blocks
// at the end of the image (overlay only), optionally with one piece marked uninitialized.
static bool deepenTree(ext4::Fs &fs, Disk &d, const ext4::Inode &ino, bool uninitOne, uint32_t *uninitL) {
  uint8_t root[60];
  memcpy(root, ino.block, 60);
  if (ext4::le16(root) != 0xF30A || ext4::le16(root + 6) != 0) return false;
  struct Piece { uint32_t l; uint16_t len; uint64_t p; };
  std::vector<Piece> pieces;
  for (uint32_t i = 0; i < ext4::le16(root + 2); i++) {
    const uint8_t *e = root + 12 + 12 * i;
    uint32_t l = ext4::le32(e), len = ext4::le16(e + 4);
    uint64_t p = static_cast<uint64_t>(ext4::le16(e + 6)) << 32 | ext4::le32(e + 8);
    for (uint32_t o = 0; o < len; o += 8) pieces.push_back({l + o, static_cast<uint16_t>(len - o < 8 ? len - o : 8), p + o});
  }
  uint32_t perLeaf = (fs.blockSize - 12) / 12;
  uint32_t leaves = static_cast<uint32_t>((pieces.size() + perLeaf - 1) / perLeaf);
  if (leaves < 2) return false;
  uint64_t next = fs.blocksCount - 1;          // free blocks at the end of the group (overlay only)
  std::vector<uint64_t> leafBlock(leaves);
  std::vector<uint8_t> blk(fs.blockSize);
  for (uint32_t k = 0; k < leaves; k++) {
    memset(blk.data(), 0, blk.size());
    uint32_t first = k * perLeaf, n = static_cast<uint32_t>(pieces.size() - first < perLeaf ? pieces.size() - first : perLeaf);
    put16(blk.data(), 0xF30A); put16(blk.data() + 2, n); put16(blk.data() + 4, perLeaf); put16(blk.data() + 6, 0);
    for (uint32_t i = 0; i < n; i++) {
      const Piece &pc = pieces[first + i];
      uint8_t *e = blk.data() + 12 + 12 * i;
      uint16_t len = pc.len;
      if (uninitOne && k == 1 && i == 1) { len = static_cast<uint16_t>(len + 32768); *uninitL = pc.l; }
      put32(e, pc.l); put16(e + 4, len); put16(e + 6, static_cast<uint32_t>(pc.p >> 32)); put32(e + 8, static_cast<uint32_t>(pc.p));
    }
    leafBlock[k] = next--;
    d.patch(leafBlock[k] * fs.blockSize, blk.data(), blk.size());
  }
  // two index blocks of depth 1, splitting the leaves
  uint32_t half = (leaves + 1) / 2;
  uint64_t idxBlock[2];
  for (uint32_t h = 0; h < 2; h++) {
    memset(blk.data(), 0, blk.size());
    uint32_t a = h ? half : 0, b = h ? leaves : half;
    put16(blk.data(), 0xF30A); put16(blk.data() + 2, b - a); put16(blk.data() + 4, perLeaf); put16(blk.data() + 6, 1);
    for (uint32_t k = a; k < b; k++) {
      uint8_t *e = blk.data() + 12 + 12 * (k - a);
      put32(e, pieces[k * perLeaf].l); put32(e + 4, static_cast<uint32_t>(leafBlock[k])); put16(e + 8, static_cast<uint32_t>(leafBlock[k] >> 32));
    }
    idxBlock[h] = next--;
    d.patch(idxBlock[h] * fs.blockSize, blk.data(), blk.size());
  }
  uint8_t nr[60] = {};
  put16(nr, 0xF30A); put16(nr + 2, 2); put16(nr + 4, 4); put16(nr + 6, 2);
  for (uint32_t h = 0; h < 2; h++) {
    uint8_t *e = nr + 12 + 12 * h;
    put32(e, pieces[(h ? half : 0) * perLeaf].l); put32(e + 4, static_cast<uint32_t>(idxBlock[h])); put16(e + 8, static_cast<uint32_t>(idxBlock[h] >> 32));
  }
  d.patch(inodeOffset(fs, d, ino.number) + 40, nr, 60);
  return true;
}

static void testModem(const char *image, const char *extracted) {
  Disk d;
  if (!openDisk(d, image)) { CHECK(false, "open image"); return; }
  ext4::Fs fs;
  int r = mountWith(d, fs);
  CHECK(r == 0, "mount: %s", ext4::errorName(r));
  if (r) return;
  printf("ext4: %llu blocks of %u, %u groups, inode %u bytes, desc %u, features %x/%x/%x\n",
         static_cast<unsigned long long>(fs.blocksCount), fs.blockSize, fs.groups, fs.inodeSize, fs.descSize, fs.compat,
         fs.incompat, fs.roCompat);
  ext4::Inode mdt, dir;
  r = fs.lookupPath("/image/modem.mdt", &mdt);
  CHECK(r == 0 && mdt.isFile() && mdt.size == 9100, "/image/modem.mdt: %s size %llu", ext4::errorName(r),
        static_cast<unsigned long long>(mdt.size));
  CHECK(fs.lookupPath("//image///modem.mdt", &mdt) == 0, "repeated slashes");
  CHECK(fs.lookupPath("/image/nope.mdt", &mdt) == ext4::kErrNotFound, "missing file");
  CHECK(fs.lookupPath("/image/modem.mdt/x", &mdt) == ext4::kErrNotDir, "file as directory");
  CHECK(fs.lookupPath("/image/../image/./modem.mdt", &mdt) == 0, "dot entries");
  r = fs.lookupPath("/image", &dir);
  CHECK(r == 0 && dir.isDir(), "/image");
  int err = 0;
  std::vector<Entry> entries = listDir(fs, dir, &err);
  CHECK(err == 0, "list /image");
  unsigned files = 0, same = 0, depthMax = 0;
  uint64_t bytes = 0;
  for (auto &e : entries) {
    if (e.name == "." || e.name == "..") continue;
    ext4::Inode f;
    if (fs.readInode(e.inode, &f) || !f.isFile()) continue;
    files++;
    depthMax = depthMax > ext4::le16(f.block + 6) ? depthMax : ext4::le16(f.block + 6);
    std::vector<uint8_t> got = readFs(fs, f, &err);
    std::vector<uint8_t> want = readHostFile(std::string(extracted) + "/" + e.name);
    bool ok = !err && got == want;
    CHECK(ok, "file %s differs (err %d, %zu vs %zu bytes)", e.name.c_str(), err, got.size(), want.size());
    if (ok) { same++; bytes += got.size(); }
  }
  printf("ext4: /image %u files identical to the extracted copy (%u checked, %llu bytes, max extent depth %u), %llu reads, none above 128 sectors: %s\n",
         same, files, static_cast<unsigned long long>(bytes), depthMax, static_cast<unsigned long long>(d.calls), d.tooBig ? "NO" : "yes");
  CHECK(d.tooBig == 0, "read above 128 sectors");
  CHECK(files >= 150 && same == files, "file count");

  // Firmware check: every .mdt of /image
  static uint8_t mdtBuf[fw::kMdtMax], chunk[65536];
  fw::Result res;
  fw::verify(fs, "/image", "modem", mdtBuf, chunk, sizeof(chunk), &res);
  printf("GuampWifi: modem firmware %u segments verified/%u failed, %llu bytes (%u program headers, MBN v%u, %u empty, failed mask %llx)\n",
         res.verified, res.failed, static_cast<unsigned long long>(res.bytes), res.segments, res.mbnVersion, res.empty,
         static_cast<unsigned long long>(res.failedMask));
  CHECK(res.error == 0 && res.verified == 26 && res.failed == 0 && res.empty == 4 && res.segments == 31 && res.mbnVersion == 6,
        "modem firmware");
  for (auto &e : entries) {
    if (e.name.size() < 5 || e.name.compare(e.name.size() - 4, 4, ".mdt") || e.name == "modem.mdt") continue;
    std::string base = e.name.substr(0, e.name.size() - 4);
    fw::verify(fs, "/image", base.c_str(), mdtBuf, chunk, sizeof(chunk), &res);
    printf("  %-14s %2u verified/%u failed, %u empty, MBN v%u, error %d\n", e.name.c_str(), res.verified, res.failed,
           res.empty, res.mbnVersion, res.error);
    CHECK(res.error != 0 || res.failed == 0, "%s", e.name.c_str());
  }

  // Corruptions (overlay only): a data byte of modem.b05, a hash table byte of segment 9 inside modem.mdt
  ext4::Inode b05;
  fs.lookupPath("/image/modem.b05", &b05);
  uint64_t pblk = 0;
  uint32_t run = 0;
  fs.mapBlock(b05, 3, &pblk, &run);
  uint8_t byte;
  d.readBytes(pblk * fs.blockSize + 77, &byte, 1);
  byte ^= 0x40;
  d.patch(pblk * fs.blockSize + 77, &byte, 1);
  fs.mapBlock(mdt, 0, &pblk, &run);
  uint64_t tableByte = pblk * fs.blockSize + 1044 + 168 + 9 * 48 + 5;   // mdt = headers (1044) + hash segment
  d.readBytes(tableByte, &byte, 1);
  byte ^= 1;
  d.patch(tableByte, &byte, 1);
  mountWith(d, fs);
  fw::verify(fs, "/image", "modem", mdtBuf, chunk, sizeof(chunk), &res);
  CHECK(res.error == 0 && res.verified == 24 && res.failed == 2 && res.failedMask == ((1ull << 5) | (1ull << 9)),
        "corrupted: %u verified %u failed mask %llx", res.verified, res.failed, static_cast<unsigned long long>(res.failedMask));
  printf("corruption: %u verified/%u failed, mask %llx (expected segments 5 and 9)\n", res.verified, res.failed,
         static_cast<unsigned long long>(res.failedMask));
  d.patches.clear();

  // modem.mdt truncated to its headers: the hash segment must come from modem.b01
  mountWith(d, fs);
  uint8_t size[4];
  put32(size, 1044);
  d.patch(inodeOffset(fs, d, mdt.number) + 4, size, 4);
  mountWith(d, fs);
  fw::verify(fs, "/image", "modem", mdtBuf, chunk, sizeof(chunk), &res);
  CHECK(res.error == 0 && res.verified == 26 && res.failed == 0, "hash from .b01: error %d verified %u", res.error, res.verified);
  printf("hash segment from modem.b01: %u verified/%u failed\n", res.verified, res.failed);
  d.patches.clear();

  // Depth-2 extent tree for the biggest file, with and without an uninitialized piece
  mountWith(d, fs);
  ext4::Inode big;
  fs.lookupPath("/image/modem.b24", &big);
  std::vector<uint8_t> want = readFs(fs, big, &err);
  for (int uninit = 0; uninit < 2; uninit++) {
    d.patches.clear();
    mountWith(d, fs);
    uint32_t zeroL = 0;
    bool built = deepenTree(fs, d, big, uninit, &zeroL);
    CHECK(built, "deepen tree");
    mountWith(d, fs);
    ext4::Inode deep;
    fs.lookupPath("/image/modem.b24", &deep);
    std::vector<uint8_t> got = readFs(fs, deep, &err);
    std::vector<uint8_t> expect = want;
    if (uninit) memset(expect.data() + static_cast<uint64_t>(zeroL) * fs.blockSize, 0, 8ull * fs.blockSize);
    CHECK(!err && ext4::le16(deep.block + 6) == 2 && got == expect, "depth-2 tree read (uninit %d, err %d)", uninit, err);
    printf("depth-2 extent tree%s: %s\n", uninit ? " with an uninitialized extent" : "", !err && got == expect ? "identical" : "DIFFERENT");
  }
  d.patches.clear();

  // Refused file systems
  struct { const char *what; uint32_t off; uint32_t value; int width; int expect; } bad[] = {
    {"bad magic", 56, 0x1234, 2, ext4::kErrFormat},
    {"block size 64 KiB", 24, 6, 4, ext4::kErrUnsupported},
    {"needs_recovery", 96, 0x42 | ext4::kIncompatRecover, 4, ext4::kErrUnsupported},
    {"inline_data", 96, 0x42 | ext4::kIncompatInlineData, 4, ext4::kErrUnsupported},
    {"meta_bg", 96, 0x42 | ext4::kIncompatMetaBg, 4, ext4::kErrUnsupported},
    {"encrypt", 96, 0x42 | 0x10000, 4, ext4::kErrUnsupported},
    {"casefold", 96, 0x42 | 0x20000, 4, ext4::kErrUnsupported},
    {"bigalloc", 100, 0x7b | ext4::kRoCompatBigalloc, 4, ext4::kErrUnsupported},
    {"blocks beyond the device", 4, 0x7fffffff, 4, ext4::kErrFormat},
    {"inode size 100", 88, 100, 2, ext4::kErrFormat},
  };
  for (auto &b : bad) {
    d.patches.clear();
    uint8_t v[4];
    put32(v, b.value);
    d.patch(1024 + b.off, v, b.width);
    r = mountWith(d, fs);
    CHECK(r == b.expect, "%s: got %d", b.what, r);
    printf("refused %-26s -> %s\n", b.what, ext4::errorName(r));
  }
  // block-mapped and inline-data inodes are refused per file
  d.patches.clear();
  mountWith(d, fs);
  uint8_t flags[4];
  put32(flags, 0);
  d.patch(inodeOffset(fs, d, mdt.number) + 32, flags, 4);
  mountWith(d, fs);
  ext4::Inode plain;
  fs.lookupPath("/image/modem.mdt", &plain);
  uint8_t tmp[16];
  CHECK(fs.readFile(plain, 0, tmp, 16) == ext4::kErrUnsupported, "block-mapped file refused");
  d.patches.clear();
  close(d.fd);
}

static void walkTree(ext4::Fs &fs, const std::string &host, const std::string &path, unsigned *files, unsigned *dirs) {
  DIR *dd = opendir(host.c_str());
  if (!dd) return;
  while (dirent *e = readdir(dd)) {
    std::string n = e->d_name;
    if (n == "." || n == "..") continue;
    std::string h = host + "/" + n, p = path + "/" + n;
    struct stat st;
    lstat(h.c_str(), &st);
    ext4::Inode ino;
    int r = fs.lookupPath(p.c_str(), &ino);
    if (S_ISDIR(st.st_mode)) {
      CHECK(r == 0 && ino.isDir(), "dir %s", p.c_str());
      (*dirs)++;
      walkTree(fs, h, p, files, dirs);
    } else if (S_ISREG(st.st_mode)) {
      int err = r;
      std::vector<uint8_t> got;
      if (!r) got = readFs(fs, ino, &err);
      CHECK(!err && got == readHostFile(h), "file %s (%d)", p.c_str(), err);
      (*files)++;
    }
  }
  closedir(dd);
}

static void testTree(const char *image, const char *src) {
  Disk d;
  if (!openDisk(d, image)) { CHECK(false, "open %s", image); return; }
  ext4::Fs fs;
  int r = mountWith(d, fs);
  CHECK(r == 0, "mount: %s (refused %x)", ext4::errorName(r), fs.refused);
  if (r) return;
  unsigned files = 0, dirs = 0;
  walkTree(fs, src, "", &files, &dirs);
  printf("mke2fs image: block %u, inode %u, desc %u, features %x/%x/%x: %u files, %u directories compared, %llu reads, none above 128 sectors: %s\n",
         fs.blockSize, fs.inodeSize, fs.descSize, fs.compat, fs.incompat, fs.roCompat, files, dirs,
         static_cast<unsigned long long>(d.calls), d.tooBig ? "NO" : "yes");
  CHECK(files > 0 && d.tooBig == 0, "tree");
  close(d.fd);
}

// --- SMEM ---------------------------------------------------------------------------------------------------------
struct Region {
  std::vector<uint8_t> mem;
  uint32_t forbidLo = 0, forbidHi = 0, violations = 0, misaligned = 0;
  static uint32_t read(void *ctx, uint32_t off) {
    Region *r = static_cast<Region *>(ctx);
    if (off & 3) r->misaligned++;
    if (off >= r->forbidLo && off < r->forbidHi) r->violations++;
    uint32_t v;
    memcpy(&v, r->mem.data() + off, 4);
    return v;
  }
  void w32(uint32_t off, uint32_t v) { put32(mem.data() + off, v); }
  void w16(uint32_t off, uint32_t v) { put16(mem.data() + off, v); }
};

struct PartBuilder {
  Region &r;
  uint32_t base, size, line, up, down;
  PartBuilder(Region &reg, uint32_t b, uint32_t s, uint16_t h0, uint16_t h1, uint32_t cl) : r(reg), base(b), size(s), line(cl), up(32), down(s) {
    memcpy(r.mem.data() + b, "$PRT", 4);
    r.w16(b + 4, h0); r.w16(b + 6, h1); r.w32(b + 8, s);
    sync();
  }
  void sync() { r.w32(base + 12, up); r.w32(base + 16, down); }
  uint32_t uncached(uint16_t item, uint32_t bytes) {         // returns the data offset
    uint32_t padded = (bytes + 7) & ~7u;
    uint32_t e = base + up;
    r.w16(e, 0xa5a5); r.w16(e + 2, item); r.w32(e + 4, padded); r.w16(e + 8, padded - bytes); r.w16(e + 10, 0);
    up += 16 + padded;
    sync();
    return e + 16;
  }
  uint32_t cached(uint16_t item, uint32_t bytes) {
    uint32_t padded = (bytes + line - 1) & ~(line - 1), hdr = (16 + line - 1) & ~(line - 1);
    uint32_t e = down - hdr;                 // header slot
    r.w16(base + e, 0xa5a5); r.w16(base + e + 2, item); r.w32(base + e + 4, padded); r.w16(base + e + 8, padded - bytes);
    down = e - padded;
    sync();
    return base + e - padded;
  }
};

static uint32_t buildSmp2p(Region &r, uint32_t at, uint32_t slaveValue) {
  r.w32(at, smem::kSmp2pMagic);
  r.w32(at + 4, 1 | (1u << 8));
  r.w16(at + 8, 1); r.w16(at + 10, 0); r.w16(at + 12, 16); r.w16(at + 14, 3); r.w32(at + 16, 0);
  const char *names[3] = {"ipa", "slave-kernel", "wlan"};
  uint32_t vals[3] = {0x10, slaveValue, 0x1};
  for (int i = 0; i < 3; i++) {
    memcpy(r.mem.data() + at + 20 + 20 * i, names[i], strlen(names[i]));
    r.w32(at + 20 + 20 * i + 16, vals[i]);
  }
  return 20 + 20 * 16;
}

static void buildSmem(Region &r, bool with428, bool corruptGlobal) {
  r.mem.assign(0x200000, 0);
  r.w32(64 + 4 * 7, 12u << 16);
  r.w32(192, 1);
  uint32_t t = 0x200000 - 4096;
  memcpy(r.mem.data() + t, "$TOC", 4);
  r.w32(t + 4, 1);
  r.w32(t + 8, 3);
  struct { uint32_t off, size; uint16_t h0, h1; uint32_t line; } parts[3] = {
    {0x8000, 0x20000, 0xfffe, 0xfffe, 64}, {0x28000, 0x20000, 0, 1, 64}, {0x48000, 0x20000, 1, 2, 64}};
  for (int i = 0; i < 3; i++) {
    uint32_t e = t + 32 + 48 * i;
    r.w32(e, parts[i].off); r.w32(e + 4, parts[i].size); r.w16(e + 12, parts[i].h0); r.w16(e + 14, parts[i].h1); r.w32(e + 16, parts[i].line);
  }
  uint32_t info = t + 32 + 48 * 3;
  memcpy(r.mem.data() + info, "SIII", 4);
  r.w16(info + 16, 512);   // smem_info.num_items (+12 is reserved)
  PartBuilder g(r, 0x8000, 0x20000, 0xfffe, 0xfffe, 64);
  g.uncached(137, 40);
  uint32_t reason = g.uncached(421, 100);
  memcpy(r.mem.data() + reason, "no crash", 8);
  g.cached(470, 24);
  PartBuilder m(r, 0x28000, 0x20000, 0, 1, 64);
  if (with428) {
    uint32_t at = m.uncached(435, 20 + 20 * 16);
    buildSmp2p(r, at, 0x2);
  }
  m.uncached(478, 32);
  m.cached(479, 16384);
  m.cached(480, 16384);
  PartBuilder x(r, 0x48000, 0x20000, 1, 2, 64);   // modem-adsp: must not be read
  x.uncached(5, 8);
  r.forbidLo = 0x48000;
  r.forbidHi = 0x68000;
  if (corruptGlobal) r.w16(0x8000 + 32, 0x1234);   // canary of the first global item
}

static void testSmem() {
  for (int pass = 0; pass < 3; pass++) {
    Region reg;
    buildSmem(reg, pass != 1, pass == 2);
    smem::Reader r = {Region::read, &reg, 0x200000};
    smem::Info info;
    smem::parse(r, &info);
    CHECK(info.error == 0 && info.version == 12 && info.partitions == 3 && info.numItems == 512, "smem parse pass %d", pass);
    CHECK(info.part[0].read && info.part[1].read && !info.part[2].read, "apps partitions only");
    CHECK(pass == 2 ? info.part[0].status != 0 : (info.part[0].status == 0 && info.part[0].items == 3), "global items");
    CHECK(info.part[1].status == 0 && info.part[1].items == (pass == 1 ? 3u : 4u), "modem items %u", info.part[1].items);
    uint32_t at = 0, size = 0;
    bool f421 = smem::find(r, info, smem::kHostModem, 421, &at, &size);
    CHECK(pass == 2 ? !f421 : (f421 && size == 100 && !memcmp(reg.mem.data() + at, "no crash", 8)), "421");
    bool f479 = smem::find(r, info, smem::kHostModem, 479, &at, &size);
    CHECK(f479 && size == 16384, "479 cached");
    bool f470 = smem::find(r, info, smem::kHostModem, 470, &at, &size);
    CHECK(pass == 2 ? !f470 : (f470 && size == 24), "470 cached global");
    CHECK(!smem::find(r, info, smem::kHostModem, 428, &at, &size), "428 absent");
    CHECK(!smem::find(r, info, smem::kHostModem, 600, &at, &size), "600 out of range");
    bool f428 = smem::find(r, info, smem::kHostModem, 435, &at, &size);
    CHECK(f428 == (pass != 1), "435 presence");
    if (f428) {
      smem::Smp2p s;
      smem::readSmp2p(r, at, size, "slave-kernel", &s);
      CHECK(s.valid && s.hasEntry && s.value == 0x2 && s.localPid == 1 && s.remotePid == 0 && s.validEntries == 3, "smp2p");
      printf("smem pass %d: SMP2P 435 valid %d pids %u->%u entries %u/%u slave-kernel %x (ready %u)\n", pass, s.valid,
             s.localPid, s.remotePid, s.validEntries, s.total, s.value, s.value >> 1 & 1);
    } else {
      printf("smem pass %d: SMP2P 435 absent (modem not running)\n", pass);
    }
    CHECK(reg.violations == 0 && reg.misaligned == 0, "reads outside apps partitions %u, misaligned %u", reg.violations, reg.misaligned);
  }
  // no partition table on a v12 header, v11 heap, unknown version
  Region reg;
  reg.mem.assign(0x200000, 0);
  reg.w32(64 + 28, 12u << 16);
  smem::Reader r = {Region::read, &reg, 0x200000};
  smem::Info info;
  smem::parse(r, &info);
  CHECK(info.error == smem::kErrNoTable, "v12 without table");
  reg.w32(64 + 28, 11u << 16);
  reg.w32(208 + 16 * 421, 1); reg.w32(208 + 16 * 421 + 4, 0x3000); reg.w32(208 + 16 * 421 + 8, 64);
  smem::parse(r, &info);
  uint32_t at = 0, size = 0;
  CHECK(info.error == 0 && smem::find(r, info, 1, 421, &at, &size) && at == 0x3000 && size == 64, "v11 toc");
  reg.w32(64 + 28, 0);
  smem::parse(r, &info);
  CHECK(info.error == smem::kErrVersion, "uninitialized smem refused");
  printf("smem: v12 without table, v11 heap and unknown version handled\n");
}

int main(int argc, char **argv) {
  if (argc >= 2 && !strcmp(argv[1], "shastream")) {
    std::vector<uint8_t> in;
    uint8_t b[65536];
    size_t n;
    while ((n = fread(b, 1, sizeof(b), stdin)) > 0) in.insert(in.end(), b, b + n);
    printf("%s\n", sha384(in.data(), in.size()).c_str());
    return 0;
  }
  if (argc >= 2 && !strcmp(argv[1], "sha")) testSha();
  else if (argc >= 4 && !strcmp(argv[1], "modem")) testModem(argv[2], argv[3]);
  else if (argc >= 4 && !strcmp(argv[1], "tree")) testTree(argv[2], argv[3]);
  else if (argc >= 2 && !strcmp(argv[1], "smem")) testSmem();
  else { fprintf(stderr, "usage: see the file header\n"); return 2; }
  printf("%s: %d checks, %d failed\n", argv[1], checks, failures);
  return failures ? 1 : 0;
}
