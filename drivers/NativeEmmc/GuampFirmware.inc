// SPDX-License-Identifier: BSD-2-Clause-Patent
// Qualcomm split firmware check for the guamp Wi-Fi preparation (GuampWifiPrep.inc), host-tested by
// tests/wifi_host.cpp. Read-only: it parses name.mdt, reads every name.bNN through GuampExt4.inc and compares its
// SHA-384 with the hash table of the hash segment. Nothing is loaded anywhere and nothing is authenticated (the
// signature over the hash table is TrustZone's business and out of scope).
//   name.mdt   ELF header + program headers (= segment 0, p_filesz bytes) followed by the hash segment
//              (layout as Linux mdt_loader.c: packed right after the headers, at its own p_offset, or only in .bNN)
//   hash seg   the program header with p_flags bits 26:24 == 2; MBN header v5 (40 bytes) or v6 (48 bytes + QTI
//              metadata + OEM metadata, sizes in words 10 and 11), then code_size bytes = one SHA-384 per program
//              header (all zero for the hash segment itself and for empty segments)
//   segments   name.bNN (NN = program header index, two digits); segment 0 falls back to the .mdt headers
// guamp modem.mdt: ELF32, 31 program headers, hash segment 1, MBN v6, hash table at 168.

namespace fw {

// --- SHA-384 (FIPS 180-4: SHA-512 compression, own initial values, 48-byte digest) ---------------------------------
struct Sha384 {
  uint64_t h[8];
  uint8_t buf[128];
  uint64_t total;
  uint32_t used;

  static uint64_t rotr(uint64_t x, unsigned n) { return (x >> n) | (x << (64 - n)); }
  static uint64_t be64(const uint8_t *p) {
    uint64_t v = 0;
    for (unsigned i = 0; i < 8; i++) v = (v << 8) | p[i];
    return v;
  }
  void block(const uint8_t *p) {
    static const uint64_t k[80] = {
      0x428a2f98d728ae22ull, 0x7137449123ef65cdull, 0xb5c0fbcfec4d3b2full, 0xe9b5dba58189dbbcull, 0x3956c25bf348b538ull,
      0x59f111f1b605d019ull, 0x923f82a4af194f9bull, 0xab1c5ed5da6d8118ull, 0xd807aa98a3030242ull, 0x12835b0145706fbeull,
      0x243185be4ee4b28cull, 0x550c7dc3d5ffb4e2ull, 0x72be5d74f27b896full, 0x80deb1fe3b1696b1ull, 0x9bdc06a725c71235ull,
      0xc19bf174cf692694ull, 0xe49b69c19ef14ad2ull, 0xefbe4786384f25e3ull, 0x0fc19dc68b8cd5b5ull, 0x240ca1cc77ac9c65ull,
      0x2de92c6f592b0275ull, 0x4a7484aa6ea6e483ull, 0x5cb0a9dcbd41fbd4ull, 0x76f988da831153b5ull, 0x983e5152ee66dfabull,
      0xa831c66d2db43210ull, 0xb00327c898fb213full, 0xbf597fc7beef0ee4ull, 0xc6e00bf33da88fc2ull, 0xd5a79147930aa725ull,
      0x06ca6351e003826full, 0x142929670a0e6e70ull, 0x27b70a8546d22ffcull, 0x2e1b21385c26c926ull, 0x4d2c6dfc5ac42aedull,
      0x53380d139d95b3dfull, 0x650a73548baf63deull, 0x766a0abb3c77b2a8ull, 0x81c2c92e47edaee6ull, 0x92722c851482353bull,
      0xa2bfe8a14cf10364ull, 0xa81a664bbc423001ull, 0xc24b8b70d0f89791ull, 0xc76c51a30654be30ull, 0xd192e819d6ef5218ull,
      0xd69906245565a910ull, 0xf40e35855771202aull, 0x106aa07032bbd1b8ull, 0x19a4c116b8d2d0c8ull, 0x1e376c085141ab53ull,
      0x2748774cdf8eeb99ull, 0x34b0bcb5e19b48a8ull, 0x391c0cb3c5c95a63ull, 0x4ed8aa4ae3418acbull, 0x5b9cca4f7763e373ull,
      0x682e6ff3d6b2b8a3ull, 0x748f82ee5defb2fcull, 0x78a5636f43172f60ull, 0x84c87814a1f0ab72ull, 0x8cc702081a6439ecull,
      0x90befffa23631e28ull, 0xa4506cebde82bde9ull, 0xbef9a3f7b2c67915ull, 0xc67178f2e372532bull, 0xca273eceea26619cull,
      0xd186b8c721c0c207ull, 0xeada7dd6cde0eb1eull, 0xf57d4f7fee6ed178ull, 0x06f067aa72176fbaull, 0x0a637dc5a2c898a6ull,
      0x113f9804bef90daeull, 0x1b710b35131c471bull, 0x28db77f523047d84ull, 0x32caab7b40c72493ull, 0x3c9ebe0a15c9bebcull,
      0x431d67c49c100d4cull, 0x4cc5d4becb3e42b6ull, 0x597f299cfc657e2aull, 0x5fcb6fab3ad6faecull, 0x6c44198c4a475817ull};
    uint64_t w[80];
    for (unsigned i = 0; i < 16; i++) w[i] = be64(p + 8 * i);
    for (unsigned i = 16; i < 80; i++) {
      uint64_t s0 = rotr(w[i - 15], 1) ^ rotr(w[i - 15], 8) ^ (w[i - 15] >> 7);
      uint64_t s1 = rotr(w[i - 2], 19) ^ rotr(w[i - 2], 61) ^ (w[i - 2] >> 6);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint64_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (unsigned i = 0; i < 80; i++) {
      uint64_t t1 = hh + (rotr(e, 14) ^ rotr(e, 18) ^ rotr(e, 41)) + ((e & f) ^ (~e & g)) + k[i] + w[i];
      uint64_t t2 = (rotr(a, 28) ^ rotr(a, 34) ^ rotr(a, 39)) + ((a & b) ^ (a & c) ^ (b & c));
      hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] = h[0] + a; h[1] = h[1] + b; h[2] = h[2] + c; h[3] = h[3] + d;
    h[4] = h[4] + e; h[5] = h[5] + f; h[6] = h[6] + g; h[7] = h[7] + hh;
  }
  void init() {
    static const uint64_t iv[8] = {0xcbbb9d5dc1059ed8ull, 0x629a292a367cd507ull, 0x9159015a3070dd17ull,
                                   0x152fecd8f70e5939ull, 0x67332667ffc00b31ull, 0x8eb44a8768581511ull,
                                   0xdb0c2e0d64f98fa7ull, 0x47b5481dbefa4fa4ull};
    for (unsigned i = 0; i < 8; i++) h[i] = iv[i];
    total = 0;
    used = 0;
  }
  void update(const uint8_t *p, uint64_t n) {
    total += n;
    while (n) {
      if (!used && n >= 128) { block(p); p += 128; n -= 128; continue; }
      uint32_t take = 128 - used;
      if (take > n) take = static_cast<uint32_t>(n);
      memcpy(buf + used, p, take);
      used += take; p += take; n -= take;
      if (used == 128) { block(buf); used = 0; }
    }
  }
  void final(uint8_t out[48]) {
    uint64_t bits = total << 3, high = total >> 61;
    buf[used++] = 0x80;
    if (used > 112) { memset(buf + used, 0, 128 - used); block(buf); used = 0; }
    memset(buf + used, 0, 112 - used);
    for (unsigned i = 0; i < 8; i++) {
      buf[112 + i] = static_cast<uint8_t>(high >> (56 - 8 * i));
      buf[120 + i] = static_cast<uint8_t>(bits >> (56 - 8 * i));
    }
    block(buf);
    for (unsigned i = 0; i < 48; i++) out[i] = static_cast<uint8_t>(h[i / 8] >> (56 - 8 * (i % 8)));
  }
};

// --- MBN / split ELF ---------------------------------------------------------------------------------------------
constexpr uint32_t kMaxSegments = 64, kHashBytes = 48;
constexpr uint32_t kMdtMax = 64 * 1024;          // .mdt plus a separate hash segment must fit the caller's buffer

enum : int { kOk = 0, kErrFormat = -20, kErrUnsupported = -21, kErrTooBig = -22, kErrNoHash = -23 };

struct Segment {
  uint32_t type, flags;
  uint64_t offset, paddr, filesz, memsz;
};

struct Image {
  uint32_t count = 0, hashIndex = 0, mbnVersion = 0, tableOffset = 0, headerBytes = 0;
  bool elf64 = false;
  Segment seg[kMaxSegments];
  const uint8_t *hashSeg = nullptr;   // inside the caller's .mdt buffer
  uint32_t hashSegBytes = 0;
  int hashWhere = 0;                  // 1 packed after the headers, 2 at p_offset, 3 in its own .bNN
};

inline uint64_t le64(const uint8_t *p) { return ext4::le32(p) | static_cast<uint64_t>(ext4::le32(p + 4)) << 32; }

// Program headers and the location of the hash segment. hashWhere 3: the caller loads name.bNN and calls setHash.
inline int parse(const uint8_t *mdt, uint32_t size, Image *img) {
  if (size < 52 || memcmp(mdt, "\x7f" "ELF", 4) || mdt[5] != 1) return kErrFormat;
  img->elf64 = mdt[4] == 2;
  if (mdt[4] != 1 && mdt[4] != 2) return kErrFormat;
  uint64_t phoff;
  uint32_t phentsize, phnum;
  if (img->elf64) {
    if (size < 64) return kErrFormat;
    phoff = le64(mdt + 32); phentsize = ext4::le16(mdt + 54); phnum = ext4::le16(mdt + 56);
    if (phentsize != 56) return kErrFormat;
  } else {
    phoff = ext4::le32(mdt + 28); phentsize = ext4::le16(mdt + 42); phnum = ext4::le16(mdt + 44);
    if (phentsize != 32) return kErrFormat;
  }
  if (!phnum || phnum > kMaxSegments) return kErrTooBig;
  if (phoff > size || static_cast<uint64_t>(phnum) * phentsize > size - phoff) return kErrFormat;
  img->count = phnum;
  img->hashIndex = phnum;
  for (uint32_t i = 0; i < phnum; i++) {
    const uint8_t *p = mdt + phoff + static_cast<uint64_t>(i) * phentsize;
    Segment &s = img->seg[i];
    s.type = ext4::le32(p);
    if (img->elf64) {
      s.flags = ext4::le32(p + 4); s.offset = le64(p + 8); s.paddr = le64(p + 24); s.filesz = le64(p + 32); s.memsz = le64(p + 40);
    } else {
      s.offset = ext4::le32(p + 4); s.paddr = ext4::le32(p + 12); s.filesz = ext4::le32(p + 16); s.memsz = ext4::le32(p + 20);
      s.flags = ext4::le32(p + 24);
    }
    if (((s.flags >> 24) & 7) == 2 && img->hashIndex == phnum) img->hashIndex = i;
  }
  if (img->hashIndex == phnum) return kErrNoHash;
  uint64_t ehdr = img->seg[0].filesz, hashSize = img->seg[img->hashIndex].filesz;
  img->headerBytes = static_cast<uint32_t>(ehdr <= size ? ehdr : 0);
  if (!img->headerBytes || ehdr < phoff + static_cast<uint64_t>(phnum) * phentsize) return kErrFormat;
  if (!hashSize || hashSize > kMdtMax) return kErrFormat;
  img->hashSegBytes = static_cast<uint32_t>(hashSize);
  uint64_t at = img->seg[img->hashIndex].offset;
  if (ehdr + hashSize == size) { img->hashSeg = mdt + ehdr; img->hashWhere = 1; }
  else if (at <= size && hashSize <= size - at) { img->hashSeg = mdt + at; img->hashWhere = 2; }
  else { img->hashSeg = nullptr; img->hashWhere = 3; }
  return kOk;
}

// Hash table inside the hash segment
inline int setHash(Image *img, const uint8_t *hashSeg, uint32_t bytes) {
  img->hashSeg = hashSeg;
  if (bytes < img->hashSegBytes || img->hashSegBytes < 48) return kErrFormat;
  const uint8_t *h = hashSeg;
  uint32_t version = ext4::le32(h + 4), codeSize = ext4::le32(h + 20);
  uint64_t table;
  if (version == 5) table = 40;
  else if (version == 6) table = 48ull + ext4::le32(h + 40) + ext4::le32(h + 44);
  else { img->mbnVersion = version; return kErrUnsupported; }   // v3 uses SHA-256, v7+ another header
  img->mbnVersion = version;
  if (codeSize < static_cast<uint64_t>(img->count) * kHashBytes || table > img->hashSegBytes ||
      codeSize > img->hashSegBytes - table) return kErrFormat;
  img->tableOffset = static_cast<uint32_t>(table);
  return kOk;
}

struct Result {                  // plain data
  int error;                     // first fatal error (mdt, hash segment); per-segment problems only count as failed
  uint32_t segments, verified, failed, empty, mbnVersion;
  uint64_t bytes;                // bytes hashed (verified and failed)
  uint64_t failedMask;           // bit per program header
};

// path/name + suffix into out (bounded)
inline bool joinName(char *out, uint32_t cap, const char *dir, const char *base, const char *suffix, int index) {
  uint32_t n = 0;
  auto put = [&](char c) { if (n + 1 < cap) out[n++] = c; };
  for (const char *p = dir; *p; p++) put(*p);
  if (n && out[n - 1] != '/') put('/');
  for (const char *p = base; *p; p++) put(*p);
  for (const char *p = suffix; *p; p++) put(*p);
  if (index >= 0) { put(static_cast<char>('0' + index / 10 % 10)); put(static_cast<char>('0' + index % 10)); }
  out[n] = 0;
  return n + 1 < cap;
}

// dir "/image", base "modem". mdtBuf: kMdtMax bytes; chunk: chunkBytes (a multiple of the fs block size)
inline void verify(ext4::Fs &fs, const char *dir, const char *base, uint8_t *mdtBuf, uint8_t *chunk, uint32_t chunkBytes,
                   Result *res) {
  char path[128];
  ext4::Inode ino;
  Image img;
  *res = Result();
  joinName(path, sizeof(path), dir, base, ".mdt", -1);
  int r = fs.lookupPath(path, &ino);
  if (!r && (!ino.isFile() || ino.size > kMdtMax / 2)) r = kErrTooBig;
  if (!r) r = fs.readFile(ino, 0, mdtBuf, static_cast<uint32_t>(ino.size));
  if (r) { res->error = r; return; }
  uint32_t mdtSize = static_cast<uint32_t>(ino.size);
  r = parse(mdtBuf, mdtSize, &img);
  const uint32_t hashAt = (mdtSize + 63) & ~63u;      // aligned: the eMMC PIO path stores 32-bit words
  if (!r && img.hashWhere == 3) {
    joinName(path, sizeof(path), dir, base, ".b", static_cast<int>(img.hashIndex));
    r = fs.lookupPath(path, &ino);
    if (!r && (ino.size != img.hashSegBytes || ino.size > kMdtMax - hashAt)) r = kErrFormat;
    if (!r) r = fs.readFile(ino, 0, mdtBuf + hashAt, img.hashSegBytes);
    if (!r) img.hashSeg = mdtBuf + hashAt;
  }
  if (!r) r = setHash(&img, img.hashSeg, img.hashSegBytes);
  res->mbnVersion = img.mbnVersion;
  res->segments = img.count;
  if (r) { res->error = r; return; }
  static const uint8_t zero[kHashBytes] = {};
  for (uint32_t i = 0; i < img.count; i++) {
    if (i == img.hashIndex) continue;
    const uint8_t *expected = img.hashSeg + img.tableOffset + i * kHashBytes;
    const Segment &s = img.seg[i];
    if (!s.filesz) { res->empty++; continue; }
    bool ok = memcmp(expected, zero, kHashBytes) != 0;   // a loaded segment without a hash fails
    Sha384 sha;
    sha.init();
    joinName(path, sizeof(path), dir, base, ".b", static_cast<int>(i));
    int lr = fs.lookupPath(path, &ino);
    if (lr == ext4::kErrNotFound && i == 0) {
      sha.update(mdtBuf, img.headerBytes);           // headers only inside the .mdt
      res->bytes += img.headerBytes;
    } else if (lr || !ino.isFile() || ino.size != s.filesz) {
      ok = false;
    } else {
      for (uint64_t off = 0; ok && off < ino.size;) {
        uint32_t n = ino.size - off < chunkBytes ? static_cast<uint32_t>(ino.size - off) : chunkBytes;
        if (fs.readFile(ino, off, chunk, n)) { ok = false; break; }
        sha.update(chunk, n);
        off += n;
        res->bytes += n;
      }
    }
    uint8_t digest[kHashBytes];
    sha.final(digest);
    if (ok && !memcmp(digest, expected, kHashBytes)) res->verified++;
    else { res->failed++; res->failedMask |= 1ull << i; }
  }
}

}  // namespace fw
