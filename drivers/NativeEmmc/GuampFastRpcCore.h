// SPDX-License-Identifier: BSD-2-Clause-Patent
// FastRPC reverse listener for the ADSP sensors PD (analisi/sensori/SENSORI-VERI-20261029.md, step 7). The sensors PD
// reads its registry and json files from the apps side through apps_std (firmware strings: "fopen failed file=%s
// error=%i, retrying", "opendir (%s) failed %i, retrying", sns_registry_osa.c fread/fwrite/readdir): on Android this is
// /vendor/bin/sscrpcd sensorspd = adsp_default_listener. Without it the PD retries forever and no physical sensor
// comes up. Pure logic, no kernel calls (host test tests/fastrpc_host.cpp); the glue is GuampSnsRpc.inc.
// Protocol (public sources: Linux drivers/misc/fastrpc.c, github.com/quic/fastrpc listener_android.c, listener_buf.h,
// adsp_listener_stub.c, remotectl_stub.c, apps_std_skel.c, apps_remotectl_skel.c; downstream adsprpc for the attach):
//   transport  GLINK channel "fastrpcglink-apps-dsp" (DT qcom,msm_fastrpc_rpmsg, intents 0x64 x 0x40)
//   request    fastrpc_msg {pid, tid, ctx u64, handle, sc, addr u64, size u64} (40 bytes); ctx = idx << 4 | pd,
//              pd 2 = sensors PD; response {ctx u64, retval i32 [, flags, early wake, version]}
//   payload    at addr (page aligned, size rounded to pages): remote_buf {pv u64, len u64} x n, invoke_buf {num, pgidx}
//              x n, phy_page {addr u64, size u64} x n, fdlist u64 x 16, crclist u32 x 64, then each buffer 128 aligned
//   sc         method << 24 | in << 16 | out << 8 (no handles here)
//   attach     handle 1 method 0, in: tgid (FASTRPC_INIT_ATTACH_SNS, pid 0)
//   then       remotectl (handle 0) open "adsp_default_listener", its method 0 (register); open "adsp_listener",
//              init2 (method 3), then next2 (method 4) in a loop: in {prevCtx, prevResult, prevLen, bufsLen} +
//              prevbufs, out {ctx, handle, sc, bufsLenReq} + bufs. bufs = the reverse call's in buffers (u32 len,
//              8 aligned data) then its out lengths (u32 each); prevbufs = the answered out buffers (u32 len, 8 aligned
//              data). Handle 0 of a reverse call is apps_remotectl (open/close), the handle it returns is apps_std.
// Files come from a small in-memory file system (Vfs): what the glue preloads (persist ext4 /sensors, read-only, and the
// optional json copy of the ADT), synthesized sysfs answers; writes of the PD go to RAM only and are lost at reboot.
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace guamp_rpc {

// AEE errors (AEEStdErr.h, AEE_EOFFSET 0) and the errno values apps_std_imp.c returns for file errors
enum : int {
  kOk = 0, kEFailed = 1, kENoMemory = 2, kEBadParm = 0x0e, kEUnsupported = 0x14, kENoSuchMod = 0x3e, kEBadSize = 0x4d,
  kErrNoEnt = 2, kErrBadF = 9, kErrExist = 17, kErrNotDir = 20, kErrIsDir = 21, kErrInval = 22, kErrNoSpc = 28,
};
enum : uint32_t {
  kMsgBytes = 40, kPdSensors = 2, kInitHandle = 1, kRemotectl = 0, kMaxArgs = 8, kAlign = 128, kPage = 4096,
  kStdHandle = 0x5d0,   // the apps_std handle this listener hands out (any nonzero value)
};
constexpr uint32_t scalars(uint32_t method, uint32_t in, uint32_t out) {
  return (method & 0x1f) << 24 | (in & 0xff) << 16 | (out & 0xff) << 8;
}
inline uint32_t scMethod(uint32_t sc) { return sc >> 24 & 0x1f; }
inline uint32_t scIn(uint32_t sc) { return sc >> 16 & 0xff; }
inline uint32_t scOut(uint32_t sc) { return sc >> 8 & 0xff; }
inline uint32_t scHandles(uint32_t sc) { return sc & 0xff; }

inline uint32_t ld32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
inline uint64_t ld64(const uint8_t *p) { return (uint64_t)ld32(p) | (uint64_t)ld32(p + 4) << 32; }
inline void st32(uint8_t *p, uint32_t v) { for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(v >> 8 * i); }
inline void st64(uint8_t *p, uint64_t v) { st32(p, (uint32_t)v); st32(p + 4, (uint32_t)(v >> 32)); }
inline uint32_t align(uint32_t v, uint32_t a) { return (v + a - 1) & ~(a - 1); }
inline uint32_t strLen(const char *s, uint32_t max) { uint32_t n = 0; while (n < max && s[n]) ++n; return n; }

// ---- the ADSP-visible region: uncached CPU view (byte accesses only) and the address the ADSP uses ----
struct Region {
  volatile uint8_t *cpu = nullptr;
  uint64_t dsp = 0;   // IOVA or physical address with the context bank in bits 32+ (adsprpc: phys += cb << 32)
  uint32_t bytes = 0;
  void put(uint32_t at, const uint8_t *d, uint32_t n) { for (uint32_t i = 0; i < n; ++i) cpu[at + i] = d[i]; }
  void get(uint32_t at, uint8_t *d, uint32_t n) const { for (uint32_t i = 0; i < n; ++i) d[i] = cpu[at + i]; }
  void zero(uint32_t at, uint32_t n) { for (uint32_t i = 0; i < n; ++i) cpu[at + i] = 0; }
  void w32(uint32_t at, uint32_t v) { for (unsigned i = 0; i < 4; ++i) cpu[at + i] = (uint8_t)(v >> 8 * i); }
  void w64(uint32_t at, uint64_t v) { w32(at, (uint32_t)v); w32(at + 4, (uint32_t)(v >> 32)); }
  uint32_t r32(uint32_t at) const { uint8_t b[4]; get(at, b, 4); return ld32(b); }
};

inline uint32_t metaBytes(uint32_t n) { return 40 * n + 8 * 16 + 4 * 64; }

// One invoke payload at offset 0 of the region: argument i (inputs first) at off[i], len[i] bytes
struct Invoke {
  uint32_t sc = 0, n = 0, total = 0, off[kMaxArgs] = {}, len[kMaxArgs] = {};
};
inline bool buildInvoke(Region &r, uint32_t sc, const uint8_t *const *in, const uint32_t *len, Invoke *iv) {
  const uint32_t nin = scIn(sc), n = nin + scOut(sc);
  if (n > kMaxArgs || scHandles(sc) || !r.cpu || (r.dsp & (kPage - 1))) return false;
  *iv = Invoke{};
  iv->sc = sc;
  iv->n = n;
  if (!n) return true;
  const uint32_t meta = metaBytes(n);
  uint64_t at = align(meta, kAlign);
  for (uint32_t i = 0; i < n; ++i) {
    if (len[i] && (uint64_t)len[i] > r.bytes) return false;
    iv->off[i] = len[i] ? (uint32_t)at : 0;
    iv->len[i] = len[i];
    if (len[i]) at = align((uint32_t)(at + len[i]), kAlign);
    if (at > r.bytes) return false;
  }
  iv->total = (uint32_t)at;
  if (align(iv->total, kPage) > r.bytes) return false;
  r.zero(0, meta);
  for (uint32_t i = 0; i < n; ++i) {
    const uint64_t a = r.dsp + iv->off[i];
    r.w64(16 * i, len[i] ? a : 0);
    r.w64(16 * i + 8, len[i]);
    r.w32(16 * n + 8 * i, len[i] ? 1 : 0);
    r.w32(16 * n + 8 * i + 4, i);
    if (len[i]) {
      r.w64(24 * n + 16 * i, a & ~(uint64_t)(kPage - 1));
      r.w64(24 * n + 16 * i + 8, (((a + len[i] - 1) >> 12) - (a >> 12) + 1) << 12);
      if (i < nin && in && in[i]) r.put(iv->off[i], in[i], len[i]);
      else r.zero(iv->off[i], len[i]);
    }
  }
  return true;
}
inline void encodeMsg(uint8_t *m, uint32_t pid, uint32_t tid, uint64_t ctx, uint32_t handle, uint32_t sc, uint64_t addr,
                      uint64_t size) {
  st32(m, pid); st32(m + 4, tid); st64(m + 8, ctx); st32(m + 16, handle); st32(m + 20, sc); st64(m + 24, addr);
  st64(m + 32, size);
}

// ---- in-memory files ----
struct Vfs {
  static constexpr uint32_t kFiles = 320, kPath = 128, kMaxFile = 1u << 20;
  struct File { char path[kPath]; uint8_t *data; uint32_t size, cap; bool used, dir, owned; };
  struct Hooks {
    void *ctx = nullptr;
    void *(*alloc)(void *, uint32_t) = nullptr;   // zeroed or not, the caller writes it
    void (*free)(void *, void *, uint32_t) = nullptr;
  } hooks;
  File files[kFiles];
  uint32_t count = 0, ramBytes = 0;

  // "/a//b/" -> "/a/b"; false when too long or not absolute
  static bool normalize(const char *in, char *out) {
    if (!in || in[0] != '/') return false;
    uint32_t n = 0;
    for (uint32_t i = 0; in[i]; ++i) {
      if (in[i] == '/' && n && out[n - 1] == '/') continue;
      if (n + 1 >= kPath) return false;
      out[n++] = in[i];
    }
    while (n > 1 && out[n - 1] == '/') --n;
    out[n] = 0;
    return true;
  }
  static bool equal(const char *a, const char *b) {
    uint32_t i = 0;
    while (a[i] && a[i] == b[i]) ++i;
    return a[i] == b[i];
  }
  // b is inside directory a ("/x" contains "/x/y")
  static bool inside(const char *dir, const char *b) {
    uint32_t i = 0;
    while (dir[i] && dir[i] == b[i]) ++i;
    if (dir[i]) return false;
    return (i == 1 && dir[0] == '/') ? b[1] != 0 : b[i] == '/' && b[i + 1];
  }
  File *find(const char *normalized) {
    for (uint32_t i = 0; i < count; ++i)
      if (files[i].used && equal(files[i].path, normalized)) return &files[i];
    return nullptr;
  }
  bool isDir(const char *normalized) {
    if (equal(normalized, "/")) return true;
    for (uint32_t i = 0; i < count; ++i)
      if (files[i].used && ((files[i].dir && equal(files[i].path, normalized)) || inside(normalized, files[i].path))) return true;
    return false;
  }
  File *slot() {
    for (uint32_t i = 0; i < count; ++i) if (!files[i].used) return &files[i];
    return count < kFiles ? &files[count++] : nullptr;
  }
  // A file: copy = own a copy (needs hooks.alloc), else borrow data (copied on the first write)
  File *add(const char *path, const uint8_t *data, uint32_t n, bool copy) {
    char p[kPath];
    if (!normalize(path, p) || n > kMaxFile) return nullptr;
    File *f = find(p);
    if (f && f->dir) return nullptr;
    if (!f) f = slot();
    if (!f) return nullptr;
    release(*f);
    for (uint32_t i = 0; i < kPath; ++i) f->path[i] = p[i];
    f->used = true;
    if (copy && n) {
      uint8_t *d = hooks.alloc ? static_cast<uint8_t *>(hooks.alloc(hooks.ctx, n)) : nullptr;
      if (!d) { f->used = false; return nullptr; }
      for (uint32_t i = 0; i < n; ++i) d[i] = data[i];
      f->data = d; f->cap = n; f->owned = true; ramBytes += n;
    } else {
      f->data = const_cast<uint8_t *>(data);
    }
    f->size = n;
    return f;
  }
  File *mkdir(const char *path) {
    char p[kPath];
    if (!normalize(path, p)) return nullptr;
    if (File *f = find(p)) return f->dir ? f : nullptr;
    File *f = slot();
    if (!f) return nullptr;
    *f = File{};
    for (uint32_t i = 0; i < kPath; ++i) f->path[i] = p[i];
    f->used = f->dir = true;
    return f;
  }
  void release(File &f) {
    if (f.owned && f.data && hooks.free) { hooks.free(hooks.ctx, f.data, f.cap); ramBytes -= f.cap; }
    f.data = nullptr; f.size = f.cap = 0; f.owned = f.dir = false;
  }
  void remove(File &f) { release(f); f.used = false; f.path[0] = 0; }
  // room for `need` bytes, owned (copy on write of borrowed data)
  int reserve(File &f, uint32_t need) {
    if (need > kMaxFile) return kErrNoSpc;
    if (f.owned && need <= f.cap) return kOk;
    uint32_t cap = f.cap ? f.cap : 256;
    while (cap < need) cap *= 2;
    if (cap > kMaxFile) cap = kMaxFile;
    uint8_t *d = hooks.alloc ? static_cast<uint8_t *>(hooks.alloc(hooks.ctx, cap)) : nullptr;
    if (!d) return kErrNoSpc;
    for (uint32_t i = 0; i < f.size; ++i) d[i] = f.data[i];
    for (uint32_t i = f.size; i < cap; ++i) d[i] = 0;
    const uint32_t size = f.size;
    release(f);
    f.data = d; f.cap = cap; f.size = size; f.owned = true; ramBytes += cap;
    return kOk;
  }
  int write(File &f, uint32_t at, const uint8_t *d, uint32_t n) {
    if ((uint64_t)at + n > kMaxFile) return kErrNoSpc;
    const int e = reserve(f, at + n > f.size ? at + n : f.size);
    if (e) return e;
    for (uint32_t i = f.size; i < at; ++i) f.data[i] = 0;
    for (uint32_t i = 0; i < n; ++i) f.data[at + i] = d[i];
    if (at + n > f.size) f.size = at + n;
    return kOk;
  }
  int truncate(File &f, uint32_t n) {
    if (n > f.size) { const int e = reserve(f, n); if (e) return e; for (uint32_t i = f.size; i < n; ++i) f.data[i] = 0; }
    f.size = n;
    return kOk;
  }
};

// One input or output buffer of a reverse call
struct Arg { const uint8_t *in; uint8_t *out; uint32_t len; };

// ---- apps_remotectl (handle 0) and apps_std (kStdHandle) on the Vfs ----
struct Server {
  static constexpr uint32_t kFds = 32, kDirs = 8;
  struct Hooks {
    void *ctx = nullptr;
    void (*log)(void *, const char *what, const char *text, uint64_t a, uint64_t b) = nullptr;
  } hooks;
  Vfs *vfs = nullptr;
  struct Fd { Vfs::File *f; uint32_t pos; uint8_t mode; bool used, eof; };   // mode 1 read, 2 write, 4 append
  struct Dir { char path[Vfs::kPath]; uint32_t at; bool used; };
  Fd fds[kFds] = {};
  Dir dirs[kDirs] = {};
  uint32_t calls = 0, reads = 0, writes = 0, unsupported = 0, opens = 0, misses = 0;

  void note(const char *what, const char *text, uint64_t a, uint64_t b) {
    if (hooks.log) hooks.log(hooks.ctx, what, text ? text : "", a, b);
  }
  static bool cstr(const Arg &a, uint32_t len, const char **s) {   // a NUL terminated string of `len` bytes
    if (!a.in || !len || len > a.len || a.in[len - 1]) return false;
    *s = reinterpret_cast<const char *>(a.in);
    return true;
  }
  static bool contains(const char *s, const char *what) {
    for (uint32_t i = 0; s[i]; ++i) {
      uint32_t k = 0;
      while (what[k] && s[i + k] == what[k]) ++k;
      if (!what[k]) return true;
    }
    return false;
  }
  Fd *fd(uint32_t h) { return h >= 3 && h - 3 < kFds && fds[h - 3].used ? &fds[h - 3] : nullptr; }

  int open(const char *name, const char *mode, uint32_t *out) {
    char p[Vfs::kPath];
    if (!Vfs::normalize(name, p)) { ++misses; note("open-miss", name, 0, 0); return kErrNoEnt; }
    const bool plus = contains(mode, "+");
    const uint8_t m = mode[0] == 'r' ? (uint8_t)(1 | (plus ? 2 : 0)) : mode[0] == 'w' ? (uint8_t)(2 | (plus ? 1 : 0))
                    : mode[0] == 'a' ? (uint8_t)(2 | 4 | (plus ? 1 : 0)) : 0;
    if (!m) return kErrInval;
    Vfs::File *f = vfs->find(p);
    if (f && f->dir) return kErrIsDir;
    if (!f && mode[0] == 'r') { ++misses; note("open-miss", p, 0, 0); return kErrNoEnt; }
    if (!f) {
      char parent[Vfs::kPath];
      uint32_t n = strLen(p, Vfs::kPath), cut = 0;
      for (uint32_t i = 0; i < n; ++i) if (p[i] == '/') cut = i;
      for (uint32_t i = 0; i < cut; ++i) parent[i] = p[i];
      parent[cut ? cut : 1] = 0;
      if (!cut) parent[0] = '/';
      if (!vfs->isDir(parent)) { ++misses; note("open-miss", p, 1, 0); return kErrNoEnt; }
      f = vfs->add(p, nullptr, 0, false);
      if (!f) return kErrNoSpc;
    }
    uint32_t i = 0;
    while (i < kFds && fds[i].used) ++i;
    if (i == kFds) return kErrNoSpc;
    if (mode[0] == 'w') vfs->truncate(*f, 0);
    fds[i] = {f, (m & 4) ? f->size : 0, m, true, false};
    *out = i + 3;
    ++opens;
    note("open", p, *out, m);
    return kOk;
  }
  int stat(const char *name, uint8_t *st96) {
    char p[Vfs::kPath];
    if (!Vfs::normalize(name, p)) return kErrNoEnt;
    for (uint32_t i = 0; i < 96; ++i) st96[i] = 0;
    Vfs::File *f = vfs->find(p);
    if (f && !f->dir) {
      st32(st96 + 24, 0100644); st32(st96 + 28, 1); st64(st96 + 40, f->size);
    } else if (vfs->isDir(p)) {
      st32(st96 + 24, 040755); st32(st96 + 28, 2); st64(st96 + 40, 4096);
    } else {
      ++misses; note("stat-miss", p, 0, 0);
      return kErrNoEnt;
    }
    st64(st96, 96);
    st64(st96 + 16, (uint64_t)(f ? (f - vfs->files) + 11 : 2));
    return kOk;
  }
  // readdir: the next direct child of the directory (files and subdirectories, each name once); "." and ".." skipped
  bool nextEntry(Dir &d, char *name, uint32_t cap) {
    const uint32_t base = strLen(d.path, Vfs::kPath);
    for (; d.at < vfs->count; ++d.at) {
      const Vfs::File &f = vfs->files[d.at];
      if (!f.used || !Vfs::inside(d.path, f.path)) continue;
      const char *rest = f.path + (base == 1 ? 1 : base + 1);
      uint32_t n = 0;
      while (rest[n] && rest[n] != '/') ++n;
      bool seen = false;   // the same child already given by an earlier entry
      for (uint32_t k = 0; k < d.at && !seen; ++k) {
        const Vfs::File &g = vfs->files[k];
        if (!g.used || !Vfs::inside(d.path, g.path)) continue;
        const char *r2 = g.path + (base == 1 ? 1 : base + 1);
        uint32_t m = 0;
        while (r2[m] && r2[m] != '/') ++m;
        if (m != n) continue;
        uint32_t j = 0;
        while (j < n && r2[j] == rest[j]) ++j;
        seen = j == n;
      }
      if (seen) continue;
      const uint32_t l = n < cap - 1 ? n : cap - 1;
      for (uint32_t i = 0; i < l; ++i) name[i] = rest[i];
      name[l] = 0;
      ++d.at;
      return true;
    }
    return false;
  }

  // One reverse call. in[0] / out[0] are the primary in / out buffers of the qaic skeletons.
  int invoke(uint32_t handle, uint32_t sc, const Arg *in, uint32_t nin, Arg *out, uint32_t nout) {
    ++calls;
    const uint32_t mid = scMethod(sc);
    if (nin != scIn(sc) || nout != scOut(sc)) return kEBadParm;
    if (handle == kRemotectl) return remotectl(mid, in, nin, out, nout);
    if (handle == kStdHandle) return std(mid, in, nin, out, nout);
    note("unknown-handle", "", handle, sc);
    return kEBadParm;
  }
  int remotectl(uint32_t mid, const Arg *in, uint32_t nin, Arg *out, uint32_t nout) {
    if (mid == 0) {   // open(name, handle*, dlerror, dlerrorLen, nErr*)
      const char *name;
      if (nin != 2 || nout != 2 || in[0].len < 8 || out[0].len < 8 || !cstr(in[1], ld32(in[0].in), &name)) return kEBadParm;
      if (contains(name, "apps_std")) {
        st32(out[0].out, kStdHandle); st32(out[0].out + 4, 0);
        note("module", name, kStdHandle, 0);
        return kOk;
      }
      static const char kMsg[] = "guamp: module not served";
      for (uint32_t i = 0; i < out[1].len && i < sizeof(kMsg); ++i) out[1].out[i] = (uint8_t)kMsg[i];
      st32(out[0].out, 0); st32(out[0].out + 4, kENoSuchMod);
      note("module-refused", name, 0, 0);
      return kENoSuchMod;
    }
    if (mid == 1) {   // close(handle, errStr, errStrLen, dlErr*)
      if (nout >= 1 && out[0].len >= 4) st32(out[0].out, 0);
      return kOk;
    }
    ++unsupported;
    note("unsupported", "apps_remotectl", mid, 0);
    return kEUnsupported;
  }
  int std(uint32_t mid, const Arg *in, uint32_t nin, Arg *out, uint32_t nout) {
    const uint8_t *pi = nin ? in[0].in : nullptr;
    const uint32_t pl = nin ? in[0].len : 0;
    auto need = [&](uint32_t ins, uint32_t outs, uint32_t prim, uint32_t primOut) {
      return nin == ins && nout == outs && pl >= prim && (!outs || out[0].len >= primOut);
    };
    const char *a = nullptr, *b = nullptr;
    switch (mid) {
      case 0: {   // fopen(name, mode, FILE*)
        if (!need(3, 1, 8, 4) || !cstr(in[1], ld32(pi), &a) || !cstr(in[2], ld32(pi + 4), &b)) return kEBadParm;
        uint32_t h = 0;
        const int e = open(a, b, &h);
        st32(out[0].out, h);
        return e;
      }
      case 2: case 23: case 11: case 14: case 3: {   // fflush, fsync, rewind, clearerr, fclose (FILE)
        if (!need(1, 0, 4, 0)) return kEBadParm;
        Fd *f = fd(ld32(pi));
        if (!f) return kErrBadF;
        if (mid == 11) f->pos = 0;
        if (mid == 11 || mid == 14) f->eof = false;
        if (mid == 3) f->used = false;
        return kOk;
      }
      case 4: case 20: {   // fread(FILE, buf, bufLen, bytesRead*, bEOF*), fgets(FILE, buf, bufLen, bEOF*)
        if (!need(1, 2, 8, mid == 4 ? 8 : 4) || out[1].len < ld32(pi + 4)) return kEBadParm;
        Fd *f = fd(ld32(pi));
        if (!f || !(f->mode & 1)) return kErrBadF;
        const uint32_t want = ld32(pi + 4), size = f->f->size;
        uint32_t n = 0;
        if (mid == 4) {
          while (n < want && f->pos + n < size) { out[1].out[n] = f->f->data[f->pos + n]; ++n; }
          f->pos += n;
          f->eof = f->pos >= size;
          st32(out[0].out, n); st32(out[0].out + 4, f->eof);
        } else {
          while (want && n + 1 < want && f->pos < size) {
            const uint8_t c = f->f->data[f->pos++];
            out[1].out[n++] = c;
            if (c == '\n') break;
          }
          if (want) out[1].out[n] = 0;
          f->eof = f->pos >= size;
          st32(out[0].out, f->eof && !n);
        }
        ++reads;
        return kOk;
      }
      case 5: {   // fwrite(FILE, buf, bufLen, bytesWritten*, bEOF*)
        if (!need(2, 1, 8, 8) || in[1].len < ld32(pi + 4)) return kEBadParm;
        Fd *f = fd(ld32(pi));
        if (!f || !(f->mode & 2)) return kErrBadF;
        const uint32_t n = ld32(pi + 4);
        if (f->mode & 4) f->pos = f->f->size;
        const int e = vfs->write(*f->f, f->pos, in[1].in, n);
        if (e) return e;
        f->pos += n;
        st32(out[0].out, n); st32(out[0].out + 4, 0);
        if (!writes++) note("write", f->f->path, n, 0);
        return kOk;
      }
      case 8: case 12: case 13: {   // ftell, feof, ferror (FILE, int*)
        if (!need(1, 1, 4, 4)) return kEBadParm;
        Fd *f = fd(ld32(pi));
        if (!f) return kErrBadF;
        st32(out[0].out, mid == 8 ? f->pos : mid == 12 ? (uint32_t)f->eof : 0);
        return kOk;
      }
      case 9: {   // fseek(FILE, offset, whence)
        if (!need(1, 0, 12, 0)) return kEBadParm;
        Fd *f = fd(ld32(pi));
        if (!f) return kErrBadF;
        const int64_t base = ld32(pi + 8) == 0 ? 0 : ld32(pi + 8) == 1 ? f->pos : ld32(pi + 8) == 2 ? f->f->size : -1;
        const int64_t to = base + (int32_t)ld32(pi + 4);
        if (base < 0 || to < 0 || to > Vfs::kMaxFile) return kErrInval;
        f->pos = (uint32_t)to;
        f->eof = false;
        return kOk;
      }
      case 10: {   // flen(FILE, uint64*)
        if (!need(1, 1, 4, 8)) return kEBadParm;
        Fd *f = fd(ld32(pi));
        if (!f) return kErrBadF;
        st64(out[0].out, f->f->size);
        return kOk;
      }
      case 15: {   // print_string(str)
        if (!need(2, 0, 4, 0) || !cstr(in[1], ld32(pi), &a)) return kEBadParm;
        note("dsp", a, 0, 0);
        return kOk;
      }
      case 16: {   // getenv(name, val, valLen, valLenReq*): nothing set
        if (!need(2, 2, 8, 4) || !cstr(in[1], ld32(pi), &a)) return kEBadParm;
        note("getenv", a, 0, 0);
        st32(out[0].out, 0);
        return kEFailed;
      }
      case 17: case 18: return kOk;   // setenv, unsetenv
      case 19: {   // fopen_with_env(envvarname, delim, name, mode, FILE*)
        const char *c = nullptr, *d = nullptr;
        if (!need(5, 1, 16, 4) || !cstr(in[1], ld32(pi), &a) || !cstr(in[2], ld32(pi + 4), &b) ||
            !cstr(in[3], ld32(pi + 8), &c) || !cstr(in[4], ld32(pi + 12), &d))
          return kEBadParm;
        uint32_t h = 0;
        int e = kErrNoEnt;
        if (c[0] == '/') e = open(c, d, &h);
        else { ++misses; note("open-env-miss", c, 0, 0); }   // a library from ADSP_LIBRARY_PATH: none is served
        st32(out[0].out, h);
        return e;
      }
      case 22: {   // fileExists(path, bool*)
        if (!need(2, 1, 4, 1) || !cstr(in[1], ld32(pi), &a)) return kEBadParm;
        char p[Vfs::kPath];
        const bool there = Vfs::normalize(a, p) && (vfs->find(p) || vfs->isDir(p));
        out[0].out[0] = there;
        if (!there) { ++misses; note("exists-miss", a, 0, 0); }
        return kOk;
      }
      case 24: case 30: {   // fremove(name), rmdir(name)
        if (!need(2, 0, 4, 0) || !cstr(in[1], ld32(pi), &a)) return kEBadParm;
        char p[Vfs::kPath];
        Vfs::File *f = Vfs::normalize(a, p) ? vfs->find(p) : nullptr;
        if (!f) return kErrNoEnt;
        vfs->remove(*f);
        note("remove", p, 0, 0);
        return kOk;
      }
      case 26: {   // opendir(name, DIR*)
        if (!need(2, 1, 4, 8) || !cstr(in[1], ld32(pi), &a)) return kEBadParm;
        char p[Vfs::kPath];
        if (!Vfs::normalize(a, p) || !vfs->isDir(p)) { ++misses; note("opendir-miss", a, 0, 0); return kErrNoEnt; }
        uint32_t i = 0;
        while (i < kDirs && dirs[i].used) ++i;
        if (i == kDirs) return kErrNoSpc;
        for (uint32_t k = 0; k < Vfs::kPath; ++k) dirs[i].path[k] = p[k];
        dirs[i].at = 0;
        dirs[i].used = true;
        st64(out[0].out, i + 1);
        note("opendir", p, i + 1, 0);
        return kOk;
      }
      case 27: {   // closedir(DIR)
        if (!need(1, 0, 8, 0)) return kEBadParm;
        const uint64_t h = ld64(pi);
        if (!h || h > kDirs || !dirs[h - 1].used) return kErrBadF;
        dirs[h - 1].used = false;
        return kOk;
      }
      case 28: {   // readdir(DIR, DIRENT*, bEOF*): {ino, name[255]} then bEOF at 260
        if (!need(1, 1, 8, 264)) return kEBadParm;
        const uint64_t h = ld64(pi);
        if (!h || h > kDirs || !dirs[h - 1].used) return kErrBadF;
        for (uint32_t i = 0; i < 264; ++i) out[0].out[i] = 0;
        char name[255];
        if (nextEntry(dirs[h - 1], name, sizeof(name))) {
          st32(out[0].out, dirs[h - 1].at + 10);
          for (uint32_t i = 0; name[i]; ++i) out[0].out[4 + i] = (uint8_t)name[i];
        } else {
          st32(out[0].out + 260, 1);
        }
        return kOk;
      }
      case 29: {   // mkdir(name, mode)
        if (!need(2, 0, 8, 0) || !cstr(in[1], ld32(pi), &a)) return kEBadParm;
        char p[Vfs::kPath];
        if (!Vfs::normalize(a, p)) return kErrInval;
        if (vfs->isDir(p)) return kOk;
        if (vfs->find(p)) return kErrExist;
        return vfs->mkdir(p) ? kOk : kErrNoSpc;
      }
      case 31: {   // extended methods: the method number is the first word of the primary input
        if (!nin || pl < 4) return kEBadParm;
        const uint32_t x = ld32(pi);
        if (x == 31) {   // stat(name, STAT*)
          if (!need(2, 1, 8, 96) || !cstr(in[1], ld32(pi + 4), &a)) return kEBadParm;
          return stat(a, out[0].out);
        }
        if (x == 32) {   // ftrunc(FILE, len)
          if (!need(1, 0, 12, 0)) return kEBadParm;
          Fd *f = fd(ld32(pi + 4));
          if (!f || !(f->mode & 2)) return kErrBadF;
          return vfs->truncate(*f->f, ld32(pi + 8));
        }
        if (x == 33) {   // frename(old, new)
          if (!need(3, 0, 12, 0) || !cstr(in[1], ld32(pi + 4), &a) || !cstr(in[2], ld32(pi + 8), &b)) return kEBadParm;
          char p[Vfs::kPath], q[Vfs::kPath];
          Vfs::File *f = Vfs::normalize(a, p) && Vfs::normalize(b, q) ? vfs->find(p) : nullptr;
          if (!f) return kErrNoEnt;
          if (Vfs::File *g = vfs->find(q)) { if (g == f) return kOk; vfs->remove(*g); }
          for (uint32_t i = 0; i < Vfs::kPath; ++i) f->path[i] = q[i];
          note("rename", q, 0, 0);
          return kOk;
        }
        ++unsupported;
        note("unsupported", "apps_std", x, 0);
        return kEUnsupported;
      }
      default:
        ++unsupported;
        note("unsupported", "apps_std", mid, 0);
        return kEUnsupported;
    }
  }
};

// ---- the listener: attach to the sensors PD, register, then serve next2 forever ----
struct Listener {
  enum State : uint32_t { Idle, Attach, OpenDefault, Register, OpenListener, Init2, Next, Failed };
  static constexpr uint32_t kDlerr = 128, kRetryMs = 1000, kTgid = 0x6d70;   // any nonzero group id
  static constexpr uint32_t kMaxErrors = 60;   // a minute of 1 s retries: the sensors PD may come up after the channel
  struct Hooks {
    void *ctx = nullptr;
    int (*send)(void *, const uint8_t *, uint32_t) = nullptr;   // 0 sent, >0 try again, <0 error
    void (*log)(void *, const char *what, const char *text, uint64_t a, uint64_t b) = nullptr;
  } hooks;
  Region shm;
  Server server;
  uint8_t *inStage = nullptr, *outStage = nullptr;   // bufsCap bytes each, 8 aligned
  uint32_t bufsCap = 0;
  State state = Idle;
  Invoke iv;
  uint8_t msg[kMsgBytes] = {};
  bool msgPending = false, inFlight = false;
  uint64_t ctx = 0, retryAt = 0;
  uint32_t seq = 0, hDefault = 0, hListener = 0, errors = 0, served = 0, prevCtx = 0, prevLen = 0;
  int32_t prevResult = -1;

  void note(const char *what, const char *text, uint64_t a, uint64_t b) {
    if (hooks.log) hooks.log(hooks.ctx, what, text ? text : "", a, b);
  }
  static const char *stateName(uint32_t s) {
    static const char *const n[] = {"idle", "attach", "open adsp_default_listener", "register", "open adsp_listener",
                                    "init2", "next2", "failed"};
    return s <= Failed ? n[s] : "?";
  }
  // Queues one invoke: pid 0 for the attach (kernel message), the group id otherwise
  bool invoke(uint32_t handle, uint32_t sc, const uint8_t *const *in, const uint32_t *len) {
    if (!buildInvoke(shm, sc, in, len, &iv)) { note("error", "invoke does not fit", sc, 0); return false; }
    seq = seq % 255 + 1;
    ctx = (uint64_t)seq << 4 | kPdSensors;
    const bool args = scIn(sc) + scOut(sc) > 0;
    encodeMsg(msg, state == Attach ? 0 : kTgid, kTgid, ctx, handle, sc, args ? shm.dsp : 0,
              args ? align(iv.total, kPage) : 0);
    msgPending = true;
    inFlight = true;
    return true;
  }
  bool openModule(const char *name) {
    uint8_t prim[8];
    const uint32_t n = strLen(name, 64) + 1;
    st32(prim, n); st32(prim + 4, kDlerr);
    const uint8_t *in[4] = {prim, reinterpret_cast<const uint8_t *>(name), nullptr, nullptr};
    const uint32_t len[4] = {8, n, 8, kDlerr};
    return invoke(kRemotectl, scalars(0, 2, 2), in, len);
  }
  bool next2() {
    uint8_t prim[16];
    st32(prim, prevCtx); st32(prim + 4, (uint32_t)prevResult); st32(prim + 8, prevLen); st32(prim + 12, bufsCap);
    const uint8_t *in[4] = {prim, outStage, nullptr, nullptr};
    const uint32_t len[4] = {16, prevLen, 16, bufsCap};
    return invoke(hListener, scalars(4, 2, 2), in, len);
  }
  bool enter(State s) {
    state = s;
    switch (s) {
      case Attach: {
        uint8_t tgid[4];
        st32(tgid, kTgid);
        const uint8_t *in[1] = {tgid};
        const uint32_t len[1] = {4};
        return invoke(kInitHandle, scalars(0, 1, 0), in, len);
      }
      case OpenDefault: return openModule("adsp_default_listener");
      case Register: return invoke(hDefault, scalars(0, 0, 0), nullptr, nullptr);
      case OpenListener: return openModule("adsp_listener");
      case Init2: return invoke(hListener, scalars(3, 0, 0), nullptr, nullptr);
      case Next: return next2();
      default: return false;
    }
  }
  void start() {
    if (state != Idle || !shm.cpu || !inStage || !outStage || bufsCap < 4096 || !server.vfs) {
      note("error", "listener not configured", state, bufsCap);
      state = Failed;
      return;
    }
    note("state", stateName(Attach), shm.dsp, shm.bytes);
    if (!enter(Attach)) state = Failed;
  }
  // Transmit the pending message, retry after an error pause; <0 on a transport error
  int step(uint64_t nowMs) {
    if (state == Failed || state == Idle) return 0;
    if (!inFlight && retryAt && nowMs >= retryAt) {
      retryAt = 0;
      prevCtx = 0; prevResult = -1; prevLen = 0;
      if (!enter(state)) state = Failed;
    }
    if (!msgPending) return 0;
    const int e = hooks.send(hooks.ctx, msg, kMsgBytes);
    if (e > 0) return 0;
    if (e < 0) { note("error", "send failed", (uint64_t)(int64_t)e, state); state = Failed; return e; }
    msgPending = false;
    return 0;
  }
  void failStep(int32_t retval, uint64_t nowMs) {
    note("error", stateName(state), (uint64_t)(uint32_t)retval, errors + 1);
    if (++errors >= kMaxErrors) { state = Failed; note("state", stateName(Failed), errors, 0); return; }
    retryAt = nowMs + kRetryMs;   // the same step again (the next2 loop restarts with ctx 0)
  }
  // One message from the channel
  void receive(const uint8_t *p, uint32_t n, uint64_t nowMs) {
    if (n < 12) { note("error", "short response", n, 0); return; }
    const uint64_t rctx = ld64(p);
    const int32_t retval = (int32_t)ld32(p + 8);
    if (n >= 16) {   // smq_invoke_rspv2: flags 1/2 = early wake-up hints, not the completion
      const uint32_t flags = ld32(p + 12);
      if (flags == 1 || flags == 2 || flags == 4) return;
    }
    if (!inFlight || (rctx & 0xff0) != (ctx & 0xff0)) { note("unexpected", "", rctx, (uint64_t)(uint32_t)retval); return; }
    inFlight = false;
    if (retval) { failStep(retval, nowMs); return; }
    switch (state) {
      case Attach: note("state", "attached to the sensors PD", 0, 0); enter(OpenDefault); return;
      case OpenDefault: case OpenListener: {
        const uint32_t h = shm.r32(iv.off[2]), err = shm.r32(iv.off[2] + 4);
        if (err || !h) { failStep(err ? (int32_t)err : (int32_t)kEFailed, nowMs); return; }
        if (state == OpenDefault) { hDefault = h; note("state", "adsp_default_listener open", h, 0); enter(Register); }
        else { hListener = h; note("state", "adsp_listener open", h, 0); enter(Init2); }
        return;
      }
      case Register: note("state", "registered as default listener", 0, 0); enter(OpenListener); return;
      case Init2:
        note("state", "listener up: serving reverse calls", 0, 0);
        prevCtx = 0; prevResult = -1; prevLen = 0;
        enter(Next);
        return;
      case Next: serve(); return;
      default: return;
    }
  }
  // The answer of next2: one reverse call to serve, then the next next2 carries its result
  void serve() {
    errors = 0;
    const uint32_t rctx = shm.r32(iv.off[2]), handle = shm.r32(iv.off[2] + 4), sc = shm.r32(iv.off[2] + 8),
                   req = shm.r32(iv.off[2] + 12);
    int32_t result = kEBadParm;
    prevLen = 0;
    if (req > bufsCap) {
      result = kEBadSize;
      note("error", "reverse call larger than the listener buffer", req, bufsCap);
    } else if (scHandles(sc) || scIn(sc) + scOut(sc) > kMaxArgs) {
      result = kEBadParm;
    } else {
      shm.get(iv.off[3], inStage, req);
      Arg ins[kMaxArgs] = {}, outs[kMaxArgs] = {};
      const uint32_t nin = scIn(sc), nout = scOut(sc);
      uint32_t at = 0;
      bool ok = true;
      for (uint32_t i = 0; i < nin && ok; ++i) {
        if (at + 4 > req) { ok = false; break; }
        const uint32_t l = ld32(inStage + at);
        at += 4;
        if (l) {
          at = align(at, 8);
          if (l > req || at > req - l) { ok = false; break; }
          ins[i] = {inStage + at, nullptr, l};
          at += l;
        }
      }
      uint32_t o = 0;
      for (uint32_t j = 0; j < nout && ok; ++j) {
        if (at + 4 > req) { ok = false; break; }
        const uint32_t l = ld32(inStage + at);
        at += 4;
        if (o + 4 > bufsCap) { ok = false; break; }
        st32(outStage + o, l);
        o += 4;
        if (l) {
          o = align(o, 8);
          if (l > bufsCap || o > bufsCap - l) { ok = false; break; }
          for (uint32_t k = 0; k < l; ++k) outStage[o + k] = 0;
          outs[j] = {nullptr, outStage + o, l};
          o += l;
        }
      }
      if (!ok) {
        note("error", "malformed reverse call", sc, req);
      } else {
        result = server.invoke(handle, sc, ins, nin, outs, nout);
        if (!result) prevLen = o;
      }
    }
    ++served;
    if (served == 1) note("state", "first reverse call served", handle, sc);
    prevCtx = rctx;
    prevResult = result;
    if (!enter(Next)) state = Failed;
  }
};

}  // namespace guamp_rpc
