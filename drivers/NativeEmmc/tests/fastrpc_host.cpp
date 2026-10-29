// Host test of GuampFastRpcCore.h and GuampSnsFiles.h: the in-memory files and apps_std on their own, then the whole
// listener against a simulated ADSP sensors PD (attach, remotectl opens, register, init2, next2 loop with reverse calls
// decoded and answered the way listener_android.c / apps_std_skel.c do), and the persist loader on the backup
// persist image when its path is given (argv[1]).
#include <string.h>
#include "GuampSnsFiles.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
using namespace guamp_rpc;
static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

static size_t live = 0;
static void *halloc(void *, uint32_t n) { live += n; return malloc(n ? n : 1); }
static void hfree(void *, void *p, uint32_t n) { live -= n; free(p); }
static std::vector<std::string> logs;
static void hlog(void *, const char *what, const char *text, uint64_t a, uint64_t b) {
  char l[400];
  snprintf(l, sizeof(l), "%s|%s|%llx|%llx", what, text, (unsigned long long)a, (unsigned long long)b);
  logs.push_back(l);
  if (getenv("RPC_LOG")) puts(l);
}
static bool logged(const char *prefix) {
  for (auto &l : logs) if (l.rfind(prefix, 0) == 0) return true;
  return false;
}

// ---- direct calls of the skeleton methods ----
struct Call {
  std::vector<std::vector<uint8_t>> in, out;
  std::vector<Arg> ai, ao;
  void prim(std::initializer_list<uint32_t> w) { std::vector<uint8_t> b(4 * w.size()); uint32_t i = 0; for (uint32_t v : w) st32(&b[4 * i++], v); in.push_back(b); }
  void str(const char *s) { in.emplace_back((const uint8_t *)s, (const uint8_t *)s + strlen(s) + 1); }
  void bytes(const void *p, uint32_t n) { in.emplace_back((const uint8_t *)p, (const uint8_t *)p + n); }
  void outbuf(uint32_t n) { out.emplace_back(n, 0xcc); }
  int run(Server &s, uint32_t handle, uint32_t mid) {
    ai.clear(); ao.clear();
    for (auto &b : in) ai.push_back({b.data(), nullptr, (uint32_t)b.size()});
    for (auto &b : out) ao.push_back({nullptr, b.data(), (uint32_t)b.size()});
    return s.invoke(handle, scalars(mid, (uint32_t)in.size(), (uint32_t)out.size()), ai.data(), (uint32_t)ai.size(), ao.data(),
                    (uint32_t)ao.size());
  }
  uint32_t o32(uint32_t b, uint32_t at) { return ld32(&out[b][at]); }
};
static int fopenCall(Server &s, const char *name, const char *mode, uint32_t *fd) {
  Call c; c.prim({(uint32_t)strlen(name) + 1, (uint32_t)strlen(mode) + 1}); c.str(name); c.str(mode); c.outbuf(4);
  const int e = c.run(s, kStdHandle, 0);
  *fd = c.o32(0, 0);
  return e;
}
static std::string freadAll(Server &s, uint32_t fd) {
  std::string r;
  for (;;) {
    Call c; c.prim({fd, 7}); c.outbuf(8); c.outbuf(7);
    if (c.run(s, kStdHandle, 4)) return "ERR";
    r.append((const char *)c.out[1].data(), c.o32(0, 0));
    if (c.o32(0, 4)) return r;
  }
}

static void testServer() {
  static Vfs vfs;
  vfs.hooks = {nullptr, halloc, hfree};
  static Server s;
  s.vfs = &vfs;
  s.hooks.log = hlog;
  static const char kOrient[] = "{\"lsm6dso_0_platform.orient\":{\"x\":\"-y\"}}\nline2\n";
  CHECK(vfs.add("/mnt/vendor/persist/sensors/registry/registry/lsm6dso_0_platform.orient", (const uint8_t *)kOrient,
                sizeof(kOrient) - 1, false));
  CHECK(vfs.add("/mnt/vendor/persist/sensors/registry/registry/vcnl36866_0_platform", (const uint8_t *)"{}", 2, true));
  CHECK(vfs.add("/mnt/vendor/persist/sensors/registry/sns_reg_version", (const uint8_t *)"version=4 \n", 11, true));
  CHECK(addSynthetic(vfs) == 7);
  CHECK(addSynthetic(vfs) == 0);   // never twice
  CHECK(!vfs.add("relative", nullptr, 0, false));
  // remotectl: apps_std is served, anything else refused with a dlerror text
  Call o; o.prim({9, 64}); o.str("apps_std"); o.outbuf(8); o.outbuf(64);
  CHECK(o.run(s, kRemotectl, 0) == kOk && o.o32(0, 0) == kStdHandle && o.o32(0, 4) == 0);
  Call m; m.prim({9, 64}); m.str("apps_mem"); m.outbuf(8); m.outbuf(64);
  CHECK(m.run(s, kRemotectl, 0) == kENoSuchMod && strstr((const char *)m.out[1].data(), "not served"));
  // fopen + fread in chunks of 7 + EOF
  uint32_t fd = 0;
  CHECK(fopenCall(s, "/mnt/vendor/persist/sensors/registry/registry//lsm6dso_0_platform.orient", "r", &fd) == kOk && fd >= 3);
  CHECK(freadAll(s, fd) == kOrient);
  { Call c; c.prim({fd}); c.outbuf(8); CHECK(c.run(s, kStdHandle, 10) == kOk && ld64(c.out[0].data()) == sizeof(kOrient) - 1); }
  // fseek to 0, fgets two lines
  { Call c; c.prim({fd, 0, 0}); CHECK(c.run(s, kStdHandle, 9) == kOk); }
  { Call c; c.prim({fd, 100}); c.outbuf(4); c.outbuf(100); CHECK(c.run(s, kStdHandle, 20) == kOk && std::string((char *)c.out[1].data()) == "{\"lsm6dso_0_platform.orient\":{\"x\":\"-y\"}}\n"); }
  { Call c; c.prim({fd, 100}); c.outbuf(4); c.outbuf(100); CHECK(c.run(s, kStdHandle, 20) == kOk && std::string((char *)c.out[1].data()) == "line2\n"); }
  { Call c; c.prim({fd}); c.outbuf(4); CHECK(c.run(s, kStdHandle, 8) == kOk && c.o32(0, 0) == sizeof(kOrient) - 1); }
  // write to a read-only handle fails; close
  { Call c; c.prim({fd, 3}); c.bytes("abc", 3); c.outbuf(8); CHECK(c.run(s, kStdHandle, 5) == kErrBadF); }
  { Call c; c.prim({fd}); CHECK(c.run(s, kStdHandle, 3) == kOk); }
  { Call c; c.prim({fd}); CHECK(c.run(s, kStdHandle, 3) == kErrBadF); }
  // missing files: ENOENT as apps_std_imp, logged
  uint32_t f2 = 0;
  CHECK(fopenCall(s, "/mnt/vendor/persist/sensors/registry/hw_device", "r", &f2) == kErrNoEnt && logged("open-miss|/mnt/vendor/persist/sensors/registry/hw_device"));
  CHECK(fopenCall(s, "/nowhere/x", "w", &f2) == kErrNoEnt);
  // write: a new file (RAM), then copy-on-write of the borrowed one
  CHECK(fopenCall(s, "/mnt/vendor/persist/sensors/registry/registry/sns_gyro_cal_persist_s0", "w", &f2) == kOk);
  { Call c; c.prim({f2, 5}); c.bytes("hello", 5); c.outbuf(8); CHECK(c.run(s, kStdHandle, 5) == kOk && c.o32(0, 0) == 5); }
  { Call c; c.prim({f2}); CHECK(c.run(s, kStdHandle, 3) == kOk); }
  CHECK(fopenCall(s, "/mnt/vendor/persist/sensors/registry/registry/sns_gyro_cal_persist_s0", "a", &f2) == kOk);
  { Call c; c.prim({f2, 3}); c.bytes("!!!", 3); c.outbuf(8); CHECK(c.run(s, kStdHandle, 5) == kOk); }
  { Call c; c.prim({f2}); CHECK(c.run(s, kStdHandle, 3) == kOk); }
  CHECK(fopenCall(s, "/mnt/vendor/persist/sensors/registry/registry/sns_gyro_cal_persist_s0", "r", &f2) == kOk && freadAll(s, f2) == "hello!!!");
  { Call c; c.prim({f2}); c.run(s, kStdHandle, 3); }
  CHECK(fopenCall(s, "/mnt/vendor/persist/sensors/registry/registry/lsm6dso_0_platform.orient", "r+", &f2) == kOk);
  { Call c; c.prim({f2, 1}); c.bytes("[", 1); c.outbuf(8); CHECK(c.run(s, kStdHandle, 5) == kOk); }
  { Call c; c.prim({f2}); c.run(s, kStdHandle, 3); }
  CHECK(kOrient[0] == '{');   // the borrowed original is untouched
  CHECK(fopenCall(s, "/mnt/vendor/persist/sensors/registry/registry/lsm6dso_0_platform.orient", "r", &f2) == kOk && freadAll(s, f2)[0] == '[');
  { Call c; c.prim({f2}); c.run(s, kStdHandle, 3); }
  // stat (extended method 31) of a file, a directory, a missing path
  { Call c; c.prim({31, 39}); c.str("/mnt/vendor/persist/sensors/registry/sns_reg_version"); c.in[0][4] = (uint8_t)c.in[1].size(); c.outbuf(96);
    CHECK(c.run(s, kStdHandle, 31) == kOk && ld32(&c.out[0][24]) == 0100644 && ld64(&c.out[0][40]) == 11); }
  { Call c; c.prim({31, 0}); c.str("/mnt/vendor/persist/sensors"); c.in[0][4] = (uint8_t)c.in[1].size(); c.outbuf(96);
    CHECK(c.run(s, kStdHandle, 31) == kOk && ld32(&c.out[0][24]) == 040755); }
  { Call c; c.prim({31, 0}); c.str("/mnt/vendor/persist/none"); c.in[0][4] = (uint8_t)c.in[1].size(); c.outbuf(96);
    CHECK(c.run(s, kStdHandle, 31) == kErrNoEnt); }
  // fileExists
  { Call c; c.prim({0}); c.str("/vendor/etc/sensors/sns_reg_config"); c.in[0][0] = (uint8_t)c.in[1].size(); c.outbuf(1);
    CHECK(c.run(s, kStdHandle, 22) == kOk && c.out[0][0] == 1); }
  { Call c; c.prim({0}); c.str("/vendor/etc/sensors/x"); c.in[0][0] = (uint8_t)c.in[1].size(); c.outbuf(1);
    CHECK(c.run(s, kStdHandle, 22) == kOk && c.out[0][0] == 0); }
  // opendir/readdir: direct children once each (files and the new file), then EOF; the config dir exists empty
  std::vector<std::string> names;
  { Call c; c.prim({0}); c.str("/mnt/vendor/persist/sensors/registry"); c.in[0][0] = (uint8_t)c.in[1].size(); c.outbuf(8);
    CHECK(c.run(s, kStdHandle, 26) == kOk);
    const uint64_t h = ld64(c.out[0].data());
    for (int i = 0; i < 10; ++i) {
      Call r; r.in.emplace_back(8); st64(r.in[0].data(), h); r.outbuf(264);
      CHECK(r.run(s, kStdHandle, 28) == kOk);
      if (r.o32(0, 260)) break;
      names.push_back((const char *)&r.out[0][4]);
    }
    Call cl; cl.in.emplace_back(8); st64(cl.in[0].data(), h); CHECK(cl.run(s, kStdHandle, 27) == kOk); }
  CHECK(names.size() == 2 && names[0] == "registry" && names[1] == "sns_reg_version");
  { Call c; c.prim({0}); c.str("/vendor/etc/sensors/config"); c.in[0][0] = (uint8_t)c.in[1].size(); c.outbuf(8);
    CHECK(c.run(s, kStdHandle, 26) == kOk);
    Call r; r.in.emplace_back(8); st64(r.in[0].data(), ld64(c.out[0].data())); r.outbuf(264);
    CHECK(r.run(s, kStdHandle, 28) == kOk && r.o32(0, 260) == 1); }
  // mkdir + rename + remove
  { Call c; c.prim({0, 0755}); c.str("/mnt/vendor/persist/sensors/registry/tmp"); c.in[0][0] = (uint8_t)c.in[1].size(); CHECK(c.run(s, kStdHandle, 29) == kOk); }
  CHECK(vfs.isDir("/mnt/vendor/persist/sensors/registry/tmp"));
  { Call c; c.prim({33, 0, 0}); c.str("/mnt/vendor/persist/sensors/registry/registry/sns_gyro_cal_persist_s0");
    c.str("/mnt/vendor/persist/sensors/registry/tmp/g"); c.in[0][4] = (uint8_t)c.in[1].size(); c.in[0][8] = (uint8_t)c.in[2].size();
    CHECK(c.run(s, kStdHandle, 31) == kOk); }
  CHECK(fopenCall(s, "/mnt/vendor/persist/sensors/registry/tmp/g", "r", &f2) == kOk && freadAll(s, f2) == "hello!!!");
  { Call c; c.prim({f2}); c.run(s, kStdHandle, 3); }
  { Call c; c.prim({0}); c.str("/mnt/vendor/persist/sensors/registry/tmp/g"); c.in[0][0] = (uint8_t)c.in[1].size(); CHECK(c.run(s, kStdHandle, 24) == kOk); }
  CHECK(fopenCall(s, "/mnt/vendor/persist/sensors/registry/tmp/g", "r", &f2) == kErrNoEnt);
  // print_string goes to the log; unknown methods are refused
  { Call c; c.prim({0}); c.str("hello from the dsp"); c.in[0][0] = (uint8_t)c.in[1].size(); CHECK(c.run(s, kStdHandle, 15) == kOk && logged("dsp|hello from the dsp")); }
  { Call c; c.prim({0}); CHECK(c.run(s, kStdHandle, 25) == kEBadParm || c.run(s, kStdHandle, 25) == kEUnsupported); }
  { Call c; c.prim({34, 1}); CHECK(c.run(s, kStdHandle, 31) == kEUnsupported); }
  // malformed: string without NUL, wrong argument counts
  { Call c; c.prim({3, 2}); c.bytes("abc", 3); c.bytes("r", 2); c.outbuf(4); CHECK(c.run(s, kStdHandle, 0) == kEBadParm); }
  { Call c; c.prim({1}); CHECK(c.run(s, kStdHandle, 4) == kEBadParm); }
  // blob
  std::vector<uint8_t> blob = {'G', 'S', 'N', 'S', 'C', 'F', 'G', '1', 2, 0, 0, 0};
  auto put = [&](const char *path, const char *data) {
    uint8_t h[8]; st32(h, (uint32_t)strlen(path)); st32(h + 4, (uint32_t)strlen(data));
    blob.insert(blob.end(), h, h + 8); blob.insert(blob.end(), path, path + strlen(path)); while (blob.size() & 3) blob.push_back(0);
    blob.insert(blob.end(), data, data + strlen(data)); while (blob.size() & 3) blob.push_back(0);
  };
  put("/vendor/etc/sensors/config/bengal_lsm6dso_0.json", "{\"config\":{}}");
  put("/vendor/etc/sensors/hals.conf", "x");
  uint32_t nb = 0;
  CHECK(loadBlob(blob.data(), (uint32_t)blob.size(), vfs, &nb) && nb == 2);
  CHECK(vfs.find("/vendor/etc/sensors/config/bengal_lsm6dso_0.json") && vfs.find("/vendor/etc/sensors/config/bengal_lsm6dso_0.json")->size == 13);
  blob[8] = 3;
  CHECK(!loadBlob(blob.data(), (uint32_t)blob.size(), vfs, &nb));
  blob[0] = 'X';
  CHECK(!loadBlob(blob.data(), (uint32_t)blob.size(), vfs, &nb) && nb == 0);
  for (uint32_t i = 0; i < vfs.count; ++i) if (vfs.files[i].used) vfs.remove(vfs.files[i]);
  CHECK(live == 0 && vfs.ramBytes == 0);
}

// ---- the simulated ADSP ----
static std::vector<uint8_t> shared(256 * 1024);
static const uint64_t kDsp = 0x380000000ull;   // IOVA 0x80000000 with context bank 3 in bits 32+
static std::vector<std::vector<uint8_t>> sent;
static int busy = 0;
static int sendHook(void *, const uint8_t *p, uint32_t n) {
  if (busy) { --busy; return 1; }
  sent.emplace_back(p, p + n);
  return 0;
}
struct Seen { uint32_t pid, tid, handle, sc; uint64_t ctx, addr, size; std::vector<std::vector<uint8_t>> in; std::vector<uint64_t> outAt; std::vector<uint32_t> outLen; };
// Decodes the last message and its payload the way fastrpc_invoke.c would: every buffer inside the mapped pages
static Seen decode(const std::vector<uint8_t> &m) {
  Seen s{};
  CHECK(m.size() == kMsgBytes);
  s.pid = ld32(&m[0]); s.tid = ld32(&m[4]); s.ctx = ld64(&m[8]); s.handle = ld32(&m[16]); s.sc = ld32(&m[20]);
  s.addr = ld64(&m[24]); s.size = ld64(&m[32]);
  CHECK((s.ctx & 0xf) == kPdSensors && (s.ctx & 0xff0));
  const uint32_t nin = scIn(s.sc), n = nin + scOut(s.sc);
  if (!n) { CHECK(s.addr == 0 && s.size == 0); return s; }
  CHECK(s.addr == kDsp && s.size % 4096 == 0 && s.size <= shared.size());
  const uint8_t *meta = shared.data();
  for (uint32_t i = 0; i < n; ++i) {
    const uint64_t pv = ld64(meta + 16 * i), len = ld64(meta + 16 * i + 8);
    const uint32_t num = ld32(meta + 16 * n + 8 * i), pg = ld32(meta + 16 * n + 8 * i + 4);
    const uint64_t pa = ld64(meta + 24 * n + 16 * pg), ps = ld64(meta + 24 * n + 16 * pg + 8);
    CHECK(pg == i && num == (len ? 1u : 0u));
    if (len) {
      CHECK(pv >= kDsp && pv + len <= kDsp + s.size && (pv & 127) == 0);
      CHECK(pa == (pv & ~0xfffull) && pa + ps >= pv + len && ps % 4096 == 0);
      CHECK(pv - kDsp >= metaBytes(n));
    }
    if (i < nin) s.in.emplace_back(shared.begin() + (pv - kDsp), shared.begin() + (pv - kDsp) + len);
    else { s.outAt.push_back(pv - kDsp); s.outLen.push_back((uint32_t)len); }
  }
  for (uint32_t i = 0; i < 16 * 8 + 64 * 4; ++i) CHECK(meta[40 * n + i] == 0);   // fdlist and crclist clear
  return s;
}
static std::vector<uint8_t> rsp(uint64_t ctx, int32_t ret, bool v2 = false, uint32_t flags = 0) {
  std::vector<uint8_t> r(v2 ? 24 : 16, 0);
  st64(&r[0], ctx); st32(&r[8], (uint32_t)ret);
  if (v2) st32(&r[12], flags);
  return r;
}
// One reverse call packed as the DSP packs it in next2's out buffer (pack_in_bufs + pack_out_lens)
struct Reverse { uint32_t ctx, handle, sc; std::vector<std::vector<uint8_t>> in; std::vector<uint32_t> outLen; };
static void answerNext(const Seen &s, const Reverse &r) {
  CHECK(s.handle == 0x77 && s.sc == scalars(4, 2, 2) && s.outLen.size() == 2 && s.outLen[0] == 16);
  std::vector<uint8_t> b;
  for (auto &x : r.in) {
    uint8_t l[4]; st32(l, (uint32_t)x.size()); b.insert(b.end(), l, l + 4);
    if (x.size()) { while (b.size() & 7) b.push_back(0xee); b.insert(b.end(), x.begin(), x.end()); }
  }
  for (uint32_t l : r.outLen) { uint8_t q[4]; st32(q, l); b.insert(b.end(), q, q + 4); }
  CHECK(b.size() <= s.outLen[1]);
  memcpy(&shared[s.outAt[1]], b.data(), b.size());
  st32(&shared[s.outAt[0]], r.ctx); st32(&shared[s.outAt[0] + 4], r.handle); st32(&shared[s.outAt[0] + 8], r.sc);
  st32(&shared[s.outAt[0] + 12], (uint32_t)b.size());
}
// The answer of the previous reverse call, as the next next2 carries it (prevCtx, prevResult, out buffers)
static std::vector<std::vector<uint8_t>> previous(const Seen &s, uint32_t *ctx, int32_t *result, const std::vector<uint32_t> &lens) {
  const uint8_t *p = s.in[0].data();
  *ctx = ld32(p); *result = (int32_t)ld32(p + 4);
  const uint32_t len = ld32(p + 8);
  CHECK(len == (s.in.size() > 1 ? s.in[1].size() : 0) && ld32(p + 12) >= 4096);
  std::vector<std::vector<uint8_t>> outs;
  uint32_t at = 0;
  for (uint32_t l : lens) {
    if (!len) break;
    CHECK(ld32(&s.in[1][at]) == l);
    at += 4;
    if (l) { at = (at + 7) & ~7u; outs.emplace_back(s.in[1].begin() + at, s.in[1].begin() + at + l); at += l; }
    else outs.emplace_back();
  }
  CHECK(at == len);
  return outs;
}
static std::vector<uint8_t> w32s(std::initializer_list<uint32_t> w) { std::vector<uint8_t> b; for (uint32_t v : w) { uint8_t q[4]; st32(q, v); b.insert(b.end(), q, q + 4); } return b; }
static std::vector<uint8_t> cs(const char *s) { return std::vector<uint8_t>(s, s + strlen(s) + 1); }

static void testListener() {
  static Vfs vfs;
  vfs.hooks = {nullptr, halloc, hfree};
  static const char kReg[] = "{\"vcnl36866_0_platform\":{}}";
  vfs.add("/mnt/vendor/persist/sensors/registry/registry/vcnl36866_0_platform", (const uint8_t *)kReg, sizeof(kReg) - 1, false);
  addSynthetic(vfs);
  static Listener l;
  static uint8_t in[64 * 1024], out[64 * 1024];
  l.hooks = {nullptr, sendHook, hlog};
  l.server.vfs = &vfs;
  l.server.hooks.log = hlog;
  l.shm = {shared.data(), kDsp, (uint32_t)shared.size()};
  l.inStage = in; l.outStage = out; l.bufsCap = sizeof(in);
  l.start();
  busy = 1;
  CHECK(l.step(0) == 0 && sent.empty());   // transport busy: kept
  CHECK(l.step(1) == 0 && sent.size() == 1);
  // attach: handle 1, method 0, one 4-byte input (the group id), pid 0
  Seen s = decode(sent.back());
  CHECK(s.handle == kInitHandle && s.sc == scalars(0, 1, 0) && s.pid == 0 && s.tid == Listener::kTgid && s.in[0].size() == 4 &&
        ld32(s.in[0].data()) == Listener::kTgid);
  l.receive(rsp(s.ctx ^ 0x10, 0).data(), 16, 2);   // someone else's context
  CHECK(logged("unexpected"));
  l.receive(rsp(s.ctx, 0, true, 1).data(), 24, 2);   // early wake-up hint: not the completion
  CHECK(l.state == Listener::Attach);
  l.receive(rsp(s.ctx, 0).data(), 16, 3);
  l.step(3);
  // remotectl open "adsp_default_listener"
  s = decode(sent.back());
  CHECK(s.handle == kRemotectl && s.sc == scalars(0, 2, 2) && s.pid == Listener::kTgid);
  CHECK(ld32(s.in[0].data()) == 22 && std::string((const char *)s.in[1].data()) == "adsp_default_listener" && s.outLen[0] == 8);
  st32(&shared[s.outAt[0]], 0x55); st32(&shared[s.outAt[0] + 4], 0);
  l.receive(rsp(s.ctx, 0).data(), 16, 4);
  l.step(4);
  s = decode(sent.back());
  CHECK(s.handle == 0x55 && s.sc == scalars(0, 0, 0));   // register
  l.receive(rsp(s.ctx, 0).data(), 16, 5);
  l.step(5);
  s = decode(sent.back());
  CHECK(s.handle == kRemotectl && std::string((const char *)s.in[1].data()) == "adsp_listener");
  st32(&shared[s.outAt[0]], 0x77); st32(&shared[s.outAt[0] + 4], 0);
  l.receive(rsp(s.ctx, 0).data(), 16, 6);
  l.step(6);
  s = decode(sent.back());
  CHECK(s.handle == 0x77 && s.sc == scalars(3, 0, 0));   // init2
  l.receive(rsp(s.ctx, 0).data(), 16, 7);
  l.step(7);
  // next2 #1: prevCtx 0, prevResult -1, nothing to give back
  s = decode(sent.back());
  uint32_t pc; int32_t pr;
  previous(s, &pc, &pr, {});
  CHECK(pc == 0 && pr == -1 && s.in.size() == 2 && s.in[1].empty());
  // reverse call 1: apps_remotectl open "apps_std"
  answerNext(s, {0x1001, 0, scalars(0, 2, 2), {w32s({9, 64}), cs("apps_std")}, {8, 64}});
  l.receive(rsp(s.ctx, 0).data(), 16, 8);
  l.step(8);
  s = decode(sent.back());
  auto outs = previous(s, &pc, &pr, {8, 64});
  CHECK(pc == 0x1001 && pr == 0 && outs.size() == 2 && ld32(outs[0].data()) == kStdHandle);
  // reverse call 2: fopen of the registry group
  const char *path = "/mnt/vendor/persist/sensors/registry/registry/vcnl36866_0_platform";
  answerNext(s, {0x1002, kStdHandle, scalars(0, 3, 1), {w32s({(uint32_t)strlen(path) + 1, 2}), cs(path), cs("r")}, {4}});
  l.receive(rsp(s.ctx, 0).data(), 16, 9);
  l.step(9);
  s = decode(sent.back());
  outs = previous(s, &pc, &pr, {4});
  const uint32_t fd = ld32(outs[0].data());
  CHECK(pc == 0x1002 && pr == 0 && fd >= 3);
  // reverse call 3: fread 4096
  answerNext(s, {0x1003, kStdHandle, scalars(4, 1, 2), {w32s({fd, 4096})}, {8, 4096}});
  l.receive(rsp(s.ctx, 0).data(), 16, 10);
  l.step(10);
  s = decode(sent.back());
  outs = previous(s, &pc, &pr, {8, 4096});
  CHECK(pc == 0x1003 && pr == 0 && ld32(outs[0].data()) == sizeof(kReg) - 1 && ld32(outs[0].data() + 4) == 1 &&
        memcmp(outs[1].data(), kReg, sizeof(kReg) - 1) == 0);
  // reverse call 4: fopen of a missing file -> errno 2, no out buffers given back
  answerNext(s, {0x1004, kStdHandle, scalars(0, 3, 1), {w32s({10, 2}), cs("/missing1"), cs("r")}, {4}});
  l.receive(rsp(s.ctx, 0).data(), 16, 11);
  l.step(11);
  s = decode(sent.back());
  previous(s, &pc, &pr, {4});
  CHECK(pc == 0x1004 && pr == kErrNoEnt && s.in[1].empty());
  // reverse call 5: larger than the listener buffer -> AEE_EBADSIZE
  answerNext(s, {0x1005, kStdHandle, scalars(4, 1, 2), {w32s({fd, 16})}, {8, 16}});
  st32(&shared[s.outAt[0] + 12], 70000);
  l.receive(rsp(s.ctx, 0).data(), 16, 12);
  l.step(12);
  s = decode(sent.back());
  previous(s, &pc, &pr, {});
  CHECK(pc == 0x1005 && pr == kEBadSize);
  // next2 fails on the DSP side: the loop restarts after the pause with ctx 0
  l.receive(rsp(s.ctx, 0x8000040d).data(), 16, 13);
  const size_t before = sent.size();
  l.step(14);
  CHECK(sent.size() == before);
  l.step(13 + Listener::kRetryMs);
  CHECK(sent.size() == before + 1);
  s = decode(sent.back());
  previous(s, &pc, &pr, {});
  CHECK(pc == 0 && pr == -1 && l.state == Listener::Next);
  CHECK(logged("state|listener up") && logged("state|first reverse call served") && logged("module|apps_std"));
  // a transport error stops the listener
  answerNext(s, {0x1006, kStdHandle, scalars(4, 1, 2), {w32s({fd, 16})}, {8, 16}});
  l.receive(rsp(s.ctx, 0).data(), 16, 2000);
  l.hooks.send = [](void *, const uint8_t *, uint32_t) { return -1; };
  CHECK(l.step(2000) < 0 && l.state == Listener::Failed);
}

// ---- persist image (optional) ----
static FILE *img;
static uint64_t imgSectors;
static bool readImg(void *, uint64_t sector, uint32_t count, uint8_t *out) {
  return fseek(img, (long)(sector * 512), SEEK_SET) == 0 && fread(out, 512, count, img) == count;
}
static void testPersist(const char *path) {
  img = fopen(path, "rb");
  if (!img) { printf("persist image %s not readable: skipped\n", path); return; }
  fseek(img, 0, SEEK_END);
  imgSectors = (uint64_t)ftell(img) / 512;
  static Vfs vfs;
  vfs.hooks = {nullptr, halloc, hfree};
  static ext4::Fs fs;
  static uint8_t scratch[ext4::kScratchBytes];
  CHECK(fs.mount(readImg, nullptr, imgSectors, scratch) == ext4::kOk);
  const TreeStats st = loadExt4(fs, "/sensors", kPersistMount, vfs);
  printf("persist: %u files, %u dirs, %u bytes, %u skipped, error %d\n", st.files, st.dirs, st.bytes, st.skipped, st.error);
  CHECK(st.error == 0 && st.files >= 170 && st.dirs >= 3);
  Vfs::File *f = vfs.find("/mnt/vendor/persist/sensors/registry/registry/lsm6dso_0_platform.orient");
  CHECK(f && f->size > 20 && memcmp(f->data, "{\"lsm6dso_0_platform.orient\"", 27) == 0);
  f = vfs.find("/mnt/vendor/persist/sensors/registry/sns_reg_version");
  CHECK(f && f->size >= 9 && memcmp(f->data, "version=4", 9) == 0);
  CHECK(vfs.isDir("/mnt/vendor/persist/sensors/registry/registry"));
  CHECK(!loadExt4(fs, "/nothing", kPersistMount, vfs).files);
  fclose(img);
  for (uint32_t i = 0; i < vfs.count; ++i) if (vfs.files[i].used) vfs.remove(vfs.files[i]);
}

int main(int argc, char **argv) {
  testServer();
  testListener();
  if (argc > 1) testPersist(argv[1]);
  if (argc > 2) {   // the blob of scripts/guamp-sensors-config.py built from the vendor backup
    FILE *b = fopen(argv[2], "rb");
    if (!b) printf("blob %s not readable: skipped\n", argv[2]);
    else {
      std::vector<uint8_t> blob(1 << 20);
      blob.resize(fread(blob.data(), 1, blob.size(), b));
      fclose(b);
      static Vfs vfs;
      vfs.hooks = {nullptr, halloc, hfree};
      uint32_t files = 0;
      CHECK(loadBlob(blob.data(), (uint32_t)blob.size(), vfs, &files) && files >= 40);
      Vfs::File *f = vfs.find("/vendor/etc/sensors/sns_reg_config");
      CHECK(f && f->size >= 9 && memcmp(f->data, "version=4", 9) == 0);
      f = vfs.find("/vendor/etc/sensors/config/bengal_lsm6dso_0.json");
      CHECK(f && f->size > 100 && f->data[0] == '{');
      CHECK(addSynthetic(vfs) == 6);   // sns_reg_config came from the blob
      printf("blob: %u files\n", files);
    }
  }
  // the payload layout: metadata and fd/crc lists, 128-byte aligned buffers, pages cover each buffer
  Region r{shared.data(), kDsp, (uint32_t)shared.size()};
  Invoke iv;
  const uint8_t a[5] = {1, 2, 3, 4, 5};
  const uint8_t *ins[3] = {a, nullptr, nullptr};
  const uint32_t lens[3] = {5, 0, 4097};
  CHECK(buildInvoke(r, scalars(1, 2, 1), ins, lens, &iv) && iv.n == 3 && iv.off[0] == 512 && iv.off[1] == 0 && iv.off[2] == 640);
  CHECK(ld64(&shared[24 * 3 + 16 * 2 + 8]) == 8192);   // 4097 bytes from offset 640 span two pages
  const uint32_t big[1] = {300 * 1024};
  CHECK(!buildInvoke(r, scalars(1, 1, 0), ins, big, &iv));
  CHECK(!buildInvoke(r, scalars(1, 1, 0) | 0x11, ins, lens, &iv));   // handles are not supported
  printf("fastrpc_host: %s (%zu messages, %zu log lines)\n", fails ? "FAIL" : "PASS", sent.size(), logs.size());
  return fails ? 1 : 0;
}
