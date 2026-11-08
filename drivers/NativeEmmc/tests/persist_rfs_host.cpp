// guamp 08/11 (fin61): GuampExt4.inc + GuampWifiFirmware.inc (kernel files, unchanged) read the modem's persisted /readwrite
// files from a persist partition image (backup persist.img) under /rfs/msm/mpss/, compared with files extracted by debugfs.
// Usage: persist_rfs_host <persist.img> <dir with the debugfs dumps: ticf.bin imei_sv id_00 mcfg (one mcfg_sw file)> <mcfg name>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <string>
#include <vector>
#include "../GuampWifiWire.inc"
#include "../GuampExt4.inc"
#include "../GuampWifiFirmware.inc"
using namespace guampwifi;
static unsigned checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)

struct Disk { int fd = -1; uint64_t sectors = 0; };
static bool readSectors(void *ctx, uint64_t sector, uint32_t count, uint8_t *out) {
  Disk *d = static_cast<Disk *>(ctx);
  if (!count || count > 128 || sector + count > d->sectors) return false;
  return pread(d->fd, out, count * 512ull, static_cast<off_t>(sector * 512)) == static_cast<ssize_t>(count * 512ull);
}
static std::vector<uint8_t> hostFile(const std::string &p) {
  std::vector<uint8_t> v; FILE *f = fopen(p.c_str(), "rb"); if (!f) return v;
  fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET); v.resize(size_t(n));
  if (n && fread(v.data(), 1, v.size(), f) != v.size()) v.clear(); fclose(f); return v;
}
static uint8_t scratch[ext4::kScratchBytes];

static void same(Ext4FirmwareSource &src, const std::string &path, const std::vector<uint8_t> &want) {
  uint32_t n = 0;
  CHECK(!want.empty());
  CHECK(src.size(path.c_str(), &n) == Ok && n == want.size());
  std::vector<uint8_t> got(n);
  for (uint32_t at = 0; at < n;) { uint32_t c = n - at > 4096 ? 4096 : n - at; CHECK(src.read(path.c_str(), at, got.data() + at, c) == Ok); at += c; }
  CHECK(got == want);
}

int main(int argc, char **argv) {
  // allow-list rooted elsewhere: same rules as allowed() for /image/
  CHECK(Ext4FirmwareSource::allowed("/image/a.mbn") && !Ext4FirmwareSource::allowed("/image/") && !Ext4FirmwareSource::allowed("/rfs/x"));
  CHECK(Ext4FirmwareSource::allowedUnder("/rfs/msm/mpss/ticf.bin", "/rfs/msm/mpss/"));
  CHECK(Ext4FirmwareSource::allowedUnder("/rfs/msm/mpss/mot_rfs/imei_sv", "/rfs/msm/mpss/"));
  CHECK(!Ext4FirmwareSource::allowedUnder("/rfs/msm/mpss/", "/rfs/msm/mpss/"));
  CHECK(!Ext4FirmwareSource::allowedUnder("/rfs/msm/mpss/../../../sensors/x", "/rfs/msm/mpss/"));
  CHECK(!Ext4FirmwareSource::allowedUnder("/rfs/msm/mpss/a//b", "/rfs/msm/mpss/"));
  CHECK(!Ext4FirmwareSource::allowedUnder("/rfs/msm/mpss/a/..", "/rfs/msm/mpss/"));
  CHECK(!Ext4FirmwareSource::allowedUnder("/rfs/msm/mpssX/a", "/rfs/msm/mpss/"));
  CHECK(!Ext4FirmwareSource::allowedUnder("/image/a.mbn", "/rfs/msm/mpss/"));
  CHECK(!Ext4FirmwareSource::allowedUnder("/rfs/a", "/rfs"));          // the root must end with '/'
  if (argc < 4) { printf("PASS %u checks (no image given)\n", checks); return 0; }
  Disk d; d.fd = open(argv[1], O_RDONLY); CHECK(d.fd >= 0);
  struct stat st; fstat(d.fd, &st); d.sectors = uint64_t(st.st_size) / 512;
  ext4::Fs fs{};
  CHECK(fs.mount(readSectors, &d, d.sectors, scratch) == ext4::kOk);
  Ext4FirmwareSource src(&fs, "/rfs/msm/mpss/"), image(&fs);
  const std::string dir = argv[2];
  same(src, "/rfs/msm/mpss/ticf.bin", hostFile(dir + "/ticf.bin"));
  same(src, "/rfs/msm/mpss/mot_rfs/imei_sv", hostFile(dir + "/imei_sv"));
  same(src, "/rfs/msm/mpss/datablock/id_00", hostFile(dir + "/id_00"));
  same(src, std::string("/rfs/msm/mpss/") + argv[3], hostFile(dir + "/mcfg"));
  uint32_t n = 0;
  CHECK(src.size("/rfs/msm/mpss/no_such_file", &n) == NotFound);
  CHECK(src.size("/rfs/msm/mpss/../sensors", &n) == Invalid);
  CHECK(src.size("/sensors/x", &n) == Invalid);                      // nothing outside the root
  CHECK(image.size("/rfs/msm/mpss/ticf.bin", &n) == Invalid);        // the default root stays /image/
  printf("PASS %u checks: persist rfs/msm/mpss read through the kernel ext4 reader, identical to debugfs\n", checks);
  return 0;
}
