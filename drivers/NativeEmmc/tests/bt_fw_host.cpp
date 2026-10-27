// Host tests of the Bluetooth M1 pure code: BtQcaFw.h (names, baud codes, TLV patch header, NVM edits, 243-byte
// segments, acks, BD address), the IBS-aware H4 parser, the non-DFS RCG choice (GuampBtBoard.h) and GuampExt4.inc on
// the bluetooth partition (extents) and on mke2fs -t ext2 images (block maps).
//   bt_fw_host unit
//   bt_fw_host image <bluetooth_a.img> <dir>   every /image file against <dir> (debugfs rdump), then the real
//                                              cmbtfw12.tlv / cmnv12.bin through the M1 path
//   bt_fw_host tree <fs.img> <dir> [blockmap]   every regular file of <dir> (recursively) read back; blockmap: at
//                                              least one inode must be block-mapped
// Prints no identifiers (no UUIDs, labels, serials or BD addresses); file names and sizes only.
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
#include "../GuampExt4.inc"   // the include guard lets BtService.inc and GuampWifiPrep.inc both include it
#include "../GuampBtBoard.h"
#include "../BtHci.h"
#include "../BtQcaFw.h"

static int failures = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { failures++; printf("FAIL %s:%d: %s", __FILE__, __LINE__, #c); printf(" " __VA_ARGS__); printf("\n"); } } while (0)

static std::vector<uint8_t> readHostFile(const std::string &path) {
  std::vector<uint8_t> v;
  FILE *f = fopen(path.c_str(), "rb");
  if (!f) return v;
  uint8_t buf[65536];
  size_t n;
  while ((n = fread(buf, 1, sizeof buf, f)) > 0) v.insert(v.end(), buf, buf + n);
  fclose(f);
  return v;
}

// --- file-backed ext4 device ----------------------------------------------------------------------------------------
struct Dev { int fd; uint64_t sectors; unsigned calls, maxCount; };
static bool devRead(void *ctx, uint64_t sector, uint32_t count, uint8_t *out) {
  Dev *d = static_cast<Dev *>(ctx);
  d->calls++;
  if (count > d->maxCount) d->maxCount = count;
  if (!count || count > ext4::kMaxSectors || sector + count > d->sectors) return false;
  return pread(d->fd, out, count * 512ull, static_cast<off_t>(sector * 512)) == static_cast<ssize_t>(count * 512ull);
}
struct Mounted {
  Dev dev{};
  std::vector<uint8_t> scratch = std::vector<uint8_t>(ext4::kScratchBytes);
  ext4::Fs fs{};
  int rc = -1;
  explicit Mounted(const char *path) {
    dev.fd = open(path, O_RDONLY);
    if (dev.fd < 0) return;
    struct stat st;
    fstat(dev.fd, &st);
    dev.sectors = static_cast<uint64_t>(st.st_size) / 512;
    rc = fs.mount(devRead, &dev, dev.sectors, scratch.data());
  }
  ~Mounted() { if (dev.fd >= 0) close(dev.fd); }
  int load(const std::string &path, std::vector<uint8_t> *out, ext4::Inode *ino) {
    int r = fs.lookupPath(path.c_str(), ino);
    if (r) return r;
    out->assign(ino->size, 0xa5);
    return ino->size ? fs.readFile(*ino, 0, out->data(), static_cast<uint32_t>(ino->size)) : 0;
  }
};

// --- unit tests -----------------------------------------------------------------------------------------------------
static bt_event ev(const uint8_t *pkt) { bt_event e; e.code = pkt[1]; e.len = pkt[2]; e.p = pkt + 3; return e; }

static void testUnit() {
  {  // guamp-bt-addr: parse (text -> HCI little-endian), masked log text, EDL_WRITE_BD_ADDR parameters (baswap)
    uint8_t a[6] = {}, p6[6] = {};
    char m[18];
    CHECK(bt_bd_addr_parse("00:11:22:Aa:bB:cc", a) == 0 && a[0] == 0xcc && a[1] == 0xbb && a[5] == 0x00, "");
    bt_bd_addr_masked(a, m);
    CHECK(!strcmp(m, "xx:xx:xx:xx:bb:cc"), "%s", m);
    bt_qca_bdaddr_params(a, p6);
    CHECK(p6[0] == 0x00 && p6[3] == 0xaa && p6[5] == 0xcc, "");
    CHECK(bt_bd_addr_parse("00:11:22:33:44", a) == -1 && bt_bd_addr_parse("00:11:22:33:44:5g", a) == -1, "");
    CHECK(bt_bd_addr_parse("00:11:22:33:44:55:", a) == -1 && bt_bd_addr_parse("00-11-22-33-44-55", a) == 0, "");
  }
  // enum qca_baudrate (btqca.h): 3000000 = 14, 3200000 = 17 (highest qca_set_baudrate accepts), 3500000 refused
  CHECK(bt_qca_baud_code(115200) == 0 && bt_qca_baud_code(230400) == 5 && bt_qca_baud_code(921600) == 10, "");
  CHECK(bt_qca_baud_code(3000000) == 0x0e && bt_qca_baud_code(3200000) == 0x11 && bt_qca_baud_code(2000000) == 13, "");
  CHECK(bt_qca_baud_code(3500000) == 0xff && bt_qca_baud_code(1234) == 0xff, "");
  uint8_t pkt[16];
  const uint8_t code = 0x0e;
  CHECK(bt_h4_cmd(pkt, sizeof pkt, BT_OP_QCA_SET_BAUD, &code, 1) == 5 && !memcmp(pkt, "\x01\x48\xfc\x01\x0e", 5), "");
  CHECK(bt_h4_cmd(pkt, sizeof pkt, BT_OP_READ_BD_ADDR, nullptr, 0) == 4 && !memcmp(pkt, "\x01\x09\x10\x00", 4), "");

  // names: the guamp controller (M0 log: product 0x0f, ROM 0x0102, SoC 0x40070120) -> WCN3950 cm files, no variant
  const uint8_t qv1[] = {0x04, 0xff, 0x0e, 0x00, 0x02, 0x0f, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x01, 0x20, 0x01, 0x07, 0x40};
  bt_event e = ev(qv1);
  bt_qca_version q{};
  CHECK(bt_decode_qca_version(&e, &q) == 0 && q.soc_ver == 0x01200102 && q.rom == 0x12, "");
  char tlv[20], nvm[20];
  CHECK(bt_qca_fw_names(&q, tlv, nvm) == 3950 && !strcmp(tlv, "cmbtfw12.tlv") && !strcmp(nvm, "cmnv12.bin"), "%s %s", tlv, nvm);
  q.soc_id = BT_QCA_SOC_WCN3950_T;
  bt_qca_fw_names(&q, tlv, nvm);
  CHECK(!strcmp(nvm, "cmnv12t.bin"), "%s", nvm);
  q.soc_id = BT_QCA_SOC_WCN3950_S;
  bt_qca_fw_names(&q, tlv, nvm);
  CHECK(!strcmp(nvm, "cmnv12s.bin"), "%s", nvm);
  q.product_id = 0x0a; q.soc_id = 0x40010214; q.rom = 0x21;
  CHECK(bt_qca_fw_names(&q, tlv, nvm) == 3990 && !strcmp(tlv, "crbtfw21.tlv") && !strcmp(nvm, "crnv21.bin"), "%s %s", tlv, nvm);
  q.soc_id = BT_QCA_SOC_WCN3991;
  bt_qca_fw_names(&q, tlv, nvm);
  CHECK(!strcmp(nvm, "crnv21u.bin"), "%s", nvm);

  // synthetic NVM: type 4 wrapper, inner type 2 with tags 2 (BD), 17 (HCI), 27 (sleep), 5 (other), then a second set
  std::vector<uint8_t> n;
  auto le32 = [&](std::vector<uint8_t> &v, uint32_t x) { for (int k = 0; k < 4; ++k) v.push_back(uint8_t(x >> (8 * k))); };
  auto tag = [&](std::vector<uint8_t> &v, uint16_t id, std::vector<uint8_t> d) {
    v.push_back(uint8_t(id)); v.push_back(uint8_t(id >> 8)); v.push_back(uint8_t(d.size())); v.push_back(uint8_t(d.size() >> 8));
    for (int k = 0; k < 8; ++k) v.push_back(0);
    v.insert(v.end(), d.begin(), d.end());
  };
  std::vector<uint8_t> set;
  tag(set, 2, {1, 2, 3, 4, 5, 6});
  tag(set, 17, {0x15, 0x11, 0x22, 0x00});
  tag(set, 27, {0x00, 0x01});
  tag(set, 5, {9, 9, 9});
  le32(n, 4u | uint32_t(set.size() + 4 + 8) << 8);
  le32(n, 2u | uint32_t(set.size()) << 8);
  n.insert(n.end(), set.begin(), set.end());
  le32(n, 2u | 0u << 8);
  const std::vector<uint8_t> orig = n;
  bt_tlv_nvm o{};
  CHECK(bt_tlv_nvm_fix(n.data(), n.size(), 0x0e, 0, &o) == 0 && o.type == 2 && o.tags == 4 && o.has_bdaddr && o.has_hci && o.has_sleep, "");
  CHECK(o.hci_before[0] == 0x15 && o.hci_after[0] == 0x95 && o.hci_after[1] == 0x11 && o.hci_after[2] == 0x0e && o.sleep_after == 1, "");
  CHECK(o.bdaddr[0] == 1 && o.bdaddr[5] == 6, "");
  unsigned diff = 0;
  for (size_t i = 0; i < n.size(); ++i) diff += n[i] != orig[i];
  CHECK(diff == 3, "%u bytes changed", diff);                      // 0x15->0x95, 0x22->0x0e, sleep 00->01
  std::vector<uint8_t> n2 = orig;
  CHECK(bt_tlv_nvm_fix(n2.data(), n2.size(), 0x0e, 1, &o) == 0 && o.hci_after[1] == 0x0e && o.hci_after[2] == 0x22, "");
  {  // guamp-bt-nvm-baud=3: both HCI bytes; =2 equals the Linux WCN3950 layout
    std::vector<uint8_t> n3 = n2, n4 = n2;
    bt_tlv_nvm o3 = {}, o4 = {};
    CHECK(bt_tlv_nvm_fix_fields(n3.data(), n3.size(), 0x11, 3u, &o3) == 0 && o3.hci_after[1] == 0x11 && o3.hci_after[2] == 0x11, "");
    CHECK(bt_tlv_nvm_fix_fields(n4.data(), n4.size(), 0x11, 2u, &o4) == 0 && o4.hci_after[2] == 0x11 &&
          o4.hci_after[1] == o4.hci_before[1], "");
  }
  {  // auto byte choice: the guamp NVM (15 11 22) gets no baud byte, a file with a rate code at data[2] keeps Linux's
    const uint8_t g[3] = {0x15, 0x11, 0x22}, r[3] = {0x15, 0x00, 0x0e}, u[3] = {0x15, 0x0e, 0x22};
    CHECK(bt_tlv_nvm_baud_fields_auto(g, 0) == 0u && bt_tlv_nvm_baud_fields_auto(r, 0) == 2u &&
          bt_tlv_nvm_baud_fields_auto(u, 1) == 1u && bt_tlv_nvm_baud_fields_auto(g, 1) == 1u, "");
    std::vector<uint8_t> n5 = n2;
    bt_tlv_nvm o5 = {};
    CHECK(bt_tlv_nvm_fix_fields(n5.data(), n5.size(), 0x0e, 0u, &o5) == 0 && o5.hci_after[1] == o5.hci_before[1] &&
          o5.hci_after[2] == o5.hci_before[2] && (o5.hci_after[0] & 0x80), "");
  }
  {  // the answer seen on the phone after writing 0x0e into data[2]: HCI_Reset Command Complete with bit 7 set, SLEEP_IND
    const uint8_t seen[] = {0x84, 0x8e, 0x84, 0x81, 0x83, 0x8c, 0x80, 0xfe}, fine[] = {0x04, 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00};
    CHECK(bt_seven_bit_event(seen, sizeof seen) && !bt_seven_bit_event(fine, sizeof fine) && !bt_seven_bit_event(seen, 5), "");
  }
  CHECK(bt_qca_baud_of_code(0x0e) == 3000000 && bt_qca_baud_of_code(0x11) == 3200000 && bt_qca_baud_of_code(0x00) == 115200 &&
        bt_qca_baud_of_code(0x22) == 0 && bt_qca_baud_of_code(bt_qca_baud_code(921600)) == 921600, "");
  std::vector<uint8_t> bad = orig;
  bad[8 + 12 + 6 + 2] = 0x40;                                      // HCI tag length runs past the set
  CHECK(bt_tlv_nvm_fix(bad.data(), bad.size(), 0, 0, &o) == -1, "");
  bad = orig;
  bad[8 + 2] = 5;                                                  // BD tag of 5 bytes
  CHECK(bt_tlv_nvm_fix(bad.data(), bad.size(), 0, 0, &o) == -1, "");
  CHECK(bt_tlv_nvm_fix(bad.data(), 20, 0, 0, &o) == -1, "");       // set longer than the file
  std::vector<uint8_t> t3 = {3, 0, 0, 0};
  CHECK(bt_tlv_nvm_fix(t3.data(), t3.size(), 0, 0, &o) == 0 && o.type == 3 && o.tags == 0, "");

  // segments: 243-byte pieces, mode 3 acks only the last; mode 0 and 2 ack every one
  std::vector<uint8_t> f(1000);
  for (size_t i = 0; i < f.size(); ++i) f[i] = uint8_t(i * 13 + 1);
  CHECK(bt_tlv_segments(1000) == 5 && bt_tlv_segments(243) == 1 && bt_tlv_segments(244) == 2 && bt_tlv_segments(0) == 0, "");
  uint8_t seg[BT_QCA_SEG_PKT_MAX];
  int ack = -1;
  CHECK(bt_tlv_segment(f.data(), f.size(), 0, 3, seg, sizeof seg, &ack) == 249 && ack == 0, "");
  CHECK(seg[0] == 1 && seg[1] == 0x00 && seg[2] == 0xfc && seg[3] == 245 && seg[4] == 0x1e && seg[5] == 243 && seg[6] == f[0], "");
  CHECK(bt_tlv_segment(f.data(), f.size(), 4, 3, seg, sizeof seg, &ack) == 6 + 28 && ack == 1 && seg[3] == 30 && seg[5] == 28, "");
  CHECK(bt_tlv_segment(f.data(), f.size(), 1, 0, seg, sizeof seg, &ack) == 249 && ack == 1, "");
  CHECK(bt_tlv_segment(f.data(), f.size(), 1, 2, seg, sizeof seg, &ack) == 249 && ack == 1, "");
  CHECK(bt_tlv_segment(f.data(), f.size(), 1, 1, seg, sizeof seg, &ack) == 249 && ack == 0, "");
  CHECK(bt_tlv_segment(f.data(), f.size(), 5, 3, seg, sizeof seg, &ack) == 0, "");
  CHECK(bt_tlv_segment(f.data(), f.size(), 0, 3, seg, 100, &ack) == 0, "");
  CHECK(bt_tlv_segment(f.data(), 486, 1, 3, seg, sizeof seg, &ack) == 249 && ack == 1, "");   // exact multiple: last acked

  // acks: vendor event {00 04 result} exactly; other sizes / types refused
  const uint8_t a0[] = {0x04, 0xff, 0x03, 0x00, 0x04, 0x00}, a1[] = {0x04, 0xff, 0x03, 0x00, 0x04, 0x07},
                a2[] = {0x04, 0xff, 0x04, 0x00, 0x04, 0x00, 0x00}, a3[] = {0x04, 0xff, 0x03, 0x00, 0x02, 0x00},
                a4[] = {0x04, 0x0e, 0x03, 0x01, 0x00, 0xfc};
  uint8_t res = 0xff;
  e = ev(a0); CHECK(bt_tlv_ack(&e, &res) == 0 && res == 0, "");
  e = ev(a1); CHECK(bt_tlv_ack(&e, &res) == 0 && res == 7, "");
  e = ev(a2); CHECK(bt_tlv_ack(&e, &res) == -1, "");
  e = ev(a3); CHECK(bt_tlv_ack(&e, &res) == -1, "");
  e = ev(a4); CHECK(bt_tlv_ack(&e, &res) == -1, "");

  // Read_BD_ADDR (synthetic address)
  const uint8_t bd[] = {0x04, 0x0e, 0x0a, 0x01, 0x09, 0x10, 0x00, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11};
  uint8_t addr[6], st = 0xff;
  char text[18];
  e = ev(bd);
  CHECK(bt_decode_bd_addr(&e, addr, &st) == 0 && st == 0 && addr[0] == 0x66 && addr[5] == 0x11, "");
  bt_bd_addr_text(addr, text);
  CHECK(!strcmp(text, "11:22:33:44:55:66") && !bt_bd_addr_zero(addr), "%s", text);
  const uint8_t bdz[] = {0x04, 0x0e, 0x0a, 0x01, 0x09, 0x10, 0x00, 0, 0, 0, 0, 0, 0}, bde[] = {0x04, 0x0e, 0x04, 0x01, 0x09, 0x10, 0x0c};
  e = ev(bdz); CHECK(bt_decode_bd_addr(&e, addr, &st) == 0 && bt_bd_addr_zero(addr), "");
  e = ev(bde); CHECK(bt_decode_bd_addr(&e, addr, &st) == 1 && st == 0x0c, "");
  const uint8_t x1[6] = {1, 2, 3, 4, 5, 6}, x2[6] = {1, 2, 3, 4, 5, 7};
  CHECK(bt_bd_addr_eq(x1, x1) && !bt_bd_addr_eq(x1, x2), "");

  // IBS: sleep / wake bytes between packets are skipped, WAKE_INDs counted; a 0xfd inside an event is data
  const uint8_t s[] = {0xfe, 0xfd, 0x04, 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00, 0xfd, 0x04, 0xff, 0x03, 0xfd, 0x04, 0x00};
  size_t pos = 0;
  unsigned skipped = 0, wake = 0, got = 0;
  while (bt_h4_next_ibs(s, sizeof s, &pos, &e, &skipped, &wake)) got++;
  CHECK(got == 2 && skipped == 3 && wake == 2 && pos == sizeof s, "got %u skipped %u wake %u", got, skipped, wake);
  {  // the extended counters: SLEEP_IND and WAKE_ACK bytes between packets
    const uint8_t s2[] = {0xfe, 0xfc, 0xfc, 0x04, 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00, 0xfe};
    size_t p2 = 0;
    unsigned sk2 = 0, w2 = 0, a2 = 0, sl2 = 0, g2 = 0;
    while (bt_h4_next_ibs_ex(s2, sizeof s2, &p2, &e, &sk2, &w2, &a2, &sl2)) g2++;
    CHECK(g2 == 1 && sk2 == 4 && w2 == 0 && a2 == 2 && sl2 == 2 && p2 == sizeof s2, "got %u skipped %u ack %u sleep %u", g2, sk2, a2, sl2);
  }

  // baud math without DFS: the RCG entry with an exact divider
  bt_uart_clock c;
  CHECK(bt_choose_rcg(3000000, 16, &c) == 0 && c.src_hz == 48000000 && c.div == 1 && c.baud_actual == 3000000 &&
        kBtRcgTable[c.level].m == 4 && kBtRcgTable[c.level].n == 25, "");
  CHECK(bt_choose_rcg(3000000, 32, &c) == 0 && c.src_hz == 96000000 && c.div == 1 && c.baud_actual == 3000000, "");
  CHECK(bt_choose_rcg(115200, 16, &c) == 0 && c.level == 0 && c.div == 4 && c.baud_actual == 115200, "");
  CHECK(bt_choose_rcg(2400, 16, &c) == 0 && c.level == 0 && c.div == 192, "");
  CHECK(bt_choose_rcg(3200000, 16, &c) == 0 && c.baud_actual == 3200000 && c.src_hz == 102400000 && c.div == 2, "");
  CHECK(bt_choose_rcg(921600, 16, &c) == 0 && c.baud_actual == 921600 && c.src_hz == 14745600 && c.div == 1, "");
  for (const bt_rcg_entry &r : kBtRcgTable)                        // every entry decodes back to its own rate
    CHECK(gt_level_hz(BT_RCG_7M37_CFG, r.m, bt_rcg_n_reg(r.m, r.n)) == r.hz && BT_RCG_AUX2_HZ * r.m / r.n == r.hz, "%u", r.hz);
  CHECK(bt_rcg_n_reg(4, 25) == 0xffea && bt_rcg_d_reg(4, 25) == 0xffe6, "");
  // at 48 MHz 115200 would be 3.5 % off: the chooser keeps 7.3728 MHz for it
  const uint64_t only48[1] = {48000000};
  CHECK(bt_choose_uart_clock(only48, 1, 115200, 16, &c) == 0 && bt_baud_error_permille(115200, c.baud_actual) > 25, "");
}

// --- the real bluetooth_a image -------------------------------------------------------------------------------------
static void testImage(const char *img, const char *dir) {
  Mounted m(img);
  CHECK(m.rc == 0, "mount: %s", ext4::errorName(m.rc));
  if (m.rc) return;
  printf("image: %u-byte blocks, %llu blocks, incompat %x\n", m.fs.blockSize, (unsigned long long)m.fs.blocksCount, m.fs.incompat);
  ext4::Inode d;
  CHECK(m.fs.lookupPath("/image", &d) == 0 && d.isDir(), "/image");
  std::vector<std::string> names;
  m.fs.forEachEntry(d, [&](const char *nm, uint32_t len, uint32_t, uint8_t) {
    std::string s(nm, len);
    if (s != "." && s != "..") names.push_back(s);
    return false;
  });
  unsigned same = 0, extents = 0;
  for (const std::string &nm : names) {
    std::vector<uint8_t> got;
    ext4::Inode ino;
    int r = m.load("/image/" + nm, &got, &ino);
    std::vector<uint8_t> want = readHostFile(std::string(dir) + "/" + nm);
    CHECK(!r && got == want, "%s: %s, %zu vs %zu bytes", nm.c_str(), ext4::errorName(r), got.size(), want.size());
    if (!r && got == want) same++;
    if (ino.flags & ext4::kFlagExtents) extents++;
  }
  printf("image: %u/%zu files of /image identical to debugfs (%u with extents, %u block-mapped)\n", same, names.size(), extents,
         static_cast<unsigned>(names.size()) - extents);
  CHECK(names.size() >= 2 && same == names.size(), "");

  // the M1 path for the guamp controller
  const uint8_t qv1[] = {0x04, 0xff, 0x0e, 0x00, 0x02, 0x0f, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x01, 0x20, 0x01, 0x07, 0x40};
  bt_event e = ev(qv1);
  bt_qca_version q{};
  bt_decode_qca_version(&e, &q);
  char tlv[20], nvm[20];
  bt_qca_fw_names(&q, tlv, nvm);
  std::vector<uint8_t> patch, cfg;
  ext4::Inode ino;
  CHECK(m.load(std::string("/image/") + tlv, &patch, &ino) == 0, "%s", tlv);
  CHECK(m.load(std::string("/image/") + nvm, &cfg, &ino) == 0, "%s", nvm);
  if (patch.empty() || cfg.empty()) return;
  bt_tlv_patch ph{};
  CHECK(bt_tlv_patch_parse(patch.data(), patch.size(), &ph) == 0 && ph.product == 0x000f && ph.rom_build == 0x0102, "");
  CHECK(ph.product == (q.product_id & 0xffff) && ph.rom_build == q.rom_ver, "header matches the controller");
  CHECK((patch[1] | patch[2] << 8 | patch[3] << 16) + 4u == patch.size(), "TLV length = file size - 4");
  // segments: reassembled they give the file back; count the acked ones
  for (int pass = 0; pass < 2; ++pass) {
    std::vector<uint8_t> &f = pass ? cfg : patch;
    const unsigned mode = pass ? BT_QCA_SKIP_NONE : ph.mode;
    std::vector<uint8_t> back;
    unsigned segs = bt_tlv_segments(f.size()), acks = 0, full = 0;
    uint8_t seg[BT_QCA_SEG_PKT_MAX];
    for (unsigned i = 0; i < segs; ++i) {
      int ack = 0;
      size_t n = bt_tlv_segment(f.data(), f.size(), i, mode, seg, sizeof seg, &ack);
      CHECK(n == size_t(seg[3]) + 4 && seg[4] == 0x1e && seg[5] + 6u == n && seg[1] == 0 && seg[2] == 0xfc, "segment %u", i);
      back.insert(back.end(), seg + 6, seg + n);
      acks += ack;
      full += seg[5] == 243;
    }
    CHECK(back == f, "reassembly");
    printf("%s: %zu bytes, TLV mode %u: %u segments (%u of 243 bytes, last %zu), %u acked\n", pass ? nvm : tlv, f.size(), mode,
           segs, full, f.size() - full * 243u, acks);
    if (!pass) CHECK(ph.mode == 3 && segs == 197 && acks == 1 && f.size() == 47767, "patch");
    else CHECK(segs == 11 && acks == 11 && f.size() == 2590, "nvm");
  }
  printf("%s: type %u, patch %04x, product %04x, ROM build %04x, format %u, signature %u, download mode %u\n", tlv, ph.type,
         ph.patch_ver, ph.product, ph.rom_build, ph.format, ph.sig, ph.mode);
  // NVM edits for 3 Mbit/s (WCN3950: baud code in data[2])
  const std::vector<uint8_t> cfgOrig = cfg;
  bt_tlv_nvm o{};
  CHECK(bt_tlv_nvm_fix(cfg.data(), cfg.size(), bt_qca_baud_code(3000000), 0, &o) == 0, "nvm fix");
  printf("%s: type %u, %u tags; HCI tag %02x %02x %02x -> %02x %02x %02x; deep sleep %02x -> %02x; BD tag %s\n", nvm, o.type, o.tags,
         o.hci_before[0], o.hci_before[1], o.hci_before[2], o.hci_after[0], o.hci_after[1], o.hci_after[2], o.sleep_before,
         o.sleep_after, o.has_bdaddr ? (bt_bd_addr_zero(o.bdaddr) ? "present, zero" : "present, non-zero") : "absent");
  CHECK(o.type == 2 && o.has_hci && o.has_sleep && o.has_bdaddr && o.hci_after[0] == (o.hci_before[0] | 0x80) &&
        o.hci_after[2] == 0x0e && o.hci_after[1] == o.hci_before[1] && o.sleep_after == (o.sleep_before | 1), "nvm edits");
  unsigned diff = 0;
  for (size_t i = 0; i < cfg.size(); ++i) diff += cfg[i] != cfgOrig[i];
  CHECK(diff <= 3, "%u bytes changed", diff);
  printf("%s: %u bytes changed by the Linux NVM edits\n", nvm, diff);
}

// --- mke2fs images --------------------------------------------------------------------------------------------------
static void walk(const std::string &root, const std::string &rel, std::vector<std::string> *out) {
  DIR *d = opendir((root + rel).c_str());
  if (!d) return;
  while (dirent *e = readdir(d)) {
    std::string n = e->d_name;
    if (n == "." || n == "..") continue;
    struct stat st;
    if (lstat((root + rel + "/" + n).c_str(), &st)) continue;
    if (S_ISDIR(st.st_mode)) walk(root, rel + "/" + n, out);
    else if (S_ISREG(st.st_mode)) out->push_back(rel + "/" + n);
  }
  closedir(d);
}
static void testTree(const char *img, const char *dir, bool wantBlockMap) {
  Mounted m(img);
  CHECK(m.rc == 0, "mount: %s", ext4::errorName(m.rc));
  if (m.rc) return;
  std::vector<std::string> files;
  walk(dir, "", &files);
  unsigned same = 0, mapped = 0;
  uint64_t bytes = 0;
  for (const std::string &f : files) {
    std::vector<uint8_t> got;
    ext4::Inode ino;
    int r = m.load(f, &got, &ino);
    std::vector<uint8_t> want = readHostFile(std::string(dir) + f);
    CHECK(!r && got == want, "%s: %s", f.c_str(), ext4::errorName(r));
    if (!r && got == want) { same++; bytes += got.size(); }
    if (!r && !(ino.flags & ext4::kFlagExtents)) mapped++;
    // an odd window in the middle of the file (crosses block and indirect boundaries)
    if (!r && ino.size > 5000) {
      std::vector<uint8_t> part(3001);
      uint64_t off = ino.size / 2 - 1500 + 7;
      CHECK(m.fs.readFile(ino, off, part.data(), 3001) == 0 && !memcmp(part.data(), want.data() + off, 3001), "%s window", f.c_str());
    }
  }
  printf("tree: %u-byte blocks, %u/%zu files identical (%llu bytes), %u block-mapped, max %u sectors per read\n", m.fs.blockSize,
         same, files.size(), (unsigned long long)bytes, mapped, m.dev.maxCount);
  CHECK(same == files.size() && !files.empty(), "");
  if (wantBlockMap) CHECK(mapped == files.size(), "every file block-mapped on ext2");
  ext4::Inode root;
  CHECK(m.fs.readInode(ext4::kRootInode, &root) == 0 && (!wantBlockMap || !(root.flags & ext4::kFlagExtents)), "root directory");
}

int main(int argc, char **argv) {
  if (argc >= 2 && !strcmp(argv[1], "unit")) testUnit();
  else if (argc >= 4 && !strcmp(argv[1], "image")) testImage(argv[2], argv[3]);
  else if (argc >= 4 && !strcmp(argv[1], "tree")) testTree(argv[2], argv[3], argc >= 5 && !strcmp(argv[4], "blockmap"));
  else { printf("usage: bt_fw_host unit | image <img> <dir> | tree <img> <dir> [blockmap]\n"); return 2; }
  printf("%s %s: %d checks, %d failed\n", failures ? "FAIL" : "PASS", argv[1], checks, failures);
  return failures ? 1 : 0;
}
