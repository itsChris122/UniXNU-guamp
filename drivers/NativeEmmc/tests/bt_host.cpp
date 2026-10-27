// Host tests of the Bluetooth M0 pure code: H4/HCI framing and decoders (BtHci.h), the QCA version and firmware
// matching, the board helpers (GuampBtBoard.h: clock choice, RCG fallback, pins, rails) and the GENI UART engine
// (GeniUartCpp.inc) against a fake serial-engine register file.
#include <stdio.h>
#include <string.h>
#include <vector>
#include "../GuampBtBoard.h"
#include "../BtHci.h"
#include "../GeniUart.h"
#include "../GeniUartCpp.inc"

static int failures = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

// --- fake GENI SE ---------------------------------------------------------------------------------------------------
struct FakeSe {
  uint32_t regs[0x4000 / 4] = {};
  std::vector<uint8_t> tx;                 // bytes the engine "sent"
  std::vector<uint32_t> rxWords;           // RX FIFO content
  uint32_t rxStatusExtra = 0;              // RX_LAST / valid bits applied with the words
  bool stuck = false, sActive = false, cancelled = false, sCancelled = false;
  unsigned txLen = 0, txCmd = 0, delays = 0;
  uint32_t qupVersion = 0x20050000;
};
static uint32_t fakeRd(void *ctx, uint32_t reg) {
  FakeSe *f = static_cast<FakeSe *>(ctx);
  switch (reg) {
  case SE_GENI_RX_FIFOn: {
    if (f->rxWords.empty()) return 0xdeadbeef;
    uint32_t w = f->rxWords.front();
    f->rxWords.erase(f->rxWords.begin());
    return w;
  }
  case SE_GENI_RX_FIFO_STATUS: return f->rxWords.empty() ? 0 : (uint32_t)f->rxWords.size() | f->rxStatusExtra;
  case SE_GENI_TX_FIFO_STATUS: return 0;
  case SE_GENI_M_IRQ_STATUS: {
    uint32_t v = 0;
    if (f->txCmd && !f->stuck && f->tx.size() >= f->txLen) v |= GENI_M_CMD_DONE;
    if (f->cancelled) v |= GENI_M_CMD_CANCEL_DONE;
    return v;
  }
  case SE_GENI_S_IRQ_STATUS: return f->sCancelled ? GU_S_CMD_CANCEL : 0;
  case SE_GENI_STATUS: return f->sActive ? GU_STATUS_S_ACTIVE : 0;
  default: return f->regs[reg / 4];
  }
}
static void fakeWr(void *ctx, uint32_t reg, uint32_t v) {
  FakeSe *f = static_cast<FakeSe *>(ctx);
  f->regs[reg / 4] = v;
  if (reg == SE_GENI_TX_FIFOn) {
    for (unsigned j = 0; j < 4 && f->tx.size() < f->txLen; ++j) f->tx.push_back((uint8_t)(v >> (8 * j)));
  } else if (reg == SE_UART_TX_TRANS_LEN) {
    f->txLen = v; f->tx.clear();
  } else if (reg == SE_GENI_M_CMD0) {
    f->txCmd = v >> 27; f->cancelled = false;
  } else if (reg == SE_GENI_M_CMD_CTRL_REG && (v & GENI_M_CMD_CANCEL)) {
    f->cancelled = true;
  } else if (reg == SE_GENI_S_CMD0) {
    f->sActive = true; f->sCancelled = false;
  } else if (reg == SE_GENI_S_CMD_CTRL_REG && (v & GU_S_CMD_CANCEL_REQ)) {
    f->sActive = false; f->sCancelled = true;
  }
}
static uint32_t wrapRd(void *ctx, uint32_t reg) { return reg == QUPV3_HW_VER_REG ? static_cast<FakeSe *>(ctx)->qupVersion : 0; }
static void fakeDelay(void *ctx, unsigned) { static_cast<FakeSe *>(ctx)->delays++; }

static void setupUart(geni_uart &u, FakeSe &f, uint32_t proto, uint32_t ifDisable) {
  memset(&u, 0, sizeof u);
  f.regs[GENI_FW_REVISION_RO / 4] = proto << 8 | 0x10;
  f.regs[GENI_IF_DISABLE_RO / 4] = ifDisable;
  f.regs[SE_HW_PARAM_0 / 4] = 32u << 24 | 16u << 16;
  f.regs[SE_HW_PARAM_1 / 4] = 32u << 24 | 16u << 16;
  u.se = {&f, fakeRd, fakeWr, fakeDelay};
  u.wrapper = {&f, wrapRd, fakeWr, fakeDelay};
  u.poll_us = 10;
}

static void testUart() {
  uint32_t c0, c1;
  geni_uart_packing(8, 4, 0, &c0, &c1);
  CHECK(c0 == 0x4380e && c1 == 0xc3e0e);                         // qcom-geni-se values for 8 bpw x4, LSB first
  {
    FakeSe f; geni_uart u;
    setupUart(u, f, GENI_PROTO_SPI, 0);
    CHECK(geni_uart_init(&u) == GENI_ERR_PROTO);
    setupUart(u, f, GENI_PROTO_UART, 1);
    CHECK(geni_uart_init(&u) == GENI_ERR_NOTSUP);
  }
  FakeSe f; geni_uart u;
  setupUart(u, f, GENI_PROTO_UART, 0);
  CHECK(geni_uart_init(&u) == 0);
  CHECK(u.info.tx_fifo_depth == 16 && u.info.rx_fifo_depth == 16 && u.info.fifo_width_bits == 32);
  CHECK(f.regs[SE_GENI_TX_PACKING_CFG0 / 4] == 0x4380e && f.regs[SE_GENI_RX_PACKING_CFG1 / 4] == 0xc3e0e);
  CHECK(f.regs[SE_GENI_RX_WATERMARK_REG / 4] == 8 && f.regs[SE_GENI_RX_RFR_WATERMARK_REG / 4] == 14);
  CHECK(f.regs[SE_UART_RX_STALE_CNT / 4] == 160 && f.regs[SE_UART_TX_WORD_LEN / 4] == 8 && f.regs[SE_UART_RX_WORD_LEN / 4] == 8);
  CHECK(f.regs[SE_UART_TX_STOP_BIT_LEN / 4] == 0 && f.regs[SE_UART_TX_PARITY_CFG / 4] == 0 && f.regs[SE_UART_LOOPBACK_CFG / 4] == 0);
  CHECK(f.regs[SE_IRQ_EN / 4] == 0 && !(f.regs[SE_GENI_DMA_MODE_EN / 4] & 1));
  CHECK(f.regs[SE_UART_TX_TRANS_CFG / 4] & GU_TX_CTS_MASK);
  CHECK(f.regs[SE_UART_MANUAL_RFR / 4] == (GU_MANUAL_RFR_EN | GU_RFR_NOT_READY));
  geni_uart_flow(&u, 1);
  CHECK(!(f.regs[SE_UART_TX_TRANS_CFG / 4] & GU_TX_CTS_MASK) && f.regs[SE_UART_MANUAL_RFR / 4] == 0);
  CHECK(geni_uart_set_clock(&u, 4, 0) == 0 && f.regs[GENI_SER_M_CLK_CFG / 4] == 0x41 && f.regs[GENI_SER_S_CLK_CFG / 4] == 0x41);
  CHECK(geni_uart_set_clock(&u, 0, 0) != 0 && geni_uart_set_clock(&u, 0x1000, 0) != 0);
  // TX: H4 HCI_Reset and a 9-byte packet, LSB first in each FIFO word
  const uint8_t reset[4] = {0x01, 0x03, 0x0c, 0x00};
  CHECK(geni_uart_write(&u, reset, 4, 1000) == 0);
  CHECK(f.txLen == 4 && f.txCmd == GU_START_TX && f.tx == std::vector<uint8_t>(reset, reset + 4));
  const uint8_t nine[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  CHECK(geni_uart_write(&u, nine, 9, 1000) == 0 && f.tx == std::vector<uint8_t>(nine, nine + 9));
  // TX that never completes (CTS never asserted): bounded, cancelled
  f.stuck = true;
  unsigned before = f.delays;
  CHECK(geni_uart_write(&u, reset, 4, 500) == GENI_ERR_TIMEOUT);
  CHECK(f.cancelled && f.delays - before <= 500 / 10 + 2);
  f.stuck = false;
  // RX start / stop
  CHECK(geni_uart_rx_start(&u) == 0 && f.sActive && f.regs[SE_GENI_S_CMD0 / 4] == ((1u << 27) | 0x80));
  CHECK(geni_uart_rx_stop(&u, 1000) == 0 && !f.sActive);
  // RX: 7 bytes = one full word + a stale-flushed word with 3 valid bytes
  const uint8_t cc[7] = {0x04, 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00};
  f.rxWords = {0x01040e04u, 0x00000c03u};
  f.rxStatusExtra = GENI_RX_LAST | 3u << GENI_RX_LAST_VALID_SHIFT;
  uint8_t buf[16];
  size_t n = geni_uart_read(&u, buf, sizeof buf);
  CHECK(n == 7 && !memcmp(buf, cc, 7));
  CHECK(geni_uart_read(&u, buf, sizeof buf) == 0);
  // RX into a short buffer: the rest is counted as dropped, never written past cap
  f.rxWords = {0x11111111u, 0x22222222u};
  f.rxStatusExtra = 0;
  memset(buf, 0xee, sizeof buf);
  n = geni_uart_read(&u, buf, 5);
  CHECK(n == 5 && buf[4] == 0x22 && buf[5] == 0xee && u.rx_dropped == 3);
}

// --- HCI ------------------------------------------------------------------------------------------------------------
static bt_event ev(const uint8_t *pkt) { bt_event e; e.code = pkt[1]; e.len = pkt[2]; e.p = pkt + 3; return e; }

static void testHci() {
  uint8_t out[16];
  CHECK(bt_h4_cmd(out, sizeof out, BT_OP_RESET, nullptr, 0) == 4 && !memcmp(out, "\x01\x03\x0c\x00", 4));
  CHECK(bt_h4_cmd(out, sizeof out, BT_OP_READ_LOCAL_VERSION, nullptr, 0) == 4 && !memcmp(out, "\x01\x01\x10\x00", 4));
  const uint8_t sub = BT_QCA_EDL_VER_REQ;
  CHECK(bt_h4_cmd(out, sizeof out, BT_OP_QCA_EDL, &sub, 1) == 5 && !memcmp(out, "\x01\x00\xfc\x01\x19", 5));
  CHECK(bt_h4_cmd(out, 4, BT_OP_QCA_EDL, &sub, 1) == 0);

  // Stream: two garbage bytes (power-on noise), an ACL packet, Command Status, Command Complete of HCI_Reset, then the
  // start of a vendor event; fed in odd chunks.
  const uint8_t stream[] = {0x00, 0xfe, 0x02, 0x01, 0x20, 0x02, 0x00, 0xaa, 0xbb, 0x04, 0x0f, 0x04, 0x00, 0x01, 0x00, 0xfc,
                            0x04, 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00, 0x04, 0xff, 0x0e, 0x00};
  size_t pos = 0; unsigned skipped = 0; bt_event e; std::vector<bt_event> got;
  for (size_t have = 1; have <= sizeof stream; have += 3) {
    size_t lim = have < sizeof stream ? have : sizeof stream;
    while (bt_h4_next(stream, lim, &pos, &e, &skipped)) got.push_back(e);
  }
  while (bt_h4_next(stream, sizeof stream, &pos, &e, &skipped)) got.push_back(e);
  CHECK(got.size() == 2 && skipped == 2 && pos == 23);
  uint16_t op = 0; uint8_t st = 0xff, rlen = 0; const uint8_t *ret = nullptr;
  CHECK(got.size() == 2 && !bt_cmd_status(&got[0], &op, &st) && op == 0xfc00 && st == 0);
  CHECK(got.size() == 2 && !bt_cmd_complete(&got[1], &op, &ret, &rlen) && op == BT_OP_RESET && rlen == 1 && ret[0] == 0);
  CHECK(got.size() == 2 && bt_cmd_status(&got[1], &op, &st) != 0);

  // Read_Local_Version: HCI 5.0 rev 0, LMP 5.0, Qualcomm (0x001d), subversion 0x0102
  const uint8_t lv[] = {0x04, 0x0e, 0x0c, 0x01, 0x01, 0x10, 0x00, 0x09, 0x00, 0x00, 0x09, 0x1d, 0x00, 0x02, 0x01};
  bt_event le = ev(lv); bt_local_version v{};
  CHECK(bt_decode_local_version(&le, &v) == 0 && v.hci_ver == 9 && v.hci_rev == 0 && v.lmp_ver == 9 && v.manufacturer == 0x1d &&
        v.lmp_subver == 0x0102);
  CHECK(!strcmp(bt_hci_version_name(v.hci_ver), "5.0") && !strcmp(bt_manufacturer_name(v.manufacturer), "Qualcomm"));
  const uint8_t lvBad[] = {0x04, 0x0e, 0x04, 0x01, 0x01, 0x10, 0x0c};
  le = ev(lvBad);
  CHECK(bt_decode_local_version(&le, &v) == 1 && v.status == 0x0c);
  le = ev(stream + 16);                                            // CC of HCI_Reset is not a version reply
  CHECK(bt_decode_local_version(&le, &v) < 0);

  // QCA EDL version, vendor event (btqca.c layout): product 0x0f, patch 0x0002, ROM 0x0102, SoC 0x40070120
  const uint8_t qv1[] = {0x04, 0xff, 0x0e, 0x00, 0x02, 0x0f, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x01, 0x20, 0x01, 0x07, 0x40};
  bt_qca_version q{};
  bt_event qe = ev(qv1);
  CHECK(bt_decode_qca_version(&qe, &q) == 0 && q.form == 1 && q.product_id == 0x0f && q.patch_ver == 2 && q.rom_ver == 0x0102 &&
        q.soc_id == 0x40070120 && q.soc_ver == 0x01200102 && q.rom == 0x12);
  char tlv[16], nvm[16];
  bt_qca_names(q.rom, tlv, nvm);
  CHECK(!strcmp(tlv, "crbtfw12.tlv") && !strcmp(nvm, "crnv12.bin"));
  CHECK(bt_fw_matches(&q) == 1u << 2 && !strcmp(kBtFirmware[2].tlv, "cmbtfw12.tlv"));
  // with one leading byte (WCN3991-style) and Cherokee ROM 2.1 (product 0x0a): crbtfw21 and its twin apbtfw11
  const uint8_t qv2[] = {0x04, 0xff, 0x0f, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00, 0x02, 0x02, 0x01, 0x02, 0x14, 0x02, 0x01, 0x40};
  qe = ev(qv2);
  CHECK(bt_decode_qca_version(&qe, &q) == 0 && q.form == 2 && q.rom_ver == 0x0201 && q.rom == 0x21 && q.soc_id == 0x40010214);
  CHECK(bt_fw_matches(&q) == ((1u << 5) | (1u << 7)));
  bt_qca_names(q.rom, tlv, nvm);
  CHECK(!strcmp(tlv, "crbtfw21.tlv") && !strcmp(nvm, "crnv21.bin"));
  // wrapped in Command Complete of 0xFC00 with a status byte first
  const uint8_t qv3[] = {0x04, 0x0e, 0x12, 0x01, 0x00, 0xfc, 0x00, 0x00, 0x02, 0x0f, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01, 0x01,
                         0x10, 0x01, 0x07, 0x40};
  qe = ev(qv3);
  CHECK(bt_decode_qca_version(&qe, &q) == 0 && q.form == 3 && q.rom_ver == 0x0101 && q.rom == 0x11 && bt_fw_matches(&q) == 1u << 1);
  // wrong response type, short event, HCI_Reset CC: refused
  const uint8_t qbad[] = {0x04, 0xff, 0x0e, 0x00, 0x04, 0x0f, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x01, 0x20, 0x01, 0x07, 0x40};
  qe = ev(qbad);
  CHECK(bt_decode_qca_version(&qe, &q) < 0);
  const uint8_t qshort[] = {0x04, 0xff, 0x04, 0x00, 0x02, 0x0f, 0x00};
  qe = ev(qshort);
  CHECK(bt_decode_qca_version(&qe, &q) < 0);
  qe = ev(stream + 16);
  CHECK(bt_decode_qca_version(&qe, &q) < 0);

  char text[64];
  bt_hex(text, sizeof text, stream + 16, 7, 48);
  CHECK(!strcmp(text, "04 0e 04 01 03 0c 00"));
  bt_hex(text, sizeof text, stream, sizeof stream, 4);
  CHECK(!strcmp(text, "00 fe 02 01..."));
  bt_hex(text, 8, stream, sizeof stream, 48);                    // never past cap
  CHECK(strlen(text) < 8);
}

// --- board ----------------------------------------------------------------------------------------------------------
static void testBoard() {
  // DFS levels as gcc-bengal.c ftbl_gcc_qupv3_wrap0_s0_clk_src (first eight)
  const uint64_t lv[8] = {7372800, 14745600, 19200000, 29491200, 32000000, 48000000, 64000000, 75000000};
  bt_uart_clock c;
  CHECK(bt_choose_uart_clock(lv, 8, 115200, 16, &c) == 0 && c.level == 0 && c.div == 4 && c.baud_actual == 115200);
  CHECK(bt_choose_uart_clock(lv, 8, 115200, 32, &c) == 0 && c.level == 0 && c.div == 2 && c.baud_actual == 115200);
  CHECK(bt_choose_uart_clock(lv, 8, 2400, 16, &c) == 0 && c.level == 0 && c.div == 192 && c.baud_actual == 2400);
  CHECK(bt_choose_uart_clock(lv, 8, 3000000, 16, &c) == 0 && c.src_hz == 48000000 && c.div == 1 && c.baud_actual == 3000000);
  const uint64_t tcxo[8] = {19200000};
  CHECK(bt_choose_uart_clock(tcxo, 8, 115200, 16, &c) == 0 && c.div == 11 && bt_baud_error_permille(115200, c.baud_actual) == 53);
  CHECK(bt_baud_error_permille(115200, 115200) == 0 && bt_baud_error_permille(100, 105) == -50);
  const uint64_t none[8] = {};
  CHECK(bt_choose_uart_clock(none, 8, 115200, 16, &c) != 0);
  // RCG fallback programs the ftbl's 7.3728 MHz entry: decoded by the same helper the DFS levels use
  CHECK(gt_level_hz(BT_RCG_7M37_CFG, BT_RCG_7M37_M, bt_rcg_n_reg(BT_RCG_7M37_M, BT_RCG_7M37_N)) == 7372800);
  CHECK(bt_rcg_n_reg(384, 15625) == (~15241u & 0xffff) && bt_rcg_d_reg(384, 15625) == (~15625u & 0xffff));
  CHECK(BT_RCG_7M37_CFG == 0x2201);
  // pins: qup3 = function 1; RTS pull-down, TX pull-up, CTS/RX no pull, 2 mA
  CHECK(bt_pin_active_cfg(8) == 0x4 && bt_pin_active_cfg(9) == 0x5 && bt_pin_active_cfg(10) == 0x7 && bt_pin_active_cfg(11) == 0x4);
  CHECK(BT_PINS_BASE == 0xd08000ULL && BT_SWCTRL_BLOCK == 0xd57000ULL);
  // rails: LDO n at 0x4000 + 0x100 (n - 1)
  const unsigned ldo[4] = {9, 16, 17, 23};
  for (unsigned i = 0; i < 4; ++i) CHECK(kBtRails[i].base == 0x4000 + 0x100 * (ldo[i] - 1));
  CHECK(bt_ldo_mv(0x08, 0x07) == 1800 && bt_ldo_mv_ok(&kBtRails[0], 1800) && !bt_ldo_mv_ok(&kBtRails[2], 1800));
  CHECK(bt_ldo_mv_ok(&kBtRails[2], 1304) && bt_ldo_mv_ok(&kBtRails[3], 3312) && !bt_ldo_mv_ok(&kBtRails[3], 3300 + 13));
  CHECK(bt_ldo_is_on(0x80, 0x00) && bt_ldo_is_on(0x00, 0x83) && !bt_ldo_is_on(0x00, 0x03));
}

int main() {
  testUart();
  testHci();
  testBoard();
  printf("%s: %d checks, %d failures\n", failures ? "FAIL" : "PASS", checks, failures);
  return failures ? 1 : 0;
}
