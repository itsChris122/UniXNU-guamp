// Host tests of the guamp audio output HW code: GeniI2cCpp.inc against a model of the GENI serial engine with an
// aw882xx and an absent fsa4480 on the bus, and the register sequences of GuampAudioHwCore.h (aw882xx probe, cold
// start, start/stop, volume; LPI i2s1 pins; bolero fs-gen, RX macro mclk and headphone interpolators; wcd937x init
// and HPH) against simulated register files. Expected values are taken from the Linux drivers named in the headers.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include "../GeniI2cCpp.inc"
#include "../GuampAudioHwCore.h"

static int gFail, gChecks;
#define CHECK(c) do { ++gChecks; if (!(c)) { ++gFail; std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

// ---------------- aw882xx model ----------------
struct Aw {
  std::map<uint8_t, uint16_t> r;
  bool clocks = false;      // BCLK/WS present
  uint16_t id = AW_ID;
  uint8_t ptr = 0;
  std::vector<std::pair<uint8_t, uint16_t>> writes;
  void reset() { r.clear(); r[AW_REG_SYSCTRL] = 0x4443; r[AW_REG_SYSCTRL2] = 0x0038; r[AW_REG_HAGCCFG4] = 0x0064; r[AW_REG_EFRM1] = 0x0005; r[AW_REG_EFRH] = 0x03ff; }
  uint16_t get(uint8_t a) {
    if (a == AW_REG_ID) return id;
    if (a == AW_REG_SYSST) return (clocks && !(r[AW_REG_SYSCTRL] & AW_PWDN)) ? 0x0311 : 0x0000;
    return r.count(a) ? r[a] : 0;
  }
};

// ---------------- GENI SE model ----------------
struct Se {
  std::map<uint32_t, uint32_t> reg;
  uint32_t status = 0;
  bool active = false, isRead = false, stuck = false;
  uint8_t addr = 0;
  uint32_t len = 0;
  std::vector<uint8_t> tx;
  std::vector<uint32_t> rxq;
  Aw *aw = nullptr;
  bool deviceAt(uint8_t a) const { return a == GAH_AW_ADDR && aw; }
  void finishWrite() {
    if (!deviceAt(addr)) { status |= GENI_M_GP_IRQ_1 | GENI_M_CMD_DONE; active = false; return; }
    if (!tx.empty()) aw->ptr = tx[0];
    if (tx.size() >= 3) { uint16_t v = (uint16_t)(tx[1] << 8 | tx[2]); aw->r[tx[0]] = v; aw->writes.push_back({tx[0], v}); }
    status |= GENI_M_CMD_DONE;
    active = false;
  }
  void startRead() {
    if (!deviceAt(addr)) { status |= GENI_M_GP_IRQ_1 | GENI_M_CMD_DONE; active = false; return; }
    std::vector<uint8_t> b;
    uint16_t v = aw->get(aw->ptr);
    for (uint32_t i = 0; i < len; i++) b.push_back(i == 0 ? (uint8_t)(v >> 8) : i == 1 ? (uint8_t)v : 0);
    for (size_t i = 0; i < b.size(); i += 4) {
      uint32_t w = 0;
      for (size_t k = 0; k < 4 && i + k < b.size(); k++) w |= (uint32_t)b[i + k] << (8 * k);
      rxq.push_back(w);
    }
    status |= GENI_M_RX_FIFO_LAST | GENI_M_CMD_DONE;
    active = false;
  }
  uint32_t rd(uint32_t r) {
    switch (r) {
    case GENI_FW_REVISION_RO: return (GENI_PROTO_I2C << 8) | 0x10;
    case GENI_IF_DISABLE_RO: return 0;
    case SE_HW_PARAM_0: return (32u << 24) | (16u << 16);
    case SE_GENI_M_IRQ_STATUS: return status;
    case SE_GENI_RX_FIFO_STATUS: return (uint32_t)rxq.size();
    case SE_GENI_RX_FIFOn: { if (rxq.empty()) return 0; uint32_t w = rxq.front(); rxq.erase(rxq.begin()); return w; }
    default: return reg[r];
    }
  }
  void wr(uint32_t r, uint32_t v) {
    reg[r] = v;
    if (r == SE_GENI_M_IRQ_CLEAR) status &= ~v;
    if (r == SE_GENI_M_CMD_CTRL_REG && (v & GENI_M_CMD_CANCEL)) { status |= GENI_M_CMD_CANCEL_DONE; active = false; }
    if (r == SE_GENI_M_CMD0) {
      uint32_t op = v >> GENI_M_OPCODE_SHIFT;
      addr = (uint8_t)((v >> GENI_I2C_SLV_ADDR_SHIFT) & 0x7f);
      active = true;
      tx.clear();
      if (stuck) return;
      if (op == GENI_I2C_CMD_WRITE) { isRead = false; len = reg[SE_I2C_TX_TRANS_LEN]; if (!len) finishWrite(); }
      else { isRead = true; len = reg[SE_I2C_RX_TRANS_LEN]; startRead(); }
    }
    if (r == SE_GENI_TX_WATERMARK_REG) { if (v && active && !isRead) status |= GENI_M_TX_FIFO_WATERMARK; else status &= ~GENI_M_TX_FIFO_WATERMARK; }
    if (r == SE_GENI_TX_FIFOn && active && !isRead) {
      for (int k = 0; k < 4 && tx.size() < len; k++) tx.push_back((uint8_t)(v >> (8 * k)));
      if (tx.size() == len) finishWrite();
    }
  }
};
static uint32_t seRd(void *c, uint32_t r) { return static_cast<Se *>(c)->rd(r); }
static void seWr(void *c, uint32_t r, uint32_t v) { static_cast<Se *>(c)->wr(r, v); }
static uint32_t wrapRd(void *, uint32_t) { return 0x10020000u; }
static void noDelay(void *, unsigned) {}

// ---------------- ops over the models ----------------
struct Board {
  Se se;
  Aw aw;
  geni_i2c i2c{};
  std::map<uint32_t, uint32_t> lpi, cdc;
  std::map<uint16_t, uint8_t> wcd;
  std::vector<std::string> log;
  std::vector<std::pair<unsigned, int>> gpio;
  unsigned sleptMs = 0;
};
static Board *B;
static int w16(void *, uint8_t d, uint8_t r, uint16_t v) { uint8_t b[3] = {r, (uint8_t)(v >> 8), (uint8_t)v}; geni_i2c_msg m = {b, 3, 0}; return geni_i2c_xfer(&B->i2c, d, &m, 1); }
static int r16(void *, uint8_t d, uint8_t r, uint16_t *v) {
  uint8_t b[2] = {0, 0};
  geni_i2c_msg m[2] = {{&r, 1, 0}, {b, 2, 1}};
  int rc = geni_i2c_xfer(&B->i2c, d, m, 2);
  if (!rc) *v = (uint16_t)(b[0] << 8 | b[1]);
  return rc;
}
static void gpioOut(void *, unsigned g, int h) { B->gpio.push_back({g, h}); if (g == GAH_GPIO_AMP_RESET && h) B->aw.reset(); }
static uint32_t lpiRd(void *, uint32_t o) { return B->lpi[o]; }
static void lpiWr(void *, uint32_t o, uint32_t v) { B->lpi[o] = v; }
static uint32_t cdcRd(void *, uint32_t r) { return B->cdc[r]; }
static void cdcWr(void *, uint32_t r, uint32_t v) { B->cdc[r] = v; }
static int wcdRd(void *, uint16_t r, uint8_t *v) { *v = B->wcd[r]; return 0; }
static int wcdWr(void *, uint16_t r, uint8_t v) { B->wcd[r] = v; return 0; }
static void sleepMs(void *, unsigned ms) { B->sleptMs += ms; }
static void note(void *, const char *s, int rc, uint32_t a, uint32_t b) {
  char t[256];
  std::snprintf(t, sizeof t, "%s rc %d 0x%x 0x%x", s, rc, a, b);
  B->log.push_back(t);
}

static gah_ops makeOps(bool withWcd) {
  gah_ops o = {nullptr, w16, r16, gpioOut, lpiRd, lpiWr, cdcRd, cdcWr, withWcd ? wcdRd : nullptr, withWcd ? wcdWr : nullptr,
               sleepMs, noDelay, note};
  return o;
}
static void setup(Board &b) {
  B = &b;
  b.se.aw = &b.aw;
  b.i2c.se = {&b.se, seRd, seWr, noDelay};
  b.i2c.wrapper = {&b.se, wrapRd, seWr, noDelay};
}

static void testGeni() {
  Board b;
  setup(b);
  CHECK(geni_i2c_init(&b.i2c) == 0);
  CHECK(b.i2c.info.protocol == GENI_PROTO_I2C && b.i2c.info.tx_fifo_depth == 16 && b.i2c.info.fifo_width_bits == 32);
  CHECK(geni_i2c_set_clock(&b.i2c, 400, 3) == 0);
  CHECK(b.se.reg[GENI_SER_M_CLK_CFG] == ((2u << 4) | 1u));                 // geni_i2c_clk_map 400 kHz: div 2
  CHECK(b.se.reg[SE_I2C_SCL_COUNTERS] == ((7u << 20) | (10u << 10) | 24u));
  CHECK(b.se.reg[SE_GENI_CLK_SEL] == 3);
  CHECK(geni_i2c_set_clock(&b.i2c, 100, 0) == 0 && b.se.reg[GENI_SER_M_CLK_CFG] == ((7u << 4) | 1u));
  CHECK(geni_i2c_set_clock(&b.i2c, 250, 0) == GENI_ERR_PARAM);
  // packing: 8 bits, 4 per word, MSB first (Linux se_config_packing(base, 8, 4, true))
  CHECK(b.se.reg[SE_GENI_TX_PACKING_CFG0] == (((7u << 5) | 0x10 | 0xe) | (((15u << 5) | 0x10 | 0xe) << 10)));
  CHECK(b.se.reg[SE_GENI_TX_PACKING_CFG1] == (((23u << 5) | 0x10 | 0xe) | (((31u << 5) | 0x10 | 0xe | 1) << 10)));
  b.aw.reset();
  uint16_t v = 0;
  CHECK(w16(nullptr, GAH_AW_ADDR, 0x0c, 0xabcd) == 0 && b.aw.r[0x0c] == 0xabcd);
  CHECK(r16(nullptr, GAH_AW_ADDR, 0x0c, &v) == 0 && v == 0xabcd);
  CHECK(r16(nullptr, GAH_AW_ADDR, AW_REG_ID, &v) == 0 && v == 0x1852);
  uint8_t one = 0x55;
  geni_i2c_msg m[2] = {{&one, 1, 0}, {&one, 1, 1}};
  CHECK(geni_i2c_xfer(&b.i2c, GAH_FSA_ADDR, m, 2) == GENI_ERR_NACK);      // fsa4480 absent: address NACK
  CHECK(b.se.reg[SE_GENI_M_CMD0] >> GENI_M_OPCODE_SHIFT == GENI_I2C_CMD_WRITE);   // stopped after the first message
  // repeated start: the register-pointer write of a read carries STOP_STRETCH, the read does not
  CHECK(r16(nullptr, GAH_AW_ADDR, 0x05, &v) == 0 && (b.se.reg[SE_GENI_M_CMD0] & GENI_I2C_STOP_STRETCH) == 0);
  b.se.stuck = true;
  CHECK(w16(nullptr, GAH_AW_ADDR, 0x05, 1) == GENI_ERR_TIMEOUT && b.i2c.last_cancel == 1);
  b.se.stuck = false;
  CHECK(w16(nullptr, GAH_AW_ADDR, 0x05, 2) == 0);                           // usable again after the cancel
}

static void testHelpers() {
  CHECK(gah_aw_att_code(3000) == 0x50);   // Linux: (30/6)<<4 + 0
  CHECK(gah_aw_att_code(300) == 0x06);    // 3 dB: (3%6)*2
  CHECK(gah_aw_att_code(9000) == 0xf0);   // VOLUME_MIN_NEG_90_DB
  CHECK(gah_aw_att_code(9550) == 0xfb && gah_aw_att_code(20000) == 0xff);
  CHECK(gah_aw_att_code(0) == 0 && gah_aw_att_code(1) == 0x01);           // rounded towards quieter
  CHECK(gah_aw_att_cb(0x50) == 3000);
  CHECK(gah_att_from_level(0, 600) == 600 && gah_att_from_level(-3000, 600) == 3000 && gah_att_from_level(100, 0) == 0);
  CHECK(gah_aw_vcalb(0x0005, 0x03ff) == 8192 * 1005 / 999);
  CHECK(gah_aw_vcalb(0x0200, 0x0000) == (uint16_t)(8192 * 488 / 1000));  // -512 sign extension
  CHECK(gah_wcd_gain_code(-600) == 4 && gah_wcd_gain_code(-3000) == 20 && gah_wcd_gain_code(500) == 0);
  CHECK(gah_rxm_vol_code(-100) == (uint8_t)-84 && gah_rxm_vol_code(-6) == 0xfa && gah_rxm_vol_code(99) == 40);
}

static void testSpeaker() {
  Board b;
  setup(b);
  gah_ops o = makeOps(false);
  CHECK(geni_i2c_init(&b.i2c) == 0);
  uint16_t id = 0;
  CHECK(gah_aw_probe(&o, &id) == 0 && id == AW_ID);
  CHECK(b.gpio.size() == 2 && b.gpio[0] == std::make_pair(97u, 0) && b.gpio[1] == std::make_pair(97u, 1) && b.sleptMs == 3);
  b.aw.writes.clear();
  CHECK(gah_aw_cold(&o, 0x50) == 0);
  CHECK(b.aw.writes.size() == 37);                                          // 36 profile records + VCALB
  CHECK(b.aw.writes[0] == std::make_pair((uint8_t)0x03, (uint16_t)0xefbf));
  CHECK(b.aw.r[AW_REG_SYSCTRL] == 0x4441);                                  // profile 0x4440 + PWDN
  CHECK(b.aw.r[AW_REG_SYSCTRL2] == 0x0038);                                 // profile 0x0028 + HMUTE
  CHECK(b.aw.r[AW_REG_I2SCTRL] == 0x1408);
  CHECK(b.aw.r[AW_REG_HAGCCFG4] == 0x5064);
  CHECK(b.aw.r[AW_REG_VTMCTRL3] == 8192 * 1005 / 999);
  CHECK(b.aw.r[0x70] == 0xff00);                                             // last record
  // start without I2S clocks: SYSST never locks -> muted and powered down again, error
  uint16_t st = 0xffff;
  CHECK(gah_aw_start(&o, &st) == GAH_ERR_IO && st == 0);
  CHECK((b.aw.r[AW_REG_SYSCTRL] & AW_PWDN) && (b.aw.r[AW_REG_SYSCTRL2] & AW_HMUTE));
  CHECK(b.sleptMs >= 3 + 20);
  b.aw.clocks = true;
  CHECK(gah_aw_start(&o, &st) == 0 && (st & AW_SYSST_OK) == AW_SYSST_OK);
  CHECK(!(b.aw.r[AW_REG_SYSCTRL] & AW_PWDN) && !(b.aw.r[AW_REG_SYSCTRL2] & AW_HMUTE));
  CHECK(b.aw.r[AW_REG_SYSCTRL] == 0x4440 && b.aw.r[AW_REG_SYSCTRL2] == 0x0028);
  CHECK(gah_aw_set_att(&o, 0x23) == 0 && b.aw.r[AW_REG_HAGCCFG4] == 0x2364);
  CHECK(gah_aw_mute(&o, 1) == 0 && (b.aw.r[AW_REG_SYSCTRL2] & AW_HMUTE));
  CHECK(gah_aw_mute(&o, 0) == 0 && !(b.aw.r[AW_REG_SYSCTRL2] & AW_HMUTE));
  CHECK(gah_aw_stop(&o) == 0 && b.aw.r[AW_REG_SYSCTRL] == 0x4441 && b.aw.r[AW_REG_SYSCTRL2] == 0x0038);
  // a board with another chip at 0x34 (fs16xx EVT): -19 after 5 tries 5 ms apart
  b.aw.id = 0x0000;
  b.sleptMs = 0;
  CHECK(gah_aw_probe(&o, &id) == GAH_ERR_NODEV && id == 0 && b.sleptMs == 3 + 25);
  // no device at all: I2C NACK reported, no retry loop
  b.se.aw = nullptr;
  CHECK(gah_aw_probe(&o, &id) == GENI_ERR_NACK);
}

static void testLpi() {
  Board b;
  setup(b);
  gah_ops o = makeOps(false);
  b.lpi[0x6000] = 0x123;
  gah_lpi_i2s1(&o, 1);
  for (unsigned g = 6; g <= 9; g++) CHECK(b.lpi[0x1000 * g] == 0x2c8);
  CHECK(b.lpi.size() == 4);
  CHECK(b.log[0].find("0x6 0x123") != std::string::npos);
  gah_lpi_i2s1(&o, 0);
  for (unsigned g = 6; g <= 9; g++) CHECK(b.lpi[0x1000 * g] == 0x009);
}

static void testRxMacro() {
  Board b;
  setup(b);
  gah_ops o = makeOps(false);
  b.cdc[RXM_FS_CNT_CONTROL] = 0x02;
  gah_rxm_mclk(&o, 1);
  CHECK(b.cdc[RXM_MCLK_CONTROL] == 0x03 && b.cdc[RXM_FS_CNT_CONTROL] == 0x01);
  CHECK(b.cdc[0x1434] == 0x02 && b.cdc[0x14b4] == 0x02 && b.cdc[0x1540] == 0x02);   // rx_macro_reg_init SEC7
  CHECK(b.cdc[0x1410] == 0x02 && b.cdc[0x1490] == 0x02 && b.cdc[0x1510] == 0x02);   // CFG3
  gah_rxm_hph(&o, 1, -20);
  CHECK(b.cdc[0x1180] == 5 && b.cdc[0x1188] == 6);                          // RX0 -> INT0, RX1 -> INT1
  CHECK(b.cdc[0x1400] == 0x24 && b.cdc[0x1480] == 0x24);                    // 48 kHz, clock on, PGA unmuted
  CHECK(b.cdc[0x1408] == 0x11 && b.cdc[0x1488] == 0x11);                    // CLSH_DSM_OUT + path enable
  CHECK(b.cdc[0x1440] == 1 && b.cdc[0x14c0] == 1 && b.cdc[0x140c] == 3 && b.cdc[0x148c] == 3);
  CHECK(b.cdc[0x1404] == 0x40 && b.cdc[0x1484] == 0x40);
  CHECK(b.cdc[0x1414] == (uint8_t)-20 && b.cdc[0x1494] == (uint8_t)-20);
  CHECK(b.cdc[RXM_CLSH_CRC] == 1 && b.cdc[RXM_CLSH_K1_LSB] == 0xc0 && b.cdc[RXM_CLSH_HPH_V_PA] == 0x1c);
  gah_rxm_volume(&o, -30);
  CHECK(b.cdc[0x1414] == (uint8_t)-30 && b.cdc[0x1494] == (uint8_t)-30);
  gah_rxm_hph(&o, 0, 0);
  CHECK(b.cdc[0x1400] == 0x14 && b.cdc[0x1480] == 0x14);                    // muted, clock off, rate 48 kHz
  CHECK(b.cdc[0x1440] == 0 && b.cdc[0x140c] == 0 && b.cdc[RXM_CLSH_CRC] == 0 && b.cdc[0x1408] == 0x01);
  gah_rxm_mclk(&o, 0);
  CHECK(b.cdc[RXM_MCLK_CONTROL] == 0 && b.cdc[RXM_FS_CNT_CONTROL] == 0x02);
}

static void testWcd() {
  Board b;
  setup(b);
  gah_ops none = makeOps(false);
  CHECK(gah_wcd_hph(&none, 1, 4) == GAH_ERR_NOTSUP && b.gpio.empty());
  gah_ops o = makeOps(true);   // init_reg + reset now live in GuampMicCodec.inc / guamp_wcd937x_power (mic_capture_host)
  CHECK(gah_wcd_hph(&o, 1, gah_wcd_gain_code(-600)) == 0);
  CHECK(b.wcd[WCD_ANA_HPH] == 0x30 && b.wcd[WCD_HPH_L_EN] == 0x24 && b.wcd[WCD_HPH_R_EN] == 0x24);
  CHECK(b.wcd[WCD_DIG_ANA_CLK_CTL] == 0x03 && b.wcd[WCD_DIG_DIG_CLK_CTL] == 0x0b && b.wcd[WCD_ANA_RX_SUPPLIES] == 0x03);
  CHECK(b.wcd[WCD_DIG_PDM_WD_CTL0] == 0x13 && b.wcd[WCD_HPH_TIMER1] == 0x02);
  CHECK(gah_wcd_hph(&o, 0, 0) == 0);
  CHECK(b.wcd[WCD_ANA_HPH] == 0 && b.wcd[WCD_DIG_PDM_WD_CTL0] == 0 && b.wcd[WCD_ANA_RX_SUPPLIES] == 0);
}

int main() {
  testGeni();
  testHelpers();
  testSpeaker();
  testLpi();
  testRxMacro();
  testWcd();
  std::printf("audio_hw_host: %d checks, %d failures\n", gChecks, gFail);
  return gFail ? 1 : 0;
}
