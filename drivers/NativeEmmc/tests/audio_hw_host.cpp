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
  std::vector<std::pair<uint16_t, uint8_t>> wcdWrites;   // order of the wcd937x writes
  std::vector<std::pair<int, uint8_t>> swr;               // swr_rx calls (on, slave ch mask)
  size_t swrAt[2] = {0, 0};                               // wcdWrites.size() at the last swr_rx on / off
  int swrRc = 0;
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
static int wcdWr(void *, uint16_t r, uint8_t v) {
  B->wcd[r] = v;
  B->wcdWrites.push_back({r, v});
  if (r == WCD_DIG_INTR_CLEAR_0) B->wcd[WCD_DIG_INTR_STATUS_0] &= (uint8_t)~v;   // clear_ack
  return 0;
}
static int swrRx(void *, int on, uint8_t ch) {
  B->swr.push_back({on, ch});
  B->swrAt[on ? 1 : 0] = B->wcdWrites.size();
  return B->swrRc;
}
// index of the first write of reg (with value v when given) after position from, or SIZE_MAX
static size_t wIdx(uint16_t reg, int v = -1, size_t from = 0) {
  for (size_t i = from; i < B->wcdWrites.size(); i++)
    if (B->wcdWrites[i].first == reg && (v < 0 || B->wcdWrites[i].second == v)) return i;
  return (size_t)-1;
}
static void sleepMs(void *, unsigned ms) { B->sleptMs += ms; }
static void note(void *, const char *s, int rc, uint32_t a, uint32_t b) {
  char t[256];
  std::snprintf(t, sizeof t, "%s rc %d 0x%x 0x%x", s, rc, a, b);
  B->log.push_back(t);
}

static gah_ops makeOps(bool withWcd) {
  gah_ops o = {nullptr, w16, r16, gpioOut, lpiRd, lpiWr, cdcRd, cdcWr, withWcd ? wcdRd : nullptr, withWcd ? wcdWr : nullptr,
               sleepMs, noDelay, note, withWcd ? swrRx : nullptr};
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
  CHECK(gah_wcd_gain_code(-1200) == 8);                                       // guamp-audio-hw-hph default -12 dB
  CHECK(gah_wcd_ear_gain(600) == 0 && gah_wcd_ear_gain(0) == 4 && gah_wcd_ear_gain(-1200) == 12);   // G_6 / G_0 / G_M12
  CHECK(gah_wcd_ear_gain(-1800) == 16 && gah_wcd_ear_gain(-9000) == 16 && gah_wcd_ear_gain(900) == 0);
  CHECK(gah_wcd_ear_gain(-100) == 5);                                         // between steps: the quieter one
  CHECK(gah_mbhc_micb_mv(0x20) == 2600 && gah_mbhc_micb_mv(0x50) == 1800);    // MICB2 VOUT (enable bits ignored)
  CHECK(gah_mbhc_adc_mv(0xff, 1800) == 3187 && gah_mbhc_adc_mv(0x40, 2600) == 1155);
  CHECK(gah_mbhc_plug_from_mv(50, 2600) == GAH_JACK_HEADPHONE && gah_mbhc_plug_from_mv(1155, 2600) == GAH_JACK_HEADSET);
  CHECK(gah_mbhc_plug_from_mv(2500, 2600) == GAH_JACK_HIGH_HPH && gah_mbhc_plug_from_mv(107, 2600) == GAH_JACK_HEADPHONE);
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
  // headphones: INT0 + INT1, muted until the wcd937x PA is up
  gah_rxm_path(&o, 1, 0, -20);
  CHECK(b.cdc[0x1180] == 5 && b.cdc[0x1188] == 6);                          // RX0 -> INT0, RX1 -> INT1
  CHECK(b.cdc[0x1400] == 0x34 && b.cdc[0x1480] == 0x34);                    // 48 kHz, clock on, PGA still muted
  CHECK(b.cdc[0x1408] == 0x11 && b.cdc[0x1488] == 0x11);                    // CLSH_DSM_OUT + path enable, no ear bit
  CHECK(b.cdc[0x1440] == 1 && b.cdc[0x14c0] == 1 && b.cdc[0x140c] == 3 && b.cdc[0x148c] == 3);
  CHECK(b.cdc[0x1404] == 0x40 && b.cdc[0x1484] == 0x40);
  CHECK(b.cdc[RXM_TOP_HPHL_COMP_LUT] == 0x80 && b.cdc[RXM_TOP_HPHR_COMP_LUT] == 0x80);   // LUT bypass
  CHECK(b.cdc[0x1414] == (uint8_t)-20 && b.cdc[0x1494] == (uint8_t)-20);
  CHECK(b.cdc[RXM_CLSH_CRC] == 1 && b.cdc[RXM_CLSH_K1_LSB] == 0xc0 && b.cdc[RXM_CLSH_HPH_V_PA] == 0x1c);
  gah_rxm_mute(&o, 0, 0);
  CHECK(b.cdc[0x1400] == 0x24 && b.cdc[0x1480] == 0x24 && b.cdc[0x1418] == 0 && b.cdc[0x1498] == 0);
  gah_rxm_volume(&o, 0, -30);
  CHECK(b.cdc[0x1414] == (uint8_t)-30 && b.cdc[0x1494] == (uint8_t)-30);
  gah_rxm_mute(&o, 0, 1);
  CHECK(b.cdc[0x1400] == 0x34 && b.cdc[0x1418] == 0x10 && b.cdc[0x1498] == 0x10);
  gah_rxm_path(&o, 0, 0, 0);
  CHECK(b.cdc[0x1400] == 0x14 && b.cdc[0x1480] == 0x14);                    // muted, clock off, rate 48 kHz
  CHECK(b.cdc[0x1440] == 0 && b.cdc[0x140c] == 0 && b.cdc[RXM_CLSH_CRC] == 0 && b.cdc[0x1408] == 0x01);
  CHECK(b.cdc[RXM_TOP_HPHL_COMP_LUT] == 0 && b.cdc[RXM_COMP0_CTL7] == 0x20 && b.cdc[RXM_COMP1_CTL7] == 0x20);
  // earpiece: INT0 only, "RX_EAR Mode" ON: CFG1 bit 1 instead of the LUT bypass, class-H V_PA 0x39
  b.cdc.clear();
  gah_rxm_path(&o, 1, 1, -30);
  CHECK(b.cdc[0x1180] == 5 && b.cdc.count(0x1188) == 0 && b.cdc.count(0x1480) == 0);   // INT1 untouched
  CHECK(b.cdc[0x1400] == 0x34 && b.cdc[0x1408] == 0x13 && b.cdc[RXM_CLSH_HPH_V_PA] == 0x39);
  CHECK(b.cdc[RXM_TOP_HPHL_COMP_LUT] == 0 && b.cdc[0x1414] == (uint8_t)-30);
  gah_rxm_mute(&o, 1, 0);
  CHECK(b.cdc[0x1400] == 0x24 && b.cdc.count(0x1480) == 0);
  gah_rxm_volume(&o, 1, -40);
  CHECK(b.cdc[0x1414] == (uint8_t)-40 && b.cdc.count(0x1494) == 0);
  gah_rxm_path(&o, 0, 1, 0);
  CHECK(b.cdc[0x1400] == 0x14 && b.cdc[0x1408] == 0x01 && b.cdc[RXM_CLSH_CRC] == 0);   // ear bit cleared too
  gah_rxm_mclk(&o, 0);
  CHECK(b.cdc[RXM_MCLK_CONTROL] == 0 && b.cdc[RXM_FS_CNT_CONTROL] == 0x02);
}

// wcd937x RX power-up / power-down of the headphone and earpiece paths (CLS_AB, no compander) against a register file:
// final values, the DAPM order (port + datapath before the class-H FSM and the PA bits, PA bits last), and the undo
// when the RX SoundWire datapath fails.
static void testWcd() {
  Board b;
  setup(b);
  gah_ops none = makeOps(false);
  CHECK(gah_wcd_hph(&none, 1, 4) == GAH_ERR_NOTSUP && gah_wcd_ear(&none, 1, 4) == GAH_ERR_NOTSUP && b.gpio.empty());
  gah_ops o = makeOps(true);   // init_reg + reset live in GuampMicCodec.inc / guamp_wcd937x_power (mic_capture_host)
  b.wcd[WCD_ANA_RX_SUPPLIES] = 0x0e;   // stray mode bits the FSM must normalise
  b.wcd[WCD_FLYBACK_VNEG_CTRL_4] = 0xa0;
  CHECK(gah_wcd_hph(&o, 1, gah_wcd_gain_code(-1200)) == 0);
  CHECK(b.swr.size() == 1 && b.swr[0] == std::make_pair(1, (uint8_t)0x03));        // HPH_L + HPH_R
  CHECK(b.wcd[WCD_ANA_HPH] == 0xfc);                       // PA L+R (7/6), REF L+R (5/4), CLS_AB hph mode 0x0c
  CHECK(b.wcd[WCD_HPH_L_EN] == 0x28 && b.wcd[WCD_HPH_R_EN] == 0x28);   // no compander + gain code 8 (-12 dB)
  CHECK(b.wcd[WCD_ANA_RX_SUPPLIES] == 0xc3);               // buck + flyback on, RX bias, CLS_AB bit 1; no HIFI bits
  CHECK(b.wcd[WCD_DIG_ANA_CLK_CTL] == 0x03 && b.wcd[WCD_DIG_DIG_CLK_CTL] == 0x0b && b.wcd[WCD_DIG_HPH_GAIN_CTL] == 0x0c);
  CHECK(b.wcd[WCD_DIG_PDM_WD_CTL0] == 0x13 && b.wcd[WCD_DIG_PDM_WD_CTL1] == 0x13 && b.wcd[WCD_HPH_TIMER1] == 0x02);
  CHECK(b.wcd[WCD_CLASSH_MODE_2] == 0x3a && b.wcd[WCD_CLASSH_MODE_3] == 0x02 && b.wcd[WCD_FLYBACK_VNEG_CTRL_4] == 0x70);
  CHECK(b.wcd[WCD_FLYBACK_VNEGDAC_CTRL_2] == 0xe0 && b.wcd[WCD_RX_BIAS_FLYB_BUFF] == 0xaa);
  CHECK(b.wcd[WCD_DIG_COMP_CTL_0] == 0);
  // order: datapath before the class-H FSM, the FSM before the PA references, the PA power bits after both PDM WDs
  CHECK(b.swrAt[1] <= wIdx(WCD_FLYBACK_VNEG_CTRL_4) && wIdx(WCD_FLYBACK_VNEG_CTRL_4) < wIdx(WCD_ANA_HPH, 0x2c));
  CHECK(wIdx(WCD_DIG_PDM_WD_CTL1, 0x13) < wIdx(WCD_ANA_HPH, 0xfc) && wIdx(WCD_HPH_L_EN) < b.swrAt[1]);
  CHECK(wIdx(WCD_ANA_RX_SUPPLIES, 0xc3) > wIdx(WCD_ANA_HPH, 0xfc));                    // CLS_AB bit after the PA
  CHECK(b.sleptMs >= 40);                                                                // 20 ms per PA, no compander
  b.wcdWrites.clear();
  CHECK(gah_wcd_hph(&o, 0, 0) == 0);
  CHECK(b.swr.size() == 2 && b.swr[1] == std::make_pair(0, (uint8_t)0x03));
  CHECK(b.wcd[WCD_ANA_HPH] == 0 && b.wcd[WCD_DIG_PDM_WD_CTL0] == 0 && b.wcd[WCD_DIG_PDM_WD_CTL1] == 0);
  CHECK(b.wcd[WCD_ANA_RX_SUPPLIES] == 0x02 && b.wcd[WCD_DIG_ANA_CLK_CTL] == 0 && b.wcd[WCD_DIG_DIG_CLK_CTL] == 0x08);
  CHECK(b.wcd[WCD_RX_BIAS_HPH_LOWPOWER] == 0x80 && b.wcd[WCD_HPH_PA_CTL1] == 0x06);
  CHECK(b.wcd[WCD_HPH_NEW_INT_RDAC_HD2_CTL_L] == 0x01 && b.wcd[WCD_HPH_NEW_INT_RDAC_HD2_CTL_R] == 0x01);
  CHECK(wIdx(WCD_ANA_HPH, 0x3c) == 0 && b.swrAt[0] == b.wcdWrites.size());            // PA bits first, datapath last
  // earpiece
  b.wcd.clear();
  b.wcdWrites.clear();
  b.wcd[WCD_DIG_EAR_PATH_CTL] = 0x01;   // RX3: must become RX1
  b.wcd[WCD_FLYBACK_EN] = 0x0c;
  b.wcd[WCD_EAR_EN_REG] = 0x02;
  CHECK(gah_wcd_ear(&o, 1, gah_wcd_ear_gain(-1200)) == 0);
  CHECK(b.swr.size() == 3 && b.swr[2] == std::make_pair(1, (uint8_t)0x01));       // HPH_L only
  CHECK(b.wcd[WCD_ANA_EAR] == 0x80 && b.wcd[WCD_ANA_EAR_COMP_CTL] == (0x80 | (12 << 2)));
  CHECK(b.wcd[WCD_DIG_EAR_PATH_CTL] == 0x00 && b.wcd[WCD_FLYBACK_EN] == 0x08);
  CHECK(b.wcd[WCD_DIG_PDM_WD_CTL0] == 0x13 && b.wcd[WCD_ANA_HPH] == 0x0c && b.wcd[WCD_ANA_RX_SUPPLIES] == 0xc3);
  CHECK(wIdx(WCD_CLASSH_MODE_2, 0x3a) < b.swrAt[1] && b.swrAt[1] <= wIdx(WCD_ANA_EAR, 0x80));   // ear_dac FSM, then PA
  CHECK(wIdx(WCD_DIG_EAR_PATH_CTL) < wIdx(WCD_DIG_ANA_CLK_CTL));
  b.wcdWrites.clear();
  CHECK(gah_wcd_ear(&o, 0, 0) == 0);
  CHECK(b.swr.size() == 4 && b.swr[3] == std::make_pair(0, (uint8_t)0x01));
  CHECK(b.wcd[WCD_ANA_EAR] == 0 && b.wcd[WCD_ANA_EAR_COMP_CTL] == (12 << 2) && b.wcd[WCD_FLYBACK_EN] == 0x0c);
  CHECK(b.wcd[WCD_EAR_EN_REG] == 0x02 && b.wcd[WCD_ANA_HPH] == 0 && b.wcd[WCD_ANA_RX_SUPPLIES] == 0x02);
  const size_t t0 = wIdx(WCD_EAR_EN_REG, 0x00), t1 = wIdx(WCD_ANA_EAR, 0x80), t2 = wIdx(WCD_ANA_EAR, 0x00, t1 + 1),
               t3 = wIdx(WCD_EAR_EN_REG, 0x02);
  CHECK(wIdx(WCD_ANA_EAR, 0x00) == 0 && t0 < t1 && t1 < t2 && t2 < t3);                // PA off, CnP FSM toggle
  // the RX SoundWire datapath fails: no PA, clocks back off
  b.wcd.clear();
  b.swrRc = -3;
  CHECK(gah_wcd_hph(&o, 1, 8) == -3 && b.wcd[WCD_ANA_HPH] == 0 && b.wcd[WCD_DIG_ANA_CLK_CTL] == 0);
  CHECK(gah_wcd_ear(&o, 1, 12) == -3 && b.wcd[WCD_ANA_EAR] == 0 && b.wcd[WCD_ANA_RX_SUPPLIES] == 0x00 &&
        b.wcd[WCD_FLYBACK_EN] == 0x04);
  b.swrRc = 0;
}

// MBHC: init values (wcd_mbhc_initialise with the guamp DT), insertion through the MBHC_SW_DET interrupt status with
// the ADC classification (headset keeps MICBIAS2, headphone does not), removal, the level fallback, lost registers.
static void testMbhc() {
  Board b;
  setup(b);
  gah_ops o = makeOps(true);
  gah_jack j = {};
  b.wcd[WCD_ANA_MBHC_MECH] = 0x39;   // reset values (wcd937x-regmap.c)
  b.wcd[WCD_ANA_MBHC_ELECT] = 0x08;
  b.wcd[WCD_MBHC_NEW_CTL_1] = 0x02;
  b.wcd[WCD_MBHC_NEW_CTL_2] = 0x05;
  b.wcd[WCD_MBHC_NEW_PLUG_DETECT_CTL] = 0xe9;
  b.wcd[WCD_DIG_INTR_MASK_0] = 0xff;
  b.wcd[WCD_ANA_MICB2] = 0x20;       // 2.6 V (micbias2-mv 2600, set by wcdSetup)
  b.wcd[WCD_ANA_MBHC_RESULT_3] = 0x10;
  CHECK(gah_mbhc_lost(&o) == 1);
  CHECK(gah_mbhc_init(&o, &j) == 0 && j.inited && j.armed == 1 && !j.present);
  CHECK(b.wcd[WCD_ANA_MBHC_MECH] == 0xbd);                 // L_DET_EN, insertion, HPHL/GND plug type NO, comp, 100k
  CHECK(b.wcd[WCD_ANA_MBHC_ELECT] == 0x09 && b.wcd[WCD_MBHC_NEW_CTL_1] == 0x82 && b.wcd[WCD_MBHC_NEW_CTL_2] == 0x02);
  CHECK(b.wcd[WCD_MBHC_NEW_PLUG_DETECT_CTL] == 0xe6 && b.wcd[WCD_MBHC_NEW_INT_MECH_DET_CURRENT] == 0x08);
  CHECK(b.wcd[WCD_DIG_INTR_MASK_0] == 0xef && b.wcd[WCD_DIG_INTR_LEVEL_0] == 0 && b.wcd[WCD_DIG_INTR_LEVEL_0 + 2] == 0);
  CHECK(gah_mbhc_lost(&o) == 0);
  CHECK(gah_mbhc_poll(&o, &j, 5) == 0 && j.polls == 1);
  // headset in: switch interrupt, ADC 0x40 at 2.6 V = 1155 mV -> headset, MICBIAS2 stays on
  b.wcd[WCD_DIG_INTR_STATUS_0] = 0x10;
  b.wcd[WCD_MBHC_NEW_FSM_STATUS] = 0x40;
  b.wcd[WCD_MBHC_NEW_ADC_RESULT] = 0x40;
  CHECK(gah_mbhc_poll(&o, &j, 5) == 1 && j.present && j.type == GAH_JACK_HEADSET && j.mic_mv == 1155 && j.micb2);
  CHECK(b.wcd[WCD_DIG_INTR_STATUS_0] == 0 && (b.wcd[WCD_ANA_MBHC_MECH] & 0x20) == 0 && j.armed == 0);   // removal armed
  CHECK((b.wcd[WCD_ANA_MICB2] & 0xc0) == 0x40 && (b.wcd[WCD_MBHC_NEW_CTL_2] & 0x70) == 0x10);         // IN2P
  CHECK((b.wcd[WCD_ANA_MBHC_ELECT] & 0x80) == 0 && b.wcd[WCD_ANA_MICB2_RAMP] == 0 && j.irq_events == 1);
  // removal
  b.wcd[WCD_DIG_INTR_STATUS_0] = 0x10;
  CHECK(gah_mbhc_poll(&o, &j, 5) == 1 && !j.present && j.type == GAH_JACK_NONE && !j.micb2);
  CHECK((b.wcd[WCD_ANA_MICB2] & 0xc0) == 0 && (b.wcd[WCD_ANA_MBHC_MECH] & 0x20) == 0x20 && j.events == 2);
  // headphone in (mic ring at ground): MICBIAS2 back off after the measurement
  b.wcd[WCD_DIG_INTR_STATUS_0] = 0x10;
  b.wcd[WCD_MBHC_NEW_ADC_RESULT] = 0x02;
  CHECK(gah_mbhc_poll(&o, &j, 5) == 1 && j.type == GAH_JACK_HEADPHONE && j.mic_mv == 36 && !j.micb2);
  CHECK((b.wcd[WCD_ANA_MICB2] & 0xc0) == 0);
  // ADC never completes: plugged, type unknown
  b.wcd[WCD_DIG_INTR_STATUS_0] = 0x10;
  CHECK(gah_mbhc_poll(&o, &j, 5) == 1 && !j.present);
  b.wcd[WCD_DIG_INTR_STATUS_0] = 0x10;
  b.wcd[WCD_MBHC_NEW_FSM_STATUS] = 0x80;   // timeout
  CHECK(gah_mbhc_poll(&o, &j, 5) == 1 && j.present && j.type == GAH_JACK_UNKNOWN && !j.micb2);
  // level fallback: no interrupt, SWCH_LEVEL_REMOVE says "no plug" for 5 polls -> removal on the 5th
  b.wcd[WCD_ANA_MBHC_RESULT_3] = 0x10;
  for (int i = 0; i < 4; i++) CHECK(gah_mbhc_poll(&o, &j, 5) == 0 && j.present);
  CHECK(gah_mbhc_poll(&o, &j, 5) == 1 && !j.present && j.level_events == 1 && (b.wcd[WCD_ANA_MBHC_MECH] & 0x20) == 0x20);
  b.wcd[WCD_ANA_MBHC_RESULT_3] = 0x00;    // plugged again, a level glitch resets the count
  b.wcd[WCD_MBHC_NEW_FSM_STATUS] = 0x40;
  b.wcd[WCD_MBHC_NEW_ADC_RESULT] = 0xf8;  // open mic ring: high impedance
  for (int i = 0; i < 3; i++) CHECK(gah_mbhc_poll(&o, &j, 5) == 0);
  b.wcd[WCD_ANA_MBHC_RESULT_3] = 0x10;
  CHECK(gah_mbhc_poll(&o, &j, 5) == 0);
  b.wcd[WCD_ANA_MBHC_RESULT_3] = 0x00;
  for (int i = 0; i < 4; i++) CHECK(gah_mbhc_poll(&o, &j, 5) == 0);
  CHECK(gah_mbhc_poll(&o, &j, 5) == 1 && j.present && j.type == GAH_JACK_HIGH_HPH && (b.wcd[WCD_ANA_MBHC_MECH] & 0x20) == 0);
  for (int i = 0; i < 10; i++) CHECK(gah_mbhc_poll(&o, &j, 0) == 0);   // fallback off: the level alone does nothing
  b.wcd[WCD_ANA_MBHC_RESULT_3] = 0x10;
  for (int i = 0; i < 10; i++) CHECK(gah_mbhc_poll(&o, &j, 0) == 0 && j.present);
  // codec reset: registers back to reset values -> lost
  b.wcd[WCD_MBHC_NEW_CTL_1] = 0x02;
  CHECK(gah_mbhc_lost(&o) == 1);
}

int main() {
  testGeni();
  testHelpers();
  testSpeaker();
  testLpi();
  testRxMacro();
  testWcd();
  testMbhc();
  std::printf("audio_hw_host: %d checks, %d failures\n", gChecks, gFail);
  return gFail ? 1 : 0;
}
