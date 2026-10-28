// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp microphone, codec side (audio agent 3): bolero 2.1 TX macro + VA macro, the VA SoundWire master and the TX half
// of wcd937x, register by register as the guamp Linux kernel does it (techpack/audio, studied, no code copied):
//   asoc/codecs/bolero/tx-macro.c   tx_macro_mclk_enable, tx_macro_put_dec_enum, tx_macro_enable_dec (PRE/POST_PMU,
//                                   PRE/POST_PMD), tx_macro_hw_params, tx_macro_mute_update_callback,
//                                   tx_macro_tx_hpf_corner_freq_callback, tx_macro_init (SEC7, VA SWR_CTRL 0x0A on 2.1)
//   asoc/codecs/bolero/va-macro.c   va_macro_tx_va_mclk_enable (TX_MCLK case: SWR pins, SWR_CONTROL), va_macro_swrm_clock
//   asoc/codecs/bolero/bolero-clk-rsc.c  bolero_clk_rsc_fs_gen_request (DT fs-gen-sequence 0x3000 1, 0x3004 1, 0x3080 2)
//   soc/swr-mstr-ctrl.c + swrm_registers.h  swrm_master_init, swrm_runtime_resume, swrm_cmd_fifo_wr_cmd/rd_cmd,
//                                   swrm_get_logical_dev_num, swrm_connect_port, swrm_slvdev_datapath_control,
//                                   swrm_copy_data_port_config, enable_bank_switch, swrm_disable_ports
//   asoc/bengal-port-config.h       tx_frame_params_default (VA master port params)
//   soc/pinctrl-lpi.c               LPI CTL/DIR/slew encoding (tx_swr_clk/data1/data2 = LPI gpio0..2 func1, 10 mA)
//   asoc/codecs/wcd937x/wcd937x.c   wcd937x_init_reg, wcd937x_set_micbias_data, wcd937x_micbias_control,
//                                   wcd937x_codec_enable_adc, wcd937x_enable_req, wcd937x_tx_swr_ctrl, ADC2 MUX
// guamp facts (dtbo 0 fragment@23/24/26/62, base DT, vendor mixer_paths.xml):
//   mics: AMIC1 (MIC BIAS1 1.8 V, wcd937x ADC1) and AMIC3 (MIC BIAS3 1.8 V, ADC2 INP3) built in, AMIC2 = headset
//   (MIC BIAS2 2.6 V, ADC2 INP2); no DMIC (cdc_dmic01/23 fragments disabled). "handset-mic" and "speaker-mic" = amic3.
//   amic3 path: TX DEC0 MUX = SWR_MIC, TX SMIC MUX0 = SWR_MIC4, ADC2 MUX = INP3, ADC2_MIXER, TX_AIF1_CAP DEC0 ->
//   AFE TX_CODEC_DMA_TX_3 0xB037, one channel. amic1: SMIC MUX0 = SWR_MIC0, ADC1_MIXER.
//   wcd937x TX slave on the VA master (va_swr_master: swrm-io-base 0x0A740000, hctl 0x0A7EC100, master id 3,
//   1 device, enumeration id 0x0A01170223); every wcd937x register (also the RX ones) goes over THIS master.
//   tx_swr_ch_map: ADC1 = slave port 0 -> master type ADC1 (master port 1 ch 1); ADC2 = slave port 1 ch 1 -> master
//   type DMIC0 (master port 2 ch 1). VA master port params: {si 3, off1 1|2, off2 0, lane 0} (bengal-port-config.h).
//   TX macro 0x0A620000, VA macro 0x0A730000 (bolero logical register - macro start offset, 0x3000 for VA).
// Transport independent: every access goes through Bus (phys address), so tests/mic_capture_host.cpp runs it against a
// simulated SoundWire master + wcd937x. Called from the guamp-adsp thread (may sleep).
#ifndef GUAMP_MIC_CODEC_INC
#define GUAMP_MIC_CODEC_INC
#include "guamp_mic_dev.h"
namespace guampmic {

constexpr uint64_t kTx = 0x0A620000, kVa = 0x0A730000, kSwrm = 0x0A740000, kSwrmHctl = 0x0A7EC100;
constexpr uint64_t kLpi = 0x0A7C0000, kLpiSlew = 0x0A95A000;
constexpr uint64_t kWcdTxSlaveId = 0x0A01170223ull, kDevIdMask = 0xFFFFFFFFFFFFull;

// bolero TX macro (offsets from kTx)
constexpr uint32_t kTxMclkCtl = 0x000, kTxFsCnt = 0x004, kTxFreqMclk = 0x090, kTxSwrDmic0 = 0x0C0;
constexpr uint32_t kTxAdcMux0Cfg0 = 0x100, kTxAdcMux0Cfg1 = 0x104, kTxAdcMuxStride = 0x8;
constexpr uint32_t kTxPathCtl = 0x400, kTxPathCfg0 = 0x404, kTxVolCtl = 0x40C, kTxPathSec2 = 0x418, kTxPathSec7 = 0x42C;
constexpr uint32_t kTxPathStride = 0x80;
// bolero VA macro (offsets from kVa = logical 0x3000)
constexpr uint32_t kVaMclkCtl = 0x000, kVaFsCnt = 0x004, kVaSwrControl = 0x008, kVaTopCfg0 = 0x080;
constexpr uint32_t kVaDmicCfg = 0x094, kVaSwrCtrl = 0x0DC;
// SoundWire master (offsets from kSwrm)
constexpr uint32_t kSwHwVersion = 0x000, kSwCompCfg = 0x004, kSwSwReset = 0x008, kSwCompStatus = 0x014;
constexpr uint32_t kSwCompParams = 0x100, kSwIntStatus = 0x200, kSwIntMask = 0x204, kSwIntClear = 0x208;
constexpr uint32_t kSwCpuIntEn = 0x210, kSwWrCmd = 0x300, kSwRdCmd = 0x304, kSwFifoStatus = 0x30C;
constexpr uint32_t kSwFifoCfg = 0x314, kSwRdFifo = 0x318, kSwEnumCfg = 0x500, kSwBusCtrl = 0x1044, kSwMcpCfg = 0x1048;
constexpr uint32_t kSwMcpStatus = 0x104C, kSwSlvStatus = 0x1090, kSwIntMaskValue = 0x1FDFD;
inline uint32_t swDevId1(uint32_t m) { return 0x530 + 8 * m; }
inline uint32_t swDevId2(uint32_t m) { return 0x534 + 8 * m; }
inline uint32_t swFrameCtl(uint32_t bank) { return 0x101C + 0x40 * bank; }
inline uint32_t swPortCtl(uint32_t n, uint32_t bank) { return 0x1124 + 0x100 * (n - 1) + 0x40 * bank; }
inline uint32_t swPortCtl2(uint32_t n, uint32_t bank) { return 0x1128 + 0x100 * (n - 1) + 0x40 * bank; }
inline uint32_t swPortHctl(uint32_t n, uint32_t bank) { return 0x1134 + 0x100 * (n - 1) + 0x40 * bank; }
// SoundWire slave registers (on the bus)
constexpr uint16_t kScpIntMask1 = 0x41;
inline uint16_t slvFrameCtl(uint32_t bank) { return static_cast<uint16_t>(0x60 + 0x10 * bank); }
inline uint16_t slvDp(uint32_t port, uint32_t bank, uint32_t reg) {
  return static_cast<uint16_t>(reg + 0x100 * port + 0x10 * bank);
}
constexpr uint32_t kDpChEn = 0x120, kDpSample1 = 0x122, kDpOffset1 = 0x124, kDpOffset2 = 0x125, kDpLane = 0x128;
constexpr uint32_t kSwrVer151 = 0x01050001, kSwrVer16 = 0x01060000;
constexpr uint32_t kMclkHz = 9600000, kFrameSync = 4000;
// wcd937x (over SoundWire)
constexpr uint16_t kWAnaBias = 0x3001, kWTxCh1 = 0x300E, kWTxCh2 = 0x300F, kWTxCh3 = 0x3010, kWTxCh3Hpf = 0x3011;
constexpr uint16_t kWMicb1 = 0x3022, kWMicb2 = 0x3023, kWMicb3 = 0x3025, kWVbgFine = 0x3029;
constexpr uint16_t kWMicb1Test1 = 0x306B, kWMicb1Test2 = 0x306C, kWMicb2Test1 = 0x306E, kWMicb2Test2 = 0x306F;
constexpr uint16_t kWMicb3Test1 = 0x3071, kWMicb3Test2 = 0x3072, kWSurgeEn = 0x30E2, kWSleepCtl = 0x3103;
constexpr uint16_t kWTxCh2Sel = 0x3127, kWLdoRxTx = 0x312A, kWChipId0 = 0x3401, kWAnaClk = 0x3408, kWDigClk = 0x3409;
constexpr uint16_t kWReqCtl = 0x3457, kWEfuse16 = 0x34C0, kWEfuse17 = 0x34C1, kWEfuse29 = 0x34CD;
constexpr uint16_t kWRdacHd2L = 0x3133, kWRdacHd2R = 0x3136, kWRdacGain = 0x3132;

struct Bus {
  void *ctx = nullptr;
  uint32_t (*rd)(void *, uint64_t phys) = nullptr;
  void (*wr)(void *, uint64_t phys, uint32_t v) = nullptr;
  void (*delayUs)(void *, uint32_t us) = nullptr;
  void (*log)(void *, const char *what, uint32_t a, uint32_t b) = nullptr;
  uint32_t r(uint64_t a) const { return rd(ctx, a); }
  void w(uint64_t a, uint32_t v) const { wr(ctx, a, v); }
  void upd(uint64_t a, uint32_t mask, uint32_t v) const { w(a, (r(a) & ~mask) | (v & mask)); }
  void us(uint32_t n) const { if (delayUs) delayUs(ctx, n); }
  void note(const char *what, uint32_t a, uint32_t b) const { if (log) log(ctx, what, a, b); }
};

// One VA-master SoundWire link with the wcd937x TX slave on it
struct Swrm {
  Bus bus;
  uint64_t base = kSwrm, hctl = kSwrmHctl;
  uint32_t version = 0, wrDepth = 0, rdDepth = 0, mclk = kMclkHz, busClk = kMclkHz;
  uint8_t wcmdId = 0, rcmdId = 0, devNum = 0;
  bool up = false;
  uint32_t cmdErrors = 0, rdFails = 0;

  uint32_t r(uint32_t off) const { return bus.r(base + off); }
  void w(uint32_t off, uint32_t v) const { bus.w(base + off, v); }
  static uint32_t pack(uint8_t data, uint8_t dev, uint8_t id, uint16_t reg) {
    return reg | (static_cast<uint32_t>(id) << 16) | (static_cast<uint32_t>(dev) << 20) | (static_cast<uint32_t>(data) << 24);
  }
  static uint8_t nextId(uint8_t id) { return id < 14 ? static_cast<uint8_t>(id + 1) : 0; }
  uint32_t ssp(uint32_t rows, uint32_t cols) const { return (busClk * 2) / (rows * cols * kFrameSync); }
  // swrm_wait_for_fifo_avail (write side: outstanding commands below the FIFO depth)
  void waitWriteRoom() {
    for (unsigned i = 0; i < 30 && wrDepth && ((r(kSwFifoStatus) & 0x1F00) >> 8) >= wrDepth; i++) bus.us(500);
  }
  // swrm_cmd_fifo_wr_cmd: cmdId 0 = next rolling id, 0xF = broadcast (then 10 ms, msm variant)
  void write(uint8_t dev, uint16_t reg, uint8_t v, uint8_t cmdId = 0) {
    uint8_t id = cmdId;
    if (id != 0xF) id = wcmdId = nextId(wcmdId);
    waitWriteRoom();
    w(kSwWrCmd, pack(v, dev, id, reg));
    bus.us(id == 0xF ? 10000 : 150);
  }
  // swrm_cmd_fifo_rd_cmd: returns false when the answer never carries our command id
  bool read(uint8_t dev, uint16_t reg, uint8_t *v) {
    rcmdId = nextId(rcmdId);
    const uint32_t cmd = pack(1, dev, rcmdId, reg);
    waitWriteRoom();
    bus.us(100);
    w(kSwRdCmd, cmd);
    bus.us(250);
    for (unsigned i = 0; i < 30 && !((r(kSwFifoStatus) & 0x1F0000) >> 16); i++) bus.us(500);
    for (unsigned attempt = 0; attempt <= 3; attempt++) {
      const uint32_t d = r(kSwRdFifo);
      if (((d & 0xF00) >> 8) == rcmdId) {
        *v = static_cast<uint8_t>(d);
        return true;
      }
      bus.us(500);
      if (attempt == 2) w(kSwRdCmd, cmd);
    }
    rdFails++;
    return false;
  }
  bool linkUp() {   // swrm_check_link_status: only versions above 1.5.1 report it
    if (version <= kSwrVer151) return true;
    for (unsigned i = 0; i < 100; i++) {
      if (r(kSwCompStatus) & 1) return true;
      bus.us(500);
    }
    return false;
  }
  void hctlGate() {   // SW workaround of swrm_master_init for version >= 1.6
    if (version >= kSwrVer16 && hctl) bus.w(hctl, bus.r(hctl) & 0xFFFFFFFDu);
  }
  // swrm_runtime_resume (cold) + swrm_master_init: reset, frame 50 x 2, auto enumeration, retries, IRQ pulse
  int init() {
    version = r(kSwHwVersion);
    const uint32_t params = r(kSwCompParams);
    wrDepth = (params & 0x7C00) >> 10;
    rdDepth = (params & 0xF8000) >> 15;
    bus.note("swr version/params", version, params);
    if (version == 0 || version == 0xFFFFFFFFu) return -1;   // macro unclocked or not there
    w(kSwSwReset, 1);
    w(kSwSwReset, 1);
    hctlGate();
    bus.us(500);
    w(swFrameCtl(0), (1u << 3) | 0u | ((ssp(50, 2) - 1) << 16));   // SWR_ROW_50, SWR_MIN_COL, ssp 48
    w(kSwEnumCfg, 1);
    w(kSwMcpCfg, (r(kSwMcpCfg) & ~0x3E0000u) | (0x1Fu << 17));
    w(kSwFifoCfg, 3);
    w(kSwBusCtrl, 2);
    w(kSwCompCfg, 2);
    w(kSwIntClear, 0xFFFFFFFFu);
    w(kSwIntMask, kSwIntMaskValue);
    w(kSwCpuIntEn, kSwIntMaskValue);
    w(kSwCompCfg, 3);
    hctlGate();
    if (!linkUp()) {
      bus.note("swr link not connected", r(kSwCompStatus), 0);
      return -2;
    }
    if (version >= kSwrVer151) w(kSwFifoCfg, r(kSwFifoCfg) | 0x80000000u);
    bus.us(100);
    write(0xF, kScpIntMask1, 0x4, 0xF);   // broadcast SCP_INT_STATUS_MASK_1 (runtime_resume)
    up = true;
    return 0;
  }
  // swrm_get_logical_dev_num: the auto-enumerated slot whose 48-bit id is the wcd937x TX slave, status attached
  int enumerate(uint64_t want, unsigned waitMs) {
    for (unsigned t = 0; t <= waitMs; t++) {
      const uint32_t st = r(kSwSlvStatus);
      for (uint32_t m = 1; m <= 11; m++) {
        const uint32_t s = (st >> (2 * m)) & 3;
        if (s != 1 && s != 2) continue;
        const uint64_t id = ((static_cast<uint64_t>(r(swDevId2(m))) << 32) | r(swDevId1(m))) & kDevIdMask;
        if (id == want) {
          devNum = static_cast<uint8_t>(m);
          bus.note("swr slave attached (dev num, ms)", m, t);
          return 0;
        }
      }
      bus.us(1000);
    }
    bus.note("swr slave status / int status", r(kSwSlvStatus), r(kSwIntStatus));
    return -3;
  }
  uint8_t inactiveBank() const { return (r(kSwMcpStatus) & 1) ? 0 : 1; }
  void bankSwitch(uint32_t bank, uint32_t row, uint32_t col) {
    write(0xF, slvFrameCtl(bank), static_cast<uint8_t>((row << 3) | col), 0xF);
  }
};

// One data port of a capture: slave port of wcd937x, master port (1-based) and the bengal params of that master port
struct Port {
  uint32_t slavePort, slaveCh, masterPort, masterCh, si, off1, off2, lane;
};
// amic1: ADC1 slave port 0 -> master port 1 ch 1, params {3, 1, 0}; amic3: ADC2 slave port 1 -> master port 2 ch 1, {3, 2, 0}
inline Port portFor(uint32_t mic) {
  return mic == GUAMP_MIC_AMIC1 ? Port{0, 1, 1, 1, 3, 1, 0, 0} : Port{1, 1, 2, 1, 3, 2, 0, 0};
}

// The whole codec side of one capture. bringUp(): clocks inside the macros + SWR link + wcd937x init (the AFE side has
// voted LPASS and enabled TX core/NPL before). pathOn()/pathOff(): the DAPM sequence of the capture. bringDown().
struct Codec {
  Bus bus;
  Swrm swr;
  uint32_t mic = GUAMP_MIC_AMIC3, dec = 0, rateCode = 4 /* 48 kHz */, dmicDiv = 2 /* TX_MACRO_CLK_DIV_4 */;
  int32_t gainDb = 0;
  uint32_t micbMv[3] = {1800, 2600, 1800};
  bool fsgen = false, txMclk = false, swrClk = false, wcdInit = false, micbOn = false, decOn = false, portOn = false;
  bool reqOn = false, adcOn = false;
  uint32_t hpfOrig = 0, chipId = 0, errors = 0;
  int failedAt = 0;

  uint64_t tx(uint32_t off) const { return kTx + off; }
  uint64_t va(uint32_t off) const { return kVa + off; }
  uint64_t txp(uint32_t off) const { return kTx + off + kTxPathStride * dec; }
  bool wrd(uint16_t reg, uint8_t *v) {
    if (swr.read(swr.devNum, reg, v)) return true;
    errors++;
    *v = 0;
    return false;
  }
  void wwr(uint16_t reg, uint8_t v) { swr.write(swr.devNum, reg, v); }
  void wupd(uint16_t reg, uint8_t mask, uint8_t v) {
    uint8_t old = 0;
    wrd(reg, &old);
    wwr(reg, static_cast<uint8_t>((old & ~mask) | (v & mask)));
  }
  static uint8_t micbVout(uint32_t mv) { return mv < 1000 || mv > 2850 ? 0x10 : static_cast<uint8_t>((mv - 1000) / 50); }

  // LPI pinctrl: func1, 10 mA, pull, slew 3 (tx_swr_*_active) or pull-down (sleep)
  void lpiPin(uint32_t gpio, uint32_t pull, uint32_t slewBit, bool active) {
    const uint64_t ctl = kLpi + 0x1000 * gpio;
    const uint32_t drive = 10 / 2 - 1;
    bus.w(ctl, (bus.r(ctl) & ~0x3FFu) | (1u << 2) | ((active ? pull : 1u) & 3) | (drive << 6));
    bus.w(ctl + 4, 0);
    if (active) bus.upd(kLpiSlew, 3u << slewBit, 3u << slewBit);
  }
  void fsGen(bool on) {   // DT fs-gen-sequence; off in reverse order
    if (on) {
      bus.upd(va(kVaMclkCtl), 1, 1);
      bus.upd(va(kVaFsCnt), 1, 1);
      bus.upd(va(kVaTopCfg0), 2, 2);
    } else {
      bus.upd(va(kVaTopCfg0), 2, 0);
      bus.upd(va(kVaFsCnt), 1, 0);
      bus.upd(va(kVaMclkCtl), 1, 0);
    }
  }
  int fail(int step, const char *why, uint32_t a) {
    if (!failedAt) failedAt = step;
    bus.note(why, static_cast<uint32_t>(step), a);
    return -step;
  }

  // ---- bring-up: TX MCLK, VA SWR clock + pins, SoundWire link, wcd937x enumeration + init -----------------------------
  int bringUp(int (*fsgenHook)(void *, bool), void *hookCtx) {
    swr.bus = bus;
    if (fsgenHook ? fsgenHook(hookCtx, true) == 0 : (fsGen(true), true)) fsgen = true;
    // tx_macro_mclk_enable (first user): 9.6 MHz MCLK, MCLK + FS counter on
    bus.upd(tx(kTxFreqMclk), 1, 1);
    bus.upd(tx(kTxMclkCtl), 1, 1);
    bus.upd(tx(kTxFsCnt), 1, 1);
    txMclk = true;
    bus.upd(txp(kTxPathSec7), 0x3F, 0x0A);   // tx_macro_reg_init
    bus.upd(va(kVaSwrCtrl), 0x0F, 0x0A);     // tx_macro_init, bolero 2.1
    // va_macro_tx_va_mclk_enable(TX_MCLK, first SWR user): pins active, SWR clock with the reset pulse of probe
    lpiPin(0, 0 /* bias-disable */, 0, true);
    lpiPin(1, 2 /* bus-hold */, 2, true);
    lpiPin(2, 2, 4, true);
    bus.upd(va(kVaSwrControl), 2, 2);
    bus.upd(va(kVaSwrControl), 1, 1);
    bus.upd(va(kVaSwrControl), 2, 0);
    swrClk = true;
    int e = swr.init();
    if (e) return fail(1, "swr master init failed", static_cast<uint32_t>(-e));
    e = swr.enumerate(kWcdTxSlaveId, 300);
    if (e) return fail(2, "wcd937x tx slave not enumerated", 0);
    return wcdSetup();
  }
  // wcd937x_init_reg + set_micbias_data (once per power-up of the codec)
  int wcdSetup() {
    uint8_t id[4] = {};
    for (uint32_t i = 0; i < 4; i++) wrd(static_cast<uint16_t>(kWChipId0 + i), &id[i]);
    chipId = id[0] | (id[1] << 8) | (id[2] << 16) | (static_cast<uint32_t>(id[3]) << 24);
    bus.note("wcd937x chip id (0..3)", chipId, swr.rdFails);
    if (swr.rdFails) return fail(3, "wcd937x register reads fail", swr.rdFails);
    uint8_t e16 = 0, e17 = 0, e29 = 0;
    wrd(kWEfuse16, &e16);
    wrd(kWEfuse17, &e17);
    wrd(kWEfuse29, &e29);
    wupd(kWSleepCtl, 0x0E, (e16 == 0x02 || e17 > 0x09) ? (e29 & 0x0F) : 0x0E);
    wupd(kWSleepCtl, 0x80, 0x80);
    bus.us(1000);
    wupd(kWSleepCtl, 0x40, 0x40);
    bus.us(1000);
    wupd(kWLdoRxTx, 0x10, 0x00);
    wupd(kWVbgFine, 0xF0, 0x80);
    wupd(kWAnaBias, 0x80, 0x80);
    wupd(kWAnaBias, 0x40, 0x40);
    bus.us(10000);
    wupd(kWAnaBias, 0x40, 0x00);
    wupd(kWSurgeEn, 0xFF, 0xD9);
    wupd(kWMicb1Test1, 0xFF, 0xFA);
    wupd(kWMicb2Test1, 0xFF, 0xFA);
    wupd(kWMicb3Test1, 0xFF, 0xFA);
    wupd(kWMicb1Test2, 0x38, 0x00);
    wupd(kWMicb2Test2, 0x38, 0x00);
    wupd(kWMicb3Test2, 0x38, 0x00);
    if (e16 == 0x01) {
      wupd(kWVbgFine, 0xF0, 0xB0);
    } else if (e16 == 0x02) {
      wupd(kWRdacHd2L, 0x1F, 0x04);
      wupd(kWRdacHd2R, 0x1F, 0x04);
      wupd(kWVbgFine, 0xF0, 0xB0);
      wupd(kWRdacGain, 0xF0, 0x50);
    }
    wupd(kWMicb1, 0x3F, micbVout(micbMv[0]));
    wupd(kWMicb2, 0x3F, micbVout(micbMv[1]));
    wupd(kWMicb3, 0x3F, micbVout(micbMv[2]));
    wcdInit = true;
    bus.note("wcd937x init done (efuse16/17)", e16, e17);
    return 0;
  }

  // ---- capture path on: mixer controls, hw_params, then the DAPM power-up order of soc-dapm.c (4.19) ------------------
  int pathOn() {
    const bool amic3 = mic != GUAMP_MIC_AMIC1;
    const uint32_t smic = amic3 ? 5 /* SWR_MIC4 */ : 1 /* SWR_MIC0 */;
    // mixer controls of mixer_paths.xml "amic3"/"amic1" (tx_macro_put_dec_enum + ADC2 MUX + volumes)
    bus.upd(tx(kTxAdcMux0Cfg1 + kTxAdcMuxStride * dec), 0x3, 0x1);          // TX DEC0 MUX = SWR_MIC
    if (smic >= 5) {
      bus.upd(txp(kTxPathCfg0), 0x80, 0x80);
      bus.upd(va(kVaDmicCfg), 0x80, 0x00);
      bus.upd(tx(kTxSwrDmic0 + ((smic - 5) / 2) * 4), 0x0E, dmicDiv << 1);
    } else {
      bus.upd(txp(kTxPathCfg0), 0x80, 0x00);
    }
    bus.upd(tx(kTxAdcMux0Cfg0 + kTxAdcMuxStride * dec), 0xF, smic);         // TX SMIC MUX0
    if (amic3) wupd(kWTxCh2Sel, 0x80, 0x80);                                  // ADC2 MUX = INP3
    wupd(amic3 ? kWTxCh2 : kWTxCh1, 0x1F, 12);                                // ADCx Volume 12 (mixer_paths default)
    bus.w(txp(kTxVolCtl), static_cast<uint8_t>(static_cast<int8_t>(gainDb)));  // TX_DEC0 Volume (s8 dB)
    // tx_macro_hw_params: decimator rate 48 kHz
    bus.upd(txp(kTxPathCtl), 0x0F, rateCode);
    // micbias (seq 3): wcd937x_micbias_control(MICB_ENABLE) + POST_PMU 1 ms
    const uint16_t micb = amic3 ? kWMicb3 : kWMicb1;
    wupd(kWDigClk, 0xF0, 0xF0);
    wupd(kWAnaClk, 0x10, 0x10);
    wupd(kWMicb1Test2, 0x01, 0x01);
    wupd(kWMicb2Test2, 0x01, 0x01);
    wupd(kWMicb3Test2, 0x01, 0x01);
    wupd(micb, 0xC0, 0x40);
    micbOn = true;
    bus.us(1000);
    // ADC (seq 4): wcd937x_codec_enable_adc PRE_PMU (amic3 = INP3: no BCS/MBHC port), port request
    wupd(kWDigClk, 0x80, 0x80);
    wupd(kWAnaClk, 0x08, 0x08);
    wupd(kWAnaClk, 0x10, 0x10);
    adcOn = true;
    // TX DEC0 MUX (seq 6): tx_macro_enable_dec PRE_PMU then POST_PMU
    if (smic >= 5) bus.upd(tx(kTxSwrDmic0 + ((smic - 5) / 2) * 4), 0x0E, dmicDiv << 1);
    bus.upd(txp(kTxPathCfg0), 0x06, 0);          // dec mode 0
    bus.upd(txp(kTxPathCtl), 0x10, 0x10);        // PGA mute
    bus.upd(txp(kTxPathCtl), 0x20, 0x20);        // decimator on
    hpfOrig = (bus.r(txp(kTxPathCfg0)) & 0x60) >> 5;
    if (hpfOrig != 2) bus.upd(txp(kTxPathCfg0), 0x60, 2u << 5);   // CF_MIN_3DB_150HZ while settling
    if (hpfOrig != 2) {
      bus.upd(txp(kTxPathSec2), 0x03, 0x02);
      bus.upd(txp(kTxPathSec2), 0x03, 0x01);
      bus.us(6000);
    }
    bus.w(txp(kTxVolCtl), bus.r(txp(kTxVolCtl)));   // apply gain after the decimator is enabled
    decOn = true;
    // mixers (seq 8): ADCx REQ (wcd937x_enable_req) then ADCx_MIXER (tx_swr_ctrl = SoundWire datapath)
    wupd(kWReqCtl, 0x02, 0x02);
    wupd(kWReqCtl, 0x01, 0x00);
    wupd(kWTxCh2, 0x40, 0x40);
    wupd(kWTxCh3Hpf, 0x40, 0x40);
    wupd(kWDigClk, 0x70, 0x70);
    wupd(kWTxCh1, 0x80, 0x80);
    wupd(kWTxCh2, 0x40, 0x00);
    wupd(kWTxCh2, 0x80, 0x80);
    wupd(kWTxCh3, 0x80, 0x80);
    reqOn = true;
    datapath(true);
    portOn = true;
    // tx_macro_mute_update_callback after TX_MACRO_AMIC_UNMUTE_DELAY_MS (100), the HPF corner back after 300 ms
    bus.us(100000);
    bus.upd(txp(kTxPathCtl), 0x10, 0x00);
    if (hpfOrig != 2) {
      bus.us(200000);
      bus.upd(txp(kTxPathCfg0), 0x60, hpfOrig << 5);
      bus.upd(txp(kTxPathSec2), 0x03, 0x02);
      bus.us(20);   // 48 kHz: 20 us between the HPF gate toggles
      bus.upd(txp(kTxPathSec2), 0x03, 0x01);
    }
    bus.note("capture path on (mic, wcd errors)", mic, errors);
    return errors ? -4 : 0;
  }
  void setGain(int32_t centibel) {
    gainDb = centibel / 100;
    if (decOn) bus.w(txp(kTxVolCtl), static_cast<uint8_t>(static_cast<int8_t>(gainDb)));
  }

  // swrm_connect_port + swrm_slvdev_datapath_control: port tables in the inactive bank, frame 50 x 16, bank switch,
  // the same tables in the other bank. Disable: channel 0 / port off in both banks, back to 50 x 2.
  void portTables(const Port &p, uint32_t bank, bool on) {
    const uint8_t d = swr.devNum;
    swr.write(d, slvDp(p.slavePort, bank, kDpChEn), on ? static_cast<uint8_t>(p.slaveCh) : 0);
    if (on) {
      swr.write(d, slvDp(p.slavePort, bank, kDpSample1), static_cast<uint8_t>(p.si));
      swr.write(d, slvDp(p.slavePort, bank, kDpOffset1), static_cast<uint8_t>(p.off1));
      swr.write(d, slvDp(p.slavePort, bank, kDpOffset2), static_cast<uint8_t>(p.off2));
      swr.write(d, slvDp(p.slavePort, bank, kDpLane), static_cast<uint8_t>(p.lane));
      swr.w(swPortCtl(p.masterPort, bank), (p.masterCh << 24) | (p.off2 << 16) | (p.off1 << 8) | p.si);
      swr.w(swPortCtl2(p.masterPort, bank), p.lane);
      swr.w(swPortHctl(p.masterPort, bank), (0xFu << 4) | 0x0u);   // SWR_HSTOP_MAX_VAL, SWR_HSTART_MIN_VAL
    } else {
      swr.w(swPortCtl(p.masterPort, bank), 0);
    }
  }
  void datapath(bool on) {
    const Port p = portFor(mic);
    const uint32_t bank = swr.inactiveBank();
    if (on) {
      swr.w(kSwCpuIntEn, kSwIntMaskValue);
      portTables(p, bank, true);   // swrm_apply_port_config (disable-div2-clk-switch = 1: no DIV2 write)
    } else {
      portTables(p, bank, false);
    }
    const uint32_t nCol = on ? 7 /* SWR_MAX_COL, 16 */ : 0 /* SWR_MIN_COL, 2 */, col = on ? 16 : 2;
    const uint32_t mask = 0xF8u | 0x7u | 0x700u | 0xFF0000u;
    const uint32_t v = (swr.r(swFrameCtl(bank)) & ~mask) | (1u << 3) | nCol | (0u << 8) | ((swr.ssp(50, col) - 1) << 16);
    swr.w(swFrameCtl(bank), v);
    swr.bankSwitch(bank, 1 /* SWR_ROW_50 */, nCol);
    portTables(p, bank ? 0 : 1, on);
    bus.note(on ? "swr datapath on (bank, frame ctl)" : "swr datapath off (bank, frame ctl)", bank, v);
  }

  // ---- capture path off: the DAPM power-down of the same widgets ----------------------------------------------------
  void pathOff() {
    const bool amic3 = mic != GUAMP_MIC_AMIC1;
    if (portOn) datapath(false);
    portOn = false;
    if (reqOn) {
      wupd(kWTxCh1, 0x80, 0x00);
      wupd(kWTxCh2, 0x80, 0x00);
      wupd(kWTxCh3, 0x80, 0x00);
      wupd(kWDigClk, 0x10, 0x00);
    }
    reqOn = false;
    if (decOn) {
      bus.upd(txp(kTxPathCtl), 0x10, 0x10);
      bus.upd(txp(kTxPathCtl), 0x20, 0x00);
      bus.upd(txp(kTxPathCfg0), 0x06, 0x00);
      bus.upd(txp(kTxPathCtl), 0x10, 0x00);
    }
    decOn = false;
    if (adcOn) wupd(kWAnaClk, 0x08, 0x00);
    adcOn = false;
    if (micbOn) {
      wupd(amic3 ? kWMicb3 : kWMicb1, 0xC0, 0x00);
      wupd(kWAnaClk, 0x10, 0x00);
      wupd(kWDigClk, 0x80, 0x00);
    }
    micbOn = false;
    bus.note("capture path off (mic, wcd errors)", mic, errors);
  }
  void bringDown(int (*fsgenHook)(void *, bool), void *hookCtx) {
    if (swrClk) {
      bus.upd(va(kVaSwrControl), 1, 0);
      for (uint32_t g = 0; g < 3; g++) lpiPin(g, 1, 0, false);   // tx_swr_*_sleep: pull-down
    }
    swrClk = false;
    swr.up = false;
    if (txMclk) {
      bus.upd(tx(kTxFsCnt), 1, 0);
      bus.upd(tx(kTxMclkCtl), 1, 0);
    }
    txMclk = false;
    if (fsgen) {
      if (fsgenHook) fsgenHook(hookCtx, false);
      else fsGen(false);
    }
    fsgen = false;
  }
};

}  // namespace guampmic
#endif
