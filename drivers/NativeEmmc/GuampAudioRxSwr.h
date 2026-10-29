// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp audio output, RX SoundWire link: the bolero RX macro's SoundWire master (rx_swr_master) and the wcd937x RX
// slave on it, which carry the earpiece/headphone PCM from the RX macro interpolators to the wcd937x DACs. Registers of
// the wcd937x itself are NOT reached here: they go over the VA (TX) link of the capture side (guamp_wcd937x_read/write).
// Reuses guampmic::Swrm (GuampMicCodec.inc: swrm_master_init, enumeration, command FIFO), another instance.
// Linux sources (techpack/audio, studied, no code copied):
//   asoc/codecs/bolero/rx-macro.c  rx_swrm_clock (pins active, rx_macro_mclk_enable, SWR_CONTROL reset pulse + enable)
//   soc/swr-mstr-ctrl.c           swrm_connect_port, swrm_get_port_config/copy_port_tables, swrm_copy_data_port_config,
//                                 swrm_slvdev_datapath_control, swrm_disable_ports, enable_bank_switch
//   asoc/bengal-port-config.h     rx_frame_params_default (UC0)
//   soc/pinctrl-lpi.c + DT        rx_swr_clk/data/data1 = LPI gpio3/4/5 func1 10 mA slew 3 (slew bits 8/10/12)
// guamp DT (dtbo 0 fragment@23): rx_swr_master swrm-io-base 0x0A610000, hctl 0x0A6A9098, master id 2, IRQ SPI 297,
//   5 ports, block packing mode 1, disable-div2-clk-switch, port mapping {1: HPH_L ch1, HPH_R ch2} {2: CLSH} {3: COMP}
//   {4: LO} {5: DSD}; wcd937x-rx-slave reg <0x0a 0x1170224>; wcd937x rx_swr_ch_map: slave port 0 = HPH_L ch1 + HPH_R ch2.
// Transport independent (guampmic::Bus by physical address): tests/audio_rx_host.cpp runs it on a simulated master.
#ifndef GUAMP_AUDIO_RX_SWR_H
#define GUAMP_AUDIO_RX_SWR_H
#include "GuampMicCodec.inc"
namespace guamprx {

constexpr uint64_t kRxSwrm = 0x0A610000, kRxSwrmHctl = 0x0A6A9098, kRxMacro = 0x0A600000;
constexpr uint64_t kWcdRxSlaveId = 0x0A01170224ull;
constexpr uint32_t kRxMacroSwrControl = 0x108;   // BOLERO_CDC_RX_CLK_RST_CTRL_SWR_CONTROL - RX_START_OFFSET
constexpr uint32_t kInvalid = 0xFF;              // SWR_INVALID_PARAM
// bengal rx_frame_params_default, port 1 (HPH): {si, off1, off2, hstart, hstop, wd_len, bp_mode, bgp_ctrl, lane_ctrl}
struct PortParams { uint32_t si, off1, off2, hstart, hstop, wdLen, bpMode, bgpCtrl, lane; };
constexpr PortParams kRxPort1 = {3, 0, 0, kInvalid, kInvalid, 1, kInvalid, kInvalid, 1};
// SoundWire slave data-port registers (swrm_registers.h, slave port p = DP(p+1), bank b)
inline uint16_t sDp(uint32_t p, uint32_t bank, uint32_t reg) { return static_cast<uint16_t>(reg + 0x100 * p + 0x10 * bank); }
constexpr uint32_t kSChEn = 0x120, kSBlk2 = 0x121, kSSample1 = 0x122, kSOff1 = 0x124, kSOff2 = 0x125, kSHctl = 0x126;
constexpr uint32_t kSBlk3 = 0x127, kSLane = 0x128;
inline uint16_t sBlk1(uint32_t p) { return static_cast<uint16_t>(0x103 + 0x100 * p); }
// master data-port registers (port n 1-based)
inline uint32_t mPortCtl(uint32_t n, uint32_t b) { return 0x1124 + 0x100 * (n - 1) + 0x40 * b; }
inline uint32_t mPortCtl2(uint32_t n, uint32_t b) { return 0x1128 + 0x100 * (n - 1) + 0x40 * b; }
inline uint32_t mBlk1(uint32_t n) { return 0x112C + 0x100 * (n - 1); }
inline uint32_t mBlk2(uint32_t n, uint32_t b) { return 0x1130 + 0x100 * (n - 1) + 0x40 * b; }
inline uint32_t mHctl(uint32_t n, uint32_t b) { return 0x1134 + 0x100 * (n - 1) + 0x40 * b; }
inline uint32_t mBlk3(uint32_t n, uint32_t b) { return 0x1138 + 0x100 * (n - 1) + 0x40 * b; }

struct RxLink {
  guampmic::Bus bus;
  guampmic::Swrm swr;
  bool pins = false, clk = false, up = false, portOn = false;
  uint8_t slaveCh = 0;   // slave channel mask of port 0 now enabled (1 = HPH_L, 3 = HPH_L + HPH_R)
  int failedAt = 0;

  // LPI pinctrl (same encoding as guampmic::Codec::lpiPin): rx_swr_clk gpio3 bias-disable, data gpio4 / data1 gpio5
  // bus-hold, func1, 10 mA, slew 3; sleep: pull-down input.
  void pin(uint32_t gpio, uint32_t pull, uint32_t slewBit, bool active) {
    const uint64_t ctl = guampmic::kLpi + 0x1000 * gpio;
    bus.w(ctl, (bus.r(ctl) & ~0x3FFu) | (1u << 2) | ((active ? pull : 1u) & 3) | ((10 / 2 - 1) << 6));
    bus.w(ctl + 4, 0);
    if (active) bus.upd(guampmic::kLpiSlew, 3u << slewBit, 3u << slewBit);
  }
  void pinsSet(bool active) {
    pin(3, 0, 8, active);
    pin(4, 2, 10, active);
    pin(5, 2, 12, active);
    pins = active;
  }
  // rx_swrm_clock(enable) after rx_macro_mclk_enable (done by the caller, gah_rxm_mclk): pins, SWR clock with the reset
  // pulse, then swrm_master_init and the enumeration of the wcd937x RX slave.
  int bringUp(unsigned enumMs = 300) {
    swr.bus = bus;
    swr.base = kRxSwrm;
    swr.hctl = kRxSwrmHctl;
    pinsSet(true);
    bus.upd(kRxMacro + kRxMacroSwrControl, 2, 2);
    bus.upd(kRxMacro + kRxMacroSwrControl, 1, 1);
    bus.upd(kRxMacro + kRxMacroSwrControl, 2, 0);
    clk = true;
    int e = swr.init();
    if (e) return fail(1, "rx swr master init failed (version, step)", swr.version);
    e = swr.enumerate(kWcdRxSlaveId, enumMs);
    if (e) return fail(2, "wcd937x rx slave not enumerated (status)", swr.r(guampmic::kSwSlvStatus));
    up = true;
    bus.note("rx swr link up (version, dev num)", swr.version, swr.devNum);
    return 0;
  }
  int fail(int step, const char *why, uint32_t a) {
    if (!failedAt) failedAt = step;
    bus.note(why, a, static_cast<uint32_t>(step));
    return -step;
  }

  // swrm_copy_data_port_config for master port 1 in one bank: slave DP1 registers (valid params only), master port.
  void portTables(uint32_t bank, uint8_t slaveCh, uint8_t masterCh) {
    const PortParams &p = kRxPort1;
    const uint8_t d = swr.devNum;
    swr.write(d, sDp(0, bank, kSChEn), slaveCh);
    swr.write(d, sDp(0, bank, kSSample1), static_cast<uint8_t>(p.si));
    swr.write(d, sDp(0, bank, kSOff1), static_cast<uint8_t>(p.off1));
    if (p.off2 != kInvalid) swr.write(d, sDp(0, bank, kSOff2), static_cast<uint8_t>(p.off2));
    if (p.hstart != kInvalid && p.hstop != kInvalid)
      swr.write(d, sDp(0, bank, kSHctl), static_cast<uint8_t>((p.hstart << 4) | p.hstop));
    if (p.wdLen != kInvalid) swr.write(d, sBlk1(0), static_cast<uint8_t>(p.wdLen));
    if (p.bpMode != kInvalid) swr.write(d, sDp(0, bank, kSBlk3), static_cast<uint8_t>(p.bpMode));
    if (p.bgpCtrl != kInvalid) swr.write(d, sDp(0, bank, kSBlk2), static_cast<uint8_t>(p.bgpCtrl));
    if (p.lane != kInvalid) swr.write(d, sDp(0, bank, kSLane), static_cast<uint8_t>(p.lane));
    uint32_t v = (static_cast<uint32_t>(masterCh) << 24) | (p.off1 << 8) | p.si;
    if (p.off2 != kInvalid) v |= p.off2 << 16;
    swr.w(mPortCtl(1, bank), v);
    if (p.lane != kInvalid) swr.w(mPortCtl2(1, bank), p.lane);
    if (p.wdLen != kInvalid) swr.w(mBlk1(1), p.wdLen);
    if (p.bgpCtrl != kInvalid) swr.w(mBlk2(1, bank), p.bgpCtrl);
    swr.w(mHctl(1, bank), (p.hstart != kInvalid && p.hstop != kInvalid) ? ((p.hstop << 4) | p.hstart) : 0xF0u);
    if (p.bpMode != kInvalid) swr.w(mBlk3(1, bank), p.bpMode);
  }
  // swrm_disable_ports for port 1 in one bank: channel enables off on both sides.
  void portOff(uint32_t bank) {
    swr.write(swr.devNum, sDp(0, bank, kSChEn), 0);
    swr.w(mPortCtl(1, bank), swr.r(mPortCtl(1, bank)) & 0x00FFFFFFu);
  }
  // swrm_slvdev_datapath_control: tables in the inactive bank, frame 50 x 16 (on) or 50 x 2 (off, no port left),
  // bank switch, the same tables in the bank that just became inactive. Master channel mask = slave mask (HPH_L is
  // master port 1 ch 1, HPH_R ch 2: swr-port-mapping).
  int datapath(bool on, uint8_t slaveCh) {
    if (!up) return fail(3, "rx swr datapath without link (on)", on);
    const uint32_t bank = swr.inactiveBank();
    if (on) {
      swr.w(guampmic::kSwCpuIntEn, guampmic::kSwIntMaskValue);
      portTables(bank, slaveCh, slaveCh);
    } else {
      portOff(bank);
    }
    const uint32_t nCol = on ? 7 /* SWR_MAX_COL, 16 */ : 0 /* SWR_MIN_COL, 2 */, col = on ? 16 : 2;
    const uint32_t mask = 0xF8u | 0x7u | 0x700u | 0xFF0000u;
    const uint32_t v = (swr.r(guampmic::swFrameCtl(bank)) & ~mask) | (1u << 3) | nCol | ((swr.ssp(50, col) - 1) << 16);
    swr.w(guampmic::swFrameCtl(bank), v);
    swr.bankSwitch(bank, 1 /* SWR_ROW_50 */, nCol);
    if (on) portTables(bank ? 0 : 1, slaveCh, slaveCh);
    else portOff(bank ? 0 : 1);
    portOn = on;
    this->slaveCh = on ? slaveCh : 0;
    bus.note(on ? "rx swr datapath on (bank, slave ch)" : "rx swr datapath off (bank, frame ctl)", bank, on ? slaveCh : v);
    return 0;
  }
  // rx_swrm_clock(disable): SWR clock off, pins to sleep. The RX macro MCLK goes off in the caller.
  void bringDown() {
    if (portOn) datapath(false, 0);
    if (clk) bus.upd(kRxMacro + kRxMacroSwrControl, 1, 0);
    clk = false;
    if (pins) pinsSet(false);
    up = false;
    swr.up = false;
    swr.devNum = 0;
    bus.note("rx swr link down", 0, 0);
  }
};

}  // namespace guamprx
#endif
