// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp (Moto G9 Play, SM6115 / bengal) Bluetooth board facts and the pure helpers around them. Every constant states
// its source (Linux tree android_kernel_motorola_sm6225, the guamp bluetooth_a partition backup). Nothing here touches
// hardware; BtService.inc owns the MMIO. Host-tested by tests/bt_host.cpp.
#ifndef GUAMP_BT_BOARD_H
#define GUAMP_BT_BOARD_H
#include <stdint.h>
#include "GuampTouchBoard.h"   // GCC base, QUP wrap0 common votes, gt_pin_cfg, gt_level_hz (same wrapper as touch SE2)

// bengal-qupv3.dtsi qupv3_se3_4uart: qcom,qup_uart@4a8c000, "qcom,msm-geni-serial-hs", reg 0x4a8c000 0x4000, clocks
// GCC_QUPV3_WRAP0_S3_CLK + wrap0 m/s AHB, interrupts GIC SPI 330 and TLMM 11 (RX wake), qcom,wakeup-byte 0xFD.
#define BT_SE_BASE 0x04a8c000ULL
#define BT_SE_SIZE 0x4000ULL
#define BT_SE_IRQ_SPI 330
// gcc-bengal.c: gcc_qupv3_wrap0_s3_clk enable_reg 0x7900c BIT(13), halt_reg 0x1f4d4 (BRANCH_HALT_VOTED);
// gcc_qupv3_wrap0_s3_clk_src cmd_rcgr 0x1f4d8, mnd 16, hid 5, parent_map_1, ftbl_gcc_qupv3_wrap0_s0_clk_src, in
// gcc_dfs_clocks (DFS registered).
#define BT_VOTE_WRAP0_S3 (1u<<13)
#define BT_VOTE_ALL (GT_VOTE_WRAP0_M_AHB|GT_VOTE_WRAP0_S_AHB|GT_VOTE_WRAP0_CORE|GT_VOTE_WRAP0_CORE_2X|BT_VOTE_WRAP0_S3)
#define BT_HALT_S3 0x1f4d4
#define BT_S3_RCG 0x1f4d8
#define BT_RCG_M 0x8
#define BT_RCG_N 0xc
#define BT_RCG_D 0x10
// ftbl entry F(7372800, P_GPLL0_OUT_AUX2, 1, 384, 15625): parent_map_1 select 2, pre_div field 2*1-1 = 1, dual edge.
// Used only when the boot chain left the RCG without DFS (then 115200 * 16 * 4 = 7372800 exactly).
#define BT_RCG_7M37_CFG (1u | (2u << 8) | (2u << 12))
#define BT_RCG_7M37_M 384u
#define BT_RCG_7M37_N 15625u
// Without DFS every rate comes from one of the ftbl entries on GPLL0_OUT_AUX2 (300 MHz) with pre_div 1 and the M/N
// counter (same CFG as above): the lowest one giving an exact divider is programmed (7.3728 MHz for 2400 and 115200,
// 48 MHz for 3000000 at 16x, 96 MHz at 32x).
struct bt_rcg_entry {uint32_t hz, m, n;};
static const struct bt_rcg_entry kBtRcgTable[11] = {
    {7372800, 384, 15625}, {14745600, 768, 15625}, {29491200, 1536, 15625}, {32000000, 8, 75}, {48000000, 4, 25},
    {64000000, 16, 75}, {80000000, 4, 15}, {96000000, 8, 25}, {102400000, 128, 375}, {112000000, 28, 75},
    {117964800, 6144, 15625},
};
#define BT_RCG_AUX2_HZ 300000000ull
static inline uint32_t bt_rcg_n_reg(uint32_t m, uint32_t n) {return ~(n - m) & 0xffffu;}
static inline uint32_t bt_rcg_d_reg(uint32_t m, uint32_t n) {         // clk-rcg2.c 50 % duty: d = clamp(n, m, 2(n-m))
    uint32_t d = n, hi = 2u * (n - m);
    if (d < m) d = m;
    if (d > hi) d = hi;
    return ~d & 0xffffu;
}

// TLMM: pinctrl-bengal.c PINGROUP(8..11, EAST, qup3, ...): EAST = 0x900000 inside the TLMM window 0x400000, function
// slot 1. bengal-pinctrl.dtsi qupv3_se3_4uart_pins, "active" state: gpio8 (CTS) + gpio11 (RX) qup3 2 mA bias-disable,
// gpio9 (RTS) qup3 2 mA pull-down, gpio10 (TX) qup3 2 mA pull-up. Pin order of the 4-wire GENI UART: CTS, RTS, TX, RX.
// gpio87 is qca,bt-sw-ctrl-gpio (bengal.dtsi bt_wcn3990): an input only read here.
#define BT_TLMM_EAST 0x00d00000ULL
#define BT_GPIO_CTS 8
#define BT_GPIO_RTS 9
#define BT_GPIO_TX 10
#define BT_GPIO_RX 11
#define BT_GPIO_SW_CTRL 87
#define BT_FUNC_QUP3 1
#define BT_DRIVE_2MA 0
#define BT_PINS_BASE (BT_TLMM_EAST + BT_GPIO_CTS * 0x1000ULL)   // gpio8..11: four consecutive 0x1000 blocks
#define BT_PINS_SIZE 0x4000ULL
#define BT_SWCTRL_BLOCK (BT_TLMM_EAST + BT_GPIO_SW_CTRL * 0x1000ULL)
static inline uint32_t bt_pin_active_cfg(unsigned gpio) {
    unsigned pull = gpio == BT_GPIO_RTS ? 1u : gpio == BT_GPIO_TX ? 3u : 0u;
    return gt_pin_cfg(BT_FUNC_QUP3, BT_DRIVE_2MA, pull);
}

// Supplies (bengal.dtsi bt_wcn3990, bluetooth-power.c order IO, XO, core, PA). All on PM6125 (SPMI slave 0) and
// RPM-owned (rpm-smd ldoa); L16/L17/L23 are shared with WLAN. Peripheral base of LDO n = 0x4000 + 0x100 * (n - 1)
// (qcom_spmi-regulator.c tables: "l9" 0x4800, "l16" 0x4f00, "l17" 0x5000, "l23" 0x5600). Registers (SPMI common):
// TYPE 0x04, SUBTYPE 0x05, STATUS1 0x08 (bit 7 VREG_OK), VSET_LB/UB 0x40/0x41 (PMIC5 layout: millivolts), MODE 0x45,
// ENABLE 0x46 (bit 7). TWRP regmap dump (analisi/pmic/twrp-pc.txt): L9A type 0d sub 03 st 83 vset 0000 en 80, while
// 0x4f00, 0x5000 and 0x5600 read as unreadable there (no apps-EE APID mapping expected).
#define BT_SID_PM6125 0
#define BT_LDO_TYPE 0x04
#define BT_LDO_SUBTYPE 0x05
#define BT_LDO_STATUS1 0x08
#define BT_LDO_VSET_LB 0x40
#define BT_LDO_VSET_UB 0x41
#define BT_LDO_MODE 0x45
#define BT_LDO_ENABLE 0x46
#define BT_LDO_ON 0x80u
#define BT_LDO_VREG_OK 0x80u
struct bt_rail {const char *name, *role; uint16_t base; uint16_t min_mv, max_mv;};
static const struct bt_rail kBtRails[4] = {
    {"L9A", "io", 0x4800, 1700, 1900},    // qca,bt-vdd-io-voltage-level = <1700000 1900000>
    {"L16A", "xo", 0x4f00, 1700, 1900},   // qca,bt-vdd-xtal-voltage-level = <1700000 1900000>
    {"L17A", "core", 0x5000, 1304, 1304}, // qca,bt-vdd-core-voltage-level = <1304000 1304000>
    {"L23A", "pa", 0x5600, 3000, 3312},   // qca,bt-vdd-pa-voltage-level = <3000000 3312000>
};
static inline unsigned bt_ldo_mv(uint8_t lb, uint8_t ub) {return (unsigned)ub << 8 | lb;}
static inline int bt_ldo_is_on(uint8_t enable, uint8_t status1) {return (enable & BT_LDO_ON) || (status1 & BT_LDO_VREG_OK);}
static inline int bt_ldo_mv_ok(const struct bt_rail *r, unsigned mv) {return mv >= r->min_mv && mv <= r->max_mv;}

// UART clock: msm_geni_serial.c (CONFIG_SERIAL_MSM_GENI_HALF_SAMPLING=y in vendor/bengal_defconfig: 16x oversampling;
// mainline halves the 32x rate from QUP 2.5 on) asks geni_se_clk_freq_match for baud * oversampling over the DFS
// levels (smallest shortfall, DIV_ROUND_UP divider) and programs SER_CLK_CFG = div << 4 | 1.
struct bt_uart_clock {unsigned level, div; uint64_t src_hz; unsigned baud_actual;};
static inline int bt_choose_uart_clock(const uint64_t *level_hz, unsigned n, unsigned baud, unsigned os, struct bt_uart_clock *out) {
    uint64_t want = (uint64_t)baud * os, best = 0;
    int found = 0;
    if (!want) return -1;
    for (unsigned l = 0; l < n; ++l) {
        uint64_t f = level_hz[l];
        if (!f) continue;
        uint64_t div = (f + want - 1) / want;
        if (!div || div > 0xfff) continue;
        uint64_t delta = want - f / div;               // f / div <= want by construction
        if (!found || delta < best) {
            found = 1; best = delta;
            out->level = l; out->div = (unsigned)div; out->src_hz = f; out->baud_actual = (unsigned)(f / div / os);
            if (!delta) break;
        }
    }
    return found ? 0 : -1;
}
// Non-DFS: entry of kBtRcgTable for baud (out->level = table index). 0 ok, -1 none.
static inline int bt_choose_rcg(unsigned baud, unsigned os, struct bt_uart_clock *out) {
    uint64_t hz[sizeof kBtRcgTable / sizeof kBtRcgTable[0]];
    const unsigned n = sizeof kBtRcgTable / sizeof kBtRcgTable[0];
    for (unsigned i = 0; i < n; ++i) hz[i] = kBtRcgTable[i].hz;
    return bt_choose_uart_clock(hz, n, baud, os, out);
}
// Error of the chosen rate in tenths of a percent (positive = slow); UART tolerates roughly +-2 %.
static inline int bt_baud_error_permille(unsigned want, unsigned actual) {
    if (!want) return 1000;
    return (int)(((int64_t)want - (int64_t)actual) * 1000 / (int64_t)want);
}

// Bluetooth firmware on the guamp bluetooth_a partition (ext4, /image), TLV patch header fields product_id/rom_build
// read from the backup (bt-m0 fw.sh). btqca.c names WCN3990 files crbtfw%02x.tlv / crnv%02x.bin from
// rom = ((soc_ver & 0xf00) >> 4) | (soc_ver & 0xf); the Android HAL adds the "ap" (Apache) and "cm" (Comanche) sets.
// cmbtfw12.ver = "BTFM.CMC.1.2.0-00200-QCACHROMZ-2" (the only .ver file: likely the guamp controller).
struct bt_fw_file {const char *tlv, *nvm; uint16_t product, rom_build;};
static const struct bt_fw_file kBtFirmware[8] = {
    {"cmbtfw10.tlv", "cmnv10.bin", 0x000f, 0x0100},
    {"cmbtfw11.tlv", "cmnv11.bin", 0x000f, 0x0101},
    {"cmbtfw12.tlv", "cmnv12.bin", 0x000f, 0x0102},
    {"crbtfw11.tlv", "crnv11.bin", 0x000a, 0x0101},
    {"crbtfw20.tlv", "crnv20.bin", 0x000a, 0x0200},
    {"crbtfw21.tlv", "crnv21.bin", 0x000a, 0x0201},   // byte-identical to apbtfw11.tlv
    {"apbtfw10.tlv", "apnv10.bin", 0x000a, 0x0200},
    {"apbtfw11.tlv", "apnv11.bin", 0x000a, 0x0201},
};
#endif
