// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp (Moto G9 Play, SM6115 / bengal) touch controller board facts and the pure helpers around them. Every constant
// states its source. Nothing here touches hardware; GuampTouch.cpp owns the MMIO.
#ifndef GUAMP_TOUCH_BOARD_H
#define GUAMP_TOUCH_BOARD_H
#include <stdint.h>

// Board DT (guamp boot_b DTB + dtbo_b overlay fragment@31): himax_0flash_ts@0 on qupv3_se2_spi = /soc/spi@4a88000,
// compatible "qcom,spi-geni", reg 0x4a88000 size 0x4000, wrapper qcom,qupv3_0_geni_se@4ac0000 (reg 0x4ac0000 0x2000,
// iommus <&apps_smmu 0xe3 0>), interrupts <0 0x149 4> (SPI 329, level high).
#define GT_QUP_WRAPPER_BASE 0x04ac0000ULL
#define GT_SE_BASE 0x04a88000ULL
#define GT_SE_SIZE 0x4000ULL
#define GT_QUP_SMMU_SID 0xe3
#define GT_APPS_SMMU_BASE 0x0c600000ULL      // apps-smmu@0xc600000 reg <0xc600000 0x80000 ...>
#define GT_APPS_SMMU_SIZE 0x80000ULL
// gcc-sm6115.c (Linux mainline): GCC at 0x1400000 (NativeEmmc uses its SDCC1 branches at +0x38000).
#define GT_GCC_BASE 0x01400000ULL
#define GT_GCC_VOTE_ENABLE 0x7900c            // enable_reg of every QUPv3 wrap0 branch below
#define GT_GCC_GPLL0_ENABLE 0x79000           // gpll0 enable_reg, BIT(0)
#define GT_GCC_GPLL0_MODE 0x0                 // gpll0 offset 0x0, bit 31 = lock
#define GT_VOTE_WRAP0_M_AHB (1u<<6)           // gcc_qupv3_wrap_0_m_ahb_clk, halt 0x1f004 (hwcg bit 1)
#define GT_VOTE_WRAP0_S_AHB (1u<<7)           // gcc_qupv3_wrap_0_s_ahb_clk, halt 0x1f008 (hwcg bit 1)
#define GT_VOTE_WRAP0_CORE (1u<<8)            // gcc_qupv3_wrap0_core_clk, halt 0x1f00c
#define GT_VOTE_WRAP0_CORE_2X (1u<<9)         // gcc_qupv3_wrap0_core_2x_clk, halt 0x1f014
#define GT_VOTE_WRAP0_S2 (1u<<12)             // gcc_qupv3_wrap0_s2_clk, halt 0x1f3a4
#define GT_VOTE_ALL (GT_VOTE_WRAP0_M_AHB|GT_VOTE_WRAP0_S_AHB|GT_VOTE_WRAP0_CORE|GT_VOTE_WRAP0_CORE_2X|GT_VOTE_WRAP0_S2)
#define GT_HALT_M_AHB 0x1f004
#define GT_HALT_S_AHB 0x1f008
#define GT_HALT_CORE 0x1f00c
#define GT_HALT_CORE_2X 0x1f014
#define GT_HALT_S2 0x1f3a4
#define GT_CBCR_CLK_OFF (1u<<31)
// gcc_qupv3_wrap0_s2_clk_src: cmd_rcgr 0x1f3a8, mnd_width 16, hid_width 5, parent_map_1 {TCXO 0, GPLL0 1, GPLL0_AUX2 2,
// GPLL6 4}; registered for DFS (clk-rcg2.c SE_* offsets below).
#define GT_S2_RCG 0x1f3a8
#define GT_RCG_CMD 0x0                        // bit 0 update, bit 31 root off
#define GT_RCG_CFG 0x4                        // src 10:8, mode 13:12, hid 4:0
#define GT_RCG_DFSR_CMD 0x14                  // bit 0 DFS enable
#define GT_RCG_PERF_CFG(l) (0x1c+4*(l))
#define GT_RCG_PERF_M(l) (0x5c+4*(l))
#define GT_RCG_PERF_N(l) (0x9c+4*(l))
#define GT_DFS_LEVELS 8
// TLMM (DT: pinctrl@400000 reg <0x400000 0xc00000>, interrupts <0 0xe3 4> = SPI 227; pinctrl-sm6115.c: one register
// block per pin at 0x1000 * gpio inside its tile, mux_bit 2, drv_bit 6, oe_bit 9, pull_bit 0, io in bit 0 / out bit 1;
// intr_cfg +8 (enable 0, polarity 1, detection 3:2, raw status 4, target 7:5 = 3 for the apps processor), status +0xc).
// All of 6, 7, 66, 67, 71, 80 are in the WEST tile; the DT gives one reg window, so the tile offset is in the window:
// sm6115.dtsi tlmm reg-names "west" 0x500000, "south" 0x900000, "east" 0xd00000 (check on the device: stage 1 prints
// the control registers of the six pins so a wrong tile reads as garbage, not as a pin configuration).
#define GT_TLMM_BASE 0x00400000ULL
#define GT_TLMM_WEST 0x00500000ULL
#define GT_GPIO_STRIDE 0x1000
#define GT_TLMM_SUMMARY_SPI 227
#define GT_GPIO_SPI_A 6                       // qupv3_se2_spi_active: gpio6, gpio7, gpio71, gpio80, function qup2
#define GT_GPIO_SPI_B 7
#define GT_GPIO_SPI_C 71
#define GT_GPIO_SPI_D 80
#define GT_FUNC_QUP2 1                        // PINGROUP(6/7/71/80, WEST, qup2, ...): function slot 1
#define GT_DRIVE_6MA 2                        // drive-strength = <6>: (mA / 2) - 1
#define GT_GPIO_RESET 66                      // himax,rst-gpio = <&tlmm 0x42 0>
#define GT_GPIO_IRQ 67                        // himax,irq-gpio = <&tlmm 0x43 0x2008>, interrupts <0x43 2> (falling edge)
#define GT_SPI_MODE 1                         // spi-cpha, no spi-cpol
#define GT_SPI_MAX_HZ 10000000u               // spi-max-frequency = <0x989680>
#define GT_PANEL_X 720                        // himax,panel-coords = <0 0x2d0 0 0x640>
#define GT_PANEL_Y 1600

static inline uint32_t gt_pin(unsigned gpio) {return (uint32_t)gpio * GT_GPIO_STRIDE;}
static inline uint32_t gt_pin_cfg(unsigned func, unsigned drive_code, unsigned pull) {
    return (pull & 3u) | (func & 15u) << 2 | (drive_code & 7u) << 6;       // pull 0 none, 1 down, 3 up
}
#define GT_PIN_OE (1u<<9)

// Source rates of parent_map_1 by select value. GPLL0 is 600 MHz on this family (gpll0_out_aux2 = /2 = 300 MHz: the
// 75 MHz entry is aux2 / 4, 100 MHz aux2 / 3); GPLL6 main 384 MHz (128 MHz = /3).
static inline uint64_t gt_src_hz(unsigned sel) {
    return sel == 0 ? 19200000ULL : sel == 1 ? 600000000ULL : sel == 2 ? 300000000ULL : sel == 4 ? 384000000ULL : 0;
}
static inline uint64_t gt_level_hz(uint32_t cfg, uint32_t mreg, uint32_t nreg) {
    unsigned src = (cfg >> 8) & 7, mode = (cfg >> 12) & 3, hid = cfg & 0x1f;
    uint64_t rate = gt_src_hz(src);
    if (hid) {rate *= 2; rate /= hid + 1;}
    if (mode) {uint32_t m = mreg & 0xffff, n = ((~nreg) & 0xffff) + m; if (n) {rate *= m; rate /= n;}}
    return rate;
}
struct gt_clock_choice {unsigned level, div; uint64_t sclk_hz, actual_hz;};
// The level giving the highest SPI rate not above target with div = ceil(sclk / (os * target)). 0 on success.
static inline int gt_choose_clock(const uint64_t *level_hz, unsigned n, unsigned os, unsigned target, struct gt_clock_choice *out) {
    int found = 0;
    for (unsigned l = 0; l < n; ++l) {
        uint64_t f = level_hz[l];
        if (!f) continue;
        uint64_t d = (f + (uint64_t)os * target - 1) / ((uint64_t)os * target);
        if (d < 1) d = 1;
        if (d > 0xfff) continue;
        uint64_t actual = f / (os * d);
        if (actual > target) continue;
        if (!found || actual > out->actual_hz || (actual == out->actual_hz && d < out->div)) {
            out->level = l; out->div = (unsigned)d; out->sclk_hz = f; out->actual_hz = actual; found = 1;
        }
    }
    return found ? 0 : -1;
}
#endif
