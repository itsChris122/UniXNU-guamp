// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp (Moto G9 Play, SM6115/bengal) audio output hardware: board facts and the register sequences of the speaker
// amplifier (aw882xx), the LPASS LPI i2s1 pins, the bolero RX macro, the wcd937x headphone/earpiece paths with the
// class-H FSM, and the wcd937x MBHC (3.5 mm jack). Pure code: every
// access goes through gah_ops, so tests/audio_hw_host.cpp runs the same sequences against simulated chips. The kernel
// glue (GuampAudioHw.inc) owns MMIO, I2C, GPIO and logging. Every constant states its source; paths are relative to
// sorgenti/guamp/android_kernel_motorola_sm6225 (K), K/techpack/audio (TA), ~/moto/guamp.dts (DT), ~/moto/dtbo/0.dts
// (DTBO0), vendor-estratto (VEND). Sequences follow the Linux drivers (studied, not copied).
#ifndef GUAMP_AUDIO_HW_CORE_H
#define GUAMP_AUDIO_HW_CORE_H
#include <stdint.h>
#include <stddef.h>

// ---- Speaker amplifier: Awinic aw882xx_smartpa@34 (DTBO0 fragment@58: reg 0x34, reset-gpio <&tlmm 0x61 0>, no IRQ, no
// supply) on qupv3_se1_i2c = /soc/i2c@4a84000 (DT: QUP0 SE1, gpio4/5 func qup1, 2 mA, bias-disable; no clk-freq-out,
// so Linux runs it at 400 kHz). Same bus: fsa4480@42, pm8008@8, nq@28. dtbo 3 (EVT, board-id 0xA000) has fs16xx@34
// instead; the vendor ships aw882xx_*.bin, and the chip id read decides.
#define GAH_SE1_BASE 0x04a84000ULL
#define GAH_SE1_SIZE 0x4000ULL
#define GAH_QUP0_WRAPPER 0x04ac0000ULL
#define GAH_GCC_VOTE_SE1 (1u << 11)           // gcc-bengal.c gcc_qupv3_wrap0_s1_clk: enable_reg 0x7900c BIT(11)
#define GAH_GCC_HALT_SE1 0x1f274              //   halt_reg 0x1f274
#define GAH_GCC_RCG_SE1 0x1f278               // gcc_qupv3_wrap0_s1_clk_src cmd_rcgr 0x1f278 (DFS, parent_map_1)
#define GAH_GPIO_I2C_SDA 4                    // PINGROUP(4, WEST, qup1, ...): function slot 1
#define GAH_GPIO_I2C_SCL 5
#define GAH_FUNC_QUP1 1
#define GAH_GPIO_AMP_RESET 97                 // PINGROUP(97, WEST, ...); active high (aw882xx_hw_reset: 0, 1 ms, 1, 2 ms)
#define GAH_GPIO_WCD_RESET 92                 // msm_cdc_pinctrl@92 wcd937x_reset_active output-high 16 mA (DT:7525)
#define GAH_TLMM_WEST_SIZE 0x62000ULL         // gpio 0..97 of the WEST tile at 0x500000 (stride 0x1000)
#define GAH_AW_ADDR 0x34
#define GAH_FSA_ADDR 0x42

// aw882xx registers (TA/asoc/codecs/awinic/aw882xx_reg.h; 8-bit address, 16-bit big-endian value)
#define AW_REG_ID 0x00
#define AW_REG_SYSST 0x01
#define AW_REG_SYSCTRL 0x04
#define AW_REG_SYSCTRL2 0x05
#define AW_REG_I2SCTRL 0x06
#define AW_REG_HAGCCFG4 0x0c
#define AW_REG_VTMCTRL3 0x56
#define AW_REG_EFRH 0x78
#define AW_REG_EFRM1 0x7a
#define AW_ID 0x1852                          // aw882xx.h AW882XX_ID
#define AW_PWDN 0x0001                        // SYSCTRL bit 0: 1 = power down
#define AW_HMUTE 0x0010                       // SYSCTRL2 bit 4: 1 = mute
#define AW_SYSST_OK 0x0011                    // AW882XX_SYSST_CHECK: CLKS (bit 4) | PLLS (bit 0)
#define AW_SYSST_TRIES 10                     // AW882XX_SYSST_CHECK_MAX, 2 ms apart
#define AW_ID_TRIES 5                         // AW_READ_CHIPID_RETRIES, AW_READ_CHIPID_RETRY_DELAY 5 ms
#define AW_ATT_MAX 0xff                       // HAGCCFG4[15:8]: high nibble 6 dB steps, low nibble 0.5 dB steps

// VEND/firmware/aw882xx_spk_reg.bin (144 bytes = 36 records {addr u16 LE, value u16 LE}), the speaker profile the
// driver writes at cold start (aw882xx_reg_container_update). Contains SYSCTRL 0x4440 (PWDN 0) and I2SCTRL 0x1408
// (left channel, 48 kHz, 16 bit, BCK 32 fs).
static const uint8_t kAwSpkProfile[144] = {
    0x03,0x00,0xbf,0xef, 0x04,0x00,0x40,0x44, 0x05,0x00,0x28,0x00, 0x06,0x00,0x08,0x14,
    0x07,0x00,0x61,0x03, 0x08,0x00,0x0c,0x21, 0x09,0x00,0x40,0x39, 0x0a,0x00,0x30,0x00,
    0x0b,0x00,0xe0,0x01, 0x0c,0x00,0x64,0x00, 0x0d,0x00,0xdb,0xa0, 0x0e,0x00,0x16,0x27,
    0x0f,0x00,0x00,0x75, 0x20,0x00,0x01,0x00, 0x38,0x00,0x60,0x00, 0x50,0x00,0x82,0x02,
    0x52,0x00,0xc6,0x20, 0x53,0x00,0x40,0xa0, 0x54,0x00,0x19,0xff, 0x55,0x00,0xd8,0x00,
    0x56,0x00,0xda,0x1e, 0x59,0x00,0x0c,0x00, 0x5a,0x00,0x00,0x21, 0x60,0x00,0x0b,0x0c,
    0x61,0x00,0x79,0x63, 0x62,0x00,0xc1,0x04, 0x63,0x00,0x0f,0x26, 0x64,0x00,0xc3,0x15,
    0x65,0x00,0x82,0x3d, 0x66,0x00,0x07,0x7f, 0x67,0x00,0x02,0x00, 0x68,0x00,0x02,0x30,
    0x69,0x00,0xa0,0x04, 0x6a,0x00,0x78,0xa0, 0x6b,0x00,0x84,0x88, 0x70,0x00,0x00,0xff,
};

// ---- LPASS LPI pinctrl (DT lpi_pinctrl: reg <0x0a7c0000>, gpio n at +0x1000*n; TA/soc/pinctrl-lpi.c: CTL +0 pull[1:0]
// func[5:2] drive[8:6] = mA/2-1, "output-high" sets bit 9). i2s1 = gpio6 SCK, 7 WS, 8 SD0 (to the amplifier), 9 SD1
// (I/V feedback): active func2 8 mA bias-disable output-high, sleep func2 2 mA pull-down input.
#define GAH_LPI_BASE 0x0a7c0000ULL
#define GAH_LPI_SIZE 0xa000ULL
#define GAH_LPI_I2S1_FIRST 6
#define GAH_LPI_I2S1_COUNT 4
#define GAH_LPI_ACTIVE 0x2c8u                 // 0x200 | (8/2-1)<<6 | 2<<2 | 0
#define GAH_LPI_SLEEP 0x009u                  // (2/2-1)<<6 | 2<<2 | 1

// ---- Bolero RX macro (DTBO0 rx-macro@a600000, bolero-version 5) and the shared FS generator (bolero-clk-rsc
// fs-gen-sequence <0x3000 1 0x3004 1 0x3080 2>: VA macro registers at 0x0a730000). Register numbers are bolero's
// (TA/asoc/codecs/bolero/bolero-cdc-registers.h): RX 0x1000.. -> 0x0a600000 + (reg - 0x1000), VA 0x3000.. ->
// 0x0a730000 + (reg - 0x3000). Clocks (DTBO0 fragment@26, set through AFE CLOCK_SET by the DSP side):
// rx_core 0x30e and rx_npl 0x30f at 22.5792 MHz.
#define GAH_RXM_BASE 0x0a600000ULL
#define GAH_VAM_BASE 0x0a730000ULL
#define GAH_LPASS_CLK_RX_CORE 0x30e
#define GAH_LPASS_CLK_RX_NPL 0x30f
#define GAH_LPASS_CLK_RX_HZ 22579200u
#define RXM_MCLK_CONTROL 0x1100
#define RXM_FS_CNT_CONTROL 0x1104
#define RXM_INP_MUX_INT0_CFG0 0x1180           // + 8 per interpolator; bits 3:0 INP0: 5 = RX0, 6 = RX1 (rx_prim_mix_text)
#define RXM_CLSH_CRC 0x1200
#define RXM_CLSH_DECAY_CTRL 0x1208
#define RXM_CLSH_HPH_V_PA 0x120c
#define RXM_CLSH_K1_MSB 0x121c
#define RXM_CLSH_K1_LSB 0x1220
#define RXM_PATH_CTL(n) (0x1400 + 0x80 * (n)) // RX_MACRO_RX_PATH_OFFSET 0x80; rate 3:0 (4 = 48 kHz), 0x10 PGA mute, 0x20 clk, 0x40 reset
#define RXM_PATH_CFG0(n) (0x1404 + 0x80 * (n))
#define RXM_PATH_CFG1(n) (0x1408 + 0x80 * (n)) // bit 0 DEM mux: 1 = CLSH_DSM_OUT; 0x10 sidetone/path enable
#define RXM_PATH_CFG2(n) (0x140c + 0x80 * (n))
#define RXM_PATH_CFG3(n) (0x1410 + 0x80 * (n))
#define RXM_VOL_CTL(n) (0x1414 + 0x80 * (n))  // signed dB, -84..+40 ("RX_RX0 Digital Volume")
#define RXM_PATH_SEC7(n) (0x1434 + 0x80 * (n))
#define RXM_DSM_CTL(n) (0x1440 + 0x80 * (n))
#define RXM_PATH_MIX_CTL(n) (0x1418 + 0x80 * (n)) // mix path, PGA mute 0x10
#define RXM_TOP_HPHL_COMP_LUT 0x1028           // bit 7: LUT bypass (rx_macro_hphdelay_lutbypass)
#define RXM_TOP_HPHR_COMP_LUT 0x103c
#define RXM_COMP0_CTL7 0x181c                  // COMPANDER0_CTL7 / COMPANDER1_CTL7
#define RXM_COMP1_CTL7 0x185c
#define RXM_SWR_CONTROL 0x1108                 // CLK_RST_CTRL_SWR_CONTROL: bit 0 RX SoundWire clock, bit 1 reset
#define VAM_MCLK_CONTROL 0x3000
#define VAM_FS_CNT_CONTROL 0x3004
#define VAM_TOP_CSR_TOP_CFG0 0x3080

// ---- wcd937x analog codec (DTBO0 wcd937x-codec, split-codec; registers reachable ONLY over SoundWire, through the
// TX slave 0x0a_01170223 on the VA master (regmap on tx_swr_dev); the RX slave 0x0a_01170224 on rx_swr_master
// 0x0a610000 carries only the HPH/EAR PCM (GuampAudioRxSwr.h); supplies L9A 1.8 V (vdd-rxtx, vddpx), L14A 1.8 V (buck);
// reset gpio92). TA/asoc/codecs/wcd937x/wcd937x-registers.h:
#define WCD_ANA_BIAS 0x3001
#define WCD_ANA_RX_SUPPLIES 0x3008
#define WCD_ANA_HPH 0x3009                    // 0x20 HPHL PA, 0x10 HPHR PA
#define WCD_BIAS_VBG_FINE_ADJ 0x3029
#define WCD_MICB1_TEST_CTL_1 0x306b
#define WCD_MICB1_TEST_CTL_2 0x306c
#define WCD_MICB2_TEST_CTL_1 0x306e
#define WCD_MICB2_TEST_CTL_2 0x306f
#define WCD_MICB3_TEST_CTL_1 0x3071
#define WCD_MICB3_TEST_CTL_2 0x3072
#define WCD_HPH_L_EN 0x30d3                   // bits 4:0 gain (inverted, 1.5 dB steps from -30 dB), 0x20 = no compander
#define WCD_HPH_R_EN 0x30d6
#define WCD_HPH_RDAC_CLK_CTL1 0x30d9
#define WCD_HPH_SURGE_EN 0x30e2
#define WCD_SLEEP_CTL 0x3103
#define WCD_LDORXTX_CONFIG 0x312a
#define WCD_RDAC_GAIN_CTL 0x3132
#define WCD_RDAC_HD2_CTL_L 0x3133
#define WCD_RDAC_HD2_CTL_R 0x3136
#define WCD_HPH_TIMER1 0x313a
#define WCD_DIG_CHIP_ID0 0x3401
#define WCD_ANA_EAR 0x300a                    // bit 7 EAR PA (DAPM "EAR PGA")
#define WCD_ANA_EAR_COMP_CTL 0x300b           // bit 7 compander off, [6:2] "EAR PA GAIN"
#define WCD_ANA_MBHC_MECH 0x3014              // 7 L_DET_EN, 5 MECH_DETECTION_TYPE, 4 HPHL_PLUG_TYPE, 3 GND_PLUG_TYPE, 2 pull-up comp, 0 100k
#define WCD_ANA_MBHC_ELECT 0x3015             // 7 FSM_EN, 6:4 BTN_ISRC, 3 ELECT_DETECTION_TYPE, 2:1 SCHMT_ISRC, 0 bias
#define WCD_ANA_MBHC_RESULT_3 0x3019          // 4 SWCH_LEVEL_REMOVE (1 = no plug)
#define WCD_ANA_MICB2 0x3023                  // 7:6 enable (1 = on), 5:0 VOUT
#define WCD_ANA_MICB2_RAMP 0x3024
#define WCD_CLASSH_MODE_2 0x3098
#define WCD_CLASSH_MODE_3 0x3099
#define WCD_FLYBACK_EN 0x30a4
#define WCD_FLYBACK_VNEG_CTRL_1 0x30a5
#define WCD_FLYBACK_VNEG_CTRL_4 0x30a8
#define WCD_FLYBACK_VNEGDAC_CTRL_2 0x30af
#define WCD_RX_BIAS_HPH_LOWPOWER 0x30bf
#define WCD_RX_BIAS_FLYB_BUFF 0x30c7
#define WCD_HPH_PA_CTL1 0x30d1
#define WCD_EAR_EN_REG 0x30e9
#define WCD_MBHC_NEW_CTL_1 0x3120             // 7 clock, 4 ADC_MODE, 3 ADC_EN, 1:0 BTN_DBNC
#define WCD_MBHC_NEW_CTL_2 0x3121             // 6:4 MUX_CTL, 3:2 moisture rref, 1:0 HS_VREF
#define WCD_MBHC_NEW_PLUG_DETECT_CTL 0x3122   // 3:0 INSREM_DBNC
#define WCD_MBHC_NEW_FSM_STATUS 0x3125        // 7 ADC_TIMEOUT, 6 ADC_COMPLETE
#define WCD_MBHC_NEW_ADC_RESULT 0x3126
#define WCD_HPH_NEW_INT_RDAC_HD2_CTL_L 0x3133
#define WCD_HPH_NEW_INT_RDAC_HD2_CTL_R 0x3136
#define WCD_HPH_NEW_INT_PA_MISC2 0x3138
#define WCD_MBHC_NEW_INT_MECH_DET_CURRENT 0x31b1
#define WCD_DIG_EAR_PATH_CTL 0x3450           // bit 0 RDAC3_MUX: 0 RX1, 1 RX3
#define WCD_DIG_INTR_MASK_0 0x346b
#define WCD_DIG_INTR_STATUS_0 0x346e          // bit 4 MBHC_SW_DET
#define WCD_DIG_INTR_CLEAR_0 0x3471
#define WCD_DIG_INTR_LEVEL_0 0x3474
#define WCD_EAR_GAIN_MAX 16                   // "EAR PA GAIN" G_M18_DB
#define WCD_DIG_ANA_CLK_CTL 0x3408
#define WCD_DIG_DIG_CLK_CTL 0x3409
#define WCD_DIG_RX0_CTL 0x340d
#define WCD_DIG_RX1_CTL 0x340e
#define WCD_DIG_RX2_CTL 0x340f
#define WCD_DIG_COMP_CTL_0 0x3414
#define WCD_DIG_HPH_GAIN_CTL 0x344e
#define WCD_DIG_PDM_WD_CTL0 0x3465
#define WCD_DIG_PDM_WD_CTL1 0x3466
#define WCD_EFUSE_16 0x34c0
#define WCD_EFUSE_17 0x34c1
#define WCD_EFUSE_29 0x34cd
#define WCD_HPH_GAIN_DEFAULT 4                // mixer_paths.xml "HPHL Volume" 16 -> register 20 - 16 = 4 (-6 dB)

#define GAH_ERR_NODEV (-19)
#define GAH_ERR_IO (-5)
#define GAH_ERR_INVAL (-22)
#define GAH_ERR_NOTSUP (-95)

struct gah_ops {
    void *ctx;
    int (*i2c_w16)(void *ctx, uint8_t dev, uint8_t reg, uint16_t v);
    int (*i2c_r16)(void *ctx, uint8_t dev, uint8_t reg, uint16_t *v);
    void (*gpio_out)(void *ctx, unsigned gpio, int high);
    uint32_t (*lpi_rd)(void *ctx, uint32_t off);
    void (*lpi_wr)(void *ctx, uint32_t off, uint32_t v);
    uint32_t (*cdc_rd)(void *ctx, uint32_t bolero_reg);   // RX/VA macro by bolero register number
    void (*cdc_wr)(void *ctx, uint32_t bolero_reg, uint32_t v);
    int (*wcd_rd)(void *ctx, uint16_t reg, uint8_t *v);  // SoundWire slave; null = no SoundWire master yet
    int (*wcd_wr)(void *ctx, uint16_t reg, uint8_t v);
    void (*sleep_ms)(void *ctx, unsigned ms);
    void (*delay_us)(void *ctx, unsigned us);
    void (*note)(void *ctx, const char *step, int rc, uint32_t a, uint32_t b);   // one log line per step
    // RX SoundWire data port of wcd937x slave port 0 (HPH_L ch 1, HPH_R ch 2 -> master port 1): connect + datapath
    // with bank switch (on), or disable + datapath off. null = no RX SoundWire link (-95).
    int (*swr_rx)(void *ctx, int on, uint8_t slave_ch_mask);
};

static inline void gah_note(const struct gah_ops *o, const char *s, int rc, uint32_t a, uint32_t b) {
    if (o->note) o->note(o->ctx, s, rc, a, b);
}

// ---- volume helpers -----------------------------------------------------------------------------------------------
// Attenuation in centibel (>= 0) -> HAGCCFG4[15:8]. Linux aw882xx_volume_set: ((dB / 6) << 4) + (dB % 6) * 2, i.e.
// the low nibble counts 0.5 dB. Rounded towards more attenuation; above 95.5 dB the maximum code.
static inline uint8_t gah_aw_att_code(uint32_t att_cb) {
    uint32_t half = (att_cb + 49) / 50;
    uint32_t hi = half / 12, lo = half % 12;
    if (hi > 15) return AW_ATT_MAX;
    return (uint8_t)((hi << 4) | lo);
}
static inline uint32_t gah_aw_att_cb(uint8_t code) { return ((code >> 4) * 12u + (code & 15u)) * 50u; }
// Requested level (centibel, 0 = full scale, negative = quieter) -> attenuation, never below floor_cb.
static inline uint32_t gah_att_from_level(int32_t level_cb, uint32_t floor_cb) {
    uint32_t att = level_cb >= 0 ? 0u : (uint32_t)(-(int64_t)level_cb);
    return att < floor_cb ? floor_cb : att;
}
// Linux aw882xx_set_vcalb: VCAL_FACTOR (1 << 13) * icalk / vcalk, icalk/vcalk = 1000 + signed 10-bit eFuse slopes.
static inline uint16_t gah_aw_vcalb(uint16_t efrm1, uint16_t efrh) {
    uint16_t ri = efrm1 & 0x03ff, rv = efrh & 0x03ff;
    if (ri & 0x0200) ri |= 0xfc00;
    if (rv & 0x0200) rv |= 0xfc00;
    int icalk = 1000 + (int16_t)ri, vcalk = 1000 + (int16_t)rv;
    return (uint16_t)(int)((1 << 13) * icalk / vcalk);
}

// ---- aw882xx ---------------------------------------------------------------------------------------------------
static inline int gah_aw_bits(const struct gah_ops *o, uint8_t reg, uint16_t keep_mask, uint16_t set) {
    uint16_t v = 0;
    int r = o->i2c_r16(o->ctx, GAH_AW_ADDR, reg, &v);
    if (r) return r;
    return o->i2c_w16(o->ctx, GAH_AW_ADDR, reg, (uint16_t)((v & keep_mask) | set));
}

// aw882xx_hw_reset + aw882xx_read_chipid. 0 and *id = 0x1852, or -19 (other id: fs16xx board?) / I2C error.
static inline int gah_aw_probe(const struct gah_ops *o, uint16_t *id) {
    o->gpio_out(o->ctx, GAH_GPIO_AMP_RESET, 0);
    o->sleep_ms(o->ctx, 1);
    o->gpio_out(o->ctx, GAH_GPIO_AMP_RESET, 1);
    o->sleep_ms(o->ctx, 2);
    gah_note(o, "aw reset gpio97 low 1 ms, high 2 ms", 0, 0, 0);
    int r = GAH_ERR_NODEV;
    uint16_t v = 0;
    for (unsigned t = 0; t < AW_ID_TRIES; t++) {
        r = o->i2c_r16(o->ctx, GAH_AW_ADDR, AW_REG_ID, &v);
        if (r) { gah_note(o, "aw chip id read (i2c 0x34 reg 0x00)", r, t, 0); return r; }   // Linux: no retry on I2C error
        if (v == AW_ID) break;
        r = GAH_ERR_NODEV;
        o->sleep_ms(o->ctx, 5);
    }
    if (v == AW_ID) r = 0;
    if (id) *id = v;
    gah_note(o, "aw chip id (want 0x1852)", r, v, 0);
    return r;
}

// Cold start (aw882xx_reg_loaded): the speaker profile, then volume, mute and VCALB. Differences from Linux, because
// here the I2S clocks are not running yet: SYSCTRL is written with PWDN = 1 and SYSCTRL2 with HMUTE = 1 (the profile
// has 0 in both), and the gain byte of HAGCCFG4 is att_code instead of the profile's 0 dB (Linux writes -90 dB).
static inline int gah_aw_cold(const struct gah_ops *o, uint8_t att_code) {
    unsigned n = 0;
    for (size_t i = 0; i + 3 < sizeof(kAwSpkProfile); i += 4) {
        uint8_t reg = kAwSpkProfile[i];
        uint16_t val = (uint16_t)(kAwSpkProfile[i + 2] | (kAwSpkProfile[i + 3] << 8));
        if (reg == AW_REG_SYSCTRL) val |= AW_PWDN;
        if (reg == AW_REG_SYSCTRL2) val |= AW_HMUTE;
        if (reg == AW_REG_HAGCCFG4) val = (uint16_t)((val & 0x00ff) | (att_code << 8));
        int r = o->i2c_w16(o->ctx, GAH_AW_ADDR, reg, val);
        if (r) { gah_note(o, "aw profile write failed at reg", r, reg, n); return r; }
        n++;
    }
    gah_note(o, "aw profile aw882xx_spk_reg.bin written (records, PWDN 1 HMUTE 1)", 0, n, 0);
    uint16_t m1 = 0, h = 0;
    int r = o->i2c_r16(o->ctx, GAH_AW_ADDR, AW_REG_EFRM1, &m1);
    if (!r) r = o->i2c_r16(o->ctx, GAH_AW_ADDR, AW_REG_EFRH, &h);
    uint16_t vcalb = gah_aw_vcalb(m1, h);
    if (!r) r = o->i2c_w16(o->ctx, GAH_AW_ADDR, AW_REG_VTMCTRL3, vcalb);
    gah_note(o, "aw vcalb (efuse 0x7a/0x78 -> VTMCTRL3)", r, vcalb, (uint32_t)m1 << 16 | h);
    gah_note(o, "aw gain (att code, att cb)", 0, att_code, gah_aw_att_cb(att_code));
    return r;
}

// aw882xx_start without the DSP calibration parts: PWDN 0, SYSST until PLLS+CLKS (10 x 2 ms), then HMUTE 0; on
// failure mute + power down again (as Linux). *sysst = last SYSST read.
static inline int gah_aw_start(const struct gah_ops *o, uint16_t *sysst) {
    int r = gah_aw_bits(o, AW_REG_SYSCTRL, (uint16_t)~AW_PWDN, 0);
    gah_note(o, "aw PWDN 0", r, 0, 0);
    if (r) return r;
    uint16_t st = 0;
    int ok = 0;
    unsigned t = 0;
    for (; t < AW_SYSST_TRIES; t++) {
        if (!o->i2c_r16(o->ctx, GAH_AW_ADDR, AW_REG_SYSST, &st) && (st & AW_SYSST_OK) == AW_SYSST_OK) { ok = 1; break; }
        o->sleep_ms(o->ctx, 2);
    }
    if (sysst) *sysst = st;
    gah_note(o, ok ? "aw SYSST PLLS+CLKS locked (I2S clocks reach the amp)" : "aw SYSST not locked: no BCLK/WS at the amp",
             ok ? 0 : GAH_ERR_IO, st, t);
    if (!ok) {
        gah_aw_bits(o, AW_REG_SYSCTRL2, (uint16_t)~AW_HMUTE, AW_HMUTE);
        gah_aw_bits(o, AW_REG_SYSCTRL, (uint16_t)~AW_PWDN, AW_PWDN);
        return GAH_ERR_IO;
    }
    r = gah_aw_bits(o, AW_REG_SYSCTRL2, (uint16_t)~AW_HMUTE, 0);
    gah_note(o, "aw HMUTE 0 (unmuted)", r, 0, 0);
    return r;
}

// aw882xx_stop: mute, then power down.
static inline int gah_aw_stop(const struct gah_ops *o) {
    int r1 = gah_aw_bits(o, AW_REG_SYSCTRL2, (uint16_t)~AW_HMUTE, AW_HMUTE);
    int r2 = gah_aw_bits(o, AW_REG_SYSCTRL, (uint16_t)~AW_PWDN, AW_PWDN);
    gah_note(o, "aw HMUTE 1, PWDN 1", r1 ? r1 : r2, 0, 0);
    return r1 ? r1 : r2;
}

static inline int gah_aw_set_att(const struct gah_ops *o, uint8_t att_code) {
    int r = gah_aw_bits(o, AW_REG_HAGCCFG4, 0x00ff, (uint16_t)(att_code << 8));
    gah_note(o, "aw gain (att code, att cb)", r, att_code, gah_aw_att_cb(att_code));
    return r;
}

static inline int gah_aw_mute(const struct gah_ops *o, int mute) {
    int r = gah_aw_bits(o, AW_REG_SYSCTRL2, (uint16_t)~AW_HMUTE, mute ? AW_HMUTE : 0);
    gah_note(o, mute ? "aw HMUTE 1" : "aw HMUTE 0", r, 0, 0);
    return r;
}

// ---- LPI i2s1 pins ---------------------------------------------------------------------------------------------
static inline void gah_lpi_i2s1(const struct gah_ops *o, int active) {
    for (unsigned i = 0; i < GAH_LPI_I2S1_COUNT; i++) {
        uint32_t off = 0x1000u * (GAH_LPI_I2S1_FIRST + i);
        uint32_t old = o->lpi_rd(o->ctx, off);
        o->lpi_wr(o->ctx, off, active ? GAH_LPI_ACTIVE : GAH_LPI_SLEEP);
        gah_note(o, active ? "lpi i2s1 pin active (gpio, old ctl)" : "lpi i2s1 pin sleep (gpio, old ctl)", 0,
                 GAH_LPI_I2S1_FIRST + i, old);
    }
}

// ---- bolero RX macro ---------------------------------------------------------------------------------------------
static inline void gah_cdc_bits(const struct gah_ops *o, uint32_t reg, uint32_t mask, uint32_t val) {
    uint32_t v = o->cdc_rd(o->ctx, reg);
    o->cdc_wr(o->ctx, reg, (v & ~mask) | (val & mask));
}

// bolero fs-gen: not here, one refcounted owner for RX and TX = guamp_bolero_fsgen (GuampMicDev.inc, guampmic::Codec::fsGen).

// rx_macro_mclk_enable after the clock request (rx core/npl running): MCLK on, FS counter on; plus rx_macro_reg_init
// (written by Linux at probe into the register cache and synced here). Off: the reverse.
static inline void gah_rxm_mclk(const struct gah_ops *o, int on) {
    if (on) {
        gah_cdc_bits(o, RXM_MCLK_CONTROL, 0x01, 0x01);
        gah_cdc_bits(o, RXM_MCLK_CONTROL, 0x02, 0x02);
        gah_cdc_bits(o, RXM_FS_CNT_CONTROL, 0x02, 0x00);
        gah_cdc_bits(o, RXM_FS_CNT_CONTROL, 0x01, 0x01);
        for (unsigned n = 0; n < 3; n++) {
            gah_cdc_bits(o, RXM_PATH_SEC7(n) + (n == 2 ? 0x0c : 0), 0x07, 0x02);   // RX2 SEC7 is at 0x1540
            gah_cdc_bits(o, RXM_PATH_CFG3(n), 0x03, 0x02);
        }
    } else {
        gah_cdc_bits(o, RXM_FS_CNT_CONTROL, 0x01, 0x00);
        gah_cdc_bits(o, RXM_FS_CNT_CONTROL, 0x02, 0x02);
        gah_cdc_bits(o, RXM_MCLK_CONTROL, 0x02, 0x00);
        gah_cdc_bits(o, RXM_MCLK_CONTROL, 0x01, 0x00);
    }
    gah_note(o, on ? "rx macro mclk on (MCLK_CONTROL, FS_CNT_CONTROL)" : "rx macro mclk off", 0,
             o->cdc_rd(o->ctx, RXM_MCLK_CONTROL), o->cdc_rd(o->ctx, RXM_FS_CNT_CONTROL));
}

static inline uint8_t gah_rxm_vol_code(int32_t db) {
    if (db < -84) db = -84;
    if (db > 40) db = 40;
    return (uint8_t)(int8_t)db;
}

// Interpolators of an output (rx-macro.c, mixer_paths.xml): headphones = RX0 -> INT0 (HPHL) + RX1 -> INT1 (HPHR);
// handset (earpiece) = RX0 -> INT0 alone with "RX_EAR Mode" ON. Both use "RX INTn DEM MUX" = CLSH_DSM_OUT.
// On: input mux, 48 kHz, DEM mux, then rx_macro_enable_interp_clk (PGA mute, DSM clock, CFG2, LUT bypass or ear-mode
// CFG1 bit 1, class-H block with the 16 ohm K1 and V_PA 0x1c, or 0x39 in ear mode), rx_macro_enable_rx_path_clk
// (CFG1 0x10, clock 0x20) and the digital gain. The path STAYS MUTED: Linux unmutes it only after the wcd937x PA is
// up (WCD_BOLERO_EVT_RX_MUTE from the PA POST_PMU), gah_rxm_mute(o, ear, 0) here. Compander, HD2, idle detect are
// off in the vendor mixer paths and are not touched. Off: the reverse, rate back to 48 kHz, class-H block off.
static inline void gah_rxm_path(const struct gah_ops *o, int on, int ear, int32_t vol_db) {
    const unsigned count = ear ? 1u : 2u;
    if (on) {
        gah_cdc_bits(o, RXM_CLSH_CRC, 0x01, 0x01);
        gah_cdc_bits(o, RXM_CLSH_K1_LSB, 0xff, 0xc0);
        gah_cdc_bits(o, RXM_CLSH_K1_MSB, 0x0f, 0x00);
        for (unsigned n = 0; n < count; n++) {
            gah_cdc_bits(o, RXM_INP_MUX_INT0_CFG0 + 8 * n, 0x0f, 5 + n);
            gah_cdc_bits(o, RXM_PATH_CTL(n), 0x0f, 0x04);
            gah_cdc_bits(o, RXM_PATH_CFG1(n), 0x01, 0x01);
            gah_cdc_bits(o, RXM_PATH_CTL(n), 0x10, 0x10);
            gah_cdc_bits(o, RXM_DSM_CTL(n), 0x01, 0x01);
            gah_cdc_bits(o, RXM_PATH_CFG2(n), 0x03, 0x03);
            if (ear) gah_cdc_bits(o, RXM_PATH_CFG1(0), 0x02, 0x02);
            else gah_cdc_bits(o, n ? RXM_TOP_HPHR_COMP_LUT : RXM_TOP_HPHL_COMP_LUT, 0x80, 0x80);
            gah_cdc_bits(o, RXM_CLSH_HPH_V_PA, 0x3f, ear ? 0x39 : 0x1c);
            gah_cdc_bits(o, RXM_CLSH_DECAY_CTRL, 0x07, 0x00);
            gah_cdc_bits(o, RXM_PATH_CFG0(n), 0x40, 0x40);
            gah_cdc_bits(o, RXM_PATH_CFG1(n), 0x10, 0x10);
            gah_cdc_bits(o, RXM_PATH_CTL(n), 0x20, 0x20);
            o->cdc_wr(o->ctx, RXM_VOL_CTL(n), gah_rxm_vol_code(vol_db));
            gah_note(o, ear ? "rx macro INT0 on, ear mode, muted (n, PATH_CTL)" : "rx macro INTn on, muted (n, PATH_CTL)", 0, n,
                     o->cdc_rd(o->ctx, RXM_PATH_CTL(n)));
        }
        return;
    }
    for (unsigned n = 0; n < count; n++) {
        gah_cdc_bits(o, RXM_PATH_CTL(n), 0x10, 0x10);
        gah_cdc_bits(o, RXM_PATH_MIX_CTL(n), 0x10, 0x10);
        gah_cdc_bits(o, RXM_PATH_CFG1(n), 0x10, 0x00);
        gah_cdc_bits(o, RXM_DSM_CTL(n), 0x01, 0x00);
        gah_cdc_bits(o, RXM_PATH_CTL(n), 0x20, 0x00);
        gah_cdc_bits(o, RXM_PATH_CTL(n), 0x40, 0x40);
        gah_cdc_bits(o, RXM_PATH_CTL(n), 0x40, 0x00);
        gah_cdc_bits(o, RXM_PATH_CTL(n), 0x0f, 0x04);
        gah_cdc_bits(o, RXM_PATH_CFG2(n), 0x03, 0x00);
        // rx_macro_hphdelay_lutbypass off. Linux leaves RX0 CFG1 bit 1 set after an ear-mode run (it clears it only
        // when ear mode is off); here it is cleared either way, so a later headphone run starts from the HPH state.
        if (n == 0) gah_cdc_bits(o, RXM_PATH_CFG1(0), 0x02, 0x00);
        gah_cdc_bits(o, n ? RXM_TOP_HPHR_COMP_LUT : RXM_TOP_HPHL_COMP_LUT, 0x80, 0x00);
        gah_cdc_bits(o, n ? RXM_COMP1_CTL7 : RXM_COMP0_CTL7, 0x20, 0x20);
        gah_note(o, "rx macro INTn off (n)", 0, n, 0);
    }
    gah_cdc_bits(o, RXM_CLSH_CRC, 0x01, 0x00);
}

// WCD_BOLERO_EVT_RX_MUTE (rx_macro_event_handler): main and mix path PGA mute of the output's interpolators.
static inline void gah_rxm_mute(const struct gah_ops *o, int ear, int mute) {
    for (unsigned n = 0; n < (ear ? 1u : 2u); n++) {
        gah_cdc_bits(o, RXM_PATH_CTL(n), 0x10, mute ? 0x10 : 0x00);
        gah_cdc_bits(o, RXM_PATH_MIX_CTL(n), 0x10, mute ? 0x10 : 0x00);
    }
    gah_note(o, mute ? "rx macro muted (interpolators)" : "rx macro unmuted (interpolators)", 0, ear ? 1u : 2u, 0);
}

static inline void gah_rxm_volume(const struct gah_ops *o, int ear, int32_t vol_db) {
    for (unsigned n = 0; n < (ear ? 1u : 2u); n++) o->cdc_wr(o->ctx, RXM_VOL_CTL(n), gah_rxm_vol_code(vol_db));
    gah_note(o, "rx macro digital gain (dB code, interpolators)", 0, gah_rxm_vol_code(vol_db), ear ? 1u : 2u);
}

// ---- wcd937x RX (registers over the VA SoundWire link of the capture side; the audio itself over the RX link) ------
static inline int gah_wcd_bits(const struct gah_ops *o, uint16_t reg, uint8_t mask, uint8_t val) {
    uint8_t v = 0;
    int r = o->wcd_rd(o->ctx, reg, &v);
    if (r) return r;
    return o->wcd_wr(o->ctx, reg, (uint8_t)((v & ~mask) | (val & mask)));
}
static inline uint8_t gah_wcd_get(const struct gah_ops *o, uint16_t reg) {
    uint8_t v = 0;
    o->wcd_rd(o->ctx, reg, &v);
    return v;
}
static inline int gah_wcd_ready(const struct gah_ops *o, const char *what) {
    if (o->wcd_rd && o->wcd_wr) return 0;
    gah_note(o, what, GAH_ERR_NOTSUP, 0, 0);
    return GAH_ERR_NOTSUP;
}

// wcd937x reset + init_reg: not here. Reset/rails = guamp_wcd937x_power (GuampAudioHw.inc), init_reg =
// guampmic::Codec::wcdSetup (GuampMicCodec.inc, run at every bring-up of the SoundWire link).

// wcd937x_rx_clk_enable / _disable for one user (the HPH pair counts as one here: both enable and disable happen in
// the same call, Linux's counter goes 0 -> 2 -> 0 and writes the same registers).
static inline void gah_wcd_rx_clk(const struct gah_ops *o, int on) {
    if (on) {
        gah_wcd_bits(o, WCD_DIG_DIG_CLK_CTL, 0x08, 0x08);
        gah_wcd_bits(o, WCD_DIG_ANA_CLK_CTL, 0x01, 0x01);
        gah_wcd_bits(o, WCD_ANA_RX_SUPPLIES, 0x01, 0x01);
        gah_wcd_bits(o, WCD_DIG_RX0_CTL, 0x40, 0x00);
        gah_wcd_bits(o, WCD_DIG_RX1_CTL, 0x40, 0x00);
        gah_wcd_bits(o, WCD_DIG_RX2_CTL, 0x40, 0x00);
        gah_wcd_bits(o, WCD_DIG_ANA_CLK_CTL, 0x02, 0x02);
    } else {
        gah_wcd_bits(o, WCD_ANA_RX_SUPPLIES, 0x01, 0x00);
        gah_wcd_bits(o, WCD_DIG_ANA_CLK_CTL, 0x02, 0x00);
        gah_wcd_bits(o, WCD_DIG_ANA_CLK_CTL, 0x01, 0x00);
    }
}

// wcd-clsh.c for "RX HPH Mode" = CLS_AB (mixer_paths.xml default; the vendor never selects a class-H mode for these
// paths), state machine IDLE -> HPHL (-> HPH_ST, nothing) or IDLE -> EAR: wcd_clsh_state_hph_l / _ear enable, and the
// disable of the last state going back to IDLE. In CLS_AB the CLSH SoundWire port is not connected (wcd937x_enable_clsh).
static inline void gah_wcd_clsh(const struct gah_ops *o, int on) {
    if (on) {
        gah_wcd_bits(o, WCD_ANA_RX_SUPPLIES, 0x02, 0x00);       // buck regulator mode
        gah_wcd_bits(o, WCD_ANA_RX_SUPPLIES, 0x04, 0x00);       // flyback mode: not HIFI
        gah_wcd_bits(o, WCD_FLYBACK_VNEG_CTRL_4, 0xf0, 0x70);
        gah_wcd_bits(o, WCD_FLYBACK_VNEGDAC_CTRL_2, 0xe0, 0xa0); // force IQ
        o->delay_us(o->ctx, 100);
        gah_wcd_bits(o, WCD_CLASSH_MODE_3, 0x02, 0x02);
        gah_wcd_bits(o, WCD_CLASSH_MODE_2, 0xff, 0x1c);
        gah_wcd_bits(o, WCD_FLYBACK_VNEG_CTRL_1, 0xe0, 0xe0);    // flyback on
        gah_wcd_bits(o, WCD_ANA_RX_SUPPLIES, 0x40, 0x40);
        o->delay_us(o->ctx, 100);
        gah_wcd_bits(o, WCD_FLYBACK_VNEGDAC_CTRL_2, 0xe0, 0xe0);
        o->delay_us(o->ctx, 500);
        gah_wcd_bits(o, WCD_RX_BIAS_FLYB_BUFF, 0x0f, 0x0a);      // flyback current
        gah_wcd_bits(o, WCD_RX_BIAS_FLYB_BUFF, 0xf0, 0xa0);
        o->delay_us(o->ctx, 100);
        gah_wcd_bits(o, WCD_ANA_RX_SUPPLIES, 0x08, 0x00);       // buck mode: not HIFI
        gah_wcd_bits(o, WCD_ANA_RX_SUPPLIES, 0x80, 0x80);       // buck on
        o->delay_us(o->ctx, 500);
        gah_wcd_bits(o, WCD_CLASSH_MODE_2, 0xff, 0x3a);
        o->delay_us(o->ctx, 500);
        gah_wcd_bits(o, WCD_ANA_HPH, 0x0c, 0x0c);                // wcd_clsh_set_hph_mode(CLS_AB)
    } else {
        gah_wcd_bits(o, WCD_ANA_HPH, 0x0c, 0x00);                // CLS_H_NORMAL
        gah_wcd_bits(o, WCD_FLYBACK_VNEG_CTRL_1, 0xe0, 0xe0);    // flyback off
        gah_wcd_bits(o, WCD_ANA_RX_SUPPLIES, 0x40, 0x00);
        o->delay_us(o->ctx, 100);
        gah_wcd_bits(o, WCD_FLYBACK_VNEGDAC_CTRL_2, 0xe0, 0xe0);
        o->delay_us(o->ctx, 500);
        gah_wcd_bits(o, WCD_ANA_RX_SUPPLIES, 0x80, 0x00);       // buck off
        o->delay_us(o->ctx, 500);
        gah_wcd_bits(o, WCD_CLASSH_MODE_2, 0xff, 0x3a);
        o->delay_us(o->ctx, 500);
        gah_wcd_bits(o, WCD_HPH_NEW_INT_PA_MISC2, 0x20, 0x00);   // force IQ off
        gah_wcd_bits(o, WCD_RX_BIAS_HPH_LOWPOWER, 0xf0, 0x80);
        gah_wcd_bits(o, WCD_HPH_PA_CTL1, 0x0e, 0x06);
        gah_wcd_bits(o, WCD_ANA_RX_SUPPLIES, 0x04, 0x00);       // flyback and buck mode back to default
        gah_wcd_bits(o, WCD_FLYBACK_VNEG_CTRL_4, 0xf0, 0x70);
        gah_wcd_bits(o, WCD_ANA_RX_SUPPLIES, 0x08, 0x00);
    }
    gah_note(o, on ? "wcd937x class-H fsm CLS_AB on (RX_SUPPLIES, ANA_HPH)" : "wcd937x class-H fsm back to IDLE", 0,
             gah_wcd_get(o, WCD_ANA_RX_SUPPLIES), gah_wcd_get(o, WCD_ANA_HPH));
}

static inline int gah_swr_rx(const struct gah_ops *o, int on, uint8_t ch_mask) {
    int r = o->swr_rx ? o->swr_rx(o->ctx, on, ch_mask) : GAH_ERR_NOTSUP;
    gah_note(o, on ? "rx soundwire port + datapath on (slave ch mask)" : "rx soundwire port + datapath off (slave ch mask)", r,
             ch_mask, 0);
    return r;
}

// Headphones, CLS_AB without compander, the DAPM power-up order of soc-dapm (4.19) over the wcd937x widgets of the
// "headphones" path: DACs RDAC1/RDAC2 (hph{l,r}_dac_event PRE/POST_PMU, "HPHL/HPHR Volume"), mixers RX1/RX2 (port
// HPH_L/HPH_R connect = slave port 0 ch 1|2 -> master port 1), PGAs HPHL/HPHR (enable_hph{l,r}_pa: datapath, class-H,
// ANA_HPH 0x20/0x10, PDM watchdog; DAPM power bits ANA_HPH 7/6; POST_PMU 20 ms each, TIMER1, RX_SUPPLIES bit 1).
// The RX macro is unmuted by the caller afterwards (WCD_BOLERO_EVT_RX_MUTE). gain_code: HPH_x_EN[4:0], see
// gah_wcd_gain_code. The SoundWire port + datapath go through o->swr_rx at the PA PRE_PMU point, as in Linux.
static inline int gah_wcd_hph(const struct gah_ops *o, int on, uint8_t gain_code) {
    int r = gah_wcd_ready(o, "wcd937x hph: no wcd937x register access");
    if (r) return r;
    if (on) {
        gah_wcd_rx_clk(o, 1);
        gah_wcd_bits(o, WCD_DIG_DIG_CLK_CTL, 0x01, 0x01);
        gah_wcd_bits(o, WCD_DIG_HPH_GAIN_CTL, 0x04, 0x04);
        gah_wcd_bits(o, WCD_HPH_RDAC_CLK_CTL1, 0x80, 0x00);
        gah_wcd_bits(o, WCD_DIG_DIG_CLK_CTL, 0x02, 0x02);
        gah_wcd_bits(o, WCD_DIG_HPH_GAIN_CTL, 0x08, 0x08);
        gah_wcd_bits(o, WCD_HPH_RDAC_CLK_CTL1, 0x80, 0x00);
        const int lowpower = gah_wcd_get(o, WCD_EFUSE_16) == 0x02 && (gah_wcd_get(o, WCD_ANA_HPH) & 0x0c) == 0x0c;
        if (lowpower) gah_wcd_bits(o, WCD_RX_BIAS_HPH_LOWPOWER, 0xf0, 0x90);
        gah_wcd_bits(o, WCD_DIG_COMP_CTL_0, 0x02, 0x00);
        gah_wcd_bits(o, WCD_HPH_L_EN, 0x3f, (uint8_t)(0x20 | (gain_code & 0x1f)));
        gah_wcd_bits(o, WCD_HPH_TIMER1, 0x02, 0x00);
        gah_wcd_bits(o, WCD_DIG_COMP_CTL_0, 0x01, 0x00);
        gah_wcd_bits(o, WCD_HPH_R_EN, 0x3f, (uint8_t)(0x20 | (gain_code & 0x1f)));
        gah_wcd_bits(o, WCD_HPH_TIMER1, 0x02, 0x00);
        r = gah_swr_rx(o, 1, 0x03);
        if (r) {
            gah_wcd_rx_clk(o, 0);
            gah_wcd_bits(o, WCD_DIG_DIG_CLK_CTL, 0x03, 0x00);
            return r;
        }
        gah_wcd_clsh(o, 1);
        gah_wcd_bits(o, WCD_ANA_HPH, 0x20, 0x20);
        o->delay_us(o->ctx, 100);
        gah_wcd_bits(o, WCD_DIG_PDM_WD_CTL0, 0x17, 0x13);
        gah_wcd_bits(o, WCD_ANA_HPH, 0x10, 0x10);
        o->delay_us(o->ctx, 100);
        gah_wcd_bits(o, WCD_DIG_PDM_WD_CTL1, 0x17, 0x13);
        r = gah_wcd_bits(o, WCD_ANA_HPH, 0xc0, 0xc0);              // DAPM: HPHL PGA bit 7, HPHR PGA bit 6
        for (unsigned side = 0; side < 2; side++) {
            o->sleep_ms(o->ctx, 20);                               // no compander: 20 ms after the PA
            gah_wcd_bits(o, WCD_HPH_TIMER1, 0x02, 0x02);
            gah_wcd_bits(o, WCD_ANA_RX_SUPPLIES, 0x02, 0x02);      // CLS_AB
        }
        gah_note(o, "wcd937x HPH PA on (gain code, ANA_HPH)", r, gain_code, gah_wcd_get(o, WCD_ANA_HPH));
        return r;
    }
    gah_wcd_bits(o, WCD_ANA_HPH, 0xc0, 0x00);
    o->sleep_ms(o->ctx, 20);
    gah_wcd_bits(o, WCD_DIG_PDM_WD_CTL0, 0x17, 0x00);
    gah_wcd_bits(o, WCD_ANA_HPH, 0x20, 0x00);
    o->sleep_ms(o->ctx, 20);
    gah_wcd_bits(o, WCD_DIG_PDM_WD_CTL1, 0x17, 0x00);
    gah_wcd_bits(o, WCD_ANA_HPH, 0x10, 0x00);
    gah_wcd_clsh(o, 0);
    gah_wcd_bits(o, WCD_HPH_NEW_INT_RDAC_HD2_CTL_L, 0x0f, 0x01);
    gah_wcd_bits(o, WCD_HPH_NEW_INT_RDAC_HD2_CTL_R, 0x0f, 0x01);
    gah_wcd_rx_clk(o, 0);
    gah_wcd_bits(o, WCD_DIG_DIG_CLK_CTL, 0x03, 0x00);
    r = gah_swr_rx(o, 0, 0x03);
    gah_note(o, "wcd937x HPH PA off", r, 0, gah_wcd_get(o, WCD_ANA_HPH));
    return r;
}

// Earpiece ("handset" path: RX_CDC_DMA_RX_0 one channel, RX0 -> INT0, EAR_RDAC on, RDAC3_MUX = RX1, RX_EAR Mode ON),
// CLS_AB without compander: RDAC3 (ear_dac_event PRE_PMU), mixer RX1 (port HPH_L = slave port 0 ch 1), EAR PGA
// (enable_ear_pa: datapath, PDM watchdog of HPHL, EAR compander off bit; DAPM ANA_EAR bit 7; POST_PMU 6 ms,
// RX_SUPPLIES bit 1). ear_gain: "EAR PA GAIN" index 0..16 (G_6_DB .. G_M18_DB, 1.5 dB steps) in EAR_COMPANDER_CTL[6:2].
static inline int gah_wcd_ear(const struct gah_ops *o, int on, uint8_t ear_gain) {
    int r = gah_wcd_ready(o, "wcd937x ear: no wcd937x register access");
    if (r) return r;
    if (ear_gain > WCD_EAR_GAIN_MAX) ear_gain = WCD_EAR_GAIN_MAX;
    if (on) {
        gah_wcd_bits(o, WCD_DIG_EAR_PATH_CTL, 0x01, 0x00);           // RDAC3_MUX = RX1
        gah_wcd_bits(o, WCD_ANA_EAR_COMP_CTL, 0x7c, (uint8_t)(ear_gain << 2));
        gah_wcd_rx_clk(o, 1);
        gah_wcd_bits(o, WCD_DIG_HPH_GAIN_CTL, 0x04, 0x04);
        gah_wcd_bits(o, WCD_DIG_DIG_CLK_CTL, 0x01, 0x01);
        o->sleep_ms(o->ctx, 5);
        gah_wcd_bits(o, WCD_FLYBACK_EN, 0x04, 0x00);
        gah_wcd_clsh(o, 1);
        r = gah_swr_rx(o, 1, 0x01);
        if (r) {
            gah_wcd_clsh(o, 0);
            gah_wcd_bits(o, WCD_FLYBACK_EN, 0x04, 0x04);
            gah_wcd_rx_clk(o, 0);
            gah_wcd_bits(o, WCD_DIG_DIG_CLK_CTL, 0x01, 0x00);
            return r;
        }
        gah_wcd_bits(o, WCD_DIG_PDM_WD_CTL0, 0x17, 0x13);
        gah_wcd_bits(o, WCD_ANA_EAR_COMP_CTL, 0x80, 0x80);
        r = gah_wcd_bits(o, WCD_ANA_EAR, 0x80, 0x80);               // DAPM: EAR PGA bit 7
        o->sleep_ms(o->ctx, 6);
        gah_wcd_bits(o, WCD_ANA_RX_SUPPLIES, 0x02, 0x02);           // CLS_AB
        gah_note(o, "wcd937x EAR PA on (gain index, EAR_COMPANDER_CTL)", r, ear_gain, gah_wcd_get(o, WCD_ANA_EAR_COMP_CTL));
        return r;
    }
    gah_wcd_bits(o, WCD_ANA_EAR, 0x80, 0x00);
    gah_wcd_bits(o, WCD_ANA_EAR_COMP_CTL, 0x80, 0x00);
    o->sleep_ms(o->ctx, 7);
    gah_wcd_clsh(o, 0);
    gah_wcd_bits(o, WCD_FLYBACK_EN, 0x04, 0x04);
    gah_wcd_bits(o, WCD_DIG_PDM_WD_CTL0, 0x17, 0x00);
    o->sleep_ms(o->ctx, 10);
    gah_wcd_bits(o, WCD_EAR_EN_REG, 0x02, 0x00);                    // EAR CnP FSM off, PA toggle, FSM on
    gah_wcd_bits(o, WCD_ANA_EAR, 0x80, 0x80);
    gah_wcd_bits(o, WCD_ANA_EAR, 0x80, 0x00);
    gah_wcd_bits(o, WCD_EAR_EN_REG, 0x02, 0x02);
    gah_wcd_bits(o, WCD_HPH_NEW_INT_RDAC_HD2_CTL_L, 0x0f, 0x01);
    gah_wcd_rx_clk(o, 0);
    gah_wcd_bits(o, WCD_DIG_DIG_CLK_CTL, 0x01, 0x00);
    r = gah_swr_rx(o, 0, 0x01);
    gah_note(o, "wcd937x EAR PA off", r, 0, gah_wcd_get(o, WCD_ANA_EAR));
    return r;
}

// Level (centibel) -> HPH_x_EN gain code: -30 dB .. 0 dB in 1.5 dB steps, code = 20 - steps (inverted control).
static inline uint8_t gah_wcd_gain_code(int32_t level_cb) {
    int32_t steps = (level_cb + 3000) / 150;
    if (steps < 0) steps = 0;
    if (steps > 20) steps = 20;
    return (uint8_t)(20 - steps);
}
// Level (centibel) -> "EAR PA GAIN" index: +6 dB .. -18 dB in 1.5 dB steps (index 0 = +6 dB), rounded quieter.
static inline uint8_t gah_wcd_ear_gain(int32_t level_cb) {
    int32_t idx = (600 - level_cb + 149) / 150;
    if (idx < 0) idx = 0;
    if (idx > WCD_EAR_GAIN_MAX) idx = WCD_EAR_GAIN_MAX;
    return (uint8_t)idx;
}

// ---- wcd937x MBHC: 3.5 mm jack (wcd-mbhc-v2.c wcd_mbhc_initialise + wcd937x-mbhc.c callbacks, ADC detection of
// wcd-mbhc-adc.c). guamp machine DT: qcom,msm-mbhc-hphl-swh = 1, gnd-swh = 1, no USB-C analog, MBHC micbias = MIC
// BIAS2 (2.6 V, hs_ext_micbias), WCD_MBHC_HS_V_MAX 1600 mV -> HS_VREF 2, moisture detection off, insertion debounce
// 96 ms. The mechanical switch raises WCD937X_IRQ_MBHC_SW_DET (INTR_STATUS_0 bit 4, regmap-irq: level registers 0,
// clear_ack); MECH_DETECTION_TYPE says which edge is armed (1 = insertion) and is flipped at every event, as
// wcd_mbhc_swch_irq_handler does. Polled here (the codec interrupt reaches the CPU only as a SoundWire slave alert).
// SWCH_LEVEL_REMOVE (RESULT_3 bit 4, 1 = no plug) is read too: it backs the interrupt up when it never comes.
enum { GAH_JACK_NONE = 0, GAH_JACK_HEADPHONE = 1, GAH_JACK_HEADSET = 2, GAH_JACK_HIGH_HPH = 3, GAH_JACK_UNKNOWN = 4 };

struct gah_jack {
    uint8_t inited, present, type, armed;   // armed = MECH_DETECTION_TYPE now (1 insertion, 0 removal)
    uint8_t micb2, level_votes, last_mech, last_res3;
    uint16_t mic_mv, adc;
    uint32_t events, irq_events, level_events, polls;
    int32_t last_rc;
};

static inline int gah_mbhc_init(const struct gah_ops *o, struct gah_jack *j) {
    int r = gah_wcd_ready(o, "mbhc: no wcd937x register access");
    if (r) return r;
    gah_wcd_bits(o, WCD_MBHC_NEW_INT_MECH_DET_CURRENT, 0x1f, 0x08);   // HS_PULLUP_I_2P0_UA
    gah_wcd_bits(o, WCD_MBHC_NEW_CTL_2, 0x0c, 0x00);                  // moisture detection off (R_OFF)
    gah_wcd_bits(o, WCD_ANA_MBHC_MECH, 0x10, 0x10);                   // HPHL_PLUG_TYPE = hphl_swh 1
    gah_wcd_bits(o, WCD_ANA_MBHC_MECH, 0x08, 0x08);                   // GND_PLUG_TYPE = gnd_swh 1
    gah_wcd_bits(o, WCD_ANA_MBHC_MECH, 0x01, 0x01);                   // SW_HPH_LP_100K_TO_GND
    gah_wcd_bits(o, WCD_ANA_MBHC_MECH, 0x04, 0x04);                   // HS_L_DET_PULL_UP_COMP_CTRL
    gah_wcd_bits(o, WCD_ANA_MBHC_MECH, 0x20, 0x20);                   // detect insertion first
    gah_wcd_bits(o, WCD_ANA_MBHC_MECH, 0x80, 0x80);                   // L_DET_EN
    gah_wcd_bits(o, WCD_MBHC_NEW_PLUG_DETECT_CTL, 0x0f, 0x06);        // INSREM_DBNC 96 ms
    gah_wcd_bits(o, WCD_MBHC_NEW_CTL_1, 0x03, 0x02);                  // BTN_DBNC 16 ms
    gah_wcd_bits(o, WCD_ANA_MBHC_ELECT, 0x01, 0x01);                  // mbhc_bias
    gah_wcd_bits(o, WCD_MBHC_NEW_CTL_1, 0x80, 0x80);                  // clk_setup
    gah_wcd_bits(o, WCD_MBHC_NEW_CTL_2, 0x03, 0x02);                  // HS_VREF (1600 - 1400) / 100
    for (unsigned i = 0; i < 3; i++) gah_wcd_bits(o, (uint16_t)(WCD_DIG_INTR_LEVEL_0 + i), 0xff, 0x00);
    gah_wcd_bits(o, WCD_DIG_INTR_CLEAR_0, 0xff, 0x10);                // clear_ack: write the bit, then 0
    gah_wcd_bits(o, WCD_DIG_INTR_CLEAR_0, 0xff, 0x00);
    r = gah_wcd_bits(o, WCD_DIG_INTR_MASK_0, 0x10, 0x00);             // unmask MBHC_SW_DET
    j->inited = r == 0;
    j->armed = 1;
    j->present = 0;
    j->type = GAH_JACK_NONE;
    j->level_votes = 0;
    j->last_rc = r;
    gah_note(o, "mbhc init (MECH, CTL_1)", r, gah_wcd_get(o, WCD_ANA_MBHC_MECH), gah_wcd_get(o, WCD_MBHC_NEW_CTL_1));
    return r;
}
// The registers this module set are lost when the codec is reset (guamp_wcd937x_power, capture link restart).
static inline int gah_mbhc_lost(const struct gah_ops *o) {
    uint8_t v = 0;
    if (o->wcd_rd(o->ctx, WCD_MBHC_NEW_CTL_1, &v)) return 1;
    return (v & 0x80) == 0 || (gah_wcd_get(o, WCD_ANA_MBHC_MECH) & 0x80) == 0;
}

// wcd937x_micbias_control(MIC_BIAS_2, MICB_ENABLE / MICB_DISABLE) for the headset microphone. The clock bits are the
// ones the capture side also sets (it never clears DIG_CLK 0xf0 either); only MICB2 is switched off here.
static inline void gah_mbhc_micb2(const struct gah_ops *o, struct gah_jack *j, int on) {
    if (on) {
        gah_wcd_bits(o, WCD_DIG_DIG_CLK_CTL, 0xf0, 0xf0);
        gah_wcd_bits(o, WCD_DIG_ANA_CLK_CTL, 0x10, 0x10);
        gah_wcd_bits(o, WCD_MICB1_TEST_CTL_2, 0x01, 0x01);
        gah_wcd_bits(o, WCD_MICB2_TEST_CTL_2, 0x01, 0x01);
        gah_wcd_bits(o, WCD_MICB3_TEST_CTL_2, 0x01, 0x01);
        gah_wcd_bits(o, WCD_ANA_MICB2, 0xc0, 0x40);
    } else {
        gah_wcd_bits(o, WCD_ANA_MICB2, 0xc0, 0x00);
    }
    j->micb2 = on ? 1 : 0;
}

// MICB2 VOUT -> mV (wcd_mbhc_get_micbias: 1000 + 50 mV per step), ADC code -> mV (wcd_get_voltage_from_adc).
static inline uint32_t gah_mbhc_micb_mv(uint8_t micb2) { return 1000u + 50u * (micb2 & 0x3fu); }
static inline uint32_t gah_mbhc_adc_mv(uint8_t adc, uint32_t micb_mv) { return (adc * 125u * micb_mv) / (1800u * 10u); }
// wcd_mbhc_get_plug_from_adc with the default thresholds scaled to the micbias (1700 / 75 mV at 1.8 V).
static inline uint8_t gah_mbhc_plug_from_mv(uint32_t mv, uint32_t micb_mv) {
    const uint32_t hs = 1700u * micb_mv / 1800u, hph = 75u * micb_mv / 1800u;
    if (mv < hph) return (uint8_t)GAH_JACK_HEADPHONE;
    if (mv > hs) return (uint8_t)GAH_JACK_HIGH_HPH;
    return (uint8_t)GAH_JACK_HEADSET;
}

// wcd_measure_adc_once(MUX_CTL_IN2P = 1): one ADC conversion of the mic ring with MICBIAS2 on. <0 on timeout.
static inline int gah_mbhc_adc_once(const struct gah_ops *o, uint8_t *code) {
    gah_wcd_bits(o, WCD_MBHC_NEW_CTL_1, 0x10, 0x00);                  // ADC_MODE one-shot
    const uint8_t adc_en = gah_wcd_get(o, WCD_MBHC_NEW_CTL_1) & 0x08;
    gah_wcd_bits(o, WCD_MBHC_NEW_CTL_1, 0x08, 0x00);
    gah_wcd_bits(o, WCD_ANA_MBHC_ELECT, 0x80, 0x00);                  // FSM_EN 0
    gah_wcd_bits(o, WCD_MBHC_NEW_CTL_2, 0x70, 0x10);                  // MUX_CTL = IN2P
    gah_wcd_bits(o, WCD_ANA_MBHC_ELECT, 0x80, 0x80);
    gah_wcd_bits(o, WCD_MBHC_NEW_CTL_1, 0x08, 0x08);                  // ADC_EN
    int r = GAH_ERR_IO;
    for (unsigned t = 0; t < 6 && r; t++) {
        o->delay_us(o->ctx, 600);
        const uint8_t st = gah_wcd_get(o, WCD_MBHC_NEW_FSM_STATUS);
        if (st & 0x80) continue;                                        // ADC_TIMEOUT
        if (!(st & 0x40)) continue;                                     // ADC_COMPLETE
        *code = gah_wcd_get(o, WCD_MBHC_NEW_ADC_RESULT);
        r = 0;
    }
    gah_wcd_bits(o, WCD_MBHC_NEW_CTL_1, 0x08, adc_en);
    return r;
}

// Insertion: micbias ramp + MICBIAS2, settle, one ADC reading -> plug type (headphone: mic ring at ground; headset:
// mic in between; high impedance: open). MICBIAS2 stays on for a headset (its microphone), off otherwise.
static inline void gah_mbhc_classify(const struct gah_ops *o, struct gah_jack *j) {
    gah_wcd_bits(o, WCD_ANA_MICB2_RAMP, 0x1c, 0x0c);
    gah_wcd_bits(o, WCD_ANA_MICB2_RAMP, 0x80, 0x80);
    gah_mbhc_micb2(o, j, 1);
    o->sleep_ms(o->ctx, 50);
    const uint32_t micb_mv = gah_mbhc_micb_mv(gah_wcd_get(o, WCD_ANA_MICB2));
    uint8_t code = 0;
    const int r = gah_mbhc_adc_once(o, &code);
    j->adc = code;
    j->mic_mv = (uint16_t)gah_mbhc_adc_mv(code, micb_mv);
    j->type = r ? (uint8_t)GAH_JACK_UNKNOWN : gah_mbhc_plug_from_mv(j->mic_mv, micb_mv);
    gah_wcd_bits(o, WCD_ANA_MBHC_ELECT, 0x80, 0x00);                  // FSM off again (no button detection)
    gah_wcd_bits(o, WCD_ANA_MICB2_RAMP, 0x80, 0x00);
    gah_wcd_bits(o, WCD_ANA_MICB2_RAMP, 0x1c, 0x00);
    if (j->type != GAH_JACK_HEADSET) gah_mbhc_micb2(o, j, 0);
    gah_note(o, "mbhc plug type (type 1 hph 2 headset 3 high-z 4 unknown, mic mV)", r, j->type, j->mic_mv);
}

static inline void gah_mbhc_removed(const struct gah_ops *o, struct gah_jack *j) {
    gah_wcd_bits(o, WCD_ANA_MBHC_ELECT, 0x80, 0x00);                  // FSM_EN 0
    gah_wcd_bits(o, WCD_ANA_MBHC_ELECT, 0x70, 0x00);                  // BTN_ISRC_CTL 0
    gah_wcd_bits(o, WCD_ANA_MBHC_ELECT, 0x08, 0x08);                  // ELECT_DETECTION_TYPE 1
    gah_wcd_bits(o, WCD_ANA_MBHC_ELECT, 0x06, 0x00);                  // ELECT_SCHMT_ISRC 0
    if (j->micb2) gah_mbhc_micb2(o, j, 0);
    j->type = GAH_JACK_NONE;
    j->mic_mv = 0;
}

// One poll. 1 = the jack state changed (j->present / j->type are new), 0 = nothing, <0 = register access failed.
// level_polls: consecutive polls SWCH_LEVEL_REMOVE must disagree with the state before it counts (0 = never).
static inline int gah_mbhc_poll(const struct gah_ops *o, struct gah_jack *j, unsigned level_polls) {
    uint8_t st = 0, mech = 0, res3 = 0;
    int r = o->wcd_rd(o->ctx, WCD_DIG_INTR_STATUS_0, &st);
    if (!r) r = o->wcd_rd(o->ctx, WCD_ANA_MBHC_MECH, &mech);
    if (!r) r = o->wcd_rd(o->ctx, WCD_ANA_MBHC_RESULT_3, &res3);
    j->polls++;
    if (r) { j->last_rc = r; return r; }
    if (mech != j->last_mech || (res3 & 0x10) != (j->last_res3 & 0x10)) {
        gah_note(o, "mbhc raw (MECH, RESULT_3 | INTR_STATUS_0 << 8)", 0, mech, (uint32_t)res3 | (uint32_t)st << 8);
        j->last_mech = mech;
        j->last_res3 = res3;
    }
    int insert = -1;   // 1 insertion, 0 removal, -1 nothing
    if (st & 0x10) {
        gah_wcd_bits(o, WCD_DIG_INTR_CLEAR_0, 0xff, 0x10);
        gah_wcd_bits(o, WCD_DIG_INTR_CLEAR_0, 0xff, 0x00);
        const uint8_t armed = (mech >> 5) & 1;
        gah_wcd_bits(o, WCD_ANA_MBHC_MECH, 0x20, armed ? 0x00 : 0x20);   // detect the other edge next
        j->armed = !armed;
        if (armed && !j->present) insert = 1;
        else if (!armed && j->present) insert = 0;
        if (insert >= 0) j->irq_events++;
        j->level_votes = 0;
    } else if (level_polls) {
        const int plugged = (res3 & 0x10) == 0;
        if (plugged != (int)j->present) {
            if (++j->level_votes >= level_polls) {
                insert = plugged;
                j->level_events++;
                gah_wcd_bits(o, WCD_ANA_MBHC_MECH, 0x20, plugged ? 0x00 : 0x20);
                j->armed = !plugged;
                j->level_votes = 0;
            }
        } else {
            j->level_votes = 0;
        }
    }
    if (insert < 0) return 0;
    if (insert) {
        j->present = 1;
        gah_mbhc_classify(o, j);
    } else {
        j->present = 0;
        gah_mbhc_removed(o, j);
    }
    j->events++;
    gah_note(o, insert ? "mbhc jack inserted (type, mic mV)" : "mbhc jack removed (events, irq events)", 0,
             insert ? j->type : j->events, insert ? j->mic_mv : j->irq_events);
    return 1;
}
#endif
