// SPDX-License-Identifier: BSD-2-Clause-Patent
// GENI I2C master in FIFO mode, polled (see GeniI2c.h). Host-tested by tests/audio_hw_host.cpp against a model of the
// serial engine. Every wait is bounded; a stuck command is cancelled, then aborted, as geni_i2c_err/geni_cancel_m_cmd.
#include "GeniI2c.h"

static uint32_t gi2c_rd(struct geni_i2c *c, uint32_t reg) { return c->se.rd(c->se.ctx, reg); }
static void gi2c_wr(struct geni_i2c *c, uint32_t reg, uint32_t v) { c->se.wr(c->se.ctx, reg, v); }
static void gi2c_set(struct geni_i2c *c, uint32_t reg, uint32_t bits) { gi2c_wr(c, reg, gi2c_rd(c, reg) | bits); }
static void gi2c_udelay(struct geni_i2c *c, unsigned us) { if (c->se.delay_us) c->se.delay_us(c->se.ctx, us); }

static void gi2c_irq_clear(struct geni_i2c *c) {
    gi2c_wr(c, SE_GSI_EVENT_EN, 0);
    gi2c_wr(c, SE_GENI_M_IRQ_CLEAR, 0xffffffffu);
    gi2c_wr(c, SE_GENI_S_IRQ_CLEAR, 0xffffffffu);
    gi2c_wr(c, SE_DMA_TX_IRQ_CLR, 0xffffffffu);
    gi2c_wr(c, SE_DMA_RX_IRQ_CLR, 0xffffffffu);
}

extern "C" int geni_i2c_read_info(struct geni_i2c *c) {
    if (!c || !c->se.rd || !c->se.wr || !c->wrapper.rd) return GENI_ERR_PARAM;
    uint32_t fw = gi2c_rd(c, GENI_FW_REVISION_RO);
    c->info.fw_revision = fw;
    c->info.protocol = (fw >> GENI_FW_PROTO_SHIFT) & 0xff;
    c->info.if_disable = gi2c_rd(c, GENI_IF_DISABLE_RO);
    c->info.fifo_disabled = (c->info.if_disable & GENI_FIFO_IF_DISABLE) != 0;
    c->info.geni_status = gi2c_rd(c, SE_GENI_STATUS);
    c->info.hw_param0 = gi2c_rd(c, SE_HW_PARAM_0);
    c->info.qup_hw_version = c->wrapper.rd(c->wrapper.ctx, QUPV3_HW_VER_REG);
    uint32_t major = c->info.qup_hw_version >> 28, minor = (c->info.qup_hw_version >> 16) & 0xfff;
    uint32_t mask = ((major == 3 && minor >= 10) || major > 3) ? 0xff0000u : 0x3f0000u;
    c->info.tx_fifo_depth = (c->info.hw_param0 & mask) >> 16;
    c->info.fifo_width_bits = (c->info.hw_param0 >> 24) & 0x3f;
    return 0;
}

// se_config_packing(base, 8, 4, true): 4 bytes per FIFO word, byte 0 in bits 7:0.
static void gi2c_packing(struct geni_i2c *c) {
    uint32_t v[4];
    for (unsigned i = 0; i < 4; i++)
        v[i] = ((uint32_t)(i * 8 + 7) << GENI_PACK_START_SHIFT) | (1u << GENI_PACK_DIR_SHIFT) | (7u << GENI_PACK_LEN_SHIFT);
    v[3] |= GENI_PACK_STOP;
    uint32_t cfg0 = v[0] | (v[1] << GENI_PACK_VECTOR_SHIFT), cfg1 = v[2] | (v[3] << GENI_PACK_VECTOR_SHIFT);
    gi2c_wr(c, SE_GENI_TX_PACKING_CFG0, cfg0);
    gi2c_wr(c, SE_GENI_TX_PACKING_CFG1, cfg1);
    gi2c_wr(c, SE_GENI_RX_PACKING_CFG0, cfg0);
    gi2c_wr(c, SE_GENI_RX_PACKING_CFG1, cfg1);
}

extern "C" int geni_i2c_init(struct geni_i2c *c) {
    int r = geni_i2c_read_info(c);
    if (r) return r;
    if (c->info.protocol != GENI_PROTO_I2C) return GENI_ERR_PROTO;
    if (c->info.fifo_disabled && !c->allow_fifo_disabled) return GENI_ERR_NOTSUP;
    if (c->info.tx_fifo_depth < 2 || c->info.fifo_width_bits != 32) return GENI_ERR_NODEV;
    // geni_se_init (tx watermark 1 as Linux i2c), FIFO mode, interrupts outputs off: completion is polled.
    gi2c_irq_clear(c);
    gi2c_set(c, SE_GENI_CGC_CTRL, GENI_DEFAULT_CGC_EN);
    gi2c_set(c, SE_DMA_GENERAL_CFG, GENI_AHB_SEC_SLV_CGC_ON | GENI_DMA_AHB_SLV_CGC_ON | GENI_DMA_TX_CGC_ON | GENI_DMA_RX_CGC_ON);
    gi2c_wr(c, GENI_OUTPUT_CTRL, GENI_DEFAULT_IO_OUTPUT_CTRL);
    gi2c_wr(c, GENI_FORCE_DEFAULT_REG, GENI_FORCE_DEFAULT);
    gi2c_wr(c, SE_GENI_DMA_MODE_EN, gi2c_rd(c, SE_GENI_DMA_MODE_EN) & ~GENI_DMA_MODE_EN);
    gi2c_wr(c, SE_GENI_RX_WATERMARK_REG, c->info.tx_fifo_depth - 3);
    gi2c_wr(c, SE_GENI_RX_RFR_WATERMARK_REG, c->info.tx_fifo_depth - 2);
    gi2c_set(c, SE_GENI_M_IRQ_EN, GENI_M_COMMON_IRQ_EN | GENI_M_CMD_DONE | GENI_M_TX_FIFO_WATERMARK | GENI_M_RX_FIFO_WATERMARK |
                                     GENI_M_RX_FIFO_LAST | GENI_M_GP_IRQ_1 | GENI_M_GP_IRQ_3 | GENI_M_GP_IRQ_4);
    gi2c_set(c, SE_GENI_S_IRQ_EN, GENI_S_COMMON_IRQ_EN);
    gi2c_wr(c, SE_IRQ_EN, 0);
    gi2c_packing(c);
    return 0;
}

extern "C" int geni_i2c_set_clock(struct geni_i2c *c, unsigned khz, unsigned clk_sel) {
    unsigned div, high, low, cycle;
    if (khz == 100) { div = 7; high = 10; low = 11; cycle = 26; }
    else if (khz == 400) { div = 2; high = 7; low = 10; cycle = 24; }
    else if (khz == 1000) { div = 1; high = 3; low = 9; cycle = 18; }
    else return GENI_ERR_PARAM;
    if (clk_sel > 7) return GENI_ERR_PARAM;
    gi2c_wr(c, SE_GENI_CLK_SEL, clk_sel);
    gi2c_wr(c, GENI_SER_M_CLK_CFG, (div << GENI_CLK_DIV_SHIFT) | GENI_SER_CLK_EN);
    gi2c_wr(c, SE_I2C_SCL_COUNTERS, (high << 20) | (low << 10) | cycle);
    c->khz = khz;
    return 0;
}

// geni_cancel_m_cmd, then geni_abort_m_cmd if the cancel does not complete.
static void gi2c_stop_cmd(struct geni_i2c *c) {
    gi2c_wr(c, SE_GENI_TX_WATERMARK_REG, 0);
    gi2c_wr(c, SE_GENI_M_CMD_CTRL_REG, GENI_M_CMD_CANCEL);
    for (unsigned i = 0; i < 1000; i++) {
        if (gi2c_rd(c, SE_GENI_M_IRQ_STATUS) & GENI_M_CMD_CANCEL_DONE) { c->last_cancel = 1; gi2c_irq_clear(c); return; }
        gi2c_udelay(c, 1);
    }
    gi2c_wr(c, SE_GENI_M_CMD_CTRL_REG, GENI_M_CMD_ABORT);
    c->last_cancel = 3;
    for (unsigned i = 0; i < 1000; i++) {
        if (gi2c_rd(c, SE_GENI_M_IRQ_STATUS) & GENI_M_CMD_ABORT_DONE) { c->last_cancel = 2; break; }
        gi2c_udelay(c, 1);
    }
    gi2c_irq_clear(c);
}

static int gi2c_one(struct geni_i2c *c, uint8_t addr, struct geni_i2c_msg *m, int stretch) {
    uint32_t param = (stretch ? GENI_I2C_STOP_STRETCH : 0) | ((uint32_t)(addr & 0x7f) << GENI_I2C_SLV_ADDR_SHIFT);
    unsigned done_bytes = 0;
    gi2c_wr(c, SE_GENI_M_IRQ_CLEAR, 0xffffffffu);
    if (m->read) {
        gi2c_wr(c, SE_I2C_RX_TRANS_LEN, m->len);
        gi2c_wr(c, SE_GENI_M_CMD0, ((uint32_t)GENI_I2C_CMD_READ << GENI_M_OPCODE_SHIFT) | param);
    } else {
        gi2c_wr(c, SE_I2C_TX_TRANS_LEN, m->len);
        gi2c_wr(c, SE_GENI_M_CMD0, ((uint32_t)GENI_I2C_CMD_WRITE << GENI_M_OPCODE_SHIFT) | param);
        if (m->len) gi2c_wr(c, SE_GENI_TX_WATERMARK_REG, 1);
    }
    // Timeout: 20 ms plus 1 ms per byte (a byte takes 90 us at 100 kHz), polled every 2 us.
    const unsigned limit = (20000u + 1000u * m->len) / 2;
    int err = 0;
    for (unsigned it = 0; it < limit; it++) {
        uint32_t st = gi2c_rd(c, SE_GENI_M_IRQ_STATUS);
        c->last_status |= st;
        if (!m->read && (st & GENI_M_TX_FIFO_WATERMARK)) {
            unsigned words = 0;
            while (done_bytes < m->len && words < c->info.tx_fifo_depth) {
                uint32_t w = 0;
                for (unsigned b = 0; b < 4 && done_bytes < m->len; b++) w |= (uint32_t)m->buf[done_bytes++] << (8 * b);
                gi2c_wr(c, SE_GENI_TX_FIFOn, w);
                words++;
            }
            if (done_bytes >= m->len) gi2c_wr(c, SE_GENI_TX_WATERMARK_REG, 0);
        }
        if (m->read && (st & (GENI_M_RX_FIFO_WATERMARK | GENI_M_RX_FIFO_LAST))) {
            uint32_t wc = gi2c_rd(c, SE_GENI_RX_FIFO_STATUS) & GENI_RX_FIFO_WC;
            for (uint32_t i = 0; i < wc; i++) {
                uint32_t w = gi2c_rd(c, SE_GENI_RX_FIFOn);
                for (unsigned b = 0; b < 4 && done_bytes < m->len; b++) m->buf[done_bytes++] = (uint8_t)(w >> (8 * b));
            }
        }
        if (st) gi2c_wr(c, SE_GENI_M_IRQ_CLEAR, st);
        if (st & GENI_I2C_ERR_BITS)
            err = (st & GENI_M_GP_IRQ_1) ? GENI_ERR_NACK : (st & (GENI_M_GP_IRQ_3 | GENI_M_GP_IRQ_4)) ? GENI_ERR_BUS : GENI_ERR_IO;
        if (st & GENI_M_CMD_DONE) {
            if (!m->read) gi2c_wr(c, SE_GENI_TX_WATERMARK_REG, 0);
            if (!err && done_bytes != m->len) err = GENI_ERR_IO;
            return err;
        }
        gi2c_udelay(c, 2);
    }
    gi2c_stop_cmd(c);
    return err ? err : GENI_ERR_TIMEOUT;
}

extern "C" int geni_i2c_xfer(struct geni_i2c *c, uint8_t addr, struct geni_i2c_msg *msgs, unsigned n) {
    if (!c || !msgs || !n) return GENI_ERR_PARAM;
    c->last_status = 0;
    c->last_cancel = 0;
    for (unsigned i = 0; i < n; i++) {
        if (msgs[i].len && !msgs[i].buf) return GENI_ERR_PARAM;
        int r = gi2c_one(c, addr, &msgs[i], i + 1 < n);
        if (r) return r;
    }
    return 0;
}
