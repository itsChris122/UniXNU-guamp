// SPDX-License-Identifier: BSD-2-Clause-Patent
// GENI UART, FIFO mode, polled (GeniUart.h). Helper names carry a gu_ prefix: GeniSpiCpp.inc already defines rd/wr in
// the same translation unit.
#include "GeniUart.h"

static uint32_t gu_rd(struct geni_uart *u, uint32_t reg) { return u->se.rd(u->se.ctx, reg); }
static void gu_wr(struct geni_uart *u, uint32_t reg, uint32_t v) { u->se.wr(u->se.ctx, reg, v); }
static void gu_set(struct geni_uart *u, uint32_t reg, uint32_t bits) { gu_wr(u, reg, gu_rd(u, reg) | bits); }
static void gu_delay(struct geni_uart *u, unsigned us) { if (u->se.delay_us) u->se.delay_us(u->se.ctx, us); }
static unsigned gu_step(const struct geni_uart *u) { return u->poll_us ? u->poll_us : 10u; }

void geni_uart_packing(unsigned bpw, unsigned pack_words, int msb_first, uint32_t *cfg0, uint32_t *cfg1) {
    uint32_t cfg[GENI_PACK_VECTORS] = {0, 0, 0, 0};
    int idx_start = msb_first ? (int)bpw - 1 : 0, idx = idx_start, idx_delta = msb_first ? -8 : 8;
    unsigned iter = ((bpw + 7) & ~7u) * pack_words / 8;
    int temp_bpw = (int)bpw;
    *cfg0 = *cfg1 = 0;
    if (iter == 0 || iter > GENI_PACK_VECTORS) return;
    for (unsigned i = 0; i < iter; i++) {
        int len = (temp_bpw < 8 ? temp_bpw : 8) - 1;
        cfg[i] = ((uint32_t)idx << GENI_PACK_START_SHIFT) | ((uint32_t)(msb_first != 0) << GENI_PACK_DIR_SHIFT) |
                 ((uint32_t)len << GENI_PACK_LEN_SHIFT);
        if (temp_bpw <= 8) {
            idx = (int)(i + 1) * 8 + idx_start;
            temp_bpw = (int)bpw;
        } else {
            idx = idx + idx_delta;
            temp_bpw = temp_bpw - 8;
        }
    }
    cfg[iter - 1] |= GENI_PACK_STOP;
    *cfg0 = cfg[0] | (cfg[1] << GENI_PACK_VECTOR_SHIFT);
    *cfg1 = cfg[2] | (cfg[3] << GENI_PACK_VECTOR_SHIFT);
}

int geni_uart_read_info(struct geni_uart *u) {
    if (!u || !u->se.rd || !u->wrapper.rd) return GENI_ERR_PARAM;
    struct geni_uart_info *i = &u->info;
    i->fw_revision = gu_rd(u, GENI_FW_REVISION_RO);
    i->protocol = (i->fw_revision >> GENI_FW_PROTO_SHIFT) & 0xff;
    i->if_disable = gu_rd(u, GENI_IF_DISABLE_RO);
    i->fifo_disabled = (i->if_disable & GENI_FIFO_IF_DISABLE) != 0;
    i->geni_status = gu_rd(u, SE_GENI_STATUS);
    i->hw_param0 = gu_rd(u, SE_HW_PARAM_0);
    i->hw_param1 = gu_rd(u, SE_HW_PARAM_1);
    i->qup_hw_version = u->wrapper.rd(u->wrapper.ctx, QUPV3_HW_VER_REG);
    uint32_t major = i->qup_hw_version >> 28, minor = (i->qup_hw_version >> 16) & 0xfff;
    uint32_t mask = ((major == 3 && minor >= 10) || major > 3) ? 0xff0000u : 0x3f0000u;
    i->tx_fifo_depth = (i->hw_param0 & mask) >> 16;
    i->rx_fifo_depth = (i->hw_param1 & mask) >> 16;
    i->fifo_width_bits = (i->hw_param0 >> 24) & 0x3f;
    return 0;
}

static void gu_irq_clear(struct geni_uart *u) {
    gu_wr(u, SE_GSI_EVENT_EN, 0);
    gu_wr(u, SE_GENI_M_IRQ_CLEAR, 0xffffffffu);
    gu_wr(u, SE_GENI_S_IRQ_CLEAR, 0xffffffffu);
    gu_wr(u, SE_DMA_TX_IRQ_CLR, 0xffffffffu);
    gu_wr(u, SE_DMA_RX_IRQ_CLR, 0xffffffffu);
}

int geni_uart_init(struct geni_uart *u) {
    int r = geni_uart_read_info(u);
    if (r) return r;
    if (u->info.protocol != GENI_PROTO_UART) return GENI_ERR_PROTO;
    if (u->info.fifo_disabled) return GENI_ERR_NOTSUP;               // SE DMA path: later milestone
    if (u->info.tx_fifo_depth < 4 || u->info.rx_fifo_depth < 4 || u->info.fifo_width_bits != 32) return GENI_ERR_NODEV;
    u->rx_dropped = 0;
    // msm_geni_serial_port_setup: stale counter, packing (8 bits, 4 per word, LSB first), then geni_se_init
    gu_wr(u, SE_UART_RX_STALE_CNT, GU_STALE_COUNT);
    uint32_t cfg0, cfg1;
    geni_uart_packing(8, 4, 0, &cfg0, &cfg1);
    gu_wr(u, SE_GENI_TX_PACKING_CFG0, cfg0);
    gu_wr(u, SE_GENI_TX_PACKING_CFG1, cfg1);
    gu_wr(u, SE_GENI_RX_PACKING_CFG0, cfg0);
    gu_wr(u, SE_GENI_RX_PACKING_CFG1, cfg1);
    gu_wr(u, SE_GENI_BYTE_GRAN, 0);
    gu_irq_clear(u);
    gu_set(u, SE_GENI_CGC_CTRL, GENI_DEFAULT_CGC_EN);
    gu_set(u, SE_DMA_GENERAL_CFG, GENI_AHB_SEC_SLV_CGC_ON | GENI_DMA_AHB_SLV_CGC_ON | GENI_DMA_TX_CGC_ON | GENI_DMA_RX_CGC_ON);
    gu_wr(u, GENI_OUTPUT_CTRL, GENI_DEFAULT_IO_OUTPUT_CTRL);
    gu_wr(u, GENI_FORCE_DEFAULT_REG, GENI_FORCE_DEFAULT);
    gu_wr(u, SE_GENI_RX_WATERMARK_REG, u->info.rx_fifo_depth >> 1);
    gu_wr(u, SE_GENI_RX_RFR_WATERMARK_REG, u->info.rx_fifo_depth - 2);
    gu_set(u, SE_GENI_M_IRQ_EN, GENI_M_COMMON_IRQ_EN);
    gu_set(u, SE_GENI_S_IRQ_EN, GENI_S_COMMON_IRQ_EN);
    // geni_se_select_mode(FIFO): status bits of the FIFO events, no DMA
    gu_irq_clear(u);
    gu_set(u, SE_GENI_M_IRQ_EN, GENI_M_CMD_DONE | GENI_M_TX_FIFO_WATERMARK | GENI_M_RX_FIFO_WATERMARK | GENI_M_RX_FIFO_LAST);
    gu_set(u, SE_GENI_S_IRQ_EN, GU_S_CMD_DONE | GU_S_RX_FIFO_WATERMARK | GU_S_RX_FIFO_LAST);
    gu_wr(u, SE_GENI_DMA_MODE_EN, gu_rd(u, SE_GENI_DMA_MODE_EN) & ~GENI_DMA_MODE_EN);
    gu_wr(u, SE_IRQ_EN, 0);                                           // polled: nothing reaches the GIC
    // set_termios: 8N1, no parity, no loopback; flow control off until the controller is powered
    gu_wr(u, SE_UART_LOOPBACK_CFG, 0);
    gu_wr(u, SE_UART_TX_TRANS_CFG, GU_TX_CTS_MASK);
    gu_wr(u, SE_UART_TX_PARITY_CFG, 0);
    gu_wr(u, SE_UART_RX_TRANS_CFG, 0);
    gu_wr(u, SE_UART_RX_PARITY_CFG, 0);
    gu_wr(u, SE_UART_TX_WORD_LEN, 8);
    gu_wr(u, SE_UART_RX_WORD_LEN, 8);
    gu_wr(u, SE_UART_TX_STOP_BIT_LEN, 0);
    geni_uart_flow(u, 0);
    return 0;
}

int geni_uart_set_clock(struct geni_uart *u, unsigned div, unsigned clk_sel) {
    if (!div || div > GENI_CLK_DIV_MAX || clk_sel > 7) return GENI_ERR_PARAM;
    const uint32_t v = ((uint32_t)div << GENI_CLK_DIV_SHIFT) | GENI_SER_CLK_EN;
    gu_wr(u, SE_GENI_CLK_SEL, clk_sel);
    gu_wr(u, GENI_SER_M_CLK_CFG, v);
    gu_wr(u, GENI_SER_S_CLK_CFG, v);
    (void)gu_rd(u, GENI_SER_M_CLK_CFG);
    return 0;
}

void geni_uart_flow(struct geni_uart *u, int hw) {
    if (hw) {
        gu_wr(u, SE_UART_TX_TRANS_CFG, gu_rd(u, SE_UART_TX_TRANS_CFG) & ~GU_TX_CTS_MASK);
        gu_wr(u, SE_UART_MANUAL_RFR, 0);
    } else {
        gu_wr(u, SE_UART_TX_TRANS_CFG, gu_rd(u, SE_UART_TX_TRANS_CFG) | GU_TX_CTS_MASK);
        gu_wr(u, SE_UART_MANUAL_RFR, GU_MANUAL_RFR_EN);
        gu_delay(u, 10);                                              // "UART FW needs delay" (msm_geni_serial.c)
        gu_wr(u, SE_UART_MANUAL_RFR, GU_MANUAL_RFR_EN | GU_RFR_NOT_READY);
    }
}

static int gu_wait(struct geni_uart *u, uint32_t reg, uint32_t bits, unsigned timeout_us, uint32_t *seen) {
    const unsigned step = gu_step(u);
    for (unsigned t = 0;; t = t + step) {
        uint32_t v = gu_rd(u, reg);
        if (seen) *seen = v;
        if (v & bits) return 0;
        if (t >= timeout_us) return GENI_ERR_TIMEOUT;
        gu_delay(u, step);
    }
}

int geni_uart_rx_stop(struct geni_uart *u, unsigned timeout_us) {
    if (!(gu_rd(u, SE_GENI_STATUS) & GU_STATUS_S_ACTIVE)) return 0;
    gu_wr(u, SE_GENI_S_CMD_CTRL_REG, GU_S_CMD_CANCEL_REQ);
    int r = gu_wait(u, SE_GENI_S_IRQ_STATUS, GU_S_CMD_CANCEL, timeout_us, &u->last_s_irq);
    if (r) {
        gu_wr(u, SE_GENI_S_CMD_CTRL_REG, GU_S_CMD_ABORT_REQ);
        r = gu_wait(u, SE_GENI_S_IRQ_STATUS, GU_S_CMD_ABORT, timeout_us, &u->last_s_irq);
    }
    gu_wr(u, SE_GENI_S_IRQ_CLEAR, 0xffffffffu);
    gu_wr(u, SE_GENI_M_IRQ_CLEAR, 0xffffffffu);
    return r;
}

int geni_uart_rx_start(struct geni_uart *u) {
    int r = geni_uart_rx_stop(u, 2000);
    gu_wr(u, SE_GENI_S_CMD0, (GU_START_READ << GENI_M_OPCODE_SHIFT) | GU_PARAM_RFR_OPEN);
    return r;
}

int geni_uart_write(struct geni_uart *u, const uint8_t *buf, size_t len, unsigned timeout_us) {
    if (!len || len > GENI_TRANS_LEN_MASK) return GENI_ERR_PARAM;
    const unsigned step = gu_step(u), depth = u->info.tx_fifo_depth;
    gu_wr(u, SE_GENI_M_IRQ_CLEAR, 0xffffffffu);
    gu_wr(u, SE_UART_TX_TRANS_LEN, (uint32_t)len);
    gu_wr(u, SE_GENI_M_CMD0, GU_START_TX << GENI_M_OPCODE_SHIFT);
    size_t at = 0;
    unsigned t = 0;
    int r = 0;
    while (at < len) {
        uint32_t used = gu_rd(u, SE_GENI_TX_FIFO_STATUS) & GU_TX_FIFO_WC_MASK;
        if (used >= depth) {
            if (t >= timeout_us) { r = GENI_ERR_TIMEOUT; break; }
            gu_delay(u, step);
            t = t + step;
            continue;
        }
        uint32_t word = 0;
        for (unsigned j = 0; j < 4 && at < len; ++j, ++at) word |= (uint32_t)buf[at] << (8 * j);
        gu_wr(u, SE_GENI_TX_FIFOn, word);
    }
    if (!r) r = gu_wait(u, SE_GENI_M_IRQ_STATUS, GENI_M_CMD_DONE, timeout_us > t ? timeout_us - t : 0, &u->last_m_irq);
    if (r) {
        // msm_geni_serial_poll_tx_done: cancel, then abort the stuck command (CTS never asserted, clock off, ...)
        gu_wr(u, SE_GENI_M_CMD_CTRL_REG, GENI_M_CMD_CANCEL);
        if (gu_wait(u, SE_GENI_M_IRQ_STATUS, GENI_M_CMD_CANCEL_DONE, 2000, NULL)) {
            gu_wr(u, SE_GENI_M_CMD_CTRL_REG, GENI_M_CMD_ABORT);
            (void)gu_wait(u, SE_GENI_M_IRQ_STATUS, GENI_M_CMD_ABORT_DONE, 2000, NULL);
        }
    }
    gu_wr(u, SE_GENI_M_IRQ_CLEAR, 0xffffffffu);
    return r;
}

size_t geni_uart_read(struct geni_uart *u, uint8_t *buf, size_t cap) {
    uint32_t st = gu_rd(u, SE_GENI_RX_FIFO_STATUS);
    uint32_t wc = st & GENI_RX_WC_MASK;
    if (!wc) return 0;
    const unsigned last_valid = (st >> GENI_RX_LAST_VALID_SHIFT) & GENI_RX_LAST_VALID_MASK;
    size_t n = 0;
    for (uint32_t i = 0; i < wc; ++i) {
        uint32_t word = gu_rd(u, SE_GENI_RX_FIFOn);
        unsigned bytes = 4;
        if (i == wc - 1 && (st & GENI_RX_LAST) && last_valid) bytes = last_valid;
        for (unsigned j = 0; j < bytes; ++j) {
            if (n < cap) buf[n++] = (uint8_t)(word >> (8 * j));
            else u->rx_dropped = u->rx_dropped + 1;
        }
    }
    gu_wr(u, SE_GENI_S_IRQ_CLEAR, gu_rd(u, SE_GENI_S_IRQ_STATUS));
    gu_wr(u, SE_GENI_M_IRQ_CLEAR, gu_rd(u, SE_GENI_M_IRQ_STATUS));
    return n;
}
