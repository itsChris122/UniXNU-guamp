// SPDX-License-Identifier: BSD-2-Clause-Patent
// Qualcomm GENI serial-engine UART, FIFO (PIO) mode, polled. Register layout and sequences follow the study of the
// Linux msm_geni_serial.c / qcom-geni-se.c drivers of the guamp vendor tree (no code copied). The common GENI registers
// come from GeniSpi.h; no board addresses, clocks or pins here. Host-tested by tests/bt_host.cpp (fake registers).
#ifndef GENI_UART_H
#define GENI_UART_H
#include <stdint.h>
#include <stddef.h>
#include "GeniSpi.h"

#define GENI_PROTO_UART 2
#define GENI_SER_S_CLK_CFG 0x4c
#define SE_HW_PARAM_1 0xe28
#define SE_GENI_S_CMD0 0x630
#define SE_GENI_S_CMD_CTRL_REG 0x634
#define SE_GENI_S_IRQ_STATUS 0x640
// UART engine registers (msm_geni_serial.c)
#define SE_UART_LOOPBACK_CFG 0x22c
#define SE_UART_TX_TRANS_CFG 0x25c
#define SE_UART_TX_WORD_LEN 0x268
#define SE_UART_TX_STOP_BIT_LEN 0x26c
#define SE_UART_TX_TRANS_LEN 0x270
#define SE_UART_RX_TRANS_CFG 0x280
#define SE_UART_RX_WORD_LEN 0x28c
#define SE_UART_RX_STALE_CNT 0x294
#define SE_UART_TX_PARITY_CFG 0x2a4
#define SE_UART_RX_PARITY_CFG 0x2a8
#define SE_UART_MANUAL_RFR 0x2ac
#define GU_TX_PAR_EN (1u<<0)
#define GU_TX_CTS_MASK (1u<<1)            // set: transmit regardless of CTS (no hardware flow control)
#define GU_RX_PAR_EN (1u<<3)
#define GU_PAR_CALC_EN (1u<<0)
#define GU_MANUAL_RFR_EN (1u<<31)
#define GU_RFR_NOT_READY (1u<<1)
#define GU_START_TX 1u                    // M command
#define GU_START_READ 1u                  // S command
#define GU_PARAM_RFR_OPEN (1u<<7)         // Linux start_rx_sequencer: keep RFR in the ready state
#define GU_STALE_COUNT (10u * 16u)        // DEFAULT_BITS_PER_CHAR * STALE_TIMEOUT
// GENI status / secondary sequencer
#define GU_STATUS_M_ACTIVE (1u<<0)
#define GU_STATUS_S_ACTIVE (1u<<12)
#define GU_S_CMD_DONE (1u<<0)
#define GU_S_CMD_CANCEL (1u<<4)
#define GU_S_CMD_ABORT (1u<<5)
#define GU_S_RX_FIFO_WATERMARK (1u<<26)
#define GU_S_RX_FIFO_LAST (1u<<27)
#define GU_S_CMD_CANCEL_REQ (1u<<2)       // S_CMD_CTRL
#define GU_S_CMD_ABORT_REQ (1u<<1)
#define GU_TX_FIFO_WC_MASK 0x0fffffffu

struct geni_uart_info {
    uint32_t fw_revision, protocol, if_disable, hw_param0, hw_param1, geni_status, qup_hw_version;
    unsigned tx_fifo_depth, rx_fifo_depth, fifo_width_bits;
    int fifo_disabled;
};

struct geni_uart {
    struct geni_io se, wrapper;
    struct geni_uart_info info;
    unsigned poll_us;                     // granularity of every wait (default 10 us)
    uint32_t last_m_irq, last_s_irq;      // IRQ status seen by the latest write / stop
    unsigned rx_dropped;                  // bytes read from the FIFO without room in the caller's buffer
};

// Read-only (valid only with the SE clocks running): protocol, FIFO geometry, FIFO-disable flag, versions.
int geni_uart_read_info(struct geni_uart *u);
// geni_se_init + FIFO mode + 8-bit packing (4 per FIFO word, LSB first) + 8N1, no loopback, CTS ignored, RFR manual
// not-ready; engine interrupt outputs off (polled). Refuses an engine not loaded with UART firmware or with FIFO
// disabled.
int geni_uart_init(struct geni_uart *u);
// Divider (1..0xfff) from the SE source clock and the DFS level / source index.
int geni_uart_set_clock(struct geni_uart *u, unsigned div, unsigned clk_sel);
// hw != 0: RTS/CTS by hardware (Linux CRTSCTS); 0: CTS ignored, RFR forced not-ready (Linux manual flow).
void geni_uart_flow(struct geni_uart *u, int hw);
int geni_uart_rx_start(struct geni_uart *u);
int geni_uart_rx_stop(struct geni_uart *u, unsigned timeout_us);
// One M-sequencer TX of len bytes, polled to CMD_DONE; on timeout the command is cancelled (then aborted).
int geni_uart_write(struct geni_uart *u, const uint8_t *buf, size_t len, unsigned timeout_us);
// Non-blocking: moves what the RX FIFO holds now into buf (at most cap bytes); returns the byte count.
size_t geni_uart_read(struct geni_uart *u, uint8_t *buf, size_t cap);
// Packing vectors of qcom-geni-se.c se_get_packing_config (pure).
void geni_uart_packing(unsigned bpw, unsigned pack_words, int msb_first, uint32_t *cfg0, uint32_t *cfg1);
#endif
