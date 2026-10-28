// SPDX-License-Identifier: BSD-2-Clause-Patent
// Qualcomm GENI serial-engine I2C master, FIFO mode, polled. Register layout and sequences follow the study of the
// Linux i2c-qcom-geni driver of the guamp kernel (drivers/i2c/busses/i2c-qcom-geni.c, include/linux/qcom-geni-se.h);
// no code copied. Shares the GENI register names and the geni_io accessors with GeniSpi.h. No board facts here.
#ifndef GENI_I2C_H
#define GENI_I2C_H
#include <stdint.h>
#include <stddef.h>
#include "GeniSpi.h"

#define SE_I2C_TX_TRANS_LEN 0x26c
#define SE_I2C_RX_TRANS_LEN 0x270
#define SE_I2C_SCL_COUNTERS 0x278

#define GENI_PROTO_I2C 3
#define GENI_I2C_CMD_WRITE 1
#define GENI_I2C_CMD_READ 2
#define GENI_I2C_STOP_STRETCH (1u<<2)         // repeated start: no STOP after this message
#define GENI_I2C_SLV_ADDR_SHIFT 9
#define GENI_M_GP_IRQ_1 (1u<<10)              // NACK
#define GENI_M_GP_IRQ_3 (1u<<12)              // bus protocol error
#define GENI_M_GP_IRQ_4 (1u<<13)              // arbitration lost
#define GENI_I2C_ERR_BITS (GENI_M_GP_IRQ_1 | GENI_M_GP_IRQ_3 | GENI_M_GP_IRQ_4 | GENI_M_CMD_OVERRUN | GENI_M_ILLEGAL_CMD | GENI_M_CMD_FAILURE)
#define GENI_RX_FIFO_WC 0x1ffffffu

#define GENI_ERR_NACK (-6)                    // ENXIO: no device answered the address (or a data byte)
#define GENI_ERR_BUS (-16)                    // EBUSY: bus protocol error / arbitration lost

struct geni_i2c_msg {
    uint8_t *buf;
    uint16_t len;
    uint8_t read;
};

struct geni_i2c {
    struct geni_io se;
    struct geni_io wrapper;
    struct geni_spi_info info;         // same fields as the SPI engine (fw revision, protocol, FIFO depth/width)
    int allow_fifo_disabled;           // try FIFO mode although FIFO_IF_DISABLE is set (Linux would use GSI DMA)
    unsigned khz;                      // bus speed set by geni_i2c_set_clock
    uint32_t last_status;              // M_IRQ_STATUS bits seen in the last transfer (errors, done)
    int last_cancel;                   // 0 none, 1 cancel worked, 2 abort needed, 3 abort timed out
};

#ifdef __cplusplus
extern "C" {
#endif
int geni_i2c_read_info(struct geni_i2c *c);
int geni_i2c_init(struct geni_i2c *c);
// Linux geni_i2c_clk_map: SE clock 19.2 MHz; 100, 400 or 1000 kHz. clk_sel = DFS level of the 19.2 MHz rate.
int geni_i2c_set_clock(struct geni_i2c *c, unsigned khz, unsigned clk_sel);
// One I2C transaction: msgs[0..n-1] to the 7-bit address, repeated start between them, STOP at the end.
int geni_i2c_xfer(struct geni_i2c *c, uint8_t addr, struct geni_i2c_msg *msgs, unsigned n);
#ifdef __cplusplus
}
#endif
#endif
