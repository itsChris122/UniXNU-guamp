// SPDX-License-Identifier: MIT
// Bounded GPIO SPI transport, MSB first, modes 0..3. No controller registers.
#ifndef HX_GPIO_SPI_H
#define HX_GPIO_SPI_H
#include <stddef.h>
#include <stdint.h>
struct hx_gpio_spi {
    void *ctx;
    void (*write)(void *, unsigned lane, unsigned level); // 0 MOSI, 1 CLK, 2 CS
    unsigned (*read)(void *);                            // MISO
    void (*delay)(void *, unsigned us);
    unsigned mode;
};
static inline int hx_gpio_transfer(struct hx_gpio_spi *s, const uint8_t *tx, uint8_t *rx, size_t length)
{
    if (!s || !s->write || !s->read || !s->delay || s->mode > 3 || length > 8192 || (length && !tx && !rx)) return -22;
    if (!length) return 0;
    const unsigned idle = (s->mode >> 1) & 1, phase = s->mode & 1;
    s->write(s->ctx, 1, idle);
    s->write(s->ctx, 2, 0);
    s->delay(s->ctx, 2);
    for (size_t i = 0; i < length; ++i) {
        const uint8_t output = tx ? tx[i] : 0;
        uint8_t input = 0;
        for (unsigned bit = 0; bit < 8; ++bit) {
            if (phase) s->write(s->ctx, 1, idle ^ 1);
            s->write(s->ctx, 0, (output >> (7 - bit)) & 1);
            s->delay(s->ctx, 2);
            s->write(s->ctx, 1, phase ? idle : idle ^ 1);
            s->delay(s->ctx, 2);
            input = (uint8_t)(((unsigned)input << 1) | (s->read(s->ctx) & 1u));
            if (!phase) s->write(s->ctx, 1, idle);
        }
        if (rx) rx[i] = input;
    }
    s->delay(s->ctx, 2);
    s->write(s->ctx, 2, 1);
    s->delay(s->ctx, 10);
    return 0;
}
#endif
