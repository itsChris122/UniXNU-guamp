/* SPDX-License-Identifier: BSD-2-Clause
 * Funzioni comuni tra i percorsi (non API). */
#ifndef GISP_INTERNAL_H
#define GISP_INTERNAL_H
#include "gisp.h"

/* Canale LSC (0 R, 1 Gr, 2 Gb, 3 B) di un sito CFA. */
int gisp__lsc_channel(uint32_t cfa, uint32_t site);
/* n pixel della riga y da x0 (valori a 10 bit). */
void gisp__unpack_row(const gisp_raw *raw, uint32_t y, uint32_t x0, uint32_t n, uint16_t *out);
/* Guadagni LSC per n pixel della riga y da x0; tmp: 4 * lsc_w float. */
void gisp__lsc_row(const gisp_raw *raw, const gisp_params *p, uint32_t y, uint32_t x0, uint32_t n, float *gain,
                   float *tmp);

static inline int gisp__crop(const gisp_raw *raw, const gisp_rect *in, gisp_rect *out)
{
    *out = in && in->w ? *in : (gisp_rect){0, 0, raw->width, raw->height};
    if ((out->x | out->y | out->w | out->h) & 1 || out->x + out->w > raw->width || out->y + out->h > raw->height ||
        out->w < 4 || out->h < 4)
        return -1;
    return 0;
}
#endif
