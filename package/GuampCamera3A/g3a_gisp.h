/* SPDX-License-Identifier: BSD-2-Clause
 * Adapter: guamp-isp statistics (gisp_stats of gisp.h, agent ISP) -> g3a_stats view. Include after gisp.h.
 * Units (gisp_stats.c): sums in 10-bit DN with the black removed and LSC applied, before WB, unsaturated quads only;
 * sharp = sum of |dG| between neighbouring sampled quads in units of the full scale (0..1), saturated quads included.
 * g3a only uses sharp / sum_g ratios (AF), so the scale of sharp does not matter. Host test: tests/g3a_gisp_test.c. */
#ifndef G3A_GISP_H
#define G3A_GISP_H
#include "g3a.h"

static inline void g3a_from_gisp(g3a_stats *v, const gisp_stats *st, const gisp_params *p) {
    float black = 0.f;
    for (int i = 0; i < 4; ++i) black += p->black[i];
    v->zones_x = st->zones_x ? st->zones_x : 16;
    v->zones_y = st->zones_y ? st->zones_y : 12;
    v->sum_r = st->sum_r; v->sum_g = st->sum_g; v->sum_b = st->sum_b;
    v->count = st->count; v->sat = st->sat; v->sharp = st->sharp; v->hist = st->hist;
    /* gisp_stats.range = white - mean black (set by gisp_stats_compute); the parameters only as a fallback */
    v->white = st->range > 0.f ? st->range : p->white - black / 4.f;
}
#endif
