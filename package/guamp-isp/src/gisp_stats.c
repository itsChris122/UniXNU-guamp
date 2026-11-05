/* SPDX-License-Identifier: BSD-2-Clause
 * Statistiche 3A sul RAW (per zona: medie R/G/B, saturazione, contrasto AF; istogramma della luma), formato IPC,
 * AWB di base, file di tuning .gisp. */
#include "gisp.h"
#include "gisp_internal.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* coppia di pixel (x pari) di una riga */
static inline void pair(const gisp_raw *raw, uint32_t y, uint32_t x, float *a, float *b)
{
    const uint8_t *row = raw->data + (size_t)y * raw->stride;
    if (raw->format == GISP_RAW16) {
        const uint8_t *q = row + 2 * (size_t)x;
        *a = (float)(q[0] | q[1] << 8);
        *b = (float)(q[2] | q[3] << 8);
        return;
    }
    const uint8_t *g = row + (x >> 2) * 5;
    uint32_t k = x & 3; /* 0 o 2: i due pixel stanno nello stesso gruppo da 5 byte */
    *a = (float)(g[k] << 2 | ((g[4] >> (2 * k)) & 3));
    *b = (float)(g[k + 1] << 2 | ((g[4] >> (2 * k + 2)) & 3));
}

#define GS_LSC_ROWS 16 /* righe RAW per blocco di guadagni LSC */

void gisp_stats_compute(const gisp_raw *raw, const gisp_rect *crop_in, const gisp_params *p, gisp_stats *st)
{
    uint32_t zx = st->zones_x ? st->zones_x : 16, zy = st->zones_y ? st->zones_y : 12;
    uint32_t step = st->step ? st->step : 2;
    if (zx > 32) zx = 32;
    if (zy > 32) zy = 32;
    memset(st, 0, sizeof *st);
    st->zones_x = zx; st->zones_y = zy; st->step = step;
    gisp_rect crop;
    if (gisp__crop(raw, crop_in, &crop)) return;
    const uint32_t qw = crop.w / 2, qh = crop.h / 2, cw = crop.w;
    const uint32_t nqx = (qw + step - 1) / step;
    const int lsc = p->lsc_w && p->lsc_h && p->lsc_w * p->lsc_h <= GISP_LSC_MAX;
    float *gain = lsc ? malloc(sizeof(float) * (2 * cw + 4 * GISP_LSC_MAX)) : NULL;
    float *gprev = calloc(2 * (size_t)nqx, sizeof(float)); /* verde dei quadrati campionati: riga sopra e due righe sopra */
    uint16_t *zcol = malloc(sizeof(uint16_t) * nqx);
    if ((lsc && !gain) || !gprev || !zcol) { free(gain); free(gprev); free(zcol); return; }
    for (uint32_t i = 0; i < nqx; i++) zcol[i] = (uint16_t)(i * step * zx / qw);

    int col[4];
    for (int s = 0; s < 4; s++) col[s] = gisp_cfa_color(raw->cfa, s);
    const int mono = raw->cfa == GISP_MONO;
    const float satv = p->white - 2;
    const float range = p->white - (p->black[0] + p->black[1] + p->black[2] + p->black[3]) * 0.25f;
    const float inv = range > 0 ? 1.0f / range : 0;
    const float lr = 0.2126f * p->wb[0] * inv, lg = 0.7152f * p->wb[1] * inv, lb = 0.0722f * p->wb[2] * inv;
    double luma_sum = 0;
    uint32_t rows_seen = 0;
    int32_t gtag = -1;

    for (uint32_t qy = 0; qy < qh; qy += step) {
        const uint32_t y = crop.y + 2 * qy;
        if (lsc && (int32_t)(y / GS_LSC_ROWS) != gtag) {
            /* guadagni a meta' del blocco (stessa parita' di riga) */
            gtag = (int32_t)(y / GS_LSC_ROWS);
            uint32_t yc = gtag * GS_LSC_ROWS + GS_LSC_ROWS / 2;
            if (yc + 1 >= raw->height) yc = (raw->height - 2) & ~1u;
            gisp__lsc_row(raw, p, yc, crop.x, cw, gain, gain + 2 * cw);
            gisp__lsc_row(raw, p, yc + 1, crop.x, cw, gain + cw, gain + 2 * cw);
        }
        const uint32_t zrow = (qy * zy / qh) * zx;
        float gleft = 0, gleft2 = 0;
        float *g1 = gprev + (size_t)(rows_seen & 1) * nqx, *g2 = gprev + (size_t)((rows_seen + 1) & 1) * nqx;
        for (uint32_t i = 0; i < nqx; i++) {
            const uint32_t x = 2 * i * step;
            float v[4];
            pair(raw, y, crop.x + x, &v[0], &v[1]);
            pair(raw, y + 1, crop.x + x, &v[2], &v[3]);
            const int sat = v[0] >= satv || v[1] >= satv || v[2] >= satv || v[3] >= satv;
            float c[3] = {0, 0, 0};
            for (int s = 0; s < 4; s++) {
                float d = v[s] - p->black[s];
                d = d < 0 ? 0 : d;
                if (lsc) d *= gain[(s >> 1) * cw + x + (s & 1)];
                c[col[s]] += d;
            }
            if (!mono) c[1] *= 0.5f;
            else { c[0] = c[2] = c[1] = c[1] * 0.25f; }
            const uint32_t z = zrow + zcol[i];
            /* contrasto AF: seconda differenza del verde (|g - 2 g1 + g2|) in orizzontale e in verticale tra quadrati
             * campionati. La prima differenza |dG| sommata su un bordo vale il salto del bordo con o senza sfocatura
             * (variazione totale): non distingue il fuoco. La seconda scende come 1 / larghezza del bordo. */
            const float g = c[1] * inv;
            float d = 0;
            if (i >= 2) d += fabsf(g - 2.0f * gleft + gleft2);
            if (rows_seen >= 2) d += fabsf(g - 2.0f * g1[i] + g2[i]);
            gleft2 = gleft;
            gleft = g;
            g2[i] = g; /* g2 diventa la riga "sopra" al prossimo giro (scambio dei due buffer) */
            st->sharp[z] += d;
            float yl = lr * c[0] + lg * c[1] + lb * c[2];
            yl = yl > 1 ? 1 : yl;
            luma_sum += yl;
            st->hist[(uint32_t)(yl * 255.0f + 0.5f)]++;
            if (sat) { st->sat[z]++; continue; }
            st->sum_r[z] += c[0]; st->sum_g[z] += c[1]; st->sum_b[z] += c[2];
            st->count[z]++;
        }
        st->total += nqx;
        rows_seen++;
    }
    st->mean_luma = st->total ? (float)(luma_sum / st->total) : 0;
    st->range = range;
    free(gain); free(gprev); free(zcol);
}

static uint16_t q12(float v)
{
    float q = v * 4096.0f + 0.5f;
    return q <= 0 ? 0 : q >= 65535 ? 65535 : (uint16_t)q;
}

int gisp_stats_to_wire(const gisp_stats *st, uint32_t frame, gisp_stats_wire *w)
{
    memset(w, 0, sizeof *w);
    w->magic = GISP_WIRE_MAGIC;
    w->size = sizeof *w;
    w->frame = frame;
    w->zones_x = GISP_WIRE_ZX; w->zones_y = GISP_WIRE_ZY;
    if (st->zones_x != GISP_WIRE_ZX || st->zones_y != GISP_WIRE_ZY) return -1;
    w->total = st->total;
    w->mean_luma_q12 = q12(st->mean_luma);
    const float range = st->range > 0 ? st->range : 1023.0f - 64.0f;
    uint32_t sat_total = 0;
    for (uint32_t z = 0; z < GISP_WIRE_ZX * GISP_WIRE_ZY; z++) {
        uint32_t n = st->count[z], all = st->count[z] + st->sat[z];
        sat_total += st->sat[z];
        if (n) {
            w->r[z] = q12(st->sum_r[z] / n / range);
            w->g[z] = q12(st->sum_g[z] / n / range);
            w->b[z] = q12(st->sum_b[z] / n / range);
        }
        if (all) {
            w->sat[z] = (uint8_t)((st->sat[z] * 255u + all / 2) / all);
            w->valid[z] = (uint8_t)((n * 255u + all / 2) / all);
            double s = st->sharp[z] / all * 65536.0;
            w->sharp[z] = s > 4e9 ? 4000000000u : (uint32_t)s;
        }
    }
    w->sat_permille = st->total ? (uint16_t)((sat_total * 1000ull + st->total / 2) / st->total) : 0;
    for (int i = 0; i < 256; i++) w->hist[i / 4] += st->hist[i];
    uint64_t acc = 0;
    int p50 = -1, p95 = -1;
    for (int i = 0; i < 256; i++) {
        acc += st->hist[i];
        if (p50 < 0 && acc * 2 >= st->total) p50 = i;
        if (p95 < 0 && acc * 100 >= (uint64_t)st->total * 95) p95 = i;
    }
    w->p50_q12 = q12((p50 < 0 ? 0 : p50) / 255.0f);
    w->p95_q12 = q12((p95 < 0 ? 0 : p95) / 255.0f);
    return 0;
}

/* ---- AWB di base ------------------------------------------------------------------------------------------------ */
/* Distanza (in log R/G, log B/G) dalla spezzata dei punti di riferimento e posizione lungo di essa (segmento + t). */
static float locus_dist(const gisp_tuning *t, float lrg, float lbg, uint32_t *seg, float *tt)
{
    float best = 1e9f;
    *seg = 0; *tt = 0;
    if (t->n_ref == 1) {
        float dx = lrg - logf(t->ref_rg[0]), dy = lbg - logf(t->ref_bg[0]);
        return sqrtf(dx * dx + dy * dy);
    }
    for (uint32_t i = 0; i + 1 < t->n_ref; i++) {
        float ax = logf(t->ref_rg[i]), ay = logf(t->ref_bg[i]);
        float bx = logf(t->ref_rg[i + 1]), by = logf(t->ref_bg[i + 1]);
        float vx = bx - ax, vy = by - ay, l2 = vx * vx + vy * vy;
        float u = l2 > 0 ? ((lrg - ax) * vx + (lbg - ay) * vy) / l2 : 0;
        if (u < 0) u = 0;
        if (u > 1) u = 1;
        float dx = lrg - (ax + u * vx), dy = lbg - (ay + u * vy);
        float d = sqrtf(dx * dx + dy * dy);
        if (d < best) { best = d; *seg = i; *tt = u; }
    }
    return best;
}

int gisp_awb_estimate(const gisp_stats *st, const gisp_tuning *t, float wb[3], float *cct)
{
    const uint32_t nz = st->zones_x * st->zones_y;
    double sr = 0, sg = 0, sb = 0, ar = 0, ag = 0, ab = 0;
    uint32_t used = 0, any = 0;
    /* soglia di buio: 1% del fondo scala nominale sulla media del verde */
    const float dark = 0.01f * (st->range > 0 ? st->range : 1023.0f - 64.0f);
    for (uint32_t z = 0; z < nz; z++) {
        uint32_t n = st->count[z];
        if (n < 4) continue;
        float r = st->sum_r[z] / n, g = st->sum_g[z] / n, b = st->sum_b[z] / n;
        if (g < dark || r <= 0 || b <= 0) continue;
        any++;
        ar += r * n; ag += g * n; ab += b * n;
        if (t && t->n_ref) {
            uint32_t seg; float tt;
            if (locus_dist(t, logf(r / g), logf(b / g), &seg, &tt) > 0.15f) continue;
        }
        sr += r * n; sg += g * n; sb += b * n;
        used++;
    }
    if (!used) {
        if (!any) return -1;
        sr = ar; sg = ag; sb = ab; /* nessuna zona vicina al luogo: grigio del mondo su tutto */
    }
    if (sr <= 0 || sb <= 0) return -1;
    wb[0] = (float)(sg / sr); wb[1] = 1; wb[2] = (float)(sg / sb);
    if (cct) {
        *cct = 0;
        if (t && t->n_ref >= 2) {
            uint32_t seg; float tt;
            locus_dist(t, logf((float)(sr / sg)), logf((float)(sb / sg)), &seg, &tt);
            float c0 = t->ref_cct[seg], c1 = t->ref_cct[seg + 1];
            if (c0 > 0 && c1 > 0) {
                float m = 1e6f / c0 + (1e6f / c1 - 1e6f / c0) * tt; /* interpolazione in mired */
                *cct = 1e6f / m;
            }
        }
    }
    return 0;
}

/* ---- tuning ----------------------------------------------------------------------------------------------------- */
int gisp_tuning_parse(const void *data, size_t size, gisp_tuning *t)
{
    const uint8_t *d = data;
    uint32_t h[3];
    if (size < 12) return -1;
    memcpy(h, d, 12);
    if (h[0] != GISP_TUNING_MAGIC || h[1] != GISP_TUNING_VERSION || h[2] != sizeof *t || size < 12 + sizeof *t)
        return -2;
    memcpy(t, d + 12, sizeof *t);
    t->sensor[sizeof t->sensor - 1] = 0;
    if (t->lsc_w * t->lsc_h > GISP_LSC_MAX || t->n_ccm > GISP_TUNING_MAX_CCM || t->n_ref > GISP_TUNING_MAX_REF ||
        t->cfa > GISP_MONO || !(t->white > t->black[0]))
        return -3;
    return 0;
}

void gisp_tuning_apply(const gisp_tuning *t, float cct, gisp_params *p)
{
    memcpy(p->black, t->black, sizeof p->black);
    p->white = t->white;
    p->lsc_w = t->lsc_w; p->lsc_h = t->lsc_h;
    memcpy(p->lsc, t->lsc, sizeof p->lsc);
    memcpy(p->gamma, t->gamma, sizeof p->gamma);
    if (!t->n_ccm) return;
    /* CCM: interpolazione lineare in mired tra le due piu' vicine (ccm_cct in ordine qualsiasi) */
    float m = 1e6f / (cct > 1000 ? cct : 5000);
    int lo = -1, hi = -1;
    for (uint32_t i = 0; i < t->n_ccm; i++) {
        float mi = 1e6f / t->ccm_cct[i];
        if (mi <= m && (lo < 0 || mi > 1e6f / t->ccm_cct[lo])) lo = (int)i;
        if (mi >= m && (hi < 0 || mi < 1e6f / t->ccm_cct[hi])) hi = (int)i;
    }
    if (lo < 0) lo = hi;
    if (hi < 0) hi = lo;
    float w = 0;
    if (lo != hi) {
        float ml = 1e6f / t->ccm_cct[lo], mh = 1e6f / t->ccm_cct[hi];
        w = (m - ml) / (mh - ml);
    }
    for (int k = 0; k < 9; k++) p->ccm[k] = t->ccm[lo][k] + (t->ccm[hi][k] - t->ccm[lo][k]) * w;
}
