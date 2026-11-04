/* SPDX-License-Identifier: BSD-2-Clause
 * Percorso fast (anteprima), tutto in interi:
 *   RAW10/RAW16 -> lineare 14 bit (nero, LSC, WB in un solo guadagno Q8 per pixel)
 *   -> demosaic bilineare (FULL) o superpixel 2x2 (BIN2), piani R/G/B uint16
 *   -> scala orizzontale (area se riduce, bilineare se ingrandisce, pesi Q14) in lineare
 *   -> anello di righe -> scala verticale -> CCM Q10 -> gamma (LUT 4096 -> 8 bit) -> Y e CbCr 4:2:0 (NV12).
 * Il lavoro e' diviso in bande orizzontali dell'uscita, indipendenti (ognuna ricalcola le poche righe di bordo).
 * Il codice NEON (arm64) da' gli stessi bit del C: i test lo verificano con GISP_NO_NEON. */
#include "gisp.h"
#include "gisp_internal.h"

#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#if (defined(__ARM_NEON) || defined(__ARM_NEON__)) && defined(__aarch64__) && !defined(GISP_NO_NEON)
#define GF_NEON 1
#include <arm_neon.h>
#else
#define GF_NEON 0
#endif

#define GF_TAPS 8
#define GF_ONE 16383 /* bianco nel lineare */
#define GF_RING (GF_TAPS + 1)
#define GF_LSC_ROWS 8 /* righe RAW per blocco di guadagni LSC (i guadagni cambiano su ~250 righe) */

typedef struct { int32_t i0; int32_t n; int16_t w[GF_TAPS]; } gf_tap;

typedef struct {
    uint16_t *raw16;          /* riga spacchettata (cw) */
    uint16_t *lin[3];         /* righe lineari con 2 pixel di bordo per lato (cw + 4) */
    int32_t lin_tag[3];
    uint16_t *gain[2];        /* guadagni Q8 per parita' di riga (cw) */
    int32_t gain_tag[2];
    float *gtmp;              /* 4 * lsc_w */
    uint16_t *dm[3];          /* riga demosaicata R, G, B (sw) */
    uint16_t *ring;           /* GF_RING righe x 3 piani x ow */
    int32_t ring_tag[GF_RING];
    uint16_t *vrow;           /* 3 x ow */
    uint8_t *rgb8[2];         /* 3 x ow, due righe */
    uint16_t *idx;            /* 3 x ow indici della LUT */
} gf_scratch;

struct gisp_fast {
    /* geometria */
    uint32_t rw, rh, stride, format, cfa;
    gisp_rect crop;
    uint32_t ow, oh, mode, sw, sh; /* sw/sh: piani demosaicati (crop, o crop/2 in BIN2) */
    gf_tap *tx, *ty;
    int hid, vid;                  /* scala identita' */
    /* parametri */
    uint16_t black[4];
    float k[4];                    /* GF_ONE / (bianco - nero) * WB, per sito */
    int lsc_on;
    gisp_params p;                 /* copia (griglia LSC) */
    uint16_t *gconst[2];           /* guadagni senza LSC, per parita' di riga (cw) */
    int32_t *lsc_xi;               /* per colonna del ritaglio: cella della griglia */
    float *lsc_wx;                 /*   e peso */
    int16_t ccm[9];                /* Q10 */
    uint8_t lut[4096];
    int32_t yc[4], cbc[3], crc[3], coff, ymin, ymax, cmin, cmax; /* Q15 */
    /* colori per sito e ruoli nel demosaic */
    int col[4];
    gf_scratch *scr[GISP_FAST_MAX_BANDS];
};

int gisp_fast_has_neon(void) { return GF_NEON; }
uint32_t gisp_fast_mode_of(const gisp_fast *f) { return f->mode; }

/* ---- pesi della scala ------------------------------------------------------------------------------------------- */
static int make_taps(gf_tap *t, uint32_t src, uint32_t dst, int *ident)
{
    *ident = src == dst;
    double scale = (double)src / dst;
    for (uint32_t o = 0; o < dst; o++) {
        double w[GF_TAPS] = {0};
        int i0, n;
        if (src == dst) { i0 = (int)o; n = 1; w[0] = 1; }
        else if (scale > 1.0) { /* area */
            double a = o * scale, b = a + scale;
            i0 = (int)floor(a);
            int i1 = (int)ceil(b - 1e-9);
            if (i1 > (int)src) i1 = (int)src;
            n = i1 - i0;
            if (n > GF_TAPS) return -1;
            for (int i = i0; i < i1; i++) {
                double lo = a > i ? a : i, hi = b < i + 1 ? b : i + 1;
                w[i - i0] = (hi - lo) / scale;
            }
        } else { /* bilineare */
            double s = (o + 0.5) * scale - 0.5;
            if (s < 0) s = 0;
            i0 = (int)floor(s);
            if (i0 > (int)src - 1) i0 = (int)src - 1;
            double fr = s - i0;
            if (i0 + 1 < (int)src && fr > 0) { n = 2; w[0] = 1 - fr; w[1] = fr; }
            else { n = 1; w[0] = 1; }
        }
        /* pesi Q14 con somma esatta 16384 (il resto va al peso maggiore) */
        int sum = 0, big = 0;
        for (int i = 0; i < n; i++) {
            t[o].w[i] = (int16_t)lrint(w[i] * 16384);
            sum += t[o].w[i];
            if (t[o].w[i] > t[o].w[big]) big = i;
        }
        t[o].w[big] = (int16_t)(t[o].w[big] + 16384 - sum);
        t[o].i0 = i0; t[o].n = n;
    }
    return 0;
}

/* ---- parametri -------------------------------------------------------------------------------------------------- */
static int16_t q10(float v)
{
    float q = v * 1024.0f;
    return (int16_t)(q > 32767 ? 32767 : q < -32768 ? -32768 : lrintf(q));
}

void gisp_fast_set_params(gisp_fast *f, const gisp_params *p)
{
    const uint32_t cw = f->crop.w;
    memcpy(&f->p, p, sizeof *p);
    for (int s = 0; s < 4; s++) {
        float b = p->black[s] < 0 ? 0 : p->black[s] > 1023 ? 1023 : p->black[s];
        f->black[s] = (uint16_t)lrintf(b);
        float range = p->white - f->black[s];
        f->k[s] = (range > 1 ? GF_ONE / range : GF_ONE) * p->wb[gisp_cfa_color(f->cfa, s)];
    }
    f->lsc_on = p->lsc_w >= 1 && p->lsc_h >= 1 && p->lsc_w * p->lsc_h <= GISP_LSC_MAX;
    for (int py = 0; py < 2; py++)
        for (uint32_t i = 0; i < cw; i++) {
            float g = f->k[py * 2 + (i & 1)] * 256.0f + 0.5f;
            f->gconst[py][i] = (uint16_t)(g > 65535 ? 65535 : g);
        }
    if (f->lsc_on) {
        float fw, ox, sc;
        if (p->lsc_field_w && p->lsc_field_h) {
            fw = (float)p->lsc_field_w; ox = p->lsc_off_x; sc = p->lsc_scale > 0 ? p->lsc_scale : 1;
        } else {
            fw = (float)f->rw; ox = 0; sc = 1;
        }
        const uint32_t gw = p->lsc_w;
        for (uint32_t i = 0; i < cw; i++) {
            float x = ox + (float)(f->crop.x + i) * sc;
            float t = fw > 1 ? x * (gw - 1) / (fw - 1) : 0;
            if (t < 0) t = 0;
            if (t > (float)(gw - 1)) t = (float)(gw - 1);
            int32_t xi = (int32_t)t;
            if (gw < 2) { xi = 0; t = 0; }
            else if (xi > (int32_t)gw - 2) xi = (int32_t)gw - 2;
            f->lsc_xi[i] = xi;
            f->lsc_wx[i] = t - (float)xi;
        }
    }
    for (int k = 0; k < 9; k++) f->ccm[k] = q10(p->ccm[k]);
    for (int i = 0; i < 4096; i++) {
        float x = (i * 4 + 2) / (float)GF_ONE * 256.0f;
        if (x > 256) x = 256;
        int k = (int)x; if (k > 255) k = 255;
        float g = p->gamma[k] + (p->gamma[k + 1] - p->gamma[k]) * (x - k);
        int v = (int)lrintf(g * 255.0f);
        f->lut[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
    }
    double kr = p->matrix == GISP_BT601 ? 0.299 : 0.2126, kb = p->matrix == GISP_BT601 ? 0.114 : 0.0722;
    double kg = 1 - kr - kb;
    double ys = p->full_range ? 1.0 : 219.0 / 255.0, cs = p->full_range ? 1.0 : 224.0 / 255.0;
    double yo = p->full_range ? 0 : 16;
    f->yc[0] = (int32_t)lrint(kr * ys * 32768); f->yc[1] = (int32_t)lrint(kg * ys * 32768);
    f->yc[2] = (int32_t)lrint(kb * ys * 32768); f->yc[3] = (int32_t)lrint((yo + 0.5) * 32768);
    /* croma sulle somme di 4 pixel: C = 128 + cs * (B - Y) / (2 (1 - kb)) */
    double sb = cs / (2 * (1 - kb)), sr = cs / (2 * (1 - kr));
    f->cbc[0] = (int32_t)lrint(-kr * sb * 8192); f->cbc[1] = (int32_t)lrint(-kg * sb * 8192);
    f->cbc[2] = (int32_t)lrint((1 - kb) * sb * 8192);
    f->crc[0] = (int32_t)lrint((1 - kr) * sr * 8192); f->crc[1] = (int32_t)lrint(-kg * sr * 8192);
    f->crc[2] = (int32_t)lrint(-kb * sr * 8192);
    f->coff = (int32_t)lrint(128.5 * 32768);
    f->ymin = p->full_range ? 0 : 16; f->ymax = p->full_range ? 255 : 235;
    f->cmin = p->full_range ? 0 : 16; f->cmax = p->full_range ? 255 : 240;
    /* i guadagni LSC in cache nelle bande non valgono piu' */
    for (int b = 0; b < GISP_FAST_MAX_BANDS; b++)
        if (f->scr[b]) f->scr[b]->gain_tag[0] = f->scr[b]->gain_tag[1] = -1;
}

gisp_fast *gisp_fast_new(const gisp_raw *g, const gisp_rect *crop_in, uint32_t ow, uint32_t oh, uint32_t mode,
                         const gisp_params *p)
{
    gisp_rect crop;
    if (!g || !p || gisp__crop(g, crop_in, &crop) || !ow || !oh || (ow | oh) & 1 || mode > GISP_FAST_BIN2) return NULL;
    if (g->format == GISP_RAW10_MIPI ? g->stride < g->width * 5 / 4 : g->stride < g->width * 2) return NULL;
    if (g->cfa == GISP_MONO && mode == GISP_FAST_BIN2) return NULL;
    gisp_fast *f = calloc(1, sizeof *f);
    if (!f) return NULL;
    f->rw = g->width; f->rh = g->height; f->stride = g->stride; f->format = g->format; f->cfa = g->cfa;
    f->crop = crop; f->ow = ow; f->oh = oh;
    if (mode == GISP_FAST_AUTO)
        mode = (crop.w >= 2 * ow && crop.h >= 2 * oh && g->cfa != GISP_MONO) ? GISP_FAST_BIN2 : GISP_FAST_FULL;
    f->mode = mode;
    f->sw = mode == GISP_FAST_BIN2 ? crop.w / 2 : crop.w;
    f->sh = mode == GISP_FAST_BIN2 ? crop.h / 2 : crop.h;
    for (int s = 0; s < 4; s++) f->col[s] = gisp_cfa_color(g->cfa, s);
    f->tx = malloc(sizeof(gf_tap) * ow);
    f->ty = malloc(sizeof(gf_tap) * oh);
    f->gconst[0] = malloc(2 * crop.w);
    f->gconst[1] = malloc(2 * crop.w);
    f->lsc_xi = malloc(sizeof(int32_t) * crop.w);
    f->lsc_wx = malloc(sizeof(float) * crop.w);
    if (!f->tx || !f->ty || !f->gconst[0] || !f->gconst[1] || !f->lsc_xi || !f->lsc_wx ||
        make_taps(f->tx, f->sw, ow, &f->hid) || make_taps(f->ty, f->sh, oh, &f->vid)) {
        gisp_fast_free(f);
        return NULL;
    }
    gisp_fast_set_params(f, p);
    return f;
}

static void scratch_free(gf_scratch *s)
{
    if (!s) return;
    free(s->raw16);
    for (int i = 0; i < 3; i++) free(s->lin[i]), free(s->dm[i]);
    free(s->gain[0]); free(s->gain[1]); free(s->gtmp);
    free(s->ring); free(s->vrow); free(s->rgb8[0]); free(s->rgb8[1]); free(s->idx);
    free(s);
}

void gisp_fast_free(gisp_fast *f)
{
    if (!f) return;
    for (int b = 0; b < GISP_FAST_MAX_BANDS; b++) scratch_free(f->scr[b]);
    free(f->tx); free(f->ty); free(f->gconst[0]); free(f->gconst[1]); free(f->lsc_xi); free(f->lsc_wx);
    free(f);
}

static gf_scratch *scratch_get(gisp_fast *f, uint32_t band)
{
    if (f->scr[band]) return f->scr[band];
    gf_scratch *s = calloc(1, sizeof *s);
    if (!s) return NULL;
    const size_t cw = f->crop.w, sw = f->sw, ow = f->ow;
    int ok = (s->raw16 = malloc(2 * (cw + 16))) != NULL;
    for (int i = 0; i < 3; i++) {
        ok &= (s->lin[i] = malloc(2 * (cw + 4 + 32))) != NULL;
        ok &= (s->dm[i] = malloc(2 * (sw + 16))) != NULL;
    }
    ok &= (s->gain[0] = malloc(2 * (cw + 16))) != NULL;
    ok &= (s->gain[1] = malloc(2 * (cw + 16))) != NULL;
    ok &= (s->gtmp = malloc(sizeof(float) * 4 * GISP_LSC_MAX)) != NULL;
    ok &= (s->ring = malloc(2 * GF_RING * 3 * ow)) != NULL;
    ok &= (s->vrow = malloc(2 * 3 * ow)) != NULL;
    ok &= (s->rgb8[0] = malloc(3 * ow)) != NULL;
    ok &= (s->rgb8[1] = malloc(3 * ow)) != NULL;
    ok &= (s->idx = malloc(2 * 3 * (ow + 8))) != NULL;
    if (!ok) { scratch_free(s); return NULL; }
    s->gain_tag[0] = s->gain_tag[1] = -1;
    f->scr[band] = s;
    return s;
}

/* ---- 1. spacchettamento --------------------------------------------------------------------------------------- */
static void unpack(const gisp_fast *f, const gisp_raw *raw, uint32_t y, uint16_t *out)
{
    const uint32_t x0 = f->crop.x, n = f->crop.w;
    const uint8_t *row = raw->data + (size_t)y * f->stride;
    if (f->format == GISP_RAW16) {
        memcpy(out, row + 2 * (size_t)x0, 2 * (size_t)n);
        return;
    }
    uint32_t i = 0;
    for (; i < n && ((x0 + i) & 3); i++) { /* fino a un gruppo di 4 */
        uint32_t x = x0 + i;
        const uint8_t *g = row + (x >> 2) * 5;
        out[i] = (uint16_t)(g[x & 3] << 2 | ((g[4] >> (2 * (x & 3))) & 3));
    }
#if GF_NEON
    /* byte leggibili dalla riga in poi (l'ultima riga del buffer non ha stride pieno garantito) */
    const size_t avail = (size_t)(f->rh - 1 - y) * f->stride + (size_t)f->rw * 5 / 4;
    static const uint8_t ihi[16] = {0, 1, 2, 3, 5, 6, 7, 8, 10, 11, 12, 13, 15, 16, 17, 18};
    static const uint8_t ilo[16] = {4, 4, 4, 4, 9, 9, 9, 9, 14, 14, 14, 14, 19, 19, 19, 19};
    static const int8_t sh[16] = {0, -2, -4, -6, 0, -2, -4, -6, 0, -2, -4, -6, 0, -2, -4, -6};
    const uint8x16_t vihi = vld1q_u8(ihi), vilo = vld1q_u8(ilo), three = vdupq_n_u8(3);
    const int8x16_t vsh = vld1q_s8(sh);
    for (; i + 16 <= n; i += 16) {
        size_t off = (size_t)((x0 + i) >> 2) * 5;
        if (off + 32 > avail) break;
        uint8x16x2_t q = {{vld1q_u8(row + off), vld1q_u8(row + off + 16)}};
        uint8x16_t hi = vqtbl2q_u8(q, vihi);
        uint8x16_t lo = vandq_u8(vshlq_u8(vqtbl2q_u8(q, vilo), vsh), three);
        uint16x8_t a = vorrq_u16(vshll_n_u8(vget_low_u8(hi), 2), vmovl_u8(vget_low_u8(lo)));
        uint16x8_t b = vorrq_u16(vshll_n_u8(vget_high_u8(hi), 2), vmovl_u8(vget_high_u8(lo)));
        vst1q_u16(out + i, a);
        vst1q_u16(out + i + 8, b);
    }
#endif
    for (; i < n; i++) {
        uint32_t x = x0 + i;
        const uint8_t *g = row + (x >> 2) * 5;
        out[i] = (uint16_t)(g[x & 3] << 2 | ((g[4] >> (2 * (x & 3))) & 3));
    }
}

/* ---- 2. guadagni per riga (nero/WB/LSC) ------------------------------------------------------------------------- */
static const uint16_t *row_gains(gisp_fast *f, gf_scratch *s, uint32_t y)
{
    const int py = y & 1;
    if (!f->lsc_on) return f->gconst[py];
    /* blocchi di GF_LSC_ROWS righe: guadagni calcolati al centro del blocco, deterministici rispetto alle bande */
    int32_t tag = (int32_t)(y / GF_LSC_ROWS);
    if (s->gain_tag[py] == tag) return s->gain[py];
    const gisp_params *p = &f->p;
    const uint32_t gw = p->lsc_w, gh = p->lsc_h, cw = f->crop.w;
    float fh, oy, sc;
    if (p->lsc_field_w && p->lsc_field_h) {
        fh = (float)p->lsc_field_h; oy = p->lsc_off_y; sc = p->lsc_scale > 0 ? p->lsc_scale : 1;
    } else {
        fh = (float)f->rh; oy = 0; sc = 1;
    }
    float yc = oy + ((float)(tag * GF_LSC_ROWS) + (GF_LSC_ROWS - 1) * 0.5f) * sc;
    float t = fh > 1 ? yc * (gh - 1) / (fh - 1) : 0;
    if (t < 0) t = 0;
    if (t > (float)(gh - 1)) t = (float)(gh - 1);
    uint32_t y0 = (uint32_t)t;
    if (gh < 2) { y0 = 0; t = 0; }
    else if (y0 > gh - 2) y0 = gh - 2;
    float wy = t - (float)y0;
    float *tmp = s->gtmp;
    int ch[2];
    for (int xs = 0; xs < 2; xs++) ch[xs] = gisp__lsc_channel(f->cfa, (uint32_t)(py * 2 + xs));
    for (int xs = 0; xs < 2; xs++) {
        const float *a = p->lsc[ch[xs]] + y0 * gw, *b = gh > 1 ? a + gw : a;
        for (uint32_t gx = 0; gx < gw; gx++) tmp[xs * gw + gx] = a[gx] + (b[gx] - a[gx]) * wy;
    }
    const float k0 = f->k[py * 2] * 256.0f, k1 = f->k[py * 2 + 1] * 256.0f;
    uint16_t *g = s->gain[py];
    for (uint32_t i = 0; i < cw; i++) {
        const float *r = tmp + (i & 1) * gw;
        int32_t xi = f->lsc_xi[i];
        float v = r[xi] + ((gw > 1 ? r[xi + 1] : r[xi]) - r[xi]) * f->lsc_wx[i];
        v = v * ((i & 1) ? k1 : k0) + 0.5f;
        g[i] = (uint16_t)(v > 65535 ? 65535 : v < 0 ? 0 : v);
    }
    s->gain_tag[py] = tag;
    return g;
}

/* ---- 3. linearizzazione: min(GF_ONE, ((v - nero)+ * guadagno) >> 8) --------------------------------------------- */
static void linearize(gisp_fast *f, gf_scratch *s, const gisp_raw *raw, uint32_t y, uint16_t *lin)
{
    const uint32_t n = f->crop.w;
    unpack(f, raw, y, s->raw16);
    const uint16_t *g = row_gains(f, s, y);
    const uint16_t *v = s->raw16;
    const uint16_t b0 = f->black[(y & 1) * 2], b1 = f->black[(y & 1) * 2 + 1];
    uint32_t i = 0;
#if GF_NEON
    const uint16_t bb[8] = {b0, b1, b0, b1, b0, b1, b0, b1};
    const uint16x8_t vb = vld1q_u16(bb), one = vdupq_n_u16(GF_ONE);
    for (; i + 8 <= n; i += 8) {
        uint16x8_t d = vqsubq_u16(vld1q_u16(v + i), vb);
        uint16x8_t gg = vld1q_u16(g + i);
        uint32x4_t lo = vmull_u16(vget_low_u16(d), vget_low_u16(gg));
        uint32x4_t hi = vmull_u16(vget_high_u16(d), vget_high_u16(gg));
        uint16x8_t r = vcombine_u16(vqshrn_n_u32(lo, 8), vqshrn_n_u32(hi, 8));
        vst1q_u16(lin + i, vminq_u16(r, one));
    }
#endif
    for (; i < n; i++) {
        uint32_t b = (i & 1) ? b1 : b0;
        uint32_t d = v[i] > b ? v[i] - b : 0;
        uint32_t r = (d * g[i]) >> 8;
        lin[i] = (uint16_t)(r > GF_ONE ? GF_ONE : r);
    }
}

/* riga lineare del ritaglio r (riflessa ai bordi), con 2 pixel di bordo per lato; restituisce il puntatore al
 * pixel 0 */
static const uint16_t *lin_row(gisp_fast *f, gf_scratch *s, const gisp_raw *raw, int32_t r)
{
    const int32_t h = (int32_t)f->crop.h, w = (int32_t)f->crop.w;
    if (r < 0) r = -r;
    if (r > h - 1) r = 2 * (h - 1) - r;
    for (int k = 0; k < 3; k++)
        if (s->lin_tag[k] == r) return s->lin[k] + 2;
    /* rimpiazza lo slot con la riga piu' lontana */
    int k = 0;
    int32_t far = -1;
    for (int j = 0; j < 3; j++) {
        int32_t d = s->lin_tag[j] < 0 ? 1 << 30 : abs(s->lin_tag[j] - r);
        if (d > far) { far = d; k = j; }
    }
    uint16_t *l = s->lin[k] + 2;
    linearize(f, s, raw, f->crop.y + (uint32_t)r, l);
    l[-1] = l[1]; l[-2] = l[2]; l[w] = l[w - 2]; l[w + 1] = l[w - 3];
    s->lin_tag[k] = r;
    return l;
}

/* ---- 4. demosaic bilineare di una riga ------------------------------------------------------------------------- */
#define AVG2(a, b) ((uint16_t)(((uint32_t)(a) + (b) + 1) >> 1))
#define AVG4(a, b, c, d) ((uint16_t)(((uint32_t)(a) + (b) + (c) + (d) + 2) >> 2))

/* gfirst: il verde sta nelle colonne pari di questa riga. X = l'altro colore della riga, Y = il colore delle righe
 * sopra/sotto. */
static void demosaic_row(const uint16_t *u, const uint16_t *c, const uint16_t *d, uint32_t w_, int gfirst,
                         uint16_t *X, uint16_t *G, uint16_t *Y)
{
    const ptrdiff_t w = (ptrdiff_t)w_; /* con segno: si legge a x - 1 */
    ptrdiff_t x = 0;
#if GF_NEON
    for (; x + 16 <= w; x += 16) {
        uint16x8x2_t cc = vld2q_u16(c + x), cp = vld2q_u16(c + x - 2), cn = vld2q_u16(c + x + 2);
        uint16x8x2_t uu = vld2q_u16(u + x), up = vld2q_u16(u + x - 2), un = vld2q_u16(u + x + 2);
        uint16x8x2_t dd = vld2q_u16(d + x), dp = vld2q_u16(d + x - 2), dn = vld2q_u16(d + x + 2);
        uint16x8x2_t oX, oG, oY;
        if (gfirst) {
            /* pari: G; dispari: X */
            oG.val[0] = cc.val[0];
            oX.val[0] = vrhaddq_u16(cp.val[1], cc.val[1]);
            oY.val[0] = vrhaddq_u16(uu.val[0], dd.val[0]);
            oX.val[1] = cc.val[1];
            oG.val[1] = vrshrq_n_u16(vaddq_u16(vaddq_u16(cc.val[0], cn.val[0]), vaddq_u16(uu.val[1], dd.val[1])), 2);
            oY.val[1] = vrshrq_n_u16(vaddq_u16(vaddq_u16(uu.val[0], un.val[0]), vaddq_u16(dd.val[0], dn.val[0])), 2);
        } else {
            /* pari: X; dispari: G */
            oX.val[0] = cc.val[0];
            oG.val[0] = vrshrq_n_u16(vaddq_u16(vaddq_u16(cp.val[1], cc.val[1]), vaddq_u16(uu.val[0], dd.val[0])), 2);
            oY.val[0] = vrshrq_n_u16(vaddq_u16(vaddq_u16(up.val[1], uu.val[1]), vaddq_u16(dp.val[1], dd.val[1])), 2);
            oG.val[1] = cc.val[1];
            oX.val[1] = vrhaddq_u16(cc.val[0], cn.val[0]);
            oY.val[1] = vrhaddq_u16(uu.val[1], dd.val[1]);
        }
        vst2q_u16(X + x, oX);
        vst2q_u16(G + x, oG);
        vst2q_u16(Y + x, oY);
    }
#endif
    for (; x < w; x += 2) {
        if (gfirst) {
            G[x] = c[x];
            X[x] = AVG2(c[x - 1], c[x + 1]);
            Y[x] = AVG2(u[x], d[x]);
            X[x + 1] = c[x + 1];
            G[x + 1] = AVG4(c[x], c[x + 2], u[x + 1], d[x + 1]);
            Y[x + 1] = AVG4(u[x], u[x + 2], d[x], d[x + 2]);
        } else {
            X[x] = c[x];
            G[x] = AVG4(c[x - 1], c[x + 1], u[x], d[x]);
            Y[x] = AVG4(u[x - 1], u[x + 1], d[x - 1], d[x + 1]);
            G[x + 1] = c[x + 1];
            X[x + 1] = AVG2(c[x], c[x + 2]);
            Y[x + 1] = AVG2(u[x + 1], d[x + 1]);
        }
    }
}

/* superpixel 2x2: righe a (sopra) e b (sotto) del quadrato */
static void bin2_row(const gisp_fast *f, const uint16_t *a, const uint16_t *b, uint32_t qw, uint16_t *R, uint16_t *G,
                     uint16_t *B)
{
    const uint16_t *site[4] = {a, a + 1, b, b + 1};
    const uint16_t *pr = NULL, *pb = NULL, *pg0 = NULL, *pg1 = NULL;
    for (int s = 0; s < 4; s++) {
        if (f->col[s] == 0) pr = site[s];
        else if (f->col[s] == 2) pb = site[s];
        else if (!pg0) pg0 = site[s];
        else pg1 = site[s];
    }
    uint32_t q = 0;
#if GF_NEON
    /* i siti di una riga sono pari/dispari: carichiamo le due righe deinterlacciate */
    for (; q + 8 <= qw; q += 8) {
        uint16x8x2_t va = vld2q_u16(a + 2 * q), vb = vld2q_u16(b + 2 * q);
        uint16x8_t v[4] = {va.val[0], va.val[1], vb.val[0], vb.val[1]};
        uint16x8_t r = v[0], bl = v[0], g0 = v[0], g1 = v[0];
        int ng = 0;
        for (int s = 0; s < 4; s++) {
            if (f->col[s] == 0) r = v[s];
            else if (f->col[s] == 2) bl = v[s];
            else if (!ng++) g0 = v[s];
            else g1 = v[s];
        }
        vst1q_u16(R + q, r);
        vst1q_u16(B + q, bl);
        vst1q_u16(G + q, vrhaddq_u16(g0, g1));
    }
#endif
    for (; q < qw; q++) {
        R[q] = pr[2 * q];
        B[q] = pb[2 * q];
        G[q] = AVG2(pg0[2 * q], pg1[2 * q]);
    }
}

/* ---- 5. scala orizzontale in lineare ---------------------------------------------------------------------------- */
static void hscale(const gisp_fast *f, uint16_t *const in[3], uint16_t *out /* 3 piani x ow */)
{
    const uint32_t ow = f->ow;
    uint16_t *o0 = out, *o1 = out + ow, *o2 = out + 2 * ow;
    if (f->hid) {
        memcpy(o0, in[0], 2 * ow); memcpy(o1, in[1], 2 * ow); memcpy(o2, in[2], 2 * ow);
        return;
    }
    const uint16_t *a = in[0], *b = in[1], *c = in[2];
    for (uint32_t x = 0; x < ow; x++) {
        const gf_tap *t = &f->tx[x];
        uint32_t s0 = 8192, s1 = 8192, s2 = 8192;
        for (int k = 0; k < t->n; k++) {
            uint32_t w = (uint32_t)t->w[k];
            int32_t i = t->i0 + k;
            s0 += w * a[i]; s1 += w * b[i]; s2 += w * c[i];
        }
        o0[x] = (uint16_t)(s0 >> 14); o1[x] = (uint16_t)(s1 >> 14); o2[x] = (uint16_t)(s2 >> 14);
    }
}

/* riga sorgente r (nei piani demosaicati, gia' scalata in orizzontale) nell'anello */
static const uint16_t *src_row(gisp_fast *f, gf_scratch *s, const gisp_raw *raw, int32_t r)
{
    int slot = (int)(r % GF_RING);
    uint16_t *dst = s->ring + (size_t)slot * 3 * f->ow;
    if (s->ring_tag[slot] == r) return dst;
    const uint32_t cw = f->crop.w;
    if (f->mode == GISP_FAST_BIN2) {
        const uint16_t *a = lin_row(f, s, raw, 2 * r), *b = lin_row(f, s, raw, 2 * r + 1);
        bin2_row(f, a, b, f->sw, s->dm[0], s->dm[1], s->dm[2]);
    } else {
        const uint16_t *u = lin_row(f, s, raw, r - 1), *c = lin_row(f, s, raw, r), *d = lin_row(f, s, raw, r + 1);
        const uint32_t py = (f->crop.y + (uint32_t)r) & 1; /* crop.x pari: le colonne pari sono il sito (py, 0) */
        if (f->cfa == GISP_MONO) {
            memcpy(s->dm[0], c, 2 * cw); memcpy(s->dm[1], c, 2 * cw); memcpy(s->dm[2], c, 2 * cw);
        } else {
            int c0 = f->col[py * 2], c1 = f->col[py * 2 + 1];
            int gfirst = c0 == 1;
            int xcol = gfirst ? c1 : c0; /* 0 R, 2 B */
            uint16_t *X = s->dm[xcol], *Y = s->dm[2 - xcol];
            demosaic_row(u, c, d, cw, gfirst, X, s->dm[1], Y);
        }
    }
    hscale(f, s->dm, dst);
    s->ring_tag[slot] = r;
    return dst;
}

/* ---- 6. scala verticale, CCM, gamma ----------------------------------------------------------------------------- */
static void out_row_rgb8(gisp_fast *f, gf_scratch *s, const gisp_raw *raw, uint32_t o, uint8_t *rgb)
{
    const uint32_t ow = f->ow, n3 = 3 * ow;
    const gf_tap *t = &f->ty[o];
    const uint16_t *v;
    if (t->n == 1) {
        v = src_row(f, s, raw, t->i0);
    } else {
        const uint16_t *rows[GF_TAPS];
        for (int k = 0; k < t->n; k++) rows[k] = src_row(f, s, raw, t->i0 + k);
        uint16_t *acc = s->vrow;
        for (uint32_t i = 0; i < n3; i++) {
            uint32_t a = 8192;
            for (int k = 0; k < t->n; k++) a += (uint32_t)t->w[k] * rows[k][i];
            acc[i] = (uint16_t)(a >> 14);
        }
        v = acc;
    }
    const uint16_t *R = v, *G = v + ow, *B = v + 2 * ow;
    const int16_t *m = f->ccm;
    uint16_t *ir = s->idx, *ig = ir + ow, *ib = ig + ow;
    uint32_t x = 0;
#if GF_NEON
    {
        const int16x4_t m0 = vld1_s16(m), m1 = vld1_s16(m + 4);
        const int16_t m8 = m[8];
        const uint16x8_t one = vdupq_n_u16(GF_ONE);
        for (; x + 8 <= ow; x += 8) {
            int16x8_t r = vreinterpretq_s16_u16(vld1q_u16(R + x));
            int16x8_t g = vreinterpretq_s16_u16(vld1q_u16(G + x));
            int16x8_t b = vreinterpretq_s16_u16(vld1q_u16(B + x));
            int32x4_t a0 = vmull_lane_s16(vget_low_s16(r), m0, 0);
            int32x4_t a1 = vmull_lane_s16(vget_high_s16(r), m0, 0);
            a0 = vmlal_lane_s16(a0, vget_low_s16(g), m0, 1); a1 = vmlal_lane_s16(a1, vget_high_s16(g), m0, 1);
            a0 = vmlal_lane_s16(a0, vget_low_s16(b), m0, 2); a1 = vmlal_lane_s16(a1, vget_high_s16(b), m0, 2);
            int32x4_t b0 = vmull_lane_s16(vget_low_s16(r), m0, 3);
            int32x4_t b1 = vmull_lane_s16(vget_high_s16(r), m0, 3);
            b0 = vmlal_lane_s16(b0, vget_low_s16(g), m1, 0); b1 = vmlal_lane_s16(b1, vget_high_s16(g), m1, 0);
            b0 = vmlal_lane_s16(b0, vget_low_s16(b), m1, 1); b1 = vmlal_lane_s16(b1, vget_high_s16(b), m1, 1);
            int32x4_t c0 = vmull_lane_s16(vget_low_s16(r), m1, 2);
            int32x4_t c1 = vmull_lane_s16(vget_high_s16(r), m1, 2);
            c0 = vmlal_lane_s16(c0, vget_low_s16(g), m1, 3); c1 = vmlal_lane_s16(c1, vget_high_s16(g), m1, 3);
            c0 = vmlal_n_s16(c0, vget_low_s16(b), m8); c1 = vmlal_n_s16(c1, vget_high_s16(b), m8);
            /* (v + 512) >> 10, saturato a 0..65535, poi al bianco, poi >> 2 = indice della LUT */
            uint16x8_t qr = vminq_u16(vcombine_u16(vqrshrun_n_s32(a0, 10), vqrshrun_n_s32(a1, 10)), one);
            uint16x8_t qg = vminq_u16(vcombine_u16(vqrshrun_n_s32(b0, 10), vqrshrun_n_s32(b1, 10)), one);
            uint16x8_t qb = vminq_u16(vcombine_u16(vqrshrun_n_s32(c0, 10), vqrshrun_n_s32(c1, 10)), one);
            vst1q_u16(ir + x, vshrq_n_u16(qr, 2));
            vst1q_u16(ig + x, vshrq_n_u16(qg, 2));
            vst1q_u16(ib + x, vshrq_n_u16(qb, 2));
        }
    }
#endif
    for (; x < ow; x++) {
        int32_t r = R[x], g = G[x], b = B[x];
        int32_t o3[3];
        for (int k = 0; k < 3; k++) {
            int32_t a = (m[3 * k] * r + m[3 * k + 1] * g + m[3 * k + 2] * b + 512) >> 10;
            o3[k] = a < 0 ? 0 : a > GF_ONE ? GF_ONE : a;
        }
        ir[x] = (uint16_t)(o3[0] >> 2); ig[x] = (uint16_t)(o3[1] >> 2); ib[x] = (uint16_t)(o3[2] >> 2);
    }
    const uint8_t *lut = f->lut;
    uint8_t *r8 = rgb, *g8 = rgb + ow, *b8 = rgb + 2 * ow;
    for (x = 0; x < ow; x++) { r8[x] = lut[ir[x]]; g8[x] = lut[ig[x]]; b8[x] = lut[ib[x]]; }
}

/* ---- 7. NV12 ---------------------------------------------------------------------------------------------------- */
static inline uint8_t clamp8(int32_t v, int32_t lo, int32_t hi) { return (uint8_t)(v < lo ? lo : v > hi ? hi : v); }

static void write_y(const gisp_fast *f, const uint8_t *rgb, uint8_t *dst)
{
    const uint32_t ow = f->ow;
    const uint8_t *r = rgb, *g = rgb + ow, *b = rgb + 2 * ow;
    const int32_t c0 = f->yc[0], c1 = f->yc[1], c2 = f->yc[2], c3 = f->yc[3], lo = f->ymin, hi = f->ymax;
    for (uint32_t x = 0; x < ow; x++) dst[x] = clamp8((c0 * r[x] + c1 * g[x] + c2 * b[x] + c3) >> 15, lo, hi);
}

static void write_uv(const gisp_fast *f, const uint8_t *a, const uint8_t *b, uint8_t *dst)
{
    const uint32_t ow = f->ow;
    const int32_t lo = f->cmin, hi = f->cmax;
    for (uint32_t x = 0; x < ow; x += 2) {
        int32_t sr = a[x] + a[x + 1] + b[x] + b[x + 1];
        int32_t sg = a[ow + x] + a[ow + x + 1] + b[ow + x] + b[ow + x + 1];
        int32_t sb = a[2 * ow + x] + a[2 * ow + x + 1] + b[2 * ow + x] + b[2 * ow + x + 1];
        dst[x] = clamp8((f->cbc[0] * sr + f->cbc[1] * sg + f->cbc[2] * sb + f->coff) >> 15, lo, hi);
        dst[x + 1] = clamp8((f->crc[0] * sr + f->crc[1] * sg + f->crc[2] * sb + f->coff) >> 15, lo, hi);
    }
}

int gisp_fast_band(gisp_fast *f, const gisp_raw *raw, gisp_nv12 *out, uint32_t band, uint32_t nbands)
{
    if (!f || !raw || !raw->data || !out || nbands == 0 || nbands > GISP_FAST_MAX_BANDS || band >= nbands) return -1;
    if (raw->width != f->rw || raw->height != f->rh || raw->stride != f->stride || raw->format != f->format ||
        raw->cfa != f->cfa || out->width != f->ow || out->height != f->oh)
        return -2;
    gf_scratch *s = scratch_get(f, band);
    if (!s) return -3;
    for (int k = 0; k < 3; k++) s->lin_tag[k] = -1000;
    for (int k = 0; k < GF_RING; k++) s->ring_tag[k] = -1;
    const uint32_t pairs = f->oh / 2;
    const uint32_t p0 = (uint32_t)((uint64_t)band * pairs / nbands), p1 = (uint32_t)((uint64_t)(band + 1) * pairs / nbands);
    for (uint32_t pr = p0; pr < p1; pr++) {
        uint32_t o = 2 * pr;
        out_row_rgb8(f, s, raw, o, s->rgb8[0]);
        out_row_rgb8(f, s, raw, o + 1, s->rgb8[1]);
        write_y(f, s->rgb8[0], out->y + (size_t)o * out->y_stride);
        write_y(f, s->rgb8[1], out->y + (size_t)(o + 1) * out->y_stride);
        write_uv(f, s->rgb8[0], s->rgb8[1], out->uv + (size_t)pr * out->uv_stride);
    }
    return 0;
}

typedef struct { gisp_fast *f; const gisp_raw *raw; gisp_nv12 *out; uint32_t band, n; int rc; } gf_job;

static void *gf_thread(void *arg)
{
    gf_job *j = arg;
    j->rc = gisp_fast_band(j->f, j->raw, j->out, j->band, j->n);
    return NULL;
}

int gisp_fast_run(gisp_fast *f, const gisp_raw *raw, gisp_nv12 *out, uint32_t nthreads)
{
    if (nthreads < 1) nthreads = 1;
    if (nthreads > GISP_FAST_MAX_BANDS) nthreads = GISP_FAST_MAX_BANDS;
    if (nthreads == 1) return gisp_fast_band(f, raw, out, 0, 1);
    gf_job job[GISP_FAST_MAX_BANDS];
    pthread_t th[GISP_FAST_MAX_BANDS];
    int started[GISP_FAST_MAX_BANDS] = {0};
    for (uint32_t b = 0; b < nthreads; b++) {
        job[b] = (gf_job){f, raw, out, b, nthreads, 0};
        if (b && pthread_create(&th[b], NULL, gf_thread, &job[b]) == 0) started[b] = 1;
    }
    gf_thread(&job[0]);
    int rc = job[0].rc;
    for (uint32_t b = 1; b < nthreads; b++) {
        if (started[b]) pthread_join(th[b], NULL);
        else gf_thread(&job[b]);
        if (job[b].rc) rc = job[b].rc;
    }
    return rc;
}
