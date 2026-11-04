/* SPDX-License-Identifier: BSD-2-Clause
 * Percorso HQ: linearizzazione (nero, LSC, WB), demosaic (bilineare o Malvar-He-Cutler), CCM, gamma, YCbCr,
 * ridimensionamento, riduzione del rumore, nitidezza, NV12. */
#include "gisp.h"
#include "gisp_internal.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

int gisp_cfa_color(uint32_t cfa, uint32_t site)
{
    static const uint8_t tab[5][4] = {
        {0, 1, 1, 2}, /* RGGB */
        {1, 0, 2, 1}, /* GRBG */
        {1, 2, 0, 1}, /* GBRG */
        {2, 1, 1, 0}, /* BGGR */
        {1, 1, 1, 1}, /* MONO */
    };
    return tab[cfa > 4 ? 0 : cfa][site & 3];
}

/* Canale LSC (0 R, 1 Gr, 2 Gb, 3 B) di un sito. */
int gisp__lsc_channel(uint32_t cfa, uint32_t site)
{
    int c = gisp_cfa_color(cfa, site);
    if (c == 0) return 0;
    if (c == 2) return 3;
    if (cfa == GISP_MONO) return 1;
    /* verde: sulla riga del rosso? */
    uint32_t other = site ^ 1; /* l'altro sito della stessa riga */
    return gisp_cfa_color(cfa, other) == 0 ? 1 : 2;
}

void gisp_gamma_srgb(float gamma[257])
{
    for (int i = 0; i <= 256; i++) {
        double x = i / 256.0;
        gamma[i] = (float)(x <= 0.0031308 ? 12.92 * x : 1.055 * pow(x, 1 / 2.4) - 0.055);
    }
}

void gisp_params_default(gisp_params *p)
{
    memset(p, 0, sizeof *p);
    for (int i = 0; i < 4; i++) p->black[i] = 64;
    p->white = 1023;
    p->wb[0] = p->wb[1] = p->wb[2] = 1;
    p->ccm[0] = p->ccm[4] = p->ccm[8] = 1;
    gisp_gamma_srgb(p->gamma);
    p->demosaic = GISP_DEMOSAIC_MHC;
    p->nr = 0.3f;
    p->sharpen = 0.5f;
    p->sharpen_thr = 1.5f;
    p->matrix = GISP_BT709;
    p->full_range = 0;
}

static void ycc_coeffs(uint32_t matrix, double *kr, double *kb)
{
    if (matrix == GISP_BT601) { *kr = 0.299; *kb = 0.114; }
    else { *kr = 0.2126; *kb = 0.0722; }
}

/* ---- accesso al RAW --------------------------------------------------------------------------------------------- */
void gisp__unpack_row(const gisp_raw *raw, uint32_t y, uint32_t x0, uint32_t n, uint16_t *out)
{
    const uint8_t *row = raw->data + (size_t)y * raw->stride;
    if (raw->format == GISP_RAW16) {
        for (uint32_t i = 0; i < n; i++) {
            const uint8_t *q = row + 2 * (size_t)(x0 + i);
            out[i] = (uint16_t)(q[0] | q[1] << 8);
        }
        return;
    }
    for (uint32_t i = 0; i < n; i++) {
        uint32_t x = x0 + i;
        const uint8_t *g = row + (x >> 2) * 5;
        uint32_t k = x & 3;
        out[i] = (uint16_t)(g[k] << 2 | ((g[4] >> (2 * k)) & 3));
    }
}

/* Guadagni LSC lungo una riga: interpolazione bilineare della griglia sul campo (vedi gisp_params). */
static void lsc_field(const gisp_raw *raw, const gisp_params *p, float *fw, float *fh, float *ox, float *oy, float *sc)
{
    if (p->lsc_field_w && p->lsc_field_h) {
        *fw = (float)p->lsc_field_w; *fh = (float)p->lsc_field_h;
        *ox = p->lsc_off_x; *oy = p->lsc_off_y; *sc = p->lsc_scale > 0 ? p->lsc_scale : 1;
    } else {
        *fw = (float)raw->width; *fh = (float)raw->height; *ox = *oy = 0; *sc = 1;
    }
}

static inline float grid_pos(float f, float field, uint32_t g, uint32_t *i0)
{
    float t = field > 1 ? f * (float)(g - 1) / (field - 1) : 0;
    if (t < 0) t = 0;
    if (t > (float)(g - 1)) t = (float)(g - 1);
    uint32_t i = (uint32_t)t;
    if (g < 2) { *i0 = 0; return 0; }
    if (i > g - 2) i = g - 2;
    *i0 = i;
    return t - (float)i;
}

void gisp__lsc_row(const gisp_raw *raw, const gisp_params *p, uint32_t y, uint32_t x0, uint32_t n, float *gain,
                   float *tmp)
{
    if (!p->lsc_w || !p->lsc_h) {
        for (uint32_t i = 0; i < n; i++) gain[i] = 1;
        return;
    }
    const uint32_t gw = p->lsc_w, gh = p->lsc_h, cfa = raw->cfa;
    float fw, fh, ox, oy, sc;
    lsc_field(raw, p, &fw, &fh, &ox, &oy, &sc);
    uint32_t y0;
    float wy = grid_pos(oy + y * sc, fh, gh, &y0);
    for (int c = 0; c < 4; c++)
        for (uint32_t gx = 0; gx < gw; gx++) {
            float a = p->lsc[c][y0 * gw + gx], b = gh > 1 ? p->lsc[c][(y0 + 1) * gw + gx] : a;
            tmp[c * gw + gx] = a + (b - a) * wy;
        }
    for (uint32_t i = 0; i < n; i++) {
        uint32_t x = x0 + i, xi;
        int c = gisp__lsc_channel(cfa, (y & 1) * 2 + (x & 1));
        float wx = grid_pos(ox + x * sc, fw, gw, &xi);
        const float *r = tmp + c * gw;
        gain[i] = r[xi] + ((gw > 1 ? r[xi + 1] : r[xi]) - r[xi]) * wx;
    }
}

float gisp_lsc_gain(const gisp_raw *raw, const gisp_params *p, uint32_t x, uint32_t y)
{
    float tmp[4 * GISP_LSC_MAX], g;
    if (!p->lsc_w || !p->lsc_h || p->lsc_w * p->lsc_h > GISP_LSC_MAX) return 1;
    gisp__lsc_row(raw, p, y, x, 1, &g, tmp);
    return g;
}

uint32_t gisp_raw_pixel(const gisp_raw *raw, uint32_t x, uint32_t y)
{
    uint16_t v;
    gisp__unpack_row(raw, y, x, 1, &v);
    return v;
}

/* ---- contesto --------------------------------------------------------------------------------------------------- */
struct gisp_ctx {
    void *buf[12];
    size_t cap[12];
};

enum { B_LIN, B_Y, B_CB, B_CR, B_Y2, B_CB2, B_CR2, B_ROW, B_TMP, B_YNR, B_CH, B_CH2 };

gisp_ctx *gisp_ctx_new(void) { return calloc(1, sizeof(gisp_ctx)); }

void gisp_ctx_free(gisp_ctx *ctx)
{
    if (!ctx) return;
    for (int i = 0; i < 12; i++) free(ctx->buf[i]);
    free(ctx);
}

static void *need(gisp_ctx *ctx, int i, size_t size)
{
    if (ctx->cap[i] < size) {
        free(ctx->buf[i]);
        ctx->buf[i] = malloc(size);
        ctx->cap[i] = ctx->buf[i] ? size : 0;
    }
    return ctx->buf[i];
}

/* ---- demosaic --------------------------------------------------------------------------------------------------- */
static inline int reflect(int v, int n)
{
    if (v < 0) v = -v;
    if (v > n - 1) v = 2 * (n - 1) - v;
    return v < 0 ? 0 : v;
}

#define LIN_ONE 16384.0f /* 1.0 nel piano lineare uint16 (fino a 4.0) */

/* un pixel del piano lineare come float (1.0 = bianco) */
#define PX(dx, dy) (lin[(size_t)reflect(y + (dy), h) * w + reflect(x + (dx), w)] * (1.0f / LIN_ONE))

static void demosaic_px(const uint16_t *lin, int w, int h, int x, int y, uint32_t cfa, uint32_t mode, float rgb[3])
{
    int site = (y & 1) * 2 + (x & 1);
    int col = gisp_cfa_color(cfa, site);
    float c = PX(0, 0);
    if (cfa == GISP_MONO) { rgb[0] = rgb[1] = rgb[2] = c; return; }
    /* nella riga del pixel c'e' il rosso? (per i verdi) */
    int row_has_r = gisp_cfa_color(cfa, (site & 2) | 0) == 0 || gisp_cfa_color(cfa, (site & 2) | 1) == 0;
    if (mode == GISP_DEMOSAIC_BILINEAR) {
        float cross = (PX(-1, 0) + PX(1, 0) + PX(0, -1) + PX(0, 1)) * 0.25f;
        float diag = (PX(-1, -1) + PX(1, -1) + PX(-1, 1) + PX(1, 1)) * 0.25f;
        float hor = (PX(-1, 0) + PX(1, 0)) * 0.5f, ver = (PX(0, -1) + PX(0, 1)) * 0.5f;
        if (col == 0) { rgb[0] = c; rgb[1] = cross; rgb[2] = diag; }
        else if (col == 2) { rgb[2] = c; rgb[1] = cross; rgb[0] = diag; }
        else if (row_has_r) { rgb[1] = c; rgb[0] = hor; rgb[2] = ver; }
        else { rgb[1] = c; rgb[0] = ver; rgb[2] = hor; }
        return;
    }
    /* Malvar, He, Cutler 2004: bilineare corretto con il gradiente del canale noto */
    float n1 = PX(0, -1), s1 = PX(0, 1), e1 = PX(1, 0), w1 = PX(-1, 0);
    float n2 = PX(0, -2), s2 = PX(0, 2), e2 = PX(2, 0), w2 = PX(-2, 0);
    float d = PX(-1, -1) + PX(1, -1) + PX(-1, 1) + PX(1, 1);
    if (col == 0 || col == 2) {
        float g = (4 * c + 2 * (n1 + s1 + e1 + w1) - (n2 + s2 + e2 + w2)) * 0.125f;
        float o = (6 * c + 2 * d - 1.5f * (n2 + s2 + e2 + w2)) * 0.125f;
        rgb[1] = g;
        rgb[col] = c;
        rgb[2 - col] = o;
    } else {
        /* colore sui vicini orizzontali */
        float h_ = (5 * c + 4 * (e1 + w1) - (e2 + w2) - d + 0.5f * (n2 + s2)) * 0.125f;
        float v_ = (5 * c + 4 * (n1 + s1) - (n2 + s2) - d + 0.5f * (e2 + w2)) * 0.125f;
        rgb[1] = c;
        if (row_has_r) { rgb[0] = h_; rgb[2] = v_; }
        else { rgb[0] = v_; rgb[2] = h_; }
    }
}
#undef PX

/* ---- ridimensionamento separabile ------------------------------------------------------------------------------- */
typedef struct { int i0, n; float w[64]; } taps;

static int make_taps(taps *t, uint32_t src, uint32_t dst)
{
    double scale = (double)src / dst;
    for (uint32_t o = 0; o < dst; o++) {
        taps *k = &t[o];
        if (scale > 1.0) { /* area */
            double a = o * scale, b = a + scale;
            int i0 = (int)floor(a), i1 = (int)ceil(b);
            if (i1 > (int)src) i1 = src;
            if (i1 - i0 > 64) return -1;
            k->i0 = i0; k->n = i1 - i0;
            double sum = 0;
            for (int i = i0; i < i1; i++) {
                double lo = a > i ? a : i, hi = b < i + 1 ? b : i + 1;
                k->w[i - i0] = (float)(hi - lo); sum += hi - lo;
            }
            for (int i = 0; i < k->n; i++) k->w[i] /= (float)sum;
        } else { /* bilineare */
            double s = (o + 0.5) * scale - 0.5;
            if (s < 0) s = 0;
            int i0 = (int)floor(s);
            if (i0 > (int)src - 1) i0 = src - 1;
            double f = s - i0;
            k->i0 = i0;
            if (i0 + 1 < (int)src && f > 0) { k->n = 2; k->w[0] = (float)(1 - f); k->w[1] = (float)f; }
            else { k->n = 1; k->w[0] = 1; }
        }
    }
    return 0;
}

/* piano int32 sorgente -> piano int32 destinazione */
static int resample(const int32_t *src, uint32_t sw, uint32_t sh, int32_t *dst, uint32_t dw, uint32_t dh, float *tmp,
                    taps *tx, taps *ty)
{
    if (make_taps(tx, sw, dw) || make_taps(ty, sh, dh)) return -1;
    /* orizzontale: sh x dw in tmp */
    for (uint32_t y = 0; y < sh; y++) {
        const int32_t *s = src + (size_t)y * sw;
        float *t = tmp + (size_t)y * dw;
        for (uint32_t x = 0; x < dw; x++) {
            float a = 0;
            for (int i = 0; i < tx[x].n; i++) a += tx[x].w[i] * s[tx[x].i0 + i];
            t[x] = a;
        }
    }
    for (uint32_t y = 0; y < dh; y++) {
        int32_t *d = dst + (size_t)y * dw;
        for (uint32_t x = 0; x < dw; x++) {
            float a = 0;
            for (int i = 0; i < ty[y].n; i++) a += ty[y].w[i] * tmp[(size_t)(ty[y].i0 + i) * dw + x];
            d[x] = (int32_t)lrintf(a);
        }
    }
    return 0;
}

static inline int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : v > hi ? hi : v; }

/* ---- pipeline completa ------------------------------------------------------------------------------------------ */
int gisp_process(gisp_ctx *ctx, const gisp_raw *raw, const gisp_rect *crop_in, const gisp_params *p, gisp_nv12 *out)
{
    if (!ctx || !raw || !p || !out || !raw->data) return -1;
    gisp_rect crop = crop_in && crop_in->w ? *crop_in : (gisp_rect){0, 0, raw->width, raw->height};
    if ((crop.x | crop.y | crop.w | crop.h) & 1 || crop.x + crop.w > raw->width || crop.y + crop.h > raw->height ||
        crop.w < 4 || crop.h < 4 || (out->width | out->height) & 1 || !out->width || !out->height)
        return -2;
    const uint32_t cw = crop.w, ch = crop.h, ow = out->width, oh = out->height;
    const size_t cn = (size_t)cw * ch, on = (size_t)ow * oh;

    uint16_t *lin = need(ctx, B_LIN, cn * 2);
    int32_t *Y = need(ctx, B_Y, cn * 4), *Cb = need(ctx, B_CB, cn * 4), *Cr = need(ctx, B_CR, cn * 4);
    uint32_t gwmax = p->lsc_w > 0 ? p->lsc_w : 1;
    size_t rowsz = (size_t)(raw->width + 4 * gwmax + 16) * sizeof(float) * 2;
    float *row = need(ctx, B_ROW, rowsz);
    if (!lin || !Y || !Cb || !Cr || !row) return -3;
    uint16_t *raw16 = (uint16_t *)row;
    float *gain = row + raw->width, *gtmp = gain + raw->width;

    /* 1. linearizzazione: RAW -> (v - nero) / (bianco - nero) * LSC * WB, uint16 con 1.0 = 16384 */
    float norm[4], wbs[4];
    for (int s = 0; s < 4; s++) {
        norm[s] = 1.0f / (p->white - p->black[s]);
        wbs[s] = p->wb[gisp_cfa_color(raw->cfa, s)];
    }
    for (uint32_t yy = 0; yy < ch; yy++) {
        uint32_t y = crop.y + yy;
        gisp__unpack_row(raw, y, crop.x, cw, raw16);
        gisp__lsc_row(raw, p, y, crop.x, cw, gain, gtmp);
        uint16_t *l = lin + (size_t)yy * cw;
        for (uint32_t i = 0; i < cw; i++) {
            int s = (y & 1) * 2 + ((crop.x + i) & 1);
            float v = (raw16[i] - p->black[s]) * norm[s];
            if (v < 0) v = 0;
            v *= gain[i] * wbs[s];
            float q = v * LIN_ONE + 0.5f;
            l[i] = q >= 65535.0f ? 65535 : (uint16_t)q;
        }
    }

    /* 2. demosaic, CCM, gamma, YCbCr (Y 0..65535, C +-32767 circa) */
    float lut[4097];
    for (int i = 0; i <= 4096; i++) {
        float f = i * (256.0f / 4096.0f);
        int k = (int)f; if (k > 255) k = 255;
        float t = f - k;
        lut[i] = p->gamma[k] + (p->gamma[k + 1] - p->gamma[k]) * t;
    }
    double kr, kb; ycc_coeffs(p->matrix, &kr, &kb);
    const float fkr = (float)kr, fkb = (float)kb, fkg = 1.0f - fkr - fkb;
    const float scb = 1.0f / (2 * (1 - fkb)), scr = 1.0f / (2 * (1 - fkr));
    for (uint32_t y = 0; y < ch; y++)
        for (uint32_t x = 0; x < cw; x++) {
            float rgb[3], o[3];
            /* il piano parte da crop.x/y (pari): la parita' dei siti non cambia */
            demosaic_px(lin, (int)cw, (int)ch, (int)x, (int)y, raw->cfa, p->demosaic, rgb);
            for (int c = 0; c < 3; c++) rgb[c] = rgb[c] < 0 ? 0 : rgb[c] > 1 ? 1 : rgb[c];
            for (int c = 0; c < 3; c++) {
                float v = p->ccm[3 * c] * rgb[0] + p->ccm[3 * c + 1] * rgb[1] + p->ccm[3 * c + 2] * rgb[2];
                v = v < 0 ? 0 : v > 1 ? 1 : v;
                float f = v * 4096.0f;
                int k = (int)f; if (k > 4095) k = 4095;
                o[c] = lut[k] + (lut[k + 1] - lut[k]) * (f - k);
            }
            float yv = fkr * o[0] + fkg * o[1] + fkb * o[2];
            size_t i = (size_t)y * cw + x;
            Y[i] = (int32_t)lrintf(yv * 65535.0f);
            Cb[i] = (int32_t)lrintf((o[2] - yv) * scb * 65535.0f);
            Cr[i] = (int32_t)lrintf((o[0] - yv) * scr * 65535.0f);
        }

    /* 3. ridimensionamento all'uscita (area se riduce, bilineare se ingrandisce) */
    int32_t *Yo = Y, *Cbo = Cb, *Cro = Cr;
    if (ow != cw || oh != ch) {
        Yo = need(ctx, B_Y2, on * 4); Cbo = need(ctx, B_CB2, on * 4); Cro = need(ctx, B_CR2, on * 4);
        float *tmp = need(ctx, B_TMP, (size_t)ch * ow * sizeof(float));
        taps *tx = malloc(sizeof(taps) * ow), *ty = malloc(sizeof(taps) * oh);
        int rc = (!Yo || !Cbo || !Cro || !tmp || !tx || !ty) ? -3 : 0;
        if (!rc) rc = resample(Y, cw, ch, Yo, ow, oh, tmp, tx, ty);
        if (!rc) rc = resample(Cb, cw, ch, Cbo, ow, oh, tmp, tx, ty);
        if (!rc) rc = resample(Cr, cw, ch, Cro, ow, oh, tmp, tx, ty);
        free(tx); free(ty);
        if (rc) return rc == -1 ? -4 : rc;
    }

    /* 4. luma: filtro sigma 3x3 (riduzione del rumore) poi unsharp mask con soglia */
    int32_t *Yn = need(ctx, B_YNR, on * 4);
    if (!Yn) return -3;
    const int32_t thr = (int32_t)(p->nr * 12.0f * 257.0f);
    for (uint32_t y = 0; y < oh; y++)
        for (uint32_t x = 0; x < ow; x++) {
            int32_t c = Yo[(size_t)y * ow + x];
            if (thr <= 0) { Yn[(size_t)y * ow + x] = c; continue; }
            int32_t sum = 0, n = 0;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    int32_t v = Yo[(size_t)reflect((int)y + dy, oh) * ow + reflect((int)x + dx, ow)];
                    if (abs(v - c) <= thr) { sum += v; n++; }
                }
            Yn[(size_t)y * ow + x] = (sum + n / 2) / n;
        }
    const float amt = p->sharpen;
    const float sthr = p->sharpen_thr * 257.0f;
    const float ys = p->full_range ? 255.0f / 65535.0f : 219.0f / 65535.0f, yo = p->full_range ? 0.0f : 16.0f;
    for (uint32_t y = 0; y < oh; y++) {
        uint8_t *dst = out->y + (size_t)y * out->y_stride;
        for (uint32_t x = 0; x < ow; x++) {
            float c = (float)Yn[(size_t)y * ow + x];
            if (amt > 0) {
                float b = 0;
                static const int k3[3] = {1, 2, 1};
                for (int dy = -1; dy <= 1; dy++)
                    for (int dx = -1; dx <= 1; dx++)
                        b += k3[dy + 1] * k3[dx + 1] *
                             (float)Yn[(size_t)reflect((int)y + dy, oh) * ow + reflect((int)x + dx, ow)];
                float d = c - b * (1.0f / 16);
                d = d > sthr ? d - sthr : d < -sthr ? d + sthr : 0;
                c += amt * d;
            }
            dst[x] = (uint8_t)clampi((int32_t)lrintf(yo + c * ys), p->full_range ? 0 : 16, p->full_range ? 255 : 235);
        }
    }

    /* 5. croma: media 2x2, poi filtro binomiale 3x3 mescolato secondo nr */
    const uint32_t hw = ow / 2, hh = oh / 2;
    int32_t *ch1 = need(ctx, B_CH, (size_t)hw * hh * 8);
    if (!ch1) return -3;
    for (uint32_t y = 0; y < hh; y++)
        for (uint32_t x = 0; x < hw; x++) {
            size_t a = (size_t)(2 * y) * ow + 2 * x, b = a + ow;
            ch1[2 * ((size_t)y * hw + x)] = (Cbo[a] + Cbo[a + 1] + Cbo[b] + Cbo[b + 1]) / 4;
            ch1[2 * ((size_t)y * hw + x) + 1] = (Cro[a] + Cro[a + 1] + Cro[b] + Cro[b + 1]) / 4;
        }
    const float cmix = p->nr > 1 ? 1 : p->nr < 0 ? 0 : p->nr;
    const float cs = p->full_range ? 255.0f / 65535.0f : 224.0f / 65535.0f;
    for (uint32_t y = 0; y < hh; y++) {
        uint8_t *dst = out->uv + (size_t)y * out->uv_stride;
        for (uint32_t x = 0; x < hw; x++)
            for (int k = 0; k < 2; k++) {
                float c = (float)ch1[2 * ((size_t)y * hw + x) + k];
                if (cmix > 0) {
                    float b = 0;
                    static const int k3[3] = {1, 2, 1};
                    for (int dy = -1; dy <= 1; dy++)
                        for (int dx = -1; dx <= 1; dx++)
                            b += k3[dy + 1] * k3[dx + 1] *
                                 (float)ch1[2 * ((size_t)reflect((int)y + dy, hh) * hw + reflect((int)x + dx, hw)) + k];
                    c += cmix * (b * (1.0f / 16) - c);
                }
                dst[2 * x + k] = (uint8_t)clampi((int32_t)lrintf(128.0f + c * cs), p->full_range ? 0 : 16,
                                                 p->full_range ? 255 : 240);
            }
    }
    return 0;
}

void gisp_nv12_to_rgb8(const gisp_nv12 *in, uint32_t matrix, uint32_t full_range, uint8_t *rgb, size_t rgb_stride)
{
    double kr, kb; ycc_coeffs(matrix, &kr, &kb);
    double kg = 1 - kr - kb;
    for (uint32_t y = 0; y < in->height; y++)
        for (uint32_t x = 0; x < in->width; x++) {
            double Y = in->y[(size_t)y * in->y_stride + x];
            const uint8_t *c = in->uv + (size_t)(y / 2) * in->uv_stride + 2 * (x / 2);
            double cb = c[0] - 128.0, cr = c[1] - 128.0;
            if (full_range) { Y /= 255; cb /= 255; cr /= 255; }
            else { Y = (Y - 16) / 219; cb /= 224; cr /= 224; }
            double r = Y + 2 * (1 - kr) * cr, b = Y + 2 * (1 - kb) * cb;
            double g = (Y - kr * r - kb * b) / kg;
            double v[3] = {r, g, b};
            for (int k = 0; k < 3; k++)
                rgb[(size_t)y * rgb_stride + 3 * x + k] = (uint8_t)clampi((int32_t)lrint(v[k] * 255), 0, 255);
        }
}
