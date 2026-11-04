/* SPDX-License-Identifier: BSD-2-Clause
 * Scena sintetica e modello inverso del sensore (per i test e per tools/gisp_cli). */
#ifndef GISP_SYNTH_H
#define GISP_SYNTH_H
#include "gisp.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ColorChecker classico, sRGB 8 bit */
static const uint8_t synth_patch[24][3] = {
    {115, 82, 68},   {194, 150, 130}, {98, 122, 157},  {87, 108, 67},   {133, 128, 177}, {103, 189, 170},
    {214, 126, 44},  {80, 91, 166},   {193, 90, 99},   {94, 60, 108},   {157, 188, 64},  {224, 163, 46},
    {56, 61, 150},   {70, 148, 73},   {175, 54, 60},   {231, 199, 31},  {187, 86, 149},  {8, 133, 161},
    {243, 243, 242}, {200, 200, 200}, {160, 160, 160}, {122, 122, 121}, {85, 85, 85},    {52, 52, 52}};

static inline double synth_lin(double s) { return s <= 0.04045 ? s / 12.92 : pow((s + 0.055) / 1.055, 2.4); }
static inline double synth_srgb(double l)
{
    l = l < 0 ? 0 : l > 1 ? 1 : l;
    return l <= 0.0031308 ? 12.92 * l : 1.055 * pow(l, 1 / 2.4) - 0.055;
}

/* Riquadro della toppa i nella scena normalizzata (u, v in 0..1). */
static inline void synth_patch_box(int i, double *u0, double *v0, double *u1, double *v1)
{
    int cx = i % 6, cy = i / 6;
    double pw = 0.60 / 6, ph = 0.52 / 4;
    *u0 = 0.05 + cx * pw + pw * 0.08; *u1 = 0.05 + (cx + 1) * pw - pw * 0.08;
    *v0 = 0.08 + cy * ph + ph * 0.08; *v1 = 0.08 + (cy + 1) * ph - ph * 0.08;
}

/* sRGB lineare della scena nel punto (u, v); aspect = larghezza / altezza in pixel. */
static inline void synth_scene(double u, double v, double aspect, double rgb[3])
{
    double g = synth_lin(118 / 255.0);
    rgb[0] = rgb[1] = rgb[2] = g;
    for (int i = 0; i < 24; i++) {
        double a, b, c, d;
        synth_patch_box(i, &a, &b, &c, &d);
        if (u >= a && u < c && v >= b && v < d) {
            for (int k = 0; k < 3; k++) rgb[k] = synth_lin(synth_patch[i][k] / 255.0);
            return;
        }
    }
    if (u >= 0.70 && u < 0.95 && v >= 0.08 && v < 0.60) { /* zone plate */
        double x = (u - 0.825) * aspect, y = v - 0.34;
        double r2 = x * x + y * y;
        double s = 0.5 + 0.45 * cos(2 * M_PI * 900.0 * r2);
        for (int k = 0; k < 3; k++) rgb[k] = 0.02 + 0.6 * s;
        return;
    }
    if (v >= 0.68 && v < 0.92) { /* rampa lineare */
        double t = (u - 0.05) / 0.90;
        t = t < 0 ? 0 : t > 1 ? 1 : t;
        rgb[0] = rgb[1] = rgb[2] = t * 0.95;
    }
}

typedef struct {
    double wb[3];      /* bilanciamento che la pipeline deve applicare */
    double ccm[9];     /* CCM vera */
    double vignette;   /* guadagno LSC = 1 + vignette * r^2 (r = 1 agli angoli) */
    double black, white;
    double noise;      /* deviazione del rumore di lettura in DN (0 = senza) */
    uint32_t cfa;
} synth_cam;

static inline void synth_cam_default(synth_cam *c)
{
    static const double ccm[9] = {1.88, -0.83, -0.05, -0.28, 1.54, -0.26, -0.04, -0.77, 1.81};
    memcpy(c->ccm, ccm, sizeof ccm);
    c->wb[0] = 1.9; c->wb[1] = 1.0; c->wb[2] = 1.6;
    c->vignette = 1.2;
    c->black = 64; c->white = 1023;
    c->noise = 1.0;
    c->cfa = GISP_GBRG;
}

static inline void inv3(const double m[9], double o[9])
{
    double d = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
    o[0] = (m[4] * m[8] - m[5] * m[7]) / d; o[1] = (m[2] * m[7] - m[1] * m[8]) / d; o[2] = (m[1] * m[5] - m[2] * m[4]) / d;
    o[3] = (m[5] * m[6] - m[3] * m[8]) / d; o[4] = (m[0] * m[8] - m[2] * m[6]) / d; o[5] = (m[2] * m[3] - m[0] * m[5]) / d;
    o[6] = (m[3] * m[7] - m[4] * m[6]) / d; o[7] = (m[1] * m[6] - m[0] * m[7]) / d; o[8] = (m[0] * m[4] - m[1] * m[3]) / d;
}

static inline double synth_vig(const synth_cam *c, double u, double v, int ch)
{
    double x = (u - 0.5) * 2, y = (v - 0.5) * 2;
    double r2 = (x * x + y * y) * 0.5;
    double k = c->vignette * (ch == 0 ? 1.1 : ch == 3 ? 0.9 : 1.0);
    return 1 + k * r2;
}

/* Griglia LSC 17x13 coerente con il modello (canali R, Gr, Gb, B). */
static inline void synth_lsc(const synth_cam *c, gisp_params *p)
{
    p->lsc_w = 17; p->lsc_h = 13;
    for (int ch = 0; ch < 4; ch++)
        for (int j = 0; j < 13; j++)
            for (int i = 0; i < 17; i++) p->lsc[ch][j * 17 + i] = (float)synth_vig(c, i / 16.0, j / 12.0, ch);
}

static inline uint32_t synth_rng(uint64_t *s)
{
    *s = *s * 6364136223846793005ull + 1442695040888963407ull;
    return (uint32_t)(*s >> 33);
}
static inline double synth_gauss(uint64_t *s)
{
    double a = (synth_rng(s) + 1.0) / 2147483649.0, b = synth_rng(s) / 2147483648.0;
    return sqrt(-2 * log(a)) * cos(2 * M_PI * b);
}

/* RAW Bayer a 10 bit (un uint16 per pixel) della scena, con supercampionamento 2x2 per pixel. */
static inline void synth_raw16(const synth_cam *c, uint32_t w, uint32_t h, uint16_t *out)
{
    double inv[9];
    inv3(c->ccm, inv);
    uint64_t seed = 12345;
    const double aspect = (double)w / h;
    for (uint32_t y = 0; y < h; y++)
        for (uint32_t x = 0; x < w; x++) {
            int site = (int)((y & 1) * 2 + (x & 1));
            int col = gisp_cfa_color(c->cfa, (uint32_t)site);
            double acc = 0;
            for (int sy = 0; sy < 2; sy++)
                for (int sx = 0; sx < 2; sx++) {
                    double u = (x + 0.25 + 0.5 * sx) / w, v = (y + 0.25 + 0.5 * sy) / h, rgb[3];
                    synth_scene(u, v, aspect, rgb);
                    double cam = inv[3 * col] * rgb[0] + inv[3 * col + 1] * rgb[1] + inv[3 * col + 2] * rgb[2];
                    acc += cam;
                }
            double u = (x + 0.5) / w, v = (y + 0.5) / h;
            int lch = col == 0 ? 0 : col == 2 ? 3 : (gisp_cfa_color(c->cfa, (uint32_t)(site ^ 1)) == 0 ? 1 : 2);
            double sig = acc / 4 / c->wb[col] / synth_vig(c, u, v, lch);
            if (sig < 0) sig = 0;
            double dn = c->black + sig * (c->white - c->black);
            if (c->noise > 0) dn += synth_gauss(&seed) * sqrt(c->noise * c->noise + 0.05 * sig * (c->white - c->black));
            long q = lrint(dn);
            out[(size_t)y * w + x] = (uint16_t)(q < 0 ? 0 : q > 1023 ? 1023 : q);
        }
}

/* Impacchetta in RAW10 MIPI (stride in byte >= w * 5 / 4). w multiplo di 4. */
static inline void synth_pack10(const uint16_t *in, uint32_t w, uint32_t h, uint8_t *out, uint32_t stride)
{
    for (uint32_t y = 0; y < h; y++) {
        const uint16_t *s = in + (size_t)y * w;
        uint8_t *d = out + (size_t)y * stride;
        for (uint32_t x = 0; x < w; x += 4) {
            uint8_t lo = 0;
            for (int k = 0; k < 4; k++) {
                d[k] = (uint8_t)(s[x + k] >> 2);
                lo |= (uint8_t)((s[x + k] & 3) << (2 * k));
            }
            d[4] = lo;
            d += 5;
        }
    }
}

/* Riferimento sRGB 8 bit della scena all'uscita ow x oh (media su 4x4 campioni per pixel). */
static inline void synth_reference(uint32_t ow, uint32_t oh, double aspect, uint8_t *rgb)
{
    for (uint32_t y = 0; y < oh; y++)
        for (uint32_t x = 0; x < ow; x++) {
            double acc[3] = {0, 0, 0};
            for (int sy = 0; sy < 4; sy++)
                for (int sx = 0; sx < 4; sx++) {
                    double c[3];
                    synth_scene((x + (sx + 0.5) / 4) / ow, (y + (sy + 0.5) / 4) / oh, aspect, c);
                    for (int k = 0; k < 3; k++) acc[k] += c[k];
                }
            for (int k = 0; k < 3; k++) rgb[((size_t)y * ow + x) * 3 + k] = (uint8_t)lrint(synth_srgb(acc[k] / 16) * 255);
        }
}

/* Parametri della pipeline coerenti con la camera sintetica. */
static inline void synth_params(const synth_cam *c, gisp_params *p, int lsc)
{
    gisp_params_default(p);
    for (int i = 0; i < 4; i++) p->black[i] = (float)c->black;
    p->white = (float)c->white;
    for (int i = 0; i < 3; i++) p->wb[i] = (float)c->wb[i];
    for (int i = 0; i < 9; i++) p->ccm[i] = (float)c->ccm[i];
    if (lsc) synth_lsc(c, p);
}
#endif
