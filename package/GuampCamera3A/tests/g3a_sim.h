/* Simulated scene for the g3a host tests (g3a_test.c, g3a_session_test.c): sensor (exposure, gain, clipping, colour
 * sensitivity), voice-coil lens (contrast peak around one DAC), flash LED (light falling off with distance). Statistics
 * in the gisp_stats layout (per-zone sums, saturated quads, sharpness, luma histogram). */
#ifndef G3A_SIM_H
#define G3A_SIM_H
#include "../g3a.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define ZX 16
#define ZY 12
#define NZ (ZX * ZY)
#define WHITE 959.f

typedef struct {
    float refl[NZ][3], depth[NZ], texture[NZ];
    float illum[3], amb;        /* ambient light colour and level (value per unit exposure at reflectance 1) */
    float sens[3];              /* sensor channel sensitivities */
    float flash_col[3], flash_k;/* flash light colour and value per mA per unit exposure at depth 1 */
    float lens_peak, lens_width;
    unsigned seed;
} scene;

typedef struct {
    float sum_r[NZ], sum_g[NZ], sum_b[NZ], sharp[NZ];
    uint32_t count[NZ], sat[NZ], hist[256];
    g3a_stats v;
} frame;

static float rnd(scene *s) { s->seed = s->seed * 1103515245u + 12345u; return (float)((s->seed >> 8) & 0xffff) / 65535.f; }

static void render(scene *s, double exposure, uint16_t dac, float flash_ma, const float wb[3], frame *f) {
    memset(f, 0, sizeof(*f));
    const float one[3] = {1.f, 1.f, 1.f};
    if (!wb) wb = one;
    const float d = ((float)dac - s->lens_peak) / s->lens_width;
    const float contrast = 0.05f + expf(-d * d);
    for (int i = 0; i < NZ; ++i) {
        float v[3];
        for (int c = 0; c < 3; ++c) {
            const float light = s->amb * s->illum[c] + flash_ma * s->flash_k * s->flash_col[c] / (s->depth[i] * s->depth[i]);
            v[c] = s->refl[i][c] * light * s->sens[c] * (float)exposure;
        }
        for (int j = 0; j < 10; ++j) {          /* 10 groups of 10 quads with a spread of brightness inside the zone */
            const float m = 0.6f + 0.08f * (float)j;
            const float r = v[0] * m, g = v[1] * m, b = v[2] * m;
            if (r >= 1.f || g >= 1.f || b >= 1.f) { f->sat[i] += 10; f->hist[255] += 10; continue; }
            f->sum_r[i] += r * WHITE * 10; f->sum_g[i] += g * WHITE * 10; f->sum_b[i] += b * WHITE * 10;
            f->count[i] += 10;
            float y = 0.2126f * wb[0] * r + 0.7152f * g + 0.0722f * wb[2] * b;
            int bin = (int)(y * 256.f);
            f->hist[bin > 255 ? 255 : bin < 0 ? 0 : bin] += 10;
        }
        f->sharp[i] = f->sum_g[i] * s->texture[i] * contrast * (1.f + 0.02f * (rnd(s) - 0.5f));
    }
    f->v.zones_x = ZX; f->v.zones_y = ZY;
    f->v.sum_r = f->sum_r; f->v.sum_g = f->sum_g; f->v.sum_b = f->sum_b;
    f->v.count = f->count; f->v.sat = f->sat; f->v.sharp = f->sharp; f->v.hist = f->hist;
    f->v.white = WHITE;
}

static void scene_grey(scene *s, float amb) {
    memset(s, 0, sizeof(*s));
    s->seed = 7;
    for (int i = 0; i < NZ; ++i) {
        const float g = 0.05f + 0.6f * rnd(s);
        s->refl[i][0] = s->refl[i][1] = s->refl[i][2] = g;
        s->depth[i] = 1.f;
        s->texture[i] = 0.5f + rnd(s);
    }
    s->illum[0] = s->illum[1] = s->illum[2] = 1.f;
    s->sens[0] = 0.55f; s->sens[1] = 1.f; s->sens[2] = 0.7f;
    s->amb = amb;
    s->flash_col[0] = 1.f; s->flash_col[1] = 1.f; s->flash_col[2] = 0.95f;
    s->lens_peak = 437.f; s->lens_width = 60.f;
}

#endif
