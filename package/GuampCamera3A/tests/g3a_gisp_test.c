/* Host test of the 3A on the real statistics of the software ISP: RAW10 frames of the gisp synthetic scene (ColorChecker,
 * zone plate, ramp; tests/synth.h of package/guamp-isp) rendered at the exposure the 3A asks for, with the colour cast
 * of the synthetic camera and a defocus blur that depends on the lens DAC, then gisp_stats_compute -> g3a_from_gisp ->
 * the AE / AWB / AF loop of gcampipe (package/GuampCamPipe). Checks the units contract of the adapter (white = gisp range, sums before WB) end to end. */
#include "gisp.h"
#include "synth.h"
#include "../g3a_gisp.h"
#include <stdio.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

#define W 320
#define H 240
static uint16_t raw16[W * H];
static uint8_t raw10[W * 5 / 4 * H];
static float scene_lin[H][W][3];   /* scene radiance at the sensor (camera RGB before the WB the ISP undoes) */

/* Camera RGB of the synthetic scene: inverse CCM, then divided by the WB gains the pipeline must find. */
static void build_scene(const synth_cam *c) {
    double inv[9];
    inv3(c->ccm, inv);
    for (uint32_t y = 0; y < H; ++y)
        for (uint32_t x = 0; x < W; ++x) {
            double acc[3] = {0, 0, 0};
            for (int s = 0; s < 4; ++s) {
                double rgb[3];
                synth_scene((x + 0.25 + 0.5 * (s & 1)) / W, (y + 0.25 + 0.5 * (s >> 1)) / H, (double)W / H, rgb);
                for (int k = 0; k < 3; ++k) acc[k] += inv[3 * k] * rgb[0] + inv[3 * k + 1] * rgb[1] + inv[3 * k + 2] * rgb[2];
            }
            for (int k = 0; k < 3; ++k) { const double v = acc[k] / 4 / c->wb[k]; scene_lin[y][x][k] = (float)(v < 0 ? 0 : v); }
        }
}

/* RAW10 frame: exposure factor k (1 = the scene white at full scale), box defocus of radius r pixels, read noise. */
static void shoot(const synth_cam *c, double k, int r, uint64_t *seed) {
    for (uint32_t y = 0; y < H; ++y)
        for (uint32_t x = 0; x < W; ++x) {
            const int site = (int)((y & 1) * 2 + (x & 1)), col = gisp_cfa_color(c->cfa, (uint32_t)site);
            double acc = 0; int n = 0;
            for (int dy = -r; dy <= r; ++dy)
                for (int dx = -r; dx <= r; ++dx) {
                    const int yy = (int)y + dy, xx = (int)x + dx;
                    if (yy < 0 || yy >= H || xx < 0 || xx >= W) continue;
                    acc += scene_lin[yy][xx][col]; ++n;
                }
            double dn = c->black + acc / n * k * (c->white - c->black) + synth_gauss(seed) * 1.0;
            const long q = lrint(dn);
            raw16[(size_t)y * W + x] = (uint16_t)(q < 0 ? 0 : q > 1023 ? 1023 : q);
        }
    synth_pack10(raw16, W, H, raw10, W * 5 / 4);
}

int main(void) {
    synth_cam cam;
    synth_cam_default(&cam);
    cam.vignette = 0;                                  /* no LSC in this test */
    build_scene(&cam);
    gisp_params p;
    gisp_params_default(&p);
    for (int i = 0; i < 4; ++i) p.black[i] = (float)cam.black;
    p.white = (float)cam.white;
    static gisp_stats st;
    const gisp_raw raw = {.width = W, .height = H, .stride = W * 5 / 4, .format = GISP_RAW10_MIPI, .cfa = cam.cfa, .data = raw10};

    /* focus measure of the statistics: sharp vs blurred, at the same exposure */
    {
        uint64_t sd = 5;
        g3a_af_config ac; g3a_af_default(&ac, 100, 700);
        float fv[2];
        for (int b = 0; b < 2; ++b) {
            shoot(&cam, 0.45, b ? 6 : 0, &sd);
            st.zones_x = 16; st.zones_y = 12; st.step = 2;
            gisp_stats_compute(&raw, NULL, &p, &st);
            g3a_stats v; g3a_from_gisp(&v, &st, &p);
            fv[b] = g3a_af_measure(&v, &ac, NULL);
        }
        printf("gisp focus value: sharp %.6f, blurred (6 px) %.6f, ratio %.2f\n", (double)fv[0], (double)fv[1], (double)(fv[0] / fv[1]));
        CHECK(fv[0] > 2.f * fv[1]);
    }
    /* the per-frame loop of gcampipe: AWB, AE on frames that are not pending (ABI v2 flag), AF on every frame */
    g3a_ae_config ae; g3a_ae_default(&ae);
    g3a_ae_result ar; memset(&ar, 0, sizeof(ar));
    g3a_awb_config awc; g3a_awb_default(&awc);
    g3a_awb_state awb; g3a_awb_reset(&awb);
    g3a_af_config afc; g3a_af_default(&afc, 100, 700); afc.continuous = 1;
    static g3a_af_state af;
    g3a_af_start(&af, &afc);
    /* "sensor": exposure applied 2 frames late, light = ns x gain / 40 ms (scene white at full scale at 40 ms, gain 1) */
    uint64_t pend_at = 0, act_ns = 2000000, q_ns = 0;
    float act_g = 1.f, q_g = 0.f;
    uint16_t dac = af.dac;
    const float peak = 430.f;
    uint64_t seed = 99;
    float luma = 0.f;
    for (uint64_t seq = 1; seq <= 100; ++seq) {
        if (pend_at && seq >= pend_at) { act_ns = q_ns; act_g = q_g; pend_at = 0; }
        const int r = (int)(fabsf((float)dac - peak) / 40.f);   /* defocus: 1 px of blur radius per 40 DAC codes */
        shoot(&cam, (double)act_ns * act_g / 40e6, r > 6 ? 6 : r, &seed);
        st.zones_x = 16; st.zones_y = 12; st.step = 2;
        memcpy(p.wb, awb.wb, sizeof(p.wb));
        gisp_stats_compute(&raw, NULL, &p, &st);
        g3a_stats v;
        g3a_from_gisp(&v, &st, &p);
        CHECK(fabsf(v.white - (1023.f - 64.f)) < 0.01f);
        g3a_awb_run(&v, &awc, &awb, NULL);
        if (!pend_at) {
            g3a_ae_run(&v, &ae, awb.wb, act_ns, act_g, &ar);
            if (ar.exposure_ns != act_ns || ar.gain != act_g) { pend_at = seq + 2; q_ns = ar.exposure_ns; q_g = ar.gain; }
        }
        dac = g3a_af_run(&v, &afc, &af);   /* the next frame already sees the new position */
        float hl;
        luma = g3a_ae_measure(&v, &ae, awb.wb, &hl);
    }
    printf("gisp 3A: luma %.3f (target 0.18) at %llu ns x%.2f; wb %.3f %.3f %.3f (camera %.2f 1 %.2f); af state %d dac %u (peak %.0f), %u scans\n",
           (double)luma, (unsigned long long)act_ns, (double)act_g, (double)awb.wb[0], (double)awb.wb[1], (double)awb.wb[2],
           cam.wb[0], cam.wb[2], af.state, dac, (double)peak, af.moves);
    CHECK(fabsf(luma - 0.18f) / 0.18f < 0.12f);
    CHECK(fabsf(awb.wb[0] / (float)cam.wb[0] - 1.f) < 0.08f && fabsf(awb.wb[2] / (float)cam.wb[2] - 1.f) < 0.08f);
    CHECK(af.state == G3A_AF_FOCUSED && fabsf((float)dac - peak) <= 40.f);
    if (fails) { printf("g3a_gisp_test: %d FAILED\n", fails); return 1; }
    printf("g3a_gisp_test: all checks passed\n");
    return 0;
}
