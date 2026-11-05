/* Host test of g3a.c against a simulated scene: sensor (exposure, gain, clipping, colour sensitivity), voice-coil lens
 * (contrast peak around one DAC), flash LED (light falling off with distance). Statistics are produced in the gisp_stats
 * layout (per-zone sums, saturated quads, sharpness, luma histogram). */
#include "../g3a.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

#include "g3a_sim.h"

static frame F;

/* ---- AE ---- */
static int ae_loop(scene *s, const g3a_ae_config *c, uint64_t *ns, float *gain, int frames, g3a_ae_result *r) {
    int conv_at = -1;
    const float wb[3] = {1.f / 0.55f, 1.f, 1.f / 0.7f};
    for (int k = 0; k < frames; ++k) {
        render(s, (double)*ns * *gain, 437, 0.f, wb, &F);
        g3a_ae_run(&F.v, c, wb, *ns, *gain, r);
        if (r->converged && conv_at < 0) conv_at = k;
        *ns = r->exposure_ns; *gain = r->gain;
    }
    return conv_at;
}

static void test_ae(void) {
    g3a_ae_config c;
    g3a_ae_default(&c);
    scene s;
    g3a_ae_result r;
    /* normal light (value 0.3 at 10 ms), from 1 ms */
    scene_grey(&s, 3e-8f);
    uint64_t ns = 1000000; float gain = 1.f;
    int at = ae_loop(&s, &c, &ns, &gain, 25, &r);
    CHECK(at >= 0 && at < 15);
    CHECK(fabsf(r.luma - r.target) / r.target < 0.1f);
    CHECK(ns % 10000000 == 0 || ns < 10000000);              /* 50 Hz: multiples of 10 ms */
    /* very bright, from a long exposure (everything clipped) */
    scene_grey(&s, 3e-6f);
    ns = 33333333; gain = 8.f;
    at = ae_loop(&s, &c, &ns, &gain, 30, &r);
    printf("ae bright: at %d luma %.3f target %.3f ns %llu gain %.2f\n", at, (double)r.luma, (double)r.target, (unsigned long long)ns, (double)gain);
    CHECK(at >= 0 && fabsf(r.luma - r.target) / r.target < 0.1f && gain == 1.f && ns < 1000000);
    /* brighter than the shortest exposure can handle: limited the other way */
    scene_grey(&s, 3e-4f);
    ae_loop(&s, &c, &ns, &gain, 20, &r);
    CHECK(r.limited == -1 && ns == c.t_min_ns && gain == 1.f);
    /* dark: limited, all time and gain used */
    scene_grey(&s, 1e-11f);
    ns = 10000000; gain = 1.f;
    ae_loop(&s, &c, &ns, &gain, 20, &r);
    CHECK(r.limited == 1 && gain == c.gain_max && ns >= 30000000);
    /* 60 Hz */
    c.flicker_hz = 60;
    scene_grey(&s, 1e-8f);
    ns = 1000000; gain = 1.f;
    ae_loop(&s, &c, &ns, &gain, 25, &r);
    CHECK(ns < 8333333 || llabs((long long)ns - (long long)((ns + 4166666) / 8333333) * 8333333) <= 1);
    /* split respects limits */
    g3a_ae_default(&c);
    g3a_ae_split(&c, 1e3, &ns, &gain);
    CHECK(ns == c.t_min_ns && gain == 1.f);
    g3a_ae_split(&c, 1e12, &ns, &gain);
    CHECK(ns == 30000000 && gain == c.gain_max);
    g3a_ae_split(&c, 25e6 * 1.5, &ns, &gain);
    CHECK(ns == 30000000 && fabsf(gain - 1.25f) < 1e-4f);
    /* highlight protection: a window of very bright zones lowers the target */
    scene_grey(&s, 3e-8f);
    for (int i = 0; i < 40; ++i) { s.refl[i][0] = s.refl[i][1] = s.refl[i][2] = 50.f; }
    ns = 5000000; gain = 1.f;
    ae_loop(&s, &c, &ns, &gain, 30, &r);
    CHECK(r.target < c.target && r.target >= c.target * c.min_target_scale);
}

/* ---- AWB ---- */
static float awb_err(const float g[3], const float want[3]) {
    return fmaxf(fabsf(g[0] / want[0] - 1.f), fabsf(g[2] / want[2] - 1.f));
}
static void test_awb(void) {
    g3a_awb_config c;
    g3a_awb_default(&c);
    scene s;
    scene_grey(&s, 3e-8f);
    s.illum[0] = 1.f; s.illum[1] = 0.72f; s.illum[2] = 0.4f;            /* tungsten-like */
    const float want[3] = {(s.illum[1] * s.sens[1]) / (s.illum[0] * s.sens[0]), 1.f, (s.illum[1] * s.sens[1]) / (s.illum[2] * s.sens[2])};
    g3a_awb_state st;
    g3a_awb_reset(&st);
    float raw[3];
    for (int k = 0; k < 40; ++k) {
        render(&s, 1.2e7, 437, 0.f, st.wb, &F);
        CHECK(g3a_awb_run(&F.v, &c, &st, raw) == 0);
    }
    CHECK(awb_err(st.wb, want) < 0.03f && st.converged && st.zones_used > NZ / 2);
    /* dominant coloured object (40 % of the frame green): weighted grey world must beat plain grey world */
    for (int i = 0; i < NZ * 4 / 10; ++i) { s.refl[i][0] = 0.08f; s.refl[i][1] = 0.45f; s.refl[i][2] = 0.1f; }
    g3a_awb_config plain = c; plain.grey_sigma = 0.f; plain.speed = 1.f;
    g3a_awb_config wgt = c; wgt.speed = 1.f;
    g3a_awb_state a, b;
    g3a_awb_reset(&a); g3a_awb_reset(&b);
    for (int k = 0; k < 5; ++k) {
        render(&s, 1.2e7, 437, 0.f, NULL, &F);
        g3a_awb_run(&F.v, &plain, &a, NULL);
        g3a_awb_run(&F.v, &wgt, &b, NULL);
    }
    const float ea = awb_err(a.wb, want), eb = awb_err(b.wb, want);
    printf("awb dominant green: plain grey world error %.3f, weighted %.3f\n", ea, eb);
    CHECK(eb < ea && eb < 0.1f);
    /* white locus: a point off the locus is pulled to it, cct interpolated */
    g3a_awb_config l = c; l.speed = 1.f; l.grey_sigma = 0.f; l.n_ref = 2;
    l.ref_rg[0] = 0.5f; l.ref_bg[0] = 0.9f; l.ref_cct[0] = 6500;
    l.ref_rg[1] = 0.9f; l.ref_bg[1] = 0.4f; l.ref_cct[1] = 2800;
    scene_grey(&s, 3e-8f);
    s.sens[0] = 0.7f; s.sens[2] = 0.65f;                                 /* white at R/G 0.7, B/G 0.65: middle of the locus */
    g3a_awb_reset(&st);
    render(&s, 1.2e7, 437, 0.f, NULL, &F);
    CHECK(g3a_awb_run(&F.v, &l, &st, NULL) == 0);
    CHECK(st.cct > 3500 && st.cct < 5500);
    /* nothing usable: all black */
    scene_grey(&s, 0.f);
    g3a_awb_reset(&st);
    render(&s, 1e7, 437, 0.f, NULL, &F);
    CHECK(g3a_awb_run(&F.v, &c, &st, NULL) == -1 && st.wb[0] == 1.f);
}

/* ---- AF ---- */
static int af_loop(scene *s, const g3a_af_config *c, g3a_af_state *st, int frames) {
    uint16_t dac = st->dac;
    for (int k = 0; k < frames; ++k) {
        render(s, 1.2e7, dac, 0.f, NULL, &F);
        dac = g3a_af_run(&F.v, c, st);
        if (!c->continuous && (st->state == G3A_AF_FOCUSED || st->state == G3A_AF_FAILED)) return k;
    }
    return frames;
}
static void test_af(void) {
    scene s;
    scene_grey(&s, 3e-8f);
    g3a_af_config c;
    g3a_af_default(&c, 50, 650);
    const float peaks[] = {437.f, 61.f, 640.f, 250.f};
    for (unsigned p = 0; p < 4; ++p) {
        s.lens_peak = peaks[p];
        g3a_af_state st;
        memset(&st, 0, sizeof(st));
        g3a_af_start(&st, &c);
        CHECK(st.dac == 50);
        const int k = af_loop(&s, &c, &st, 80);
        printf("af peak %.0f -> dac %u in %d frames (%u moves)\n", peaks[p], st.dac, k, st.moves);
        CHECK(st.state == G3A_AF_FOCUSED && fabsf((float)st.dac - peaks[p]) <= 8.f && k < 50);
    }
    /* flat scene: failed, hyperfocal */
    for (int i = 0; i < NZ; ++i) s.texture[i] = 0.f;
    g3a_af_state st;
    memset(&st, 0, sizeof(st));
    g3a_af_start(&st, &c);
    af_loop(&s, &c, &st, 80);
    CHECK(st.state == G3A_AF_FAILED && st.dac == 50 + (uint16_t)(0.15f * 600));
    /* continuous: subject moves, AF re-scans and follows */
    scene_grey(&s, 3e-8f);
    c.continuous = 1;
    memset(&st, 0, sizeof(st));
    g3a_af_start(&st, &c);
    af_loop(&s, &c, &st, 60);
    CHECK(st.state == G3A_AF_FOCUSED && fabsf((float)st.dac - 437.f) <= 8.f);
    s.lens_peak = 200.f;
    af_loop(&s, &c, &st, 80);
    CHECK(st.state == G3A_AF_FOCUSED && fabsf((float)st.dac - 200.f) <= 8.f);
    const uint32_t m = st.moves;
    af_loop(&s, &c, &st, 30);                                           /* steady scene: no hunting */
    CHECK(st.moves == m);
}

/* ---- flash ---- */
static void test_flash(void) {
    scene s;
    scene_grey(&s, 2e-11f);                                             /* dark room */
    s.illum[0] = 1.f; s.illum[1] = 0.7f; s.illum[2] = 0.4f;
    s.flash_k = 2e-11f;
    g3a_ae_config ae; g3a_ae_default(&ae);
    g3a_flash_config fc; g3a_flash_default(&fc);
    const float wb_amb[3] = {(0.7f * 1.f) / (1.f * 0.55f), 1.f, (0.7f * 1.f) / (0.4f * 0.7f)};
    uint64_t ns = 33000000; float gain = 16.f;
    g3a_ae_result r;
    ae_loop(&s, &ae, &ns, &gain, 10, &r);
    CHECK(g3a_flash_needed(&fc, &r));
    const double e0 = (double)ns * gain;
    static g3a_flash_ambient_t amb;
    render(&s, e0, 437, 0.f, NULL, &F);
    g3a_flash_ambient(&F.v, e0, &amb);
    const double e1 = g3a_flash_pre_exposure(&fc, &ae, e0);            /* pre-flash frame at a lower exposure */
    CHECK(e1 < e0 && e1 >= (double)ae.t_min_ns);
    render(&s, e1, 437, (float)fc.pre_ma, NULL, &F);
    g3a_flash_plan_t p;
    g3a_flash_plan(&F.v, e1, &amb, &fc, &ae, wb_amb, &p);
    CHECK(p.ok && p.ma >= fc.min_ma && p.ma <= fc.main_ma && p.ms == fc.main_ms);
    CHECK(p.exposure_ns <= fc.t_capture_max_ns && p.gain >= 1.f && p.gain <= ae.gain_max);
    const float fw[3] = {(s.flash_col[1] * s.sens[1]) / (s.flash_col[0] * s.sens[0]), 1.f, (s.flash_col[1] * s.sens[1]) / (s.flash_col[2] * s.sens[2])};
    CHECK(awb_err(p.flash_wb, fw) < 0.05f);
    render(&s, (double)p.exposure_ns * p.gain, 437, (float)p.ma, p.wb, &F);
    float hl;
    const float y = g3a_ae_measure(&F.v, &ae, p.wb, &hl);
    printf("flash: %u mA %llu ns x%.2f, flash part %.2f, expected %.3f measured %.3f\n", p.ma, (unsigned long long)p.exposure_ns,
           p.gain, p.flash_fraction, p.expected_luma, y);
    CHECK(fabsf(y - fc.target) / fc.target < 0.15f && fabsf(p.expected_luma - y) / y < 0.1f);
    CHECK(p.flash_fraction > 0.8f && awb_err(p.wb, fw) < 0.1f);
    /* subject close (pre-flash frame partly clipped) and very close (shortest exposure reached: less current) */
    const float depths[2] = {0.3f, 0.01f};
    for (int t = 0; t < 2; ++t) {
        for (int i = 0; i < NZ; ++i) s.depth[i] = depths[t];
        render(&s, e0, 437, 0.f, NULL, &F);
        g3a_flash_ambient(&F.v, e0, &amb);
        double el = e1;
        uint32_t satq = 0;
        int tries = 0;
        for (; tries < 6; ++tries) {             /* the service repeats the lit frame while the plan asks for it */
            render(&s, el, 437, (float)fc.pre_ma, NULL, &F);
            satq = 0;
            for (int i = 0; i < NZ; ++i) satq += F.sat[i];
            g3a_flash_plan(&F.v, el, &amb, &fc, &ae, wb_amb, &p);
            if (p.ok != -1) break;
            CHECK(p.retry_exposure < el);
            el = p.retry_exposure;
        }
        CHECK(p.ok == 1 && tries < 6);
        if (t == 1) CHECK(tries > 0);
        render(&s, (double)p.exposure_ns * p.gain, 437, (float)p.ma, p.wb, &F);
        const float y2 = g3a_ae_measure(&F.v, &ae, p.wb, &hl);
        printf("flash depth %.3f: %d retries, pre-flash clipped quads %u, %u mA %llu ns x%.2f measured %.3f\n", (double)depths[t], tries, satq, p.ma,
               (unsigned long long)p.exposure_ns, (double)p.gain, (double)y2);
        CHECK(y2 < 1.5f * fc.target && y2 > 0.5f * fc.target);
        if (t == 1) CHECK(p.ma < fc.main_ma && p.exposure_ns == ae.t_min_ns);
    }
    /* subject out of reach: the pre-flash adds nothing */
    for (int i = 0; i < NZ; ++i) s.depth[i] = 100.f;
    render(&s, e0, 437, 0.f, NULL, &F);
    g3a_flash_ambient(&F.v, e0, &amb);
    render(&s, e1, 437, (float)fc.pre_ma, NULL, &F);
    g3a_flash_plan(&F.v, e1, &amb, &fc, &ae, wb_amb, &p);
    CHECK(!p.ok);
}

int main(void) {
    test_ae();
    test_awb();
    test_af();
    test_flash();
    if (fails) { printf("g3a_test: %d FAILED\n", fails); return 1; }
    printf("g3a_test: all checks passed\n");
    return 0;
}
