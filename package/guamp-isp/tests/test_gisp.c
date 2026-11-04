/* SPDX-License-Identifier: BSD-2-Clause
 * Test di guamp-isp su scene sintetiche.
 *   test_gisp                 test funzionali (esce con 1 se qualcosa fallisce)
 *   test_gisp bench [iter]    tempi del percorso fast e delle statistiche
 *   test_gisp dump DIR        scrive le uscite NV12 dei casi fissi (per confrontare NEON e C)
 *   test_gisp ppm DIR         scrive PPM di controllo (HQ, FULL, BIN2, riferimento) */
#include "gisp.h"
#include "synth.h"

#include <stdio.h>
#include <time.h>

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FALLITO: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

typedef struct { uint32_t w, h; uint16_t *r16; uint8_t *r10; uint32_t stride10; } scene;

static scene make_scene(const synth_cam *c, uint32_t w, uint32_t h)
{
    scene s = {w, h, malloc(sizeof(uint16_t) * w * h), NULL, w * 5 / 4};
    synth_raw16(c, w, h, s.r16);
    s.r10 = malloc((size_t)s.stride10 * h);
    synth_pack10(s.r16, w, h, s.r10, s.stride10);
    return s;
}

static gisp_raw raw10(const scene *s, uint32_t cfa)
{
    return (gisp_raw){s->w, s->h, s->stride10, GISP_RAW10_MIPI, cfa, s->r10};
}

typedef struct { uint32_t w, h; uint8_t *buf; gisp_nv12 nv; } frame;

static frame frame_new(uint32_t w, uint32_t h)
{
    frame f = {w, h, calloc((size_t)w * h * 3 / 2 + 64, 1), {0}};
    f.nv = (gisp_nv12){w, h, f.buf, w, f.buf + (size_t)w * h, w};
    return f;
}

static double psnr(const uint8_t *a, const uint8_t *b, uint32_t w, uint32_t h, uint32_t border)
{
    double se = 0;
    size_t n = 0;
    for (uint32_t y = border; y < h - border; y++)
        for (uint32_t x = border; x < w - border; x++)
            for (int k = 0; k < 3; k++) {
                double d = (double)a[((size_t)y * w + x) * 3 + k] - b[((size_t)y * w + x) * 3 + k];
                se += d * d; n++;
            }
    return se ? 10 * log10(255.0 * 255.0 / (se / n)) : 99;
}

/* errore medio sulle 24 toppe (centro 50%), in livelli sRGB a 8 bit; maxe = peggiore */
static double patch_error(const uint8_t *rgb, uint32_t w, uint32_t h, double *maxe)
{
    double tot = 0;
    *maxe = 0;
    for (int i = 0; i < 24; i++) {
        double u0, v0, u1, v1, acc[3] = {0, 0, 0};
        synth_patch_box(i, &u0, &v0, &u1, &v1);
        double du = (u1 - u0) / 4, dv = (v1 - v0) / 4;
        uint32_t x0 = (uint32_t)((u0 + du) * w), x1 = (uint32_t)((u1 - du) * w);
        uint32_t y0 = (uint32_t)((v0 + dv) * h), y1 = (uint32_t)((v1 - dv) * h);
        size_t n = 0;
        for (uint32_t y = y0; y < y1; y++)
            for (uint32_t x = x0; x < x1; x++, n++)
                for (int k = 0; k < 3; k++) acc[k] += rgb[((size_t)y * w + x) * 3 + k];
        double e = 0;
        for (int k = 0; k < 3; k++) e += fabs(acc[k] / n - synth_patch[i][k]);
        e /= 3;
        tot += e;
        if (e > *maxe) *maxe = e;
    }
    return tot / 24;
}

static void to_rgb(const frame *f, const gisp_params *p, uint8_t *rgb)
{
    gisp_nv12_to_rgb8(&f->nv, p->matrix, p->full_range, rgb, (size_t)f->w * 3);
}

static void write_ppm(const char *path, const uint8_t *rgb, uint32_t w, uint32_t h)
{
    FILE *f = fopen(path, "wb");
    if (!f) return;
    fprintf(f, "P6\n%u %u\n255\n", w, h);
    fwrite(rgb, 3, (size_t)w * h, f);
    fclose(f);
}

static double now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

/* tempo CPU del thread: sul PC carico dagli altri processi e' piu' stabile del tempo a parete */
static double cpu_now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

/* caso di qualita': RAW w x h (ritaglio crop) -> uscita ow x oh */
static void quality_case(const char *name, const synth_cam *c, const scene *s, uint32_t ow, uint32_t oh, int hq,
                         uint32_t mode, double min_psnr, double max_patch, const char *ppmdir)
{
    gisp_params p;
    synth_params(c, &p, 1);
    gisp_raw r = raw10(s, c->cfa);
    uint8_t *ref = malloc((size_t)ow * oh * 3), *rgb = malloc((size_t)ow * oh * 3);
    synth_reference(ow, oh, (double)s->w / s->h, ref);
    frame f = frame_new(ow, oh);
    int rc;
    if (hq) {
        gisp_ctx *ctx = gisp_ctx_new();
        rc = gisp_process(ctx, &r, NULL, &p, &f.nv);
        gisp_ctx_free(ctx);
    } else {
        gisp_fast *fx = gisp_fast_new(&r, NULL, ow, oh, mode, &p);
        rc = fx ? gisp_fast_run(fx, &r, &f.nv, 4) : -100;
        gisp_fast_free(fx);
    }
    CHECK(rc == 0, "%s: rc %d", name, rc);
    to_rgb(&f, &p, rgb);
    double mx, pe = patch_error(rgb, ow, oh, &mx), ps = psnr(rgb, ref, ow, oh, 4);
    printf("  %-34s PSNR %5.2f dB  toppe: errore medio %4.2f, max %4.2f (livelli sRGB)\n", name, ps, pe, mx);
    CHECK(ps >= min_psnr, "%s: PSNR %.2f < %.2f", name, ps, min_psnr);
    CHECK(pe <= max_patch, "%s: errore toppe %.2f > %.2f", name, pe, max_patch);
    if (ppmdir) {
        char path[512];
        snprintf(path, sizeof path, "%s/%s.ppm", ppmdir, name);
        write_ppm(path, rgb, ow, oh);
        snprintf(path, sizeof path, "%s/%s-ref.ppm", ppmdir, name);
        write_ppm(path, ref, ow, oh);
    }
    free(ref); free(rgb); free(f.buf);
}

static int same_frame(const frame *a, const frame *b) { return !memcmp(a->buf, b->buf, (size_t)a->w * a->h * 3 / 2); }

static void test_exactness(const synth_cam *c, const scene *s)
{
    gisp_params p;
    synth_params(c, &p, 1);
    gisp_raw r10 = raw10(s, c->cfa);
    gisp_raw r16 = {s->w, s->h, s->w * 2, GISP_RAW16, c->cfa, (const uint8_t *)s->r16};
    /* spacchettamento */
    int bad = 0;
    for (uint32_t y = 0; y < s->h; y += 7)
        for (uint32_t x = 0; x < s->w; x += 3)
            if (gisp_raw_pixel(&r10, x, y) != s->r16[(size_t)y * s->w + x]) bad++;
    CHECK(!bad, "RAW10: %d pixel diversi", bad);
    const uint32_t outs[][2] = {{1280, 720}, {s->w / 2, s->h / 2}, {640, 360}, {1440, 1080}};
    for (unsigned k = 0; k < sizeof outs / sizeof outs[0]; k++) {
        for (uint32_t mode = GISP_FAST_FULL; mode <= GISP_FAST_BIN2; mode++) {
            uint32_t ow = outs[k][0], oh = outs[k][1];
            frame a = frame_new(ow, oh), b = frame_new(ow, oh), d = frame_new(ow, oh);
            gisp_fast *f10 = gisp_fast_new(&r10, NULL, ow, oh, mode, &p);
            gisp_fast *f16 = gisp_fast_new(&r16, NULL, ow, oh, mode, &p);
            CHECK(f10 && f16, "fast_new %ux%u modo %u", ow, oh, mode);
            if (!f10 || !f16) { gisp_fast_free(f10); gisp_fast_free(f16); continue; }
            gisp_fast_run(f10, &r10, &a.nv, 1);
            gisp_fast_run(f10, &r10, &b.nv, 5);
            gisp_fast_run(f16, &r16, &d.nv, 3);
            CHECK(same_frame(&a, &b), "%ux%u modo %u: 1 thread != 5 thread", ow, oh, mode);
            CHECK(same_frame(&a, &d), "%ux%u modo %u: RAW10 != RAW16", ow, oh, mode);
            gisp_fast_free(f10); gisp_fast_free(f16);
            free(a.buf); free(b.buf); free(d.buf);
        }
    }
    /* ritaglio + cambio parametri al volo */
    gisp_rect crop = {40, 20, s->w - 80, s->h - 40};
    frame a = frame_new(1280, 720), b = frame_new(1280, 720);
    gisp_fast *f = gisp_fast_new(&r10, &crop, 1280, 720, GISP_FAST_AUTO, &p);
    CHECK(f != NULL, "fast_new con ritaglio");
    if (f) {
        gisp_fast_run(f, &r10, &a.nv, 2);
        gisp_params q = p;
        q.wb[0] *= 1.3f;
        gisp_fast_set_params(f, &q);
        gisp_fast_run(f, &r10, &b.nv, 2);
        CHECK(!same_frame(&a, &b), "set_params non ha effetto");
        gisp_fast_set_params(f, &p);
        gisp_fast_run(f, &r10, &b.nv, 3);
        CHECK(same_frame(&a, &b), "set_params non ripristina");
        gisp_fast_free(f);
    }
    free(a.buf); free(b.buf);
}

static void test_stats(const synth_cam *c, const scene *s)
{
    gisp_params p;
    synth_params(c, &p, 1);
    p.wb[0] = p.wb[1] = p.wb[2] = 1; /* il 3A misura prima del WB */
    gisp_raw r = raw10(s, c->cfa);
    static gisp_stats st;
    memset(&st, 0, sizeof st);
    st.zones_x = 16; st.zones_y = 12; st.step = 1;
    gisp_stats_compute(&r, NULL, &p, &st);
    CHECK(st.total == (s->w / 2) * (s->h / 2), "stats: total %u", st.total);
    /* grigio del mondo puro e con il luogo dei riferimenti (punto del grigio vero + due vicini) */
    float wb[3], cct = 0;
    int rc = gisp_awb_estimate(&st, NULL, wb, &cct);
    printf("  AWB grigio del mondo:  R %.3f B %.3f (vero %.3f %.3f)\n", wb[0], wb[2], c->wb[0], c->wb[2]);
    CHECK(rc == 0 && fabs(wb[0] / c->wb[0] - 1) < 0.12 && fabs(wb[2] / c->wb[2] - 1) < 0.12, "AWB grigio del mondo");
    static gisp_tuning t;
    memset(&t, 0, sizeof t);
    t.n_ref = 3;
    const float g_rg = (float)(1 / c->wb[0]), g_bg = (float)(1 / c->wb[2]);
    t.ref_rg[0] = g_rg * 0.8f; t.ref_bg[0] = g_bg * 1.25f; t.ref_cct[0] = 6500;
    t.ref_rg[1] = g_rg;        t.ref_bg[1] = g_bg;         t.ref_cct[1] = 5000;
    t.ref_rg[2] = g_rg * 1.3f; t.ref_bg[2] = g_bg * 0.75f; t.ref_cct[2] = 3000;
    rc = gisp_awb_estimate(&st, &t, wb, &cct);
    printf("  AWB con luogo:         R %.3f B %.3f CCT %.0f K (atteso ~5000)\n", wb[0], wb[2], cct);
    CHECK(rc == 0 && fabs(wb[0] / c->wb[0] - 1) < 0.04 && fabs(wb[2] / c->wb[2] - 1) < 0.04, "AWB con luogo");
    CHECK(cct > 4300 && cct < 5800, "AWB CCT %.0f", cct);
    /* media della luma: scena con WB applicato */
    gisp_params pw = p;
    for (int i = 0; i < 3; i++) pw.wb[i] = (float)c->wb[i];
    static gisp_stats s2;
    memset(&s2, 0, sizeof s2);
    s2.zones_x = 16; s2.zones_y = 12;
    gisp_stats_compute(&r, NULL, &pw, &s2);
    printf("  luma media %.3f (step 2), saturi %u\n", s2.mean_luma, s2.sat[0]);
    CHECK(s2.mean_luma > 0.10 && s2.mean_luma < 0.40, "luma media %.3f", s2.mean_luma);
    /* nitidezza: la zona del zone plate deve avere piu' contrasto di una zona uniforme dello sfondo */
    uint32_t zp = 6 * 16 + 13, flat = 11 * 16 + 1;
    float shz = s2.sharp[zp] / (s2.count[zp] + s2.sat[zp]), shf = s2.sharp[flat] / (s2.count[flat] + s2.sat[flat]);
    printf("  contrasto AF: zone plate %.4f, sfondo %.4f\n", shz, shf);
    CHECK(shz > 4 * shf, "contrasto AF");
    static gisp_stats_wire w;
    CHECK(gisp_stats_to_wire(&s2, 7, &w) == 0 && w.size == sizeof w && sizeof w <= 4096, "wire");
    printf("  wire: %zu byte, p50 %.3f p95 %.3f\n", sizeof w, w.p50_q12 / 4096.0, w.p95_q12 / 4096.0);
    /* tuning: ida e ritorno */
    static gisp_tuning tt, t2;
    memset(&tt, 0, sizeof tt);
    strcpy(tt.sensor, "synth");
    tt.cfa = c->cfa; tt.white = 1023;
    for (int i = 0; i < 4; i++) tt.black[i] = 64;
    tt.n_ccm = 2;
    tt.ccm_cct[0] = 3000; tt.ccm_cct[1] = 6500;
    for (int i = 0; i < 9; i++) { tt.ccm[0][i] = (i % 4 == 0) ? 1.0f : 0; tt.ccm[1][i] = (i % 4 == 0) ? 2.0f : 0; }
    gisp_gamma_srgb(tt.gamma);
    uint8_t *blob = malloc(12 + sizeof tt);
    uint32_t h[3] = {GISP_TUNING_MAGIC, GISP_TUNING_VERSION, sizeof tt};
    memcpy(blob, h, 12); memcpy(blob + 12, &tt, sizeof tt);
    CHECK(gisp_tuning_parse(blob, 12 + sizeof tt, &t2) == 0, "tuning_parse");
    gisp_params q;
    gisp_params_default(&q);
    gisp_tuning_apply(&t2, 4000, &q);
    /* 4000 K: 250 mired, tra 333 (3000 K) e 154 (6500 K): peso (333-250)/(333-154) = 0.466 */
    CHECK(fabsf(q.ccm[0] - 1.466f) < 0.01f, "tuning_apply CCM %.3f", q.ccm[0]);
    free(blob);
}

static void bench(int iters)
{
    synth_cam c;
    synth_cam_default(&c);
    gisp_params p;
    synth_params(&c, &p, 1);
    struct { const char *name; uint32_t w, h, ow, oh, mode; } cases[] = {
        {"720p RAW -> 720p (FULL)", 1280, 720, 1280, 720, GISP_FAST_FULL},
        {"1080p RAW -> 720p (FULL)", 1920, 1080, 1280, 720, GISP_FAST_FULL},
        {"1080p RAW -> 540p (BIN2)", 1920, 1080, 960, 540, GISP_FAST_BIN2},
        {"12MP 4000x2250 -> 720p (BIN2)", 4000, 2250, 1280, 720, GISP_FAST_BIN2},
        {"1080p RAW -> 1080p (FULL)", 1920, 1080, 1920, 1080, GISP_FAST_FULL},
    };
    printf("percorso fast: %s\n", gisp_fast_has_neon() ? "NEON" : "C");
    for (unsigned k = 0; k < sizeof cases / sizeof cases[0]; k++) {
        scene s = make_scene(&c, cases[k].w, cases[k].h);
        gisp_raw r = raw10(&s, c.cfa);
        frame f = frame_new(cases[k].ow, cases[k].oh);
        gisp_fast *fx = gisp_fast_new(&r, NULL, cases[k].ow, cases[k].oh, cases[k].mode, &p);
        printf("  %-32s", cases[k].name);
        {
            gisp_fast_run(fx, &r, &f.nv, 1);
            double t0 = cpu_now();
            for (int i = 0; i < iters; i++) gisp_fast_run(fx, &r, &f.nv, 1);
            printf("  CPU 1 thr %6.2f ms |", (cpu_now() - t0) / iters * 1e3);
        }
        for (uint32_t th = 2; th <= 4; th *= 2) {
            gisp_fast_run(fx, &r, &f.nv, th);
            double t0 = now();
            for (int i = 0; i < iters; i++) gisp_fast_run(fx, &r, &f.nv, th);
            double ms = (now() - t0) / iters * 1e3;
            printf("  parete %u thr %6.2f ms", th, ms);
        }
        static gisp_stats st;
        st.zones_x = 16; st.zones_y = 12; st.step = 2;
        double t0 = cpu_now();
        for (int i = 0; i < iters; i++) { st.zones_x = 16; st.zones_y = 12; st.step = 2; gisp_stats_compute(&r, NULL, &p, &st); }
        printf("  | stats CPU %5.2f ms\n", (cpu_now() - t0) / iters * 1e3);
        gisp_fast_free(fx);
        free(f.buf); free(s.r10); free(s.r16);
    }
}

static void dump(const char *dir)
{
    synth_cam c;
    synth_cam_default(&c);
    gisp_params p;
    synth_params(&c, &p, 1);
    scene s = make_scene(&c, 1920, 1080);
    gisp_raw r = raw10(&s, c.cfa);
    const uint32_t outs[][3] = {{1280, 720, GISP_FAST_FULL}, {1920, 1080, GISP_FAST_FULL}, {960, 540, GISP_FAST_BIN2},
                                {1000, 562, GISP_FAST_FULL}, {2560, 1440, GISP_FAST_FULL}};
    for (unsigned k = 0; k < sizeof outs / sizeof outs[0]; k++) {
        frame f = frame_new(outs[k][0], outs[k][1]);
        gisp_fast *fx = gisp_fast_new(&r, NULL, outs[k][0], outs[k][1], outs[k][2], &p);
        if (!fx) { printf("fast_new fallito %u\n", k); continue; }
        gisp_fast_run(fx, &r, &f.nv, 2);
        char path[512];
        snprintf(path, sizeof path, "%s/fast-%ux%u-%u.nv12", dir, outs[k][0], outs[k][1], outs[k][2]);
        FILE *o = fopen(path, "wb");
        if (o) { fwrite(f.buf, 1, (size_t)f.w * f.h * 3 / 2, o); fclose(o); }
        gisp_fast_free(fx);
        free(f.buf);
    }
    free(s.r10); free(s.r16);
}

int main(int argc, char **argv)
{
    if (argc > 1 && !strcmp(argv[1], "bench")) { bench(argc > 2 ? atoi(argv[2]) : 20); return 0; }
    if (argc > 2 && !strcmp(argv[1], "dump")) { dump(argv[2]); return 0; }
    const char *ppmdir = argc > 2 && !strcmp(argv[1], "ppm") ? argv[2] : NULL;
    int quick = argc > 1 && !strcmp(argv[1], "quick"); /* sotto qemu: niente HQ grande */

    synth_cam c;
    synth_cam_default(&c);
    printf("guamp-isp test (%s)\n", gisp_fast_has_neon() ? "NEON" : "C");
    scene s1080 = make_scene(&c, 1920, 1080);
    printf("esattezza:\n");
    test_exactness(&c, &s1080);
    printf("statistiche:\n");
    test_stats(&c, &s1080);
    printf("qualita' (rispetto alla scena vera):\n");
    quality_case("fast-1080-720-FULL", &c, &s1080, 1280, 720, 0, GISP_FAST_FULL, 27, 3.0, ppmdir);
    quality_case("fast-1080-540-BIN2", &c, &s1080, 960, 540, 0, GISP_FAST_BIN2, 25, 3.0, ppmdir);
    quality_case("fast-1080-1080-FULL", &c, &s1080, 1920, 1080, 0, GISP_FAST_FULL, 24, 3.0, ppmdir);
    if (!quick) quality_case("hq-1080-720", &c, &s1080, 1280, 720, 1, 0, 27, 3.0, ppmdir);
    scene s720 = make_scene(&c, 1280, 720);
    quality_case("fast-720-720-FULL", &c, &s720, 1280, 720, 0, GISP_FAST_FULL, 22, 3.0, ppmdir);
    if (!quick) {
        scene s12 = make_scene(&c, 4000, 2250);
        quality_case("fast-12MP-720-BIN2", &c, &s12, 1280, 720, 0, GISP_FAST_BIN2, 27, 3.0, ppmdir);
        free(s12.r10); free(s12.r16);
    }
    /* CFA diversi */
    for (uint32_t cfa = GISP_RGGB; cfa <= GISP_BGGR; cfa++) {
        synth_cam c2 = c;
        c2.cfa = cfa;
        scene s = make_scene(&c2, 640, 480);
        char name[64];
        snprintf(name, sizeof name, "fast-cfa%u-640-480", cfa);
        quality_case(name, &c2, &s, 640, 480, 0, GISP_FAST_FULL, 20, 3.5, NULL);
        free(s.r10); free(s.r16);
    }
    free(s720.r10); free(s720.r16); free(s1080.r10); free(s1080.r16);
    printf(fails ? "%d test FALLITI\n" : "tutto ok\n", fails);
    return fails ? 1 : 0;
}
