/* SPDX-License-Identifier: BSD-2-Clause
 * gisp_cli: elabora un frame RAW (es. il primo dump TFE RDI) sul PC.
 *   gisp_cli -i frame.raw -w 1920 -h 1080 [-s stride] [-f raw10|raw16] [-c gbrg|rggb|grbg|bggr|mono]
 *            [-t gm1st.gisp] [--cct 5000] [--wb r,g,b | --awb] [--black n] [-W 1280 -H 720] [--hq] [--bin2]
 *            [-o out.ppm] [--nv12 out.nv12] [--stats out.json]
 *   gisp_cli --synth frame.raw -w 1920 -h 1080      scrive un RAW10 sintetico (scena di test, GBRG)
 * Senza -t: nero 64, bianco 1023, CCM identita', gamma sRGB. Con --awb il WB viene dalle statistiche del frame. */
#include "gisp.h"
#include "synth.h"

#include <stdio.h>

static void usage(void)
{
    fprintf(stderr, "uso: gisp_cli -i RAW -w W -h H [opzioni]  (vedi l'intestazione di tools/gisp_cli.c)\n");
    exit(2);
}

static void *read_file(const char *path, size_t *size)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    void *d = malloc(n > 0 ? (size_t)n : 1);
    if (d && fread(d, 1, (size_t)n, f) != (size_t)n) { free(d); d = NULL; }
    fclose(f);
    *size = (size_t)n;
    return d;
}

static uint32_t parse_cfa(const char *s)
{
    static const char *n[] = {"rggb", "grbg", "gbrg", "bggr", "mono"};
    for (uint32_t i = 0; i < 5; i++)
        if (!strcmp(s, n[i])) return i;
    usage();
    return 0;
}

int main(int argc, char **argv)
{
    const char *in = NULL, *out = NULL, *nv = NULL, *stats = NULL, *tun = NULL, *synth = NULL;
    uint32_t w = 0, h = 0, stride = 0, fmt = GISP_RAW10_MIPI, cfa = GISP_GBRG, ow = 0, oh = 0, mode = GISP_FAST_AUTO;
    int hq = 0, awb = 0, have_wb = 0;
    float wbv[3] = {1, 1, 1}, cct = 5000, black = -1;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
#define ARG(name) (!strcmp(a, name) && v && (i++, 1))
        if (ARG("-i")) in = v;
        else if (ARG("--synth")) synth = v;
        else if (ARG("-w")) w = (uint32_t)atoi(v);
        else if (ARG("-h")) h = (uint32_t)atoi(v);
        else if (ARG("-s")) stride = (uint32_t)atoi(v);
        else if (ARG("-f")) fmt = !strcmp(v, "raw16") ? GISP_RAW16 : GISP_RAW10_MIPI;
        else if (ARG("-c")) cfa = parse_cfa(v);
        else if (ARG("-t")) tun = v;
        else if (ARG("--cct")) cct = (float)atof(v);
        else if (ARG("--wb")) { have_wb = sscanf(v, "%f,%f,%f", &wbv[0], &wbv[1], &wbv[2]) == 3; }
        else if (ARG("--black")) black = (float)atof(v);
        else if (ARG("-W")) ow = (uint32_t)atoi(v);
        else if (ARG("-H")) oh = (uint32_t)atoi(v);
        else if (ARG("-o")) out = v;
        else if (ARG("--nv12")) nv = v;
        else if (ARG("--stats")) stats = v;
        else if (!strcmp(a, "--hq")) hq = 1;
        else if (!strcmp(a, "--bin2")) mode = GISP_FAST_BIN2;
        else if (!strcmp(a, "--awb")) awb = 1;
        else usage();
    }
    if (!w || !h) usage();
    if (synth) {
        synth_cam c;
        synth_cam_default(&c);
        uint16_t *r16 = malloc(sizeof(uint16_t) * w * h);
        uint32_t st = w * 5 / 4;
        uint8_t *r10 = malloc((size_t)st * h);
        synth_raw16(&c, w, h, r16);
        synth_pack10(r16, w, h, r10, st);
        FILE *f = fopen(synth, "wb");
        if (!f) return 1;
        fwrite(r10, 1, (size_t)st * h, f);
        fclose(f);
        printf("RAW10 sintetico %ux%u GBRG, WB vero %.2f,1,%.2f\n", w, h, c.wb[0], c.wb[2]);
        return 0;
    }
    if (!in) usage();
    if (!stride) stride = fmt == GISP_RAW16 ? w * 2 : w * 5 / 4;
    size_t size;
    uint8_t *data = read_file(in, &size);
    if (!data || size < (size_t)stride * (h - 1) + (fmt == GISP_RAW16 ? w * 2 : w * 5 / 4)) {
        fprintf(stderr, "%s: file troppo corto o illeggibile (%zu byte)\n", in, data ? size : 0);
        return 1;
    }
    gisp_raw raw = {w, h, stride, fmt, cfa, data};
    gisp_params p;
    gisp_params_default(&p);
    static gisp_tuning t;
    int have_t = 0;
    if (tun) {
        size_t ts;
        void *td = read_file(tun, &ts);
        int rc = td ? gisp_tuning_parse(td, ts, &t) : -1;
        free(td);
        if (rc) { fprintf(stderr, "%s: tuning non valido (%d)\n", tun, rc); return 1; }
        gisp_tuning_apply(&t, cct, &p);
        have_t = 1;
    }
    if (black >= 0) for (int i = 0; i < 4; i++) p.black[i] = black;
    static gisp_stats st;
    st.zones_x = 16; st.zones_y = 12; st.step = 2;
    gisp_stats_compute(&raw, NULL, &p, &st);
    if (awb) {
        float e = 0;
        if (gisp_awb_estimate(&st, have_t ? &t : NULL, wbv, &e) == 0) {
            have_wb = 1;
            printf("AWB: R %.3f G 1 B %.3f, CCT %.0f K\n", wbv[0], wbv[2], e);
            if (have_t && e > 0) gisp_tuning_apply(&t, e, &p);
        }
    }
    if (have_wb) memcpy(p.wb, wbv, sizeof wbv);
    if (!ow) { ow = w & ~1u; oh = h & ~1u; }
    uint8_t *buf = calloc((size_t)ow * oh * 3 / 2, 1);
    gisp_nv12 o = {ow, oh, buf, ow, buf + (size_t)ow * oh, ow};
    int rc;
    if (hq) {
        gisp_ctx *ctx = gisp_ctx_new();
        rc = gisp_process(ctx, &raw, NULL, &p, &o);
        gisp_ctx_free(ctx);
    } else {
        gisp_fast *f = gisp_fast_new(&raw, NULL, ow, oh, mode, &p);
        rc = f ? gisp_fast_run(f, &raw, &o, 4) : -100;
        gisp_fast_free(f);
    }
    if (rc) { fprintf(stderr, "elaborazione fallita (%d)\n", rc); return 1; }
    if (nv) {
        FILE *f = fopen(nv, "wb");
        if (f) { fwrite(buf, 1, (size_t)ow * oh * 3 / 2, f); fclose(f); }
    }
    if (out) {
        uint8_t *rgb = malloc((size_t)ow * oh * 3);
        gisp_nv12_to_rgb8(&o, p.matrix, p.full_range, rgb, (size_t)ow * 3);
        FILE *f = fopen(out, "wb");
        if (f) { fprintf(f, "P6\n%u %u\n255\n", ow, oh); fwrite(rgb, 3, (size_t)ow * oh, f); fclose(f); }
        free(rgb);
    }
    if (stats) {
        FILE *f = fopen(stats, "w");
        if (f) {
            fprintf(f, "{\"zones\":[%u,%u],\"total\":%u,\"mean_luma\":%.5f,\"range\":%.1f,\"zone\":[", st.zones_x,
                    st.zones_y, st.total, st.mean_luma, st.range);
            for (uint32_t z = 0; z < st.zones_x * st.zones_y; z++) {
                uint32_t n = st.count[z] ? st.count[z] : 1;
                fprintf(f, "%s[%.2f,%.2f,%.2f,%u,%u,%.4f]", z ? "," : "", st.sum_r[z] / n, st.sum_g[z] / n,
                        st.sum_b[z] / n, st.count[z], st.sat[z], st.sharp[z]);
            }
            fprintf(f, "],\"hist\":[");
            for (int i = 0; i < 256; i++) fprintf(f, "%s%u", i ? "," : "", st.hist[i]);
            fprintf(f, "],\"wb\":[%.4f,%.4f,%.4f]}\n", p.wb[0], p.wb[1], p.wb[2]);
            fclose(f);
        }
    }
    printf("ok: %ux%u -> %ux%u (%s), luma media %.3f\n", w, h, ow, oh,
           hq ? "HQ" : (mode == GISP_FAST_BIN2 ? "fast BIN2" : "fast"), st.mean_luma);
    free(buf); free(data);
    return 0;
}
