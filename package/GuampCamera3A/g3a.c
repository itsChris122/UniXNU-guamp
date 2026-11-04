/* SPDX-License-Identifier: BSD-2-Clause
 * guamp 3A (see g3a.h). Own design: centre-weighted AE with highlight protection and flicker-safe time/gain split,
 * grey-world AWB weighted towards near-grey zones (optional white locus), hill-climbing contrast AF with a parabolic
 * fine peak, pre-flash metering with ambient/flash separation. No vendor tables or code. */
#include "g3a.h"
#include <math.h>
#include <string.h>

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
static uint32_t nzones(const g3a_stats *s) {
    const uint32_t n = s->zones_x * s->zones_y;
    return n > G3A_ZONES_MAX ? G3A_ZONES_MAX : n;
}
static float lum(float r, float g, float b, const float wb[3]) { return 0.2126f * wb[0] * r + 0.7152f * wb[1] * g + 0.0722f * wb[2] * b; }

/* Centre weight of zone i: 1 + cw x gaussian (sigma = a quarter of the frame). */
static float zone_weight(const g3a_stats *s, uint32_t i, float cw) {
    if (cw <= 0.f) return 1.f;
    const float u = ((float)(i % s->zones_x) + 0.5f) / (float)s->zones_x - 0.5f;
    const float v = ((float)(i / s->zones_x) + 0.5f) / (float)s->zones_y - 0.5f;
    return 1.f + cw * expf(-(u * u + v * v) / (2.f * 0.25f * 0.25f));
}

/* ---- AE ------------------------------------------------------------------------------------------------------------ */
void g3a_ae_default(g3a_ae_config *c) {
    memset(c, 0, sizeof(*c));
    c->target = 0.18f;
    c->center_weight = 2.f;
    c->highlight_frac = 0.02f;
    c->min_target_scale = 0.5f;
    c->t_min_ns = 20000;
    c->t_max_ns = 33333333;
    c->gain_max = 16.f;
    c->flicker_hz = 50;
    c->max_step = 4.f;
    c->damping = 0.6f;
    c->tolerance = 0.1f;
}

float g3a_ae_measure(const g3a_stats *s, const g3a_ae_config *c, const float wb[3], float *highlights) {
    static const float one[3] = {1.f, 1.f, 1.f};
    if (!wb) wb = one;
    const uint32_t n = nzones(s);
    double sw = 0, sy = 0, quads = 0, satq = 0;
    for (uint32_t i = 0; i < n; ++i) {
        const uint32_t cnt = s->count[i], sat = s->sat ? s->sat[i] : 0;
        if (!cnt && !sat) continue;
        float y = 1.f;   /* saturated quads count as white */
        if (cnt) y = lum(s->sum_r[i], s->sum_g[i], s->sum_b[i], wb) / ((float)cnt * s->white);
        y = clampf(((float)cnt * y + (float)sat) / (float)(cnt + sat), 0.f, 1.5f);
        const float w = zone_weight(s, i, c->center_weight);
        sw += w; sy += w * y;
        quads += cnt + sat; satq += sat;
    }
    if (highlights) {
        if (s->hist) {
            double tot = 0, top = 0;
            for (unsigned b = 0; b < 256; ++b) { tot += s->hist[b]; if (b >= 248) top += s->hist[b]; }
            *highlights = tot > 0 ? (float)(top / tot) : 0.f;
        } else {
            *highlights = quads > 0 ? (float)(satq / quads) : 0.f;
        }
    }
    return sw > 0 ? (float)(sy / sw) : 0.f;
}

void g3a_ae_split(const g3a_ae_config *c, double total, uint64_t *ns, float *gain) {
    double t = total;
    if (t > (double)c->t_max_ns) t = (double)c->t_max_ns;
    if (t < (double)c->t_min_ns) t = (double)c->t_min_ns;
    if (c->flicker_hz) {
        const double period = 1e9 / (2.0 * c->flicker_hz);   /* light flickers at twice the mains frequency */
        if (t >= period) t = floor(t / period) * period;
    }
    double g = total / t;
    if (g < 1.0) g = 1.0;
    if (g > c->gain_max) g = c->gain_max;
    *ns = (uint64_t)(t + 0.5);
    *gain = (float)g;
}

void g3a_ae_run(const g3a_stats *s, const g3a_ae_config *c, const float wb[3], uint64_t applied_ns, float applied_gain,
                g3a_ae_result *out) {
    float hl = 0.f;
    const float y = g3a_ae_measure(s, c, wb, &hl);
    float tgt = c->target;
    if (hl > c->highlight_frac && hl > 0.f) tgt *= clampf(sqrtf(c->highlight_frac / hl), c->min_target_scale, 1.f);
    const double cur = (double)applied_ns * (applied_gain < 1.f ? 1.f : applied_gain);
    const float maxe = log2f(c->max_step > 1.f ? c->max_step : 1.f);
    float err = y > 1e-5f ? log2f(tgt / y) : maxe;
    const int conv = fabsf(err) < c->tolerance;
    if (conv) err = 0.f;                      /* hold: no hunting inside the tolerance */
    else if (fabsf(err) < 1.f) err *= c->damping;
    err = clampf(err, -maxe, maxe);
    const double want = cur * exp2((double)err);
    g3a_ae_split(c, want, &out->exposure_ns, &out->gain);
    const double got = (double)out->exposure_ns * out->gain;
    out->luma = y;
    out->target = tgt;
    out->converged = conv;
    out->limited = want > got * 1.001 ? 1 : want < got * 0.999 ? -1 : 0;
}

/* ---- AWB ----------------------------------------------------------------------------------------------------------- */
void g3a_awb_default(g3a_awb_config *c) {
    memset(c, 0, sizeof(*c));
    c->gain_min = 0.5f;
    c->gain_max = 4.f;
    c->grey_sigma = 0.35f;
    c->dark = 0.02f;
    c->bright = 0.9f;
    c->speed = 0.25f;
    c->ref_max_dist = 0.08f;
}
void g3a_awb_reset(g3a_awb_state *st) {
    memset(st, 0, sizeof(*st));
    st->wb[0] = st->wb[1] = st->wb[2] = 1.f;
}

/* White point (log R/G, log B/G) pulled to within max_dist of the locus polyline; cct interpolated along it. */
static void awb_locus(const g3a_awb_config *c, float *lr, float *lb, float *cct) {
    if (c->n_ref == 0) { *cct = 0.f; return; }
    if (c->n_ref == 1) { *cct = c->ref_cct[0]; return; }
    float best = 1e30f, px = *lr, py = *lb, bc = 0.f;
    const uint32_t n = c->n_ref > G3A_AWB_REF_MAX ? G3A_AWB_REF_MAX : c->n_ref;
    for (uint32_t k = 0; k + 1 < n; ++k) {
        const float ax = logf(c->ref_rg[k]), ay = logf(c->ref_bg[k]), bx = logf(c->ref_rg[k + 1]), by = logf(c->ref_bg[k + 1]);
        const float dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
        float t = l2 > 0 ? ((*lr - ax) * dx + (*lb - ay) * dy) / l2 : 0.f;
        t = clampf(t, 0.f, 1.f);
        const float qx = ax + t * dx, qy = ay + t * dy, d = (qx - *lr) * (qx - *lr) + (qy - *lb) * (qy - *lb);
        if (d < best) { best = d; px = qx; py = qy; bc = c->ref_cct[k] + t * (c->ref_cct[k + 1] - c->ref_cct[k]); }
    }
    const float d = sqrtf(best);
    if (d > c->ref_max_dist && d > 0) {
        const float k = c->ref_max_dist / d;
        *lr = px + (*lr - px) * k;
        *lb = py + (*lb - py) * k;
    }
    *cct = bc;
}

/* Weighted mean chromaticity of the usable zones; weight = brightness x closeness to the grey of `ref` gains. */
static int awb_pass(const g3a_stats *s, const g3a_awb_config *c, const float ref[3], float sigma, float *rg, float *bg,
                    uint32_t *used) {
    const uint32_t n = nzones(s);
    double sr = 0, sb = 0, sw = 0;
    uint32_t u = 0;
    for (uint32_t i = 0; i < n; ++i) {
        const uint32_t cnt = s->count[i];
        if (!cnt || (s->sat && s->sat[i] * 50u > cnt)) continue;
        const float r = s->sum_r[i] / (float)cnt, g = s->sum_g[i] / (float)cnt, b = s->sum_b[i] / (float)cnt;
        if (g < c->dark * s->white || r <= 0.f || b <= 0.f) continue;
        if (r > c->bright * s->white || g > c->bright * s->white || b > c->bright * s->white) continue;
        float w = g;
        if (sigma > 0.f) {
            const float lr = logf(r * ref[0] / g), lb = logf(b * ref[2] / g);
            w *= expf(-(lr * lr + lb * lb) / (2.f * sigma * sigma));
        }
        if (w <= 0.f) continue;
        sr += w * (r / g); sb += w * (b / g); sw += w;
        ++u;
    }
    if (used) *used = u;
    if (sw <= 0) return -1;
    *rg = (float)(sr / sw);
    *bg = (float)(sb / sw);
    return 0;
}

int g3a_awb_run(const g3a_stats *s, const g3a_awb_config *c, g3a_awb_state *st, float raw[3]) {
    float rg, bg;
    uint32_t used = 0;
    /* pass 1: wide window around the current gains; pass 2: narrow window around the pass 1 white */
    if (awb_pass(s, c, st->wb, c->grey_sigma * 2.f, &rg, &bg, &used)) { st->zones_used = 0; return -1; }
    if (c->grey_sigma > 0.f) {
        const float g1[3] = {1.f / rg, 1.f, 1.f / bg};
        float rg2, bg2;
        if (!awb_pass(s, c, g1, c->grey_sigma, &rg2, &bg2, &used)) { rg = rg2; bg = bg2; }
    }
    float lr = logf(rg), lb = logf(bg), cct = 0.f;
    awb_locus(c, &lr, &lb, &cct);
    const float tr = clampf(-lr, logf(c->gain_min), logf(c->gain_max));
    const float tb = clampf(-lb, logf(c->gain_min), logf(c->gain_max));
    if (raw) { raw[0] = expf(tr); raw[1] = 1.f; raw[2] = expf(tb); }
    const float cr = logf(st->wb[0]), cb = logf(st->wb[2]);
    const float k = clampf(c->speed, 0.f, 1.f);
    const float nr = cr + k * (tr - cr), nb = cb + k * (tb - cb);
    st->wb[0] = expf(nr); st->wb[1] = 1.f; st->wb[2] = expf(nb);
    st->converged = fabsf(tr - nr) < 0.02f && fabsf(tb - nb) < 0.02f;
    st->cct = cct;
    st->zones_used = used;
    return 0;
}

/* ---- AF ------------------------------------------------------------------------------------------------------------ */
void g3a_af_default(g3a_af_config *c, uint16_t dac_lo, uint16_t dac_hi) {
    memset(c, 0, sizeof(*c));
    c->dac_lo = dac_lo < dac_hi ? dac_lo : dac_hi;
    c->dac_hi = dac_lo < dac_hi ? dac_hi : dac_lo;
    c->coarse_steps = 12;
    c->fine_div = 4;
    c->settle_frames = 1;
    c->min_peak_ratio = 1.15f;
    c->drop_stop = 0.75f;
    c->hyperfocal = 0.15f;
    c->rescan_drop = 0.7f;
    c->rescan_frames = 5;
}

float g3a_af_measure(const g3a_stats *s, const g3a_af_config *c, float *luma) {
    uint32_t x0 = c->roi_x, y0 = c->roi_y, w = c->roi_w, h = c->roi_h;
    if (!w || !h) { w = (s->zones_x + 2) / 3; h = (s->zones_y + 2) / 3; x0 = (s->zones_x - w) / 2; y0 = (s->zones_y - h) / 2; }
    double sh = 0, sg = 0, cnt = 0;
    for (uint32_t y = y0; y < y0 + h && y < s->zones_y; ++y)
        for (uint32_t x = x0; x < x0 + w && x < s->zones_x; ++x) {
            const uint32_t i = y * s->zones_x + x;
            if (i >= G3A_ZONES_MAX) continue;
            if (s->sharp) sh += s->sharp[i];
            sg += s->sum_g[i];
            cnt += s->count[i];
        }
    if (luma) *luma = cnt > 0 ? (float)(sg / (cnt * s->white)) : 0.f;
    return sg > 0 ? (float)(sh / sg) : 0.f;
}

static uint16_t clampdac(const g3a_af_config *c, int v) { return (uint16_t)(v < c->dac_lo ? c->dac_lo : v > c->dac_hi ? c->dac_hi : v); }

static void af_pass(g3a_af_state *st, uint16_t from, uint16_t step) {
    st->n = 0;
    st->best_fv = 0.f;
    st->worst_fv = 1e30f;
    st->below = 0;
    st->step = step ? step : 1;
    st->dac = from;
}

void g3a_af_start(g3a_af_state *st, const g3a_af_config *c) {
    const uint32_t frames = st->frames, moves = st->moves;
    memset(st, 0, sizeof(*st));
    st->frames = frames; st->moves = moves + 1;
    uint16_t steps = c->coarse_steps < 2 ? 2 : c->coarse_steps > 64 ? 64 : c->coarse_steps;
    af_pass(st, c->dac_lo, (uint16_t)((c->dac_hi - c->dac_lo + steps - 2) / (steps - 1)));
    st->state = G3A_AF_COARSE;
    st->settle = c->settle_frames;
}

static void af_record(g3a_af_state *st, float fv) {
    if (st->n < 64) { st->pos[st->n] = st->dac; st->fv[st->n] = fv; st->n++; }
    if (fv > st->best_fv) { st->best_fv = fv; st->best_dac = st->dac; }
    if (fv < st->worst_fv) st->worst_fv = fv;
}

static void af_move(g3a_af_state *st, const g3a_af_config *c, uint16_t to) {
    if (to != st->dac) st->moves++;
    st->dac = to;
    st->settle = c->settle_frames;
}

uint16_t g3a_af_run(const g3a_stats *s, const g3a_af_config *c, g3a_af_state *st) {
    st->frames++;
    if (st->state == G3A_AF_IDLE) return st->dac;
    if (st->settle) { st->settle--; return st->dac; }
    float luma = 0.f;
    const float fv = g3a_af_measure(s, c, &luma);
    if (st->state == G3A_AF_COARSE) {
        af_record(st, fv);
        const int peaked = st->best_fv > st->worst_fv * c->min_peak_ratio;
        if (st->dac > st->best_dac && fv < c->drop_stop * st->best_fv) st->below++;
        else st->below = 0;
        if ((st->below >= 2 && peaked) || st->dac >= c->dac_hi) {
            if (!peaked) {                        /* flat: nothing to focus on (or too dark) */
                st->state = G3A_AF_FAILED;
                st->lock_fv = 0.f;
                st->drift = 0;
                af_move(st, c, clampdac(c, c->dac_lo + (int)(c->hyperfocal * (float)(c->dac_hi - c->dac_lo))));
                return st->dac;
            }
            const uint16_t coarse = st->step, centre = st->best_dac;
            const uint16_t div = c->fine_div < 1 ? 1 : c->fine_div > 31 ? 31 : c->fine_div;
            const uint16_t from = clampdac(c, (int)centre - (int)coarse);
            st->fine_end = clampdac(c, (int)centre + (int)coarse);
            af_pass(st, st->dac, (uint16_t)(coarse / div ? coarse / div : 1));
            st->state = G3A_AF_FINE;
            af_move(st, c, from);
            return st->dac;
        }
        af_move(st, c, clampdac(c, (int)st->dac + (int)st->step));
        return st->dac;
    }
    if (st->state == G3A_AF_FINE) {
        af_record(st, fv);
        if (st->dac < st->fine_end && st->n < 64) {
            af_move(st, c, clampdac(c, (int)st->dac + (int)st->step));
            return st->dac;
        }
        uint16_t k = 0;
        for (uint16_t i = 1; i < st->n; ++i) if (st->fv[i] > st->fv[k]) k = i;
        float pos = (float)st->pos[k];
        if (k > 0 && k + 1 < st->n) {             /* parabola through the best sample and its neighbours */
            const float a = st->fv[k - 1], b = st->fv[k], d = st->fv[k + 1];
            const float den = a - 2.f * b + d;
            if (den < 0.f) pos += clampf(0.5f * (a - d) / den, -0.5f, 0.5f) * (float)(st->pos[k + 1] - st->pos[k - 1]) * 0.5f;
        }
        st->state = G3A_AF_FOCUSED;
        st->lock_fv = 0.f;
        st->drift = 0;
        af_move(st, c, clampdac(c, (int)(pos + 0.5f)));
        return st->dac;
    }
    /* FOCUSED / FAILED: watch the scene in continuous mode */
    if (!c->continuous) return st->dac;
    if (st->lock_fv <= 0.f) { st->lock_fv = fv; st->lock_luma = luma; return st->dac; }
    const int moved = (st->state == G3A_AF_FOCUSED && fv < c->rescan_drop * st->lock_fv) ||
                      (st->lock_luma > 0.f && fabsf(luma / st->lock_luma - 1.f) > 0.5f) ||
                      (st->state == G3A_AF_FAILED && fv > st->lock_fv * c->min_peak_ratio * 1.2f);
    if (moved) st->drift++;
    else { st->drift = 0; if (fv > st->lock_fv) st->lock_fv = fv; }
    if (st->drift >= (c->rescan_frames ? c->rescan_frames : 1)) g3a_af_start(st, c);
    return st->dac;
}

/* ---- flash --------------------------------------------------------------------------------------------------------- */
void g3a_flash_default(g3a_flash_config *c) {
    memset(c, 0, sizeof(*c));
    c->pre_ma = 150;
    c->main_ma = 500;
    c->main_ms = 200;
    c->min_ma = 25;
    c->pre_exposure_div = 8.f;
    c->t_capture_max_ns = 66666666;
    c->target = 0.18f;
    c->highlight = 0.9f;
    c->need_luma = 0.6f;
}

int g3a_flash_needed(const g3a_flash_config *c, const g3a_ae_result *ae) {
    return ae->limited == 1 && ae->luma < c->need_luma * ae->target;
}

/* Zone means normalised to white, per unit exposure (ns x gain); -1 in y for zones that cannot be used. */
static void zone_unit(const g3a_stats *s, uint32_t i, double exposure, float *r, float *g, float *b, int *ok) {
    const uint32_t cnt = s->count[i];
    *ok = cnt > 0 && !(s->sat && s->sat[i] * 20u > cnt);
    const double k = cnt ? 1.0 / ((double)cnt * s->white * exposure) : 0.0;
    *r = (float)(s->sum_r[i] * k); *g = (float)(s->sum_g[i] * k); *b = (float)(s->sum_b[i] * k);
}

void g3a_flash_ambient(const g3a_stats *s, double exposure, g3a_flash_ambient_t *amb) {
    memset(amb, 0, sizeof(*amb));
    amb->zones = nzones(s);
    amb->exposure = exposure;
    for (uint32_t i = 0; i < amb->zones; ++i) {
        int ok;
        zone_unit(s, i, exposure, &amb->r[i], &amb->g[i], &amb->b[i], &ok);
        amb->y[i] = ok ? amb->g[i] : -1.f;
    }
    amb->valid = exposure > 0;
}

double g3a_flash_pre_exposure(const g3a_flash_config *c, const g3a_ae_config *ae, double ambient_exposure) {
    const double e = ambient_exposure / (c->pre_exposure_div > 1.f ? c->pre_exposure_div : 1.f);
    return e < (double)ae->t_min_ns ? (double)ae->t_min_ns : e;
}

void g3a_flash_plan(const g3a_stats *lit, double lit_exposure, const g3a_flash_ambient_t *amb, const g3a_flash_config *c,
                    const g3a_ae_config *ae, const float wb_ambient[3], g3a_flash_plan_t *out) {
    memset(out, 0, sizeof(*out));
    out->ms = c->main_ms;
    const uint32_t n = amb->zones < nzones(lit) ? amb->zones : nzones(lit);
    /* flash light per zone, per unit exposure and per mA */
    float fr[G3A_ZONES_MAX], fg[G3A_ZONES_MAX], fb[G3A_ZONES_MAX];
    double FR = 0, FG = 0, FB = 0;
    const float pre = (float)(c->pre_ma ? c->pre_ma : 1);
    double wclip = 0, wall = 0;
    for (uint32_t i = 0; i < n; ++i) {
        float r, g, b; int ok;
        if (amb->y[i] < 0.f || lit_exposure <= 0) { fr[i] = fg[i] = fb[i] = -1.f; continue; }
        zone_unit(lit, i, lit_exposure, &r, &g, &b, &ok);
        const float zw = zone_weight(lit, i, ae->center_weight);
        wall += zw;
        if (!ok) wclip += zw;
        if (!ok) {                                /* clipped under the pre-flash: at least white, colour unknown */
            const float w = (float)(1.0 / lit_exposure);
            fr[i] = fmaxf(0.f, w - amb->r[i]) / pre; fg[i] = fmaxf(0.f, w - amb->g[i]) / pre; fb[i] = fmaxf(0.f, w - amb->b[i]) / pre;
            continue;
        }
        fr[i] = fmaxf(0.f, r - amb->r[i]) / pre; fg[i] = fmaxf(0.f, g - amb->g[i]) / pre; fb[i] = fmaxf(0.f, b - amb->b[i]) / pre;
        FR += fr[i]; FG += fg[i]; FB += fb[i];
    }
    /* more than a quarter of the metered area clipped: the flash light is only known as a lower bound */
    if (wall > 0 && wclip > 0.25 * wall && lit_exposure > (double)ae->t_min_ns * 1.01) {
        out->ok = -1;
        out->retry_exposure = fmax((double)ae->t_min_ns, lit_exposure / 16.0);
        out->flash_wb[0] = out->flash_wb[1] = out->flash_wb[2] = 1.f;
        return;
    }
    const float wmin = 0.25f, wmax = 4.f;
    out->flash_wb[0] = FR > 0 ? clampf((float)(FG / FR), wmin, wmax) : 1.f;
    out->flash_wb[1] = 1.f;
    out->flash_wb[2] = FB > 0 ? clampf((float)(FG / FB), wmin, wmax) : 1.f;
    float wb[3] = {out->flash_wb[0], 1.f, out->flash_wb[2]};
    double A = 0, F = 0, sw = 0, E = 0;
    float I = (float)c->main_ma, frac = 0.f;
    for (int iter = 0; iter < 3; ++iter) {
        A = F = sw = 0;
        for (uint32_t i = 0; i < n; ++i) {
            if (fr[i] < 0.f) continue;
            const float w = zone_weight(lit, i, ae->center_weight);
            A += w * lum(amb->r[i], amb->g[i], amb->b[i], wb);
            F += w * lum(fr[i], fg[i], fb[i], wb);
            sw += w;
        }
        if (sw <= 0) return;
        A /= sw; F /= sw;
        /* pre-flash lit the subject by less than 5 % of the target at the ambient exposure: too far, no flash */
        if (F * pre * lit_exposure < 0.05 * c->target) { out->ok = 0; out->flash_wb[0] = out->flash_wb[2] = 1.f; return; }
        I = (float)c->main_ma;
        E = c->target / (A + I * F);
        const double emin = (double)ae->t_min_ns, emax = (double)c->t_capture_max_ns * ae->gain_max;
        if (E < emin) {                           /* too close: less current at the shortest exposure */
            I = clampf((float)((c->target / emin - A) / F), (float)c->min_ma, (float)c->main_ma);
            E = fmax(emin, c->target / (A + I * F));
        }
        /* brightest zone must stay below the highlight level (not below 60 % of the metered exposure) */
        double hmax = 0;
        for (uint32_t i = 0; i < n; ++i)
            if (fr[i] >= 0.f) hmax = fmax(hmax, lum(amb->r[i], amb->g[i], amb->b[i], wb) + I * lum(fr[i], fg[i], fb[i], wb));
        if (hmax * E > c->highlight) E = fmax(E * 0.6, c->highlight / hmax);
        if (E > emax) E = emax;
        if (E < emin) E = emin;
        frac = (float)(I * F / (A + I * F));
        /* white point = mix of the two lights in the proportion they reach the image */
        const float wr = frac / out->flash_wb[0] + (1.f - frac) / wb_ambient[0];
        const float wbb = frac / out->flash_wb[2] + (1.f - frac) / wb_ambient[2];
        wb[0] = 1.f / wr; wb[2] = 1.f / wbb;
    }
    g3a_ae_config cap = *ae;
    cap.t_max_ns = c->t_capture_max_ns;
    cap.flicker_hz = frac > 0.8f ? 0 : ae->flicker_hz;   /* flash dominates: no mains banding to avoid */
    g3a_ae_split(&cap, E, &out->exposure_ns, &out->gain);
    const double got = (double)out->exposure_ns * out->gain;
    out->ma = (uint32_t)(I + 0.5f);
    out->wb[0] = wb[0]; out->wb[1] = 1.f; out->wb[2] = wb[2];
    out->expected_luma = (float)(got * (A + I * F));
    out->flash_fraction = frac;
    out->ok = 1;
}
