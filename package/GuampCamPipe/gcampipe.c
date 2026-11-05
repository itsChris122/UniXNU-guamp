/* SPDX-License-Identifier: BSD-2-Clause
 * gcampipe - per-frame core of the guamp camera service (see gcampipe.h). C11 + libm, links gisp (agent 3) and g3a
 * (agent 4); no CoreFoundation, so it builds for the plug-in, for the host tests and for command-line probes. */
#include "gcampipe.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "g3a.h"
#include "g3a_gisp.h"

_Static_assert((int)GCP_AF_FOCUSED == (int)G3A_AF_FOCUSED && (int)GCP_AF_FAILED == (int)G3A_AF_FAILED && (int)GCP_AF_COARSE == (int)G3A_AF_COARSE,
               "gcp_af_state mirrors g3a_af_state");
_Static_assert((int)kGuampRawMipi10 == (int)GISP_RAW10_MIPI && (int)kGuampRawU16 == (int)GISP_RAW16, "raw formats");
_Static_assert((int)kGuampBayerGBRG == (int)GISP_GBRG && (int)kGuampBayerBGGR == (int)GISP_BGGR &&
               (int)kGuampBayerMono == (int)GISP_MONO, "CFA codes");

struct gcp {
    uint32_t sensor;
    GuampCamIosSensorInfo info;
    GuampCamIosModeInfo mode;
    gcp_ops ops;
    gisp_params params;
    gisp_tuning *tuning;          /* NULL: defaults */
    gisp_stats *stats;            /* ~21 KB */
    gisp_fast *fast;
    gisp_raw fastGeom;
    gisp_rect fastCrop;
    uint32_t fastW, fastH, fastFull;
    gisp_ctx *hq;
    g3a_ae_config ae;
    g3a_ae_result aeLast;
    g3a_awb_config awbCfg;
    g3a_awb_state awb;
    g3a_af_config afCfg;
    g3a_af_state af;
    int afActive, aeAllowed;
    uint32_t lastUs, lastQ8;
};

void gcp_controls_default(gcp_controls *c) {
    memset(c, 0, sizeof(*c));
    c->ae_enabled = 1;
    c->focus_request = -1;
}

const gisp_params *gcp_params(const gcp *p) { return &p->params; }

gcp *gcp_new(uint32_t sensor, const GuampCamIosSensorInfo *info, const GuampCamIosModeInfo *mode, float min_fps,
             const gcp_ops *ops, const void *tuning, size_t tuning_size) {
    if (!info || !mode || !mode->width || !mode->height) return NULL;
    gcp *p = calloc(1, sizeof(*p));
    if (!p) return NULL;
    p->sensor = sensor;
    p->info = *info;
    p->mode = *mode;
    if (ops) p->ops = *ops;
    p->stats = calloc(1, sizeof(gisp_stats));
    p->hq = gisp_ctx_new();
    if (!p->stats || !p->hq) { gcp_free(p); return NULL; }
    gisp_params_default(&p->params);
    for (int i = 0; i < 4; ++i) p->params.black[i] = (float)mode->blackLevel;
    p->params.white = (float)((1u << (info->bitDepth ? info->bitDepth : 10)) - 1);
    g3a_awb_default(&p->awbCfg);
    if (tuning && tuning_size) {
        p->tuning = malloc(sizeof(gisp_tuning));
        if (p->tuning && gisp_tuning_parse(tuning, tuning_size, p->tuning) == 0) {
            gisp_tuning_apply(p->tuning, 5000.f, &p->params);
            const uint32_t n = p->tuning->n_ref < G3A_AWB_REF_MAX ? p->tuning->n_ref : G3A_AWB_REF_MAX;
            p->awbCfg.n_ref = n;
            for (uint32_t i = 0; i < n; ++i) {
                p->awbCfg.ref_rg[i] = p->tuning->ref_rg[i];
                p->awbCfg.ref_bg[i] = p->tuning->ref_bg[i];
                p->awbCfg.ref_cct[i] = p->tuning->ref_cct[i];
            }
        } else {
            free(p->tuning);
            p->tuning = NULL;
        }
    }
    g3a_awb_reset(&p->awb);
    g3a_ae_default(&p->ae);
    p->ae.t_min_ns = (uint64_t)(mode->minExposureUs ? mode->minExposureUs : 30) * 1000u;
    const float fps = min_fps > 0 ? min_fps : mode->minFpsQ8 ? mode->minFpsQ8 / 256.f : 15.f;
    p->ae.t_max_ns = (uint64_t)(1e9 / fps);
    if (mode->maxExposureUs && p->ae.t_max_ns > (uint64_t)mode->maxExposureUs * 1000u)
        p->ae.t_max_ns = (uint64_t)mode->maxExposureUs * 1000u;
    p->ae.gain_max = mode->maxAnalogGainQ8 ? mode->maxAnalogGainQ8 / 256.f : 16.f;
    /* testPattern 2 = CAMSS TPG: the picture does not follow the exposure, AE would only run to its limits. */
    p->aeAllowed = info->testPattern != 2;
    g3a_af_default(&p->afCfg, (uint16_t)mode->lensMin, (uint16_t)(mode->lensMax ? mode->lensMax : 1023));
    return p;
}

void gcp_free(gcp *p) {
    if (!p) return;
    if (p->fast) gisp_fast_free(p->fast);
    if (p->hq) gisp_ctx_free(p->hq);
    free(p->stats);
    free(p->tuning);
    free(p);
}

int gcp_raw(const GuampCamIosFrame *fr, const uint8_t *slot, size_t slot_size, gisp_raw *out) {
    memset(out, 0, sizeof(*out));
    if (!fr || !slot || !fr->width || !fr->height || (fr->width & 1) || (fr->height & 1)) return -1;
    if (fr->rawFormat != kGuampRawMipi10 && fr->rawFormat != kGuampRawU16) return -1;
    if (fr->bayer > kGuampBayerMono) return -1;
    const uint64_t line = fr->rawFormat == kGuampRawMipi10 ? (uint64_t)fr->width * 10 / 8 : (uint64_t)fr->width * 2;
    if (fr->strideBytes < line || (uint64_t)fr->strideBytes * fr->height > slot_size) return -1;
    out->width = fr->width;
    out->height = fr->height;
    out->stride = fr->strideBytes;
    out->format = fr->rawFormat;   /* GuampCamIosRawFormat == gisp_raw_format */
    out->cfa = fr->bayer;          /* GuampCamIosBayer == gisp_cfa */
    out->data = slot;
    return 0;
}

gisp_rect gcp_crop(const GuampCamIosFrame *fr, uint32_t ow, uint32_t oh, float zoom) {
    double w = fr->width, h = fr->height;
    if (ow && oh) { if (w * oh > h * ow) w = h * ow / oh; else h = w * oh / ow; }
    if (zoom > 1.f) { w /= zoom; h /= zoom; }
    gisp_rect r = {.w = (uint32_t)w & ~1u, .h = (uint32_t)h & ~1u};
    r.x = ((fr->width - r.w) / 2) & ~1u;
    r.y = ((fr->height - r.h) / 2) & ~1u;
    return r;
}

int gcp_frame(gcp *p, const GuampCamIosFrame *fr, const gisp_raw *raw, const gcp_controls *c, gcp_meta *m) {
    gcp_controls def;
    if (!c) { gcp_controls_default(&def); c = &def; }
    memset(m, 0, sizeof(*m));
    m->sequence = fr->sequence;
    m->host_time = fr->timestamp;
    m->flags = fr->flags;
    m->exposure_us = fr->exposureUs;
    m->again = fr->analogGainQ8 / 256.f;
    m->dgain = fr->digitalGainQ8 ? fr->digitalGainQ8 / 256.f : 1.f;
    m->iso = 50.f * m->again * m->dgain;
    m->focus_dac = fr->focusDac;
    m->temperature_mc = fr->temperatureMilliC;
    m->chip_id = p->info.chipId;
    int rc = 0;
    if (!raw || !raw->data || (fr->flags & kGuampFrameCorrupt)) {
        rc = -1;
        goto out;
    }
    p->stats->zones_x = 16; p->stats->zones_y = 12; p->stats->step = 2;
    gisp_stats_compute(raw, NULL, &p->params, p->stats);
    g3a_stats v;
    g3a_from_gisp(&v, p->stats, &p->params);
    if (!c->awb_lock && raw->cfa != GISP_MONO) {
        if (g3a_awb_run(&v, &p->awbCfg, &p->awb, NULL) == 0) {
            memcpy(p->params.wb, p->awb.wb, sizeof(p->params.wb));
            if (p->tuning && p->awb.cct > 0) gisp_tuning_apply(p->tuning, p->awb.cct, &p->params);   /* CCM follows the CCT */
        }
    }
    /* AE waits while a request is pending: the frame still shows the old exposure (GuampCamIosAbi.h "Exposure"). */
    if (p->aeAllowed && c->ae_enabled && !c->ae_lock && !(fr->flags & kGuampFrameSettingsPending)) {
        g3a_ae_config ac = p->ae;
        ac.target = p->ae.target * powf(2.f, c->exposure_bias);
        g3a_ae_run(&v, &ac, p->params.wb, (uint64_t)fr->exposureUs * 1000u, m->again * m->dgain, &p->aeLast);
        const uint32_t us = (uint32_t)(p->aeLast.exposure_ns / 1000u), q8 = (uint32_t)(p->aeLast.gain * 256.f + 0.5f);
        if ((us != p->lastUs || q8 != p->lastQ8) && (us != fr->exposureUs || q8 != fr->analogGainQ8)) {
            if (p->ops.set_exposure) p->ops.set_exposure(p->ops.ctx, p->sensor, us, q8, 256);
            p->lastUs = us; p->lastQ8 = q8;
            m->requested_exposure_us = us;
            m->requested_gain_q8 = q8;
        }
    } else if (!p->aeAllowed) {
        float hl = 0;
        p->aeLast.luma = g3a_ae_measure(&v, &p->ae, p->params.wb, &hl);
        p->aeLast.target = p->ae.target;
    }
    if (p->info.hasAutoFocus) {
        p->afCfg.continuous = c->af_continuous;
        if (c->focus_request == -2) { g3a_af_start(&p->af, &p->afCfg); p->afActive = 1; }
        else if (c->focus_request >= 0) {
            if (p->ops.set_focus) p->ops.set_focus(p->ops.ctx, p->sensor, (uint32_t)c->focus_request);
            p->afActive = 0;
        }
        if (p->afActive && !c->af_lock) {
            const uint16_t dac = g3a_af_run(&v, &p->afCfg, &p->af);
            if (p->ops.set_focus) p->ops.set_focus(p->ops.ctx, p->sensor, dac);
            if (p->af.state == G3A_AF_FOCUSED || p->af.state == G3A_AF_FAILED) p->afActive = p->afCfg.continuous;
        }
        float luma = 0;
        m->focus_value = g3a_af_measure(&v, &p->afCfg, &luma);
    }
out:
    memcpy(m->wb, p->params.wb, sizeof(m->wb));
    m->cct = p->awb.cct;
    m->ae_target = p->aeLast.target;
    m->ae_luma = p->aeLast.luma;
    m->ae_converged = p->aeLast.converged;
    m->ae_limited = p->aeLast.limited;
    m->af_state = p->af.state;
    const double exposure = fr->exposureUs / 1e6;
    m->lux = exposure > 0 ? (float)(p->aeLast.luma * 250.0 / (exposure * m->again * m->dgain)) : 0.f;
    return rc;
}

int gcp_render_fast_prepare(gcp *p, const gisp_raw *raw, const gisp_rect *crop, uint32_t ow, uint32_t oh, int full_range) {
    const gisp_rect cr = crop ? *crop : (gisp_rect){0, 0, 0, 0};
    p->params.full_range = full_range ? 1u : 0u;
    const int same = p->fast && p->fastW == ow && p->fastH == oh && p->fastGeom.width == raw->width &&
                     p->fastGeom.height == raw->height && p->fastGeom.stride == raw->stride &&
                     p->fastGeom.format == raw->format && p->fastGeom.cfa == raw->cfa && !memcmp(&p->fastCrop, &cr, sizeof(cr)) &&
                     p->fastFull == p->params.full_range;
    if (same) { gisp_fast_set_params(p->fast, &p->params); return 0; }
    if (p->fast) gisp_fast_free(p->fast);
    p->fast = gisp_fast_new(raw, crop, ow, oh, GISP_FAST_AUTO, &p->params);
    p->fastGeom = *raw;
    p->fastCrop = cr;
    p->fastW = ow; p->fastH = oh; p->fastFull = p->params.full_range;
    return p->fast ? 0 : -1;
}

int gcp_render_fast_band(gcp *p, const gisp_raw *raw, gisp_nv12 *out, uint32_t band, uint32_t nbands) {
    return p->fast ? gisp_fast_band(p->fast, raw, out, band, nbands) : -1;
}

int gcp_render_hq(gcp *p, const gisp_raw *raw, const gisp_rect *crop, gisp_nv12 *out, int full_range) {
    p->params.full_range = full_range ? 1u : 0u;
    return gisp_process(p->hq, raw, crop, &p->params, out);
}
