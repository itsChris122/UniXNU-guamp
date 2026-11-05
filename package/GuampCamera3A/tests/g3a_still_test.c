/* Host test of g3a_still.c against a simulated camera that behaves like the kernel service (AppleH6CamIn, ABI v2):
 * exposure applied `latency` frames after SetExposure, frames flagged pending until then (or not flagged, for the
 * fallback), time quantised to 10 us lines and gain to 1/32 codes, flash LED lit from the request for its length with a
 * rolling shutter (top and bottom rows see different parts of the flash). The per-frame loop around it is the one of
 * gcampipe: AWB, then AE on frames that are not pending, both frozen while the still sequencer is busy. */
#include "../g3a_still.h"
#include "g3a_sim.h"
#include <stdio.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

typedef struct {
    scene *sc;
    uint32_t latency; int flags_pending;
    uint64_t seq, frame_ns, readout_ns;
    double t_end;
    uint64_t q_at, q_ns; float q_gain; int q_valid;
    uint64_t act_ns; float act_gain;
    int fl_kind; uint32_t fl_ma; double fl_on, fl_off;
    unsigned pre_cmds, main_cmds, off_cmds, exp_cmds;
    uint32_t max_pre_ms, max_main_ms;
    float lit_frac;
} cam;

static uint64_t q_ns(uint64_t ns) { uint64_t l = (ns + 5000) / 10000; return (l ? l : 1) * 10000; }
static float q_gain(float g) { float c = floorf(g * 32.f + 0.5f) / 32.f; return c < 1.f ? 1.f : c; }

static void cam_init(cam *c, scene *sc, uint32_t latency, int flags) {
    memset(c, 0, sizeof(*c));
    c->sc = sc; c->latency = latency; c->flags_pending = flags;
    c->frame_ns = 33333333; c->readout_ns = 25000000;
    c->act_ns = 10000000; c->act_gain = 1.f;
}

static double overlap(const cam *c, double a, double b) {
    if (!c->fl_kind || b <= a) return 0.0;
    const double lo = a > c->fl_on ? a : c->fl_on, hi = b < c->fl_off ? b : c->fl_off;
    return hi > lo ? (hi - lo) / (b - a) : 0.0;
}

static void cam_frame(cam *c, frame *f, g3a_frame_info *fi) {
    c->seq++;
    if (c->q_valid && c->seq >= c->q_at) { c->act_ns = c->q_ns; c->act_gain = c->q_gain; c->q_valid = 0; }
    const double period = (double)(c->act_ns + 1000000 > c->frame_ns ? c->act_ns + 1000000 : c->frame_ns);
    c->t_end += period;
    const double e = (double)c->act_ns;
    const double top = overlap(c, c->t_end - c->readout_ns - e, c->t_end - c->readout_ns), bot = overlap(c, c->t_end - e, c->t_end);
    c->lit_frac = (float)(0.5 * (top + bot));
    render(c->sc, (double)c->act_ns * c->act_gain, 437, c->lit_frac * (float)c->fl_ma, NULL, f);
    fi->sequence = c->seq;
    fi->exposure_ns = c->act_ns;         /* ABI v2: the applied values of this frame */
    fi->gain = c->act_gain;
    fi->pending = c->flags_pending && c->q_valid;
}

static void cam_exposure(cam *c, uint64_t ns, float gain) {
    c->exp_cmds++;
    c->q_ns = q_ns(ns); c->q_gain = q_gain(gain); c->q_at = c->seq + c->latency; c->q_valid = 1;
}

static void cam_flash(cam *c, int kind, uint32_t ma, uint32_t ms) {
    const double now = c->t_end + 5e6;   /* the request reaches the PMIC thread a few ms after the frame */
    if (kind == G3A_FLASH_CMD_OFF) { c->off_cmds++; if (c->fl_kind && now < c->fl_off) c->fl_off = now; }
    if (kind == G3A_FLASH_CMD_PRE || kind == G3A_FLASH_CMD_MAIN) {
        if (kind == G3A_FLASH_CMD_PRE) { c->pre_cmds++; if (ms > c->max_pre_ms) c->max_pre_ms = ms; }
        else { c->main_cmds++; if (ms > c->max_main_ms) c->max_main_ms = ms; }
        c->fl_kind = kind; c->fl_ma = ma;
        c->fl_on = now + 3e6;            /* VREG_OK + LED rise */
        c->fl_off = c->fl_on + (double)ms * 1e6;
    }
}

/* the per-frame loop of gcampipe, reduced to the 3A */
typedef struct {
    g3a_ae_config ae; g3a_ae_result ae_last;
    g3a_awb_config awb_cfg; g3a_awb_state awb;
    g3a_still still;
    uint64_t last_cmd;
} pipe;

static frame F;

static void pipe_init(pipe *p, int has_flash, uint32_t latency_fallback) {
    memset(p, 0, sizeof(*p));
    g3a_ae_default(&p->ae);
    p->ae.t_max_ns = 66666666;
    g3a_awb_default(&p->awb_cfg);
    g3a_awb_reset(&p->awb);
    const g3a_still_config sc = {has_flash, latency_fallback, 33333333};
    g3a_still_init(&p->still, &sc, &p->ae);
}

/* one frame; returns 1 when it is the photo (o filled) */
static int pipe_frame(pipe *p, cam *c, uint32_t latency_fallback, g3a_still_out *o) {
    g3a_frame_info fi;
    cam_frame(c, &F, &fi);
    g3a_still_frame(&p->still, &F.v, &fi, o);
    if (!o->busy && !o->capture) {
        g3a_awb_run(&F.v, &p->awb_cfg, &p->awb, NULL);
        uint64_t *last_cmd = &p->last_cmd;
        const int pend = fi.pending || (latency_fallback && *last_cmd && fi.sequence < *last_cmd + latency_fallback);
        if (!pend) {
            g3a_ae_run(&F.v, &p->ae, p->awb.wb, fi.exposure_ns, fi.gain, &p->ae_last);
            if (q_ns(p->ae_last.exposure_ns) != fi.exposure_ns || q_gain(p->ae_last.gain) != fi.gain) {
                cam_exposure(c, p->ae_last.exposure_ns, p->ae_last.gain);
                *last_cmd = fi.sequence;
            }
        }
    }
    if (o->set_exposure) { cam_exposure(c, o->exposure_ns, o->gain); p->last_cmd = c->seq; }   /* the still restores the exposure too */
    if (o->flash != G3A_FLASH_CMD_NONE) cam_flash(c, o->flash, o->flash_ma, o->flash_ms);
    return o->capture;
}

static void preview(pipe *p, cam *c, uint32_t lf, int frames) {
    for (int k = 0; k < frames; ++k) {
        g3a_still_out o;
        CHECK(!pipe_frame(p, c, lf, &o) && o.flash == G3A_FLASH_CMD_NONE);
    }
}

static int shoot(pipe *p, cam *c, uint32_t lf, int mode, g3a_still_out *o, float *luma) {
    CHECK(g3a_still_begin(&p->still, mode, &p->ae_last, p->awb.wb) == 0);
    CHECK(g3a_still_begin(&p->still, mode, &p->ae_last, p->awb.wb) == -1);   /* one at a time */
    for (int k = 0; k < 60; ++k)
        if (pipe_frame(p, c, lf, o)) {
            float hl;
            *luma = g3a_ae_measure(&F.v, &p->ae, o->capture_flash ? o->wb : p->awb.wb, &hl);
            return k;
        }
    return -1;
}

static void dark(scene *sc) {
    scene_grey(sc, 2e-11f);
    sc->illum[0] = 1.f; sc->illum[1] = 0.7f; sc->illum[2] = 0.4f;
    sc->flash_k = 2e-11f;
}

int main(void) {
    static pipe p;
    g3a_still_out o; float y = 0.f;
    for (int variant = 0; variant < 2; ++variant) {
        const int flags = variant == 0;           /* 0: kernel flags pending frames; 1: no flag, 2-frame fallback */
        const uint32_t lf = flags ? 0 : 2;
        printf("-- %s\n", flags ? "kernel pending flag" : "no flag, exposure_latency 2");
        /* 1. bright room, flash auto: plain photo, no flash */
        {
            scene sc; scene_grey(&sc, 3e-8f);
            cam c; cam_init(&c, &sc, 2, flags);
            pipe_init(&p, 1, lf);
            preview(&p, &c, lf, 40);
            const int k = shoot(&p, &c, lf, G3A_FLASH_AUTO, &o, &y);
            printf("bright auto: after %d frames, flash %d, luma %.3f\n", k, o.capture_flash, (double)y);
            CHECK(k >= 0 && k <= 3 && !o.capture_flash && c.pre_cmds == 0 && c.main_cmds == 0);
            CHECK(fabsf(y - 0.18f) / 0.18f < 0.2f);
        }
        /* 2. dark room, flash auto: pre-flash, main flash, photo lit over its whole exposure at the target */
        {
            scene sc; dark(&sc);
            cam c; cam_init(&c, &sc, 2, flags);
            pipe_init(&p, 1, lf);
            preview(&p, &c, lf, 40);
            CHECK(p.ae_last.limited == 1);
            const uint64_t amb_ns = c.act_ns; const float amb_gain = c.act_gain;
            const int k = shoot(&p, &c, lf, G3A_FLASH_AUTO, &o, &y);
            printf("dark auto: after %d frames, flash %d, lit %.2f, %u mA, pre %u ms main %u ms, luma %.3f, %llu ns x%.2f\n", k,
                   o.capture_flash, (double)c.lit_frac, p.still.plan.ma, c.max_pre_ms, c.max_main_ms, (double)y,
                   (unsigned long long)c.act_ns, (double)c.act_gain);
            CHECK(k > 0 && k < 20 && o.capture_flash && c.pre_cmds == 1 && c.main_cmds == 1 && c.off_cmds == 1);
            CHECK(c.lit_frac > 0.999f);
            CHECK(fabsf(y - 0.18f) / 0.18f < 0.2f);
            CHECK(c.max_main_ms <= 500 && c.max_pre_ms <= 1000 && p.still.plan.ma <= 500);
            preview(&p, &c, lf, 4);                 /* ambient exposure back, sequencer idle */
            CHECK(c.act_ns == q_ns(amb_ns) && c.act_gain == q_gain(amb_gain) && p.still.state == G3A_STILL_IDLE);
            preview(&p, &c, lf, 30);                /* and the LED is off for the preview */
            CHECK(c.lit_frac == 0.f);
        }
        /* 3. subject out of reach, flash on: no flash light, photo at the ambient exposure */
        {
            scene sc; dark(&sc);
            for (int i = 0; i < NZ; ++i) sc.depth[i] = 100.f;
            cam c; cam_init(&c, &sc, 2, flags);
            pipe_init(&p, 1, lf);
            preview(&p, &c, lf, 40);
            const uint64_t amb_ns = c.act_ns;
            const int k = shoot(&p, &c, lf, G3A_FLASH_ON, &o, &y);
            printf("far subject: after %d frames, flash %d\n", k, o.capture_flash);
            CHECK(k > 0 && !o.capture_flash && p.still.no_flash_light == 1 && c.main_cmds == 0 && c.off_cmds == 1);
            CHECK(c.act_ns == q_ns(amb_ns) && c.lit_frac == 0.f);
        }
        /* 4. very close subject: pre-flash clipped, retried lower, less current */
        {
            scene sc; dark(&sc);
            for (int i = 0; i < NZ; ++i) sc.depth[i] = 0.01f;
            cam c; cam_init(&c, &sc, 2, flags);
            pipe_init(&p, 1, lf);
            preview(&p, &c, lf, 40);
            const int k = shoot(&p, &c, lf, G3A_FLASH_ON, &o, &y);
            printf("close subject: after %d frames, flash %d, retries %d, %u mA, luma %.3f\n", k, o.capture_flash, p.still.retries,
                   p.still.plan.ma, (double)y);
            CHECK(k > 0 && o.capture_flash && p.still.retries > 0 && p.still.plan.ma < 500 && fabsf(y - 0.18f) / 0.18f < 0.3f);
        }
        /* 5. front camera (no flash), flash on: plain photo */
        {
            scene sc; dark(&sc);
            cam c; cam_init(&c, &sc, 2, flags);
            pipe_init(&p, 0, lf);
            preview(&p, &c, lf, 30);
            const int k = shoot(&p, &c, lf, G3A_FLASH_ON, &o, &y);
            CHECK(k >= 0 && k <= 3 && !o.capture_flash && c.pre_cmds == 0);
        }
    }
    /* 6. stuck: frames never leave pending (exposure never applied): taken after the timeout, flash off */
    {
        scene sc; dark(&sc);
        cam c; cam_init(&c, &sc, 1000, 1);
        pipe_init(&p, 1, 0);
        const int k = shoot(&p, &c, 0, G3A_FLASH_ON, &o, &y);
        printf("stuck: photo after %d frames, aborts %u\n", k, p.still.aborts);
        CHECK(k >= 35 && k <= 45 && p.still.aborts == 1 && c.off_cmds == 1);
    }
    if (fails) { printf("g3a_still_test: %d FAILED\n", fails); return 1; }
    printf("g3a_still_test: all checks passed\n");
    return 0;
}
