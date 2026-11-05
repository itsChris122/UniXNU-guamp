/* SPDX-License-Identifier: BSD-2-Clause
 * guamp 3A still capture with pre-flash metering (see g3a_still.h). Own design, no vendor code. */
#include "g3a_still.h"
#include <math.h>
#include <string.h>

#define G3A_PRE_RETRIES 3                /* pre-flash frames re-taken at a lower exposure when clipped */
#define G3A_STILL_TIMEOUT 40             /* frames */
#define G3A_PRE_MAX_MS 1000u             /* GuampTorchCore.h kPreflashMaxMs */
#define G3A_MAIN_MAX_MS 500u             /* GuampTorchCore.h kFlashMaxMs */
#define G3A_FLASH_MARGIN_NS 40000000ull  /* PMIC thread wake-up + VREG_OK + LED rise, with room to spare */

/* Frames from a flash request to the first fully lit frame: the next frame is lit only in part (rolling shutter). */
static uint32_t lit_latency(const g3a_still *s) { return s->cfg.exposure_latency < 2 ? 2 : s->cfg.exposure_latency; }

void g3a_still_init(g3a_still *s, const g3a_still_config *c, const g3a_ae_config *ae) {
    memset(s, 0, sizeof(*s));
    s->cfg = *c;
    if (!s->cfg.frame_ns) s->cfg.frame_ns = 33333333;
    if (ae) s->ae = *ae;
    else g3a_ae_default(&s->ae);
    g3a_flash_default(&s->fl_cfg);
    /* the photo must fit in the main flash window: (lit latency + 1) frames of max(frame, exposure) + margin */
    double tcap = ((double)G3A_MAIN_MAX_MS * 1e6 - (double)G3A_FLASH_MARGIN_NS) / (double)(lit_latency(s) + 1);
    if (tcap > (double)s->ae.t_max_ns) tcap = (double)s->ae.t_max_ns;
    s->fl_cfg.t_capture_max_ns = (uint64_t)tcap;
    s->wb_amb[0] = s->wb_amb[1] = s->wb_amb[2] = 1.f;
}

int g3a_still_begin(g3a_still *s, int flash_mode, const g3a_ae_result *ae_last, const float wb[3]) {
    if (s->state != G3A_STILL_IDLE) return -1;
    const int flash = s->cfg.has_flash &&
                      (flash_mode == G3A_FLASH_ON || (flash_mode == G3A_FLASH_AUTO && ae_last && g3a_flash_needed(&s->fl_cfg, ae_last)));
    s->state = flash ? G3A_STILL_AMBIENT : G3A_STILL_PLAIN;
    s->retries = 0;
    s->deadline = 0;                     /* set on the first frame (sequence numbers come with frames) */
    s->amb_ns = 0;
    if (wb) memcpy(s->wb_amb, wb, sizeof(s->wb_amb));
    return 0;
}

static void command(g3a_still *s, uint64_t seq, uint64_t ns, float gain, g3a_still_out *o) {
    o->set_exposure = 1; o->exposure_ns = ns; o->gain = gain;
    s->have_cmd = 1; s->cmd_seq = seq;
}

static void command_total(g3a_still *s, uint64_t seq, double total, g3a_still_out *o) {
    g3a_ae_config c = s->ae;
    c.flicker_hz = 0;                    /* short pre-flash frames: no mains banding to avoid, keep the exposure exact */
    uint64_t ns; float gain;
    g3a_ae_split(&c, total, &ns, &gain);
    command(s, seq, ns, gain, o);
}

static void flash(g3a_still_out *o, int kind, uint32_t ma, uint32_t ms) { o->flash = kind; o->flash_ma = ma; o->flash_ms = ms; }

static uint32_t window_ms(uint32_t frames, uint64_t frame_ns, uint64_t exposure_ns, uint32_t max_ms) {
    const uint64_t period = exposure_ns + 1000000u > frame_ns ? exposure_ns + 1000000u : frame_ns;
    const uint64_t ms = ((uint64_t)frames * period + G3A_FLASH_MARGIN_NS + 999999u) / 1000000u;
    return ms < 20u ? 20u : ms > max_ms ? max_ms : (uint32_t)ms;
}

static void deliver(g3a_still *s, g3a_still_out *o, int with_flash) {
    o->capture = 1;
    o->capture_flash = with_flash;
    s->stills++;
    if (with_flash) s->flash_stills++;
    s->state = G3A_STILL_IDLE;
}

void g3a_still_frame(g3a_still *s, const g3a_stats *st, const g3a_frame_info *fi, g3a_still_out *o) {
    memset(o, 0, sizeof(*o));
    o->flash = G3A_FLASH_CMD_NONE;
    memcpy(o->wb, s->wb_amb, sizeof(o->wb));
    if (s->state == G3A_STILL_IDLE) return;
    o->busy = 1;
    const uint64_t seq = fi->sequence;
    const uint64_t cur_ns = fi->exposure_ns ? fi->exposure_ns : 10000000u;
    const float cur_gain = fi->gain >= 1.f ? fi->gain : 1.f;
    const double exposure = (double)cur_ns * cur_gain;
    const int pending = fi->pending || (s->have_cmd && seq < s->cmd_seq + s->cfg.exposure_latency);

    if (!s->deadline) s->deadline = seq + G3A_STILL_TIMEOUT;
    if (seq >= s->deadline) {            /* stuck (flash refused, frames without the new exposure ...): take it as it is */
        if (s->state != G3A_STILL_PLAIN) {
            flash(o, G3A_FLASH_CMD_OFF, 0, 0);
            if (s->amb_ns) command(s, seq, s->amb_ns, s->amb_gain, o);
        }
        s->aborts++;
        deliver(s, o, 0);
        return;
    }

    switch (s->state) {
    case G3A_STILL_PLAIN:
        if (!pending) deliver(s, o, 0);
        break;
    case G3A_STILL_AMBIENT:
        if (pending) break;
        g3a_flash_ambient(st, exposure, &s->amb);
        s->amb_ns = cur_ns;
        s->amb_gain = cur_gain;
        command_total(s, seq, g3a_flash_pre_exposure(&s->fl_cfg, &s->ae, exposure), o);
        flash(o, G3A_FLASH_CMD_PRE, s->fl_cfg.pre_ma,
              window_ms(lit_latency(s) + G3A_PRE_RETRIES * lit_latency(s) + 2, s->cfg.frame_ns, cur_ns, G3A_PRE_MAX_MS));
        s->cap_at = seq + lit_latency(s);
        s->state = G3A_STILL_PRE;
        break;
    case G3A_STILL_PRE: {
        if (pending || seq < s->cap_at) break;
        g3a_flash_config fc = s->fl_cfg;
        fc.target = s->fl_cfg.target * exp2f(s->bias);
        g3a_flash_plan(st, exposure, &s->amb, &fc, &s->ae, s->wb_amb, &s->plan);
        if (s->plan.ok == -1 && s->retries < G3A_PRE_RETRIES) {
            s->retries++;
            command_total(s, seq, s->plan.retry_exposure, o);
            s->cap_at = seq + 1;         /* the flash is fully on already: only the exposure has to land (pending) */
            break;
        }
        if (s->plan.ok == 1) {
            command(s, seq, s->plan.exposure_ns, s->plan.gain, o);
            flash(o, G3A_FLASH_CMD_MAIN, s->plan.ma,
                  window_ms(lit_latency(s) + 1, s->cfg.frame_ns, s->plan.exposure_ns, G3A_MAIN_MAX_MS));
            s->cap_at = seq + lit_latency(s);
            s->state = G3A_STILL_MAIN;
            break;
        }
        /* the pre-flash did not reach the subject (or stayed clipped): no flash, back to the ambient exposure */
        s->no_flash_light++;
        flash(o, G3A_FLASH_CMD_OFF, 0, 0);
        command(s, seq, s->amb_ns, s->amb_gain, o);
        s->state = G3A_STILL_PLAIN;
        break;
    }
    case G3A_STILL_MAIN:
        if (pending || seq < s->cap_at) break;
        memcpy(o->wb, s->plan.wb, sizeof(o->wb));
        flash(o, G3A_FLASH_CMD_OFF, 0, 0);
        command(s, seq, s->amb_ns, s->amb_gain, o);   /* preview resumes at the ambient exposure */
        deliver(s, o, 1);
        break;
    default:
        s->state = G3A_STILL_IDLE;
        break;
    }
}
