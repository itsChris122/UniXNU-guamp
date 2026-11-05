/* SPDX-License-Identifier: BSD-2-Clause
 * guamp 3A: auto exposure, auto white balance, contrast autofocus and flash metering for the Moto G9 Play cameras.
 * User space, plain C11 + libm, no allocation, no threads, no vendor data. One call per frame with the statistics of
 * guamp-isp (gisp_stats: per-zone sums, saturation, sharpness, luma histogram; see g3a_gisp.h for the adapter).
 * Outputs go to the kernel (GuampCamera3A.inc / GuampTorch.inc):
 *   exposure time ns + analog gain   -> camera::af::exposure() (sensor lines/FLL/gain code inside the grouped hold)
 *   lens DAC (10 bit)                -> camera::af::lensSet()  (DW9767 main / GT9772 macro, range from lensOpen)
 *   flash kind / mA / ms             -> GuampTorch property {FlashKind, FlashMA, FlashMS}
 *   WB gains                         -> gisp_params.wb of the ISP
 * Host-tested by tests/g3a_test.c against a simulated sensor, lens and flash.
 */
#ifndef G3A_H
#define G3A_H
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- statistics (a view, nothing is copied) ------------------------------------------------------------------------
 * Per zone, row by row: sums of R, G, B (G = mean of the two greens) over the counted 2x2 quads, black removed, before
 * WB, in sensor units; count = quads summed (unsaturated); sat = quads with a saturated channel (not in the sums);
 * sharp = contrast measure of the zone (gisp: sum of the second difference |G - 2 G1 + G2| between sampled quads;
 * a first difference sum |dG| does not fall with defocus on plain edges, see tests/g3a_gisp_test.c). hist: 256 bins of linear luma (WB applied) 0..white, may be NULL.
 * white = saturation level in the units of the sums (gisp: white - black). */
typedef struct {
    uint32_t zones_x, zones_y;
    const float *sum_r, *sum_g, *sum_b;
    const uint32_t *count, *sat;
    const float *sharp;      /* may be NULL (no AF) */
    const uint32_t *hist;    /* may be NULL */
    float white;
} g3a_stats;

#define G3A_ZONES_MAX 1024

/* ---- auto exposure ----------------------------------------------------------------------------------------------- */
typedef struct {
    float target;            /* weighted mean linear luma wanted, 0..1 of white (0.18 = middle grey) */
    float center_weight;     /* extra weight of the centre (0 = flat average) */
    float highlight_frac;    /* fraction of the histogram allowed in the top 3 % before the target is lowered */
    float min_target_scale;  /* lowest target scale for highlight protection (0.5) */
    uint64_t t_min_ns, t_max_ns; /* exposure time limits (t_max = frame period for a fixed frame rate) */
    float gain_max;          /* analog gain limit (16 main/front, 15.5 OV02B) */
    uint32_t flicker_hz;     /* 50, 60 or 0: times >= half a mains period become multiples of it */
    float max_step;          /* largest exposure change per frame (factor, 4) */
    float damping;           /* 0..1, fraction of the log error corrected per frame near the target (0.6) */
    float tolerance;         /* |log2 error| below which AE counts as converged (0.1) */
} g3a_ae_config;

typedef struct {
    uint64_t exposure_ns;    /* time to program */
    float gain;              /* analog gain to program (1..gain_max) */
    float luma;              /* measured weighted luma of the frame (0..1) */
    float target;            /* target after highlight protection */
    int converged;           /* |log2(target / luma)| < tolerance */
    int limited;             /* 1: wanted more light than t_max x gain_max (dark: flash candidate); -1: less than t_min */
} g3a_ae_result;

void g3a_ae_default(g3a_ae_config *c);
/* Weighted luma of one frame (WB gains applied, 0..1) and the fraction of quads in the top 3 % of the histogram. */
float g3a_ae_measure(const g3a_stats *s, const g3a_ae_config *c, const float wb[3], float *highlights);
/* applied_ns / applied_gain: what the sensor really used for this frame (kernel readback). */
void g3a_ae_run(const g3a_stats *s, const g3a_ae_config *c, const float wb[3], uint64_t applied_ns, float applied_gain,
                g3a_ae_result *out);
/* Split a total exposure (ns x gain) into time and gain: time first (flicker multiples), gain for the rest. */
void g3a_ae_split(const g3a_ae_config *c, double total, uint64_t *ns, float *gain);

/* ---- auto white balance -------------------------------------------------------------------------------------------- */
#define G3A_AWB_REF_MAX 16
typedef struct {
    float gain_min, gain_max;   /* limits of the R and B gains (G = 1) */
    float grey_sigma;           /* log-chroma width of "near grey" zones (0.35); 0 = plain grey world */
    float dark, bright;         /* zones with G mean below dark x white or any channel above bright x white are skipped */
    float speed;                /* 0..1 IIR speed in the log domain (0.25) */
    uint32_t n_ref;             /* optional white locus: R/G, B/G of white under known lights (gisp_tuning ref_*) */
    float ref_rg[G3A_AWB_REF_MAX], ref_bg[G3A_AWB_REF_MAX], ref_cct[G3A_AWB_REF_MAX];
    float ref_max_dist;         /* how far (log units) the estimate may stay off the locus (0.08) */
} g3a_awb_config;

typedef struct {
    float wb[3];                /* current smoothed gains (state, start with 1 or a guess) */
    float cct;                  /* estimated colour temperature (0 = unknown, needs the locus) */
    int converged;
    uint32_t zones_used;
} g3a_awb_state;

void g3a_awb_default(g3a_awb_config *c);
void g3a_awb_reset(g3a_awb_state *st);
/* One frame: raw gains of this frame in raw[3] (may be NULL); returns 0, or -1 when no zone is usable (gains kept). */
int g3a_awb_run(const g3a_stats *s, const g3a_awb_config *c, g3a_awb_state *st, float raw[3]);

/* ---- contrast autofocus ------------------------------------------------------------------------------------------- */
/* enum tag g3a_af_status (not g3a_af_state): in C++ an enum and a struct of the same name clash. */
enum g3a_af_status { G3A_AF_IDLE = 0, G3A_AF_COARSE, G3A_AF_FINE, G3A_AF_FOCUSED, G3A_AF_FAILED };
typedef struct {
    uint16_t dac_lo, dac_hi;    /* lens range from the kernel (calibrated infinity .. macro) */
    uint16_t coarse_steps;      /* positions of the coarse scan (12) */
    uint16_t fine_div;          /* fine step = coarse step / fine_div (4) */
    uint16_t settle_frames;     /* frames skipped after each move (1: the frame being exposed during the move) */
    float min_peak_ratio;       /* best / worst focus value needed to call it focused (1.15) */
    float drop_stop;            /* coarse scan stops after two values below drop_stop x best past the peak (0.75) */
    float hyperfocal;           /* fallback position in the range, 0 = infinity end (0.15) */
    uint32_t roi_x, roi_y, roi_w, roi_h; /* zones used; roi_w = 0: the centre third */
    int continuous;             /* re-scan when the scene changes */
    float rescan_drop;          /* continuous: focus value below rescan_drop x locked value ... */
    uint16_t rescan_frames;     /* ... for this many frames (5) */
} g3a_af_config;

typedef struct {
    int state;
    uint16_t dac;               /* position asked last (apply it) */
    uint16_t best_dac, step;
    float best_fv, worst_fv, lock_fv, lock_luma; /* lock_luma: during a scan, the luma it started with */
    uint16_t settle, below, drift;
    uint16_t pos[64]; float fv[64]; uint16_t n;   /* samples of the current pass */
    uint16_t fine_end;
    uint32_t frames, moves;
} g3a_af_state;

void g3a_af_default(g3a_af_config *c, uint16_t dac_lo, uint16_t dac_hi);
/* Focus value of the ROI: sum(sharp) / sum(G), independent of exposure to first order. */
float g3a_af_measure(const g3a_stats *s, const g3a_af_config *c, float *luma);
void g3a_af_start(g3a_af_state *st, const g3a_af_config *c);
/* One frame (stats taken with the lens at the position asked before): returns the DAC to program now.
 * A scan whose ROI luma moves by more than x2 (1 EV) from where it started (AE still converging, light changed) starts
 * again: focus values of frames at different exposure are not comparable (read noise in the contrast term).
 * Callers should also skip frames whose exposure is still pending (kGuampFrameSettingsPending). */
uint16_t g3a_af_run(const g3a_stats *s, const g3a_af_config *c, g3a_af_state *st);

/* ---- flash ---------------------------------------------------------------------------------------------------------
 * Sequence driven by the camera service:
 *   1. g3a_flash_needed() on the preview AE result (or flash forced on);
 *   2. freeze AE, keep the ambient stats of one frame: g3a_flash_ambient();
 *   3. program the pre-flash exposure (g3a_flash_pre_exposure), pre-flash (FlashKind 1, pre_ma, about 300 ms), skip
 *      the frames exposed partly lit, take one fully lit frame: g3a_flash_plan() -> main current, exposure, gain, WB;
 *   4. main flash (FlashKind 2, plan.ma, plan.ms) and the capture frame inside the flash window (FlashOnTime). */
typedef struct {
    uint32_t pre_ma, main_ma, main_ms, min_ma; /* pre-flash 150, main 500 (kernel cap 750), 200 ms, 25 */
    float pre_exposure_div;     /* pre-flash frame exposure = ambient exposure / this (8): keeps the lit frame unclipped */
    uint64_t t_capture_max_ns;  /* longest exposure inside the flash window (e.g. main_ms minus two frame readouts) */
    float target;               /* luma wanted with the flash (0.18) */
    float highlight;            /* brightest zone allowed after the flash (0.9) */
    float need_luma;            /* auto flash when AE is limited and the luma is below need_luma x target (0.6) */
} g3a_flash_config;

typedef struct {
    float y[G3A_ZONES_MAX], r[G3A_ZONES_MAX], g[G3A_ZONES_MAX], b[G3A_ZONES_MAX]; /* ambient per zone, per unit exposure */
    uint32_t zones;
    double exposure;            /* ns x gain of the ambient frame */
    int valid;
} g3a_flash_ambient_t;

typedef struct {
    uint32_t ma, ms;            /* main flash */
    uint64_t exposure_ns; float gain;
    float wb[3];                /* WB gains for the flash frame (ambient and flash light mixed) */
    float flash_wb[3];          /* gains of the flash light alone (measured) */
    float expected_luma, flash_fraction;
    int ok;                     /* 1 plan valid; 0: the pre-flash added no light (subject too far): capture without
                                   flash; -1: lit frame too clipped: take another pre-flash frame at retry_exposure */
    double retry_exposure;
} g3a_flash_plan_t;

void g3a_flash_default(g3a_flash_config *c);
int g3a_flash_needed(const g3a_flash_config *c, const g3a_ae_result *ae);
void g3a_flash_ambient(const g3a_stats *s, double exposure, g3a_flash_ambient_t *amb);
/* Exposure (ns x gain) for the pre-flash frame. */
double g3a_flash_pre_exposure(const g3a_flash_config *c, const g3a_ae_config *ae, double ambient_exposure);
/* lit = pre-flash frame taken at lit_exposure; ae = AE config (weights, limits); wb_ambient = AWB of the scene. */
void g3a_flash_plan(const g3a_stats *lit, double lit_exposure, const g3a_flash_ambient_t *amb, const g3a_flash_config *c,
                    const g3a_ae_config *ae, const float wb_ambient[3], g3a_flash_plan_t *out);

#ifdef __cplusplus
}
#endif
#endif
