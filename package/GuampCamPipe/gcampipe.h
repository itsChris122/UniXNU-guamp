/* SPDX-License-Identifier: BSD-2-Clause
 * gcampipe - the per-frame core of the guamp camera service, without CoreFoundation/CoreMedia (agent 6, integration).
 *
 *   kernel (GuampCamIos.inc, ABI GuampCamIosAbi.h v2)   RAW slot + GuampCamIosFrame (applied exposure, geometry, flags)
 *     -> gcp_frame()        gisp statistics (agent 3) -> g3a AE / AWB / AF (agent 4) -> SetExposure / SetFocus via gcp_ops
 *     -> gcp_render_*()     gisp fast (preview, bands) or HQ (still) -> NV12 planes the caller owns (IOSurface, malloc)
 *     -> gcp_meta           per-frame metadata, plain C (the plug-in turns it into the MetadataDictionary attachment)
 *
 * Users: the H6ISP.mediacapture plug-in (package/GuampISP, agent 5: GuampFrames.c is expected to call this instead of its
 * own run3A/renderInto), the video/save path (agent 8), the secondary cameras (agent 7) and the host skeleton test
 * drivers/NativeEmmc/tests/campipe_host.cpp (TPG -> RAW10 -> camios user client -> gcampipe -> NV12, synthetic frames).
 * No threads inside: the caller runs gcp_render_fast_band on its own workers (dispatch_apply, pthreads). Not thread safe
 * across calls on the same gcp except gcp_render_fast_band for different bands of one frame.
 */
#ifndef GCAMPIPE_H
#define GCAMPIPE_H
#include <stddef.h>
#include <stdint.h>
#include "gisp.h"
#include "../../drivers/NativeEmmc/GuampCamIosAbi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GCP_VERSION 1

/* How the pipe reaches the kernel. The plug-in implements these with IOConnectCallScalarMethod (kGuampCamSelSetExposure,
 * kGuampCamSelSetFocus), the host test calls the user client directly. Return 0 on success. */
typedef struct {
    void *ctx;
    int (*set_exposure)(void *ctx, uint32_t sensor, uint32_t exposure_us, uint32_t again_q8, uint32_t dgain_q8);
    int (*set_focus)(void *ctx, uint32_t sensor, uint32_t dac);
} gcp_ops;

/* Requests from the client side (FigCaptureStream property setters, app controls), read once per frame. */
typedef struct {
    int ae_enabled;          /* 0: exposure stays where it is (manual exposure) */
    int ae_lock, awb_lock, af_lock;
    float exposure_bias;     /* EV */
    int focus_request;       /* -1 none, -2 one-shot AF scan, >= 0 manual DAC */
    int af_continuous;
} gcp_controls;

/* Per-frame metadata (units chosen for kFigCaptureStreamMetadata_*: see GuampFrames.c GFramesCopyMetadata). */
typedef struct {
    uint64_t sequence, host_time;   /* GuampCamIosFrame.sequence / timestamp (mach units) */
    uint32_t flags;                 /* GuampCamIosFrameFlags of the frame */
    uint32_t exposure_us;           /* applied for this frame */
    float again, dgain, iso;
    float wb[3];                    /* gains used by the ISP for this frame */
    float cct;                      /* AWB estimate, 0 = unknown */
    float ae_target, ae_luma;
    int ae_converged, ae_limited;   /* g3a_ae_result */
    int af_state;                   /* enum g3a_af_state */
    uint32_t focus_dac;             /* applied lens position of this frame */
    float focus_value;              /* g3a_af_measure of the ROI */
    float lux;                      /* rough scene estimate */
    int32_t temperature_mc;
    uint32_t chip_id;
    uint32_t requested_exposure_us, requested_gain_q8;   /* what this frame asked for (0 = nothing new) */
} gcp_meta;

/* gcp_meta.af_state: same values as enum g3a_af_state (g3a.h cannot be included from C++). */
enum gcp_af_state { GCP_AF_IDLE = 0, GCP_AF_COARSE = 1, GCP_AF_FINE = 2, GCP_AF_FOCUSED = 3, GCP_AF_FAILED = 4 };

typedef struct gcp gcp;

/* sensor/mode: GetInfo and SetMode answers. min_fps bounds the AE time (0 = from mode.minFpsQ8). tuning: a .gisp file
 * (guamp-isp tools/chromatix.py, vendor data, not in the repo) or NULL for the defaults. */
gcp *gcp_new(uint32_t sensor, const GuampCamIosSensorInfo *info, const GuampCamIosModeInfo *mode, float min_fps,
             const gcp_ops *ops, const void *tuning, size_t tuning_size);
void gcp_free(gcp *p);

/* Wrap one delivered slot as a gisp_raw, checking the frame geometry against the slot size. 0 ok, -1 unusable. */
int gcp_raw(const GuampCamIosFrame *fr, const uint8_t *slot, size_t slot_size, gisp_raw *out);

/* Statistics and 3A for one frame; sends the exposure/focus requests through ops. Corrupt frames only fill meta. */
int gcp_frame(gcp *p, const GuampCamIosFrame *fr, const gisp_raw *raw, const gcp_controls *c, gcp_meta *meta);

/* Centre crop of the frame with the output aspect ratio, divided by zoom (>= 1), even coordinates. */
gisp_rect gcp_crop(const GuampCamIosFrame *fr, uint32_t out_w, uint32_t out_h, float zoom);

/* Preview: prepare once per frame (rebuilds the gisp_fast plan when geometry or output change), then the bands. */
int gcp_render_fast_prepare(gcp *p, const gisp_raw *raw, const gisp_rect *crop, uint32_t out_w, uint32_t out_h, int full_range);
int gcp_render_fast_band(gcp *p, const gisp_raw *raw, gisp_nv12 *out, uint32_t band, uint32_t nbands);
/* Still: HQ path (MHC demosaic, NR, sharpen). */
int gcp_render_hq(gcp *p, const gisp_raw *raw, const gisp_rect *crop, gisp_nv12 *out, int full_range);

const gisp_params *gcp_params(const gcp *p);
void gcp_controls_default(gcp_controls *c);

#ifdef __cplusplus
}
#endif
#endif
