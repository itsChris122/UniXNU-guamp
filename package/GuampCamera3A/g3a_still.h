/* SPDX-License-Identifier: BSD-2-Clause
 * guamp 3A: still capture with pre-flash metering, as a per-frame sequencer on top of g3a.h (algorithms) for the
 * per-frame loop of gcampipe (package/GuampCamPipe, agent 6), which owns AE / AWB / AF and calls this only for stills.
 * Plain C11 + libm, no allocation, no threads; includable from C++.
 *
 * Use (one g3a_still per stream, frame thread only):
 *   g3a_still_init()                       after SetMode (limits of the mode, the AE config of the stream)
 *   g3a_still_begin(flash mode, AE result, current WB)   when the client asks for a photo
 *   every frame while it returns busy:     g3a_still_frame(stats, frame info) -> carry out
 *       set_exposure -> SetExposure (us, gain)        flash >= 0 -> SetFlash (0 off, 1 pre-flash mA, 2 main mA; ms)
 *       capture      -> this frame is the photo: render it (HQ) with out.wb when capture_flash, else the current WB
 *       busy         -> keep AE, AWB and AF frozen on this frame (pre-flash and flash frames must not move them)
 *
 * Sequence with flash (G3A_FLASH_ON, or AUTO when g3a_flash_needed): ambient frame (exposure as it is) ->
 * pre-flash (FlashKind 1, pre_ma) with the exposure / pre_exposure_div -> first fully lit frame, at least 2 frames after
 * the request (rolling shutter) and with the new exposure applied -> g3a_flash_plan -> clipped: up to 3 retries at a
 * lower exposure; no useful flash light: photo without flash at the ambient exposure; else main flash (plan mA, window
 * covering lit latency + 1 frames + 40 ms, <= 500 ms) + plan exposure -> photo on the first fully lit frame, flash off,
 * ambient exposure back. Without flash: the first frame whose exposure is not pending. A capture that has not finished
 * after 40 frames is taken as it is (flash off). */
#ifndef G3A_STILL_H
#define G3A_STILL_H
#include "g3a.h"

#ifdef __cplusplus
extern "C" {
#endif

enum g3a_flash_mode { G3A_FLASH_OFF = 0, G3A_FLASH_ON = 1, G3A_FLASH_AUTO = 2 };
enum g3a_still_status { G3A_STILL_IDLE = 0, G3A_STILL_PLAIN, G3A_STILL_AMBIENT, G3A_STILL_PRE, G3A_STILL_MAIN };
enum g3a_flash_cmd { G3A_FLASH_CMD_NONE = -1, G3A_FLASH_CMD_OFF = 0, G3A_FLASH_CMD_PRE = 1, G3A_FLASH_CMD_MAIN = 2 };

typedef struct {
    int has_flash;                  /* GuampCamIosSensorInfo.hasFlash */
    uint32_t exposure_latency;      /* frames after SetExposure still exposed with the old values, for kernels that do
                                       not flag them (0: trust g3a_frame_info.pending = kGuampFrameSettingsPending) */
    uint64_t frame_ns;              /* frame period (1 / fps) */
} g3a_still_config;

typedef struct {
    uint64_t sequence;              /* GuampCamIosFrame.sequence */
    uint64_t exposure_ns;           /* applied exposure of this frame */
    float gain;                     /* applied analog x digital gain */
    int pending;                    /* a SetExposure is not on this frame yet (kGuampFrameSettingsPending) */
} g3a_frame_info;

typedef struct {
    int busy;
    int set_exposure; uint64_t exposure_ns; float gain;
    int flash; uint32_t flash_ma, flash_ms;  /* enum g3a_flash_cmd */
    int capture, capture_flash;
    float wb[3];                    /* WB of the photo when capture_flash (ambient and flash light mixed) */
} g3a_still_out;

typedef struct {
    g3a_still_config cfg;
    g3a_ae_config ae;
    g3a_flash_config fl_cfg;
    g3a_flash_ambient_t amb;
    g3a_flash_plan_t plan;
    float bias;                     /* exposure bias in EV (same as the AE target) */
    int state, retries, have_cmd;
    uint64_t cmd_seq, cap_at, deadline;
    uint64_t amb_ns; float amb_gain;
    float wb_amb[3];
    uint32_t stills, flash_stills, no_flash_light, aborts;
} g3a_still;

/* ae: the AE config of the stream (limits t_min/t_max/gain_max, weights, flicker). */
void g3a_still_init(g3a_still *s, const g3a_still_config *c, const g3a_ae_config *ae);
/* 0 accepted, -1 a capture is already running. ae_last: last AE result (AUTO decision); wb: current AWB gains. */
int g3a_still_begin(g3a_still *s, int flash_mode, const g3a_ae_result *ae_last, const float wb[3]);
/* One frame while busy (returns at once with busy = 0 when idle). */
void g3a_still_frame(g3a_still *s, const g3a_stats *stats, const g3a_frame_info *fi, g3a_still_out *out);

#ifdef __cplusplus
}
#endif
#endif
