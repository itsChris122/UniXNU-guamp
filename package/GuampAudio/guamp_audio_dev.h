/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * guamp audio playback device, ABI v2: THE one header shared by the kernel (drivers/NativeEmmc: GuampAudioRing.inc,
 * GuampAudioDev.inc, GuampAsm.inc include it as ../../package/GuampAudio/guamp_audio_dev.h) and GuampAudio.driver (the
 * AudioServerPlugIn inside audiomxd, GuampAudio.c). It lives next to the plug-in because the plug-in build (macOS runner)
 * only checks out package/GuampAudio. Plain C, fixed-size fields, no pointers, no Apple or kernel types, extern "C".
 * v1 had two incompatible copies (kernel group 'A', plug-in group 'g'); v2 merges them (plug-in layout and names, the
 * kernel's POSITION and DRAIN, route values of GuampAudioHw.h) and is the only one.
 *
 * Device node
 *   GUAMP_AUDIO_DEV_PATH, devfs char device made by the kernel with guamp-audio-dev=1. audiomxd runs as user mobile, so
 *   the node is root:mobile 0660 (GUAMP_AUDIO_DEV_UID/GID/MODE). One opener at a time: a second open() fails with EBUSY.
 *   close() of the opener implies GUAMP_AUDIO_STOP.
 *
 * Stream format (fixed, nothing to negotiate)
 *   48000 Hz, 2 channels interleaved (left, right), signed 16-bit little endian: one frame = 4 bytes. The kernel may
 *   send it to the ADSP as 24-bit (guamp-adsp-pcm-bits=24, info.dsp_bits): that conversion is the kernel's.
 *
 * write(fd, buf, n)
 *   n must be a multiple of GUAMP_AUDIO_BYTES_PER_FRAME (else EINVAL, nothing taken). The kernel copies as many whole
 *   frames as its ring has room for and returns that many bytes (a short count is normal when the ring is nearly full).
 *   With O_NONBLOCK (the plug-in always opens with it) a full ring returns -1/EAGAIN; without it the call waits for room.
 *   Frames written while stopped are kept (prefill) and play from the next GUAMP_AUDIO_START. The kernel never
 *   resamples. When the ring runs empty while started the sink plays silence and counts an underrun.
 * read()  not supported (ENODEV): capture comes through another node.
 *
 * Clock
 *   frames_played advances at 48 kHz of the sink (GUAMP_AUDIO_INFO_SINK_ADSP: the ADSP periods taken; else a kernel
 *   timer that drops the data at the right pace). GUAMP_AUDIO_GET_POSITION returns one consistent pair
 *   (frames_played, host_time in mach_absolute_time units): usable as the plug-in's zero time stamp.
 *
 * ioctl (BSD <sys/ioccom.h> encoding, spelled out below so every compiler gets the same numbers; group 'g';
 * every call returns 0 or -1/errno)
 *   GUAMP_AUDIO_GET_INFO      struct guamp_audio_info out. abi must equal GUAMP_AUDIO_ABI_VERSION, else the plug-in
 *                             closes the device and stays a null device.
 *   GUAMP_AUDIO_START         start draining the ring into the output path (idempotent).
 *   GUAMP_AUDIO_STOP          stop and discard what is still in the ring (idempotent).
 *   GUAMP_AUDIO_SET_VOLUME    struct guamp_audio_volume in: linear gain Q16 (0 .. GUAMP_AUDIO_GAIN_UNITY) already
 *                             including mute; the kernel scales the samples with it (GUAMP_AUDIO_INFO_HW_VOLUME).
 *                             centi_db and mute are informative (log). If the ioctl fails (ENOTTY, EPERM from a
 *                             sandbox) the plug-in scales the samples itself and sends unity.
 *   GUAMP_AUDIO_SET_ROUTE     uint32_t in: GUAMP_AUDIO_DEV_ROUTE_*. EINVAL for an unknown value, ENODEV for a route
 *                             this kernel has not wired (see info.routes).
 *   GUAMP_AUDIO_GET_ROUTE     uint32_t out.
 *   GUAMP_AUDIO_GET_POSITION  struct guamp_audio_position out.
 *   GUAMP_AUDIO_DRAIN         block until the ring is empty (at most 2 s, then ETIMEDOUT).
 */
#ifndef GUAMP_AUDIO_DEV_H
#define GUAMP_AUDIO_DEV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GUAMP_AUDIO_DEV_NAME "guamp-audio"
#define GUAMP_AUDIO_DEV_PATH "/dev/guamp-audio"
#define GUAMP_AUDIO_DEV_UID 0
#define GUAMP_AUDIO_DEV_GID 501 /* mobile */
#define GUAMP_AUDIO_DEV_MODE 0660

#define GUAMP_AUDIO_ABI_VERSION 2u
#define GUAMP_AUDIO_RATE 48000u
#define GUAMP_AUDIO_CHANNELS 2u
#define GUAMP_AUDIO_BITS 16u
#define GUAMP_AUDIO_BYTES_PER_FRAME 4u
#define GUAMP_AUDIO_GAIN_UNITY 0x10000u
#define GUAMP_AUDIO_PERIOD_FRAMES 480u     /* 10 ms: one ADSP write (ASM_DATA_CMD_WRITE_V2) */
#define GUAMP_AUDIO_RING_FRAMES 16384u     /* kernel ring: 64 KiB, ~341 ms */

/* Routes: the same values as enum guamp_audio_route of drivers/NativeEmmc/GuampAudioHw.h (kernel static_assert) */
#define GUAMP_AUDIO_DEV_ROUTE_NONE 0u
#define GUAMP_AUDIO_DEV_ROUTE_SPEAKER 1u   /* aw882xx, AFE PRIMARY_MI2S_RX 0x1000 */
#define GUAMP_AUDIO_DEV_ROUTE_EARPIECE 2u  /* wcd937x EAR, AFE RX_CODEC_DMA_RX_0 0xB030 */
#define GUAMP_AUDIO_DEV_ROUTE_HEADPHONE 3u /* wcd937x HPH, AFE RX_CODEC_DMA_RX_0 0xB030 */

/* guamp_audio_info.flags */
#define GUAMP_AUDIO_INFO_RUNNING 0x1u   /* between START and STOP */
#define GUAMP_AUDIO_INFO_HW_READY 0x2u  /* the ADSP path is streaming; 0 = the ring drains into nothing */
#define GUAMP_AUDIO_INFO_HW_VOLUME 0x4u /* SET_VOLUME scales the samples in the kernel */
#define GUAMP_AUDIO_INFO_SINK_ADSP 0x8u /* the ADSP sink is enabled in this boot (guamp-adsp-ipc=6) */

struct guamp_audio_info {
	uint32_t abi;            /* GUAMP_AUDIO_ABI_VERSION */
	uint32_t rate;           /* GUAMP_AUDIO_RATE */
	uint32_t channels;       /* GUAMP_AUDIO_CHANNELS */
	uint32_t bits;           /* GUAMP_AUDIO_BITS */
	uint32_t ring_frames;    /* ring capacity */
	uint32_t period_frames;  /* frames the sink takes per transfer (latency hint) */
	uint32_t flags;          /* GUAMP_AUDIO_INFO_* */
	uint32_t routes;         /* bit (1 << GUAMP_AUDIO_DEV_ROUTE_x) for each route this kernel has wired */
	uint64_t frames_written; /* accepted by write() since open */
	uint64_t frames_played;  /* consumed by the sink since open */
	uint64_t underruns;      /* periods (ADSP) or dry spells (null sink) played as silence while started */
	uint32_t route;          /* current GUAMP_AUDIO_DEV_ROUTE_* */
	uint32_t dsp_bits;       /* bits per sample sent to the ADSP (16 or 24), 0 = no ADSP sink */
};

struct guamp_audio_volume {
	uint32_t gain_q16;  /* 0 .. GUAMP_AUDIO_GAIN_UNITY, mute already applied */
	int32_t centi_db;   /* informative: -4800 .. 0 */
	uint32_t mute;      /* informative: 1 = muted */
	uint32_t reserved;
};

struct guamp_audio_position {
	uint64_t frames_played;  /* frames consumed by the sink since open */
	uint64_t host_time;      /* mach_absolute_time() at which frames_played was exact */
	uint32_t queued_frames;  /* still in the ring */
	uint32_t underruns;
};

/* BSD ioctl encoding (sys/ioccom.h): IOC_VOID 0x20000000, IOC_OUT 0x40000000, IOC_IN 0x80000000, length << 16,
 * group << 8, number. Spelled out so Linux host tests compute the Apple values too. */
#define GUAMP_AUDIO_IOC(inout, num, len) ((uint32_t)(inout) | ((uint32_t)((len) & 0x1fffu) << 16) | (0x67u << 8) | (num))
#define GUAMP_AUDIO_GET_INFO GUAMP_AUDIO_IOC(0x40000000u, 0x40u, sizeof(struct guamp_audio_info))
#define GUAMP_AUDIO_START GUAMP_AUDIO_IOC(0x20000000u, 0x41u, 0)
#define GUAMP_AUDIO_STOP GUAMP_AUDIO_IOC(0x20000000u, 0x42u, 0)
#define GUAMP_AUDIO_SET_VOLUME GUAMP_AUDIO_IOC(0x80000000u, 0x43u, sizeof(struct guamp_audio_volume))
#define GUAMP_AUDIO_SET_ROUTE GUAMP_AUDIO_IOC(0x80000000u, 0x44u, sizeof(uint32_t))
#define GUAMP_AUDIO_GET_ROUTE GUAMP_AUDIO_IOC(0x40000000u, 0x45u, sizeof(uint32_t))
#define GUAMP_AUDIO_GET_POSITION GUAMP_AUDIO_IOC(0x40000000u, 0x46u, sizeof(struct guamp_audio_position))
#define GUAMP_AUDIO_DRAIN GUAMP_AUDIO_IOC(0x20000000u, 0x47u, 0)

#ifdef __cplusplus
static_assert(sizeof(struct guamp_audio_info) == 64, "guamp_audio_info ABI");
static_assert(sizeof(struct guamp_audio_volume) == 16, "guamp_audio_volume ABI");
static_assert(sizeof(struct guamp_audio_position) == 24, "guamp_audio_position ABI");
static_assert(GUAMP_AUDIO_GET_INFO == 0x40406740u && GUAMP_AUDIO_DRAIN == 0x20006747u, "ioctl numbers");
}
#else
_Static_assert(sizeof(struct guamp_audio_info) == 64, "guamp_audio_info ABI");
_Static_assert(sizeof(struct guamp_audio_volume) == 16, "guamp_audio_volume ABI");
_Static_assert(sizeof(struct guamp_audio_position) == 24, "guamp_audio_position ABI");
_Static_assert(GUAMP_AUDIO_GET_INFO == 0x40406740u && GUAMP_AUDIO_DRAIN == 0x20006747u, "ioctl numbers");
#endif

#endif
