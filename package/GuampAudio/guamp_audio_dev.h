// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp audio playback device: the interface between GuampAudio.driver (the AudioServerPlugIn inside audiomxd) and the kernel's
// character device. Shared by both sides: plain C, fixed-size fields, no pointers, no Apple framework types. The kernel side
// (another tree) includes this file as is; user space includes it from GuampAudio.c.
//
// Device node
//   GUAMP_AUDIO_DEV_PATH, a devfs character device created by the kernel driver. audiomxd runs as user mobile (launchd
//   com.apple.audiomxd.plist UserName = mobile), so the node is root:mobile 0660 (GUAMP_AUDIO_DEV_UID/GID/MODE).
//   One opener at a time: a second open() fails with EBUSY. close() of the opener implies GUAMP_AUDIO_STOP.
//
// Stream format (fixed, nothing to negotiate)
//   48000 Hz, 2 channels interleaved (left, right), signed 16-bit little endian: one frame = 4 bytes.
//
// write(fd, buf, n)
//   n must be a multiple of GUAMP_AUDIO_BYTES_PER_FRAME (else EINVAL, nothing taken). The kernel copies as many whole frames as
//   its ring has room for and returns that many bytes (a short count is normal when the ring is nearly full). With O_NONBLOCK
//   (the plug-in always opens with it) a full ring returns -1/EAGAIN; without it the call waits for room. Frames written while
//   stopped are kept (prefill) and play from the next GUAMP_AUDIO_START. The kernel never resamples: it drains the ring at
//   the hardware's 48 kHz; when the ring runs empty while started it plays silence and counts an underrun.
//
// ioctl (BSD encoding of <sys/ioccom.h>, group 'g'; every call returns 0 or -1/errno)
//   GUAMP_AUDIO_GET_INFO    struct guamp_audio_info out. abi must equal GUAMP_AUDIO_ABI_VERSION, else the plug-in closes the
//                           device and stays a null device.
//   GUAMP_AUDIO_START       start draining the ring into the output path (idempotent).
//   GUAMP_AUDIO_STOP        stop and discard what is still in the ring (idempotent).
//   GUAMP_AUDIO_SET_VOLUME  struct guamp_audio_volume in: linear gain Q16 (0 .. GUAMP_AUDIO_GAIN_UNITY) already including
//                           mute; the dB value and the mute flag are informative (log, amplifier gain choice). The kernel
//                           applies it with no ramp longer than 10 ms. When this ioctl fails (ENOTTY, EPERM from a sandbox),
//                           the plug-in scales the samples itself and sends unity.
//   GUAMP_AUDIO_SET_ROUTE   uint32_t in: GUAMP_AUDIO_ROUTE_*. EINVAL for a route the hardware does not have.
//   GUAMP_AUDIO_GET_ROUTE   uint32_t out.
#ifndef GUAMP_AUDIO_DEV_H
#define GUAMP_AUDIO_DEV_H

#include <stdint.h>
#if defined(__APPLE__) || defined(KERNEL)
#include <sys/ioccom.h>
#else
#include <sys/ioctl.h> /* host tests only: Linux spells the same _IO/_IOR/_IOW macros there */
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define GUAMP_AUDIO_DEV_NAME "guamp-audio"
#define GUAMP_AUDIO_DEV_PATH "/dev/guamp-audio"
#define GUAMP_AUDIO_DEV_UID 0
#define GUAMP_AUDIO_DEV_GID 501 /* mobile */
#define GUAMP_AUDIO_DEV_MODE 0660

#define GUAMP_AUDIO_ABI_VERSION 1u
#define GUAMP_AUDIO_RATE 48000u
#define GUAMP_AUDIO_CHANNELS 2u
#define GUAMP_AUDIO_BITS 16u
#define GUAMP_AUDIO_BYTES_PER_FRAME 4u
#define GUAMP_AUDIO_GAIN_UNITY 0x10000u

#define GUAMP_AUDIO_ROUTE_NONE 0u
#define GUAMP_AUDIO_ROUTE_SPEAKER 1u
#define GUAMP_AUDIO_ROUTE_HEADPHONE 2u
#define GUAMP_AUDIO_ROUTE_EARPIECE 3u

/* guamp_audio_info.flags */
#define GUAMP_AUDIO_INFO_RUNNING 0x1u   /* between START and STOP */
#define GUAMP_AUDIO_INFO_HW_READY 0x2u  /* the output path (ADSP session, amplifier) is up; 0 = the ring drains into nothing */
#define GUAMP_AUDIO_INFO_HW_VOLUME 0x4u /* SET_VOLUME acts on the samples or the amplifier */

struct guamp_audio_info {
	uint32_t abi;            /* GUAMP_AUDIO_ABI_VERSION */
	uint32_t rate;           /* GUAMP_AUDIO_RATE */
	uint32_t channels;       /* GUAMP_AUDIO_CHANNELS */
	uint32_t bits;           /* GUAMP_AUDIO_BITS */
	uint32_t ring_frames;    /* ring capacity */
	uint32_t period_frames;  /* frames the hardware takes per transfer (latency hint for the plug-in) */
	uint32_t flags;          /* GUAMP_AUDIO_INFO_* */
	uint32_t routes;         /* bit (1 << GUAMP_AUDIO_ROUTE_x) for each route the hardware has */
	uint64_t frames_written; /* accepted by write() since open */
	uint64_t frames_played;  /* consumed by the hardware since open */
	uint64_t underruns;      /* periods played as silence while started */
	uint64_t reserved;
};

struct guamp_audio_volume {
	uint32_t gain_q16;  /* 0 .. GUAMP_AUDIO_GAIN_UNITY, mute already applied */
	int32_t centi_db;   /* informative: -4800 .. 0 */
	uint32_t mute;      /* informative: 1 = muted */
	uint32_t reserved;
};

#define GUAMP_AUDIO_GET_INFO _IOR('g', 0x40, struct guamp_audio_info)
#define GUAMP_AUDIO_START _IO('g', 0x41)
#define GUAMP_AUDIO_STOP _IO('g', 0x42)
#define GUAMP_AUDIO_SET_VOLUME _IOW('g', 0x43, struct guamp_audio_volume)
#define GUAMP_AUDIO_SET_ROUTE _IOW('g', 0x44, uint32_t)
#define GUAMP_AUDIO_GET_ROUTE _IOR('g', 0x45, uint32_t)

#ifdef __cplusplus
static_assert(sizeof(struct guamp_audio_info) == 64, "guamp_audio_info ABI");
static_assert(sizeof(struct guamp_audio_volume) == 16, "guamp_audio_volume ABI");
}
#else
_Static_assert(sizeof(struct guamp_audio_info) == 64, "guamp_audio_info ABI");
_Static_assert(sizeof(struct guamp_audio_volume) == 16, "guamp_audio_volume ABI");
#endif

#endif
