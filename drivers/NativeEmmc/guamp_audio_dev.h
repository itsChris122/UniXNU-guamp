/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * guamp audio output device: the ABI between the kernel (GuampAudioDev.inc, NativeEmmc) and the userland
 * AudioServerPlugIn (package/GuampAudio, audiomxd). Plain C, no kernel types: include it as is from the plug-in.
 *
 *   node     /dev/guamp-audio (char device, 0666, created with guamp-audio-dev=1)
 *   format   48000 Hz, 2 channels interleaved (L, R), signed 16-bit little endian: 4 bytes per frame. Fixed.
 *   write()  appends frames to the kernel ring (GUAMP_AUDIO_RING_BYTES). Length must be a multiple of 4
 *            (EINVAL otherwise). Blocks while the ring is full; with O_NONBLOCK returns the bytes taken so far, or
 *            -1/EAGAIN if none. Writing while stopped pre-fills (nothing plays until START).
 *   read()   not supported (ENODEV): input comes later through another node.
 *   close()  of the last opener stops playback and drops what is queued.
 *   ioctl    below. One opener at a time (EBUSY): audiomxd.
 *
 * Clock: frames_played advances at exactly 48 kHz of the sink while running and frames are queued. The pair
 * (frames_played, host_time) of GUAMP_AUDIO_IOC_POSITION is one consistent sample of that clock in
 * mach_absolute_time units: the plug-in's GetZeroTimeStamp can use it directly (sample time = frames_played,
 * host time = host_time; period GUAMP_AUDIO_ZERO_TS_FRAMES). When the ring runs dry the clock stops (underruns
 * counts it) and restarts at the next write: the plug-in should keep >= 2 periods queued.
 * Sinks: GUAMP_AUDIO_SINK_NULL = kernel timer only, data dropped at the right pace (the ADSP path is not up or
 * not enabled); GUAMP_AUDIO_SINK_ADSP = q6asm/q6adm to the AFE port of the route (GuampAsm.inc, later).
 */
#ifndef GUAMP_AUDIO_DEV_H
#define GUAMP_AUDIO_DEV_H
#include <stdint.h>
#if defined(__APPLE__)
#include <sys/ioccom.h>
#else /* host tests on Linux: the BSD encoding of sys/ioccom.h, same values */
#define GUAMP_IOC_VOID 0x20000000u
#define GUAMP_IOC_OUT 0x40000000u
#define GUAMP_IOC_IN 0x80000000u
#define GUAMP_IOC(inout, group, num, len) ((inout) | (((len) & 0x1fffu) << 16) | ((group) << 8) | (num))
#define _IO(g, n) GUAMP_IOC(GUAMP_IOC_VOID, (g), (n), 0)
#define _IOR(g, n, t) GUAMP_IOC(GUAMP_IOC_OUT, (g), (n), sizeof(t))
#define _IOW(g, n, t) GUAMP_IOC(GUAMP_IOC_IN, (g), (n), sizeof(t))
#endif

#define GUAMP_AUDIO_DEV_PATH "/dev/guamp-audio"
#define GUAMP_AUDIO_ABI 1
#define GUAMP_AUDIO_RATE 48000u
#define GUAMP_AUDIO_CHANNELS 2u
#define GUAMP_AUDIO_BITS 16u
#define GUAMP_AUDIO_FRAME_BYTES 4u
#define GUAMP_AUDIO_PERIOD_FRAMES 480u          /* 10 ms: one ADSP write (ASM_DATA_CMD_WRITE_V2) */
#define GUAMP_AUDIO_RING_BYTES (64u * 1024u)    /* ~341 ms */
#define GUAMP_AUDIO_ZERO_TS_FRAMES 4800u        /* suggested kAudioDevicePropertyZeroTimeStampPeriod */

enum guamp_audio_state { GUAMP_AUDIO_STOPPED = 0, GUAMP_AUDIO_RUNNING = 1 };
enum guamp_audio_sink { GUAMP_AUDIO_SINK_NULL = 0, GUAMP_AUDIO_SINK_ADSP = 1 };
/* routes: same values as enum guamp_audio_route of GuampAudioHw.h */
enum guamp_audio_dev_route {
  GUAMP_AUDIO_DEV_ROUTE_SPEAKER = 1,
  GUAMP_AUDIO_DEV_ROUTE_EARPIECE = 2,
  GUAMP_AUDIO_DEV_ROUTE_HEADPHONE = 3,
};

struct guamp_audio_info {
  uint32_t abi;            /* GUAMP_AUDIO_ABI */
  uint32_t rate;           /* 48000 */
  uint32_t channels;       /* 2 */
  uint32_t bits;           /* 16 */
  uint32_t ring_bytes;     /* GUAMP_AUDIO_RING_BYTES */
  uint32_t period_frames;  /* GUAMP_AUDIO_PERIOD_FRAMES */
  uint32_t state;          /* enum guamp_audio_state */
  uint32_t sink;           /* enum guamp_audio_sink */
  uint32_t route;          /* enum guamp_audio_dev_route */
  int32_t volume_cb;       /* centibel, 0 = 0 dB, -9600 = silent */
  uint32_t muted;          /* 0/1 */
  uint32_t underruns;
  uint64_t frames_written; /* since open */
  uint64_t frames_played;  /* since open */
};

struct guamp_audio_position {
  uint64_t frames_played;  /* frames consumed by the sink since open */
  uint64_t host_time;      /* mach_absolute_time() at which frames_played was exact */
  uint32_t queued_frames;  /* still in the ring */
  uint32_t underruns;
};

struct guamp_audio_volume {
  int32_t centibel;        /* -9600..0 (clamped) */
  uint32_t muted;          /* 0/1 */
};

#define GUAMP_AUDIO_IOC_INFO     _IOR('A', 1, struct guamp_audio_info)
#define GUAMP_AUDIO_IOC_START    _IO('A', 2)                                  /* clock runs from now */
#define GUAMP_AUDIO_IOC_STOP     _IO('A', 3)                                  /* stop, drop queued frames */
#define GUAMP_AUDIO_IOC_VOLUME   _IOW('A', 4, struct guamp_audio_volume)
#define GUAMP_AUDIO_IOC_ROUTE    _IOW('A', 5, uint32_t)                        /* EINVAL for unknown, ENODEV if unwired */
#define GUAMP_AUDIO_IOC_POSITION _IOR('A', 6, struct guamp_audio_position)
#define GUAMP_AUDIO_IOC_DRAIN    _IO('A', 7)                                  /* block until the ring is empty */

#endif
