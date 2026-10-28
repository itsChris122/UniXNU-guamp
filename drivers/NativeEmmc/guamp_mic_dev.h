/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * guamp audio INPUT device (microphone): the ABI between the kernel (GuampMicDev.inc, NativeEmmc) and the userland
 * AudioServerPlugIn (package/GuampAudio, audiomxd). Plain C, no kernel types: include it as is from the plug-in.
 * Companion of guamp_audio_dev.h (output): same rate, same Apple sys/ioccom.h encoding, its own node and ioctl group
 * ('M'), so the output ABI can change without touching this one (and vice versa).
 *
 *   node     /dev/guamp-mic (char device, root:mobile 0660, created with guamp-audio-mic=1)
 *   format   48000 Hz, 1 channel, signed 16-bit little endian: 2 bytes per frame. Fixed.
 *   read()   takes captured frames from the kernel ring (GUAMP_MIC_RING_BYTES). Length must be a multiple of 2
 *            (EINVAL otherwise). Blocks until at least one frame is there; with O_NONBLOCK returns what is there, or
 *            -1/EAGAIN. Nothing is captured before START. When the ring overflows the OLDEST frames are dropped
 *            (overruns counts them): the reader always gets the most recent audio.
 *   write()  not supported (ENODEV).
 *   close()  of the last opener stops the capture clock and drops what is queued.
 *   ioctl    below. One opener at a time (EBUSY): audiomxd.
 *
 * Clock: frames_captured advances at 48 kHz while running. GUAMP_MIC_IOC_POSITION gives one consistent pair
 * (frames_captured, host_time in mach_absolute_time units) for the plug-in's GetZeroTimeStamp.
 * Sources: GUAMP_MIC_SOURCE_NULL = kernel timer only, silence at the right pace (the ADSP capture path is not up, or its
 * shared memory is missing); GUAMP_MIC_SOURCE_ADSP = q6asm read session (GuampMicPath.inc) fed by AFE TX_CODEC_DMA_TX_3.
 * Mic: GUAMP_MIC_AMIC3 (main, bottom: Android "handset-mic"/"speaker-mic") or GUAMP_MIC_AMIC1 (secondary), both analog
 * on wcd937x; the kernel picks one at boot (guamp-audio-mic-src), SET_MIC switches only while stopped.
 */
#ifndef GUAMP_MIC_DEV_H
#define GUAMP_MIC_DEV_H
#include <stdint.h>
/* The Apple sys/ioccom.h encoding, spelled out so the values are the same on the PC (where Linux _IO differs) */
#define GUAMP_MIC_IOC(inout, num, len) ((uint32_t)(inout) | ((uint32_t)((len) & 0x1fffu) << 16) | ((uint32_t)0x4du /* 'M' */ << 8) | (num))
#define GUAMP_MIC_IO(n) GUAMP_MIC_IOC(0x20000000u, (n), 0)
#define GUAMP_MIC_IOR(n, t) GUAMP_MIC_IOC(0x40000000u, (n), sizeof(t))
#define GUAMP_MIC_IOW(n, t) GUAMP_MIC_IOC(0x80000000u, (n), sizeof(t))

#define GUAMP_MIC_DEV_PATH "/dev/guamp-mic"
#define GUAMP_MIC_ABI 1
#define GUAMP_MIC_RATE 48000u
#define GUAMP_MIC_CHANNELS 1u
#define GUAMP_MIC_BITS 16u
#define GUAMP_MIC_FRAME_BYTES 2u
#define GUAMP_MIC_PERIOD_FRAMES 480u          /* 10 ms: one ADSP read (ASM_DATA_CMD_READ_V2) */
#define GUAMP_MIC_RING_BYTES (32u * 1024u)    /* 16384 frames, ~341 ms */

enum guamp_mic_state { GUAMP_MIC_STOPPED = 0, GUAMP_MIC_RUNNING = 1 };
enum guamp_mic_source { GUAMP_MIC_SOURCE_NULL = 0, GUAMP_MIC_SOURCE_ADSP = 1 };
enum guamp_mic_select { GUAMP_MIC_AMIC3 = 3, GUAMP_MIC_AMIC1 = 1 };

struct guamp_mic_info {
  uint32_t abi;             /* GUAMP_MIC_ABI */
  uint32_t rate;            /* 48000 */
  uint32_t channels;        /* 1 */
  uint32_t bits;            /* 16 */
  uint32_t ring_bytes;      /* GUAMP_MIC_RING_BYTES */
  uint32_t period_frames;   /* GUAMP_MIC_PERIOD_FRAMES */
  uint32_t state;           /* enum guamp_mic_state */
  uint32_t source;          /* enum guamp_mic_source */
  uint32_t mic;             /* enum guamp_mic_select */
  int32_t gain_cb;          /* digital gain of the TX decimator, centibel (0 = 0 dB) */
  uint32_t overruns;        /* frames dropped because nobody read them */
  uint32_t peak;            /* highest |sample| of the last period (0..32767): a level meter for the logs */
  uint64_t frames_captured; /* since open */
  uint64_t frames_read;     /* since open */
};

struct guamp_mic_position {
  uint64_t frames_captured; /* frames produced by the source since open */
  uint64_t host_time;       /* mach_absolute_time() at which frames_captured was exact */
  uint32_t queued_frames;   /* still in the ring */
  uint32_t overruns;
};

#define GUAMP_MIC_IOC_INFO     GUAMP_MIC_IOR(1, struct guamp_mic_info)
#define GUAMP_MIC_IOC_START    GUAMP_MIC_IO(2)                          /* capture clock runs from now */
#define GUAMP_MIC_IOC_STOP     GUAMP_MIC_IO(3)                          /* stop, drop queued frames */
#define GUAMP_MIC_IOC_POSITION GUAMP_MIC_IOR(4, struct guamp_mic_position)
#define GUAMP_MIC_IOC_GAIN     GUAMP_MIC_IOW(5, int32_t)                /* centibel -8400..4000 (TX_VOL_CTL steps of 1 dB) */
#define GUAMP_MIC_IOC_SET_MIC  GUAMP_MIC_IOW(6, uint32_t)               /* EBUSY while running, EINVAL unknown */

#endif
