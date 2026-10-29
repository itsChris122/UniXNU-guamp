/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * guamp 3.5 mm jack events: the ABI between the kernel (GuampJackDev.inc, wcd937x MBHC) and the userland
 * AudioServerPlugIn (package/GuampAudio, audiomxd). Plain C, no kernel types. Owned by the kernel side: the plug-in
 * copies this file VERBATIM into package/GuampAudio (its CI checks out only that directory) and its check script should
 * compare the two copies, as for guamp_mic_dev.h. Companion of guamp_audio_dev.h (output, group 'g') and
 * guamp_mic_dev.h (input, 'M'): own node, own ioctl group ('j'), so none of them changes for this.
 *
 *   node     /dev/guamp-jack (char device, root:mobile 0660, created with guamp-audio-jack=1). Any number of openers.
 *   read()/write()  not supported (ENODEV): everything goes through the ioctls (stateless, so several readers work).
 *   ioctl    (Apple sys/ioccom.h encoding spelled out below; 0 or -1/errno)
 *     GUAMP_JACK_IOC_STATE   struct guamp_jack_state out: the state now. Never blocks.
 *     GUAMP_JACK_IOC_WAIT    struct guamp_jack_state in/out. In: seq = the last seq the caller has seen, timeout_ms
 *                            (0 = 5000, max 60000). Blocks until the kernel's seq differs from it, then returns the
 *                            new state (0). On timeout: -1/ETIMEDOUT (nothing copied out: call again, or STATE).
 *                            A signal ends the wait with -1/EINTR. Call it in a loop from one plug-in thread.
 *     GUAMP_JACK_IOC_REDETECT  no data: forget the state and detect again (new seq when it is known again).
 *
 * Meaning (one event = seq + 1; the first state after boot is seq 1, seq 0 = MBHC not running yet):
 *   present   1 = a plug is in the 3.5 mm jack (mechanical switch), 0 = nothing.
 *   type      GUAMP_JACK_NONE / HEADPHONE (3-pole, no microphone) / HEADSET (4-pole: microphone, MICBIAS2 on) /
 *             HIGH_IMPEDANCE (line out or a cable without load) / UNKNOWN (plugged, the measurement failed).
 *   route_hint  what iOS should do on its own: GUAMP_JACK_ROUTE_HEADPHONE (3) while present, else
 *             GUAMP_JACK_ROUTE_SPEAKER (1). Same numbers as GUAMP_AUDIO_DEV_ROUTE_* of guamp_audio_dev.h.
 *   flags     GUAMP_JACK_F_MBHC: the codec registers are reachable and MBHC is armed (0 = state not known: the wcd937x
 *             register link of the capture side is down, see guamp-audio-mic=1); GUAMP_JACK_F_HPH_ROUTE: the kernel
 *             accepts route 3 on /dev/guamp-audio (guamp-audio-cdc=1 and the RX path linked); GUAMP_JACK_F_MIC: the
 *             headset microphone is biased (MICBIAS2) and selectable for capture.
 *
 * What the plug-in is expected to do (proposal for the iOS agent, not implemented on this side):
 *   - a thread loops on GUAMP_JACK_IOC_WAIT; on each new seq it updates its output device: "Headphones" data source /
 *     port available while present, built-in speaker otherwise (kAudioDevicePropertyDataSource +
 *     kAudioDevicePropertyDataSources, and jack connected kAudioDevicePropertyJackIsConnected), then
 *     PropertiesChanged so audiomxd/CoreAudio re-route by itself (iOS route change notification
 *     AVAudioSessionRouteChangeReasonNewDeviceAvailable / OldDeviceUnavailable: on removal iOS pauses media apps).
 *   - the plug-in then sets GUAMP_AUDIO_SET_ROUTE (route_hint) on /dev/guamp-audio itself; the kernel never switches
 *     the playback route behind its back.
 *   - with GUAMP_JACK_HEADSET, an input data source "Headset Microphone" may be offered (capture of AMIC2 is a kernel
 *     to-do: today /dev/guamp-mic stays on the built-in microphone).
 */
#ifndef GUAMP_JACK_DEV_H
#define GUAMP_JACK_DEV_H
#include <stdint.h>
#define GUAMP_JACK_IOC(inout, num, len) ((uint32_t)(inout) | ((uint32_t)((len) & 0x1fffu) << 16) | ((uint32_t)0x6au /* 'j' */ << 8) | (num))

#define GUAMP_JACK_DEV_PATH "/dev/guamp-jack"
#define GUAMP_JACK_ABI 1

enum guamp_jack_type {
  GUAMP_JACK_NONE = 0,
  GUAMP_JACK_HEADPHONE = 1,
  GUAMP_JACK_HEADSET = 2,
  GUAMP_JACK_HIGH_IMPEDANCE = 3,
  GUAMP_JACK_UNKNOWN = 4,
};
#define GUAMP_JACK_ROUTE_SPEAKER 1u
#define GUAMP_JACK_ROUTE_HEADPHONE 3u
#define GUAMP_JACK_F_MBHC 0x1u
#define GUAMP_JACK_F_HPH_ROUTE 0x2u
#define GUAMP_JACK_F_MIC 0x4u

struct guamp_jack_state {
  uint32_t abi;          /* GUAMP_JACK_ABI */
  uint32_t seq;          /* +1 at every change of present/type; in: last seen (WAIT) */
  uint32_t present;      /* 0 / 1 */
  uint32_t type;         /* enum guamp_jack_type */
  uint32_t route_hint;   /* GUAMP_JACK_ROUTE_* */
  uint32_t flags;        /* GUAMP_JACK_F_* */
  uint32_t mic_mv;       /* MBHC ADC reading of the microphone ring at insertion (mV), 0 if none */
  uint32_t timeout_ms;   /* in (WAIT): 0 = 5000 */
  uint64_t host_time;    /* mach_absolute_time() of the last change */
  uint32_t inserts;      /* insertions since boot */
  uint32_t removals;     /* removals since boot */
  uint32_t reserved[6];  /* 0 */
};

#define GUAMP_JACK_IOC_STATE    GUAMP_JACK_IOC(0x40000000u, 1, sizeof(struct guamp_jack_state))
#define GUAMP_JACK_IOC_WAIT     GUAMP_JACK_IOC(0xc0000000u, 2, sizeof(struct guamp_jack_state))
#define GUAMP_JACK_IOC_REDETECT GUAMP_JACK_IOC(0x20000000u, 3, 0)

#endif
