// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp 3.5 mm jack (wcd937x MBHC), boot argument guamp-audio-jack=1: a kernel thread polls the MBHC of the wcd937x
// (GuampAudioHwCore.h gah_mbhc_*) over the codec register link of the capture side (guamp_wcd937x_read/write: needs
// guamp-audio-mic=1, hold 1) and publishes every insertion / removal / plug type on /dev/guamp-jack (guamp_jack_dev.h)
// for the AudioServerPlugIn, which switches the iOS output by itself. The kernel never changes the playback route.
// Options: guamp-audio-jack-ms=<poll period> (20..1000, default 100), guamp-audio-jack-level=<polls> (0..50, default 5:
// the SWCH_LEVEL_REMOVE level must disagree that many polls in a row before it counts when the MBHC_SW_DET interrupt
// status never comes; 0 = interrupt status only).
// Log lines "GuampJack: ..."; the verdicts: "JACK INSERTED (headphone|headset|high impedance|unknown, mic N mV, seq S)",
// "JACK REMOVED (seq S)", "JACK STATE (...)" at the first known state, "MBHC ARMED" / "MBHC LINK DOWN".
#ifndef GUAMP_JACK_DEV_INC
#define GUAMP_JACK_DEV_INC
#include "guamp_jack_dev.h"
#include "GuampAudioHwCore.h"
#include "GuampAudioMicHw.h"
#include <sys/conf.h>
#include <sys/errno.h>
#include <miscfs/devfs/devfs.h>

namespace jackdev {

constinit IOLock *gLock = nullptr;   // gState + the WAIT sleepers
constinit bool gOn = false, gRedetect = false, gArmed = false;
constinit unsigned gPollMs = 100, gLevelPolls = 5, gSettle = 0, gLinkLogged = 0;
constinit int gMajor = -1;
constinit void *gNode = nullptr;
guamp_jack_state gState = {};
gah_jack gJack = {};

int jRd(void *, uint16_t reg, uint8_t *v) { return guamp_wcd937x_read(reg, v); }
int jWr(void *, uint16_t reg, uint8_t v) { return guamp_wcd937x_write(reg, v); }
void jSleep(void *, unsigned ms) { IOSleep(ms ? ms : 1); }
void jDelay(void *, unsigned us) { IODelay(us); }
void jNote(void *, const char *step, int rc, uint32_t a, uint32_t b) {
  IOLog("GuampJack: %s: rc %d (0x%x 0x%x)\n", step, rc, a, b);
}
const gah_ops kOps = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, jRd, jWr, jSleep, jDelay, jNote, nullptr};

const char *typeName(uint32_t t) {
  switch (t) {
    case GUAMP_JACK_HEADPHONE: return "headphone";
    case GUAMP_JACK_HEADSET: return "headset";
    case GUAMP_JACK_HIGH_IMPEDANCE: return "high impedance";
    case GUAMP_JACK_UNKNOWN: return "unknown";
    default: return "none";
  }
}
uint32_t flagsNow() {
  uint32_t f = gArmed ? GUAMP_JACK_F_MBHC : 0;
  if ((audiodev::gStream.routes & (1u << GUAMP_AUDIO_DEV_ROUTE_HEADPHONE)) && audiohw::gMode) f |= GUAMP_JACK_F_HPH_ROUTE;
  if (gJack.present && gJack.type == GAH_JACK_HEADSET && gJack.micb2) f |= GUAMP_JACK_F_MIC;
  return f;
}
// Publish the MBHC state when present/type differ from what readers know (new seq, wake the WAIT sleepers); flags and
// counters are refreshed without a new seq.
void publish(bool force) {
  static_assert(int(GAH_JACK_HEADPHONE) == int(GUAMP_JACK_HEADPHONE) && int(GAH_JACK_HEADSET) == int(GUAMP_JACK_HEADSET) &&
                    int(GAH_JACK_HIGH_HPH) == int(GUAMP_JACK_HIGH_IMPEDANCE) && int(GAH_JACK_UNKNOWN) == int(GUAMP_JACK_UNKNOWN),
                "MBHC plug types = ABI types");
  static_assert(GUAMP_JACK_ROUTE_HEADPHONE == GUAMP_AUDIO_DEV_ROUTE_HEADPHONE &&
                    GUAMP_JACK_ROUTE_SPEAKER == GUAMP_AUDIO_DEV_ROUTE_SPEAKER, "route hints = /dev/guamp-audio routes");
  IOLockLock(gLock);
  const uint32_t present = gJack.present, type = present ? gJack.type : GUAMP_JACK_NONE;
  const bool changed = force || gState.seq == 0 || present != gState.present || type != gState.type;
  if (changed) {
    if (present && !gState.present) gState.inserts++;
    if (!present && gState.present) gState.removals++;
    gState.seq++;
    gState.present = present;
    gState.type = type;
    gState.mic_mv = present ? gJack.mic_mv : 0;
    gState.route_hint = present ? GUAMP_JACK_ROUTE_HEADPHONE : GUAMP_JACK_ROUTE_SPEAKER;
    gState.host_time = mach_absolute_time();
  }
  gState.flags = flagsNow();
  const guamp_jack_state st = gState;
  if (changed) IOLockWakeup(gLock, &gState.seq, false);
  IOLockUnlock(gLock);
  if (!changed) return;
  if (st.seq == 1 || force)
    IOLog("GuampJack: JACK STATE (%s, %s, mic %u mV, route hint %u, flags 0x%x, seq %u)\n", st.present ? "plugged" : "empty",
          typeName(st.type), st.mic_mv, st.route_hint, st.flags, st.seq);
  else if (st.present)
    IOLog("GuampJack: JACK INSERTED (%s, mic %u mV, adc 0x%02x, route hint %u, flags 0x%x, seq %u)\n", typeName(st.type),
          st.mic_mv, gJack.adc, st.route_hint, st.flags, st.seq);
  else
    IOLog("GuampJack: JACK REMOVED (route hint %u, seq %u, %u inserts / %u removals)\n", st.route_hint, st.seq, st.inserts,
          st.removals);
}

void pollThread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-jack");
  for (;;) {
    uint8_t id = 0;
    const int link = guamp_wcd937x_read(WCD_DIG_CHIP_ID0, &id);
    if (link) {
      if (gArmed || !gLinkLogged) {
        IOLog("GuampJack: MBHC LINK DOWN (wcd937x registers unreachable, %d: the capture side's VA SoundWire link; "
              "guamp-audio-mic=1)\n", link);
        gLinkLogged = 1;
      }
      gArmed = false;
      gJack.inited = 0;
      IOLockLock(gLock);
      gState.flags = flagsNow();
      IOLockUnlock(gLock);
      IOSleep(1000);
      continue;
    }
    if (gRedetect || !gJack.inited || gah_mbhc_lost(&kOps)) {
      const bool asked = gRedetect;
      gRedetect = false;
      const int r = gah_mbhc_init(&kOps, &gJack);
      gArmed = r == 0;
      gSettle = gLevelPolls + 3;   // let the switch interrupt (96 ms debounce) or the level fallback find a plug
      IOLog("GuampJack: MBHC %s (%d), wcd937x chip id0 0x%02x, poll %u ms, level fallback %u polls%s\n",
            r ? "INIT FAILED" : "ARMED", r, id, gPollMs, gLevelPolls, asked ? ", redetect asked" : "");
      if (r) { IOSleep(1000); continue; }
    }
    const int e = gah_mbhc_poll(&kOps, &gJack, gLevelPolls);
    if (e < 0) gJack.inited = 0;   // link went away in the middle: start over
    if (gSettle && --gSettle == 0) publish(gState.seq == 0);
    else if (e == 1 && !gSettle) publish(false);
    IOSleep(gPollMs);
  }
}

// ---- /dev/guamp-jack ------------------------------------------------------------------------------------------------
int jOpen(dev_t dev, int flags, int devtype, struct proc *p) { return minor(dev) == 0 && gLock ? 0 : ENXIO; }
int jClose(dev_t dev, int flags, int devtype, struct proc *p) { return 0; }
int jRead(dev_t dev, struct uio *uio, int ioflag) { return ENODEV; }
int jWrite(dev_t dev, struct uio *uio, int ioflag) { return ENODEV; }
int jIoctl(dev_t dev, u_long cmd, caddr_t data, int fflag, struct proc *p) {
  guamp_jack_state *st = reinterpret_cast<guamp_jack_state *>(data);
  switch (static_cast<uint32_t>(cmd)) {
    case GUAMP_JACK_IOC_STATE:
      IOLockLock(gLock);
      gState.flags = flagsNow();
      *st = gState;
      IOLockUnlock(gLock);
      return 0;
    case GUAMP_JACK_IOC_WAIT: {
      const uint32_t seen = st->seq;
      uint32_t ms = st->timeout_ms ? st->timeout_ms : 5000;
      if (ms > 60000) ms = 60000;
      uint64_t deadline = 0;
      clock_interval_to_deadline(ms, kMillisecondScale, &deadline);
      int e = 0;
      IOLockLock(gLock);
      while (gState.seq == seen) {
        const int w = IOLockSleepDeadline(gLock, &gState.seq, deadline, THREAD_INTERRUPTIBLE);
        if (w == THREAD_TIMED_OUT) { e = ETIMEDOUT; break; }
        if (w == THREAD_INTERRUPTED) { e = EINTR; break; }
      }
      *st = gState;
      IOLockUnlock(gLock);
      return e;
    }
    case GUAMP_JACK_IOC_REDETECT:
      gRedetect = true;
      IOLog("GuampJack: redetect asked\n");
      return 0;
    default: return ENOTTY;
  }
}
int jSelect(dev_t dev, int which, void *wql, struct proc *p) { return 0; }
int jStop(struct tty *tp, int rw) { return ENODEV; }
int jReset(int uban) { return ENODEV; }
int jMmap(void) { return ENODEV; }
void jStrategy(struct buf *bp) {}
const struct cdevsw kSw = {
  jOpen, jClose, jRead, jWrite, jIoctl, jStop, jReset, nullptr, jSelect, jMmap, jStrategy, nullptr, nullptr, 0,
};

void nodeThread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-jack-dev");
  for (unsigned i = 0; i < 30 && !gNode; i = i + 1) {
    gNode = devfs_make_node(makedev(gMajor, 0), DEVFS_CHAR, UID_ROOT, 501 /* mobile: audiomxd */, 0660, "guamp-jack");
    if (!gNode) IOSleep(1000);
  }
  IOLog("GuampJack: %s %s (major %d, ABI %u, state %u bytes, STATE %08x WAIT %08x REDETECT %08x)\n", GUAMP_JACK_DEV_PATH,
        gNode ? "created" : "NOT created after 30 s", gMajor, GUAMP_JACK_ABI, static_cast<unsigned>(sizeof(guamp_jack_state)),
        static_cast<unsigned>(GUAMP_JACK_IOC_STATE), static_cast<unsigned>(GUAMP_JACK_IOC_WAIT),
        static_cast<unsigned>(GUAMP_JACK_IOC_REDETECT));
}

void start() {
  unsigned on = 0, v = 0;
  PE_parse_boot_argn("guamp-audio-jack", &on, sizeof(on));
  if (on != 1 || gLock) return;
  if (PE_parse_boot_argn("guamp-audio-jack-ms", &v, sizeof(v)) && v >= 20 && v <= 1000) gPollMs = v;
  if (PE_parse_boot_argn("guamp-audio-jack-level", &v, sizeof(v)) && v <= 50) gLevelPolls = v;
  gLock = IOLockAlloc();
  if (!gLock) {
    IOLog("GuampJack: allocation failed\n");
    return;
  }
  gState.abi = GUAMP_JACK_ABI;
  gState.route_hint = GUAMP_JACK_ROUTE_SPEAKER;
  gMajor = cdevsw_add(-1, &kSw);
  if (gMajor < 0) IOLog("GuampJack: cdevsw_add failed %d\n", gMajor);
  thread_t t = nullptr;
  if (gMajor >= 0 && kernel_thread_start(nodeThread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
  t = nullptr;
  if (kernel_thread_start(pollThread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
  gOn = true;
  IOLog("GuampJack: on: poll %u ms, level fallback %u polls; waits for the wcd937x register link (guamp-audio-mic=1)\n",
        gPollMs, gLevelPolls);
}

}  // namespace jackdev
#endif
