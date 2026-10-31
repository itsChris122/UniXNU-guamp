// SPDX-License-Identifier: BSD-2-Clause-Patent
// /dev/guamp-audio (guamp-audio-dev=1): the PCM output node the userland AudioServerPlugIn writes to; ABI v2 in
// package/GuampAudio/guamp_audio_dev.h (the one header shared with the plug-in: 48 kHz, s16le stereo, ioctls
// GET_INFO/START/STOP/SET_VOLUME/SET_ROUTE/GET_ROUTE/GET_POSITION/DRAIN). Ring and clock in GuampAudioRing.inc.
// Sinks: null (frames dropped at the 48 kHz pace of the host clock) until the ADSP chain streams; with
// guamp-adsp-ipc=6 apr::Playback (GuampAsm.inc, glue GuampAudioPlay.inc) attaches and pulls the ring period by period.
// A plain cdevsw like GuampBtSerial.inc (no IOSerialFamily/IOAudioFamily in this kernel). One opener (audiomxd, user
// mobile): node root:mobile 0660. Own thread "guamp-audio-dev": devfs may not be up at platform start, so the node is
// retried once a second (30 s). guamp-audio-cdc=1 also accepts the earpiece/headphone routes (codec DMA 0xB030).
#ifndef GUAMP_AUDIO_DEV_INC
#define GUAMP_AUDIO_DEV_INC
#include "GuampAudioRing.inc"
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/vnode.h>
#include <miscfs/devfs/devfs.h>

// patches/xnu/zzzzz-guamp-mac-dev-exempt.patch: [0] /dev/guamp-audio, [1] /dev/guamp-mic. audiomxd's sandbox profile names only
// Apple audio nodes; a registered major skips the MAC policies for lookup/open/read/write/ioctl (owner and mode still apply).
// guamp-audio-mac=0 leaves the decision to the sandbox (diagnostics).
extern "C" int guamp_mac_exempt_major[2];
inline bool guamp_audio_mac_exempt_wanted() {
  unsigned on = 1;
  PE_parse_boot_argn("guamp-audio-mac", &on, sizeof(on));
  return on != 0;
}

namespace audiodev {

IOLock *gLock = nullptr;
guampaudio::Stream gStream;
bool gOpen = false;
int gMajor = -1;
void *gNode = nullptr;
uint64_t gWrittenFrames = 0;
uint32_t gOpens = 0, gWrites = 0, gIoctls = 0, gBlocked = 0;   // gWrites/gIoctls/gBlocked: this session (reset at open)
uint32_t gBusy = 0;                                            // opens refused (EBUSY) since boot
int gOwnerPid = -1;
char gOwner[32];
// guamp-audio-log (default 1): one line per session (sound); 0 = only the first 8 and then one in 64, as before
unsigned gLogAll = 1;
// The session line. A sound that ended with frames still queued (STOP -> draining) is logged when its tail has been
// played (pollPending(), from the wanted() hook of the ADSP sink) or when the next open comes first, so "played" is final.
extern "C" int guamp_coreaudio_host_clock(uint64_t unslid, uint64_t *seen_out);

struct SessionLine {
  bool pending = false, cut = false;
  uint32_t n = 0, writes = 0, blocked = 0, ioctls = 0, underruns = 0, prerollWaits = 0, maxQueued = 0, routes = 0;
  uint32_t volumes = 0, holds = 0, holdDrops = 0, drains = 0, gain = 0;
  uint64_t written = 0, played = 0, trimmed = 0;
  uint32_t maxGapMs = 0, lateWrites = 0, pulls = 0, pullMs = 0, maxBurst = 0;
  int pid = -1;
  char who[32] = {};
};
SessionLine gPending;

int adOpen(dev_t dev, int flags, int devtype, struct proc *p);
int adClose(dev_t dev, int flags, int devtype, struct proc *p);
int adRead(dev_t dev, struct uio *uio, int ioflag);
int adWrite(dev_t dev, struct uio *uio, int ioflag);
int adIoctl(dev_t dev, u_long cmd, caddr_t data, int fflag, struct proc *p);
int adSelect(dev_t dev, int which, void *wql, struct proc *p);
// The unused entries with their exact types (as GuampBtSerial.inc)
int adStop(struct tty *tp, int rw) { return ENODEV; }
int adReset(int uban) { return ENODEV; }
int adMmap(void) { return ENODEV; }
void adStrategy(struct buf *bp) {}

const struct cdevsw kSw = {
  adOpen, adClose, adRead, adWrite, adIoctl, adStop, adReset, nullptr, adSelect, adMmap, adStrategy, nullptr, nullptr, 0,
};

bool logged(uint32_t n) { return gLogAll || n <= 8 || !(n % 64); }

// Fills the played/trimmed/deepest-queue part of a session line (device lock held)
void sessionTail(SessionLine *l) {
  l->played = gStream.sessionPlayed();
  l->trimmed = gStream.trimmedFrames;
  l->maxGapMs = gStream.ticksPerSec ? static_cast<uint32_t>(gStream.maxWriteGap * 1000 / gStream.ticksPerSec) : 0;
  l->lateWrites = gStream.lateWrites;
  l->pulls = gStream.pulls;
  l->pullMs = gStream.ticksPerSec ? static_cast<uint32_t>((gStream.lastPull - gStream.firstPull) * 1000 / gStream.ticksPerSec) : 0;
  l->maxBurst = gStream.maxBurst;
  l->maxQueued = gStream.maxQueued;
}

void logSession(const SessionLine &l) {
  if (!logged(l.n)) return;
  // One line per sound: frames written/played, underruns (ADSP sink: periods that ran dry while playing), pre-roll
  // periods, deepest queue the ADSP saw (ms of latency), writes and their average size, final gain (Q16)
  IOLog("GuampAudioDev: session %u %s[%d]: %llu frames written, %llu played, %u underruns, %u pre-roll, max queue %u ms, "
        "%u writes (avg %llu frames, %u blocked), %u ioctls, %u route / %u volume changes, gain %x, holds %u (dropped %u), "
        "drains %u, trimmed %llu, longest write pause %u ms, %u pauses over 32 ms, DSP took %u periods in %u ms (burst %u)%s\n", l.n, l.who, l.pid, static_cast<unsigned long long>(l.written),
        static_cast<unsigned long long>(l.played), l.underruns, l.prerollWaits, l.maxQueued / (GUAMP_AUDIO_RATE / 1000),
        l.writes, static_cast<unsigned long long>(l.writes ? l.written / l.writes : 0), l.blocked, l.ioctls, l.routes,
        l.volumes, l.gain, l.holds, l.holdDrops, l.drains, static_cast<unsigned long long>(l.trimmed), l.maxGapMs,
        l.lateWrites, l.pulls, l.pullMs, l.maxBurst, l.cut ? " (tail cut short by the next open)" : "");
}

// The pending session line once its tail has been played (or dropped); called without the device lock
void pollPending() {
  if (!gLock) return;
  SessionLine l;
  IOLockLock(gLock);
  if (gPending.pending && !(gStream.draining && gStream.used())) {
    sessionTail(&gPending);
    l = gPending;
    gPending.pending = false;
  }
  IOLockUnlock(gLock);
  if (l.pending) logSession(l);
}

int adOpen(dev_t dev, int flags, int devtype, struct proc *p) {
  if (minor(dev) != 0 || !gLock) return ENXIO;
  char who[32] = {};
  proc_selfname(who, sizeof(who));
  const int pid = proc_selfpid();
  SessionLine prev;
  IOLockLock(gLock);
  if (gOpen) {
    gBusy = gBusy + 1;
    const uint32_t busy = gBusy;
    char owner[32];
    memcpy(owner, gOwner, sizeof(owner));
    const int ownerPid = gOwnerPid;
    IOLockUnlock(gLock);
    // A refused opener plays nothing for its whole IO session (the plug-in falls back to the null device): always say so
    if (busy <= 32 || !(busy % 64))
      IOLog("GuampAudioDev: open by %s[%d] refused (EBUSY, %u so far): held by %s[%d]\n", who, pid, busy, owner, ownerPid);
    return EBUSY;
  }
  if (gPending.pending) {   // the previous sound is still draining: its line now, with what has been played so far
    sessionTail(&gPending);
    gPending.cut = gStream.draining && gStream.used();
    prev = gPending;
    gPending.pending = false;
  }
  gOpen = true;
  gOpens = gOpens + 1;
  // CoreAudio's 24 MHz host clock constant set to the 19.2 MHz timer (guamp_coreaudio_host_clock, patch ...-audio-hostclock):
  // the opener is the audio server, which has the constant's page mapped. guamp-audio-hostclock=<unslid address>, 0 = off.
  static bool hostClockDone = false;
  static uint32_t hostClockTries = 0;
  if (!hostClockDone && hostClockTries < 8) {
    uint64_t unslid = 0x1b6b93908ULL, seen = 0;
    PE_parse_boot_argn("guamp-audio-hostclock", &unslid, sizeof(unslid));
    const int r = unslid ? guamp_coreaudio_host_clock(unslid, &seen) : -1;
    hostClockTries++;
    hostClockDone = r <= 0;
    IOLog("GuampAudioDev: CoreAudio host clock constant at %llx (unslid) read %llx, result %d\n",
          static_cast<unsigned long long>(unslid), static_cast<unsigned long long>(seen), r);
  }
  gStream.reset();
  gStream.beginSession();
  gWrittenFrames = 0;
  gWrites = gIoctls = gBlocked = 0;
  gOwnerPid = pid;
  memcpy(gOwner, who, sizeof(gOwner));
  const bool adsp = gStream.adspEnabled;
  const uint32_t n = gOpens, preroll = gStream.prerollFrames;
  IOLockUnlock(gLock);
  if (prev.pending) logSession(prev);
  if (n <= 8 || !(n % 64))
    IOLog("GuampAudioDev: open (%u) by %s[%d], sink %s, ABI %u, pre-roll %u frames\n", n, who, pid,
          adsp ? "adsp (on sound)" : "null", GUAMP_AUDIO_ABI_VERSION, preroll);
  return 0;
}

int adClose(dev_t dev, int flags, int devtype, struct proc *p) {
  SessionLine l;
  IOLockLock(gLock);
  gStream.consume(mach_absolute_time());
  l.n = gOpens;
  l.pid = gOwnerPid;
  memcpy(l.who, gOwner, sizeof(l.who));
  l.written = gWrittenFrames;
  l.writes = gWrites;
  l.blocked = gBlocked;
  l.ioctls = gIoctls;
  l.underruns = gStream.underruns;
  l.prerollWaits = gStream.prerollWaits;
  l.routes = gStream.routeChanges;
  l.volumes = gStream.volumeChanges;
  l.holds = gStream.holds;
  l.holdDrops = gStream.holdDrops;
  l.drains = gStream.drains;
  l.gain = gStream.gainQ16;
  sessionTail(&l);
  // The tail of the sound (STOP with frames queued) keeps playing after close: its line waits for it (pollPending)
  const bool tail = gStream.draining && gStream.used();
  if (tail) {
    l.pending = true;
    gPending = l;
  }
  gStream.reset();
  gOpen = false;
  gOwnerPid = -1;
  IOLockUnlock(gLock);
  if (!tail) logSession(l);
  return 0;
}

int adRead(dev_t dev, struct uio *uio, int ioflag) { return ENODEV; }

int adWrite(dev_t dev, struct uio *uio, int ioflag) {
  if (uio_resid(uio) % GUAMP_AUDIO_BYTES_PER_FRAME) return EINVAL;
  gWrites = gWrites + 1;
  uint8_t buf[1024];
  bool took = false;
  while (uio_resid(uio) > 0) {
    IOLockLock(gLock);
    gStream.consume(mach_absolute_time());
    const uint32_t space = gStream.space();
    IOLockUnlock(gLock);
    if (!space) {
      if (ioflag & IO_NDELAY) return took ? 0 : EAGAIN;
      gBlocked = gBlocked + 1;
      IOSleep(5);   // ~240 frames at 48 kHz; the null sink frees space by itself, the ADSP sink by its periods
      continue;
    }
    const user_ssize_t resid = uio_resid(uio);
    uint32_t n = space < sizeof(buf) ? space : static_cast<uint32_t>(sizeof(buf));
    if (resid < static_cast<user_ssize_t>(n)) n = static_cast<uint32_t>(resid);
    const int e = uiomove(reinterpret_cast<char *>(buf), static_cast<int>(n), uio);
    if (e) return e;
    IOLockLock(gLock);
    const uint32_t pushed = gStream.push(buf, n, mach_absolute_time());   // single writer: all of it fits
    gWrittenFrames = gWrittenFrames + pushed / GUAMP_AUDIO_BYTES_PER_FRAME;
    IOLockUnlock(gLock);
    took = true;
  }
  return 0;
}

int adIoctl(dev_t dev, u_long cmd, caddr_t data, int fflag, struct proc *p) {
  gIoctls = gIoctls + 1;
  if (cmd == GUAMP_AUDIO_DRAIN) {
    for (unsigned i = 0; i < 400; i = i + 1) {   // at most 2 s (the ring holds ~341 ms)
      IOLockLock(gLock);
      gStream.consume(mach_absolute_time());
      const bool empty = !gStream.used() || !gStream.running;
      IOLockUnlock(gLock);
      if (empty) return 0;
      IOSleep(5);
    }
    return ETIMEDOUT;
  }
  IOLockLock(gLock);
  const uint32_t route = gStream.route, gain = gStream.gainQ16;
  const int e = gStream.ioctl(static_cast<uint32_t>(cmd), data, mach_absolute_time(), gWrittenFrames);
  const uint32_t newRoute = gStream.route, newGain = gStream.gainQ16;
  IOLockUnlock(gLock);
  if (((cmd == GUAMP_AUDIO_START || cmd == GUAMP_AUDIO_STOP) && gOpens <= 8) || e || route != newRoute)
    IOLog("GuampAudioDev: ioctl %x -> %d (route %u, gain %x)\n", static_cast<unsigned>(cmd), e, newRoute, newGain);
  else if (gain != newGain && gIoctls < 64)
    IOLog("GuampAudioDev: volume gain %x\n", newGain);
  return e;
}

// Writable while the ring has space (the plug-in normally blocks in write instead)
int adSelect(dev_t dev, int which, void *wql, struct proc *p) {
  if (which != FWRITE) return 0;
  IOLockLock(gLock);
  gStream.consume(mach_absolute_time());
  const uint32_t space = gStream.space();
  IOLockUnlock(gLock);
  return space ? 1 : 0;
}

void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-audio-dev");
  for (unsigned i = 0; i < 30 && !gNode; i = i + 1) {
    gNode = devfs_make_node(makedev(gMajor, 0), DEVFS_CHAR, GUAMP_AUDIO_DEV_UID, GUAMP_AUDIO_DEV_GID, GUAMP_AUDIO_DEV_MODE,
                            GUAMP_AUDIO_DEV_NAME);
    if (!gNode) IOSleep(1000);
  }
  IOLog("GuampAudioDev: %s %s (major %d, ring %u bytes, %llu host ticks/s, ABI %u, root:%u %o, routes %x, sink %s %u bit)\n",
        GUAMP_AUDIO_DEV_PATH, gNode ? "created" : "NOT created after 30 s", gMajor, gStream.bytes,
        static_cast<unsigned long long>(gStream.ticksPerSec), GUAMP_AUDIO_ABI_VERSION, GUAMP_AUDIO_DEV_GID,
        GUAMP_AUDIO_DEV_MODE, gStream.routes, gStream.adspEnabled ? "adsp" : "null", gStream.dspBits);
}

// Once, from the platform start path (after adsp::start): boot argument guamp-audio-dev=1
void start() {
  unsigned on = 0, cdc = 0;
  PE_parse_boot_argn("guamp-audio-dev", &on, sizeof(on));
  if (on != 1 || gLock) return;
  gLock = IOLockAlloc();
  gStream.buf = static_cast<uint8_t *>(IOMallocZero(guampaudio::kRingBytes));
  if (!gLock || !gStream.buf) {
    IOLog("GuampAudioDev: allocation failed\n");
    return;
  }
  gStream.bytes = guampaudio::kRingBytes;
  gStream.adspEnabled = adspipc::gLevel == 6;
  // guamp-audio-idle-ms (default 4000, 0 = off): a stream that writes only silence that long no longer holds the ADSP path
  unsigned idleMs = 4000;
  PE_parse_boot_argn("guamp-audio-idle-ms", &idleMs, sizeof(idleMs));
  if (idleMs > 600000) idleMs = 4000;
  uint64_t idle = 0;
  nanoseconds_to_absolutetime(static_cast<uint64_t>(idleMs) * 1000000ull, &idle);
  gStream.idleTicks = idle;
  gStream.dspBits = adspipc::gPcmBits;
  // guamp-audio-preroll-ms (default 40, 0 = off, at most 200): audio the ADSP sink waits for before it plays a running
  // stream, and again after the stream ran dry (GuampAudioRing.inc Stream::prerollFrames)
  unsigned prerollMs = 40;
  PE_parse_boot_argn("guamp-audio-preroll-ms", &prerollMs, sizeof(prerollMs));
  if (prerollMs > 300) prerollMs = 300;   // the ring holds 341 ms; fin20b asked 400 and fell back to 40 (35-172 underruns per sound against 1-13 at 200)
  gStream.prerollFrames = prerollMs * (GUAMP_AUDIO_RATE / 1000);
  PE_parse_boot_argn("guamp-audio-log", &gLogAll, sizeof(gLogAll));
  if (PE_parse_boot_argn("guamp-audio-cdc", &cdc, sizeof(cdc)) && cdc == 1)
    gStream.routes |= (1u << GUAMP_AUDIO_DEV_ROUTE_EARPIECE) | (1u << GUAMP_AUDIO_DEV_ROUTE_HEADPHONE);
  uint64_t second = 0;
  nanoseconds_to_absolutetime(1000000000ull, &second);
  gStream.ticksPerSec = second;
  gMajor = cdevsw_add(-1, &kSw);
  if (gMajor < 0) {
    IOLog("GuampAudioDev: cdevsw_add failed %d\n", gMajor);
    return;
  }
  if (guamp_audio_mac_exempt_wanted()) guamp_mac_exempt_major[0] = gMajor;
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}

}  // namespace audiodev
#endif
