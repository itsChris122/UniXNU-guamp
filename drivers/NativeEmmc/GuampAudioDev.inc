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
uint32_t gOpens = 0, gWrites = 0, gIoctls = 0, gBlocked = 0;

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

int adOpen(dev_t dev, int flags, int devtype, struct proc *p) {
  if (minor(dev) != 0 || !gLock) return ENXIO;
  IOLockLock(gLock);
  if (gOpen) {
    IOLockUnlock(gLock);
    return EBUSY;
  }
  gOpen = true;
  gOpens = gOpens + 1;
  gStream.reset();
  gWrittenFrames = 0;
  const bool adsp = gStream.adspEnabled;
  IOLockUnlock(gLock);
  if (gOpens <= 8 || !(gOpens % 64))
    IOLog("GuampAudioDev: open (%u), sink %s, ABI %u\n", gOpens, adsp ? "adsp (on sound)" : "null", GUAMP_AUDIO_ABI_VERSION);
  return 0;
}

int adClose(dev_t dev, int flags, int devtype, struct proc *p) {
  IOLockLock(gLock);
  gStream.consume(mach_absolute_time());
  const uint64_t written = gWrittenFrames, played = gStream.framesPlayed();
  const uint32_t underruns = gStream.underruns, routeChanges = gStream.routeChanges, volumes = gStream.volumeChanges;
  const uint32_t holds = gStream.holds, holdDrops = gStream.holdDrops, drains = gStream.drains;
  const unsigned long long trimmed = gStream.trimmedFrames;
  const bool draining = gStream.draining;
  gStream.reset();
  gOpen = false;
  IOLockUnlock(gLock);
  if (gOpens <= 8 || !(gOpens % 64))   // iOS opens/closes for every sound: the first ones, then one in 64
    IOLog("GuampAudioDev: close (%u); %llu frames written, %llu played, %u underruns, %u writes (%u blocked), %u ioctls, "
          "%u route / %u volume changes; holds %u (dropped %u), drains %u%s, trimmed %llu frames\n", gOpens,
          static_cast<unsigned long long>(written), static_cast<unsigned long long>(played), underruns, gWrites, gBlocked,
          gIoctls, routeChanges, volumes, holds, holdDrops, drains, draining ? " (draining)" : "", trimmed);
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
