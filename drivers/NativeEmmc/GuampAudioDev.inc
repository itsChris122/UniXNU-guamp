// SPDX-License-Identifier: BSD-2-Clause-Patent
// /dev/guamp-audio (guamp-audio-dev=1): the PCM output node the userland AudioServerPlugIn writes to; ABI in
// guamp_audio_dev.h (48 kHz, s16le stereo, ioctls INFO/START/STOP/VOLUME/ROUTE/POSITION/DRAIN). Ring and clock in
// GuampAudioRing.inc. Sink today: GUAMP_AUDIO_SINK_NULL (frames dropped at the 48 kHz pace of the host clock), so the
// plug-in can be brought up before the ADSP PCM path (GuampAsm.inc) exists; the ADSP sink will pull() the same ring.
// A plain cdevsw like GuampBtSerial.inc (no IOSerialFamily/IOAudioFamily in this kernel). One opener (audiomxd).
// Own thread "guamp-audio-dev": devfs may not be up at platform start, so the node is retried once a second (30 s).
#ifndef GUAMP_AUDIO_DEV_INC
#define GUAMP_AUDIO_DEV_INC
#include "GuampAudioRing.inc"
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/vnode.h>
#include <miscfs/devfs/devfs.h>

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
  IOLockUnlock(gLock);
  IOLog("GuampAudioDev: open (%u), sink %s\n", gOpens, gStream.sink == GUAMP_AUDIO_SINK_NULL ? "null" : "adsp");
  return 0;
}

int adClose(dev_t dev, int flags, int devtype, struct proc *p) {
  IOLockLock(gLock);
  gStream.consume(mach_absolute_time());
  const uint64_t written = gWrittenFrames, played = gStream.framesPlayed();
  const uint32_t underruns = gStream.underruns;
  gStream.reset();
  gOpen = false;
  IOLockUnlock(gLock);
  IOLog("GuampAudioDev: close; %llu frames written, %llu played, %u underruns, %u writes (%u blocked), %u ioctls\n",
        static_cast<unsigned long long>(written), static_cast<unsigned long long>(played), underruns, gWrites, gBlocked,
        gIoctls);
  return 0;
}

int adRead(dev_t dev, struct uio *uio, int ioflag) { return ENODEV; }

int adWrite(dev_t dev, struct uio *uio, int ioflag) {
  if (uio_resid(uio) % GUAMP_AUDIO_FRAME_BYTES) return EINVAL;
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
    gWrittenFrames = gWrittenFrames + pushed / GUAMP_AUDIO_FRAME_BYTES;
    IOLockUnlock(gLock);
    took = true;
  }
  return 0;
}

int adIoctl(dev_t dev, u_long cmd, caddr_t data, int fflag, struct proc *p) {
  gIoctls = gIoctls + 1;
  if (cmd == GUAMP_AUDIO_IOC_DRAIN) {
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
  const int e = gStream.ioctl(static_cast<uint32_t>(cmd), data, mach_absolute_time(), gWrittenFrames);
  IOLockUnlock(gLock);
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
    gNode = devfs_make_node(makedev(gMajor, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "guamp-audio");
    if (!gNode) IOSleep(1000);
  }
  IOLog("GuampAudioDev: /dev/guamp-audio %s (major %d, ring %u bytes, %llu host ticks/s, ABI %u)\n",
        gNode ? "created" : "NOT created after 30 s", gMajor, gStream.bytes,
        static_cast<unsigned long long>(gStream.ticksPerSec), GUAMP_AUDIO_ABI);
}

// Once, from the platform start path: boot argument guamp-audio-dev=1
void start() {
  unsigned on = 0;
  PE_parse_boot_argn("guamp-audio-dev", &on, sizeof(on));
  if (on != 1 || gLock) return;
  gLock = IOLockAlloc();
  gStream.buf = static_cast<uint8_t *>(IOMallocZero(GUAMP_AUDIO_RING_BYTES));
  if (!gLock || !gStream.buf) {
    IOLog("GuampAudioDev: allocation failed\n");
    return;
  }
  gStream.bytes = GUAMP_AUDIO_RING_BYTES;
  uint64_t second = 0;
  nanoseconds_to_absolutetime(1000000000ull, &second);
  gStream.ticksPerSec = second;
  gMajor = cdevsw_add(-1, &kSw);
  if (gMajor < 0) {
    IOLog("GuampAudioDev: cdevsw_add failed %d\n", gMajor);
    return;
  }
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}

}  // namespace audiodev
#endif
