// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp microphone, kernel glue (audio agent 3), boot argument guamp-audio-mic=1 (needs guamp-adsp=1; raises
// guamp-adsp-ipc to 3 when lower, APR is the transport):
//   - /dev/guamp-mic (guamp_mic_dev.h): the capture node the AudioServerPlugIn reads, ring + clock of GuampMicRing.inc,
//     null source (silence at 48 kHz) until the ADSP read session delivers periods;
//   - the codec side (GuampMicCodec.inc) on mapped LPASS windows, and the DSP chain (GuampMicPath.inc), stepped from the
//     guamp-adsp thread by GuampAdspIpc.inc after "ADSP APR OK" (micglue::service / micglue::receive);
//   - the shared C hooks of GuampAudioMicHw.h (weak stand-ins for agent 1/2 parts, strong wcd937x/fs-gen exports).
// Options: guamp-audio-mic-src=1|3 (amic1 secondary, amic3 main = default), guamp-audio-mic-ms=<test ms> (100..20000,
// default 3000: levels logged every 500 ms, then the verdict), guamp-audio-mic-hold=0 (tear the chain down after the
// verdict; default 1 keeps it up for the plug-in), guamp-audio-mic-gain=<dB> (TX decimator, -84..40, default 0).
// Verdict lines (prefix "GuampMic:"): "MIC CAPTURE OK (...)", "MIC PATH UP (no shared memory ...)",
// "MIC CAPTURE FAILED (<step> status <s> ...)".
#ifndef GUAMP_MIC_DEV_INC
#define GUAMP_MIC_DEV_INC
#include "GuampMicRing.inc"
#include "GuampMicCodec.inc"
#include "GuampMicPath.inc"
#include "GuampAudioMicHw.h"
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/vnode.h>
#include <miscfs/devfs/devfs.h>

#if !defined(GUAMP_WCD937X_POWER_PROVIDED) || !GUAMP_WCD937X_POWER_PROVIDED
extern "C" __attribute__((weak)) int guamp_wcd937x_power(int) { return GUAMP_AUDIO_HW_ABSENT; }
#endif
#if !defined(GUAMP_AUDIO_SHM_PROVIDED) || !GUAMP_AUDIO_SHM_PROVIDED
extern "C" __attribute__((weak)) int guamp_audio_shm_get(uint32_t, struct guamp_audio_shm *) { return GUAMP_AUDIO_HW_ABSENT; }
extern "C" __attribute__((weak)) void guamp_audio_shm_put(struct guamp_audio_shm *) {}
#endif

namespace micglue {

struct Window { uint64_t phys, bytes; volatile uint8_t *va; };
constinit Window gWin[6] = {{guampmic::kTx, 0x1000, nullptr},  {guampmic::kVa, 0x1000, nullptr},
                            {guampmic::kSwrm, 0x2000, nullptr}, {guampmic::kSwrmHctl & ~0xFFFull, 0x1000, nullptr},
                            {guampmic::kLpi, 0x3000, nullptr},  {guampmic::kLpiSlew, 0x1000, nullptr}};
constinit bool gOn = false, gHold = true, gStarted = false, gVerdict = false, gMapped = false, gPathLogged = false;
constinit unsigned gTestMs = 3000;
constinit uint32_t gMic = GUAMP_MIC_AMIC3, gStartAt = 0, gNextLevel = 0, gLastFrames = 0, gBadAccess = 0;
constinit int32_t gGainDb = 0;
constinit IOLock *gLock = nullptr;
constinit IOLock *gCodecLock = nullptr;   // the SoundWire link: adsp thread + guamp_wcd937x_* callers
guampmic::Capture gRing;
guampmic::Codec gCodec;
apr::MicPath gPath;
guamp_audio_shm gShm = {};
constinit bool gOpen = false;
constinit int gMajor = -1;
constinit void *gNode = nullptr;
constinit uint32_t gOpens = 0, gReads = 0, gFsgenUsers = 0;
constinit uint64_t gEpochTicks = 0;

volatile uint32_t *reg(uint64_t phys) {
  for (Window &w : gWin)
    if (w.va && phys >= w.phys && phys + 4 <= w.phys + w.bytes)
      return reinterpret_cast<volatile uint32_t *>(w.va + (phys - w.phys));
  return nullptr;
}
uint32_t busRd(void *, uint64_t phys) {
  volatile uint32_t *p = reg(phys);
  if (!p) { gBadAccess++; return 0xFFFFFFFFu; }
  const uint32_t v = *p;
  __asm__ volatile("dsb sy" ::: "memory");
  return v;
}
void busWr(void *, uint64_t phys, uint32_t v) {
  volatile uint32_t *p = reg(phys);
  if (!p) { gBadAccess++; return; }
  __asm__ volatile("dsb sy" ::: "memory");
  *p = v;
  __asm__ volatile("dsb sy" ::: "memory");
}
void busDelay(void *, uint32_t us) {
  if (us >= 1000) IOSleep((us + 999) / 1000);
  else IODelay(us);
}
uint32_t nowMsLocal() {
  uint64_t ns = 0;
  absolutetime_to_nanoseconds(mach_absolute_time() - gEpochTicks, &ns);
  return static_cast<uint32_t>(ns / 1000000);
}
void codecLog(void *, const char *what, uint32_t a, uint32_t b) {
  IOLog("GuampMic: codec %s %x %x at %u ms\n", what, a, b, nowMsLocal());
}
void pathLog(void *, const char *what, uint32_t a, uint32_t b) {
  IOLog("GuampMic: dsp %s %x %x at %u ms\n", what, a, b, nowMsLocal());
}
int fsgenHook(void *, bool on) { return guamp_bolero_fsgen(on ? 1 : 0); }

// The codec steps of apr::MicPath (guamp-adsp thread)
int codecStep(void *, int step) {
  IOLockLock(gCodecLock);
  int e = 0;
  switch (step) {
    case apr::MicPath::CodecUp: {
      if (!gMapped) {
        for (Window &w : gWin) w.va = mapPhysical(w.phys, w.bytes);
        gMapped = true;
        for (const Window &w : gWin)
          if (!w.va) IOLog("GuampMic: codec window %llx not mapped\n", static_cast<unsigned long long>(w.phys));
      }
      const int p = guamp_wcd937x_power(1);
      IOLog("GuampMic: codec wcd937x power %s (%d)\n",
            p == GUAMP_AUDIO_HW_ABSENT ? "side not linked (agent 2 stand-in): trying with the ABL state" : p ? "FAILED" : "on", p);
      gCodec.bus = {nullptr, busRd, busWr, busDelay, codecLog};
      gCodec.mic = gMic;
      gCodec.gainDb = gGainDb;
      e = gCodec.bringUp(fsgenHook, nullptr);
      break;
    }
    case apr::MicPath::PathOn: e = gCodec.pathOn(); break;
    case apr::MicPath::PathOff: gCodec.pathOff(); break;
    case apr::MicPath::CodecDown:
      gCodec.bringDown(fsgenHook, nullptr);
      guamp_wcd937x_power(0);
      break;
    default: e = -1; break;
  }
  IOLockUnlock(gCodecLock);
  if (gBadAccess) IOLog("GuampMic: codec %u accesses outside the mapped windows\n", gBadAccess);
  return e;
}
void sinkPcm(void *, const uint8_t *pcm, uint32_t bytes) {
  IOLockLock(gLock);
  gRing.push(pcm, bytes, mach_absolute_time());
  IOLockUnlock(gLock);
}

// ---- called from GuampAdspIpc.inc (guamp-adsp thread) ---------------------------------------------------------------
bool enabled() { return gOn; }
// AFE/ADM answers with the mic token, ASM packets for the mic session: ours
bool receive(const uint8_t *p, uint32_t n, uint32_t now) {
  if (!gOn || !gStarted) return false;
  apr::Header h;
  const uint8_t *pl = nullptr;
  uint32_t pn = 0;
  if (!apr::parse(p, n, &h, &pl, &pn)) return false;
  const bool mine = (h.srcSvc == apr::kSvcAsm && h.destPort == gPath.port()) ||
                    ((h.srcSvc == apr::kSvcAfe || h.srcSvc == apr::kSvcAdm || h.srcSvc == apr::kSvcAsm) &&
                     (h.token & apr::kMicTokenMask) == apr::kMicToken);
  if (mine) gPath.receive(p, n, now);
  return mine;
}
void verdict() {
  gVerdict = true;
  if (gPath.setupFailed)
    IOLog("GuampMic: MIC CAPTURE FAILED (%s status %x, %u answers, codec errors %u, wcd chip id %x, swr dev %u, undo errors %u)\n",
          apr::MicPath::stepName(static_cast<int>(gPath.failedStep)), gPath.failedStatus, gPath.answered, gPath.hwErrors,
          gCodec.chipId, gCodec.swr.devNum, gPath.undoErrors);
  else if (gPath.upWithoutMemory())
    IOLog("GuampMic: MIC PATH UP (no shared memory: no samples; AFE port %x started, amic%u, wcd chip id %x, swr dev %u, "
          "votes %u/%u)\n", gPath.afePort, gMic, gCodec.chipId, gCodec.swr.devNum, gPath.votedMacro, gPath.votedDcodec);
  else if (gPath.capturing() && gPath.readDones && !gPath.readErrors)
    IOLog("GuampMic: MIC CAPTURE OK (%u frames in %u reads, peak %u, rms %u, amic%u, wcd chip id %x)\n", gPath.framesIn,
          gPath.readDones, gPath.peakMax, gPath.rms(), gMic, gCodec.chipId);
  else
    IOLog("GuampMic: MIC CAPTURE FAILED (reads %u done %u errors %u, step %s)\n", gPath.reads, gPath.readDones,
          gPath.readErrors, apr::MicPath::stepName(gPath.step));
  if (!gHold && !gPath.finished()) gPath.stop();
}
void service(uint32_t now, int (*send)(void *, const uint8_t *, uint32_t)) {
  if (!gOn) return;
  if (!gStarted) {
    gStarted = true;
    gEpochTicks = mach_absolute_time();
    gPath.ctx = nullptr;
    gPath.send = send;
    gPath.log = pathLog;
    gPath.hw = codecStep;
    gPath.sink = sinkPcm;
    const uint32_t want = apr::MicPath::kSlots * apr::MicPath::kSlotStride;
    const int s = guamp_audio_shm_get(want, &gShm);
    if (!s && gShm.cpu && gShm.bytes >= want) gPath.shm = {static_cast<uint8_t *>(gShm.cpu), gShm.iova_lsw, gShm.msw, want};
    IOLog("GuampMic: start: amic%u, AFE port %x, ASM session %u, test %u ms, hold %u, gain %d dB, shared memory %s (%d)\n",
          gMic, gPath.afePort, gPath.session, gTestMs, gHold, gGainDb, gPath.shm.cpu ? "yes" : "NO", s);
    gPath.begin(now);
  }
  gPath.stepAt(now);
  if (gPath.capturing() && !gStartAt) {
    gStartAt = now;
    gNextLevel = now + 500;
    IOLockLock(gLock);
    gRing.source = gPath.shm.cpu ? GUAMP_MIC_SOURCE_ADSP : GUAMP_MIC_SOURCE_NULL;
    IOLockUnlock(gLock);
  }
  if (gStartAt && !gVerdict && gPath.capturing() && now >= gNextLevel) {
    gNextLevel += 500;
    IOLog("GuampMic: level: %u frames (+%u), reads %u/%u, peak %u, rms %u\n", gPath.framesIn, gPath.framesIn - gLastFrames,
          gPath.readDones, gPath.reads, gPath.peakMax, gPath.rms());
    gLastFrames = gPath.framesIn;
  }
  if (!gVerdict && ((gStartAt && now - gStartAt >= gTestMs) || (gPath.finished()))) verdict();
  // SET_MIC from the plug-in (only while stopped): switch the codec path here, on the adsp thread
  if (gPath.capturing() && !gPath.stopping) {
    IOLockLock(gLock);
    const uint32_t want = gRing.mic;
    const bool idle = !gRing.running;
    IOLockUnlock(gLock);
    if (idle && want != gCodec.mic) {
      IOLockLock(gCodecLock);
      gCodec.pathOff();
      gCodec.mic = gMic = want;
      const int e = gCodec.pathOn();
      IOLockUnlock(gCodecLock);
      IOLog("GuampMic: switched to amic%u (%d)\n", want, e);
    }
  }
  if (gPath.finished() && !gPathLogged) {
    gPathLogged = true;
    IOLog("GuampMic: chain down (%u reads, %u undo errors)\n", gPath.readDones, gPath.undoErrors);
  }
}

// ---- /dev/guamp-mic ------------------------------------------------------------------------------------------------
int mOpen(dev_t dev, int flags, int devtype, struct proc *p);
int mClose(dev_t dev, int flags, int devtype, struct proc *p);
int mRead(dev_t dev, struct uio *uio, int ioflag);
int mWrite(dev_t dev, struct uio *uio, int ioflag) { return ENODEV; }
int mIoctl(dev_t dev, u_long cmd, caddr_t data, int fflag, struct proc *p);
int mSelect(dev_t dev, int which, void *wql, struct proc *p);
int mStop(struct tty *tp, int rw) { return ENODEV; }
int mReset(int uban) { return ENODEV; }
int mMmap(void) { return ENODEV; }
void mStrategy(struct buf *bp) {}
const struct cdevsw kSw = {
  mOpen, mClose, mRead, mWrite, mIoctl, mStop, mReset, nullptr, mSelect, mMmap, mStrategy, nullptr, nullptr, 0,
};

int mOpen(dev_t dev, int flags, int devtype, struct proc *p) {
  if (minor(dev) != 0 || !gLock) return ENXIO;
  IOLockLock(gLock);
  if (gOpen) {
    IOLockUnlock(gLock);
    return EBUSY;
  }
  gOpen = true;
  gOpens = gOpens + 1;
  const uint32_t mic = gRing.mic, source = gRing.source;
  gRing.reset();
  gRing.mic = mic;
  gRing.source = source;
  IOLockUnlock(gLock);
  IOLog("GuampMic: open (%u), source %s, amic%u\n", gOpens, source == GUAMP_MIC_SOURCE_ADSP ? "adsp" : "null", mic);
  return 0;
}
int mClose(dev_t dev, int flags, int devtype, struct proc *p) {
  IOLockLock(gLock);
  gRing.produce(mach_absolute_time());
  const uint64_t captured = gRing.framesCaptured(), read = gRing.read / GUAMP_MIC_FRAME_BYTES;
  const uint32_t overruns = gRing.overruns;
  gRing.running = false;
  gRing.rd = gRing.wr;
  gOpen = false;
  IOLockUnlock(gLock);
  IOLog("GuampMic: close; %llu frames captured, %llu read, %u overruns, %u reads\n",
        static_cast<unsigned long long>(captured), static_cast<unsigned long long>(read), overruns, gReads);
  return 0;
}
int mRead(dev_t dev, struct uio *uio, int ioflag) {
  if (uio_resid(uio) % GUAMP_MIC_FRAME_BYTES) return EINVAL;
  gReads = gReads + 1;
  uint8_t buf[1024];
  bool got = false;
  for (unsigned spins = 0;; spins++) {
    IOLockLock(gLock);
    gRing.produce(mach_absolute_time());
    const user_ssize_t resid = uio_resid(uio);
    const uint32_t n = gRing.take(buf, resid < static_cast<user_ssize_t>(sizeof(buf)) ? static_cast<uint32_t>(resid)
                                                                                         : static_cast<uint32_t>(sizeof(buf)));
    const bool running = gRing.running;
    IOLockUnlock(gLock);
    if (n) {
      const int e = uiomove(reinterpret_cast<char *>(buf), static_cast<int>(n), uio);
      if (e || uio_resid(uio) <= 0) return e;
      got = true;
      continue;   // more may be there; return when empty
    }
    if (got) return 0;   // returned something already
    if ((ioflag & IO_NDELAY) || !running) return EAGAIN;
    if (spins > 400) return 0;   // 2 s without a frame: let the caller look again
    IOSleep(5);
  }
}
int mIoctl(dev_t dev, u_long cmd, caddr_t data, int fflag, struct proc *p) {
  IOLockLock(gLock);
  const int e = gRing.ioctl(static_cast<uint32_t>(cmd), data, mach_absolute_time());
  const int32_t gain = gRing.gainCb;
  IOLockUnlock(gLock);
  if (!e && cmd == GUAMP_MIC_IOC_GAIN && gCodecLock) {
    IOLockLock(gCodecLock);
    gCodec.setGain(gain);
    IOLockUnlock(gCodecLock);
  }
  return e;
}
int mSelect(dev_t dev, int which, void *wql, struct proc *p) {
  if (which != FREAD) return 0;
  IOLockLock(gLock);
  gRing.produce(mach_absolute_time());
  const uint32_t used = gRing.used();
  IOLockUnlock(gLock);
  return used ? 1 : 0;
}
void nodeThread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-mic-dev");
  for (unsigned i = 0; i < 30 && !gNode; i = i + 1) {
    gNode = devfs_make_node(makedev(gMajor, 0), DEVFS_CHAR, UID_ROOT, 501 /* mobile: audiomxd */, 0660, "guamp-mic");
    if (!gNode) IOSleep(1000);
  }
  IOLog("GuampMic: /dev/guamp-mic %s (major %d, ring %u bytes, ABI %u)\n", gNode ? "created" : "NOT created after 30 s",
        gMajor, gRing.bytes, GUAMP_MIC_ABI);
}

// From adsp::start(): parses the options, allocates, creates the node. Returns true when the mic is on.
bool configure() {
  unsigned on = 0, v = 0;
  PE_parse_boot_argn("guamp-audio-mic", &on, sizeof(on));
  if (on != 1 || gLock) return gOn;
  if (PE_parse_boot_argn("guamp-audio-mic-src", &v, sizeof(v)) && (v == 1 || v == 3)) gMic = v;
  if (PE_parse_boot_argn("guamp-audio-mic-ms", &v, sizeof(v)) && v >= 100 && v <= 20000) gTestMs = v;
  if (PE_parse_boot_argn("guamp-audio-mic-hold", &v, sizeof(v))) gHold = v != 0;
  int gain = 0;
  if (PE_parse_boot_argn("guamp-audio-mic-gain", &gain, sizeof(gain)) && gain >= -84 && gain <= 40) gGainDb = gain;
  gLock = IOLockAlloc();
  gCodecLock = IOLockAlloc();
  gRing.buf = static_cast<uint8_t *>(IOMallocZero(GUAMP_MIC_RING_BYTES));
  if (!gLock || !gCodecLock || !gRing.buf) {
    IOLog("GuampMic: allocation failed\n");
    return false;
  }
  gRing.bytes = GUAMP_MIC_RING_BYTES;
  gRing.mic = gMic;
  gRing.gainCb = gGainDb * 100;
  uint64_t second = 0;
  nanoseconds_to_absolutetime(1000000000ull, &second);
  gRing.ticksPerSec = second;
  gMajor = cdevsw_add(-1, &kSw);
  if (gMajor < 0) IOLog("GuampMic: cdevsw_add failed %d\n", gMajor);
  thread_t t = nullptr;
  if (gMajor >= 0 && kernel_thread_start(nodeThread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
  gOn = true;
  return true;
}

}  // namespace micglue

// ---- exports of GuampAudioMicHw.h ------------------------------------------------------------------------------------
extern "C" int guamp_wcd937x_read(uint16_t reg, uint8_t *value) {
  if (!micglue::gCodecLock || !value) return -19;
  IOLockLock(micglue::gCodecLock);
  const bool up = micglue::gCodec.swr.up && micglue::gCodec.swr.devNum;
  const bool ok = up && micglue::gCodec.swr.read(micglue::gCodec.swr.devNum, reg, value);
  IOLockUnlock(micglue::gCodecLock);
  return !up ? -19 : ok ? 0 : -5;
}
extern "C" int guamp_wcd937x_write(uint16_t reg, uint8_t value) {
  if (!micglue::gCodecLock) return -19;
  IOLockLock(micglue::gCodecLock);
  const bool up = micglue::gCodec.swr.up && micglue::gCodec.swr.devNum;
  if (up) micglue::gCodec.swr.write(micglue::gCodec.swr.devNum, reg, value);
  IOLockUnlock(micglue::gCodecLock);
  return up ? 0 : -19;
}
// Refcounted fs-gen; called with or without gCodecLock held by the same thread? Only from codecStep (lock held) or by
// agent 2 (not holding it): it takes no lock of its own beyond the counter's atomicity on the adsp thread.
extern "C" int guamp_bolero_fsgen(int on) {
  if (!micglue::gMapped) {
    for (micglue::Window &w : micglue::gWin) w.va = mapPhysical(w.phys, w.bytes);
    micglue::gMapped = true;
  }
  guampmic::Codec c;
  c.bus = {nullptr, micglue::busRd, micglue::busWr, micglue::busDelay, nullptr};
  const uint32_t users = __atomic_load_n(&micglue::gFsgenUsers, __ATOMIC_SEQ_CST);
  if (on) {
    if (__atomic_fetch_add(&micglue::gFsgenUsers, 1, __ATOMIC_SEQ_CST) == 0) c.fsGen(true);
  } else if (users) {
    if (__atomic_sub_fetch(&micglue::gFsgenUsers, 1, __ATOMIC_SEQ_CST) == 0) c.fsGen(false);
  }
  return 0;
}
#endif
