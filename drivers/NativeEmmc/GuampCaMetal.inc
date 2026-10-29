// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp-ca-metal runtime side (owner request 27/10). backboardd picks its renderer once, at launch, long before NativeAdreno publishes
// the Metal device (guamp-gpu-delay), so GuampAdrenoProbe.cpp reports here when its level 2 stages are done:
//   ready + gate open: backboardd is restarted once so launchd relaunches it on the Metal device (mode 1 does only this);
//   not usable:        the plugin's publish marker is parked (guamp_kernel_rename, zzzzzz-guamp-ca-metal.patch), software stays.
// Mode 2 also watches the Metal session and settles the boot count that the patch raised before launchd (GuampCaMetalCore.h):
//   good session: /private/var/ngpu/ca-metal.state = "attempts 0" (the next boots keep Metal)
//   bad session:  "attempts <max>" (the next boots use the software renderer), marker parked and guamp_ca_metal_enabled = 0;
//                 unless guamp-ca-metal-recover=0, backboardd is restarted once more so it composites in software now.
//   device not published within kReadyWaitSec: marker parked, the raised count stays (a hang there counts as a bad boot).
// Mode 0: the patch parked the marker at boot, nothing runs here.
#ifndef GUAMP_CA_METAL_HOST_CHECK
#include <sys/vnode.h>
#include <sys/fcntl.h>
#include <sys/uio.h>
#include <sys/signal.h>
#endif
#include "GuampCaMetalCore.h"

// The kernel is linked statically: an undefined weak_import fails the link. zzzzzz-guamp-ca-metal.patch (kern_exec.c) defines
// the mode/enabled/attempts words and guamp_kernel_rename, package/LegacyDisplayProbe.cpp the swap counter.
extern "C" unsigned int guamp_ca_metal_mode;
extern "C" volatile int guamp_ca_metal_enabled;
extern "C" volatile int guamp_ca_metal_attempts;
extern "C" int guamp_kernel_rename(const char *from, const char *to);   // zzzzzz-guamp-ca-metal.patch (vfs_syscalls.c)
extern "C" volatile uint32_t native_legacy_swap_accepted_count;
extern "C" void proc_iterate(unsigned int, int (*)(struct proc *, void *), void *, int (*)(struct proc *, void *), void *);
extern "C" int proc_pid(struct proc *);
extern "C" char *proc_best_name(struct proc *);
extern "C" struct proc *proc_find(int);
extern "C" int proc_rele(struct proc *);
extern "C" void psignal(struct proc *, int);

namespace camtl {
static const char kState[] = "/private/var/ngpu/ca-metal.state";
static const unsigned kMaxAttempts = 3;   // GUAMP_CA_METAL_MAX in the patch

static int findVisit(struct proc *p, void *arg) {
  const char *name = proc_best_name(p);
  if (name && !strcmp(name, "backboardd")) *static_cast<int *>(arg) = proc_pid(p);
  return 0;   // PROC_RETURNED
}

static int backboardd() {
  int pid = 0;
  proc_iterate(1u /* PROC_ALLPROCLIST */, findVisit, &pid, nullptr, nullptr);
  return pid;
}

static int writeState(unsigned attempts) {
  char text[32];
  const int len = snprintf(text, sizeof(text), "attempts %u\n", attempts);
  vfs_context_t ctx = vfs_context_create(nullptr);
  if (!ctx) return ENOMEM;
  vnode_t vp = NULLVP;
  // vnode_open takes FREAD/FWRITE (O_WRONLY is 1 = FREAD): opened read-only, the FWRITE close drove writecount to -1 (panic 28/10)
  int e = vnode_open(kState, FWRITE | O_CREAT | O_TRUNC | O_NOFOLLOW, 0644, 0, &vp, ctx);
  if (!e) {
    e = vn_rdwr(UIO_WRITE, vp, text, len, 0, UIO_SYSSPACE, IO_SYNC | IO_NOAUTH, vfs_context_ucred(ctx), nullptr,
                vfs_context_proc(ctx));
    const int c = vnode_close(vp, FWRITE, ctx);
    if (!e) e = c;
  }
  vfs_context_rele(ctx);
  return e;
}

static const char kActive[] = "/private/var/ngpu/addons/NativeAdrenoMetal.publish";
static const char kParked[] = "/private/var/ngpu/addons/NativeAdrenoMetal.publish.off";
static volatile int gReady = 0;   // GuampAdreno: 1 Metal device published and self-tested, 2 not usable
static const unsigned kReadyWaitSec = 600;

// The plugin returns no device without its marker: parked, a backboardd started afterwards composites in software.
static int park() { return guamp_kernel_rename(kActive, kParked); }

static void restartBackboardd(const char *why) {
  const int pid = backboardd();
  if (struct proc *p = pid ? proc_find(pid) : nullptr) {
    psignal(p, SIGKILL);
    proc_rele(p);
  }
  IOLog("GuampCaMetal: backboardd (pid %d) restarted: %s\n", pid, why);
}

static void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-ca-metal");
  unsigned recover = 1;
  PE_parse_boot_argn("guamp-ca-metal-recover", &recover, sizeof(recover));
  // backboardd picks its renderer once, at launch, before NativeAdreno publishes (guamp-gpu-delay): wait for the device first.
  uint32_t waited = 0;
  while (!gReady && waited < kReadyWaitSec) {
    IOSleep(1000);
    waited = waited + 1;
  }
  // The kern_exec patch decides at launchd's exec, long before the device (guamp-gpu-delay): the gate is final by now.
  if (guamp_ca_metal_mode != 2 || !guamp_ca_metal_enabled) {
    IOLog("GuampCaMetal: watchdog off after %u s: gate closed (mode %u, attempts %d)\n", waited, guamp_ca_metal_mode,
          guamp_ca_metal_attempts);
    return;
  }
  IOLog("GuampCaMetal: Metal boot %d of %u before the software fallback\n", guamp_ca_metal_attempts + 1, kMaxAttempts);
  if (gReady != 1) {
    const int parked = park();
    guamp_ca_metal_enabled = 0;
    const int e = gReady == 2 ? writeState(kMaxAttempts) : 0;
    IOLog("GuampCaMetal: Metal device %s after %u s: marker parked (%d), software renderer%s, status %d\n",
          gReady == 2 ? "not usable" : "not published", waited, parked,
          gReady == 2 ? " from the next boot too" : " (boot count kept raised)", e);
    return;
  }
  restartBackboardd("the Metal device is published");
  const bool haveFrames = true;
  gcm_state s = {};
  // 29/10: saf1 tripped the frame rule (60 s) while lsd rebuilt its database, and the next boots stayed in software. One-shot
  // boots (guamp-ls-rebuild / guamp-install-staged-apps) get kGcmNoFrameOneShotSec; guamp-ca-metal-noframe=N sets it (N >= 30 s).
  unsigned lsRebuild = 0, staged = 0, noFrame = 0;
  PE_parse_boot_argn("guamp-ls-rebuild", &lsRebuild, sizeof(lsRebuild));
  PE_parse_boot_argn("guamp-install-staged-apps", &staged, sizeof(staged));
  if (lsRebuild || staged) noFrame = kGcmNoFrameOneShotSec;
  if (PE_parse_boot_argn("guamp-ca-metal-noframe", &noFrame, sizeof(noFrame)) && noFrame < 30) noFrame = 30;
  s.noFrameSec = noFrame;
  IOLog("GuampCaMetal: frame rule %u s (ls-rebuild %u, staged apps %u)\n", noFrame ? noFrame : kGcmNoFrameSec, lsRebuild, staged);
  uint32_t now = 0;
  for (;;) {
    IOSleep(5000);
    now += 5;
    if (!guamp_ca_metal_enabled) return;   // the GPU was parked (guamp_ca_metal_gpu_lost): the state is already settled
    const int pid = backboardd();
    const gcm_verdict v = gcm_step(&s, now, pid, haveFrames, native_legacy_swap_accepted_count);
    if (v == GCM_WAIT) continue;
    if (v == GCM_GOOD) {
      const int e = writeState(0);
      IOLog("GuampCaMetal: render server on Metal healthy for %u s (pid %d, frames seen): boot count cleared, status %d\n",
            kGcmGoodSec, pid, e);
      return;
    }
    guamp_ca_metal_enabled = 0;
    const int e = writeState(kMaxAttempts);
    const int parked = park();
    const bool kill = recover && pid && !parked;
    IOLog("GuampCaMetal: render server on Metal unhealthy (%s: restarts %u, pid %d, up %u s): marker parked (%d), software "
          "renderer from the next boot%s, status %d\n", gcm_name(v), s.restarts, pid, pid ? now - s.since : 0, parked,
          kill ? " and now" : "", e);
    if (kill) restartBackboardd("back to the software renderer");
    return;
  }
}

// Runs from the kext start, before the patch has parsed the mode (launchd's exec): read the boot arg here, the thread checks the gate.
void start() {
  unsigned mode = 2;   // default 2 (verified 28/10), as zzzzzzzzz-guamp-default-args.patch; guamp-ca-metal=0 keeps software
  PE_parse_boot_argn("guamp-ca-metal", &mode, sizeof(mode));
  if (mode != 2) return;
  IOLog("GuampCaMetal: watchdog on, waiting for the Metal device\n");
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}
}  // namespace camtl

// GuampAdrenoProbe.cpp, after its level 2 stages: ok = accelerator published and the user-client self test passed.
extern "C" void guamp_ca_metal_gpu_ready(int ok) {
  if (!guamp_ca_metal_enabled) {
    IOLog("GuampCaMetal: Metal device %s; gate closed (guamp-ca-metal=%u): the render server stays in software\n",
          ok ? "ready" : "not usable", guamp_ca_metal_mode);
    return;
  }
  if (guamp_ca_metal_mode == 2) {   // the watchdog thread restarts backboardd and judges the session
    camtl::gReady = ok ? 1 : 2;
    return;
  }
  if (!ok) {
    const int parked = camtl::park();
    guamp_ca_metal_enabled = 0;
    IOLog("GuampCaMetal: Metal device not usable: marker parked (%d), the render server stays in software\n", parked);
    return;
  }
  camtl::restartBackboardd("the Metal device is published (guamp-ca-metal=1, no watchdog)");
}

// GuampAdrenoProbe.cpp (guamp-gpu-recover=1, GuampAdrenoRecover.inc) after it parked the GPU: hangs it could not recover. The same
// as an unhealthy session: marker parked, "attempts <max>" in mode 2 (software renderer from the next boot), and unless
// guamp-ca-metal-recover=0 backboardd restarted now so it composites in software. On its own thread: the caller holds the GPU lock
// inside a client call, and the state file is VFS work.
namespace camtl {
static void lost(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-gpu-lost");
  unsigned recover = 1;
  PE_parse_boot_argn("guamp-ca-metal-recover", &recover, sizeof(recover));
  const int parked = park();
  const int e = guamp_ca_metal_mode == 2 ? writeState(kMaxAttempts) : 0;
  const bool kill = recover && !parked;
  IOLog("GuampCaMetal: GPU parked after unrecoverable hangs: marker parked (%d), software renderer from the next boot%s, status %d\n",
        parked, kill ? " and now" : "", e);
  if (kill) restartBackboardd("the GPU is parked");
}
}  // namespace camtl

extern "C" void guamp_ca_metal_gpu_lost(void) {
  if (!guamp_ca_metal_enabled) {
    IOLog("GuampCaMetal: GPU parked; gate closed (guamp-ca-metal=%u): nothing to switch\n", guamp_ca_metal_mode);
    return;
  }
  guamp_ca_metal_enabled = 0;   // the watchdog thread stops at its next sample
  thread_t t = nullptr;
  if (kernel_thread_start(camtl::lost, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}
