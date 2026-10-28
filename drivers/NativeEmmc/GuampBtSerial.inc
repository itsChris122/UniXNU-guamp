// SPDX-License-Identifier: BSD-2-Clause-Patent
// Bluetooth M2 device nodes (skeleton, off by default: guamp-bt-shim=1), per analisi/bt-ios/PROGETTO-BT-IOS.md:
//   /dev/cu.bluetooth, /dev/uart.bluetooth   minors 0/1 of one port: the H4 stream of GuampBtShim.h (bluetoothd tries
//                                            cu.bluetooth then uart.bluetooth); one client at a time
//   /dev/btreset, /dev/btwake, /dev/btpoweroff  minors 0/1/2 of a control device (BlueTool, the BTReset class of
//                                            AppleBluetooth): open = assert, close = deassert
//
// Why a plain cdevsw and not IOSerialFamily: this kernel is built from xnu + IOStorageFamily only (UniXNU feeds); no
// IOSerialFamily, so no IOSerialBSDClient to publish /dev/cu.* the Apple way. A cdevsw entry (cdevsw_add +
// devfs_make_node, as bsd/dev/arm/km.c and the pty drivers do) gives the same nodes. struct tty is not used: the
// serial ioctls (tcgetattr/tcsetattr/cfsetspeed/cfmakeraw/tcflush/tcdrain, modem lines, IOSSIOSPEED, FIONREAD/FIONBIO)
// are answered by bts_tty_ioctl from the stored argument bytes. kevent: d_type 0 plus cdevsw_setkqueueok makes
// spec_kqfilter use EVFILTID_SPEC (bsd/miscfs/specfs/spec_vnops.c filt_specattach / filt_spec_common), which calls
// d_select and takes its return value as the byte count; so bsSelect always selrecord()s and returns the number of
// readable bytes (0 = not ready). A D_TTY with struct tty (tty_pty.c model) stays the fallback if that misbehaves.
//
// Interface toward the M1 driver (BtService.inc), not wired yet:
//   btshim::attach(link)          link.tx sends one whole H4 packet on the UART (thread context; the M1 side serializes
//                                 its UART against its own rxPump); set_bdaddr / reset / power are optional.
//   btshim::controllerBytes(p, n) every byte read from the UART after M1 completed (IBS bytes may be included, they
//                                 are skipped); thread context, takes the shim lock.
//   btshim::start()               once, from the platform start path: boot-arg, allocation, device nodes.
// Nothing here touches hardware; reset/power reach it only through the link the M1 driver attaches.
//
// Kernel controls (guamp-bt-ctl=1, with guamp-bt-shim=1; default off): iOS 17's BlueTool and bluetoothd open the UART
// as PF_SYSTEM kernel controls "com.apple.uart.bluetooth" / "com.apple.uart.sk.bluetooth" (AppleOnboardSerialBSDClient
// on Apple hardware), not as /dev/cu.bluetooth; protocol in GuampBtShim.h (bts_ctl_sockopt). Both names are
// registered here with ctl_register and carry the same H4 stream as the tty; the tty and a control share the one
// client slot (gOpen). Controller bytes reach the socket through pumpCtl (ctl_getenqueuespace, then ctl_enqueuedata),
// again on ctl_rcvd once the client has read.
#include "GuampBtShim.h"
#include "GuampBtDefaults.h"   // guamp-bt=2: verified sub-option defaults
#ifndef GUAMP_BT_SERIAL_HOST_CHECK
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/select.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/vnode.h>
#include <sys/kern_control.h>
#include <sys/kpi_mbuf.h>
#include <miscfs/devfs/devfs.h>
#endif

namespace btshim {

IOLock *gLock = nullptr;
bts_shim *gShim = nullptr;
bts_tty gTty;
struct selinfo gSel;
bool gEnabled = false, gOpen = false;
bool gCtl[3] = {};                 // btreset, btwake, btpoweroff currently open
int gMajor = -1, gCtlMajor = -1;
uint32_t gOpens = 0, gReads = 0, gWrites = 0, gIoctls = 0, gNodes = 0;
uint32_t gCtlOpens[3] = {};        // btreset, btwake, btpoweroff opens (the periodic GuampBt line shows them)
const char *const kTtyNames[2] = {"cu.bluetooth", "uart.bluetooth"};
const char *const kCtlNames[3] = {"btreset", "btwake", "btpoweroff"};
// Kernel controls: the connected client (gViaKctl, under gLock), registration refs, counters for the GuampBt line.
const char *const kKctlNames[2] = {"com.apple.uart.bluetooth", "com.apple.uart.sk.bluetooth"};
kern_ctl_ref gKctlRefs[2] = {};
IOLock *gPumpLock = nullptr;        // one pumpCtl at a time: keeps the byte order toward the socket
bool gViaKctl = false;
kern_ctl_ref gKctlRef = nullptr;
uint32_t gKctlUnit = 0, gKctlWhich = 0;
uint32_t gKctlRegs = 0, gKctlConnects = 0, gKctlBusy = 0, gKctlSends = 0, gKctlOpts = 0, gKctlOptErrs = 0;
uint32_t gKctlFull = 0, gKctlEnqErrs = 0, gKctlPreempts = 0;
uint64_t gKctlTxBytes = 0, gKctlRxBytes = 0;

int bsOpen(dev_t dev, int flags, int devtype, struct proc *p);
int bsClose(dev_t dev, int flags, int devtype, struct proc *p);
int bsRead(dev_t dev, struct uio *uio, int ioflag);
int bsWrite(dev_t dev, struct uio *uio, int ioflag);
int bsIoctl(dev_t dev, u_long cmd, caddr_t data, int fflag, struct proc *p);
int bsSelect(dev_t dev, int which, void *wql, struct proc *p);
int ctlOpen(dev_t dev, int flags, int devtype, struct proc *p);
int ctlClose(dev_t dev, int flags, int devtype, struct proc *p);
int ctlRw(dev_t dev, struct uio *uio, int ioflag);
int ctlIoctl(dev_t dev, u_long cmd, caddr_t data, int fflag, struct proc *p);
int ctlSelect(dev_t dev, int which, void *wql, struct proc *p);

// The unused entries with their exact types (the eno_* casts of sys/conf.h trip -Wcast-function-type-mismatch in C++).
int noStop(struct tty *tp, int rw) { return ENODEV; }
int noReset(int uban) { return ENODEV; }
int noMmap(void) { return ENODEV; }
void noStrategy(struct buf *bp) {}

const struct cdevsw kSw = {
  bsOpen, bsClose, bsRead, bsWrite, bsIoctl, noStop, noReset, nullptr, bsSelect, noMmap, noStrategy,
  nullptr, nullptr, 0,
};
const struct cdevsw kCtlSw = {
  ctlOpen, ctlClose, ctlRw, ctlRw, ctlIoctl, noStop, noReset, nullptr, ctlSelect, noMmap, noStrategy,
  nullptr, nullptr, 0,
};

int bsOpen(dev_t dev, int flags, int devtype, struct proc *p) {
  if (minor(dev) > 1 || !gShim) return ENXIO;
  IOLockLock(gLock);
  if (gOpen) { IOLockUnlock(gLock); return EBUSY; }   // one client: bluetoothd (or BlueTool)
  gOpen = true;
  gOpens = gOpens + 1;
  bts_tty_init(&gTty);
  bts_shim_flush_rx(gShim);
  bts_shim_flush_tx(gShim);
  const bool linked = gShim->link.tx != nullptr;
  IOLockUnlock(gLock);
  IOLog("GuampBtShim: open /dev/%s (%u), link %s\n", kTtyNames[minor(dev)], gOpens,
        linked ? "attached" : "NOT attached: commands fail");
  return 0;
}

int bsClose(dev_t dev, int flags, int devtype, struct proc *p) {
  IOLockLock(gLock);
  gOpen = false;
  IOLockWakeup(gLock, &gShim->rx, false);
  selthreadclear(&gSel);
  const bts_stats st = gShim->st;
  const uint32_t baud = gShim->baud;
  IOLockUnlock(gLock);
  IOLog("GuampBtShim: close; host %u pkts (%u cmds, %u forwarded, %u emulated, %u unknown vendor, %u tx errors), "
        "controller %u pkts (%u rewritten, %u vendor dropped, %u internal, %u dropped full), patch %u writes %llu bytes "
        "%u launches, baud %u\n", st.hostPkts, st.hostCmds, st.forwarded, st.emulated, st.unknownVendor, st.txErrors,
        st.ctrlPkts, st.rewritten, st.vendorDropped, st.internalDropped, st.rxDropped, st.writeRamCmds,
        st.writeRamBytes, st.launches, baud);
  return 0;
}

int bsRead(dev_t dev, struct uio *uio, int ioflag) {
  uint8_t buf[1024];
  IOLockLock(gLock);
  while (!bts_shim_readable(gShim)) {
    if (!gOpen) { IOLockUnlock(gLock); return 0; }
    if ((ioflag & IO_NDELAY) || gTty.nonblocking) { IOLockUnlock(gLock); return EWOULDBLOCK; }
    if (IOLockSleep(gLock, &gShim->rx, THREAD_ABORTSAFE) != THREAD_AWAKENED) { IOLockUnlock(gLock); return EINTR; }
  }
  const user_ssize_t resid = uio_resid(uio);
  const size_t want = resid < (user_ssize_t)sizeof buf ? (size_t)resid : sizeof buf;
  const size_t n = bts_shim_read(gShim, buf, want);
  gReads = gReads + 1;
  IOLockUnlock(gLock);
  return n ? uiomove(reinterpret_cast<const char *>(buf), static_cast<int>(n), uio) : 0;
}

int bsWrite(dev_t dev, struct uio *uio, int ioflag) {
  uint8_t buf[512];
  while (uio_resid(uio) > 0) {
    const user_ssize_t resid = uio_resid(uio);
    const size_t n = resid < (user_ssize_t)sizeof buf ? (size_t)resid : sizeof buf;
    const int e = uiomove(reinterpret_cast<const char *>(buf), static_cast<int>(n), uio);
    if (e) return e;
    IOLockLock(gLock);
    const size_t before = bts_shim_readable(gShim);
    bts_shim_host_write(gShim, buf, n);
    const bool more = bts_shim_readable(gShim) > before;   // synthetic replies (vendor commands, tx errors)
    gWrites = gWrites + 1;
    if (more) IOLockWakeup(gLock, &gShim->rx, false);
    IOLockUnlock(gLock);
    if (more) selwakeup(&gSel);
  }
  return 0;
}

int bsIoctl(dev_t dev, u_long cmd, caddr_t data, int fflag, struct proc *p) {
  gIoctls = gIoctls + 1;
  IOLockLock(gLock);
  int flush = 0;
  const bts_ioctl_result r = bts_tty_ioctl(&gTty, static_cast<uint32_t>(cmd), reinterpret_cast<uint8_t *>(data),
                                           bts_shim_readable(gShim), &flush);
  if (r == BTS_IOCTL_FLUSH) {
    if (flush & 1) bts_shim_flush_rx(gShim);
    if (flush & 2) bts_shim_flush_tx(gShim);
  }
  IOLockUnlock(gLock);
  return r == BTS_IOCTL_NOTTY ? ENOTTY : r == BTS_IOCTL_INVAL ? EINVAL : 0;
}

// select(2) and kevent (EVFILTID_SPEC): the return value is the readable byte count; always recorded so the knote is
// hooked even when data is already there.
int bsSelect(dev_t dev, int which, void *wql, struct proc *p) {
  if (which != FREAD) return 1;   // writes never block
  IOLockLock(gLock);
  const size_t n = bts_shim_readable(gShim);
  selrecord(p, &gSel, wql);
  IOLockUnlock(gLock);
  return n > 0x7fffffff ? 0x7fffffff : static_cast<int>(n);
}

// ---- /dev/btreset, /dev/btwake, /dev/btpoweroff -------------------------------------------------------------------
void ctlCall(unsigned which, bool asserted) {
  void (*reset)(void *, bool) = nullptr, (*power)(void *, bool) = nullptr;
  void *ctx = nullptr;
  if (gShim) {
    IOLockLock(gLock);
    reset = gShim->link.reset; power = gShim->link.power; ctx = gShim->link.ctx;
    IOLockUnlock(gLock);
  }
  // btwake: no-op (the WCN3950 sleeps in-band, IBS, handled by the M1 driver).
  if (which == 0 && reset) reset(ctx, asserted);
  if (which == 2 && power) power(ctx, asserted);
  IOLog("GuampBtShim: %s %s%s\n", kCtlNames[which], asserted ? "asserted (open)" : "deasserted (close)",
        which == 1 ? ", no-op" : (which == 0 ? reset : power) ? "" : ", no link");
}

int ctlOpen(dev_t dev, int flags, int devtype, struct proc *p) {
  const int m = minor(dev);
  if (m < 0 || m > 2) return ENXIO;
  IOLockLock(gLock);
  if (gCtl[m]) { IOLockUnlock(gLock); return EBUSY; }
  gCtl[m] = true;
  gCtlOpens[m] = gCtlOpens[m] + 1;
  IOLockUnlock(gLock);
  ctlCall(static_cast<unsigned>(m), true);
  return 0;
}

int ctlClose(dev_t dev, int flags, int devtype, struct proc *p) {
  const int m = minor(dev);
  if (m < 0 || m > 2) return ENXIO;
  IOLockLock(gLock);
  gCtl[m] = false;
  IOLockUnlock(gLock);
  ctlCall(static_cast<unsigned>(m), false);
  return 0;
}

int ctlRw(dev_t dev, struct uio *uio, int ioflag) { return 0; }
int ctlIoctl(dev_t dev, u_long cmd, caddr_t data, int fflag, struct proc *p) { return ENOTTY; }
int ctlSelect(dev_t dev, int which, void *wql, struct proc *p) { return 1; }

// ---- toward the M1 driver -------------------------------------------------------------------------------------------
void attach(const bts_link *link) {
  if (!gEnabled) return;
  IOLockLock(gLock);
  gShim->link = *link;
  IOLockUnlock(gLock);
  IOLog("GuampBtShim: controller link attached (reset %s, power %s, bdaddr %s)\n", link->reset ? "yes" : "no",
        link->power ? "yes" : "no", link->set_bdaddr ? "yes" : "no");
}

void pumpCtl();

void controllerBytes(const uint8_t *data, size_t len) {
  if (!gEnabled || !len) return;
  IOLockLock(gLock);
  const size_t before = bts_shim_readable(gShim);
  bts_shim_ctrl_input(gShim, data, len);
  const bool more = gOpen && bts_shim_readable(gShim) > before;
  const bool kctl = gViaKctl;
  if (!gOpen) bts_shim_flush_rx(gShim);   // nobody reads: do not let stale events greet the next open
  if (more && !kctl) IOLockWakeup(gLock, &gShim->rx, false);
  IOLockUnlock(gLock);
  if (more && kctl) pumpCtl();
  else if (more) selwakeup(&gSel);
}

// The controller was reset or powered off (BtService.inc transport): forget a half-reassembled controller packet.
void controllerReset() {
  if (!gEnabled) return;
  IOLockLock(gLock);
  bts_h4_reset(&gShim->ctrl);
  IOLockUnlock(gLock);
}

// ---- kernel controls com.apple.uart.bluetooth / com.apple.uart.sk.bluetooth (guamp-bt-ctl=1) ----------------------
bool kctlMine(kern_ctl_ref ref, uint32_t unit) { return gViaKctl && gKctlRef == ref && gKctlUnit == unit; }

// Controller bytes waiting in the shim FIFO go to the connected socket, no more than its receive buffer has room for;
// the rest waits for ctl_rcvd (the client read) or the next controller bytes. Called without gLock (ctl_enqueuedata
// takes the socket lock; the ctl callbacks run with it dropped and take gLock, so gLock is never held around it).
void pumpCtl() {
  uint8_t buf[1024];
  IOLockLock(gPumpLock);
  for (;;) {
    IOLockLock(gLock);
    const bool on = gViaKctl && bts_shim_readable(gShim) > 0;
    kern_ctl_ref ref = gKctlRef;
    const uint32_t unit = gKctlUnit;
    IOLockUnlock(gLock);
    if (!on) break;
    size_t space = 0;
    if (ctl_getenqueuespace(ref, unit, &space) != 0) break;   // unit gone: the disconnect callback follows
    if (!space) { gKctlFull = gKctlFull + 1; break; }
    IOLockLock(gLock);
    const size_t n = kctlMine(ref, unit) ? bts_ctl_take(gShim, buf, sizeof buf, space) : 0;
    IOLockUnlock(gLock);
    if (!n) break;
    if (ctl_enqueuedata(ref, unit, buf, n, 0) != 0) { gKctlEnqErrs = gKctlEnqErrs + 1; break; }
    gKctlRxBytes = gKctlRxBytes + n;
  }
  IOLockUnlock(gPumpLock);
}

errno_t kctlConnect(kern_ctl_ref ref, struct sockaddr_ctl *sac, void **unitinfo) {
  const uint32_t which = ref == gKctlRefs[1] ? 1 : 0;
  *unitinfo = nullptr;
  IOLockLock(gLock);
  // One client: the tty or one control; bluetoothd's sk control takes the slot over from BlueTool (bts_ctl_admit).
  const bts_ctl_admit_result admit = bts_ctl_admit(gOpen, gViaKctl, gKctlWhich, which);
  if (admit == BTS_CTL_REFUSE) {
    gKctlBusy = gKctlBusy + 1;
    const bool tty = !gViaKctl;
    IOLockUnlock(gLock);
    IOLog("GuampBtShim: connect %s unit %u refused: %s already open\n", kKctlNames[which], sac->sc_unit,
          tty ? "the tty" : kKctlNames[gKctlWhich]);
    return EBUSY;
  }
  if (admit == BTS_CTL_PREEMPT) {
    const uint32_t oldUnit = gKctlUnit, oldWhich = gKctlWhich;
    const uint16_t cmd = gShim->lastCmd, reply = gShim->lastReply;
    const uint8_t status = gShim->lastReplyStatus;
    const size_t pending = bts_shim_readable(gShim);
    gKctlPreempts = gKctlPreempts + 1;
    IOLog("GuampBtShim: %s unit %u takes over from %s unit %u (still open; last cmd %04x, last reply %04x status %02x, "
          "%u bytes unread dropped)\n", kKctlNames[which], sac->sc_unit, kKctlNames[oldWhich], oldUnit, cmd, reply,
          status, static_cast<unsigned>(pending));
  }
  gOpen = true;
  gViaKctl = true;
  gKctlRef = ref;
  gKctlUnit = sac->sc_unit;
  gKctlWhich = which;
  gKctlConnects = gKctlConnects + 1;
  bts_tty_init(&gTty);
  bts_shim_flush_rx(gShim);
  bts_shim_flush_tx(gShim);
  const bool linked = gShim->link.tx != nullptr;
  IOLockUnlock(gLock);
  IOLog("GuampBtShim: connect %s unit %u (%u), link %s\n", kKctlNames[which], sac->sc_unit, gKctlConnects,
        linked ? "attached" : "NOT attached: commands fail");
  return 0;
}

errno_t kctlDisconnect(kern_ctl_ref ref, u_int32_t unit, void *unitinfo) {
  IOLockLock(gLock);
  const bool mine = kctlMine(ref, unit);
  if (mine) { gViaKctl = false; gOpen = false; gKctlRef = nullptr; }
  const bts_stats st = gShim->st;
  const uint32_t baud = gShim->baud;
  const uint64_t speed = gTty.speed;
  const uint16_t cmd = gShim->lastCmd, reply = gShim->lastReply;
  const uint8_t status = gShim->lastReplyStatus;
  uint32_t ring[BTS_VENDOR_RING];
  for (unsigned i = 0; i < BTS_VENDOR_RING; i = i + 1) ring[i] = gShim->vendorRing[i];
  const uint32_t vendors = gShim->vendorCount;
  IOLockUnlock(gLock);
  if (mine && vendors) {   // the last vendor commands, oldest first: opcode/first parameter byte:status answered
    char line[BTS_VENDOR_RING * 12 + 1];
    unsigned at = 0;
    const uint32_t n = vendors < BTS_VENDOR_RING ? vendors : BTS_VENDOR_RING;
    for (uint32_t i = vendors - n; i < vendors; i = i + 1) {
      const uint32_t e = ring[i % BTS_VENDOR_RING];
      at += static_cast<unsigned>(snprintf(line + at, sizeof line - at, " %04x/%02x:%02x", e >> 16, (e >> 8) & 0xff,
                                           e & 0xff));
    }
    line[sizeof line - 1] = 0;
    IOLog("GuampBtShim: %u vendor commands (%u unknown), last (opcode/first byte:status):%s\n", vendors,
          st.unknownVendor, line);
  }
  if (mine)
    IOLog("GuampBtShim: disconnect %s unit %u; host %u pkts (%u cmds, %u forwarded, %u emulated, %u tx errors), "
          "controller %u pkts; socket %llu bytes in %llu out, %u full %u lost; baud %u, termios speed %llu; last cmd "
          "%04x, last reply %04x status %02x\n", kKctlNames[gKctlWhich], unit, st.hostPkts, st.hostCmds, st.forwarded,
          st.emulated, st.txErrors, st.ctrlPkts, static_cast<unsigned long long>(gKctlTxBytes),
          static_cast<unsigned long long>(gKctlRxBytes), gKctlFull, gKctlEnqErrs, baud,
          static_cast<unsigned long long>(speed), cmd, reply, status);
  else
    IOLog("GuampBtShim: disconnect %s unit %u (refused or taken over)\n", kKctlNames[ref == gKctlRefs[1] ? 1 : 0],
          unit);
  return 0;
}

// write()/send() of the client: one record, possibly an mbuf chain; the H4 reassembly of the shim takes any split.
errno_t kctlSend(kern_ctl_ref ref, u_int32_t unit, void *unitinfo, mbuf_t m, int flags) {
  bool more = false;
  IOLockLock(gLock);
  const bool mine = kctlMine(ref, unit);
  if (mine) {
    gKctlSends = gKctlSends + 1;
    for (mbuf_t n = m; n; n = mbuf_next(n)) {
      const size_t len = mbuf_len(n);
      if (!len) continue;
      const size_t before = bts_shim_readable(gShim);
      bts_shim_host_write(gShim, static_cast<const uint8_t *>(mbuf_data(n)), len);
      gKctlTxBytes = gKctlTxBytes + len;
      more = more || bts_shim_readable(gShim) > before;   // synthetic replies (vendor commands, tx errors)
    }
  }
  IOLockUnlock(gLock);
  mbuf_freem(m);   // the send callback owns the chain
  if (more) pumpCtl();
  return mine ? 0 : ENOTCONN;
}

errno_t kctlOption(kern_ctl_ref ref, uint32_t unit, int opt, bool set, void *data, size_t *len) {
  const size_t given = len ? *len : 0;
  IOLockLock(gLock);
  if (!kctlMine(ref, unit)) { IOLockUnlock(gLock); return ENOTCONN; }
  int flush = 0;
  const bts_ctl_opt_result r = bts_ctl_sockopt(&gTty, static_cast<uint32_t>(opt), set, static_cast<uint8_t *>(data),
                                               len, bts_shim_readable(gShim), &flush);
  if (r == BTS_CTL_OPT_FLUSH) {
    if (flush & 1) bts_shim_flush_rx(gShim);
    if (flush & 2) bts_shim_flush_tx(gShim);
  }
  const bool refused = r == BTS_CTL_OPT_NOPROTO || r == BTS_CTL_OPT_INVAL;
  gKctlOpts = gKctlOpts + 1;
  if (refused) gKctlOptErrs = gKctlOptErrs + 1;
  const uint32_t count = gKctlOpts;
  const uint64_t speed = gTty.speed;
  IOLockUnlock(gLock);
  if (count <= 32 || refused) {   // the first 32 options since boot (BlueTool, then bluetoothd) and every refusal
    const char *what = r == BTS_CTL_OPT_NOPROTO ? "ENOPROTOOPT" : r == BTS_CTL_OPT_INVAL ? "EINVAL" : "ok";
    IOLog("GuampBtShim: %s %s %#x (%zu bytes): %s, speed %llu\n", kKctlNames[gKctlWhich],
          set ? "setsockopt" : "getsockopt", static_cast<unsigned>(opt), given, what,
          static_cast<unsigned long long>(speed));
  }
  return r == BTS_CTL_OPT_NOPROTO ? ENOPROTOOPT : r == BTS_CTL_OPT_INVAL ? EINVAL : 0;
}

errno_t kctlSetopt(kern_ctl_ref ref, u_int32_t unit, void *unitinfo, int opt, void *data, size_t len) {
  return kctlOption(ref, unit, opt, true, data, &len);
}

errno_t kctlGetopt(kern_ctl_ref ref, u_int32_t unit, void *unitinfo, int opt, void *data, size_t *len) {
  return kctlOption(ref, unit, opt, false, data, len);
}

void kctlRcvd(kern_ctl_ref ref, u_int32_t unit, void *unitinfo, int flags) { pumpCtl(); }

void registerCtls() {
  gPumpLock = IOLockAlloc();
  if (!gPumpLock) { IOLog("GuampBtShim: kernel control lock allocation failed\n"); return; }
  for (unsigned i = 0; i < 2; i = i + 1) {
    struct kern_ctl_reg reg;
    bts_zero(&reg, sizeof reg);
    strlcpy(reg.ctl_name, kKctlNames[i], sizeof reg.ctl_name);
    reg.ctl_flags = CTL_FLAG_REG_SOCK_STREAM | CTL_FLAG_REG_EXTENDED;   // Apple's: SOCK_STREAM; EXTENDED for ctl_rcvd
    reg.ctl_recvsize = 64 * 1024;
    reg.ctl_connect = kctlConnect;
    reg.ctl_disconnect = kctlDisconnect;
    reg.ctl_send = kctlSend;
    reg.ctl_setopt = kctlSetopt;
    reg.ctl_getopt = kctlGetopt;
    reg.ctl_rcvd = kctlRcvd;
    const errno_t e = ctl_register(&reg, &gKctlRefs[i]);
    if (!e) gKctlRegs = gKctlRegs + 1;
    IOLog("GuampBtShim: kernel control %s id %u: %s %d\n", kKctlNames[i], reg.ctl_id,
          e ? "ctl_register error" : "registered", e);
  }
}

void start() {
  unsigned on = guamp_bt_default(1);
  PE_parse_boot_argn("guamp-bt-shim", &on, sizeof(on));
  if (on != 1) return;
  gLock = IOLockAlloc();
  gShim = static_cast<bts_shim *>(IOMallocZero(sizeof(bts_shim)));
  if (!gLock || !gShim) { IOLog("GuampBtShim: allocation failed\n"); return; }
  bts_shim_init(gShim, nullptr, nullptr);
  unsigned bd = guamp_bt_default(1);
  PE_parse_boot_argn("guamp-bt-shim-bdaddr", &bd, sizeof(bd));
  gShim->id.translateBdaddr = bd == 1;
  bts_tty_init(&gTty);
  gMajor = cdevsw_add(-1, &kSw);
  gCtlMajor = cdevsw_add(-1, &kCtlSw);
  if (gMajor < 0 || gCtlMajor < 0) { IOLog("GuampBtShim: cdevsw_add failed %d/%d\n", gMajor, gCtlMajor); return; }
  const int kq = cdevsw_setkqueueok(gMajor, &kSw, 0);
  unsigned nodes = 0;
  for (int m = 0; m < 2; m = m + 1)
    nodes += devfs_make_node(makedev(gMajor, m), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "%s", kTtyNames[m]) ? 1 : 0;
  for (int m = 0; m < 3; m = m + 1)
    nodes += devfs_make_node(makedev(gCtlMajor, m), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "%s", kCtlNames[m]) ? 1 : 0;
  gNodes = nodes;
  unsigned ctl = guamp_bt_default(1);
  PE_parse_boot_argn("guamp-bt-ctl", &ctl, sizeof(ctl));
  if (ctl == 1) registerCtls();
  gEnabled = nodes != 0 || gKctlRegs != 0;
  IOLog("GuampBtShim: tty major %d (kqueue %d), control major %d, %u/5 nodes; identity %s manufacturer %u subver "
        "%04x, fc01->fc14 %u (%u bytes state)\n", gMajor, kq, gCtlMajor, nodes, gShim->id.name,
        gShim->id.manufacturer, gShim->id.lmpSubver, gShim->id.translateBdaddr ? 1u : 0u,
        static_cast<unsigned>(sizeof(bts_shim)));
}

}  // namespace btshim
