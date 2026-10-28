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
#include "GuampBtShim.h"
#ifndef GUAMP_BT_SERIAL_HOST_CHECK
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/select.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/vnode.h>
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

void controllerBytes(const uint8_t *data, size_t len) {
  if (!gEnabled || !len) return;
  IOLockLock(gLock);
  const size_t before = bts_shim_readable(gShim);
  bts_shim_ctrl_input(gShim, data, len);
  const bool more = gOpen && bts_shim_readable(gShim) > before;
  if (!gOpen) bts_shim_flush_rx(gShim);   // nobody reads: do not let stale events greet the next open
  if (more) IOLockWakeup(gLock, &gShim->rx, false);
  IOLockUnlock(gLock);
  if (more) selwakeup(&gSel);
}

// The controller was reset or powered off (BtService.inc transport): forget a half-reassembled controller packet.
void controllerReset() {
  if (!gEnabled) return;
  IOLockLock(gLock);
  bts_h4_reset(&gShim->ctrl);
  IOLockUnlock(gLock);
}

void start() {
  unsigned on = 0;
  PE_parse_boot_argn("guamp-bt-shim", &on, sizeof(on));
  if (on != 1) return;
  gLock = IOLockAlloc();
  gShim = static_cast<bts_shim *>(IOMallocZero(sizeof(bts_shim)));
  if (!gLock || !gShim) { IOLog("GuampBtShim: allocation failed\n"); return; }
  bts_shim_init(gShim, nullptr, nullptr);
  unsigned bd = 0;
  if (PE_parse_boot_argn("guamp-bt-shim-bdaddr", &bd, sizeof(bd))) gShim->id.translateBdaddr = bd == 1;
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
  gEnabled = nodes != 0;
  IOLog("GuampBtShim: tty major %d (kqueue %d), control major %d, %u/5 nodes; identity %s manufacturer %u subver "
        "%04x, fc01->fc14 %u (%u bytes state)\n", gMajor, kq, gCtlMajor, nodes, gShim->id.name,
        gShim->id.manufacturer, gShim->id.lmpSubver, gShim->id.translateBdaddr ? 1u : 0u,
        static_cast<unsigned>(sizeof(bts_shim)));
}

}  // namespace btshim
