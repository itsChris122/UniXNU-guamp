// -Werror syntax and codegen check of GuampBtSerial.inc on the arm64-apple-ios target, with stand-ins for the BSD
// declarations it uses (signatures as in xnu bsd/sys/conf.h, uio.h, select.h, miscfs/devfs/devfs.h, IOKit/IOLocks.h).
// The real header set is only exercised by the first kernel build that includes the file.
#include "kernel_stub.h"
#define GUAMP_BT_SERIAL_HOST_CHECK 1
typedef int32_t dev_t;
typedef unsigned long u_long;
typedef char *caddr_t;
typedef int64_t user_ssize_t;
typedef uint32_t uid_t, gid_t;
struct uio;
struct proc;
struct tty;
struct buf;
struct selinfo { uint8_t storage[64]; void *note; unsigned flags; };
typedef int open_close_fcn_t(dev_t dev, int flags, int devtype, struct proc *p);
typedef int read_write_fcn_t(dev_t dev, struct uio *uio, int ioflag);
typedef int ioctl_fcn_t(dev_t dev, u_long cmd, caddr_t data, int fflag, struct proc *p);
typedef int stop_fcn_t(struct tty *tp, int rw);
typedef int reset_fcn_t(int uban);
typedef int select_fcn_t(dev_t dev, int which, void *wql, struct proc *p);
typedef int mmap_fcn_t(void);
typedef void strategy_fcn_t(struct buf *bp);
typedef int rsvd_fcn_t(void);
struct cdevsw {
  open_close_fcn_t *d_open; open_close_fcn_t *d_close; read_write_fcn_t *d_read; read_write_fcn_t *d_write;
  ioctl_fcn_t *d_ioctl; stop_fcn_t *d_stop; reset_fcn_t *d_reset; struct tty **d_ttys; select_fcn_t *d_select;
  mmap_fcn_t *d_mmap; strategy_fcn_t *d_strategy; rsvd_fcn_t *d_reserved_1; rsvd_fcn_t *d_reserved_2; int d_type;
};
extern "C" {
int cdevsw_add(int, const struct cdevsw *);
int cdevsw_setkqueueok(int, const struct cdevsw *, int);
void *devfs_make_node(dev_t dev, int chrblk, uid_t uid, gid_t gid, int perms, const char *fmt, ...)
    __attribute__((format(printf, 6, 7)));
void selrecord(struct proc *selector, struct selinfo *, void *);
void selwakeup(struct selinfo *);
void selthreadclear(struct selinfo *);
int uiomove(const char *cp, int n, struct uio *uio);
user_ssize_t uio_resid(struct uio *a_uio);
}
#define minor(x) ((int32_t)((x) & 0xffffff))
#define makedev(x, y) ((dev_t)(((x) << 24) | (y)))
#define ENXIO 6
#define ENODEV 19
#define EBUSY 16
#define EINVAL 22
#define ENOTTY 25
#define EWOULDBLOCK 35
#define EINTR 4
#define FREAD 0x0001
#define IO_NDELAY 0x0010
#define DEVFS_CHAR 0
#define UID_ROOT 0
#define GID_WHEEL 0
#define THREAD_AWAKENED 0
#define THREAD_ABORTSAFE 2
IOLock *IOLockAlloc(void);
int IOLockSleep(IOLock *lock, void *event, uint32_t interType);
void IOLockWakeup(IOLock *lock, void *event, bool oneThread);
#include "../GuampBtSerial.inc"
void use() {
  btshim::start();
  static const uint8_t evt[] = {4, 0x0e, 4, 1, 3, 0x0c, 0};
  btshim::controllerBytes(evt, sizeof evt);
  bts_link l = {};
  btshim::attach(&l);
}
