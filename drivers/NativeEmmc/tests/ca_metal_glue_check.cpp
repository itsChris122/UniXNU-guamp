// -Werror syntax and codegen check of the guamp-ca-metal watchdog glue (GuampCaMetal.inc) on the arm64-apple-ios target,
// with the stand-ins of kernel_stub.h plus the BSD/VFS calls it adds (signatures follow the XNU headers).
#include "kernel_stub.h"
#define ENOMEM 12
#define O_WRONLY 0x0001
#define O_CREAT 0x0200
#define O_TRUNC 0x0400
#define O_NOFOLLOW 0x0100
#define FWRITE 0x0002
#define IO_SYNC 0x0004
#define IO_NOAUTH 0x8000
#define SIGKILL 9
#define NULLVP nullptr
extern "C" {
int snprintf(char *, size_t, const char *, ...);
void IOSleep(unsigned);
typedef struct vnode *vnode_t;
typedef struct vfs_context *vfs_context_t;
typedef struct ucred *kauth_cred_t;
typedef char *caddr_t;
typedef int64_t off_t;
enum uio_rw { UIO_READ, UIO_WRITE };
enum uio_seg { UIO_USERSPACE = 0, UIO_SYSSPACE = 2 };
vfs_context_t vfs_context_create(vfs_context_t);
int vfs_context_rele(vfs_context_t);
kauth_cred_t vfs_context_ucred(vfs_context_t);
struct proc *vfs_context_proc(vfs_context_t);
int vnode_open(const char *, int, int, int, vnode_t *, vfs_context_t);
int vnode_close(vnode_t, int, vfs_context_t);
int vn_rdwr(enum uio_rw, struct vnode *, caddr_t, int, off_t, enum uio_seg, int, kauth_cred_t, int *, struct proc *);
}
#define GUAMP_CA_METAL_HOST_CHECK 1
#include "../GuampCaMetal.inc"
void use() { camtl::start(); }
