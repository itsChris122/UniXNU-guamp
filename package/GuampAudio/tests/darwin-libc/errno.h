#pragma once
extern int *__error(void);
#define errno (*__error())
#define EPERM 1
#define ENOENT 2
#define EBUSY 16
#define EINVAL 22
#define ENOTTY 25
#define EAGAIN 35
#define EPROTO 100
