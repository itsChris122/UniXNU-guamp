#pragma once
#include <sys/types.h>
#define O_RDONLY 0x0000
#define O_WRONLY 0x0001
#define O_NONBLOCK 0x0004
#define O_CLOEXEC 0x01000000
int open(const char *, int, ...);
