#pragma once
#include <sys/types.h>
#define F_OK 0
int access(const char *, int);
int close(int);
ssize_t write(int, const void *, size_t);
