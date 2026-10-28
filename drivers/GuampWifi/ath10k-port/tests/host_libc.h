/* Host-test declarations; the kernel libc shim intentionally omits these. */
#pragma once
#include <stddef.h>
int printf(const char *,...);
char *getenv(const char *);
void abort(void) __attribute__((noreturn));
void *malloc(size_t);
void *calloc(size_t,size_t);
void free(void *);
