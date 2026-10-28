#pragma once
typedef struct { long __sig; char __opaque[56]; } pthread_mutex_t;
#define PTHREAD_MUTEX_INITIALIZER {0x32AAABA7, {0}}
int pthread_mutex_lock(pthread_mutex_t *);
int pthread_mutex_unlock(pthread_mutex_t *);
