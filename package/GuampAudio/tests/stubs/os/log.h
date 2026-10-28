#pragma once
typedef struct os_log_s *os_log_t; os_log_t os_log_create(const char *, const char *);
void os_log_impl(os_log_t, const char *, ...);
#define os_log(log, fmt, ...) os_log_impl(log, fmt, ##__VA_ARGS__)
