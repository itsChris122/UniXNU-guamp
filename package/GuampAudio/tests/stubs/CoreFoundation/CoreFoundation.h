#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>
typedef unsigned char Boolean; typedef uint32_t UInt32; typedef int32_t SInt32; typedef uint64_t UInt64; typedef double Float64;
typedef float Float32; typedef SInt32 OSStatus; typedef const struct __CFString *CFStringRef; typedef const struct __CFUUID *CFUUIDRef;
typedef const struct __CFAllocator *CFAllocatorRef; typedef const struct __CFDictionary *CFDictionaryRef; typedef const void *CFTypeRef;
typedef CFTypeRef CFPropertyListRef;
typedef struct { uint8_t b[16]; } CFUUIDBytes;
CFStringRef __CFStringMakeConstantString(const char *);
#define CFSTR(s) ((CFStringRef)(const void *)("" s))
CFTypeRef CFRetain(CFTypeRef); void CFRelease(CFTypeRef); Boolean CFEqual(CFTypeRef, CFTypeRef);
CFUUIDRef CFUUIDGetConstantUUIDWithBytes(CFAllocatorRef, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t,
  uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
CFUUIDRef CFUUIDCreateFromUUIDBytes(CFAllocatorRef, CFUUIDBytes);
