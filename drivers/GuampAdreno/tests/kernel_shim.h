// SPDX-License-Identifier: BSD-2-Clause-Patent
// Authored stand-ins (not XNU headers) for the -Werror syntax check of GuampAdrenoProbe.cpp on arm64-apple-ios
// (tests/run-tests.sh): every kernel header the probe and the NativeAdreno headers include resolves to this file.
#ifndef GUAMP_ADRENO_KERNEL_SHIM_H
#define GUAMP_ADRENO_KERNEL_SHIM_H
#include <stddef.h>
#include <stdint.h>
extern "C" {
void *memcpy(void *, const void *, size_t);
void *memset(void *, int, size_t);
int memcmp(const void *, const void *, size_t);
size_t strlen(const char *);
}
typedef int IOReturn;
typedef int kern_return_t;
typedef unsigned IOOptionBits;
typedef unsigned IODirection;
typedef size_t IOByteCount;
typedef uint64_t IOPhysicalAddress;
typedef uint64_t addr64_t;
typedef uint64_t mach_vm_address_t;
typedef uintptr_t vm_offset_t;
typedef unsigned ppnum_t;
typedef int boolean_t;
typedef void *event_t;
typedef struct task *task_t;
typedef uint32_t UInt32;
typedef int wait_result_t;
typedef int (*thread_continue_t)(void);
enum : IOReturn {kIOReturnSuccess = 0, kIOReturnError = 0x2bc, kIOReturnNoMemory, kIOReturnNoResources, kIOReturnBadArgument = 0x2c2,
                 kIOReturnUnsupported = 0x2c7, kIOReturnNotReady = 0x2d8, kIOReturnNotFound = 0x2f0, kIOReturnBusy = 0x2d5,
                 kIOReturnTimeout = 0x2d6, kIOReturnIOError = 0x2ca, kIOReturnInvalid = 0x2c1, kIOReturnNotAttached = 0x2c4,
                 kIOReturnInternalError = 0x2c9};
enum : IOOptionBits {kIOMapAnywhere = 1, kIOMapInhibitCache = 0x100, kIOMapWriteCombineCache = 0x400,
                     kIODirectionInOut = 3, kIOMemoryPhysicallyContiguous = 0x10};
enum {THREAD_UNINT = 0, KERN_SUCCESS = 0};
#define NSEC_PER_USEC 1000ull
#define THREAD_CONTINUE_NULL ((thread_continue_t)0)
extern task_t kernel_task;
extern "C" {
void IOLog(const char *, ...) __attribute__((format(printf, 1, 2)));
void IODelay(unsigned);
void IOSleep(unsigned);
int PE_parse_boot_argn(const char *, void *, int);
uint64_t mach_absolute_time(void);
void absolutetime_to_nanoseconds(uint64_t, uint64_t *);
void thread_wakeup(event_t);
int assert_wait_timeout(event_t, int, uint32_t, uint64_t);
int thread_block(thread_continue_t);
// guamp-adreno-zap GFSR snapshot thread (kern/thread.h KPI, the real continuation type)
typedef struct thread *thread_t;
kern_return_t kernel_thread_start(void (*)(void *, wait_result_t), void *, thread_t *);
void thread_deallocate(thread_t);
}
#define IONewData(type, count) static_cast<type *>(nullptr)
#define IODeleteData(ptr, type, count) do {(ptr) = nullptr;} while (0)
struct OSNoRetainTag {};
constexpr OSNoRetainTag OSNoRetain{};
template <class T> class OSSharedPtr {
  T *p_ = nullptr;
public:
  OSSharedPtr() = default;
  OSSharedPtr(T *p, OSNoRetainTag) : p_(p) {}
  T *operator->() const {return p_;}
  explicit operator bool() const {return p_ != nullptr;}
};
class OSObject {
public:
  virtual ~OSObject() = default;
  void release() const;
  void retain() const;
  virtual void free();
};
#define OSSafeReleaseNULL(p) do {if (p) (p)->release(); (p) = nullptr;} while (0)
// NativeAdrenoPublish.cpp (patched; tests/run-tests.sh syntax check): the libkern/IOKit pieces it uses, as plain declarations.
#define OSDeclareDefaultStructors(c)
#define OSDefineMetaClassAndStructors(c, s)
#define APPLE_KEXT_OVERRIDE override
#define OSDynamicCast(type, inst) (static_cast<type *>(inst))
class OSDictionary : public OSObject {};
class OSBoolean : public OSObject {};
extern OSBoolean *const kOSBooleanTrue;
class OSSymbol : public OSObject {
public:
  static const OSSymbol *withCString(const char *);
};
class OSMetaClass {
public:
  static const OSMetaClass *getMetaClassWithName(const OSSymbol *);
  unsigned getClassSize() const;
  static void *preModLoad(const char *);
  static IOReturn postModLoad(void *);
};
class IOMemoryMap : public OSObject {
public:
  uint64_t getVirtualAddress();
};
struct IOAddressRange {uint64_t address, length;};
enum : IOOptionBits {kIOMemoryTypePhysical64 = 0x50};
class IOMemoryDescriptor : public OSObject {
public:
  IOMemoryMap *map(IOOptionBits);
  IOMemoryMap *createMappingInTask(task_t, uint64_t, IOOptionBits);
  static IOMemoryDescriptor *withAddressRanges(IOAddressRange *, UInt32, IOOptionBits, task_t);
};
class IOBufferMemoryDescriptor : public IOMemoryDescriptor {
public:
  static IOBufferMemoryDescriptor *inTaskWithOptions(task_t, IOOptionBits, size_t, size_t);
  void *getBytesNoCopy();
};
class IODeviceMemory : public IOMemoryDescriptor {
public:
  static IODeviceMemory *withRange(uint64_t, uint64_t);
};
class IOService : public OSObject {
public:
  virtual bool init();
  bool setProperty(const char *, const char *);
  bool setProperty(const char *, OSObject *);
  bool attach(IOService *);
  void detach(IOService *);
  virtual bool start(IOService *);
  bool terminate(IOOptionBits = 0);
  void setName(const char *);
  void registerService();
  static IOService *getPlatform();
  virtual IOReturn newUserClient(task_t, void *, UInt32, OSDictionary *, class IOUserClient **);
};
struct IOExternalMethodArguments {
  uint32_t version, selector;
  const void *structureInput;
  uint32_t structureInputSize;
  void *structureOutput;
  uint32_t structureOutputSize;
};
enum {kIOExternalMethodArgumentsCurrentVersion = 2};
typedef IOReturn (*IOExternalMethodAction)(OSObject *, void *, IOExternalMethodArguments *);
struct IOExternalMethodDispatch {
  IOExternalMethodAction function;
  uint32_t checkScalarInputCount, checkStructureInputSize, checkScalarOutputCount, checkStructureOutputSize;
};
class IOUserClient : public IOService {
public:
  virtual bool initWithTask(task_t, void *, UInt32, OSDictionary *);
  virtual IOReturn externalMethod(uint32_t, IOExternalMethodArguments *, IOExternalMethodDispatch *, OSObject *, void *);
  virtual IOReturn clientMemoryForType(UInt32, IOOptionBits *, IOMemoryDescriptor **);
  virtual IOReturn clientClose();
  IOMemoryMap *removeMappingForDescriptor(IOMemoryDescriptor *);
};
typedef void *io_filter_policy_t;
struct io_filter_callbacks {
  IOReturn (*io_filter_resolver)(task_t, IOUserClient *, unsigned, io_filter_policy_t *);
  void (*io_filter_release)(io_filter_policy_t);
};
typedef struct IOLock IOLock;
extern "C" {
IOLock *IOLockAlloc(void);
void IOLockLock(IOLock *);
void IOLockUnlock(IOLock *);
int IOLockSleep(IOLock *, void *, UInt32);
void IOLockWakeup(IOLock *, void *, bool);
thread_t current_thread(void);
void thread_set_thread_name(thread_t, const char *);
task_t current_task(void);
size_t strlcpy(char *, const char *, size_t);
int strncmp(const char *, const char *, size_t);
int strcmp(const char *, const char *);
}
#define IONewZero(type, count) static_cast<type *>(nullptr)
#define IODelete(ptr, type, count) do {(ptr) = nullptr;} while (0)
enum : IOReturn {kIOReturnNotPermitted = 0x2e2, kIOReturnNoSpace = 0x2c4 + 0x100};
struct kmod_info_t {char name[64];};
#endif
