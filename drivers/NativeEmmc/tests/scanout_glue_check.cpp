// -Werror syntax/codegen check of GuampScanout.inc on the arm64-apple-ios target with stand-ins for the IOKit/XNU
// declarations it uses (no XNU build tree on this host). Signatures follow the XNU headers.
#include <stdint.h>
#include <stddef.h>
extern "C" {
void IOLog(const char *format, ...) __attribute__((format(printf, 1, 2)));
int PE_parse_boot_argn(const char *arg_string, void *arg_ptr, int max_arg);
void IODelay(unsigned us);
void IOSleep(unsigned ms);
}
typedef struct _IOLock IOLock;
IOLock *IOLockAlloc(void);
void IOLockLock(IOLock *);
void IOLockUnlock(IOLock *);
void *IOMalloc(size_t);
void IOFree(void *, size_t);
typedef unsigned int vm_size_t_32;
typedef size_t vm_size_t;
typedef uint32_t IOOptionBits;
typedef uint64_t IOPhysicalAddress64, IOByteCount64, mach_vm_address_t;
enum IODirection { kIODirectionNone = 0, kIODirectionIn = 1, kIODirectionOut = 2, kIODirectionInOut = 3 };
enum { kIOMapInhibitCache = 0x00000100, kIOMapWriteCombineCache = 0x00000400, kIOMapReadOnly = 0x00001000 };
class IOMemoryMap { public: mach_vm_address_t getVirtualAddress(); void release() const; };
class IOMemoryDescriptor {
public:
  static IOMemoryDescriptor *withPhysicalAddress(IOPhysicalAddress64, IOByteCount64, IODirection);
  IOMemoryMap *map(IOOptionBits options = 0);
  void release() const;
};
#define OSSafeReleaseNULL(inst) do { if (inst) (inst)->release(); (inst) = nullptr; } while (0)
typedef void *thread_call_param_t;
typedef struct thread_call *thread_call_t;
typedef void (*thread_call_func_t)(thread_call_param_t, thread_call_param_t);
typedef enum { THREAD_CALL_PRIORITY_HIGH = 0, THREAD_CALL_PRIORITY_KERNEL = 1, THREAD_CALL_PRIORITY_KERNEL_HIGH = 5 } thread_call_priority_t;
extern "C" thread_call_t thread_call_allocate_with_options(thread_call_func_t, thread_call_param_t, thread_call_priority_t, uint32_t);
extern "C" bool thread_call_enter_delayed(thread_call_t, uint64_t);
extern "C" void clock_interval_to_deadline(uint32_t, uint32_t, uint64_t *);
#define kMillisecondScale 1000000
volatile uint8_t *mapPhysical(uint64_t physical, uint64_t length);
#include "../GuampBacklightCore.h"
#include "../GuampScanout.inc"
// The NativeEmmc call sites.
void flushSite() { gscan::restore(); }
void logSite() { gscan::log(); }
