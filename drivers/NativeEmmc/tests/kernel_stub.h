// Minimal stand-ins for the XNU/IOKit declarations GuampWifiPrep.inc uses: a -Werror syntax and codegen check of the
// kernel glue on the arm64-apple-ios target (no XNU build tree on this host). Signatures follow the XNU headers.
#include <stdint.h>
#include <stddef.h>
extern "C" {
void *memcpy(void *, const void *, size_t);
void *memset(void *, int, size_t);
int memcmp(const void *, const void *, size_t);
int strcmp(const char *, const char *);
size_t strlen(const char *);
void IOLog(const char *format, ...) __attribute__((format(printf, 1, 2)));
int PE_parse_boot_argn(const char *arg_string, void *arg_ptr, int max_arg);
void IOSleep(unsigned int);   // C linkage as in IOKit/IOLib.h (the other glue checks declare it extern "C" too)
}
typedef struct _IOLock IOLock;
IOLock *IOLockAlloc(void);
void IOLockLock(IOLock *);
void IOLockUnlock(IOLock *);
void *IOMalloc(size_t);
void *IOMallocZero(size_t);
void IOFree(void *, size_t);
typedef int wait_result_t;
typedef struct thread *thread_t;
typedef int kern_return_t;
#define KERN_SUCCESS 0
typedef void (*thread_continue_t)(void *, wait_result_t);
kern_return_t kernel_thread_start(thread_continue_t, void *, thread_t *);
void thread_deallocate(thread_t);
thread_t current_thread(void);
void thread_set_thread_name(thread_t, const char *);
uint64_t mach_absolute_time(void);
void absolutetime_to_nanoseconds(uint64_t, uint64_t *);
typedef uint32_t IOOptionBits;
typedef uint64_t IOPhysicalAddress64, IOByteCount64, mach_vm_address_t;
enum IODirection { kIODirectionNone = 0, kIODirectionIn = 1, kIODirectionOut = 2, kIODirectionInOut = 3 };
enum { kIOMapInhibitCache = 0x00000100, kIOMapReadOnly = 0x00001000 };
class IOMemoryMap { public: mach_vm_address_t getVirtualAddress(); void release() const; };
class IOMemoryDescriptor {
public:
  static IOMemoryDescriptor *withPhysicalAddress(IOPhysicalAddress64, IOByteCount64, IODirection);
  IOMemoryMap *map(IOOptionBits options = 0);
  void release() const;
};
constexpr uint32_t kBlock = 512, kBounceBlocks = 128;
struct Card { IOLock *gate; bool online; uint64_t capacity; bool readBlocks(uint64_t lba, uint32_t count, uint8_t *out); };
extern Card gCard;
