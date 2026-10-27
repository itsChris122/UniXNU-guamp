// -Werror syntax and codegen check of the Adreno 610 M1/M2 kernel glue (GpuService.inc) on the arm64-apple-ios target,
// with the stand-ins of kernel_stub.h plus the few calls it adds.
#include "kernel_stub.h"
extern "C" {
void IODelay(unsigned);
void IOSleep(unsigned);
}
volatile uint8_t *mapPhysical(uint64_t physical, uint64_t length);
typedef struct task *task_t;
extern task_t kernel_task;
enum { kIOMemoryPhysicallyContiguous = 0x00000010, kIOMemoryMapperNone = 0x00000800 };
class IOBufferMemoryDescriptor {
public:
  static IOBufferMemoryDescriptor *inTaskWithPhysicalMask(task_t, IOOptionBits, uint64_t capacity, uint64_t mask);
  void *getBytesNoCopy();
  uint64_t getPhysicalSegment(uint64_t offset, uint64_t *length, IOOptionBits options);
  void release() const;
};
#include "../GpuService.inc"
void use() { gpum1::start(); }
