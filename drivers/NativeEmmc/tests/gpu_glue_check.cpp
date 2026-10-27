// -Werror syntax and codegen check of the Adreno 610 M1 kernel glue (GpuService.inc) on the arm64-apple-ios target, with
// the stand-ins of kernel_stub.h plus the few calls it adds.
#include "kernel_stub.h"
extern "C" {
void IODelay(unsigned);
void IOSleep(unsigned);
}
volatile uint8_t *mapPhysical(uint64_t physical, uint64_t length);
#include "../GpuService.inc"
void use() { gpum1::start(); }
