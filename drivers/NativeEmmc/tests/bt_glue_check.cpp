// -Werror syntax and codegen check of the Bluetooth M0 kernel glue (BtService.inc) on the arm64-apple-ios target, with
// the stand-ins of kernel_stub.h plus the few calls it adds. The pmic:: symbols are declared by BtService.inc itself.
#include "kernel_stub.h"
extern "C" {
void IODelay(unsigned);
void IOSleep(unsigned);
}
volatile uint8_t *mapPhysical(uint64_t physical, uint64_t length);
#include "../BtService.inc"
void use() { bt::start(); }
