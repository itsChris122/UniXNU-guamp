// -Werror syntax and codegen check of the RPM client kernel glue (GuampRpm.inc) and of its two users, in the include
// order of NativeEmmc.cpp (GuampRpm.inc first, so the rpm:: declarations in BtService.inc / GpuService.inc must match
// the definitions), on the arm64-apple-ios target with the stand-ins of kernel_stub.h plus the calls they add.
#include "kernel_stub.h"
extern "C" {
void IODelay(unsigned);
void IOSleep(unsigned);
}
IOLock *IOLockAlloc(void);
volatile uint8_t *mapPhysical(uint64_t physical, uint64_t length);
#include "../GuampRpm.inc"
#include "../BtService.inc"
#include "../GpuService.inc"
void use() {
  rpm::start();
  bt::start();
  gpum1::start();
  rpm::set(RPM_RES_CLK0, RPM_CLK0_XO, RPM_KEY_ENAB, 1);
}
