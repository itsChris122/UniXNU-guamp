// -Werror syntax and codegen check of the ADSP kernel glue (GuampAdsp.inc with GuampPas.inc, GuampScm.inc and the SMC
// wrapper) in the include order of NativeEmmc.cpp (GuampRpm.inc, GuampWifiPrep.inc, then GuampAdsp.inc), on the
// arm64-apple-ios target with the stand-ins of kernel_stub.h plus the calls they add. The assembly must contain smc.
#include "kernel_stub.h"
extern "C" {
void IODelay(unsigned);
void IOSleep(unsigned);
}
IOLock *IOLockAlloc(void);
volatile uint8_t *mapPhysical(uint64_t physical, uint64_t length);
#include "../GuampRpm.inc"
#include "../GuampWifiPrep.inc"
#include "../GuampAdsp.inc"
void use() {
  rpm::start();
  wifiprep::start();
  adsp::start();
}
