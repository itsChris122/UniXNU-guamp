// -Werror syntax and codegen check of the Bluetooth M0 kernel glue (BtService.inc) on the arm64-apple-ios target, with
// the stand-ins of kernel_stub.h plus the few calls it adds. The pmic:: symbols are declared by BtService.inc itself.
#include "kernel_stub.h"
extern "C" {
void IODelay(unsigned);
void IOSleep(unsigned);
}
volatile uint8_t *mapPhysical(uint64_t physical, uint64_t length);
// The transport (step 5) uses the shim interface of GuampBtSerial.inc (checked by run-bt-shim-tests.sh) and a few IOKit
// calls: stand-ins with the kernel signatures.
#define GUAMP_BT_HOST_CHECK 1
#include "../GuampBtShim.h"
namespace btshim {
extern bool gEnabled;
extern uint32_t gOpens, gReads, gWrites, gIoctls, gNodes, gCtlOpens[3];
extern uint32_t gKctlRegs, gKctlConnects, gKctlBusy, gKctlSends, gKctlOpts, gKctlOptErrs, gKctlFull;
extern uint64_t gKctlTxBytes, gKctlRxBytes;
void start();
void attach(const bts_link *link);
void controllerBytes(const uint8_t *data, size_t len);
void controllerReset();
}
IOLock *IOLockAlloc(void);
class OSObject { public: void release() const; };
class OSData : public OSObject { public: static OSData *withBytes(const void *bytes, unsigned numBytes); };
class IORegistryPlane;
extern const IORegistryPlane *gIODTPlane;
class IORegistryEntry : public OSObject {
public:
  static IORegistryEntry *fromPath(const char *path, const IORegistryPlane *plane = nullptr, char *residualPath = nullptr,
                                   int *residualLength = nullptr, IORegistryEntry *fromEntry = nullptr);
  bool setProperty(const char *aKey, OSObject *anObject);
};
#include "../BtService.inc"
void use() { bt::start(); }
