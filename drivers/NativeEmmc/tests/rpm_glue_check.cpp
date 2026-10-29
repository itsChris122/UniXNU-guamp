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
// The stand-ins of bt_glue_check.cpp (BT transport shim, IORegistry for guamp-bt-addr) and gpu_glue_check.cpp (QA 29/10:
// BtService.inc includes IOKit/IODeviceTreeSupport.h unless GUAMP_BT_HOST_CHECK is set, since b149107).
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
#include "../GuampRpm.inc"
#include "../BtService.inc"
#include "../GpuService.inc"
void use() {
  rpm::start();
  bt::start();
  gpum1::start();
  rpm::set(RPM_RES_CLK0, RPM_CLK0_XO, RPM_KEY_ENAB, 1);
}
