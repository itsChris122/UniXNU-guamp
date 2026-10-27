// -Werror syntax/codegen check of GuampTorch.inc on the arm64-apple-ios target with stand-ins for the IOKit/XNU
// declarations and the PmicService.inc helpers it uses (no XNU build tree on this host). Signatures follow the XNU headers.
#include <stdint.h>
#include <stddef.h>
extern "C" {
void IOLog(const char *format, ...) __attribute__((format(printf, 1, 2)));
int PE_parse_boot_argn(const char *arg_string, void *arg_ptr, int max_arg);
uint64_t mach_absolute_time(void);
void absolutetime_to_nanoseconds(uint64_t, uint64_t *);
}
void IOSleep(unsigned ms);
void IODelay(unsigned us);
typedef int IOReturn;
#define kIOReturnSuccess 0
#define kIOReturnBadArgument ((IOReturn)0xe00002c2)
#define kIOReturnUnsupported ((IOReturn)0xe00002c7)
#define kIOReturnNotPrivileged ((IOReturn)0xe00002c1)
typedef struct task *task_t;
task_t current_task(void);
#define kIOClientPrivilegeAdministrator "root"
class OSObject { public: virtual ~OSObject(); void release() const; };
class OSNumber : public OSObject { public: uint32_t unsigned32BitValue() const; };
class OSBoolean : public OSObject { public: bool isTrue() const; };
extern OSBoolean *const kOSBooleanTrue, *const kOSBooleanFalse;
class OSDictionary : public OSObject { public: OSObject *getObject(const char *) const; };
template <class T> T *OSDynamicCastImpl(OSObject *o);
#define OSDynamicCast(type, inst) OSDynamicCastImpl<type>(inst)
class IOService : public OSObject {
public:
  virtual bool init(OSDictionary *dict = nullptr);
  virtual IOReturn setProperties(OSObject *properties);
  bool setProperty(const char *key, unsigned long long number, unsigned bits);
  bool setProperty(const char *key, OSObject *object);
  void setName(const char *);
  bool attach(IOService *);
  void detach(IOService *);
  virtual bool start(IOService *);
  void registerService();
  static IOService *getPlatform();
};
class IOUserClient { public: static IOReturn clientHasPrivilege(void *securityToken, const char *privilegeName); };
#define OSDeclareDefaultStructors(name) public: name(); static name *alloc();
#define OSDefineMetaClassAndStructors(name, super) name::name() {} name *name::alloc() { return new name; }
#define OSTypeAlloc(type) type::alloc()
#include "../GuampPmicBoard.h"
#include "../GuampSpmi.h"
namespace pmic {
struct spmi_arb gArb;
int apidOf(unsigned sid, uint16_t addr);
int read8(unsigned sid, uint16_t addr, uint8_t *v);
int write8(unsigned sid, uint16_t addr, uint8_t v);
constexpr unsigned kPmi632 = 2;   // PmicService.inc board constant
#include "../GuampTorch.inc"
}  // namespace pmic
void use(unsigned tick) {
  if (pmic::torch::init()) pmic::torch::poll(tick);
  pmic::torch::request(500);
  pmic::torch::shutdown(true);
}
