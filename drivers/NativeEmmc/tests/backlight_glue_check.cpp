// -Werror syntax/codegen check of GuampBacklight.inc on the arm64-apple-ios target with stand-ins for the IOKit/XNU
// declarations it uses (no XNU build tree on this host). Signatures follow the XNU headers.
#include <stdint.h>
#include <stddef.h>
extern "C" {
void IOLog(const char *format, ...) __attribute__((format(printf, 1, 2)));
int PE_parse_boot_argn(const char *arg_string, void *arg_ptr, int max_arg);
uint64_t mach_absolute_time(void);
void absolutetime_to_nanoseconds(uint64_t, uint64_t *);
void nanoseconds_to_absolutetime(uint64_t, uint64_t *);
void IODelay(unsigned us);
void IOSleep(unsigned ms);
}
void *IOMallocZero(size_t);
typedef int IOReturn;
#define kIOReturnSuccess 0
#define kIOReturnBadArgument ((IOReturn)0xe00002c2)
class OSMetaClass;
class OSMetaClassBase { public: virtual ~OSMetaClassBase(); virtual const OSMetaClass *getMetaClass() const = 0; void release() const; };
class OSObject : public OSMetaClassBase {};
class OSSymbol : public OSObject { public: static const OSSymbol *withCString(const char *); };
class OSMetaClass : public OSMetaClassBase { public: static const OSMetaClass *getMetaClassWithName(const OSSymbol *); };
class OSNumber : public OSObject {
public:
  static OSNumber *withNumber(unsigned long long value, unsigned int numberOfBits);
  uint32_t unsigned32BitValue() const;
};
class OSDictionary : public OSObject {
public:
  static OSDictionary *withCapacity(unsigned int);
  OSObject *getObject(const char *) const;
  bool setObject(const char *, OSMetaClassBase *);
};
template <class T> T *OSDynamicCastImpl(OSObject *o);
#define OSDynamicCast(type, inst) OSDynamicCastImpl<type>(inst)
#define OSSafeReleaseNULL(inst) do { if (inst) (inst)->release(); (inst) = nullptr; } while (0)
class IOTimerEventSource;
class IOEventSource : public OSObject {};
class IOTimerEventSource : public IOEventSource {
public:
  typedef void (*Action)(OSObject *owner, IOTimerEventSource *sender);
  static IOTimerEventSource *timerEventSource(OSObject *owner, Action action = nullptr);
  IOReturn setTimeoutMS(uint32_t ms);
  IOReturn setTimeoutUS(uint32_t us);
  void cancelTimeout();
};
class IOWorkLoop : public OSObject {
public:
  static IOWorkLoop *workLoop();
  IOReturn addEventSource(IOEventSource *);
};
class IOService : public OSObject {
public:
  const OSMetaClass *getMetaClass() const override;
  virtual bool init(OSDictionary *dict = nullptr);
  virtual IOReturn setProperties(OSObject *properties);
  virtual bool start(IOService *provider);
  bool setProperty(const char *key, OSObject *object);
  bool setProperty(const char *key, const char *string);
  bool setProperty(const char *key, bool value);
  bool attach(IOService *);
  void detach(IOService *);
  void registerService();
  static IOService *getPlatform();
};
#define OSDeclareDefaultStructors(name) public: name(); static name *alloc(); private:
#define OSDefineMetaClassAndStructors(name, super) name::name() {} name *name::alloc() { return new name; }
#define OSTypeAlloc(type) type::alloc()
volatile uint8_t *mapPhysical(uint64_t physical, uint64_t length);
#include "../GuampBacklight.inc"
void use() { backlight::start(); }
