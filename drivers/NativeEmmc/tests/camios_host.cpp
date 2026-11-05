// Host test of GuampCamIos.inc (the AppleH6CamIn service and its user client): stand-ins for the IOKit/XNU calls it
// uses (locks and threads on pthreads, the ring on malloc), then the user-client sequence the H6ISP plug-in runs:
// GetInfo, Open, SetMode, map, Start, WaitFrame/ReleaseFrame, SetExposure (the test pattern gets brighter), Stop, Close.
// Also built with --target=arm64-apple-ios17 -fsyntax-only by run-tests.sh (the kernel target), same stand-ins.
#include <stdint.h>
#include <stddef.h>
#ifndef CAMIOS_SYNTAX_ONLY
#include <string.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <stdarg.h>
#include <initializer_list>
#else
extern "C" {
int printf(const char *, ...);
void *memset(void *, int, size_t);
void *memcpy(void *, const void *, size_t);
void *memmove(void *, const void *, size_t);
int strcmp(const char *, const char *);
}
#endif

// ---- stand-ins (signatures follow the XNU headers) ------------------------------------------------------------------
typedef int IOReturn;
typedef uint32_t UInt32;
typedef uint32_t IOOptionBits;
typedef int wait_result_t;
typedef int kern_return_t;
typedef void *thread_call_param_t;
#define KERN_SUCCESS 0
#define THREAD_AWAKENED 0
#define THREAD_TIMED_OUT 1
#define THREAD_ABORTSAFE 2
#define kIOReturnSuccess 0
#define kIOReturnError ((IOReturn)0xe00002bc)
#define kIOReturnNoMemory ((IOReturn)0xe00002bd)
#define kIOReturnBadArgument ((IOReturn)0xe00002c2)
#define kIOReturnUnsupported ((IOReturn)0xe00002c7)
#define kIOReturnNotPrivileged ((IOReturn)0xe00002c1)
#define kIOReturnBusy ((IOReturn)0xe00002d5)
#define kIOReturnTimeout ((IOReturn)0xe00002d6)
#define kIOReturnNotReady ((IOReturn)0xe00002d8)
#define kIOReturnNotOpen ((IOReturn)0xe00002cd)
#define kIOReturnExclusiveAccess ((IOReturn)0xe00002c5)
#define kIOMapReadOnly 0x00001000
#define kIODirectionInOut 3
#define kIOMemoryKernelUserShared 0x00010000
#define kMillisecondScale 1000000
#define kIOClientPrivilegeAdministrator "root"
#define kIOUserClientClassKey "IOUserClientClass"
#define APPLE_KEXT_OVERRIDE override
typedef struct task *task_t;
typedef struct thread *thread_t;
typedef struct thread_call *thread_call_t;
typedef void (*thread_continue_t)(void *, wait_result_t);
typedef void (*thread_call_func_t)(thread_call_param_t, thread_call_param_t);
extern task_t kernel_task;
extern "C" {
void IOLog(const char *format, ...) __attribute__((format(printf, 1, 2)));
int PE_parse_boot_argn(const char *arg_string, void *arg_ptr, int max_arg);
uint64_t mach_absolute_time(void);
void clock_interval_to_deadline(uint32_t interval, uint32_t scale_factor, uint64_t *result);
kern_return_t kernel_thread_start(thread_continue_t, void *, thread_t *);
void thread_deallocate(thread_t);
thread_call_t thread_call_allocate(thread_call_func_t, thread_call_param_t);
bool thread_call_enter_delayed(thread_call_t, uint64_t);
void proc_selfname(char *buf, int size);
void nanoseconds_to_absolutetime(uint64_t ns, uint64_t *abs);
task_t current_task(void);
}
void IOSleep(unsigned ms);
struct IOLock;
IOLock *IOLockAlloc(void);
void IOLockLock(IOLock *);
void IOLockUnlock(IOLock *);
int IOLockSleepDeadline(IOLock *, void *event, uint64_t deadline, int interType);
void IOLockWakeup(IOLock *, void *event, bool oneThread);
extern unsigned long page_size;
#define INT32_MIN_ (-2147483647 - 1)
#define round_page(x) ((((uint64_t)(x)) + 4095) & ~4095ull)
class OSObject { public: virtual ~OSObject() {} int refs = 1; void retain() { ++refs; } void release() { if (--refs == 0) delete this; } };
class OSDictionary : public OSObject {};
class IOMemoryDescriptor : public OSObject {};
class IOBufferMemoryDescriptor : public IOMemoryDescriptor {
public:
  void *bytes = nullptr; size_t length = 0;
  ~IOBufferMemoryDescriptor() override;
  static IOBufferMemoryDescriptor *inTaskWithOptions(task_t, IOOptionBits, size_t capacity, size_t alignment);
  void *getBytesNoCopy() { return bytes; }
};
class IOService : public OSObject {
public:
  virtual bool init(OSDictionary * = nullptr) { return true; }
  bool setProperty(const char *, unsigned long long, unsigned) { return true; }
  bool setProperty(const char *, const char *) { return true; }
  void setName(const char *) {}
  bool attach(IOService *) { return true; }
  void detach(IOService *) {}
  virtual bool start(IOService *) { return true; }
  void registerService() {}
  bool isInactive() { return false; }
  bool terminate() { return true; }
  static IOService *getPlatform();
  virtual IOReturn newUserClient(task_t, void *, UInt32, OSDictionary *, class IOUserClient **) { return kIOReturnUnsupported; }
};
struct IOExternalMethodArguments {
  const uint64_t *scalarInput; uint32_t scalarInputCount;
  void *structureOutput; uint32_t structureOutputSize;
};
struct IOExternalMethodDispatch;
class IOUserClient : public IOService {
public:
  virtual bool initWithTask(task_t, void *, UInt32) { return true; }
  virtual IOReturn clientClose() { return kIOReturnSuccess; }
  virtual IOReturn clientMemoryForType(UInt32, IOOptionBits *, IOMemoryDescriptor **) { return kIOReturnUnsupported; }
  virtual IOReturn externalMethod(uint32_t, IOExternalMethodArguments *, IOExternalMethodDispatch *, OSObject *, void *) {
    return kIOReturnUnsupported;
  }
  static IOReturn clientHasPrivilege(void *, const char *);
};
#define OSDeclareDefaultStructors(name) public: name(); static name *alloc();
#define OSDefineMetaClassAndStructors(name, super) name::name() {} name *name::alloc() { return new name; }
#define OSTypeAlloc(type) type::alloc()
#define OSSafeReleaseNULL(p) do { if (p) (p)->release(); (p) = nullptr; } while (0)
extern "C" void guamp_torch_request(uint32_t permille);

#include "../GuampCamIos.inc"

#ifndef CAMIOS_SYNTAX_ONLY
// ---- stand-in implementations on pthreads --------------------------------------------------------------------------
task_t kernel_task;
unsigned long page_size = 4096;
#if defined(CAMIOS_ARG_MODE)
static uint32_t gArgMode = CAMIOS_ARG_MODE;   // campipe_host.cpp (integration skeleton) sets it
#elif defined(CAMIOS_FAKE_CAMSS)
static uint32_t gArgMode = 1;
#else
static uint32_t gArgMode = 2;
#endif
static char gProcName[32] = "mediaserverd";
static uint32_t gTorch = 9999;
extern "C" {
void IOLog(const char *format, ...) { va_list ap; va_start(ap, format); vprintf(format, ap); va_end(ap); }
int PE_parse_boot_argn(const char *a, void *p, int n) {
  if (!strcmp(a, "guamp-camera-ios") && n >= 4) { memcpy(p, &gArgMode, 4); return 1; }
  return 0;
}
uint64_t mach_absolute_time(void) { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (uint64_t)t.tv_sec * 1000000000ull + t.tv_nsec; }
void clock_interval_to_deadline(uint32_t i, uint32_t s, uint64_t *r) { *r = mach_absolute_time() + (uint64_t)i * s; }
struct Tramp { thread_continue_t fn; void *arg; };
static void *tramp(void *p) { Tramp t = *(Tramp *)p; free(p); t.fn(t.arg, 0); return nullptr; }
kern_return_t kernel_thread_start(thread_continue_t fn, void *arg, thread_t *out) {
  pthread_t th; Tramp *t = (Tramp *)malloc(sizeof(Tramp)); t->fn = fn; t->arg = arg;
  if (pthread_create(&th, nullptr, tramp, t)) return 1;
  pthread_detach(th); *out = (thread_t)1; return KERN_SUCCESS;
}
void thread_deallocate(thread_t) {}
thread_call_t thread_call_allocate(thread_call_func_t, thread_call_param_t) { return (thread_call_t)1; }
bool thread_call_enter_delayed(thread_call_t, uint64_t) { return true; }
void proc_selfname(char *buf, int size) { snprintf(buf, size, "%s", gProcName); }
void nanoseconds_to_absolutetime(uint64_t ns, uint64_t *abs) { *abs = ns; }
task_t current_task(void) { return nullptr; }
void guamp_torch_request(uint32_t permille) { gTorch = permille; }
}
void IOSleep(unsigned ms) { usleep(ms * 1000); }
struct IOLock { pthread_mutex_t m; pthread_cond_t c; };
IOLock *IOLockAlloc(void) { IOLock *l = new IOLock; pthread_mutex_init(&l->m, nullptr); pthread_cond_init(&l->c, nullptr); return l; }
void IOLockLock(IOLock *l) { pthread_mutex_lock(&l->m); }
void IOLockUnlock(IOLock *l) { pthread_mutex_unlock(&l->m); }
int IOLockSleepDeadline(IOLock *l, void *, uint64_t deadline, int) {
  timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  uint64_t now = (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
  if (deadline <= now) return THREAD_TIMED_OUT;
  timespec abs; clock_gettime(CLOCK_REALTIME, &abs);
  uint64_t t = (uint64_t)abs.tv_sec * 1000000000ull + abs.tv_nsec + (deadline - now);
  abs.tv_sec = t / 1000000000ull; abs.tv_nsec = t % 1000000000ull;
  return pthread_cond_timedwait(&l->c, &l->m, &abs) == ETIMEDOUT ? THREAD_TIMED_OUT : THREAD_AWAKENED;
}
void IOLockWakeup(IOLock *l, void *, bool) { pthread_cond_broadcast(&l->c); }
IOBufferMemoryDescriptor::~IOBufferMemoryDescriptor() { free(bytes); }
IOBufferMemoryDescriptor *IOBufferMemoryDescriptor::inTaskWithOptions(task_t, IOOptionBits, size_t cap, size_t) {
  IOBufferMemoryDescriptor *d = new IOBufferMemoryDescriptor; d->bytes = calloc(1, cap); d->length = cap; return d;
}
#ifdef CAMIOS_FAKE_CAMSS
// Fake CAMSS ABI 2: 4 buffers of 2000x1500 RAW10, one frame every 10 ms into a buffer nobody holds.
static IOBufferMemoryDescriptor *gCamssBuf[4];
static bool gCamssHeld[4], gCamssOpen, gCamssStreaming;
static uint32_t gCamssSeq, gCamssReleases, gCamssExposures;
static pthread_mutex_t gCamssLock = PTHREAD_MUTEX_INITIALIZER;
extern "C" {
int guamp_camss_open(const struct guamp_camss_config *c) {
  if (c->abi != GUAMP_CAMSS_ABI || c->source != GUAMP_CAMSS_SRC_SENSOR || c->buffers > 4) return GUAMP_CAMSS_EINVAL;
  for (int i = 0; i < 4; ++i) gCamssBuf[i] = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, 0, 2500 * 1500, 4096);
  gCamssOpen = true; return 0;
}
int guamp_camss_start(uint32_t) { gCamssStreaming = true; return 0; }
int guamp_camss_wait_frame(uint32_t after, uint32_t, struct guamp_camss_frame *f) {
  usleep(10000);
  pthread_mutex_lock(&gCamssLock);
  int b = -1;
  for (int i = 0; i < 4 && b < 0; ++i) if (!gCamssHeld[(gCamssSeq + i) % 4]) b = (gCamssSeq + i) % 4;
  if (b < 0 || !gCamssStreaming) { pthread_mutex_unlock(&gCamssLock); return GUAMP_CAMSS_ETIMEDOUT; }
  gCamssHeld[b] = true;
  *f = {}; f->abi = GUAMP_CAMSS_ABI; f->seq = ++gCamssSeq; f->buffer = b; f->format = GUAMP_CAMSS_FMT_RAW10_MIPI;
  f->width = 2000; f->height = 1500; f->line_bytes = 2500; f->bytes = 2500 * 1500; f->bayer = GUAMP_CAMSS_GRBG;
  f->sof_ns = mach_absolute_time(); f->cpu = gCamssBuf[b]->bytes;
  pthread_mutex_unlock(&gCamssLock);
  return after < f->seq ? 0 : GUAMP_CAMSS_EINVAL;
}
void guamp_camss_release(uint32_t b) { pthread_mutex_lock(&gCamssLock); gCamssHeld[b] = false; ++gCamssReleases; pthread_mutex_unlock(&gCamssLock); }
int guamp_camss_stop(void) { gCamssStreaming = false; return 0; }
void guamp_camss_close(void) { for (auto &b : gCamssBuf) { b->release(); b = nullptr; } gCamssOpen = false; }
void *guamp_camss_buffer_memory(uint32_t b) { return b < 4 ? gCamssBuf[b] : nullptr; }
int guamp_camios_hw_exposure(uint32_t, uint32_t us, uint32_t q8, uint32_t *aus, uint32_t *aq8) { ++gCamssExposures; *aus = us; *aq8 = q8; return 0; }
}
#endif
static IOService gPlatform;
IOService *IOService::getPlatform() { return &gPlatform; }
IOReturn IOUserClient::clientHasPrivilege(void *, const char *) { return kIOReturnNotPrivileged; }

[[maybe_unused]] static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

static IOReturn call(IOUserClient *uc, uint32_t sel, std::initializer_list<uint64_t> in, void *out = nullptr, uint32_t outSize = 0) {
  uint64_t v[8]; uint32_t n = 0;
  for (uint64_t x : in) v[n++] = x;
  IOExternalMethodArguments a = {v, n, out, outSize};
  return uc->externalMethod(sel, &a, nullptr, nullptr, nullptr);
}

// Mean of the red sites of the white bar (left eighth) of a RAW16 RGGB pattern frame.
[[maybe_unused]] static double whiteRed(const uint8_t *slot, const GuampCamIosModeInfo &m) {
  double sum = 0; unsigned n = 0;
  for (uint32_t y = 0; y < m.height; y += 2)
    for (uint32_t x = 0; x < m.width / 8; x += 2) { sum += ((const uint16_t *)(slot + y * m.strideBytes))[x]; ++n; }
  return sum / n;
}

#if defined(CAMIOS_NO_MAIN)
// included by campipe_host.cpp: stand-ins only
#elif !defined(CAMIOS_FAKE_CAMSS)
int main() {
  camios::start();
  CHECK(camios::gService != nullptr);
  CHECK(camios::gSensors == 2);
  AppleH6CamIn *svc = static_cast<AppleH6CamIn *>(camios::gService);
  IOUserClient *uc = nullptr;
  CHECK(svc->newUserClient(nullptr, nullptr, 0, nullptr, &uc) == kIOReturnBadArgument);       // wrong type
  strcpy(gProcName, "SpringBoard");
  CHECK(svc->newUserClient(nullptr, nullptr, GUAMP_CAMIOS_CONNECT_TYPE, nullptr, &uc) == kIOReturnNotPrivileged);
  strcpy(gProcName, "mediaserverd");
  CHECK(svc->newUserClient(nullptr, nullptr, GUAMP_CAMIOS_CONNECT_TYPE, nullptr, &uc) == kIOReturnSuccess && uc);
  if (!uc) return 1;

  GuampCamIosInfo info = {};
  CHECK(call(uc, kGuampCamSelGetInfo, {}, &info, sizeof(info)) == kIOReturnSuccess);
  CHECK(info.abiVersion == GUAMP_CAMIOS_ABI_VERSION && info.sensorCount == 2);
  CHECK(info.sensor[0].position == kGuampCamBack && info.sensor[1].position == kGuampCamFront && info.sensor[0].testPattern);

  CHECK(call(uc, kGuampCamSelStart, {0}) == kIOReturnNotOpen);
  CHECK(call(uc, kGuampCamSelOpen, {0}) == kIOReturnSuccess);
  CHECK(call(uc, kGuampCamSelOpen, {0}) == kIOReturnExclusiveAccess);
  GuampCamIosModeInfo mode = {};
  CHECK(call(uc, kGuampCamSelSetMode, {0, kGuampModePreview, 60u << 8, 15u << 8}, &mode, sizeof(mode)) == kIOReturnSuccess);
  CHECK(mode.width == 1280 && mode.height == 960 && mode.rawFormat == kGuampRawU16 && mode.slotCount == 4);
  CHECK(mode.slotStride % 4096 == 0 && mode.slotStride >= mode.strideBytes * mode.height);
  IOOptionBits opts = 0; IOMemoryDescriptor *mem = nullptr;
  for (uint32_t i = 0; i < mode.slotCount; ++i)
    CHECK(uc->clientMemoryForType(kGuampCamMemRing + i, &opts, &mem) == kIOReturnSuccess && (opts & kIOMapReadOnly));
  CHECK(uc->clientMemoryForType(kGuampCamMemRing + mode.slotCount, &opts, &mem) == kIOReturnBadArgument);   // no such slot
  CHECK(uc->clientMemoryForType(kGuampCamMemRing + kGuampCamMemSlotsPerSensor, &opts, &mem) == kIOReturnBadArgument);   // front not opened
  CHECK(call(uc, kGuampCamSelStart, {0}) == kIOReturnSuccess);

  GuampCamIosFrame f = {};
  double dark = 0, bright = 0;
  uint64_t lastSeq = 0;
  for (int i = 0; i < 12; ++i) {
    if (i == 6) CHECK(call(uc, kGuampCamSelSetExposure, {0, 20000, 512, 256}) == kIOReturnSuccess);   // x4 light
    CHECK(call(uc, kGuampCamSelWaitFrame, {0, 500}, &f, sizeof(f)) == kIOReturnSuccess);
    CHECK(f.slot < mode.slotCount && f.width == mode.width && (f.flags & 4u));
    if (i) CHECK(f.sequence > lastSeq);
    lastSeq = f.sequence;
    const double r = whiteRed((const uint8_t *)camios::gSensor[0].mem[f.slot]->getBytesNoCopy(), mode);
    if (i == 4) dark = r;
    if (i == 11) bright = r;
    CHECK(call(uc, kGuampCamSelReleaseFrame, {0, f.slot}) == kIOReturnSuccess);
  }
  printf("white bar red: %.0f at 10 ms x1, %.0f at 20 ms x2 (black 64)\n", dark, bright);
  CHECK(dark > 600 && dark < 900 && bright > 950);

  // A client that falls behind: the oldest ready frames are recycled, delivery goes on.
  usleep(400 * 1000);
  CHECK(call(uc, kGuampCamSelWaitFrame, {0, 500}, &f, sizeof(f)) == kIOReturnSuccess);
  CHECK(call(uc, kGuampCamSelReleaseFrame, {0, f.slot}) == kIOReturnSuccess);
  CHECK(camios::gSensor[0].dropped > 0);

  CHECK(call(uc, kGuampCamSelSetFlash, {kGuampFlashTorch, 300, 0}) == kIOReturnSuccess && gTorch == 300);
  CHECK(call(uc, kGuampCamSelSetFlash, {kGuampFlashOff, 0, 0}) == kIOReturnSuccess && gTorch == 0);
  CHECK(call(uc, kGuampCamSelSetFocus, {0, 5000}) == kIOReturnSuccess && camios::gSensor[0].focusDac == 1023);

  CHECK(call(uc, kGuampCamSelStop, {0}) == kIOReturnSuccess);
  CHECK(call(uc, kGuampCamSelWaitFrame, {0, 100}, &f, sizeof(f)) != kIOReturnSuccess || true);   // drains at most what was queued
  CHECK(call(uc, kGuampCamSelSetMode, {0, kGuampModePhoto, 30u << 8, 15u << 8}, &mode, sizeof(mode)) == kIOReturnSuccess);
  CHECK(mode.width == 2016 && mode.height == 1512);
  CHECK(call(uc, kGuampCamSelStart, {0}) == kIOReturnSuccess);
  CHECK(call(uc, kGuampCamSelWaitFrame, {0, 500}, &f, sizeof(f)) == kIOReturnSuccess && f.width == 2016);
  uc->clientClose();                                   // closes the sensor, stops the generator, frees the ring
  CHECK(!camios::gSensor[0].open && !camios::gSensor[0].mem[0] && !camios::pattern::gRun[0].alive);
  printf(failures ? "camios host: %d FAILURES\n" : "camios host: all checks passed\n", failures);
  return failures ? 1 : 0;
}
#else
int main() {
  camios::start();
  CHECK(camios::gBackend == &camios::camss::kBackend && camios::gSensors == 2);
  AppleH6CamIn *svc = static_cast<AppleH6CamIn *>(camios::gService);
  IOUserClient *uc = nullptr;
  CHECK(svc->newUserClient(nullptr, nullptr, GUAMP_CAMIOS_CONNECT_TYPE, nullptr, &uc) == kIOReturnSuccess && uc);
  if (!uc) return 1;
  CHECK(call(uc, kGuampCamSelOpen, {0}) == kIOReturnSuccess);
  CHECK(call(uc, kGuampCamSelOpen, {1}) == kIOReturnExclusiveAccess);           // one CAMSS stream at a time
  GuampCamIosModeInfo mode = {};
  CHECK(call(uc, kGuampCamSelSetMode, {0, kGuampModePreview, 30u << 8, 15u << 8}, &mode, sizeof(mode)) == kIOReturnSuccess);
  CHECK(gCamssOpen && mode.rawFormat == kGuampRawMipi10 && mode.slotCount == 4 && camios::gSensor[0].mem[2] == gCamssBuf[2]);
  CHECK(call(uc, kGuampCamSelStart, {0}) == kIOReturnSuccess);
  GuampCamIosFrame f = {};
  uint64_t last = 0;
  for (int i = 0; i < 20; ++i) {
    CHECK(call(uc, kGuampCamSelWaitFrame, {0, 500}, &f, sizeof(f)) == kIOReturnSuccess);
    CHECK(f.width == 2000 && f.strideBytes == 2500 && f.bayer == kGuampBayerGRBG && f.rawFormat == kGuampRawMipi10);
    CHECK(f.sequence > last); last = f.sequence;
    if (i == 10) usleep(100 * 1000);                                           // fall behind: the pump recycles
    CHECK(call(uc, kGuampCamSelReleaseFrame, {0, f.slot}) == kIOReturnSuccess);
  }
  CHECK(call(uc, kGuampCamSelSetExposure, {0, 20000, 512, 256}) == kIOReturnSuccess && gCamssExposures == 1);
  printf("camss: %u frames made, %u releases, %llu delivered, %llu recycled\n", gCamssSeq, gCamssReleases,
         (unsigned long long)camios::gSensor[0].delivered, (unsigned long long)camios::gSensor[0].dropped);
  CHECK(camios::gSensor[0].dropped > 0 && gCamssReleases >= 20);
  uc->clientClose();
  CHECK(!gCamssOpen && !camios::gSensor[0].open);
  printf(failures ? "camios camss: %d FAILURES\n" : "camios camss: all checks passed\n", failures);
  return failures ? 1 : 0;
}
#endif
#endif
