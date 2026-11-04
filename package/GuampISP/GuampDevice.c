// GuampDevice.c - FigCaptureDevice of the guamp plug-in. Entry point H6ISPCaptureDeviceCreate (dlsym'd by CMCapture's
// FigCaptureGetCaptureDeviceCreateFunction once IOServiceMatching("AppleH6CamIn") finds the kernel service).
//
// Device properties (H9ISP _deviceProperties, same keys): SupportedPropertiesDictionary, Clock, StreamArray,
// FirmwareVersion, SupportedSynchronizedStreamsGroups (read); MemoryPoolID, DARTCacheParameters, DiagnosticsLogDump,
// SensorAgileClockingEnable, FactoryFeaturesMode, FastISPClockEnable (write, accepted and ignored here).
#include <dlfcn.h>
#include <os/log.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "GuampISP.h"

GFigApi gFig;

void GuampLog(const char *fmt, ...) {
  static os_log_t log;
  if (!log) log = os_log_create("org.guamp.camera", "isp");
  char line[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  os_log(log, "%{public}s", line);
}

static CFStringRef lookupKey(const char *symbol) {
  CFStringRef *slot = (CFStringRef *)dlsym(RTLD_DEFAULT, symbol);
  return slot ? *slot : NULL;
}

int GFigApiLoad(GFigApi *api) {
  if (api->derivedObjectCreate) return 0;
  api->derivedObjectCreate = dlsym(RTLD_DEFAULT, "CMDerivedObjectCreate");
  api->getDerivedStorage = dlsym(RTLD_DEFAULT, "CMBaseObjectGetDerivedStorage");
  api->deviceClassID = dlsym(RTLD_DEFAULT, "FigCaptureDeviceGetClassID");
  api->streamClassID = dlsym(RTLD_DEFAULT, "FigCaptureStreamGetClassID");
  api->lookupKey = lookupKey;
  if (!api->derivedObjectCreate || !api->getDerivedStorage || !api->deviceClassID || !api->streamClassID) {
    GISP_LOG("missing CoreMedia/CMCapture symbol (create %p storage %p dev %p stream %p)", api->derivedObjectCreate,
             api->getDerivedStorage, api->deviceClassID, api->streamClassID);
    memset(api, 0, sizeof(*api));
    return -1;
  }
  return 0;
}

static inline GDevice *deviceOf(GBaseObjectRef o) { return (GDevice *)gFig.getDerivedStorage(o); }

static Boolean keyIs(CFStringRef key, CFStringRef literal) { return key && CFEqual(key, literal); }

// The property attribute dictionary H9ISP builds for SupportedPropertiesDictionary (see GuampStream.c, gPropertyAttrs).
CFDictionaryRef GCopySupportedProperties(const char *const *keys, const char *const *types, const uint8_t *rw, size_t n);

static const char *const kDevKeys[] = {"Clock", "StreamArray", "FirmwareVersion", "SupportedSynchronizedStreamsGroups",
                                       "MemoryPoolID", "DARTCacheParameters"};
static const char *const kDevTypes[] = {"Clock", "Collection", "String", "Collection", "Number", "Collection"};
static const uint8_t kDevRw[] = {0, 0, 0, 0, 2, 2};   // 0 read only, 1 read write, 2 write only

static OSStatus devCopyProperty(GBaseObjectRef o, CFStringRef key, CFAllocatorRef alloc, void *valueOut) {
  GDevice *d = deviceOf(o);
  CFTypeRef *out = (CFTypeRef *)valueOut;
  if (!out) return kGErrParam;
  *out = NULL;
  if (d->invalidated) return kGErrInvalidated;
  if (keyIs(key, CFSTR("StreamArray"))) {
    *out = CFArrayCreateCopy(alloc, d->streams);
  } else if (keyIs(key, CFSTR("Clock"))) {
    *out = CFRetain(d->clock);
  } else if (keyIs(key, CFSTR("FirmwareVersion"))) {
    *out = CFStringCreateWithFormat(alloc, NULL, CFSTR("GuampISP %u (%s)"), GUAMP_CAMIOS_ABI_VERSION,
                                    d->kernel.info.sensor[0].testPattern ? "test pattern" : "CAMSS");
  } else if (keyIs(key, CFSTR("SupportedSynchronizedStreamsGroups"))) {
    *out = CFArrayCreate(alloc, NULL, 0, &kCFTypeArrayCallBacks);   // no multi-camera sync on guamp
  } else if (keyIs(key, CFSTR("SupportedPropertiesDictionary"))) {
    *out = GCopySupportedProperties(kDevKeys, kDevTypes, kDevRw, sizeof(kDevRw));
  } else {
    return kGErrPropertyNotFound;
  }
  return *out ? 0 : kGErrAlloc;
}

static OSStatus devSetProperty(GBaseObjectRef o, CFStringRef key, CFTypeRef value) {
  GDevice *d = deviceOf(o);
  if (d->invalidated) return kGErrInvalidated;
  // Pool ids, DART cache, logging and clock tuning only make sense for Apple's ISP: accept them so the vendor proceeds.
  static const CFStringRef ignored[] = {CFSTR("MemoryPoolID"), CFSTR("DARTCacheParameters"), CFSTR("DiagnosticsLogDump"),
                                        CFSTR("SensorAgileClockingEnable"), CFSTR("FactoryFeaturesMode"),
                                        CFSTR("FastISPClockEnable")};
  for (size_t i = 0; i < sizeof(ignored) / sizeof(ignored[0]); ++i)
    if (keyIs(key, ignored[i])) return 0;
  (void)value;
  return kGErrPropertyNotFound;
}

static OSStatus devInvalidate(GBaseObjectRef o) {
  GDevice *d = deviceOf(o);
  if (d->invalidated) return 0;
  d->invalidated = 1;
  CFIndex n = d->streams ? CFArrayGetCount(d->streams) : 0;
  for (CFIndex i = 0; i < n; ++i) {
    GBaseObjectRef s = (GBaseObjectRef)CFArrayGetValueAtIndex(d->streams, i);
    GFramesStop((GStream *)gFig.getDerivedStorage(s));
  }
  GKernelClose(&d->kernel);
  return 0;
}

static void devFinalize(GBaseObjectRef o) {
  GDevice *d = deviceOf(o);
  devInvalidate(o);
  if (d->streams) CFRelease(d->streams);
  if (d->clock) CFRelease(d->clock);
}

static CFStringRef devCopyDebugDescription(GBaseObjectRef o) {
  GDevice *d = deviceOf(o);
  return CFStringCreateWithFormat(NULL, NULL, CFSTR("<GuampISPCaptureDevice %p sensors %u>"), o, d->kernel.info.sensorCount);
}

static OSStatus devCreateProcessingSession(GBaseObjectRef device, CFAllocatorRef alloc, CFDictionaryRef config,
                                           const void **out) {
  // Offline ISP sessions (RAW reprocessing, DRC) need Apple's ISP hardware; CMCapture falls back to software paths.
  (void)device; (void)alloc; (void)config;
  if (out) *out = NULL;
  return kGErrUnsupported;
}

static const GBaseClass kDeviceBase = {
    .version = 1,
    .derivedStorageSize = sizeof(GDevice),
    .invalidate = devInvalidate,
    .finalize = devFinalize,
    .copyDebugDescription = devCopyDebugDescription,
    .copyProperty = devCopyProperty,
    .setProperty = devSetProperty,
};
static const GCaptureDeviceClass kDeviceClass = {.version = 1, .createProcessingSession = devCreateProcessingSession};
static const GDeviceVTable kDeviceVTable = {NULL, &kDeviceBase, &kDeviceClass};

// Same shape as H9ISPCaptureDeviceCreate(allocator, <unused here>, FigCaptureDeviceRef *out): x0 allocator, x2 out.
__attribute__((visibility("default"))) OSStatus H6ISPCaptureDeviceCreate(CFAllocatorRef alloc, CFTypeRef unused,
                                                                          GBaseObjectRef *out) {
  (void)unused;
  if (!out) return kGErrParam;
  *out = NULL;
  if (GFigApiLoad(&gFig)) return kGErrUnsupported;
  GBaseObjectRef obj = NULL;
  OSStatus err = gFig.derivedObjectCreate(alloc, &kDeviceVTable, gFig.deviceClassID(), &obj);
  if (err) return err;
  GDevice *d = deviceOf(obj);
  d->self = obj;
  d->clock = CMClockGetHostTimeClock();
  CFRetain(d->clock);
  d->streams = CFArrayCreateMutable(alloc, 0, &kCFTypeArrayCallBacks);
  if (GKernelOpen(&d->kernel)) {
    GISP_LOG("kernel service AppleH6CamIn not usable");
    CFRelease(obj);
    return kGErrUnsupported;
  }
  // One FigCaptureStream per present back/front sensor, back first (AVCaptureDevice order of the J120 plist).
  for (uint32_t pos = kGuampCamBack; pos <= kGuampCamFront; ++pos) {
    for (uint32_t i = 0; i < d->kernel.info.sensorCount && i < GUAMP_CAMIOS_MAX_SENSORS; ++i) {
      const GuampCamIosSensorInfo *s = &d->kernel.info.sensor[i];
      if (!s->present || s->position != pos) continue;
      GBaseObjectRef stream = NULL;
      if (GStreamCreate(d, i, &stream) == 0 && stream) {
        CFArrayAppendValue(d->streams, stream);
        CFRelease(stream);
      }
    }
  }
  GISP_LOG("device created: %ld stream(s), abi %u", (long)CFArrayGetCount(d->streams), d->kernel.info.abiVersion);
  *out = obj;
  return 0;
}
