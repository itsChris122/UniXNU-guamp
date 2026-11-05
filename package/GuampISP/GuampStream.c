// GuampStream.c - FigCaptureStream of the guamp plug-in: one per camera (back GM1ST, front 4H7, with
// the secondary-camera kernel work also the two OV02B: macro and depth).
//
// The property names are the CFString values of CMCapture's kFigCaptureStreamProperty_* constants, read out of
// H9ISP's _streamProperties table (145 entries, analisi/camera/cam5-servizio-ios.md section 4). Only the ones the
// capture pipeline needs for preview, photo, torch/flash, exposure and focus are implemented; the Apple-ISP-only
// ones are accepted (set) or reported unavailable (copy) so BWFigVideoCaptureStream keeps going.
#include <Block.h>
#include <math.h>
#include <string.h>
#include "GuampISP.h"

static inline GStream *streamOf(GBaseObjectRef o) { return (GStream *)gFig.getDerivedStorage(o); }
static Boolean keyIs(CFStringRef key, const char *literal) {
  // CFSTR needs a literal; compare by content through a stack CFString.
  CFStringRef s = CFStringCreateWithCStringNoCopy(NULL, literal, kCFStringEncodingASCII, kCFAllocatorNull);
  Boolean eq = s && key && CFEqual(key, s);
  if (s) CFRelease(s);
  return eq;
}
static CFNumberRef num32(int32_t v) { return CFNumberCreate(NULL, kCFNumberSInt32Type, &v); }
static CFNumberRef numF(float v) { return CFNumberCreate(NULL, kCFNumberFloat32Type, &v); }
static Boolean getNum(CFTypeRef v, double *out) {
  if (!v) return false;
  if (CFGetTypeID(v) == CFNumberGetTypeID()) return CFNumberGetValue((CFNumberRef)v, kCFNumberDoubleType, out);
  if (CFGetTypeID(v) == CFBooleanGetTypeID()) { *out = CFBooleanGetValue((CFBooleanRef)v); return true; }
  return false;
}

// ---- property table ---------------------------------------------------------------------------------------------
// type: kFigCapturePropertyType_* value; rw: 0 read only, 1 read write, 2 write only (kFigCapturePropertyReadWriteStatus_*)
typedef struct GProp { const char *key, *type; uint8_t rw; } GProp;
static const GProp kStreamProps[] = {
    {"SupportedPropertiesDictionary", "Collection", 0}, {"PortType", "String", 0},
    {"SupportedFormatsArray", "Collection", 0},         {"FormatIndex", "Number", 1},
    {"FormatDescription", "FormatDescription", 0},     {"MinimumFrameRate", "Number", 1},
    {"MaximumFrameRate", "Number", 1},                 {"MinIntegrationTime", "Number", 0},
    {"MaxIntegrationTime", "Number", 1},               {"SensorTemperature", "Number", 0},
    {"ModuleInfo", "Collection", 0},                   {"ZoomFactor", "Number", 1},
    {"MaxZoomFactor", "Number", 0},                    {"FocalLength", "Number", 0},
    {"PixelSize", "Number", 0},                        {"LensFNumber", "Number", 0},
    {"VideoOutputHandlers", "Collection", 2},          {"VideoOutputsEnabled", "Collection", 1},
    {"VideoOutputConfigurations", "Collection", 1},    {"VideoOutputsColorInfo", "Collection", 0},
    {"SupportedColorSpaces", "Collection", 0},         {"ColorSpace", "Number", 1},
    {"StillImageCaptureNow", "Unused", 2},             {"TorchLevel", "Number", 2},
    {"FlashMode", "String", 2},                        {"ExposureBias", "Collection", 2},
    {"AEEnabled", "Boolean", 2},                       {"LockAENow", "Unused", 2},
    {"UnlockAENow", "Unused", 2},                      {"LockAWBNow", "Unused", 2},
    {"ImagerUnlockAWBNow", "Unused", 2},               {"AutoFocusNow", "Unused", 2},
    {"FocusNow", "Collection", 2},                     {"ManualFocusNow", "Collection", 2},
    {"LockAFNow", "Unused", 2},                        {"AutoFocusMode", "String", 1},
    {"AFWindowParams", "Collection", 2},               {"AEWindowParams", "Collection", 2},
    {"ManualAENow", "Collection", 2},                  {"ManualWhiteBalanceGain", "Collection", 2},
    {"WhiteBalanceGain", "Collection", 0},             {"DeferAdditionOfAttachments", "Boolean", 1},
    {"MultipleOutputSupportEnabled", "Boolean", 1},    {"AEMaxGain", "Number", 1},
    {"AEMinGain", "Number", 1},                        {"HasAutoFocusPositionSensor", "Collection", 0},
};

CFDictionaryRef GCopySupportedProperties(const char *const *keys, const char *const *types, const uint8_t *rw, size_t n) {
  // Per property: {PropertyType, ReadWriteStatus, ShouldBeSerialized}, the three keys of H9ISP
  // addPropertyDictionaryToPropertiesDictionary (0x21e4edcf4..0x21e4edd34). Status: ReadOnly / ReadWrite / WriteOnly.
  static const CFStringRef rwName[] = {CFSTR("ReadOnly"), CFSTR("ReadWrite"), CFSTR("WriteOnly")};
  CFMutableDictionaryRef all = CFDictionaryCreateMutable(NULL, (CFIndex)n, &kCFTypeDictionaryKeyCallBacks,
                                                         &kCFTypeDictionaryValueCallBacks);
  for (size_t i = 0; i < n; ++i) {
    CFStringRef k = CFStringCreateWithCString(NULL, keys[i], kCFStringEncodingASCII);
    CFStringRef t = CFStringCreateWithCString(NULL, types[i], kCFStringEncodingASCII);
    const void *ak[] = {CFSTR("PropertyType"), CFSTR("ReadWriteStatus"), CFSTR("ShouldBeSerialized")};
    const void *av[] = {t, rwName[rw[i] % 3], kCFBooleanTrue};
    CFDictionaryRef attrs = CFDictionaryCreate(NULL, ak, av, 3, &kCFTypeDictionaryKeyCallBacks,
                                               &kCFTypeDictionaryValueCallBacks);
    CFDictionarySetValue(all, k, attrs);
    CFRelease(attrs); CFRelease(t); CFRelease(k);
  }
  return all;
}

static CFDictionaryRef copyStreamSupported(void) {
  enum { n = sizeof(kStreamProps) / sizeof(kStreamProps[0]) };
  const char *keys[n], *types[n];
  uint8_t rw[n];
  for (size_t i = 0; i < n; ++i) { keys[i] = kStreamProps[i].key; types[i] = kStreamProps[i].type; rw[i] = kStreamProps[i].rw; }
  return GCopySupportedProperties(keys, types, rw, n);
}

// Accepted and ignored: Apple ISP tuning, face detection, OIS/sphere, depth engine, synchronized streams, RAW reprocessing.
static const char *const kIgnoredSets[] = {
    "TemporalNoiseReductionEnabled", "FaceDetectionEnabled", "LuxAdaptiveParameters", "AWBScheme",
    "AEGainThresholdForFlickerZoneIntegrationTimeTransition", "AEIntegrationTimeForUnityGainToMinGainTransition",
    "FaceDetectionRectangleOfInterest", "MatrixFocusParams", "ClientMaxBufferCountHint", "AttachRAW", "ColorRange",
    "Apply3AWindowSettings", "AEFaceDetectionMetering", "ColorTables", "AEConvergenceSpeed", "AEStability",
    "AFSearchPositions", "VideoRecordingInProgress", "HistogramParams", "AEMeteringMode", "AFEarlyOutParams",
    "AEMatrixMetadataEnable", "AEGainConvergenceNormalizationEnable", "DeviceOrientation", "LockDynamicToneCurveNow",
    "UnlockDynamicToneCurveNow", "LockBLCNow", "UnlockBLCNow", "DiagnosticMode", "ImageMapParams", "AlternateOutputSize",
    "UsePreviewBufferQueueForRawReprocessing", "AFPeakPredictionEnabled", "AWB1stGainManual", "LimitAERange",
    "ChromaNoiseReductionEnabled", "PeakTrackingFocusNow", "LockLTMNow", "UnlockLTMNow", "DynamicToneCurve",
    "MaxPerFrameExposureDelta", "LocalHistogramsEnabled", "MaxFrameCoalescingDuration", "ContinuousAutoFocusNow",
    "FaceDetectionConfiguration", "VideoStabilizationISPOutputVectorsEnabled", "AutoFocusHintNow",
    "GainAdaptiveParameters", "FaceDetectionMetadataCoordinateSpace", "DecoupleBackEndFromStreaming",
    "SensorSidebandEnable", "ChromaNoiseReductionLevel", "NoiseReductionLevel", "LPDPFilterEnable",
    "HighSpeedOutputLevel", "LinkFrequencyIndex", "TestPattern", "MemoryPoolID", "MotionDataFromISPEnabled",
    "ISPMotionDataEnabled", "ISPHallDataEnabled", "AutoAENow", "TorchColor", "TorchManualParameters",
    "HighCurrentTorchEnabled", "RawImageProcessBegin", "RawImageProcessPrepare", "RawImageProcessGo",
    "RawImageProcessEnd", "ReloadNVM", "SphereMode",
};

// kFigCapturePortType_* values (CFStrings of CMCapture). CMCapture binds them to the uniqueName of the device entries of
// CMCapture.framework/<hw.model>/AVCaptureSession.plist (FigCaptureSourcePreprocessVideoDeviceDict 0x1a6ac7488): Back Camera,
// Front Camera, Back Ultra Wide Camera, Back Telephoto Camera (scripts/make-guamp-cmcapture-plist.py writes those entries).
// The OV02B macro stands in as the ultra wide (the role iOS gives macro shots), the OV02B depth module as the telephoto.
CFStringRef GPortTypeFor(uint32_t position) {
  switch (position) {
    case kGuampCamFront: return CFSTR("PortTypeFront");
    case kGuampCamMacro: return CFSTR("PortTypeBackSuperWide");
    case kGuampCamDepth: return CFSTR("PortTypeBackTelephoto");
    default: return CFSTR("PortTypeBack");
  }
}

static const GuampCamIosSensorInfo *sensorOf(GStream *s) { return &s->device->kernel.info.sensor[s->sensor]; }

static CFTypeRef copyOutputsEnabled(GStream *s) {
  static const CFStringRef ids[kGOutCount] = {CFSTR("PrimaryScaler"), CFSTR("SecondaryScaler"), CFSTR("StillImage")};
  CFMutableDictionaryRef d = CFDictionaryCreateMutable(NULL, kGOutCount, &kCFTypeDictionaryKeyCallBacks,
                                                       &kCFTypeDictionaryValueCallBacks);
  for (int i = 0; i < kGOutCount; ++i) CFDictionarySetValue(d, ids[i], s->out[i].enabled ? kCFBooleanTrue : kCFBooleanFalse);
  return d;
}

static OSStatus streamCopyProperty(GBaseObjectRef o, CFStringRef key, CFAllocatorRef alloc, void *valueOut) {
  GStream *s = streamOf(o);
  CFTypeRef *out = (CFTypeRef *)valueOut;
  const GuampCamIosSensorInfo *si = sensorOf(s);
  size_t nf = 0;
  const GFormat *f = GFormatsFor(s->position, &nf);
  const GFormat *cur = s->formatIndex < nf ? &f[s->formatIndex] : NULL;
  (void)alloc;
  if (!out) return kGErrParam;
  *out = NULL;
  pthread_mutex_lock(&s->lock);
  if (keyIs(key, "SupportedPropertiesDictionary")) *out = copyStreamSupported();
  else if (keyIs(key, "PortType")) *out = CFRetain(GPortTypeFor(s->position));
  else if (keyIs(key, "SupportedFormatsArray")) *out = GFormatsCopyArray(s->position);
  else if (keyIs(key, "FormatIndex")) *out = num32((int32_t)s->formatIndex);
  else if (keyIs(key, "FormatDescription")) { if (s->formatDescription) *out = CFRetain(s->formatDescription); }
  else if (keyIs(key, "MinimumFrameRate")) *out = numF(s->minFps);
  else if (keyIs(key, "MaximumFrameRate")) *out = numF(s->maxFps);
  else if (keyIs(key, "MinIntegrationTime")) *out = num32((int32_t)(s->mode.minExposureUs ? s->mode.minExposureUs : 30));
  else if (keyIs(key, "MaxIntegrationTime"))
    *out = num32((int32_t)(s->mode.maxExposureUs ? s->mode.maxExposureUs : 1000000.f / (cur ? cur->minFps : 15.f)));
  else if (keyIs(key, "SensorTemperature")) *out = num32(25);
  else if (keyIs(key, "ModuleInfo")) {
    CFNumberRef id = num32((int32_t)si->chipId);
    const void *k[] = {CFSTR("SensorID")}, *v[] = {id};
    *out = CFDictionaryCreate(NULL, k, v, 1, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    CFRelease(id);
  } else if (keyIs(key, "ZoomFactor")) *out = numF(s->zoom);
  else if (keyIs(key, "MaxZoomFactor")) *out = numF(8.f);                      // digital crop in gisp
  else if (keyIs(key, "FocalLength")) *out = numF(si->focalLengthUm / 1000.f);
  else if (keyIs(key, "PixelSize")) *out = numF(si->pixelSizeNm / 1000.f);
  else if (keyIs(key, "LensFNumber")) *out = numF(si->fNumberQ8 / 256.f);
  else if (keyIs(key, "VideoOutputsEnabled")) *out = copyOutputsEnabled(s);
  else if (keyIs(key, "SupportedColorSpaces")) {
    CFNumberRef srgb = num32(0);                                               // AVCaptureColorSpace_sRGB
    *out = CFArrayCreate(NULL, (const void **)&srgb, 1, &kCFTypeArrayCallBacks);
    CFRelease(srgb);
  } else if (keyIs(key, "ColorSpace")) *out = num32(0);
  else if (keyIs(key, "HasAutoFocusPositionSensor")) *out = CFRetain(kCFBooleanFalse);
  else if (keyIs(key, "DeferAdditionOfAttachments") || keyIs(key, "MultipleOutputSupportEnabled"))
    *out = CFRetain(kCFBooleanFalse);
  else {
    pthread_mutex_unlock(&s->lock);
    return kGErrPropertyNotFound;
  }
  pthread_mutex_unlock(&s->lock);
  return *out ? 0 : kGErrValueNotAvailable;
}

static void setHandlers(GStream *s, CFDictionaryRef all) {
  static const CFStringRef ids[kGOutCount] = {CFSTR("PrimaryScaler"), CFSTR("SecondaryScaler"), CFSTR("StillImage")};
  for (int i = 0; i < kGOutCount; ++i) {
    CFDictionaryRef h = CFDictionaryGetValue(all, ids[i]);
    if (!h || CFGetTypeID(h) != CFDictionaryGetTypeID()) continue;
    // Values are Objective-C blocks (H9ISP: CFEqual against kCFNull, then _Block_copy), kCFNull clears.
    const void *pb = CFDictionaryGetValue(h, CFSTR("PixelBufferHandler"));
    const void *ev = CFDictionaryGetValue(h, CFSTR("EventHandler"));
    if (s->out[i].pixelHandler) { Block_release(s->out[i].pixelHandler); s->out[i].pixelHandler = NULL; }
    if (s->out[i].eventHandler) { Block_release(s->out[i].eventHandler); s->out[i].eventHandler = NULL; }
    if (pb && !CFEqual(pb, kCFNull)) s->out[i].pixelHandler = Block_copy((GPixelBufferHandler)pb);
    if (ev && !CFEqual(ev, kCFNull)) s->out[i].eventHandler = Block_copy((GEventHandler)ev);
  }
}

static void setOutputsEnabled(GStream *s, CFDictionaryRef d) {
  static const CFStringRef ids[kGOutCount] = {CFSTR("PrimaryScaler"), CFSTR("SecondaryScaler"), CFSTR("StillImage")};
  for (int i = 0; i < kGOutCount; ++i) {
    CFTypeRef v = CFDictionaryGetValue(d, ids[i]);
    if (v && CFGetTypeID(v) == CFBooleanGetTypeID()) s->out[i].enabled = CFBooleanGetValue(v);
  }
}

static void setOutputConfigurations(GStream *s, CFDictionaryRef d) {
  // Per output: {Width, Height, PixelFormat} (key names as used by BWFigVideoCaptureStream: to confirm, doc section 9).
  static const CFStringRef ids[kGOutCount] = {CFSTR("PrimaryScaler"), CFSTR("SecondaryScaler"), CFSTR("StillImage")};
  for (int i = 0; i < kGOutCount; ++i) {
    CFDictionaryRef c = CFDictionaryGetValue(d, ids[i]);
    if (!c || CFGetTypeID(c) != CFDictionaryGetTypeID()) continue;
    double w = 0, h = 0, pf = 0;
    if (getNum(CFDictionaryGetValue(c, CFSTR("Width")), &w)) s->out[i].width = (uint32_t)w;
    if (getNum(CFDictionaryGetValue(c, CFSTR("Height")), &h)) s->out[i].height = (uint32_t)h;
    if (getNum(CFDictionaryGetValue(c, CFSTR("PixelFormat")), &pf) && (OSType)pf) s->out[i].pixelFormat = (OSType)pf;
    if (s->out[i].pool) { CVPixelBufferPoolRelease(s->out[i].pool); s->out[i].pool = NULL; }   // rebuilt on next frame
  }
}

static OSStatus streamSetProperty(GBaseObjectRef o, CFStringRef key, CFTypeRef value) {
  GStream *s = streamOf(o);
  GKernel *k = &s->device->kernel;
  double v = 0;
  OSStatus err = 0;
  pthread_mutex_lock(&s->lock);
  if (keyIs(key, "FormatIndex")) {
    size_t nf = 0;
    GFormatsFor(s->position, &nf);
    if (getNum(value, &v) && v >= 0 && (size_t)v < nf) s->formatIndex = (size_t)v; else err = kGErrParam;
  } else if (keyIs(key, "MinimumFrameRate")) { if (getNum(value, &v)) s->minFps = (float)v; }
  else if (keyIs(key, "MaximumFrameRate")) { if (getNum(value, &v)) s->maxFps = (float)v; }
  else if (keyIs(key, "MaxIntegrationTime")) { if (getNum(value, &v)) s->mode.maxExposureUs = (uint32_t)v; }
  else if (keyIs(key, "VideoOutputHandlers") && value && CFGetTypeID(value) == CFDictionaryGetTypeID()) setHandlers(s, value);
  else if (keyIs(key, "VideoOutputsEnabled") && value && CFGetTypeID(value) == CFDictionaryGetTypeID()) setOutputsEnabled(s, value);
  else if (keyIs(key, "VideoOutputConfigurations") && value && CFGetTypeID(value) == CFDictionaryGetTypeID())
    setOutputConfigurations(s, value);
  else if (keyIs(key, "StillImageCaptureNow")) {
    pthread_mutex_unlock(&s->lock);
    GFramesRequestStill(s, value && CFGetTypeID(value) == CFDictionaryGetTypeID() ? value : NULL);
    return 0;
  } else if (keyIs(key, "TorchLevel")) {
    // AVCaptureDevice torch: 0..1 float. Same LED and limits as GuampTorch (PMI632 0xd300).
    if (getNum(value, &v)) {
      s->torchPermille = (uint32_t)(fmin(fmax(v, 0.0), 1.0) * 1000.0);
      const uint64_t in[] = {s->torchPermille ? kGuampFlashTorch : kGuampFlashOff, s->torchPermille, 0};
      err = GKernelCall(k, kGuampCamSelSetFlash, in, 3, NULL, NULL) ? kGErrUnsupported : 0;
    }
  } else if (keyIs(key, "FlashMode")) {
    s->flashMode = value && CFGetTypeID(value) == CFStringGetTypeID() && CFStringFind(value, CFSTR("On"), 0).location != kCFNotFound;
  } else if (keyIs(key, "ExposureBias")) {
    // {Bias: float EV} or a bare number
    if (value && CFGetTypeID(value) == CFDictionaryGetTypeID()) getNum(CFDictionaryGetValue(value, CFSTR("Bias")), &v);
    else getNum(value, &v);
    s->exposureBias = (float)v;
  } else if (keyIs(key, "AEEnabled")) { if (getNum(value, &v)) atomic_store(&s->aeEnabled, v != 0); }
  else if (keyIs(key, "LockAENow")) atomic_store(&s->aeLocked, 1);
  else if (keyIs(key, "UnlockAENow")) atomic_store(&s->aeLocked, 0);
  else if (keyIs(key, "LockAWBNow")) atomic_store(&s->awbLocked, 1);
  else if (keyIs(key, "ImagerUnlockAWBNow")) atomic_store(&s->awbLocked, 0);
  else if (keyIs(key, "AutoFocusNow") || keyIs(key, "FocusNow")) atomic_store(&s->focusRequest, -2);
  else if (keyIs(key, "LockAFNow")) atomic_store(&s->afLocked, 1);
  else if (keyIs(key, "ManualFocusNow")) {
    // {Position: 0..1023}? lens position mapping to confirm; DW9767 takes a 10-bit DAC.
    if (value && CFGetTypeID(value) == CFDictionaryGetTypeID() && getNum(CFDictionaryGetValue(value, CFSTR("Position")), &v))
      atomic_store(&s->focusRequest, (int)fmin(fmax(v, 0), 1023));
  } else if (keyIs(key, "ZoomFactor")) { if (getNum(value, &v) && v >= 1.0) s->zoom = (float)v; }
  else if (keyIs(key, "AutoFocusMode") || keyIs(key, "AFWindowParams") || keyIs(key, "AEWindowParams") ||
           keyIs(key, "ManualAENow") || keyIs(key, "ManualWhiteBalanceGain") || keyIs(key, "ColorSpace") ||
           keyIs(key, "AEMaxGain") || keyIs(key, "AEMinGain") || keyIs(key, "DeferAdditionOfAttachments") ||
           keyIs(key, "MultipleOutputSupportEnabled")) {
    // TODO(3A, agent 4): forward windows/manual AE/WB to the 3A context; accepted for now.
  } else {
    err = kGErrPropertyNotFound;
    for (size_t i = 0; i < sizeof(kIgnoredSets) / sizeof(kIgnoredSets[0]); ++i)
      if (keyIs(key, kIgnoredSets[i])) { err = 0; break; }
  }
  pthread_mutex_unlock(&s->lock);
  return err;
}

static OSStatus streamStart(GBaseObjectRef o) { return GFramesStart(streamOf(o)); }
static OSStatus streamStop(GBaseObjectRef o) { GFramesStop(streamOf(o)); return 0; }

static OSStatus streamAddAttachments(GBaseObjectRef o, CMSampleBufferRef sbuf, CFDictionaryRef options) {
  // Attachments are already set on the pixel buffer when it is delivered (DeferAdditionOfAttachments is false).
  (void)o; (void)sbuf; (void)options;
  return 0;
}

static void streamFinalize(GBaseObjectRef o) {
  GStream *s = streamOf(o);
  GFramesStop(s);
  for (int i = 0; i < kGOutCount; ++i) {
    if (s->out[i].pixelHandler) Block_release(s->out[i].pixelHandler);
    if (s->out[i].eventHandler) Block_release(s->out[i].eventHandler);
    if (s->out[i].pool) CVPixelBufferPoolRelease(s->out[i].pool);
  }
  if (s->formatDescription) CFRelease(s->formatDescription);
  pthread_mutex_destroy(&s->lock);
}

static const GBaseClass kStreamBase = {
    .version = 1,
    .derivedStorageSize = sizeof(GStream),
    .finalize = streamFinalize,
    .copyProperty = streamCopyProperty,
    .setProperty = streamSetProperty,
};
static const GCaptureStreamClass kStreamClass = {
    .version = 2, .start = streamStart, .stop = streamStop, .addAttachmentsToSampleBuffer = streamAddAttachments};
static const GStreamVTable kStreamVTable = {NULL, &kStreamBase, &kStreamClass};

OSStatus GStreamCreate(GDevice *d, uint32_t sensor, GBaseObjectRef *out) {
  GBaseObjectRef obj = NULL;
  OSStatus err = gFig.derivedObjectCreate(NULL, &kStreamVTable, gFig.streamClassID(), &obj);
  if (err) return err;
  GStream *s = streamOf(obj);
  s->self = obj;
  s->device = d;
  s->sensor = sensor;
  s->position = d->kernel.info.sensor[sensor].position;
  pthread_mutex_init(&s->lock, NULL);
  size_t nf = 0;
  const GFormat *f = GFormatsFor(s->position, &nf);
  s->formatIndex = 0;
  s->minFps = nf ? f[0].minFps : 15.f;
  s->maxFps = nf ? f[0].maxFps : 30.f;
  s->zoom = 1.f;
  atomic_store(&s->aeEnabled, 1);
  atomic_store(&s->focusRequest, -1);
  for (int i = 0; i < kGOutCount; ++i) s->out[i].pixelFormat = kCVPixelFormatType_420YpCbCr8BiPlanarFullRange;
  s->out[kGOutPrimary].enabled = 1;
  *out = obj;
  return 0;
}
