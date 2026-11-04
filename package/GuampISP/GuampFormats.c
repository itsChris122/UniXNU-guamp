// GuampFormats.c - the FigCaptureStream formats the plug-in offers.
//
// CMCapture builds the AVCaptureDevice formats from the device plist CMCapture.framework/J120/AVCaptureSession.plist
// (MultiStream -> FigCaptureStreamFormats / AVCaptureDeviceFormats / AVCaptureSessionPresets) and pairs each plist stream
// format with an entry of the plug-in's SupportedFormatsArray with the same sensor dimensions (SensorWidth x SensorHeight,
// plus the Binned/FES flags). So the plug-in advertises the J120 sensor geometry, not the guamp one: the RAW from
// GM1ST/4H7 is cropped/scaled by gisp to whatever output size BWFigVideoCaptureStream asks for. Formats the guamp sensors
// cannot sustain (120/240 fps, 4K) are simply not offered; the plist entries that point at them stay unmatched.
//
// Back (GM1ST, 48 MP quad Bayer: full 8000x6000, 2x2 binned 4000x3000, sensor modes in GuampCamIosMode)
// Front (4H7, 8 MP: full 3264x2448, binned 1632x1224)
#include "GuampISP.h"

static const GFormat kBack[] = {
    // name                         sensor        out default  fps        mode               still binned
    {"Photo",                       4032, 3024,   1440, 1080,  15, 30,    kGuampModePhoto,    1, 0},
    {"Video-4by3-Low-Resolution",   4032, 3024,   1440, 1080,  15, 30,    kGuampModePreview,  0, 0},
    {"LowPower-Binned-2048x1536",   2048, 1536,   1920, 1440,  15, 30,    kGuampModePreview,  0, 1},
    {"Video-FES-30",                4096, 2304,   1920, 1080,  15, 30,    kGuampModeVideo,    0, 0},
};
static const GFormat kFront[] = {
    {"Photo",                       3088, 2320,   1440, 1080,  15, 30,    kGuampModePhoto,    1, 0},
    {"Video-4by3-Low-Resolution",   3088, 2320,   1440, 1080,  15, 30,    kGuampModePreview,  0, 0},
    {"LowPower-Binned-4by3",        3088, 2320,   1920, 1440,  15, 30,    kGuampModePreview,  0, 0},
    {"Video-FES-30",                3840, 2160,   1920, 1080,  15, 30,    kGuampModeVideo,    0, 0},
};

const GFormat *GFormatsFor(uint32_t position, size_t *count) {
  if (position == kGuampCamFront) { *count = sizeof(kFront) / sizeof(kFront[0]); return kFront; }
  *count = sizeof(kBack) / sizeof(kBack[0]);
  return kBack;
}

static void setNum(CFMutableDictionaryRef d, CFStringRef k, double v) {
  CFNumberRef n = CFNumberCreate(NULL, kCFNumberDoubleType, &v);
  CFDictionarySetValue(d, k, n);
  CFRelease(n);
}

static void setBool(CFMutableDictionaryRef d, CFStringRef k, int v) { CFDictionarySetValue(d, k, v ? kCFBooleanTrue : kCFBooleanFalse); }

static CFDictionaryRef copyRect(double x, double y, double w, double h) {
  // CGRectCreateDictionaryRepresentation layout (X, Y, Width, Height), built by hand to avoid CoreGraphics.
  CFMutableDictionaryRef r = CFDictionaryCreateMutable(NULL, 4, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  setNum(r, CFSTR("X"), x); setNum(r, CFSTR("Y"), y); setNum(r, CFSTR("Width"), w); setNum(r, CFSTR("Height"), h);
  return r;
}

// One output of SupportedOutputs (PrimaryScaler / SecondaryScaler / StillImage).
static CFDictionaryRef copyOutput(const GFormat *f, uint32_t w, uint32_t h, double maxDownscale) {
  CFMutableDictionaryRef o = CFDictionaryCreateMutable(NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  int32_t pf[2] = {kCVPixelFormatType_420YpCbCr8BiPlanarFullRange, kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange};
  CFNumberRef n0 = CFNumberCreate(NULL, kCFNumberSInt32Type, &pf[0]), n1 = CFNumberCreate(NULL, kCFNumberSInt32Type, &pf[1]);
  const void *v[] = {n0, n1};
  CFArrayRef formats = CFArrayCreate(NULL, v, 2, &kCFTypeArrayCallBacks);
  CFDictionarySetValue(o, CFSTR("SupportedPixelFormats"), formats);
  CFRelease(formats); CFRelease(n0); CFRelease(n1);
  setNum(o, CFSTR("NativeWidth"), w);
  setNum(o, CFSTR("NativeHeight"), h);
  setNum(o, CFSTR("MaxDownscaleFactor"), maxDownscale);
  CFDictionaryRef crop = copyRect(0, 0, f->sensorWidth, f->sensorHeight);
  CFDictionarySetValue(o, CFSTR("DefaultCropRect"), crop);
  CFRelease(crop);
  return o;
}

// One SupportedFormatsArray entry with the key set of H9ISP CopySupportedFormatsArray (0x21e4d2294, keys read through
// the GOT, doc section 5). The nesting (which keys sit inside SupportedOutputs/<output>) and the units of the
// integration times are inferred, not yet traced instruction by instruction: doc section 9 item 1.
static CFDictionaryRef copyFormat(const GFormat *f, uint32_t position) {
  CFMutableDictionaryRef d = CFDictionaryCreateMutable(NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  CMVideoFormatDescriptionRef fd = NULL;
  if (CMVideoFormatDescriptionCreate(NULL, kCVPixelFormatType_420YpCbCr8BiPlanarFullRange, (int32_t)f->sensorWidth,
                                     (int32_t)f->sensorHeight, NULL, &fd) == 0 && fd) {
    CFDictionarySetValue(d, CFSTR("FormatDescription"), fd);
    CFRelease(fd);
  }
  setNum(d, CFSTR("VideoMaxFrameRate"), f->maxFps);
  setNum(d, CFSTR("VideoMinFrameRate"), f->minFps);
  setBool(d, CFSTR("Experimental"), 0);
  setNum(d, CFSTR("VideoScaleFactor"), 1.0);
  setBool(d, CFSTR("VideoIsBinned"), f->binned);
  setNum(d, CFSTR("VideoBinningFactorHorizontal"), f->binned ? 2 : 1);
  setNum(d, CFSTR("VideoBinningFactorVertical"), f->binned ? 2 : 1);
  setBool(d, CFSTR("VideoLowLightContextSwitchSupported"), 0);
  setNum(d, CFSTR("VideoRawBitDepth"), 10);
  setNum(d, CFSTR("VideoMaxIntegrationTime"), 1e6 / f->minFps);   // microseconds (assumed)
  setNum(d, CFSTR("VideoMinIntegrationTime"), 30);
  setNum(d, CFSTR("ISOBase"), 50);
  setNum(d, CFSTR("DefaultAEMaxGain"), 16 * 256);                  // Q8, analog x16 (g3a gain_max)
  setNum(d, CFSTR("VideoMaxWidth"), f->sensorWidth);
  setNum(d, CFSTR("VideoMaxHeight"), f->sensorHeight);
  setNum(d, CFSTR("SensorWidth"), f->sensorWidth);
  setNum(d, CFSTR("SensorHeight"), f->sensorHeight);
  setBool(d, CFSTR("SubBandSplitEnabled"), 0);
  CFDictionaryRef crop = copyRect(0, 0, f->sensorWidth, f->sensorHeight);
  CFDictionarySetValue(d, CFSTR("VideoCropRect"), crop);
  CFRelease(crop);
  setNum(d, CFSTR("VideoFieldOfView"), position == kGuampCamFront ? 77.0 : 75.0);   // degrees, horizontal (estimate)
  setNum(d, CFSTR("SensorHDRMode"), 0);
  CFMutableDictionaryRef outs = CFDictionaryCreateMutable(NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  CFDictionaryRef primary = copyOutput(f, f->outWidth, f->outHeight, 16.0);
  CFDictionarySetValue(outs, CFSTR("PrimaryScaler"), primary);
  CFRelease(primary);
  if (f->still) {
    CFDictionaryRef still = copyOutput(f, f->sensorWidth, f->sensorHeight, 4.0);
    CFDictionarySetValue(outs, CFSTR("StillImage"), still);
    CFRelease(still);
  }
  CFDictionarySetValue(d, CFSTR("SupportedOutputs"), outs);
  CFRelease(outs);
  return d;
}

CFArrayRef GFormatsCopyArray(uint32_t position) {
  size_t n = 0;
  const GFormat *f = GFormatsFor(position, &n);
  CFMutableArrayRef a = CFArrayCreateMutable(NULL, (CFIndex)n, &kCFTypeArrayCallBacks);
  for (size_t i = 0; i < n; ++i) {
    CFDictionaryRef d = copyFormat(&f[i], position);
    CFArrayAppendValue(a, d);
    CFRelease(d);
  }
  return a;
}
