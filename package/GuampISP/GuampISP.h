// GuampISP.h - internal interfaces of the guamp capture plug-in (H6ISP.mediacapture).
#ifndef GUAMP_ISP_PLUGIN_H
#define GUAMP_ISP_PLUGIN_H

#include <pthread.h>
#include <stdatomic.h>
#include <IOSurface/IOSurfaceRef.h>
#include "FigShim.h"
#include "../../drivers/NativeEmmc/GuampCamIosAbi.h"

#define GISP_LOG(fmt, ...) GuampLog("GuampISP: " fmt, ##__VA_ARGS__)
void GuampLog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// ---- kernel link (GuampKernel.c) ---------------------------------------------------------------------------------
typedef struct GKernel {
  unsigned int connection;     // io_connect_t, 0 = closed
  GuampCamIosInfo info;
} GKernel;
int  GKernelOpen(GKernel *);                                   // IOServiceOpen on AppleH6CamIn
void GKernelClose(GKernel *);
int  GKernelCall(GKernel *, uint32_t selector, const uint64_t *in, uint32_t inCount, void *structOut, size_t *structOutSize);
const uint8_t *GKernelMapSlot(GKernel *, uint32_t sensor, uint32_t slot, size_t *size);   // one RAW slot, read-only
void GKernelUnmapSlot(GKernel *, uint32_t sensor, uint32_t slot, const uint8_t *base);

// ---- formats (GuampFormats.c) -----------------------------------------------------------------------------------
typedef struct GFormat {
  const char *name;            // FigCaptureStreamFormats key of the J120 AVCaptureSession.plist it stands for
  uint32_t sensorWidth, sensorHeight;   // dimensions CMCapture matches against that plist
  uint32_t outWidth, outHeight;         // default output (PrimaryScaler) size
  float minFps, maxFps;
  uint32_t mode;               // GuampCamIosMode
  uint32_t still;              // 1 = photo format (StillImage output allowed at full size)
  uint32_t binned;
} GFormat;
const GFormat *GFormatsFor(uint32_t position, size_t *count);
CFArrayRef GFormatsCopyArray(uint32_t position);              // kFigCaptureStreamProperty_SupportedFormatsArray

// ---- processing glue (GuampFrames.c): RAW slot -> gisp -> 420f IOSurface, 3A ----------------------------------------
typedef struct GOutput {
  GPixelBufferHandler pixelHandler;     // Block_copy'd
  GEventHandler eventHandler;
  uint32_t enabled;
  uint32_t width, height;               // requested by VideoOutputConfigurations (0 = format default)
  OSType pixelFormat;                   // '420f' (default) or '420v'
  CVPixelBufferPoolRef pool;
} GOutput;

enum { kGOutPrimary = 0, kGOutSecondary = 1, kGOutStill = 2, kGOutCount = 3 };

typedef struct GStream {
  GBaseObjectRef self;
  struct GDevice *device;
  uint32_t sensor;                      // index into GuampCamIosInfo.sensor
  uint32_t position;
  pthread_mutex_t lock;
  size_t formatIndex;
  float minFps, maxFps;
  GOutput out[kGOutCount];
  // 3A state and requests (values the property setters fill, the frame thread consumes)
  atomic_int aeLocked, awbLocked, afLocked, aeEnabled;
  atomic_int focusRequest;              // -1 none, else DAC; -2 one-shot AF scan
  atomic_int stillRequests;             // pending StillImageCaptureNow
  float exposureBias;
  uint32_t torchPermille;
  uint32_t flashMode;                   // 0 off, 1 on, 2 auto
  float zoom;
  // running
  pthread_t thread;
  atomic_int running;
  GuampCamIosModeInfo mode;
  const uint8_t *slot[GUAMP_CAMIOS_MAX_SLOTS];   // mapped RAW slots (kGuampCamMemRing + sensor * 16 + i)
  size_t slotSize[GUAMP_CAMIOS_MAX_SLOTS];
  void *isp;                            // gisp context (agent 3)
  void *aaa;                            // 3A context (agent 4)
  uint64_t frames, dropped;
  CMFormatDescriptionRef formatDescription;
} GStream;

typedef struct GDevice {
  GBaseObjectRef self;
  GKernel kernel;
  CFMutableArrayRef streams;            // GBaseObjectRef of each GStream
  CMClockRef clock;
  uint32_t invalidated;
} GDevice;

extern GFigApi gFig;
OSStatus GStreamCreate(GDevice *, uint32_t sensor, GBaseObjectRef *out);
OSStatus GFramesStart(GStream *);
void     GFramesStop(GStream *);
void     GFramesRequestStill(GStream *, CFDictionaryRef settings);
CFDictionaryRef GFramesCopyMetadata(GStream *, const GuampCamIosFrame *);

#endif
