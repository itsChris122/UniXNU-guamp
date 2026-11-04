// FigShim.h - the few CoreMedia / CMCapture interfaces a capture plug-in implements, declared from what H9ISP.mediacapture
// (iPadOS 17.7.11, 21H461, dyld cache of iPad7,1) actually uses. No Apple header is copied: the layouts below are the ones
// read from kH9ISPCaptureDeviceVTable / kH9ISPCaptureStreamVTable (see analisi/camera/cam5-servizio-ios.md, section 3).
//
// CMBaseClass is the public CoreMedia base-object class (CMBaseObject.h of the macOS SDK has the same field order).
// Private symbols (FigCapture*GetClassID, CMDerivedObjectCreate) are looked up at run time with dlsym, so the plug-in links
// only against public SDK libraries.
#ifndef GUAMP_FIG_SHIM_H
#define GUAMP_FIG_SHIM_H

#include <CoreFoundation/CoreFoundation.h>
#include <CoreMedia/CoreMedia.h>
#include <CoreVideo/CoreVideo.h>

typedef struct OpaqueCMBaseObject *GBaseObjectRef;
typedef uintptr_t GBaseClassID;

typedef struct GBaseClass {
  uintptr_t version;                                                     // 1
  size_t derivedStorageSize;                                             // bytes behind the object, zero-filled
  Boolean (*equal)(GBaseObjectRef, GBaseObjectRef);
  OSStatus (*invalidate)(GBaseObjectRef);
  void (*finalize)(GBaseObjectRef);
  CFStringRef (*copyDebugDescription)(GBaseObjectRef);
  OSStatus (*copyProperty)(GBaseObjectRef, CFStringRef key, CFAllocatorRef, void *valueOut /* CFTypeRef * */);
  OSStatus (*setProperty)(GBaseObjectRef, CFStringRef key, CFTypeRef value);
  OSStatus (*notificationBarrier)(GBaseObjectRef);
  const void *(*getProtocolVTable)(GBaseObjectRef, uintptr_t protocolID);
} GBaseClass;

// kH9ISPCaptureDeviceClass: version 1, then the ISP processing-session constructor (offline RAW/DRC reprocessing).
typedef struct GCaptureDeviceClass {
  uintptr_t version;                                                     // 1
  OSStatus (*createProcessingSession)(GBaseObjectRef device, CFAllocatorRef, CFDictionaryRef config, const void **out);
  void *reserved[3];
} GCaptureDeviceClass;

// kH9ISPCaptureStreamClass: version 2, start, stop, add attachments.
typedef struct GCaptureStreamClass {
  uintptr_t version;                                                     // 2
  OSStatus (*start)(GBaseObjectRef stream);
  OSStatus (*stop)(GBaseObjectRef stream);
  OSStatus (*addAttachmentsToSampleBuffer)(GBaseObjectRef stream, CMSampleBufferRef, CFDictionaryRef options);
  void *reserved[1];
} GCaptureStreamClass;

typedef struct GDeviceVTable { const void *reserved; const GBaseClass *base; const GCaptureDeviceClass *klass; } GDeviceVTable;
typedef struct GStreamVTable { const void *reserved; const GBaseClass *base; const GCaptureStreamClass *klass; } GStreamVTable;

// CMBaseObject errors as returned by H9ISP (0xffffce14 = -12780, 0xffffce0f = -12785).
enum {
  kGErrParam = -12780,
  kGErrAlloc = -12781,
  kGErrUnsupported = -12782,
  kGErrValueNotAvailable = -12783,
  kGErrPropertyNotFound = -12784,
  kGErrInvalidated = -12785,
};

// Video output handler blocks (kFigCaptureStreamProperty_VideoOutputHandlers): H9ISP calls the PixelBufferHandler as
// block(pixelBuffer, &presentationTime) after attaching the metadata dictionaries to the pixel buffer
// (H9ISPCaptureStream frame path, 0x21e4ece98). The event handler signature is still to be confirmed (section 9 of the doc).
typedef void (^GPixelBufferHandler)(CVPixelBufferRef, const CMTime *);
typedef void (^GEventHandler)(CFStringRef event, CFDictionaryRef payload);

typedef struct GFigApi {
  OSStatus (*derivedObjectCreate)(CFAllocatorRef, const void *vtable, GBaseClassID, GBaseObjectRef *);
  void *(*getDerivedStorage)(GBaseObjectRef);
  GBaseClassID (*deviceClassID)(void);
  GBaseClassID (*streamClassID)(void);
  // CMCapture's own key constants, when exported (the plug-in also matches by CFString content).
  CFStringRef (*lookupKey)(const char *symbol);
} GFigApi;

int GFigApiLoad(GFigApi *api);   // 0 on success; logs the missing symbol otherwise

#endif
