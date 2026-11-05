// GuampH264SW.c - a visible software H.264 ('avc1') encoder for VideoToolbox on the guamp port (no AVE hardware).
//
// Why (analisi/camera/cam-media-20261106.md): VTLoadVideoEncoders (VideoToolbox 0x1978a4414, 21H461) dlopen()s a fixed list
// of encoder plug-ins. On this phone none of them gives a usable 'avc1' encoder:
//   - H264H9.videoencoder registers 'avc1' only when AppleAVEVA_CheckPlatform() finds the AVE (it does not here);
//   - VCPHEVC.videocodec VCPHEVCRegisterEncoder is an empty function on iOS (ret);
//   - VideoProcessing VPModuleInitialize registers 'avc1' com.apple.videotoolbox.videoencoder.h264.rtvc with
//     kVTVideoEncoder_Hide = true, Rating -1, IsHardwareAccelerated = false (the FaceTime/AVConference wrapper).
// So a plain VTCompressionSessionCreate('avc1') (FigAssetWriter, the camera's movie file sink) finds no encoder.
// VideoProcessing does carry Apple's own software H.264 encoder, H264VideoEncoder_CreateInstance (exported; the RTVC
// wrapper creates it with codec 'ftvc' when no hardware is there). Its signature is VideoToolbox's encoder factory
// (codecType, allocator, VTVideoEncoderRef *out): it builds the encoder object itself with CMDerivedObjectCreate and its own
// vtable, so this plug-in only has to register that factory again, visible, under its own encoder ID.
//
// The file is /System/Library/VideoEncoders/H264H7.videoencoder (a path the list dlopen()s but the 21H461 dyld cache does
// not carry: the A7 encoder), shown there by the kernel's bindfs view (guamp-venc, patches/xnu/...-guamp-camera-venc.patch)
// from the addon's videoencoders/ folder. Entry point: H264H7EncoderRegister (dlsym'd by VTLoadVideoEncoders).
// Every private symbol is looked up with dlsym: the plug-in links only against libSystem and CoreFoundation.
#include <CoreFoundation/CoreFoundation.h>
#include <dlfcn.h>
#include <os/log.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>

#define GUAMP_VENC_EXPORT __attribute__((visibility("default")))

typedef int32_t GOSStatus;
typedef GOSStatus (*GEncoderCreateFn)(uint32_t codecType, CFAllocatorRef allocator, void *encoderOut /* VTVideoEncoderRef * */);
typedef GOSStatus (*GRegisterWithInfoFn)(uint32_t codecType, CFDictionaryRef info, GEncoderCreateFn create);

#define GUAMP_FOURCC(a, b, c, d) (((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | ((uint32_t)(c) << 8) | (uint32_t)(d))
static const uint32_t kCodecAVC1 = GUAMP_FOURCC('a', 'v', 'c', '1');
static const uint32_t kCodecFTVC = GUAMP_FOURCC('f', 't', 'v', 'c');  // what the RTVC wrapper passes (0x1b7607c48)

static const char kVideoProcessing[] = "/System/Library/PrivateFrameworks/VideoProcessing.framework/VideoProcessing";
// Kill switch on the phone without a new image: touch /private/var/ngpu/videoencoders/.off (read once per process).
static const char kOffSwitch[] = "/private/var/ngpu/videoencoders/.off";

static GEncoderCreateFn g_apple_create;
static os_log_t g_log;

static GOSStatus guamp_h264sw_create(uint32_t codecType, CFAllocatorRef allocator, void *encoderOut)
{
  if (!g_apple_create || !encoderOut) {
    return -12902;  // kVTParameterErr
  }
  // Same call as Apple's wrapper; the codec argument is stored at +0x1388 and never read back (checked in the dyld cache).
  GOSStatus err = g_apple_create(kCodecFTVC, allocator, encoderOut);
  os_log(g_log, "GuampH264SW: encoder instance for '%{public}c%{public}c%{public}c%{public}c': %d",
         (int)(codecType >> 24), (int)((codecType >> 16) & 0xff), (int)((codecType >> 8) & 0xff), (int)(codecType & 0xff),
         (int)err);
  return err;
}

static void set_key(CFMutableDictionaryRef info, const char *symbol, CFTypeRef value)
{
  CFStringRef *key = (CFStringRef *)dlsym(RTLD_DEFAULT, symbol);
  if (key && *key) {
    CFDictionarySetValue(info, *key, value);
  }
}

GUAMP_VENC_EXPORT void H264H7EncoderRegister(void)
{
  g_log = os_log_create("org.guamp.videoencoder", "h264sw");
  struct stat st;
  if (stat(kOffSwitch, &st) == 0) {
    os_log(g_log, "GuampH264SW: disabled by %{public}s", kOffSwitch);
    return;
  }
  GRegisterWithInfoFn reg = (GRegisterWithInfoFn)dlsym(RTLD_DEFAULT, "VTRegisterVideoEncoderWithInfo");
  void *vp = dlopen(kVideoProcessing, RTLD_NOW | RTLD_LOCAL);
  g_apple_create = vp ? (GEncoderCreateFn)dlsym(vp, "H264VideoEncoder_CreateInstance") : NULL;
  if (!reg || !g_apple_create) {
    os_log_error(g_log, "GuampH264SW: not registered (VTRegisterVideoEncoderWithInfo %p, H264VideoEncoder_CreateInstance %p)",
                 (void *)reg, (void *)g_apple_create);
    return;
  }
  CFMutableDictionaryRef info = CFDictionaryCreateMutable(NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  if (!info) {
    return;
  }
  // As H264H9EncoderRegister: the class implementation ID is the encoder ID VTCopyVideoEncoderList reports.
  CFDictionarySetValue(info, CFSTR("CMClassImplementationID"), CFSTR("org.guamp.videotoolbox.videoencoder.h264.sw"));
  set_key(info, "kVTVideoEncoder_IsHardwareAccelerated", kCFBooleanFalse);
  set_key(info, "kVTVideoEncoder_Hide", kCFBooleanFalse);
  GOSStatus err = reg(kCodecAVC1, info, guamp_h264sw_create);
  CFRelease(info);
  os_log(g_log, "GuampH264SW: 'avc1' software encoder registered: %d", (int)err);
}
