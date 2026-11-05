// GuampFrames.c - the frame thread of one stream: RAW slot (kernel, ABI GuampCamIosAbi.h v2) -> gcampipe (package/GuampCamPipe:
// gisp statistics, g3a AE/AWB/AF, gisp rendering) -> 420f/420v IOSurface CVPixelBuffer -> PixelBufferHandler block of each
// enabled output.
//
//   preview (PrimaryScaler)  gcp_render_fast_prepare once per frame, then gcp_render_fast_band over dispatch_apply,
//                            written straight into the IOSurface planes
//   photo   (StillImage)     gcp_render_hq (MHC demosaic, NR, sharpen) on the next frame after StillImageCaptureNow,
//                            sensor already in kGuampModePhoto for the "Photo" format
// gcp_frame runs the 3A on every frame and programs the sensor through the kernel (gcp_ops below); the values the sensor
// really used come back in GuampCamIosFrame (exposureUs, analogGainQ8, focusDac) and feed the next AE step.
// Timestamps: GuampCamIosFrame.timestamp is mach_absolute_time() at SOF, the clock of the device (CMClockGetHostTimeClock),
// so the presentation time is CMClockMakeHostTimeFromSystemUnits of it, no conversion.
#include <Block.h>
#include <dispatch/dispatch.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "GuampISP.h"
#include "gcampipe.h"       // package/GuampCamPipe (agent 6), includes gisp.h

static CFStringRef metaKey(const char *suffix) {
  char sym[96];
  snprintf(sym, sizeof(sym), "kFigCaptureStreamMetadata_%s", suffix);
  CFStringRef k = gFig.lookupKey ? gFig.lookupKey(sym) : NULL;
  return k ? CFRetain(k) : CFStringCreateWithCString(NULL, suffix, kCFStringEncodingASCII);
}

static void putNum(CFMutableDictionaryRef d, const char *suffix, double v) {
  CFStringRef k = metaKey(suffix);
  CFNumberRef n = CFNumberCreate(NULL, kCFNumberDoubleType, &v);
  CFDictionarySetValue(d, k, n);
  CFRelease(n); CFRelease(k);
}

// Per-frame metadata dictionary attached to the pixel buffer as "MetadataDictionary" (H9ISP MyH9ISPFrameReceivedProc).
static CFDictionaryRef copyMetadata(GStream *s, const gcp_meta *m) {
  CFMutableDictionaryRef d = CFDictionaryCreateMutable(NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  putNum(d, "ExposureTime", m->exposure_us / 1e6);
  putNum(d, "AGC", m->again * 1024.0);                // H9 reports gains x1024 (to confirm against a real dump)
  putNum(d, "ISO", m->iso);
  putNum(d, "SensorID", m->chip_id);
  putNum(d, "SensorTemperature", m->temperature_mc == INT32_MIN ? 25 : m->temperature_mc / 1000.0);
  putNum(d, "AWBRGain", m->wb[0] * 1024);
  putNum(d, "AWBGGain", m->wb[1] * 1024);
  putNum(d, "AWBBGain", m->wb[2] * 1024);
  if (m->cct > 0) putNum(d, "AWBCCT", m->cct);
  putNum(d, "AETarget", m->ae_target * 1024);
  putNum(d, "AEAverage", m->ae_luma * 1024);
  putNum(d, "AEStable", m->ae_converged);
  putNum(d, "AFStable", m->af_state == GCP_AF_FOCUSED || m->af_state == GCP_AF_IDLE);
  putNum(d, "AFStatus", m->af_state);
  putNum(d, "LensPosition", m->focus_dac);
  putNum(d, "LuxLevel", m->lux);
  putNum(d, "SNR", 30);
  putNum(d, "ExposureBias", s->exposureBias);
  putNum(d, "FocalLength", s->device->kernel.info.sensor[s->sensor].focalLengthUm / 1000.0);
  CFStringRef portKey = metaKey("PortType");
  CFDictionarySetValue(d, portKey, GPortTypeFor(s->position));
  CFRelease(portKey);
  return d;
}

static CVPixelBufferPoolRef makePool(uint32_t w, uint32_t h, OSType pf) {
  CFMutableDictionaryRef attrs = CFDictionaryCreateMutable(NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  CFDictionaryRef ios = CFDictionaryCreate(NULL, NULL, NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  // Row alignment 64: the planes are sampled as Metal textures by the preview layer (CoreAnimation) and by the
  // photo/video encoders' converters; CMCapture's own pools use the same alignment.
  int32_t iw = (int32_t)w, ih = (int32_t)h, ipf = (int32_t)pf, align = 64;
  CFNumberRef nw = CFNumberCreate(NULL, kCFNumberSInt32Type, &iw), nh = CFNumberCreate(NULL, kCFNumberSInt32Type, &ih),
              npf = CFNumberCreate(NULL, kCFNumberSInt32Type, &ipf), nal = CFNumberCreate(NULL, kCFNumberSInt32Type, &align);
  CFDictionarySetValue(attrs, kCVPixelBufferWidthKey, nw);
  CFDictionarySetValue(attrs, kCVPixelBufferHeightKey, nh);
  CFDictionarySetValue(attrs, kCVPixelBufferPixelFormatTypeKey, npf);
  CFDictionarySetValue(attrs, kCVPixelBufferBytesPerRowAlignmentKey, nal);
  CFDictionarySetValue(attrs, kCVPixelBufferIOSurfacePropertiesKey, ios);   // IOSurface backed: crosses to the client
  CVPixelBufferPoolRef pool = NULL;
  CVPixelBufferPoolCreate(NULL, NULL, attrs, &pool);
  CFRelease(nw); CFRelease(nh); CFRelease(npf); CFRelease(nal); CFRelease(ios); CFRelease(attrs);
  return pool;
}

// gcampipe writes straight into the locked IOSurface planes.
static int renderInto(GStream *s, GOutput *o, const gisp_raw *raw, int hq, const gisp_rect *crop, CVPixelBufferRef *outPb) {
  gcp *p = (gcp *)s->isp;
  if (!o->pool) o->pool = makePool(o->width, o->height, o->pixelFormat);
  CVPixelBufferRef pb = NULL;
  if (!o->pool || CVPixelBufferPoolCreatePixelBuffer(NULL, o->pool, &pb) || !pb) { s->dropped++; return -1; }
  CVPixelBufferLockBaseAddress(pb, 0);
  gisp_nv12 out = {.width = o->width, .height = o->height,
                   .y = CVPixelBufferGetBaseAddressOfPlane(pb, 0), .y_stride = CVPixelBufferGetBytesPerRowOfPlane(pb, 0),
                   .uv = CVPixelBufferGetBaseAddressOfPlane(pb, 1), .uv_stride = CVPixelBufferGetBytesPerRowOfPlane(pb, 1)};
  const int full = o->pixelFormat == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange;
  int rc;
  if (hq) {
    rc = gcp_render_hq(p, raw, crop, &out, full);
  } else {
    rc = gcp_render_fast_prepare(p, raw, crop, o->width, o->height, full);
    if (rc == 0) {
      // Bands in parallel on GCD (gcp_render_fast_band: different bands of one frame may run concurrently).
      __block int bandErr = 0;
      gisp_nv12 *dst = &out;
      const size_t bands = 8;
      dispatch_apply(bands, dispatch_get_global_queue(QOS_CLASS_USER_INTERACTIVE, 0), ^(size_t i) {
        if (gcp_render_fast_band(p, raw, dst, (uint32_t)i, (uint32_t)bands)) bandErr = -1;
      });
      rc = bandErr;
    }
  }
  CVPixelBufferUnlockBaseAddress(pb, 0);
  if (rc) { CVPixelBufferRelease(pb); s->dropped++; return rc; }
  *outPb = pb;
  return 0;
}

static void deliver(GOutput *o, CVPixelBufferRef pb, CFDictionaryRef meta, uint64_t hostTime, uint32_t matrix) {
  CVBufferSetAttachment(pb, CFSTR("MetadataDictionary"), meta, kCVAttachmentMode_ShouldPropagate);
  // Colour tags of what gisp wrote (gisp_params.matrix, sRGB-like gamma, Rec.709 primaries): without them the preview
  // layer and the HEIC/JPEG encoders guess the YCbCr matrix, and a 601/709 mix-up tints the image.
  CVBufferSetAttachment(pb, kCVImageBufferYCbCrMatrixKey,
                        matrix == GISP_BT601 ? kCVImageBufferYCbCrMatrix_ITU_R_601_4 : kCVImageBufferYCbCrMatrix_ITU_R_709_2,
                        kCVAttachmentMode_ShouldPropagate);
  CVBufferSetAttachment(pb, kCVImageBufferColorPrimariesKey, kCVImageBufferColorPrimaries_ITU_R_709_2,
                        kCVAttachmentMode_ShouldPropagate);
  CVBufferSetAttachment(pb, kCVImageBufferTransferFunctionKey, kCVImageBufferTransferFunction_ITU_R_709_2,
                        kCVAttachmentMode_ShouldPropagate);
  const CMTime pts = CMClockMakeHostTimeFromSystemUnits(hostTime);
  if (o->pixelHandler) o->pixelHandler(pb, &pts);
}

// gcp_ops: the pipe's exposure / focus requests go down the same user client.
static int opsExposure(void *ctx, uint32_t sensor, uint32_t us, uint32_t againQ8, uint32_t dgainQ8) {
  const uint64_t in[] = {sensor, us, againQ8, dgainQ8};
  return GKernelCall((GKernel *)ctx, kGuampCamSelSetExposure, in, 4, NULL, NULL);
}
static int opsFocus(void *ctx, uint32_t sensor, uint32_t dac) {
  const uint64_t in[] = {sensor, dac};
  return GKernelCall((GKernel *)ctx, kGuampCamSelSetFocus, in, 2, NULL, NULL);
}

static void readControls(GStream *s, gcp_controls *c) {
  gcp_controls_default(c);
  c->ae_enabled = atomic_load(&s->aeEnabled);
  c->ae_lock = atomic_load(&s->aeLocked);
  c->awb_lock = atomic_load(&s->awbLocked);
  c->af_lock = atomic_load(&s->afLocked);
  c->exposure_bias = s->exposureBias;
  c->focus_request = atomic_exchange(&s->focusRequest, -1);
  c->af_continuous = 1;      // the Camera app expects continuous AF unless it locks it (LockAFNow)
}

static void *frameThread(void *arg) {
  GStream *s = (GStream *)arg;
  GKernel *k = &s->device->kernel;
  gcp *p = (gcp *)s->isp;
  while (atomic_load(&s->running)) {
    GuampCamIosFrame fr;
    size_t sz = sizeof(fr);
    const uint64_t in[] = {s->sensor, 200};
    if (GKernelCall(k, kGuampCamSelWaitFrame, in, 2, &fr, &sz)) continue;    // timeout or stop
    // Geometry and CFA come with each frame (CAMSS fills them per frame); gcp_raw checks them against the slot.
    gisp_raw raw;
    memset(&raw, 0, sizeof(raw));
    const int usable = fr.slot < s->mode.slotCount &&
                       gcp_raw(&fr, s->slot[fr.slot], s->slotSize[fr.slot], &raw) == 0;
    gcp_controls c;
    readControls(s, &c);
    gcp_meta m;
    if (gcp_frame(p, &fr, usable ? &raw : NULL, &c, &m) == 0) {
      CFDictionaryRef meta = copyMetadata(s, &m);
      const uint32_t matrix = gcp_params(p)->matrix;
      pthread_mutex_lock(&s->lock);
      GOutput *pre = &s->out[kGOutPrimary];
      if (pre->enabled && pre->pixelHandler) {
        const gisp_rect crop = gcp_crop(&fr, pre->width, pre->height, s->zoom);
        CVPixelBufferRef pb = NULL;
        if (renderInto(s, pre, &raw, 0, &crop, &pb) == 0) { deliver(pre, pb, meta, fr.timestamp, matrix); CVPixelBufferRelease(pb); }
      }
      GOutput *st = &s->out[kGOutStill];
      if (atomic_load(&s->stillRequests) > 0 && st->pixelHandler) {
        atomic_fetch_sub(&s->stillRequests, 1);
        const gisp_rect crop = gcp_crop(&fr, st->width, st->height, s->zoom);
        CVPixelBufferRef pb = NULL;
        if (renderInto(s, st, &raw, 1, &crop, &pb) == 0) { deliver(st, pb, meta, fr.timestamp, matrix); CVPixelBufferRelease(pb); }
      }
      pthread_mutex_unlock(&s->lock);
      CFRelease(meta);
      s->frames++;
    }
    const uint64_t rel[] = {s->sensor, fr.slot};
    GKernelCall(k, kGuampCamSelReleaseFrame, rel, 2, NULL, NULL);
  }
  return NULL;
}

void GFramesRequestStill(GStream *s, CFDictionaryRef settings) {
  // Settings carry bracketing / flash / tone-curve keys (H9ISP StillImageCaptureNow reads AEBracketedCaptureParams,
  // IntegrationTime, Gain, ...). One plain frame for now; flash pre-metering (g3a_flash_*) is the next step.
  (void)settings;
  if (s->flashMode && s->device->kernel.info.sensor[s->sensor].hasFlash) {
    const uint64_t in[] = {kGuampFlashPulse, 500, 200};   // main flash 500 mA, 200 ms (GuampTorch photo flash, agent 4)
    GKernelCall(&s->device->kernel, kGuampCamSelSetFlash, in, 3, NULL, NULL);
  }
  atomic_fetch_add(&s->stillRequests, 1);
}

OSStatus GFramesStart(GStream *s) {
  if (atomic_load(&s->running)) return 0;
  GKernel *k = &s->device->kernel;
  size_t nf = 0;
  const GFormat *f = GFormatsFor(s->position, &nf);
  const GFormat *fmt = s->formatIndex < nf ? &f[s->formatIndex] : &f[0];
  uint64_t open[] = {s->sensor};
  if (GKernelCall(k, kGuampCamSelOpen, open, 1, NULL, NULL)) return kGErrUnsupported;
  size_t sz = sizeof(s->mode);
  const uint64_t mode[] = {s->sensor, fmt->mode, (uint64_t)(s->maxFps * 256.f), (uint64_t)(s->minFps * 256.f)};
  if (GKernelCall(k, kGuampCamSelSetMode, mode, 4, &s->mode, &sz)) goto fail;
  for (uint32_t i = 0; i < s->mode.slotCount && i < GUAMP_CAMIOS_MAX_SLOTS; ++i) {
    s->slot[i] = GKernelMapSlot(k, s->sensor, i, &s->slotSize[i]);
    if (!s->slot[i]) goto fail;
  }
  // TODO(agent 3): pass the per-sensor .gisp tuning (chromatix-derived, vendor data outside the repo) instead of NULL.
  const gcp_ops ops = {.ctx = k, .set_exposure = opsExposure, .set_focus = opsFocus};
  s->isp = gcp_new(s->sensor, &k->info.sensor[s->sensor], &s->mode, s->minFps, &ops, NULL, 0);
  if (!s->isp) goto fail;
  for (int i = 0; i < kGOutCount; ++i) {
    GOutput *o = &s->out[i];
    if (!o->width || !o->height) {
      o->width = i == kGOutStill ? fmt->sensorWidth : fmt->outWidth;
      o->height = i == kGOutStill ? fmt->sensorHeight : fmt->outHeight;
    }
  }
  const uint64_t start[] = {s->sensor};
  if (GKernelCall(k, kGuampCamSelStart, start, 1, NULL, NULL)) goto fail;
  atomic_store(&s->running, 1);
  if (pthread_create(&s->thread, NULL, frameThread, s)) { atomic_store(&s->running, 0); goto fail; }
  GISP_LOG("stream %u (position %u) started: format %s raw %ux%u fmt %u, out %ux%u", s->sensor, s->position, fmt->name,
           s->mode.width, s->mode.height, s->mode.rawFormat, s->out[kGOutPrimary].width, s->out[kGOutPrimary].height);
  return 0;
fail:
  GFramesStop(s);
  return kGErrUnsupported;
}

void GFramesStop(GStream *s) {
  GKernel *k = &s->device->kernel;
  const uint64_t in[] = {s->sensor};
  if (atomic_exchange(&s->running, 0)) {
    GKernelCall(k, kGuampCamSelStop, in, 1, NULL, NULL);   // wakes WaitFrame
    pthread_join(s->thread, NULL);
  }
  for (uint32_t i = 0; i < GUAMP_CAMIOS_MAX_SLOTS; ++i)
    if (s->slot[i]) { GKernelUnmapSlot(k, s->sensor, i, s->slot[i]); s->slot[i] = NULL; }
  GKernelCall(k, kGuampCamSelClose, in, 1, NULL, NULL);
  if (s->isp) { gcp_free((gcp *)s->isp); s->isp = NULL; }
  if (s->torchPermille) {
    const uint64_t off[] = {kGuampFlashOff, 0, 0};
    GKernelCall(k, kGuampCamSelSetFlash, off, 3, NULL, NULL);
  }
}
