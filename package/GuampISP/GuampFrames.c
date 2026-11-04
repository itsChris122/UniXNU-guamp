// GuampFrames.c - the frame thread of one stream: RAW slot (kernel) -> statistics + 3A (g3a, agent 4) -> software ISP
// (gisp, agent 3) -> 420f/420v IOSurface CVPixelBuffer -> PixelBufferHandler block of each enabled output.
//
//   preview (PrimaryScaler)  gisp_fast_set_params, then gisp_fast_band over dispatch_apply (cam3-isp.md section 2), every frame,
//                            written straight into the IOSurface planes
//   photo   (StillImage)     gisp_process (MHC demosaic, NR, sharpen) on the next frame after StillImageCaptureNow,
//                            sensor already in kGuampModePhoto for the "Photo" format; flash: g3a_flash_* sequence
// The 3A runs on every frame from gisp statistics of the same RAW and programs the sensor through the kernel; the
// values the sensor really used come back in GuampCamIosFrame (exposureUs, analogGainQ8, focusDac).
#include <Block.h>
#include <dispatch/dispatch.h>
#include <mach/mach_time.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "GuampISP.h"
#include "gisp.h"           // ~/moto/guamp-isp/include (agent 3)
#include "g3a.h"            // package/GuampCamera3A (agent 4)
#include "g3a_gisp.h"

typedef struct GProc {
  gisp_params params;
  gisp_fast *fast;          // preview pipeline, rebuilt when the output size changes
  uint32_t fastW, fastH;
  gisp_ctx *hq;             // still pipeline
  gisp_stats *stats;        // ~21 KB, heap
  g3a_ae_config ae;
  g3a_ae_result aeLast;
  g3a_awb_config awbCfg;
  g3a_awb_state awb;
  g3a_af_config afCfg;
  g3a_af_state af;
  int afActive;
  uint32_t lastExposureUs, lastGainQ8;
} GProc;

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
CFDictionaryRef GFramesCopyMetadata(GStream *s, const GuampCamIosFrame *fr) {
  GProc *p = (GProc *)s->isp;
  CFMutableDictionaryRef d = CFDictionaryCreateMutable(NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  const double exposure = fr->exposureUs / 1e6, again = fr->analogGainQ8 / 256.0, dgain = fr->digitalGainQ8 / 256.0;
  putNum(d, "ExposureTime", exposure);
  putNum(d, "AGC", fr->analogGainQ8 * 4.0);          // H9 reports gains x1024 (to confirm against a real dump)
  putNum(d, "ISO", 50.0 * again * (dgain > 0 ? dgain : 1));
  putNum(d, "SensorID", s->device->kernel.info.sensor[s->sensor].chipId);
  putNum(d, "SensorTemperature", fr->temperatureMilliC == INT32_MIN ? 25 : fr->temperatureMilliC / 1000.0);
  if (p) {
    putNum(d, "AWBRGain", p->params.wb[0] * 1024);
    putNum(d, "AWBGGain", p->params.wb[1] * 1024);
    putNum(d, "AWBBGain", p->params.wb[2] * 1024);
    putNum(d, "AETarget", p->aeLast.target * 1024);
    putNum(d, "AEAverage", p->aeLast.luma * 1024);
    putNum(d, "AEStable", p->aeLast.converged);
    putNum(d, "AFStable", p->af.state == G3A_AF_FOCUSED || !p->afActive);
    putNum(d, "AFStatus", p->af.state);
    putNum(d, "LuxLevel", exposure > 0 ? p->aeLast.luma * 250.0 / (exposure * again) : 0);   // rough
    putNum(d, "SNR", 30);
  }
  putNum(d, "ExposureBias", s->exposureBias);
  putNum(d, "FocalLength", s->device->kernel.info.sensor[s->sensor].focalLengthUm / 1000.0);
  CFStringRef portKey = metaKey("PortType");
  CFDictionarySetValue(d, portKey, s->position == kGuampCamFront ? CFSTR("PortTypeFront") : CFSTR("PortTypeBack"));
  CFRelease(portKey);
  return d;
}

static CVPixelBufferPoolRef makePool(uint32_t w, uint32_t h, OSType pf) {
  CFMutableDictionaryRef attrs = CFDictionaryCreateMutable(NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  CFDictionaryRef ios = CFDictionaryCreate(NULL, NULL, NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  int32_t iw = (int32_t)w, ih = (int32_t)h, ipf = (int32_t)pf;
  CFNumberRef nw = CFNumberCreate(NULL, kCFNumberSInt32Type, &iw), nh = CFNumberCreate(NULL, kCFNumberSInt32Type, &ih),
              npf = CFNumberCreate(NULL, kCFNumberSInt32Type, &ipf);
  CFDictionarySetValue(attrs, kCVPixelBufferWidthKey, nw);
  CFDictionarySetValue(attrs, kCVPixelBufferHeightKey, nh);
  CFDictionarySetValue(attrs, kCVPixelBufferPixelFormatTypeKey, npf);
  CFDictionarySetValue(attrs, kCVPixelBufferIOSurfacePropertiesKey, ios);   // IOSurface backed: crosses to the client
  CVPixelBufferPoolRef pool = NULL;
  CVPixelBufferPoolCreate(NULL, NULL, attrs, &pool);
  CFRelease(nw); CFRelease(nh); CFRelease(npf); CFRelease(ios); CFRelease(attrs);
  return pool;
}

// gisp writes straight into the locked IOSurface planes.
static int renderInto(GStream *s, GOutput *o, const gisp_raw *raw, int hq, const gisp_rect *crop, CVPixelBufferRef *outPb) {
  GProc *p = (GProc *)s->isp;
  if (!o->pool) o->pool = makePool(o->width, o->height, o->pixelFormat);
  CVPixelBufferRef pb = NULL;
  if (!o->pool || CVPixelBufferPoolCreatePixelBuffer(NULL, o->pool, &pb) || !pb) { s->dropped++; return -1; }
  CVPixelBufferLockBaseAddress(pb, 0);
  gisp_nv12 out = {.width = o->width, .height = o->height,
                   .y = CVPixelBufferGetBaseAddressOfPlane(pb, 0), .y_stride = CVPixelBufferGetBytesPerRowOfPlane(pb, 0),
                   .uv = CVPixelBufferGetBaseAddressOfPlane(pb, 1), .uv_stride = CVPixelBufferGetBytesPerRowOfPlane(pb, 1)};
  p->params.full_range = o->pixelFormat == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange;
  int rc;
  if (hq) {
    rc = gisp_process(p->hq, raw, crop, &p->params, &out);
  } else {
    if (!p->fast || p->fastW != o->width || p->fastH != o->height) {
      if (p->fast) gisp_fast_free(p->fast);
      p->fast = gisp_fast_new(raw, crop, o->width, o->height, GISP_FAST_AUTO, &p->params);
      p->fastW = o->width; p->fastH = o->height;
    } else {
      gisp_fast_set_params(p->fast, &p->params);
    }
    // Bands in parallel on GCD (gisp_fast_band: bands < GISP_FAST_MAX_BANDS may run concurrently).
    __block int bandErr = 0;
    if (p->fast) {
      gisp_fast *fast = p->fast;
      const gisp_nv12 *dst = &out;
      const size_t bands = 8;
      dispatch_apply(bands, dispatch_get_global_queue(QOS_CLASS_USER_INTERACTIVE, 0), ^(size_t i) {
        if (gisp_fast_band(fast, raw, (gisp_nv12 *)dst, (uint32_t)i, (uint32_t)bands)) bandErr = -1;
      });
    }
    rc = p->fast ? bandErr : -1;
  }
  CVPixelBufferUnlockBaseAddress(pb, 0);
  if (rc) { CVPixelBufferRelease(pb); s->dropped++; return rc; }
  *outPb = pb;
  return 0;
}

// Centre crop of the RAW with the output aspect ratio, divided by the zoom factor (even coordinates).
static gisp_rect cropFor(const GuampCamIosFrame *m, uint32_t ow, uint32_t oh, float zoom) {
  double w = m->width, h = m->height;
  if (ow && oh) { if (w * oh > h * ow) w = h * ow / oh; else h = w * oh / ow; }
  if (zoom > 1.f) { w /= zoom; h /= zoom; }
  gisp_rect r = {.w = (uint32_t)w & ~1u, .h = (uint32_t)h & ~1u};
  r.x = ((m->width - r.w) / 2) & ~1u;
  r.y = ((m->height - r.h) / 2) & ~1u;
  return r;
}

static void run3A(GStream *s, GProc *p, const gisp_raw *raw, const GuampCamIosFrame *fr) {
  GKernel *k = &s->device->kernel;
  p->stats->zones_x = 16; p->stats->zones_y = 12; p->stats->step = 2;
  gisp_stats_compute(raw, NULL, &p->params, p->stats);
  g3a_stats v;
  g3a_from_gisp(&v, p->stats, &p->params);
  if (!atomic_load(&s->awbLocked)) {
    float rawWb[3];
    if (g3a_awb_run(&v, &p->awbCfg, &p->awb, rawWb) == 0) memcpy(p->params.wb, p->awb.wb, sizeof(p->params.wb));
  }
  if (atomic_load(&s->aeEnabled) && !atomic_load(&s->aeLocked)) {
    g3a_ae_config c = p->ae;
    c.target = p->ae.target * powf(2.f, s->exposureBias);
    g3a_ae_run(&v, &c, p->params.wb, (uint64_t)fr->exposureUs * 1000u, fr->analogGainQ8 / 256.f, &p->aeLast);
    const uint32_t us = (uint32_t)(p->aeLast.exposure_ns / 1000u), gq8 = (uint32_t)(p->aeLast.gain * 256.f);
    if (us != p->lastExposureUs || gq8 != p->lastGainQ8) {
      const uint64_t in[] = {s->sensor, us, gq8, 256};
      GKernelCall(k, kGuampCamSelSetExposure, in, 4, NULL, NULL);
      p->lastExposureUs = us; p->lastGainQ8 = gq8;
    }
  }
  const int req = atomic_exchange(&s->focusRequest, -1);
  if (req == -2 && s->device->kernel.info.sensor[s->sensor].hasAutoFocus) { g3a_af_start(&p->af, &p->afCfg); p->afActive = 1; }
  else if (req >= 0) { const uint64_t in[] = {s->sensor, (uint64_t)req}; GKernelCall(k, kGuampCamSelSetFocus, in, 2, NULL, NULL); p->afActive = 0; }
  if (p->afActive && !atomic_load(&s->afLocked)) {
    const uint16_t dac = g3a_af_run(&v, &p->afCfg, &p->af);
    const uint64_t in[] = {s->sensor, dac};
    GKernelCall(k, kGuampCamSelSetFocus, in, 2, NULL, NULL);
    if (p->af.state == G3A_AF_FOCUSED || p->af.state == G3A_AF_FAILED) p->afActive = p->afCfg.continuous;
  }
}

static void deliver(GOutput *o, CVPixelBufferRef pb, CFDictionaryRef meta, uint64_t hostTime) {
  CVBufferSetAttachment(pb, CFSTR("MetadataDictionary"), meta, kCVAttachmentMode_ShouldPropagate);
  const CMTime pts = CMClockMakeHostTimeFromSystemUnits(hostTime);
  if (o->pixelHandler) o->pixelHandler(pb, &pts);
}

static void *frameThread(void *arg) {
  GStream *s = (GStream *)arg;
  GKernel *k = &s->device->kernel;
  GProc *p = (GProc *)s->isp;
  while (atomic_load(&s->running)) {
    GuampCamIosFrame fr;
    size_t sz = sizeof(fr);
    const uint64_t in[] = {s->sensor, 200};
    if (GKernelCall(k, kGuampCamSelWaitFrame, in, 2, &fr, &sz)) continue;    // timeout or stop
    // Geometry and CFA come with each frame (CAMSS fills them per frame); RAW10 MIPI and RAW16 match gisp_raw_format.
    const gisp_raw raw = {.width = fr.width, .height = fr.height, .stride = fr.strideBytes, .format = fr.rawFormat,
                          .cfa = fr.bayer, .data = fr.slot < s->mode.slotCount ? s->slot[fr.slot] : NULL};
    if (raw.data && fr.width && fr.height && !(fr.flags & 1u)) {
      run3A(s, p, &raw, &fr);
      CFDictionaryRef meta = GFramesCopyMetadata(s, &fr);
      pthread_mutex_lock(&s->lock);
      GOutput *pre = &s->out[kGOutPrimary];
      if (pre->enabled && pre->pixelHandler) {
        const gisp_rect crop = cropFor(&fr, pre->width, pre->height, s->zoom);
        CVPixelBufferRef pb = NULL;
        if (renderInto(s, pre, &raw, 0, &crop, &pb) == 0) { deliver(pre, pb, meta, fr.timestamp); CVPixelBufferRelease(pb); }
      }
      GOutput *st = &s->out[kGOutStill];
      if (atomic_load(&s->stillRequests) > 0 && st->pixelHandler) {
        atomic_fetch_sub(&s->stillRequests, 1);
        const gisp_rect crop = cropFor(&fr, st->width, st->height, s->zoom);
        CVPixelBufferRef pb = NULL;
        if (renderInto(s, st, &raw, 1, &crop, &pb) == 0) { deliver(st, pb, meta, fr.timestamp); CVPixelBufferRelease(pb); }
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
  GProc *p = calloc(1, sizeof(GProc));
  p->stats = calloc(1, sizeof(gisp_stats));
  p->hq = gisp_ctx_new();
  gisp_params_default(&p->params);
  for (int i = 0; i < 4; ++i) p->params.black[i] = (float)s->mode.blackLevel;
  // TODO(agent 3): load the per-sensor tuning (gisp_tuning_parse of the chromatix-derived file) and apply it here.
  g3a_ae_default(&p->ae);
  p->ae.t_min_ns = (uint64_t)s->mode.minExposureUs * 1000u;
  p->ae.t_max_ns = (uint64_t)(1e9 / (s->minFps > 0 ? s->minFps : 15.f));
  p->ae.gain_max = s->mode.maxAnalogGainQ8 ? s->mode.maxAnalogGainQ8 / 256.f : 16.f;
  g3a_awb_default(&p->awbCfg);
  g3a_awb_reset(&p->awb);
  g3a_af_default(&p->afCfg, (uint16_t)s->mode.lensMin, (uint16_t)(s->mode.lensMax ? s->mode.lensMax : 1023));   // EEPROM range
  s->isp = p;
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
  GISP_LOG("stream %u started: format %s raw %ux%u fmt %u, out %ux%u", s->sensor, fmt->name, s->mode.width, s->mode.height,
           s->mode.rawFormat, s->out[kGOutPrimary].width, s->out[kGOutPrimary].height);
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
  if (s->isp) {
    GProc *p = (GProc *)s->isp;
    if (p->fast) gisp_fast_free(p->fast);
    if (p->hq) gisp_ctx_free(p->hq);
    free(p->stats);
    free(p);
    s->isp = NULL;
  }
  if (s->torchPermille) {
    const uint64_t off[] = {kGuampFlashOff, 0, 0};
    GKernelCall(k, kGuampCamSelSetFlash, off, 3, NULL, NULL);
  }
}
