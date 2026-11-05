// campipe_host.cpp - integration skeleton of the guamp camera, on the host (agent 6, codex/cam-int-20261106):
//
//   fake CAMSS (GuampCamssApi.h ABI 2)        RAW10 MIPI frames of a synthetic scene in "TFE buffers"; with
//                                               CAMIOS_ARG_MODE=1 it plays a sensor (exposure x gain, 2-frame latency,
//                                               lens blur around DAC 600, colour cast of the light), with =3 the TPG
//   GuampCamIos.inc (real code, ABI v2)        AppleH6CamIn user client, CAMSS backend, pump thread, slot ring
//   package/GuampCamPipe/gcampipe.c            gisp statistics -> g3a AE/AWB/AF -> SetExposure/SetFocus -> gisp fast NV12
//
// It is the same sequence the H6ISP plug-in runs in mediaserverd (GetInfo, Open, SetMode, map the slots, Start,
// WaitFrame -> process -> ReleaseFrame, Stop, Close) with the real kernel service code and the real ISP and 3A, so each
// agent can plug in its piece and see the whole chain move: a sensor driver replaces the fake CAMSS on the phone, the
// plug-in replaces main(). Checks: ABI enum equalities, frame flags (gap, pending), applied exposure = the exposure the
// frame was made with, AE convergence, AWB against the light, AF peak, NV12 colours, GetStatus, clean close.
// CAMPIPE_PPM=<file> writes the last preview frame as a PPM.
#include "camios_host.cpp"   // XNU/IOKit stand-ins + GuampCamIos.inc (CAMIOS_NO_MAIN, CAMIOS_ARG_MODE from the build)
#include <math.h>
#include "gcampipe.h"   // g3a.h itself is C only (enum and struct g3a_af_state share a name): gcp_af_state mirrors it

#ifndef CAMIOS_SYNTAX_ONLY
static_assert(+kGuampRawMipi10 == +GISP_RAW10_MIPI && +kGuampRawU16 == +GISP_RAW16, "raw format enums must match gisp");
static_assert(+kGuampBayerRGGB == +GISP_RGGB && +kGuampBayerGRBG == +GISP_GRBG && +kGuampBayerGBRG == +GISP_GBRG &&
              +kGuampBayerBGGR == +GISP_BGGR && +kGuampBayerMono == +GISP_MONO, "CFA enums must match gisp");
static_assert(+GUAMP_CAMSS_MONO == +kGuampBayerMono, "mono has the same code in CAMSS and camios");

static const bool kTpg = CAMIOS_ARG_MODE == 3;

// ---- fake CAMSS ---------------------------------------------------------------------------------------------------
namespace fake {
const uint32_t W = 1280, H = 960;
IOBufferMemoryDescriptor *buf[4];
bool held[4], open_, streaming;
uint32_t nbuf, seq, width, height, bayer;
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
struct Exp { uint64_t from; uint32_t us, q8; };
Exp cur = {0, 10000, 256}, pend = {0, 0, 0};
bool pending;
uint32_t focus = 300, exposures, focuses;
uint32_t madeUs[1024], madeQ8[1024];       // per seq: what the frame was exposed with (seq & 1023)

// Scene: left half 8 colour bars, right half 4 neutral grey patches; the light tints the raw channels.
const float bars[8][3] = {{.9f, .9f, .9f}, {.8f, .8f, .1f}, {.1f, .8f, .8f}, {.1f, .7f, .1f},
                          {.7f, .1f, .7f}, {.7f, .1f, .1f}, {.1f, .1f, .7f}, {.05f, .05f, .05f}};
const float light[3] = {0.55f, 1.0f, 0.75f};   // warm-ish illuminant: AWB should find about 1/0.55, 1, 1/0.75

void scene(uint32_t x, uint32_t y, float rgb[3]) {
  (void)y;
  if (x < W / 2) { const float *b = bars[(x * 16) / W]; rgb[0] = b[0]; rgb[1] = b[1]; rgb[2] = b[2]; return; }
  const float g = 0.15f + 0.2f * (float)((x - W / 2) * 4 / (W / 2));
  rgb[0] = rgb[1] = rgb[2] = g;
}

void put10(uint8_t *row, uint32_t x, uint32_t v) {   // MIPI RAW10: 4 pixels in 5 bytes, low bits in the 5th byte
  uint8_t *g = row + (x / 4) * 5;
  g[x & 3] = static_cast<uint8_t>(v >> 2);
  g[4] = static_cast<uint8_t>((g[4] & ~(3u << ((x & 3) * 2))) | ((v & 3u) << ((x & 3) * 2)));
}

// CFA colour of site (x, y) for the CAMSS bayer code (TFE order RGGB, GRBG, BGGR, GBRG).
int cfa(uint32_t x, uint32_t y) {
  static const int tab[4][4] = {{0, 1, 1, 2}, {1, 0, 2, 1}, {2, 1, 1, 0}, {1, 2, 0, 1}};
  return tab[bayer][(y & 1) * 2 + (x & 1)];
}

void fill(uint8_t *p, uint32_t s) {
  const uint32_t lb = width * 10 / 8;
  if (kTpg) {                                            // TPG: fixed split colour bars, RGGB, no light, no lens
    for (uint32_t y = 0; y < height; ++y)
      for (uint32_t x = 0; x < width; ++x) {
        const float *b = bars[(x * 8) / width];
        put10(p + y * lb, x, 64 + static_cast<uint32_t>(b[cfa(x, y)] * 900));
      }
    return;
  }
  // Exposure x gain sets the level (white bar ~ 250 codes at 10 ms x1), the lens blurs the contrast away from DAC 600.
  const double k = 250.0 / 0.9 * (cur.us / 10000.0) * (cur.q8 / 256.0);
  const float sharp = 1.f / (1.f + fabsf(static_cast<float>(focus) - 600.f) / 60.f);
  static float line[4096][3];
  for (uint32_t x = 0; x < width; ++x) scene(x, 0, line[x]);
  float mean[3] = {0, 0, 0};
  for (uint32_t x = 0; x < width; ++x) for (int c = 0; c < 3; ++c) mean[c] += line[x][c] / width;
  for (uint32_t y = 0; y < height; ++y)
    for (uint32_t x = 0; x < width; ++x) {
      const int c = cfa(x, y);
      // Blur model: fine texture (checker of +-8 %) and edges lose contrast with defocus.
      const float tex = ((x / 4 + y / 4) & 1) ? 1.08f : 0.92f;
      const float v = mean[c] + (line[x][c] * (1.f + (tex - 1.f) * sharp) - mean[c]) * (0.4f + 0.6f * sharp);
      double code = 64 + v * light[c] * k;
      put10(p + y * lb, x, code > 1023 ? 1023 : static_cast<uint32_t>(code));
    }
  madeUs[s & 1023] = cur.us;
  madeQ8[s & 1023] = cur.q8;
}
}  // namespace fake

extern "C" {
int guamp_camss_open(const struct guamp_camss_config *c) {
  if (!c || c->abi != GUAMP_CAMSS_ABI || c->buffers < 2 || c->buffers > 4) return GUAMP_CAMSS_EINVAL;
  if ((c->source == GUAMP_CAMSS_SRC_TPG) != kTpg) return GUAMP_CAMSS_EINVAL;
  fake::width = kTpg ? (c->width ? c->width : 1600) : fake::W;
  fake::height = kTpg ? (c->height ? c->height : 1200) : fake::H;
  fake::bayer = kTpg ? GUAMP_CAMSS_RGGB : GUAMP_CAMSS_GBRG;   // GM1ST is GBRG (cam1 section 4)
  fake::nbuf = c->buffers;
  for (uint32_t i = 0; i < fake::nbuf; ++i) {
    fake::buf[i] = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, 0, fake::width * fake::height * 10 / 8, 4096);
    fake::held[i] = false;
  }
  fake::seq = 0;
  fake::open_ = true;
  return 0;
}
int guamp_camss_start(uint32_t) { fake::streaming = true; return 0; }
int guamp_camss_wait_frame(uint32_t after, uint32_t, struct guamp_camss_frame *f) {
  usleep(4000);                                          // ~250 fps source: the consumer falls behind on purpose
  pthread_mutex_lock(&fake::lock);
  int b = -1;
  for (uint32_t i = 0; i < fake::nbuf && b < 0; ++i) if (!fake::held[(fake::seq + i) % fake::nbuf]) b = (fake::seq + i) % fake::nbuf;
  if (b < 0 || !fake::streaming) { pthread_mutex_unlock(&fake::lock); return GUAMP_CAMSS_ETIMEDOUT; }
  const uint32_t s = ++fake::seq;
  if (fake::pending && s >= fake::pend.from) { fake::cur = fake::pend; fake::pending = false; }
  fake::held[b] = true;
  pthread_mutex_unlock(&fake::lock);
  fake::fill(static_cast<uint8_t *>(fake::buf[b]->bytes), s);
  *f = {};
  f->abi = GUAMP_CAMSS_ABI; f->seq = s; f->buffer = static_cast<uint32_t>(b); f->format = GUAMP_CAMSS_FMT_RAW10_MIPI;
  f->width = fake::width; f->height = fake::height; f->line_bytes = fake::width * 10 / 8; f->bytes = f->line_bytes * fake::height;
  f->bayer = fake::bayer; f->sof_ns = mach_absolute_time(); f->eof_ns = f->sof_ns + 8000000; f->cpu = fake::buf[b]->bytes;
  return after < s ? 0 : GUAMP_CAMSS_EINVAL;
}
void guamp_camss_release(uint32_t b) { pthread_mutex_lock(&fake::lock); if (b < 4) fake::held[b] = false; pthread_mutex_unlock(&fake::lock); }
int guamp_camss_stop(void) { fake::streaming = false; return 0; }
void guamp_camss_close(void) {
  for (uint32_t i = 0; i < fake::nbuf; ++i) { fake::buf[i]->release(); fake::buf[i] = nullptr; }
  fake::open_ = false;
}
int guamp_camss_counters(struct guamp_camss_counters *c) {   // the SOF count = frames generated so far
  *c = {};
  c->abi = GUAMP_CAMSS_ABI;
  pthread_mutex_lock(&fake::lock);
  c->sof = c->done = fake::seq;
  pthread_mutex_unlock(&fake::lock);
  return 0;
}
void *guamp_camss_buffer_memory(uint32_t b) { return b < fake::nbuf ? fake::buf[b] : nullptr; }
// Sensor hooks (GuampCamIosHooks.inc on the phone): the request exposes the frame made two frames after the last one.
int guamp_camios_hw_exposure(uint32_t cell, uint32_t us, uint32_t q8, uint32_t *aus, uint32_t *aq8) {
  if (cell != kGuampCellGm1st) return -22;
  const uint32_t lineNs = 10000;                         // quantise to lines like the sensor model (cam1 section 4)
  us = static_cast<uint32_t>(((uint64_t)us * 1000 / lineNs) * lineNs / 1000);
  pthread_mutex_lock(&fake::lock);
  fake::pend = {fake::seq + 2, us, q8};
  fake::pending = true;
  ++fake::exposures;
  pthread_mutex_unlock(&fake::lock);
  *aus = us; *aq8 = q8;
  return 0;
}
int guamp_camios_hw_focus(uint32_t cell, uint32_t dac, uint32_t *lo, uint32_t *hi) {
  if (cell != kGuampCellGm1st) return -22;
  if (lo) *lo = 100;
  if (hi) *hi = 900;
  if (dac != UINT32_MAX) { fake::focus = dac; ++fake::focuses; }
  return 0;
}
}

// ---- the client: what the plug-in does in mediaserverd -----------------------------------------------------------
struct Client { IOUserClient *uc; };
static int opsExposure(void *ctx, uint32_t sensor, uint32_t us, uint32_t ag, uint32_t dg) {
  return call(static_cast<Client *>(ctx)->uc, kGuampCamSelSetExposure, {sensor, us, ag, dg}) == kIOReturnSuccess ? 0 : -1;
}
static int opsFocus(void *ctx, uint32_t sensor, uint32_t dac) {
  return call(static_cast<Client *>(ctx)->uc, kGuampCamSelSetFocus, {sensor, dac}) == kIOReturnSuccess ? 0 : -1;
}

static void writePpm(const char *path, const gisp_nv12 &nv) {
  const size_t stride = nv.width * 3;
  uint8_t *rgb = static_cast<uint8_t *>(malloc(stride * nv.height));
  gisp_nv12_to_rgb8(&nv, GISP_BT709, 1, rgb, stride);
  FILE *f = fopen(path, "wb");
  if (f) { fprintf(f, "P6\n%u %u\n255\n", nv.width, nv.height); fwrite(rgb, 1, stride * nv.height, f); fclose(f); }
  free(rgb);
}

// Mean RGB of a box of the preview (NV12 -> RGB through gisp).
static void boxRgb(const gisp_nv12 &nv, uint32_t x0, uint32_t x1, uint32_t y0, uint32_t y1, double out[3]) {
  const size_t stride = nv.width * 3;
  uint8_t *rgb = static_cast<uint8_t *>(malloc(stride * nv.height));
  gisp_nv12_to_rgb8(&nv, GISP_BT709, 1, rgb, stride);
  out[0] = out[1] = out[2] = 0;
  unsigned n = 0;
  for (uint32_t y = y0; y < y1; ++y)
    for (uint32_t x = x0; x < x1; ++x, ++n)
      for (int c = 0; c < 3; ++c) out[c] += rgb[y * stride + x * 3 + c];
  for (int c = 0; c < 3; ++c) out[c] /= n ? n : 1;
  free(rgb);
}

int main() {
  camios::start();
  CHECK(camios::gService && camios::gBackend == &camios::camss::kBackend && camios::gBackendKind == (kTpg ? 2u : 1u));
  AppleH6CamIn *svc = static_cast<AppleH6CamIn *>(camios::gService);
  IOUserClient *uc = nullptr;
  CHECK(svc && svc->newUserClient(nullptr, nullptr, GUAMP_CAMIOS_CONNECT_TYPE, nullptr, &uc) == kIOReturnSuccess && uc);
  if (!uc) return 1;
  Client cl = {uc};

  GuampCamIosInfo info = {};
  CHECK(call(uc, kGuampCamSelGetInfo, {}, &info, sizeof(info)) == kIOReturnSuccess);
  CHECK(info.abiVersion == 2 && info.sensorCount == 2 && info.sensor[0].cell == kGuampCellGm1st && info.sensor[1].cell == kGuampCell4h7);
  CHECK(info.sensor[0].testPattern == (kTpg ? 2u : 0u));
  GuampCamIosStatus st = {};
  CHECK(call(uc, kGuampCamSelGetStatus, {0}, &st, sizeof(st)) == kIOReturnSuccess && st.state == kGuampStateClosed);

  CHECK(call(uc, kGuampCamSelOpen, {0}) == kIOReturnSuccess);
  GuampCamIosModeInfo mode = {};
  CHECK(call(uc, kGuampCamSelSetMode, {0, kGuampModePreview, 30u << 8, 15u << 8}, &mode, sizeof(mode)) == kIOReturnSuccess);
  CHECK(call(uc, kGuampCamSelGetStatus, {0}, &st, sizeof(st)) == kIOReturnSuccess && st.state == kGuampStateConfigured);
  CHECK(st.exposureLatency == 2 && st.backend == (kTpg ? 2u : 1u));
  if (!kTpg) CHECK(mode.lensMin == 100 && mode.lensMax == 900);
  // Map the slots (the plug-in: IOConnectMapMemory64(kGuampCamMemRing + sensor * 16 + i), read-only).
  const uint8_t *slot[GUAMP_CAMIOS_MAX_SLOTS] = {};
  size_t slotSize[GUAMP_CAMIOS_MAX_SLOTS] = {};
  for (uint32_t i = 0; i < mode.slotCount; ++i) {
    IOOptionBits opts = 0; IOMemoryDescriptor *mem = nullptr;
    CHECK(uc->clientMemoryForType(kGuampCamMemRing + i, &opts, &mem) == kIOReturnSuccess && (opts & kIOMapReadOnly));
    IOBufferMemoryDescriptor *b = static_cast<IOBufferMemoryDescriptor *>(mem);
    slot[i] = static_cast<const uint8_t *>(b->bytes);
    slotSize[i] = b->length;
    mem->release();
  }
  gcp_ops ops = {&cl, opsExposure, opsFocus};
  gcp *pipe = gcp_new(0, &info.sensor[0], &mode, 15.f, &ops, nullptr, 0);
  CHECK(pipe != nullptr);
  if (!pipe) return 1;
  CHECK(call(uc, kGuampCamSelStart, {0}) == kIOReturnSuccess);

  const uint32_t OW = 640, OH = 480;
  uint8_t *ybuf = static_cast<uint8_t *>(malloc(OW * OH)), *uvbuf = static_cast<uint8_t *>(malloc(OW * OH / 2));
  gisp_nv12 nv = {OW, OH, ybuf, OW, uvbuf, OW};
  gcp_controls ctl;
  gcp_controls_default(&ctl);
  gcp_meta meta = {};
  uint32_t frames = 0, gaps = 0, pendingFrames = 0, mismatches = 0, checked = 0;
  uint64_t lastTs = 0, lastSeq = 0;
  const uint32_t N = kTpg ? 12 : 90;
  for (uint32_t i = 0; i < N; ++i) {
    if (i == 45 && !kTpg) ctl.focus_request = -2;        // one-shot AF once AE/AWB settled
    GuampCamIosFrame fr = {};
    if (call(uc, kGuampCamSelWaitFrame, {0, 500}, &fr, sizeof(fr)) != kIOReturnSuccess) { CHECK(!"WaitFrame"); break; }
    CHECK(fr.sequence > lastSeq && fr.timestamp >= lastTs && fr.sensorMode == kGuampModePreview && fr.readoutUs == 8000);
    CHECK(((fr.flags & kGuampFrameTestPattern) != 0) == kTpg);
    gaps += (fr.flags & kGuampFrameGap) != 0;
    pendingFrames += (fr.flags & kGuampFrameSettingsPending) != 0;
    if (!kTpg) {                                         // applied values = what the frame was really exposed with
      ++checked;
      mismatches += fake::madeUs[fr.sequence & 1023] != fr.exposureUs || fake::madeQ8[fr.sequence & 1023] != fr.analogGainQ8;
    }
    lastSeq = fr.sequence; lastTs = fr.timestamp;
    gisp_raw raw;
    CHECK(gcp_raw(&fr, slot[fr.slot], slotSize[fr.slot], &raw) == 0);
    gcp_frame(pipe, &fr, &raw, &ctl, &meta);
    ctl.focus_request = -1;
    const gisp_rect crop = gcp_crop(&fr, OW, OH, 1.f);
    CHECK(gcp_render_fast_prepare(pipe, &raw, &crop, OW, OH, 1) == 0);
    for (uint32_t b = 0; b < 4; ++b) CHECK(gcp_render_fast_band(pipe, &raw, &nv, b, 4) == 0);
    if (i == 5) usleep(60 * 1000);                       // fall behind once while holding a slot: the pump recycles
    CHECK(call(uc, kGuampCamSelReleaseFrame, {0, fr.slot}) == kIOReturnSuccess);
    ++frames;
  }
  CHECK(call(uc, kGuampCamSelGetStatus, {0}, &st, sizeof(st)) == kIOReturnSuccess);
  CHECK(st.state == kGuampStateStreaming && st.delivered == frames && st.held == 0);
  printf("campipe %s: %u frames, seq %llu, %u gaps (%llu recycled), %u pending, exposure %u us gain %.2f, luma %.3f/%.3f %s, "
         "wb %.2f %.2f %.2f, af state %d dac %u (fv %.4f), %u exposure / %u focus writes, %u/%u applied mismatches\n",
         kTpg ? "TPG" : "sensor", frames, (unsigned long long)lastSeq, gaps, (unsigned long long)st.dropped, pendingFrames,
         meta.exposure_us, meta.again, meta.ae_luma, meta.ae_target, meta.ae_converged ? "converged" : "moving", meta.wb[0],
         meta.wb[1], meta.wb[2], meta.af_state, fake::focus, meta.focus_value, fake::exposures, fake::focuses, mismatches, checked);
  CHECK(gaps > 0 && st.dropped > 0);                     // the source is faster than the client: recycling works
  // NV12 colours: bars on the left, neutral patches on the right.
  double white[3], red[3], blue[3], grey[3];
  const uint32_t barW = kTpg ? OW / 8 : OW / 16;
  boxRgb(nv, barW / 4, barW * 3 / 4, OH / 3, OH * 2 / 3, white);
  boxRgb(nv, barW * 5 + barW / 4, barW * 5 + barW * 3 / 4, OH / 3, OH * 2 / 3, red);
  boxRgb(nv, barW * 6 + barW / 4, barW * 6 + barW * 3 / 4, OH / 3, OH * 2 / 3, blue);
  printf("preview rgb: white %.0f %.0f %.0f, red %.0f %.0f %.0f, blue %.0f %.0f %.0f\n", white[0], white[1], white[2],
         red[0], red[1], red[2], blue[0], blue[1], blue[2]);
  CHECK(red[0] > red[1] + 40 && red[0] > red[2] + 40);
  CHECK(blue[2] > blue[0] + 40 && blue[2] > blue[1] + 30);
  CHECK(white[1] > 150);
  if (kTpg) {
    CHECK(fake::exposures == 0);                         // the TPG does not follow the exposure: AE stays off
  } else {
    boxRgb(nv, OW / 2 + OW / 16 + 4, OW / 2 + OW / 8 + OW / 16 - 4, OH / 3, OH * 2 / 3, grey);
    printf("preview rgb: grey %.0f %.0f %.0f\n", grey[0], grey[1], grey[2]);
    CHECK(fabs(grey[0] - grey[1]) < 14 && fabs(grey[2] - grey[1]) < 14);   // AWB took the light out
    CHECK(fabsf(meta.wb[0] / meta.wb[1] - 1 / 0.55f) < 0.3f && fabsf(meta.wb[2] / meta.wb[1] - 1 / 0.75f) < 0.25f);
    CHECK(meta.ae_converged && fake::exposures > 0);
    CHECK(mismatches <= 2);                              // at most a race at a request edge
    CHECK(pendingFrames > 0);
    CHECK(meta.af_state == GCP_AF_FOCUSED && fake::focus > 540 && fake::focus < 660);
  }
  if (const char *ppm = getenv("CAMPIPE_PPM")) writePpm(ppm, nv);

  CHECK(call(uc, kGuampCamSelStop, {0}) == kIOReturnSuccess);
  CHECK(call(uc, kGuampCamSelGetStatus, {0}, &st, sizeof(st)) == kIOReturnSuccess && st.state == kGuampStateConfigured);
  CHECK(call(uc, kGuampCamSelClose, {0}) == kIOReturnSuccess);
  CHECK(!fake::open_ && !camios::gSensor[0].open);
  gcp_free(pipe);
  free(ybuf); free(uvbuf);
  uc->clientClose();
  uc->release();
  printf(failures ? "campipe %s: %d FAILURES\n" : "campipe %s: all checks passed\n", kTpg ? "TPG" : "sensor", failures);
  return failures ? 1 : 0;
}
#endif
