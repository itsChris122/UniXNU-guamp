// GuampCamIos.inc - the camera as iOS sees it: kernel half of the capture service (user half: package/GuampISP,
// /System/Library/MediaCapture/H6ISP.mediacapture). Boot argument guamp-camera-ios (default 0):
//   1  publish AppleH6CamIn; frames from CAMSS (GuampCamssApi.h ABI 2, agent 2) and the sensor session (agent 1)
//   2  publish AppleH6CamIn with the built-in test pattern only (no CAMSS), to bring up the iOS path alone
// guamp-camera-ios-slots = RAW slots per sensor (2..4, default 4; CAMSS has GUAMP_CAMSS_MAX_BUFFERS = 4).
//
// The service only has to exist under that class name for CMCapture to load our plug-in (GuampCamIosAbi.h). As agreed
// with the ISP (cam3-isp.md section 2) the kernel processes no pixel: it hands the RAW buffers to user space (one
// IOConnectMapMemory64 mapping per slot, zero copy), with mode, applied exposure/gain and timestamp, and writes the
// sensor registers (exposure, lens, flash) the user-space 3A asks for. List of commands: cam5-servizio-ios.md section 7.
// IOBufferMemoryDescriptor, IOUserClient: included by NativeEmmc.cpp before this file
#include "GuampCamIosAbi.h"
#include "GuampCamssApi.h"

extern "C" void guamp_torch_request(uint32_t permille);   // PmicService.inc (GuampTorch, PMI632 flash LED)

// CAMSS (agent 2, codex/cam-camss-20261105) and the hardware hooks of the sensor/lens side are resolved weakly: this
// file builds and the test pattern works on a kernel without them. The integration branch (agent 6) defines the hooks:
//   guamp_camios_hw_exposure -> camera::sensorExposure (agent 1) / camera::af::exposure (agent 4)
//   guamp_camios_hw_focus    -> camera::af::lensSet (agent 4), returns the lens range in *lo / *hi on dac = UINT32_MAX
//   guamp_camios_hw_flash    -> the photo-flash API of GuampTorch (FlashKind 1 pre-flash / 2 main, mA, ms; agent 4)
extern "C" {
int guamp_camss_open(const struct guamp_camss_config *) __attribute__((weak));
int guamp_camss_start(uint32_t) __attribute__((weak));
int guamp_camss_wait_frame(uint32_t, uint32_t, struct guamp_camss_frame *) __attribute__((weak));
void guamp_camss_release(uint32_t) __attribute__((weak));
int guamp_camss_stop(void) __attribute__((weak));
void guamp_camss_close(void) __attribute__((weak));
void *guamp_camss_buffer_memory(uint32_t) __attribute__((weak));
int guamp_camios_hw_exposure(uint32_t cell, uint32_t exposureUs, uint32_t gainQ8, uint32_t *appliedUs, uint32_t *appliedQ8)
    __attribute__((weak));
int guamp_camios_hw_focus(uint32_t cell, uint32_t dac, uint32_t *lo, uint32_t *hi) __attribute__((weak));
int guamp_camios_hw_flash(uint32_t kind, uint32_t ma, uint32_t ms) __attribute__((weak));
}

// ---- backend interface ------------------------------------------------------------------------------------------------
// Callbacks run in thread context with no camios lock held. A backend either fills the slots camios allocates
// (slotMemory = nullptr) or owns its buffers (slotMemory returns them, release gives one back after the client is done).
struct GuampCamIosBackend {
  const char *name;
  int (*probe)(void *ctx, uint32_t index, GuampCamIosSensorInfo *info);
  int (*setMode)(void *ctx, uint32_t sensor, uint32_t modeIndex, uint32_t maxFpsQ8, uint32_t minFpsQ8, GuampCamIosModeInfo *mode);
  IOBufferMemoryDescriptor *(*slotMemory)(void *ctx, uint32_t sensor, uint32_t slot);
  int (*start)(void *ctx, uint32_t sensor);
  void (*stop)(void *ctx, uint32_t sensor);
  void (*close)(void *ctx, uint32_t sensor);
  void (*release)(void *ctx, uint32_t sensor, uint32_t slot);
  int (*setExposure)(void *ctx, uint32_t sensor, uint32_t exposureUs, uint32_t againQ8, uint32_t dgainQ8);
  int (*setFocus)(void *ctx, uint32_t sensor, uint32_t dac);
};

namespace camios {

enum SlotState : uint8_t { kFree = 0, kFilling = 1, kReady = 2, kClient = 3 };

struct Sensor {
  GuampCamIosSensorInfo info;
  GuampCamIosModeInfo mode;
  IOBufferMemoryDescriptor *mem[GUAMP_CAMIOS_MAX_SLOTS];
  bool ownMem;                                 // allocated here (released here) or the backend's
  uint8_t state[GUAMP_CAMIOS_MAX_SLOTS];
  GuampCamIosFrame frame[GUAMP_CAMIOS_MAX_SLOTS];
  uint8_t queue[GUAMP_CAMIOS_MAX_SLOTS];       // ready slots, oldest first
  uint32_t queued;
  bool open, streaming;
  void *owner;                                 // the user client that opened it
  uint64_t sequence, delivered, dropped;
  uint32_t exposureUs, againQ8, dgainQ8, focusDac;
};

IOLock *gLock;
Sensor gSensor[GUAMP_CAMIOS_MAX_SENSORS];
uint32_t gSensors;
uint32_t gMode, gSlots = 4;
const GuampCamIosBackend *gBackend;
void *gBackendCtx;
IOService *gService;

uint32_t indexOf(const Sensor &s) { return static_cast<uint32_t>(&s - gSensor); }

// Lock held. Take the oldest ready-but-untaken frame back (the client is behind); returns its slot or -1.
int recycleOldest(Sensor &s) {
  if (!s.queued) return -1;
  const int slot = s.queue[0];
  memmove(s.queue, s.queue + 1, --s.queued);
  s.dropped++;
  return slot;
}

// Lock held. Queue a filled slot and wake WaitFrame.
void publish(Sensor &s, uint32_t slot, const GuampCamIosFrame &frame) {
  GuampCamIosFrame &f = s.frame[slot];
  f = frame;
  f.slot = slot;
  if (!f.sequence) f.sequence = ++s.sequence;
  if (!f.timestamp) f.timestamp = mach_absolute_time();
  if (!f.width) { f.width = s.mode.width; f.height = s.mode.height; f.strideBytes = s.mode.strideBytes; f.rawFormat = s.mode.rawFormat; }
  s.state[slot] = kReady;
  s.queue[s.queued++] = static_cast<uint8_t>(slot);
  IOLockWakeup(gLock, &s.queued, false);
}

// ---- built-in test pattern backend: RAW16 RGGB colour bars with a moving square, brightness follows the exposure ----
namespace pattern {
struct Run { volatile bool run, alive; uint32_t sensor; };
Run gRun[GUAMP_CAMIOS_MAX_SENSORS];

int probe(void *, uint32_t index, GuampCamIosSensorInfo *info) {
  if (index > 1) return -1;                               // back and front
  *info = {};
  info->present = 1;
  info->position = index == 0 ? kGuampCamBack : kGuampCamFront;
  info->fullWidth = index == 0 ? 4000 : 3264;
  info->fullHeight = index == 0 ? 3000 : 2448;
  info->bayer = kGuampBayerRGGB;
  info->bitDepth = 10;
  info->hasAutoFocus = index == 0;
  info->hasFlash = index == 0;
  info->focalLengthUm = index == 0 ? 4730 : 2580;          // estimates until the module data (agent 1) is wired in
  info->fNumberQ8 = index == 0 ? 461 : 563;                // f/1.8, f/2.2
  info->pixelSizeNm = index == 0 ? 800 : 1120;
  info->testPattern = 1;
  return 0;
}

int setMode(void *, uint32_t, uint32_t modeIndex, uint32_t maxFpsQ8, uint32_t minFpsQ8, GuampCamIosModeInfo *m) {
  // Small frames: the generator runs on the CPU and only has to exercise the iOS path.
  m->width = modeIndex == kGuampModePhoto ? 2016 : 1280;
  m->height = modeIndex == kGuampModePhoto ? 1512 : modeIndex == kGuampModeVideo ? 720 : 960;
  m->rawFormat = kGuampRawU16;
  m->bayer = kGuampBayerRGGB;
  m->strideBytes = m->width * 2;
  m->maxFpsQ8 = maxFpsQ8 ? maxFpsQ8 : 30u << 8;
  m->minFpsQ8 = minFpsQ8 ? minFpsQ8 : 15u << 8;
  m->minExposureUs = 30;
  m->maxExposureUs = 66000;
  m->maxAnalogGainQ8 = 16u << 8;
  m->maxDigitalGainQ8 = 4u << 8;
  m->blackLevel = 64;
  m->lineTimeNs = 10000;
  m->binning = 1;
  m->lensMin = 0;
  m->lensMax = 1023;
  return 0;
}

void fill(uint16_t *p, const GuampCamIosModeInfo &m, uint64_t seq, uint32_t exposureUs, uint32_t gainQ8) {
  // 8 vertical bars (white, yellow, cyan, green, magenta, red, blue, black) as R,G,B levels, scaled by exposure x gain
  static const uint8_t bars[8][3] = {{1, 1, 1}, {1, 1, 0}, {0, 1, 1}, {0, 1, 0}, {1, 0, 1}, {1, 0, 0}, {0, 0, 1}, {0, 0, 0}};
  const uint64_t light = static_cast<uint64_t>(exposureUs ? exposureUs : 10000) * (gainQ8 ? gainQ8 : 256);
  uint32_t level = static_cast<uint32_t>((light * 700u) / (10000ull * 256u));   // ~700 codes at 10 ms x1
  if (level > 959) level = 959;
  const uint32_t sq = m.height / 6, sx = static_cast<uint32_t>((seq * 8) % (m.width - sq)), sy = m.height / 2 - sq / 2;
  for (uint32_t y = 0; y < m.height; ++y) {
    uint16_t *row = reinterpret_cast<uint16_t *>(reinterpret_cast<uint8_t *>(p) + y * m.strideBytes);
    for (uint32_t x = 0; x < m.width; ++x) {
      const unsigned c = (y & 1) ? ((x & 1) ? 2 : 1) : ((x & 1) ? 1 : 0);        // RGGB
      unsigned on = bars[(x * 8) / m.width][c];
      if (x >= sx && x < sx + sq && y >= sy && y < sy + sq) on = c == 1;         // green square
      row[x] = static_cast<uint16_t>(64 + (on ? level : 0));
    }
  }
}

void thread(void *arg, wait_result_t) {
  Run *r = static_cast<Run *>(arg);
  Sensor &s = gSensor[r->sensor];
  while (r->run) {
    const uint32_t fps = s.mode.maxFpsQ8 ? s.mode.maxFpsQ8 >> 8 : 30;
    int slot = -1;
    IOLockLock(gLock);
    for (uint32_t i = 0; i < s.mode.slotCount && slot < 0; ++i)
      if (s.state[i] == kFree) slot = static_cast<int>(i);
    if (slot < 0) slot = recycleOldest(s);
    if (slot >= 0) s.state[slot] = kFilling;
    IOLockUnlock(gLock);
    if (slot >= 0) {
      GuampCamIosFrame f = {};
      f.exposureUs = s.exposureUs;
      f.analogGainQ8 = s.againQ8;
      f.digitalGainQ8 = s.dgainQ8;
      f.focusDac = s.focusDac;
      f.flags = 4u;
      f.temperatureMilliC = 30000;
      fill(static_cast<uint16_t *>(s.mem[slot]->getBytesNoCopy()), s.mode, s.sequence, f.exposureUs, f.analogGainQ8);
      IOLockLock(gLock);
      if (s.streaming && s.state[slot] == kFilling) publish(s, static_cast<uint32_t>(slot), f);
      IOLockUnlock(gLock);
    }
    IOSleep(1000 / (fps ? fps : 30));
  }
  r->alive = false;   // returning ends the kernel thread
}

int start(void *, uint32_t sensor) {
  Run &r = gRun[sensor];
  if (r.alive) return 0;
  r.sensor = sensor;
  r.run = true;
  r.alive = true;
  thread_t th = nullptr;
  if (kernel_thread_start(thread, &r, &th) != KERN_SUCCESS) { r.run = false; r.alive = false; return -1; }
  thread_deallocate(th);
  return 0;
}

void stop(void *, uint32_t sensor) {
  Run &r = gRun[sensor];
  r.run = false;
  for (int i = 0; i < 100 && r.alive; ++i) IOSleep(10);
}

int setExposure(void *, uint32_t, uint32_t, uint32_t, uint32_t) { return 0; }   // the generator reads the Sensor values
int setFocus(void *, uint32_t, uint32_t) { return 0; }

const GuampCamIosBackend kBackend = {"test pattern", probe, setMode, nullptr, start, stop, nullptr, nullptr, setExposure, setFocus};
}  // namespace pattern

// ---- CAMSS backend: one RDI stream at a time (guamp_camss_* is a single global session), zero copy ------------------
// Buffers are CAMSS's (Normal non-cacheable, SMMU stream 0x400); each is mapped read-only into the plug-in as one slot.
// A pump thread turns guamp_camss_wait_frame into published slots; the client's ReleaseFrame gives the buffer back.
// Note: user-space reads of non-cacheable memory are slow; if gisp_fast cannot hold 30 fps on them, copy each frame
// into a cached slot here (memcpy at ~1 GB/s costs ~4 ms for a 2000x1500 RAW10 frame) - decision after the first test.
namespace camss {
struct Pump { volatile bool run, alive; uint32_t sensor; };
Pump gPump;
uint32_t gOpenSensor = UINT32_MAX;

bool available() {
  return guamp_camss_open && guamp_camss_start && guamp_camss_wait_frame && guamp_camss_release && guamp_camss_stop &&
         guamp_camss_close && guamp_camss_buffer_memory;
}

int probe(void *, uint32_t index, GuampCamIosSensorInfo *info) {
  // Cell 0 S5KGM1ST (back, DW9767 AF, PMI632 flash), cell 1 S5K4H7 (front). Depth/macro OV02B are not offered to iOS.
  if (index > 1) return -1;
  *info = {};
  info->present = 1;
  info->position = index == 0 ? kGuampCamBack : kGuampCamFront;
  info->chipId = index == 0 ? 0x08d1 : 0x487b;
  info->fullWidth = index == 0 ? 4000 : 3264;              // GM1ST 2x2-binned (quad Bayer remosaic not used) / 4H7 full
  info->fullHeight = index == 0 ? 3000 : 2448;
  info->bayer = kGuampBayerGRBG;                           // replaced by the first frame's bayer (frame metadata wins)
  info->bitDepth = 10;
  info->hasAutoFocus = index == 0;
  info->hasFlash = index == 0;
  info->focalLengthUm = index == 0 ? 4730 : 2580;
  info->fNumberQ8 = index == 0 ? 461 : 563;
  info->pixelSizeNm = index == 0 ? 1600 : 1120;            // binned GM1ST pitch
  return 0;
}

uint32_t bayerOf(uint32_t camssBayer) {
  switch (camssBayer) {
    case GUAMP_CAMSS_GRBG: return kGuampBayerGRBG;
    case GUAMP_CAMSS_BGGR: return kGuampBayerBGGR;
    case GUAMP_CAMSS_GBRG: return kGuampBayerGBRG;
    default: return kGuampBayerRGGB;
  }
}

int setMode(void *, uint32_t sensor, uint32_t modeIndex, uint32_t maxFpsQ8, uint32_t minFpsQ8, GuampCamIosModeInfo *m) {
  if (gOpenSensor != UINT32_MAX) { guamp_camss_close(); gOpenSensor = UINT32_MAX; }
  guamp_camss_config c = {};
  c.abi = GUAMP_CAMSS_ABI;
  c.source = GUAMP_CAMSS_SRC_SENSOR;
  c.cell = sensor == 0 ? 0 : 1;
  c.mode = modeIndex == kGuampModePhoto ? 1 : 0;           // sensor driver resolution: 0 low (binned), 1 full (agent 1)
  c.buffers = gSlots;
  c.csid = 0;
  c.rdi = 0;
  c.clock_level = modeIndex == kGuampModePhoto ? 1 : 0;
  const int r = guamp_camss_open(&c);
  if (r) { IOLog("GuampCamIos: guamp_camss_open cell %u mode %u: %d\n", c.cell, c.mode, r); return -1; }
  gOpenSensor = sensor;
  // Geometry the sensor driver programs; the authoritative values travel with every frame (GuampCamIosFrame).
  const bool back = sensor == 0, full = c.mode == 1;
  m->width = back ? (full ? 4000 : 2000) : (full ? 3264 : 1632);
  m->height = back ? (full ? 3000 : 1500) : (full ? 2448 : 1224);
  m->rawFormat = kGuampRawMipi10;
  m->bayer = gSensor[sensor].info.bayer;
  m->strideBytes = m->width * 10 / 8;
  m->maxFpsQ8 = maxFpsQ8 ? maxFpsQ8 : 30u << 8;
  m->minFpsQ8 = minFpsQ8 ? minFpsQ8 : 15u << 8;
  m->minExposureUs = 30;
  m->maxExposureUs = (256u * 1000000u) / (m->minFpsQ8 ? m->minFpsQ8 : 15u << 8);
  m->maxAnalogGainQ8 = 16u << 8;                           // g3a gain_max main/front
  m->maxDigitalGainQ8 = 256;
  m->blackLevel = 64;
  m->binning = back && !full ? 2 : 1;
  uint32_t lo = 0, hi = 1023;
  if (back && guamp_camios_hw_focus) guamp_camios_hw_focus(0, UINT32_MAX, &lo, &hi);   // EEPROM-calibrated range
  m->lensMin = lo;
  m->lensMax = hi;
  return 0;
}

IOBufferMemoryDescriptor *slotMemory(void *, uint32_t, uint32_t slot) {
  return static_cast<IOBufferMemoryDescriptor *>(guamp_camss_buffer_memory(slot));
}

void pump(void *arg, wait_result_t) {
  Pump *p = static_cast<Pump *>(arg);
  Sensor &s = gSensor[p->sensor];
  uint32_t after = 0;
  while (p->run) {
    guamp_camss_frame cf = {};
    if (guamp_camss_wait_frame(after, 200, &cf) != GUAMP_CAMSS_OK) continue;
    after = cf.seq;
    GuampCamIosFrame f = {};
    f.flags = cf.errors ? 1u : 0u;
    f.sequence = cf.seq;
    uint64_t abs = 0;
    nanoseconds_to_absolutetime(cf.sof_ns ? cf.sof_ns : cf.done_ns, &abs);   // CAMSS reports ns, the plug-in mach units
    f.timestamp = abs;
    f.exposureUs = s.exposureUs;
    f.analogGainQ8 = s.againQ8;
    f.digitalGainQ8 = s.dgainQ8;
    f.focusDac = s.focusDac;
    f.width = cf.width;
    f.height = cf.height;
    f.strideBytes = cf.line_bytes;
    f.rawFormat = kGuampRawMipi10;
    f.bayer = bayerOf(cf.bayer);
    f.temperatureMilliC = INT32_MIN;
    int recycled = -1;
    IOLockLock(gLock);
    if (!s.streaming || cf.buffer >= s.mode.slotCount) {
      IOLockUnlock(gLock);
      guamp_camss_release(cf.buffer);
      continue;
    }
    uint32_t busy = 1;                                     // this frame
    for (uint32_t i = 0; i < s.mode.slotCount; ++i) busy += i != cf.buffer && s.state[i] != kFree;
    if (busy >= s.mode.slotCount) recycled = recycleOldest(s);   // keep one buffer free for the TFE
    if (recycled >= 0) s.state[recycled] = kFree;
    publish(s, cf.buffer, f);
    IOLockUnlock(gLock);
    if (recycled >= 0) guamp_camss_release(static_cast<uint32_t>(recycled));
  }
  p->alive = false;
}

int start(void *, uint32_t sensor) {
  if (gOpenSensor != sensor || gPump.alive) return gPump.alive ? 0 : -1;
  const int r = guamp_camss_start(0);
  if (r) { IOLog("GuampCamIos: guamp_camss_start: %d\n", r); return -1; }
  gPump.sensor = sensor;
  gPump.run = true;
  gPump.alive = true;
  thread_t th = nullptr;
  if (kernel_thread_start(pump, &gPump, &th) != KERN_SUCCESS) { gPump.run = false; gPump.alive = false; guamp_camss_stop(); return -1; }
  thread_deallocate(th);
  return 0;
}

void stop(void *, uint32_t) {
  gPump.run = false;
  for (int i = 0; i < 100 && gPump.alive; ++i) IOSleep(10);
  guamp_camss_stop();
}

void close(void *, uint32_t sensor) {
  if (gOpenSensor == sensor) { guamp_camss_close(); gOpenSensor = UINT32_MAX; }
}

void release(void *, uint32_t, uint32_t slot) { guamp_camss_release(slot); }

int setExposure(void *, uint32_t sensor, uint32_t us, uint32_t againQ8, uint32_t dgainQ8) {
  if (!guamp_camios_hw_exposure) return 0;                 // no register path in this kernel: frames keep coming
  uint32_t appliedUs = us, appliedQ8 = againQ8;
  const int r = guamp_camios_hw_exposure(sensor == 0 ? 0 : 1, us, (againQ8 * (dgainQ8 ? dgainQ8 : 256)) >> 8, &appliedUs, &appliedQ8);
  if (!r) { gSensor[sensor].exposureUs = appliedUs; gSensor[sensor].againQ8 = appliedQ8; gSensor[sensor].dgainQ8 = 256; }
  return r;
}

int setFocus(void *, uint32_t sensor, uint32_t dac) {
  if (sensor != 0 || !guamp_camios_hw_focus) return 0;
  return guamp_camios_hw_focus(0, dac, nullptr, nullptr);
}

const GuampCamIosBackend kBackend = {"CAMSS", probe, setMode, slotMemory, start, stop, close, release, setExposure, setFocus};
}  // namespace camss

// ---- flash: GuampTorch LED (torch level, or the photo flash API when the integration provides it) -----------------
thread_call_t gFlashOff;
void flashOff(thread_call_param_t, thread_call_param_t) { guamp_torch_request(0); }
int flash(uint32_t mode, uint32_t level, uint32_t ms) {
  if (mode == kGuampFlashOff) { guamp_torch_request(0); return 0; }
  if (mode == kGuampFlashTorch) { guamp_torch_request(level > 1000 ? 1000 : level); return 0; }
  // kGuampFlashPulse (main) / kGuampFlashPre: level = current in mA here
  if (guamp_camios_hw_flash) return guamp_camios_hw_flash(mode == kGuampFlashPre ? 1 : 2, level, ms ? ms : 200);
  guamp_torch_request(1000);                               // fallback: torch at its maximum for `ms`
  if (gFlashOff) {
    uint64_t deadline;
    clock_interval_to_deadline(ms ? (ms > 1000 ? 1000 : ms) : 200, kMillisecondScale, &deadline);
    thread_call_enter_delayed(gFlashOff, deadline);
  }
  return 0;
}

void probeSensors() {
  gSensors = 0;
  for (uint32_t i = 0; i < GUAMP_CAMIOS_MAX_SENSORS; ++i) {
    GuampCamIosSensorInfo info = {};
    if (gBackend->probe(gBackendCtx, i, &info) == 0 && info.present) gSensor[gSensors++].info = info;
  }
  IOLog("GuampCamIos: backend '%s', %u sensor(s)\n", gBackend->name, gSensors);
}

}  // namespace camios

// ---- IOKit: AppleH6CamIn + its user client ------------------------------------------------------------------------
class AppleH9CamInUserClient : public IOUserClient {
  OSDeclareDefaultStructors(AppleH9CamInUserClient)
public:
  IOReturn clientClose() override;
  IOReturn clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory) override;
  IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args, IOExternalMethodDispatch *, OSObject *,
                          void *) override;
};
OSDefineMetaClassAndStructors(AppleH9CamInUserClient, IOUserClient)

class AppleH6CamIn : public IOService {
  OSDeclareDefaultStructors(AppleH6CamIn)
public:
  IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *, IOUserClient **handler) override;
};
OSDefineMetaClassAndStructors(AppleH6CamIn, IOService)

namespace camios {

bool allowed(char *who, size_t n) {
  proc_selfname(who, static_cast<int>(n));
  // mediaserverd hosts CMCapture on iPadOS 17; guampcam is our command-line probe (package/GuampISP/tools).
  return !strcmp(who, "mediaserverd") || !strcmp(who, "guampcam") ||
         IOUserClient::clientHasPrivilege(current_task(), kIOClientPrivilegeAdministrator) == kIOReturnSuccess;
}

void freeSlots(Sensor &s) {
  for (uint32_t i = 0; i < GUAMP_CAMIOS_MAX_SLOTS; ++i) {
    if (s.mem[i] && s.ownMem) s.mem[i]->release();
    s.mem[i] = nullptr;
  }
}

void stopSensor(Sensor &s) {
  if (s.streaming) gBackend->stop(gBackendCtx, indexOf(s));
  IOLockLock(gLock);
  s.streaming = false;
  s.queued = 0;
  memset(s.state, 0, sizeof(s.state));
  IOLockWakeup(gLock, &s.queued, false);
  IOLockUnlock(gLock);
}

void closeSensor(Sensor &s) {
  stopSensor(s);
  if (gBackend->close) gBackend->close(gBackendCtx, indexOf(s));
  freeSlots(s);
  IOLockLock(gLock);
  s.open = false;
  s.owner = nullptr;
  IOLockUnlock(gLock);
}

IOReturn setMode(Sensor &s, uint32_t idx, uint32_t mode, uint32_t maxQ8, uint32_t minQ8) {
  if (s.streaming || mode >= kGuampModeCount) return kIOReturnBusy;
  freeSlots(s);
  GuampCamIosModeInfo m = {};
  if (gBackend->setMode(gBackendCtx, idx, mode, maxQ8, minQ8, &m) || !m.width || !m.height || !m.strideBytes)
    return kIOReturnUnsupported;
  m.slotStride = static_cast<uint32_t>(round_page(static_cast<uint64_t>(m.strideBytes) * m.height));
  m.slotCount = gSlots;
  s.ownMem = !gBackend->slotMemory;
  for (uint32_t i = 0; i < m.slotCount; ++i) {
    s.mem[i] = s.ownMem ? IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryKernelUserShared),
                                                                       m.slotStride, page_size)
                        : gBackend->slotMemory(gBackendCtx, idx, i);
    if (!s.mem[i]) { freeSlots(s); return kIOReturnNoMemory; }
  }
  s.mode = m;
  s.exposureUs = 10000;
  s.againQ8 = 256;
  s.dgainQ8 = 256;
  IOLog("GuampCamIos: sensor %u mode %u: %ux%u fmt %u stride %u, %u slots (%s)\n", idx, mode, m.width, m.height, m.rawFormat,
        m.strideBytes, m.slotCount, s.ownMem ? "own" : gBackend->name);
  return kIOReturnSuccess;
}

IOReturn waitFrame(Sensor &s, uint32_t timeoutMs, GuampCamIosFrame *out) {
  uint64_t deadline;
  clock_interval_to_deadline(timeoutMs ? (timeoutMs > 2000 ? 2000 : timeoutMs) : 200, kMillisecondScale, &deadline);
  IOLockLock(gLock);
  while (s.streaming && !s.queued) {
    if (IOLockSleepDeadline(gLock, &s.queued, deadline, THREAD_ABORTSAFE) != THREAD_AWAKENED) break;
  }
  IOReturn r = kIOReturnTimeout;
  if (s.queued) {
    const uint8_t slot = s.queue[0];
    memmove(s.queue, s.queue + 1, --s.queued);
    s.state[slot] = kClient;
    *out = s.frame[slot];
    s.delivered++;
    r = kIOReturnSuccess;
  } else if (!s.streaming) {
    r = kIOReturnNotReady;
  }
  IOLockUnlock(gLock);
  return r;
}

}  // namespace camios

IOReturn AppleH6CamIn::newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *, IOUserClient **handler) {
  char who[32] = "?";
  if (!handler || type != GUAMP_CAMIOS_CONNECT_TYPE) return kIOReturnBadArgument;
  if (!camios::allowed(who, sizeof(who))) {
    IOLog("GuampCamIos: user client refused for %s\n", who);
    return kIOReturnNotPrivileged;
  }
  AppleH9CamInUserClient *client = OSTypeAlloc(AppleH9CamInUserClient);
  if (!client || !client->initWithTask(owningTask, securityID, type)) { OSSafeReleaseNULL(client); return kIOReturnNoMemory; }
  if (!client->attach(this)) { client->release(); return kIOReturnError; }
  if (!client->start(this)) { client->detach(this); client->release(); return kIOReturnError; }
  IOLog("GuampCamIos: user client opened by %s\n", who);
  *handler = client;
  return kIOReturnSuccess;
}

IOReturn AppleH9CamInUserClient::clientClose() {
  using namespace camios;
  for (uint32_t i = 0; i < gSensors; ++i)
    if (gSensor[i].owner == this) closeSensor(gSensor[i]);
  if (!isInactive()) terminate();
  return kIOReturnSuccess;
}

// type = kGuampCamMemRing + sensor * kGuampCamMemSlotsPerSensor + slot: one RAW slot, read-only.
IOReturn AppleH9CamInUserClient::clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory) {
  using namespace camios;
  if (type < kGuampCamMemRing) return kIOReturnBadArgument;
  const uint32_t idx = (type - kGuampCamMemRing) / kGuampCamMemSlotsPerSensor, slot = (type - kGuampCamMemRing) % kGuampCamMemSlotsPerSensor;
  if (idx >= gSensors || gSensor[idx].owner != this || slot >= gSensor[idx].mode.slotCount || !gSensor[idx].mem[slot])
    return kIOReturnBadArgument;
  gSensor[idx].mem[slot]->retain();
  *memory = gSensor[idx].mem[slot];
  *options |= kIOMapReadOnly;
  return kIOReturnSuccess;
}

IOReturn AppleH9CamInUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *a, IOExternalMethodDispatch *,
                                                OSObject *, void *) {
  using namespace camios;
  if (!a || selector >= kGuampCamSelCount) return kIOReturnBadArgument;
  const uint64_t *in = a->scalarInput;
  const uint32_t n = a->scalarInputCount;
  if (selector == kGuampCamSelGetInfo) {
    if (!a->structureOutput || a->structureOutputSize < sizeof(GuampCamIosInfo)) return kIOReturnBadArgument;
    GuampCamIosInfo info = {};
    info.abiVersion = GUAMP_CAMIOS_ABI_VERSION;
    info.sensorCount = gSensors;
    for (uint32_t i = 0; i < gSensors; ++i) info.sensor[i] = gSensor[i].info;
    memcpy(a->structureOutput, &info, sizeof(info));
    a->structureOutputSize = sizeof(info);
    return kIOReturnSuccess;
  }
  if (selector == kGuampCamSelSetFlash)
    return n >= 3 && flash(static_cast<uint32_t>(in[0]), static_cast<uint32_t>(in[1]), static_cast<uint32_t>(in[2])) == 0
               ? kIOReturnSuccess : kIOReturnBadArgument;
  if (n < 1 || in[0] >= gSensors) return kIOReturnBadArgument;
  const uint32_t idx = static_cast<uint32_t>(in[0]);
  Sensor &s = gSensor[idx];
  if (selector == kGuampCamSelOpen) {
    IOLockLock(gLock);
    bool free = !s.open;
    // CAMSS runs one stream: a second sensor can be opened only while the first is closed.
    for (uint32_t i = 0; free && gBackend == &camss::kBackend && i < gSensors; ++i) free = !gSensor[i].open;
    if (free) { s.open = true; s.owner = this; }
    IOLockUnlock(gLock);
    return free ? kIOReturnSuccess : kIOReturnExclusiveAccess;
  }
  if (s.owner != this) return kIOReturnNotOpen;
  switch (selector) {
    case kGuampCamSelClose: closeSensor(s); return kIOReturnSuccess;
    case kGuampCamSelSetMode: {
      if (n < 4 || !a->structureOutput || a->structureOutputSize < sizeof(GuampCamIosModeInfo)) return kIOReturnBadArgument;
      IOReturn r = setMode(s, idx, static_cast<uint32_t>(in[1]), static_cast<uint32_t>(in[2]), static_cast<uint32_t>(in[3]));
      if (r == kIOReturnSuccess) { memcpy(a->structureOutput, &s.mode, sizeof(s.mode)); a->structureOutputSize = sizeof(s.mode); }
      return r;
    }
    case kGuampCamSelStart: {
      if (!s.mem[0]) return kIOReturnNotReady;
      if (s.streaming) return kIOReturnSuccess;
      IOLockLock(gLock);
      memset(s.state, 0, sizeof(s.state));
      s.queued = 0;
      s.streaming = true;
      IOLockUnlock(gLock);
      if (gBackend->start(gBackendCtx, idx)) { s.streaming = false; return kIOReturnError; }
      return kIOReturnSuccess;
    }
    case kGuampCamSelStop:
      stopSensor(s);
      IOLog("GuampCamIos: sensor %u stopped: %llu frames delivered, %llu dropped\n", idx, static_cast<unsigned long long>(s.delivered),
            static_cast<unsigned long long>(s.dropped));
      return kIOReturnSuccess;
    case kGuampCamSelWaitFrame: {
      if (!a->structureOutput || a->structureOutputSize < sizeof(GuampCamIosFrame)) return kIOReturnBadArgument;
      GuampCamIosFrame f = {};
      IOReturn r = waitFrame(s, n >= 2 ? static_cast<uint32_t>(in[1]) : 200, &f);
      if (r == kIOReturnSuccess) { memcpy(a->structureOutput, &f, sizeof(f)); a->structureOutputSize = sizeof(f); }
      return r;
    }
    case kGuampCamSelReleaseFrame: {
      if (n < 2 || in[1] >= s.mode.slotCount) return kIOReturnBadArgument;
      IOLockLock(gLock);
      const bool mine = s.state[in[1]] == kClient;
      if (mine) s.state[in[1]] = kFree;
      IOLockUnlock(gLock);
      if (mine && gBackend->release) gBackend->release(gBackendCtx, idx, static_cast<uint32_t>(in[1]));
      return kIOReturnSuccess;
    }
    case kGuampCamSelSetExposure:
      if (n < 4) return kIOReturnBadArgument;
      s.exposureUs = static_cast<uint32_t>(in[1]);
      s.againQ8 = static_cast<uint32_t>(in[2]);
      s.dgainQ8 = static_cast<uint32_t>(in[3]);
      return gBackend->setExposure(gBackendCtx, idx, s.exposureUs, s.againQ8, s.dgainQ8) ? kIOReturnError : kIOReturnSuccess;
    case kGuampCamSelSetFocus:
      if (n < 2) return kIOReturnBadArgument;
      s.focusDac = static_cast<uint32_t>(in[1] > 1023 ? 1023 : in[1]);
      return gBackend->setFocus(gBackendCtx, idx, s.focusDac) ? kIOReturnError : kIOReturnSuccess;
    default: return kIOReturnUnsupported;
  }
}

namespace camios {
void start() {
  PE_parse_boot_argn("guamp-camera-ios", &gMode, sizeof(gMode));
  if (gMode != 1 && gMode != 2) return;
  PE_parse_boot_argn("guamp-camera-ios-slots", &gSlots, sizeof(gSlots));
  if (gSlots < 2) gSlots = 2;
  if (gSlots > GUAMP_CAMSS_MAX_BUFFERS) gSlots = GUAMP_CAMSS_MAX_BUFFERS;
  gLock = IOLockAlloc();
  gFlashOff = thread_call_allocate(flashOff, nullptr);
  if (!gLock) return;
  gBackend = &pattern::kBackend;
  if (gMode == 1) {
    if (camss::available()) gBackend = &camss::kBackend;
    else IOLog("GuampCamIos: CAMSS API not in this kernel, test pattern instead\n");
  }
  probeSensors();
  IOService *platform = IOService::getPlatform();
  AppleH6CamIn *svc = OSTypeAlloc(AppleH6CamIn);
  if (!platform || !svc || !svc->init()) { OSSafeReleaseNULL(svc); IOLog("GuampCamIos: service not created\n"); return; }
  svc->setName("isp");
  svc->setProperty(kIOUserClientClassKey, GUAMP_CAMIOS_USERCLIENT_CLASS);
  svc->setProperty("GuampCamIosABI", GUAMP_CAMIOS_ABI_VERSION, 32);
  if (!svc->attach(platform) || !svc->start(platform)) { svc->detach(platform); svc->release(); IOLog("GuampCamIos: service not started\n"); return; }
  svc->registerService();
  gService = svc;
  IOLog("GuampCamIos: AppleH6CamIn published (mode %u, backend %s, %u slots), plug-in /System/Library/MediaCapture/H6ISP.mediacapture\n",
        gMode, gBackend->name, gSlots);
}
}  // namespace camios
