// GuampCamIosAbi.h - contract between the kernel camera service (drivers/NativeEmmc/GuampCamIos.inc) and the iOS capture
// plug-in (camera-ios/GuampISP, published as /System/Library/MediaCapture/H6ISP.mediacapture). Plain C, no kernel or
// CoreFoundation headers, so the same file builds into the kernel, the plug-in and the host tests.
//
// Why "AppleH6CamIn": CMCapture's FigCaptureGetCaptureDeviceCreateFunction walks a fixed table
//   {AppleH16CamIn, H16ISP}, {AppleH13CamIn, H13ISP}, {AppleH10CamIn, H10ISP}, {AppleH9CamIn, H9ISP}, {AppleH6CamIn, H6ISP}
// and, for the first IOServiceMatching(<class>) that finds a service, dlopen()s /System/Library/MediaCapture/<n>ISP.mediacapture
// and calls dlsym(<n>ISPCaptureDeviceCreate). H9ISP.mediacapture lives in the iPad7,1 dyld cache (it would drive the Apple H9
// firmware protocol); H6ISP.mediacapture does not exist on iOS 17, so a file at that path is ours and no Apple binary changes.
//
// Frame model: the kernel owns a ring of RAW Bayer slots (filled by CAMSS TFE RDI, agent 2, or by the built-in test pattern);
// user space waits for a filled slot, reads it through a read-only mapping, runs the software ISP (gisp) and the 3A, and
// releases the slot (cam3-isp.md section 2: no pixel work in the kernel). Exposure, gain, focus and flash go back down through the same user client.
#ifndef GUAMP_CAM_IOS_ABI_H
#define GUAMP_CAM_IOS_ABI_H

#include <stdint.h>

#define GUAMP_CAMIOS_ABI_VERSION      1u
#define GUAMP_CAMIOS_SERVICE_CLASS    "AppleH6CamIn"            // IOServiceMatching() key used by CMCapture
// The user client class name is what sandbox profiles filter on (iokit-user-client-class). mediaserverd on iPad7,1 is
// expected to allow the H9 one; keep this name unless a sandbox report says otherwise.
#define GUAMP_CAMIOS_USERCLIENT_CLASS "AppleH9CamInUserClient"
#define GUAMP_CAMIOS_CONNECT_TYPE     0x47434931u               // 'GCI1', IOServiceOpen type
#define GUAMP_CAMIOS_MAX_SENSORS      4u
#define GUAMP_CAMIOS_MAX_SLOTS        8u

// Sensor positions (stream order in kFigCaptureDeviceProperty_StreamArray).
enum GuampCamIosPosition { kGuampCamBack = 0, kGuampCamFront = 1, kGuampCamDepth = 2, kGuampCamMacro = 3 };

// RAW layouts a slot can carry.
enum GuampCamIosRawFormat {
  kGuampRawMipi10 = 0,   // MIPI CSI-2 RAW10, 4 pixels in 5 bytes (TFE RDI "plain" write of the packet payload)
  kGuampRawU16    = 1,   // 16-bit little endian, value in the low `bitDepth` bits (TFE unpack mode, test pattern)
};

// CFA order of the top-left 2x2 block.
enum GuampCamIosBayer { kGuampBayerRGGB = 0, kGuampBayerGRBG = 1, kGuampBayerGBRG = 2, kGuampBayerBGGR = 3 };

// Sensor modes. The plug-in maps every FigCaptureStream format to one of these (see camera-ios/GuampFormats.c).
enum GuampCamIosMode {
  kGuampModePreview = 0,  // binned, >= 30 fps (GM1ST 2000x1500 / 4H7 1632x1224 class)
  kGuampModeVideo   = 1,  // 16:9 crop for 1080p
  kGuampModePhoto   = 2,  // full or 2x2-binned 12 MP still mode, low fps
  kGuampModeCount   = 3,
};

// External selector numbers of the user client (IOConnectCallScalarMethod / IOConnectCallStructMethod).
enum GuampCamIosSelector {
  kGuampCamSelGetInfo      = 0,  // struct out GuampCamIosInfo
  kGuampCamSelOpen         = 1,  // scalar in: sensor index; exclusive per sensor
  kGuampCamSelClose        = 2,  // scalar in: sensor index
  kGuampCamSelSetMode      = 3,  // scalar in: sensor, mode, max fps (Q8), min fps (Q8); struct out GuampCamIosModeInfo
  kGuampCamSelStart        = 4,  // scalar in: sensor
  kGuampCamSelStop         = 5,  // scalar in: sensor
  kGuampCamSelWaitFrame    = 6,  // scalar in: sensor, timeout ms; struct out GuampCamIosFrame (kIOReturnTimeout if none)
  kGuampCamSelReleaseFrame = 7,  // scalar in: sensor, slot
  kGuampCamSelSetExposure  = 8,  // scalar in: sensor, exposure us, analog gain Q8, digital gain Q8 (applied at a frame start)
  kGuampCamSelSetFocus     = 9,  // scalar in: sensor, actuator DAC (0..1023, DW9767 on the back camera)
  kGuampCamSelSetFlash     = 10, // scalar in: mode (GuampCamIosFlash), level permille / current mA, duration ms
  kGuampCamSelCount        = 11,
};

// SetFlash: Torch level = permille; Pre (pre-flash for metering) and Pulse (main flash) level = current mA, duration ms.
enum GuampCamIosFlash { kGuampFlashOff = 0, kGuampFlashTorch = 1, kGuampFlashPulse = 2, kGuampFlashPre = 3 };

// Memory types for IOConnectMapMemory64.
enum GuampCamIosMemory {
  kGuampCamMemRing  = 0x100,   // + sensor * kGuampCamMemSlotsPerSensor + slot: one RAW slot, read-only (zero copy:
                               //   with CAMSS these are the TFE RDI buffers themselves, Normal non-cacheable)
  kGuampCamMemSlotsPerSensor = 16,
};

typedef struct GuampCamIosSensorInfo {
  uint32_t present;          // chip id read back (or test pattern)
  uint32_t position;         // GuampCamIosPosition
  uint32_t chipId;           // 0x08d1 GM1ST, 0x487b 4H7, 0x2b OV02B ... 0 for the test pattern
  uint32_t fullWidth, fullHeight;
  uint32_t bayer;            // GuampCamIosBayer
  uint32_t bitDepth;         // 10
  uint32_t hasAutoFocus;     // DW9767 present
  uint32_t hasFlash;         // PMI632 flash on this side
  uint32_t focalLengthUm;    // lens focal length in micrometres (EXIF), 0 unknown
  uint32_t fNumberQ8;        // f-number Q8, 0 unknown
  uint32_t pixelSizeNm;      // sensor pixel pitch in nanometres (0.8 um GM1ST = 800)
  uint32_t testPattern;      // 1 = frames come from the built-in generator, not from CAMSS
  uint32_t reserved[3];
} GuampCamIosSensorInfo;

typedef struct GuampCamIosInfo {
  uint32_t abiVersion;       // GUAMP_CAMIOS_ABI_VERSION
  uint32_t sensorCount;
  GuampCamIosSensorInfo sensor[GUAMP_CAMIOS_MAX_SENSORS];
} GuampCamIosInfo;

typedef struct GuampCamIosModeInfo {
  uint32_t width, height;    // RAW frame size delivered in each slot
  uint32_t rawFormat;        // GuampCamIosRawFormat
  uint32_t bayer;            // CFA order after any sensor mirror/flip
  uint32_t strideBytes;      // bytes per RAW line
  uint32_t slotStride;       // bytes between slots in the kGuampCamMemRing mapping (page aligned)
  uint32_t slotCount;        // <= GUAMP_CAMIOS_MAX_SLOTS
  uint32_t maxFpsQ8, minFpsQ8;
  uint32_t minExposureUs, maxExposureUs;
  uint32_t maxAnalogGainQ8, maxDigitalGainQ8;
  uint32_t blackLevel;       // in RAW codes at bitDepth
  uint32_t lineTimeNs;       // for rolling shutter metadata
  uint32_t binning;          // 1, 2 (2x2) or 4
  uint32_t cropX, cropY;     // active area inside the full sensor, in full-resolution pixels
  uint32_t lensMin, lensMax; // focus DAC range (EEPROM infinity .. macro), 0/0 = no AF
} GuampCamIosModeInfo;

typedef struct GuampCamIosFrame {
  uint32_t slot;             // index into the ring; owned by the client until kGuampCamSelReleaseFrame
  uint32_t flags;            // bit 0: corrupt/short frame, bit 1: flash fired during exposure, bit 2: test pattern
  uint64_t sequence;         // sensor frame counter (gaps = dropped frames)
  uint64_t timestamp;        // mach_absolute_time() at start of frame (SOF IRQ), same clock as CMClockGetHostTimeClock
  uint32_t exposureUs;       // applied values for this frame
  uint32_t analogGainQ8;
  uint32_t digitalGainQ8;
  uint32_t focusDac;
  uint32_t width, height, strideBytes, rawFormat;   // authoritative geometry of this frame (CAMSS reports it per frame)
  int32_t  temperatureMilliC;// sensor temperature if readable, else INT32_MIN
  uint32_t bayer;            // GuampCamIosBayer of this frame
  uint32_t reserved[4];
} GuampCamIosFrame;

#ifdef __cplusplus
static_assert(sizeof(GuampCamIosSensorInfo) == 64, "abi");
static_assert(sizeof(GuampCamIosFrame) == 80, "abi");
#endif

#endif  // GUAMP_CAM_IOS_ABI_H
