// SPDX-License-Identifier: BSD-2-Clause-Patent
// GuampAudio.driver: the "null audio device" step of the guamp iOS audio plan (analisi/audio/IOS-AUDIO-LATO-UTENTE-20261028.md,
// route (c)). An AudioServerPlugIn loaded by the HAL inside audiomxd (iPadOS 17.7.11, 21H461) from
// /System/Library/Audio/Plug-Ins/HAL. It presents the built-in devices VirtualAudio knows by UID, so that it builds the
// Built-In Speaker / Microphone ports instead of leaving everything on Null_Device:
//   "Speaker"  stereo output (+ a silent stereo input, the amplifier sense of an iPhone/iPad), volume control 'vlme'
//   "Hawking"  one silent input (VirtualAudio's 'vhaw' companion device; J120 has an audio-hawking-device node)
//   "Codec"    one silent input with a data source selector (VirtualAudio asks for DataSources on the codec input)
// This step plays into nowhere and records silence: WriteMix is dropped, ReadInput returns zeros. Step 4 replaces the two
// marked places with the kernel bridge (/dev/guamp-audio ring, s16 48 kHz, see DESIGN.md); nothing else changes then.
// Clock: exactly kRate frames per second of mach_absolute_time(), zero time stamp every kRingFrames frames.
// No Apple code: written from the public macOS AudioServerPlugIn documentation (declarations in GuampAudioServerPlugIn.h).
// Diagnostics: os_log subsystem "com.guamp.audio" (Initialize, devices, StartIO/StopIO, every property it does not know,
// once per selector: the list that tells which properties VirtualAudio of J120 really needs, plan step 3).
#include "GuampAudioServerPlugIn.h"
#include <mach/mach_time.h>
#include <os/log.h>
#include <pthread.h>
#include <string.h>

enum {
  kPlugIn = kAudioObjectPlugInObject,
  kSpeaker = 2, kSpeakerOut, kSpeakerIn, kSpeakerVolume,
  kHawking, kHawkingIn,
  kCodec, kCodecIn, kCodecSource,
  kObjectCount
};
static const Float64 kRate = 48000.0;
static const UInt32 kRingFrames = 16384, kSafetyFrames = 0, kLatencyFrames = 0;
static const Float32 kMinDb = -48.0f, kMaxDb = 0.0f;   // 16 iOS volume steps of 3 dB
static const UInt32 kSourceInternalMic = 'imic';       // to confirm against VirtualAudio's queries (step 3)

typedef struct {
  AudioObjectID object, out, in, volume, source;
  UInt32 outChannels, inChannels;
  CFStringRef uid, name;
  UInt32 running;        // StartIO count
  UInt64 anchor, cycles;  // zero time stamp state while running
} Device;

static Device gDevices[] = {
  {kSpeaker, kSpeakerOut, kSpeakerIn, kSpeakerVolume, 0, 2, 2, CFSTR("Speaker"), CFSTR("Speaker"), 0, 0, 0},
  {kHawking, 0, kHawkingIn, 0, 0, 0, 1, CFSTR("Hawking"), CFSTR("Hawking"), 0, 0, 0},
  {kCodec, 0, kCodecIn, 0, kCodecSource, 0, 1, CFSTR("Codec"), CFSTR("Codec"), 0, 0, 0},
};
static const UInt32 kDeviceCount = sizeof(gDevices) / sizeof(gDevices[0]);

static pthread_mutex_t gLock = PTHREAD_MUTEX_INITIALIZER;
static AudioServerPlugInHostRef gHost;
static Float64 gTicksPerFrame;
static Float32 gVolumeDb = kMaxDb;
static UInt32 gSourceItem = kSourceInternalMic;
static UInt32 gRefs = 1;
static os_log_t gLog;
static UInt32 gUnknownSeen[64];   // selectors already reported as unknown (diagnostics only)

// ---- object model -------------------------------------------------------------------------------------------------------

static Device *deviceOwning(AudioObjectID object) {
  for (UInt32 i = 0; i < kDeviceCount; i++) {
    Device *d = &gDevices[i];
    if (object == d->object || (object && (object == d->out || object == d->in || object == d->volume || object == d->source)))
      return d;
  }
  return NULL;
}
static Boolean isStream(const Device *d, AudioObjectID o) { return o && (o == d->out || o == d->in); }
static UInt32 streamChannels(const Device *d, AudioObjectID o) { return o == d->out ? d->outChannels : d->inChannels; }

static AudioStreamBasicDescription format(UInt32 channels) {
  AudioStreamBasicDescription f;
  memset(&f, 0, sizeof(f));
  f.mSampleRate = kRate;
  f.mFormatID = kAudioFormatLinearPCM;
  f.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagsNativeEndian | kAudioFormatFlagIsPacked;
  f.mBytesPerPacket = f.mBytesPerFrame = 4 * channels;
  f.mFramesPerPacket = 1;
  f.mChannelsPerFrame = channels;
  f.mBitsPerChannel = 32;
  return f;
}

// ---- replies: out == NULL means "size only" ---------------------------------------------------------------------------

typedef struct {
  void *out;
  UInt32 cap, used;
  OSStatus status;
} Reply;
static void put(Reply *r, const void *value, UInt32 bytes) {
  if (r->out) {
    if (bytes > r->cap) { r->status = kAudioHardwareBadPropertySizeError; return; }
    memcpy(r->out, value, bytes);
  }
  r->used = bytes;
}
static void putU32(Reply *r, UInt32 v) { put(r, &v, sizeof(v)); }
static void putF64(Reply *r, Float64 v) { put(r, &v, sizeof(v)); }
static void putF32(Reply *r, Float32 v) { put(r, &v, sizeof(v)); }
static void putString(Reply *r, CFStringRef s) {   // the caller releases what it gets
  if (r->out && r->cap >= sizeof(s)) CFRetain(s);
  put(r, &s, sizeof(s));
}
static void putObjects(Reply *r, const AudioObjectID *ids, UInt32 count) {   // as many as fit (HAL convention)
  UInt32 n = r->out ? r->cap / sizeof(AudioObjectID) : count;
  if (n > count) n = count;
  put(r, ids, n * sizeof(AudioObjectID));
}

static void unknown(AudioObjectID object, const AudioObjectPropertyAddress *a) {
  for (UInt32 i = 0; i < sizeof(gUnknownSeen) / sizeof(gUnknownSeen[0]); i++) {
    if (gUnknownSeen[i] == a->mSelector) return;
    if (!gUnknownSeen[i]) { gUnknownSeen[i] = a->mSelector; break; }
  }
  os_log(gLog, "GuampAudio: unknown property %{public}.4s scope %{public}.4s on object %u", (const char *)&a->mSelector,
         (const char *)&a->mScope, object);
}

static UInt32 deviceStreams(const Device *d, AudioObjectPropertyScope scope, AudioObjectID ids[2]) {
  UInt32 n = 0;
  if (d->out && scope != kAudioObjectPropertyScopeInput) ids[n++] = d->out;
  if (d->in && scope != kAudioObjectPropertyScopeOutput) ids[n++] = d->in;
  return n;
}

// One table for HasProperty, GetPropertyDataSize and GetPropertyData. Returns kAudioHardwareUnknownPropertyError when
// the object does not have the property.
static OSStatus property(AudioObjectID object, const AudioObjectPropertyAddress *a, UInt32 qualSize, const void *qual,
                         Reply *r) {
  r->status = kAudioHardwareNoError;
  const UInt32 sel = a->mSelector;
  if (object == kPlugIn) {
    AudioObjectID list[3] = {kSpeaker, kHawking, kCodec};
    switch (sel) {
      case kAudioObjectPropertyBaseClass: putU32(r, kAudioObjectClassID); break;
      case kAudioObjectPropertyClass: putU32(r, kAudioPlugInClassID); break;
      case kAudioObjectPropertyOwner: putU32(r, kAudioObjectUnknown); break;
      case kAudioObjectPropertyManufacturer: putString(r, CFSTR("guamp")); break;
      case kAudioObjectPropertyOwnedObjects:
      case kAudioPlugInPropertyDeviceList: putObjects(r, list, kDeviceCount); break;
      case kAudioPlugInPropertyBoxList: putObjects(r, list, 0); break;
      case kAudioPlugInPropertyResourceBundle: putString(r, CFSTR("")); break;
      case kAudioPlugInPropertyTranslateUIDToDevice: {
        AudioObjectID found = kAudioObjectUnknown;
        if (qual && qualSize == sizeof(CFStringRef))
          for (UInt32 i = 0; i < kDeviceCount; i++)
            if (CFEqual(*(const CFStringRef *)qual, gDevices[i].uid)) found = gDevices[i].object;
        putU32(r, found);
        break;
      }
      default: return kAudioHardwareUnknownPropertyError;
    }
    return r->status;
  }
  Device *d = deviceOwning(object);
  if (!d) return kAudioHardwareBadObjectError;
  if (object == d->object) {
    AudioObjectID ids[4];
    UInt32 n;
    switch (sel) {
      case kAudioObjectPropertyBaseClass: putU32(r, kAudioObjectClassID); break;
      case kAudioObjectPropertyClass: putU32(r, kAudioDeviceClassID); break;
      case kAudioObjectPropertyOwner: putU32(r, kPlugIn); break;
      case kAudioObjectPropertyName: putString(r, d->name); break;
      case kAudioObjectPropertyManufacturer: putString(r, CFSTR("Apple Inc.")); break;
      case kAudioDevicePropertyDeviceUID:
      case kAudioDevicePropertyModelUID: putString(r, d->uid); break;
      case kAudioDevicePropertyTransportType: putU32(r, kAudioDeviceTransportTypeBuiltIn); break;
      case kAudioDevicePropertyRelatedDevices: putObjects(r, &d->object, 1); break;
      case kAudioDevicePropertyClockDomain: putU32(r, 0); break;
      case kAudioDevicePropertyDeviceIsAlive: putU32(r, 1); break;
      case kAudioDevicePropertyDeviceIsRunning: putU32(r, d->running ? 1 : 0); break;
      case kAudioDevicePropertyDeviceCanBeDefaultDevice: putU32(r, d->object == kSpeaker || d->object == kCodec); break;
      case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice: putU32(r, d->object == kSpeaker); break;
      case kAudioDevicePropertyLatency: putU32(r, kLatencyFrames); break;
      case kAudioDevicePropertySafetyOffset: putU32(r, kSafetyFrames); break;
      case kAudioDevicePropertyNominalSampleRate: putF64(r, kRate); break;
      case kAudioDevicePropertyAvailableNominalSampleRates: {
        AudioValueRange range = {kRate, kRate};
        put(r, &range, sizeof(range));
        break;
      }
      case kAudioDevicePropertyIsHidden: putU32(r, 0); break;
      case kAudioDevicePropertyZeroTimeStampPeriod: putU32(r, kRingFrames); break;
      case kAudioDevicePropertyPreferredChannelsForStereo: {
        UInt32 pair[2] = {1, 2};
        put(r, pair, sizeof(pair));
        break;
      }
      case kAudioDevicePropertyStreams:
        n = deviceStreams(d, a->mScope, ids);
        putObjects(r, ids, n);
        break;
      case kAudioObjectPropertyControlList:
        n = 0;
        if (d->volume) ids[n++] = d->volume;
        if (d->source) ids[n++] = d->source;
        putObjects(r, ids, n);
        break;
      case kAudioObjectPropertyOwnedObjects:
        n = deviceStreams(d, a->mScope, ids);
        if (d->volume && a->mScope != kAudioObjectPropertyScopeInput) ids[n++] = d->volume;
        if (d->source && a->mScope != kAudioObjectPropertyScopeOutput) ids[n++] = d->source;
        putObjects(r, ids, n);
        break;
      default: return kAudioHardwareUnknownPropertyError;
    }
    return r->status;
  }
  if (isStream(d, object)) {
    const UInt32 channels = streamChannels(d, object);
    const Boolean input = object == d->in;
    switch (sel) {
      case kAudioObjectPropertyBaseClass: putU32(r, kAudioObjectClassID); break;
      case kAudioObjectPropertyClass: putU32(r, kAudioStreamClassID); break;
      case kAudioObjectPropertyOwner: putU32(r, d->object); break;
      case kAudioStreamPropertyIsActive: putU32(r, 1); break;
      case kAudioStreamPropertyDirection: putU32(r, input ? 1 : 0); break;
      case kAudioStreamPropertyTerminalType:
        putU32(r, input ? kAudioStreamTerminalTypeMicrophone : kAudioStreamTerminalTypeSpeaker);
        break;
      case kAudioStreamPropertyStartingChannel: putU32(r, 1); break;
      case kAudioStreamPropertyLatency: putU32(r, kLatencyFrames); break;
      case kAudioStreamPropertyVirtualFormat:
      case kAudioStreamPropertyPhysicalFormat: {
        AudioStreamBasicDescription f = format(channels);
        put(r, &f, sizeof(f));
        break;
      }
      case kAudioStreamPropertyAvailableVirtualFormats:
      case kAudioStreamPropertyAvailablePhysicalFormats: {
        AudioStreamRangedDescription f;
        f.mFormat = format(channels);
        f.mSampleRateRange.mMinimum = f.mSampleRateRange.mMaximum = kRate;
        put(r, &f, sizeof(f));
        break;
      }
      default: return kAudioHardwareUnknownPropertyError;
    }
    return r->status;
  }
  if (object == d->volume) {
    switch (sel) {
      case kAudioObjectPropertyBaseClass: putU32(r, kAudioLevelControlClassID); break;
      case kAudioObjectPropertyClass: putU32(r, kAudioVolumeControlClassID); break;
      case kAudioObjectPropertyOwner: putU32(r, d->object); break;
      case kAudioControlPropertyScope: putU32(r, kAudioObjectPropertyScopeOutput); break;
      case kAudioControlPropertyElement: putU32(r, kAudioObjectPropertyElementMain); break;
      case kAudioLevelControlPropertyScalarValue: putF32(r, (gVolumeDb - kMinDb) / (kMaxDb - kMinDb)); break;
      case kAudioLevelControlPropertyDecibelValue: putF32(r, gVolumeDb); break;
      case kAudioLevelControlPropertyDecibelRange: {
        AudioValueRange range = {kMinDb, kMaxDb};
        put(r, &range, sizeof(range));
        break;
      }
      case kAudioLevelControlPropertyConvertScalarToDecibels:
      case kAudioLevelControlPropertyConvertDecibelsToScalar: {
        // In-out property: the caller's value arrives in the output buffer (as on macOS)
        Float32 v = r->out && r->cap >= sizeof(Float32) ? *(Float32 *)r->out : 0;
        if (v != v) v = 0;   // NaN
        if (sel == kAudioLevelControlPropertyConvertScalarToDecibels) {
          v = v < 0 ? 0 : v > 1 ? 1 : v;
          v = kMinDb + v * (kMaxDb - kMinDb);
        } else {
          v = v < kMinDb ? kMinDb : v > kMaxDb ? kMaxDb : v;
          v = (v - kMinDb) / (kMaxDb - kMinDb);
        }
        putF32(r, v);
        break;
      }
      default: return kAudioHardwareUnknownPropertyError;
    }
    return r->status;
  }
  // data source selector of the codec input
  switch (sel) {
    case kAudioObjectPropertyBaseClass: putU32(r, kAudioSelectorControlClassID); break;
    case kAudioObjectPropertyClass: putU32(r, kAudioDataSourceControlClassID); break;
    case kAudioObjectPropertyOwner: putU32(r, d->object); break;
    case kAudioControlPropertyScope: putU32(r, kAudioObjectPropertyScopeInput); break;
    case kAudioControlPropertyElement: putU32(r, kAudioObjectPropertyElementMain); break;
    case kAudioSelectorControlPropertyCurrentItem: putU32(r, gSourceItem); break;
    case kAudioSelectorControlPropertyAvailableItems: putU32(r, kSourceInternalMic); break;
    case kAudioSelectorControlPropertyItemName:
      if (!qual || qualSize != sizeof(UInt32) || *(const UInt32 *)qual != kSourceInternalMic) return kAudioHardwareIllegalOperationError;
      putString(r, CFSTR("Microphone"));
      break;
    default: return kAudioHardwareUnknownPropertyError;
  }
  return r->status;
}

// ---- COM entry points ---------------------------------------------------------------------------------------------------

static AudioServerPlugInDriverInterface gInterface;
static AudioServerPlugInDriverInterface *gInterfacePtr = &gInterface;

static HRESULT queryInterface(void *self, REFIID iid, LPVOID *out) {
  if (!out) return E_POINTER;
  CFUUIDRef requested = CFUUIDCreateFromUUIDBytes(NULL, iid);
  const Boolean ours = requested && (CFEqual(requested, IUnknownUUID) || CFEqual(requested, kAudioServerPlugInDriverInterfaceUUID));
  if (requested) CFRelease(requested);
  if (!ours) { *out = NULL; return E_NOINTERFACE; }
  pthread_mutex_lock(&gLock);
  gRefs++;
  pthread_mutex_unlock(&gLock);
  *out = self;
  return S_OK;
}
static ULONG addRef(void *self) {
  (void)self;
  pthread_mutex_lock(&gLock);
  const ULONG n = ++gRefs;
  pthread_mutex_unlock(&gLock);
  return n;
}
static ULONG release(void *self) {
  (void)self;
  pthread_mutex_lock(&gLock);
  const ULONG n = gRefs > 1 ? --gRefs : gRefs;   // the driver object is static: never freed
  pthread_mutex_unlock(&gLock);
  return n;
}

static OSStatus initialize(AudioServerPlugInDriverRef driver, AudioServerPlugInHostRef host) {
  (void)driver;
  gHost = host;
  mach_timebase_info_data_t tb;
  mach_timebase_info(&tb);
  gTicksPerFrame = (1.0e9 / kRate) * (Float64)tb.denom / (Float64)tb.numer;
  os_log(gLog, "GuampAudio: initialized, %u devices (Speaker, Hawking, Codec), %.0f Hz, ring %u frames, null I/O",
         kDeviceCount, kRate, kRingFrames);
  return kAudioHardwareNoError;
}
static OSStatus createDevice(AudioServerPlugInDriverRef d, CFDictionaryRef desc, const AudioServerPlugInClientInfo *c,
                             AudioObjectID *out) {
  (void)d; (void)desc; (void)c; (void)out;
  return kAudioHardwareUnsupportedOperationError;
}
static OSStatus destroyDevice(AudioServerPlugInDriverRef d, AudioObjectID id) {
  (void)d; (void)id;
  return kAudioHardwareUnsupportedOperationError;
}
static OSStatus addClient(AudioServerPlugInDriverRef d, AudioObjectID id, const AudioServerPlugInClientInfo *c) {
  (void)d; (void)c;
  return deviceOwning(id) && deviceOwning(id)->object == id ? kAudioHardwareNoError : kAudioHardwareBadObjectError;
}
static OSStatus removeClient(AudioServerPlugInDriverRef d, AudioObjectID id, const AudioServerPlugInClientInfo *c) {
  return addClient(d, id, c);
}
static OSStatus configChange(AudioServerPlugInDriverRef d, AudioObjectID id, UInt64 action, void *info) {
  (void)d; (void)id; (void)action; (void)info;
  return kAudioHardwareNoError;
}

static Boolean hasProperty(AudioServerPlugInDriverRef d, AudioObjectID object, pid_t pid, const AudioObjectPropertyAddress *a) {
  (void)d; (void)pid;
  if (!a) return false;
  Reply r = {NULL, 0, 0, 0};
  pthread_mutex_lock(&gLock);
  // a qualifier is needed only by the translate/item-name properties: they exist, their data just cannot be computed here
  const OSStatus s = property(object, a, 0, NULL, &r);
  pthread_mutex_unlock(&gLock);
  return s == kAudioHardwareNoError || s == kAudioHardwareIllegalOperationError;
}
static OSStatus isSettable(AudioServerPlugInDriverRef d, AudioObjectID object, pid_t pid, const AudioObjectPropertyAddress *a,
                           Boolean *out) {
  if (!a || !out) return kAudioHardwareIllegalOperationError;
  if (!hasProperty(d, object, pid, a)) return kAudioHardwareUnknownPropertyError;
  const UInt32 s = a->mSelector;
  *out = (object == kSpeaker || object == kHawking || object == kCodec) ? s == kAudioDevicePropertyNominalSampleRate
       : object == kSpeakerVolume ? s == kAudioLevelControlPropertyScalarValue || s == kAudioLevelControlPropertyDecibelValue
       : object == kCodecSource   ? s == kAudioSelectorControlPropertyCurrentItem
       : s == kAudioStreamPropertyVirtualFormat || s == kAudioStreamPropertyPhysicalFormat;
  return kAudioHardwareNoError;
}
static OSStatus getSize(AudioServerPlugInDriverRef d, AudioObjectID object, pid_t pid, const AudioObjectPropertyAddress *a,
                        UInt32 qualSize, const void *qual, UInt32 *out) {
  (void)d; (void)pid;
  if (!a || !out) return kAudioHardwareIllegalOperationError;
  Reply r = {NULL, 0, 0, 0};
  pthread_mutex_lock(&gLock);
  const OSStatus s = property(object, a, qualSize, qual, &r);
  pthread_mutex_unlock(&gLock);
  if (s == kAudioHardwareUnknownPropertyError) unknown(object, a);
  if (s == kAudioHardwareIllegalOperationError) { *out = sizeof(CFStringRef); return kAudioHardwareNoError; }
  *out = r.used;
  return s;
}
static OSStatus getData(AudioServerPlugInDriverRef d, AudioObjectID object, pid_t pid, const AudioObjectPropertyAddress *a,
                        UInt32 qualSize, const void *qual, UInt32 inSize, UInt32 *outSize, void *out) {
  (void)d; (void)pid;
  if (!a || !outSize || !out) return kAudioHardwareIllegalOperationError;
  Reply r = {out, inSize, 0, 0};
  pthread_mutex_lock(&gLock);
  const OSStatus s = property(object, a, qualSize, qual, &r);
  pthread_mutex_unlock(&gLock);
  if (s == kAudioHardwareUnknownPropertyError) unknown(object, a);
  *outSize = r.used;
  return s;
}

static void changed(AudioObjectID object, AudioObjectPropertySelector sel, AudioObjectPropertyScope scope) {
  AudioObjectPropertyAddress a = {sel, scope, kAudioObjectPropertyElementMain};
  if (gHost) gHost->PropertiesChanged(gHost, object, 1, &a);
}

static OSStatus setData(AudioServerPlugInDriverRef d, AudioObjectID object, pid_t pid, const AudioObjectPropertyAddress *a,
                        UInt32 qualSize, const void *qual, UInt32 inSize, const void *in) {
  (void)d; (void)pid; (void)qualSize; (void)qual;
  if (!a || !in) return kAudioHardwareIllegalOperationError;
  const UInt32 sel = a->mSelector;
  Device *dev = deviceOwning(object);
  if (!dev) return kAudioHardwareBadObjectError;
  if (object == dev->object && sel == kAudioDevicePropertyNominalSampleRate) {
    if (inSize != sizeof(Float64)) return kAudioHardwareBadPropertySizeError;
    return *(const Float64 *)in == kRate ? kAudioHardwareNoError : kAudioDeviceUnsupportedFormatError;
  }
  if (isStream(dev, object) && (sel == kAudioStreamPropertyVirtualFormat || sel == kAudioStreamPropertyPhysicalFormat)) {
    if (inSize != sizeof(AudioStreamBasicDescription)) return kAudioHardwareBadPropertySizeError;
    const AudioStreamBasicDescription want = format(streamChannels(dev, object)), *got = in;
    return got->mSampleRate == want.mSampleRate && got->mFormatID == want.mFormatID && got->mFormatFlags == want.mFormatFlags &&
           got->mChannelsPerFrame == want.mChannelsPerFrame && got->mBitsPerChannel == want.mBitsPerChannel
         ? kAudioHardwareNoError : kAudioDeviceUnsupportedFormatError;
  }
  if (object == kSpeakerVolume && (sel == kAudioLevelControlPropertyScalarValue || sel == kAudioLevelControlPropertyDecibelValue)) {
    if (inSize != sizeof(Float32)) return kAudioHardwareBadPropertySizeError;
    Float32 v = *(const Float32 *)in;
    if (v != v) return kAudioHardwareIllegalOperationError;
    if (sel == kAudioLevelControlPropertyScalarValue) v = kMinDb + (v < 0 ? 0 : v > 1 ? 1 : v) * (kMaxDb - kMinDb);
    v = v < kMinDb ? kMinDb : v > kMaxDb ? kMaxDb : v;
    pthread_mutex_lock(&gLock);
    const Boolean differs = v != gVolumeDb;
    gVolumeDb = v;
    pthread_mutex_unlock(&gLock);
    if (differs) {
      // step 4: the kernel bridge gets the level here (ioctl), as hackios' HAKIOS_AUDIO_SET_VOLUME
      changed(object, kAudioLevelControlPropertyScalarValue, kAudioObjectPropertyScopeGlobal);
      changed(object, kAudioLevelControlPropertyDecibelValue, kAudioObjectPropertyScopeGlobal);
    }
    return kAudioHardwareNoError;
  }
  if (object == kCodecSource && sel == kAudioSelectorControlPropertyCurrentItem) {
    if (inSize != sizeof(UInt32)) return kAudioHardwareBadPropertySizeError;
    if (*(const UInt32 *)in != kSourceInternalMic) return kAudioHardwareIllegalOperationError;
    gSourceItem = kSourceInternalMic;
    return kAudioHardwareNoError;
  }
  return kAudioHardwareUnknownPropertyError;
}

static OSStatus startIO(AudioServerPlugInDriverRef d, AudioObjectID device, UInt32 client) {
  (void)d; (void)client;
  Device *dev = deviceOwning(device);
  if (!dev || dev->object != device) return kAudioHardwareBadObjectError;
  pthread_mutex_lock(&gLock);
  if (!dev->running++) {
    dev->anchor = mach_absolute_time();
    dev->cycles = 0;
  }
  const UInt32 n = dev->running;
  pthread_mutex_unlock(&gLock);
  os_log(gLog, "GuampAudio: StartIO %{public}@ (%u clients)", dev->uid, n);
  return kAudioHardwareNoError;
}
static OSStatus stopIO(AudioServerPlugInDriverRef d, AudioObjectID device, UInt32 client) {
  (void)d; (void)client;
  Device *dev = deviceOwning(device);
  if (!dev || dev->object != device) return kAudioHardwareBadObjectError;
  pthread_mutex_lock(&gLock);
  const OSStatus s = dev->running ? kAudioHardwareNoError : kAudioHardwareNotRunningError;
  if (dev->running) dev->running--;
  const UInt32 n = dev->running;
  pthread_mutex_unlock(&gLock);
  os_log(gLog, "GuampAudio: StopIO %{public}@ (%u clients)", dev->uid, n);
  return s;
}
// Zero time stamps exactly kRingFrames frames apart on the host clock (the NullAudio sample driver's model)
static OSStatus zeroTimeStamp(AudioServerPlugInDriverRef d, AudioObjectID device, UInt32 client, Float64 *sampleTime,
                              UInt64 *hostTime, UInt64 *seed) {
  (void)d; (void)client;
  Device *dev = deviceOwning(device);
  if (!dev || dev->object != device || !sampleTime || !hostTime || !seed) return kAudioHardwareBadObjectError;
  pthread_mutex_lock(&gLock);
  const Float64 ticksPerRing = gTicksPerFrame * kRingFrames;
  const UInt64 now = mach_absolute_time();
  if ((Float64)dev->anchor + ticksPerRing * (Float64)(dev->cycles + 1) <= (Float64)now) dev->cycles++;
  *sampleTime = (Float64)dev->cycles * kRingFrames;
  *hostTime = dev->anchor + (UInt64)(ticksPerRing * (Float64)dev->cycles);
  *seed = 1;
  pthread_mutex_unlock(&gLock);
  return kAudioHardwareNoError;
}
static OSStatus willDo(AudioServerPlugInDriverRef d, AudioObjectID device, UInt32 client, UInt32 op, Boolean *will,
                       Boolean *inPlace) {
  (void)d; (void)device; (void)client;
  if (!will || !inPlace) return kAudioHardwareIllegalOperationError;
  *will = op == kAudioServerPlugInIOOperationReadInput || op == kAudioServerPlugInIOOperationWriteMix;
  *inPlace = true;
  return kAudioHardwareNoError;
}
static OSStatus ioEdge(AudioServerPlugInDriverRef d, AudioObjectID device, UInt32 client, UInt32 op, UInt32 frames,
                       const AudioServerPlugInIOCycleInfo *cycle) {
  (void)d; (void)device; (void)client; (void)op; (void)frames; (void)cycle;
  return kAudioHardwareNoError;
}
static OSStatus doIO(AudioServerPlugInDriverRef d, AudioObjectID device, AudioObjectID stream, UInt32 client, UInt32 op,
                     UInt32 frames, const AudioServerPlugInIOCycleInfo *cycle, void *main, void *secondary) {
  (void)d; (void)client; (void)cycle; (void)secondary;
  Device *dev = deviceOwning(device);
  if (!dev || !isStream(dev, stream) || !main) return kAudioHardwareBadObjectError;
  if (op == kAudioServerPlugInIOOperationReadInput) {
    // step 6: microphone samples from the kernel (/dev/guamp-mic); null device: silence
    memset(main, 0, (size_t)frames * streamChannels(dev, stream) * sizeof(Float32));
  } else if (op == kAudioServerPlugInIOOperationWriteMix) {
    // step 4: Float32 -> s16 and a non-blocking write to /dev/guamp-audio (kernel ring -> q6asm/AFE -> aw882xx);
    // null device: dropped
  }
  return kAudioHardwareNoError;
}

static AudioServerPlugInDriverInterface gInterface = {
  NULL, queryInterface, addRef, release, initialize, createDevice, destroyDevice, addClient, removeClient, configChange,
  configChange, hasProperty, isSettable, getSize, getData, setData, startIO, stopIO, zeroTimeStamp, willDo, ioEdge, doIO,
  ioEdge,
};

// CFPlugInFactories entry of Info.plist
__attribute__((visibility("default"))) void *GuampAudio_Create(CFAllocatorRef allocator, CFUUIDRef type) {
  (void)allocator;
  if (!CFEqual(type, kAudioServerPlugInTypeUUID)) return NULL;
  if (!gLog) gLog = os_log_create("com.guamp.audio", "plugin");
  return &gInterfacePtr;
}
