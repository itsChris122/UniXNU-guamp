// SPDX-License-Identifier: BSD-2-Clause-Patent
// GuampAudio.driver: the iOS half of guamp audio (analisi/audio/IOS-AUDIO-LATO-UTENTE-20261028.md, route (c)). An
// AudioServerPlugIn loaded by the HAL inside audiomxd (iPadOS 17.7.11, 21H461) from /System/Library/Audio/Plug-Ins/HAL. It
// presents the built-in devices VirtualAudio knows by UID, so that it builds the Built-In Speaker / Microphone ports instead
// of leaving everything on Null_Device:
//   "Speaker"  stereo output (+ a silent stereo input, the amplifier sense of an iPhone/iPad), volume 'vlme' and mute 'mute'
//   "Hawking"  one silent input (VirtualAudio's 'vhaw' companion device; J120 has an audio-hawking-device node)
//   "Codec"    one input with a data source selector (VirtualAudio asks for DataSources on the codec input): the
//              built-in microphone, read from the kernel's /dev/guamp-mic (guamp_mic_dev.h, 48 kHz s16 mono) when it exists
// Output: when the kernel's /dev/guamp-audio exists (guamp_audio_dev.h), the Speaker mix is converted to s16 48 kHz stereo and
// written to it (non-blocking, through a small staging FIFO); volume/mute go to the kernel by ioctl, or are applied here when
// the ioctl is refused. Without the device the output is dropped (null device). Input: the Codec input carries the
// microphone (/dev/guamp-mic, opened by the Codec's first StartIO, closed by its last StopIO; silence without the node);
// the Speaker and Hawking inputs stay silent. The Microphone privacy prompt is TCC's (tccd, kTCCServiceMicrophone) for the
// recording app; nothing here bypasses it.
// Clock: kRate frames per second of mach_absolute_time(), zero time stamp every kRingFrames frames (GuampAudioCore.h).
// No Apple code: written from the public macOS AudioServerPlugIn documentation (declarations in GuampAudioServerPlugIn.h).
// Diagnostics: os_log subsystem "com.guamp.audio" (Initialize, devices, StartIO/StopIO, backend open/close and counters, every
// property it does not know, once per selector: the list that tells which properties VirtualAudio of J120 really needs).
#include "GuampAudioServerPlugIn.h"
#include "GuampAudioCore.h"
#include "guamp_audio_dev.h"
#include "GuampMicCore.h"
#include "guamp_mic_dev.h"
#include <errno.h>
#include <fcntl.h>
#include <mach/mach_time.h>
#include <os/log.h>
#include <pthread.h>
#include <stdatomic.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

enum {
  kPlugIn = kAudioObjectPlugInObject,
  kSpeaker = 2, kSpeakerOut, kSpeakerIn, kSpeakerVolume, kSpeakerMute,
  kHawking, kHawkingIn,
  kCodec, kCodecIn, kCodecSource,
  kObjectCount
};
static const Float64 kRate = 48000.0;
static const UInt32 kRingFrames = 16384, kSafetyFrames = 0, kLatencyFrames = 0;
static const Float32 kMinDb = -48.0f, kMaxDb = 0.0f;   // 16 iOS volume steps of 3 dB
static const UInt32 kSourceInternalMic = 'imic';       // to confirm against VirtualAudio's queries (step 3)
_Static_assert(GUAMP_AUDIO_RATE == 48000 && GUAMP_AUDIO_CHANNELS == GUAMP_FIFO_CHANNELS, "device format");

typedef struct {
  AudioObjectID object, out, in, volume, mute, source;
  UInt32 outChannels, inChannels;
  CFStringRef uid, name;
  UInt32 running;   // StartIO count
  GuampClock clock;
} Device;

static Device gDevices[] = {
  {kSpeaker, kSpeakerOut, kSpeakerIn, kSpeakerVolume, kSpeakerMute, 0, 2, 2, CFSTR("Speaker"), CFSTR("Speaker"), 0, {0, 0, 0, 0, 0}},
  {kHawking, 0, kHawkingIn, 0, 0, 0, 0, 1, CFSTR("Hawking"), CFSTR("Hawking"), 0, {0, 0, 0, 0, 0}},
  {kCodec, 0, kCodecIn, 0, 0, kCodecSource, 0, 1, CFSTR("Codec"), CFSTR("Codec"), 0, {0, 0, 0, 0, 0}},
};
static const UInt32 kDeviceCount = sizeof(gDevices) / sizeof(gDevices[0]);

static pthread_mutex_t gLock = PTHREAD_MUTEX_INITIALIZER;   // object model, clocks, controls
static AudioServerPlugInHostRef gHost;
static Float32 gVolumeDb = kMaxDb;
static UInt32 gMute;
static UInt32 gSourceItem = kSourceInternalMic;
static UInt32 gRefs = 1;
static os_log_t gLog;
static UInt32 gUnknownSeen[64];   // selectors already reported as unknown (diagnostics only)

// ---- kernel backend (/dev/guamp-audio) ----------------------------------------------------------------------------------------
// Opened by the Speaker's first StartIO, closed by its last StopIO. gIOLock serialises the IO thread (WriteMix) against
// open/close and the volume ioctl; it is never taken while gLock is held.
static pthread_mutex_t gIOLock = PTHREAD_MUTEX_INITIALIZER;
static struct {
  int fd;                      // -1: null device
  int lastOpenErrno;           // logged once per distinct value
  Boolean hwVolume;            // the kernel accepted SET_VOLUME: samples leave here at unity
  _Atomic UInt32 softGainQ16;  // gain applied while converting (unity when hwVolume)
  GuampFifo fifo;
  UInt64 eagain, errors;
  Boolean wanted;              // between the Speaker's first StartIO and last StopIO
  UInt64 retryAt;              // mach time of the next open attempt while wanted and not open
  UInt64 retries;
} gBackend = {.fd = -1, .softGainQ16 = GUAMP_AUDIO_GAIN_UNITY};
static double gTicksPerSec = 24000000.0;   // mach_absolute_time() ticks per second (set by initialize)

static long sinkWrite(void *ctx, const void *bytes, size_t length) {
  const ssize_t n = write(*(const int *)ctx, bytes, length);
  if (n >= 0) return (long)(n - n % GUAMP_AUDIO_BYTES_PER_FRAME);
  if (errno == EAGAIN) gBackend.eagain++;
  else gBackend.errors++;
  return -1;
}

// Called with gIOLock held. Sends the current volume to the kernel, or keeps it for the conversion.
static void backendVolume(Float32 db, UInt32 mute) {
  const UInt32 gain = guamp_gain_q16(db, kMinDb, kMaxDb, (int)mute);
  if (gBackend.fd >= 0) {
    struct guamp_audio_volume v = {gain, (int32_t)lrintf(db * 100.0f), mute ? 1u : 0u, 0};
    gBackend.hwVolume = ioctl(gBackend.fd, GUAMP_AUDIO_SET_VOLUME, &v) == 0;
  }
  atomic_store(&gBackend.softGainQ16, gBackend.hwVolume ? GUAMP_AUDIO_GAIN_UNITY : gain);
}

// Called with gIOLock held. A refused open (EBUSY: the node is still held, or any other error) used to leave the whole
// IO session on the null device, i.e. silent until the next StartIO (sounds that "randomly" stop): backendWrite now
// retries once a second while the Speaker runs.
static void backendOpenLocked(Float32 db, UInt32 mute) {
  gBackend.wanted = true;
  if (gBackend.fd < 0) {
    const int fd = open(GUAMP_AUDIO_DEV_PATH, O_WRONLY | O_NONBLOCK | O_CLOEXEC);
    struct guamp_audio_info info;
    memset(&info, 0, sizeof(info));
    int why = fd < 0 ? errno : 0;
    if (fd >= 0 && ioctl(fd, GUAMP_AUDIO_GET_INFO, &info)) why = errno;
    else if (fd >= 0 && (info.abi != GUAMP_AUDIO_ABI_VERSION || info.rate != GUAMP_AUDIO_RATE ||
                         info.channels != GUAMP_AUDIO_CHANNELS || info.bits != GUAMP_AUDIO_BITS))
      why = EPROTO;
    if (why) {
      if (fd >= 0) close(fd);
      if (why != gBackend.lastOpenErrno)
        os_log(gLog, "GuampAudio: %{public}s unavailable (errno %d, abi %u): null output", GUAMP_AUDIO_DEV_PATH, why, info.abi);
      gBackend.lastOpenErrno = why;
    } else {
      gBackend.fd = fd;
      gBackend.lastOpenErrno = 0;
      gBackend.hwVolume = false;
      gBackend.eagain = gBackend.errors = 0;
      gBackend.fifo.dropped = gBackend.fifo.written = 0;
      guamp_fifo_reset(&gBackend.fifo);
      UInt32 route = GUAMP_AUDIO_DEV_ROUTE_SPEAKER;
      const int routeErr = ioctl(fd, GUAMP_AUDIO_SET_ROUTE, &route) ? errno : 0;
      backendVolume(db, mute);
      const int startErr = ioctl(fd, GUAMP_AUDIO_START) ? errno : 0;
      os_log(gLog, "GuampAudio: %{public}s open: ring %u, period %u, flags 0x%x, route err %d, hw volume %d, start err %d, "
             "retries %llu", GUAMP_AUDIO_DEV_PATH, info.ring_frames, info.period_frames, info.flags, routeErr, gBackend.hwVolume,
             startErr, (unsigned long long)gBackend.retries);
    }
  }
  if (gBackend.fd < 0) gBackend.retryAt = mach_absolute_time() + (UInt64)gTicksPerSec;
}

static void backendOpen(Float32 db, UInt32 mute) {
  pthread_mutex_lock(&gIOLock);
  gBackend.retries = 0;
  backendOpenLocked(db, mute);
  pthread_mutex_unlock(&gIOLock);
}

static void backendClose(void) {
  pthread_mutex_lock(&gIOLock);
  gBackend.wanted = false;
  if (gBackend.fd >= 0) {
    struct guamp_audio_info info;
    memset(&info, 0, sizeof(info));
    (void)ioctl(gBackend.fd, GUAMP_AUDIO_GET_INFO, &info);
    (void)ioctl(gBackend.fd, GUAMP_AUDIO_STOP);
    close(gBackend.fd);
    gBackend.fd = -1;
    os_log(gLog, "GuampAudio: %{public}s closed: written %llu, played %llu, underruns %llu; plug-in sent %llu, dropped %llu, "
           "full %llu, errors %llu", GUAMP_AUDIO_DEV_PATH, (unsigned long long)info.frames_written,
           (unsigned long long)info.frames_played, (unsigned long long)info.underruns, (unsigned long long)gBackend.fifo.written,
           (unsigned long long)gBackend.fifo.dropped, (unsigned long long)gBackend.eagain, (unsigned long long)gBackend.errors);
  }
  pthread_mutex_unlock(&gIOLock);
}

// IO thread: one WriteMix buffer (Float32 interleaved stereo) into the device.
static void backendWrite(const Float32 *mix, UInt32 frames) {
  pthread_mutex_lock(&gIOLock);
  if (gBackend.fd < 0 && gBackend.wanted && mach_absolute_time() >= gBackend.retryAt) {
    pthread_mutex_lock(&gLock);   // order gIOLock -> gLock is allowed (gIOLock is never taken under gLock)
    const Float32 db = gVolumeDb;
    const UInt32 mute = gMute;
    pthread_mutex_unlock(&gLock);
    gBackend.retries++;
    backendOpenLocked(db, mute);
  }
  if (gBackend.fd >= 0) {
    const UInt32 gain = atomic_load(&gBackend.softGainQ16);
    int16_t chunk[512 * GUAMP_FIFO_CHANNELS];
    for (UInt32 done = 0; done < frames;) {
      const UInt32 n = frames - done < 512 ? frames - done : 512;
      guamp_f32_to_s16(mix + (size_t)done * GUAMP_FIFO_CHANNELS, chunk, (size_t)n * GUAMP_FIFO_CHANNELS, gain);
      (void)guamp_fifo_push(&gBackend.fifo, chunk, n);
      done += n;
      (void)guamp_fifo_flush(&gBackend.fifo, sinkWrite, &gBackend.fd);
    }
  }
  pthread_mutex_unlock(&gIOLock);
}

// ---- kernel capture backend (/dev/guamp-mic) ----------------------------------------------------------------------------------
// Opened by the Codec's first StartIO, closed by its last StopIO. gMicLock serialises the IO thread (ReadInput) against
// open/close; never taken while gLock is held.
static pthread_mutex_t gMicLock = PTHREAD_MUTEX_INITIALIZER;
static struct {
  int fd;              // -1: silence
  int lastOpenErrno;
  UInt64 frames, padded, skipped, cycles;
} gMic = {.fd = -1};

static void micOpen(void) {
  pthread_mutex_lock(&gMicLock);
  if (gMic.fd < 0) {
    const int fd = open(GUAMP_MIC_DEV_PATH, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    struct guamp_mic_info info;
    memset(&info, 0, sizeof(info));
    int why = fd < 0 ? errno : 0;
    if (fd >= 0 && ioctl(fd, GUAMP_MIC_IOC_INFO, &info)) why = errno;
    else if (fd >= 0 && (info.abi != GUAMP_MIC_ABI || info.rate != GUAMP_MIC_RATE || info.channels != GUAMP_MIC_CHANNELS ||
                         info.bits != GUAMP_MIC_BITS))
      why = EPROTO;
    if (!why && ioctl(fd, GUAMP_MIC_IOC_START)) why = errno;
    if (why) {
      if (fd >= 0) close(fd);
      if (why != gMic.lastOpenErrno)
        os_log(gLog, "GuampAudio: %{public}s unavailable (errno %d, abi %u): silent microphone", GUAMP_MIC_DEV_PATH, why, info.abi);
      gMic.lastOpenErrno = why;
    } else {
      gMic.fd = fd;
      gMic.lastOpenErrno = 0;
      gMic.frames = gMic.padded = gMic.skipped = gMic.cycles = 0;
      os_log(gLog, "GuampAudio: %{public}s open: source %{public}s, amic%u, ring %u bytes, period %u", GUAMP_MIC_DEV_PATH,
             info.source == GUAMP_MIC_SOURCE_ADSP ? "adsp" : "null", info.mic, info.ring_bytes, info.period_frames);
    }
  }
  pthread_mutex_unlock(&gMicLock);
}

static void micClose(void) {
  pthread_mutex_lock(&gMicLock);
  if (gMic.fd >= 0) {
    struct guamp_mic_info info;
    memset(&info, 0, sizeof(info));
    (void)ioctl(gMic.fd, GUAMP_MIC_IOC_INFO, &info);
    (void)ioctl(gMic.fd, GUAMP_MIC_IOC_STOP);
    close(gMic.fd);
    gMic.fd = -1;
    os_log(gLog, "GuampAudio: %{public}s closed: captured %llu, overruns %u, last peak %u; plug-in got %llu frames, padded "
           "%llu, skipped %llu in %llu cycles", GUAMP_MIC_DEV_PATH, (unsigned long long)info.frames_captured, info.overruns,
           info.peak, (unsigned long long)gMic.frames, (unsigned long long)gMic.padded, (unsigned long long)gMic.skipped,
           (unsigned long long)gMic.cycles);
  }
  pthread_mutex_unlock(&gMicLock);
}

// IO thread: one ReadInput buffer (Float32, `channels` interleaved) from the device; silence for what is missing.
static void micRead(Float32 *out, UInt32 frames, UInt32 channels) {
  int16_t chunk[512];
  UInt32 done = 0;
  pthread_mutex_lock(&gMicLock);
  if (gMic.fd >= 0) {
    gMic.cycles++;
    struct guamp_mic_position pos;
    if (ioctl(gMic.fd, GUAMP_MIC_IOC_POSITION, &pos) == 0) {
      for (UInt32 drop = guamp_mic_excess(pos.queued_frames, frames); drop;) {
        const UInt32 n = drop < 512 ? drop : 512;
        const ssize_t got = read(gMic.fd, chunk, (size_t)n * GUAMP_MIC_FRAME_BYTES);
        if (got <= 0) break;
        gMic.skipped += (UInt64)got / GUAMP_MIC_FRAME_BYTES;
        drop -= (UInt32)got / GUAMP_MIC_FRAME_BYTES;
      }
    }
    while (done < frames) {
      const UInt32 n = frames - done < 512 ? frames - done : 512;
      const ssize_t got = read(gMic.fd, chunk, (size_t)n * GUAMP_MIC_FRAME_BYTES);
      if (got <= 0) break;
      const UInt32 f = (UInt32)got / GUAMP_MIC_FRAME_BYTES;
      guamp_s16_to_f32_dup(chunk, out + (size_t)done * channels, f, channels);
      done += f;
      if (f < n) break;
    }
    gMic.frames += done;
    gMic.padded += frames - done;
  }
  pthread_mutex_unlock(&gMicLock);
  if (done < frames) memset(out + (size_t)done * channels, 0, (size_t)(frames - done) * channels * sizeof(Float32));
}

// ---- object model -------------------------------------------------------------------------------------------------------------

static Device *deviceOwning(AudioObjectID object) {
  for (UInt32 i = 0; i < kDeviceCount; i++) {
    Device *d = &gDevices[i];
    if (object == d->object ||
        (object && (object == d->out || object == d->in || object == d->volume || object == d->mute || object == d->source)))
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

// ---- replies: out == NULL means "size only" -----------------------------------------------------------------------------------

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
// the object does not have the property. Called with gLock held.
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
    AudioObjectID ids[5];
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
        if (d->mute) ids[n++] = d->mute;
        if (d->source) ids[n++] = d->source;
        putObjects(r, ids, n);
        break;
      case kAudioObjectPropertyOwnedObjects:
        n = deviceStreams(d, a->mScope, ids);
        if (d->volume && a->mScope != kAudioObjectPropertyScopeInput) ids[n++] = d->volume;
        if (d->mute && a->mScope != kAudioObjectPropertyScopeInput) ids[n++] = d->mute;
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
      case kAudioLevelControlPropertyScalarValue: putF32(r, guamp_db_to_scalar(gVolumeDb, kMinDb, kMaxDb)); break;
      case kAudioLevelControlPropertyDecibelValue: putF32(r, gVolumeDb); break;
      case kAudioLevelControlPropertyDecibelRange: {
        AudioValueRange range = {kMinDb, kMaxDb};
        put(r, &range, sizeof(range));
        break;
      }
      case kAudioLevelControlPropertyConvertScalarToDecibels:
      case kAudioLevelControlPropertyConvertDecibelsToScalar: {
        // In-out property: the caller's value arrives in the output buffer (as on macOS)
        const Float32 v = r->out && r->cap >= sizeof(Float32) ? *(Float32 *)r->out : 0;
        putF32(r, sel == kAudioLevelControlPropertyConvertScalarToDecibels ? guamp_scalar_to_db(v, kMinDb, kMaxDb)
                                                                           : guamp_db_to_scalar(v, kMinDb, kMaxDb));
        break;
      }
      default: return kAudioHardwareUnknownPropertyError;
    }
    return r->status;
  }
  if (object == d->mute) {
    switch (sel) {
      case kAudioObjectPropertyBaseClass: putU32(r, kAudioBooleanControlClassID); break;
      case kAudioObjectPropertyClass: putU32(r, kAudioMuteControlClassID); break;
      case kAudioObjectPropertyOwner: putU32(r, d->object); break;
      case kAudioControlPropertyScope: putU32(r, kAudioObjectPropertyScopeOutput); break;
      case kAudioControlPropertyElement: putU32(r, kAudioObjectPropertyElementMain); break;
      case kAudioBooleanControlPropertyValue: putU32(r, gMute); break;
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

// ---- COM entry points ---------------------------------------------------------------------------------------------------------

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
  const double ticksPerFrame = guamp_ticks_per_frame(tb.numer, tb.denom, kRate);
  gTicksPerSec = ticksPerFrame * kRate;
  for (UInt32 i = 0; i < kDeviceCount; i++) guamp_clock_init(&gDevices[i].clock, ticksPerFrame, kRingFrames);
  const Boolean present = access(GUAMP_AUDIO_DEV_PATH, F_OK) == 0, mic = access(GUAMP_MIC_DEV_PATH, F_OK) == 0;
  os_log(gLog, "GuampAudio: initialized, %u devices (Speaker, Hawking, Codec), %.0f Hz, ring %u frames, %{public}s %{public}s, "
         "%{public}s %{public}s", kDeviceCount, kRate, kRingFrames, GUAMP_AUDIO_DEV_PATH,
         present ? "present" : "absent (null output)", GUAMP_MIC_DEV_PATH, mic ? "present" : "absent (silent microphone)");
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
       : object == kSpeakerMute   ? s == kAudioBooleanControlPropertyValue
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
  if (s == kAudioHardwareUnknownPropertyError) unknown(object, a);
  pthread_mutex_unlock(&gLock);
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
  if (s == kAudioHardwareUnknownPropertyError) unknown(object, a);
  pthread_mutex_unlock(&gLock);
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
    v = sel == kAudioLevelControlPropertyScalarValue ? guamp_scalar_to_db(v, kMinDb, kMaxDb) : guamp_clampf(v, kMinDb, kMaxDb);
    pthread_mutex_lock(&gLock);
    const Boolean differs = v != gVolumeDb;
    gVolumeDb = v;
    const UInt32 mute = gMute;
    pthread_mutex_unlock(&gLock);
    if (differs) {
      pthread_mutex_lock(&gIOLock);
      backendVolume(v, mute);
      pthread_mutex_unlock(&gIOLock);
      changed(object, kAudioLevelControlPropertyScalarValue, kAudioObjectPropertyScopeGlobal);
      changed(object, kAudioLevelControlPropertyDecibelValue, kAudioObjectPropertyScopeGlobal);
    }
    return kAudioHardwareNoError;
  }
  if (object == kSpeakerMute && sel == kAudioBooleanControlPropertyValue) {
    if (inSize != sizeof(UInt32)) return kAudioHardwareBadPropertySizeError;
    const UInt32 mute = *(const UInt32 *)in ? 1 : 0;
    pthread_mutex_lock(&gLock);
    const Boolean differs = mute != gMute;
    gMute = mute;
    const Float32 db = gVolumeDb;
    pthread_mutex_unlock(&gLock);
    if (differs) {
      pthread_mutex_lock(&gIOLock);
      backendVolume(db, mute);
      pthread_mutex_unlock(&gIOLock);
      changed(object, kAudioBooleanControlPropertyValue, kAudioObjectPropertyScopeGlobal);
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
  const Boolean first = dev->running == 0;
  dev->running++;
  if (first) guamp_clock_start(&dev->clock, mach_absolute_time());
  const UInt32 n = dev->running;
  const Float32 db = gVolumeDb;
  const UInt32 mute = gMute;
  pthread_mutex_unlock(&gLock);
  if (first && dev->out) backendOpen(db, mute);
  if (first && dev->object == kCodec) micOpen();
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
  if (!s && !n && dev->out) backendClose();
  if (!s && !n && dev->object == kCodec) micClose();
  os_log(gLog, "GuampAudio: StopIO %{public}@ (%u clients)", dev->uid, n);
  return s;
}
static OSStatus zeroTimeStamp(AudioServerPlugInDriverRef d, AudioObjectID device, UInt32 client, Float64 *sampleTime,
                              UInt64 *hostTime, UInt64 *seed) {
  (void)d; (void)client;
  Device *dev = deviceOwning(device);
  if (!dev || dev->object != device || !sampleTime || !hostTime || !seed) return kAudioHardwareBadObjectError;
  double t = 0;
  uint64_t h = 0, s = 0;
  pthread_mutex_lock(&gLock);
  guamp_clock_zero(&dev->clock, mach_absolute_time(), &t, &h, &s);
  pthread_mutex_unlock(&gLock);
  *sampleTime = t;
  *hostTime = h;
  *seed = s;
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
  if (op == kAudioServerPlugInIOOperationReadInput && stream == kCodecIn) {
    micRead(main, frames, streamChannels(dev, stream));   // the built-in microphone (silence without /dev/guamp-mic)
  } else if (op == kAudioServerPlugInIOOperationReadInput) {
    memset(main, 0, (size_t)frames * streamChannels(dev, stream) * sizeof(Float32));
  } else if (op == kAudioServerPlugInIOOperationWriteMix && stream == kSpeakerOut) {
    backendWrite(main, frames);   // no-op while /dev/guamp-audio is not open (null device)
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
