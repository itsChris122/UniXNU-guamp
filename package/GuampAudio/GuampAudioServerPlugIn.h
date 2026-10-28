// SPDX-License-Identifier: BSD-2-Clause-Patent
// The part of the AudioServerPlugIn interface GuampAudio.driver needs. macOS documents it in CoreAudio/AudioServerPlugIn.h and
// CoreAudio/AudioHardwareBase.h; the iOS SDK ships CoreAudioTypes but not these headers, while the HAL inside audiomxd
// (iPadOS 17: HALS_PlugIn / HALS_PlugInDevice) loads plug-ins of the same CFPlugIn type from /System/Library/Audio/Plug-Ins/HAL.
// Only the declarations are restated here (four-char codes, the COM table layout); no Apple code.
#ifndef GUAMP_AUDIO_SERVER_PLUGIN_H
#define GUAMP_AUDIO_SERVER_PLUGIN_H
#include <CoreFoundation/CoreFoundation.h>
#include <CoreFoundation/CFPlugInCOM.h>
#include <CoreAudioTypes/CoreAudioTypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef UInt32 AudioObjectID;
typedef UInt32 AudioClassID;
typedef UInt32 AudioObjectPropertySelector;
typedef UInt32 AudioObjectPropertyScope;
typedef UInt32 AudioObjectPropertyElement;
typedef struct AudioObjectPropertyAddress {
  AudioObjectPropertySelector mSelector;
  AudioObjectPropertyScope mScope;
  AudioObjectPropertyElement mElement;
} AudioObjectPropertyAddress;
typedef struct AudioStreamRangedDescription {
  AudioStreamBasicDescription mFormat;
  AudioValueRange mSampleRateRange;
} AudioStreamRangedDescription;
typedef struct AudioServerPlugInClientInfo {
  UInt32 mClientID;
  pid_t mProcessID;
  Boolean mIsNativeEndian;
  CFStringRef mBundleID;
} AudioServerPlugInClientInfo;
// Only received by pointer from the HAL; this driver reads just the two leading fields, so the time stamps that follow
// (input/output/current AudioTimeStamp and reserved words) are deliberately not restated.
typedef struct AudioServerPlugInIOCycleInfo {
  UInt64 mIOCycleCounter;
  UInt32 mNominalIOBufferFrameSize;
} AudioServerPlugInIOCycleInfo;

typedef struct AudioServerPlugInHostInterface AudioServerPlugInHostInterface;
typedef const AudioServerPlugInHostInterface *AudioServerPlugInHostRef;
struct AudioServerPlugInHostInterface {
  OSStatus (*PropertiesChanged)(AudioServerPlugInHostRef, AudioObjectID, UInt32, const AudioObjectPropertyAddress *);
  OSStatus (*CopyFromStorage)(AudioServerPlugInHostRef, CFStringRef, CFPropertyListRef *);
  OSStatus (*WriteToStorage)(AudioServerPlugInHostRef, CFStringRef, CFPropertyListRef);
  OSStatus (*DeleteFromStorage)(AudioServerPlugInHostRef, CFStringRef);
  OSStatus (*RequestDeviceConfigurationChange)(AudioServerPlugInHostRef, AudioObjectID, UInt64, void *);
};

typedef struct AudioServerPlugInDriverInterface AudioServerPlugInDriverInterface;
typedef AudioServerPlugInDriverInterface **AudioServerPlugInDriverRef;
struct AudioServerPlugInDriverInterface {
  void *_reserved;
  HRESULT (*QueryInterface)(void *, REFIID, LPVOID *);
  ULONG (*AddRef)(void *);
  ULONG (*Release)(void *);
  OSStatus (*Initialize)(AudioServerPlugInDriverRef, AudioServerPlugInHostRef);
  OSStatus (*CreateDevice)(AudioServerPlugInDriverRef, CFDictionaryRef, const AudioServerPlugInClientInfo *, AudioObjectID *);
  OSStatus (*DestroyDevice)(AudioServerPlugInDriverRef, AudioObjectID);
  OSStatus (*AddDeviceClient)(AudioServerPlugInDriverRef, AudioObjectID, const AudioServerPlugInClientInfo *);
  OSStatus (*RemoveDeviceClient)(AudioServerPlugInDriverRef, AudioObjectID, const AudioServerPlugInClientInfo *);
  OSStatus (*PerformDeviceConfigurationChange)(AudioServerPlugInDriverRef, AudioObjectID, UInt64, void *);
  OSStatus (*AbortDeviceConfigurationChange)(AudioServerPlugInDriverRef, AudioObjectID, UInt64, void *);
  Boolean (*HasProperty)(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress *);
  OSStatus (*IsPropertySettable)(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress *, Boolean *);
  OSStatus (*GetPropertyDataSize)(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress *, UInt32,
                                  const void *, UInt32 *);
  OSStatus (*GetPropertyData)(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress *, UInt32,
                              const void *, UInt32, UInt32 *, void *);
  OSStatus (*SetPropertyData)(AudioServerPlugInDriverRef, AudioObjectID, pid_t, const AudioObjectPropertyAddress *, UInt32,
                              const void *, UInt32, const void *);
  OSStatus (*StartIO)(AudioServerPlugInDriverRef, AudioObjectID, UInt32);
  OSStatus (*StopIO)(AudioServerPlugInDriverRef, AudioObjectID, UInt32);
  OSStatus (*GetZeroTimeStamp)(AudioServerPlugInDriverRef, AudioObjectID, UInt32, Float64 *, UInt64 *, UInt64 *);
  OSStatus (*WillDoIOOperation)(AudioServerPlugInDriverRef, AudioObjectID, UInt32, UInt32, Boolean *, Boolean *);
  OSStatus (*BeginIOOperation)(AudioServerPlugInDriverRef, AudioObjectID, UInt32, UInt32, UInt32,
                               const AudioServerPlugInIOCycleInfo *);
  OSStatus (*DoIOOperation)(AudioServerPlugInDriverRef, AudioObjectID, AudioObjectID, UInt32, UInt32, UInt32,
                            const AudioServerPlugInIOCycleInfo *, void *, void *);
  OSStatus (*EndIOOperation)(AudioServerPlugInDriverRef, AudioObjectID, UInt32, UInt32, UInt32,
                             const AudioServerPlugInIOCycleInfo *);
};

// CFPlugIn type and interface (AudioServerPlugIn.h): 443ABAB8-E7B3-491A-B985-BEB9187030DB, EEA5773D-CC43-49F1-8E00-8F96E7D23B17
#define kAudioServerPlugInTypeUUID                                                                                       \
  CFUUIDGetConstantUUIDWithBytes(NULL, 0x44, 0x3A, 0xBA, 0xB8, 0xE7, 0xB3, 0x49, 0x1A, 0xB9, 0x85, 0xBE, 0xB9, 0x18, 0x70, \
                                 0x30, 0xDB)
#define kAudioServerPlugInDriverInterfaceUUID                                                                            \
  CFUUIDGetConstantUUIDWithBytes(NULL, 0xEE, 0xA5, 0x77, 0x3D, 0xCC, 0x43, 0x49, 0xF1, 0x8E, 0x00, 0x8F, 0x96, 0xE7, 0xD2, \
                                 0x3B, 0x17)

enum {
  kAudioObjectUnknown = 0, kAudioObjectPlugInObject = 1,
  kAudioHardwareNoError = 0, kAudioHardwareNotRunningError = 'stop', kAudioHardwareUnspecifiedError = 'what',
  kAudioHardwareUnknownPropertyError = 'who?', kAudioHardwareBadPropertySizeError = '!siz',
  kAudioHardwareIllegalOperationError = 'nope', kAudioHardwareBadObjectError = '!obj',
  kAudioHardwareUnsupportedOperationError = 'unop', kAudioDeviceUnsupportedFormatError = '!dat',
  kAudioObjectPropertyScopeGlobal = 'glob', kAudioObjectPropertyScopeInput = 'inpt', kAudioObjectPropertyScopeOutput = 'outp',
  kAudioObjectPropertyElementMain = 0,
  kAudioObjectClassID = 'aobj', kAudioPlugInClassID = 'aplg', kAudioDeviceClassID = 'adev', kAudioStreamClassID = 'astr',
  kAudioLevelControlClassID = 'levl', kAudioVolumeControlClassID = 'vlme', kAudioSelectorControlClassID = 'slct',
  kAudioDataSourceControlClassID = 'dsrc', kAudioBooleanControlClassID = 'togl', kAudioMuteControlClassID = 'mute',
  kAudioObjectPropertyBaseClass = 'bcls', kAudioObjectPropertyClass = 'clas', kAudioObjectPropertyOwner = 'stdv',
  kAudioObjectPropertyName = 'lnam', kAudioObjectPropertyManufacturer = 'lmak', kAudioObjectPropertyOwnedObjects = 'ownd',
  kAudioObjectPropertyControlList = 'ctrl',
  kAudioPlugInPropertyBoxList = 'box#', kAudioPlugInPropertyTranslateUIDToBox = 'uidb', kAudioPlugInPropertyDeviceList = 'dev#',
  kAudioPlugInPropertyTranslateUIDToDevice = 'uidd', kAudioPlugInPropertyResourceBundle = 'rsrc',
  kAudioDevicePropertyDeviceUID = 'uid ', kAudioDevicePropertyModelUID = 'muid', kAudioDevicePropertyTransportType = 'tran',
  kAudioDevicePropertyRelatedDevices = 'akin', kAudioDevicePropertyClockDomain = 'clkd',
  kAudioDevicePropertyDeviceIsAlive = 'livn', kAudioDevicePropertyDeviceIsRunning = 'goin',
  kAudioDevicePropertyDeviceCanBeDefaultDevice = 'dflt', kAudioDevicePropertyDeviceCanBeDefaultSystemDevice = 'sflt',
  kAudioDevicePropertyLatency = 'ltnc', kAudioDevicePropertyStreams = 'stm#', kAudioDevicePropertySafetyOffset = 'saft',
  kAudioDevicePropertyNominalSampleRate = 'nsrt', kAudioDevicePropertyAvailableNominalSampleRates = 'nsr#',
  kAudioDevicePropertyIsHidden = 'hidn', kAudioDevicePropertyPreferredChannelsForStereo = 'dch2',
  kAudioDevicePropertyZeroTimeStampPeriod = 'ring',
  kAudioDeviceTransportTypeBuiltIn = 'bltn',
  kAudioStreamPropertyIsActive = 'sact', kAudioStreamPropertyDirection = 'sdir', kAudioStreamPropertyTerminalType = 'term',
  kAudioStreamPropertyStartingChannel = 'schn', kAudioStreamPropertyLatency = 'ltnc',
  kAudioStreamPropertyVirtualFormat = 'sfmt', kAudioStreamPropertyAvailableVirtualFormats = 'sfma',
  kAudioStreamPropertyPhysicalFormat = 'pft ', kAudioStreamPropertyAvailablePhysicalFormats = 'pfta',
  kAudioStreamTerminalTypeSpeaker = 'spkr', kAudioStreamTerminalTypeMicrophone = 'micr',
  kAudioControlPropertyScope = 'cscp', kAudioControlPropertyElement = 'celm',
  kAudioLevelControlPropertyScalarValue = 'lcsv', kAudioLevelControlPropertyDecibelValue = 'lcdv',
  kAudioLevelControlPropertyDecibelRange = 'lcdr', kAudioLevelControlPropertyConvertScalarToDecibels = 'lcsd',
  kAudioLevelControlPropertyConvertDecibelsToScalar = 'lcds',
  kAudioSelectorControlPropertyCurrentItem = 'scci', kAudioSelectorControlPropertyAvailableItems = 'scai',
  kAudioSelectorControlPropertyItemName = 'scin', kAudioBooleanControlPropertyValue = 'bcvl',
  kAudioServerPlugInIOOperationReadInput = 'read', kAudioServerPlugInIOOperationWriteMix = 'rite',
};

#ifdef __cplusplus
}
#endif
#endif
