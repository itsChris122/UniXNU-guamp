#pragma once
typedef struct { Float64 mSampleRate; UInt32 mFormatID, mFormatFlags, mBytesPerPacket, mFramesPerPacket, mBytesPerFrame,
  mChannelsPerFrame, mBitsPerChannel, mReserved; } AudioStreamBasicDescription;
typedef struct { Float64 mMinimum, mMaximum; } AudioValueRange;
enum { kAudioFormatLinearPCM = 'lpcm', kAudioFormatFlagIsFloat = 1, kAudioFormatFlagsNativeEndian = 0, kAudioFormatFlagIsPacked = 8 };
