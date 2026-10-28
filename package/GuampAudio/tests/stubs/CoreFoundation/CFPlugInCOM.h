#pragma once
typedef SInt32 HRESULT; typedef UInt32 ULONG; typedef void *LPVOID; typedef CFUUIDBytes REFIID;
#define S_OK ((HRESULT)0) 
#define E_NOINTERFACE ((HRESULT)0x80000004)
#define E_POINTER ((HRESULT)0x80000005)
#define IUnknownUUID CFUUIDGetConstantUUIDWithBytes(0,0,0,0,0,0,0,0,0,0xC0,0,0,0,0,0,0,0x46)
