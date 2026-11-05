#!/usr/bin/env python3
"""Build CMCapture's model-specific camera description for guamp from the J120 one of the OS image.

  make-guamp-cmcapture-plist.py --j120 <OS>/System/Library/PrivateFrameworks/CMCapture.framework/J120 [--cameras 2|4] OUTDIR

Why: CMCapture names its camera description after sysctl hw.model (FigCaptureGetModelSpecificName, CMCapture 0x1a697900c:
sysctlbyname("hw.model"), suffix "AP" or "DEV" dropped, "N51" if empty) and reads
CMCapture.framework/<name>/AVCaptureSession.plist and CameraSetup.plist (FigCaptureCFCreatePropertyListForModelAndBundleIdentifier
0x1a68d0458: subdirectory <name> if it exists, else "iOS", whose AVCaptureSession.plist only has session presets). On guamp
hw.model is the first root compatible string, "motorola,guamp", so without this directory CMCapture finds no AVCaptureDevices
entry for our FigCaptureStreams: no camera at all in AVFoundation, whatever the plug-in publishes.

The output is OUTDIR/<model>/{AVCaptureSession.plist,CameraSetup.plist}; the addon carries it as cmcapture/<model>/* and the kernel
shows it inside /System/Library/PrivateFrameworks/CMCapture.framework with guamp-camera-ios=1|2 (union view, camera-plugin patch).
Nothing from Apple is committed: the files are derived at build time from the J120 ones of the OS image being booted.

What changes against J120 (only what the guamp plug-in, package/GuampISP, cannot honour):
  - stream formats: only the ones GuampFormats.c offers (same names and sensor geometry); device formats and presets that point
    at a dropped stream format go too, "Experimental" device formats go;
  - features of Apple's ISP: HDR, SIS, Night mode, Live Photo (Iris), video stabilisation, temporal noise reduction, wide colour
    (the plug-in delivers sRGB 420f/420v), phase-detect AF (AutoFocusSystem 1 = contrast, the 3A of package/GuampCamera3A);
  - the front camera has no "Retina flash" (hasFlash false).
--cameras 4 adds the two OV02B 2 MP modules as physical devices (the kernel publishes them as positions kGuampCamMacro and
  kGuampCamDepth, secondary-camera work of agent 7): macro OV02B10 (GT9772 AF) = "Back Ultra Wide Camera"
  (PortTypeBackSuperWide), depth OV02B1B (mono, fixed focus) = "Back Telephoto Camera" (PortTypeBackTelephoto), one
  stream format "Photo" 1600x1200. CMCapture binds a uniqueName to a port type (FigCaptureSourcePreprocessVideoDeviceDict
  0x1a6ac7488); no virtual device (Dual/Dual Wide/Triple) is described, so the Camera app keeps back/front and the two extra
  devices are for AVCaptureDevice.DiscoverySession clients until a synchronized Dual Wide stream group exists.
CameraSetup.plist: J120 tuning unchanged; the extra port types reuse the back camera's tuning (DefaultSensorIDs)."""
import argparse
import copy
import plistlib
import sys
from pathlib import Path

# Stream formats per device, as in package/GuampISP/GuampFormats.c (keep both lists in step).
KEEP_STREAM_FORMATS = {
    'Back Camera': ['Photo', 'Video-4by3-Low-Resolution', 'LowPower-Binned-2048x1536', 'Video-FES-30'],
    'Front Camera': ['Photo', 'Video-4by3-Low-Resolution', 'LowPower-Binned-4by3', 'Video-FES-30'],
}
EXTRA_DEVICES = (('Back Ultra Wide Camera', 'PortTypeBackSuperWide', 'macro'),
                 ('Back Telephoto Camera', 'PortTypeBackTelephoto', 'depth'))
OV02B = (1600, 1200)

DROP_FORMAT_KEYS = ('HDRSupported', 'SISSupported', 'NightModeSupported', 'IrisSupported', 'IrisVideoStabilizationSupported',
                    'VideoStabilizationSupported', 'CinematicVideoStabilizationSupported',
                    'VideoStabilizationTypeOverrideForCinematic', 'CaptureStillsFromVideoStream')
DROP_STREAM_KEYS = ('VideoStabilizationTypeOverrideForStandard', 'FESCompanionFormats')
DROP_DEVICE_KEYS = ('stillImageNoiseReductionAndFusionScheme', 'ispChromaNoiseReductionForPairingDisabled')


def prune_device(dev, keep):
    ms = dev['MultiStream']
    streams = ms['FigCaptureStreamFormats']
    missing = [name for name in keep if name not in streams]
    if missing:
        sys.exit(f"{dev['uniqueName']}: J120 has no stream format {', '.join(missing)} (GuampFormats.c out of step?)")
    ms['FigCaptureStreamFormats'] = {name: streams[name] for name in keep}
    for fmt in ms['FigCaptureStreamFormats'].values():
        for key in DROP_STREAM_KEYS:
            fmt.pop(key, None)
        if fmt.get('AutoFocusSystem'):
            fmt['AutoFocusSystem'] = 1          # contrast detection (no PDAF statistics without Apple's ISP)
    formats = {}
    for name, fmt in ms['AVCaptureDeviceFormats'].items():
        if name == 'Common':
            fmt = dict(fmt)
            fmt['TemporalNoiseReductionMode'] = 0
            fmt.pop('ISPChromaNoiseReduction', None)
            formats[name] = fmt
            continue
        if fmt.get('Experimental') or fmt.get('CaptureStreamFormat') not in keep:
            continue
        fmt = {k: v for k, v in fmt.items() if k not in DROP_FORMAT_KEYS}
        fmt.pop('TemporalNoiseReductionMode', None)
        formats[name] = fmt
    ms['AVCaptureDeviceFormats'] = formats
    ms['AVCaptureSessionPresets'] = prune_presets(ms['AVCaptureSessionPresets'], formats)
    for key in DROP_DEVICE_KEYS:
        dev.pop(key, None)
    return dev


def prune_presets(presets, formats):
    kept = {}
    for name, preset in presets.items():
        if isinstance(preset, dict) and 'DeviceFormat' in preset and preset['DeviceFormat'] not in formats:
            continue
        kept[name] = preset
    # aliases ("AVCaptureSessionPresetHigh": "AVCaptureSessionPreset1920x1080") only when the target survived
    return {n: p for n, p in kept.items() if not isinstance(p, str) or p in kept}


def ov02b_device(back, unique_name, macro):
    """A 2 MP back module: one stream format "Photo" 1600x1200, still and preview from the same stream."""
    dev = {k: copy.deepcopy(v) for k, v in back.items() if k != 'MultiStream'}
    dev['uniqueName'] = unique_name
    for key in ('smoothAutoFocusSupported', 'hasTorch', 'hasFlash'):
        dev[key] = False
    dev['hasAutoFocus'] = macro                     # OV02B10 macro: GT9772 voice coil; OV02B1B depth: fixed focus
    if not macro:
        for key in [k for k in dev if 'Focus' in k and k != 'hasAutoFocus']:
            dev.pop(key)
    for key in [k for k in dev if 'autoFlash' in k]:
        dev.pop(key)
    dev['minimumFocusDistance'] = 40 if macro else -1   # mm; the macro lens focuses at ~4 cm
    w, h = OV02B
    common = dict(back['MultiStream']['AVCaptureDeviceFormats'].get('Common', {}))
    formats = {
        'Common': common,
        '2MP': {'Width': w, 'Height': h, 'CaptureStreamFormat': 'Photo'},
        '1024x768': {'Width': 1024, 'Height': 768, 'CaptureStreamFormat': 'Photo'},
        '640x480': {'Width': 640, 'Height': 480, 'CaptureStreamFormat': 'Photo'},
        '480x360': {'Width': 480, 'Height': 360, 'CaptureStreamFormat': 'Photo'},
        '192x144': {'Width': 192, 'Height': 144, 'CaptureStreamFormat': 'Photo'},
    }
    presets = {
        'Common': copy.deepcopy(back['MultiStream']['AVCaptureSessionPresets'].get('Common', {})),
        'AVCaptureSessionPresetPhoto': {'DeviceFormat': '2MP', 'MaxFrameRate': 30, 'MinFrameRate': 15},
        'AVCaptureSessionPresetHigh': 'AVCaptureSessionPresetPhoto',
        'AVCaptureSessionPreset640x480': {'DeviceFormat': '640x480'},
        'AVCaptureSessionPresetMedium': {'DeviceFormat': '480x360'},
        'AVCaptureSessionPresetLow': {'DeviceFormat': '192x144', 'MaxFrameRate': 15, 'MinFrameRate': 7},
    }
    dev['MultiStream'] = {
        'FigCaptureStreamFormats': {'Photo': {'SensorWidth': w, 'SensorHeight': h, 'FES': False}},
        'AVCaptureDeviceFormats': formats,
        'AVCaptureSessionPresets': presets,
    }
    return dev


def build_session(j120, cameras):
    session = copy.deepcopy(j120)
    devices = []
    by_name = {d.get('uniqueName'): d for d in session['AVCaptureDevices']}
    for name in ('Back Camera', 'Front Camera'):
        if name not in by_name:
            sys.exit(f'J120 AVCaptureSession.plist has no "{name}"')
        devices.append(prune_device(by_name[name], KEEP_STREAM_FORMATS[name]))
    by_name['Front Camera']['hasFlash'] = False
    if cameras == 4:
        devices += [ov02b_device(by_name['Back Camera'], name, role == 'macro') for name, _, role in EXTRA_DEVICES]
    devices += [d for d in session['AVCaptureDevices'] if d.get('mediaType') != 'vide']   # Microphone
    session['AVCaptureDevices'] = devices
    common = session.get('AVCaptureDeviceCommonSettings')
    if isinstance(common, dict):
        common['wideColorSupported'] = False
    return session


def build_setup(j120, cameras):
    setup = copy.deepcopy(j120)
    tuning = setup.get('TuningParameters', {})
    if cameras == 4 and 'PortTypeBack' in tuning:
        defaults = tuning.setdefault('DefaultSensorIDs', {})
        for _, port, _ in EXTRA_DEVICES:
            tuning.setdefault(port, copy.deepcopy(tuning['PortTypeBack']))
            if 'PortTypeBack' in defaults:
                defaults.setdefault(port, defaults['PortTypeBack'])
    return setup


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--j120', type=Path, required=True, help='CMCapture.framework/J120 of the OS image')
    parser.add_argument('--model', default='motorola,guamp', help='hw.model of the payload (first root compatible)')
    parser.add_argument('--cameras', type=int, choices=(2, 4), default=2)
    parser.add_argument('outdir', type=Path)
    args = parser.parse_args()
    if args.model.endswith(('AP', 'DEV')) or '/' in args.model or not args.model:
        sys.exit('--model: the hw.model string as the kernel reports it (no AP/DEV suffix, no slash)')
    with open(args.j120 / 'AVCaptureSession.plist', 'rb') as f:
        session = build_session(plistlib.load(f), args.cameras)
    with open(args.j120 / 'CameraSetup.plist', 'rb') as f:
        setup = build_setup(plistlib.load(f), args.cameras)
    out = args.outdir / args.model
    out.mkdir(parents=True, exist_ok=True)
    for name, data in (('AVCaptureSession.plist', session), ('CameraSetup.plist', setup)):
        (out / name).write_bytes(plistlib.dumps(data, fmt=plistlib.FMT_BINARY))
    for dev in session['AVCaptureDevices']:
        if dev.get('mediaType') == 'vide':
            ms = dev['MultiStream']
            print(f"{dev['uniqueName']}: stream formats {', '.join(ms['FigCaptureStreamFormats'])}; "
                  f"{len(ms['AVCaptureDeviceFormats']) - 1} device formats, {len(ms['AVCaptureSessionPresets'])} presets")
    print(f'{out}: AVCaptureSession.plist, CameraSetup.plist')


if __name__ == '__main__':
    main()
