# GuampAudio.driver — iOS side of guamp audio

Route (c) of `analisi/audio/IOS-AUDIO-LATO-UTENTE-20261028.md`: an AudioServerPlugIn in userland that presents the built-in
devices with the UIDs VirtualAudio expects, instead of an `IOAudio2Device` in the kernel (undocumented ABI) or nubs for
Apple's MCA/SIO/codec kexts (would drive Apple hardware). Proven on iOS 16 by hackios (D22); here the host is
**audiomxd** (iPadOS 17.7.11, 21H461, `CoreAudioServices/audiomxd` feature flag).

Status: complete source, PC-checked (host test + arm64-apple-ios17 syntax), CI workflow and kernel mount prepared. Not built
on a Mac yet, not booted.

## Pieces

| Piece | Where |
|---|---|
| Plug-in (CFPlugIn `443ABAB8-…`, factory `GuampAudio_Create`) | `GuampAudio.c`, `GuampAudioServerPlugIn.h`, `Info.plist` |
| Clock / volume law / s16 conversion / staging FIFO (no OS calls) | `GuampAudioCore.h`, host test `tests/guamp_audio_core_test.c` |
| Kernel interface `/dev/guamp-audio` (shared with the kernel side) | `guamp_audio_dev.h` |
| PC checks | `tests/check-guamp-audio.sh` (stand-in headers in `tests/stubs`, `tests/darwin-libc`) |
| Build + 4 KiB ad hoc signature + CDHash | `build-guamp-audio-plugin.sh`; CI: `guamp-kernel-build/.github/workflows/audio-plugin.yml` |
| Addon entry `audio/HAL/GuampAudio.driver/*` | `scripts/make-guamp-gpu-addon.py --audio DIR [--no-gpu]` |
| Addon + owner trust bundle on the PC | `scripts/compose-guamp-audio-addon.py` (recomputes the CDHash from the signature) |
| Mount over `/System/Library/Audio/Plug-Ins/HAL` | `patches/xnu/zzzzzzz-guamp-audio-plugin.patch`, boot-arg `guamp-audio-plugin=1` (default off) |
| ADT facts | `scripts/make-guamp-audio-adt.py` → `Codex/cpu-config-8-audio/kernel.macho.adt` |

## Devices

| UID | Streams | Controls | Why |
|---|---|---|---|
| `Speaker` | out 2 ch (`spkr`), in 2 ch silent | volume `vlme` −48…0 dB, mute `mute` | VirtualAudio's built-in speaker port |
| `Hawking` | in 1 ch silent | — | VirtualAudio's `vhaw` companion VAD (hackios: without it every route stays on the null port) |
| `Codec` | in 1 ch (`micr`): the built-in microphone from `/dev/guamp-mic` (`guamp_mic_dev.h`, s16 mono 48 kHz; silence without the node) | data source `imic` "Microphone" | built-in microphone port |

Format to the HAL: Float32 interleaved 48 kHz (other rates/formats refused). Clock: `mach_absolute_time()`, zero time stamp
every 16384 frames, one period per call (NullAudio model), seed bumped on each restart. Unknown properties are logged once
(`os_log` subsystem `com.guamp.audio`) to learn what VirtualAudio of J120 asks.

## Output path

- Speaker's first StartIO opens `/dev/guamp-audio` (`O_WRONLY|O_NONBLOCK`), checks `GET_INFO` (abi 1, 48 kHz, 2 ch, 16 bit),
  `SET_ROUTE speaker`, `SET_VOLUME`, `START`. Last StopIO: `STOP`, close, counters to the log.
- WriteMix: Float32 → s16 (rounded, saturated, NaN → 0), 512-frame chunks into a 4096-frame FIFO, flushed with non-blocking
  write(); a short write/EAGAIN leaves the rest for the next cycle; a full FIFO drops the oldest frames (bounded latency).
- Volume/mute: Q16 gain to the kernel by ioctl; if the ioctl fails (ENOTTY, sandbox EPERM) the plug-in scales the samples.
- No device, wrong ABI or open refused: null output (logged once per errno), everything else unchanged.

## Placement

`/System/Library/Audio/Plug-Ins/HAL` is on the sealed **root system volume** (verified on 21H461: `044-76040` holds it with
BuiltinAudioPlugin, BTAudioHALPlugin…; the SystemOS cryptex `044-76034` has no Audio directory). The HAL (dyld cache .17)
scans `/System/Library/Audio/Plug-Ins/HAL`, `/Library/Audio/Plug-Ins/HAL` and `/Library/Apple/Audio/Plug-Ins/HAL` (the
last does not exist on the volume, so no mount point). With `guamp-audio-plugin=1` the kernel installs the payload addon into
Data's `/private/var/ngpu` (also without `sheng-gpu-addon`/`guamp-iphone-res`) and mounts `/private/var/ngpu/audio/HAL`
read-only, `MNT_UNION`, over the HAL directory; stage 5 = BuiltinAudioPlugin and GuampAudio both visible. Enumeration falls
through to the Apple plug-ins (`native-union-getattrlistbulk.patch`, getdirentries union code).

## Signing, trust, sandbox

- Ad hoc, `--pagesize 4096`, identifier `com.guamp.audio.GuampAudio`, no entitlements (it runs inside audiomxd). Apple's own
  HAL plug-ins and audiomxd are 4 KiB-page signed and in the static trust cache; ours goes into the owner trust module.
- audiomxd: `platform-application`, `com.apple.private.audio.driver-host`, sandbox `com.apple.private.sandbox.profile:embedded`
  = `temporary-sandbox`, runs as `mobile`. File exceptions: read `/Library/Audio/Plug-Ins/HAL/`, `/Library/Audio/Tunings/`,
  read-write only `/dev/exfiltration-audio_capture`. Whether its profile lets it open `/dev/guamp-audio` is open: the
  plug-in then logs `unavailable (errno 1)` and stays silent. Fallback without touching the sandbox: an IOKit user client
  published with `NativeSandboxGrantProcess = audiomxd` (`zzw-native-adreno-sandbox-grant.patch` mechanism).
- Node permissions: root:mobile 0660 (`guamp_audio_dev.h`).
- No plug-in allow-list strings found in the HAL (`HALS_PlugInManager::LoadPlugIns: loading plug-in from: %s`); optional
  Info.plist keys seen: `AudioServerPlugIn_LoadingConditions`, `AudioServerPlugIn_MachServices`,
  `AudioServerPlugIn_IOKitUserClients` (not needed).
