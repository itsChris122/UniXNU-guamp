# GuampAudio.driver — iOS side, step "null audio device"

Route (c) of `analisi/audio/IOS-AUDIO-LATO-UTENTE-20261028.md`: an AudioServerPlugIn in userland that presents the built-in
devices with the UIDs VirtualAudio expects, instead of an `IOAudio2Device` in the kernel (undocumented ABI) or nubs for
Apple's MCA/SIO/codec kexts (would drive Apple hardware). Proven on iOS 16 by hackios (D22); here the host is
**audiomxd** (iPadOS 17.7.11, 21H461, `CoreAudioServices/audiomxd` feature flag).

Status: source skeleton only. Nothing is built, packaged or booted yet.

## Pieces

| Piece | Where | State |
|---|---|---|
| Plug-in (CFPlugIn `443ABAB8-…`, factory `GuampAudio_Create`) | `GuampAudio.c`, `GuampAudioServerPlugIn.h`, `Info.plist` | written, host syntax check only |
| Build + CDHash | `build-guamp-audio-plugin.sh` (macOS runner, iPhoneOS SDK, ad hoc signature) | written, not run |
| ADT facts | `scripts/make-guamp-audio-adt.py` → `Codex/cpu-config-8-audio/kernel.macho.adt` | generated (base `cpu-config-8-gpu-pub`) |
| Placement | NGPUADD2 addon entry `audio/HAL/GuampAudio.driver/*` + kernel bindfs union over `/System/Library/Audio/Plug-Ins/HAL` | design below, kernel patch not written |
| Trust | CDHash in the owner trust cache (`make-owner-trust-bundle.py`), as the Adreno Metal plugin | design below |
| Kernel bridge `/dev/guamp-audio` | step 4 | not started |

## Devices (null I/O)

| UID | Streams | Controls | Why |
|---|---|---|---|
| `Speaker` | out 2 ch (`spkr`), in 2 ch silent | volume `vlme`, −48…0 dB (16 × 3 dB) | VirtualAudio's built-in speaker port; the silent input stands for the amplifier sense |
| `Hawking` | in 1 ch silent | — | VirtualAudio's `vhaw` companion VAD (hackios: without it every route stays on the null port); J120 has `audio-hawking-device` |
| `Codec` | in 1 ch silent (`micr`) | data source selector, one item `imic` "Microphone" | built-in microphone port; VirtualAudio asks DataSources on the codec input |

Format: Float32 interleaved, 48 kHz, one format per stream (sample-rate and format changes to anything else are refused).
Clock: `mach_absolute_time()`, zero time stamp every 16384 frames (NullAudio model), clock domain 0, latency/safety 0.
WriteMix is dropped, ReadInput is zeros. Every property the plug-in does not know is logged once per selector
(`os_log` subsystem `com.guamp.audio`): that list, plus audiomxd's own log, answers plan step 3 (which UIDs and
properties VirtualAudio of J120 really needs). `Actuator`/`Digital Mic` are left out until the log asks for them.

Expected effect when it works: the volume HUD and the Control Center slider move, `AVAudioSession.currentRoute` shows
Speaker, apps "play" silently; no change for Bluetooth (BTAudioHALPlugin untouched).

## Placement: addon + bindfs union (design, kernel patch to write)

`/System/Library/Audio/Plug-Ins/HAL` is on the sealed system volume. The mechanism is the one of the iPhone resources and
of the Metal plugin (`patches/xnu/zzy-native-gpu-addon-mount.patch`, `zzzz-guamp-iphone-res.patch`):

1. the payload addon (`kernel.macho.gpuaddon`, NGPUADD2) carries `audio/HAL/GuampAudio.driver/Info.plist` and
   `…/GuampAudio` (mode 0755); `native_gpu_addon_install` writes them below Data's `/private/var/ngpu`;
2. new boot arg `guamp-audio-plugin=1`: after the Data mount and before launchd, `native_gpu_addon_mount(…,
   "/private/var/ngpu/audio/HAL", "/System/Library/Audio/Plug-Ins/HAL", visible = ".../HAL/BTAudioHALPlugin.driver",
   own = ".../HAL/GuampAudio.driver", …)` — read-only `MNT_UNION` bindfs, original plug-ins still visible (stage 4),
   ours visible (stage 5); never panics, status logged;
3. to check first: the HAL directory is in the System volume and not in the SystemOS cryptex (iphone-res had the same
   question for its frameworks); if it is in the cryptex the union goes over the cryptex path.

Without `guamp-audio-plugin=1` nothing changes (default off), like `sheng-gpu-addon`.

## Signing and trust

- Ad hoc signature (`codesign -s -`), no entitlements: the HAL loads the bundle into audiomxd, so audiomxd's
  entitlements and sandbox apply; the plug-in itself needs only to pass AMFI library validation.
- AMFI accepts it because its CDHash is in the **owner trust cache** loaded by the payload
  (`kernel.macho.trustcache` built with `UniXNU/scripts/make-owner-trust-bundle.py`: the base trust cache modules byte for
  byte + one owner module with our CDHashes), exactly as `cpu-config-8-gpu-pub` does for NativeAdrenoMetal
  (`gpu-userspace.json` `own_cdhashes`). The RAM test passes it with `prepara-boot-smp-fix.py --trustcache`.
- Trust-cache code counts as platform code, which is what audiomxd's library validation requires (hackios: same on iOS 16
  in mediaserverd). Open point: whether audiomxd on 17.x adds a plug-in allow-list or a bundle-ID check; the first boot
  will tell (audiomxd log: "HALS_PlugInManager" lines, our `GuampAudio: initialized`).
- Sandbox: the null device opens nothing. Step 4 opens `/dev/guamp-audio`; audiomxd's profile may deny it (hackios saw no
  block, unverified on 17): alternatives are an IOKit user client of NativeEmmc or a shared memory ring mapped by the
  kernel.

## Next steps

1. Build on the runner (`build-guamp-audio-plugin.sh`), add the addon entry + owner trust bundle, write the
   `guamp-audio-plugin=1` mount patch; RAM test with ADT `cpu-config-8-audio`.
2. Read audiomxd/VirtualAudio logs (unified log reader of `analisi/bt/log-unificato/`), add or rename UIDs until the
   Speaker route appears.
3. Kernel bridge `/dev/guamp-audio` (s16 48 kHz ring, volume ioctl), then the ADSP sink (q6asm/AFE → PRI MI2S 0x1000 →
   aw882xx) once `GuampAdsp: ADSP APR OK` is seen.
