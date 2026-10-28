#!/usr/bin/env python3
"""Compile the generic platform expert against the actual source-kernel ABI."""
import argparse
import json
from pathlib import Path
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('component', choices=['ArmVirtPlatform', 'ArmDTPlatform', 'ArmCpu', 'NativeAES', 'NativeVirtioBlock', 'NativeRamBlock', 'NativeMobileDisplay', 'NativeVirtioConsole', 'NativeLegacyDisplay', 'NativeGOPPresenter', 'NativeOfflineGPU', 'NativeOfflineAccelerator', 'NativeIOGPUProviderProbe', 'NativeUfs', 'NativeHidInput', 'NativeKeyStoreEvents', 'NativeAdreno', 'NativeEmmc'])
parser.add_argument('--negative-aes-kat', action='store_true', help='Isolated own-source wrong-answer refusal control; never changes original inputs')
parser.add_argument('--verified-root', action='store_true', help='Authenticate all actual disk reads against an independently signed immutable policy')
parser.add_argument('--verified-root-bundle', action='store_true', help='Select exactly one capacity-matching policy per device, then independently authenticate it')
parser.add_argument('--negative-verified-block', action='store_true', help='Negative own-buffer control: corrupt only the completed DMA snapshot')
parser.add_argument('--wifi-runtime', action='store_true', help='guamp NativeEmmc: GUAMP_WIFI_RUNTIME=1 plus the SM6115/WCN3990 provider (drivers/GuampWifi) in the same archive; still opt-in at boot (guamp-wifi-start=1)')
args = parser.parse_args()
if args.wifi_runtime and args.component != 'NativeEmmc':
    parser.error('The Wi-Fi runtime is part of NativeEmmc')
if args.verified_root_bundle and not args.verified_root:
    parser.error('A policy bundle requires verified block mode')
if args.verified_root_bundle and args.negative_verified_block:
    parser.error('Select either bundle policy qualification or the isolated DMA snapshot fault')
if (args.verified_root or args.negative_verified_block) and args.component != 'NativeVirtioBlock':
    parser.error('Verified block mode requires the concrete read-only block transport')
if args.negative_verified_block and not args.verified_root:
    parser.error('The isolated input fault requires verified block mode')
component = args.component
recipe = ROOT / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/iokit/DEVELOPMENT/IOPlatformExpert.cpo.json'
record = json.loads(recipe.read_text().strip().rstrip(','))
source = ROOT / 'drivers' / ('NativeMobileDisplay' if component in ('NativeLegacyDisplay','NativeGOPPresenter') else 'NativeMetal' if component in ('NativeOfflineGPU','NativeOfflineAccelerator','NativeIOGPUProviderProbe') else component) / {'NativeAdreno':'NativeAdrenoService.cpp'}.get(component, component + '.cpp')
out = ROOT / 'build' / {'ArmVirtPlatform': 'armvirt-platform', 'ArmDTPlatform': 'armdt-platform', 'ArmCpu': 'armcpu', 'NativeAES': 'native-aes',
                        'NativeVirtioBlock': 'native-virtio-block', 'NativeRamBlock': 'native-ram-block',
                        'NativeMobileDisplay': 'native-mobile-display', 'NativeVirtioConsole': 'native-virtio-console',
                        'NativeLegacyDisplay': 'native-legacy-display', 'NativeGOPPresenter': 'native-gop-presenter', 'NativeOfflineGPU':'native-offline-gpu','NativeOfflineAccelerator':'native-offline-accelerator','NativeIOGPUProviderProbe':'native-original-provider', 'NativeUfs':'native-ufs', 'NativeHidInput':'native-hid-input', 'NativeKeyStoreEvents':'native-keystore-events', 'NativeAdreno':'native-adreno', 'NativeEmmc':'native-emmc'}[component]
if args.verified_root:
    out = ROOT / ('build/native-virtio-block-verified-fault' if args.negative_verified_block else 'build/native-virtio-block-verified')
    if args.verified_root_bundle:
        out = ROOT / 'build/native-virtio-block-verified-bundle'
out.mkdir(parents=True, exist_ok=True)
if args.negative_aes_kat:
    if component != 'NativeAES':
        parser.error('The own-source KAT control requires NativeAES')
    out = ROOT / 'build/native-aes-kat-negative'
    out.mkdir(parents=True, exist_ok=True)
    text = source.read_text()
    marker = '0x76,0x49,0xab,0xac'
    if text.count(marker) != 1:
        raise SystemExit('The exact own-source NIST answer was not uniquely identified')
    (out / 'OriginalAESABI.h').write_bytes(source.with_name('OriginalAESABI.h').read_bytes())
    source = out / 'NativeAES.cpp'
    source.write_text(text.replace(marker, '0x77,0x49,0xab,0xac'))
destination = out / (component + '.o')
arguments = record['arguments']
command = []
index = 0
while index < len(arguments):
    value = arguments[index]
    if value == record['file']:
        command.append(str(source))
    elif value == '-o':
        command.extend(['-o', str(destination)])
        index += 1
    elif value == '-Xclang' and arguments[index + 1].startswith('-W'):
        index += 1
    elif not value.startswith('-W'):
        command.append(value)
    index += 1
command.extend(['-Wall', '-Wextra', '-Werror', '-Wno-error=unknown-warning-option', '-Wno-unused-parameter',
                '-Wno-c99-designator', '-Wno-reorder-init-list', '-Wno-deprecated-enum-enum-conversion',
                '-Wno-unnecessary-virtual-specifier', '-I', str(ROOT / '.cache/xnu-work/osfmk')])
if component in ('NativeVirtioBlock', 'NativeRamBlock', 'NativeUfs', 'NativeEmmc'):
    command += ['-I', str(ROOT / 'build/storage-abi/include')]
    if args.verified_root:
        command += ['-I', str(ROOT / 'kits/VerifiedRoot/include'), '-DNATIVE_VERIFIED_BLOCK=1']
    if args.negative_verified_block:
        command += ['-DNATIVE_VERIFIED_BLOCK_INJECT_FAULT=1']
    if args.verified_root_bundle:
        command += ['-DNATIVE_VERIFIED_BLOCK_MULTI_POLICY=1']
if component == 'NativeUfs':
    command += ['-I', str(ROOT / 'kits/VerifiedRoot/include'), '-I', str(ROOT / 'drivers/NativeVirtioBlock'),
                '-DNATIVE_VERIFIED_BLOCK=1', '-DNATIVE_VERIFIED_BLOCK_MULTI_POLICY=1', '-mgeneral-regs-only']
if component=='NativeOfflineGPU':
    command += ['-I',str(ROOT/'drivers/NativeVirtioConsole')]
if component in ('NativeLegacyDisplay','NativeHidInput'):
    command += ['-I', str(out)]
if component == 'NativeAdreno':
    command += ['-I', str(ROOT / 'kits/ArmSmmu4K'), '-I', str(ROOT / 'kits/AdrenoPM4'),
                '-I', str(ROOT / 'kits/sheng'), '-I', str(ROOT / 'kits/AdrenoFirmware'),
                '-I', str(ROOT / 'build/sheng-gmu')]
wifi = ROOT / 'drivers/GuampWifi'
if args.wifi_runtime:
    # Same C++ flags for NativeEmmc.cpp and the two provider objects: one definition of every shared inline
    # (guampwifi::Rpm becomes the shared-client variant in all of them, see GuampWifiRpm.inc).
    command += ['-DGUAMP_WIFI_RUNTIME=1', '-I', str(wifi / 'integration'), '-I', str(source.parent)]
(out / 'compile-command.json').write_text(json.dumps(command, indent=2) + '\n')
result = subprocess.run(command, cwd=record['directory'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
(out / 'compile.log').write_text(result.stdout)
if result.returncode:
    raise SystemExit(result.stdout[-3500:])
objects = [destination]
if args.wifi_runtime:
    # drivers/GuampWifi: the runtime and the XNU DMA owner with exactly the NativeEmmc C++ command, and the 25 objects of
    # the ath10k port (vendored Linux ath10k + NativeWifi shim, see drivers/GuampWifi/ath10k-port) with the kernel's C
    # code-generation flags (bsd_init recipe) but their own freestanding headers: the recipe's include paths, defines,
    # -include, -std and warning flags are dropped, the Linux sources are compiled with -w (third-party code, as Codex's
    # prepara-ath10k.py). iOS mode 2 (guamp_xnu_station.c, GUAMP_WIFI_REDWOOD) is deliberately not built.
    for name in ('guamp_xnu_sm6115_runtime', 'guamp_xnu_dma'):
        target = out / (name + '.o')
        cxx = [str(wifi / 'integration' / (name + '.cpp')) if value == str(source) else value for value in command]
        cxx[cxx.index('-o') + 1] = str(target)
        run = subprocess.run(cxx, cwd=record['directory'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        (out / ('compile-' + name + '.log')).write_text(run.stdout)
        if run.returncode:
            raise SystemExit(name + ':\n' + run.stdout[-3500:])
        objects.append(target)
    c_record = json.loads((ROOT / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/bsd/DEVELOPMENT/bsd_init.o.json').read_text().strip().rstrip(','))
    codegen, index = [], 0
    c_arguments = c_record['arguments']
    while index < len(c_arguments):
        value = c_arguments[index]
        if value == c_record['file']:
            pass
        elif value in ('-o', '-I', '-D', '-U', '-include', '-imacros', '-isystem', '-iquote', '-idirafter', '-MF', '-MJ', '-MT', '-MQ'):
            index += 1
        elif value == '-Xclang' and index + 1 < len(c_arguments) and c_arguments[index + 1].startswith('-W'):
            index += 1
        elif value.startswith(('-I', '-D', '-U', '-W', '-std=', '-isystem', '-iquote', '-flto')) or value in ('-MD', '-MP', '-MMD', '-w'):
            pass
        else:
            codegen.append(value)
        index += 1
    port = wifi / 'ath10k-port/vendor'
    resource = subprocess.run([codegen[0], '-print-resource-dir'], capture_output=True, text=True, check=True).stdout.strip()
    ath_flags = ['-std=gnu11', '-w', '-ffreestanding', '-fno-stack-protector', '-mgeneral-regs-only', '-nostdinc',
                 '-isystem', str(port / 'shim/libc'), '-isystem', str(Path(resource) / 'include'),
                 '-DCONFIG_ATH10K=1', '-DCONFIG_ATH10K_SNOC=1', '-DCONFIG_ATH10K_CE=1', '-include', 'gw_ath10k_port.h',
                 '-I', str(port / 'shim/include'), '-I', str(port / 'include'), '-I', str(port / 'out/pinned'),
                 '-I', str(port / 'out/ath10k-src')]
    ath_sources = [port / 'out/ath10k-src' / (name + '.c') for name in (
        'htc', 'ce', 'wmi-tlv', 'wmi', 'htt', 'htt_rx', 'htt_tx', 'txrx', 'mac', 'core', 'hw', 'bmi', 'p2p', 'swap', 'debug')]
    ath_sources += [port / 'shim/src/lx_rt.c']
    ath_sources += [port / 'src' / (name + '.c') for name in (
        'nw_net', 'nw_mac80211', 'nw_sta', 'nw_crypto', 'gw_runtime', 'gw_bitrate', 'gw_snoc', 'gw_snoc_api')]
    ath_sources += [port / 'out/regd.c']
    if len(ath_sources) != 25:
        raise SystemExit('ath10k port: expected 25 objects')
    for path in ath_sources:
        target = out / ('ath10k-' + ('ath_regd' if path.name == 'regd.c' else path.stem) + '.o')
        run = subprocess.run(codegen + ath_flags + ['-c', str(path), '-o', str(target)], cwd=c_record['directory'],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        if run.returncode:
            raise SystemExit(str(path) + ':\n' + run.stdout[-3500:])
        objects.append(target)
    (out / 'compile-ath10k-flags.json').write_text(json.dumps(codegen + ath_flags, indent=2) + '\n')
for obj in objects:
    data = obj.read_bytes()
    magic, cpu, subtype, kind, count, size = struct.unpack_from('<6I', data)
    if (magic, cpu, subtype, kind) != (0xfeedfacf, 0x100000c, 0, 1):
        raise SystemExit(f'Unexpected platform object architecture: {obj.name}')
    offset = 32
    deployment = None
    for _ in range(count):
        cmd, length = struct.unpack_from('<II', data, offset)
        if cmd == 0x32:
            deployment = struct.unpack_from('<II', data, offset + 8)
        offset += length
    if deployment != (2, 0x110000):
        raise SystemExit(f'Unexpected platform deployment {deployment}: {obj.name}')
archive = out / ('lib' + component + '.a')
archive.unlink(missing_ok=True)
subprocess.run(['xcrun', 'ar', 'rcs', archive, *objects], check=True)
if args.wifi_runtime:
    # The archive is force-loaded into the static kernel: every provider/port symbol must be defined in it exactly once,
    # no member may need C++ static initialisers, and no strstr (not exported by the kernel).
    nm = subprocess.run(['xcrun', 'nm', '-gP', archive], capture_output=True, text=True, check=True).stdout.splitlines()
    defined, undefined = {}, set()
    for line in nm:
        fields = line.split()
        if len(fields) > 1 and fields[1] in 'TSDBC':
            defined[fields[0]] = defined.get(fields[0], 0) + 1
        elif len(fields) > 1 and fields[1] == 'U':
            undefined.add(fields[0])
    weak = {line.split()[-1] for line in subprocess.run(['xcrun', 'nm', '-gm', archive], capture_output=True, text=True,
                                                          check=True).stdout.splitlines() if 'weak external' in line}
    duplicates = sorted(name for name, n in defined.items() if n > 1 and name not in weak)
    missing = sorted(name for name in undefined - defined.keys()
                     if 'guampwifi' in name or name.startswith(('_guamp_sm6115_', '_gw_snoc_', '_nw_wifi_', '_guamp_rpm_shared_',
                                                                    '_guamp_wifi_firmware_', '_guamp_ath10k_', '_lx_', '_gGuampXnuDma')))
    if duplicates or missing or '_strstr' in undefined:
        raise SystemExit(f'Wi-Fi archive symbols: duplicates {duplicates[:20]} missing {missing[:20]} strstr {"_strstr" in undefined}')
    for obj in objects[1:]:
        sections = subprocess.run(['xcrun', 'otool', '-l', obj], capture_output=True, text=True).stdout
        if '__mod_init_func' in sections:
            raise SystemExit(f'Static initialiser in {obj.name}')
    print(f'NativeEmmc Wi-Fi runtime: {len(objects) - 1} provider/ath10k objects archived, {len(undefined - defined.keys())} kernel imports')
print(f'{component}: -Werror build and actual ARM64/iOS 17 object audit passed')
