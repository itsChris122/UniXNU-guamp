"""Host patches of the common UniXNU modules (guamp), applied by the kernel step itself (first [kernel] archive).

Until 05/11 the CI pre_build applied them by hand (git -C kits/NativePrelink apply ...); a run started with only
"git checkout --detach 245aca1a..." prelinked with the unpatched NativePrelink and stopped on FairPlayIOKit
(Kernel target 0xfffffff0077a65a2 = IOMallocAligned+2 has 0 native address candidates, CI 42127660201), and it
would have built NativeAES without the emulated hardware keys. Idempotent: an already patched module is left alone.
"""
from pathlib import Path
import os, subprocess

TREE = Path(__file__).absolute().parents[1]
# (module of the tree, patch of the device repository, text that only the patched sources contain)
PATCHES = [
    ('kits/NativePrelink', 'patches/host/nativeprelink-kernel-bias.patch',
     {'native_prelink.py': 'MAX_KERNEL_BIAS', 'emit_prelink.py': 'opaque_external_words'}),
    ('drivers/NativeAES', 'patches/host/nativeaes-hwkeys.patch', {'NativeAES.cpp': 'native_aes_emulated_key'}),
    # 06/11: DMS Get Revision answer shortened for Settings > About > Modem Firmware ("...KAMORTA_GEN_PACK-1.438466.25" -> "1.438466.25").
    # Device submodule (UniXNU/QcomBaseband dd0199c), only present with --baseband: skipped when not initialised.
    ('drivers/QcomBaseband', 'patches/host/qcombaseband-short-revision.patch',
     {'baseband/coachz_bb_data.h': 'cbd_revision_fix', 'baseband/coachz_baseband.cpp': 'cbd_revision_fix'}),
    # 07/11: UIM provisioning session (0x38) for the card's USIM: the SM6115 modem waits for the host (QCRIL on Android) to open it,
    # CommCenter never asks, so the SIM stayed unread ("SOS", no ICCID/IMSI). Applies after qcombaseband-short-revision.patch.
    ('drivers/QcomBaseband', 'patches/host/qcombaseband-uim-provisioning.patch',
     {'baseband/coachz_bb_uimprov.h': 'cbu_imsi_request', 'baseband/coachz_baseband.cpp': 'bb_uim_imsi_result'}),
]
OPTIONAL = {'drivers/QcomBaseband'}


def device_root():
    candidates = [os.environ.get('UNIXNU_DEVICE')]
    firmware = TREE / '.cache/firmware'   # symlink to <device>/.cache/firmware (unixnu tree)
    if firmware.is_symlink():
        candidates.append(str(firmware.resolve().parents[1]))
    for candidate in candidates:
        if candidate and (Path(candidate) / PATCHES[0][1]).is_file():
            return Path(candidate)
    raise SystemExit('guamp-host-patches: device repository not found (UNIXNU_DEVICE, .cache/firmware)')


def patched(module, markers):
    return all((module / name).is_file() and marker in (module / name).read_text(errors='replace') for name, marker in markers.items())


def main():
    device = device_root()
    for relative, patch, markers in PATCHES:
        module = (TREE / relative).resolve()
        if not module.is_dir() or (relative in OPTIONAL and not any(module.iterdir())):
            if relative in OPTIONAL:
                print(f'{relative}: not in the tree, {patch} skipped')
                continue
            raise SystemExit(f'guamp-host-patches: {relative} missing in the tree')
        if patched(module, markers):
            print(f'{relative}: already patched')
            continue
        for check in (['--check'], []):
            result = subprocess.run(['git', '-C', str(module), 'apply', *check, str(device / patch)],
                                    capture_output=True, text=True)
            if result.returncode:
                raise SystemExit(f'guamp-host-patches: {patch} does not apply to {relative}: {result.stderr.strip()}')
        if not patched(module, markers):
            raise SystemExit(f'guamp-host-patches: {relative} still unpatched after {patch}')
        print(f'{relative}: applied {patch}')


if __name__ == '__main__':
    main()
