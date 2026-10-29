#!/usr/bin/env python3
"""Rewrites, in place, the dylib load commands of a thin arm64 Mach-O that name /System/Library/PrivateFrameworks/<N>.framework/<N>
into @rpath/<N>.framework/<N> for the given framework names (guamp: frameworks that exist only in the iPhone's shared cache travel
inside the app bundle, <App>.app/Frameworks, found through the app's LC_RPATH @executable_path/Frameworks).
The new name is shorter, so each command keeps its size and the rest of the string is zero filled: no offset in the file moves.
LC_ID_DYLIB is left alone (a dlopen by the system path still matches the loaded image). The existing signature becomes invalid:
sign afterwards. Refuses fat files, a name that is not found, and a command without an LC_RPATH @executable_path/Frameworks when
--need-rpath is given.
  macho-rpath-frameworks.py FILE NAME [NAME...] [--need-rpath]"""
import struct
import sys

LOADS = {0xc: 'LC_LOAD_DYLIB', 0x80000018: 'LC_LOAD_WEAK_DYLIB', 0x8000001f: 'LC_REEXPORT_DYLIB', 0x20: 'LC_LAZY_LOAD_DYLIB',
         0x80000023: 'LC_LOAD_UPWARD_DYLIB'}


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    need_rpath = '--need-rpath' in sys.argv
    path, names = args[0], args[1:]
    data = bytearray(open(path, 'rb').read())
    magic, _, _, _, ncmds, sizeofcmds = struct.unpack_from('<6I', data)
    assert magic == 0xfeedfacf, path + ': not a thin 64-bit Mach-O'
    old = {f'/System/Library/PrivateFrameworks/{n}.framework/{n}': f'@rpath/{n}.framework/{n}' for n in names}
    done, rpaths, at = set(), [], 32
    for _ in range(ncmds):
        cmd, size = struct.unpack_from('<II', data, at)
        if cmd == 0x8000001c:  # LC_RPATH
            off = struct.unpack_from('<I', data, at + 8)[0]
            rpaths.append(bytes(data[at + off:at + size]).split(b'\0')[0].decode())
        if cmd in LOADS:
            off = struct.unpack_from('<I', data, at + 8)[0]
            name = bytes(data[at + off:at + size]).split(b'\0')[0].decode()
            if name in old:
                new = old[name].encode()
                assert len(new) < size - off
                data[at + off:at + size] = new + bytes(size - off - len(new))
                done.add(name)
                print(f'{path}: {LOADS[cmd]} {name} -> {old[name]}')
        at += size
    assert at == 32 + sizeofcmds
    missing = set(old) - done
    assert not missing, f'{path}: not found: {sorted(missing)}'
    if need_rpath:
        assert '@executable_path/Frameworks' in rpaths, f'{path}: no LC_RPATH @executable_path/Frameworks ({rpaths})'
    open(path, 'wb').write(data)


if __name__ == '__main__':
    main()
