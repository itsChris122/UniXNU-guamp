#!/usr/bin/env python3
"""Text fixups that keep SpringBoard's lock screen floor (SBMinimumLockscreenIdleTime, guamp-lock-min) for the boot only.

guamp-lock-min (patches/xnu/zzzz-guamp-setup-skip.patch) writes SBMinimumLockscreenIdleTime into com.apple.springboard
at every boot, so the poster drawn ~31-35 s after the display came on stays visible. SpringBoard reads that default once
(SBIdleTimerGlobalStateMonitor, a BSAbstractDefaultDomain monitor fed by observeDefault:onQueue:withBlock:, i.e. only
cfprefsd-delivered writes) and the kernel cannot reach cfprefsd, so the value cannot be taken back after the boot.
Instead the place that applies it is gated on the time since boot:

  -[SBIdleTimerDescriptorFactory sanitizeDescriptorForLockscreenDefaults:]  (SpringBoard.framework, iPadOS 17.7.11 21H461,
  dyld_shared_cache_arm64.25 file offset 0x199b3d4 = 0x1cfa2f3d4), original after the "total > 0" test:
      ldr x0,[x21,#8]; bl minimumLockscreenIdleTime; fcmp d8,d0; fcsel d0,d0,d8,mi; mov x0,x19; bl setTotalInterval:;
      then the audit reason string (SBIdleTimerIntervalToNSString, addAuditReason:, retain/release)
  replacement (same 16 instructions, 64 bytes, 0x1cfa2f3d8..0x1cfa2f417):
      d0 = CACurrentMediaTime(); if (d0 >= BOOT + LIMIT) goto end;     // LIMIT s after this boot: no floor
      ldr x0,[x21,#8]; bl minimumLockscreenIdleTime; fcmp; fcsel; mov x0,x19; bl setTotalInterval:; b end
  BOOT: the timebase keeps counting across warm resets and fastboot boot (fin48 in RAM: ~570 s at the first log line),
  so the limit is "movz x9,#LIMIT; movk x9,#0xb007,lsl #16" and the kernel rewrites that pair to the timebase seconds
  at its start + LIMIT (patches/xnu/zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-text-fixup-uptime.patch). A kernel
  without it leaves 0xb007xxxx s (~93 years): the floor then stays for the whole boot, as with 4b12eb7 alone.
  Only the audit-reason line of the descriptor goes; x19-x21/d8 are callee-saved, so CACurrentMediaTime keeps them.

Every entry is <file>,<offset>,<original>,<replacement> (16 bytes, cs_text_fixup of patches/xnu/native-text-fixup.patch):
the kernel changes the page only when the original bytes match. Output: one entry per line, for the ADT
(scripts/make-guamp-adt.py --text-fixup ...) or the boot arguments text-fixup0..7.
Usage: gen-guamp-lockmin-fixup.py [--limit SECONDS] [--dsc dyld_shared_cache_arm64.25] [--check]"""
import argparse, shutil, struct, subprocess, sys, tempfile, os

FILE = 'dyld_shared_cache_arm64.25'
FILE_OFF = 0x199b3d8                 # first changed instruction
VM = 0x1cfa2f3d8
END = 0x1cfa2f418                    # loc_1cfa2f418: release of the descriptor, return
CA_CURRENT_MEDIA_TIME = 0x18946aee0  # QuartzCore _CACurrentMediaTime (same shared cache, same slide)
ORIGINAL = bytes.fromhex('a00640f919792d940021601e004c681ee00313aa7d512e940041601e9bbc1194'
                         'fd684595f50300aae00300f9c2420db042a02791e00313aa74672c9447694595')

def word(b, i): return struct.unpack_from('<I', b, 4 * i)[0]

def bl_target(insn, pc):
    assert insn >> 26 == 0x25, hex(insn)
    imm = insn & 0x3ffffff
    if imm & 0x2000000: imm -= 0x4000000
    return pc + 4 * imm

def bl(pc, target):
    d = (target - pc) >> 2
    assert -(1 << 25) <= d < (1 << 25)
    return 0x94000000 | (d & 0x3ffffff)

def b(pc, target):
    d = (target - pc) >> 2
    assert -(1 << 25) <= d < (1 << 25)
    return 0x14000000 | (d & 0x3ffffff)

def b_ge(pc, target):
    d = (target - pc) >> 2
    assert -(1 << 18) <= d < (1 << 18)
    return 0x54000000 | ((d & 0x7ffff) << 5) | 0xa

def adrp(pc, target, rd):
    d = (target >> 12) - (pc >> 12)
    assert -(1 << 20) <= d < (1 << 20)
    return 0x90000000 | ((d & 3) << 29) | (((d >> 2) & 0x7ffff) << 5) | rd

def add_imm(rd, rn, imm):
    assert 0 <= imm < 4096
    return 0x91000000 | (imm << 10) | (rn << 5) | rd

MARK = 0xb007                         # movk x9, #0xb007, lsl #16: "boot-relative" (zzzz...-guamp-text-fixup-uptime.patch)

def build(limit):
    assert 1 <= limit <= 0xffff
    min_stub = bl_target(word(ORIGINAL, 1), VM + 4)      # _objc_msgSend$minimumLockscreenIdleTime
    set_stub = bl_target(word(ORIGINAL, 5), VM + 20)     # _objc_msgSend$setTotalInterval:
    code = [
        adrp(VM, CA_CURRENT_MEDIA_TIME, 16),
        add_imm(16, 16, CA_CURRENT_MEDIA_TIME & 0xfff),
        0xd63f0200,                                      # blr x16
        0xd503201f,                                      # nop (the pair below stays inside one 16-byte entry)
        0xd2800000 | (limit << 5) | 9,                   # movz x9, #limit
        0xf2a00000 | (MARK << 5) | 9,                    # movk x9, #0xb007, lsl #16 (the kernel adds the boot time)
        0x9e620121,                                      # scvtf d1, x9
        0x1e612000,                                      # fcmp d0, d1
        None,                                            # b.ge end
        word(ORIGINAL, 0),                               # ldr x0, [x21, #0x8]
        None,                                            # bl minimumLockscreenIdleTime
        word(ORIGINAL, 2),                               # fcmp d8, d0
        word(ORIGINAL, 3),                               # fcsel d0, d0, d8, mi
        word(ORIGINAL, 4),                               # mov x0, x19
        None,                                            # bl setTotalInterval:
        None,                                            # b end
    ]
    code[8] = b_ge(VM + 32, END)
    code[10] = bl(VM + 40, min_stub)
    code[14] = bl(VM + 56, set_stub)
    code[15] = b(VM + 60, END)
    assert len(code) == 16 and VM + 64 == END
    return struct.pack('<16I', *code)

def entries(new):
    return ['%s,%x,%s,%s' % (FILE, FILE_OFF + i, ORIGINAL[i:i + 16].hex(), new[i:i + 16].hex()) for i in range(0, 64, 16)]

def disassemble(data):
    objdump = shutil.which('aarch64-linux-gnu-objdump')
    if not objdump: return None
    with tempfile.TemporaryDirectory() as d:
        raw = os.path.join(d, 'code.bin'); open(raw, 'wb').write(data)
        out = subprocess.run([objdump, '-D', '-b', 'binary', '-m', 'aarch64', '--adjust-vma=0x%x' % VM, raw],
                             capture_output=True, text=True)
        return out.stdout if out.returncode == 0 else None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--limit', type=int, default=180, help='seconds of uptime the floor lasts (default 180)')
    ap.add_argument('--dsc', help='the 21H461 dyld_shared_cache_arm64.25 to check the original bytes against')
    ap.add_argument('--check', action='store_true', help='host test: disassemble and check the replacement')
    a = ap.parse_args()
    new = build(a.limit)
    if a.dsc:
        with open(a.dsc, 'rb') as f:
            f.seek(FILE_OFF); have = f.read(64)
        assert have == ORIGINAL, 'the cache is not 21H461 or the function moved'
        # _CACurrentMediaTime: dyld_shared_cache_arm64.03 file offset 0x26beee0 (stp fp,lr; mov fp,sp; bl; ldp fp,lr)
        with open(os.path.join(os.path.dirname(a.dsc), 'dyld_shared_cache_arm64.03'), 'rb') as f:
            f.seek(0x26beee0); ca = f.read(16)
        assert ca == bytes.fromhex('fd7bbfa9fd0300914c591895fd7bc1a8'), 'CACurrentMediaTime is not where 21H461 has it'
        print('original bytes and CACurrentMediaTime checked in', os.path.dirname(a.dsc))
    if a.check:
        text = disassemble(new)
        assert text, 'aarch64-linux-gnu-objdump needed for --check'
        lines = [l.split('\t', 2)[-1].strip() for l in text.splitlines() if l.strip().startswith('1cfa2f')]
        want = ['adrp	x16, 0x18946a000', 'add	x16, x16, #0xee0', 'blr	x16', 'nop', 'mov	x9, #0x%x' % a.limit,
                'movk	x9, #0xb007, lsl #16', 'scvtf	d1, x9', 'fcmp	d0, d1', 'b.ge	0x1cfa2f418', 'ldr	x0, [x21, #8]',
                'bl	0x%x' % bl_target(word(ORIGINAL, 1), VM + 4), 'fcmp	d8, d0', 'fcsel	d0, d0, d8, mi', 'mov	x0, x19',
                'bl	0x%x' % bl_target(word(ORIGINAL, 5), VM + 20), 'b	0x1cfa2f418']
        assert len(lines) == 16, lines
        for got, exp in zip(lines, want):
            g = ' '.join(got.split('//')[0].split()); e = ' '.join(exp.split())
            assert g.startswith(e), (g, e)
        # the two message stubs are the ones the original called (decoded from the original bl, re-encoded from the new pc)
        assert bl_target(word(new, 10), VM + 40) == bl_target(word(ORIGINAL, 1), VM + 4)
        assert bl_target(word(new, 14), VM + 56) == bl_target(word(ORIGINAL, 5), VM + 20)
        for l in lines: print('  ' + l)
    if a.check:
        for limit in (1, 60, 180, 600, 0xffff):   # every limit encodes, the rest of the code does not move
            other = build(limit)
            assert other[:16] == new[:16] and other[20:] == new[20:] and word(other, 4) == 0xd2800000 | (limit << 5) | 9
        assert all(len(e.split(',')[2]) == 32 == len(e.split(',')[3]) and len(e) < 160 for e in entries(new))
        print('lockmin fixup: PASS')
    for e in entries(new): print(e)

if __name__ == '__main__':
    main()
