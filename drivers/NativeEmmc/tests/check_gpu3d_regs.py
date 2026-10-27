#!/usr/bin/env python3
"""Check every G3D_REG_* offset of GuampGpu3D.h against Mesa's a6xx.xml (A6XX variant). The XML is read only.
usage: check_gpu3d_regs.py GuampGpu3D.h [path/to/registers/adreno]   (skips, exit 0, when the XML is not there)"""
import os, re, sys
import xml.etree.ElementTree as ET

ALIASES = {  # header name -> XML name (array element 0 as ARRAY.REG)
    'RB_MRT0_CONTROL': 'RB_MRT.CONTROL', 'RB_MRT0_BUF_INFO': 'RB_MRT.BUF_INFO',
    'RB_MRT0_FLAG_BUFFER': 'RB_MRT_FLAG_BUFFER.ADDR', 'VPC_VARYING_INTERP_MODE0': 'VPC_VARYING_INTERP.MODE',
    'VPC_VARYING_PS_REPL_MODE0': 'VPC_VARYING_PS_REPL.MODE', 'VPC_VAR_DISABLE0': 'VPC_VAR.DISABLE',
    'VFD_FETCH0': 'VFD_FETCH.BASE', 'VFD_DECODE0': 'VFD_DECODE.INSTR', 'VFD_DEST_CNTL0': 'VFD_DEST_CNTL.INSTR',
    'SP_VS_OUT_REG0': 'SP_VS_OUT.REG', 'SP_VS_VPC_DST_REG0': 'SP_VS_VPC_DST.REG',
    'SP_FS_OUTPUT_REG0': 'SP_FS_OUTPUT.REG', 'SP_FS_MRT_REG0': 'SP_FS_MRT.REG',
    'GRAS_SC_SCREEN_SCISSOR_TL': 'GRAS_SC_SCREEN_SCISSOR.TL', 'GRAS_SC_VIEWPORT_SCISSOR_TL': 'GRAS_SC_VIEWPORT_SCISSOR.TL',
}

def ok(v):
    if not v:
        return True
    return any(p.strip() == 'A6XX' or p.strip().startswith(('A6XX-', 'A5XX-', 'A4XX-', 'A3XX-', 'A2XX-'))
               for p in v.split(','))

def main():
    hdr = sys.argv[1]
    d = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser(
        '~/moto/riferimenti/mesa-freedreno/src/freedreno/registers/adreno')
    if not os.path.exists(os.path.join(d, 'a6xx.xml')):
        print('gpu3d regs: a6xx.xml not found, skipped')
        return 0
    regs = {}
    for dom in ET.parse(os.path.join(d, 'a6xx.xml')).getroot():
        if dom.tag.split('}')[-1] != 'domain' or dom.get('name') != 'A6XX':
            continue
        for el in dom:
            tag = el.tag.split('}')[-1]
            if not ok(el.get('variants')):
                continue
            if tag in ('reg32', 'reg64'):
                regs.setdefault(el.get('name'), int(el.get('offset'), 0))
            elif tag == 'array':
                base = int(el.get('offset'), 0)
                for r in el:
                    if r.tag.split('}')[-1] in ('reg32', 'reg64') and ok(r.get('variants')):
                        regs.setdefault(el.get('name') + '.' + r.get('name'), base + int(r.get('offset'), 0))
    bad = n = 0
    for name, val in re.findall(r'#define G3D_REG_(\w+) (0x[0-9a-fA-F]+)', open(hdr).read()):
        xml = ALIASES.get(name, name)
        n += 1
        if xml not in regs:
            print('gpu3d regs: %s (%s) not in a6xx.xml' % (name, xml))
            bad += 1
        elif regs[xml] != int(val, 16):
            print('gpu3d regs: %s = %s, a6xx.xml %s = 0x%x' % (name, val, xml, regs[xml]))
            bad += 1
    print('gpu3d regs: %d offsets checked against a6xx.xml, %d wrong' % (n, bad))
    return 1 if bad else 0

if __name__ == '__main__':
    sys.exit(main())
