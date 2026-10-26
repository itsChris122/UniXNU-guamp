"""Exact pinned ArmCpu transformation; default hardware path unchanged."""
from pathlib import Path
import hashlib,json,sys
ORIGINAL_SHA256 = '3c87b3bd617be5582a6a24ff797c359497ee9feaa702ee883ecee4270db21c9f'
PATCHED_SHA256 = 'db92ed80dea58ab3ed9636f3e7a9ca934d70da3b7ea7a3ca6726eccd70950429'
EDITS = [('static bool gSpiIpi = false;', 'static bool gPendingIpi = false; // opt-in guamp-ipi-pending=1: SGI0 through target GICR_ISPENDR0\nstatic bool gSpiIpi = false;'), ('volatile uint64_t armcpu_ipi_selftest_sent = 0;', 'volatile uint64_t armcpu_ipi_selftest_sent = 0;\nvolatile uint32_t armcpu_ipi_pending_mode = 0;'), ('  gSelfIpi = selfIpi == 1;', '  gSelfIpi = selfIpi == 1;\n  unsigned pendingIpi = 0;\n  PE_parse_boot_argn ("guamp-ipi-pending", &pendingIpi, sizeof (pendingIpi));\n  gPendingIpi = pendingIpi == 1;\n  armcpu_ipi_pending_mode = gPendingIpi ? 1 : 0;'), ('  if (gSpiIpi) { spiIpiSend (gDist, peer->logicalId & 7); return; }', '  if (gPendingIpi) {\n    // GICv3: each redistributor\'s SGI frame +0x0200 sets SGI/PPI pending.\n    // Use the frame found for this target\'s MPIDR, never a logical-ID stride.\n    // W1S: do not read/modify/write or pend unrelated interrupts.\n    if (peer->redist == 0) { panic ("ArmCpu: IPI target redistributor unavailable"); }\n    __asm__ volatile ("dsb ishst" ::: "memory");\n    write32 (peer->redist + 0x10000, 0x200, 1U);\n    __asm__ volatile ("dsb sy; isb" ::: "memory");\n    return;\n  }\n  if (gSpiIpi) { spiIpiSend (gDist, peer->logicalId & 7); return; }')]
def transform(data):
    digest = hashlib.sha256(data).hexdigest()
    if digest == PATCHED_SHA256: return data
    assert digest == ORIGINAL_SHA256, 'Refuse an unreviewed ArmCpu revision'
    text = data.decode()
    for old,new in EDITS:
        assert text.count(old) == 1
        text = text.replace(old,new)
    result = text.encode()
    assert hashlib.sha256(result).hexdigest() == PATCHED_SHA256
    return result
if __name__ == '__main__':
    path = Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[1]/'drivers/ArmCpu/ArmCpu.cpp'
    path.write_bytes(transform(path.read_bytes()))
    print('ArmCpu: verified opt-in GICR IPI variant, default unchanged')
