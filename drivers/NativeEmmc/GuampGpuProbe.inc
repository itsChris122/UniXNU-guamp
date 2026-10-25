// SPDX-License-Identifier: MIT
// Initial A610 bring-up: clock snapshot only. No GPU/SMMU MMIO or writes.
// Include in NativeEmmc.cpp after mapPhysical(), call guamp_gpu::start().
// Addresses: boot_a-0.dts; offsets: Motorola gpucc-bengal.c.
namespace guamp_gpu {
static void start()
{
  unsigned enabled = 0;
  if (!PE_parse_boot_argn("guamp-gpu-probe", &enabled, sizeof(enabled)) || !enabled) return;
  if (enabled != 1) {
    IOLog("GuampGPU: unsupported probe level %u; no register access\n", enabled);
    return;
  }
  volatile uint8_t *cc = mapPhysical(0x05990000ULL, 0x9000);
  if (!cc) { IOLog("GuampGPU: GPUCC mapping failed\n"); return; }
  struct Register { const char *name; uint32_t offset; };
  static const Register regs[] = {
    {"AHB", 0x1078}, {"CX_GFX3D", 0x10a4}, {"GX_GFX3D", 0x1054},
    {"CX_GMU", 0x1098}, {"CX_SNOC_DVM", 0x108c}, {"SLEEP", 0x1090},
    {"GFX3D_RCG_CMD", 0x101c}, {"GFX3D_RCG_CFG", 0x1020}
  };
  IOLog("GuampGPU: A610 board profile, expected chipid 06010000 (DT, not hardware detection)\n");
  for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i) {
    uint32_t value = *reinterpret_cast<volatile uint32_t *>(cc + regs[i].offset);
    IOLog("GuampGPU: GPUCC %s +%04x = %08x\n", regs[i].name, regs[i].offset, value);
  }
  IOLog("GuampGPU: clock snapshot complete; GPU power, firmware, SMMU and submission not initialized\n");
}
}
