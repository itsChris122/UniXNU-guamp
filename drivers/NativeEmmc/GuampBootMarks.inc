// GuampBoot checkpoints in the kernel log (boot speed, 05/11/2026; GuampBootMarksCore.h has the ring format).
// guamp-gb-marks=0 turns it off (default 1: a read-only mapping of GuampBoot's RAM ring outside the XNU window, one
// line). read() runs once at the start of the driver; the line is printed then and again with the first three
// 10 s reports, so the USB mirror (which starts ~2-3 s after the kernel) has it too. Under the UEFI chain the ring
// has no "GBLG" magic and the line says so.
#include "GuampBootMarksCore.h"

namespace gbmarks {
static char gLine[320];
static unsigned gPrinted = 0;

static inline uint64_t counter() {
  uint64_t t;
  __asm__ volatile("isb\n\tmrs %0, cntpct_el0" : "=r"(t));
  return t;
}
static inline uint64_t frequency() {
  uint64_t f;
  __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(f));
  return f & 0xffffffffu;
}

static void read() {
  uint32_t on = 1;
  PE_parse_boot_argn("guamp-gb-marks", &on, sizeof(on));
  if (!on) return;
  const uint64_t now = counter(), freq = frequency();
  // The ring is not RAM XNU manages (the window ends at 0x13FA7C000): checked here, so a different layout never maps
  // a page the VM owns.
  if (kRingPhys < guamp_phys_base + guamp_phys_size && guamp_phys_base < kRingPhys + kRingBytes) {
    snprintf(gLine, sizeof(gLine), "GuampBoot: ring 0x%llx inside the XNU window, not read", kRingPhys);
    IOLog("%s\n", gLine);
    return;
  }
  IOMemoryDescriptor *d = IOMemoryDescriptor::withPhysicalAddress(kRingPhys, kHeader + kMaxText, kIODirectionIn);
  IOMemoryMap *map = d ? d->map(kIOMapInhibitCache | kIOMapReadOnly) : nullptr;
  if (d) d->release();
  if (!map) { snprintf(gLine, sizeof(gLine), "GuampBoot: ring not mapped"); IOLog("%s\n", gLine); return; }
  const volatile uint8_t *src = reinterpret_cast<const volatile uint8_t *>(map->getVirtualAddress());
  uint8_t *copy = static_cast<uint8_t *>(IOMallocZero(kHeader + kMaxText));
  if (copy) {
    for (size_t i = 0; i < kHeader + kMaxText; i++) copy[i] = src[i];   // byte loads: the mapping is device memory
    const Result r = parse(copy, reinterpret_cast<const char *>(copy + kHeader), kMaxText, now, 120 * freq);
    format(r, now, freq, gLine, sizeof(gLine));
    IOFree(copy, kHeader + kMaxText);
  }
  map->release();
  if (gLine[0]) IOLog("%s\n", gLine);
}

static void reprint() {   // from the 10 s report
  if (gLine[0] && gPrinted < 3) {
    gPrinted++;
    IOLog("%s\n", gLine);
  }
}
}  // namespace gbmarks
