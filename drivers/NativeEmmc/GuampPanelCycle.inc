// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp panel power cycle (guamp-touch-panelcycle=5), included by NativeEmmc.cpp after GuampScanout.inc (needs the
// backlight DSI window and lock, the scanout hold, the PMIC SPMI arbiter and the touch pins). The sequence, its Linux
// sources and its recovery rules are in GuampPanelCycleCore.h; this file only maps INTF_1, reads the options and runs it
// from the touch thread (TouchService.inc, after the touch pins are configured and before the SPI bus is opened).
// Options: guamp-touch-pc-lcdb=0 (no rail cycle: DSI off / on and the reset only), guamp-touch-pc-rstlow=0 (LCD reset
// kept high while the rails are off, exactly as Linux with this dtbo), guamp-touch-pc-mv=<mV> (LDO / NCP after the ramp,
// default 5600 as the dtbo supply; 0 = keep the boot loader's value), guamp-touch-pc-off=<ms> (rails off time, default 10).
// Logs: every line starts with "GuampPanel:".
#include <stdarg.h>
#include "GuampPanelCycleCore.h"

namespace panelcycle {
volatile uint8_t *gIntf;
struct Io : backlight::Io {
  uint32_t intfRd(uint32_t o) { return *reinterpret_cast<volatile uint32_t *>(gIntf + o); }
  void intfWr(uint32_t o, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(gIntf + o) = v; __asm__ volatile("dsb sy" ::: "memory"); }
  int spmiRd(uint16_t off, uint8_t *v) {
    return pmic::read8(guamp_panelcycle::kLcdbSid, static_cast<uint16_t>(guamp_panelcycle::kLcdbBase + off), v);
  }
  int spmiWr(uint16_t off, uint8_t v) {
    return pmic::write8(guamp_panelcycle::kLcdbSid, static_cast<uint16_t>(guamp_panelcycle::kLcdbBase + off), v);
  }
  bool spmiWritable() {
    const int apid = pmic::apidOf(guamp_panelcycle::kLcdbSid, guamp_panelcycle::kLcdbBase);
    return apid >= 0 && pmic::gArb.write_owner[apid] == 0;
  }
  bool lcdResetIsOutput() { return (touch::pinRead(GT_GPIO_LCD_RESET, 0) & GT_PIN_OE) != 0; }
  unsigned lcdResetLevel() { return touch::pinRead(GT_GPIO_LCD_RESET, 4) & 1u; }
  void lcdReset(bool high) { touch::pinWrite(GT_GPIO_LCD_RESET, 4, high ? 2u : 0u); }
  void tpReset(bool high) { touch::pinWrite(GT_GPIO_RESET, 4, high ? 2u : 0u); }
  void log(const char *fmt, ...) __attribute__((format(printf, 2, 3))) {
    va_list ap;
    va_start(ap, fmt);
    IOLogv(fmt, ap);
    va_end(ap);
  }
};

int run() {
  guamp_panelcycle::Options o;
  unsigned v = 1;
  PE_parse_boot_argn("guamp-touch-pc-lcdb", &v, sizeof(v));
  o.lcdb = v != 0;
  v = 1;
  PE_parse_boot_argn("guamp-touch-pc-rstlow", &v, sizeof(v));
  o.resetLow = v != 0;
  v = guamp_panelcycle::kLinuxPanelMv;
  PE_parse_boot_argn("guamp-touch-pc-mv", &v, sizeof(v));
  o.targetMv = v;
  v = 10;
  if (PE_parse_boot_argn("guamp-touch-pc-off", &v, sizeof(v)) && v <= 2000) o.offMs = v;
  o.level = backlight::gLastLevel ? backlight::gLastLevel : guamp_backlight::kDefaultLevel;
  if (!backlight::gDsi) { IOLog("GuampPanel: refused: no DSI window (guamp-backlight=0?)\n"); return -1; }
  if (!gIntf) {
    volatile uint8_t *page = mapPhysical(guamp_panelcycle::kIntf1Phys & ~0xfffULL, 0x1000);
    if (!page) { IOLog("GuampPanel: refused: INTF1 not mapped\n"); return -1; }
    gIntf = page + (guamp_panelcycle::kIntf1Phys & 0xfffULL);
  }
  if (o.lcdb) {
    for (unsigned i = 0; i < 300 && !pmic::gArbReady; ++i) IOSleep(10);
    if (!pmic::gArbReady) { IOLog("GuampPanel: refused: SPMI arbiter not ready\n"); return -1; }
  }
  // No other DSI command (backlight levels, wake commands) and no scanout flip while the link is in command mode.
  backlight::dsiLock();
  gscan::hold(true);
  Io io;
  guamp_panelcycle::Report r;
  const uint64_t t0 = backlight::nowUs();
  const int result = guamp_panelcycle::run(io, o, r);
  gscan::hold(false);
  backlight::dsiUnlock();
  IOLog("GuampPanel: RESULT %d in %llu ms (lcdb en %02x->%02x->%02x rt %02x->%02x->%02x, VREG_OK after %u polls, frames %u->%u)\n",
        result, (unsigned long long)((backlight::nowUs() - t0) / 1000u), r.enBefore, r.enDown, r.enUp, r.rtBefore, r.rtDown, r.rtUp,
        r.pollsUp, r.frames[4], r.frames[5]);
  return result;
}
}  // namespace panelcycle
