// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp PMIC service, included by NativeEmmc.cpp after UsbMirror.inc (same file for the mapping helper and the boot log).
// One kernel thread on the SPMI PMIC arbiter (GuampSpmi.c logic, GuampPmicBoard.h facts):
//   buttons   Power and Volume Down from the PM6125 PON real-time status, Volume Up from PM6125 GPIO 5, polled every
//             25 ms with a 2-sample debounce, sent to NativeHidInput's button sink (report 2: bit 1 Power, bit 2 Volume
//             Up, bit 3 Volume Down) when the pressed set changes
//   charger   PMI632 SMB5: pets the charger watchdog, enables charging and raises the USB input current limit and the
//             fast charge current once a charger is detected; register offsets from Linux qcom_smbx.c (SMB2/SMB5 share
//             them); the current codes assume 50 mA steps, so a 25 mA hardware step only halves them (never exceeds)
// Boot arguments: guamp-pmic=0 turns it off; guamp-icl-ma (default 1500) and guamp-fcc-ma (default 2000).
// Every 30 s one log line: buttons, charger status bytes, APSD result, the codes written, SPMI errors.

#include "GuampSpmi.h"
#include "GuampPmicBoard.h"

namespace pmic {
// --- SPMI arbiter v5 (copy of GuampSpmi.c in C++ form, host-tested there) --------------------------------------
volatile uint8_t *gCore, *gCnfg, *gObs, *gChn;
uint32_t ioRd(void *, unsigned region, uint32_t off) {
  volatile uint8_t *base = region == SPMI_CORE ? gCore : region == SPMI_CNFG ? gCnfg : region == SPMI_OBSRVR ? gObs : gChn;
  return *reinterpret_cast<volatile uint32_t *>(base + off);
}
void ioWr(void *, unsigned region, uint32_t off, uint32_t v) {
  volatile uint8_t *base = region == SPMI_CORE ? gCore : region == SPMI_CNFG ? gCnfg : region == SPMI_OBSRVR ? gObs : gChn;
  *reinterpret_cast<volatile uint32_t *>(base + off) = v;
  __asm__ volatile("dsb sy" ::: "memory");
}
void ioDelay(void *, unsigned us) { IODelay(us); }

struct spmi_arb gArb;

int init() {
  struct spmi_io io = {nullptr, ioRd, ioWr, ioDelay};
  gArb.io = io;
  gArb.ee = 0;
  gArb.version = ioRd(nullptr, SPMI_CORE, 0);
  if (gArb.version < SPMI_V5_MIN || gArb.version >= SPMI_V7_MIN) return SPMI_ERR_VERSION;
  gArb.apids = ioRd(nullptr, SPMI_CORE, 4) & 0x7ffu;
  if (gArb.apids > SPMI_MAX_APIDS) gArb.apids = SPMI_MAX_APIDS;
  for (unsigned n = 0; n < gArb.apids; ++n) {
    uint32_t map = ioRd(nullptr, SPMI_CORE, 0x900 + 4 * n);
    if (!map) continue;
    uint16_t ppid = static_cast<uint16_t>((map >> 8) & 0xfffu);
    gArb.write_owner[n] = static_cast<uint8_t>(ioRd(nullptr, SPMI_CNFG, 0x700 + 4 * n) & 7u);
    if (!gArb.ppid_to_apid[ppid] || gArb.write_owner[n] == 0) gArb.ppid_to_apid[ppid] = static_cast<uint16_t>(n + 1);
  }
  return SPMI_OK;
}
int waitDone(unsigned region, uint32_t chan) {
  for (unsigned n = 0; n < SPMI_TIMEOUT_POLLS; ++n) {
    uint32_t st = ioRd(nullptr, region, chan + 0x08);
    gArb.last_status = st;
    if (st & 1u) return (st & 0xeu) ? ((st & 4u) ? SPMI_ERR_PERM : SPMI_ERR_IO) : SPMI_OK;
    IODelay(1);
  }
  return SPMI_ERR_TIMEOUT;
}
int apidOf(unsigned sid, uint16_t addr) {
  uint16_t e = gArb.ppid_to_apid[(sid << 8) | (addr >> 8)];
  return e ? e - 1 : SPMI_ERR_NODEV;
}
int read8(unsigned sid, uint16_t addr, uint8_t *v) {
  int apid = apidOf(sid, addr);
  if (apid < 0) return apid;
  uint32_t chan = 0x80u * static_cast<uint32_t>(apid);
  ioWr(nullptr, SPMI_OBSRVR, chan, (1u << 27) | (static_cast<uint32_t>(addr & 0xff) << 4));
  int r = waitDone(SPMI_OBSRVR, chan);
  if (!r) *v = static_cast<uint8_t>(ioRd(nullptr, SPMI_OBSRVR, chan + 0x18) & 0xff);
  return r;
}
int write8(unsigned sid, uint16_t addr, uint8_t v) {
  int apid = apidOf(sid, addr);
  if (apid < 0) return apid;
  if (gArb.write_owner[apid] != 0) return SPMI_ERR_PERM;
  uint32_t chan = 0x10000u * static_cast<uint32_t>(apid);
  ioWr(nullptr, SPMI_CHNLS, chan + 0x10, v);
  ioWr(nullptr, SPMI_CHNLS, chan, static_cast<uint32_t>(addr & 0xff) << 4);
  return waitDone(SPMI_CHNLS, chan);
}

// --- board ---------------------------------------------------------------------------------------------------------
constexpr unsigned kPm6125 = 0, kPmi632 = 2;
constexpr uint16_t kPonRtSts = 0x0810, kVolUpStatus = 0xc408;
// SMB5 charger (PMI632 0x1000 block, offsets of qcom_smbx.c relative to it)
constexpr uint16_t kChgStatus1 = 0x1006, kChgStatus2 = 0x1007, kChgStatus7 = 0x100d, kChgEnableCmd = 0x1042,
                   kFccCfg = 0x1061, kApsdStatus = 0x1307, kApsdResult = 0x1308, kUsbinCmdIl = 0x1340,
                   kIclOptions = 0x1366, kIclCfg = 0x1370, kWdogPet = 0x1643;

void (*volatile gSink)(void *, uint32_t);
void *volatile gSinkContext;
volatile uint32_t gButtons, gErrors;
volatile uint8_t gChg[6];

uint32_t readButtons() {
  uint8_t pon = 0, vup = 1;
  uint32_t mask = 0;
  if (read8(kPm6125, kPonRtSts, &pon)) gErrors = gErrors + 1;
  else {
    if (pon & 1u) mask |= 1u << 1;     // Power
    if (pon & 2u) mask |= 1u << 3;     // Volume Down (RESIN)
  }
  if (read8(kPm6125, kVolUpStatus, &vup)) gErrors = gErrors + 1;
  else if (!(vup & 1u)) mask |= 1u << 2;   // Volume Up, active low
  return mask;
}

void charger(bool configure, unsigned iclMa, unsigned fccMa, uint8_t *iclCode, uint8_t *fccCode) {
  write8(kPmi632, kWdogPet, 1);
  uint8_t s1 = 0, s2 = 0, s7 = 0, apsd = 0, result = 0;
  read8(kPmi632, kChgStatus1, &s1); read8(kPmi632, kChgStatus2, &s2); read8(kPmi632, kChgStatus7, &s7);
  read8(kPmi632, kApsdStatus, &apsd); read8(kPmi632, kApsdResult, &result);
  gChg[0] = s1; gChg[1] = s2; gChg[2] = s7; gChg[3] = apsd; gChg[4] = result;
  if (!configure) return;
  *iclCode = static_cast<uint8_t>(iclMa / 50 > 0xff ? 0xff : iclMa / 50);
  *fccCode = static_cast<uint8_t>(fccMa / 50 > 0xff ? 0xff : fccMa / 50);
  write8(kPmi632, kChgEnableCmd, 1);       // charging enable command
  write8(kPmi632, kFccCfg, *fccCode);
  write8(kPmi632, kIclCfg, *iclCode);
  write8(kPmi632, kIclOptions, 0x02);      // USB 5/1 high current mode: the configured limit applies
  write8(kPmi632, kUsbinCmdIl, 0x00);      // input not suspended
}

void sinkFromHid(void (*sink)(void *, uint32_t), void *context) {
  gSink = nullptr;
  __asm__ volatile("dmb ish" ::: "memory");
  gSinkContext = context;
  __asm__ volatile("dmb ish" ::: "memory");
  gSink = sink;
}

void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-pmic");
  gCore = mapPhysical(GP_SPMI_CORE, GP_SPMI_CORE_SIZE);
  gCnfg = mapPhysical(GP_SPMI_CNFG, 0x26000);
  gObs = mapPhysical(GP_SPMI_OBSRVR, GP_SPMI_OBSRVR_SIZE);
  gChn = mapPhysical(GP_SPMI_CHNLS, GP_SPMI_CHNLS_SIZE);
  if (!gCore || !gCnfg || !gObs || !gChn) { IOLog("GuampPmic: mapping failed\n"); return; }
  int r = init();
  IOLog("GuampPmic: SPMI arbiter version %08x, %u APIDs, init %d\n", gArb.version, gArb.apids, r);
  if (r) return;
  unsigned iclMa = 1500, fccMa = 2000;
  PE_parse_boot_argn("guamp-icl-ma", &iclMa, sizeof(iclMa));
  PE_parse_boot_argn("guamp-fcc-ma", &fccMa, sizeof(fccMa));
  if (iclMa > 3000) iclMa = 3000;
  if (fccMa > 3000) fccMa = 3000;
  uint32_t stable = 0, last = 0, sent = 0;
  uint8_t iclCode = 0, fccCode = 0;
  bool configured = false;
  for (unsigned tick = 0;; ++tick) {
    uint32_t now = readButtons();
    stable = now == last ? stable + 1 : 0;
    last = now;
    if (stable == 1 && now != sent) {             // the same set on two polls in a row
      sent = now;
      gButtons = now;
      if (void (*sink)(void *, uint32_t) = gSink) sink(gSinkContext, now);
      IOLog("GuampPmic: buttons %x\n", now);
    }
    if (tick % 200 == 0) {                        // every 5 s: watchdog pet, status; configure once a charger is seen
      bool present = (gChg[3] & 0x01) != 0 || gChg[4] != 0;
      charger(present && !configured, iclMa, fccMa, &iclCode, &fccCode);
      if (present && !configured) configured = true;
      if (!present) configured = false;
      if (tick % 1200 == 0)
        IOLog("GuampPmic: buttons %x, charger status %02x %02x %02x, APSD %02x result %02x, ICL code %u FCC code %u, SPMI errors %u (last %08x)\n",
              gButtons, gChg[0], gChg[1], gChg[2], gChg[3], gChg[4], iclCode, fccCode, gErrors, gArb.last_status);
    }
    IOSleep(25);
  }
}

void start() {
  unsigned enabled = 1;
  PE_parse_boot_argn("guamp-pmic", &enabled, sizeof(enabled));
  if (!enabled) return;
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}
}  // namespace pmic

// NativeHidInput registers its button sink here (weak import on its side)
extern "C" void native_buttons_set_sink(void (*sink)(void *, uint32_t), void *context) { pmic::sinkFromHid(sink, context); }
