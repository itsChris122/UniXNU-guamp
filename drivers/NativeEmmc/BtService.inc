// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Bluetooth milestones M0 and M1, included by NativeEmmc.cpp after PmicService.inc (mapPhysical, the boot log,
// the SPMI helpers, gCard). A one-shot diagnostic on its own kernel thread, opt-in with boot argument guamp-bt=1 (M0)
// or guamp-bt=2 (M0, then M1):
//   1 rails   PM6125 L9A (io), L16A (xo), L17A (core), L23A (pa) read over SPMI (type, subtype, status, enable,
//             voltage); guamp-bt-ldo=1 enables the rails that are off (enable bit only), =2 also writes Linux's
//             voltage when the read-back is outside bt_wcn3990's range. Any rail the apps EE may not write stops this
//             step before the first write (RPM-owned). guamp-bt-ldo=3 instead votes the four rails on through the RPM
//             (GuampRpm.inc, ldoa 9/16/17/23, swen 1 + Linux's voltage, active and sleep set), then checks VREG_OK
//             over SPMI (read only).
//   2 uart    QUPv3 wrap0 SE3 (GCC votes like the touch SE2, DFS level or 7.3728 MHz RCG), gpio8-11 to qup3, GENI
//             UART FIFO mode polled, 8N1
//   3 hci     hci_qca.c qca_wcn3990_init: power-off pulse 0xC0 at 2400 baud, power-on pulse 0xFC at 115200 (flow
//             control off), 100 ms, re-open with RTS/CTS; then HCI_Reset, HCI_Read_Local_Version and the QCA EDL
//             version request (VSC 0xFC00, 0x19), raw bytes (at most 10 lines) and the decoded versions, plus the
//             firmware files of the bluetooth_a partition that fit the controller. At the end RX is stopped and the
//             power-off pulse is sent again (guamp-bt-keep=1 leaves the controller on).
//   4 M1      (guamp-bt=2 only, after a decoded QCA version) btqca.c qca_uart_setup for the controller family of the
//             version reply (product 0x0f = WCN3950: cmbtfw<rom>.tlv + cmnv<rom>.bin): both files read from /image of
//             the bluetooth_<slot> partition (GPT name; slot as the Wi-Fi preparation finds it, guamp-bt-slot=a|b
//             overrides) with the read-only GuampExt4.inc reader; the TLV patch header must name the controller's
//             product and ROM build. hci_qca.c qca_set_speed: flow control off (RFR not ready), VSC 0xFC48 with the
//             baud code, 1-10 ms, host to the new rate (without DFS the RCG moves to 48 MHz for 3 Mbit/s at 16x),
//             flow control on, the controller's vendor event awaited 100 ms, Read_Local_Version at the new rate.
//             Then patch, 10 ms, NVM (with Linux's NVM edits: in-band sleep, baud code, deep sleep) in 243-byte VSC
//             0xFC00/0x1E segments acked as the TLV download mode says, HCI_Reset, Read_Local_Version, the EDL
//             version (patch version after the download) and HCI_Read_BD_ADDR, compared with the NVM default as
//             qca_check_bdaddr does (no address is written). guamp-bt-baud=<rate> (default 3000000; 115200 = no
//             switch). After a successful M1 the controller stays powered (guamp-bt-keep=0 sends the power-off pulse).
// Other boot arguments: guamp-bt-delay=<s> (start later, so the USB mirror sees the lines), guamp-bt-uart=0 (rails
// only), guamp-bt-pulse=0 (no power pulses), guamp-bt-os=16|32 (UART oversampling; default from the QUP version).
// Outcome of a boot: the single "GuampBt: RESULT ..." line. Writes stay limited to SE3, gpio8-11, the SE3 GCC votes /
// RCG and (only with guamp-bt-ldo=1/2) the four LDO enable/voltage registers. Every wait has a timeout.
// Board facts: GuampBtBoard.h; UART engine: GeniUart.h; HCI framing and decoders: BtHci.h; rampatch/NVM handling:
// BtQcaFw.h (all host-tested).

#include "GuampSpmi.h"
#include "GuampPmicBoard.h"
#include "GuampBtBoard.h"
#include "GeniUart.h"
#include "BtHci.h"
#include "GeniUartCpp.inc"
#include "GuampRpmCore.h"
#include "BtQcaFw.h"
#include "GuampExt4.inc"

#ifndef GUAMP_BT_HOST_CHECK
#include <IOKit/IODeviceTreeSupport.h>   // gIODTPlane (guamp-bt-addr)
#endif
namespace pmic {
extern volatile int gArbReady;
extern struct spmi_arb gArb;
int read8(unsigned sid, uint16_t addr, uint8_t *v);
int write8(unsigned sid, uint16_t addr, uint8_t v);
int apidOf(unsigned sid, uint16_t addr);
}
namespace rpm {   // GuampRpm.inc
int request(unsigned which, uint32_t type, uint32_t id, const rpm_kvp *kv, unsigned n, unsigned timeoutMs);
}
namespace wifiprep {   // GuampWifiPrep.inc (included later): GPT lookup by name, active slot
bool findPartition(const char *wanted, uint64_t *first, uint64_t *blocks, uint64_t *attributes);
char activeSlot();
}

namespace bt {
volatile uint8_t *gGcc, *gPins, *gSwCtrl, *gWrap, *gSe;
uint32_t rd(volatile uint8_t *b, uint32_t o) { return *reinterpret_cast<volatile uint32_t *>(b + o); }
void wr(volatile uint8_t *b, uint32_t o, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(b + o) = v; __asm__ volatile("dsb sy" ::: "memory"); }
uint32_t seRd(void *ctx, uint32_t reg) { return rd(static_cast<volatile uint8_t *>(ctx), reg); }
void seWr(void *ctx, uint32_t reg, uint32_t v) { wr(static_cast<volatile uint8_t *>(ctx), reg, v); }
void delayUs(void *, unsigned us) { IODelay(us); }
uint32_t pinRead(unsigned gpio, uint32_t off) { return rd(gPins, (gpio - BT_GPIO_CTS) * 0x1000u + off); }
void pinWrite(unsigned gpio, uint32_t off, uint32_t v) { wr(gPins, (gpio - BT_GPIO_CTS) * 0x1000u + off, v); }

struct geni_uart gUart;
uint64_t gLevelHz[GT_DFS_LEVELS];
bool gDfs;
unsigned gOs = 16, gHexLines;
uint8_t gRx[640];
size_t gRxLen, gRxPos;
unsigned gIbsAcks;
bool gIbs;                                           // set once the M1 NVM (in-band sleep enabled) is in the controller
// IBS state seen from the controller: a SLEEP_IND makes it asleep until its WAKE_IND / WAKE_ACK or an event; a
// command to an asleep controller is preceded by a WAKE_IND (hci_qca.c qca_enqueue when IBS is on).
bool gAsleep;
unsigned gIbsSleeps, gIbsAcksRx, gIbsWakeSent;
// Bytes read since the last command was sent (what the controller really answered, independent of gRx compaction).
uint8_t gCap[96];
size_t gCapLen;
volatile uint32_t gStep;
// guamp-bt-addr=aa:bb:cc:dd:ee:ff: the BD address written after the NVM (EDL_WRITE_BD_ADDR) and into the ADT nodes.
uint8_t gAddr[6];
bool gHaveAddr;

// --- step 1: supplies ---------------------------------------------------------------------------------------------
struct Rail { int err, apid; unsigned owner; uint8_t type, sub, st, lb, ub, mode, en; };
Rail gRail[4];
void railRead(unsigned i) {
  Rail &r = gRail[i];
  const uint16_t b = kBtRails[i].base;
  r = Rail{};
  r.apid = pmic::apidOf(BT_SID_PM6125, b);
  r.owner = r.apid >= 0 ? pmic::gArb.write_owner[r.apid] : 7u;
  const struct { uint16_t off; uint8_t *v; } regs[7] = {{BT_LDO_TYPE, &r.type}, {BT_LDO_SUBTYPE, &r.sub}, {BT_LDO_STATUS1, &r.st},
      {BT_LDO_VSET_LB, &r.lb}, {BT_LDO_VSET_UB, &r.ub}, {BT_LDO_MODE, &r.mode}, {BT_LDO_ENABLE, &r.en}};
  for (const auto &x : regs) {
    *x.v = 0xff;
    int e = pmic::read8(BT_SID_PM6125, static_cast<uint16_t>(b + x.off), x.v);
    if (e && !r.err) r.err = e;
  }
}
bool railOn(unsigned i) { return !gRail[i].err && bt_ldo_is_on(gRail[i].en, gRail[i].st); }
void railLog(unsigned i, const char *when) {
  const Rail &r = gRail[i];
  const struct bt_rail &d = kBtRails[i];
  if (r.err && r.apid < 0) {
    IOLog("GuampBt: %s %s(%s) 0x%04x: no APID for the apps EE (SPMI %d), not readable\n", when, d.name, d.role, d.base, r.err);
    return;
  }
  const unsigned mv = bt_ldo_mv(r.lb, r.ub);
  IOLog("GuampBt: %s %s(%s) 0x%04x apid %d owner ee%u%s type %02x sub %02x st %02x en %02x mode %02x vset %u mV (Linux %u-%u)%s -> %s\n",
        when, d.name, d.role, d.base, r.apid, r.owner, r.owner == GP_SPMI_EE ? "" : " (not ours)", r.type, r.sub, r.st, r.en,
        r.mode, mv, d.min_mv, d.max_mv, r.err ? " SPMI error" : "", r.err ? "?" : railOn(i) ? "ON" : "OFF");
}
bool waitRailOk(unsigned i) {
  for (unsigned n = 0; n < 20; ++n) {
    uint8_t st = 0;
    if (!pmic::read8(BT_SID_PM6125, static_cast<uint16_t>(kBtRails[i].base + BT_LDO_STATUS1), &st) && (st & BT_LDO_VREG_OK)) return true;
    IOSleep(1);
  }
  return false;
}
// guamp-bt-ldo=3: the rails voted on through the RPM, as Linux's rpm-smd-regulator does for bluetooth-power.c
// (qcom,set = <3>): per rail one request in the active set and one in the sleep set, each with swen = 1 and uv = the
// voltage Linux ends up with (kRpmBtLdo, GuampRpmCore.h). With SPMI up, VREG_OK is then polled and the rail re-read
// (read only). Stops at the first rail when the RPM link itself is down.
const char *railsRpm(bool spmi) {
  unsigned ok = 0;
  for (unsigned i = 0; i < 4; ++i) {
    const rpm_bt_ldo &l = kRpmBtLdo[i];
    const rpm_kvp kv[2] = {{RPM_KEY_SWEN, 4, 1}, {RPM_KEY_UV, 4, l.uv}};
    const int a = rpm::request(RPM_SET_ACTIVE, RPM_RES_LDOA, l.id, kv, 2, 500);
    const int s = a ? a : rpm::request(RPM_SET_SLEEP, RPM_RES_LDOA, l.id, kv, 2, 500);
    const char *chk = "";
    if (spmi && !a) {
      const bool readable = !gRail[i].err;   // from the read before the votes
      const bool vreg = readable && waitRailOk(i);
      railRead(i);
      railLog(i, "rpm");
      chk = !readable ? ", VREG_OK not readable over SPMI" : vreg ? ", VREG_OK" : ", no VREG_OK in 20 ms";
    }
    IOLog("GuampBt: rails: rpm %s(%s) ldoa %u %u uV swen 1: active %s, sleep %s%s\n", l.name, kBtRails[i].role, l.id, l.uv,
          rpm_err_name(a), a ? "not sent" : rpm_err_name(s), chk);
    if (!a && !s) ok = ok + 1;
    if (a == RPM_ERR_NOMAP || a == RPM_ERR_TOC || a == RPM_ERR_FIFO || a == RPM_ERR_VERSION || a == RPM_ERR_OPEN) return "rpm-down";
  }
  return ok == 4 ? "rpm-on" : ok ? "rpm-partial" : "rpm-refused";
}

// Returns a short word for the RESULT line.
const char *rails() {
  gStep = 1;
  unsigned ldo = 0;
  PE_parse_boot_argn("guamp-bt-ldo", &ldo, sizeof(ldo));
  for (unsigned i = 0; i < 300 && !pmic::gArbReady; ++i) IOSleep(10);
  if (!pmic::gArbReady) {
    IOLog("GuampBt: rails: PMIC service not ready (guamp-pmic=0?), SPMI not read%s\n", ldo == 3 ? ", RPM votes only" : "");
    return ldo == 3 ? railsRpm(false) : "no-spmi";
  }
  unsigned off = 0, unknown = 0;
  for (unsigned i = 0; i < 4; ++i) {
    railRead(i);
    railLog(i, "rail");
    if (gRail[i].err) unknown = unknown + 1;
    else if (!railOn(i)) off = off + 1;
  }
  if (ldo == 3) return railsRpm(true);   // voted even when on: keeps them on for the apps master
  if (!off) return unknown ? "unknown" : "on";
  if (ldo != 1 && ldo != 2) {
    IOLog("GuampBt: rails: %u off, not enabled (guamp-bt-ldo=1 tries SPMI, =3 the RPM)\n", off);
    return "off";
  }
  // No write at all unless every rail to switch is ours to write (spmi-pmic-arb write owner = apps EE 0).
  for (unsigned i = 0; i < 4; ++i) {
    if (railOn(i) || gRail[i].err) continue;
    if (gRail[i].apid < 0 || gRail[i].owner != GP_SPMI_EE) {
      IOLog("GuampBt: rails: %s write owner ee%u, not ours: refused before any write (RPM-owned: guamp-bt-ldo=3 votes through the RPM)\n",
            kBtRails[i].name, gRail[i].owner);
      return "rpm-owned";
    }
  }
  for (unsigned i = 0; i < 4; ++i) {                 // bluetooth-power.c order: io, xo, core, pa
    if (railOn(i) || gRail[i].err) continue;
    const struct bt_rail &d = kBtRails[i];
    const unsigned mv = bt_ldo_mv(gRail[i].lb, gRail[i].ub);
    int r = 0;
    if (ldo == 2 && !bt_ldo_mv_ok(&d, mv)) {
      r = pmic::write8(BT_SID_PM6125, static_cast<uint16_t>(d.base + BT_LDO_VSET_LB), static_cast<uint8_t>(d.min_mv & 0xff));
      if (!r) r = pmic::write8(BT_SID_PM6125, static_cast<uint16_t>(d.base + BT_LDO_VSET_UB), static_cast<uint8_t>(d.min_mv >> 8));
      IOLog("GuampBt: rails: %s vset %u -> %u mV rc %d\n", d.name, mv, d.min_mv, r);
    }
    if (!r) r = pmic::write8(BT_SID_PM6125, static_cast<uint16_t>(d.base + BT_LDO_ENABLE), BT_LDO_ON);
    const bool ok = !r && waitRailOk(i);
    railRead(i);
    railLog(i, "after");
    if (r) {
      IOLog("GuampBt: rails: %s write refused by the arbiter (%d, status %08x): stop (guamp-bt-ldo=3 votes through the RPM)\n", d.name, r,
            pmic::gArb.last_status);
      return "refused";
    }
    if (!ok) IOLog("GuampBt: rails: %s enabled but VREG_OK not seen in 20 ms\n", d.name);
    IOSleep(1);
  }
  return "enabled";
}

// --- step 2: SE3 UART ---------------------------------------------------------------------------------------------
void hexLine(const char *tag, const uint8_t *p, size_t n) {
  if (gHexLines >= 10) return;
  gHexLines = gHexLines + 1;
  char text[48 * 3 + 8];
  bt_hex(text, sizeof text, p, n, 48);
  IOLog("GuampBt: %s %u bytes: %s%s\n", tag, static_cast<unsigned>(n), text, gHexLines == 10 ? " (raw log limit)" : "");
}
// Non-DFS: the SE3 RCG on an ftbl entry (GPLL0_OUT_AUX2, M/N) as clk_rcg2 would; gLevelHz[0] = the rate read back.
void rcgProgram(const bt_rcg_entry &e) {
  wr(gGcc, BT_S3_RCG + BT_RCG_M, e.m);
  wr(gGcc, BT_S3_RCG + BT_RCG_N, bt_rcg_n_reg(e.m, e.n));
  wr(gGcc, BT_S3_RCG + BT_RCG_D, bt_rcg_d_reg(e.m, e.n));
  wr(gGcc, BT_S3_RCG + GT_RCG_CFG, BT_RCG_7M37_CFG);
  wr(gGcc, BT_S3_RCG + GT_RCG_CMD, rd(gGcc, BT_S3_RCG + GT_RCG_CMD) | 1u);
  for (unsigned i = 0; i < 1000 && (rd(gGcc, BT_S3_RCG + GT_RCG_CMD) & 1u); ++i) IODelay(1);
  gLevelHz[0] = gt_level_hz(rd(gGcc, BT_S3_RCG + GT_RCG_CFG), rd(gGcc, BT_S3_RCG + BT_RCG_M), rd(gGcc, BT_S3_RCG + BT_RCG_N));
}
// With DFS a level of the boot chain's table; without, the lowest ftbl entry with an exact divider (kBtRcgTable),
// reprogrammed only when the rate changes (7.3728 MHz serves 2400 and 115200, 48 MHz 3000000 at 16x).
bool setBaud(unsigned baud) {
  struct bt_uart_clock c;
  if (gDfs ? bt_choose_uart_clock(gLevelHz, GT_DFS_LEVELS, baud, gOs, &c) : bt_choose_rcg(baud, gOs, &c)) {
    IOLog("GuampBt: no clock level for %u baud\n", baud);
    return false;
  }
  const int err = bt_baud_error_permille(baud, c.baud_actual);
  const unsigned aerr = static_cast<unsigned>(err < 0 ? -err : err);
  const bool reprogram = !gDfs && aerr <= 25 && gLevelHz[0] != c.src_hz;
  if (reprogram) rcgProgram(kBtRcgTable[c.level]);
  const bool rcgOk = gDfs || gLevelHz[0] == c.src_hz;
  IOLog("GuampBt: %u baud: level %u (%llu Hz) div %u -> %u baud (%s%u.%u%%), os %u%s\n", baud, c.level,
        static_cast<unsigned long long>(c.src_hz), c.div, c.baud_actual, err < 0 ? "+" : "-", aerr / 10, aerr % 10, gOs,
        !reprogram ? "" : rcgOk ? ", RCG reprogrammed" : ", RCG read-back mismatch");
  if (aerr > 25) { IOLog("GuampBt: %u baud error above 2.5%%, not used\n", baud); return false; }
  if (!rcgOk) return false;
  return geni_uart_set_clock(&gUart, c.div, gDfs ? c.level : 0) == 0;
}
bool cbcrOn(uint32_t v) { return !(v & GT_CBCR_CLK_OFF) || (v & 2u); }
bool uartUp() {
  gStep = 2;
  gGcc = mapPhysical(GT_GCC_BASE, 0x80000);
  gPins = mapPhysical(BT_PINS_BASE, BT_PINS_SIZE);
  gSwCtrl = mapPhysical(BT_SWCTRL_BLOCK, 0x1000);
  gWrap = mapPhysical(GT_QUP_WRAPPER_BASE, 0x2000);
  gSe = mapPhysical(BT_SE_BASE, BT_SE_SIZE);
  if (!gGcc || !gPins || !gSwCtrl || !gWrap || !gSe) { IOLog("GuampBt: mapping failed\n"); return false; }
  gDfs = (rd(gGcc, BT_S3_RCG + GT_RCG_DFSR_CMD) & 1u) != 0;
  for (unsigned l = 0; l < GT_DFS_LEVELS; ++l)
    gLevelHz[l] = gDfs ? gt_level_hz(rd(gGcc, BT_S3_RCG + GT_RCG_PERF_CFG(l)), rd(gGcc, BT_S3_RCG + GT_RCG_PERF_M(l)),
                                     rd(gGcc, BT_S3_RCG + GT_RCG_PERF_N(l))) : 0;
  IOLog("GuampBt: before: gcc vote %08x s3 cbcr %08x rcg cfg %08x dfs %d level0 %llu level1 %llu | gpio8 %08x/%x gpio9 %08x/%x "
        "gpio10 %08x/%x gpio11 %08x/%x sw_ctrl gpio87 %08x/%x\n", rd(gGcc, GT_GCC_VOTE_ENABLE), rd(gGcc, BT_HALT_S3),
        rd(gGcc, BT_S3_RCG + GT_RCG_CFG), gDfs, static_cast<unsigned long long>(gLevelHz[0]),
        static_cast<unsigned long long>(gLevelHz[1]), pinRead(8, 0), pinRead(8, 4), pinRead(9, 0), pinRead(9, 4), pinRead(10, 0),
        pinRead(10, 4), pinRead(11, 0), pinRead(11, 4), rd(gSwCtrl, 0), rd(gSwCtrl, 4));
  // Clocks: GPLL0 (source of the QUP levels), the wrap0 AHB/core votes and SE3 (touch votes the same set plus S2).
  wr(gGcc, GT_GCC_GPLL0_ENABLE, rd(gGcc, GT_GCC_GPLL0_ENABLE) | 1u);
  for (unsigned i = 0; i < 10000 && !(rd(gGcc, GT_GCC_GPLL0_MODE) & (1u << 31)); ++i) IODelay(10);
  wr(gGcc, GT_GCC_VOTE_ENABLE, rd(gGcc, GT_GCC_VOTE_ENABLE) | BT_VOTE_ALL);
  bool ok = false;
  for (unsigned i = 0; i < 2000 && !ok; ++i) {
    ok = cbcrOn(rd(gGcc, GT_HALT_M_AHB)) && cbcrOn(rd(gGcc, GT_HALT_S_AHB)) && !(rd(gGcc, GT_HALT_CORE) & GT_CBCR_CLK_OFF) &&
         !(rd(gGcc, GT_HALT_CORE_2X) & GT_CBCR_CLK_OFF) && !(rd(gGcc, BT_HALT_S3) & GT_CBCR_CLK_OFF);
    if (!ok) IODelay(10);
  }
  if (!ok) { IOLog("GuampBt: SE3 clocks did not start (s3 cbcr %08x)\n", rd(gGcc, BT_HALT_S3)); return false; }
  if (!gDfs) {
    // No DFS table: program the ftbl's 7.3728 MHz entry (GPLL0_OUT_AUX2 * 384 / 15625), as clk_rcg2 would.
    rcgProgram(kBtRcgTable[0]);
    IOLog("GuampBt: SE3 RCG without DFS: programmed cfg %08x cmd %08x -> %llu Hz\n", rd(gGcc, BT_S3_RCG + GT_RCG_CFG),
          rd(gGcc, BT_S3_RCG + GT_RCG_CMD), static_cast<unsigned long long>(gLevelHz[0]));
  }
  gUart.se.ctx = const_cast<uint8_t *>(gSe); gUart.se.rd = seRd; gUart.se.wr = seWr; gUart.se.delay_us = delayUs;
  gUart.wrapper.ctx = const_cast<uint8_t *>(gWrap); gUart.wrapper.rd = seRd; gUart.wrapper.wr = seWr; gUart.wrapper.delay_us = delayUs;
  gUart.poll_us = 10;
  geni_uart_read_info(&gUart);
  const geni_uart_info &in = gUart.info;
  gOs = in.qup_hw_version >= 0x20050000u ? 16u : 32u;
  unsigned os = 0;
  if (PE_parse_boot_argn("guamp-bt-os", &os, sizeof(os)) && (os == 16 || os == 32)) gOs = os;
  IOLog("GuampBt: SE3 qup %08x fw %08x proto %u%s if_disable %08x status %08x tx depth %u rx depth %u width %u\n",
        in.qup_hw_version, in.fw_revision, in.protocol, in.protocol == GENI_PROTO_UART ? " (UART)" : " (NOT UART)", in.if_disable,
        in.geni_status, in.tx_fifo_depth, in.rx_fifo_depth, in.fifo_width_bits);
  int r = geni_uart_init(&gUart);
  if (r) { IOLog("GuampBt: UART init refused (%d)%s\n", r, r == GENI_ERR_NOTSUP ? ": FIFO interface disabled (SE DMA = later milestone)" : ""); return false; }
  // Pins after the engine holds its defaults (TX idles high): qupv3_se3 "active" state.
  for (unsigned g = BT_GPIO_CTS; g <= BT_GPIO_RX; ++g) pinWrite(g, 0, bt_pin_active_cfg(g));
  IOLog("GuampBt: pins qup3: gpio8 %08x/%x gpio9 %08x/%x gpio10 %08x/%x gpio11 %08x/%x\n", pinRead(8, 0), pinRead(8, 4),
        pinRead(9, 0), pinRead(9, 4), pinRead(10, 0), pinRead(10, 4), pinRead(11, 0), pinRead(11, 4));
  return true;
}

// --- step 3: power pulses and HCI ---------------------------------------------------------------------------------
// Receive side: gRx holds the bytes read so far, gRxPos the parse position. rxPump moves the RX FIFO in (compacting
// only once the buffer is half full, so the raw dump of a short exchange still shows every byte); nextEvent parses and
// once the M1 NVM has switched in-band sleep on, answers each IBS WAKE_IND of the controller with a WAKE_ACK (before,
// a 0xFD is just noise and 0xFC, the power-on pulse byte, is never sent).
struct RxStats { unsigned events, skipped; };
void rxPump() {
  if (gRxPos && gRxLen > sizeof gRx / 2) {
    size_t k = 0;
    for (size_t i = gRxPos; i < gRxLen; ++i) gRx[k++] = gRx[i];
    gRxLen = k;
    gRxPos = 0;
  }
  if (gRxLen == sizeof gRx) gRxLen = gRxPos = 0;     // stuck on a packet larger than the buffer: drop, resynchronise
  const size_t got = geni_uart_read(&gUart, gRx + gRxLen, sizeof gRx - gRxLen);
  for (size_t i = 0; i < got && gCapLen < sizeof gCap; ++i) gCap[gCapLen++] = gRx[gRxLen + i];
  gRxLen += got;
}
bool nextEvent(bt_event *ev, RxStats *st) {
  unsigned wake = 0, ack = 0, sleep = 0;
  const bool got = bt_h4_next_ibs_ex(gRx, gRxLen, &gRxPos, ev, &st->skipped, gIbs ? &wake : nullptr, &ack, &sleep) != 0;
  if (gIbs) {
    gIbsSleeps += sleep;
    gIbsAcksRx += ack;
    if (sleep) gAsleep = true;
    if (wake || ack || got) gAsleep = false;
  }
  for (; wake; --wake) {
    const uint8_t a = BT_IBS_WAKE_ACK;
    if (!geni_uart_write(&gUart, &a, 1, 20000)) gIbsAcks = gIbsAcks + 1;
  }
  if (got) st->events = st->events + 1;
  return got;
}
// Events until the expected one or timeoutMs. kind 0: Command Complete of op; 1: QCA version reply; 2: any vendor
// event (TLV segment ack, baud change). A Command Status with an error for op ends the wait (1). The returned event
// points into gRx (valid until the next pump). quiet: raw dump only when the wait fails.
int waitEvent(const char *what, uint16_t op, unsigned kind, unsigned timeoutMs, bt_event *out, bool quiet) {
  RxStats st = {0, 0};
  int found = GENI_ERR_TIMEOUT;
  for (unsigned t = 0; t <= timeoutMs && found == GENI_ERR_TIMEOUT; ++t) {
    rxPump();
    bt_event ev;
    while (found == GENI_ERR_TIMEOUT && nextEvent(&ev, &st)) {
      uint16_t eop = 0; const uint8_t *ret = nullptr; uint8_t rlen = 0, status = 0;
      bt_qca_version qv;
      if (kind == 0 && !bt_cmd_complete(&ev, &eop, &ret, &rlen) && eop == op) { *out = ev; found = 0; }
      else if (kind == 1 && !bt_decode_qca_version(&ev, &qv)) { *out = ev; found = 0; }
      else if (kind == 2 && ev.code == BT_EVT_VENDOR) { *out = ev; found = 0; }
      else if (!bt_cmd_status(&ev, &eop, &status) && eop == op && status) {
        IOLog("GuampBt: %s: Command Status %02x\n", what, status);
        *out = ev; found = 1;
      }
    }
    if (found == GENI_ERR_TIMEOUT) IOSleep(1);
  }
  if (!quiet || found) hexLine(what, gRx, gRxLen);
  if (found == GENI_ERR_TIMEOUT) {
    IOLog("GuampBt: %s: no reply in %u ms (%u bytes, %u events, %u bytes skipped, dropped %u, CTS gpio8 level %u)\n", what,
          timeoutMs, static_cast<unsigned>(gRxLen), st.events, st.skipped, gUart.rx_dropped, pinRead(8, 4) & 1u);
    char text[32 * 3 + 8];                             // what came in after the command (not limited like hexLine)
    bt_hex(text, sizeof text, gCap, gCapLen, 32);
    IOLog("GuampBt: %s: received after the command %u bytes: %s\n", what, static_cast<unsigned>(gCapLen),
          gCapLen ? text : "none");
    if (bt_seven_bit_event(gCap, gCapLen))
      IOLog("GuampBt: %s: the answer has bit 7 set on every byte: the controller sends 7-bit characters (NVM HCI tag "
            "data[0]: try guamp-bt-nvm-ibs=0)\n", what);
  }
  return found;
}
// IBS wake (hci_qca.c: WAKE_IND, the controller answers WAKE_ACK). Waits up to 30 ms for the ack without consuming
// events (only peeks at the skipped bytes before the next one). True when acked.
bool wake(const char *what) {
  const uint8_t ind = BT_IBS_WAKE_IND;
  const int tx = geni_uart_write(&gUart, &ind, 1, 20000);
  gIbsWakeSent = gIbsWakeSent + 1;
  unsigned ms = 0, ack = 0;
  for (; !tx && ms < 30 && !ack; ++ms) {
    rxPump();
    size_t p = gRxPos;
    unsigned sk = 0;
    bt_event e;
    bt_h4_next_ibs_ex(gRx, gRxLen, &p, &e, &sk, nullptr, &ack, nullptr);
    if (!ack) IOSleep(1);
  }
  IOLog("GuampBt: %s: WAKE_IND tx %d, WAKE_ACK %s in %u ms\n", what, tx, ack ? "seen" : "none", ms);
  if (ack) gAsleep = false;
  return ack != 0;
}
int sendPacket(const char *what, const uint8_t *pkt, size_t n) {
  const int r = geni_uart_write(&gUart, pkt, n, 200000);
  if (r) IOLog("GuampBt: %s: TX failed %d (m_irq %08x, CTS gpio8 level %u)\n", what, r, gUart.last_m_irq, pinRead(8, 4) & 1u);
  return r;
}
// One command, then waitEvent. fresh: forget the bytes received before (M0); M1 keeps them (an event or a WAKE_IND
// may straddle two exchanges).
int command(const char *what, uint16_t op, const uint8_t *params, uint8_t plen, unsigned kind, unsigned timeoutMs, bt_event *out,
            bool fresh = true, bool quiet = false) {
  uint8_t pkt[16];
  const size_t n = bt_h4_cmd(pkt, sizeof pkt, op, params, plen);
  if (fresh) gRxLen = gRxPos = 0;
  if (gIbs && gAsleep) wake(what);
  gCapLen = 0;
  const int r = sendPacket(what, pkt, n);
  return r ? r : waitEvent(what, op, kind, timeoutMs, out, quiet);
}

int pulse(uint8_t byte, unsigned baud) {
  geni_uart_flow(&gUart, 0);                         // flow control off for the pulses (hci_qca.c)
  if (!setBaud(baud)) return GENI_ERR_PARAM;
  const int r = geni_uart_write(&gUart, &byte, 1, 50000);
  IODelay(200);                                      // "Wait for 100 uS for SoC to settle down"
  return r;
}

const char *gReset = "-", *gVersion = "-", *gQca = "-";
bt_qca_version gQv;
bool gQvOk;
void hci() {
  gStep = 3;
  unsigned pulses = 1;
  PE_parse_boot_argn("guamp-bt-pulse", &pulses, sizeof(pulses));
  if (pulses) {
    const int off = pulse(BT_QCA_POWEROFF_PULSE, 2400);
    const int on = pulse(BT_QCA_POWERON_PULSE, 115200);
    IOLog("GuampBt: power pulses: off c0 @2400 rc %d, on fc @115200 rc %d, sw_ctrl gpio87 %x\n", off, on, rd(gSwCtrl, 4) & 1u);
    IOSleep(100);                                    // "Wait for 100 ms for SoC to boot"
  }
  // hci_qca.c re-opens the port here to resynchronise RTS/CTS: engine defaults again, 115200, hardware flow control.
  if (geni_uart_init(&gUart) || !setBaud(115200)) { gReset = "uart"; return; }
  geni_uart_flow(&gUart, 1);
  const int rs = geni_uart_rx_start(&gUart);
  IOSleep(20);
  gRxLen = geni_uart_read(&gUart, gRx, sizeof gRx);
  IOLog("GuampBt: uart open 115200 8N1 RTS/CTS, rx start %d, status %08x, CTS gpio8 level %u, sw_ctrl %x, %u bytes pending\n", rs,
        rd(gSe, SE_GENI_STATUS), pinRead(8, 4) & 1u, rd(gSwCtrl, 4) & 1u, static_cast<unsigned>(gRxLen));
  if (gRxLen) hexLine("pending", gRx, gRxLen);

  bt_event ev;
  int r = command("HCI_Reset", BT_OP_RESET, nullptr, 0, 0, 2000, &ev);
  if (r) { gReset = r == GENI_ERR_TIMEOUT ? "timeout" : r < 0 ? "tx-fail" : "status"; return; }
  IOLog("GuampBt: HCI_Reset status %02x\n", ev.len > 3 ? ev.p[3] : 0xff);
  gReset = ev.len > 3 && !ev.p[3] ? "ok" : "error";

  r = command("Read_Local_Version", BT_OP_READ_LOCAL_VERSION, nullptr, 0, 0, 1000, &ev);
  bt_local_version lv;
  if (!r && !bt_decode_local_version(&ev, &lv)) {
    IOLog("GuampBt: local version HCI %s (%u) rev %04x LMP %u subver %04x manufacturer %04x (%s)\n", bt_hci_version_name(lv.hci_ver),
          lv.hci_ver, lv.hci_rev, lv.lmp_ver, lv.lmp_subver, lv.manufacturer, bt_manufacturer_name(lv.manufacturer));
    gVersion = "ok";
  } else {
    gVersion = r == GENI_ERR_TIMEOUT ? "timeout" : "error";
  }

  const uint8_t sub = BT_QCA_EDL_VER_REQ;
  r = command("QCA_EDL_version", BT_OP_QCA_EDL, &sub, 1, 1, 2000, &ev);
  bt_qca_version qv;
  if (r || bt_decode_qca_version(&ev, &qv)) { gQca = r == GENI_ERR_TIMEOUT ? "timeout" : "error"; return; }
  IOLog("GuampBt: QCA product %08x patch %04x rom %04x soc %08x -> soc_ver %08x rom %02x (reply form %u)\n", qv.product_id,
        qv.patch_ver, qv.rom_ver, qv.soc_id, qv.soc_ver, qv.rom, qv.form);
  gQv = qv;
  gQvOk = true;
  char tlv[16], nvm[16], match[160];
  bt_qca_names(qv.rom, tlv, nvm);
  const unsigned m = bt_fw_matches(&qv);
  unsigned at = 0;
  for (unsigned k = 0; k < sizeof kBtFirmware / sizeof kBtFirmware[0]; ++k) {
    if (!(m & (1u << k))) continue;
    const char *parts[4] = {kBtFirmware[k].tlv, "+", kBtFirmware[k].nvm, " "};
    for (const char *s : parts)
      for (; *s && at + 1 < sizeof match; ++s) match[at++] = *s;
  }
  match[at] = 0;
  IOLog("GuampBt: firmware: btqca name %s + %s; bluetooth_a header match (product %04x rom %04x): %s\n", tlv, nvm,
        static_cast<unsigned>(qv.product_id & 0xffff), qv.rom_ver, at ? match : "none");
  gQca = at ? "ok" : "no-fw";
}

// --- step 4 (guamp-bt=2): rampatch + NVM download at the operating speed ----------------------------------------
constexpr uint32_t kFwMax = 512 * 1024;                  // largest file of bluetooth_a: 229868 bytes
struct FwWindow { uint64_t first, blocks; };
struct FwFile { uint8_t *data; uint32_t size; };
bool fwRead(void *ctx, uint64_t sector, uint32_t count, uint8_t *out) {
  const FwWindow *w = static_cast<const FwWindow *>(ctx);
  if (!count || count > kBounceBlocks || sector >= w->blocks || count > w->blocks - sector) return false;
  Card &card = gCard;
  IOLockLock(card.gate);
  const bool ok = card.online && card.readBlocks(w->first + sector, count, out);
  IOLockUnlock(card.gate);
  return ok;
}
void fwFree(FwFile *f) {
  if (f->data) IOFree(f->data, f->size);
  f->data = nullptr;
  f->size = 0;
}
// /image/<name> into a buffer of its size. 0 or an ext4 error (kErrRange: empty, not a file or above kFwMax).
int fwLoad(ext4::Fs *fs, const char *name, FwFile *out) {
  char path[48] = {'/', 'i', 'm', 'a', 'g', 'e', '/'};
  unsigned at = 7;
  for (const char *s = name; *s && at + 1 < sizeof path; ++s) path[at++] = *s;
  path[at] = 0;
  ext4::Inode ino;
  int r = fs->lookupPath(path, &ino);
  if (r) return r;
  if (!ino.isFile() || !ino.size || ino.size > kFwMax) return ext4::kErrRange;
  const uint32_t size = static_cast<uint32_t>(ino.size);
  uint8_t *p = static_cast<uint8_t *>(IOMallocZero(size));
  if (!p) return ext4::kErrIo;
  r = fs->readFile(ino, 0, p, size);
  if (r) { IOFree(p, size); return r; }
  out->data = p;
  out->size = size;
  return 0;
}
// Both files from the bluetooth_<slot> partition, read only; one log line.
bool fwLoadBoth(const char *tlv, const char *nvm, FwFile *patch, FwFile *cfg) {
  char arg[4] = {};
  const char slot = PE_parse_boot_argn("guamp-bt-slot", arg, sizeof(arg)) && (arg[0] == 'a' || arg[0] == 'b') ? arg[0]
                                                                                                             : wifiprep::activeSlot();
  char part[12] = {'b', 'l', 'u', 'e', 't', 'o', 'o', 't', 'h', '_', slot, 0};
  FwWindow w = {0, 0};
  uint64_t attr = 0;
  if (!gCard.online || !gCard.gate || !wifiprep::findPartition(part, &w.first, &w.blocks, &attr)) {
    IOLog("GuampBt: fw: partition %s not found (eMMC %s)\n", part, gCard.online ? "online" : "offline");
    return false;
  }
  uint8_t *scratch = static_cast<uint8_t *>(IOMallocZero(ext4::kScratchBytes));
  ext4::Fs *fs = static_cast<ext4::Fs *>(IOMallocZero(sizeof(ext4::Fs)));
  int m = ext4::kErrIo, a = ext4::kErrIo, b = ext4::kErrIo;
  if (scratch && fs) {
    m = fs->mount(fwRead, &w, w.blocks, scratch);
    if (!m) a = fwLoad(fs, tlv, patch);
    if (!m) b = fwLoad(fs, nvm, cfg);
  }
  IOLog("GuampBt: fw: %s at LBA %llu (%llu sectors): ext4 %s, /image/%s %u bytes %s, /image/%s %u bytes %s\n", part,
        static_cast<unsigned long long>(w.first), static_cast<unsigned long long>(w.blocks),
        scratch && fs ? ext4::errorName(m) : "out of memory", tlv, patch->size, m ? "-" : a ? ext4::errorName(a) : "read",
        nvm, cfg->size, m ? "-" : b ? ext4::errorName(b) : "read");
  if (scratch) IOFree(scratch, ext4::kScratchBytes);
  if (fs) IOFree(fs, sizeof(ext4::Fs));
  return !m && !a && !b;
}

// hci_qca.c qca_set_speed(QCA_OPER_SPEED), WCN399x branch: flow control off (RTS deasserted, the controller holds its
// reply), VSC 0xFC48 {code}, 1-10 ms for the controller, host to the new rate, flow control on. Then the controller's
// vendor event at the new rate (100 ms, logged only: Linux drops it) and Read_Local_Version as the proof.
bool switchBaud(unsigned baud, uint8_t code) {
  uint8_t pkt[8];
  const size_t n = bt_h4_cmd(pkt, sizeof pkt, BT_OP_QCA_SET_BAUD, &code, 1);
  rxPump();
  geni_uart_flow(&gUart, 0);
  const int tx = sendPacket("QCA_set_baud", pkt, n);
  IOSleep(5);
  rxPump();
  const int stop = geni_uart_rx_stop(&gUart, 2000);
  const bool host = !tx && setBaud(baud);
  const int start = geni_uart_rx_start(&gUart);
  geni_uart_flow(&gUart, 1);
  RxStats st = {0, 0};
  uint8_t vlen = 0, v0 = 0, v1 = 0;
  bool vendor = false;
  for (unsigned t = 0; host && t < 100 && !vendor; ++t) {
    rxPump();
    bt_event ev;
    while (!vendor && nextEvent(&ev, &st))
      if (ev.code == BT_EVT_VENDOR) { vendor = true; vlen = ev.len; v0 = ev.len > 0 ? ev.p[0] : 0; v1 = ev.len > 1 ? ev.p[1] : 0; }
    if (!vendor) IOSleep(1);
  }
  bt_event ev;
  bt_local_version lv;
  const int r = host ? command("Read_Local_Version@baud", BT_OP_READ_LOCAL_VERSION, nullptr, 0, 0, 500, &ev, false, true) : -1;
  const bool ok = !r && !bt_decode_local_version(&ev, &lv);
  IOLog("GuampBt: baud %u: VSC fc48 %02x tx %d, rx stop %d start %d, host %s, vendor event %s (len %u: %02x %02x), "
        "Read_Local_Version %s at the new rate\n", baud, code, tx, stop, start, host ? "switched" : "NOT switched",
        vendor ? "seen" : "none in 100 ms", vlen, v0, v1, ok ? "ok" : r == GENI_ERR_TIMEOUT ? "timeout" : "error");
  return ok;
}

// qca_download_firmware: the whole file in 243-byte VSC 0xFC00/0x1E segments, sent back to back (UART flow control
// only) except where the TLV download mode wants an ack: then the vendor event must be {00 04 result} within 5 s
// (Linux: wrong size/type fails, a non-zero result is only logged). Events arriving between segments are consumed.
uint8_t gSeg[BT_QCA_SEG_PKT_MAX];
int download(const char *what, const char *name, const FwFile &f, unsigned mode) {
  const unsigned segs = bt_tlv_segments(f.size);
  unsigned sent = 0, acked = 0, nonzero = 0;
  uint8_t lastNonzero = 0;
  RxStats st = {0, 0};
  int err = 0;
  const uint64_t began = mach_absolute_time();
  for (unsigned i = 0; i < segs && !err; ++i) {
    int ack = 0;
    const size_t n = bt_tlv_segment(f.data, f.size, i, mode, gSeg, sizeof gSeg, &ack);
    if (!n) { err = -1; break; }
    if (sendPacket(what, gSeg, n)) { err = -2; break; }
    sent = sent + 1;
    bt_event ev;
    if (!ack) {
      rxPump();
      while (nextEvent(&ev, &st)) {}
      continue;
    }
    const int w = waitEvent(what, BT_OP_QCA_EDL, 2, 5000, &ev, true);
    uint8_t res = 0;
    if (w) {
      err = -3;
    } else if (bt_tlv_ack(&ev, &res)) {
      hexLine("bad TLV ack", ev.p, ev.len);
      err = -4;
    } else {
      acked = acked + 1;
      if (res) { nonzero = nonzero + 1; lastNonzero = res; }
    }
  }
  uint64_t ns = 0;
  absolutetime_to_nanoseconds(mach_absolute_time() - began, &ns);
  IOLog("GuampBt: %s %s: %u bytes, %u/%u segments sent, %u acked (%u with result != 0, last %02x), mode %u, %u other events, "
        "%llu ms -> %s\n", what, name, f.size, sent, segs, acked, nonzero, lastNonzero, mode, st.events,
        static_cast<unsigned long long>(ns / 1000000), err == 0 ? "ok" : err == -2 ? "TX failed" : err == -3 ? "no ack" :
        err == -4 ? "bad ack" : "error");
  return err;
}

const char *gDownload = "-", *gBaud = "-", *gAfter = "-", *gBdaddr = "-";
unsigned gBaudRate = 115200;
bool gNvmIbs = true;                                  // in-band sleep on in the NVM as sent (HCI tag data[0] bit 7)

// Host side only: RX stopped, flow control off, new rate, RX on again (as in switchBaud), buffer forgotten.
bool hostRate(unsigned baud) {
  rxPump();
  geni_uart_flow(&gUart, 0);
  const int stop = geni_uart_rx_stop(&gUart, 2000);
  const bool ok = setBaud(baud);
  const int start = geni_uart_rx_start(&gUart);
  geni_uart_flow(&gUart, 1);
  gRxLen = gRxPos = 0;
  if (stop || start) IOLog("GuampBt: host %u baud: rx stop %d start %d\n", baud, stop, start);
  return ok && !start;
}

// The patched controller did not answer HCI_Reset at the download rate. Linux gets the Command Complete right away
// (qca_uart_setup -> qca_send_reset, IBS still off on the host). Two causes are checked, each logged:
//  - IBS: the NVM switched in-band sleep on; a WAKE_IND first, then HCI_Reset again at the same rate.
//  - rate: the HCI tag's baud byte is applied at reset. Which byte the WCN3950 reads is not certain: btqca.c writes
//    data[2] before WCN3991, but this NVM ships 15 11 22 (0x11 = 3200000, 0x22 no rate code). The rates of every
//    HCI tag byte (as sent and as shipped) and 115200 are tried on the host side, HCI_Reset at each.
// 0 when a reset was answered (gBaudRate then is the working rate), else GENI_ERR_TIMEOUT with the host back at rate.
int resetRecover(const bt_tlv_nvm &nv, unsigned rate, bt_event *ev) {
  gAsleep = true;                                      // forces the WAKE_IND in command()
  int r = command("HCI_Reset after wake", BT_OP_RESET, nullptr, 0, 0, 500, ev, false, true);
  IOLog("GuampBt: recovery: WAKE_IND + HCI_Reset at %u: %s\n", rate, r ? "no reply" : "answered");
  if (!r) return 0;
  unsigned rates[6], n = 0;
  const uint8_t codes[4] = {nv.hci_after[1], nv.hci_after[2], nv.hci_before[1], nv.hci_before[2]};
  for (unsigned k = 0; k < 5; ++k) {
    const unsigned b = k < 4 ? bt_qca_baud_of_code(codes[k]) : 115200u;
    bool dup = !b || b == rate;
    for (unsigned j = 0; j < n; ++j) dup = dup || rates[j] == b;
    if (!dup) rates[n++] = b;
  }
  for (unsigned i = 0; i < n; ++i) {
    if (!hostRate(rates[i])) { IOLog("GuampBt: recovery: host cannot run %u baud\n", rates[i]); continue; }
    gAsleep = true;
    r = command("HCI_Reset at another rate", BT_OP_RESET, nullptr, 0, 0, 500, ev, false, true);
    IOLog("GuampBt: recovery: HCI_Reset at %u (host): %s\n", rates[i], r ? "no reply" : "ANSWERED, the controller moved here");
    if (!r) { gBaudRate = rates[i]; gBaud = "moved"; return 0; }
  }
  hostRate(rate);
  return GENI_ERR_TIMEOUT;
}
char gAfterText[48] = "-";
// The M1 sequence (btqca.c qca_uart_setup after qca_set_speed). True when the patched controller answered HCI_Reset.
bool m1() {
  gStep = 4;
  if (!gQvOk) { gDownload = "skipped"; IOLog("GuampBt: M1 skipped: no QCA version\n"); return false; }
  char tlv[20], nvm[20];
  const unsigned family = bt_qca_fw_names(&gQv, tlv, nvm);
  const bool newLayout = family == 3990 && gQv.soc_id == BT_QCA_SOC_WCN3991;   // soc_type >= QCA_WCN3991
  unsigned baud = 3000000;
  PE_parse_boot_argn("guamp-bt-baud", &baud, sizeof(baud));
  uint8_t code = bt_qca_baud_code(baud);
  if (code == 0xff) {
    IOLog("GuampBt: guamp-bt-baud=%u is not a QCA rate, 115200 kept\n", baud);
    baud = 115200;
    code = 0;
  }
  gBaudRate = baud;
  FwFile patch = {nullptr, 0}, cfg = {nullptr, 0};
  if (!fwLoadBoth(tlv, nvm, &patch, &cfg)) {
    fwFree(&patch); fwFree(&cfg);
    gDownload = "no-file";
    return false;
  }
  bt_tlv_patch ph = {};
  const int pr = bt_tlv_patch_parse(patch.data, patch.size, &ph);
  const bool match = !pr && ph.product == (gQv.product_id & 0xffff) && ph.rom_build == gQv.rom_ver;
  bt_tlv_nvm nv = {};
  // guamp-bt-nvm-baud: HCI tag bytes that get the baud code, 0 none, 1 data[1], 2 data[2] (btqca.c for WCN3950), 3
  // both; absent: bt_tlv_nvm_baud_fields_auto (the Linux byte only if it holds a rate code: none on the guamp NVM).
  // guamp-bt-nvm-ibs: 1 = Linux's data[0] |= 0x80 (in-band sleep); default 0 on the WCN3950 family (product 0x0f):
  // with it the patched controller came back from HCI_Reset sending 7-bit characters and ignored every later command.
  bt_tlv_nvm probe = {};
  bt_tlv_nvm_fix_ex(cfg.data, cfg.size, code, 0u, 0, &probe);   // reads the tags; only the idempotent deep-sleep edit
  unsigned fields = bt_tlv_nvm_baud_fields_auto(probe.hci_before, newLayout);
  if (PE_parse_boot_argn("guamp-bt-nvm-baud", &fields, sizeof(fields)) && fields > 3) fields = 0;
  unsigned ibs = family == 3950 ? 0u : 1u;
  PE_parse_boot_argn("guamp-bt-nvm-ibs", &ibs, sizeof(ibs));
  const int nr = bt_tlv_nvm_fix_ex(cfg.data, cfg.size, code, fields, ibs ? 1 : 0, &nv);
  nv.sleep_before = probe.sleep_before;
  gNvmIbs = ibs != 0 || (nv.hci_after[0] & 0x80) != 0;
  IOLog("GuampBt: fw: WCN%u %s patch %04x product %04x rom %04x mode %u (%u segments)%s; %s type %u, %u tags, hci %02x %02x %02x -> "
        "%02x %02x %02x, deep sleep %02x -> %02x, BD tag %s%s\n", family, tlv, ph.patch_ver, ph.product, ph.rom_build, ph.mode,
        bt_tlv_segments(patch.size), pr ? " NOT A TLV PATCH" : match ? "" : " NOT FOR THIS CONTROLLER", nvm, nv.type, nv.tags,
        nv.hci_before[0], nv.hci_before[1], nv.hci_before[2], nv.hci_after[0], nv.hci_after[1], nv.hci_after[2], nv.sleep_before,
        nv.sleep_after, nv.has_bdaddr ? (bt_bd_addr_zero(nv.bdaddr) ? "zero" : "set") : "absent", nr ? ", NVM MALFORMED" : "");
  if (!match || nr || newLayout) {
    if (newLayout) IOLog("GuampBt: WCN3991 command-complete acks not implemented: no download\n");
    fwFree(&patch); fwFree(&cfg);
    gDownload = newLayout ? "unsupported" : "bad-file";
    return false;
  }
  if (baud != 115200) {
    const bool ok = switchBaud(baud, code);
    gBaud = ok ? "ok" : "fail";
    if (!ok) { fwFree(&patch); fwFree(&cfg); gDownload = "skipped"; return false; }
  } else {
    gBaud = "unchanged";
  }
  int e = download("patch", tlv, patch, ph.mode);
  if (!e) {
    IOSleep(10);                                           // "Give the controller some time to get ready to receive the NVM"
    e = download("nvm", nvm, cfg, BT_QCA_SKIP_NONE);
    gDownload = e ? "fail-nvm" : "ok";
    gIbs = !e && gNvmIbs;
  } else {
    gDownload = "fail-patch";
  }
  fwFree(&patch); fwFree(&cfg);
  if (e) return false;

  bt_event ev;
  int r = command("HCI_Reset after", BT_OP_RESET, nullptr, 0, 0, 2000, &ev, false, true);
  if (r == GENI_ERR_TIMEOUT) r = resetRecover(nv, baud, &ev);
  const uint8_t rst = !r && ev.len > 3 ? ev.p[3] : 0xff;
  bt_local_version lv = {};
  const int lr = r ? -1 : command("Read_Local_Version after", BT_OP_READ_LOCAL_VERSION, nullptr, 0, 0, 1000, &ev, false, true);
  const bool lvOk = !lr && !bt_decode_local_version(&ev, &lv);
  IOLog("GuampBt: after download: HCI_Reset %s (status %02x) at %u, local version %s HCI %s (%u) rev %04x LMP %u subver %04x "
        "manufacturer %04x; NVM baud field %u, NVM IBS %u, IBS sleep %u wake-ack rx %u wake sent %u\n", r ? "no reply" : rst ? "error" : "ok",
        rst, gBaudRate, lvOk ? "ok" : "FAILED", bt_hci_version_name(lv.hci_ver), lv.hci_ver, lv.hci_rev, lv.lmp_ver,
        lv.lmp_subver, lv.manufacturer, fields, gNvmIbs ? 1u : 0u, gIbsSleeps, gIbsAcksRx, gIbsWakeSent);
  if (r || rst) return false;

  const uint8_t sub = BT_QCA_EDL_VER_REQ;
  bt_qca_version qa;
  const int qr = command("QCA_EDL_version after", BT_OP_QCA_EDL, &sub, 1, 1, 2000, &ev, false, true);
  if (!qr && !bt_decode_qca_version(&ev, &qa)) {
    static const char hx[] = "0123456789abcdef";
    const uint16_t v[2] = {qa.patch_ver, qa.rom_ver};
    const char *label[2] = {"patch ", " rom "};
    unsigned at = 0;
    for (unsigned k = 0; k < 2; ++k) {
      for (const char *s = label[k]; *s; ++s) gAfterText[at++] = *s;
      for (int sh = 12; sh >= 0; sh -= 4) gAfterText[at++] = hx[(v[k] >> sh) & 15];
    }
    gAfterText[at] = 0;
    gAfter = gAfterText;
    IOLog("GuampBt: after download: QCA product %08x patch %04x rom %04x soc %08x (TLV patch %04x: %s), IBS wake acks %u\n",
          qa.product_id, qa.patch_ver, qa.rom_ver, qa.soc_id, ph.patch_ver, qa.patch_ver == ph.patch_ver ? "running" : "differs",
          gIbsAcks);
  } else {
    gAfter = qr == GENI_ERR_TIMEOUT ? "timeout" : "error";
  }

  // qca_check_bdaddr: an address equal to the NVM's tag 2 is a placeholder (Linux then needs local-bd-address from the
  // device tree, HCI_QUIRK_USE_BDADDR_PROPERTY). Nothing is written here.
  if (gHaveAddr) {
    uint8_t params[6];
    bt_qca_bdaddr_params(gAddr, params);
    const int wres = command("EDL_WRITE_BD_ADDR", BT_OP_QCA_WRITE_BD_ADDR, params, 6, 0, 1000, &ev, false, true);
    const uint8_t ws = !wres && ev.len > 3 ? ev.p[3] : 0xff;
    char masked[18];
    bt_bd_addr_masked(gAddr, masked);
    IOLog("GuampBt: EDL_WRITE_BD_ADDR %s: %s (status %02x)\n", masked, wres ? "no reply" : ws ? "error" : "ok", ws);
  }
  uint8_t addr[6] = {}, status = 0xff;
  const int br = command("Read_BD_ADDR", BT_OP_READ_BD_ADDR, nullptr, 0, 0, 1000, &ev, false, true);
  const int bd = br ? -1 : bt_decode_bd_addr(&ev, addr, &status);
  char text[18];
  bt_bd_addr_masked(addr, text);   // never the whole address in the log
  if (bd) {
    gBdaddr = br == GENI_ERR_TIMEOUT ? "timeout" : "error";
    IOLog("GuampBt: Read_BD_ADDR %s (status %02x)\n", gBdaddr, status);
  } else if (bt_bd_addr_zero(addr)) {
    gBdaddr = "zero";
    IOLog("GuampBt: BD address %s: not set, NVM/BD address set needed (Linux: local-bd-address from DT; none invented)\n", text);
  } else if (nv.has_bdaddr && bt_bd_addr_eq(addr, nv.bdaddr)) {
    gBdaddr = "nvm-default";
    IOLog("GuampBt: BD address %s = NVM default: placeholder, BD address set needed (Linux: local-bd-address from DT; none "
          "invented)\n", text);
  } else if (gHaveAddr && bt_bd_addr_eq(addr, gAddr)) {
    gBdaddr = "guamp-bt-addr";
    IOLog("GuampBt: BD address %s = guamp-bt-addr\n", text);
  } else {
    gBdaddr = "set";
    IOLog("GuampBt: BD address %s (not the NVM default)%s\n", text, gHaveAddr ? ", NOT guamp-bt-addr" : "");
  }
  return true;
}

// --- step 5: transport (guamp-bt=2 guamp-bt-shim=1, after M1) ---------------------------------------------------
// The controller becomes the H4 transport of the Bluetooth M2 shim (GuampBtSerial.inc / GuampBtShim.h): this thread
// moves every UART byte to btshim::controllerBytes, link.tx (bluetoothd's write, through the shim) sends whole H4
// packets. gUartLock serializes the UART between the two; bytes read while link.tx waits for an IBS WAKE_ACK go to
// gPend and reach the shim from this thread (the shim lock is held around link.tx). The IBS bytes between packets are
// tracked with bt_ibs_scan: each controller WAKE_IND gets a WAKE_ACK, a SLEEP_IND marks it asleep until it speaks.
// BlueTool's /dev/btreset (release after assert) and /dev/btpoweroff (on again) re-run M0+M1 here, since a real reset
// drops the rampatch (guamp-bt-shim-reset=0: ignored); /dev/btpoweroff sends the power-off pulse.
IOLock *gUartLock;
uint8_t gPend[2048];
size_t gPendLen;
bt_ibs_scan gScan;
volatile bool gReinit, gPowerOff, gResetHeld, gOff;
uint32_t gTxPkts, gTxErr, gRxBytes, gReinits;

// Under gUartLock: new UART bytes into dst (cap), IBS bookkeeping (WAKE_ACK answers). Returns the bytes read.
size_t transportRead(uint8_t *dst, size_t cap) {
  const size_t n = geni_uart_read(&gUart, dst, cap);
  if (!n) return 0;
  unsigned wakes = bt_ibs_scan_feed(&gScan, dst, n);
  for (; wakes; --wakes) {
    const uint8_t a = BT_IBS_WAKE_ACK;
    if (!geni_uart_write(&gUart, &a, 1, 20000)) gIbsAcks = gIbsAcks + 1;
  }
  // In stream order: an event followed by SLEEP_IND in the same read leaves the controller asleep (not awake).
  if (gScan.state) gAsleep = gScan.state == 2;
  gRxBytes += static_cast<uint32_t>(n);
  return n;
}

int linkTx(void *, const uint8_t *pkt, size_t len) {
  if (gOff) return -1;
  IOLockLock(gUartLock);
  if (gIbs && gAsleep) {
    const uint8_t ind = BT_IBS_WAKE_IND;
    geni_uart_write(&gUart, &ind, 1, 20000);
    gIbsWakeSent = gIbsWakeSent + 1;
    for (unsigned ms = 0; ms < 30 && gAsleep; ++ms) {
      const uint32_t acks = gScan.ack;
      if (gPendLen < sizeof gPend) gPendLen += transportRead(gPend + gPendLen, sizeof gPend - gPendLen);
      if (gScan.ack != acks) gAsleep = false;
      if (gAsleep) IOSleep(1);
    }
  }
  const int r = geni_uart_write(&gUart, pkt, len, 200000);
  IOLockUnlock(gUartLock);
  gTxPkts = gTxPkts + 1;
  if (r) gTxErr = gTxErr + 1;
  return r ? -1 : 0;
}
bool gResetEnabled = true;
void linkReset(void *, bool asserted) {
  if (asserted) { gResetHeld = true; return; }
  if (gResetHeld && gResetEnabled) gReinit = true;
  gResetHeld = false;
}
void linkPower(void *, bool off) {
  if (off) gPowerOff = true;
  else if (gOff && gResetEnabled) gReinit = true;
}

void transport() {
  gUartLock = IOLockAlloc();
  if (!gUartLock) { IOLog("GuampBt: transport: no lock\n"); return; }
  unsigned rs = 1;
  if (PE_parse_boot_argn("guamp-bt-shim-reset", &rs, sizeof(rs))) gResetEnabled = rs != 0;
  bts_link link = {};
  link.tx = linkTx;
  link.reset = linkReset;
  link.power = linkPower;
  btshim::attach(&link);
  IOLog("GuampBt: transport on at %u baud (IBS %u), reset re-download %u\n", gBaudRate, gIbs ? 1u : 0u,
        gResetEnabled ? 1u : 0u);
  static uint8_t buf[2048];
  uint64_t lastLog = 0;
  for (;;) {
    if (gPowerOff) {
      IOLockLock(gUartLock);
      gPowerOff = false;
      geni_uart_rx_stop(&gUart, 2000);
      const int r = pulse(BT_QCA_POWEROFF_PULSE, 2400);
      gOff = true;
      IOLockUnlock(gUartLock);
      btshim::controllerReset();   // a packet cut by the power-off must not prefix the next one
      IOLog("GuampBt: transport: power off (btpoweroff) pulse rc %d\n", r);
    }
    if (gReinit) {
      IOLockLock(gUartLock);
      gReinit = false;
      gRxLen = gRxPos = 0;
      gPendLen = 0;
      gIbs = false;
      gAsleep = false;
      gScan = bt_ibs_scan{};
      gReinits = gReinits + 1;
      hci();
      const bool ok = m1();
      gOff = !ok;
      IOLockUnlock(gUartLock);
      btshim::controllerReset();   // bytes M1 consumed never reached the shim: drop its half packet
      IOLog("GuampBt: transport: controller re-initialised (%u): M1 %s, BD address %s\n", gReinits, ok ? "ok" : "FAILED",
            gBdaddr);
    }
    size_t n = 0;
    if (!gOff) {
      IOLockLock(gUartLock);
      if (gPendLen) {
        for (size_t i = 0; i < gPendLen; ++i) buf[i] = gPend[i];
        n = gPendLen;
        gPendLen = 0;
      }
      n += transportRead(buf + n, sizeof buf - n);
      IOLockUnlock(gUartLock);
    }
    if (n) btshim::controllerBytes(buf, n);
    else IOSleep(2);
    uint64_t now = 0;
    absolutetime_to_nanoseconds(mach_absolute_time(), &now);
    if (now - lastLog > 60000000000ULL) {
      lastLog = now;
      IOLog("GuampBt: transport: tx %u packets (%u errors), rx %u bytes (%u packets, %u noise), IBS wake rx %u ack tx %u "
            "sleep rx %u wake tx %u, %s, re-inits %u; shim nodes %u opens %u ioctls %u reads %u writes %u, btreset %u btwake %u "
            "btpoweroff %u opens\n", gTxPkts, gTxErr, gRxBytes, gScan.packets, gScan.noise, gScan.wake,
            gIbsAcks, gScan.sleep, gIbsWakeSent, gOff ? "off" : gAsleep ? "asleep" : "awake", gReinits, btshim::gNodes,
            btshim::gOpens, btshim::gIoctls, btshim::gReads, btshim::gWrites, btshim::gCtlOpens[0], btshim::gCtlOpens[1],
            btshim::gCtlOpens[2]);
    }
  }
}

void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-bt");
  btshim::start();                                     // guamp-bt-shim=1: /dev/cu.bluetooth & co. (devfs is up by now)
  unsigned mode = 1, delay = 0;
  PE_parse_boot_argn("guamp-bt", &mode, sizeof(mode));
  PE_parse_boot_argn("guamp-bt-delay", &delay, sizeof(delay));
  if (delay && delay <= 600) IOSleep(delay * 1000);
  const char *railState = rails();
  unsigned uart = 1, keep = 0;
  PE_parse_boot_argn("guamp-bt-uart", &uart, sizeof(uart));
  const bool keepSet = PE_parse_boot_argn("guamp-bt-keep", &keep, sizeof(keep));
  const char *uartState = "skipped";
  bool on = false;
  if (uart) {
    if (!uartUp()) {
      uartState = "fail";
    } else {
      uartState = "up";
      hci();
      const bool m1ok = mode == 2 && m1();
      if (m1ok && btshim::gEnabled) {
        IOLog("GuampBt: RESULT rails %s uart %s reset %s version %s qca %s download %s baud %u %s version after %s bdaddr %s "
              "controller on (transport)\n", railState, uartState, gReset, gVersion, gQca, gDownload, gBaudRate, gBaud, gAfter,
              gBdaddr);
        transport();                                     // does not return while the transport runs
        return;
      }
      if (!keepSet) keep = m1ok ? 1u : 0u;               // =2: a patched controller stays on; =1 and failures: off
      geni_uart_rx_stop(&gUart, 2000);
      unsigned pulses = 1;
      PE_parse_boot_argn("guamp-bt-pulse", &pulses, sizeof(pulses));
      if (pulses && !keep) IOLog("GuampBt: power-off pulse at the end rc %d\n", pulse(BT_QCA_POWEROFF_PULSE, 2400));
      on = !pulses || keep;
      geni_uart_flow(&gUart, 0);
    }
  }
  gStep = 9;
  if (mode != 2) {
    IOLog("GuampBt: RESULT rails %s uart %s reset %s version %s qca %s\n", railState, uartState, gReset, gVersion, gQca);
    return;
  }
  IOLog("GuampBt: RESULT rails %s uart %s reset %s version %s qca %s download %s baud %u %s version after %s bdaddr %s "
        "controller %s\n", railState, uartState, gReset, gVersion, gQca, gDownload, gBaudRate, gBaud, gAfter, gBdaddr,
        on ? "on" : "off");
}

// guamp-bt-addr: parsed at start, and published in the ADT nodes iOS reads (ADT-CAPACITA-BT.md: libMobileGestalt takes
// /chosen/mac-address-bluetooth0, the bluetooth node carries local-mac-address) before bluetoothd and MobileGestalt run.
// Only the nodes that exist are changed; six bytes in text order, as the Apple ADTs hold them.
void publishAddress() {
  char text[24] = {};
  if (!PE_parse_boot_argn("guamp-bt-addr", text, sizeof(text))) return;
  if (bt_bd_addr_parse(text, gAddr)) { IOLog("GuampBt: guamp-bt-addr not aa:bb:cc:dd:ee:ff, ignored\n"); return; }
  gHaveAddr = true;
  uint8_t be[6];
  bt_qca_bdaddr_params(gAddr, be);
  static const char *const kNodes[2][2] = {{"/arm-io/uart1/bluetooth", "local-mac-address"},
                                           {"/chosen", "mac-address-bluetooth0"}};
  unsigned done = 0;
  for (unsigned i = 0; i < 2; ++i) {
    IORegistryEntry *e = IORegistryEntry::fromPath(kNodes[i][0], gIODTPlane);
    if (!e) continue;
    OSData *d = OSData::withBytes(be, sizeof be);
    if (d) {
      if (e->setProperty(kNodes[i][1], d)) done = done + 1;
      d->release();
    }
    e->release();
  }
  char masked[18];
  bt_bd_addr_masked(gAddr, masked);
  IOLog("GuampBt: guamp-bt-addr %s: %u/2 ADT properties set\n", masked, done);
}

void start() {
  publishAddress();
  unsigned on = 0;
  if (!PE_parse_boot_argn("guamp-bt", &on, sizeof(on)) || (on != 1 && on != 2)) return;
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}
}  // namespace bt
