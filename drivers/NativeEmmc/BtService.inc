// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Bluetooth milestone M0, included by NativeEmmc.cpp after PmicService.inc (mapPhysical, the boot log, the SPMI
// helpers). A one-shot diagnostic on its own kernel thread, opt-in with boot argument guamp-bt=1:
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
// Other boot arguments: guamp-bt-delay=<s> (start later, so the USB mirror sees the lines), guamp-bt-uart=0 (rails
// only), guamp-bt-pulse=0 (no power pulses), guamp-bt-os=16|32 (UART oversampling; default from the QUP version).
// Outcome of a boot: the single "GuampBt: RESULT ..." line. Writes stay limited to SE3, gpio8-11, the SE3 GCC votes /
// RCG and (only with guamp-bt-ldo=1/2) the four LDO enable/voltage registers. Every wait has a timeout.
// Board facts: GuampBtBoard.h; UART engine: GeniUart.h; HCI framing and decoders: BtHci.h (host-tested).

#include "GuampSpmi.h"
#include "GuampPmicBoard.h"
#include "GuampBtBoard.h"
#include "GeniUart.h"
#include "BtHci.h"
#include "GeniUartCpp.inc"
#include "GuampRpmCore.h"

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
size_t gRxLen;
volatile uint32_t gStep;

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
bool setBaud(unsigned baud) {
  struct bt_uart_clock c;
  if (bt_choose_uart_clock(gLevelHz, GT_DFS_LEVELS, baud, gOs, &c)) { IOLog("GuampBt: no clock level for %u baud\n", baud); return false; }
  const int err = bt_baud_error_permille(baud, c.baud_actual);
  const unsigned aerr = static_cast<unsigned>(err < 0 ? -err : err);
  IOLog("GuampBt: %u baud: level %u (%llu Hz) div %u -> %u baud (%s%u.%u%%), os %u\n", baud, c.level,
        static_cast<unsigned long long>(c.src_hz), c.div, c.baud_actual, err < 0 ? "+" : "-", aerr / 10, aerr % 10, gOs);
  if (aerr > 25) { IOLog("GuampBt: %u baud error above 2.5%%, not used\n", baud); return false; }
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
    const uint32_t m = BT_RCG_7M37_M, n = BT_RCG_7M37_N;
    wr(gGcc, BT_S3_RCG + BT_RCG_M, m);
    wr(gGcc, BT_S3_RCG + BT_RCG_N, bt_rcg_n_reg(m, n));
    wr(gGcc, BT_S3_RCG + BT_RCG_D, bt_rcg_d_reg(m, n));
    wr(gGcc, BT_S3_RCG + GT_RCG_CFG, BT_RCG_7M37_CFG);
    wr(gGcc, BT_S3_RCG + GT_RCG_CMD, rd(gGcc, BT_S3_RCG + GT_RCG_CMD) | 1u);
    for (unsigned i = 0; i < 1000 && (rd(gGcc, BT_S3_RCG + GT_RCG_CMD) & 1u); ++i) IODelay(1);
    gLevelHz[0] = gt_level_hz(rd(gGcc, BT_S3_RCG + GT_RCG_CFG), rd(gGcc, BT_S3_RCG + BT_RCG_M), rd(gGcc, BT_S3_RCG + BT_RCG_N));
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
// One command, then events until the expected one or timeoutMs. kind 0: Command Complete of op; 1: QCA version reply.
// The returned event points into gRx (valid until the next command).
int command(const char *what, uint16_t op, const uint8_t *params, uint8_t plen, unsigned kind, unsigned timeoutMs, bt_event *out) {
  uint8_t pkt[16];
  const size_t n = bt_h4_cmd(pkt, sizeof pkt, op, params, plen);
  gRxLen = 0;
  int r = geni_uart_write(&gUart, pkt, n, 200000);
  if (r) {
    IOLog("GuampBt: %s: TX failed %d (m_irq %08x, CTS gpio8 level %u)\n", what, r, gUart.last_m_irq, pinRead(8, 4) & 1u);
    return r;
  }
  size_t pos = 0;
  unsigned skipped = 0, events = 0;
  int found = GENI_ERR_TIMEOUT;
  for (unsigned t = 0; t <= timeoutMs && found == GENI_ERR_TIMEOUT; ++t) {
    gRxLen += geni_uart_read(&gUart, gRx + gRxLen, sizeof gRx - gRxLen);
    bt_event ev;
    while (found == GENI_ERR_TIMEOUT && bt_h4_next(gRx, gRxLen, &pos, &ev, &skipped)) {
      events = events + 1;
      uint16_t eop = 0; const uint8_t *ret = nullptr; uint8_t rlen = 0, st = 0;
      bt_qca_version qv;
      if (kind == 0 && !bt_cmd_complete(&ev, &eop, &ret, &rlen) && eop == op) { *out = ev; found = 0; }
      else if (kind == 1 && !bt_decode_qca_version(&ev, &qv)) { *out = ev; found = 0; }
      else if (!bt_cmd_status(&ev, &eop, &st) && eop == op && st) {
        IOLog("GuampBt: %s: Command Status %02x\n", what, st);
        *out = ev; found = 1;
      }
    }
    if (gRxLen == sizeof gRx) break;
    if (found == GENI_ERR_TIMEOUT) IOSleep(1);
  }
  hexLine(what, gRx, gRxLen);
  if (found == GENI_ERR_TIMEOUT)
    IOLog("GuampBt: %s: no reply in %u ms (%u bytes, %u events, %u bytes skipped, dropped %u, CTS gpio8 level %u)\n", what,
          timeoutMs, static_cast<unsigned>(gRxLen), events, skipped, gUart.rx_dropped, pinRead(8, 4) & 1u);
  return found;
}

int pulse(uint8_t byte, unsigned baud) {
  geni_uart_flow(&gUart, 0);                         // flow control off for the pulses (hci_qca.c)
  if (!setBaud(baud)) return GENI_ERR_PARAM;
  const int r = geni_uart_write(&gUart, &byte, 1, 50000);
  IODelay(200);                                      // "Wait for 100 uS for SoC to settle down"
  return r;
}

const char *gReset = "-", *gVersion = "-", *gQca = "-";
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

void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-bt");
  unsigned delay = 0;
  PE_parse_boot_argn("guamp-bt-delay", &delay, sizeof(delay));
  if (delay && delay <= 600) IOSleep(delay * 1000);
  const char *railState = rails();
  unsigned uart = 1, keep = 0;
  PE_parse_boot_argn("guamp-bt-uart", &uart, sizeof(uart));
  PE_parse_boot_argn("guamp-bt-keep", &keep, sizeof(keep));
  const char *uartState = "skipped";
  if (uart) {
    if (!uartUp()) {
      uartState = "fail";
    } else {
      uartState = "up";
      hci();
      geni_uart_rx_stop(&gUart, 2000);
      unsigned pulses = 1;
      PE_parse_boot_argn("guamp-bt-pulse", &pulses, sizeof(pulses));
      if (pulses && !keep) IOLog("GuampBt: power-off pulse at the end rc %d\n", pulse(BT_QCA_POWEROFF_PULSE, 2400));
      geni_uart_flow(&gUart, 0);
    }
  }
  gStep = 9;
  IOLog("GuampBt: RESULT rails %s uart %s reset %s version %s qca %s\n", railState, uartState, gReset, gVersion, gQca);
}

void start() {
  unsigned on = 0;
  if (!PE_parse_boot_argn("guamp-bt", &on, sizeof(on)) || on != 1) return;
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}
}  // namespace bt
