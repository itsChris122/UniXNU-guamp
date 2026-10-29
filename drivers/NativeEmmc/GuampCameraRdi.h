// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp camera, step 2 SKELETON (not called by the kernel yet): register plan of the first test frame
// TPG -> CSID RDI0 -> TFE write master 7 (RDI0) -> memory, RAW10 packed, frame-based write master.
// Built as a list of writes so the host test can print and check it before anything touches the hardware.
// Offsets: techpack camera-bengal cam_tfe_csid530.h, cam_tfe530.h, cam_top_tpg_v1.h; sequences studied from
// cam_tfe_csid_core.c (enable_csi2, init_config_rdi_path, enable_rdi_path), cam_tfe_bus.c (acquire_rdi_wm,
// start_wm), cam_tfe_core.c (RDI start), cam_top_tpg_core.c (start). Structure follows the mainline QCM2290 CAMSS
// (camss-csid-340.c / camss-vfe-340.c: RDI only, one write master per RDI, frame based), which is the reference for
// the XNU driver; mainline has no TPG for that SoC. Nothing here is verified on the device.
// Missing before it can run (see analisi/camera/FOTOCAMERA-PASSO1-20261029.md): CPAS/CAMNOC AHB+AXI votes, TFE and
// CSID clocks (gcc_camss_tfe_N, csiN, cphy_rx), SMMU stream 0x400 (bypass or identity context), CSIPHY for the
// real sensor, IRQ or polling of the RDI SOF/EOF and write-master done bits.
#ifndef GUAMP_CAMERA_RDI_H
#define GUAMP_CAMERA_RDI_H
#include <stdint.h>

namespace gcam {
namespace rdi {

// physical bases (DT)
constexpr uint64_t kCpasTop = 0x05c11000;   // "top": TPG mux at +0x1c
constexpr uint64_t kTpg[2] = {0x05c66000, 0x05c68000};
constexpr uint64_t kCsidTfe[3] = {0x05c6e000, 0x05c75000, 0x05c7c000};   // CSID at +0, TFE top/bus at +0x1000.. (0x5000)
constexpr uint64_t kCsiphy[3] = {0x05c52000, 0x05c53000, 0x05c54000};

enum Block : uint8_t { kTop = 0, kTpgBlk = 1, kCsid = 2, kTfe = 3 };

// TPG v1
constexpr uint32_t kTpgCtrl = 0x60, kTpgVcCfg0 = 0x64, kTpgVcCfg1 = 0x68, kTpgLfsr = 0x6c;
constexpr uint32_t kTpgDt0Cfg0 = 0x70, kTpgDt0Cfg1 = 0x80, kTpgDt0Cfg2 = 0x90, kTpgColorBar = 0xa0, kTpgCommonGen = 0xa4, kTpgVbi = 0xa8;
constexpr uint32_t kTopTpgMux = 0x1c;
// CSID 530
constexpr uint32_t kCsidRst = 0x10, kCsidTopIrqStatus = 0x70, kCsidTopIrqMask = 0x74, kCsidTopIrqClear = 0x78, kCsidIrqCmd = 0x80;
constexpr uint32_t kCsi2Cfg0 = 0x100, kCsi2Cfg1 = 0x104, kCsi2IrqMask = 0x24;
constexpr uint32_t rdiCfg0(unsigned n) { return 0x300 + 0x100 * n; }
constexpr uint32_t rdiCfg1(unsigned n) { return 0x304 + 0x100 * n; }
constexpr uint32_t rdiCtrl(unsigned n) { return 0x308 + 0x100 * n; }
constexpr uint32_t rdiErrCfg0(unsigned n) { return 0x3b0 + 0x100 * n; }
constexpr uint32_t rdiIrqMask(unsigned n) { return 0x44 + 0x10 * n; }
// TFE 530 (same window, offsets from the CSID/TFE base)
constexpr uint32_t kTfeRegUpdate = 0x102c;
constexpr uint32_t tfeRdiModule(unsigned n) { return 0x1460 + 0x200 * n; }
constexpr unsigned kWmRdi0 = 7;                          // bus client 7 = RDI0, 8 = RDI1, 9 = RDI2/PDAF
constexpr uint32_t wm(unsigned c) { return 0x1c00 + 0x100 * c; }
constexpr uint32_t kWmCfg = 0x00, kWmImageAddr = 0x04, kWmFrameIncr = 0x08, kWmImageCfg0 = 0x0c, kWmImageCfg2 = 0x14,
                   kWmPacker = 0x18, kWmBwLimit = 0x1c;

struct Cfg {
  bool tpg = true;            // source: TPG (true) or CSIPHY phy (false)
  unsigned tpgIndex = 0, phy = 0, lanes = 1, rdi = 0;
  unsigned width = 1600, height = 1200, vc = 0, dt = 0x2b;   // RAW10 (MIPI DT 0x2b)
  uint32_t bufferIova = 0;    // write-master target (SMMU IOVA or bypass physical, 4 KiB aligned)
  uint32_t pattern = 0;       // TPG pattern 0..7 (techpack values), 9 = split colour bars
};

struct Write { uint8_t block; uint8_t rmwOr; uint32_t off, val; const char *what; };

// RAW10 packed: 5 bytes per 4 pixels, stride rounded up to 16 bytes
constexpr uint32_t raw10Stride(uint32_t w) { return ((w * 5 / 4) + 15) / 16 * 16; }

// Builds the plan into out[0..max); returns the count, 0 when it does not fit. rmwOr=1: OR into the current value.
inline unsigned plan(const Cfg &c, Write *out, unsigned max) {
  unsigned n = 0;
  auto add = [&](uint8_t b, uint32_t off, uint32_t v, const char *w, uint8_t rmw = 0) {
    if (n < max) out[n] = {b, rmw, off, v, w};
    ++n;
  };
  const unsigned lanes = c.lanes ? c.lanes : 1;
  if (c.tpg) {
    add(kTpgBlk, kTpgLfsr, 0x12345678, "tpg lfsr seed");
    add(kTpgBlk, kTpgDt0Cfg0, (c.width & 0xffff) << 16 | (c.height & 0x3fff), "tpg dt0 width/height");
    add(kTpgBlk, kTpgDt0Cfg1, c.dt, "tpg dt0 data type");
    add(kTpgBlk, kTpgVcCfg0, (0u << 8) | c.vc, "tpg vc0: one dt");
    add(kTpgBlk, kTpgVcCfg1, 0x2581f4, "tpg h/v blank (techpack default)");
    if (c.pattern > 7) add(kTpgBlk, kTpgColorBar, 1u << 5, "tpg split colour bar");
    add(kTpgBlk, kTpgDt0Cfg2, 2u << 16 | (c.pattern > 7 ? 9u : c.pattern), "tpg dt0 RAW10 encode + pattern");
    add(kTpgBlk, kTpgCommonGen, 0, "tpg pixel pattern");
    add(kTpgBlk, kTpgVbi, 0xafff, "tpg vbi");
    add(kTop, kTopTpgMux, 1u << c.tpgIndex, "cpas top: tpg mux select", 1);
  }
  // CSID: software reset, CSI2 RX, RDI path
  add(kCsid, kCsidTopIrqMask, 1u, "csid top irq: reset done");
  add(kCsid, kCsidRst, 0x1f, "csid reset strobes (sw all), then wait top irq bit 0");
  const unsigned phySel = c.tpg ? c.tpgIndex : c.phy;   // TPG: techpack phy_sel - 1, to verify on the device
  add(kCsid, kCsi2Cfg0, (lanes - 1) | (c.tpg ? 0u : 0x3210u) << 4 | 0u << 24 | (phySel & 3) << 20, "csi2 rx cfg0: lanes, lane map, d-phy, phy sel");
  add(kCsid, kCsi2Cfg1, 1u << 6 | 1u, "csi2 rx cfg1: misr + ecc correction");
  add(kCsid, kCsi2IrqMask, 0x0fffffff, "csi2 rx irq mask all");
  add(kCsid, rdiCfg0(c.rdi), c.vc << 22 | c.dt << 16 | 0u << 27 | 0xfu << 12 | 0u << 10 | 1u << 2 | 1u,
      "rdi cfg0: vc, dt, MIPI RAW10 passthrough (fmt 0xf), timestamps");
  add(kCsid, rdiCfg1(c.rdi), 2, "rdi cfg1: timestamp strobe post irq");
  add(kCsid, rdiErrCfg0(c.rdi), 0x9, "rdi error detection, overflow detect");
  add(kCsid, rdiCtrl(c.rdi), 0, "rdi halt mode");
  add(kCsid, rdiCfg0(c.rdi), 1u << 31, "rdi path enable", 1);
  add(kCsid, rdiIrqMask(c.rdi), 0x3ffff, "rdi irq mask all (sof/eof/overflow)");
  // TFE: RDI input, write master frame based, RAW10 packed
  const unsigned client = kWmRdi0 + c.rdi;
  const uint32_t stride = raw10Stride(c.width);
  add(kTfe, tfeRdiModule(c.rdi), 1, "tfe rdi module: out enable");
  add(kTfe, wm(client) + kWmBwLimit, 0xf, "wm bw limit");
  add(kTfe, wm(client) + kWmImageCfg0, 0xffff, "wm image cfg0: frame based (width 0xffff, height 0)");
  add(kTfe, wm(client) + kWmPacker, 0xa, "wm packer: plain/raw passthrough (0xa)");
  add(kTfe, wm(client) + kWmImageCfg2, 0xffff, "wm stride: frame based default");
  add(kTfe, wm(client) + kWmImageAddr, c.bufferIova, "wm image address");
  add(kTfe, wm(client) + kWmFrameIncr, stride * c.height, "wm frame increment (one frame)");
  add(kTfe, wm(client) + kWmCfg, 1u << 16 | 1u, "wm enable, frame based");
  add(kTfe, kTfeRegUpdate, 1u << (1 + c.rdi), "tfe reg update (RDIn)");
  // start: CSID resume at frame boundary, then the source
  add(kCsid, rdiCtrl(c.rdi), 1, "rdi resume at frame boundary");
  if (c.tpg)
    add(kTpgBlk, kTpgCtrl, (lanes - 1) << 4 | 1u << 2 | 1u << 1 | (phySel & 7) << 3 | 1u, "tpg ctrl: lanes, fe/fs packets, phy sel, enable");
  return n <= max ? n : 0;
}

}  // namespace rdi
}  // namespace gcam
#endif
