// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp scanout, hardware scaling (no kernel dependency; host test tests/scanout_hw_host.cpp): the register values that make
// the MDSS VIG0 source pipe scan a canvas-sized frame (750x1666 BGRA) and scale it to the panel (720x1600) in its QSEED3LITE
// scaler, so the copy thread only has to copy the swapped surface 1:1 (no CPU scaler) into one of two slots of the boot
// loader's display region, and flip.
//
// The values are those of Linux v6.12 drivers/gpu/drm/msm/disp/dpu1 for an RGB plane with src != dst:
//   dpu_plane.c   _dpu_plane_setup_scaler3: phase_step = mult_frac(1 << 21, src, dst) for every component (hsub = vsub = 1),
//                 init phase 0, preload (version >= 0x3000) x 4 / y 2, filters DPU_SCALE_BIL (3) and DPU_SCALE_ALPHA_BIL (1),
//                 blend_cfg 1, lut_flag 0 (no LUT for the bilinear filter), dir_en / de off.
//                 _dpu_plane_setup_pixel_ext: num_ext_pxls_left/top = src w/h, everything else 0.
//   dpu_hw_util.c dpu_hw_setup_scaler3: OP_MODE = BIT0 | y_rgb_filter << 16 | blend << 31 | BIT14 (not DX) |
//                 (alpha_enable: BIT10 | alpha_filter << 29); PHASE_INIT_Y_H/Y_V/UV_H/UV_V 0x90..0x9c (version != 0x1002),
//                 PHASE_STEP_Y_H 0x10, _Y_V 0x14, _UV_H 0x18, _UV_V 0x1c, PRELOAD 0x20, SRC_SIZE_Y_RGB_A 0x40, SRC_SIZE_UV 0x44,
//                 DST_SIZE 0x48, OP_MODE 0x04 (written last); HW_VERSION 0x00.
//   dpu_hw_sspp.c SSPP_SRC_SIZE 0x00, SRC_XY 0x08, OUT_SIZE 0x0c, SRC0_ADDR 0x14, SRC_YSTRIDE0 0x24, SRC_FORMAT 0x30 (BIT8 =
//                 alpha, SRCC3_EN), SRC_OP_MODE 0x38 (setup_format always sets MDSS_MDP_OP_PE_OVERRIDE, BIT31),
//                 SW_PIX_EXT C0 0x100/0x104/0x108, C1C2 0x110/0x114/0x118, C3 0x120/0x124/0x128 (LR, TB, REQ_PIXELS =
//                 (h << 16) | w with the extension above).
//   catalog/dpu_6_3_sm6115.h: VIG0 at MDP + 0x4000 (MDSS + 0x5000), dpu_vig_sblk_qseed3_3_0: scaler at pipe + 0xa00,
//                 version SSPP_SCALER_VER(3, 0) (QSEED3LITE).
// All SSPP and scaler registers are double buffered: written while the pipe scans, they are taken together with SRC0_ADDR by
// the CTL flush at the next vsync, so a flip between a 1:1 frame and a scaled one is atomic.
#ifndef GUAMP_SCANOUT_HW_H
#define GUAMP_SCANOUT_HW_H

namespace gscanhw {
typedef unsigned long long u64;
typedef unsigned int u32;

// Pipe-relative register offsets.
enum : u32 {
  kSrcSize = 0x00, kSrcXY = 0x08, kOutSize = 0x0c, kSrc0Addr = 0x14, kYStride0 = 0x24, kSrcFormat = 0x30, kSrcOpMode = 0x38,
  kPeC0LR = 0x100, kPeC0TB = 0x104, kPeC0Req = 0x108, kPeC12LR = 0x110, kPeC12TB = 0x114, kPeC12Req = 0x118,
  kPeC3LR = 0x120, kPeC3TB = 0x124, kPeC3Req = 0x128,
  kScaler = 0xa00,
  kScVersion = 0x00, kScOpMode = 0x04, kScStepYH = 0x10, kScStepYV = 0x14, kScStepUVH = 0x18, kScStepUVV = 0x1c,
  kScPreload = 0x20, kScSrcY = 0x40, kScSrcUV = 0x44, kScDst = 0x48,
  kScInitYH = 0x90, kScInitYV = 0x94, kScInitUVH = 0x98, kScInitUVV = 0x9c,
};
constexpr u32 kPeOverride = 1u << 31;
constexpr u32 kPhaseShift = 21;

// mult_frac() of linux/math.h for unsigned values: x * n / d without overflowing x * n.
inline u32 multFrac(u32 x, u32 n, u32 d) { return (x / d) * n + ((x % d) * n) / d; }

// QSEED3LITE: HW_VERSION major 3 (bits 31:28).
inline bool liteScaler(u32 version) { return (version >> 28) == 3; }

// The registers a flip writes: everything the configuration of one kind of frame (1:1 or scaled) owns. The order of `regs` is
// the write order (scaler OP_MODE last of the scaler block, as dpu_hw_setup_scaler3 does).
struct Reg { u32 off, value; };
constexpr unsigned kMaxRegs = 32;
struct Config {
  Reg regs[kMaxRegs];
  unsigned count = 0;
  void add(u32 off, u32 value) { if (count < kMaxRegs) regs[count++] = Reg{off, value}; }
  bool has(u32 off, u32 *value) const
  {
    for (unsigned i = 0; i < count; ++i) if (regs[i].off == off) { if (value) *value = regs[i].value; return true; }
    return false;
  }
};

// The registers of the boot loader's 1:1 setup, read once at the probe; written back to return to the GOP frame.
constexpr u32 kSaved[] = {kSrcSize, kSrcXY, kYStride0, kSrcOpMode, kPeC0LR, kPeC0TB, kPeC0Req, kPeC12LR, kPeC12TB, kPeC12Req,
                          kPeC3LR, kPeC3TB, kPeC3Req, kScaler + kScStepYH, kScaler + kScStepYV, kScaler + kScStepUVH,
                          kScaler + kScStepUVV, kScaler + kScPreload, kScaler + kScSrcY, kScaler + kScSrcUV, kScaler + kScDst,
                          kScaler + kScInitYH, kScaler + kScInitYV, kScaler + kScInitUVH, kScaler + kScInitUVV,
                          kScaler + kScOpMode};
constexpr unsigned kSavedCount = sizeof(kSaved) / sizeof(kSaved[0]);
static_assert(kSavedCount <= kMaxRegs, "saved set");

// The boot loader's values (saved[i] for kSaved[i]) as they are: the GOP frame's own setup.
inline Config original(const u32 *saved)
{
  Config c;
  for (unsigned i = 0; i < kSavedCount; ++i) c.add(kSaved[i], saved[i]);
  return c;
}

// A 1:1 frame of w x h at `stride` in another buffer (a composed frame in a slot): the boot loader's setup with the size and
// stride of this frame; false when it is not the panel's size (the mixer output stays the panel).
inline bool panel(Config &c, const u32 *saved, u32 outSize, u32 w, u32 h, u32 stride)
{
  c = original(saved);
  if (!w || !h || ((h << 16) | w) != outSize || stride < w * 4 || stride > 0xffff || (stride & 7)) return false;
  for (unsigned i = 0; i < c.count; ++i) {
    if (c.regs[i].off == kSrcSize) c.regs[i].value = (h << 16) | w;
    if (c.regs[i].off == kSrcXY) c.regs[i].value = 0;
    if (c.regs[i].off == kYStride0) c.regs[i].value = stride;
  }
  return true;
}

// A canvas frame sw x sh at `stride`, scaled to the pipe's output (outSize, the panel). Down-scaling below 2:1 and the same
// direction on both axes only (the canvas of the variant: at least the panel, less than twice it), 8-bit RGB formats.
inline bool scaled(Config &c, const u32 *saved, u32 outSize, u32 srcFormat, u32 sw, u32 sh, u32 stride)
{
  c = Config{};
  const u32 dw = outSize & 0xffff, dh = outSize >> 16;
  if (!dw || !dh || sw < dw || sh < dh || sw >= 2 * dw || sh >= 2 * dh || (sw == dw && sh == dh)) return false;
  if (stride < sw * 4 || stride > 0xffff || (stride & 7) || sw > 0xffff || sh > 0xffff) return false;
  if ((srcFormat & 0xff) != 0xff || ((srcFormat >> 9) & 3) != 3) return false;   // 8 bits per channel, 4 bytes per pixel
  const bool alpha = (srcFormat >> 8) & 1;
  u32 opMode = 0;
  for (unsigned i = 0; i < kSavedCount; ++i) if (kSaved[i] == kSrcOpMode) opMode = saved[i];
  const u32 stepX = multFrac(1u << kPhaseShift, sw, dw), stepY = multFrac(1u << kPhaseShift, sh, dh);
  const u32 size = (sh << 16) | sw;
  c.add(kSrcSize, size);
  c.add(kSrcXY, 0);
  c.add(kYStride0, stride);
  c.add(kSrcOpMode, opMode | kPeOverride);
  c.add(kPeC0LR, 0); c.add(kPeC0TB, 0); c.add(kPeC0Req, size);
  c.add(kPeC12LR, 0); c.add(kPeC12TB, 0); c.add(kPeC12Req, size);
  c.add(kPeC3LR, 0); c.add(kPeC3TB, 0); c.add(kPeC3Req, size);
  c.add(kScaler + kScInitYH, 0); c.add(kScaler + kScInitYV, 0); c.add(kScaler + kScInitUVH, 0); c.add(kScaler + kScInitUVV, 0);
  c.add(kScaler + kScStepYH, stepX & 0xffffff); c.add(kScaler + kScStepYV, stepY & 0xffffff);
  c.add(kScaler + kScStepUVH, stepX & 0xffffff); c.add(kScaler + kScStepUVV, stepY & 0xffffff);
  c.add(kScaler + kScPreload, 4u | (2u << 8) | (4u << 16) | (2u << 24));
  c.add(kScaler + kScSrcY, (sw & 0x1ffff) | ((sh & 0x1ffff) << 16));
  c.add(kScaler + kScSrcUV, (sw & 0x1ffff) | ((sh & 0x1ffff) << 16));
  c.add(kScaler + kScDst, (dw & 0x1ffff) | ((dh & 0x1ffff) << 16));
  c.add(kScaler + kScOpMode, 1u | (3u << 16) | (1u << 31) | (1u << 14) | (alpha ? (1u << 10) | (1u << 29) : 0));
  return true;
}

// The two slots of the display region after the GOP frame: slot i at gop + first + i * size, each `size` bytes, page aligned,
// above the GOP frame, below `end` (the boot loader's cont_splash part: 0xF00000 above its start, the rest is dfps).
struct Slots { u64 pa[2]; u64 size; };
inline bool slots(Slots &s, u64 gopPa, u64 gopBytes, u64 first, u64 size, u64 end)
{
  s = Slots{};
  if (!size || (size & 0xfff) || (first & 0xfff) || (gopPa & 0xfff) || first < gopBytes) return false;
  const u64 a = gopPa + first, b = a + size;
  if (b + size > end || b + size < b || (b + size - 1) >> 32) return false;   // 32-bit SRC0_ADDR
  s.pa[0] = a; s.pa[1] = b; s.size = size;
  return true;
}

// The slot stride of a canvas row: 64-byte aligned (what the GPU allocator hands out too).
inline u32 slotStride(u32 w) { return (w * 4 + 63) & ~63u; }
}  // namespace gscanhw
#endif
