// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Adreno 610: the first visible GPU work, a 2D-engine solid fill (CP_BLIT, BLIT_OP_SCALE with SOLID_COLOR) of a
// rectangle of a linear 32 bpp surface, followed by a CACHE_FLUSH_TS fence. Pure code: it only builds PM4 dwords,
// the owner (GpuService.inc, after a successful M2 CP start) copies them into the ring, flushes the cache lines,
// advances CP_RB_WPTR and polls the fence word. Host-tested by tests/gpu_blit_host.cpp.
// References: Mesa 25.0.0 (gitlab.freedesktop.org/mesa/mesa tag mesa-25.0.0, 4fa244fd), the gallium a6xx clear:
//   fd6_blitter.cc:990   fd6_clear_surface: GRAS_2D_DST_TL/BR, emit_clear_color, emit_blit_setup, emit_blit_dst,
//                        emit_blit_fini
//   fd6_blitter.cc:255   emit_setup: CCU clean + invalidate (color, depth), then fd6_emit_ccu_cntl(gmem = false)
//   fd6_blitter.cc:292   emit_blit_setup: RB_2D_BLIT_CNTL = GRAS_2D_BLIT_CNTL (MASK 0xf, COLOR_FORMAT, IFMT,
//                        SOLID_COLOR), SP_2D_DST_FORMAT (COLOR_FORMAT, MASK 0xf), RB_2D_UNKNOWN_8C01 0
//   fd6_blitter.cc:739   emit_clear_color: RB_2D_SRC_SOLID_C0..C3 = R, G, B, A as unorm8 bytes for R2D_UNORM8
//   fd6_blitter.cc:558   emit_blit_dst: RB_2D_DST_INFO (format, TILE6_LINEAR, swap), RB_2D_DST, RB_2D_DST_PITCH
//   fd6_blitter.cc:272   emit_blit_fini: LABEL event, WFI, RB_DBG_ECO_CNTL = magic blit value, CP_BLIT
//                        BLIT_OP_SCALE, WFI, RB_DBG_ECO_CNTL 0
//   fd6_emit.cc:794      fd6_emit_ccu_cntl, sysmem: WFI, RB_CCU_CNTL with GMEM_FAST_CLEAR_DISABLE (A610 has no gmem
//                        fast clear), cache sizes FULL, depth offset 0, color offset = depth cache size
//                        (fd6_screen.cc:155-166: num_ccu 1 x sysmem_per_ccu_depth_cache_size 8 KiB = 0x2000)
//   fd6_barrier.cc:23    fd6_emit_flushes: CCU_CLEAN_COLOR/DEPTH, CCU_INVALIDATE_COLOR/DEPTH
//   fd6_emit.h:204       __event_write on A6XX: CP_EVENT_WRITE, 4 dwords {event | TIMESTAMP, addr lo, hi, value}
//                        for the _TS events, 1 dword otherwise; freedreno_gpu_event.h:62 the A6XX event table
//   tu_clear_blit.cc     r2d_setup_common (turnip): the same register set for the 2D engine outside a render pass
// Register offsets and fields: registers/adreno/a6xx.xml (RB_2D_BLIT_CNTL 0x8c00 :3992, GRAS_2D_BLIT_CNTL 0x8400
// :3491, a6xx_2d_blit_cntl :3474, GRAS_2D_DST_TL/BR 0x8405/0x8406 :3500, a6xx_reg_xy :3026, RB_2D_UNKNOWN_8C01
// :3993, RB_2D_DST_INFO 0x8c17 :4025 with a6xx_2d_dst_surf_info :4014, RB_2D_DST 0x8c18 (64 bit, 64-byte aligned),
// RB_2D_DST_PITCH 0x8c1a (bytes >> 6) :4027, RB_2D_SRC_SOLID_C0..3 0x8c2c..0x8c2f :4041, RB_UNKNOWN_8E01 :4051,
// RB_DBG_ECO_CNTL 0x8e04 :4053, RB_CCU_CNTL 0x8e07 (A6XX) :4058, SP_2D_DST_FORMAT 0xacc0 (A6XX) :5329 with
// a6xx_sp_2d_dst_format :5316, FMT6_8_8_8_8_UNORM 0x30 :59, R2D_UNORM8 0x10 :1139, TILE6_LINEAR 0 :25);
// adreno_common.xml:345 color swap WZYX 0 / WXYZ 1; adreno_pm4.xml: CP_WAIT_FOR_IDLE 0x26 :278, CP_EVENT_WRITE 0x46
// :310, CP_BLIT 0x2c :518 (BLIT_OP_SCALE 3 :1719), events CACHE_FLUSH_TS 4, PC_CCU_INVALIDATE_DEPTH 24 / COLOR 25,
// PC_CCU_FLUSH_DEPTH_TS 28 / COLOR_TS 29, LABEL 63. freedreno_devices.py:440 (GPUId 610): RB_DBG_ECO_CNTL_blit
// 0x04100000, RB_UNKNOWN_8E01 1, num_ccu 1; a6xx_gen1_low :348 has_gmem_fast_clear False. Formats:
// fdl/fd6_format_table.c:174 B8G8R8A8_UNORM = 8_8_8_8_UNORM swap WXYZ, :163 R8G8B8A8_UNORM swap WZYX.
// Type 4 header: kgsl adreno_pm4types.h:187 cp_type4_packet (odd parity of the count at bit 7, of the register at 27).
//
// Not done (on purpose): no CP_SET_MARKER / render mode, no fd6_emit_restore register set except RB_UNKNOWN_8E01,
// no UBWC, no scissor, no MSAA. The surface must be reachable by the GPU at the address given: with the GPU SMMU
// translating stream 0 (gm2 "translated") the caller must pass an IOVA it mapped itself and set addrIsGpuVa; with
// the SMMU in identity/bypass the physical address is used as is.
//
// Hooking it after M2 (GpuService.inc, not done here): when gm2_state shows the CP consumed the M2 ring (rptr ==
// wptr, CP_MEM_WRITE magic seen) and the SMMU decision is identity, build at the current ring write offset
//   gmb_clear c = {}; c.dst = 0x5C000000 (NativeEmmc.cpp kFbPhys, the GOP surface the panel scans out, 720x1600,
//   row 2880 bytes, BGRA8888); c.pitch = 2880; c.surfW = 720; c.surfH = 1600; c.x/y/w/h = the rectangle;
//   c.argb = 0xFFFF00FF; c.bgra = true; c.ts = scratchPhys + 32; c.fence = scratchPhys + 48; c.fenceValue = n;
//   n = gmb_build_clear(&ring[wptr], ringDwords - wptr, &c);
// then flush the ring lines, write CP_RB_WPTR = wptr + n, poll the fence word (and CP_RB_RPTR, RBBM_STATUS,
// CP_HW_FAULT, RBBM_INT_0_STATUS) for at most a few hundred ms, and log it. iOS repaints that surface on its next
// frame, so the rectangle is visible until then (best on a still screen, e.g. the lock screen).
#ifndef GUAMP_GPU_BLIT_H
#define GUAMP_GPU_BLIT_H
#include <stdint.h>
#include <stddef.h>
#include "GuampGpuM2.h"

// --- registers (dword offsets) ----------------------------------------------------------------------------------
#define GMB_GRAS_2D_BLIT_CNTL 0x8400
#define GMB_GRAS_2D_DST_TL 0x8405
#define GMB_GRAS_2D_DST_BR 0x8406
#define GMB_RB_2D_BLIT_CNTL 0x8c00
#define GMB_RB_2D_UNKNOWN_8C01 0x8c01
#define GMB_RB_2D_DST_INFO 0x8c17
#define GMB_RB_2D_DST_LO 0x8c18
#define GMB_RB_2D_DST_HI 0x8c19
#define GMB_RB_2D_DST_PITCH 0x8c1a
#define GMB_RB_2D_SRC_SOLID_C0 0x8c2c
#define GMB_RB_UNKNOWN_8E01 0x8e01
#define GMB_RB_DBG_ECO_CNTL 0x8e04
#define GMB_RB_CCU_CNTL 0x8e07
#define GMB_SP_2D_DST_FORMAT 0xacc0

// --- PM4 opcodes and events --------------------------------------------------------------------------------------
#define GMB_CP_WAIT_FOR_IDLE 0x26
#define GMB_CP_EVENT_WRITE 0x46
#define GMB_CP_BLIT 0x2c
#define GMB_BLIT_OP_SCALE 3
#define GMB_EV_CACHE_FLUSH_TS 4
#define GMB_EV_PC_CCU_INVALIDATE_DEPTH 24
#define GMB_EV_PC_CCU_INVALIDATE_COLOR 25
#define GMB_EV_PC_CCU_FLUSH_DEPTH_TS 28
#define GMB_EV_PC_CCU_FLUSH_COLOR_TS 29
#define GMB_EV_LABEL 63
#define GMB_EV_TIMESTAMP (1u << 30)

// --- values ------------------------------------------------------------------------------------------------------
#define GMB_FMT6_8_8_8_8_UNORM 0x30u
#define GMB_R2D_UNORM8 0x10u
#define GMB_SWAP_WZYX 0u                          // R8G8B8A8 in memory
#define GMB_SWAP_WXYZ 1u                          // B8G8R8A8 in memory
#define GMB_RB_DBG_ECO_CNTL_BLIT 0x04100000u      // freedreno_devices.py GPUId 610 magic RB_DBG_ECO_CNTL_blit
// RB_CCU_CNTL, sysmem: GMEM_FAST_CLEAR_DISABLE (bit 0), FULL cache sizes, depth offset 0, color offset 0x2000 >> 12
// in bits 23..31
#define GMB_RB_CCU_CNTL_SYSMEM (1u | (static_cast<uint32_t>(0x2000u >> 12) << 23))
#define GMB_BLIT_CNTL(fmt) ((static_cast<uint32_t>(fmt) << 8) | (1u << 7) | (0xfu << 20) | (GMB_R2D_UNORM8 << 24))
#define GMB_SP_2D_DST_FORMAT_VAL(fmt) ((static_cast<uint32_t>(fmt) << 3) | (0xfu << 12))
#define GMB_MAX_COORD 0x3fffu                     // a6xx_reg_xy: 14-bit X and Y
#define GMB_CLEAR_MAX_DWORDS 96u                  // capacity gmb_build_clear requires (60 dwords used today)

struct gmb_clear {
  uint64_t dst;          // surface base the GPU writes (physical with the SMMU in identity, else an IOVA), 64-aligned
  uint32_t pitch;        // bytes per row, multiple of 64
  uint32_t surfW, surfH; // surface size in pixels (bounds check of the rectangle)
  uint32_t x, y, w, h;   // rectangle, pixels
  uint32_t argb;         // 0xAARRGGBB
  bool bgra;             // true: B8G8R8A8 in memory (the guamp GOP surface), false: R8G8B8A8
  bool addrIsGpuVa;      // the SMMU translates and dst/ts/fence are IOVAs the caller mapped
  bool smmuIdentity;     // the M2 decision: stream 0 not translated
  uint64_t ts;           // 16 bytes the CCU flush timestamps may write (scratch), 4-aligned
  uint64_t fence;        // fence word written by the final CACHE_FLUSH_TS after the fill reached memory, 4-aligned
  uint32_t fenceValue;
};

static inline uint32_t gmb_pkt4(uint32_t reg, uint32_t cnt) {
  return 0x40000000u | (cnt & 0x7fu) | (gm2_parity(cnt) << 7) | ((reg & 0x3ffffu) << 8) | (gm2_parity(reg) << 27);
}

// The 32-bit little-endian word one filled pixel holds in memory.
static inline uint32_t gmb_pixel(uint32_t argb, bool bgra) {
  if (bgra) return argb;   // bytes B, G, R, A
  return (argb & 0xff00ff00u) | ((argb >> 16) & 0xffu) | ((argb & 0xffu) << 16);   // bytes R, G, B, A
}

// Why a clear cannot be built (null when it can).
static inline const char *gmb_check(const gmb_clear *c) {
  if (!c->smmuIdentity && !c->addrIsGpuVa) return "smmu translates, no IOVA";
  if (c->dst & 63u) return "dst not 64-byte aligned";
  if (!c->pitch || (c->pitch & 63u) || (c->pitch >> 6) > 0xffffu) return "bad pitch";
  if (c->pitch < c->surfW * 4u) return "pitch below width";
  if (!c->w || !c->h) return "empty rectangle";
  if (c->surfW > GMB_MAX_COORD + 1u || c->surfH > GMB_MAX_COORD + 1u) return "surface too large";
  if (c->x >= c->surfW || c->y >= c->surfH || c->w > c->surfW - c->x || c->h > c->surfH - c->y) return "rectangle out";
  if ((c->ts & 3u) || (c->fence & 3u) || !c->ts || !c->fence) return "bad ts/fence";
  return nullptr;
}

static inline uint32_t gmb_event(uint32_t *r, uint32_t n, uint32_t ev, uint64_t addr, uint32_t value, bool ts) {
  if (!ts) {
    r[n++] = gm2_pkt7(GMB_CP_EVENT_WRITE, 1);
    r[n++] = ev;
    return n;
  }
  r[n++] = gm2_pkt7(GMB_CP_EVENT_WRITE, 4);
  r[n++] = ev | GMB_EV_TIMESTAMP;
  r[n++] = static_cast<uint32_t>(addr);
  r[n++] = static_cast<uint32_t>(addr >> 32);
  r[n++] = value;
  return n;
}

static inline uint32_t gmb_wfi(uint32_t *r, uint32_t n) {
  r[n++] = gm2_pkt7(GMB_CP_WAIT_FOR_IDLE, 0);
  return n;
}

static inline uint32_t gmb_reg(uint32_t *r, uint32_t n, uint32_t reg, uint32_t v) {
  r[n++] = gmb_pkt4(reg, 1);
  r[n++] = v;
  return n;
}

// Builds the fill into ring[0..]: returns the number of dwords, 0 when gmb_check fails or capDwords is too small.
static inline uint32_t gmb_build_clear(uint32_t *ring, size_t capDwords, const gmb_clear *c) {
  if (gmb_check(c) || capDwords < GMB_CLEAR_MAX_DWORDS) return 0;
  uint32_t *r = ring;
  uint32_t n = 0;
  const uint32_t fmt = GMB_FMT6_8_8_8_8_UNORM;
  const uint32_t swap = c->bgra ? GMB_SWAP_WXYZ : GMB_SWAP_WZYX;
  const uint32_t cntl = GMB_BLIT_CNTL(fmt);

  // emit_setup: CCU clean + invalidate, then the sysmem (bypass) CCU layout
  n = gmb_event(r, n, GMB_EV_PC_CCU_FLUSH_COLOR_TS, c->ts, 0, true);
  n = gmb_event(r, n, GMB_EV_PC_CCU_FLUSH_DEPTH_TS, c->ts + 8, 0, true);
  n = gmb_event(r, n, GMB_EV_PC_CCU_INVALIDATE_COLOR, 0, 0, false);
  n = gmb_event(r, n, GMB_EV_PC_CCU_INVALIDATE_DEPTH, 0, 0, false);
  n = gmb_wfi(r, n);
  n = gmb_reg(r, n, GMB_RB_CCU_CNTL, GMB_RB_CCU_CNTL_SYSMEM);
  n = gmb_reg(r, n, GMB_RB_UNKNOWN_8E01, 1);   // fd6_emit_restore magic, A610 value

  // fd6_clear_surface: destination rectangle (inclusive bottom right)
  r[n++] = gmb_pkt4(GMB_GRAS_2D_DST_TL, 2);
  r[n++] = (c->x & GMB_MAX_COORD) | ((c->y & GMB_MAX_COORD) << 16);
  r[n++] = ((c->x + c->w - 1u) & GMB_MAX_COORD) | (((c->y + c->h - 1u) & GMB_MAX_COORD) << 16);

  // emit_clear_color: R, G, B, A unorm8
  r[n++] = gmb_pkt4(GMB_RB_2D_SRC_SOLID_C0, 4);
  r[n++] = (c->argb >> 16) & 0xffu;
  r[n++] = (c->argb >> 8) & 0xffu;
  r[n++] = c->argb & 0xffu;
  r[n++] = c->argb >> 24;

  // emit_blit_setup
  n = gmb_reg(r, n, GMB_RB_2D_BLIT_CNTL, cntl);
  n = gmb_reg(r, n, GMB_GRAS_2D_BLIT_CNTL, cntl);
  n = gmb_reg(r, n, GMB_SP_2D_DST_FORMAT, GMB_SP_2D_DST_FORMAT_VAL(fmt));
  n = gmb_reg(r, n, GMB_RB_2D_UNKNOWN_8C01, 0);

  // emit_blit_dst: one OUT_REG run 0x8c17..0x8c1a
  r[n++] = gmb_pkt4(GMB_RB_2D_DST_INFO, 4);
  r[n++] = fmt | (0u << 8) | (swap << 10);   // TILE6_LINEAR, no UBWC flags, not sRGB
  r[n++] = static_cast<uint32_t>(c->dst);
  r[n++] = static_cast<uint32_t>(c->dst >> 32);
  r[n++] = c->pitch >> 6;

  // emit_blit_fini
  n = gmb_event(r, n, GMB_EV_LABEL, 0, 0, false);
  n = gmb_wfi(r, n);
  n = gmb_reg(r, n, GMB_RB_DBG_ECO_CNTL, GMB_RB_DBG_ECO_CNTL_BLIT);
  r[n++] = gm2_pkt7(GMB_CP_BLIT, 1);
  r[n++] = GMB_BLIT_OP_SCALE;
  n = gmb_wfi(r, n);
  n = gmb_reg(r, n, GMB_RB_DBG_ECO_CNTL, 0);

  // CP_BLIT writes through the CCU: clean it, then the fence after the caches reached memory
  n = gmb_event(r, n, GMB_EV_PC_CCU_FLUSH_COLOR_TS, c->ts, 0, true);
  n = gmb_event(r, n, GMB_EV_CACHE_FLUSH_TS, c->fence, c->fenceValue, true);
  return n;
}

#endif
