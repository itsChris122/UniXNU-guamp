// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Adreno 610: hand-assembled ir3 shaders and the command stream of one rectangle draw in sysmem (direct render)
// mode, linear surfaces, no UBWC, no GMEM, no binning. Two kinds: a solid colour (FS reads c0) and a textured copy (FS
// samples texture 0 at unnormalised coordinates). Base for a layer compositor; not wired into GpuService.inc yet.
// Pure code, no kernel calls, no floating point (the kernel builds with -mgeneral-regs-only). Host-tested by
// tests/gpu_3d_host.cpp. NOT run on the GPU yet: every value below comes from the sources, none from a capture.
//
// Sources (Mesa 25.0.0, read only; no Mesa code is built or linked):
//   ISA       src/freedreno/isa/ir3-cat0.xml (nop, end: OPC bits 55-58, cat 61-63 = 000), ir3-cat1.xml (mov-const:
//             SRC 0-10, DST 32-39, DST_TYPE 46-48, SRC_TYPE 50-52, 53-54 = 01, cat 001), ir3-cat2.xml (bary.f:
//             #instruction-cat2-2src-input, 53-58 = 111001, bit 50 = 0, FULL 52, EI 47), ir3-cat5.xml (sam:
//             54-58 = 00011, FULL 0, SRC1 1-8, SAMP 21-24, TEX 25-31, WRMASK 40-43, TYPE 44-46, cat 101),
//             ir3-common.xml (#reg-gpr = num << 2 | comp, #reg-const the same on 11 bits, #mulitsrc-immed 11-13 = 100).
//             Cross-checked against src/freedreno/ir3/tests/disasm.c: 03000000_00000000 end, 20244000_00000020
//             mov.f32f32 r0.x, c8.x, 47308a02_00002000 (rpt2)bary.f (ei)r0.z, (r)0, r0.x, a0c01f04_0cc00005
//             sam (f32)(xyzw)r1.x, r0.z, s#6, t#6 (tests/gpu_3d_host.cpp re-encodes these four).
//   Delays    src/freedreno/ir3/ir3_delay.c ir3_delayslots: alu -> tex consumer needs delay_slots.non_alu (6) cycles;
//             a tex result read by end is synchronised with (sy).
//   Registers src/freedreno/registers/adreno/a6xx.xml, adreno_pm4.xml (offsets, fields, shr), every offset below is
//             printed from those files by the extractor kept with the notes (analisi/gpu-shader/NOTE.md).
//   Sequence  src/freedreno/vulkan/tu_clear_blit.cc r3d_common (870), r3d_setup (1550), r3d_dst_buffer (1450),
//             r3d_src_common (1107), r3d_run (1661); tu_shader.cc tu6_emit_xs (1239), tu6_emit_vfd_dest (1570),
//             tu6_emit_fs_inputs (1624), tu6_emit_fs_outputs (1813), tu6_emit_vs (1899), tu6_emit_fs (2066);
//             tu_pipeline.cc tu6_emit_xs_config (374), tu6_emit_vpc (798), tu6_emit_vertex_input (2329);
//             tu_cmd_buffer.cc tu6_emit_ccu_cntl (295), tu6_emit_msaa (892), tu6_init_static_regs (1324),
//             tu6_sysmem_render_begin (2093); tu_device.cc ccu offsets (1405).
//   A610      src/freedreno/common/freedreno_devices.py: a6xx_base + a6xx_gen1_low (no fs tex prefetch, no GMEM fast
//             clear, instr_cache_size 64, sysmem CCU 8 KiB depth + 8 KiB colour, num_ccu 1, prim_alloc_threshold 7,
//             no double threadsize), magic_regs of GPUId(610).
// Differences from turnip, by design: vertices come from a vertex buffer (2 vertices, RECTLIST) instead of constants
// selected by the vertex id, so the VS is a bare end; the textured FS uses bary.f + sam since A610 has no FS texture
// prefetch; constants are loaded with SS6_DIRECT; no draw states (CP_SET_DRAW_STATE), plain type-4 writes only.
#ifndef GUAMP_GPU_3D_H
#define GUAMP_GPU_3D_H

#include "GuampGpuM2.h"   // gm2_parity, gm2_pkt7

// ---- ir3 instructions (64 bits, stored low dword first) --------------------------------------------------------------
#define G3D_I(hi, lo) (lo), (hi)
#define G3D_NOP G3D_I(0x00000000u, 0x00000000u)            // cat0 nop
#define G3D_END G3D_I(0x03000000u, 0x00000000u)            // cat0 end, OPC 0110 << 55
#define G3D_SY_END G3D_I(0x13000000u, 0x00000000u)         // (sy)end: SY bit 60, waits for the sam result
// mov.f32f32 r0.<c>, c0.<c>: cat1 (001 << 61), 53-54 = 01 (const src), SRC_TYPE f32 (1 << 50), DST_TYPE f32 (1 << 46)
#define G3D_MOV_C(c) G3D_I(0x20244000u | (c), (c))
// bary.f r0.z, 0, r0.x / (ei)bary.f r0.w, 1, r0.x: cat2 (010 << 61), 53-58 = 111001, FULL (52), SRC1 immediate inloc
// (11-13 = 100), SRC2 = r0.x (the i/j pair written by HLSQ_CONTROL_3_REG.IJ_PERSP_PIXEL = r0.x), EI (47) on the last
#define G3D_BARY_U G3D_I(0x47300002u, 0x00002000u)
#define G3D_BARY_V_EI G3D_I(0x47308003u, 0x00002001u)
// sam (f32)(xyzw)r1.x, r0.z, s#0, t#0: cat5 (101 << 61), opc 00011 << 54, TYPE f32 (1 << 44), WRMASK xyzw (0xf << 40),
// DST r1.x (4), FULL (bit 0), SRC1 r0.z (2 << 1)
#define G3D_SAM_R1 G3D_I(0xa0c01f04u, 0x00000005u)

#define G3D_SHADER_INSTRS 32u                  // one instrlen unit is 16 instructions (128 bytes); 16 more of nops
#define G3D_SHADER_DWORDS (2u * G3D_SHADER_INSTRS)   // cover the instruction prefetch past the end
#define G3D_SHADER_BYTES (4u * G3D_SHADER_DWORDS)
#define G3D_INSTRLEN 1u                        // SP_xS_INSTRLEN, in 16-instruction units

#define G3D_PAD4 G3D_NOP, G3D_NOP, G3D_NOP, G3D_NOP
#define G3D_PAD16 G3D_PAD4, G3D_PAD4, G3D_PAD4, G3D_PAD4

// VS: the vertex fetch writes position to r0.xyzw and the texture coordinate to r1.xy (VFD_DEST_CNTL), and the outputs
// are read from the same registers (SP_VS_OUT_REG), so the program is only end. Footprint r0..r1.
static const uint32_t g3d_vs[G3D_SHADER_DWORDS] = {
  G3D_END, G3D_NOP, G3D_NOP, G3D_NOP, G3D_PAD4, G3D_PAD4, G3D_PAD4, G3D_PAD16,
};
// FS solid: colour from c0 (CP_LOAD_STATE6 constants, 4 floats) to r0.xyzw = SP_FS_OUTPUT_REG(0). Footprint r0.
static const uint32_t g3d_fs_solid[G3D_SHADER_DWORDS] = {
  G3D_MOV_C(0), G3D_MOV_C(1), G3D_MOV_C(2), G3D_MOV_C(3), G3D_END, G3D_NOP, G3D_NOP, G3D_NOP,
  G3D_PAD4, G3D_PAD4, G3D_PAD16,
};
// FS textured: interpolate varying 0 (u at inloc 0, v at inloc 1) into r0.zw, 6 cycles for the alu -> tex delay,
// sample texture/sampler 0 into r1.xyzw = SP_FS_OUTPUT_REG(0), (sy)end. Footprint r0..r1.
static const uint32_t g3d_fs_tex[G3D_SHADER_DWORDS] = {
  G3D_BARY_U, G3D_BARY_V_EI, G3D_NOP, G3D_NOP, G3D_NOP, G3D_NOP, G3D_NOP, G3D_NOP,
  G3D_SAM_R1, G3D_SY_END, G3D_NOP, G3D_NOP, G3D_NOP, G3D_NOP, G3D_NOP, G3D_NOP, G3D_PAD16,
};

// ---- registers (a6xx.xml, A6XX variant) -----------------------------------------------------------------------------
#define G3D_REG_UCHE_UNKNOWN_0E12 0x0e12
#define G3D_REG_UCHE_CLIENT_PF 0x0e19
#define G3D_REG_GRAS_CL_CNTL 0x8000
#define G3D_REG_GRAS_VS_CL_CNTL 0x8001
#define G3D_REG_GRAS_CNTL 0x8005
#define G3D_REG_GRAS_SU_CNTL 0x8090
#define G3D_REG_GRAS_SU_DEPTH_PLANE_CNTL 0x8094
#define G3D_REG_GRAS_SU_CONSERVATIVE_RAS_CNTL 0x8099
#define G3D_REG_GRAS_VS_LAYER_CNTL 0x809b
#define G3D_REG_GRAS_SC_CNTL 0x80a0
#define G3D_REG_GRAS_BIN_CONTROL 0x80a1
#define G3D_REG_GRAS_RAS_MSAA_CNTL 0x80a2              // + GRAS_DEST_MSAA_CNTL 0x80a3
#define G3D_REG_GRAS_UNKNOWN_80AF 0x80af
#define G3D_REG_GRAS_SC_SCREEN_SCISSOR_TL 0x80b0       // + BR 0x80b1
#define G3D_REG_GRAS_SC_VIEWPORT_SCISSOR_TL 0x80d0     // + BR 0x80d1
#define G3D_REG_GRAS_SC_WINDOW_SCISSOR_TL 0x80f0       // + BR 0x80f1
#define G3D_REG_GRAS_LRZ_CNTL 0x8100
#define G3D_REG_GRAS_LRZ_PS_INPUT_CNTL 0x8101
#define G3D_REG_GRAS_LRZ_MRT_BUF_INFO_0 0x8102
#define G3D_REG_GRAS_SAMPLE_CNTL 0x8109
#define G3D_REG_GRAS_UNKNOWN_8110 0x8110
#define G3D_REG_GRAS_SU_DEPTH_CNTL 0x8114
#define G3D_REG_GRAS_SU_STENCIL_CNTL 0x8115
#define G3D_REG_GRAS_2D_RESOLVE_CNTL_1 0x840a          // + _2 0x840b
#define G3D_REG_GRAS_DBG_ECO_CNTL 0x8600
#define G3D_REG_RB_BIN_CONTROL 0x8800
#define G3D_REG_RB_RENDER_CNTL 0x8801
#define G3D_REG_RB_RAS_MSAA_CNTL 0x8802                // + RB_DEST_MSAA_CNTL 0x8803
#define G3D_REG_RB_RENDER_CONTROL0 0x8809              // + RB_RENDER_CONTROL1 0x880a
#define G3D_REG_RB_FS_OUTPUT_CNTL0 0x880b              // + RB_FS_OUTPUT_CNTL1 0x880c
#define G3D_REG_RB_RENDER_COMPONENTS 0x880d
#define G3D_REG_RB_DITHER_CNTL 0x880e
#define G3D_REG_RB_SRGB_CNTL 0x880f
#define G3D_REG_RB_SAMPLE_CNTL 0x8810
#define G3D_REG_RB_UNKNOWN_8811 0x8811
#define G3D_REG_RB_UNKNOWN_8818 0x8818                 // .. 0x881e
#define G3D_REG_RB_MRT0_CONTROL 0x8820                 // RB_MRT[0]: CONTROL +0, BLEND_CONTROL +1, BUF_INFO +2,
#define G3D_REG_RB_MRT0_BUF_INFO 0x8822                //   PITCH +3, ARRAY_PITCH +4, BASE +5/+6, BASE_GMEM +7
#define G3D_REG_RB_ALPHA_CONTROL 0x8864
#define G3D_REG_RB_BLEND_CNTL 0x8865
#define G3D_REG_RB_DEPTH_PLANE_CNTL 0x8870
#define G3D_REG_RB_DEPTH_CNTL 0x8871
#define G3D_REG_RB_STENCIL_CONTROL 0x8880
#define G3D_REG_RB_STENCILREF 0x8887                   // + STENCILMASK 0x8888, STENCILWRMASK 0x8889
#define G3D_REG_RB_WINDOW_OFFSET 0x8890
#define G3D_REG_RB_SAMPLE_COUNT_CONTROL 0x8891
#define G3D_REG_RB_LRZ_CNTL 0x8898
#define G3D_REG_RB_WINDOW_OFFSET2 0x88d4
#define G3D_REG_RB_UNKNOWN_88F0 0x88f0
#define G3D_REG_RB_MRT0_FLAG_BUFFER 0x8903             // ADDR +0/+1, PITCH +2
#define G3D_REG_RB_UNKNOWN_8E01 0x8e01
#define G3D_REG_RB_DBG_ECO_CNTL 0x8e04
#define G3D_REG_RB_CCU_CNTL 0x8e07
#define G3D_REG_VPC_VS_CLIP_CNTL 0x9101
#define G3D_REG_VPC_VS_LAYER_CNTL 0x9104
#define G3D_REG_VPC_UNKNOWN_9107 0x9107
#define G3D_REG_VPC_VARYING_INTERP_MODE0 0x9200
#define G3D_REG_VPC_VARYING_PS_REPL_MODE0 0x9208
#define G3D_REG_VPC_UNKNOWN_9210 0x9210                // + 9211
#define G3D_REG_VPC_VAR_DISABLE0 0x9212                // 4 dwords
#define G3D_REG_VPC_POINT_COORD_INVERT 0x9236
#define G3D_REG_VPC_UNKNOWN_9300 0x9300
#define G3D_REG_VPC_VS_PACK 0x9301
#define G3D_REG_VPC_CNTL_0 0x9304
#define G3D_REG_VPC_SO_DISABLE 0x9306
#define G3D_REG_VPC_DBG_ECO_CNTL 0x9600
#define G3D_REG_VPC_UNKNOWN_9602 0x9602
#define G3D_REG_PC_MODE_CNTL 0x9804
#define G3D_REG_PC_POWER_CNTL 0x9805
#define G3D_REG_PC_PS_CNTL 0x9806
#define G3D_REG_PC_DGEN_SU_CONSERVATIVE_RAS_CNTL 0x980a
#define G3D_REG_PC_RASTER_CNTL 0x9980
#define G3D_REG_PC_PRIMITIVE_CNTL_0 0x9b00
#define G3D_REG_PC_VS_OUT_CNTL 0x9b01
#define G3D_REG_PC_MULTIVIEW_CNTL 0x9b07
#define G3D_REG_PC_UNKNOWN_9E72 0x9e72
#define G3D_REG_VFD_CONTROL_0 0xa000                   // .. VFD_CONTROL_6 0xa006
#define G3D_REG_VFD_MODE_CNTL 0xa007
#define G3D_REG_VFD_MULTIVIEW_CNTL 0xa008
#define G3D_REG_VFD_ADD_OFFSET 0xa009
#define G3D_REG_VFD_INDEX_OFFSET 0xa00e                // + VFD_INSTANCE_START_OFFSET 0xa00f
#define G3D_REG_VFD_FETCH0 0xa010                      // BASE +0/+1, SIZE +2, STRIDE +3
#define G3D_REG_VFD_DECODE0 0xa090                     // INSTR +0, STEP_RATE +1, stride 2
#define G3D_REG_VFD_DEST_CNTL0 0xa0d0                  // stride 1
#define G3D_REG_SP_VS_CTRL_REG0 0xa800
#define G3D_REG_SP_VS_PRIMITIVE_CNTL 0xa802
#define G3D_REG_SP_VS_OUT_REG0 0xa803
#define G3D_REG_SP_VS_VPC_DST_REG0 0xa813
#define G3D_REG_SP_VS_OBJ_FIRST_EXEC_OFFSET 0xa81b     // + OBJ_START 0xa81c/d, PVT_MEM_PARAM 0xa81e,
                                                       //   PVT_MEM_ADDR 0xa81f/20, PVT_MEM_SIZE 0xa821
#define G3D_REG_SP_VS_CONFIG 0xa823
#define G3D_REG_SP_VS_INSTRLEN 0xa824
#define G3D_REG_SP_VS_PVT_MEM_HW_STACK_OFFSET 0xa825
#define G3D_REG_SP_HS_CONFIG 0xa83b
#define G3D_REG_SP_DS_CONFIG 0xa863
#define G3D_REG_SP_GS_CONFIG 0xa894
#define G3D_REG_SP_FS_CTRL_REG0 0xa980
#define G3D_REG_SP_FS_OBJ_FIRST_EXEC_OFFSET 0xa982     // + OBJ_START 0xa983/4, PVT_MEM_PARAM 0xa985,
                                                       //   PVT_MEM_ADDR 0xa986/7, PVT_MEM_SIZE 0xa988
#define G3D_REG_SP_BLEND_CNTL 0xa989
#define G3D_REG_SP_SRGB_CNTL 0xa98a
#define G3D_REG_SP_FS_RENDER_COMPONENTS 0xa98b
#define G3D_REG_SP_FS_OUTPUT_CNTL0 0xa98c              // + SP_FS_OUTPUT_CNTL1 0xa98d
#define G3D_REG_SP_FS_OUTPUT_REG0 0xa98e
#define G3D_REG_SP_FS_MRT_REG0 0xa996
#define G3D_REG_SP_FS_PREFETCH_CNTL 0xa99e
#define G3D_REG_SP_FS_TEX_COUNT 0xa9a7
#define G3D_REG_SP_UNKNOWN_A9A8 0xa9a8
#define G3D_REG_SP_FS_PVT_MEM_HW_STACK_OFFSET 0xa9a9
#define G3D_REG_SP_FS_TEX_SAMP 0xa9e0                  // 64-bit
#define G3D_REG_SP_FS_TEX_CONST 0xa9e4                 // 64-bit
#define G3D_REG_SP_MODE_CONTROL 0xab00
#define G3D_REG_SP_FS_CONFIG 0xab04
#define G3D_REG_SP_FS_INSTRLEN 0xab05
#define G3D_REG_SP_IBO_COUNT 0xab20
#define G3D_REG_SP_DBG_ECO_CNTL 0xae00
#define G3D_REG_SP_CHICKEN_BITS 0xae03
#define G3D_REG_SP_FLOAT_CNTL 0xae04
#define G3D_REG_SP_PERFCTR_ENABLE 0xae0f
#define G3D_REG_SP_PS_TP_BORDER_COLOR_BASE_ADDR 0xb180 // 64-bit
#define G3D_REG_SP_UNKNOWN_B182 0xb182                 // + B183
#define G3D_REG_SP_TP_RAS_MSAA_CNTL 0xb300             // + SP_TP_DEST_MSAA_CNTL 0xb301
#define G3D_REG_SP_TP_BORDER_COLOR_BASE_ADDR 0xb302    // 64-bit
#define G3D_REG_SP_TP_MODE_CNTL 0xb309
#define G3D_REG_TPL1_DBG_ECO_CNTL 0xb600
#define G3D_REG_TPL1_UNKNOWN_B605 0xb605
#define G3D_REG_HLSQ_VS_CNTL 0xb800                    // + HS 0xb801, DS 0xb802, GS 0xb803
#define G3D_REG_HLSQ_FS_CNTL_0 0xb980
#define G3D_REG_HLSQ_CONTROL_1_REG 0xb982              // .. HLSQ_CONTROL_5_REG 0xb986
#define G3D_REG_HLSQ_INVALIDATE_CMD 0xbb08
#define G3D_REG_HLSQ_FS_CNTL 0xbb10
#define G3D_REG_HLSQ_SHARED_CONSTS 0xbb11
#define G3D_REG_HLSQ_UNKNOWN_BE00 0xbe00               // + BE01
#define G3D_REG_HLSQ_DBG_ECO_CNTL 0xbe04

// ---- PM4 (adreno_pm4.xml) ---------------------------------------------------------------------------------------------
#define G3D_CP_NOP 0x10
#define G3D_CP_SKIP_IB2_ENABLE_GLOBAL 0x1d
#define G3D_CP_WAIT_FOR_IDLE 0x26
#define G3D_CP_LOAD_STATE6_GEOM 0x32
#define G3D_CP_LOAD_STATE6_FRAG 0x34
#define G3D_CP_DRAW_INDX_OFFSET 0x38
#define G3D_CP_EVENT_WRITE 0x46
#define G3D_CP_SET_MODE 0x63
#define G3D_CP_SET_VISIBILITY_OVERRIDE 0x64
#define G3D_CP_SET_MARKER 0x65
#define G3D_RM6_DIRECT_RENDER 1u               // CP_SET_MARKER MODE (sysmem)
#define G3D_EV_CACHE_FLUSH_TS 4u               // vgt_event_type
#define G3D_EV_PC_CCU_INVALIDATE_DEPTH 24u
#define G3D_EV_PC_CCU_INVALIDATE_COLOR 25u
#define G3D_EV_PC_CCU_FLUSH_DEPTH_TS 28u
#define G3D_EV_PC_CCU_FLUSH_COLOR_TS 29u
#define G3D_EV_CACHE_INVALIDATE 49u
// CP_LOAD_STATE6 dword 0: DST_OFF 0-13, STATE_TYPE 14-15, STATE_SRC 16-17, STATE_BLOCK 18-21, NUM_UNIT 22-31
#define G3D_ST6_SHADER 0u
#define G3D_ST6_CONSTANTS 1u
#define G3D_SS6_DIRECT 0u
#define G3D_SS6_INDIRECT 2u
#define G3D_SB6_FS_TEX 4u
#define G3D_SB6_VS_SHADER 8u
#define G3D_SB6_FS_SHADER 12u
static inline uint32_t g3d_load_state0(uint32_t off, uint32_t type, uint32_t src, uint32_t block, uint32_t units) {
  return (off & 0x3fffu) | (type & 3u) << 14 | (src & 3u) << 16 | (block & 0xfu) << 18 | (units & 0x3ffu) << 22;
}
// CP_DRAW_INDX_OFFSET dword 0 (vgt_draw_initiator_a4xx): PRIM_TYPE 0-5 (DI_PT_RECTLIST 8), SOURCE_SELECT 6-7
// (DI_SRC_SEL_AUTO_INDEX 2), VIS_CULL 8-9 (IGNORE_VISIBILITY 0)
#define G3D_DRAW_RECTLIST_AUTO 0x88u

// Formats (a6xx_format, a3xx_color_swap): BGRA8888 in memory = FMT6_8_8_8_8_UNORM with swap WXYZ, RGBA8888 = WZYX.
#define G3D_FMT6_8_8_8_8_UNORM 0x30u
#define G3D_FMT6_32_32_FLOAT 0x67u
#define G3D_FMT6_32_32_32_32_FLOAT 0x82u
#define G3D_SWAP_WZYX 0u
#define G3D_SWAP_WXYZ 1u
#define G3D_REGID_NONE 0xfcu                   // regid(63, 0): no register

// ---- inputs ----------------------------------------------------------------------------------------------------------
enum g3d_kind { G3D_SOLID = 0, G3D_TEXTURED = 1 };

// GPU addresses (identity through the GPU SMMU, M3) of the memory the owner allocates and fills.
struct g3d_mem {
  uint64_t vs, fs;      // G3D_SHADER_BYTES each, 128-byte aligned: copies of g3d_vs and g3d_fs_solid / g3d_fs_tex
  uint64_t vbo;         // G3D_VBO_DWORDS dwords from g3d_fill_vertices, 16-byte aligned
  uint64_t tex;         // textured only: G3D_TEX_DWORDS dwords from g3d_fill_texture, 64-byte aligned
  uint64_t bcolor;      // 128 zero bytes, 128-byte aligned (border colour table, init only)
  uint64_t flush;       // 8 bytes the CCU flush timestamp is written to
};
struct g3d_surface {
  uint64_t base;        // 64-byte aligned (render target), 32-byte aligned (texture)
  uint32_t pitch;       // bytes per row, multiple of 64
  uint32_t width, height;
  uint32_t fmt, swap;   // G3D_FMT6_*, G3D_SWAP_*
};
struct g3d_draw {
  unsigned kind;        // g3d_kind
  g3d_surface dst;
  g3d_surface src;      // textured
  uint32_t x0, y0, x1, y1;          // destination rectangle in pixels, x1/y1 exclusive
  uint32_t u0, v0;      // textured: source texel of the top-left corner (unnormalised coordinates, same size copy)
  uint32_t color[4];    // solid: R, G, B, A as IEEE single-precision bit patterns (g3d_f32 / g3d_unorm8)
  bool linear;          // textured: bilinear instead of nearest
  uint32_t seqno;       // value written to mem.flush by the CCU flush
};

// IEEE-754 single bits of a small unsigned integer, no FPU (n < 2^24 is exact).
static inline uint32_t g3d_f32(uint32_t n) {
  if (!n) return 0;
  uint32_t e = 31;
  while (!(n >> e)) e = e - 1;
  const uint32_t mant = e > 23 ? n >> (e - 23) : n << (23 - e);
  return (127u + e) << 23 | (mant & 0x7fffffu);
}
// v / 255 for a colour channel, rounded to nearest: built from the exact quotient bits, no FPU.
static inline uint32_t g3d_unorm8(uint32_t v) {
  if (!v) return 0;
  if (v >= 255) return 0x3f800000u;
  // v / 255 in [1/255, 254/255]: exponent e with 2^e <= v/255 < 2^(e+1), e in -8..-1
  int e = -1;
  while ((v << (-e)) < 255u) e = e - 1;          // v * 2^-e >= 255
  const uint64_t num = static_cast<uint64_t>(v) << (23 - e + 1);   // (v / 255) * 2^(23 - e), one guard bit
  uint64_t q = num / 255u;
  q = (q + 1) >> 1;                              // round half up on the guard bit
  if (q >> 24) { q >>= 1; e = e + 1; }
  return static_cast<uint32_t>(127 + e) << 23 | (static_cast<uint32_t>(q) & 0x7fffffu);
}

// Vertex buffer: two RECTLIST vertices (top-left, bottom-right) of 6 floats each: x, y, z = 0, w = 1, u, v.
// GRAS_CL_CNTL disables the viewport transform and the perspective division, so x/y are window pixels.
#define G3D_VBO_DWORDS 12u
#define G3D_VBO_STRIDE 24u
static inline void g3d_fill_vertices(uint32_t *v, const g3d_draw *d) {
  const uint32_t one = 0x3f800000u;
  const uint32_t w = d->x1 - d->x0, h = d->y1 - d->y0;
  v[0] = g3d_f32(d->x0); v[1] = g3d_f32(d->y0); v[2] = 0; v[3] = one;
  v[4] = g3d_f32(d->u0); v[5] = g3d_f32(d->v0);
  v[6] = g3d_f32(d->x1); v[7] = g3d_f32(d->y1); v[8] = 0; v[9] = one;
  v[10] = g3d_f32(d->u0 + w); v[11] = g3d_f32(d->v0 + h);
}

// Texture descriptor (A6XX_TEX_CONST, 16 dwords) + sampler (A6XX_TEX_SAMP, 4 dwords) for one linear 2D surface.
#define G3D_TEX_CONST_DWORDS 16u
#define G3D_TEX_DWORDS (G3D_TEX_CONST_DWORDS + 4u)
static inline void g3d_fill_texture(uint32_t *t, const g3d_surface *s, bool linear) {
  for (unsigned i = 0; i < G3D_TEX_DWORDS; i = i + 1) t[i] = 0;
  // 0: TILE_MODE 0-1 (linear), SWIZ_X/Y/Z/W 4-6/7-9/10-12/13-15 = X Y Z W, MIPLVLS 16-19 = 0, FMT 22-29, SWAP 30-31
  t[0] = 0u << 4 | 1u << 7 | 2u << 10 | 3u << 13 | (s->fmt & 0xffu) << 22 | (s->swap & 3u) << 30;
  t[1] = (s->width & 0x7fffu) | (s->height & 0x7fffu) << 15;            // WIDTH 0-14, HEIGHT 15-29
  t[2] = (s->pitch & 0x3fffffu) << 7 | 1u << 29;                        // PITCH 7-28 (bytes), TYPE 29-31 = A6XX_TEX_2D
  t[3] = 0;                                                             // ARRAY_PITCH: one layer
  t[4] = static_cast<uint32_t>(s->base);                                // BASE_LO 5-31 (raw address, 32-byte aligned)
  t[5] = static_cast<uint32_t>(s->base >> 32) & 0x1ffffu;               // BASE_HI 0-16
  t[5] |= 1u << 17;                                                     // DEPTH 17-29 = 1
  // sampler 0: MIPFILTER_LINEAR_NEAR 0, XY_MAG 1-2, XY_MIN 3-4 (A6XX_TEX_NEAREST 0 / LINEAR 1), WRAP_S/T/R 5-7/8-10/
  // 11-13 = A6XX_TEX_CLAMP_TO_EDGE 1, 0x60000 as the blob (tu_clear_blit.cc r3d_src_common)
  const uint32_t f = linear ? 1u : 0u;
  t[16] = f << 1 | f << 3 | 1u << 5 | 1u << 8 | 1u << 11 | 0x60000u;
  t[17] = 1u << 5 | 1u << 6;                                            // UNNORM_COORDS, MIPFILTER_LINEAR_FAR
}

// ---- command stream writer --------------------------------------------------------------------------------------------
struct g3d_cs { uint32_t *p; size_t n, cap; bool overflow; };
static inline void g3d_emit(g3d_cs *cs, uint32_t v) {
  if (cs->n < cs->cap) cs->p[cs->n] = v; else cs->overflow = true;
  cs->n = cs->n + 1;
}
// type-4 header: count 0-6, odd parity of count 7, register 8-25, odd parity of register 27 (Mesa pm4_pkt4_hdr)
static inline uint32_t g3d_pkt4(uint32_t reg, uint32_t cnt) {
  return 0x40000000u | (cnt & 0x7fu) | gm2_parity(cnt) << 7 | (reg & 0x3ffffu) << 8 | gm2_parity(reg) << 27;
}
static inline void g3d_reg(g3d_cs *cs, uint32_t reg, uint32_t v) { g3d_emit(cs, g3d_pkt4(reg, 1)); g3d_emit(cs, v); }
static inline void g3d_regs(g3d_cs *cs, uint32_t reg, const uint32_t *v, uint32_t cnt) {
  g3d_emit(cs, g3d_pkt4(reg, cnt));
  for (uint32_t i = 0; i < cnt; i = i + 1) g3d_emit(cs, v[i]);
}
static inline void g3d_reg64(g3d_cs *cs, uint32_t reg, uint64_t v) {
  const uint32_t w[2] = {static_cast<uint32_t>(v), static_cast<uint32_t>(v >> 32)};
  g3d_regs(cs, reg, w, 2);
}
static inline void g3d_pkt7(g3d_cs *cs, uint32_t op, uint32_t cnt) { g3d_emit(cs, gm2_pkt7(op, cnt)); }
static inline void g3d_event(g3d_cs *cs, uint32_t ev) { g3d_pkt7(cs, G3D_CP_EVENT_WRITE, 1); g3d_emit(cs, ev); }
static inline void g3d_event_ts(g3d_cs *cs, uint32_t ev, uint64_t addr, uint32_t value) {
  g3d_pkt7(cs, G3D_CP_EVENT_WRITE, 4);
  g3d_emit(cs, ev);
  g3d_emit(cs, static_cast<uint32_t>(addr));
  g3d_emit(cs, static_cast<uint32_t>(addr >> 32));
  g3d_emit(cs, value);
}
static inline void g3d_wfi(g3d_cs *cs) { g3d_pkt7(cs, G3D_CP_WAIT_FOR_IDLE, 0); }

// ---- static state, once after the CP start (tu6_init_static_regs, A6XX branch, A610 magic values) --------------------
static inline void g3d_emit_init(g3d_cs *cs, const g3d_mem *m) {
  g3d_reg(cs, G3D_REG_RB_DBG_ECO_CNTL, 0x04100000u);
  g3d_reg(cs, G3D_REG_SP_FLOAT_CNTL, 0);
  g3d_reg(cs, G3D_REG_SP_DBG_ECO_CNTL, 0);
  g3d_reg(cs, G3D_REG_SP_PERFCTR_ENABLE, 0x3f);
  g3d_reg(cs, G3D_REG_TPL1_UNKNOWN_B605, 0x44);
  g3d_reg(cs, G3D_REG_TPL1_DBG_ECO_CNTL, 0);
  g3d_reg(cs, G3D_REG_HLSQ_UNKNOWN_BE00, 0x80);
  g3d_reg(cs, G3D_REG_HLSQ_UNKNOWN_BE00 + 1, 0);
  g3d_reg(cs, G3D_REG_VPC_DBG_ECO_CNTL, 0);
  g3d_reg(cs, G3D_REG_GRAS_DBG_ECO_CNTL, 0);
  g3d_reg(cs, G3D_REG_HLSQ_DBG_ECO_CNTL, 0);
  g3d_reg(cs, G3D_REG_SP_CHICKEN_BITS, 0);
  g3d_reg(cs, G3D_REG_SP_IBO_COUNT, 0);
  g3d_reg(cs, G3D_REG_SP_UNKNOWN_B182, 0);
  g3d_reg(cs, G3D_REG_HLSQ_SHARED_CONSTS, 0);
  g3d_reg(cs, G3D_REG_UCHE_UNKNOWN_0E12, 0x10000000u);
  g3d_reg(cs, G3D_REG_UCHE_CLIENT_PF, 0x00000004u);
  g3d_reg(cs, G3D_REG_RB_UNKNOWN_8E01, 0x00000001u);
  g3d_reg(cs, G3D_REG_SP_UNKNOWN_A9A8, 0);
  g3d_reg(cs, G3D_REG_SP_MODE_CONTROL, 1u | 2u << 1);            // CONSTANT_DEMOTION_ENABLE, ISAMMODE_GL
  g3d_reg(cs, G3D_REG_VFD_ADD_OFFSET, 1u);                       // VERTEX
  g3d_reg(cs, G3D_REG_RB_UNKNOWN_8811, 0x00000010u);
  g3d_reg(cs, G3D_REG_PC_MODE_CNTL, 0xfu);
  g3d_reg(cs, G3D_REG_PC_POWER_CNTL, 0);
  g3d_reg(cs, G3D_REG_GRAS_UNKNOWN_8110, 0);
  for (uint32_t r = G3D_REG_RB_UNKNOWN_8818; r <= G3D_REG_RB_UNKNOWN_8818 + 6; r = r + 1) g3d_reg(cs, r, 0);
  g3d_reg(cs, G3D_REG_RB_UNKNOWN_88F0, 0);
  g3d_reg(cs, G3D_REG_VPC_POINT_COORD_INVERT, 0);
  g3d_reg(cs, G3D_REG_VPC_UNKNOWN_9300, 0);
  g3d_reg(cs, G3D_REG_VPC_SO_DISABLE, 1u);
  g3d_reg(cs, G3D_REG_SP_UNKNOWN_B182 + 1, 0);
  g3d_reg(cs, G3D_REG_GRAS_UNKNOWN_80AF, 0);
  g3d_reg(cs, G3D_REG_GRAS_SU_CONSERVATIVE_RAS_CNTL, 0);
  g3d_reg(cs, G3D_REG_PC_DGEN_SU_CONSERVATIVE_RAS_CNTL, 0);
  g3d_reg(cs, G3D_REG_VPC_UNKNOWN_9210, 0);
  g3d_reg(cs, G3D_REG_VPC_UNKNOWN_9210 + 1, 0);
  g3d_reg(cs, G3D_REG_VPC_UNKNOWN_9602, 0);
  g3d_reg(cs, G3D_REG_PC_UNKNOWN_9E72, 0);
  g3d_reg(cs, G3D_REG_SP_TP_MODE_CNTL, 0x000000a0u | 2u);        // ISAMMODE_GL
  g3d_reg(cs, G3D_REG_HLSQ_CONTROL_1_REG + 4, G3D_REGID_NONE);  // HLSQ_CONTROL_5_REG
  g3d_reg(cs, G3D_REG_VFD_MODE_CNTL, 0);
  g3d_reg(cs, G3D_REG_RB_ALPHA_CONTROL, 0);
  g3d_reg64(cs, G3D_REG_SP_TP_BORDER_COLOR_BASE_ADDR, m->bcolor);
  g3d_reg64(cs, G3D_REG_SP_PS_TP_BORDER_COLOR_BASE_ADDR, m->bcolor);
  g3d_reg(cs, G3D_REG_SP_FS_CTRL_REG0, 0);                       // no leftover early preamble
  g3d_wfi(cs);
}

// ---- one rectangle --------------------------------------------------------------------------------------------------
// Register linkage (tu6_emit_vpc): the FS varyings first (u, v at locations 0-1, only when textured), then the
// position (4 components). SP_VS_OUT_REG halves: REGID 0-7, COMPMASK 8-11 (A) / 16-23, 24-27 (B).
static inline void g3d_emit_draw(g3d_cs *cs, const g3d_mem *m, const g3d_draw *d) {
  const bool tex = d->kind == G3D_TEXTURED;
  const uint32_t w = d->dst.width, h = d->dst.height;

  // sysmem render begin (tu6_sysmem_render_begin): window, bin control, marker, CCU in sysmem mode
  const uint32_t win[2] = {0, ((h - 1) & 0x3fffu) << 16 | ((w - 1) & 0x3fffu)};
  g3d_regs(cs, G3D_REG_GRAS_SC_WINDOW_SCISSOR_TL, win, 2);
  g3d_regs(cs, G3D_REG_GRAS_2D_RESOLVE_CNTL_1, win, 2);
  g3d_reg(cs, G3D_REG_RB_WINDOW_OFFSET, 0);
  g3d_reg(cs, G3D_REG_RB_WINDOW_OFFSET2, 0);
  // GRAS/RB_BIN_CONTROL: RENDER_MODE RENDERING_PASS 0, FORCE_LRZ_WRITE_DIS (no LRZ feedback on A610), BUFFERS_IN_SYSMEM
  g3d_reg(cs, G3D_REG_GRAS_BIN_CONTROL, 1u << 21 | 3u << 22);
  g3d_reg(cs, G3D_REG_RB_BIN_CONTROL, 1u << 21 | 3u << 22);
  g3d_pkt7(cs, G3D_CP_SET_MARKER, 1);
  g3d_emit(cs, G3D_RM6_DIRECT_RENDER);
  g3d_pkt7(cs, G3D_CP_SKIP_IB2_ENABLE_GLOBAL, 1);
  g3d_emit(cs, 0);
  g3d_event(cs, G3D_EV_PC_CCU_INVALIDATE_COLOR);
  g3d_event(cs, G3D_EV_PC_CCU_INVALIDATE_DEPTH);
  g3d_wfi(cs);
  // RB_CCU_CNTL sysmem: GMEM_FAST_CLEAR_DISABLE (A610), colour cache full at GMEM offset 8 KiB after the 8 KiB depth
  // cache (COLOR_OFFSET 23-31 shr 12 = 2), depth offset 0
  g3d_reg(cs, G3D_REG_RB_CCU_CNTL, 1u | (0x2000u >> 12) << 23);
  g3d_pkt7(cs, G3D_CP_SET_VISIBILITY_OVERRIDE, 1);
  g3d_emit(cs, 1);
  g3d_pkt7(cs, G3D_CP_SET_MODE, 1);
  g3d_emit(cs, 0);
  g3d_reg(cs, G3D_REG_VFD_MODE_CNTL, 0);

  // r3d_common: invalidate the shader state of every stage, VS + FS enabled, HS/DS/GS off
  g3d_reg(cs, G3D_REG_HLSQ_INVALIDATE_CMD, 0x1fu | 1u << 7 | 1u << 8 | 0x1fu << 9 | 0x1fu << 14);
  g3d_reg(cs, G3D_REG_SP_VS_CONFIG, 1u << 8);                    // ENABLED
  g3d_reg(cs, G3D_REG_HLSQ_VS_CNTL, 1u << 8);                    // ENABLED, CONSTLEN 0
  g3d_reg(cs, G3D_REG_SP_HS_CONFIG, 0);
  g3d_reg(cs, G3D_REG_HLSQ_VS_CNTL + 1, 0);
  g3d_reg(cs, G3D_REG_SP_DS_CONFIG, 0);
  g3d_reg(cs, G3D_REG_HLSQ_VS_CNTL + 2, 0);
  g3d_reg(cs, G3D_REG_SP_GS_CONFIG, 0);
  g3d_reg(cs, G3D_REG_HLSQ_VS_CNTL + 3, 0);
  // SP_FS_CONFIG: ENABLED, NTEX 9-16 / NSAMP 17-21 = 1 when textured; HLSQ_FS_CNTL: CONSTLEN (vec4 >> 2) 1 for c0..c3
  g3d_reg(cs, G3D_REG_SP_FS_CONFIG, 1u << 8 | (tex ? (1u << 9 | 1u << 17) : 0));
  g3d_reg(cs, G3D_REG_HLSQ_FS_CNTL, 1u << 8 | (tex ? 0 : 1u));

  // tu6_emit_xs VS: footprint r0..r1 (FULLREGFOOTPRINT 7-12 = 2), MERGEDREGS 20
  g3d_reg(cs, G3D_REG_SP_VS_CTRL_REG0, 2u << 7 | 1u << 20);
  g3d_reg(cs, G3D_REG_SP_VS_INSTRLEN, G3D_INSTRLEN);
  const uint32_t vsObj[7] = {0, static_cast<uint32_t>(m->vs), static_cast<uint32_t>(m->vs >> 32), 0, 0, 0, 0};
  g3d_regs(cs, G3D_REG_SP_VS_OBJ_FIRST_EXEC_OFFSET, vsObj, 7);   // no private memory
  g3d_reg(cs, G3D_REG_SP_VS_PVT_MEM_HW_STACK_OFFSET, 0);
  g3d_pkt7(cs, G3D_CP_LOAD_STATE6_GEOM, 3);
  g3d_emit(cs, g3d_load_state0(0, G3D_ST6_SHADER, G3D_SS6_INDIRECT, G3D_SB6_VS_SHADER, G3D_INSTRLEN));
  g3d_emit(cs, static_cast<uint32_t>(m->vs));
  g3d_emit(cs, static_cast<uint32_t>(m->vs >> 32));
  // tu6_emit_xs FS: footprint r0 (solid) / r0..r1 (textured), THREADSIZE 64 (20 = 0), VARYING 22, LODPIXMASK 23 (sam
  // with implicit lod), UNK24 as turnip, MERGEDREGS 31
  g3d_reg(cs, G3D_REG_SP_FS_CTRL_REG0, (tex ? 2u : 1u) << 7 | (tex ? (1u << 22 | 1u << 23) : 0) | 1u << 24 | 1u << 31);
  g3d_reg(cs, G3D_REG_SP_FS_INSTRLEN, G3D_INSTRLEN);
  const uint32_t fsObj[7] = {0, static_cast<uint32_t>(m->fs), static_cast<uint32_t>(m->fs >> 32), 0, 0, 0, 0};
  g3d_regs(cs, G3D_REG_SP_FS_OBJ_FIRST_EXEC_OFFSET, fsObj, 7);
  g3d_reg(cs, G3D_REG_SP_FS_PVT_MEM_HW_STACK_OFFSET, 0);
  g3d_pkt7(cs, G3D_CP_LOAD_STATE6_FRAG, 3);
  g3d_emit(cs, g3d_load_state0(0, G3D_ST6_SHADER, G3D_SS6_INDIRECT, G3D_SB6_FS_SHADER, G3D_INSTRLEN));
  g3d_emit(cs, static_cast<uint32_t>(m->fs));
  g3d_emit(cs, static_cast<uint32_t>(m->fs >> 32));

  g3d_reg(cs, G3D_REG_PC_PRIMITIVE_CNTL_0, 0);

  // tu6_emit_vpc
  const uint32_t outCnt = tex ? 2u : 1u, posLoc = tex ? 2u : 0u, stride = posLoc + 4u;
  g3d_reg(cs, G3D_REG_SP_VS_OUT_REG0, tex ? (4u | 0x3u << 8 | 0u << 16 | 0xfu << 24) : (0u | 0xfu << 8));
  g3d_reg(cs, G3D_REG_SP_VS_VPC_DST_REG0, tex ? (0u | posLoc << 8) : 0u);           // OUTLOC0/1
  g3d_reg(cs, G3D_REG_VPC_VS_PACK, stride | posLoc << 8 | 0xffu << 16);             // PSIZELOC none
  g3d_reg(cs, G3D_REG_VPC_VS_CLIP_CNTL, 0xffu << 8 | 0xffu << 16);                  // no clip distances
  g3d_reg(cs, G3D_REG_GRAS_VS_CL_CNTL, 0);
  g3d_reg(cs, G3D_REG_PC_VS_OUT_CNTL, stride);
  g3d_reg(cs, G3D_REG_SP_VS_PRIMITIVE_CNTL, outCnt | G3D_REGID_NONE << 6);
  g3d_reg(cs, G3D_REG_VPC_VS_LAYER_CNTL, 0xffu | 0xffu << 8);
  g3d_reg(cs, G3D_REG_GRAS_VS_LAYER_CNTL, 0);
  // r3d_common: smooth interpolation, RECTLIST replacement of components 0/1 of location 0 with S/T across the rect
  g3d_reg(cs, G3D_REG_VPC_VARYING_INTERP_MODE0, 0);
  g3d_reg(cs, G3D_REG_VPC_VARYING_PS_REPL_MODE0, tex ? (2u << 2 | 1u) : 0u);

  // tu6_emit_vs: no multiview, vertex fetch into r0.xyzw (position) and r1.xy (texture coordinate)
  g3d_reg(cs, G3D_REG_PC_MULTIVIEW_CNTL, 0);
  g3d_reg(cs, G3D_REG_VFD_MULTIVIEW_CNTL, 0);
  g3d_reg(cs, G3D_REG_VFD_CONTROL_0, 2u | 2u << 8);                                // FETCH_CNT, DECODE_CNT
  const uint32_t dest[2] = {0xfu | 0u << 4, 0x3u | 4u << 4};                        // WRITEMASK 0-3, REGID 4-11
  g3d_regs(cs, G3D_REG_VFD_DEST_CNTL0, dest, 2);
  const uint32_t vfdCtl[6] = {0xfcfcfcfcu, 0x0000fcfcu, 0xfcfcfcfcu, 0x000000fcu, 0x0000fcfcu, 0};   // CONTROL_1..6
  g3d_regs(cs, G3D_REG_VFD_CONTROL_0 + 1, vfdCtl, 6);
  // tu6_emit_vertex_input: binding 0, attribute 0 = 4 floats at 0, attribute 1 = 2 floats at 16; UNK30, FLOAT
  const uint32_t decode[4] = {
    0u | 0u << 5 | G3D_FMT6_32_32_32_32_FLOAT << 20 | G3D_SWAP_WZYX << 28 | 1u << 30 | 1u << 31, 0,
    0u | 16u << 5 | G3D_FMT6_32_32_FLOAT << 20 | G3D_SWAP_WZYX << 28 | 1u << 30 | 1u << 31, 0};
  g3d_regs(cs, G3D_REG_VFD_DECODE0, decode, 4);
  const uint32_t fetch[4] = {static_cast<uint32_t>(m->vbo), static_cast<uint32_t>(m->vbo >> 32),
                             4u * G3D_VBO_DWORDS, G3D_VBO_STRIDE};
  g3d_regs(cs, G3D_REG_VFD_FETCH0, fetch, 4);
  const uint32_t vfdOff[2] = {0, 0};
  g3d_regs(cs, G3D_REG_VFD_INDEX_OFFSET, vfdOff, 2);

  // tu6_emit_fs: inputs
  g3d_reg(cs, G3D_REG_PC_PS_CNTL, 0);
  g3d_reg(cs, G3D_REG_SP_FS_PREFETCH_CNTL, tex ? 0u : 1u << 3);                     // IJ_WRITE_DISABLE when unused
  const uint32_t hlsq[4] = {0x7u,                                                   // PRIMALLOCTHRESHOLD
                            0xfcfcfcfcu,                                            // face, sampleid, mask, rhw
                            tex ? 0xfcfcfc00u : 0xfcfcfcfcu,                        // IJ_PERSP_PIXEL = r0.x
                            0xfcfcfcfcu};
  g3d_regs(cs, G3D_REG_HLSQ_CONTROL_1_REG, hlsq, 4);
  g3d_reg(cs, G3D_REG_HLSQ_CONTROL_1_REG + 4, G3D_REGID_NONE | G3D_REGID_NONE << 8);
  g3d_reg(cs, G3D_REG_HLSQ_FS_CNTL_0, tex ? 1u << 1 : 0u);                          // THREAD64, VARYINGS
  g3d_reg(cs, G3D_REG_GRAS_CNTL, tex ? 1u : 0u);                                     // IJ_PERSP_PIXEL
  const uint32_t rctl[2] = {tex ? (1u | 1u << 10) : 0u, 0};                          // + UNK10 with varyings
  g3d_regs(cs, G3D_REG_RB_RENDER_CONTROL0, rctl, 2);
  g3d_reg(cs, G3D_REG_RB_SAMPLE_CNTL, 0);
  g3d_reg(cs, G3D_REG_GRAS_LRZ_PS_INPUT_CNTL, 0);
  g3d_reg(cs, G3D_REG_GRAS_SAMPLE_CNTL, 0);
  const uint32_t varDis[4] = {tex ? ~0x3u : ~0u, ~0u, ~0u, ~0u};
  g3d_regs(cs, G3D_REG_VPC_VAR_DISABLE0, varDis, 4);
  g3d_reg(cs, G3D_REG_VPC_CNTL_0, (tex ? 2u | 1u << 16 : 0u) | 0xffu << 8 | 0xffu << 24);   // NUMNONPOSVAR, VARYING
  // outputs: colour 0 from r0.x (solid) / r1.x (textured), no depth / sample mask / stencil output
  g3d_reg(cs, G3D_REG_SP_FS_OUTPUT_CNTL0, G3D_REGID_NONE << 8 | G3D_REGID_NONE << 16 | G3D_REGID_NONE << 24);
  g3d_reg(cs, G3D_REG_SP_FS_OUTPUT_REG0, tex ? 4u : 0u);
  g3d_reg(cs, G3D_REG_SP_FS_RENDER_COMPONENTS, 0xfu);
  g3d_reg(cs, G3D_REG_RB_FS_OUTPUT_CNTL0, 0);
  g3d_reg(cs, G3D_REG_RB_RENDER_COMPONENTS, 0xfu);

  // r3d_common: no clipping, no viewport transform, no perspective division (window coordinates)
  g3d_reg(cs, G3D_REG_GRAS_CL_CNTL, 1u | 1u << 7 | 1u << 8 | 1u << 9);
  g3d_reg(cs, G3D_REG_GRAS_SU_CNTL, 0);
  g3d_reg(cs, G3D_REG_PC_RASTER_CNTL, 0);
  g3d_reg(cs, G3D_REG_VPC_UNKNOWN_9107, 0);
  const uint32_t sc[2] = {0, 0x7fffu | 0x7fffu << 16};
  g3d_regs(cs, G3D_REG_GRAS_SC_VIEWPORT_SCISSOR_TL, sc, 2);
  g3d_regs(cs, G3D_REG_GRAS_SC_SCREEN_SCISSOR_TL, sc, 2);
  // tu6_emit_msaa, one sample: SAMPLES MSAA_ONE 0, MSAA_DISABLE on the destination registers
  const uint32_t msaa[2] = {0, 1u << 2};
  g3d_regs(cs, G3D_REG_SP_TP_RAS_MSAA_CNTL, msaa, 2);
  g3d_regs(cs, G3D_REG_GRAS_RAS_MSAA_CNTL, msaa, 2);
  g3d_regs(cs, G3D_REG_RB_RAS_MSAA_CNTL, msaa, 2);

  // r3d_setup: one render target, no blend / depth / stencil / LRZ / sRGB / dither
  g3d_reg(cs, G3D_REG_SP_FS_OUTPUT_CNTL0 + 1, 1u);                // SP_FS_OUTPUT_CNTL1 MRT 1
  g3d_reg(cs, G3D_REG_RB_FS_OUTPUT_CNTL0 + 1, 1u);                // RB_FS_OUTPUT_CNTL1 MRT 1
  g3d_reg(cs, G3D_REG_SP_BLEND_CNTL, 0);
  g3d_reg(cs, G3D_REG_RB_BLEND_CNTL, 0xffffu << 16);              // SAMPLE_MASK
  g3d_reg(cs, G3D_REG_RB_DEPTH_PLANE_CNTL, 0);
  g3d_reg(cs, G3D_REG_RB_DEPTH_CNTL, 0);
  g3d_reg(cs, G3D_REG_GRAS_SU_DEPTH_CNTL, 0);
  g3d_reg(cs, G3D_REG_GRAS_SU_DEPTH_PLANE_CNTL, 0);
  g3d_reg(cs, G3D_REG_RB_STENCIL_CONTROL, 0);
  g3d_reg(cs, G3D_REG_GRAS_SU_STENCIL_CNTL, 0);
  const uint32_t stencil[3] = {0, 0, 0};                          // STENCILREF, STENCILMASK, STENCILWRMASK
  g3d_regs(cs, G3D_REG_RB_STENCILREF, stencil, 3);
  g3d_reg(cs, G3D_REG_SP_FS_MRT_REG0, d->dst.fmt & 0xffu);
  g3d_reg(cs, G3D_REG_RB_MRT0_CONTROL, 0xfu << 7);                // COMPONENT_ENABLE rgba
  g3d_reg(cs, G3D_REG_RB_SRGB_CNTL, 0);
  g3d_reg(cs, G3D_REG_SP_SRGB_CNTL, 0);
  g3d_reg(cs, G3D_REG_GRAS_LRZ_CNTL, 0);
  g3d_reg(cs, G3D_REG_RB_LRZ_CNTL, 0);
  g3d_reg(cs, G3D_REG_GRAS_SC_CNTL, 2u);                          // CCUSINGLECACHELINESIZE 2
  g3d_reg(cs, G3D_REG_RB_SAMPLE_COUNT_CONTROL, 1u);               // DISABLE
  g3d_reg(cs, G3D_REG_RB_DITHER_CNTL, 0);

  // r3d_dst_buffer: linear colour buffer. BUF_INFO: COLOR_FORMAT 0-7, TILE_MODE 8-9 linear, COLOR_SWAP 13-14;
  // PITCH bytes >> 6; ARRAY_PITCH 0; BASE; BASE_GMEM 0
  const uint32_t mrt[6] = {(d->dst.fmt & 0xffu) | (d->dst.swap & 3u) << 13, d->dst.pitch >> 6, 0,
                           static_cast<uint32_t>(d->dst.base), static_cast<uint32_t>(d->dst.base >> 32), 0};
  g3d_regs(cs, G3D_REG_RB_MRT0_BUF_INFO, mrt, 6);
  const uint32_t flag[3] = {0, 0, 0};
  g3d_regs(cs, G3D_REG_RB_MRT0_FLAG_BUFFER, flag, 3);
  g3d_reg(cs, G3D_REG_GRAS_LRZ_MRT_BUF_INFO_0, d->dst.fmt & 0xffu);
  g3d_reg(cs, G3D_REG_RB_RENDER_CNTL, 2u << 3);                   // CCUSINGLECACHELINESIZE 2, no UBWC

  if (tex) {
    // r3d_src_common: sampler, then texture descriptor, both indirect from mem.tex
    g3d_pkt7(cs, G3D_CP_LOAD_STATE6_FRAG, 3);
    g3d_emit(cs, g3d_load_state0(0, G3D_ST6_SHADER, G3D_SS6_INDIRECT, G3D_SB6_FS_TEX, 1));
    const uint64_t samp = m->tex + 4u * G3D_TEX_CONST_DWORDS;
    g3d_emit(cs, static_cast<uint32_t>(samp));
    g3d_emit(cs, static_cast<uint32_t>(samp >> 32));
    g3d_reg64(cs, G3D_REG_SP_FS_TEX_SAMP, samp);
    g3d_pkt7(cs, G3D_CP_LOAD_STATE6_FRAG, 3);
    g3d_emit(cs, g3d_load_state0(0, G3D_ST6_CONSTANTS, G3D_SS6_INDIRECT, G3D_SB6_FS_TEX, 1));
    g3d_emit(cs, static_cast<uint32_t>(m->tex));
    g3d_emit(cs, static_cast<uint32_t>(m->tex >> 32));
    g3d_reg64(cs, G3D_REG_SP_FS_TEX_CONST, m->tex);
    g3d_reg(cs, G3D_REG_SP_FS_TEX_COUNT, 1);
  } else {
    // r3d_clear_value: the colour as FS constant c0, here inline (SS6_DIRECT, address dwords 0)
    g3d_pkt7(cs, G3D_CP_LOAD_STATE6_FRAG, 7);
    g3d_emit(cs, g3d_load_state0(0, G3D_ST6_CONSTANTS, G3D_SS6_DIRECT, G3D_SB6_FS_SHADER, 1));
    g3d_emit(cs, 0);
    g3d_emit(cs, 0);
    for (unsigned i = 0; i < 4; i = i + 1) g3d_emit(cs, d->color[i]);
  }

  // r3d_run: RECTLIST from 2 auto-indexed vertices, 1 instance
  g3d_pkt7(cs, G3D_CP_DRAW_INDX_OFFSET, 3);
  g3d_emit(cs, G3D_DRAW_RECTLIST_AUTO);
  g3d_emit(cs, 1);
  g3d_emit(cs, 2);

  // sysmem render end: CCU colour flush to memory with a timestamp, then wait (the submit adds CACHE_FLUSH_TS)
  g3d_event_ts(cs, G3D_EV_PC_CCU_FLUSH_COLOR_TS, m->flush, d->seqno);
  g3d_wfi(cs);
}

// Whole stream: init (optional) + draw. Returns the dword count, 0 when cap was too small.
static inline size_t g3d_build(uint32_t *out, size_t cap, const g3d_mem *m, const g3d_draw *d, bool init) {
  g3d_cs cs = {out, 0, cap, false};
  if (init) g3d_emit_init(&cs, m);
  g3d_emit_draw(&cs, m, d);
  return cs.overflow ? 0 : cs.n;
}

#endif
