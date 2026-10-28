// SPDX-License-Identifier: BSD-2-Clause-Patent
// The guamp board probe of NativeAdreno (drivers/GuampAdreno/GuampAdrenoProbe.cpp, archive scripts/build-guamp-adreno.py):
// the interface its caller, the GPU thread of NativeEmmc (GpuService.inc), uses once the M1 power-up has left the Adreno 610
// powered and readable. The probe then owns the GPU's SMMU (kgsl_smmu 0x59a0000), the ring and the CP: the caller must not run
// its own M2/M3 stages (CP start, SMMU stream 0) in the same boot.
#ifndef GUAMP_ADRENO_H
#define GUAMP_ADRENO_H
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// guamp-adreno=<level> (default 0, off):
//   1  NativeAdreno's GPU context on the GPU SMMU (context bank 0, SIDs 0 and 1), the A610 hw_init, ring 0, CP_ME_INIT, a
//      CP_MEM_WRITE check, RBBM_SECVID_TRUST_CNTL = 0 written directly (no zap shader), the GPU interrupt (SPI 177), then the
//      device published for user clients
//   2  1, then the staged AGXAccelerator (IOAcceleratorES) with the Metal plugin keys, when the staged class is linked
// guamp-adreno-zap=<mode> (default 0: unchanged): after CP_ME_INIT, as mainline a6xx_hw_init (a6xx_zap_shader_init), the
// ORIGINAL signed zap shader (a610_zap, PAS id 13) is started through TrustZone PAS by the caller's zap callback, then
// CP_SET_SECURE_MODE 0 goes on ring 0. 1 authenticated start; 2 SCM/PAS queries, integrity and placement only (nothing written,
// no CP_SET_SECURE_MODE). With the option set RBBM_SECVID_TRUST_CNTL is never written (TrustZone-owned on guamp: no fallback).
typedef int (*GuampAdrenoZapLoad)(void *context, unsigned mode);   // 0 started, 1 queried only, <0 refused (pas:: codes)
struct GuampAdrenoStart {
  volatile uint8_t *gpu;     // kgsl_3d0_reg_memory 0x5900000, mapped uncached by the caller
  size_t gpuBytes;           // 0x90000
  const uint8_t *sqe;        // the SQE microcode: a630_sqe.fw without its first dword, as GpuService.inc loadSqe() returns it
  uint32_t sqeBytes;
  unsigned level;            // guamp-adreno
  GuampAdrenoZapLoad zap;    // NativeEmmc GuampGpuZap.inc (PAS loader); null: guamp-adreno-zap is refused with a log line
  void *zapContext;
};

// Result codes (also the last GuampAdreno log line). Runs on the caller's thread and returns when the stages are done; the
// published device and the GPU stay up afterwards.
enum {
  GuampAdrenoOk = 0, GuampAdrenoBadArgs = -1, GuampAdrenoSmmuMap = -2, GuampAdrenoContext = -3, GuampAdrenoBuffers = -4,
  GuampAdrenoSqe = -5, GuampAdrenoRing = -6, GuampAdrenoCpInit = -7, GuampAdrenoMemWrite = -8, GuampAdrenoIrq = -9,
  GuampAdrenoPublish = -10, GuampAdrenoAccelerator = -11, GuampAdrenoZap = -12
};
int guamp_adreno_start(const struct GuampAdrenoStart *start);

// guamp-copy-gpu-wait: waits (sleeping, thread context) until the CP timestamp reaches the newest fence handed out at the call, at
// most timeout_us. 0 done (also when nothing was ever submitted), 1 timed out, -1 no ring (not started, poisoned or parked).
// *waited_us: the time spent. NativeLegacyDisplay (scripts/guamp-canvas-variant.py) holds a weak stand-in that answers -1.
int guamp_adreno_wait_submitted(uint32_t timeout_us, uint32_t *waited_us);
// The same ring, read only: *submitted the newest fence handed out, *retired the CP timestamp of the fence page. 0, or -1 without a
// ring (as above). NativeLegacyDisplay's latest-frame copy compares *submitted before and after a copy (more GPU work since: the
// surface may have changed) and holds a weak stand-in that answers -1.
int guamp_adreno_ring_marks(uint32_t *submitted, uint32_t *retired);

#ifdef __cplusplus
}
#endif
#endif
