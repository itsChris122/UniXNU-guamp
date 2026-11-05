// SPDX-License-Identifier: BSD-2-Clause-Patent
// The fence the display copy waits for (guamp-copy-gpu-wait; GuampAdrenoProbe.cpp copyTarget, host test tests/copy_target_host.cpp).
// Fence numbers grow and wrap: a is later than b when int32_t(a - b) > 0.
//  - guamp-copy-wait-compositor=0: the ring's newest fence (before patch 0017).
//  - On (default): the later of the render server's last fence (patch 0017) and the ring's newest fence when the display last accepted
//    a swap (guamp_adreno_note_swap). fin49 ("the GPU glitches a lot"): the render server's fence alone let the copy read before
//    work handed out ahead of the swap had run; whatever was queued before the swap now always comes first, and only work queued after
//    the swap (an app's next frames, 05/11 Maps) no longer keeps the screen waiting. Nothing known (both 0): the ring's newest.
//  - Never past the ring's newest fence (a stale or wrapped value falls back to it).
#ifndef GUAMP_COPY_TARGET_H
#define GUAMP_COPY_TARGET_H
// uint32_t comes from the includer (the kernel headers, or <stdint.h> in the host test).

static inline uint32_t guampCopyTarget(uint32_t newest, uint32_t compositor, uint32_t swap, bool on) {
  if (!on) return newest;
  uint32_t target = compositor;
  if (swap && (!target || int32_t(swap - target) > 0)) target = swap;
  return target && int32_t(newest - target) >= 0 ? target : newest;
}
#endif
