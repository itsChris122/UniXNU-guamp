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

// guamp-copy-gpu-wait-max (ms, default 2000; 0 = the caller's bound alone, as before). fin50 Maps (live-20261105-204852): from 331 s
// to 502 s more than 100 copy waits timed out at 500 ms, the GPU 100% busy behind the app's frames, and the copy then read the render
// server's surface before its frame was written: an older frame (the map jumping back while moved fast) or a frame half drawn (the
// cleared damage rectangle read before the map layer was drawn: the black horizontal bar, or the whole map black). Past the caller's
// bound the wait goes on while the GPU still retires fences, so the copy reads a finished frame, later (the swap queued behind it waits,
// which slows the render server down instead of showing torn frames); it gives up only when no fence retired for a whole bound (a
// stalled GPU: the hang detector's case) or at the hard bound. True: stop waiting, timed out.
static inline bool guampCopyWaitGiveUp(uint64_t spent_us, uint64_t stalled_us, uint32_t timeout_us, uint32_t max_us) {
  if (spent_us < timeout_us) return false;
  if (max_us <= timeout_us) return true;
  if (spent_us >= max_us) return true;
  return stalled_us >= timeout_us;
}
#endif
