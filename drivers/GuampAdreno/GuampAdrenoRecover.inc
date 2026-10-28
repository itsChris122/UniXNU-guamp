// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp-gpu-recover=1 (default 1 since 28/10; =0 off: the hooks below only test gRecEnabled/gRecArmed and nothing else changes): hang detection and
// recovery of the published Adreno 610, included by GuampAdrenoProbe.cpp inside its namespace. Decisions in GuampGpuRecoverCore.h.
// NativeAdreno (the pinned xVanTuring/Adreno610 submodule) has no reset path and is not changed: everything runs from the board
// callbacks it already calls.
//   where:   cacheAcquire, which NativeAdrenoRing calls before every read of the fence page. After publish every such read comes
//            from a client call that holds NativeAdrenoPublish's GPU lock, so the recovery is serialized against every submit, map
//            and wait; nothing here takes that lock. A hang nobody waits for or submits behind is noticed at the next such call.
//   detect:  GuampGpuRecoverCore.h: fence stuck for guamp-gpu-recover-ms (default 2000) with work pending, or for 100 ms after an
//            error interrupt (gpuIrq) or a fault of context bank 0 of the GPU SMMU.
//   recover: what Linux a6xx_recover does for a GPU without GMU (gmu wrapper, as the A610), without the CX power collapse:
//            snapshot, CP_SQE_CNTL halt, SMMU fault status cleared, GBIF drained (a6xx_bus_clear_pending_transactions with the GX
//            side), RBBM_SW_RESET_CMD asserted 100 us, then naHwInit, CP_SQE_INSTR_BASE and the ring registers as at bring-up, the
//            SQE started on a ring head of CP_ME_INIT, a CP_MEM_WRITE check and a CP_NOP up to the ring object's write index. When
//            the check lands, every fence the ring handed out is written as completed: the lost submissions retire and free their
//            buffers (the reset left nothing in flight), the waiting clients return, and the next submission continues the fence
//            numbers. No replay: unlike msm, the innocent jobs queued behind the hung one are dropped too.
//   park:    a failed check, or more than 3 hangs in 60 s: the GPU is halted and reset, the fences are signalled, and every later
//            submission fails with kIOReturnNoResources (the ring reads as full: CP_RB_RPTR one past the write index, which
//            NativeAdrenoRing refuses without poisoning anything); guamp-ca-metal is told (guamp_ca_metal_gpu_lost) so the render
//            server goes back to the software renderer.
// guamp-gpu-recover-inject=<n> (debug, needs guamp-gpu-recover=1): n user submissions after arming (and after each recovery), the
// kernel queues a CP_WAIT_REG_MEM on a scratch word nothing writes; the CP polls memory forever (no bus access outside the kernel's
// scratch page), which is a real hang for the detector. guamp-gpu-recover-inject-times=<k> (default 1): k such hangs; 4 within a
// minute exercise the park path.
// Never reads a GPU register at or above 0xF000 (only naHwInit writes the SECVID ones, as at bring-up). The C-linkage counters
// (guamp_gpu_recover) and guamp_ca_metal_gpu_lost are declared at file scope in GuampAdrenoProbe.cpp: this file is inside its
// unnamed namespace.

struct Recover {
  grc_state core;
  bool busy = false;           // a recovery runs (pads and the injection re-enter the ring)
  uint32_t bank0 = 0;          // context bank 0 of the GPU SMMU
  uint32_t lastSmmuMs = 0;     // the SMMU fault status is read at most every 10 ms of a stall
  uint32_t smmuCause = 0;
  uint32_t injectEvery = 0;    // guamp-gpu-recover-inject
  uint32_t injectLeft = 0;     // guamp-gpu-recover-inject-times still to go
  uint32_t injectBase = 0;     // gSubmitCount when the count started
};
Recover gRec;

uint32_t nowMs() {return uint32_t(naNow(nullptr) / 1000);}
volatile uint32_t *fencePage(const Session &s) {return reinterpret_cast<volatile uint32_t *>(s.ncpu[2]);}
volatile uint32_t *scratchPage(const Session &s) {return reinterpret_cast<volatile uint32_t *>(s.ncpu[4]);}

// Every CP_RB_WPTR write of the ring object (naWrite): the ring's write index, and a new submission when the value changes (kick()
// writes the same value again).
void recoverNoteWptr(uint32_t value) {
  if (value != gLastWptr) {
    gSubmitCount = gSubmitCount + 1;
    if (gRecArmed && !gRec.busy) grc_submitted(&gRec.core, nowMs(), fencePage(gSession)[1], gRing->lastSubmitted());
  }
  gLastWptr = value;
}

bool spinAck(const Session &s, uint32_t reg, uint32_t mask, unsigned us) {
  for (unsigned i = 0; i <= us; i = i + 1) {
    if ((gr(s, reg) & mask) == mask) return true;
    IODelay(1);
  }
  return false;
}

// CP_SQE_CNTL halt, the SMMU fault status cleared (the bank terminates faulting transactions: nothing to resume), the GBIF drained,
// then the soft reset. Returns the drain acknowledges (bit 0 RBBM GBIF, 1 client, 2 arbiter).
uint32_t quiesce(Session &s, uint32_t fsr) {
  gw(s, 0x808, 3);                                                     // CP_SQE_CNTL: halt the SQE first (a6xx_recover)
  if (fsr) wr(s.smmu, gRec.bank0 + 0x58, fsr);                         // CB0 FSR, write 1 to clear
  uint32_t acks = 0;
  gw(s, 0x0016, 1);                                                    // RBBM_GBIF_HALT: the GX side
  if (spinAck(s, 0x0017, 1, 1000)) acks = acks | 1;
  gw(s, 0x3C45, 1);                                                    // GBIF_HALT: client requests
  if (spinAck(s, 0x3C46, 1, 1000)) acks = acks | 2;
  gw(s, 0x3C45, 2);                                                    // then all AXI requests
  if (spinAck(s, 0x3C46, 2, 1000)) acks = acks | 4;
  gw(s, 0x3C45, 0);                                                    // the GBIF halt needs an explicit clear
  gw(s, 0x0043, 1); (void)gr(s, 0x0043); IODelay(100);                 // RBBM_SW_RESET_CMD asserted at least 100 us
  gw(s, 0x0043, 0); (void)gr(s, 0x0043); IODelay(1);
  return acks;
}

// The bring-up again on the same buffers and the same ring object; true when the CP ran the new head (scratch check) and reached the
// ring object's write index.
bool restartCp(Session &s, uint32_t check) {
  naHwInit(s);                                                         // also clears both GBIF halts and rewrites RBBM_INT_0_MASK
  gw64(s, 0x830, s.nb[0].iova);                                        // CP_SQE_INSTR_BASE, before CP_SQE_CNTL
  const uint32_t words = uint32_t(s.nb[1].bytes / 4), mask = words - 1;
  uint32_t control = 0;                                                // CP_RB_CNTL as NativeAdrenoRing::attach: BUFSZ, BLKSZ 32 B, NO_UPDATE
  for (uint64_t v = s.nb[1].bytes / 8; v > 1; v = v >> 1) control = control + 1;
  control = control | (2u << 8) | (1u << 27);
  gw64(s, 0x800, s.nb[1].iova);                                        // CP_RB_BASE
  gw(s, 0x802, control);
  gw64(s, 0x804, s.nb[2].iova);                                        // CP_RB_RPTR_ADDR
  gw(s, 0x807, 0); gw(s, 0x885, 0);                                    // CP_RB_WPTR, CP_SCRATCH(2)
  const uint32_t start = gr(s, 0x806);                                 // CP_RB_RPTR after the reset
  grc_ring_plan plan;
  if (!grc_plan_ring(start, gLastWptr, words, &plan)) {
    IOLog("GuampAdreno: recover: ring indices out of range (rptr %u, write index %u)\n", start, gLastWptr);
    return false;
  }
  // Short distance: kernel CP_NOP submissions lengthen it while the SQE is still stopped (their fences are signalled below).
  for (uint32_t i = 0; i < plan.pads; i = i + 1) {
    uint32_t nop = 0, fence = 0;
    if (AdrenoPM4Type7(0x10, 0, &nop) != AdrenoPM4Success || gRing->submit(&nop, 1, &fence)) return false;
  }
  if (plan.pads && (!grc_plan_ring(start, gLastWptr, words, &plan) || plan.pads)) return false;
  grc_head_words head{};
  if (AdrenoPM4Type7(0x48, 8, &head.meInit) != AdrenoPM4Success || AdrenoPM4Type7(0x3d, 3, &head.memWrite) != AdrenoPM4Success ||
      (plan.nop && AdrenoPM4Type7(0x10, plan.nopPayload, &head.nop) != AdrenoPM4Success))
    return false;
  volatile uint32_t *scratch = scratchPage(s);
  scratch[1] = 0;
  gw(s, 0x807, start);                                                 // empty while the microcode starts
  gw(s, 0x808, 1);                                                     // CP_SQE_CNTL: start the microcode
  grc_fill_ring(reinterpret_cast<volatile uint32_t *>(s.ncpu[1]), words, start, &plan, &head, s.nb[4].iova + 4, check);
  __asm__ volatile("dsb sy" ::: "memory");
  gw(s, 0x807, gLastWptr);                                             // the ring object's write index: CP_RB_WPTR as it left it
  for (unsigned i = 0; i < 4000; i = i + 1) {                          // 200 ms
    __asm__ volatile("dsb sy" ::: "memory");
    if (scratch[1] == check && (gr(s, 0x806) & mask) == gLastWptr) return true;
    IODelay(50);
  }
  IOLog("GuampAdreno: recover: restart check missing: scratch %08x (want %08x) rptr %u write index %u rbbm status %08x hw_fault %08x\n",
        scratch[1], check, gr(s, 0x806), gLastWptr, gr(s, 0x210), gr(s, 0x821));
  return false;
}

// Every fence the ring handed out (pads included) is written as completed: nothing runs on the GPU after the reset.
uint32_t signalAll(Session &s) {
  volatile uint32_t *fence = fencePage(s);
  const uint32_t before = fence[1], newest = gRing->lastSubmitted();
  fence[1] = newest;
  __asm__ volatile("dsb sy" ::: "memory");
  return newest - before;
}

void park(Session &s, uint32_t fsr, const char *why) {
  guamp_gpu_dcvs_hold = 1;                                             // no clock changes on a parked GPU
  (void)quiesce(s, fsr);
  const uint32_t lost = signalAll(s);
  gGpuParked = true;
  guamp_gpu_recover[3] = 1;
  IOLog("GuampAdreno: recover: GPU parked (%s): %u fences signalled, submissions refused until reboot; %u recoveries, %u failed\n",
        why, lost, gRec.core.recoveries, gRec.core.failures);
  guamp_ca_metal_gpu_lost();
}

void recover(Session &s, grc_reason why, uint32_t stalled, uint32_t fsr) {
  const uint32_t completed = fencePage(s)[1], newest = gRing->lastSubmitted();
  guamp_gpu_recover[0] = guamp_gpu_recover[0] + 1;
  guamp_gpu_recover[4] = why;
  guamp_gpu_recover[5] = completed + 1;
  IOLog("GuampAdreno: recover: hang (%s) after %u ms: fence %u stuck, newest %u; rbbm status %08x cp hw_fault %08x rptr %u wptr %u "
        "ib1 %08x%08x rem %u, irq errors %08x, cb0 fsr %08x far %08x%08x\n", grc_name(why), stalled, completed + 1, newest,
        gr(s, 0x210), gr(s, 0x821), gr(s, 0x806), gr(s, 0x807), gr(s, 0x929), gr(s, 0x928), gr(s, 0x92a), gIrqErrors, fsr,
        rd(s.smmu, gRec.bank0 + 0x64), rd(s.smmu, gRec.bank0 + 0x60));
  const grc_action action = grc_decide(&gRec.core, nowMs(), why);
  if (action == GRC_PARK) {
    park(s, fsr, "too many hangs");
    return;
  }
  if (action != GRC_RECOVER) return;
  gRec.busy = true;
  guamp_gpu_dcvs_hold = 1;                                             // the governor keeps the clocks still while the GPU restarts
  const uint32_t acks = quiesce(s, fsr);
  const uint32_t check = 0xA6100000u | (gRec.core.recoveries & 0xffffu);
  const bool ok = restartCp(s, check);
  if (ok) {
    const uint32_t lost = signalAll(s);
    guamp_gpu_recover[6] = lost;
    if (NativeAdrenoVm *vm = NativeAdrenoGpuContextVm()) (void)vm->invalidateTlb();
    gIrqErrors = 0;
    gIrqMasked = 0;
    gIrqRepeat = 0;
    // guamp-adreno-zap: the soft reset may put the CP back in secure mode; the zap shader TrustZone loaded stays in its region, so
    // CP_SET_SECURE_MODE 0 is sent again (no PAS call, no SECVID write). Without the option, as before.
    if (gZapUp) (void)naSecureMode(s, "recover");
    else if (bootArg("guamp-adreno-zap", 1) == 0 && bootArg("guamp-adreno-secvid", 0) == 1) naSecureFallback(s);
    (void)grc_done(&gRec.core, true);
    guamp_gpu_recover[1] = guamp_gpu_recover[1] + 1;
    IOLog("GuampAdreno: recover: GPU restarted (%u of %u in this minute): drain acks %x, %u fences signalled (dropped), rbbm status "
          "%08x\n", gRec.core.inWindow, kGrcMaxRecoveries, acks, lost, gr(s, 0x210));
    gRec.injectBase = gSubmitCount;
    guamp_gpu_dcvs_hold = 0;
  } else {
    (void)grc_done(&gRec.core, false);
    guamp_gpu_recover[2] = guamp_gpu_recover[2] + 1;
    park(s, 0, "restart failed");
  }
  gRec.busy = false;
}

// guamp-gpu-recover-inject: a CP_WAIT_REG_MEM (WRITE_EQ, memory) on scratch word 2 for a value nothing writes.
void injectHang(Session &s) {
  gRec.injectLeft = gRec.injectLeft - 1;
  volatile uint32_t *scratch = scratchPage(s);
  scratch[2] = 0;
  __asm__ volatile("dsb sy" ::: "memory");
  const uint64_t at = s.nb[4].iova + 8;
  uint32_t words[7] = {0, 3u | (1u << 4), uint32_t(at), uint32_t(at >> 32), 0x1A6610FFu, 0xffffffffu, 0x10};
  uint32_t fence = 0;
  const bool queued = AdrenoPM4Type7(0x3c, 6, &words[0]) == AdrenoPM4Success && !gRing->submit(words, 7, &fence);
  guamp_gpu_recover[7] = guamp_gpu_recover[7] + 1;
  IOLog("GuampAdreno: recover: injected hang %s (fence %u, %u more)\n", queued ? "queued" : "FAILED", fence, gRec.injectLeft);
}

// cacheAcquire, under the GPU lock (see the top of this file).
void recoverPoll() {
  if (gRec.busy || gGpuParked || !gRing || gRing->poisoned() || (gSubmit && gSubmit->poisoned())) return;
  Session &s = gSession;
  if (gRec.injectLeft && gSubmitCount - gRec.injectBase >= gRec.injectEvery) {
    gRec.busy = true;
    injectHang(s);
    gRec.injectBase = gSubmitCount;
    gRec.busy = false;
  }
  const uint32_t now = nowMs();
  const uint32_t stalled = grc_track(&gRec.core, now, fencePage(s)[1], gRing->lastSubmitted());
  if (stalled < kGrcFaultMs) {
    gRec.smmuCause = 0;
    return;
  }
  uint32_t fsr = 0;
  if (now - gRec.lastSmmuMs >= 10) {   // MMIO at most every 10 ms of a stall; the waiters poll every 100 us
    gRec.lastSmmuMs = now;
    fsr = rd(s.smmu, gRec.bank0 + 0x58) & 0xc00001feu;   // CB0 FSR fault bits
    gRec.smmuCause = fsr ? GRC_CAUSE_SMMU : 0;
  }
  const uint32_t causes = gRec.smmuCause | ((gIrqErrors & kRecoverIrqErrors) ? GRC_CAUSE_IRQ : 0);
  const grc_reason why = grc_cause(&gRec.core, stalled, causes);
  if (why == GRC_NONE) return;
  if (!fsr) fsr = rd(s.smmu, gRec.bank0 + 0x58) & 0xc00001feu;
  gRec.smmuCause = 0;
  recover(s, why, stalled, fsr);
}

// After the self test: from here on every fence read may run the detector.
void recoverArm(Session &s) {
  if (!gRecEnabled) return;
  grc_init(&gRec.core, bootArg("guamp-gpu-recover-ms", kGrcHangMs));
  gRec.bank0 = smmuBank0(s);
  gRec.injectEvery = bootArg("guamp-gpu-recover-inject", 0);
  gRec.injectLeft = gRec.injectEvery ? bootArg("guamp-gpu-recover-inject-times", 1) : 0;
  gRec.injectBase = gSubmitCount;
  __asm__ volatile("dmb ish" ::: "memory");                           // the state above before a client thread sees the flag
  gRecArmed = true;
  IOLog("GuampAdreno: recover: armed (hang after %u ms, or %u ms with an error interrupt or smmu fault; %u restarts per %u s, then "
        "parked; inject %u x after %u submissions)\n", gRec.core.hangMs, kGrcFaultMs, kGrcMaxRecoveries, kGrcWindowMs / 1000,
        gRec.injectLeft, gRec.injectEvery);
}
