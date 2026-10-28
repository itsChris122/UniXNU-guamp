// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp ADSP transports, audio phase 2: kernel glue of GuampSmemWrite.inc (the one SMEM allocator, shared with Wi-Fi R1,
// under the one SMEM lock of GuampSmemLock.inc: local IOLock + TCSR mutex),
// GuampSmp2pEdge.inc (apps SMP2P item 429), GuampGlinkEdge.inc (GLINK over SMEM 478/479/480) and GuampApr.inc (APR,
// q6core). Included by GuampAdsp.inc, driven from its guamp-adsp thread (single thread; SMEM allocations take the
// shared lock, so they are serialised with the Wi-Fi provider's). Opt-in with guamp-adsp=1 and
//   guamp-adsp-ipc=1  before the PAS start: apps SMP2P item 429 (as smp2p.c probes it at boot: "master-kernel" 0,
//                     "rdbg" 0, "sleepstate" bit 12 = apps awake), APCS IPC bit 10 kicks, inbound 443 negotiation and
//                     SSR-ack handshake while waiting for err-ready and afterwards
//   guamp-adsp-ipc=2  + after err-ready (SUBSYS_AFTER_POWERUP, when glink_probe.c registers the SMEM edge): GLINK
//                     descriptor 478 + TX FIFO 479 (16 KiB), version 1 / INTENT_REUSE, RX FIFO 480 of the ADSP,
//                     channel "apr_audio_svc" opened with its DT intents (20 x 512 bytes); every step logged
//   guamp-adsp-ipc=3  + APR on that channel: q6core ADSP state query, then the framework version: "ADSP APR OK"
//   guamp-adsp-ipc=4  + after "ADSP APR OK": the first tone (GuampAfe.inc apr::AfeTone) on the speaker port
//                     PRIMARY_MI2S_RX 0x1000: LPASS DCODEC vote, codec side prepare (GuampAudioHw.h, weak stand-in
//                     until wt-audio-hw links its own), MI2S IBIT clock, I2S config, DEVICE_START, amplifier on,
//                     AFE DTMF for guamp-adsp-tone=<ms> (default 3000, 100..20000; gain guamp-adsp-tone-gain, default
//                     0x800), then all undone: "ADSP TONE OK|FAILED". No shared memory, no ASM/ADM.
//   guamp-adsp-ipc=5  instead of the tone, the PCM chain of Linux q6afe/q6adm/q6asm (GuampAsm.inc apr::Playback): AFE
//                     port held (vote, codec prepare, clock, port config, start, amp), shared buffer mapped with
//                     ASM_CMD_SHARED_MEM_MAP_REGIONS, ADM COPP open, ASM stream open, matrix route, media format, run,
//                     a 1 kHz sine written in 10 ms periods refilled on WRITE_DONE for guamp-adsp-tone ms, then all
//                     closed in reverse: "ADSP PCM OK|FAILED". Route guamp-adsp-route (1 speaker, 2 earpiece,
//                     3 headphone), stream width guamp-adsp-pcm-bits (16|24), buffer address mode guamp-adsp-pcm-mem
//                     (GuampAudioPlay.inc: 0 auto from the SMMU probe, 1 physical msw 0, 2 physical msw 1)
//   guamp-adsp-ipc=6  the same chain as the sink of /dev/guamp-audio (with guamp-audio-dev=1): up while the plug-in
//                     plays, down 2 s after STOP/close, route from GUAMP_AUDIO_SET_ROUTE
// Default 0: GuampAdsp.inc behaves exactly as in phase 1 (SMEM read-only). Other GLINK channels the ADSP opens
// (IPCRTR, fastrpcglink-apps-dsp, glink_ssr) are logged and left unanswered (no QRTR/QMI/FastRPC here).
// Facts (guamp DT ~/moto/guamp.dts and kernel): SMEM 0x46000000 + 2 MiB, hwlocks = <&tcsr_mutex 3>, tcsr_mutex syscon
// 0x340000 stride 0x1000 (lock 3 = 0x343000, apps owner id 1, qcom_hwspinlock.c); APCS IPC 0x0f111000 + 8
// (qcom,bengal-apcs-hmss-global), write BIT(n): smp2p-adsp n = 10, glink adsp edge n = 8; glink adsp edge
// qcom,remote-pid 2, channels IPCRTR (0x800x5 0x2000x3 0x4400x2), apr_audio_svc (0x200 x 0x14),
// fastrpcglink-apps-dsp (0x64 x 0x40), glink_ssr. Interrupts (SPI 279 smp2p, 277 glink) are not used: 20-50 ms poll.
// Nothing outside SMEM, the TCSR mutex word and the IPC register is written; XNU RAM is never mapped.

#include "GuampSmemWrite.inc"
#include "GuampSmemLock.inc"
#include "GuampSmp2pEdge.inc"
#include "GuampGlinkEdge.inc"
#include "GuampApr.inc"
#include "GuampAfe.inc"
#include "GuampAudioHwWeak.inc"
#include "GuampAsm.inc"
#ifndef GUAMP_MIC_DEV_INC   // without the microphone glue (host tests of this file): no mic
namespace micglue {
inline bool configure() { return false; }
inline bool enabled() { return false; }
inline bool receive(const uint8_t *, uint32_t, uint32_t) { return false; }
inline void service(uint32_t, int (*)(void *, const uint8_t *, uint32_t)) {}
}  // namespace micglue
#endif

namespace adspipc {

constexpr uint16_t kHost = 2;
constexpr uint32_t kSmp2pOut = 429, kSmp2pIn = 443, kGlinkDesc = 478, kGlinkTx = 479, kGlinkRx = 480;
constexpr uint32_t kDescBytes = 32, kTxBytes = 0x4000;
constexpr uint64_t kApcsPhys = 0x0f111000;
constexpr uint32_t kApcsIpc = 8, kBitSmp2p = 10, kBitGlink = 8, kAwake = 1u << 12, kFatal = 1u << 0;
constexpr unsigned kRxWaitMs = 10000, kOpenWaitMs = 10000, kLoopMs = 20;
const char *const kOutNames[3] = {"master-kernel", "rdbg", "sleepstate"};
const uint32_t kOutValues[3] = {0, 0, kAwake};
const qipc::GlinkChannelSpec kChannels[1] = {{"apr_audio_svc", 1, {{0x200, 20}, {0, 0}, {0, 0}}}};

constinit unsigned gLevel = 0;
constinit volatile uint8_t *gSmem = nullptr, *gApcs = nullptr;
constinit smem::Info *gInfo = nullptr;
constinit smem::Smp2p *gScratch = nullptr;
constinit qipc::SmemArena gArena = {};
constinit qipc::Smp2pEdge gSmp2p;
constinit qipc::Glink *gGlink = nullptr;
constinit apr::Core gCore;
constinit apr::AfeTone gTone;
constinit uint32_t gToneMs = 3000, gToneGain = 0x800;
constinit bool gToneStarted = false;
// Levels 5/6: the playback chain; the kernel parts (buffer + SMMU probe, the /dev/guamp-audio ring) come through
// gPlayGlue (GuampAudioPlay.inc registers them before adsp::start; a host test sets its own)
struct PlayGlue {
  void (*report)(const char *when) = nullptr;   // SMMU fault registers of the ADSP stream (logs only on change)
  bool (*shm)(apr::Shm *out) = nullptr;
  uint32_t (*source)(void *, uint8_t *dst, uint32_t frames, uint32_t bits) = nullptr;
  bool (*wanted)(void *, uint32_t *route) = nullptr;
  void (*attach)(void *, bool on) = nullptr;
};
constinit PlayGlue gPlayGlue = {};
constinit apr::Playback gPlay;
constinit bool gPlayStarted = false, gPlayNoShm = false;
constinit uint32_t gPcmBits = 16, gPcmRoute = GUAMP_AUDIO_DEV_ROUTE_SPEAKER;
void startPlay();
constinit uint16_t gApr = 0;
constinit bool gReady = false, gNegotiatedLogged = false, gOpenLogged = false, gAprStarted = false, gFailed = false;
constinit uint32_t gSlave = 0, gDesc = 0, gTxFifo = 0, gReadyAt = 0, gLinkError = 0;
constinit uint64_t gEpoch = 0;

uint32_t nowMs() {
  uint64_t ns = 0;
  absolutetime_to_nanoseconds(mach_absolute_time() - gEpoch, &ns);
  return static_cast<uint32_t>(ns / 1000000);
}
void barrier(void *) { guampModemBarrier(); }
uint32_t smemRd(void *, uint32_t at) { return *reinterpret_cast<volatile uint32_t *>(gSmem + at); }
void smemWr(void *, uint32_t at, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(gSmem + at) = v; }
void ipcKick(uint32_t bit) {
  guampModemBarrier();
  *reinterpret_cast<volatile uint32_t *>(gApcs + kApcsIpc) = 1u << bit;
  guampModemBarrier();
}
void kickSmp2p(void *) { ipcKick(kBitSmp2p); }
void kickGlink(void *) { ipcKick(kBitGlink); }

// The kernel's one SMEM lock (GuampSmemLock.inc), the same the Wi-Fi provider takes
bool hwLock(void *) { return guamp_smem_hwlock_acquire(); }
void hwUnlock(void *) { guamp_smem_hwlock_release(); }
void smemNote(void *, const qipc::SmemArena::Note &n) {
  IOLog("GuampAdsp: ipc SMEM item %u (host %u) exists with %u bytes, wanted %u: %s\n", n.item, n.host, n.found, n.wanted,
        n.result ? "refused" : "reused");
}

void *heapAlloc(void *, uint32_t n) { return IOMallocZero(n); }
void heapFree(void *, void *p, uint32_t n) { IOFree(p, n); }

void glinkTrace(void *, int event, const char *name, uint32_t a, uint32_t b) {
  IOLog("GuampAdsp: glink %s%s%s %x %x at %u ms\n", qipc::Glink::traceName(event), name[0] ? " " : "", name, a, b, nowMs());
}
void aprLog(void *, const char *what, uint32_t a, uint32_t b) {
  IOLog("GuampAdsp: apr core %s %x %x at %u ms\n", what, a, b, nowMs());
}
void afeLog(void *, const char *what, uint32_t a, uint32_t b) {
  IOLog("GuampAdsp: afe %s %x %x at %u ms\n", what, a, b, nowMs());
}
// The codec/amplifier side (GuampAudioHw.h), speaker route, 48 kHz 16-bit stereo on PRIMARY_MI2S_RX
int afeHw(void *, int step, bool on) {
  const guamp_audio_hw_format fmt = {48000, 16, 2, apr::kPortPriMi2sRx};
  int e = 0;
  if (step == apr::AfeTone::HwPrepare) e = guamp_audio_hw_prepare(GUAMP_AUDIO_ROUTE_SPEAKER, &fmt);
  else if (step == apr::AfeTone::AmpOn || step == apr::AfeTone::AmpOff) e = guamp_audio_hw_amp(GUAMP_AUDIO_ROUTE_SPEAKER, on);
  else guamp_audio_hw_release(GUAMP_AUDIO_ROUTE_SPEAKER);
  if (e == GUAMP_AUDIO_HW_ABSENT)
    IOLog("GuampAdsp: afe %s: codec side not linked (GuampAudioHw.h stand-in): DSP part only, no sound expected\n",
          apr::AfeTone::stepName(step));
  return e;
}
void playLog(void *, const char *what, uint32_t a, uint32_t b) {
  IOLog("GuampAdsp: %s %x %x at %u ms\n", what, a, b, nowMs());
}
// GuampAudioHw.h for the playback route: prepare(route, {48000, 16, channels, port}), amp on/off, release
int playHw(void *, uint32_t route, uint16_t port, uint32_t channels, int step, bool on) {
  static_assert(GUAMP_AUDIO_ROUTE_SPEAKER == GUAMP_AUDIO_DEV_ROUTE_SPEAKER &&
                GUAMP_AUDIO_ROUTE_EARPIECE == GUAMP_AUDIO_DEV_ROUTE_EARPIECE &&
                GUAMP_AUDIO_ROUTE_HEADPHONE == GUAMP_AUDIO_DEV_ROUTE_HEADPHONE, "route values of the ABI and GuampAudioHw.h");
  const guamp_audio_hw_format fmt = {48000, 16, channels, port};
  int e = 0;
  if (step == apr::AfeTone::HwPrepare) e = guamp_audio_hw_prepare(route, &fmt);
  else if (step == apr::AfeTone::AmpOn || step == apr::AfeTone::AmpOff) e = guamp_audio_hw_amp(route, on);
  else guamp_audio_hw_release(route);
  IOLog("GuampAdsp: play hw %s route %u port %x %u ch: %d%s\n", apr::AfeTone::stepName(step), route, port, channels, e,
        e == GUAMP_AUDIO_HW_ABSENT ? " (codec side not linked: DSP part only, no sound expected)" : "");
  return e;
}
// guamp_audio_hw_set_lpass_clock_hook (GuampAudioHwExt.h of the HW side): its headphone/earpiece prepare asks for
// rx_core/rx_npl. They are AFE commands answered on this same thread, so the chain switches them itself (AfeTone
// ClockOn/ClockOn2, before HwPrepare) and the hook only confirms they run; "off" is a no-op (the chain switches them
// off after HwRelease).
int playLpassClock(uint32_t clk, uint32_t hz, int on) {
  if (!on) return 0;
  const apr::AfeTone &a = gPlay.afe;
  const bool held = gPlayStarted && a.port.cdcDma && hz == apr::kRxMacroClkHz &&
                    ((clk == apr::kClkRxCore && a.clockOn) || (clk == apr::kClkRxNpl && a.clockOn2));
  IOLog("GuampAdsp: play lpass clock %x %u Hz requested by the codec side: %s\n", clk, hz, held ? "running" : "NOT running");
  return held ? 0 : -5;
}
int aprSend(void *, const uint8_t *p, uint32_t n) { return gGlink && gGlink->send(gApr, p, n) == qipc::Ok ? 0 : -1; }
void glinkData(void *, uint16_t channel, const uint8_t *p, uint32_t n) {
  apr::Header h;
  const uint8_t *payload = nullptr;
  uint32_t bytes = 0;
  const bool ok = apr::parse(p, n, &h, &payload, &bytes);
  IOLog("GuampAdsp: apr rx ch %u %u bytes: %s src %x:%x port %x dst %x:%x port %x token %x opcode %x payload %u\n", channel,
        n, ok ? "ok" : "MALFORMED", ok ? h.srcDomain : 0, ok ? h.srcSvc : 0, ok ? h.srcPort : 0, ok ? h.destDomain : 0,
        ok ? h.destSvc : 0, ok ? h.destPort : 0, ok ? h.token : 0, ok ? h.opcode : 0, bytes);
  if (gLevel < 3 || channel != gApr || !ok) return;
  if (micglue::receive(p, n, nowMs())) return;   // microphone chain first (GuampMicDev.inc): only its mic token / ASM session 2
  if (gPlayStarted && gPlay.receive(p, n, nowMs())) return;   // playback (session 1) takes every other ASM/ADM packet
  if (h.srcSvc == apr::kSvcAfe && gToneStarted) gTone.receive(p, n, nowMs());
  else gCore.receive(p, n, nowMs());
}

// pas::Hooks::prepare (after the stopped-peer check, before any vote or RAM write): mappings + SMP2P 429.
// Nonzero = PAS does not start (kErrPrepare, no RAM written).
int prepare(void *) {
  gEpoch = mach_absolute_time();
  if (pas::overlap(wifiprep::kSmemPhys, wifiprep::kSmemBytes, guamp_phys_base, guamp_phys_size)) {
    IOLog("GuampAdsp: ipc: SMEM overlaps XNU RAM: refused\n");
    return qipc::Range;
  }
  if (!gSmem) gSmem = mapPhysical(wifiprep::kSmemPhys, wifiprep::kSmemBytes);
  if (!gApcs) gApcs = mapPhysical(kApcsPhys, 0x1000);
  if (!gInfo) gInfo = static_cast<smem::Info *>(IOMallocZero(sizeof(smem::Info)));
  if (!gScratch) gScratch = static_cast<smem::Smp2p *>(IOMallocZero(sizeof(smem::Smp2p)));
  if (!gSmem || !gApcs || !gInfo || !gScratch) {
    IOLog("GuampAdsp: ipc: mapping/allocation failed (smem %d apcs %d)\n", gSmem != nullptr, gApcs != nullptr);
    return qipc::NoMemory;
  }
  gArena.mem = {nullptr, static_cast<uint32_t>(wifiprep::kSmemBytes), smemRd, smemWr, barrier};
  gArena.info = gInfo;
  gArena.lock = hwLock;
  gArena.unlock = hwUnlock;
  gArena.note = smemNote;
  const int parsed = gArena.refresh();
  const smem::Partition *p = parsed ? nullptr : smem::partitionFor(*gInfo, kHost);
  IOLog("GuampAdsp: ipc level %u: SMEM v%u %s, apps-adsp partition %s (+%x, %x bytes, %u items), hwlock %llx, IPC %llx bits %u/%u\n",
        gLevel, gInfo->version, qipc::errorName(parsed), p ? "found" : "ABSENT", p ? p->offset : 0, p ? p->size : 0,
        p ? p->items : 0, smemlock::kMutexPhys, kApcsPhys + kApcsIpc, kBitSmp2p, kBitGlink);
  if (parsed || !p) return parsed ? parsed : qipc::NotFound;
  const int e = gSmp2p.start(&gArena, 0, kHost, kSmp2pOut, kSmp2pIn, kOutNames, kOutValues, 3, kickSmp2p, nullptr);
  IOLog("GuampAdsp: ipc smp2p out %u: %s (%s at +%x, %u bytes): master-kernel 0, rdbg 0, sleepstate %x, features 1, kick bit %u\n",
        kSmp2pOut, qipc::errorName(e), gSmp2p.created ? "created" : "existing", gSmp2p.outbound, qipc::Smp2pEdge::kBytes,
        kAwake, kBitSmp2p);
  return e;
}

// glink_probe_smem_reg at SUBSYS_AFTER_POWERUP: 478 + 479, pipes, version
void startGlink() {
  bool created = false, createdTx = false;
  uint32_t txBytes = 0;
  int e = gArena.allocate(kHost, kGlinkDesc, kDescBytes, &gDesc, &created);
  if (!e) e = gArena.allocate(kHost, kGlinkTx, kTxBytes, &gTxFifo, &createdTx, &txBytes);   // existing: its own size
  if (!e && !gGlink) gGlink = static_cast<qipc::Glink *>(IOMallocZero(sizeof(qipc::Glink)));
  if (!e && !gGlink) e = qipc::NoMemory;
  if (!e) {
    qipc::GlinkPipe tx, rx;
    tx.mem = rx.mem = gArena.mem;
    tx.tx = true;
    tx.fifo = gTxFifo;
    tx.length = txBytes;
    tx.tail = gDesc;
    tx.head = gDesc + 4;
    rx.tail = gDesc + 8;
    rx.head = gDesc + 12;
    const qipc::Allocator heap = {nullptr, heapAlloc, heapFree};
    e = gGlink->start(tx, rx, heap, kChannels, gLevel >= 2 ? 1 : 0, nullptr, kickGlink, glinkTrace, glinkData);
    if (!e) e = gGlink->step();
  }
  IOLog("GuampAdsp: ipc glink: desc %u %s at +%x, tx fifo %u %s at +%x (%u bytes): %s at %u ms\n", kGlinkDesc,
        created ? "created" : "existing", gDesc, kGlinkTx, createdTx ? "created" : "existing", gTxFifo, txBytes,
        qipc::errorName(e), nowMs());
  if (e) gFailed = true;
}

// One pass, every 20-50 ms from the guamp-adsp thread
void service() {
  if (!gSmem || gFailed) return;
  bool has = false;
  uint32_t value = 0;
  const int e = gSmp2p.poll(gScratch, "slave-kernel", &value, &has);
  if (!e && !gNegotiatedLogged) {
    gNegotiatedLogged = true;
    IOLog("GuampAdsp: ipc smp2p 443 open at %u ms: in features %x flags %x, out features %x, ssr-ack %s\n", nowMs(),
          gSmp2p.inFeatures, gSmp2p.inFlags, gSmp2p.outFeatures, gSmp2p.ssrEnabled ? "on" : "off");
  }
  if (!e && gSmp2p.acks) {
    static uint32_t logged = 0;
    if (logged != gSmp2p.acks) IOLog("GuampAdsp: ipc smp2p restart-done %x acked (%u)\n", gSmp2p.inFlags & 1, gSmp2p.acks);
    logged = gSmp2p.acks;
  }
  if (e && e != qipc::Again) IOLog("GuampAdsp: ipc smp2p 443 %s\n", qipc::errorName(e));
  if (!e) gSlave = value;
  if (!gGlink || !gGlink->running) return;
  if (!gGlink->rx.attached()) {
    uint32_t at = 0, bytes = 0;
    if (gArena.find(kHost, kGlinkRx, &at, &bytes) == qipc::Ok) {
      const int a = gGlink->attachRx(at, bytes);
      if (a) IOLog("GuampAdsp: ipc glink rx fifo 480 at +%x (%u bytes) refused: %s\n", at, bytes, qipc::errorName(a));
    }
  }
  const int s = gGlink->step();
  if (s) {
    gLinkError = static_cast<uint32_t>(-s);
    gFailed = true;
    IOLog("GuampAdsp: ipc glink link error %s after %u commands: stopped\n", qipc::errorName(s), gGlink->rxCommands);
    return;
  }
  if (gLevel >= 2 && gGlink->negotiated && !gApr) {
    const int o = gGlink->open(kChannels[0].name, &gApr);
    if (o) IOLog("GuampAdsp: ipc glink open %s: %s\n", kChannels[0].name, qipc::errorName(o));
  }
  if (gApr && gGlink->ready(gApr) && !gOpenLogged) {
    gOpenLogged = true;
    const qipc::Glink::Channel *c = gGlink->channel(gApr);
    uint32_t intents = 0;
    for (const auto &ri : c->remoteIntent) intents += ri.id ? 1 : 0;
    IOLog("GuampAdsp: ipc glink %s ready at %u ms (lcid %u rcid %u, %u remote intents so far)\n", kChannels[0].name, nowMs(),
          gApr, c->remote, intents);
  }
  if (gLevel >= 3 && gApr && gGlink->ready(gApr)) {
    if (!gAprStarted) {
      gAprStarted = true;
      gCore.ctx = nullptr;
      gCore.send = aprSend;
      gCore.log = aprLog;
      gCore.readyBudgetMs = 10000;
      gCore.begin(nowMs());
    }
    gCore.step(nowMs());
    if (gLevel >= 5 && gCore.finished() && gCore.stateAnswered && !gPlayStarted && !gPlayNoShm) startPlay();
    if (gPlayStarted) gPlay.stepAt(nowMs());
    if (micglue::enabled() && gCore.finished() && gCore.stateAnswered) micglue::service(nowMs(), aprSend);
    if (gLevel == 4 && gCore.finished() && gCore.stateAnswered && !gToneStarted) {
      gToneStarted = true;
      gTone.ctx = nullptr;
      gTone.send = aprSend;
      gTone.log = afeLog;
      gTone.hw = afeHw;
      gTone.toneMs = gToneMs;
      gTone.gain = static_cast<uint16_t>(gToneGain);
      IOLog("GuampAdsp: afe tone: port %x, mi2s ibit %u Hz, i2s %u bit SD0 stereo %u Hz, dtmf %u/%u Hz gain %x for %u ms\n",
            gTone.port.port, gTone.clockHz, gTone.port.bits, gTone.port.rate, gTone.high, gTone.low, gTone.gain,
            gTone.toneMs);
      gTone.begin(nowMs());
    }
    if (gToneStarted) gTone.stepAt(nowMs());
    gGlink->step();
  }
}

// Levels 5/6: shared buffer, then the chain (test sine or /dev/guamp-audio sink)
void startPlay() {
  apr::Shm shm;
  if (!gPlayGlue.shm || !gPlayGlue.shm(&shm) || !shm.cpu || shm.bytes < apr::PcmPath::kSlots * apr::PcmPath::kSlotStride) {
    gPlayNoShm = true;
    IOLog("GuampAdsp: play: no shared buffer for the ADSP (%s): PCM path not started\n",
          gPlayGlue.shm ? "allocation or SMMU check failed" : "kernel glue not registered");
    return;
  }
  gPlayStarted = true;
  gPlay.mode = gLevel == 5 ? apr::Playback::Test : apr::Playback::Sink;
  gPlay.route = gPcmRoute;
  gPlay.bits = gPcmBits;
  gPlay.testMs = gToneMs;
  gPlay.shm = shm;
  gPlay.hooks.ctx = nullptr;
  gPlay.hooks.send = aprSend;
  gPlay.hooks.log = playLog;
  gPlay.hooks.hw = playHw;
  gPlay.hooks.source = gPlayGlue.source;
  gPlay.hooks.wanted = gPlayGlue.wanted;
  gPlay.hooks.attach = gPlayGlue.attach;
  IOLog("GuampAdsp: play %s: route %u port %x, %u ch on the port, stream %u bit 48000 Hz stereo, %u x %u-byte periods, "
        "buffer %x:%08x %u bytes%s\n", gLevel == 5 ? "test (1 kHz sine)" : "sink of /dev/guamp-audio", gPlay.route,
        apr::Playback::portOf(gPlay.route), apr::Playback::channelsOf(gPlay.route), gPlay.bits, apr::PcmPath::kSlots,
        apr::PcmPath::kFrames * 2 * (gPlay.bits == 24 ? 4 : 2), shm.msw, shm.iovaLsw, shm.bytes,
        gLevel == 5 ? "" : ", waiting for the plug-in");
  gPlay.stepAt(nowMs());
}

// From awaitReady when err-ready is seen (Linux: SUBSYS_AFTER_POWERUP)
void onReady() {
  if (gReady || !gSmem) return;
  gReady = true;
  gReadyAt = nowMs();
  if (gLevel >= 2) startGlink();
}

// After pas::boot returned "up": waits for the milestones, logs the verdict, then keeps the edge serviced. Returns
// only on an ADSP err-fatal or a link error (the caller logs the failure reason).
void run() {
  if (!gSmem) return;
  if (!gReady) onReady();
  const uint32_t begin = nowMs();
  bool rxLogged = false, verdict = gLevel < 2, toneVerdict = gLevel != 4, playVerdict = gLevel != 5;
  for (;;) {
    service();
    const uint32_t t = nowMs() - begin;
    if (gSlave & kFatal) {
      IOLog("GuampAdsp: ipc ADSP err-fatal (slave-kernel %x) at %u ms\n", gSlave, nowMs());
      return;
    }
    if (gFailed) {
      if (!verdict) IOLog("GuampAdsp: ADSP APR FAILED (transport error %u)\n", gLinkError);
      return;
    }
    if (!verdict && gGlink) {
      if (!rxLogged && gGlink->rx.attached()) rxLogged = true;
      if (!rxLogged && t >= kRxWaitMs) {
        IOLog("GuampAdsp: ipc glink: item 480 (ADSP RX FIFO) not allocated %u ms after ready\n", t);
        IOLog("GuampAdsp: ADSP APR FAILED (no GLINK from the ADSP)\n");
        verdict = true;
      } else if (gLevel == 2 && gOpenLogged) {
        IOLog("GuampAdsp: ipc milestone 2: GLINK up, %s open with intents (%u commands, %u opens ignored)\n",
              kChannels[0].name, gGlink->rxCommands, gGlink->ignoredOpens);
        verdict = true;
      } else if (!gOpenLogged && t >= kRxWaitMs + kOpenWaitMs) {
        IOLog("GuampAdsp: ipc glink: %s not ready (negotiated %u, %u commands, %u opens ignored)\n", kChannels[0].name,
              gGlink->negotiated, gGlink->rxCommands, gGlink->ignoredOpens);
        IOLog("GuampAdsp: ADSP APR FAILED (channel %s)\n", kChannels[0].name);
        verdict = true;
      } else if (gLevel >= 3 && gCore.finished()) {
        if (gCore.stateAnswered)
          IOLog("GuampAdsp: ADSP APR OK (state %u after %u tries, fwk %s %u.%u.%u.%u, %u services)\n", gCore.state, gCore.tries,
                gCore.fwkAnswered ? "answered" : gCore.fwkRefused ? "refused" : "no answer", gCore.major, gCore.minor,
                gCore.branch, gCore.subbranch, gCore.services);
        else
          IOLog("GuampAdsp: ADSP APR FAILED (no answer to %u state queries)\n", gCore.tries);
        for (uint32_t i = 0; i < gCore.listed; i++)
          IOLog("GuampAdsp: apr service %x api %u branch %u\n", gCore.service[i].id, gCore.service[i].api,
                gCore.service[i].branch);
        verdict = true;
      }
      if (verdict && toneVerdict) IOLog("GuampAdsp: ipc edge kept serviced every %u ms\n", kLoopMs);
    }
    if (verdict && !toneVerdict && (gTone.finished() || !gCore.stateAnswered)) {
      toneVerdict = true;
      if (!gToneStarted)
        IOLog("GuampAdsp: ADSP TONE FAILED (no APR)\n");
      else if (gTone.ok())
        IOLog("GuampAdsp: ADSP TONE OK (%u ms on port %x, %u answers, vote %s, codec errors %u, undo errors %u)\n",
              gTone.toneMs, gTone.port.port, gTone.answered, gTone.handle ? "held" : "missing", gTone.hwErrors,
              gTone.undoErrors);
      else
        IOLog("GuampAdsp: ADSP TONE FAILED (%s status %x, %u answers, undo errors %u)\n",
              apr::AfeTone::stepName(static_cast<int>(gTone.failedStep)), gTone.failedStatus, gTone.answered,
              gTone.undoErrors);
      IOLog("GuampAdsp: ipc edge kept serviced every %u ms\n", kLoopMs);
    }
    if (verdict && !playVerdict && (gPlay.finished() || gPlayNoShm || !gCore.stateAnswered)) {
      playVerdict = true;
      if (gPlayGlue.report) gPlayGlue.report("at the PCM verdict");
      const apr::PcmPath &c = gPlay.pcm;
      if (!gPlayStarted)
        IOLog("GuampAdsp: ADSP PCM FAILED (%s)\n", gPlayNoShm ? "no shared buffer" : "no APR");
      else if (gPlay.ok())
        IOLog("GuampAdsp: ADSP PCM OK (route %u port %x, %u bit, %u ms, %u writes, %u done, 0 errors, copp %x, handle %x, "
              "max gap %u ms, codec errors %u)\n", gPlay.route, c.afePort, gPlay.bits, gPlay.testMs, c.writes,
              c.writeDones, c.copp, c.handle, c.maxLateMs, gPlay.afe.hwErrors);
      else
        IOLog("GuampAdsp: ADSP PCM FAILED (afe %s status %x; pcm %s status %x; %u writes, %u done, %u errors (last %x), "
              "%u unexpected)\n", gPlay.afe.setupFailed ? apr::AfeTone::stepName(static_cast<int>(gPlay.afe.failedStep)) : "ok",
              gPlay.afe.failedStatus, c.failed ? apr::PcmPath::stepName(static_cast<int>(c.failedStep)) : "ok",
              c.failedStatus, c.writes, c.writeDones, c.writeErrors, c.lastError, c.unexpected);
      IOLog("GuampAdsp: ipc edge kept serviced every %u ms\n", kLoopMs);
    }
    // 8 x 10 ms periods in flight: poll faster while the ASM stream runs
    static uint32_t lastReport = 0;
    if (gPlayGlue.report && gPlayStarted && gPlay.active() && nowMs() - lastReport >= 1000) {
      lastReport = nowMs();
      gPlayGlue.report("while streaming");
    }
    IOSleep(gPlayStarted && gPlay.active() ? 5 : kLoopMs);
  }
}

}  // namespace adspipc
