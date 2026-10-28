// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp ADSP (LPASS Hexagon) authenticated start, audio phase 1. Included by NativeEmmc.cpp after GuampWifiPrep.inc
// (ext4, firmware verifier, SMEM parser, partition helpers) and GuampRpm.inc (proxy votes). Opt-in, own thread:
//   guamp-adsp=2   SCM/PAS queries, firmware integrity (SHA-384 of every adsp.bNN against the hash table) and
//                  placement only: nothing mapped, written or started
//   guamp-adsp=1   the authenticated start of GuampPas.inc (PAS INIT_IMAGE, MEM_SETUP, load, AUTH_AND_RESET),
//                  then the ready signal over SMP2P; PAS SHUTDOWN and cleanup on any failure
//   guamp-adsp-delay=<s>  start later (0..600 s, default 0), so the USB mirror (connected ~4 s after boot) sees every
//                  GuampAdsp line (as guamp-gpu-delay / guamp-wifi-delay); the thread logs the delay it honoured
//   guamp-adsp-ipc=1|2|3  with guamp-adsp=1 only, audio phase 2 (GuampAdspIpc.inc): 1 apps SMP2P item 443 before the
//                  start + inbound negotiation; 2 + GLINK SMEM edge after err-ready and channel apr_audio_svc;
//                  3 + APR q6core state / framework version queries ("ADSP APR OK"). Default 0 = phase 1 only.
// Default: nothing runs. The original signed firmware is loaded unchanged; TrustZone authenticates it.
// Facts (guamp device tree: decompiled ~/moto/guamp.dts = bengal.dtsi + guamp dtbo, guamp kernel sources):
//   qcom,lpass@ab00000 (qcom,pil-tz-generic): qcom,pas-id 1, qcom,firmware-name "adsp", memory-region
//     pil_adsp_region@52200000 size 0x1c00000 (removed-dma-pool, no-map; UEFI "PIL Reserved" 0x4ab00000-0x53f00000,
//     far below the XNU window 0x80000000+), qcom,smem-id 423 (failure reason), proxy supplies vdd_lpi_cx (RPM
//     "rwlc" 0) and vdd_lpi_mx ("rwlm" 0) at level 384, proxy clock xo (RPM clk0 0 via cxo_pil_lpass voter),
//     qcom,proxy-timeout-ms 10000; interrupts: wdog GIC SPI 282, err-fatal / err-ready / proxy-unvote / stop-ack =
//     bits 0 / 1 / 2 / 3 of the ADSP's SMP2P "slave-kernel" entry; qcom,force-stop = bit 0 of apps "master-kernel".
//   qcom,smp2p-adsp: qcom,smem = <443 429> (443 apps -> adsp, allocated by the apps side; 429 adsp -> apps,
//     allocated by the ADSP), local pid 0, remote pid 2, IPC: APCS 0x0f111008 bit 10, GIC SPI 279.
//   GLINK edge "lpass": remote pid 2, APCS bit 8, GIC SPI 277; SMEM 478 descriptor and 479 TX FIFO allocated by apps,
//     480 RX FIFO allocated by the ADSP (qcom_glink_smem.c): 480 appearing = the ADSP's GLINK is up.
//   adsp-loader (guamp dtbo): nvmem "adsp_variant" = qfprom 0x1b40000 + 0x6011 bit 3; value 1 selects "adsp2".
//   Firmware: modem_<slot> partition (ext4), /image/adsp.mdt + adsp.b00..b35 (the Android /vendor/firmware_mnt).
// Without guamp-adsp-ipc read-only towards SMEM: nothing allocated there (no 443, no GLINK items), only the ADSP's own
// 429 entry is observed. The cleanup path uses PAS SHUTDOWN directly (no graceful force-stop through 443).

#include "GuampScm.inc"
#include "GuampScmArm64.inc"
#include "GuampPasCore.inc"
#include "GuampPas.inc"
#include "GuampAdspIpc.inc"

namespace adsp {

constexpr uint64_t kBase = 0x52200000, kBytes = 0x1c00000;
constexpr unsigned kPasId = 1;
constexpr uint16_t kHost = 2;                                     // SMEM/SMP2P/GLINK remote pid of the ADSP
constexpr uint32_t kSmp2pIn = 429, kSmp2pOut = 443, kReason = 423, kGlinkDesc = 478, kGlinkTx = 479, kGlinkRx = 480;
constexpr uint32_t kFatal = 1u << 0, kReady = 1u << 1, kHandover = 1u << 2, kStopAck = 1u << 3;
constexpr uint64_t kQfpromPage = 0x1b46000;                       // qfprom 0x1b40000 + 0x6000
constexpr uint32_t kVariantWord = 0x10, kVariantBit = 8 + 3;      // byte 0x6011 bit 3 inside the word at 0x6010
constexpr uint32_t kRwlc = 0x636c7772u, kRwlm = 0x6d6c7772u;      // "rwlc" / "rwlm" (pm6125_l3_level / l2_level)
constexpr uint32_t kKeyVlvl = 0x6c766c76u;                         // "vlvl"
constexpr uint32_t kLevel = 384;                                  // qcom,vdd_lpi_*-uV-uA = <384 0>
constexpr unsigned kReadyMs = 10000, kProxyMs = 10000, kPollMs = 50;

// --- SMEM, read-only uncached view ----------------------------------------------------------------------------------
struct Smem {
  IOMemoryMap *map = nullptr;
  smem::Info *info = nullptr;
  smem::Reader reader = {};
  bool open() {
    if (pas::overlap(wifiprep::kSmemPhys, wifiprep::kSmemBytes, guamp_phys_base, guamp_phys_size)) return false;
    IOMemoryDescriptor *d = IOMemoryDescriptor::withPhysicalAddress(wifiprep::kSmemPhys, wifiprep::kSmemBytes, kIODirectionIn);
    map = d ? d->map(kIOMapInhibitCache | kIOMapReadOnly) : nullptr;
    if (d) d->release();
    info = static_cast<smem::Info *>(IOMallocZero(sizeof(smem::Info)));
    if (!map || !info) return false;
    reader = {wifiprep::smemRead, reinterpret_cast<void *>(map->getVirtualAddress()), static_cast<uint32_t>(wifiprep::kSmemBytes)};
    return true;
  }
  void parse() {
    __asm__ volatile("dmb sy" ::: "memory");
    smem::parse(reader, info);
  }
  bool find(uint32_t item, uint32_t *at, uint32_t *bytes) { return !info->error && smem::find(reader, *info, kHost, item, at, bytes); }
  ~Smem() {
    if (info) IOFree(info, sizeof(smem::Info));
    if (map) map->release();
  }
};

// ADSP -> apps SMP2P "slave-kernel": false if the item is absent or not a valid entry from pid 2 to pid 0
bool slaveKernel(Smem &s, smem::Smp2p *state, uint32_t *value) {
  uint32_t at = 0, bytes = 0;
  if (!s.find(kSmp2pIn, &at, &bytes)) return false;
  smem::readSmp2p(s.reader, at, bytes, "slave-kernel", state);
  if (!state->valid || !state->hasEntry || state->localPid != kHost || state->remotePid != 0) return false;
  *value = state->value;
  return true;
}

void glinkReport(const char *when) {
  Smem s;
  if (!s.open()) return;
  s.parse();
  uint32_t at, bytes, size[3] = {};
  bool found[3];
  static const uint32_t items[3] = {kGlinkDesc, kGlinkTx, kGlinkRx};
  for (unsigned i = 0; i < 3; i++) found[i] = s.find(items[i], &at, &size[i]);
  const smem::Partition *p = s.info->error ? nullptr : smem::partitionFor(*s.info, kHost);
  IOLog("GuampAdsp: SMEM %s: apps-adsp partition %s, smp2p out %u %s, glink 478 %s %u / 479 %s %u / 480 %s %u%s\n", when,
        p ? (p->status ? "WALK ERROR" : "ok") : "absent", kSmp2pOut, s.find(kSmp2pOut, &at, &bytes) ? "present" : "absent",
        found[0] ? "present" : "absent", size[0], found[1] ? "present" : "absent", size[1], found[2] ? "present" : "absent",
        size[2], found[2] ? " (ADSP GLINK FIFO allocated: edge up on the remote side)" : "");
}

// --- pas::Hooks -----------------------------------------------------------------------------------------------------
bool stopped(void *) {
  Smem s;
  if (!s.open()) return false;
  s.parse();
  uint32_t at = 0, bytes = 0;
  const bool present = s.find(kSmp2pIn, &at, &bytes);
  IOLog("GuampAdsp: SMEM v%u initialized %u %s, adsp smp2p 429 %s\n", s.info->version, s.info->initialized,
        s.info->error ? "PARSE ERROR" : "ok", present ? "PRESENT (peer ran before)" : "absent");
  return !s.info->error && s.info->initialized && !present;
}

int levelVote(uint32_t type, uint32_t swen, uint32_t level) {
  const rpm_kvp kv[2] = {{RPM_KEY_SWEN, 4, swen}, {kKeyVlvl, 4, level}};
  int rc = rpm::request(RPM_SET_ACTIVE, type, 0, kv, 2, 500);
  if (!rc) rc = rpm::request(RPM_SET_SLEEP, type, 0, kv, 2, 500);   // qcom,set = <3>: active and sleep set
  return rc;
}

// pil_proxy_vote: vdd_lpi_cx and vdd_lpi_mx enabled at level 384, XO on. XO is voted in the active set only and never
// removed: that is the vote Linux keeps permanently (bi_tcxo_ao at rpmcc probe), so the unvote cannot drop a vote of
// another apps user. The rails are not voted by anything else on the apps side.
bool vote(void *, bool on) {
  if (!on) {
    const int mx = levelVote(kRwlm, 0, 0), cx = levelVote(kRwlc, 0, 0);
    IOLog("GuampAdsp: proxy unvote: lpi_mx %s, lpi_cx %s\n", rpm_err_name(mx), rpm_err_name(cx));
    return !mx && !cx;
  }
  const int xo = rpm::set(RPM_RES_CLK0, RPM_CLK0_XO, RPM_KEY_ENAB, 1, RPM_SET_ACTIVE, 500);
  const int cx = xo ? xo : levelVote(kRwlc, 1, kLevel);
  const int mx = cx ? cx : levelVote(kRwlm, 1, kLevel);
  IOLog("GuampAdsp: proxy vote: xo %s, lpi_cx %s, lpi_mx %s (level %u)\n", rpm_err_name(xo), xo ? "-" : rpm_err_name(cx),
        cx ? "-" : rpm_err_name(mx), kLevel);
  if (xo || cx || mx) {   // undo a rail vote that may have reached one set
    if (!xo) levelVote(kRwlc, 0, 0);
    if (!xo && !cx) levelVote(kRwlm, 0, 0);
    return false;
  }
  return true;
}

// wait_for_err_ready (10 s) on the err-ready bit, err-fatal ends it; then, as the proxy-unvote interrupt / the
// 10 s proxy timer of pil_proxy_vote, the handover bit or the timer releases the proxy votes.
int awaitReady(void *) {
  Smem s;
  smem::Smp2p *state = static_cast<smem::Smp2p *>(IOMallocZero(sizeof(smem::Smp2p)));
  if (!state || !s.open()) {
    if (state) IOFree(state, sizeof(smem::Smp2p));
    IOLog("GuampAdsp: SMEM not mapped: ready signal cannot be observed\n");
    return pas::kNotReady;
  }
  int result = pas::kNotReady;
  uint32_t last = ~0u, value = 0;
  bool seen = false;
  unsigned elapsed = 0, readyAt = 0;
  for (; elapsed <= kReadyMs + kProxyMs; elapsed += kPollMs) {
    s.parse();
    if (slaveKernel(s, state, &value)) {
      if (!seen) IOLog("GuampAdsp: smp2p 429 appeared after %u ms (pids %u->%u, %u/%u entries)\n", elapsed,
                       state->localPid, state->remotePid, state->validEntries, state->total);
      seen = true;
      if (value != last) {
        IOLog("GuampAdsp: slave-kernel %x at %u ms (fatal %u ready %u handover %u stop-ack %u)\n", value, elapsed,
              value & kFatal ? 1 : 0, value & kReady ? 1 : 0, value & kHandover ? 1 : 0, value & kStopAck ? 1 : 0);
        last = value;
      }
      if (value & kFatal) { result = pas::kFatal; break; }
      if ((value & kReady) && result != pas::kReady) {
        result = pas::kReady;
        readyAt = elapsed;
        if (adspipc::gLevel) adspipc::onReady();
      }
      if (result == pas::kReady && (value & kHandover)) break;
    }
    if (result != pas::kReady && elapsed >= kReadyMs) break;
    if (result == pas::kReady && elapsed >= kProxyMs) break;
    if (adspipc::gLevel) adspipc::service();
    IOSleep(kPollMs);
  }
  if (result == pas::kReady)
    IOLog("GuampAdsp: ready at %u ms, proxy votes released at %u ms (%s)\n", readyAt, elapsed,
          value & kHandover ? "handover" : "proxy timeout");
  else if (result == pas::kNotReady)
    IOLog("GuampAdsp: no err-ready within %u ms (smp2p 429 %s)\n", kReadyMs, seen ? "present" : "never allocated");
  IOFree(state, sizeof(smem::Smp2p));
  return result;
}

// log_failure_reason: SMEM item 423 (qcom,smem-id), a NUL-terminated text written by the ADSP
void failure(void *) {
  Smem s;
  if (!s.open()) return;
  s.parse();
  uint32_t at = 0, bytes = 0;
  if (!s.find(kReason, &at, &bytes) || !bytes) { IOLog("GuampAdsp: failure reason: item 423 absent\n"); return; }
  char text[97];
  uint32_t n = 0;
  for (; n < bytes && n + 1 < sizeof(text); n++) {
    const uint8_t c = s.reader.r8(at + n);
    if (!c) break;
    text[n] = c >= 32 && c < 127 ? static_cast<char>(c) : '?';
  }
  text[n] = 0;
  IOLog("GuampAdsp: failure reason (item 423, %u bytes): %s\n", bytes, n ? text : "(empty)");
}

// --- firmware variant (adsp-loader.c) -------------------------------------------------------------------------------
const char *firmwareName() {
  IOMemoryDescriptor *d = IOMemoryDescriptor::withPhysicalAddress(kQfpromPage, 0x1000, kIODirectionIn);
  IOMemoryMap *map = d ? d->map(kIOMapInhibitCache | kIOMapReadOnly) : nullptr;
  if (d) d->release();
  if (!map) { IOLog("GuampAdsp: qfprom not mapped: firmware adsp\n"); return "adsp"; }
  const uint32_t word = *reinterpret_cast<const volatile uint32_t *>(map->getVirtualAddress() + kVariantWord);
  map->release();
  const unsigned variant = word >> kVariantBit & 1;
  IOLog("GuampAdsp: adsp_variant fuse %u: firmware %s\n", variant, variant ? "adsp2" : "adsp");
  return variant ? "adsp2" : "adsp";
}

void run(unsigned mode) {
  IOLog("GuampAdsp: mode %u, ipc %u, PAS id %u, carveout %llx + %llx, XNU RAM %lx + %lx, smp2p in %u out %u, reason %u\n",
        mode, adspipc::gLevel, kPasId, kBase, kBytes, guamp_phys_base, guamp_phys_size, kSmp2pIn, kSmp2pOut, kReason);
  if (pas::overlap(kBase, kBytes, guamp_phys_base, guamp_phys_size)) {
    IOLog("GuampAdsp: carveout overlaps XNU RAM: refused\n");
    return;
  }
  const pas::Peripheral peripheral = {"GuampAdsp", kPasId, kBase, kBytes, "/image", firmwareName()};
  char part[8] = {'m', 'o', 'd', 'e', 'm', '_', wifiprep::activeSlot(), 0};
  wifiprep::Window window = {};
  uint64_t attributes = 0;
  if (!wifiprep::findPartition(part, &window.first, &window.blocks, &attributes)) {
    IOLog("GuampAdsp: partition %s not found\n", part);
    return;
  }
  uint8_t *scratch = static_cast<uint8_t *>(IOMallocZero(ext4::kScratchBytes));
  uint8_t *mdt = static_cast<uint8_t *>(IOMallocZero(fw::kMdtMax));
  uint8_t *chunk = static_cast<uint8_t *>(IOMallocZero(wifiprep::kChunkBytes));
  ext4::Fs *fs = static_cast<ext4::Fs *>(IOMallocZero(sizeof(ext4::Fs)));
  fw::Result *res = static_cast<fw::Result *>(IOMallocZero(sizeof(fw::Result)));
  fw::Image *image = static_cast<fw::Image *>(IOMallocZero(sizeof(fw::Image)));
  if (scratch && mdt && chunk && fs && res && image) {
    int r = fs->mount(wifiprep::readWindow, &window, window.blocks, scratch);
    if (r) {
      IOLog("GuampAdsp: %s ext4 mount: %s\n", part, ext4::errorName(r));
    } else {
      fw::verify(*fs, peripheral.dir, peripheral.firmware, mdt, chunk, wifiprep::kChunkBytes, res);
      IOLog("GuampAdsp: %s/%s firmware on %s: error %d, %u verified / %u failed / %u empty of %u, MBN v%u, %llu bytes, mask %llx\n",
            peripheral.dir, peripheral.firmware, part, res->error, res->verified, res->failed, res->empty, res->segments,
            res->mbnVersion, res->bytes, res->failedMask);
      const bool verified = !res->error && !res->failed && res->verified > 0;
      char path[128];
      ext4::Inode inode;
      int parsed = fw::joinName(path, sizeof(path), peripheral.dir, peripheral.firmware, ".mdt", -1) ? fs->lookupPath(path, &inode)
                                                                                                    : fw::kErrFormat;
      if (!parsed && inode.size <= fw::kMdtMax / 2) parsed = fw::parse(mdt, static_cast<uint32_t>(inode.size), image);
      else if (!parsed) parsed = fw::kErrTooBig;
      if (!parsed && image->hashWhere == 3) {
        const uint32_t hashAt = (static_cast<uint32_t>(inode.size) + 63) & ~63u;   // where fw::verify read it
        if (image->hashSegBytes > fw::kMdtMax - hashAt) parsed = fw::kErrFormat;
        else image->hashSeg = mdt + hashAt;
      }
      if (parsed) {
        IOLog("GuampAdsp: metadata not usable (%d); nothing started\n", parsed);
      } else {
        scm::Transport transport = {guampScmInstruction, [](void *, uint32_t ms) { IOSleep(ms); }, nullptr, true};
        pas::Hooks hooks;
        hooks.stopped = stopped;
        hooks.vote = vote;
        hooks.awaitReady = awaitReady;
        hooks.failure = failure;
        if (adspipc::gLevel) hooks.prepare = adspipc::prepare;
        const int rc = pas::boot(peripheral, hooks, transport, *fs, mdt, static_cast<uint32_t>(inode.size), *image, chunk,
                                 wifiprep::kChunkBytes, verified, mode, guamp_phys_base, guamp_phys_size);
        IOLog("GuampAdsp: RESULT %d (%s)\n", rc,
              rc == pas::kUp ? "ADSP up" : rc == pas::kQueried ? "queries only" : "not started, cleaned up");
        if (mode == 1) glinkReport("after start");
        if (rc == pas::kUp && adspipc::gLevel) {
          adspipc::run();   // returns only on err-fatal or a link error
          failure(nullptr);
        }
      }
    }
  } else {
    IOLog("GuampAdsp: out of memory\n");
  }
  if (scratch) IOFree(scratch, ext4::kScratchBytes);
  if (mdt) IOFree(mdt, fw::kMdtMax);
  if (chunk) IOFree(chunk, wifiprep::kChunkBytes);
  if (fs) IOFree(fs, sizeof(ext4::Fs));
  if (res) IOFree(res, sizeof(fw::Result));
  if (image) IOFree(image, sizeof(fw::Image));
}

unsigned gMode;

void thread(void *, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-adsp");
  unsigned delay = 0;
  PE_parse_boot_argn("guamp-adsp-delay", &delay, sizeof(delay));
  if (delay && delay <= 600) {
    IOSleep(delay * 1000);
    IOLog("GuampAdsp: start delayed %u s (guamp-adsp-delay)\n", delay);
  }
  run(gMode);
}

// Once, after rpm::start(): boot argument guamp-adsp=1|2
void start() {
  static bool started = false;
  unsigned mode = 0;
  if (started || !PE_parse_boot_argn("guamp-adsp", &mode, sizeof(mode)) || (mode != 1 && mode != 2)) return;
  if (!gCard.online || !gCard.gate) return;
  started = true;
  gMode = mode;
  unsigned ipc = 0;
  if (mode == 1 && PE_parse_boot_argn("guamp-adsp-ipc", &ipc, sizeof(ipc)) && ipc <= 3) adspipc::gLevel = ipc;
  thread_t t = nullptr;
  if (kernel_thread_start(thread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
}

}  // namespace adsp
