// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp RPM client (Resource Power Manager, the Cortex-M power controller), included by NativeEmmc.cpp before the
// services that use it (mapPhysical, the boot log). GLINK over the RPM message RAM + rpm-smd requests, the protocol
// in GuampRpmCore.h (host-tested by tests/rpm_host.cpp). Polled (no IRQ), one request at a time under a lock, every
// wait bounded. Only the RPM edge: no modem/remote processor, no SMEM, no TrustZone call.
// API (any thread after rpm::start()):
//   rpm::set(type, id, key, value[, set, timeoutMs])   one 4-byte key/value, e.g. set(RPM_RES_CLK0, 0, RPM_KEY_ENAB, 1)
//   rpm::request(set, type, id, kvp, n, timeoutMs)      several keys in one request (rpm-smd-regulator style)
//   both return RPM_OK (acked) or an RPM_ERR_* (rpm_err_name); the channel is opened on first use and reopened once
//   after a remote close or a protocol error.
// Boot arguments:
//   guamp-rpm-test=1   self-test on its own thread: TOC and FIFO indices (read only), handshake, then one vote for
//                      something that is on whenever the CPU runs (clk0 0 = XO, Enab 1, active set: Linux's
//                      bi_tcxo_ao vote at rpmcc probe); "GuampRpm: RESULT ..." line. =2: the read-only part only.
//   guamp-rpm-delay=<s> start the self-test later (USB mirror). guamp-rpm-ring=1 keeps the FIFO indices found at
//                      start instead of Linux's reset to 0. guamp-rpm-fmt=1 rpm-smd v1 format (bengal/Linux: v0).
//   guamp-rpm-log=0    no per-request log line (default 1).

#include "GuampRpmCore.h"

namespace rpm {
volatile uint8_t *gRam, *gApcs;
IOLock *gLock;
rpm_link gLink;
unsigned gLog = 1, gRing = RPM_RING_LINUX, gFmt = RPM_FMT_V0, gOpens;
int gOpenRc = RPM_ERR_CLOSED;

uint32_t ramRd(void *, uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(gRam + off); }
void ramWr(void *, uint32_t off, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(gRam + off) = v; }
void barrier(void *) { __asm__ volatile("dsb sy" ::: "memory"); }
void kick(void *) {
  __asm__ volatile("dsb sy" ::: "memory");
  *reinterpret_cast<volatile uint32_t *>(gApcs + RPM_APCS_IPC) = RPM_APCS_BIT;
  __asm__ volatile("dsb sy" ::: "memory");
}
void delayUs(void *, unsigned us) {
  if (us >= 1000) IOSleep(us / 1000);
  else IODelay(us);
}
const rpm_io gIo = {nullptr, ramRd, ramWr, barrier, kick, delayUs};

bool mapAll() {
  if (!gRam) gRam = mapPhysical(RPM_MSG_RAM_PHYS, RPM_MSG_RAM_SIZE);
  if (!gApcs) gApcs = mapPhysical(RPM_APCS_PHYS, RPM_APCS_SIZE);
  return gRam && gApcs;
}

void logLink(const char *what, int rc) {
  const rpm_link &l = gLink;
  IOLog("GuampRpm: %s %s: toc %08x count %u bad %u tx %x/%u tail %u head %u rx %x/%u tail %u head %u (as found), "
        "ring %s, fmt v%u, version sends %u acked %d %s open rcid %u open-acked %d, rx cmds %u kicks %u, broken %d cmd %u\n",
        what, rpm_err_name(rc), l.toc_magic, l.toc_count, l.toc_bad, l.tx.desc, l.tx.len, l.tx_tail0, l.tx_head0, l.rx.desc,
        l.rx.len, l.rx_tail0, l.rx_head0, gRing == RPM_RING_RESUME || gOpens > 1 ? "resume" : "reset", l.fmt, l.version_sends,
        l.version_acked, l.local_first ? "local" : l.remote_open ? "remote" : "no", l.rcid, l.open_acked, l.rx_cmds, l.kicks,
        l.broken, l.bad_cmd);
}

// Caller holds gLock. The first session follows guamp-rpm-ring (Linux: indices reset); a later one (after a remote
// close or a protocol error) keeps the indices both sides have been using.
int openLocked() {
  if (gLink.up && !gLink.broken) return RPM_OK;
  if (!mapAll()) return gOpenRc = RPM_ERR_NOMAP;
  const unsigned ring = gOpens ? RPM_RING_RESUME : gRing;
  rpm_link_init(&gLink, &gIo, RPM_MSG_RAM_SIZE, gFmt);
  gOpenRc = rpm_link_open(&gLink, ring, 1000000, 100000, 50);
  gOpens = gOpens + 1;
  logLink(gOpens == 1 ? "open" : "reopen", gOpenRc);
  return gOpenRc;
}

void logRequest(unsigned which, uint32_t type, uint32_t id, const rpm_kvp *kv, unsigned n, int rc, const rpm_result &r) {
  if (!gLog) return;
  char t[5], k0[5], k1[5];
  rpm_fourcc(type, t);
  rpm_fourcc(kv[0].key, k0);
  rpm_fourcc(n > 1 ? kv[1].key : 0, k1);
  const char *s = which == RPM_SET_SLEEP ? "slp" : "act";
  if (n == 1)
    IOLog("GuampRpm: %s %s %u %s=%llu -> %s (id %u, %u us)%s%s code %u\n", s, t, id, k0,
          static_cast<unsigned long long>(kv[0].value), rpm_err_name(rc), r.msg_id, r.us, r.text[0] ? " rpm: " : "", r.text, r.code);
  else
    IOLog("GuampRpm: %s %s %u %s=%llu %s=%llu%s -> %s (id %u, %u us)%s%s code %u\n", s, t, id, k0,
          static_cast<unsigned long long>(kv[0].value), k1, static_cast<unsigned long long>(kv[1].value), n > 2 ? " +more" : "",
          rpm_err_name(rc), r.msg_id, r.us, r.text[0] ? " rpm: " : "", r.text, r.code);
}

int request(unsigned which, uint32_t type, uint32_t id, const rpm_kvp *kv, unsigned n, unsigned timeoutMs = 500) {
  if (!gLock || !kv || !n || n > RPM_MAX_KVP) return RPM_ERR_PARAM;
  if (!timeoutMs || timeoutMs > 10000) timeoutMs = 500;
  rpm_result r{};
  IOLockLock(gLock);
  int rc = openLocked();
  if (!rc) {
    rc = rpm_link_request(&gLink, which, type, id, kv, n, timeoutMs * 1000u, 50, &r);
    if (rc == RPM_ERR_CLOSED || rc == RPM_ERR_PROTO) {   // remote close / corrupt FIFO: one fresh session, one retry
      logLink("lost", rc);
      gLink.up = false;
      rc = openLocked();
      if (!rc) rc = rpm_link_request(&gLink, which, type, id, kv, n, timeoutMs * 1000u, 50, &r);
    }
  }
  IOLockUnlock(gLock);
  logRequest(which, type, id, kv, n, rc, r);
  return rc;
}

int set(uint32_t type, uint32_t id, uint32_t key, uint32_t value, unsigned which = RPM_SET_ACTIVE, unsigned timeoutMs = 500) {
  const rpm_kvp kv = {key, 4, value};
  return request(which, type, id, &kv, 1, timeoutMs);
}

// --- self-test ----------------------------------------------------------------------------------------------------
void testThread(void *arg, wait_result_t) {
  thread_set_thread_name(current_thread(), "guamp-rpm-test");
  const unsigned mode = static_cast<unsigned>(reinterpret_cast<uintptr_t>(arg));
  unsigned delay = 0;
  PE_parse_boot_argn("guamp-rpm-delay", &delay, sizeof(delay));
  if (delay && delay <= 600) IOSleep(delay * 1000);
  // Read-only part: map, TOC, indices as found (nothing written).
  IOLockLock(gLock);
  if (!mapAll()) {
    IOLockUnlock(gLock);
    IOLog("GuampRpm: RESULT toc - link - vote - (message RAM / APCS map failed)\n");
    return;
  }
  rpm_link probe;
  rpm_link_init(&probe, &gIo, RPM_MSG_RAM_SIZE, gFmt);
  const int toc = rpm_parse_toc(&probe);
  IOLockUnlock(gLock);
  IOLog("GuampRpm: toc %s magic %08x count %u bad %u | ap2r @%x len %u tail %u head %u (pending %u) | r2ap @%x len %u tail %u "
        "head %u (pending %u)\n", rpm_err_name(toc), probe.toc_magic, probe.toc_count, probe.toc_bad, probe.tx.desc, probe.tx.len,
        probe.tx_tail0, probe.tx_head0,
        toc ? 0u : rpm_ring_used(probe.tx_head0 % probe.tx.len, probe.tx_tail0 % probe.tx.len, probe.tx.len), probe.rx.desc,
        probe.rx.len, probe.rx_tail0, probe.rx_head0,
        toc ? 0u : rpm_ring_used(probe.rx_head0 % probe.rx.len, probe.rx_tail0 % probe.rx.len, probe.rx.len));
  if (toc || mode == 2) {
    IOLog("GuampRpm: RESULT toc %s link - vote - (%s)\n", rpm_err_name(toc), mode == 2 ? "read-only test" : "no FIFO, nothing written");
    return;
  }
  // Handshake, then a vote that cannot switch anything off: XO enable in the active set (on while any CPU runs).
  IOLockLock(gLock);
  const int link = openLocked();
  IOLockUnlock(gLock);
  int vote = RPM_ERR_CLOSED;
  if (!link) vote = set(RPM_RES_CLK0, RPM_CLK0_XO, RPM_KEY_ENAB, 1, RPM_SET_ACTIVE, 500);
  IOLog("GuampRpm: RESULT toc ok link %s (%s) vote xo %s | acks %u stale %u read-notif %u intents %u signals %u closes %u "
        "other-opens %u dropped %u\n", rpm_err_name(link), link ? "-" : gLink.local_first ? "local open" : "remote open",
        rpm_err_name(vote), gLink.acks, gLink.stale_acks, gLink.read_notifs, gLink.intents, gLink.signals, gLink.closes,
        gLink.other_opens, gLink.dropped);
}

void start() {
  gLock = IOLockAlloc();
  unsigned v = 0;
  if (PE_parse_boot_argn("guamp-rpm-ring", &v, sizeof(v)) && v == 1) gRing = RPM_RING_RESUME;
  v = 0;
  if (PE_parse_boot_argn("guamp-rpm-fmt", &v, sizeof(v)) && v == 1) gFmt = RPM_FMT_V1;
  v = 1;
  if (PE_parse_boot_argn("guamp-rpm-log", &v, sizeof(v))) gLog = v ? 1u : 0u;
  unsigned test = 0;
  if (!gLock || !PE_parse_boot_argn("guamp-rpm-test", &test, sizeof(test)) || (test != 1 && test != 2)) return;
  thread_t t = nullptr;
  if (kernel_thread_start(testThread, reinterpret_cast<void *>(static_cast<uintptr_t>(test)), &t) == KERN_SUCCESS)
    thread_deallocate(t);
}
}  // namespace rpm

#if defined(GUAMP_WIFI_RUNTIME) && GUAMP_WIFI_RUNTIME
// Single owner of the RPM edge: the Wi-Fi provider (GuampWifiRpm.inc, separate object) votes through this client
// instead of opening a second GLINK session on the same FIFO. Ready once rpm::start() has created the lock.
extern "C" int guamp_rpm_shared_ready(void) { return rpm::gLock != nullptr; }
extern "C" int guamp_rpm_shared_vote(uint32_t resource, uint32_t id, uint32_t key, uint32_t value, uint32_t state,
                                     uint32_t timeoutMs) {
  if (state > RPM_SET_SLEEP) return RPM_ERR_PARAM;
  return rpm::set(resource, id, key, value, state, timeoutMs);
}
#endif
