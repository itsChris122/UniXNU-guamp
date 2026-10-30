/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/*
 * XNU bridge for the ath10k/WCN3990 station ABI. It targets the Redwood BSD
 * station/network boundary used by the HackiOS XNU reference; board setup,
 * modem/WLFW, MSA, SMMU, regulators and the iOS Apple80211 provider are separate.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <net/ethernet.h>
#include <net/if_ether.h>
#include <sys/errno.h>
#include <sys/kpi_mbuf.h>
#include <pexpert/pexpert.h>
#if __has_include(<sys/redwood_net.h>) && __has_include(<sys/redwood_sta.h>)
#include <sys/redwood_net.h>
#include <sys/redwood_sta.h>
#elif __has_include(<sys/hakios.net.h>) && __has_include(<sys/hakios.sta.h>)
/* The current HackiOS reference renamed its public station/net boundary.
 * Use the real declarations from that checkout rather than copied ABI structs.
 * This verifies the source boundary, not the selected iOS userland ABI. */
#include <sys/hakios.net.h>
#include <sys/hakios.sta.h>
#define REDWOOD_NET_FRAME_MAX HAKIOS_NET_FRAME_MAX
#define redwood_net_link hakios_net_link
#define redwood_net_attach hakios_net_attach
#define redwood_net_input hakios_net_input
#define redwood_net_link_state hakios_net_link_state
#define redwood_sta_bss hakios_sta_bss
#define redwood_sta_scan hakios_sta_scan
#define redwood_sta_join hakios_sta_join
#define redwood_sta_phase hakios_sta_phase
#define redwood_sta_state hakios_sta_state
#define redwood_sta_channel hakios_sta_channel
#define redwood_sta_ops hakios_sta_ops
#define redwood_sta_attach hakios_sta_attach
#define redwood_sta_lost hakios_sta_lost
#define REDWOOD_STA_OFF HAKIOS_STA_OFF
#define REDWOOD_STA_IDLE HAKIOS_STA_IDLE
#define REDWOOD_STA_SCANNING HAKIOS_STA_SCANNING
#define REDWOOD_STA_JOINING HAKIOS_STA_JOINING
#define REDWOOD_STA_JOINED HAKIOS_STA_JOINED
#define REDWOOD_STA_OPEN HAKIOS_STA_OPEN
#define REDWOOD_STA_WPA2_PSK HAKIOS_STA_WPA2_PSK
#define REDWOOD_STA_CHANNEL_PASSIVE HAKIOS_STA_CHANNEL_PASSIVE
#define REDWOOD_STA_CHANNEL_RADAR HAKIOS_STA_CHANNEL_RADAR
#else
#error "Guamp station requires matching Redwood or Hakios net/station headers"
#endif

#include "../ath10k-port/gw_snoc_abi.h"
#include "../ath10k-port/nw_abi.h"
#include "guamp_xnu_station.h"

#define GUAMP_WIFI_FW_NOT_FOUND (-111) /* GuampWifiWire.inc: guampwifi::NotFound */
#define GUAMP_WIFI_FW_NOT_READY (-105) /* GuampWifiWire.inc: guampwifi::NotReady */

static lck_grp_t *guamp_wifi_group;
static lck_mtx_t *guamp_wifi_event_lock;
static lck_spin_t *guamp_wifi_state_lock;
/* Public kernel lock allocation avoids XNU-private startup-section macros.
 * attach's singleton claim serializes this initialization before callbacks. */
static int station_locks_init(void)
{
  if (guamp_wifi_group) return 0;
  lck_grp_t *group = lck_grp_alloc_init("guamp-wifi", LCK_GRP_ATTR_NULL);
  if (!group) return ENOMEM;
  lck_mtx_t *event = lck_mtx_alloc_init(group, LCK_ATTR_NULL);
  lck_spin_t *state = lck_spin_alloc_init(group, LCK_ATTR_NULL);
  if (!event || !state) {
    if (event) lck_mtx_free(event, group);
    if (state) lck_spin_free(state, group);
    lck_grp_free(group); return ENOMEM;
  }
  guamp_wifi_event_lock = event; guamp_wifi_state_lock = state;
  guamp_wifi_group = group; return 0;
}
static const struct lx_platform *g_platform;
static struct lx_platform g_wifi_platform;
static int (*g_fallback_fw_get)(const char *, const uint8_t **, size_t *);
extern int guamp_ath10k_firmware_get(const char *, const uint8_t **, size_t *);

/* Prefer Guamp's staged API5 metadata and OEM BDF; leave unrelated firmware
 * requests to the board's existing provider. The backing store is persistent. */
static int guamp_fw_get(const char *name, const uint8_t **data, size_t *size)
{
  const char *base = name;
  if (name) for (const char *p = name; *p; ++p)
    if (*p == '/' || *p == '\\') base = p + 1;
  /* Prefer an exact firmware-5.bin from the caller; the generic upstream
   * metadata is only the fallback because its feature match is unverified. */
  if (name && !strcmp(base, "firmware-5.bin") && g_fallback_fw_get) {
    int error = g_fallback_fw_get(name, data, size);
    if (error == 0 || (error != -ENOENT && error != ENOENT)) return error;
  }
  int error = guamp_ath10k_firmware_get(name, data, size);
  if (error == 0) return 0;
  if ((error != GUAMP_WIFI_FW_NOT_FOUND && error != GUAMP_WIFI_FW_NOT_READY) ||
      !g_fallback_fw_get) return error;
  return g_fallback_fw_get(name, data, size);
}

static struct redwood_net_link g_link;
static struct redwood_sta_state g_public_state;
static volatile int g_worker_started;
enum { GUAMP_WORKER_STOPPED = 0, GUAMP_WORKER_ATTACHING = -1,
       GUAMP_WORKER_RUNNING = 1, GUAMP_WORKER_FAILED = -2 };
static int g_scan_done, g_scan_result;
static int g_join_done, g_join_result;
static int g_power_done, g_power_result, g_power_value;
static int g_disconnect_done, g_disconnect_result;

static bool worker_online(void)
{
  return __atomic_load_n(&g_worker_started, __ATOMIC_ACQUIRE) == GUAMP_WORKER_RUNNING;
}

static void public_state_set(enum redwood_sta_phase phase, int error)
{
  lck_spin_lock(guamp_wifi_state_lock);
  g_public_state.phase = phase;
  g_public_state.last_error = error;
  lck_spin_unlock(guamp_wifi_state_lock);
}

static void public_state_copy(struct redwood_sta_state *state)
{
  bzero(state, sizeof(*state));
  lck_spin_lock(guamp_wifi_state_lock);
  state->phase = g_public_state.phase;
  state->last_error = g_public_state.last_error;
  memcpy(state->bss.bssid, g_public_state.bss.bssid, sizeof(state->bss.bssid));
  memcpy(state->bss.ssid, g_public_state.bss.ssid, g_public_state.bss.ssid_length);
  state->bss.ssid_length = g_public_state.bss.ssid_length;
  state->bss.frequency = g_public_state.bss.frequency;
  state->bss.rssi = g_public_state.bss.rssi;
  state->bss.capability = g_public_state.bss.capability;
  state->bss.beacon_interval = g_public_state.bss.beacon_interval;
  state->bss.elements_length = g_public_state.bss.elements_length;
  memcpy(state->bss.elements, g_public_state.bss.elements, g_public_state.bss.elements_length);
  lck_spin_unlock(guamp_wifi_state_lock);
}

static enum redwood_sta_phase public_phase(void)
{
  lck_spin_lock(guamp_wifi_state_lock);
  enum redwood_sta_phase phase = g_public_state.phase;
  lck_spin_unlock(guamp_wifi_state_lock);
  return phase;
}

static void request_failed(enum redwood_sta_phase pending,
    enum redwood_sta_phase previous, int error)
{
  lck_spin_lock(guamp_wifi_state_lock);
  /* A concurrent loss/power-off notification takes precedence over the
   * caller's saved phase, especially when an operation timed out. */
  if (g_public_state.phase == pending) g_public_state.phase = previous;
  g_public_state.last_error = error;
  lck_spin_unlock(guamp_wifi_state_lock);
}

/* Worker callbacks can safely take the station's current driver snapshot.
 * Caller threads use only g_public_state, never the mutable driver state. */
static bool public_state_from_driver(int error)
{
  struct nw_wifi_status status;
  nw_wifi_status_get(&status);
  enum redwood_sta_phase phase = REDWOOD_STA_IDLE;
  if (status.state == NW_STA_OFF) phase = REDWOOD_STA_OFF;
  else if (status.state == NW_STA_UP) phase = REDWOOD_STA_JOINED;
  else if (status.state >= NW_STA_AUTH) phase = REDWOOD_STA_JOINING;
  else if (status.state == NW_STA_SCANNING) phase = REDWOOD_STA_SCANNING;
  /* guamp 29/10: the joined network's elements, capability and beacon interval from the scan list, so that
   * CURRENT_NETWORK (Apple80211 103) carries the RSN element and wifid sees the same security it asked for.
   * Looked up before the spin lock: nw_wifi_bss_get takes the station's own lock. */
  static struct nw_bss_info joined;
  bool have_joined = false;
  if (status.state >= NW_STA_AUTH) {
    int count = nw_wifi_bss_count();
    for (int i = 0; i < count && !have_joined; ++i)
      if (nw_wifi_bss_get(i, &joined) == 0 && memcmp(joined.bssid, status.bssid, 6) == 0 &&
          joined.freq_mhz == status.freq_mhz) have_joined = true;
  }
  lck_spin_lock(guamp_wifi_state_lock);
  g_public_state.phase = phase; g_public_state.last_error = error;
  bzero(&g_public_state.bss, sizeof(g_public_state.bss));
  if (status.state >= NW_STA_AUTH) {
    memcpy(g_public_state.bss.bssid, status.bssid, sizeof(status.bssid));
    size_t ssid_length = status.ssid_len < sizeof(g_public_state.bss.ssid) ?
        status.ssid_len : sizeof(g_public_state.bss.ssid);
    memcpy(g_public_state.bss.ssid, status.ssid, ssid_length);
    g_public_state.bss.ssid_length = (uint8_t)ssid_length;
    g_public_state.bss.frequency = (uint16_t)status.freq_mhz;
    g_public_state.bss.rssi = status.signal_dbm;
    if (have_joined) {
      size_t n = joined.ie_len < sizeof(g_public_state.bss.elements) ? joined.ie_len : sizeof(g_public_state.bss.elements);
      if (n > sizeof(joined.ie)) n = sizeof(joined.ie);
      memcpy(g_public_state.bss.elements, joined.ie, n);
      g_public_state.bss.elements_length = (uint16_t)n;
      g_public_state.bss.capability = joined.caps;
      g_public_state.bss.beacon_interval = joined.beacon_int;
    }
  }
  lck_spin_unlock(guamp_wifi_state_lock);
  return status.state == NW_STA_UP;
}

static void worker_failed(int error)
{
  /* Keep this terminal: a stopped worker cannot be reattached to its live
   * firmware/net link. Do not call sta_attach(NULL) here: it can wait for a
   * station call that is itself waiting for this worker. */
  __atomic_store_n(&g_worker_started, GUAMP_WORKER_FAILED, __ATOMIC_RELEASE);
  public_state_set(REDWOOD_STA_OFF, error);
  redwood_net_link_state(false);
  lck_mtx_lock(guamp_wifi_event_lock);
  g_scan_result = g_join_result = g_power_result = g_disconnect_result = NW_RES_FAILED;
  g_scan_done = g_join_done = g_power_done = g_disconnect_done = 1;
  lck_mtx_unlock(guamp_wifi_event_lock);
  thread_wakeup((event_t)&g_scan_done); thread_wakeup((event_t)&g_join_done);
  thread_wakeup((event_t)&g_power_done); thread_wakeup((event_t)&g_disconnect_done);
  redwood_sta_lost(error);
}

static int
wait_event(int *done, int timeout_ms)
{
  uint64_t deadline;
  clock_interval_to_deadline((uint32_t)timeout_ms, NSEC_PER_MSEC, &deadline);
  lck_mtx_lock(guamp_wifi_event_lock);
  for (;;) {
    if (!worker_online()) { lck_mtx_unlock(guamp_wifi_event_lock); return ENETDOWN; }
    if (*done) break;
    if (lck_mtx_sleep_deadline(guamp_wifi_event_lock, LCK_SLEEP_DEFAULT,
        (event_t)done, THREAD_UNINT, deadline) == THREAD_TIMED_OUT) {
      int error = worker_online() ? ETIMEDOUT : ENETDOWN;
      lck_mtx_unlock(guamp_wifi_event_lock);
      return error;
    }
  }
  *done = 0;
  lck_mtx_unlock(guamp_wifi_event_lock);
  return 0;
}

static int result_to_errno(int result)
{
  switch (result) {
  case NW_RES_OK: return 0;
  case NW_RES_NOT_FOUND: return ENOENT;
  case NW_RES_TIMEOUT: return ETIMEDOUT;
  case NW_RES_REJECTED: return ECONNREFUSED;
  case NW_RES_BAD_KEY: return EAUTH;
  case NW_RES_UNSUPPORTED: return EOPNOTSUPP;
  default: return EIO;
  }
}

/* The reusable station is compiled with Linux errno values; Darwin's
 * ENETDOWN differs numerically, so translate the C ABI explicitly. */
static int request_to_errno(int result)
{
  if (result == -19 || result == -100) return ENETDOWN;
  if (result == -16) return EBUSY;
  if (result == -12) return ENOMEM;
  return EINVAL;
}

static void on_scan(int result)
{
  if (__atomic_load_n(&g_worker_started, __ATOMIC_ACQUIRE) == GUAMP_WORKER_FAILED) return;
  (void)public_state_from_driver(result_to_errno(result));
  lck_mtx_lock(guamp_wifi_event_lock);
  g_scan_result = result; g_scan_done = 1;
  lck_mtx_unlock(guamp_wifi_event_lock);
  thread_wakeup((event_t)&g_scan_done);
}

static void on_join(int result)
{
  if (__atomic_load_n(&g_worker_started, __ATOMIC_ACQUIRE) == GUAMP_WORKER_FAILED) return;
  bool joined = public_state_from_driver(result_to_errno(result));
  lck_mtx_lock(guamp_wifi_event_lock);
  g_join_result = result; g_join_done = 1;
  lck_mtx_unlock(guamp_wifi_event_lock);
  redwood_net_link_state(joined);
  thread_wakeup((event_t)&g_join_done);
}

static void on_lost(int reason)
{
  if (__atomic_load_n(&g_worker_started, __ATOMIC_ACQUIRE) == GUAMP_WORKER_FAILED) return;
  public_state_set(REDWOOD_STA_IDLE, reason ? ECONNRESET : 0);
  redwood_net_link_state(false);
  redwood_sta_lost(ECONNRESET);
}

static void on_rx(const uint8_t *frame, size_t length)
{
  /* nw_wifi_step dispatches RX in worker context, never from the CE interrupt. */
  if (worker_online()) redwood_net_input(frame, length);
}

static void on_power(int on, int result)
{
  if (__atomic_load_n(&g_worker_started, __ATOMIC_ACQUIRE) == GUAMP_WORKER_FAILED) return;
  bool joined = public_state_from_driver(result_to_errno(result));
  lck_mtx_lock(guamp_wifi_event_lock);
  g_power_value = on; g_power_result = result; g_power_done = 1;
  lck_mtx_unlock(guamp_wifi_event_lock);
  redwood_net_link_state(joined);
  thread_wakeup((event_t)&g_power_done);
}

static void on_disconnect(int result)
{
  if (__atomic_load_n(&g_worker_started, __ATOMIC_ACQUIRE) == GUAMP_WORKER_FAILED) return;
  (void)public_state_from_driver(result_to_errno(result));
  lck_mtx_lock(guamp_wifi_event_lock);
  g_disconnect_result = result; g_disconnect_done = 1;
  lck_mtx_unlock(guamp_wifi_event_lock);
  redwood_net_link_state(false);
  thread_wakeup((event_t)&g_disconnect_done);
}

static const struct nw_wifi_events g_events = {
  .scan_done = on_scan,
  .connect_done = on_join,
  .link_lost = on_lost,
  .rx = on_rx,
  .power_done = on_power,
  .disconnect_done = on_disconnect,
};

static bool link_transmit(mbuf_t packet, size_t length)
{
  if (!worker_online()) return false;
  uint8_t frame[REDWOOD_NET_FRAME_MAX];
  if (length < ETHER_HDR_LEN || length > sizeof(frame) ||
      mbuf_copydata(packet, 0, length, frame) != 0) return false;
  /* guamp 30/10: why en0 drops frames on the way out (fin10: 2809 of 35234): queue full (128) or link not authorized */
  static uint32_t refused, full, down;
  int error = nw_wifi_tx(frame, length);
  if (error) {
    refused++;
    if (error == -ENOBUFS) full++;
    else if (error == -ENETDOWN) down++;
    if (refused <= 8 || (refused & 255) == 0)
      printf("GuampWiFi: net: frame of %u bytes refused by the station: %d (refused %u: queue full %u, link down %u)\n",
          (unsigned)length, error, refused, full, down);
  }
  return error == 0;
}

static int station_power(bool on)
{
  if (!worker_online()) return ENETDOWN;
  lck_mtx_lock(guamp_wifi_event_lock); g_power_done = 0;
  lck_mtx_unlock(guamp_wifi_event_lock);
  int queued = nw_wifi_power(on);
  if (queued) return request_to_errno(queued);
  int error = wait_event(&g_power_done, 2000);
  if (error) return error;
  if (g_power_value != !!on) return EIO;
  return result_to_errno(g_power_result);
}

static int station_scan(const struct redwood_sta_scan *request)
{
  if (!worker_online()) return ENETDOWN;
  if (!request) return EINVAL;
  enum redwood_sta_phase previous = public_phase();
  lck_mtx_lock(guamp_wifi_event_lock); g_scan_done = 0;
  lck_mtx_unlock(guamp_wifi_event_lock);
  struct nw_wifi_scan_request scan;
  if (request->ssid_length > sizeof(scan.ssid) || request->frequency_count > 64) return EINVAL;
  bzero(&scan, sizeof(scan));
  scan.ssid_len = request->ssid_length;
  if (scan.ssid_len) memcpy(scan.ssid, request->ssid, scan.ssid_len);
  scan.passive = request->passive;
  scan.dwell_ms = request->dwell_ms;
  scan.frequency_count = request->frequency_count;
  for (unsigned i = 0; i < scan.frequency_count; ++i) scan.frequencies_mhz[i] = request->frequencies[i];
  /* guamp 30/10: a scan asked while joined (wifid's background scans) leaves the phase JOINED. Phase SCANNING made SSID, BSSID,
   * RSSI and CURRENT_NETWORK answer ENXIO for the whole scan (334 refusals in fin10), so wifid saw the network come and go:
   * empty signal icon with the link up. The station keeps its connection while it scans (do_scan does not touch C.state). */
  const bool joined = previous == REDWOOD_STA_JOINED;
  if (!joined) public_state_set(REDWOOD_STA_SCANNING, 0);
  int error = nw_wifi_scan_ex(&scan);
  if (error) {
    error = request_to_errno(error);
    request_failed(REDWOOD_STA_SCANNING, previous, error);
    return error;
  }
  error = wait_event(&g_scan_done, 35000);
  if (!error) error = result_to_errno(g_scan_result);
  if (error) request_failed(REDWOOD_STA_SCANNING, previous, error);
  /* guamp 29/10: scans asked by iOS, said in the log (the first 20) */
  static unsigned said;
  if (said < 20) {
    said++;
    printf("GuampWiFi: scan %u for iOS %s: %d BSS entries, %u channels asked, errno %d\n", said, error ? "FAILED" : "done",
        nw_wifi_bss_count(), (unsigned)scan.frequency_count, error);
  }
  return error;
}

/* guamp 29/10: mode 2 does the scan of mode 1 by itself before iOS gets the station (guamp-wifi-selfscan, default 1),
 * with the same lines, so one boot shows both the radio (SCAN COMPLETE, BSS) and the iOS path. */
static void station_self_scan(void)
{
  unsigned selfscan = 1;
  PE_parse_boot_argn("guamp-wifi-selfscan", &selfscan, sizeof(selfscan));
  if (!selfscan) { printf("GuampWiFi: mode 2 kernel scan skipped (guamp-wifi-selfscan=0)\n"); return; }
  struct redwood_sta_scan all;
  bzero(&all, sizeof(all));
  int error = station_scan(&all);
  if (error) {
    printf("GuampWiFi: mode 2 kernel scan FAILED, errno %d (iOS may still scan)\n", error);
    return;
  }
  int count = nw_wifi_bss_count();
  printf("GuampWiFi: SCAN COMPLETE: %d BSS entries (mode 2 kernel scan; zero is a valid completed scan)\n", count);
  static struct nw_bss_info bss;
  for (int i = 0; i < count; ++i) {
    if (nw_wifi_bss_get(i, &bss) != 0) continue;
    const uint8_t *rsn = NULL;
    for (unsigned at = 0; at + 2 <= bss.ie_len && at + 2 + bss.ie[at + 1] <= bss.ie_len; at += 2 + bss.ie[at + 1])
      if (bss.ie[at] == 48) { rsn = bss.ie + at; break; }
    /* the RSN element says what the network needs: WPA2-PSK (AKM 2), SAE (8), 802.1X (1); the SSID stays out of the log */
    unsigned akms = 0;   /* bit n: AKM suite 00-0f-ac:n offered */
    if (rsn && rsn[1] >= 8) {
      const unsigned end = 2u + rsn[1];
      unsigned pairwise = rsn[8] | rsn[9] << 8, at = 10 + 4 * pairwise;
      unsigned n = at + 2 <= end ? (rsn[at] | rsn[at + 1] << 8) : 0;
      for (unsigned k = 0; k < n && at + 2 + 4 * k + 4 <= end; ++k)
        if (rsn[at + 2 + 4 * k] == 0x00 && rsn[at + 3 + 4 * k] == 0x0f && rsn[at + 4 + 4 * k] == 0xac && rsn[at + 5 + 4 * k] < 32)
          akms |= 1u << rsn[at + 5 + 4 * k];
    }
    printf("GuampWiFi: BSS %02x:%02x:%02x:%02x:%02x:%02x %u MHz %d dBm IE %u bytes, %s\n",
        bss.bssid[0], bss.bssid[1], bss.bssid[2], bss.bssid[3], bss.bssid[4], bss.bssid[5],
        bss.freq_mhz, (int)bss.signal_dbm, (unsigned)bss.ie_len,
        !rsn ? (bss.caps & 0x10 ? "WEP/WPA1 (not supported)" : "open") :
        (akms & (1u << 2)) && (akms & (1u << 8)) ? "WPA2-PSK + SAE transition (joined as WPA2-PSK)" :
        (akms & (1u << 2)) ? "WPA2-PSK" : (akms & (1u << 8)) ? "SAE only (WPA3, not supported yet)" :
        (akms & (1u << 1)) ? "802.1X (not supported)" : "RSN other (not supported)");
  }
}

static void station_heard(void (*each)(const struct redwood_sta_bss *, void *), void *context)
{
  if (!worker_online() || !each) return;
  int count = nw_wifi_bss_count();
  for (int i = 0; i < count; ++i) {
    struct nw_bss_info in;
    struct redwood_sta_bss out;
    if (nw_wifi_bss_get(i, &in) != 0) continue;
    bzero(&out, sizeof(out));
    memcpy(out.bssid, in.bssid, sizeof(out.bssid));
    memcpy(out.ssid, in.ssid, in.ssid_len); out.ssid_length = in.ssid_len;
    out.frequency = (uint16_t)in.freq_mhz; out.rssi = in.signal_dbm;
    out.beacon_interval = in.beacon_int; out.capability = in.caps;
    out.age_ms = in.age_ms;
    out.elements_length = in.ie_len < sizeof(out.elements) ? in.ie_len : sizeof(out.elements);
    memcpy(out.elements, in.ie, out.elements_length);
    each(&out, context);
  }
}

static int station_join(const struct redwood_sta_join *request)
{
  if (!worker_online()) return ENETDOWN;
  if (!request) return EINVAL;
  enum redwood_sta_phase previous = public_phase();
  char pmk_hex[64];
  const char *pass = NULL; size_t pass_len = 0;
  static const char hex[] = "0123456789abcdef";
  const uint8_t *bssid = NULL;
  uint8_t zero[6] = {0};
  if (request->ssid_length > sizeof(request->ssid) || request->passphrase_length > sizeof(request->passphrase)) return EINVAL;
  if (request->security == REDWOOD_STA_WPA2_PSK) {
    if (request->passphrase_length) {
      pass = (const char *)request->passphrase; pass_len = request->passphrase_length;
    } else {
      for (unsigned i = 0; i < sizeof(request->pmk); ++i) {
        pmk_hex[2*i] = hex[request->pmk[i] >> 4]; pmk_hex[2*i+1] = hex[request->pmk[i] & 15];
      }
      pass = pmk_hex; pass_len = sizeof(pmk_hex);
    }
  } else if (request->security != REDWOOD_STA_OPEN) return EOPNOTSUPP;
  if (memcmp(request->bssid, zero, sizeof(zero)) != 0) bssid = request->bssid;
  lck_spin_lock(guamp_wifi_state_lock);
  bzero(&g_public_state.bss, sizeof(g_public_state.bss));
  memcpy(g_public_state.bss.ssid, request->ssid, request->ssid_length);
  g_public_state.bss.ssid_length = request->ssid_length;
  if (bssid) memcpy(g_public_state.bss.bssid, bssid, sizeof(g_public_state.bss.bssid));
  g_public_state.phase = REDWOOD_STA_JOINING;
  g_public_state.last_error = 0;
  lck_spin_unlock(guamp_wifi_state_lock);
  lck_mtx_lock(guamp_wifi_event_lock); g_join_done = 0;
  lck_mtx_unlock(guamp_wifi_event_lock);
  int error = nw_wifi_connect(request->ssid, request->ssid_length, bssid, pass, pass_len);
  bzero(pmk_hex, sizeof(pmk_hex));
  if (error) {
    error = request_to_errno(error);
    request_failed(REDWOOD_STA_JOINING, previous, error);
    return error;
  }
  error = wait_event(&g_join_done, 30000);
  int driver_result = error ? -1 : g_join_result;
  if (!error) error = result_to_errno(g_join_result);
  if (error) request_failed(REDWOOD_STA_JOINING, previous, error);
  /* guamp 29/10: one line per join for leggi-wifi.sh (never the name or the key) */
  printf("GuampWiFi: join %s: %s, %s access point, driver result %d, errno %d\n", error ? "FAILED" : "done",
      request->security == REDWOOD_STA_WPA2_PSK ? (request->passphrase_length ? "WPA2-PSK passphrase" : "WPA2-PSK PMK") : "open",
      bssid ? "a given" : "any", driver_result, error);
  return error;
}

static int station_leave(void)
{
  if (!worker_online()) return ENETDOWN;
  lck_mtx_lock(guamp_wifi_event_lock); g_disconnect_done = 0;
  lck_mtx_unlock(guamp_wifi_event_lock);
  nw_wifi_disconnect();
  int error = wait_event(&g_disconnect_done, 3000);
  if (!error) error = result_to_errno(g_disconnect_result);
  if (error) {
    lck_spin_lock(guamp_wifi_state_lock); g_public_state.last_error = error;
    lck_spin_unlock(guamp_wifi_state_lock);
  }
  return error;
}

static void station_state(struct redwood_sta_state *state)
{
  /* Redwood may ask from any thread: publish a short locked snapshot, never
   * read the ath10k state machine concurrently with its driver worker. */
  public_state_copy(state);
}

static unsigned station_channels(struct redwood_sta_channel *out, unsigned most)
{
  if (!worker_online() || (!out && most)) return 0;
  unsigned count = (unsigned)nw_wifi_channel_count();
  if (count > most) count = most;
  for (unsigned i = 0; i < count; ++i) {
    struct nw_wifi_channel ch;
    if (nw_wifi_channel_get((int)i, &ch) != 0) return i;
    out[i].frequency = (uint16_t)ch.freq_mhz;
    out[i].flags = (ch.flags & NW_CHAN_PASSIVE ? REDWOOD_STA_CHANNEL_PASSIVE : 0) |
        (ch.flags & NW_CHAN_RADAR ? REDWOOD_STA_CHANNEL_RADAR : 0);
  }
  return count;
}

static struct redwood_sta_ops g_station = {
  .power = station_power, .scan = station_scan, .heard = station_heard,
  .join = station_join, .leave = station_leave, .state = station_state,
  .channels = station_channels,
};

static void wifi_worker(void *arg, wait_result_t result)
{
  (void)arg; (void)result;
  thread_set_thread_name(current_thread(), "guamp-wifi");
  while (1) {
    int work = nw_wifi_step();
    if (work < 0) {
      if (g_platform && g_platform->log) g_platform->log(3, "guamp wifi worker stopped on driver error");
      worker_failed(EIO);
      break;
    }
    if (work == 0) g_platform->sleep_ms(1);
  }
}

int guamp_xnu_wifi_attach(const struct lx_platform *platform, const struct gw_snoc_board *board)
{
  struct nw_wifi_status status;
  thread_t worker = NULL;
  int expected = 0;
  if (__atomic_load_n(&g_worker_started, __ATOMIC_ACQUIRE) == GUAMP_WORKER_FAILED) return ENETDOWN;
  if (!platform || !platform->sleep_ms ||
      !__atomic_compare_exchange_n(&g_worker_started, &expected, -1, false,
          __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) return EBUSY;
  int lock_error = station_locks_init();
  if (lock_error) { __atomic_store_n(&g_worker_started, 0, __ATOMIC_RELEASE); return lock_error; }
  g_wifi_platform = *platform;
  g_fallback_fw_get = platform->fw_get;
  g_wifi_platform.fw_get = guamp_fw_get;
  g_platform = &g_wifi_platform;
  lck_spin_lock(guamp_wifi_state_lock);
  bzero(&g_public_state, sizeof(g_public_state));
  g_public_state.phase = REDWOOD_STA_OFF;
  lck_spin_unlock(guamp_wifi_state_lock);
  nw_wifi_set_events(&g_events);
  /* guamp-wifi-trace (nw_abi.h): breadcrumbs only, nothing below changes with it */
  if (nw_wifi_trace_budget) nw_wifi_trace("attach: nw_wifi_start_snoc");
  int error = nw_wifi_start_snoc(g_platform, board);
  if (nw_wifi_trace_budget) nw_wifi_trace("attach: nw_wifi_start_snoc result %d", error);
  if (error) { __atomic_store_n(&g_worker_started, 0, __ATOMIC_RELEASE); return error; }
  bool have_address = false;
  for (unsigned ms = 0; ms < 10000; ++ms) {
    nw_wifi_status_get(&status);
    if (status.state == NW_STA_IDLE && status.channels && !(status.mac[0] & 1) &&
        bcmp(status.mac, "\0\0\0\0\0\0", 6) != 0) { have_address = true; break; }
    if (nw_wifi_trace_budget && ms < nw_wifi_trace_budget)
      nw_wifi_trace("attach: loop %u: state %u channels %u, nw_wifi_step", ms, (unsigned)status.state, (unsigned)status.channels);
    int work = nw_wifi_step();
    if (work < 0) { nw_wifi_stop(); __atomic_store_n(&g_worker_started, 0, __ATOMIC_RELEASE); return EIO; }
    g_platform->sleep_ms(1);
  }
  if (nw_wifi_trace_budget) nw_wifi_trace("attach: probe loop left, address %s", have_address ? "ready" : "NOT ready");
  if (!have_address) { nw_wifi_stop(); __atomic_store_n(&g_worker_started, 0, __ATOMIC_RELEASE); return ETIMEDOUT; }
  bzero(&g_link, sizeof(g_link));
  g_link.name = "guamp WCN3990 ath10k"; g_link.wireless = true;
  memcpy(g_link.address, status.mac, sizeof(g_link.address));
  memcpy(g_station.address, status.mac, sizeof(g_station.address));
  g_link.transmit = link_transmit;
  if (!redwood_net_attach(&g_link)) {
    nw_wifi_stop(); __atomic_store_n(&g_worker_started, 0, __ATOMIC_RELEASE); return EBUSY;
  }
  __atomic_store_n(&g_worker_started, 1, __ATOMIC_RELEASE);
  error = kernel_thread_start(wifi_worker, NULL, &worker);
  if (error != KERN_SUCCESS) {
    nw_wifi_stop(); worker_failed(EAGAIN); return EAGAIN;
  }
  thread_deallocate(worker);
  /* The new worker may already have failed; do not publish a healthy station
   * or overwrite its terminal OFF/error snapshot after that handoff. */
  lck_spin_lock(guamp_wifi_state_lock);
  bool online = worker_online();
  if (online) { g_public_state.phase = REDWOOD_STA_IDLE; g_public_state.last_error = 0; }
  lck_spin_unlock(guamp_wifi_state_lock);
  if (!online) return ENETDOWN;
  /* the same checkpoint line as mode 1 (leggi-wifi.sh); the address is the radio's, as in mode 1 */
  printf("GuampWiFi: ath10k ready, MAC %02x:%02x:%02x:%02x:%02x:%02x, %u channels (mode 2)\n", status.mac[0], status.mac[1],
      status.mac[2], status.mac[3], status.mac[4], status.mac[5], status.channels);
  station_self_scan();
  redwood_sta_attach(&g_station);
  printf("GuampWiFi: mode 2 station handed to iOS (wifid: POWER, SCAN_REQ, ASSOCIATE)\n");
  return worker_online() ? 0 : ENETDOWN;
}
