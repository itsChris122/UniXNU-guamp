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
static int g_scan_done, g_scan_result;
static int g_join_done, g_join_result;
static int g_power_done, g_power_result, g_power_value;
static int g_disconnect_done, g_disconnect_result;

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
  lck_spin_unlock(guamp_wifi_state_lock);
}

static int
wait_event(int *done, int timeout_ms)
{
  uint64_t deadline;
  clock_interval_to_deadline((uint32_t)timeout_ms, NSEC_PER_MSEC, &deadline);
  lck_mtx_lock(guamp_wifi_event_lock);
  while (!*done) {
    if (lck_mtx_sleep_deadline(guamp_wifi_event_lock, LCK_SLEEP_DEFAULT,
        (event_t)done, THREAD_UNINT, deadline) == THREAD_TIMED_OUT) {
      lck_mtx_unlock(guamp_wifi_event_lock);
      return ETIMEDOUT;
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

static void on_scan(int result)
{
  public_state_set(REDWOOD_STA_IDLE, result_to_errno(result));
  lck_mtx_lock(guamp_wifi_event_lock);
  g_scan_result = result; g_scan_done = 1;
  lck_mtx_unlock(guamp_wifi_event_lock);
  thread_wakeup((event_t)&g_scan_done);
}

static void on_join(int result)
{
  if (result == NW_RES_OK) {
    struct nw_wifi_status status;
    nw_wifi_status_get(&status); /* this event is dispatched on the driver worker */
    lck_spin_lock(guamp_wifi_state_lock);
    g_public_state.phase = REDWOOD_STA_JOINED;
    g_public_state.last_error = 0;
    memcpy(g_public_state.bss.bssid, status.bssid, sizeof(g_public_state.bss.bssid));
    memcpy(g_public_state.bss.ssid, status.ssid, status.ssid_len);
    g_public_state.bss.ssid_length = status.ssid_len;
    g_public_state.bss.frequency = (uint16_t)status.freq_mhz;
    g_public_state.bss.rssi = status.signal_dbm;
    lck_spin_unlock(guamp_wifi_state_lock);
  } else {
    public_state_set(REDWOOD_STA_IDLE, result_to_errno(result));
  }
  lck_mtx_lock(guamp_wifi_event_lock);
  g_join_result = result; g_join_done = 1;
  lck_mtx_unlock(guamp_wifi_event_lock);
  redwood_net_link_state(result == NW_RES_OK);
  thread_wakeup((event_t)&g_join_done);
}

static void on_lost(int reason)
{
  public_state_set(REDWOOD_STA_IDLE, reason ? ECONNRESET : 0);
  redwood_net_link_state(false);
  redwood_sta_lost(ECONNRESET);
}

static void on_rx(const uint8_t *frame, size_t length)
{
  /* nw_wifi_step dispatches RX in worker context, never from the CE interrupt. */
  redwood_net_input(frame, length);
}

static void on_power(int on, int result)
{
  public_state_set(on ? REDWOOD_STA_IDLE : REDWOOD_STA_OFF,
      result_to_errno(result));
  lck_mtx_lock(guamp_wifi_event_lock);
  g_power_value = on; g_power_result = result; g_power_done = 1;
  lck_mtx_unlock(guamp_wifi_event_lock);
  if (!on) redwood_net_link_state(false);
  thread_wakeup((event_t)&g_power_done);
}

static void on_disconnect(int result)
{
  public_state_set(REDWOOD_STA_IDLE, result_to_errno(result));
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
  uint8_t frame[REDWOOD_NET_FRAME_MAX];
  if (length < ETHER_HDR_LEN || length > sizeof(frame) ||
      mbuf_copydata(packet, 0, length, frame) != 0) return false;
  return nw_wifi_tx(frame, length) == 0;
}

static int station_power(bool on)
{
  lck_mtx_lock(guamp_wifi_event_lock); g_power_done = 0;
  lck_mtx_unlock(guamp_wifi_event_lock);
  if (nw_wifi_power(on) != 0) return EIO;
  int error = wait_event(&g_power_done, 2000);
  if (error) return error;
  if (g_power_value != !!on) return EIO;
  return result_to_errno(g_power_result);
}

static int station_scan(const struct redwood_sta_scan *request)
{
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
  public_state_set(REDWOOD_STA_SCANNING, 0);
  int error = nw_wifi_scan_ex(&scan);
  if (error) {
    error = error == -19 ? ENETDOWN : EINVAL;
    public_state_set(REDWOOD_STA_IDLE, error);
    return error;
  }
  error = wait_event(&g_scan_done, 35000);
  if (!error) error = result_to_errno(g_scan_result);
  public_state_set(REDWOOD_STA_IDLE, error);
  return error;
}

static void station_heard(void (*each)(const struct redwood_sta_bss *, void *), void *context)
{
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
  bzero(g_public_state.bss.bssid, sizeof(g_public_state.bss.bssid));
  bzero(g_public_state.bss.ssid, sizeof(g_public_state.bss.ssid));
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
    error = error == -19 ? ENETDOWN : EINVAL;
    public_state_set(REDWOOD_STA_IDLE, error);
    return error;
  }
  error = wait_event(&g_join_done, 30000);
  if (!error) error = result_to_errno(g_join_result);
  public_state_set(error ? REDWOOD_STA_IDLE : REDWOOD_STA_JOINED, error);
  return error;
}

static int station_leave(void)
{
  lck_mtx_lock(guamp_wifi_event_lock); g_disconnect_done = 0;
  lck_mtx_unlock(guamp_wifi_event_lock);
  nw_wifi_disconnect();
  int error = wait_event(&g_disconnect_done, 3000) ? ETIMEDOUT :
      result_to_errno(g_disconnect_result);
  public_state_set(REDWOOD_STA_IDLE, error);
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
      break;
    }
    if (work == 0) g_platform->sleep_ms(1);
  }
  __atomic_store_n(&g_worker_started, 0, __ATOMIC_RELEASE);
}

int guamp_xnu_wifi_attach(const struct lx_platform *platform, const struct gw_snoc_board *board)
{
  struct nw_wifi_status status;
  thread_t worker = NULL;
  int expected = 0;
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
  int error = nw_wifi_start_snoc(g_platform, board);
  if (error) { __atomic_store_n(&g_worker_started, 0, __ATOMIC_RELEASE); return error; }
  bool have_address = false;
  for (unsigned ms = 0; ms < 10000; ++ms) {
    nw_wifi_status_get(&status);
    if (!(status.mac[0] & 1) && bcmp(status.mac, "\0\0\0\0\0\0", 6) != 0) { have_address = true; break; }
    int work = nw_wifi_step();
    if (work < 0) { nw_wifi_stop(); __atomic_store_n(&g_worker_started, 0, __ATOMIC_RELEASE); return EIO; }
    g_platform->sleep_ms(1);
  }
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
    redwood_sta_attach(NULL); nw_wifi_stop();
    __atomic_store_n(&g_worker_started, 0, __ATOMIC_RELEASE); return EAGAIN;
  }
  thread_deallocate(worker);
  public_state_set(REDWOOD_STA_IDLE, 0);
  redwood_sta_attach(&g_station);
  return 0;
}
