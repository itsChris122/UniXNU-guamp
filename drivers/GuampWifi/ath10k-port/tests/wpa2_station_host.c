/* The station code against a fake driver (records the operations it is given) and a fake access point that answers like one:
 * scan results, open authentication, association, the WPA2 four-way handshake with a wrapped group key, then data in both directions. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <net/mac80211.h>
#include "lx_os.h"
#include "lx_plat.h"
#include "nw_abi.h"
#include "nw_crypto.h"
#include "nw_mac80211.h"
#include "nw_sta.h"

struct nw_wifi_stats nw_wifi_stats; unsigned int ath12k_debug_mask;
static u64 g_ms; static int fails;
static int g_fail_next_alloc;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL: " __VA_ARGS__); printf("\n"); fails++; } else if (getenv("V")) { printf("ok:   " __VA_ARGS__); printf("\n"); } } while (0)
static u64 p_now(void) { return g_ms; }
static void p_sleep(unsigned ms) { g_ms += ms; }
static void p_poll(void) {}
static void p_log(int l, const char *s) { if (getenv("V") || l <= 3) printf("[%d %llu] %s\n", l, (unsigned long long)g_ms, s); }
static void p_halt(void) { abort(); }
static void *p_alloc(size_t n) { if(g_fail_next_alloc){g_fail_next_alloc--;return NULL;}return malloc(n); }
static void p_free(void *p) { free(p); }
static void *p_dma_alloc(size_t n, uint64_t *b) { void *p = calloc(1, n + 64); *b = (uint64_t)p; return p; }
static void p_dma_free(size_t n, void *p, uint64_t b) { (void)n; (void)b; free(p); }
static uint64_t p_dma_map(void *p, size_t n, int d) { (void)n; (void)d; return (uint64_t)p; }
static void p_dma_sync(uint64_t b, size_t n, int d, int action) { (void)b; (void)n; (void)d; (void)action; }
static int p_fw(const char *n, const u8 **d, size_t *s) { (void)n; (void)d; (void)s; return -1; }
static const struct lx_platform plat = { p_now, p_sleep, p_poll, p_log, p_halt, p_alloc, p_free, p_dma_alloc, p_dma_free, p_dma_map, p_dma_sync, p_fw };

/* ---- the access point ---- */
static const u8 AP[6] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x55 };
static const char *SSID = "TestNet", *PASS = "correct horse";
static u8 g_pmk[32], g_ptk[48], g_anonce[32], g_snonce[32], g_sta_mac[6]; static u64 g_replay = 100;
static int g_stage, g_open, g_right, g_group_tkip;   /* g_group_tkip: WPA/WPA2 mixed network, group cipher TKIP */
static u8 g_ap_gtk32[32];
static u8 g_last_msg3[200]; static unsigned g_last_msg3_len;
static struct ieee80211_supported_band band;
static struct ieee80211_channel chans[3]; static struct ieee80211_rate rates[12];

static void inject(struct sk_buff *skb, int freq) { struct ieee80211_rx_status *st = IEEE80211_SKB_RXCB(skb); memset(st, 0, sizeof *st); st->freq = freq; st->signal = -47; nw_sta_rx(skb); kfree_skb(skb); }
static void beacon(void) {
  struct sk_buff *skb = alloc_skb(300, 0); struct ieee80211_mgmt *m = skb_put_zero(skb, 36);
  m->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_BEACON); memset(m->da, 0xff, 6); memcpy(m->sa, AP, 6); memcpy(m->bssid, AP, 6);
  m->u.beacon.beacon_int = cpu_to_le16(100); m->u.beacon.capab_info = cpu_to_le16(0x0411);
  u8 *e = skb_put(skb, 2 + strlen(SSID)); e[0] = 0; e[1] = strlen(SSID); memcpy(e + 2, SSID, strlen(SSID));
  e = skb_put(skb, 10); e[0] = 1; e[1] = 8; static const u8 r[8] = { 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24 }; memcpy(e + 2, r, 8);
  e = skb_put(skb, 6); e[0] = 5; e[1] = 4; e[2] = 0; e[3] = 2; e[4] = 0; e[5] = 0;     /* TIM with DTIM period 2 */
  if (!g_open) { u8 rsn[] = { 48, 20, 1, 0, 0, 0x0f, 0xac, 4, 1, 0, 0, 0x0f, 0xac, 4, 1, 0, 0, 0x0f, 0xac, 2, 0, 0 }; if (g_group_tkip) rsn[7] = 2; skb_put_data(skb, rsn, sizeof rsn); }
  inject(skb, 2437); }
static struct sk_buff *mgmt_to_sta(u16 stype, const void *body, unsigned n) {
  struct sk_buff *skb = alloc_skb(200 + n, 0); struct ieee80211_mgmt *m = skb_put_zero(skb, 24);
  m->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT | stype); memcpy(m->da, g_sta_mac, 6); memcpy(m->sa, AP, 6); memcpy(m->bssid, AP, 6); skb_put_data(skb, body, n); return skb; }
static void ap_eapol(const u8 *k, unsigned n) {
  struct sk_buff *skb = alloc_skb(400, 0); struct ieee80211_hdr *h = skb_put_zero(skb, 26);
  h->frame_control = cpu_to_le16(IEEE80211_FTYPE_DATA | IEEE80211_STYPE_QOS_DATA | IEEE80211_FCTL_FROMDS); memcpy(h->addr1, g_sta_mac, 6); memcpy(h->addr2, AP, 6); memcpy(h->addr3, AP, 6);
  static const u8 snap[8] = { 0xaa, 0xaa, 3, 0, 0, 0, 0x88, 0x8e }; skb_put_data(skb, snap, 8); skb_put_data(skb, k, n); inject(skb, 2437); }
static void be16(u8 *p, unsigned v) { p[0] = v >> 8; p[1] = v; }
static void aes_wrap(const u8 kek[16], const u8 *plain, unsigned n, u8 *out) {   /* RFC 3394 5.1 */
  uint32_t rk[44]; nw_aes128_expand(kek, rk); u8 a[8], b[16], r[256]; unsigned blocks = n / 8; memset(a, 0xa6, 8); memcpy(r, plain, n);
  for (int j = 0; j < 6; j++) for (unsigned i = 1; i <= blocks; i++) { memcpy(b, a, 8); memcpy(b + 8, r + (i - 1) * 8, 8); nw_aes128_encrypt(rk, b, b);
    uint64_t t = (uint64_t)blocks * j + i; memcpy(a, b, 8); for (int k = 0; k < 8; k++) a[k] ^= (u8)(t >> (56 - 8 * k)); memcpy(r + (i - 1) * 8, b + 8, 8); }
  memcpy(out, a, 8); memcpy(out + 8, r, n); }
static void send_msg1(void) {
  u8 f[200]; memset(f, 0, sizeof f); f[0] = 2; f[1] = 3; f[4] = 2; be16(f + 5, 0x008a); be16(f + 7, 16); f[16] = g_replay >> 8 & 0; f[16 - 1 + 0] = 0;
  for (int i = 0; i < 8; i++) f[9 + i] = g_replay >> (56 - 8 * i); memcpy(f + 17, g_anonce, 32); unsigned total = 99; be16(f + 2, total - 4); ap_eapol(f, total); }
static void derive(void) {
  u8 d[76]; const u8 *lo = memcmp(AP, g_sta_mac, 6) < 0 ? AP : g_sta_mac, *hi = lo == AP ? g_sta_mac : AP; memcpy(d, lo, 6); memcpy(d + 6, hi, 6);
  const u8 *nl = memcmp(g_anonce, g_snonce, 32) < 0 ? g_anonce : g_snonce, *nh = nl == g_anonce ? g_snonce : g_anonce; memcpy(d + 12, nl, 32); memcpy(d + 44, nh, 32);
  nw_prf(g_pmk, 32, "Pairwise key expansion", d, 76, g_ptk, 384); }
static int check_mic(const u8 *k, unsigned n) { u8 c[400], h[20]; memcpy(c, k, n); memset(c + 81, 0, 16); nw_hmac_sha1(g_ptk, 16, c, n, h); return !memcmp(h, k + 81, 16); }
static const u8 GTK[16] = { 0xa0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
static u8 g_ap_gtk[16]; static int g_ap_gtk_id=1, g_msg3_gtk=1;
static u8 g_last_group_update[200]; static unsigned g_last_group_update_len;
static void send_msg3_tkip(void) {   /* GTK KDE with a 32-byte TKIP group key: 2 + 38 bytes, wrapped to 48 */
  u8 kd[40], wrapped[48], f[220]; memset(kd, 0, sizeof kd); kd[0] = 0xdd; kd[1] = 38; kd[2] = 0x00; kd[3] = 0x0f; kd[4] = 0xac; kd[5] = 1; kd[6] = g_ap_gtk_id; memcpy(kd + 8, g_ap_gtk32, 32);
  aes_wrap(g_ptk + 16, kd, 40, wrapped);
  memset(f, 0, sizeof f); f[0] = 2; f[1] = 3; f[4] = 2; be16(f + 5, 0x13ca); be16(f + 7, 16); g_replay++; for (int i = 0; i < 8; i++) f[9 + i] = g_replay >> (56 - 8 * i);
  memcpy(f + 17, g_anonce, 32); be16(f + 97, 48); memcpy(f + 99, wrapped, 48); unsigned total = 99 + 48; be16(f + 2, total - 4);
  u8 h[20]; nw_hmac_sha1(g_ptk, 16, f, total, h); memcpy(f + 81, h, 16);
  memcpy(g_last_msg3, f, total); g_last_msg3_len = total; ap_eapol(f, total); }
static void send_msg3(void) {
  if (g_group_tkip) { send_msg3_tkip(); return; }
  u8 kd[40], wrapped[48], f[200]; memset(kd, 0, sizeof kd); if(g_msg3_gtk) { kd[0] = 0xdd; kd[1] = 22; kd[2] = 0x00; kd[3] = 0x0f; kd[4] = 0xac; kd[5] = 1; kd[6] = g_ap_gtk_id; kd[7] = 0; memcpy(kd + 8, g_ap_gtk, 16); kd[24] = 0xdd; } /* GTK KDE and padding */
  aes_wrap(g_ptk + 16, kd, 32, wrapped);
  memset(f, 0, sizeof f); f[0] = 2; f[1] = 3; f[4] = 2; be16(f + 5, 0x13ca); be16(f + 7, 16); g_replay++; for (int i = 0; i < 8; i++) f[9 + i] = g_replay >> (56 - 8 * i);
  memcpy(f + 17, g_anonce, 32); be16(f + 97, 40); memcpy(f + 99, wrapped, 40); unsigned total = 99 + 40; be16(f + 2, total - 4);
  u8 h[20]; nw_hmac_sha1(g_ptk, 16, f, total, h); memcpy(f + 81, h, 16);
  memcpy(g_last_msg3, f, total); g_last_msg3_len = total; ap_eapol(f, total); }
static int g_assoc_rsn_group, g_m2_rsn_group, g_m4_keys_before = -1, g_m4_protected;
static void resend_msg3(void) { if (g_last_msg3_len) ap_eapol(g_last_msg3, g_last_msg3_len); }
static void send_group_update(int corrupt_mic) {
  u8 kd[40]={0}, wrapped[48], f[200]={0}, h[20];
  kd[0]=0xdd; kd[1]=22; kd[2]=0; kd[3]=0x0f; kd[4]=0xac; kd[5]=1; kd[6]=g_ap_gtk_id; memcpy(kd+8,g_ap_gtk,16); kd[24]=0xdd;
  aes_wrap(g_ptk+16,kd,32,wrapped); f[0]=2; f[1]=3; f[4]=2; be16(f+5,0x1382); be16(f+7,16);
  g_replay++; for(int i=0;i<8;i++) f[9+i]=g_replay>>(56-8*i); be16(f+97,40); memcpy(f+99,wrapped,40);
  g_last_group_update_len=139; be16(f+2,g_last_group_update_len-4); nw_hmac_sha1(g_ptk,16,f,g_last_group_update_len,h); memcpy(f+81,h,16);
  memcpy(g_last_group_update,f,g_last_group_update_len); if(corrupt_mic) f[81]^=0x80; ap_eapol(f,g_last_group_update_len);
}
static void resend_group_update(void) { if(g_last_group_update_len) ap_eapol(g_last_group_update,g_last_group_update_len); }
static void send_rekey_msg1(void) { g_replay++; for(int i=0;i<32;i++) g_anonce[i]=(u8)(0x70+i); send_msg1(); }

/* ---- the fake driver ---- */
static int n_tx, n_setkey, n_state, n_disable_group, n_disable_pairwise, g_expect_gtk_id=1, g_fail_pairwise, g_fail_group; static int g_states[8]; static struct ieee80211_key_conf *g_pk, *g_gk; static struct sk_buff *g_last_data, *g_pending_m2_skb, *g_pending_m4_skb;
static u8 g_auth_seen, g_assoc_seen, g_msg2_seen, g_msg4_seen; static int g_scan_req, g_scan_ssids, g_scan_duration, g_scan_result, g_msg4_tx_count, g_group_m4_count; static u32 g_scan_first_freq;
static int g_ready_start_error,g_ready_add_error,n_ready_starts,n_ready_stops,n_ready_adds,n_ready_removes;
static int op_start(struct ieee80211_hw *hw) { (void)hw;n_ready_starts++;return g_ready_start_error; }
static void op_stop(struct ieee80211_hw *hw,bool suspend) { (void)hw;(void)suspend;n_ready_stops++; }
static int op_add_if(struct ieee80211_hw *hw, struct ieee80211_vif *v) { (void)hw;(void)v;n_ready_adds++;return g_ready_add_error; }
static void op_remove_if(struct ieee80211_hw *hw,struct ieee80211_vif *v) { (void)hw;(void)v;n_ready_removes++; }
static int op_hw_scan(struct ieee80211_hw *hw, struct ieee80211_vif *v, struct ieee80211_scan_request *r) { (void)hw; (void)v; g_scan_req = r->req.n_channels; g_scan_ssids=r->req.n_ssids; g_scan_duration=r->req.duration; g_scan_first_freq=g_scan_req?r->req.channels[0]->center_freq:0; return 0; }
static void op_cancel_scan(struct ieee80211_hw *hw, struct ieee80211_vif *v) { (void)hw; (void)v; nw_sta_scan_done(true); }
static int op_add_ctx(struct ieee80211_hw *hw, struct ieee80211_chanctx_conf *c) { (void)hw; CHECK(c->def.chan->center_freq == 2437, "chanctx on the AP's channel"); return 0; }
static void op_rm_ctx(struct ieee80211_hw *hw, struct ieee80211_chanctx_conf *c) { (void)hw; (void)c; }
static int op_assign(struct ieee80211_hw *hw, struct ieee80211_vif *v, struct ieee80211_bss_conf *b, struct ieee80211_chanctx_conf *c) { (void)hw; (void)v; (void)b; (void)c; return 0; }
static void op_unassign(struct ieee80211_hw *hw, struct ieee80211_vif *v, struct ieee80211_bss_conf *b, struct ieee80211_chanctx_conf *c) { (void)hw; (void)v; (void)b; (void)c; }
static int op_state(struct ieee80211_hw *hw, struct ieee80211_vif *v, struct ieee80211_sta *s, enum ieee80211_sta_state o, enum ieee80211_sta_state n) { (void)hw; (void)v; (void)s; (void)o; if (n_state < 8) g_states[n_state++] = n; return 0; }
static void op_link(struct ieee80211_hw *hw, struct ieee80211_vif *v, struct ieee80211_bss_conf *b, u64 ch) { (void)hw; (void)v; (void)b; (void)ch; }
static void op_cfg(struct ieee80211_hw *hw, struct ieee80211_vif *v, u64 ch) { (void)hw; (void)v; (void)ch; }
static int op_setkey(struct ieee80211_hw *hw, enum set_key_cmd cmd, struct ieee80211_vif *v, struct ieee80211_sta *s, struct ieee80211_key_conf *k) {
  (void)hw; (void)v; if (cmd != SET_KEY) { if(k->flags&IEEE80211_KEY_FLAG_PAIRWISE)n_disable_pairwise++;else n_disable_group++; return 0; }
  n_setkey++; if (k->flags & IEEE80211_KEY_FLAG_PAIRWISE) { CHECK(s != 0, "pairwise key has a station"); CHECK(!memcmp(k->key, g_ptk + 32, 16), "TK equals the AP's"); if(g_fail_pairwise){g_fail_pairwise--;return -EIO;} g_pk = k; }
  else if (g_group_tkip) { CHECK(k->cipher == WLAN_CIPHER_SUITE_TKIP && k->keylen == 32 && k->keyidx == g_expect_gtk_id, "TKIP group key: cipher, length and index");
    CHECK(!memcmp(k->key, g_ap_gtk32, 16) && !memcmp(k->key + 16, g_ap_gtk32 + 24, 8) && !memcmp(k->key + 24, g_ap_gtk32 + 16, 8), "TKIP group key: Michael keys swapped for the supplicant");
    g_gk = k; }
  else { CHECK(k->cipher == WLAN_CIPHER_SUITE_CCMP && !memcmp(k->key, g_ap_gtk, 16) && k->keyidx == g_expect_gtk_id, "GTK and its index arrive intact"); if(g_fail_group){g_fail_group--;return -EIO;} g_gk = k; } return 0; }
static void op_tx(struct ieee80211_hw *hw, struct ieee80211_tx_control *c, struct sk_buff *skb) {
  (void)hw; (void)c; n_tx++;
  /* ath10k advertises HW_80211_ENCAP: ordinary data reaches its TX op as
   * Ethernet, while management and EAPOL handshake frames remain 802.11. */
  struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
  if (txi->flags & IEEE80211_TX_CTL_HW_80211_ENCAP) {
    CHECK(skb->len >= 14, "hardware encapsulation receives a full Ethernet header");
    CHECK(txi->control.hw_key == g_pk, "Ethernet data selects the installed pairwise key");
    if (g_last_data) kfree_skb(g_last_data); g_last_data = skb; return;
  }
  struct ieee80211_hdr *h = (void *)skb->data; u16 fc = le16_to_cpu(h->frame_control);
  if (ieee80211_is_auth(h->frame_control)) { g_auth_seen = 1; u8 b[6] = { 0, 0, 2, 0, 0, 0 }; inject(mgmt_to_sta(IEEE80211_STYPE_AUTH, b, 6), 2437); }
  else if (ieee80211_is_assoc_req(h->frame_control)) { g_assoc_seen = 1;
    const u8 *ie = skb->data + 24 + 4; unsigned left = skb->len - 28; int ssid_ok = 0, rsn_ok = 0;
    while (left >= 2 && left >= 2u + ie[1]) { if (ie[0] == 0 && ie[1] == strlen(SSID) && !memcmp(ie + 2, SSID, ie[1])) ssid_ok = 1; if (ie[0] == 48 && ie[1] == 20) { rsn_ok = 1; g_assoc_rsn_group = ie[7]; } left -= 2 + ie[1]; ie += 2 + ie[1]; }
    CHECK(ssid_ok, "association request names the network"); CHECK(rsn_ok == !g_open, "association request carries the RSN element when needed");
    u8 b[6] = { 0x11, 0x04, 0, 0, 0x01, 0xc0 }; inject(mgmt_to_sta(IEEE80211_STYPE_ASSOC_RESP, b, 6), 2437);
    if (!g_open) { g_stage = 1; send_msg1(); } }
  else if (ieee80211_is_data(h->frame_control) && skb->len > 26 + 8 && skb->data[26 + 6] == 0x88 && skb->data[26 + 7] == 0x8e) {
    const u8 *k = skb->data + 26 + 8; unsigned n = skb->len - 26 - 8; unsigned info = k[5] << 8 | k[6];
    if (info == 0x010a) { g_msg2_seen = 1; if (n >= 99 + 22) g_m2_rsn_group = k[99 + 7]; memcpy(g_snonce, k + 17, 32); derive(); int ok = check_mic(k, n); if (g_right) CHECK(ok, "message 2 MIC verifies under the PTK"); else CHECK(!ok, "message 2 MIC differs under a wrong passphrase");
      if (g_pending_m2_skb) kfree_skb(g_pending_m2_skb); g_pending_m2_skb=skb; send_msg3(); return; }
    else if (info == 0x030a) { if (!g_msg4_seen) g_m4_keys_before = n_setkey; g_msg4_seen = 1; g_msg4_tx_count++;
      if (ieee80211_has_protected(h->frame_control) || txi->control.hw_key) g_m4_protected = 1; CHECK(check_mic(k, n), "message 4 MIC verifies");
      if (g_pending_m4_skb) kfree_skb(g_pending_m4_skb); g_pending_m4_skb=skb; return; }
    else if (info == 0x0302) { g_group_m4_count++; CHECK(check_mic(k,n),"group-key message 2 MIC verifies"); } }
  else if (ieee80211_is_data(h->frame_control)) { if (g_last_data) kfree_skb(g_last_data); g_last_data = skb; (void)fc; return; }
  kfree_skb(skb); }
static const struct ieee80211_ops ops = { .start=op_start,.stop=op_stop,.add_interface=op_add_if,.remove_interface=op_remove_if,.hw_scan = op_hw_scan, .cancel_hw_scan = op_cancel_scan, .add_chanctx = op_add_ctx, .remove_chanctx = op_rm_ctx,
  .assign_vif_chanctx = op_assign, .unassign_vif_chanctx = op_unassign, .sta_state = op_state, .bss_info_changed = op_link, .set_key = op_setkey, .tx = op_tx };

static int g_scan_done, g_conn_done = -1, g_rx_n; static u8 g_rx_eth[200]; static size_t g_rx_len;
static int g_power_done, g_power_on, g_power_result;
static void ev_scan(int result) { g_scan_result=result; g_scan_done++; }
static void ev_conn(int r) { g_conn_done = r; }
static void ev_lost(int r) { (void)r; }
static void ev_rx(const u8 *e, size_t n) { g_rx_n++; g_rx_len = n; memcpy(g_rx_eth, e, n); }
static void ev_power(int on, int result) { g_power_on=on; g_power_result=result; g_power_done++; }
static void ev_disconnect(int result) { CHECK(result==NW_RES_OK,"disconnect completion result"); }
static void ev_tx_status(u64 replay, u32 cookie, bool acked) { nw_sta_eapol_status(replay, cookie, acked); }
static const struct nw_wifi_hooks hooks = { .eapol_status = ev_tx_status };
static void complete_tx(struct sk_buff **pending, bool acked) {
  struct sk_buff *skb=*pending; if(!skb) return; *pending=0;
  if(getenv("V")) { struct ieee80211_hdr *h=(void *)skb->data; unsigned hl=ieee80211_hdrlen(h->frame_control); const u8 *e=skb->data+hl+8;
    printf("TX status len=%u hl=%u fc=%04x eapol=%u/%u body=%u desc=%u keyinfo=%02x%02x keylen=%02x%02x\n",skb->len,hl,le16_to_cpu(h->frame_control),e[0],e[1],e[2]<<8|e[3],e[4],e[5],e[6],e[7],e[8]); }
  struct ieee80211_tx_info *info=IEEE80211_SKB_CB(skb); if(acked) info->flags|=IEEE80211_TX_STAT_ACK;
  struct ieee80211_tx_status status; memset(&status,0,sizeof status); status.skb=skb; ieee80211_tx_status_ext(0,&status);
}
static void spin(int ms) { for (int i = 0; i < ms; i++) { nw_sta_run(); lx_pump(); g_ms++; } }
#include "wpa2_rx_cases.inc"
#include "wpa2_control_cases.inc"
#include "wpa2_ready_cases.inc"

int main(void) {
  lx_plat = &plat; lx_root_init();
  for (int i = 0; i < 12; i++) { static const int br[12] = { 10, 20, 55, 110, 60, 90, 120, 180, 240, 360, 480, 540 }; rates[i].bitrate = br[i]; }
  chans[0].center_freq = 2412; chans[1].center_freq = 2437; chans[2].center_freq = 2462; for (int i = 0; i < 3; i++) chans[i].band = NL80211_BAND_2GHZ;
  band.channels = chans; band.n_channels = 3; band.bitrates = rates; band.n_bitrates = 12; band.band = NL80211_BAND_2GHZ;
  struct ieee80211_hw *hw = ieee80211_alloc_hw_nm(64, &ops, "t"); hw->wiphy->bands[NL80211_BAND_2GHZ] = &band; hw->sta_data_size = 64; hw->vif_data_size = 64; hw->chanctx_data_size = 16;
  memcpy(hw->wiphy->perm_addr, "\x02\xaa\xbb\xcc\xdd\xee", 6); memcpy(g_sta_mac, hw->wiphy->perm_addr, 6);
  memcpy(g_ap_gtk,GTK,sizeof GTK);
  struct nw_wifi_events ev = { .scan_done=ev_scan, .connect_done=ev_conn, .link_lost=ev_lost,
    .rx=ev_rx, .power_done=ev_power, .disconnect_done=ev_disconnect };
  nw_wifi_set_events(&ev); nw_wifi_set_hooks(&hooks);
  nw_pbkdf2_sha1(PASS, strlen(PASS), (const u8 *)SSID, strlen(SSID), 4096, g_pmk, 32); for (int i = 0; i < 32; i++) g_anonce[i] = 0x40 + i;
  char pskhex[65]; static const char hexdigits[]="0123456789abcdef"; for(int i=0;i<32;i++){pskhex[2*i]=hexdigits[g_pmk[i]>>4];pskhex[2*i+1]=hexdigits[g_pmk[i]&15];} pskhex[64]=0;
  nw_sta_ready(hw);
  CHECK(nw_wifi_channel_count()==3,"only enabled 2.4 GHz channels are exposed to the OS");
  { struct nw_wifi_channel ch; CHECK(!nw_wifi_channel_get(1,&ch) && ch.freq_mhz==2437,"channel enumeration preserves center frequency"); }
  nw_wifi_scan(NULL,0); spin(5); CHECK(g_scan_req==3,"initial scan starts before deferred join");
  nw_wifi_connect((const u8 *)SSID,strlen(SSID),0,"wrong password",14); spin(5);
  CHECK(g_scan_done==1 && g_scan_result==NW_RES_FAILED && g_conn_done==NW_RES_NOT_FOUND,
      "join queued during scan survives cancellation, completes, and reports missing BSS");
  nw_wifi_scan(NULL,0);
  spin(5); CHECK(g_scan_req == 3 && g_scan_ssids==1, "active wildcard scan covers all three channels (%d)", g_scan_req);
  beacon(); nw_sta_scan_done(false); CHECK(g_scan_done == 2 && g_scan_result==NW_RES_OK && nw_wifi_bss_count() == 1, "scan finished with one network");
  { struct nw_wifi_scan_request request={0}; request.ssid_len=sizeof(SSID)-1; memcpy(request.ssid,SSID,request.ssid_len);
    request.passive=1; request.dwell_ms=100; request.frequency_count=1; request.frequencies_mhz[0]=2412;
    CHECK(!nw_wifi_scan_ex(&request),"extended passive scan request accepted"); spin(5);
    CHECK(g_scan_req==1 && g_scan_first_freq==2412 && g_scan_ssids==0 && g_scan_duration==98,
        "passive scan applies requested channel, suppresses probes and converts dwell to TU");
    nw_sta_scan_done(false); }
  struct nw_bss_info bi; CHECK(!nw_wifi_bss_get(0, &bi) && bi.ssid_len == strlen(SSID) && bi.freq_mhz == 2437 && bi.signal_dbm == -47, "network details");
  { char badhex[64]; memset(badhex,'z',sizeof badhex); CHECK(nw_wifi_connect((const u8 *)SSID,strlen(SSID),0,badhex,sizeof badhex)<0,"64-character PSK must be hexadecimal"); }
  /* wrong passphrase: handshake must fail with a key error, not hang */
  nw_wifi_connect((const u8 *)SSID, strlen(SSID), 0, "wrong password", 14); spin(9000);
  CHECK(g_conn_done == NW_RES_BAD_KEY, "wrong passphrase gives a key error (%d)", g_conn_done);
  /* the right one */
  g_right = 1; g_conn_done = -1; g_stage = 0; n_state = 0; n_setkey = 0; g_msg2_seen = g_msg4_seen = 0; g_msg4_tx_count = 0;
  nw_wifi_connect((const u8 *)SSID, strlen(SSID), 0, pskhex, 64); spin(500);
  /* guamp 30/10: as wpa_supplicant, the keys go in right after message 4 is handed to the driver (fin9g: the WCN3990 gave no
   * transmit status for it and the access point gave up); message 4 itself leaves before them and unprotected. */
  CHECK(g_msg4_seen && g_m4_keys_before == 0 && !g_m4_protected, "message 4 is handed to the driver unprotected, before any key");
  CHECK(g_conn_done == NW_RES_OK && n_setkey == 2, "keys installed and link authorized right after message 4, without a TX status");
  (void)rx_regression_unauthorized;
  complete_tx(&g_pending_m2_skb,true);
  CHECK(g_conn_done == NW_RES_OK && n_setkey == 2, "a late ACK for message 2 changes nothing");
  complete_tx(&g_pending_m4_skb,false);
  CHECK(g_conn_done == NW_RES_OK && n_setkey == 2, "a negative message 4 TX status does not reinstall or drop the keys");
  resend_msg3(); spin(5);
  CHECK(g_msg4_tx_count >= 2 && n_setkey == 2 && !g_m4_protected, "replayed message 3 is answered again, unprotected, without reinstalling keys");
  complete_tx(&g_pending_m4_skb,true); spin(300);
  CHECK(g_auth_seen && g_assoc_seen && g_msg2_seen && g_msg4_seen, "auth, assoc, message 2 and message 4 all went out");
  CHECK(g_assoc_rsn_group == 4 && g_m2_rsn_group == 4, "CCMP network: association and message 2 name group CCMP");
  CHECK(g_conn_done == NW_RES_OK, "connected (%d)", g_conn_done); CHECK(n_setkey == 2, "two keys installed (%d)", n_setkey);
  CHECK(n_state >= 4 && g_states[0] == IEEE80211_STA_NONE && g_states[n_state - 1] == IEEE80211_STA_AUTHORIZED, "station state ends authorized");
  struct nw_wifi_status st; nw_wifi_status_get(&st); CHECK(st.state == NW_STA_UP && st.freq_mhz == 2437, "status says up");
  rx_regression_secure_up();
  /* Group rekey: corrupt MIC must not install; same valid replay is accepted once. */
  { static const u8 gtk2[16]={0xb0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; memcpy(g_ap_gtk,gtk2,16); g_ap_gtk_id=2; g_expect_gtk_id=2;
    send_group_update(1); CHECK(n_setkey==2 && g_group_m4_count==0,"invalid group-key MIC is ignored");
    g_fail_group=1; resend_group_update(); CHECK(n_setkey==3 && g_group_m4_count==0,"failed GTK installation is not acknowledged");
    resend_group_update(); spin(3); CHECK(n_setkey==4 && g_group_m4_count==1,"retransmitted GTK update succeeds after install failure");
    resend_group_update(); CHECK(n_setkey==4 && g_group_m4_count==2,"duplicate GTK update is acknowledged without reinstalling"); }
  /* AP-initiated pairwise rekey keeps the old link up until M4 is ACKed. */
  send_rekey_msg1(); spin(3);
  CHECK(g_conn_done==NW_RES_OK && n_setkey==4 && g_pending_m4_skb,"pairwise rekey leaves the active link and keys intact before M4 completion");
  complete_tx(&g_pending_m2_skb,true); CHECK(n_setkey==4,"M2 ACK cannot install the rekeyed keys");
  complete_tx(&g_pending_m4_skb,true); spin(150);
  CHECK(n_setkey==6 && g_conn_done==NW_RES_OK,"ACKed pairwise rekey replaces PTK and updates GTK");
  nw_wifi_status_get(&st); CHECK(st.state==NW_STA_UP,"station remains connected after pairwise rekey");
  { struct ieee80211_key_conf *old_pk=g_pk; g_fail_pairwise=1; send_rekey_msg1(); spin(3); complete_tx(&g_pending_m2_skb,true); complete_tx(&g_pending_m4_skb,true); spin(150);
    nw_wifi_status_get(&st); CHECK(st.state==NW_STA_UP && g_pk==old_pk && g_conn_done==NW_RES_OK,"failed pairwise key install restores the previous live link"); }
  { struct ieee80211_key_conf *old_pk=g_pk; send_rekey_msg1(); spin(3); spin(8100); nw_wifi_status_get(&st);
    CHECK(st.state==NW_STA_UP && g_pk==old_pk && g_conn_done==NW_RES_OK,"pairwise rekey timeout abandons the candidate and preserves the link");
    complete_tx(&g_pending_m2_skb,true); complete_tx(&g_pending_m4_skb,false); }
  /* data out */
  u8 eth[60]; memset(eth, 0, sizeof eth); memcpy(eth, AP, 6); memcpy(eth + 6, g_sta_mac, 6); eth[12] = 0x08; eth[13] = 0x00; memcpy(eth + 14, "payload-bytes", 13);
  nw_wifi_tx(eth, 27); spin(5);
  CHECK(g_last_data != 0, "a data frame reached the driver");
  if (g_last_data) { struct ieee80211_tx_info *info = IEEE80211_SKB_CB(g_last_data);
    CHECK(info->flags & IEEE80211_TX_CTL_HW_80211_ENCAP, "data requests hardware 802.11 encapsulation");
    CHECK(info->control.hw_key == g_pk && info->control.vif != 0, "Ethernet frame names the pairwise key and interface");
    CHECK(!memcmp(g_last_data->data, AP, 6) && !memcmp(g_last_data->data + 6, g_sta_mac, 6), "Ethernet addresses are AP and station");
    CHECK(!memcmp(g_last_data->data + 12, "\x08\x00", 2) && !memcmp(g_last_data->data + 14, "payload-bytes", 13), "Ethernet payload reaches the hardware unchanged"); }
  /* data in */
  { struct sk_buff *skb = alloc_skb(200, 0); struct ieee80211_hdr *h = skb_put_zero(skb, 26); h->frame_control = cpu_to_le16(IEEE80211_FTYPE_DATA | IEEE80211_STYPE_QOS_DATA | IEEE80211_FCTL_FROMDS | IEEE80211_FCTL_PROTECTED);
    memcpy(h->addr1, g_sta_mac, 6); memcpy(h->addr2, AP, 6); memcpy(h->addr3, "\x02\x77\x66\x55\x44\x33", 6); u8 snap[8] = { 0xaa, 0xaa, 3, 0, 0, 0, 0x08, 0x06 }; skb_put_data(skb, snap, 8); skb_put_data(skb, "ARPDATA", 7);
    struct ieee80211_rx_status *rs = IEEE80211_SKB_RXCB(skb); memset(rs, 0, sizeof *rs); rs->freq = 2437; rs->flag = RX_FLAG_DECRYPTED | RX_FLAG_IV_STRIPPED; nw_sta_rx(skb); kfree_skb(skb); }
  CHECK(g_rx_n == 1 && g_rx_len == 14 + 7 && g_rx_eth[12] == 0x08 && g_rx_eth[13] == 0x06 && !memcmp(g_rx_eth, g_sta_mac, 6) && !memcmp(g_rx_eth + 6, "\x02\x77\x66\x55\x44\x33", 6), "frame from the AP becomes an Ethernet frame");
  /* open network */
  nw_wifi_disconnect(); spin(5); CHECK(n_disable_group==2 && n_disable_pairwise>=1,"disconnect releases every active GTK slot and PTK"); g_open = 1; beacon(); g_conn_done = -1; n_state = 0;
  nw_wifi_connect((const u8 *)SSID, strlen(SSID), 0, 0, 0); spin(3000);
  CHECK(g_conn_done == NW_RES_OK, "open network connects (%d)", g_conn_done);
  rx_regression_open_up();
  nw_wifi_power(0); spin(3); nw_wifi_status_get(&st);
  CHECK(st.state==NW_STA_OFF && g_power_done==1 && g_power_on==0 && g_power_result==NW_RES_OK,"software radio-off disconnects and acknowledges");
  CHECK(nw_wifi_scan(0,0)<0 && nw_wifi_tx(eth,27)<0,"station scan and TX are rejected while radio is disabled");
  nw_wifi_power(1); spin(3); nw_wifi_status_get(&st);
  CHECK(st.state==NW_STA_IDLE && g_power_done==2 && g_power_on==1 && g_power_result==NW_RES_OK,"software radio-on re-enables station requests");
  control_regression_cases();
  m4_status_missing_case();
  tkip_group_case();
  ready_regression_cases(hw);
  printf("%s\n", fails ? "FAILED" : "all passed"); return fails ? 1 : 0; }
