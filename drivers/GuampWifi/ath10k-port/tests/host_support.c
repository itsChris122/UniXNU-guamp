/* Host-only stand-ins for XNU/board dependencies. Never linked into driver. */
#include "gw_ath10k_port.h"
#include "lx_plat.h"
#include <net/mac80211.h>
int gw_random_nonce(u8 *p,size_t n) {
  /* Deterministic fixtures for reproducible protocol tests, not runtime RNG. */
  static u32 stream=0x12345678;
  for(size_t i=0;i<n;i++) { stream=1664525u*stream+1013904223u; p[i]=(u8)(stream>>24); }
  return 0;
}
int gw_queue_stopped(int q) { (void)q; return 0; }
void gw_stop_queues(struct ieee80211_hw *hw) { (void)hw; }
void gw_wake_queues(void) {}
void wiphy_apply_custom_regulatory(struct wiphy *w,const struct ieee80211_regdomain *r) { w->regd=r; }
void nw_wifi_htt_stats(int type) { (void)type; }
void nw_wifi_tx_reasons(char *p,int n) { if(n) p[0]=0; }
void nw_wifi_rx_state(char *p,int n) { if(n) p[0]=0; }
void nw_wifi_ring_state(char *p,int n) { if(n) p[0]=0; }
size_t strlcat(char *dst,const char *src,size_t n) {
  size_t used=0,len=strlen(src); while(used<n&&dst[used]) used++;
  if(used<n) { size_t copy=len<n-used-1?len:n-used-1; memcpy(dst+used,src,copy); dst[used+copy]=0; }
  return used+len;
}
