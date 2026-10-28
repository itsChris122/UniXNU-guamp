#ifndef NW_STA_H
#define NW_STA_H
#include "lx_os.h"
struct ieee80211_hw; struct sk_buff;
void nw_sta_ready(struct ieee80211_hw *hw);
void nw_sta_rx(struct sk_buff *skb);
void nw_sta_scan_done(bool aborted);
void nw_sta_link_lost(void);
void nw_sta_eapol_status(u64 replay, u32 cookie, bool acked);
void nw_sta_run(void);
int nw_sta_tx_pending(void);
#endif
