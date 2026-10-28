/* Include after spin() and before main() in wpa2_station_host.c.
 * Call rx_regression_unauthorized() while M4 is pending, before its TX ACK;
 * call rx_regression_secure_up() immediately after the first successful
 * secure connection, before rekeys. rx_regression_open_up() follows the open
 * connection. Each preserves the existing test's receive-output counters.
 */
static struct sk_buff *r2_rx_raw(u16 extra_fc,u64 flags,int bad_da,int bad_ta,unsigned fragment) {
  struct sk_buff *skb=alloc_skb(100,0);
  struct ieee80211_hdr *h=skb_put_zero(skb,26);
  h->frame_control=cpu_to_le16(IEEE80211_FTYPE_DATA|IEEE80211_STYPE_QOS_DATA|extra_fc);
  memcpy(h->addr1,g_sta_mac,6);if(bad_da)h->addr1[5]^=1;
  memcpy(h->addr2,AP,6);if(bad_ta)h->addr2[5]^=1;
  memcpy(h->addr3,"\x02\x77\x66\x55\x44\x33",6);h->seq_ctrl=cpu_to_le16(fragment);
  const u8 snap[]={0xaa,0xaa,3,0,0,0,0x08,0x06};skb_put_data(skb,snap,sizeof snap);skb_put_data(skb,"R2",2);
  struct ieee80211_rx_status *rs=IEEE80211_SKB_RXCB(skb);memset(rs,0,sizeof *rs);rs->freq=2437;rs->flag=flags;
  return skb;
}
static struct sk_buff *r2_rx_eth(int eapol,const u8 *payload,unsigned length,u64 flags) {
  struct sk_buff *skb=alloc_skb(14+length,0);u8 *eth=skb_put_zero(skb,14);
  memcpy(eth,g_sta_mac,6);memcpy(eth+6,AP,6);eth[12]=eapol?0x88:0x08;eth[13]=eapol?0x8e:0x06;
  skb_put_data(skb,payload,length);struct ieee80211_rx_status *rs=IEEE80211_SKB_RXCB(skb);
  memset(rs,0,sizeof *rs);rs->freq=2437;rs->flag=flags|RX_FLAG_8023;return skb;
}
static void r2_rx_deliver(struct sk_buff *skb) { nw_sta_rx(skb);kfree_skb(skb); }

static void rx_regression_unauthorized(void) {
  const int before=g_rx_n;
  r2_rx_deliver(r2_rx_raw(IEEE80211_FCTL_FROMDS,RX_FLAG_DECRYPTED,0,0,0));
  r2_rx_deliver(r2_rx_eth(0,(const u8 *)"R2",2,RX_FLAG_DECRYPTED));
  CHECK(g_rx_n==before,"802.11 and Ethernet payloads remain blocked while WPA2 waits for M4 ACK");
}

static void rx_regression_secure_up(void) {
  const int before=g_rx_n;const size_t old_len=g_rx_len;u8 old_eth[sizeof g_rx_eth];memcpy(old_eth,g_rx_eth,sizeof old_eth);
  r2_rx_deliver(r2_rx_raw(IEEE80211_FCTL_FROMDS,0,0,0,0));
  r2_rx_deliver(r2_rx_raw(IEEE80211_FCTL_FROMDS|IEEE80211_FCTL_PROTECTED,RX_FLAG_IV_STRIPPED,0,0,0));
  r2_rx_deliver(r2_rx_eth(0,(const u8 *)"R2",2,0));
  CHECK(g_rx_n==before,"secure data requires hardware decryption, even when the Protected bit was stripped");
  const u64 errors[]={RX_FLAG_FAILED_FCS_CRC,RX_FLAG_FAILED_PLCP_CRC,RX_FLAG_MMIC_ERROR,RX_FLAG_ONLY_MONITOR};
  for(unsigned i=0;i<sizeof errors/sizeof errors[0];i++)
    r2_rx_deliver(r2_rx_raw(IEEE80211_FCTL_FROMDS,RX_FLAG_DECRYPTED|errors[i],0,0,0));
  CHECK(g_rx_n==before,"RX errors and monitor-only frames do not enter the station data path");
  r2_rx_deliver(r2_rx_raw(IEEE80211_FCTL_TODS,RX_FLAG_DECRYPTED,0,0,0));
  r2_rx_deliver(r2_rx_raw(IEEE80211_FCTL_FROMDS|IEEE80211_FCTL_TODS,RX_FLAG_DECRYPTED,0,0,0));
  r2_rx_deliver(r2_rx_raw(IEEE80211_FCTL_FROMDS,RX_FLAG_DECRYPTED,1,0,0));
  r2_rx_deliver(r2_rx_raw(IEEE80211_FCTL_FROMDS,RX_FLAG_DECRYPTED,0,1,0));
  r2_rx_deliver(r2_rx_raw(IEEE80211_FCTL_FROMDS|IEEE80211_FCTL_MOREFRAGS,RX_FLAG_DECRYPTED,0,0,0));
  r2_rx_deliver(r2_rx_raw(IEEE80211_FCTL_FROMDS,RX_FLAG_DECRYPTED,0,0,1));
  CHECK(g_rx_n==before,"wrong direction, other receiver/transmitter and unreassembled fragments are rejected");
  {struct sk_buff *skb=r2_rx_raw(IEEE80211_FCTL_FROMDS,RX_FLAG_DECRYPTED,0,0,0);skb->data[26+5]=1;r2_rx_deliver(skb);}
  CHECK(g_rx_n==before,"unsupported SNAP OUI is not converted into Ethernet");
  for(unsigned length=24;length<=25;length++) {
    struct sk_buff *skb=alloc_skb(length,0);struct ieee80211_mgmt *m=skb_put_zero(skb,length);
    m->frame_control=cpu_to_le16(IEEE80211_STYPE_DEAUTH);memcpy(m->da,g_sta_mac,6);memcpy(m->sa,AP,6);memcpy(m->bssid,AP,6);
    struct ieee80211_rx_status *rs=IEEE80211_SKB_RXCB(skb);memset(rs,0,sizeof *rs);r2_rx_deliver(skb);
  }
  {struct nw_wifi_status status;nw_wifi_status_get(&status);CHECK(status.state==NW_STA_UP,"24/25-byte deauth cannot read a missing reason or tear down the link");}
  const int keys=n_setkey,previous_m4=g_msg4_tx_count;
  r2_rx_deliver(r2_rx_eth(1,g_last_msg3,g_last_msg3_len,0));
  CHECK(g_msg4_tx_count==previous_m4+1&&n_setkey==keys,"Ethernet-decapped EAPOL reaches the handshake and duplicate M3 does not reinstall keys");
  complete_tx(&g_pending_m4_skb,false);
  CHECK(g_rx_n==before,"EAPOL is consumed internally and never delivered to the OS network interface");
  r2_rx_deliver(r2_rx_raw(IEEE80211_FCTL_FROMDS,RX_FLAG_DECRYPTED,0,0,0));
  r2_rx_deliver(r2_rx_eth(0,(const u8 *)"R2",2,RX_FLAG_DECRYPTED));
  CHECK(g_rx_n==before+2,"authenticated secure data is delivered from raw and Ethernet-decapped paths");
  g_rx_n=before;g_rx_len=old_len;memcpy(g_rx_eth,old_eth,sizeof old_eth);
}

static void rx_regression_open_up(void) {
  const int before=g_rx_n;const size_t old_len=g_rx_len;u8 old_eth[sizeof g_rx_eth];memcpy(old_eth,g_rx_eth,sizeof old_eth);
  r2_rx_deliver(r2_rx_raw(IEEE80211_FCTL_FROMDS,0,0,0,0));
  r2_rx_deliver(r2_rx_eth(0,(const u8 *)"R2",2,0));
  CHECK(g_rx_n==before+2,"open networks continue to accept unencrypted data in both receive layouts");
  g_rx_n=before;g_rx_len=old_len;memcpy(g_rx_eth,old_eth,sizeof old_eth);
}
