/* Regression cases for accepted-request ownership and link lifetime. */
static void control_regression_cases(void) {
  struct nw_wifi_status st;u8 packet[60]={0};memcpy(packet,AP,6);memcpy(packet+6,g_sta_mac,6);packet[12]=8;
  CHECK(nw_wifi_tx(packet,sizeof packet)==-ENETDOWN,"TX is rejected before an authorized association");
  const int scans=g_scan_done;
  CHECK(nw_wifi_scan(NULL,0)==0,"first queued scan accepted");
  CHECK(nw_wifi_scan(NULL,0)==-EBUSY&&nw_wifi_power(1)==-EBUSY,"occupied request slot is reported busy instead of overwritten");
  spin(2);
  CHECK(nw_wifi_connect((const u8 *)SSID,strlen(SSID),NULL,NULL,0)==0,"join can wait for an active scan");
  CHECK(nw_wifi_connect((const u8 *)SSID,strlen(SSID),NULL,NULL,0)==-EBUSY,"accepted join retains its callback ownership");
  g_conn_done=-1;nw_wifi_disconnect();spin(3);nw_wifi_status_get(&st);
  CHECK(g_conn_done==NW_RES_FAILED&&g_scan_done==scans+1&&st.state==NW_STA_IDLE,
      "disconnect completes canceled queued join and scan without losing callbacks");
  g_open=1;beacon();g_conn_done=-1;
  CHECK(nw_wifi_connect((const u8 *)SSID,strlen(SSID),NULL,NULL,0)==0,"open join accepted after cancellation");
  spin(20);nw_wifi_status_get(&st);CHECK(st.state==NW_STA_UP,"open association succeeds after cancellation");
  const int states=n_state;
  CHECK(nw_wifi_power(1)==0,"repeated power-on accepted");spin(3);nw_wifi_status_get(&st);
  CHECK(st.state==NW_STA_UP&&n_state==states,"repeated power-on preserves live station/channel/key state");
  const int sent=n_tx;
  CHECK(nw_wifi_tx(packet,sizeof packet)==0&&nw_sta_tx_pending(),"authorized frame enters TX queue");
  nw_wifi_disconnect();spin(2);nw_wifi_status_get(&st);
  CHECK(st.state==NW_STA_IDLE&&!nw_sta_tx_pending()&&n_tx==sent,"disconnect purges queued data before another association");
  CHECK(nw_wifi_tx(packet,sizeof packet)==-ENETDOWN,"disconnected station refuses new Ethernet data");
  beacon();CHECK(nw_wifi_connect((const u8 *)SSID,strlen(SSID),NULL,NULL,0)==0,"rejoin accepted after TX purge");spin(20);
  nw_sta_link_lost();
  for(int i=0;i<6;i++) {
    const u8 body[12]={0};struct sk_buff *foreign=mgmt_to_sta(IEEE80211_STYPE_BEACON,body,sizeof body);
    struct ieee80211_mgmt *m=(void *)foreign->data;m->sa[5]^=1;m->bssid[5]^=1;
    inject(foreign,2437);spin(1000);
  }
  nw_wifi_status_get(&st);
  CHECK(st.state==NW_STA_IDLE,"open-network timer detects lost link despite beacons from a different BSSID");
  beacon();CHECK(nw_wifi_connect((const u8 *)SSID,strlen(SSID),NULL,NULL,0)==0,"reconnect after real link-loss works");spin(20);
  nw_sta_link_lost();beacon();beacon();spin(5100);nw_wifi_status_get(&st);
  CHECK(st.state==NW_STA_UP,"valid current-AP traffic cancels beacon-miss grace period");
  nw_wifi_disconnect();spin(2);
}
