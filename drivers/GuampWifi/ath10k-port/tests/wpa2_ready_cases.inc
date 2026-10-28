/* Station registration ownership regression. Called last, after the ordinary
 * station/control cases; it shuts down and finally frees the synthetic hw.
 */
extern void gw_sta_shutdown(void);
static void ready_regression_cases(struct ieee80211_hw *hw) {
  struct nw_wifi_status status;
  int starts=n_ready_starts,stops=n_ready_stops,adds=n_ready_adds,removes=n_ready_removes;
  gw_sta_shutdown();
  CHECK(n_ready_stops==stops+1&&n_ready_removes==removes+1,"initial teardown removes the added vif and stops started hardware once");
  gw_sta_shutdown();
  CHECK(n_ready_stops==stops+1&&n_ready_removes==removes+1,"repeated shutdown never removes or stops twice");
  starts=n_ready_starts;stops=n_ready_stops;adds=n_ready_adds;removes=n_ready_removes;
  g_ready_start_error=-EIO;nw_sta_ready(hw);g_ready_start_error=0;
  CHECK(nw_wifi_stats.stage==3&&nw_wifi_stats.last_err==-EIO,"failed hardware start publishes its real initialization error");
  CHECK(n_ready_starts==starts+1&&n_ready_stops==stops&&n_ready_adds==adds&&n_ready_removes==removes,
    "unsuccessful start does not acquire stop or vif-removal ownership");
  nw_wifi_status_get(&status);CHECK(status.state==NW_STA_OFF&&!memcmp(status.mac,"\0\0\0\0\0\0",6)&&nw_wifi_power(1)==-ENODEV,
    "failed start clears stale interface identity and rejects OS requests");
  gw_sta_shutdown();CHECK(n_ready_stops==stops&&n_ready_removes==removes,"shutdown after a failed start is inert");
  starts=n_ready_starts;stops=n_ready_stops;adds=n_ready_adds;removes=n_ready_removes;
  g_fail_next_alloc=1;nw_sta_ready(hw);
  CHECK(nw_wifi_stats.stage==3&&nw_wifi_stats.last_err==-ENOMEM,"vif allocation failure is visible to probe status");
  CHECK(n_ready_starts==starts+1&&n_ready_stops==stops+1&&n_ready_adds==adds&&n_ready_removes==removes,
    "vif allocation failure stops successfully started hardware without removing a nonexistent vif");
  gw_sta_shutdown();CHECK(n_ready_stops==stops+1&&n_ready_removes==removes,"allocation rollback leaves no stop ownership behind");
  starts=n_ready_starts;stops=n_ready_stops;adds=n_ready_adds;removes=n_ready_removes;
  g_ready_add_error=-EIO;nw_sta_ready(hw);g_ready_add_error=0;
  CHECK(nw_wifi_stats.stage==3&&nw_wifi_stats.last_err==-EIO,"add_interface failure publishes its actual error");
  CHECK(n_ready_starts==starts+1&&n_ready_stops==stops+1&&n_ready_adds==adds+1&&n_ready_removes==removes,
    "failed add_interface frees its allocated vif and never calls remove_interface on an unadded vif");
  gw_sta_shutdown();CHECK(n_ready_stops==stops+1&&n_ready_removes==removes,"add-interface rollback cannot stop or remove twice");
  starts=n_ready_starts;stops=n_ready_stops;adds=n_ready_adds;removes=n_ready_removes;
  nw_sta_ready(hw);nw_wifi_status_get(&status);
  CHECK(nw_wifi_stats.stage==4&&nw_wifi_stats.last_err==0&&status.state==NW_STA_IDLE&&!memcmp(status.mac,g_sta_mac,6),
    "the same synthetic hardware can initialize successfully after every partial failure");
  CHECK(n_ready_starts==starts+1&&n_ready_adds==adds+1&&n_ready_stops==stops&&n_ready_removes==removes,
    "successful setup holds exactly one hardware-start and one vif-add ownership");
  gw_sta_shutdown();gw_sta_shutdown();
  CHECK(n_ready_stops==stops+1&&n_ready_removes==removes+1,"successful reinitialization teardown is balanced and idempotent");
  ieee80211_free_hw(hw);
}
