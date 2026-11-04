/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#include "gw_snoc_abi.h"
#include <linux/errno.h>
#include "lx_plat.h"
#include "nw_mac80211.h"
#include "nw_sta.h"
/* guamp-wifi-trace one-shot breadcrumbs (nw_abi.h): nothing runs while the budget is 0 */
#define GW_TRACE(...) do { if(nw_wifi_trace_budget) nw_wifi_trace(__VA_ARGS__); } while(0)
static struct gw_snoc_board gw_board;
static struct platform_device gw_device;
static struct ath10k *gw_ar;
struct nw_wifi_stats nw_wifi_stats;
static bool gw_polling;
static bool gw_servicing_ce;
/* guamp 04/11 (fin45 panic, kpan-20261104-021110: wifi_worker -> gw_snoc_step -> gw_snoc_poll_ce -> ath10k_snoc_htt_tx_cb ->
 * ath10k_ce_completed_send_next read CE4's index from the freed RRI page): the WMI timeout's hardware restart runs as work
 * inside a copy engine handler of the same poll (nested lx_pump), powers the block down and frees the RRI memory, and the
 * poll then went on to the next engine. Every engine service now checks gw_ce_live first, and the RRI page is freed only
 * when no copy engine service is on the stack (gw_ce_users), else when the outermost one ends. */
static unsigned gw_ce_users;
static bool gw_rri_free_pending;
static void gw_rri_release_if_idle(struct ath10k *ar) {
  if(gw_rri_free_pending&&!gw_ce_users) { gw_rri_free_pending=false; ath10k_ce_free_rri(ar); }
}
static int gw_boot_error;
/* gw_regs_live / gw_ce_live (declared in gw_snoc.c, before the register accessors): a register access outside a confirmed
 * wlan_enable .. wlan_disable hangs the SoC bus; CE servicing runs only from a successful hif power_up to hif_stop. */

static int gw_firmware_preflight(const struct lx_platform *platform) {
  static const char *const firmware[] = {
    "firmware-6.bin","firmware-5.bin","firmware-4.bin",
    "firmware-3.bin","firmware-2.bin"
  };
  const uint8_t *data=NULL;size_t bytes=0;int found=0;
  for(unsigned i=0;i<sizeof(firmware)/sizeof(firmware[0]);i++) {
    data=NULL;bytes=0;
    if(!platform->fw_get(firmware[i],&data,&bytes)&&data&&bytes) { found=1;break; }
  }
  if(!found) {
    if(platform->log)platform->log(3,"ath10k: missing WCN3990 firmware-2.bin through firmware-6.bin");
    return -ENOENT;
  }
  data=NULL;bytes=0;
  if(!platform->fw_get("board-2.bin",&data,&bytes)&&data&&bytes)return 0;
  data=NULL;bytes=0;
  if(!platform->fw_get("board-snoc-guamp-wcn-snoc.bin",&data,&bytes)&&data&&bytes)return 0;
  data=NULL;bytes=0;
  if(!platform->fw_get("board.bin",&data,&bytes)&&data&&bytes)return 0;
  if(platform->log)platform->log(3,"ath10k: missing WCN3990 board data (API2 board-2.bin or API1 board file)");
  return -ENOENT;
}

/* This port services CE rings from gw_snoc_step()/NAPI polling. No XNU CE IRQ
 * handlers are registered yet, so keep the upstream IRQ path disabled. */
static int gw_poll_hif_start(struct ath10k *ar) {
  struct ath10k_snoc *s = ath10k_snoc_priv(ar);
  if(!gw_regs_live) return -ESHUTDOWN;
  gw_ce_live=true;
  bitmap_clear(s->pending_ce_irqs, 0, CE_COUNT_MAX);
  ath10k_core_napi_enable(ar);
  ath10k_snoc_rx_post(ar);
  clear_bit(ATH10K_SNOC_FLAG_RECOVERY, &s->flags);
  { char line[160]; int n=0; unsigned total=0;
    for(unsigned i=0;i<12;i++) { struct ath10k_ce_ring *r=s->ce.ce_states[i].dest_ring; if(!r) continue;
      unsigned posted=(r->write_index-r->sw_index)&r->nentries_mask; total+=posted;
      if(n<(int)sizeof(line)-12) n+=snprintf(line+n,sizeof(line)-n," ce%u=%u",i,posted); }
    lx_log(6,"ath10k: phase HIF start (CE polling, no CE IRQ): %u RX buffers posted:%s",total,line); }
  return 0;
}
/* As Linux's hif_stop disables the CE interrupts first: no poll, NAPI pass, RX refill or TX after this point, before the
 * RX buffers still on the rings are freed (the error path of core_start runs this with the firmware up). */
static void gw_poll_hif_stop(struct ath10k *ar) {
  gw_ce_live=false;
  timer_delete_sync(&ath10k_snoc_priv(ar)->rx_post_retry);
  ath10k_core_napi_sync_disable(ar);
  ath10k_snoc_buffer_cleanup(ar);
}

int ath10k_qmi_wlan_enable(struct ath10k *ar,struct ath10k_qmi_wlan_enable_cfg *cfg,
                           enum wlfw_driver_mode_enum_v01 mode,const char *version) {
  (void)ar; (void)version;
  if(mode!=QMI_WLFW_MISSION_V01) return -EOPNOTSUPP;
  if(!gw_board.wlan_enable||!gw_board.ready||!gw_board.ready(gw_board.ctx)) return -ENODEV;
  _Static_assert(sizeof(struct gw_ce_pipe)==sizeof(struct ath10k_tgt_pipe_cfg),"pipe ABI");
  _Static_assert(sizeof(struct gw_ce_service)==sizeof(struct ath10k_svc_pipe_cfg),"service ABI");
  _Static_assert(sizeof(struct gw_ce_shadow)==sizeof(struct ath10k_shadow_reg_cfg),"shadow ABI");
  lx_log(6,"ath10k: phase WLFW wlan_enable: %u CE target pipes, %u service maps, %u shadow regs",
    cfg->num_ce_tgt_cfg,cfg->num_ce_svc_pipe_cfg,cfg->num_shadow_reg_cfg);
  int e=gw_board.wlan_enable(gw_board.ctx,(const struct gw_ce_pipe *)cfg->ce_tgt_cfg,cfg->num_ce_tgt_cfg,
    (const struct gw_ce_service *)cfg->ce_svc_cfg,cfg->num_ce_svc_pipe_cfg,
    (const struct gw_ce_shadow *)cfg->shadow_reg_cfg,cfg->num_shadow_reg_cfg);
  lx_log(e?3:6,"ath10k: phase WLFW wlan_enable (cfg 0x23 + mode 0x22 mission) result %d",e);
  return e;
}
static int gw_hif_power_up(struct ath10k *ar,enum ath10k_firmware_mode mode) {
  if(!gw_board.ready(gw_board.ctx)) return -ENODEV;
  int e=gw_board.power(gw_board.ctx,1); if(e) return e;
  e=ath10k_snoc_wlan_enable(ar,mode); if(e) return e;
  gw_regs_live=true;   // WLFW confirmed mission mode: the block answers from here to wlan_disable
  if(gw_rri_free_pending) { gw_rri_free_pending=false; ath10k_ce_free_rri(ar); }   /* (guamp 04/11: a release still waiting) */
  ath10k_ce_alloc_rri(ar);
  e=ath10k_snoc_init_pipes(ar);
  if(e) { gw_regs_live=false; lx_log(3,"ath10k: CE init failed %d",e); ath10k_ce_free_rri(ar); return e; }
  gw_ce_live=true;
  { struct ath10k_ce *ce=ath10k_ce_priv(ar);
    lx_log(6,"ath10k: phase CE init: 12 pipes at PA 0x%llx+0x240000, RRI bus 0x%llx",
      (unsigned long long)ath10k_snoc_priv(ar)->mem_pa,(unsigned long long)ce->paddr_rri); }
  return 0;
}
static void gw_hif_power_down(struct ath10k *ar) {
  // Nothing may reach the block once WLAN_MODE off is asked for: stop CE servicing and close the register window first.
  gw_ce_live=false; gw_regs_live=false;
  timer_delete_sync(&ath10k_snoc_priv(ar)->rx_post_retry);
  int e=gw_board.wlan_disable(gw_board.ctx);
  if(e) { gw_boot_error=e; lx_log(3,"ath10k: WLAN stop unconfirmed; DMA retained"); return; }
  gw_rri_free_pending=true; gw_rri_release_if_idle(ar);   /* (guamp 04/11: not under a running copy engine service) */
  e=gw_board.power(gw_board.ctx,0); if(e) gw_boot_error=e;
}
static int gw_map_service_to_pipe(struct ath10k *ar,u16 service,u8 *ul,u8 *dl) {
  int e=ath10k_snoc_hif_map_service_to_pipe(ar,service,ul,dl);
  /* ath10k_htc_start asks for the pktlog service only to find out whether the bus has it: the snoc table gives it a DL pipe
   * and no UL pipe, so -ENOENT is the answer Linux gets as well (ath10k_htc_pktlog_svc_supported, logged at debug level),
   * and pktlog is skipped. Not an error. */
  if(e==-ENOENT&&service==ATH10K_HTC_SVC_ID_HTT_LOG_MSG) {
    lx_log(6,"ath10k: phase HTC service 0x%04x (pktlog) has no UL pipe on snoc: pktlog not used, as in Linux",service);
    return e;
  }
  lx_log(e?3:6,"ath10k: phase HTC service connect 0x%04x -> UL pipe %u DL pipe %u (%d)",service,e?0:*ul,e?0:*dl,e);
  return e;
}
/*
 * guamp 04/11 (fin43 panic, kpan-20261104-000423): after "wmi command 20487 timeout, restarting hardware" the hif was powered
 * down (gw_hif_power_down: CE servicing off, RRI memory freed), but the station went on transmitting (handshake message 2):
 * ath10k_htc_send -> hif send_complete_check -> ath10k_ce_completed_send_next read CE4's read index from the freed RRI page
 * (far = RRI + 4 * 4): data abort. Transmit and completion checks touch the copy engines only while CE servicing is live,
 * like everything else (gw_snoc_poll_ce, gw_napi_poll); otherwise the frame is refused (-ESHUTDOWN, its caller frees it).
 */
static int gw_hif_tx_sg(struct ath10k *ar,u8 pipe,struct ath10k_hif_sg_item *items,int n) {
  if(!gw_ce_live) { static unsigned said; if(said++<4) lx_log(4,"ath10k: transmit on pipe %u refused: copy engines down",pipe); return -ESHUTDOWN; }
  return ath10k_snoc_hif_tx_sg(ar,pipe,items,n);
}
static void gw_hif_send_complete_check(struct ath10k *ar,u8 pipe,int force) {
  if(gw_ce_live) ath10k_snoc_hif_send_complete_check(ar,pipe,force);
}
static u16 gw_hif_get_free_queue_number(struct ath10k *ar,u8 pipe) {
  return gw_ce_live?ath10k_snoc_hif_get_free_queue_number(ar,pipe):0;
}
static const struct ath10k_hif_ops gw_hif={
  .read32=ath10k_snoc_read32,.write32=ath10k_snoc_write32,
  .start=gw_poll_hif_start,.stop=gw_poll_hif_stop,
  .map_service_to_pipe=gw_map_service_to_pipe,.get_default_pipe=ath10k_snoc_hif_get_default_pipe,
  .power_up=gw_hif_power_up,.power_down=gw_hif_power_down,
  .tx_sg=gw_hif_tx_sg,.send_complete_check=gw_hif_send_complete_check,
  .get_free_queue_number=gw_hif_get_free_queue_number,.get_target_info=ath10k_snoc_hif_get_target_info,
};
static const struct ath10k_bus_ops gw_bus={.read32=ath10k_snoc_read32,.write32=ath10k_snoc_write32};
static int gw_napi_poll(struct napi_struct *n,int budget) {
  struct ath10k *ar=container_of(n,struct ath10k,napi);
  gw_ce_users++;
  for(unsigned i=0;i<12&&gw_ce_live;i++) ath10k_ce_per_engine_service(ar,i);   /* (guamp 04/11: checked before each engine) */
  gw_ce_users--; gw_rri_release_if_idle(ar);
  int done=ath10k_htt_txrx_compl_task(ar,budget);
  if(done<budget) napi_complete_done(n,done); return done;
}
static void gw_on_rx(struct ieee80211_hw *hw,struct sk_buff *s) { (void)hw; nw_sta_rx(s); }
static void gw_on_scan(struct ieee80211_hw *hw,bool aborted) {
  (void)hw; lx_log(6,"ath10k: phase WMI scan completion event%s, %u beacons/probe responses received",aborted?" (aborted)":"",nw_wifi_stats.beacons);
  nw_sta_scan_done(aborted); }
static void gw_on_lost(void) { nw_sta_link_lost(); }
static void gw_on_status(u64 replay, u32 cookie, bool ack) { nw_sta_eapol_status(replay,cookie,ack); }
static void gw_on_registered(struct ieee80211_hw *hw) {
  hw->wiphy->interface_modes=BIT(NL80211_IFTYPE_STATION);
  hw->wiphy->flags&=~WIPHY_FLAG_SUPPORTS_TDLS;
  if(gw_ar) {
    struct ath_regulatory *rd=&gw_ar->ath_common.regulatory; unsigned c2=0,c5=0;
    if(hw->wiphy->bands[NL80211_BAND_2GHZ]) for(int i=0;i<hw->wiphy->bands[NL80211_BAND_2GHZ]->n_channels;i++)
      c2+=!(hw->wiphy->bands[NL80211_BAND_2GHZ]->channels[i].flags&IEEE80211_CHAN_DISABLED);
    if(hw->wiphy->bands[NL80211_BAND_5GHZ]) for(int i=0;i<hw->wiphy->bands[NL80211_BAND_5GHZ]->n_channels;i++)
      c5+=!(hw->wiphy->bands[NL80211_BAND_5GHZ]->channels[i].flags&IEEE80211_CHAN_DISABLED);
    lx_log(6,"ath10k: phase WMI ready + wmi_init done, mac registered: fw %s, EEPROM regdomain 0x%x alpha2 %c%c, channels 2.4 GHz %u 5 GHz %u",
      gw_ar->hw->wiphy->fw_version,rd->current_rd,rd->alpha2[0]?rd->alpha2[0]:'?',rd->alpha2[1]?rd->alpha2[1]:'?',c2,c5);
  }
  nw_sta_ready(hw); // this calls .start and creates a station only after core registration
  if(nw_wifi_stats.stage!=4||nw_wifi_stats.last_err) {
    gw_boot_error=nw_wifi_stats.last_err?nw_wifi_stats.last_err:-EIO;
    lx_log(3,"ath10k: station initialization failed: %d",gw_boot_error);
  }
}
static const struct nw_wifi_hooks gw_hooks={.registered=gw_on_registered,.rx=gw_on_rx,
  .scan_done=gw_on_scan,.beacon_loss=gw_on_lost,.eapol_status=gw_on_status};

int gw_snoc_start(const struct lx_platform *platform,const struct gw_snoc_board *board) {
  if(gw_ar) return -EBUSY;
  if(!platform||!board||!board->registers||board->register_bytes<0x800000||
     !board->power||!board->ready||!board->wlan_enable||!board->wlan_disable||
     !platform->alloc||!platform->free||!platform->dma_alloc||!platform->dma_free||!platform->dma_map||
     !platform->dma_sync||!platform->now_ms||!platform->sleep_ms||!platform->poll||!platform->fw_get) return -EINVAL;
  GW_TRACE("start: board ready check");
  if(!board->ready(board->ctx)) return -ENODEV;
  GW_TRACE("start: firmware preflight");
  int firmware_error=gw_firmware_preflight(platform);
  GW_TRACE("start: firmware preflight result %d",firmware_error);
  if(firmware_error)return firmware_error;
  gw_board=*board; lx_plat=platform; gw_boot_error=0; gw_ce_live=false; gw_regs_live=false; memset(&nw_wifi_stats,0,sizeof(nw_wifi_stats));
  memset(&gw_device,0,sizeof(gw_device)); gw_device.dev.name="guamp-wcn-snoc";
  nw_wifi_set_hooks(&gw_hooks);
  GW_TRACE("start: ath10k_core_create");
  gw_ar=ath10k_core_create(sizeof(struct ath10k_snoc),&gw_device.dev,ATH10K_BUS_SNOC,ATH10K_HW_WCN3990,&gw_hif);
  GW_TRACE("start: ath10k_core_create %s",gw_ar?"done":"FAILED");
  if(!gw_ar) return -ENOMEM;
  struct ath10k_snoc *s=ath10k_snoc_priv(gw_ar);
  s->dev=&gw_device; s->ar=gw_ar; s->mem=board->registers; s->mem_pa=0x0c800000;
  s->mem_len=board->register_bytes; s->ce.bus_ops=&gw_bus; gw_ar->ce_priv=&s->ce;
  gw_ar->regs=&wcn3990_regs; gw_ar->hw_values=&wcn3990_values; gw_ar->hw_ce_regs=&wcn3990_ce_regs;
  spin_lock_init(&s->ce.ce_lock); timer_setup(&s->rx_post_retry,ath10k_snoc_rx_replenish_retry,0);
  netif_napi_add(gw_ar->napi_dev,&gw_ar->napi,gw_napi_poll);
  int e=0;
  for(unsigned i=0;i<12;i++) {
    struct ath10k_snoc_pipe *p=&s->pipe_info[i];
    p->ce_hdl=&s->ce.ce_states[i]; p->pipe_num=i; p->hif_ce_state=gw_ar;
    p->buf_sz=host_ce_config_wlan[i].src_sz_max; p->ar_snoc=s; spin_lock_init(&p->pipe_lock);
    GW_TRACE("start: ath10k_ce_alloc_pipe %u",i);
    e=ath10k_ce_alloc_pipe(gw_ar,i,&host_ce_config_wlan[i]);
    if(e) {
      while(i) ath10k_ce_free_pipe(gw_ar,--i);
      ath10k_core_destroy(gw_ar); gw_ar=0; return e;
    }
  }
  struct ath10k_bus_params params={.dev_type=ATH10K_DEV_TYPE_LL,.chip_id=board->firmware_chip_id};
  GW_TRACE("start: ath10k_core_register");
  e=ath10k_core_register(gw_ar,&params);
  GW_TRACE("start: ath10k_core_register result %d",e);
  lx_log(e?3:6,"ath10k: phase probe queued: WCN3990 regs PA 0x0c800000 len 0x%x, chip id 0x%x, 12 CE pipes allocated (%d)",
    (unsigned)board->register_bytes,board->firmware_chip_id,e);
  if(e) { for(unsigned i=0;i<12;i++) ath10k_ce_free_pipe(gw_ar,i); ath10k_core_destroy(gw_ar); gw_ar=0; return e; }
  return 0; // enqueued probe, NOT associated/connected
}
static volatile unsigned gw_trace_polls,gw_trace_steps;
void gw_snoc_poll_ce(void) {
  if(!gw_ar||gw_boot_error||gw_servicing_ce||!gw_ce_live)return;
  gw_servicing_ce=true; gw_ce_users++;
  const bool trace=nw_wifi_trace_budget&&nw_wifi_trace_take(&gw_trace_polls);
  for(unsigned i=0;i<12&&gw_ce_live;i++) {   /* (guamp 04/11: a handler may have powered the block down: stop there) */
    if(trace) nw_wifi_trace("poll_ce %u: ath10k_ce_per_engine_service ce%u (ctrl +0x%x)",gw_trace_polls,i,
                            (unsigned)ath10k_snoc_priv(gw_ar)->ce.ce_states[i].ctrl_addr);
    ath10k_ce_per_engine_service(gw_ar,i);
  }
  if(trace) nw_wifi_trace("poll_ce %u: napi_schedule",gw_trace_polls);
  if(gw_ce_live) napi_schedule(&gw_ar->napi);
  gw_servicing_ce=false; gw_ce_users--; gw_rri_release_if_idle(gw_ar);
}
/* guamp 04/11: a cheap fingerprint of every copy engine's indices: it moves when anything was sent or received */
static unsigned gw_ce_fingerprint(void) {
  if(!gw_ar||!gw_ce_live) return 0;
  struct ath10k_snoc *s=ath10k_snoc_priv(gw_ar); unsigned f=0;
  for(unsigned i=0;i<12;i++) {
    struct ath10k_ce_ring *d=s->ce.ce_states[i].dest_ring,*r=s->ce.ce_states[i].src_ring;
    if(d) f=f*31+d->sw_index; if(r) f=f*31+r->sw_index*7+r->write_index;
  }
  return f;
}
int gw_snoc_step(void) {
  if(!gw_ar) return -ENODEV;
  if(gw_boot_error) return gw_boot_error;
  const bool trace=nw_wifi_trace_budget&&nw_wifi_trace_take(&gw_trace_steps);
  if(trace) nw_wifi_trace("step %u: poll_ce (stage %u, error %d)",gw_trace_steps,nw_wifi_stats.stage,nw_wifi_stats.last_err);
  const unsigned before=gw_ce_fingerprint();
  gw_snoc_poll_ce();
  if(gw_polling) { if(trace) nw_wifi_trace("step %u: nested, dispatcher skipped",gw_trace_steps); return 0; }
  if(trace) nw_wifi_trace("step %u: lx_pump",gw_trace_steps);
  gw_polling=true; lx_pump();
  if(trace) nw_wifi_trace("step %u: nw_sta_run",gw_trace_steps);
  nw_sta_run(); gw_polling=false;
  if(trace) nw_wifi_trace("step %u: done",gw_trace_steps);
  /* guamp 04/11: work = frames waiting to go out, or copy engine activity in this step. It was the first only, so
   * guamp-wifi-poll-max (142dffd) saw an idle link during reception and the handshake, and slowed the polling then too. */
  const unsigned after=gw_ce_fingerprint();
  return nw_sta_tx_pending() || after!=before;
}
int gw_snoc_stop(void) {
  if(!gw_ar) return 0;
  extern void gw_sta_shutdown(void);
  gw_sta_shutdown(); ath10k_core_unregister(gw_ar);
  if(gw_boot_error) return gw_boot_error;
  for(unsigned i=0;i<12;i++) ath10k_ce_free_pipe(gw_ar,i);
  extern void gw_devm_release(struct device *);
  gw_devm_release(&gw_device.dev); ath10k_core_destroy(gw_ar); gw_ar=0; return 0;
}
/* guamp 03/11: what still comes from the firmware while nothing is received (WMI kept moving at the beacon rate, HTT stopped),
 * and the WMI commands that went out just before (two, every time, right at the stall). Counted from the trace hooks
 * (trace.h); said by nw_sta.c's heartbeat with the 10-second line and in every quiet second. */
static struct { unsigned id, count; } gw_wmi_events[10]; static unsigned gw_wmi_events_other;
static struct { unsigned id; u64 ms; } gw_wmi_cmds[16]; static unsigned gw_wmi_cmd_count;
static unsigned gw_mgmt_drops[6], gw_mgmt_drops_said[6];     /* CAC, decrypt error, key cache miss, CRC, channel; 5: handed on */
static unsigned gw_htt_types[24], gw_htt_types_said[24];
void gw_note_wmi_cmd(unsigned id) { unsigned at=gw_wmi_cmd_count++&15; gw_wmi_cmds[at].id=id; gw_wmi_cmds[at].ms=lx_now_ms(); }
void gw_note_wmi_event(unsigned id) {
  for(unsigned i=0;i<10;i++) {
    if(gw_wmi_events[i].count&&gw_wmi_events[i].id==id) { gw_wmi_events[i].count++; return; }
    if(!gw_wmi_events[i].count) { gw_wmi_events[i].id=id; gw_wmi_events[i].count=1; return; }
  }
  gw_wmi_events_other++;
}
void gw_note_mgmt_drop(int why) { if(why>=0&&why<6) gw_mgmt_drops[why]++; }
void gw_wmi_timeout(unsigned cmd_id) {
  static unsigned timeouts; timeouts++;
  lx_log(3,"ath10k: WMI command %#x had no credit for 10 s (%u so far): refused, no hardware restart (this port cannot restart the chip)",cmd_id,timeouts);
}
void gw_note_htt_t2h(unsigned type) { gw_htt_types[type<23?type:23]++; }
/* Everything since the last call: WMI events by id, management frames dropped by reason, HTT messages by type; with
 * commands, the last WMI commands (id and how long ago). */
void nw_wifi_quiet_report(char *b,int n,int commands) {
  int at=snprintf(b,n,"WMI events");
  for(unsigned i=0;i<10&&at>=0&&at<n;i++) if(gw_wmi_events[i].count) at+=snprintf(b+at,n-at," %#x x%u",gw_wmi_events[i].id,gw_wmi_events[i].count);
  if(at>=0&&at<n&&gw_wmi_events_other) at+=snprintf(b+at,n-at," other x%u",gw_wmi_events_other);
  memset(gw_wmi_events,0,sizeof(gw_wmi_events)); gw_wmi_events_other=0;
  unsigned d[6]; for(unsigned i=0;i<6;i++) { d[i]=gw_mgmt_drops[i]-gw_mgmt_drops_said[i]; gw_mgmt_drops_said[i]=gw_mgmt_drops[i]; }
  if(at>=0&&at<n) at+=snprintf(b+at,n-at,"; mgmt rx handed on %u, dropped CAC %u decrypt %u key miss %u CRC %u channel %u; HTT",
    d[5],d[0],d[1],d[2],d[3],d[4]);
  for(unsigned i=0;i<24&&at>=0&&at<n;i++) { unsigned k=gw_htt_types[i]-gw_htt_types_said[i]; gw_htt_types_said[i]=gw_htt_types[i];
    if(k) at+=snprintf(b+at,n-at," t%u x%u",i,k); }
  if(!commands) return;
  u64 now=lx_now_ms();
  if(at>=0&&at<n) at+=snprintf(b+at,n-at,"; last WMI commands (id, ms ago):");
  for(unsigned k=gw_wmi_cmd_count>16?gw_wmi_cmd_count-16:0;k<gw_wmi_cmd_count&&at>=0&&at<n;k++)
    at+=snprintf(b+at,n-at," %#x %llu",gw_wmi_cmds[k&15].id,(unsigned long long)(now-gw_wmi_cmds[k&15].ms));
}
void nw_wifi_htt_stats(int type) { (void)type; /* ath12k-only diagnostic request intentionally not sent to ath10k */ }
void nw_wifi_tx_reasons(char *b,int n) { snprintf(b,n,"ath10k state %u error %d",nw_wifi_stats.stage,nw_wifi_stats.last_err); }
void nw_wifi_rx_state(char *b,int n) { snprintf(b,n,"rx %u beacons %u",nw_wifi_stats.frames,nw_wifi_stats.beacons); }
void nw_wifi_ring_state(char *b,int n) {
  if(!gw_ar) { snprintf(b,n,"radio off"); return; }
  struct ath10k_snoc *s=ath10k_snoc_priv(gw_ar);
  struct ath10k_ce_ring *r=s->ce.ce_states[3].src_ring;
  int at=snprintf(b,n,"WMI CE3 read %u write %u",r?r->sw_index:0,r?r->write_index:0);
  /* guamp 03/11: the HTT side too: its transmit ring (CE4: data frames, EAPOL) and the host receive ring the firmware
   * writes every received frame into (filled by the host, separate from the CE rings): an empty one stops all reception */
  { struct ath10k_ce_ring *h=s->ce.ce_states[4].src_ring; struct ath10k_htt *t=&gw_ar->htt;
    if(at>=0&&at<n) at+=snprintf(b+at,n-at,", HTT CE4 read %u write %u, HTT rx ring %d of %d (alloc %u, read %u, retry %s)",
      h?h->sw_index:0,h?h->write_index:0,t->rx_ring.fill_cnt,t->rx_ring.fill_level,
      t->rx_ring.alloc_idx.vaddr?(unsigned)__le32_to_cpu(*t->rx_ring.alloc_idx.vaddr):0u,t->rx_ring.sw_rd_idx.msdu_payld,
      t->rx_ring.refill_retry_timer.d.armed?"armed":"idle"); }
  /* guamp 03/11: reception stops at ~512 s with WMI still working and no HTT stats answer: every receive (destination)
   * ring with its software read index, write index and entries, to see which one is stuck. */
  if(at<0||at>=n)return;
  at+=snprintf(b+at,n-at,";");
  for(int i=0;i<CE_COUNT_MAX&&at<n-40;i++){
    struct ath10k_ce_ring *d=s->ce.ce_states[i].dest_ring;
    if(!d)continue;
    at+=snprintf(b+at,n-at," CE%d dst sw %u wr %u n %u",i,d->sw_index,d->write_index,d->nentries);
  }
}
