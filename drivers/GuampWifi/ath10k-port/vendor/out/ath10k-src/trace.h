#pragma once
#define trace_ath10k_htt_pktlog(...) ((void)0)
#define trace_ath10k_htt_rx_desc(...) ((void)0)
#define trace_ath10k_htt_stats(...) ((void)0)
#define trace_ath10k_htt_tx(...) ((void)0)
#define trace_ath10k_log_dbg(...) ((void)0)
#define trace_ath10k_log_dbg_dump(...) ((void)0)
#define trace_ath10k_log_dbg_enabled(...) 0
/* guamp: an ath10k_err() hands its format to the trace (gw_snoc.c), which extends the register trace after an error */
void nw_wifi_trace_error(int level, const char *what);
#define trace_ath10k_log_err(ar, vaf) nw_wifi_trace_error(3, (vaf)->fmt)
#define trace_ath10k_log_info(...) ((void)0)
#define trace_ath10k_log_warn(...) ((void)0)
#define trace_ath10k_rx_hdr(...) ((void)0)
#define trace_ath10k_rx_payload(...) ((void)0)
#define trace_ath10k_tx_hdr(...) ((void)0)
#define trace_ath10k_tx_payload(...) ((void)0)
#define trace_ath10k_txrx_tx_unref(...) ((void)0)
/* guamp 03/11: reception stops (0 frames, not even a beacon) while the WMI receive pipe keeps moving at the beacon rate, and
 * every time exactly two WMI commands had just gone out: each WMI command and event id is counted (gw_snoc_tail.inc), and
 * the last commands and the events of a quiet second are logged with the "wifi: quiet" lines. */
void gw_note_wmi_cmd(unsigned id); void gw_note_wmi_event(unsigned id);
void gw_note_mgmt_drop(int why); void gw_note_htt_t2h(unsigned type);
#define trace_ath10k_wmi_cmd(ar, id, buf, len) gw_note_wmi_cmd((unsigned)(id))
#define trace_ath10k_wmi_dbglog(...) ((void)0)
#define trace_ath10k_wmi_diag(...) ((void)0)
#define trace_ath10k_wmi_diag_container(...) ((void)0)
#define trace_ath10k_wmi_event(ar, id, buf, len) gw_note_wmi_event((unsigned)(id))
