/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/* ath10k additions to the UniXNU single-dispatcher compatibility runtime.
 * Station mode uses direct .tx, without mac80211 TXQ/AP/TDLS facilities.
 * Unsupported optional providers return errors, never successful radio state.
 */
#include "gw_ath10k_port.h"
#include "lx_plat.h"
#include <linux/kfifo.h>
#include <linux/dmapool.h>
#include <linux/nvmem-consumer.h>
#include <linux/pm_qos.h>
#include <net/mac80211.h>
#include "nw_mac80211.h"
#include "nw_abi.h"

extern void read_random(void *,unsigned int); /* XNU entropy, required link dependency */
int gw_random_nonce(u8 *out,size_t n) {
  if(!out||n>4096) return -EINVAL;
  read_random(out,(unsigned int)n); return 0;
}
void eth_random_addr(u8 *out) { read_random(out,6); out[0]=(out[0]&0xfe)|2; }
u32 crc32_le(u32 crc,const unsigned char *p,size_t n) {
  while(n--) { crc^=*p++; for(unsigned i=0;i<8;i++) crc=(crc>>1)^(0xedb88320u&(0u-(crc&1))); }
  return crc;
}
int gw_printk(const char *format,...) {
  char text[384]; va_list ap; va_start(ap,format); int n=vsnprintf(text,sizeof(text),format,ap); va_end(ap);
  if(lx_plat&&lx_plat->log) lx_plat->log(7,text); return n;
}
int of_property_read_string_index(const struct device_node *n,const char *key,int i,const char **out) {
  (void)n; (void)key; (void)i; (void)out; return -ENOENT;
}
int of_property_read_u8_array(const struct device_node *n,const char *key,u8 *out,size_t len) {
  (void)n; (void)key; (void)out; (void)len; return -ENOENT;
}
int of_property_read_u8(const struct device_node *n,const char *key,u8 *out) {
  (void)n; (void)key; (void)out; return -ENOENT;
}
int device_get_mac_address(struct device *d,u8 *out) { (void)d; (void)out; return -ENOENT; }
struct nvmem_cell *devm_nvmem_cell_get(struct device *d,const char *name) { (void)d; (void)name; return ERR_PTR(-ENOENT); }
struct nvmem_cell *nvmem_cell_get(struct device *d,const char *name) { return devm_nvmem_cell_get(d,name); }
void *nvmem_cell_read(struct nvmem_cell *cell,size_t *n) { (void)cell; (void)n; return ERR_PTR(-ENOENT); }
void nvmem_cell_put(struct nvmem_cell *cell) { (void)cell; }

/* Fixed element FIFO, as used for HTT completion records. */
int gw_kfifo_alloc(struct gw_kfifo *f,unsigned int n,unsigned int size) {
  if(!f||!n||!size||n>(1u<<20)||size>4096) return -EINVAL;
  unsigned count=1; while(count<n) count<<=1;
  f->data=lx_alloc((size_t)count*size,1); if(!f->data) return -ENOMEM;
  f->head=f->tail=0; f->mask=count-1; f->size=size; return 0;
}
void gw_kfifo_free(struct gw_kfifo *f) { lx_free(f->data); memset(f,0,sizeof(*f)); }
unsigned int gw_kfifo_in(struct gw_kfifo *f,const void *p,unsigned int n) {
  if(!f->data||f->head-f->tail>f->mask+1) return 0;
  unsigned free=f->mask+1-(f->head-f->tail); if(n>free) n=free;
  for(unsigned i=0;i<n;i++) memcpy((u8 *)f->data+((f->head+i)&f->mask)*f->size,(const u8 *)p+i*f->size,f->size);
  f->head+=n; return n;
}
unsigned int gw_kfifo_out(struct gw_kfifo *f,void *p,unsigned int n) {
  if(!f->data||f->head-f->tail>f->mask+1) return 0;
  unsigned ready=f->head-f->tail; if(n>ready) n=ready;
  for(unsigned i=0;i<n;i++) memcpy((u8 *)p+i*f->size,(u8 *)f->data+((f->tail+i)&f->mask)*f->size,f->size);
  f->tail+=n; return n;
}

struct pool_item { struct pool_item *next; void *owner,*cpu; u64 owner_bus,bus; size_t bytes; };
struct dma_pool { size_t size,align,boundary; struct pool_item *items; };
struct dma_pool *dma_pool_create(const char *name,struct device *d,size_t size,size_t align,size_t boundary) {
  (void)name; (void)d;
  if(!size||!align||(align&(align-1))||align>65536||(boundary&&((boundary&(boundary-1))||size>boundary))) return 0;
  struct dma_pool *p=lx_alloc(sizeof(*p),1); if(p) { p->size=size; p->align=align; p->boundary=boundary; } return p;
}
void *dma_pool_alloc(struct dma_pool *p,gfp_t flags,dma_addr_t *bus) {
  (void)flags; if(!p||!bus||p->size>1u<<20) return 0;
  size_t n=p->size+max(p->align,p->boundary); u64 owner_bus=0;
  void *owner=lx_plat->dma_alloc(n,&owner_bus); if(!owner) return 0;
  size_t offset=(p->align-(owner_bus&(p->align-1)))&(p->align-1);
  if(p->boundary&&((owner_bus+offset)&~(p->boundary-1))!=((owner_bus+offset+p->size-1)&~(p->boundary-1)))
    offset=ALIGN(owner_bus+offset,p->boundary)-owner_bus;
  struct pool_item *i=lx_alloc(sizeof(*i),1);
  if(!i||offset>n||p->size>n-offset||((uintptr_t)owner+offset)&(p->align-1)) {
    lx_free(i); lx_plat->dma_free(n,owner,owner_bus); return 0;
  }
  i->next=p->items; i->owner=owner; i->owner_bus=owner_bus; i->bytes=n;
  i->cpu=(u8 *)owner+offset; i->bus=owner_bus+offset; p->items=i; *bus=i->bus; return i->cpu;
}
void dma_pool_free(struct dma_pool *p,void *cpu,dma_addr_t bus) {
  if(!p) return;
  for(struct pool_item **at=&p->items;*at;at=&(*at)->next) if((*at)->cpu==cpu&&(*at)->bus==bus) {
    struct pool_item *i=*at; *at=i->next; lx_plat->dma_free(i->bytes,i->owner,i->owner_bus); lx_free(i); return;
  }
  lx_log(3,"ath10k: invalid DMA pool release");
}
void dma_pool_destroy(struct dma_pool *p) {
  if(!p) return;
  while(p->items) dma_pool_free(p,p->items->cpu,p->items->bus);
  lx_free(p);
}

/* Firmware directly owns rate control/RX BA. These callbacks record telemetry;
 * no host AP scheduler is advertised or enabled by the guamp station adapter. */
static u64 tx_airtime,rx_airtime; static u32 rx_ba;
void ieee80211_sta_register_airtime(struct ieee80211_sta *sta,u8 tid,u32 tx,u32 rx) { (void)sta; (void)tid; tx_airtime+=tx; rx_airtime+=rx; }
void ieee80211_manage_rx_ba_offl(struct ieee80211_vif *v,const u8 *peer,unsigned tid) {
  (void)v; (void)peer; if(tid<16) rx_ba|=1u<<tid; else if(tid<32) rx_ba&=~(1u<<(tid-16));
}
void ieee80211_tx_rate_update(struct ieee80211_hw *hw,struct ieee80211_sta *sta,struct ieee80211_tx_info *i) {
  (void)hw; (void)sta; (void)i;
}
void ieee80211_queue_work(struct ieee80211_hw *hw,struct work_struct *w) { (void)hw; lx_queue_work(w); }
struct ieee80211_sta *ieee80211_find_sta_by_ifaddr(struct ieee80211_hw *hw,const u8 *addr,const u8 *local) {
  struct ieee80211_sta *sta=__ieee80211_iterate_stations(hw,0); (void)local;
  return sta&&ether_addr_equal(sta->addr,addr)?sta:0;
}
struct sk_buff *ieee80211_beacon_get_tim(struct ieee80211_hw *hw,struct ieee80211_vif *v,u16 *a,u16 *b,unsigned link) {
  (void)hw; (void)v; (void)a; (void)b; (void)link; return 0; /* no AP interface */
}
struct sk_buff *ieee80211_proberesp_get(struct ieee80211_hw *hw,struct ieee80211_vif *v) { (void)hw; (void)v; return 0; }
void ieee80211_tdls_oper_request(struct ieee80211_vif *v,const u8 *p,enum nl80211_tdls_operation op,u16 reason,gfp_t f) {
  (void)v; (void)p; (void)op; (void)reason; (void)f; lx_log(5,"ath10k: TDLS not advertised by station adapter");
}
struct ieee80211_txq *ieee80211_next_txq(struct ieee80211_hw *hw,u8 ac) { (void)hw; (void)ac; return 0; }
struct sk_buff *ieee80211_tx_dequeue(struct ieee80211_hw *hw,struct ieee80211_txq *q) { (void)hw; (void)q; return 0; }
void ieee80211_txq_schedule_start(struct ieee80211_hw *hw,u8 ac) { (void)hw; (void)ac; }
void __ieee80211_schedule_txq(struct ieee80211_hw *hw,struct ieee80211_txq *q,bool force) { (void)hw; (void)q; (void)force; }
bool ieee80211_txq_may_transmit(struct ieee80211_hw *hw,struct ieee80211_txq *q) { (void)hw; (void)q; return false; }
void ieee80211_txq_get_depth(struct ieee80211_txq *q,unsigned long *frames,unsigned long *bytes) { (void)q; *frames=0; *bytes=0; }
const unsigned char rfc1042_header[6]={0xaa,0xaa,0x03,0,0,0};

struct sk_buff *skb_get(struct sk_buff *s) { if(s) __atomic_add_fetch(&s->users,1,__ATOMIC_RELAXED); return s; }
void cpu_latency_qos_add_request(struct pm_qos_request *r,s32 value) { r->value=value; r->active=true; }
void cpu_latency_qos_update_request(struct pm_qos_request *r,s32 value) { if(r->active) r->value=value; }
void cpu_latency_qos_remove_request(struct pm_qos_request *r) { r->value=PM_QOS_DEFAULT_VALUE; r->active=false; }
/* XNU has no Linux CPU-latency governor. These are bookkeeping hints only,
 * not board clock or regulator operations. */
struct managed_item { struct managed_item *next; struct device *device; void *data; };
static struct managed_item *managed;
void *devm_kzalloc(struct device *d,size_t n,gfp_t f) {
  (void)f; struct managed_item *i=lx_alloc(sizeof(*i),1); if(!i) return 0;
  i->data=lx_alloc(n,1); if(!i->data) { lx_free(i); return 0; }
  i->device=d; i->next=managed; managed=i; return i->data;
}
void gw_devm_release(struct device *d) {
  for(struct managed_item **at=&managed;*at;) {
    struct managed_item *i=*at;
    if(i->device==d) { *at=i->next; lx_free(i->data); lx_free(i); } else at=&i->next;
  }
}

/* Apply, rather than ignore, the upstream ath world/country rule tables. */
const struct ieee80211_reg_rule *freq_reg_info(struct wiphy *w,u32 khz) {
  const struct ieee80211_regdomain *domain=w->regd;
  if(!domain) return ERR_PTR(-EINVAL);
  for(unsigned i=0;i<domain->n_reg_rules;i++) {
    const struct ieee80211_reg_rule *r=&domain->reg_rules[i];
    if(khz>=10000&&khz-10000>=r->freq_range.start_freq_khz&&khz+10000<=r->freq_range.end_freq_khz) return r;
  }
  return ERR_PTR(-ERANGE);
}
void wiphy_apply_custom_regulatory(struct wiphy *w,const struct ieee80211_regdomain *domain) {
  w->regd=domain;
  for(unsigned b=0;b<NUM_NL80211_BANDS;b++) if(w->bands[b]) {
    struct ieee80211_supported_band *band=w->bands[b];
    for(int i=0;i<band->n_channels;i++) {
      struct ieee80211_channel *c=&band->channels[i];
      const struct ieee80211_reg_rule *r=freq_reg_info(w,c->center_freq*1000);
      if(IS_ERR(r)) { c->flags|=IEEE80211_CHAN_DISABLED; continue; }
      if(r->flags&NL80211_RRF_NO_IR) c->flags|=IEEE80211_CHAN_NO_IR;
      if(r->flags&NL80211_RRF_DFS) c->flags|=IEEE80211_CHAN_RADAR;
      if(r->flags&NL80211_RRF_NO_OFDM) c->flags|=IEEE80211_CHAN_NO_OFDM;
      int maximum=r->power_rule.max_eirp/100;
      c->max_reg_power=maximum; c->max_power=min(c->max_power,maximum);
      c->orig_flags=c->flags; c->orig_mpwr=c->max_power;
    }
  }
}
const char *reg_initiator_name(enum nl80211_reg_initiator who) {
  switch(who) { case NL80211_REGDOM_SET_BY_CORE:return "core"; case NL80211_REGDOM_SET_BY_USER:return "user";
    case NL80211_REGDOM_SET_BY_DRIVER:return "driver"; case NL80211_REGDOM_SET_BY_COUNTRY_IE:return "country-ie"; }
  return "unknown";
}
int regulatory_hint(struct wiphy *w,const char *alpha2) {
  /* No regulatory database is provided. In Linux the hint is only queued and
   * returns 0 when no database answers: the wiphy keeps the custom table that
   * ath_regd_init() already applied (the restrictive ath default world table
   * for a non-world EEPROM domain), and the firmware still receives the
   * EEPROM regpair through ath10k_regd_update(). Returning an error here made
   * ath10k_mac_register() fail for every BDF with a country/default domain
   * (e.g. 0x0 -> "US"), i.e. before the first scan. */
  (void)w;
  lx_log(5,"ath10k: regulatory hint %c%c kept as driver world table (no regdb)",
    alpha2&&alpha2[0]?alpha2[0]:'?',alpha2&&alpha2[0]&&alpha2[1]?alpha2[1]:'?');
  return 0;
}

static unsigned long stoppedQueues;
void ieee80211_stop_queue(struct ieee80211_hw *hw,int q) { (void)hw; if(q>=0&&q<64) stoppedQueues|=1UL<<q; }
void ieee80211_wake_queue(struct ieee80211_hw *hw,int q) { (void)hw; if(q>=0&&q<64) stoppedQueues&=~(1UL<<q); }
int gw_queue_stopped(int q) { return q>=0&&q<64&&((stoppedQueues>>q)&1); }
void gw_stop_queues(struct ieee80211_hw *hw) { stoppedQueues=hw->queues>=64?~0UL:(1UL<<hw->queues)-1; }
void gw_wake_queues(void) { stoppedQueues=0; }
