/* Runtime for the Linux-derived sources: one dispatcher runs queued work, timers and interrupt handlers; waits run it. */
#include "lx_os.h"
#include <stdlib.h>
#include "lx_plat.h"

#ifndef LX_BOOT_SETTLE_MS
#define LX_BOOT_SETTLE_MS 8000
#endif
#ifndef LX_MAX_DEPTH
#define LX_MAX_DEPTH 16
#endif
const struct lx_platform *lx_plat;
void (*lx_mmio_trace)(volatile void *a, u32 v);   /* host tests only */
static struct list_head g_work = LIST_HEAD_INIT(g_work);
static struct list_head g_timers = LIST_HEAD_INIT(g_timers);   /* delayed_work entries ordered by insertion */
static struct { int used; unsigned int irq; irq_handler_t h, t; void *id; int pending; int disabled, strict; } g_irq[64];
static u32 g_seed = 0x2545f491u;
static int g_depth;
/* What runs right now, further out in the nesting: a wait inside one of them pumps again, and like in Linux a work item or an interrupt handler
 * does not run a second time inside itself (it stays queued or pending until the outer run returned). Kept by pointer: a work function may free
 * its own item. */
static const void *g_running[LX_MAX_DEPTH + 2]; static int g_nrunning;
static int is_running(const void *p) { for (int i = 0; i < g_nrunning; i++) if (g_running[i] == p) return 1; return 0; }
static char *g_stack_top; unsigned long lx_stack_peak, lx_depth_peak;

u64 lx_now_ms(void) { return lx_plat->now_ms(); }
void lx_delay_us(unsigned int us) { lx_delay_ms((us + 999) / 1000 ? (us + 999) / 1000 : 0); if (!us) return; }
void lx_warn(const char *f, int l) { lx_log(4, "warning at %s:%d", f, l); }
void lx_bug(const char *f, int l) { lx_log(3, "bug at %s:%d", f, l); lx_plat->halt(); for (;;) {} }
void (*lx_log_tap)(int level, const char *line);
/* printk's own conversions. The kernel's vsnprintf knows the C ones only and prints "%pV" (every ath10k_info/warn/err/dbg
 * line: a struct va_format) as a bare pointer followed by "V". This formatter takes the conversions one at a time: the C ones
 * go to snprintf with the value fetched here, the Linux pointer extensions are expanded here, as lib/vsprintf.c does:
 * %pV (nested format, recursive), %pM %pMR %pMF %pm (MAC address), %pad %pap %pa (dma/phys address by reference),
 * %*ph %*phC %*phD %*phN (hex dump of up to 64 bytes), %pI4 %pi4 (IPv4); any other extension (%pK %pS %ps %pB ...) prints
 * the plain pointer. As in Linux, every letter or digit after the "p" belongs to the extension. */
struct lx_fmt_out { char *b; size_t n, at; };
static void fmt_put(struct lx_fmt_out *o, const char *s, size_t len) {
  for (; len; len--, s++, o->at++) if (o->at + 1 < o->n) o->b[o->at] = *s; }
static void fmt_putf(struct lx_fmt_out *o, const char *f, ...) __printf(2, 3);
static void fmt_putf(struct lx_fmt_out *o, const char *f, ...) {
  char t[80]; va_list ap; va_start(ap, f); vsnprintf(t, sizeof t, f, ap); va_end(ap); fmt_put(o, t, strlen(t)); }
static int fmt_alnum(char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }
static void fmt_core(struct lx_fmt_out *o, const char *f, va_list *ap, int depth);
static void fmt_pointer(struct lx_fmt_out *o, const char *ext, int width, void *p, int depth) {
  const u8 *a = p;
  if (*ext && (uintptr_t)p < 4096) { fmt_put(o, "(null)", 6); return; }
  if (ext[0] == 'V') {
    const struct va_format *v = p; va_list c;
    if (depth >= 4) { fmt_put(o, "(nested %pV)", 12); return; }
    va_copy(c, *v->va); fmt_core(o, v->fmt, &c, depth + 1); va_end(c);
  } else if (ext[0] == 'M' || ext[0] == 'm') {
    int rev = ext[1] == 'R'; char sep = ext[0] == 'm' ? 0 : ext[1] == 'F' ? '-' : ':';
    for (int i = 0; i < 6; i++) { if (i && sep) fmt_put(o, &sep, 1); fmt_putf(o, "%02x", a[rev ? 5 - i : i]); }
  } else if (ext[0] == 'a') {
    fmt_putf(o, "0x%016llx", (unsigned long long)*(const u64 *)p);
  } else if (ext[0] == 'h') {
    char sep = ext[1] == 'C' ? ':' : ext[1] == 'D' ? '-' : ext[1] == 'N' ? 0 : ' ';
    int n = width < 0 ? 1 : width > 64 ? 64 : width;
    for (int i = 0; i < n; i++) { if (i && sep) fmt_put(o, &sep, 1); fmt_putf(o, "%02x", a[i]); }
  } else if ((ext[0] == 'I' || ext[0] == 'i') && ext[1] == '4') {
    fmt_putf(o, ext[0] == 'I' ? "%u.%u.%u.%u" : "%03u.%03u.%03u.%03u", a[0], a[1], a[2], a[3]);
  } else {
    fmt_putf(o, "%p", p);
  }
}
static void fmt_core(struct lx_fmt_out *o, const char *f, va_list *ap, int depth) {
  while (*f) {
    const char *lit = f; while (*f && *f != '%') f++;
    fmt_put(o, lit, (size_t)(f - lit));
    if (!*f) break;
    const char *start = f++;
    char flags[8]; int nf = 0, width = -1, prec = -1, len = 0;   /* len: 0 int, 1 long, 2 long long, -1 short, -2 char */
    while (*f == '-' || *f == '+' || *f == ' ' || *f == '#' || *f == '0') { if (nf < 7) flags[nf++] = *f; f++; }
    flags[nf] = 0;
    if (*f == '*') { width = va_arg(*ap, int); f++; if (width < 0) { width = -width; if (nf < 7) { flags[nf++] = '-'; flags[nf] = 0; } } }
    else if (*f >= '0' && *f <= '9') { width = 0; while (*f >= '0' && *f <= '9') width = width * 10 + (*f++ - '0'); }
    if (*f == '.') { f++; prec = 0;
      if (*f == '*') { prec = va_arg(*ap, int); f++; } else while (*f >= '0' && *f <= '9') prec = prec * 10 + (*f++ - '0'); }
    for (;; f++) {
      if (*f == 'h') len = len < 0 ? -2 : -1;
      else if (*f == 'l') len = len ? 2 : 1;
      else if (*f == 'z' || *f == 't' || *f == 'j' || *f == 'q' || *f == 'L') len = 2;
      else break;
    }
    char c = *f; if (c) f++;
    char spec[32];
    if (width > 64) width = 64;
    if (prec > 64 && c != 's') prec = 64;
    if (c == 'd' || c == 'i' || c == 'u' || c == 'x' || c == 'X' || c == 'o' || c == 'c') {
      if (prec >= 0) snprintf(spec, sizeof spec, "%%%s%.0d.%d%s%c", flags, width < 0 ? 0 : width, prec, c == 'c' ? "" : "ll", c);
      else snprintf(spec, sizeof spec, "%%%s%.0d%s%c", flags, width < 0 ? 0 : width, c == 'c' ? "" : "ll", c);
      if (c == 'c') { fmt_putf(o, spec, va_arg(*ap, int)); continue; }
      int sgn = c == 'd' || c == 'i'; unsigned long long v;
      if (len >= 2) v = va_arg(*ap, unsigned long long);
      else if (len == 1) v = sgn ? (unsigned long long)va_arg(*ap, long) : va_arg(*ap, unsigned long);
      else { int i = va_arg(*ap, int);
        if (len == -1) v = sgn ? (unsigned long long)(long long)(short)i : (unsigned short)i;
        else if (len == -2) v = sgn ? (unsigned long long)(long long)(signed char)i : (unsigned char)i;
        else v = sgn ? (unsigned long long)(long long)i : (unsigned)i; }
      fmt_putf(o, spec, v);
    } else if (c == 's') {
      const char *s = va_arg(*ap, const char *); if (!s) s = "(null)";
      size_t n = 0; while (s[n] && (prec < 0 || n < (size_t)prec)) n++;
      int pad = width > (int)n ? width - (int)n : 0, left = 0;
      for (int i = 0; i < nf; i++) left |= flags[i] == '-';
      if (!left) for (; pad; pad--) fmt_put(o, " ", 1);
      fmt_put(o, s, n);
      for (; pad; pad--) fmt_put(o, " ", 1);
    } else if (c == 'p') {
      char ext[8]; int ne = 0; while (fmt_alnum(*f)) { if (ne < 7) ext[ne++] = *f; f++; }
      ext[ne] = 0;
      fmt_pointer(o, ext, width, va_arg(*ap, void *), depth);
    } else if (c == 'n') {
      (void)va_arg(*ap, void *);
    } else if (c == '%') {
      fmt_put(o, "%", 1);
    } else {
      fmt_put(o, start, (size_t)(f - start));   /* unknown (no floating point in the kernel): kept as written */
    }
  }
}
int lx_vsnprintf(char *buf, size_t size, const char *fmt, va_list ap) {
  struct lx_fmt_out o = { buf, size, 0 }; va_list c;
  va_copy(c, ap); fmt_core(&o, fmt, &c, 0); va_end(c);
  if (size) buf[o.at < size ? o.at : size - 1] = 0;
  return (int)o.at; }
void lx_log(int level, const char *fmt, ...) {
  char b[256]; va_list ap; va_start(ap, fmt); lx_vsnprintf(b, sizeof b, fmt, ap); va_end(ap); lx_plat->log(level, b);
  if (lx_log_tap) lx_log_tap(level, b); }
u32 lx_random_u32(void) { g_seed ^= g_seed << 13; g_seed ^= g_seed >> 17; g_seed ^= g_seed << 5; return g_seed; }

void *lx_alloc(size_t n, int zero) { void *p = lx_plat->alloc(n); if (p && zero) memset(p, 0, n); return p; }
void lx_free(const void *p) { if (p) lx_plat->free((void *)p); }
void *lx_memdup(const void *p, size_t n) { void *q = lx_alloc(n, 0); if (q) memcpy(q, p, n); return q; }
char *lx_strdup(const char *s) { return lx_memdup(s, strlen(s) + 1); }
int lx_kstrtoul(const char *s, int base, unsigned long *r) { char *e; *r = strtoul(s, &e, base); return e == s ? -EINVAL : 0; }
unsigned long find_first_bit(const unsigned long *a, unsigned long n) { for (unsigned long i = 0; i < n; i++) if (test_bit(i, a)) return i; return n; }
unsigned long find_last_bit(const unsigned long *a, unsigned long n) { for (unsigned long i = n; i; i--) if (test_bit(i - 1, a)) return i - 1; return n; }

/* work */
int lx_queue_work(struct work_struct *w) { if (w->queued) return 0; w->queued = 1; list_add_tail(&w->entry, &g_work); return 1; }
int lx_cancel_work(struct work_struct *w) { if (!w->queued) return 0; w->queued = 0; list_del_init(&w->entry); return 1; }
int lx_queue_delayed(struct delayed_work *d, unsigned long ms) {
  if (!ms) { d->armed = 0; return lx_queue_work(&d->work); }
  int was = d->armed; if (!was) list_add_tail(&d->work.entry, &g_timers);
  d->armed = 1; d->due = lx_now_ms() + ms; return !was; }
int lx_cancel_delayed(struct delayed_work *d) {
  int was = d->armed; if (was) { d->armed = 0; list_del_init(&d->work.entry); } return lx_cancel_work(&d->work) | was; }
static void timer_tramp(struct work_struct *w) {
  struct timer_list *t = container_of(w, struct timer_list, d.work); t->function(t); }
int lx_timer_mod(struct timer_list *t, unsigned long exp) {
  unsigned long now = (unsigned long)lx_now_ms(); lx_cancel_delayed(&t->d);
  t->d.work.func = timer_tramp;
  return lx_queue_delayed(&t->d, (long)(exp - now) > 0 ? exp - now : 0); }
int lx_timer_del(struct timer_list *t) { return lx_cancel_delayed(&t->d); }
static void tasklet_tramp(struct work_struct *w) {
  struct tasklet_struct *t = container_of(w, struct tasklet_struct, w);
  if (t->legacy) t->legacy(t->data); else if (t->func) t->func(t); }
void lx_tasklet_schedule(struct tasklet_struct *t) { t->w.func = tasklet_tramp; lx_queue_work(&t->w); }

/* interrupts */
int lx_request_irq(unsigned int irq, irq_handler_t h, irq_handler_t t, unsigned long fl, const char *n, void *id) {
  (void)fl; for (int i = 0; i < 64; i++) if (!g_irq[i].used) { g_irq[i].used = 1; g_irq[i].irq = irq; g_irq[i].h = h; g_irq[i].t = t; g_irq[i].id = id; g_irq[i].pending = 0; g_irq[i].disabled = 0; g_irq[i].strict = n && (!strcmp(n, "bhi") || !strcmp(n, "mhi")); return 0; }
  return -ENOMEM; }
void lx_free_irq(unsigned int irq, void *id) { for (int i = 0; i < 64; i++) if (g_irq[i].used && g_irq[i].irq == irq && g_irq[i].id == id) g_irq[i].used = 0; }
/* Disabling holds the bus controller's handlers ("bhi", "mhi"), whose disable and enable pair up around a power down: a disabled one keeps
 * what is pending and runs nothing until it is enabled again. The radio's copy engine and data groups share vectors with each other, so a
 * depth per vector would not pair up for them; their handlers check their own enable flags and keep running as before. */
void lx_disable_irq(unsigned int irq) { for (int i = 0; i < 64; i++) if (g_irq[i].used && g_irq[i].strict && g_irq[i].irq == irq) g_irq[i].disabled++; }
void lx_enable_irq(unsigned int irq) { for (int i = 0; i < 64; i++) if (g_irq[i].used && g_irq[i].strict && g_irq[i].irq == irq && g_irq[i].disabled) g_irq[i].disabled--; }
void lx_irq_raise(unsigned int irq) { for (int i = 0; i < 64; i++) if (g_irq[i].used && g_irq[i].irq == irq) g_irq[i].pending = 1; }

void lx_irq_raise_all(void);
volatile uint64_t nw_wifi_where[4];
/* guamp-wifi-trace (nw_abi.h): off while the budget is 0 */
volatile unsigned nw_wifi_trace_budget;
void (*volatile nw_wifi_trace_sink)(const char *line);
int nw_wifi_trace_take(volatile unsigned *count) {
  if (!nw_wifi_trace_budget || !nw_wifi_trace_sink || *count >= nw_wifi_trace_budget) return 0;
  ++*count; return 1; }
void nw_wifi_trace(const char *fmt, ...) {
  void (*sink)(const char *) = nw_wifi_trace_sink;
  if (!nw_wifi_trace_budget || !sink) return;
  char b[200]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap); sink(b); }
static volatile unsigned g_trace_where;
static void where(int kind, const void *fn) {
  nw_wifi_where[0] = kind; nw_wifi_where[1] = fn ? (uint64_t)((uintptr_t)fn - (uintptr_t)lx_now_ms) : 0;
  nw_wifi_where[2] = g_depth; nw_wifi_where[3] = lx_now_ms();
  /* the first N interrupt handlers, timers and work items the dispatcher runs (board polls and idle are left out) */
  if (nw_wifi_trace_budget && kind >= 1 && kind <= 3 && nw_wifi_trace_take(&g_trace_where))
    nw_wifi_trace("pump %u: runs %s, fn lx_now_ms+0x%llx (64-bit offset), depth %d", g_trace_where,
                  kind == 1 ? "irq" : kind == 2 ? "timer" : "work", (unsigned long long)nw_wifi_where[1], g_depth); }
/* The dispatcher: interrupts first, then due timers, then one round of queued work. */
int lx_pump(void) {
  int ran = 0;
  if (g_depth > LX_MAX_DEPTH) return 0;
  { char here; if (!g_stack_top) g_stack_top = &here; unsigned long used = (unsigned long)(g_stack_top - &here); if (used > lx_stack_peak && used < (1UL << 24)) lx_stack_peak = used; if ((unsigned long)g_depth + 1 > lx_depth_peak) lx_depth_peak = g_depth + 1; }
  g_depth++;
  where(4, (const void *)lx_plat->poll);
  lx_plat->poll();
  /* The endpoint's interrupts are writes into a sink nobody watches: every handler is offered a look about once a millisecond, in every wait as well
   * as in the board's loop (a handler finds out from its own device registers whether there is anything to do). */
  { static u64 last_raise; u64 t = lx_now_ms(); if (t != last_raise) { last_raise = t; lx_irq_raise_all(); } }
  for (int i = 0; i < 64; i++) if (g_irq[i].used && g_irq[i].pending && !g_irq[i].disabled && !is_running(&g_irq[i])) {
    g_irq[i].pending = 0; ran = 1;
    where(1, (const void *)g_irq[i].h);
    g_running[g_nrunning++] = &g_irq[i];
    irqreturn_t r = g_irq[i].h ? g_irq[i].h(g_irq[i].irq, g_irq[i].id) : IRQ_WAKE_THREAD;
    if (r == IRQ_WAKE_THREAD && g_irq[i].t) g_irq[i].t(g_irq[i].irq, g_irq[i].id);
    g_nrunning--;
  }
  u64 now = lx_now_ms();
  struct list_head *p, *n;
  for (p = g_timers.next; p != &g_timers; p = n) {
    n = p->next; struct delayed_work *d = container_of(p, struct delayed_work, work.entry);
    if (d->armed && now >= d->due && !is_running(&d->work)) {
      d->armed = 0; list_del_init(p); ran = 1; where(2, (const void *)d->work.func);
      g_running[g_nrunning++] = &d->work; d->work.func(&d->work); g_nrunning--; }
  }
  /* one item at a time off the shared list, so a wait inside a work function still reaches work queued behind it */
  for (int budget = 256; budget > 0; budget--) {
    struct work_struct *w = 0; struct list_head *e;
    for (e = g_work.next; e != &g_work; e = e->next) { struct work_struct *c = container_of(e, struct work_struct, entry); if (!is_running(c)) { w = c; break; } }
    if (!w) break;
    list_del_init(&w->entry); w->queued = 0; ran = 1;
    where(3, (const void *)w->func);
    g_running[g_nrunning++] = w; if (w->func) w->func(w); g_nrunning--;
  }
  g_depth--;
  where(0, 0);
  return ran;
}
void lx_idle(unsigned int ms) { lx_plat->sleep_ms(ms); }
void lx_delay_ms(unsigned int ms) {
  u64 end = lx_now_ms() + ms;
  while (lx_now_ms() < end) { if (!lx_pump()) lx_idle(1); }
}

/* device model */
#define MAXBUS 4
static const struct bus_type *g_bus[MAXBUS]; static struct device_driver *g_drv[16];
static int match_probe(struct device *d, struct device_driver *dr) {
  if (!d->bus || dr->bus != d->bus) return 0;
  if (d->bus->match && !d->bus->match(d, dr)) return 0;
  d->driver = dr; if (d->bus->probe) { int r = d->bus->probe(d); if (r) { d->driver = 0; return r; } }
  return 0; }
int bus_register(const struct bus_type *b) { for (int i = 0; i < MAXBUS; i++) if (!g_bus[i]) { g_bus[i] = b; return 0; } return -ENOMEM; }
void bus_unregister(const struct bus_type *b) { for (int i = 0; i < MAXBUS; i++) if (g_bus[i] == b) g_bus[i] = 0; }
static struct device g_root;
int driver_register(struct device_driver *dr) {
  for (int i = 0; i < 16; i++) if (!g_drv[i]) { g_drv[i] = dr; break; }
  for (int i = 0; i < 64; i++) (void)0;
  /* devices created before the driver are matched from the registry */
  extern void lx_probe_all(struct device_driver *);
  lx_probe_all(dr); return 0; }
void driver_unregister(struct device_driver *dr) { for (int i = 0; i < 16; i++) if (g_drv[i] == dr) g_drv[i] = 0; }
static struct device *g_devs[128];
void lx_probe_all(struct device_driver *dr) { for (int i = 0; i < 128; i++) if (g_devs[i] && !g_devs[i]->driver) match_probe(g_devs[i], dr); }
int device_register(struct device *d) {
  if (!d->children.next) INIT_LIST_HEAD(&d->children);
  INIT_LIST_HEAD(&d->sibling);
  if (d->parent) { if (!d->parent->children.next) INIT_LIST_HEAD(&d->parent->children); list_add_tail(&d->sibling, &d->parent->children); }
  for (int i = 0; i < 128; i++) if (!g_devs[i]) { g_devs[i] = d; break; }
  d->registered = 1;
  for (int i = 0; i < 16; i++) if (g_drv[i] && !d->driver) match_probe(d, g_drv[i]);
  return 0; }
void device_unregister(struct device *d) {
  if (!d->registered) return; d->registered = 0;
  if (d->driver && d->bus && d->bus->remove) d->bus->remove(d);
  d->driver = 0;
  if (d->parent) list_del_init(&d->sibling);
  for (int i = 0; i < 128; i++) if (g_devs[i] == d) g_devs[i] = 0;
  if (d->release) d->release(d); else if (d->type && d->type->release) d->type->release(d); }
int device_for_each_child(struct device *d, void *data, int (*fn)(struct device *, void *)) {
  if (!d->children.next) return 0;
  struct list_head *p, *n; int r = 0;
  for (p = d->children.next; p != &d->children; p = n) { n = p->next; r = fn(container_of(p, struct device, sibling), data); if (r) break; }
  return r; }
void lx_root_init(void) { INIT_LIST_HEAD(&g_root.children); }

/* ids */
int ida_alloc_range(struct ida *i, unsigned int lo, unsigned int hi, gfp_t f) {
  (void)f; for (unsigned int n = lo; n <= hi && n < 512; n++) if (!test_bit(n, i->bits)) { set_bit(n, i->bits); return n; } return -ENOSPC; }
void ida_free(struct ida *i, unsigned int id) { clear_bit(id, i->bits); }

/* DMA and firmware go to the platform */
void *lx_dma_alloc(struct device *d, size_t n, dma_addr_t *bus) { (void)d; return lx_plat->dma_alloc(n, bus); }
void *lx_dma_zalloc(size_t n) { u64 bus; return lx_plat->dma_alloc(n, &bus); }
void lx_dma_kfree(void *p) { if (p) lx_plat->dma_free(0, p, 0); }
void lx_dma_free(struct device *d, size_t n, void *p, dma_addr_t bus) { (void)d; lx_plat->dma_free(n, p, bus); }
dma_addr_t lx_dma_map(struct device *d, void *p, size_t n, int dir) { (void)d; return lx_plat->dma_map(p, n, dir); }
void lx_dma_sync(struct device *d, dma_addr_t bus, size_t n, int dir, int action) { (void)d; lx_plat->dma_sync(bus, n, dir, action); }
static int has_amss(const char *n) { for (; *n; n++) if (n[0] == 'a' && n[1] == 'm' && n[2] == 's' && n[3] == 's') return 1; return 0; }
int lx_request_firmware(const struct firmware **fw, const char *name) {
  struct firmware *f = lx_alloc(sizeof *f, 1); if (!f) return -ENOMEM;
  if (lx_plat->fw_get(name, &f->data, &f->size)) { lx_free(f); return -ENOENT; }
  static int settled;
  if (!settled && has_amss(name) && f->size >= 4096) {
    u32 sum = 0; const u32 *w = (const u32 *)f->data; for (size_t i = 0; i < 0x80000 / 4 && i < f->size / 4; i++) sum += w[i];
    lx_log(6, "firmware %s: %zu bytes, sum of the first 512 KiB %08x, first word %08x", name, f->size, sum, w[0]);
    settled = 1; }
  *fw = f; return 0; }
void lx_release_firmware(const struct firmware *fw) { if (fw) lx_free(fw); }

/* idr */
void idr_init(struct idr *i) { i->slot = 0; i->cap = 0; i->next = 0; }
void idr_destroy(struct idr *i) { lx_free(i->slot); idr_init(i); }
static int idr_grow(struct idr *i, unsigned int need) {
  if (need < i->cap) return 0;
  unsigned int nc = i->cap ? i->cap : 16; while (nc <= need) nc *= 2;
  void **n = lx_alloc(sizeof(void *) * nc, 1); if (!n) return -ENOMEM;
  if (i->slot) memcpy(n, i->slot, sizeof(void *) * i->cap);
  lx_free(i->slot); i->slot = n; i->cap = nc; return 0; }
int idr_alloc(struct idr *i, void *p, int start, int end, gfp_t f) {
  (void)f; unsigned int hi = end > 0 ? (unsigned int)end : 0x7fffffffu;
  for (unsigned int n = start; n < hi; n++) {
    if (n < i->cap && i->slot[n]) continue;
    if (idr_grow(i, n)) return -ENOMEM;
    i->slot[n] = p; return n; }
  return -ENOSPC; }
int idr_alloc_cyclic(struct idr *i, void *p, int start, int end, gfp_t f) {
  unsigned int from = i->next > (unsigned int)start ? i->next : (unsigned int)start;
  int r = idr_alloc(i, p, from, end, f); if (r < 0) r = idr_alloc(i, p, start, end, f);
  if (r >= 0) i->next = r + 1; return r; }
void *idr_find(const struct idr *i, unsigned long id) { return id < i->cap ? i->slot[id] : 0; }
void *idr_remove(struct idr *i, unsigned long id) { void *p = 0; if (id < i->cap) { p = i->slot[id]; i->slot[id] = 0; } return p; }
int idr_for_each(const struct idr *i, int (*fn)(int, void *, void *), void *data) {
  for (unsigned int n = 0; n < i->cap; n++) if (i->slot[n]) { int r = fn(n, i->slot[n], data); if (r) return r; }
  return 0; }

/* without interrupt delivery from the endpoint, every registered handler is given a turn each round; handlers check their own status */
void lx_irq_raise_all(void);
void lx_irq_raise_all(void) { for (int i = 0; i < 64; i++) if (g_irq[i].used) g_irq[i].pending = 1; }
