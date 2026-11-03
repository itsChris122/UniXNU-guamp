// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp 04/11, piano B del touch: rilevatore software dei tocchi dalla mappa mutual 32x18 (delta IIR o raw DC del
// firmware Himax), senza le coordinate del firmware. Nessun accesso all'hardware, solo interi (il kernel non usa la FPU);
// provato sul PC da tests/softtouch_host.cpp con mappe sintetiche e con le mappe TWRP di riferimento. Usato da
// GuampSoftTouch.inc (lettura della mappa dalla DSRAM e pubblicazione verso il percorso HID esistente).
//
// Per ogni mappa (indice i = t * rx + r, come hx_diag_read: tx righe da rx valori):
//  1. baseline propria: media delle prime cfg.learn mappe (a riposo), poi media mobile lenta (1/2^slowShift per mappa) sui
//     nodi lontani dai tocchi, veloce (1/2^fastShift) dove il valore va nel verso opposto al tocco (baseline presa col dito
//     appoggiato, o deriva); maschera automatica dei nodi saturi o troppo rumorosi in apprendimento (una riga o colonna con
//     piu' di meta' nodi mascherati si maschera tutta: la riga TX0 satura del 03/11) piu' le maschere dei boot-arg;
//  2. differenza dalla baseline nel verso del tocco (cfg.sign), meno il modo comune di ogni riga TX e/o colonna RX
//     (mediana: il rumore del display TDDI arriva a righe intere);
//  3. rumore = mediana di |d| * 1.5 (media mobile), soglia di tocco = max(thrMin, noiseK * rumore / 4), soglia di
//     mantenimento = relPct % della soglia di tocco (isteresi);
//  4. massimi locali (8 vicini) sopra la soglia di mantenimento, scarto dei nodi isolati (meno di minNodes nodi accesi
//     attorno, se non molto forti), soppressione dei massimi vicini a uno piu' forte (distanza <= 1 nodo sempre, fino a
//     suppR nodi se la valle tra i due resta sopra valleyPct % del piu' debole);
//  5. posizione sub-nodo in 1/256 di nodo: per asse, profilo a 3 punti (somma sulla finestra trasversale 2 win + 1) e
//     vertice della gaussiana per i 3 punti (log2 intero a tabella); nodo di bordo o vicino mascherato: gaussiana di
//     larghezza sigma2 dal solo lato presente; interp = 2 parabola, 0 baricentro pesato (peso d - soglia mant. / 2);
//  6. pixel: x dai TX (18 su 720), y dagli RX (32 su 1600) salvo scambio/specchio (cfg.axes); con edge = 1 il centro del
//     primo nodo va al pixel 0 e quello dell'ultimo a max-1 (centro della mappa = centro dello schermo: 8.5 -> 359.5,
//     15.5 -> 799.5), con edge = 0 il nodo n copre i pixel [n * passo, (n + 1) * passo);
//  7. tracciamento: abbinamento ingordo per distanza (porta gatePx), ID (slot) stabili per tutta la vita del contatto,
//     nuovo contatto solo sopra la soglia di tocco e dopo downFrames mappe, rilascio dopo upFrames mappe senza
//     abbinamento, filtro della posizione (passo intero sopra fastPx, smooth/16 sotto), al massimo maxContacts contatti;
//     un contatto fermo (entro stuckPx) per stuckFrames mappe e' preso come residuo: assorbito nella baseline e rilasciato.
#ifndef GUAMP_SOFT_TOUCH_CORE_H
#define GUAMP_SOFT_TOUCH_CORE_H
#include <stdint.h>

namespace guamp_softtouch {
constexpr unsigned kMaxRx = 32, kMaxTx = 32, kMaxNodes = 1024, kMaxPeaks = 16, kMaxTracks = 10;

// Senza inizializzatori (oggetto globale del kernel senza costruttore statico): i valori di default in defaultConfig().
struct Config {
  unsigned rx, tx;          // canali (gChip.rx_num / tx_num)
  int width, height;     // pixel del pannello
  int sign;                       // verso del tocco: 1 positivo, -1 negativo, 0 per ogni mappa il |massimo| piu' grande
  uint32_t txMask, rxMask;    // righe TX / colonne RX ignorate (bit n = canale n)
  int autoMask;                   // maschera i nodi saturi/rumorosi trovati in apprendimento
  int satLevel;               // |valore| da cui un nodo e' saturo
  unsigned learn;                // mappe per la baseline iniziale
  unsigned slowShift, fastShift;
  int cm;                         // modo comune: bit 0 per riga TX, bit 1 per colonna RX
  int thrMin;                    // soglia di tocco minima (conteggi della mappa)
  int noiseK;                    // soglia = noiseK / 4 * rumore (default 6 sigma)
  int relPct;                    // soglia di mantenimento in % della soglia di tocco
  unsigned minNodes;              // nodi accesi minimi attorno al massimo (incluso)
  unsigned suppR;                 // raggio di soppressione con controllo della valle (nodi)
  int valleyPct;
  unsigned win;                   // raggio della finestra trasversale / del baricentro (1: 3x3, 2: 5x5)
  int interp;                     // sub-nodo: 1 gaussiana a 3 punti, 2 parabola, 0 baricentro pesato
  int sigma2;                   // larghezza del dito per i nodi di bordo: sigma^2 in 1/256 di nodo^2 (0,85^2)
  int axes;                       // bit 0 x dagli RX e y dai TX, bit 1 specchio x, bit 2 specchio y
  int edge;                       // 1 stiramento centro primo/ultimo nodo -> bordo, 0 celle uniformi
  unsigned maxContacts;
  int gatePx;                   // distanza massima di abbinamento tra due mappe
  unsigned downFrames, upFrames;
  int fastPx, smooth;        // filtro della posizione
  int stuckPx;
  unsigned stuckFrames;         // 10 s a 60 Hz; 0 = mai
};
inline Config defaultConfig() {
  Config c;
  c.rx = 32;
  c.tx = 18;
  c.width = 720;
  c.height = 1600;
  c.sign = 1;
  c.txMask = 0;
  c.rxMask = 0;
  c.autoMask = 1;
  c.satLevel = 32000;
  c.learn = 16;
  c.slowShift = 6;
  c.fastShift = 2;
  c.cm = 1;
  c.thrMin = 80;
  c.noiseK = 24;
  c.relPct = 55;
  c.minNodes = 2;
  c.suppR = 3;
  c.valleyPct = 70;
  c.win = 1;
  c.interp = 1;
  c.sigma2 = 185;
  c.axes = 0;
  c.edge = 1;
  c.maxContacts = 5;
  c.gatePx = 220;
  c.downFrames = 2;
  c.upFrames = 2;
  c.fastPx = 24;
  c.smooth = 8;
  c.stuckPx = 12;
  c.stuckFrames = 600;
  return c;
}

struct Contact { int slot, x, y, amp; };

struct Stats {
  unsigned frames, learnLeft, masked, saturated, peaks, rejected, detections, down;
  int noise, thrHigh, thrLow, maxPos, maxNeg;   // maxPos/maxNeg: estremi della differenza (prima del verso)
  unsigned maxAt, minAt;
  unsigned started, released, stuck;
};

struct Track {
  int state;                          // 0 libero, 1 in attesa, 2 premuto
  int slot, x, y, rawX, rawY, ax, ay, amp, pt, pr;
  unsigned hits, miss, still;
  bool matched;
};

class Detector {
 public:
  Config cfg;
  Stats st;

  void reset() {
    st = Stats();
    for (unsigned i = 0; i < kMaxNodes; ++i) { base16_[i] = 0; sum_[i] = 0; sq_[i] = 0; mask_[i] = 0; }
    for (unsigned k = 0; k < kMaxTracks; ++k) tr_[k] = Track();
    noise16_ = 0;
    st.learnLeft = cfg.learn ? cfg.learn : 1;
    learnN_ = 0;
    absorbN_ = 0;
    staticMask();
  }
  void configure(const Config &c) {
    cfg = c;
    if (cfg.rx > kMaxRx) cfg.rx = kMaxRx;
    if (cfg.tx > kMaxTx) cfg.tx = kMaxTx;
    if (cfg.rx * cfg.tx > kMaxNodes) cfg.tx = kMaxNodes / cfg.rx;
    if (cfg.maxContacts > kMaxTracks) cfg.maxContacts = kMaxTracks;
    if (!cfg.maxContacts) cfg.maxContacts = 1;
    if (cfg.win > 2) cfg.win = 2;
    if (cfg.slowShift > 12) cfg.slowShift = 12;
    if (cfg.fastShift > cfg.slowShift) cfg.fastShift = cfg.slowShift;
    if (cfg.relPct < 10) cfg.relPct = 10;
    if (cfg.relPct > 100) cfg.relPct = 100;
    if (cfg.smooth < 1) cfg.smooth = 1;
    if (cfg.smooth > 16) cfg.smooth = 16;
    reset();
  }
  bool masked(unsigned t, unsigned r) const { return mask_[t * cfg.rx + r] != 0; }
  int base(unsigned i) const { return base16_[i] / 16; }
  const int32_t *diff() const { return d_; }   // ultima differenza (nel verso del tocco, modo comune tolto)

  // Una mappa; ritorna i contatti premuti (al massimo cfg.maxContacts, in ordine di slot).
  unsigned frame(const int32_t *v, Contact *out) {
    const unsigned rx = cfg.rx, tx = cfg.tx, n = rx * tx;
    st.frames++;
    if (st.learnLeft) { learnStep(v); return 0; }
    // 1-2: differenza dalla baseline, nodi saturi a 0.
    st.saturated = 0;
    int mx = -2147483647, mn = 2147483647;
    for (unsigned i = 0; i < n; ++i) {
      int d = 0;
      if (!mask_[i]) {
        if (v[i] >= cfg.satLevel || v[i] <= -cfg.satLevel) st.saturated++;
        else d = v[i] - base16_[i] / 16;
      }
      d_[i] = d;
      if (!mask_[i]) {
        if (d > mx) { mx = d; st.maxAt = i; }
        if (d < mn) { mn = d; st.minAt = i; }
      }
    }
    st.maxPos = mx; st.maxNeg = mn;
    if (cfg.cm & 1) for (unsigned t = 0; t < tx; ++t) commonMode(t * rx, 1, rx);
    if (cfg.cm & 2) for (unsigned r = 0; r < rx; ++r) commonMode(r, rx, tx);
    int s = cfg.sign;
    if (!s) {
      int p = 0, q = 0;
      for (unsigned i = 0; i < n; ++i) { if (d_[i] > p) p = d_[i]; if (-d_[i] > q) q = -d_[i]; }
      s = p >= q ? 1 : -1;
    }
    if (s < 0) for (unsigned i = 0; i < n; ++i) d_[i] = -d_[i];
    // 3: rumore e soglie.
    updateNoise();
    // 4-5: massimi, baricentri, pixel.
    unsigned np = findPeaks();
    // 7: tracciamento.
    track(np);
    // 1: baseline.
    updateBaseline(v);
    unsigned k = 0;
    st.down = 0;
    for (int slot = 0; slot < static_cast<int>(kMaxTracks); ++slot)
      for (unsigned j = 0; j < kMaxTracks; ++j)
        if (tr_[j].state == 2 && tr_[j].slot == slot) {
          if (k < cfg.maxContacts) out[k++] = Contact{slot, tr_[j].x, tr_[j].y, tr_[j].amp};
          st.down++;
        }
    return k;
  }

  // Posizione in pixel di un punto della mappa in 1/256 di nodo (pt lungo i TX, pr lungo gli RX).
  void toPixels(int pt, int pr, int *x, int *y) const {
    const bool swap = cfg.axes & 1;
    int ax = swap ? pr : pt, ay = swap ? pt : pr;
    const int nx = static_cast<int>(swap ? cfg.rx : cfg.tx), ny = static_cast<int>(swap ? cfg.tx : cfg.rx);
    int px = axisPx(ax, nx, cfg.width), py = axisPx(ay, ny, cfg.height);
    if (cfg.axes & 2) px = cfg.width - 1 - px;
    if (cfg.axes & 4) py = cfg.height - 1 - py;
    *x = px; *y = py;
  }

 private:
  int32_t base16_[kMaxNodes];
  int64_t sum_[kMaxNodes], sq_[kMaxNodes];
  uint8_t mask_[kMaxNodes], frozen_[kMaxNodes];
  int32_t d_[kMaxNodes], sd_[kMaxNodes];
  unsigned hist_[256];
  unsigned learnN_;
  int noise16_;
  struct Peak { int amp; unsigned t, r; int pt, pr, x, y; bool used; };
  Peak pk_[kMaxPeaks];
  Track tr_[kMaxTracks];

  static int iabs(int v) { return v < 0 ? -v : v; }
  int axisPx(int pos256, int nodes, int pixels) const {
    int p;
    if (nodes <= 1) p = pixels / 2;
    else if (cfg.edge) p = static_cast<int>((static_cast<int64_t>(pos256) * (pixels - 1) + 128 * (nodes - 1)) / (256 * (nodes - 1)));
    else p = static_cast<int>((static_cast<int64_t>(pos256 + 128) * pixels) / (256 * nodes));
    return p < 0 ? 0 : p > pixels - 1 ? pixels - 1 : p;
  }

  void staticMask() {
    for (unsigned t = 0; t < cfg.tx; ++t)
      for (unsigned r = 0; r < cfg.rx; ++r)
        mask_[t * cfg.rx + r] = ((t < 32 && (cfg.txMask >> t) & 1u) || (r < 32 && (cfg.rxMask >> r) & 1u)) ? 1 : 0;
  }

  void learnStep(const int32_t *v) {
    const unsigned n = cfg.rx * cfg.tx;
    for (unsigned i = 0; i < n; ++i) {
      int x = v[i];
      if (x >= cfg.satLevel || x <= -cfg.satLevel) mask_[i] |= 2;  // saturo almeno una volta
      sum_[i] += x;
      sq_[i] += static_cast<int64_t>(x) * x;
    }
    learnN_++;
    if (--st.learnLeft) return;
    // Baseline = media; rumore per nodo = varianza.
    unsigned hist[64] = {};
    int32_t *sd = sd_;
    for (unsigned i = 0; i < n; ++i) {
      base16_[i] = static_cast<int32_t>(sum_[i] * 16 / static_cast<int64_t>(learnN_));
      const int64_t mean = sum_[i] / static_cast<int64_t>(learnN_);
      int64_t var = sq_[i] / static_cast<int64_t>(learnN_) - mean * mean;
      if (var < 0) var = 0;
      sd[i] = isqrt(var);
      hist[sd[i] < 63 ? sd[i] : 63]++;
    }
    // Rumore tipico = mediana delle deviazioni; un nodo oltre 8 volte (e oltre thrMin / 2) e' mascherato.
    unsigned acc = 0, med = 0;
    for (; med < 64; ++med) { acc += hist[med]; if (acc * 2 >= n) break; }
    int lim = static_cast<int>(med) * 8;
    if (lim < cfg.thrMin / 2) lim = cfg.thrMin / 2;
    if (cfg.autoMask) {
      for (unsigned i = 0; i < n; ++i)
        if ((mask_[i] & 2) || sd[i] > lim) mask_[i] |= 2;
      // Righe e colonne con piu' di meta' dei nodi mascherati: tutte fuori.
      for (unsigned t = 0; t < cfg.tx; ++t) {
        unsigned c = 0;
        for (unsigned r = 0; r < cfg.rx; ++r) c += mask_[t * cfg.rx + r] ? 1 : 0;
        if (c * 2 > cfg.rx) for (unsigned r = 0; r < cfg.rx; ++r) mask_[t * cfg.rx + r] |= 4;
      }
      for (unsigned r = 0; r < cfg.rx; ++r) {
        unsigned c = 0;
        for (unsigned t = 0; t < cfg.tx; ++t) c += mask_[t * cfg.rx + r] ? 1 : 0;
        if (c * 2 > cfg.tx) for (unsigned t = 0; t < cfg.tx; ++t) mask_[t * cfg.rx + r] |= 4;
      }
    } else {
      for (unsigned i = 0; i < n; ++i) mask_[i] &= 1;
    }
    st.masked = 0;
    for (unsigned i = 0; i < n; ++i) st.masked += mask_[i] ? 1 : 0;
    noise16_ = static_cast<int>(med) * 16;
    if (noise16_ < 16) noise16_ = 16;
  }

  static int isqrt(int64_t v) {
    if (v <= 0) return 0;
    int64_t x = v, y = (x + 1) / 2;
    while (y < x) { x = y; y = (x + v / x) / 2; }
    return x > 2147483647 ? 2147483647 : static_cast<int>(x);
  }

  // Mediana dei nodi non mascherati di una riga/colonna, sottratta da tutti.
  void commonMode(unsigned start, unsigned step, unsigned count) {
    int vals[kMaxRx > kMaxTx ? kMaxRx : kMaxTx];
    unsigned m = 0;
    for (unsigned k = 0; k < count; ++k) {
      const unsigned i = start + k * step;
      if (mask_[i]) continue;
      int x = d_[i], j = static_cast<int>(m);
      while (j > 0 && vals[j - 1] > x) { vals[j] = vals[j - 1]; --j; }
      vals[j] = x;
      ++m;
    }
    if (m < 4) return;
    const int med = (m & 1) ? vals[m / 2] : (vals[m / 2 - 1] + vals[m / 2]) / 2;
    for (unsigned k = 0; k < count; ++k) {
      const unsigned i = start + k * step;
      if (!mask_[i]) d_[i] -= med;
    }
  }

  void updateNoise() {
    const unsigned n = cfg.rx * cfg.tx;
    unsigned *hist = hist_, m = 0;
    for (unsigned k = 0; k < 256; ++k) hist[k] = 0;
    for (unsigned i = 0; i < n; ++i) {
      if (mask_[i]) continue;
      const int a = iabs(d_[i]);
      hist[a < 255 ? a : 255]++;
      ++m;
    }
    if (!m) return;
    unsigned acc = 0, med = 0;
    for (; med < 256; ++med) { acc += hist[med]; if (acc * 2 >= m) break; }
    const int now16 = static_cast<int>(med) * 24;          // * 1.5 * 16
    noise16_ += (now16 - noise16_) / 8;
    if (noise16_ < 16) noise16_ = 16;
    st.noise = noise16_ / 16;
    int thr = cfg.noiseK * noise16_ / 64;
    if (thr < cfg.thrMin) thr = cfg.thrMin;
    st.thrHigh = thr;
    st.thrLow = thr * cfg.relPct / 100;
    if (st.thrLow < 1) st.thrLow = 1;
  }

  int at(int t, int r) const {
    if (t < 0 || r < 0 || t >= static_cast<int>(cfg.tx) || r >= static_cast<int>(cfg.rx)) return 0;
    const unsigned i = static_cast<unsigned>(t) * cfg.rx + static_cast<unsigned>(r);
    return mask_[i] ? 0 : d_[i];
  }

  // Valle tra due massimi: minimo lungo il segmento (passi di 1/4 di nodo).
  int valley(const Peak &a, const Peak &b) const {
    const int dt = static_cast<int>(b.t) - static_cast<int>(a.t), dr = static_cast<int>(b.r) - static_cast<int>(a.r);
    const int steps = 4 * (iabs(dt) > iabs(dr) ? iabs(dt) : iabs(dr));
    int lo = a.amp < b.amp ? a.amp : b.amp;
    for (int k = 1; k < steps; ++k) {
      const int t = static_cast<int>(a.t) + (dt * k * 2 + (dt >= 0 ? steps : -steps)) / (2 * steps);
      const int r = static_cast<int>(a.r) + (dr * k * 2 + (dr >= 0 ? steps : -steps)) / (2 * steps);
      const int v = at(t, r);
      if (v < lo) lo = v;
    }
    return lo;
  }

  unsigned findPeaks() {
    const int rx = static_cast<int>(cfg.rx), tx = static_cast<int>(cfg.tx);
    unsigned np = 0;
    st.peaks = 0; st.rejected = 0;
    for (int t = 0; t < tx; ++t)
      for (int r = 0; r < rx; ++r) {
        const int v = at(t, r);
        if (v < st.thrLow || mask_[t * rx + r]) continue;
        bool peak = true;
        unsigned lit = 1;
        for (int a = -1; a <= 1 && peak; ++a)
          for (int b = -1; b <= 1; ++b) {
            if (!a && !b) continue;
            const int w = at(t + a, r + b);
            const bool before = a < 0 || (a == 0 && b < 0);
            if (before ? w >= v : w > v) { peak = false; break; }
            if (w >= st.thrLow) lit++;
          }
        if (!peak) continue;
        st.peaks++;
        if (lit < cfg.minNodes && v < 2 * st.thrHigh) { st.rejected++; continue; }
        Peak p{v, static_cast<unsigned>(t), static_cast<unsigned>(r), 0, 0, 0, 0, false};
        if (np < kMaxPeaks) pk_[np++] = p;
        else {  // tiene i piu' forti
          unsigned w = 0;
          for (unsigned k = 1; k < np; ++k) if (pk_[k].amp < pk_[w].amp) w = k;
          if (pk_[w].amp < v) pk_[w] = p;
        }
      }
    // Ordine per ampiezza decrescente.
    for (unsigned i = 1; i < np; ++i) {
      Peak p = pk_[i];
      unsigned j = i;
      while (j > 0 && pk_[j - 1].amp < p.amp) { pk_[j] = pk_[j - 1]; --j; }
      pk_[j] = p;
    }
    // Soppressione.
    unsigned kept = 0;
    for (unsigned i = 0; i < np; ++i) {
      bool ok = true;
      for (unsigned j = 0; j < kept && ok; ++j) {
        const int dt = iabs(static_cast<int>(pk_[i].t) - static_cast<int>(pk_[j].t));
        const int dr = iabs(static_cast<int>(pk_[i].r) - static_cast<int>(pk_[j].r));
        const int dist = dt > dr ? dt : dr;
        if (dist <= 1) ok = false;
        else if (dist <= static_cast<int>(cfg.suppR) &&
                 valley(pk_[i], pk_[j]) * 100 >= pk_[i].amp * cfg.valleyPct) ok = false;
      }
      if (!ok) { st.rejected++; continue; }
      pk_[kept++] = pk_[i];
    }
    // Posizione sub-nodo.
    const int floor = st.thrLow / 2, win = static_cast<int>(cfg.win);
    for (unsigned i = 0; i < kept; ++i) {
      Peak &p = pk_[i];
      if (cfg.interp) {
        p.pt = static_cast<int>(p.t) * 256 + interp(p, true);
        p.pr = static_cast<int>(p.r) * 256 + interp(p, false);
      } else {
        int64_t sw = 0, st_ = 0, sr = 0;
        for (int a = -win; a <= win; ++a)
          for (int b = -win; b <= win; ++b) {
            const int t = static_cast<int>(p.t) + a, r = static_cast<int>(p.r) + b;
            int w = at(t, r) - floor;
            if (w <= 0) continue;
            sw += w; st_ += static_cast<int64_t>(w) * t * 256; sr += static_cast<int64_t>(w) * r * 256;
          }
        p.pt = sw ? static_cast<int>(st_ / sw) : static_cast<int>(p.t) * 256;
        p.pr = sw ? static_cast<int>(sr / sw) : static_cast<int>(p.r) * 256;
      }
      toPixels(p.pt, p.pr, &p.x, &p.y);
    }
    st.detections = kept;
    return kept;
  }

  // log2 in Q16 di x > 0 (tabella di log2(1 + k/32), errore < 1e-4).
  static int32_t log2q16(uint32_t x) {
    static const uint16_t kTab[33] = {0, 2909, 5732, 8473, 11136, 13727, 16248, 18704, 21098, 23433, 25711, 27936, 30109,
                                      32234, 34312, 36346, 38336, 40286, 42196, 44068, 45904, 47705, 49472, 51207, 52911,
                                      54584, 56229, 57845, 59434, 60997, 62534, 64047, 65535};
    if (!x) x = 1;
    int e = 31;
    while (!(x >> e)) --e;
    // mantissa in [1, 2) con 16 bit di frazione
    const uint32_t m = e >= 16 ? x >> (e - 16) : x << (16 - e);
    const uint32_t f = m & 0xFFFF, k = f >> 11, rem = f & 0x7FF;
    const int32_t lo = kTab[k], hi = k < 32 ? kTab[k + 1] : 65536;
    return e * 65536 + lo + static_cast<int32_t>((static_cast<int64_t>(hi - lo) * rem) >> 11);
  }
  // Profilo lungo un asse (somma sulla finestra trasversale) e posizione del massimo in 1/256 di nodo dal nodo del picco:
  // interp 1 gaussiana a 3 punti (logaritmi), 2 parabola. Un vicino fuori dal vetro o mascherato: gaussiana con la
  // larghezza cfg.sigma2 (sigma^2 in 1/256 di nodo^2) dal solo lato presente.
  int interp(const Peak &p, bool alongT) const {
    const int win = static_cast<int>(cfg.win);
    int64_t P[3] = {0, 0, 0};
    bool have[3] = {false, true, false};
    for (int a = -1; a <= 1; ++a) {
      const int t0 = static_cast<int>(p.t) + (alongT ? a : 0), r0 = static_cast<int>(p.r) + (alongT ? 0 : a);
      if (t0 < 0 || r0 < 0 || t0 >= static_cast<int>(cfg.tx) || r0 >= static_cast<int>(cfg.rx)) continue;
      if (a && mask_[t0 * static_cast<int>(cfg.rx) + r0]) continue;
      have[a + 1] = true;
      for (int b = -win; b <= win; ++b) {
        const int v = at(t0 + (alongT ? 0 : b), r0 + (alongT ? b : 0));
        if (v > 0) P[a + 1] += v;
      }
    }
    if (P[1] <= 0) return 0;
    for (int k = 0; k < 3; ++k) { if (P[k] < 1) P[k] = 1; if (P[k] > 0x7FFFFFFF) P[k] = 0x7FFFFFFF; }
    int64_t mu;   // 1/256 di nodo
    if (cfg.interp == 2) {
      if (!have[0]) P[0] = P[2];
      if (!have[2]) P[2] = P[0];
      const int64_t den = 2 * (2 * P[1] - P[0] - P[2]);
      mu = den > 0 ? (P[2] - P[0]) * 256 / den : 0;
    } else {
      const int64_t L0 = log2q16(static_cast<uint32_t>(P[0])), L1 = log2q16(static_cast<uint32_t>(P[1])),
                    L2 = log2q16(static_cast<uint32_t>(P[2]));
      if (have[0] && have[2]) {
        const int64_t den = 2 * (2 * L1 - L0 - L2);
        mu = den > 0 ? (L2 - L0) * 256 / den : 0;
      } else if (have[0] || have[2]) {
        // mu = +-(0.5 + sigma^2 ln2 (L_lato - L0)); ln2 = 45426 / 65536
        const int64_t Ls = have[2] ? L2 : L0;
        mu = 128 + (static_cast<int64_t>(cfg.sigma2) * 45426 / 65536) * (Ls - L1) / 65536;
        if (!have[2]) mu = -mu;
      } else {
        mu = 0;
      }
    }
    if (mu > 192) mu = 192;
    if (mu < -192) mu = -192;
    return static_cast<int>(mu);
  }

  int freeSlot() const {
    for (int s = 0; s < static_cast<int>(kMaxTracks); ++s) {
      bool used = false;
      for (unsigned j = 0; j < kMaxTracks; ++j) used = used || (tr_[j].state == 2 && tr_[j].slot == s);
      if (!used) return s;
    }
    return -1;
  }

  void track(unsigned np) {
    for (unsigned j = 0; j < kMaxTracks; ++j) tr_[j].matched = false;
    for (unsigned i = 0; i < np; ++i) pk_[i].used = false;
    // Abbinamento ingordo: coppia piu' vicina per prima.
    const int64_t gate2 = static_cast<int64_t>(cfg.gatePx) * cfg.gatePx;
    for (;;) {
      int64_t best = gate2 + 1;
      unsigned bi = 0, bj = 0;
      for (unsigned j = 0; j < kMaxTracks; ++j) {
        if (!tr_[j].state || tr_[j].matched) continue;
        for (unsigned i = 0; i < np; ++i) {
          if (pk_[i].used) continue;
          const int64_t dx = pk_[i].x - tr_[j].rawX, dy = pk_[i].y - tr_[j].rawY, d2 = dx * dx + dy * dy;
          if (d2 < best) { best = d2; bi = i; bj = j; }
        }
      }
      if (best > gate2) break;
      Track &t = tr_[bj];
      const Peak &p = pk_[bi];
      pk_[bi].used = true;
      t.matched = true;
      t.hits++; t.miss = 0;
      t.amp = p.amp; t.rawX = p.x; t.rawY = p.y; t.pt = p.pt; t.pr = p.pr;
      if (t.state == 2) {
        const int dx = p.x - t.x, dy = p.y - t.y;
        const int k = (iabs(dx) > cfg.fastPx || iabs(dy) > cfg.fastPx) ? 16 : cfg.smooth;
        t.x += dx * k / 16; t.y += dy * k / 16;
        if (iabs(p.x - t.ax) > cfg.stuckPx || iabs(p.y - t.ay) > cfg.stuckPx) { t.ax = p.x; t.ay = p.y; t.still = 0; }
        else t.still++;
      } else {
        t.x = p.x; t.y = p.y;
      }
    }
    // Nuovi contatti: solo sopra la soglia di tocco.
    for (unsigned i = 0; i < np; ++i) {
      if (pk_[i].used || pk_[i].amp < st.thrHigh) continue;
      for (unsigned j = 0; j < kMaxTracks; ++j) {
        if (tr_[j].state) continue;
        Track &t = tr_[j];
        t = Track();
        t.state = 1; t.slot = -1; t.hits = 1; t.matched = true;
        t.x = t.rawX = t.ax = pk_[i].x; t.y = t.rawY = t.ay = pk_[i].y; t.amp = pk_[i].amp; t.pt = pk_[i].pt; t.pr = pk_[i].pr;
        break;
      }
    }
    // Passaggi di stato.
    unsigned down = 0;
    for (unsigned j = 0; j < kMaxTracks; ++j) down += tr_[j].state == 2 ? 1 : 0;
    for (unsigned j = 0; j < kMaxTracks; ++j) {
      Track &t = tr_[j];
      if (!t.state) continue;
      if (!t.matched) {
        if (t.state == 1 || ++t.miss >= cfg.upFrames) {
          if (t.state == 2) { st.released++; down--; }
          t.state = 0;
        }
        continue;
      }
      if (t.state == 1 && t.hits >= cfg.downFrames && t.amp >= st.thrHigh && down < cfg.maxContacts) {
        t.slot = freeSlot();
        if (t.slot < 0) continue;
        t.state = 2; t.still = 0; t.ax = t.rawX; t.ay = t.rawY;
        down++;
        st.started++;
      }
      if (t.state == 2 && cfg.stuckFrames && t.still >= cfg.stuckFrames) {
        absorb(t.pt, t.pr);
        t.state = 0;
        st.stuck++; st.released++; down--;
      }
    }
  }

  // Residuo fermo: la baseline prende il valore attuale attorno al punto.
  void absorb(int pt, int pr) {
    const int t0 = (pt + 128) / 256, r0 = (pr + 128) / 256;
    for (int a = -2; a <= 2; ++a)
      for (int b = -2; b <= 2; ++b) {
        const int t = t0 + a, r = r0 + b;
        if (t < 0 || r < 0 || t >= static_cast<int>(cfg.tx) || r >= static_cast<int>(cfg.rx)) continue;
        absorbAt_[absorbN_ < kAbsorb ? absorbN_++ : kAbsorb - 1] = static_cast<uint16_t>(t * static_cast<int>(cfg.rx) + r);
      }
  }
  static constexpr unsigned kAbsorb = 128;
  uint16_t absorbAt_[kAbsorb];
  unsigned absorbN_;

  void updateBaseline(const int32_t *v) {
    const unsigned rx = cfg.rx, tx = cfg.tx, n = rx * tx;
    for (unsigned i = 0; i < n; ++i) frozen_[i] = 0;
    // Congelati: 2 nodi attorno a ogni massimo sopra la soglia di mantenimento e a ogni contatto.
    auto freeze = [&](int t0, int r0) {
      for (int a = -2; a <= 2; ++a)
        for (int b = -2; b <= 2; ++b) {
          const int t = t0 + a, r = r0 + b;
          if (t >= 0 && r >= 0 && t < static_cast<int>(tx) && r < static_cast<int>(rx)) frozen_[t * static_cast<int>(rx) + r] = 1;
        }
    };
    for (unsigned i = 0; i < st.detections; ++i) freeze(static_cast<int>(pk_[i].t), static_cast<int>(pk_[i].r));
    for (unsigned j = 0; j < kMaxTracks; ++j)
      if (tr_[j].state) freeze((tr_[j].pt + 128) / 256, (tr_[j].pr + 128) / 256);
    for (unsigned k = 0; k < absorbN_; ++k) {
      const unsigned i = absorbAt_[k];
      if (i < n && !mask_[i]) { base16_[i] = v[i] * 16; frozen_[i] = 2; }
    }
    absorbN_ = 0;
    for (unsigned i = 0; i < n; ++i) {
      if (mask_[i] || frozen_[i] || v[i] >= cfg.satLevel || v[i] <= -cfg.satLevel) continue;
      const int d = d_[i];                 // nel verso del tocco, modo comune tolto
      unsigned sh;
      if (d <= -st.thrLow) sh = cfg.fastShift;       // verso opposto al tocco: la baseline era sbagliata
      else if (d < st.thrLow) sh = cfg.slowShift;
      else continue;                                 // nodo acceso isolato: non lo impara
      base16_[i] += (v[i] * 16 - base16_[i]) / (1 << sh);   // divisione: niente deriva verso il basso
    }
  }
};
}  // namespace guamp_softtouch
#endif
