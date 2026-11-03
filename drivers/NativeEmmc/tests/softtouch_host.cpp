// Prova sul PC di GuampSoftTouchCore.h (piano B del touch): mappe sintetiche costruite sulle mappe TWRP a riposo
// (log-xnu/twrp-raw-{bank,dc,iir}-riposo.txt) con rumore, modo comune, dita gaussiane, riga TX0 satura, e le mappe
// TWRP "dito" cosi' come sono. Uso: softtouch_host <cartella log-xnu>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <initializer_list>
#include "../GuampSoftTouchCore.h"
using namespace guamp_softtouch;

static int failures;
static void check(bool ok, const char *what) {
  printf("%s %s\n", ok ? "ok  " : "FAIL", what);
  if (!ok) failures++;
}

constexpr unsigned RX = 32, TX = 18, N = RX * TX;
// Mappe nel formato di /proc/android_touch/diag: "ChannelStart", righe "[tt] v0 .. v31 self", "ChannelEnd".
static unsigned readFrames(const char *path, int32_t (*out)[N], unsigned max, unsigned *filled) {
  FILE *f = fopen(path, "r");
  if (!f) return 0;
  char line[4096];
  unsigned nf = 0;
  int in = 0, t = 0;
  while (fgets(line, sizeof line, f)) {
    if (!strncmp(line, "ChannelStart", 12)) { in = 1; t = 0; if (nf < max) memset(out[nf], 0, sizeof out[nf]); filled[nf < max ? nf : 0] = 0; continue; }
    if (!strncmp(line, "ChannelEnd", 10)) { in = 0; if (nf < max) nf++; continue; }
    if (!in) continue;
    char *p = strchr(line, '[');
    if (!p) continue;
    int idx = atoi(p + 1);
    if (idx == 0) continue;                       // intestazione delle colonne
    p = strchr(p, ']') + 1;
    for (unsigned r = 0; r < RX; ++r) {
      char *e;
      long v = strtol(p, &e, 10);
      if (e == p) break;
      p = e;
      if (nf < max && t < static_cast<int>(TX)) { out[nf][t * RX + r] = static_cast<int32_t>(v); if (v) filled[nf]++; }
    }
    t++;
  }
  fclose(f);
  return nf;
}

static uint64_t rng = 88172645463325252ull;
static double urand() { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (rng >> 11) * (1.0 / 9007199254740992.0); }
static double grand() { double u = urand() + 1e-12, v = urand(); return sqrt(-2 * log(u)) * cos(6.283185307179586 * v); }

struct Finger { double x, y, amp; };
// Forma del dito: gaussiana di sigma gSigma nodi (gShape 0) o coseno rialzato di raggio gRadius nodi (gShape 1).
static double gSigma = 0.85, gRadius = 2.0;
static int gShape = 0;
// Posizione in pixel -> nodo (inversa della mappatura edge = 1, assi di default): t = x * 17 / 719, r = y * 31 / 1599.
static void synth(const int32_t *rest, int32_t *out, const Finger *f, unsigned nf, double noise, double cmRow, int sign = 1,
                  bool satTx0 = false) {
  double rowCm[TX];
  for (unsigned t = 0; t < TX; ++t) rowCm[t] = cmRow * grand();
  for (unsigned t = 0; t < TX; ++t)
    for (unsigned r = 0; r < RX; ++r) {
      double v = rest[t * RX + r] + noise * grand() + rowCm[t];
      for (unsigned k = 0; k < nf; ++k) {
        const double ft = f[k].x * (TX - 1) / 719.0, fr = f[k].y * (RX - 1) / 1599.0;
        const double dt = t - ft, dr = r - fr, s = gSigma, q = dt * dt + dr * dr;
        if (gShape == 0) v += sign * f[k].amp * exp(-q / (2 * s * s));
        else if (q < gRadius * gRadius) v += sign * f[k].amp * 0.5 * (1 + cos(3.141592653589793 * sqrt(q) / gRadius));
      }
      if (satTx0 && t == 0) v = 32767;
      out[t * RX + r] = static_cast<int32_t>(lround(v));
    }
}

static Detector det;
static Contact cs[kMaxTracks];
static unsigned learn(const int32_t *rest, double noise, double cm, bool sat = false) {
  int32_t m[N];
  for (unsigned i = 0; i < det.cfg.learn; ++i) { synth(rest, m, nullptr, 0, noise, cm, 1, sat); det.frame(m, cs); }
  return det.st.masked;
}

int main(int argc, char **argv) {
  const char *dir = argc > 1 ? argv[1] : "/mnt/c/Users/Chris/Desktop/Moto/log-xnu";
  static int32_t bank[4][N], dc[4][N], iir[4][N], dito[32][N];
  unsigned fb[4], fd[4], fi[4], fdito[32];
  char p[512];
  snprintf(p, sizeof p, "%s/twrp-raw-bank-riposo.txt", dir);
  const unsigned nb = readFrames(p, bank, 4, fb);
  snprintf(p, sizeof p, "%s/twrp-raw-dc-riposo.txt", dir);
  const unsigned nd = readFrames(p, dc, 4, fd);
  snprintf(p, sizeof p, "%s/twrp-raw-iir-riposo.txt", dir);
  const unsigned ni = readFrames(p, iir, 4, fi);
  snprintf(p, sizeof p, "%s/twrp-raw-dito.txt", dir);
  const unsigned ndito = readFrames(p, dito, 32, fdito);
  check(nb == 1 && nd == 1 && ni == 1 && fb[0] == N && fd[0] == N, "mappe TWRP a riposo lette (bank, dc: 576 nodi; iir)");
  int mn = 1 << 30, mx = -(1 << 30);
  double sd = 0;
  for (unsigned i = 0; i < N; ++i) {
    mn = bank[0][i] < mn ? bank[0][i] : mn; mx = bank[0][i] > mx ? bank[0][i] : mx;
    sd += (dc[0][i] - bank[0][i]) * (double)(dc[0][i] - bank[0][i]);
  }
  sd = sqrt(sd / N / 2);   // due misure indipendenti: sigma di una mappa
  printf("     bank a riposo %d..%d, rumore per mappa stimato da dc-bank %.1f conteggi\n", mn, mx, sd);
  const double noise = sd > 4 ? sd : 11;

  Config c = defaultConfig();
  // --- 1. riposo: nessun contatto (raw DC + rumore reale + modo comune per riga)
  det.configure(c);
  learn(dc[0], noise, 20);
  int32_t m[N];
  unsigned ghosts = 0;
  for (unsigned k = 0; k < 600; ++k) { synth(dc[0], m, nullptr, 0, noise, 20); ghosts += det.frame(m, cs); }
  printf("     riposo: rumore %d soglie %d/%d, mascherati %u\n", det.st.noise, det.st.thrHigh, det.st.thrLow, det.st.masked);
  check(ghosts == 0 && det.st.started == 0, "riposo 600 mappe (10 s): nessun fantasma");

  // --- 2. dito al centro dello schermo (cella 8.5, 15.5), ampiezza come un dito leggero (600) e normale (2500)
  for (double amp : {600.0, 2500.0}) {
    det.configure(c);
    learn(dc[0], noise, 20);
    Finger f{360, 800, amp};
    unsigned got = 0;
    int x = -1, y = -1;
    for (unsigned k = 0; k < 30; ++k) { synth(dc[0], m, &f, 1, noise, 20); got = det.frame(m, cs); if (got) { x = cs[0].x; y = cs[0].y; } }
    char what[128];
    snprintf(what, sizeof what, "dito al centro ampiezza %.0f -> %d,%d (atteso 360,800 +-12)", amp, x, y);
    check(got == 1 && abs(x - 360) <= 12 && abs(y - 800) <= 12, what);
  }

  // --- 3. griglia di posizioni: errore medio e massimo (bordi inclusi), forme diverse del dito e i tre interpolatori
  {
    struct Shape { int shape; double sigma, radius; const char *name; double inMax; };
    const Shape shapes[] = {{0, 0.85, 0, "gauss 0.85", 3}, {0, 0.6, 0, "gauss 0.60", 10}, {0, 1.2, 0, "gauss 1.20", 10},
                            {1, 0, 2.0, "coseno r2.0", 10}, {1, 0, 2.8, "coseno r2.8", 10}};
    for (const Shape &sh : shapes)
      for (int ip : {1, 2, 0}) {
        gShape = sh.shape; gSigma = sh.sigma; gRadius = sh.radius;
        Config c2 = c;
        c2.interp = ip;
        double sum = 0, worst = 0, sumIn = 0;
        unsigned cnt = 0, cntIn = 0, miss = 0;
        for (int yy = 0; yy <= 1599; yy += 41)
          for (int xx = 0; xx <= 719; xx += 23) {
            det.configure(c2);
            learn(dc[0], noise, 10);
            Finger f{static_cast<double>(xx), static_cast<double>(yy), 1500};
            unsigned got = 0;
            for (unsigned k = 0; k < 6; ++k) { synth(dc[0], m, &f, 1, noise, 10); got = det.frame(m, cs); }
            if (got != 1) { miss++; continue; }
            const double e = hypot(cs[0].x - xx, cs[0].y - yy);
            sum += e; cnt++;
            if (e > worst) worst = e;
            if (xx >= 40 && xx <= 680 && yy >= 50 && yy <= 1550) { sumIn += e; cntIn++; }
          }
        printf("     griglia %-11s interp %d: %u punti, errore medio %.1f px (interno %.1f), massimo %.1f, mancati %u\n", sh.name,
               ip, cnt + miss, cnt ? sum / cnt : 0, cntIn ? sumIn / cntIn : 0, worst, miss);
        if (ip == 1) {
          char what[160];
          snprintf(what, sizeof what, "griglia %s (gaussiana): interno < %.0f px di media, bordi < 45 px, nessun mancato", sh.name,
                   sh.inMax);
          check(miss == 0 && cntIn && sumIn / cntIn < sh.inMax && worst < 45, what);
        }
      }
    gShape = 0; gSigma = 0.85;
  }

  // --- 4. cinque dita, ID stabili durante uno spostamento, rilascio
  {
    det.configure(c);
    learn(dc[0], noise, 20);
    Finger f[5] = {{120, 300, 1500}, {600, 300, 1500}, {360, 800, 1500}, {120, 1300, 1500}, {600, 1300, 1500}};
    int slotAt[5] = {-1, -1, -1, -1, -1};
    bool stable = true;
    unsigned got = 0;
    for (unsigned k = 0; k < 60; ++k) {
      for (auto &q : f) { q.y += 4; }
      synth(dc[0], m, f, 5, noise, 20);
      got = det.frame(m, cs);
      if (got != 5) continue;
      for (unsigned i = 0; i < 5; ++i) {
        unsigned best = 0;
        for (unsigned j = 1; j < 5; ++j)
          if (hypot(cs[j].x - f[i].x, cs[j].y - f[i].y) < hypot(cs[best].x - f[i].x, cs[best].y - f[i].y)) best = j;
        if (slotAt[i] < 0) slotAt[i] = cs[best].slot;
        else if (slotAt[i] != cs[best].slot) stable = false;
      }
    }
    check(got == 5 && stable, "cinque dita in movimento: 5 contatti, ID stabili per 60 mappe");
    for (unsigned k = 0; k < 4; ++k) { synth(dc[0], m, nullptr, 0, noise, 20); got = det.frame(m, cs); }
    check(got == 0 && det.st.released == 5, "rilascio di tutte e cinque entro 4 mappe");
    Finger six[6] = {{100, 200, 1500}, {600, 200, 1500}, {100, 800, 1500}, {600, 800, 1500}, {100, 1400, 1500}, {600, 1400, 1500}};
    for (unsigned k = 0; k < 6; ++k) { synth(dc[0], m, six, 6, noise, 20); got = det.frame(m, cs); }
    check(got == 5, "sei dita: al massimo 5 contatti");
  }

  // --- 5. due dita vicine (3 nodi = 120 px) restano due; un dito largo resta uno
  {
    det.configure(c);
    learn(dc[0], noise, 20);
    Finger f[2] = {{240, 800, 1500}, {360, 800, 1500}};
    unsigned got = 0;
    for (unsigned k = 0; k < 6; ++k) { synth(dc[0], m, f, 2, noise, 20); got = det.frame(m, cs); }
    check(got == 2, "due dita a 120 px: due contatti");
    det.configure(c);
    learn(dc[0], noise, 20);
    Finger wide[3] = {{350, 800, 1500}, {370, 820, 1500}, {360, 780, 1500}};
    for (unsigned k = 0; k < 6; ++k) { synth(dc[0], m, wide, 3, noise, 20); got = det.frame(m, cs); }
    check(got == 1, "dito largo (pollice): un contatto");
  }

  // --- 6. riga TX0 satura: mascherata, niente fantasmi, il dito si trova
  {
    det.configure(c);
    learn(dc[0], noise, 20, true);
    bool tx0 = true;
    for (unsigned r = 0; r < RX; ++r) tx0 = tx0 && det.masked(0, r);
    unsigned g = 0;
    for (unsigned k = 0; k < 300; ++k) { synth(dc[0], m, nullptr, 0, noise, 20, 1, true); g += det.frame(m, cs); }
    Finger f{360, 800, 1500};
    unsigned got = 0;
    for (unsigned k = 0; k < 6; ++k) { synth(dc[0], m, &f, 1, noise, 20, 1, true); got = det.frame(m, cs); }
    printf("     TX0 satura: mascherati %u nodi\n", det.st.masked);
    check(tx0 && g == 0 && got == 1 && abs(cs[0].x - 360) <= 12, "TX0 satura: riga mascherata, nessun fantasma, dito trovato");
  }

  // --- 7. sorgente delta (IIR TWRP a riposo = 0) con rumore
  {
    det.configure(c);
    learn(iir[0], noise / 2, 5);
    Finger f{200, 1200, 800};
    unsigned g = 0, got = 0;
    for (unsigned k = 0; k < 300; ++k) { synth(iir[0], m, nullptr, 0, noise / 2, 5); g += det.frame(m, cs); }
    for (unsigned k = 0; k < 6; ++k) { synth(iir[0], m, &f, 1, noise / 2, 5); got = det.frame(m, cs); }
    char what[128];
    snprintf(what, sizeof what, "delta IIR: nessun fantasma, dito 200,1200 -> %d,%d", got ? cs[0].x : -1, got ? cs[0].y : -1);
    check(g == 0 && got == 1 && abs(cs[0].x - 200) <= 12 && abs(cs[0].y - 1200) <= 12, what);
  }

  // --- 8. verso negativo (sign = -1) e automatico (sign = 0)
  for (int sg : {-1, 0}) {
    Config c2 = c;
    c2.sign = sg;
    det.configure(c2);
    learn(dc[0], noise, 20);
    Finger f{500, 400, 1500};
    unsigned got = 0, g = 0;
    for (unsigned k = 0; k < 120; ++k) { synth(dc[0], m, nullptr, 0, noise, 20); g += det.frame(m, cs); }
    for (unsigned k = 0; k < 6; ++k) { synth(dc[0], m, &f, 1, noise, 20, -1); got = det.frame(m, cs); }
    char what[128];
    snprintf(what, sizeof what, "verso %d con dito che abbassa il raw: %u contatti %d,%d, fantasmi %u", sg, got, got ? cs[0].x : -1,
             got ? cs[0].y : -1, g);
    check(g == 0 && got == 1 && abs(cs[0].x - 500) <= 12 && abs(cs[0].y - 400) <= 12, what);
  }

  // --- 9. tap breve (3 mappe = 50 ms) e strisciata
  {
    det.configure(c);
    learn(dc[0], noise, 20);
    Finger f{360, 1000, 1200};
    unsigned seen = 0;
    for (unsigned k = 0; k < 3; ++k) { synth(dc[0], m, &f, 1, noise, 20); seen += det.frame(m, cs) ? 1 : 0; }
    for (unsigned k = 0; k < 4; ++k) { synth(dc[0], m, nullptr, 0, noise, 20); det.frame(m, cs); }
    check(seen == 2 && det.st.started == 1 && det.st.released == 1, "tap di 3 mappe: un contatto (premuto 2 mappe), rilasciato");
    Finger s{360, 1400, 1500};
    int slot = -1;
    bool ok = true;
    unsigned frames = 0;
    for (; s.y > 100; s.y -= 40) {   // 40 px a mappa = 2,4 m/s a 60 Hz
      synth(dc[0], m, &s, 1, noise, 20);
      if (det.frame(m, cs) == 1) {
        if (slot < 0) slot = cs[0].slot;
        ok = ok && cs[0].slot == slot && fabs(cs[0].y - s.y) < 60;
        frames++;
      }
    }
    check(ok && frames > 25 && det.st.started == 2, "strisciata verso l'alto a 40 px/mappa: un solo contatto, sempre vicino al dito");
  }

  // --- 10. dito appoggiato durante l'apprendimento: dopo il distacco niente contatto e la baseline si riprende
  {
    det.configure(c);
    Finger f{360, 800, 2000};
    for (unsigned k = 0; k < det.cfg.learn; ++k) { synth(dc[0], m, &f, 1, noise, 20); det.frame(m, cs); }
    unsigned g = 0;
    for (unsigned k = 0; k < 120; ++k) { synth(dc[0], m, nullptr, 0, noise, 20); g += det.frame(m, cs); }
    Finger f2{360, 800, 1500};
    unsigned got = 0;
    for (unsigned k = 0; k < 6; ++k) { synth(dc[0], m, &f2, 1, noise, 20); got = det.frame(m, cs); }
    check(g == 0 && got == 1, "dito presente in apprendimento: nessun fantasma dopo, e il dito di nuovo trovato");
  }

  // --- 11. residuo fermo: assorbito dopo stuckFrames
  {
    Config c2 = c;
    c2.stuckFrames = 120;
    det.configure(c2);
    learn(dc[0], noise, 20);
    Finger f{360, 800, 1500};
    unsigned got = 0;
    for (unsigned k = 0; k < 200; ++k) { synth(dc[0], m, &f, 1, noise, 20); got = det.frame(m, cs); }
    printf("     fermo: got %u stuck %u started %u released %u\n", got, det.st.stuck, det.st.started, det.st.released);
    check(got == 0 && det.st.stuck == 1, "contatto fermo 2 s (stuckFrames 120): assorbito e rilasciato");
  }

  // --- 12. deriva lenta di tutta la mappa (+300 in 20 s): nessun fantasma
  {
    det.configure(c);
    learn(dc[0], noise, 20);
    unsigned g = 0;
    int32_t drift[N];
    for (unsigned k = 0; k < 1200; ++k) {
      for (unsigned i = 0; i < N; ++i) drift[i] = dc[0][i] + static_cast<int32_t>(k / 4) + static_cast<int32_t>((i % RX) * k / 400);
      synth(drift, m, nullptr, 0, noise, 20);
      g += det.frame(m, cs);
    }
    check(g == 0, "deriva +300 uniforme e a rampa sulle colonne in 1200 mappe: nessun fantasma");
  }

  // --- 13. scambio e specchio degli assi
  {
    int x, y;
    Config c2 = c;
    det.configure(c2);
    det.toPixels(0, 0, &x, &y);
    const bool a = x == 0 && y == 0;
    det.toPixels(17 * 256, 31 * 256, &x, &y);
    const bool b = x == 719 && y == 1599;
    det.toPixels(8 * 256 + 128, 15 * 256 + 128, &x, &y);
    const bool cc = x == 360 && y == 800;
    c2.axes = 6;
    det.configure(c2);
    det.toPixels(0, 0, &x, &y);
    const bool d = x == 719 && y == 1599;
    c2.axes = 0; c2.edge = 0;
    det.configure(c2);
    det.toPixels(0, 0, &x, &y);
    const bool e = x == 20 && y == 25;
    check(a && b && cc && d && e, "mappatura: nodo 0 -> 0,0; 17,31 -> 719,1599; 8.5,15.5 -> 360,800; specchi; edge 0 -> 20,25");
  }

  // --- 14. mappe TWRP "dito" come sono (solo informazione: il file ha 3 righe TX su 18 e mappe vuote alternate)
  {
    printf("     twrp-raw-dito.txt: %u mappe, nodi diversi da 0 per mappa:", ndito);
    for (unsigned k = 0; k < ndito; ++k) printf(" %u", fdito[k]);
    printf("\n");
    det.configure(c);
    for (unsigned k = 0; k < det.cfg.learn; ++k) { synth(dc[0], m, nullptr, 0, 0, 0); det.frame(m, cs); }
    for (unsigned k = 0; k < ndito; ++k) {
      if (!fdito[k]) continue;
      for (unsigned i = 0; i < N; ++i) m[i] = i < 3 * RX ? dito[k][i] : dc[0][i];
      const unsigned got = det.frame(m, cs);
      printf("     dito mappa %u: d max %d @t%u,r%u, %u massimi, %u contatti", k, det.st.maxPos, det.st.maxAt / RX, det.st.maxAt % RX,
             det.st.detections, got);
      for (unsigned j = 0; j < got; ++j) printf(" [%d: %d,%d amp %d]", cs[j].slot, cs[j].x, cs[j].y, cs[j].amp);
      printf("\n");
    }
  }

  // --- 15. costo di calcolo per mappa (host)
  {
    det.configure(c);
    learn(dc[0], noise, 20);
    static int32_t pre[64][N];
    Finger f[2] = {{200, 500, 1500}, {500, 1100, 1500}};
    for (unsigned k = 0; k < 64; ++k) synth(dc[0], pre[k], f, 2, noise, 20);
    timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    const unsigned reps = 20000;
    for (unsigned k = 0; k < reps; ++k) det.frame(pre[k & 63], cs);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    const double us = ((t1.tv_sec - t0.tv_sec) * 1e9 + (t1.tv_nsec - t0.tv_nsec)) / 1e3 / reps;
    printf("     calcolo: %.1f us per mappa sul PC (2 dita)\n", us);
    check(us < 200, "calcolo per mappa < 200 us sul PC");
  }
  printf("%s: %d errori\n", failures ? "FALLITO" : "PASS", failures);
  return failures ? 1 : 0;
}
