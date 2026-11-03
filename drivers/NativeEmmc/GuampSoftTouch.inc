// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp 04/11, piano B del touch (incluso da TouchService.inc dentro namespace touch, dopo rawdump): tocchi calcolati da
// noi dalla mappa mutual 32x18 del sensore, senza le coordinate del firmware (che sotto XNU manda fantasmi sulla riga TX0).
// Algoritmo in GuampSoftTouchCore.h (provato sul PC: tests/softtouch_host.cpp, tests/run-softtouch-tests.sh); qui la
// lettura della mappa dalla DSRAM e la pubblicazione con publish(), lo stesso percorso HID dei report del firmware.
//
// guamp-touch-softdetect=<m> (default 0 = spento): 1 i contatti calcolati sostituiscono i report del firmware (le
// interruzioni si leggono ancora, ma non si pubblicano), 2 solo log (ombra: si pubblica ancora il firmware).
// Lettura: raw out select = tipo | 8 una volta sola (resta acceso), poi per ogni mappa attesa di A55A a 0x10000000 a passi di
// 1 ms (al massimo -wait ms), (rx*tx + rx + tx) * 2 + 4 = 1256 byte a pezzi di -chunk byte, chiusura, somma a 16 bit, e
// subito la richiesta 5AA5 della mappa seguente (pronta quando torniamo: niente attesa nel caso normale).
// Boot-arg (guamp-touch-softdetect-<nome>, default tra parentesi):
//   src (1): 1 delta IIR del firmware, 2 raw DC; in entrambi i casi baseline propria sopra (offset del firmware tolto)
//   hz (0 = automatico: parte da 60, sale fino a 120 se la lettura costa poco e la mappa e' gia' pronta, scende fino a 30
//      se il bus SPI supera il 25% del tempo o la mappa non e' pronta); 10..120 = fisso
//   thr (80) soglia minima, k (24) soglia = k/4 * rumore, rel (55) soglia di mantenimento in %, sign (1 = il tocco alza il
//   valore, 2 = lo abbassa, 0 = ogni mappa il verso del massimo piu' grande), txmask (0) / rxmask (0) canali ignorati
//   (bit n = canale n; es. txmask=1 toglie la riga TX0), automask (1) maschera i nodi saturi/rumorosi in apprendimento,
//   cm (1) modo comune (1 righe TX, 2 colonne RX, 3 entrambe), learn (16) mappe di baseline, axes (0): bit 0 scambia
//   (x dagli RX), bit 1 specchio x, bit 2 specchio y; edge (1), max (5) contatti, down (2) / up (2) mappe per premere /
//   rilasciare, gate (220) px, smooth (8)/16, stuck (10) secondi di contatto fermo prima di assorbirlo (0 = mai), interp
//   (1 gaussiana, 2 parabola, 0 baricentro), win (1), chunk (128 come Linux, 256 = HX_READ_CHUNK), wait (30) ms,
//   log (10) secondi tra due righe "soft stats" (0 = mai), maplog (4) mappe differenza registrate ai primi inizi di contatto.
// Righe: "GuampTouch: soft ..." (avvio, baseline, contatti giu'/su, statistiche con i tempi misurati, mappe "sd<n> t<TX>").
namespace softdetect {
using guamp_softtouch::Contact;
using guamp_softtouch::Detector;
constexpr unsigned kCells = guamp_softtouch::kMaxNodes;
Detector gDet;
static_assert(__is_trivially_constructible(Detector), "oggetto globale senza costruttore statico");
uint8_t gBuf[HX_RAW_BYTES(32u, 32u)];
int32_t gMap[kCells];
unsigned gMode, gSrc = 1, gHzArg, gHz = 60, gChunk = 128, gWaitMax = 30, gLogS = 10, gMapLog = 4;
bool gArmed, gSelected, gLearned, gSlow;
unsigned gFailRun, gSeq, gLogged, gPrevCount;
int gPrevSlots[guamp_softtouch::kMaxTracks];
uint64_t gNext, gPeriod, gLogNext, gLastOk;
struct Acc {
  unsigned frames, waited, errs, timeouts, sums, bus;
  uint64_t waitUs, waitMaxUs, readUs, readMaxUs, calcUs, calcMaxUs, startT;
};
Acc gAcc, gWin;   // dall'avvio e dall'ultima riga di statistiche / dall'ultimo adattamento della frequenza

uint64_t usToTicks(uint64_t us) { uint64_t t = 0; nanoseconds_to_absolutetime(us * 1000, &t); return t; }
uint64_t ticksToUs(uint64_t t) { uint64_t ns = 0; absolutetime_to_nanoseconds(t, &ns); return ns / 1000; }
bool active() { return gMode != 0; }
void setHz(unsigned hz) { gHz = hz; gPeriod = usToTicks(1000000u / hz); }

void row(unsigned seq, unsigned t, const int32_t *v, unsigned n) {
  char line[640];
  unsigned len = 0;
  line[0] = 0;
  for (unsigned i = 0; i < n; ++i) {
    const int w = snprintf(line + len, sizeof(line) - len, " %d", v[i]);
    if (w < 0 || static_cast<unsigned>(w) >= sizeof(line) - len) break;
    len += static_cast<unsigned>(w);
  }
  IOLog("GuampTouch: sd%u t%02u:%s\n", seq, t, line);
}

bool selectMap() {
  const uint8_t sel = static_cast<uint8_t>(gSrc | HX_RAW_DSRAM);
  const uint32_t back = hx_diag_select(&gChip, sel);
  gSelected = (back & 0xFF) == sel;
  gArmed = false;
  IOLog("GuampTouch: soft raw out select %02x (riletto %08x) %s\n", sel, back, gSelected ? "ok" : "NON preso");
  return gSelected;
}

// Una mappa dalla DSRAM in gMap. *waitUs: attesa del dato pronto, *readUs: trasferimento + controllo.
int readFrame(unsigned rx, unsigned tx, uint64_t *waitUs, uint64_t *readUs) {
  const size_t total = HX_RAW_BYTES(rx, tx);
  uint32_t v = 0;
  const uint64_t t0 = mach_absolute_time();
  *waitUs = *readUs = 0;
  if (!gArmed && hx_reg_write32(&gChip, HX_ADDR_RAWDATA, HX_RAW_PWD_START)) return HX_ERR_BUS;
  gArmed = false;
  for (unsigned i = 0;; ++i) {
    if (hx_reg_read32(&gChip, HX_ADDR_RAWDATA, &v)) return HX_ERR_BUS;
    if ((v & 0xFFFF) == HX_RAW_PWD_END) break;
    if (i >= gWaitMax) { *waitUs = ticksToUs(mach_absolute_time() - t0); return HX_ERR_TIMEOUT; }
    IOSleep(1);
    if (i % 10 == 9) hx_reg_write32(&gChip, HX_ADDR_RAWDATA, HX_RAW_PWD_START);   // come himax_write_read_reg
  }
  const uint64_t t1 = mach_absolute_time();
  *waitUs = ticksToUs(t1 - t0);
  for (size_t done = 0; done < total;) {
    const size_t n = total - done > gChunk ? gChunk : total - done;
    if (hx_reg_read(&gChip, HX_ADDR_RAWDATA + static_cast<uint32_t>(done), gBuf + done, n)) return HX_ERR_BUS;
    done += n;
  }
  hx_reg_write32(&gChip, HX_ADDR_RAWDATA, static_cast<uint32_t>(gBuf[2]) << 16 | static_cast<uint32_t>(gBuf[3]) << 24);
  unsigned sum = 0;
  for (size_t k = 2; k < total; k += 2) sum += static_cast<unsigned>(gBuf[k + 1]) << 8 | gBuf[k];
  // Subito la richiesta della mappa seguente.
  gArmed = !hx_reg_write32(&gChip, HX_ADDR_RAWDATA, HX_RAW_PWD_START);
  *readUs = ticksToUs(mach_absolute_time() - t1);
  if (sum & 0xFFFF) return HX_ERR_CHECKSUM;
  const uint8_t *p = gBuf + 4;
  for (unsigned k = 0; k < rx * tx; ++k) gMap[k] = static_cast<int16_t>(static_cast<uint16_t>(p[2 * k] | p[2 * k + 1] << 8));
  return HX_OK;
}

void configure() {
  guamp_softtouch::Config c = guamp_softtouch::defaultConfig();
  unsigned v;
  c.rx = gChip.rx_num; c.tx = gChip.tx_num;
  c.width = gChip.x_res ? gChip.x_res : GT_PANEL_X;
  c.height = gChip.y_res ? gChip.y_res : GT_PANEL_Y;
  if (PE_parse_boot_argn("guamp-touch-softdetect-thr", &v, sizeof(v)) && v && v < 30000) c.thrMin = static_cast<int>(v);
  if (PE_parse_boot_argn("guamp-touch-softdetect-k", &v, sizeof(v)) && v <= 200) c.noiseK = static_cast<int>(v);
  if (PE_parse_boot_argn("guamp-touch-softdetect-rel", &v, sizeof(v))) c.relPct = static_cast<int>(v);
  if (PE_parse_boot_argn("guamp-touch-softdetect-sign", &v, sizeof(v))) c.sign = v == 0 ? 0 : v == 2 ? -1 : 1;
  if (PE_parse_boot_argn("guamp-touch-softdetect-txmask", &v, sizeof(v))) c.txMask = v;
  if (PE_parse_boot_argn("guamp-touch-softdetect-rxmask", &v, sizeof(v))) c.rxMask = v;
  if (PE_parse_boot_argn("guamp-touch-softdetect-automask", &v, sizeof(v))) c.autoMask = v != 0;
  if (PE_parse_boot_argn("guamp-touch-softdetect-cm", &v, sizeof(v))) c.cm = static_cast<int>(v & 3);
  if (PE_parse_boot_argn("guamp-touch-softdetect-learn", &v, sizeof(v)) && v && v <= 256) c.learn = v;
  if (PE_parse_boot_argn("guamp-touch-softdetect-axes", &v, sizeof(v))) c.axes = static_cast<int>(v & 7);
  if (PE_parse_boot_argn("guamp-touch-softdetect-edge", &v, sizeof(v))) c.edge = v != 0;
  if (PE_parse_boot_argn("guamp-touch-softdetect-max", &v, sizeof(v)) && v) c.maxContacts = v;
  if (PE_parse_boot_argn("guamp-touch-softdetect-down", &v, sizeof(v)) && v && v <= 30) c.downFrames = v;
  if (PE_parse_boot_argn("guamp-touch-softdetect-up", &v, sizeof(v)) && v && v <= 30) c.upFrames = v;
  if (PE_parse_boot_argn("guamp-touch-softdetect-gate", &v, sizeof(v)) && v) c.gatePx = static_cast<int>(v);
  if (PE_parse_boot_argn("guamp-touch-softdetect-smooth", &v, sizeof(v))) c.smooth = static_cast<int>(v);
  if (PE_parse_boot_argn("guamp-touch-softdetect-interp", &v, sizeof(v)) && v <= 2) c.interp = static_cast<int>(v);
  if (PE_parse_boot_argn("guamp-touch-softdetect-win", &v, sizeof(v))) c.win = v;
  unsigned stuckS = 10;
  PE_parse_boot_argn("guamp-touch-softdetect-stuck", &stuckS, sizeof(stuckS));
  // Le mappe del contatto fermo seguono la frequenza scelta all'avvio (60 Hz se automatica).
  c.stuckFrames = stuckS * (gHzArg ? gHzArg : 60);
  gDet.configure(c);
}

void init() {
  PE_parse_boot_argn("guamp-touch-softdetect", &gMode, sizeof(gMode));
  if (gMode > 2) gMode = 0;
  if (!gMode) return;
  PE_parse_boot_argn("guamp-touch-softdetect-src", &gSrc, sizeof(gSrc));
  if (gSrc < 1 || gSrc > 2) gSrc = 1;
  PE_parse_boot_argn("guamp-touch-softdetect-hz", &gHzArg, sizeof(gHzArg));
  if (gHzArg && gHzArg < 10) gHzArg = 10;
  if (gHzArg > 120) gHzArg = 120;
  PE_parse_boot_argn("guamp-touch-softdetect-chunk", &gChunk, sizeof(gChunk));
  if (gChunk != 256) gChunk = 128;
  PE_parse_boot_argn("guamp-touch-softdetect-wait", &gWaitMax, sizeof(gWaitMax));
  if (gWaitMax < 2) gWaitMax = 2;
  if (gWaitMax > 200) gWaitMax = 200;
  PE_parse_boot_argn("guamp-touch-softdetect-log", &gLogS, sizeof(gLogS));
  PE_parse_boot_argn("guamp-touch-softdetect-maplog", &gMapLog, sizeof(gMapLog));
  const unsigned rx = gChip.rx_num, tx = gChip.tx_num;
  if (!rx || !tx || rx > guamp_softtouch::kMaxRx || tx > guamp_softtouch::kMaxTx || rx * tx > kCells ||
      HX_RAW_BYTES(rx, tx) > sizeof(gBuf)) {
    IOLog("GuampTouch: soft: canali %ux%u fuori misura, spento\n", rx, tx);
    gMode = 0;
    return;
  }
  setHz(gHzArg ? gHzArg : 60);
  configure();
  if (rawdump::gMode) {
    IOLog("GuampTouch: soft: rawdump %u spento (stesso raw out select)\n", rawdump::gMode);
    rawdump::gMode = 0;
  }
  const guamp_softtouch::Config &c = gDet.cfg;
  IOLog("GuampTouch: soft modo %u (%s), sorgente %s, %u Hz%s, %u byte a pezzi di %u, attesa max %u ms | soglia min %d k %d/4 "
        "rel %d%% verso %d | maschere tx %x rx %x auto %d, modo comune %d, baseline %u mappe | assi %d edge %d, %ux%u -> %dx%d px, "
        "max %u contatti, giu' %u su %u mappe, porta %d px, filtro %d/16, fermo %u mappe, interp %d\n",
        gMode, gMode == 1 ? "sostituisce i report del firmware" : "solo log, si pubblica il firmware", gSrc == 1 ? "delta IIR" : "raw DC",
        gHz, gHzArg ? "" : " (automatico 30..120)", static_cast<unsigned>(HX_RAW_BYTES(rx, tx)), gChunk, gWaitMax, c.thrMin, c.noiseK,
        c.relPct, c.sign, c.txMask, c.rxMask, c.autoMask, c.cm, c.learn, c.axes, c.edge, tx, rx, c.width, c.height, c.maxContacts,
        c.downFrames, c.upFrames, c.gatePx, c.smooth, c.stuckFrames, c.interp);
  selectMap();
  gNext = mach_absolute_time();
  gLogNext = gNext + usToTicks(static_cast<uint64_t>(gLogS ? gLogS : 1) * 1000000u);
  gAcc = Acc(); gWin = Acc();
  gAcc.startT = gWin.startT = gNext;
}

// Millisecondi fino alla prossima mappa (per l'attesa del ciclo principale), al massimo 20.
unsigned waitMs() {
  if (!gMode) return 20;
  const uint64_t now = mach_absolute_time();
  if (now >= gNext) return 1;
  const uint64_t ms = ticksToUs(gNext - now) / 1000;
  return ms < 1 ? 1 : ms > 20 ? 20 : static_cast<unsigned>(ms);
}

void release() {
  if (gMode == 1 && gPrevCount) { struct hx_report up = {}; publish(up); }
  gPrevCount = 0;
}

// Schermo spento / acceso (ciclo del gesto di risveglio): fermo e poi tutto da capo (reset del chip = select perso).
void suspend() { release(); gSelected = false; }
void resume() { if (!gMode) return; selectMap(); configure(); gLearned = false; gNext = mach_absolute_time(); }

void logStats(uint64_t now) {
  Acc &w = gWin;
  const uint64_t spanUs = ticksToUs(now - w.startT);
  const unsigned f = w.frames ? w.frames : 1;
  const guamp_softtouch::Stats &s = gDet.st;
  const unsigned rx = gDet.cfg.rx;
  IOLog("GuampTouch: soft stats: %u mappe in %llu ms = %llu.%llu Hz (obiettivo %u), attese %u, attesa media %llu us max %llu, "
        "lettura media %llu us max %llu (bus %llu%%), calcolo media %llu us max %llu | errori %u (timeout %u, somma %u) | rumore %d "
        "soglie %d/%d, mascherati %u, saturi %u, massimi %u scartati %u, d %d@t%u,r%u .. %d@t%u,r%u | contatti ora %u, iniziati %u, "
        "rilasciati %u, fermi %u | totale %u mappe\n",
        w.frames, spanUs / 1000, spanUs ? w.frames * 1000000ull / spanUs : 0, spanUs ? (w.frames * 10000000ull / spanUs) % 10 : 0,
        gHz, w.waited, w.waitUs / f, w.waitMaxUs, w.readUs / f, w.readMaxUs, spanUs ? w.readUs * 100 / spanUs : 0, w.calcUs / f,
        w.calcMaxUs, w.errs, w.timeouts, w.sums, s.noise, s.thrHigh, s.thrLow, s.masked, s.saturated, s.peaks, s.rejected,
        s.maxPos, s.maxAt / rx, s.maxAt % rx, s.maxNeg, s.minAt / rx, s.minAt % rx, s.down, s.started, s.released, s.stuck,
        gAcc.frames);
}

// Frequenza automatica, ogni 64 mappe: bus SPI entro il 25% del periodo, e non piu' veloce del firmware (attese).
void adapt(Acc &w) {
  if (gHzArg || w.frames < 64) return;
  const uint64_t readUs = w.readUs / w.frames;
  unsigned hz = gHz;
  const unsigned busCap = readUs ? static_cast<unsigned>(250000u / readUs) : 120;
  if (w.waited * 2 > w.frames || hz > busCap) hz = hz * 9 / 10;
  else if (!w.waited && hz < busCap) hz = hz * 11 / 10 + 1;
  if (hz > busCap) hz = busCap;
  if (hz > 120) hz = 120;
  if (hz < 30) hz = 30;
  if (hz != gHz) setHz(hz);
}

// Dal ciclo principale a ogni giro. Ritorna true se i contatti sono nostri (modo 1): il chiamante non pubblica il firmware.
bool step() {
  if (!gMode) return false;
  const uint64_t now = mach_absolute_time();
  if (gLogS && now >= gLogNext) {
    logStats(now);
    gWin = Acc();
    gWin.startT = now;
    gLogNext = now + usToTicks(static_cast<uint64_t>(gLogS) * 1000000u);
  }
  if (now < gNext) return gMode == 1;
  if (!gSelected && !selectMap()) { gNext = now + usToTicks(1000000); return gMode == 1; }
  const unsigned rx = gDet.cfg.rx, tx = gDet.cfg.tx;
  uint64_t waitUs = 0, readUs = 0;
  Acc *const both[2] = {&gAcc, &gWin};
  const int rc = readFrame(rx, tx, &waitUs, &readUs);
  const uint64_t t2 = mach_absolute_time();
  for (Acc *a : both) {
    if (waitUs > 1000) a->waited++;
    a->waitUs += waitUs; a->readUs += readUs;
    if (waitUs > a->waitMaxUs) a->waitMaxUs = waitUs;
    if (readUs > a->readMaxUs) a->readMaxUs = readUs;
  }
  if (rc) {
    for (Acc *a : both) { a->errs++; if (rc == HX_ERR_TIMEOUT) a->timeouts++; if (rc == HX_ERR_CHECKSUM) a->sums++; }
    gFailRun++;
    if (gAcc.errs <= 8 || (gAcc.errs & 255) == 0)
      IOLog("GuampTouch: soft mappa %u: rc %d dopo %llu us (errori %u, di fila %u), testa %02x%02x%02x%02x\n", gSeq, rc, waitUs,
            gAcc.errs, gFailRun, gBuf[0], gBuf[1], gBuf[2], gBuf[3]);
    // Cinque errori di fila (schermo spento, firmware fermo): contatti rilasciati, un tentativo al secondo.
    if (gFailRun >= 5) {
      if (!gSlow) { IOLog("GuampTouch: soft: %u errori di fila, un tentativo al secondo\n", gFailRun); release(); }
      gSlow = true;
      gNext = now + usToTicks(1000000);
    } else {
      gNext = now + gPeriod;
    }
    return gMode == 1;
  }
  gFailRun = 0;
  if (gSlow || (gLastOk && now - gLastOk > usToTicks(1000000))) {
    // Ripresa dopo una pausa: baseline da rifare.
    IOLog("GuampTouch: soft: mappe di nuovo leggibili, baseline da capo\n");
    gSlow = false;
    configure();
    gLearned = false;
  }
  gLastOk = now;
  gNext = (now - gNext < gPeriod ? gNext : now) + gPeriod;
  gSeq++;
  Contact cs[guamp_softtouch::kMaxTracks];
  const unsigned started = gDet.st.started;
  const unsigned n = gDet.frame(gMap, cs);
  const uint64_t calcUs = ticksToUs(mach_absolute_time() - t2);
  for (Acc *a : both) {
    a->frames++;
    a->calcUs += calcUs;
    if (calcUs > a->calcMaxUs) a->calcMaxUs = calcUs;
  }
  adapt(gWin);
  if (!gLearned && !gDet.st.learnLeft) {
    gLearned = true;
    int bmn = 0x7fffffff, bmx = -0x7fffffff;
    for (unsigned i = 0; i < rx * tx; ++i) { const int b = gDet.base(i); bmn = b < bmn ? b : bmn; bmx = b > bmx ? b : bmx; }
    char rows[3 * guamp_softtouch::kMaxTx + 1] = "";
    unsigned len = 0;
    for (unsigned t = 0; t < tx; ++t) {
      unsigned m = 0;
      for (unsigned r = 0; r < rx; ++r) m += gDet.masked(t, r) ? 1 : 0;
      if (m == rx && len + 4 < sizeof(rows)) len += static_cast<unsigned>(snprintf(rows + len, sizeof(rows) - len, " %u", t));
    }
    IOLog("GuampTouch: soft baseline pronta (mappa %u): base %d..%d, rumore %d, soglie %d/%d, mascherati %u nodi (righe TX intere:%s), "
          "lettura %llu us, attesa %llu us\n", gSeq, bmn, bmx, gDet.st.noise, gDet.st.thrHigh, gDet.st.thrLow, gDet.st.masked,
          len ? rows : " nessuna", readUs, waitUs);
  }
  // Contatti giu'/su nel log.
  for (unsigned i = 0; i < n; ++i) {
    bool old = false;
    for (unsigned j = 0; j < gPrevCount; ++j) old = old || gPrevSlots[j] == cs[i].slot;
    if (!old && (gLogged < 64 || (gLogged & 31) == 0))
      IOLog("GuampTouch: soft contatto %u slot %d giu' a %d,%d px (ampiezza %d, soglia %d, rumore %d), mappa %u\n", gDet.st.started,
            cs[i].slot, cs[i].x, cs[i].y, cs[i].amp, gDet.st.thrHigh, gDet.st.noise, gSeq);
    if (!old) gLogged++;
  }
  for (unsigned j = 0; j < gPrevCount; ++j) {
    bool still = false;
    for (unsigned i = 0; i < n; ++i) still = still || cs[i].slot == gPrevSlots[j];
    if (!still && gLogged <= 64) IOLog("GuampTouch: soft slot %d su, mappa %u\n", gPrevSlots[j], gSeq);
  }
  if (gDet.st.started != started && gMapLog) {
    gMapLog--;
    const int32_t *d = gDet.diff();
    for (unsigned t = 0; t < tx; ++t) row(gSeq, t, d + t * rx, rx);
  }
  // Pubblicazione: ogni mappa con contatti, e una volta "tutti su".
  if (gMode == 1 && (n || gPrevCount)) {
    struct hx_report rep = {};
    rep.count = n;
    for (unsigned i = 0; i < n && i < HX_MAX_PT; ++i) {
      rep.slot[i] = cs[i].slot;
      rep.point[i].x = cs[i].x; rep.point[i].y = cs[i].y; rep.point[i].w = cs[i].amp;
    }
    rep.point_cnt = static_cast<uint8_t>(n);
    publish(rep);
  }
  gPrevCount = n;
  for (unsigned i = 0; i < n; ++i) gPrevSlots[i] = cs[i].slot;
  return gMode == 1;
}
}  // namespace softdetect
