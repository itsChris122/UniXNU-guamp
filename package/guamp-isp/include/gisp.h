/* SPDX-License-Identifier: BSD-2-Clause
 * guamp-isp: ISP software per i sensori del Moto G9 Play (guamp).
 * RAW10 Bayer (TFE RDI) -> NV12 (420v/420f) + statistiche 3A.
 *
 * Due percorsi:
 *  - "HQ" in virgola mobile (gisp_process): demosaic MHC, riduzione del rumore, nitidezza, ridimensionamento ad area.
 *    Per le foto e come riferimento.
 *  - "fast" in interi (gisp_fast_*): nero, LSC, WB, demosaic bilineare (o superpixel 2x2), scala separabile in
 *    lineare, CCM, gamma a LUT, NV12. Per l'anteprima: NEON su arm64, C portabile altrove, a bande per piu' thread.
 * Nessun dato del vendor e' contenuto qui: i parametri arrivano a runtime (gisp_tuning, file .gisp).
 */
#ifndef GISP_H
#define GISP_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Posizione del rosso nel quadrato 2x2 in alto a sinistra. */
enum gisp_cfa { GISP_RGGB = 0, GISP_GRBG = 1, GISP_GBRG = 2, GISP_BGGR = 3, GISP_MONO = 4 };
enum gisp_raw_format {
    GISP_RAW10_MIPI = 0, /* 4 pixel in 5 byte: b0..b3 = bit 9..2, b4 = bit 1..0 (pixel 0 nei bit 1..0) */
    GISP_RAW16 = 1       /* un uint16 little endian per pixel, valore a 10 bit allineato a destra */
};
enum gisp_matrix { GISP_BT601 = 0, GISP_BT709 = 1 };
enum gisp_demosaic { GISP_DEMOSAIC_BILINEAR = 0, GISP_DEMOSAIC_MHC = 1 };

/* Sito CFA: indice = (y & 1) * 2 + (x & 1). Colore di un sito: 0 R, 1 G, 2 B. */
int gisp_cfa_color(uint32_t cfa, uint32_t site);

typedef struct {
    uint32_t width, height; /* pari */
    uint32_t stride;        /* byte per riga */
    uint32_t format;        /* enum gisp_raw_format */
    uint32_t cfa;           /* enum gisp_cfa */
    const uint8_t *data;
} gisp_raw;

typedef struct { uint32_t x, y, w, h; } gisp_rect; /* ritaglio in pixel del sensore, tutto pari; w = 0: intero */

#define GISP_LSC_MAX 1024 /* celle per canale (la griglia Qualcomm e' 17x13 = 221) */

typedef struct {
    float black[4];          /* livello del nero per sito CFA (unita' a 10 bit) */
    float white;             /* saturazione del sensore, 1023 */
    uint32_t lsc_w, lsc_h;   /* griglia della correzione d'ombra; 0 = spenta */
    float lsc[4][GISP_LSC_MAX]; /* guadagni R, Gr, Gb, B (Gr = verde sulla riga del rosso), riga per riga */
    /* Campo coperto dalla griglia LSC (di solito l'intero sensore a piena risoluzione). Il pixel RAW (x, y) sta nel
     * campo a (lsc_off_x + x * lsc_scale, lsc_off_y + y * lsc_scale). lsc_field_w = 0: il campo e' il RAW stesso.
     * Esempio GM1ST 1920x1080 binned 2x2: campo 4000x3000, scala 2, offset del ritaglio del sensore. */
    uint32_t lsc_field_w, lsc_field_h;
    float lsc_off_x, lsc_off_y, lsc_scale;
    float wb[3];             /* guadagni R, G, B del bilanciamento del bianco */
    float ccm[9];            /* RGB della camera (gia' bilanciato) -> sRGB lineare, per righe */
    float gamma[257];        /* curva: uscita 0..1 per ingresso lineare i/256 */
    uint32_t demosaic;       /* enum gisp_demosaic (solo HQ) */
    float nr;                /* 0..1 riduzione del rumore (luma e croma, solo HQ) */
    float sharpen;           /* 0..2 quantita' di nitidezza (unsharp mask sulla luma, solo HQ) */
    float sharpen_thr;       /* soglia in passi di luma a 8 bit */
    uint32_t matrix;         /* enum gisp_matrix */
    uint32_t full_range;     /* 1 = 420f (0..255), 0 = 420v (Y 16..235, C 16..240) */
} gisp_params;

/* Valori ragionevoli: nero 64, bianco 1023, niente LSC, WB 1, CCM identita', gamma sRGB, MHC, BT.709 video. */
void gisp_params_default(gisp_params *p);
void gisp_gamma_srgb(float gamma[257]);

typedef struct {
    uint32_t width, height;  /* pari */
    uint8_t *y;  size_t y_stride;
    uint8_t *uv; size_t uv_stride; /* CbCr interlacciati (NV12) */
} gisp_nv12;

/* Lettura di un pixel RAW (valore a 10 bit). */
uint32_t gisp_raw_pixel(const gisp_raw *raw, uint32_t x, uint32_t y);
/* Guadagno LSC (1.0 = nessuna correzione) del pixel RAW (x, y), secondo la griglia e il campo dei parametri. */
float gisp_lsc_gain(const gisp_raw *raw, const gisp_params *p, uint32_t x, uint32_t y);

/* ---- statistiche 3A ------------------------------------------------------------------------------------------ */
#define GISP_ZONES_MAX (32 * 32)
typedef struct {
    uint32_t zones_x, zones_y; /* da impostare prima del calcolo (1..32); 0 = 16x12 */
    uint32_t step;             /* sottocampionamento in quadrati 2x2 (1 = tutti); 0 = 2 */
    /* per zona, prima del WB (nero tolto, LSC applicata), unita' a 10 bit, solo quadrati non saturi */
    float sum_r[GISP_ZONES_MAX], sum_g[GISP_ZONES_MAX], sum_b[GISP_ZONES_MAX];
    uint32_t count[GISP_ZONES_MAX];
    uint32_t sat[GISP_ZONES_MAX];   /* quadrati con almeno un canale saturo */
    float sharp[GISP_ZONES_MAX];    /* contrasto per l'AF: somma |dG| orizzontale + verticale tra quadrati, per zona */
    uint32_t hist[256];             /* istogramma della luma lineare (WB applicato), 0..bianco in 256 classi */
    uint32_t total;                 /* quadrati contati */
    float mean_luma;                /* media della luma lineare, 0..1 */
    float range;                    /* fondo scala delle somme: bianco - nero medio */
} gisp_stats;

/* crop NULL = intero. Le zone dividono il ritaglio. */
void gisp_stats_compute(const gisp_raw *raw, const gisp_rect *crop, const gisp_params *p, gisp_stats *st);

/* Formato compatto per l'IPC verso il 3A (IOConnectCallStructMethod inline, < 4096 byte), little endian.
 * Zone fisse 16x12. Medie per zona in Q12 (4096 = bianco - nero), prima del WB, LSC applicata. */
#define GISP_WIRE_MAGIC 0x31545347u /* "GST1" */
#define GISP_WIRE_ZX 16
#define GISP_WIRE_ZY 12
#define GISP_WIRE_HIST 64
typedef struct {
    uint32_t magic, size;          /* size = sizeof(gisp_stats_wire) */
    uint32_t frame;                /* numero del frame RAW */
    uint32_t zones_x, zones_y;     /* 16, 12 */
    uint32_t total;                /* quadrati contati */
    uint16_t mean_luma_q12;        /* luma lineare con WB, media */
    uint16_t sat_permille;         /* quadrati saturi, per mille */
    uint16_t p50_q12, p95_q12;     /* percentili 50 e 95 della luma */
    uint16_t r[GISP_WIRE_ZX * GISP_WIRE_ZY], g[GISP_WIRE_ZX * GISP_WIRE_ZY], b[GISP_WIRE_ZX * GISP_WIRE_ZY];
    uint8_t sat[GISP_WIRE_ZX * GISP_WIRE_ZY];      /* per zona: frazione di quadrati saturi, 0..255 */
    uint8_t valid[GISP_WIRE_ZX * GISP_WIRE_ZY];    /* per zona: frazione di quadrati non saturi, 0..255 */
    uint32_t sharp[GISP_WIRE_ZX * GISP_WIRE_ZY];   /* contrasto AF per zona: media |dG| per quadrato, Q16 di 1.0 */
    uint32_t hist[GISP_WIRE_HIST];                 /* luma lineare 0..1 in 64 classi */
} gisp_stats_wire;
/* st deve essere stato calcolato con zones 16x12 (altrimenti restituisce -1). */
int gisp_stats_to_wire(const gisp_stats *st, uint32_t frame, gisp_stats_wire *w);

/* ---- percorso HQ ---------------------------------------------------------------------------------------------- */
typedef struct gisp_ctx gisp_ctx;
gisp_ctx *gisp_ctx_new(void);
void gisp_ctx_free(gisp_ctx *ctx);
/* 0 = ok, <0 errore. crop NULL = intero. L'uscita puo' essere piu' piccola (ridimensionamento ad area) o piu' grande. */
int gisp_process(gisp_ctx *ctx, const gisp_raw *raw, const gisp_rect *crop, const gisp_params *p, gisp_nv12 *out);

/* Conversione inversa NV12 -> RGB 8 bit interlacciato (per PPM e per i test). */
void gisp_nv12_to_rgb8(const gisp_nv12 *in, uint32_t matrix, uint32_t full_range, uint8_t *rgb, size_t rgb_stride);

/* ---- percorso fast (anteprima) --------------------------------------------------------------------------------- */
enum gisp_fast_mode {
    GISP_FAST_AUTO = 0, /* BIN2 se il ritaglio e' almeno il doppio dell'uscita nei due assi, altrimenti FULL */
    GISP_FAST_FULL = 1, /* demosaic bilineare a piena risoluzione, poi scala */
    GISP_FAST_BIN2 = 2  /* superpixel 2x2 (R, (Gr+Gb)/2, B) a meta' risoluzione, poi scala */
};
#define GISP_FAST_MAX_BANDS 16
typedef struct gisp_fast gisp_fast;
/* Geometria fissa: dimensioni, stride, formato e CFA del RAW, ritaglio, uscita (larghezza e altezza pari). */
gisp_fast *gisp_fast_new(const gisp_raw *geom, const gisp_rect *crop, uint32_t out_w, uint32_t out_h, uint32_t mode,
                         const gisp_params *p);
void gisp_fast_free(gisp_fast *f);
/* Nuovi parametri (WB, CCM, nero, LSC, gamma) tra un frame e l'altro: costo trascurabile. Non durante un frame. */
void gisp_fast_set_params(gisp_fast *f, const gisp_params *p);
uint32_t gisp_fast_mode_of(const gisp_fast *f);
/* Una banda orizzontale dell'uscita (band < nbands <= GISP_FAST_MAX_BANDS). Bande diverse possono girare in
 * parallelo (dispatch_apply, pthread). Il RAW deve avere la geometria data a gisp_fast_new. */
int gisp_fast_band(gisp_fast *f, const gisp_raw *raw, gisp_nv12 *out, uint32_t band, uint32_t nbands);
/* Tutto il frame con nthreads pthread (1 = nel thread chiamante). */
int gisp_fast_run(gisp_fast *f, const gisp_raw *raw, gisp_nv12 *out, uint32_t nthreads);
/* 1 se compilato con il percorso NEON. */
int gisp_fast_has_neon(void);

/* ---- tuning (file .gisp prodotto da tools/chromatix.py) -------------------------------------------------------- */
#define GISP_TUNING_MAGIC 0x50534947u /* "GISP" */
#define GISP_TUNING_VERSION 1u
#define GISP_TUNING_MAX_CCM 8
#define GISP_TUNING_MAX_REF 16
/* Il file e' {u32 magic, u32 version, u32 size = sizeof(gisp_tuning)} seguito dalla struttura cosi' com'e'
 * (solo campi da 4 byte, little endian, senza padding). */
typedef struct {
    char sensor[32];
    uint32_t cfa;
    float black[4];
    float white;
    uint32_t lsc_w, lsc_h;
    float lsc[4][GISP_LSC_MAX];
    uint32_t n_ccm;
    float ccm_cct[GISP_TUNING_MAX_CCM]; /* centro dell'intervallo di CCT (K) */
    float ccm[GISP_TUNING_MAX_CCM][9];
    uint32_t n_ref;                     /* punti di riferimento AWB: R/G, B/G e CCT stimata (0 = ignota) */
    float ref_rg[GISP_TUNING_MAX_REF], ref_bg[GISP_TUNING_MAX_REF], ref_cct[GISP_TUNING_MAX_REF];
    float gamma[257];
} gisp_tuning;

int gisp_tuning_parse(const void *data, size_t size, gisp_tuning *t);
/* Copia nero/LSC/gamma e interpola la CCM (in mired) alla CCT data in kelvin. Non tocca il campo LSC ne' il WB. */
void gisp_tuning_apply(const gisp_tuning *t, float cct, gisp_params *p);
/* AWB di base per l'agente 3A: grigio del mondo sulle zone vicine al luogo dei punti di riferimento.
 * Restituisce 0 e riempie wb[3] (G = 1) e *cct, oppure -1 se non ci sono zone utili. t NULL: grigio del mondo puro. */
int gisp_awb_estimate(const gisp_stats *st, const gisp_tuning *t, float wb[3], float *cct);

#ifdef __cplusplus
}
#endif
#endif
