// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp-aks-keybag=1: a software keybag for the owned AppleKeyStore (NativeKeyStoreEvents + scripts/guamp-aks-variant.py).
//
// The model (how keybagd --init and securityd use a keybag, the system bag as a separate copy, kIOReturnNotFound for "no
// system bag yet", plain systembag.kb when /defaults says no-effaceable-storage, entitlement-gated bag and key selectors)
// follows HakiOS MiniAppleKeyStore.cpp (github.com/oosexgang hakios-xnu-kernel, branch mtk, APSL 2.0); the code here is
// written anew for guamp and checked against iPadOS 21H461 (libaks selectors, keybagd load_devicebag), see
// analisi/KEYBAG-REALE-20261107.md. Pure logic, no kernel dependency: the same file is compiled by the host test
// package/tests/guamp_keybag_host.c and by package/KeyStoreProbe.cpp, which supplies file I/O, randomness and logging.
//
// What it is: a bag = a random device key + 32 random class keys + a UUID. aks_wrap_key/aks_unwrap_key (selectors 10/11)
// are AES-256 key wrap (RFC 3394) with the key of the item's class, with the integrity check on unwrap. The bag leaves the
// kernel only as the blob of aks_save_bag / systembag.kb, which carries the device key in clear: there is no hardware key
// (no SEP UID) to protect it, so whoever can read the Data partition can read every class key. It protects keychain keys
// from other programs (entitlements), not from someone holding the flash.
#ifndef GUAMP_KEYBAG_CORE_H
#define GUAMP_KEYBAG_CORE_H

#include <stddef.h>
#include <stdint.h>

#define GKB_KEY 32u
#define GKB_CLASSES 32u                       /* protection classes 0..31 (keychain 6..11 seen on guamp, files 1..4) */
#define GKB_BAGS 8u                           /* handle = slot index; 0 is never handed out: it names the system bag */
#define GKB_MAGIC 0x3142474bu                 /* bytes 'K','G','B','1' */
#define GKB_VERSION 1u
#define GKB_WRAPPED_CLASSES (GKB_CLASSES * GKB_KEY + 8u)

/* The old stand-in's blob (guamp-aks-wrap=1 without keybag): 8-byte tag + key in clear. Read for migration. */
static const uint8_t gkb_legacy_tag[8] = {'G', 'U', 'A', 'M', 'P', 'A', 'K', 'S'};

enum {
    GKB_OK = 0,
    GKB_NOT_FOUND,          /* kIOReturnNotFound 0xe00002f0: keybagd --init reads this as "load systembag.kb" */
    GKB_BAD_ARGUMENT,
    GKB_NOT_PRIVILEGED,
    GKB_NOT_PERMITTED,      /* unwrap integrity check failed: another bag's key, or not a wrapped key */
    GKB_NO_RESOURCES,
    GKB_NO_SPACE,
};

/* rights of a client, from its entitlements (KeyStoreProbe.cpp) */
#define GKB_RIGHT_DEVICE 1u                   /* com.apple.keystore.device (keybagd) */
#define GKB_RIGHT_KEYCHAIN 2u                 /* com.apple.keystore.access-keychain-keys (securityd, sharingd, ...) */

/* 21H461 GSSCred has platform-application, com.apple.mkb.usersession.info and its own seatbelt profile, but neither
 * keystore entitlement. It wraps its credential-cache keys directly through libaks. This exact signed capability
 * combination permits key operations; exporting or replacing the system bag still requires RIGHT_DEVICE. */
static inline uint32_t gkb_client_rights(int device, int keychain, int platform, int usersession, int gsscred_profile) {
    uint32_t rights = device ? GKB_RIGHT_DEVICE : 0;
    if (keychain || (platform && usersession && gsscred_profile)) rights |= GKB_RIGHT_KEYCHAIN;
    return rights;
}

struct gkb_bag {
    uint8_t used;
    uint32_t type;
    uint8_t uuid[16];
    uint8_t device_key[GKB_KEY];
    uint8_t class_key[GKB_CLASSES][GKB_KEY];
};

/* aks_save_bag output and the KeyBagKeys data of systembag.kb (little endian, 1096 bytes) */
struct gkb_blob {
    uint32_t magic, version, type, classes;
    uint8_t uuid[16];
    uint8_t device_key[GKB_KEY];
    uint8_t wrapped[GKB_WRAPPED_CLASSES];
};
typedef char gkb_blob_size_check[sizeof(struct gkb_blob) == 1096 ? 1 : -1];

/* The environment: systembag.kb, randomness, a log line. exists/read/write return 0 on success. */
struct gkb_io {
    void *ctx;
    int (*file_exists)(void *ctx);
    int (*file_read)(void *ctx, uint8_t *buf, size_t cap, size_t *len);
    int (*file_write)(void *ctx, const uint8_t *buf, size_t len);   /* create only: never replaces a file */
    void (*random)(void *ctx, void *buf, size_t len);
    void (*log)(void *ctx, const char *event, long value);
};

struct gkb_state {
    struct gkb_bag bag[GKB_BAGS];
    int system;                  /* slot of the system bag, -1 none */
    uint8_t not_found_given;     /* keybagd was told "no system bag" once: it loads the file */
    uint8_t load_failed;         /* a systembag.kb was offered and did not check out */
    uint8_t ephemeral;           /* the system bag lives only until the next boot (file present but unreadable, or unwritable) */
    uint8_t from_file;           /* the system bag came from systembag.kb (keybagd or the kernel read it) */
};

static inline void gkb_zero(void *p, size_t n) { volatile uint8_t *b = (volatile uint8_t *)p; while (n--) *b++ = 0; }
static inline void gkb_copy(void *d, const void *s, size_t n) {
    uint8_t *o = (uint8_t *)d; const uint8_t *i = (const uint8_t *)s;
    if (o < i) { while (n--) *o++ = *i++; } else { o += n; i += n; while (n--) *--o = *--i; }
}
static inline int gkb_differ(const void *a, const void *b, size_t n) {
    const uint8_t *x = (const uint8_t *)a, *y = (const uint8_t *)b; uint8_t d = 0;
    for (size_t i = 0; i < n; ++i) d |= (uint8_t)(x[i] ^ y[i]);
    return d != 0;
}

/* ---- AES-256 (FIPS-197), byte oriented: a few hundred block operations per boot, speed does not matter ---- */
static const uint8_t gkb_sbox[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16};
static const uint8_t gkb_inv_sbox[256] = {
    0x52,0x09,0x6a,0xd5,0x30,0x36,0xa5,0x38,0xbf,0x40,0xa3,0x9e,0x81,0xf3,0xd7,0xfb,
    0x7c,0xe3,0x39,0x82,0x9b,0x2f,0xff,0x87,0x34,0x8e,0x43,0x44,0xc4,0xde,0xe9,0xcb,
    0x54,0x7b,0x94,0x32,0xa6,0xc2,0x23,0x3d,0xee,0x4c,0x95,0x0b,0x42,0xfa,0xc3,0x4e,
    0x08,0x2e,0xa1,0x66,0x28,0xd9,0x24,0xb2,0x76,0x5b,0xa2,0x49,0x6d,0x8b,0xd1,0x25,
    0x72,0xf8,0xf6,0x64,0x86,0x68,0x98,0x16,0xd4,0xa4,0x5c,0xcc,0x5d,0x65,0xb6,0x92,
    0x6c,0x70,0x48,0x50,0xfd,0xed,0xb9,0xda,0x5e,0x15,0x46,0x57,0xa7,0x8d,0x9d,0x84,
    0x90,0xd8,0xab,0x00,0x8c,0xbc,0xd3,0x0a,0xf7,0xe4,0x58,0x05,0xb8,0xb3,0x45,0x06,
    0xd0,0x2c,0x1e,0x8f,0xca,0x3f,0x0f,0x02,0xc1,0xaf,0xbd,0x03,0x01,0x13,0x8a,0x6b,
    0x3a,0x91,0x11,0x41,0x4f,0x67,0xdc,0xea,0x97,0xf2,0xcf,0xce,0xf0,0xb4,0xe6,0x73,
    0x96,0xac,0x74,0x22,0xe7,0xad,0x35,0x85,0xe2,0xf9,0x37,0xe8,0x1c,0x75,0xdf,0x6e,
    0x47,0xf1,0x1a,0x71,0x1d,0x29,0xc5,0x89,0x6f,0xb7,0x62,0x0e,0xaa,0x18,0xbe,0x1b,
    0xfc,0x56,0x3e,0x4b,0xc6,0xd2,0x79,0x20,0x9a,0xdb,0xc0,0xfe,0x78,0xcd,0x5a,0xf4,
    0x1f,0xdd,0xa8,0x33,0x88,0x07,0xc7,0x31,0xb1,0x12,0x10,0x59,0x27,0x80,0xec,0x5f,
    0x60,0x51,0x7f,0xa9,0x19,0xb5,0x4a,0x0d,0x2d,0xe5,0x7a,0x9f,0x93,0xc9,0x9c,0xef,
    0xa0,0xe0,0x3b,0x4d,0xae,0x2a,0xf5,0xb0,0xc8,0xeb,0xbb,0x3c,0x83,0x53,0x99,0x61,
    0x17,0x2b,0x04,0x7e,0xba,0x77,0xd6,0x26,0xe1,0x69,0x14,0x63,0x55,0x21,0x0c,0x7d};

struct gkb_aes { uint8_t rk[240]; };    /* 15 round keys */

static inline uint8_t gkb_xtime(uint8_t a) { return (uint8_t)((a << 1) ^ ((a & 0x80) ? 0x1b : 0)); }
static inline uint8_t gkb_mul(uint8_t a, uint8_t b) {
    uint8_t r = 0;
    while (b) { if (b & 1) r ^= a; a = gkb_xtime(a); b >>= 1; }
    return r;
}
static inline void gkb_aes_key(struct gkb_aes *c, const uint8_t key[32]) {
    uint8_t rcon = 1;
    gkb_copy(c->rk, key, 32);
    for (unsigned i = 8; i < 60; ++i) {
        uint8_t t[4];
        gkb_copy(t, c->rk + 4 * (i - 1), 4);
        if (i % 8 == 0) {
            uint8_t f = t[0];
            t[0] = (uint8_t)(gkb_sbox[t[1]] ^ rcon); t[1] = gkb_sbox[t[2]]; t[2] = gkb_sbox[t[3]]; t[3] = gkb_sbox[f];
            rcon = gkb_xtime(rcon);
        } else if (i % 8 == 4) {
            for (unsigned k = 0; k < 4; ++k) t[k] = gkb_sbox[t[k]];
        }
        for (unsigned k = 0; k < 4; ++k) c->rk[4 * i + k] = (uint8_t)(c->rk[4 * (i - 8) + k] ^ t[k]);
    }
}
static inline void gkb_aes_encrypt(const struct gkb_aes *c, uint8_t s[16]) {
    for (unsigned k = 0; k < 16; ++k) s[k] ^= c->rk[k];
    for (unsigned r = 1; r <= 14; ++r) {
        uint8_t t[16];
        for (unsigned k = 0; k < 16; ++k) t[k] = gkb_sbox[s[(k + 4 * (k % 4)) % 16]];   /* SubBytes + ShiftRows */
        if (r != 14) {
            for (unsigned col = 0; col < 4; ++col) {                                     /* MixColumns */
                uint8_t *a = t + 4 * col, a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3], all = (uint8_t)(a0 ^ a1 ^ a2 ^ a3);
                a[0] ^= (uint8_t)(all ^ gkb_xtime((uint8_t)(a0 ^ a1)));
                a[1] ^= (uint8_t)(all ^ gkb_xtime((uint8_t)(a1 ^ a2)));
                a[2] ^= (uint8_t)(all ^ gkb_xtime((uint8_t)(a2 ^ a3)));
                a[3] ^= (uint8_t)(all ^ gkb_xtime((uint8_t)(a3 ^ a0)));
            }
        }
        for (unsigned k = 0; k < 16; ++k) s[k] = (uint8_t)(t[k] ^ c->rk[16 * r + k]);
        gkb_zero(t, sizeof t);
    }
}
static inline void gkb_aes_decrypt(const struct gkb_aes *c, uint8_t s[16]) {
    for (unsigned k = 0; k < 16; ++k) s[k] ^= c->rk[16 * 14 + k];
    for (unsigned r = 14; r-- > 0;) {
        uint8_t t[16];
        for (unsigned k = 0; k < 16; ++k) t[(k + 4 * (k % 4)) % 16] = gkb_inv_sbox[s[k]];   /* InvShiftRows + InvSubBytes */
        for (unsigned k = 0; k < 16; ++k) s[k] = (uint8_t)(t[k] ^ c->rk[16 * r + k]);
        if (r != 0) {
            for (unsigned col = 0; col < 4; ++col) {                                         /* InvMixColumns */
                uint8_t *a = s + 4 * col, a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3];
                a[0] = (uint8_t)(gkb_mul(a0, 14) ^ gkb_mul(a1, 11) ^ gkb_mul(a2, 13) ^ gkb_mul(a3, 9));
                a[1] = (uint8_t)(gkb_mul(a0, 9) ^ gkb_mul(a1, 14) ^ gkb_mul(a2, 11) ^ gkb_mul(a3, 13));
                a[2] = (uint8_t)(gkb_mul(a0, 13) ^ gkb_mul(a1, 9) ^ gkb_mul(a2, 14) ^ gkb_mul(a3, 11));
                a[3] = (uint8_t)(gkb_mul(a0, 11) ^ gkb_mul(a1, 13) ^ gkb_mul(a2, 9) ^ gkb_mul(a3, 14));
            }
        }
        gkb_zero(t, sizeof t);
    }
}

/* ---- AES key wrap, RFC 3394 section 2.2.1/2.2.2, default IV A6..A6 ---- */
static const uint8_t gkb_kw_iv[8] = {0xa6, 0xa6, 0xa6, 0xa6, 0xa6, 0xa6, 0xa6, 0xa6};

/* len: multiple of 8, at least 16. out: len + 8 bytes (may not overlap in). */
static inline int gkb_kw_wrap(const uint8_t kek[32], const uint8_t *in, size_t len, uint8_t *out) {
    if (len < 16 || len % 8) return 0;
    struct gkb_aes c;
    uint8_t b[16];
    const size_t n = len / 8;
    gkb_aes_key(&c, kek);
    gkb_copy(b, gkb_kw_iv, 8);
    gkb_copy(out + 8, in, len);
    for (size_t j = 0; j < 6; ++j) {
        for (size_t i = 1; i <= n; ++i) {
            const uint64_t t = (uint64_t)(n * j + i);
            gkb_copy(b + 8, out + 8 * i, 8);
            gkb_aes_encrypt(&c, b);
            for (unsigned k = 0; k < 8; ++k) b[7 - k] ^= (uint8_t)(t >> (8 * k));
            gkb_copy(out + 8 * i, b + 8, 8);
        }
    }
    gkb_copy(out, b, 8);
    gkb_zero(&c, sizeof c);
    gkb_zero(b, sizeof b);
    return 1;
}
/* len: wrapped length (multiple of 8, at least 24). out: len - 8 bytes. 0 when the integrity check fails (out zeroed). */
static inline int gkb_kw_unwrap(const uint8_t kek[32], const uint8_t *in, size_t len, uint8_t *out) {
    if (len < 24 || len % 8) return 0;
    struct gkb_aes c;
    uint8_t b[16];
    const size_t n = len / 8 - 1;
    gkb_aes_key(&c, kek);
    gkb_copy(b, in, 8);
    gkb_copy(out, in + 8, len - 8);
    for (size_t j = 6; j-- > 0;) {
        for (size_t i = n; i >= 1; --i) {
            const uint64_t t = (uint64_t)(n * j + i);
            for (unsigned k = 0; k < 8; ++k) b[7 - k] ^= (uint8_t)(t >> (8 * k));
            gkb_copy(b + 8, out + 8 * (i - 1), 8);
            gkb_aes_decrypt(&c, b);
            gkb_copy(out + 8 * (i - 1), b + 8, 8);
        }
    }
    const int whole = !gkb_differ(b, gkb_kw_iv, 8);
    gkb_zero(&c, sizeof c);
    gkb_zero(b, sizeof b);
    if (!whole) gkb_zero(out, len - 8);
    return whole;
}

/* ---- bags ---- */
static inline void gkb_init(struct gkb_state *st) { gkb_zero(st, sizeof *st); st->system = -1; }

/* the bag a handle names: 0 = the system bag. NULL if none. */
static inline struct gkb_bag *gkb_bag_for(struct gkb_state *st, uint64_t handle) {
    if (handle == 0) {
        if (st->system < 0) return NULL;
        handle = (uint64_t)st->system;
    }
    if (handle >= GKB_BAGS || !st->bag[handle].used) return NULL;
    return &st->bag[handle];
}
static inline int gkb_free_slot(const struct gkb_state *st) {
    for (unsigned i = 1; i < GKB_BAGS; ++i) if (!st->bag[i].used) return (int)i;
    return -1;
}
static inline int gkb_create(struct gkb_state *st, const struct gkb_io *io, uint32_t type) {
    const int h = gkb_free_slot(st);
    if (h < 0) return -1;
    struct gkb_bag *b = &st->bag[h];
    gkb_zero(b, sizeof *b);
    b->used = 1;
    b->type = type;
    io->random(io->ctx, b->uuid, sizeof b->uuid);
    b->uuid[6] = (uint8_t)((b->uuid[6] & 0x0f) | 0x40);     /* version 4 UUID */
    b->uuid[8] = (uint8_t)((b->uuid[8] & 0x3f) | 0x80);
    io->random(io->ctx, b->device_key, sizeof b->device_key);
    io->random(io->ctx, b->class_key, sizeof b->class_key);
    return h;
}
static inline void gkb_save(const struct gkb_bag *b, struct gkb_blob *blob) {
    blob->magic = GKB_MAGIC;
    blob->version = GKB_VERSION;
    blob->type = b->type;
    blob->classes = GKB_CLASSES;
    gkb_copy(blob->uuid, b->uuid, sizeof blob->uuid);
    gkb_copy(blob->device_key, b->device_key, sizeof blob->device_key);
    gkb_kw_wrap(b->device_key, &b->class_key[0][0], sizeof b->class_key, blob->wrapped);
}
/* the slot of the loaded bag; -1 not ours or does not check out, -2 no room */
static inline int gkb_load(struct gkb_state *st, const uint8_t *bytes, size_t len) {
    struct gkb_blob blob;
    if (len != sizeof blob) return -1;
    gkb_copy(&blob, bytes, sizeof blob);
    if (blob.magic != GKB_MAGIC || blob.version != GKB_VERSION || blob.classes != GKB_CLASSES) { gkb_zero(&blob, sizeof blob); return -1; }
    for (unsigned h = 1; h < GKB_BAGS; ++h)     /* the same bag twice is one bag, but never the system's own copy */
        if ((int)h != st->system && st->bag[h].used && !gkb_differ(st->bag[h].uuid, blob.uuid, 16)) { gkb_zero(&blob, sizeof blob); return (int)h; }
    const int h = gkb_free_slot(st);
    if (h < 0) { gkb_zero(&blob, sizeof blob); return -2; }
    struct gkb_bag *b = &st->bag[h];
    if (!gkb_kw_unwrap(blob.device_key, blob.wrapped, sizeof blob.wrapped, &b->class_key[0][0])) {
        gkb_zero(b, sizeof *b);
        gkb_zero(&blob, sizeof blob);
        return -1;
    }
    b->used = 1;
    b->type = blob.type;
    gkb_copy(b->uuid, blob.uuid, 16);
    gkb_copy(b->device_key, blob.device_key, GKB_KEY);
    gkb_zero(&blob, sizeof blob);
    return h;
}
static inline void gkb_unload_slot(struct gkb_state *st, int h) {
    gkb_zero(&st->bag[h], sizeof st->bag[h]);
    if (st->system == h) st->system = -1;
}
/* make a COPY of slot h the system bag (keybagd unloads the handle it loaded by right after aks_set_system) */
static inline int gkb_set_system_slot(struct gkb_state *st, int h) {
    if (h == st->system) return GKB_OK;
    const int old = st->system;
    if (old >= 0) gkb_unload_slot(st, old);
    const int copy = gkb_free_slot(st);
    if (copy < 0) return GKB_NO_RESOURCES;
    st->bag[copy] = st->bag[h];
    st->system = copy;
    return GKB_OK;
}

/* ---- systembag.kb: a binary property list { KeyBagKeys: <blob> }, as keybagd's load_binary_dict reads it ---- */
#define GKB_PLIST_MAX 1200u
static inline size_t gkb_plist_encode(const struct gkb_blob *blob, uint8_t *out, size_t cap) {
    static const char head[8] = {'b', 'p', 'l', 'i', 's', 't', '0', '0'};
    static const char key[10] = {'K', 'e', 'y', 'B', 'a', 'g', 'K', 'e', 'y', 's'};
    const size_t blen = sizeof *blob;
    size_t n = 0, off[3];
    if (cap < GKB_PLIST_MAX) return 0;
    gkb_copy(out, head, 8); n = 8;
    off[0] = n; out[n++] = 0xd1; out[n++] = 1; out[n++] = 2;                 /* dict, 1 entry: key ref 1, value ref 2 */
    off[1] = n; out[n++] = 0x5a; gkb_copy(out + n, key, 10); n += 10;        /* ASCII string, 10 chars */
    off[2] = n; out[n++] = 0x4f; out[n++] = 0x11;                            /* data, length as a 2-byte int */
    out[n++] = (uint8_t)(blen >> 8); out[n++] = (uint8_t)blen;
    gkb_copy(out + n, blob, blen); n += blen;
    const size_t table = n;
    for (unsigned i = 0; i < 3; ++i) { out[n++] = (uint8_t)(off[i] >> 8); out[n++] = (uint8_t)off[i]; }
    for (unsigned i = 0; i < 6; ++i) out[n++] = 0;                            /* trailer: unused, sort version */
    out[n++] = 2; out[n++] = 1;                                               /* offset int size, object ref size */
    for (unsigned i = 0; i < 7; ++i) out[n++] = 0;
    out[n++] = 3;                                                             /* objects */
    for (unsigned i = 0; i < 8; ++i) out[n++] = 0;                            /* top object 0 */
    for (unsigned i = 0; i < 8; ++i) out[n++] = (uint8_t)((uint64_t)table >> (8 * (7 - i)));
    return n;
}
/* Find our blob in a systembag.kb whatever wrote it (this encoder, or keybagd after aks_save_bag): the data bytes are
 * contiguous in a binary plist. Returns the offset, or -1. */
static inline long gkb_plist_find(const uint8_t *file, size_t len) {
    const struct gkb_blob *probe;
    for (size_t i = 0; i + sizeof *probe <= len; ++i) {
        uint32_t magic = (uint32_t)file[i] | (uint32_t)file[i + 1] << 8 | (uint32_t)file[i + 2] << 16 | (uint32_t)file[i + 3] << 24;
        if (magic != GKB_MAGIC) continue;
        uint32_t version = (uint32_t)file[i + 4] | (uint32_t)file[i + 5] << 8, classes = (uint32_t)file[i + 12] | (uint32_t)file[i + 13] << 8;
        if (version == GKB_VERSION && classes == GKB_CLASSES && !file[i + 6] && !file[i + 7] && !file[i + 14] && !file[i + 15]) return (long)i;
    }
    return -1;
}

/* ---- the system bag from systembag.kb, read by the kernel itself; never makes or writes anything ----
 * Also used with the keybag OFF (rollback): keys wrapped during a guamp-aks-keybag=1 boot stay readable. */
static inline int gkb_read_system(struct gkb_state *st, const struct gkb_io *io) {
    if (st->system >= 0) return GKB_OK;
    if (st->load_failed || io->file_exists(io->ctx) != 0) return GKB_NOT_FOUND;
    static uint8_t file[4096];
    size_t len = 0;
    if (io->file_read(io->ctx, file, sizeof file, &len) == 0) {
        long at = gkb_plist_find(file, len);
        int h = at >= 0 ? gkb_load(st, file + at, sizeof(struct gkb_blob)) : -1;
        gkb_zero(file, sizeof file);
        if (h > 0 && gkb_set_system_slot(st, h) == GKB_OK) {
            gkb_unload_slot(st, h);
            st->from_file = 1;
            io->log(io->ctx, "system bag read from systembag.kb by the kernel", st->system);
            return GKB_OK;
        }
    }
    st->load_failed = 1;
    io->log(io->ctx, "systembag.kb present but not ours or damaged: left untouched", (long)len);
    return GKB_NOT_FOUND;
}

/* ---- the system bag when nobody loaded one: read systembag.kb in the kernel, or make it ---- */
static inline int gkb_ensure_system(struct gkb_state *st, const struct gkb_io *io) {
    if (gkb_read_system(st, io) == GKB_OK) return GKB_OK;
    const int have_file = io->file_exists(io->ctx) == 0;
    const int h = gkb_create(st, io, 0);
    if (h < 0) return GKB_NO_RESOURCES;
    st->system = h;
    if (!have_file) {
        struct gkb_blob blob;
        static uint8_t plist[GKB_PLIST_MAX];
        gkb_save(&st->bag[h], &blob);
        size_t n = gkb_plist_encode(&blob, plist, sizeof plist);
        int rc = n ? io->file_write(io->ctx, plist, n) : -1;
        gkb_zero(&blob, sizeof blob);
        gkb_zero(plist, sizeof plist);
        st->ephemeral = rc != 0;
        io->log(io->ctx, rc == 0 ? "new system bag written to systembag.kb" : "new system bag NOT written (lost at the next boot)", rc);
    } else {
        st->ephemeral = 1;
        io->log(io->ctx, "new system bag for this boot only: the existing systembag.kb is kept, its keys are not readable", h);
    }
    return GKB_OK;
}

/* aks_get_system (selector 14). keybagd (device right), the first time, with a systembag.kb on disk: kIOReturnNotFound,
 * so that keybagd --init loads it itself (aks_load_bag 6, aks_set_system 5, aks_unload_bag 4: keybagd 21H461
 * load_devicebag 0x10001643c). Otherwise the system bag, made or read by the kernel if needed. */
static inline int gkb_get_system(struct gkb_state *st, const struct gkb_io *io, uint32_t rights, uint64_t *handle) {
    if (st->system < 0 && (rights & GKB_RIGHT_DEVICE) && !st->not_found_given && !st->load_failed &&
        io->file_exists(io->ctx) == 0) {
        st->not_found_given = 1;
        io->log(io->ctx, "aks_get_system: no system bag yet, keybagd loads systembag.kb", 0);
        return GKB_NOT_FOUND;
    }
    int rc = gkb_ensure_system(st, io);
    if (rc == GKB_OK) *handle = (uint64_t)st->system;
    return rc;
}

/* aks_load_bag (6): device right */
static inline int gkb_load_bag(struct gkb_state *st, const struct gkb_io *io, uint32_t rights, const uint8_t *bytes, size_t len, uint64_t *handle) {
    if (!(rights & GKB_RIGHT_DEVICE)) return GKB_NOT_PRIVILEGED;
    int h = gkb_load(st, bytes, len);
    if (h == -2) return GKB_NO_RESOURCES;
    if (h < 0) {
        if (st->system < 0) st->load_failed = 1;
        io->log(io->ctx, "aks_load_bag: not a guamp bag or damaged", (long)len);
        return GKB_BAD_ARGUMENT;
    }
    *handle = (uint64_t)h;
    return GKB_OK;
}
/* aks_set_system (5): device right. A handle that is not a bag of ours changes nothing (as before the keybag). */
static inline int gkb_set_system(struct gkb_state *st, const struct gkb_io *io, uint32_t rights, uint64_t handle) {
    if (handle == 0 || handle >= GKB_BAGS || !st->bag[handle].used) return GKB_OK;
    if (!(rights & GKB_RIGHT_DEVICE)) return GKB_NOT_PRIVILEGED;
    int rc = gkb_set_system_slot(st, (int)handle);
    if (rc == GKB_OK) {
        st->from_file = 1;
        st->ephemeral = 0;
        io->log(io->ctx, "system bag set by keybagd", st->system);
    }
    return rc;
}
/* aks_unload_bag (4): a bag of ours is dropped (device right); the system bag stays; other handles: success, as before. */
static inline int gkb_unload_bag(struct gkb_state *st, uint32_t rights, uint64_t handle) {
    if (handle == 0 || handle >= GKB_BAGS || !st->bag[handle].used) return GKB_OK;
    if (!(rights & GKB_RIGHT_DEVICE)) return GKB_NOT_PRIVILEGED;
    if ((int)handle == st->system) return GKB_NOT_PERMITTED;
    gkb_unload_slot(st, (int)handle);
    return GKB_OK;
}
/* aks_save_bag (3): device right */
static inline int gkb_save_bag(struct gkb_state *st, uint32_t rights, uint64_t handle, struct gkb_blob *blob) {
    if (!(rights & GKB_RIGHT_DEVICE)) return GKB_NOT_PRIVILEGED;
    struct gkb_bag *b = gkb_bag_for(st, handle);
    if (!b) return GKB_NOT_FOUND;
    gkb_save(b, blob);
    return GKB_OK;
}

/* aks_wrap_key (10): out = len + 8 bytes */
static inline int gkb_wrap(struct gkb_state *st, const struct gkb_io *io, uint32_t rights, uint64_t handle, uint64_t key_class,
                           const uint8_t *in, size_t len, uint8_t *out) {
    if (!(rights & (GKB_RIGHT_DEVICE | GKB_RIGHT_KEYCHAIN))) return GKB_NOT_PRIVILEGED;
    const uint64_t cls = key_class & 0xff;        /* above the class: libaks flags */
    if (cls >= GKB_CLASSES || len < 16 || len > 64 || len % 8) return GKB_BAD_ARGUMENT;
    if (handle == 0 && gkb_ensure_system(st, io) != GKB_OK) return GKB_NO_RESOURCES;
    struct gkb_bag *b = gkb_bag_for(st, handle);
    if (!b) return GKB_NOT_FOUND;
    gkb_kw_wrap(b->class_key[cls], in, len, out);
    return GKB_OK;
}
/* aks_unwrap_key (11): out = len - 8 bytes. A blob of the old stand-in (GUAMPAKS tag) is read when legacy_ok. */
static inline int gkb_unwrap(struct gkb_state *st, const struct gkb_io *io, uint32_t rights, uint64_t handle, uint64_t key_class,
                             const uint8_t *in, size_t len, uint8_t *out, int legacy_ok, int *was_legacy) {
    *was_legacy = 0;
    if (!(rights & (GKB_RIGHT_DEVICE | GKB_RIGHT_KEYCHAIN))) return GKB_NOT_PRIVILEGED;
    const uint64_t cls = key_class & 0xff;
    if (cls >= GKB_CLASSES || len <= 8 || len > 72) return GKB_BAD_ARGUMENT;
    if (handle == 0 && gkb_ensure_system(st, io) != GKB_OK) return GKB_NO_RESOURCES;
    struct gkb_bag *b = gkb_bag_for(st, handle);
    if (b && len % 8 == 0 && len >= 24 && gkb_kw_unwrap(b->class_key[cls], in, len, out)) return GKB_OK;
    if (legacy_ok && !gkb_differ(in, gkb_legacy_tag, 8)) {
        gkb_copy(out, in + 8, len - 8);
        *was_legacy = 1;
        return GKB_OK;
    }
    return b ? GKB_NOT_PERMITTED : GKB_NOT_FOUND;
}

/* Keybag OFF (rollback to the plain stand-in): a key that is not a GUAMPAKS blob may have been wrapped during a
 * guamp-aks-keybag=1 boot. Unwrap it with the bag in systembag.kb if there is one; nothing is made or written.
 * GKB_NOT_FOUND: not ours to answer (the stand-in fails it as before). */
static inline int gkb_rescue_unwrap(struct gkb_state *st, const struct gkb_io *io, uint64_t key_class,
                                    const uint8_t *in, size_t len, uint8_t *out) {
    const uint64_t cls = key_class & 0xff;
    if (cls >= GKB_CLASSES || len < 24 || len > 72 || len % 8 || !gkb_differ(in, gkb_legacy_tag, 8)) return GKB_NOT_FOUND;
    if (gkb_read_system(st, io) != GKB_OK) return GKB_NOT_FOUND;
    return gkb_kw_unwrap(st->bag[st->system].class_key[cls], in, len, out) ? GKB_OK : GKB_NOT_FOUND;
}

#endif
