// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of package/GuampKeybagCore.h (guamp-aks-keybag=1): AES-256 (FIPS-197 C.3), AES key wrap (RFC 3394 4.6), and
// the keybagd/securityd flows across two simulated boots with systembag.kb in a memory "file".
// Usage: guamp_keybag_host [plist-out]   (writes the systembag.kb it made, for the plistlib check in run-keybag-tests.sh)
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../GuampKeybagCore.h"

static uint8_t disk[4096];
static size_t disk_len;
static int disk_present, disk_readonly, writes, logs;
static uint32_t rng = 0x12345678;

static int f_exists(void *c) { (void)c; return disk_present ? 0 : -1; }
static int f_read(void *c, uint8_t *b, size_t cap, size_t *len) {
    (void)c;
    if (!disk_present || disk_len > cap) return -1;
    memcpy(b, disk, disk_len); *len = disk_len; return 0;
}
static int f_write(void *c, const uint8_t *b, size_t len) {
    (void)c;
    if (disk_present || disk_readonly || len > sizeof disk) return -1;
    memcpy(disk, b, len); disk_len = len; disk_present = 1; ++writes; return 0;
}
static void f_random(void *c, void *b, size_t len) {
    (void)c;
    for (size_t i = 0; i < len; ++i) { rng = rng * 1103515245u + 12345u; ((uint8_t *)b)[i] = (uint8_t)(rng >> 16); }
}
static void f_log(void *c, const char *e, long v) { (void)c; ++logs; printf("  log: %s (%ld)\n", e, v); }
static const struct gkb_io io = {NULL, f_exists, f_read, f_write, f_random, f_log};

static void hex(const char *s, uint8_t *out) { for (size_t i = 0; s[2 * i]; ++i) sscanf(s + 2 * i, "%2hhx", &out[i]); }

/* what keybagd --init does on 21H461 (load_devicebag 0x10001643c), with our answers */
static uint64_t keybagd_init(struct gkb_state *st) {
    uint64_t h = 99;
    int rc = gkb_get_system(st, &io, GKB_RIGHT_DEVICE, &h);
    if (rc == GKB_NOT_FOUND) {
        assert(disk_present);
        long at = gkb_plist_find(disk, disk_len);                       /* stands for CFPropertyList + KeyBagKeys */
        assert(at >= 0);
        uint64_t loaded = 0;
        rc = gkb_load_bag(st, &io, GKB_RIGHT_DEVICE, disk + at, sizeof(struct gkb_blob), &loaded);
        if (rc == GKB_OK) {
            assert(gkb_set_system(st, &io, GKB_RIGHT_DEVICE, loaded) == GKB_OK);
            assert(gkb_unload_bag(st, GKB_RIGHT_DEVICE, loaded) == GKB_OK);  /* keybagd drops its own handle at once */
        }
        rc = gkb_get_system(st, &io, GKB_RIGHT_DEVICE, &h);              /* the check after it (no FATAL if it fails) */
    }
    assert(rc == GKB_OK && h > 0 && h < GKB_BAGS);
    return h;
}

int main(int argc, char **argv) {
    uint8_t k[32], p[16], c[16], kek[32], data[32], exp[40], out[40], back[32];
    struct gkb_aes aes;

    /* FIPS-197 C.3 */
    hex("000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f", k);
    hex("00112233445566778899aabbccddeeff", p);
    hex("8ea2b7ca516745bfeafc49904b496089", c);
    gkb_aes_key(&aes, k);
    uint8_t s[16]; memcpy(s, p, 16);
    gkb_aes_encrypt(&aes, s); assert(!memcmp(s, c, 16));
    gkb_aes_decrypt(&aes, s); assert(!memcmp(s, p, 16));
    puts("PASS aes256 fips197-c3");

    /* RFC 3394 4.6: 256 bits of key data with a 256-bit KEK */
    hex("000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f", kek);
    hex("00112233445566778899aabbccddeeff000102030405060708090a0b0c0d0e0f", data);
    hex("28c9f404c4b810f4cbccb35cfb87f8263f5786e2d80ed326cbc7f0e71a99f43bfb988b9b7a02dd21", exp);
    assert(gkb_kw_wrap(kek, data, 32, out)); assert(!memcmp(out, exp, 40));
    assert(gkb_kw_unwrap(kek, out, 40, back)); assert(!memcmp(back, data, 32));
    out[17] ^= 1; assert(!gkb_kw_unwrap(kek, out, 40, back));
    for (int i = 0; i < 32; ++i) assert(back[i] == 0);
    puts("PASS rfc3394 4.6 + tamper");

    /* boot 1: no systembag.kb. keybagd gets a bag the kernel made and wrote. */
    static struct gkb_state st;
    gkb_init(&st);
    uint64_t h1 = keybagd_init(&st);
    assert(disk_present && writes == 1 && !st.ephemeral);
    uint8_t item_key[32], wrapped[40], wrapped7[40], plain[32];
    f_random(NULL, item_key, 32);
    assert(gkb_wrap(&st, &io, GKB_RIGHT_KEYCHAIN, 0, 10, item_key, 32, wrapped) == GKB_OK);
    assert(gkb_wrap(&st, &io, GKB_RIGHT_KEYCHAIN, 0, 7 | 0x100, item_key, 32, wrapped7) == GKB_OK);   /* flags above the class */
    assert(memcmp(wrapped, wrapped7, 40));                                        /* class keys differ */
    assert(gkb_wrap(&st, &io, GKB_RIGHT_KEYCHAIN, h1, 10, item_key, 32, out) == GKB_OK && !memcmp(out, wrapped, 40));
    int legacy = 0;
    assert(gkb_unwrap(&st, &io, GKB_RIGHT_KEYCHAIN, 0, 10, wrapped, 40, plain, 1, &legacy) == GKB_OK && !legacy && !memcmp(plain, item_key, 32));
    assert(gkb_unwrap(&st, &io, GKB_RIGHT_KEYCHAIN, 0, 11, wrapped, 40, plain, 1, &legacy) == GKB_NOT_PERMITTED);
    /* rights */
    assert(gkb_wrap(&st, &io, 0, 0, 10, item_key, 32, out) == GKB_NOT_PRIVILEGED);
    assert(gkb_unwrap(&st, &io, 0, 0, 10, wrapped, 40, plain, 1, &legacy) == GKB_NOT_PRIVILEGED);
    struct gkb_blob blob;
    assert(gkb_save_bag(&st, GKB_RIGHT_KEYCHAIN, 0, &blob) == GKB_NOT_PRIVILEGED);
    assert(gkb_save_bag(&st, GKB_RIGHT_DEVICE, 0, &blob) == GKB_OK);
    uint64_t hx;
    assert(gkb_load_bag(&st, &io, GKB_RIGHT_KEYCHAIN, (const uint8_t *)&blob, sizeof blob, &hx) == GKB_NOT_PRIVILEGED);
    assert(gkb_unload_bag(&st, GKB_RIGHT_DEVICE, h1) == GKB_NOT_PERMITTED);       /* the system bag stays */
    assert(gkb_unload_bag(&st, 0, 5) == GKB_OK);                                  /* not a bag: success, as before */
    /* bad arguments */
    assert(gkb_wrap(&st, &io, GKB_RIGHT_KEYCHAIN, 0, 10, item_key, 20, out) == GKB_BAD_ARGUMENT);
    assert(gkb_wrap(&st, &io, GKB_RIGHT_KEYCHAIN, 0, 40, item_key, 32, out) == GKB_BAD_ARGUMENT);
    assert(gkb_wrap(&st, &io, GKB_RIGHT_KEYCHAIN, 6, 10, item_key, 32, out) == GKB_NOT_FOUND);
    /* migration: a blob of the old stand-in still reads (and only with legacy on) */
    uint8_t old[40]; memcpy(old, gkb_legacy_tag, 8); memcpy(old + 8, item_key, 32);
    assert(gkb_unwrap(&st, &io, GKB_RIGHT_KEYCHAIN, 0, 6, old, 40, plain, 1, &legacy) == GKB_OK && legacy && !memcmp(plain, item_key, 32));
    assert(gkb_unwrap(&st, &io, GKB_RIGHT_KEYCHAIN, 0, 6, old, 40, plain, 0, &legacy) == GKB_NOT_PERMITTED);
    puts("PASS boot 1: bag made + written, wrap/unwrap, rights, legacy");
    if (argc > 1) { FILE *f = fopen(argv[1], "wb"); assert(f); fwrite(disk, 1, disk_len, f); fclose(f); }

    /* boot 2: keybagd loads systembag.kb itself; the keys of boot 1 are back */
    gkb_init(&st);
    uint64_t h2 = keybagd_init(&st);
    assert(st.not_found_given && st.from_file && !st.ephemeral && writes == 1);
    assert(gkb_unwrap(&st, &io, GKB_RIGHT_KEYCHAIN, 0, 10, wrapped, 40, plain, 0, &legacy) == GKB_OK && !memcmp(plain, item_key, 32));
    assert(gkb_unwrap(&st, &io, GKB_RIGHT_KEYCHAIN, h2, 10, wrapped, 40, plain, 0, &legacy) == GKB_OK);
    int used = 0; for (unsigned i = 0; i < GKB_BAGS; ++i) used += st.bag[i].used;
    assert(used == 1);                                                            /* only the system copy remains */
    puts("PASS boot 2: keybagd --init loads systembag.kb (NotFound, load, set_system, unload)");

    /* boot 3: keybagd never asks (no entitlement seen, boot task skipped): securityd's first unwrap reads the file */
    gkb_init(&st);
    assert(gkb_unwrap(&st, &io, GKB_RIGHT_KEYCHAIN, 0, 10, wrapped, 40, plain, 0, &legacy) == GKB_OK && !memcmp(plain, item_key, 32));
    uint64_t h3;
    assert(gkb_get_system(&st, &io, GKB_RIGHT_DEVICE, &h3) == GKB_OK && h3 == (uint64_t)st.system);   /* no NotFound once a bag is there */
    assert(writes == 1);
    puts("PASS boot 3: the kernel reads systembag.kb itself");

    /* rollback: keybag OFF, plain stand-in; a key wrapped during a keybag boot is unwrapped from systembag.kb, read only */
    gkb_init(&st);
    assert(gkb_rescue_unwrap(&st, &io, 10, wrapped, 40, plain) == GKB_OK && !memcmp(plain, item_key, 32));
    assert(gkb_rescue_unwrap(&st, &io, 11, wrapped, 40, plain) == GKB_NOT_FOUND);       /* wrong class: integrity fails */
    assert(gkb_rescue_unwrap(&st, &io, 6, old, 40, plain) == GKB_NOT_FOUND);            /* GUAMPAKS: the stand-in's own */
    disk_present = 0;
    gkb_init(&st);
    assert(gkb_rescue_unwrap(&st, &io, 10, wrapped, 40, plain) == GKB_NOT_FOUND && !disk_present && writes == 1);
    disk_present = 1;
    puts("PASS rollback: keybag off still unwraps keys of a keybag boot, never writes");

    /* boot 4: damaged file: keybagd's load fails, the second ask gets a bag for this boot only; the file is kept */
    uint8_t saved[4096]; size_t saved_len = disk_len; memcpy(saved, disk, disk_len);
    long at = gkb_plist_find(disk, disk_len); assert(at >= 0);
    disk[at + 40] ^= 0xff;                                                        /* inside the device key */
    gkb_init(&st);
    uint64_t h4 = keybagd_init(&st);
    assert(st.ephemeral && st.load_failed && writes == 1 && h4 > 0);
    assert(gkb_unwrap(&st, &io, GKB_RIGHT_KEYCHAIN, 0, 10, wrapped, 40, plain, 0, &legacy) == GKB_NOT_PERMITTED);
    assert(gkb_wrap(&st, &io, GKB_RIGHT_KEYCHAIN, 0, 10, item_key, 32, out) == GKB_OK);   /* the keychain still works */
    memcpy(disk, saved, saved_len);
    puts("PASS boot 4: damaged systembag.kb kept, bag for this boot, no failure");

    /* boot 5: no file and /private/var/keybags not writable: a bag for this boot, keychain works */
    disk_present = 0; disk_readonly = 1;
    gkb_init(&st);
    keybagd_init(&st);
    assert(st.ephemeral && !disk_present);
    puts("PASS boot 5: unwritable keybags directory");

    /* a different blob loaded twice is one slot; slots run out cleanly */
    disk_readonly = 0;
    gkb_init(&st);
    keybagd_init(&st);
    struct gkb_bag other = st.bag[st.system];
    other.uuid[0] ^= 1;
    gkb_save(&other, &blob);
    uint64_t a = 0, b = 0;
    assert(gkb_load_bag(&st, &io, GKB_RIGHT_DEVICE, (const uint8_t *)&blob, sizeof blob, &a) == GKB_OK);
    assert(gkb_load_bag(&st, &io, GKB_RIGHT_DEVICE, (const uint8_t *)&blob, sizeof blob, &b) == GKB_OK && a == b);
    assert(gkb_load_bag(&st, &io, GKB_RIGHT_DEVICE, (const uint8_t *)&blob, 100, &b) == GKB_BAD_ARGUMENT);
    puts("PASS load twice / wrong size");
    printf("blob %zu bytes, systembag.kb %zu bytes, %d log lines\n", sizeof(struct gkb_blob), disk_len, logs);
    return 0;
}
