// SPDX-License-Identifier: BSD-2-Clause-Patent
// HCI over H4 (UART) framing and the decoders the Bluetooth M0 probe needs: Command Complete / Command Status, Read
// Local Version Information (Core spec vol 4 part E 7.4.1) and the Qualcomm EDL version reply (btqca.c study: VSC
// 0xFC00 with sub-command 0x19 answered by a vendor event 0xFF carrying {cresp 0x00, rtype 0x02} + struct rome_version
// {le32 product_id, le16 patch_ver, le16 rom_ver, le32 soc_id}; newer chips wrap it in Command Complete or prefix one
// byte). Pure code, no allocation; host-tested by tests/bt_host.cpp.
#ifndef BT_HCI_H
#define BT_HCI_H
#include <stdint.h>
#include <stddef.h>
#include "GuampBtBoard.h"

#define BT_H4_CMD 0x01
#define BT_H4_ACL 0x02
#define BT_H4_SCO 0x03
#define BT_H4_EVT 0x04
#define BT_EVT_CMD_COMPLETE 0x0e
#define BT_EVT_CMD_STATUS 0x0f
#define BT_EVT_VENDOR 0xff
#define BT_OP_RESET 0x0c03
#define BT_OP_READ_LOCAL_VERSION 0x1001
#define BT_OP_QCA_EDL 0xfc00
#define BT_QCA_EDL_VER_REQ 0x19
#define BT_QCA_POWERON_PULSE 0xfc               // btqca.h QCA_WCN3990_POWERON_PULSE (sent at 115200)
#define BT_QCA_POWEROFF_PULSE 0xc0              // QCA_WCN3990_POWEROFF_PULSE (sent at 2400)

// H4 command packet: 01, opcode (LE), parameter length, parameters. Returns the packet length, 0 if cap is too small.
static inline size_t bt_h4_cmd(uint8_t *out, size_t cap, uint16_t opcode, const uint8_t *params, uint8_t plen) {
    if (cap < (size_t)4 + plen) return 0;
    out[0] = BT_H4_CMD; out[1] = (uint8_t)(opcode & 0xff); out[2] = (uint8_t)(opcode >> 8); out[3] = plen;
    for (unsigned i = 0; i < plen; ++i) out[4 + i] = params[i];
    return (size_t)4 + plen;
}

struct bt_event {uint8_t code, len; const uint8_t *p;};
// Next complete H4 event in buf[*pos..len). Bytes that cannot start a packet are skipped (counted in *skipped); ACL
// and SCO packets are stepped over whole. 1 = event in *ev and *pos past it, 0 = need more bytes (*pos at the start of
// the incomplete packet).
static inline int bt_h4_next(const uint8_t *buf, size_t len, size_t *pos, struct bt_event *ev, unsigned *skipped) {
    size_t at = *pos;
    while (at < len) {
        const uint8_t t = buf[at];
        if (t == BT_H4_EVT) {
            if (len - at < 3) break;
            const uint8_t plen = buf[at + 2];
            if (len - at < (size_t)3 + plen) break;
            ev->code = buf[at + 1]; ev->len = plen; ev->p = buf + at + 3;
            *pos = at + 3 + plen;
            return 1;
        }
        if (t == BT_H4_ACL) {
            if (len - at < 5) break;
            const size_t dlen = (size_t)buf[at + 3] | (size_t)buf[at + 4] << 8;
            if (len - at < 5 + dlen) break;
            at += 5 + dlen;
            continue;
        }
        if (t == BT_H4_SCO) {
            if (len - at < 4) break;
            const size_t dlen = buf[at + 3];
            if (len - at < 4 + dlen) break;
            at += 4 + dlen;
            continue;
        }
        if (skipped) *skipped += 1;
        at += 1;
    }
    *pos = at;
    return 0;
}

static inline uint16_t bt_le16(const uint8_t *p) {return (uint16_t)(p[0] | p[1] << 8);}
static inline uint32_t bt_le32(const uint8_t *p) {return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;}

// Command Complete: num_cmds, opcode, return parameters (first = status for most commands). 0 on success.
static inline int bt_cmd_complete(const struct bt_event *ev, uint16_t *opcode, const uint8_t **ret, uint8_t *rlen) {
    if (ev->code != BT_EVT_CMD_COMPLETE || ev->len < 3) return -1;
    *opcode = bt_le16(ev->p + 1); *ret = ev->p + 3; *rlen = (uint8_t)(ev->len - 3);
    return 0;
}
// Command Status: status, num_cmds, opcode. 0 on success.
static inline int bt_cmd_status(const struct bt_event *ev, uint16_t *opcode, uint8_t *status) {
    if (ev->code != BT_EVT_CMD_STATUS || ev->len < 4) return -1;
    *status = ev->p[0]; *opcode = bt_le16(ev->p + 2);
    return 0;
}

struct bt_local_version {uint8_t status, hci_ver, lmp_ver; uint16_t hci_rev, manufacturer, lmp_subver;};
static inline int bt_decode_local_version(const struct bt_event *ev, struct bt_local_version *v) {
    uint16_t op; const uint8_t *r; uint8_t n;
    if (bt_cmd_complete(ev, &op, &r, &n) || op != BT_OP_READ_LOCAL_VERSION || n < 1) return -1;
    v->status = r[0];
    if (v->status) return 1;
    if (n < 9) return -1;
    v->hci_ver = r[1]; v->hci_rev = bt_le16(r + 2); v->lmp_ver = r[4]; v->manufacturer = bt_le16(r + 5);
    v->lmp_subver = bt_le16(r + 7);
    return 0;
}
static inline const char *bt_hci_version_name(uint8_t v) {
    static const char *const k[] = {"1.0b", "1.1", "1.2", "2.0", "2.1", "3.0", "4.0", "4.1", "4.2", "5.0", "5.1", "5.2",
                                     "5.3", "5.4", "6.0"};
    return v < sizeof k / sizeof k[0] ? k[v] : "?";
}
static inline const char *bt_manufacturer_name(uint16_t id) {   // Bluetooth SIG company identifiers
    return id == 0x001d ? "Qualcomm" : id == 0x000a ? "Qualcomm Technologies International" :
           id == 0x0045 ? "Atheros" : id == 0x000f ? "Broadcom" : id == 0x005d ? "Realtek" : "other";
}

// form: 1 vendor event, 2 vendor event with one leading byte, 3 Command Complete of 0xFC00.
struct bt_qca_version {uint32_t product_id, soc_id, soc_ver; uint16_t patch_ver, rom_ver; uint8_t rom, form;};
static inline int bt_qca_edl_at(const uint8_t *p, unsigned n, struct bt_qca_version *v) {
    if (n < 14 || p[0] != 0x00 || (p[1] != 0x02 && p[1] != BT_QCA_EDL_VER_REQ)) return -1;
    v->product_id = bt_le32(p + 2); v->patch_ver = bt_le16(p + 6); v->rom_ver = bt_le16(p + 8); v->soc_id = bt_le32(p + 10);
    v->soc_ver = v->soc_id << 16 | v->rom_ver;
    v->rom = (uint8_t)(((v->soc_ver & 0xf00) >> 4) | (v->soc_ver & 0xf));   // btqca.c qca_uart_setup
    return 0;
}
static inline int bt_decode_qca_version(const struct bt_event *ev, struct bt_qca_version *v) {
    if (ev->code == BT_EVT_VENDOR) {
        if (!bt_qca_edl_at(ev->p, ev->len, v)) { v->form = 1; return 0; }
        if (ev->len > 0 && !bt_qca_edl_at(ev->p + 1, ev->len - 1u, v)) { v->form = 2; return 0; }
        return -1;
    }
    uint16_t op; const uint8_t *r; uint8_t n;
    if (bt_cmd_complete(ev, &op, &r, &n) || op != BT_OP_QCA_EDL) return -1;
    for (unsigned k = 0; k < 2 && k < n; ++k)
        if (!bt_qca_edl_at(r + k, n - k, v)) { v->form = 3; return 0; }
    return -1;
}
// btqca.c file names for a WCN3990-class soc_ver ("crbtfw%02x.tlv", "crnv%02x.bin"), lower-case hex.
static inline void bt_qca_names(uint8_t rom, char tlv[16], char nvm[16]) {
    static const char hx[] = "0123456789abcdef";
    const char *a = "crbtfw", *b = "crnv";
    unsigned i = 0, j = 0;
    for (; a[i]; ++i) tlv[i] = a[i];
    tlv[i++] = hx[rom >> 4]; tlv[i++] = hx[rom & 15]; tlv[i++] = '.'; tlv[i++] = 't'; tlv[i++] = 'l'; tlv[i++] = 'v'; tlv[i] = 0;
    for (; b[j]; ++j) nvm[j] = b[j];
    nvm[j++] = hx[rom >> 4]; nvm[j++] = hx[rom & 15]; nvm[j++] = '.'; nvm[j++] = 'b'; nvm[j++] = 'i'; nvm[j++] = 'n'; nvm[j] = 0;
}
// Files of the guamp bluetooth_a partition whose TLV header (product id, ROM build) matches the controller: bit k set
// for kBtFirmware[k].
static inline unsigned bt_fw_matches(const struct bt_qca_version *v) {
    unsigned m = 0;
    for (unsigned k = 0; k < sizeof kBtFirmware / sizeof kBtFirmware[0]; ++k)
        if (kBtFirmware[k].product == (v->product_id & 0xffff) && kBtFirmware[k].rom_build == v->rom_ver) m |= 1u << k;
    return m;
}

// Hex dump into out (3 chars per byte, at most max bytes, "..." when cut). Returns the characters written.
static inline unsigned bt_hex(char *out, unsigned cap, const uint8_t *p, size_t n, unsigned max) {
    static const char hx[] = "0123456789abcdef";
    unsigned o = 0;
    for (size_t i = 0; i < n && i < max && o + 4 < cap; ++i) {
        if (i) out[o++] = ' ';
        out[o++] = hx[p[i] >> 4]; out[o++] = hx[p[i] & 15];
    }
    if (n > max && o + 4 < cap) { out[o++] = '.'; out[o++] = '.'; out[o++] = '.'; }
    if (cap) out[o < cap ? o : cap - 1] = 0;
    return o;
}
#endif
