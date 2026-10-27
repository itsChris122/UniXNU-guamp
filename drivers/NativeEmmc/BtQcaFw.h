// SPDX-License-Identifier: BSD-2-Clause-Patent
// Qualcomm WCN39xx rampatch / NVM download, pure code (no allocation, no libc), host-tested by tests/bt_fw_host.cpp
// against the files of the guamp bluetooth_a partition. Behaviour follows the study of mainline Linux
// drivers/bluetooth/btqca.c + btqca.h + hci_qca.c (no code copied):
//   names     qca_uart_setup: rom_ver = ((soc_ver & 0xf00) >> 4) | (soc_ver & 0xf); WCN3950 (product 0x0f, the
//             "Comanche" family of the guamp controller) "cmbtfw%02x.tlv" + "cmnv%02x%s.bin" (variant "t" for SoC id
//             0x40074130, "s" for 0x40075130, else none); WCN3990/3991/3998 "crbtfw%02x.tlv" + "crnv%02x%s.bin"
//             ("u" for SoC id 0x40014320)
//   patch     qca_tlv_check_data TLV_TYPE_PATCH: le32 type_len, then struct tlv_type_patch; download_mode decides
//             which segments the controller acks (QCA_SKIP_EVT_VSE / _VSE_CC: only the last one)
//   nvm       qca_tlv_check_data TLV_TYPE_NVM: type 4 wraps the sets (only the first, type 2, is edited); tag 17
//             (HCI) data[0] |= 0x80 (in-band sleep) and the UART baud code in data[2] (data[1] from WCN3991 on);
//             tag 27 (deep sleep) data[0] |= 0x01; tag 2 (BD address, 6 bytes) is remembered (qca_check_bdaddr)
//   segments  qca_download_firmware: 243-byte pieces of the whole file, VSC 0xFC00 {0x1E, len, data}; acked by a
//             vendor event {0x00, 0x04, result} (before WCN3991) unless the mode skips it (the last one always)
//   baud      qca_set_baudrate: VSC 0xFC48 {code}, enum qca_baudrate
#ifndef BT_QCA_FW_H
#define BT_QCA_FW_H
#include <stdint.h>
#include <stddef.h>
#include "BtHci.h"

#define BT_OP_QCA_SET_BAUD 0xfc48
#define BT_OP_READ_BD_ADDR 0x1009
#define BT_QCA_TLV_REQ 0x1e                  // EDL_PATCH_TLV_REQ_CMD
#define BT_QCA_TLV_SEG_MAX 243u              // MAX_SIZE_PER_TLV_SEGMENT
#define BT_QCA_EDL_CMD_RES 0x00              // EDL_CMD_REQ_RES_EVT
#define BT_QCA_EDL_TLV_RES 0x04              // EDL_TVL_DNLD_RES_EVT
#define BT_QCA_SKIP_NONE 0
#define BT_QCA_SKIP_VSE 1
#define BT_QCA_SKIP_CC 2
#define BT_QCA_SKIP_VSE_CC 3
#define BT_QCA_TAG_BD_ADDR 2
#define BT_QCA_TAG_HCI 17
#define BT_QCA_TAG_DEEP_SLEEP 27
#define BT_QCA_PRODUCT_WCN3950 0x000fu       // "cm" files (bluetooth_a header product id)
#define BT_QCA_SOC_WCN3950_T 0x40074130u
#define BT_QCA_SOC_WCN3950_S 0x40075130u
#define BT_QCA_SOC_WCN3991 0x40014320u
#define BT_QCA_SEG_PKT_MAX (4u + 2u + BT_QCA_TLV_SEG_MAX)   // H4 01, opcode, length, {0x1E, len, 243 bytes}

// enum qca_baudrate, indexed by code.
static const unsigned kBtQcaBauds[19] = {115200, 57600, 38400, 19200, 9600, 230400, 250000, 460800, 500000, 720000,
                                         921600, 1000000, 1250000, 2000000, 3000000, 4000000, 1600000, 3200000, 3500000};
// enum qca_baudrate code for a UART speed (qca_get_baudrate_value); 0xff = not a speed the controller knows.
static inline uint8_t bt_qca_baud_code(unsigned baud) {
    for (unsigned i = 0; i < sizeof kBtQcaBauds / sizeof kBtQcaBauds[0]; ++i)
        if (kBtQcaBauds[i] == baud) return i <= 17u ? (uint8_t)i : 0xff;   // qca_set_baudrate refuses codes above 3200000
    return 0xff;
}
// UART speed of an enum qca_baudrate code, 0 for a byte that is no rate code (e.g. the 0x22 of the guamp NVM).
static inline unsigned bt_qca_baud_of_code(uint8_t code) {
    return code < sizeof kBtQcaBauds / sizeof kBtQcaBauds[0] ? kBtQcaBauds[code] : 0u;
}

// File names (no "qca/" prefix) for the controller of the EDL version reply. Returns the family: 3950 or 3990.
static inline unsigned bt_qca_fw_names(const struct bt_qca_version *v, char tlv[20], char nvm[20]) {
    static const char hx[] = "0123456789abcdef";
    const int cm = (v->product_id & 0xffff) == BT_QCA_PRODUCT_WCN3950;
    const char *a = cm ? "cmbtfw" : "crbtfw", *b = cm ? "cmnv" : "crnv";
    char variant = 0;
    if (cm && v->soc_id == BT_QCA_SOC_WCN3950_T) variant = 't';
    else if (cm && v->soc_id == BT_QCA_SOC_WCN3950_S) variant = 's';
    else if (!cm && v->soc_id == BT_QCA_SOC_WCN3991) variant = 'u';
    unsigned i = 0, j = 0;
    for (; a[i]; ++i) tlv[i] = a[i];
    tlv[i++] = hx[v->rom >> 4]; tlv[i++] = hx[v->rom & 15];
    for (const char *s = ".tlv"; *s; ++s) tlv[i++] = *s;
    tlv[i] = 0;
    for (; b[j]; ++j) nvm[j] = b[j];
    nvm[j++] = hx[v->rom >> 4]; nvm[j++] = hx[v->rom & 15];
    if (variant) nvm[j++] = variant;
    for (const char *s = ".bin"; *s; ++s) nvm[j++] = *s;
    nvm[j] = 0;
    return cm ? 3950u : 3990u;
}

struct bt_tlv_patch {uint32_t total, data_len, entry; uint16_t product, rom_build, patch_ver; uint8_t type, format, sig, mode;};
// 0 ok, -1 too short or not a TLV patch (type 1).
static inline int bt_tlv_patch_parse(const uint8_t *f, size_t n, struct bt_tlv_patch *p) {
    if (n < 4 + 24) return -1;
    p->type = f[0];
    const uint8_t *h = f + 4;
    p->total = bt_le32(h); p->data_len = bt_le32(h + 4); p->format = h[8]; p->sig = h[9]; p->mode = h[10];
    p->product = bt_le16(h + 12); p->rom_build = bt_le16(h + 14); p->patch_ver = bt_le16(h + 16); p->entry = bt_le32(h + 20);
    return p->type == 1 ? 0 : -1;
}

struct bt_tlv_nvm {
    unsigned tags, type;                 // tags walked in the edited set, its TLV type (2 = edited)
    int has_bdaddr, has_hci, has_sleep;
    uint8_t bdaddr[6];                   // tag 2 as stored (little endian)
    uint8_t hci_before[3], hci_after[3], sleep_before, sleep_after;
};
// Applies Linux's NVM edits in place. fields: which HCI tag bytes get the baud code, bit 0 data[1] (Linux from
// WCN3991 on), bit 1 data[2] (Linux before, WCN3950 included). 0 ok, -1 malformed (Linux -EINVAL: no download).
static inline int bt_tlv_nvm_fix_fields(uint8_t *f, size_t n, uint8_t baud_code, unsigned fields, struct bt_tlv_nvm *o) {
    o->tags = 0; o->type = 0; o->has_bdaddr = o->has_hci = o->has_sleep = 0;
    if (n < 4) return -1;
    size_t at = 0;
    uint32_t tl = bt_le32(f);
    if ((tl & 0xff) == 4) {
        if (n < 8) return -1;
        at = 4;
        tl = bt_le32(f + 4);
    }
    const size_t length = tl >> 8;
    o->type = tl & 0xff;
    if (o->type != 2) return 0;
    uint8_t *data = f + at + 4;
    if (n < length + at + 4) return -1;
    size_t idx = 0;
    while (idx + 12 <= length) {
        uint8_t *t = data + idx;
        const uint16_t id = bt_le16(t), len = bt_le16(t + 2);
        uint8_t *d = t + 12;
        if (length < idx + 12 + len) return -1;
        if (id == BT_QCA_TAG_BD_ADDR) {
            if (len != 6) return -1;
            for (unsigned k = 0; k < 6; ++k) o->bdaddr[k] = d[k];
            o->has_bdaddr = 1;
        } else if (id == BT_QCA_TAG_HCI) {
            if (len < 3) return -1;
            for (unsigned k = 0; k < 3; ++k) o->hci_before[k] = d[k];
            d[0] = (uint8_t)(d[0] | 0x80);
            if (fields & 1u) d[1] = baud_code;
            if (fields & 2u) d[2] = baud_code;
            for (unsigned k = 0; k < 3; ++k) o->hci_after[k] = d[k];
            o->has_hci = 1;
        } else if (id == BT_QCA_TAG_DEEP_SLEEP) {
            if (len < 1) return -1;
            o->sleep_before = d[0];
            d[0] = (uint8_t)(d[0] | 0x01);
            o->sleep_after = d[0];
            o->has_sleep = 1;
        }
        o->tags = o->tags + 1;
        idx += 12 + (size_t)len;
    }
    return 0;
}
// Which HCI tag bytes may take the baud code (guamp-bt-nvm-baud absent): the Linux byte, but only when the file
// holds a rate code there. On the guamp WCN3950 NVM (15 11 22) data[2] = 0x22 is no rate: writing 0x0e there made
// the patched controller answer with 7-bit characters (reset Command Complete read as 84 8e 84 81 83 8c 80), so the
// byte is a line-format field on this ROM and is left alone (0 = no byte written; the rate set by VSC 0xFC48 stays).
static inline unsigned bt_tlv_nvm_baud_fields_auto(const uint8_t hci_before[3], int new_layout) {
    const unsigned k = new_layout ? 1u : 2u;
    return bt_qca_baud_of_code(hci_before[k]) ? (new_layout ? 1u : 2u) : 0u;
}
// Bytes with bit 7 set where an H4 event would start (0x84 = 0x04 | 0x80): the controller sends 7 data bits and
// the host reads the stop bit as bit 7. 1 when buf, with bit 7 cleared, starts with a complete H4 event.
static inline int bt_seven_bit_event(const uint8_t *buf, size_t n) {
    if (n < 3 || buf[0] != (0x04 | 0x80)) return 0;
    const size_t plen = buf[2] & 0x7f;
    if (n < 3 + plen) return 0;
    for (size_t i = 0; i < 3 + plen; ++i) if (!(buf[i] & 0x80)) return 0;
    return 1;
}
// The Linux layout: new_layout = WCN3991 and later (baud code in data[1]), else data[2].
static inline int bt_tlv_nvm_fix(uint8_t *f, size_t n, uint8_t baud_code, int new_layout, struct bt_tlv_nvm *o) {
    return bt_tlv_nvm_fix_fields(f, n, baud_code, new_layout ? 1u : 2u, o);
}

static inline unsigned bt_tlv_segments(size_t n) {return (unsigned)((n + BT_QCA_TLV_SEG_MAX - 1) / BT_QCA_TLV_SEG_MAX);}
// H4 packet of segment i of a file (VSC 0xFC00 {0x1E, len, data}); *ack = 1 when the controller answers it (mode
// QCA_SKIP_EVT_NONE / _CC, or the last segment). Returns the packet length, 0 past the end or when cap is short.
static inline size_t bt_tlv_segment(const uint8_t *f, size_t n, unsigned i, unsigned mode, uint8_t *pkt, size_t cap, int *ack) {
    const size_t off = (size_t)i * BT_QCA_TLV_SEG_MAX;
    if (off >= n) return 0;
    const size_t seg = n - off < BT_QCA_TLV_SEG_MAX ? n - off : BT_QCA_TLV_SEG_MAX;
    if (cap < 6 + seg) return 0;
    pkt[0] = BT_H4_CMD; pkt[1] = (uint8_t)(BT_OP_QCA_EDL & 0xff); pkt[2] = (uint8_t)(BT_OP_QCA_EDL >> 8);
    pkt[3] = (uint8_t)(seg + 2); pkt[4] = BT_QCA_TLV_REQ; pkt[5] = (uint8_t)seg;
    for (size_t k = 0; k < seg; ++k) pkt[6 + k] = f[off + k];
    const int last = off + seg == n || seg < BT_QCA_TLV_SEG_MAX;
    *ack = last || !(mode == BT_QCA_SKIP_VSE || mode == BT_QCA_SKIP_VSE_CC);
    return 6 + seg;
}
// Segment ack (qca_tlv_send_segment before WCN3991): the vendor event must be exactly {0x00, 0x04, result}.
// 0 = well formed (*result = the controller's status, Linux only logs a non-zero one), -1 = wrong size or type.
static inline int bt_tlv_ack(const struct bt_event *ev, uint8_t *result) {
    if (ev->code != BT_EVT_VENDOR || ev->len != 3 || ev->p[0] != BT_QCA_EDL_CMD_RES || ev->p[1] != BT_QCA_EDL_TLV_RES) return -1;
    *result = ev->p[2];
    return 0;
}

// HCI_Read_BD_ADDR Command Complete: status, address little endian. 0 ok, 1 status error, -1 not this reply.
static inline int bt_decode_bd_addr(const struct bt_event *ev, uint8_t addr[6], uint8_t *status) {
    uint16_t op; const uint8_t *r; uint8_t n;
    if (bt_cmd_complete(ev, &op, &r, &n) || op != BT_OP_READ_BD_ADDR || n < 1) return -1;
    *status = r[0];
    if (r[0]) return 1;
    if (n < 7) return -1;
    for (unsigned k = 0; k < 6; ++k) addr[k] = r[1 + k];
    return 0;
}
// "aa:bb:cc:dd:ee:ff" (most significant byte first, as hciconfig prints it) into out[18].
static inline void bt_bd_addr_text(const uint8_t a[6], char out[18]) {
    static const char hx[] = "0123456789abcdef";
    for (unsigned k = 0; k < 6; ++k) {
        out[3 * k] = hx[a[5 - k] >> 4]; out[3 * k + 1] = hx[a[5 - k] & 15]; out[3 * k + 2] = k < 5 ? ':' : 0;
    }
}
static inline int bt_bd_addr_zero(const uint8_t a[6]) {return !(a[0] | a[1] | a[2] | a[3] | a[4] | a[5]);}
static inline int bt_bd_addr_eq(const uint8_t a[6], const uint8_t b[6]) {
    for (unsigned k = 0; k < 6; ++k) if (a[k] != b[k]) return 0;
    return 1;
}
#endif
