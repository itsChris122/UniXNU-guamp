// SPDX-License-Identifier: BSD-2-Clause-Patent
// Bluetooth M2 shim, pure part: the H4 transport that iOS's bluetoothd sees on the serial node of GuampBtSerial.inc,
// in front of the Qualcomm WCN3950 that the M1 driver (BtService.inc) brings up on GENI SE3.
//
//   host side   bytes written by bluetoothd: H4 packets (01 cmd, 02 ACL, 03 SCO, 05 ISO). Standard HCI goes to the
//               controller unchanged through bts_link::tx. Broadcom vendor commands (OGF 0x3f) never reach the QCA
//               chip (its own vendor space 0xfcxx means something else): the known ones (table kBtsBcmCmds, from
//               Linux btbcm.c / hci_bcm.c) get a synthetic Command Complete, the unknown ones Command Complete with
//               status 0x01 (Unknown HCI Command).
//   controller  bytes from the UART (after the M1 driver's IBS handling): H4 events, ACL, SCO, ISO. Read Local Version
//               and Read Local Name replies are rewritten with bts_identity (a Broadcom part, default BCM4355C0);
//               Qualcomm vendor events (0xff) are dropped unless bts_identity::passVendorEvents.
//   read side   whole packets only, in a byte FIFO that bluetoothd drains with read(); a packet that does not fit is
//               dropped whole (counted), never split.
//   tty side    bts_tty: the termios / modem-line ioctls a serial client issues, answered without struct tty (the
//               ioctl argument is stored as opaque bytes of the size the command encodes; layouts of the 64-bit
//               user_termios, 72 bytes, and the 32-bit termios, 44 bytes, only for the defaults and the speed).
//
// No allocation, no locking, no kernel calls: GuampBtSerial.inc owns locking and wake-ups. Host-tested by
// tests/bt_shim_host.cpp (tests/run-bt-shim-tests.sh).
#ifndef GUAMP_BT_SHIM_H
#define GUAMP_BT_SHIM_H
#include <stdint.h>
#include <stddef.h>

#define BTS_H4_CMD 0x01
#define BTS_H4_ACL 0x02
#define BTS_H4_SCO 0x03
#define BTS_H4_EVT 0x04
#define BTS_H4_ISO 0x05
#define BTS_EVT_CMD_COMPLETE 0x0e
#define BTS_EVT_CMD_STATUS 0x0f
#define BTS_EVT_VENDOR 0xff
#define BTS_OP_READ_LOCAL_VERSION 0x1001
#define BTS_OP_READ_LOCAL_NAME 0x0c14
#define BTS_OGF_VENDOR 0x3f
#define BTS_STATUS_UNKNOWN_COMMAND 0x01
#define BTS_STATUS_HARDWARE_FAILURE 0x03
#define BTS_NAME_LEN 248                       // Read Local Name: status + 248-byte name
#define BTS_PKT_MAX 1100                       // largest H4 packet accepted either way (ACL 1021 + header, margin)
#define BTS_RX_FIFO 16384                      // bytes waiting for bluetoothd's read()

static inline uint16_t bts_le16(const uint8_t *p) {return (uint16_t)(p[0] | p[1] << 8);}
static inline void bts_put16(uint8_t *p, uint16_t v) {p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);}
static inline void bts_copy(uint8_t *d, const uint8_t *s, size_t n) {for (size_t i = 0; i < n; i = i + 1) d[i] = s[i];}
static inline void bts_zero(void *d, size_t n) {uint8_t *b = (uint8_t *)d; for (size_t i = 0; i < n; i = i + 1) b[i] = 0;}

// ---- H4 reassembly ------------------------------------------------------------------------------------------------
// Header length after the type byte and where its payload length sits: cmd opcode+len8, ACL handle+len16, SCO
// handle+len8, event code+len8, ISO handle+len14 (Core spec vol 4 part A; ISO length is 14 bits).
static inline unsigned bts_h4_header(uint8_t type) {
    switch (type) {
    case BTS_H4_CMD: return 3;
    case BTS_H4_ACL: return 4;
    case BTS_H4_SCO: return 3;
    case BTS_H4_EVT: return 2;
    case BTS_H4_ISO: return 4;
    default: return 0;
    }
}
static inline size_t bts_h4_payload(uint8_t type, const uint8_t *hdr) {   // hdr = bytes after the type byte
    switch (type) {
    case BTS_H4_CMD: return hdr[2];
    case BTS_H4_ACL: return bts_le16(hdr + 2);
    case BTS_H4_SCO: return hdr[2];
    case BTS_H4_EVT: return hdr[1];
    case BTS_H4_ISO: return bts_le16(hdr + 2) & 0x3fff;
    default: return 0;
    }
}

struct bts_h4 {
    uint8_t buf[BTS_PKT_MAX];
    size_t have;          // bytes of the current packet in buf (type byte included)
    size_t need;          // total length once the header is known, 0 before
    uint32_t skipped;     // bytes that could not start a packet (resync, IBS bytes, noise)
    uint32_t oversize;    // packets longer than BTS_PKT_MAX, discarded while their bytes pass
    size_t discard;       // bytes still to throw away of an oversize packet
};
typedef void (*bts_pkt_fn)(void *ctx, const uint8_t *pkt, size_t len);

static inline void bts_h4_reset(struct bts_h4 *r) {r->have = 0; r->need = 0; r->discard = 0;}

// Feed bytes; every complete packet (type byte first) goes to fn. Returns the number of packets delivered.
static inline unsigned bts_h4_feed(struct bts_h4 *r, const uint8_t *data, size_t len, bts_pkt_fn fn, void *ctx) {
    unsigned out = 0;
    for (size_t i = 0; i < len; i = i + 1) {
        const uint8_t b = data[i];
        if (r->discard) {r->discard = r->discard - 1; continue;}
        if (r->have == 0) {
            if (!bts_h4_header(b)) {r->skipped = r->skipped + 1; continue;}
            r->buf[0] = b; r->have = 1; r->need = 0;
            continue;
        }
        r->buf[r->have] = b; r->have = r->have + 1;
        const uint8_t type = r->buf[0];
        const unsigned hl = bts_h4_header(type);
        if (!r->need && r->have == 1 + (size_t)hl) {
            const size_t total = 1 + (size_t)hl + bts_h4_payload(type, r->buf + 1);
            if (total > BTS_PKT_MAX) {
                r->oversize = r->oversize + 1;
                r->discard = total - r->have;
                r->have = 0;
                continue;
            }
            r->need = total;
        }
        if (r->need && r->have == r->need) {
            fn(ctx, r->buf, r->have);
            out = out + 1;
            r->have = 0; r->need = 0;
        }
    }
    return out;
}

// ---- event encoders ------------------------------------------------------------------------------------------------
// H4 Command Complete: 04 0e plen ncmd opcode(le) status ret[n]. Returns the length, 0 when cap is short.
static inline size_t bts_cmd_complete(uint8_t *out, size_t cap, uint16_t opcode, uint8_t status, const uint8_t *ret,
                                      size_t n) {
    const size_t plen = 4 + n;
    if (plen > 255 || cap < 3 + plen) return 0;
    out[0] = BTS_H4_EVT; out[1] = BTS_EVT_CMD_COMPLETE; out[2] = (uint8_t)plen; out[3] = 1;
    bts_put16(out + 4, opcode); out[6] = status;
    if (n) bts_copy(out + 7, ret, n);
    return 3 + plen;
}
// H4 Command Status: 04 0f 04 status ncmd opcode(le).
static inline size_t bts_cmd_status(uint8_t *out, size_t cap, uint16_t opcode, uint8_t status) {
    if (cap < 7) return 0;
    out[0] = BTS_H4_EVT; out[1] = BTS_EVT_CMD_STATUS; out[2] = 4; out[3] = status; out[4] = 1;
    bts_put16(out + 5, opcode);
    return 7;
}

// ---- byte FIFO of whole packets -------------------------------------------------------------------------------------
struct bts_fifo { uint8_t data[BTS_RX_FIFO]; size_t head, count; };   // head = oldest byte
static inline size_t bts_fifo_room(const struct bts_fifo *f) {return BTS_RX_FIFO - f->count;}
static inline bool bts_fifo_put(struct bts_fifo *f, const uint8_t *p, size_t n) {   // all or nothing
    if (n > bts_fifo_room(f)) return false;
    size_t at = (f->head + f->count) % BTS_RX_FIFO;
    for (size_t i = 0; i < n; i = i + 1) {f->data[at] = p[i]; at = at + 1 == BTS_RX_FIFO ? 0 : at + 1;}
    f->count += n;
    return true;
}
static inline size_t bts_fifo_get(struct bts_fifo *f, uint8_t *p, size_t cap) {
    const size_t n = cap < f->count ? cap : f->count;
    for (size_t i = 0; i < n; i = i + 1) {p[i] = f->data[f->head]; f->head = f->head + 1 == BTS_RX_FIFO ? 0 : f->head + 1;}
    f->count -= n;
    if (!f->count) f->head = 0;
    return n;
}

// ---- Broadcom identity and vendor commands -------------------------------------------------------------------------
// What the controller claims to be. Defaults (analisi/bt-ios/PROGETTO-BT-IOS.md): Broadcom (company id 15), Bluetooth
// 5.0 (HCI/LMP 9, as the WCN3950), LMP subversion 0x6103 (BlueTool: 0x6103 -> "BCM4355C0"), name BCM4355C0, USB vid
// 0x0a5c. hciRev and the verbose-config bytes are not checked by bluetoothd as far as the study found: placeholders.
struct bts_identity {
    uint8_t hciVer;
    uint16_t hciRev;
    uint8_t lmpVer;
    uint16_t manufacturer;
    uint16_t lmpSubver;
    char name[32];
    uint8_t verbose[6];        // 0xfc79 return parameters after status (btbcm_read_verbose_config: chip_id, target_id,
                               // build_base le16, build_num le16)
    uint8_t features[8];       // 0xfc6e return parameters after status (btbcm_read_controller_features)
    uint8_t usbProduct[4];     // 0xfc5a return parameters after status (vid le16, pid le16)
    bool rewriteVersion, rewriteName, passVendorEvents;
    bool translateBdaddr;      // also send 0xfc01's address to the WCN3950 as Qualcomm EDL_WRITE_BD_ADDR 0xfc14
    uint8_t unknownVendorStatus;   // Command Complete status for unlisted OGF 0x3f commands
};
static inline void bts_identity_default(struct bts_identity *id) {
    bts_zero(id, sizeof *id);
    id->hciVer = 9; id->hciRev = 0x0000; id->lmpVer = 9;
    id->manufacturer = 15;                     // Broadcom Corporation
    id->lmpSubver = 0x6103;                    // BCM4355C0 for BlueTool / bluetoothd
    id->verbose[0] = 0x55; id->verbose[4] = 1; // chip id, build_num non-zero (placeholders)
    id->usbProduct[0] = 0x5c; id->usbProduct[1] = 0x0a;   // vid 0x0a5c (Broadcom), pid 0
    const char n[] = "BCM4355C0";
    for (size_t i = 0; i < sizeof n; i = i + 1) id->name[i] = n[i];
    id->rewriteVersion = true; id->rewriteName = true; id->passVendorEvents = false;
    id->unknownVendorStatus = BTS_STATUS_UNKNOWN_COMMAND;
}

enum bts_bcm_action {
    BTS_BCM_ACK,          // Command Complete, status 0, no data
    BTS_BCM_BAUD,         // ACK + record the requested rate (the real UART stays where M1 put it)
    BTS_BCM_BDADDR,       // ACK + record the address + bts_link::set_bdaddr when present
    BTS_BCM_MINIDRIVER,   // ACK + firmware download state
    BTS_BCM_WRITE_RAM,    // ACK + count the patch bytes (address le32 + data)
    BTS_BCM_LAUNCH_RAM,   // ACK + download finished
    BTS_BCM_VERBOSE,      // ACK with bts_identity::verbose
    BTS_BCM_FEATURES,     // ACK with bts_identity::features
    BTS_BCM_USB_PRODUCT,  // ACK with bts_identity::usbProduct
};
struct bts_bcm_cmd { uint16_t opcode; enum bts_bcm_action action; const char *name; };
// Linux drivers/bluetooth btbcm.c / hci_bcm.c opcodes. Entries marked (?) are less certain; an unlisted vendor command
// still gets an answer (unknownVendorStatus), so a wrong entry costs a status code, not a hang.
static const struct bts_bcm_cmd kBtsBcmCmds[] = {
    {0xfc01, BTS_BCM_BDADDR, "write_bdaddr"},          // btbcm_set_bdaddr
    {0xfc18, BTS_BCM_BAUD, "update_baud"},             // hci_bcm bcm_set_baudrate
    {0xfc1c, BTS_BCM_ACK, "sco_pcm_int"},              // Write_SCO_PCM_Int_Param (?)
    {0xfc1e, BTS_BCM_ACK, "pcm_data_format"},          // Write_PCM_Data_Format_Param (?)
    {0xfc27, BTS_BCM_ACK, "sleep_mode"},               // hci_bcm bcm_setup_sleep
    {0xfc2e, BTS_BCM_MINIDRIVER, "download_minidriver"},   // btbcm_patchram
    {0xfc45, BTS_BCM_ACK, "uart_clock"},               // hci_bcm BCM_UART_CLOCK write
    {0xfc4c, BTS_BCM_WRITE_RAM, "write_ram"},          // btbcm_patchram
    {0xfc4e, BTS_BCM_LAUNCH_RAM, "launch_ram"},        // btbcm_patchram
    {0xfc5a, BTS_BCM_USB_PRODUCT, "read_usb_product"}, // btbcm_read_usb_product (?)
    {0xfc6d, BTS_BCM_ACK, "i2spcm_param"},             // I2S/PCM interface parameters (?)
    {0xfc6e, BTS_BCM_FEATURES, "read_features"},       // btbcm_read_controller_features (?)
    {0xfc79, BTS_BCM_VERBOSE, "read_verbose_config"},  // btbcm_read_verbose_config (?)
};
static inline const struct bts_bcm_cmd *bts_bcm_find(uint16_t opcode) {
    for (size_t i = 0; i < sizeof kBtsBcmCmds / sizeof kBtsBcmCmds[0]; i = i + 1)
        if (kBtsBcmCmds[i].opcode == opcode) return &kBtsBcmCmds[i];
    return nullptr;
}

// ---- the shim ------------------------------------------------------------------------------------------------------
// Toward the controller. tx: one whole H4 packet, 0 = sent; called from thread context with the shim lock held, the
// link owner serializes its UART. Optional: set_bdaddr (the address bluetoothd asked for with 0xfc01), reset (true =
// /dev/btreset opened: hold the chip in reset; false = closed: release and redo the M1 setup), power (true =
// /dev/btpoweroff opened: chip off; false = closed: on). reset/power are called without the shim lock.
struct bts_link {
    void *ctx;
    int (*tx)(void *ctx, const uint8_t *pkt, size_t len);
    void (*set_bdaddr)(void *ctx, const uint8_t addr[6]);
    void (*reset)(void *ctx, bool asserted);
    void (*power)(void *ctx, bool off);
};
#define BTS_QCA_EDL_WRITE_BD_ADDR 0xfc14       // btqca.h EDL_WRITE_BD_ADDR_OPCODE (qca_set_bdaddr)
struct bts_stats {
    uint32_t hostPkts, hostCmds, forwarded, txErrors, emulated, unknownVendor;
    uint32_t ctrlPkts, rewritten, vendorDropped, rxDropped, internalDropped;
    uint32_t writeRamCmds, launches, minidriver;
    uint64_t writeRamBytes, rxBytes;
};
struct bts_shim {
    struct bts_h4 host, ctrl;
    struct bts_fifo rx;
    struct bts_identity id;
    struct bts_link link;
    struct bts_stats st;
    uint32_t baud;              // last 0xfc18 request (0 = none)
    uint8_t bdaddr[6];
    bool bdaddrSet, downloading;
    uint16_t lastVendorOpcode;
    uint16_t internalOpcode;    // a command the shim itself sent: its Command Complete / Status is not for bluetoothd
    uint16_t lastCmd;           // last HCI command the client wrote
    uint16_t lastReply;         // opcode of the last Command Complete / Command Status queued for the client
    uint8_t lastReplyStatus;    // its status (lastCmd == lastReply and status 0: the client's last command succeeded)
    uint8_t scratch[BTS_PKT_MAX];
};

static inline void bts_shim_init(struct bts_shim *s, const struct bts_link *link, const struct bts_identity *id) {
    bts_zero(s, sizeof *s);
    if (link) s->link = *link;
    if (id) s->id = *id; else bts_identity_default(&s->id);
}
// Drop what is queued for read() and any half packet (open, TIOCFLUSH read side).
static inline void bts_shim_flush_rx(struct bts_shim *s) {s->rx.head = 0; s->rx.count = 0;}
static inline void bts_shim_flush_tx(struct bts_shim *s) {bts_h4_reset(&s->host);}
static inline size_t bts_shim_readable(const struct bts_shim *s) {return s->rx.count;}
static inline size_t bts_shim_read(struct bts_shim *s, uint8_t *out, size_t cap) {return bts_fifo_get(&s->rx, out, cap);}

static inline void bts_shim_queue(struct bts_shim *s, const uint8_t *pkt, size_t len) {
    if (bts_fifo_put(&s->rx, pkt, len)) s->st.rxBytes += len;
    else s->st.rxDropped = s->st.rxDropped + 1;
    if (len >= 6 && pkt[0] == BTS_H4_EVT && pkt[1] == BTS_EVT_CMD_COMPLETE) {
        s->lastReply = bts_le16(pkt + 4); s->lastReplyStatus = len >= 7 ? pkt[6] : 0;
    } else if (len >= 7 && pkt[0] == BTS_H4_EVT && pkt[1] == BTS_EVT_CMD_STATUS) {
        s->lastReply = bts_le16(pkt + 5); s->lastReplyStatus = pkt[3];
    }
}

static inline void bts_shim_vendor(struct bts_shim *s, uint16_t opcode, const uint8_t *p, size_t plen) {
    const struct bts_bcm_cmd *c = bts_bcm_find(opcode);
    s->lastVendorOpcode = opcode;
    uint8_t *o = s->scratch;
    size_t n = 0;
    if (!c) {
        s->st.unknownVendor = s->st.unknownVendor + 1;
        n = bts_cmd_complete(o, sizeof s->scratch, opcode, s->id.unknownVendorStatus, nullptr, 0);
        bts_shim_queue(s, o, n);
        return;
    }
    s->st.emulated = s->st.emulated + 1;
    switch (c->action) {
    case BTS_BCM_BAUD:   // hci_bcm: le16 reserved(0), le32 baud
        if (plen >= 6) s->baud = (uint32_t)p[2] | (uint32_t)p[3] << 8 | (uint32_t)p[4] << 16 | (uint32_t)p[5] << 24;
        break;
    case BTS_BCM_BDADDR:
        if (plen >= 6) {
            bts_copy(s->bdaddr, p, 6); s->bdaddrSet = true;
            if (s->link.set_bdaddr) s->link.set_bdaddr(s->link.ctx, s->bdaddr);
            if (s->id.translateBdaddr && s->link.tx) {   // Linux qca_set_bdaddr: baswap of the little-endian bdaddr_t
                uint8_t q[10] = {BTS_H4_CMD, (uint8_t)BTS_QCA_EDL_WRITE_BD_ADDR, (uint8_t)(BTS_QCA_EDL_WRITE_BD_ADDR >> 8), 6};
                for (size_t i = 0; i < 6; i = i + 1) q[4 + i] = s->bdaddr[5 - i];
                if (!s->link.tx(s->link.ctx, q, sizeof q)) s->internalOpcode = BTS_QCA_EDL_WRITE_BD_ADDR;
                else s->st.txErrors = s->st.txErrors + 1;
            }
        }
        break;
    case BTS_BCM_MINIDRIVER: s->downloading = true; s->st.minidriver = s->st.minidriver + 1; break;
    case BTS_BCM_WRITE_RAM:
        s->st.writeRamCmds = s->st.writeRamCmds + 1;
        if (plen > 4) s->st.writeRamBytes += plen - 4;
        break;
    case BTS_BCM_LAUNCH_RAM: s->downloading = false; s->st.launches = s->st.launches + 1; break;
    default: break;
    }
    const uint8_t *ret = nullptr;
    size_t rn = 0;
    if (c->action == BTS_BCM_VERBOSE) {ret = s->id.verbose; rn = sizeof s->id.verbose;}
    if (c->action == BTS_BCM_FEATURES) {ret = s->id.features; rn = sizeof s->id.features;}
    if (c->action == BTS_BCM_USB_PRODUCT) {ret = s->id.usbProduct; rn = sizeof s->id.usbProduct;}
    n = bts_cmd_complete(o, sizeof s->scratch, opcode, 0, ret, rn);
    bts_shim_queue(s, o, n);
}

static inline void bts_shim_host_pkt(void *ctx, const uint8_t *pkt, size_t len) {
    struct bts_shim *s = (struct bts_shim *)ctx;
    s->st.hostPkts = s->st.hostPkts + 1;
    if (pkt[0] == BTS_H4_CMD) {
        s->st.hostCmds = s->st.hostCmds + 1;
        const uint16_t opcode = bts_le16(pkt + 1);
        s->lastCmd = opcode;
        if ((opcode >> 10) == BTS_OGF_VENDOR) {bts_shim_vendor(s, opcode, pkt + 4, len - 4); return;}
        if (!s->link.tx || s->link.tx(s->link.ctx, pkt, len)) {
            s->st.txErrors = s->st.txErrors + 1;
            const size_t n = bts_cmd_status(s->scratch, sizeof s->scratch, opcode, BTS_STATUS_HARDWARE_FAILURE);
            bts_shim_queue(s, s->scratch, n);
            return;
        }
        s->st.forwarded = s->st.forwarded + 1;
        return;
    }
    if (!s->link.tx || s->link.tx(s->link.ctx, pkt, len)) s->st.txErrors = s->st.txErrors + 1;
    else s->st.forwarded = s->st.forwarded + 1;
}

// Bytes bluetoothd wrote. Returns the number of packets completed by this call.
static inline unsigned bts_shim_host_write(struct bts_shim *s, const uint8_t *data, size_t len) {
    return bts_h4_feed(&s->host, data, len, bts_shim_host_pkt, s);
}

static inline void bts_shim_ctrl_pkt(void *ctx, const uint8_t *pkt, size_t len) {
    struct bts_shim *s = (struct bts_shim *)ctx;
    s->st.ctrlPkts = s->st.ctrlPkts + 1;
    if (pkt[0] == BTS_H4_EVT) {
        const uint8_t code = pkt[1], plen = pkt[2];
        if (code == BTS_EVT_VENDOR && !s->id.passVendorEvents) {s->st.vendorDropped = s->st.vendorDropped + 1; return;}
        if (s->internalOpcode && ((code == BTS_EVT_CMD_COMPLETE && plen >= 3 && bts_le16(pkt + 4) == s->internalOpcode) ||
                                  (code == BTS_EVT_CMD_STATUS && plen >= 4 && bts_le16(pkt + 5) == s->internalOpcode))) {
            s->internalOpcode = 0;
            s->st.internalDropped = s->st.internalDropped + 1;
            return;
        }
        if (code == BTS_EVT_CMD_COMPLETE && plen >= 4) {
            const uint16_t opcode = bts_le16(pkt + 4);
            uint8_t *o = s->scratch;
            // Read Local Version: status, hci_ver, hci_rev le16, lmp_ver, manufacturer le16, lmp_subver le16.
            if (opcode == BTS_OP_READ_LOCAL_VERSION && s->id.rewriteVersion && plen >= 12 && pkt[6] == 0) {
                bts_copy(o, pkt, len);
                o[7] = s->id.hciVer; bts_put16(o + 8, s->id.hciRev); o[10] = s->id.lmpVer;
                bts_put16(o + 11, s->id.manufacturer); bts_put16(o + 13, s->id.lmpSubver);
                s->st.rewritten = s->st.rewritten + 1;
                bts_shim_queue(s, o, len);
                return;
            }
            // Read Local Name: status + 248 bytes.
            if (opcode == BTS_OP_READ_LOCAL_NAME && s->id.rewriteName && plen >= 4 + BTS_NAME_LEN && pkt[6] == 0) {
                bts_copy(o, pkt, len);
                bts_zero(o + 7, BTS_NAME_LEN);
                for (size_t i = 0; i < sizeof s->id.name && s->id.name[i]; i = i + 1) o[7 + i] = (uint8_t)s->id.name[i];
                s->st.rewritten = s->st.rewritten + 1;
                bts_shim_queue(s, o, len);
                return;
            }
        }
    }
    bts_shim_queue(s, pkt, len);
}

// Bytes from the controller (IBS bytes may still be present: they cannot start a packet and are skipped).
static inline unsigned bts_shim_ctrl_input(struct bts_shim *s, const uint8_t *data, size_t len) {
    return bts_h4_feed(&s->ctrl, data, len, bts_shim_ctrl_pkt, s);
}

// ---- serial-port ioctls without struct tty --------------------------------------------------------------------------
#define BTS_IOC_LEN(c) (((c) >> 16) & 0x1fffu)
#define BTS_IOC_GROUP(c) (((c) >> 8) & 0xffu)
#define BTS_IOC_NUM(c) ((c) & 0xffu)
#define BTS_IOC_IN 0x80000000u
#define BTS_IOC_OUT 0x40000000u
#define BTS_TIOCM_LE 0x001
#define BTS_TIOCM_DTR 0x002
#define BTS_TIOCM_RTS 0x004
#define BTS_TIOCM_CTS 0x020
#define BTS_TIOCM_CAR 0x040
#define BTS_TIOCM_DSR 0x100
#define BTS_TERMIOS64 72      // struct user_termios: 4 x u64 flags, c_cc[20], aligned u64 ispeed at 56, ospeed at 64
#define BTS_TERMIOS32 44      // struct termios32: 4 x u32 flags, c_cc[20], u32 ispeed at 36, ospeed at 40
#define BTS_CS8 0x300u
#define BTS_CREAD 0x800u
#define BTS_CLOCAL 0x8000u

struct bts_tty {
    uint8_t termios[BTS_TERMIOS64];
    unsigned termiosLen;   // 0 = never set: TIOCGETA answers the defaults
    uint64_t speed;        // from TIOCSETA* or IOSSIOSPEED
    int modem;             // DTR/RTS as set by the client
    bool exclusive, nonblocking;
};
enum bts_ioctl_result {
    BTS_IOCTL_OK = 0,
    BTS_IOCTL_NOTTY = 1,       // not a serial ioctl: ENOTTY
    BTS_IOCTL_INVAL = 2,       // malformed argument: EINVAL
    BTS_IOCTL_FLUSH = 3,       // OK; caller flushes per *flush (bit 0 read side, bit 1 write side)
};
static inline void bts_tty_init(struct bts_tty *t) {
    bts_zero(t, sizeof *t);
    t->speed = 115200;
    t->modem = BTS_TIOCM_DTR | BTS_TIOCM_RTS;
}
static inline void bts_wr_u64(uint8_t *p, uint64_t v) {for (unsigned i = 0; i < 8; i = i + 1) p[i] = (uint8_t)(v >> 8 * i);}
static inline void bts_wr_u32(uint8_t *p, uint32_t v) {for (unsigned i = 0; i < 4; i = i + 1) p[i] = (uint8_t)(v >> 8 * i);}
static inline uint64_t bts_rd_u64(const uint8_t *p) {uint64_t v = 0; for (unsigned i = 0; i < 8; i = i + 1) v |= (uint64_t)p[i] << 8 * i; return v;}
static inline uint32_t bts_rd_u32(const uint8_t *p) {return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;}

// Raw 8N1 local line at t->speed, in the layout the size names.
static inline void bts_termios_default(const struct bts_tty *t, uint8_t *out, unsigned len) {
    bts_zero(out, len);
    if (len == BTS_TERMIOS64) {
        bts_wr_u64(out + 16, BTS_CS8 | BTS_CREAD | BTS_CLOCAL);
        bts_wr_u64(out + 56, t->speed); bts_wr_u64(out + 64, t->speed);
    } else if (len == BTS_TERMIOS32) {
        bts_wr_u32(out + 8, BTS_CS8 | BTS_CREAD | BTS_CLOCAL);
        bts_wr_u32(out + 36, (uint32_t)t->speed); bts_wr_u32(out + 40, (uint32_t)t->speed);
    }
}

// cmd/data as the cdevsw ioctl entry receives them (data already in kernel memory, IOCPARM_LEN(cmd) bytes).
// readable = bytes queued for read() (FIONREAD). *flush is set for BTS_IOCTL_FLUSH.
static inline enum bts_ioctl_result bts_tty_ioctl(struct bts_tty *t, uint32_t cmd, uint8_t *data, size_t readable,
                                                  int *flush) {
    const unsigned len = BTS_IOC_LEN(cmd), group = BTS_IOC_GROUP(cmd), num = BTS_IOC_NUM(cmd);
    const bool in = cmd & BTS_IOC_IN, out = cmd & BTS_IOC_OUT;
    if (group == 't') {
        switch (num) {
        case 19:   // TIOCGETA
            if (!out || (len != BTS_TERMIOS64 && len != BTS_TERMIOS32) || !data) return BTS_IOCTL_INVAL;
            if (t->termiosLen == len) bts_copy(data, t->termios, len);
            else bts_termios_default(t, data, len);
            return BTS_IOCTL_OK;
        case 20: case 21: case 22:   // TIOCSETA, TIOCSETAW, TIOCSETAF (F also flushes input)
            if (!in || (len != BTS_TERMIOS64 && len != BTS_TERMIOS32) || !data) return BTS_IOCTL_INVAL;
            bts_copy(t->termios, data, len); t->termiosLen = len;
            t->speed = len == BTS_TERMIOS64 ? bts_rd_u64(data + 64) : bts_rd_u32(data + 40);
            if (num == 22) {*flush = 1; return BTS_IOCTL_FLUSH;}
            return BTS_IOCTL_OK;
        case 13: t->exclusive = true; return BTS_IOCTL_OK;    // TIOCEXCL
        case 14: t->exclusive = false; return BTS_IOCTL_OK;   // TIOCNXCL
        case 16: {   // TIOCFLUSH: FREAD 1, FWRITE 2, 0 = both
            if (!in || len != 4 || !data) return BTS_IOCTL_INVAL;
            const int what = (int)bts_rd_u32(data) & 3;
            *flush = what ? what : 3;
            return BTS_IOCTL_FLUSH;
        }
        case 94: return BTS_IOCTL_OK;                     // TIOCDRAIN: writes are synchronous
        case 115:                                         // TIOCOUTQ
            if (!out || len != 4 || !data) return BTS_IOCTL_INVAL;
            bts_wr_u32(data, 0); return BTS_IOCTL_OK;
        case 106:                                         // TIOCMGET: the far side always present
            if (!out || len != 4 || !data) return BTS_IOCTL_INVAL;
            bts_wr_u32(data, (uint32_t)(t->modem | BTS_TIOCM_LE | BTS_TIOCM_CTS | BTS_TIOCM_CAR | BTS_TIOCM_DSR));
            return BTS_IOCTL_OK;
        case 107: case 108: case 109: {                   // TIOCMBIC, TIOCMBIS, TIOCMSET
            if (!in || len != 4 || !data) return BTS_IOCTL_INVAL;
            const int v = (int)bts_rd_u32(data) & (BTS_TIOCM_DTR | BTS_TIOCM_RTS);
            t->modem = num == 107 ? t->modem & ~v : num == 108 ? t->modem | v : v;
            return BTS_IOCTL_OK;
        }
        case 120: t->modem &= ~BTS_TIOCM_DTR; return BTS_IOCTL_OK;   // TIOCCDTR
        case 121: t->modem |= BTS_TIOCM_DTR; return BTS_IOCTL_OK;    // TIOCSDTR
        case 122: case 123: return BTS_IOCTL_OK;                      // TIOCCBRK, TIOCSBRK
        default: return BTS_IOCTL_NOTTY;
        }
    }
    if (group == 'f') {
        switch (num) {
        case 127:   // FIONREAD
            if (!out || len != 4 || !data) return BTS_IOCTL_INVAL;
            bts_wr_u32(data, (uint32_t)readable); return BTS_IOCTL_OK;
        case 126:   // FIONBIO
            if (!in || len != 4 || !data) return BTS_IOCTL_INVAL;
            t->nonblocking = bts_rd_u32(data) != 0; return BTS_IOCTL_OK;
        case 125: return BTS_IOCTL_OK;   // FIOASYNC: accepted, no SIGIO
        default: return BTS_IOCTL_NOTTY;
        }
    }
    if (group == 'T') {   // IOKit/serial/ioss.h: IOSSDATALAT 0, IOSSPREEMPT 1, IOSSIOSPEED 2
        if (num == 2) {
            if (!in || !data || (len != 8 && len != 4)) return BTS_IOCTL_INVAL;
            t->speed = len == 8 ? bts_rd_u64(data) : bts_rd_u32(data);
            return BTS_IOCTL_OK;
        }
        if (num == 0 || num == 1) return BTS_IOCTL_OK;
    }
    return BTS_IOCTL_NOTTY;
}

// ---- kernel control socket (GuampBtSerial.inc, guamp-bt-ctl=1) ------------------------------------------------------
// iOS 17 opens the Bluetooth UART as a PF_SYSTEM kernel control, not as /dev/cu.bluetooth: BlueTool 21H461
// (0x10001e988; "device -D" = getenv BT_UART or "com.apple.uart.bluetooth", "device -d <name>" the same with <name>) and
// bluetoothd (0x1000ddaa8, default "com.apple.uart.sk.bluetooth") do socket(PF_SYSTEM, SOCK_STREAM, SYSPROTO_CONTROL),
// ioctl CTLIOCGINFO, connect with sc_unit 0. AppleOnboardSerialBSDClient (kernelcache 21H461, 0xfffffff005c5eb00)
// registers "com.apple.uart.%s" and "com.apple.uart.sk.%s" with CTL_FLAG_REG_SOCK_STREAM and recvsize 64 KiB.
//   data      the same H4 byte stream as the tty: write() = whole H4 packets, read() = type, header, body.
//   options   the tty ioctls travel as socket options at level SYSPROTO_CONTROL whose name is the ioctl number:
//             getsockopt(TIOCGETA 0x40487413, 72 bytes), cfmakeraw + cfsetspeed, setsockopt(TIOCSETA 0x80487414, 72)
//             with c_cflag CS8|CREAD|CLOCAL|CCTS_OFLOW|CRTS_IFLOW. The kext's option handler answers ENOPROTOOPT where
//             its ioctl switch says ENOTTY.
//   sk        the "sk" control adds Skywalk options (setsockopt IOAOSSKYSETCHANNELSPEC 0x800c5414, getsockopt
//             IOAOSSKYGETCHANNELUUID 0x40105412) and moves the data to an os_channel. There is no Skywalk nexus here:
//             they answer ENOPROTOOPT, and bluetoothd stays on its plain read()/write() path only with its strcmp
//             against the sk name skipped (ADT text fixup bluetoothd,ddd20,a0050034,1f2003d5).
enum bts_ctl_opt_result {
    BTS_CTL_OPT_OK = 0,
    BTS_CTL_OPT_NOPROTO = 1,   // not a serial option: ENOPROTOOPT
    BTS_CTL_OPT_INVAL = 2,     // wrong direction or length: EINVAL
    BTS_CTL_OPT_FLUSH = 3,     // OK; caller flushes per *flush (bit 0 read side, bit 1 write side)
};
#define BTS_CTL_OPT_MAX BTS_TERMIOS64   // largest serial ioctl argument answered here (the 64-bit termios)
// set = setsockopt (data, *len in), else getsockopt (*len = buffer size in, bytes written out). The argument is
// IOCPARM_LEN(opt) bytes; a larger buffer is accepted (only that many bytes are used or written).
static inline enum bts_ctl_opt_result bts_ctl_sockopt(struct bts_tty *t, uint32_t opt, bool set, uint8_t *data,
                                                      size_t *len, size_t readable, int *flush) {
    const size_t need = BTS_IOC_LEN(opt);
    *flush = 0;
    if (need > BTS_CTL_OPT_MAX) return BTS_CTL_OPT_NOPROTO;
    if (need && !(opt & (set ? BTS_IOC_IN : BTS_IOC_OUT))) return BTS_CTL_OPT_INVAL;
    if (need && (!data || !len || *len < need)) return BTS_CTL_OPT_INVAL;
    uint8_t arg[BTS_CTL_OPT_MAX];
    bts_zero(arg, sizeof arg);
    if (need && set) bts_copy(arg, data, need);
    const enum bts_ioctl_result r = bts_tty_ioctl(t, opt, need ? arg : (uint8_t *)0, readable, flush);
    if (r == BTS_IOCTL_NOTTY) return BTS_CTL_OPT_NOPROTO;
    if (r == BTS_IOCTL_INVAL) return BTS_CTL_OPT_INVAL;
    if (!set && len) {
        if (need) bts_copy(data, arg, need);
        *len = need;
    }
    return r == BTS_IOCTL_FLUSH ? BTS_CTL_OPT_FLUSH : BTS_CTL_OPT_OK;
}
// Who gets the one client slot. which/holder: 0 = com.apple.uart.bluetooth (BlueTool), 1 = com.apple.uart.sk.bluetooth
// (bluetoothd). BlueTool's boot script ends with "device -D -S", which reopens the control, and BlueTool stays alive
// holding it while bluetoothd connects the sk control (RAM test 6): bluetoothd takes the slot over (BlueTool's later
// writes get ENOTCONN). Anything else while the slot is taken (the tty, BlueTool against bluetoothd) is refused.
enum bts_ctl_admit_result { BTS_CTL_REFUSE = 0, BTS_CTL_ACCEPT = 1, BTS_CTL_PREEMPT = 2 };
static inline enum bts_ctl_admit_result bts_ctl_admit(bool open, bool viaKctl, unsigned holder, unsigned which) {
    if (!open) return BTS_CTL_ACCEPT;
    return viaKctl && holder == 0 && which == 1 ? BTS_CTL_PREEMPT : BTS_CTL_REFUSE;
}
// Bytes for the socket: no more than the receive buffer has room for (space, from ctl_getenqueuespace) nor cap.
// Packets may be split across enqueues (stream socket; the client reassembles by H4 header).
static inline size_t bts_ctl_take(struct bts_shim *s, uint8_t *out, size_t cap, size_t space) {
    return bts_shim_read(s, out, space < cap ? space : cap);
}

#endif
