// SPDX-License-Identifier: BSD-2-Clause-Patent
// Himax HX83102D zero-flash protocol over SPI (see HxProtocol.h for the sources).
#include "HxProtocol.h"

static void le32(uint8_t *p, uint32_t v) {p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);}
static uint32_t rd_le32(const uint8_t *p) {return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;}
static void copy(uint8_t *d, const uint8_t *s, size_t n) {for (size_t i = 0; i < n; ++i) d[i] = s[i];}
static void sleep_ms(struct hx_chip *c, unsigned ms) {if (c->bus->sleep_ms) c->bus->sleep_ms(c->bus->ctx, ms);}

uint32_t hx_crc(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i + 4 <= len; i += 4) {
        crc ^= rd_le32(data + i);
        for (unsigned j = 0; j < 32; ++j) crc = (crc & 1) ? (crc >> 1) ^ 0x82F63B78u : crc >> 1;
    }
    return crc;
}

int hx_fw_parse(struct hx_fw *fw, const uint8_t *data, size_t size)
{
    for (size_t i = 0; i < sizeof *fw; ++i) ((uint8_t *)fw)[i] = 0;
    if (!data || size < HX_ZF_TABLE + 16) return HX_ERR_FW;
    fw->data = data;
    fw->size = size;
    int header = 1;
    for (unsigned i = 0; i < 8; ++i) header &= data[i] == 0;
    fw->base = header && data[0x0E] == 0x87 ? HX_ZF_HEADER_1K : 0;
    const uint8_t *table = data + fw->base + HX_ZF_TABLE;
    if ((size_t)(table - data) + 16 > size) return HX_ERR_FW;
    fw->count = table[12];
    if (fw->count < 2 || fw->count > HX_ZF_MAX_PARTS || (size_t)(table - data) + 16u * fw->count > size) return HX_ERR_FW;
    uint32_t lo = 0xFFFFFFFFu, hi = 0, hi_size = 0;
    fw->overlay = -1;
    for (unsigned i = 0; i < fw->count; ++i) {
        const uint8_t *e = table + 16u * i;
        struct hx_zf_part *p = &fw->part[i];
        p->sram = rd_le32(e);
        p->size = rd_le32(e + 4);
        p->offset = rd_le32(e + 8);
        if (i && !p->size) continue;                 // empty entry (the guamp image has an empty overlay slot)
        if (!p->size || p->offset > size || p->size > size - p->offset) return HX_ERR_FW;
        if (i == 0) continue;
        // Code overlays (HX_CODE_OVERLAY: sorting, gesture, border) load into instruction SRAM like the firmware itself,
        // not into the config window; the first one (sorting, normal touch) is written after the config.
        if (p->sram >= 0x20000000u) { if (fw->overlay < 0) fw->overlay = (int)i; continue; }
        p->sram &= ~3u;                          // config windows are word aligned (the vendor rounds down the same way)
        if (p->sram < lo) lo = p->sram;
        if (p->sram >= hi) {hi = p->sram; hi_size = p->size;}
    }
    uint32_t span = hi - lo + hi_size;
    span = (span + 15u) & ~15u;
    if (span > HX_ZF_CFG_MAX) return HX_ERR_FW;
    fw->cfg_base = lo;
    fw->cfg_size = span;
    return HX_OK;
}

int hx_fw_config(const struct hx_fw *fw, uint8_t *out, size_t cap)
{
    if (!fw || !out || fw->cfg_size > cap) return HX_ERR_FW;
    for (size_t i = 0; i < fw->cfg_size; ++i) out[i] = 0;
    for (unsigned i = 1; i < fw->count; ++i) {
        const struct hx_zf_part *p = &fw->part[i];
        if (!p->size || p->sram >= 0x20000000u) continue;
        uint32_t at = p->sram - fw->cfg_base;
        if (at > fw->cfg_size || p->size > fw->cfg_size - at) return HX_ERR_FW;
        copy(out + at, fw->data + p->offset, p->size);
    }
    return HX_OK;
}

int hx_bus_write(struct hx_chip *c, uint8_t reg, const uint8_t *data, size_t len)
{
    if (len > HX_WRITE_CHUNK + 4) return HX_ERR_BUS;
    c->frame[0] = HX_SPI_WRITE;
    c->frame[1] = reg;
    if (len) copy(c->frame + 2, data, len);
    return c->bus->xfer(c->bus->ctx, c->frame, 0, len + 2) ? HX_ERR_BUS : HX_OK;
}

int hx_bus_read(struct hx_chip *c, uint8_t reg, uint8_t *data, size_t len)
{
    if (len > HX_WRITE_CHUNK + 4) return HX_ERR_BUS;
    for (size_t i = 0; i < len + 3; ++i) c->frame[i] = 0;
    c->frame[0] = HX_SPI_READ;
    c->frame[1] = reg;
    if (c->bus->xfer(c->bus->ctx, c->frame, c->rx, len + 3)) return HX_ERR_BUS;
    copy(data, c->rx + 3, len);
    return HX_OK;
}

static int burst(struct hx_chip *c, int incr4)
{
    uint8_t v = 0x31;
    if (hx_bus_write(c, HX_BUS_CONTI, &v, 1)) return HX_ERR_BUS;
    v = (uint8_t)(0x10 | (incr4 ? 1 : 0));
    return hx_bus_write(c, HX_BUS_INCR4, &v, 1);
}

// Address then data in one bus frame (flash_write_burst_lenth), split into HX_WRITE_CHUNK pieces.
int hx_reg_write(struct hx_chip *c, uint32_t addr, const uint8_t *data, size_t len)
{
    if (burst(c, len > 4)) return HX_ERR_BUS;
    uint8_t *buf = c->rx;                        // the read buffer is free during a write
    for (size_t done = 0; done < len;) {
        size_t n = len - done > HX_WRITE_CHUNK ? HX_WRITE_CHUNK : len - done;
        le32(buf, addr + (uint32_t)done);
        copy(buf + 4, data + done, n);
        if (hx_bus_write(c, HX_BUS_ADDR, buf, n + 4)) return HX_ERR_BUS;
        done += n;
    }
    return HX_OK;
}

int hx_reg_read(struct hx_chip *c, uint32_t addr, uint8_t *data, size_t len)
{
    for (size_t done = 0; done < len;) {
        size_t n = len - done > HX_READ_CHUNK ? HX_READ_CHUNK : len - done;
        uint8_t a[4], dir = 0;
        if (burst(c, n > 4)) return HX_ERR_BUS;
        le32(a, addr + (uint32_t)done);
        if (hx_bus_write(c, HX_BUS_ADDR, a, 4) || hx_bus_write(c, HX_BUS_DIR, &dir, 1) ||
            hx_bus_read(c, HX_BUS_RDATA, data + done, n)) return HX_ERR_BUS;
        done += n;
    }
    if (len > 4) burst(c, 0);
    return HX_OK;
}

int hx_reg_write32(struct hx_chip *c, uint32_t addr, uint32_t value)
{
    uint8_t v[4];
    le32(v, value);
    return hx_reg_write(c, addr, v, 4);
}

int hx_reg_read32(struct hx_chip *c, uint32_t addr, uint32_t *value)
{
    uint8_t v[4];
    int r = hx_reg_read(c, addr, v, 4);
    *value = r ? 0 : rd_le32(v);
    return r;
}

void hx_pin_reset(struct hx_chip *c)
{
    if (!c->bus->reset || c->reset_skip) return;
    c->bus->reset(c->bus->ctx, c->reset_invert ? 1 : 0);
    sleep_ms(c, c->reset_low_ms ? c->reset_low_ms : 20);
    c->bus->reset(c->bus->ctx, c->reset_invert ? 0 : 1);
    sleep_ms(c, c->reset_high_ms ? c->reset_high_ms : 50);
}

int hx_interface_on(struct hx_chip *c)
{
    uint8_t dummy[4], a, b;
    if (hx_bus_read(c, HX_BUS_RDATA, dummy, 4)) return HX_ERR_BUS;   // wakes the bus
    for (unsigned k = 0; k < 4; ++k) c->dbg_dummy[k] = dummy[k];
    for (unsigned i = 0; i < 10; ++i) {
        if (burst(c, 0) || hx_bus_read(c, HX_BUS_CONTI, &a, 1) || hx_bus_read(c, HX_BUS_INCR4, &b, 1)) return HX_ERR_BUS;
        c->dbg_conti = a; c->dbg_incr4 = b;
        // Linux (himax_mcu_interface_on in hx83102 sense_off path) never checks the read-back: on this HX83102D the burst
        // registers read 0 and the chip still answers (bit-banged Linux sequence: status 0C, id 83 10 2D). Log only.
        if (a == 0x31 && b == 0x10) return HX_OK;
        sleep_ms(c, 1);
    }
    return HX_OK;
}

int hx_sense_off(struct hx_chip *c)
{
    uint32_t v = 0;
    for (unsigned i = 0; i < 10; ++i) {
        if (i == 0 || ((v & 0xFF) != 0xA5 && (v & 0xFF) != 0x00 && (v & 0xFF) != 0x87))
            hx_reg_write32(c, HX_ADDR_CTRL_FW_ISR, 0xA5);
        sleep_ms(c, 10);
        if (hx_reg_read32(c, HX_ADDR_CENTRAL_STATE, &v)) return HX_ERR_BUS;
        if ((v & 0xFF) != 0x05) break;                      // firmware not running: nothing to wait for
        if (hx_reg_read32(c, HX_ADDR_CTRL_FW_ISR, &v)) return HX_ERR_BUS;
        if ((v & 0xFF) == 0x87) break;
    }
    for (unsigned i = 0; i < 6; ++i) {
        uint8_t p = 0x27;
        if (hx_bus_write(c, HX_BUS_PSW_LB, &p, 1)) return HX_ERR_BUS;
        p = 0x95;
        if (hx_bus_write(c, HX_BUS_PSW_UB, &p, 1)) return HX_ERR_BUS;
        if (hx_reg_read32(c, HX_ADDR_CENTRAL_STATE, &v)) return HX_ERR_BUS;
        c->central_state = (uint8_t)v;
        if ((v & 0xFF) == 0x0C) {
            hx_reg_write32(c, HX_ADDR_TCON_RESET, 0);
            sleep_ms(c, 1);
            hx_reg_write32(c, HX_ADDR_TCON_RESET, 1);
            hx_reg_write32(c, HX_ADDR_ADC_RESET, 0);
            sleep_ms(c, 1);
            hx_reg_write32(c, HX_ADDR_ADC_RESET, 1);
            return HX_OK;
        }
        sleep_ms(c, 5);
        hx_pin_reset(c);
    }
    return HX_ERR_CHIP;
}

int hx_identify(struct hx_chip *c)
{
    c->stage = 1;
    hx_pin_reset(c);
    if (hx_interface_on(c)) return HX_ERR_CHIP;
    c->stage = 2;
    int r = hx_sense_off(c);
    if (r) return r;
    c->stage = 3;
    if (hx_reg_read(c, HX_ADDR_CHIP_ID, c->id, 4)) return HX_ERR_BUS;
    return c->id[3] == 0x83 && c->id[2] == 0x10 && c->id[1] == 0x2D ? HX_OK : HX_ERR_CHIP;
}

// Hardware CRC of len bytes of SRAM at addr (check_CRC); 0 means the data matches what the image carries.
static int sram_crc(struct hx_chip *c, uint32_t addr, uint32_t len, uint32_t *crc)
{
    uint32_t words = len / 4, v = 0;
    if (hx_reg_write32(c, HX_ADDR_RELOAD_FROM, addr) ||
        hx_reg_write32(c, HX_ADDR_RELOAD_CMD_BEAT, 0x00990000u | (words & 0xFFFF))) return HX_ERR_BUS;
    for (unsigned i = 0; i < 100; ++i) {
        if (hx_reg_read32(c, HX_ADDR_RELOAD_STATUS, &v)) return HX_ERR_BUS;
        if (!(v & 1)) return hx_reg_read32(c, HX_ADDR_RELOAD_CRC32, crc);
        sleep_ms(c, 1);
    }
    return HX_ERR_TIMEOUT;
}

int hx_download(struct hx_chip *c, const struct hx_fw *fw, uint8_t *cfg)
{
    c->stage = 10;
    hx_reg_write32(c, HX_ADDR_SYSTEM_RESET, 0x55);
    sleep_ms(c, 10);
    int r = hx_sense_off(c);
    if (r) return r;
    // The firmware entity (entry 0), checked by the reload engine: the image already carries its own CRC, so the
    // hardware result over the whole partition is 0 (himax_sram_write_crc_check).
    c->stage = 11;
    const struct hx_zf_part *e = &fw->part[0];
    unsigned tries = 0;
    do {
        if (hx_reg_write(c, e->sram, fw->data + e->offset, e->size)) return HX_ERR_BUS;
        if (sram_crc(c, e->sram, e->size, &c->fw_crc_hw)) return HX_ERR_BUS;
    } while (c->fw_crc_hw && ++tries < 3);
    if (c->fw_crc_hw) return HX_ERR_CRC;
    // The config partitions, packed into one window and compared with the software CRC of the same bytes.
    c->stage = 12;
    if (hx_fw_config(fw, cfg, HX_ZF_CFG_MAX)) return HX_ERR_FW;
    c->cfg_size = fw->cfg_size;
    c->parts = fw->count;
    c->cfg_crc_sw = hx_crc(cfg, fw->cfg_size);
    tries = 0;
    do {
        if (hx_reg_write(c, fw->cfg_base, cfg, fw->cfg_size)) return HX_ERR_BUS;
        if (sram_crc(c, fw->cfg_base, fw->cfg_size, &c->cfg_crc_hw)) return HX_ERR_BUS;
    } while (c->cfg_crc_hw != c->cfg_crc_sw && ++tries < 4);
    if (c->cfg_crc_hw != c->cfg_crc_sw) return HX_ERR_CRC;
    if (fw->overlay > 0) {
        const struct hx_zf_part *o = &fw->part[fw->overlay];
        uint32_t crc = 1;
        tries = 0;
        do {
            if (hx_reg_write(c, o->sram, fw->data + o->offset, o->size)) return HX_ERR_BUS;
            if (sram_crc(c, o->sram, o->size, &crc)) return HX_ERR_BUS;
        } while (crc && ++tries < 3);
        if (crc) return HX_ERR_CRC;
    }
    c->stage = 13;
    if (hx_reg_write32(c, HX_ADDR_SET_FRAME, 1)) return HX_ERR_BUS;               // n frame = 1 (firmware_update_0f)
    if (hx_reg_write32(c, HX_ADDR_FLASH_RELOAD, 0)) return HX_ERR_BUS;            // reload enabled (reload_disable(0))
    return HX_OK;
}

static int reload_to_active(struct hx_chip *c)
{
    uint32_t v = 0;
    for (unsigned i = 0; i < 10; ++i) {
        if (hx_reg_write32(c, HX_ADDR_RELOAD_TO_ACTIVE, 0xEC)) return HX_ERR_BUS;
        sleep_ms(c, 1);
        if (hx_reg_read32(c, HX_ADDR_RELOAD_TO_ACTIVE, &v)) return HX_ERR_BUS;
        if ((v & 0xFFFF) == 0x01EC) return HX_OK;
    }
    return HX_ERR_TIMEOUT;
}

int hx_start(struct hx_chip *c)
{
    uint8_t d[8];
    c->stage = 20;
    // Sense on (himax_hx83102d_sense_on, FlashMode 0 with HX_RST_PIN_FUNC): clear the ISR, pin reset, reload to active.
    if (hx_interface_on(c)) return HX_ERR_CHIP;
    hx_reg_write32(c, HX_ADDR_CTRL_FW_ISR, 0);
    sleep_ms(c, 10);
    hx_pin_reset(c);
    c->stage = 21;
    int r = reload_to_active(c);
    if (r) return r;
    sleep_ms(c, 50);
    // Touch information (himax_mcu_touch_information) from the running firmware.
    c->stage = 22;
    if (hx_reg_read(c, HX_ADDR_RX_TX_MAXPT, d, 8)) return HX_ERR_BUS;
    c->rx_num = d[2]; c->tx_num = d[3]; c->max_pt = d[4];
    if (hx_reg_read(c, HX_ADDR_XY_RES_ENABLE, d, 4)) return HX_ERR_BUS;
    c->xy_reverse = (d[1] & 0x04) != 0;
    if (hx_reg_read(c, HX_ADDR_XY_RES, d, 4)) return HX_ERR_BUS;
    c->y_res = (uint16_t)(d[0] << 8 | d[1]);
    c->x_res = (uint16_t)(d[2] << 8 | d[3]);
    if (hx_reg_read(c, HX_ADDR_INT_IS_EDGE, d, 4)) return HX_ERR_BUS;
    c->int_is_edge = (d[1] & 1) != 0;
    // Implausible information (seen: 9234x1027, 0 points) falls back to what Linux reads on guamp:
    // HX_RX_NUM 32, HX_TX_NUM 18, HX_MAX_PT 10, 720x1600, HX_INT_IS_EDGE 1 (TWRP dmesg).
    if (c->x_res < 100 || c->x_res > 4096 || c->y_res < 100 || c->y_res > 4096 || !c->max_pt || c->max_pt > HX_MAX_PT) {
        c->info_fallback = 1;
        c->rx_num = 32; c->tx_num = 18; c->max_pt = 10;
        c->x_res = 720; c->y_res = 1600; c->int_is_edge = 1;
    }
    c->stage = 23;
    return HX_OK;
}

int hx_read_event(struct hx_chip *c, uint8_t *buf)
{
    uint8_t v = 0x00;
    if (hx_bus_write(c, HX_BUS_AHB, &v, 1)) return HX_ERR_BUS;
    int r = hx_bus_read(c, HX_BUS_EVENT, buf, HX_EVENT_LEN);
    v = 0x01;
    if (hx_bus_write(c, HX_BUS_AHB, &v, 1)) return HX_ERR_BUS;
    return r;
}

int hx_parse_event(const uint8_t *buf, int x_max, int y_max, struct hx_report *out)
{
    unsigned sum = 0, zero = 0;
    for (unsigned i = 0; i < HX_EVENT_LEN; ++i) {sum += buf[i]; zero += buf[i] == 0;}
    out->count = 0;
    out->point_cnt = buf[HX_EVENT_POINT_CNT];
    if (zero == HX_EVENT_LEN) return HX_OK;                                          // empty stack: no fingers (release)
    if (sum & 0xFF) return HX_ERR_CHECKSUM;
    if (out->point_cnt == 0xFF || !(out->point_cnt & 0x0F)) return HX_OK;           // all fingers up
    for (unsigned i = 0; i < HX_MAX_PT; ++i) {
        const uint8_t *p = buf + 4 * i;
        int x = p[0] << 8 | p[1], y = p[2] << 8 | p[3];
        if (x > x_max || y > y_max) continue;                                         // 0xFFFF: slot not in use
        if (!x && !y) continue;                                                       // HX83102D leaves unused slots at 0,0
        out->slot[out->count] = (int)i;
        out->point[out->count].x = x;
        out->point[out->count].y = y;
        out->point[out->count].w = buf[HX_MAX_PT * 4 + i];
        ++out->count;
    }
    return HX_OK;
}
