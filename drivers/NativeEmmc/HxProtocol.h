// SPDX-License-Identifier: BSD-2-Clause-Patent
// Himax HX83102D zero-flash ("0flash") in-cell touch controller on SPI, as the Moto G9 Play (guamp) uses it. Register
// layout, sequences and constants follow the study of the Motorola himax_0flash_mmi driver (motorola-kernel-modules,
// drivers/input/touchscreen/himax_0flash_mmi: himax_platform_QCT_SPI.c, himax_ic_incell_core.c, himax_ic_core.h,
// support_ic/himax_0flash_mmi_hx83102d/himax_ic_HX83102d.c, himax_common.c); no code copied. Nothing here touches
// hardware: the owner supplies one SPI transfer, a sleep and the reset line. Host-tested by tests/hx_host.c.
#ifndef HX_PROTOCOL_H
#define HX_PROTOCOL_H
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

// SPI framing (himax_platform_QCT_SPI.c): write = F2 <bus register> <data...>, read = F3 <bus register> 00 then the data,
// all in one chip-select frame.
#define HX_SPI_WRITE 0xF2
#define HX_SPI_READ 0xF3
// Bus registers (himax_ic_core.h ic_adr_*): the AHB bridge into the controller's 32-bit address space.
#define HX_BUS_ADDR 0x00           // 4 address bytes (little endian), followed by data on a write
#define HX_BUS_RDATA 0x08          // read data port
#define HX_BUS_DIR 0x0C            // 0x00 = start the read at the address just written
#define HX_BUS_INCR4 0x0D          // 0x10 | 1 = auto increment by 4 (burst)
#define HX_BUS_CONTI 0x13          // 0x31 = continuous access
#define HX_BUS_AHB 0x11            // event stack: 0x00 burst off around the read, 0x01 back on
#define HX_BUS_EVENT 0x30          // touch event stack
#define HX_BUS_PSW_LB 0x31         // safe mode password 0x27, 0x95
#define HX_BUS_PSW_UB 0x32
// Controller addresses (himax_ic_core.h fw_*/zf_*/driver_* and the HX83102D support file).
#define HX_ADDR_CHIP_ID 0x900000D0u          // reads 83 10 2D for HX83102D
#define HX_ADDR_CENTRAL_STATE 0x900000A8u    // 0x0C in safe mode, 0x05 running
#define HX_ADDR_SYSTEM_RESET 0x90000018u     // 0x55
#define HX_ADDR_CTRL_FW_ISR 0x9000005Cu      // 0xA5 stops the firmware, 0x00 clears
#define HX_ADDR_SAFE_MODE_RELEASE 0x90000098u
#define HX_ADDR_FLAG_RESET_EVENT 0x900000E4u
#define HX_ADDR_RELOAD_TO_ACTIVE 0x90000048u // 0xEC; reads back 01 EC once active
#define HX_ADDR_TCON_RESET 0x80020020u
#define HX_ADDR_ADC_RESET 0x80020094u
#define HX_ADDR_RELOAD_STATUS 0x80050000u    // bit 0 busy
#define HX_ADDR_RELOAD_CRC32 0x80050018u
#define HX_ADDR_RELOAD_FROM 0x80050020u
#define HX_ADDR_RELOAD_CMD_BEAT 0x80050028u  // 00 99 <words high> <words low>: hardware CRC over SRAM
#define HX_ADDR_FLASH_RELOAD 0x10007F00u     // 0 = reload enabled (the zero-flash flow writes 0 before sensing)
#define HX_ADDR_SET_FRAME 0x10007294u
#define HX_ADDR_RX_TX_MAXPT 0x100070F4u
#define HX_ADDR_XY_RES_ENABLE 0x100070F8u
#define HX_ADDR_XY_RES 0x100070FCu
#define HX_ADDR_INT_IS_EDGE 0x10007088u
// Zero-flash image (himax_zf_part_info): a 64 KiB firmware followed by a partition table of 16-byte entries
// {SRAM address LE32, size LE32, offset in the file LE32, reserved}; byte 12 of the first entry is the partition count.
// Some images carry an extra 1 KiB header first (bytes 0..7 zero and byte 0x0E = 0x87).
#define HX_ZF_TABLE 0x10000u
#define HX_ZF_HEADER_1K 0x400u
#define HX_ZF_MAX_PARTS 32
#define HX_ZF_CFG_MAX 0x4000u                // FW_BIN_16K_SZ: the config partitions packed into one SRAM window
#define HX_WRITE_CHUNK 4088u                 // PACKET_MAX_SZ - 8 data bytes per bus write
#define HX_READ_CHUNK 256u                   // FLASH_RW_MAX_LEN

#define HX_MAX_PT 10
// Event stack for HX_MAX_PT fingers (himax_report_data_init, calculate_point_number): 4 bytes per finger
// {x hi, x lo, y hi, y lo}, then one width byte per finger, then the point count byte, two status bytes; the whole
// stack sums to 0 modulo 256.
#define HX_EVENT_LEN ((HX_MAX_PT + HX_MAX_PT / 4 + 2) * 4)
#define HX_EVENT_POINT_CNT (HX_MAX_PT * 4 + ((HX_MAX_PT / 4) + 1) * 4)

#define HX_OK 0
#define HX_ERR_BUS (-5)
#define HX_ERR_CHIP (-19)
#define HX_ERR_FW (-22)
#define HX_ERR_CRC (-74)
#define HX_ERR_TIMEOUT (-110)
#define HX_ERR_CHECKSUM (-84)

struct hx_bus {
    void *ctx;
    // One chip-select frame of len bytes, full duplex (rx may be null).
    int (*xfer)(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len);
    void (*sleep_ms)(void *ctx, unsigned ms);
    void (*reset)(void *ctx, int level);     // the reset line: 0 = asserted (low), 1 = released
};

struct hx_chip {
    const struct hx_bus *bus;
    uint8_t *frame;                 // HX_WRITE_CHUNK + 8 bytes of scratch for one bus frame (tx and rx share it)
    uint8_t *rx;                    // HX_WRITE_CHUNK + 8 bytes
    uint8_t id[4];
    uint8_t central_state;
    uint32_t fw_crc_hw, cfg_crc_hw, cfg_crc_sw;
    unsigned parts, cfg_size;
    int stage;                      // last step reached, for the diagnostic rows
    // Panel information read back from the running firmware.
    uint8_t rx_num, tx_num, max_pt;
    uint16_t x_res, y_res;
    int xy_reverse, int_is_edge;
};

struct hx_zf_part {uint32_t sram, size, offset;};
struct hx_fw {
    const uint8_t *data;
    size_t size;
    unsigned base;                  // 0 or HX_ZF_HEADER_1K
    unsigned count;                 // partitions, entry 0 = the firmware itself
    struct hx_zf_part part[HX_ZF_MAX_PARTS];
    uint32_t cfg_base, cfg_size;    // the packed config window (entries 1..count-1)
    int overlay;                    // first non-empty code overlay entry (SRAM 0x2xxxxxxx after the firmware), -1 if none
};

struct hx_point {int x, y, w;};
struct hx_report {
    unsigned count;                 // contacts in point[] (only valid ones, in slot order)
    int slot[HX_MAX_PT];
    struct hx_point point[HX_MAX_PT];
    uint8_t point_cnt;              // the raw point count byte
};

// The CRC-32C variant the controller's reload engine computes (Calculate_CRC_with_AP): little-endian 32-bit words,
// reflected polynomial 0x82F63B78, initial value 0xFFFFFFFF, no final xor.
uint32_t hx_crc(const uint8_t *data, size_t len);
int hx_fw_parse(struct hx_fw *fw, const uint8_t *data, size_t size);
// Builds the packed config window of a parsed image into out (fw->cfg_size bytes, at most HX_ZF_CFG_MAX).
int hx_fw_config(const struct hx_fw *fw, uint8_t *out, size_t cap);

int hx_bus_write(struct hx_chip *c, uint8_t reg, const uint8_t *data, size_t len);
int hx_bus_read(struct hx_chip *c, uint8_t reg, uint8_t *data, size_t len);
int hx_reg_write(struct hx_chip *c, uint32_t addr, const uint8_t *data, size_t len);
int hx_reg_read(struct hx_chip *c, uint32_t addr, uint8_t *data, size_t len);
int hx_reg_write32(struct hx_chip *c, uint32_t addr, uint32_t value);
int hx_reg_read32(struct hx_chip *c, uint32_t addr, uint32_t *value);

void hx_pin_reset(struct hx_chip *c);
int hx_interface_on(struct hx_chip *c);
// Stops the firmware and enters safe mode (the HX83102D sequence, TCON and ADC reset on success).
int hx_sense_off(struct hx_chip *c);
// Reset, safe mode, chip id. HX_OK when the id is 83 10 2D.
int hx_identify(struct hx_chip *c);
// The zero-flash download: system reset, safe mode, firmware and config to SRAM with hardware CRC checks, frame setting,
// flash reload enable. Then hx_start.
int hx_download(struct hx_chip *c, const struct hx_fw *fw, uint8_t *cfg_scratch);
// Touch information, then sense on (pin reset, reload to active). HX_OK once the firmware reports active.
int hx_start(struct hx_chip *c);
// One event stack read (HX_EVENT_LEN bytes) into buf.
int hx_read_event(struct hx_chip *c, uint8_t *buf);
// Parses an event stack. HX_OK, HX_ERR_CHECKSUM for a torn or empty stack. x_max/y_max bound valid coordinates.
int hx_parse_event(const uint8_t *buf, int x_max, int y_max, struct hx_report *out);

#ifdef __cplusplus
}
#endif
#endif
