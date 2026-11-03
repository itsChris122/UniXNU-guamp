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
// Firmware handshake words in DSRAM (himax_ic_core.h fw_addr_* / ovl_*, HX83102D support file): the download leaves them
// in a known state before the first sense on, as himax_zf_part_info and himax_mcu_power_on_init do.
#define HX_ADDR_SORTING_MODE 0x10007F04u     // fw_addr_sorting_mode_en: 0 = normal coordinate reporting
#define HX_ADDR_SMWP_ENABLE 0x10007F10u      // fw_addr_smwp_enable: 0 = no smart wake-up (gesture) mode
#define HX_ADDR_USB_DETECT 0x10007F38u       // fw_usb_detect_addr: A55AA55A = cable in, 0 = out
#define HX_ADDR_FW_DBG_MSG 0x10007F40u       // fw_addr_fw_dbg_msg_addr (diagnostics only)
#define HX_ADDR_OVL_HANDSHAKE 0x10007FFCu    // ovl_handshaking_addr: driver reply after loading a code overlay
#define HX_ADDR_2ND_FLASH_RELOAD 0x100072C0u // driver_addr_fw_define_2nd_flash_reload: reads C0 72 once reloaded
#define HX_ADDR_FW_VER 0x10007004u           // fw_addr_fw_ver_addr: panel ver, fw ver hi, lo
#define HX_ADDR_FW_CFG 0x10007084u           // fw_addr_fw_cfg_addr: touch cfg ver [2], display cfg ver [3]
#define HX_ADDR_FW_VENDOR 0x10007000u        // fw_addr_fw_vendor_addr: CID major [2], minor [3]
#define HX_ADDR_FW_STATE 0x900000F8u         // fw_addr_fw_state_addr (diagnostics only)
#define HX_ADDR_RAW_OUT_SEL 0x800204F4u      // hx83102d_fw_addr_raw_out_sel: 0 = no raw data output
#define HX_HANDSHAKE_PWD 0xA55AA55Au         // fw_func_handshaking_pwd
// Overlay handshake values (himax_ic_core.h, HX83102D): the firmware requests, the driver replies.
#define HX_OVL_GESTURE_REQUEST 0x11
#define HX_OVL_GESTURE_REPLY 0x22
#define HX_OVL_BORDER_REQUEST 0x55
#define HX_OVL_BORDER_REPLY 0x66
#define HX_OVL_SORTING_REQUEST 0x99
#define HX_OVL_SORTING_REPLY 0xAA
#define HX_OVL_FAULT 0xFF
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
    int info_fallback;                  // 1: touch information was implausible, Linux values used
    uint8_t dbg_dummy[4], dbg_conti, dbg_incr4;   // last interface_on replies, for the boot log
    unsigned reset_low_ms, reset_high_ms;          // 0: vendor timing (20 ms low, 50 ms high)
    int reset_invert, reset_skip;                  // reset line active high / do not touch it
    // Download options (set by the owner before hx_download; 0 = the Linux sequence).
    int usb_in;                     // 1: tell the firmware a USB cable is in (himax_mcu_usb_detect_set, IN)
    int ovl_force;                  // >0: load this partition index as the code overlay instead of the border one
    int no_recode;                  // 1: skip the second write of the firmware entity (Linux does it, see hx_download)
    // Results for the boot log.
    int ovl_loaded;                 // partition index of the code overlay written, -1 none
    uint8_t ovl_reply;              // reply written to HX_ADDR_OVL_HANDSHAKE
    uint32_t ovl_handshake;         // read back
    uint32_t usb_word, smwp_word, sort_word;     // read back after writing
    uint32_t recode_crc;            // hardware CRC of the second firmware write (0 = ok)
    uint32_t reload1, reload2;      // HX_ADDR_FLASH_RELOAD / HX_ADDR_2ND_FLASH_RELOAD at "reload OK"
    unsigned reload_polls;          // 10 ms polls until reload OK (200 = never)
    uint32_t ovl_seen;              // handshake word when the overlay swap stopped waiting
    unsigned ovl_wait_ms;           // how long it waited for the firmware's request
    uint8_t panel_ver, touch_cfg_ver, display_cfg_ver;
    uint16_t fw_ver, cid_ver;
};

struct hx_zf_part {uint32_t sram, size, offset;};
struct hx_fw {
    const uint8_t *data;
    size_t size;
    unsigned base;                  // 0 or HX_ZF_HEADER_1K
    unsigned count;                 // partitions, entry 0 = the firmware itself
    struct hx_zf_part part[HX_ZF_MAX_PARTS];
    uint32_t cfg_base, cfg_size;    // the packed config window (entries 1..count-1)
    // Code overlays (HX_CODE_OVERLAY, hx_parse_bin_cfg_data): table entries whose bytes 14..15 are 66 55 carry their
    // type in byte 13 (0 sorting, 1 gesture, 2 border); older images mark them by the SRAM address 0x20008CE0 instead
    // and get their type from the order. -1: no such entry.
    int ovl[3];
    int overlay;                    // the overlay the boot loads: ovl[2] (border, normal touch), as himax_zf_part_info
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
// The zero-flash download (himax_mcu_firmware_update_0f + himax_zf_part_info, HX83102D): system reset, safe mode,
// firmware and config to SRAM with hardware CRC checks, firmware again, SMWP off / cable state, the border code overlay
// and its handshake reply, raw out / sorting mode cleared, frame setting. Then hx_start.
int hx_download(struct hx_chip *c, const struct hx_fw *fw, uint8_t *cfg_scratch);
// After the download (himax_mcu_0f_operation): reload enable, sense on, wait for "reload OK", sense off, versions and
// touch information, sense on again. HX_OK once the firmware reports active.
int hx_start(struct hx_chip *c);
// Sense on as himax_hx83102d_sense_on(0): interface on, clear the ISR, pin reset, reload to active.
int hx_sense_on(struct hx_chip *c);
// Overlay swap while the firmware runs (hx83102d_0f_overlay): wait up to wait_ms for the firmware's request in the
// handshake word, write partition idx with its hardware CRC, write the reply (HX_OVL_FAULT on a CRC failure), then
// (reset_after) sense on again, as the border swap of resume does. Without a request: force = 1 does the same from
// safe mode (sense off, overlay, reply, sense on), force = 0 leaves everything. Returns 1 after a request, 2 forced,
// 0 nothing done, < 0 error.
int hx_overlay_swap(struct hx_chip *c, const struct hx_fw *fw, int idx, uint8_t request, uint8_t reply, int reset_after,
                    unsigned wait_ms, int force);
// Smart wake-up mode word (himax_mcu_set_SMWP_enable): A55AA55A on, 0 off, written and read back. Returns the read value.
uint32_t hx_set_smwp(struct hx_chip *c, int on);
// Gesture report in the first 8 bytes of the event stack (himax_wake_event_parse: HX_REPORT_SMWP_EVENT reads
// GEST_PTLG_ID_LEN + GEST_PTLG_HDR_LEN): 4 equal id bytes from gest_event[] (0x80 double tap, 0x81 finger gesture,
// 0x01..0x0F letters / swipes, 0x1D..0x54 others), then CC 44. Returns the id, 0 when it is no gesture.
#define HX_GEST_DOUBLE_TAP 0x80
int hx_parse_gesture(const uint8_t *buf);
// One event stack read (HX_EVENT_LEN bytes) into buf.
int hx_read_event(struct hx_chip *c, uint8_t *buf);
// Parses an event stack. HX_OK, HX_ERR_CHECKSUM for a torn or empty stack. x_max/y_max bound valid coordinates.
int hx_parse_event(const uint8_t *buf, int x_max, int y_max, struct hx_report *out);

#ifdef __cplusplus
}
#endif
#endif
