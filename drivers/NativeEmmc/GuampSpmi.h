// SPDX-License-Identifier: BSD-2-Clause-Patent
// Qualcomm SPMI PMIC arbiter v5 (SM6115), polled register reads and writes from one execution environment. Register
// layout and sequences follow the study of Linux drivers/spmi/spmi-pmic-arb.c (no code copied):
//   version            core + 0x0 (v5: 0x50000000 <= v < 0x70000000)
//   peripheral count   core + 0x4, bits 10:0
//   APID n -> PPID     core + 0x900 + 4n, PPID = bits 19:8 (sid << 8 | peripheral), 0 = unused
//   APID n write owner cnfg + 0x700 + 4n, bits 2:0 (the EE allowed to write through its channel)
//   read channel       obsrvr + 0x10000 * ee + 0x80 * apid
//   write channel      chnls + 0x10000 * apid (only when the APID's write owner is our EE)
//   channel registers  CMD 0x00, STATUS 0x08 (bit 0 done, 1 failure, 2 denied, 3 dropped), WDATA0/1 0x10/0x14,
//                      RDATA0/1 0x18/0x1C; data little endian, up to 8 bytes per command
//   command word       opcode << 27 | (address & 0xff) << 4 | (bytes - 1); EXT_WRITEL = 0, EXT_READL = 1
// Nothing here touches hardware directly: the owner supplies the register accessors. Host-tested by tests/spmi_host.c.
#ifndef GUAMP_SPMI_H
#define GUAMP_SPMI_H
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

#define SPMI_V5_MIN 0x50000000u
#define SPMI_V7_MIN 0x70000000u
#define SPMI_MAX_APIDS 512
#define SPMI_MAX_PPID 4096
#define SPMI_TIMEOUT_POLLS 1000

#define SPMI_OK 0
#define SPMI_ERR_VERSION (-95)
#define SPMI_ERR_NODEV (-19)
#define SPMI_ERR_PERM (-1)
#define SPMI_ERR_IO (-5)
#define SPMI_ERR_TIMEOUT (-110)
#define SPMI_ERR_PARAM (-22)

struct spmi_io {
    void *ctx;
    // region 0 core, 1 cnfg, 2 obsrvr, 3 chnls; offsets inside the region
    uint32_t (*rd)(void *ctx, unsigned region, uint32_t off);
    void (*wr)(void *ctx, unsigned region, uint32_t off, uint32_t value);
    void (*delay_us)(void *ctx, unsigned us);
};
enum {SPMI_CORE, SPMI_CNFG, SPMI_OBSRVR, SPMI_CHNLS};

struct spmi_arb {
    struct spmi_io io;
    unsigned ee;
    uint32_t version;
    unsigned apids;
    uint16_t ppid_to_apid[SPMI_MAX_PPID];   // apid + 1, 0 = no mapping
    uint8_t write_owner[SPMI_MAX_APIDS];
    uint32_t last_status;
};

// Reads the version and the APID table. A PPID that several APIDs map takes the one our EE may write, else the first.
int spmi_init(struct spmi_arb *a, const struct spmi_io *io, unsigned ee);
int spmi_read(struct spmi_arb *a, unsigned sid, uint16_t addr, uint8_t *buf, size_t len);
int spmi_write(struct spmi_arb *a, unsigned sid, uint16_t addr, const uint8_t *buf, size_t len);
static inline int spmi_read8(struct spmi_arb *a, unsigned sid, uint16_t addr, uint8_t *v) {return spmi_read(a, sid, addr, v, 1);}
static inline int spmi_write8(struct spmi_arb *a, unsigned sid, uint16_t addr, uint8_t v) {return spmi_write(a, sid, addr, &v, 1);}

#ifdef __cplusplus
}
#endif
#endif
