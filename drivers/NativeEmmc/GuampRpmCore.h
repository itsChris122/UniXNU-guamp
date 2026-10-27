// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp (Moto G9 Play, SM6115 / bengal) RPM client core: GLINK over the RPM message RAM plus the rpm-smd request/ack
// encoding. Pure code: every message RAM access goes through rpm_io (GuampRpm.inc maps the RAM and the APCS doorbell,
// tests/rpm_host.cpp is a fake message RAM with a fake RPM peer). Written from a study of the vendor tree
// android_kernel_motorola_sm6225 (no code copied): drivers/rpmsg/qcom_glink_rpm.c (TOC, FIFOs), qcom_glink_native.c
// (commands, handshake), rpm-smd.c (request/ack layout), drivers/regulator/rpm-smd-regulator.c and
// drivers/clk/qcom/clk-smd-rpm.c (resource types, keys), include/linux/soc/qcom/smd-rpm.h, msm-bus-board.h,
// bengal.dtsi / pm6125-rpm-regulator.dtsi / bengal-regulator.dtsi / bengal-gpu.dtsi / bengal-bus.dtsi.
//
// Message RAM (bengal.dtsi rpm_msg_ram 0x45f0000 len 0x7000): the last 256 bytes are a TOC {magic "grt0", count,
// entries {id, offset, size}}; entry "ap2r" is the apps->RPM FIFO, "r2ap" RPM->apps. At offset: tail word, head word,
// then the FIFO of `size` bytes. The writer owns head, the reader owns tail. Every packet is padded to 8 bytes.
// Doorbell (apcs_glb 0xf111000, qcom,bengal-apcs-hmss-global .data 8, mboxes <&apcs_glb 0>): write BIT(0) at +8.
// The RPM edge is intentless (qcom_glink_native_probe(..., true)) with features 0: no intents, no RX_DONE.
//
// Session (Linux order): reset our indices (tx head = 0, rx tail = 0), VERSION(1, 0); the RPM answers VERSION_ACK and
// opens "rpm_requests" (OPEN rcid); we answer OPEN_ACK(rcid) and OPEN(lcid), the RPM answers OPEN_ACK(lcid). If the
// RPM does not open within local_after_us we open first (the local-initiated order of qcom_glink_create_local).
// Requests: rpm-smd v0 format (bengal: the qcom,rpm-smd node has no reg, so rpm_msg_fmt_ver stays 0), one outstanding
// request at a time, acked by message id; a stale ack (from a request that timed out) is counted and dropped.
#ifndef GUAMP_RPM_CORE_H
#define GUAMP_RPM_CORE_H
#include <stdint.h>
#include <stddef.h>

// --- addresses (bengal.dtsi) --------------------------------------------------------------------------------------
#define RPM_MSG_RAM_PHYS 0x045f0000ULL
#define RPM_MSG_RAM_SIZE 0x7000u
#define RPM_APCS_PHYS 0x0f111000ULL
#define RPM_APCS_SIZE 0x1000u
#define RPM_APCS_IPC 0x8u                   // qcom-apcs-ipc-mailbox.c: bengal offset 8
#define RPM_APCS_BIT (1u << 0)              // mboxes = <&apcs_glb 0>

// --- qcom_glink_rpm.c -------------------------------------------------------------------------------------------------
#define RPM_TOC_SIZE 256u
#define RPM_TOC_MAGIC 0x67727430u           // "grt0"
#define RPM_TOC_MAX ((RPM_TOC_SIZE - 8u) / 12u)
#define RPM_TX_FIFO_ID 0x61703272u          // "ap2r"
#define RPM_RX_FIFO_ID 0x72326170u          // "r2ap"

// --- qcom_glink_native.c ------------------------------------------------------------------------------------------
enum : uint16_t {
  GL_VERSION = 0, GL_VERSION_ACK = 1, GL_OPEN = 2, GL_CLOSE = 3, GL_OPEN_ACK = 4, GL_INTENT = 5, GL_RX_DONE = 6,
  GL_RX_INTENT_REQ = 7, GL_RX_INTENT_REQ_ACK = 8, GL_TX_DATA = 9, GL_CLOSE_ACK = 11, GL_TX_DATA_CONT = 12,
  GL_READ_NOTIF = 13, GL_RX_DONE_W_REUSE = 14, GL_SIGNALS = 15
};
#define GL_VERSION_1 1u
#define GL_NAME_SIZE 32u
#define RPM_CHANNEL "rpm_requests"          // rpm-smd.c rpmsg_driver_rpm_id_table / bengal.dtsi rpm-channel-name
#define RPM_LCID 1u                          // first id of idr_alloc_cyclic(RPM_GLINK_CID_MIN = 1)

// --- rpm-smd.c ----------------------------------------------------------------------------------------------------
enum : unsigned { RPM_SET_ACTIVE = 0, RPM_SET_SLEEP = 1 };            // enum msm_rpm_set
enum : unsigned { RPM_FMT_V0 = 0, RPM_FMT_V1 = 1 };
#define RPM_SERVICE_REQ 0x00716572u         // "req\0" (msm_rpm_request_service_v1)
#define RPM_ACK_MSG_ID 0x2367736du          // "msg#": sub-message carrying the acked id
#define RPM_ACK_ERR 0x00727265u             // "err\0": sub-message carrying an error string
#define RPM_V1_ACK 3u                        // RPM_V1_ACK_SERVICE
#define RPM_V1_NACK 4u                       // RPM_V1_NACK_SERVICE
#define RPM_MAX_RX 128u                      // MAX_ERR_BUFFER_SIZE
#define RPM_MAX_KVP 4u

// resource types (smd-rpm.h, msm-bus-board.h; little-endian four-character codes)
#define RPM_RES_LDOA 0x616f646cu            // "ldoa"
#define RPM_RES_SMPA 0x61706d73u            // "smpa"
#define RPM_RES_CLK0 0x306b6c63u            // "clk0" QCOM_SMD_RPM_MISC_CLK
#define RPM_RES_CLK1 0x316b6c63u            // "clk1" QCOM_SMD_RPM_BUS_CLK
#define RPM_RES_CLK2 0x326b6c63u            // "clk2" QCOM_SMD_RPM_MEM_CLK
#define RPM_RES_BMAS 0x73616d62u            // "bmas" RPM_BUS_MASTER_REQ
#define RPM_RES_BSLV 0x766c7362u            // "bslv" RPM_BUS_SLAVE_REQ
// keys
#define RPM_KEY_SWEN 0x6e657773u            // "swen" regulator software enable
#define RPM_KEY_UV 0x00007675u              // "uv"   regulator voltage
#define RPM_KEY_MA 0x0000616du              // "ma"   regulator current
#define RPM_KEY_KHZ 0x007a484bu             // "KHz"  clock rate (QCOM_RPM_SMD_KEY_RATE)
#define RPM_KEY_ENAB 0x62616e45u            // "Enab" branch clock enable (QCOM_RPM_SMD_KEY_ENABLE)
#define RPM_KEY_BW 0x00007762u              // "bw"   bus bandwidth, 8 bytes (RPM_SLAVE_FIELD_BW)

// --- guamp board (bengal clk-smd-rpm.c, pm6125-rpm-regulator.dtsi, bengal-regulator.dtsi, bengal.dtsi) -------------
// clk0: 0 bi_tcxo (XO), 1 qdss, 2 scaling enable (QCOM_RPM_SCALING_ENABLE_ID); clk1: 0 snoc_periph, 1 cnoc, 2 snoc,
// 5 snoc_lpass; clk2: 0 bimc.
#define RPM_CLK0_XO 0u
#define RPM_CLK0_QDSS 1u
#define RPM_CLK1_CNOC 1u
#define RPM_CLK1_SNOC 2u
#define RPM_CLK2_BIMC 0u
#define RPM_KHZ_MAX 0x7fffffffu             // clk-smd-rpm.c probe: clk_vote_bimc(bimc, INT_MAX) on bengal

// BT supplies (bt_wcn3990: qca,bt-vdd-*-supply, bluetooth-power.c order io, xo, core, pa = kBtRails order). Voltage:
// what the regulator core picks for regulator_set_voltage(bt min, bt max) inside the rail constraints of
// bengal-regulator.dtsi (the lowest voltage in both ranges): L9A 1.8-2.0 V & 1.7-1.9 -> 1.8 V; L16A 1.704-1.904 &
// 1.7-1.9 -> 1.704 V; L17A 1.152-1.384 & 1.304 -> 1.304 V; L23A 3.2-3.4 & 3.0-3.312 -> 3.2 V. qcom,set = <3>: active
// and sleep set. rpm-smd-regulator sends the pending voltage together with swen on enable (one request per set).
struct rpm_bt_ldo { const char *name; uint32_t id, uv; };
static const rpm_bt_ldo kRpmBtLdo[4] = {
    {"L9A", 9, 1800000}, {"L16A", 16, 1704000}, {"L17A", 17, 1304000}, {"L23A", 23, 3200000}};

// GPU (bengal-gpu.dtsi kgsl clocks + qcom,msm-bus "grp3d" GPU -> EBI, bengal-bus.dtsi fab_bimc / fab_sys_noc /
// fab_config_noc): apb_pclk = RPM_SMD_QDSS_CLK (branch, both sets), the BIMC fabric clock behind gcc_gpu_memnoc_gfx /
// gcc_bimc_gpu_axi, SNOC behind gcc_gpu_snoc_dvm_gfx, CNOC (register path; Linux also holds cnoc/snoc keepalive votes).
// The GPU master (mas_gpu_cdsp_bimc) is qcom,ap-owned and kgsl's ab is 0, so Linux sends no bmas/bslv bandwidth for it.
struct rpm_vote { const char *name; uint32_t type, id, key; bool sleep; };
static const rpm_vote kRpmGpuVotes[4] = {
    {"qdss", RPM_RES_CLK0, RPM_CLK0_QDSS, RPM_KEY_ENAB, true},
    {"bimc", RPM_RES_CLK2, RPM_CLK2_BIMC, RPM_KEY_KHZ, false},
    {"snoc", RPM_RES_CLK1, RPM_CLK1_SNOC, RPM_KEY_KHZ, false},
    {"cnoc", RPM_RES_CLK1, RPM_CLK1_CNOC, RPM_KEY_KHZ, false}};

// --- errors -------------------------------------------------------------------------------------------------------
enum : int {
  RPM_OK = 0, RPM_ERR_TOC = -1, RPM_ERR_FIFO = -2, RPM_ERR_TXFULL = -3, RPM_ERR_VERSION = -4, RPM_ERR_OPEN = -5,
  RPM_ERR_ACK = -6, RPM_ERR_PROTO = -7, RPM_ERR_CLOSED = -8, RPM_ERR_PARAM = -9, RPM_ERR_NACK = -10,
  RPM_ERR_INV_RSC = -11, RPM_ERR_NOMAP = -12, RPM_ERR_BADACK = -13
};
static inline const char *rpm_err_name(int rc) {
  switch (rc) {
    case RPM_OK: return "ok";
    case RPM_ERR_TOC: return "no-toc";
    case RPM_ERR_FIFO: return "no-fifo";
    case RPM_ERR_TXFULL: return "tx-full";
    case RPM_ERR_VERSION: return "version-timeout";
    case RPM_ERR_OPEN: return "open-timeout";
    case RPM_ERR_ACK: return "ack-timeout";
    case RPM_ERR_PROTO: return "protocol";
    case RPM_ERR_CLOSED: return "closed";
    case RPM_ERR_PARAM: return "param";
    case RPM_ERR_NACK: return "nack";
    case RPM_ERR_INV_RSC: return "no-resource";
    case RPM_ERR_NOMAP: return "no-map";
    case RPM_ERR_BADACK: return "bad-ack";
    default: return "?";
  }
}

// --- io -----------------------------------------------------------------------------------------------------------
struct rpm_io {
  void *ctx;
  uint32_t (*rd)(void *ctx, uint32_t off);            // 32-bit read, off 4-aligned, below the RAM size
  void (*wr)(void *ctx, uint32_t off, uint32_t v);    // 32-bit write: only our tx FIFO, tx head, rx tail
  void (*barrier)(void *ctx);                         // FIFO data visible before the index update / doorbell
  void (*kick)(void *ctx);                            // APCS IPC doorbell to the RPM
  void (*delay_us)(void *ctx, unsigned us);
};

struct rpm_pipe { uint32_t desc, len; };              // tail at desc, head at desc + 4, FIFO at desc + 8
struct rpm_kvp { uint32_t key, nbytes; uint64_t value; };   // nbytes 4 or 8, little-endian
struct rpm_result { uint32_t msg_id, code, us; char text[32]; };

struct rpm_link {
  const rpm_io *io;
  uint32_t ram_size;
  unsigned fmt;
  uint32_t toc_magic, toc_count, toc_bad;
  rpm_pipe tx, rx;
  uint32_t tx_tail0, tx_head0, rx_tail0, rx_head0;   // indices found before the reset (diagnostics)
  // glink
  uint16_t lcid, rcid;
  uint32_t features;
  unsigned version_sends;
  bool version_acked, remote_open, open_sent, open_ack_sent, open_acked, up, broken, local_first;
  uint16_t bad_cmd;
  // reassembly of one RPM message
  uint8_t msg[RPM_MAX_RX];
  uint32_t msg_len;
  bool msg_over;
  // request in flight
  uint32_t next_id, wait_id;
  bool acked;
  int ack_rc;
  uint32_t ack_code;
  char ack_text[32];
  // counters
  uint32_t rx_cmds, acks, stale_acks, read_notifs, intents, signals, closes, other_opens, dropped, kicks;
};

static inline uint32_t rpm_get32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | static_cast<uint32_t>(p[1]) << 8 | static_cast<uint32_t>(p[2]) << 16 |
         static_cast<uint32_t>(p[3]) << 24;
}
static inline void rpm_put32(uint8_t *p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v); p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16); p[3] = static_cast<uint8_t>(v >> 24);
}
static inline uint32_t rpm_align8(uint32_t n) { return (n + 7u) & ~7u; }

// Ring arithmetic of qcom_glink_rpm.c: bytes the reader may consume, bytes the writer may fill.
static inline uint32_t rpm_ring_used(uint32_t head, uint32_t tail, uint32_t len) { return head >= tail ? head - tail : len - tail + head; }
static inline uint32_t rpm_ring_free(uint32_t head, uint32_t tail, uint32_t len) { return tail <= head ? len - head + tail : tail - head; }

static inline uint32_t rpm_rd(const rpm_link *l, uint32_t off) { return l->io->rd(l->io->ctx, off); }
static inline void rpm_wr(const rpm_link *l, uint32_t off, uint32_t v) { l->io->wr(l->io->ctx, off, v); }
static inline void rpm_kick(rpm_link *l) { l->io->barrier(l->io->ctx); l->io->kick(l->io->ctx); l->kicks = l->kicks + 1; }

static inline void rpm_link_init(rpm_link *l, const rpm_io *io, uint32_t ram_size, unsigned fmt) {
  *l = rpm_link{};
  l->io = io;
  l->ram_size = ram_size;
  l->fmt = fmt == RPM_FMT_V1 ? RPM_FMT_V1 : RPM_FMT_V0;
  l->next_id = 2;   // rpm-smd.c: ids 0 and 1 are reserved
}

// Read-only: TOC and both FIFO descriptors (glink_rpm_parse_toc), plus the indices as found.
static inline int rpm_parse_toc(rpm_link *l) {
  if (l->ram_size < RPM_TOC_SIZE || l->ram_size % 4) return RPM_ERR_TOC;
  const uint32_t base = l->ram_size - RPM_TOC_SIZE;
  l->toc_magic = rpm_rd(l, base);
  l->toc_count = rpm_rd(l, base + 4);
  l->tx = rpm_pipe{};
  l->rx = rpm_pipe{};
  l->toc_bad = 0;
  if (l->toc_magic != RPM_TOC_MAGIC || l->toc_count > RPM_TOC_MAX) return RPM_ERR_TOC;
  for (uint32_t i = 0; i < l->toc_count; ++i) {
    const uint32_t e = base + 8 + 12 * i;
    const uint32_t id = rpm_rd(l, e), off = rpm_rd(l, e + 4), size = rpm_rd(l, e + 8);
    if ((id != RPM_TX_FIFO_ID && id != RPM_RX_FIFO_ID)) continue;
    if (off % 4 || !size || size % 8 || off > l->ram_size || size > l->ram_size || off + 8 + size > l->ram_size) {
      l->toc_bad = l->toc_bad + 1;
      continue;
    }
    (id == RPM_TX_FIFO_ID ? l->tx : l->rx) = rpm_pipe{off, size};
  }
  if (!l->tx.len || !l->rx.len) return RPM_ERR_FIFO;
  const uint32_t te = l->tx.desc + 8 + l->tx.len, re = l->rx.desc + 8 + l->rx.len;
  if (l->tx.desc < re && l->rx.desc < te) return RPM_ERR_FIFO;   // overlapping FIFOs
  l->tx_tail0 = rpm_rd(l, l->tx.desc);
  l->tx_head0 = rpm_rd(l, l->tx.desc + 4);
  l->rx_tail0 = rpm_rd(l, l->rx.desc);
  l->rx_head0 = rpm_rd(l, l->rx.desc + 4);
  return RPM_OK;
}

// --- transmit -----------------------------------------------------------------------------------------------------
// One packet, zero-padded to 8 bytes (glink_rpm_tx_write), then the head and the doorbell. Never fills the FIFO
// completely (head == tail would read as empty).
static inline int rpm_tx_try(rpm_link *l, const uint8_t *p, uint32_t n) {
  const uint32_t len = l->tx.len, fifo = l->tx.desc + 8;
  const uint32_t tail = rpm_rd(l, l->tx.desc), head = rpm_rd(l, l->tx.desc + 4);
  if (tail >= len || head >= len || head % 4 || tail % 4) { l->broken = true; return RPM_ERR_PROTO; }
  const uint32_t padded = rpm_align8(n);
  if (padded >= rpm_ring_free(head, tail, len)) return RPM_ERR_TXFULL;
  uint32_t pos = head;
  for (uint32_t i = 0; i < padded; i += 4) {
    uint32_t w = 0;
    for (uint32_t k = 0; k < 4; ++k)
      if (i + k < n) w |= static_cast<uint32_t>(p[i + k]) << (8 * k);
    rpm_wr(l, fifo + pos, w);
    pos += 4;
    if (pos >= len) pos = 0;
  }
  l->io->barrier(l->io->ctx);
  rpm_wr(l, l->tx.desc + 4, pos);
  rpm_kick(l);
  return RPM_OK;
}

static inline int rpm_send_cmd(rpm_link *l, uint16_t cmd, uint16_t p1, uint32_t p2) {
  uint8_t m[8];
  rpm_put32(m, static_cast<uint32_t>(cmd) | static_cast<uint32_t>(p1) << 16);
  rpm_put32(m + 4, p2);
  return rpm_tx_try(l, m, 8);
}

static inline int rpm_send_open(rpm_link *l) {
  uint8_t m[8 + GL_NAME_SIZE] = {};
  const char *name = RPM_CHANNEL;
  uint32_t n = 0;
  while (name[n]) { m[8 + n] = static_cast<uint8_t>(name[n]); n = n + 1; }
  n = n + 1;                                             // name_len counts the NUL
  rpm_put32(m, static_cast<uint32_t>(GL_OPEN) | static_cast<uint32_t>(l->lcid) << 16);
  rpm_put32(m + 4, n);
  const int rc = rpm_tx_try(l, m, 8 + n);
  if (!rc) {
    l->open_sent = true;
    l->local_first = !l->remote_open;
  }
  return rc;
}

static inline int rpm_send_version(rpm_link *l) {
  const int rc = rpm_send_cmd(l, GL_VERSION, GL_VERSION_1, l->features);
  if (!rc) l->version_sends = l->version_sends + 1;
  return rc;
}

// --- rpm-smd request / ack ----------------------------------------------------------------------------------------
// Returns the request length, 0 when it does not fit or a field is out of range.
static inline uint32_t rpm_encode_request(uint8_t *out, uint32_t cap, unsigned fmt, uint32_t msg_id, unsigned set,
                                          uint32_t type, uint32_t id, const rpm_kvp *kv, unsigned n) {
  if (!n || n > RPM_MAX_KVP || set > RPM_SET_SLEEP) return 0;
  uint32_t data = 0;
  for (unsigned i = 0; i < n; ++i) {
    if (kv[i].nbytes != 4 && kv[i].nbytes != 8) return 0;
    data += 8 + kv[i].nbytes;
  }
  const uint32_t hdr = fmt == RPM_FMT_V1 ? 16u : 28u;
  if (hdr + data > cap) return 0;
  if (fmt == RPM_FMT_V1) {
    if (id > 0xfffu) return 0;
    rpm_put32(out, (data + 12u) | 0u << 16 | 1u << 24);  // len = data + msg header (16) - request header (4); REQUEST; v1
    rpm_put32(out + 4, msg_id);
    rpm_put32(out + 8, type);
    rpm_put32(out + 12, data | id << 16 | static_cast<uint32_t>(set) << 28);
  } else {
    rpm_put32(out, RPM_SERVICE_REQ);
    rpm_put32(out + 4, data + 20u);                       // data + msg header (28) - request header (8)
    rpm_put32(out + 8, msg_id);
    rpm_put32(out + 12, set);
    rpm_put32(out + 16, type);
    rpm_put32(out + 20, id);
    rpm_put32(out + 24, data);
  }
  uint32_t at = hdr;
  for (unsigned i = 0; i < n; ++i) {
    rpm_put32(out + at, kv[i].key);
    rpm_put32(out + at + 4, kv[i].nbytes);
    rpm_put32(out + at + 8, static_cast<uint32_t>(kv[i].value));
    if (kv[i].nbytes == 8) rpm_put32(out + at + 12, static_cast<uint32_t>(kv[i].value >> 32));
    at += 8 + kv[i].nbytes;
  }
  return at;
}

static inline bool rpm_prefix(const char *s, const char *p) {
  for (; *p; ++s, ++p)
    if (*s != *p) return false;
  return true;
}

// Ack decoding. v0: {service, length} then sub-messages {type, length, data}: "msg#" = id, "err\0" = error text
// (rpm-smd.c msm_rpm_get_error_from_ack treats any trailing data as an error; "resource does not exist" = no such
// resource). v1: {hdr (length, type ACK/NACK, version), id[, nack code]}.
static inline int rpm_decode_ack(const uint8_t *p, uint32_t n, unsigned fmt, uint32_t *id, uint32_t *code, char *text,
                                 uint32_t text_cap) {
  *id = 0; *code = 0;
  if (text_cap) text[0] = 0;
  if (fmt == RPM_FMT_V1) {
    if (n < 8) return RPM_ERR_BADACK;
    const uint32_t type = (rpm_get32(p) >> 16) & 0xffu;
    *id = rpm_get32(p + 4);
    if (type == RPM_V1_ACK) return RPM_OK;
    if (type == RPM_V1_NACK) { *code = n >= 12 ? rpm_get32(p + 8) : 0; return RPM_ERR_NACK; }
    return RPM_ERR_BADACK;
  }
  if (n < 8) return RPM_ERR_BADACK;
  if (rpm_get32(p) == RPM_V1_ACK && n >= 20) { *id = rpm_get32(p + 16); return RPM_OK; }   // v0 parser's v1 shortcut
  uint32_t end = 8 + rpm_get32(p + 4);
  if (end > n || end < 8) end = n;
  bool haveId = false, err = false;
  int rc = RPM_OK;
  for (uint32_t at = 8; at + 8 <= end;) {
    const uint32_t type = rpm_get32(p + at), len = rpm_get32(p + at + 4);
    if (len > end - at - 8) return haveId ? RPM_ERR_NACK : RPM_ERR_BADACK;
    const uint8_t *d = p + at + 8;
    if (type == RPM_ACK_MSG_ID && len >= 4) {
      *id = rpm_get32(d);
      haveId = true;
    } else {
      err = true;
      if (type == RPM_ACK_ERR && text_cap) {
        uint32_t k = 0;
        for (; k < len && k + 1 < text_cap && d[k]; ++k) text[k] = static_cast<char>(d[k]);
        text[k] = 0;
        rc = rpm_prefix(text, "resource does not exist") ? RPM_ERR_INV_RSC : RPM_ERR_NACK;
      } else if (rc == RPM_OK) {
        rc = RPM_ERR_NACK;
      }
    }
    at += 8 + ((len + 3u) & ~3u);
  }
  if (!haveId) return RPM_ERR_BADACK;
  return err ? rc : RPM_OK;
}

static inline void rpm_on_message(rpm_link *l) {
  uint32_t id = 0, code = 0;
  char text[32];
  const int rc = rpm_decode_ack(l->msg, l->msg_len, l->fmt, &id, &code, text, sizeof text);
  l->acks = l->acks + 1;
  if (l->wait_id && id == l->wait_id && rc != RPM_ERR_BADACK) {
    l->acked = true;
    l->ack_rc = rc;
    l->ack_code = code;
    for (unsigned k = 0; k < sizeof text; ++k) l->ack_text[k] = text[k];
    l->wait_id = 0;
  } else {
    l->stale_acks = l->stale_acks + 1;
  }
}

// --- receive ------------------------------------------------------------------------------------------------------
static inline void rpm_rx_peek(const rpm_link *l, uint32_t tail, uint32_t off, uint8_t *dst, uint32_t n) {
  const uint32_t len = l->rx.len, fifo = l->rx.desc + 8;
  uint32_t pos = tail + off;
  while (pos >= len) pos -= len;
  for (uint32_t i = 0; i < n; i += 4) {
    const uint32_t w = rpm_rd(l, fifo + pos);
    for (uint32_t k = 0; k < 4 && i + k < n; ++k) dst[i + k] = static_cast<uint8_t>(w >> (8 * k));
    pos += 4;
    if (pos >= len) pos = 0;
  }
}

// Handles every complete command in the RPM->apps FIFO (qcom_glink_native_intr + qcom_glink_work, polled). Returns
// RPM_OK (also when a command is still incomplete) or RPM_ERR_PROTO (unknown command or corrupt indices: the link is
// then marked broken, nothing more is read).
static inline int rpm_poll(rpm_link *l) {
  if (l->broken) return RPM_ERR_PROTO;
  const uint32_t len = l->rx.len;
  for (unsigned guard = 0; guard < 256; ++guard) {
    const uint32_t head = rpm_rd(l, l->rx.desc + 4), tail = rpm_rd(l, l->rx.desc);
    if (head >= len || tail >= len || head % 4 || tail % 4) { l->broken = true; return RPM_ERR_PROTO; }
    const uint32_t avail = rpm_ring_used(head, tail, len);
    if (avail < 8) return RPM_OK;
    uint8_t h[16];
    rpm_rx_peek(l, tail, 0, h, 8);
    const uint16_t cmd = static_cast<uint16_t>(rpm_get32(h) & 0xffffu);
    const uint16_t p1 = static_cast<uint16_t>(rpm_get32(h) >> 16);
    const uint32_t p2 = rpm_get32(h + 4);
    uint32_t adv = 8;
    enum { A_NONE, A_VERSION_ACK, A_VERSION, A_OPEN, A_CLOSE_ACK, A_KICK } act = A_NONE;
    switch (cmd) {
      case GL_VERSION:
        if (p1 == GL_VERSION_1) l->features &= p2;
        if (p1 != 0) act = A_VERSION_ACK;
        break;
      case GL_VERSION_ACK:
        if (p1 == GL_VERSION_1 && p2 == l->features) l->version_acked = true;
        else if (p1 != 0) {
          if (p1 == GL_VERSION_1) l->features &= p2;
          if (l->version_sends < 4) act = A_VERSION;
          else l->version_acked = true;            // stop renegotiating, features stay 0
        }
        break;
      case GL_OPEN: {
        const uint32_t nameLen = p2 & 0xffffu;
        if (nameLen > len) { l->broken = true; l->bad_cmd = cmd; return RPM_ERR_PROTO; }
        adv = 8 + rpm_align8(nameLen);
        if (avail < adv) return RPM_OK;
        uint8_t name[GL_NAME_SIZE] = {};
        rpm_rx_peek(l, tail, 8, name, nameLen < GL_NAME_SIZE ? nameLen : GL_NAME_SIZE);
        const char *want = RPM_CHANNEL;
        bool match = nameLen >= 13 && nameLen <= GL_NAME_SIZE;
        for (unsigned k = 0; match && k < 13; ++k) match = name[k] == static_cast<uint8_t>(want[k]);   // with NUL
        if (match) { l->rcid = p1; l->remote_open = true; act = A_OPEN; }
        else l->other_opens = l->other_opens + 1;  // no driver for it in Linux either: never acked
        break;
      }
      case GL_CLOSE:
        l->closes = l->closes + 1;
        if (l->remote_open && p1 == l->rcid) {
          l->remote_open = l->up = l->open_acked = l->open_sent = l->open_ack_sent = false;
          act = A_CLOSE_ACK;
        }
        break;
      case GL_OPEN_ACK:
        if (p1 == l->lcid && l->open_sent) l->open_acked = true;
        break;
      case GL_INTENT:
        if (p2 > len / 8) { l->broken = true; l->bad_cmd = cmd; return RPM_ERR_PROTO; }
        adv = rpm_align8(8 + 8 * p2);
        if (avail < adv) return RPM_OK;
        l->intents = l->intents + 1;
        break;
      case GL_RX_DONE: case GL_RX_DONE_W_REUSE: case GL_RX_INTENT_REQ: case GL_RX_INTENT_REQ_ACK: case GL_CLOSE_ACK:
        break;
      case GL_SIGNALS:
        l->signals = l->signals + 1;
        break;
      case GL_READ_NOTIF:
        l->read_notifs = l->read_notifs + 1;
        act = A_KICK;
        break;
      case GL_TX_DATA: case GL_TX_DATA_CONT: {
        if (avail < 16) return RPM_OK;
        rpm_rx_peek(l, tail, 0, h, 16);
        const uint32_t chunk = rpm_get32(h + 8), left = rpm_get32(h + 12);
        if (chunk > len - 16 || left > 0x10000u) { l->broken = true; l->bad_cmd = cmd; return RPM_ERR_PROTO; }
        adv = rpm_align8(16 + chunk);
        if (avail < 16 + chunk) return RPM_OK;
        if (!l->remote_open || p1 != l->rcid) { l->dropped = l->dropped + 1; break; }
        if (cmd == GL_TX_DATA) { l->msg_len = 0; l->msg_over = false; }
        if (l->msg_over || l->msg_len + chunk > RPM_MAX_RX) {
          l->msg_over = true;
        } else {
          uint8_t tmp[RPM_MAX_RX + 4];
          rpm_rx_peek(l, tail, 16, tmp, chunk);
          for (uint32_t k = 0; k < chunk; ++k) l->msg[l->msg_len + k] = tmp[k];
          l->msg_len += chunk;
        }
        if (!left) {
          if (l->msg_over) l->dropped = l->dropped + 1;
          else rpm_on_message(l);
          l->msg_len = 0;
          l->msg_over = false;
        }
        break;
      }
      default:
        l->broken = true;
        l->bad_cmd = cmd;
        return RPM_ERR_PROTO;
    }
    uint32_t nt = tail + adv;
    while (nt >= len) nt -= len;
    rpm_wr(l, l->rx.desc, nt);
    l->rx_cmds = l->rx_cmds + 1;
    int rc = RPM_OK;
    switch (act) {
      case A_VERSION_ACK: rc = rpm_send_cmd(l, GL_VERSION_ACK, GL_VERSION_1, l->features); break;
      case A_VERSION: rc = rpm_send_version(l); break;
      case A_OPEN:
        rc = rpm_send_cmd(l, GL_OPEN_ACK, l->rcid, 0);
        if (!rc) l->open_ack_sent = true;
        if (!rc && !l->open_sent) rc = rpm_send_open(l);
        break;
      case A_CLOSE_ACK: rc = rpm_send_cmd(l, GL_CLOSE_ACK, p1, 0); break;
      case A_KICK: rpm_kick(l); break;
      case A_NONE: break;
    }
    if (rc == RPM_ERR_PROTO) return rc;
  }
  return RPM_OK;
}

// --- session ------------------------------------------------------------------------------------------------------
enum : unsigned { RPM_RING_LINUX = 0, RPM_RING_RESUME = 1 };

// TOC, index reset (Linux: tx head = rx tail = 0; resume: tx head kept, rx tail = rx head, i.e. stale input dropped)
// and VERSION. Then polls until the channel is open both ways. local_after_us: when we send OPEN ourselves if the
// RPM has not opened the channel by then.
static inline int rpm_link_open(rpm_link *l, unsigned ring, uint32_t timeout_us, uint32_t local_after_us, uint32_t step_us) {
  int rc = rpm_parse_toc(l);
  if (rc) return rc;
  if (ring == RPM_RING_RESUME) {
    if (l->rx_head0 >= l->rx.len || l->rx_head0 % 8 || l->tx_head0 >= l->tx.len || l->tx_head0 % 8) return RPM_ERR_PROTO;
    rpm_wr(l, l->rx.desc, l->rx_head0);
  } else {
    rpm_wr(l, l->tx.desc + 4, 0);
    rpm_wr(l, l->rx.desc, 0);
  }
  l->lcid = RPM_LCID;
  l->rcid = 0;
  l->features = 0;
  l->version_sends = 0;
  l->version_acked = l->remote_open = l->open_sent = l->open_ack_sent = l->open_acked = l->up = l->broken = false;
  l->local_first = false;
  l->msg_len = 0;
  l->msg_over = false;
  l->wait_id = 0;
  rc = rpm_send_version(l);
  if (rc) return rc;
  if (!step_us) step_us = 1;
  for (uint32_t t = 0;; t += step_us) {
    rc = rpm_poll(l);
    if (rc) return rc;
    if (l->remote_open && l->open_ack_sent && l->open_sent && l->open_acked) { l->up = true; return RPM_OK; }
    if (!l->open_sent && l->version_acked && t >= local_after_us) {
      rc = rpm_send_open(l);
      if (rc == RPM_ERR_PROTO) return rc;
    }
    if (t >= timeout_us) return l->version_acked || l->remote_open ? RPM_ERR_OPEN : RPM_ERR_VERSION;
    l->io->delay_us(l->io->ctx, step_us);
  }
}

// TX_DATA with a full FIFO: like qcom_glink_tx, one READ_NOTIF asks the RPM for a doorbell once it has read, then wait.
static inline int rpm_tx_wait(rpm_link *l, const uint8_t *p, uint32_t n, uint32_t timeout_us, uint32_t step_us, uint32_t *t) {
  bool notified = false;
  for (;;) {
    const int rc = rpm_tx_try(l, p, n);
    if (rc != RPM_ERR_TXFULL) return rc;
    if (!notified) { notified = true; rpm_send_cmd(l, GL_READ_NOTIF, 0, 0); }
    if (*t >= timeout_us) return RPM_ERR_TXFULL;
    l->io->delay_us(l->io->ctx, step_us);
    *t += step_us;
    if (rpm_poll(l)) return RPM_ERR_PROTO;
  }
}

// One rpm-smd request on the open channel, waits for its ack. res may be null.
static inline int rpm_link_request(rpm_link *l, unsigned set, uint32_t type, uint32_t id, const rpm_kvp *kv, unsigned n,
                                   uint32_t timeout_us, uint32_t step_us, rpm_result *res) {
  if (res) *res = rpm_result{};
  if (!l->up || l->broken) return RPM_ERR_CLOSED;
  uint8_t pkt[16 + 28 + RPM_MAX_KVP * 16];
  const uint32_t msg_id = l->next_id;
  const uint32_t rlen = rpm_encode_request(pkt + 16, sizeof pkt - 16, l->fmt, msg_id, set, type, id, kv, n);
  if (!rlen) return RPM_ERR_PARAM;
  l->next_id = l->next_id + 1;
  if (l->next_id < 2) l->next_id = 2;
  rpm_put32(pkt, static_cast<uint32_t>(GL_TX_DATA) | static_cast<uint32_t>(l->lcid) << 16);
  rpm_put32(pkt + 4, 0);         // intentless: iid 0
  rpm_put32(pkt + 8, rlen);      // chunk_size
  rpm_put32(pkt + 12, 0);        // left_size
  if (!step_us) step_us = 1;
  l->acked = false;
  l->wait_id = msg_id;
  uint32_t t = 0;
  int rc = rpm_tx_wait(l, pkt, 16 + rlen, timeout_us, step_us, &t);
  if (rc) { l->wait_id = 0; return rc; }
  for (;; t += step_us) {
    rc = rpm_poll(l);
    if (rc) { l->wait_id = 0; return rc; }
    if (l->acked) {
      if (res) {
        res->msg_id = msg_id; res->code = l->ack_code; res->us = t;
        for (unsigned k = 0; k < sizeof res->text; ++k) res->text[k] = l->ack_text[k];
      }
      return l->ack_rc;
    }
    if (!l->up) { l->wait_id = 0; return RPM_ERR_CLOSED; }
    if (t >= timeout_us) {   // a late ack then counts as stale
      l->wait_id = 0;
      if (res) { res->msg_id = msg_id; res->us = t; }
      return RPM_ERR_ACK;
    }
    l->io->delay_us(l->io->ctx, step_us);
  }
}

// "ldoa", "clk2" ... for the log (non-printable bytes as '.').
static inline void rpm_fourcc(uint32_t v, char out[5]) {
  unsigned n = 0;
  for (unsigned k = 0; k < 4; ++k) {
    const char c = static_cast<char>(v >> (8 * k));
    if (!c) break;
    out[n] = c >= 0x20 && c < 0x7f ? c : '.';
    n = n + 1;
  }
  out[n] = 0;
}

#endif
