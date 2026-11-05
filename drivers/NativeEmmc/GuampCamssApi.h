// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp CAMSS RAW capture: the interface other kernel code uses (C ABI, one translation unit with NativeEmmc).
//   - consumers (software ISP, the iOS camera service): open/start/wait_frame/release/stop/close and the frame
//     metadata below; the frame bytes are the MIPI RAW10 stream of the RDI path, one buffer per frame;
//   - the sensor driver (cam_sensor side): the three guamp_camss_sensor_* hooks. GuampCamss.inc carries weak
//     stand-ins that answer GUAMP_CAMSS_SENSOR_ABSENT, so only the TPG (test pattern generator) source works until a
//     sensor driver defines them (strong definitions win at link time; same model as GuampAudioHwWeak.inc).
// Layout rules: fixed-size fields only, abi first; a new field goes at the end and bumps GUAMP_CAMSS_ABI.
#ifndef GUAMP_CAMSS_API_H
#define GUAMP_CAMSS_API_H
#include <stdint.h>

#define GUAMP_CAMSS_ABI 3
#define GUAMP_CAMSS_MAX_BUFFERS 4

// errors (negative errno style; also returned by the sensor hooks)
#define GUAMP_CAMSS_OK 0
#define GUAMP_CAMSS_EBUSY (-16)
#define GUAMP_CAMSS_EINVAL (-22)
#define GUAMP_CAMSS_ENOMEM (-12)
#define GUAMP_CAMSS_ETIMEDOUT (-110)
#define GUAMP_CAMSS_ENOTREADY (-1002)        // nothing open / not streaming
#define GUAMP_CAMSS_ESMMU (-1003)            // stream 0x400 context bank or mapping failed
#define GUAMP_CAMSS_EHW (-1004)              // a block did not come out of reset / clock did not turn on
#define GUAMP_CAMSS_SENSOR_ABSENT (-1000)    // no sensor driver in this kernel (weak stand-in)

enum guamp_camss_source { GUAMP_CAMSS_SRC_TPG = 0, GUAMP_CAMSS_SRC_SENSOR = 1 };
enum guamp_camss_format { GUAMP_CAMSS_FMT_RAW10_MIPI = 1 };   // 4 pixels in 5 bytes, MIPI CSI-2 order
// Bayer order of the first two lines (TFE pixel pattern encoding): RGGB, GRBG, BGGR, GBRG; 4 = Y only (mono)
enum guamp_camss_bayer { GUAMP_CAMSS_RGGB = 0, GUAMP_CAMSS_GRBG = 1, GUAMP_CAMSS_BGGR = 2, GUAMP_CAMSS_GBRG = 3,
                         GUAMP_CAMSS_MONO = 4 };

// What the sensor sends on the CSI-2 link. The sensor driver fills it in guamp_camss_sensor_prepare.
struct guamp_camss_sensor_link {
  uint32_t abi;               // GUAMP_CAMSS_ABI
  uint32_t phy;               // CSIPHY 0..2 (DT csiphy-sd-index)
  uint32_t lanes;             // data lanes 1..4
  uint32_t lane_assign;       // CSID lane_cfg: nibble i = physical lane of logical lane i (GM1ST 0x3210)
  uint32_t lane_mask;         // CSIPHY lanes, 0 = from lanes/lane_assign (bit 0 DL0, 1 CLK, 2 DL1, 3 DL2, 4 DL3)
  uint32_t settle_count;      // CSIPHY THS-settle in 200 MHz timer cycles, 0 = computed from data_rate_mbps
  uint32_t data_rate_mbps;    // per lane
  uint32_t vc, dt;            // virtual channel, data type (RAW10 = 0x2b)
  uint32_t width, height;     // active pixels of the stream that goes to RDI
  uint32_t bayer;             // enum guamp_camss_bayer
  uint32_t combo;             // 1 = CSIPHY combo mode (two sensors on one PHY, OV02B pair); 0 normal
};

struct guamp_camss_config {
  uint32_t abi;               // GUAMP_CAMSS_ABI
  uint32_t source;            // enum guamp_camss_source
  uint32_t cell;              // sensor cell (DT cell-index, 0 = S5KGM1ST) for GUAMP_CAMSS_SRC_SENSOR
  uint32_t mode;              // sensor resolution index (sensor driver's numbering)
  uint32_t buffers;           // 2..GUAMP_CAMSS_MAX_BUFFERS
  uint32_t csid;              // CSID/TFE instance 0..2 (CSID n feeds TFE n)
  uint32_t rdi;               // RDI path 0..2
  uint32_t clock_level;       // 0 = GPLL0 only (cphy_rx/csid 240 MHz), 1 = DT svs_l1 (GPLL6, 384 MHz)
  uint32_t tpg_pattern;       // TPG source: 0..7 techpack patterns, 8 = unicolour, 9 = split colour bars
  uint32_t width, height;     // TPG source only (0 = 1600 x 1200)
  uint32_t tpg_lanes;         // TPG source only (0 = 4)
};

// One finished frame. Valid while the consumer holds the buffer (wait_frame holds, release gives it back).
struct guamp_camss_frame {
  uint32_t abi;
  uint32_t seq;               // 1, 2, ... frames completed since start (buffer done interrupts)
  uint32_t buffer;            // 0..buffers-1
  uint32_t format;            // enum guamp_camss_format
  uint32_t width, height;
  uint32_t line_bytes;        // width * 10 / 8 (frame-based write master: lines back to back, no padding)
  uint32_t bytes;             // line_bytes * height
  uint32_t bayer;
  uint32_t errors;            // CSID/TFE error bits seen during this frame (GCAMSS_FE_* in GuampCamssCore.h)
  uint64_t sof_ns, eof_ns, done_ns;   // mach_absolute_time in ns; 0 when the interrupt was not seen
  uint64_t iova;              // device address (SMMU stream 0x400)
  void *cpu;                  // kernel virtual address, Normal non-cacheable
};

struct guamp_camss_counters {
  uint32_t abi;
  uint32_t tfe_irqs, csid_irqs, polls;           // handler invocations (interrupt) and polling passes with work
  uint32_t sof, eof, rup, done, drops;           // TFE RDI start/end of frame, reg update done, buffer done, overwrites
  uint32_t csid_sof, csid_eof;
  uint32_t csid_rx_errors, csid_rdi_errors, tfe_errors, bus_errors;
  uint32_t first_rx_error, first_rdi_error, first_tfe_error[3], first_bus_error[2];
  uint32_t bus_violation, bus_overflow, bus_image_size;
  uint32_t csid_rx_unmapped, last_unmapped_hdr;  // ABI 2: packets of an unmapped VC/DT (PDAF), not an error
  // ABI 3: buffer claims and stream recovery
  uint32_t claim_redirects;   // wait_frame took a buffer already programmed as the next address: address moved
  uint32_t claim_stale;       // wait_frame found the newest frame being overwritten (its buffer is latched): skipped
  uint32_t held_overwrites;   // a consumer-held buffer was latched anyway (claim lost the race with the SOF)
  uint32_t recoveries;        // CSID/TFE path restarted after a stall (continuous streaming, guamp-cam2-recover)
  uint32_t recovery_failed;   // restarts whose CSID/TFE reset did not answer (the stream stays down)
};

#ifdef __cplusplus
extern "C" {
#endif
// Consumers
int guamp_camss_open(const struct guamp_camss_config *config);   // power, clocks, SMMU, buffers; not streaming
int guamp_camss_start(uint32_t frames);    // 0 = until stop; otherwise the source stops by itself after frames
// Newest finished frame with seq > after_seq; the buffer is held (never written) until guamp_camss_release. A frame
// whose buffer the write master is already writing again is skipped (claim_stale) and the wait goes on.
int guamp_camss_wait_frame(uint32_t after_seq, uint32_t timeout_ms, struct guamp_camss_frame *out);
void guamp_camss_release(uint32_t buffer);
int guamp_camss_stop(void);
void guamp_camss_close(void);
int guamp_camss_counters(struct guamp_camss_counters *out);
void *guamp_camss_buffer_memory(uint32_t buffer);   // IOBufferMemoryDescriptor * of that buffer (for user mapping)
// Sensor driver hooks (weak stand-ins in GuampCamss.inc). prepare: power on, initialise, fill *link, NOT streaming;
// stream: start/stop the MIPI output (called after/before the CSIPHY/CSID/TFE are ready); release: power off.
int guamp_camss_sensor_prepare(uint32_t cell, uint32_t mode, struct guamp_camss_sensor_link *link);
int guamp_camss_sensor_stream(uint32_t cell, int on);
void guamp_camss_sensor_release(uint32_t cell);
#ifdef __cplusplus
}
#endif
#endif
