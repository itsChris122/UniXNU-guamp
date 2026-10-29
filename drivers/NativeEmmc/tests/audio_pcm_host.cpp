// Host test of the PCM side of guamp audio: GuampAudioRing.inc (the /dev/guamp-audio ring, its 48 kHz null-sink clock
// and the ioctl core of the ABI v2 header package/GuampAudio/guamp_audio_dev.h), the q6asm/q6adm builders of GuampAsm.inc checked byte by byte against
// the Linux struct layouts (offsets written out here, from include/dsp/apr_audio-v2.h compiled with a 20-byte
// apr_hdr), apr::PcmPath against a simulated ADSP (map, ADM open, ASM open, route, format, run, 4 periods in flight
// refilled from the ring on WRITE_DONE, stop, close in reverse, refusal undo), and apr::AfeTone in hold mode (port up
// without DTMF until release(), MI2S and codec DMA), apr::Playback (the whole chain: test sine and /dev/guamp-audio
// sink, route change, refusals) and the SMMU decision for stream 0x1c1. No device access. Build: see run-pas-tests.sh.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <vector>
#include <string>
#include "../GuampAsm.inc"
#include "../GuampAudioRing.inc"

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); exit(1); } } while (0)

static uint32_t u32(const std::vector<uint8_t> &p, uint32_t at) { uint32_t v; memcpy(&v, &p[at], 4); return v; }
static uint32_t u16(const std::vector<uint8_t> &p, uint32_t at) { uint16_t v; memcpy(&v, &p[at], 2); return v; }

// ---- ABI values the plug-in compiles against (Apple sys/ioccom.h encoding) -------------------------------------------
static void abi() {
  CHECK(sizeof(guamp_audio_info) == 64 && sizeof(guamp_audio_position) == 24 && sizeof(guamp_audio_volume) == 16);
  CHECK(GUAMP_AUDIO_ABI_VERSION == 2);
  CHECK(GUAMP_AUDIO_GET_INFO == 0x40406740u && GUAMP_AUDIO_START == 0x20006741u && GUAMP_AUDIO_STOP == 0x20006742u);
  CHECK(GUAMP_AUDIO_SET_VOLUME == 0x80106743u && GUAMP_AUDIO_SET_ROUTE == 0x80046744u && GUAMP_AUDIO_GET_ROUTE == 0x40046745u);
  CHECK(GUAMP_AUDIO_GET_POSITION == 0x40186746u && GUAMP_AUDIO_DRAIN == 0x20006747u);
  CHECK(offsetof(guamp_audio_info, flags) == 24 && offsetof(guamp_audio_info, underruns) == 48 &&
        offsetof(guamp_audio_info, route) == 56 && offsetof(guamp_audio_info, dsp_bits) == 60);
  printf("abi v2: GET_INFO %08x START %08x STOP %08x SET_VOLUME %08x SET_ROUTE %08x GET_ROUTE %08x GET_POSITION %08x "
         "DRAIN %08x\n", (unsigned)GUAMP_AUDIO_GET_INFO, (unsigned)GUAMP_AUDIO_START, (unsigned)GUAMP_AUDIO_STOP,
         (unsigned)GUAMP_AUDIO_SET_VOLUME, (unsigned)GUAMP_AUDIO_SET_ROUTE, (unsigned)GUAMP_AUDIO_GET_ROUTE,
         (unsigned)GUAMP_AUDIO_GET_POSITION, (unsigned)GUAMP_AUDIO_DRAIN);
}

// ---- ring + null-sink clock + ioctl core ---------------------------------------------------------------------------
static void ring() {
  static uint8_t mem[guampaudio::kRingBytes];
  guampaudio::Stream s;
  s.buf = mem;
  s.bytes = sizeof(mem);
  s.ticksPerSec = 24000000;   // 24 MHz host clock
  std::vector<uint8_t> pcm(48000 * 4);
  for (size_t i = 0; i < pcm.size(); i++) pcm[i] = static_cast<uint8_t>(i * 7);
  // pre-fill while stopped: nothing plays
  CHECK(s.push(pcm.data(), 4801, 0) == 4800 && s.used() == 4800);
  s.consume(24000000);
  CHECK(s.framesPlayed() == 0);
  guamp_audio_info info;
  CHECK(s.ioctl(GUAMP_AUDIO_GET_INFO, &info, 0, 1200) == 0 && info.abi == 2 && info.rate == 48000 &&
        info.flags == GUAMP_AUDIO_INFO_HW_VOLUME && info.ring_frames == 16384 && info.frames_written == 1200 &&
        info.route == 1 && info.routes == 2 && info.dsp_bits == 0 && info.period_frames == 480);
  // start at t = 1 s: 10 ms later 480 frames played
  const uint64_t t0 = 24000000;
  CHECK(s.ioctl(GUAMP_AUDIO_START, nullptr, t0, 0) == 0 && s.running && s.clockRunning);
  guamp_audio_position pos;
  CHECK(s.ioctl(GUAMP_AUDIO_GET_POSITION, &pos, t0 + 240000, 0) == 0);
  CHECK(pos.frames_played == 480 && pos.host_time == t0 + 240000 && pos.queued_frames == 720 && pos.underruns == 0);
  // more data keeps the clock continuous
  CHECK(s.push(pcm.data(), 48000, t0 + 240000) == 48000);
  CHECK(s.ioctl(GUAMP_AUDIO_GET_POSITION, &pos, t0 + 2400000 + 7, 0) == 0);   // 100 ms (+7 ticks: not a whole frame)
  CHECK(pos.frames_played == 4800 && pos.host_time == t0 + 2400000 && pos.underruns == 0);
  // the ring fills: push takes only the space
  CHECK(s.push(pcm.data(), static_cast<uint32_t>(pcm.size()), t0 + 2400000) == 65536 - (13200 - 4800) * 4);
  CHECK(s.space() == 0);
  // run dry: the clock stops, one underrun, then restarts at the next write
  s.consume(t0 + 24000000 * 2ull);
  CHECK(!s.clockRunning && s.underruns == 1 && s.used() == 0 && s.framesPlayed() * 4 == s.wr);
  const uint64_t dry = s.framesPlayed(), t1 = t0 + 24000000 * 3ull;
  CHECK(s.push(pcm.data(), 1920, t1) == 1920 && s.clockRunning && s.anchorTicks == t1);
  CHECK(s.ioctl(GUAMP_AUDIO_GET_POSITION, &pos, t1 + 120000, 0) == 0 && pos.frames_played == dry + 240);
  // data integrity through the wrap (ADSP consumer)
  guampaudio::Stream d;
  d.buf = mem;
  d.bytes = sizeof(mem);
  d.sink = guampaudio::SinkAdsp;
  d.running = true;
  std::vector<uint8_t> out(1920);
  for (int i = 0; i < 100; i++) {
    CHECK(d.push(&pcm[(i * 1920) % 96000], 1920, 0) == 1920);
    CHECK(d.pull(out.data(), 480, 16, 0) == 480 && !memcmp(out.data(), &pcm[(i * 1920) % 96000], 1920));
  }
  CHECK(d.pull(out.data(), 480, 16, 0) == 0 && out[0] == 0 && out[1919] == 0 && d.underruns == 1);   // silence when short
  // 24-bit container (top 24 bits of s32) and Q16 gain
  {
    const uint8_t two[8] = {0x34, 0x12, 0xcc, 0xed, 0, 0x80, 0xff, 0x7f};   // 0x1234, -0x1234, -32768, 32767
    std::vector<uint8_t> o(16);
    CHECK(d.push(two, 8, 0) == 8 && d.pull(o.data(), 2, 24, 0) == 2);
    CHECK(u32(o, 0) == 0x12340000u && u32(o, 4) == 0xedcc0000u && u32(o, 8) == 0x80000000u && u32(o, 12) == 0x7fff0000u);
    d.gainQ16 = 0x8000;
    CHECK(d.push(two, 8, 0) == 8 && d.pull(o.data(), 2, 16, 0) == 2);
    CHECK(u16(o, 0) == 0x091a && u16(o, 2) == 0xf6e6 && u16(o, 4) == 0xc000 && u16(o, 6) == 0x3fff);
  }
  // volume clamp, routes, stop, unknown ioctl
  guamp_audio_volume v = {0x20000, -100, 0, 0};
  CHECK(s.ioctl(GUAMP_AUDIO_SET_VOLUME, &v, 0, 0) == 0 && s.gainQ16 == 0x10000 && s.mute == 0);
  v = {0, -4800, 1, 0};
  CHECK(s.ioctl(GUAMP_AUDIO_SET_VOLUME, &v, 0, 0) == 0 && s.gainQ16 == 0 && s.mute == 1 && s.centiDb == -4800);
  uint32_t r = GUAMP_AUDIO_DEV_ROUTE_HEADPHONE;
  CHECK(s.ioctl(GUAMP_AUDIO_SET_ROUTE, &r, 0, 0) == 19);   // not wired without guamp-audio-cdc=1
  r = 9;
  CHECK(s.ioctl(GUAMP_AUDIO_SET_ROUTE, &r, 0, 0) == 22);
  s.routes |= 1u << GUAMP_AUDIO_DEV_ROUTE_HEADPHONE;
  r = GUAMP_AUDIO_DEV_ROUTE_HEADPHONE;
  CHECK(s.ioctl(GUAMP_AUDIO_SET_ROUTE, &r, 0, 0) == 0 && s.route == 3 && s.routeChanges == 1);
  r = 0;
  CHECK(s.ioctl(GUAMP_AUDIO_GET_ROUTE, &r, 0, 0) == 0 && r == 3);
  r = 1;
  CHECK(s.ioctl(GUAMP_AUDIO_SET_ROUTE, &r, 0, 0) == 0);
  s.adspEnabled = true;
  s.dspBits = 24;
  s.setSink(guampaudio::SinkAdsp, 0);
  CHECK(s.ioctl(GUAMP_AUDIO_GET_INFO, &info, 0, 0) == 0 &&
        info.flags == (GUAMP_AUDIO_INFO_RUNNING | GUAMP_AUDIO_INFO_HW_READY | GUAMP_AUDIO_INFO_HW_VOLUME |
                       GUAMP_AUDIO_INFO_SINK_ADSP) && info.dsp_bits == 24);
  s.setSink(guampaudio::SinkNull, 0);
  CHECK(s.ioctl(GUAMP_AUDIO_STOP, nullptr, 0, 0) == 0 && !s.running && s.used() == 0);
  CHECK(s.ioctl(0x12345678u, nullptr, 0, 0) == 25);
  printf("ring: pre-fill, 48 kHz clock from START, exact positions, full ring, dry -> underrun + re-anchor, wrap, "
         "silence padding, s24 container + gain, volume/route/info flags/stop: ok\n");
}

// ---- on-demand ADSP sink: hold while the path comes up, drain after STOP, idle gate, trim, live edge ----------------
static void ondemand() {
  static uint8_t mem[guampaudio::kRingBytes];
  const uint64_t T = 24000000, ms = T / 1000;   // 24 MHz host clock
  guampaudio::Stream s;
  s.buf = mem;
  s.bytes = sizeof(mem);
  s.ticksPerSec = T;
  s.adspEnabled = true;
  s.idleTicks = 4000 * ms;
  std::vector<uint8_t> quiet(4800 * 4, 0), click(14400 * 4);   // 100 ms of silence, 300 ms of sound
  for (size_t i = 0; i < click.size(); i += 2) { click[i] = 0x00; click[i + 1] = (i / 2) & 1 ? 0x10 : 0xf0; }
  std::vector<uint8_t> out(480 * 4);
  // START + silence: not wanted, the null sink consumes
  uint64_t now = T;
  CHECK(s.ioctl(GUAMP_AUDIO_START, nullptr, now, 0) == 0 && !s.wantAdsp(now));
  CHECK(s.push(quiet.data(), static_cast<uint32_t>(quiet.size()), now) == quiet.size() && !s.hold && !s.wantAdsp(now));
  now += 200 * ms;
  s.consume(now);
  CHECK(s.used() == 0);
  // a click: hold (kept in the ring), wanted
  CHECK(s.push(click.data(), static_cast<uint32_t>(click.size()), now) == click.size() && s.hold && s.holds == 1);
  CHECK(s.wantAdsp(now));
  now += 300 * ms;
  s.consume(now);
  CHECK(s.used() == click.size());
  // STOP before the path is up (iOS stops IO when the sound ends): draining, still wanted; close/open keep it
  CHECK(s.ioctl(GUAMP_AUDIO_STOP, nullptr, now, 0) == 0 && s.draining && s.drains == 1 && s.used() == click.size());
  guamp_audio_volume v = {0x4000, -1200, 0, 0};
  s.gainQ16 = v.gain_q16;
  s.reset();
  CHECK(s.used() == click.size() && s.draining && s.hold && s.gainQ16 == 0x4000 && s.wantAdsp(now));
  // path up 150 ms later: the whole click plays (no live-edge cut: the client stopped), then drained
  now += 150 * ms;
  s.setSink(guampaudio::SinkAdsp, now);
  CHECK(!s.hold && s.draining && s.used() == click.size());
  uint32_t got = 0;
  while (s.used()) got += s.pull(out.data(), 480, 16, now);
  CHECK(got == 14400 && !s.draining && !s.wantAdsp(now));
  s.setSink(guampaudio::SinkNull, now);
  // a hold the path never takes is dropped after 2 s and the ring flows again
  CHECK(s.ioctl(GUAMP_AUDIO_START, nullptr, now, 0) == 0);
  CHECK(s.push(click.data(), 4800 * 4, now) == 4800 * 4 && s.hold);
  s.consume(now + 1900 * ms);
  CHECK(s.hold && s.used() == 4800 * 4);
  s.consume(now + 2001 * ms);
  CHECK(!s.hold && s.holdDrops == 1 && s.used() == 0);
  // music started cold: path up while the client still writes -> live edge (2 x 40 ms kept)
  now += 3 * T;
  CHECK(s.push(click.data(), static_cast<uint32_t>(click.size()), now) == click.size() && s.hold);
  s.setSink(guampaudio::SinkAdsp, now);
  CHECK(s.used() == 2 * 1920 * 4 && s.trimmedFrames == 14400 - 3840);
  // idle gate: only silence for 4 s -> not wanted while still running
  CHECK(s.wantAdsp(now + 3999 * ms) && !s.wantAdsp(now + 4001 * ms) && s.running);
  // trim: head silence beyond 40 ms is skipped once the ring has more than that
  while (s.used()) s.pull(out.data(), 480, 16, now);
  const uint64_t before = s.trimmedFrames;
  CHECK(s.push(quiet.data(), static_cast<uint32_t>(quiet.size()), now) == quiet.size());
  CHECK(s.push(click.data(), 960 * 4, now) == 960 * 4);
  CHECK(s.pull(out.data(), 480, 16, now) == 480 && s.trimmedFrames - before == 4800 + 960 - 1920);
  // STOP with the path streaming silence only queued: still drains (at most the ring), then not wanted
  CHECK(s.ioctl(GUAMP_AUDIO_STOP, nullptr, now, 0) == 0 && s.draining);
  while (s.used()) s.pull(out.data(), 480, 16, now);
  CHECK(!s.wantAdsp(now));
  // null sink only (guamp-adsp-ipc != 6): no hold, STOP discards as before
  guampaudio::Stream n;
  n.buf = mem;
  n.bytes = sizeof(mem);
  n.ticksPerSec = T;
  CHECK(n.ioctl(GUAMP_AUDIO_START, nullptr, now, 0) == 0 && n.push(click.data(), 4800 * 4, now) == 4800 * 4 && !n.hold);
  CHECK(!n.wantAdsp(now) && n.ioctl(GUAMP_AUDIO_STOP, nullptr, now, 0) == 0 && n.used() == 0 && !n.draining);
  printf("on-demand sink: silence not wanted, hold while the path comes up, drain after STOP across close/open with its "
         "volume, hold dropped after 2 s, live edge when still writing, idle gate 4 s, head-silence trim: ok\n");
}

// ---- ASM/ADM builders ----------------------------------------------------------------------------------------------
static std::vector<uint8_t> pkt(uint32_t n, const uint8_t *b) { return std::vector<uint8_t>(b, b + n); }
static void hdr(const std::vector<uint8_t> &p, uint8_t svc, uint16_t src, uint16_t dst, uint32_t opcode) {
  CHECK(u16(p, 0) == 0x0250 && u16(p, 2) == p.size() && p[4] == svc && p[5] == 5 && u16(p, 6) == src && p[8] == svc &&
        p[9] == 4 && u16(p, 10) == dst && u32(p, 16) == opcode);
}
static void builders() {
  uint8_t b[512] = {};
  auto m = pkt(apr::asmMemMap(b, sizeof(b), 1, 0x10000000, 1, 0x4000), b);   // avs_cmd_shared_mem_map_regions + region
  CHECK(m.size() == 28 + 12);
  hdr(m, 7, 0, 0, 0x10d92);
  CHECK(u16(m, 20) == 3 && u16(m, 22) == 1 && u32(m, 24) == 0 && u32(m, 28) == 0x10000000 && u32(m, 32) == 1 &&
        u32(m, 36) == 0x4000);
  auto o = pkt(apr::admOpen(b, sizeof(b), 2, 0x1000, 0x10312, 2, 16, 48000), b);   // adm_cmd_device_open_v5
  CHECK(o.size() == 48);
  hdr(o, 8, 0x1000, 0x1000, 0x10326);
  CHECK(u16(o, 20) == 0 && u16(o, 22) == 1 && u16(o, 24) == 0x1000 && u16(o, 26) == 0xffff && u32(o, 28) == 0x10312);
  CHECK(u16(o, 32) == 2 && u16(o, 34) == 16 && u32(o, 36) == 48000 && o[40] == 1 && o[41] == 2 && o[42] == 0 && o[47] == 0);
  auto om = pkt(apr::admOpen(b, sizeof(b), 2, 0xb030, 0x10312, 1, 16, 48000), b);   // mono: FC
  CHECK(om.size() == 48 && u16(om, 24) == 0xb030 && u16(om, 32) == 1 && om[40] == 3 && om[41] == 0);
  auto w24 = pkt(apr::asmOpenWrite(b, sizeof(b), 3, apr::asmPort(1, 1), 24), b);
  CHECK(u16(w24, 26) == 24);
  apr::I2sConfig cdc;
  cdc.port = 0xb030;
  cdc.cdcDma = true;
  auto cd = pkt(apr::afeCdcDma(b, sizeof(b), 11, cdc), b);   // afe_port_cmd_set_param_v2 36 + param_hdr 12 + union 144
  CHECK(cd.size() == 20 + 16 + 12 + 144);
  hdr(cd, 4, 0, 0, 0x100ef);
  CHECK(u16(cd, 20) == 0xb030 && u16(cd, 22) == 12 + 144 && u32(cd, 36) == 0x1020c && u32(cd, 40) == 0x102b8 &&
        u16(cd, 44) == 144 && u32(cd, 48) == 1 && u32(cd, 52) == 48000 && u16(cd, 56) == 16 && u16(cd, 58) == 0 &&
        u16(cd, 60) == 2 && u16(cd, 62) == 3 && u32(cd, 64) == 0);
  cdc.stereo = 0;
  cdc.paramBytes = 16;
  auto cm = pkt(apr::afeCdcDma(b, sizeof(b), 12, cdc), b);
  CHECK(cm.size() == 20 + 16 + 12 + 16 && u16(cm, 60) == 1 && u16(cm, 62) == 1);
  auto w = pkt(apr::asmOpenWrite(b, sizeof(b), 3, apr::asmPort(1, 1), 16), b);   // asm_stream_cmd_open_write_v3
  CHECK(w.size() == 36);
  hdr(w, 7, 0x101, 0x101, 0x10db3);
  CHECK(u32(w, 20) == 0 && u16(w, 24) == 0 && u16(w, 26) == 16 && u32(w, 28) == 0x10c68 && u32(w, 32) == 0x10da5);
  auto r = pkt(apr::admRoute(b, sizeof(b), 4, 1, 5), b);   // matrix map v5 + node + 1 copp (padded)
  CHECK(r.size() == 28 + 4 + 4);
  hdr(r, 8, 0, 0, 0x10325);
  CHECK(u32(r, 20) == 0 && u32(r, 24) == 1 && u16(r, 28) == 1 && u16(r, 30) == 1 && u16(r, 32) == 5 && u16(r, 34) == 0);
  auto f = pkt(apr::asmMediaFmt(b, sizeof(b), 5, 0x101, 48000, 2, 16), b);   // asm_multi_channel_pcm_fmt_blk_v2
  CHECK(f.size() == 44);
  hdr(f, 7, 0x101, 0x101, 0x10d98);
  CHECK(u32(f, 20) == 20 && u16(f, 24) == 2 && u16(f, 26) == 16 && u32(f, 28) == 48000 && u16(f, 32) == 1 &&
        u16(f, 34) == 0 && f[36] == 1 && f[37] == 2 && f[38] == 0);
  auto run = pkt(apr::asmRun(b, sizeof(b), 6, 0x101), b);   // asm_session_cmd_run_v2
  CHECK(run.size() == 32);
  hdr(run, 7, 0x101, 0x101, 0x10daa);
  auto wr = pkt(apr::asmWrite(b, sizeof(b), 7, 0x101, 0x10001000, 1, 9, 1920, 3), b);   // asm_data_cmd_write_v2
  CHECK(wr.size() == 52);
  hdr(wr, 7, 0x101, 0x101, 0x10dab);
  CHECK(u32(wr, 20) == 0x10001000 && u32(wr, 24) == 1 && u32(wr, 28) == 9 && u32(wr, 32) == 1920 && u32(wr, 36) == 3 &&
        u32(wr, 40) == 0 && u32(wr, 44) == 0 && u32(wr, 48) == 0);
  auto c = pkt(apr::admClose(b, sizeof(b), 8, 0x1000, 5), b);   // adm_close: src AFE port, dest copp
  CHECK(c.size() == 20);
  hdr(c, 8, 0x1000, 5, 0x10327);
  auto ac = pkt(apr::asmClose(b, sizeof(b), 9, 0x101), b);
  hdr(ac, 7, 0x101, 0x101, 0x10bcd);
  auto um = pkt(apr::asmMemUnmap(b, sizeof(b), 10, 9), b);
  CHECK(um.size() == 24 && u32(um, 20) == 9);
  hdr(um, 7, 0, 0, 0x10d94);
  printf("asm/adm builders: map, ADM open/route/close, ASM open/format/run/write/close, unmap: ok\n");
}

// ---- PcmPath against a simulated ADSP ------------------------------------------------------------------------------
struct Dsp {
  std::vector<std::vector<uint8_t>> rx;   // packets from apps
  std::vector<uint32_t> ops;
  uint32_t refuse = 0, writes = 0;
  std::vector<std::vector<uint8_t>> pendingDone;   // WRITE_DONE events the test releases
};
static std::vector<uint8_t> answer(uint8_t svc, uint16_t port, uint32_t token, uint32_t opcode, std::vector<uint32_t> pl) {
  std::vector<uint8_t> p(20 + 4 * pl.size());
  const uint16_t f = 0x0150, n = static_cast<uint16_t>(p.size());
  memcpy(&p[0], &f, 2); memcpy(&p[2], &n, 2);
  p[4] = svc; p[5] = 4; memcpy(&p[6], &port, 2); p[8] = svc; p[9] = 5; memcpy(&p[10], &port, 2);
  memcpy(&p[12], &token, 4); memcpy(&p[16], &opcode, 4);
  if (!pl.empty()) memcpy(&p[20], pl.data(), 4 * pl.size());
  return p;
}
static void pcmPath() {
  for (int variant = 0; variant < 2; variant++) {
    Dsp dsp;
    dsp.refuse = variant ? 0x10db3 : 0;   // variant 1: ASM open refused -> ADM close + unmap
    static uint8_t shmem[8 * 4096], ringMem[guampaudio::kRingBytes];
    guampaudio::Stream ring;
    ring.buf = ringMem;
    ring.bytes = sizeof(ringMem);
    ring.sink = guampaudio::SinkAdsp;
    ring.running = true;
    std::vector<uint8_t> pcm(1920 * 3);
    for (size_t i = 0; i < pcm.size(); i++) pcm[i] = static_cast<uint8_t>(i + 1);
    CHECK(ring.push(pcm.data(), static_cast<uint32_t>(pcm.size()), 0) == pcm.size());
    apr::PcmPath path;
    path.ctx = &dsp;
    path.shm = {shmem, 0x10000000, 1, sizeof(shmem)};
    path.send = [](void *c, const uint8_t *p, uint32_t n) { static_cast<Dsp *>(c)->rx.emplace_back(p, p + n); return 0; };
    static guampaudio::Stream *gRing;
    gRing = &ring;
    path.fill = [](void *, uint8_t *dst, uint32_t frames, uint32_t bits) { return gRing->pull(dst, frames, bits, 0); };
    path.begin(0);
    size_t seen = 0;
    uint32_t now = 0;
    for (; now < 20000 && !path.finished(); now += 10) {
      path.stepAt(now);
      while (seen < dsp.rx.size()) {
        const auto q = dsp.rx[seen++];
        const uint32_t token = u32(q, 12), op = u32(q, 16);
        const uint16_t port = static_cast<uint16_t>(u16(q, 10));
        dsp.ops.push_back(op);
        if (op == 0x10dab) {   // WRITE_V2: data checked, done event held back
          dsp.writes++;
          const uint8_t *slot = shmem + (u32(q, 20) - 0x10000000);
          CHECK(u32(q, 24) == 1 && u32(q, 28) == 0x99 && u32(q, 32) == 1920 && u32(q, 36) == dsp.writes - 1);
          if (dsp.writes <= 3) CHECK(!memcmp(slot, pcm.data() + (dsp.writes - 1) * 1920, 1920));
          else CHECK(slot[0] == 0 && slot[1919] == 0);   // the ring is empty: silence
          dsp.pendingDone.push_back(answer(7, port, token, 0x10d99, {u32(q, 20), u32(q, 24), u32(q, 28), 0}));
          continue;
        }
        std::vector<uint8_t> a;
        if (op == 0x10d92) a = answer(7, 0, token, 0x10d93, {0x99});
        else if (op == 0x10326) a = answer(8, port, token, 0x10329, {0, 5});
        else a = answer(q[4], port, token, 0x110e8, {op, op == dsp.refuse ? 0x2u : 0u});
        path.receive(a.data(), static_cast<uint32_t>(a.size()), now);
      }
      if (path.streaming() && now % 10 == 0 && !dsp.pendingDone.empty()) {   // one period done per 10 ms
        const auto e = dsp.pendingDone.front();
        dsp.pendingDone.erase(dsp.pendingDone.begin());
        path.receive(e.data(), static_cast<uint32_t>(e.size()), now);
      }
      if (path.writeDones >= 8) path.stop();
      if (path.stopping && path.inFlight && !dsp.pendingDone.empty()) {
        const auto e = dsp.pendingDone.front();
        dsp.pendingDone.erase(dsp.pendingDone.begin());
        path.receive(e.data(), static_cast<uint32_t>(e.size()), now);
      }
    }
    CHECK(path.finished());
    if (!variant) {
      CHECK(!path.failed && path.writes >= 8 && path.writeDones == path.writes && path.writeErrors == 0);
      std::vector<uint32_t> setup(dsp.ops.begin(), dsp.ops.begin() + 6), tail(dsp.ops.end() - 3, dsp.ops.end());
      CHECK(setup == (std::vector<uint32_t>{0x10d92, 0x10326, 0x10db3, 0x10325, 0x10d98, 0x10daa}));
      CHECK(tail == (std::vector<uint32_t>{0x10bcd, 0x10327, 0x10d94}));
      CHECK(ring.underruns == path.writes - 3);   // every period after the 3 queued ones was silence
      const auto &close = dsp.rx[dsp.rx.size() - 2];
      CHECK(u16(close, 6) == 0x1000 && u16(close, 10) == 5);
      CHECK(u32(dsp.rx.back(), 20) == 0x99);
    } else {
      CHECK(path.failed && path.failedStep == apr::PcmPath::AsmOpen && path.failedStatus == 2 && path.writes == 0);
      CHECK(dsp.ops == (std::vector<uint32_t>{0x10d92, 0x10326, 0x10db3, 0x10327, 0x10d94}));
    }
  }
  printf("pcm path: setup order, 8 periods in flight refilled from the ring on WRITE_DONE, silence on underrun, stop "
         "after drain, close/unmap; refused ASM open undone: ok\n");
}

// ---- AfeTone hold mode: port up without DTMF until release() --------------------------------------------------------
static void holdMode() {
  std::vector<std::vector<uint8_t>> sent;
  apr::AfeTone t;
  t.ctx = &sent;
  t.send = [](void *c, const uint8_t *p, uint32_t n) {
    static_cast<std::vector<std::vector<uint8_t>> *>(c)->emplace_back(p, p + n);
    return 0;
  };
  t.hw = [](void *, int, bool) { return 0; };
  t.useDtmf = false;
  t.holdPort = true;
  t.begin(0);
  size_t seen = 0;
  for (uint32_t now = 0; now < 60000 && !t.finished(); now += 10) {
    t.stepAt(now);
    while (seen < sent.size()) {
      const auto &q = sent[seen++];
      const uint32_t op = u32(q, 16);
      const auto a = op == 0x100f4 ? answer(4, 0, u32(q, 12), 0x100f5, {3}) : answer(4, 0, u32(q, 12), 0x110e8, {op, 0});
      t.receive(a.data(), static_cast<uint32_t>(a.size()), now);
    }
    if (now == 30000) {
      CHECK(t.portRunning());
      t.release();
    }
  }
  std::vector<uint32_t> ops;
  for (const auto &q : sent) ops.push_back(u32(q, 16));
  CHECK(t.ok() && ops == (std::vector<uint32_t>{0x100f4, 0x100f3, 0x100ef, 0x100e5, 0x100e6, 0x100f3, 0x100f6}));
  printf("afe hold mode: port held without DTMF for 30 s until release, then undone: ok\n");
}

// ---- AfeTone hold mode on the codec DMA port (earpiece/headphone): macro vote + rx_core/rx_npl clocks -------------
static void holdModeCdc() {
  std::vector<std::vector<uint8_t>> sent;
  apr::AfeTone t;
  t.ctx = &sent;
  t.send = [](void *c, const uint8_t *p, uint32_t n) {
    static_cast<std::vector<std::vector<uint8_t>> *>(c)->emplace_back(p, p + n);
    return 0;
  };
  static size_t sentAtPrepare;
  static std::vector<std::vector<uint8_t>> *gSent;
  gSent = &sent;
  t.hw = [](void *, int step, bool) {
    if (step == apr::AfeTone::HwPrepare) sentAtPrepare = gSent->size();
    return 0;
  };
  t.useDtmf = false;
  t.holdPort = true;
  t.port.port = 0xb030;
  t.port.cdcDma = true;
  t.port.stereo = 0;
  t.begin(0);
  size_t seen = 0;
  for (uint32_t now = 0; now < 20000 && !t.finished(); now += 10) {
    t.stepAt(now);
    while (seen < sent.size()) {
      const auto &q = sent[seen++];
      const uint32_t op = u32(q, 16);
      const auto a = op == 0x100f4 ? answer(4, 0, u32(q, 12), 0x100f5, {u32(q, 20) + 10})
                                   : answer(4, 0, u32(q, 12), 0x110e8, {op, 0});
      t.receive(a.data(), static_cast<uint32_t>(a.size()), now);
    }
    if (now == 10000) {
      CHECK(t.portRunning());
      t.release();
    }
  }
  std::vector<uint32_t> ops, extra;
  for (const auto &q : sent) ops.push_back(u32(q, 16));
  CHECK(t.ok() && ops == (std::vector<uint32_t>{0x100f4, 0x100f4, 0x100f3, 0x100f3, 0x100ef, 0x100e5, 0x100e6, 0x100f3,
                                                0x100f3, 0x100f6, 0x100f6}));
  CHECK(u32(sent[0], 20) == 4 && u32(sent[1], 20) == 3);                               // DCODEC then MACRO
  CHECK(sentAtPrepare == 4);   // both votes and both RX macro clocks before the codec side touches the macro
  CHECK(u32(sent[2], 48 + 4) == 0x30e && u32(sent[2], 48 + 8) == 22579200);              // rx_core
  CHECK(u32(sent[3], 48 + 4) == 0x30f && u32(sent[3], 48 + 16) == 1);                    // rx_npl on
  CHECK(u32(sent[4], 40) == 0x102b8 && u16(sent[4], 60) == 1 && u16(sent[4], 20) == 0xb030);
  CHECK(u32(sent[7], 48 + 4) == 0x30f && u32(sent[7], 48 + 16) == 0 && u32(sent[8], 48 + 4) == 0x30e);
  CHECK(u32(sent[9], 20) == 3 && u32(sent[9], 24) == 13 && u32(sent[10], 20) == 4 && u32(sent[10], 24) == 14);
  printf("afe hold mode, codec DMA 0xB030: macro + dcodec votes, rx_core/rx_npl clocks, CODEC_DMA_CONFIG mono, undone in "
         "reverse: ok\n");
}

// ---- SMMU decision for stream 0x1c1 -----------------------------------------------------------------------------------
static void smmu() {
  uint32_t smr[4] = {0, 0, 0, 0}, s2cr[4] = {0, 0, 0, 0};
  CHECK(apr::smmuDecide(1, smr, s2cr, 4, 0x1c1).path == apr::SmmuDisabled);
  CHECK(apr::smmuDecide(0, smr, s2cr, 4, 0x1c1).path == apr::SmmuUnmatchedBypass);
  CHECK(apr::smmuDecide(1u << 10, smr, s2cr, 4, 0x1c1).path == apr::SmmuUnmatchedFault);
  smr[2] = (1u << 31) | (0xfu << 16) | 0x1c0;   // 0x1c0 mask 0xf covers 0x1c1
  s2cr[2] = (1u << 16);
  apr::SmmuView v = apr::smmuDecide(1u << 10, smr, s2cr, 4, 0x1c1);
  CHECK(v.path == apr::SmmuBypass && v.slot == 2 && v.usable());
  s2cr[2] = 5;
  v = apr::smmuDecide(0, smr, s2cr, 4, 0x1c1);
  CHECK(v.path == apr::SmmuTranslate && v.cb == 5 && !v.usable());
  s2cr[2] = 2u << 16;
  CHECK(apr::smmuDecide(0, smr, s2cr, 4, 0x1c1).path == apr::SmmuFault);
  smr[2] = (1u << 31) | 0x1a0;   // another stream only
  CHECK(apr::smmuDecide(0, smr, s2cr, 4, 0x1c1).path == apr::SmmuUnmatchedBypass);
  printf("smmu decision for 0x1c1: disabled, bypass, translate, fault, unmatched bypass/fault: ok\n");
}

// ---- Playback: the whole chain against a simulated ADSP (AFE + ADM + ASM) ---------------------------------------------
struct Rig {
  std::vector<std::vector<uint8_t>> rx;
  std::vector<uint32_t> ops;
  std::vector<std::vector<uint8_t>> pendingDone;
  std::vector<std::string> hw;
  uint32_t refuse = 0, attach = 0, detach = 0, sineChecks = 0;
  bool wanted = false;
  uint32_t route = 1;
  guampaudio::Stream *ring = nullptr;
  uint8_t *shm = nullptr;
  std::vector<uint8_t> pulled;   // what the ADSP buffer received, first periods
};
static void runPlayback(apr::Playback &pb, Rig &rig, uint32_t from, uint32_t until, uint32_t (*event)(apr::Playback &, Rig &, uint32_t)) {
  size_t seen = rig.rx.size();
  for (uint32_t now = from; now < until; now += 5) {
    if (event) event(pb, rig, now);
    pb.stepAt(now);
    while (seen < rig.rx.size()) {
      const auto q = rig.rx[seen++];
      const uint32_t token = u32(q, 12), op = u32(q, 16);
      const uint16_t port = static_cast<uint16_t>(u16(q, 10));
      rig.ops.push_back(op);
      if (op == 0x10dab) {
        rig.pendingDone.push_back(answer(7, port, token, 0x10d99, {u32(q, 20), u32(q, 24), u32(q, 28), 0}));
        if (pb.mode == apr::Playback::Test && rig.shm) {   // the sine: 1 kHz, -12 dBFS, both channels equal
          const uint8_t *slot = rig.shm + (u32(q, 20) - 0x20000000u);
          const uint32_t w = pb.bits == 24 ? 4 : 2;
          int32_t peak = 0;
          for (uint32_t f = 0; f < 480; f++) {
            int32_t l = 0, r = 0;
            memcpy(&l, slot + f * 2 * w, w);
            memcpy(&r, slot + f * 2 * w + w, w);
            if (w == 2) { l = static_cast<int16_t>(l); r = static_cast<int16_t>(r); } else { l >>= 16; r >>= 16; }
            CHECK(l == r);
            if (l > peak) peak = l;
          }
          CHECK(peak == 8192);
          rig.sineChecks++;
        }
        continue;
      }
      std::vector<uint8_t> a;
      if (op == 0x100f4) a = answer(4, 0, token, 0x100f5, {7});
      else if (op == 0x10d92) a = answer(7, 0, token, 0x10d93, {0x55});
      else if (op == 0x10326) a = answer(8, port, token, 0x10329, {0, 9});
      else a = answer(q[4], port, token, 0x110e8, {op, op == rig.refuse ? 0x2u : 0u});
      pb.receive(a.data(), static_cast<uint32_t>(a.size()), now);
    }
    if (pb.state == apr::Playback::Streaming && now % 10 == 0 && !rig.pendingDone.empty()) {
      const auto e = rig.pendingDone.front();
      rig.pendingDone.erase(rig.pendingDone.begin());
      pb.receive(e.data(), static_cast<uint32_t>(e.size()), now);
    }
    if (pb.state == apr::Playback::PathDown && !rig.pendingDone.empty()) {
      const auto e = rig.pendingDone.front();
      rig.pendingDone.erase(rig.pendingDone.begin());
      pb.receive(e.data(), static_cast<uint32_t>(e.size()), now);
    }
  }
}
static void playbackSetup(apr::Playback &pb, Rig &rig, uint8_t *shm) {
  rig.shm = shm;
  pb.shm = {shm, 0x20000000u, 1, 8 * 4096};
  pb.hooks.ctx = &rig;
  pb.hooks.send = [](void *c, const uint8_t *p, uint32_t n) { static_cast<Rig *>(c)->rx.emplace_back(p, p + n); return 0; };
  pb.hooks.hw = [](void *c, uint32_t route, uint16_t port, uint32_t ch, int step, bool on) {
    static_cast<Rig *>(c)->hw.push_back(std::to_string(route) + " " + std::to_string(port) + " " + std::to_string(ch) + " " +
                                        apr::AfeTone::stepName(step) + (on ? " on" : " off"));
    return 0;
  };
}
static void playback() {
  static uint8_t shm[8 * 4096];
  // test mode, speaker, 24-bit: full chain order, sine in every period, verdict
  {
    Rig rig;
    apr::Playback pb;
    playbackSetup(pb, rig, shm);
    pb.mode = apr::Playback::Test;
    pb.bits = 24;
    pb.testMs = 500;
    runPlayback(pb, rig, 0, 5000, nullptr);
    CHECK(pb.finished() && pb.ok() && pb.sessions == 1 && pb.failures == 0);
    CHECK(rig.sineChecks >= 50 && pb.pcm.writeDones == pb.pcm.writes && pb.pcm.copp == 9 && pb.pcm.handle == 0x55);
    std::vector<uint32_t> head(rig.ops.begin(), rig.ops.begin() + 10);
    CHECK(head == (std::vector<uint32_t>{0x100f4, 0x100f3, 0x100ef, 0x100e5, 0x10d92, 0x10326, 0x10db3, 0x10325, 0x10d98,
                                         0x10daa}));
    std::vector<uint32_t> tail6(rig.ops.end() - 6, rig.ops.end());
    CHECK(tail6 == (std::vector<uint32_t>{0x10bcd, 0x10327, 0x10d94, 0x100e6, 0x100f3, 0x100f6}));
    const auto &open = rig.rx[6];   // ASM open write: 24 bit
    CHECK(u32(open, 16) == 0x10db3 && u16(open, 26) == 24);
    CHECK(rig.hw == (std::vector<std::string>{"1 4096 2 codec prepare on", "1 4096 2 amp on on", "1 4096 2 amp off off",
                                              "1 4096 2 codec release off"}));
  }
  // test mode, ADM open refused: ASM never opened, unmap, port down, failed verdict
  {
    Rig rig;
    rig.refuse = 0x10db3;
    apr::Playback pb;
    playbackSetup(pb, rig, shm);
    pb.testMs = 300;
    runPlayback(pb, rig, 0, 5000, nullptr);
    CHECK(pb.finished() && !pb.ok() && pb.failures == 1 && pb.pcm.failed && pb.pcm.failedStep == apr::PcmPath::AsmOpen);
    std::vector<uint32_t> tail(rig.ops.end() - 5, rig.ops.end());
    CHECK(tail == (std::vector<uint32_t>{0x10327, 0x10d94, 0x100e6, 0x100f3, 0x100f6}));
  }
  // sink mode: idle until wanted, ring data reaches the buffer, attach/detach, linger, route change restarts
  {
    Rig rig;
    static uint8_t ringMem[guampaudio::kRingBytes];
    guampaudio::Stream ring;
    ring.buf = ringMem;
    ring.bytes = sizeof(ringMem);
    ring.running = true;
    std::vector<uint8_t> pcm(1920 * 4);
    for (size_t i = 0; i < pcm.size(); i++) pcm[i] = static_cast<uint8_t>(i * 3 + 1);
    CHECK(ring.push(pcm.data(), static_cast<uint32_t>(pcm.size()), 0) == pcm.size());
    rig.ring = &ring;
    apr::Playback pb;
    playbackSetup(pb, rig, shm);
    pb.mode = apr::Playback::Sink;
    pb.hooks.source = [](void *c, uint8_t *dst, uint32_t frames, uint32_t bits) {
      Rig *r = static_cast<Rig *>(c);
      const uint32_t n = r->ring->pull(dst, frames, bits, 0);
      if (r->pulled.size() < 4 * 1920) r->pulled.insert(r->pulled.end(), dst, dst + frames * 4);
      return n;
    };
    pb.hooks.wanted = [](void *c, uint32_t *route) { *route = static_cast<Rig *>(c)->route; return static_cast<Rig *>(c)->wanted; };
    pb.hooks.attach = [](void *c, bool on) { (on ? static_cast<Rig *>(c)->attach : static_cast<Rig *>(c)->detach)++; };
    runPlayback(pb, rig, 0, 1000, nullptr);
    CHECK(pb.state == apr::Playback::Idle && rig.rx.empty());
    rig.wanted = true;
    runPlayback(pb, rig, 1000, 3000, nullptr);
    CHECK(pb.state == apr::Playback::Streaming && rig.attach == 1 && ring.used() == 0 && ring.underruns > 0);
    CHECK(rig.pulled.size() == pcm.size() && rig.pulled == pcm);   // the ring's frames, in order, into the slots
    rig.route = GUAMP_AUDIO_DEV_ROUTE_HEADPHONE;   // route change: down, then up on 0xB030
    runPlayback(pb, rig, 3000, 6000, nullptr);
    CHECK(pb.state == apr::Playback::Streaming && pb.route == 3 && pb.sessions == 1 && rig.detach == 1 && rig.attach == 2);
    CHECK(pb.afe.port.port == 0xb030 && pb.afe.port.cdcDma && pb.pcm.afePort == 0xb030 && pb.lastOk);
    rig.wanted = false;   // plug-in stopped: 2 s linger, then down
    runPlayback(pb, rig, 6000, 7500, nullptr);
    CHECK(pb.state == apr::Playback::Streaming);
    runPlayback(pb, rig, 7500, 12000, nullptr);
    CHECK(pb.state == apr::Playback::Idle && pb.sessions == 2 && pb.failures == 0 && rig.detach == 2);
  }
  printf("playback: test sine 24-bit through AFE/ADM/ASM + verdict, ASM refusal undone, sink idle/attach/route change/"
         "linger: ok\n");
}

// ---- a command GLINK cannot send (no remote intent) fails after 1 s instead of blocking the chain forever ----------
static void sendBlocked() {
  std::vector<std::string> notes;
  static std::vector<std::string> *gNotes;
  gNotes = &notes;
  apr::AfeTone t;
  t.send = [](void *, const uint8_t *, uint32_t) { return -1; };
  t.hw = [](void *, int, bool) { return 0; };
  t.log = [](void *, const char *what, uint32_t, uint32_t) { gNotes->push_back(what); };
  t.useDtmf = false;
  t.holdPort = true;
  t.begin(0);
  uint32_t now = 0;
  for (; now < 10000 && !t.finished(); now += 5) t.stepAt(now);
  CHECK(t.finished() && t.setupFailed && t.failedStep == apr::AfeTone::ClockOn && t.failedStatus == 0xfffffffeu);
  CHECK(now >= 2000 && now < 2200);   // vote blocked 1 s (not fatal), clock blocked 1 s (fatal), nothing to undo
  size_t blocked = 0;
  for (const auto &n : notes) blocked += n == "SEND BLOCKED for 1 s at step";
  CHECK(blocked == 2);
  apr::PcmPath p;
  static uint8_t shm[8 * 4096];
  p.shm = {shm, 0x10000000, 1, sizeof(shm)};
  p.send = [](void *, const uint8_t *, uint32_t) { return -1; };
  p.begin(0);
  for (now = 0; now < 5000 && !p.finished(); now += 5) p.stepAt(now);
  CHECK(p.finished() && p.failed && p.failedStep == apr::PcmPath::MemMap && p.failedStatus == 0xfffffffeu && now < 1100);
  printf("send blocked: AFE and PCM steps fail after 1 s with a log line, nothing left hanging: ok\n");
}

int main() {
  sendBlocked();
  abi();
  ring();
  ondemand();
  builders();
  pcmPath();
  holdMode();
  holdModeCdc();
  smmu();
  playback();
  printf("audio_pcm_host: PASS\n");
  return 0;
}
