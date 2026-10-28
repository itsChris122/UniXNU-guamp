// Host test of the PCM side of guamp audio: GuampAudioRing.inc (the /dev/guamp-audio ring, its 48 kHz null-sink clock
// and the ioctl core of guamp_audio_dev.h), the q6asm/q6adm builders of GuampAsm.inc checked byte by byte against
// the Linux struct layouts (offsets written out here, from include/dsp/apr_audio-v2.h compiled with a 20-byte
// apr_hdr), apr::PcmPath against a simulated ADSP (map, ADM open, ASM open, route, format, run, 4 periods in flight
// refilled from the ring on WRITE_DONE, stop, close in reverse, refusal undo), and apr::AfeTone in hold mode (port up
// without DTMF until release()). No device access. Build: see run-pas-tests.sh.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "../GuampAsm.inc"
#include "../GuampAudioRing.inc"

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); exit(1); } } while (0)

static uint32_t u32(const std::vector<uint8_t> &p, uint32_t at) { uint32_t v; memcpy(&v, &p[at], 4); return v; }
static uint32_t u16(const std::vector<uint8_t> &p, uint32_t at) { uint16_t v; memcpy(&v, &p[at], 2); return v; }

// ---- ABI values the plug-in compiles against (Apple sys/ioccom.h encoding) -------------------------------------------
static void abi() {
  CHECK(sizeof(guamp_audio_info) == 64 && sizeof(guamp_audio_position) == 24 && sizeof(guamp_audio_volume) == 8);
  CHECK(GUAMP_AUDIO_IOC_INFO == 0x40404101u && GUAMP_AUDIO_IOC_START == 0x20004102u && GUAMP_AUDIO_IOC_STOP == 0x20004103u);
  CHECK(GUAMP_AUDIO_IOC_VOLUME == 0x80084104u && GUAMP_AUDIO_IOC_ROUTE == 0x80044105u);
  CHECK(GUAMP_AUDIO_IOC_POSITION == 0x40184106u && GUAMP_AUDIO_IOC_DRAIN == 0x20004107u);
  printf("abi: struct sizes 64/24/8, ioctl numbers INFO %08x START %08x STOP %08x VOLUME %08x ROUTE %08x POSITION %08x "
         "DRAIN %08x\n", (unsigned)GUAMP_AUDIO_IOC_INFO, (unsigned)GUAMP_AUDIO_IOC_START, (unsigned)GUAMP_AUDIO_IOC_STOP,
         (unsigned)GUAMP_AUDIO_IOC_VOLUME, (unsigned)GUAMP_AUDIO_IOC_ROUTE, (unsigned)GUAMP_AUDIO_IOC_POSITION,
         (unsigned)GUAMP_AUDIO_IOC_DRAIN);
}

// ---- ring + null-sink clock + ioctl core ---------------------------------------------------------------------------
static void ring() {
  static uint8_t mem[GUAMP_AUDIO_RING_BYTES];
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
  CHECK(s.ioctl(GUAMP_AUDIO_IOC_INFO, &info, 0, 1200) == 0 && info.abi == 1 && info.rate == 48000 && info.state == 0 &&
        info.ring_bytes == 65536 && info.frames_written == 1200 && info.sink == GUAMP_AUDIO_SINK_NULL && info.route == 1);
  // start at t = 1 s: 10 ms later 480 frames played
  const uint64_t t0 = 24000000;
  CHECK(s.ioctl(GUAMP_AUDIO_IOC_START, nullptr, t0, 0) == 0 && s.running && s.clockRunning);
  guamp_audio_position pos;
  CHECK(s.ioctl(GUAMP_AUDIO_IOC_POSITION, &pos, t0 + 240000, 0) == 0);
  CHECK(pos.frames_played == 480 && pos.host_time == t0 + 240000 && pos.queued_frames == 720 && pos.underruns == 0);
  // more data keeps the clock continuous
  CHECK(s.push(pcm.data(), 48000, t0 + 240000) == 48000);
  CHECK(s.ioctl(GUAMP_AUDIO_IOC_POSITION, &pos, t0 + 2400000 + 7, 0) == 0);   // 100 ms (+7 ticks: not a whole frame)
  CHECK(pos.frames_played == 4800 && pos.host_time == t0 + 2400000 && pos.underruns == 0);
  // the ring fills: push takes only the space
  CHECK(s.push(pcm.data(), static_cast<uint32_t>(pcm.size()), t0 + 2400000) == 65536 - (13200 - 4800) * 4);
  CHECK(s.space() == 0);
  // run dry: the clock stops, one underrun, then restarts at the next write
  s.consume(t0 + 24000000 * 2ull);
  CHECK(!s.clockRunning && s.underruns == 1 && s.used() == 0 && s.framesPlayed() * 4 == s.wr);
  const uint64_t dry = s.framesPlayed(), t1 = t0 + 24000000 * 3ull;
  CHECK(s.push(pcm.data(), 1920, t1) == 1920 && s.clockRunning && s.anchorTicks == t1);
  CHECK(s.ioctl(GUAMP_AUDIO_IOC_POSITION, &pos, t1 + 120000, 0) == 0 && pos.frames_played == dry + 240);
  // data integrity through the wrap (ADSP consumer)
  guampaudio::Stream d;
  d.buf = mem;
  d.bytes = sizeof(mem);
  d.sink = GUAMP_AUDIO_SINK_ADSP;
  d.running = true;
  std::vector<uint8_t> out(1920);
  for (int i = 0; i < 100; i++) {
    CHECK(d.push(&pcm[(i * 1920) % 96000], 1920, 0) == 1920);
    CHECK(d.pull(out.data(), 1920) == 1920 && !memcmp(out.data(), &pcm[(i * 1920) % 96000], 1920));
  }
  CHECK(d.pull(out.data(), 1920) == 0 && out[0] == 0 && out[1919] == 0 && d.underruns == 1);   // silence when short
  // volume clamp, routes, stop, unknown ioctl
  guamp_audio_volume v = {-20000, 1};
  CHECK(s.ioctl(GUAMP_AUDIO_IOC_VOLUME, &v, 0, 0) == 0 && s.volume == -9600 && s.muted == 1);
  v = {300, 0};
  CHECK(s.ioctl(GUAMP_AUDIO_IOC_VOLUME, &v, 0, 0) == 0 && s.volume == 0 && s.muted == 0);
  uint32_t r = 2;
  CHECK(s.ioctl(GUAMP_AUDIO_IOC_ROUTE, &r, 0, 0) == 19);
  r = 9;
  CHECK(s.ioctl(GUAMP_AUDIO_IOC_ROUTE, &r, 0, 0) == 22);
  r = 1;
  CHECK(s.ioctl(GUAMP_AUDIO_IOC_ROUTE, &r, 0, 0) == 0);
  CHECK(s.ioctl(GUAMP_AUDIO_IOC_STOP, nullptr, 0, 0) == 0 && !s.running && s.used() == 0);
  CHECK(s.ioctl(0x12345678u, nullptr, 0, 0) == 25);
  printf("ring: pre-fill, 48 kHz clock from START, exact positions, full ring, dry -> underrun + re-anchor, wrap, "
         "silence padding, volume/route/stop: ok\n");
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
    static uint8_t shmem[4 * 4096], ringMem[GUAMP_AUDIO_RING_BYTES];
    guampaudio::Stream ring;
    ring.buf = ringMem;
    ring.bytes = sizeof(ringMem);
    ring.sink = GUAMP_AUDIO_SINK_ADSP;
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
    path.fill = [](void *, uint8_t *dst, uint32_t n) { return gRing->pull(dst, n); };
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
  printf("pcm path: setup order, 4 periods in flight refilled from the ring on WRITE_DONE, silence on underrun, stop "
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

int main() {
  abi();
  ring();
  builders();
  pcmPath();
  holdMode();
  printf("audio_pcm_host: PASS\n");
  return 0;
}
