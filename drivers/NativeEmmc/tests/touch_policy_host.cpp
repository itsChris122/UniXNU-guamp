// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host tests (WSL) of the touch wake / release guard decisions (GuampTouchPolicyCore.h) and of the HxProtocol pieces
// the wake uses (hx_parse_gesture, hx_set_smwp, hx_overlay_swap, hx_sense_off, hx_parse_event) on a fake AHB bus.
// Usage: touch_policy_host [hlt_Himax_firmware.bin] (the overlay tests run on the real partition table when given).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>
#include "../HxProtocolCpp.inc"
#include "../GuampTouchPolicyCore.h"

static int fails = 0, checks = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++fails; std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

// Fake controller: byte memory behind the AHB bridge plus the few registers the sequences poll.
struct Fake {
  std::map<uint32_t, uint8_t> mem;
  uint32_t addr = 0;
  uint8_t event[HX_EVENT_LEN] = {};
  bool busError = false;
  unsigned resets = 0;
  uint32_t r32(uint32_t a) { uint32_t v = 0; for (int i = 0; i < 4; ++i) v |= (uint32_t)mem[a + i] << (8 * i); return v; }
  void w32(uint32_t a, uint32_t v) { for (int i = 0; i < 4; ++i) mem[a + i] = (uint8_t)(v >> (8 * i)); }
  void written(uint32_t a, uint32_t v) {
    if (a == HX_ADDR_RELOAD_TO_ACTIVE && (v & 0xFF) == 0xEC) w32(a, 0x01EC);
    if (a == HX_ADDR_CTRL_FW_ISR && (v & 0xFF) == 0xA5) w32(HX_ADDR_CENTRAL_STATE, 0x0C);
    if (a == HX_ADDR_RELOAD_STATUS || a == HX_ADDR_RELOAD_CRC32) w32(a, 0);
  }
};
static int xfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len) {
  Fake *f = static_cast<Fake *>(ctx);
  if (f->busError) return -1;
  if (tx[0] == HX_SPI_WRITE) {
    const uint8_t reg = tx[1];
    const uint8_t *d = tx + 2;
    const size_t n = len - 2;
    if (reg == HX_BUS_ADDR && n >= 4) {
      f->addr = rd_le32(d);
      for (size_t i = 4; i < n; ++i) f->mem[f->addr + (uint32_t)(i - 4)] = d[i];
      if (n == 8) f->written(f->addr, rd_le32(d + 4));
    }
    return 0;
  }
  if (tx[0] == HX_SPI_READ && rx) {
    const uint8_t reg = tx[1];
    const size_t n = len - 3;
    for (size_t i = 0; i < n; ++i) {
      uint8_t v = 0;
      if (reg == HX_BUS_RDATA) v = f->mem[f->addr + (uint32_t)i];
      else if (reg == HX_BUS_EVENT) v = i < HX_EVENT_LEN ? f->event[i] : 0;
      else if (reg == HX_BUS_CONTI) v = 0x31;
      else if (reg == HX_BUS_INCR4) v = 0x10;
      rx[3 + i] = v;
    }
  }
  return 0;
}
static void nosleep(void *, unsigned) {}
static void reset(void *ctx, int level) { if (level) static_cast<Fake *>(ctx)->resets++; }

struct Rig {
  Fake f;
  hx_bus bus{&f, xfer, nosleep, reset};
  std::vector<uint8_t> frame = std::vector<uint8_t>(HX_WRITE_CHUNK + 16), rxb = std::vector<uint8_t>(HX_WRITE_CHUNK + 16);
  hx_chip c{};
  Rig() { c.bus = &bus; c.frame = frame.data(); c.rx = rxb.data(); c.zf_dis_reload = 1; }
};

static void gestureStack(uint8_t *ev, uint8_t id) {
  std::memset(ev, 0, HX_EVENT_LEN);
  for (int i = 0; i < 4; ++i) ev[i] = id;
  ev[4] = 0xCC; ev[5] = 0x44;
}

static void testParseAndDecide() {
  using namespace guamp_touch_wake;
  uint8_t ev[HX_EVENT_LEN];
  gestureStack(ev, 0x81); CHECK(hx_parse_gesture(ev) == 0x81);
  gestureStack(ev, 0x80); CHECK(hx_parse_gesture(ev) == 0x80);
  gestureStack(ev, 0x81); ev[2] = 0x80; CHECK(hx_parse_gesture(ev) == 0);       // ids must be equal
  gestureStack(ev, 0x81); ev[5] = 0x45; CHECK(hx_parse_gesture(ev) == 0);       // CC 44 header
  gestureStack(ev, 0x99); CHECK(hx_parse_gesture(ev) == 0);                     // unknown id

  CHECK(decide(kFinger, 0x81, true, true).verdict == kWake);
  CHECK(decide(kFinger, 0x80, true, true).verdict == kIgnored);                 // double tap excluded by default
  CHECK(decide(kFinger | kDoubleTap, 0x80, true, true).verdict == kWake);
  CHECK(decide(kFinger, 0x05, true, true).verdict == kIgnored);
  CHECK(decide(kFinger | kOtherGestures, 0x05, true, true).verdict == kWake);
  CHECK(decide(kFinger | kOtherGestures, 0x80, true, true).verdict == kIgnored);
  CHECK(decide(kFinger, 0, true, true).verdict == kNone);                       // a plain report needs bit 8
  CHECK(decide(kFinger | kAnyReport, 0, true, true).verdict == kWake);
  CHECK(decide(kFinger | kAnyReport, 0, true, false).verdict == kNone);         // empty stack never wakes
  CHECK(decide(kAll, 0x81, false, true).verdict == kNone);                      // failed read never wakes

  CHECK(gestureActive(1, 11, 11, kGestureReply, kSmwpOn));
  CHECK(gestureActive(2, 11, 11, kGestureReply, kSmwpOn));
  CHECK(!gestureActive(0, 11, 11, kGestureReply, kSmwpOn));
  CHECK(!gestureActive(1, -1, 11, HX_OVL_FAULT, kSmwpOn));
  CHECK(!gestureActive(1, 11, 11, kGestureReply, 0));                           // SMWP not taken
  CHECK(resumeOk(1, 12, 12, kBorderReply, 0x05));
  CHECK(!resumeOk(1, 12, 12, kBorderReply, 0x0C));
  CHECK(!resumeOk(-5, 12, 12, kBorderReply, 0x05));
}

static void testGuard() {
  using namespace guamp_touch_guard;
  // Finger at rest: the stack still carries the contact -> keep.
  CHECK(verdict(0, 0, 0x01, 1, 3001, 30000) == kRearmContacts);
  CHECK(!releases(verdict(0, 0, 0x01, 1, 3001, 30000)));
  // Stack zero (HX_ERR_EMPTY) or checksum noise -> keep.
  CHECK(verdict(0, HX_ERR_EMPTY, 0, 0, 3001, 30000) == kRearmNoData);
  CHECK(verdict(0, HX_ERR_CHECKSUM, 0x31, 0, 3001, 30000) == kRearmNoData);
  // Explicit all up -> release.
  CHECK(verdict(0, 0, 0xFF, 0, 3001, 30000) == kReleaseUp);
  CHECK(verdict(0, 0, 0xF0, 0, 3001, 30000) == kReleaseUp);
  // Point count says a finger even though every slot was filtered -> keep.
  CHECK(verdict(0, 0, 0x01, 0, 3001, 30000) == kRearmContacts);
  // Bus error -> release; safety limit -> release whatever the stack says; 0 = no limit.
  CHECK(verdict(-5, HX_ERR_BUS, 0, 0, 3001, 30000) == kReleaseBus);
  CHECK(verdict(0, 0, 0x01, 1, 30000, 30000) == kReleaseLimit);
  CHECK(verdict(0, 0, 0x01, 1, 999999, 0) == kRearmContacts);
}

static void testEventParse() {
  uint8_t ev[HX_EVENT_LEN] = {};
  hx_report r{};
  // One finger at 100,200 then the checksum byte, as the HX83102D stack.
  ev[0] = 0; ev[1] = 100; ev[2] = 0; ev[3] = 200;
  for (unsigned i = 4; i < HX_MAX_PT * 4; ++i) ev[i] = 0xFF;
  ev[HX_EVENT_POINT_CNT] = 0x01;
  unsigned sum = 0;
  for (unsigned i = 0; i < HX_EVENT_LEN - 1; ++i) sum += ev[i];
  ev[HX_EVENT_LEN - 1] = (uint8_t)(0x100 - (sum & 0xFF));
  CHECK(hx_parse_event(ev, 720, 1600, &r) == HX_OK && r.count == 1);
  CHECK(guamp_touch_guard::verdict(0, HX_OK, r.point_cnt, r.count, 4000, 30000) == guamp_touch_guard::kRearmContacts);
  std::memset(ev, 0, sizeof ev);
  CHECK(hx_parse_event(ev, 720, 1600, &r) == HX_ERR_EMPTY);
}

static void testSwap(const std::vector<uint8_t> &img) {
  hx_fw fw{};
  if (img.empty() || hx_fw_parse(&fw, img.data(), img.size()) != HX_OK) {
    std::printf("overlay swap: no firmware image, skipped\n");
    return;
  }
  std::printf("firmware: %u partitions, gesture %d border %d\n", fw.count, fw.ovl[1], fw.ovl[2]);
  CHECK(fw.ovl[1] > 0 && fw.ovl[2] > 0);
  {  // display off with the firmware asking for the gesture overlay
    Rig g;
    g.f.w32(HX_ADDR_CENTRAL_STATE, 0x05);
    g.f.w32(HX_ADDR_OVL_HANDSHAKE, HX_OVL_GESTURE_REQUEST);
    const uint32_t smwp = hx_set_smwp(&g.c, 1);
    const int r = hx_overlay_swap(&g.c, &fw, fw.ovl[1], HX_OVL_GESTURE_REQUEST, HX_OVL_GESTURE_REPLY, 0, 500, 1);
    CHECK(smwp == guamp_touch_wake::kSmwpOn);
    CHECK(r == 1);
    CHECK(guamp_touch_wake::gestureActive(r, g.c.ovl_loaded, fw.ovl[1], g.c.ovl_reply, smwp));
    CHECK((g.f.r32(HX_ADDR_OVL_HANDSHAKE) & 0xFF) == HX_OVL_GESTURE_REPLY);
    const hx_zf_part &p = fw.part[fw.ovl[1]];
    bool same = true;
    for (uint32_t i = 0; i < p.size; ++i) same = same && g.f.mem[p.sram + i] == img[p.offset + i];
    CHECK(same);                                                  // overlay 11 written at its SRAM address
    CHECK(g.f.resets == 0);                                       // the firmware keeps running (no sense on)
  }
  {  // no request and no force: nothing swapped -> the caller stops the scanning
    Rig g;
    g.f.w32(HX_ADDR_CENTRAL_STATE, 0x05);
    const uint32_t smwp = hx_set_smwp(&g.c, 1);
    const int r = hx_overlay_swap(&g.c, &fw, fw.ovl[1], HX_OVL_GESTURE_REQUEST, HX_OVL_GESTURE_REPLY, 0, 50, 0);
    CHECK(r == 0);
    CHECK(!guamp_touch_wake::gestureActive(r, g.c.ovl_loaded, fw.ovl[1], g.c.ovl_reply, smwp));
    CHECK(hx_sense_off(&g.c) == HX_OK && g.c.central_state == 0x0C);
  }
  {  // bus dead: SMWP never reads back, no gesture mode
    Rig g;
    g.f.busError = true;
    const uint32_t smwp = hx_set_smwp(&g.c, 1);
    const int r = hx_overlay_swap(&g.c, &fw, fw.ovl[1], HX_OVL_GESTURE_REQUEST, HX_OVL_GESTURE_REPLY, 0, 50, 1);
    CHECK(r < 0);
    CHECK(!guamp_touch_wake::gestureActive(r, g.c.ovl_loaded, fw.ovl[1], g.c.ovl_reply, smwp));
  }
  {  // display on: border request, overlay 12, reply 66, ic_reset, firmware running
    Rig g;
    g.f.w32(HX_ADDR_CENTRAL_STATE, 0x05);
    g.f.w32(HX_ADDR_OVL_HANDSHAKE, HX_OVL_BORDER_REQUEST);
    const int r = hx_overlay_swap(&g.c, &fw, fw.ovl[2], HX_OVL_BORDER_REQUEST, HX_OVL_BORDER_REPLY, 1, 500, 1);
    CHECK(r == 1);
    CHECK(g.f.resets >= 1);
    CHECK(hx_set_smwp(&g.c, 0) == 0);
    CHECK(guamp_touch_wake::resumeOk(r, g.c.ovl_loaded, fw.ovl[2], g.c.ovl_reply, g.f.r32(HX_ADDR_CENTRAL_STATE)));
  }
}

int main(int argc, char **argv) {
  std::vector<uint8_t> img;
  if (argc > 1) {
    if (FILE *fp = std::fopen(argv[1], "rb")) {
      std::fseek(fp, 0, SEEK_END);
      img.resize((size_t)std::ftell(fp));
      std::fseek(fp, 0, SEEK_SET);
      if (std::fread(img.data(), 1, img.size(), fp) != img.size()) img.clear();
      std::fclose(fp);
    }
  }
  testParseAndDecide();
  testGuard();
  testEventParse();
  testSwap(img);
  std::printf("touch policy: %d/%d checks passed\n", checks - fails, checks);
  return fails ? 1 : 0;
}
