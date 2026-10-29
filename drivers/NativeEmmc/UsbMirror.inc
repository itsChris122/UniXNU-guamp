// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Live screen and kernel log over USB for guamp (included by NativeEmmc.cpp; strumenti/specchio-ios.py is the PC side).
//
// The SM6115 USB 3.0 controller (Synopsys DWC3 at 0x04E00000, Qualcomm QSCRATCH wrapper at 0x04EF8800, QUSB2 PHY at
// 0x01613000) runs as a high-speed device with one vendor interface: bulk IN 0x81 carries the frame buffer rows that
// changed and the new kernel log text, bulk OUT 0x01 is reserved for input. Microsoft OS 2.0 descriptors make Windows
// bind WinUSB by itself (DeviceInterfaceGUID below). Polled from a kernel thread: no interrupts, DMA through physical
// addresses below 4 GiB (the SMMU stream is in bypass, as for the SDHC ADMA).
//
// Boot argument guamp-usb-mirror (default 1 since 28/10; =0 off). The controller is touched only when its GDSC and clocks are on (an unclocked access
// can hang the bus); the GCC state is logged either way. Counters: native_usb_stage (last step reached), _events, _frames.
//
// Message format (see specchio-ios.py): "GMIR", u32 type, u32 body length, u32 argument, then the body.
//   1 INFO (width, height, row bytes, format 1 = XRGB8888)  2 ROWS (argument = first row)  3 LOG (UTF-8)  4 END (frame)

extern "C" {
volatile uint32_t native_usb_stage = 0, native_usb_speed = 0, native_usb_configured = 0;
// Touches from the viewer (bulk OUT 0x01): "GTCH", count, then count x {x u16, y u16, id u8} in frame pixels. They go
// to iOS like the Himax ones (TouchState.cpp of the display archive: native_touch_* and native_touch_published).
extern "C" {
extern volatile uint32_t native_touch_seq, native_touch_count;
extern volatile int native_touch_xs[12], native_touch_ys[12], native_touch_ids[12];
void native_touch_published(void);
}
uint64_t native_usb_touches = 0;
uint64_t native_usb_events = 0, native_usb_frames = 0, native_usb_bytes = 0, native_usb_errors = 0, native_usb_ended = 0;
}

namespace usbm {

constexpr uint64_t kDwcPhys = 0x04E00000, kQscratchPhys = 0x04EF8800, kGccUsbPhys = 0x0141A000;
constexpr uint16_t kVid = 0x1209, kPid = 0x6D47;

// DWC3 global and device registers (offsets from the core base)
enum : uint32_t {
  GSBUSCFG0 = 0xC100, GCTL = 0xC110, GSNPSID = 0xC120, GUSB2PHYCFG = 0xC200, GUSB3PIPECTL = 0xC2C0,
  GEVNTADRLO = 0xC400, GEVNTADRHI = 0xC404, GEVNTSIZ = 0xC408, GEVNTCOUNT = 0xC40C,
  DCFG = 0xC700, DCTL = 0xC704, DEVTEN = 0xC708, DSTS = 0xC70C, DALEPENA = 0xC720,
  DEPCMDPAR2 = 0xC800, DEPCMDPAR1 = 0xC804, DEPCMDPAR0 = 0xC808, DEPCMD = 0xC80C,   // + 0x10 * physical endpoint
};
enum : uint32_t {   // DEPCMD command types
  CMD_SETEPCONFIG = 1, CMD_SETTRANSFRESOURCE = 2, CMD_STARTTRANSFER = 6, CMD_ENDTRANSFER = 8, CMD_DEPSTARTCFG = 9,
  CMD_CLEARSTALL = 5, CMD_SETSTALL = 4,
};
// Physical endpoints: 0 = EP0 OUT, 1 = EP0 IN, 2 = EP1 OUT, 3 = EP1 IN
constexpr unsigned kEp0Out = 0, kEp0In = 1, kEp1Out = 2, kEp1In = 3;

// Descriptors
const uint8_t kDevice[18] = {18, 1, 0x10, 0x02, 0xFF, 0, 0, 64, kVid & 0xFF, kVid >> 8, kPid & 0xFF, kPid >> 8,
                             0x00, 0x01, 1, 2, 0, 1};   // bcdUSB 2.10 (BOS), vendor class, EP0 64, 1 configuration
const uint8_t kConfig[32] = {
  9, 2, 32, 0, 1, 1, 0, 0x80, 50,                     // configuration: 1 interface, bus powered, 100 mA
  9, 4, 0, 0, 2, 0xFF, 0, 0, 0,                       // interface 0: 2 endpoints, vendor
  7, 5, 0x81, 2, 0x00, 0x02, 0,                       // bulk IN 0x81, 512
  7, 5, 0x01, 2, 0x00, 0x02, 0,                       // bulk OUT 0x01, 512
};
constexpr uint8_t kMsVendorCode = 0x20;
// BOS with the Microsoft OS 2.0 platform capability (descriptor set length 162, vendor code 0x20)
const uint8_t kBos[33] = {
  5, 15, 33, 0, 1,
  28, 16, 5, 0, 0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C, 0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F,
  0x00, 0x00, 0x03, 0x06, 162, 0, kMsVendorCode, 0,
};
// Microsoft OS 2.0 descriptor set: compatible ID WINUSB and DeviceInterfaceGUIDs (the GUID of specchio-ios.py)
uint8_t gMsOs20[162];
void buildMsOs20()
{
  uint8_t *p = gMsOs20;
  auto put16 = [&](uint16_t v) { *p++ = v & 0xFF; *p++ = v >> 8; };
  auto put32 = [&](uint32_t v) { put16(v & 0xFFFF); put16(v >> 16); };
  put16(10); put16(0); put32(0x06030000); put16(162);                       // set header, Windows 8.1+
  static const char compatible[8] = {'W', 'I', 'N', 'U', 'S', 'B', 0, 0};
  put16(20); put16(3); for (char c : compatible) *p++ = static_cast<uint8_t>(c);
  for (int i = 0; i < 8; i++) *p++ = 0;                                      // compatible ID
  const char name[] = "DeviceInterfaceGUIDs";                               // REG_MULTI_SZ property
  const char guid[] = "{4D6F746F-4739-4D69-7272-6F7230303031}";
  put16(132); put16(4); put16(7); put16(42);
  for (char c : name) put16(static_cast<uint8_t>(c));                       // 21 UTF-16 chars incl. NUL = 42 bytes
  put16(80);
  for (char c : guid) put16(static_cast<uint8_t>(c));                       // 39 chars incl. NUL = 78 bytes
  put16(0);                                                                 // second NUL of the multi-string
}
// String descriptors: 0 = languages, 1 = manufacturer, 2 = product
const char *const kStrings[] = {nullptr, "UniXNU guamp", "Moto G9 Play screen mirror"};

struct Trb { uint32_t bpl, bph, size, ctrl; };
enum : uint32_t {   // TRB control
  TRB_HWO = 1u << 0, TRB_LST = 1u << 1, TRB_CHN = 1u << 2, TRB_CSP = 1u << 3, TRB_ISP_IMI = 1u << 10, TRB_IOC = 1u << 11,
  TRBCTL_NORMAL = 1u << 4, TRBCTL_CONTROL_SETUP = 2u << 4, TRBCTL_CONTROL_STATUS2 = 3u << 4,
  TRBCTL_CONTROL_STATUS3 = 4u << 4, TRBCTL_CONTROL_DATA = 5u << 4,
};

struct Mirror {
  volatile uint8_t *dwc = nullptr, *qscratch = nullptr;
  IOBufferMemoryDescriptor *dma = nullptr;            // uncached, contiguous, below 4 GiB
  volatile uint8_t *mem = nullptr;
  uint32_t memPhys = 0;
  // Layout of the DMA block: events (4 KiB), EP0 TRB + buffer, EP1 IN TRB, bulk IN staging (64 KiB)
  static constexpr uint32_t kEvents = 0, kEventBytes = 4096, kEp0Trb = 4096, kEp0Buf = 4096 + 64, kEp1Trb = 8192,
                            kEp1OutTrb = 8192 + 64, kOutBuf = 8192 + 512, kOutBytes = 512,
                            kStage = 12288, kStageBytes = 65536, kDmaBytes = kStage + kStageBytes;
  uint32_t eventPos = 0;
  uint8_t setup[8] = {};
  bool ep0DataIn = false, inBusy = false, running = false;
  unsigned stuck = 0;
  uint64_t lastStatus = 0;
  uint32_t ep0Resource[2] = {}, ep1InResource = 0, ep1OutResource = 0;
  uint16_t address = 0;
  uint8_t pendingAddress = 0xFF;
  uint32_t logPos = 0;                                 // next message buffer byte to send
  uint32_t frame = 0;
  uint8_t *shadow = nullptr;                          // last frame sent

  uint32_t r(uint32_t o) const { return *reinterpret_cast<volatile uint32_t *>(dwc + o); }
  void w(uint32_t o, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(dwc + o) = v; }
  Trb *trb(uint32_t at) { return reinterpret_cast<Trb *>(const_cast<uint8_t *>(mem + at)); }

  bool command(unsigned ep, uint32_t cmd, uint32_t p0 = 0, uint32_t p1 = 0, uint32_t p2 = 0, uint32_t param = 0) {
    uint32_t base = DEPCMDPAR2 + 0x10 * ep;
    w(base + 0, p2); w(base + 4, p1); w(base + 8, p0);
    w(base + 0xC, cmd | (1u << 10) | (param << 16));             // CMDACT
    uint64_t end = Host::deadline(100);
    while (r(base + 0xC) & (1u << 10)) {
      if (mach_absolute_time() > end) { native_usb_errors++; return false; }
    }
    return ((r(base + 0xC) >> 12) & 0xF) == 0;
  }

  // Physical endpoint configuration: type 0 control, 2 bulk; max packet
  bool configure(unsigned ep, unsigned type, unsigned maxPacket, bool modify = false) {
    uint32_t p0 = (type << 1) | (maxPacket << 3) | (modify ? (2u << 30) : 0);
    uint32_t p1 = (1u << 10) | (1u << 8) | (ep << 25);                   // USB endpoint number and direction = physical index
    return command(ep, CMD_SETEPCONFIG, p0, p1) && (modify || command(ep, CMD_SETTRANSFRESOURCE, 1));
  }

  bool startTransfer(unsigned ep, uint32_t trbAt, uint32_t bufAt, uint32_t length, uint32_t type, uint32_t &resource) {
    Trb *t = trb(trbAt);
    t->bpl = memPhys + bufAt; t->bph = 0; t->size = length;
    t->ctrl = type | TRB_HWO | TRB_LST | TRB_IOC | (type == TRBCTL_NORMAL ? TRB_ISP_IMI : 0);
    __asm__ volatile("dsb sy" ::: "memory");
    if (!command(ep, CMD_STARTTRANSFER, 0, memPhys + trbAt)) return false;
    resource = (r(DEPCMD + 0x10 * ep) >> 16) & 0x7F;
    return true;
  }

  void setupStage() {
    startTransfer(kEp0Out, kEp0Trb, kEp0Buf, 8, TRBCTL_CONTROL_SETUP, ep0Resource[0]);
  }

  void ep0Reply(const uint8_t *data, uint32_t length) {
    uint16_t wanted = static_cast<uint16_t>(setup[6] | (setup[7] << 8));
    if (length > wanted) length = wanted;
    if (length > 4096 - 64) length = 4096 - 64;
    for (uint32_t i = 0; i < length; i++) mem[kEp0Buf + i] = data[i];
    ep0DataIn = true;
    startTransfer(kEp0In, kEp0Trb, kEp0Buf, length, TRBCTL_CONTROL_DATA, ep0Resource[1]);
  }

  void ep0Status(bool in) {
    startTransfer(in ? kEp0In : kEp0Out, kEp0Trb, kEp0Buf, 0, ep0DataIn ? TRBCTL_CONTROL_STATUS3 : TRBCTL_CONTROL_STATUS2,
                  ep0Resource[in ? 1 : 0]);
  }

  void stall() {
    command(kEp0Out, CMD_SETSTALL);
    setupStage();
  }

  void handleSetup() {
    for (int i = 0; i < 8; i++) setup[i] = mem[kEp0Buf + i];
    uint8_t type = setup[0], request = setup[1];
    uint16_t value = static_cast<uint16_t>(setup[2] | (setup[3] << 8)), index = static_cast<uint16_t>(setup[4] | (setup[5] << 8));
    uint16_t length = static_cast<uint16_t>(setup[6] | (setup[7] << 8));
    ep0DataIn = false;
    if (type == 0x80 && request == 6) {                            // GET_DESCRIPTOR
      uint8_t kind = value >> 8, which = value & 0xFF;
      if (kind == 1) return ep0Reply(kDevice, sizeof(kDevice));
      if (kind == 2) return ep0Reply(kConfig, sizeof(kConfig));
      if (kind == 15) return ep0Reply(kBos, sizeof(kBos));
      if (kind == 3) {
        uint8_t s[64] = {};
        if (which == 0) { s[0] = 4; s[1] = 3; s[2] = 0x09; s[3] = 0x04; return ep0Reply(s, 4); }
        if (which < sizeof(kStrings) / sizeof(kStrings[0])) {
          const char *text = kStrings[which];
          unsigned n = 0;
          while (text[n] && n < 30) { s[2 + 2 * n] = static_cast<uint8_t>(text[n]); n++; }
          s[0] = static_cast<uint8_t>(2 + 2 * n); s[1] = 3;
          return ep0Reply(s, s[0]);
        }
      }
      return stall();
    }
    if (type == 0xC0 && request == kMsVendorCode && index == 7) return ep0Reply(gMsOs20, sizeof(gMsOs20));
    if (type == 0x00 && request == 5) {                            // SET_ADDRESS: applied before the status stage
      w(DCFG, (r(DCFG) & ~(0x7Fu << 3)) | ((value & 0x7F) << 3));
      address = value;
      return ep0Status(true);
    }
    if (type == 0x00 && request == 9) {                            // SET_CONFIGURATION
      if (value == 1 && !native_usb_configured) {
        configure(kEp1In, 2, 512);
        configure(kEp1Out, 2, 512);
        w(DALEPENA, r(DALEPENA) | (1u << kEp1In) | (1u << kEp1Out));
        native_usb_configured = 1;
        native_usb_stage = 6;
        receive();
        logPos = 0;                                               // a new host gets the whole log again
        if (shadow) memset(shadow, 0xA5, kFbBytes);              // and a whole frame
      }
      return ep0Status(true);
    }
    if (type == 0x80 && request == 0) {                            // GET_STATUS
      static const uint8_t zero[2] = {0, 0};
      return ep0Reply(zero, 2);
    }
    if (type == 0x80 && request == 8) {                            // GET_CONFIGURATION
      uint8_t c = native_usb_configured ? 1 : 0;
      return ep0Reply(&c, 1);
    }
    (void)length;
    if (!(type & 0x80)) return ep0Status(true);                    // other OUT requests without data: accept
    stall();
  }

  // Endpoint event: physical endpoint, event type (1 XferComplete, 3 XferNotReady)
  void endpointEvent(unsigned ep, unsigned type, uint32_t event) {
    if (ep == kEp0Out && type == 1) {
      if (ep0DataIn || ((trb(kEp0Trb)->ctrl >> 4) & 0x3F) == (TRBCTL_CONTROL_SETUP >> 4)) {
        if (((trb(kEp0Trb)->ctrl >> 4) & 0x3F) == (TRBCTL_CONTROL_SETUP >> 4)) return handleSetup();
      }
      setupStage();                                              // status OUT done: next setup
      return;
    }
    if (ep == kEp0In && type == 1) {
      if (ep0DataIn) { ep0Status(false); ep0DataIn = false; return; }   // data IN done: status OUT
      setupStage();                                              // status IN done
      return;
    }
    if (ep == kEp1In && type == 1) { inBusy = false; return; }
    if (ep == kEp1Out && type == 1) {
      uint32_t got = kOutBytes - (trb(kEp1OutTrb)->size & 0xFFFFFF);
      touch(mem + kOutBuf, got);
      receive();
      return;
    }
    (void)event;
  }

  void receive() { startTransfer(kEp1Out, kEp1OutTrb, kOutBuf, kOutBytes, TRBCTL_NORMAL, ep1OutResource); }

  void touch(volatile const uint8_t *b, uint32_t n) {
    if (sensors::fromMirror(b, n)) return;                       // "GSNS": synthetic sensor sample (GuampSensors.inc)
    if (n < 5 || b[0] != 'G' || b[1] != 'T' || b[2] != 'C' || b[3] != 'H') return;
    unsigned count = b[4];
    if (count > 12 || n < 5 + count * 5) return;
    native_touch_seq = native_touch_seq + 1;
    __asm__ volatile("dmb ish" ::: "memory");
    native_touch_count = count;
    for (unsigned i = 0; i < count; i++) {
      volatile const uint8_t *t = b + 5 + i * 5;
      uint32_t x = t[0] | (t[1] << 8), y = t[2] | (t[3] << 8);
      if (x >= kFbWidth) x = kFbWidth - 1;
      if (y >= kFbHeight) y = kFbHeight - 1;
      native_touch_xs[i] = x * 30479 / (kFbWidth - 1);       // the scale TouchService.inc publishes in
      native_touch_ys[i] = y * 20319 / (kFbHeight - 1);
      native_touch_ids[i] = t[4];
    }
    __asm__ volatile("dmb ish" ::: "memory");
    native_touch_seq = native_touch_seq + 1;
    native_touch_published();
    native_usb_touches++;
  }

  void deviceEvent(uint32_t event) {
    unsigned type = (event >> 8) & 0xF;
    if (type == 1) {                                               // USB reset
      native_usb_configured = 0; inBusy = false;
      w(DCFG, r(DCFG) & ~(0x7Fu << 3));
    } else if (type == 2) {                                        // connect done
      native_usb_speed = r(DSTS) & 7;
      configure(kEp0Out, 0, 64, true);
      configure(kEp0In, 0, 64, true);
      native_usb_stage = 5;
    }
  }

  void pollEvents() {
    uint32_t count = r(GEVNTCOUNT) & 0xFFFC;
    while (count) {
      uint32_t event = *reinterpret_cast<volatile uint32_t *>(mem + kEvents + eventPos);
      eventPos = (eventPos + 4) % kEventBytes;
      w(GEVNTCOUNT, 4);
      count -= 4;
      native_usb_events++;
      if (event & 1) deviceEvent(event);
      else endpointEvent((event >> 1) & 0x1F, (event >> 6) & 0xF, event);
    }
  }

  // Queue one bulk IN transfer of the staged bytes; waits for the previous one (polling events meanwhile)
  // Waits up to ms for the previous IN transfer to complete. A transfer the host has not read yet stays queued in the
  // controller: clearing inBusy on a timeout (as before) made every later DEPSTRTXFER fail on the still active endpoint,
  // so a viewer that opened the pipe a little late never got a byte. The staging buffer is only rewritten once idle.
  bool idle(uint32_t ms) {
    uint64_t end = Host::deadline(ms);
    while (inBusy && native_usb_configured) {
      pollEvents();
      if (!inBusy) break;
      if (mach_absolute_time() > end) {
        // Nobody read it for 4 timeouts in a row: end the transfer, so a stuck one does not stop the mirror for good
        if (++stuck >= 4) {
          command(kEp1In, CMD_ENDTRANSFER | (1u << 11), 0, 0, 0, ep1InResource);   // ForceRM
          native_usb_ended++;
          inBusy = false; stuck = 0;
        }
        return false;
      }
      IOSleep(1);   // do not spin a core while the host is not reading
    }
    stuck = 0;
    return native_usb_configured;
  }

  // One status row in the kernel log every 30 s (the mirror's own log, also in the kpan)
  void status() {
    uint64_t now = mach_absolute_time(), interval = 0;
    nanoseconds_to_absolutetime(30ULL * 1000 * 1000 * 1000, &interval);
    if (now - lastStatus < interval) return;
    lastStatus = now;
    IOLog("NativeEmmc USB mirror: stage %u configured %u speed %u in-busy %d bytes %llu frames %llu errors %llu ended %llu touches %llu"
          " events %llu, ep1 trb %08x cmd %08x\n", native_usb_stage, native_usb_configured, native_usb_speed, inBusy ? 1 : 0,
          native_usb_bytes, native_usb_frames, native_usb_errors, native_usb_ended, native_usb_touches, native_usb_events,
          trb(kEp1Trb)->ctrl, r(DEPCMD + 0x10 * kEp1In));
  }

  bool send(uint32_t bytes) {
    if (!idle(500)) { native_usb_errors++; return false; }
    inBusy = true;
    if (!startTransfer(kEp1In, kEp1Trb, kStage, bytes, TRBCTL_NORMAL, ep1InResource)) { inBusy = false; return false; }
    native_usb_bytes += bytes;
    return true;
  }

  uint32_t header(uint32_t at, uint32_t type, uint32_t length, uint32_t arg) {
    volatile uint8_t *h = mem + kStage + at;
    h[0] = 'G'; h[1] = 'M'; h[2] = 'I'; h[3] = 'R';
    uint32_t words[3] = {type, length, arg};
    for (int i = 0; i < 12; i++) h[4 + i] = reinterpret_cast<uint8_t *>(words)[i];
    return at + 16;
  }

  void sendLog() {
    guamp_msgbuf *ring = msgbufp;
    if (!ring || !ring->msg_bufc || ring->msg_size <= 0) return;
    uint32_t size = static_cast<uint32_t>(ring->msg_size), head = static_cast<uint32_t>(ring->msg_bufx) % size;
    while (logPos != head && native_usb_configured) {
      if (!idle(500)) return;
      uint32_t n = 0, at = 16;
      while (logPos != head && n < kStageBytes - 32) {
        mem[kStage + at + n] = static_cast<uint8_t>(ring->msg_bufc[logPos]);
        logPos = (logPos + 1) % size; n++;
      }
      header(0, 3, n, 0);
      if (!send(16 + n)) return;
    }
  }

  void sendFrame(volatile const uint8_t *fb) {
    if (!fb || !shadow) return;
    if (!idle(500)) return;
    uint32_t at = header(0, 1, 16, 0);
    uint32_t info[4] = {kFbWidth, kFbHeight, kFbRowBytes, 1};
    for (int i = 0; i < 16; i++) mem[kStage + at + i] = reinterpret_cast<uint8_t *>(info)[i];
    if (!send(32)) return;
    const uint32_t rowsPerMessage = (kStageBytes - 16) / kFbRowBytes;
    for (uint32_t y = 0; y < kFbHeight && native_usb_configured;) {
      // skip unchanged rows, then send a run of changed ones
      while (y < kFbHeight) {
        bool same = true;
        for (uint32_t i = (y & 3) * 8; i < kFbRowBytes; i += 32) {   // one word in four, offset by row
          uint64_t a = *reinterpret_cast<volatile const uint64_t *>(fb + y * kFbRowBytes + i);
          if (memcmp(&a, shadow + y * kFbRowBytes + i, 8)) { same = false; break; }
        }
        if (!same) break;
        y++;
      }
      if (y >= kFbHeight) break;
      if (!idle(500)) return;
      uint32_t first = y, rows = 0, off = 16;
      while (y < kFbHeight && rows < rowsPerMessage) {
        for (uint32_t i = 0; i < kFbRowBytes; i += 8) {
          uint64_t a = *reinterpret_cast<volatile const uint64_t *>(fb + y * kFbRowBytes + i);
          memcpy(shadow + y * kFbRowBytes + i, &a, 8);
          *reinterpret_cast<volatile uint64_t *>(mem + kStage + off + i) = a;
        }
        off += kFbRowBytes; rows++; y++;
      }
      header(0, 2, rows * kFbRowBytes, first);
      if (!send(off)) return;
    }
    if (!idle(500)) return;
    header(0, 4, 0, ++frame);
    send(16);
    native_usb_frames++;
  }

  bool start() {
    volatile uint8_t *gcc = mapPhysical(kGccUsbPhys, 0x4000);
    if (!gcc) return false;
    auto g = [gcc](uint32_t offset) { return *reinterpret_cast<volatile uint32_t *>(gcc + offset - 0x1a000); };
    bool on = (g(0x1a004) & (1u << 31)) && !(g(0x1a010) & (1u << 31)) && !(g(0x1a084) & (1u << 31)) && !(g(0x1a080) & (1u << 31));
    native_usb_stage = 1;
    if (!on) { IOLog("NativeEmmc USB mirror: controller not powered/clocked, not started\n"); return false; }
    dwc = mapPhysical(kDwcPhys, 0x10000);
    qscratch = mapPhysical(kQscratchPhys, 0x400);
    dma = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
        static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous) |
        static_cast<IOOptionBits>(kIOMapInhibitCache), kDmaBytes, 0xFFFFF000ULL);
    shadow = static_cast<uint8_t *>(IOMallocZero(kFbBytes));
    if (!dwc || !qscratch || !dma || !shadow) return false;
    mem = static_cast<volatile uint8_t *>(dma->getBytesNoCopy());
    uint64_t phys = dma->getPhysicalSegment(0, nullptr, kIOMemoryMapperNone);
    if (!mem || !phys || phys + kDmaBytes > 0x100000000ULL) return false;
    memPhys = static_cast<uint32_t>(phys);
    for (uint32_t i = 0; i < kDmaBytes; i++) mem[i] = 0;
    memset(shadow, 0xA5, kFbBytes);
    buildMsOs20();
    native_usb_stage = 2;
    uint32_t id = r(GSNPSID);
    if ((id >> 16) != 0x5533 && (id >> 16) != 0x3331) { IOLog("NativeEmmc USB mirror: unexpected GSNPSID %08x\n", id); return false; }
    // Stop whatever the firmware left running, soft-reset the device core
    w(DCTL, r(DCTL) & ~(1u << 31));
    uint64_t end = Host::deadline(100);
    while (!(r(DSTS) & (1u << 22)) && mach_absolute_time() < end) {}   // DEVCTRLHLT
    w(DCTL, 1u << 30);                                             // CSFTRST
    end = Host::deadline(500);
    while ((r(DCTL) & (1u << 30)) && mach_absolute_time() < end) {}
    if (r(DCTL) & (1u << 30)) { IOLog("NativeEmmc USB mirror: core soft reset stuck\n"); return false; }
    native_usb_stage = 3;
    // VBUS/session valid from software (no PMIC VBUS detection here), as Linux dwc3-qcom vbus override
    *reinterpret_cast<volatile uint32_t *>(qscratch + 0x10) |= (1u << 20) | (1u << 28);   // HS_PHY_CTRL: UTMI_OTG_VBUS_VALID, SW_SESSVLD_SEL
    *reinterpret_cast<volatile uint32_t *>(qscratch + 0x30) |= (1u << 24);                // SS_PHY_CTRL: LANE0_PWR_PRESENT
    // Device role, USB 2.0 only (no SS PHY bring-up), UTMI 16-bit turnaround as the firmware set it
    w(GCTL, (r(GCTL) & ~(3u << 12)) | (2u << 12));
    w(GUSB2PHYCFG, r(GUSB2PHYCFG) & ~((1u << 6) | (1u << 8)));    // no SUSPENDUSB20, no ENBLSLPM
    w(GUSB3PIPECTL, r(GUSB3PIPECTL) & ~(1u << 17));                // no SUSPHY
    w(GEVNTADRLO, memPhys + kEvents); w(GEVNTADRHI, 0);
    w(GEVNTSIZ, kEventBytes);
    w(GEVNTCOUNT, r(GEVNTCOUNT) & 0xFFFC);
    w(DCFG, (r(DCFG) & ~(7u | (0x7Fu << 3))) | 0);                 // high speed, address 0
    w(DEVTEN, (1u << 1) | (1u << 2));                              // reset, connect done
    if (!command(kEp0Out, CMD_DEPSTARTCFG, 0, 0, 0, 0)) { IOLog("NativeEmmc USB mirror: DEPSTARTCFG failed\n"); return false; }
    configure(kEp0Out, 0, 64);
    configure(kEp0In, 0, 64);
    w(DALEPENA, (1u << kEp0Out) | (1u << kEp0In));
    setupStage();
    native_usb_stage = 4;
    w(DCTL, r(DCTL) | (1u << 31));                                 // RUN_STOP: the host sees the device
    running = true;
    IOLog("NativeEmmc USB mirror: running (GSNPSID %08x, DMA at %08x)\n", id, memPhys);
    return true;
  }
};

Mirror gMirror;

void thread(void *, wait_result_t)
{
  Mirror &m = gMirror;
  uint64_t lastFrame = 0;
  for (;;) {
    m.pollEvents();
    m.status();
    if (native_usb_configured) {
      m.sendLog();
      uint64_t now = mach_absolute_time(), interval = 0;
      nanoseconds_to_absolutetime(500 * 1000 * 1000ULL, &interval);   // 2 frames per second: the frame buffer is uncached
      if (now - lastFrame > interval) {
        // A whole frame every 30 frames (~3 s): a viewer opened later, or after a lost transfer, catches up
        if (m.frame % 6 == 0 && m.shadow) memset(m.shadow, 0xA5, kFbBytes);                  // whole frame every ~3 s
        m.sendFrame(gCard.fb);
        lastFrame = now;
      }
    }
    IOSleep(native_usb_configured ? 5 : 20);   // leave the CPU to iOS
  }
}

}  // namespace usbm

static void usbMirrorStart()
{
  uint32_t enabled = 1;   // default on (verified 28/10): guamp-usb-mirror=0 disables
  PE_parse_boot_argn("guamp-usb-mirror", &enabled, sizeof(enabled));
  if (!enabled) return;
  if (!usbm::gMirror.start()) return;
  thread_t thread = nullptr;
  if (kernel_thread_start(usbm::thread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
}
