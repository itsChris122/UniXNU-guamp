// Host tests of GuampBtShim.h: H4 reassembly (all packet types, any split, resync, oversize), the encoders, the packet
// FIFO, Broadcom vendor emulation, controller-reply rewriting, tx failures, the serial ioctls, and a random-input run
// (ASan/UBSan in run-bt-shim-tests.sh).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "../GuampBtShim.h"
#include "../BtHci.h"      // bt_ibs_scan: the transport side of BtService.inc

static unsigned gFail = 0, gChecks = 0;
#define CHECK(c) do { gChecks = gChecks + 1; if (!(c)) { gFail = gFail + 1; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

typedef std::vector<uint8_t> Bytes;
struct Sink { std::vector<Bytes> pkts; };
static void sinkFn(void *ctx, const uint8_t *p, size_t n) { static_cast<Sink *>(ctx)->pkts.push_back(Bytes(p, p + n)); }

static Bytes pktCmd(uint16_t op, const Bytes &params) {
    Bytes b = {BTS_H4_CMD, (uint8_t)op, (uint8_t)(op >> 8), (uint8_t)params.size()};
    b.insert(b.end(), params.begin(), params.end());
    return b;
}
static Bytes pktAcl(uint16_t handle, size_t n) {
    Bytes b = {BTS_H4_ACL, (uint8_t)handle, (uint8_t)(handle >> 8), (uint8_t)n, (uint8_t)(n >> 8)};
    for (size_t i = 0; i < n; i++) b.push_back((uint8_t)(i * 7));
    return b;
}
static Bytes pktSco(size_t n) { Bytes b = {BTS_H4_SCO, 1, 0, (uint8_t)n}; for (size_t i = 0; i < n; i++) b.push_back((uint8_t)i); return b; }
static Bytes pktIso(size_t n) {
    Bytes b = {BTS_H4_ISO, 2, 0x40, (uint8_t)n, (uint8_t)((n >> 8) | 0xc0)};   // top 2 bits of the length word are flags
    for (size_t i = 0; i < n; i++) b.push_back((uint8_t)(i ^ 0x5a));
    return b;
}
static Bytes pktEvt(uint8_t code, const Bytes &p) { Bytes b = {BTS_H4_EVT, code, (uint8_t)p.size()}; b.insert(b.end(), p.begin(), p.end()); return b; }
static Bytes cc(uint16_t op, const Bytes &ret) { Bytes p = {1, (uint8_t)op, (uint8_t)(op >> 8)}; p.insert(p.end(), ret.begin(), ret.end()); return pktEvt(BTS_EVT_CMD_COMPLETE, p); }

static void testReassembly() {
    std::vector<Bytes> in = {pktCmd(0x0c03, {}), pktAcl(0x0042, 27), pktSco(60), pktEvt(0x13, {1, 2, 3, 4, 5}), pktIso(100),
                             pktEvt(0x05, {}), pktAcl(1, 0), pktCmd(0x2008, Bytes(31, 0xaa)), pktAcl(3, 1021)};
    Bytes all;
    for (auto &p : in) all.insert(all.end(), p.begin(), p.end());
    for (size_t chunk : {(size_t)1, (size_t)2, (size_t)3, (size_t)7, (size_t)64, all.size()}) {
        static bts_h4 r; memset(&r, 0, sizeof r);
        Sink s;
        unsigned got = 0;
        for (size_t at = 0; at < all.size(); at += chunk) got += bts_h4_feed(&r, all.data() + at, std::min(chunk, all.size() - at), sinkFn, &s);
        CHECK(got == in.size());
        CHECK(s.pkts == in);
        CHECK(r.skipped == 0 && r.oversize == 0 && r.have == 0);
    }
    // Garbage and IBS bytes between packets are skipped; an oversize ACL is passed over and the next packet still parses.
    static bts_h4 r; memset(&r, 0, sizeof r);
    Sink s;
    Bytes g = {0x00, 0xfd, 0xfe, 0xfc, 0x99};
    Bytes big = {BTS_H4_ACL, 1, 0, 0xd0, 0x07};   // 2000 bytes
    big.resize(5 + 2000, 0x01);
    Bytes evt = pktEvt(0x0e, {1, 3, 0x0c, 0});
    Bytes all2 = g; all2.insert(all2.end(), big.begin(), big.end()); all2.insert(all2.end(), evt.begin(), evt.end());
    bts_h4_feed(&r, all2.data(), all2.size(), sinkFn, &s);
    CHECK(r.skipped == 5);
    CHECK(r.oversize == 1);
    CHECK(s.pkts.size() == 1 && s.pkts[0] == evt);
    // ISO length keeps 14 bits only.
    Bytes iso = pktIso(3);
    memset(&r, 0, sizeof r); s.pkts.clear();
    bts_h4_feed(&r, iso.data(), iso.size(), sinkFn, &s);
    CHECK(s.pkts.size() == 1 && s.pkts[0].size() == 8);
}

static void testEncoders() {
    uint8_t o[300];
    const uint8_t ret[2] = {0x11, 0x22};
    size_t n = bts_cmd_complete(o, sizeof o, 0xfc4c, 0, ret, 2);
    const uint8_t want[] = {4, 0x0e, 6, 1, 0x4c, 0xfc, 0, 0x11, 0x22};
    CHECK(n == sizeof want && !memcmp(o, want, n));
    CHECK(bts_cmd_complete(o, 8, 0xfc4c, 0, ret, 2) == 0);
    CHECK(bts_cmd_complete(o, sizeof o, 1, 0, o, 252) == 0);   // plen 256 does not fit one byte
    n = bts_cmd_status(o, sizeof o, 0x0405, 3);
    const uint8_t ws[] = {4, 0x0f, 4, 3, 1, 5, 4};
    CHECK(n == 7 && !memcmp(o, ws, 7));
    CHECK(bts_cmd_status(o, 6, 1, 0) == 0);
}

static void testFifo() {
    static bts_fifo f; memset(&f, 0, sizeof f);
    Bytes a(BTS_RX_FIFO - 10, 0x33);
    CHECK(bts_fifo_put(&f, a.data(), a.size()));
    uint8_t b[20] = {};
    CHECK(!bts_fifo_put(&f, b, 11));    // all or nothing
    CHECK(f.count == a.size());
    CHECK(bts_fifo_put(&f, b, 10));
    uint8_t out[BTS_RX_FIFO];
    CHECK(bts_fifo_get(&f, out, 100) == 100);
    for (int i = 0; i < 50; i++) b[i % 20] = (uint8_t)i;
    CHECK(bts_fifo_put(&f, b, 20));     // wraps
    size_t got = bts_fifo_get(&f, out, sizeof out);
    CHECK(got == BTS_RX_FIFO - 100 + 20);
    CHECK(!memcmp(out + got - 20, b, 20));
    CHECK(f.count == 0 && f.head == 0);
}

struct Link { std::vector<Bytes> sent; int fail; uint8_t addr[6]; int addrCalls; };
static int linkTx(void *ctx, const uint8_t *p, size_t n) { Link *l = (Link *)ctx; if (l->fail) return -1; l->sent.push_back(Bytes(p, p + n)); return 0; }
static void linkAddr(void *ctx, const uint8_t a[6]) { Link *l = (Link *)ctx; memcpy(l->addr, a, 6); l->addrCalls++; }
static Bytes drain(bts_shim *s) { Bytes b(bts_shim_readable(s)); bts_shim_read(s, b.data(), b.size()); return b; }

static void testShim() {
    static bts_shim s;
    Link l = {};
    bts_link link = {&l, linkTx, linkAddr, nullptr, nullptr};
    bts_shim_init(&s, &link, nullptr);
    // Standard HCI goes through untouched, in one write or byte by byte.
    Bytes reset = pktCmd(0x0c03, {}), acl = pktAcl(5, 40);
    Bytes w = reset; w.insert(w.end(), acl.begin(), acl.end());
    for (size_t i = 0; i < w.size(); i++) bts_shim_host_write(&s, &w[i], 1);
    CHECK(l.sent.size() == 2 && l.sent[0] == reset && l.sent[1] == acl);
    CHECK(bts_shim_readable(&s) == 0);
    // Broadcom vendor commands are answered here and never reach the controller.
    Bytes baud = pktCmd(0xfc18, {0, 0, 0xc0, 0xc6, 0x2d, 0x00});   // 3000000
    Bytes bd = pktCmd(0xfc01, {1, 2, 3, 4, 5, 6});
    Bytes mini = pktCmd(0xfc2e, {});
    Bytes wr = pktCmd(0xfc4c, Bytes(4 + 251, 0x77));
    Bytes launch = pktCmd(0xfc4e, {0xff, 0xff, 0xff, 0xff});
    Bytes verbose = pktCmd(0xfc79, {});
    Bytes unknown = pktCmd(0xfc00, {0x19});   // the QCA EDL opcode must not reach the QCA chip from bluetoothd
    for (auto *p : {&baud, &bd, &mini, &wr, &wr, &launch, &verbose, &unknown}) bts_shim_host_write(&s, p->data(), p->size());
    CHECK(l.sent.size() == 2);
    CHECK(s.baud == 3000000);
    CHECK(s.bdaddrSet && l.addrCalls == 1 && l.addr[0] == 1 && l.addr[5] == 6);
    CHECK(s.st.writeRamCmds == 2 && s.st.writeRamBytes == 2 * 251 && s.st.launches == 1 && s.st.minidriver == 1);
    CHECK(!s.downloading);
    CHECK(s.st.emulated == 7 && s.st.unknownVendor == 1);
    Bytes got = drain(&s);
    Bytes want;
    for (uint16_t op : {0xfc18, 0xfc01, 0xfc2e, 0xfc4c, 0xfc4c, 0xfc4e}) { Bytes c = cc(op, {0}); want.insert(want.end(), c.begin(), c.end()); }
    Bytes cv = cc(0xfc79, {0, 0x55, 0, 0, 0, 1, 0}); want.insert(want.end(), cv.begin(), cv.end());
    Bytes cu = cc(0xfc00, {BTS_STATUS_UNKNOWN_COMMAND}); want.insert(want.end(), cu.begin(), cu.end());
    CHECK(got == want);
    // A failed command transmit answers Command Status hardware failure; a failed ACL is only counted.
    l.fail = 1;
    bts_shim_host_write(&s, reset.data(), reset.size());
    bts_shim_host_write(&s, acl.data(), acl.size());
    got = drain(&s);
    Bytes st = pktEvt(BTS_EVT_CMD_STATUS, {BTS_STATUS_HARDWARE_FAILURE, 1, 3, 0x0c});
    CHECK(got == st);
    CHECK(s.st.txErrors == 2);
    l.fail = 0;
    // fc01 also sent to the WCN3950 as EDL_WRITE_BD_ADDR 0xfc14 when asked; its replies do not reach bluetoothd.
    s.id.translateBdaddr = true;
    l.sent.clear();
    bts_shim_host_write(&s, bd.data(), bd.size());
    CHECK(l.sent.size() == 1 && l.sent[0] == pktCmd(0xfc14, {6, 5, 4, 3, 2, 1}));   // qca_set_bdaddr baswap
    CHECK(drain(&s) == cc(0xfc01, {0}));
    Bytes qv = pktEvt(0xff, {0x00, 0x14}), qcc = cc(0xfc14, {0}), other = cc(0x0c03, {0});
    for (auto *q : {&qv, &qcc, &qcc, &other}) bts_shim_ctrl_input(&s, q->data(), q->size());
    Bytes rest = qcc; rest.insert(rest.end(), other.begin(), other.end());   // only the first fc14 reply is internal
    CHECK(drain(&s) == rest);
    CHECK(s.st.internalDropped == 1 && s.internalOpcode == 0);
    // No link at all: commands fail the same way instead of hanging bluetoothd.
    static bts_shim s2; bts_shim_init(&s2, nullptr, nullptr);
    bts_shim_host_write(&s2, reset.data(), reset.size());
    CHECK(drain(&s2) == st);
}

static void testRewrite() {
    static bts_shim s;
    bts_shim_init(&s, nullptr, nullptr);
    CHECK(s.id.lmpSubver == 0x6103 && s.id.manufacturer == 15 && s.id.usbProduct[0] == 0x5c && s.id.usbProduct[1] == 0x0a);
    s.id.lmpSubver = 0x4321; s.id.hciRev = 0x0102;
    // QCA Read Local Version reply: HCI 5.0, rev 0x0000, LMP 5.0, Qualcomm (29), subver 0x0120.
    Bytes v = cc(0x1001, {0, 9, 0, 0, 9, 29, 0, 0x20, 0x01});
    bts_shim_ctrl_input(&s, v.data(), v.size());
    Bytes got = drain(&s);
    CHECK(got == cc(0x1001, {0, 9, 2, 1, 9, 15, 0, 0x21, 0x43}));
    // A failed Read Local Version is passed as is.
    Bytes vf = cc(0x1001, {0x0c, 9, 0, 0, 9, 29, 0, 0x20, 0x01});
    bts_shim_ctrl_input(&s, vf.data(), vf.size());
    CHECK(drain(&s) == vf);
    // Read Local Name: the name becomes the identity's, zero padded.
    Bytes nameRet(1 + BTS_NAME_LEN, 0); const char *q = "WCN3950 Qualcomm"; memcpy(nameRet.data() + 1, q, strlen(q));
    Bytes nm = cc(0x0c14, nameRet);
    bts_shim_ctrl_input(&s, nm.data(), nm.size());
    got = drain(&s);
    CHECK(got.size() == nm.size());
    CHECK(!memcmp(got.data() + 7, "BCM4355C0", 10));
    bool zeros = true; for (size_t i = 7 + 9; i < got.size(); i++) zeros = zeros && got[i] == 0;
    CHECK(zeros);
    // Vendor events are dropped by default, passed when asked; IBS bytes in the stream are skipped.
    Bytes ve = pktEvt(0xff, {0x00, 0x02, 1, 2, 3});
    Bytes conn = pktEvt(0x03, Bytes(11, 0x42));
    Bytes mix = {0xfd}; mix.insert(mix.end(), ve.begin(), ve.end()); mix.push_back(0xfe); mix.insert(mix.end(), conn.begin(), conn.end());
    bts_shim_ctrl_input(&s, mix.data(), mix.size());
    CHECK(drain(&s) == conn);
    CHECK(s.st.vendorDropped == 1 && s.ctrl.skipped == 2);
    s.id.passVendorEvents = true;
    bts_shim_ctrl_input(&s, ve.data(), ve.size());
    CHECK(drain(&s) == ve);
    // ACL from the controller passes; when the FIFO is full a packet is dropped whole, never split.
    Bytes acl = pktAcl(1, 1000);
    unsigned put = 0;
    for (int i = 0; i < 40; i++) { size_t before = bts_shim_readable(&s); bts_shim_ctrl_input(&s, acl.data(), acl.size()); put += bts_shim_readable(&s) > before; }
    CHECK(put == BTS_RX_FIFO / acl.size());
    CHECK(s.st.rxDropped == 40 - put);
    CHECK(bts_shim_readable(&s) == put * acl.size());
}

static uint32_t ioc(bool in, bool out, char g, unsigned num, unsigned len) {
    return (in ? BTS_IOC_IN : 0) | (out ? BTS_IOC_OUT : 0) | (len & 0x1fff) << 16 | (uint32_t)(uint8_t)g << 8 | num;
}
static void testTty() {
    bts_tty t; bts_tty_init(&t);
    int flush = 0;
    uint8_t d[80] = {};
    // Defaults in the 64-bit layout: CS8|CREAD|CLOCAL, 115200 both ways.
    CHECK(bts_tty_ioctl(&t, ioc(false, true, 't', 19, 72), d, 0, &flush) == BTS_IOCTL_OK);
    CHECK(bts_rd_u64(d + 16) == (BTS_CS8 | BTS_CREAD | BTS_CLOCAL) && bts_rd_u64(d + 56) == 115200 && bts_rd_u64(d + 64) == 115200);
    // Set (cfmakeraw + 3 Mbaud) and read back the same bytes.
    for (int i = 0; i < 72; i++) d[i] = (uint8_t)(i * 3);
    bts_wr_u64(d + 64, 3000000);
    uint8_t keep[72]; memcpy(keep, d, 72);
    CHECK(bts_tty_ioctl(&t, ioc(true, false, 't', 20, 72), d, 0, &flush) == BTS_IOCTL_OK);
    CHECK(t.speed == 3000000);
    memset(d, 0, sizeof d);
    CHECK(bts_tty_ioctl(&t, ioc(false, true, 't', 19, 72), d, 0, &flush) == BTS_IOCTL_OK && !memcmp(d, keep, 72));
    CHECK(bts_tty_ioctl(&t, ioc(true, false, 't', 22, 72), keep, 0, &flush) == BTS_IOCTL_FLUSH && flush == 1);
    // 32-bit layout defaults, then a wrong size.
    bts_tty t2; bts_tty_init(&t2);
    CHECK(bts_tty_ioctl(&t2, ioc(false, true, 't', 19, 44), d, 0, &flush) == BTS_IOCTL_OK);
    CHECK(bts_rd_u32(d + 8) == (BTS_CS8 | BTS_CREAD | BTS_CLOCAL) && bts_rd_u32(d + 40) == 115200);
    CHECK(bts_tty_ioctl(&t2, ioc(false, true, 't', 19, 40), d, 0, &flush) == BTS_IOCTL_INVAL);
    // Modem lines.
    uint8_t m[4];
    bts_wr_u32(m, BTS_TIOCM_DTR | BTS_TIOCM_RTS);
    CHECK(bts_tty_ioctl(&t, ioc(true, false, 't', 107, 4), m, 0, &flush) == BTS_IOCTL_OK);   // TIOCMBIC
    CHECK(bts_tty_ioctl(&t, ioc(false, true, 't', 106, 4), m, 0, &flush) == BTS_IOCTL_OK);
    CHECK((bts_rd_u32(m) & (BTS_TIOCM_DTR | BTS_TIOCM_RTS)) == 0 && (bts_rd_u32(m) & BTS_TIOCM_CTS));
    CHECK(bts_tty_ioctl(&t, ioc(false, false, 't', 121, 0), nullptr, 0, &flush) == BTS_IOCTL_OK && (t.modem & BTS_TIOCM_DTR));
    // FIONREAD, FIONBIO, IOSSIOSPEED, TIOCFLUSH 0 = both, TIOCEXCL, unknown.
    CHECK(bts_tty_ioctl(&t, ioc(false, true, 'f', 127, 4), m, 1234, &flush) == BTS_IOCTL_OK && bts_rd_u32(m) == 1234);
    bts_wr_u32(m, 1);
    CHECK(bts_tty_ioctl(&t, ioc(true, false, 'f', 126, 4), m, 0, &flush) == BTS_IOCTL_OK && t.nonblocking);
    uint8_t sp[8]; bts_wr_u64(sp, 921600);
    CHECK(bts_tty_ioctl(&t, ioc(true, false, 'T', 2, 8), sp, 0, &flush) == BTS_IOCTL_OK && t.speed == 921600);
    bts_wr_u32(m, 0);
    CHECK(bts_tty_ioctl(&t, ioc(true, false, 't', 16, 4), m, 0, &flush) == BTS_IOCTL_FLUSH && flush == 3);
    CHECK(bts_tty_ioctl(&t, ioc(false, false, 't', 13, 0), nullptr, 0, &flush) == BTS_IOCTL_OK && t.exclusive);
    CHECK(bts_tty_ioctl(&t, ioc(false, true, 't', 200, 4), m, 0, &flush) == BTS_IOCTL_NOTTY);
    CHECK(bts_tty_ioctl(&t, ioc(false, true, 'x', 1, 4), m, 0, &flush) == BTS_IOCTL_NOTTY);
    CHECK(bts_tty_ioctl(&t, ioc(false, true, 't', 106, 4), nullptr, 0, &flush) == BTS_IOCTL_INVAL);
}

// Random bytes both ways, random splits: no crash, no out-of-bounds (sanitizers), invariants hold.
static void testRandom() {
    static bts_shim s;
    Link l = {};
    bts_link link = {&l, linkTx, nullptr, nullptr, nullptr};
    bts_shim_init(&s, &link, nullptr);
    srand(12345);
    Bytes buf(4096);
    uint64_t total = 0;
    for (int round = 0; round < 20000; round++) {
        size_t n = (size_t)(rand() % 600);
        for (size_t i = 0; i < n; i++) {
            int r = rand() % 10;
            buf[i] = r < 3 ? (uint8_t)(1 + rand() % 5) : (uint8_t)rand();   // bias toward valid type bytes
        }
        if (rand() & 1) bts_shim_host_write(&s, buf.data(), n); else bts_shim_ctrl_input(&s, buf.data(), n);
        total += n;
        if (rand() % 4 == 0) { uint8_t o[700]; bts_shim_read(&s, o, (size_t)(rand() % 700)); }
        if (l.sent.size() > 1000) l.sent.clear();
        CHECK(s.rx.count <= BTS_RX_FIFO);
        CHECK(s.host.have < BTS_PKT_MAX && s.ctrl.have < BTS_PKT_MAX);
    }
    for (auto &p : l.sent) CHECK(p.size() >= 4 && p.size() <= BTS_PKT_MAX && bts_h4_header(p[0]));
    printf("random: %llu bytes, host %u pkts, ctrl %u pkts, skipped %u/%u, oversize %u/%u\n", (unsigned long long)total,
           s.st.hostPkts, s.st.ctrlPkts, s.host.skipped, s.ctrl.skipped, s.host.oversize, s.ctrl.oversize);
}

// End to end as on the phone: a fake bluetoothd writes to the shim, the shim's link goes to a fake WCN3950 (Qualcomm
// identity), the controller's byte stream passes the transport scanner of BtService.inc (IBS bytes between packets,
// payload bytes with IBS values) into bts_shim_ctrl_input, and bluetoothd reads a BCM4355C0.
struct FakeQca { std::vector<Bytes> got; Bytes out; };
static int qcaTx(void *ctx, const uint8_t *p, size_t n) {
    FakeQca *q = (FakeQca *)ctx;
    q->got.push_back(Bytes(p, p + n));
    if (n < 4 || p[0] != BTS_H4_CMD) return 0;
    const uint16_t op = (uint16_t)(p[1] | p[2] << 8);
    Bytes reply;
    if (op == BTS_OP_READ_LOCAL_VERSION) reply = cc(op, {0x00, 0x09, 0x00, 0x00, 0x09, 0x1d, 0x00, 0x08, 0x01});   // Qualcomm 0x1d
    else if (op == BTS_OP_READ_LOCAL_NAME) {
        Bytes r(1 + BTS_NAME_LEN, 0);
        memcpy(r.data() + 1, "WCN3950", 7);
        reply = cc(op, r);
    } else reply = cc(op, {0x00});
    q->out.push_back(BT_IBS_SLEEP_IND);                 // IBS bytes before the answer
    q->out.push_back(BT_IBS_WAKE_IND);
    q->out.insert(q->out.end(), reply.begin(), reply.end());
    return 0;
}
static void testEndToEnd() {
    FakeQca q;
    bts_link l = {};
    l.ctx = &q;
    l.tx = qcaTx;
    bts_shim *s = (bts_shim *)calloc(1, sizeof(bts_shim));
    bts_shim_init(s, &l, nullptr);
    bt_ibs_scan scan = {};
    unsigned wakeAcks = 0;
    auto pump = [&](size_t chunk) {                      // the transport loop: UART reads of chunk bytes
        for (size_t i = 0; i < q.out.size(); i += chunk) {
            const size_t n = q.out.size() - i < chunk ? q.out.size() - i : chunk;
            wakeAcks += bt_ibs_scan_feed(&scan, q.out.data() + i, n);
            bts_shim_ctrl_input(s, q.out.data() + i, n);
        }
        q.out.clear();
    };
    Bytes cmd = pktCmd(BTS_OP_READ_LOCAL_VERSION, {});
    bts_shim_host_write(s, cmd.data(), cmd.size());
    CHECK(q.got.size() == 1 && q.got[0] == cmd);
    pump(3);
    Bytes ev = drain(s);
    CHECK(ev.size() == 3 + 3 + 9 && ev[0] == BTS_H4_EVT && ev[1] == BTS_EVT_CMD_COMPLETE);
    if (ev.size() == 15) {
        CHECK(bts_le16(&ev[4]) == BTS_OP_READ_LOCAL_VERSION && ev[6] == 0);
        CHECK(bts_le16(&ev[11]) == 15 && bts_le16(&ev[13]) == 0x6103);
    }
    CHECK(wakeAcks == 1 && scan.sleep == 1 && scan.packets == 1 && scan.noise == 0);
    cmd = pktCmd(BTS_OP_READ_LOCAL_NAME, {});
    bts_shim_host_write(s, cmd.data(), cmd.size());
    pump(64);
    ev = drain(s);
    CHECK(ev.size() == 3 + 3 + 1 + BTS_NAME_LEN && !memcmp(ev.data() + 7, "BCM4355C0", 10));
    CHECK(wakeAcks == 2 && scan.packets == 2);
    // An ACL packet whose payload holds IBS byte values: not counted, delivered unchanged.
    Bytes acl = {BTS_H4_ACL, 0x01, 0x20, 0x04, 0x00, BT_IBS_WAKE_IND, BT_IBS_SLEEP_IND, BT_IBS_WAKE_ACK, 0x00};
    q.out = acl;
    q.out.push_back(BT_IBS_WAKE_ACK);
    pump(2);
    ev = drain(s);
    CHECK(ev == acl);
    CHECK(wakeAcks == 2 && scan.sleep == 2 && scan.ack == 1 && scan.packets == 3 && scan.noise == 0);
    free(s);
}


// Review fixes: IBS state in stream order, and a controller reset dropping a half packet.
static void testReview() {
    bt_ibs_scan sc = {};
    Bytes ev = cc(0x0c03, {0});
    Bytes a = ev; a.push_back(BT_IBS_SLEEP_IND);            // event then SLEEP_IND in one read: asleep
    bt_ibs_scan_feed(&sc, a.data(), a.size());
    CHECK(sc.state == 2 && sc.packets == 1 && sc.sleep == 1);
    Bytes b = {BT_IBS_SLEEP_IND}; b.insert(b.end(), ev.begin(), ev.end());   // SLEEP_IND then event: awake
    bt_ibs_scan_feed(&sc, b.data(), b.size());
    CHECK(sc.state == 1);
    Bytes noise = {0x00, 0x77};                              // noise leaves the state as it was
    bt_ibs_scan_feed(&sc, noise.data(), noise.size());
    CHECK(sc.state == 1);
    bt_ibs_scan fresh = {};
    bt_ibs_scan_feed(&fresh, noise.data(), noise.size());
    CHECK(fresh.state == 0);                                 // unknown: the transport keeps M1's view
    // Half a Read Local Version reply, then the controller is re-initialised (M1 consumes its bytes): without the
    // reset the next event would be appended to the stale header and bluetoothd would read garbage.
    static bts_shim s; bts_shim_init(&s, nullptr, nullptr);
    Bytes half = {BTS_H4_EVT, BTS_EVT_CMD_COMPLETE, 12, 1, 0x01};
    bts_shim_ctrl_input(&s, half.data(), half.size());
    bts_h4_reset(&s.ctrl);                                   // btshim::controllerReset()
    Bytes r = cc(0x0c03, {0});
    bts_shim_ctrl_input(&s, r.data(), r.size());
    uint8_t out[64]; const size_t n = bts_shim_read(&s, out, sizeof out);
    CHECK(n == r.size() && !memcmp(out, r.data(), n));
}

// Kernel control (com.apple.uart.bluetooth): socket options as BlueTool 0x10001ecc4 and bluetoothd 0x1000ddaa8 issue
// them, a write split over mbufs, and controller bytes leaving through a receive buffer of varying room.
static void testCtl() {
    bts_tty t; bts_tty_init(&t);
    int flush = -1;
    uint8_t tio[80]; size_t len = 72;
    memset(tio, 0xee, sizeof tio);
    CHECK(bts_ctl_sockopt(&t, 0x40487413, false, tio, &len, 0, &flush) == BTS_CTL_OPT_OK && len == 72);   // TIOCGETA
    uint64_t v; memcpy(&v, tio + 64, 8); CHECK(v == 115200);
    CHECK(tio[72] == 0xee);                                  // nothing past the 72-byte argument
    // cfmakeraw + cfsetspeed(3000000) + c_iflag |= IGNPAR, c_cflag = CS8|CREAD|CLOCAL|CCTS_OFLOW|CRTS_IFLOW
    v = 4; memcpy(tio, &v, 8); v = 0x38b00; memcpy(tio + 16, &v, 8);
    v = 3000000; memcpy(tio + 56, &v, 8); memcpy(tio + 64, &v, 8);
    uint8_t sent[72]; memcpy(sent, tio, 72);
    len = 72;
    CHECK(bts_ctl_sockopt(&t, 0x80487414, true, tio, &len, 0, &flush) == BTS_CTL_OPT_OK && t.speed == 3000000);
    uint8_t back[72] = {}; len = sizeof back;
    CHECK(bts_ctl_sockopt(&t, 0x40487413, false, back, &len, 0, &flush) == BTS_CTL_OPT_OK && !memcmp(back, sent, 72));
    uint8_t t32[44]; len = 44;                               // 32-bit TIOCGETA also answered (the kext has both)
    CHECK(bts_ctl_sockopt(&t, 0x402c7413, false, t32, &len, 0, &flush) == BTS_CTL_OPT_OK && len == 44);
    // Length and direction checks.
    len = 44;
    CHECK(bts_ctl_sockopt(&t, 0x80487414, true, tio, &len, 0, &flush) == BTS_CTL_OPT_INVAL);
    len = 16;
    CHECK(bts_ctl_sockopt(&t, 0x40487413, false, back, &len, 0, &flush) == BTS_CTL_OPT_INVAL);
    len = 72;
    CHECK(bts_ctl_sockopt(&t, 0x80487414, false, back, &len, 0, &flush) == BTS_CTL_OPT_INVAL);   // get of a set
    CHECK(bts_ctl_sockopt(&t, 0x40487413, true, tio, &len, 0, &flush) == BTS_CTL_OPT_INVAL);     // set of a get
    CHECK(bts_ctl_sockopt(&t, 0x80487414, true, nullptr, &len, 0, &flush) == BTS_CTL_OPT_INVAL);
    // bluetoothd's Skywalk options on com.apple.uart.sk.bluetooth: no nexus here.
    uint8_t spec[12] = {0, 4, 0, 0, 0x3f, 0, 0, 0, 0x3f, 0, 0, 0}; len = 12;
    CHECK(bts_ctl_sockopt(&t, 0x800c5414, true, spec, &len, 0, &flush) == BTS_CTL_OPT_NOPROTO);   // SETCHANNELSPEC
    uint8_t uuid[16]; len = 16;
    CHECK(bts_ctl_sockopt(&t, 0x40105412, false, uuid, &len, 0, &flush) == BTS_CTL_OPT_NOPROTO);  // GETCHANNELUUID
    len = 4; uint8_t big[4] = {};
    CHECK(bts_ctl_sockopt(&t, 0xc0ff7400, false, big, &len, 0, &flush) == BTS_CTL_OPT_NOPROTO);   // 0xff bytes
    // Void and small options.
    len = 0;
    CHECK(bts_ctl_sockopt(&t, 0x2000745e, true, nullptr, &len, 0, &flush) == BTS_CTL_OPT_OK);     // TIOCDRAIN
    uint8_t w[4] = {1, 0, 0, 0}; len = 4;
    CHECK(bts_ctl_sockopt(&t, 0x80047410, true, w, &len, 0, &flush) == BTS_CTL_OPT_FLUSH && flush == 1);
    uint8_t q[4] = {}; len = 4;
    CHECK(bts_ctl_sockopt(&t, 0x4004667f, false, q, &len, 9, &flush) == BTS_CTL_OPT_OK && q[0] == 9);   // FIONREAD
    uint8_t m[4] = {}; len = 4;
    CHECK(bts_ctl_sockopt(&t, 0x4004746a, false, m, &len, 0, &flush) == BTS_CTL_OPT_OK && (m[0] & 0x20));  // CTS

    // Framing: H4 written in mbuf-sized pieces reaches the controller as whole packets.
    static bts_shim s;
    Link l = {};
    bts_link link = {&l, linkTx, linkAddr, nullptr, nullptr};
    bts_shim_init(&s, &link, nullptr);
    Bytes reset = pktCmd(0x0c03, {}), acl = pktAcl(7, 300), bdaddr = pktCmd(0xfc01, {1, 2, 3, 4, 5, 6});
    Bytes host = reset;
    host.insert(host.end(), acl.begin(), acl.end());
    host.insert(host.end(), bdaddr.begin(), bdaddr.end());
    const size_t cuts[] = {1, 2, 5, 256, 3, 1000};
    size_t at = 0;
    for (size_t i = 0; at < host.size(); i++) {
        const size_t n = cuts[i % 6] < host.size() - at ? cuts[i % 6] : host.size() - at;
        bts_shim_host_write(&s, host.data() + at, n);
        at += n;
    }
    CHECK(l.sent.size() == 2 && l.sent[0] == reset && l.sent[1] == acl);
    // The emulated "bcm -A" reply, then controller events, leave in pieces no bigger than the room the socket has.
    Bytes want = cc(0xfc01, {0});
    Bytes ev1 = cc(0x0c03, {0}), ev2 = pktEvt(0x13, {1, 7, 0, 1, 0});
    Bytes ctrl = ev1;
    ctrl.insert(ctrl.end(), ev2.begin(), ev2.end());
    bts_shim_ctrl_input(&s, ctrl.data(), 3);
    bts_shim_ctrl_input(&s, ctrl.data() + 3, ctrl.size() - 3);
    want.insert(want.end(), ctrl.begin(), ctrl.end());
    Bytes sock;
    const size_t room[] = {0, 1, 3, 0, 2, 7, 64};
    for (size_t i = 0; bts_shim_readable(&s) && i < 100; i++) {
        uint8_t buf[16];
        const size_t space = room[i % 7];
        const size_t n = bts_ctl_take(&s, buf, sizeof buf, space);
        CHECK(n <= space && n <= sizeof buf);
        sock.insert(sock.end(), buf, buf + n);
    }
    CHECK(sock == want);
    // The client (BlueTool uart_get_hci_event) reads type, 2-byte header, body: the same packets come back.
    size_t p = 0;
    unsigned events = 0;
    while (p + 3 <= sock.size() && sock[p] == BTS_H4_EVT) { p += 3 + sock[p + 2]; events++; }
    CHECK(p == sock.size() && events == 3);
    // Last command and last reply: "bcm -A" (0xfc01) was the last command, the last reply is the controller's 0x0c03.
    CHECK(s.lastCmd == 0xfc01 && s.lastReply == 0x0c03 && s.lastReplyStatus == 0);
    Bytes st = {BTS_H4_EVT, BTS_EVT_CMD_STATUS, 4, 0x0c, 1, 0x05, 0x04};   // Command Status 0x0c for Create Connection
    bts_shim_ctrl_input(&s, st.data(), st.size());
    CHECK(s.lastReply == 0x0405 && s.lastReplyStatus == 0x0c);

    // Slot: bluetoothd's sk control takes over from BlueTool's control (RAM test 6), nothing else does.
    CHECK(bts_ctl_admit(false, false, 0, 0) == BTS_CTL_ACCEPT);
    CHECK(bts_ctl_admit(true, true, 0, 1) == BTS_CTL_PREEMPT);
    CHECK(bts_ctl_admit(true, true, 1, 0) == BTS_CTL_REFUSE);
    CHECK(bts_ctl_admit(true, true, 1, 1) == BTS_CTL_REFUSE);
    CHECK(bts_ctl_admit(true, true, 0, 0) == BTS_CTL_REFUSE);
    CHECK(bts_ctl_admit(true, false, 0, 1) == BTS_CTL_REFUSE);   // the tty holds it

    // RAM test 7: the WCN3950 mask (octet 18 = 0x83) lacks Read/Write Default Erroneous Data Reporting; the shim
    // advertises them and answers them, everything else in the mask is untouched.
    static const uint8_t wcn[64] = {0xff, 0xff, 0xff, 0x03, 0xce, 0xff, 0xef, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xf2, 0x0f,
                                    0xe8, 0xfe, 0x3f, 0xf7, 0x83, 0xff, 0x1c, 0x00, 0x04, 0x00, 0x61, 0xff, 0xff, 0xff,
                                    0x7f, 0x3e, 0x20, 0x00, 0xfe, 0xf0, 0xff, 0x0f, 0x00, 0x00, 0x80, 0x04, 0x00, 0x08};
    Bytes rc = cc(BTS_OP_READ_LOCAL_COMMANDS, Bytes(1, 0));
    rc.insert(rc.end(), wcn, wcn + 64);
    rc[2] = (uint8_t)(rc.size() - 3);
    bts_shim_flush_rx(&s);
    bts_shim_ctrl_input(&s, rc.data(), rc.size());
    Bytes mask = drain(&s);
    CHECK(mask.size() == rc.size() && mask[7 + 18] == (0x83 | 0x0c));
    mask[7 + 18] = 0x83;
    CHECK(mask == rc);
    const size_t sentBefore = l.sent.size();
    Bytes wr = pktCmd(0x0c5b, {0}), rd = pktCmd(0x0c5a, {});
    bts_shim_host_write(&s, wr.data(), wr.size());
    bts_shim_host_write(&s, rd.data(), rd.size());
    Bytes both = cc(0x0c5b, {0}), r2 = cc(0x0c5a, {0, 0});
    both.insert(both.end(), r2.begin(), r2.end());
    CHECK(drain(&s) == both && l.sent.size() == sentBefore);
    CHECK(s.lastCmd == 0x0c5a && s.lastReply == 0x0c5a && s.lastReplyStatus == 0);

    // RAM test 8: Apple's multiplexed VSC 0xfce2 gets status 0 with its sub-opcode echoed, never reaches the chip;
    // other unknown vendor commands still get 0x01 and land in the ring the disconnect line prints.
    const uint32_t unk = s.st.unknownVendor;
    Bytes oly = pktCmd(0xfce2, {0x2b, 1, 2, 3}), other = pktCmd(0xfca0, {0x07});
    bts_shim_host_write(&s, oly.data(), oly.size());
    bts_shim_host_write(&s, other.data(), other.size());
    Bytes ans = cc(0xfce2, {0, 0x2b}), ans2 = cc(0xfca0, {BTS_STATUS_UNKNOWN_COMMAND});
    ans.insert(ans.end(), ans2.begin(), ans2.end());
    CHECK(drain(&s) == ans && l.sent.size() == sentBefore);
    CHECK(s.st.unknownVendor == unk + 1 && s.unknownRing[unk % BTS_UNKNOWN_RING] == (0xfca0u << 8 | 0x07));
}

int main() {
    testCtl();
    testReassembly();
    testEncoders();
    testFifo();
    testShim();
    testRewrite();
    testTty();
    testRandom();
    testEndToEnd();
    testReview();
    printf("bt_shim_host: %u checks, %u failed\n", gChecks, gFail);
    return gFail ? 1 : 0;
}
