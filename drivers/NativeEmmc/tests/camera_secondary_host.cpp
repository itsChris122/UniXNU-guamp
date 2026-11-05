// Host test of the secondary cameras (GuampCameraSecondary.h, GuampCamMacroPolicy.h) and of their path through the
// sensor session of GuampCameraSensor.h: CSIPHY lane masks (combo half of the OV02B pair), which cells stream together,
// the macro mirror + flip after its mode list and the CFA it reports, the OV02B chip id read on page 0 after stream on,
// the facts for the iOS service and the macro switch policy. Reuses the simulator of camera_host.cpp (its main is
// renamed). Optional argument: a .gcam file (vendor tables, kept outside the repo) to run cells 1..3 with the real lists.
#define main camera_host_main
#include "camera_host.cpp"
#undef main
#include "../GuampCamMacroPolicy.h"

// ---- tables against kBoard / kPhy ----------------------------------------------------------------------------------
static void testTables() {
  for (unsigned c = 0; c < kSensors; ++c) {
    CHECK(kLanes[c].csiphy == kPhy[c].csiphy && kLanes[c].laneAssign == kPhy[c].laneAssign && kLanes[c].combo == kPhy[c].combo);
    CHECK(kLanes[c].csiphy == kBoard[c].csiphy);
    CHECK(kRole[c].cciMaster == kBoard[c].cci && kRole[c].mclk == kBoard[c].mclk);
    CHECK(kRole[c].use == c);                      // cell n has use n: the GuampCamIosPosition numbering
    CHECK(laneMaskOfCell(c) != 0);
  }
  CHECK(cellFor(kUseFront) == 1 && cellFor(kUseDepth) == 2 && cellFor(kUseMacro) == 3);
  CHECK(kRole[1].facing == kFacingFront && kRole[3].facing == kFacingBack && kRole[2].ios == kIosNone);
  // every back camera ends in the main camera orientation
  for (unsigned c = 0; c < kSensors; ++c)
    if (kRole[c].facing == kFacingBack) CHECK(kRole[c].uprightRollDeg == kRole[0].rollDeg);
  // the macro frames like the main camera (within 5 degrees), so the back stream can switch to it
  const int d = static_cast<int>(kRole[3].hfovCdeg) - static_cast<int>(kRole[0].hfovCdeg);
  CHECK(d > -500 && d < 500);
}

static void testLaneMasks() {
  CHECK(laneMaskOfCell(0) == (kLaneDl0 | kLaneDl1 | kLaneDl2 | kLaneDl3 | kLaneClk));   // 0x1f
  CHECK(laneMaskOfCell(1) == 0x1f);
  CHECK(laneMaskOfCell(2) == (kLaneDl2 | kLaneDl3) && laneMaskOfCell(2) == 0x18);       // combo half: DL3 is its clock
  CHECK(laneMaskOfCell(3) == (kLaneDl0 | kLaneClk) && laneMaskOfCell(3) == 0x03);
  // the generic CAMSS form would give CLK + DL2 for the depth sensor: different, hence the explicit mask in the bridge
  CHECK(laneMaskOfCell(2) != (kLaneClk | kLaneDl2));
  CHECK(laneMaskOf(0, 0, false) == 0 && laneMaskOf(5, 0x3210, false) == 0);
  CHECK(laneMaskOf(2, 0x0010, true) == 0);       // a 2-lane sensor cannot sit on the combo half
  CHECK(laneMaskOf(1, 0x0000, true) == 0);       // combo half data lane is DL2
  CHECK(laneMaskOf(2, 0x0000, false) == 0);      // the same physical lane twice
  CHECK(laneMaskOf(2, 0x0010, false) == (kLaneDl0 | kLaneDl1 | kLaneClk));
  CHECK(laneMaskOf(1, 0x0004, false) == 0);
  CHECK(laneBit(3) == kLaneDl3 && laneBit(0) == kLaneDl0 && laneBit(4) == 0);
}

static void testTogether() {
  CHECK(together(2, 3) == kTogetherCombo && together(3, 2) == kTogetherCombo);
  CHECK((laneMaskOfCell(2) | laneMaskOfCell(3)) == 0x1b);
  CHECK(together(0, 1) == kTogetherYes && together(0, 3) == kTogetherYes && together(1, 2) == kTogetherYes && together(0, 2) == kTogetherYes);
  CHECK(together(1, 1) == kTogetherNo && together(4, 0) == kTogetherNo);
}

static void testPost() {
  Reg r[kPostRegsMax];
  uint8_t cfa = 0;                                                         // BGGR
  CHECK(postModeRegs(3, kPostDefault, r, &cfa) == 3 && cfa == 3);          // RGGB after mirror + flip
  CHECK(r[0].addr == 0xfd && r[0].data == 1 && r[1].addr == 0x12 && r[1].data == 3 && r[2].addr == 0xfe && r[2].data == 2);
  CHECK(r[0].addrBytes == 1 && r[0].dataBytes == 1);
  cfa = 0;
  CHECK(postModeRegs(3, postFlagsFromArg(2), r, &cfa) == 3 && cfa == 0);   // flip, CFA kept
  CHECK(postModeRegs(3, postFlagsFromArg(0), r, &cfa) == 0);
  CHECK(postFlagsFromArg(1) == kPostDefault && postFlagsFromArg(7) == kPostDefault);
  for (unsigned c = 0; c < 3; ++c) CHECK(postModeRegs(c, kPostDefault, r, &cfa) == 0);   // depth: the vendor list flips it
  CHECK(cfaRotated180(0) == 3 && cfaRotated180(1) == 2 && cfaRotated180(2) == 1 && cfaRotated180(3) == 0 && cfaRotated180(4) == 4);
}

static void testIosFacts() {
  const IosFacts f = iosFacts(1), m = iosFacts(3), z = iosFacts(2), x = iosFacts(9);
  CHECK(f.facing == kFacingFront && f.fullW == 3264 && f.previewW == 1632 && f.chipId == 0x487b && !f.hasAf && !f.hasFlash);
  CHECK(f.focalUm == 2670 && f.fNumberQ8 == 563 && f.pixelNm == 1120 && f.maxGainX16 == 256);
  CHECK(m.use == kUseMacro && m.hasAf && m.hasFlash && m.fullW == 1600 && m.maxGainX16 == 248 && m.minFocusMm && m.chipId == 0x2b);
  CHECK(z.mono && !z.hasAf);
  CHECK(x.fullW == 0);
}

// ---- synthetic OV02B table + session ----------------------------------------------------------------------------
static void synOv(std::vector<uint8_t> &b, const char *name, const Sensor &s, uint8_t cfa, bool vendorFlip) {
  char n[16] = {};
  strncpy(n, name, 15);
  b.insert(b.end(), n, n + 16);
  b.insert(b.end(), {s.slave7, s.addrBytes, 1, 1});
  put16(b, s.idReg); put16(b, s.id);
  const std::vector<std::vector<uint16_t>> init = {{0xfd, 0}, {0xfc, 1, 5000}, {0xfd, 1}, {0x0e, 2}, {0x0f, 0x1a}, {0xfe, 2}, {0xfd, 0}};
  const std::vector<std::vector<uint16_t>> on = {{0xfd, 1}, {0x10, 1}, {0xfd, 3}, {0xc2, 1}, {0xfb, 1}};
  const std::vector<std::vector<uint16_t>> off = {{0xfd, 1}, {0x10, 1}, {0xfd, 3}, {0xc2, 0}, {0xfb, 0}};
  std::vector<std::vector<uint16_t>> mode = {{0xfd, 0}, {0x50, 0x40}, {0x52, 0xb0}, {0xfd, 1}, {0x03, 0x70}, {0x05, 0x10}};
  if (vendorFlip) mode.push_back({0x12, 3});
  put16(b, 1); put16(b, static_cast<uint16_t>(init.size())); put16(b, static_cast<uint16_t>(on.size())); put16(b, static_cast<uint16_t>(off.size()));
  put16(b, 64); put16(b, 1600); put16(b, 1200);
  b.insert(b.end(), 10, 0);
  const size_t at = b.size();
  put16(b, 1600); put16(b, 1200); put16(b, 0); put16(b, 0);
  b.insert(b.end(), {1, 0x2b, 0, cfa});
  put32(b, 66000000); put32(b, 448); put32(b, 1221); put32(b, 30070);
  b.insert(b.end(), {1, 1, 14, 10});
  put16(b, 0); put16(b, 0); b.push_back(0); b.push_back(0); put16(b, static_cast<uint16_t>(mode.size()));
  b.insert(b.end(), 48 - (b.size() - at), 0);
  b.insert(b.end(), {8, 0, 10, 0, 1, 0, 0, 0});
  auto list = [&](const std::vector<std::vector<uint16_t>> &L) {
    for (const auto &r : L) putReg(b, r[0], r[1], 1, 1, r.size() > 2 ? r[2] : 0);
  };
  list(init); list(on); list(off); list(mode);
}
static std::vector<uint8_t> synOvGcam() {
  std::vector<uint8_t> b = {'G', 'C', 'A', 'M', 2, 0, 0, 0, 2, 0, 0, 0};
  synOv(b, "mot_ov02b1b", kBoard[2], 4, true);
  synOv(b, "mot_ov02b10", kBoard[3], 0, false);
  return b;
}

static uint8_t ovReg(unsigned cell, unsigned page, unsigned reg) { return S->regs[cell][page << 8 | reg]; }

// One OV02B session: configure, the flip state and CFA, status id on page 0 after stream on, stream off, close.
static void ovSession(const uint8_t *blob, size_t len, unsigned cell, uint8_t post, uint8_t wantFlip, uint8_t wantCfa, bool verbose) {
  Sim *sim = new Sim; S = sim;
  LdoState st = {};
  Cci cci = {&kOps};
  CHECK(cci.init(kCciFast) == 0 && pmChip(kOps, true) == 0);
  Gcam g;
  CHECK(gcamFind(blob, len, kBoard[cell].name, &g) == 0);
  Session ss;
  ss.post = post;
  CHECK(ss.open(kOps, st, cci, kCciFast, kBoard[cell], &g) == 0);
  CHECK(ss.configure(kResLow, -1, {600, 512, 0}) == 0 && ss.state == kConfigured);
  CHECK((ovReg(cell, 1, 0x12) & 3) == wantFlip);
  CHECK(ss.mode.cfa == wantCfa && mipiOf(kBoard[cell], ss.mode).cfa == wantCfa);
  const Mipi mi = mipiOf(kBoard[cell], ss.mode);
  CHECK(mi.csiphy == 1 && mi.lanes == 1 && mi.combo == kLanes[cell].combo && mi.width == 1600 && mi.height == 1200 && mi.laneMbps == 660);
  Status a;
  CHECK(ss.status(&a) == 0 && a.ok && a.id == 0x2b && a.lines == ss.exp.lines && a.fll == ss.exp.fll);
  CHECK(ss.streamOn() == 0 && streaming(kBoard[cell]));
  CHECK(S->regs[cell][0xfd] == 3);                                         // stream on ends on page 3
  CHECK(ss.status(&a) == 0 && a.ok && a.id == 0x2b);                       // id still read right (page 0 first)
  CHECK(ss.setExposure({1000, 1024, 0}) == 0 && ss.status(&a) == 0 && a.lines == ss.exp.lines && a.gainCode == ss.exp.code);
  CHECK((ovReg(cell, 1, 0x12) & 3) == wantFlip);                           // nothing undid the flip
  CHECK(ss.streamOff() == 0 && !streaming(kBoard[cell]) && ss.close() == 0 && ldoUsers(st) == 0);
  if (verbose)
    printf("  session %-8s post %u: flip %u, cfa %u, %ux%u %u lane %u Mbit/s, lane mask 0x%02x, exposure %u lines fll %u gain 0x%x\n",
           kBoard[cell].name, post, wantFlip, ss.mode.cfa, mi.width, mi.height, mi.lanes, mi.laneMbps, laneMaskOfCell(cell),
           ss.exp.lines, ss.exp.fll, ss.exp.code);
  delete sim;
}

static void testOvSessions() {
  const std::vector<uint8_t> b = synOvGcam();
  ovSession(b.data(), b.size(), 3, kPostDefault, 3, 3, false);             // macro: flipped here, RGGB
  ovSession(b.data(), b.size(), 3, postFlagsFromArg(2), 3, 0, false);      // flipped, CFA kept BGGR
  ovSession(b.data(), b.size(), 3, 0, 0, 0, false);                        // vendor orientation
  ovSession(b.data(), b.size(), 2, kPostDefault, 3, 4, false);             // depth: the vendor list flips it, Y
}

// ---- macro policy -----------------------------------------------------------------------------------------------------
static void testMacroPolicy() {
  GuampMacroSwitch s;
  guamp_macro_init(&s, kGuampMacroAuto);
  CHECK(!s.onMacro);
  GuampMacroInput nearIn = {1000, 100, 1023, kGuampAfSearching, 512};     // main lens at its near end, not in focus
  for (unsigned i = 0; i + 1 < GUAMP_MACRO_ENTER_FRAMES; ++i) CHECK(!guamp_macro_update(&s, &nearIn));
  CHECK(guamp_macro_update(&s, &nearIn) && s.switches == 1);
  // dwell: no decision for GUAMP_MACRO_DWELL_FRAMES even with the subject gone
  GuampMacroInput farIn = {110, 100, 900, kGuampAfLocked, 512};
  for (unsigned i = 0; i < GUAMP_MACRO_DWELL_FRAMES; ++i) CHECK(guamp_macro_update(&s, &farIn));
  for (unsigned i = 0; i + 1 < GUAMP_MACRO_EXIT_FRAMES; ++i) CHECK(guamp_macro_update(&s, &farIn));
  CHECK(!guamp_macro_update(&s, &farIn) && s.switches == 2);
  // a focus lock at the near end (subject in focus on the main camera) never switches; neither does a dark scene
  guamp_macro_init(&s, kGuampMacroAuto);
  GuampMacroInput locked = nearIn;
  locked.afState = kGuampAfLocked;
  for (unsigned i = 0; i < 100; ++i) CHECK(!guamp_macro_update(&s, &locked));
  GuampMacroInput dark = nearIn;
  dark.gainQ8 = 10u << 8;
  for (unsigned i = 0; i < 100; ++i) CHECK(!guamp_macro_update(&s, &dark));
  // an interrupted run starts again
  for (unsigned i = 0; i + 1 < GUAMP_MACRO_ENTER_FRAMES; ++i) guamp_macro_update(&s, &nearIn);
  guamp_macro_update(&s, &locked);
  CHECK(!guamp_macro_update(&s, &nearIn));
  // on the macro: too dark goes back
  guamp_macro_init(&s, kGuampMacroAuto);
  for (unsigned i = 0; i < GUAMP_MACRO_ENTER_FRAMES; ++i) guamp_macro_update(&s, &nearIn);
  CHECK(s.onMacro);
  for (unsigned i = 0; i < GUAMP_MACRO_DWELL_FRAMES; ++i) guamp_macro_update(&s, &nearIn);
  GuampMacroInput macroDark = {500, 100, 900, kGuampAfLocked, 15u << 8};
  for (unsigned i = 0; i + 1 < GUAMP_MACRO_EXIT_FRAMES; ++i) CHECK(guamp_macro_update(&s, &macroDark));
  CHECK(!guamp_macro_update(&s, &macroDark));
  // fixed policies and a bad lens range
  guamp_macro_init(&s, kGuampMacroAlways);
  CHECK(s.onMacro && guamp_macro_update(&s, &farIn));
  guamp_macro_init(&s, kGuampMacroOff);
  for (unsigned i = 0; i < 100; ++i) CHECK(!guamp_macro_update(&s, &nearIn));
  guamp_macro_init(&s, 9);
  CHECK(s.policy == kGuampMacroAuto);
  GuampMacroInput bad = {1000, 900, 900, kGuampAfFailed, 256};
  for (unsigned i = 0; i < 100; ++i) CHECK(!guamp_macro_update(&s, &bad));
}

// ---- real tables --------------------------------------------------------------------------------------------------------
static void testSecondaryFile(const char *path) {
  FILE *f = fopen(path, "rb");
  if (!f) { printf("FAIL cannot open %s\n", path); ++fails; return; }
  std::vector<uint8_t> b;
  int c;
  while ((c = fgetc(f)) != EOF) b.push_back(static_cast<uint8_t>(c));
  fclose(f);
  Gcam g;
  CHECK(gcamFind(b.data(), b.size(), "ov02b10", &g) == 0 && g.modes[0].cfa == 0 && g.modes[0].lanes == 1);
  CHECK(gcamFind(b.data(), b.size(), "ov02b1b", &g) == 0 && g.modes[0].cfa == 4);
  ovSession(b.data(), b.size(), 3, kPostDefault, 3, 3, true);
  ovSession(b.data(), b.size(), 3, 0, 0, 0, true);
  ovSession(b.data(), b.size(), 2, kPostDefault, 3, 4, true);
  sessionRun(b.data(), b.size(), 1, kResLow, true);
  sessionRun(b.data(), b.size(), 1, kResFull, true);
}

int main(int argc, char **argv) {
  testTables();
  testLaneMasks();
  testTogether();
  testPost();
  testIosFacts();
  testOvSessions();
  testMacroPolicy();
  if (argc > 1) testSecondaryFile(argv[1]);
  printf("camera_secondary_host: %s\n", fails ? "FAIL" : "PASS");
  return fails != 0;
}
