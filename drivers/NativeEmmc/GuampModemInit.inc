// SPDX-License-Identifier: BSD-2-Clause-Patent
// Modem (MPSS) init: Linux-side prerequisites and diagnostics (29/10/2026). Included by GuampWifiPrep.inc inside
// namespace wifiprep after modemAwaitReady; hooked by GuampModem.inc (GUAMP_MODEM_INIT).
// Context: first authenticated start (wifi3, kernel 58540b0): AUTH_AND_RESET status 0, then no SMP2P 428 within 10 s
// and item 421 = "SFR Init: wdog or kernel error suspected.", the placeholder the modem writes at its err init: the
// modem died (watchdog bite or QuRT kernel error) or hung after err init and before its SMP2P init.
// prepare (after the cold-boot check, before any modem RAM write):
//   unit info  SMEM item 134 (SMEM_ID_VENDOR0, 1024 bytes, global partition, QCOM_SMEM_HOST_ANY) = struct
//              mmi_unit_info v3 that Motorola's drivers/mmi_info/mmi_unit_info.c writes "for modem" at module init,
//              long before Android starts the modem (TWRP dmesg: "mmi_unit_info (SMEM) for modem: version = 0x03,
//              device = 'guamp' ..."). The bootloader writes 135/136 (VENDOR1/2), the kernel 134; XNU never did. It is
//              the only Linux-side SMEM write before the modem start missing here. guamp-modem-unitinfo=0 skips it.
//              Strings (PE_parse_boot_argn: a value starting with a digit is read as a number, not a string):
//              guamp-mmi-barcode=<androidboot.serialno> (default empty: not committed), guamp-mmi-carrier=<s> (timit),
//              guamp-mmi-radio=<s> (ROW), guamp-mmi-pu=<n> powerup reason (0x4000, as in the TWRP boot).
//   mx         guamp-modem-mx=<level>: rwmx swen 1 + vlvl, both sets, floor held. Linux does not vote MX for qcom,mss
//              (only vdd_cx): experiment, default off.
//   baseline   SMEM items of the global and apps-modem partitions and the MSS watchdog interrupt state.
// watch (thread from AUTH_AND_RESET, 15 s): every 10 ms GIC SPI 307 (qcom,wdog of qcom,mss = INTID 339, rising edge:
//   the distributor latches the pending bit even while the line is disabled); every 50 ms SMEM: new items, item 421
//   text, item 428 slave-kernel bits. One line per change with ms since AUTH_AND_RESET, then a summary.
// report (failure path, before PAS SHUTDOWN): item 421 dump (text + hex of the used bytes), wdog latch, TCSR mutex 3,
//   RPM rail floors.
// Read-only except item 134 (allocated under the shared SMEM lock, GuampSmemLock.inc) and the optional MX vote.
#ifndef GUAMP_MODEM_INIT_INC
#define GUAMP_MODEM_INIT_INC
namespace mi {
constexpr uint64_t kGicd = 0x0F200000, kGicdBytes = 0x10000, kTcsrMutex3 = 0x00343000;
constexpr uint32_t kWdogIntid = 32 + 307;
constexpr uint32_t kWord = 4 * (kWdogIntid / 32), kBit = 1u << (kWdogIntid % 32);
constexpr uint32_t kIsenabler = 0x100 + kWord, kIspendr = 0x200 + kWord, kIsactiver = 0x300 + kWord;
constexpr uint32_t kIcfgr = 0xc00 + 4 * (kWdogIntid / 16), kIcfgrEdge = 2u << (2 * (kWdogIntid % 16));
constexpr uint32_t kUnitItem = 134, kUnitBytes = 1024, kWatchMs = 15000;

// drivers/mmi_info/mmi_info.h, MMI_UNIT_INFO_VER 3 (natural alignment, as the kernel and the modem build it)
struct UnitInfo {
  uint32_t version, system_rev, system_serial_low, system_serial_high;
  char machine[33], barcode[65], carrier[65], baseband[97], device[33];
  uint32_t radio, powerup_reason;
  char radio_str[33];
};
static_assert(sizeof(UnitInfo) == 356 && __builtin_offsetof(UnitInfo, radio) == 312 && __builtin_offsetof(UnitInfo, radio_str) == 320,
              "struct mmi_unit_info layout");

struct Ids { uint32_t n; uint16_t id[smem::kMaxIds]; };
constinit Ids gBaseGlobal = {}, gBaseModem = {};
constinit bool gBaseValid = false, gWatchRunning = false;
constinit IOMemoryMap *gGicMap = nullptr;
constinit uint32_t gWdogBase = 0, gWdogAt = ~0u, g421At = ~0u, g428At = ~0u, gNew = 0, gLast428 = ~0u;
constinit uint64_t gAuthAt = 0;
constinit char gSfr[96] = {};

uint32_t msSinceAuth() {
  uint64_t ns = 0;
  absolutetime_to_nanoseconds(mach_absolute_time() - gAuthAt, &ns);
  return static_cast<uint32_t>(ns / 1000000);
}
IOMemoryMap *mapRange(uint64_t phys, uint64_t bytes, bool write) {
  if (pas::overlap(phys, bytes, guamp_phys_base, guamp_phys_size)) return nullptr;
  IOMemoryDescriptor *d = IOMemoryDescriptor::withPhysicalAddress(phys, bytes, write ? kIODirectionInOut : kIODirectionIn);
  IOMemoryMap *m = d ? d->map(write ? kIOMapInhibitCache : kIOMapInhibitCache | kIOMapReadOnly) : nullptr;
  if (d) d->release();
  return m;
}
uint32_t gic(uint32_t off) {
  if (!gGicMap) gGicMap = mapRange(kGicd, kGicdBytes, false);
  if (!gGicMap) return 0;
  return *reinterpret_cast<const volatile uint32_t *>(gGicMap->getVirtualAddress() + off);
}
bool wdogPending() { return gic(kIspendr) & kBit; }
void logWdog(const char *when) {
  const uint32_t p = gic(kIspendr), a = gic(kIsactiver), e = gic(kIsenabler), c = gic(kIcfgr);
  IOLog("GuampModem: wdog (GIC SPI 307, INTID %u) %s: pending %u active %u enabled %u %s%s\n", kWdogIntid, when,
        p & kBit ? 1 : 0, a & kBit ? 1 : 0, e & kBit ? 1 : 0, c & kIcfgrEdge ? "edge" : "level",
        gGicMap ? "" : " (GICD not mapped)");
}

// ids of the global and apps-modem partitions
void collect(const smem::Info &info, Ids *global, Ids *modem) {
  const smem::Partition *g = smem::partitionFor(info, smem::kHostGlobal), *m = smem::partitionFor(info, smem::kHostModem);
  global->n = g ? (g->items < smem::kMaxIds ? g->items : smem::kMaxIds) : 0;
  modem->n = m ? (m->items < smem::kMaxIds ? m->items : smem::kMaxIds) : 0;
  for (uint32_t i = 0; i < global->n; i++) global->id[i] = g->ids[i];
  for (uint32_t i = 0; i < modem->n; i++) modem->id[i] = m->ids[i];
}
bool has(const Ids &s, uint16_t id) {
  for (uint32_t i = 0; i < s.n; i++)
    if (s.id[i] == id) return true;
  return false;
}
void listIds(const char *what, const Ids &s) {
  char line[200];
  uint32_t n = 0;
  line[0] = 0;
  for (uint32_t i = 0; i < s.n && n + 8 < sizeof(line); i++) {
    uint32_t v = s.id[i];
    char tmp[8];
    int k = 0;
    do { tmp[k++] = static_cast<char>('0' + v % 10); v /= 10; } while (v && k < 7);
    line[n++] = ' ';
    while (k) line[n++] = tmp[--k];
    line[n] = 0;
  }
  IOLog("GuampModem: SMEM %s items (%u):%s\n", what, s.n, line);
}
// text of item 421 (printable, up to the first NUL), "" when absent
void readSfr(const smem::Reader &r, const smem::Info &info, char *out, uint32_t cap, uint32_t *bytes) {
  uint32_t at = 0, n = 0;
  out[0] = 0;
  *bytes = 0;
  if (info.error || !smem::find(r, info, smem::kHostModem, 421, &at, &n)) return;
  *bytes = n;
  uint32_t i = 0;
  for (; i < n && i + 1 < cap; i++) {
    const uint8_t c = r.r8(at + i);
    if (!c) break;
    out[i] = c >= 32 && c < 127 ? static_cast<char>(c) : '?';
  }
  out[i] = 0;
}

void writeUnitInfo(IOMemoryMap *map) {
  unsigned on = 1;
  PE_parse_boot_argn("guamp-modem-unitinfo", &on, sizeof(on));
  if (!on) {
    IOLog("GuampModem: unit info (SMEM 134) skipped (guamp-modem-unitinfo=0)\n");
    return;
  }
  UnitInfo *u = static_cast<UnitInfo *>(IOMallocZero(kUnitBytes));
  smem::Info *info = static_cast<smem::Info *>(IOMallocZero(sizeof(smem::Info)));
  if (!u || !info) {
    if (u) IOFree(u, kUnitBytes);
    if (info) IOFree(info, sizeof(*info));
    IOLog("GuampModem: unit info: out of memory\n");
    return;
  }
  u->version = 3;
  u->system_rev = 0xc000;   // /chosen linux,hwrev of this phone (TWRP dmesg)
  u->powerup_reason = 0x4000;
  PE_parse_boot_argn("guamp-mmi-pu", &u->powerup_reason, sizeof(u->powerup_reason));
  memcpy(u->device, "guamp", 6);
  memcpy(u->carrier, "timit", 6);
  memcpy(u->radio_str, "ROW", 4);
  PE_parse_boot_argn("guamp-mmi-carrier", u->carrier, sizeof(u->carrier) - 1);
  PE_parse_boot_argn("guamp-mmi-radio", u->radio_str, sizeof(u->radio_str) - 1);
  PE_parse_boot_argn("guamp-mmi-barcode", u->barcode, sizeof(u->barcode) - 1);
  guampwifi::SmemArena arena;
  arena.mem = {reinterpret_cast<void *>(map->getVirtualAddress()), static_cast<uint32_t>(kSmemBytes),
               [](void *c, uint32_t at) { return *reinterpret_cast<const volatile uint32_t *>(static_cast<uint8_t *>(c) + at); },
               [](void *c, uint32_t at, uint32_t v) { *reinterpret_cast<volatile uint32_t *>(static_cast<uint8_t *>(c) + at) = v; },
               [](void *) { guampModemBarrier(); }};
  arena.info = info;
  arena.lock = [](void *) { return guamp_smem_hwlock_acquire(); };
  arena.unlock = [](void *) { guamp_smem_hwlock_release(); };
  uint32_t at = 0, actual = 0;
  bool created = false;
  const int e = arena.allocate(smem::kHostGlobal, kUnitItem, kUnitBytes, &at, &created, &actual);
  if (!e && created) {
    arena.mem.copyIn(at, reinterpret_cast<const uint8_t *>(u), kUnitBytes);
    arena.mem.fence();
  }
  IOLog("GuampModem: unit info (SMEM 134, mmi_unit_info v3 for the modem): %s%s (%u bytes at %x): device '%s' radio '%s' "
        "carrier '%s' rev %x pu %x barcode %s\n",
        e ? "FAILED " : created ? "written" : "already present, left as is", e ? guampwifi::errorName(e) : "", actual, at,
        u->device, u->radio_str, u->carrier, u->system_rev, u->powerup_reason, u->barcode[0] ? "set" : "empty");
  IOFree(u, kUnitBytes);
  IOFree(info, sizeof(*info));
}
}  // namespace mi

static void modemInitPrepare() {
  using namespace mi;
  IOMemoryMap *map = mapRange(kSmemPhys, kSmemBytes, true);
  if (!map) {
    IOLog("GuampModem: init prepare: SMEM not mapped\n");
    return;
  }
  writeUnitInfo(map);
#if defined(GUAMP_RPM_CLIENT_INC)
  unsigned mx = 0;
  if (PE_parse_boot_argn("guamp-modem-mx", &mx, sizeof(mx)) && mx && mx <= 512) {
    const rpm_kvp kv[2] = {{RPM_KEY_SWEN, 4, 1}, {rpm::kKeyVlvl, 4, mx}};
    rpm::holdFloor(rpm::kRailMx, RPM_SET_ACTIVE, mx);
    rpm::holdFloor(rpm::kRailMx, RPM_SET_SLEEP, mx);
    const int a = rpm::request(RPM_SET_ACTIVE, rpm::kRailMx, 0, kv, 2, 500);
    const int s = a ? a : rpm::request(RPM_SET_SLEEP, rpm::kRailMx, 0, kv, 2, 500);
    IOLog("GuampModem: guamp-modem-mx: rwmx level %u %s (floor held)\n", mx, rpm_err_name(s));
  }
#endif
  smem::Info *info = static_cast<smem::Info *>(IOMallocZero(sizeof(smem::Info)));
  if (info) {
    smem::Reader reader = {smemRead, reinterpret_cast<void *>(map->getVirtualAddress()), static_cast<uint32_t>(kSmemBytes)};
    guampModemBarrier();
    smem::parse(reader, info);
    collect(*info, &gBaseGlobal, &gBaseModem);
    gBaseValid = !info->error;
    listIds("global before the modem start", gBaseGlobal);
    listIds("apps-modem before the modem start", gBaseModem);
    IOFree(info, sizeof(*info));
  }
  map->release();
  gWdogBase = wdogPending();
  logWdog("before the modem start");
}

static void modemWatchThread(void *, wait_result_t) {
  using namespace mi;
  thread_set_thread_name(current_thread(), "guamp-modem-watch");
  IOMemoryMap *map = mapRange(kSmemPhys, kSmemBytes, false);
  smem::Info *info = static_cast<smem::Info *>(IOMallocZero(sizeof(smem::Info)));
  Ids *now = static_cast<Ids *>(IOMallocZero(2 * sizeof(Ids)));
  if (!map || !info || !now) {
    IOLog("GuampModem: watch: setup failed\n");
  } else {
    smem::Reader reader = {smemRead, reinterpret_cast<void *>(map->getVirtualAddress()), static_cast<uint32_t>(kSmemBytes)};
    Ids seenGlobal = gBaseGlobal, seenModem = gBaseModem;
    char sfr[sizeof(gSfr)];
    for (uint32_t tick = 0;; tick++) {
      const uint32_t t = msSinceAuth();
      if (t >= kWatchMs) break;
      if (gWdogAt == ~0u && wdogPending() && !gWdogBase) {
        gWdogAt = t;
        IOLog("GuampModem: watch +%u ms: MSS WATCHDOG BITE latched (GIC SPI 307 pending)\n", t);
      }
      if (!(tick % 5)) {
        guampModemBarrier();
        smem::parse(reader, info);
        if (!info->error) {
          collect(*info, &now[0], &now[1]);
          for (int k = 0; k < 2; k++) {
            const Ids &cur = now[k];
            Ids &seen = k ? seenModem : seenGlobal;
            for (uint32_t i = 0; i < cur.n; i++) {
              if (has(seen, cur.id[i])) continue;
              if (seen.n < smem::kMaxIds) seen.id[seen.n++] = cur.id[i];
              gNew++;
              IOLog("GuampModem: watch +%u ms: new SMEM item %u (%s partition)\n", t, cur.id[i], k ? "apps-modem" : "global");
            }
          }
          uint32_t bytes = 0;
          readSfr(reader, *info, sfr, sizeof(sfr), &bytes);
          if (bytes && g421At == ~0u) g421At = t;
          if (bytes && strcmp(sfr, gSfr)) {
            memcpy(gSfr, sfr, sizeof(gSfr));
            IOLog("GuampModem: watch +%u ms: item 421 (%u bytes): \"%s\"\n", t, bytes, sfr);
          }
          uint32_t at = 0, n = 0;
          if (smem::find(reader, *info, smem::kHostModem, 428, &at, &n)) {
            if (g428At == ~0u) g428At = t;
            smem::Smp2p *st = static_cast<smem::Smp2p *>(IOMallocZero(sizeof(smem::Smp2p)));
            if (st) {
              smem::readSmp2p(reader, at, n, "slave-kernel", st);
              if (st->value != gLast428) {
                gLast428 = st->value;
                IOLog("GuampModem: watch +%u ms: smp2p 428 slave-kernel %x (fatal %u ready %u handover %u)\n", t, st->value,
                      st->value & 1, st->value >> 1 & 1, st->value >> 2 & 1);
              }
              IOFree(st, sizeof(*st));
            }
          }
        }
      }
      IOSleep(10);
    }
    IOLog("GuampModem: watch summary %u ms: wdog %s%u ms, item 421 at %d ms, smp2p 428 at %d ms, %u new SMEM items, "
          "last 421 \"%s\"\n", kWatchMs, gWdogAt == ~0u ? (gWdogBase ? "pending BEFORE start, " : "never latched, ") : "bite at ",
          gWdogAt == ~0u ? 0u : gWdogAt, g421At == ~0u ? -1 : static_cast<int>(g421At),
          g428At == ~0u ? -1 : static_cast<int>(g428At), gNew, gSfr);
  }
  if (now) IOFree(now, 2 * sizeof(Ids));
  if (info) IOFree(info, sizeof(*info));
  if (map) map->release();
  gWatchRunning = false;
}

static void modemWatchStart() {
  using namespace mi;
  if (gWatchRunning) return;
  gAuthAt = mach_absolute_time();
  gWdogAt = g421At = g428At = gLast428 = ~0u;
  gNew = 0;
  gSfr[0] = 0;
  gWatchRunning = true;
  thread_t t = nullptr;
  if (kernel_thread_start(modemWatchThread, nullptr, &t) == KERN_SUCCESS) thread_deallocate(t);
  else gWatchRunning = false;
}

// Failure path (subsys-pil-tz log_failure_reason, more of it): the whole item 421, the wdog latch, TCSR mutex 3 and
// the rails, before PAS SHUTDOWN.
static void modemInitReport() {
  using namespace mi;
  const uint32_t t = gAuthAt ? msSinceAuth() : 0;
  logWdog("at the failure");
  IOLog("GuampModem: failure at +%u ms after AUTH_AND_RESET: wdog latch %s, item 421 seen at %d ms, %u new SMEM items\n", t,
        gWdogAt != ~0u ? "SET by the modem" : gWdogBase ? "set before the start" : "never",
        g421At == ~0u ? -1 : static_cast<int>(g421At), gNew);
  if (IOMemoryMap *m = mapRange(kTcsrMutex3, 0x1000, false)) {
    IOLog("GuampModem: TCSR mutex 3 (SMEM hwlock) owner %u (0 free, 1 apps)\n",
          *reinterpret_cast<const volatile uint32_t *>(m->getVirtualAddress()));
    m->release();
  }
#if defined(GUAMP_RPM_CLIENT_INC)
  IOLog("GuampModem: rail floors held: rwcx act %u slp %u, rwmx act %u slp %u; %u lower votes raised\n",
        rpm::floorOf(rpm::kRailCx, RPM_SET_ACTIVE), rpm::floorOf(rpm::kRailCx, RPM_SET_SLEEP),
        rpm::floorOf(rpm::kRailMx, RPM_SET_ACTIVE), rpm::floorOf(rpm::kRailMx, RPM_SET_SLEEP), rpm::gRaised);
#endif
  IOMemoryMap *map = mapRange(kSmemPhys, kSmemBytes, false);
  smem::Info *info = static_cast<smem::Info *>(IOMallocZero(sizeof(smem::Info)));
  if (map && info) {
    smem::Reader reader = {smemRead, reinterpret_cast<void *>(map->getVirtualAddress()), static_cast<uint32_t>(kSmemBytes)};
    guampModemBarrier();
    smem::parse(reader, info);
    uint32_t at = 0, n = 0;
    if (!info->error && smem::find(reader, *info, smem::kHostModem, 421, &at, &n) && n) {
      uint32_t used = 0;
      for (uint32_t i = 0; i < n && i < 1024; i++)
        if (reader.r8(at + i)) used = i + 1;
      IOLog("GuampModem: item 421 dump: %u bytes, %u used\n", n, used);
      static const char hex[] = "0123456789abcdef";
      for (uint32_t row = 0; row < used; row += 32) {
        char line[32 * 3 + 1];
        char text[33];
        uint32_t k = 0;
        for (uint32_t i = 0; i < 32 && row + i < used; i++) {
          const uint8_t c = reader.r8(at + row + i);
          line[3 * i] = hex[c >> 4];
          line[3 * i + 1] = hex[c & 15];
          line[3 * i + 2] = ' ';
          text[i] = c >= 32 && c < 127 ? static_cast<char>(c) : '.';
          k = i + 1;
        }
        line[3 * k] = 0;
        text[k] = 0;
        IOLog("GuampModem: 421 +%03x: %s |%s|\n", row, line, text);
      }
    }
    Ids g = {}, m = {};
    if (!info->error) {
      collect(*info, &g, &m);
      listIds("global at the failure", g);
      listIds("apps-modem at the failure", m);
    }
  }
  if (info) IOFree(info, sizeof(*info));
  if (map) map->release();
}
#endif
