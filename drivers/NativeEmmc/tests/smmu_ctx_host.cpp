// Host test of GuampSmmu.inc SmmuContext (apps SMMU stage-1 context per client stream) against a register model:
// the ADSP audio instance (SID 0x1c1 mask 0, IOVA 0x10000000..0x10400000) and the WLAN instance (unchanged layout).
// Checks SMR/S2CR/CBAR/TTBR programming, slot and bank choice next to a live stream, refusal of an overlapping SMR,
// a page-table walk of every mapped page, window limits, unmap, stop() restoring the previous words, the global lock.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "../GuampSmmu.inc"

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); exit(1); } } while (0)
using namespace guampwifi;

struct Model {
  std::vector<uint32_t> regs = std::vector<uint32_t>(0x80000 / 4);
  std::vector<uint8_t> tables;
  uint64_t tablePhys = 0x90000000ull;
  unsigned syncs = 0;
  static uint32_t rd(void *c, uint32_t at) { return static_cast<Model *>(c)->regs[at / 4]; }
  static void wr(void *c, uint32_t at, uint32_t v) {
    Model *m = static_cast<Model *>(c);
    if ((at & 0xfff) == 0x7f0 && at >= 0x40000) m->syncs++;
    m->regs[at / 4] = v;
  }
  static uint32_t trd(void *c, uint32_t at) { uint32_t v; memcpy(&v, &static_cast<Model *>(c)->tables[at], 4); return v; }
  static void twr(void *c, uint32_t at, uint32_t v) { memcpy(&static_cast<Model *>(c)->tables[at], &v, 4); }
  static void fence(void *) {}
  static void delay(void *, uint32_t) {}
  Memory mmio() { return {this, 0x80000, rd, wr, fence}; }
  Memory tbl() { return {this, static_cast<uint32_t>(tables.size()), trd, twr, fence}; }
  void ids(uint32_t groups, uint32_t banks) {
    regs[0x20 / 4] = (1u << 30) | (1u << 27) | groups;
    regs[0x24 / 4] = banks;          // NUMS2CB 0, 4K pages
    regs[0x28 / 4] = (1u << 12) | (2u << 4);   // PTFS 4K, OAS 40 bit
  }
  uint64_t entry(uint64_t tablePa, uint32_t index) const {
    const uint64_t at = tablePa - tablePhys + index * 8ull;
    uint64_t v;
    memcpy(&v, &tables[at], 8);
    return v;
  }
  // Stage-1 walk with 4K granule, 32-bit VA (TTBCR T0SZ 32: walk starts at level 1)
  bool walk(uint64_t iova, uint64_t *pa) const {
    uint64_t e = entry(tablePhys, uint32_t(iova >> 30) & 3);
    if ((e & 3) != 3) return false;
    e = entry(e & 0xfffffffff000ull, uint32_t(iova >> 21) & 511);
    if ((e & 3) != 3) return false;
    e = entry(e & 0xfffffffff000ull, uint32_t(iova >> 12) & 511);
    if ((e & 3) != 3 || !(e & (1u << 10))) return false;
    *pa = (e & 0xfffffffff000ull) | (iova & 0xfff);
    return true;
  }
};

template <class Ctx> static int startOn(Model &m, Ctx &c) {
  m.tables.assign(Ctx::TableBytes, 0xa5);
  return c.start(m.mmio(), m.tbl(), m.tablePhys, 0x80000000ull, 0x40000000ull, true, &m, Model::delay);
}

int main() {
  static_assert(AdspAudioSmmu::TableBytes == 4 * 4096 && AdspAudioSmmu::L1Index == 0 && AdspAudioSmmu::L2First == 128);
  // 1. audio on a cold SMMU (no SMR anywhere: "unmatched streams fault"), next to WLAN already live in slot 0 / CB 0
  {
    Model m;
    m.ids(32, 16);
    m.regs[0x800 / 4] = (1u << 31) | (1u << 16) | 0x1a0;   // SMR0 WLAN
    m.regs[0xc00 / 4] = 0;                                 // S2CR0 translate, CB 0
    m.regs[0x40000 / 4] = 1;                               // CB0 SCTLR.M
    const uint32_t oldS2cr1 = 0x20000;                     // slot 1 was "fault"
    m.regs[0xc04 / 4] = oldS2cr1;
    AdspAudioSmmu a;
    CHECK(startOn(m, a) == Ok && a.active && !a.preexisting);
    CHECK(a.slot == 1 && a.cb == 1 && SmmuGlobalLock::word == 0);
    CHECK(m.regs[0x804 / 4] == ((1u << 31) | 0x1c1) && m.regs[0xc04 / 4] == 1);
    const uint32_t bank = 0x40000 + 4096;
    CHECK(m.regs[(4096 + 4) / 4] == ((1u << 16) | (3u << 8) | (15u << 12)));   // CBAR: S1 + S2 bypass
    CHECK(m.regs[(4096 + 0x800 + 4) / 4] == 1);                               // CBA2R AArch64
    CHECK(m.regs[(bank + 0x20) / 4] == uint32_t(m.tablePhys) && m.regs[(bank + 0x24) / 4] == (2u << 16));   // ASID 2
    CHECK((m.regs[bank / 4] & 1) && m.regs[(bank + 0x30) / 4] == (32u | (2u << 12) | (1u << 23)));
    // the tables: L1[0] -> L2, L2[128..129] -> L3, everything else zero
    CHECK((m.entry(m.tablePhys, 0) & 0xfffffffff003ull) == ((m.tablePhys + 4096) | 3));
    CHECK(m.entry(m.tablePhys, 2) == 0 && m.entry(m.tablePhys + 4096, 127) == 0 && m.entry(m.tablePhys + 4096, 130) == 0);
    CHECK((m.entry(m.tablePhys + 4096, 129) & ~3ull) == m.tablePhys + 3 * 4096);
    // map the playback buffer (32 KiB) and the capture buffer (64 KiB at +1 MiB)
    const uint64_t play = 0x91000000ull, cap = 0x91200000ull;
    const unsigned before = m.syncs;
    CHECK(a.map(0x10000000ull, play, 0x8000) == Ok && m.syncs == before + 1);
    CHECK(a.map(0x10100000ull, cap, 0x10000) == Ok);
    for (uint32_t off = 0; off < 0x8000; off += 0x800) {
      uint64_t pa = 0;
      CHECK(m.walk(0x10000000ull + off, &pa) && pa == play + off);
    }
    uint64_t pa = 0;
    CHECK(m.walk(0x1010fffcull, &pa) && pa == cap + 0xfffc);
    CHECK(!m.walk(0x10008000ull, &pa) && !m.walk(0x10110000ull, &pa));
    CHECK(a.map(0x10000000ull, play, 0x1000) == Busy);              // already mapped
    CHECK(a.map(0x10400000ull, play, 0x1000) == Range);             // outside the window
    CHECK(a.map(0x103ff000ull, play, 0x2000) == Range);             // crosses the end
    CHECK(a.map(0x10200000ull, 0x10000000ull, 0x1000) == Range);    // not XNU RAM
    CHECK(a.map(0x10200000ull, m.tablePhys, 0x1000) == Invalid);    // the page tables themselves
    CHECK(a.unmap(0x10000000ull, 0x8000) == Ok && !m.walk(0x10000000ull, &pa) && m.walk(0x10100000ull, &pa));
    // fault registers readable, stop restores the slot
    m.regs[(bank + 0x58) / 4] = 0x2;
    m.regs[(bank + 0x60) / 4] = 0x10000040;
    CHECK(a.faultStatus() == 2 && a.faultAddressLow() == 0x10000040);
    a.clearFault();
    CHECK(m.regs[(bank + 0x58) / 4] == 0xffffffffu);
    CHECK(a.stop(true) == Ok && !a.active && m.regs[0x804 / 4] == 0 && m.regs[0xc04 / 4] == oldS2cr1);
    CHECK(m.regs[0x800 / 4] == ((1u << 31) | (1u << 16) | 0x1a0) && SmmuGlobalLock::word == 0);
  }
  // 2. an SMR that covers 0x1c1 with another mask (0x1c0/0xf): refused, nothing written
  {
    Model m;
    m.ids(8, 4);
    m.regs[0x800 / 4] = (1u << 31) | (0xfu << 16) | 0x1c0;
    m.regs[0xc00 / 4] = 1u << 16;
    AdspAudioSmmu a;
    CHECK(startOn(m, a) == Busy && !a.active && m.regs[0x804 / 4] == 0 && SmmuGlobalLock::word == 0);
  }
  // 3. the same exact SMR already valid (set up before XNU): reused, preexisting
  {
    Model m;
    m.ids(8, 4);
    m.regs[0x80c / 4] = (1u << 31) | 0x1c1;
    m.regs[0xc0c / 4] = 2u << 16;
    AdspAudioSmmu a;
    CHECK(startOn(m, a) == Ok && a.preexisting && a.slot == 3 && m.regs[0xc0c / 4] == a.cb);
  }
  // 4. WLAN instance: the layout of the original code (L1[2], L2[256..383], SMR 0x1a0 mask 1)
  {
    Model m;
    m.ids(8, 4);
    WlanSmmu w;
    CHECK(startOn(m, w) == Ok && m.regs[0x800 / 4] == ((1u << 31) | (1u << 16) | 0x1a0));
    CHECK((m.entry(m.tablePhys, 2) & ~3ull) == m.tablePhys + 4096 && m.entry(m.tablePhys, 0) == 0);
    CHECK((m.entry(m.tablePhys + 4096, 256) & ~3ull) == m.tablePhys + 2 * 4096);
    CHECK((m.entry(m.tablePhys + 4096, 383) & ~3ull) == m.tablePhys + 129 * 4096 && m.entry(m.tablePhys + 4096, 384) == 0);
    uint64_t pa = 0;
    CHECK(w.map(0xa0001000ull, 0x92000000ull, 0x1000) == Ok && m.walk(0xa0001010ull, &pa) && pa == 0x92000010ull);
  }
  printf("smmu_ctx_host: audio 0x1c1 context next to live WLAN, tables/walk/limits/unmap/fault/stop, overlap refused, "
         "preexisting reused, WLAN layout unchanged: PASS\n");
  return 0;
}
