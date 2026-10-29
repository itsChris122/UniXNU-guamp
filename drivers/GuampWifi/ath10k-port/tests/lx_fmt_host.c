/* lx_vsnprintf (shim/src/lx_rt.c), the formatter behind lx_log and printk: the C conversions and the printk pointer
 * extensions, %pV first (every ath10k_info/warn/err line). Built like wpa2_station_host.c, against lx_rt.c alone, from ath10k-port:
 *   clang -std=gnu11 -O1 -nostdinc -isystem vendor/shim/libc -isystem $(clang -print-resource-dir)/include -DCONFIG_ATH10K=1
 *     -include gw_ath10k_port.h -include tests/host_libc.h -I vendor/shim/include -I vendor/include -I vendor/out/pinned
 *     -I vendor/out/ath10k-src tests/lx_fmt_host.c vendor/shim/src/lx_rt.c -Wl,--unresolved-symbols=ignore-all */
#include "lx_os.h"
#include "lx_plat.h"
int printf(const char *, ...);
int strcmp(const char *, const char *);
static int fails;
static void check(const char *want, const char *fmt, ...) {
  char b[256]; va_list ap; va_start(ap, fmt); lx_vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
  if (strcmp(b, want)) { fails++; printf("FAIL [%s]: got [%s] want [%s]\n", fmt, b, want); }
}
static void viaf(const char *want, const char *fmt, ...) {
  struct va_format vaf = { .fmt = fmt }; va_list args; va_start(args, fmt); vaf.va = &args;
  check(want, "%pV", &vaf); va_end(args);
}
static int tr(char *b, size_t n, const char *fmt, ...) { va_list ap; va_start(ap, fmt); int r = lx_vsnprintf(b, n, fmt, ap); va_end(ap); return r; }
int main(void) {
  u8 mac[6] = { 0x00, 0x11, 0x22, 0xaa, 0xbb, 0xcc }; dma_addr_t d = 0x8a001e000ull; u8 ip[4] = { 192, 168, 1, 7 };
  viaf("qca wcn3990 hw1.0 target 0x00000008 chip_id 0x00000120 sub 0000:0000", "%s target 0x%08x chip_id 0x%08x sub %04x:%04x", "qca wcn3990 hw1.0", 8, 0x120, 0, 0);
  viaf("wmi command 1 timeout, restarting hardware\n", "wmi command %d timeout, restarting hardware\n", 1);
  viaf("could not send WMI init command (-11)\n", "could not send WMI init command (%d)\n", -11);
  viaf("peer 00:11:22:aa:bb:cc vdev 3", "peer %pM vdev %d", mac, 3);
  check("00-11-22-aa-bb-cc|cc:bb:aa:22:11:00|001122aabbcc", "%pMF|%pMR|%pm", mac, mac, mac);
  check("paddr 0x00000008a001e000 len 5", "paddr %pad len %d", &d, 5);
  check("00 11 22|00:11|0011", "%*ph|%*phC|%*phN", 3, mac, 2, mac, 2, mac);
  check("192.168.1.7", "%pI4", ip);
  check("(null)", "%pM", (void *)0);
  check("[  ab][ab  ][00ab][-0005][ 7][18446744073709551615][-1][65535][-1][x]", "[%4s][%-4s][%04x][%05d][%*d][%llu][%ld][%hu][%hhd][%c]", "ab", "ab", 0xab, -5, 2, 7, ~0ull, -1L, 0xffff, 0xff, 'x');
  check("[abc][(null)][100%][  -3][0x1f][   5]", "[%.3s][%s][%d%%][%4zd][%#x][%*u]", "abcdef", (char *)0, 100, (long)-3, 0x1f, -4 + 8, 5u);
  check("12345678901 -9 4294967295", "%lld %zd %u", 12345678901ll, (long)-9, 0xffffffffu);
  char b[8]; int n = tr(b, sizeof b, "%s-%d", "abcdefgh", 42);
  if (n != 11 || strcmp(b, "abcdefg")) { fails++; printf("FAIL truncation: %d [%s]\n", n, b); }
  n = tr(b, sizeof b, "%pV", (void *)0); if (strcmp(b, "(null)")) { fails++; printf("FAIL null pV [%s]\n", b); }
  printf(fails ? "%d failed\n" : "fmt: all passed\n", fails);
  return fails != 0;
}
