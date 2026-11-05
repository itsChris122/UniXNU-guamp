#!/bin/bash
# Host test of the guamp-oslog line filters (guamp-oslog-skip/-only, dedup) and of the AMFI bit: the header is taken
# from patches/xnu/zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-oslog-tee.patch itself, so the test checks what gets built.
set -euo pipefail
here=$(cd "$(dirname "$0")/.." && pwd)
patch="$here/patches/xnu/zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz-guamp-oslog-tee.patch"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

# the new file's lines of the patch (after "+++ b/bsd/kern/guamp_oslog_filter.h", up to the next file)
awk '/^\+\+\+ b\/bsd\/kern\/guamp_oslog_filter.h/ {on=1; next} on && /^(--- |diff )/ {on=0} on && /^\+/ {print substr($0, 2)}' \
    "$patch" > "$work/guamp_oslog_filter.h"
[ -s "$work/guamp_oslog_filter.h" ] || { echo "FAIL: guamp_oslog_filter.h not found in the patch"; exit 1; }
grep -q 'guamp_oslog_amfi_policy(p, uap->call, uap->arg, error)' "$patch" || { echo "FAIL: mac_base.c hook missing"; exit 1; }

cat > "$work/t.c" <<'EOF'
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "guamp_oslog_filter.h"
static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)
int main(void)
{
	const char *assert_line = "AssertMacros: ![WiFiUsageMonitor sharedInstance],  file: /Library/Caches/"
	    "com.apple.xbs/Sources/WiFiManager/WiFiPerfPerAssoc.m, line: 79, value: 0";
	const char *aj = "[corewifi] AUTO-JOIN: Found matching network (x)";

	/* skip / only */
	CHECK(guamp_oslog_keep(aj, "", ""));
	CHECK(!guamp_oslog_keep(assert_line, "WiFiPerfPerAssoc", ""));
	CHECK(!guamp_oslog_keep(assert_line, "nothing,,WiFiUsageMonitor", ""));
	CHECK(guamp_oslog_keep(aj, "WiFiPerfPerAssoc", ""));
	CHECK(guamp_oslog_keep(aj, "", "AUTO-JOIN,candidate"));
	CHECK(!guamp_oslog_keep(assert_line, "", "AUTO-JOIN,candidate"));
	CHECK(guamp_oslog_keep("Known network matched using=x", "", "AUTO-JOIN,matched"));
	CHECK(!guamp_oslog_keep(aj, "AUTO", "AUTO-JOIN"));      /* skip wins */
	CHECK(guamp_oslog_has("abc", "c"));
	CHECK(!guamp_oslog_has("abc", "abcd"));
	CHECK(!guamp_oslog_has("", "a"));

	/* dedup: first kept, the next 999 dropped, count printed at 10, 100, 999 not */
	struct guamp_oslog_ring r;
	memset(&r, 0, sizeof(r));
	unsigned rep, printed = 0, kept = 0, last = 0;
	for (int i = 0; i < 1000; i++) {
		if (guamp_oslog_ring_note(&r, 42, assert_line, &rep)) {
			kept++;
		}
		if (rep) {
			printed++;
			last = rep;
		}
	}
	CHECK(kept == 1);
	CHECK(printed == 2 && last == 100);
	rep = 0;
	(void)guamp_oslog_ring_note(&r, 42, assert_line, &rep);         /* 1000th repeat */
	CHECK(rep == 1000);
	/* same text from another process is new; other lines interleaved still keep the repeated one out */
	CHECK(guamp_oslog_ring_note(&r, 43, assert_line, &rep));
	CHECK(guamp_oslog_ring_note(&r, 42, aj, &rep));
	CHECK(!guamp_oslog_ring_note(&r, 42, assert_line, &rep));
	/* after 16 other distinct lines the old one leaves the ring and is printed again */
	char buf[64];
	for (int i = 0; i < 16; i++) {
		snprintf(buf, sizeof(buf), "line %d", i);
		CHECK(guamp_oslog_ring_note(&r, 42, buf, &rep));
	}
	CHECK(guamp_oslog_ring_note(&r, 42, assert_line, &rep));
	/* lines that differ only after 400 characters count as the same */
	char a[600], b[600];
	memset(a, 'x', 500); a[500] = 0; memcpy(b, a, sizeof(a)); b[450] = 'y';
	CHECK(guamp_oslog_ring_note(&r, 7, a, &rep));
	CHECK(!guamp_oslog_ring_note(&r, 7, b, &rep));

	/* the AMFI bit libsystem_trace tests: byte 1, bit 0 of the 64-bit flags */
	uint64_t flags = GUAMP_AMFI_DEV_ENV_VARS;
	CHECK((((const uint8_t *)&flags)[1] & 1) == 1);

	printf(fails ? "%d FAILED\n" : "all passed\n", fails);
	return fails != 0;
}
EOF
cc -std=c11 -Wall -Wextra -Werror -I"$work" -o "$work/t" "$work/t.c"
"$work/t"
