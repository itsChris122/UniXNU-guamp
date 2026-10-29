// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Bluetooth defaults (28/10; guamp-bt=2 default since 29/10). With guamp-bt=2 (the full stack, kernel-bt7..bt11) the
// sub-options take their verified values: guamp-bt-ldo=3, guamp-bt-nvm-baud=1, guamp-bt-shim=1, guamp-bt-shim-bdaddr=1,
// guamp-bt-nub=1, guamp-bt-ctl=1 (and guamp-bt-tool=1 in patches/xnu/zzzzzzzzz-guamp-default-args.patch). An explicit key
// still wins (=0 turns one off); without guamp-bt=2 every default stays 0 as before. Included by BtService.inc,
// GuampBtSerial.inc and GuampBtNub.inc; the includer declares PE_parse_boot_argn.
#pragma once

// guamp-bt itself defaults to 2 since 29/10 (owner: the full stack is verified with the finale ADT); guamp-bt=0 turns Bluetooth
// off, guamp-bt=1 keeps the old power-only mode.
static inline unsigned guamp_bt_mode() {
  unsigned mode = 2;
  PE_parse_boot_argn("guamp-bt", &mode, sizeof(mode));
  return mode;
}

static inline unsigned guamp_bt_default(unsigned verified) {
  return guamp_bt_mode() == 2 ? verified : 0u;
}
