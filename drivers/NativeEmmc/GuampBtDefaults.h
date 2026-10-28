// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Bluetooth defaults (28/10): guamp-bt itself stays opt-in. With guamp-bt=2 (the full stack, kernel-bt7..bt11) the
// sub-options take their verified values: guamp-bt-ldo=3, guamp-bt-nvm-baud=1, guamp-bt-shim=1, guamp-bt-shim-bdaddr=1,
// guamp-bt-nub=1, guamp-bt-ctl=1 (and guamp-bt-tool=1 in patches/xnu/zzzzzzzzz-guamp-default-args.patch). An explicit key
// still wins (=0 turns one off); without guamp-bt=2 every default stays 0 as before. Included by BtService.inc,
// GuampBtSerial.inc and GuampBtNub.inc; the includer declares PE_parse_boot_argn.
#pragma once

static inline unsigned guamp_bt_default(unsigned verified) {
  unsigned mode = 0;
  return PE_parse_boot_argn("guamp-bt", &mode, sizeof(mode)) && mode == 2 ? verified : 0u;
}
