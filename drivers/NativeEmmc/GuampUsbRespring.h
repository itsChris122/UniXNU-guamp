// SPDX-License-Identifier: BSD-2-Clause-Patent
#ifndef GUAMP_USB_RESPRING_H
#define GUAMP_USB_RESPRING_H
#include <stdint.h>
inline uint32_t guampRespringPid(volatile const uint8_t *b,uint32_t n,bool enabled) {
  if (!enabled || n!=12 || b[0]!='G' || b[1]!='R' || b[2]!='S' || b[3]!='P' ||
      b[8]!=1 || b[9] || b[10] || b[11]) return 0;
  uint32_t pid=static_cast<uint32_t>(b[4]) | (static_cast<uint32_t>(b[5])<<8) |
               (static_cast<uint32_t>(b[6])<<16) | (static_cast<uint32_t>(b[7])<<24);
  return pid>1 && pid<=0x7fffffff ? pid : 0;
}
#endif
