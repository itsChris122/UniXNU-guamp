// SPDX-License-Identifier: BSD-2-Clause-Patent
// SMCCC register preservation, including the Qualcomm x6 resume quirk. Copy of wifi/driver-modem/GuampScmArm64.inc
// with an include guard, shared by the modem and the ADSP PAS loaders (GuampPas.inc).
#ifndef GUAMP_SCM_ARM64_INC
#define GUAMP_SCM_ARM64_INC
static void guampModemBarrier() {__asm__ volatile("dsb sy" ::: "memory");}
static void guampScmInstruction(void *,const scm::Registers &in,scm::Registers &out) {
  register uint64_t x0 __asm__("x0")=in.x[0], x1 __asm__("x1")=in.x[1];
  register uint64_t x2 __asm__("x2")=in.x[2], x3 __asm__("x3")=in.x[3];
  register uint64_t x4 __asm__("x4")=in.x[4], x5 __asm__("x5")=in.x[5];
  register uint64_t x6 __asm__("x6")=in.x[6], x7 __asm__("x7")=in.x[7];
  __asm__ volatile("smc #0" : "+r"(x0),"+r"(x1),"+r"(x2),"+r"(x3),
      "+r"(x4),"+r"(x5),"+r"(x6),"+r"(x7) ::
      "x8","x9","x10","x11","x12","x13","x14","x15","x16","x17","cc","memory");
  out.x[0]=x0;out.x[1]=x1;out.x[2]=x2;out.x[3]=x3;
  out.x[4]=x4;out.x[5]=x5;out.x[6]=x6;out.x[7]=x7;
}
#endif
