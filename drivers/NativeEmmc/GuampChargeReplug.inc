// SPDX-License-Identifier: BSD-2-Clause-Patent
// Included in pmic after the existing charger function. Opt-in replacement for
// the legacy charge poll, not an additional competing writer.
static guamp_charge::Tracker gChargeTracker;
static unsigned gChargeErrors;
static uint64_t gChargeWatchdog;
static int chargeMasked(uint16_t address, uint8_t mask, uint8_t value) {
  uint8_t old = 0;
  int error = read8(kPmi632,address,&old);
  if (error) return error;
  uint8_t desired=static_cast<uint8_t>((old & ~mask) | (value & mask));
  return old==desired ? 0 : write8(kPmi632,address,desired);
}
static int chargeConfigure(unsigned iclMa,unsigned fccMa,uint8_t apsd,uint8_t *iclCode,uint8_t *fccCode) {
  iclMa=guamp_charge::inputMa(iclMa,apsd);
  uint8_t icl=static_cast<uint8_t>(iclMa/50),fcc=static_cast<uint8_t>(fccMa/50);
  uint8_t fv=static_cast<uint8_t>((kFvMv-3600)/10);
  // Same voltage/current ceiling as the installed charger service. Do not raise limits.
  int error=write8(kPmi632,kFvCfg,fv);
  if (!error) error=write8(kPmi632,kFccCfg,fcc);
  if (!error) error=write8(kPmi632,kIclCfg,icl);
  if (!error) error=write8(kPmi632,kIclOptions,0x03);
  if (!error) error=chargeMasked(kUsbinCmdIl,1,0);
  if (!error) error=chargeMasked(kChgEnableCmd,1,0);
  if (!error) {
    IODelay(1000);
    error=chargeMasked(kChgEnableCmd,1,1);
  }
  uint8_t actualIcl=0,actualFcc=0,actualFv=0,enable=0;
  if (!error) error=read8(kPmi632,kIclCfg,&actualIcl);
  if (!error) error=read8(kPmi632,kFccCfg,&actualFcc);
  if (!error) error=read8(kPmi632,kFvCfg,&actualFv);
  if (!error) error=read8(kPmi632,kChgEnableCmd,&enable);
  if (!error && (actualIcl!=icl || actualFcc!=fcc || actualFv!=fv || !(enable&1))) error=SPMI_ERR_IO;
  if (!error) { *iclCode=icl; *fccCode=fcc; }
  return error;
}
static void chargeReplugPoll(unsigned iclMa,unsigned fccMa,uint8_t *iclCode,uint8_t *fccCode) {
  uint64_t ns=0;
  absolutetime_to_nanoseconds(mach_absolute_time(),&ns);
  uint64_t ms=ns/1000000;
  uint8_t mode=0,usb=0,typec=0,misc=0,state=0,s2=0,s7=0,apsd=0,result=0,fv=0,suspend=0;
  int error=read8(kPmi632,kTypecModeCfg,&mode);
  // The old 0x10 was TRY_SINK, not SINK_ONLY. Linux's role setter masks bits0..4
  // and writes bit1 for receiver-only mode; preserve unrelated upper bits.
  uint8_t desired=guamp_charge::receiverMode(mode);
  if (!error && mode!=desired) error=write8(kPmi632,kTypecModeCfg,desired);
  if (!error) error=chargeMasked(kTypecCfg4a,0x1f,0x03);
  if (!error) error=read8(kPmi632,0x1310,&usb);
  if (!error) error=read8(kPmi632,0x1510,&typec);
  if (!error) error=read8(kPmi632,0x150b,&misc);
  if (!error) error=read8(kPmi632,kChgStatus1,&state);
  if (!error) error=read8(kPmi632,kChgStatus2,&s2);
  if (!error) error=read8(kPmi632,kChgStatus7,&s7);
  if (!error) error=read8(kPmi632,kApsdStatus,&apsd);
  if (!error) error=read8(kPmi632,kApsdResult,&result);
  if (!error) error=read8(kPmi632,kFvCfg,&fv);
  if (!error) error=read8(kPmi632,kUsbinCmdIl,&suspend);
  if (!error && ms-gChargeWatchdog>=5000) {
    error=write8(kPmi632,kWdogPet,1);
    if (!error) gChargeWatchdog=ms;
  }
  if (error) {
    ++gChargeErrors;
    if (gChargeErrors==1 || gChargeErrors%20==0) IOLog("GuampCharge: register read/setup error %d count %u; no fake detach/configuration acknowledgement\n",error,gChargeErrors);
    return;
  }
  gTypecMode=desired;
  gChg[0]=state;gChg[1]=s2;gChg[2]=s7;gChg[3]=apsd;gChg[4]=result;gChg[5]=fv;gChg[6]=usb;gChg[7]=typec;
  bool present=(usb&0x10)!=0;
  if (present!=gChargeTracker.present)
    IOLog("GuampCharge: source %s USBIN %02x TypeC-misc %02x mode %02x charger-state %u\n",present?"attached":"detached",usb,misc,desired,state&7);
  guamp_charge::Action action=gChargeTracker.next(ms,present,(apsd&1)!=0,state&7,(suspend&1)!=0);
  if (action==guamp_charge::BeginDetection) {
    // Enable BC1.2 source detection only; leave high-voltage negotiation policy untouched.
    error=chargeMasked(0x1362,0x08,0x08);
    if (!error) error=chargeMasked(kUsbinCmdIl,1,0);
    if (!error) error=write8(kPmi632,0x1341,1);
  } else if (action==guamp_charge::Configure) {
    if (s2&2) error=SPMI_ERR_IO; // Battery overvoltage status: do not restart the charger.
    else error=chargeConfigure(iclMa,fccMa,result,iclCode,fccCode);
  } else return;
  gChargeTracker.acknowledge(action,error==0,ms);
  if (error) ++gChargeErrors;
  IOLog("GuampCharge: action %s status %d APSD %02x result %02x ICL-code %u FCC-code %u configured %u errors %u\n",
        action==guamp_charge::BeginDetection?"source-detection":"configure/readback",error,apsd,result,*iclCode,*fccCode,gChargeTracker.configured,gChargeErrors);
}
