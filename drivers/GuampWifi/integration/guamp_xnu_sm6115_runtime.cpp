/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * Guamp cold-boot owner: RPM -> PAS loader hooks -> SMP2P/GLINK/QRTR/WLFW ->
 * MSA/rails/SMMU/DMA -> ath10k -> diagnostic scan or optional iOS station.
 * All progress is based on ACKs/indications; failure retains live resources.
 */
#include <IOKit/IOLib.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/debug.h>
#include <pexpert/pexpert.h>
#include "guamp_xnu_sm6115_runtime.h"
#include "guamp_xnu_sm6115_provider.h"
#include "GuampScm.inc"
#include "GuampScmArm64.inc"
#include "GuampMsa.inc"
#include "guamp_xnu_dma.h"
#include "guamp_wlfw_board.h"
#include "guamp_xnu_firmware.h"
#if defined(GUAMP_WIFI_REDWOOD) && GUAMP_WIFI_REDWOOD
extern "C" {
#include "guamp_xnu_station.h"
}
#endif

extern "C" int guamp_ath10k_firmware_get(const char *,const uint8_t **,size_t *);
extern "C" {
volatile unsigned guamp_wifi_provider_stage=0;
volatile int guamp_wifi_provider_error=0;
}

namespace guampwifi {
struct Sm6115Runtime {
  enum Stage : unsigned { Off,Mapping,RpmConnect,ProxyVotes,Prepared,Modem,
    Firmware,RadioProbe,Scanning,ScanComplete,IosAttached,Failed,AwaitNetwork };
  GuampXnuSm6115Provider owner{};
  GuampXnuBuffer msaPayload{},pageTables{};
  scm::Transport transport{guampScmInstruction,scmSleep,nullptr,true};
  MsaAssignment msa{};
  WlanSmmu smmu{};
  WifiControl control{};
  WlfwBoardBinding binding{};
  lx_platform platform{};
  gw_snoc_board board{};
  Stage stage=Off;
  int error=0;
  bool remoteMayRun=false,shutdownFailed=false,dmaStarted=false;
  bool pumping=false,ath10kStarted=false,workerLive=false;
  bool scanDone=false;
  int scanResult=NW_RES_FAILED;
  unsigned workerMode=0,networkReady=0;

  static uint64_t nowMs() {
    uint64_t ns=0;absolutetime_to_nanoseconds(mach_absolute_time(),&ns);return ns/1000000;
  }
  static void scmSleep(void *,uint32_t ms) { IOSleep(ms); }
  void transition(Stage next) {
    __atomic_store_n(&stage,next,__ATOMIC_RELEASE);guamp_wifi_provider_stage=next;
    IOLog("GuampWiFi: provider stage %u at %llu ms\n",unsigned(next),nowMs());
  }
  int fail(int e,const char *where) {
    error=e?e:Io;guamp_wifi_provider_error=error;
    IOLog("GuampWiFi: %s failed %d (control %u WLFW %u SMMU fault %x); resources retained while remote may run\n",
      where,error,unsigned(control.stage),unsigned(control.wlan.state),smmu.faultStatus());
    transition(Failed);return error;
  }

  int prepare(uint64_t base,uint64_t bytes) {
    if(stage!=Off)return Busy;
    transition(Mapping);
    if(!owner.start(base,bytes))return fail(NoMemory,"physical mappings");
    if(!transport.discover())return fail(Unsupported,"SCM convention");
    if(!msaPayload.open(4096,0xfffff000ull,base,bytes)||
       !pageTables.open(WlanSmmu::TableBytes,0xfffff000ull,base,bytes))return fail(NoMemory,"SCM/SMMU backing");
    msa.payload=msaPayload.memory;msa.physical=msaPayload.physical;msa.transport=&transport;
    int e=owner.startRpm();if(e)return fail(e,"RPM start");
    transition(RpmConnect);
    uint64_t deadline=nowMs()+7000;
    while(!owner.rpm.online()) {
      e=owner.stepRpm(nowMs());if(e)return fail(e,"RPM negotiate");
      if(nowMs()>=deadline)return fail(Timeout,"RPM channel");
      IOSleep(1);
    }
    transition(ProxyVotes);e=applyVotes(false);if(e)return fail(e,"proxy votes");
    WifiControl::Ops ops{};ops.ctx=this;ops.smpKick=smpKick;ops.glinkKick=glinkKick;
    ops.wlan={this,nullptr,assignMsa,wlanPower,fileSize,fileRead,wlfwEvent};
    ops.event=controlEvent;
    e=control.prepare(&owner.arena,owner.allocator,ops,true);
    if(e)return fail(e,"APPS SMP2P/GLINK prepare");
    transition(Prepared);return Ok;
  }

  int modemStarted() {
    if(stage!=Prepared)return NotReady;
    remoteMayRun=true;transition(Modem);
    int e=control.modemStarted(nowMs());if(e)return fail(e,"modem handoff");
    const uint64_t deadline=nowMs()+120000;
    while(!control.firmwareReady()) {
      e=pump();if(e)return e;
      if(nowMs()>=deadline)return fail(Timeout,"WLFW firmware ready");
      IOSleep(1);
    }
    if(!physicalReady())return fail(NotReady,"radio prerequisites");
    transition(Firmware);return Ok;
  }

  int applyVotes(bool wlan) {
    int e=owner.beginPowerVotes(wlan);if(e)return e;
    const uint64_t deadline=nowMs()+30000;
    while(owner.power.busy) {
      e=owner.stepPowerVotes(nowMs());if(e)return e;
      if(nowMs()>=deadline)return Timeout;
      IOSleep(1);
    }
    return (wlan?owner.power.wlanHeld:owner.power.proxyHeld)?Ok:NotReady;
  }

  int startWlanResources() {
    // A newly booted PAS owner is mandatory; an already-running WLAN service
    // cannot be remapped underneath its DMA engines.
    if(!remoteMayRun||control.wlan.alreadyRunning) {
      IOLog("GuampWiFi: WLAN power refused: remote %s, WLAN %s\n",remoteMayRun?"started":"not started by this boot",
            control.wlan.alreadyRunning?"ALREADY RUNNING (not a cold boot)":"not running");
      return Busy;
    }
    if(!smmu.active) {
      if(!pageTables.retainForDevice()) {IOLog("GuampWiFi: SMMU page tables not retained\n");return NoMemory;}
      int e=smmu.start(owner.smmuRegisters,pageTables.memory,pageTables.physical,
        owner.xnuBase,owner.xnuBytes,true,this,delayUs);
      IOLog("GuampWiFi: SMMU c600000 SID %x mask %x: %d groups, %d banks (%u S2), SMR slot %u%s, context bank %u, "
            "tables %llx, result %d\n",WlanSmmu::Stream,WlanSmmu::StreamMask,int(smmu.groupsSeen),int(smmu.banksSeen),
            smmu.s2cbSeen,smmu.slot,smmu.preexisting?" (SMR ALREADY VALID: WLAN set up before XNU)":" (free)",smmu.cb,
            pageTables.physical,e);
      if(e)return e;
    }
    if(!dmaStarted) {
      extern GuampXnuDma gGuampXnuDma;
      if(!gGuampXnuDma.init(&smmu,owner.xnuBase,owner.xnuBytes)) {IOLog("GuampWiFi: DMA window init failed\n");return NoMemory;}
      dmaStarted=true;
    }
    int e=applyVotes(true);
    IOLog("GuampWiFi: WLAN rails L8/L16/L17/L23 votes %s (%d)\n",e?"FAILED":"acked",e);
    return e;
  }

  bool physicalReady() const {
    return remoteMayRun&&!shutdownFailed&&owner.power.wlanHeld&&smmu.active&&dmaStarted&&
      (control.firmwareReady()||control.radioEnabled())&&control.wlan.capability.chipValid;
  }
  int pump() {
    if(pumping)return Ok;
    if(error)return error;
    pumping=true;
    int e=owner.stepRpm(nowMs());
    if(!e&&control.stage!=WifiControl::Off&&control.stage!=WifiControl::Prepared)e=control.step(nowMs());
    if(!e&&ath10kStarted)gw_snoc_poll_ce();
    pumping=false;
    return e?fail(e,"transport pump"):Ok;
  }

  int configureStation() {
    if(!physicalReady())return NotReady;
    platform={nowMs,sleepMs,poll,log,halt,GuampXnuDma::allocate,GuampXnuDma::deallocate,
      nullptr,nullptr,nullptr,nullptr,firmwareGet};
    if(!GuampXnuDma::installHooks(&platform))return NoMemory;
    gw_snoc_board physical{};physical.ctx=this;
    physical.registers=const_cast<uint8_t *>(owner.wlanMap.bytes);
    physical.register_bytes=owner.wlanMap.size;
    physical.firmware_chip_id=control.wlan.capability.chip;
    physical.power=boardPower;physical.ready=boardReady;
    if(!binding.bind(&physical,&control,this,bridgeNow,bridgeSleep,bridgePoll,&board))return Invalid;
    return Ok;
  }

  int startStation(unsigned mode) {
    if(__atomic_load_n(&stage,__ATOMIC_ACQUIRE)!=Firmware||workerLive)return Busy;
    if(mode!=1&&mode!=2)return Invalid;
#if !defined(GUAMP_WIFI_REDWOOD) || !GUAMP_WIFI_REDWOOD
    if(mode==2)return Unsupported;
#endif
    int e=configureStation();if(e)return fail(e,"station config");
    transition(RadioProbe);
    thread_t worker=nullptr;workerLive=true;workerMode=mode;
    if(kernel_thread_start(scanWorker,this,&worker)!=KERN_SUCCESS) {
      workerLive=false;return fail(NoMemory,"scan worker");
    }
    thread_deallocate(worker);return Ok;
  }

  void runStation() {
    if(workerMode==1){runScan();return;}
#if defined(GUAMP_WIFI_REDWOOD) && GUAMP_WIFI_REDWOOD
    transition(AwaitNetwork);const uint64_t deadline=nowMs()+120000;
    while(!__atomic_load_n(&networkReady,__ATOMIC_ACQUIRE)) {
      if(pump())return;
      if(nowMs()>=deadline){fail(Timeout,"BSD network-ready hook");return;}
      IOSleep(1);
    }
    transition(RadioProbe);ath10kStarted=true;
    const int e=guamp_xnu_wifi_attach(&platform,&board);
    if(e){fail(e,"iOS attach");return;}
    // attach has handed dispatch to its own worker. Do not resume provider
    // pumping here, and do not overwrite an error reported by that worker.
    Stage expected=RadioProbe;
    if(__atomic_compare_exchange_n(&stage,&expected,IosAttached,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE)) {
      guamp_wifi_provider_stage=IosAttached;IOLog("GuampWiFi: iOS station attached\n");
    }
#endif
  }

  void runScan() {
    const nw_wifi_events events={scanFinished,nullptr,nullptr,nullptr,nullptr,nullptr};
    nw_wifi_set_events(&events);ath10kStarted=true;
    int e=nw_wifi_start_snoc(&platform,&board);
    if(e){fail(e,"ath10k start");return;}
    uint64_t deadline=nowMs()+30000;
    nw_wifi_status status{};
    for(;;) {
      e=nw_wifi_step();if(e<0||error){fail(error?error:e,"ath10k probe");return;}
      nw_wifi_status_get(&status);
      bool mac=false;for(unsigned i=0;i<6;i++)mac|=status.mac[i]!=0;
      if(status.state==NW_STA_IDLE&&status.channels&&mac&&!(status.mac[0]&1))break;
      if(nowMs()>=deadline){fail(Timeout,"ath10k station ready");return;}
      IOSleep(1);
    }
    IOLog("GuampWiFi: ath10k ready, MAC %02x:%02x:%02x:%02x:%02x:%02x, %u channels\n",
      status.mac[0],status.mac[1],status.mac[2],status.mac[3],status.mac[4],status.mac[5],status.channels);
    scanDone=false;scanResult=NW_RES_FAILED;transition(Scanning);
    e=nw_wifi_scan(nullptr,0);if(e){fail(e,"scan request");return;}
    deadline=nowMs()+35000;
    while(!scanDone) {
      e=nw_wifi_step();if(e<0||error){fail(error?error:e,"scan pump");return;}
      if(nowMs()>=deadline){fail(Timeout,"scan completion");return;}
      IOSleep(1);
    }
    if(scanResult!=NW_RES_OK){fail(Io,"scan aborted");return;}
    const int count=nw_wifi_bss_count();
    IOLog("GuampWiFi: SCAN COMPLETE: %d BSS entries (zero is a valid completed scan)\n",count);
    for(int i=0;i<count;i++) {
      nw_bss_info bss{};if(nw_wifi_bss_get(i,&bss))continue;
      // Do not print SSIDs/passphrases; BSSID/frequency are sufficient to
      // correlate a completed radio scan with its CE/WMI messages.
      IOLog("GuampWiFi: BSS %02x:%02x:%02x:%02x:%02x:%02x %u MHz %d dBm IE %u bytes\n",
        bss.bssid[0],bss.bssid[1],bss.bssid[2],bss.bssid[3],bss.bssid[4],bss.bssid[5],
        bss.freq_mhz,int(bss.signal_dbm),unsigned(bss.ie_len));
    }
    transition(ScanComplete);
    // Retain the single dispatcher and firmware services for this boot.
    while(!error) { e=nw_wifi_step();if(e<0){fail(e,"radio worker");break;}if(!e)IOSleep(1); }
  }

  void shutdownResult(bool stopped) {
    shutdownFailed=!stopped;remoteMayRun=!stopped;
    if(stopped)releaseStopped();
  }
  void abort() { if(!remoteMayRun&&!shutdownFailed&&!workerLive)releaseStopped(); }
  void releaseStopped() {
    if(workerLive||ath10kStarted)return; // teardown requires a joined station worker
    control.release(true);
    extern GuampXnuDma gGuampXnuDma;
    if(dmaStarted&&!gGuampXnuDma.stop(true))return;
    dmaStarted=false;
    if(smmu.stop(true))return;
    pageTables.close(true);msaPayload.close(true);owner.stop(true);
  }

  static Sm6115Runtime &instance();
  static void delayUs(void *,uint32_t us) { IODelay(us); }
  static void smpKick(void *p) { static_cast<Sm6115Runtime *>(p)->owner.kickSmp2p(); }
  static void glinkKick(void *p) { static_cast<Sm6115Runtime *>(p)->owner.kickModemGlink(); }
  static int assignMsa(void *p,const Wlfw::Region *regions,uint32_t count) {
    auto &r=*static_cast<Sm6115Runtime *>(p);
    if(!regions||!count||count>2) {IOLog("GuampWiFi: MSA info refused: %u regions\n",count);return Invalid;}
    // Validate the complete list before the first irreversible assignment.
    for(uint32_t i=0;i<count;i++) {
      IOLog("GuampWiFi: MSA region %u: %llx + %llx %s\n",i,uint64_t(regions[i].address),uint64_t(regions[i].size),
            regions[i].secure?"secure":"non-secure");
      if(!regions[i].size||(regions[i].address&4095)||
        (regions[i].size&4095)||regions[i].address<Wlfw::MsaBase||
        regions[i].address-Wlfw::MsaBase>=Wlfw::MsaBytes||
        regions[i].size>Wlfw::MsaBytes-(regions[i].address-Wlfw::MsaBase)) {
        IOLog("GuampWiFi: MSA region %u outside wlan_msa_region 51900000 + 100000\n",i);return Range;
      }
    }
    // icnss.c assigns MSA permissions only when its DT node has "qcom,hyp_enabled" (icnss_assign_msa_perm returns 0
    // otherwise). The guamp qcom,icnss@C800000 node does not have it: Linux never sends MEM_PROT_ASSIGN for the MSA on
    // this phone, the fixed wlan_msa_region keeps the ownership the boot chain gave it. Same here by default;
    // guamp-wifi-msa-assign=1 sends the documented hyp_assign (HLOS -> MSS_MSA/WLAN[/WLAN_CE] RW) as an experiment.
    unsigned assign=0;
    PE_parse_boot_argn("guamp-wifi-msa-assign",&assign,sizeof(assign));
    if(assign!=1) {
      IOLog("GuampWiFi: MSA permissions left as set by the boot chain (DT icnss without qcom,hyp_enabled, as Linux)\n");
      return Ok;
    }
    if(!r.msaPayload.retainForDevice()) {IOLog("GuampWiFi: MSA SCM payload not retained\n");return Invalid;}
    for(uint32_t i=0;i<count;i++) {
      int e=r.msa.assign(regions[i].address,regions[i].size,regions[i].secure);
      IOLog("GuampWiFi: MSA hyp_assign region %u -> VMID 15/24%s: %d\n",i,regions[i].secure?"":"/25",e);
      if(e)return e;
    }
    return Ok;
  }
  static int wlanPower(void *p,bool on) { return on?static_cast<Sm6115Runtime *>(p)->startWlanResources():Busy; }
  static int fileSize(void *,const char *name,uint32_t *bytes) { return guamp_wifi_firmware_file_size(name,bytes); }
  static int fileRead(void *,const char *name,uint32_t at,uint8_t *out,uint32_t bytes) { return guamp_wifi_firmware_file_read(name,at,out,bytes); }
  static void controlEvent(void *,WifiControl::Stage state,int e) { IOLog("GuampWiFi: control %u error %d\n",unsigned(state),e); }
  static void wlfwEvent(void *,Wlfw::State state,int e) { IOLog("GuampWiFi: WLFW %u error %d\n",unsigned(state),e); }
  static int boardReady(void *p) { return static_cast<Sm6115Runtime *>(p)->physicalReady(); }
  static int boardPower(void *p,int on) {
    auto &r=*static_cast<Sm6115Runtime *>(p);
    if(on)return r.physicalReady()?Ok:NotReady;
    return r.control.firmwareReady()?r.owner.power.releaseWlan(true):Busy;
  }
  static uint64_t bridgeNow(void *) { return nowMs(); }
  static void bridgeSleep(void *,uint32_t ms) { IOSleep(ms); }
  static void bridgePoll(void *p) { (void)static_cast<Sm6115Runtime *>(p)->pump(); }
  static void sleepMs(unsigned ms) { IOSleep(ms); }
  static void poll() { (void)instance().pump(); }
  static void log(int level,const char *line) { IOLog("GuampWiFi[%d]: %s\n",level,line); }
  static void halt() { panic("GuampWiFi: fatal ath10k runtime error"); }
  static int firmwareGet(const char *name,const uint8_t **data,size_t *bytes) {
    int e=guamp_ath10k_firmware_get(name,data,bytes);return !e?0:(e==NotFound?-2:-5);
  }
  static void scanFinished(int result) { instance().scanResult=result;instance().scanDone=true; }
  static void scanWorker(void *p,wait_result_t) {
    auto &r=*static_cast<Sm6115Runtime *>(p);thread_set_thread_name(current_thread(),"guamp-wifi-scan");
    r.runStation();r.workerLive=false;
  }
};
static Sm6115Runtime gSm6115Runtime;
Sm6115Runtime &Sm6115Runtime::instance() { return gSm6115Runtime; }
} // namespace guampwifi

extern "C" int guamp_sm6115_wifi_prepare(uint64_t base,uint64_t bytes) { return guampwifi::Sm6115Runtime::instance().prepare(base,bytes); }
extern "C" int guamp_sm6115_wifi_modem_started() { return guampwifi::Sm6115Runtime::instance().modemStarted(); }
extern "C" void guamp_sm6115_wifi_shutdown_result(int stopped) { guampwifi::Sm6115Runtime::instance().shutdownResult(stopped!=0); }
extern "C" void guamp_sm6115_wifi_abort() { guampwifi::Sm6115Runtime::instance().abort(); }
extern "C" int guamp_sm6115_wifi_start_station(unsigned mode) { return guampwifi::Sm6115Runtime::instance().startStation(mode); }
extern "C" void guamp_sm6115_wifi_network_ready() { __atomic_store_n(&guampwifi::Sm6115Runtime::instance().networkReady,1u,__ATOMIC_RELEASE); }
