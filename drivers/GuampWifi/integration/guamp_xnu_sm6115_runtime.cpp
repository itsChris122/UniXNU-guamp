/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * Guamp cold-boot owner: RPM -> PAS loader hooks -> SMP2P/GLINK/QRTR/WLFW ->
 * MSA/rails/SMMU/DMA -> ath10k -> diagnostic scan or optional iOS station.
 * All progress is based on ACKs/indications; failure retains live resources.
 */
#include <IOKit/IOLib.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/debug.h>
#include <kern/sched_prim.h>
#include <machine/machine_routines.h>
#include <libkern/libkern.h>
#include <pexpert/pexpert.h>
#include <IOKit/IORegistryEntry.h>
#include <libkern/c++/OSData.h>
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
extern unsigned char gw_board_mac[6];   // ath10k-port gw_runtime.c: device_get_mac_address()
extern volatile int gw_board_mac_valid;
volatile unsigned guamp_wifi_provider_stage=0;
volatile int guamp_wifi_provider_error=0;
void guamp_emmc_log_sync(void);   // NativeEmmc.cpp: one kpan pass of the kernel log (guamp-wifi-trace-kpan=1)
}

namespace guampwifi {
// guamp-wifi-trace=N (diagnostics, default 0 = off, nothing changes): one "GuampWiFiTrace:" line before each radio probe
// step (runStation, attach, gw_snoc_start, gw_snoc_step) and before the first N register accesses, CE polls, dispatcher
// items, steps and attach iterations; each line is followed by guamp-wifi-trace-ms (default 150) of sleep so the USB mirror
// sends it before a bus hang can stop the phone. guamp-wifi-trace-kpan=1 also writes the kernel log to kpan after each
// line (synchronous eMMC pass, no framebuffer). Where the thread may not sleep the line says "nosleep" and nothing waits.
static unsigned gTraceMs=150,gTraceKpan=0;
struct Sm6115Runtime {
  enum Stage : unsigned { Off,Mapping,RpmConnect,ProxyVotes,Prepared,Modem,
    Firmware,RadioProbe,Scanning,ScanComplete,IosAttached,Failed,AwaitNetwork,TransportOnly };
  GuampXnuSm6115Provider owner{};
  GuampXnuBuffer msaPayload{},pageTables{};
  // qcom,rmtfs_sharedmem (msm_sharedmem client "rmtfs"): reg <0 0x200000> = dynamic 2 MiB, qcom,guard-memory
  // (4 KiB before and after), qcom,client-id 1 (MPSS RMTS), qcom,vm-nav-path.
  static constexpr uint32_t RmtfsRegion=0x200000,RmtfsGuard=0x1000;
  GuampXnuBuffer rmtfsMem{};
  Rmtfs::Storage rmtfsIo{};
  bool rmtfsReady=false,rmtfsAssigned=false;
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
  bool scanDone=false,modemReady=false;
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
    // control: 0 Off 1 Prepared 2 AwaitModem 3 AwaitFifo 4 Running 5 Failed; WLFW 15 = Failed (set by peerLost()
    // whenever the control layer fails, so it is a consequence, not a WLFW error); gate = the step that failed.
    IOLog("GuampWiFi: %s failed %d %s (control %u gate \"%s\" WLFW %u SMMU fault %x); resources retained while remote may run\n",
      where,error,errorName(error),unsigned(control.stage),control.where(),unsigned(control.wlan.state),smmu.faultStatus());
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
    ops.event=controlEvent;ops.trace=controlTrace;ops.qrtrTrace=qrtrTrace;
    ops.qftpTrace=qftpTrace;ops.servregTrace=servregTrace;
    rmtfsReady=setupRmtfs(base,bytes);ops.rmtfs=rmtfsReady?&rmtfsIo:nullptr;
    unsigned sim=0;PE_parse_boot_argn("guamp-sim",&sim,sizeof(sim));
    ops.sim=sim==1;ops.simSnapshot=simSnapshot;
    if(ops.sim)IOLog("GuampSim: read-only SIM reader on: QMI UIM/DMS/NAS clients on QRTR (no operating mode, PIN or write request)\n");
    e=control.prepare(&owner.arena,owner.allocator,ops,true);
    if(e)return fail(e,"APPS SMP2P/GLINK prepare");
    transition(Prepared);return Ok;
  }

  // Before the PAS start: the region the modem's EFS client is told about (alloc_buff) must already exist with the
  // permissions Linux gives it. msm_sharedmem.c: dma_alloc_coherent(size + 8 KiB), base + 4 KiB, then
  // setup_shared_ram_perms() = hyp_assign_phys(HLOS -> HLOS, MSS_MSA, NAV, all RW), because the DT address is 0
  // (dynamic) and client-id is 1. The boot chain never assigns this region (it has no fixed address), so this is
  // the one assignment the DT asks for; guamp-rmtfs-assign=0 skips it. A failure leaves Wi-Fi as before (no server).
  bool setupRmtfs(uint64_t base,uint64_t bytes) {
    unsigned enabled=1;PE_parse_boot_argn("guamp-rmtfs",&enabled,sizeof(enabled));
    if(!enabled) {IOLog("GuampRmtfs: disabled by guamp-rmtfs=0: service 14 not published\n");return false;}
    // rw_iovec phys_offset is a 32-bit physical address: the region must sit below 4 GiB (Linux: 32-bit DMA mask).
    if(!rmtfsMem.open(RmtfsRegion+2*RmtfsGuard,0xfffff000ull,base,bytes)) {
      IOLog("GuampRmtfs: shared memory allocation FAILED (2 MiB + 8 KiB contiguous below 4 GiB): service 14 not published\n");
      return false;
    }
    const uint64_t region=rmtfsMem.physical+RmtfsGuard;
    IOLog("GuampRmtfs: shared memory %llx + %x (allocation %llx + %x, 4 KiB guards), uncached, client-id 1\n",
          region,RmtfsRegion,rmtfsMem.physical,rmtfsMem.size);
    // From here the pages may be handed to the modem: they are never returned to XNU during this boot.
    rmtfsMem.retainForDevice();
    unsigned assign=1;PE_parse_boot_argn("guamp-rmtfs-assign",&assign,sizeof(assign));
    if(assign) {
      static constexpr uint32_t vmids[3]={3u,15u,0x2bu};   // VMID_HLOS, VMID_MSS_MSA, VMID_NAV (secure_buffer.h)
      int e=msaPayload.retainForDevice()?msa.assignShared(region,RmtfsRegion,vmids,3):Invalid;
      rmtfsAssigned=!e;
      IOLog("GuampRmtfs: hyp_assign %llx + %x HLOS -> HLOS/MSS_MSA/NAV RW (msm_sharedmem, vm-nav-path): %d %s%s\n",
            region,RmtfsRegion,e,errorName(e),e?" (continuing as Linux does; the modem may fault on the region)":"");
    } else {
      IOLog("GuampRmtfs: hyp_assign skipped (guamp-rmtfs-assign=0): region stays HLOS-only\n");
    }
    unsigned write=0;PE_parse_boot_argn("guamp-rmtfs-write",&write,sizeof(write));
    rmtfsIo={this,rmtfsMem.memory,RmtfsGuard,region,RmtfsRegion,owner.allocator,
             rmtfsPartition,rmtfsRead,write==1?rmtfsWrite:nullptr,rmtfsTrace};
    IOLog("GuampRmtfs: modem EFS writes %s\n",write==1?
          "go to eMMC for modemst1/modemst2 (guamp-rmtfs-write=1); fsc/fsg writes stay in RAM":
          "stay in RAM for this boot (eMMC read-only; guamp-rmtfs-write=1 writes modemst1/2 to eMMC)");
    return true;
  }

  // PAS readiness = the modem's own ready signal (SMP2P 435 ready bit, as subsys-pil-tz err_ready), nothing more.
  // Only a modem that never gets there (control fails after ReadyMs, or a fatal) is handed back for PAS SHUTDOWN,
  // and the transport is closed first (quiesce). From the ready signal on, no Wi-Fi outcome (WLFW never announced,
  // ath10k failure...) shuts the modem down: the station worker awaits WLFW and keeps QRTR/RMTFS/TFTP/SIM pumped.
  int modemStarted() {
    if(stage!=Prepared)return NotReady;
    remoteMayRun=true;transition(Modem);
    int e=control.modemStarted(nowMs());
    if(e){control.quiesce();return fail(e,"modem handoff");}
    while(control.stage==WifiControl::AwaitModem) {
      e=pump();
      if(e){control.quiesce();IOLog("GuampWiFi: modem not ready (%d %s): transport closed before the PAS shutdown\n",e,errorName(e));return e;}
      IOSleep(1);
    }
    if(control.stage==WifiControl::Failed||control.stage==WifiControl::Off) {
      control.quiesce();return fail(control.error?control.error:Io,"modem ready");
    }
    modemReady=true;
    IOLog("GuampWiFi: modem ready at %llu ms: never shut down by the Wi-Fi path from now on; WLFW awaited by the station worker\n",nowMs());
    return Ok;
  }

  // Station worker, first step: WLFW firmware-ready while pumping the transport. guamp-wifi-wlfw-wait=<s> (default
  // 300, 0 = no limit). A timeout stops the Wi-Fi part only.
  bool awaitFirmware() {
    unsigned wait=300;PE_parse_boot_argn("guamp-wifi-wlfw-wait",&wait,sizeof(wait));
    const uint64_t began=nowMs(),deadline=wait?began+uint64_t(wait)*1000:0;
    uint64_t beat=began;
    while(!control.firmwareReady()) {
      if(pump())return false;
      const uint64_t now=nowMs();
      if(deadline&&now>=deadline) {
        IOLog("GuampWiFi: WLFW 0x45 not announced after %u s: Wi-Fi part stopped; modem, QRTR, RMTFS, TFTP, SERVREG_LOC "
              "and SIM keep running (guamp-wifi-wlfw-wait=0 waits without limit)\n",wait);
        transition(TransportOnly);return false;
      }
      if(now-beat>=30000){beat=now;IOLog("GuampWiFi: still waiting for WLFW 0x45 (%llu s, modem alive)\n",(now-began)/1000);}
      IOSleep(1);
    }
    if(!physicalReady()){fail(NotReady,"radio prerequisites");return false;}
    transition(Firmware);return true;
  }
  // The live modem keeps needing RMTFS/TFTP/SERVREG_LOC/SIM after any Wi-Fi outcome: pump the transport for the boot.
  void transportOnly() {
    if(__atomic_load_n(&stage,__ATOMIC_ACQUIRE)==IosAttached)return;   // the iOS worker owns dispatch
    IOLog("GuampWiFi: worker keeps the modem transport running (QRTR, RMTFS, TFTP, SERVREG_LOC, SIM)\n");
    while(control.stage!=WifiControl::Failed&&control.stage!=WifiControl::Off){(void)pump();IOSleep(1);}
    IOLog("GuampWiFi: modem transport stopped (control %u, error %d); modem left running\n",unsigned(control.stage),control.error);
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
      IOLog("GuampWiFi: SMMU SID 0x%x/0x%x -> context bank %u (SMR slot %u), tables PA 0x%llx, IOVA 0x%llx-0x%llx, NC walks\n",
        WlanSmmu::Stream,WlanSmmu::StreamMask,smmu.cb,smmu.slot,pageTables.physical,WlanSmmu::IovaBase,WlanSmmu::IovaEnd);
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
  // The transport is stepped even after a Wi-Fi failure (error set): only the CE poll stops. The result is the
  // Wi-Fi error (so Wi-Fi loops end), or the transport error that failed it.
  int pump() {
    if(pumping)return error;
    pumping=true;
    int e=owner.stepRpm(nowMs());
    if(!e&&control.stage!=WifiControl::Off&&control.stage!=WifiControl::Prepared&&control.stage!=WifiControl::Failed)
      e=control.step(nowMs());
    if(!e&&ath10kStarted&&!error)gw_snoc_poll_ce();
    pumping=false;
    if(e&&!error)return fail(e,"transport pump");
    return error;
  }

  // guamp 29/10: the radio uses the board's Wi-Fi address (ADT /chosen/mac-address-wifi0), the one the guamp-wifi-ui
  // placeholder en0 and MobileGestalt already have, as Linux hands ath10k a DT local-mac-address. guamp-wifi-mac=0: the
  // firmware's (WMI service ready). The address is not logged here.
  static void boardAddress() {
    unsigned use=1;PE_parse_boot_argn("guamp-wifi-mac",&use,sizeof(use));
    gw_board_mac_valid=0;
    if(!use){IOLog("GuampWiFi: radio address: the firmware's (guamp-wifi-mac=0)\n");return;}
    bool found=false;
    if(IORegistryEntry *chosen=IORegistryEntry::fromPath("IODeviceTree:/chosen")) {
      OSData *mac=OSDynamicCast(OSData,chosen->getProperty("mac-address-wifi0"));
      if(mac&&mac->getLength()==6) {
        const uint8_t *b=static_cast<const uint8_t *>(mac->getBytesNoCopy());
        if((b[0]|b[1]|b[2]|b[3]|b[4]|b[5])&&!(b[0]&1)){memcpy(gw_board_mac,b,6);found=true;}
      }
      chosen->release();
    }
    gw_board_mac_valid=found;
    IOLog("GuampWiFi: radio address: %s\n",found?"the ADT's (/chosen/mac-address-wifi0, as en0 and MobileGestalt)":
          "no valid /chosen/mac-address-wifi0, the firmware's");
  }

  int configureStation() {
    if(!physicalReady())return NotReady;
    boardAddress();
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
    if(!modemReady||workerLive)return Busy;
    if(mode!=1&&mode!=2)return Invalid;
#if !defined(GUAMP_WIFI_REDWOOD) || !GUAMP_WIFI_REDWOOD
    if(mode==2)return Unsupported;
#endif
    thread_t worker=nullptr;workerLive=true;workerMode=mode;
    if(kernel_thread_start(scanWorker,this,&worker)!=KERN_SUCCESS) {
      workerLive=false;return fail(NoMemory,"scan worker");
    }
    thread_deallocate(worker);return Ok;
  }

  static void traceSink(const char *line) {
    if(!ml_get_interrupts_enabled()){printf("GuampWiFiTrace: %s (nosleep: interrupts off)\n",line);return;}
    const bool canSleep=!ml_at_interrupt_context()&&preemption_enabled();
    IOLog("GuampWiFiTrace: %s%s at %llu ms\n",line,canSleep?"":" (nosleep)",nowMs());
    if(!canSleep)return;
    if(gTraceKpan)guamp_emmc_log_sync();
    if(gTraceMs)IOSleep(gTraceMs);
  }
  static void traceSetup() {
    unsigned n=0,ms=gTraceMs,kpan=0;
    PE_parse_boot_argn("guamp-wifi-trace",&n,sizeof(n));
    if(!n)return;
    PE_parse_boot_argn("guamp-wifi-trace-ms",&ms,sizeof(ms));
    PE_parse_boot_argn("guamp-wifi-trace-kpan",&kpan,sizeof(kpan));
    gTraceMs=ms>5000?5000:ms;gTraceKpan=kpan==1;
    nw_wifi_trace_sink=traceSink;nw_wifi_trace_budget=n;
    IOLog("GuampWiFiTrace: on: budget %u per phase, %u ms after each line, kpan sync %s\n",n,gTraceMs,gTraceKpan?"on":"off");
  }

  void runStation() {
    if(!awaitFirmware()){transportOnly();return;}
    int e=configureStation();if(e){fail(e,"station config");transportOnly();return;}
    // guamp-wifi-attach=0: stop before ath10k (no CE/MMIO access to the WLAN block, no iOS attach); the modem transport
    // (QRTR, RMTFS, TFTP, SERVREG_LOC, SIM) keeps running as after any Wi-Fi outcome.
    unsigned attach=1;PE_parse_boot_argn("guamp-wifi-attach",&attach,sizeof(attach));
    if(!attach){IOLog("GuampWiFi: ath10k start/attach skipped (guamp-wifi-attach=0)\n");transition(TransportOnly);transportOnly();return;}
    traceSetup();
    transition(RadioProbe);
    if(workerMode==1){runScan();transportOnly();return;}
#if defined(GUAMP_WIFI_REDWOOD) && GUAMP_WIFI_REDWOOD
    transition(AwaitNetwork);const uint64_t deadline=nowMs()+120000;
    while(!__atomic_load_n(&networkReady,__ATOMIC_ACQUIRE)) {
      if(pump()){transportOnly();return;}
      if(nowMs()>=deadline){fail(Timeout,"BSD network-ready hook");transportOnly();return;}
      IOSleep(1);
    }
    transition(RadioProbe);ath10kStarted=true;
    if(nw_wifi_trace_budget)nw_wifi_trace("runStation: guamp_xnu_wifi_attach");
    e=guamp_xnu_wifi_attach(&platform,&board);
    if(nw_wifi_trace_budget)nw_wifi_trace("runStation: guamp_xnu_wifi_attach result %d",e);
    if(e){fail(e,"iOS attach");transportOnly();return;}
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
    if(nw_wifi_trace_budget)nw_wifi_trace("runScan: nw_wifi_start_snoc");
    int e=nw_wifi_start_snoc(&platform,&board);
    if(nw_wifi_trace_budget)nw_wifi_trace("runScan: nw_wifi_start_snoc result %d",e);
    if(e){fail(e,"ath10k start");return;}
    uint64_t deadline=nowMs()+30000;
    nw_wifi_status status{};
    for(;;) {
      e=nw_wifi_step();if(e<0||error){fail(error?error:e,"ath10k probe");return;}
      nw_wifi_status_get(&status);
      bool mac=false;for(unsigned i=0;i<6;i++)mac|=status.mac[i]!=0;
      if(status.state==NW_STA_IDLE&&status.channels&&mac&&!(status.mac[0]&1))break;
      if(nowMs()>=deadline){dmaReport("probe timeout");fail(Timeout,"ath10k station ready");return;}
      IOSleep(1);
    }
    IOLog("GuampWiFi: ath10k ready, MAC %02x:%02x:%02x:%02x:%02x:%02x, %u channels\n",
      status.mac[0],status.mac[1],status.mac[2],status.mac[3],status.mac[4],status.mac[5],status.channels);
    dmaReport("after probe");
    scanDone=false;scanResult=NW_RES_FAILED;transition(Scanning);
    e=nw_wifi_scan(nullptr,0);if(e){fail(e,"scan request");return;}
    deadline=nowMs()+35000;
    while(!scanDone) {
      e=nw_wifi_step();if(e<0||error){fail(error?error:e,"scan pump");return;}
      if(nowMs()>=deadline){dmaReport("scan timeout");fail(Timeout,"scan completion");return;}
      IOSleep(1);
    }
    if(scanResult!=NW_RES_OK){dmaReport("scan aborted");fail(Io,"scan aborted");return;}
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
    dmaReport("scan complete");
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
    // rmtfsMem stays allocated: it may be assigned to MSS_MSA/NAV, and Linux never frees it either.
    pageTables.close(true);msaPayload.close(true);owner.stop(true);
  }

  static void dmaReport(const char *when) { extern GuampXnuDma gGuampXnuDma; gGuampXnuDma.report(when); }
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
  static void controlTrace(void *,const char *what,int e,uint32_t a,uint32_t b,uint32_t c) {
    IOLog("GuampWiFi: transport %s: %u/%u/%u (0x%x/0x%x/0x%x) result %d %s at %llu ms\n",
      what,a,b,c,a,b,c,e,errorName(e),nowMs());
  }
  static int rmtfsPartition(void *,uint32_t index,const char *label,uint64_t *bytes) {
    return guamp_rmtfs_partition(index,label,bytes);
  }
  static int rmtfsRead(void *,uint32_t index,uint64_t at,uint8_t *out,uint32_t bytes) {
    return guamp_rmtfs_partition_read(index,at,out,bytes);
  }
  static int rmtfsWrite(void *,uint32_t index,uint64_t at,const uint8_t *in,uint32_t bytes) {
    return guamp_rmtfs_partition_write(index,at,in,bytes);
  }
  static const char *qrtrType(uint32_t t) {
    switch(t) {
      case Qrtr::Data: return "DATA"; case Qrtr::Hello: return "HELLO"; case Qrtr::Bye: return "BYE";
      case Qrtr::NewServer: return "NEW_SERVER"; case Qrtr::DelServer: return "DEL_SERVER"; case Qrtr::DelClient: return "DEL_CLIENT";
      case Qrtr::Resume: return "RESUME_TX"; case Qrtr::Exit: return "EXIT"; case Qrtr::Ping: return "PING";
      case Qrtr::NewLookup: return "NEW_LOOKUP"; case Qrtr::DelLookup: return "DEL_LOOKUP"; case Qrtr::DelProc: return "DEL_PROC";
      default: return "?";
    }
  }
  // One line per QRTR control packet received or handed to GLINK (IPCRTR), per refused packet, and for the first
  // DATA packets to local service ports (QFTP/SERVREG/RMTFS/SIM clients). Addresses are node:port in hex.
  static void qrtrTrace(void *,bool tx,const Qrtr::Header &h,const uint8_t *b,uint32_t n,int e) {
    const char *dir=tx?"tx":"rx";
    if(!b) {
      IOLog("GuampQrtr: %s REFUSED type %u %x:%x -> %x:%x %u bytes: %d %s\n",dir,h.type,h.sn,h.sp,h.dn,h.dp,h.size,e,errorName(e));
      return;
    }
    if(h.type==Qrtr::Data&&h.dp!=Qrtr::Ctrl) {
      IOLog("GuampQrtr: %s DATA %x:%x -> %x:%x %u bytes, QMI type %u txn %u msg 0x%x\n",dir,h.sn,h.sp,h.dn,h.dp,n,
            n>=7?b[0]:0u,n>=7?unsigned(le16(b+1)):0u,n>=7?unsigned(le16(b+3)):0u);
      return;
    }
    uint32_t w[5]={};
    for(uint32_t i=0;i<5;i++) if(n>=4*(i+1)) w[i]=le32(b+4*i);
    const char *cmd=qrtrType(w[0]);
    if(w[0]==Qrtr::NewServer||w[0]==Qrtr::DelServer)
      IOLog("GuampQrtr: %s %s %x:%x -> %x:%x service %u (0x%x) version %u instance %u node %u port 0x%x: %d %s\n",
            dir,cmd,h.sn,h.sp,h.dn,h.dp,w[1],w[1],w[2]&0xffu,w[2]>>8,w[3],w[4],e,errorName(e));
    else if(w[0]==Qrtr::NewLookup||w[0]==Qrtr::DelLookup)
      IOLog("GuampQrtr: %s %s %x:%x -> %x:%x service %u (0x%x) version %u instance %u: %d %s\n",
            dir,cmd,h.sn,h.sp,h.dn,h.dp,w[1],w[1],w[2]&0xffu,w[2]>>8,e,errorName(e));
    else
      IOLog("GuampQrtr: %s %s %x:%x -> %x:%x (header type %u, %u bytes) node %u port 0x%x: %d %s\n",
            dir,cmd,h.sn,h.sp,h.dn,h.dp,h.type,n,w[1],w[2],e,errorName(e));
  }
  static void qftpTrace(void *,const char *what,const char *name,int e,uint64_t a,uint64_t b,uint64_t c) {
    IOLog("GuampQftp: %s%s%s: %llu/%llu/%llu (0x%llx/0x%llx/0x%llx) result %d %s at %llu ms\n",
      name?name:"",name?" ":"",what,a,b,c,a,b,c,e,errorName(e),nowMs());
  }
  static void servregTrace(void *,const char *what,const char *name,int e,uint64_t a,uint64_t b,uint64_t c) {
    IOLog("GuampServreg: %s%s%s: %llu/%llu/%llu result %d %s at %llu ms\n",
      name?name:"",name?" ":"",what,a,b,c,e,errorName(e),nowMs());
  }
  static void rmtfsTrace(void *,const char *what,const char *name,int e,uint64_t a,uint64_t b,uint64_t c) {
    IOLog("GuampRmtfs: %s%s%s: %llu/%llu/%llu (0x%llx/0x%llx/0x%llx) result %d %s at %llu ms\n",
      name?name:"",name?" ":"",what,a,b,c,a,b,c,e,errorName(e),nowMs());
  }
  // Identifiers stay in the registry (GuampSim service); the log shows only their last 4 digits.
  static const char *tail4(const char *s) { size_t n=strlen(s); return n>4?s+n-4:s; }
  static void simSnapshot(void *,const guampsim::SimProbe &s) {
    GuampSimInfo info;guampsim::fillInfo(s,&info);
    IOLog("GuampSim: snapshot %u: SIM state %u app %u PIN/PUK retries %u/%u, modem mode %u, registration %u radio %u "
          "PLMN %u/%u \"%s\", ICCID ..%s IMSI ..%s IMEI ..%s MSISDN %s SPN \"%s\" revision \"%s\"\n",
          info.snapshots,info.state,info.appType,info.pin1Retries,info.puk1Retries,info.operatingMode,info.registration,
          info.radio,info.mcc,info.mnc,info.plmn,tail4(info.iccid),tail4(info.imsi),tail4(info.imei),
          info.msisdn[0]?"read":"none",info.spn,info.revision);
    guamp_sim_publish(&info);
  }
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
