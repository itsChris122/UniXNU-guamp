// SPDX-License-Identifier: BSD-2-Clause-Patent
// Opt-in authenticated modem (MPSS) start. Included inside namespace wifiprep after its readers (GuampWifiPrep.inc).
// Since the merge of the Codex Wi-Fi provider this is a thin glue over the ONE generic PAS loader shared with the ADSP
// (GuampPasCore.inc / GuampPas.inc: placement, original metadata, INIT_IMAGE, MEM_SETUP, segments, AUTH_AND_RESET,
// SHUTDOWN + cleanup) and the ONE SCM transport (GuampScm.inc: interrupted calls resumed with the returned x0-x6,
// discovery as is_scm_armv8). What stays modem-specific: the carveout and PAS id, the stopped-peer and ready checks
// over SMP2P 428, and ModemBootHooks, the hand-off that feeds the Wi-Fi bootstrap (prepare before any RAM write,
// started after AUTH_AND_RESET instead of the plain ready observation, shutdown with the actual PAS SHUTDOWN result).
// Disk remains read-only. Only the reserved modem RAM is written. TrustZone authenticates the original signed image.
namespace modem {
constexpr uint64_t kBase=0x4ab00000ULL,kBytes=0x6900000ULL;   // modem_region of the guamp DT, below XNU RAM
constexpr unsigned kPas=4;                                       // qcom,pas-id of qcom,mss
}

// Optional owner handoff for the integrated Wi-Fi boot path. prepare runs only
// after image placement and the stopped-PAS check, before any reserved-RAM
// write. started runs only after AUTH_AND_RESET succeeds; it must observe real
// remote readiness while pumping its SMP2P/GLINK state. Neither hook may retain
// pointers to the temporary MDT/chunk buffers supplied to this loader.
struct ModemBootHooks {
  void *ctx;
  int (*prepare)(void *);
  int (*started)(void *);
  void (*shutdown)(void *,bool);
};

static const ModemBootHooks *gModemHooks;   // valid only during modemBoot (prep thread, one call at a time)
static bool modemHookStopped(void *) {return modemStopped();}
static int modemHookPrepare(void *) {return gModemHooks->prepare(gModemHooks->ctx);}
static int modemHookReady(void *) {
  if(gModemHooks && gModemHooks->started) {
    const int e=gModemHooks->started(gModemHooks->ctx);
    if(e)IOLog("GuampModem: board readiness failed %d\n",e);
    return e?pas::kNotReady:pas::kReady;
  }
  return modemAwaitReady()?pas::kReady:pas::kNotReady;
}
static void modemHookCleanup(void *,bool stopped) {gModemHooks->shutdown(gModemHooks->ctx,stopped);}

static bool modemBoot(ext4::Fs &fs,uint8_t *mdt,uint8_t *chunk,bool verified,
                      const ModemBootHooks *hooks=nullptr,bool startForWifi=false) {
  unsigned enabled=0;
  if(startForWifi)enabled=1; // caller has already checked its explicit Wi-Fi opt-in
  else if(!PE_parse_boot_argn("guamp-modem-start",&enabled,sizeof(enabled)) || (enabled!=1 && enabled!=2))return false;
  const pas::Peripheral peripheral={"GuampModem",modem::kPas,modem::kBase,modem::kBytes,"/image","modem"};
  fw::Image *image=static_cast<fw::Image *>(IOMallocZero(sizeof(fw::Image)));
  if(!image)return false;
  ext4::Inode inode;
  int parsed=fs.lookupPath("/image/modem.mdt",&inode);
  if(!parsed && inode.size<=fw::kMdtMax/2) parsed=fw::parse(mdt,static_cast<uint32_t>(inode.size),image);
  else parsed=fw::kErrFormat;
  if(!parsed && image->hashWhere==3) {
    uint32_t hashAt=(static_cast<uint32_t>(inode.size)+63)&~63u;
    if(image->hashSegBytes>fw::kMdtMax-hashAt)parsed=fw::kErrFormat;
    else image->hashSeg=mdt+hashAt; // original bytes already read/verified by fw::verify
  }
  bool booted=false;
  if(parsed) {
    IOLog("GuampModem: metadata not usable (%d); no RAM written\n",parsed);
  } else {
    scm::Transport transport={guampScmInstruction,[](void *,uint32_t ms) {IOSleep(ms);},nullptr,true};
    pas::Hooks h;
    h.stopped=modemHookStopped;
    h.awaitReady=modemHookReady;
    if(hooks && hooks->prepare)h.prepare=modemHookPrepare;
    if(hooks && hooks->shutdown)h.cleanup=modemHookCleanup;
    gModemHooks=hooks;
    const int rc=pas::boot(peripheral,h,transport,fs,mdt,static_cast<uint32_t>(inode.size),*image,chunk,kChunkBytes,
                           verified,enabled,guamp_phys_base,guamp_phys_size);
    gModemHooks=nullptr;
    booted=rc==pas::kUp;
    if(booted)IOLog("GuampModem: firmware ready%s\n",hooks&&hooks->started?"; board owner active":"; WLAN transport board adapter not attached");
    else IOLog("GuampModem: RESULT %d (%s)\n",rc,rc==pas::kQueried?"SCM/PAS capabilities checked only; modem not started":"not started");
  }
  IOFree(image,sizeof(*image));
  return booted;
}
