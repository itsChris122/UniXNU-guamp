// SPDX-License-Identifier: BSD-2-Clause-Patent
// guamp Adreno 610 zap shader through TrustZone PAS: the zap callback GpuService.inc hands to NativeAdreno (GuampAdreno.h
// GuampAdrenoStart::zap), called by GuampAdrenoProbe.cpp after CP_ME_INIT when guamp-adreno-zap=1|2. Included by NativeEmmc.cpp
// after GuampAdsp.inc (SCM transport, PAS loader, ext4, firmware verifier, partition helpers). The same sequence as mainline
// adreno_zap_shader_load (qcom_mdt_load + qcom_scm_pas_auth_and_reset) and kgsl's subsystem_get("a610_zap") on the guamp kernel
// (pil-tz-generic: INIT_IMAGE, MEM_SETUP, the .bNN segments, AUTH_AND_RESET), and UniXNU-tb128fu GpuProbe zapLoad:
//   - the ORIGINAL signed a610_zap.mdt / .b00-.b02 of the modem_<slot> partition (/image, the Android /vendor/firmware_mnt),
//     copied byte for byte; TrustZone alone authenticates it; every PAS status is honoured (GuampPas.inc)
//   - no proxy votes and no ready signal: the GPU is already powered by the caller, and success is AUTH_AND_RESET returning 0
//     (mainline waits for nothing either); CP_SET_SECURE_MODE 0 is the probe's job
// Facts (guamp device tree ~/moto/guamp.dts, bengal.dtsi + guamp dtbo):
//   qcom,kgsl-hyp (qcom,pil-tz-generic): qcom,pas-id 0x0d, qcom,firmware-name "a610_zap", qcom,mas-crypto (bus vote only);
//   reserved-memory gpu_region@53e15000 size 0x2000 (removed-dma-pool, no-map; __symbols__ pil_gpu_mem), inside UEFI
//   "PIL Reserved" 0x4ab00000-0x53f00000, apart from the ADSP (0x52200000 + 0x1c00000) and the modem (0x4ab00000 + 0x6900000)
//   carveouts and from XNU RAM (0x80000000+, checked at run time against gPhysBase/gPhysSize).
//   a610_zap.mdt: 6860 bytes (ELF32 headers + the hash segment), one loadable relocatable segment (p_paddr 0x5000, 0x830 bytes,
//   a610_zap.b02): region 0x53e15000 + 0x1000, metadata at region_start running into the second page of the carveout.
// Once per boot. Disk read-only; only gpu_region is written.

namespace gpuzap {

constexpr uint64_t kBase = 0x53e15000, kBytes = 0x2000;
constexpr unsigned kPasId = 13;
static_assert(kBase >= adsp::kBase + adsp::kBytes || kBase + kBytes <= adsp::kBase, "zap region overlaps the ADSP carveout");
static_assert(kBase >= 0x4ab00000ULL + 0x6900000ULL, "zap region overlaps the modem carveout");
static_assert(kBase + kBytes <= 0x80000000ULL, "zap region inside the XNU RAM window");

// GuampAdrenoZapLoad: 0 authenticated start accepted (pas::kUp), 1 queries only (pas::kQueried), <0 refused (pas:: codes).
int load(void *, unsigned mode) {
  static bool done = false;
  if (done) {
    IOLog("GuampAdreno: zap: already run in this boot; not repeated\n");
    return pas::kErrPeer;
  }
  done = true;
  IOLog("GuampAdreno: zap: mode %u, PAS id %u, carveout %llx + %llx, XNU RAM %lx + %lx\n", mode, kPasId, kBase, kBytes,
        guamp_phys_base, guamp_phys_size);
  if (pas::overlap(kBase, kBytes, guamp_phys_base, guamp_phys_size)) {
    IOLog("GuampAdreno: zap: carveout overlaps XNU RAM: refused\n");
    return pas::kErrPlace;
  }
  if (!gCard.online || !gCard.gate) {
    IOLog("GuampAdreno: zap: eMMC not online\n");
    return pas::kErrPrepare;
  }
  const pas::Peripheral peripheral = {"GuampAdreno: zap", kPasId, kBase, kBytes, "/image", "a610_zap"};
  char part[8] = {'m', 'o', 'd', 'e', 'm', '_', wifiprep::activeSlot(), 0};
  wifiprep::Window window = {};
  uint64_t attributes = 0;
  if (!wifiprep::findPartition(part, &window.first, &window.blocks, &attributes)) {
    IOLog("GuampAdreno: zap: partition %s not found\n", part);
    return pas::kErrPrepare;
  }
  int rc = pas::kErrMemoryAlloc;
  uint8_t *scratch = static_cast<uint8_t *>(IOMallocZero(ext4::kScratchBytes));
  uint8_t *mdt = static_cast<uint8_t *>(IOMallocZero(fw::kMdtMax));
  uint8_t *chunk = static_cast<uint8_t *>(IOMallocZero(wifiprep::kChunkBytes));
  ext4::Fs *fs = static_cast<ext4::Fs *>(IOMallocZero(sizeof(ext4::Fs)));
  fw::Result *res = static_cast<fw::Result *>(IOMallocZero(sizeof(fw::Result)));
  fw::Image *image = static_cast<fw::Image *>(IOMallocZero(sizeof(fw::Image)));
  if (scratch && mdt && chunk && fs && res && image) {
    const int mounted = fs->mount(wifiprep::readWindow, &window, window.blocks, scratch);
    if (mounted) {
      IOLog("GuampAdreno: zap: %s ext4 mount: %s\n", part, ext4::errorName(mounted));
      rc = pas::kErrPrepare;
    } else {
      fw::verify(*fs, peripheral.dir, peripheral.firmware, mdt, chunk, wifiprep::kChunkBytes, res);
      IOLog("GuampAdreno: zap: %s/%s firmware on %s: error %d, %u verified / %u failed / %u empty of %u, MBN v%u, %llu bytes, "
            "mask %llx\n", peripheral.dir, peripheral.firmware, part, res->error, res->verified, res->failed, res->empty,
            res->segments, res->mbnVersion, res->bytes, res->failedMask);
      const bool verified = !res->error && !res->failed && res->verified > 0;
      char path[128];
      ext4::Inode inode;
      int parsed = fw::joinName(path, sizeof(path), peripheral.dir, peripheral.firmware, ".mdt", -1) ? fs->lookupPath(path, &inode)
                                                                                                    : fw::kErrFormat;
      if (!parsed && inode.size <= fw::kMdtMax / 2) parsed = fw::parse(mdt, static_cast<uint32_t>(inode.size), image);
      else if (!parsed) parsed = fw::kErrTooBig;
      if (!parsed && image->hashWhere == 3) {
        const uint32_t hashAt = (static_cast<uint32_t>(inode.size) + 63) & ~63u;   // where fw::verify read it
        if (image->hashSegBytes > fw::kMdtMax - hashAt) parsed = fw::kErrFormat;
        else image->hashSeg = mdt + hashAt;
      }
      if (parsed) {
        IOLog("GuampAdreno: zap: metadata not usable (%d); nothing started\n", parsed);
        rc = pas::kErrPrepare;
      } else {
        scm::Transport transport = {guampScmInstruction, [](void *, uint32_t ms) { IOSleep(ms); }, nullptr, true};
        pas::Hooks hooks;   // no peer state, no proxy votes, no ready signal (see above)
        rc = pas::boot(peripheral, hooks, transport, *fs, mdt, static_cast<uint32_t>(inode.size), *image, chunk,
                       wifiprep::kChunkBytes, verified, mode == 1 ? 1 : 2, guamp_phys_base, guamp_phys_size);
      }
    }
  } else {
    IOLog("GuampAdreno: zap: out of memory\n");
  }
  IOLog("GuampAdreno: zap: PAS result %d (%s)\n", rc,
        rc == pas::kUp ? "zap shader authenticated and started by TrustZone" : rc == pas::kQueried ? "queries only"
                                                                                                  : "not started, cleaned up");
  if (scratch) IOFree(scratch, ext4::kScratchBytes);
  if (mdt) IOFree(mdt, fw::kMdtMax);
  if (chunk) IOFree(chunk, wifiprep::kChunkBytes);
  if (fs) IOFree(fs, sizeof(ext4::Fs));
  if (res) IOFree(res, sizeof(fw::Result));
  if (image) IOFree(image, sizeof(fw::Image));
  return rc;
}

}  // namespace gpuzap
