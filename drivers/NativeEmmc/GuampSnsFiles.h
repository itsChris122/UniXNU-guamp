// SPDX-License-Identifier: BSD-2-Clause-Patent
// Files of the ADSP sensors PD for the FastRPC listener (GuampFastRpcCore.h Vfs). Pure code, host-tested by
// tests/fastrpc_host.cpp against the backup persist image.
//   persist   the ext4 subtree /sensors of the persist partition, read-only, mounted where Android mounts it
//             (/mnt/vendor/persist/sensors: registry/registry/<group> json files, registry/sns_reg_version,
//             sensors_list.txt, calibration); the files are copied to RAM once (about 300 KB)
//   blob      optional copy of /vendor/etc/sensors (sns_reg_config, config/*.json) carried in the ADT property
//             /chosen/guamp-sensors-config (scripts/guamp-sensors-config.py builds it from the vendor backup):
//             "GSNSCFG1", u32 count, then per file u32 path length, u32 data length, path, data (each padded to 4)
//   synthetic what the registry reads besides: sns_reg_config (when the blob has none; version 4 = the persist registry,
//             so the PD keeps the registry and does not parse the json again), the /sys/devices/soc0 answers of
//             bengal (soc_id 417, hw_platform IDP: the bengal_*.json of this phone list IDP), the config directory
#pragma once
#include "GuampFastRpcCore.h"
#include "GuampExt4.inc"

namespace guamp_rpc {

constexpr char kPersistMount[] = "/mnt/vendor/persist";

// A file the caller allocated with vfs.hooks.alloc (n bytes): the Vfs owns it from now on
inline bool adopt(Vfs &vfs, const char *path, uint8_t *data, uint32_t n) {
  Vfs::File *f = vfs.add(path, data, n, false);
  if (!f) return false;
  f->owned = data != nullptr;
  f->cap = n;
  vfs.ramBytes += n;
  return true;
}

inline bool joinPath(char *out, uint32_t cap, const char *a, const char *b, uint32_t bLen) {
  uint32_t n = 0;
  for (; a[n]; ++n) { if (n + 1 >= cap) return false; out[n] = a[n]; }
  if (n + 1 >= cap) return false;
  out[n++] = '/';
  for (uint32_t i = 0; i < bLen; ++i) { if (n + 1 >= cap) return false; out[n++] = b[i]; }
  out[n] = 0;
  return true;
}

struct TreeStats { uint32_t files = 0, dirs = 0, bytes = 0, skipped = 0; int error = 0; };

// Copies the ext4 directory `from` (e.g. "/sensors") and everything below it to `to` + from in the Vfs. Directory
// entries are collected in batches before any file is read: ext4::Fs keeps one directory block in its scratch.
inline TreeStats loadExt4(ext4::Fs &fs, const char *from, const char *to, Vfs &vfs, uint32_t maxFile = 256 * 1024) {
  TreeStats st;
  struct Dir { uint32_t inode; char path[Vfs::kPath]; };
  struct Entry { uint32_t inode; uint8_t type; char name[104]; };
  constexpr uint32_t kDirs = 24, kBatch = 48;
  Dir *dirs = static_cast<Dir *>(vfs.hooks.alloc(vfs.hooks.ctx, kDirs * sizeof(Dir)));
  Entry *batch = static_cast<Entry *>(vfs.hooks.alloc(vfs.hooks.ctx, kBatch * sizeof(Entry)));
  if (!dirs || !batch) {
    if (dirs) vfs.hooks.free(vfs.hooks.ctx, dirs, kDirs * sizeof(Dir));
    if (batch) vfs.hooks.free(vfs.hooks.ctx, batch, kBatch * sizeof(Entry));
    st.error = ext4::kErrIo;
    return st;
  }
  ext4::Inode root;
  st.error = fs.lookupPath(from, &root);
  uint32_t head = 0, tail = 0;
  if (!st.error && !root.isDir()) st.error = ext4::kErrNotDir;
  if (!st.error) {
    dirs[tail].inode = root.number;
    if (!joinPath(dirs[tail].path, Vfs::kPath, to, from[0] == '/' ? from + 1 : from, strLen(from[0] == '/' ? from + 1 : from, 200)))
      st.error = ext4::kErrRange;
    else { vfs.mkdir(dirs[tail].path); ++tail; ++st.dirs; }
  }
  while (!st.error && head < tail) {
    const Dir d = dirs[head++];
    ext4::Inode dir;
    if ((st.error = fs.readInode(d.inode, &dir))) break;
    for (uint32_t skip = 0;; skip += kBatch) {
      uint32_t seen = 0, got = 0;
      st.error = fs.forEachEntry(dir, [&](const char *name, uint32_t len, uint32_t ino, uint8_t type) {
        if ((len == 1 && name[0] == '.') || (len == 2 && name[0] == '.' && name[1] == '.')) return false;
        if (seen++ < skip) return false;
        if (len >= sizeof(batch[0].name)) { ++st.skipped; return false; }
        batch[got].inode = ino;
        batch[got].type = type;
        for (uint32_t i = 0; i < len; ++i) batch[got].name[i] = name[i];
        batch[got].name[len] = 0;
        return ++got == kBatch;
      });
      if (st.error) break;
      for (uint32_t i = 0; i < got && !st.error; ++i) {
        char path[Vfs::kPath];
        if (!joinPath(path, Vfs::kPath, d.path, batch[i].name, strLen(batch[i].name, 104))) { ++st.skipped; continue; }
        ext4::Inode ino;
        if ((st.error = fs.readInode(batch[i].inode, &ino))) break;
        if (ino.isDir()) {
          if (tail < kDirs) {
            dirs[tail].inode = ino.number;
            for (uint32_t k = 0; k < Vfs::kPath; ++k) dirs[tail].path[k] = path[k];
            vfs.mkdir(path);
            ++tail; ++st.dirs;
          } else ++st.skipped;
          continue;
        }
        if (!ino.isFile() || ino.size > maxFile) { ++st.skipped; continue; }
        const uint32_t n = (uint32_t)ino.size;
        uint8_t *data = n ? static_cast<uint8_t *>(vfs.hooks.alloc(vfs.hooks.ctx, n)) : nullptr;
        if (n && !data) { st.error = ext4::kErrIo; break; }
        if (n && (st.error = fs.readFile(ino, 0, data, n))) { vfs.hooks.free(vfs.hooks.ctx, data, n); break; }
        if (!adopt(vfs, path, data, n)) { if (data) vfs.hooks.free(vfs.hooks.ctx, data, n); ++st.skipped; continue; }
        ++st.files;
        st.bytes += n;
      }
      if (got < kBatch) break;
    }
  }
  vfs.hooks.free(vfs.hooks.ctx, dirs, kDirs * sizeof(Dir));
  vfs.hooks.free(vfs.hooks.ctx, batch, kBatch * sizeof(Entry));
  return st;
}

// The ADT blob: files borrowed in place (the property stays alive); false on a malformed blob (files added so far stay)
inline bool loadBlob(const uint8_t *p, uint32_t n, Vfs &vfs, uint32_t *files) {
  *files = 0;
  if (!p || n < 12) return false;
  static const char kMagic[8] = {'G', 'S', 'N', 'S', 'C', 'F', 'G', '1'};
  for (uint32_t i = 0; i < 8; ++i) if (p[i] != (uint8_t)kMagic[i]) return false;
  const uint32_t count = ld32(p + 8);
  uint32_t at = 12;
  for (uint32_t k = 0; k < count; ++k) {
    if (n - at < 8) return false;
    const uint32_t pl = ld32(p + at), dl = ld32(p + at + 4);
    at += 8;
    if (!pl || pl >= Vfs::kPath || pl > n - at) return false;
    char path[Vfs::kPath];
    for (uint32_t i = 0; i < pl; ++i) path[i] = (char)p[at + i];
    path[pl] = 0;
    at = align(at + pl, 4);
    if (at > n || dl > n - at) return false;
    if (!vfs.add(path, p + at, dl, false)) return false;
    at = align(at + dl, 4);
    if (at > n && k + 1 < count) return false;
    ++*files;
  }
  return true;
}

// sns_reg_config of guamp's vendor (/vendor/etc/sensors/sns_reg_config, version 4 like the persist registry)
constexpr char kRegConfig[] =
    "version=4\n"
    "file=hw_platform=/sys/devices/soc0/hw_platform\n"
    "file=platform_subtype=/sys/devices/soc0/platform_subtype\n"
    "file=platform_subtype_id=/sys/devices/soc0/platform_subtype_id\n"
    "file=platform_version=/sys/devices/soc0/platform_version\n"
    "file=soc_id=/sys/devices/soc0/soc_id\n"
    "file=revision=/sys/devices/soc0/revision\n"
    "file=output=/mnt/vendor/persist/sensors/registry/registry\n"
    "property=ro.vendor.hw.device=/mnt/vendor/persist/sensors/registry/hw_device\n"
    "property=ro.vendor.hw.hwrev=/mnt/vendor/persist/sensors/registry/hw_rev\n"
    "property=ro.vendor.hw.revision=/mnt/vendor/persist/sensors/registry/hw_revision\n";

inline uint32_t addSynthetic(Vfs &vfs) {
  struct S { const char *path, *text; };
  static const S kFiles[] = {
      {"/sys/devices/soc0/hw_platform", "IDP\n"}, {"/sys/devices/soc0/soc_id", "417\n"},
      {"/sys/devices/soc0/platform_subtype", "Unknown\n"}, {"/sys/devices/soc0/platform_subtype_id", "0\n"},
      {"/sys/devices/soc0/platform_version", "65536\n"}, {"/sys/devices/soc0/revision", "1.0\n"},
  };
  uint32_t added = 0;
  char p[Vfs::kPath];
  if (Vfs::normalize("/vendor/etc/sensors/sns_reg_config", p) && !vfs.find(p) &&
      vfs.add(p, reinterpret_cast<const uint8_t *>(kRegConfig), sizeof(kRegConfig) - 1, false))
    ++added;
  for (const S &s : kFiles)
    if (Vfs::normalize(s.path, p) && !vfs.find(p) &&
        vfs.add(p, reinterpret_cast<const uint8_t *>(s.text), strLen(s.text, 64), false))
      ++added;
  if (!vfs.isDir("/vendor/etc/sensors/config")) vfs.mkdir("/vendor/etc/sensors/config");
  if (!vfs.isDir("/mnt/vendor/persist/sensors/registry/registry")) vfs.mkdir("/mnt/vendor/persist/sensors/registry/registry");
  return added;
}

}  // namespace guamp_rpc
