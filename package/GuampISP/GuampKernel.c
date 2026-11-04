// GuampKernel.c - user client of the kernel camera service AppleH6CamIn (drivers/NativeEmmc/GuampCamIos.inc).
// IOKit is resolved with dlsym: the iOS SDK has no public IOKit headers, and the functions used here are stable C API.
#include <dlfcn.h>
#include <mach/mach.h>
#include <string.h>
#include "GuampISP.h"

typedef unsigned int io_object_t_;
static struct {
  int loaded;
  mach_port_t main;
  CFMutableDictionaryRef (*matching)(const char *);
  io_object_t_ (*getMatching)(mach_port_t, CFDictionaryRef);
  kern_return_t (*open)(io_object_t_, task_port_t, uint32_t, io_object_t_ *);
  kern_return_t (*close)(io_object_t_);
  kern_return_t (*release)(io_object_t_);
  kern_return_t (*call)(io_object_t_, uint32_t, const uint64_t *, uint32_t, const void *, size_t, uint64_t *, uint32_t *,
                        void *, size_t *);
  kern_return_t (*map)(io_object_t_, uint32_t, task_port_t, mach_vm_address_t *, mach_vm_size_t *, uint32_t);
  kern_return_t (*unmap)(io_object_t_, uint32_t, task_port_t, mach_vm_address_t);
} io;

static int ioLoad(void) {
  if (io.loaded) return io.loaded > 0 ? 0 : -1;
  void *h = dlopen("/System/Library/Frameworks/IOKit.framework/IOKit", RTLD_LAZY | RTLD_LOCAL);
  io.matching = h ? dlsym(h, "IOServiceMatching") : NULL;
  io.getMatching = h ? dlsym(h, "IOServiceGetMatchingService") : NULL;
  io.open = h ? dlsym(h, "IOServiceOpen") : NULL;
  io.close = h ? dlsym(h, "IOServiceClose") : NULL;
  io.release = h ? dlsym(h, "IOObjectRelease") : NULL;
  io.call = h ? dlsym(h, "IOConnectCallMethod") : NULL;
  io.map = h ? dlsym(h, "IOConnectMapMemory64") : NULL;
  io.unmap = h ? dlsym(h, "IOConnectUnmapMemory64") : NULL;
  io.main = MACH_PORT_NULL;   // kIOMainPortDefault
  io.loaded = (io.matching && io.getMatching && io.open && io.close && io.release && io.call && io.map && io.unmap) ? 1 : -1;
  return io.loaded > 0 ? 0 : -1;
}

int GKernelOpen(GKernel *k) {
  memset(k, 0, sizeof(*k));
  if (ioLoad()) { GISP_LOG("IOKit symbols missing"); return -1; }
  io_object_t_ service = io.getMatching(io.main, io.matching(GUAMP_CAMIOS_SERVICE_CLASS));
  if (!service) { GISP_LOG("no %s service", GUAMP_CAMIOS_SERVICE_CLASS); return -1; }
  kern_return_t kr = io.open(service, mach_task_self(), GUAMP_CAMIOS_CONNECT_TYPE, &k->connection);
  io.release(service);
  if (kr) { GISP_LOG("IOServiceOpen failed 0x%x", kr); k->connection = 0; return -1; }
  size_t sz = sizeof(k->info);
  if (GKernelCall(k, kGuampCamSelGetInfo, NULL, 0, &k->info, &sz) || k->info.abiVersion != GUAMP_CAMIOS_ABI_VERSION) {
    GISP_LOG("GetInfo failed or ABI %u != %u", k->info.abiVersion, GUAMP_CAMIOS_ABI_VERSION);
    GKernelClose(k);
    return -1;
  }
  return 0;
}

void GKernelClose(GKernel *k) {
  if (k->connection) io.close(k->connection);
  k->connection = 0;
}

int GKernelCall(GKernel *k, uint32_t selector, const uint64_t *in, uint32_t inCount, void *structOut, size_t *structOutSize) {
  if (!k->connection) return -1;
  uint32_t outCount = 0;
  kern_return_t kr = io.call(k->connection, selector, in, inCount, NULL, 0, NULL, &outCount, structOut, structOutSize);
  return kr ? (int)kr : 0;
}

static uint32_t slotType(uint32_t sensor, uint32_t slot) { return kGuampCamMemRing + sensor * kGuampCamMemSlotsPerSensor + slot; }

const uint8_t *GKernelMapSlot(GKernel *k, uint32_t sensor, uint32_t slot, size_t *size) {
  mach_vm_address_t addr = 0;
  mach_vm_size_t len = 0;
  if (!k->connection) return NULL;
  kern_return_t kr = io.map(k->connection, slotType(sensor, slot), mach_task_self(), &addr, &len, 1 /* kIOMapAnywhere */);
  if (kr) { GISP_LOG("slot map sensor %u slot %u failed 0x%x", sensor, slot, kr); return NULL; }
  *size = (size_t)len;
  return (const uint8_t *)(uintptr_t)addr;
}

void GKernelUnmapSlot(GKernel *k, uint32_t sensor, uint32_t slot, const uint8_t *base) {
  if (k->connection && base) io.unmap(k->connection, slotType(sensor, slot), mach_task_self(), (mach_vm_address_t)(uintptr_t)base);
}
