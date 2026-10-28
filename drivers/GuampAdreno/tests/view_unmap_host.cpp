/* Host test of AGXDeviceUserClient::dropViews (drivers/NativeAdreno/NativeAdrenoPublish.cpp) against XNU's map reference rules.
 * The loop between "if(!view)return;" and "view->release();" is taken verbatim from the source (VIEW_DROP_LOOP, extracted by run-tests.sh)
 * and runs over a model of the IOKit objects with the reference counting of xnu-10063 (iokit/Kernel/IOUserClient.cpp,
 * IOMemoryDescriptor.cpp):
 *  - a user mapping (is_io_connect_map_memory_into_task into the calling task) is held by the client's `mappings` set and by the
 *    descriptor's own _mappings list; IOMemoryMap::taggedRelease frees the map at the last reference outside that list, and freeing
 *    it unmaps the user range (the user PTEs go);
 *  - removeMappingForDescriptor returns the map retained, after taking it out of `mappings`.
 * guamp kernel-gpu12 (28/10): Preferences exited with BO views mapped, clientClose -> dropViews(0) -> releaseAll freed the BO pages while a
 * user PTE still pointed at one: pmap_assert_free "User CPU mapping". EXPECT_LEAK=1 checks that the pinned loop leaves the PTE (the bug),
 * EXPECT_LEAK=0 that the patched loop removes every PTE before the pages could be freed. */
#include <cstdio>
#include <cstdint>
#include <vector>
#include <algorithm>

#ifndef EXPECT_LEAK
#error "build with -DEXPECT_LEAK=0|1"
#endif

static int live_ptes = 0, live_maps = 0;
struct IOMemoryDescriptor;
struct IOMemoryMap {
    int rc = 1;
    IOMemoryDescriptor *md = nullptr;
    bool user_pte = false;
    void retain() { rc = rc + 1; }
    void release();
};
struct IOMemoryDescriptor {
    int rc = 1;
    std::vector<IOMemoryMap *> mappings_;   /* _mappings: each entry holds one reference */
    bool freed = false;
    void retain() { rc = rc + 1; }
    void release() { rc = rc - 1; if (rc == 0) freed = true; }
    /* createMappingInTask: new map (1, the caller's) plus the descriptor's list reference; the map retains the descriptor. A second
     * kIOMapAnywhere request from the same task gets the existing map retained (IOMemoryDescriptor::makeMapping, copyCompatible). */
    IOMemoryMap *map()
    {
        if (!mappings_.empty()) { mappings_.front()->retain(); return mappings_.front(); }
        IOMemoryMap *m = new IOMemoryMap;
        m->md = this; retain();
        m->user_pte = true; live_ptes = live_ptes + 1; live_maps = live_maps + 1;
        m->retain(); mappings_.push_back(m);
        return m;
    }
};
/* IOMemoryMap::taggedRelease(tag, 2): free at the last reference outside the descriptor's list; free() unmaps and leaves the list. */
void IOMemoryMap::release()
{
    rc = rc - 1;
    if (rc < 2) {
        if (user_pte) { user_pte = false; live_ptes = live_ptes - 1; }
        auto &l = md->mappings_;
        l.erase(std::remove(l.begin(), l.end(), this), l.end());
        md->release();
        live_maps = live_maps - 1;
        delete this;
    }
}
static struct { volatile uint32_t forced_unmaps; } native_adreno_metal_diag;

struct UserClientModel {
    std::vector<IOMemoryMap *> mappings;   /* OSSet: setObject retains, removeObject releases */
    /* is_io_connect_map_memory_into_task, into_task == current_task(): kept with the user client. */
    void connectMap(IOMemoryDescriptor *view)
    {
        IOMemoryMap *m = view->map();
        if (std::find(mappings.begin(), mappings.end(), m) == mappings.end()) { m->retain(); mappings.push_back(m); }
        m->release();
    }
    /* IOUserClient::removeMappingForDescriptor, as in xnu. */
    IOMemoryMap *removeMappingForDescriptor(IOMemoryDescriptor *mem)
    {
        for (IOMemoryMap *m : mappings) {
            if (m->md == mem) {
                m->retain();
                mappings.erase(std::find(mappings.begin(), mappings.end(), m));
                m->release();
                return m;
            }
        }
        return nullptr;
    }
    void dropOne(IOMemoryDescriptor *view)
    {
#include VIEW_DROP_LOOP
        view->release();
    }
};

int main()
{
    int fails = 0;
    UserClientModel client;
    IOMemoryDescriptor *view[3];
    for (auto &v : view) { v = new IOMemoryDescriptor; v->retain(); }   /* the view cache's reference (clientMemoryForType) */
    /* Preferences maps three BO views, one of them twice (IOConnectMapMemory64 answered from the view cache), and exits without unmapping. */
    client.connectMap(view[0]); client.connectMap(view[1]); client.connectMap(view[1]); client.connectMap(view[2]);
    for (auto &v : view) v->release();   /* the creator's reference: the view cache's and the maps' remain */
    /* clientClose -> dropViews(0): every cached view goes; releaseAll then frees the BOs' pages. */
    for (auto &v : view) client.dropOne(v);
    const int ptes = live_ptes;
    std::printf("user PTEs left when releaseAll frees the pages: %d (maps %d, forced_unmaps %u)\n", ptes, live_maps,
                unsigned(native_adreno_metal_diag.forced_unmaps));
    if (EXPECT_LEAK ? ptes == 0 : ptes != 0) { std::printf("FAIL: expected %s\n", EXPECT_LEAK ? "the pinned loop to leak" : "no PTE left"); fails = fails + 1; }
    if (!EXPECT_LEAK) {
        for (auto &v : view) if (!v->freed) { std::printf("FAIL: view descriptor still referenced\n"); fails = fails + 1; break; }
        if (native_adreno_metal_diag.forced_unmaps != 3) { std::printf("FAIL: forced_unmaps %u, expected 3\n", unsigned(native_adreno_metal_diag.forced_unmaps)); fails = fails + 1; }
    }
    /* Leaked maps of the pinned loop: freed here so the sanitizer run stays about this test's own memory. */
    for (auto &v : view) {
        while (!v->mappings_.empty()) { IOMemoryMap *m = v->mappings_.front(); m->rc = 2; m->release(); }
        if (!v->freed) v->release();
        if (v->freed) delete v;
    }
    std::printf("%s\n", fails ? "FAIL" : "PASS");
    return fails ? 1 : 0;
}
