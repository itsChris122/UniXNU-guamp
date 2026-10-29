/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * Page-granular IOVA owner table for the WCN3990 SNOC DMA window.
 * Pure C++ (no IOKit), so the allocator is testable on the host.
 * Each IOVA page records the 1-based slot that owns it (0 = free);
 * allocation is next-fit, lookup of a bus address is O(1).
 * GuampVaIndex maps CPU pages of coherent memory back to their slot.
 */
#ifndef GUAMP_XNU_IOVA_H
#define GUAMP_XNU_IOVA_H
#include <stdint.h>

namespace guampwifi {
struct GuampIovaPages {
  uint16_t *owner=nullptr;   // pages entries, caller-provided zeroed storage
  uint32_t pages=0,cursor=0,used=0,peak=0;

  bool init(uint16_t *storage,uint32_t count) {
    if(!storage||!count)return false;
    owner=storage;pages=count;cursor=0;used=0;peak=0;
    for(uint32_t i=0;i<count;i++)owner[i]=0;
    return true;
  }
  // Finds `count` consecutive free pages, next-fit from the cursor with one
  // wrap; marks them with `tag` (slot index + 1).
  bool reserve(uint32_t count,uint16_t tag,uint32_t *first) {
    if(!owner||!count||!tag||!first||count>pages)return false;
    for(uint32_t pass=0;pass<2;pass++) {
      uint32_t start=pass?0:cursor,limit=pass?cursor+count:pages;
      if(limit>pages)limit=pages;
      uint32_t run=0;
      for(uint32_t p=start;p<limit;p++) {
        if(owner[p]){run=0;continue;}
        if(++run==count) {
          uint32_t at=p+1-count;
          for(uint32_t i=0;i<count;i++)owner[at+i]=tag;
          cursor=(p+1)%pages;used+=count;if(used>peak)peak=used;
          *first=at;return true;
        }
      }
    }
    return false;
  }
  void release(uint32_t first,uint32_t count,uint16_t tag) {
    if(!owner||first>=pages||count>pages-first)return;
    for(uint32_t i=0;i<count;i++)if(owner[first+i]==tag){owner[first+i]=0;used--;}
  }
  uint16_t at(uint32_t page) const { return owner&&page<pages?owner[page]:0; }
};
// CPU-page -> slot index for coherent (dma_alloc) memory, so dma_map of a
// pointer that already lies in a coherent allocation (every skb of the port:
// nw_net.c allocates packet data with dma_alloc) resolves in O(1) to that
// mapping instead of a bounce page and a second slot. Open addressing with
// linear probing and backward-shift deletion (no tombstones); key 0 = empty
// (kernel VA page 0 never holds DMA memory). insert() refuses beyond 3/4 load:
// the caller then simply does not index that allocation (bounce fallback).
struct GuampVaIndex {
  uint64_t *keys=nullptr;    // capacity entries, caller-provided storage
  uint16_t *tags=nullptr;    // capacity entries
  uint32_t capacity=0,count=0;

  bool init(uint64_t *keyStorage,uint16_t *tagStorage,uint32_t entries) {
    if(!keyStorage||!tagStorage||entries<4||(entries&(entries-1)))return false;
    keys=keyStorage;tags=tagStorage;capacity=entries;count=0;
    for(uint32_t i=0;i<entries;i++){keys[i]=0;tags[i]=0;}
    return true;
  }
  uint32_t home(uint64_t page) const {
    uint64_t h=page*0x9e3779b97f4a7c15ull;return uint32_t(h>>32)&(capacity-1);
  }
  bool insert(uint64_t page,uint16_t tag) {
    if(!keys||!page||!tag||count+1>capacity-capacity/4)return false;
    for(uint32_t i=home(page);;i=(i+1)&(capacity-1)) {
      if(keys[i]==page)return false;               // a page belongs to one allocation
      if(!keys[i]){keys[i]=page;tags[i]=tag;count++;return true;}
    }
  }
  uint16_t lookup(uint64_t page) const {
    if(!keys||!page)return 0;
    for(uint32_t i=home(page);keys[i];i=(i+1)&(capacity-1))if(keys[i]==page)return tags[i];
    return 0;
  }
  void erase(uint64_t page,uint16_t tag) {
    if(!keys||!page)return;
    uint32_t i=home(page);
    for(;keys[i]!=page;i=(i+1)&(capacity-1))if(!keys[i])return;
    if(tags[i]!=tag)return;
    // backward shift: move later entries of the cluster into the hole when
    // their home position does not lie cyclically in (hole, entry].
    for(uint32_t j=i;;) {
      j=(j+1)&(capacity-1);
      if(!keys[j])break;
      uint32_t h=home(keys[j]);
      bool stays=(i<=j)?(i<h&&h<=j):(i<h||h<=j);
      if(stays)continue;
      keys[i]=keys[j];tags[i]=tags[j];i=j;
    }
    keys[i]=0;tags[i]=0;count--;
  }
};
} // namespace guampwifi
#endif
