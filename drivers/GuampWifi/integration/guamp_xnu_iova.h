/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * Page-granular IOVA owner table for the WCN3990 SNOC DMA window.
 * Pure C++ (no IOKit), so the allocator is testable on the host.
 * Each IOVA page records the 1-based slot that owns it (0 = free);
 * allocation is next-fit, lookup of a bus address is O(1).
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
} // namespace guampwifi
#endif
