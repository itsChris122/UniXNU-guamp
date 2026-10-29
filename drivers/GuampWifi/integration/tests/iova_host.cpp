// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of the WCN3990 IOVA page table (guamp_xnu_iova.h): the load
// ath10k puts on it at HIF start (~4800 RX buffers + rings), fragmentation,
// wrap-around and exhaustion. Build: clang++ -std=c++17 -fsanitize=address,undefined
#include "guamp_xnu_iova.h"
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <map>
#include <vector>
using guampwifi::GuampIovaPages;
using guampwifi::GuampVaIndex;
static int failures=0;
#define CHECK(c) do{if(!(c)){std::printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c);failures++;}}while(0)

int main() {
  const uint32_t pages=65536; // 0xa0000000-0xb0000000 / 4 KiB
  std::vector<uint16_t> owner(pages,0xffff);
  GuampIovaPages t;
  CHECK(t.init(owner.data(),pages));
  CHECK(owner[0]==0&&owner[pages-1]==0);
  // ath10k WCN3990 at HIF start: 12 rings/RRI, 2750 CE RX + 2047 HTT RX buffers (1-2 pages each).
  struct M { uint32_t first,count; uint16_t tag; };
  std::vector<M> live; uint16_t tag=1;
  for(uint32_t i=0;i<4900;i++) {
    uint32_t count=(i%3==0)?2:1,first=0;
    CHECK(t.reserve(count,tag,&first));
    for(uint32_t k=0;k<count;k++)CHECK(owner[first+k]==tag);
    live.push_back({first,count,tag});tag=uint16_t(tag%8192+1);
  }
  CHECK(t.used==live.size()+live.size()/3+1||t.used>4900);
  // no overlaps: each page owned by exactly the tag that reserved it
  for(const M &m:live)for(uint32_t k=0;k<m.count;k++)CHECK(t.at(m.first+k)==m.tag);
  // RX churn: free every other mapping, allocate again (next-fit, wraps)
  for(int round=0;round<40;round++) {
    for(size_t i=round&1;i<live.size();i+=2){t.release(live[i].first,live[i].count,live[i].tag);live[i].count=0;}
    for(size_t i=round&1;i<live.size();i+=2){uint32_t c=(i%5==0)?3:1,f=0;CHECK(t.reserve(c,live[i].tag,&f));live[i]={f,c,live[i].tag};}
  }
  for(const M &m:live)for(uint32_t k=0;k<m.count;k++)CHECK(t.at(m.first+k)==m.tag);
  // release all -> fully free
  for(const M &m:live)t.release(m.first,m.count,m.tag);
  CHECK(t.used==0);
  // release with a wrong tag must not free someone else's page
  uint32_t a=0;CHECK(t.reserve(4,7,&a));t.release(a,4,9);CHECK(t.used==4&&t.at(a)==7);t.release(a,4,7);CHECK(t.used==0);
  // exhaustion and a large contiguous request after fragmentation
  uint32_t f=0;CHECK(t.reserve(pages,1,&f)&&f==0);CHECK(!t.reserve(1,2,&f));t.release(0,pages,1);
  for(uint32_t p=0;p<pages;p++)CHECK(t.reserve(1,3,&f)&&f==p);      // next-fit fills in order
  for(uint32_t p=0;p<pages;p+=2)t.release(p,1,3);                    // isolated one-page holes
  CHECK(!t.reserve(2,4,&f)&&t.reserve(1,4,&f)&&t.at(f)==4);
  t.release(f,1,4);
  for(uint32_t p=0;p<pages;p++)if(t.at(p)==3)t.release(p,1,3);
  CHECK(t.used==0&&t.reserve(256,5,&f));                             // 1 MiB span still fits
  CHECK(!t.reserve(0,1,&f)&&!t.reserve(1,0,&f)&&!t.reserve(pages+1,1,&f));
  // CPU-page index of coherent allocations (zero-copy dma_map of skb data):
  // ~4850 one-page skbs + churn against a reference map, then empty.
  {
    const uint32_t cap=32768;
    std::vector<uint64_t> keys(cap,~0ull);std::vector<uint16_t> tags(cap,0xffff);
    GuampVaIndex x;CHECK(!x.init(keys.data(),tags.data(),3000));CHECK(x.init(keys.data(),tags.data(),cap));
    std::map<uint64_t,uint16_t> ref;
    uint64_t seed=88172645463325252ull;
    auto rnd=[&](){seed^=seed<<13;seed^=seed>>7;seed^=seed<<17;return seed;};
    // kernel VA pages, clustered like zone/kmem allocations so probe chains collide
    auto page=[&](){return 0xfffffff000000ull+(rnd()%60000)*((rnd()&1)?1:64);};
    for(int step=0;step<400000;step++) {
      if(ref.size()<4850||(rnd()&1)) {
        uint64_t p=page();uint16_t tg=uint16_t(rnd()%16384+1);
        bool ok=x.insert(p,tg);CHECK(ok==!ref.count(p));if(ok)ref[p]=tg;
      } else {
        auto it=ref.begin();std::advance(it,rnd()%ref.size());
        x.erase(it->first,uint16_t(it->second+1));CHECK(x.lookup(it->first)==it->second); // wrong tag: kept
        x.erase(it->first,it->second);CHECK(!x.lookup(it->first));ref.erase(it);
      }
      if(!(step%20000))for(auto &e:ref)CHECK(x.lookup(e.first)==e.second);
    }
    CHECK(x.count==ref.size());
    for(auto &e:ref)CHECK(x.lookup(e.first)==e.second);
    for(auto &e:ref)x.erase(e.first,e.second);
    CHECK(x.count==0);for(uint32_t i=0;i<cap;i++)CHECK(!keys[i]);
    // 3/4 load limit, then refusal (caller bounces instead)
    uint32_t n=0;while(x.insert(0x1000+n,1))n++;CHECK(n==cap-cap/4);
    CHECK(!x.insert(0,1)&&!x.insert(5,0)&&!x.lookup(0));
  }
  std::printf(failures?"iova_host: %d FAILURES\n":"PASS: iova_host (%d failures, peak %u pages)\n",failures,t.peak);
  return failures?1:0;
}
