// SPDX-License-Identifier: BSD-2-Clause-Patent
// Minimal RMTFS server (QMI service 14, version 1, instance 0) for the modem EFS partitions, published on QRTR.
// Wire format and behaviour follow linux-msm/rmtfs: qmi_rmtfs.qmi (messages 1..5, result TLV 2, errors
// INTERNAL=1 / MALFORMED=2), rmtfs.c (one handle per node+partition, caller id = handle slot, iovec sectors of 512
// bytes, any non-zero direction = write, is_force_sync only informative), storage.c (paths /boot/modem_fs1|fs2|fsc|fsg
// -> modemst1|modemst2|fsc|fsg, reads past the end are zero-filled) and sharedmem.c (iovec phys_offset and the
// alloc_buff answer are ABSOLUTE physical addresses inside the one shared region; alloc_buff only checks the size).
// The shared region itself is the downstream msm_sharedmem "rmtfs" client (qcom,rmtfs_sharedmem): its owner maps it,
// assigns it and hands the view here. Writes: an in-RAM overlay per 64 KiB chunk (as rmtfs's read-only shadow mode,
// initialised from eMMC, capped at 16 MiB = STORAGE_MAX_SIZE), unless the owner supplies an eMMC write callback;
// even then only modemst1/modemst2 are written to eMMC (fsc/fsg stay in RAM). No arbitrary path is ever opened.
#ifndef GUAMP_RMTFS_INC
#define GUAMP_RMTFS_INC
#include "GuampWifiWire.inc"
#include "GuampQmi.inc"
namespace guampwifi {
struct Rmtfs {
  static constexpr uint32_t Service=14,ServiceVersion=1,ServiceInstanceId=0;
  static constexpr uint32_t ServiceInstance=ServiceVersion|(ServiceInstanceId<<8),Port=0x4300;
  enum Msg : uint16_t { Open=1,Close=2,RwIovec=3,AllocBuff=4,GetDevError=5,ForceSyncInd=6 };
  enum : uint16_t { ResultSuccess=0,ResultFailure=1,ErrNone=0,ErrInternal=1,ErrMalformed=2 };
  static constexpr uint32_t Sector=512,PartitionCount=4,Callers=10,MaxIovec=255;
  static constexpr uint32_t ChunkBytes=64u*1024u,ShadowLimit=16u*1024u*1024u,PendingSlots=8,MaxReply=32;
  static constexpr uint32_t DetailedRequests=48;   // per-request log lines, then one summary every 64 requests
  struct Storage {
    void *ctx;
    Memory shared;           // the whole allocation, guard pages included; aligned 32-bit accesses only
    uint32_t sharedOffset;   // first byte of the rmtfs region inside `shared` (after the 4 KiB guard)
    uint64_t physical;       // physical address of that byte: what alloc_buff returns and iovec addresses use
    uint32_t bytes;          // region size (DT reg size, 0x200000)
    Allocator allocator;
    // Partition `index` (0 modemst1, 1 modemst2, 2 fsc, 3 fsg): resolve label, then sector-aligned transfers.
    int (*partition)(void *,uint32_t index,const char *label,uint64_t *bytes);
    int (*read)(void *,uint32_t index,uint64_t offset,uint8_t *out,uint32_t n);
    int (*write)(void *,uint32_t index,uint64_t offset,const uint8_t *in,uint32_t n);   // nullptr: RAM overlay only
    void (*trace)(void *,const char *what,const char *name,int e,uint64_t a,uint64_t b,uint64_t c);
  };
  struct Part {
    const char *path,*label; uint64_t bytes; bool present,emmcWrites;
    uint8_t **chunks; uint32_t chunkCount; uint64_t readBytes,writtenBytes;
  };
  struct Caller { bool used; uint32_t node,part; };
  struct Reply { bool used; uint32_t node,port,bytes; uint8_t data[MaxReply]; };

  Storage io={};
  Part parts[PartitionCount]={
    {"/boot/modem_fs1","modemst1",0,false,false,nullptr,0,0,0},
    {"/boot/modem_fs2","modemst2",0,false,false,nullptr,0,0,0},
    {"/boot/modem_fsc","fsc",0,false,false,nullptr,0,0,0},
    {"/boot/modem_fsg","fsg",0,false,false,nullptr,0,0,0},
  };
  Caller callers[Callers]={};
  Reply pending[PendingSlots]={};
  void *txCtx=nullptr;
  int (*send)(void *,uint32_t,uint32_t,uint32_t,const uint8_t *,uint32_t)=nullptr;
  uint8_t *scratch=nullptr;
  uint32_t shadowBytes=0,requests=0,failures=0,dropped=0;
  bool ready=false;

  int start(const Storage &s,void *opaque,int (*tx)(void *,uint32_t,uint32_t,uint32_t,const uint8_t *,uint32_t)) {
    if(ready)return Busy;
    if(!tx||!s.partition||!s.read||!s.allocator.alloc||!s.allocator.free||!s.shared.read||!s.shared.write||
       !s.bytes||(s.bytes&(Sector-1))||(s.sharedOffset&3)||!s.shared.valid(s.sharedOffset,s.bytes)||!s.physical||
       s.physical>~uint64_t(0)-s.bytes)return Invalid;
    io=s;txCtx=opaque;send=tx;
    scratch=static_cast<uint8_t *>(io.allocator.alloc(io.allocator.ctx,ChunkBytes));
    if(!scratch)return NoMemory;
    memset(callers,0,sizeof(callers));memset(pending,0,sizeof(pending));
    shadowBytes=requests=failures=dropped=0;
    uint32_t found=0;
    for(uint32_t i=0;i<PartitionCount;i++) {
      Part &p=parts[i];p.bytes=0;p.present=false;p.chunks=nullptr;p.chunkCount=0;p.readBytes=p.writtenBytes=0;
      int e=io.partition(io.ctx,i,p.label,&p.bytes);
      p.present=!e&&p.bytes>=Sector&&!(p.bytes&(Sector-1))&&p.bytes<=0xffffffffull*Sector;
      p.emmcWrites=p.present&&io.write&&i<2;   // modemst1/modemst2 only
      trace(p.present?"partition: bytes/eMMC writes/index":"partition MISSING: bytes/eMMC writes/index",
            p.path,p.present?Ok:(e?e:Range),p.bytes,p.emmcWrites,i);
      if(p.present)found++;
    }
    ready=true;
    trace("server ready: service/instance/port",io.write?"modemst writes to eMMC":"writes kept in RAM",
          found?Ok:NotFound,Service,ServiceInstance,Port);
    return Ok;
  }
  void stop() {
    if(io.allocator.free) {
      for(auto &p:parts) {
        if(p.chunks) {
          for(uint32_t c=0;c<p.chunkCount;c++)if(p.chunks[c])io.allocator.free(io.allocator.ctx,p.chunks[c],ChunkBytes);
          io.allocator.free(io.allocator.ctx,p.chunks,p.chunkCount*uint32_t(sizeof(uint8_t *)));
        }
        p.chunks=nullptr;p.chunkCount=0;
      }
      if(scratch)io.allocator.free(io.allocator.ctx,scratch,ChunkBytes);
    }
    scratch=nullptr;shadowBytes=0;ready=false;send=nullptr;txCtx=nullptr;
    memset(callers,0,sizeof(callers));memset(pending,0,sizeof(pending));
  }
  // Retries replies the QRTR queue could not take; never fails the transport.
  int step() {
    if(!ready)return Ok;
    for(auto &r:pending)if(r.used) {
      int e=send(txCtx,Port,r.node,r.port,r.data,r.bytes);
      if(e==Again||e==NotReady)return Ok;
      if(e)trace("reply dropped: node/port/bytes",nullptr,e,r.node,r.port,r.bytes);
      r.used=false;
    }
    return Ok;
  }
  // One QMI request from node/port. Malformed or failed requests are answered with a QMI failure result
  // (rmtfs.c behaviour); the return value only reports what happened, the caller must not fail QRTR on it.
  int receive(uint32_t node,uint32_t port,const uint8_t *p,uint32_t n) {
    if(!ready||!p||node==0xffffffffu||!port)return Invalid;
    if(n<7||n!=7u+le16(p+5))return Protocol;
    const uint16_t txn=le16(p+1),msg=le16(p+3);
    if(p[0]!=0) { trace("non-request QMI packet ignored: type/msg/node",nullptr,Unsupported,p[0],msg,node);return Unsupported; }
    TlvView body={p+7,uint32_t(n-7)};
    if(body.validate()) { (void)reply(node,port,txn,msg,ResultFailure,ErrMalformed,nullptr,0);return Protocol; }
    requests++;
    switch(msg) {
      case Open: return open(node,port,txn,body);
      case Close: return close(node,port,txn,body);
      case RwIovec: return iovec(node,port,txn,body);
      case AllocBuff: return allocBuff(node,port,txn,body);
      case GetDevError: return devError(node,port,txn,body);
      default:
        // rmtfs.c logs unknown messages and does not answer them.
        trace("unknown message: id/node/port",nullptr,Unsupported,msg,node,port);return Unsupported;
    }
  }
  bool shadowed(uint32_t index) const { return index<PartitionCount&&parts[index].chunks; }

private:
  void trace(const char *what,const char *name,int e,uint64_t a,uint64_t b,uint64_t c) {
    if(io.trace)io.trace(io.ctx,what,name,e,a,b,c);
  }
  bool detailed() const { return requests<=DetailedRequests||!(requests&63); }
  static bool u32Tlv(const TlvView &b,uint8_t tag,uint32_t *v) {
    uint32_t n=0;const uint8_t *x=b.find(tag,&n);if(!x||n!=4)return false;*v=le32(x);return true;
  }
  static bool u8Tlv(const TlvView &b,uint8_t tag,uint8_t *v) {
    uint32_t n=0;const uint8_t *x=b.find(tag,&n);if(!x||n!=1)return false;*v=x[0];return true;
  }
  // Response: type 2, same transaction and message id, result TLV 2 then optional TLVs.
  int reply(uint32_t node,uint32_t port,uint16_t txn,uint16_t msg,uint16_t result,uint16_t error,
            const uint8_t *extra,uint32_t extraBytes) {
    uint8_t packet[MaxReply]={};
    packet[0]=2;put16(packet+1,txn);put16(packet+3,msg);
    TlvWriter w={packet+7,MaxReply-7};
    uint8_t r[4];put16(r,result);put16(r+2,error);
    if(!w.add(2,r,4))return Range;
    if(extraBytes&&(extraBytes>MaxReply-7-w.size||!extra))return Range;
    if(extraBytes){memcpy(packet+7+w.size,extra,extraBytes);w.size+=extraBytes;}
    put16(packet+5,uint16_t(w.size));
    const uint32_t bytes=7+w.size;
    if(result)failures++;
    int e=send(txCtx,Port,node,port,packet,bytes);
    if(e!=Again&&e!=NotReady)return e;
    for(auto &q:pending)if(!q.used) { q.used=true;q.node=node;q.port=port;q.bytes=bytes;memcpy(q.data,packet,bytes);return Ok; }
    dropped++;trace("reply queue full: node/port/msg",nullptr,NoMemory,node,port,msg);
    return NoMemory;
  }
  static uint32_t tlvU32(uint8_t *out,uint8_t tag,uint32_t v) { out[0]=tag;put16(out+1,4);put32(out+3,v);return 7; }
  static uint32_t tlvU64(uint8_t *out,uint8_t tag,uint64_t v) { out[0]=tag;put16(out+1,8);put64(out+3,v);return 11; }
  static uint32_t tlvU8(uint8_t *out,uint8_t tag,uint8_t v) { out[0]=tag;put16(out+1,1);out[3]=v;return 4; }
  Caller *caller(uint32_t node,uint32_t id) {
    return id<Callers&&callers[id].used&&callers[id].node==node?&callers[id]:nullptr;
  }
  int open(uint32_t node,uint32_t port,uint16_t txn,const TlvView &b) {
    uint32_t n=0;const uint8_t *s=b.find(1,&n);
    while(s&&n&&!s[n-1])n--;                       // a trailing NUL is tolerated
    char path[64]={};
    if(!s||!n||n>=sizeof(path)) {
      trace("open MALFORMED: bytes/node/port",nullptr,Protocol,n,node,port);
      return reply(node,port,txn,Open,ResultFailure,ErrMalformed,nullptr,0);
    }
    memcpy(path,s,n);
    uint32_t index=PartitionCount;
    for(uint32_t i=0;i<PartitionCount;i++)if(n==strlen(parts[i].path)&&!memcmp(path,parts[i].path,n))index=i;
    if(index==PartitionCount||!parts[index].present) {
      trace(index==PartitionCount?"open REFUSED (unknown path): node/port/-":"open REFUSED (partition missing): node/port/-",
            path,NotFound,node,port,0);
      return reply(node,port,txn,Open,ResultFailure,ErrInternal,nullptr,0);
    }
    uint32_t id=Callers;
    for(uint32_t i=0;i<Callers&&id==Callers;i++)if(callers[i].used&&callers[i].node==node&&callers[i].part==index)id=i;
    const bool reused=id!=Callers;
    for(uint32_t i=0;i<Callers&&id==Callers;i++)if(!callers[i].used)id=i;
    if(id==Callers) {
      trace("open REFUSED (10 handles in use): node/port/-",path,Busy,node,port,0);
      return reply(node,port,txn,Open,ResultFailure,ErrInternal,nullptr,0);
    }
    callers[id]={true,node,index};
    trace(reused?"open (existing handle) -> caller/node/port":"open -> caller/node/port",path,Ok,id,node,port);
    uint8_t extra[7];return reply(node,port,txn,Open,ResultSuccess,ErrNone,extra,tlvU32(extra,0x10,id));
  }
  int close(uint32_t node,uint32_t port,uint16_t txn,const TlvView &b) {
    uint32_t id=0;
    if(!u32Tlv(b,1,&id))return reply(node,port,txn,Close,ResultFailure,ErrMalformed,nullptr,0);
    Caller *c=caller(node,id);
    if(!c) { trace("close of unknown caller: caller/node/port",nullptr,NotFound,id,node,port);
      return reply(node,port,txn,Close,ResultFailure,ErrInternal,nullptr,0); }
    // rmtfs's read-only mode drops its shadow on close; here the overlay belongs to the partition and lives for
    // the boot, so a modem that closes and reopens modemst1/2 still reads back what it wrote.
    const Part &p=parts[c->part];
    trace("close: caller/read bytes/written bytes",p.path,Ok,id,p.readBytes,p.writtenBytes);
    c->used=false;
    return reply(node,port,txn,Close,ResultSuccess,ErrNone,nullptr,0);
  }
  int allocBuff(uint32_t node,uint32_t port,uint16_t txn,const TlvView &b) {
    uint32_t id=0,size=0;
    if(!u32Tlv(b,1,&id)||!u32Tlv(b,2,&size))return reply(node,port,txn,AllocBuff,ResultFailure,ErrMalformed,nullptr,0);
    if(!caller(node,id)) { trace("alloc_buff for unknown caller: caller/size/node",nullptr,NotFound,id,size,node);
      return reply(node,port,txn,AllocBuff,ResultFailure,ErrInternal,nullptr,0); }
    if(size>io.bytes) {
      trace("alloc_buff TOO LARGE: caller/size/region",nullptr,Range,id,size,io.bytes);
      return reply(node,port,txn,AllocBuff,ResultFailure,ErrInternal,nullptr,0);
    }
    trace("alloc_buff -> caller/size/address",nullptr,Ok,id,size,io.physical);
    uint8_t extra[11];return reply(node,port,txn,AllocBuff,ResultSuccess,ErrNone,extra,tlvU64(extra,0x10,io.physical));
  }
  int devError(uint32_t node,uint32_t port,uint16_t txn,const TlvView &b) {
    uint32_t id=0;
    if(!u32Tlv(b,1,&id))return reply(node,port,txn,GetDevError,ResultFailure,ErrMalformed,nullptr,0);
    // rmtfs.c inverts its storage_get() test here (answers INTERNAL for a valid caller); the intended answer is
    // success with status 0 (no device error), which is what the modem expects from a healthy server.
    if(!caller(node,id)) { trace("get_dev_error for unknown caller: caller/node/port",nullptr,NotFound,id,node,port);
      return reply(node,port,txn,GetDevError,ResultFailure,ErrInternal,nullptr,0); }
    trace("get_dev_error -> caller/status/-",parts[callers[id].part].path,Ok,id,0,0);
    uint8_t extra[4];return reply(node,port,txn,GetDevError,ResultSuccess,ErrNone,extra,tlvU8(extra,0x10,0));
  }
  int iovec(uint32_t node,uint32_t port,uint16_t txn,const TlvView &b) {
    uint32_t id=0,n=0;uint8_t direction=0,force=0;
    const uint8_t *list=b.find(3,&n);
    if(!u32Tlv(b,1,&id)||!u8Tlv(b,2,&direction)||!u8Tlv(b,4,&force)||!list||!n||n!=1u+12u*list[0]) {
      trace("rw_iovec MALFORMED: node/port/array bytes",nullptr,Protocol,node,port,n);
      return reply(node,port,txn,RwIovec,ResultFailure,ErrMalformed,nullptr,0);
    }
    Caller *c=caller(node,id);
    if(!c) { trace("rw_iovec for unknown caller: caller/node/port",nullptr,NotFound,id,node,port);
      return reply(node,port,txn,RwIovec,ResultFailure,ErrInternal,nullptr,0); }
    const bool write=direction!=0;
    const uint32_t count=list[0];
    uint64_t sectors=0;int e=Ok;uint32_t bad=0;
    for(uint32_t i=0;i<count&&!e;i++) {
      const uint8_t *x=list+1+12*i;
      const uint64_t sector=le32(x),address=le32(x+4),num=le32(x+8);
      e=transfer(c->part,write,sector*Sector,address,num*Sector);
      if(e)bad=i;else sectors+=num;
    }
    Part &p=parts[c->part];
    if(e) trace(write?"rw_iovec WRITE FAILED: caller/entry/entries":"rw_iovec READ FAILED: caller/entry/entries",p.path,e,id,bad,count);
    else if(detailed()||force)
      trace(write?(force?"rw_iovec write (force sync): caller/entries/sectors":"rw_iovec write: caller/entries/sectors")
                 :"rw_iovec read: caller/entries/sectors",p.path,Ok,id,count,sectors);
    if(!detailed()&&!e&&!force&&!(requests&63))
      trace("totals: requests/failures/overlay bytes",nullptr,Ok,requests,failures,shadowBytes);
    return reply(node,port,txn,RwIovec,e?ResultFailure:ResultSuccess,e?ErrInternal:ErrNone,nullptr,0);
  }
  // One iovec entry: `bytes` at partition offset `at` <-> physical `address` in the shared region.
  int transfer(uint32_t index,bool write,uint64_t at,uint64_t address,uint64_t bytes) {
    Part &p=parts[index];
    if(address<io.physical||address-io.physical>io.bytes||bytes>io.bytes-(address-io.physical)) {
      trace("iovec address outside the shared region: address/bytes/region base",p.path,Range,address,bytes,io.physical);
      return Range;
    }
    uint32_t mem=io.sharedOffset+uint32_t(address-io.physical);
    while(bytes) {
      const uint64_t inChunk=ChunkBytes-(at%ChunkBytes);
      const uint32_t n=uint32_t(bytes<inChunk?bytes:inChunk);
      int e=Ok;
      if(write) {
        if(!io.shared.copyOut(mem,scratch,n))return Range;
        e=put(p,index,at,scratch,n);
      } else {
        e=get(p,index,at,scratch,n);
        if(!e&&!io.shared.copyIn(mem,scratch,n))e=Range;
      }
      if(e)return e;
      at+=n;mem+=n;bytes-=n;
    }
    io.shared.fence();
    return Ok;
  }
  // Reads from the overlay or the partition; bytes past the partition end read as zero (storage_pread).
  int get(Part &p,uint32_t index,uint64_t at,uint8_t *out,uint32_t n) {
    memset(out,0,n);
    if(at>=p.bytes)return Ok;
    const uint32_t valid=uint32_t(p.bytes-at<n?p.bytes-at:n);
    const uint32_t c=uint32_t(at/ChunkBytes);
    if(p.chunks&&p.chunks[c]) memcpy(out,p.chunks[c]+at%ChunkBytes,valid);
    else { int e=io.read(io.ctx,index,at,out,valid);if(e)return e; }
    p.readBytes+=n;return Ok;
  }
  int put(Part &p,uint32_t index,uint64_t at,const uint8_t *in,uint32_t n) {
    if(at>=p.bytes||n>p.bytes-at) {
      trace("write past the partition end: offset/bytes/partition bytes",p.path,Range,at,n,p.bytes);
      return Range;
    }
    if(p.emmcWrites) { int e=io.write(io.ctx,index,at,in,n);if(!e)p.writtenBytes+=n;return e; }
    if(!p.chunks) {
      const uint64_t count=(p.bytes+ChunkBytes-1)/ChunkBytes;
      if(count*sizeof(uint8_t *)>ShadowLimit)return NoMemory;
      p.chunks=static_cast<uint8_t **>(io.allocator.alloc(io.allocator.ctx,uint32_t(count*sizeof(uint8_t *))));
      if(!p.chunks)return NoMemory;
      memset(p.chunks,0,size_t(count*sizeof(uint8_t *)));p.chunkCount=uint32_t(count);
      trace("first write: RAM overlay created (eMMC untouched): partition bytes/chunks/-",p.path,Ok,p.bytes,count,0);
    }
    const uint32_t c=uint32_t(at/ChunkBytes);
    if(!p.chunks[c]) {
      if(shadowBytes+ChunkBytes>ShadowLimit) {
        trace("RAM overlay full: used/limit/offset",p.path,NoMemory,shadowBytes,ShadowLimit,at);return NoMemory;
      }
      uint8_t *chunk=static_cast<uint8_t *>(io.allocator.alloc(io.allocator.ctx,ChunkBytes));
      if(!chunk)return NoMemory;
      // Populate from eMMC first (storage_populate_shadow_buf), so partial writes keep the rest of the chunk.
      memset(chunk,0,ChunkBytes);
      const uint64_t base=uint64_t(c)*ChunkBytes;
      const uint32_t valid=uint32_t(p.bytes-base<ChunkBytes?p.bytes-base:ChunkBytes);
      int e=io.read(io.ctx,index,base,chunk,valid);
      if(e){io.allocator.free(io.allocator.ctx,chunk,ChunkBytes);return e;}
      p.chunks[c]=chunk;shadowBytes+=ChunkBytes;
    }
    memcpy(p.chunks[c]+at%ChunkBytes,in,n);p.writtenBytes+=n;
    return Ok;
  }
};
} // namespace guampwifi
#endif
