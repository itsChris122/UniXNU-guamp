// SPDX-License-Identifier: BSD-2-Clause-Patent
// QRTR TFTP server (service 4096 v1 i0) for the modem, as linux-msm/tqftpserv:
//  - requests (RRQ/WRQ) arrive on the service port; every transfer then runs on its own port (tqftpserv opens a
//    new QRTR socket per transfer and connect()s it to the client), so all replies leave from that session port;
//  - options blksize, timeoutms, tsize, rsize, wsize, seek; OACK when any option was sent, otherwise DATA 1 (RRQ)
//    or ACK 0 (WRQ); RRQ sends up to wsize blocks per ACK; WRQ ACKs when block % wsize == 0 or on the short block;
//  - translate.c paths: /readonly/firmware/image/X, /readonly/vendor/firmware_mnt/image/X and
//    /readonly/vendor/firmware/X -> /image/X of the modem firmware partition (read only, original signed files),
//    /readonly/firmware/modem_pr... -> /image/modem_pr...; /readwrite/X -> a RAM store for this boot (tqftpserv
//    writes them to /data/vendor/tmp/tqftpserv); ".." and empty components are refused (sanitize_path).
// Error codes and texts as tqftpserv. No eMMC write ever happens here.
#ifndef GUAMP_QFTP_INC
#define GUAMP_QFTP_INC
namespace guampwifi {
struct Qftp {
  // QMI/QRTR packs version in bits7:0, instance in bits31:8: v1/i0 =>1.
  static constexpr uint32_t Service=4096,ServiceInstance=1,Port=0x4100,Sessions=4,FirstSessionPort=0x4101;
  static constexpr uint32_t MaxBlock=4096,MaxFile=64u*1024u*1024u,MaxWindow=64;
  static constexpr uint32_t StoreFiles=8,StoreFileBytes=128u*1024u,PathBytes=96;
  enum Op : uint16_t { Rrq=1,Wrq=2,DataOp=3,AckOp=4,ErrorOp=5,OackOp=6 };
  struct StoreFile { char name[PathBytes]; uint8_t *data; uint32_t size; };
  struct Session {
    bool active,writer,readwrite,awaitOack,done;
    uint32_t node,port,localPort,size,seek,limit,blockSize,window,timeoutMs,lastBlock,acked,sent,retries,received,bytes;
    int file; uint64_t lastActivity,started; char path[PathBytes];
  };
  void *ctx=nullptr;
  int (*send)(void *,uint32_t,uint32_t,uint32_t,const uint8_t *,uint32_t)=nullptr;
  int (*fileSize)(void *,const char *,uint32_t *)=nullptr;
  int (*fileRead)(void *,const char *,uint32_t,uint8_t *,uint32_t)=nullptr;
  // Optional: RAM store for /readwrite (no allocator: WRQ answered "Access violation"), one-line diagnostics.
  Allocator allocator={};
  void *traceCtx=nullptr;
  void (*trace)(void *,const char *what,const char *name,int e,uint64_t a,uint64_t b,uint64_t c)=nullptr;
  Session sessions[Sessions]={};
  StoreFile store[StoreFiles]={};
  uint8_t packet[MaxBlock+4]={};
  uint32_t requests=0,refused=0,completed=0;

  int start(void *opaque,int (*tx)(void *,uint32_t,uint32_t,uint32_t,const uint8_t *,uint32_t),
            int (*stat)(void *,const char *,uint32_t *),
            int (*read)(void *,const char *,uint32_t,uint8_t *,uint32_t),const Allocator *rw=nullptr) {
    if(!tx||!stat||!read) return Invalid;
    stop();ctx=opaque;send=tx;fileSize=stat;fileRead=read;allocator=rw?*rw:Allocator{};
    requests=refused=completed=0;return Ok;
  }
  void stop() {
    for(auto &s:sessions) s={};
    for(auto &f:store) { if(f.data&&allocator.free) allocator.free(allocator.ctx,f.data,StoreFileBytes); f={}; }
    ctx=nullptr;send=nullptr;fileSize=nullptr;fileRead=nullptr;
  }
  static uint32_t sessionPort(uint32_t i) { return FirstSessionPort+i; }
  bool active() const { for(const auto &s:sessions) if(s.active) return true; return false; }
  // localPort: Port for new requests, a session port for the rest of a transfer.
  int receive(uint32_t localPort,uint32_t node,uint32_t port,const uint8_t *p,uint32_t n,uint64_t now=0) {
    if(!send||!p||node==0xffffffffu||!port||n<2||n>MaxBlock+1024) return Invalid;
    const uint16_t op=be16(p);
    if(localPort==Port) {
      if(op!=Rrq&&op!=Wrq) return error(Port,node,port,4,"Expected RRQ or WRQ opcode");
      return request(op==Wrq,node,port,p,n,now);
    }
    Session *s=nullptr;
    for(auto &x:sessions) if(x.active&&x.localPort==localPort) s=&x;
    if(!s||node!=s->node||port!=s->port) return NotFound;
    if(op==ErrorOp) {   // client abort, or code 9 "End of Transfer" after reading what it needed
      trace2(s->writer?"WRQ ended by the client: code/bytes/-":"RRQ ended by the client: code/bytes/-",s->path,Ok,n>=4?be16(p+2):0,s->bytes,0);
      close(*s);return Ok;
    }
    s->lastActivity=now;s->retries=0;
    if(s->writer) {
      if(op!=DataOp||n<4) { int e=error(s->localPort,node,port,4,"Expected DATA opcode");close(*s);return e; }
      return writeBlock(*s,p+4,n-4,be16(p+2),now);
    }
    if(op!=AckOp||n<4) { int e=error(s->localPort,node,port,4,"Expected ACK opcode");close(*s);return e; }
    return ack(*s,be16(p+2),now);
  }
  int step(uint64_t now) {
    if(!send) return Ok;
    for(auto &s:sessions) {
      if(!s.active||now<s.lastActivity||now-s.lastActivity<s.timeoutMs) continue;
      if(s.retries>=5) {
        trace2("transfer timeout: blocks acked/bytes/-",s.path,Timeout,s.acked,s.bytes,0);
        uint32_t lp=s.localPort,node=s.node,port=s.port;close(s);(void)error(lp,node,port,0,"Transfer timeout");continue;
      }
      s.retries++;s.lastActivity=now;
      int e=Ok;
      if(s.awaitOack) e=oack(s);
      else if(s.writer) e=ackBlock(s,s.received);
      else { s.sent=s.acked; e=window(s); }
      if(e&&e!=Again&&e!=NotReady) return e;
    }
    return Ok;
  }
  const StoreFile *stored(const char *name) const {
    for(const auto &f:store) if(f.data&&same(f.name,name)) return &f;
    return nullptr;
  }
private:
  static uint16_t be16(const uint8_t *p) { return uint16_t(p[0])<<8|p[1]; }
  static void putBe16(uint8_t *p,uint16_t v) { p[0]=uint8_t(v>>8);p[1]=uint8_t(v); }
  static char lower(char c) { return c>='A'&&c<='Z'?char(c+32):c; }
  static bool sameNoCase(const char *a,const char *b) { while(*a&&*b) if(lower(*a++)!=lower(*b++)) return false; return *a==*b; }
  static bool same(const char *a,const char *b) { while(*a&&*b) if(*a++!=*b++) return false; return *a==*b; }
  static bool prefix(const char *s,const char *p,const char **rest) {
    uint32_t i=0;while(p[i]){if(s[i]!=p[i])return false;i++;} *rest=s+i;return true;
  }
  static bool number(const char *p,uint32_t *v) {
    if(!p||!*p)return false;uint32_t x=0;
    for(;*p;p++) { if(*p<'0'||*p>'9'||x>(0xffffffffu-uint32_t(*p-'0'))/10)return false;x=x*10+uint32_t(*p-'0'); }
    *v=x;return true;
  }
  void trace2(const char *what,const char *name,int e,uint64_t a,uint64_t b,uint64_t c) { if(trace) trace(traceCtx,what,name,e,a,b,c); }
  void close(Session &s) { s={}; }
  int emit(uint32_t from,uint32_t node,uint32_t port,const uint8_t *p,uint32_t n) { return send(ctx,from,node,port,p,n); }
  int error(uint32_t from,uint32_t node,uint32_t port,uint16_t code,const char *message) {
    uint8_t out[64]={0};putBe16(out,ErrorOp);putBe16(out+2,code);uint32_t n=0;
    while(message[n]&&n<sizeof(out)-5){out[4+n]=uint8_t(message[n]);n++;}out[4+n]=0;
    return emit(from,node,port,out,5+n);
  }
  // sanitize_path: no "..", no empty component; then the translate.c prefixes.
  static bool translate(const char *name,char *out,bool *readwrite) {
    if(name[0]!='/')return false;
    for(const char *c=name;*c;) {
      if(*c=='/') { c++; if(*c=='/'||!*c)return false; if(c[0]=='.'&&c[1]=='.'&&(c[2]=='/'||!c[2]))return false; }
      else c++;
    }
    const char *rest=nullptr;const char *base="/image/";
    *readwrite=false;
    if(prefix(name,"/readonly/firmware/image/",&rest)) {}
    else if(prefix(name,"/readonly/firmware/modem_pr",&rest)) { rest=name+strlen("/readonly/firmware/"); }
    else if(prefix(name,"/readonly/vendor/firmware_mnt/image/",&rest)) {}
    else if(prefix(name,"/readonly/vendor/firmware/",&rest)) {}
    else if(prefix(name,"/readwrite/",&rest)) { base="/readwrite/";*readwrite=true; }
    else return false;
    if(!*rest||strlen(base)+strlen(rest)>=PathBytes)return false;
    memcpy(out,base,strlen(base));memcpy(out+strlen(base),rest,strlen(rest)+1);return true;
  }
  static bool field(const uint8_t *p,uint32_t n,uint32_t *at,const char **out,uint32_t *len) {
    if(*at>=n)return false;uint32_t start=*at;while(*at<n&&p[*at])++*at;if(*at==n)return false;
    *out=reinterpret_cast<const char *>(p+start);*len=*at-start;++*at;return true;
  }
  int fileOpen(bool write,const char *name,bool create,uint32_t *bytes) {
    for(int i=0;i<int(StoreFiles);i++) if(store[i].data&&same(store[i].name,name)) { *bytes=store[i].size;return i; }
    if(!write||!create||!allocator.alloc) return -1;
    for(int i=0;i<int(StoreFiles);i++) if(!store[i].data) {
      store[i].data=static_cast<uint8_t *>(allocator.alloc(allocator.ctx,StoreFileBytes));
      if(!store[i].data) return -1;
      memset(store[i].data,0,StoreFileBytes);memcpy(store[i].name,name,strlen(name)+1);store[i].size=0;*bytes=0;return i;
    }
    return -1;
  }
  int request(bool write,uint32_t node,uint32_t port,const uint8_t *p,uint32_t n,uint64_t now) {
    requests++;
    uint32_t at=2;const char *name=nullptr,*mode=nullptr;uint32_t nameLen=0,modeLen=0;
    if(!field(p,n,&at,&name,&nameLen)||!field(p,n,&at,&mode,&modeLen)||!nameLen||nameLen>=PathBytes||!modeLen||modeLen>=8) {
      refused++;trace2("request MALFORMED: node/port/bytes",nullptr,Protocol,node,port,n);
      return error(Port,node,port,4,"Malformed packet");
    }
    char path[PathBytes]={},modeBuf[8]={};memcpy(path,name,nameLen);memcpy(modeBuf,mode,modeLen);
    if(!sameNoCase(modeBuf,"octet")) { refused++;return error(Port,node,port,4,"Only octet mode supported"); }
    Session *s=nullptr;uint32_t index=0;
    for(uint32_t i=0;i<Sessions&&!s;i++) if(!sessions[i].active) { s=&sessions[i];index=i; }
    if(!s) {
      refused++;trace2(write?"WRQ REFUSED (all sessions busy): node/port/-":"RRQ REFUSED (all sessions busy): node/port/-",path,Busy,node,port,0);
      return error(Port,node,port,0,"Resources temporarily unavailable");
    }
    char file[PathBytes]={};bool readwrite=false;
    if(!translate(path,file,&readwrite)||(write&&!readwrite)) {
      refused++;trace2(write?"WRQ REFUSED (access violation): node/port/-":"RRQ REFUSED (access violation): node/port/-",path,Unsupported,node,port,0);
      return error(Port,node,port,2,"Access violation");
    }
    Session x={};x.node=node;x.port=port;x.localPort=sessionPort(index);x.writer=write;x.readwrite=readwrite;
    x.blockSize=512;x.window=1;x.timeoutMs=1000;x.file=-1;x.lastActivity=x.started=now;
    memcpy(x.path,path,sizeof(x.path));
    uint32_t bytes=0;
    if(readwrite) {
      x.file=fileOpen(write,file,true,&bytes);
      if(x.file<0) {
        refused++;trace2(write?"WRQ REFUSED (RAM store full): node/port/-":"RRQ: file not found: node/port/-",path,write?NoMemory:NotFound,node,port,0);
        return write?error(Port,node,port,2,"Access violation"):error(Port,node,port,1,"file not found");
      }
    } else {
      int e=fileSize(ctx,file,&bytes);
      if(e||bytes>MaxFile) { refused++;trace2("RRQ: file not found: node/port/-",path,e?e:Range,node,port,0);
        return error(Port,node,port,1,"file not found"); }
    }
    x.size=bytes;
    bool options=false,wantBlock=false,wantTimeout=false,wantTsize=false,wantRsize=false,wantWindow=false,wantSeek=false;
    uint32_t tsize=0,rsize=0;
    while(at<n) {
      const char *key,*value;uint32_t kl,vl;
      if(!field(p,n,&at,&key,&kl)||!field(p,n,&at,&value,&vl))break;
      char k[16]={},v[16]={};if(!kl||kl>=sizeof(k)||!vl||vl>=sizeof(v))continue;
      memcpy(k,key,kl);memcpy(v,value,vl);uint32_t val=0;
      if(!number(v,&val))return error(Port,node,port,8,"Option negotiation failed");
      if(sameNoCase(k,"blksize")) { if(val<8||val>65464)return error(Port,node,port,8,"Option negotiation failed");
        x.blockSize=val>MaxBlock?MaxBlock:val;wantBlock=options=true; }   // RFC 2348: the server may answer smaller
      else if(sameNoCase(k,"timeoutms")) { if(!val||val>255000)return error(Port,node,port,8,"Option negotiation failed");
        x.timeoutMs=val;wantTimeout=options=true; }
      else if(sameNoCase(k,"tsize")) { tsize=val;wantTsize=options=true; }
      else if(sameNoCase(k,"rsize")) { if(!val||val>100u*1024u*1024u)return error(Port,node,port,8,"Option negotiation failed");
        rsize=val;wantRsize=options=true; }
      else if(sameNoCase(k,"wsize")) { if(!val||val>65535)return error(Port,node,port,8,"Option negotiation failed");
        x.window=val>MaxWindow?MaxWindow:val;wantWindow=options=true; }
      else if(sameNoCase(k,"seek")) { x.seek=val;wantSeek=options=true; }
    }
    if(!write) {
      if(x.seek>x.size) { refused++;return error(Port,node,port,1,"file not found"); }
      x.limit=x.size-x.seek;if(wantRsize&&rsize<x.limit)x.limit=rsize;
      x.lastBlock=x.limit/x.blockSize+1;   // the last block is the short (possibly empty) one
    } else {
      x.limit=wantTsize?tsize:0;
      if(x.seek>StoreFileBytes) { refused++;return error(Port,node,port,3,"Disk full or write error"); }
    }
    x.active=true;x.awaitOack=options;*s=x;
    s->bytes=0;
    // tsize: the size for a read, the client's announced size echoed for a write (tqftpserv echoes the options).
    uint32_t echoTsize=write?tsize:s->size;
    trace2(write?"WRQ -> session port/blksize/wsize":"RRQ -> session port/blksize/wsize",path,Ok,s->localPort,s->blockSize,s->window);
    if(!write) trace2("RRQ file bytes/seek/limit",path,Ok,s->size,s->seek,s->limit);
    if(options) {
      uint8_t *o=packet;uint32_t z=2;putBe16(o,OackOp);
      if((wantBlock&&!option(o,&z,"blksize",s->blockSize))||(wantTimeout&&!option(o,&z,"timeoutms",s->timeoutMs))||
         (wantTsize&&!option(o,&z,"tsize",echoTsize))||(wantRsize&&!option(o,&z,"rsize",write?rsize:s->limit))||
         (wantWindow&&!option(o,&z,"wsize",s->window))||(wantSeek&&!option(o,&z,"seek",s->seek))) {
        close(*s);return error(Port,node,port,8,"Option negotiation failed");
      }
      oackBytes[s-sessions]=z;memcpy(oackPacket[s-sessions],o,z);
      return oack(*s);
    }
    if(write) return ackBlock(*s,0);
    return window(*s);
  }
  uint8_t oackPacket[Sessions][128]={};uint32_t oackBytes[Sessions]={};
  static bool option(uint8_t *p,uint32_t *at,const char *name,uint32_t value) {
    char digits[12];uint32_t v=value,d=0;do{digits[d++]=char('0'+v%10);v/=10;}while(v&&d<sizeof(digits));
    uint32_t nl=uint32_t(strlen(name));if(*at+nl+d+2>128)return false;
    memcpy(p+*at,name,nl);*at+=nl+1;p[*at-1]=0;while(d)p[(*at)++]=uint8_t(digits[--d]);p[(*at)++]=0;return true;
  }
  int oack(Session &s) { const uint32_t i=uint32_t(&s-sessions);return emit(s.localPort,s.node,s.port,oackPacket[i],oackBytes[i]); }
  int ackBlock(Session &s,uint32_t block) {
    uint8_t a[4];putBe16(a,AckOp);putBe16(a+2,uint16_t(block));return emit(s.localPort,s.node,s.port,a,4);
  }
  // 16-bit wire block -> 32-bit counter next to `base` (files above 65535 blocks wrap on the wire).
  static uint32_t unwrap(uint32_t base,uint16_t wire) { return base+uint16_t(wire-uint16_t(base)); }
  int dataBlock(Session &s,uint32_t block) {
    const uint64_t first=uint64_t(block-1)*s.blockSize;
    uint32_t n=first<s.limit?uint32_t(s.limit-first<s.blockSize?s.limit-first:s.blockSize):0;
    putBe16(packet,DataOp);putBe16(packet+2,uint16_t(block));
    if(n) {
      if(s.readwrite) memcpy(packet+4,store[s.file].data+s.seek+first,n);
      else {
        char file[PathBytes]={};bool rw=false;(void)translate(s.path,file,&rw);
        for(uint32_t done=0;done<n;) {
          uint32_t chunk=n-done>4096?4096:n-done;
          int e=fileRead(ctx,file,uint32_t(s.seek+first+done),packet+4+done,chunk);
          if(e) { trace2("RRQ read error: offset/bytes/-",s.path,e,s.seek+first+done,chunk,0);
            uint32_t lp=s.localPort,node=s.node,port=s.port;close(s);return error(lp,node,port,0,"Read error"); }
          done+=chunk;
        }
      }
    }
    return emit(s.localPort,s.node,s.port,packet,n+4);
  }
  // handle_reader: after ACK `acked`, blocks acked+1 .. acked+wsize (never past the last block).
  int window(Session &s) {
    while(s.sent<s.lastBlock&&s.sent<s.acked+s.window) {
      int e=dataBlock(s,s.sent+1);if(!s.active)return e;
      if(e==Again||e==NotReady) return Ok;   // resent by step() after timeoutms
      if(e) return e;
      s.sent++;
    }
    return Ok;
  }
  int ack(Session &s,uint16_t wire,uint64_t now) {
    if(s.awaitOack) {
      if(wire!=0) return Protocol;
      s.awaitOack=false;s.acked=s.sent=0;return window(s);
    }
    const uint32_t block=unwrap(s.acked,wire);
    if(block<s.acked||block>s.sent) return Protocol;   // stale or future ACK: ignored
    s.acked=block;
    s.bytes=uint32_t(uint64_t(block)*s.blockSize>s.limit?s.limit:uint64_t(block)*s.blockSize);
    if(block>=s.lastBlock) {
      completed++;trace2("RRQ done: bytes/blocks/ms",s.path,Ok,s.limit,block,now-s.started);close(s);return Ok;
    }
    return window(s);
  }
  // handle_writer: copy the block, ACK at the end of each window or on the short last block.
  int writeBlock(Session &s,const uint8_t *data,uint32_t n,uint16_t wire,uint64_t now) {
    if(s.awaitOack) s.awaitOack=false;   // the first DATA acknowledges our OACK
    const uint32_t block=unwrap(s.received,wire);
    if(block<=s.received) return ackBlock(s,s.received);   // retransmission: confirm what we hold
    if(block!=s.received+1) {
      uint32_t lp=s.localPort,node=s.node,port=s.port;close(s);return error(lp,node,port,4,"Block number out of sequence");
    }
    if(n>s.blockSize) { uint32_t lp=s.localPort,node=s.node,port=s.port;close(s);return error(lp,node,port,4,"Malformed packet"); }
    StoreFile &f=store[s.file];
    const uint64_t at=uint64_t(s.seek)+uint64_t(block-1)*s.blockSize;
    if(at+n>StoreFileBytes) {
      trace2("WRQ REFUSED (RAM file limit): offset/bytes/limit",s.path,NoMemory,at,n,StoreFileBytes);
      uint32_t lp=s.localPort,node=s.node,port=s.port;close(s);return error(lp,node,port,3,"Disk full or write error");
    }
    if(n) memcpy(f.data+at,data,n);
    if(at+n>f.size) f.size=uint32_t(at+n);
    s.received=block;s.bytes+=n;
    const bool last=n<s.blockSize;
    if(!last&&block%s.window) return Ok;
    int e=ackBlock(s,block);
    if(last) {
      // Short printable files (e.g. the modem's server check) are shown in the log.
      char preview[49]={};uint32_t k=0;bool text=true;
      for(;k<f.size&&k<48;k++){char c=char(f.data[k]);if(c<32||c>126){text=false;break;}preview[k]=c;}
      completed++;trace2("WRQ done (RAM only): bytes/file size/ms",s.path,Ok,s.bytes,f.size,now-s.started);
      if(text&&f.size) trace2("WRQ content (text)",preview,Ok,f.size,0,0);
      close(s);
    }
    return e;
  }
};
using QftpReadOnly=Qftp;   // historical name (read-only before /readwrite support)
} // namespace guampwifi
#endif
