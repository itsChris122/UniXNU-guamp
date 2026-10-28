// SPDX-License-Identifier: BSD-2-Clause-Patent
// Read-only QRTR TFTP endpoint for the modem's WLAN protection domain.
// Wire behavior follows linux-msm/tqftpserv; only exact guamp firmware paths
// are exposed. One transfer at a time; WRQ and arbitrary filesystem paths fail.
#ifndef GUAMP_QFTP_INC
#define GUAMP_QFTP_INC
namespace guampwifi {
struct QftpReadOnly {
  // QMI/QRTR packs version in bits7:0, instance in bits31:8: v1/i0 =>1.
  static constexpr uint32_t Service=4096,ServiceInstance=1,Port=0x4100;
  static constexpr uint32_t MaxBlock=4096,MaxFile=64u*1024u*1024u;
  struct Peer { uint32_t node,port; };
  void *ctx=nullptr;
  int (*send)(void *,uint32_t,uint32_t,uint32_t,const uint8_t *,uint32_t)=nullptr;
  int (*fileSize)(void *,const char *,uint32_t *)=nullptr;
  int (*fileRead)(void *,const char *,uint32_t,uint8_t *,uint32_t)=nullptr;
  char path[48]={};
  uint8_t pending[MaxBlock+4]={};uint32_t pendingBytes=0,retries=0;
  uint32_t size=0,offset=0,endOffset=0,limit=0,blockSize=512,block=0;
  uint64_t lastActivity=0;
  Peer peer={}; bool active=false,waitAck=false,finalSent=false;

  int start(void *opaque,int (*tx)(void *,uint32_t,uint32_t,uint32_t,const uint8_t *,uint32_t),
            int (*stat)(void *,const char *,uint32_t *),
            int (*read)(void *,const char *,uint32_t,uint8_t *,uint32_t)) {
    if(!tx||!stat||!read) return Invalid;
    ctx=opaque;send=tx;fileSize=stat;fileRead=read;reset();return Ok;
  }
  void stop() { reset();ctx=nullptr;send=nullptr;fileSize=nullptr;fileRead=nullptr; }
  int receive(uint32_t node,uint32_t port,const uint8_t *p,uint32_t n,uint64_t now=0) {
    if(!send||!p||node==0xffffffffu||!port||n<2||n>MaxBlock+1024) return Invalid;
    uint16_t op=be16(p);
    if(op==1) return request(node,port,p,n,now);
    if(op==2) return error(node,port,2,"read only"); // WRQ is never accepted
    if(!active||node!=peer.node||port!=peer.port||n<4) return NotFound;
    if(op==5) { reset();return Ok; }
    if(op!=4) return error(node,port,4,"bad operation");
    uint32_t ack=be16(p+2);
    if(!waitAck||ack!=block) return Protocol;
    waitAck=false;lastActivity=now;pendingBytes=retries=0;
    if(finalSent) { reset();return Ok; }
    return dataBlock();
  }
  int step(uint64_t now) {
    if(!active||!waitAck||now<lastActivity||now-lastActivity<1000)return Ok;
    if(retries>=5) {uint32_t node=peer.node,port=peer.port;reset();return error(node,port,0,"transfer timeout");}
    int e=emit(peer.node,peer.port,pending,pendingBytes);
    lastActivity=now;if(!e)retries++;return e==Again||e==NotReady?Ok:e;
  }
private:
  static uint16_t be16(const uint8_t *p) { return uint16_t(p[0])<<8|p[1]; }
  static void putBe16(uint8_t *p,uint16_t v) { p[0]=uint8_t(v>>8);p[1]=uint8_t(v); }
  static bool same(const char *a,const char *b) {
    while(*a&&*b) { char x=*a++,y=*b++;if(x>='A'&&x<='Z')x=char(x+32);if(y>='A'&&y<='Z')y=char(y+32);if(x!=y)return false; }
    return *a==*b;
  }
  static bool number(const char *p,uint32_t *v) {
    if(!p||!*p)return false;uint32_t x=0;
    for(;*p;p++) { if(*p<'0'||*p>'9'||x>(0xffffffffu-uint32_t(*p-'0'))/10)return false;x=x*10+uint32_t(*p-'0'); }
    *v=x;return true;
  }
  void reset() { active=waitAck=finalSent=false;size=offset=endOffset=limit=block=retries=pendingBytes=0;blockSize=512;lastActivity=0;peer={};memset(path,0,sizeof(path));memset(pending,0,sizeof(pending)); }
  int emit(uint32_t node,uint32_t port,const uint8_t *p,uint32_t n) { return send(ctx,Port,node,port,p,n); }
  int error(uint32_t node,uint32_t port,uint16_t code,const char *message) {
    uint8_t out[40]={0};putBe16(out,5);putBe16(out+2,code);uint32_t n=0;
    while(message[n]&&n<sizeof(out)-5){out[4+n]=uint8_t(message[n]);n++;}out[4+n]=0;
    return emit(node,port,out,5+n);
  }
  int request(uint32_t node,uint32_t port,const uint8_t *p,uint32_t n,uint64_t now) {
    reset();
    uint32_t at=2;const char *name=nullptr,*mode=nullptr;uint32_t nameLen=0,modeLen=0;
    if(!field(p,n,&at,&name,&nameLen)||!field(p,n,&at,&mode,&modeLen)||!nameLen||!modeLen||modeLen>=8)
      return error(node,port,4,"malformed request");
    char modeBuf[8]={};memcpy(modeBuf,mode,modeLen);
    if(!same(modeBuf,"octet")) return error(node,port,4,"octet only");
    static const char prefix[]="/readonly/firmware/image/";
    if(nameLen<=sizeof(prefix)-1||nameLen>=96||memcmp(name,prefix,sizeof(prefix)-1))
      return error(node,port,2,"access denied");
    const char *base=name+sizeof(prefix)-1;uint32_t baseLen=nameLen-(sizeof(prefix)-1);
    const char *allowedA="wlanmdsp.mbn",*allowedB="bdwlan_guamp_ipa.bin";
    const char *selected=nullptr;
    if(baseLen==strlen(allowedA)&&!memcmp(base,allowedA,baseLen))selected="/image/wlanmdsp.mbn";
    if(baseLen==strlen(allowedB)&&!memcmp(base,allowedB,baseLen))selected="/image/bdwlan_guamp_ipa.bin";
    if(!selected)return error(node,port,2,"file not found");
    memcpy(path,selected,strlen(selected)+1);
    uint32_t fileBytes=0;int e=fileSize(ctx,path,&fileBytes);
    if(e||!fileBytes||fileBytes>MaxFile)return error(node,port,1,"file unavailable");
    size=fileBytes;limit=size;blockSize=512;offset=0;
    bool wantOptions=false,wantTsize=false,wantSeek=false,wantRsize=false,wantBlock=false;
    uint32_t reqRsize=0,seek=0,requestedBlock=512;
    while(at<n) {
      const char *key,*value;uint32_t kl,vl;
      if(!field(p,n,&at,&key,&kl)||!field(p,n,&at,&value,&vl)||!kl||!vl)return error(node,port,8,"bad option");
      char k[20]={},v[24]={};if(kl>=sizeof(k)||vl>=sizeof(v))continue;
      memcpy(k,key,kl);memcpy(v,value,vl);
      uint32_t x=0;
      if(same(k,"tsize")) { if(wantTsize||!number(v,&x))return error(node,port,8,"bad tsize");wantTsize=wantOptions=true; }
      else if(same(k,"seek")) { if(wantSeek||!number(v,&seek))return error(node,port,8,"bad seek");wantSeek=wantOptions=true; }
      else if(same(k,"rsize")) { if(wantRsize||!number(v,&reqRsize)||!reqRsize||reqRsize>MaxFile)return error(node,port,8,"bad rsize");wantRsize=wantOptions=true; }
      else if(same(k,"blksize")) { if(wantBlock||!number(v,&requestedBlock)||requestedBlock<512||requestedBlock>MaxBlock)return error(node,port,8,"bad blksize");wantBlock=wantOptions=true; }
    }
    if(seek>size)return error(node,port,1,"seek past end");
    offset=seek;limit=size-seek;if(wantRsize&&reqRsize<limit)limit=reqRsize;
    endOffset=offset+limit;
    if(wantBlock)blockSize=requestedBlock;
    peer={node,port};active=true;block=0;lastActivity=now;
    if(wantOptions) {
      uint8_t out[128]={0};uint32_t z=2;putBe16(out,6);
      if(wantBlock&&!option(out,sizeof(out),&z,"blksize",blockSize))return failRange(node,port);
      if(wantTsize&&!option(out,sizeof(out),&z,"tsize",size))return failRange(node,port);
      if(wantRsize&&!option(out,sizeof(out),&z,"rsize",limit))return failRange(node,port);
      if(wantSeek&&!option(out,sizeof(out),&z,"seek",seek))return failRange(node,port);
      waitAck=true;memcpy(pending,out,z);pendingBytes=z;return emit(node,port,pending,pendingBytes);
    }
    return dataBlock();
  }
  static bool field(const uint8_t *p,uint32_t n,uint32_t *at,const char **out,uint32_t *len) {
    if(*at>=n)return false;uint32_t start=*at;while(*at<n&&p[*at])++*at;if(*at==n)return false;
    *out=reinterpret_cast<const char *>(p+start);*len=*at-start;++*at;return true;
  }
  static bool option(uint8_t *p,uint32_t cap,uint32_t *at,const char *name,uint32_t value) {
    char numberBuf[16];uint32_t v=value,digits=0;do{numberBuf[digits++]=char('0'+v%10);v/=10;}while(v&&digits<sizeof(numberBuf));
    uint32_t nl=strlen(name);if(*at>cap||nl+digits+2>cap-*at)return false;
    memcpy(p+*at,name,nl);*at+=nl+1;while(digits)p[(*at)++]=uint8_t(numberBuf[--digits]);p[(*at)++]=0;return true;
  }
  int failRange(uint32_t node,uint32_t port) { reset();return error(node,port,8,"options too large"); }
  int dataBlock() {
    uint8_t out[MaxBlock+4]={};
    uint32_t n=offset<endOffset?endOffset-offset:0;if(n>blockSize)n=blockSize;
    if(block>=0xffffu)return error(peer.node,peer.port,0,"block overflow");
    if(n) {
      int e=fileRead(ctx,path,offset,out+4,n);
      if(e) { uint32_t node=peer.node,port=peer.port;reset();return error(node,port,0,"read failed"); }
    }
    block++;putBe16(out,3);putBe16(out+2,uint16_t(block));offset+=n;
    if(n<blockSize||offset>=endOffset)finalSent=true;
    waitAck=true;memcpy(pending,out,n+4);pendingBytes=n+4;retries=0;
    return emit(peer.node,peer.port,pending,pendingBytes);
  }
};
} // namespace guampwifi
#endif
